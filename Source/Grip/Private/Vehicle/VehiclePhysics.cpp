/**
*
* Vehicle physics implementation.
*
* Original author: Rob Baker.
* Current maintainer: Rob Baker.
*
* Copyright Caged Element Inc, code provided for educational purposes only.
*
* Handle all of the physics-related activity of the vehicle. Most, if not all of
* this, will be executed via the SubstepPhysics function, and so via the physics
* sub-step, so we need to be very careful what we do here. All of the vehicle
* dynamics code can be found here.
*
***********************************************************************************/

#include "vehicle/vehiclephysics.h"
#include "vehicle/flippablevehicle.h"
#include "effects/drivingsurfacecharacteristics.h"
#include "pickups/shield.h"

#if WITH_PHYSX
#include "pxcontactmodifycallback.h"
#include "runtime/engine/private/physicsengine/physxsupport.h"
#endif // WITH_PHYSX

/**
* Do the regular physics update tick, for every sub-step.
*
* This is executed just prior to apply all forces and torques to this particular
* vehicle, though not necessarily before or after any other vehicles.
*
* Once all vehicles have been sub-stepped and forces / torques applied in this way
* the simulation itself is then stepped. Hence, any transforms within the physics
* system that are used in calculations here will be from the last physics sub-step.
*
* Consider this function here called in preparation for running this physics
* sub-step iteration.
*
* As the regular vehicle actor Tick is run PostPhysics you can do any cleanup work
* at the beginning of that Tick function, knowing that you'll be reading the most
* up-to-date information from the physics system.
***********************************************************************************/

void ABaseVehicle::SubstepPhysics(float deltaSeconds, FBodyInstance* bodyInstance)
{
	if (World == nullptr)
	{
		return;
	}

	// If the vehicle is idle-locked then clamp it by settings its location and orientation
	// and nullifying any velocity.

	if (VehicleMesh->UpdateIdleLock(true) == true)
	{
		VehicleMesh->SetPhysicsLocationAndQuaternionSubstep(VehicleMesh->GetIdleLocation(), VehicleMesh->GetIdleRotation());
		VehicleMesh->SetPhysicsLinearVelocitySubstep(FVector::ZeroVector);
		VehicleMesh->SetPhysicsAngularVelocityInRadiansSubstep(FVector::ZeroVector);
	}

	// Adjust the time passed to take into account custom time dilation for this actor.
	// This will always be 1 in this stripped version of the code, but it's important
	// that if you ever extend this to use CustomTimeDilation that we factor this in
	// right here.

	deltaSeconds *= CustomTimeDilation;
	deltaSeconds = FMath::Max(deltaSeconds, KINDA_SMALL_NUMBER);

	bool firstFrame = (Physics.Timing.TickCount <= 0);

	Physics.Timing.TickCount++;

	if (Physics.Timing.TickCount > 0)
	{
		Physics.Timing.TickSum += deltaSeconds;
	}

	// Grab a few things directly from the physics body and keep them in local variables,
	// sharing them around the update where appropriate.

	const FTransform& transform = VehicleMesh->GetPhysicsTransform();
	FQuat transformQuaternion = transform.GetRotation();
	FVector xdirection = transform.GetUnitAxis(EAxis::X);
	FVector ydirection = transform.GetUnitAxis(EAxis::Y);
	FVector zdirection = transform.GetUnitAxis(EAxis::Z);

	check(xdirection.ContainsNaN() == false);
	check(ydirection.ContainsNaN() == false);
	check(zdirection.ContainsNaN() == false);

	Physics.LastPhysicsTransform = Physics.PhysicsTransform;
	Physics.PhysicsTransform = transform;
	Physics.Direction = xdirection;

	// Get the world and local velocity in meters per second of the vehicle.

	FVector lastVelocity = Physics.VelocityData.Velocity;

	Physics.VelocityData.SetVelocities(VehicleMesh->GetPhysicsLinearVelocity(), VehicleMesh->GetPhysicsAngularVelocityInDegrees(), xdirection);

	// Calculate the acceleration vector of the vehicle in meters per second.

	Physics.VelocityData.AccelerationWorldSpace = (Physics.VelocityData.Velocity - lastVelocity) / deltaSeconds;
	Physics.VelocityData.AccelerationLocalSpace = transform.InverseTransformVector(Physics.VelocityData.AccelerationWorldSpace);
	Physics.DistanceTraveled += GetSpeedMPS() * deltaSeconds;
	Physics.AntigravitySideSlip = FMath::Max(0.0f, Physics.AntigravitySideSlip - (deltaSeconds * 0.333f));
	Physics.VelocityData.AngularVelocity = transform.InverseTransformVector(VehicleMesh->GetPhysicsAngularVelocityInDegrees());
	Physics.VehicleTBoned = FMath::Max(Physics.VehicleTBoned - deltaSeconds, 0.0f);
	Physics.SpringScaleTimer = FMath::Max(Physics.SpringScaleTimer - deltaSeconds, 0.0f);
	Physics.CurrentMass = Physics.StockMass;

#pragma region VehicleContactSensors

	// Update the springs and record how many wheels are in contact with surfaces.
	// This is the core processing of contact sensors and most the work required for
	// them resides in UpdateContactSensors.

	Wheels.NumWheelsInContact = UpdateContactSensors(deltaSeconds, transform, xdirection, ydirection, zdirection);
	Wheels.FrontAxlePosition = transform.TransformPosition(FVector(Wheels.FrontAxleOffset, 0.0f, 0.0f));
	Wheels.RearAxlePosition = transform.TransformPosition(FVector(Wheels.RearAxleOffset, 0.0f, 0.0f));

#pragma endregion VehicleContactSensors

	float brakePosition = 0.0f;
}

#pragma region VehicleContactSensors

/**
* Update the contact sensors.
***********************************************************************************/

int32 ABaseVehicle::UpdateContactSensors(float deltaSeconds, const FTransform& transform, const FVector& xdirection, const FVector& ydirection, const FVector& zdirection)
{
	static FName noSurface("None");

	Wheels.SurfaceName = noSurface;

	float physicsClock = Physics.Timing.TickSum;
	int32 numWheels = Wheels.Wheels.Num();
	int32 numUpContact = 0;
	int32 numDownContact = 0;
	bool bounceCompression = false;

	if (numWheels != 0)
	{
		// This is an optimization to halve the number of sweeps performed of the car was
		// completely on the ground last frame and still is again this frame.

		int32 halfTheWheels = numWheels >> 1;
		int32 numAxles = halfTheWheels;
		bool estimate =
			((/*We're in the air and have no wheels within 2m of the ground*/Physics.ContactData.Airborne == true && IsPracticallyGrounded(200.0f, true) == false) ||
				(/*We're grounded and have been for a moment*/Physics.ContactData.Grounded == true && Physics.ContactData.GroundedList.GetAbsMeanValue(physicsClock - 0.333f) > 1.0f - KINDA_SMALL_NUMBER));

		if (PlayGameMode == nullptr)
		{
			estimate = false;
		}

#if GRIP_CYCLE_SUSPENSION == GRIP_CYCLE_SUSPENSION_NONE
#define SHOULD_ESTIMATE false
#endif

#if GRIP_CYCLE_SUSPENSION == GRIP_CYCLE_SUSPENSION_BY_AXLE
		// Do this for axle per frame. This assumes two wheels per axle, added in axle order
		// in the WheelAssignments array.

#define SHOULD_ESTIMATE (((wheelIndex++ >> 1) % numAxles) != (Physics.Timing.TickCount % numAxles))
#endif

		if (Physics.ContactData.Grounded == true)
		{
			// If the vehicle is grounded then we can do less work, by ticking the contact sensors
			// in a specific way, the in-contact set first and the alternate set second - the
			// alternate set performing a very minimal Tick where possible.

			bool allInContact = true;
			int32 wheelIndex = 0;

			for (FVehicleWheel& wheel : Wheels.Wheels)
			{
				FVehicleContactSensor& sensor = wheel.Sensors[Wheels.GroundedSensorSet];
				FVector springTop = GetWheelBoneLocation(wheel, transform, true);

				sensor.Tick(deltaSeconds, World, transform, springTop, zdirection, true, estimate == true && SHOULD_ESTIMATE, IsFlippable());

				allInContact &= sensor.IsInContact();
			}

			wheelIndex = 0;

			for (FVehicleWheel& wheel : Wheels.Wheels)
			{
				FVehicleContactSensor& sensor = wheel.Sensors[Wheels.GroundedSensorSet ^ 1];
				FVector springTop = GetWheelBoneLocation(wheel, transform, true);

				sensor.Tick(deltaSeconds, World, transform, springTop, zdirection, (allInContact == false), estimate == true && SHOULD_ESTIMATE, IsFlippable());
			}
		}
		else
		{
			// If we're not properly grounded then tick the contact sensors in the less optimal way.

			int32 wheelIndex = 0;

			for (FVehicleWheel& wheel : Wheels.Wheels)
			{
				for (FVehicleContactSensor& sensor : wheel.Sensors)
				{
					FVector springTop = GetWheelBoneLocation(wheel, transform, true);

					sensor.Tick(deltaSeconds, World, transform, springTop, zdirection, true, estimate == true && SHOULD_ESTIMATE, IsFlippable());
				}
			}
		}

		// Determine the compression characteristics of the contact sensors, or how hard
		// the suspension is working.

		bool surfaceSet = false;
		bool hardCompression = false;
		float rearCompression = 0.0f;
		float frontCompression = 0.0f;

		for (FVehicleWheel& wheel : Wheels.Wheels)
		{
			for (FVehicleContactSensor& sensor : wheel.Sensors)
			{
				bool compressedHard = sensor.WasCompressedHard();

				hardCompression |= compressedHard;

				if (compressedHard == true &&
					Physics.ContactData.AirborneList.GetMeanValue(physicsClock - 2.0f) > 0.75f)
				{
					sensor.SpawnCompressionEffect();
				}

				if (surfaceSet == false &&
					sensor.IsInContact() == true)
				{
					// On the first contact for this frame and this vehicle, determine the surface and
					// skidding sound.

					surfaceSet = true;

					EGameSurface surfaceType = sensor.GetGameSurface();

					Wheels.SurfaceName = GetNameFromSurfaceType(surfaceType);
				}

				if (wheel.HasRearPlacement() == true)
				{
					rearCompression = FMath::Max(rearCompression, sensor.GetNormalizedCompression());
				}
				else
				{
					frontCompression = FMath::Max(frontCompression, sensor.GetNormalizedCompression());
				}
			}
		}

		if (hardCompression == true)
		{
			if (Wheels.HardCompressionTime == 0.0f)
			{
				Wheels.HardCompression = true;
			}

			Wheels.HardCompressionTime = 0.2f;
		}

		Wheels.HardCompressionTime = FMath::Max(Wheels.HardCompressionTime - deltaSeconds, 0.0f);

		int32 numUpNear = 0;
		int32 numDownNear = 0;
		float contactSeconds = 1.5f;

		FVector upNormal = FVector::ZeroVector;
		FVector downNormal = FVector::ZeroVector;
		FVector upLocation = FVector::ZeroVector;
		FVector downLocation = FVector::ZeroVector;

		// Determine which wheels are in contact with or are close to the ground.

		for (FVehicleWheel& wheel : Wheels.Wheels)
		{
			// Identify the contact sensor to be used for the wheel.

			// Sensors 0 = up, 1 = down (opposite if vehicle flipped)

			if (wheel.Sensors[0].IsInEffect() == true)
			{
				numUpContact++;

				upNormal += wheel.Sensors[0].GetNearestContactNormal();
				upLocation = wheel.Sensors[0].GetNearestContactPoint();
			}
			else if (wheel.Sensors[0].HasNearestContactPoint(wheel.Velocity, 0.0f) == true)
			{
				numUpNear++;

				upNormal += wheel.Sensors[0].GetNearestContactNormal();
				upLocation = wheel.Sensors[0].GetNearestContactPoint();
			}

			if (wheel.Sensors[1].IsInEffect() == true)
			{
				numDownContact++;

				downNormal += wheel.Sensors[1].GetNearestContactNormal();
				downLocation = wheel.Sensors[1].GetNearestContactPoint();
			}
			else if (wheel.Sensors[1].HasNearestContactPoint(wheel.Velocity, 0.0f))
			{
				numDownNear++;

				downNormal += wheel.Sensors[1].GetNearestContactNormal();
				downLocation = wheel.Sensors[1].GetNearestContactPoint();
			}
		}

		if (numUpContact + numUpNear > 0)
		{
			upNormal *= 1.0f / (numUpContact + numUpNear);
			upNormal.Normalize();
		}

		if (numDownContact + numDownNear > 0)
		{
			downNormal *= 1.0f / (numDownContact + numDownNear);
			downNormal.Normalize();
		}

		Physics.ContactData.WasAirborne = Physics.ContactData.Airborne;
		Physics.ContactData.Airborne = (numUpContact + numDownContact == 0);
		Physics.ContactData.Grounded = (numUpContact == numWheels || numDownContact == numWheels);

		// Manage the amount of time the car has been falling back to earth.
		// (We're officially falling if we've been falling back to earth for more than 0.666 seconds)

		if (Physics.ContactData.Airborne == true &&
			Physics.VelocityData.Velocity.Z < 0.0f)
		{
			Physics.ContactData.FallingTime += deltaSeconds;
		}
		else
		{
			Physics.ContactData.FallingTime = 0.0f;
		}

		// Determine which is the currently grounded sensor set, if any.

		if (numUpContact == numWheels)
		{
			Wheels.GroundedSensorSet = 0;
		}
		else if (numDownContact == numWheels)
		{
			Wheels.GroundedSensorSet = 1;
		}

		// Manage the time spent in airborne / non-airborne states.

		bool mostlyGrounded = (numUpContact > halfTheWheels || numDownContact > halfTheWheels);

		Physics.ContactData.GroundedList.AddValue(physicsClock, (mostlyGrounded == true) ? 1.0f : 0.0f);
		Physics.ContactData.AirborneList.AddValue(physicsClock, (Physics.ContactData.Airborne == true) ? 1.0f : 0.0f);

		if (Physics.ContactData.WasAirborne != Physics.ContactData.Airborne)
		{
			Physics.ContactData.LastModeTime = Physics.ContactData.ModeTime;
			Physics.ContactData.ModeTime = 0.0f;
		}
		else
		{
			Physics.ContactData.ModeTime += deltaSeconds;
		}

		if (Physics.ContactData.Grounded == true &&
			Physics.ContactData.ModeTime > 2.0f)
		{
			Physics.ContactData.RespawnLanded = true;
		}

		// Now try to figure out what's going on with the vehicle, mostly about whether it's flipped
		// or not. We put a lot of work into this because primarily, this flipped state affects the
		// spring arm and therefore the camera, and so we want no erratic changes in the flipped state
		// and try to determine it as best we can, only changing it when we're sure we need to.

		float d0 = 0.0f;
		float d1 = 0.0f;
		float dp0 = 0.0f;
		float dp1 = 0.0f;
		FVector i0 = FVector::ZeroVector;
		FVector i1 = FVector::ZeroVector;
		FVector location = transform.GetTranslation();
		bool upContactImminent = numUpContact > 0;
		bool downContactImminent = numDownContact > 0;
		FVector rayDirection = Physics.VelocityData.VelocityDirection;
		float cornerAngle = FMathEx::DotProductToDegrees(FVector::DotProduct(upNormal, downNormal));

		if (upContactImminent == false &&
			numUpNear + numUpContact != 0)
		{
			upContactImminent = (FVector::DotProduct(rayDirection, upNormal) < 0.0f && FMathEx::RayIntersectsPlane(location, rayDirection, upLocation, upNormal, i0) == true);

			if (upContactImminent == true)
			{
				d0 = (i0 - location).Size();

				if (d0 / Physics.VelocityData.Speed > contactSeconds)
				{
					upContactImminent = false;
				}
			}
		}

		if (downContactImminent == false &&
			numDownNear + numDownContact != 0)
		{
			downContactImminent = (FVector::DotProduct(rayDirection, downNormal) < 0.0f && FMathEx::RayIntersectsPlane(location, rayDirection, downLocation, downNormal, i1) == true);

			if (downContactImminent == true)
			{
				d1 = (i1 - location).Size();

				if (d1 / Physics.VelocityData.Speed > contactSeconds)
				{
					downContactImminent = false;
				}
			}
		}

		if (numUpNear + numUpContact != 0)
		{
			FVector p0 = FVector::PointPlaneProject(location, upLocation, upNormal);

			dp0 = (p0 - location).Size();
		}

		if (numDownNear + numDownContact != 0)
		{
			FVector p1 = FVector::PointPlaneProject(location, downLocation, downNormal);

			dp1 = (p1 - location).Size();
		}

		// Managed the detection of flip direction.

		bool flipped = Wheels.SoftFlipped;

		Wheels.SurfacesVincinal = true;

		if (IsFlippable() == false)
		{
			// If the vehicle isn't flippable then always indicate not flipped.

			Wheels.FlipDetection = 0;
			Wheels.SoftFlipped = false;
		}
		else if ((numUpContact != 0 && numDownContact == 0) ||
			(numUpContact == 0 && numDownContact != 0))
		{
			// We've a definite surface in contact with nothing on the other side. Simple case.

			Wheels.FlipDetection = 0;
			Wheels.SoftFlipped = numUpContact != 0;
		}
		else if (numUpContact != 0 && numDownContact != 0 && cornerAngle < 120.0f)
		{
			// We have contacts on both sides so we need to discriminate.

			// We're jammed in a corner.

			Wheels.FlipDetection = 2;

			// Figure out which surface we're most oriented towards and pick that if it's clear.

			if (dp0 < dp1 * 0.666f)
			{
				Wheels.SoftFlipped = true;
			}
			else if (dp1 < dp0 * 0.666f)
			{
				Wheels.SoftFlipped = false;
			}
		}
		else if (upContactImminent != downContactImminent)
		{
			// We've a surface coming into contact with nothing imminent on the other side. Another relatively simple case.

			Wheels.FlipDetection = 1;

			if (upContactImminent == true &&
				Wheels.SoftFlipped == false &&
				(dp0 < dp1 * 0.666f || dp1 == 0.0f))
			{
				Wheels.SoftFlipped = true;
			}
			else if (downContactImminent == true &&
				Wheels.SoftFlipped == true &&
				(dp1 < dp0 * 0.666f || dp0 == 0.0f))
			{
				Wheels.SoftFlipped = false;
			}
		}
		else if (IsFalling() == true)
		{
			Wheels.FlipDetection = 4;
			Wheels.SoftFlipped = (zdirection.Z < 0.0f);
			Wheels.SurfacesVincinal = false;
		}
		else
		{
			Wheels.FlipDetection = 5;
			Wheels.SurfacesVincinal = false;
		}

		if (flipped != Wheels.SoftFlipped)
		{
			Wheels.FlipTimer = 1.0f;
		}

		// NOTE: Only now is the current contact sensor set known, but we still need to update each wheel
		// so that they also know before using GetActiveSensor().

		Wheels.DetectedSurfaces = false;
		Wheels.FlipTimer = FMath::Max(Wheels.FlipTimer - (deltaSeconds * 4.0f), 0.0f);

		bounceCompression = (PlayGameMode != nullptr && PlayGameMode->PastGameSequenceStart());

		for (FVehicleWheel& wheel : Wheels.Wheels)
		{
			wheel.SensorIndex = (Wheels.SoftFlipped == true) ? 0 : 1;

			if (wheel.GetActiveSensor().HasNearestContactPoint(wheel.Velocity, 0.0f) == true)
			{
				Wheels.DetectedSurfaces = true;
			}

			{
				if (wheel.GetActiveSensor().GetNormalizedCompression() < 1.0f)
				{
					bounceCompression = false;
				}
			}
		}

		for (FVehicleWheel& wheel : Wheels.Wheels)
		{
			// Finally, actually apply the suspension forces to the vehicle for each wheel.

			for (FVehicleContactSensor& sensor : wheel.Sensors)
			{
				if (sensor.IsInContact() == true)
				{
					FVector forcesLocation = GetSuspensionForcesLocation(wheel, transform, deltaSeconds);

					sensor.ForceApplied = FMath::Max(sensor.ForceApplied, sensor.ForceToApply.Size());

					sensor.ApplyForce(forcesLocation);
				}
			}

			// Calculate how long a wheel has either been in contact or not in contact with a
			// driving surface through its ModeTime.

			bool wasInContact = wheel.IsInContact;

			wheel.IsInContact = wheel.GetActiveSensor().IsInContact();

			if (wasInContact != wheel.IsInContact)
			{
				wheel.ModeTime = 0.0f;
			}
			else
			{
				wheel.ModeTime += deltaSeconds;
			}
		}

		if (Wheels.HardFlipped != Wheels.SoftFlipped &&
			IsPracticallyGrounded() == true)
		{
			Wheels.HardFlipped = Wheels.SoftFlipped;
		}

		Wheels.SurfacesVincinal &= IsPracticallyGrounded(250.0f, true);
	}

	return numUpContact + numDownContact;
}

/**
* Are we allowed to engage the throttle to the wheels? (correct race state)
***********************************************************************************/

bool ABaseVehicle::IsPowerAvailable() const
{
	return (PlayGameMode != nullptr) ? PlayGameMode->PastGameSequenceStart() : true;
}

#pragma endregion VehicleContactSensors

#if WITH_PHYSX
#if GRIP_ENGINE_PHYSICS_MODIFIED

/**
* Modify a collision contact.
*
* Be very careful here! This is called from the physics sub-stepping at the same
* time as other game code may be executing its normal ticks. Therefore, this code
* needs to be thread-safe and be able to handle re-entrancy.
***********************************************************************************/

bool ABaseVehicle::ModifyContact(uint32 bodyIndex, AActor* other, physx::PxContactSet& contacts)
{
	return false;
}

#endif // GRIP_ENGINE_PHYSICS_MODIFIED
#endif // WITH_PHYSX

/**
* Set the velocities and related data for the physics state.
***********************************************************************************/

void FPhysicsVelocityData::SetVelocities(const FVector& linearVelocity, const FVector& angularVelocity, const FVector& xdirection)
{
	check(linearVelocity.ContainsNaN() == false);
	check(angularVelocity.ContainsNaN() == false);

	Velocity = linearVelocity;
	VelocityDirection = Velocity;

	if (VelocityDirection.SizeSquared() < KINDA_SMALL_NUMBER)
	{
		VelocityDirection = xdirection;
	}
	else
	{
		VelocityDirection.Normalize();
	}

	Speed = Velocity.Size();

	// Set a maximum speed of 2,000 kph to help stop code breakages further down the line.

	float maxSpeed = 55555.555f;

	if (Speed > maxSpeed)
	{
		Speed = maxSpeed;
		Velocity = VelocityDirection * Speed;
	}

	DirectedSpeed = Speed;

	if (Speed > 10.0f)
	{
		DirectedSpeed *= FVector::DotProduct(xdirection, VelocityDirection);
	}
}

/**
* Construct a UTireFrictionModel structure.
***********************************************************************************/

UTireFrictionModel::UTireFrictionModel()
{
	LateralGripVsSlip.GetRichCurve()->AddKey(0, 0.0f);
	LateralGripVsSlip.GetRichCurve()->AddKey(2, 0.3f);
	LateralGripVsSlip.GetRichCurve()->AddKey(4, 0.5f);
	LateralGripVsSlip.GetRichCurve()->AddKey(8, 0.7f);
	LateralGripVsSlip.GetRichCurve()->AddKey(16, 1.0f);
	LateralGripVsSlip.GetRichCurve()->AddKey(32, 1.3f);

	LongitudinalGripVsSlip.GetRichCurve()->AddKey(0.0f, 0.0f);
	LongitudinalGripVsSlip.GetRichCurve()->AddKey(21.0f, 0.75f);
	LongitudinalGripVsSlip.GetRichCurve()->AddKey(28.0f, 0.8f);
	LongitudinalGripVsSlip.GetRichCurve()->AddKey(100.0f, 0.5f);

	LateralGripVsSpeed.GetRichCurve()->AddKey(0.0f, 128.0f);
	LateralGripVsSpeed.GetRichCurve()->AddKey(100.0f, 175.0f);
	LateralGripVsSpeed.GetRichCurve()->AddKey(200.0f, 285.0f);
	LateralGripVsSpeed.GetRichCurve()->AddKey(300.0f, 400.0f);
	LateralGripVsSpeed.GetRichCurve()->AddKey(400.0f, 525.0f);
	LateralGripVsSpeed.GetRichCurve()->AddKey(500.0f, 650.0f);
	LateralGripVsSpeed.GetRichCurve()->AddKey(600.0f, 775.0f);

	GripVsSuspensionCompression.GetRichCurve()->AddKey(0.0f, 0.0f);
	GripVsSuspensionCompression.GetRichCurve()->AddKey(0.5f, 0.8f);
	GripVsSuspensionCompression.GetRichCurve()->AddKey(1.0f, 1.0f);
	GripVsSuspensionCompression.GetRichCurve()->AddKey(2.0f, 2.0f);

	RearLateralGripVsSpeed.GetRichCurve()->AddKey(0.0f, 1.25f);
	RearLateralGripVsSpeed.GetRichCurve()->AddKey(500.0f, 1.25f);
}

/**
* Construct a UVehicleEngineModel structure.
***********************************************************************************/

UVehicleEngineModel::UVehicleEngineModel()
{
	GearPowerRatios.Emplace(0.75f);
	GearPowerRatios.Emplace(0.5f);
	GearPowerRatios.Emplace(0.75f);
}

/**
* Construct a USteeringModel structure.
***********************************************************************************/

USteeringModel::USteeringModel()
{
	FrontSteeringVsSpeed.GetRichCurve()->AddKey(0.0f, 1.0f);
	FrontSteeringVsSpeed.GetRichCurve()->AddKey(88.0f, 0.65f);
	FrontSteeringVsSpeed.GetRichCurve()->AddKey(166.0f, 0.4f);
	FrontSteeringVsSpeed.GetRichCurve()->AddKey(300.0f, 0.3f);
	FrontSteeringVsSpeed.GetRichCurve()->AddKey(450.0f, 0.25f);

	BackSteeringVsSpeed.GetRichCurve()->AddKey(0.0f, 1.0f);
	BackSteeringVsSpeed.GetRichCurve()->AddKey(50, 0.66f);
	BackSteeringVsSpeed.GetRichCurve()->AddKey(100.0f, 0.0f);
}
