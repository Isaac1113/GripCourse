/**
*
* Vehicle AI bot implementation.
*
* Original author: Rob Baker.
* Current maintainer: Rob Baker.
*
* Copyright Caged Element Inc, code provided for educational purposes only.
*
* The core of the AI bot implementation for vehicles. Most of the vehicle-specific
* AI code you'll find here in this module. Specifically though, the collision
* avoidance code you'll find in a separate VehicleAvoidance.cpp module.
*
***********************************************************************************/

#include "vehicle/flippablevehicle.h"
#include "ai/pursuitsplineactor.h"
#include "ai/avoidancesphere.h"
#include "game/globalgamestate.h"
#include "ai/playeraicontext.h"

/**
* Construct an AI context.
***********************************************************************************/

FVehicleAI::FVehicleAI()
{
	int32 rand = FMath::Rand();

	PursuitSplineWidthTime = FMath::FRand() * PI;
	PursuitSplineWidthOverTime = FMath::FRand() * 0.25f + 0.25f;
	WheelplayCycles = ((rand % 2) == 0) ? 3 + ((rand >> 3) % 3) : 0.0f;
	VariableSpeedOffset = FMath::FRand() * PI * 2.0f;

	for (float& time : DrivingModeTimes)
	{
		time = 0.0f;
	}
}

/**
* Lock the steering to spline direction?
***********************************************************************************/

void ABaseVehicle::SteeringToSplineDirection(bool locked, bool avoidStaticObjects)
{

#pragma region AIVehicleControl

	AI.VolumeLockSteeringToSplineDirection = locked;
	AI.VolumeLockSteeringAvoidStaticObjects = avoidStaticObjects;

#pragma endregion AIVehicleControl

}

/**
* Is an AI driver good for a launch?
***********************************************************************************/

bool ABaseVehicle::AIVehicleGoodForLaunch(float probability, float minimumSpeedKPH) const
{

#pragma region AIVehicleControl

	if (AI.BotDriver == true)
	{
		if (FMath::FRand() <= probability &&
			GetSpeedKPH() > minimumSpeedKPH)
		{
			FVector vehicleHeading = GetTargetHeading();
			FVector vehicleDirection = GetFacingDirection();
			float headingAngleDifference = FVector::DotProduct(vehicleDirection, vehicleHeading);

			if (headingAngleDifference > FMathEx::ConeDegreesToDotProduct(10.0f))
			{
				return true;
			}
		}
	}

#pragma endregion AIVehicleControl

	return false;
}

#pragma region NavigationSplines

/**
* Get the direction of the vehicle compared to its pursuit spline.
***********************************************************************************/

int32 ABaseVehicle::GetPursuitSplineDirection() const
{
	if (GRIP_POINTER_VALID(AI.RouteFollower.ThisSpline) == false)
	{
		return 0;
	}
	else
	{
		return AI.RouteFollower.ThisSpline->GetRelativeDirectionAtDistanceAlongSpline(AI.RouteFollower.ThisDistance, GetFacingDirection());
	}
}

#pragma endregion NavigationSplines

#pragma region AINavigation

/**
* Perform the AI for a vehicle.
***********************************************************************************/

void ABaseVehicle::UpdateAI(float deltaSeconds)
{
	bool gameStartedForThisVehicle = (PlayGameMode->PastGameSequenceStart() == true);
	FVector location = GetActorLocation();
	const FTransform& transform = VehicleMesh->GetComponentTransform();
	FVector direction = transform.GetUnitAxis(EAxis::X);
	FVector movement = location - AI.LastLocation;
	FVector movementPerSecond = movement / deltaSeconds;

	AI.PrevLocation = AI.LastLocation;
	AI.LastLocation = location;

	// Handle all the movement of the vehicle.

	bool hasHeading = false;
	FVector wasHeadingTo = AI.HeadingTo;

	AI.OptimumSpeed = 0.0f;
	AI.MinimumSpeed = 0.0f;
	AI.HeadingTo = FVector(0.0f, 0.0f, 0.0f);

	float accuracy = 1.0f;
	float numIterations = 5;

	// If we're into the race then add some power, not full power as we want to allow
	// the human player to catch up.

	if (GRIP_POINTER_VALID(AI.RouteFollower.ThisSpline) == true)
	{
		// Handle spline following, always have some movement to help find where we are on
		// splines with some accuracy.

		float movementSize = FMath::Max(100.0f, movement.Size());

		AIFollowSpline(location, wasHeadingTo, movement, movementSize, deltaSeconds, numIterations, accuracy);

		// See if we should be driving carefully at this point along the spline.

		bool locked = AI.RouteFollower.ThisSpline->GetCarefulDrivingAtDistanceAlongSpline(AI.RouteFollower.ThisDistance);

		AI.LockSteeringToSplineDirection = AI.VolumeLockSteeringToSplineDirection | locked;
		AI.LockSteeringAvoidStaticObjects = AI.VolumeLockSteeringAvoidStaticObjects | locked;

		// We want to aim for half a second ahead at normal distance from spline.
		// Determine the aim point on the spline at that distance ahead, switching splines at branches if necessary.

		float ahead = FMath::Max(3333.333f, Physics.VelocityData.Velocity.Size() * 0.5f);

#pragma region AIVehicleControl

		if (AI.LastTime(EVehicleAIDrivingMode::ReversingFromBlockage) != 0.0f)
		{
			float timeSinceReversing = AI.TimeSince(EVehicleAIDrivingMode::ReversingFromBlockage, VehicleClock);

			if (AI.DrivingMode == EVehicleAIDrivingMode::GeneralManeuvering &&
				timeSinceReversing < 5.0f)
			{
				// If we've recently had to reverse out from a blockage, then try to get back onto
				// the spline more directly as the blockage is still likely around.

				float ratio = FMathEx::GetRatio(timeSinceReversing, 2.0f, 5.0f);

				ahead = FMath::Lerp(5.0f * 100.0f, ahead, ratio);
			}
		}

#pragma endregion AIVehicleControl

		AIDetermineSplineAimPoint(ahead, movementSize);

		// So now we know where we are and where we're aiming for.

		AI.HeadingTo = AI.RouteFollower.NextSpline->GetWorldLocationAtDistanceAlongSpline(AI.RouteFollower.NextDistance);
		AI.OptimumSpeed = AI.RouteFollower.ThisSpline->GetOptimumSpeedAtDistanceAlongSpline(AI.RouteFollower.ThisDistance);
		AI.MinimumSpeed = AI.RouteFollower.ThisSpline->GetMinimumSpeedAtDistanceAlongSpline(AI.RouteFollower.ThisDistance);
		AI.TrackOptimumSpeed = AI.OptimumSpeed;

#pragma region AIVehicleControl

#if GRIP_BOT_INTELLIGENT_SPEEDVSGRIP

		if (AI.OptimumSpeed != 0.0f &&
			AI.OptimumSpeedExtension > 0.0f)
		{
			AI.OptimumSpeed += 66.0f * AI.OptimumSpeedExtension;
		}

#endif // GRIP_BOT_INTELLIGENT_SPEEDVSGRIP

		{
			if (AI.OptimumSpeed != 0.0f)
			{
				float optimumSpeed = AI.OptimumSpeed;
				float makeUpSpeed = FMathEx::GetRatio(optimumSpeed - GetSpeedKPH(), 0.0f, 100.0f);

				if (makeUpSpeed > KINDA_SMALL_NUMBER)
				{
					// If speeding up to the optimum speed then aim long until we get there as we
					// want to get there quickly. I'm not even sure this really does anything much
					// as the bot use of throttle is already pretty aggressive. But every little
					// helps.

					AI.OptimumSpeed += optimumSpeed * FMath::Pow(makeUpSpeed, 0.5f) * 0.25f;
				}

				AI.OptimumSpeed += FMath::Sin(AI.VariableSpeedOffset) * optimumSpeed * 0.05f;

				AI.OptimumSpeed = FMath::Min(AI.OptimumSpeed, 1000.0f);
			}
		}

		if (AI.MinimumSpeed < 150.0f &&
			RaceState.RaceTime > 10.0f)
		{
			AI.MinimumSpeed = 150.0f;
		}

#pragma endregion AIVehicleControl

		// Update the variables used for spline weaving and speed variation.

		AI.UpdateSplineFollowing(deltaSeconds, GetSpeedKPH());

		AIUpdateSplineWeaving(location);

		// Add in the side offset for maneuvering across the spline width into the head-to location.
		// NOTE: Roll on the spline is important here, so we need to ensure this setup properly in the track data.

		FQuat splineRotation = AI.RouteFollower.NextSpline->GetWorldSpaceQuaternionAtDistanceAlongSpline(AI.RouteFollower.NextDistance);

		// Add in the width offset to the heading.

		AI.HeadingTo += splineRotation.RotateVector(FVector(0.0f, AI.GetSplineWeavingOffset(true), 0.0f));
		AI.WeavingPosition = AI.HeadingTo;

		hasHeading = true;
	}

#pragma region AIVehicleControl

	if (AI.BotDriver == true)
	{
		FVector heading = AI.HeadingTo - location;

		heading.Normalize();

		AIUpdateDrivingMode(movementPerSecond, direction, heading);

		AI.DrivingModeTime += deltaSeconds;

		if (hasHeading == true)
		{
			// If we have somewhere to go, then calculate the control inputs required to get there.

			AICalculateControlInputs(transform, location, direction, movementPerSecond, deltaSeconds);
		}
	}

	if (gameStartedForThisVehicle == true)
	{
		AIRecordVehicleProgress(transform, movement, direction, deltaSeconds);
	}

#pragma endregion AIVehicleControl

}

/**
* Is this bot driver driving casually, and not in a tight driving situation?
***********************************************************************************/

bool FVehicleAI::IsDrivingCasually(bool ignoreVehicles) const
{
	return (DrivingMode == EVehicleAIDrivingMode::GeneralManeuvering && Fishtailing == false);
}

/**
* Reset the spline following so that it starts over.
***********************************************************************************/

void ABaseVehicle::AIResetSplineFollowing(bool beginPlay, bool allowDeadEnds, bool keepCurrentSpline, bool retainLapPosition, float minMatchingDistance)
{
	if (GameState != nullptr &&
		PlayGameMode != nullptr)
	{
		if (beginPlay == true ||
			keepCurrentSpline == true ||
			AI.ClosestSplineEvaluationEnabled == true)
		{
			float distance = 0.0f;
			TWeakObjectPtr<UPursuitSplineComponent> spline;

			if (keepCurrentSpline == true)
			{
				spline = AI.RouteFollower.ThisSpline;
				distance = AI.RouteFollower.ThisDistance;

				if (GRIP_POINTER_VALID(spline) == true)
				{
					AI.DistanceFromPursuitSpline = (GetActorLocation() - spline->GetWorldLocationAtDistanceAlongSpline(distance)).Size();
				}
			}
			else
			{
				AI.DistanceFromPursuitSpline = -1.0f;

				FVector direction = GetFacingDirection();
				bool gameInProgress = (beginPlay == false);

				if (GameState->IsGameModeRace() == false)
				{
					retainLapPosition = false;
				}

				for (int32 pass = ((retainLapPosition == true) ? 0 : 1); pass < 2; pass++)
				{
					// Only look for splines that are in the vicinity of the current main spline distance,
					// but only if this is pass 0 as we've been asked to retain the lap position. On pass 1
					// we've either not been asked to find a match against a master racing spline distance
					// or we couldn't find a suitable match on pass 0.

					bool matchReferenceDistanceAlongSpline = pass == 0;

					distance = RaceState.DistanceAlongMasterRacingSpline;

					// Look just for visible splines first.

					bool splineIsVisible = APursuitSplineActor::FindNearestPursuitSpline(AI.LastLocation, direction, GetWorld(), spline, AI.DistanceFromPursuitSpline, distance, EPursuitSplineType::General, gameInProgress, matchReferenceDistanceAlongSpline, beginPlay, allowDeadEnds, minMatchingDistance);

					// If we're a distance away from the nearest visible spline then also look for any spline whether visible or not.

					if ((GRIP_POINTER_VALID(spline) == false) ||
						(splineIsVisible == true && AI.DistanceFromPursuitSpline > 250.0f * 100.0f))
					{
						float otherDistanceFromPursuitSpline = -1.0f;
						float otherDistance = RaceState.DistanceAlongMasterRacingSpline;
						TWeakObjectPtr<UPursuitSplineComponent> otherSpline;

						// Look for any spline whether visible or not, because we really want a better match
						// than the last one which was too far away really.

						APursuitSplineActor::FindNearestPursuitSpline(AI.LastLocation, direction, GetWorld(), otherSpline, otherDistanceFromPursuitSpline, otherDistance, EPursuitSplineType::General, false, matchReferenceDistanceAlongSpline, beginPlay, allowDeadEnds, minMatchingDistance);

						if (GRIP_POINTER_VALID(otherSpline) == true)
						{
							// If the distance away from any spline is less than half that of the nearest visible spline then
							// use that one instead. We're taking a risk on an invisible spline so it needs to be considerably
							// closer for us to want to take that risk.

							if (otherDistanceFromPursuitSpline < AI.DistanceFromPursuitSpline * 0.5f)
							{
								spline = otherSpline;
								distance = otherDistance;
								AI.DistanceFromPursuitSpline = otherDistanceFromPursuitSpline;
							}
						}
					}

					if (GRIP_POINTER_VALID(spline) == true)
					{
						break;
					}
				}
			}

			if (PlayGameMode->PursuitSplines.Num() > 0)
			{
				ensureAlwaysMsgf(GRIP_POINTER_VALID(spline) == true, TEXT("Couldn't find a spline to link to"));
			}

			if (GRIP_POINTER_VALID(spline) == true)
			{
				if (beginPlay == false &&
					retainLapPosition == true)
				{
					// Do a check to ensure our new distance hasn't jumped too far from the master racing spline
					// distance if that's what we've been matching against.

					float distanceAlongMasterRacingSpline = spline->GetMasterDistanceAtDistanceAlongSpline(distance, PlayGameMode->MasterRacingSplineLength);

					ensureAlwaysMsgf(FMath::Abs(PlayGameMode->MasterRacingSpline->GetDistanceDifference(RaceState.DistanceAlongMasterRacingSpline, distanceAlongMasterRacingSpline)) < 250.0f * 100.0f, TEXT("Jumped too far along the master racing spline"));
				}

				// Check whether we need to switch away from the current spline to the new spline we've identified.

				if (retainLapPosition == false ||
					AI.RouteFollower.ThisSpline != spline ||
					spline->GetDistanceDifference(AI.RouteFollower.ThisDistance, distance) > 10.0f * 100.0f)
				{
					// Don't switch to a path that will quickly merge into the one we're on.

					if (retainLapPosition == false ||
						AI.RouteFollower.ThisSpline.IsValid() == false ||
						spline->IsAboutToMergeWith(AI.RouteFollower.ThisSpline.Get(), distance) == false)
					{
						// OK, let's switch splines.

						AI.RouteFollower.SwitchingSpline = false;
						AI.RouteFollower.LastSpline = AI.RouteFollower.ThisSpline;
						AI.RouteFollower.LastDistance = AI.RouteFollower.ThisDistance;
						AI.RouteFollower.ThisSpline = spline;
						AI.RouteFollower.ThisDistance = distance;
						AI.RouteFollower.NextSpline = spline;
						AI.RouteFollower.NextDistance = distance;
						AI.RouteFollower.DecidedDistance = -1.0f;
						AI.RouteFollower.ThisSwitchDistance = 0.0f;

						AI.SplineWorldLocation = AI.RouteFollower.ThisSpline->GetWorldLocationAtDistanceAlongSpline(AI.RouteFollower.ThisDistance);
						AI.SplineWorldDirection = AI.RouteFollower.ThisSpline->GetWorldDirectionAtDistanceAlongSpline(AI.RouteFollower.ThisDistance);

						AI.OutsideSplineCount = 0.0f;

						AIResetSplineWeaving();
					}
				}
			}
		}
	}
}

/**
* Follow the current spline, and switch over to the next if necessary.
***********************************************************************************/

void ABaseVehicle::AIFollowSpline(const FVector& location, const FVector& wasHeadingTo, const FVector& movement, float movementSize, float deltaSeconds, int32 numIterations, float accuracy)
{
	if (IsVehicleDestroyed() == false)
	{
		RaceState.LastDistanceAlongMasterRacingSpline = RaceState.DistanceAlongMasterRacingSpline;

		if (Clock0p25.ShouldTickNow() == true)
		{
			AI.RouteFollower.DetermineThis(location, movementSize, numIterations, accuracy);
		}
		else
		{
			AI.RouteFollower.EstimateThis(location, movement, movementSize, numIterations, accuracy);
		}

		if ((AI.RouteFollower.ThisSpline->DeadEnd == true) &&
			FMath::Abs(AI.RouteFollower.ThisSpline->GetSplineLength() - AI.RouteFollower.ThisDistance) < Physics.VelocityData.Speed * 0.1f)
		{
			// Dead end so probably arena mode, the absolute nearest point will do rather than lap distance.

			AIResetSplineFollowing(false, false, false, false);
		}

		bool resetTrackFollowing = false;

		if (IsPracticallyGrounded() == false)
		{
			AI.ReassessSplineWhenGrounded = true;
		}
		else if (AI.ReassessSplineWhenGrounded == true && IsGrounded() == true)
		{
			AI.ReassessSplineWhenGrounded = false;

			FTransform transform = AI.RouteFollower.ThisSpline->GetTransformAtDistanceAlongSpline(AI.RouteFollower.ThisDistance, ESplineCoordinateSpace::World);
			FVector vehicleUp = GetLaunchDirection();
			FVector splineUp = transform.GetUnitAxis(EAxis::Z);

			if (FMath::Abs(FVector::DotProduct(splineUp, vehicleUp)) < 0.5f)
			{
				float width = AI.RouteFollower.ThisSpline->GetWidthAtDistanceAlongSpline(AI.RouteFollower.ThisDistance);

				if ((AI.LastLocation - transform.GetLocation()).Size() > width * 100.0f * 0.5f)
				{
					resetTrackFollowing = true;
				}
			}
		}

		if (Clock0p25.ShouldTickNow() == true &&
			HasAIDriver() == false)
		{
			// Ensure human drivers are linked to the closest splines if at all possible.

			resetTrackFollowing = true;
		}

		// Check that a connection from one spline to another has been taken.

		if ((resetTrackFollowing == true) ||
			(AI.RouteFollower.CheckBranchConnection(GetWorld(), location, 100.0f * 100.0f) == true))
		{
			// Find nearest to current lap distance.

			AIResetSplineFollowing(false);
		}
		else if (Clock0p25.ShouldTickNow() == true &&
			AI.RouteFollower.SwitchingSpline == false)
		{
			// Check the spline is still in range of the vehicle.

			AICheckSplineValidity(location, 0.25f, false);
		}

		// So we have the nearest point on the spline we're following.
		// Now we need to head towards a point on that spline. We'll calculate that from
		// the speed we are going along with how far away we are from the spline.

		AI.SplineWorldLocation = AI.RouteFollower.ThisSpline->GetWorldLocationAtDistanceAlongSpline(AI.RouteFollower.ThisDistance);
		AI.SplineWorldDirection = AI.RouteFollower.ThisSpline->GetWorldDirectionAtDistanceAlongSpline(FMath::Clamp(AI.RouteFollower.ThisDistance, 1.0f, AI.RouteFollower.ThisSpline->GetSplineLength() - 1.0f));
		AI.DistanceFromPursuitSpline = (location - AI.SplineWorldLocation).Size();

		if (GameState->IsGameModeRace() == true)
		{
			float lastDistance = RaceState.DistanceAlongMasterRacingSpline;

			RaceState.DistanceAlongMasterRacingSpline = AI.RouteFollower.ThisSpline->GetMasterDistanceAtDistanceAlongSpline(AI.RouteFollower.ThisDistance, PlayGameMode->MasterRacingSplineLength);
		}

		if (IsPracticallyGrounded(100.0f) == true)
		{
			Physics.LastGroundedLocation = location;

			RaceState.GroundedDistanceAlongMasterRacingSpline = RaceState.DistanceAlongMasterRacingSpline;
		}
	}
}

/**
* Has this vehicle gone off-track somehow?
***********************************************************************************/

bool ABaseVehicle::IsVehicleOffTrack(bool extendedChecks)
{
	if (AI.RouteFollower.ThisSpline == nullptr)
	{
		return false;
	}

	FVector up = AI.RouteFollower.ThisSpline->GetWorldSpaceUpVectorAtDistanceAlongSpline(AI.RouteFollower.ThisDistance);
	float maxDistance = FMathEx::MetersToCentimeters(AI.RouteFollower.ThisSpline->GetWidthAtDistanceAlongSpline(AI.RouteFollower.ThisDistance) * 0.5f);
	float offTrackDistance = FMathEx::MetersToCentimeters(GameState->TransientGameState.OffTrackDistance);
	float underTrackDistance = FMathEx::MetersToCentimeters(GameState->TransientGameState.UnderTrackDistance);

	if ((AI.DistanceFromPursuitSpline - maxDistance > offTrackDistance && offTrackDistance > KINDA_SMALL_NUMBER) ||
		(FVector::DotProduct(AI.LastLocation - AI.SplineWorldLocation, up) < 0.0f &&
		FPlane::PointPlaneDist(AI.LastLocation, AI.SplineWorldLocation, up) - maxDistance > underTrackDistance && underTrackDistance > KINDA_SMALL_NUMBER))
	{
		if ((extendedChecks == false) ||
			(IsPracticallyGrounded() == false))
		{
			return true;
		}
	}

	return false;
}

/**
* Switch splines if the current one looks suspect.
***********************************************************************************/

bool ABaseVehicle::AICheckSplineValidity(const FVector& location, float checkCycle, bool testOnly)
{
	if (PlayGameMode != nullptr &&
		PlayGameMode->PastGameSequenceStart() == true)
	{
		// OK, so we need to project this point in space onto the nearest driving surface, ideally.
		// The reason being, splines are often quite high above the ground and perhaps not very wide,
		// so we need to compare against that projection instead.

		FVector gp = AI.RouteFollower.ThisSpline->GetWorldClosestPosition(AI.RouteFollower.ThisDistance, true);
		float dt = (location - gp).Size();
		bool offTrack = IsVehicleOffTrack(false);
		bool tooFarAway = dt > FMathEx::MetersToCentimeters(FMath::Max(AI.RouteFollower.ThisSpline->GetWidthAtDistanceAlongSpline(AI.RouteFollower.ThisDistance) * 1.5f, 15.0f) + GetAvoidanceRadius());
		bool canSee = AI.RouteFollower.ThisSpline->IsWorldLocationWithinRange(AI.RouteFollower.ThisDistance, location);

		if (canSee == false ||
			offTrack == true ||
			tooFarAway == true)
		{
			if (testOnly == false)
			{
				AI.OutsideSplineCount += checkCycle;

				if (offTrack == true ||
					AI.OutsideSplineCount > 2.5f)
				{
					// If we've not been within our current spline bounds for a couple of seconds then
					// reset the track following.

					// Find nearest to current lap distance.

					AIResetSplineFollowing(false);
				}
			}

			return true;
		}
		else
		{
			AI.OutsideSplineCount = 0.0f;
		}
	}

	return false;
}

/**
* Determine where to aim on the spline, switching splines at branches if necessary.
*
* The vehicle itself will follow on a little later, as the aim point is always ahead
* of the vehicle.
***********************************************************************************/

void ABaseVehicle::AIDetermineSplineAimPoint(float ahead, float movementSize)
{
	bool freeSlot = false;

	AI.RouteFollower.DetermineNext(ahead, movementSize, (StayOnThisSpline() == true || HasAIDriver() == false) ? GetAI().RouteFollower.ThisSpline.Get() : nullptr, false, freeSlot, IsUsingTurbo(), -RaceState.RaceCatchupRatio);
}

/**
* Update an offset from the center line of the current aiming spline that makes the
* car weaves around a little on the track rather than appearing robotic.
***********************************************************************************/

void ABaseVehicle::AIUpdateSplineWeaving(const FVector& location)
{
	if (AI.RouteFollower.NextSpline != nullptr)
	{
		// Now handle the width we're aiming for across the current spline.

		float maxDistance = FMathEx::MetersToCentimeters(AI.RouteFollower.NextSpline->GetWidthAtDistanceAlongSpline(AI.RouteFollower.NextDistance) * 0.5f);

		// Ensure we have at least 1m to play with either side.

		AI.PursuitSplineWidthOffset = FMath::Max(maxDistance, 1.0f * 100.0f);

		if (AI.ResetPursuitSplineWidthOffset == true)
		{
			// Handle resetting of the spline width offset to match the current vehicle state, normally the
			// direction its moving or facing in. This is useful for smoothly getting back into weaving after
			// we've been distracted with more important maneuvering.

			AI.ResetPursuitSplineWidthOffset = false;

			AI.SmoothedPursuitSplineWidthOffset = AI.PursuitSplineWidthOffset;

			// Construct a plane at the point ahead on the that we're aiming at, and see where our
			// vehicle direction vector intersects it.

			FVector locationAhead = AI.RouteFollower.NextSpline->GetLocationAtDistanceAlongSpline(AI.RouteFollower.NextDistance, ESplineCoordinateSpace::World);
			FVector directionAhead = AI.RouteFollower.NextSpline->GetDirectionAtDistanceAlongSpline(AI.RouteFollower.NextDistance, ESplineCoordinateSpace::World) * -1.0f;
			FVector intersection = FVector::ZeroVector;

			if (FMathEx::RayIntersectsPlane(location, GetVelocityOrFacingDirection(), locationAhead, directionAhead, intersection) == true)
			{
				// Find a ray plane intersection so go ahead and transform it back into spline space
				// in order to find its Y or side position in that space.

				FTransform transformAhead = AI.RouteFollower.NextSpline->GetTransformAtDistanceAlongSpline(AI.RouteFollower.NextDistance, ESplineCoordinateSpace::World);

				intersection = transformAhead.InverseTransformPosition(intersection);

				// We can now convert that side position into a ratio against the width offset that
				// we have available.

				float ratio = FMath::Min(FMath::Abs(intersection.Y) / AI.SmoothedPursuitSplineWidthOffset, 1.0f);

				// And then convert the ratio using Asin to get the width time (which will be multiplied
				// by Sin later in the computation of the weaving offset vector).

				AI.PursuitSplineWidthTime = FMath::Asin(ratio) * FMathEx::UnitSign(intersection.Y);
			}
			else
			{
				// Convert the approximate side position into a ratio against the width offset that we
				// have available. We're not taking direction into account here, as this entire code
				// block is just a fall-back position that is rarely called.

				float ratio = FMath::Min(AI.DistanceFromPursuitSpline / AI.SmoothedPursuitSplineWidthOffset, 1.0f);

				// Get the side of the spline that the vehicle location falls on.

				float side = AI.RouteFollower.ThisSpline->GetSide(AI.RouteFollower.ThisDistance, location);

				// And then convert the ratio using Asin to get the width time (which will be multiplied
				// by Sin later in the computation of the weaving offset vector).

				AI.PursuitSplineWidthTime = FMath::Asin(ratio) * side;
			}

			if (FMath::RandBool() == true)
			{
				// Randomize the two times on the Sin arc that equate to this width, to try to randomize
				// the weaving vehicles will exhibit from hereon in.

				AI.PursuitSplineWidthTime = (HALF_PI + (HALF_PI - FMath::Abs(AI.PursuitSplineWidthTime))) * FMathEx::UnitSign(AI.PursuitSplineWidthTime);
			}
		}
	}
}

/**
* Update the variables used for spline weaving and speed variation.
***********************************************************************************/

void FVehicleAI::UpdateSplineFollowing(float deltaSeconds, float speedKPH)
{
	if (LockSteeringToSplineDirection == false &&
		LockSteeringAvoidStaticObjects == false)
	{
		// If we're not locked into a steering solution then animate the weaving here.

		const float minSpeed = 150.0f;
		const float maxSpeed = 300.0f;

		float weavingRatio = PursuitSplineWeavingRatio;

		if (speedKPH < minSpeed)
		{
			// No weaving around when we're at low speed.

			weavingRatio = 0.0f;
		}
		else if (speedKPH < maxSpeed)
		{
			// Ramp up the weaving as we gather more speed.

			weavingRatio *= (speedKPH - minSpeed) / (maxSpeed - minSpeed);
		}

		// Animate the weaving time.

		PursuitSplineWidthTime += PursuitSplineWidthOverTime * weavingRatio * deltaSeconds;

		// Smooth in weaving when we've just reset splines, after deviating to an
		// attractable for example and rejoining spline following.

		PursuitSplineWeavingRatio = FMath::Min(PursuitSplineWeavingRatio + deltaSeconds, 1.0f);

		if (PursuitSplineTransitionSpeed > KINDA_SMALL_NUMBER)
		{
			// Smooth in the transition between pursuit splines and attractable objects.

			PursuitSplineFollowingRatio = FMath::Min(PursuitSplineFollowingRatio + (PursuitSplineTransitionSpeed * deltaSeconds), 1.0f);
		}
	}

	SmoothedPursuitSplineWidthOffset = FMathEx::GravitateToTarget(SmoothedPursuitSplineWidthOffset, PursuitSplineWidthOffset, (50.0f * 100.0f) * deltaSeconds);

	// Animate the variation in optimum speed for vehicles.

	VariableSpeedOffset += deltaSeconds / 10.0f;
}

#pragma endregion AINavigation

#pragma region AIVehicleControl

/**
* Request a new driving mode for the vehicle.
***********************************************************************************/

void FVehicleAI::SetDrivingMode(EVehicleAIDrivingMode mode)
{
	DrivingMode = mode;
	DrivingModeTime = 0.0f;
	DrivingModeDistance = 0.0f;

	if (mode == EVehicleAIDrivingMode::JTurnToReorient)
	{
		ReorientationStage = 0;
	}
	else if (mode == EVehicleAIDrivingMode::RecoveringControl)
	{
		switch (DifficultyLevel)
		{
		case 2:
			UseProRecovery = (FMath::Rand() & 1) == 0;
			break;
		case 3:
			UseProRecovery = true;
			break;
		default:
			UseProRecovery = false;
			break;
		}
	}
}

/**
* Update the start-line engine revving.
***********************************************************************************/

void FVehicleAI::UpdateRevving(float deltaSeconds, bool gameStarted)
{
	if (gameStarted == false &&
		WillRevOnStartLine == true)
	{
		RevvingTimer += deltaSeconds;

		if (RevvingTimer >= RevvingTime)
		{
			Revving ^= true;
			RevvingTimer = 0.0f;

			if (Revving == true)
			{
				if (WillBurnoutOnStartLine == true)
				{
					RevvingTime = FMath::FRandRange(1.5f, 2.5f);
				}
				else if (FMath::Rand() & 1)
				{
					RevvingTime = FMath::FRandRange(0.25f, 0.5f);
				}
				else
				{
					RevvingTime = FMath::FRandRange(1.0f, 1.5f);
				}
			}
			else
			{
				RevvingTime = FMath::FRandRange(0.5f, 0.75f);
			}
		}
	}
	else
	{
		Revving = false;
	}

	if (Revving == true)
	{
		TorqueRoll += deltaSeconds * 5.0f;
		TorqueRoll = FMath::Min(TorqueRoll, 1.0f);
	}
	else
	{
		TorqueRoll -= deltaSeconds * 5.0f;
		TorqueRoll = FMath::Max(TorqueRoll, 0.0f);
	}
}

/**
* Manage drifting around long, sweeping corners.
***********************************************************************************/

void ABaseVehicle::AIUpdateDrifting(const FVector& location, const FVector& direction)
{
	// Of course, only drift if it would be wise to do so.

	if (AICanDrift() == true &&
		IsDrifting() == false &&
		Physics.Drifting.Timer > 1.5f)
	{
		// Done all the easy checks, now to prevent less easily identifiable conditions.

		// We don't want to drift for a short period, we really want it for several seconds
		// as otherwise it's not really worth doing, but how can we determine that ahead
		// of time?

		{
			// Obtain the change in rotation of the spline over 2 seconds time at the current
			// vehicle speed.

			float splineSeconds = 2.0f;
			FRotator splineDegrees = FRotator::ZeroRotator;
			float distanceAhead = splineSeconds * FMathEx::MetersToCentimeters(GetSpeedMPS());

			if (AI.RouteFollower.IsValid() == true)
			{
				splineDegrees = AI.RouteFollower.GetCurvatureOverDistance(AI.RouteFollower.ThisDistance, distanceAhead, GetPursuitSplineDirection(), FQuat::Identity, false);

				// Convert to degrees per second.

				splineDegrees *= 1.0f / splineSeconds;
			}

			// The degrees we've been given are in world space for easy comparison.
			// We only really want to be examining the yaw component, so first ensure
			// the car is relative upright (can be on floor or ceiling, either way up).

			FRotator rotation = GetActorRotation();

			// Only if we're roughly horizontal with regard to roll.

			if (FMath::Abs(rotation.Roll) < 30.0f ||
				FMath::Abs(rotation.Roll) > 150.0f)
			{
				if (FMath::Abs(splineDegrees.Yaw) > FMath::Lerp(20.0f, 10.0f, FMathEx::GetRatio(GetSpeedKPH(), 250.0f, 500.0f)))
				{
					StartDrifting();
				}
			}
		}
	}
}

/**
* Update the driving mode of the vehicle, this is the main driving coordination
* center.
***********************************************************************************/

void ABaseVehicle::AIUpdateDrivingMode(const FVector& movementPerSecond, const FVector& direction, const FVector& heading)
{
	AI.DrivingModeTimes[(int32)AI.DrivingMode] = VehicleClock;

	switch (AI.DrivingMode)
	{
	case EVehicleAIDrivingMode::GeneralManeuvering:
		AIUpdateGeneralManeuvering(movementPerSecond, direction, heading);
		break;

	case EVehicleAIDrivingMode::RecoveringControl:
		AIUpdateRecoveringControl(direction, heading);
		break;

	case EVehicleAIDrivingMode::ReversingToReorient:
		AIUpdateReversingToReorient(movementPerSecond, direction, heading);
		break;

	case EVehicleAIDrivingMode::ReversingFromBlockage:
		AIUpdateReversingFromBlockage(movementPerSecond);
		break;

#pragma region VehicleLaunch

	case EVehicleAIDrivingMode::LaunchToReorient:
		AIUpdateLaunchToReorient(direction, heading);
		break;

#pragma endregion VehicleLaunch

	case EVehicleAIDrivingMode::JTurnToReorient:
		AIUpdateJTurnToReorient(direction, heading);
		break;
	}
}

/**
* Determine if the vehicle is still in normal control and switch driving mode if
* not.
***********************************************************************************/

void ABaseVehicle::AIUpdateGeneralManeuvering(const FVector& movementPerSecond, const FVector& direction, const FVector& heading)
{
	AIAreWeStuck(movementPerSecond, false);
	AIHaveWeLostControl(direction, heading);
}

/**
* Determine if the vehicle has recovered control and switch to a new driving mode if
* so.
***********************************************************************************/

void ABaseVehicle::AIUpdateRecoveringControl(const FVector& direction, const FVector& heading)
{
	// If we're trying to recover control then limit the speed.

	AI.OptimumSpeed = FMath::Min(AI.OptimumSpeed, 250.0f);

	float angleAway = FMathEx::DotProductToDegrees(FVector::DotProduct(direction, heading));
	float maxAngleAway = 45.0f;

	if (angleAway < maxAngleAway &&
		FMath::Abs(Physics.VelocityData.AngularVelocity.Z) < FMath::Lerp(125.0f, 75.0f, angleAway / maxAngleAway))
	{
		// If we're heading back in the normal direction and not flat-spinning
		// then no more recovering control.

		AI.SetDrivingMode(EVehicleAIDrivingMode::GeneralManeuvering);
	}
	else
	{
		// We're not heading in the right direction or our spin rate is too high,
		// so let's look at other ways to recover.

		float splineAngleAway = FMathEx::DotProductToDegrees(FVector::DotProduct(direction, AI.SplineWorldDirection));
		float angleFromVertical = FMathEx::DotProductToDegrees(FVector::DotProduct(FVector(0.0f, 0.0f, 1.0f), GetLaunchDirection()));

		if (AI.UseProRecovery == true &&
			angleFromVertical < 45.0f &&
			(angleAway > 135.0f || splineAngleAway > 135.0f) &&
			IsPracticallyGrounded() == true &&
			FMath::Abs(Physics.VelocityData.AngularVelocity.Z) < 50.0f &&
			(AI.CollisionBlockage & (VehicleBlockedRight | VehicleBlockedLeft)) == 0)
		{
			AI.SetDrivingMode(EVehicleAIDrivingMode::JTurnToReorient);
		}

#pragma region VehicleLaunch

		else if (AI.UseProRecovery == true &&
			angleFromVertical < 45.0f &&
			(angleAway > 135.0f || splineAngleAway > 135.0f) &&
			IsPracticallyGrounded() == true &&
			FMath::Abs(Physics.VelocityData.AngularVelocity.Z) < 50.0f &&
			GetSpeedKPH() < 400.0f)
		{
			LaunchChargeOn(true);

			if (LaunchCharging == ELaunchStage::Charging)
			{
				AI.SetDrivingMode(EVehicleAIDrivingMode::LaunchToReorient);
			}
		}

#pragma endregion VehicleLaunch

		else
		{
			if (FMath::Abs(Physics.VelocityData.AngularVelocity.Z) < 75.0f &&
				GetSpeedKPH() < FMath::Lerp(250.0f, 125.0f, FMath::Min(1.0f, angleAway / maxAngleAway)))
			{
				// We've recovered some measure of control.

				if (angleAway > 135.0f)
				{
					// Reverse up if we need to reorient.

					AI.SetDrivingMode(EVehicleAIDrivingMode::ReversingToReorient);
				}
				else
				{
					// Otherwise let's just get back to normal.

					AI.SetDrivingMode(EVehicleAIDrivingMode::GeneralManeuvering);
				}
			}
		}
	}
}

/**
* Determine if the vehicle has reoriented correctly and switch to a new driving mode
* if so.
***********************************************************************************/

void ABaseVehicle::AIUpdateReversingToReorient(const FVector& movementPerSecond, const FVector& direction, const FVector& heading)
{
	// If we're done reversing, then head back to general maneuvering.

	if (AI.TimeInDrivingMode() > 3.0f ||
		AIMovementPossible() == false ||
		AIAreWeStuck(movementPerSecond, true) == true ||
		FVector::DotProduct(direction, heading) > 0.25f)
	{
		AI.SetDrivingMode(EVehicleAIDrivingMode::GeneralManeuvering);
	}
}

/**
* Determine if the vehicle has reversed away from a blockage and switch to a new
* driving mode if so.
***********************************************************************************/

void ABaseVehicle::AIUpdateReversingFromBlockage(const FVector& movementPerSecond)
{
	// If we're done reversing, then head back to general maneuvering.

	if (AI.TimeInDrivingMode() > 3.0f ||
		AIMovementPossible() == false ||
		AIAreWeStuck(movementPerSecond, true) == true ||
		AI.DistanceInDrivingMode() > 8.0f * 100.0f)
	{
		AI.SetDrivingMode(EVehicleAIDrivingMode::GeneralManeuvering);
	}
}

/**
* Determine if the vehicle has launched to the correct direction and switch to a new
* driving mode if so.
***********************************************************************************/

void ABaseVehicle::AIUpdateLaunchToReorient(const FVector& direction, const FVector& heading)
{
	float angleAway = FMathEx::DotProductToDegrees(FVector::DotProduct(direction, heading));
	float splineAngleAway = FMathEx::DotProductToDegrees(FVector::DotProduct(direction, AI.SplineWorldDirection));

	if ((angleAway > 125.0f || splineAngleAway > 125.0f) &&
		LaunchCharging == ELaunchStage::Charging)
	{
		if (LaunchTimer >= 1.0f &&
			IsPracticallyGrounded() == true)
		{
			// Perform the launch as the conditions are now met.

			LaunchChargeOff(true);

			// Kick us back into normal driving.

			AI.SetDrivingMode(EVehicleAIDrivingMode::GeneralManeuvering);
		}
	}
	else
	{
		// Cancel the launch as we're no longer good for it.

		LaunchChargeCancel(true);

		// Kick us into recovery control as we're now out of orientation.

		AI.SetDrivingMode(EVehicleAIDrivingMode::RecoveringControl);
	}
}

/**
* Update the J turn maneuver and determine if the vehicle has reoriented to the
* correct direction and switch to a new driving mode if so.
***********************************************************************************/

void ABaseVehicle::AIUpdateJTurnToReorient(const FVector& direction, const FVector& heading)
{
	float angleAway = FMathEx::DotProductToDegrees(FVector::DotProduct(direction, heading));

	if (AI.DrivingModeTime > 5.0f)
	{
		// It looks like this hasn't worked, too much time has passed and we've not
		// completed the maneuver.

#pragma region VehicleLaunch

		if (angleAway > 120.0f)
		{
			// So we can either launch to reorient instead if we're still not facing
			// anything like the correct direction, or ...

			AI.SetDrivingMode(EVehicleAIDrivingMode::LaunchToReorient);
		}
		else

#pragma endregion VehicleLaunch

		{
			// We enter recovering control as it looks like we need it.

			AI.SetDrivingMode(EVehicleAIDrivingMode::RecoveringControl);
		}
	}
	else
	{
		switch (AI.ReorientationStage)
		{
		case 0:
			if ((GetSpeedKPH() >= GetGearSpeedRange() * 1.6f) ||
				(AI.DrivingModeTime > 2.5f && GetSpeedKPH() >= GetGearSpeedRange() * 1.5f))
			{
				// We're now going fast enough in reverse to be able to kick the front end around.

				AI.ReorientationStage = 1;
			}
			break;
		case 1:
			if (angleAway < 120.0f ||
				FMath::Abs(Control.SteeringPosition) >= 1.0f - KINDA_SMALL_NUMBER)
			{
				// We've steered around enough to create enough inertia on the front end
				// so we can now apply the handbrake to follow it through.

				AI.ReorientationStage = 2;
			}
			break;
		case 2:
			if (angleAway < 45.0f ||
				GetSpeedKPH() < 50.0f)
			{
				// We're more or less pointing in the right direction or don't have enough
				// speed to complete the maneuver. But either way, switch back into
				// general maneuvering.

				AI.SetDrivingMode(EVehicleAIDrivingMode::GeneralManeuvering);
			}
			break;
		}
	}
}

/**
* Is the vehicle stuck and should we reverse direction to try to get out of it.
***********************************************************************************/

bool ABaseVehicle::AIAreWeStuck(const FVector& movementPerSecond, bool reversing)
{
	float halfSecond = VehicleClock - 0.5f;
	float oneSecond = VehicleClock - 1.0f;
	float twoSeconds = VehicleClock - 2.0f;

	bool c0 = RaceState.RaceTime > 5.0f; // We're into the event and not messing around on the start line
	bool c2 = AI.Thrust.TimeRange() >= 2.0f; // We have at least two seconds of thrust data to look at

	if (reversing == true)
	{
		// Quick reaction for blocked reverse movement.

		float movementThreshold = 0.1f * 100.0f;

		bool c1 = AI.Thrust.GetMeanValue(halfSecond) < -0.25f; // We've been trying to thrust backwards for the last half second
		bool c4 = AI.BackwardSpeed.GetMeanValue(halfSecond) < movementThreshold; // We've not really moved backwards at all
		bool c7 = (AI.CollisionBlockage & VehicleBlockedRear) != 0;

		if (c0 && c1 && c2 && c4 && c7)
		{
			// Find nearest to current lap distance.

			AIResetSplineFollowing(false);
			AI.SetDrivingMode(EVehicleAIDrivingMode::GeneralManeuvering);

			return true;
		}
	}
	else
	{
		// Quick reaction for blocked forward movement.

		float movementThreshold = 0.1f * 100.0f;

		bool c1 = AI.Thrust.GetMeanValue(halfSecond) > 0.25f; // We've been trying to thrust forwards for the last half second
		bool c4 = AI.ForwardSpeed.GetMeanValue(halfSecond) < movementThreshold; // We've not really moved forwards at all
		bool c7 = (AI.CollisionBlockage & VehicleBlockedFront) != 0;

		if (c0 && c1 && c2 && c4 && c7)
		{
			// Find nearest to current lap distance.

			AIResetSplineFollowing(false);
			AI.SetDrivingMode(EVehicleAIDrivingMode::ReversingFromBlockage);

			return true;
		}
		else
		{
			movementThreshold = 1.0f * 100.0f;

			c1 = AI.Thrust.GetMeanValue(oneSecond) > 0.25f; // We've been trying to thrust forwards for the last second
			c4 = AI.ForwardSpeed.GetMeanValue(twoSeconds) < movementThreshold;

			bool c3 = movementPerSecond.Size() < movementThreshold; // We've not moved the distance required at all
			bool c5 = AI.TimeSince(EVehicleAIDrivingMode::ReversingFromBlockage, VehicleClock) > 2.0f;
			bool c6 = AI.TimeSince(EVehicleAIDrivingMode::ReversingToReorient, VehicleClock) > 2.0f;

			if (AIMovementPossible() == false ||
				(c0 && c1 && c2 && c3 && c4 && c5 && c6))
			{
				// Find nearest to current lap distance.

				AIResetSplineFollowing(false);
				AI.SetDrivingMode(EVehicleAIDrivingMode::ReversingFromBlockage);

				return true;
			}
		}
	}

	return false;
}

/**
* Have we lost control?
***********************************************************************************/

void ABaseVehicle::AIHaveWeLostControl(const FVector& direction, const FVector& heading)
{
	if ((FMath::Abs(Physics.VelocityData.AngularVelocity.Z) > 100.0f) ||
		(FVector::DotProduct(direction, heading) < 0.25f && GetSpeedKPH() > 50.0f))
	{
		// If we're flat-spinning more than 100 degrees a second then recover control.
		// Or if we're pointing the wrong way then recover control.

		if (AI.TimeSince(EVehicleAIDrivingMode::RecoveringControl, VehicleClock) > 5.0f)
		{
			// But only if we've not been recovering control for the last 5 seconds
			// do we try to recover control again.

			AI.SetDrivingMode(EVehicleAIDrivingMode::RecoveringControl);
		}
	}
}

/**
* Given all the current state, update the control inputs to the vehicle to achieve
* the desired goals.
***********************************************************************************/

void ABaseVehicle::AICalculateControlInputs(const FTransform& transform, const FVector& location, const FVector& direction, const FVector& movementPerSecond, float deltaSeconds)
{
	const bool gameStartedForThisVehicle = PlayGameMode->PastGameSequenceStart();

	bool handbrake = false;
	float throttle = 0.0f;

#pragma region AIVehicleRollControl

	float rollControlSteering = AICalculateRollControlInputs(transform, deltaSeconds);

#pragma endregion AIVehicleRollControl

	if (AI.DrivingMode == EVehicleAIDrivingMode::JTurnToReorient)
	{
		throttle = -1.0f;
		handbrake = AI.ReorientationStage == 2;
	}
	else if (AI.DrivingMode == EVehicleAIDrivingMode::ReversingToReorient ||
		AI.DrivingMode == EVehicleAIDrivingMode::ReversingFromBlockage ||
		AI.DrivingMode == EVehicleAIDrivingMode::LaunchToReorient)
	{
		// If we're reversing, then apply full reverse power.

		throttle = -1.0f;
	}
	else if (AI.DrivingMode == EVehicleAIDrivingMode::GeneralManeuvering ||
		AI.DrivingMode == EVehicleAIDrivingMode::RecoveringControl)
	{
		// Now we need to do something real clever - speed matching.

		if (AI.OptimumSpeed < 0.01f)
		{
			// If we have no speed to follow then full throttle.

			throttle = 1.0f;
		}
		else
		{
			// First, decide if we need some braking.

			if (AI.DrivingMode == EVehicleAIDrivingMode::RecoveringControl)
			{
				throttle = 0.0f;
				handbrake = true;
			}
			else
			{
				// Calculate the throttle required, reverse if necessary, to achieve the desired speed.

				throttle = AICalculateThrottleForSpeed(direction, FMathEx::KilometersPerHourToCentimetersPerSecond(AI.OptimumSpeed));
			}
		}

		if (PlayGameMode->PastGameSequenceStart() == false)
		{
			handbrake = true;
		}

		if (AI.FishtailRecovery != 0.0f)
		{
			if (AI.Fishtailing == true)
			{
				throttle *= ((1.0f - FMath::Pow(AI.FishtailRecovery, 2.0f)) * 0.5f) + 0.5f;
			}
		}

		if (throttle >= -0.25f)
		{
			// If we're doing just regular maneuvering then see if some drifting may help things.

			AIUpdateDrifting(location, direction);
		}
	}

	// The AI bots rev their engines on the start line, and this code manages all that.

	AI.UpdateRevving(deltaSeconds, IsPowerAvailable());

	// Emergency stop for all AI bots for game testing.

	if (PlayGameMode != nullptr &&
		PlayGameMode->StopWhatYouDoing == true)
	{
		handbrake = true;
		throttle = 0.0f;
	}

	if (IsPowerAvailable() == false)
	{
		// If no power available to the bot yet, because the game hasn't started, just rev the engine.

		Throttle(AI.TorqueRoll, true);
	}
	else if (RaceState.RaceTime > AI.StartDelay)
	{
		// Otherwise, apply the throttle if we've passed our random start delay for this vehicle.

		Throttle(throttle, true);
	}

	// Handle the handbrake.

	if (handbrake == true)
	{
		HandbrakePressed(true);
	}
	else
	{
		HandbrakeReleased(true);
	}

	float steer = 0.0f;
	FVector localDirection = transform.InverseTransformPosition(AI.HeadingTo); localDirection.Normalize();

	if (AI.DrivingMode == EVehicleAIDrivingMode::LaunchToReorient ||
		AI.DrivingMode == EVehicleAIDrivingMode::JTurnToReorient)
	{
		localDirection *= -1.0f;
	}

	// NOTE: This looks arbitrary, but works well. Doing it properly related to steering
	// setup can produce harsh movements and loss of control. It just works better like this.
	// As currently setup, it uses almost all of the available steering at low speed.

	steer = FMath::Atan2(localDirection.Y, localDirection.X) / PI * 8.0f;

	if (IsFlipped() == true)
	{
		// Flip the steering if the vehicle is flipped.

		steer *= -1.0f;
	}

	// If we're reversing, invert the steering.

	if (Control.ThrottleInput < 0.0f &&
		FVector::DotProduct(direction, Physics.VelocityData.VelocityDirection) < 0.0f)
	{
		steer *= -1.0f;
	}

	// Mess with the steering if we're on the start line.

	if (AI.WheelplayCycles != 0.0f &&
		gameStartedForThisVehicle == false)
	{
		float cyclesPerSecond = 5.0f;
		float time = VehicleClock - AI.WheelplayStartTime;

		if (time > 0.0f &&
			time < AI.WheelplayCycles / cyclesPerSecond)
		{
			steer = FMath::Sin((PI * 0.5f * time) * cyclesPerSecond) * 0.8f;
		}
	}

#pragma region AIVehicleRollControl

	if (rollControlSteering != GRIP_UNSPECIFIED_CONTROLLER_INPUT)
	{
		steer = rollControlSteering;
	}

#pragma endregion AIVehicleRollControl

	// Setup and handle the J turn steering, for in the center of the turn when
	// on full steering lock.

	if (AI.DrivingMode == EVehicleAIDrivingMode::JTurnToReorient)
	{
		if (AI.ReorientationStage == 0)
		{
			AI.ReorientationDirection = FMathEx::UnitSign(steer);
		}
		else
		{
			steer = AI.ReorientationDirection;
		}
	}

	// Now set the desired steering into the driver controls.

	Steering(steer, true, true);
}

/**
* Calculate the throttle required, reverse if necessary, to achieve the desired
* speed. Target speed is in centimeters per second.
***********************************************************************************/

float ABaseVehicle::AICalculateThrottleForSpeed(const FVector& xdirection, float targetSpeed)
{
	// Perform all calculations in centimeter units, over 1 second of time.
	// Full throttle by default, unless overridden later.

	float throttle = 1.0f;
	FVector velocityDirection = GetVelocityOrFacingDirection();
	FVector gravity = FVector(0.0f, 0.0f, -Physics.GravityStrength) * (1.0f / Physics.CurrentMass);
	FVector drag = GetDragForceFor(velocityDirection * targetSpeed);
	FVector resistance = GetRollingResistanceForceFor(targetSpeed, velocityDirection, xdirection);

	// Now we have all the main forces that degrade speed (engine power), so sum
	// them against the velocity vector of the vehicle.

	FVector total = drag + gravity + resistance;
	FVector totalNormalized = total; totalNormalized.Normalize();

	total *= -FVector::DotProduct(totalNormalized, velocityDirection);

	// total is now the force required simply to counteract the other forces to
	// maintain the target speed, assuming we were at it already.

	// Get the total engine power here, piston and jet engine.

	float enginePower = GetJetEnginePower(2, xdirection);

	// Hopefully, the engine power will exceed the total forces acting against it.
	// If it doesn't, it means we're asking for more speed than the vehicle is
	// capable of.

	// Calculate the throttle position required to achieve that engine power.

	float targetThrottle = (total.Size() / enginePower);

	// Clamp the throttle in case target speed is exceeded.

	targetThrottle = FMath::Min(targetThrottle, 1.0f);

	float speed = GetSpeed();
	float mergeRange = FMathEx::KilometersPerHourToCentimetersPerSecond(50.0f);
	float minSpeed = FMath::Max(0.0f, targetSpeed - mergeRange);

	if (speed > targetSpeed)
	{
		// If we're already faster than the target speed then set the throttle
		// level to that required to maintain target speed and it will slowly
		// come down to meet it (due to drag). We assist it though by apply some
		// reverse throttle if much more than the target speed.

		float maxSpeed = targetSpeed + mergeRange;
		float ratio = FMathEx::GetRatio(speed, targetSpeed, maxSpeed);

		throttle = FMath::Lerp(targetThrottle, -1.0f, ratio);
	}
	else if (speed > minSpeed)
	{
		// We're nearing the target, so calculate a ratio between full
		// and target throttle. The ratio is cubed (because drag is squared)
		// and we end up getting there quickly while slowing up acceleration
		// towards the end.

		float ratio = (speed - minSpeed) / (targetSpeed - minSpeed);

		throttle = FMath::Lerp(1.0f, targetThrottle, ratio * ratio * ratio);
	}

	return throttle;
}

/**
* Record vehicle progress, backwards and forwards, throttle settings and other data
* that we can use later in AI bot decision making.
***********************************************************************************/

void ABaseVehicle::AIRecordVehicleProgress(const FTransform& transform, const FVector& movement, const FVector& direction, float deltaSeconds)
{
	float vehicleClock = VehicleClock;

	// Record our thrust request so we can compare it with distance traveled later.

	AI.Thrust.AddValue(vehicleClock, Propulsion.JetEngineThrottle);

	// Record our distance traveled.

	float movementSize = movement.Size();
	float dot = FVector::DotProduct(direction, movement);

	AI.Speed.AddValue(vehicleClock, GetSpeedMPS() * 100.0f);

	if (dot >= 0.0f)
	{
		// Going forwards.

		if (Propulsion.PistonEngineThrottle > 0.0f)
		{
			AI.DrivingModeDistance += movementSize;
		}

		AI.ForwardSpeed.AddValue(vehicleClock, (movementSize / deltaSeconds) * dot);
		AI.BackwardSpeed.AddValue(vehicleClock, 0.0f);
		AI.ForwardDistanceTraveled.AddValue(vehicleClock, movementSize);
		AI.BackwardDistanceTraveled.AddValue(vehicleClock, 0.0f);
	}
	else
	{
		// Going backwards.

		if (Propulsion.PistonEngineThrottle < 0.0f)
		{
			AI.DrivingModeDistance += movementSize;
		}

		AI.ForwardSpeed.AddValue(vehicleClock, 0.0f);
		AI.BackwardSpeed.AddValue(vehicleClock, (movementSize / deltaSeconds) * dot);
		AI.BackwardDistanceTraveled.AddValue(vehicleClock, movementSize);
		AI.ForwardDistanceTraveled.AddValue(vehicleClock, 0.0f);
	}

	FVector localVelocity = transform.InverseTransformVector(GetVelocityOrFacingDirection());

	AI.YawDirectionVsVelocity.AddValue(vehicleClock, localVelocity.Rotation().Yaw);

	AI.RaceDistances.AddValue(vehicleClock, RaceState.RaceDistance);

	AI.FacingDirectionValid.AddValue(vehicleClock, (ShouldTurnLeft() == true || ShouldTurnRight() == true) ? 0.0f : 1.0f);

	// Clear out old data.

	AI.ForwardDistanceTraveled.Clear(vehicleClock - 21.0f);
	AI.BackwardDistanceTraveled.Clear(vehicleClock - 21.0f);

	AI.Thrust.Clear(vehicleClock - 21.0f);
	AI.RaceDistances.Clear(vehicleClock - 21.0f);
	AI.FacingDirectionValid.Clear(vehicleClock - 21.0f);

	if (GetSpeedKPH() < 50.0f)
	{
		AI.YawDirectionVsVelocity.Clear();
	}

	// Update the calculation of fishtailing.

	AIUpdateFishTailing(deltaSeconds);
}

/**
* Update the vehicle fishtailing.
***********************************************************************************/

void ABaseVehicle::AIUpdateFishTailing(float deltaSeconds)
{
	bool fishtailing = false;

	if (IsGrounded(2.0f) == true &&
		GetSpeedKPH() > 150.0f)
	{
		if (AI.YawDirectionVsVelocity.TimeRange() >= 3.0f)
		{
			int32 numValues = AI.YawDirectionVsVelocity.GetNumValues();
			float lastSide = 0.0f;
			float lastSideTime = 0.0f;
			int32 numSwitches = 0;
			float lastTimeAdded = AI.YawDirectionVsVelocity.GetLastTime();
			float timeLimit = (AI.Fishtailing == true) ? 1.0f : 3.0f;

			for (int32 i = numValues - 1; i >= 0; i--)
			{
				float time = AI.YawDirectionVsVelocity[i].Time;

				if (lastTimeAdded - time < timeLimit)
				{
					float yaw = AI.YawDirectionVsVelocity[i].Value;

					if (AI.Fishtailing == true)
					{
						// Detect finished fishtailing state.

						if (FMath::Abs(yaw) > 5.0f)
						{
							fishtailing = true;
							break;
						}
					}
					else
					{
						// Detect fishtailing state.

						if (FMath::Abs(yaw) > 10.0f)
						{
							float side = FMathEx::UnitSign(yaw);

							if (lastSide != side)
							{
								if (lastSide != 0.0f)
								{
									if (time - lastSideTime < 2.0f)
									{
										numSwitches++;
									}
									else
									{
										numSwitches = 0;
									}
								}
								else
								{
									numSwitches++;
								}

								lastSide = side;
								lastSideTime = time;
							}
						}
					}
				}
				else
				{
					break;
				}
			}

			if (AI.Fishtailing == false)
			{
				// If the back-end has pendulumed at least twice then we're fishtailing.

				fishtailing = numSwitches >= 2;
			}
		}
	}

	if (fishtailing == true)
	{
		if (AI.Fishtailing == false)
		{
			AI.FishtailingOnTime = 0.0f;
		}

		AI.FishtailingOnTime += deltaSeconds;
		AI.FishtailRecovery = FMathEx::GravitateToTarget(AI.FishtailRecovery, 1.0f, deltaSeconds * 2.0f);
	}
	else
	{
		if (AI.Fishtailing == true)
		{
			AI.FishtailingOffTime = 0.0f;
			AI.YawDirectionVsVelocity.Clear();
		}

		AI.FishtailingOffTime += deltaSeconds;
		AI.FishtailRecovery = FMathEx::GravitateToTarget(AI.FishtailRecovery, 0.0f, deltaSeconds);
	}

	AI.Fishtailing = fishtailing;
}

/**
* Is movement of the vehicle possible or is it stuck unable to move in the desired
* direction?
***********************************************************************************/

bool ABaseVehicle::AIMovementPossible() const
{
	// Find the distance traveled in the last two seconds.

	if (RaceState.RaceTime > 5.0f &&
		AI.TimeInDrivingMode() > 3.0f &&
		AI.Thrust.GetAbsMeanValue() > 0.75f)
	{
		float forward = AI.ForwardDistanceTraveled.GetSumValue(VehicleClock - 2.0f);
		float backward = AI.BackwardDistanceTraveled.GetSumValue(VehicleClock - 2.0f);

		if (forward + backward < 100.0f)
		{
			return false;
		}
	}

	return true;
}

#pragma endregion AIVehicleControl

#pragma region AIVehicleRollControl

/**
* Given all the current state, update the airborne roll control inputs to the
* vehicle to achieve the desired goals.
***********************************************************************************/

float ABaseVehicle::AICalculateRollControlInputs(const FTransform& transform, float deltaSeconds)
{
	bool rollTargetDetected = false;
	bool rollControlPossiblyRequired = (IsAirborne() == true && IsPracticallyGrounded(3.0f * 100.0f) == false);
	float relativeRollTarget = 0.0f;
	float rollTargetTime = 0.0f;

	if (rollControlPossiblyRequired == true)
	{
		float rollTargetTimeTest = 3.0f;
		FVector endPoint = AI.LastLocation + Physics.VelocityData.Velocity * rollTargetTimeTest;

		if (AI.RollControlTime != 0.0f &&
			Clock0p1.ShouldTickNow() == false)
		{
			// Don't do a line trace every frame, we can reuse the data from the last line
			// trace for a few frames at least.

			rollTargetDetected = AI.RollTargetDetected;

			if (rollTargetDetected == true)
			{
				AI.RollControlTime = FMath::Max(0.0f, AI.RollControlTime - (deltaSeconds * rollTargetTimeTest));

				// Get the last ground surface normal we detected and bring it into
				// local, vehicle space.

				FVector normal = transform.InverseTransformVector(AI.RollControlNormal);

				// We now have the normal vector in 2D YZ on the vehicle's local space.

				relativeRollTarget = FMath::RadiansToDegrees(FMath::Atan2(normal.Y, normal.Z));
				rollTargetTime = AI.RollControlTime;
			}
		}
		else
		{
			FHitResult hit;

			QueryParams.bReturnPhysicalMaterial = true;
			QueryParams.ClearIgnoredActors();
			QueryParams.AddIgnoredActor(this);

			if (GetWorld()->LineTraceSingleByChannel(hit, AI.LastLocation, endPoint, ABaseGameMode::ECC_LineOfSightTest, QueryParams) == true)
			{
				AI.RollControlSurfaceType = (EGameSurface)UGameplayStatics::GetSurfaceType(hit);

				if (AI.RollControlSurfaceType != EGameSurface::Field &&
					AI.RollControlSurfaceType != EGameSurface::Tractionless)
				{
					// Record the impact point and normal in world space so we can reuse it when estimating
					// for a few frames rather than calling LineTraceSingleByChannel every frame.

					AI.RollControlNormal = hit.ImpactNormal;
					AI.RollControlLocation = hit.ImpactPoint;

					rollTargetDetected = true;

					// Get the last ground surface normal we detected and bring it into
					// local, vehicle space.

					FVector normal = transform.InverseTransformVector(AI.RollControlNormal);

					// We now have the normal vector in 2D YZ on the vehicle's local space.

					relativeRollTarget = FMath::RadiansToDegrees(FMath::Atan2(normal.Y, normal.Z));
					AI.RollControlTime = rollTargetTime = ((hit.ImpactPoint - AI.LastLocation).Size() / (endPoint - AI.LastLocation).Size()) * rollTargetTimeTest;
				}
			}

			AI.RollTargetDetected = rollTargetDetected;
		}
	}
	else
	{
		AI.RollControlTime = 0.0f;
	}

	if (rollTargetDetected == true &&
		rollControlPossiblyRequired == true)
	{
		float rollOffsetRequiresCorrection = 10.0f;

		if ((FMath::Abs(Physics.VelocityData.AngularVelocity.X) > AI.RollVelocityRequiresDamping) ||
			(FMath::Abs(relativeRollTarget) > rollOffsetRequiresCorrection && FMath::Abs(relativeRollTarget) < 180.0f - rollOffsetRequiresCorrection))
		{
			Propulsion.ThrottleOffWhileAirborne = true;
		}
	}

	// If we're airborne and we've initiated air control, then use roll control to fly
	// the ship down. Assume a flat zero roll landing for now as this is almost
	// certainly to be the case.

	float steerOutput = GRIP_UNSPECIFIED_CONTROLLER_INPUT;

	if (rollTargetDetected == true &&
		rollControlPossiblyRequired == true &&
		Propulsion.ThrottleOffWhileAirborne == true)
	{
		AIPerformRollControl(relativeRollTarget, rollTargetTime, steerOutput, AI.AirborneRollControl);
	}
	else
	{
		AI.AirborneRollControl = ERollControlStage::Inactive;
	}

	return steerOutput;
}

/**
* Perform the control required to match the target roll.
***********************************************************************************/

void ABaseVehicle::AIPerformRollControl(float relativeRollTarget, float rollTargetTime, float& steer, ERollControlStage& rollControl) const
{
	if (rollControl == ERollControlStage::Inactive)
	{
		// Check the current angular velocity and see if the correction we need to make
		// correlates to that.

		if (FMath::Abs(Physics.VelocityData.AngularVelocity.X) > AI.RollVelocityRequiresDamping)
		{
			rollControl = ERollControlStage::Damping;
		}
		else
		{
			rollControl = ERollControlStage::Rolling;
		}
	}

	if (rollControl == ERollControlStage::Damping)
	{
		// Damp the roll to something we can use.

		float predictedRoll = FMath::Abs(FRotator::NormalizeAxis((Physics.VelocityData.AngularVelocity.X * rollTargetTime) - relativeRollTarget));

		if ((rollTargetTime > 0.0f) &&
			(predictedRoll < 10.0f || predictedRoll > 170.0f))
		{
			steer = 0.0f;
		}
		else
		{
			if (FMath::Abs(Physics.VelocityData.AngularVelocity.X) <= AI.RollVelocityRequiresDamping)
			{
				rollControl = ERollControlStage::Rolling;
			}
			else
			{
				steer = (Physics.VelocityData.AngularVelocity.X < 0.5f) ? -1.0f : 1.0f;
			}
		}
	}

	if (rollControl == ERollControlStage::Rolling)
	{
		if (rollTargetTime <= 0.0f)
		{
			if (FMath::Abs(relativeRollTarget) < 90.0f)
			{
				// Roll to regular up.

				steer = FMathEx::GetRatio(FMath::Abs(relativeRollTarget), 1.0f, 20.0f) * 0.5f + 0.25f;
				steer = (relativeRollTarget > 0.0f) ? steer : -steer;
			}
			else
			{
				// Roll to inverted up as it's closer.

				steer = FMathEx::GetRatio(180.0f - FMath::Abs(relativeRollTarget), 1.0f, 20.0f) * 0.5f + 0.25f;
				steer = (relativeRollTarget > 0.0f) ? -steer : steer;
			}
		}
		else
		{
			if (FMath::Abs(relativeRollTarget) < 90.0f)
			{
				// Roll to regular up.

				steer = FMathEx::GetRatio(FMath::Abs(relativeRollTarget), 20.0f, 50.0f) * 0.5f + 0.5f;
				steer = (relativeRollTarget > 0.0f) ? steer : -steer;
			}
			else
			{
				// Roll to inverted up as it's closer.

				steer = FMathEx::GetRatio(180.0f - FMath::Abs(relativeRollTarget), 20.0f, 50.0f) * 0.5f + 0.5f;
				steer = (relativeRollTarget > 0.0f) ? -steer : steer;
			}
		}
	}
}

#pragma endregion AIVehicleRollControl
