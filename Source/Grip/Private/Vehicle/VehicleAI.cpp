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
}

/**
* Is an AI driver good for a launch?
***********************************************************************************/

bool ABaseVehicle::AIVehicleGoodForLaunch(float probability, float minimumSpeedKPH) const
{
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

		AIDetermineSplineAimPoint(ahead, movementSize);

		// So now we know where we are and where we're aiming for.

		AI.HeadingTo = AI.RouteFollower.NextSpline->GetWorldLocationAtDistanceAlongSpline(AI.RouteFollower.NextDistance);
		AI.OptimumSpeed = AI.RouteFollower.ThisSpline->GetOptimumSpeedAtDistanceAlongSpline(AI.RouteFollower.ThisDistance);
		AI.MinimumSpeed = AI.RouteFollower.ThisSpline->GetMinimumSpeedAtDistanceAlongSpline(AI.RouteFollower.ThisDistance);
		AI.TrackOptimumSpeed = AI.OptimumSpeed;

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
