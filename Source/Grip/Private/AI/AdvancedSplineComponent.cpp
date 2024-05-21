/**
*
* Advanced spline components.
*
* Original author: Rob Baker.
* Current maintainer: Rob Baker.
*
* Copyright Caged Element Inc, code provided for educational purposes only.
*
* Spline components with extended functionality over USplineComponent but not really
* much in the way of new properties. It performs some extended geometry analysis
* on splines, including GetNearestDistance which returns the nearest position on a
* spline for a given position in space.
*
***********************************************************************************/

#include "ai/advancedsplinecomponent.h"
#include "system/mathhelpers.h"

/**
* Construct an advanced spline component.
***********************************************************************************/

UAdvancedSplineComponent::UAdvancedSplineComponent()
{
	UPrimitiveComponent::SetCollisionEnabled(ECollisionEnabled::Type::NoCollision);
	UPrimitiveComponent::SetCollisionProfileName(UCollisionProfile::NoCollision_ProfileName);

	SetGenerateOverlapEvents(false);

	Mobility = EComponentMobility::Movable;

	// Grab the actor's name and store it locally for easier diagnostic work.

	AActor* actor = GetOwner();

	if (actor != nullptr)
	{
		ActorName = actor->GetName();
	}
}

#pragma region NavigationSplines

/**
* Post initialize the component.
***********************************************************************************/

void UAdvancedSplineComponent::PostInitialize()
{
	// Ensure we have high accuracy in determining distance along the spline.

	if (ReparamStepsPerSegment != 100)
	{
		ReparamStepsPerSegment = 100;

		UpdateSpline();
	}

	CalculateSections();
}

/**
* Find the nearest distance along a spline to a given world location.
* The fewer iterations and samples you use the faster it will be, but also the less
* accurate it will be. Conversely, the smaller the difference between startDistance
* and endDistance the more accurate the result will be.
***********************************************************************************/

float UAdvancedSplineComponent::GetNearestDistance(FVector location, float startDistance, float endDistance, int32 numIterations, int32 numSamples, float earlyExitDistance) const
{
	// This is a relatively slow iterative method, but it works solidly. I tried a couple of analytical
	// methods, which worked a lot of the time, but didn't always, which was frustrating.

	float splineLength = GetSplineLength();

	if (endDistance <= 0.0f)
	{
		endDistance = splineLength;
	}

	if (numIterations <= 0)
	{
		numIterations = 5;
	}

	float minDistance = startDistance;
	float maxDistance = endDistance;
	float minDistanceAway = -1.0f;
	float resultDistance = minDistance;
	float invNumSamples = 1.0f / (float)numSamples;

	// Bring the world location supplied into local space for faster comparison against
	// points on the spline.

	location = GetComponentTransform().InverseTransformPosition(location);

	for (int32 iteration = 0; iteration < numIterations; iteration++)
	{
		float distanceAlong = minDistance;
		float deltaStep = (maxDistance - minDistance) * invNumSamples;
		float lastResultDistance = resultDistance;

		// This will sample between minDistance and maxDistance inclusively.

		for (int32 sample = 0; sample <= numSamples; sample++)
		{
			// Determine the test position on the spline for distanceAlong. Functionally equivalent
			// to GetLocationAtDistanceAlongSpline, but slightly faster.

			float clampedDistanceAlong = ClampDistanceAgainstLength(distanceAlong, splineLength);
			float inputKey = SplineCurves.ReparamTable.Eval(clampedDistanceAlong, 0.0f);
			FVector testPosition = SplineCurves.Position.Eval(inputKey, FVector::ZeroVector);

			// Test against size squared because it's much faster than size.

			float distanceAway = (location - testPosition).SizeSquared();

			if (minDistanceAway == -1.0f ||
				minDistanceAway > distanceAway)
			{
				// If the minimum distanceAway was less than the last then record it.

				minDistanceAway = distanceAway;
				resultDistance = clampedDistanceAlong;
			}

			distanceAlong += deltaStep;
		}

		if (iteration > 0 &&
			deltaStep < earlyExitDistance * 2.0f &&
			GetDistanceDifference(resultDistance, lastResultDistance) < earlyExitDistance)
		{
			// Early break if the last refinement only took us less than a set distance away from the last.

			break;
		}

		minDistance = resultDistance - deltaStep;
		maxDistance = resultDistance + deltaStep;
	}

	return resultDistance;
}

/**
* Find the nearest distance along a spline to a given plane location and direction.
* The fewer iterations and samples you use the faster it will be, but also the less
* accurate it will be. Conversely, the smaller the difference between startDistance
* and endDistance the more accurate the result will be.
***********************************************************************************/

float UAdvancedSplineComponent::GetNearestDistance(FVector planeLocation, FVector planeDirection, float startDistance, float endDistance, int32 numIterations, int32 numSamples, float earlyExitDistance) const
{
	// This is a relatively slow iterative method, but it works solidly. I tried a couple of analytical
	// methods, which worked a lot of the time, but didn't always, which was frustrating.

	float splineLength = GetSplineLength();

	if (endDistance <= 0.0f)
	{
		endDistance = splineLength;
	}

	if (numIterations <= 0)
	{
		numIterations = 5;
	}

	float minDistance = startDistance;
	float maxDistance = endDistance;
	float minDistanceAway = -1.0f;
	float resultDistance = minDistance;
	float invNumSamples = 1.0f / (float)numSamples;

	// Bring the plane location and direction supplied into local space for faster comparison against
	// points on the spline.

	planeLocation = GetComponentTransform().InverseTransformPosition(planeLocation);
	planeDirection = GetComponentTransform().InverseTransformVector(planeDirection); planeDirection.Normalize();

	for (int32 iteration = 0; iteration < numIterations; iteration++)
	{
		float distanceAlong = minDistance;
		float deltaStep = (maxDistance - minDistance) * invNumSamples;
		float lastResultDistance = resultDistance;

		// This will sample between minDistance and maxDistance inclusively.

		for (int32 sample = 0; sample <= numSamples; sample++)
		{
			// Determine the test position on the spline for distanceAlong. Functionally equivalent
			// to GetLocationAtDistanceAlongSpline, but slightly faster.

			float clampedDistanceAlong = ClampDistanceAgainstLength(distanceAlong, splineLength);
			float inputKey = SplineCurves.ReparamTable.Eval(clampedDistanceAlong, 0.0f);
			FVector testPosition = SplineCurves.Position.Eval(inputKey, FVector::ZeroVector);

			// Test against size squared because it's much faster than size.

			float distanceAway = FMath::Abs(FVector::PointPlaneDist(testPosition, planeLocation, planeDirection));

			if (minDistanceAway == -1.0f ||
				minDistanceAway > distanceAway)
			{
				// If the minimum distanceAway was less than the last then record it.

				minDistanceAway = distanceAway;
				resultDistance = clampedDistanceAlong;
			}

			distanceAlong += deltaStep;
		}

		if (iteration > 0 &&
			deltaStep < earlyExitDistance * 2.0f &&
			GetDistanceDifference(resultDistance, lastResultDistance) < earlyExitDistance)
		{
			// Early break if the last refinement only took us less than a set distance away from the last.

			break;
		}

		minDistance = resultDistance - deltaStep;
		maxDistance = resultDistance + deltaStep;
	}

	return resultDistance;
}

/**
* Get the distance between two points on a spline (accounting for looped splines).
* Subtracting distance1 from distance0, notionally if you want an unsigned result.
***********************************************************************************/

float UAdvancedSplineComponent::GetDistanceDifference(float distance0, float distance1, float length, bool signedDifference) const
{
	float difference = distance0 - distance1;

	if (IsClosedLoop() == true)
	{
		if (length == 0.0f)
		{
			length = GetSplineLength();
		}

		float halfLength = length * 0.5f;

		if (FMath::Abs(difference) > halfLength)
		{
			if (distance0 <= halfLength &&
				distance1 >= length - halfLength)
			{
				difference = distance0 + (length - distance1);
			}
			else if (distance1 <= halfLength &&
				distance0 >= length - halfLength)
			{
				difference = -(distance1 + (length - distance0));
			}
		}
	}

	return (signedDifference == true) ? difference : FMath::Abs(difference);
}

/**
* Clamp a distance along the spline to its length if it's not looped, or wrapped
* within its length if looped.
***********************************************************************************/

float UAdvancedSplineComponent::ClampDistanceAgainstLength(float distance, float length) const
{
	if (distance < 0.0f)
	{
		distance = (IsClosedLoop() == true) ? length - FMath::Fmod(-distance, length) : 0.0f;
	}
	else if (distance > length)
	{
		distance = (IsClosedLoop() == true) ? FMath::Fmod(distance, length) : length;
	}

	return distance;
}

/**
* Get which side a world location is on with respect to its nearest point along the
* spline center-line.
***********************************************************************************/

float UAdvancedSplineComponent::GetSide(float distance, const FVector& fromLocation) const
{
	FRotator rotation = GetRotationAtDistanceAlongSpline(distance, ESplineCoordinateSpace::World);
	FVector sideVector = rotation.RotateVector(FVector(0.0f, 1.0f, 0.0f));
	FVector location = GetLocationAtDistanceAlongSpline(distance, ESplineCoordinateSpace::World);

	return (FVector::DotProduct((fromLocation - location), sideVector) >= 0.0f) ? 1.0f : -1.0f;
}

/**
* Calculate the sections of the spline.
***********************************************************************************/

void UAdvancedSplineComponent::CalculateSections()
{
}

#pragma endregion NavigationSplines
