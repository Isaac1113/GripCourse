/**
*
* Pursuit spline components.
*
* Original author: Rob Baker.
* Current maintainer: Rob Baker.
*
* Copyright Caged Element Inc, code provided for educational purposes only.
*
* This kind of spline is used primarily for AI bot track navigation, but also for
* cinematic camera work, weather determination and also for the Assassin missile
* navigation in the full version of the game. They're also critically important for
* race position determination.
*
***********************************************************************************/

#include "ai/pursuitsplinecomponent.h"
#include "ai/pursuitsplineactor.h"
#include "kismet/kismetmathlibrary.h"
#include "kismet/kismetmateriallibrary.h"
#include "system/mathhelpers.h"
#include "gamemodes/playgamemode.h"

DEFINE_LOG_CATEGORY(GripLogPursuitSplines);

/**
* Construct a pursuit spline component.
***********************************************************************************/

UPursuitSplineComponent::UPursuitSplineComponent()
{

#pragma region NavigationSplines

	PursuitSplineParent = Cast<APursuitSplineActor>(GetOwner());

	if (PursuitSplineParent != nullptr)
	{
		ActorName = PursuitSplineParent->GetName();
	}

#pragma endregion NavigationSplines

}

/**
* Set the spline component for this spline mesh component.
***********************************************************************************/

void UPursuitSplineMeshComponent::SetupSplineComponent(UPursuitSplineComponent* splineComponent, int32 startPoint, int32 endPoint, bool selected)
{

#pragma region NavigationSplines

	PursuitSplineComponent = splineComponent;
	PursuitSplineComponent->PursuitSplineMeshComponents.Emplace(this);

	StartPoint = startPoint;
	EndPoint = endPoint;

	SetupMaterial(selected);

#pragma endregion NavigationSplines

}

/**
* Setup the rendering material for this spline mesh component.
***********************************************************************************/

void UPursuitSplineMeshComponent::SetupMaterial(bool selected)
{

#pragma region NavigationSplines

	UStaticMesh* mesh = GetStaticMesh();

	if (mesh != nullptr)
	{
		UMaterialInterface* originalMaterial = GetMaterial(0);
		UMaterialInstanceDynamic* dynamicMaterial = Cast<UMaterialInstanceDynamic>(originalMaterial);

		// Create a dynamic material for this mesh if not already done so.

		if (dynamicMaterial == nullptr)
		{
			dynamicMaterial = UKismetMaterialLibrary::CreateDynamicMaterialInstance(nullptr, originalMaterial);

			SetMaterial(0, dynamicMaterial);
		}

		float s0 = PursuitSplineComponent->GetOptimumSpeedAtSplinePoint(StartPoint);
		float s1 = PursuitSplineComponent->GetOptimumSpeedAtSplinePoint(EndPoint);

		if (s0 == 0.0f)
		{
			s0 = 1000.0f;
		}

		if (s1 == 0.0f)
		{
			s1 = 1000.0f;
		}

		// Colorize the spline according to its optimum speed.

		s0 = FMath::Pow(FMath::Clamp(s0, 0.0f, 1000.0f) / 1000.0f, 0.5f) * 360.0f;
		s1 = FMath::Pow(FMath::Clamp(s1, 0.0f, 1000.0f) / 1000.0f, 0.5f) * 360.0f;

		FLinearColor sc0 = UKismetMathLibrary::HSVToRGB(s0, 1.0f, 0.75f, 1.0f);
		FLinearColor sc1 = UKismetMathLibrary::HSVToRGB(s1, 1.0f, 0.75f, 1.0f);

		if (PursuitSplineComponent->Type == EPursuitSplineType::MissileAssistance)
		{
			// Missile splines always rendered in white.

			sc0 = UKismetMathLibrary::HSVToRGB(s0, 0.0f, 0.5f, 1.0f);
			sc1 = UKismetMathLibrary::HSVToRGB(s1, 0.0f, 0.5f, 1.0f);
		}

		// Set all of the scalar and vector parameters on this material so that it
		// can be rendered with the correct attributes.

		dynamicMaterial->SetScalarParameterValue("Selected", (selected == true) ? 1.0f : 0.0f);
		dynamicMaterial->SetVectorParameterValue("Speed0", sc0);
		dynamicMaterial->SetVectorParameterValue("Speed1", sc1);
		dynamicMaterial->SetScalarParameterValue("Width0", PursuitSplineComponent->GetWidthAtSplinePoint(StartPoint));
		dynamicMaterial->SetScalarParameterValue("Width1", PursuitSplineComponent->GetWidthAtSplinePoint(EndPoint));
		dynamicMaterial->SetScalarParameterValue("Distance0", PursuitSplineComponent->GetDistanceAlongSplineAtSplinePoint(StartPoint) / (10.0f * 100.0f));

		if (EndPoint == 0 &&
			PursuitSplineComponent->IsClosedLoop() == true)
		{
			dynamicMaterial->SetScalarParameterValue("Distance1", PursuitSplineComponent->GetSplineLength() / (10.0f * 100.0f));
		}
		else
		{
			dynamicMaterial->SetScalarParameterValue("Distance1", PursuitSplineComponent->GetDistanceAlongSplineAtSplinePoint(EndPoint) / (10.0f * 100.0f));
		}
	}

#pragma endregion NavigationSplines

}

#pragma region NavigationSplines

static const float UnlimitedSplineDistance = 1000.0f * 100.0f;

/**
* Get the angle difference between to environment samples.
***********************************************************************************/

float FPursuitPointExtendedData::DifferenceInDegrees(int32 indexFrom, int32 indexTo)
{
	float angleFrom = indexFrom * (360.0f / NumDistances);
	float angleTo = indexTo * (360.0f / NumDistances);

	return FMath::Abs(FMathEx::GetUnsignedDegreesDifference(angleFrom, angleTo));
}

/**
* Get the average tunnel diameter over a set distance.
***********************************************************************************/

float FRouteFollower::GetTunnelDiameterOverDistance(float distance, float overDistance, int32 direction, bool minimum) const
{
	float c0 = 0.0f;
	float c1 = 0.0f;

	if (GRIP_POINTER_VALID(ThisSpline) == true)
	{
		c0 = c1 = ThisSpline->GetTunnelDiameterOverDistance(distance, overDistance, direction, minimum);
	}

	if (GRIP_POINTER_VALID(NextSpline) == true &&
		NextSpline != ThisSpline)
	{
		c1 = NextSpline->GetTunnelDiameterOverDistance(NextSwitchDistance, overDistance, direction, minimum);
	}

	if (minimum == true)
	{
		return FMath::Min(c0, c1);
	}
	else
	{
		return (c0 + c1) * 0.5f;
	}
}

/**
* Get the average tunnel diameter over a set distance.
***********************************************************************************/

float UPursuitSplineComponent::GetTunnelDiameterOverDistance(float distance, float overDistance, int32 direction, bool minimum) const
{
	if (PursuitSplineParent->PointExtendedData.Num() < 2)
	{
		return 0.0f;
	}

	float length = GetSplineLength();
	float endDistance = distance + (overDistance * direction);

	if (IsClosedLoop() == false)
	{
		endDistance = ClampDistanceAgainstLength(endDistance, length);
	}

	float averageDiameter = 0.0f;
	float iterationDistance = FMathEx::MetersToCentimeters(ExtendedPointMeters);
	int32 numIterations = FMath::CeilToInt(FMath::Abs(endDistance - distance) / iterationDistance);

	for (int32 i = 0; i <= numIterations; i++)
	{
		float diameter = GetTunnelDiameterAtDistanceAlongSpline(distance);

		if (minimum == true)
		{
			if (i == 0 ||
				averageDiameter > diameter)
			{
				averageDiameter = diameter;
			}
		}
		else
		{
			averageDiameter += diameter;
		}

		distance = ClampDistanceAgainstLength(distance + (iterationDistance * direction), length);
	}

	if (minimum == true)
	{
		return averageDiameter;
	}
	else
	{
		return averageDiameter / (numIterations + 1);
	}
}

/**
* Get the tunnel diameter at a distance along a spline.
***********************************************************************************/

float UPursuitSplineComponent::GetTunnelDiameterAtDistanceAlongSpline(float distance) const
{
	if (PursuitSplineParent->PointExtendedData.Num() < 2)
	{
		return 0.0f;
	}

	int32 thisKey = 0;
	int32 nextKey = 0;
	float ratio = 0.0f;

	GetExtendedPointKeys(distance, thisKey, nextKey, ratio);

	float v0 = PursuitSplineParent->PointExtendedData[thisKey].MaxTunnelDiameter;
	float v1 = PursuitSplineParent->PointExtendedData[nextKey].MaxTunnelDiameter;

	const float notATunnel = 100.0f * 100.0f;

	if (v0 <= 0.0f &&
		v1 <= 0.0f)
	{
		return notATunnel;
	}

	if (v0 <= 0.0f)
	{
		v0 = notATunnel;
	}

	if (v1 <= 0.0f)
	{
		v1 = notATunnel;
	}

	return FMath::Min(FMath::Lerp(v0, v1, ratio), notATunnel);
}

/**
* Is the spline and distance referenced by this link valid for a route choice
* decision?
***********************************************************************************/

bool FSplineLink::LinkIsRouteChoice() const
{
	// Either a closed loop or at least 50m left on the spline at the point we link to it
	// in order for it to be worthwhile.

	return (ForwardLink == true && (Spline->IsClosedLoop() == true || (Spline->GetSplineLength() - NextDistance) >= 50.0f * 100.0f));
}

/**
* Add a spline link to this spline component.
***********************************************************************************/

void UPursuitSplineComponent::AddSplineLink(const FSplineLink& link)
{
	if (SplineLinks.Find(link) == INDEX_NONE)
	{
		SplineLinks.Emplace(link);
	}
}

/**
* Calculate the extended point data by examining the scene around the spline.
***********************************************************************************/

void UPursuitSplineComponent::Build(bool fromMenu, bool performChecks, bool bareData, TArray<FVector>* intersectionPoints)
{
	APursuitSplineActor* owner = Cast<APursuitSplineActor>(GetAttachmentRootActor());

	if (owner != nullptr)
	{
		CalculateSections();
	}
}

/**
* Post initialize the component.
***********************************************************************************/

void UPursuitSplineComponent::PostInitialize()
{
	Build(false, false, true, nullptr);

	Super::PostInitialize();

	int32 numPoints = GetNumberOfSplinePoints();

	ensureMsgf(numPoints > 1, TEXT("Not enough points on a pursuit spline"));

	TArray<FPursuitPointExtendedData>& pursuitPointExtendedData = PursuitSplineParent->PointExtendedData;

	for (FPursuitPointExtendedData& point : pursuitPointExtendedData)
	{
		point.Quaternion = GetQuaternionAtDistanceAlongSpline(point.Distance, ESplineCoordinateSpace::World);
	}
}

/**
* Get the master distance at a distance along a spline.
***********************************************************************************/

float UPursuitSplineComponent::GetMasterDistanceAtDistanceAlongSpline(float distance, float masterSplineLength) const
{
	if (PursuitSplineParent->PointExtendedData.Num() < 2)
	{
		return 0.0f;
	}

	int32 thisKey = 0;
	int32 nextKey = 0;
	float ratio = 0.0f;

	GetExtendedPointKeys(distance, thisKey, nextKey, ratio);

	float v0 = PursuitSplineParent->PointExtendedData[thisKey].MasterSplineDistance;
	float v1 = PursuitSplineParent->PointExtendedData[nextKey].MasterSplineDistance;

	ensureMsgf(v0 != -1.0f && v1 != -1.0f, TEXT("Bad master spline distance"));

	if (v1 >= v0 ||
		masterSplineLength == 0.0f ||
		v0 - v1 < masterSplineLength * 0.25f)
	{
		// Handle the easy case of master distance interpolation.

		return FMath::Lerp(v0, v1, ratio);
	}
	else
	{
		// Need to work out the break going across the wrap here. This normally happens
		// because the master spline has wrapped, it's starting point, happens to be
		// between the two extended data points that we need to sample.

		float l0 = masterSplineLength - v0;	// end length
		float l1 = v1;						// start length
		float lt = l0 + l1;					// total length
		float l = ratio * lt;

		if (l <= l0 &&
			l0 > 0.0f)
		{
			return FMath::Lerp(v0, masterSplineLength, l / l0);
		}
		else if (l1 > 0.0f)
		{
			return FMath::Lerp(0.0f, v1, (l - l0) / l1);
		}
		else
		{
			return v1;
		}
	}
}

/**
* Get the extended point keys bounding a distance along the spline.
***********************************************************************************/

void UPursuitSplineComponent::GetExtendedPointKeys(float distance, int32& key0, int32& key1, float& ratio) const
{
	TArray<FPursuitPointExtendedData>& pursuitPointExtendedData = PursuitSplineParent->PointExtendedData;
	int32 numIndices = pursuitPointExtendedData.Num();

	if (numIndices > 1)
	{
		float length = GetSplineLength();

		distance = ClampDistanceAgainstLength(distance, length);

		// Ratio between 0 and 1 for the entire spline.

		float pointLength = length / (float)(numIndices - 1);

		ratio = distance / pointLength;

		key0 = ThisExtendedKey(pursuitPointExtendedData, ratio);
		key1 = NextExtendedKey(pursuitPointExtendedData, ratio);

		int32 attempts = 2;

		while (attempts-- > 0)
		{
			FPursuitPointExtendedData& p0 = pursuitPointExtendedData[key0];

			if (distance < p0.Distance)
			{
				key0 = BindExtendedKey(pursuitPointExtendedData, key0 - 1);
				key1 = BindExtendedKey(pursuitPointExtendedData, key1 - 1);
			}
			else if (distance - p0.Distance > pointLength * 1.5f)
			{
				key0 = BindExtendedKey(pursuitPointExtendedData, key0 + 1);
				key1 = BindExtendedKey(pursuitPointExtendedData, key1 + 1);
			}
			else
			{
				break;
			}
		}

		FPursuitPointExtendedData& p0 = pursuitPointExtendedData[key0];

		ratio = (distance - p0.Distance) / pointLength;
		ratio = FMath::Clamp(ratio, 0.0f, 1.0f);

		ensure(key0 >= 0 && key0 < numIndices);
		ensure(key1 >= 0 && key1 < numIndices);
	}
	else
	{
		ratio = 0.0f;
		key0 = key1 = 0;
	}
}

/**
* Calculate distances along the master spline for this spline and each of its links.
***********************************************************************************/

bool UPursuitSplineComponent::CalculateMasterSplineDistances(UPursuitSplineComponent* masterSpline, float masterSplineLength, float startingDistance, int32 degreesOfSeparation, bool report, int32 recalibrate, int32 recalibrationAttempt)
{
	bool reportGoodData = recalibrate == 2;

	if (recalibrate != 0 &&
		MasterDistanceClass < 2)
	{
		return reportGoodData;
	}

	// Do the calculation.

	bool result = false;
	int32 dataClass = degreesOfSeparation;

	if (recalibrate != 0 ||
		HasMasterSplineDistances() == false)
	{
		TArray<FPursuitPointExtendedData>& pursuitPointExtendedData = PursuitSplineParent->PointExtendedData;
		int32 numExtendedPoints = pursuitPointExtendedData.Num();

		if (numExtendedPoints > 0)
		{
			// UE_LOG(GripLogPursuitSplines, Log, TEXT("CalculateMasterSplineDistances for %s with %d points starting at %0.01f"), *ActorName, numExtendedPoints, startingDistance);

			if (this == masterSpline)
			{
				// Simple case, this is the master spline so just copy across the regular distances.

				if (recalibrate == 0)
				{
					for (int32 i = 0; i < numExtendedPoints; i++)
					{
						FPursuitPointExtendedData& point = pursuitPointExtendedData[i];

						point.MasterSplineDistance = point.Distance;
					}

					if (report == true)
					{
						UE_LOG(GripLogPursuitSplines, Log, TEXT("Pursuit spline %s calculated master distances with class %d data."), *ActorName, degreesOfSeparation + 1);
					}

					MasterDistanceClass = dataClass;

					result = true;
				}
				else
				{
					result = reportGoodData;
				}
			}
			else
			{
				float accuracy = 1.0f;
				float scanSpan = 16.0f;
				int32 numIterations = 5;
				float masterDistance = startingDistance;
				float movementSize = FMathEx::MetersToCentimeters(ExtendedPointMeters);
				int32 numSamples = masterSpline->GetNumSamplesForRange(movementSize * scanSpan, numIterations, accuracy);

				// UE_LOG(GripLogPursuitSplines, Log, TEXT("CalculateMasterSplineDistances numSamples is %d"), numSamples);

				bool linkedStart = false;
				bool linkedEnd = false;
				float startDistance = 0.0f;
				float endDistance = 0.0f;
				float startDistanceOffset = 0.0f;
				float endDistanceOffset = 0.0f;
				float splineLength = GetSplineLength();

				UPursuitSplineComponent* startSpline = nullptr;
				UPursuitSplineComponent* endSpline = nullptr;

				for (FSplineLink& link : SplineLinks)
				{
					if (link.Spline == masterSpline)
					{
						if (link.ThisDistance < KINDA_SMALL_NUMBER)
						{
							linkedStart = true;
							startSpline = link.Spline.Get();
							startDistance = link.NextDistance;
						}
						else if (link.ThisDistance >= splineLength - KINDA_SMALL_NUMBER)
						{
							linkedEnd = true;
							endSpline = link.Spline.Get();
							endDistance = link.NextDistance;
						}
					}
				}

				if (degreesOfSeparation > 0)
				{
					if (linkedStart == false)
					{
						// We have no start link. See if any of the splines we're linked to are connected
						// to the master spline at their starts.

						for (FSplineLink& link : SplineLinks)
						{
							if (link.ThisDistance < KINDA_SMALL_NUMBER)
							{
								for (FSplineLink& childLink : link.Spline->SplineLinks)
								{
									if (childLink.Spline == masterSpline &&
										childLink.ThisDistance < KINDA_SMALL_NUMBER)
									{
										startSpline = link.Spline.Get();

										if (link.Spline->HasMasterSplineDistances() == true)
										{
											// It's best if we can grab a master distance directly from the connected spline.

											linkedStart = true;
											startDistance = link.Spline->GetMasterDistanceAtDistanceAlongSpline(link.NextDistance, masterSplineLength);
											break;
										}
										else if (degreesOfSeparation > 1)
										{
											// This is OK too, but it's not as accurate and can deviate by hundreds of meters.

											linkedStart = true;
											startDistance = childLink.NextDistance;
											startDistanceOffset = link.NextDistance;
											break;
										}
									}
								}
							}

							if (linkedStart == true)
							{
								break;
							}
						}
					}

					if (linkedStart == true &&
						linkedEnd == false)
					{
						// We have a start link, but no end. See if any of the splines we're linked to are connected
						// to the master spline at their ends.

						for (FSplineLink& link : SplineLinks)
						{
							if (link.ThisDistance >= splineLength - KINDA_SMALL_NUMBER)
							{
								float childSplineLength = link.Spline->GetSplineLength();

								for (FSplineLink& childLink : link.Spline->SplineLinks)
								{
									if (childLink.Spline == masterSpline &&
										childLink.ThisDistance >= childSplineLength - KINDA_SMALL_NUMBER)
									{
										endSpline = link.Spline.Get();

										if (link.Spline->HasMasterSplineDistances() == true)
										{
											// It's best if we can grab a master distance directly from the connected spline.

											linkedEnd = true;
											endDistance = link.Spline->GetMasterDistanceAtDistanceAlongSpline(link.NextDistance, masterSplineLength);
											break;
										}
										else if (degreesOfSeparation > 1)
										{
											// This is OK too, but it's not as accurate and can deviate by hundreds of meters.

											linkedEnd = true;
											endDistance = childLink.NextDistance;
											endDistanceOffset = childLink.ThisDistance - link.NextDistance;
											break;
										}
									}
								}
							}

							if (linkedEnd == true)
							{
								break;
							}
						}
					}
				}

				if (recalibrate == 1 &&
					recalibrationAttempt > 0)
				{
					if (linkedStart == false ||
						linkedEnd == false)
					{
						startSpline = endSpline = nullptr;

						for (FSplineLink& link : SplineLinks)
						{
							if (link.Spline->HasMasterSplineDistances() == true)
							{
								if (startSpline == nullptr && link.Spline->MasterDistanceClass < 3 && link.ThisDistance < KINDA_SMALL_NUMBER)
								{
									linkedStart = true;
									startSpline = link.Spline.Get();
									startDistance = link.Spline->GetMasterDistanceAtDistanceAlongSpline(link.NextDistance, masterSplineLength);
								}
								else if (endSpline == nullptr && link.Spline->MasterDistanceClass < 3 && link.ThisDistance >= splineLength - KINDA_SMALL_NUMBER)
								{
									linkedEnd = true;
									endSpline = link.Spline.Get();
									endDistance = link.Spline->GetMasterDistanceAtDistanceAlongSpline(link.NextDistance, masterSplineLength);
								}
							}
						}
					}
				}

				float totalSplineLength = startDistanceOffset + splineLength + endDistanceOffset;

				if (linkedStart == true &&
					linkedEnd == true &&
					splineLength > KINDA_SMALL_NUMBER &&
					totalSplineLength > KINDA_SMALL_NUMBER)
				{
					bool regenerate = false;

					if (recalibrate != 0)
					{
						FPursuitPointExtendedData& first = pursuitPointExtendedData[0];
						FPursuitPointExtendedData& last = pursuitPointExtendedData.Last();
						float startDifference = masterSpline->GetDistanceDifference(startDistance, first.MasterSplineDistance);
						float endDifference = masterSpline->GetDistanceDifference(endDistance, last.MasterSplineDistance);

						int32 numGood = 0;
						int32 numBad = 0;

						if (recalibrate == 1 &&
							startDifference > 25.0f * 100.0f)
						{
							if (report == true)
							{
								UE_LOG(GripLogPursuitSplines, Log, TEXT("Pursuit spline %s calculated master distances are out at the start by %dm"), *ActorName, (int32)(startDifference / 100.0f));
							}

							if (startSpline != nullptr &&
								startSpline->CalculateMasterSplineDistances(masterSpline, masterSplineLength, startingDistance, degreesOfSeparation, false, 2) == true)
							{
								numGood++;

								if (report == true)
								{
									UE_LOG(GripLogPursuitSplines, Log, TEXT("Pursuit spline %s it's connected to has good data"), *startSpline->ActorName);
								}
							}
							else
							{
								numBad++;

								if (report == true)
								{
									UE_LOG(GripLogPursuitSplines, Log, TEXT("Pursuit spline %s it's connected to has insufficient data"), *startSpline->ActorName);
								}
							}
						}

						if (recalibrate == 1 &&
							endDifference > 25.0f * 100.0f)
						{
							if (report == true)
							{
								UE_LOG(GripLogPursuitSplines, Log, TEXT("Pursuit spline %s calculated master distances are out at the end by %dm"), *ActorName, (int32)(endDifference / 100.0f));
							}

							if (endSpline != nullptr &&
								endSpline->CalculateMasterSplineDistances(masterSpline, masterSplineLength, startingDistance, degreesOfSeparation, false, 2) == true)
							{
								numGood++;

								if (report == true)
								{
									UE_LOG(GripLogPursuitSplines, Log, TEXT("Pursuit spline %s it's connected to has good data"), *endSpline->ActorName);
								}
							}
							else
							{
								numBad++;

								if (report == true)
								{
									UE_LOG(GripLogPursuitSplines, Log, TEXT("Pursuit spline %s it's connected to has insufficient data"), *endSpline->ActorName);
								}
							}
						}

						regenerate = numGood > 0 && numBad == 0;

						if (reportGoodData == true)
						{
							result = startDifference <= 25.0f * 100.0f && endDifference <= 25.0f * 100.0f;
						}

						if (recalibrate == 1 &&
							regenerate == true)
						{
							if (startSpline == nullptr)
							{
								startSpline = this;
							}

							if (endSpline == nullptr)
							{
								endSpline = this;
							}

							dataClass = FMath::Max(startSpline->MasterDistanceClass, endSpline->MasterDistanceClass);

							UE_LOG(GripLogPursuitSplines, Log, TEXT("Pursuit spline %s is being regenerated from the good data"), *ActorName);
						}
					}

					if (recalibrate == 0 ||
						regenerate == true)
					{
						// Easy case where the start and end points of the spline are connected directly to the master spline,
						// or indirectly via splines we're directly connected to which are in themselves directly connected
						// to the master spline - so only one degree of separation.

						float masterSectionLength = (startDistance < endDistance) ? endDistance - startDistance : (masterSplineLength - startDistance) + endDistance;

						for (int32 i = 0; i < numExtendedPoints; i++)
						{
							FPursuitPointExtendedData& point = pursuitPointExtendedData[i];
							float distance = (point.Distance + startDistanceOffset) / totalSplineLength;

							distance *= masterSectionLength;
							distance += startDistance;
							distance = FMath::Fmod(distance, masterSplineLength);

							point.MasterSplineDistance = distance;
						}

						if (report == true)
						{
							UE_LOG(GripLogPursuitSplines, Log, TEXT("Pursuit spline %s calculated master distances with class %d data."), *ActorName, dataClass + 1);
						}

						MasterDistanceClass = dataClass;

						result = true;
					}
				}
				else if (degreesOfSeparation == 3)
				{
					if (recalibrate == 0)
					{
						for (int32 i = 0; i < numExtendedPoints; i++)
						{
							FPursuitPointExtendedData& point = pursuitPointExtendedData[i];
							float t0 = masterDistance - (movementSize * scanSpan * 0.5f);
							float t1 = masterDistance + (movementSize * scanSpan * 0.5f);

							point.MasterSplineDistance = masterSpline->GetNearestDistance(GetWorldLocationAtDistanceAlongSpline(point.Distance), t0, t1, numIterations, numSamples);

							masterDistance = point.MasterSplineDistance;
						}

						if (report == true)
						{
							UE_LOG(GripLogPursuitSplines, Log, TEXT("Pursuit spline %s calculated master distances with class %d data."), *ActorName, dataClass + 1);
						}

						MasterDistanceClass = dataClass;

						result = true;
					}
				}
				else
				{
					return result;
				}
			}

			if (recalibrate == 0)
			{
				for (FSplineLink& link : SplineLinks)
				{
					if (link.ForwardLink == true &&
						link.NextDistance < 100.0f &&
						link.Spline->HasMasterSplineDistances() == false)
					{
						// UE_LOG(GripLogPursuitSplines, Log, TEXT("Linking forward to spline %s at distance %0.01f"), *link.Spline->ActorName, link.ThisDistance);

						result |= link.Spline->CalculateMasterSplineDistances(masterSpline, masterSplineLength, GetMasterDistanceAtDistanceAlongSpline(link.ThisDistance, masterSplineLength), degreesOfSeparation, report);
					}
					else
					{
						// UE_LOG(GripLogPursuitSplines, Log, TEXT("Rejected spline link to %s at distance %0.01f"), *link.Spline->ActorName, link.ThisDistance);
					}
				}
			}
		}
		else
		{
			UE_LOG(GripLogPursuitSplines, Log, TEXT("No extended points in CalculateMasterSplineDistances"));
		}
	}

	return result;
}

/**
* Helper function when using the Editor.
***********************************************************************************/

TStructOnScope<FActorComponentInstanceData> UPursuitSplineComponent::GetComponentInstanceData() const
{
	auto InstanceData = MakeStructOnScope<FActorComponentInstanceData, FPursuitSplineInstanceData>(this);
	FPursuitSplineInstanceData* SplineInstanceData = InstanceData.Cast<FPursuitSplineInstanceData>();

	if (bSplineHasBeenEdited)
	{
		SplineInstanceData->SplineCurves = SplineCurves;
		SplineInstanceData->bClosedLoop = IsClosedLoop();
		SplineInstanceData->Type = Type;
	}

	SplineInstanceData->bSplineHasBeenEdited = bSplineHasBeenEdited;

	return InstanceData;
}

/**
* Helper function when using the Editor.
***********************************************************************************/

void UPursuitSplineComponent::ApplyComponentInstanceData(FPursuitSplineInstanceData* SplineInstanceData, const bool bPostUCS)
{
	check(SplineInstanceData);

	if (bPostUCS)
	{
		if (bInputSplinePointsToConstructionScript)
		{
			// Don't reapply the saved state after the UCS has run if we are inputting the points to it.
			// This allows the UCS to work on the edited points and make its own changes.
			return;
		}
		else
		{
			bModifiedByConstructionScript = (SplineInstanceData->SplineCurvesPreUCS != SplineCurves);
			bModifiedByConstructionScript |= (SplineInstanceData->bClosedLoop != IsClosedLoop());
			bModifiedByConstructionScript |= (SplineInstanceData->TypePreUCS != Type);

			// If we are restoring the saved state, unmark the SplineCurves property as 'modified'.
			// We don't want to consider that these changes have been made through the UCS.
			TArray<FProperty*> Properties;
			Properties.Emplace(FindFProperty<FProperty>(USplineComponent::StaticClass(), GET_MEMBER_NAME_CHECKED(USplineComponent, SplineCurves)));
			RemoveUCSModifiedProperties(Properties);

			Properties.Empty();
			Properties.Emplace(FindFProperty<FProperty>(USplineComponent::StaticClass(), FName(TEXT("bClosedLoop"))));
			RemoveUCSModifiedProperties(Properties);

			Properties.Empty();
			Properties.Emplace(FindFProperty<FProperty>(UPursuitSplineComponent::StaticClass(), GET_MEMBER_NAME_CHECKED(UPursuitSplineComponent, Type)));
			RemoveUCSModifiedProperties(Properties);
		}
	}
	else
	{
		SplineInstanceData->SplineCurvesPreUCS = SplineCurves;
		SplineInstanceData->bClosedLoopPreUCS = IsClosedLoop();
		SplineInstanceData->TypePreUCS = Type;
	}

	if (SplineInstanceData->bSplineHasBeenEdited)
	{
		SplineCurves = SplineInstanceData->SplineCurves;
		SetClosedLoop(SplineInstanceData->bClosedLoop);
		Type = SplineInstanceData->Type;

		bModifiedByConstructionScript = false;
	}

	bSplineHasBeenEdited = SplineInstanceData->bSplineHasBeenEdited;

	UpdateSpline();
}

/**
* Calculate the sections of the spline.
***********************************************************************************/

void UPursuitSplineComponent::CalculateSections()
{
	Super::CalculateSections();
}

/**
* The point data, referenced from the parent actor.
***********************************************************************************/

TArray<FPursuitPointData>& UPursuitSplineComponent::GetPursuitPointData() const
{
	return PursuitSplineParent->PointData;
}

/**
* The extended point data, referenced from the parent actor.
***********************************************************************************/

TArray<FPursuitPointExtendedData>& UPursuitSplineComponent::GetPursuitPointExtendedData() const
{
	return PursuitSplineParent->PointExtendedData;
}

#pragma endregion NavigationSplines
