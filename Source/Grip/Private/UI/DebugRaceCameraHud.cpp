/**
*
* Race camera debugging HUD.
*
* Original author: Rob Baker.
* Current maintainer: Rob Baker.
*
* Copyright Caged Element Inc, code provided for educational purposes only.
*
***********************************************************************************/

#include "ui/debugracecamerahud.h"
#include "vehicle/flippablevehicle.h"

#pragma region VehicleCamera

/**
* Draw the HUD.
***********************************************************************************/

void ADebugRaceCameraHUD::DrawHUD()
{
	Super::DrawHUD();

	HorizontalOffset = 200.0f;

	APawn* owningPawn = GetOwningPawn();
	ABaseVehicle* vehicle = Cast<ABaseVehicle>(owningPawn);

	if (vehicle != nullptr)
	{
		URaceCameraComponent* camera = vehicle->Camera;

		vehicle = vehicle->CameraTarget();

		{
			AddBool(TEXT("IsFlipped"), vehicle->IsFlipped());
			AddBool(TEXT("IsFlippedAndWheelsOnGround"), vehicle->IsFlippedAndWheelsOnGround());
			AddInt(TEXT("FlipDetection"), vehicle->GetWheels().FlipDetection);
			AddBool(TEXT("IsAirborne"), vehicle->IsAirborne(false));

#pragma region VehicleSpringArm

			UFlippableSpringArmComponent* arm = vehicle->SpringArm;

			AddBool(TEXT("HasSmashedIntoSomething"), vehicle->HasSmashedIntoSomething(150.0f));
			AddBool(TEXT("ArmAirborne"), arm->Airborne);
			AddInt(TEXT("FromFollowingMode"), (int32)arm->FromFollowingMode);
			AddInt(TEXT("FollowingMode"), (int32)arm->FollowingMode);
			AddFloat(TEXT("NoAirborneContactTime"), arm->NoAirborneContactTime);
			AddFloat(TEXT("FollowingModeTime"), arm->FollowingModeTime);
			AddFloat(TEXT("ThisModeTransitionTime"), arm->ThisModeTransitionTime);
			AddFloat(TEXT("GetFollowingTransitionRatio"), arm->GetFollowingTransitionRatio());
			const FTransform& vehicleTransform = vehicle->VehicleMesh->GetComponentTransform();
			FRotator r0 = vehicleTransform.Rotator();
			AddVector(TEXT("VehicleRotation"), FVector(r0.Yaw, r0.Pitch, r0.Roll));
			r0 = arm->TransitionRotations[(int32)arm->FromFollowingMode][0];
			AddVector(TEXT("FromRotation"), FVector(r0.Yaw, r0.Pitch, r0.Roll));
			r0 = arm->TransitionRotations[(int32)arm->FollowingMode][0];
			AddVector(TEXT("ToRotation"), FVector(r0.Yaw, r0.Pitch, r0.Roll));
			r0 = arm->Rotations[(int32)UFlippableSpringArmComponent::EFollowingMode::Normal];
			AddVector(TEXT("NormalRotation"), FVector(r0.Yaw, r0.Pitch, r0.Roll));
			r0 = arm->Rotations[(int32)UFlippableSpringArmComponent::EFollowingMode::Airborne];
			AddVector(TEXT("AirborneRotation"), FVector(r0.Yaw, r0.Pitch, r0.Roll));
			r0 = arm->Rotations[(int32)UFlippableSpringArmComponent::EFollowingMode::Crashed];
			AddVector(TEXT("CrashedRotation"), FVector(r0.Yaw, r0.Pitch, r0.Roll));
			r0 = arm->Rotations[(int32)arm->FollowingMode];
			AddVector(TEXT("SelectedRotation"), FVector(r0.Yaw, r0.Pitch, r0.Roll));
			r0 = arm->SmoothedRotations[(int32)arm->FollowingMode];
			AddVector(TEXT("SmoothedRotation"), FVector(r0.Yaw, r0.Pitch, r0.Roll));
			r0 = arm->TargetRotation;
			AddVector(TEXT("TargetRotation"), FVector(r0.Yaw, r0.Pitch, r0.Roll));
			AddFloat(TEXT("LastClippingDistance"), arm->LastClippingDistance);
			AddFloat(TEXT("LaunchDirectionFlipTime"), arm->LaunchDirectionFlipTime);
			AddFloat(TEXT("AirToGroundTime"), arm->AirToGroundTime);
			AddFloat(TEXT("NativeFieldOfView"), camera->NativeFieldOfView);
			AddVector(TEXT("ArmRoot"), arm->ArmRoot);
			AddInt(TEXT("ArmRootMode"), (int32)arm->ArmRootMode);

#pragma endregion VehicleSpringArm

			AddLine(vehicle->GetCenterLocation(), vehicle->GetCenterLocation() + vehicle->GetActorRotation().RotateVector(FVector(0.0f, 0.0f, (vehicle->IsFlipped() == true) ? 5.0f : -5.0f) * 33.0f), FLinearColor(1.0f, 0.0f, 0.0f), 6.0f);
			AddLine(vehicle->GetCenterLocation(), vehicle->GetCenterLocation() + vehicle->GetActorRotation().RotateVector(FVector(0.0f, 0.0f, (vehicle->IsFlippedAndWheelsOnGround() == true) ? 5.0f : -5.0f) * 33.0f), FLinearColor(0.0f, 1.0f, 0.0f), 2.0f);

			int32 index = 0;

			for (const FVehicleWheel& wheel : vehicle->GetWheels().Wheels)
			{
				bool inContact = wheel.GetActiveSensor().IsInContact();
				bool inEffect = wheel.GetActiveSensor().IsInEffect();
				bool inPossibleContact = wheel.GetActiveSensor().HasNearestContactPoint(wheel.Velocity, 2.0f);

				AddBox(vehicle->GetWheelBoneLocationFromIndex(index), FMath::Lerp(FLinearColor::Red, FLinearColor::Green, (inEffect == true) ? 1.0f : 0.0f), 5.0f);
				AddBox(vehicle->GetWheelBoneLocationFromIndex(index), FMath::Lerp(FLinearColor::Red, FLinearColor::Green, (inContact == true) ? 1.0f : 0.0f), 15.0f);
				AddBox(vehicle->GetWheelBoneLocationFromIndex(index), FMath::Lerp(FLinearColor::Red, FLinearColor::Green, (inPossibleContact == true) ? 1.0f : 0.0f), 25.0f);

				for (const FVehicleContactSensor& sensor : wheel.Sensors)
				{
					if (sensor.HasNearestContactPoint(wheel.Velocity, 0.0f) == true)
					{
						AddLine(vehicle->GetWheelBoneLocationFromIndex(index), sensor.GetNearestContactPoint(), FMath::Lerp(FLinearColor::Red, FLinearColor::Green, (sensor.IsInEffect() == true) ? 1.0f : 0.0f), 2.0f);
						AddBox(sensor.GetNearestContactPoint(), FMath::Lerp(FLinearColor::Red, FLinearColor::Green, (sensor.IsInEffect() == true) ? 1.0f : 0.0f), 5.0f);
					}
				}

				index++;
			}
		}
	}
}

#pragma endregion VehicleCamera
