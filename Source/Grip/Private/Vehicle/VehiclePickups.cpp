/**
*
* Vehicle pickups implementation.
*
* Original author: Rob Baker.
* Current maintainer: Rob Baker.
*
* Copyright Caged Element Inc, code provided for educational purposes only.
*
* Handle all of the pickups-related activity for the vehicle, mostly related to the
* two pickup slots that each vehicle has for two different pickups.
*
***********************************************************************************/

#include "vehicle/flippablevehicle.h"
#include "gamemodes/playgamemode.h"
#include "game/globalgamestate.h"
#include "pickups/speedpad.h"
#include "pickups/gatlinggun.h"
#include "pickups/turbo.h"
#include "pickups/shield.h"
#include "components/widgetcomponent.h"

DEFINE_LOG_CATEGORY_STATIC(GripLogPickups, Warning, All);

/**
* Give a particular pickup to a vehicle.
***********************************************************************************/

int32 ABaseVehicle::GivePickup(EPickupType type, int32 pickupSlot, bool fromTrack)
{
	return -1;
}

/**
* Is a pickup slot currently charging?
***********************************************************************************/

bool FPlayerPickupSlot::IsCharging(bool confirmed) const
{
	return ChargingState == EPickupSlotChargingState::Charging && (confirmed == false || HookTimer >= ABaseVehicle::PickupHookTime);
}

#pragma region SpeedPads

/**
* Collect the speed pads overlapping with a vehicle.
***********************************************************************************/

void ABaseVehicle::CollectSpeedPads()
{
	if (GRIP_OBJECT_VALID(VehicleCollision) == true)
	{
		// Determine which speed pad actors are currently overlapping with this
		// vehicle's collision shell.

		TSet<AActor*> collectedActors;

		VehicleCollision->GetOverlappingActors(collectedActors, ASpeedPad::StaticClass());

		if (collectedActors.Num() > 0)
		{
			// If we have any overlapping speed pads then find the closest one to the vehicle.

			float minDistance = 0.0f;
			AActor* closestSpeedpad = nullptr;
			FVector location = GetActorLocation();

			for (AActor* actor : collectedActors)
			{
				float distance = (actor->GetActorLocation() - location).SizeSquared();

				if (minDistance > distance ||
					closestSpeedpad == nullptr)
				{
					minDistance = distance;
					closestSpeedpad = actor;
				}
			}

			// Collect the closest speed pad from this vehicle.

			(Cast<ASpeedPad>(closestSpeedpad))->OnSpeedPadCollected(this);
		}
	}
}

/**
* Add a temporary boost to the vehicle, for when running over speed pads and the
* like.
*
* amount is between 0 and 1, 1 being 100% more engine power.
* duration is in seconds.
* direction is the world direction to apply the speed boost force.
*
***********************************************************************************/

bool ABaseVehicle::SpeedBoost(ASpeedPad* speedpad, float amount, float duration, const FVector& direction)
{
	FVector thisLocation = speedpad->GetActorLocation();
	FVector thisDirection = speedpad->GetActorRotation().Vector();

	for (FSpeedpadVehicleBoost& boost : Propulsion.SpeedPadBoosts)
	{
		if (speedpad == boost.SpeedPad)
		{
			// Reject the speed pad given as we're already boosting from it.

			return false;
		}

		// Block the speed pad if we're already going over one that is more or less
		// horizontally aligned with the speed pad given. This is to prevent one
		// vehicle hogging a couple of pads in a line across the track when there are
		// other players that need them too. This is a real game-play fix and not
		// something I would have thought we'd need to do, but the players think so.

		FVector location = boost.SpeedPad->GetActorLocation();
		FRotator rotation = boost.SpeedPad->GetActorRotation();
		float radius = boost.SpeedPad->CollisionBox->GetScaledBoxExtent().Size();
		FVector difference = location - thisLocation;
		float distance = difference.Size();

		// Are these speed pads close to one another?

		if (distance < radius * 2.0f)
		{
			difference.Normalize();

			// Are these speed pads broadly facing the same direction?

			if (FVector::DotProduct(rotation.Vector(), thisDirection) > 0.8f)
			{
				// Are these speed pads horizontally aligned?

				if (FMath::Abs(FVector::DotProduct(rotation.Vector(), difference)) < 0.1f)
				{
					return false;
				}
			}
		}
	}

	Propulsion.SpeedPadBoosts.Emplace(FSpeedpadVehicleBoost(speedpad, amount, duration, direction));

	return true;
}

#pragma endregion SpeedPads

#pragma region PickupPads

/**
* Collect the pickups overlapping with a vehicle.
***********************************************************************************/

void ABaseVehicle::CollectPickups()
{
	if (GRIP_OBJECT_VALID(VehicleCollision) == true)
	{
		TSet<AActor*> collectedActors;

		VehicleCollision->GetOverlappingActors(collectedActors, APickup::StaticClass());

		for (AActor* actor : collectedActors)
		{
			APickup* pickup = Cast<APickup>(actor);

			if (pickup->IsCollectible() == true)
			{
				if (pickup->Class == EPickupClass::Pickup)
				{
					pickup->OnPickupPadCollected(this);
				}
				else if (pickup->Class == EPickupClass::Health)
				{
					if (RaceState.HitPoints != RaceState.MaxHitPoints)
					{
						pickup->OnPickupPadCollected(this);

						RaceState.HitPoints += (RaceState.MaxHitPoints >> 2);
						RaceState.HitPoints = FMath::Min(RaceState.HitPoints, RaceState.MaxHitPoints);

						HUD.Warning(EHUDWarningSource::HealthPickup, 1.0f, 0.666f);
					}
				}
				else if (pickup->Class == EPickupClass::DoubleDamage)
				{
					if (RaceState.DoubleDamage == 0.0f)
					{
						pickup->OnPickupPadCollected(this);

						RaceState.DoubleDamage = GRIP_DOUBLE_DAMAGE_SECONDS;

						HUD.Warning(EHUDWarningSource::DoubleDamagePickup, 1.0f, 0.666f);
					}
				}
				else if (pickup->Class == EPickupClass::Collectible)
				{
					pickup->OnPickupPadCollected(this);
				}
			}
		}
	}
}

#pragma endregion PickupPads
