/**
*
* Play game mode implementation.
*
* Original author: Rob Baker.
* Current maintainer: Rob Baker.
*
* Copyright Caged Element Inc, code provided for educational purposes only.
*
* The play game mode to use for the game, specifically for playing a level and
* is the C++ game mode used in GRIP, with a blueprint wrapping it for actual use.
*
***********************************************************************************/

#include "gamemodes/playgamemode.h"
#include "ai/pursuitsplineactor.h"
#include "vehicle/basevehicle.h"
#include "game/globalgamestate.h"
#include "system/worldfilter.h"
#include "blueprint/userwidget.h"
#include "components/panelwidget.h"
#include "components/textblock.h"
#include "blueprint/widgetblueprintlibrary.h"
#include "blueprint/widgetlayoutlibrary.h"
#include "components/canvaspanelslot.h"
#include "components/image.h"
#include "camera/statictrackcamera.h"
#include "ui/hudwidget.h"

/**
* APlayGameMode statics.
***********************************************************************************/

// The type of widget to use for the single screen UI.
TSubclassOf<USingleHUDWidget> APlayGameMode::SingleScreenWidgetClass = nullptr;

/**
* Construct a play game mode.
***********************************************************************************/

APlayGameMode::APlayGameMode()
{
	// We need all of the players to be ticked before the game state so that we can
	// calculate race position effectively.

	PrimaryActorTick.bCanEverTick = true;
	PrimaryActorTick.bTickEvenWhenPaused = true;
	PrimaryActorTick.TickGroup = TG_PostUpdateWork;

	// Ensure that random is random.

	FMath::RandInit((int32)FDateTime::Now().ToUnixTimestamp() + (uint64)(this));
}

/**
* Get the vehicle for a vehicle index.
***********************************************************************************/

ABaseVehicle* APlayGameMode::GetVehicleForVehicleIndex(int32 vehicleIndex) const
{
	if (vehicleIndex >= 0)
	{
		for (ABaseVehicle* vehicle : Vehicles)
		{
			if (vehicle->VehicleIndex == vehicleIndex)
			{
				return vehicle;
			}
		}
	}

	return nullptr;
}

/**
* Do some post initialization just before the game is ready to play.
***********************************************************************************/

void APlayGameMode::PostInitializeComponents()
{
	UE_LOG(GripLog, Log, TEXT("APlayGameMode::PostInitializeComponents"));

	Super::PostInitializeComponents();

#if UE_BUILD_SHIPPING
	if (GameStateOverrides != nullptr)
	{
		GameStateOverrides->OverrideGrid = false;
	}
#endif // UE_BUILD_SHIPPING

#if !WITH_EDITOR
	HUDClass = nullptr;
#endif // !WITH_EDITOR

	if (GetWorld() != nullptr && GetWorld()->GetGameViewport() != nullptr)
	{
		GetWorld()->GetGameViewport()->SetForceDisableSplitscreen(false);
	}

	// Iterate through the navigation layers present in the level and record their names.

	TArray<FName> navigationLayers;

	for (TActorIterator<APursuitSplineActor> actorItr0(GetWorld()); actorItr0; ++actorItr0)
	{
		for (const FName& layer : (*actorItr0)->Layers)
		{
			if (layer.ToString().EndsWith("Navigation") == true)
			{
				if (navigationLayers.Contains(layer) == false)
				{
					navigationLayers.Emplace(layer);
				}
			}
		}
	}

	if (GlobalGameState != nullptr)
	{
		const bool inTransition = IsValid(GetWorld()) && GetWorld()->IsInSeamlessTravel();

		// Pick a valid navigation layer name to use.

		if (navigationLayers.Contains(FName(*GlobalGameState->TransientGameState.NavigationLayer)) == false && inTransition == false)
		{
			GlobalGameState->TransientGameState.NavigationLayer = TEXT("");

			if (navigationLayers.Num() > 0)
			{
				if (navigationLayers.Contains("ForwardNavigation") == true)
				{
					GlobalGameState->TransientGameState.NavigationLayer = "ForwardNavigation";
				}
				else
				{
					GlobalGameState->TransientGameState.NavigationLayer = navigationLayers[0].ToString();
				}
			}
		}

		// Now iterate the actors, destroying those that are not compatible with the
		// current navigation layer.

		for (TActorIterator<AActor> actorItr(GetWorld()); actorItr; ++actorItr)
		{
			FWorldFilter::IsValid(*actorItr, GlobalGameState);
		}

#pragma region VehicleSurfaceEffects

		// Find the driving surface properties for this level and store them away in the game play setup.

		for (TActorIterator<ADrivingSurfaceProperties> actorItr(GetWorld()); actorItr; ++actorItr)
		{
			if (FWorldFilter::IsValid(*actorItr, GlobalGameState) == true)
			{
				ADrivingSurfaceProperties* properties = *actorItr;

				GlobalGameState->TransientGameState.MapBrightness = properties->AmbientBrightness;
				GlobalGameState->TransientGameState.MapSurfaceColor = FVector(properties->SurfaceColor.R, properties->SurfaceColor.G, properties->SurfaceColor.B);
				GlobalGameState->TransientGameState.MapLightingColor = FVector(properties->LightColor.R, properties->LightColor.G, properties->LightColor.B);

				break;
			}
		}

#pragma endregion VehicleSurfaceEffects

	}
}

/**
* Calculate the maximum number of players.
***********************************************************************************/

int32 APlayGameMode::CalculateMaxPlayers() const
{
	int32 maxPlayers = FMath::Min(GlobalGameState->GeneralOptions.NumberOfPlayers, Startpoints.Num());

	if (GameStateOverrides != nullptr &&
		GameStateOverrides->OverrideGrid == true)
	{
		maxPlayers = FMath::Min(maxPlayers, GameStateOverrides->Grid.Num() + GlobalGameState->TransientGameState.NumberOfLocalPlayers);
		maxPlayers = FMath::Min(maxPlayers, Startpoints.Num());
	}

#if WITH_EDITOR

	// If we're not starting on the normal start line, then only create one player.

	for (TActorIterator<APlayerStartPIE> itr(GetWorld()); itr; ++itr)
	{
		maxPlayers = FMath::Min(maxPlayers, 1);
		break;
	}

#endif // WITH_EDITOR

	return maxPlayers;
}

/**
* Do some initialization when the game is ready to play.
***********************************************************************************/

void APlayGameMode::BeginPlay()
{
	UE_LOG(GripLog, Log, TEXT("APlayGameMode::BeginPlay"));

	Super::BeginPlay();

	// Create a new single screen widget and add it to the viewport. This is what will
	// contain all of the HUDs for each player - there is more than one in split-screen
	// games. It ordinarily contains the pause menu and other full-screen elements too,
	// but are missing from this stripped implementation.

	if (SingleScreenWidgetClass != nullptr)
	{
		SingleScreenWidget = NewObject<USingleHUDWidget>(this, SingleScreenWidgetClass);

		if (SingleScreenWidget != nullptr)
		{
			SingleScreenWidget->AddToViewport(1);
		}
	}

	StartLineDropTime = 6;
	StartLineCountFrom = StartLineDropTime;
	StartLineCountTo = StartLineCountFrom + 3;

	UWorld* world = GetWorld();

	for (TActorIterator<AActor> actorItr(world); actorItr; ++actorItr)
	{
#if GRIP_FIX_REVERB_FADE_TIMES
		if (FWorldFilter::IsValid(*actorItr, GlobalGameState) == true)
		{
			if ((*actorItr)->IsA<AAudioVolume>() == true)
			{
				// Hack to stop glitching audio by setting the reverb fade time to zero.

				AAudioVolume* volume = Cast<AAudioVolume>(*actorItr);

				if (volume != nullptr)
				{
					FReverbSettings settings = volume->GetReverbSettings();
					settings.FadeTime = 0.0f;
					volume->SetReverbSettings(settings);
				}
			}
		}
#else // GRIP_FIX_REVERB_FADE_TIMES
		FWorldFilter::IsValid(*actorItr, GlobalGameState);
#endif // GRIP_FIX_REVERB_FADE_TIMES
	}

	ChangeTimeDilation(1.0f, 0.0f);

	// Setup some good defaults for the game setup if not already set.

	if (GlobalGameState->GamePlaySetup.GameType == EGameType::SinglePlayerEvent)
	{
		GlobalGameState->GamePlaySetup.DrivingMode = EDrivingMode::Race;
	}

	if (GlobalGameState->GamePlaySetup.DrivingMode == EDrivingMode::None)
	{
		GlobalGameState->GamePlaySetup.DrivingMode = EDrivingMode::Race;
	}

	if (GlobalGameState->GeneralOptions.NumberOfLaps == 0)
	{
		GlobalGameState->GeneralOptions.NumberOfLaps = 4;
	}

	// Find a master racing spline against which we can measure race distance.

	if (MasterRacingSpline.IsValid() == false)
	{
		MasterRacingSpline = DetermineMasterRacingSpline(FName(*GlobalGameState->TransientGameState.NavigationLayer), world, GlobalGameState);
	}

	// Now determine the length of that master racing spline.

	if (MasterRacingSpline.IsValid() == true)
	{
		MasterRacingSplineLength = MasterRacingSpline->GetSplineLength();
	}

	// Do some conditioning on all the pursuit splines so that we have accurate data
	// to work with, especially regarding race distance.

	BuildPursuitSplines(false, FName(*GlobalGameState->TransientGameState.NavigationLayer), world, GlobalGameState, MasterRacingSpline.Get());
	EstablishPursuitSplineLinks(false, FName(*GlobalGameState->TransientGameState.NavigationLayer), world, GlobalGameState, MasterRacingSpline.Get());

	int32 index = 0;

	Vehicles.Empty();

	// Setup all the vehicles that have already been created in the menu UI
	// (all local players normally).

	for (TActorIterator<ABaseVehicle> actorItr(world); actorItr; ++actorItr)
	{
		ABaseVehicle* vehicle = *actorItr;

		if (Vehicles.Num() == 0)
		{
			ViewingPawn = vehicle;
		}

		vehicle->PostSpawn(index++, true, false);
	}

	GameSequence = EGameSequence::Initialise;

	// Record all of the frictional actors in the level.

	for (TActorIterator<AActor> actorItr(world); actorItr; ++actorItr)
	{
		AActor* actor = *actorItr;

		if (actor->GetClass()->GetName().StartsWith("StartingGateBP") == true)
		{
			FrictionalActors.Emplace(actor);
		}
		else if (FWorldFilter::IsValid(actor, GlobalGameState) == true)
		{
			for (const FName& layer : actor->Layers)
			{
				if (layer == TEXT("LimitVehicleLaunching"))
				{
					FrictionalActors.Emplace(actor);
					break;
				}
			}
		}
	}

	LastOptionsResetTime = GetClock();
}

/**
* Do some shutdown when the actor is being destroyed.
***********************************************************************************/

void APlayGameMode::EndPlay(const EEndPlayReason::Type endPlayReason)
{
	UE_LOG(GripLog, Log, TEXT("APlayGameMode::EndPlay"));

	if (SingleScreenWidget != nullptr)
	{
		SingleScreenWidget->RemoveFromViewport();
		SingleScreenWidget = nullptr;
	}

	// Ensure time dilation is switched off here.

	ChangeTimeDilation(1.0f, 0.0f);

	Super::EndPlay(endPlayReason);
}

/**
* Determine the vehicles that are currently present in the level.
***********************************************************************************/

void APlayGameMode::DetermineVehicles()
{
	Vehicles.Empty();

	for (TActorIterator<ABaseVehicle> actorItr(GetWorld()); actorItr; ++actorItr)
	{
		Vehicles.Emplace(*actorItr);
	}

	// Sort the vehicles by vehicle index, not strictly necessary, but this could
	// help to avoid bugs when referencing vehicles later.

	Vehicles.Sort([] (const ABaseVehicle& object1, const ABaseVehicle& object2)
		{
			return object1.GetVehicleIndex() < object2.GetVehicleIndex();
		});
}

/**
* Determine the pursuit splines that are currently present in the level.
***********************************************************************************/

void APlayGameMode::DeterminePursuitSplines()
{
	PursuitSplines.Empty();

	for (TActorIterator<APursuitSplineActor> actorItr(GetWorld()); actorItr; ++actorItr)
	{
		if (FWorldFilter::IsValid(*actorItr, GlobalGameState) == true)
		{
			PursuitSplines.Emplace(*actorItr);
		}
	}
}

/**
* Determine the master racing spline.
***********************************************************************************/

UPursuitSplineComponent* APlayGameMode::DetermineMasterRacingSpline(const FName& navigationLayer, UWorld* world, UGlobalGameState* gameState)
{
	// Go through every spline in the world to find a master or master racing spline.

	for (TActorIterator<APursuitSplineActor> actorItr0(world); actorItr0; ++actorItr0)
	{
		if ((gameState != nullptr && FWorldFilter::IsValid(*actorItr0, gameState) == true) ||
			(gameState == nullptr && FWorldFilter::IsValid(*actorItr0, navigationLayer) == true))
		{
			TArray<UActorComponent*> splines;

			(*actorItr0)->GetComponents(UPursuitSplineComponent::StaticClass(), splines);

			for (UActorComponent* component : splines)
			{
				UPursuitSplineComponent* spline = Cast<UPursuitSplineComponent>(component);

				if (spline->GetNumberOfSplinePoints() > 1)
				{
					if (spline->IsClosedLoop() == true)
					{
						// The first looped spline becomes the master racing spline.
						// There should only ever be one looped spline on a track (for each navigation layer).

						return spline;
					}
				}
			}
		}
	}

	return nullptr;
}

/**
* Build all of the pursuit splines.
***********************************************************************************/

void APlayGameMode::BuildPursuitSplines(bool check, const FName& navigationLayer, UWorld* world, UGlobalGameState* gameState, UPursuitSplineComponent* masterRacingSpline)
{
}

/**
* Establish all of the links between pursuit splines.
***********************************************************************************/

void APlayGameMode::EstablishPursuitSplineLinks(bool check, const FName& navigationLayer, UWorld* world, UGlobalGameState* gameState, UPursuitSplineComponent* masterRacingSpline)
{
}

/**
* Do the regular update tick, post update work for this actor, guaranteed to execute
* after other regular actor ticks.
***********************************************************************************/

void APlayGameMode::Tick(float deltaSeconds)
{
	float clock = Clock;

	Super::Tick(deltaSeconds);

	FrameTimes.AddValue(GetRealTimeClock(), deltaSeconds);

	if (clock == 0.0f)
	{
		LastOptionsResetTime = clock;
	}

	// Handle the update of each game sequence by calling the appropriate function.

	switch (GameSequence)
	{
	case EGameSequence::Initialise:
		GameSequence = EGameSequence::Start;

		// We purposefully don't break here to do the Start immediately.

	case EGameSequence::Start:
		UpdateRaceStartLine();
		UpdateRacePositions(deltaSeconds);
		break;

	case EGameSequence::Play:
		UpdateRacePositions(deltaSeconds);
		UpdateUILoading();
		break;

	case EGameSequence::End:
		UpdateRacePositions(deltaSeconds);
		UpdateUILoading();
		break;
	}

#pragma region VehicleAudio

	UpdateVehicleVolumes(deltaSeconds);

#pragma endregion VehicleAudio

}

/**
* Upload the loading of the main UI.
***********************************************************************************/

void APlayGameMode::UpdateUILoading()
{
	if (GameSequence == EGameSequence::End)
	{
		QuitGame();
	}
}

/**
* Restart the game.
***********************************************************************************/

void APlayGameMode::RestartGame()
{
	UE_LOG(GripLog, Log, TEXT("APlayGameMode::RestartGame"));

	Super::RestartGame();
}

/**
* Quit the game.
***********************************************************************************/

void APlayGameMode::QuitGame(bool force)
{
}

/**
* Calculate the race positions for each of the vehicles.
***********************************************************************************/

void APlayGameMode::UpdateRacePositions(float deltaSeconds)
{
}

/**
* Get a local player's vehicle.
***********************************************************************************/

ABaseVehicle* APlayGameMode::GetPlayerVehicle(int32 localPlayerIndex) const
{
	APlayerController* controller = UGameplayStatics::GetPlayerController(this, localPlayerIndex);

	if (controller != nullptr)
	{
		return Cast<ABaseVehicle>(controller->GetPawn());
	}
	else
	{
		return nullptr;
	}
}

/**
* Get the vehicle that is the current camera target.
***********************************************************************************/

ABaseVehicle* APlayGameMode::CameraTarget(int32 localPlayerIndex)
{
	ABaseVehicle* player = GetPlayerVehicle(localPlayerIndex);

	if (player != nullptr &&
		localPlayerIndex == 0)
	{
		AController* controller = player->GetController();

		if (controller != nullptr)
		{
			AActor* target = controller->GetViewTarget();
			ABaseVehicle* vehicle = Cast<ABaseVehicle>(target);

			if (vehicle != nullptr)
			{
				return vehicle;
			}
		}
	}

	return player;
}

/**
* Quick function for grabbing the children of a panel.
***********************************************************************************/

void APlayGameMode::GetAllWidgetsForParent(TArray<UWidget*>& widgets, UPanelWidget* panel)
{
	int32 numChildren = panel->GetChildrenCount();

	for (int32 i = 0; i < numChildren; i++)
	{
		widgets.Emplace(panel->GetChildAt(i));
	}
}

/**
* Update the player tags on the HUD.
***********************************************************************************/

void APlayGameMode::UpdatePlayerTags(APawn* owningPawn, UPanelWidget* parent)
{
}

/**
* The default ChoosePlayerStart is broken in the engine, so we override it here to
* allocate player starts serially to vehicles.
***********************************************************************************/

AActor* APlayGameMode::ChoosePlayerStartProperly(AController* player, int32 maxPlayers)
{
	if (ResetPlayerStarts == true)
	{
		Startpoints.Empty();
		UnusedStartpoints.Empty();
		ResetPlayerStarts = false;
	}

	FString startName = "";

	UWorld* world = GetWorld();

	if (Startpoints.Num() == 0)
	{
		for (TActorIterator<APlayerStart> itr(world); itr; ++itr)
		{
			APlayerStart* playerStart = *itr;

			if (FWorldFilter::IsValid(playerStart, GlobalGameState) == true)
			{
				if (playerStart->IsA<APlayerStartPIE>() == false)
				{
					Startpoints.Emplace(playerStart);
					UnusedStartpoints.Emplace(playerStart);
				}
			}
		}

		for (TActorIterator<APlayerStart> itr(world); itr; ++itr)
		{
			APlayerStart* playerStart = *itr;

			if (FWorldFilter::IsValid(playerStart, GlobalGameState) == true)
			{
				if (playerStart->IsA<APlayerStartPIE>() == true)
				{
					Startpoints.Insert(playerStart, 0);
					UnusedStartpoints.Insert(playerStart, 0);
				}
			}
		}
	}

	if (maxPlayers == 0)
	{
		maxPlayers = CalculateMaxPlayers();
	}

	if (UnusedStartpoints.Num() > 0)
	{
		if (startName.Len() > 0)
		{
			for (int32 i = 0; i < UnusedStartpoints.Num(); i++)
			{
				if (UnusedStartpoints[i]->GetName() == startName)
				{
					APlayerStart* result = UnusedStartpoints[i];

					UnusedStartpoints.RemoveAt(i);

					return result;
				}
			}
		}

		int32 index = FMath::Rand() % FMath::Max(1, FMath::Min(UnusedStartpoints.Num(), maxPlayers - (Startpoints.Num() - UnusedStartpoints.Num())));

		if (UnusedStartpoints[0]->IsA<APlayerStartPIE>())
		{
			index = 0;
		}

		APlayerStart* result = UnusedStartpoints[index];

		UnusedStartpoints.RemoveAt(index);

		return result;
	}

	return nullptr;
}

/**
* Record an event that has just occurred within the game.
***********************************************************************************/

void APlayGameMode::AddGameEvent(FGameEvent& gameEvent)
{
	// Process the event.

	gameEvent.Time = GetRealTimeClock();

	// Record the event.

	GameEvents.Emplace(gameEvent);
}

/**
* Convert a master racing spline distance to a lap distance.
***********************************************************************************/

float APlayGameMode::MasterRacingSplineDistanceToLapDistance(float distance)
{
	// Only if we've crossed the start line should be consider where in are in the lap.

	if (distance <= MasterRacingSplineStartDistance)
	{
		// If we're before the start line in the master racing spline.

		return distance + (MasterRacingSplineLength - MasterRacingSplineStartDistance);
	}
	else
	{
		// If we're after the start line in the master racing spline.

		return distance - MasterRacingSplineStartDistance;
	}
}

/**
* Project a point in world space for use on the HUD.
***********************************************************************************/

bool APlayGameMode::ProjectWorldLocationToWidgetPosition(APawn* pawn, FVector worldLocation, FVector2D& screenPosition, FMinimalViewInfo* cachedView)
{
	return false;
}

/**
* Get the difficulty characteristics for a given level, or the current level if -1
* is passed.
***********************************************************************************/

FDifficultyCharacteristics& APlayGameMode::GetDifficultyCharacteristics(int32 level)
{
	if (level < 0)
	{
		level = GlobalGameState->GetDifficultyLevel();
	}

	switch (level)
	{
	default:
		return DifficultyEasy;
	case 1:
		return DifficultyMed;
	case 2:
		return DifficultyHard;
	case 3:
		return DifficultyNeo;
	}
}

/**
* Set the graphics options into the system.
***********************************************************************************/

void APlayGameMode::SetGraphicsOptions(bool initialization)
{
	Super::SetGraphicsOptions(initialization);
}

/**
* Get a random player start point.
***********************************************************************************/

APlayerStart* APlayGameMode::GetRandomPlayerStart() const
{
	if (Startpoints.Num() > 0)
	{
		return Startpoints[FMath::Rand() % Startpoints.Num()];
	}

	return nullptr;
}

/**
* Have all the players finished the event.
***********************************************************************************/

bool APlayGameMode::HaveAllPlayersFinished() const
{
	for (ABaseVehicle* vehicle : Vehicles)
	{
		if (vehicle->GetRaceState().PlayerCompletionState < EPlayerCompletionState::Complete)
		{
			return false;
		}
	}

	return true;
}

/**
* Get the ratio of completion for the current event, 1 being fully complete.
***********************************************************************************/

float APlayGameMode::GetEventProgress()
{
	return 0.0f;
}

/**
* Get the number of players dead or alive in the game.
***********************************************************************************/

int32 APlayGameMode::GetNumOpponents(bool humansOnly)
{
	if (humansOnly == true)
	{
		int32 numHumans = 0;

		for (ABaseVehicle* vehicle : Vehicles)
		{
			if (vehicle->IsAIVehicle() == false)
			{
				numHumans++;
			}
		}

		return numHumans;
	}
	else
	{
		return Vehicles.Num();
	}
}

/**
* Update the race start line stuff, mostly the camera at this point.
***********************************************************************************/

void APlayGameMode::UpdateRaceStartLine()
{
	if (GameSequence == EGameSequence::Start)
	{
		if (Clock < StartLineDropTime)
		{
			Clock = StartLineDropTime;
		}

		bool startingGame = Clock > StartLineCountTo;

		if (startingGame == true)
		{
			// Kick off the game as we're done with the start line intro.

			GameSequence = EGameSequence::Play;

			RealTimeGameClockTicking = true;
		}
}
}

/**
* Get the countdown time for the race.
***********************************************************************************/

FText APlayGameMode::GetCountDownTime() const
{
	return FText::FromString("");
}

/**
* Get the countdown opacity for the text at the start of a race.
***********************************************************************************/

float APlayGameMode::GetCountdownOpacity() const
{
	return 0.0f;
}

/**
* Get the time left before the game starts.
***********************************************************************************/

float APlayGameMode::GetPreStartTime() const
{
	if (GameSequence <= EGameSequence::Start)
	{
		return StartLineCountTo - Clock;
	}

	return 0.0f;
}

/**
* Get the scale of the HUD.
***********************************************************************************/

float APlayGameMode::GetHUDScale() const
{
	return 0.0f;
}

#pragma region VehiclePhysicsTweaks

#if GRIP_ANTI_SKYWARD_LAUNCH

/**
* Should an actor actively limit the collision response when a vehicle collides
* with it?
***********************************************************************************/

bool APlayGameMode::ShouldActorLimitCollisionResponse(AActor* actor)
{
	if (actor == LastFrictionalActorCheck.Get())
	{
		return LastFrictionalActorCheckResult;
	}

	LastFrictionalActorCheck = actor;
	LastFrictionalActorCheckResult = FrictionalActors.Contains(actor);

	return LastFrictionalActorCheckResult;
}

#endif // GRIP_ANTI_SKYWARD_LAUNCH

#pragma endregion VehiclePhysicsTweaks

#pragma region VehicleAudio

/**
* Increase the sound volume of vehicles that are close to the local player.
* This will be capped at a max overall volume to keep things from getting drowned
* out.
***********************************************************************************/

void APlayGameMode::UpdateVehicleVolumes(float deltaSeconds)
{
	WatchedVehicles.Reset();

	// Get a list of local player camera locations.

	TArray<FVector, TInlineAllocator<16>> localPositions;

	for (ABaseVehicle* vehicle : Vehicles)
	{
		if (vehicle->LocalPlayerIndex >= 0)
		{
			FMinimalViewInfo desiredView;

			vehicle->Camera->GetCameraViewNoPostProcessing(0.0f, desiredView);

			localPositions.Emplace(desiredView.Location);

			ABaseVehicle* target = vehicle->CameraTarget();

			if (WatchedVehicles.Contains(target) == false)
			{
				WatchedVehicles.Emplace(target);
			}
		}
	}

#if !UE_BUILD_SHIPPING
	// If this isn't a shipping build, and our pawn is a spectator pawn, then override
	// the camera locations with just one, single location.

	APlayerController* controller = UGameplayStatics::GetPlayerController(GetWorld(), 0);

	if (controller != nullptr)
	{
		APawn* pawn = controller->GetPawn();

		if (IsValid(pawn) == true &&
			pawn->IsA<ASpectatorPawn>() == true)
		{
			localPositions.Empty();
			WatchedVehicles.Empty();

			localPositions.Emplace(pawn->GetActorLocation());
		}
	}
#endif // !UE_BUILD_SHIPPING

	if (localPositions.Num() > 0)
	{
		TArray<ABaseVehicle*, TInlineAllocator<16>> volumeVehicles;

		for (ABaseVehicle* vehicle : Vehicles)
		{
			if (vehicle->IsVehicleDestroyed() == false)
			{
				// Find the shortest distance to one of the player cameras.

				vehicle->GlobalVolumeRatio = 0.0f;

				for (FVector& location : localPositions)
				{
					// Normalize the distance of the vehicle between the min and max volume distances.

					float size = (vehicle->GetActorLocation() - location).Size();
					float volume = 1.0f - FMathEx::GetRatio(size, MinVehicleVolumeDistance, MaxVehicleVolumeDistance);

					vehicle->GlobalVolumeRatio = FMath::Max(vehicle->GlobalVolumeRatio, volume);
				}

				volumeVehicles.Emplace(vehicle);
			}
		}

		// For each vehicle, GlobalVolumeRatio is now the normalized linear proximity to the nearest listener
		// 1 being within MinVehicleVolumeDistance and 0 being MaxVehicleVolumeDistance or further away.

		int32 numVehicles = volumeVehicles.Num();

		if (numVehicles > 0)
		{
			// Sort the vehicles based on distance to camera, closest and therefore loudest first.

			volumeVehicles.Sort([this] (const ABaseVehicle& object1, const ABaseVehicle& object2)
				{
					return object1.GlobalVolumeRatio > object2.GlobalVolumeRatio;
				});

			// Fit the vehicles to the range of the vehicles.

			float min = volumeVehicles[volumeVehicles.Num() - 1]->GlobalVolumeRatio;
			float max = volumeVehicles[0]->GlobalVolumeRatio;
			float switchRatio = FMathEx::GetRatio(numVehicles / MaxGlobalVolume, 1.0f, 2.0f);

			for (ABaseVehicle* vehicle : volumeVehicles)
			{
				if (min != max)
				{
					vehicle->GlobalVolumeRatio = FMath::Lerp(1.0f, (vehicle->GlobalVolumeRatio - min) / (max - min), switchRatio);
				}
				else
				{
					vehicle->GlobalVolumeRatio = 1.0f;
				}
			}

			// Apply a bell curve to that fitting, so volume is biased more to the closest vehicles.

			float sum = 0.0f;
			float watchedSum = 0.0f;

			for (ABaseVehicle* vehicle : volumeVehicles)
			{
				if (WatchedVehicles.Contains(vehicle) == true)
				{
					// A watched vehicle is always top volume.

					vehicle->GlobalVolumeRatio = 1.0f;

					watchedSum += vehicle->GlobalVolumeRatio;
				}
				else
				{
					// Apply a bell curve to the volume ratio here.

					vehicle->GlobalVolumeRatio = FMath::Sin(vehicle->GlobalVolumeRatio * PI * 0.5f);
					vehicle->GlobalVolumeRatio *= vehicle->GlobalVolumeRatio;
					vehicle->GlobalVolumeRatio *= vehicle->GlobalVolumeRatio;

					sum += vehicle->GlobalVolumeRatio;
				}
			}

			// Normalize the unwatched vehicle volumes to fit the available volume space.

			float maxGlobalVolume = MaxGlobalVolume - watchedSum;

			if (sum > 0.0f)
			{
				for (ABaseVehicle* vehicle : volumeVehicles)
				{
					if (WatchedVehicles.Contains(vehicle) == false)
					{
						vehicle->GlobalVolumeRatio = FMath::Min(1.0f, (vehicle->GlobalVolumeRatio / sum) * maxGlobalVolume);
					}
				}
			}

			// Adjust the volume level of all vehicles to these new normalized values.
			// Do this swiftly but not instantaneously.

			float ratio = FMathEx::GetSmoothingRatio(0.9f, deltaSeconds);

			for (ABaseVehicle* vehicle : volumeVehicles)
			{
				vehicle->GlobalVolume = FMath::Lerp(vehicle->GlobalVolumeRatio, vehicle->GlobalVolume, ratio);
			}
		}
	}
}

#pragma endregion VehicleAudio
