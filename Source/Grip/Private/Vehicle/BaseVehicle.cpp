/**
*
* Base vehicle implementation.
*
* Original author: Rob Baker.
* Current maintainer: Rob Baker.
*
* Copyright Caged Element Inc, code provided for educational purposes only.
*
* The main vehicle class, containing almost all the meat of the vehicle
* implementation, both standard and flippable.
*
***********************************************************************************/

#include "vehicle/basevehicle.h"
#include "components/inputcomponent.h"
#include "components/widgetcomponent.h"
#include "effects/vehicleimpacteffect.h"
#include "gamemodes/playgamemode.h"
#include "runtime/umg/public/umg.h"
#include "game/globalgamestate.h"
#include "ui/hudwidget.h"
#include "gamemodes/menugamemode.h"
#include "ai/pursuitsplineactor.h"
#include "blueprint/widgetblueprintlibrary.h"
#include "pickups/shield.h"
#include "pickups/turbo.h"
#include "camera/camerapointcomponent.h"

#pragma region BlueprintAssets

TSubclassOf<AGatlingGun> ABaseVehicle::Level1GatlingGunBlueprint = nullptr;
TSubclassOf<AGatlingGun> ABaseVehicle::Level2GatlingGunBlueprint = nullptr;
TSubclassOf<AHomingMissile> ABaseVehicle::Level1MissileBlueprint = nullptr;
TSubclassOf<AHomingMissile> ABaseVehicle::Level2MissileBlueprint = nullptr;
TSubclassOf<AShield> ABaseVehicle::Level1ShieldBlueprint = nullptr;
TSubclassOf<AShield> ABaseVehicle::Level2ShieldBlueprint = nullptr;
TSubclassOf<ATurbo> ABaseVehicle::Level1TurboBlueprint = nullptr;
TSubclassOf<ATurbo> ABaseVehicle::Level2TurboBlueprint = nullptr;
TSubclassOf<AElectricalBomb> ABaseVehicle::DestroyedElectricalBomb = nullptr;
UParticleSystem* ABaseVehicle::DestroyedParticleSystem = nullptr;
UParticleSystem* ABaseVehicle::ResetEffectBlueprint = nullptr;
UParticleSystem* ABaseVehicle::LaunchEffectBlueprint = nullptr;
UParticleSystem* ABaseVehicle::HardImpactEffect = nullptr;
UParticleSystem* ABaseVehicle::DamageEffect = nullptr;
UParticleSystem* ABaseVehicle::DamageSparks = nullptr;
UMaterialInterface* ABaseVehicle::CockpitGhostMaterial = nullptr;
UMaterialInterface* ABaseVehicle::CheapCameraMaterial = nullptr;
UMaterialInterface* ABaseVehicle::ExpensiveCameraMaterial = nullptr;
USoundCue* ABaseVehicle::TeleportSound = nullptr;
USoundCue* ABaseVehicle::LaunchSound = nullptr;
USoundCue* ABaseVehicle::DestroyedSound = nullptr;
USoundCue* FVehicleHUD::HomingMissileIndicatorSound = nullptr;
USoundCue* FVehicleHUD::HomingMissileIndicatorCriticalSound = nullptr;
USoundCue* FVehicleHUD::PickupChargedSound = nullptr;
USoundCue* FVehicleHUD::PickupChargingSound = nullptr;
USoundCue* FVehicleHUD::PickupNotChargeableSound = nullptr;
USoundCue* FVehicleElimination::AlertSound = nullptr;

#pragma endregion BlueprintAssets

float ABaseVehicle::PickupHookTime = 0.5f;
bool ABaseVehicle::ProbabilitiesInitialized = false;

#pragma region Vehicle

/**
* Construct a base vehicle.
***********************************************************************************/

ABaseVehicle::ABaseVehicle()
{
	{
		static ConstructorHelpers::FObjectFinder<UMaterialInterface> asset(TEXT("Material'/Game/Vehicles/Materials/M_HMDGhostVehicle.M_HMDGhostVehicle'"));
		CockpitGhostMaterial = asset.Object;
	}

	{
		static ConstructorHelpers::FObjectFinder<USoundCue> asset(TEXT("SoundCue'/Game/Audio/Sounds/UI/A_EliminationAlert_Cue.A_EliminationAlert_Cue'"));
		Elimination.AlertSound = asset.Object;
	}

#pragma region VehicleLaunch

	{
		static ConstructorHelpers::FObjectFinder<UParticleSystem> asset(TEXT("ParticleSystem'/Game/Vehicles/Effects/Launch/PS_VehicleLaunch.PS_VehicleLaunch'"));
		LaunchEffectBlueprint = asset.Object;
	}

	{
		static ConstructorHelpers::FObjectFinder<USoundCue> asset(TEXT("SoundCue'/Game/Audio/Sounds/Vehicles/A_VehicleLaunch_Cue.A_VehicleLaunch_Cue'"));
		LaunchSound = asset.Object;
	}

#pragma endregion VehicleLaunch

#pragma region VehicleSurfaceImpacts

	{
		static ConstructorHelpers::FObjectFinder<UParticleSystem> asset(TEXT("ParticleSystem'/Game/Vehicles/Effects/VehicleImpacts/PS_HardFloorLanding.PS_HardFloorLanding'"));
		HardImpactEffect = asset.Object;
	}

#pragma endregion VehicleSurfaceImpacts

	WheelAssignments.Emplace(FWheelAssignment(("F_L_T"), EWheelPlacement::Front, 80.0f, 85.0f, 2.0f, 10.0f));
	WheelAssignments.Emplace(FWheelAssignment(("F_R_T"), EWheelPlacement::Front, 80.0f, 85.0f, 2.0f, 10.0f));
	WheelAssignments.Emplace(FWheelAssignment(("B_L_T"), EWheelPlacement::Rear, 80.0f, 85.0f, 2.0f, 10.0f));
	WheelAssignments.Emplace(FWheelAssignment(("B_R_T"), EWheelPlacement::Rear, 80.0f, 85.0f, 2.0f, 10.0f));

	// We choose to Tick post-physics because we want to be working with the very latest physics data,
	// and also to help avoid any multi-threading issues that might arise from a vehicle accessing its
	// own member data simultaneously while in the main game thread and the physics sub-step thread.

	PrimaryActorTick.bCanEverTick = true;
	PrimaryActorTick.TickGroup = TG_PostPhysics;

	VehicleMesh = CreateDefaultSubobject<UVehicleMeshComponent>(TEXT("VehicleMesh"));

	VehicleMesh->SetCollisionProfileName(UCollisionProfile::Vehicle_ProfileName);
	VehicleMesh->BodyInstance.bSimulatePhysics = true;
	VehicleMesh->BodyInstance.bContactModification = true;
	VehicleMesh->BodyInstance.bNotifyRigidBodyCollision = true;
	VehicleMesh->BodyInstance.bUseCCD = false;
	VehicleMesh->bBlendPhysics = true;

	VehicleMesh->PrimaryComponentTick.TickGroup = PrimaryActorTick.TickGroup;

	SetRootComponent(VehicleMesh);

	SpringArm = CreateDefaultSubobject<UFlippableSpringArmComponent>(TEXT("SpringArm"));
	GRIP_ATTACH(SpringArm, RootComponent, NAME_None);
	SpringArm->PrimaryComponentTick.TickGroup = PrimaryActorTick.TickGroup;

	Camera = CreateDefaultSubobject<URaceCameraComponent>(TEXT("VehicleCamera"));
	GRIP_ATTACH(Camera, SpringArm, UFlippableSpringArmComponent::SocketName);
	Camera->bUsePawnControlRotation = false;
	Camera->PrimaryComponentTick.TickGroup = PrimaryActorTick.TickGroup;

	DamageLight = CreateDefaultSubobject<UPointLightComponent>(TEXT("DamageLight"));
	GRIP_ATTACH(DamageLight, RootComponent, NAME_None);

	DestroyedExplosionForce = CreateDefaultSubobject<URadialForceComponent>(TEXT("DestroyedExplosionForce"));
	DestroyedExplosionForce->bAutoActivate = false;
	GRIP_ATTACH(DestroyedExplosionForce, RootComponent, NAME_None);

	PickedUpEffect = CreateDefaultSubobject<UParticleSystemComponent>(TEXT("PickedUpEffect"));
	PickedUpEffect->bAutoDestroy = false;
	PickedUpEffect->bAutoActivate = false;
	PickedUpEffect->SetHiddenInGame(true);
	GRIP_ATTACH(PickedUpEffect, VehicleMesh, "RootDummy");

	for (int32 i = 0; i < NumDefaultWheels; i++)
	{
		WheelOffsets.Emplace(FVector::ZeroVector);
		WheelRotations.Emplace(FRotator::ZeroRotator);
	}

#if GRIP_ENGINE_PHYSICS_MODIFIED
	OnCalculateCustomPhysics.BindUObject(this, &ABaseVehicle::SubstepPhysics);
#endif // GRIP_ENGINE_PHYSICS_MODIFIED
}

#pragma region APawn

/**
* Setup the player input.
***********************************************************************************/

void ABaseVehicle::SetupPlayerInputComponent(UInputComponent* inputComponent)
{
	int32 localPlayerIndex = DetermineLocalPlayerIndex();

	if (localPlayerIndex >= 0)
	{

#pragma region VehicleControls

		inputComponent->BindAxis("Throttle", this, &ABaseVehicle::Throttle);
		inputComponent->BindAxis("DigitalSteering", this, &ABaseVehicle::DigitalSteering);
		inputComponent->BindAxis("AnalogSteering", this, &ABaseVehicle::AnalogSteering);
		inputComponent->BindAction("Brake", IE_Pressed, this, &ABaseVehicle::HandbrakePressed);
		inputComponent->BindAction("Brake", IE_Released, this, &ABaseVehicle::HandbrakeReleased);
		inputComponent->BindAxis("PitchInput", this, &ABaseVehicle::PitchControl);

#pragma endregion VehicleControls

#pragma region VehicleSpringArm

		inputComponent->BindAxis("LookForwards", this, &ABaseVehicle::LookForwards);
		inputComponent->BindAxis("LookSideways", this, &ABaseVehicle::LookSideways);
		inputComponent->BindAction("CameraIn", IE_Pressed, this, &ABaseVehicle::CameraIn);
		inputComponent->BindAction("CameraOut", IE_Pressed, this, &ABaseVehicle::CameraOut);
		inputComponent->BindAction("LookBack", IE_Pressed, this, &ABaseVehicle::RearViewCamera);
		inputComponent->BindAction("LookBack", IE_Released, this, &ABaseVehicle::FrontViewCamera);
		inputComponent->BindAction("LookLeft", IE_Pressed, this, &ABaseVehicle::LeftViewCamera);
		inputComponent->BindAction("LookLeft", IE_Released, this, &ABaseVehicle::FrontViewCamera);
		inputComponent->BindAction("LookRight", IE_Pressed, this, &ABaseVehicle::RightViewCamera);
		inputComponent->BindAction("LookRight", IE_Released, this, &ABaseVehicle::FrontViewCamera);

#pragma endregion VehicleSpringArm

#pragma region VehicleLaunch

		inputComponent->BindAction("LaunchCharge", IE_Pressed, this, &ABaseVehicle::LaunchChargeInputOn);
		inputComponent->BindAction("LaunchCharge", IE_Released, this, &ABaseVehicle::LaunchChargeInputOff);

#pragma endregion VehicleLaunch

		APlayerController* controller = Cast<APlayerController>(GetController());

		if (GameMode != nullptr &&
			controller != nullptr)
		{
			GameMode->SetInputOptions(controller);
		}
	}
}

/**
* Do some pre initialization just before the game is ready to play.
***********************************************************************************/

void ABaseVehicle::PreInitializeComponents()
{
	UE_LOG(GripLog, Log, TEXT("ABaseVehicle::PreInitializeComponents"));

	World = GetWorld();
	GameMode = ABaseGameMode::Get(this);
	PlayGameMode = APlayGameMode::Get(this);
	GameState = UGlobalGameState::GetGlobalGameState(this);

	if (VehicleMesh != nullptr)
	{
		PhysicsBody = VehicleMesh->GetBodyInstance();

		if (PhysicsBody != nullptr)
		{
			if (PlayGameMode != nullptr)
			{
				Physics.StockMass = PhysicsBody->GetBodyMass();
				Physics.CurrentMass = Physics.CompressedMass = Physics.StockMass;

				PhysicsBody->PositionSolverIterationCount = 4;
				PhysicsBody->VelocitySolverIterationCount = 1;

				VehicleMesh->SetMassOverrideInKg(NAME_None, Physics.StockMass, true);
				VehicleMesh->SetAngularDamping(0.333f);
			}

			SetupExtraCollision();
		}
	}
}

/**
* Do some post initialization just before the game is ready to play.
***********************************************************************************/

void ABaseVehicle::PostInitializeComponents()
{
	UE_LOG(GripLog, Log, TEXT("ABaseVehicle::PostInitializeComponents"));

	Super::PostInitializeComponents();

	RaceState.HitPoints = 150;
	RaceState.MaxHitPoints = RaceState.HitPoints;

	DamageLight->SetIntensity(0.0f);

	FTransform identity;

	identity.SetIdentity();
	identity.SetScale3D(VehicleMesh->GetComponentTransform().GetScale3D());

	FTransform rootBoneTransform = VehicleMesh->GetBoneTransform(0);

	AttachedEffectsScale = FVector(1.0f, 1.0f, 1.0f) / rootBoneTransform.GetScale3D();

	AI.LastLocation = AI.PrevLocation = GetActorLocation();
	Physics.VelocityData.VelocityDirection = GetActorRotation().Vector();

	// Initial hookup, the absolute nearest point will do.

	int32 numWheels = WheelAssignments.Num();

	if (numWheels != 0)
	{
		WheelOffsets.Empty();
		WheelRotations.Empty();
	}

#pragma region VehicleContactSensors

	// Let's setup the wheels from the wheel bone assignments.

	float frontSum = 0.0f;
	float rearSum = 0.0f;

	Wheels.Wheels.Reserve(numWheels);

	ContactSensorQueryParams.bReturnPhysicalMaterial = true;

	for (const FWheelAssignment& assignment : WheelAssignments)
	{
		FName boneName = assignment.BoneName;
		int32 boneIndex = VehicleMesh->GetBoneIndex(boneName);
		EWheelPlacement placement = assignment.Placement;

		if (boneIndex != INDEX_NONE)
		{
			FVector boneOffset = VehicleMesh->GetBoneTransform(boneIndex, identity).GetLocation();
			FVector standardOffset = FVector(boneOffset.X, boneOffset.Y, 0.0f);
			FVector suspensionForcesOffset = standardOffset;

#pragma region VehiclePhysicsTweaks

			// Ensure the contact sensor itself sits half a wheel width in from the original physics asset bounds
			// which is often a little further from the vehicle body than the bone to which the wheel is located.
			// This can have a beneficial effect of stabilizing the vehicle more effectively with suspension.

			if (Physics.BodyBounds.Max.Y != 0.0f)
			{
				suspensionForcesOffset.Y = (FMath::Abs(Physics.BodyBounds.Max.Y) - assignment.Width * 0.5f) * FMathEx::UnitSign(suspensionForcesOffset.Y);
			}

#pragma endregion VehiclePhysicsTweaks

#pragma region VehicleGrip

			if (TireFrictionModel != nullptr &&
				TireFrictionModel->Model == ETireFrictionModel::Arcade)
			{
				// This standard offset is use purely for the application of grip, in order to bring predictable
				// handling to the vehicles. If we don't do this, we'll have the back-end spin-out when cornering
				// hard for example. Setting the application of grip at relatively fixed offsets around the
				// vehicle helps a lot to reduce unwanted, inconsistent behavior across different vehicles.

				standardOffset = FVector(0.0f, boneOffset.Y, 0.0f);

				if (placement == EWheelPlacement::Rear ||
					placement == EWheelPlacement::Front)
				{
					standardOffset.X = 175.0f * FMathEx::UnitSign(boneOffset.X);
				}
			}

#pragma endregion VehicleGrip

			// Create the wheel from the data we now have.

			FVehicleWheel wheel = FVehicleWheel(boneName, boneOffset, standardOffset, suspensionForcesOffset, placement, assignment.Width, assignment.Radius);

			// Determine where the front and rear axle offsets will end up.

			if (wheel.HasFrontPlacement() == true)
			{
				frontSum += 1.0f;
				Wheels.FrontAxleOffset += boneOffset.X;
			}
			else if (wheel.HasRearPlacement() == true)
			{
				rearSum += 1.0f;
				Wheels.RearAxleOffset += boneOffset.X;
			}

			// Now create the contact sensors for the wheel.

			int32 sensorIndex = 0;

			for (FVehicleContactSensor& sensor : wheel.Sensors)
			{
				sensor.Setup(this, ((sensorIndex++ == 0) ? 1 : -1), boneOffset.Y, assignment.VerticalOffset, assignment.Width, assignment.Radius, assignment.RestingCompression);
			}

			// Add the new wheel with its sensors to our internal list.

			Wheels.Wheels.Emplace(wheel);

			// Create the data required for the animation blueprint.

			WheelOffsets.Emplace(FVector::ZeroVector);
			WheelRotations.Emplace(FRotator::ZeroRotator);
		}
	}

	// Complete the calculation of where the front and rear offsets are, from the average of
	// the wheels attached to those axles.

	if (frontSum != 0.0f)
	{
		Wheels.FrontAxleOffset /= frontSum;
	}

	if (rearSum != 0.0f)
	{
		Wheels.RearAxleOffset /= rearSum;
	}

#pragma endregion VehicleContactSensors

#pragma region VehicleBasicForces

	// Record the total gravity for later to save continually computing it.

	Physics.GravityStrength = FMath::Abs(GetGravityForce(true).Z);

#pragma endregion VehicleBasicForces

	AI.OptimumSpeedExtension = FMath::Max(0.0f, (GripCoefficient - 0.5f) * 2.0f);

	if (PlayGameMode != nullptr &&
		VehicleEngineModel != nullptr)
	{
		float scale = GameState->GeneralOptions.GetEnginePowerScale(GameState->GetDifficultyLevel());

		Propulsion.MaxJetEnginePower = (VehicleEngineModel->JetEnginePower) * scale * PowerCoefficient;
		Propulsion.MaxJetEnginePowerAirborne = VehicleEngineModel->JetEnginePowerAirborne * scale * PowerCoefficient;
	}
}

/**
* Do some initialization when the game is ready to play.
***********************************************************************************/

void ABaseVehicle::BeginPlay()
{
	UE_LOG(GripLog, Log, TEXT("ABaseVehicle::BeginPlay"));

	Super::BeginPlay();

	ProbabilitiesInitialized = false;

	DetermineLocalPlayerIndex();

	CompletePostSpawn();

	TArray<UActorComponent*> components;

	GetComponents(UStaticMeshComponent::StaticClass(), components);

	for (UActorComponent* component : components)
	{
		UStaticMeshComponent* mesh = Cast<UStaticMeshComponent>(component);

		if (mesh != nullptr &&
			mesh->GetName().EndsWith("Rim"))
		{
			mesh->SetForcedLodModel(1);
		}

#pragma region VehicleSurfaceEffects

		// Find all of the tire meshes for this vehicle and associate them with their
		// relevant wheel structures.

		if ((mesh != nullptr) &&
			(mesh->GetName().EndsWith("Tire") || mesh->GetName().EndsWith("Tyre")))
		{
			FName boneName = mesh->GetAttachSocketName();
			FVehicleWheel* wheel = Wheels.Wheels.FindByKey(boneName);

			if (wheel != nullptr)
			{
				wheel->TireMesh = mesh;
			}
		}

#pragma endregion VehicleSurfaceEffects

	}

	GetComponents(UParticleSystemComponent::StaticClass(), components);

	for (UActorComponent* component : components)
	{
		UParticleSystemComponent* particles = Cast<UParticleSystemComponent>(component);

		if (particles->Template != nullptr &&
			particles->Template->GetName().Contains(TEXT("Turbo")) == true)
		{
			TurboParticleSystems.Emplace(particles);
		}
	}

#pragma region VehicleSpringArm

	TArray<int32>& racePositions = GameState->TransientGameState.RaceCameraPositions;

	if (racePositions.IsValidIndex(LocalPlayerIndex) == true)
	{
		SpringArm->CameraAt(racePositions[LocalPlayerIndex]);
	}
	else
	{
		SpringArm->CameraAt(1);
	}

#pragma endregion VehicleSpringArm

	Physics.StartLocation = GetActorLocation();
	Physics.StartRotation = GetActorRotation();

	GetComponents(ULightStreakComponent::StaticClass(), components);

	for (UActorComponent* component : components)
	{
		Cast<ULightStreakComponent>(component)->SetGlobalAmount(0.0f, 0.0f);

		ABaseGameMode::SleepComponent(Cast<ULightStreakComponent>(component));

		LightStreaks.Emplace(Cast<ULightStreakComponent>(component));
	}

	GetComponents(UCameraPointComponent::StaticClass(), components);

	for (UActorComponent* component : components)
	{
		ABaseGameMode::SleepComponent(Cast<UCameraPointComponent>(component));
	}

	static FName rootDummy = FName("RootDummy");

	RootDummyBoneIndex = VehicleMesh->GetBoneIndex(rootDummy);
}

/**
* Do some shutdown when the actor is being destroyed.
***********************************************************************************/

void ABaseVehicle::EndPlay(const EEndPlayReason::Type endPlayReason)
{
	UE_LOG(GripLog, Log, TEXT("ABaseVehicle::EndPlay"));

#pragma region VehicleSurfaceEffects

	// Destroy all of the wheel surface effects.

	for (FVehicleWheel& wheel : Wheels.Wheels)
	{
		wheel.SurfaceComponents.DestroyComponents();
		wheel.FixedSurfaceComponents.DestroyComponents();
	}

#pragma endregion VehicleSurfaceEffects

	if (PlayGameMode != nullptr)
	{
		GRIP_REMOVE_FROM_GAME_MODE_LIST_FROM(Vehicles, PlayGameMode);

		PlayGameMode->RemoveAvoidable(this);
	}

	Super::EndPlay(endPlayReason);
}

/**
* Do the regular update tick, in this case just after the physics has been done.
***********************************************************************************/

void ABaseVehicle::Tick(float deltaSeconds)
{
	Super::Tick(deltaSeconds);

	const FTransform& transform = VehicleMesh->GetComponentTransform();
	FQuat quaternion = transform.GetRotation();
	FVector xdirection = transform.GetUnitAxis(EAxis::X);
	FVector ydirection = transform.GetUnitAxis(EAxis::Y);
	FVector zdirection = transform.GetUnitAxis(EAxis::Z);

	UpdatePhysics(deltaSeconds, transform);

	// Emergency check, should always be a valid pointer for a running game though.

	if (PlayGameMode == nullptr)
	{
		return;
	}

#pragma region VehicleSpringArm

	UpdateCockpitMaterials();

#pragma endregion VehicleSpringArm

	RaceState.Tick(deltaSeconds, PlayGameMode, GameState);

	// If we're now finished playing as a result of that Tick, then hand
	// over to AI control now.

	if (AI.BotDriver == false &&
		RaceState.PlayerCompletionState >= EPlayerCompletionState::Complete)
	{
		SetAIDriver(true);
	}

#pragma region VehicleControls

	InterpolateControlInputs(deltaSeconds);

#pragma endregion VehicleControls

#pragma region VehicleDrifting

	UpdateDriftingState(deltaSeconds);

#pragma endregion VehicleDrifting

#pragma region VehicleControls

	UpdateSteering(deltaSeconds, xdirection, ydirection, quaternion);

#pragma endregion VehicleControls

#pragma region VehicleAnimation

	// Update the animated bones, mostly related to having the wheels animate with rolling,
	// steering and suspension movement.

	UpdateAnimatedBones(deltaSeconds, xdirection, ydirection);

#pragma endregion VehicleAnimation

#pragma region VehicleBasicForces

	UpdatePowerAndGearing(deltaSeconds, xdirection, zdirection);

#pragma endregion VehicleBasicForces

#pragma region VehicleAudio

	UpdateSkidAudio(deltaSeconds);

#pragma endregion VehicleAudio

#pragma region VehicleSurfaceImpacts

	UpdateHardCompression();

#pragma endregion VehicleSurfaceImpacts

#pragma region VehicleSurfaceEffects

	UpdateSurfaceEffects(deltaSeconds);

#pragma endregion VehicleSurfaceEffects

#pragma region VehicleLaunch

	UpdateLaunch(deltaSeconds);

#pragma endregion VehicleLaunch

	UpdateIdleLock();

	AI.LastVehicleContacts = AI.VehicleContacts;
	AI.LastCollisionBlockage = AI.CollisionBlockage;
	AI.LastHardCollisionBlockage = AI.HardCollisionBlockage;

	AI.VehicleContacts = VehicleUnblocked;
	AI.CollisionBlockage = VehicleUnblocked;
	AI.HardCollisionBlockage = VehicleUnblocked;
}

/**
* Receive hit information from the collision system.
***********************************************************************************/

void ABaseVehicle::NotifyHit(class UPrimitiveComponent* thisComponent, class AActor* other, class UPrimitiveComponent* otherComponent, bool selfMoved, FVector hitLocation, FVector hitNormal, FVector normalForce, const FHitResult& hitResult)
{
	normalForce *= 1.0f / CustomTimeDilation;

	Super::NotifyHit(thisComponent, other, otherComponent, selfMoved, hitLocation, hitNormal, normalForce, hitResult);

	if (hitResult.IsValidBlockingHit() == true)
	{

#pragma region VehicleSurfaceImpacts

		if (PlayGameMode != nullptr &&
			PlayGameMode->PastGameSequenceStart() == true)
		{
			if (DrivingSurfaceImpactCharacteristics != nullptr &&
				normalForce.Size() > ImpactEffectNormalForceThreshold)
			{
				// If the impact force is strong enough then spawn an impact effect.

				if (VehicleClock - Physics.LastHit > 0.25f)
				{
					Physics.LastHit = VehicleClock;

					// Calculate the relative velocities of the two components involved in this collision.

					ABaseVehicle* otherVehicle = Cast<ABaseVehicle>(otherComponent->GetOwner());

					FVector v0 = VehicleMesh->GetPhysicsLinearVelocity();
					FVector v1 = (otherVehicle != nullptr) ? otherVehicle->VehicleMesh->GetPhysicsLinearVelocity() : otherComponent->GetComponentVelocity();
					FVector velocity = (v0.SizeSquared() < v1.SizeSquared()) ? v0 : v1;

					if (velocity.IsNearlyZero() == false)
					{
						// As long as the lowest velocity isn't zero then take the highest instead.
						// Not sure why, but the velocity taken by the effect is not keeping up with
						// the vehicle even when taking the highest, let alone the lowest, but it
						// seems to fit better in any event.

						velocity = (v0.SizeSquared() > v1.SizeSquared()) ? v0 : v1;
					}

					if (otherVehicle != nullptr)
					{
						// If what we hit was another vehicle then calculate a new hit normal based
						// on the launch direction of this vehicle and the velocity vector. This will
						// work better with vehicle / vehicle collisions, showing more of the effect.

						FVector forward = velocity;

						forward.Normalize();

						hitNormal = GetLaunchDirection();
						hitNormal += forward * 0.5f;

						hitNormal.Normalize();
					}

					// Finally spawn the surface impact effect with all of the relevant data.

					SpawnSurfaceImpactEffect(hitLocation, hitNormal, hitResult, velocity, normalForce.Size() / 50000000.0f, false);
				}
			}
		}

#pragma endregion VehicleSurfaceImpacts

	}
}

#pragma endregion APawn

#pragma region VehiclePhysics

/**
* Update the physics portion of the vehicle.
***********************************************************************************/

void ABaseVehicle::UpdatePhysics(float deltaSeconds, const FTransform& transform)
{
	// This feels wrong adding custom physics every tick, but it's exactly right.

	PhysicsBody = VehicleMesh->GetBodyInstance();

	if (PhysicsBody != nullptr)
	{
#if GRIP_ENGINE_PHYSICS_MODIFIED
		PhysicsBody->AddCustomPhysics(OnCalculateCustomPhysics);
#else // GRIP_ENGINE_PHYSICS_MODIFIED
		SubstepPhysics(deltaSeconds, PhysicsBody);
#endif // GRIP_ENGINE_PHYSICS_MODIFIED
	}

	if (IsVehicleDestroyed() == true)
	{
		SetActorLocation(Physics.StaticHold.Location, false, nullptr, ETeleportType::TeleportPhysics);
	}

	Wheels.FrontAxlePosition = transform.TransformPosition(FVector(Wheels.FrontAxleOffset, 0.0f, 0.0f));
	Wheels.RearAxlePosition = transform.TransformPosition(FVector(Wheels.RearAxleOffset, 0.0f, 0.0f));

	VehicleClock += deltaSeconds;
	Physics.Drifting.Timer += deltaSeconds;

	if (PlayGameMode != nullptr)
	{
		int32 totalVehicles = PlayGameMode->GetVehicles().Num();

		Clock0p5.Tick(VehicleIndex, totalVehicles);
		Clock0p25.Tick(VehicleIndex, totalVehicles);
		Clock0p1.Tick(VehicleIndex, totalVehicles);

#pragma region VehicleBasicForces

		if (PlayGameMode->PastGameSequenceStart() == false)
		{
			// Lock the vehicle down until the game has started.

			ArrestVehicle();
		}
		else
		{
			Physics.StaticHold.Active = false;
		}

#pragma endregion VehicleBasicForces

	}

	if (Physics.Timing.TickCount > 0)
	{
		Physics.Timing.GeneralTickSum += deltaSeconds;
		Physics.Timing.GeneralTickCount++;

		// If we have an impulse to apply which we've built-up during the physics sub-step
		// then apply it now.

		if (Physics.ApplyImpulse != FVector::ZeroVector)
		{
			VehicleMesh->IdleUnlock();
			VehicleMesh->AddImpulse(Physics.ApplyImpulse);

			Physics.ApplyImpulse = FVector::ZeroVector;

			if (ShieldChargedImpactSound != nullptr)
			{
				AShield* shield = Level2ShieldBlueprint->GetDefaultObject<AShield>();

				ShieldChargedImpactSound = shield->ChargedImpact;
			}

			UGameplayStatics::SpawnSoundAttached(ShieldChargedImpactSound, VehicleMesh, NAME_None, FVector::ZeroVector, EAttachLocation::Type::KeepRelativeOffset);
		}
	}

	ContactPoints[1].Reset();
	ContactForces[1].Reset();

	ContactPoints[1] = ContactPoints[0];
	ContactForces[1] = ContactForces[0];

	ContactPoints[0].Reset();
	ContactForces[0].Reset();
}

/**
* Is the vehicle currently with all wheels off the ground?
***********************************************************************************/

bool ABaseVehicle::IsAirborne(bool ignoreSkipping)
{
	if (ignoreSkipping == false)
	{
		return Physics.ContactData.Airborne;
	}
	else
	{

#pragma region VehicleContactSensors

		for (FVehicleWheel& wheel : Wheels.Wheels)
		{
			// If any wheel is some distance from the ground then return the physics airborne state.

			if (wheel.GetActiveSensor().HasNearestContactPoint(wheel.Velocity, 0.0f) == false ||
				wheel.GetActiveSensor().GetNearestContactPointDistance() > (wheel.Radius + HoverDistance) * 2.0f)
			{
				return Physics.ContactData.Airborne;
			}
		}

		// Otherwise assume we're grounded.

#pragma endregion VehicleContactSensors

		return false;
	}
}

/**
* Setup any additional collision structures for the vehicle.
***********************************************************************************/

void ABaseVehicle::SetupExtraCollision()
{
	if (VehicleMesh != nullptr &&
		PhysicsBody != nullptr)
	{
		const float maxDepenetration = 200.0f;

		PhysicsBody->SetMaxDepenetrationVelocity(maxDepenetration);
		PhysicsBody->SetContactModification(true);

		UPhysicalMaterial* material = PhysicsBody->GetSimplePhysicalMaterial();

#if GRIP_ENGINE_PHYSICS_MODIFIED
		Physics.BodyBounds = PhysicsBody->CalculateLocalBounds(VehicleMesh->GetComponentTransform().GetScale3D());
#else // GRIP_ENGINE_PHYSICS_MODIFIED
		Physics.BodyBounds = FBox(VehicleMesh->Bounds.BoxExtent * -1.0f, VehicleMesh->Bounds.BoxExtent);
#endif // GRIP_ENGINE_PHYSICS_MODIFIED

		material->Friction = 0.0f;
		material->bOverrideFrictionCombineMode = true;
		material->FrictionCombineMode = EFrictionCombineMode::Type::Min;

		material->Restitution = 0.0f;
		material->bOverrideRestitutionCombineMode = true;
		material->RestitutionCombineMode = EFrictionCombineMode::Type::Min;

		FPhysicsInterface::UpdateMaterial(material->GetPhysicsMaterial(), material);

		FVector extent = (VehicleMesh->SkeletalMesh->GetImportedBounds().BoxExtent * 0.5f);

		for (const FWheelAssignment& assignment : WheelAssignments)
		{
			int32 boneIndex = VehicleMesh->GetBoneIndex(assignment.BoneName);

			if (boneIndex != INDEX_NONE)
			{
				FTransform identity;

				identity.SetIdentity();
				identity.SetScale3D(VehicleMesh->GetComponentTransform().GetScale3D());

				FVector boneOffset = VehicleMesh->GetBoneTransform(boneIndex, identity).GetLocation();

				extent.X = FMath::Max(extent.X, FMath::Abs(boneOffset.X) + (assignment.Width * 0.5f));
				extent.Y = FMath::Max(extent.Y, FMath::Abs(boneOffset.Y) + (assignment.Width * 0.5f));
			}

			if (Antigravity == false)
			{
				extent.Z = FMath::Max(extent.Z, assignment.Radius);
			}
		}

		extent += FVector(10.0f, 10.0f, 10.0f);

		CameraClipBox = FBox(extent * -1.0f, extent);

		BoundingExtent = extent + FVector(5.0f, 5.0f, 10.0f);

#pragma region VehicleCollision

		// Scale and expand the box extent for a new vehicle / vehicle collision component.

		extent /= VehicleMesh->GetRelativeScale3D();
		extent += FVector(5.0f, 5.0f, 10.0f);

		// Create a new box component to handle the vehicle / vehicle collision.

		VehicleCollision = NewObject<UBoxComponent>(this, TEXT("VehicleShell"));

		// Ensure that we set the profile to VehicleShell so it has the correct collision detection properties.

		VehicleCollision->SetCollisionProfileName((PlayGameMode != nullptr) ? "VehicleShell" : "NoCollision");
		VehicleCollision->SetBoxExtent(extent);
		VehicleCollision->SetHiddenInGame(true);
		VehicleCollision->SetLinearDamping(0.0f);
		VehicleCollision->SetAngularDamping(0.0f);
		VehicleCollision->SetEnableGravity(false);
		VehicleCollision->SetMassOverrideInKg(NAME_None, 1.0f, true);
		VehicleCollision->SetGenerateOverlapEvents(true);
		VehicleCollision->ShapeColor = FColor::Green;

		// Now setup the body instance for this box component and ensure that we have contact modification enabled.

		VehicleCollision->GetBodyInstance()->bNotifyRigidBodyCollision = true;
		VehicleCollision->GetBodyInstance()->SetContactModification(true);
		VehicleCollision->GetBodyInstance()->SetEnableGravity(false);
		VehicleCollision->GetBodyInstance()->SetMaxDepenetrationVelocity(maxDepenetration);
		VehicleCollision->GetBodyInstance()->SetPhysMaterialOverride(material);

#if GRIP_ENGINE_PHYSICS_MODIFIED
		VehicleCollision->GetBodyInstance()->bCentraliseMass = true;
#endif // GRIP_ENGINE_PHYSICS_MODIFIED

		GRIP_ATTACH(VehicleCollision, VehicleMesh, NAME_None);

		VehicleCollision->RegisterComponent();

#pragma endregion VehicleCollision

	}
}

#pragma endregion VehiclePhysics

#pragma region VehicleContactSensors

/**
* Get the name of a surface from its type.
***********************************************************************************/

FName ABaseVehicle::GetNameFromSurfaceType(EGameSurface surfaceType)
{
	static FName Asphalt("Asphalt");
	static FName Dirt("Dirt");
	static FName Water("Water");
	static FName Rock("Rock");
	static FName Wood("Wood");
	static FName Metal("Metal");
	static FName Grass("Grass");
	static FName Gravel("Gravel");
	static FName Sand("Sand");
	static FName Snow("Snow");
	static FName Field("Field");
	static FName Default("Default");
	static FName Tractionless("Tractionless");
	static FName Unknown("Unknown");

	switch (surfaceType)
	{
	case EGameSurface::Asphalt:
		return Asphalt;
	case EGameSurface::Dirt:
		return Dirt;
	case EGameSurface::Water:
		return Water;
	case EGameSurface::Wood:
		return Wood;
	case EGameSurface::Rock:
		return Rock;
	case EGameSurface::Metal:
		return Metal;
	case EGameSurface::Grass:
		return Grass;
	case EGameSurface::Gravel:
		return Gravel;
	case EGameSurface::Sand:
		return Sand;
	case EGameSurface::Snow:
		return Snow;
	case EGameSurface::Field:
		return Field;
	case EGameSurface::Default:
		return Default;
	case EGameSurface::Tractionless:
		return Tractionless;
	default:
		return Unknown;
	}
}

/**
* Is the vehicle currently with all wheels (more or less) on the ground?
***********************************************************************************/

bool ABaseVehicle::IsPracticallyGrounded(float distance, bool anyWheel)
{
	if (anyWheel == true)
	{
		for (FVehicleWheel& wheel : Wheels.Wheels)
		{
			if (wheel.IsInContact == true)
			{
				return true;
			}
			else
			{
				if (wheel.GetActiveSensor().HasNearestContactPoint(wheel.Velocity, 0.0f) == true)
				{
					if (wheel.GetActiveSensor().GetNearestContactPointDistanceFromTire() < distance)
					{
						return true;
					}
				}
			}
		}

		return false;
	}
	else
	{
		for (FVehicleWheel& wheel : Wheels.Wheels)
		{
			if (wheel.IsInContact == false)
			{
				if (wheel.GetActiveSensor().HasNearestContactPoint(wheel.Velocity, 0.0f) == true)
				{
					if (wheel.GetActiveSensor().GetNearestContactPointDistanceFromTire() > distance)
					{
						return false;
					}
				}
				else
				{
					return false;
				}
			}
		}

		return true;
	}
}

/**
* Get the direction from the vehicle to the nearest driving surface.
***********************************************************************************/

FVector ABaseVehicle::GetSurfaceDirection()
{
	if (GetNumWheels() > 0)
	{
		// All wheels have the same direction, and this will be pointing towards the
		// nearest surface, even though the direction vector that describes the
		// shortest distance to that surface may be something different.

		return Wheels.Wheels[0].GetActiveSensor().GetDirection();
	}

	return GetUpDirection() * -1.0f;
}

/**
* Get the direction from the vehicle to launch weapons from, often opposing the
* nearest surface direction.
***********************************************************************************/

FVector ABaseVehicle::GetLaunchDirection(bool inContact) const
{
	// All wheels have the same direction, and this will be pointing towards the
	// nearest surface, even though the direction vector that describes the
	// shortest distance to that surface may be something different.

	for (const FVehicleWheel& wheel : Wheels.Wheels)
	{
		if ((inContact == false || wheel.GetActiveSensor().IsInContact() == true) &&
			(wheel.GetActiveSensor().HasNearestContactPoint(wheel.Velocity, 0.0f) == true))
		{
			return wheel.GetActiveSensor().GetDirection() * -1.0f;
		}
	}

	FVector zdirection = GetUpDirection();

	return ((zdirection.Z >= 0.0f) ? zdirection : zdirection * -1.0f);
}

/**
* Get the location of the nearest driving surface to the center of the vehicle.
***********************************************************************************/

FVector ABaseVehicle::GetSurfaceLocation() const
{
	FVector normal = FVector::ZeroVector;
	FVector location = GetCenterLocation();

	if (GetNumWheels() >= 3)
	{
		// This assumes all of the wheels have contacts on their active sensors.

		const FVector& direction = Wheels.Wheels[0].GetActiveSensor().GetDirection();
		const FVector& p0 = Wheels.Wheels[0].GetActiveSensor().GetNearestContactPoint();
		const FVector& p1 = Wheels.Wheels[1].GetActiveSensor().GetNearestContactPoint();
		const FVector& p2 = Wheels.Wheels[2].GetActiveSensor().GetNearestContactPoint();

		// Take the contact locations of 3 of the wheels and take a surface normal
		// away from the plane that the 3 contacts form.

		normal = FVector::CrossProduct(p1 - p0, p2 - p0);

		normal.Normalize();

		// Ensure the plane normal is pointing in the correct direction, towards the
		// center location from the plane's location.

		if (FVector::DotProduct(direction, normal) > 0.0f)
		{
			normal *= -1.0f;
		}

		// Now project the center location onto that imaginary plane and return the result.

		return FVector::PointPlaneProject(location, p0, normal);
	}

	return location;
}

/**
* Get the normal of the nearest driving surface.
***********************************************************************************/

FVector ABaseVehicle::GetSurfaceNormal() const
{
	FVector normal = FVector::ZeroVector;

	if (GetNumWheels() >= 3)
	{
		// This assumes all of the wheels have contacts on their active sensors.

		const FVector& direction = Wheels.Wheels[0].GetActiveSensor().GetDirection();
		const FVector& p0 = Wheels.Wheels[0].GetActiveSensor().GetNearestContactPoint();
		const FVector& p1 = Wheels.Wheels[1].GetActiveSensor().GetNearestContactPoint();
		const FVector& p2 = Wheels.Wheels[2].GetActiveSensor().GetNearestContactPoint();

		// Take the contact locations of 3 of the wheels and take a surface normal
		// away from the plane that the 3 contacts form.

		normal = FVector::CrossProduct(p1 - p0, p2 - p0);

		normal.Normalize();

		// Ensure the normal is pointing in the correct direction, towards the vehicle.

		if (FVector::DotProduct(direction, normal) > 0.0f)
		{
			normal *= -1.0f;
		}
	}

	return normal;
}

/**
* Guess the normal of the nearest driving surface.
***********************************************************************************/

FVector ABaseVehicle::GuessSurfaceNormal() const
{
	FVector normal = FVector::ZeroVector;
	int32 numWheels = GetNumWheels();

	// OK, so sometimes we need to know what the surface normal is of the nearest
	// surface even if we're not in good contact with one. As long as we have 3
	// wheels where the contact sensors have sensed a surface we can do this.

	if (numWheels >= 4)
	{
		// Determine which of the wheels have a surface contact detected.

		TArray<FVector> contacts;

		// #TODO: This will break if you pick 3 wheels on a single axle. This will never
		// happen in GRIP though.

		for (const FVehicleWheel& wheel : Wheels.Wheels)
		{
			if (wheel.GetActiveSensor().HasNearestContactPoint(FVector::ZeroVector, 0.0f) == true)
			{
				contacts.Emplace(wheel.GetActiveSensor().GetNearestContactPoint());

				if (contacts.Num() >= 3)
				{
					normal = FVector::CrossProduct(contacts[1] - contacts[0], contacts[2] - contacts[0]);

					normal.Normalize();

					if (FVector::DotProduct(wheel.GetActiveSensor().GetDirection(), normal) > 0.0f)
					{
						normal *= -1.0f;
					}

					return normal;
				}
			}
		}
	}

	return normal;
}

/**
* Do we have a valid surface contact, optionally over a period of seconds.
***********************************************************************************/

bool ABaseVehicle::IsSurfaceDirectionValid(float contactSeconds)
{
	for (FVehicleWheel& wheel : Wheels.Wheels)
	{
		if (wheel.GetActiveSensor().HasNearestContactPoint(wheel.Velocity, contactSeconds))
		{
			return true;
		}
	}

	return false;
}

/**
* Get the average distance of the wheels from the vehicle to the nearest driving
* surface, 0 for not near any driving surface.
***********************************************************************************/

float ABaseVehicle::GetSurfaceDistance(bool discountFrontWheelsWhenRaised, bool closest)
{
	float sum = 0.0f;
	float averageDistance = 0.0f;
	float minDistance = 0.0f;

	for (FVehicleWheel& wheel : Wheels.Wheels)
	{
		float distance = wheel.GetActiveSensor().GetSurfaceDistance();

		// This hack here is to try to keep the vehicle on the ceiling when doing a charged turbo,
		// nothing more than that really.

		if (wheel.HasRearPlacement() == false &&
			discountFrontWheelsWhenRaised == true &&
			Propulsion.RaiseFrontScale > KINDA_SMALL_NUMBER)
		{
			distance = wheel.Radius;
		}

		if (distance != 0.0f)
		{
			sum += 1.0f;
			averageDistance += distance;

			if (minDistance == 0.0f ||
				minDistance > distance)
			{
				minDistance = distance;
			}
		}
	}

	if (sum != 0.0f)
	{
		averageDistance /= sum;
	}

	if (closest == true)
	{
		return minDistance;
	}
	else
	{
		return averageDistance;
	}
}

/**
* Get the location of the bone for a wheel, in world space.
* Optionally clipped on the Y axis to within the bounds of the collision shape.
***********************************************************************************/

FVector ABaseVehicle::GetWheelBoneLocation(const FVehicleWheel& wheel, const FTransform& transform, bool clipToCollision)
{
	if (clipToCollision == true)
	{
		FVector offset = wheel.BoneOffset;

		if (FMath::Abs(offset.Y) > FMath::Abs(wheel.SuspensionForcesOffset.Y))
		{
			offset.Y = wheel.SuspensionForcesOffset.Y;
		}

		return transform.TransformPosition(offset);
	}
	else
	{
		return transform.TransformPosition(wheel.BoneOffset);
	}
}

/**
* Get the location to apply suspension forces to for a particular wheel in world
* space.
***********************************************************************************/

FVector ABaseVehicle::GetSuspensionForcesLocation(const FVehicleWheel& wheel, const FTransform& transform, float deltaSeconds)
{
	FVector offset = wheel.SuspensionForcesOffset;

	return transform.TransformPosition(offset);
}

/**
* Get how much grip we should apply to a particular contact sensor at this time.
***********************************************************************************/

float ABaseVehicle::GetGripRatio(const FVehicleContactSensor& sensor) const
{
	{
		if (sensor.IsInContact() == true)
		{
			return TireFrictionModel->GripVsSuspensionCompression.GetRichCurve()->Eval(sensor.GetNormalizedCompression());
		}
		else
		{
			return 0.0f;
		}
	}
}

#pragma endregion VehicleContactSensors

#pragma region VehicleBasicForces

/**
* Arrest the vehicle until the game has started.
***********************************************************************************/

void ABaseVehicle::ArrestVehicle()
{
	if (Physics.StaticHold.Active == false &&
		Physics.ContactData.ModeTime > 1.0f &&
		Physics.VelocityData.Speed < 100.0f)
	{
		if (Physics.StaticHold.Active == false)
		{
			Physics.StaticHold.Location = VehicleMesh->GetPhysicsLocation();
			Physics.StaticHold.Rotation = VehicleMesh->GetPhysicsQuaternion();
		}

		Physics.StaticHold.Active = true;
	}

	if (Physics.StaticHold.Active == true)
	{
		if (Wheels.BurnoutForce > 0.0f)
		{
			Physics.StaticHold.Location = VehicleMesh->GetPhysicsLocation();
			Physics.StaticHold.Rotation = VehicleMesh->GetPhysicsQuaternion();
		}
	}
}

/**
* Update the power and gearing, returns true if just shifted up a gear.
***********************************************************************************/

void ABaseVehicle::UpdatePowerAndGearing(float deltaSeconds, const FVector& xdirection, const FVector& zdirection)
{
	if (IsVehicleDestroyed() == false)
	{
		int32 topGear = FMath::Max(VehicleEngineModel->GearPowerRatios.Num(), 7) - 1;

#pragma region VehicleAudio

		if (VehicleAudio != nullptr)
		{
			// Ensure we have enough gears in the audio, by replicating them where we need to.

			while (VehicleAudio->Gears.Num() > 0 &&
				VehicleAudio->Gears.Num() <= topGear)
			{
				VehicleAudio->Gears.Emplace(VehicleAudio->Gears[VehicleAudio->Gears.Num() - 1]);
			}

			topGear = VehicleAudio->Gears.Num() - 1;
		}

#pragma endregion VehicleAudio

		float speed = GetSpeedKPH();
		float measuredGearPosition = speed / GetGearSpeedRange();
		float acceleration = (AI.Speed.DifferenceFromPerSecond(VehicleClock - 0.2f, VehicleClock, GetSpeedMPS() * 100.0f) / 100.0f);
		bool accelerating = (acceleration > -1.0f && Control.ThrottleInput > 0.25f);
		bool decelerating = (acceleration < -1.0f && Control.ThrottleInput < 0.25f);

		// measuredGearPosition contains the gear and the fraction within that gear.

		int32 gear = FMath::FloorToInt(measuredGearPosition);

		// gear is the integral gear we are currently using.

		Propulsion.CurrentGearPosition = measuredGearPosition - gear;

		// CurrentGearPosition is the fraction of the current gear, 1 being max revs.

		Propulsion.GearTime += deltaSeconds;

		// GearTime is the time spent within the current gear.

		// The amount of overlap to give between gears when accelerating or decelerating.

		float revOverlap = 0.333f;
		float currentGearPosition = Propulsion.CurrentGearPosition;
		bool grounded = IsPracticallyGrounded(100.0f);

		// Don't let gear changes happen too frequently, we don't want that
		// awful high-speed switching between gears that can sometimes occur
		// during hard cornering.

		bool keepGear = (Propulsion.GearTime < 1.0f);

		if (gear >= topGear)
		{
			gear = topGear;
			currentGearPosition = measuredGearPosition - topGear;
		}
		else
		{
			// Determine if we're going up or down the gearbox and then over-rev
			// at the top of a gear if accelerating and under-rev at the bottom of a
			// gear if decelerating. Give time between gear changes so you can see if
			// a change is required (rev high where possible).

			if (accelerating == true)
			{
				keepGear |= (gear == Propulsion.LastGear + 1 && currentGearPosition < revOverlap);
			}

			if (decelerating == true)
			{
				keepGear |= (gear == Propulsion.LastGear - 1 && currentGearPosition > 1.0f - revOverlap);
			}
		}

		if (keepGear == true)
		{
			if (gear > Propulsion.LastGear)
			{
				// We're overrevving.

				currentGearPosition += gear - Propulsion.LastGear;
				currentGearPosition = FMath::Min(currentGearPosition, 1.0f + revOverlap);
			}
			else if (gear < Propulsion.LastGear)
			{
				// We're underrevving.

				currentGearPosition -= Propulsion.LastGear - gear;
				currentGearPosition = FMath::Max(currentGearPosition, -revOverlap);
			}

			gear = Propulsion.LastGear;
		}

		// Calculate the launch boost to boost the overall engine power.

		float launchBoostPower = 1.0f;

		// In low gears, the more away from the flat, the more power we give.
		// The reason being, it's hard to accelerate up a steep hill in low gear.

		float inclineHelp = 0.0f;
		float inclineHelpMax = 0.6f;

		if (Propulsion.PistonEngineThrottle > 0.0f)
		{
			// If propelling forwards.
			// If facing downhill then don't do anything, otherwise give more power the more we're facing uphill.

			inclineHelp = (xdirection.Z < 0.0f) ? 0.0f : (FMath::Min(xdirection.Z, inclineHelpMax) / inclineHelpMax);
		}
		else
		{
			// If propelling backwards.
			// If facing uphill then don't do anything, otherwise give more power the more we're facing downhill.

			inclineHelp = (xdirection.Z > 0.0f) ? 0.0f : (FMath::Min(-xdirection.Z, inclineHelpMax) / inclineHelpMax);
		}

		// Translate the position to a based on a power curve for now.
		// This means low power at beginning of gear and high power at the end.
		// By 4th gear, we are often producing maximum power throughout the gear range.

		float maxJetEnginePower = Propulsion.MaxJetEnginePower * launchBoostPower;
		float gearPower = FMath::Lerp(0.0f, 1.0f, inclineHelp);
		float gearPowerRatio = 1.0f;

		if (VehicleEngineModel->GearPowerRatios.Num() > gear)
		{
			gearPowerRatio = VehicleEngineModel->GearPowerRatios[gear];
#if GRIP_STATIC_ACCELERATION

#pragma region VehiclePhysicsTweaks

			// With low-powered vehicles, the low-speed acceleration felt too weak for many players,
			// even though the top speed was fast enough. So here, we're giving the low-powered vehicles
			// the same low-speed acceleration characteristics as a high-powered vehicle.

			if (gearPowerRatio < 1.0f &&
				GameState->GeneralOptions.EnginePowerLevel < 2)
			{
				float p0 = GameState->GeneralOptions.GetEnginePowerScale(GameState->GetDifficultyLevel());
				float p1 = GameState->GeneralOptions.GetEnginePowerScale(GameState->GetDifficultyLevel(), 2);

				if (p0 < p1)
				{
					gearPowerRatio *= p1 / p0;
				}
			}

#pragma endregion VehiclePhysicsTweaks

#endif // GRIP_STATIC_ACCELERATION
		}

		if (gearPowerRatio < 1.0f - KINDA_SMALL_NUMBER)
		{
			gearPowerRatio *= AccelerationCoefficient;
		}

		gearPowerRatio = FMath::Min(1.0f, gearPowerRatio);

		float enginePower = maxJetEnginePower;
		float lowPower = FMath::Lerp(enginePower * gearPowerRatio, enginePower, gearPower);

		Propulsion.CurrentJetEnginePower = FMath::Lerp(lowPower, (IsAirborne() == true) ? Propulsion.MaxJetEnginePowerAirborne : maxJetEnginePower, FMath::Pow(FMath::Max(currentGearPosition, 0.0f), 1.5f));

		float j0 = Propulsion.CurrentJetEnginePower;

		// So now we've got all the engine power calculated, let's manage the gearing simulation.

		bool hasStarted = Propulsion.HasStarted;
		float throttleInput = Control.ThrottleInput;

		if (PlayGameMode != nullptr &&
			PlayGameMode->PastGameSequenceStart() == false)
		{
			hasStarted |= AI.WillRevOnStartLine;
		}

#pragma region VehicleAudio

		// Manage the engine audio.

		if (VehicleAudio != nullptr)
		{
			TArray<FVehicleAudioGear>& gears = VehicleAudio->Gears;

			if (gears.Num() > 0)
			{
				if (hasStarted == false)
				{
					// If we haven't started yet then idle.

					Propulsion.IdleTransitionDirection = -1.0f;
				}
				else if (FMath::Abs(throttleInput) < KINDA_SMALL_NUMBER && speed < 10.0f)
				{
					// If we're going real slow and not applying power then idle.

					Propulsion.IdleTransitionDirection = -1.0f;
				}
				else
				{
					// Otherwise don't idle.

					Propulsion.IdleTransitionDirection = 1.0f;
				}

				if (grounded == false)
				{
					// We're in the air, so let the engine only run in its last gear
					// when on the ground, but spin the engine up / down depending on
					// whether the throttle is being pressed.

					float airborneScale = 0.5f;
					float gearPosition = Propulsion.LastGearPosition;

					if (FMath::Abs(throttleInput) > 0.25f)
					{
						gearPosition = FMath::Min(gearPosition + (deltaSeconds * FMath::Abs(throttleInput) * airborneScale), 1.0f + revOverlap);
						currentGearPosition = FMath::Max(currentGearPosition, gearPosition);
					}
					else
					{
						gearPosition = FMath::Max(gearPosition - (deltaSeconds * airborneScale), -revOverlap);
						currentGearPosition = FMath::Min(currentGearPosition, gearPosition);
					}

					gear = Propulsion.LastGear;
				}

				Propulsion.LastGearPosition = currentGearPosition;

				// Choose gear audio based on whether or not we're an AI driver.

				FVehicleAudioGear& gearAudio = gears[FMath::Min(gear, gears.Num() - 1)];

				if (PlayGameMode != nullptr &&
					PlayGameMode->PastGameSequenceStart() == false)
				{
					// Simulated engine revving on the start line for AI bots.

					currentGearPosition = AI.TorqueRoll;
				}

				// This is the normal gear pitch range.

				float minPitch = gearAudio.MinEnginePitch;
				float maxPitch = gearAudio.MaxEnginePitch;
				float pitchRange = maxPitch - minPitch;

				maxPitch -= pitchRange * revOverlap;
				pitchRange = maxPitch - minPitch;

				float enginePitch = FMath::Lerp(minPitch, maxPitch, currentGearPosition);

				Propulsion.CurrentGearPosition = FMathEx::GetRatio(enginePitch, minPitch - (pitchRange * revOverlap), maxPitch + (pitchRange * revOverlap));

				static FName rpmParameter("GearPosition");
				static FName kphParameter("KPH");
				static FName throttleParameter("Throttle");

				float appliedThrottle = FMath::Lerp(FMath::Abs(Control.ThrottleInput), 0.0f, Control.BrakePosition);

				if (Propulsion.LastGear != gear)
				{
					// Handle a gear change in the audio.

					Propulsion.GearTime = 0.0f;

					EngineAudioIndex ^= 1;

					// Play the engine sound for the new gear.

					GRIP_STOP_IF_PLAYING(PistonEngineAudio[GRIP_VEHICLE_AUDIO_GEAR_C(EngineAudioIndex)]);

					LastGearPitch = enginePitch;

					PistonEngineAudio[GRIP_VEHICLE_AUDIO_GEAR_C(EngineAudioIndex)]->SetSound(gearAudio.EngineSound);
					PistonEngineAudio[GRIP_VEHICLE_AUDIO_GEAR_C(EngineAudioIndex)]->SetVolumeMultiplier(0.0f);
					PistonEngineAudio[GRIP_VEHICLE_AUDIO_GEAR_C(EngineAudioIndex)]->SetPitchMultiplier(LastGearPitch);
					PistonEngineAudio[GRIP_VEHICLE_AUDIO_GEAR_C(EngineAudioIndex)]->SetFloatParameter(rpmParameter, Propulsion.CurrentGearPosition);
					PistonEngineAudio[GRIP_VEHICLE_AUDIO_GEAR_C(EngineAudioIndex)]->SetFloatParameter(kphParameter, speed);
					PistonEngineAudio[GRIP_VEHICLE_AUDIO_GEAR_C(EngineAudioIndex)]->SetFloatParameter(throttleParameter, appliedThrottle);
					PistonEngineAudio[GRIP_VEHICLE_AUDIO_GEAR_C(EngineAudioIndex)]->Play();

					// Handle the gear change up / down sounds.

					GearShiftAudio->SetSound((Propulsion.LastGear < gear) ? gearAudio.ChangeUpSound : gearAudio.ChangeDownSound);
					GearShiftAudio->Play();
				}
				else
				{
					// Set the latest properties on the current gear.

					LastGearPitch = FMathEx::GravitateToTarget(LastGearPitch, enginePitch, deltaSeconds * pitchRange * 2.0f);

					PistonEngineAudio[GRIP_VEHICLE_AUDIO_GEAR_C(EngineAudioIndex)]->SetPitchMultiplier(LastGearPitch);
					PistonEngineAudio[GRIP_VEHICLE_AUDIO_GEAR_C(EngineAudioIndex)]->SetFloatParameter(rpmParameter, Propulsion.CurrentGearPosition);
					PistonEngineAudio[GRIP_VEHICLE_AUDIO_GEAR_C(EngineAudioIndex)]->SetFloatParameter(kphParameter, speed);
					PistonEngineAudio[GRIP_VEHICLE_AUDIO_GEAR_C(EngineAudioIndex)]->SetFloatParameter(throttleParameter, appliedThrottle);
				}

				// Handle the fading in and out of gears.

				float inVolume = 0.0f;
				float outVolume = 0.0f;

				if (Propulsion.GearTime >= VehicleAudio->EngineSoundFadeOutTime)
				{
					GRIP_STOP_IF_PLAYING(PistonEngineAudio[GRIP_VEHICLE_AUDIO_GEAR_C(EngineAudioIndex ^ 1)]);
				}
				else
				{
					outVolume = 1.0f - (Propulsion.GearTime / VehicleAudio->EngineSoundFadeOutTime);
				}

				if (Propulsion.GearTime > VehicleAudio->EngineSoundDelayTime)
				{
					if ((Propulsion.GearTime - VehicleAudio->EngineSoundDelayTime) < VehicleAudio->EngineSoundFadeInTime)
					{
						inVolume = (Propulsion.GearTime - VehicleAudio->EngineSoundDelayTime) / VehicleAudio->EngineSoundFadeInTime;
					}
					else
					{
						inVolume = 1.0f;
					}
				}

				// Handle the management of the piston engine idle sound.

				// Fade into or out of idle, +1.0 being out, -1.0 being in.

				Propulsion.IdleTransitionTime += deltaSeconds * Propulsion.IdleTransitionDirection * 3.0f;
				Propulsion.IdleTransitionTime = FMath::Clamp(Propulsion.IdleTransitionTime, 0.0f, 1.0f);

				if (PistonEngineAudio[GRIP_VEHICLE_AUDIO_PE_IDLE]->Sound != nullptr)
				{
					if (Propulsion.IdleTransitionTime == 1.0f)
					{
						GRIP_STOP_IF_PLAYING(PistonEngineAudio[GRIP_VEHICLE_AUDIO_PE_IDLE]);
					}
					else
					{
						PistonEngineAudio[GRIP_VEHICLE_AUDIO_PE_IDLE]->SetVolumeMultiplier((1.0f - Propulsion.IdleTransitionTime) * GlobalVolume);

						GRIP_PLAY_IF_NOT_PLAYING(PistonEngineAudio[GRIP_VEHICLE_AUDIO_PE_IDLE]);
					}
				}

				PistonEngineAudio[GRIP_VEHICLE_AUDIO_GEAR_C(EngineAudioIndex)]->SetVolumeMultiplier(inVolume * Propulsion.IdleTransitionTime * GlobalVolume);
				PistonEngineAudio[GRIP_VEHICLE_AUDIO_GEAR_C(EngineAudioIndex ^ 1)]->SetVolumeMultiplier(outVolume * Propulsion.IdleTransitionTime * GlobalVolume);

				// Handle the jet engine audio.

				float pitch = FMath::Min(1.0f, GetSpeedKPH() / VehicleAudio->MaxJetEngineSpeed);

				if (JetEngineAudio[GRIP_VEHICLE_AUDIO_JE_IDLE]->Sound != nullptr)
				{
					if (Propulsion.IdleTransitionTime == 1.0f)
					{
						GRIP_STOP_IF_PLAYING(JetEngineAudio[GRIP_VEHICLE_AUDIO_JE_IDLE]);
					}
					else
					{
						JetEngineAudio[GRIP_VEHICLE_AUDIO_JE_IDLE]->SetVolumeMultiplier((1.0f - Propulsion.IdleTransitionTime) * GlobalVolume);

						GRIP_PLAY_IF_NOT_PLAYING(JetEngineAudio[GRIP_VEHICLE_AUDIO_JE_IDLE]);
					}
				}

				if (JetEngineAudio[GRIP_VEHICLE_AUDIO_JE_THRUST]->Sound != nullptr)
				{
					JetEngineAudio[GRIP_VEHICLE_AUDIO_JE_THRUST]->SetVolumeMultiplier(Propulsion.IdleTransitionTime * GlobalVolume);
					JetEngineAudio[GRIP_VEHICLE_AUDIO_JE_THRUST]->SetPitchMultiplier(FMath::Lerp(VehicleAudio->MinJetEnginePitch, VehicleAudio->MaxJetEnginePitch, pitch));
					JetEngineAudio[GRIP_VEHICLE_AUDIO_JE_THRUST]->SetFloatParameter(kphParameter, speed);
					JetEngineAudio[GRIP_VEHICLE_AUDIO_JE_THRUST]->SetFloatParameter(throttleParameter, appliedThrottle);
				}
			}
		}

#pragma endregion VehicleAudio

		bool shiftedUp = (Propulsion.LastGear < gear);
		bool shiftedDown = (Propulsion.LastGear > gear);

		// Handle the blueprint effects for gear-shifting.

		if (shiftedUp == true)
		{
			GearUpEngaged();
		}
		else if (shiftedDown == true)
		{
			GearDownEngaged();
		}

		Propulsion.LastGear = gear;

		if (shiftedUp == true)
		{
			// If we're shifting up then added a back-end physics impulse if the conditions are right.

			if (Physics.ContactData.Grounded == true &&
				Physics.ContactData.ModeTime > 0.2f &&
				Wheels.HardCompression == false)
			{
				bool valid = (Control.ThrottleInput >= 0.0f) ? Wheels.RearAxleDown : Wheels.FrontAxleDown;

				if (valid == true)
				{
					bool reversing = FVector::DotProduct(xdirection, GetVelocityOrFacingDirection()) < 0.0f;

					if (reversing == false &&
						Antigravity == false)
					{
						float direction = (Wheels.SoftFlipped == true) ? -1.0f : 1.0f;

						// Although this is clearly physics-related, we're leaving it in the general Tick function
						// as it's an impulse which doesn't need any sub-stepping.

						VehicleMesh->AddImpulseAtLocation(zdirection * -75.0f * direction * Physics.CurrentMass, Wheels.RearAxlePosition);
					}
				}
			}
		}
	}
}

#pragma endregion VehicleBasicForces

#pragma region VehicleControls

/**
* Control the forwards / backwards motion.
* The value will be somewhere between -1 and +1, often at 0 or the extremes.
***********************************************************************************/

void ABaseVehicle::Throttle(float value, bool bot)
{
	if (bot == AI.BotDriver)
	{
		bool paused = false;

		if (PlayGameMode != nullptr)
		{
			paused = PlayGameMode->GamePaused == true && AI.BotDriver == false;
		}

		if (paused == false)
		{
			float thrustForce = Control.ThrottleInput;

			Control.RawThrottleInput = FMath::Clamp(value, -1.0f, 1.0f);
			Control.ThrottleInput = Control.RawThrottleInput;

			if (Control.ThrottleInput != 0.0f)
			{
				Propulsion.HasStarted = true;
			}

			if (thrustForce == 0.0f &&
				Control.ThrottleInput > 0.0f)
			{
				ThrustEngaged();

				Control.DecideWheelSpin = true;
			}
			else if (Control.ThrottleInput == 0.0f &&
				thrustForce > 0.0f)
			{
				ThrustDisengaged();
			}

			Control.ThrottleInput = CalculateAssistedThrottleInput();
		}
	}
}

/**
* Control the left / right motion.
* The value will be somewhere between -1 and +1.
***********************************************************************************/

void ABaseVehicle::Steering(float value, bool analog, bool bot)
{
	if (bot == AI.BotDriver)
	{
		bool paused = false;

		if (PlayGameMode != nullptr)
		{
			paused = PlayGameMode->GamePaused == true && AI.BotDriver == false;
		}

		if (bot == false)
		{
			if (GameState->IsTrackMirrored() == true)
			{
				value *= -1.0f;
			}
		}

		if (paused == false)
		{
			value = FMath::Clamp(value, -1.0f, 1.0f);

			if (AI.BotDriver == false &&
				GameState->InputControllerOptions.IsValidIndex(LocalPlayerIndex) == true)
			{
				FInputControllerOptions& input = GameState->InputControllerOptions[LocalPlayerIndex];

				if (FMath::Abs(value) < input.AnalogDeadZone)
				{
					value = 0.0f;
				}

				// Make the sensitivity less responsive at lower levels in the new engine because - because players complaining.

				value = FMathEx::NegativePow(value, 1.0f + ((1.0f - input.SteeringSensitivity) * 4.0f));
			}

			if (analog == true)
			{
				Control.SteeringInputAnalog = value;

				if (bot == true ||
					value != 0.0f)
				{
					Control.SteeringAnalog = true;
				}
			}
			else
			{
				Control.SteeringInputDigital = value;

				if (value != 0.0f)
				{
					Control.SteeringAnalog = false;
				}
			}
		}
	}
}

/**
* Engage the brake.
***********************************************************************************/

void ABaseVehicle::HandbrakePressed(bool bot)
{
	if (bot == AI.BotDriver)
	{
		if (Control.BrakeInput < 0.1f)
		{
			// Determine the braking bias only when the brake is off, and maintain
			// that bias for the duration of the braking action.

			Physics.BrakingSteeringBias = FMathEx::UnitSign(Physics.SteeringBias);
		}

		if (Control.BrakeInput != 1.0f)
		{
			Control.BrakeInput = 1.0f;
			Control.HandbrakePressed = GetRealTimeClock();
		}
	}
}

/**
* Release the brake.
***********************************************************************************/

void ABaseVehicle::HandbrakeReleased(bool bot)
{
	if (bot == AI.BotDriver)
	{
		if (Control.BrakeInput != 0.0f)
		{
			Control.BrakeInput = 0.0f;

			if (RaceState.RaceTime == 0.0f)
			{
				Control.BrakePosition = Control.BrakeInput;
			}

#pragma region VehicleDrifting

			if (CanDrift() == true &&
				(GetRealTimeClock() - Control.HandbrakePressed) < 0.333f)
			{
				// If we just tapped the brake then start drifting.

				StartDrifting();
			}

#pragma endregion VehicleDrifting

		}
	}
}

/**
* Handle the use of automatic braking to assist the driver.
***********************************************************************************/

float ABaseVehicle::AutoBrakePosition(const FVector& xdirection) const
{
	float speed = GetSpeedKPH();

	if (speed > 5.0f)
	{
		FVector direction = GetVelocityOrFacingDirection();
		float dotProduct = FVector::DotProduct(direction, xdirection);

		// If we're throttling forwards but are actually currently reversing, or we're throttling
		// backwards but actually going forwards, then apply the brake to make the transition to
		// the intended direction of travel pass more quickly.

		if ((Control.ThrottleInput > 0.0f && dotProduct < -0.5f) ||
			(Control.ThrottleInput < 0.0f && dotProduct > +0.5f))
		{
			float ratio = 1.0f - FMathEx::GetRatio(speed, 75.0f, 150.0f);

			return FMath::Max(Control.BrakePosition, ratio);
		}
	}

	return Control.BrakePosition;
}

/**
* Calculate the assisted throttle input for a player.
***********************************************************************************/

float ABaseVehicle::CalculateAssistedThrottleInput()
{
	float finalThrottle = Control.RawThrottleInput;

	return finalThrottle;
}

/**
* Interpolate the control inputs to give smooth changes to digital inputs.
***********************************************************************************/

void ABaseVehicle::InterpolateControlInputs(float deltaSeconds)
{
	float steeringInput = Control.SteeringInputAnalog;
	float steeringInputSpeed = 8.0f;

	if (AI.BotDriver == false)
	{
		// Decide which direction to pitch the vehicle in when using air control.

		if (Control.AirbornePitchInput == 0.0f &&
			Control.AirborneControlActive == true)
		{
			Control.AirborneControlTimer += deltaSeconds;

			if (Control.AirborneControlTimer > 1.0f)
			{
				FMinimalViewInfo viewInfo;

				Camera->GetCameraViewNoPostProcessing(0.0f, viewInfo);

				FVector cameraUp = viewInfo.Rotation.Quaternion().GetUpVector();
				FVector vehicleUp = GetActorRotation().Quaternion().GetUpVector();

				Control.AirborneControlScale = ((FVector::DotProduct(vehicleUp, cameraUp) < 0.0f) ? -1.0f : 1.0f);
			}
		}
		else
		{
			Control.AirborneControlTimer = 0.0f;
		}

		if (Control.SteeringAnalog == true)
		{
			if (LocalPlayerIndex >= 0 &&
				LocalPlayerIndex < GameState->InputControllerOptions.Num())
			{
				FInputControllerOptions& input = GameState->InputControllerOptions[LocalPlayerIndex];

				steeringInputSpeed = 4.0f + (input.AnalogSteeringSpeed * 4.0f);
			}
		}
		else
		{
			steeringInput = Control.SteeringInputDigital;

			if (LocalPlayerIndex >= 0 &&
				LocalPlayerIndex < GameState->InputControllerOptions.Num())
			{
				FInputControllerOptions& input = GameState->InputControllerOptions[LocalPlayerIndex];

				steeringInputSpeed = 4.0f + (input.DigitalSteeringSpeed * 4.0f);
			}
		}

		Control.ThrottleInput = CalculateAssistedThrottleInput();
	}

	// Interpolate the steering and brake positions.

	Control.SteeringPosition = FMathEx::GravitateToTarget(Control.SteeringPosition, steeringInput, deltaSeconds * steeringInputSpeed);
	Control.BrakePosition = FMathEx::GravitateToTarget(Control.BrakePosition, Control.BrakeInput, deltaSeconds * BrakingInputSpeed);

	Control.AirborneRollInput = steeringInput;

	Control.AirborneRollPosition = FMathEx::GravitateToTarget(Control.AirborneRollPosition, Control.AirborneRollInput, deltaSeconds * steeringInputSpeed);
	Control.AirbornePitchPosition = FMathEx::GravitateToTarget(Control.AirbornePitchPosition, Control.AirbornePitchInput, deltaSeconds * steeringInputSpeed);

	if (Physics.ContactData.Airborne == true)
	{
		if (FMath::Abs(Control.ThrottleInput) < 0.25f)
		{
			Propulsion.ThrottleOffWhileAirborne = true;
		}
	}
	else
	{
		Propulsion.ThrottleOffWhileAirborne = false;
	}

	if (PlayGameMode != nullptr)
	{
		if (PlayGameMode->PastGameSequenceStart() == false)
		{
			Control.BrakePosition = 1.0f;
		}

		Control.ThrottleList.AddValue(GameMode->GetRealTimeClock(), Control.ThrottleInput);
	}
}

/**
* Update the steering of the wheels.
***********************************************************************************/

void ABaseVehicle::UpdateSteering(float deltaSeconds, const FVector& xdirection, const FVector& ydirection, const FQuat& quaternion)
{
	// Manage the steering control.

	float speed = GetSpeedKPH();
	float rfb = SteeringModel->FrontSteeringVsSpeed.GetRichCurve()->Eval(speed);
	float rbb = SteeringModel->BackSteeringVsSpeed.GetRichCurve()->Eval(speed);

#pragma region VehicleBidirectionalTraction

	// With VehicleAutoDirectionTraction, the steering characteristics between front and
	// rear wheels get reversed according to Physics.SteeringBias. And this is set between
	// -1 and +1, with +1 being driving forwards and -1 being driving backwards.

	float rf = FMath::Lerp(rbb, rfb, Physics.SteeringBias * 0.5f + 0.5f);
	float rb = FMath::Lerp(rfb, rbb, Physics.SteeringBias * 0.5f + 0.5f);

#pragma endregion VehicleBidirectionalTraction

	rf = FMath::Max(rf, 0.001f);
	rb = FMath::Max(rb, 0.001f);

	float steeringPosition = Control.SteeringPosition;

#pragma region NavigationSplines

#pragma region VehiclePhysicsTweaks

#if GRIP_VEHICLE_AUTO_TUNNEL_STEERING

	bool autoSteered = false;

	if (AI.BotDriver == false &&
		FMath::Abs(steeringPosition) < GRIP_STEERING_ACTIVE &&
		GRIP_POINTER_VALID(GetAI().RouteFollower.ThisSpline) == true)
	{
		int32 direction = GetPursuitSplineDirection();
		float tunnelDiameter = GetAI().RouteFollower.GetTunnelDiameterOverDistance(GetAI().RouteFollower.ThisDistance, FMath::Max(GetSpeedMPS() * 0.25f, 10.0f) * 100.0f, direction, false) / 100.0f;

		if (tunnelDiameter > 0.0f &&
			tunnelDiameter < 15.0f)
		{
			FVector splineDirection = GetAI().RouteFollower.ThisSpline->GetDirectionAtDistanceAlongSpline(GetAI().RouteFollower.ThisDistance, ESplineCoordinateSpace::World);
			float steeringScale = SteeringModel->FrontWheelsMaxSteeringAngle * rf;
			float angleOffset = 90.0f - FMathEx::DotProductToDegrees(FVector::DotProduct(ydirection, splineDirection * direction));

			if (FMath::Abs(angleOffset) > 5.0f)
			{
				angleOffset += -5.0f * FMathEx::UnitSign(angleOffset);

				steeringPosition = FMath::Clamp(angleOffset / steeringScale, -1.0f, 1.0f);
				steeringPosition = FMathEx::NegativePow(steeringPosition, 1.5f);
				steeringPosition *= 0.5f;

				if (IsFlipped() == true)
				{
					// Flip the steering if the vehicle is flipped.

					steeringPosition *= -1.0f;
				}

				if (FMath::Abs(steeringPosition) < GRIP_STEERING_ACTIVE)
				{
					steeringPosition = Control.SteeringPosition;
				}

				if (tunnelDiameter > 12.0f)
				{
					steeringScale = 1.0f - ((tunnelDiameter - 12.0f) / 3.0f);
				}
				else
				{
					steeringScale = 1.0f;
				}

				if (steeringScale > KINDA_SMALL_NUMBER)
				{
					steeringScale = FMath::Lerp(0.0f, steeringScale, FMathEx::GetRatio(speed, 25.0f, 50.0f));
				}

				float ratio = FMathEx::GetSmoothingRatio(0.5f, deltaSeconds);

				Control.AutoSteeringPosition = Control.AutoSteeringPosition * ratio + steeringPosition * (1.0f - ratio);

				steeringPosition = FMath::Lerp(Control.SteeringPosition, Control.AutoSteeringPosition, steeringScale);

				autoSteered = true;
			}
		}
	}

	if (autoSteered == false)
	{
		Control.AutoSteeringPosition = Control.SteeringPosition;
	}

#endif // GRIP_VEHICLE_AUTO_TUNNEL_STEERING

#pragma endregion VehiclePhysicsTweaks

#pragma endregion NavigationSplines

	float mfb = SteeringModel->FrontWheelsMaxSteeringAngle;
	float mbb = SteeringModel->BackWheelsMaxSteeringAngle;

#pragma region VehicleBidirectionalTraction

	float mf = FMath::Lerp(mbb, mfb, Physics.SteeringBias * 0.5f + 0.5f);
	float mb = FMath::Lerp(mfb, mbb, Physics.SteeringBias * 0.5f + 0.5f);

#pragma endregion VehicleBidirectionalTraction

	Wheels.BackSteeringAngle = steeringPosition * mb * rb;
	Wheels.FrontSteeringAngle = -steeringPosition * mf * rf;

	if (Wheels.FlipTimer > 0.0f)
	{
		Wheels.BackSteeringAngle = FMath::Lerp(Wheels.BackSteeringAngle, Wheels.BackSteeringAngle * -1.0f, Wheels.FlipTimer);
		Wheels.FrontSteeringAngle = FMath::Lerp(Wheels.FrontSteeringAngle, Wheels.FrontSteeringAngle * -1.0f, Wheels.FlipTimer);
	}

	if (Wheels.SoftFlipped == false)
	{
		Wheels.BackSteeringAngle *= -1.0f;
		Wheels.FrontSteeringAngle *= -1.0f;
	}

	float rf1 = SteeringModel->FrontSteeringVsSpeed.GetRichCurve()->Eval(0);
	float rb1 = SteeringModel->BackSteeringVsSpeed.GetRichCurve()->Eval(0);

	Wheels.FrontVisualSteeringAngle = Wheels.FrontSteeringAngle;
	Wheels.BackVisualSteeringAngle = Wheels.BackSteeringAngle;

	if (rf1 > 0.0f)
	{
		Wheels.FrontVisualSteeringAngle = FMath::Lerp(Wheels.FrontSteeringAngle, Wheels.FrontSteeringAngle * (rf1 / rf), SteeringModel->FrontVisualUnderSteerRatio);
	}

	if (rb1 > 0.0f)
	{
		Wheels.BackVisualSteeringAngle = FMath::Lerp(Wheels.BackSteeringAngle, Wheels.BackSteeringAngle * (rb1 / rb), SteeringModel->BackVisualUnderSteerRatio);
	}

	for (FVehicleWheel& wheel : Wheels.Wheels)
	{
		FRotator steering = FRotator(0.0f, ((wheel.HasRearPlacement() == true) ? Wheels.BackSteeringAngle : Wheels.FrontSteeringAngle), 0.0f);
		float steeringScale = FMathEx::GetRatio(GetSpeedKPH() * FMath::Abs(FVector::DotProduct(GetDirection(), GetVelocityDirection())), 10.0f, 100.0f);

		wheel.SetSteeringTransform(quaternion, steering, steering * steeringScale);
	}
}

/**
* Handle the pitch control for airborne control.
***********************************************************************************/

void ABaseVehicle::PitchControl(float value)
{
	if (AI.BotDriver == false &&
		GameState->InputControllerOptions.IsValidIndex(LocalPlayerIndex) == true)
	{
		FInputControllerOptions& input = GameState->InputControllerOptions[LocalPlayerIndex];

		if (FMath::Abs(value) < input.AnalogDeadZone)
		{
			value = 0.0f;
		}
	}

	Control.AirbornePitchInput = value;
}

#pragma endregion VehicleControls

#pragma region VehicleSurfaceEffects

/**
* Spawn a new surface effect for a given wheel.
***********************************************************************************/

UParticleSystemComponent* ABaseVehicle::SpawnDrivingSurfaceEffect(const FVehicleWheel& wheel, UParticleSystem* particleSystem)
{
	UParticleSystemComponent* component = NewObject<UParticleSystemComponent>(this);

	if (component != nullptr)
	{
		// We don't auto-destroy components at this point because they often get reused
		// quickly after they are apparently finished with.

		component->bAutoActivate = true;
		component->bAutoDestroy = false;

		// Attach the new component to the wheel.

		GRIP_VEHICLE_EFFECT_ATTACH(component, this, wheel.BoneName, false);

		if (GRIP_POINTER_VALID(wheel.TireMesh) == true)
		{
			// Configure the coating mesh for the tire mesh.

			static const FName CoatingSizeName("CoatingSize");

			component->SetVectorParameter(CoatingSizeName, wheel.TireMesh->GetRelativeScale3D());
			component->SetRelativeLocation(wheel.TireMesh->GetRelativeLocation());
		}

		// Assign the new effect.

		component->SetTemplate(particleSystem);
		component->SetOwnerNoSee(IsCockpitView());

		// Don't forget to register the component.

		component->RegisterComponent();

		// And now activate it.

		component->Activate();
	}

	return component;
}

/**
* Update the surface effects from the wheels.
***********************************************************************************/

void ABaseVehicle::UpdateSurfaceEffects(float deltaSeconds)
{
	static const FName GritVelocityName("GritVelocity");
	static const FName GritColorName("GritColour");
	static const FName SoftDustSizeName("SoftDustSize");
	static const FName GritAmountName("GritAmount");
	static const FName DustAlphaName("DustAlpha");
	static const FName DustColorName("DustColour");
	static const FName DustInitialLocationName("DustInitialLocation");
	static const FName CoatingAlphaName("CoatingAlpha");

	if (DrivingSurfaceCharacteristics != nullptr)
	{
		if (LocalPlayerIndex >= 0 ||
			PlayGameMode == nullptr ||
			PlayGameMode->GetVehicles().Num() <= 6)
		{
			Wheels.SurfaceEffectsTimer = DrivingSurfaceFullyVisible;
		}
		else
		{
			Wheels.SurfaceEffectsTimer += deltaSeconds / 5.0f;

			if (Wheels.SurfaceEffectsTimer >= DrivingSurfaceMaxTime)
			{
				Wheels.SurfaceEffectsTimer -= DrivingSurfaceMaxTime;
			}
		}

		float fadeInTime = 1.0f;
		float fadeOutTime = 1.5f;
		float currentSpeed = GetSpeedKPH();
		int32 maxSet = (Antigravity == true) ? 1 : 2;
		bool justLaunched = LaunchCharging == ELaunchStage::Released || LaunchCharging == ELaunchStage::Discharging;

		for (FVehicleWheel& wheel : Wheels.Wheels)
		{
			for (int32 set = 0; set < maxSet; set++)
			{
				FWheelDrivingSurfaces& components = ((set == 0) ? wheel.SurfaceComponents : wheel.FixedSurfaceComponents);

				if ((set == 1) ||
					(wheel.HasRearPlacement() == true))
				{
					FWheelDrivingSurface& activeSurface = components.Surfaces[0];
					FWheelDrivingSurface& previousSurface = components.Surfaces[1];

					// Emitters only on the rear wheels for set 0, or all wheels for set 1.

					EGameSurface surfaceType = wheel.GetActiveSensor().GetGameSurface();

					// The effect, if any, that is already in use on this wheel.

					UParticleSystem* currentEffect = (GRIP_POINTER_VALID(activeSurface.Surface) == true) ? activeSurface.Surface->Template : nullptr;

					// Is there an effect currently running on this wheel?

					bool currentIsActive = GRIP_POINTER_VALID(activeSurface.Surface);

					if (surfaceType < EGameSurface::Num)
					{
						// Record the current material for later if we have one.

						wheel.LastSurfaceContact = surfaceType;
					}
					else
					{
						if (currentIsActive == true &&
							DrivingSurfaceCharacteristics->GetContactless(wheel.LastSurfaceContact) == true)
						{
							// Reuse the last material if its contactless and we don't have one already.

							surfaceType = wheel.LastSurfaceContact;
						}

						if (justLaunched == true)
						{
							surfaceType = EGameSurface::Launched;
						}
					}

					// The effect, if any, we should be using on this surface.

					UParticleSystem* wheelEffect = nullptr;

					bool skidding = IsSkidding(true) && surfaceType != EGameSurface::Launched;
					bool spinning = SpinningTheWheel() && surfaceType != EGameSurface::Launched;
					bool mandatory = surfaceType == EGameSurface::Launched;

					if (mandatory == true ||
						Wheels.SurfaceEffectsTimer < DrivingSurfaceFadeOutAt)
					{
						wheelEffect = DrivingSurfaceCharacteristics->GetVisualEffect(surfaceType, currentSpeed, skidding, spinning, (set == 1));
					}

					float damageSmokeAlpha = 0.0f;

					float wheelFadeOutTime = (currentIsActive == true && activeSurface.Launched == true) ? 3.0f : (currentIsActive == true && activeSurface.Spinning == true) ? 0.1f : fadeOutTime;

					if ((wheelEffect != nullptr) &&
						(currentEffect != wheelEffect || currentIsActive == false) &&
						(damageSmokeAlpha == 0.0f))
					{
						// If we need to create a new effect, then do this now.
						// First we setup the existing effect for fading out to make way
						// for the new effect to fade in.

						if (currentIsActive == true &&
							wheel.HasRearPlacement() == true)
						{
							activeSurface.Surface->SetFloatParameter(GritAmountName, 0.0f);
						}

						components.SetupLastComponent(0.0f, true);

						// Create a new effect.

						activeSurface.Surface = SpawnDrivingSurfaceEffect(wheel, wheelEffect);

						if (spinning == true ||
							surfaceType == EGameSurface::Launched)
						{
							activeSurface.FadeTime = 0.1f;
						}
						else if (skidding == true)
						{
							activeSurface.FadeTime = 0.25f;
						}
						else
						{
							activeSurface.FadeTime = fadeInTime;
						}

						activeSurface.Timer = activeSurface.FadeTime;
						activeSurface.Skidding = skidding;
						activeSurface.Spinning = spinning;
						activeSurface.Launched = surfaceType == EGameSurface::Launched;
						activeSurface.Mandatory = mandatory;
					}
					else if ((wheelEffect == nullptr) &&
						(currentIsActive == true))
					{
						// If there is an old effect then deactivate that now.

						if (wheel.HasRearPlacement() == true)
						{
							activeSurface.Surface->SetFloatParameter(GritAmountName, 0.0f);
						}

						components.SetupLastComponent(wheelFadeOutTime, false);
					}

					if (GRIP_POINTER_VALID(activeSurface.Surface) == true)
					{
						// Update the current surface.

						activeSurface.Timer = FMath::Max(activeSurface.Timer - deltaSeconds, 0.0f);

						float alphaScale = 1.0f - (activeSurface.Timer / activeSurface.FadeTime);
						float speedScale = (set == 0) ? 1.0f : FMath::Clamp((GetSpeedKPH() - 50.0f) / 100.0f, 0.0f, 1.0f);
						float wheelScale = FMath::Min(FMath::Abs(wheel.RPS) / 10.0f, 1.0f);

						if (wheel.HasRearPlacement() == true)
						{
							activeSurface.Surface->SetVectorParameter(GritVelocityName, GetGritVelocity());
							activeSurface.Surface->SetVectorParameter(GritColorName, GetGritColor());
							activeSurface.Surface->SetVectorParameter(SoftDustSizeName, GetDustSize());
							activeSurface.Surface->SetFloatParameter(GritAmountName, GetGritAmount() * ((set == 0) ? 1.0f : 0.5f));
						}

						activeSurface.Surface->SetFloatParameter(DustAlphaName, GetDustAlpha(wheel, true, activeSurface.Spinning, activeSurface.Launched == false, activeSurface.Mandatory == false) * speedScale);
						activeSurface.Surface->SetVectorParameter(DustColorName, GetDustColor((set == 0)));

						if (surfaceType == EGameSurface::Dirt)
						{
							activeSurface.Surface->SetVectorParameter(DustInitialLocationName, FRotator(FMath::FRandRange(0, 360), FMath::FRandRange(0, 360), 0).RotateVector(FVector(150.0f, 0.0f, 0.0f)));
						}
						else
						{
							activeSurface.Surface->SetVectorParameter(DustInitialLocationName, FRotator(FMath::FRandRange(0, 360), FMath::FRandRange(0, 360), 0).RotateVector(FVector(150.0f, 0.0f, 0.0f)));
						}

						float alpha = activeSurface.CoatingAlpha;
						float coatingScale = FMath::Min(speedScale, wheelScale);
						float coatingAlpha = GetDustAlpha(wheel, (set == 0), activeSurface.Spinning, (set == 0) && activeSurface.Launched == false, activeSurface.Mandatory == false) * coatingScale * alphaScale;
						float difference = coatingAlpha - alpha;
						float changePerSecond = 1.0f / activeSurface.FadeTime;

						{
							if (FMath::Abs(difference) > changePerSecond * deltaSeconds)
							{
								coatingAlpha = alpha + (FMathEx::UnitSign(difference) * changePerSecond * deltaSeconds);
							}
						}

						activeSurface.CoatingAlpha = coatingAlpha;
						activeSurface.Surface->SetFloatParameter(CoatingAlphaName, coatingAlpha);
					}

					if (GRIP_POINTER_VALID(previousSurface.Surface) == true)
					{
						// Update the transitioning out surface.

						previousSurface.Timer = FMath::Max(previousSurface.Timer - deltaSeconds, 0.0f);

						float alphaScale = previousSurface.Timer / previousSurface.FadeTime;
						float speedScale = (previousSurface.Launched == false) ? FMath::Clamp((GetSpeedKPH() - 50.0f) / 100.0f, 0.0f, 1.0f) : 1.0f;

						previousSurface.Surface->SetFloatParameter(DustAlphaName, GetDustAlpha(wheel, true, previousSurface.Spinning, previousSurface.Launched == false, previousSurface.Mandatory == false) * speedScale * alphaScale);
						previousSurface.Surface->SetFloatParameter(CoatingAlphaName, previousSurface.CoatingAlpha * alphaScale);

						if (previousSurface.Timer == 0.0f ||
							damageSmokeAlpha == 1.0f)
						{
							components.DestroyLastComponent();
						}
					}
				}
			}
		}
	}
}

/**
* Get the size for a dust trail.
***********************************************************************************/

FVector ABaseVehicle::GetDustSize()
{
	float intensity = Noise(Physics.DistanceTraveled / 7.5f);

	intensity = 150.0f + (intensity * 250.0f);
	intensity = intensity + ((PerlinNoise.GetRandom() * 50.0f) - 25.0f);
	intensity *= 0.75f;

	return FVector(intensity, intensity, intensity);
}

/**
* Get the color for grit.
***********************************************************************************/

FVector ABaseVehicle::GetGritColor()
{
	return GetDustColor(true) * 0.125f;
}

/**
* Get the color for a dust trail.
***********************************************************************************/

FVector ABaseVehicle::GetDustColor(bool noise)
{
	float intensity = (noise == true) ? Noise(Physics.DistanceTraveled / 5.0f) : 1.0f;

	intensity = 0.4f + (intensity * 0.6f);

	if (noise == true)
	{
		intensity = intensity + ((PerlinNoise.GetRandom() * 0.4f) - 0.2f);
	}

	return FVector(intensity, intensity, intensity) * GameState->TransientGameState.MapSurfaceColor * GameState->TransientGameState.MapLightingColor;
}

/**
* Get the alpha for a dust trail.
***********************************************************************************/

float ABaseVehicle::GetDustAlpha(FVehicleWheel& wheel, bool noise, bool spinning, bool integrateContact, bool integrateTimer)
{
	float contactScale = (integrateContact == true) ? wheel.IsInNearContact(wheel.Radius) : 1.0f;

	if (integrateContact == true &&
		wheel.IsInContact == false)
	{
		// Fade off after one second of no contact.

		contactScale *= 1.0f - FMath::Min(wheel.ModeTime, 1.0f);
	}

	if (contactScale < KINDA_SMALL_NUMBER)
	{
		// If the wheel is too far away from the ground then no dust.

		return 0.0f;
	}
	else
	{
		float globalAlpha = ((noise == true) ? FMath::FRandRange(0.666f, 1.0f) : 1.0f) * contactScale;

		if (integrateTimer == true)
		{
			int32 phase = FMath::FloorToInt(Wheels.SurfaceEffectsTimer) % DrivingSurfaceMaxTime;

			switch (phase)
			{
			case 0:
				// Fade in.
				globalAlpha *= FMath::Fmod(Wheels.SurfaceEffectsTimer, 1.0f);
				break;
			case 1:
			case 2:
				// 1 and 2 do nothing to mitigate the alpha as it's in full effect then.
				break;
			case 3:
				// Fade out.
				globalAlpha *= 1.0f - FMath::Fmod(Wheels.SurfaceEffectsTimer, 1.0f);
				break;
			case 4:
			case 5:
				// 4 and 5 are fully faded out.
				globalAlpha = 0.0f;
				break;
			}
		}

		float intensity = 1.0f;

		if (globalAlpha > KINDA_SMALL_NUMBER)
		{
			if (noise == true)
			{
				intensity = Noise(Physics.DistanceTraveled / 2.5f) * 0.875f + 0.125f;

				intensity = intensity * intensity;
				intensity *= 0.75f;
				intensity *= FMath::Min(1.0f, (GetSpeedKPH() / 20.0f));
			}
		}

		if (spinning == true)
		{
			return 0.75f * globalAlpha;
		}
		else
		{
			return intensity * globalAlpha;
		}
	}
}

/**
* Get the amount of grit in a dust trail.
***********************************************************************************/

float ABaseVehicle::GetGritAmount() const
{
	float nominal = 0.0f;
	float additional = GetDriftRatio();

	if (SpinningTheWheel() == true)
	{
		nominal = FMath::Abs(Wheels.WheelRPS) / (VehicleEngineModel->StartingWheelSpinRPM / 60.0f);
	}

	if (FMath::Abs(Wheels.WheelRPS) < 50.0f / 60.0f)
	{
		return 0.0f;
	}

	return (nominal * 75.0f) + (75.0f * additional * nominal);
}

/**
* Get the velocity for the grit in a dust trail.
***********************************************************************************/

FVector ABaseVehicle::GetGritVelocity()
{
	float x = FMathEx::UnitSign(Wheels.WheelRPS) * ((IsFlipped() == true) ? -1.0f : 1.0f);

	return GetTransform().TransformVectorNoScale(FVector(((PerlinNoise.GetRandom() * 300.0f) + 500.0f) * x, PerlinNoise.GetRandom() ^ 100.0f, ((PerlinNoise.GetRandom() * 500.0f) + 150.0f) * ((IsFlipped() == true) ? -1.0f : 1.0f)));
}

/**
* Compute a timer to co-ordinate the concurrent use of effects across vehicles.
***********************************************************************************/

void ABaseVehicle::ComputeSurfaceEffectsTimer()
{
	if (PlayGameMode != nullptr)
	{
		int32 numVehicles = PlayGameMode->GetVehicles().Num();

		Wheels.SurfaceEffectsTimer = ((float)VehicleIndex / (float)numVehicles) * DrivingSurfaceMaxTime;
	}
}

/**
* Get a noise value.
***********************************************************************************/

float ABaseVehicle::Noise(float value) const
{
	float height = PerlinNoise.Noise1(value * 0.03125f);

	height += PerlinNoise.Noise1(value * 0.0625f) * 0.5f;
	height += PerlinNoise.Noise1(value * 0.125f) * 0.25f;
	height += PerlinNoise.Noise1(value * 0.25f) * 0.125f;

	return height + 0.625f;
}

#pragma endregion VehicleSurfaceEffects

#pragma region VehicleSurfaceImpacts

/**
* Update effects because of hard compression of the springs.
***********************************************************************************/

void ABaseVehicle::UpdateHardCompression()
{
	if (Wheels.HardCompression == true)
	{
		if (PlayGameMode != nullptr &&
			PlayGameMode->PastGameSequenceStart() == true)
		{

#pragma region VehicleAudio

			if (VehicleAudio != nullptr)
			{
				UGameplayStatics::SpawnSoundAttached(VehicleAudio->HardLandingSound, RootComponent, NAME_None, FVector::ZeroVector, EAttachLocation::KeepRelativeOffset, true, GlobalVolume);
			}

#pragma endregion VehicleAudio

			if (GetSpeedKPH() > 400.0f &&
				HardImpactEffect != nullptr &&
				(FMath::Rand() & 1) == 0)
			{
				FVector direction = GetDirection();
				FVector velocity = GetVelocityOrFacingDirection();

				if (FVector::DotProduct(direction, velocity) > 0.9f)
				{
					// If we're facing roughly the direction we're traveling, then we'll spawn an undercarriage
					// sparks effect.

					for (FVehicleWheel& wheel : Wheels.Wheels)
					{
						if (wheel.IsInContact == true &&
							wheel.HasRearPlacement() == true)
						{
							EGameSurface surfaceType = wheel.GetActiveSensor().GetGameSurface();

							if (surfaceType == EGameSurface::Asphalt ||
								surfaceType == EGameSurface::Rock ||
								surfaceType == EGameSurface::Metal)
							{
								// We only want the effect if we're on a hard surface.

								FRotator rotation = GetActorRotation();

								if (IsFlipped() == true)
								{
									rotation.Roll += 180.0f;
									rotation.Normalize();
								}

								UGameplayStatics::SpawnEmitterAtLocation(this, HardImpactEffect, GetSurfaceLocation(), rotation, true);

								break;
							}
						}
					}
				}
			}

			ShakeCamera(0.2f);

			ShakeController(0.7f, 0.15f, true, false, true, false, EDynamicForceFeedbackAction::Start);
		}
	}

	Wheels.HardCompression = false;

	if (DrivingSurfaceImpactCharacteristics != nullptr)
	{
		for (FVehicleWheel& wheel : Wheels.Wheels)
		{
			FVector location;

			if (wheel.GetActiveSensor().IsCompressionEffectRequired(location) == true)
			{
				// If the contact sensor is in contact and was just compressed hard down,
				// then spawn an impact effect for the tire.

				SpawnSurfaceImpactEffect(location, wheel.GetActiveSensor().GetDirection() * -1.0f, wheel.GetActiveSensor().GetHitResult(), FVector::ZeroVector, 0.0f, true);
			}
		}
	}
}

/**
* Spawn an impact effect.
***********************************************************************************/

void ABaseVehicle::SpawnSurfaceImpactEffect(const FVector& hitLocation, const FVector& hitNormal, const FHitResult& hitResult, const FVector& velocity, float controllerForce, bool tireImpact)
{
	UPhysicalMaterial* material = hitResult.PhysMaterial.Get();

	if (material != nullptr)
	{
		EGameSurface surfaceType = (EGameSurface)UGameplayStatics::GetSurfaceType(hitResult);
		const FDrivingSurfaceImpact* surface = DrivingSurfaceImpactCharacteristics->Surfaces.FindByKey(surfaceType);

		if (surface != nullptr)
		{
			UDrivingSurfaceImpactCharacteristics::SpawnImpact(this, *surface, tireImpact, hitLocation, hitNormal.Rotation(), velocity, GetDustColor(true), GameState->TransientGameState.MapLightingColor);
		}
	}

	if (controllerForce > 0.0f)
	{
		ShakeCamera(FMath::Clamp(controllerForce, 0.0f, 0.25f));

		ShakeController(0.7f, 0.15f, true, false, true, false, EDynamicForceFeedbackAction::Start);
	}
}

#pragma endregion VehicleSurfaceImpacts

#pragma region VehicleAnimation

/**
* Update the animated bones.
***********************************************************************************/

void ABaseVehicle::UpdateAnimatedBones(float deltaSeconds, const FVector& xdirection, const FVector& ydirection)
{
	float shiftVertical = 0.0f;

	for (int32 wheelIndex = 0; wheelIndex < GetNumWheels(); wheelIndex++)
	{
		FVehicleWheel& wheel = Wheels.Wheels[wheelIndex];

		if (Antigravity == false)
		{
			// Setup the wheel rotations for rendering with.

			WheelRotations[wheelIndex].Yaw = GetVisualSteeringAngle(wheel);

			// We're rolling the wheel so just add in the rotations for this frame.

			WheelRotations[wheelIndex].Pitch += wheel.RPS * deltaSeconds * 360.0f;
			WheelRotations[wheelIndex].Pitch = FMath::Fmod(WheelRotations[wheelIndex].Pitch, 3600.0f * FMathEx::UnitSign(WheelRotations[wheelIndex].Pitch));
		}

		// Setup the offset of the wheel to be rendered with.

		WheelOffsets[wheelIndex].Z = wheel.GetActiveSensor().GetExtension();

		float travel = MaximumWheelTravel;

		if (WheelOffsets[wheelIndex].Z > travel)
		{
			shiftVertical += WheelOffsets[wheelIndex].Z - travel;
		}
		else if (WheelOffsets[wheelIndex].Z < -travel)
		{
			shiftVertical += WheelOffsets[wheelIndex].Z + travel;
		}

		if (Wheels.FlipTimer > 0.0f)
		{
			FVehicleContactSensor& sensor = wheel.Sensors[wheel.SensorIndex ^ 1];

			WheelOffsets[wheelIndex].Z = FMath::Lerp(WheelOffsets[wheelIndex].Z, sensor.GetExtension(), Wheels.FlipTimer);
		}
	}

#pragma region VehicleLaunch

	float launchOffset = FMathEx::EaseInOut(LaunchTimer);

	if (LaunchCharging == ELaunchStage::Discharging)
	{
		launchOffset = LaunchTimer;
	}

	VehicleOffset.Z = launchOffset * ((IsFlipped() == true) ? +MaximumWheelTravel : -MaximumWheelTravel);
	VehicleOffset.Z += shiftVertical / (float)GetNumWheels();

#pragma endregion VehicleLaunch

	// Apply a visual roll to add tilt to the vehicle when cornering and most
	// of the wheels are on the ground.

	UpdateVisualRotation(deltaSeconds, xdirection, ydirection);
}

/**
* Apply a visual roll to add tilt to the vehicle when cornering and most of the
* wheels are on the ground.
***********************************************************************************/

void ABaseVehicle::UpdateVisualRotation(float deltaSeconds, const FVector& xdirection, const FVector& ydirection)
{
	float clock = VehicleClock;
	float torqueRoll = (AI.TorqueRoll * 0.15f) + (FMath::Sin(clock * AI.TorqueRoll * 100.0f) * 0.2f * AI.TorqueRoll);

	if (GetSpeedMPS() > 1.0f &&
		Wheels.NumWheelsInContact > (GetNumWheels() >> 1))
	{
		// First calculate the pitch of the vehicle based on acceleration on the vehicle's X axis.
		// This will make the back-end dip when accelerating and raise when decelerating. This would
		// normally be done through dynamic loading on the suspension in a driving simulator but would
		// result in far too much instability in GRIP. So we provide visual indicators only here.

		float ratio = FMathEx::GetSmoothingRatio(0.9f, deltaSeconds);
		float pitch = FMath::Clamp(FMathEx::CentimetersToMeters(Physics.VelocityData.AccelerationLocalSpace.X) * -0.1f * BrakingLeanScale, -BrakingLeanMaximum, BrakingLeanMaximum);

		if (IsFlipped() == false)
		{
			pitch *= -1.0f;
		}

		VehicleRotation.Pitch = FMath::Lerp(pitch, VehicleRotation.Pitch, ratio);

		// Now calculate the roll angle of the vehicle, based on how hard it's cornering.
		// Use the lateral forces on the tires to gauge where we're trying to push the vehicle towards.
		// We use this TwoFrameLateralForceStrength variable as it is an average of lateral force applied
		// over the last couple of frames, and therefore avoids the innate ping-ponging effect lateral
		// forces have of shifting a vehicle one way and then the next when not cornering sufficient hard.

		float lateralForce = 0.0f;
		float lateralForceSum = 0.0f;

		for (FVehicleWheel& wheel : Wheels.Wheels)
		{
			if (wheel.GetActiveSensor().IsInContact() == true)
			{
				lateralForce += wheel.TwoFrameLateralForceStrength;
				break;
			}
		}

		if (lateralForceSum != 0.0f)
		{
			lateralForce /= lateralForceSum;
		}

		ratio = FMathEx::GetSmoothingRatio(0.95f, deltaSeconds);

		// Note that we have to ignore anything under 50KPH as we get rogue forces in this regime.

		float scale = FMath::Pow(FMathEx::GetRatio(GetSpeedKPH(), 50.0f, 250.0f), 0.5f);

		lateralForce *= scale;

		// Now we have the lateral force computed, convert that into a body roll value.

		float roll = lateralForce * 0.04f;

		roll = (FMath::Abs(roll) < 0.25f) ? 0.0f : roll - 0.25f * FMathEx::UnitSign(roll);
		roll = FMath::Clamp(roll * CorneringLeanScale, -CorneringLeanMaximum, CorneringLeanMaximum);
		roll *= 1.0f - Control.BrakePosition;

		if (IsFlipped() == false)
		{
			roll *= -1.0f;
		}

		VehicleRotation.Roll = (VehicleRotation.Roll * ratio) + (roll * (1.0f - ratio)) + torqueRoll;
		VehiclePitchAccumulator = 0.0f;
		VehiclePitchFrom = VehicleRotation.Pitch;
	}
	else
	{
		// Gently kill pitch and roll when moving real slow.

		float ratio = FMathEx::GetSmoothingRatio(0.95f, deltaSeconds);

		VehiclePitchAccumulator += deltaSeconds * 0.5f;

		VehicleRotation.Roll = (VehicleRotation.Roll * ratio) + torqueRoll;
		VehicleRotation.Pitch = FMath::Lerp(VehiclePitchFrom, 0.0f, FMathEx::EaseInOut(FMath::Min(1.0f, VehiclePitchAccumulator), 3.0f));
	}
}

#pragma endregion VehicleAnimation

#pragma region VehicleLaunch

/**
* Update the launching of the vehicle.
***********************************************************************************/

void ABaseVehicle::UpdateLaunch(float deltaSeconds)
{
	switch (LaunchCharging)
	{
	case ELaunchStage::Charging:
		if (IsPracticallyGrounded() == true)
		{
			LaunchTimer += deltaSeconds * 1.5f;
			LaunchTimer = FMath::Min(1.0f, LaunchTimer);
		}
		break;

	case ELaunchStage::Released:
		if (IsPracticallyGrounded() == true)
		{
			if (PlayGameMode != nullptr &&
				PlayGameMode->PastGameSequenceStart() == true)
			{
				FVector direction = GetLaunchDirection();

				direction *= Physics.CurrentMass * LaunchTimer * 2000.0f;

				if (GetSpeedKPH() < 50.0f ||
					FVector::DotProduct(Physics.VelocityData.VelocityDirection, GetDirection()) < -0.5f)
				{
					VehicleMesh->AddImpulseAtLocation(direction * 0.666f, Wheels.RearAxlePosition);
				}
				else
				{
					VehicleMesh->AddImpulse(direction);
				}

#pragma region VehicleAudio

				UGameplayStatics::SpawnSoundAttached(LaunchSound, VehicleMesh, NAME_None, FVector(ForceInit), EAttachLocation::KeepRelativeOffset, false, GlobalVolume);

#pragma endregion VehicleAudio

				FRotator rotation = GetActorRotation();

				if (IsFlipped() == true)
				{
					rotation += FRotator(0.0f, 0.0f, 180.0f);
					rotation.Normalize();
				}

				FVector normal = GetSurfaceNormal();
				FVector location = GetSurfaceLocation();

				location += normal * 100.0f;

				UGameplayStatics::SpawnEmitterAtLocation(this, LaunchEffectBlueprint, location, rotation);

				LastLaunchTime = GetVehicleClock();
				LaunchSurfaceNormal = GuessSurfaceNormal();
			}
		}

		LaunchCharging = ELaunchStage::Discharging;

		break;

	case ELaunchStage::Discharging:
		LaunchTimer -= deltaSeconds * 5.0f;
		LaunchTimer = FMath::Max(0.0f, LaunchTimer);

		if (LaunchTimer == 0.0f)
		{
			LaunchCharging = ELaunchStage::Idle;
		}
		break;
	}
}

#pragma endregion VehicleLaunch

#pragma region VehicleDrifting

/**
* Update the drifting of the back end state.
***********************************************************************************/

void ABaseVehicle::UpdateDriftingState(float deltaSeconds)
{
	// We cancel any drifting if we get airborne, we stop steering very much,
	// we reduce throttle below 50% or we go below 150kph.

	if (IsDrifting() == true)
	{
		if ((Physics.ContactData.Airborne == true && Physics.ContactData.ModeTime > 0.5f) ||
			(FMath::Abs(Control.SteeringPosition) < GRIP_STEERING_PURPOSEFUL) ||
			(AI.BotDriver == false && Control.ThrottleInput < 0.5f) ||
			(AI.BotDriver == true && Control.ThrottleInput < 0.1f) ||
			(GetSpeedKPH() < 150.0f))
		{
			Physics.Drifting.Active = false;

			if (Physics.Drifting.Timer < 0.25f)
			{
				Physics.Drifting.Timer += Physics.Drifting.NonDriftingTimer;
			}
			else
			{
				Physics.Drifting.Timer = 0.0f;
			}
		}
	}

	if (Antigravity == false)
	{
		if (GetDriftRatio() > 0.2f)
		{
			ShakeController(GetDriftRatio() * 0.3f + 0.1f, 0.10f, true, true, false, false, EDynamicForceFeedbackAction::Start);
		}
	}

	// Manage the timer for the skidding state, used to smooth out changes in that state.

	if (IsSkidding() == true)
	{
		Wheels.SkidTimer = 0.25f;
	}
	else if (IsPracticallyGrounded(75.0f) == false)
	{
		Wheels.SkidTimer = 0.0f;
	}
	else
	{
		Wheels.SkidTimer = FMath::Max(Wheels.SkidTimer - deltaSeconds, 0.0f);
	}
}

#pragma endregion VehicleDrifting

#pragma region VehicleAudio

/**
* Configure the vehicle's engine audio.
***********************************************************************************/

void ABaseVehicle::SetupEngineAudio()
{
	GearShiftAudio = NewObject<UAudioComponent>(this, TEXT("GearShiftAudio")); GearShiftAudio->RegisterComponent();
	GRIP_ATTACH(GearShiftAudio, RootComponent, "RootDummy");

	EngineBoostAudio = NewObject<UAudioComponent>(this, TEXT("EngineBoostAudio")); EngineBoostAudio->RegisterComponent();
	GRIP_ATTACH(EngineBoostAudio, RootComponent, "RootDummy");

	SkiddingAudio = NewObject<UAudioComponent>(this, TEXT("SkiddingAudio")); SkiddingAudio->RegisterComponent();
	GRIP_ATTACH(SkiddingAudio, RootComponent, "RootDummy");

	for (int32 i = 0; i < 3; i++)
	{
		PistonEngineAudio.Emplace(NewObject<UAudioComponent>(this, FName(*FString::Printf(TEXT("PistonEngineAudio%d"), i)))); PistonEngineAudio[i]->RegisterComponent();
		GRIP_ATTACH(PistonEngineAudio[i], RootComponent, "RootDummy");
	}

	for (int32 i = 0; i < 2; i++)
	{
		JetEngineAudio.Emplace(NewObject<UAudioComponent>(this, FName(*FString::Printf(TEXT("JetEngineAudio%d"), i)))); JetEngineAudio[i]->RegisterComponent();
		GRIP_ATTACH(JetEngineAudio[i], RootComponent, "RootDummy");
	}

	if (VehicleAudio != nullptr)
	{
		SET_VEHICLE_SOUND_NON_SPATIALIZED(VehicleAudio->EngineBoostSound);
		SET_VEHICLE_SOUND_NON_SPATIALIZED(VehicleAudio->EngineIdleSound);
		SET_VEHICLE_SOUND_NON_SPATIALIZED(VehicleAudio->JetEngineIdleSound);
		SET_VEHICLE_SOUND_NON_SPATIALIZED(VehicleAudio->JetEngineSound);

		for (FVehicleAudioGear& gear : VehicleAudio->Gears)
		{
			SET_VEHICLE_SOUND_NON_SPATIALIZED(gear.EngineSound);
			SET_VEHICLE_SOUND_NON_SPATIALIZED(gear.ChangeUpSound);
			SET_VEHICLE_SOUND_NON_SPATIALIZED(gear.ChangeDownSound);
		}

		PistonEngineAudio[GRIP_VEHICLE_AUDIO_PE_IDLE]->SetSound(VehicleAudio->EngineIdleSound);
		PistonEngineAudio[GRIP_VEHICLE_AUDIO_PE_IDLE]->SetVolumeMultiplier(GlobalVolume);
		PistonEngineAudio[GRIP_VEHICLE_AUDIO_PE_IDLE]->Play();

		if (VehicleAudio->Gears.Num() > 0)
		{
			FVehicleAudioGear& gear = VehicleAudio->Gears[0];

			PistonEngineAudio[GRIP_VEHICLE_AUDIO_GEAR_C(EngineAudioIndex)]->SetSound(gear.EngineSound);
			PistonEngineAudio[GRIP_VEHICLE_AUDIO_GEAR_C(EngineAudioIndex)]->SetVolumeMultiplier(0.0f);
			PistonEngineAudio[GRIP_VEHICLE_AUDIO_GEAR_C(EngineAudioIndex)]->SetPitchMultiplier(gear.MinEnginePitch);
			PistonEngineAudio[GRIP_VEHICLE_AUDIO_GEAR_C(EngineAudioIndex)]->Play();
		}

		JetEngineAudio[GRIP_VEHICLE_AUDIO_JE_IDLE]->SetSound(VehicleAudio->JetEngineIdleSound);
		JetEngineAudio[GRIP_VEHICLE_AUDIO_JE_IDLE]->SetVolumeMultiplier(GlobalVolume);
		JetEngineAudio[GRIP_VEHICLE_AUDIO_JE_IDLE]->Play();

		JetEngineAudio[GRIP_VEHICLE_AUDIO_JE_THRUST]->SetSound(VehicleAudio->JetEngineSound);
		JetEngineAudio[GRIP_VEHICLE_AUDIO_JE_THRUST]->SetVolumeMultiplier(0.0f);
		JetEngineAudio[GRIP_VEHICLE_AUDIO_JE_THRUST]->Play();
	}
}

/**
* Manage the audio for skidding.
***********************************************************************************/

void ABaseVehicle::UpdateSkidAudio(float deltaSeconds)
{
	if (SkiddingAudio != nullptr &&
		IsVehicleDestroyed() == false)
	{
		{
			SkidAudioVolume = FMathEx::GravitateToTarget(SkidAudioVolume, FMath::Max(Wheels.SkidAudioVolumeTarget, Wheels.SpinAudioVolumeTarget), deltaSeconds * 3.0f);
		}

		static FName skidStrength("Strength");

		SkiddingAudio->SetVolumeMultiplier(SkidAudioVolume * GlobalVolume);
		SkiddingAudio->SetFloatParameter(skidStrength, SkidAudioVolume);

		if (SkidAudioVolume > 0.0f &&
			SkidAudioPlaying == false &&
			GRIP_POINTER_VALID(SkiddingSound) == true)
		{
			SkidAudioPlaying = true;

			SET_VEHICLE_SOUND_NON_SPATIALIZED(SkiddingSound);
			SkiddingAudio->SetSound(SkiddingSound.Get());
			SkiddingAudio->Play();
			LastSkiddingSound = SkiddingSound;
		}
		else if (SkidAudioVolume <= 0.0f &&
			SkidAudioPlaying == true)
		{
			SkidAudioPlaying = false;

			SkiddingAudio->Stop();
		}

		if (SkidAudioVolume > 0.0f &&
			SkidAudioPlaying == true &&
			GRIP_POINTER_VALID(SkiddingSound) == true &&
			SkiddingSound.Get() != LastSkiddingSound.Get())
		{
			SET_VEHICLE_SOUND_NON_SPATIALIZED(SkiddingSound);
			SkiddingAudio->SetSound(SkiddingSound.Get());
			LastSkiddingSound = SkiddingSound;
		}
	}
}

#pragma endregion VehicleAudio

#pragma region VehicleSpringArm

/**
* Looking forwards or backwards.
***********************************************************************************/

void ABaseVehicle::LookForwards(float val)
{
	float deadZone = 0.0f;

	if (AI.BotDriver == false &&
		LocalPlayerIndex >= 0 &&
		LocalPlayerIndex < GameState->InputControllerOptions.Num())
	{
		FInputControllerOptions& input = GameState->InputControllerOptions[LocalPlayerIndex];

		deadZone = input.AnalogDeadZone;

		if (input.IgnoreRightStick == true)
		{
			return;
		}
	}

	CameraTarget()->SpringArm->LookForwards(val, deadZone);
}

/**
* Looking left or right.
***********************************************************************************/

void ABaseVehicle::LookSideways(float val)
{
	if (GameState->IsTrackMirrored() == true)
	{
		val *= -1.0f;
	}

	float deadZone = 0.0f;

	if (AI.BotDriver == false &&
		LocalPlayerIndex >= 0 &&
		LocalPlayerIndex < GameState->InputControllerOptions.Num())
	{
		FInputControllerOptions& input = GameState->InputControllerOptions[LocalPlayerIndex];

		deadZone = input.AnalogDeadZone;

		if (input.IgnoreRightStick == true)
		{
			return;
		}
	}

	CameraTarget()->SpringArm->LookSideways(val, deadZone);
}

/**
* Looking left.
***********************************************************************************/

void ABaseVehicle::LeftViewCamera()
{
	if (GameState->IsTrackMirrored() == true)
	{
		CameraTarget()->SpringArm->RightViewCamera(GameState->GeneralOptions.InstantaneousLook);
	}
	else
	{
		CameraTarget()->SpringArm->LeftViewCamera(GameState->GeneralOptions.InstantaneousLook);
	}
}

/**
* Looking right.
***********************************************************************************/

void ABaseVehicle::RightViewCamera()
{
	if (GameState->IsTrackMirrored() == true)
	{
		CameraTarget()->SpringArm->LeftViewCamera(GameState->GeneralOptions.InstantaneousLook);
	}
	else
	{
		CameraTarget()->SpringArm->RightViewCamera(GameState->GeneralOptions.InstantaneousLook);
	}
}

/**
* The angle that the rear-end is currently drifting at.
***********************************************************************************/

float ABaseVehicle::GetSpringArmYaw() const
{
	float yaw = GetDriftRatio();

	yaw = FMathEx::NegativePow(yaw, 0.4f);
	yaw = yaw * Physics.Drifting.RearDriftAngle * SpringArm->DriftYawExtension;

	return yaw;
}

/**
* The roll angle.
***********************************************************************************/

float ABaseVehicle::GetSpringArmRoll() const
{
	// This is pretty much just a bit of extra vehicle lean, it's not the entire rotation of the vehicle.

	return (VehicleRotation.Roll * 0.5f) + (GetDriftRatio() * 6.0f * FMathEx::UnitSign(VehicleRotation.Roll));
}

/**
* Has the vehicle just smashed into something and requires the forward-facing
* crash-camera?
***********************************************************************************/

bool ABaseVehicle::HasSmashedIntoSomething(float maxKPH) const
{
	if (PlayGameMode != nullptr)
	{
		float lastSpeed = AI.Speed.GetLastValue();

		if (lastSpeed < FMathEx::KilometersPerHourToCentimetersPerSecond(maxKPH))
		{
			// We're going slow enough, now see if there was a sharp drop-off in speed to get us here.

			float lastTime = AI.Speed.GetLastTime();
			float hundredKPH = FMathEx::KilometersPerHourToCentimetersPerSecond(100.0f);

			for (int32 i = AI.Speed.GetNumValues() - 1; i >= 0; i--)
			{
				if ((lastTime - AI.Speed[i].Time) < 0.5f)
				{
					if ((AI.Speed[i].Value - lastSpeed) > hundredKPH)
					{
						return true;
					}
				}
				else
				{
					break;
				}
			}
		}
	}

	return false;
}

/**
* Update the materials used to render the vehicle based on cockpit-camera state.
***********************************************************************************/

void ABaseVehicle::UpdateCockpitMaterials()
{
	bool isCockpitView = IsCockpitView();

	if (UsingCockpitMaterial != isCockpitView)
	{
		UsingCockpitMaterial = isCockpitView;

		if (isCockpitView == false &&
			BaseMaterials.Num() > 0)
		{
			int32 materialIndex = 0;
			UObject* lastObject = nullptr;

			for (FMeshMaterialOverride& component : BaseMaterials)
			{
				if (lastObject != component.Component)
				{
					materialIndex = 0;
					lastObject = component.Component;
				}

				component.Component->SetMaterial(materialIndex++, component.Material);
			}
		}
		else if (isCockpitView == true)
		{
			if (OurGhostMaterial == nullptr)
			{
				OurGhostMaterial = UMaterialInstanceDynamic::Create(CockpitGhostMaterial, this);
			}

			OurGhostMaterial->SetScalarParameterValue("CentreViewSize", 8.0f);

			if (BaseMaterials.Num() == 0)
			{
				for (int32 j = 0; j < VehicleMesh->GetNumMaterials(); j++)
				{
					BaseMaterials.Emplace(FMeshMaterialOverride(VehicleMesh, VehicleMesh->GetMaterial(j)));
				}

				for (int32 i = 0; i < VehicleMesh->GetNumChildrenComponents(); i++)
				{
					USceneComponent* child = VehicleMesh->GetChildComponent(i);
					UStaticMeshComponent* staticMesh = Cast<UStaticMeshComponent>(child);
					UChildActorComponent* childActor = Cast<UChildActorComponent>(child);

					if (staticMesh != nullptr)
					{
						for (int32 j = 0; j < staticMesh->GetNumMaterials(); j++)
						{
							BaseMaterials.Emplace(FMeshMaterialOverride(staticMesh, staticMesh->GetMaterial(j)));
						}
					}
					else if (childActor != nullptr)
					{
						ACanard* canard = Cast<ACanard>(childActor->GetChildActor());

						if (canard != nullptr)
						{
							for (int32 j = 0; j < canard->CanardMesh->GetNumMaterials(); j++)
							{
								BaseMaterials.Emplace(FMeshMaterialOverride(canard->CanardMesh, canard->CanardMesh->GetMaterial(j)));
							}
						}
					}
				}
			}

			int32 materialIndex = 0;
			UObject* lastObject = nullptr;

			for (FMeshMaterialOverride& component : BaseMaterials)
			{
				if (lastObject != component.Component)
				{
					materialIndex = 0;
					lastObject = component.Component;
				}

				component.Component->SetMaterial(materialIndex++, OurGhostMaterial);
			}
		}
	}

	if (isCockpitView == true)
	{
		OurGhostMaterial->SetScalarParameterValue("CockpitOpacity", GameState->GraphicsOptions.CockpitVehicleVisibility);
	}
}

#pragma endregion VehicleSpringArm

#pragma region VehicleHUD

/**
* Shake the HUD, following an explosion or something.
***********************************************************************************/

void ABaseVehicle::ShakeHUD(float strength)
{
	if (PlayGameMode != nullptr &&
		PlayGameMode->PastGameSequenceStart() == true)
	{
		float shakeStrength = FMath::Sqrt(FMath::Min(strength, 1.0f));

		ShakeController(shakeStrength, FMath::Max(0.1f, shakeStrength * 0.5f), true, false, true, false, EDynamicForceFeedbackAction::Start);

		if (strength > 0.2f)
		{
			float thisMagnitude = HUD.ShakeMagnitude * (HUD.ShakeTimer / HUD.ShakeTime);

			if (thisMagnitude < strength)
			{
				HUD.ShakeTime = 4.0f;
				HUD.ShakeTimer = HUD.ShakeTime;
				HUD.ShakeMagnitude = FMath::Max(thisMagnitude, strength);
			}
		}
	}
}

/**
* Play a 1D client sound.
***********************************************************************************/

void ABaseVehicle::ClientPlaySound(USoundBase* Sound, float VolumeMultiplier, float PitchMultiplier) const
{
	if (IsHumanPlayer() == true &&
		HasAIDriver() == false)
	{
		UGameplayStatics::PlaySound2D(this, Sound, VolumeMultiplier, PitchMultiplier);
	}
}

/**
* Play the denied sound when a player tries to do something that they cannot.
***********************************************************************************/

void ABaseVehicle::PlayDeniedSound()
{
	if (IsHumanPlayer() == true &&
		IsCinematicCameraActive() == false)
	{
		ClientPlaySound(HUD.PickupNotChargeableSound);
	}
}

/**
* Get the speed of the vehicle, in kilometers / miles per hour.
***********************************************************************************/

FString ABaseVehicle::GetFormattedSpeedKPH(int32 index) const
{
	if (GameState->TransientGameState.ShowFPS == true &&
		GameState->GeneralOptions.SpeedUnit != ESpeedDisplayUnit::MACH)
	{
		return FString::Printf(TEXT("%03d"), FMath::RoundToInt(1.0f / PlayGameMode->FrameTimes.GetScaledMeanValue()));
	}
	else
	{
		float speed = GetSpeedKPH(true);

		switch (GameState->GeneralOptions.SpeedUnit)
		{
		case ESpeedDisplayUnit::MPH:
			return FString::Printf(TEXT("%03d"), FMath::FloorToInt(speed * 0.621371f));
		case ESpeedDisplayUnit::KPH:
			return FString::Printf(TEXT("%03d"), FMath::FloorToInt(speed));
		default:
			if (index == 0)
			{
				return FString::Printf(TEXT("%01d"), FMath::FloorToInt(speed * 0.000809848f));
			}
			else
			{
				return FString::Printf(TEXT("%02d"), FMath::FloorToInt(FMath::Frac(speed * 0.000809848f) * 100.0f));
			}
		}
	}
}

/**
* Get a formatted time for racing.
***********************************************************************************/

FString ABaseVehicle::GetFormattedTime(float seconds)
{
	float minutes = FMath::FloorToFloat(seconds / 60.0f);

	seconds -= minutes * 60.0f;

	float thousands = FMath::Frac(seconds) * 1000.0f;

	return FString::Printf(TEXT("%02d:%02d.%03d"), FMath::FloorToInt(minutes), FMath::FloorToInt(seconds), FMath::FloorToInt(thousands));
}

/**
* Is the vehicle going the wrong way around the track?
***********************************************************************************/

bool ABaseVehicle::IsGoingTheWrongWay() const
{
	if (GameState->IsGameModeRace() == true &&
		PlayGameMode->PastGameSequenceStart() == true &&
		PlayGameMode->GameHasEnded() == false)
	{
		if (GetSpeedKPH() > 100.0f &&
			HUD.WrongWayTimer > 2.0f)
		{
			return true;
		}
	}

	return false;
}

/**
* Show a status message.
***********************************************************************************/

void ABaseVehicle::ShowStatusMessage(const FStatusMessage& message, bool queue, bool inChatIfPossible) const
{
	if (HUDWidget != nullptr)
	{
		if (inChatIfPossible == true &&
			HUDWidget->GetEventPlayingVisibility() != ESlateVisibility::Collapsed)
		{
			if (PlayGameMode != nullptr)
			{
				FGameEvent event;

				event.LaunchVehicleIndex = -1;
				event.EventType = EGameEventType::ChatMessage;
				event.ExtraInformation = message.Message.ToString();

				PlayGameMode->AddGameEvent(event);
			}
		}
		else
		{
			HUDWidget->ShowStatusMessage(message, queue);
		}
	}
}

/**
* Get the alpha value of the wrong way indicator.
***********************************************************************************/

float ABaseVehicle::GetWrongWayAlpha()
{
	if (IsGoingTheWrongWay() == true)
	{
		return (FMath::Fmod(HUD.WrongWayTimer, 1.0f) * 0.5f < 0.25f) ? 1.0f : 0.0f;
	}

	return 0.0f;
}

#pragma endregion VehicleHUD

#pragma region ClocksAndTime

/**
* Reset the timer used for controlling attack frequency.
***********************************************************************************/

void ABaseVehicle::ResetAttackTimer()
{
	float attackDelay = PlayGameMode->GetDifficultyCharacteristics().PickupUseCharacteristics.Race.MaxHumanAttackFrequency;

	attackDelay = FMath::Max(attackDelay, FMath::Lerp(attackDelay, 50.0f, FMath::Min(1.0f, PlayGameMode->LastLapRatio * 1.5f)));

	AttackAfter = VehicleClock + FMath::FRandRange(attackDelay, attackDelay * 1.25f);
}

#pragma endregion ClocksAndTime

#pragma region Miscellaneous

/**
* Set whether the vehicle should use an AI driver or not.
***********************************************************************************/

void ABaseVehicle::SetAIDriver(bool aiDriver, bool setVehicle, bool setInputMappings)
{
	if (AI.BotDriver != aiDriver)
	{
		AI.BotDriver = aiDriver;

		if (AI.BotDriver == true)
		{
		}
		else
		{
			HandbrakeReleased(false);
		}
	}

	if (setVehicle == true)
	{
		AI.BotVehicle = AI.BotDriver;
	}

	if (setInputMappings == true)
	{
		GameMode->SetInputOptions(Cast<APlayerController>(GetController()));
	}
}

/**
* Add points to the player's total if the player's game hasn't ended.
***********************************************************************************/

bool ABaseVehicle::AddPoints(int32 numPoints, bool visualize, ABaseVehicle* fromVehicle, const FVector& worldLocation)
{
	if ((numPoints > 0) &&
		(IsVehicleDestroyed() == false))
	{
		if (RaceState.AddPoints(numPoints) == true)
		{
			return true;
		}
	}

	return false;
}

/**
* Get the progress through the game event, from 0 to 1.
***********************************************************************************/

float ABaseVehicle::GetEventProgress()
{
	if (GameState->IsGameModeLapBased() == true)
	{
		RaceState.EventProgress = FMath::Min(RaceState.RaceDistance / (PlayGameMode->MasterRacingSplineLength * GameState->GeneralOptions.NumberOfLaps), 1.0f);
	}
	else
	{
		RaceState.EventProgress = 0.0f;
	}

	return RaceState.EventProgress;
}

/**
* Cycle through the camera points on the vehicle.
***********************************************************************************/

void ABaseVehicle::CycleCameraPoint()
{
}

/**
* Should the vehicle turn left to head in the correct direction?
***********************************************************************************/

bool ABaseVehicle::ShouldTurnLeft() const
{
	return false;
}

/**
* Should the vehicle turn right to head in the correct direction?
***********************************************************************************/

bool ABaseVehicle::ShouldTurnRight() const
{
	return false;
}

/**
* Does this vehicle belong to a human player?
***********************************************************************************/

int32 ABaseVehicle::DetermineLocalPlayerIndex()
{
	ControllerID = INDEX_NONE;
	LocalPlayerIndex = INDEX_NONE;

	if (IsHumanPlayer() == true)
	{
		APlayerController* controller = Cast<APlayerController>(GetController());

		if (controller != nullptr)
		{
			int32 index = 0;

			// #TODO: Check this indexing method, in SP and SS.

			for (TActorIterator<AController> actorItr(GetWorld()); actorItr; ++actorItr)
			{
				if (*actorItr == controller)
				{
					LocalPlayerIndex = index;
					break;
				}

				index++;
			}

			ControllerID = controller->GetLocalPlayer()->GetControllerId();
		}
	}

	return LocalPlayerIndex;
}

/**
* Disqualify this player from the game event.
***********************************************************************************/

void ABaseVehicle::Disqualify()
{
	if (PlayGameMode != nullptr &&
		RaceState.PlayerCompletionState < EPlayerCompletionState::Complete)
	{
		RaceState.GameFinishedAt = PlayGameMode->GetRealTimeClock();
	}

	RaceState.PlayerCompletionState = EPlayerCompletionState::Disqualified;
	RaceState.RaceRank = -1;
	RaceState.RacePosition = -1;
}

/**
* Perform some initialization on the vehicle post spawn.
***********************************************************************************/

void ABaseVehicle::PostSpawn(int32 vehicleIndex, bool isLocalPlayer, bool bot)
{
	// NOTE: You cannot rely on PreInitializeComponents, PostInitializeComponents or
	// anything else having been called before this function executes. It will have
	// for automatically created pawns like the local players, but for bots for
	// example, this will be the first function called in that execution chain.

	UE_LOG(GripLog, Log, TEXT("ABaseVehicle::PostSpawn"));

	PostSpawnStarted = true;

	World = GetWorld();
	GameMode = ABaseGameMode::Get(this);
	PlayGameMode = APlayGameMode::Get(this);
	GameState = UGlobalGameState::GetGlobalGameState(this);

	VehicleIndex = vehicleIndex;

	AI.BotDriver = AI.BotVehicle = bot;
	AI.DifficultyLevel = GameState->GeneralOptions.DifficultyLevel;

	if (isLocalPlayer == true)
	{
		DetermineLocalPlayerIndex();
	}

	if (PlayGameMode != nullptr)
	{
		PlayGameMode->DetermineVehicles();
	}

	if (HasActorBegunPlay() == true)
	{
		CompletePostSpawn();
	}
}

/**
* Complete the post spawn sequence.
***********************************************************************************/

void ABaseVehicle::CompletePostSpawn()
{
	if (PostSpawnStarted == true &&
		PostSpawnComplete == false)
	{
		UE_LOG(GripLog, Log, TEXT("ABaseVehicle::CompletePostSpawn"));

		PostSpawnComplete = true;

#pragma region VehicleSurfaceEffects

		// Compute a timer to co-ordinate the concurrent use of effects across vehicles.

		ComputeSurfaceEffectsTimer();

#pragma endregion VehicleSurfaceEffects

		if (PlayGameMode != nullptr)
		{
			PlayGameMode->AddAvoidable(this);

			TWeakObjectPtr<UPursuitSplineComponent> mainSpline = PlayGameMode->MasterRacingSpline;

			if (mainSpline != nullptr)
			{

#pragma region NavigationSplines

				RaceState.DistanceAlongMasterRacingSpline = mainSpline->GetNearestDistance(GetActorLocation(), 0.0f, 0.0f, 10, PlayGameMode->MasterRacingSplineLength / (50.0f * 100.0f));
				RaceState.LastDistanceAlongMasterRacingSpline = RaceState.GroundedDistanceAlongMasterRacingSpline = RaceState.DistanceAlongMasterRacingSpline;

#pragma endregion NavigationSplines

				if (PlayGameMode->MasterRacingSplineStartDistance != 0.0f &&
					PlayGameMode->UnknownPlayerStart == false)
				{
					ensureAlwaysMsgf(RaceState.DistanceAlongMasterRacingSpline < PlayGameMode->MasterRacingSplineStartDistance, TEXT("Player in front of starting line (%f %f)"), RaceState.DistanceAlongMasterRacingSpline, PlayGameMode->MasterRacingSplineStartDistance);
				}
			}
		}

#pragma region VehicleAudio

		SetupEngineAudio();

#pragma endregion VehicleAudio

	}
}

/**
* Get the target heading for the vehicle, roughly what direction it should be
* heading in for this part of the track.
***********************************************************************************/

FVector ABaseVehicle::GetTargetHeading() const
{

#pragma region NavigationSplines

	if (GRIP_POINTER_VALID(AI.RouteFollower.ThisSpline) == true)
	{
		FVector v0 = AI.RouteFollower.ThisSpline->GetDirectionAtDistanceAlongSpline(AI.RouteFollower.ThisDistance, ESplineCoordinateSpace::World);
		FVector v1 = AI.RouteFollower.NextSpline->GetDirectionAtDistanceAlongSpline(AI.RouteFollower.NextDistance, ESplineCoordinateSpace::World);
		FVector v2 = FMath::Lerp(v0, v1, 0.5f); v2.Normalize();

		return v2;
	}
	else

#pragma endregion NavigationSplines

	{
		return GetFacingDirection();
	}
}

/**
* Get the target vehicle for the camera.
***********************************************************************************/

ABaseVehicle* ABaseVehicle::CameraTarget()
{
	ABaseVehicle* result = this;

	return result;
}

/**
* Get the name of the player, optionally shortened or full.
***********************************************************************************/

const FString& ABaseVehicle::GetPlayerName(bool shortened, bool full)
{
	if (PlayerNamesValid == false)
	{
		// If we've gotten to here, we're either in offline mode or for some reason getting the
		// online name failed and we've not got it cached yet.

		if (IsHumanPlayer() == true &&
			LocalPlayerIndex == 0)
		{
			PlayerName = ABaseGameMode::GetPlayerName(GetPlayerState(), 1, true);
		}
		else
		{
			int32 playerNumber = (LocalPlayerIndex != INDEX_NONE) ? LocalPlayerIndex + 1 : VehicleIndex + 1;

			PlayerName = ABaseGameMode::GetPlayerName(GetPlayerState(), playerNumber, true, true);
		}

		ShortPlayerName = ABaseGameMode::ShortenString(PlayerName, 20);

		PlayerNamesValid = true;
	}

	return (shortened == true) ? ShortPlayerName : PlayerName;
}

/**
* Spawn an appropriately scaled particle system on the vehicle.
***********************************************************************************/

UParticleSystemComponent* ABaseVehicle::SpawnParticleSystem(UParticleSystem* emitterTemplate, FName attachPointName, FVector location, FRotator rotation, EAttachLocation::Type locationType, float scale, bool autoDestroy)
{
	UParticleSystemComponent* component = nullptr;

	if (emitterTemplate != nullptr)
	{
		component = NewObject<UParticleSystemComponent>(RootComponent->GetOwner());

		component->bAutoDestroy = autoDestroy;
		component->bAllowAnyoneToDestroyMe = true;
		component->SecondsBeforeInactive = 0.0f;
		component->bAutoActivate = false;
		component->SetTemplate(emitterTemplate);
		component->bOverrideLODMethod = false;

		GRIP_ATTACH(component, RootComponent, attachPointName);

		if (locationType == EAttachLocation::KeepWorldPosition)
		{
			component->SetWorldLocationAndRotation(location, rotation);
		}
		else
		{
			component->SetRelativeLocationAndRotation(location, rotation);
		}

		if (scale < KINDA_SMALL_NUMBER)
		{
			scale = 1.0f;
		}

		component->SetRelativeScale3D(AttachedEffectsScale * scale);
		component->RegisterComponent();
		component->ActivateSystem(true);
	}

	return component;
}

/**
* Shakes the user GamePad, according to strength and duration.
***********************************************************************************/

void ABaseVehicle::ShakeController(float strength, float duration, bool smallLeft, bool smallRight, bool largeLeft, bool largeRight, TEnumAsByte<EDynamicForceFeedbackAction::Type> action)
{
	if (AI.BotDriver == false &&
		IsVehicleDestroyed() == false)
	{
		if (PlayGameMode != nullptr &&
			PlayGameMode->PastGameSequenceStart() == true)
		{
			if (LocalPlayerIndex >= 0 &&
				LocalPlayerIndex < GameState->InputControllerOptions.Num())
			{
				if (GameState->InputControllerOptions[LocalPlayerIndex].UseForceFeedback == true)
				{
					APlayerController* controller = Cast<APlayerController>(Controller);

					if (controller != nullptr)
					{
						strength *= GameState->InputControllerOptions[LocalPlayerIndex].ForceFeedbackStrength;

						Control.ForceFeedbackHandle = controller->PlayDynamicForceFeedback(strength, FMath::Clamp(duration, 0.1f, 0.5f), largeLeft, smallLeft, largeRight, smallRight, action, Control.ForceFeedbackHandle);
					}
				}
			}
		}
	}
}

/**
* Shake the camera.
***********************************************************************************/

bool ABaseVehicle::ShakeCamera(float strength)
{
	bool result = false;

	if (ImpactCameraShake != nullptr)
	{
		if (PlayGameMode != nullptr &&
			PlayGameMode->PastGameSequenceStart() == true)
		{
			if (IsHumanPlayer() == true &&
				IsCinematicCameraActive() == false)
			{
				APlayerController* controller = Cast<APlayerController>(Controller);

				if (controller != nullptr &&
					controller->IsLocalController())
				{
					controller->ClientPlayCameraShake(ImpactCameraShake, strength);

					result = true;
				}
			}
		}
	}

	return result;
}

/**
* Begin teleportation.
***********************************************************************************/

void ABaseVehicle::BeginTeleport()
{
}

/**
* Handle the update of the idle locking, ensuring the vehicle stays still at very
* low speed rather than subtly sliding around.
***********************************************************************************/

void ABaseVehicle::UpdateIdleLock()
{
	VehicleMesh->UpdateIdleLock(false);

	if (VehicleMesh->IsIdle() == false)
	{

#pragma region VehicleGrip

		// Determine if the vehicle is idle and lock it in place if it is.

		if (Antigravity == false &&
			IsGrounded() == true &&
			GetSpeedKPH() <= 1.0f &&
			FMath::Abs(Control.ThrottleInput) <= 0.1f &&
			FMath::Abs(FVector::DotProduct(GetLaunchDirection(), FVector(0.0f, 0.0f, 1.0f))) > 0.5f)
		{
			bool idle = true;

			for (FVehicleWheel& wheel : Wheels.Wheels)
			{
				if (wheel.GetActiveSensor().IsAtRest() == false ||
					wheel.GetActiveSensor().IsInContact() == false ||
					wheel.GetActiveSensor().GetHitResult().Component->Mobility != EComponentMobility::Static)
				{
					idle = false;
					break;
				}
			}

			if (idle == true)
			{
				VehicleMesh->IdleAt(GetActorLocation(), GetActorQuat());
			}
		}

#pragma endregion VehicleGrip

	}
	else
	{
		// Come out of idle lock if we've gained any speed on throttle from the player or bot.

		if (GetSpeedKPH() > 1.0f ||
			FMath::Abs(Control.ThrottleInput) > 0.1f)
		{
			VehicleMesh->IdleUnlock();
		}
	}
}

#pragma endregion Miscellaneous

/**
* Construct a canard.
***********************************************************************************/

ACanard::ACanard()
{
	CanardMesh = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("CanardMesh"));

	CanardMesh->SetCollisionEnabled(ECollisionEnabled::Type::NoCollision);
	CanardMesh->SetCollisionProfileName(UCollisionProfile::NoCollision_ProfileName);
	CanardMesh->SetGenerateOverlapEvents(false);
	CanardMesh->Mobility = EComponentMobility::Movable;

	SetRootComponent(CanardMesh);
}

#pragma endregion Vehicle
