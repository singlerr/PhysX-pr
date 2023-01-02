#ifndef WEB_IDL_BINDINGS_H
#define WEB_IDL_BINDINGS_H

#include <vector>
#include <cstring>
#include <iostream>

#include "PxPhysicsAPI.h"
#include "common/PxRenderOutput.h"
#include "extensions/PxCollectionExt.h"
#include "geomutils/PxContactBuffer.h"
#include "omnipvd/PxOmniPvd.h"
#include "pvd/PxPvdTransport.h"

#include "vehicle/Base.h"
#include "vehicle/DirectDrivetrain.h"
#include "vehicle/EngineDrivetrain.h"
#include "vehicle/PhysXIntegration.h"

// enums within namespaces are not supported by webidl binder, as a hack we can use typedefs
typedef physx::PxActorFlag::Enum PxActorFlagEnum;
typedef physx::PxPvdInstrumentationFlag::Enum PxPvdInstrumentationFlagEnum;
typedef physx::PxActorType::Enum PxActorTypeEnum;
typedef physx::PxActorTypeFlag::Enum PxActorTypeFlagEnum;
typedef physx::PxArticulationAxis::Enum PxArticulationAxisEnum;
typedef physx::PxArticulationCacheFlag::Enum PxArticulationCacheFlagEnum;
typedef physx::PxArticulationDriveType::Enum PxArticulationDriveTypeEnum;
typedef physx::PxArticulationFlag::Enum PxArticulationFlagEnum;
typedef physx::PxArticulationJointType::Enum PxArticulationJointTypeEnum;
typedef physx::PxArticulationKinematicFlag::Enum PxArticulationKinematicFlagEnum;
typedef physx::PxArticulationMotion::Enum PxArticulationMotionEnum;
typedef physx::PxArticulationSensorFlag::Enum PxArticulationSensorFlagEnum;
typedef physx::PxBaseFlag::Enum PxBaseFlagEnum;
typedef physx::PxBroadPhaseType::Enum PxBroadPhaseTypeEnum;
typedef physx::PxBVHBuildStrategy::Enum PxBVHBuildStrategyEnum;
typedef physx::PxCapsuleClimbingMode::Enum PxCapsuleClimbingModeEnum;
typedef physx::PxCombineMode::Enum PxCombineModeEnum;
typedef physx::PxConstraintFlag::Enum PxConstraintFlagEnum;
typedef physx::PxContactPairFlag::Enum PxContactPairFlagEnum;
typedef physx::PxContactPairHeaderFlag::Enum PxContactPairHeaderFlagEnum;
typedef physx::PxControllerBehaviorFlag::Enum PxControllerBehaviorFlagEnum;
typedef physx::PxControllerCollisionFlag::Enum PxControllerCollisionFlagEnum;
typedef physx::PxControllerNonWalkableMode::Enum PxControllerNonWalkableModeEnum;
typedef physx::PxControllerShapeType::Enum PxControllerShapeTypeEnum;
typedef physx::PxConvexFlag::Enum PxConvexFlagEnum;
typedef physx::PxConvexMeshCookingType::Enum PxConvexMeshCookingTypeEnum;
typedef physx::PxConvexMeshGeometryFlag::Enum PxConvexMeshGeometryFlagEnum;
typedef physx::PxD6Axis::Enum PxD6AxisEnum;
typedef physx::PxD6Drive::Enum PxD6DriveEnum;
typedef physx::PxD6Motion::Enum PxD6MotionEnum;
typedef physx::PxD6JointDriveFlag::Enum PxD6JointDriveFlagEnum;
typedef physx::PxDistanceJointFlag::Enum PxDistanceJointFlagEnum;
typedef physx::PxDynamicTreeSecondaryPruner::Enum PxDynamicTreeSecondaryPrunerEnum;
typedef physx::PxErrorCode::Enum PxErrorCodeEnum;
typedef physx::PxFilterFlag::Enum PxFilterFlagEnum;
typedef physx::PxFilterObjectFlag::Enum PxFilterObjectFlagEnum;
typedef physx::PxForceMode::Enum PxForceModeEnum;
typedef physx::PxFrictionType::Enum PxFrictionTypeEnum;
typedef physx::PxGeometryType::Enum PxGeometryTypeEnum;
typedef physx::PxHeightFieldFlag::Enum PxHeightFieldFlagEnum;
typedef physx::PxHeightFieldFormat::Enum PxHeightFieldFormatEnum;
typedef physx::PxHitFlag::Enum PxHitFlagEnum;
typedef physx::PxIDENTITY PxIDENTITYEnum;
typedef physx::PxJointActorIndex::Enum PxJointActorIndexEnum;
typedef physx::PxMaterialFlag::Enum PxMaterialFlagEnum;
typedef physx::PxMeshCookingHint::Enum PxMeshCookingHintEnum;
typedef physx::PxMeshFlag::Enum PxMeshFlagEnum;
typedef physx::PxMeshGeometryFlag::Enum PxMeshGeometryFlagEnum;
typedef physx::PxMeshMidPhase::Enum PxMeshMidPhaseEnum;
typedef physx::PxMeshPreprocessingFlag::Enum PxMeshPreprocessingFlagEnum;
typedef physx::PxPairFilteringMode::Enum PxPairFilteringModeEnum;
typedef physx::PxPairFlag::Enum PxPairFlagEnum;
typedef physx::PxParticleSolverType::Enum PxParticleSolverTypeEnum;
typedef physx::PxPrismaticJointFlag::Enum PxPrismaticJointFlagEnum;
typedef physx::PxPruningStructureType::Enum PxPruningStructureTypeEnum;
typedef physx::PxQueryFlag::Enum PxQueryFlagEnum;
typedef physx::PxQueryHitType::Enum PxQueryHitType;
typedef physx::PxRevoluteJointFlag::Enum PxRevoluteJointFlagEnum;
typedef physx::PxRigidBodyFlag::Enum PxRigidBodyFlagEnum;
typedef physx::PxRigidDynamicLockFlag::Enum PxRigidDynamicLockFlagEnum;
typedef physx::PxSceneFlag::Enum PxSceneFlagEnum;
typedef physx::PxSceneQueryUpdateMode::Enum PxSceneQueryUpdateModeEnum;
typedef physx::PxShapeFlag::Enum PxShapeFlagEnum;
typedef physx::PxSphericalJointFlag::Enum PxSphericalJointFlagEnum;
typedef physx::PxSolverType::Enum PxSolverTypeEnum;
typedef physx::PxTriangleMeshFlag::Enum PxTriangleMeshFlagEnum;
typedef physx::PxTriggerPairFlag::Enum PxTriggerPairFlagEnum;
typedef physx::vehicle2::PxVehicleAxes::Enum PxVehicleAxesEnum;
typedef physx::vehicle2::PxVehicleClutchAccuracyMode::Enum PxVehicleClutchAccuracyModeEnum;
typedef physx::vehicle2::PxVehicleCommandNonLinearResponseParams::Enum PxVehicleCommandNonLinearResponseParamsEnum;
typedef physx::vehicle2::PxVehicleCommandValueResponseTable::Enum PxVehicleCommandValueResponseTableEnum;
typedef physx::vehicle2::PxVehicleDirectDriveTransmissionCommandState::Enum PxVehicleDirectDriveTransmissionCommandStateEnum;
typedef physx::vehicle2::PxVehicleEngineDriveTransmissionCommandState::Enum PxVehicleEngineDriveTransmissionCommandStateEnum;
typedef physx::vehicle2::PxVehicleGearboxParams::Enum PxVehicleGearboxParamsEnum;
typedef physx::vehicle2::PxVehicleLimits::Enum PxVehicleLimitsEnum;
typedef physx::vehicle2::PxVehiclePhysXActorUpdateMode::Enum PxVehiclePhysXActorUpdateModeEnum;
typedef physx::vehicle2::PxVehiclePhysXConstraintLimits::Enum PxVehiclePhysXConstraintLimitsEnum;
typedef physx::vehicle2::PxVehiclePhysXRoadGeometryQueryType::Enum PxVehiclePhysXRoadGeometryQueryTypeEnum;
typedef physx::vehicle2::PxVehiclePhysXSuspensionLimitConstraintParams::DirectionSpecifier PxVehiclePhysXSuspensionLimitConstraintParamsDirectionSpecifierEnum;
typedef physx::vehicle2::PxVehicleSimulationContextType::Enum PxVehicleSimulationContextTypeEnum;
typedef physx::vehicle2::PxVehicleSuspensionJounceCalculationType::Enum PxVehicleSuspensionJounceCalculationTypeEnum;
typedef physx::vehicle2::PxVehicleTireDirectionModes::Enum PxVehicleTireDirectionModesEnum;
typedef physx::PxVisualizationParameter::Enum PxVisualizationParameterEnum;

// typedefs for vehicle lookup tables
typedef physx::vehicle2::PxVehicleFixedSizeLookupTable<physx::PxReal,3> PxVehicleFixedSizeLookupTableFloat_3;
typedef physx::vehicle2::PxVehicleFixedSizeLookupTable<physx::PxVec3,3> PxVehicleFixedSizeLookupTableVec3_3;
typedef physx::vehicle2::PxVehicleFixedSizeLookupTable<physx::PxReal,8> PxVehicleTorqueCurveLookupTable;

// typedefs for pointer types
typedef const physx::PxU8* PxU8ConstPtr;
typedef const physx::PxU16* PxU16ConstPtr;
typedef const physx::PxU32* PxU32ConstPtr;
typedef const physx::PxMaterial* PxMaterialConstPtr;
typedef physx::PxU8* PxU8Ptr;
typedef physx::PxU16* PxU16Ptr;
typedef physx::PxU32* PxU32Ptr;
typedef physx::PxReal* PxRealPtr;
typedef physx::PxMaterial* PxMaterialPtr;
typedef physx::PxActor* PxActorPtr;
typedef physx::PxVehicleWheels* PxVehicleWheelsPtr;

// template classes are not supported by webidl binder, as a hack we can use typedefs
typedef physx::PxFixedSizeLookupTable<physx::PxVehicleEngineData::eMAX_NB_ENGINE_TORQUE_CURVE_ENTRIES> PxEngineTorqueLookupTable;
typedef physx::PxTypedStridedData<physx::PxU16> PxU16StridedData;

typedef physx::PxOverlapBufferN<10> PxOverlapBuffer10;
typedef physx::PxRaycastBufferN<10> PxRaycastBuffer10;
typedef physx::PxSweepBufferN<10> PxSweepBuffer10;

typedef std::vector<PxMaterialConstPtr> Vector_PxMaterialConst;
typedef std::vector<PxActorPtr> Vector_PxActorPtr;
typedef std::vector<physx::PxContactPairPoint> Vector_PxContactPairPoint;
typedef std::vector<physx::PxHeightFieldSample> Vector_PxHeightFieldSample;
typedef std::vector<physx::PxRaycastHit> Vector_PxRaycastHit;
typedef std::vector<physx::PxSweepHit> Vector_PxSweepHit;
typedef std::vector<physx::PxVehicleDrivableSurfaceType> Vector_PxVehicleDrivableSurfaceType;
typedef std::vector<physx::PxWheelQueryResult> Vector_PxWheelQueryResult;
typedef std::vector<PxVehicleWheelsPtr> Vector_PxVehicleWheels;

typedef std::vector<physx::PxReal> Vector_PxReal;
typedef std::vector<physx::PxU8> Vector_PxU8;
typedef std::vector<physx::PxU16> Vector_PxU16;
typedef std::vector<physx::PxU32> Vector_PxU32;
typedef std::vector<physx::PxVec3> Vector_PxVec3;

class PassThroughFilterShader {
    public:
        virtual physx::PxU32 filterShader(physx::PxU32 attributes0,
                                          physx::PxU32 filterData0w0, physx::PxU32 filterData0w1, physx::PxU32 filterData0w2, physx::PxU32 filterData0w3,
                                          physx::PxU32 attributes1,
                                          physx::PxU32 filterData1w0, physx::PxU32 filterData1w1, physx::PxU32 filterData1w2, physx::PxU32 filterData1w3) = 0;

        virtual ~PassThroughFilterShader() { }

        physx::PxU32 outputPairFlags;
};

physx::PxFilterFlags passThrFilterShader(physx::PxFilterObjectAttributes attributes0, physx::PxFilterData filterData0,
                                         physx::PxFilterObjectAttributes attributes1, physx::PxFilterData filterData1,
                                         physx::PxPairFlags& pairFlags, const void* constantBlock, physx::PxU32 constantBlockSize) {
    PX_UNUSED(constantBlockSize);

    PassThroughFilterShader* shader = *((PassThroughFilterShader* const *) constantBlock);
    shader->outputPairFlags = physx::PxPairFlag::eCONTACT_DEFAULT;

    physx::PxFilterFlags result = physx::PxFilterFlags(static_cast<physx::PxU16>(shader->filterShader(
            (physx::PxU32) attributes0, filterData0.word0, filterData0.word1, filterData0.word2, filterData0.word3,
            (physx::PxU32) attributes1, filterData1.word0, filterData1.word1, filterData1.word2, filterData1.word3)));

    pairFlags = physx::PxPairFlags(static_cast<physx::PxU16>(shader->outputPairFlags));
    return result;
}

// default scene filter / query shaders, implemented in C++ for performance reasons
physx::PxFilterFlags defaultFilterShader(physx::PxFilterObjectAttributes attributes0, physx::PxFilterData filterData0,
                                         physx::PxFilterObjectAttributes attributes1, physx::PxFilterData filterData1,
                                         physx::PxPairFlags& pairFlags, const void* constantBlock, physx::PxU32 constantBlockSize) {
    PX_UNUSED(constantBlock);
    PX_UNUSED(constantBlockSize);

    if ((0 == (filterData0.word0 & filterData1.word1)) && (0 == (filterData1.word0 & filterData0.word1))) {
        return physx::PxFilterFlag::eSUPPRESS;
    }

    if (physx::PxFilterObjectIsTrigger(attributes0) || physx::PxFilterObjectIsTrigger(attributes1)) {
        pairFlags = physx::PxPairFlag::eTRIGGER_DEFAULT;
    } else {
        pairFlags = physx::PxPairFlag::eCONTACT_DEFAULT;
    }
    pairFlags |= physx::PxPairFlags(physx::PxU16(filterData0.word2 | filterData1.word2));

    return physx::PxFilterFlag::eDEFAULT;
}

// Slightly simplified SimulationEventCallback which can be implemented in non-native code
class SimpleSimulationEventCallback : physx::PxSimulationEventCallback {
    public:
        virtual void onConstraintBreak(physx::PxConstraintInfo*, physx::PxU32) = 0;
        virtual void onWake(physx::PxActor**, physx::PxU32) = 0;
        virtual void onSleep(physx::PxActor**, physx::PxU32) = 0;
        virtual void onContact(const physx::PxContactPairHeader&, const physx::PxContactPair*, physx::PxU32 nbPairs) = 0;
        virtual void onTrigger(physx::PxTriggerPair*, physx::PxU32) = 0;

        // implement onAdvance with empty body so it does not have to be implemented
        // in non-native code (for the sake of performance)
        virtual void onAdvance(const physx::PxRigidBody *const *, const physx::PxTransform*, const physx::PxU32) { }
};

// Slightly simplified PxCustomGeometry::Callbacks which can be implemented in non-native code
class SimpleCustomGeometryCallbacks : public physx::PxCustomGeometry::Callbacks {
    public:
        // non-const virtual methods
        virtual physx::PxBounds3* getLocalBoundsImpl(const physx::PxGeometry& geometry) = 0;
        virtual bool generateContactsImpl(const physx::PxGeometry& geom0, const physx::PxGeometry& geom1, const physx::PxTransform& pose0, const physx::PxTransform& pose1,
            physx::PxReal contactDistance, physx::PxReal meshContactMargin, physx::PxReal toleranceLength,
            physx::PxContactBuffer& contactBuffer) = 0;
        virtual physx::PxU32 raycastImpl(const physx::PxVec3& origin, const physx::PxVec3& unitDir, const physx::PxGeometry& geom, const physx::PxTransform& pose,
            physx::PxReal maxDist, physx::PxHitFlags& hitFlags, physx::PxU32 maxHits, physx::PxGeomRaycastHit* rayHits, physx::PxU32 stride) = 0;
        virtual bool overlapImpl(const physx::PxGeometry& geom0, const physx::PxTransform& pose0, const physx::PxGeometry& geom1, const physx::PxTransform& pose1) = 0;
        virtual bool sweepImpl(const physx::PxVec3& unitDir, physx::PxReal maxDist,
            const physx::PxGeometry& geom0, const physx::PxTransform& pose0, const physx::PxGeometry& geom1, const physx::PxTransform& pose1,
            physx::PxGeomSweepHit& sweepHit, physx::PxHitFlags& hitFlags, physx::PxReal inflation) = 0;
        virtual void computeMassPropertiesImpl(const physx::PxGeometry& geometry, physx::PxMassProperties& massProperties) = 0;
        virtual bool usePersistentContactManifoldImpl(const physx::PxGeometry& geometry) = 0;

        // original callbacks methods, forwarding to non-const methods
        virtual physx::PxBounds3 getLocalBounds(const physx::PxGeometry& geometry) const {
            return *((SimpleCustomGeometryCallbacks*) this)->getLocalBoundsImpl(geometry);
        }
        virtual bool generateContacts(const physx::PxGeometry& geom0, const physx::PxGeometry& geom1, const physx::PxTransform& pose0, const physx::PxTransform& pose1,
            const physx::PxReal contactDistance, const physx::PxReal meshContactMargin, const physx::PxReal toleranceLength,
            physx::PxContactBuffer& contactBuffer) const
        {
            return ((SimpleCustomGeometryCallbacks*) this)->generateContactsImpl(geom0, geom1, pose0, pose1, contactDistance, meshContactMargin, toleranceLength, contactBuffer);
        }
        virtual physx::PxU32 raycast(const physx::PxVec3& origin, const physx::PxVec3& unitDir, const physx::PxGeometry& geom, const physx::PxTransform& pose,
            physx::PxReal maxDist, physx::PxHitFlags hitFlags, physx::PxU32 maxHits, physx::PxGeomRaycastHit* rayHits, physx::PxU32 stride, physx::PxRaycastThreadContext*) const
        {
            return ((SimpleCustomGeometryCallbacks*) this)->raycastImpl(origin, unitDir, geom, pose, maxDist, hitFlags, maxHits, rayHits, stride);
        }
        virtual bool overlap(const physx::PxGeometry& geom0, const physx::PxTransform& pose0, const physx::PxGeometry& geom1, const physx::PxTransform& pose1, physx::PxOverlapThreadContext*) const
        {
            return ((SimpleCustomGeometryCallbacks*) this)->overlapImpl(geom0, pose0, geom1, pose1);
        }
        virtual bool sweep(const physx::PxVec3& unitDir, const physx::PxReal maxDist,
            const physx::PxGeometry& geom0, const physx::PxTransform& pose0, const physx::PxGeometry& geom1, const physx::PxTransform& pose1,
            physx::PxGeomSweepHit& sweepHit, physx::PxHitFlags hitFlags, const physx::PxReal inflation, physx::PxSweepThreadContext*) const
        {
            return ((SimpleCustomGeometryCallbacks*) this)->sweepImpl(unitDir, maxDist, geom0, pose0, geom1, pose1, sweepHit, hitFlags, inflation);
        }
        virtual void computeMassProperties(const physx::PxGeometry& geometry, physx::PxMassProperties& massProperties) const
        {
            return ((SimpleCustomGeometryCallbacks*) this)->computeMassPropertiesImpl(geometry, massProperties);
        }
        virtual bool usePersistentContactManifold(const physx::PxGeometry& geometry, physx::PxReal& breakingThreshold) const
        {
            bool retVal = ((SimpleCustomGeometryCallbacks*) this)->usePersistentContactManifoldImpl(geometry);
            breakingThreshold = persistentContactManifold_outBreakingThreshold;
            return retVal;
        }

        // unused / not-available in non-native code
        virtual physx::PxCustomGeometry::Type getCustomType() const { return physx::PxCustomGeometry::Type(); }
        virtual void visualize(const physx::PxGeometry&, physx::PxRenderOutput&, const physx::PxTransform&, const physx::PxBounds3&) const { }

        physx::PxReal persistentContactManifold_outBreakingThreshold;
};

class SimplePvdTransport : physx::PxPvdTransport {
    public:
        SimplePvdTransport() { }

        virtual bool connect() = 0;
        virtual bool isConnected() = 0;
        virtual void send(void* inBytes, uint32_t inLength) = 0;
        virtual void flush() = 0;
        virtual void disconnect() = 0;

        bool write(const uint8_t *inBytes, uint32_t inLength) {
            send((void*) inBytes, inLength);
            return true;
        }

        PxPvdTransport &lock() {
            return *this;
        }

        void unlock() { }
        uint64_t getWrittenDataSize() { return 0; }
        void release() { }
};

class SimpleControllerBehaviorCallback : physx::PxControllerBehaviorCallback {
    public:
        virtual physx::PxU32 getShapeBehaviorFlags(const physx::PxShape& shape, const physx::PxActor& actor) = 0;
        virtual physx::PxU32 getControllerBehaviorFlags(const physx::PxController& controller) = 0;
        virtual physx::PxU32 getObstacleBehaviorFlags(const physx::PxObstacle& obstacle) = 0;

        virtual physx::PxControllerBehaviorFlags getBehaviorFlags(const physx::PxShape& shape, const physx::PxActor& actor) {
            return physx::PxControllerBehaviorFlags(static_cast<physx::PxU8>(getShapeBehaviorFlags(shape, actor)));
        }
        virtual physx::PxControllerBehaviorFlags getBehaviorFlags(const physx::PxController& controller) {
            return physx::PxControllerBehaviorFlags(static_cast<physx::PxU8>(getControllerBehaviorFlags(controller)));
        }
        virtual physx::PxControllerBehaviorFlags getBehaviorFlags(const physx::PxObstacle& obstacle) {
            return physx::PxControllerBehaviorFlags(static_cast<physx::PxU8>(getObstacleBehaviorFlags(obstacle)));
        }

        virtual ~SimpleControllerBehaviorCallback() { }
};

class SimpleQueryFilterCallback : physx::PxQueryFilterCallback {
    public:
        virtual physx::PxU32 simplePreFilter(const physx::PxFilterData& filterData, const physx::PxShape* shape, const physx::PxRigidActor* actor, physx::PxHitFlags &queryFlags) = 0;
        virtual physx::PxU32 simplePostFilter(const physx::PxFilterData& filterData, const physx::PxQueryHit& hit) = 0;

        virtual physx::PxQueryHitType::Enum preFilter(const physx::PxFilterData& filterData, const physx::PxShape* shape, const physx::PxRigidActor* actor, physx::PxHitFlags &queryFlags) {
            return static_cast<physx::PxQueryHitType::Enum>(simplePreFilter(filterData, shape, actor, queryFlags));
        }
        virtual physx::PxQueryHitType::Enum postFilter(const physx::PxFilterData& filterData, const physx::PxQueryHit& hit) {
            return static_cast<physx::PxQueryHitType::Enum>(simplePostFilter(filterData, hit));
        }

        virtual ~SimpleQueryFilterCallback() { }
};

// top-level functions are not supported by webidl binder, we need to wrap them in a class
struct PxTopLevelFunctions {
    static const physx::PxU32 PHYSICS_VERSION = PX_PHYSICS_VERSION;

    static physx::PxSimulationFilterShader DefaultFilterShader() {
        return &defaultFilterShader;
    }

    static void setupPassThroughFilterShader(physx::PxSceneDesc* sceneDesc, PassThroughFilterShader* filterShader) {
        PassThroughFilterShader** data = new PassThroughFilterShader*[1];
        data[0] = filterShader;
        sceneDesc->filterShader = &passThrFilterShader;
        sceneDesc->filterShaderData = data;
        sceneDesc->filterShaderDataSize = sizeof(PassThroughFilterShader*);
    }

    static physx::PxFoundation* CreateFoundation(physx::PxU32 version, physx::PxDefaultAllocator& allocator, physx::PxErrorCallback& errorCallback) {
        return PxCreateFoundation(version, allocator, errorCallback);
    }

    static physx::PxPhysics *CreatePhysics(physx::PxU32 version, physx::PxFoundation &foundation, const physx::PxTolerancesScale &scale, physx::PxPvd* pvd = NULL, physx::PxOmniPvd* omniPvd = NULL) {
        return PxCreatePhysics(version, foundation, scale, false, pvd, omniPvd);
    }

    static physx::PxCooking* CreateCooking(physx::PxU32 version, physx::PxFoundation& foundation, const physx::PxCookingParams& params) {
        return PxCreateCooking(version, foundation, params);
    }

    static physx::PxControllerManager* CreateControllerManager(physx::PxScene& scene, bool lockingEnabled = false) {
        return PxCreateControllerManager(scene, lockingEnabled);
    }

    static physx::PxPvd *CreatePvd(physx::PxFoundation &foundation) {
        return PxCreatePvd(foundation);
    }

    static physx::PxPvdTransport* DefaultPvdSocketTransportCreate(const char *host, int port, unsigned int timeoutInMilliseconds) {
        return physx::PxDefaultPvdSocketTransportCreate(host, port, timeoutInMilliseconds);
    }

    static physx::PxOmniPvd *CreateOmniPvd(physx::PxFoundation &foundation) {
        return PxCreateOmniPvd(foundation);
    }

    static physx::PxDefaultCpuDispatcher* DefaultCpuDispatcherCreate(physx::PxU32 numThreads) {
        return physx::PxDefaultCpuDispatcherCreate(numThreads);
    }

    static bool InitExtensions(physx::PxPhysics& physics) {
        return PxInitExtensions(physics, NULL);
    }

    static void CloseExtensions() {
        PxCloseExtensions();
    }

    static physx::PxD6Joint* D6JointCreate(physx::PxPhysics& physics, physx::PxRigidActor* actor0, physx::PxTransform& localFrame0, physx::PxRigidActor* actor1, physx::PxTransform& localFrame1) {
        return physx::PxD6JointCreate(physics, actor0, localFrame0, actor1, localFrame1);
    }

    static physx::PxDistanceJoint* DistanceJointCreate(physx::PxPhysics& physics, physx::PxRigidActor* actor0, physx::PxTransform& localFrame0, physx::PxRigidActor* actor1, physx::PxTransform& localFrame1) {
        return physx::PxDistanceJointCreate(physics, actor0, localFrame0, actor1, localFrame1);
    }

    static physx::PxFixedJoint* FixedJointCreate(physx::PxPhysics& physics, physx::PxRigidActor* actor0, physx::PxTransform& localFrame0, physx::PxRigidActor* actor1, physx::PxTransform& localFrame1) {
        return physx::PxFixedJointCreate(physics, actor0, localFrame0, actor1, localFrame1);
    }

    static physx::PxPrismaticJoint* PrismaticJointCreate(physx::PxPhysics& physics, physx::PxRigidActor* actor0, physx::PxTransform& localFrame0, physx::PxRigidActor* actor1, physx::PxTransform& localFrame1) {
        return physx::PxPrismaticJointCreate(physics, actor0, localFrame0, actor1, localFrame1);
    }

    static physx::PxRevoluteJoint* RevoluteJointCreate(physx::PxPhysics& physics, physx::PxRigidActor* actor0, physx::PxTransform& localFrame0, physx::PxRigidActor* actor1, physx::PxTransform& localFrame1) {
        return physx::PxRevoluteJointCreate(physics, actor0, localFrame0, actor1, localFrame1);
    }

    static physx::PxSphericalJoint* SphericalJointCreate(physx::PxPhysics& physics, physx::PxRigidActor* actor0, physx::PxTransform& localFrame0, physx::PxRigidActor* actor1, physx::PxTransform& localFrame1) {
        return physx::PxSphericalJointCreate(physics, actor0, localFrame0, actor1, localFrame1);
    }

    static physx::PxConvexMesh* CreateConvexMesh(const physx::PxCookingParams& params, const physx::PxConvexMeshDesc& desc) {
        return PxCreateConvexMesh(params, desc);
    }

    static physx::PxTriangleMesh* CreateTriangleMesh(const physx::PxCookingParams &params, const physx::PxTriangleMeshDesc &desc) {
        return PxCreateTriangleMesh(params, desc);
    }

    static physx::PxHeightField* CreateHeightField(const physx::PxHeightFieldDesc &desc) {
        return PxCreateHeightField(desc);
    }
};

struct PxVehicleTopLevelFunctions {

    static bool InitVehicleExtension(physx::PxFoundation& foundation) {
        return physx::vehicle2::PxInitVehicleExtension(foundation);
    }

    static void CloseVehicleExtension() {
        physx::vehicle2::PxCloseVehicleExtension();
    }

    static bool VehicleComputeSprungMasses(physx::PxU32 nbSprungMasses, const Vector_PxVec3& sprungMassCoordinates, physx::PxReal totalMass, PxVehicleAxesEnum gravityDirection, Vector_PxReal& sprungMasses) {
        return physx::vehicle2::PxVehicleComputeSprungMasses(nbSprungMasses, sprungMassCoordinates.data(), totalMass, gravityDirection, sprungMasses.data());
    }

    static physx::PxConvexMesh* VehicleUnitCylinderSweepMeshCreate(const physx::vehicle2::PxVehicleFrame& vehicleFrame, physx::PxPhysics& physics, const physx::PxCookingParams& params) {
        return physx::vehicle2::PxVehicleUnitCylinderSweepMeshCreate(vehicleFrame, physics, params);
    }

    static void VehicleUnitCylinderSweepMeshDestroy(physx::PxConvexMesh* mesh) {
        physx::vehicle2::PxVehicleUnitCylinderSweepMeshDestroy(mesh);
    }

    static const physx::PxU32 MAX_NB_ENGINE_TORQUE_CURVE_ENTRIES = physx::vehicle2::PxVehicleEngineParams::eMAX_NB_ENGINE_TORQUE_CURVE_ENTRIES;
};

struct PxVehicleTireForceParamsExt {
    static void setFrictionVsSlip(physx::vehicle2::PxVehicleTireForceParams* tireForceParams, int i, int j, float value) {
        tireForceParams->frictionVsSlip[i][j] = value;
    }

    static void setLoadFilter(physx::vehicle2::PxVehicleTireForceParams* tireForceParams, int i, int j, float value) {
        tireForceParams->loadFilter[i][j] = value;
    }
};

// Various helper functions for pointer access and conversion
struct NativeArrayHelpers {
    static physx::PxU8 getU8At(const physx::PxU8* base, int index) {
        return base[index];
    }

    static physx::PxU16 getU16At(const physx::PxU16* base, int index) {
        return base[index];
    }

    static physx::PxU32 getU32At(const physx::PxU32* base, int index) {
        return base[index];
    }

    static physx::PxReal getRealAt(physx::PxReal* base, int index) {
        return base[index];
    }

    static physx::PxActor* getActorAt(physx::PxActor* base, int index) {
        return &base[index];
    }

    static physx::PxBounds3* getBounds3At(physx::PxBounds3* base, int index) {
        return &base[index];
    }

    static physx::PxContactPair* getContactPairAt(physx::PxContactPair* base, int index) {
        return &base[index];
    }

    static physx::PxContactPairHeader* getContactPairHeaderAt(physx::PxContactPairHeader* base, int index) {
        return &base[index];
    }

    static physx::PxController* getControllerAt(physx::PxController* base, int index) {
        return &base[index];
    }

    static physx::PxControllerShapeHit* getControllerShapeHitAt(physx::PxControllerShapeHit* base, int index) {
        return &base[index];
    }

    static physx::PxControllersHit* getControllersHitAt(physx::PxControllersHit* base, int index) {
        return &base[index];
    }

    static physx::PxControllerObstacleHit* getControllerObstacleHitAt(physx::PxControllerObstacleHit* base, int index) {
        return &base[index];
    }

    static physx::PxObstacle* getObstacleAt(physx::PxObstacle* base, int index) {
        return &base[index];
    }

    static physx::PxShape* getShapeAt(physx::PxShape* base, int index) {
        return &base[index];
    }

    static physx::PxTriggerPair* getTriggerPairAt(physx::PxTriggerPair* base, int index) {
        return &base[index];
    }

    static physx::PxVec3* getVec3At(physx::PxVec3* base, int index) {
        return &base[index];
    }

    static PxU8Ptr voidToU8Ptr(void* voidPtr) {
        return (PxU8Ptr) voidPtr;
    }

    static PxU16Ptr voidToU16Ptr(void* voidPtr) {
        return (PxU16Ptr) voidPtr;
    }

    static PxU32Ptr voidToU32Ptr(void* voidPtr) {
        return (PxU32Ptr) voidPtr;
    }

    static PxRealPtr voidToRealPtr(void* voidPtr) {
        return (PxRealPtr) voidPtr;
    }
};

// Helper functions for accessing functions, which don't map well to JS / Java
struct SupportFunctions {
    static physx::PxShape* PxActor_getShape(physx::PxRigidActor& actor, physx::PxU32 i) {
        physx::PxShape* shapePtr;
        actor.getShapes(&shapePtr, 1, i);
        return shapePtr;
    }

    static physx::PxActor* PxContactPairHeader_getActor(physx::PxContactPairHeader& pairHeader, physx::PxU32 i) {
        return pairHeader.actors[i];
    }

    static Vector_PxActorPtr& PxScene_getActiveActors(physx::PxScene* scene) {
        static Vector_PxActorPtr activeActors;
        physx::PxU32 nbActors;
        physx::PxActor** actors = scene->getActiveActors(nbActors);

        activeActors.resize(static_cast<size_t>(nbActors));
        std::memcpy(activeActors.data(), actors, static_cast<size_t>(sizeof(physx::PxActor*) * nbActors));
        return activeActors;
    }
};

#endif