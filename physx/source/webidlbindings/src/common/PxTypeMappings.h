#ifndef PX_TYPE_MAPPINGS_H
#define PX_TYPE_MAPPINGS_H

#include "PxPhysicsAPI.h"

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

/**
 * PxArrayExt extends PxArray to get a slightly more Java(-script) friendly interface with get() and set() methods.
 * Also adds at(), push_back() and data() methods, so it can be used as drop-in replacement for std::vector.
 */
template <class T>
class PxArrayExt : public physx::PxArray<T> {
public:
    PX_INLINE PxArrayExt() : physx::PxArray<T>() { }
    PX_INLINE PxArrayExt(uint32_t size, const T& a = T()) : physx::PxArray<T>(size, a) { }

    PX_INLINE T& get(uint32_t index) { return this->operator[](index); }
    PX_INLINE void set(uint32_t index, const T& value) { get(index) = value; }

    // compatibility functions, so that PxArrayList can be used as replacement for std::vector
    PX_INLINE T& at(uint32_t index) { return this->operator[](index); }
    PX_INLINE void push_back(const T& value) { this->pushBack(value); }
    PX_INLINE T* data() { return this->begin(); }
};

class PxArray_PxVec3 : public PxArrayExt<physx::PxVec3> {
public:
    PxArray_PxVec3() : PxArrayExt<physx::PxVec3>() { }
    PxArray_PxVec3(uint32_t size) : PxArrayExt<physx::PxVec3>(size, physx::PxVec3(physx::PxZERO::PxZero)) { }
};

class PxArray_PxVec4 : public PxArrayExt<physx::PxVec4> {
public:
    PxArray_PxVec4() : PxArrayExt<physx::PxVec4>() { }
    PxArray_PxVec4(uint32_t size) : PxArrayExt<physx::PxVec4>(size, physx::PxVec4(physx::PxZERO::PxZero)) { }
};

typedef PxArrayExt<PxMaterialConstPtr> PxArray_PxMaterialConst;
typedef PxArrayExt<PxActorPtr> PxArray_PxActorPtr;
typedef PxArrayExt<physx::PxContactPairPoint> PxArray_PxContactPairPoint;
typedef PxArrayExt<physx::PxHeightFieldSample> PxArray_PxHeightFieldSample;
typedef PxArrayExt<physx::PxRaycastHit> PxArray_PxRaycastHit;
typedef PxArrayExt<physx::PxSweepHit> PxArray_PxSweepHit;
typedef PxArrayExt<physx::PxVehicleDrivableSurfaceType> PxArray_PxVehicleDrivableSurfaceType;
typedef PxArrayExt<physx::PxWheelQueryResult> PxArray_PxWheelQueryResult;
typedef PxArrayExt<PxVehicleWheelsPtr> PxArray_PxVehicleWheels;

typedef PxArrayExt<physx::PxReal> PxArray_PxReal;
typedef PxArrayExt<physx::PxU8> PxArray_PxU8;
typedef PxArrayExt<physx::PxU16> PxArray_PxU16;
typedef PxArrayExt<physx::PxU32> PxArray_PxU32;
typedef PxArray_PxVec3 PxArray_PxVec3;
typedef PxArray_PxVec4 PxArray_PxVec4;

// deprecated std::vector style types
typedef PxArrayExt<PxMaterialConstPtr> Vector_PxMaterialConst;
typedef PxArrayExt<PxActorPtr> Vector_PxActorPtr;
typedef PxArrayExt<physx::PxContactPairPoint> Vector_PxContactPairPoint;
typedef PxArrayExt<physx::PxHeightFieldSample> Vector_PxHeightFieldSample;
typedef PxArrayExt<physx::PxRaycastHit> Vector_PxRaycastHit;
typedef PxArrayExt<physx::PxSweepHit> Vector_PxSweepHit;
typedef PxArrayExt<physx::PxVehicleDrivableSurfaceType> Vector_PxVehicleDrivableSurfaceType;
typedef PxArrayExt<physx::PxWheelQueryResult> Vector_PxWheelQueryResult;
typedef PxArrayExt<PxVehicleWheelsPtr> Vector_PxVehicleWheels;

typedef PxArrayExt<physx::PxReal> Vector_PxReal;
typedef PxArrayExt<physx::PxU8> Vector_PxU8;
typedef PxArrayExt<physx::PxU16> Vector_PxU16;
typedef PxArrayExt<physx::PxU32> Vector_PxU32;
typedef PxArray_PxVec3 Vector_PxVec3;
typedef PxArray_PxVec4 Vector_PxVec4;

#endif
