#ifndef CUDA_BINDINGS_H
#define CUDA_BINDINGS_H

#include "PxPhysicsAPI.h"

typedef physx::PxCudaInteropMode::Enum PxCudaInteropModeEnum;

struct PxCudaTopLevelFunctions {
    static physx::PxCudaContextManager* CreateCudaContextManager(physx::PxFoundation& foundation, const physx::PxCudaContextManagerDesc& desc) {
        #if defined(__EMSCRIPTEN__) || defined(__APPLE__)
            PX_UNUSED(foundation);
            PX_UNUSED(desc);
            return NULL;
        #else
            return PxCreateCudaContextManager(foundation, desc);
        #endif
    }
};

#endif