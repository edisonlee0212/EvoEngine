#pragma once
#include "RayTracerUtilities.cuh"

namespace evo_engine {

    static __forceinline__ __device__ void
    BRDF(float metallic, Random &random, const glm::vec3 &inDirection, const glm::vec3 &inNormal,
         float3 &outDirection) {
        const glm::vec3 reflected = Reflect(inDirection, inNormal);
        const glm::vec3 newRayDirection =
                RandomSampleHemisphere(random, reflected, metallic);
        outDirection = make_float3(newRayDirection.x, newRayDirection.y, newRayDirection.z);
    }
}