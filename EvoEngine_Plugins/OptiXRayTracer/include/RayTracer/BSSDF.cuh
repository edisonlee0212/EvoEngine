#pragma once

#include "RayTracerUtilities.cuh"

namespace evo_engine {
    struct SSHitRecord {
        glm::vec3 m_outPosition;
        glm::vec3 m_outNormal;
    };

    struct SSPerRayData {
        unsigned long long handle;
        Random random;
        int m_recordSize = 0;
        SSHitRecord m_records[4];
    };

    static __forceinline__ __device__ void SSAnyHit() {
        const auto &sbtData = *(const SBT *) optixGetSbtDataPointer();
        SSPerRayData &perRayData =
                *GetRayDataPointer<SSPerRayData>();
        if (perRayData.handle != sbtData.m_handle) {
            optixIgnoreIntersection();
        }
        if (perRayData.m_recordSize >= 4) optixTerminateRay();
        const float3 rayDirectionInternal = optixGetWorldRayDirection();
        glm::vec3 rayDirection = glm::vec3(
                rayDirectionInternal.x, rayDirectionInternal.y, rayDirectionInternal.z);
        auto hit_info = sbtData.GetHitInfo(rayDirection, false);

        static_cast<SurfaceMaterial *>(sbtData.m_material)
                ->ApplyNormalTexture(hit_info.normal, hit_info.tex_coord, hit_info.tangent);

        perRayData.m_records[perRayData.m_recordSize].m_outNormal = hit_info.normal;
        perRayData.m_records[perRayData.m_recordSize].m_outPosition = hit_info.position;
        perRayData.m_recordSize++;
    }

    static __forceinline__ __device__ void SSHit() {
        const auto &sbtData = *(const SBT *) optixGetSbtDataPointer();
        SSPerRayData &perRayData =
                *GetRayDataPointer<SSPerRayData>();
    }

    static __forceinline__ __device__ bool
    BSSRDF(float metallic, Random &random, float radius, unsigned long long handle, OptixTraversableHandle traversable,
           const glm::vec3 &inPosition, const glm::vec3 &inDirection, const glm::vec3 &inNormal,
           float3 &outPosition, float3 &outDirection, glm::vec3 &outNormal) {
        glm::vec3 diskNormal = inNormal; //RandomSampleHemisphere(random, inNormal);
        glm::vec3 diskCenter = inPosition + radius * diskNormal / 2.0f;
        float diskRadius = radius * glm::sqrt(random());
        float distance = glm::sqrt(radius * radius - diskRadius * diskRadius);
        glm::vec3 samplePosition = diskCenter +
                                   diskRadius * glm::rotate(glm::vec3(diskNormal.y, diskNormal.z, diskNormal.x),
                                                            2.0f * glm::pi<float>() * random(), diskNormal);
        glm::vec3 sampleDirection = -diskNormal;
        SSPerRayData perRayData;
        perRayData.handle = handle;
        perRayData.m_recordSize = 0;
        perRayData.random = random;
        uint32_t u0, u1;
        PackRayDataPointer(&perRayData, u0, u1);
        optixTrace(
                traversable, make_float3(samplePosition.x, samplePosition.y, samplePosition.z),
                make_float3(sampleDirection.x, sampleDirection.y, sampleDirection.z),
                distance, // tmin
                radius + distance, // tmax
                0.0f,  // rayTime
                static_cast<OptixVisibilityMask>(255), OPTIX_RAY_FLAG_NONE,
                static_cast<int>(
                        RayType::SpacialSampling), // SBT offset
                static_cast<int>(
                        RayType::RayTypeCount), // SBT stride
                static_cast<int>(
                        RayType::SpacialSampling), // missSBTIndex
                u0, u1);
        if (perRayData.m_recordSize > 0) {
            int index = glm::clamp((int)(perRayData.random() * perRayData.m_recordSize), 0, perRayData.m_recordSize - 1);
            if (glm::distance(inPosition, perRayData.m_records[index].m_outPosition) <= radius) {
                outNormal = perRayData.m_records[index].m_outNormal;
                auto out = perRayData.m_records[index].m_outPosition + outNormal * 0.01f;
                outPosition = make_float3(out.x, out.y, out.z);
                //outDirection = make_float3(outNormal.x, outNormal.y, outNormal.z);
                BRDF(metallic, random, -outNormal, outNormal, outDirection);
                return true;
            }
        }
        return false;
    }
}