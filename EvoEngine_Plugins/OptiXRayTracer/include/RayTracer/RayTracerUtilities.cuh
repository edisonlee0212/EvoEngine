#pragma once

#include <LinearCongruenceGenerator.hpp>
#define GLM_ENABLE_EXPERIMENTAL
#include "glm/glm.hpp"
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/vector_angle.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtc/random.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <optix_device.h>

#include <fstream>
#include <random>
#include <sstream>
#include <vector>

#include <CUDAModule.hpp>
#include <Optix7.hpp>
#include <RayDataDefinations.hpp>

namespace evo_engine {
    typedef LinearCongruenceGenerator<16> Random;
#pragma region Data
    template<typename T>
    struct PerRayData {
        unsigned hit_count;
        Random random;
        T energy;
        glm::vec3 normal;
        glm::vec3 albedo;
        glm::vec3 position;
    };


#pragma endregion
#pragma region Helpers

    static __forceinline__ __device__ void *
    UnpackRayDataPointer(const uint32_t &i0, const uint32_t &i1) {
        const uint64_t uPointer = static_cast<uint64_t>(i0) << 32 | i1;
        void *pointer = reinterpret_cast<void *>(uPointer);
        return pointer;
    }

    static __forceinline__ __device__ void
    PackRayDataPointer(void *ptr, uint32_t &i0, uint32_t &i1) {
        const auto uPointer = reinterpret_cast<uint64_t>(ptr);
        i0 = uPointer >> 32;
        i1 = uPointer & 0x00000000ffffffff;
    }

    template<typename T>
    static __forceinline__ __device__ T
    SampleCubeMap(const cudaTextureObject_t cubeMap[], const glm::vec3 &direction) {
        const float absX = abs(direction.x);
        const float absY = abs(direction.y);
        const float absZ = abs(direction.z);
        float ma;
        int faceIndex;
        glm::vec2 uv;
        if (absZ >= absX && absZ >= absY) {
            faceIndex = direction.z < 0.0 ? 5 : 4;
            ma = 0.5f / absZ;
            uv =
                    glm::vec2(direction.z < 0.0 ? -direction.x : direction.x, -direction.y);
        } else if (absY >= absX) {
            faceIndex = direction.y < 0.0 ? 3 : 2;
            ma = 0.5f / absY;
            uv = glm::vec2(direction.x, direction.y > 0.0 ? direction.z : -direction.z);
        } else {
            faceIndex = direction.x < 0.0 ? 1 : 0;
            ma = 0.5f / absX;
            uv =
                    glm::vec2(direction.x < 0.0 ? direction.z : -direction.z, -direction.y);
        }
        uv = uv * ma + glm::vec2(0.5);
        return tex2D<T>(cubeMap[faceIndex], uv.x, uv.y);
    }

    template<typename T>
    static __forceinline__ __device__ T *GetRayDataPointer() {
        const uint32_t u0 = optixGetPayload_0();
        const uint32_t u1 = optixGetPayload_1();
        return static_cast<T *>(UnpackRayDataPointer(u0, u1));
    }

    static __forceinline__ __device__ glm::vec3 Reflect(const glm::vec3 &incident,
                                                        const glm::vec3 &normal) {
        return incident - 2 * glm::dot(incident, normal) * normal;
    }

    static __forceinline__ __device__ glm::vec3
    Refract(const glm::vec3 &incident, const glm::vec3 &normal, const float &ior) {
        float cosI = glm::clamp(glm::dot(incident, normal), -1.0f, 1.0f);
        float etai = 1, etat = ior;
        glm::vec3 n = normal;
        if (cosI < 0) {
            cosI = -cosI;
        } else {
            std::swap(etai, etat);
            n = -normal;
        }
        const float eta = etai / etat;
        const float k = 1 - eta * eta * (1 - cosI * cosI);
        return k < 0 ? glm::vec3(0.0f) : incident * eta + (eta * cosI - sqrtf(k)) * n;
    }

    static __forceinline__ __device__ glm::mat3x3
    GetTangentSpace(const glm::vec3 &normal) {
        // Choose a helper vector for the cross product
        glm::vec3 helper = glm::vec3(1.0f, 0.0f, 0.0f);
        if (abs(normal.x) > 0.99f)
            helper = glm::vec3(0.0f, 0.0f, 1.0f);
        // Generate vectors
        const auto tangent = glm::normalize(cross(normal, helper));
        const auto binormal = glm::normalize(cross(normal, tangent));
        return glm::mat3x3(tangent, binormal, normal);
    }

    static __forceinline__ __device__ glm::vec3
    RandomSampleHemisphere(Random &random, const glm::vec3 &normal,
                           const float &alpha) {
        // Uniformly sample hemisphere direction
        auto cosTheta = 1.0f - random() * (1.0f - alpha) * (1.0f - alpha);
        const auto sinTheta = sqrt(glm::max(0.0f, 1.0f - cosTheta * cosTheta));
        const auto phi = 2.0f * glm::pi<float>() * random();
        const auto tangentSpaceDir =
                glm::vec3(glm::cos(phi) * sinTheta, glm::sin(phi) * sinTheta, cosTheta);
        // Transform direction to world space
        return GetTangentSpace(normal) * tangentSpaceDir;
    }

    static __forceinline__ __device__ glm::vec3
    RandomSampleHemisphere(Random &random, const glm::vec3 &normal) {
        // Uniformly sample hemisphere direction
        auto cosTheta = random();
        const auto sinTheta = sqrt(glm::max(0.0f, 1.0f - cosTheta * cosTheta));
        const auto phi = 2.0f * glm::pi<float>() * random();
        const auto tangentSpaceDir =
                glm::vec3(glm::cos(phi) * sinTheta, glm::sin(phi) * sinTheta, cosTheta);
        // Transform direction to world space
        return GetTangentSpace(normal) * tangentSpaceDir;
    }

    static __forceinline__ __device__ glm::vec3 RandomSampleSphere(Random &random) {
        const float theta = 2 * glm::pi<float>() * random();
        const float phi = glm::acos(1.0f - 2.0f * random());
        return glm::vec3(glm::sin(phi) * glm::cos(theta),
                         glm::sin(phi) * glm::sin(theta), glm::cos(phi));
    }

    static __forceinline__ __device__ glm::vec2 RandomSampleDisk(Random &random) {
        const float theta = 2 * glm::pi<float>() * random();
        return glm::vec2(glm::cos(theta), glm::sin(theta));
    }

#pragma endregion

} // namespace EvoEngine
