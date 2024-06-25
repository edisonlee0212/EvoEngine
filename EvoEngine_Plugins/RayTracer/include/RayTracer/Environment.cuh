#pragma once

#include "RayTracerUtilities.cuh"

namespace evo_engine {
#pragma region Sky illuminance

    static __forceinline__ __device__ float
    CIESkyIntensity(glm::vec3 rayDir, const glm::vec3 &sunDir, const glm::vec3 &zenith) {
        if (rayDir.y <= 0) {
            rayDir = glm::normalize(glm::vec3(rayDir.x, 0.01f, rayDir.z));
        } else {
            rayDir = glm::normalize(rayDir);
        }
        const float gamma = glm::angle(sunDir, rayDir);
        const float cosGamma = glm::cos(gamma);
        const float cos2Gamma = cosGamma * cosGamma;
        const float theta = glm::angle(zenith, rayDir);
        const float cosTheta = glm::cos(theta);
        const float z0 = glm::angle(zenith, sunDir);
        const float cosz0 = glm::cos(z0);
        const float cos2z0 = cosz0 * cosz0;
        return (0.91f + 10.0f * glm::pow(2.7182818f, -3.0f * gamma) + 0.45f * cos2Gamma)
               * (1.0f - glm::pow(2.7182818f, -0.32f / cosTheta))
               / 0.27f / (0.91f + 10.0f * glm::pow(2.7182818f, -3.0f * z0) + 0.45f * cos2z0);
    }

    static __forceinline__ __device__ bool
    SolveQuadratic(float a, float b, float c, float &x1, float &x2) {
        if (b == 0) {
            // Handle special case where the the two vector ray.dir and V are perpendicular
            // with V = ray.orig - sphere.centre
            if (a == 0) return false;
            x1 = 0;
            x2 = glm::sqrt(-c / a);
            return true;
        }
        float discr = b * b - 4 * a * c;

        if (discr < 0) return false;

        float q = (b < 0.f) ? -0.5f * (b - glm::sqrt(discr)) : -0.5f * (b + glm::sqrt(discr));
        x1 = q / a;
        x2 = c / q;

        return true;
    }

    static __forceinline__ __device__ bool
    RaySphereIntersect(const glm::vec3 &orig, const glm::vec3 &dir, const float &radius, float &t0, float &t1) {
        // They ray dir is normalized so A = 1
        float A = dir.x * dir.x + dir.y * dir.y + dir.z * dir.z;
        float B = 2.0f * (dir.x * orig.x + dir.y * orig.y + dir.z * orig.z);
        float C = orig.x * orig.x + orig.y * orig.y + orig.z * orig.z - radius * radius;

        if (!SolveQuadratic(A, B, C, t0, t1)) return false;

        if (t0 > t1) {
            float temp = t0;
            t0 = t1;
            t1 = temp;
        }

        return true;
    }


    /**
     * From https://www.scratchapixel.com/lessons/procedural-generation-virtual-worlds/simulating-sky/simulating-colors-of-the-sky
     * @param position
     * @param rayDir
     * @param environment
     * @return
     */
    static __forceinline__ __device__ glm::vec3
    NishitaSkyIncidentLight(const glm::vec3 &position, const glm::vec3 &rayDir,
                            const const EnvironmentProperties &environment) {
        float earthRadius = environment.m_atmosphere.m_earthRadius *
                            1000.0f;      // In the paper this is usually Rg or Re (radius ground, eart)
        float atmosphereRadius = environment.m_atmosphere.m_atmosphereRadius *
                                 1000.0f; // In the paper this is usually R or Ra (radius atmosphere)
        float Hr = environment.m_atmosphere.m_Hr;               // Thickness of the atmosphere if density was uniform (Hr)
        float Hm = environment.m_atmosphere.m_Hm;               // Same as above but for Mie scattering (Hm)

        glm::vec3 betaR = glm::vec3(3.8e-6f, 13.5e-6f, 33.1e-6f);
        glm::vec3 betaM = glm::vec3(21e-6f);
        float tmin = 0;
        float tmax = 999999999999;
        glm::vec3 orig = position + glm::vec3(0.0f, earthRadius, 0.0f);
        float t0, t1;
        if (!RaySphereIntersect(orig, rayDir, atmosphereRadius, t0, t1) || t1 < 0.0f)
            return glm::vec3(0.0f, 0.0f, 0.0f);
        if (t0 > tmin && t0 > 0.0f) tmin = t0;
        if (t1 < tmax) tmax = t1;
        unsigned numSamples = environment.m_atmosphere.m_numSamples;
        unsigned numSamplesLight = environment.m_atmosphere.m_numSamplesLight;
        float segmentLength = (tmax - tmin) / numSamples;
        float tCurrent = tmin;
        glm::vec3 sumR = glm::vec3(0.0f);
        glm::vec3 sumM = glm::vec3(0.0f); // mie and rayleigh contribution
        float opticalDepthR = 0, opticalDepthM = 0;
        float mu = glm::dot(rayDir,
                            environment.m_sunDirection); // mu in the paper which is the cosine of the angle between the sun direction and the ray direction
        float phaseR = 3.f / (16.f * 3.1415926f) * (1.0f + mu * mu);
        float g = environment.m_atmosphere.m_g;
        float phaseM = 3.f / (8.f * 3.1415926f) * ((1.f - g * g) * (1.f + mu * mu)) /
                       ((2.f + g * g) * glm::pow(1.f + g * g - 2.f * g * mu, 1.5f));
        for (unsigned i = 0; i < numSamples; ++i) {
            glm::vec3 samplePosition = orig + (tCurrent + segmentLength * 0.5f) * rayDir;
            float height = glm::length(samplePosition) - earthRadius;
            // compute optical depth for light
            float hr = glm::exp(-height / Hr) * segmentLength;
            float hm = glm::exp(-height / Hm) * segmentLength;
            opticalDepthR += hr;
            opticalDepthM += hm;
            // light optical depth
            float t0Light, t1Light;
            RaySphereIntersect(samplePosition, environment.m_sunDirection, atmosphereRadius, t0Light, t1Light);
            float segmentLengthLight = t1Light / numSamplesLight, tCurrentLight = 0;
            float opticalDepthLightR = 0, opticalDepthLightM = 0;
            unsigned j;
            for (j = 0; j < numSamplesLight; ++j) {
                glm::vec3 samplePositionLight =
                        samplePosition + (tCurrentLight + segmentLengthLight * 0.5f) * environment.m_sunDirection;
                float heightLight = glm::length(samplePositionLight) - earthRadius;
                if (heightLight < 0) break;
                opticalDepthLightR += glm::exp(-heightLight / Hr) * segmentLengthLight;
                opticalDepthLightM += glm::exp(-heightLight / Hm) * segmentLengthLight;
                tCurrentLight += segmentLengthLight;
            }
            if (j == numSamplesLight) {
                glm::vec3 tau = betaR * (opticalDepthR + opticalDepthLightR) +
                                betaM * 1.1f * (opticalDepthM + opticalDepthLightM);
                glm::vec3 attenuation(glm::exp(-tau.x), glm::exp(-tau.y), glm::exp(-tau.z));
                sumR += attenuation * hr;
                sumM += attenuation * hm;
            }
            tCurrent += segmentLength;
        }
        // We use a magic number here for the intensity of the sun (20). We will make it more
        // scientific in a future revision of this lesson/code
        glm::vec3 result = (glm::vec3(sumR.x * betaR.x, sumR.y * betaR.y, sumR.z * betaR.z) * phaseR +
                            glm::vec3(sumM.x * betaM.x, sumM.y * betaM.y, sumM.z * betaM.z) * phaseM) * 20.0f;
        return result;
    }

    static __forceinline__ __device__ glm::vec3
    CalculateEnvironmentalLight(const glm::vec3 &position, const glm::vec3 &rayDir,
                                const EnvironmentProperties &environment) {
        glm::vec3 environmentalLightColor = glm::vec3(1.0f);
        switch (environment.m_environmentalLightingType) {
            case EnvironmentalLightingType::Scene:
                if (environment.m_useEnvironmentalMap && environment.m_environmentalMap) {
                    float4 color = texCubemap<float4>(
                            environment.m_environmentalMap,
                            rayDir.x, rayDir.y, rayDir.z);
                    environmentalLightColor = glm::vec3(color.x, color.y, color.z);
                } else {
                    environmentalLightColor = environment.m_color;
                }
                environmentalLightColor *= environment.m_skylightIntensity;
                break;
            case EnvironmentalLightingType::Skydome:
                environmentalLightColor = NishitaSkyIncidentLight(position, rayDir,
                                                                  environment);
                environmentalLightColor *= environment.m_skylightIntensity;
                break;
            case EnvironmentalLightingType::SingleLightSource:
                environmentalLightColor = glm::vec3(environment.m_color * environment.m_skylightIntensity);
                break;
        }
        environmentalLightColor = pow(environmentalLightColor,
                                      glm::vec3(1.0f / environment.m_gamma));
        return glm::max(glm::vec3(0.0f), environmentalLightColor);
    }

#pragma endregion
}