#pragma once

#include "CUDABuffer.hpp"
#include "glm/glm.hpp"
namespace evo_engine {
    struct SharedCoordinates {
        // false ... use uniform distribution in Beta
        // true ... use uniform distribution in cos(Beta)
        bool m_useCosBeta;

        int m_numOfBeta;
        float m_stepAlpha;
        int m_numOfAlpha;
        float m_stepTheta;
        int m_numOfTheta;
        float m_stepPhi;
        int m_numOfPhi;

        // the BTF single point coordinates in degrees
        float m_beta;  // 1D
        float m_alpha; // 2D
        float m_theta; // 3D
        float m_phi;   // 4D

        // interpolation values for PDF1D
        int m_currentBetaLowBound;
        float m_weightBeta;
        float m_wMinBeta2;

        // interpolation values for PDF2D
        int m_currentAlphaLowBound;
        float m_weightAlpha;
        float m_wMinAlpha2;

        // interpolation values for PDF3D
        int m_currentThetaLowBound;
        float m_weightTheta;
        float m_wMinTheta2;

        // interpolation values for PDF4D
        int m_currentPhiLowBound;
        float m_weightPhi;

        float m_scale;

        bool m_hdrFlag;
        bool m_codeBtfFlag;
#pragma region CUDA
        // the values to be used for interpolation in beta coordinate
        CudaBuffer m_betaAnglesBuffer;
        float *m_deviceBetaAngles; // the sequence of values used
        // Here we set the structure for particular angle beta
        __device__ void SetForAngleBetaDeg(const float &beta) {
            assert(beta > -90.001f && beta < 90.001f);
            m_beta = beta;
            if (m_useCosBeta) {
                m_currentBetaLowBound =
                        glm::clamp(static_cast<int>((glm::sin(glm::radians(m_beta)) + 1.0f) /
                                                    2.0f * (m_numOfBeta - 1)),
                                   0, m_numOfBeta - 2);
                m_weightBeta = (m_beta - m_deviceBetaAngles[m_currentBetaLowBound]) /
                               (m_deviceBetaAngles[m_currentBetaLowBound + 1] -
                                m_deviceBetaAngles[m_currentBetaLowBound]);
                assert(m_weightBeta > -0.001f && m_weightBeta < 1.001f);
            } else {
                // The angles are quantized uniformly in degrees
                const float stepBeta = 180.0f / static_cast<float>(m_numOfBeta - 1);
                m_currentBetaLowBound = glm::clamp(
                        static_cast<int>((beta + 90.0f) / stepBeta), 0, m_numOfBeta - 2);
                m_weightBeta =
                        (beta + 90.0f - m_currentBetaLowBound * stepBeta) / stepBeta;
                assert(m_weightBeta > -0.001f && m_weightBeta < 1.001f);
            }
        }

        // Here we set the structure for particular angle alpha
        __device__ void SetForAngleAlphaDeg(const float &alpha) {
            assert(alpha > -90.001f && alpha < 90.001f);
            m_alpha = alpha;
            m_currentAlphaLowBound = glm::clamp(
                    static_cast<int>((m_alpha + 90.0f) / m_stepAlpha), 0, m_numOfAlpha - 2);
            m_weightAlpha =
                    (m_alpha + 90.f - m_currentAlphaLowBound * m_stepAlpha) / m_stepAlpha;
            assert(m_weightAlpha > -0.001f && m_weightAlpha < 1.001f);
        }

        // Here we set the structure for particular angle alpha
        __device__ void SetForAnglePhiDeg(const float &phi) {
            assert(phi > -0.001f && phi < 360.001f);
            m_phi = phi;
            m_currentPhiLowBound =
                    glm::clamp(static_cast<int>(m_phi / m_stepPhi), 0, m_numOfPhi - 1);
            m_weightPhi = (m_phi - m_currentPhiLowBound * m_stepPhi) / m_stepPhi;
            assert(m_weightPhi > -0.001f && m_weightPhi < 1.001f);
        }

        // Here we set the structure for particular angle alpha
        __device__ void SetForAngleThetaDeg(const float &theta) {
            assert(theta > -0.001f && theta < 90.001f);
            m_theta = theta;
            m_currentThetaLowBound = glm::clamp(static_cast<int>(m_theta / m_stepTheta),
                                                0, m_numOfTheta - 2);
            m_weightTheta =
                    (m_theta - m_currentThetaLowBound * m_stepTheta) / m_stepTheta;
            assert(m_weightTheta > -0.001f && m_weightTheta < 1.001f);
        }

#pragma endregion
        void Set(const bool &useBtfFlag, const bool &useCosBeta,
                          const int &numOfBeta,
                          const int &numOfAlpha, const float &stepAlpha,
                          const int &numOfTheta, const float &stepTheta,
                          const int &numOfPhi, const float &stepPhi) {
            m_codeBtfFlag = useBtfFlag;
            m_useCosBeta = useCosBeta;
            m_numOfBeta = numOfBeta;
            m_numOfAlpha = numOfAlpha;
            m_stepAlpha = stepAlpha;
            m_numOfTheta = numOfTheta;
            m_stepTheta = stepTheta;
            m_stepPhi = stepPhi;
            m_numOfPhi = numOfPhi;
            m_hdrFlag = false;
        }
    };

    __device__ inline void ConvertThetaPhiToBetaAlpha(const float theta,
                                                      const float phi, float &beta,
                                                      float &alpha,
                                                      const SharedCoordinates &tc) {
        if (tc.m_codeBtfFlag) {
            const float x = cos(phi - tc.m_phi) * sin(theta);
            const float y = sin(phi - tc.m_phi) * sin(theta);
            // float z = cos(thetaI);

            beta = asin(glm::clamp(y, -1.0f, 1.0f));
            const float cosBeta = cos(beta);

            if (cosBeta < 0.001f) {
                alpha = 0.0f;
                return;
            }
            const float tmp = glm::clamp(-x / cosBeta, -1.0f, 1.0f);
            alpha = asin(tmp);
            return;
        }

        // This is three dimensional vector
        glm::vec3 xyz;
        // Here we convert the angles to 3D vector
        xyz[0] = glm::cos(phi) * glm::sin(theta);
        xyz[1] = glm::sin(phi) * glm::sin(theta);
        xyz[2] = glm::cos(theta);

        // Here we convert 3D vector to alpha-beta parametrization over hemisphere
        beta = glm::asin(xyz[0]);
        const float cosBeta = glm::cos(beta);
        if (cosBeta < 0.001f) {
            alpha = 0.0f;
            return;
        }
        const float aux = glm::clamp(xyz[1] / cosBeta, -1.0f, 1.0f);
        alpha = glm::asin(aux);
    }
} // namespace EvoEngine