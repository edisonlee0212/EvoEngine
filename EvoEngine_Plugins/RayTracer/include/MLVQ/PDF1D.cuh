#pragma once

#include "SharedCoordinates.cuh"

#define HERMITE_INTERPOLANT
namespace evo_engine {
    struct PDF1D {
        // the number of values for 1D function
        int m_numOfBeta;
        // current number of stored 1D functions
        int m_numOfPdf1D;
        // The shared coordinates to be used for interpolation
        // when retrieving the data from the database
        void Init(const int &lengthOfSlice) {
            assert(lengthOfSlice > 0);
            m_numOfBeta = lengthOfSlice;
            m_numOfPdf1D = 0;
        }
#pragma region CUDA
        // the data array of 1D functions. These are normalized !
        CudaBuffer m_pdf1DBasisBuffer;
        float *m_devicePdf1DBasis;
        __device__ float GetVal(const int &sliceIndex, SharedCoordinates &tc) const {
            assert(sliceIndex >= 0 && sliceIndex < m_numOfPdf1D);
            assert(tc.m_currentBetaLowBound >= 0 &&
                   tc.m_currentBetaLowBound < m_numOfBeta);
#ifdef LINEAR_INTERPOLANT
            // This implements simple linear interpolation between two values
            return (1.f - tc.wBeta) * PDF1Dbasis[sliceIndex][tc.iBeta] +
                   tc.wBeta * PDF1Dbasis[sliceIndex][tc.iBeta + 1];
#endif

#ifdef HERMITE_INTERPOLANT
            // This implements Fergusson cubic interpolation based on Cubic Hermite
            // Splines
            const float w = tc.m_weightBeta;
            const float p0 =
                    m_devicePdf1DBasis[sliceIndex * m_numOfBeta + tc.m_currentBetaLowBound];
            const float p1 =
                    m_devicePdf1DBasis[sliceIndex * m_numOfBeta + tc.m_currentBetaLowBound + 1];
            float m0h, m1h;
            if (tc.m_currentBetaLowBound == 0)
                m0h = p1 - p0; // end point
            else
                // standard way
                m0h = 0.5f * (p1 - m_devicePdf1DBasis[sliceIndex * m_numOfBeta +
                                                      tc.m_currentBetaLowBound - 1]);

            assert(tc.m_currentBetaLowBound < m_numOfBeta - 1);
            if (tc.m_currentBetaLowBound == m_numOfBeta - 2)
                m1h = p1 - p0; // end point
            else
                // standard way
                m1h = 0.5f * (m_devicePdf1DBasis[sliceIndex * m_numOfBeta +
                                                 tc.m_currentBetaLowBound + 1] -
                              p0);
            const float t2 = w * w;
            const float t3 = t2 * w;
            const float h01 = -2.0f * t3 + 3.0f * t2;
            const float h00 = 1.0f - h01;
            const float h11 = t3 - t2;
            const float h10 = h11 - t2 + w;

            // This implements the whole formula
            const float res = h00 * p0 + h10 * m0h + h01 * p1 + h11 * m1h;
            return res;
#endif
        }

#pragma endregion
    };
} // namespace EvoEngine