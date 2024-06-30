#pragma once

#include "Optix7.hpp"
#include "PDF3D.cuh"
#include "SharedCoordinates.cuh"
#include "glm/glm.hpp"

namespace evo_engine {
    template<typename T>
    struct PDF4D {
        // the used number of 4D functions
        int m_numOfPdf4D;
        // the number of slices per phi (=3D functions) to represent one 4D function
        int m_numOfPhi;
        // angle phi quantization step
        float m_stepPhi;

        PDF3D<T> m_pdf3;

        void Init(const int &numOfPhi) {
            m_numOfPhi = numOfPhi;
            m_stepPhi = 360.0f / numOfPhi;
            m_numOfPdf4D = 0;
        }

#pragma region CUDA
        // These are the data allocated maxPDF4D times, serving to represent the
        // function
        CudaBuffer m_pdf4DSlicesBuffer;
        int *m_devicePdf4DSlices;
        CudaBuffer m_pdf4DScalesBuffer;
        float *m_devicePdf4DScales;
        __device__ void GetVal(const int &pdf4DIndex, T &out, SharedCoordinates &tc,
                               const bool &print) const {
            const int lowPhi = tc.m_currentPhiLowBound;
            const float w = tc.m_weightPhi;
            assert(lowPhi >= 0 && lowPhi < m_numOfPhi);
            assert(pdf4DIndex >= 0 && pdf4DIndex < m_numOfPdf4D);
            if (print)
                printf("Sampling from PDF3...");
            if (lowPhi != m_numOfPhi - 1) {
                glm::vec3 out2;
                m_pdf3.GetVal(m_devicePdf4DSlices[pdf4DIndex * m_numOfPhi + lowPhi], out, tc,
                              print);
                m_pdf3.GetVal(m_devicePdf4DSlices[pdf4DIndex * m_numOfPhi + lowPhi + 1], out2,
                              tc, print);
                const float s1 =
                        m_devicePdf4DScales[pdf4DIndex * m_numOfPhi + lowPhi] * (1.0f - w);
                const float s2 = m_devicePdf4DScales[pdf4DIndex * m_numOfPhi + lowPhi + 1] * w;
                out = out * s1 + out2 * s2;
            } else {
                glm::vec3 out2;
                m_pdf3.GetVal(m_devicePdf4DSlices[pdf4DIndex * m_numOfPhi + lowPhi], out, tc,
                              print);
                m_pdf3.GetVal(m_devicePdf4DSlices[pdf4DIndex * m_numOfPhi], out2, tc, print);
                const float s1 =
                        m_devicePdf4DScales[pdf4DIndex * m_numOfPhi + lowPhi] * (1.0f - w);
                const float s2 = m_devicePdf4DScales[pdf4DIndex * m_numOfPhi] * w;
                out = out * s1 + out2 * s2;
            }
            if (print)
                printf("Col3[%.2f, %.2f, %.2f]\n", out.x, out.y, out.z);
        }
#pragma endregion
    };
} // namespace EvoEngine
