#pragma once

#include "Optix7.hpp"
#include "PDF2D.cuh"
#include "SharedCoordinates.cuh"

namespace evo_engine {
    template<typename T>
    struct PDF3D {
        // the used number of 3D functions
        int m_numOfPdf3D;
        // the number of slices per theta (=2D functions) to represent one 3D function
        int m_numOfTheta;
        // the size of the data entry to be used here during restoration

        // the database of 2D functions to which we point in the array PDF3Dslices
        PDF2D m_pdf2;

        void Init(const int &numOfTheta) {
            m_numOfTheta = numOfTheta;
            m_numOfPdf3D = 0;
        }

#pragma region CUDA
        // These are the data allocated maxPDF2D times, serving to represent the
        // function
        CudaBuffer m_pdf3DSlicesBuffer;
        int *m_devicePdf3DSlices;
        CudaBuffer m_pdf3DScalesBuffer;
        float *m_devicePdf3DScales;
        __device__ void GetVal(const int &pdf3DIndex, T &out, SharedCoordinates &tc,
                               const bool &print) const {
            const int i = tc.m_currentThetaLowBound;
            assert(i >= 0 && i < m_numOfTheta - 1);
            assert(pdf3DIndex >= 0 && pdf3DIndex < m_numOfPdf3D);
            if (print)
                printf("Sampling from PDF2...");
            const float w = tc.m_weightTheta;
            glm::vec3 out2;
            m_pdf2.GetVal(m_devicePdf3DSlices[pdf3DIndex * m_numOfTheta + i], out, tc, print);
            m_pdf2.GetVal(m_devicePdf3DSlices[pdf3DIndex * m_numOfTheta + i + 1], out2, tc,
                          print);
            const float s1 = m_devicePdf3DScales[pdf3DIndex * m_numOfTheta + i] * (1.0f - w);
            const float s2 = m_devicePdf3DScales[pdf3DIndex * m_numOfTheta + i + 1] * w;
            out = out * s1 + out2 * s2;
        }

#pragma endregion
    };
} // namespace EvoEngine