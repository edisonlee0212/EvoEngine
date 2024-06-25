#pragma once

#include "CIELab.cuh"
#include "IndexAB.cuh"
#include "Optix7.hpp"
#include "PDF1D.cuh"
#include <PDF2D.cuh>
#include "SharedCoordinates.cuh"

/**
 * \brief Since CUDA have poor support for virtual class functions, I used this
 * instead.
 */
#define PDF2D PDF2DSeperate
namespace evo_engine {
    struct PDF2DSeperate {
        // the used number of 2D functions
        int m_numOfPdf2D;
        // the size of the data entry to be used here during restoration
        int m_size2D;
        // length of index slice, should be 2 here.
        int m_lengthOfSlice;
        IndexAB m_iab;
        PDF1D m_pdf1;

        struct PDF2DColor {
            // the used number of 2D functions
            int m_numOfPdf2D;
            // length of index slice
            int m_numOfAlpha;
            // the size of the data entry to be used here during restoration
            int m_size2D;
            // the database of indices of color 1D functions

            void Init(const int &lengthOfSlice, const int &slicesPerHemisphere) {
                m_numOfAlpha = lengthOfSlice;
                m_numOfPdf2D = 0;
                m_size2D = slicesPerHemisphere * lengthOfSlice;
            }


#pragma region CUDA
            // Here are the indices to CIndexAB class
            CudaBuffer m_pdf2DColorsBuffer;
            int *m_devicePdf2DColors;
            __device__ void GetVal(const int &pdf2DIndex, glm::vec3 &out,
                                   SharedCoordinates &tc, const IndexAB &iab,
                                   const bool &print) const {
                const int i = tc.m_currentAlphaLowBound;
                assert(i >= 0 && i < m_numOfAlpha - 1);
                assert(pdf2DIndex >= 0 && pdf2DIndex < m_numOfPdf2D);
                const float w = tc.m_weightAlpha;
                glm::vec3 ab1, ab2;
                // colors
                iab.GetVal(m_devicePdf2DColors[pdf2DIndex * m_numOfAlpha + i], ab1, tc);
                iab.GetVal(m_devicePdf2DColors[pdf2DIndex * m_numOfAlpha + i + 1], ab2, tc);
                out[1] = ab1[0] * (1.0f - w) + ab2[0] * w;
                out[2] = ab1[1] * (1.0f - w) + ab2[1] * w;
            }

#pragma endregion
        };

        struct PDF2DLuminance {
            // the used number of 2D functions
            int m_numOfPdf2D;
            // length of index slice
            int m_numOfAlpha;

            void Init(const int &lengthOfSlice, const int &slicesPerHemisphere) {
                m_numOfAlpha = lengthOfSlice;
                m_numOfPdf2D = 0;
            }


#pragma region CUDA
            // the database of 1D functions over luminance
            // Here are the indices to PDF1D class
            CudaBuffer m_pdf2DSlicesBuffer;
            int *m_devicePdf2DSlices;
            // Here are the scale to PDF1D class, PDF1 functions are multiplied by that
            CudaBuffer m_pdf2DScalesBuffer;
            float *m_devicePdf2DScales;
            // This is optional, not required for rendering, except importance sampling
            // float* m_pdf2DNorm;
            __device__ void GetVal(const int &pdf2DIndex, glm::vec3 &out,
                                   SharedCoordinates &tc, const PDF1D &pdf1,
                                   const bool &print) const {
                assert(pdf2DIndex >= 0 && pdf2DIndex < m_numOfPdf2D);
                const int i = tc.m_currentAlphaLowBound;
                const float w = tc.m_weightAlpha;
                assert(i >= 0 && i < m_numOfAlpha - 1);
                // This is different to compact representation ! we interpolate in
                // luminances
                const float l1 =
                        m_devicePdf2DScales[pdf2DIndex * m_numOfAlpha + i] *
                        pdf1.GetVal(m_devicePdf2DSlices[pdf2DIndex * m_numOfAlpha + i], tc);
                const float l2 =
                        m_devicePdf2DScales[pdf2DIndex * m_numOfAlpha + i + 1] *
                        pdf1.GetVal(m_devicePdf2DSlices[pdf2DIndex * m_numOfAlpha + i + 1], tc);
                out[0] = (1.f - w) * l1 + w * l2;
            }

#pragma endregion
        };

        // Here are the instances of color and luminance 2D function database
        PDF2DColor m_color;
        PDF2DLuminance m_luminance;


        void Init(const int &slicesPerHemisphere) {
            m_size2D = slicesPerHemisphere * m_pdf1.m_numOfBeta;
            m_color.Init(m_pdf1.m_numOfBeta, slicesPerHemisphere);
            m_luminance.Init(m_pdf1.m_numOfBeta, slicesPerHemisphere);
        }

#pragma region CUDA
        // Here are the indices of luminances + color 2D functions
        // index [][0] is luminance, index [][1] is color
        CudaBuffer m_indexLuminanceColorBuffer;
        int *m_deviceIndexLuminanceColor;
        __device__ void GetVal(const int &pdf2DIndex, glm::vec3 &out,
                               SharedCoordinates &tc, const bool &print) const {
            assert(pdf2DIndex >= 0 && pdf2DIndex < m_numOfPdf2D);

            glm::vec3 userCMdata;
            // First, get only luminance
            out = glm::vec3(1.0f);
            if (print)
                printf("Sampling from Color...");
            m_color.GetVal(m_deviceIndexLuminanceColor[pdf2DIndex * m_lengthOfSlice + 1],
                           userCMdata, tc, m_iab, print);
            if (print)
                printf("Sampling from Luminance...");
            m_luminance.GetVal(m_deviceIndexLuminanceColor[pdf2DIndex * m_lengthOfSlice + 0],
                               userCMdata, tc, m_pdf1, print);

            // Convert to RGB
            UserCmToRgb(userCMdata, out, tc);
        }
#pragma endregion
    };

} // namespace EvoEngine
