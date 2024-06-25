#pragma once
#include "Optix7.hpp"
#include "PDF4D.cuh"
#include "SharedCoordinates.cuh"

namespace evo_engine {
template <typename T> struct PDF6D {
  int m_numOfRows;   //! no. of rows in spatial BTF index
  int m_numOfCols;   //! no. of columns in spatial BTF index
  int m_rowsOffset;  //! offset of the first row as we do not need to start from
                     //! 0
  int m_colsOffset;  //! offset of the first column as we do not need to start
                     //! from 0
  int m_colorAmount; // the number of colors

  // the database of 4D functions to which we point in the array PDF6Dslices
  PDF4D<T> m_pdf4;

  void Init(const int &numOfRows, const int &numOfCols, const int &rowsOffset,
            const int &colsOffset, const int &colorAmount) {
    m_numOfRows = numOfRows;
    m_numOfCols = numOfCols;
    m_rowsOffset = rowsOffset;
    m_colsOffset = colsOffset;
    m_colorAmount = colorAmount;
  }
#pragma region CUDA
    CudaBuffer m_pdf6DSlicesBuffer;
    CudaBuffer m_pdf6DScalesBuffer;
    int *m_devicePdf6DSlices;  //! planar index pointing on 4D PDF for individual pixels
    float *m_devicePdf6DScales; //! corresponding normalization values
    __device__ void GetValDeg2(const glm::vec2 &texCoord, float illuminationTheta,
                               float illuminationPhi, float viewTheta,
                               float viewPhi, T &out, SharedCoordinates &tc,
                               const bool &print) const {
        int x = texCoord.x * m_numOfCols;
        int y = texCoord.y * m_numOfRows;

        x -= m_colsOffset;
        while (x < 0)
            x += m_numOfCols;
        y -= m_rowsOffset;
        while (y < 0)
            y += m_numOfRows;
        x %= m_numOfCols;
        y %= m_numOfRows;

        // recompute from clockwise to anti-clockwise phi_i notation
        viewPhi = glm::mod(360.0f - viewPhi, 360.0f);
        illuminationPhi =
                glm::mod((360.0f - illuminationPhi) - (90.0f + viewPhi), 360.0f);

        ConvertThetaPhiToBetaAlpha(glm::radians(illuminationTheta),
                                   glm::radians(illuminationPhi), tc.m_beta,
                                   tc.m_alpha, tc);

        // Back to degrees. Set the values to auxiliary structure
        tc.m_alpha = glm::degrees(tc.m_alpha);
        tc.m_beta = glm::degrees(tc.m_beta);
        if (glm::isnan(tc.m_beta) || glm::isnan(tc.m_alpha) ||
            glm::isnan(viewTheta) || glm::isnan(viewPhi)) {
            if (print)
                printf("Value is nan! beta[%.2f] alpha[%.2f] theta[%.2f] phi[%.2f]",
                       tc.m_beta, tc.m_alpha, viewTheta, viewPhi);
            return;
        }
        assert(!glm::isnan(tc.m_beta) && !glm::isnan(tc.m_alpha) &&
               !glm::isnan(viewTheta) && !glm::isnan(viewPhi));
        // Now we set the object interpolation data
        // For PDF1D and IndexAB, beta coefficient, use correct
        // parameterization
        tc.SetForAngleBetaDeg(glm::clamp(tc.m_beta, -90.0f, 90.0f));
        // For PDF2D
        tc.SetForAngleAlphaDeg(glm::clamp(tc.m_alpha, -90.0f, 90.0f));
        // For PDF3D
        tc.SetForAngleThetaDeg(glm::clamp(viewTheta, 0.0f, 90.0f));
        // For PDF4D
        tc.SetForAnglePhiDeg(glm::clamp(viewPhi, 0.0f, 360.0f));

        // Now get the value by interpolation between 2 PDF4D, 4 PDF3D,
        // 8 PDF2D, 16 PDF1D, and 16 IndexAB values for precomputed
        // interpolation coefficients and indices

        assert(y >= 0 && y < m_numOfRows);
        assert(x >= 0 && x < m_numOfCols);

        if (print)
            printf("Sampling from PDF4...");
        m_pdf4.GetVal(m_devicePdf6DSlices[y * m_numOfCols + x] - 1, out, tc, print);
        if (print)
            printf("Col4[%.2f, %.2f, %.2f]\n", out.x, out.y, out.z);
        // we have to multiply it by valid scale factor at the end
        const float scale = m_devicePdf6DScales[y * m_numOfCols + x];
        out *= scale;
    }
#pragma endregion
};
} // namespace EvoEngine
