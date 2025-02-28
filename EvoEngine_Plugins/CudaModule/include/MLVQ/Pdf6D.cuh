#pragma once

#include "Pdf4D.cuh"

namespace evo_engine {
template <typename T>
struct Pdf6D {
  /**
   * \brief Number of rows in spatial BTF index
   */
  int row_size;
  /**
   * \brief Number of columns in spatial BTF index
   */
  int column_size;
  /**
   * \brief Offset of the first row as we do not need to start from 0
   */
  int row_offset;
  /**
   * \brief Offset of the first column as we do not need to start from 0
   */
  int column_offset;
  /**
   * \brief Number of colors
   */
  int color_size;
  /**
   * \brief The database of 4D functions to which we point in the array PDF6Dslices
   */
  Pdf4D<T> pdf4d;

  void Init(const int num_of_rows, const int num_of_columns, const int row_offset, const int column_offset,
            const int num_of_colors) {
    row_size = num_of_rows;
    column_size = num_of_columns;
    this->row_offset = row_offset;
    this->column_offset = column_offset;
    color_size = num_of_colors;
  }
#pragma region CUDA
  CudaBuffer pdf6d_buffer;
  int *pdf6d_device_ptr;  //! planar index pointing on 4D PDF for individual pixels

  CudaBuffer pdf6d_scale_buffer;
  float *pdf6d_scale_device_ptr;  //! corresponding normalization values
  __device__ void GetValDeg2(const glm::vec2 &tex_coord, const float illumination_theta, float illumination_phi,
                             const float view_theta, float view_phi, T &out, SharedCoordinates &tc) const {
    int x = static_cast<int>(tex_coord.x * static_cast<float>(column_size));
    int y = static_cast<int>(tex_coord.y * static_cast<float>(row_size));

    x -= column_offset;
    while (x < 0)
      x += column_size;
    y -= row_offset;
    while (y < 0)
      y += row_size;
    x %= column_size;
    y %= row_size;

    // recompute from clockwise to anti-clockwise phi_i notation
    view_phi = glm::mod(360.0f - view_phi, 360.0f);
    illumination_phi = glm::mod((360.0f - illumination_phi) - (90.0f + view_phi), 360.0f);

    ConvertThetaPhiToBetaAlpha(glm::radians(illumination_theta), glm::radians(illumination_phi), tc.beta, tc.alpha, tc);

    // Back to degrees. Set the values to auxiliary structure
    tc.alpha = glm::degrees(tc.alpha);
    tc.beta = glm::degrees(tc.beta);
    if (glm::isnan(tc.beta) || glm::isnan(tc.alpha) || glm::isnan(view_theta) || glm::isnan(view_phi)) {
      return;
    }
    // Now we set the object interpolation data
    // For Pdf1D and IndexAB, beta coefficient, use correct
    // parameterization
    tc.SetForAngleBetaDeg(glm::clamp(tc.beta, -90.0f, 90.0f));
    // For Pdf2D
    tc.SetForAngleAlphaDeg(glm::clamp(tc.alpha, -90.0f, 90.0f));
    // For Pdf3D
    tc.SetForAngleThetaDeg(glm::clamp(view_theta, 0.0f, 90.0f));
    // For Pdf4D
    tc.SetForAnglePhiDeg(glm::clamp(view_phi, 0.0f, 360.0f));

    // Now get the value by interpolation between 2 Pdf4D, 4 Pdf3D,
    // 8 Pdf2D, 16 Pdf1D, and 16 IndexAB values for precomputed
    // interpolation coefficients and indices

    assert(y >= 0 && y < row_size);
    assert(x >= 0 && x < column_size);

    pdf4d.GetVal(pdf6d_device_ptr[y * column_size + x] - 1, out, tc);
    // we have to multiply it by valid scale factor at the end
    const float scale = pdf6d_scale_device_ptr[y * column_size + x];
    out *= scale;
  }
#pragma endregion
};
}  // namespace evo_engine
