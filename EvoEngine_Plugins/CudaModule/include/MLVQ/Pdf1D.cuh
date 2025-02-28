#pragma once

#include "SharedCoordinates.cuh"

#define HERMITE_INTERPOLANT
namespace evo_engine {
struct Pdf1D {
  /**
   * \brief The number of values for 1D function
   */
  int beta_size;
  /**
   * \brief Current number of stored 1D functions
   */
  int pdf1d_size;
  /**
   * \brief The shared coordinates to be used for interpolation when retrieving the data from the database
   */
  void Init(const int num_of_beta) {
    assert(num_of_beta > 0);
    beta_size = num_of_beta;
    pdf1d_size = 0;
  }
#pragma region CUDA
  /**
   * \brief The data array of 1D functions. These are normalized
   */
  CudaBuffer pdf1d_buffer;
  float *pdf1d_device_ptr;

  __device__ [[nodiscard]] float GetVal(const int slice, const SharedCoordinates &tc) const {
    assert(slice >= 0 && slice < pdf1d_size);
    assert(tc.current_beta_low_bound >= 0 && tc.current_beta_low_bound < beta_size);
#ifdef LINEAR_INTERPOLANT
    // This implements simple linear interpolation between two values
    return (1.f - tc.wBeta) * PDF1Dbasis[slice][tc.iBeta] + tc.wBeta * PDF1Dbasis[slice][tc.iBeta + 1];
#endif

#ifdef HERMITE_INTERPOLANT
    // This implements Ferguson cubic interpolation based on Cubic Hermite
    // Splines
    const float w = tc.beta_weight;
    const float p0 = pdf1d_device_ptr[slice * beta_size + tc.current_beta_low_bound];
    const float p1 = pdf1d_device_ptr[slice * beta_size + tc.current_beta_low_bound + 1];
    float m0_h, m1_h;
    if (tc.current_beta_low_bound == 0) {
      m0_h = p1 - p0;  // end point
    } else {
      // standard way
      m0_h = 0.5f * (p1 - pdf1d_device_ptr[slice * beta_size + tc.current_beta_low_bound - 1]);
    }
    assert(tc.current_beta_low_bound < beta_size - 1);
    if (tc.current_beta_low_bound == beta_size - 2)
      m1_h = p1 - p0;  // end point
    else
      // standard way
      m1_h = 0.5f * (pdf1d_device_ptr[slice * beta_size + tc.current_beta_low_bound + 1] - p0);
    const float t2 = w * w;
    const float t3 = t2 * w;
    const float h01 = -2.0f * t3 + 3.0f * t2;
    const float h00 = 1.0f - h01;
    const float h11 = t3 - t2;
    const float h10 = h11 - t2 + w;

    // This implements the whole formula
    const float res = h00 * p0 + h10 * m0_h + h01 * p1 + h11 * m1_h;
    return res;
#endif
  }
#pragma endregion
};
}  // namespace evo_engine