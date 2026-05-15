#pragma once

#include "Pdf2D.cuh"

namespace evo_engine {
template <typename T>
struct Pdf3D {
  /**
   * \brief The used number of 3D functions
   */
  int pdf3d_size;
  /**
   * \brief The number of slices per theta (=2D functions) to represent one 3D function
   */
  int theta_size;
  /**
   * \brief The database of 2D functions to which we point in the array PDF3Dslices
   */
  Pdf2D pdf2d;

  void Init(const int num_of_theta) {
    theta_size = num_of_theta;
    pdf3d_size = 0;
  }

#pragma region CUDA
  // These are the data allocated maxPDF2D times, serving to represent the
  // function
  CudaBuffer pdf3d_buffer;
  int *pdf3d_device_ptr;
  CudaBuffer pdf3d_scale_buffer;
  float *pdf3d_scale_device_ptr;
  __device__ void GetVal(const int slice, T &out, const SharedCoordinates &tc) const {
    const int i = tc.current_theta_low_bound;
    assert(i >= 0 && i < theta_size - 1);
    assert(slice >= 0 && slice < pdf3d_size);
    const float w = tc.theta_weight;
    glm::vec3 out2;
    pdf2d.GetVal(pdf3d_device_ptr[slice * theta_size + i], out, tc);
    pdf2d.GetVal(pdf3d_device_ptr[slice * theta_size + i + 1], out2, tc);
    const float s1 = pdf3d_scale_device_ptr[slice * theta_size + i] * (1.0f - w);
    const float s2 = pdf3d_scale_device_ptr[slice * theta_size + i + 1] * w;
    out = out * s1 + out2 * s2;
  }
#pragma endregion
};
}  // namespace evo_engine