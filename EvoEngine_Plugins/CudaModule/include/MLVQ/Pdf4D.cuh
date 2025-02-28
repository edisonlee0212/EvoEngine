#pragma once

#include "Pdf3D.cuh"

namespace evo_engine {
template <typename T>
struct Pdf4D {
  /**
   * \brief The used number of 4D functions
   */
  int pdf4d_size;
  /**
   * \brief The number of slices per phi (=3D functions) to represent one 4D function
   */
  int phi_size;
  /**
   * \brief The angle phi quantization step
   */
  float phi_step;
  /**
   * \brief The database of 3D functions to which we point in the array PDF4Dslices
   */
  Pdf3D<T> pdf3d;

  void Init(const int num_of_phi) {
    phi_size = num_of_phi;
    phi_step = 360.0f / num_of_phi;
    pdf4d_size = 0;
  }

#pragma region CUDA
  // These are the data allocated maxPDF4D times, serving to represent the
  // function
  CudaBuffer pdf4d_buffer;
  int *pdf4d_device_ptr;
  CudaBuffer pdf4d_scale_buffer;
  float *pdf4d_scale_device_ptr;
  __device__ void GetVal(const int slice, T &out, SharedCoordinates &tc) const {
    const int low_phi = tc.current_phi_low_bound;
    const float w = tc.phi_weight;
    assert(low_phi >= 0 && low_phi < phi_size);
    assert(slice >= 0 && slice < pdf4d_size);
    if (low_phi != phi_size - 1) {
      glm::vec3 out2;
      pdf3d.GetVal(pdf4d_device_ptr[slice * phi_size + low_phi], out, tc);
      pdf3d.GetVal(pdf4d_device_ptr[slice * phi_size + low_phi + 1], out2, tc);
      const float s1 = pdf4d_scale_device_ptr[slice * phi_size + low_phi] * (1.0f - w);
      const float s2 = pdf4d_scale_device_ptr[slice * phi_size + low_phi + 1] * w;
      out = out * s1 + out2 * s2;
    } else {
      glm::vec3 out2;
      pdf3d.GetVal(pdf4d_device_ptr[slice * phi_size + low_phi], out, tc);
      pdf3d.GetVal(pdf4d_device_ptr[slice * phi_size], out2, tc);
      const float s1 = pdf4d_scale_device_ptr[slice * phi_size + low_phi] * (1.0f - w);
      const float s2 = pdf4d_scale_device_ptr[slice * phi_size] * w;
      out = out * s1 + out2 * s2;
    }
  }
#pragma endregion
};
}  // namespace evo_engine
