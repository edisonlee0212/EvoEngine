#pragma once

#include "VectorColor.cuh"

namespace evo_engine {
struct IndexAB {
  /**
   * \brief The number of values for 1D index slices
   */
  int index_size;
  /**
   * \brief Current length of index slice
   */
  int beta_size;
  VectorColor vector_color;
  void Init(const int num_of_beta) {
    assert(num_of_beta > 0);
    beta_size = num_of_beta;
    index_size = 0;
  }
#pragma region CUDA
  /**
   * \brief The data array of 1D colour index slices
   */
  CudaBuffer indices_buffer;
  int *indices_device_ptr;
  // get a single color value specified by slice, slice position and ab
  // (0,1)
  /**
   * \brief Get a single color value specified by slice, slice position and ab
   * \param slice current index of slice.
   * \param beta Index of the color.
   * \param ab Channel of the color.
   * \return The color represented by float.
   */
  __device__ [[nodiscard]] float GetVal(const int slice, const int beta, const int ab) const {
    assert(slice >= 0 && slice < index_size);
    assert(beta >= 0 && beta < beta_size);
    return vector_color.GetVal(indices_device_ptr[slice * beta_size + beta], ab);
  }

  /**
   * \brief Get a-b color from given coordinate.
   * \param slice current index of slice.
   * \param out Target position to save color.
   * \param tc Input coordinates.
   */
  __device__ void ToAbColor(const int slice, glm::vec3 &out, const SharedCoordinates &tc) const {
    out[0] = (1.f - tc.beta_weight) * GetVal(slice, tc.current_beta_low_bound, 0) +
             tc.beta_weight * GetVal(slice, tc.current_beta_low_bound + 1, 0);
    out[1] = (1.f - tc.beta_weight) * GetVal(slice, tc.current_beta_low_bound, 1) +
             tc.beta_weight * GetVal(slice, tc.current_beta_low_bound + 1, 1);
  }
#pragma endregion
};
}  // namespace evo_engine