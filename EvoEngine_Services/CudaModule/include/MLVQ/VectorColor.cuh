#pragma once

#include "SharedCoordinates.cuh"

namespace evo_engine {
struct VectorColor {
  /**
   * \brief The index from which we start the search
   */
  int start_index = 0;
  /**
   * \brief Number of channels describing one color (in our case usually 2 (CIE a-b))
   */
  int channel_size = 2;
  /**
   * \brief Number of stored a-b colors
   */
  int color_size = 0;

  void Init() {
    start_index = 0;
    color_size = 0;
    channel_size = 2;
  }
#pragma region CUDA
  /**
   * \brief The data array of a-b colors
   */
  CudaBuffer colors_buffer;
  float *colors_device_ptr;
  /**
   * \brief Retrieve a-b color from the array.
   * \param color_index The index of the color.
   * \param channel_index The channel of the color.
   * \return Color represented by single float.
   */
  __device__ [[nodiscard]] float GetVal(const int color_index, const int channel_index) const {
    assert(channel_index >= 0 || channel_index < channel_size);
    assert(color_index >= 0 && color_index < color_size);
    return colors_device_ptr[color_index * channel_size + channel_index];
  }
#pragma endregion
};
}  // namespace evo_engine