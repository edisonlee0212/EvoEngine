#pragma once
#include <vector>

#include "Optix7.hpp"

namespace evo_engine {
class CudaImage {
 public:
  int mipmap_levels = 1;
  cudaExternalMemory_t image_external_memory = nullptr;
  cudaArray_t base_image_array = nullptr;
  cudaMipmappedArray_t mipmapped_image_array = nullptr;
  cudaTextureObject_t texture_object = 0;
  std::vector<cudaSurfaceObject_t> surface_objects = {};
  ~CudaImage();
};

class CudaSemaphore {
 public:
  cudaExternalSemaphore_t m_semaphore;
};
}  // namespace evo_engine