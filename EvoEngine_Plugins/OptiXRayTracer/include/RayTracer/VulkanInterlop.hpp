#pragma once
#include <vector>

#include "Optix7.hpp"

namespace evo_engine {
class CudaImage {
 public:
  int m_mipmapLevels = 1;
  cudaExternalMemory_t m_imageExternalMemory = nullptr;
  cudaArray_t m_baseImageArray = nullptr;
  cudaMipmappedArray_t m_mipmappedImageArray = nullptr;
  cudaTextureObject_t m_textureObject = 0;
  std::vector<cudaSurfaceObject_t> m_surfaceObjects = {};
  ~CudaImage();
};

class CudaSemaphore {
 public:
  cudaExternalSemaphore_t m_semaphore;
};
}  // namespace evo_engine