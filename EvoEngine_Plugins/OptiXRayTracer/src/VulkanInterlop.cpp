#include "VulkanInterlop.hpp"

using namespace evo_engine;

CudaImage::~CudaImage() {
  if (m_imageExternalMemory != nullptr) {
    for (int i = 0; i < m_mipmapLevels; i++) {
      CUDA_CHECK(DestroySurfaceObject(m_surfaceObjects[i]));
    }
    m_surfaceObjects.clear();
    CUDA_CHECK(DestroyTextureObject(m_textureObject));
    CUDA_CHECK(FreeMipmappedArray(m_mipmappedImageArray));
    CUDA_CHECK(DestroyExternalMemory(m_imageExternalMemory));
  }
}
