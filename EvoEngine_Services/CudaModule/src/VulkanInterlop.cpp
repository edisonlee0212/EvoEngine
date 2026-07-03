#include "VulkanInterlop.hpp"

using namespace evo_engine;

CudaImage::~CudaImage() {
  for (const auto surface_object : surface_objects) {
    if (surface_object != 0) {
      CUDA_CHECK_NOEXCEPT(DestroySurfaceObject(surface_object));
    }
  }
  surface_objects.clear();
  if (texture_object != 0) {
    CUDA_CHECK_NOEXCEPT(DestroyTextureObject(texture_object));
  }
  if (mipmapped_image_array != nullptr) {
    CUDA_CHECK_NOEXCEPT(FreeMipmappedArray(mipmapped_image_array));
  }
  if (image_external_memory != nullptr) {
    CUDA_CHECK_NOEXCEPT(DestroyExternalMemory(image_external_memory));
  }
}
