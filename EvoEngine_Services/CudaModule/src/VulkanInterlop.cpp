#include "VulkanInterlop.hpp"

using namespace evo_engine;

CudaImage::~CudaImage() {
  if (image_external_memory != nullptr) {
    for (int i = 0; i < mipmap_levels; i++) {
      CUDA_CHECK(DestroySurfaceObject(surface_objects[i]));
    }
    surface_objects.clear();
    CUDA_CHECK(DestroyTextureObject(texture_object));
    CUDA_CHECK(FreeMipmappedArray(mipmapped_image_array));
    CUDA_CHECK(DestroyExternalMemory(image_external_memory));
  }
}
