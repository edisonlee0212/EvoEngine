#include "VulkanInterlop.hpp"

using namespace evo_engine;

CudaImage::~CudaImage() {
  // Creation can fail partway through the mip chain; release only successfully created resources.
  for (const auto surface : surface_objects)
    CUDA_CHECK_NOEXCEPT(DestroySurfaceObject(surface));
  if (texture_object)
    CUDA_CHECK_NOEXCEPT(DestroyTextureObject(texture_object));
  if (mipmapped_image_array)
    CUDA_CHECK_NOEXCEPT(FreeMipmappedArray(mipmapped_image_array));
  if (image_external_memory)
    CUDA_CHECK_NOEXCEPT(DestroyExternalMemory(image_external_memory));
}
