#include <cstdio>

#include <CUDAModule.hpp>

#include <Optix7.hpp>

#include <OptiXRayTracer.hpp>

#include <cuda_gl_interop.h>

#include <cuda.h>

#include <sstream>

#include <stdexcept>

#include <vector>

#include <glm/glm.hpp>

#include "EvoEngine_SDK_PCH.hpp"

#include "Texture2D.hpp"

#include "Cubemap.hpp"

#include "RenderTexture.hpp"

#include "Platform.hpp"

#include "VulkanInterlop.hpp"

#include "Application.hpp"
#include "RenderLayer.hpp"
using namespace evo_engine;

std::unique_ptr<OptiXRayTracer>& CudaModule::GetRayTracer() {
  return GetInstance().ray_tracer_;
}

CudaModule& CudaModule::GetInstance() {
  static CudaModule instance;
  return instance;
}

void CudaModule::Init() {
  auto& cuda_module = GetInstance();
  // Choose which GPU to run on, change this on a multi-GPU system.
  CUDA_CHECK(SetDevice(0));
  OPTIX_CHECK(optixInitWithHandle(&cuda_module.optix_handle_));
  cuda_module.ray_tracer_ = std::make_unique<OptiXRayTracer>();
  cuda_module.initialized_ = true;
}

void CudaModule::Terminate() {
  auto& cuda_module = GetInstance();
  cuda_module.ray_tracer_.reset();
  OPTIX_CHECK(optixUninitWithHandle(cuda_module.optix_handle_));
  CUDA_CHECK(DeviceReset());
  cuda_module.initialized_ = false;
}

void CudaModule::EstimateIlluminationRayTracing(const EnvironmentProperties& environmentProperties,
                                                const RayProperties& rayProperties,
                                                std::vector<IlluminationSampler<glm::vec3>>& lightProbes, unsigned seed,
                                                float pushNormalDistance) {
  auto& cudaModule = GetInstance();
#pragma region Prepare light probes
  size_t size = lightProbes.size();
  CudaBuffer deviceLightProbes;
  deviceLightProbes.Upload(lightProbes);
#pragma endregion
  cudaModule.ray_tracer_->EstimateIllumination(size, environmentProperties, rayProperties, deviceLightProbes, seed,
                                               pushNormalDistance);
  deviceLightProbes.Download(lightProbes.data(), size);
  deviceLightProbes.Free();
}

void CudaModule::SamplePointCloud(const EnvironmentProperties& environmentProperties,
                                  std::vector<PointCloudSample>& samples) {
  auto& cudaModule = GetInstance();
#pragma region Prepare light probes
  size_t size = samples.size();
  CudaBuffer deviceSamples;
  deviceSamples.Upload(samples);
#pragma endregion
  cudaModule.ray_tracer_->ScanPointCloud(size, environmentProperties, deviceSamples);
  deviceSamples.Download(samples.data(), size);
  deviceSamples.Free();
}

std::shared_ptr<CudaImage> CudaModule::ImportTexture2D(const std::shared_ptr<evo_engine::Texture2D>& texture2D) {
  if (!ApplicationContext::Get().GetLayer<RenderLayer>())
    return nullptr;

  auto image = texture2D->GetImage();

  auto cudaImage = std::make_shared<CudaImage>();

  cudaExternalMemoryHandleDesc cudaExtMemHandleDesc;
  memset(&cudaExtMemHandleDesc, 0, sizeof(cudaExtMemHandleDesc));
#if ENABLE_EXTERNAL_MEMORY
#  ifdef _WIN64
  cudaExtMemHandleDesc.type = cudaExternalMemoryHandleTypeOpaqueWin32;
  cudaExtMemHandleDesc.handle.win32.handle =
      image->GetVkImageMemHandle(VK_EXTERNAL_MEMORY_HANDLE_TYPE_OPAQUE_WIN32_BIT);
#  else
  cudaExtMemHandleDesc.type = cudaExternalMemoryHandleTypeOpaqueFd;

  cudaExtMemHandleDesc.handle.fd = image->GetVkImageMemHandle(VK_EXTERNAL_MEMORY_HANDLE_TYPE_OPAQUE_FD_BIT_KHR);
#  endif
  VkMemoryRequirements vkMemoryRequirements = {};
  vkGetImageMemoryRequirements(evo_engine::Platform::GetVkDevice(), image->GetVkImage(), &vkMemoryRequirements);
  size_t totalImageMemSize = vkMemoryRequirements.size;
  cudaExtMemHandleDesc.size = totalImageMemSize;

  CUDA_CHECK(ImportExternalMemory(&cudaImage->image_external_memory, &cudaExtMemHandleDesc));
#endif

  cudaExternalMemoryMipmappedArrayDesc externalMemoryMipmappedArrayDesc;

  memset(&externalMemoryMipmappedArrayDesc, 0, sizeof(externalMemoryMipmappedArrayDesc));
  VkExtent3D imageExtent = image->GetExtent();
  cudaExtent extent = make_cudaExtent(imageExtent.width, imageExtent.height, 0);
  cudaChannelFormatDesc formatDesc;

  int bit_size = 32;
  switch (Platform::Constants::texture_2d) {
    case VK_FORMAT_R64G64B64A64_SFLOAT: {
      bit_size = 64;
      break;
    }
    case VK_FORMAT_R32G32B32A32_SFLOAT: {
      bit_size = 32;
      break;
    }
    case VK_FORMAT_R16G16B16A16_SFLOAT: {
      bit_size = 16;
      break;
    }
  }

  formatDesc.x = bit_size;
  formatDesc.y = bit_size;
  formatDesc.z = bit_size;
  formatDesc.w = bit_size;
  formatDesc.f = cudaChannelFormatKindFloat;

  externalMemoryMipmappedArrayDesc.offset = 0;
  externalMemoryMipmappedArrayDesc.formatDesc = formatDesc;
  externalMemoryMipmappedArrayDesc.extent = extent;
  externalMemoryMipmappedArrayDesc.flags = cudaArrayDefault;
  externalMemoryMipmappedArrayDesc.numLevels = image->GetMipLevels();

  CUDA_CHECK(ExternalMemoryGetMappedMipmappedArray(&cudaImage->mipmapped_image_array, cudaImage->image_external_memory,
                                                   &externalMemoryMipmappedArrayDesc));

  for (int mipLevelIdx = 0; mipLevelIdx < image->GetMipLevels(); mipLevelIdx++) {
    cudaArray_t cudaMipLevelArray;
    cudaResourceDesc resourceDesc;

    CUDA_CHECK(GetMipmappedArrayLevel(&cudaMipLevelArray, cudaImage->mipmapped_image_array, mipLevelIdx));

    memset(&resourceDesc, 0, sizeof(resourceDesc));
    resourceDesc.resType = cudaResourceTypeArray;
    resourceDesc.res.array.array = cudaMipLevelArray;

    cudaSurfaceObject_t surfaceObject;
    CUDA_CHECK(CreateSurfaceObject(&surfaceObject, &resourceDesc));

    cudaImage->surface_objects.push_back(surfaceObject);
  }

  cudaResourceDesc resDescr;
  memset(&resDescr, 0, sizeof(cudaResourceDesc));

  resDescr.resType = cudaResourceTypeMipmappedArray;
  resDescr.res.mipmap.mipmap = cudaImage->mipmapped_image_array;

  cudaTextureDesc texDescr;
  memset(&texDescr, 0, sizeof(cudaTextureDesc));

  texDescr.normalizedCoords = true;
  texDescr.filterMode = cudaFilterModeLinear;
  texDescr.mipmapFilterMode = cudaFilterModeLinear;

  texDescr.addressMode[0] = cudaAddressModeWrap;
  texDescr.addressMode[1] = cudaAddressModeWrap;
  texDescr.addressMode[2] = cudaAddressModeWrap;

  texDescr.maxAnisotropy = evo_engine::Platform::GetSelectedPhysicalDevice()->properties.limits.maxSamplerAnisotropy;

  texDescr.minMipmapLevelClamp = 0;
  texDescr.maxMipmapLevelClamp = static_cast<float>(image->GetMipLevels() - 1);

  texDescr.readMode = cudaReadModeElementType;

  CUDA_CHECK(CreateTextureObject(&cudaImage->texture_object, &resDescr, &texDescr, NULL));

  return cudaImage;
}

std::shared_ptr<CudaImage> CudaModule::ImportCubemap(const std::shared_ptr<evo_engine::Cubemap>& cubemap) {
  if (!ApplicationContext::Get().GetLayer<RenderLayer>())
    return nullptr;

  auto image = cubemap->GetImage();

  auto cudaImage = std::make_shared<CudaImage>();

  cudaExternalMemoryHandleDesc cudaExtMemHandleDesc;
  memset(&cudaExtMemHandleDesc, 0, sizeof(cudaExtMemHandleDesc));
#if ENABLE_EXTERNAL_MEMORY
#  ifdef _WIN64
  cudaExtMemHandleDesc.type = cudaExternalMemoryHandleTypeOpaqueWin32;
  cudaExtMemHandleDesc.handle.win32.handle =
      image->GetVkImageMemHandle(VK_EXTERNAL_MEMORY_HANDLE_TYPE_OPAQUE_WIN32_BIT);
#  else
  cudaExtMemHandleDesc.type = cudaExternalMemoryHandleTypeOpaqueFd;

  cudaExtMemHandleDesc.handle.fd = image->GetVkImageMemHandle(VK_EXTERNAL_MEMORY_HANDLE_TYPE_OPAQUE_FD_BIT_KHR);
#  endif
  VkMemoryRequirements vkMemoryRequirements = {};
  vkGetImageMemoryRequirements(evo_engine::Platform::GetVkDevice(), image->GetVkImage(), &vkMemoryRequirements);
  size_t totalImageMemSize = vkMemoryRequirements.size;
  cudaExtMemHandleDesc.size = totalImageMemSize;

  CUDA_CHECK(ImportExternalMemory(&cudaImage->image_external_memory, &cudaExtMemHandleDesc));
#endif

  cudaExternalMemoryMipmappedArrayDesc externalMemoryMipmappedArrayDesc;

  memset(&externalMemoryMipmappedArrayDesc, 0, sizeof(externalMemoryMipmappedArrayDesc));
  VkExtent3D imageExtent = image->GetExtent();
  cudaExtent extent = make_cudaExtent(imageExtent.width, imageExtent.height, 6);
  cudaChannelFormatDesc formatDesc;
  int bit_size = 32;
  switch (Platform::Constants::texture_2d) {
    case VK_FORMAT_R64G64B64A64_SFLOAT: {
      bit_size = 64;
      break;
    }
    case VK_FORMAT_R32G32B32A32_SFLOAT: {
      bit_size = 32;
      break;
    }
    case VK_FORMAT_R16G16B16A16_SFLOAT: {
      bit_size = 16;
      break;
    }
  }

  formatDesc.x = bit_size;
  formatDesc.y = bit_size;
  formatDesc.z = bit_size;
  formatDesc.w = bit_size;
  formatDesc.f = cudaChannelFormatKindFloat;

  externalMemoryMipmappedArrayDesc.offset = 0;
  externalMemoryMipmappedArrayDesc.formatDesc = formatDesc;
  externalMemoryMipmappedArrayDesc.extent = extent;
  externalMemoryMipmappedArrayDesc.flags = cudaArrayCubemap;
  externalMemoryMipmappedArrayDesc.numLevels = image->GetMipLevels();

  CUDA_CHECK(ExternalMemoryGetMappedMipmappedArray(&cudaImage->mipmapped_image_array, cudaImage->image_external_memory,
                                                   &externalMemoryMipmappedArrayDesc));

  for (int mipLevelIdx = 0; mipLevelIdx < image->GetMipLevels(); mipLevelIdx++) {
    cudaArray_t cudaMipLevelArray;
    cudaResourceDesc resourceDesc;

    CUDA_CHECK(GetMipmappedArrayLevel(&cudaMipLevelArray, cudaImage->mipmapped_image_array, mipLevelIdx));

    memset(&resourceDesc, 0, sizeof(resourceDesc));
    resourceDesc.resType = cudaResourceTypeArray;
    resourceDesc.res.array.array = cudaMipLevelArray;

    cudaSurfaceObject_t surfaceObject;
    CUDA_CHECK(CreateSurfaceObject(&surfaceObject, &resourceDesc));

    cudaImage->surface_objects.push_back(surfaceObject);
  }

  cudaResourceDesc resDescr;
  memset(&resDescr, 0, sizeof(cudaResourceDesc));

  resDescr.resType = cudaResourceTypeMipmappedArray;
  resDescr.res.mipmap.mipmap = cudaImage->mipmapped_image_array;

  cudaTextureDesc texDescr;
  memset(&texDescr, 0, sizeof(cudaTextureDesc));

  texDescr.normalizedCoords = true;
  texDescr.filterMode = cudaFilterModeLinear;
  texDescr.mipmapFilterMode = cudaFilterModeLinear;

  texDescr.addressMode[0] = cudaAddressModeWrap;
  texDescr.addressMode[1] = cudaAddressModeWrap;
  texDescr.addressMode[2] = cudaAddressModeWrap;

  texDescr.maxAnisotropy = evo_engine::Platform::GetSelectedPhysicalDevice()->properties.limits.maxSamplerAnisotropy;

  texDescr.minMipmapLevelClamp = 0;
  texDescr.maxMipmapLevelClamp = static_cast<float>(image->GetMipLevels() - 1);
  texDescr.seamlessCubemap = true;
  texDescr.readMode = cudaReadModeElementType;

  CUDA_CHECK(CreateTextureObject(&cudaImage->texture_object, &resDescr, &texDescr, NULL));

  return cudaImage;
}

std::shared_ptr<CudaImage> CudaModule::ImportRenderTexture(const std::shared_ptr<RenderTexture>& renderTexture) {
  if (!ApplicationContext::Get().GetLayer<RenderLayer>())
    return nullptr;

  auto image = renderTexture->GetColorImage();

  auto cudaImage = std::make_shared<CudaImage>();

  cudaExternalMemoryHandleDesc cudaExtMemHandleDesc;
  memset(&cudaExtMemHandleDesc, 0, sizeof(cudaExtMemHandleDesc));
#if ENABLE_EXTERNAL_MEMORY
#  ifdef _WIN64
  cudaExtMemHandleDesc.type = cudaExternalMemoryHandleTypeOpaqueWin32;
  cudaExtMemHandleDesc.handle.win32.handle =
      image->GetVkImageMemHandle(VK_EXTERNAL_MEMORY_HANDLE_TYPE_OPAQUE_WIN32_BIT);
#  else
  cudaExtMemHandleDesc.type = cudaExternalMemoryHandleTypeOpaqueFd;

  cudaExtMemHandleDesc.handle.fd = image->GetVkImageMemHandle(VK_EXTERNAL_MEMORY_HANDLE_TYPE_OPAQUE_FD_BIT_KHR);
#  endif
  VkMemoryRequirements vkMemoryRequirements = {};
  vkGetImageMemoryRequirements(evo_engine::Platform::GetVkDevice(), image->GetVkImage(), &vkMemoryRequirements);
  size_t totalImageMemSize = vkMemoryRequirements.size;
  cudaExtMemHandleDesc.size = totalImageMemSize;

  CUDA_CHECK(ImportExternalMemory(&cudaImage->image_external_memory, &cudaExtMemHandleDesc));
#endif

  cudaExternalMemoryMipmappedArrayDesc externalMemoryMipmappedArrayDesc;

  memset(&externalMemoryMipmappedArrayDesc, 0, sizeof(externalMemoryMipmappedArrayDesc));
  VkExtent3D imageExtent = image->GetExtent();
  cudaExtent extent = make_cudaExtent(imageExtent.width, imageExtent.height, 0);
  int bit_size = 32;
  switch (Platform::Constants::render_texture_color) {
    case VK_FORMAT_R64G64B64A64_SFLOAT: {
      bit_size = 64;
      break;
    }
    case VK_FORMAT_R32G32B32A32_SFLOAT: {
      bit_size = 32;
      break;
    }
    case VK_FORMAT_R16G16B16A16_SFLOAT: {
      bit_size = 16;
      break;
    }
  }

  cudaChannelFormatDesc formatDesc;
  formatDesc.x = bit_size;
  formatDesc.y = bit_size;
  formatDesc.z = bit_size;
  formatDesc.w = bit_size;

  formatDesc.f = cudaChannelFormatKindFloat;

  externalMemoryMipmappedArrayDesc.offset = 0;
  externalMemoryMipmappedArrayDesc.formatDesc = formatDesc;
  externalMemoryMipmappedArrayDesc.extent = extent;
  externalMemoryMipmappedArrayDesc.flags = cudaArrayDefault;
  externalMemoryMipmappedArrayDesc.numLevels = image->GetMipLevels();

  CUDA_CHECK(ExternalMemoryGetMappedMipmappedArray(&cudaImage->mipmapped_image_array, cudaImage->image_external_memory,
                                                   &externalMemoryMipmappedArrayDesc));

  for (int mipLevelIdx = 0; mipLevelIdx < image->GetMipLevels(); mipLevelIdx++) {
    cudaArray_t cudaMipLevelArray;
    cudaResourceDesc resourceDesc;

    CUDA_CHECK(GetMipmappedArrayLevel(&cudaMipLevelArray, cudaImage->mipmapped_image_array, mipLevelIdx));

    memset(&resourceDesc, 0, sizeof(resourceDesc));
    resourceDesc.resType = cudaResourceTypeArray;
    resourceDesc.res.array.array = cudaMipLevelArray;

    cudaSurfaceObject_t surfaceObject;
    CUDA_CHECK(CreateSurfaceObject(&surfaceObject, &resourceDesc));

    cudaImage->surface_objects.push_back(surfaceObject);
  }

  cudaResourceDesc resDescr;
  memset(&resDescr, 0, sizeof(cudaResourceDesc));

  resDescr.resType = cudaResourceTypeMipmappedArray;
  resDescr.res.mipmap.mipmap = cudaImage->mipmapped_image_array;

  cudaTextureDesc texDescr;
  memset(&texDescr, 0, sizeof(cudaTextureDesc));

  texDescr.normalizedCoords = true;
  texDescr.filterMode = cudaFilterModeLinear;
  texDescr.mipmapFilterMode = cudaFilterModeLinear;

  texDescr.addressMode[0] = cudaAddressModeWrap;
  texDescr.addressMode[1] = cudaAddressModeWrap;
  texDescr.addressMode[2] = cudaAddressModeWrap;

  texDescr.maxAnisotropy = evo_engine::Platform::GetSelectedPhysicalDevice()->properties.limits.maxSamplerAnisotropy;

  texDescr.minMipmapLevelClamp = 0;
  texDescr.maxMipmapLevelClamp = static_cast<float>(image->GetMipLevels() - 1);

  texDescr.readMode = cudaReadModeElementType;

  CUDA_CHECK(CreateTextureObject(&cudaImage->texture_object, &resDescr, &texDescr, NULL));

  return cudaImage;
}

std::shared_ptr<CudaSemaphore> CudaModule::ImportSemaphore(const std::shared_ptr<evo_engine::Semaphore>& semaphore) {
  if (!ApplicationContext::Get().GetLayer<RenderLayer>())
    return nullptr;
  auto cudaSemaphore = std::make_shared<CudaSemaphore>();

  cudaExternalSemaphoreHandleDesc externalSemaphoreHandleDesc;
  memset(&externalSemaphoreHandleDesc, 0, sizeof(externalSemaphoreHandleDesc));
#ifdef _WIN64
  externalSemaphoreHandleDesc.type = cudaExternalSemaphoreHandleTypeOpaqueWin32;
  externalSemaphoreHandleDesc.handle.win32.handle =
      semaphore->GetVkSemaphoreHandle(VK_EXTERNAL_SEMAPHORE_HANDLE_TYPE_OPAQUE_WIN32_BIT);
#else
  externalSemaphoreHandleDesc.type = cudaExternalSemaphoreHandleTypeOpaqueFd;
  externalSemaphoreHandleDesc.handle.fd =
      semaphore->GetVkSemaphoreHandle(VK_EXTERNAL_SEMAPHORE_HANDLE_TYPE_OPAQUE_FD_BIT);
#endif
  externalSemaphoreHandleDesc.flags = 0;

  CUDA_CHECK(ImportExternalSemaphore(&cudaSemaphore->m_semaphore, &externalSemaphoreHandleDesc));

  return cudaSemaphore;
}
