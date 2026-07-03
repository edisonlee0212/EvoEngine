#include <cstdio>

#include <cstring>

#include <CUDAModule.hpp>

#include <Optix7.hpp>
#include <optix_stubs.h>

#include <OptiXRayTracer.hpp>

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

namespace {
void ImportExternalImageMemory(const std::shared_ptr<Image>& image, CudaImage& cuda_image) {
#if ENABLE_EXTERNAL_MEMORY
  cudaExternalMemoryHandleDesc cuda_ext_mem_handle_desc{};
#  ifdef _WIN64
  cuda_ext_mem_handle_desc.type = cudaExternalMemoryHandleTypeOpaqueWin32;
  cuda_ext_mem_handle_desc.handle.win32.handle =
      image->GetVkImageMemHandle(VK_EXTERNAL_MEMORY_HANDLE_TYPE_OPAQUE_WIN32_BIT);
#  else
  cuda_ext_mem_handle_desc.type = cudaExternalMemoryHandleTypeOpaqueFd;
  cuda_ext_mem_handle_desc.handle.fd = image->GetVkImageMemHandle(VK_EXTERNAL_MEMORY_HANDLE_TYPE_OPAQUE_FD_BIT_KHR);
#  endif

  const auto& allocation_info = image->GetVmaAllocationInfo();
  cuda_ext_mem_handle_desc.size = allocation_info.size;
  cuda_ext_mem_handle_desc.flags = cudaExternalMemoryDedicated;

  const auto result = cudaImportExternalMemory(&cuda_image.image_external_memory, &cuda_ext_mem_handle_desc);
#  ifdef _WIN64
  if (cuda_ext_mem_handle_desc.handle.win32.handle) {
    CloseHandle(cuda_ext_mem_handle_desc.handle.win32.handle);
  }
#  endif
  if (result != cudaSuccess) {
    std::stringstream message;
    const auto extent = image->GetExtent();
    message << "CUDA Error " << cudaGetErrorName(result) << " (" << cudaGetErrorString(result)
            << ") while importing external image memory. extent=" << extent.width << "x" << extent.height << "x"
            << extent.depth << ", mip_levels=" << image->GetMipLevels() << ", allocation_size="
            << allocation_info.size << ", allocation_offset=" << allocation_info.offset;
    throw std::runtime_error(message.str());
  }
#endif
}

unsigned int GetCudaArrayFlags(const std::shared_ptr<Image>& image, const bool cubemap, const bool surface_access) {
  auto flags = surface_access ? cudaArraySurfaceLoadStore : cudaArrayDefault;
  if ((image->GetUsage() & VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT) != 0) {
    flags |= cudaArrayColorAttachment;
  }
  if (cubemap) {
    flags |= cudaArrayCubemap;
  }
  return flags;
}

void MapExternalMipmappedArray(const std::shared_ptr<Image>& image, CudaImage& cuda_image,
                               const cudaExternalMemoryMipmappedArrayDesc& descriptor) {
  const auto prior_error = cudaGetLastError();
  size_t free_memory = 0;
  size_t total_memory = 0;
  const auto mem_info_result = cudaMemGetInfo(&free_memory, &total_memory);

  const auto result = cudaExternalMemoryGetMappedMipmappedArray(&cuda_image.mipmapped_image_array,
                                                               cuda_image.image_external_memory, &descriptor);
  if (result != cudaSuccess) {
    std::stringstream message;
    const auto extent = image->GetExtent();
    const auto& allocation_info = image->GetVmaAllocationInfo();
    message << "CUDA Error " << cudaGetErrorName(result) << " (" << cudaGetErrorString(result)
            << ") while mapping external image array. extent=" << extent.width << "x" << extent.height << "x"
            << extent.depth << ", mip_levels=" << image->GetMipLevels() << ", allocation_size="
            << allocation_info.size << ", allocation_offset=" << allocation_info.offset
            << ", descriptor_offset=" << descriptor.offset << ", descriptor_flags=" << descriptor.flags
            << ", image_usage=" << image->GetUsage() << ", image_format=" << image->GetFormat()
            << ", prior_cuda_error=" << cudaGetErrorName(prior_error) << " (" << cudaGetErrorString(prior_error)
            << "), mem_info_result=" << cudaGetErrorName(mem_info_result) << " ("
            << cudaGetErrorString(mem_info_result) << "), free_cuda_memory=" << free_memory
            << ", total_cuda_memory=" << total_memory;
    throw std::runtime_error(message.str());
  }
}

int SelectCudaDeviceForVulkanDevice() {
  int cuda_device_count = 0;
  CUDA_CHECK(GetDeviceCount(&cuda_device_count));
  const auto vulkan_device = Platform::GetSelectedPhysicalDevice();
  if (!vulkan_device) {
    return 0;
  }

#ifdef _WIN64
  if (vulkan_device->device_id_properties.deviceLUIDValid) {
    for (int cuda_device = 0; cuda_device < cuda_device_count; ++cuda_device) {
      cudaDeviceProp cuda_properties{};
      CUDA_CHECK(GetDeviceProperties(&cuda_properties, cuda_device));
      if (std::memcmp(cuda_properties.luid, vulkan_device->device_id_properties.deviceLUID, VK_LUID_SIZE) == 0 &&
          cuda_properties.luidDeviceNodeMask == vulkan_device->device_id_properties.deviceNodeMask) {
        EVOENGINE_LOG("CUDA matched Vulkan physical device by LUID: " + std::string(cuda_properties.name));
        return cuda_device;
      }
    }
    throw std::runtime_error("No CUDA device matches the selected Vulkan physical device LUID.");
  }
#endif

  for (int cuda_device = 0; cuda_device < cuda_device_count; ++cuda_device) {
    cudaDeviceProp cuda_properties{};
    CUDA_CHECK(GetDeviceProperties(&cuda_properties, cuda_device));
    if (std::memcmp(cuda_properties.uuid.bytes, vulkan_device->device_id_properties.deviceUUID, VK_UUID_SIZE) == 0) {
      EVOENGINE_LOG("CUDA matched Vulkan physical device by UUID: " + std::string(cuda_properties.name));
      return cuda_device;
    }
  }
  return 0;
}
}  // namespace

std::unique_ptr<OptiXRayTracer>& CudaModule::GetRayTracer() {
  return GetInstance().ray_tracer_;
}

CudaModule& CudaModule::GetInstance() {
  static CudaModule instance;
  return instance;
}

void CudaModule::Init() {
  auto& cuda_module = GetInstance();
  cuda_module.texture_2d_cache_.clear();
  CUDA_CHECK(SetDevice(SelectCudaDeviceForVulkanDevice()));
  OPTIX_CHECK(optixInitWithHandle(&cuda_module.optix_handle_));
  cuda_module.ray_tracer_ = std::make_unique<OptiXRayTracer>();
  cuda_module.initialized_ = true;
}

void CudaModule::Terminate() {
  auto& cuda_module = GetInstance();
  cuda_module.texture_2d_cache_.clear();
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

void CudaModule::EstimateIlluminationRayTracingSpectral(const EnvironmentProperties& environmentProperties,
                                                        const RayProperties& rayProperties,
                                                        std::vector<IlluminationSampler<glm::vec3>>& lightProbes,
                                                        const unsigned seed, const float pushNormalDistance) {
  auto& cudaModule = GetInstance();
#pragma region Prepare light probes
  const size_t size = lightProbes.size();
  CudaBuffer deviceLightProbes;
  deviceLightProbes.Upload(lightProbes);
#pragma endregion
  cudaModule.ray_tracer_->EstimateIlluminationSpectral(size, environmentProperties, rayProperties, deviceLightProbes,
                                                       seed, pushNormalDistance);
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

  const auto texture_handle = texture2D->GetHandle().GetValue();
  const auto texture_version = texture2D->GetVersion();
  auto& cache = GetInstance().texture_2d_cache_;
  if (texture_handle != 0) {
    if (const auto search = cache.find(texture_handle);
        search != cache.end() && search->second.first == texture_version) {
      return search->second.second;
    }
  }

  auto cudaImage = std::make_shared<CudaImage>();

  const auto resolution = texture2D->GetResolution();
  const auto& local_data = texture2D->GetLocalData();
  cudaExtent extent = make_cudaExtent(resolution.x, resolution.y, 0);
  cudaChannelFormatDesc formatDesc{};
  formatDesc.x = 32;
  formatDesc.y = 32;
  formatDesc.z = 32;
  formatDesc.w = 32;
  formatDesc.f = cudaChannelFormatKindFloat;
  size_t free_memory = 0;
  size_t total_memory = 0;
  const auto mem_info_result = cudaMemGetInfo(&free_memory, &total_memory);
  const auto allocation_result =
      cudaMallocMipmappedArray(&cudaImage->mipmapped_image_array, &formatDesc, extent, 1, cudaArrayDefault);
  if (allocation_result != cudaSuccess) {
    std::stringstream message;
    message << "CUDA Error " << cudaGetErrorName(allocation_result) << " ("
            << cudaGetErrorString(allocation_result) << ") while allocating material texture CUDA array. resolution="
            << resolution.x << "x" << resolution.y << ", local_pixels=" << local_data.size()
            << ", requested_bytes=" << static_cast<size_t>(resolution.x) * resolution.y * sizeof(glm::vec4)
            << ", mem_info_result=" << cudaGetErrorName(mem_info_result) << " ("
            << cudaGetErrorString(mem_info_result) << "), free_cuda_memory=" << free_memory
            << ", total_cuda_memory=" << total_memory;
    throw std::runtime_error(message.str());
  }
  cudaImage->mipmap_levels = 1;

  cudaArray_t cudaMipLevelArray;
  CUDA_CHECK(GetMipmappedArrayLevel(&cudaMipLevelArray, cudaImage->mipmapped_image_array, 0));
  const auto row_size = static_cast<size_t>(resolution.x) * sizeof(glm::vec4);
  CUDA_CHECK(Memcpy2DToArray(cudaMipLevelArray, 0, 0, local_data.data(), row_size, row_size, resolution.y,
                             cudaMemcpyHostToDevice));

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
  texDescr.maxMipmapLevelClamp = 0.0f;

  texDescr.readMode = cudaReadModeElementType;

  CUDA_CHECK(CreateTextureObject(&cudaImage->texture_object, &resDescr, &texDescr, NULL));

  if (texture_handle != 0)
    cache[texture_handle] = {texture_version, cudaImage};

  return cudaImage;
}

std::shared_ptr<CudaImage> CudaModule::ImportCubemap(const std::shared_ptr<evo_engine::Cubemap>& cubemap) {
  if (!ApplicationContext::Get().GetLayer<RenderLayer>())
    return nullptr;

  auto image = cubemap->GetImage();

  auto cudaImage = std::make_shared<CudaImage>();

  ImportExternalImageMemory(image, *cudaImage);

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

  externalMemoryMipmappedArrayDesc.offset = image->GetVmaAllocationInfo().offset;
  externalMemoryMipmappedArrayDesc.formatDesc = formatDesc;
  externalMemoryMipmappedArrayDesc.extent = extent;
  externalMemoryMipmappedArrayDesc.flags = GetCudaArrayFlags(image, true, false);
  externalMemoryMipmappedArrayDesc.numLevels = image->GetMipLevels();
  cudaImage->mipmap_levels = image->GetMipLevels();

  MapExternalMipmappedArray(image, *cudaImage, externalMemoryMipmappedArrayDesc);

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

  ImportExternalImageMemory(image, *cudaImage);

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

  externalMemoryMipmappedArrayDesc.offset = image->GetVmaAllocationInfo().offset;
  externalMemoryMipmappedArrayDesc.formatDesc = formatDesc;
  externalMemoryMipmappedArrayDesc.extent = extent;
  externalMemoryMipmappedArrayDesc.flags = GetCudaArrayFlags(image, false, false);
  externalMemoryMipmappedArrayDesc.numLevels = image->GetMipLevels();
  cudaImage->mipmap_levels = image->GetMipLevels();

  MapExternalMipmappedArray(image, *cudaImage, externalMemoryMipmappedArrayDesc);

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
