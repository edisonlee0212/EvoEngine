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
using namespace evo_engine;

std::unique_ptr<OptiXRayTracer>& CudaModule::GetRayTracer() {
	return GetInstance().m_rayTracer;
}

CudaModule& CudaModule::GetInstance() {
	static CudaModule instance;
	return instance;
}

void CudaModule::Init() {
	auto& cudaModule = GetInstance();
	// Choose which GPU to run on, change this on a multi-GPU system.
	CUDA_CHECK(SetDevice(0));
	OPTIX_CHECK(optixInitWithHandle(&cudaModule.m_optixHandle));
	cudaModule.m_rayTracer = std::make_unique<OptiXRayTracer>();
	cudaModule.m_initialized = true;
}

void CudaModule::Terminate() {
	auto& cudaModule = GetInstance();
	cudaModule.m_rayTracer.reset();
	OPTIX_CHECK(optixUninitWithHandle(cudaModule.m_optixHandle));
	CUDA_CHECK(DeviceReset());
	cudaModule.m_initialized = false;
}


void CudaModule::EstimateIlluminationRayTracing(const EnvironmentProperties& environmentProperties, const RayProperties& rayProperties,
	std::vector<IlluminationSampler<glm::vec3>>& lightProbes, unsigned seed, float pushNormalDistance) {
	auto& cudaModule = GetInstance();
#pragma region Prepare light probes
	size_t size = lightProbes.size();
	CudaBuffer deviceLightProbes;
	deviceLightProbes.Upload(lightProbes);
#pragma endregion
	cudaModule.m_rayTracer->EstimateIllumination(size, environmentProperties, rayProperties, deviceLightProbes, seed, pushNormalDistance);
	deviceLightProbes.Download(lightProbes.data(), size);
	deviceLightProbes.Free();
}

void
CudaModule::SamplePointCloud(const EnvironmentProperties& environmentProperties,
	std::vector<PointCloudSample>& samples) {
	auto& cudaModule = GetInstance();
#pragma region Prepare light probes
	size_t size = samples.size();
	CudaBuffer deviceSamples;
	deviceSamples.Upload(samples);
#pragma endregion
	cudaModule.m_rayTracer->ScanPointCloud(size, environmentProperties, deviceSamples);
	deviceSamples.Download(samples.data(), size);
	deviceSamples.Free();
}

std::shared_ptr<CudaImage> CudaModule::ImportTexture2D(const std::shared_ptr<evo_engine::Texture2D>& texture2D)
{
	auto image = texture2D->GetImage();

	auto cudaImage = std::make_shared<CudaImage>();

	cudaExternalMemoryHandleDesc cudaExtMemHandleDesc;
	memset(&cudaExtMemHandleDesc, 0, sizeof(cudaExtMemHandleDesc));
#ifdef _WIN64
	cudaExtMemHandleDesc.type =
		cudaExternalMemoryHandleTypeOpaqueWin32;
	cudaExtMemHandleDesc.handle.win32.handle = image->GetVkImageMemHandle(
		VK_EXTERNAL_MEMORY_HANDLE_TYPE_OPAQUE_WIN32_BIT);
#else
	cudaExtMemHandleDesc.type = cudaExternalMemoryHandleTypeOpaqueFd;

	cudaExtMemHandleDesc.handle.fd =
		image->GetVkImageMemHandle(VK_EXTERNAL_MEMORY_HANDLE_TYPE_OPAQUE_FD_BIT_KHR);
#endif
	VkMemoryRequirements vkMemoryRequirements = {};
        vkGetImageMemoryRequirements(evo_engine::Platform::GetVkDevice(), image->GetVkImage(), &vkMemoryRequirements);
	size_t totalImageMemSize = vkMemoryRequirements.size;
	cudaExtMemHandleDesc.size = totalImageMemSize;

	CUDA_CHECK(ImportExternalMemory(&cudaImage->m_imageExternalMemory,
		&cudaExtMemHandleDesc));

	cudaExternalMemoryMipmappedArrayDesc externalMemoryMipmappedArrayDesc;

	memset(&externalMemoryMipmappedArrayDesc, 0,
		sizeof(externalMemoryMipmappedArrayDesc));
	VkExtent3D imageExtent = image->GetExtent();
	cudaExtent extent = make_cudaExtent(imageExtent.width, imageExtent.height, 0);
	cudaChannelFormatDesc formatDesc;
	formatDesc.x = 32;
	formatDesc.y = 32;
	formatDesc.z = 32;
	formatDesc.w = 32;
	formatDesc.f = cudaChannelFormatKindFloat;

	externalMemoryMipmappedArrayDesc.offset = 0;
	externalMemoryMipmappedArrayDesc.formatDesc = formatDesc;
	externalMemoryMipmappedArrayDesc.extent = extent;
	externalMemoryMipmappedArrayDesc.flags = cudaArrayDefault;
	externalMemoryMipmappedArrayDesc.numLevels = image->GetMipLevels();

	CUDA_CHECK(ExternalMemoryGetMappedMipmappedArray(
		&cudaImage->m_mipmappedImageArray, cudaImage->m_imageExternalMemory,
		&externalMemoryMipmappedArrayDesc));


	for (int mipLevelIdx = 0; mipLevelIdx < image->GetMipLevels(); mipLevelIdx++) {
		cudaArray_t cudaMipLevelArray;
		cudaResourceDesc resourceDesc;

		CUDA_CHECK(GetMipmappedArrayLevel(
			&cudaMipLevelArray, cudaImage->m_mipmappedImageArray, mipLevelIdx));

		memset(&resourceDesc, 0, sizeof(resourceDesc));
		resourceDesc.resType = cudaResourceTypeArray;
		resourceDesc.res.array.array = cudaMipLevelArray;

		cudaSurfaceObject_t surfaceObject;
		CUDA_CHECK(CreateSurfaceObject(&surfaceObject, &resourceDesc));

		cudaImage->m_surfaceObjects.push_back(surfaceObject);
	}

	cudaResourceDesc resDescr;
	memset(&resDescr, 0, sizeof(cudaResourceDesc));

	resDescr.resType = cudaResourceTypeMipmappedArray;
	resDescr.res.mipmap.mipmap = cudaImage->m_mipmappedImageArray;

	cudaTextureDesc texDescr;
	memset(&texDescr, 0, sizeof(cudaTextureDesc));

	texDescr.normalizedCoords = true;
	texDescr.filterMode = cudaFilterModeLinear;
	texDescr.mipmapFilterMode = cudaFilterModeLinear;

	texDescr.addressMode[0] = cudaAddressModeWrap;
	texDescr.addressMode[1] = cudaAddressModeWrap;
	texDescr.addressMode[2] = cudaAddressModeWrap;

	texDescr.maxAnisotropy = evo_engine::Platform::GetVkPhysicalDeviceProperties().limits.maxSamplerAnisotropy;

	texDescr.minMipmapLevelClamp = 0;
	texDescr.maxMipmapLevelClamp = static_cast<float>(image->GetMipLevels() - 1);

	texDescr.readMode = cudaReadModeElementType;

	CUDA_CHECK(CreateTextureObject(&cudaImage->m_textureObject, &resDescr,
		&texDescr, NULL));

	return cudaImage;
}

std::shared_ptr<CudaImage> CudaModule::ImportCubemap(const std::shared_ptr<evo_engine::Cubemap>& cubemap)
{
	auto image = cubemap->GetImage();

	auto cudaImage = std::make_shared<CudaImage>();

	cudaExternalMemoryHandleDesc cudaExtMemHandleDesc;
	memset(&cudaExtMemHandleDesc, 0, sizeof(cudaExtMemHandleDesc));
#ifdef _WIN64
	cudaExtMemHandleDesc.type =
		cudaExternalMemoryHandleTypeOpaqueWin32;
	cudaExtMemHandleDesc.handle.win32.handle = image->GetVkImageMemHandle(
		VK_EXTERNAL_MEMORY_HANDLE_TYPE_OPAQUE_WIN32_BIT);
#else
	cudaExtMemHandleDesc.type = cudaExternalMemoryHandleTypeOpaqueFd;

	cudaExtMemHandleDesc.handle.fd =
		image->GetVkImageMemHandle(VK_EXTERNAL_MEMORY_HANDLE_TYPE_OPAQUE_FD_BIT_KHR);
#endif
	VkMemoryRequirements vkMemoryRequirements = {};
        vkGetImageMemoryRequirements(evo_engine::Platform::GetVkDevice(), image->GetVkImage(), &vkMemoryRequirements);
	size_t totalImageMemSize = vkMemoryRequirements.size;
	cudaExtMemHandleDesc.size = totalImageMemSize;

	CUDA_CHECK(ImportExternalMemory(&cudaImage->m_imageExternalMemory,
		&cudaExtMemHandleDesc));

	cudaExternalMemoryMipmappedArrayDesc externalMemoryMipmappedArrayDesc;

	memset(&externalMemoryMipmappedArrayDesc, 0,
		sizeof(externalMemoryMipmappedArrayDesc));
	VkExtent3D imageExtent = image->GetExtent();
	cudaExtent extent = make_cudaExtent(imageExtent.width, imageExtent.height, 6);
	cudaChannelFormatDesc formatDesc;
	formatDesc.x = 32;
	formatDesc.y = 32;
	formatDesc.z = 32;
	formatDesc.w = 32;
	formatDesc.f = cudaChannelFormatKindFloat;

	externalMemoryMipmappedArrayDesc.offset = 0;
	externalMemoryMipmappedArrayDesc.formatDesc = formatDesc;
	externalMemoryMipmappedArrayDesc.extent = extent;
	externalMemoryMipmappedArrayDesc.flags = cudaArrayCubemap;
	externalMemoryMipmappedArrayDesc.numLevels = image->GetMipLevels();

	CUDA_CHECK(ExternalMemoryGetMappedMipmappedArray(
		&cudaImage->m_mipmappedImageArray, cudaImage->m_imageExternalMemory,
		&externalMemoryMipmappedArrayDesc));


	for (int mipLevelIdx = 0; mipLevelIdx < image->GetMipLevels(); mipLevelIdx++) {
		cudaArray_t cudaMipLevelArray;
		cudaResourceDesc resourceDesc;

		CUDA_CHECK(GetMipmappedArrayLevel(
			&cudaMipLevelArray, cudaImage->m_mipmappedImageArray, mipLevelIdx));

		memset(&resourceDesc, 0, sizeof(resourceDesc));
		resourceDesc.resType = cudaResourceTypeArray;
		resourceDesc.res.array.array = cudaMipLevelArray;

		cudaSurfaceObject_t surfaceObject;
		CUDA_CHECK(CreateSurfaceObject(&surfaceObject, &resourceDesc));

		cudaImage->m_surfaceObjects.push_back(surfaceObject);
	}

	cudaResourceDesc resDescr;
	memset(&resDescr, 0, sizeof(cudaResourceDesc));

	resDescr.resType = cudaResourceTypeMipmappedArray;
	resDescr.res.mipmap.mipmap = cudaImage->m_mipmappedImageArray;

	cudaTextureDesc texDescr;
	memset(&texDescr, 0, sizeof(cudaTextureDesc));

	texDescr.normalizedCoords = true;
	texDescr.filterMode = cudaFilterModeLinear;
	texDescr.mipmapFilterMode = cudaFilterModeLinear;

	texDescr.addressMode[0] = cudaAddressModeWrap;
	texDescr.addressMode[1] = cudaAddressModeWrap;
	texDescr.addressMode[2] = cudaAddressModeWrap;

	texDescr.maxAnisotropy = evo_engine::Platform::GetVkPhysicalDeviceProperties().limits.maxSamplerAnisotropy;

	texDescr.minMipmapLevelClamp = 0;
	texDescr.maxMipmapLevelClamp = static_cast<float>(image->GetMipLevels() - 1);
	texDescr.seamlessCubemap = true;
	texDescr.readMode = cudaReadModeElementType;

	CUDA_CHECK(CreateTextureObject(&cudaImage->m_textureObject, &resDescr,
		&texDescr, NULL));

	return cudaImage;
}

std::shared_ptr<CudaImage> CudaModule::ImportRenderTexture(
	const std::shared_ptr<evo_engine::RenderTexture>& renderTexture)
{
	auto image = renderTexture->GetColorImage();

	auto cudaImage = std::make_shared<CudaImage>();

	cudaExternalMemoryHandleDesc cudaExtMemHandleDesc;
	memset(&cudaExtMemHandleDesc, 0, sizeof(cudaExtMemHandleDesc));
#ifdef _WIN64
	cudaExtMemHandleDesc.type =
		cudaExternalMemoryHandleTypeOpaqueWin32;
	cudaExtMemHandleDesc.handle.win32.handle = image->GetVkImageMemHandle(
		VK_EXTERNAL_MEMORY_HANDLE_TYPE_OPAQUE_WIN32_BIT);
#else
	cudaExtMemHandleDesc.type = cudaExternalMemoryHandleTypeOpaqueFd;

	cudaExtMemHandleDesc.handle.fd =
		image->GetVkImageMemHandle(VK_EXTERNAL_MEMORY_HANDLE_TYPE_OPAQUE_FD_BIT_KHR);
#endif
	VkMemoryRequirements vkMemoryRequirements = {};
        vkGetImageMemoryRequirements(evo_engine::Platform::GetVkDevice(), image->GetVkImage(), &vkMemoryRequirements);
	size_t totalImageMemSize = vkMemoryRequirements.size;
	cudaExtMemHandleDesc.size = totalImageMemSize;

	CUDA_CHECK(ImportExternalMemory(&cudaImage->m_imageExternalMemory,
		&cudaExtMemHandleDesc));

	cudaExternalMemoryMipmappedArrayDesc externalMemoryMipmappedArrayDesc;

	memset(&externalMemoryMipmappedArrayDesc, 0,
		sizeof(externalMemoryMipmappedArrayDesc));
	VkExtent3D imageExtent = image->GetExtent();
	cudaExtent extent = make_cudaExtent(imageExtent.width, imageExtent.height, 0);
	cudaChannelFormatDesc formatDesc;
	formatDesc.x = 32;
	formatDesc.y = 32;
	formatDesc.z = 32;
	formatDesc.w = 32;
	formatDesc.f = cudaChannelFormatKindFloat;

	externalMemoryMipmappedArrayDesc.offset = 0;
	externalMemoryMipmappedArrayDesc.formatDesc = formatDesc;
	externalMemoryMipmappedArrayDesc.extent = extent;
	externalMemoryMipmappedArrayDesc.flags = cudaArrayDefault;
	externalMemoryMipmappedArrayDesc.numLevels = image->GetMipLevels();

	CUDA_CHECK(ExternalMemoryGetMappedMipmappedArray(
		&cudaImage->m_mipmappedImageArray, cudaImage->m_imageExternalMemory,
		&externalMemoryMipmappedArrayDesc));


	for (int mipLevelIdx = 0; mipLevelIdx < image->GetMipLevels(); mipLevelIdx++) {
		cudaArray_t cudaMipLevelArray;
		cudaResourceDesc resourceDesc;

		CUDA_CHECK(GetMipmappedArrayLevel(
			&cudaMipLevelArray, cudaImage->m_mipmappedImageArray, mipLevelIdx));

		memset(&resourceDesc, 0, sizeof(resourceDesc));
		resourceDesc.resType = cudaResourceTypeArray;
		resourceDesc.res.array.array = cudaMipLevelArray;

		cudaSurfaceObject_t surfaceObject;
		CUDA_CHECK(CreateSurfaceObject(&surfaceObject, &resourceDesc));

		cudaImage->m_surfaceObjects.push_back(surfaceObject);
	}

	cudaResourceDesc resDescr;
	memset(&resDescr, 0, sizeof(cudaResourceDesc));

	resDescr.resType = cudaResourceTypeMipmappedArray;
	resDescr.res.mipmap.mipmap = cudaImage->m_mipmappedImageArray;

	cudaTextureDesc texDescr;
	memset(&texDescr, 0, sizeof(cudaTextureDesc));

	texDescr.normalizedCoords = true;
	texDescr.filterMode = cudaFilterModeLinear;
	texDescr.mipmapFilterMode = cudaFilterModeLinear;

	texDescr.addressMode[0] = cudaAddressModeWrap;
	texDescr.addressMode[1] = cudaAddressModeWrap;
	texDescr.addressMode[2] = cudaAddressModeWrap;

	texDescr.maxAnisotropy = evo_engine::Platform::GetVkPhysicalDeviceProperties().limits.maxSamplerAnisotropy;

	texDescr.minMipmapLevelClamp = 0;
	texDescr.maxMipmapLevelClamp = static_cast<float>(image->GetMipLevels() - 1);

	texDescr.readMode = cudaReadModeElementType;

	CUDA_CHECK(CreateTextureObject(&cudaImage->m_textureObject, &resDescr,
		&texDescr, NULL));

	return cudaImage;
}

std::shared_ptr<CudaSemaphore> CudaModule::ImportSemaphore(const std::shared_ptr<evo_engine::Semaphore>& semaphore)
{
	auto cudaSemaphore = std::make_shared<CudaSemaphore>();

	cudaExternalSemaphoreHandleDesc externalSemaphoreHandleDesc;
	memset(&externalSemaphoreHandleDesc, 0,
		sizeof(externalSemaphoreHandleDesc));
#ifdef _WIN64
	externalSemaphoreHandleDesc.type =
		cudaExternalSemaphoreHandleTypeOpaqueWin32;
	externalSemaphoreHandleDesc.handle.win32.handle = semaphore->GetVkSemaphoreHandle(
		VK_EXTERNAL_SEMAPHORE_HANDLE_TYPE_OPAQUE_WIN32_BIT);
#else
	externalSemaphoreHandleDesc.type = cudaExternalSemaphoreHandleTypeOpaqueFd;
	externalSemaphoreHandleDesc.handle.fd = semaphore->GetVkSemaphoreHandle(
		VK_EXTERNAL_SEMAPHORE_HANDLE_TYPE_OPAQUE_FD_BIT);
#endif
	externalSemaphoreHandleDesc.flags = 0;

	CUDA_CHECK(ImportExternalSemaphore(&cudaSemaphore->m_semaphore,
		&externalSemaphoreHandleDesc));

	return cudaSemaphore;
}
