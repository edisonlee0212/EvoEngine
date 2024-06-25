#include <RayTracer.hpp>

#include <optix_function_table_definition.h>
#include <optix_stack_size.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtc/random.hpp>
#include <glm/gtc/type_ptr.hpp>

#define GL_TEXTURE_CUBE_MAP 0x8513

#include <cuda_gl_interop.h>

#include <iostream>

#include <RayDataDefinations.hpp>

#include <functional>

#include <filesystem>

#include <imgui.h>
#include <CUDAModule.hpp>

using namespace evo_engine;

void CameraProperties::Set(const glm::vec3 &position, const glm::quat &rotation) {
  auto newFront = glm::normalize(rotation * glm::vec3(0, 0, -1));
  auto newUp = glm::normalize(rotation * glm::vec3(0, 1, 0));
  const float aspect = static_cast<float>(m_frame.m_size.x) / static_cast<float>(m_frame.m_size.y);
  const auto projection = glm::perspective(glm::radians(m_fov * 0.5f), aspect, 0.1f, 100.f);
  const auto view = glm::lookAt(position, position + newFront, newUp);
  auto inv = glm::inverse(projection * view);
  m_from = position;
  if (inv != m_inverseProjectionView)
    m_modified = true;
  m_inverseProjectionView = inv;

  const float cosFovY = glm::radians(m_fov * 0.5f);

  m_horizontal = cosFovY * aspect * glm::normalize(glm::cross(newFront, newUp));
  m_vertical = cosFovY * glm::normalize(newUp);
}

void CameraProperties::Resize(const glm::uvec2 &newSize) {
  if (m_frame.m_size == newSize)
    return;
  m_frame.m_size = newSize;
  m_modified = true;
  if (m_denoiser) {
    OPTIX_CHECK(optixDenoiserDestroy(m_denoiser));
  };
  // ------------------------------------------------------------------
  // create the denoiser:
  OptixDenoiserOptions denoiserOptions = {};
  OPTIX_CHECK(optixDenoiserCreate(CudaModule::GetRayTracer()->m_optixDeviceContext, OPTIX_DENOISER_MODEL_KIND_LDR,
                                  &denoiserOptions, &m_denoiser));
  // .. then compute and allocate memory resources for the denoiser
  OptixDenoiserSizes denoiserReturnSizes;
  OPTIX_CHECK(
      optixDenoiserComputeMemoryResources(m_denoiser, m_frame.m_size.x, m_frame.m_size.y, &denoiserReturnSizes));

  m_denoiserScratch.Resize(std::max(denoiserReturnSizes.withOverlapScratchSizeInBytes,
                                    denoiserReturnSizes.withoutOverlapScratchSizeInBytes));

  m_denoiserState.Resize(denoiserReturnSizes.stateSizeInBytes);
  // ------------------------------------------------------------------
  // resize our cuda frame buffer
  m_denoisedBuffer.Resize(m_frame.m_size.x * m_frame.m_size.y * sizeof(glm::vec4));
  m_frameBufferColor.Resize(m_frame.m_size.x * m_frame.m_size.y * sizeof(glm::vec4));
  m_frameBufferNormal.Resize(m_frame.m_size.x * m_frame.m_size.y * sizeof(glm::vec4));
  m_frameBufferAlbedo.Resize(m_frame.m_size.x * m_frame.m_size.y * sizeof(glm::vec4));

  // update the launch parameters that we'll pass to the optix
  // launch:
  m_frame.m_colorBuffer = (glm::vec4 *)m_frameBufferColor.DevicePointer();
  m_frame.m_normalBuffer = (glm::vec4 *)m_frameBufferNormal.DevicePointer();
  m_frame.m_albedoBuffer = (glm::vec4 *)m_frameBufferAlbedo.DevicePointer();

  // ------------------------------------------------------------------
  OPTIX_CHECK(optixDenoiserSetup(m_denoiser, 0, m_frame.m_size.x, m_frame.m_size.y, m_denoiserState.DevicePointer(),
                                 m_denoiserState.size_in_bytes, m_denoiserScratch.DevicePointer(),
                                 m_denoiserScratch.size_in_bytes));
}

void CameraProperties::SetFov(float value) {
  m_modified = true;
  m_fov = value;
}

const char *OutputTypes[]{"Color", "Normal", "Albedo", "Depth"};

void CameraProperties::OnInspect() {
  if (ImGui::TreeNode("Camera Properties")) {
    if (ImGui::Checkbox("Accumulate", &m_accumulate)) {
      m_modified = true;
    }
    if (ImGui::DragFloat("Gamma", &m_gamma, 0.01f, 0.1f, 5.0f)) {
      SetGamma(m_gamma);
    }
    int outputType = (int)m_outputType;
    if (ImGui::Combo("Output Type", &outputType, OutputTypes, IM_ARRAYSIZE(OutputTypes))) {
      m_outputType = static_cast<OutputType>(outputType);
      m_modified = true;
    }
    if (ImGui::DragFloat("Max Distance", &m_maxDistance, 0.1f, 0.1f, 10000.0f)) {
      SetMaxDistance(m_maxDistance);
    }

    if (ImGui::DragFloat("FOV", &m_fov, 1.0f, 1, 359)) {
      SetFov(m_fov);
    }
    if (ImGui::DragFloat("Aperture", &m_aperture, 0.0001f, 0.0f, 99999.0f, "%.4f")) {
      SetAperture(m_aperture);
    }
    if (ImGui::DragFloat("Focal Length", &m_focalLength, 0.0001f, 0.0f, 99999.0f, "%.4f")) {
      SetFocalLength(m_focalLength);
    }
    if (ImGui::DragFloat("Denoiser Strength", &m_denoiserStrength, 0.01f, 0.0f, 1.0f)) {
      SetDenoiserStrength(m_denoiserStrength);
    }
    ImGui::TreePop();
  }
}

void CameraProperties::SetDenoiserStrength(float value) {
  m_denoiserStrength = glm::clamp(value, 0.0f, 1.0f);
  m_modified = true;
}

void CameraProperties::SetGamma(float value) {
  m_modified = true;
  m_gamma = value;
}

void CameraProperties::SetOutputType(OutputType value) {
  m_modified = true;
  m_outputType = value;
}

void CameraProperties::SetAperture(float value) {
  m_modified = true;
  m_aperture = value;
}

void CameraProperties::SetFocalLength(float value) {
  m_modified = true;
  m_focalLength = value;
}

void CameraProperties::SetMaxDistance(float value) {
  m_maxDistance = value;
  m_modified = true;
}

const char *EnvironmentalLightingTypes[]{"Scene", "Skydome", "SingleLightSource"};

void EnvironmentProperties::OnInspect() {
  static int type = 0;
  if (ImGui::Combo("Environment Lighting", &type, EnvironmentalLightingTypes,
                   IM_ARRAYSIZE(EnvironmentalLightingTypes))) {
    m_environmentalLightingType = static_cast<EnvironmentalLightingType>(type);
  }
  if (m_environmentalLightingType == EnvironmentalLightingType::Skydome) {
    if (ImGui::TreeNodeEx("Atmosphere Settings", ImGuiTreeNodeFlags_DefaultOpen)) {
      if (ImGui::DragFloat("Earth Radius (km)", &m_atmosphere.m_earthRadius, 1.0f, 0.0f,
                           m_atmosphere.m_atmosphereRadius - 1.0f)) {
        m_atmosphere.m_earthRadius =
            glm::clamp(m_atmosphere.m_earthRadius, 1.0f, m_atmosphere.m_atmosphereRadius - 1.0f);
      }
      if (ImGui::DragFloat("Atmosphere Radius (km)", &m_atmosphere.m_atmosphereRadius, 1.0f,
                           m_atmosphere.m_earthRadius + 1.0f, 100000.0f)) {
        m_atmosphere.m_atmosphereRadius =
            glm::clamp(m_atmosphere.m_atmosphereRadius, m_atmosphere.m_earthRadius + 1.0f, 100000.0f);
      }
      if (ImGui::DragFloat("Rayleigh scale height (m)", &m_atmosphere.m_Hr, 1.0f, 0.0f, 100000.0f)) {
        m_atmosphere.m_Hr = glm::clamp(m_atmosphere.m_Hr, 0.0f, 10000.0f);
      }
      if (ImGui::DragFloat("Mie scale height (m)", &m_atmosphere.m_Hm, 1.0f, 0.0f, 100000.0f)) {
        m_atmosphere.m_Hm = glm::clamp(m_atmosphere.m_Hm, 0.0f, 10000.0f);
      }
      if (ImGui::DragFloat("Mie scattering mean cosine", &m_atmosphere.m_g, 0.001f, 0.0f, 0.999f, "%.4f")) {
        m_atmosphere.m_g = glm::clamp(m_atmosphere.m_g, 0.0f, 0.999f);
      }
      if (ImGui::DragInt("Samples", &m_atmosphere.m_numSamples, 1, 128)) {
        m_atmosphere.m_numSamples = glm::clamp(m_atmosphere.m_numSamples, 1, 128);
      }
      if (ImGui::DragInt("Samples light", &m_atmosphere.m_numSamplesLight, 1, 128)) {
        m_atmosphere.m_numSamplesLight = glm::clamp(m_atmosphere.m_numSamplesLight, 1, 128);
      }
      static glm::vec3 angles = glm::vec3(90, 0, 0);
      if (ImGui::DragFloat3("Sun angle", &angles.x, 1.0f)) {
        m_sunDirection = glm::quat(glm::radians(angles)) * glm::vec3(0, 0, -1);
      }
      ImGui::TreePop();
    }
    if (ImGui::Button("Reset Atmosphere")) {
      m_atmosphere.m_earthRadius = 6360;       // In the paper this is usually Rg or Re (radius ground, eart)
      m_atmosphere.m_atmosphereRadius = 6420;  // In the paper this is usually R or Ra (radius atmosphere)
      m_atmosphere.m_Hr = 7994;                // Thickness of the atmosphere if density was uniform (Hr)
      m_atmosphere.m_Hm = 1200;                // Same as above but for Mie scattering (Hm)
      m_atmosphere.m_g = 0.76f;                // Mean cosine for Mie scattering
      m_atmosphere.m_numSamples = 16;
      m_atmosphere.m_numSamplesLight = 8;
    }
  } else if (m_environmentalLightingType == EnvironmentalLightingType::SingleLightSource) {
    if (ImGui::DragFloat("Light Size", &m_lightSize, 0.001f, 0.0f, 1.0f)) {
      m_lightSize = glm::clamp(m_lightSize, 0.0f, 1.0f);
    }
    if (ImGui::DragFloat("Ambient light intensity", &m_ambientLightIntensity, 0.001f, 0.0f, 1.0f)) {
      m_ambientLightIntensity = glm::clamp(m_ambientLightIntensity, 0.0f, 1.0f);
    }
    static glm::vec3 angles = glm::vec3(90, 0, 0);
    if (ImGui::DragFloat3("Sun angle", &angles.x, 1.0f)) {
      m_sunDirection = glm::quat(glm::radians(angles)) * glm::vec3(0, 0, -1);
    }
  }
}

void CameraProperties::SetSkybox(const std::shared_ptr<CudaImage> &cubemap) {
  m_skybox = cubemap->m_textureObject;
}

void RayProperties::OnInspect() {
  if (ImGui::TreeNode("Ray Properties")) {
    ImGui::DragInt("bounce limit", &m_bounces, 1, 1, 8);
    ImGui::DragInt("pixel samples", &m_samples, 1, 1, 64);
    ImGui::TreePop();
  }
}

void RayTracerProperties::OnInspect() {
  m_environment.OnInspect();
  m_rayProperties.OnInspect();
}

bool RayTracer::RenderToCamera(const EnvironmentProperties &environmentProperties, CameraProperties &cameraProperties,
                               const RayProperties &rayProperties) {
  if (cameraProperties.m_frame.m_size.x == 0 | cameraProperties.m_frame.m_size.y == 0)
    return true;
  if (!m_hasAccelerationStructure)
    return false;
  BuildSBT();
  bool statusChanged = false;
  if (m_sceneModified)
    statusChanged = true;
  m_cameraRenderingLaunchParams.m_cameraProperties = cameraProperties;
  statusChanged = statusChanged || cameraProperties.m_modified;
  cameraProperties.m_modified = false;
  if (m_cameraRenderingLaunchParams.m_rayTracerProperties.m_environment.Changed(environmentProperties)) {
    m_cameraRenderingLaunchParams.m_rayTracerProperties.m_environment = environmentProperties;
    statusChanged = true;
  }
  if (m_cameraRenderingLaunchParams.m_rayTracerProperties.m_rayProperties.Changed(rayProperties)) {
    m_cameraRenderingLaunchParams.m_rayTracerProperties.m_rayProperties = rayProperties;
    statusChanged = true;
  }
  if (!m_cameraRenderingLaunchParams.m_cameraProperties.m_accumulate || statusChanged) {
    m_cameraRenderingLaunchParams.m_cameraProperties.m_frame.m_frameId = 0;
    cameraProperties.m_frame.m_frameId = 0;
  }
#pragma region Upload parameters
  m_cameraRenderingPipeline.m_launchParamsBuffer.Upload(&m_cameraRenderingLaunchParams, 1);
  m_cameraRenderingLaunchParams.m_cameraProperties.m_frame.m_frameId++;
  cameraProperties.m_frame.m_frameId++;
#pragma endregion
#pragma region Launch rays from camera
  OPTIX_CHECK(optixLaunch(/*! pipeline we're launching launch: */
                          m_cameraRenderingPipeline.m_pipeline, m_stream,
                          /*! parameters and SBT */
                          m_cameraRenderingPipeline.m_launchParamsBuffer.DevicePointer(),
                          m_cameraRenderingPipeline.m_launchParamsBuffer.size_in_bytes,
                          &m_cameraRenderingPipeline.m_sbt,
                          /*! dimensions of the launch: */
                          m_cameraRenderingLaunchParams.m_cameraProperties.m_frame.m_size.x,
                          m_cameraRenderingLaunchParams.m_cameraProperties.m_frame.m_size.y, 1));
#pragma endregion
  CUDA_SYNC_CHECK();
#pragma region Bind output texture
  cudaArray_t outputArray;
  CUDA_CHECK(GetMipmappedArrayLevel(
      &outputArray, m_cameraRenderingLaunchParams.m_cameraProperties.m_image->m_mipmappedImageArray, 0));
#pragma endregion
#pragma region Copy results to output texture
  OptixImage2D inputLayer[3];
  inputLayer[0].data = m_cameraRenderingLaunchParams.m_cameraProperties.m_frameBufferColor.DevicePointer();
  /// Width of the image (in pixels)
  inputLayer[0].width = m_cameraRenderingLaunchParams.m_cameraProperties.m_frame.m_size.x;
  /// Height of the image (in pixels)
  inputLayer[0].height = m_cameraRenderingLaunchParams.m_cameraProperties.m_frame.m_size.y;
  /// Stride between subsequent rows of the image (in bytes).
  inputLayer[0].rowStrideInBytes =
      m_cameraRenderingLaunchParams.m_cameraProperties.m_frame.m_size.x * sizeof(glm::vec4);
  /// Stride between subsequent pixels of the image (in bytes).
  /// For now, only 0 or the value that corresponds to a dense packing of pixels
  /// (no gaps) is supported.
  inputLayer[0].pixelStrideInBytes = sizeof(glm::vec4);
  /// Pixel format.
  inputLayer[0].format = OPTIX_PIXEL_FORMAT_FLOAT4;

  // ..................................................................
  inputLayer[1].data = m_cameraRenderingLaunchParams.m_cameraProperties.m_frameBufferAlbedo.DevicePointer();
  /// Width of the image (in pixels)
  inputLayer[1].width = m_cameraRenderingLaunchParams.m_cameraProperties.m_frame.m_size.x;
  /// Height of the image (in pixels)
  inputLayer[1].height = m_cameraRenderingLaunchParams.m_cameraProperties.m_frame.m_size.y;
  /// Stride between subsequent rows of the image (in bytes).
  inputLayer[1].rowStrideInBytes =
      m_cameraRenderingLaunchParams.m_cameraProperties.m_frame.m_size.x * sizeof(glm::vec4);
  /// Stride between subsequent pixels of the image (in bytes).
  /// For now, only 0 or the value that corresponds to a dense packing of pixels
  /// (no gaps) is supported.
  inputLayer[1].pixelStrideInBytes = sizeof(glm::vec4);
  /// Pixel format.
  inputLayer[1].format = OPTIX_PIXEL_FORMAT_FLOAT4;

  // ..................................................................
  inputLayer[2].data = m_cameraRenderingLaunchParams.m_cameraProperties.m_frameBufferNormal.DevicePointer();
  /// Width of the image (in pixels)
  inputLayer[2].width = m_cameraRenderingLaunchParams.m_cameraProperties.m_frame.m_size.x;
  /// Height of the image (in pixels)
  inputLayer[2].height = m_cameraRenderingLaunchParams.m_cameraProperties.m_frame.m_size.y;
  /// Stride between subsequent rows of the image (in bytes).
  inputLayer[2].rowStrideInBytes =
      m_cameraRenderingLaunchParams.m_cameraProperties.m_frame.m_size.x * sizeof(glm::vec4);
  /// Stride between subsequent pixels of the image (in bytes).
  /// For now, only 0 or the value that corresponds to a dense packing of pixels
  /// (no gaps) is supported.
  inputLayer[2].pixelStrideInBytes = sizeof(glm::vec4);
  /// Pixel format.
  inputLayer[2].format = OPTIX_PIXEL_FORMAT_FLOAT4;

  // -------------------------------------------------------
  OptixImage2D outputLayer;
  outputLayer.data = m_cameraRenderingLaunchParams.m_cameraProperties.m_denoisedBuffer.DevicePointer();
  /// Width of the image (in pixels)
  outputLayer.width = m_cameraRenderingLaunchParams.m_cameraProperties.m_frame.m_size.x;
  /// Height of the image (in pixels)
  outputLayer.height = m_cameraRenderingLaunchParams.m_cameraProperties.m_frame.m_size.y;
  /// Stride between subsequent rows of the image (in bytes).
  outputLayer.rowStrideInBytes = m_cameraRenderingLaunchParams.m_cameraProperties.m_frame.m_size.x * sizeof(glm::vec4);
  /// Stride between subsequent pixels of the image (in bytes).
  /// For now, only 0 or the value that corresponds to a dense packing of pixels
  /// (no gaps) is supported.
  outputLayer.pixelStrideInBytes = sizeof(glm::vec4);
  /// Pixel format.
  outputLayer.format = OPTIX_PIXEL_FORMAT_FLOAT4;

  switch (m_cameraRenderingLaunchParams.m_cameraProperties.m_outputType) {
    case OutputType::Color: {
      if (cameraProperties.m_denoiserStrength == 0.0f) {
        CUDA_CHECK(MemcpyToArray(outputArray, 0, 0,
                                 (void *)m_cameraRenderingLaunchParams.m_cameraProperties.m_frame.m_colorBuffer,
                                 sizeof(glm::vec4) * m_cameraRenderingLaunchParams.m_cameraProperties.m_frame.m_size.x *
                                     m_cameraRenderingLaunchParams.m_cameraProperties.m_frame.m_size.y,
                                 cudaMemcpyDeviceToDevice));
      } else {
        OptixDenoiserParams denoiserParams;
        m_cameraRenderingLaunchParams.m_cameraProperties.m_denoiserIntensity.Resize(sizeof(float));
        if (m_cameraRenderingLaunchParams.m_cameraProperties.m_denoiserIntensity.size_in_bytes != sizeof(float))
          m_cameraRenderingLaunchParams.m_cameraProperties.m_denoiserIntensity.Resize(sizeof(float));
        denoiserParams.hdrIntensity =
            m_cameraRenderingLaunchParams.m_cameraProperties.m_denoiserIntensity.DevicePointer();
        if (m_cameraRenderingLaunchParams.m_cameraProperties.m_accumulate &&
            m_cameraRenderingLaunchParams.m_cameraProperties.m_frame.m_frameId > 1)
          denoiserParams.blendFactor = (1.0f - cameraProperties.m_denoiserStrength) /
                                       m_cameraRenderingLaunchParams.m_cameraProperties.m_frame.m_frameId;
        else
          denoiserParams.blendFactor = (1.0f - cameraProperties.m_denoiserStrength);

        OPTIX_CHECK(optixDenoiserComputeIntensity(
            m_cameraRenderingLaunchParams.m_cameraProperties.m_denoiser,
            /*stream*/ 0, &inputLayer[0],
            (CUdeviceptr)m_cameraRenderingLaunchParams.m_cameraProperties.m_denoiserIntensity.DevicePointer(),
            (CUdeviceptr)m_cameraRenderingLaunchParams.m_cameraProperties.m_denoiserScratch.DevicePointer(),
            m_cameraRenderingLaunchParams.m_cameraProperties.m_denoiserScratch.size_in_bytes));

        OptixDenoiserLayer denoiserLayer = {};
        denoiserLayer.input = inputLayer[0];
        denoiserLayer.output = outputLayer;

        OptixDenoiserGuideLayer denoiserGuideLayer = {};
        denoiserGuideLayer.albedo = inputLayer[1];
        denoiserGuideLayer.normal = inputLayer[2];

        OPTIX_CHECK(optixDenoiserInvoke(
            m_cameraRenderingLaunchParams.m_cameraProperties.m_denoiser,
            /*stream*/ 0, &denoiserParams,
            m_cameraRenderingLaunchParams.m_cameraProperties.m_denoiserState.DevicePointer(),
            m_cameraRenderingLaunchParams.m_cameraProperties.m_denoiserState.size_in_bytes, &denoiserGuideLayer,
            &denoiserLayer, 1,
            /*inputOffsetX*/ 0,
            /*inputOffsetY*/ 0, m_cameraRenderingLaunchParams.m_cameraProperties.m_denoiserScratch.DevicePointer(),
            m_cameraRenderingLaunchParams.m_cameraProperties.m_denoiserScratch.size_in_bytes));
        CUDA_CHECK(MemcpyToArray(outputArray, 0, 0, (void *)outputLayer.data,
                                 sizeof(glm::vec4) * m_cameraRenderingLaunchParams.m_cameraProperties.m_frame.m_size.x *
                                     m_cameraRenderingLaunchParams.m_cameraProperties.m_frame.m_size.y,
                                 cudaMemcpyDeviceToDevice));
      }
    } break;
    case OutputType::Normal: {
      CUDA_CHECK(MemcpyToArray(outputArray, 0, 0,
                               (void *)m_cameraRenderingLaunchParams.m_cameraProperties.m_frame.m_normalBuffer,
                               sizeof(glm::vec4) * m_cameraRenderingLaunchParams.m_cameraProperties.m_frame.m_size.x *
                                   m_cameraRenderingLaunchParams.m_cameraProperties.m_frame.m_size.y,
                               cudaMemcpyDeviceToDevice));
    } break;
    case OutputType::Albedo: {
      CUDA_CHECK(MemcpyToArray(outputArray, 0, 0,
                               (void *)m_cameraRenderingLaunchParams.m_cameraProperties.m_frame.m_albedoBuffer,
                               sizeof(glm::vec4) * m_cameraRenderingLaunchParams.m_cameraProperties.m_frame.m_size.x *
                                   m_cameraRenderingLaunchParams.m_cameraProperties.m_frame.m_size.y,
                               cudaMemcpyDeviceToDevice));
    } break;
    case OutputType::Depth: {
      CUDA_CHECK(MemcpyToArray(outputArray, 0, 0,
                               (void *)m_cameraRenderingLaunchParams.m_cameraProperties.m_frame.m_albedoBuffer,
                               sizeof(glm::vec4) * m_cameraRenderingLaunchParams.m_cameraProperties.m_frame.m_size.x *
                                   m_cameraRenderingLaunchParams.m_cameraProperties.m_frame.m_size.y,
                               cudaMemcpyDeviceToDevice));
    } break;
  }

#pragma endregion
  return true;
}

void RayTracer::EstimateIllumination(const size_t &size, const EnvironmentProperties &environmentProperties,
                                     const RayProperties &rayProperties, CudaBuffer &lightProbes, unsigned seed,
                                     float pushNormalDistance) {
  if (!m_hasAccelerationStructure)
    return;
  if (size == 0) {
    std::cout << "Error: Lightprobe is empty" << std::endl;
    return;
  }
  BuildSBT();

  m_illuminationEstimationLaunchParams.m_rayTracerProperties.m_environment = environmentProperties;
  m_illuminationEstimationLaunchParams.m_rayTracerProperties.m_rayProperties = rayProperties;
#pragma region Upload parameters
  m_illuminationEstimationLaunchParams.m_seed = seed;
  m_illuminationEstimationLaunchParams.m_pushNormalDistance = pushNormalDistance;
  m_illuminationEstimationLaunchParams.m_size = size;
  m_illuminationEstimationLaunchParams.m_rayTracerProperties.m_environment = environmentProperties;
  m_illuminationEstimationLaunchParams.m_rayTracerProperties.m_rayProperties = rayProperties;
  m_illuminationEstimationLaunchParams.m_lightProbes =
      reinterpret_cast<IlluminationSampler<glm::vec3> *>(lightProbes.DevicePointer());
  m_illuminationEstimationPipeline.m_launchParamsBuffer.Upload(&m_illuminationEstimationLaunchParams, 1);
#pragma endregion
#pragma region Launch rays from light probes
  OPTIX_CHECK(optixLaunch(/*! pipeline we're launching launch: */
                          m_illuminationEstimationPipeline.m_pipeline, m_stream,
                          /*! parameters and SBT */
                          m_illuminationEstimationPipeline.m_launchParamsBuffer.DevicePointer(),
                          m_illuminationEstimationPipeline.m_launchParamsBuffer.size_in_bytes,
                          &m_illuminationEstimationPipeline.m_sbt,
                          /*! dimensions of the launch: */
                          size, 1, 1));
  CUDA_SYNC_CHECK();
#pragma endregion
}

void RayTracer::ScanPointCloud(const size_t &size, const EnvironmentProperties &environmentProperties,
                               CudaBuffer &samples) {
  if (!m_hasAccelerationStructure)
    return;
  if (size == 0) {
    std::cout << "Error: Samples is empty" << std::endl;
    return;
  }
  BuildSBT();
#pragma region Upload parameters
  m_pointCloudScanningLaunchParams.m_size = size;
  m_pointCloudScanningLaunchParams.m_samples = reinterpret_cast<PointCloudSample *>(samples.DevicePointer());
  m_pointCloudScanningPipeline.m_launchParamsBuffer.Upload(&m_pointCloudScanningLaunchParams, 1);
#pragma endregion
#pragma region Launch rays from samples
  OPTIX_CHECK(optixLaunch(/*! pipeline we're launching launch: */
                          m_pointCloudScanningPipeline.m_pipeline, m_stream,
                          /*! parameters and SBT */
                          m_pointCloudScanningPipeline.m_launchParamsBuffer.DevicePointer(),
                          m_pointCloudScanningPipeline.m_launchParamsBuffer.size_in_bytes,
                          &m_pointCloudScanningPipeline.m_sbt,
                          /*! dimensions of the launch: */
                          size, 1, 1));
  CUDA_SYNC_CHECK();
#pragma endregion
}

RayTracer::RayTracer() {
  m_cameraRenderingLaunchParams.m_cameraProperties.m_frame.m_frameId = 0;
  // std::cout << "#Optix: creating optix context ..." << std::endl;
  CreateContext();
  // std::cout << "#Optix: setting up module ..." << std::endl;
  CreateModules();
  // std::cout << "#Optix: creating raygen programs ..." << std::endl;
  CreateRayGenPrograms();
  // std::cout << "#Optix: creating miss programs ..." << std::endl;
  CreateMissPrograms();
  // std::cout << "#Optix: creating hitgroup programs ..." << std::endl;
  CreateHitGroupPrograms();
  // std::cout << "#Optix: setting up optix pipeline ..." << std::endl;
  AssemblePipelines();

  std::cout << "#Optix: context, module, pipeline, etc, all set up ..." << std::endl;
}

static void context_log_cb(const unsigned int level, const char *tag, const char *message, void *) {
  fprintf(stderr, "[%2d][%12s]: %s\n", static_cast<int>(level), tag, message);
}

void printLogMessage(unsigned int level, const char *tag, const char *message, void * /* cbdata */) {
  std::cerr << "[" << std::setw(2) << level << "][" << std::setw(12) << tag << "]: " << message << std::endl;
}

void RayTracer::CreateContext() {
  // for this sample, do everything on one device
  const int deviceID = 0;
  CUDA_CHECK(StreamCreate(&m_stream));
  CUDA_CHECK(GetDeviceProperties(&m_deviceProps, deviceID));
  std::cout << "#Optix: running on device: " << m_deviceProps.name << std::endl;
  const CUresult cuRes = cuCtxGetCurrent(&m_cudaContext);
  if (cuRes != CUDA_SUCCESS)
    fprintf(stderr, "Error querying current context: error code %d\n", cuRes);

  OptixDeviceContextOptions options = {};
  options.logCallbackFunction = &printLogMessage;
  options.logCallbackLevel = 4;
  // options.validationMode = OPTIX_DEVICE_CONTEXT_VALIDATION_MODE_ALL;

  OPTIX_CHECK(optixDeviceContextCreate(m_cudaContext, &options, &m_optixDeviceContext));
  OPTIX_CHECK(optixDeviceContextSetLogCallback(m_optixDeviceContext, context_log_cb, nullptr, 4));
}

extern "C" char CAMERA_RENDERING_PTX[];
extern "C" char ILLUMINATION_ESTIMATION_PTX[];
extern "C" char POINT_CLOUD_SCANNING_PTX[];

void RayTracer::CreateModules() {
  CreateModule(m_cameraRenderingPipeline, CAMERA_RENDERING_PTX, "cameraRenderingLaunchParams");
  CreateModule(m_illuminationEstimationPipeline, ILLUMINATION_ESTIMATION_PTX, "illuminationEstimationLaunchParams");
  CreateModule(m_pointCloudScanningPipeline, POINT_CLOUD_SCANNING_PTX, "pointCloudScanningLaunchParams");
}

void RayTracer::CreateRayGenPrograms() {
  CreateRayGenProgram(m_cameraRenderingPipeline, "__raygen__CR");
  CreateRayGenProgram(m_illuminationEstimationPipeline, "__raygen__IE");
  CreateRayGenProgram(m_pointCloudScanningPipeline, "__raygen__PCS");
}

void RayTracer::CreateMissPrograms() {
  {
    char log[2048];
    size_t sizeofLog = sizeof(log);

    OptixProgramGroupOptions pgOptions = {};
    OptixProgramGroupDesc pgDesc = {};
    pgDesc.kind = OPTIX_PROGRAM_GROUP_KIND_MISS;
    pgDesc.miss.module = m_cameraRenderingPipeline.m_module;

    // ------------------------------------------------------------------
    // radiance rays
    // ------------------------------------------------------------------
    pgDesc.miss.entryFunctionName = "__miss__CR_R";

    OPTIX_CHECK(optixProgramGroupCreate(m_optixDeviceContext, &pgDesc, 1, &pgOptions, log, &sizeofLog,
                                        &m_cameraRenderingPipeline.m_missProgramGroups[RayType::Radiance]));
    if (sizeofLog > 1)
      std::cout << log << std::endl;
    // ------------------------------------------------------------------
    // BSSRDF Spatial sampler rays
    // ------------------------------------------------------------------
    pgDesc.miss.entryFunctionName = "__miss__CR_SS";
    OPTIX_CHECK(optixProgramGroupCreate(m_optixDeviceContext, &pgDesc, 1, &pgOptions, log, &sizeofLog,
                                        &m_cameraRenderingPipeline.m_missProgramGroups[RayType::SpacialSampling]));
    if (sizeofLog > 1)
      std::cout << log << std::endl;
  }
  {
    char log[2048];
    size_t sizeofLog = sizeof(log);

    OptixProgramGroupOptions pgOptions = {};
    OptixProgramGroupDesc pgDesc = {};
    pgDesc.kind = OPTIX_PROGRAM_GROUP_KIND_MISS;
    pgDesc.miss.module = m_illuminationEstimationPipeline.m_module;

    // ------------------------------------------------------------------
    // radiance rays
    // ------------------------------------------------------------------
    pgDesc.miss.entryFunctionName = "__miss__IE_R";

    OPTIX_CHECK(optixProgramGroupCreate(m_optixDeviceContext, &pgDesc, 1, &pgOptions, log, &sizeofLog,
                                        &m_illuminationEstimationPipeline.m_missProgramGroups[RayType::Radiance]));
    if (sizeofLog > 1)
      std::cout << log << std::endl;
    // ------------------------------------------------------------------
    // BSSRDF Spatial sampler rays
    // ------------------------------------------------------------------
    pgDesc.miss.entryFunctionName = "__miss__IE_SS";
    OPTIX_CHECK(
        optixProgramGroupCreate(m_optixDeviceContext, &pgDesc, 1, &pgOptions, log, &sizeofLog,
                                &m_illuminationEstimationPipeline.m_missProgramGroups[RayType::SpacialSampling]));
    if (sizeofLog > 1)
      std::cout << log << std::endl;
  }
  {
    char log[2048];
    size_t sizeofLog = sizeof(log);

    OptixProgramGroupOptions pgOptions = {};
    OptixProgramGroupDesc pgDesc = {};
    pgDesc.kind = OPTIX_PROGRAM_GROUP_KIND_MISS;
    pgDesc.miss.module = m_pointCloudScanningPipeline.m_module;

    // ------------------------------------------------------------------
    // radiance rays
    // ------------------------------------------------------------------
    pgDesc.miss.entryFunctionName = "__miss__PCS_R";

    OPTIX_CHECK(optixProgramGroupCreate(m_optixDeviceContext, &pgDesc, 1, &pgOptions, log, &sizeofLog,
                                        &m_pointCloudScanningPipeline.m_missProgramGroups[RayType::Radiance]));
    if (sizeofLog > 1)
      std::cout << log << std::endl;
    // ------------------------------------------------------------------
    // BSSRDF Spatial sampler rays
    // ------------------------------------------------------------------
    pgDesc.miss.entryFunctionName = "__miss__PCS_SS";
    OPTIX_CHECK(optixProgramGroupCreate(m_optixDeviceContext, &pgDesc, 1, &pgOptions, log, &sizeofLog,
                                        &m_pointCloudScanningPipeline.m_missProgramGroups[RayType::SpacialSampling]));
    if (sizeofLog > 1)
      std::cout << log << std::endl;
  }
}

void RayTracer::CreateHitGroupPrograms() {
  {
    char log[2048];
    size_t sizeofLog = sizeof(log);

    OptixProgramGroupOptions pgOptions = {};
    OptixProgramGroupDesc pgDesc = {};
    pgDesc.kind = OPTIX_PROGRAM_GROUP_KIND_HITGROUP;
    pgDesc.hitgroup.moduleCH = m_cameraRenderingPipeline.m_module;
    pgDesc.hitgroup.moduleAH = m_cameraRenderingPipeline.m_module;

    // -------------------------------------------------------
    // radiance rays
    // -------------------------------------------------------
    pgDesc.hitgroup.entryFunctionNameCH = "__closesthit__CR_R";
    pgDesc.hitgroup.entryFunctionNameAH = "__anyhit__CR_R";
    pgDesc.hitgroup.entryFunctionNameIS = 0;
    OPTIX_CHECK(optixProgramGroupCreate(
        m_optixDeviceContext, &pgDesc, 1, &pgOptions, log, &sizeofLog,
        &m_cameraRenderingPipeline.m_hitGroupProgramGroups[RayType::Radiance][PrimitiveType::Triangle]));

    pgDesc.hitgroup.moduleIS = m_cameraRenderingPipeline.m_linearCurveModule;
    OPTIX_CHECK(optixProgramGroupCreate(
        m_optixDeviceContext, &pgDesc, 1, &pgOptions, log, &sizeofLog,
        &m_cameraRenderingPipeline.m_hitGroupProgramGroups[RayType::Radiance][PrimitiveType::Linear]));

    pgDesc.hitgroup.moduleIS = m_cameraRenderingPipeline.m_quadraticCurveModule;
    OPTIX_CHECK(optixProgramGroupCreate(
        m_optixDeviceContext, &pgDesc, 1, &pgOptions, log, &sizeofLog,
        &m_cameraRenderingPipeline.m_hitGroupProgramGroups[RayType::Radiance][PrimitiveType::QuadraticBSpline]));

    pgDesc.hitgroup.moduleIS = m_cameraRenderingPipeline.m_cubicCurveModule;
    OPTIX_CHECK(optixProgramGroupCreate(
        m_optixDeviceContext, &pgDesc, 1, &pgOptions, log, &sizeofLog,
        &m_cameraRenderingPipeline.m_hitGroupProgramGroups[RayType::Radiance][PrimitiveType::CubicBSpline]));

    if (sizeofLog > 1)
      std::cout << log << std::endl;

    // -------------------------------------------------------
    // BSSRDF Sampler ray
    // -------------------------------------------------------
    pgDesc.hitgroup.entryFunctionNameCH = "__closesthit__CR_SS";
    pgDesc.hitgroup.entryFunctionNameAH = "__anyhit__CR_SS";
    OPTIX_CHECK(optixProgramGroupCreate(
        m_optixDeviceContext, &pgDesc, 1, &pgOptions, log, &sizeofLog,
        &m_cameraRenderingPipeline.m_hitGroupProgramGroups[RayType::SpacialSampling][PrimitiveType::Triangle]));

    pgDesc.hitgroup.moduleIS = m_cameraRenderingPipeline.m_linearCurveModule;
    OPTIX_CHECK(optixProgramGroupCreate(
        m_optixDeviceContext, &pgDesc, 1, &pgOptions, log, &sizeofLog,
        &m_cameraRenderingPipeline.m_hitGroupProgramGroups[RayType::SpacialSampling][PrimitiveType::Linear]));
    ;

    pgDesc.hitgroup.moduleIS = m_cameraRenderingPipeline.m_quadraticCurveModule;
    OPTIX_CHECK(optixProgramGroupCreate(
        m_optixDeviceContext, &pgDesc, 1, &pgOptions, log, &sizeofLog,
        &m_cameraRenderingPipeline.m_hitGroupProgramGroups[RayType::SpacialSampling][PrimitiveType::QuadraticBSpline]));

    pgDesc.hitgroup.moduleIS = m_cameraRenderingPipeline.m_cubicCurveModule;
    OPTIX_CHECK(optixProgramGroupCreate(
        m_optixDeviceContext, &pgDesc, 1, &pgOptions, log, &sizeofLog,
        &m_cameraRenderingPipeline.m_hitGroupProgramGroups[RayType::SpacialSampling][PrimitiveType::CubicBSpline]));

    if (sizeofLog > 1)
      std::cout << log << std::endl;
  }
  {
    char log[2048];
    size_t sizeofLog = sizeof(log);

    OptixProgramGroupOptions pgOptions = {};
    OptixProgramGroupDesc pgDesc = {};
    pgDesc.kind = OPTIX_PROGRAM_GROUP_KIND_HITGROUP;
    pgDesc.hitgroup.moduleCH = m_illuminationEstimationPipeline.m_module;
    pgDesc.hitgroup.moduleAH = m_illuminationEstimationPipeline.m_module;
    // -------------------------------------------------------
    // radiance rays
    // -------------------------------------------------------
    pgDesc.hitgroup.entryFunctionNameCH = "__closesthit__IE_R";
    pgDesc.hitgroup.entryFunctionNameAH = "__anyhit__IE_R";
    OPTIX_CHECK(optixProgramGroupCreate(
        m_optixDeviceContext, &pgDesc, 1, &pgOptions, log, &sizeofLog,
        &m_illuminationEstimationPipeline.m_hitGroupProgramGroups[RayType::Radiance][PrimitiveType::Triangle]));

    pgDesc.hitgroup.moduleIS = m_illuminationEstimationPipeline.m_linearCurveModule;
    OPTIX_CHECK(optixProgramGroupCreate(
        m_optixDeviceContext, &pgDesc, 1, &pgOptions, log, &sizeofLog,
        &m_illuminationEstimationPipeline.m_hitGroupProgramGroups[RayType::Radiance][PrimitiveType::Linear]));

    pgDesc.hitgroup.moduleIS = m_illuminationEstimationPipeline.m_quadraticCurveModule;
    OPTIX_CHECK(optixProgramGroupCreate(
        m_optixDeviceContext, &pgDesc, 1, &pgOptions, log, &sizeofLog,
        &m_illuminationEstimationPipeline.m_hitGroupProgramGroups[RayType::Radiance][PrimitiveType::QuadraticBSpline]));

    pgDesc.hitgroup.moduleIS = m_illuminationEstimationPipeline.m_cubicCurveModule;
    OPTIX_CHECK(optixProgramGroupCreate(
        m_optixDeviceContext, &pgDesc, 1, &pgOptions, log, &sizeofLog,
        &m_illuminationEstimationPipeline.m_hitGroupProgramGroups[RayType::Radiance][PrimitiveType::CubicBSpline]));
    if (sizeofLog > 1)
      std::cout << log << std::endl;
    // -------------------------------------------------------
    // BSSRDF Sampler ray
    // -------------------------------------------------------
    pgDesc.hitgroup.entryFunctionNameCH = "__closesthit__IE_SS";
    pgDesc.hitgroup.entryFunctionNameAH = "__anyhit__IE_SS";
    OPTIX_CHECK(optixProgramGroupCreate(
        m_optixDeviceContext, &pgDesc, 1, &pgOptions, log, &sizeofLog,
        &m_illuminationEstimationPipeline.m_hitGroupProgramGroups[RayType::SpacialSampling][PrimitiveType::Triangle]));

    pgDesc.hitgroup.moduleIS = m_illuminationEstimationPipeline.m_linearCurveModule;
    OPTIX_CHECK(optixProgramGroupCreate(
        m_optixDeviceContext, &pgDesc, 1, &pgOptions, log, &sizeofLog,
        &m_illuminationEstimationPipeline.m_hitGroupProgramGroups[RayType::SpacialSampling][PrimitiveType::Linear]));

    pgDesc.hitgroup.moduleIS = m_illuminationEstimationPipeline.m_quadraticCurveModule;
    OPTIX_CHECK(optixProgramGroupCreate(
        m_optixDeviceContext, &pgDesc, 1, &pgOptions, log, &sizeofLog,
        &m_illuminationEstimationPipeline
             .m_hitGroupProgramGroups[RayType::SpacialSampling][PrimitiveType::QuadraticBSpline]));

    pgDesc.hitgroup.moduleIS = m_illuminationEstimationPipeline.m_cubicCurveModule;
    OPTIX_CHECK(
        optixProgramGroupCreate(m_optixDeviceContext, &pgDesc, 1, &pgOptions, log, &sizeofLog,
                                &m_illuminationEstimationPipeline
                                     .m_hitGroupProgramGroups[RayType::SpacialSampling][PrimitiveType::CubicBSpline]));
    if (sizeofLog > 1)
      std::cout << log << std::endl;
  }
  {
    char log[2048];
    size_t sizeofLog = sizeof(log);

    OptixProgramGroupOptions pgOptions = {};
    OptixProgramGroupDesc pgDesc = {};
    pgDesc.kind = OPTIX_PROGRAM_GROUP_KIND_HITGROUP;
    pgDesc.hitgroup.moduleCH = m_pointCloudScanningPipeline.m_module;
    pgDesc.hitgroup.moduleAH = m_pointCloudScanningPipeline.m_module;
    // -------------------------------------------------------
    // radiance rays
    // -------------------------------------------------------
    pgDesc.hitgroup.entryFunctionNameCH = "__closesthit__PCS_R";
    pgDesc.hitgroup.entryFunctionNameAH = "__anyhit__PCS_R";
    OPTIX_CHECK(optixProgramGroupCreate(
        m_optixDeviceContext, &pgDesc, 1, &pgOptions, log, &sizeofLog,
        &m_pointCloudScanningPipeline.m_hitGroupProgramGroups[RayType::Radiance][PrimitiveType::Triangle]));

    pgDesc.hitgroup.moduleIS = m_pointCloudScanningPipeline.m_linearCurveModule;
    OPTIX_CHECK(optixProgramGroupCreate(
        m_optixDeviceContext, &pgDesc, 1, &pgOptions, log, &sizeofLog,
        &m_pointCloudScanningPipeline.m_hitGroupProgramGroups[RayType::Radiance][PrimitiveType::Linear]));

    pgDesc.hitgroup.moduleIS = m_pointCloudScanningPipeline.m_quadraticCurveModule;
    OPTIX_CHECK(optixProgramGroupCreate(
        m_optixDeviceContext, &pgDesc, 1, &pgOptions, log, &sizeofLog,
        &m_pointCloudScanningPipeline.m_hitGroupProgramGroups[RayType::Radiance][PrimitiveType::QuadraticBSpline]));

    pgDesc.hitgroup.moduleIS = m_pointCloudScanningPipeline.m_cubicCurveModule;
    OPTIX_CHECK(optixProgramGroupCreate(
        m_optixDeviceContext, &pgDesc, 1, &pgOptions, log, &sizeofLog,
        &m_pointCloudScanningPipeline.m_hitGroupProgramGroups[RayType::Radiance][PrimitiveType::CubicBSpline]));
    if (sizeofLog > 1)
      std::cout << log << std::endl;
    // -------------------------------------------------------
    // BSSRDF Sampler ray
    // -------------------------------------------------------
    pgDesc.hitgroup.entryFunctionNameCH = "__closesthit__PCS_SS";
    pgDesc.hitgroup.entryFunctionNameAH = "__anyhit__PCS_SS";
    OPTIX_CHECK(optixProgramGroupCreate(
        m_optixDeviceContext, &pgDesc, 1, &pgOptions, log, &sizeofLog,
        &m_pointCloudScanningPipeline.m_hitGroupProgramGroups[RayType::SpacialSampling][PrimitiveType::Triangle]));

    pgDesc.hitgroup.moduleIS = m_pointCloudScanningPipeline.m_linearCurveModule;
    OPTIX_CHECK(optixProgramGroupCreate(
        m_optixDeviceContext, &pgDesc, 1, &pgOptions, log, &sizeofLog,
        &m_pointCloudScanningPipeline.m_hitGroupProgramGroups[RayType::SpacialSampling][PrimitiveType::Linear]));

    pgDesc.hitgroup.moduleIS = m_pointCloudScanningPipeline.m_quadraticCurveModule;
    OPTIX_CHECK(optixProgramGroupCreate(
        m_optixDeviceContext, &pgDesc, 1, &pgOptions, log, &sizeofLog,
        &m_pointCloudScanningPipeline
             .m_hitGroupProgramGroups[RayType::SpacialSampling][PrimitiveType::QuadraticBSpline]));

    pgDesc.hitgroup.moduleIS = m_pointCloudScanningPipeline.m_cubicCurveModule;
    OPTIX_CHECK(optixProgramGroupCreate(
        m_optixDeviceContext, &pgDesc, 1, &pgOptions, log, &sizeofLog,
        &m_pointCloudScanningPipeline.m_hitGroupProgramGroups[RayType::SpacialSampling][PrimitiveType::CubicBSpline]));
    if (sizeofLog > 1)
      std::cout << log << std::endl;
  }
}

__global__ void CopyVerticesInstancedKernel(int matricesSize, int verticesSize, InstanceMatrix *matrices,
                                            evo_engine::Vertex *vertices, glm::vec3 *targetPositions,
                                            evo_engine::Vertex *targetVertices) {
  const int idx = threadIdx.x + blockIdx.x * blockDim.x;
  if (idx < verticesSize * matricesSize) {
    const glm::vec3 position =
        matrices[idx / verticesSize].m_instanceMatrix * glm::vec4(vertices[idx % verticesSize].position, 1.0f);
    targetPositions[idx] = position;
    glm::vec3 N = glm::normalize(matrices[idx / verticesSize].m_instanceMatrix *
                                 glm::vec4(vertices[idx % verticesSize].normal, 0.0f));
    glm::vec3 T = glm::normalize(matrices[idx / verticesSize].m_instanceMatrix *
                                 glm::vec4(vertices[idx % verticesSize].tangent, 0.0f));
    T = glm::normalize(T - dot(T, N) * N);
    targetVertices[idx] = {};
    targetVertices[idx].position = position;
    targetVertices[idx].tangent = T;
    targetVertices[idx].normal = N;
    targetVertices[idx].tex_coord = vertices[idx % verticesSize].tex_coord;
    targetVertices[idx].color = matrices[idx / verticesSize].m_instanceColor;
  }
}

__global__ void CopyStrandPointsKernel(int size, evo_engine::StrandPoint *strandPoints, float *targetThicknesses) {
  const int idx = threadIdx.x + blockIdx.x * blockDim.x;
  if (idx < size) {
    targetThicknesses[idx] = strandPoints[idx].thickness;
  }
}

__global__ void CopyVerticesKernel(int size, evo_engine::Vertex *vertices, glm::vec3 *targetPositions) {
  const int idx = threadIdx.x + blockIdx.x * blockDim.x;
  if (idx < size) {
    targetPositions[idx] = vertices[idx].position;
  }
}

__global__ void CopySkinnedVerticesKernel(int size, evo_engine::SkinnedVertex *vertices, glm::mat4 *boneMatrices,
                                          glm::vec3 *targetPositions, evo_engine::Vertex *targetVertices) {
  const int idx = threadIdx.x + blockIdx.x * blockDim.x;
  if (idx < size) {
    glm::mat4 boneTransform = boneMatrices[vertices[idx].bond_id[0]] * vertices[idx].weight[0];
    if (vertices[idx].bond_id[1] != -1) {
      boneTransform += boneMatrices[vertices[idx].bond_id[1]] * vertices[idx].weight[1];
    }
    if (vertices[idx].bond_id[2] != -1) {
      boneTransform += boneMatrices[vertices[idx].bond_id[2]] * vertices[idx].weight[2];
    }
    if (vertices[idx].bond_id[3] != -1) {
      boneTransform += boneMatrices[vertices[idx].bond_id[3]] * vertices[idx].weight[3];
    }
    if (vertices[idx].bond_id2[0] != -1) {
      boneTransform += boneMatrices[vertices[idx].bond_id2[0]] * vertices[idx].weight2[0];
    }
    if (vertices[idx].bond_id2[1] != -1) {
      boneTransform += boneMatrices[vertices[idx].bond_id2[1]] * vertices[idx].weight2[1];
    }
    if (vertices[idx].bond_id2[2] != -1) {
      boneTransform += boneMatrices[vertices[idx].bond_id2[2]] * vertices[idx].weight2[2];
    }
    if (vertices[idx].bond_id2[3] != -1) {
      boneTransform += boneMatrices[vertices[idx].bond_id2[3]] * vertices[idx].weight2[3];
    }
    const glm::vec3 position = boneTransform * glm::vec4(vertices[idx].position, 1.0f);
    targetPositions[idx] = position;
    glm::vec3 N = glm::normalize(boneTransform * glm::vec4(vertices[idx].normal, 0.0f));
    glm::vec3 T = glm::normalize(boneTransform * glm::vec4(vertices[idx].tangent, 0.0f));
    T = glm::normalize(T - dot(T, N) * N);
    targetVertices[idx].position = position;
    targetVertices[idx].normal = N;
    targetVertices[idx].tangent = T;
    targetVertices[idx].tex_coord = vertices[idx].tex_coord;
    targetVertices[idx].color = vertices[idx].color;
  }
}

void RayTracedGeometry::BuildGAS(const OptixDeviceContext &context) {
#pragma region Clean previous buffer
  m_vertexDataBuffer.Free();
  m_triangleBuffer.Free();

  m_curveStrandUBuffer.Free();
  m_curveStrandIBuffer.Free();
  m_curveStrandInfoBuffer.Free();

  m_acceleratedStructureBuffer.Free();
#pragma endregion

  CudaBuffer devicePositionBuffer;
  CudaBuffer deviceWidthBuffer;

#pragma region Geometry Inputs
  // ==================================================================
  // geometry inputs
  // ==================================================================
  OptixBuildInput buildInput;
  const uint32_t triangleInputFlags[1] = {OPTIX_GEOMETRY_FLAG_NONE};
  switch (m_rendererType) {
    case RendererType::Curve: {
      CUdeviceptr devicePoints;
      CUdeviceptr deviceWidths;
      CUdeviceptr deviceStrands;

      // m_curveStrandUBuffer.Upload(*m_strandU);
      // m_curveStrandIBuffer.Upload(*m_strandIndices);
      // m_curveStrandInfoBuffer.Upload(*m_strandInfos);

      deviceWidthBuffer.Resize(m_curvePoints->size() * sizeof(float));
      m_vertexDataBuffer.Upload(*m_curvePoints);
      m_triangleBuffer.Upload(*m_curveSegments);

      int blockSize = 0;    // The launch configurator returned block size
      int minGridSize = 0;  // The minimum grid size needed to achieve the
      // maximum occupancy for a full device launch
      int gridSize = 0;  // The actual grid size needed, based on input size
      int size = m_curvePoints->size();
      cudaOccupancyMaxPotentialBlockSize(&minGridSize, &blockSize, CopyStrandPointsKernel, 0, size);
      gridSize = (size + blockSize - 1) / blockSize;
      CopyStrandPointsKernel<<<gridSize, blockSize>>>(size,
                                                      static_cast<evo_engine::StrandPoint *>(m_vertexDataBuffer.d_ptr),
                                                      static_cast<float *>(deviceWidthBuffer.d_ptr));
      CUDA_SYNC_CHECK();

      buildInput.type = OPTIX_BUILD_INPUT_TYPE_CURVES;
      switch (m_geometryType) {
        case PrimitiveType::Linear:
          buildInput.curveArray.curveType = OPTIX_PRIMITIVE_TYPE_ROUND_LINEAR;
          break;
        case PrimitiveType::QuadraticBSpline:
          buildInput.curveArray.curveType = OPTIX_PRIMITIVE_TYPE_ROUND_QUADRATIC_BSPLINE;
          break;
        case PrimitiveType::CubicBSpline:
          buildInput.curveArray.curveType = OPTIX_PRIMITIVE_TYPE_ROUND_CUBIC_BSPLINE;
          break;
      }
      devicePoints = m_vertexDataBuffer.DevicePointer();
      deviceWidths = deviceWidthBuffer.DevicePointer();
      deviceStrands = m_triangleBuffer.DevicePointer();
      buildInput.curveArray.numPrimitives = m_curveSegments->size();
      buildInput.curveArray.vertexBuffers = &devicePoints;
      buildInput.curveArray.numVertices = static_cast<unsigned int>(m_curvePoints->size());
      buildInput.curveArray.vertexStrideInBytes = sizeof(evo_engine::StrandPoint);
      buildInput.curveArray.widthBuffers = &deviceWidths;
      buildInput.curveArray.widthStrideInBytes = sizeof(float);
      buildInput.curveArray.normalBuffers = 0;
      buildInput.curveArray.normalStrideInBytes = 0;
      buildInput.curveArray.indexBuffer = deviceStrands;
      buildInput.curveArray.indexStrideInBytes = sizeof(int);
      buildInput.curveArray.flag = OPTIX_GEOMETRY_FLAG_NONE;
      buildInput.curveArray.endcapFlags = OPTIX_CURVE_ENDCAP_ON;
      buildInput.curveArray.primitiveIndexOffset = 0;
    } break;
    case RendererType::Default: {
      CUdeviceptr deviceVertexPositions;
      CUdeviceptr deviceVertexTriangles;

      m_vertexDataBuffer.Upload(*m_vertices);
      int blockSize = 0;    // The launch configurator returned block size
      int minGridSize = 0;  // The minimum grid size needed to achieve the
      // maximum occupancy for a full device launch
      int gridSize = 0;  // The actual grid size needed, based on input size
      int size = m_vertices->size();
      cudaOccupancyMaxPotentialBlockSize(&minGridSize, &blockSize, CopyVerticesKernel, 0, size);
      gridSize = (size + blockSize - 1) / blockSize;
      CUDA_SYNC_CHECK();
      m_triangleBuffer.Upload(*m_triangles);

      buildInput = {};
      buildInput.type = OPTIX_BUILD_INPUT_TYPE_TRIANGLES;

      // create local variables, because we need a *pointer* to the
      // device pointers
      deviceVertexPositions = m_vertexDataBuffer.DevicePointer();
      deviceVertexTriangles = m_triangleBuffer.DevicePointer();

      buildInput.triangleArray.vertexFormat = OPTIX_VERTEX_FORMAT_FLOAT3;
      buildInput.triangleArray.vertexStrideInBytes = sizeof(evo_engine::Vertex);
      buildInput.triangleArray.numVertices = static_cast<int>(m_vertices->size());
      buildInput.triangleArray.vertexBuffers = &deviceVertexPositions;

      buildInput.triangleArray.indexFormat = OPTIX_INDICES_FORMAT_UNSIGNED_INT3;
      buildInput.triangleArray.indexStrideInBytes = sizeof(glm::uvec3);
      buildInput.triangleArray.numIndexTriplets = static_cast<int>(m_triangleBuffer.size_in_bytes / sizeof(glm::uvec3));
      buildInput.triangleArray.indexBuffer = deviceVertexTriangles;

      // in this example we have one SBT entry, and no per-primitive
      // materials:
      buildInput.triangleArray.flags = triangleInputFlags;
      buildInput.triangleArray.numSbtRecords = 1;
      buildInput.triangleArray.sbtIndexOffsetBuffer = 0;
      buildInput.triangleArray.sbtIndexOffsetSizeInBytes = 0;
      buildInput.triangleArray.sbtIndexOffsetStrideInBytes = 0;
    } break;
    case RendererType::Skinned: {
      CUdeviceptr deviceVertexPositions;
      CUdeviceptr deviceVertexTriangles;

      CudaBuffer skinnedVerticesBuffer;
      CudaBuffer boneMatricesBuffer;
      skinnedVerticesBuffer.Upload(*m_skinnedVertices);
      boneMatricesBuffer.Upload(*m_boneMatrices);
      m_vertexDataBuffer.Resize(m_skinnedVertices->size() * sizeof(evo_engine::Vertex));
      devicePositionBuffer.Resize(m_skinnedVertices->size() * sizeof(glm::vec3));
      int blockSize = 0;    // The launch configurator returned block size
      int minGridSize = 0;  // The minimum grid size needed to achieve the
      // maximum occupancy for a full device launch
      int gridSize = 0;  // The actual grid size needed, based on input size
      int size = m_skinnedVertices->size();
      cudaOccupancyMaxPotentialBlockSize(&minGridSize, &blockSize, CopySkinnedVerticesKernel, 0, size);
      gridSize = (size + blockSize - 1) / blockSize;
      CopySkinnedVerticesKernel<<<gridSize, blockSize>>>(
          size, static_cast<evo_engine::SkinnedVertex *>(skinnedVerticesBuffer.d_ptr),
          static_cast<glm::mat4 *>(boneMatricesBuffer.d_ptr), static_cast<glm::vec3 *>(devicePositionBuffer.d_ptr),
          static_cast<evo_engine::Vertex *>(m_vertexDataBuffer.d_ptr));
      CUDA_SYNC_CHECK();
      m_triangleBuffer.Upload(*m_triangles);
      buildInput = {};
      buildInput.type = OPTIX_BUILD_INPUT_TYPE_TRIANGLES;
      // create local variables, because we need a *pointer* to the
      // device pointers
      deviceVertexPositions = devicePositionBuffer.DevicePointer();
      deviceVertexTriangles = m_triangleBuffer.DevicePointer();
      buildInput.triangleArray.vertexFormat = OPTIX_VERTEX_FORMAT_FLOAT3;
      buildInput.triangleArray.vertexStrideInBytes = sizeof(glm::vec3);
      buildInput.triangleArray.numVertices = static_cast<int>(devicePositionBuffer.size_in_bytes / sizeof(glm::vec3));
      buildInput.triangleArray.vertexBuffers = &deviceVertexPositions;
      buildInput.triangleArray.indexFormat = OPTIX_INDICES_FORMAT_UNSIGNED_INT3;
      buildInput.triangleArray.indexStrideInBytes = sizeof(glm::uvec3);
      buildInput.triangleArray.numIndexTriplets = static_cast<int>(m_triangleBuffer.size_in_bytes / sizeof(glm::uvec3));
      buildInput.triangleArray.indexBuffer = deviceVertexTriangles;
      // in this example we have one SBT entry, and no per-primitive
      // materials:
      buildInput.triangleArray.flags = triangleInputFlags;
      buildInput.triangleArray.numSbtRecords = 1;
      buildInput.triangleArray.sbtIndexOffsetBuffer = 0;
      buildInput.triangleArray.sbtIndexOffsetSizeInBytes = 0;
      buildInput.triangleArray.sbtIndexOffsetStrideInBytes = 0;
      skinnedVerticesBuffer.Free();
      boneMatricesBuffer.Free();
    } break;
    case RendererType::Instanced: {
      CUdeviceptr deviceVertexPositions;
      CUdeviceptr deviceVertexTriangles;

      CudaBuffer verticesBuffer;
      CudaBuffer instanceMatricesBuffer;
      verticesBuffer.Upload(*m_vertices);
      instanceMatricesBuffer.Upload(*m_instanceMatrices);
      m_vertexDataBuffer.Resize(m_instanceMatrices->size() * m_vertices->size() * sizeof(evo_engine::Vertex));

      devicePositionBuffer.Resize(m_instanceMatrices->size() * m_vertices->size() * sizeof(glm::vec3));
      int blockSize = 0;    // The launch configurator returned block verticesSize
      int minGridSize = 0;  // The minimum grid verticesSize needed to achieve the
      // maximum occupancy for a full device launch
      int gridSize = 0;  // The actual grid verticesSize needed, based on input verticesSize
      int verticesSize = m_vertices->size();
      int matricesSize = m_instanceMatrices->size();
      int size = verticesSize * matricesSize;
      cudaOccupancyMaxPotentialBlockSize(&minGridSize, &blockSize, CopyVerticesInstancedKernel, 0, size);
      gridSize = (size + blockSize - 1) / blockSize;
      CopyVerticesInstancedKernel<<<gridSize, blockSize>>>(
          matricesSize, verticesSize, static_cast<InstanceMatrix *>(instanceMatricesBuffer.d_ptr),
          static_cast<evo_engine::Vertex *>(verticesBuffer.d_ptr), static_cast<glm::vec3 *>(devicePositionBuffer.d_ptr),
          static_cast<evo_engine::Vertex *>(m_vertexDataBuffer.d_ptr));
      CUDA_SYNC_CHECK();
      auto triangles = std::vector<glm::uvec3>();
      triangles.resize(m_triangles->size() * m_instanceMatrices->size());
      unsigned offset = 0;
      for (const auto &matrix : *m_instanceMatrices) {
        for (const auto &i : *m_triangles) {
          triangles.push_back(i);
          triangles.back() += glm::uvec3(offset);
        }
        offset += m_vertices->size();
      }
      m_triangleBuffer.Upload(triangles);
      buildInput = {};
      buildInput.type = OPTIX_BUILD_INPUT_TYPE_TRIANGLES;
      // create local variables, because we need a *pointer* to the
      // device pointers
      deviceVertexPositions = devicePositionBuffer.DevicePointer();
      deviceVertexTriangles = m_triangleBuffer.DevicePointer();
      buildInput.triangleArray.vertexFormat = OPTIX_VERTEX_FORMAT_FLOAT3;
      buildInput.triangleArray.vertexStrideInBytes = sizeof(glm::vec3);
      buildInput.triangleArray.numVertices = static_cast<int>(devicePositionBuffer.size_in_bytes / sizeof(glm::vec3));
      buildInput.triangleArray.vertexBuffers = &deviceVertexPositions;
      buildInput.triangleArray.indexFormat = OPTIX_INDICES_FORMAT_UNSIGNED_INT3;
      buildInput.triangleArray.indexStrideInBytes = sizeof(glm::uvec3);
      buildInput.triangleArray.numIndexTriplets = static_cast<int>(m_triangleBuffer.size_in_bytes / sizeof(glm::uvec3));
      buildInput.triangleArray.indexBuffer = deviceVertexTriangles;
      // in this example we have one SBT entry, and no per-primitive
      // materials:
      buildInput.triangleArray.flags = triangleInputFlags;
      buildInput.triangleArray.numSbtRecords = 1;
      buildInput.triangleArray.sbtIndexOffsetBuffer = 0;
      buildInput.triangleArray.sbtIndexOffsetSizeInBytes = 0;
      buildInput.triangleArray.sbtIndexOffsetStrideInBytes = 0;
      verticesBuffer.Free();
      instanceMatricesBuffer.Free();
      instanceMatricesBuffer.Free();
    } break;
  }
#pragma endregion
#pragma region BLAS setup
  // ==================================================================
  // BLAS setup
  // ==================================================================

  OptixAccelBuildOptions accelerateOptions = {};
  accelerateOptions.buildFlags =
      OPTIX_BUILD_FLAG_NONE | OPTIX_BUILD_FLAG_ALLOW_COMPACTION | OPTIX_BUILD_FLAG_PREFER_FAST_TRACE;
  accelerateOptions.motionOptions.numKeys = 1;
  accelerateOptions.operation = OPTIX_BUILD_OPERATION_BUILD;

  OptixAccelBufferSizes blasBufferSizes;
  OPTIX_CHECK(optixAccelComputeMemoryUsage(context, &accelerateOptions, &buildInput,
                                           1,  // num_build_inputs
                                           &blasBufferSizes));
#pragma endregion
#pragma region Prapere compaction
  // ==================================================================
  // prepare compaction
  // ==================================================================

  CudaBuffer compactedSizeBuffer;
  compactedSizeBuffer.Resize(sizeof(uint64_t));
  OptixAccelEmitDesc emitDesc;
  emitDesc.type = OPTIX_PROPERTY_TYPE_COMPACTED_SIZE;
  emitDesc.result = compactedSizeBuffer.DevicePointer();
#pragma endregion
#pragma region Build AS
  // ==================================================================
  // execute build (main stage)
  // ==================================================================

  CudaBuffer tempBuffer;
  tempBuffer.Resize(blasBufferSizes.tempSizeInBytes);

  CudaBuffer outputBuffer;
  outputBuffer.Resize(blasBufferSizes.outputSizeInBytes);

  OPTIX_CHECK(optixAccelBuild(context,
                              /* stream */ nullptr, &accelerateOptions, &buildInput, 1, tempBuffer.DevicePointer(),
                              tempBuffer.size_in_bytes, outputBuffer.DevicePointer(), outputBuffer.size_in_bytes,
                              &m_traversableHandle, &emitDesc, 1));
  CUDA_SYNC_CHECK();
#pragma endregion
#pragma region Perform compaction
  // ==================================================================
  // perform compaction
  // ==================================================================
  uint64_t compactedSize;
  compactedSizeBuffer.Download(&compactedSize, 1);
  m_acceleratedStructureBuffer.Resize(compactedSize);
  OPTIX_CHECK(optixAccelCompact(context,
                                /*stream:*/ nullptr, m_traversableHandle, m_acceleratedStructureBuffer.DevicePointer(),
                                m_acceleratedStructureBuffer.size_in_bytes, &m_traversableHandle));
  CUDA_SYNC_CHECK();
#pragma endregion
#pragma region Compaction clean up
  // ==================================================================
  // aaaaaand .... clean up
  // ==================================================================
  outputBuffer.Free();  // << the Uncompacted, temporary output buffer
  tempBuffer.Free();
  compactedSizeBuffer.Free();
#pragma endregion

  devicePositionBuffer.Free();
  deviceWidthBuffer.Free();
  m_updateFlag = false;
}

void RayTracedGeometry::UploadForSBT() {
  m_geometryBuffer.Free();
  if (m_geometryType != PrimitiveType::Triangle) {
    Curves curves;
    curves.strand_points = reinterpret_cast<evo_engine::StrandPoint *>(m_vertexDataBuffer.DevicePointer());
    // curves.m_strandU = reinterpret_cast<glm::vec2 *>(m_curveStrandUBuffer.DevicePointer());
    // curves.m_strandIndices = reinterpret_cast<int *>(m_curveStrandIBuffer.DevicePointer());
    // curves.m_strandInfos = reinterpret_cast<glm::uvec2 *>(m_curveStrandInfoBuffer.DevicePointer());
    curves.m_segments = reinterpret_cast<int *>(m_triangleBuffer.DevicePointer());
    m_geometryBuffer.Upload(&curves, 1);
  } else {
    TriangularMesh mesh;
    mesh.m_vertices = reinterpret_cast<evo_engine::Vertex *>(m_vertexDataBuffer.DevicePointer());
    mesh.m_triangles = reinterpret_cast<glm::uvec3 *>(m_triangleBuffer.DevicePointer());
    m_geometryBuffer.Upload(&mesh, 1);
  }
}

void RayTracer::BuildIAS() {
  std::vector<uint64_t> removeQueue;
  for (const auto &i : m_geometries) {
    if (i.second.m_removeFlag) {
      removeQueue.emplace_back(i.first);
    }
  }
  for (auto &i : removeQueue) {
    auto &geometry = m_geometries.at(i);
    geometry.m_geometryBuffer.Free();
    geometry.m_vertexDataBuffer.Free();
    geometry.m_triangleBuffer.Free();

    geometry.m_curveStrandUBuffer.Free();
    geometry.m_curveStrandIBuffer.Free();
    geometry.m_curveStrandInfoBuffer.Free();

    geometry.m_acceleratedStructureBuffer.Free();
    m_geometries.erase(i);
  }
  for (auto &i : m_geometries) {
    if (i.second.m_updateFlag) {
      i.second.BuildGAS(m_optixDeviceContext);
      i.second.UploadForSBT();
    }
  }
  removeQueue.clear();
  for (const auto &i : m_instances) {
    if (i.second.m_removeFlag) {
      removeQueue.emplace_back(i.first);
    }
  }
  for (auto &i : removeQueue) {
    m_instances.erase(i);
  }

  std::vector<OptixInstance> optixInstances;
  unsigned int sbtOffset = 0;

  OptixInstance optixInstance = {};
  // Common optixInstance settings
  optixInstance.instanceId = 0;
  optixInstance.visibilityMask = 0xFF;
  optixInstance.flags = OPTIX_INSTANCE_FLAG_NONE;

  for (auto &instance : m_instances) {
    glm::mat3x4 transform = glm::transpose(instance.second.m_globalTransform);
    memcpy(optixInstance.transform, &transform, sizeof(glm::mat3x4));
    optixInstance.sbtOffset = sbtOffset;
    optixInstance.traversableHandle = m_geometries.at(instance.second.m_geometryMapKey).m_traversableHandle;
    sbtOffset += (int)RayType::RayTypeCount;
    optixInstances.push_back(optixInstance);
  }

  CudaBuffer deviceTempInstances;
  deviceTempInstances.Upload(optixInstances);

  // Instance build input.
  OptixBuildInput buildInput = {};

  buildInput.type = OPTIX_BUILD_INPUT_TYPE_INSTANCES;
  buildInput.instanceArray.instances = deviceTempInstances.DevicePointer();
  buildInput.instanceArray.numInstances = static_cast<unsigned int>(optixInstances.size());

  OptixAccelBuildOptions accelBuildOptions = {};
  accelBuildOptions.buildFlags = OPTIX_BUILD_FLAG_NONE;
  accelBuildOptions.operation = OPTIX_BUILD_OPERATION_BUILD;

  OptixAccelBufferSizes bufferSizesIAS;
  OPTIX_CHECK(optixAccelComputeMemoryUsage(m_optixDeviceContext, &accelBuildOptions, &buildInput,
                                           1,  // Number of build inputs
                                           &bufferSizesIAS));

  CudaBuffer deviceTempBufferIAS;
  deviceTempBufferIAS.Resize(bufferSizesIAS.tempSizeInBytes);
  m_iASBuffer.Resize(bufferSizesIAS.outputSizeInBytes);

  OptixTraversableHandle iASHandle = 0;
  OPTIX_CHECK(optixAccelBuild(m_optixDeviceContext,
                              0,  // CUDA stream
                              &accelBuildOptions, &buildInput,
                              1,  // num build inputs
                              deviceTempBufferIAS.DevicePointer(), bufferSizesIAS.tempSizeInBytes,
                              m_iASBuffer.DevicePointer(), bufferSizesIAS.outputSizeInBytes, &iASHandle,
                              nullptr,  // emitted property list
                              0));      // num emitted properties
  deviceTempInstances.Free();
  deviceTempBufferIAS.Free();

  m_cameraRenderingLaunchParams.m_traversable = iASHandle;
  m_illuminationEstimationLaunchParams.m_traversable = iASHandle;
  m_pointCloudScanningLaunchParams.m_traversable = iASHandle;
  m_hasAccelerationStructure = true;
  m_sceneModified = true;
}

void RayTracer::AssemblePipelines() {
  AssemblePipeline(m_cameraRenderingPipeline);
  AssemblePipeline(m_illuminationEstimationPipeline);
  AssemblePipeline(m_pointCloudScanningPipeline);
}

void RayTracer::CreateRayGenProgram(RayTracerPipeline &targetPipeline, char entryFunctionName[]) const {
  OptixProgramGroupOptions pgOptions = {};
  OptixProgramGroupDesc pgDesc = {};
  pgDesc.kind = OPTIX_PROGRAM_GROUP_KIND_RAYGEN;
  pgDesc.raygen.module = targetPipeline.m_module;
  pgDesc.raygen.entryFunctionName = entryFunctionName;
  char log[2048];
  size_t sizeofLog = sizeof(log);
  OPTIX_CHECK(optixProgramGroupCreate(m_optixDeviceContext, &pgDesc, 1, &pgOptions, log, &sizeofLog,
                                      &targetPipeline.m_rayGenProgramGroups));
  if (sizeofLog > 1)
    std::cout << log << std::endl;
}

void RayTracer::CreateModule(RayTracerPipeline &targetPipeline, char ptxCode[], char launchParamsName[]) const {
  targetPipeline.m_launchParamsName = launchParamsName;

  targetPipeline.m_moduleCompileOptions.maxRegisterCount = 50;
  targetPipeline.m_moduleCompileOptions.optLevel = OPTIX_COMPILE_OPTIMIZATION_DEFAULT;
  targetPipeline.m_moduleCompileOptions.debugLevel = OPTIX_COMPILE_DEBUG_LEVEL_NONE;

  targetPipeline.m_pipelineCompileOptions = {};
  targetPipeline.m_pipelineCompileOptions.traversableGraphFlags = OPTIX_TRAVERSABLE_GRAPH_FLAG_ALLOW_ANY;
  targetPipeline.m_pipelineCompileOptions.usesMotionBlur = false;
  targetPipeline.m_pipelineCompileOptions.numPayloadValues = 2;
  targetPipeline.m_pipelineCompileOptions.numAttributeValues = 2;
  targetPipeline.m_pipelineCompileOptions.exceptionFlags = OPTIX_EXCEPTION_FLAG_NONE;
  targetPipeline.m_pipelineCompileOptions.pipelineLaunchParamsVariableName = launchParamsName;
  targetPipeline.m_pipelineCompileOptions.usesPrimitiveTypeFlags =
      OPTIX_PRIMITIVE_TYPE_FLAGS_TRIANGLE | OPTIX_PRIMITIVE_TYPE_FLAGS_ROUND_LINEAR |
      OPTIX_PRIMITIVE_TYPE_FLAGS_ROUND_QUADRATIC_BSPLINE | OPTIX_PRIMITIVE_TYPE_FLAGS_ROUND_CUBIC_BSPLINE;

  const std::string code = ptxCode;

  char log[2048];
  size_t sizeof_log = sizeof(log);
  OPTIX_CHECK(optixModuleCreate(m_optixDeviceContext, &targetPipeline.m_moduleCompileOptions,
                                &targetPipeline.m_pipelineCompileOptions, code.c_str(), code.size(), log, &sizeof_log,
                                &targetPipeline.m_module));

  OptixBuiltinISOptions builtinISOptions = {};
  builtinISOptions.builtinISModuleType = OPTIX_PRIMITIVE_TYPE_ROUND_QUADRATIC_BSPLINE;
  builtinISOptions.curveEndcapFlags = OPTIX_CURVE_ENDCAP_ON;
  OPTIX_CHECK(optixBuiltinISModuleGet(m_optixDeviceContext, &targetPipeline.m_moduleCompileOptions,
                                      &targetPipeline.m_pipelineCompileOptions, &builtinISOptions,
                                      &targetPipeline.m_quadraticCurveModule));

  builtinISOptions.builtinISModuleType = OPTIX_PRIMITIVE_TYPE_ROUND_CUBIC_BSPLINE;
  OPTIX_CHECK(optixBuiltinISModuleGet(m_optixDeviceContext, &targetPipeline.m_moduleCompileOptions,
                                      &targetPipeline.m_pipelineCompileOptions, &builtinISOptions,
                                      &targetPipeline.m_cubicCurveModule));

  builtinISOptions.builtinISModuleType = OPTIX_PRIMITIVE_TYPE_ROUND_LINEAR;
  OPTIX_CHECK(optixBuiltinISModuleGet(m_optixDeviceContext, &targetPipeline.m_moduleCompileOptions,
                                      &targetPipeline.m_pipelineCompileOptions, &builtinISOptions,
                                      &targetPipeline.m_linearCurveModule));

  if (sizeof_log > 1)
    std::cout << log << std::endl;
}

void RayTracer::AssemblePipeline(RayTracerPipeline &targetPipeline) const {
  std::vector<OptixProgramGroup> programGroups;
  programGroups.push_back(targetPipeline.m_rayGenProgramGroups);
  for (auto &i : targetPipeline.m_missProgramGroups)
    programGroups.push_back(i.second);
  for (auto &i : targetPipeline.m_hitGroupProgramGroups)
    for (auto &j : i.second)
      programGroups.push_back(j.second);

  const uint32_t maxTraceDepth = 31;
  targetPipeline.m_pipelineLinkOptions.maxTraceDepth = maxTraceDepth;
  char log[2048];
  size_t sizeofLog = sizeof(log);
  OPTIX_CHECK(optixPipelineCreate(m_optixDeviceContext, &targetPipeline.m_pipelineCompileOptions,
                                  &targetPipeline.m_pipelineLinkOptions, programGroups.data(),
                                  static_cast<int>(programGroups.size()), log, &sizeofLog, &targetPipeline.m_pipeline));
  if (sizeofLog > 1)
    std::cout << log << std::endl;

  OptixStackSizes stackSizes = {};
  for (auto &progGroup : programGroups) {
    OPTIX_CHECK(optixUtilAccumulateStackSizes(progGroup, &stackSizes, targetPipeline.m_pipeline));
  }

  uint32_t directCallableStackSizeFromTraversal;
  uint32_t directCallableStackSizeFromState;
  uint32_t continuationStackSize;
  OPTIX_CHECK(optixUtilComputeStackSizes(&stackSizes, maxTraceDepth,
                                         0,  // maxCCDepth
                                         0,  // maxDCDEpth
                                         &directCallableStackSizeFromTraversal, &directCallableStackSizeFromState,
                                         &continuationStackSize));
  OPTIX_CHECK(optixPipelineSetStackSize(targetPipeline.m_pipeline, directCallableStackSizeFromTraversal,
                                        directCallableStackSizeFromState, continuationStackSize,
                                        2  // maxTraversableDepth
                                        ));
  if (sizeofLog > 1)
    std::cout << log << std::endl;
}

void RayTracer::BuildSBT() {
  std::vector<uint64_t> removeQueue;
  for (auto &i : m_materials) {
    auto &material = i.second;
    material.m_materialBuffer.Free();
    if (material.m_removeFlag) {
      removeQueue.emplace_back(i.first);
    } else {
      material.UploadForSBT();
    }
  }
  for (auto &i : removeQueue) {
    auto &material = m_materials.at(i);
    m_materials.erase(i);
  }
#pragma region Prepare SBTs
  std::map<uint64_t, SBT> sBTs;
  for (auto &instancePair : m_instances) {
    auto &instance = instancePair.second;
    auto &material = m_materials.at(instance.m_materialMapKey);
    auto &geometry = m_geometries.at(instance.m_geometryMapKey);
    auto &sBT = sBTs[instancePair.first];
    sBT.m_handle = instance.m_privateComponentHandle;
    sBT.m_globalTransform = instance.m_globalTransform;
    sBT.m_geometryType = geometry.m_rendererType;
    sBT.m_geometry = reinterpret_cast<void *>(geometry.m_geometryBuffer.DevicePointer());
    sBT.m_materialType = material.m_materialType;
    sBT.m_material = reinterpret_cast<void *>(material.m_materialBuffer.DevicePointer());
  }
#pragma endregion
  {
    // ------------------------------------------------------------------
    // build raygen records
    // ------------------------------------------------------------------
    std::vector<CameraRenderingRayGenRecord> raygenRecords;
    CameraRenderingRayGenRecord rec;
    OPTIX_CHECK(optixSbtRecordPackHeader(m_cameraRenderingPipeline.m_rayGenProgramGroups, &rec));
    rec.data = nullptr; /* for now ... */
    raygenRecords.push_back(rec);
    m_cameraRenderingPipeline.m_rayGenRecordsBuffer.Upload(raygenRecords);
    m_cameraRenderingPipeline.m_sbt.raygenRecord = m_cameraRenderingPipeline.m_rayGenRecordsBuffer.DevicePointer();

    // ------------------------------------------------------------------
    // build miss records
    // ------------------------------------------------------------------
    std::vector<CameraRenderingRayMissRecord> missRecords;
    for (auto &i : m_cameraRenderingPipeline.m_missProgramGroups) {
      CameraRenderingRayMissRecord rec;
      OPTIX_CHECK(optixSbtRecordPackHeader(i.second, &rec));
      rec.data = nullptr; /* for now ... */
      missRecords.push_back(rec);
    }
    m_cameraRenderingPipeline.m_missRecordsBuffer.Upload(missRecords);
    m_cameraRenderingPipeline.m_sbt.missRecordBase = m_cameraRenderingPipeline.m_missRecordsBuffer.DevicePointer();
    m_cameraRenderingPipeline.m_sbt.missRecordStrideInBytes = sizeof(CameraRenderingRayMissRecord);
    m_cameraRenderingPipeline.m_sbt.missRecordCount = static_cast<int>(missRecords.size());

    // ------------------------------------------------------------------
    // build hit records
    // ------------------------------------------------------------------

    // we don't actually have any objects in this example, but let's
    // create a dummy one so the SBT doesn't have any null pointers
    // (which the sanity checks in compilation would complain about)

    std::vector<CameraRenderingRayHitRecord> hitGroupRecords;
    for (auto &instancePair : m_instances) {
      for (int rayID = 0; rayID < static_cast<int>(RayType::RayTypeCount); rayID++) {
        auto &collection = m_cameraRenderingPipeline.m_hitGroupProgramGroups[(RayType)rayID];
        auto &geometry = m_geometries[instancePair.second.m_geometryMapKey];
        auto group = collection[geometry.m_geometryType];
        CameraRenderingRayHitRecord rec;
        rec.data = sBTs[instancePair.first];
        OPTIX_CHECK(optixSbtRecordPackHeader(group, &rec));
        hitGroupRecords.push_back(rec);
      }
    }
    m_cameraRenderingPipeline.m_hitGroupRecordsBuffer.Upload(hitGroupRecords);
    m_cameraRenderingPipeline.m_sbt.hitgroupRecordBase =
        m_cameraRenderingPipeline.m_hitGroupRecordsBuffer.DevicePointer();
    m_cameraRenderingPipeline.m_sbt.hitgroupRecordStrideInBytes = sizeof(CameraRenderingRayHitRecord);
    m_cameraRenderingPipeline.m_sbt.hitgroupRecordCount = static_cast<int>(hitGroupRecords.size());
  }
  {
    // ------------------------------------------------------------------
    // build raygen records
    // ------------------------------------------------------------------
    std::vector<IlluminationEstimationRayGenRecord> raygenRecords;
    IlluminationEstimationRayGenRecord rec;
    OPTIX_CHECK(optixSbtRecordPackHeader(m_illuminationEstimationPipeline.m_rayGenProgramGroups, &rec));
    rec.data = nullptr; /* for now ... */
    raygenRecords.push_back(rec);
    m_illuminationEstimationPipeline.m_rayGenRecordsBuffer.Upload(raygenRecords);
    m_illuminationEstimationPipeline.m_sbt.raygenRecord =
        m_illuminationEstimationPipeline.m_rayGenRecordsBuffer.DevicePointer();

    // ------------------------------------------------------------------
    // build miss records
    // ------------------------------------------------------------------
    std::vector<IlluminationEstimationRayMissRecord> missRecords;
    for (auto &i : m_illuminationEstimationPipeline.m_missProgramGroups) {
      IlluminationEstimationRayMissRecord rec;
      OPTIX_CHECK(optixSbtRecordPackHeader(i.second, &rec));
      rec.data = nullptr; /* for now ... */
      missRecords.push_back(rec);
    }
    m_illuminationEstimationPipeline.m_missRecordsBuffer.Upload(missRecords);
    m_illuminationEstimationPipeline.m_sbt.missRecordBase =
        m_illuminationEstimationPipeline.m_missRecordsBuffer.DevicePointer();
    m_illuminationEstimationPipeline.m_sbt.missRecordStrideInBytes = sizeof(IlluminationEstimationRayMissRecord);
    m_illuminationEstimationPipeline.m_sbt.missRecordCount = static_cast<int>(missRecords.size());

    // ------------------------------------------------------------------
    // build hit records
    // ------------------------------------------------------------------

    // we don't actually have any objects in this example, but let's
    // create a dummy one so the SBT doesn't have any null pointers
    // (which the sanity checks in compilation would complain about)
    std::vector<IlluminationEstimationRayHitRecord> hitGroupRecords;
    for (auto &instancePair : m_instances) {
      for (int rayID = 0; rayID < static_cast<int>(RayType::RayTypeCount); rayID++) {
        auto &collection = m_illuminationEstimationPipeline.m_hitGroupProgramGroups[(RayType)rayID];
        auto &geometry = m_geometries[instancePair.second.m_geometryMapKey];
        auto group = collection[geometry.m_geometryType];
        IlluminationEstimationRayHitRecord rec;
        rec.data = sBTs[instancePair.first];
        OPTIX_CHECK(optixSbtRecordPackHeader(group, &rec));
        hitGroupRecords.push_back(rec);
      }
    }
    m_illuminationEstimationPipeline.m_hitGroupRecordsBuffer.Upload(hitGroupRecords);
    m_illuminationEstimationPipeline.m_sbt.hitgroupRecordBase =
        m_illuminationEstimationPipeline.m_hitGroupRecordsBuffer.DevicePointer();
    m_illuminationEstimationPipeline.m_sbt.hitgroupRecordStrideInBytes = sizeof(IlluminationEstimationRayHitRecord);
    m_illuminationEstimationPipeline.m_sbt.hitgroupRecordCount = static_cast<int>(hitGroupRecords.size());
  }

  {
    // ------------------------------------------------------------------
    // build raygen records
    // ------------------------------------------------------------------
    std::vector<PointCloudScanningRayGenRecord> raygenRecords;
    PointCloudScanningRayGenRecord rec;
    OPTIX_CHECK(optixSbtRecordPackHeader(m_pointCloudScanningPipeline.m_rayGenProgramGroups, &rec));
    rec.data = nullptr; /* for now ... */
    raygenRecords.push_back(rec);
    m_pointCloudScanningPipeline.m_rayGenRecordsBuffer.Upload(raygenRecords);
    m_pointCloudScanningPipeline.m_sbt.raygenRecord =
        m_pointCloudScanningPipeline.m_rayGenRecordsBuffer.DevicePointer();

    // ------------------------------------------------------------------
    // build miss records
    // ------------------------------------------------------------------
    std::vector<PointCloudScanningRayMissRecord> missRecords;
    for (auto &i : m_pointCloudScanningPipeline.m_missProgramGroups) {
      PointCloudScanningRayMissRecord rec;
      OPTIX_CHECK(optixSbtRecordPackHeader(i.second, &rec));
      rec.data = nullptr; /* for now ... */
      missRecords.push_back(rec);
    }
    m_pointCloudScanningPipeline.m_missRecordsBuffer.Upload(missRecords);
    m_pointCloudScanningPipeline.m_sbt.missRecordBase =
        m_pointCloudScanningPipeline.m_missRecordsBuffer.DevicePointer();
    m_pointCloudScanningPipeline.m_sbt.missRecordStrideInBytes = sizeof(PointCloudScanningRayMissRecord);
    m_pointCloudScanningPipeline.m_sbt.missRecordCount = static_cast<int>(missRecords.size());

    // ------------------------------------------------------------------
    // build hit records
    // ------------------------------------------------------------------

    // we don't actually have any objects in this example, but let's
    // create a dummy one so the SBT doesn't have any null pointers
    // (which the sanity checks in compilation would complain about)
    std::vector<PointCloudScanningRayHitRecord> hitGroupRecords;
    for (auto &instancePair : m_instances) {
      for (int rayID = 0; rayID < static_cast<int>(RayType::RayTypeCount); rayID++) {
        auto &collection = m_pointCloudScanningPipeline.m_hitGroupProgramGroups[(RayType)rayID];
        auto &geometry = m_geometries[instancePair.second.m_geometryMapKey];
        auto group = collection[geometry.m_geometryType];
        PointCloudScanningRayHitRecord rec;
        rec.data = sBTs[instancePair.first];
        OPTIX_CHECK(optixSbtRecordPackHeader(group, &rec));
        hitGroupRecords.push_back(rec);
      }
    }
    m_pointCloudScanningPipeline.m_hitGroupRecordsBuffer.Upload(hitGroupRecords);
    m_pointCloudScanningPipeline.m_sbt.hitgroupRecordBase =
        m_pointCloudScanningPipeline.m_hitGroupRecordsBuffer.DevicePointer();
    m_pointCloudScanningPipeline.m_sbt.hitgroupRecordStrideInBytes = sizeof(PointCloudScanningRayHitRecord);
    m_pointCloudScanningPipeline.m_sbt.hitgroupRecordCount = static_cast<int>(hitGroupRecords.size());
  }
}

void RayTracedMaterial::UploadForSBT() {
  switch (m_materialType) {
    case MaterialType::VertexColor: {
      SurfaceMaterial material;
#pragma region Material Settings
      material.m_materialProperties = m_materialProperties;
      if (m_albedoTexture)
        material.m_albedoTexture = m_albedoTexture->m_textureObject;
      else
        material.m_albedoTexture = 0;
      if (m_normalTexture)
        material.normalTexture = m_normalTexture->m_textureObject;
      else
        material.normalTexture = 0;
      if (m_roughnessTexture)
        material.m_roughnessTexture = m_roughnessTexture->m_textureObject;
      else
        material.m_roughnessTexture = 0;
      if (m_metallicTexture)
        material.m_metallicTexture = m_metallicTexture->m_textureObject;
      else
        material.m_metallicTexture = 0;

#pragma endregion
      m_materialBuffer.Upload(&material, 1);
    } break;
    case MaterialType::CompressedBTF: {
      SurfaceCompressedBTF material;
      material.m_btf = *m_btfBase;
      m_materialBuffer.Upload(&material, 1);
    } break;
    case MaterialType::Default: {
      SurfaceMaterial material;
#pragma region Material Settings
      material.m_materialProperties = m_materialProperties;
      if (m_albedoTexture)
        material.m_albedoTexture = m_albedoTexture->m_textureObject;
      else
        material.m_albedoTexture = 0;
      if (m_normalTexture)
        material.normalTexture = m_normalTexture->m_textureObject;
      else
        material.normalTexture = 0;
      if (m_roughnessTexture)
        material.m_roughnessTexture = m_roughnessTexture->m_textureObject;
      else
        material.m_roughnessTexture = 0;
      if (m_metallicTexture)
        material.m_metallicTexture = m_metallicTexture->m_textureObject;
      else
        material.m_metallicTexture = 0;
#pragma endregion
      m_materialBuffer.Upload(&material, 1);
    } break;
  }
}

void RayTracedMaterial::BindTexture(unsigned int id, cudaGraphicsResource_t &graphicsResource,
                                    cudaTextureObject_t &textureObject) {
  cudaArray_t textureArray;
  CUDA_CHECK(GraphicsGLRegisterImage(&graphicsResource, id, GL_TEXTURE_2D, cudaGraphicsRegisterFlagsReadOnly));
  CUDA_CHECK(GraphicsMapResources(1, &graphicsResource, nullptr));
  CUDA_CHECK(GraphicsSubResourceGetMappedArray(&textureArray, graphicsResource, 0, 0));
  struct cudaResourceDesc cudaResourceDesc;
  memset(&cudaResourceDesc, 0, sizeof(cudaResourceDesc));
  cudaResourceDesc.resType = cudaResourceTypeArray;
  cudaResourceDesc.res.array.array = textureArray;
  struct cudaTextureDesc cudaTextureDesc;
  memset(&cudaTextureDesc, 0, sizeof(cudaTextureDesc));
  cudaTextureDesc.addressMode[0] = cudaAddressModeWrap;
  cudaTextureDesc.addressMode[1] = cudaAddressModeWrap;
  cudaTextureDesc.filterMode = cudaFilterModeLinear;
  cudaTextureDesc.readMode = cudaReadModeElementType;
  cudaTextureDesc.normalizedCoords = 1;
  CUDA_CHECK(CreateTextureObject(&textureObject, &cudaResourceDesc, &cudaTextureDesc, nullptr));
}
