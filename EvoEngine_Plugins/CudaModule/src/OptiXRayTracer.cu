#include <OptiXRayTracer.hpp>

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
  const float aspect = static_cast<float>(target_frame.size.x) / static_cast<float>(target_frame.size.y);
  const auto projection = glm::perspective(glm::radians(fov * 0.5f), aspect, 0.1f, 100.f);
  const auto view = glm::lookAt(position, position + newFront, newUp);
  auto inv = glm::inverse(projection * view);
  camera_position = position;
  if (inv != inverse_projection_view)
    modified = true;
  inverse_projection_view = inv;

  const float cosFovY = glm::radians(fov * 0.5f);

  horizontal_direction = cosFovY * aspect * glm::normalize(glm::cross(newFront, newUp));
  vertical_direction = cosFovY * glm::normalize(newUp);
}

void CameraProperties::Resize(const glm::uvec2 &newSize) {
  if (target_frame.size == newSize)
    return;
  target_frame.size = newSize;
  modified = true;
  if (denoiser) {
    OPTIX_CHECK(optixDenoiserDestroy(denoiser));
  };
  // ------------------------------------------------------------------
  // create the denoiser:
  OptixDenoiserOptions denoiserOptions = {};
  OPTIX_CHECK(optixDenoiserCreate(CudaModule::GetRayTracer()->optix_device_context_, OPTIX_DENOISER_MODEL_KIND_LDR,
                                  &denoiserOptions, &denoiser));
  // .. then compute and allocate memory resources for the denoiser
  OptixDenoiserSizes denoiserReturnSizes;
  OPTIX_CHECK(
      optixDenoiserComputeMemoryResources(denoiser, target_frame.size.x, target_frame.size.y, &denoiserReturnSizes));

  denoiser_scratch.Resize(std::max(denoiserReturnSizes.withOverlapScratchSizeInBytes,
                                   denoiserReturnSizes.withoutOverlapScratchSizeInBytes));

  denoiser_state.Resize(denoiserReturnSizes.stateSizeInBytes);
  // ------------------------------------------------------------------
  // resize our cuda frame buffer
  denoised_buffer.Resize(target_frame.size.x * target_frame.size.y * sizeof(glm::vec4));
  frame_buffer_color.Resize(target_frame.size.x * target_frame.size.y * sizeof(glm::vec4));
  frame_buffer_normal.Resize(target_frame.size.x * target_frame.size.y * sizeof(glm::vec4));
  frame_buffer_albedo.Resize(target_frame.size.x * target_frame.size.y * sizeof(glm::vec4));

  // update the launch parameters that we'll pass to the optix
  // launch:
  target_frame.color_buffer = (glm::vec4 *)frame_buffer_color.DevicePointer();
  target_frame.normal_buffer = (glm::vec4 *)frame_buffer_normal.DevicePointer();
  target_frame.albedo_buffer = (glm::vec4 *)frame_buffer_albedo.DevicePointer();

  // ------------------------------------------------------------------
  OPTIX_CHECK(optixDenoiserSetup(denoiser, 0, target_frame.size.x, target_frame.size.y, denoiser_state.DevicePointer(),
                                 denoiser_state.size_in_bytes, denoiser_scratch.DevicePointer(),
                                 denoiser_scratch.size_in_bytes));
}

void CameraProperties::SetFov(float value) {
  modified = true;
  fov = value;
}

const char *OutputTypes[]{"Color", "Normal", "Albedo", "Depth"};

void CameraProperties::OnInspect() {
  if (ImGui::TreeNode("Camera Properties")) {
    if (ImGui::Checkbox("Accumulate", &accumulate)) {
      modified = true;
    }
    if (ImGui::DragFloat("Gamma", &gamma, 0.01f, 0.1f, 5.0f)) {
      SetGamma(gamma);
    }
    int outputType = (int)output_type;
    if (ImGui::Combo("Output Type", &outputType, OutputTypes, IM_ARRAYSIZE(OutputTypes))) {
      output_type = static_cast<OutputType>(outputType);
      modified = true;
    }
    if (ImGui::DragFloat("Max Distance", &max_distance, 0.1f, 0.1f, 10000.0f)) {
      SetMaxDistance(max_distance);
    }

    if (ImGui::DragFloat("FOV", &fov, 1.0f, 1, 359)) {
      SetFov(fov);
    }
    if (ImGui::DragFloat("Aperture", &aperture, 0.0001f, 0.0f, 99999.0f, "%.4f")) {
      SetAperture(aperture);
    }
    if (ImGui::DragFloat("Focal Length", &focal_length, 0.0001f, 0.0f, 99999.0f, "%.4f")) {
      SetFocalLength(focal_length);
    }
    if (ImGui::DragFloat("Denoiser Strength", &denoiser_strength, 0.01f, 0.0f, 1.0f)) {
      SetDenoiserStrength(denoiser_strength);
    }
    ImGui::TreePop();
  }
}

void CameraProperties::SetDenoiserStrength(float value) {
  denoiser_strength = glm::clamp(value, 0.0f, 1.0f);
  modified = true;
}

void CameraProperties::SetGamma(float value) {
  modified = true;
  gamma = value;
}

void CameraProperties::SetOutputType(OutputType value) {
  modified = true;
  output_type = value;
}

void CameraProperties::SetAperture(float value) {
  modified = true;
  aperture = value;
}

void CameraProperties::SetFocalLength(float value) {
  modified = true;
  focal_length = value;
}

void CameraProperties::SetMaxDistance(float value) {
  max_distance = value;
  modified = true;
}

const char *EnvironmentalLightingTypes[]{"Scene", "Skydome", "SingleLightSource"};

void EnvironmentProperties::OnInspect() {
  static int type = 0;
  if (ImGui::Combo("Environment Lighting", &type, EnvironmentalLightingTypes,
                   IM_ARRAYSIZE(EnvironmentalLightingTypes))) {
    environmental_lighting_type = static_cast<EnvironmentalLightingType>(type);
  }
  if (environmental_lighting_type == EnvironmentalLightingType::Skydome) {
    if (ImGui::TreeNodeEx("Atmosphere Settings", ImGuiTreeNodeFlags_DefaultOpen)) {
      if (ImGui::DragFloat("Earth Radius (km)", &atmosphere.earth_radius, 1.0f, 0.0f,
                           atmosphere.atmosphere_radius - 1.0f)) {
        atmosphere.earth_radius = glm::clamp(atmosphere.earth_radius, 1.0f, atmosphere.atmosphere_radius - 1.0f);
      }
      if (ImGui::DragFloat("Atmosphere Radius (km)", &atmosphere.atmosphere_radius, 1.0f,
                           atmosphere.earth_radius + 1.0f, 100000.0f)) {
        atmosphere.atmosphere_radius =
            glm::clamp(atmosphere.atmosphere_radius, atmosphere.earth_radius + 1.0f, 100000.0f);
      }
      if (ImGui::DragFloat("Rayleigh scale height (m)", &atmosphere.hr, 1.0f, 0.0f, 100000.0f)) {
        atmosphere.hr = glm::clamp(atmosphere.hr, 0.0f, 10000.0f);
      }
      if (ImGui::DragFloat("Mie scale height (m)", &atmosphere.hm, 1.0f, 0.0f, 100000.0f)) {
        atmosphere.hm = glm::clamp(atmosphere.hm, 0.0f, 10000.0f);
      }
      if (ImGui::DragFloat("Mie scattering mean cosine", &atmosphere.g, 0.001f, 0.0f, 0.999f, "%.4f")) {
        atmosphere.g = glm::clamp(atmosphere.g, 0.0f, 0.999f);
      }
      if (ImGui::DragInt("Samples", &atmosphere.num_samples, 1, 128)) {
        atmosphere.num_samples = glm::clamp(atmosphere.num_samples, 1, 128);
      }
      if (ImGui::DragInt("Samples light", &atmosphere.num_samples_light, 1, 128)) {
        atmosphere.num_samples_light = glm::clamp(atmosphere.num_samples_light, 1, 128);
      }
      static glm::vec3 angles = glm::vec3(90, 0, 0);
      if (ImGui::DragFloat3("Sun angle", &angles.x, 1.0f)) {
        sun_direction = glm::quat(glm::radians(angles)) * glm::vec3(0, 0, -1);
      }
      ImGui::TreePop();
    }
    if (ImGui::Button("Reset Atmosphere")) {
      atmosphere.earth_radius = 6360;       // In the paper this is usually Rg or Re (radius ground, eart)
      atmosphere.atmosphere_radius = 6420;  // In the paper this is usually R or Ra (radius atmosphere)
      atmosphere.hr = 7994;                 // Thickness of the atmosphere if density was uniform (Hr)
      atmosphere.hm = 1200;                 // Same as above but for Mie scattering (Hm)
      atmosphere.g = 0.76f;                 // Mean cosine for Mie scattering
      atmosphere.num_samples = 16;
      atmosphere.num_samples_light = 8;
    }
  } else if (environmental_lighting_type == EnvironmentalLightingType::SingleLightSource) {
    if (ImGui::DragFloat("Light Size", &light_size, 0.001f, 0.0f, 1.0f)) {
      light_size = glm::clamp(light_size, 0.0f, 1.0f);
    }
    if (ImGui::DragFloat("Ambient light intensity", &ambient_light_intensity, 0.001f, 0.0f, 1.0f)) {
      ambient_light_intensity = glm::clamp(ambient_light_intensity, 0.0f, 1.0f);
    }
    static glm::vec3 angles = glm::vec3(90, 0, 0);
    if (ImGui::DragFloat3("Sun angle", &angles.x, 1.0f)) {
      sun_direction = glm::quat(glm::radians(angles)) * glm::vec3(0, 0, -1);
    }
  }
}

void CameraProperties::SetSkybox(const std::shared_ptr<CudaImage> &cubemap) {
  skybox = cubemap->texture_object;
}

void RayProperties::OnInspect() {
  if (ImGui::TreeNode("Ray Properties")) {
    ImGui::DragInt("bounce limit", &bounces, 1, 1, 8);
    ImGui::DragInt("pixel samples", &samples, 1, 1, 64);
    ImGui::TreePop();
  }
}

void RayTracerProperties::OnInspect() {
  environment.OnInspect();
  ray_properties.OnInspect();
}

bool OptiXRayTracer::RenderToCamera(const EnvironmentProperties &environment_properties,
                                    CameraProperties &camera_properties, const RayProperties &ray_properties) {
  if (camera_properties.target_frame.size.x == 0 | camera_properties.target_frame.size.y == 0)
    return true;
  if (!has_acceleration_structure_)
    return false;
  BuildSbt();
  bool statusChanged = false;
  if (scene_modified)
    statusChanged = true;
  camera_rendering_launch_params_.camera_properties = camera_properties;
  statusChanged = statusChanged || camera_properties.modified;
  camera_properties.modified = false;
  if (camera_rendering_launch_params_.ray_tracer_properties.environment.Changed(environment_properties)) {
    camera_rendering_launch_params_.ray_tracer_properties.environment = environment_properties;
    statusChanged = true;
  }
  if (camera_rendering_launch_params_.ray_tracer_properties.ray_properties.Changed(ray_properties)) {
    camera_rendering_launch_params_.ray_tracer_properties.ray_properties = ray_properties;
    statusChanged = true;
  }
  if (!camera_rendering_launch_params_.camera_properties.accumulate || statusChanged) {
    camera_rendering_launch_params_.camera_properties.target_frame.frame_id = 0;
    camera_properties.target_frame.frame_id = 0;
  }
#pragma region Upload parameters
  camera_rendering_pipeline_.launch_params_buffer.Upload(&camera_rendering_launch_params_, 1);
  camera_rendering_launch_params_.camera_properties.target_frame.frame_id++;
  camera_properties.target_frame.frame_id++;
#pragma endregion
#pragma region Launch rays from camera
  OPTIX_CHECK(optixLaunch(/*! pipeline we're launching launch: */
                          camera_rendering_pipeline_.pipeline, stream_,
                          /*! parameters and SBT */
                          camera_rendering_pipeline_.launch_params_buffer.DevicePointer(),
                          camera_rendering_pipeline_.launch_params_buffer.size_in_bytes,
                          &camera_rendering_pipeline_.sbt,
                          /*! dimensions of the launch: */
                          camera_rendering_launch_params_.camera_properties.target_frame.size.x,
                          camera_rendering_launch_params_.camera_properties.target_frame.size.y, 1));
#pragma endregion
  CUDA_SYNC_CHECK();
#pragma region Bind output texture
  cudaArray_t outputArray;
  CUDA_CHECK(GetMipmappedArrayLevel(
      &outputArray, camera_rendering_launch_params_.camera_properties.target_image->mipmapped_image_array, 0));
#pragma endregion
#pragma region Copy results to output texture
  OptixImage2D inputLayer[3];
  inputLayer[0].data = camera_rendering_launch_params_.camera_properties.frame_buffer_color.DevicePointer();
  /// Width of the image (in pixels)
  inputLayer[0].width = camera_rendering_launch_params_.camera_properties.target_frame.size.x;
  /// Height of the image (in pixels)
  inputLayer[0].height = camera_rendering_launch_params_.camera_properties.target_frame.size.y;
  /// Stride between subsequent rows of the image (in bytes).
  inputLayer[0].rowStrideInBytes =
      camera_rendering_launch_params_.camera_properties.target_frame.size.x * sizeof(glm::vec4);
  /// Stride between subsequent pixels of the image (in bytes).
  /// For now, only 0 or the value that corresponds to a dense packing of pixels
  /// (no gaps) is supported.
  inputLayer[0].pixelStrideInBytes = sizeof(glm::vec4);
  /// Pixel format.
  inputLayer[0].format = OPTIX_PIXEL_FORMAT_FLOAT4;

  // ..................................................................
  inputLayer[1].data = camera_rendering_launch_params_.camera_properties.frame_buffer_albedo.DevicePointer();
  /// Width of the image (in pixels)
  inputLayer[1].width = camera_rendering_launch_params_.camera_properties.target_frame.size.x;
  /// Height of the image (in pixels)
  inputLayer[1].height = camera_rendering_launch_params_.camera_properties.target_frame.size.y;
  /// Stride between subsequent rows of the image (in bytes).
  inputLayer[1].rowStrideInBytes =
      camera_rendering_launch_params_.camera_properties.target_frame.size.x * sizeof(glm::vec4);
  /// Stride between subsequent pixels of the image (in bytes).
  /// For now, only 0 or the value that corresponds to a dense packing of pixels
  /// (no gaps) is supported.
  inputLayer[1].pixelStrideInBytes = sizeof(glm::vec4);
  /// Pixel format.
  inputLayer[1].format = OPTIX_PIXEL_FORMAT_FLOAT4;

  // ..................................................................
  inputLayer[2].data = camera_rendering_launch_params_.camera_properties.frame_buffer_normal.DevicePointer();
  /// Width of the image (in pixels)
  inputLayer[2].width = camera_rendering_launch_params_.camera_properties.target_frame.size.x;
  /// Height of the image (in pixels)
  inputLayer[2].height = camera_rendering_launch_params_.camera_properties.target_frame.size.y;
  /// Stride between subsequent rows of the image (in bytes).
  inputLayer[2].rowStrideInBytes =
      camera_rendering_launch_params_.camera_properties.target_frame.size.x * sizeof(glm::vec4);
  /// Stride between subsequent pixels of the image (in bytes).
  /// For now, only 0 or the value that corresponds to a dense packing of pixels
  /// (no gaps) is supported.
  inputLayer[2].pixelStrideInBytes = sizeof(glm::vec4);
  /// Pixel format.
  inputLayer[2].format = OPTIX_PIXEL_FORMAT_FLOAT4;

  // -------------------------------------------------------
  OptixImage2D outputLayer;
  outputLayer.data = camera_rendering_launch_params_.camera_properties.denoised_buffer.DevicePointer();
  /// Width of the image (in pixels)
  outputLayer.width = camera_rendering_launch_params_.camera_properties.target_frame.size.x;
  /// Height of the image (in pixels)
  outputLayer.height = camera_rendering_launch_params_.camera_properties.target_frame.size.y;
  /// Stride between subsequent rows of the image (in bytes).
  outputLayer.rowStrideInBytes =
      camera_rendering_launch_params_.camera_properties.target_frame.size.x * sizeof(glm::vec4);
  /// Stride between subsequent pixels of the image (in bytes).
  /// For now, only 0 or the value that corresponds to a dense packing of pixels
  /// (no gaps) is supported.
  outputLayer.pixelStrideInBytes = sizeof(glm::vec4);
  /// Pixel format.
  outputLayer.format = OPTIX_PIXEL_FORMAT_FLOAT4;

  switch (camera_rendering_launch_params_.camera_properties.output_type) {
    case OutputType::Color: {
      if (camera_properties.denoiser_strength == 0.0f) {
        CUDA_CHECK(MemcpyToArray(
            outputArray, 0, 0, (void *)camera_rendering_launch_params_.camera_properties.target_frame.color_buffer,
            sizeof(glm::vec4) * camera_rendering_launch_params_.camera_properties.target_frame.size.x *
                camera_rendering_launch_params_.camera_properties.target_frame.size.y,
            cudaMemcpyDeviceToDevice));
      } else {
        OptixDenoiserParams denoiserParams;
        camera_rendering_launch_params_.camera_properties.denoiser_intensity.Resize(sizeof(float));
        if (camera_rendering_launch_params_.camera_properties.denoiser_intensity.size_in_bytes != sizeof(float))
          camera_rendering_launch_params_.camera_properties.denoiser_intensity.Resize(sizeof(float));
        denoiserParams.hdrIntensity =
            camera_rendering_launch_params_.camera_properties.denoiser_intensity.DevicePointer();
        if (camera_rendering_launch_params_.camera_properties.accumulate &&
            camera_rendering_launch_params_.camera_properties.target_frame.frame_id > 1)
          denoiserParams.blendFactor = (1.0f - camera_properties.denoiser_strength) /
                                       camera_rendering_launch_params_.camera_properties.target_frame.frame_id;
        else
          denoiserParams.blendFactor = (1.0f - camera_properties.denoiser_strength);

        OPTIX_CHECK(optixDenoiserComputeIntensity(
            camera_rendering_launch_params_.camera_properties.denoiser,
            /*stream*/ 0, &inputLayer[0],
            (CUdeviceptr)camera_rendering_launch_params_.camera_properties.denoiser_intensity.DevicePointer(),
            (CUdeviceptr)camera_rendering_launch_params_.camera_properties.denoiser_scratch.DevicePointer(),
            camera_rendering_launch_params_.camera_properties.denoiser_scratch.size_in_bytes));

        OptixDenoiserLayer denoiserLayer = {};
        denoiserLayer.input = inputLayer[0];
        denoiserLayer.output = outputLayer;

        OptixDenoiserGuideLayer denoiserGuideLayer = {};
        denoiserGuideLayer.albedo = inputLayer[1];
        denoiserGuideLayer.normal = inputLayer[2];

        OPTIX_CHECK(optixDenoiserInvoke(
            camera_rendering_launch_params_.camera_properties.denoiser,
            /*stream*/ 0, &denoiserParams,
            camera_rendering_launch_params_.camera_properties.denoiser_state.DevicePointer(),
            camera_rendering_launch_params_.camera_properties.denoiser_state.size_in_bytes, &denoiserGuideLayer,
            &denoiserLayer, 1,
            /*inputOffsetX*/ 0,
            /*inputOffsetY*/ 0, camera_rendering_launch_params_.camera_properties.denoiser_scratch.DevicePointer(),
            camera_rendering_launch_params_.camera_properties.denoiser_scratch.size_in_bytes));
        CUDA_CHECK(
            MemcpyToArray(outputArray, 0, 0, (void *)outputLayer.data,
                          sizeof(glm::vec4) * camera_rendering_launch_params_.camera_properties.target_frame.size.x *
                              camera_rendering_launch_params_.camera_properties.target_frame.size.y,
                          cudaMemcpyDeviceToDevice));
      }
    } break;
    case OutputType::Normal: {
      CUDA_CHECK(MemcpyToArray(
          outputArray, 0, 0, (void *)camera_rendering_launch_params_.camera_properties.target_frame.normal_buffer,
          sizeof(glm::vec4) * camera_rendering_launch_params_.camera_properties.target_frame.size.x *
              camera_rendering_launch_params_.camera_properties.target_frame.size.y,
          cudaMemcpyDeviceToDevice));
    } break;
    case OutputType::Albedo: {
      CUDA_CHECK(MemcpyToArray(
          outputArray, 0, 0, (void *)camera_rendering_launch_params_.camera_properties.target_frame.albedo_buffer,
          sizeof(glm::vec4) * camera_rendering_launch_params_.camera_properties.target_frame.size.x *
              camera_rendering_launch_params_.camera_properties.target_frame.size.y,
          cudaMemcpyDeviceToDevice));
    } break;
    case OutputType::Depth: {
      CUDA_CHECK(MemcpyToArray(
          outputArray, 0, 0, (void *)camera_rendering_launch_params_.camera_properties.target_frame.albedo_buffer,
          sizeof(glm::vec4) * camera_rendering_launch_params_.camera_properties.target_frame.size.x *
              camera_rendering_launch_params_.camera_properties.target_frame.size.y,
          cudaMemcpyDeviceToDevice));
    } break;
  }

#pragma endregion
  return true;
}

void OptiXRayTracer::EstimateIllumination(const size_t &size, const EnvironmentProperties &environment_properties,
                                          const RayProperties &ray_properties, CudaBuffer &light_probes, unsigned seed,
                                          float push_normal_distance) {
  if (!has_acceleration_structure_)
    return;
  if (size == 0) {
    std::cout << "Error: Lightprobe is empty" << std::endl;
    return;
  }
  BuildSbt();

  illumination_estimation_launch_params_.ray_tracer_properties.environment = environment_properties;
  illumination_estimation_launch_params_.ray_tracer_properties.ray_properties = ray_properties;
#pragma region Upload parameters
  illumination_estimation_launch_params_.seed = seed;
  illumination_estimation_launch_params_.push_normal_distance = push_normal_distance;
  illumination_estimation_launch_params_.size = size;
  illumination_estimation_launch_params_.ray_tracer_properties.environment = environment_properties;
  illumination_estimation_launch_params_.ray_tracer_properties.ray_properties = ray_properties;
  illumination_estimation_launch_params_.light_probes =
      reinterpret_cast<IlluminationSampler<glm::vec3> *>(light_probes.DevicePointer());
  illumination_estimation_pipeline_.launch_params_buffer.Upload(&illumination_estimation_launch_params_, 1);
#pragma endregion
#pragma region Launch rays from light probes
  OPTIX_CHECK(optixLaunch(/*! pipeline we're launching launch: */
                          illumination_estimation_pipeline_.pipeline, stream_,
                          /*! parameters and SBT */
                          illumination_estimation_pipeline_.launch_params_buffer.DevicePointer(),
                          illumination_estimation_pipeline_.launch_params_buffer.size_in_bytes,
                          &illumination_estimation_pipeline_.sbt,
                          /*! dimensions of the launch: */
                          size, 1, 1));
  CUDA_SYNC_CHECK();
#pragma endregion
}

void OptiXRayTracer::ScanPointCloud(const size_t &size, const EnvironmentProperties &environment_properties,
                                    CudaBuffer &samples) {
  if (!has_acceleration_structure_)
    return;
  if (size == 0) {
    std::cout << "Error: Samples is empty" << std::endl;
    return;
  }
  BuildSbt();
#pragma region Upload parameters
  point_cloud_scanning_launch_params_.size = size;
  point_cloud_scanning_launch_params_.samples = reinterpret_cast<PointCloudSample *>(samples.DevicePointer());
  point_cloud_scanning_pipeline_.launch_params_buffer.Upload(&point_cloud_scanning_launch_params_, 1);
#pragma endregion
#pragma region Launch rays from samples
  OPTIX_CHECK(optixLaunch(/*! pipeline we're launching launch: */
                          point_cloud_scanning_pipeline_.pipeline, stream_,
                          /*! parameters and SBT */
                          point_cloud_scanning_pipeline_.launch_params_buffer.DevicePointer(),
                          point_cloud_scanning_pipeline_.launch_params_buffer.size_in_bytes,
                          &point_cloud_scanning_pipeline_.sbt,
                          /*! dimensions of the launch: */
                          size, 1, 1));
  CUDA_SYNC_CHECK();
#pragma endregion
}

OptiXRayTracer::OptiXRayTracer() {
  camera_rendering_launch_params_.camera_properties.target_frame.frame_id = 0;
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

OptiXRayTracer::~OptiXRayTracer() {
  materials.clear();
  geometries.clear();
  instances.clear();
}

static void context_log_cb(const unsigned int level, const char *tag, const char *message, void *) {
  fprintf(stderr, "[%2d][%12s]: %s\n", static_cast<int>(level), tag, message);
}

void printLogMessage(unsigned int level, const char *tag, const char *message, void * /* cbdata */) {
  std::cerr << "[" << std::setw(2) << level << "][" << std::setw(12) << tag << "]: " << message << std::endl;
}

void OptiXRayTracer::CreateContext() {
  // for this sample, do everything on one device
  const int deviceID = 0;
  CUDA_CHECK(StreamCreate(&stream_));
  CUDA_CHECK(GetDeviceProperties(&device_props_, deviceID));
  std::cout << "#Optix: running on device: " << device_props_.name << std::endl;
  const CUresult cuRes = cuCtxGetCurrent(&cuda_context_);
  if (cuRes != CUDA_SUCCESS)
    fprintf(stderr, "Error querying current context: error code %d\n", cuRes);

  OptixDeviceContextOptions options = {};
  options.logCallbackFunction = &printLogMessage;
  options.logCallbackLevel = 4;
  // options.validationMode = OPTIX_DEVICE_CONTEXT_VALIDATION_MODE_ALL;

  OPTIX_CHECK(optixDeviceContextCreate(cuda_context_, &options, &optix_device_context_));
  OPTIX_CHECK(optixDeviceContextSetLogCallback(optix_device_context_, context_log_cb, nullptr, 4));
}

extern "C" char CAMERA_RENDERING_PTX[];
extern "C" char ILLUMINATION_ESTIMATION_PTX[];
extern "C" char POINT_CLOUD_SCANNING_PTX[];

void OptiXRayTracer::CreateModules() {
  CreateModule(camera_rendering_pipeline_, CAMERA_RENDERING_PTX, "cameraRenderingLaunchParams");
  CreateModule(illumination_estimation_pipeline_, ILLUMINATION_ESTIMATION_PTX, "illuminationEstimationLaunchParams");
  CreateModule(point_cloud_scanning_pipeline_, POINT_CLOUD_SCANNING_PTX, "pointCloudScanningLaunchParams");
}

void OptiXRayTracer::CreateRayGenPrograms() {
  CreateRayGenProgram(camera_rendering_pipeline_, "__raygen__CR");
  CreateRayGenProgram(illumination_estimation_pipeline_, "__raygen__IE");
  CreateRayGenProgram(point_cloud_scanning_pipeline_, "__raygen__PCS");
}

void OptiXRayTracer::CreateMissPrograms() {
  {
    char log[2048];
    size_t sizeofLog = sizeof(log);

    OptixProgramGroupOptions pgOptions = {};
    OptixProgramGroupDesc pgDesc = {};
    pgDesc.kind = OPTIX_PROGRAM_GROUP_KIND_MISS;
    pgDesc.miss.module = camera_rendering_pipeline_.module;

    // ------------------------------------------------------------------
    // radiance rays
    // ------------------------------------------------------------------
    pgDesc.miss.entryFunctionName = "__miss__CR_R";

    OPTIX_CHECK(optixProgramGroupCreate(optix_device_context_, &pgDesc, 1, &pgOptions, log, &sizeofLog,
                                        &camera_rendering_pipeline_.miss_program_groups[RayType::Radiance]));
    if (sizeofLog > 1)
      std::cout << log << std::endl;
    // ------------------------------------------------------------------
    // BSSRDF Spatial sampler rays
    // ------------------------------------------------------------------
    pgDesc.miss.entryFunctionName = "__miss__CR_SS";
    OPTIX_CHECK(optixProgramGroupCreate(optix_device_context_, &pgDesc, 1, &pgOptions, log, &sizeofLog,
                                        &camera_rendering_pipeline_.miss_program_groups[RayType::SpacialSampling]));
    if (sizeofLog > 1)
      std::cout << log << std::endl;
  }
  {
    char log[2048];
    size_t sizeofLog = sizeof(log);

    OptixProgramGroupOptions pgOptions = {};
    OptixProgramGroupDesc pgDesc = {};
    pgDesc.kind = OPTIX_PROGRAM_GROUP_KIND_MISS;
    pgDesc.miss.module = illumination_estimation_pipeline_.module;

    // ------------------------------------------------------------------
    // radiance rays
    // ------------------------------------------------------------------
    pgDesc.miss.entryFunctionName = "__miss__IE_R";

    OPTIX_CHECK(optixProgramGroupCreate(optix_device_context_, &pgDesc, 1, &pgOptions, log, &sizeofLog,
                                        &illumination_estimation_pipeline_.miss_program_groups[RayType::Radiance]));
    if (sizeofLog > 1)
      std::cout << log << std::endl;
    // ------------------------------------------------------------------
    // BSSRDF Spatial sampler rays
    // ------------------------------------------------------------------
    pgDesc.miss.entryFunctionName = "__miss__IE_SS";
    OPTIX_CHECK(
        optixProgramGroupCreate(optix_device_context_, &pgDesc, 1, &pgOptions, log, &sizeofLog,
                                &illumination_estimation_pipeline_.miss_program_groups[RayType::SpacialSampling]));
    if (sizeofLog > 1)
      std::cout << log << std::endl;
  }
  {
    char log[2048];
    size_t sizeofLog = sizeof(log);

    OptixProgramGroupOptions pgOptions = {};
    OptixProgramGroupDesc pgDesc = {};
    pgDesc.kind = OPTIX_PROGRAM_GROUP_KIND_MISS;
    pgDesc.miss.module = point_cloud_scanning_pipeline_.module;

    // ------------------------------------------------------------------
    // radiance rays
    // ------------------------------------------------------------------
    pgDesc.miss.entryFunctionName = "__miss__PCS_R";

    OPTIX_CHECK(optixProgramGroupCreate(optix_device_context_, &pgDesc, 1, &pgOptions, log, &sizeofLog,
                                        &point_cloud_scanning_pipeline_.miss_program_groups[RayType::Radiance]));
    if (sizeofLog > 1)
      std::cout << log << std::endl;
    // ------------------------------------------------------------------
    // BSSRDF Spatial sampler rays
    // ------------------------------------------------------------------
    pgDesc.miss.entryFunctionName = "__miss__PCS_SS";
    OPTIX_CHECK(optixProgramGroupCreate(optix_device_context_, &pgDesc, 1, &pgOptions, log, &sizeofLog,
                                        &point_cloud_scanning_pipeline_.miss_program_groups[RayType::SpacialSampling]));
    if (sizeofLog > 1)
      std::cout << log << std::endl;
  }
}

void OptiXRayTracer::CreateHitGroupPrograms() {
  {
    char log[2048];
    size_t sizeofLog = sizeof(log);

    OptixProgramGroupOptions pgOptions = {};
    OptixProgramGroupDesc pgDesc = {};
    pgDesc.kind = OPTIX_PROGRAM_GROUP_KIND_HITGROUP;
    pgDesc.hitgroup.moduleCH = camera_rendering_pipeline_.module;
    pgDesc.hitgroup.moduleAH = camera_rendering_pipeline_.module;

    // -------------------------------------------------------
    // radiance rays
    // -------------------------------------------------------
    pgDesc.hitgroup.entryFunctionNameCH = "__closesthit__CR_R";
    pgDesc.hitgroup.entryFunctionNameAH = "__anyhit__CR_R";
    pgDesc.hitgroup.entryFunctionNameIS = 0;
    OPTIX_CHECK(optixProgramGroupCreate(
        optix_device_context_, &pgDesc, 1, &pgOptions, log, &sizeofLog,
        &camera_rendering_pipeline_.hit_group_program_groups[RayType::Radiance][PrimitiveType::Triangle]));

    pgDesc.hitgroup.moduleIS = camera_rendering_pipeline_.linear_curve_module;
    OPTIX_CHECK(optixProgramGroupCreate(
        optix_device_context_, &pgDesc, 1, &pgOptions, log, &sizeofLog,
        &camera_rendering_pipeline_.hit_group_program_groups[RayType::Radiance][PrimitiveType::Linear]));

    pgDesc.hitgroup.moduleIS = camera_rendering_pipeline_.quadratic_curve_module;
    OPTIX_CHECK(optixProgramGroupCreate(
        optix_device_context_, &pgDesc, 1, &pgOptions, log, &sizeofLog,
        &camera_rendering_pipeline_.hit_group_program_groups[RayType::Radiance][PrimitiveType::QuadraticBSpline]));

    pgDesc.hitgroup.moduleIS = camera_rendering_pipeline_.cubic_curve_module;
    OPTIX_CHECK(optixProgramGroupCreate(
        optix_device_context_, &pgDesc, 1, &pgOptions, log, &sizeofLog,
        &camera_rendering_pipeline_.hit_group_program_groups[RayType::Radiance][PrimitiveType::CubicBSpline]));

    if (sizeofLog > 1)
      std::cout << log << std::endl;

    // -------------------------------------------------------
    // BSSRDF Sampler ray
    // -------------------------------------------------------
    pgDesc.hitgroup.entryFunctionNameCH = "__closesthit__CR_SS";
    pgDesc.hitgroup.entryFunctionNameAH = "__anyhit__CR_SS";
    OPTIX_CHECK(optixProgramGroupCreate(
        optix_device_context_, &pgDesc, 1, &pgOptions, log, &sizeofLog,
        &camera_rendering_pipeline_.hit_group_program_groups[RayType::SpacialSampling][PrimitiveType::Triangle]));

    pgDesc.hitgroup.moduleIS = camera_rendering_pipeline_.linear_curve_module;
    OPTIX_CHECK(optixProgramGroupCreate(
        optix_device_context_, &pgDesc, 1, &pgOptions, log, &sizeofLog,
        &camera_rendering_pipeline_.hit_group_program_groups[RayType::SpacialSampling][PrimitiveType::Linear]));
    ;

    pgDesc.hitgroup.moduleIS = camera_rendering_pipeline_.quadratic_curve_module;
    OPTIX_CHECK(optixProgramGroupCreate(
        optix_device_context_, &pgDesc, 1, &pgOptions, log, &sizeofLog,
        &camera_rendering_pipeline_
             .hit_group_program_groups[RayType::SpacialSampling][PrimitiveType::QuadraticBSpline]));

    pgDesc.hitgroup.moduleIS = camera_rendering_pipeline_.cubic_curve_module;
    OPTIX_CHECK(optixProgramGroupCreate(
        optix_device_context_, &pgDesc, 1, &pgOptions, log, &sizeofLog,
        &camera_rendering_pipeline_.hit_group_program_groups[RayType::SpacialSampling][PrimitiveType::CubicBSpline]));

    if (sizeofLog > 1)
      std::cout << log << std::endl;
  }
  {
    char log[2048];
    size_t sizeofLog = sizeof(log);

    OptixProgramGroupOptions pgOptions = {};
    OptixProgramGroupDesc pgDesc = {};
    pgDesc.kind = OPTIX_PROGRAM_GROUP_KIND_HITGROUP;
    pgDesc.hitgroup.moduleCH = illumination_estimation_pipeline_.module;
    pgDesc.hitgroup.moduleAH = illumination_estimation_pipeline_.module;
    // -------------------------------------------------------
    // radiance rays
    // -------------------------------------------------------
    pgDesc.hitgroup.entryFunctionNameCH = "__closesthit__IE_R";
    pgDesc.hitgroup.entryFunctionNameAH = "__anyhit__IE_R";
    OPTIX_CHECK(optixProgramGroupCreate(
        optix_device_context_, &pgDesc, 1, &pgOptions, log, &sizeofLog,
        &illumination_estimation_pipeline_.hit_group_program_groups[RayType::Radiance][PrimitiveType::Triangle]));

    pgDesc.hitgroup.moduleIS = illumination_estimation_pipeline_.linear_curve_module;
    OPTIX_CHECK(optixProgramGroupCreate(
        optix_device_context_, &pgDesc, 1, &pgOptions, log, &sizeofLog,
        &illumination_estimation_pipeline_.hit_group_program_groups[RayType::Radiance][PrimitiveType::Linear]));

    pgDesc.hitgroup.moduleIS = illumination_estimation_pipeline_.quadratic_curve_module;
    OPTIX_CHECK(
        optixProgramGroupCreate(optix_device_context_, &pgDesc, 1, &pgOptions, log, &sizeofLog,
                                &illumination_estimation_pipeline_
                                     .hit_group_program_groups[RayType::Radiance][PrimitiveType::QuadraticBSpline]));

    pgDesc.hitgroup.moduleIS = illumination_estimation_pipeline_.cubic_curve_module;
    OPTIX_CHECK(optixProgramGroupCreate(
        optix_device_context_, &pgDesc, 1, &pgOptions, log, &sizeofLog,
        &illumination_estimation_pipeline_.hit_group_program_groups[RayType::Radiance][PrimitiveType::CubicBSpline]));
    if (sizeofLog > 1)
      std::cout << log << std::endl;
    // -------------------------------------------------------
    // BSSRDF Sampler ray
    // -------------------------------------------------------
    pgDesc.hitgroup.entryFunctionNameCH = "__closesthit__IE_SS";
    pgDesc.hitgroup.entryFunctionNameAH = "__anyhit__IE_SS";
    OPTIX_CHECK(
        optixProgramGroupCreate(optix_device_context_, &pgDesc, 1, &pgOptions, log, &sizeofLog,
                                &illumination_estimation_pipeline_
                                     .hit_group_program_groups[RayType::SpacialSampling][PrimitiveType::Triangle]));

    pgDesc.hitgroup.moduleIS = illumination_estimation_pipeline_.linear_curve_module;
    OPTIX_CHECK(optixProgramGroupCreate(
        optix_device_context_, &pgDesc, 1, &pgOptions, log, &sizeofLog,
        &illumination_estimation_pipeline_.hit_group_program_groups[RayType::SpacialSampling][PrimitiveType::Linear]));

    pgDesc.hitgroup.moduleIS = illumination_estimation_pipeline_.quadratic_curve_module;
    OPTIX_CHECK(optixProgramGroupCreate(
        optix_device_context_, &pgDesc, 1, &pgOptions, log, &sizeofLog,
        &illumination_estimation_pipeline_
             .hit_group_program_groups[RayType::SpacialSampling][PrimitiveType::QuadraticBSpline]));

    pgDesc.hitgroup.moduleIS = illumination_estimation_pipeline_.cubic_curve_module;
    OPTIX_CHECK(
        optixProgramGroupCreate(optix_device_context_, &pgDesc, 1, &pgOptions, log, &sizeofLog,
                                &illumination_estimation_pipeline_
                                     .hit_group_program_groups[RayType::SpacialSampling][PrimitiveType::CubicBSpline]));
    if (sizeofLog > 1)
      std::cout << log << std::endl;
  }
  {
    char log[2048];
    size_t sizeofLog = sizeof(log);

    OptixProgramGroupOptions pgOptions = {};
    OptixProgramGroupDesc pgDesc = {};
    pgDesc.kind = OPTIX_PROGRAM_GROUP_KIND_HITGROUP;
    pgDesc.hitgroup.moduleCH = point_cloud_scanning_pipeline_.module;
    pgDesc.hitgroup.moduleAH = point_cloud_scanning_pipeline_.module;
    // -------------------------------------------------------
    // radiance rays
    // -------------------------------------------------------
    pgDesc.hitgroup.entryFunctionNameCH = "__closesthit__PCS_R";
    pgDesc.hitgroup.entryFunctionNameAH = "__anyhit__PCS_R";
    OPTIX_CHECK(optixProgramGroupCreate(
        optix_device_context_, &pgDesc, 1, &pgOptions, log, &sizeofLog,
        &point_cloud_scanning_pipeline_.hit_group_program_groups[RayType::Radiance][PrimitiveType::Triangle]));

    pgDesc.hitgroup.moduleIS = point_cloud_scanning_pipeline_.linear_curve_module;
    OPTIX_CHECK(optixProgramGroupCreate(
        optix_device_context_, &pgDesc, 1, &pgOptions, log, &sizeofLog,
        &point_cloud_scanning_pipeline_.hit_group_program_groups[RayType::Radiance][PrimitiveType::Linear]));

    pgDesc.hitgroup.moduleIS = point_cloud_scanning_pipeline_.quadratic_curve_module;
    OPTIX_CHECK(optixProgramGroupCreate(
        optix_device_context_, &pgDesc, 1, &pgOptions, log, &sizeofLog,
        &point_cloud_scanning_pipeline_.hit_group_program_groups[RayType::Radiance][PrimitiveType::QuadraticBSpline]));

    pgDesc.hitgroup.moduleIS = point_cloud_scanning_pipeline_.cubic_curve_module;
    OPTIX_CHECK(optixProgramGroupCreate(
        optix_device_context_, &pgDesc, 1, &pgOptions, log, &sizeofLog,
        &point_cloud_scanning_pipeline_.hit_group_program_groups[RayType::Radiance][PrimitiveType::CubicBSpline]));
    if (sizeofLog > 1)
      std::cout << log << std::endl;
    // -------------------------------------------------------
    // BSSRDF Sampler ray
    // -------------------------------------------------------
    pgDesc.hitgroup.entryFunctionNameCH = "__closesthit__PCS_SS";
    pgDesc.hitgroup.entryFunctionNameAH = "__anyhit__PCS_SS";
    OPTIX_CHECK(optixProgramGroupCreate(
        optix_device_context_, &pgDesc, 1, &pgOptions, log, &sizeofLog,
        &point_cloud_scanning_pipeline_.hit_group_program_groups[RayType::SpacialSampling][PrimitiveType::Triangle]));

    pgDesc.hitgroup.moduleIS = point_cloud_scanning_pipeline_.linear_curve_module;
    OPTIX_CHECK(optixProgramGroupCreate(
        optix_device_context_, &pgDesc, 1, &pgOptions, log, &sizeofLog,
        &point_cloud_scanning_pipeline_.hit_group_program_groups[RayType::SpacialSampling][PrimitiveType::Linear]));

    pgDesc.hitgroup.moduleIS = point_cloud_scanning_pipeline_.quadratic_curve_module;
    OPTIX_CHECK(optixProgramGroupCreate(
        optix_device_context_, &pgDesc, 1, &pgOptions, log, &sizeofLog,
        &point_cloud_scanning_pipeline_
             .hit_group_program_groups[RayType::SpacialSampling][PrimitiveType::QuadraticBSpline]));

    pgDesc.hitgroup.moduleIS = point_cloud_scanning_pipeline_.cubic_curve_module;
    OPTIX_CHECK(
        optixProgramGroupCreate(optix_device_context_, &pgDesc, 1, &pgOptions, log, &sizeofLog,
                                &point_cloud_scanning_pipeline_
                                     .hit_group_program_groups[RayType::SpacialSampling][PrimitiveType::CubicBSpline]));
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
        matrices[idx / verticesSize].instance_matrix * glm::vec4(vertices[idx % verticesSize].position, 1.0f);
    targetPositions[idx] = position;
    glm::vec3 N = glm::normalize(matrices[idx / verticesSize].instance_matrix *
                                 glm::vec4(vertices[idx % verticesSize].normal, 0.0f));
    glm::vec3 T = glm::normalize(matrices[idx / verticesSize].instance_matrix *
                                 glm::vec4(vertices[idx % verticesSize].tangent, 0.0f));
    T = glm::normalize(T - dot(T, N) * N);
    targetVertices[idx] = {};
    targetVertices[idx].position = position;
    targetVertices[idx].tangent = T;
    targetVertices[idx].normal = N;
    targetVertices[idx].tex_coord = vertices[idx % verticesSize].tex_coord;
    targetVertices[idx].color = matrices[idx / verticesSize].instance_color;
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

void RayTracedGeometry::BuildGas(const OptixDeviceContext &context) {
#pragma region Clean previous buffer
  vertex_data_buffer.Free();
  triangle_buffer.Free();

  curve_strand_u_buffer.Free();
  curve_strand_i_buffer.Free();
  curve_strand_info_buffer.Free();

  accelerated_structure_buffer.Free();
#pragma endregion

  CudaBuffer devicePositionBuffer;
  CudaBuffer deviceWidthBuffer;

#pragma region Geometry Inputs
  // ==================================================================
  // geometry inputs
  // ==================================================================
  OptixBuildInput buildInput;
  const uint32_t triangleInputFlags[1] = {OPTIX_GEOMETRY_FLAG_NONE};
  switch (renderer_type) {
    case RendererType::Curve: {
      CUdeviceptr devicePoints;
      CUdeviceptr deviceWidths;
      CUdeviceptr deviceStrands;

      // curve_strand_u_buffer.Upload(*m_strandU);
      // curve_strand_i_buffer.Upload(*m_strandIndices);
      // curve_strand_info_buffer.Upload(*m_strandInfos);

      deviceWidthBuffer.Resize(curve_points->size() * sizeof(float));
      vertex_data_buffer.Upload(*curve_points);
      triangle_buffer.Upload(*curve_segments);

      int blockSize = 0;    // The launch configurator returned block size
      int minGridSize = 0;  // The minimum grid size needed to achieve the
      // maximum occupancy for a full device launch
      int gridSize = 0;  // The actual grid size needed, based on input size
      int size = curve_points->size();
      cudaOccupancyMaxPotentialBlockSize(&minGridSize, &blockSize, CopyStrandPointsKernel, 0, size);
      gridSize = (size + blockSize - 1) / blockSize;
      CopyStrandPointsKernel<<<gridSize, blockSize>>>(size,
                                                      static_cast<evo_engine::StrandPoint *>(vertex_data_buffer.d_ptr),
                                                      static_cast<float *>(deviceWidthBuffer.d_ptr));
      CUDA_SYNC_CHECK();

      buildInput.type = OPTIX_BUILD_INPUT_TYPE_CURVES;
      switch (geometry_type) {
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
      devicePoints = vertex_data_buffer.DevicePointer();
      deviceWidths = deviceWidthBuffer.DevicePointer();
      deviceStrands = triangle_buffer.DevicePointer();
      buildInput.curveArray.numPrimitives = curve_segments->size();
      buildInput.curveArray.vertexBuffers = &devicePoints;
      buildInput.curveArray.numVertices = static_cast<unsigned int>(curve_points->size());
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

      vertex_data_buffer.Upload(*vertices);
      int blockSize = 0;    // The launch configurator returned block size
      int minGridSize = 0;  // The minimum grid size needed to achieve the
      // maximum occupancy for a full device launch
      int gridSize = 0;  // The actual grid size needed, based on input size
      int size = vertices->size();
      cudaOccupancyMaxPotentialBlockSize(&minGridSize, &blockSize, CopyVerticesKernel, 0, size);
      gridSize = (size + blockSize - 1) / blockSize;
      CUDA_SYNC_CHECK();
      triangle_buffer.Upload(*triangles);

      buildInput = {};
      buildInput.type = OPTIX_BUILD_INPUT_TYPE_TRIANGLES;

      // create local variables, because we need a *pointer* to the
      // device pointers
      deviceVertexPositions = vertex_data_buffer.DevicePointer();
      deviceVertexTriangles = triangle_buffer.DevicePointer();

      buildInput.triangleArray.vertexFormat = OPTIX_VERTEX_FORMAT_FLOAT3;
      buildInput.triangleArray.vertexStrideInBytes = sizeof(evo_engine::Vertex);
      buildInput.triangleArray.numVertices = static_cast<int>(vertices->size());
      buildInput.triangleArray.vertexBuffers = &deviceVertexPositions;

      buildInput.triangleArray.indexFormat = OPTIX_INDICES_FORMAT_UNSIGNED_INT3;
      buildInput.triangleArray.indexStrideInBytes = sizeof(glm::uvec3);
      buildInput.triangleArray.numIndexTriplets = static_cast<int>(triangle_buffer.size_in_bytes / sizeof(glm::uvec3));
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
      skinnedVerticesBuffer.Upload(*skinned_vertices);
      boneMatricesBuffer.Upload(*bone_matrices);
      vertex_data_buffer.Resize(skinned_vertices->size() * sizeof(evo_engine::Vertex));
      devicePositionBuffer.Resize(skinned_vertices->size() * sizeof(glm::vec3));
      int blockSize = 0;    // The launch configurator returned block size
      int minGridSize = 0;  // The minimum grid size needed to achieve the
      // maximum occupancy for a full device launch
      int gridSize = 0;  // The actual grid size needed, based on input size
      int size = skinned_vertices->size();
      cudaOccupancyMaxPotentialBlockSize(&minGridSize, &blockSize, CopySkinnedVerticesKernel, 0, size);
      gridSize = (size + blockSize - 1) / blockSize;
      CopySkinnedVerticesKernel<<<gridSize, blockSize>>>(
          size, static_cast<evo_engine::SkinnedVertex *>(skinnedVerticesBuffer.d_ptr),
          static_cast<glm::mat4 *>(boneMatricesBuffer.d_ptr), static_cast<glm::vec3 *>(devicePositionBuffer.d_ptr),
          static_cast<evo_engine::Vertex *>(vertex_data_buffer.d_ptr));
      CUDA_SYNC_CHECK();
      triangle_buffer.Upload(*triangles);
      buildInput = {};
      buildInput.type = OPTIX_BUILD_INPUT_TYPE_TRIANGLES;
      // create local variables, because we need a *pointer* to the
      // device pointers
      deviceVertexPositions = devicePositionBuffer.DevicePointer();
      deviceVertexTriangles = triangle_buffer.DevicePointer();
      buildInput.triangleArray.vertexFormat = OPTIX_VERTEX_FORMAT_FLOAT3;
      buildInput.triangleArray.vertexStrideInBytes = sizeof(glm::vec3);
      buildInput.triangleArray.numVertices = static_cast<int>(devicePositionBuffer.size_in_bytes / sizeof(glm::vec3));
      buildInput.triangleArray.vertexBuffers = &deviceVertexPositions;
      buildInput.triangleArray.indexFormat = OPTIX_INDICES_FORMAT_UNSIGNED_INT3;
      buildInput.triangleArray.indexStrideInBytes = sizeof(glm::uvec3);
      buildInput.triangleArray.numIndexTriplets = static_cast<int>(triangle_buffer.size_in_bytes / sizeof(glm::uvec3));
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
      verticesBuffer.Upload(*vertices);
      instanceMatricesBuffer.Upload(*instance_matrices);
      vertex_data_buffer.Resize(instance_matrices->size() * vertices->size() * sizeof(evo_engine::Vertex));

      devicePositionBuffer.Resize(instance_matrices->size() * vertices->size() * sizeof(glm::vec3));
      int blockSize = 0;    // The launch configurator returned block verticesSize
      int minGridSize = 0;  // The minimum grid verticesSize needed to achieve the
      // maximum occupancy for a full device launch
      int gridSize = 0;  // The actual grid verticesSize needed, based on input verticesSize
      int verticesSize = vertices->size();
      int matricesSize = instance_matrices->size();
      int size = verticesSize * matricesSize;
      cudaOccupancyMaxPotentialBlockSize(&minGridSize, &blockSize, CopyVerticesInstancedKernel, 0, size);
      gridSize = (size + blockSize - 1) / blockSize;
      CopyVerticesInstancedKernel<<<gridSize, blockSize>>>(
          matricesSize, verticesSize, static_cast<InstanceMatrix *>(instanceMatricesBuffer.d_ptr),
          static_cast<Vertex *>(verticesBuffer.d_ptr), static_cast<glm::vec3 *>(devicePositionBuffer.d_ptr),
          static_cast<Vertex *>(vertex_data_buffer.d_ptr));
      CUDA_SYNC_CHECK();
      auto triangles = std::vector<glm::uvec3>();
      triangles.resize(this->triangles->size() * instance_matrices->size());
      unsigned offset = 0;
      for (const auto &matrix : *instance_matrices) {
        for (const auto &i : *this->triangles) {
          triangles.push_back(i);
          triangles.back() += glm::uvec3(offset);
        }
        offset += vertices->size();
      }
      triangle_buffer.Upload(triangles);
      buildInput = {};
      buildInput.type = OPTIX_BUILD_INPUT_TYPE_TRIANGLES;
      // create local variables, because we need a *pointer* to the
      // device pointers
      deviceVertexPositions = devicePositionBuffer.DevicePointer();
      deviceVertexTriangles = triangle_buffer.DevicePointer();
      buildInput.triangleArray.vertexFormat = OPTIX_VERTEX_FORMAT_FLOAT3;
      buildInput.triangleArray.vertexStrideInBytes = sizeof(glm::vec3);
      buildInput.triangleArray.numVertices = static_cast<int>(devicePositionBuffer.size_in_bytes / sizeof(glm::vec3));
      buildInput.triangleArray.vertexBuffers = &deviceVertexPositions;
      buildInput.triangleArray.indexFormat = OPTIX_INDICES_FORMAT_UNSIGNED_INT3;
      buildInput.triangleArray.indexStrideInBytes = sizeof(glm::uvec3);
      buildInput.triangleArray.numIndexTriplets = static_cast<int>(triangle_buffer.size_in_bytes / sizeof(glm::uvec3));
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
                              &traversable_handle, &emitDesc, 1));
  CUDA_SYNC_CHECK();
#pragma endregion
#pragma region Perform compaction
  // ==================================================================
  // perform compaction
  // ==================================================================
  uint64_t compactedSize;
  compactedSizeBuffer.Download(&compactedSize, 1);
  accelerated_structure_buffer.Resize(compactedSize);
  OPTIX_CHECK(optixAccelCompact(context,
                                /*stream:*/ nullptr, traversable_handle, accelerated_structure_buffer.DevicePointer(),
                                accelerated_structure_buffer.size_in_bytes, &traversable_handle));
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
  update_flag = false;
}

void RayTracedGeometry::UploadForSbt() {
  geometry_buffer.Free();
  if (geometry_type != PrimitiveType::Triangle) {
    Curves curves;
    curves.strand_points = reinterpret_cast<evo_engine::StrandPoint *>(vertex_data_buffer.DevicePointer());
    // curves.m_strandU = reinterpret_cast<glm::vec2 *>(curve_strand_u_buffer.DevicePointer());
    // curves.m_strandIndices = reinterpret_cast<int *>(curve_strand_i_buffer.DevicePointer());
    // curves.m_strandInfos = reinterpret_cast<glm::uvec2 *>(curve_strand_info_buffer.DevicePointer());
    curves.segments = reinterpret_cast<int *>(triangle_buffer.DevicePointer());
    geometry_buffer.Upload(&curves, 1);
  } else {
    TriangularMesh mesh;
    mesh.vertices = reinterpret_cast<evo_engine::Vertex *>(vertex_data_buffer.DevicePointer());
    mesh.triangles = reinterpret_cast<glm::uvec3 *>(triangle_buffer.DevicePointer());
    geometry_buffer.Upload(&mesh, 1);
  }
}

void OptiXRayTracer::BuildIas() {
  std::vector<uint64_t> removeQueue;
  for (const auto &i : geometries) {
    if (i.second.remove_flag) {
      removeQueue.emplace_back(i.first);
    }
  }
  for (auto &i : removeQueue) {
    auto &geometry = geometries.at(i);
    geometry.geometry_buffer.Free();
    geometry.vertex_data_buffer.Free();
    geometry.triangle_buffer.Free();

    geometry.curve_strand_u_buffer.Free();
    geometry.curve_strand_i_buffer.Free();
    geometry.curve_strand_info_buffer.Free();

    geometry.accelerated_structure_buffer.Free();
    geometries.erase(i);
  }
  for (auto &i : geometries) {
    if (i.second.update_flag) {
      i.second.BuildGas(optix_device_context_);
      i.second.UploadForSbt();
    }
  }
  removeQueue.clear();
  for (const auto &i : instances) {
    if (i.second.remove_flag) {
      removeQueue.emplace_back(i.first);
    }
  }
  for (auto &i : removeQueue) {
    instances.erase(i);
  }

  std::vector<OptixInstance> optixInstances;
  unsigned int sbtOffset = 0;

  OptixInstance optixInstance = {};
  // Common optixInstance settings
  optixInstance.instanceId = 0;
  optixInstance.visibilityMask = 0xFF;
  optixInstance.flags = OPTIX_INSTANCE_FLAG_NONE;

  for (auto &instance : instances) {
    glm::mat3x4 transform = glm::transpose(instance.second.global_transform);
    memcpy(optixInstance.transform, &transform, sizeof(glm::mat3x4));
    optixInstance.sbtOffset = sbtOffset;
    optixInstance.traversableHandle = geometries.at(instance.second.geometry_map_key).traversable_handle;
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
  OPTIX_CHECK(optixAccelComputeMemoryUsage(optix_device_context_, &accelBuildOptions, &buildInput,
                                           1,  // Number of build inputs
                                           &bufferSizesIAS));

  CudaBuffer deviceTempBufferIAS;
  deviceTempBufferIAS.Resize(bufferSizesIAS.tempSizeInBytes);
  ias_buffer_.Resize(bufferSizesIAS.outputSizeInBytes);

  OptixTraversableHandle iASHandle = 0;
  OPTIX_CHECK(optixAccelBuild(optix_device_context_,
                              0,  // CUDA stream
                              &accelBuildOptions, &buildInput,
                              1,  // num build inputs
                              deviceTempBufferIAS.DevicePointer(), bufferSizesIAS.tempSizeInBytes,
                              ias_buffer_.DevicePointer(), bufferSizesIAS.outputSizeInBytes, &iASHandle,
                              nullptr,  // emitted property list
                              0));      // num emitted properties
  deviceTempInstances.Free();
  deviceTempBufferIAS.Free();

  camera_rendering_launch_params_.traversable = iASHandle;
  illumination_estimation_launch_params_.traversable = iASHandle;
  point_cloud_scanning_launch_params_.traversable = iASHandle;
  has_acceleration_structure_ = true;
  scene_modified = true;
}

void OptiXRayTracer::AssemblePipelines() {
  AssemblePipeline(camera_rendering_pipeline_);
  AssemblePipeline(illumination_estimation_pipeline_);
  AssemblePipeline(point_cloud_scanning_pipeline_);
}

void OptiXRayTracer::CreateRayGenProgram(RayTracerPipeline &targetPipeline, char entryFunctionName[]) const {
  OptixProgramGroupOptions pgOptions = {};
  OptixProgramGroupDesc pgDesc = {};
  pgDesc.kind = OPTIX_PROGRAM_GROUP_KIND_RAYGEN;
  pgDesc.raygen.module = targetPipeline.module;
  pgDesc.raygen.entryFunctionName = entryFunctionName;
  char log[2048];
  size_t sizeofLog = sizeof(log);
  OPTIX_CHECK(optixProgramGroupCreate(optix_device_context_, &pgDesc, 1, &pgOptions, log, &sizeofLog,
                                      &targetPipeline.ray_gen_program_groups));
  if (sizeofLog > 1)
    std::cout << log << std::endl;
}

void OptiXRayTracer::CreateModule(RayTracerPipeline &targetPipeline, char ptxCode[], char launchParamsName[]) const {
  targetPipeline.launch_params_name = launchParamsName;

  targetPipeline.module_compile_options.maxRegisterCount = 50;
  targetPipeline.module_compile_options.optLevel = OPTIX_COMPILE_OPTIMIZATION_DEFAULT;
  targetPipeline.module_compile_options.debugLevel = OPTIX_COMPILE_DEBUG_LEVEL_NONE;

  targetPipeline.pipeline_compile_options = {};
  targetPipeline.pipeline_compile_options.traversableGraphFlags = OPTIX_TRAVERSABLE_GRAPH_FLAG_ALLOW_ANY;
  targetPipeline.pipeline_compile_options.usesMotionBlur = false;
  targetPipeline.pipeline_compile_options.numPayloadValues = 2;
  targetPipeline.pipeline_compile_options.numAttributeValues = 2;
  targetPipeline.pipeline_compile_options.exceptionFlags = OPTIX_EXCEPTION_FLAG_NONE;
  targetPipeline.pipeline_compile_options.pipelineLaunchParamsVariableName = launchParamsName;
  targetPipeline.pipeline_compile_options.usesPrimitiveTypeFlags =
      OPTIX_PRIMITIVE_TYPE_FLAGS_TRIANGLE | OPTIX_PRIMITIVE_TYPE_FLAGS_ROUND_LINEAR |
      OPTIX_PRIMITIVE_TYPE_FLAGS_ROUND_QUADRATIC_BSPLINE | OPTIX_PRIMITIVE_TYPE_FLAGS_ROUND_CUBIC_BSPLINE;

  const std::string code = ptxCode;

  char log[2048];
  size_t sizeof_log = sizeof(log);
  OPTIX_CHECK(optixModuleCreate(optix_device_context_, &targetPipeline.module_compile_options,
                                &targetPipeline.pipeline_compile_options, code.c_str(), code.size(), log, &sizeof_log,
                                &targetPipeline.module));

  OptixBuiltinISOptions builtinISOptions = {};
  builtinISOptions.builtinISModuleType = OPTIX_PRIMITIVE_TYPE_ROUND_QUADRATIC_BSPLINE;
  builtinISOptions.curveEndcapFlags = OPTIX_CURVE_ENDCAP_ON;
  OPTIX_CHECK(optixBuiltinISModuleGet(optix_device_context_, &targetPipeline.module_compile_options,
                                      &targetPipeline.pipeline_compile_options, &builtinISOptions,
                                      &targetPipeline.quadratic_curve_module));

  builtinISOptions.builtinISModuleType = OPTIX_PRIMITIVE_TYPE_ROUND_CUBIC_BSPLINE;
  OPTIX_CHECK(optixBuiltinISModuleGet(optix_device_context_, &targetPipeline.module_compile_options,
                                      &targetPipeline.pipeline_compile_options, &builtinISOptions,
                                      &targetPipeline.cubic_curve_module));

  builtinISOptions.builtinISModuleType = OPTIX_PRIMITIVE_TYPE_ROUND_LINEAR;
  OPTIX_CHECK(optixBuiltinISModuleGet(optix_device_context_, &targetPipeline.module_compile_options,
                                      &targetPipeline.pipeline_compile_options, &builtinISOptions,
                                      &targetPipeline.linear_curve_module));

  if (sizeof_log > 1)
    std::cout << log << std::endl;
}

void OptiXRayTracer::AssemblePipeline(RayTracerPipeline &targetPipeline) const {
  std::vector<OptixProgramGroup> programGroups;
  programGroups.push_back(targetPipeline.ray_gen_program_groups);
  for (auto &i : targetPipeline.miss_program_groups)
    programGroups.push_back(i.second);
  for (auto &i : targetPipeline.hit_group_program_groups)
    for (auto &j : i.second)
      programGroups.push_back(j.second);

  const uint32_t maxTraceDepth = 31;
  targetPipeline.pipeline_link_options.maxTraceDepth = maxTraceDepth;
  char log[2048];
  size_t sizeofLog = sizeof(log);
  OPTIX_CHECK(optixPipelineCreate(optix_device_context_, &targetPipeline.pipeline_compile_options,
                                  &targetPipeline.pipeline_link_options, programGroups.data(),
                                  static_cast<int>(programGroups.size()), log, &sizeofLog, &targetPipeline.pipeline));
  if (sizeofLog > 1)
    std::cout << log << std::endl;

  OptixStackSizes stackSizes = {};
  for (auto &progGroup : programGroups) {
    OPTIX_CHECK(optixUtilAccumulateStackSizes(progGroup, &stackSizes, targetPipeline.pipeline));
  }

  uint32_t directCallableStackSizeFromTraversal;
  uint32_t directCallableStackSizeFromState;
  uint32_t continuationStackSize;
  OPTIX_CHECK(optixUtilComputeStackSizes(&stackSizes, maxTraceDepth,
                                         0,  // maxCCDepth
                                         0,  // maxDCDEpth
                                         &directCallableStackSizeFromTraversal, &directCallableStackSizeFromState,
                                         &continuationStackSize));
  OPTIX_CHECK(optixPipelineSetStackSize(targetPipeline.pipeline, directCallableStackSizeFromTraversal,
                                        directCallableStackSizeFromState, continuationStackSize,
                                        2  // maxTraversableDepth
                                        ));
  if (sizeofLog > 1)
    std::cout << log << std::endl;
}

void OptiXRayTracer::BuildSbt() {
  std::vector<uint64_t> removeQueue;
  for (auto &i : materials) {
    auto &material = i.second;
    material.material_buffer.Free();
    if (material.remove_flag) {
      removeQueue.emplace_back(i.first);
    } else {
      material.UploadForSbt();
    }
  }
  for (auto &i : removeQueue) {
    auto &material = materials.at(i);
    materials.erase(i);
  }
#pragma region Prepare SBTs
  std::map<uint64_t, SBT> sBTs;
  for (auto &instancePair : instances) {
    auto &instance = instancePair.second;
    auto &material = materials.at(instance.material_map_key);
    auto &geometry = geometries.at(instance.geometry_map_key);
    auto &sBT = sBTs[instancePair.first];
    sBT.handle = instance.private_component_handle;
    sBT.global_transform = instance.global_transform;
    sBT.geometry_type = geometry.renderer_type;
    sBT.geometry = reinterpret_cast<void *>(geometry.geometry_buffer.DevicePointer());
    sBT.material_type = material.material_type;
    sBT.material = reinterpret_cast<void *>(material.material_buffer.DevicePointer());
  }
#pragma endregion
  {
    // ------------------------------------------------------------------
    // build raygen records
    // ------------------------------------------------------------------
    std::vector<CameraRenderingRayGenRecord> raygenRecords;
    CameraRenderingRayGenRecord rec;
    OPTIX_CHECK(optixSbtRecordPackHeader(camera_rendering_pipeline_.ray_gen_program_groups, &rec));
    rec.data = nullptr; /* for now ... */
    raygenRecords.push_back(rec);
    camera_rendering_pipeline_.ray_gen_records_buffer.Upload(raygenRecords);
    camera_rendering_pipeline_.sbt.raygenRecord = camera_rendering_pipeline_.ray_gen_records_buffer.DevicePointer();

    // ------------------------------------------------------------------
    // build miss records
    // ------------------------------------------------------------------
    std::vector<CameraRenderingRayMissRecord> missRecords;
    for (auto &i : camera_rendering_pipeline_.miss_program_groups) {
      CameraRenderingRayMissRecord rec;
      OPTIX_CHECK(optixSbtRecordPackHeader(i.second, &rec));
      rec.data = nullptr; /* for now ... */
      missRecords.push_back(rec);
    }
    camera_rendering_pipeline_.miss_records_buffer.Upload(missRecords);
    camera_rendering_pipeline_.sbt.missRecordBase = camera_rendering_pipeline_.miss_records_buffer.DevicePointer();
    camera_rendering_pipeline_.sbt.missRecordStrideInBytes = sizeof(CameraRenderingRayMissRecord);
    camera_rendering_pipeline_.sbt.missRecordCount = static_cast<int>(missRecords.size());

    // ------------------------------------------------------------------
    // build hit records
    // ------------------------------------------------------------------

    // we don't actually have any objects in this example, but let's
    // create a dummy one so the SBT doesn't have any null pointers
    // (which the sanity checks in compilation would complain about)

    std::vector<CameraRenderingRayHitRecord> hitGroupRecords;
    for (auto &instancePair : instances) {
      for (int rayID = 0; rayID < static_cast<int>(RayType::RayTypeCount); rayID++) {
        auto &collection = camera_rendering_pipeline_.hit_group_program_groups[(RayType)rayID];
        auto &geometry = geometries[instancePair.second.geometry_map_key];
        auto group = collection[geometry.geometry_type];
        CameraRenderingRayHitRecord rec;
        rec.data = sBTs[instancePair.first];
        OPTIX_CHECK(optixSbtRecordPackHeader(group, &rec));
        hitGroupRecords.push_back(rec);
      }
    }
    camera_rendering_pipeline_.hit_group_records_buffer.Upload(hitGroupRecords);
    camera_rendering_pipeline_.sbt.hitgroupRecordBase =
        camera_rendering_pipeline_.hit_group_records_buffer.DevicePointer();
    camera_rendering_pipeline_.sbt.hitgroupRecordStrideInBytes = sizeof(CameraRenderingRayHitRecord);
    camera_rendering_pipeline_.sbt.hitgroupRecordCount = static_cast<int>(hitGroupRecords.size());
  }
  {
    // ------------------------------------------------------------------
    // build raygen records
    // ------------------------------------------------------------------
    std::vector<IlluminationEstimationRayGenRecord> raygenRecords;
    IlluminationEstimationRayGenRecord rec;
    OPTIX_CHECK(optixSbtRecordPackHeader(illumination_estimation_pipeline_.ray_gen_program_groups, &rec));
    rec.data = nullptr; /* for now ... */
    raygenRecords.push_back(rec);
    illumination_estimation_pipeline_.ray_gen_records_buffer.Upload(raygenRecords);
    illumination_estimation_pipeline_.sbt.raygenRecord =
        illumination_estimation_pipeline_.ray_gen_records_buffer.DevicePointer();

    // ------------------------------------------------------------------
    // build miss records
    // ------------------------------------------------------------------
    std::vector<IlluminationEstimationRayMissRecord> missRecords;
    for (auto &i : illumination_estimation_pipeline_.miss_program_groups) {
      IlluminationEstimationRayMissRecord rec;
      OPTIX_CHECK(optixSbtRecordPackHeader(i.second, &rec));
      rec.data = nullptr; /* for now ... */
      missRecords.push_back(rec);
    }
    illumination_estimation_pipeline_.miss_records_buffer.Upload(missRecords);
    illumination_estimation_pipeline_.sbt.missRecordBase =
        illumination_estimation_pipeline_.miss_records_buffer.DevicePointer();
    illumination_estimation_pipeline_.sbt.missRecordStrideInBytes = sizeof(IlluminationEstimationRayMissRecord);
    illumination_estimation_pipeline_.sbt.missRecordCount = static_cast<int>(missRecords.size());

    // ------------------------------------------------------------------
    // build hit records
    // ------------------------------------------------------------------

    // we don't actually have any objects in this example, but let's
    // create a dummy one so the SBT doesn't have any null pointers
    // (which the sanity checks in compilation would complain about)
    std::vector<IlluminationEstimationRayHitRecord> hitGroupRecords;
    for (auto &instancePair : instances) {
      for (int rayID = 0; rayID < static_cast<int>(RayType::RayTypeCount); rayID++) {
        auto &collection = illumination_estimation_pipeline_.hit_group_program_groups[(RayType)rayID];
        auto &geometry = geometries[instancePair.second.geometry_map_key];
        auto group = collection[geometry.geometry_type];
        IlluminationEstimationRayHitRecord rec;
        rec.data = sBTs[instancePair.first];
        OPTIX_CHECK(optixSbtRecordPackHeader(group, &rec));
        hitGroupRecords.push_back(rec);
      }
    }
    illumination_estimation_pipeline_.hit_group_records_buffer.Upload(hitGroupRecords);
    illumination_estimation_pipeline_.sbt.hitgroupRecordBase =
        illumination_estimation_pipeline_.hit_group_records_buffer.DevicePointer();
    illumination_estimation_pipeline_.sbt.hitgroupRecordStrideInBytes = sizeof(IlluminationEstimationRayHitRecord);
    illumination_estimation_pipeline_.sbt.hitgroupRecordCount = static_cast<int>(hitGroupRecords.size());
  }

  {
    // ------------------------------------------------------------------
    // build raygen records
    // ------------------------------------------------------------------
    std::vector<PointCloudScanningRayGenRecord> raygenRecords;
    PointCloudScanningRayGenRecord rec;
    OPTIX_CHECK(optixSbtRecordPackHeader(point_cloud_scanning_pipeline_.ray_gen_program_groups, &rec));
    rec.data = nullptr; /* for now ... */
    raygenRecords.push_back(rec);
    point_cloud_scanning_pipeline_.ray_gen_records_buffer.Upload(raygenRecords);
    point_cloud_scanning_pipeline_.sbt.raygenRecord =
        point_cloud_scanning_pipeline_.ray_gen_records_buffer.DevicePointer();

    // ------------------------------------------------------------------
    // build miss records
    // ------------------------------------------------------------------
    std::vector<PointCloudScanningRayMissRecord> missRecords;
    for (auto &i : point_cloud_scanning_pipeline_.miss_program_groups) {
      PointCloudScanningRayMissRecord rec;
      OPTIX_CHECK(optixSbtRecordPackHeader(i.second, &rec));
      rec.data = nullptr; /* for now ... */
      missRecords.push_back(rec);
    }
    point_cloud_scanning_pipeline_.miss_records_buffer.Upload(missRecords);
    point_cloud_scanning_pipeline_.sbt.missRecordBase =
        point_cloud_scanning_pipeline_.miss_records_buffer.DevicePointer();
    point_cloud_scanning_pipeline_.sbt.missRecordStrideInBytes = sizeof(PointCloudScanningRayMissRecord);
    point_cloud_scanning_pipeline_.sbt.missRecordCount = static_cast<int>(missRecords.size());

    // ------------------------------------------------------------------
    // build hit records
    // ------------------------------------------------------------------

    // we don't actually have any objects in this example, but let's
    // create a dummy one so the SBT doesn't have any null pointers
    // (which the sanity checks in compilation would complain about)
    std::vector<PointCloudScanningRayHitRecord> hitGroupRecords;
    for (auto &instancePair : instances) {
      for (int rayID = 0; rayID < static_cast<int>(RayType::RayTypeCount); rayID++) {
        auto &collection = point_cloud_scanning_pipeline_.hit_group_program_groups[(RayType)rayID];
        auto &geometry = geometries[instancePair.second.geometry_map_key];
        auto group = collection[geometry.geometry_type];
        PointCloudScanningRayHitRecord rec;
        rec.data = sBTs[instancePair.first];
        OPTIX_CHECK(optixSbtRecordPackHeader(group, &rec));
        hitGroupRecords.push_back(rec);
      }
    }
    point_cloud_scanning_pipeline_.hit_group_records_buffer.Upload(hitGroupRecords);
    point_cloud_scanning_pipeline_.sbt.hitgroupRecordBase =
        point_cloud_scanning_pipeline_.hit_group_records_buffer.DevicePointer();
    point_cloud_scanning_pipeline_.sbt.hitgroupRecordStrideInBytes = sizeof(PointCloudScanningRayHitRecord);
    point_cloud_scanning_pipeline_.sbt.hitgroupRecordCount = static_cast<int>(hitGroupRecords.size());
  }
}

void RayTracedMaterial::UploadForSbt() {
  switch (material_type) {
    case MaterialType::VertexColor: {
      SurfaceMaterial material;
#pragma region Material Settings
      material.material_properties = material_properties;
      if (albedo_texture)
        material.albedo_texture = albedo_texture->texture_object;
      else
        material.albedo_texture = 0;
      if (normal_texture)
        material.normal_texture = normal_texture->texture_object;
      else
        material.normal_texture = 0;
      if (roughness_texture)
        material.roughness_texture = roughness_texture->texture_object;
      else
        material.roughness_texture = 0;
      if (metallic_texture)
        material.metallic_texture = metallic_texture->texture_object;
      else
        material.metallic_texture = 0;

#pragma endregion
      material_buffer.Upload(&material, 1);
    } break;
    case MaterialType::CompressedBTF: {
      SurfaceCompressedBtf material;
      material.btf = *btf_base;
      material_buffer.Upload(&material, 1);
    } break;
    case MaterialType::Default: {
      SurfaceMaterial material;
#pragma region Material Settings
      material.material_properties = material_properties;
      if (albedo_texture)
        material.albedo_texture = albedo_texture->texture_object;
      else
        material.albedo_texture = 0;
      if (normal_texture)
        material.normal_texture = normal_texture->texture_object;
      else
        material.normal_texture = 0;
      if (roughness_texture)
        material.roughness_texture = roughness_texture->texture_object;
      else
        material.roughness_texture = 0;
      if (metallic_texture)
        material.metallic_texture = metallic_texture->texture_object;
      else
        material.metallic_texture = 0;
#pragma endregion
      material_buffer.Upload(&material, 1);
    } break;
  }
}

void RayTracedMaterial::BindTexture(unsigned int id, cudaGraphicsResource_t &graphics_resource,
                                    cudaTextureObject_t &texture_object) {
  cudaArray_t textureArray;
  CUDA_CHECK(GraphicsGLRegisterImage(&graphics_resource, id, GL_TEXTURE_2D, cudaGraphicsRegisterFlagsReadOnly));
  CUDA_CHECK(GraphicsMapResources(1, &graphics_resource, nullptr));
  CUDA_CHECK(GraphicsSubResourceGetMappedArray(&textureArray, graphics_resource, 0, 0));
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
  CUDA_CHECK(CreateTextureObject(&texture_object, &cudaResourceDesc, &cudaTextureDesc, nullptr));
}
