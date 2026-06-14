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

#include "Console.hpp"
using namespace evo_engine;

void CameraProperties::Set(const glm::vec3 &position, const glm::quat &rotation) {
  const auto new_front = glm::normalize(rotation * glm::vec3(0, 0, -1));
  const auto new_up = glm::normalize(rotation * glm::vec3(0, 1, 0));
  const float aspect = static_cast<float>(target_frame.size.x) / static_cast<float>(target_frame.size.y);
  const auto projection = glm::perspective(glm::radians(fov * 0.5f), aspect, 0.1f, 100.f);
  const auto view = glm::lookAt(position, position + new_front, new_up);
  const auto inv_proj_view = glm::inverse(projection * view);
  camera_position = position;
  if (inv_proj_view != inverse_projection_view)
    modified = true;
  inverse_projection_view = inv_proj_view;

  const float cos_fov_y = glm::radians(fov * 0.5f);

  horizontal_direction = cos_fov_y * aspect * glm::normalize(glm::cross(new_front, new_up));
  vertical_direction = cos_fov_y * glm::normalize(new_up);
}

void CameraProperties::Resize(const glm::uvec2 &new_size) {
  if (target_frame.size == new_size)
    return;
  target_frame.size = new_size;
  modified = true;
  // ------------------------------------------------------------------
  // resize our cuda frame buffer

  frame_buffer_color.Resize(target_frame.size.x * target_frame.size.y * sizeof(glm::vec4));
  frame_buffer_normal.Resize(target_frame.size.x * target_frame.size.y * sizeof(glm::vec4));
  frame_buffer_albedo.Resize(target_frame.size.x * target_frame.size.y * sizeof(glm::vec4));

  // update the launch parameters that we'll pass to the optix
  // launch:
  target_frame.color_buffer = reinterpret_cast<glm::vec4 *>(frame_buffer_color.DevicePointer());
  target_frame.normal_buffer = reinterpret_cast<glm::vec4 *>(frame_buffer_normal.DevicePointer());
  target_frame.albedo_buffer = reinterpret_cast<glm::vec4 *>(frame_buffer_albedo.DevicePointer());

#if ENABLE_OPTIX_DENOISER
  if (denoiser) {
    OPTIX_CHECK(optixDenoiserDestroy(denoiser));
  };
  // ------------------------------------------------------------------
  // create the denoiser:
  constexpr OptixDenoiserOptions denoiser_options = {};
  OPTIX_CHECK(optixDenoiserCreate(CudaModule::GetRayTracer()->optix_device_context_, OPTIX_DENOISER_MODEL_KIND_LDR,
                                  &denoiser_options, &denoiser));
  // .. then compute and allocate memory resources for the denoiser
  OptixDenoiserSizes denoiser_return_sizes;
  OPTIX_CHECK(
      optixDenoiserComputeMemoryResources(denoiser, target_frame.size.x, target_frame.size.y, &denoiser_return_sizes));

  denoiser_scratch.Resize(std::max(denoiser_return_sizes.withOverlapScratchSizeInBytes,
                                   denoiser_return_sizes.withoutOverlapScratchSizeInBytes));
  denoised_buffer.Resize(target_frame.size.x * target_frame.size.y * sizeof(glm::vec4));
  denoiser_state.Resize(denoiser_return_sizes.stateSizeInBytes);

  // ------------------------------------------------------------------
  OPTIX_CHECK(optixDenoiserSetup(denoiser, nullptr, target_frame.size.x, target_frame.size.y,
                                 denoiser_state.DevicePointer(), denoiser_state.size_in_bytes,
                                 denoiser_scratch.DevicePointer(), denoiser_scratch.size_in_bytes));
#endif
}

void CameraProperties::SetFov(const float value) {
  modified = true;
  fov = value;
}

void CameraProperties::DrawGui() {
  if (ImGui::TreeNode("Camera Properties")) {
    if (ImGui::Checkbox("Accumulate", &accumulate)) {
      modified = true;
    }
    if (ImGui::DragFloat("Gamma", &gamma, 0.01f, 0.1f, 5.0f)) {
      SetGamma(gamma);
    }
    const char *output_types[]{"Color", "Normal", "Albedo", "Depth"};
    int type = static_cast<int>(output_type);
    if (ImGui::Combo("Output Type", &type, output_types, IM_ARRAYSIZE(output_types))) {
      output_type = static_cast<OutputType>(type);
      modified = true;
    }

    const char *background_types[]{"Environment", "Skybox", "Color"};
    int bg_type = static_cast<int>(background_type);
    if (ImGui::Combo("Background Type", &bg_type, background_types, IM_ARRAYSIZE(background_types))) {
      background_type = static_cast<BackgroundType>(bg_type);
      modified = true;
    }
    if (background_type == BackgroundType::Color) {
      if (ImGui::ColorEdit4("Background color", &background_color.x)) {
        modified = true;
      }
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
#if ENABLE_OPTIX_DENOISER
    if (ImGui::DragFloat("Denoiser Strength", &denoiser_strength, 0.01f, 0.0f, 1.0f)) {
      SetDenoiserStrength(denoiser_strength);
    }
#endif
    ImGui::TreePop();
  }
}
void CameraProperties::SetDenoiserStrength(const float value) {
  denoiser_strength = glm::clamp(value, 0.0f, 1.0f);
  modified = true;
}

void CameraProperties::SetGamma(const float value) {
  modified = true;
  gamma = value;
}

void CameraProperties::SetOutputType(const OutputType value) {
  modified = true;
  output_type = value;
}

void CameraProperties::SetBackgroundType(const BackgroundType background_type) {
  modified = true;
  this->background_type = background_type;
}
void CameraProperties::SetBackgroundColor(const glm::vec4 &background_color) {
  modified = true;
  this->background_color = background_color;
}
void CameraProperties::SetAperture(const float value) {
  modified = true;
  aperture = value;
}

void CameraProperties::SetFocalLength(const float value) {
  modified = true;
  focal_length = value;
}

void CameraProperties::SetMaxDistance(const float value) {
  max_distance = value;
  modified = true;
}

void EnvironmentProperties::DrawGui() {
  static int type = 0;
  const char *environmental_lighting_types[]{"Scene", "Skydome", "SingleLightSource"};
  if (ImGui::Combo("Environment Lighting", &type, environmental_lighting_types,
                   IM_ARRAYSIZE(environmental_lighting_types))) {
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
      static auto angles = glm::vec3(90, 0, 0);
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

void RayProperties::DrawGui() {
  if (ImGui::TreeNode("Ray Properties")) {
    ImGui::DragInt("bounce limit", &bounces, 1, 1, 8);
    ImGui::DragInt("pixel samples", &samples, 1, 1, 64);
    ImGui::TreePop();
  }
}

void RayTracerProperties::DrawGui() {
  environment.DrawGui();
  ray_properties.DrawGui();
}

bool OptiXRayTracer::RenderToCamera(const EnvironmentProperties &environment_properties,
                                    CameraProperties &camera_properties, const RayProperties &ray_properties) {
  if (camera_properties.target_frame.size.x == 0 | camera_properties.target_frame.size.y == 0)
    return true;
  if (!has_acceleration_structure_)
    return false;
  BuildSbt();
  bool status_changed = false;
  if (scene_modified)
    status_changed = true;
  camera_rendering_launch_params_.camera_properties = camera_properties;
  status_changed = status_changed || camera_properties.modified;
  camera_properties.modified = false;
  if (camera_rendering_launch_params_.ray_tracer_properties.environment.Changed(environment_properties)) {
    camera_rendering_launch_params_.ray_tracer_properties.environment = environment_properties;
    status_changed = true;
  }
  if (camera_rendering_launch_params_.ray_tracer_properties.ray_properties.Changed(ray_properties)) {
    camera_rendering_launch_params_.ray_tracer_properties.ray_properties = ray_properties;
    status_changed = true;
  }
  if (!camera_rendering_launch_params_.camera_properties.accumulate || status_changed) {
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
  cudaArray_t output_array;
  CUDA_CHECK(GetMipmappedArrayLevel(
      &output_array, camera_rendering_launch_params_.camera_properties.target_image->mipmapped_image_array, 0));
#pragma endregion
#pragma region Copy results to output texture
  OptixImage2D input_layer[3];
  input_layer[0].data = camera_rendering_launch_params_.camera_properties.frame_buffer_color.DevicePointer();
  /// Width of the image (in pixels)
  input_layer[0].width = camera_rendering_launch_params_.camera_properties.target_frame.size.x;
  /// Height of the image (in pixels)
  input_layer[0].height = camera_rendering_launch_params_.camera_properties.target_frame.size.y;
  /// Stride between subsequent rows of the image (in bytes).
  input_layer[0].rowStrideInBytes =
      camera_rendering_launch_params_.camera_properties.target_frame.size.x * sizeof(glm::vec4);
  /// Stride between subsequent pixels of the image (in bytes).
  /// For now, only 0 or the value that corresponds to a dense packing of pixels
  /// (no gaps) is supported.
  input_layer[0].pixelStrideInBytes = sizeof(glm::vec4);
  /// Pixel format.
  input_layer[0].format = OPTIX_PIXEL_FORMAT_FLOAT4;

  // ..................................................................
  input_layer[1].data = camera_rendering_launch_params_.camera_properties.frame_buffer_albedo.DevicePointer();
  /// Width of the image (in pixels)
  input_layer[1].width = camera_rendering_launch_params_.camera_properties.target_frame.size.x;
  /// Height of the image (in pixels)
  input_layer[1].height = camera_rendering_launch_params_.camera_properties.target_frame.size.y;
  /// Stride between subsequent rows of the image (in bytes).
  input_layer[1].rowStrideInBytes =
      camera_rendering_launch_params_.camera_properties.target_frame.size.x * sizeof(glm::vec4);
  /// Stride between subsequent pixels of the image (in bytes).
  /// For now, only 0 or the value that corresponds to a dense packing of pixels
  /// (no gaps) is supported.
  input_layer[1].pixelStrideInBytes = sizeof(glm::vec4);
  /// Pixel format.
  input_layer[1].format = OPTIX_PIXEL_FORMAT_FLOAT4;

  // ..................................................................
  input_layer[2].data = camera_rendering_launch_params_.camera_properties.frame_buffer_normal.DevicePointer();
  /// Width of the image (in pixels)
  input_layer[2].width = camera_rendering_launch_params_.camera_properties.target_frame.size.x;
  /// Height of the image (in pixels)
  input_layer[2].height = camera_rendering_launch_params_.camera_properties.target_frame.size.y;
  /// Stride between subsequent rows of the image (in bytes).
  input_layer[2].rowStrideInBytes =
      camera_rendering_launch_params_.camera_properties.target_frame.size.x * sizeof(glm::vec4);
  /// Stride between subsequent pixels of the image (in bytes).
  /// For now, only 0 or the value that corresponds to a dense packing of pixels
  /// (no gaps) is supported.
  input_layer[2].pixelStrideInBytes = sizeof(glm::vec4);
  /// Pixel format.
  input_layer[2].format = OPTIX_PIXEL_FORMAT_FLOAT4;

  // -------------------------------------------------------
  switch (camera_rendering_launch_params_.camera_properties.output_type) {
    case OutputType::Color: {
#if ENABLE_OPTIX_DENOISER
      if (camera_properties.denoiser_strength == 0.0f) {
        CUDA_CHECK(MemcpyToArray(
            output_array, 0, 0, (void *)camera_rendering_launch_params_.camera_properties.target_frame.color_buffer,
            sizeof(glm::vec4) * camera_rendering_launch_params_.camera_properties.target_frame.size.x *
                camera_rendering_launch_params_.camera_properties.target_frame.size.y,
            cudaMemcpyDeviceToDevice));
      } else {
        OptixImage2D output_layer;
        output_layer.data = camera_rendering_launch_params_.camera_properties.denoised_buffer.DevicePointer();
        /// Width of the image (in pixels)
        output_layer.width = camera_rendering_launch_params_.camera_properties.target_frame.size.x;
        /// Height of the image (in pixels)
        output_layer.height = camera_rendering_launch_params_.camera_properties.target_frame.size.y;
        /// Stride between subsequent rows of the image (in bytes).
        output_layer.rowStrideInBytes =
            camera_rendering_launch_params_.camera_properties.target_frame.size.x * sizeof(glm::vec4);
        /// Stride between subsequent pixels of the image (in bytes).
        /// For now, only 0 or the value that corresponds to a dense packing of pixels
        /// (no gaps) is supported.
        output_layer.pixelStrideInBytes = sizeof(glm::vec4);
        /// Pixel format.
        output_layer.format = OPTIX_PIXEL_FORMAT_FLOAT4;

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
            /*stream*/ nullptr, &input_layer[0],
            camera_rendering_launch_params_.camera_properties.denoiser_intensity.DevicePointer(),
            camera_rendering_launch_params_.camera_properties.denoiser_scratch.DevicePointer(),
            camera_rendering_launch_params_.camera_properties.denoiser_scratch.size_in_bytes));

        OptixDenoiserLayer denoiser_layer = {};
        denoiser_layer.input = input_layer[0];
        denoiser_layer.output = output_layer;

        OptixDenoiserGuideLayer denoiser_guide_layer = {};
        denoiser_guide_layer.albedo = input_layer[1];
        denoiser_guide_layer.normal = input_layer[2];

        OPTIX_CHECK(optixDenoiserInvoke(
            camera_rendering_launch_params_.camera_properties.denoiser,
            /*stream*/ 0, &denoiserParams,
            camera_rendering_launch_params_.camera_properties.denoiser_state.DevicePointer(),
            camera_rendering_launch_params_.camera_properties.denoiser_state.size_in_bytes, &denoiser_guide_layer,
            &denoiser_layer, 1,
            /*inputOffsetX*/ 0,
            /*inputOffsetY*/ 0, camera_rendering_launch_params_.camera_properties.denoiser_scratch.DevicePointer(),
            camera_rendering_launch_params_.camera_properties.denoiser_scratch.size_in_bytes));
        CUDA_CHECK(
            MemcpyToArray(output_array, 0, 0, (void *)output_layer.data,
                          sizeof(glm::vec4) * camera_rendering_launch_params_.camera_properties.target_frame.size.x *
                              camera_rendering_launch_params_.camera_properties.target_frame.size.y,
                          cudaMemcpyDeviceToDevice));
      }
#else
      CUDA_CHECK(MemcpyToArray(
          output_array, 0, 0, (void *)camera_rendering_launch_params_.camera_properties.target_frame.color_buffer,
          sizeof(glm::vec4) * camera_rendering_launch_params_.camera_properties.target_frame.size.x *
              camera_rendering_launch_params_.camera_properties.target_frame.size.y,
          cudaMemcpyDeviceToDevice));
#endif
    } break;
    case OutputType::Normal: {
      CUDA_CHECK(MemcpyToArray(
          output_array, 0, 0, (void *)camera_rendering_launch_params_.camera_properties.target_frame.normal_buffer,
          sizeof(glm::vec4) * camera_rendering_launch_params_.camera_properties.target_frame.size.x *
              camera_rendering_launch_params_.camera_properties.target_frame.size.y,
          cudaMemcpyDeviceToDevice));
    } break;
    case OutputType::Albedo: {
      CUDA_CHECK(MemcpyToArray(
          output_array, 0, 0, (void *)camera_rendering_launch_params_.camera_properties.target_frame.albedo_buffer,
          sizeof(glm::vec4) * camera_rendering_launch_params_.camera_properties.target_frame.size.x *
              camera_rendering_launch_params_.camera_properties.target_frame.size.y,
          cudaMemcpyDeviceToDevice));
    } break;
    case OutputType::Depth: {
      CUDA_CHECK(MemcpyToArray(
          output_array, 0, 0, (void *)camera_rendering_launch_params_.camera_properties.target_frame.albedo_buffer,
          sizeof(glm::vec4) * camera_rendering_launch_params_.camera_properties.target_frame.size.x *
              camera_rendering_launch_params_.camera_properties.target_frame.size.y,
          cudaMemcpyDeviceToDevice));
    } break;
  }

#pragma endregion
  return true;
}

void OptiXRayTracer::EstimateIllumination(const size_t &size, const EnvironmentProperties &environment_properties,
                                          const RayProperties &ray_properties, const CudaBuffer &light_probes,
                                          const unsigned seed, const float push_normal_distance) {
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
                                    const CudaBuffer &samples) {
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
#ifndef NDEBUG
  EVOENGINE_LOG("Optix: creating optix context...");
#endif
  CreateContext();
#ifndef NDEBUG
  EVOENGINE_LOG("Optix: setting up module...");
#endif
  CreateModules();
#ifndef NDEBUG
  EVOENGINE_LOG("Optix: creating raygen programs...");
#endif
  CreateRayGenPrograms();
#ifndef NDEBUG
  EVOENGINE_LOG("Optix: creating miss programs...");
#endif
  CreateMissPrograms();
#ifndef NDEBUG
  EVOENGINE_LOG("Optix: creating hitgroup programs...");
#endif
  CreateHitGroupPrograms();
#ifndef NDEBUG
  EVOENGINE_LOG("Optix: setting up optix pipeline...");
#endif
  AssemblePipelines();
#ifndef NDEBUG
  EVOENGINE_LOG("Optix set up finished.");
#endif
}

OptiXRayTracer::~OptiXRayTracer() {
  materials.clear();
  geometries.clear();
  instances.clear();
}

static void context_log_cb(const unsigned int level, const char *tag, const char *message, void *) {
#ifndef NDEBUG
  fprintf(stderr, "[%2d][%12s]: %s\n", static_cast<int>(level), tag, message);
#endif
}

void PrintLogMessage(unsigned int level, const char *tag, const char *message, void * /* cbdata */) {
#ifndef NDEBUG
  std::cerr << "[" << std::setw(2) << level << "][" << std::setw(12) << tag << "]: " << message << std::endl;
#endif
}

void OptiXRayTracer::CreateContext() {
  // for this sample, do everything on one device
  constexpr int device_id = 0;
  CUDA_CHECK(StreamCreate(&stream_));
  CUDA_CHECK(GetDeviceProperties(&device_props_, device_id));
  EVOENGINE_LOG(std::string("Optix: running on device: ") + device_props_.name);
  if (const CUresult cu_res = cuCtxGetCurrent(&cuda_context_); cu_res != CUDA_SUCCESS)
    fprintf(stderr, "Error querying current context: error code %d\n", cu_res);

  OptixDeviceContextOptions options = {};
  options.logCallbackFunction = &PrintLogMessage;
  options.logCallbackLevel = 4;
#ifndef NDEBUG
  options.validationMode = OPTIX_DEVICE_CONTEXT_VALIDATION_MODE_ALL;
#endif
  OPTIX_CHECK(optixDeviceContextCreate(cuda_context_, &options, &optix_device_context_));
  OPTIX_CHECK(optixDeviceContextSetLogCallback(optix_device_context_, context_log_cb, nullptr, 4));
}

extern "C" char camera_rendering_ptx[];
extern "C" char illumination_estimation_ptx[];
extern "C" char point_cloud_scanning_ptx[];

void OptiXRayTracer::CreateModules() {
  CreateModule(camera_rendering_pipeline_, camera_rendering_ptx, "cameraRenderingLaunchParams");
  CreateModule(illumination_estimation_pipeline_, illumination_estimation_ptx, "illuminationEstimationLaunchParams");
  CreateModule(point_cloud_scanning_pipeline_, point_cloud_scanning_ptx, "pointCloudScanningLaunchParams");
}

void OptiXRayTracer::CreateRayGenPrograms() {
  CreateRayGenProgram(camera_rendering_pipeline_, "__raygen__CR");
  CreateRayGenProgram(illumination_estimation_pipeline_, "__raygen__IE");
  CreateRayGenProgram(point_cloud_scanning_pipeline_, "__raygen__PCS");
}

void OptiXRayTracer::CreateMissPrograms() {
  {
    char log[2048];
    size_t sizeof_log = sizeof(log);

    OptixProgramGroupOptions pg_options = {};
    OptixProgramGroupDesc pg_desc = {};
    pg_desc.kind = OPTIX_PROGRAM_GROUP_KIND_MISS;
    pg_desc.miss.module = camera_rendering_pipeline_.module;

    // ------------------------------------------------------------------
    // radiance rays
    // ------------------------------------------------------------------
    pg_desc.miss.entryFunctionName = "__miss__CR_R";

    OPTIX_CHECK(optixProgramGroupCreate(optix_device_context_, &pg_desc, 1, &pg_options, log, &sizeof_log,
                                        &camera_rendering_pipeline_.miss_program_groups[RayType::Radiance]));
    if (sizeof_log > 1)
      std::cout << log << std::endl;
    // ------------------------------------------------------------------
    // BSSRDF Spatial sampler rays
    // ------------------------------------------------------------------
    pg_desc.miss.entryFunctionName = "__miss__CR_SS";
    OPTIX_CHECK(optixProgramGroupCreate(optix_device_context_, &pg_desc, 1, &pg_options, log, &sizeof_log,
                                        &camera_rendering_pipeline_.miss_program_groups[RayType::SpacialSampling]));
#ifndef NDEBUG
    if (sizeof_log > 1)
      std::cout << log << std::endl;
#endif
  }
  {
    char log[2048];
    size_t sizeof_log = sizeof(log);

    OptixProgramGroupOptions pg_options = {};
    OptixProgramGroupDesc pg_desc = {};
    pg_desc.kind = OPTIX_PROGRAM_GROUP_KIND_MISS;
    pg_desc.miss.module = illumination_estimation_pipeline_.module;

    // ------------------------------------------------------------------
    // radiance rays
    // ------------------------------------------------------------------
    pg_desc.miss.entryFunctionName = "__miss__IE_R";

    OPTIX_CHECK(optixProgramGroupCreate(optix_device_context_, &pg_desc, 1, &pg_options, log, &sizeof_log,
                                        &illumination_estimation_pipeline_.miss_program_groups[RayType::Radiance]));
    if (sizeof_log > 1)
      std::cout << log << std::endl;
    // ------------------------------------------------------------------
    // BSSRDF Spatial sampler rays
    // ------------------------------------------------------------------
    pg_desc.miss.entryFunctionName = "__miss__IE_SS";
    OPTIX_CHECK(
        optixProgramGroupCreate(optix_device_context_, &pg_desc, 1, &pg_options, log, &sizeof_log,
                                &illumination_estimation_pipeline_.miss_program_groups[RayType::SpacialSampling]));
#ifndef NDEBUG
    if (sizeof_log > 1)
      std::cout << log << std::endl;
#endif
  }
  {
    char log[2048];
    size_t sizeof_log = sizeof(log);

    OptixProgramGroupOptions pg_options = {};
    OptixProgramGroupDesc pg_desc = {};
    pg_desc.kind = OPTIX_PROGRAM_GROUP_KIND_MISS;
    pg_desc.miss.module = point_cloud_scanning_pipeline_.module;

    // ------------------------------------------------------------------
    // radiance rays
    // ------------------------------------------------------------------
    pg_desc.miss.entryFunctionName = "__miss__PCS_R";

    OPTIX_CHECK(optixProgramGroupCreate(optix_device_context_, &pg_desc, 1, &pg_options, log, &sizeof_log,
                                        &point_cloud_scanning_pipeline_.miss_program_groups[RayType::Radiance]));
    if (sizeof_log > 1)
      std::cout << log << std::endl;
    // ------------------------------------------------------------------
    // BSSRDF Spatial sampler rays
    // ------------------------------------------------------------------
    pg_desc.miss.entryFunctionName = "__miss__PCS_SS";
    OPTIX_CHECK(optixProgramGroupCreate(optix_device_context_, &pg_desc, 1, &pg_options, log, &sizeof_log,
                                        &point_cloud_scanning_pipeline_.miss_program_groups[RayType::SpacialSampling]));
#ifndef NDEBUG
    if (sizeof_log > 1)
      std::cout << log << std::endl;
#endif
  }
}

void OptiXRayTracer::CreateHitGroupPrograms() {
  {
    char log[2048];
    size_t sizeof_log = sizeof(log);

    OptixProgramGroupOptions pg_options = {};
    OptixProgramGroupDesc pg_desc = {};
    pg_desc.kind = OPTIX_PROGRAM_GROUP_KIND_HITGROUP;
    pg_desc.hitgroup.moduleCH = camera_rendering_pipeline_.module;
    pg_desc.hitgroup.moduleAH = camera_rendering_pipeline_.module;

    // -------------------------------------------------------
    // radiance rays
    // -------------------------------------------------------
    pg_desc.hitgroup.entryFunctionNameCH = "__closesthit__CR_R";
    pg_desc.hitgroup.entryFunctionNameAH = "__anyhit__CR_R";
    pg_desc.hitgroup.entryFunctionNameIS = 0;
    OPTIX_CHECK(optixProgramGroupCreate(
        optix_device_context_, &pg_desc, 1, &pg_options, log, &sizeof_log,
        &camera_rendering_pipeline_.hit_group_program_groups[RayType::Radiance][PrimitiveType::Triangle]));

    pg_desc.hitgroup.moduleIS = camera_rendering_pipeline_.linear_curve_module;
    OPTIX_CHECK(optixProgramGroupCreate(
        optix_device_context_, &pg_desc, 1, &pg_options, log, &sizeof_log,
        &camera_rendering_pipeline_.hit_group_program_groups[RayType::Radiance][PrimitiveType::Linear]));

    pg_desc.hitgroup.moduleIS = camera_rendering_pipeline_.quadratic_curve_module;
    OPTIX_CHECK(optixProgramGroupCreate(
        optix_device_context_, &pg_desc, 1, &pg_options, log, &sizeof_log,
        &camera_rendering_pipeline_.hit_group_program_groups[RayType::Radiance][PrimitiveType::QuadraticBSpline]));

    pg_desc.hitgroup.moduleIS = camera_rendering_pipeline_.cubic_curve_module;
    OPTIX_CHECK(optixProgramGroupCreate(
        optix_device_context_, &pg_desc, 1, &pg_options, log, &sizeof_log,
        &camera_rendering_pipeline_.hit_group_program_groups[RayType::Radiance][PrimitiveType::CubicBSpline]));

#ifndef NDEBUG
    if (sizeof_log > 1)
      std::cout << log << std::endl;
#endif

    // -------------------------------------------------------
    // BSSRDF Sampler ray
    // -------------------------------------------------------
    pg_desc.hitgroup.entryFunctionNameCH = "__closesthit__CR_SS";
    pg_desc.hitgroup.entryFunctionNameAH = "__anyhit__CR_SS";
    OPTIX_CHECK(optixProgramGroupCreate(
        optix_device_context_, &pg_desc, 1, &pg_options, log, &sizeof_log,
        &camera_rendering_pipeline_.hit_group_program_groups[RayType::SpacialSampling][PrimitiveType::Triangle]));

    pg_desc.hitgroup.moduleIS = camera_rendering_pipeline_.linear_curve_module;
    OPTIX_CHECK(optixProgramGroupCreate(
        optix_device_context_, &pg_desc, 1, &pg_options, log, &sizeof_log,
        &camera_rendering_pipeline_.hit_group_program_groups[RayType::SpacialSampling][PrimitiveType::Linear]));
    ;

    pg_desc.hitgroup.moduleIS = camera_rendering_pipeline_.quadratic_curve_module;
    OPTIX_CHECK(optixProgramGroupCreate(
        optix_device_context_, &pg_desc, 1, &pg_options, log, &sizeof_log,
        &camera_rendering_pipeline_
             .hit_group_program_groups[RayType::SpacialSampling][PrimitiveType::QuadraticBSpline]));

    pg_desc.hitgroup.moduleIS = camera_rendering_pipeline_.cubic_curve_module;
    OPTIX_CHECK(optixProgramGroupCreate(
        optix_device_context_, &pg_desc, 1, &pg_options, log, &sizeof_log,
        &camera_rendering_pipeline_.hit_group_program_groups[RayType::SpacialSampling][PrimitiveType::CubicBSpline]));

#ifndef NDEBUG
    if (sizeof_log > 1)
      std::cout << log << std::endl;
#endif
  }
  {
    char log[2048];
    size_t sizeof_log = sizeof(log);

    OptixProgramGroupOptions pg_options = {};
    OptixProgramGroupDesc pg_desc = {};
    pg_desc.kind = OPTIX_PROGRAM_GROUP_KIND_HITGROUP;
    pg_desc.hitgroup.moduleCH = illumination_estimation_pipeline_.module;
    pg_desc.hitgroup.moduleAH = illumination_estimation_pipeline_.module;
    // -------------------------------------------------------
    // radiance rays
    // -------------------------------------------------------
    pg_desc.hitgroup.entryFunctionNameCH = "__closesthit__IE_R";
    pg_desc.hitgroup.entryFunctionNameAH = "__anyhit__IE_R";
    OPTIX_CHECK(optixProgramGroupCreate(
        optix_device_context_, &pg_desc, 1, &pg_options, log, &sizeof_log,
        &illumination_estimation_pipeline_.hit_group_program_groups[RayType::Radiance][PrimitiveType::Triangle]));

    pg_desc.hitgroup.moduleIS = illumination_estimation_pipeline_.linear_curve_module;
    OPTIX_CHECK(optixProgramGroupCreate(
        optix_device_context_, &pg_desc, 1, &pg_options, log, &sizeof_log,
        &illumination_estimation_pipeline_.hit_group_program_groups[RayType::Radiance][PrimitiveType::Linear]));

    pg_desc.hitgroup.moduleIS = illumination_estimation_pipeline_.quadratic_curve_module;
    OPTIX_CHECK(
        optixProgramGroupCreate(optix_device_context_, &pg_desc, 1, &pg_options, log, &sizeof_log,
                                &illumination_estimation_pipeline_
                                     .hit_group_program_groups[RayType::Radiance][PrimitiveType::QuadraticBSpline]));

    pg_desc.hitgroup.moduleIS = illumination_estimation_pipeline_.cubic_curve_module;
    OPTIX_CHECK(optixProgramGroupCreate(
        optix_device_context_, &pg_desc, 1, &pg_options, log, &sizeof_log,
        &illumination_estimation_pipeline_.hit_group_program_groups[RayType::Radiance][PrimitiveType::CubicBSpline]));
#ifndef NDEBUG
    if (sizeof_log > 1)
      std::cout << log << std::endl;
#endif
    // -------------------------------------------------------
    // BSSRDF Sampler ray
    // -------------------------------------------------------
    pg_desc.hitgroup.entryFunctionNameCH = "__closesthit__IE_SS";
    pg_desc.hitgroup.entryFunctionNameAH = "__anyhit__IE_SS";
    OPTIX_CHECK(
        optixProgramGroupCreate(optix_device_context_, &pg_desc, 1, &pg_options, log, &sizeof_log,
                                &illumination_estimation_pipeline_
                                     .hit_group_program_groups[RayType::SpacialSampling][PrimitiveType::Triangle]));

    pg_desc.hitgroup.moduleIS = illumination_estimation_pipeline_.linear_curve_module;
    OPTIX_CHECK(optixProgramGroupCreate(
        optix_device_context_, &pg_desc, 1, &pg_options, log, &sizeof_log,
        &illumination_estimation_pipeline_.hit_group_program_groups[RayType::SpacialSampling][PrimitiveType::Linear]));

    pg_desc.hitgroup.moduleIS = illumination_estimation_pipeline_.quadratic_curve_module;
    OPTIX_CHECK(optixProgramGroupCreate(
        optix_device_context_, &pg_desc, 1, &pg_options, log, &sizeof_log,
        &illumination_estimation_pipeline_
             .hit_group_program_groups[RayType::SpacialSampling][PrimitiveType::QuadraticBSpline]));

    pg_desc.hitgroup.moduleIS = illumination_estimation_pipeline_.cubic_curve_module;
    OPTIX_CHECK(
        optixProgramGroupCreate(optix_device_context_, &pg_desc, 1, &pg_options, log, &sizeof_log,
                                &illumination_estimation_pipeline_
                                     .hit_group_program_groups[RayType::SpacialSampling][PrimitiveType::CubicBSpline]));
#ifndef NDEBUG
    if (sizeof_log > 1)
      std::cout << log << std::endl;
#endif
  }
  {
    char log[2048];
    size_t sizeof_log = sizeof(log);

    OptixProgramGroupOptions pg_options = {};
    OptixProgramGroupDesc pg_desc = {};
    pg_desc.kind = OPTIX_PROGRAM_GROUP_KIND_HITGROUP;
    pg_desc.hitgroup.moduleCH = point_cloud_scanning_pipeline_.module;
    pg_desc.hitgroup.moduleAH = point_cloud_scanning_pipeline_.module;
    // -------------------------------------------------------
    // radiance rays
    // -------------------------------------------------------
    pg_desc.hitgroup.entryFunctionNameCH = "__closesthit__PCS_R";
    pg_desc.hitgroup.entryFunctionNameAH = "__anyhit__PCS_R";
    OPTIX_CHECK(optixProgramGroupCreate(
        optix_device_context_, &pg_desc, 1, &pg_options, log, &sizeof_log,
        &point_cloud_scanning_pipeline_.hit_group_program_groups[RayType::Radiance][PrimitiveType::Triangle]));

    pg_desc.hitgroup.moduleIS = point_cloud_scanning_pipeline_.linear_curve_module;
    OPTIX_CHECK(optixProgramGroupCreate(
        optix_device_context_, &pg_desc, 1, &pg_options, log, &sizeof_log,
        &point_cloud_scanning_pipeline_.hit_group_program_groups[RayType::Radiance][PrimitiveType::Linear]));

    pg_desc.hitgroup.moduleIS = point_cloud_scanning_pipeline_.quadratic_curve_module;
    OPTIX_CHECK(optixProgramGroupCreate(
        optix_device_context_, &pg_desc, 1, &pg_options, log, &sizeof_log,
        &point_cloud_scanning_pipeline_.hit_group_program_groups[RayType::Radiance][PrimitiveType::QuadraticBSpline]));

    pg_desc.hitgroup.moduleIS = point_cloud_scanning_pipeline_.cubic_curve_module;
    OPTIX_CHECK(optixProgramGroupCreate(
        optix_device_context_, &pg_desc, 1, &pg_options, log, &sizeof_log,
        &point_cloud_scanning_pipeline_.hit_group_program_groups[RayType::Radiance][PrimitiveType::CubicBSpline]));
#ifndef NDEBUG
    if (sizeof_log > 1)
      std::cout << log << std::endl;
#endif
    // -------------------------------------------------------
    // BSSRDF Sampler ray
    // -------------------------------------------------------
    pg_desc.hitgroup.entryFunctionNameCH = "__closesthit__PCS_SS";
    pg_desc.hitgroup.entryFunctionNameAH = "__anyhit__PCS_SS";
    OPTIX_CHECK(optixProgramGroupCreate(
        optix_device_context_, &pg_desc, 1, &pg_options, log, &sizeof_log,
        &point_cloud_scanning_pipeline_.hit_group_program_groups[RayType::SpacialSampling][PrimitiveType::Triangle]));

    pg_desc.hitgroup.moduleIS = point_cloud_scanning_pipeline_.linear_curve_module;
    OPTIX_CHECK(optixProgramGroupCreate(
        optix_device_context_, &pg_desc, 1, &pg_options, log, &sizeof_log,
        &point_cloud_scanning_pipeline_.hit_group_program_groups[RayType::SpacialSampling][PrimitiveType::Linear]));

    pg_desc.hitgroup.moduleIS = point_cloud_scanning_pipeline_.quadratic_curve_module;
    OPTIX_CHECK(optixProgramGroupCreate(
        optix_device_context_, &pg_desc, 1, &pg_options, log, &sizeof_log,
        &point_cloud_scanning_pipeline_
             .hit_group_program_groups[RayType::SpacialSampling][PrimitiveType::QuadraticBSpline]));

    pg_desc.hitgroup.moduleIS = point_cloud_scanning_pipeline_.cubic_curve_module;
    OPTIX_CHECK(
        optixProgramGroupCreate(optix_device_context_, &pg_desc, 1, &pg_options, log, &sizeof_log,
                                &point_cloud_scanning_pipeline_
                                     .hit_group_program_groups[RayType::SpacialSampling][PrimitiveType::CubicBSpline]));
#ifndef NDEBUG
    if (sizeof_log > 1)
      std::cout << log << std::endl;
#endif
  }
}

__global__ void CopyVerticesInstancedKernel(const int matrices_size, const int vertices_size,
                                            const InstanceMatrix *matrices, const Vertex *vertices,
                                            glm::vec3 *target_positions, Vertex *target_vertices) {
  if (const int idx = threadIdx.x + blockIdx.x * blockDim.x; idx < vertices_size * matrices_size) {
    const glm::vec3 position =
        matrices[idx / vertices_size].instance_matrix * glm::vec4(vertices[idx % vertices_size].position, 1.0f);
    target_positions[idx] = position;
    const glm::vec3 normal = glm::normalize(matrices[idx / vertices_size].instance_matrix *
                                            glm::vec4(vertices[idx % vertices_size].normal, 0.0f));
    glm::vec3 tangent = glm::normalize(matrices[idx / vertices_size].instance_matrix *
                                       glm::vec4(vertices[idx % vertices_size].tangent, 0.0f));
    tangent = glm::normalize(tangent - dot(tangent, normal) * normal);
    target_vertices[idx] = {};
    target_vertices[idx].position = position;
    target_vertices[idx].tangent = tangent;
    target_vertices[idx].normal = normal;
    target_vertices[idx].tex_coord = vertices[idx % vertices_size].tex_coord;
    target_vertices[idx].color = matrices[idx / vertices_size].instance_color;
  }
}

__global__ void CopyStrandPointsKernel(const int size, const StrandPoint *strand_points, float *target_thicknesses) {
  if (const int idx = threadIdx.x + blockIdx.x * blockDim.x; idx < size) {
    target_thicknesses[idx] = strand_points[idx].thickness;
  }
}

__global__ void CopyVerticesKernel(const int size, const Vertex *vertices, glm::vec3 *target_positions) {
  if (const int idx = threadIdx.x + blockIdx.x * blockDim.x; idx < size) {
    target_positions[idx] = vertices[idx].position;
  }
}

__global__ void CopySkinnedVerticesKernel(const int size, SkinnedVertex *vertices, const glm::mat4 *bone_matrices,
                                          glm::vec3 *target_positions, Vertex *target_vertices) {
  if (const int idx = threadIdx.x + blockIdx.x * blockDim.x; idx < size) {
    glm::mat4 bone_transform = bone_matrices[vertices[idx].bond_id[0]] * vertices[idx].weight[0];
    if (vertices[idx].bond_id[1] != -1) {
      bone_transform += bone_matrices[vertices[idx].bond_id[1]] * vertices[idx].weight[1];
    }
    if (vertices[idx].bond_id[2] != -1) {
      bone_transform += bone_matrices[vertices[idx].bond_id[2]] * vertices[idx].weight[2];
    }
    if (vertices[idx].bond_id[3] != -1) {
      bone_transform += bone_matrices[vertices[idx].bond_id[3]] * vertices[idx].weight[3];
    }
    if (vertices[idx].bond_id2[0] != -1) {
      bone_transform += bone_matrices[vertices[idx].bond_id2[0]] * vertices[idx].weight2[0];
    }
    if (vertices[idx].bond_id2[1] != -1) {
      bone_transform += bone_matrices[vertices[idx].bond_id2[1]] * vertices[idx].weight2[1];
    }
    if (vertices[idx].bond_id2[2] != -1) {
      bone_transform += bone_matrices[vertices[idx].bond_id2[2]] * vertices[idx].weight2[2];
    }
    if (vertices[idx].bond_id2[3] != -1) {
      bone_transform += bone_matrices[vertices[idx].bond_id2[3]] * vertices[idx].weight2[3];
    }
    const glm::vec3 position = bone_transform * glm::vec4(vertices[idx].position, 1.0f);
    target_positions[idx] = position;
    const glm::vec3 normal = glm::normalize(bone_transform * glm::vec4(vertices[idx].normal, 0.0f));
    glm::vec3 tangent = glm::normalize(bone_transform * glm::vec4(vertices[idx].tangent, 0.0f));
    tangent = glm::normalize(tangent - dot(tangent, normal) * normal);
    target_vertices[idx].position = position;
    target_vertices[idx].normal = normal;
    target_vertices[idx].tangent = tangent;
    target_vertices[idx].tex_coord = vertices[idx].tex_coord;
    target_vertices[idx].color = vertices[idx].color;
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

  CudaBuffer device_position_buffer;
  CudaBuffer device_width_buffer;

  CUdeviceptr device_vertex_positions;
  CUdeviceptr device_vertex_triangles;
  CUdeviceptr device_points;
  CUdeviceptr device_widths;

#pragma region Geometry Inputs
  // ==================================================================
  // geometry inputs
  // ==================================================================
  OptixBuildInput build_input;
  constexpr uint32_t triangle_input_flags[1] = {OPTIX_GEOMETRY_FLAG_NONE};
  switch (renderer_type) {
    case RendererType::Curve: {
      CUdeviceptr device_strands;
      device_width_buffer.Resize(curve_points->size() * sizeof(float));
      vertex_data_buffer.Upload(*curve_points);
      triangle_buffer.Upload(*curve_segments);

      int block_size = 0;     // The launch configurator returned block size
      int min_grid_size = 0;  // The minimum grid size needed to achieve the
      // maximum occupancy for a full device launch
      int grid_size = 0;  // The actual grid size needed, based on input size
      int size = curve_points->size();
      cudaOccupancyMaxPotentialBlockSize(&min_grid_size, &block_size, CopyStrandPointsKernel, 0, size);
      grid_size = (size + block_size - 1) / block_size;
      CopyStrandPointsKernel<<<grid_size, block_size>>>(
          size, static_cast<evo_engine::StrandPoint *>(vertex_data_buffer.d_ptr),
          static_cast<float *>(device_width_buffer.d_ptr));
      CUDA_SYNC_CHECK();
      build_input.type = OPTIX_BUILD_INPUT_TYPE_CURVES;
      switch (geometry_type) {
        case PrimitiveType::Linear:
          build_input.curveArray.curveType = OPTIX_PRIMITIVE_TYPE_ROUND_LINEAR;
          break;
        case PrimitiveType::QuadraticBSpline:
          build_input.curveArray.curveType = OPTIX_PRIMITIVE_TYPE_ROUND_QUADRATIC_BSPLINE;
          break;
        case PrimitiveType::CubicBSpline:
          build_input.curveArray.curveType = OPTIX_PRIMITIVE_TYPE_ROUND_CUBIC_BSPLINE;
          break;
      }
      device_points = vertex_data_buffer.DevicePointer();
      device_widths = device_width_buffer.DevicePointer();
      device_strands = triangle_buffer.DevicePointer();
      build_input.curveArray.numPrimitives = curve_segments->size();
      build_input.curveArray.vertexBuffers = &device_points;
      build_input.curveArray.numVertices = static_cast<unsigned int>(curve_points->size());
      build_input.curveArray.vertexStrideInBytes = sizeof(evo_engine::StrandPoint);
      build_input.curveArray.widthBuffers = &device_widths;
      build_input.curveArray.widthStrideInBytes = sizeof(float);
      build_input.curveArray.normalBuffers = 0;
      build_input.curveArray.normalStrideInBytes = 0;
      build_input.curveArray.indexBuffer = device_strands;
      build_input.curveArray.indexStrideInBytes = sizeof(int);
      build_input.curveArray.flag = OPTIX_GEOMETRY_FLAG_NONE;
      build_input.curveArray.endcapFlags = OPTIX_CURVE_ENDCAP_ON;
      build_input.curveArray.primitiveIndexOffset = 0;
    } break;
    case RendererType::Default: {
      vertex_data_buffer.Upload(*vertices);
      int block_size = 0;     // The launch configurator returned block size
      int min_grid_size = 0;  // The minimum grid size needed to achieve the
      // maximum occupancy for a full device launch
      int size = vertices->size();
      cudaOccupancyMaxPotentialBlockSize(&min_grid_size, &block_size, CopyVerticesKernel, 0, size);
      CUDA_SYNC_CHECK();
      triangle_buffer.Upload(*triangles);

      build_input = {};
      build_input.type = OPTIX_BUILD_INPUT_TYPE_TRIANGLES;

      // create local variables, because we need a *pointer* to the
      // device pointers
      device_vertex_positions = vertex_data_buffer.DevicePointer();
      device_vertex_triangles = triangle_buffer.DevicePointer();

      build_input.triangleArray.vertexFormat = OPTIX_VERTEX_FORMAT_FLOAT3;
      build_input.triangleArray.vertexStrideInBytes = sizeof(Vertex);
      build_input.triangleArray.numVertices = static_cast<int>(vertices->size());
      build_input.triangleArray.vertexBuffers = &device_vertex_positions;

      build_input.triangleArray.indexFormat = OPTIX_INDICES_FORMAT_UNSIGNED_INT3;
      build_input.triangleArray.indexStrideInBytes = sizeof(glm::uvec3);
      build_input.triangleArray.numIndexTriplets = static_cast<int>(triangle_buffer.size_in_bytes / sizeof(glm::uvec3));
      build_input.triangleArray.indexBuffer = device_vertex_triangles;

      // in this example we have one SBT entry, and no per-primitive
      // materials:
      build_input.triangleArray.flags = triangle_input_flags;
      build_input.triangleArray.numSbtRecords = 1;
      build_input.triangleArray.sbtIndexOffsetBuffer = 0;
      build_input.triangleArray.sbtIndexOffsetSizeInBytes = 0;
      build_input.triangleArray.sbtIndexOffsetStrideInBytes = 0;
    } break;
    case RendererType::Skinned: {
      CudaBuffer skinned_vertices_buffer;
      CudaBuffer bone_matrices_buffer;
      skinned_vertices_buffer.Upload(*skinned_vertices);
      bone_matrices_buffer.Upload(*bone_matrices);
      vertex_data_buffer.Resize(skinned_vertices->size() * sizeof(evo_engine::Vertex));
      device_position_buffer.Resize(skinned_vertices->size() * sizeof(glm::vec3));
      int block_size = 0;     // The launch configurator returned block size
      int min_grid_size = 0;  // The minimum grid size needed to achieve the
      // maximum occupancy for a full device launch
      int grid_size = 0;  // The actual grid size needed, based on input size
      int size = skinned_vertices->size();
      cudaOccupancyMaxPotentialBlockSize(&min_grid_size, &block_size, CopySkinnedVerticesKernel, 0, size);
      grid_size = (size + block_size - 1) / block_size;
      CopySkinnedVerticesKernel<<<grid_size, block_size>>>(
          size, static_cast<SkinnedVertex *>(skinned_vertices_buffer.d_ptr),
          static_cast<glm::mat4 *>(bone_matrices_buffer.d_ptr), static_cast<glm::vec3 *>(device_position_buffer.d_ptr),
          static_cast<Vertex *>(vertex_data_buffer.d_ptr));
      CUDA_SYNC_CHECK();
      triangle_buffer.Upload(*triangles);
      build_input = {};
      build_input.type = OPTIX_BUILD_INPUT_TYPE_TRIANGLES;
      // create local variables, because we need a *pointer* to the
      // device pointers
      device_vertex_positions = device_position_buffer.DevicePointer();
      device_vertex_triangles = triangle_buffer.DevicePointer();
      build_input.triangleArray.vertexFormat = OPTIX_VERTEX_FORMAT_FLOAT3;
      build_input.triangleArray.vertexStrideInBytes = sizeof(glm::vec3);
      build_input.triangleArray.numVertices =
          static_cast<int>(device_position_buffer.size_in_bytes / sizeof(glm::vec3));
      build_input.triangleArray.vertexBuffers = &device_vertex_positions;
      build_input.triangleArray.indexFormat = OPTIX_INDICES_FORMAT_UNSIGNED_INT3;
      build_input.triangleArray.indexStrideInBytes = sizeof(glm::uvec3);
      build_input.triangleArray.numIndexTriplets = static_cast<int>(triangle_buffer.size_in_bytes / sizeof(glm::uvec3));
      build_input.triangleArray.indexBuffer = device_vertex_triangles;
      // in this example we have one SBT entry, and no per-primitive
      // materials:
      build_input.triangleArray.flags = triangle_input_flags;
      build_input.triangleArray.numSbtRecords = 1;
      build_input.triangleArray.sbtIndexOffsetBuffer = 0;
      build_input.triangleArray.sbtIndexOffsetSizeInBytes = 0;
      build_input.triangleArray.sbtIndexOffsetStrideInBytes = 0;
      skinned_vertices_buffer.Free();
      bone_matrices_buffer.Free();
    } break;
    case RendererType::Instanced: {
      CudaBuffer vertices_buffer;
      CudaBuffer instance_matrices_buffer;
      vertices_buffer.Upload(*vertices);
      instance_matrices_buffer.Upload(*instance_matrices);
      vertex_data_buffer.Resize(instance_matrices->size() * vertices->size() * sizeof(evo_engine::Vertex));

      device_position_buffer.Resize(instance_matrices->size() * vertices->size() * sizeof(glm::vec3));
      int block_size = 0;     // The launch configurator returned block verticesSize
      int min_grid_size = 0;  // The minimum grid verticesSize needed to achieve the
      // maximum occupancy for a full device launch
      int grid_size = 0;  // The actual grid verticesSize needed, based on input verticesSize
      int vertices_size = vertices->size();
      int matrices_size = instance_matrices->size();
      int size = vertices_size * matrices_size;
      cudaOccupancyMaxPotentialBlockSize(&min_grid_size, &block_size, CopyVerticesInstancedKernel, 0, size);
      grid_size = (size + block_size - 1) / block_size;
      CopyVerticesInstancedKernel<<<grid_size, block_size>>>(
          matrices_size, vertices_size, static_cast<InstanceMatrix *>(instance_matrices_buffer.d_ptr),
          static_cast<Vertex *>(vertices_buffer.d_ptr), static_cast<glm::vec3 *>(device_position_buffer.d_ptr),
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
      build_input = {};
      build_input.type = OPTIX_BUILD_INPUT_TYPE_TRIANGLES;
      // create local variables, because we need a *pointer* to the
      // device pointers
      device_vertex_positions = device_position_buffer.DevicePointer();
      device_vertex_triangles = triangle_buffer.DevicePointer();
      build_input.triangleArray.vertexFormat = OPTIX_VERTEX_FORMAT_FLOAT3;
      build_input.triangleArray.vertexStrideInBytes = sizeof(glm::vec3);
      build_input.triangleArray.numVertices =
          static_cast<int>(device_position_buffer.size_in_bytes / sizeof(glm::vec3));
      build_input.triangleArray.vertexBuffers = &device_vertex_positions;
      build_input.triangleArray.indexFormat = OPTIX_INDICES_FORMAT_UNSIGNED_INT3;
      build_input.triangleArray.indexStrideInBytes = sizeof(glm::uvec3);
      build_input.triangleArray.numIndexTriplets = static_cast<int>(triangle_buffer.size_in_bytes / sizeof(glm::uvec3));
      build_input.triangleArray.indexBuffer = device_vertex_triangles;
      // in this example we have one SBT entry, and no per-primitive
      // materials:
      build_input.triangleArray.flags = triangle_input_flags;
      build_input.triangleArray.numSbtRecords = 1;
      build_input.triangleArray.sbtIndexOffsetBuffer = 0;
      build_input.triangleArray.sbtIndexOffsetSizeInBytes = 0;
      build_input.triangleArray.sbtIndexOffsetStrideInBytes = 0;
      vertices_buffer.Free();
      instance_matrices_buffer.Free();
      instance_matrices_buffer.Free();
    } break;
  }
#pragma endregion
#pragma region BLAS setup
  // ==================================================================
  // BLAS setup
  // ==================================================================

  OptixAccelBuildOptions accelerate_options = {};
  accelerate_options.buildFlags =
      OPTIX_BUILD_FLAG_NONE | OPTIX_BUILD_FLAG_ALLOW_COMPACTION | OPTIX_BUILD_FLAG_PREFER_FAST_TRACE;
  accelerate_options.motionOptions.numKeys = 1;
  accelerate_options.operation = OPTIX_BUILD_OPERATION_BUILD;

  OptixAccelBufferSizes blas_buffer_sizes;
  OPTIX_CHECK(optixAccelComputeMemoryUsage(context, &accelerate_options, &build_input,
                                           1,  // num_build_inputs
                                           &blas_buffer_sizes));
#pragma endregion
#pragma region Prapere compaction
  // ==================================================================
  // prepare compaction
  // ==================================================================

  CudaBuffer compacted_size_buffer;
  compacted_size_buffer.Resize(sizeof(uint64_t));
  OptixAccelEmitDesc emit_desc;
  emit_desc.type = OPTIX_PROPERTY_TYPE_COMPACTED_SIZE;
  emit_desc.result = compacted_size_buffer.DevicePointer();
#pragma endregion
#pragma region Build AS
  // ==================================================================
  // execute build (main stage)
  // ==================================================================

  CudaBuffer temp_buffer;
  temp_buffer.Resize(blas_buffer_sizes.tempSizeInBytes);

  CudaBuffer output_buffer;
  output_buffer.Resize(blas_buffer_sizes.outputSizeInBytes);

  OPTIX_CHECK(optixAccelBuild(context,
                              /* stream */ nullptr, &accelerate_options, &build_input, 1, temp_buffer.DevicePointer(),
                              temp_buffer.size_in_bytes, output_buffer.DevicePointer(), output_buffer.size_in_bytes,
                              &traversable_handle, &emit_desc, 1));
  CUDA_SYNC_CHECK();
#pragma endregion
#pragma region Perform compaction
  // ==================================================================
  // perform compaction
  // ==================================================================
  uint64_t compacted_size;
  compacted_size_buffer.Download(&compacted_size, 1);
  accelerated_structure_buffer.Resize(compacted_size);
  OPTIX_CHECK(optixAccelCompact(context,
                                /*stream:*/ nullptr, traversable_handle, accelerated_structure_buffer.DevicePointer(),
                                accelerated_structure_buffer.size_in_bytes, &traversable_handle));
  CUDA_SYNC_CHECK();
#pragma endregion
#pragma region Compaction clean up
  // ==================================================================
  // and .... clean up
  // ==================================================================
  output_buffer.Free();  // << the Un-compacted, temporary output buffer
  temp_buffer.Free();
  compacted_size_buffer.Free();
#pragma endregion

  device_position_buffer.Free();
  device_width_buffer.Free();
  update_flag = false;
}

void RayTracedGeometry::UploadForSbt() {
  geometry_buffer.Free();
  if (geometry_type != PrimitiveType::Triangle) {
    Curves curves;
    curves.strand_points = reinterpret_cast<StrandPoint *>(vertex_data_buffer.DevicePointer());
    curves.segments = reinterpret_cast<int *>(triangle_buffer.DevicePointer());
    geometry_buffer.Upload(&curves, 1);
  } else {
    TriangularMesh mesh;
    mesh.vertices = reinterpret_cast<Vertex *>(vertex_data_buffer.DevicePointer());
    mesh.triangles = reinterpret_cast<glm::uvec3 *>(triangle_buffer.DevicePointer());
    geometry_buffer.Upload(&mesh, 1);
  }
}

void OptiXRayTracer::BuildIas() {
  std::vector<uint64_t> remove_queue;
  for (const auto &i : geometries) {
    if (i.second.remove_flag) {
      remove_queue.emplace_back(i.first);
    }
  }
  for (auto &i : remove_queue) {
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
  remove_queue.clear();
  for (const auto &i : instances) {
    if (i.second.remove_flag) {
      remove_queue.emplace_back(i.first);
    }
  }
  for (auto &i : remove_queue) {
    instances.erase(i);
  }

  std::vector<OptixInstance> optix_instances;
  unsigned int sbt_offset = 0;

  OptixInstance optix_instance = {};
  // Common optixInstance settings
  optix_instance.instanceId = 0;
  optix_instance.visibilityMask = 0xFF;
  optix_instance.flags = OPTIX_INSTANCE_FLAG_NONE;

  for (auto &instance : instances) {
    glm::mat3x4 transform = glm::transpose(instance.second.global_transform);
    memcpy(optix_instance.transform, &transform, sizeof(glm::mat3x4));
    optix_instance.sbtOffset = sbt_offset;
    optix_instance.traversableHandle = geometries.at(instance.second.geometry_map_key).traversable_handle;
    sbt_offset += static_cast<int>(RayType::RayTypeCount);
    optix_instances.push_back(optix_instance);
  }

  CudaBuffer device_temp_instances;
  device_temp_instances.Upload(optix_instances);

  // Instance build input.
  OptixBuildInput build_input = {};

  build_input.type = OPTIX_BUILD_INPUT_TYPE_INSTANCES;
  build_input.instanceArray.instances = device_temp_instances.DevicePointer();
  build_input.instanceArray.numInstances = static_cast<unsigned int>(optix_instances.size());

  OptixAccelBuildOptions accel_build_options = {};
  accel_build_options.buildFlags = OPTIX_BUILD_FLAG_NONE;
  accel_build_options.operation = OPTIX_BUILD_OPERATION_BUILD;

  OptixAccelBufferSizes buffer_sizes_ias;
  OPTIX_CHECK(optixAccelComputeMemoryUsage(optix_device_context_, &accel_build_options, &build_input,
                                           1,  // Number of build inputs
                                           &buffer_sizes_ias));

  CudaBuffer device_temp_buffer_ias;
  device_temp_buffer_ias.Resize(buffer_sizes_ias.tempSizeInBytes);
  ias_buffer_.Resize(buffer_sizes_ias.outputSizeInBytes);

  OptixTraversableHandle i_as_handle = 0;
  OPTIX_CHECK(optixAccelBuild(optix_device_context_,
                              nullptr,  // CUDA stream
                              &accel_build_options, &build_input,
                              1,  // num build inputs
                              device_temp_buffer_ias.DevicePointer(), buffer_sizes_ias.tempSizeInBytes,
                              ias_buffer_.DevicePointer(), buffer_sizes_ias.outputSizeInBytes, &i_as_handle,
                              nullptr,  // emitted property list
                              0));      // num emitted properties
  device_temp_instances.Free();
  device_temp_buffer_ias.Free();

  camera_rendering_launch_params_.traversable = i_as_handle;
  illumination_estimation_launch_params_.traversable = i_as_handle;
  point_cloud_scanning_launch_params_.traversable = i_as_handle;
  has_acceleration_structure_ = true;
  scene_modified = true;
}

void OptiXRayTracer::AssemblePipelines() {
  AssemblePipeline(camera_rendering_pipeline_);
  AssemblePipeline(illumination_estimation_pipeline_);
  AssemblePipeline(point_cloud_scanning_pipeline_);
}

void OptiXRayTracer::CreateRayGenProgram(RayTracerPipeline &target_pipeline, char entry_function_name[]) const {
  constexpr OptixProgramGroupOptions pg_options = {};
  OptixProgramGroupDesc pg_desc = {};
  pg_desc.kind = OPTIX_PROGRAM_GROUP_KIND_RAYGEN;
  pg_desc.raygen.module = target_pipeline.module;
  pg_desc.raygen.entryFunctionName = entry_function_name;
  char log[2048];
  size_t sizeof_log = sizeof(log);
  OPTIX_CHECK(optixProgramGroupCreate(optix_device_context_, &pg_desc, 1, &pg_options, log, &sizeof_log,
                                      &target_pipeline.ray_gen_program_groups));
#ifndef NDEBUG
  if (sizeof_log > 1)
    std::cout << log << std::endl;
#endif
}

void OptiXRayTracer::CreateModule(RayTracerPipeline &target_pipeline, char ptx_code[],
                                  char launch_params_name[]) const {
  target_pipeline.launch_params_name = launch_params_name;

  target_pipeline.module_compile_options.maxRegisterCount = 50;
  target_pipeline.module_compile_options.optLevel = OPTIX_COMPILE_OPTIMIZATION_DEFAULT;
  target_pipeline.module_compile_options.debugLevel = OPTIX_COMPILE_DEBUG_LEVEL_NONE;

  target_pipeline.pipeline_compile_options = {};
  target_pipeline.pipeline_compile_options.traversableGraphFlags = OPTIX_TRAVERSABLE_GRAPH_FLAG_ALLOW_ANY;
  target_pipeline.pipeline_compile_options.usesMotionBlur = false;
  target_pipeline.pipeline_compile_options.numPayloadValues = 2;
  target_pipeline.pipeline_compile_options.numAttributeValues = 2;
  target_pipeline.pipeline_compile_options.exceptionFlags = OPTIX_EXCEPTION_FLAG_NONE;
  target_pipeline.pipeline_compile_options.pipelineLaunchParamsVariableName = launch_params_name;
  target_pipeline.pipeline_compile_options.usesPrimitiveTypeFlags =
      OPTIX_PRIMITIVE_TYPE_FLAGS_TRIANGLE | OPTIX_PRIMITIVE_TYPE_FLAGS_ROUND_LINEAR |
      OPTIX_PRIMITIVE_TYPE_FLAGS_ROUND_QUADRATIC_BSPLINE | OPTIX_PRIMITIVE_TYPE_FLAGS_ROUND_CUBIC_BSPLINE;

  const std::string code = ptx_code;

  char log[2048];
  size_t sizeof_log = sizeof(log);
  OPTIX_CHECK(optixModuleCreate(optix_device_context_, &target_pipeline.module_compile_options,
                                &target_pipeline.pipeline_compile_options, code.c_str(), code.size(), log, &sizeof_log,
                                &target_pipeline.module));

  OptixBuiltinISOptions builtin_is_options = {};
  builtin_is_options.builtinISModuleType = OPTIX_PRIMITIVE_TYPE_ROUND_QUADRATIC_BSPLINE;
  builtin_is_options.curveEndcapFlags = OPTIX_CURVE_ENDCAP_ON;
  OPTIX_CHECK(optixBuiltinISModuleGet(optix_device_context_, &target_pipeline.module_compile_options,
                                      &target_pipeline.pipeline_compile_options, &builtin_is_options,
                                      &target_pipeline.quadratic_curve_module));

  builtin_is_options.builtinISModuleType = OPTIX_PRIMITIVE_TYPE_ROUND_CUBIC_BSPLINE;
  OPTIX_CHECK(optixBuiltinISModuleGet(optix_device_context_, &target_pipeline.module_compile_options,
                                      &target_pipeline.pipeline_compile_options, &builtin_is_options,
                                      &target_pipeline.cubic_curve_module));

  builtin_is_options.builtinISModuleType = OPTIX_PRIMITIVE_TYPE_ROUND_LINEAR;
  OPTIX_CHECK(optixBuiltinISModuleGet(optix_device_context_, &target_pipeline.module_compile_options,
                                      &target_pipeline.pipeline_compile_options, &builtin_is_options,
                                      &target_pipeline.linear_curve_module));
#ifndef NDEBUG
  if (sizeof_log > 1)
    std::cout << log << std::endl;
#endif
}

void OptiXRayTracer::AssemblePipeline(RayTracerPipeline &target_pipeline) const {
  std::vector<OptixProgramGroup> program_groups;
  program_groups.push_back(target_pipeline.ray_gen_program_groups);
  for (auto &i : target_pipeline.miss_program_groups)
    program_groups.push_back(i.second);
  for (auto &i : target_pipeline.hit_group_program_groups)
    for (auto &j : i.second)
      program_groups.push_back(j.second);

  constexpr uint32_t max_trace_depth = 31;
  target_pipeline.pipeline_link_options.maxTraceDepth = max_trace_depth;
  char log[2048];
  size_t sizeof_log = sizeof(log);
  OPTIX_CHECK(optixPipelineCreate(
      optix_device_context_, &target_pipeline.pipeline_compile_options, &target_pipeline.pipeline_link_options,
      program_groups.data(), static_cast<int>(program_groups.size()), log, &sizeof_log, &target_pipeline.pipeline));
#ifndef NDEBUG
  if (sizeof_log > 1)
    std::cout << log << std::endl;
#endif

  OptixStackSizes stack_sizes = {};
  for (const auto &program_group : program_groups) {
    OPTIX_CHECK(optixUtilAccumulateStackSizes(program_group, &stack_sizes, target_pipeline.pipeline));
  }

  uint32_t direct_callable_stack_size_from_traversal;
  uint32_t direct_callable_stack_size_from_state;
  uint32_t continuation_stack_size;
  OPTIX_CHECK(optixUtilComputeStackSizes(&stack_sizes, max_trace_depth,
                                         0,  // maxCCDepth
                                         0,  // maxDCDEpth
                                         &direct_callable_stack_size_from_traversal,
                                         &direct_callable_stack_size_from_state, &continuation_stack_size));
  OPTIX_CHECK(optixPipelineSetStackSize(target_pipeline.pipeline, direct_callable_stack_size_from_traversal,
                                        direct_callable_stack_size_from_state, continuation_stack_size,
                                        2  // maxTraversableDepth
                                        ));
#ifndef NDEBUG
  if (sizeof_log > 1)
    std::cout << log << std::endl;
#endif
}

void OptiXRayTracer::BuildSbt() {
  std::vector<uint64_t> remove_queue;
  for (auto &i : materials) {
    auto &material = i.second;
    material.material_buffer.Free();
    if (material.remove_flag) {
      remove_queue.emplace_back(i.first);
    } else {
      material.UploadForSbt();
    }
  }
  for (auto &i : remove_queue) {
    materials.erase(i);
  }
#pragma region Prepare SBTs
  std::map<uint64_t, SBT> shader_binding_tables;
  for (auto &instance_pair : instances) {
    auto &instance = instance_pair.second;
    auto &material = materials.at(instance.material_map_key);
    auto &geometry = geometries.at(instance.geometry_map_key);
    auto &sbt = shader_binding_tables[instance_pair.first];
    sbt.handle = instance.private_component_handle;
    sbt.global_transform = instance.global_transform;
    sbt.geometry_type = geometry.renderer_type;
    sbt.geometry = reinterpret_cast<void *>(geometry.geometry_buffer.DevicePointer());
    sbt.material_type = material.material_type;
    sbt.material = reinterpret_cast<void *>(material.material_buffer.DevicePointer());
  }
#pragma endregion
  {
    // ------------------------------------------------------------------
    // build raygen records
    // ------------------------------------------------------------------
    std::vector<CameraRenderingRayGenRecord> raygen_records;
    CameraRenderingRayGenRecord camera_rendering_ray_gen_record;
    OPTIX_CHECK(
        optixSbtRecordPackHeader(camera_rendering_pipeline_.ray_gen_program_groups, &camera_rendering_ray_gen_record));
    camera_rendering_ray_gen_record.data = nullptr; /* for now ... */
    raygen_records.push_back(camera_rendering_ray_gen_record);
    camera_rendering_pipeline_.ray_gen_records_buffer.Upload(raygen_records);
    camera_rendering_pipeline_.sbt.raygenRecord = camera_rendering_pipeline_.ray_gen_records_buffer.DevicePointer();

    // ------------------------------------------------------------------
    // build miss records
    // ------------------------------------------------------------------
    std::vector<CameraRenderingRayMissRecord> miss_records;
    for (auto &i : camera_rendering_pipeline_.miss_program_groups) {
      CameraRenderingRayMissRecord camera_rendering_ray_miss_record;
      OPTIX_CHECK(optixSbtRecordPackHeader(i.second, &camera_rendering_ray_miss_record));
      camera_rendering_ray_miss_record.data = nullptr; /* for now ... */
      miss_records.push_back(camera_rendering_ray_miss_record);
    }
    camera_rendering_pipeline_.miss_records_buffer.Upload(miss_records);
    camera_rendering_pipeline_.sbt.missRecordBase = camera_rendering_pipeline_.miss_records_buffer.DevicePointer();
    camera_rendering_pipeline_.sbt.missRecordStrideInBytes = sizeof(CameraRenderingRayMissRecord);
    camera_rendering_pipeline_.sbt.missRecordCount = static_cast<int>(miss_records.size());

    // ------------------------------------------------------------------
    // build hit records
    // ------------------------------------------------------------------

    // we don't actually have any objects in this example, but let's
    // create a dummy one so the SBT doesn't have any null pointers
    // (which the sanity checks in compilation would complain about)

    std::vector<CameraRenderingRayHitRecord> hit_group_records;
    for (auto &instance_pair : instances) {
      for (int ray_id = 0; ray_id < static_cast<int>(RayType::RayTypeCount); ray_id++) {
        auto &collection = camera_rendering_pipeline_.hit_group_program_groups[static_cast<RayType>(ray_id)];
        auto &geometry = geometries[instance_pair.second.geometry_map_key];
        auto group = collection[geometry.geometry_type];
        CameraRenderingRayHitRecord rec;
        rec.data = shader_binding_tables[instance_pair.first];
        OPTIX_CHECK(optixSbtRecordPackHeader(group, &rec));
        hit_group_records.push_back(rec);
      }
    }
    camera_rendering_pipeline_.hit_group_records_buffer.Upload(hit_group_records);
    camera_rendering_pipeline_.sbt.hitgroupRecordBase =
        camera_rendering_pipeline_.hit_group_records_buffer.DevicePointer();
    camera_rendering_pipeline_.sbt.hitgroupRecordStrideInBytes = sizeof(CameraRenderingRayHitRecord);
    camera_rendering_pipeline_.sbt.hitgroupRecordCount = static_cast<int>(hit_group_records.size());
  }
  {
    // ------------------------------------------------------------------
    // build raygen records
    // ------------------------------------------------------------------
    std::vector<IlluminationEstimationRayGenRecord> raygen_records;
    IlluminationEstimationRayGenRecord rec;
    OPTIX_CHECK(optixSbtRecordPackHeader(illumination_estimation_pipeline_.ray_gen_program_groups, &rec));
    rec.data = nullptr; /* for now ... */
    raygen_records.push_back(rec);
    illumination_estimation_pipeline_.ray_gen_records_buffer.Upload(raygen_records);
    illumination_estimation_pipeline_.sbt.raygenRecord =
        illumination_estimation_pipeline_.ray_gen_records_buffer.DevicePointer();

    // ------------------------------------------------------------------
    // build miss records
    // ------------------------------------------------------------------
    std::vector<IlluminationEstimationRayMissRecord> miss_records;
    for (auto &i : illumination_estimation_pipeline_.miss_program_groups) {
      IlluminationEstimationRayMissRecord illumination_estimation_ray_miss_record;
      OPTIX_CHECK(optixSbtRecordPackHeader(i.second, &illumination_estimation_ray_miss_record));
      illumination_estimation_ray_miss_record.data = nullptr; /* for now ... */
      miss_records.push_back(illumination_estimation_ray_miss_record);
    }
    illumination_estimation_pipeline_.miss_records_buffer.Upload(miss_records);
    illumination_estimation_pipeline_.sbt.missRecordBase =
        illumination_estimation_pipeline_.miss_records_buffer.DevicePointer();
    illumination_estimation_pipeline_.sbt.missRecordStrideInBytes = sizeof(IlluminationEstimationRayMissRecord);
    illumination_estimation_pipeline_.sbt.missRecordCount = static_cast<int>(miss_records.size());

    // ------------------------------------------------------------------
    // build hit records
    // ------------------------------------------------------------------

    // we don't actually have any objects in this example, but let's
    // create a dummy one so the SBT doesn't have any null pointers
    // (which the sanity checks in compilation would complain about)
    std::vector<IlluminationEstimationRayHitRecord> hit_group_records;
    for (auto &instance_pair : instances) {
      for (int ray_id = 0; ray_id < static_cast<int>(RayType::RayTypeCount); ray_id++) {
        auto &collection = illumination_estimation_pipeline_.hit_group_program_groups[static_cast<RayType>(ray_id)];
        auto &geometry = geometries[instance_pair.second.geometry_map_key];
        auto group = collection[geometry.geometry_type];
        IlluminationEstimationRayHitRecord illumination_estimation_ray_hit_record;
        illumination_estimation_ray_hit_record.data = shader_binding_tables[instance_pair.first];
        OPTIX_CHECK(optixSbtRecordPackHeader(group, &illumination_estimation_ray_hit_record));
        hit_group_records.push_back(illumination_estimation_ray_hit_record);
      }
    }
    illumination_estimation_pipeline_.hit_group_records_buffer.Upload(hit_group_records);
    illumination_estimation_pipeline_.sbt.hitgroupRecordBase =
        illumination_estimation_pipeline_.hit_group_records_buffer.DevicePointer();
    illumination_estimation_pipeline_.sbt.hitgroupRecordStrideInBytes = sizeof(IlluminationEstimationRayHitRecord);
    illumination_estimation_pipeline_.sbt.hitgroupRecordCount = static_cast<int>(hit_group_records.size());
  }

  {
    // ------------------------------------------------------------------
    // build raygen records
    // ------------------------------------------------------------------
    std::vector<PointCloudScanningRayGenRecord> raygen_records;
    PointCloudScanningRayGenRecord rec;
    OPTIX_CHECK(optixSbtRecordPackHeader(point_cloud_scanning_pipeline_.ray_gen_program_groups, &rec));
    rec.data = nullptr; /* for now ... */
    raygen_records.push_back(rec);
    point_cloud_scanning_pipeline_.ray_gen_records_buffer.Upload(raygen_records);
    point_cloud_scanning_pipeline_.sbt.raygenRecord =
        point_cloud_scanning_pipeline_.ray_gen_records_buffer.DevicePointer();

    // ------------------------------------------------------------------
    // build miss records
    // ------------------------------------------------------------------
    std::vector<PointCloudScanningRayMissRecord> miss_records;
    for (auto &i : point_cloud_scanning_pipeline_.miss_program_groups) {
      PointCloudScanningRayMissRecord point_cloud_scanning_ray_miss_record;
      OPTIX_CHECK(optixSbtRecordPackHeader(i.second, &point_cloud_scanning_ray_miss_record));
      point_cloud_scanning_ray_miss_record.data = nullptr; /* for now ... */
      miss_records.push_back(point_cloud_scanning_ray_miss_record);
    }
    point_cloud_scanning_pipeline_.miss_records_buffer.Upload(miss_records);
    point_cloud_scanning_pipeline_.sbt.missRecordBase =
        point_cloud_scanning_pipeline_.miss_records_buffer.DevicePointer();
    point_cloud_scanning_pipeline_.sbt.missRecordStrideInBytes = sizeof(PointCloudScanningRayMissRecord);
    point_cloud_scanning_pipeline_.sbt.missRecordCount = static_cast<int>(miss_records.size());

    // ------------------------------------------------------------------
    // build hit records
    // ------------------------------------------------------------------

    // we don't actually have any objects in this example, but let's
    // create a dummy one so the SBT doesn't have any null pointers
    // (which the sanity checks in compilation would complain about)
    std::vector<PointCloudScanningRayHitRecord> hit_group_records;
    for (auto &instance_pair : instances) {
      for (int ray_id = 0; ray_id < static_cast<int>(RayType::RayTypeCount); ray_id++) {
        auto &collection = point_cloud_scanning_pipeline_.hit_group_program_groups[(RayType)ray_id];
        auto &geometry = geometries[instance_pair.second.geometry_map_key];
        auto group = collection[geometry.geometry_type];
        PointCloudScanningRayHitRecord point_cloud_scanning_ray_hit_record;
        point_cloud_scanning_ray_hit_record.data = shader_binding_tables[instance_pair.first];
        OPTIX_CHECK(optixSbtRecordPackHeader(group, &point_cloud_scanning_ray_hit_record));
        hit_group_records.push_back(point_cloud_scanning_ray_hit_record);
      }
    }
    point_cloud_scanning_pipeline_.hit_group_records_buffer.Upload(hit_group_records);
    point_cloud_scanning_pipeline_.sbt.hitgroupRecordBase =
        point_cloud_scanning_pipeline_.hit_group_records_buffer.DevicePointer();
    point_cloud_scanning_pipeline_.sbt.hitgroupRecordStrideInBytes = sizeof(PointCloudScanningRayHitRecord);
    point_cloud_scanning_pipeline_.sbt.hitgroupRecordCount = static_cast<int>(hit_group_records.size());
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
  cudaArray_t texture_array;
  CUDA_CHECK(GraphicsGLRegisterImage(&graphics_resource, id, GL_TEXTURE_2D, cudaGraphicsRegisterFlagsReadOnly));
  CUDA_CHECK(GraphicsMapResources(1, &graphics_resource, nullptr));
  CUDA_CHECK(GraphicsSubResourceGetMappedArray(&texture_array, graphics_resource, 0, 0));
  cudaResourceDesc cuda_resource_desc = {};
  cuda_resource_desc.resType = cudaResourceTypeArray;
  cuda_resource_desc.res.array.array = texture_array;
  cudaTextureDesc cuda_texture_desc = {};
  cuda_texture_desc.addressMode[0] = cudaAddressModeWrap;
  cuda_texture_desc.addressMode[1] = cudaAddressModeWrap;
  cuda_texture_desc.filterMode = cudaFilterModeLinear;
  cuda_texture_desc.readMode = cudaReadModeElementType;
  cuda_texture_desc.normalizedCoords = 1;
  CUDA_CHECK(CreateTextureObject(&texture_object, &cuda_resource_desc, &cuda_texture_desc, nullptr));
}
