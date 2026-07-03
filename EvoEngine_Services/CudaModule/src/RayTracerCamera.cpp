//
// Created by lllll on 11/15/2021.
//

#include "CudaSerializationAdapters.hpp"
#include "IHandle.hpp"
#include "Optix7.hpp"
#include <optix_stubs.h>
#include "RayTracerLayer.hpp"

#include "Application.hpp"
#include "Scene.hpp"
#include "TransformGraph.hpp"
using namespace evo_engine;

void RayTracerCamera::Ready(const glm::vec3 &position, const glm::quat &rotation) {
  if (camera_properties_.target_frame.size != frame_size) {
    frame_size = glm::max(glm::uvec2(512, 512), frame_size);
    camera_properties_.Resize(frame_size);
    VkExtent3D extent;
    extent.width = frame_size.x;
    extent.depth = 1;
    extent.height = frame_size.y;
    render_texture->Resize(extent);
  }

  camera_properties_.target_image = CudaModule::ImportRenderTexture(render_texture);
  camera_properties_.Set(position, rotation);
}

bool RayTracerCamera::DrawGui(const std::shared_ptr<EditorLayer> &editor_layer) {
  if (GetScene()->IsEntityValid(GetOwner()))
    ImGui::Checkbox("Main Camera", &main_camera_);

  camera_properties_.DrawGui();
  ray_properties.DrawGui();
  if (ImGui::TreeNode("Debug")) {
    static float debug_scale = 0.25f;
    ImGui::DragFloat("Scale", &debug_scale, 0.01f, 0.1f, 10.0f);
    debug_scale = glm::clamp(debug_scale, 0.1f, 1.0f);
    ImGui::Image(render_texture->GetColorImTextureId(),
                 ImVec2(camera_properties_.target_frame.size.x * debug_scale,
                        camera_properties_.target_frame.size.y * debug_scale),
                 ImVec2(0, 1), ImVec2(1, 0));
    ImGui::TreePop();
  }
  FileUtils::SaveFile(
      "Export Screenshot", "Texture2D", {".png", ".jpg", ".hdr"},
      [this](const std::filesystem::path &file_path) {
        render_texture->Save(file_path);
      },
      false);
  ImGui::Checkbox("Allow auto resize", &allow_auto_resize);
  if (!allow_auto_resize) {
    glm::ivec2 resolution = {frame_size.x, frame_size.y};
    if (ImGui::DragInt2("Resolution", &resolution.x, 1, 1, 4096)) {
      frame_size = {resolution.x, resolution.y};
    }
  }
  return false;
}

void RayTracerCamera::OnCreate() {
  frame_size = glm::uvec2(512, 512);
  RenderTextureCreateInfo render_texture_create_info{};
  render_texture_create_info.extent.width = frame_size.x;
  render_texture_create_info.extent.height = frame_size.y;
  render_texture_create_info.extent.depth = 1;
  render_texture = std::make_shared<RenderTexture>(render_texture_create_info);
}

void RayTracerCamera::OnDestroy() {
  camera_properties_.frame_buffer_color.Free();
  camera_properties_.frame_buffer_normal.Free();
  camera_properties_.frame_buffer_albedo.Free();
#if ENABLE_OPTIX_DENOISER
  OPTIX_CHECK(optixDenoiserDestroy(camera_properties_.denoiser));
  camera_properties_.denoiser_scratch.Free();
  camera_properties_.denoiser_state.Free();
  camera_properties_.denoiser_intensity.Free();
#endif
}

void evo_engine::DeserializeRayTracerCamera(const YAML::Node &in, RayTracerCamera &target) {
  if (in["main_camera_"])
    target.main_camera_ = in["main_camera_"].as<bool>();

  if (in["allow_auto_resize"])
    target.allow_auto_resize = in["allow_auto_resize"].as<bool>();
  if (in["frame_size.x"])
    target.frame_size.x = in["frame_size.x"].as<int>();
  if (in["frame_size.y"])
    target.frame_size.y = in["frame_size.y"].as<int>();

  if (in["ray_properties.samples"])
    target.ray_properties.samples = in["ray_properties.samples"].as<int>();
  if (in["ray_properties.bounces"])
    target.ray_properties.bounces = in["ray_properties.bounces"].as<int>();

  if (in["camera_properties_.fov"])
    target.camera_properties_.fov = in["camera_properties_.fov"].as<float>();
  if (in["camera_properties_.gamma"])
    target.camera_properties_.gamma = in["camera_properties_.gamma"].as<float>();
  if (in["camera_properties_.accumulate"])
    target.camera_properties_.accumulate = in["camera_properties_.accumulate"].as<bool>();
  if (in["camera_properties_.denoiser_strength"])
    target.camera_properties_.denoiser_strength = in["camera_properties_.denoiser_strength"].as<float>();
  if (in["camera_properties_.focal_length"])
    target.camera_properties_.focal_length = in["camera_properties_.focal_length"].as<float>();
  if (in["camera_properties_.aperture"])
    target.camera_properties_.aperture = in["camera_properties_.aperture"].as<float>();
}

void evo_engine::SerializeRayTracerCamera(YAML::Emitter &out, const RayTracerCamera &target) {
  out << YAML::Key << "main_camera_" << YAML::Value << target.main_camera_;

  out << YAML::Key << "allow_auto_resize" << YAML::Value << target.allow_auto_resize;
  out << YAML::Key << "frame_size.x" << YAML::Value << target.frame_size.x;
  out << YAML::Key << "frame_size.y" << YAML::Value << target.frame_size.y;

  out << YAML::Key << "ray_properties.bounces" << YAML::Value << target.ray_properties.bounces;
  out << YAML::Key << "ray_properties.samples" << YAML::Value << target.ray_properties.samples;

  out << YAML::Key << "camera_properties_.fov" << YAML::Value << target.camera_properties_.fov;
  out << YAML::Key << "camera_properties_.gamma" << YAML::Value << target.camera_properties_.gamma;
  out << YAML::Key << "camera_properties_.accumulate" << YAML::Value << target.camera_properties_.accumulate;
  out << YAML::Key << "camera_properties_.denoiser_strength" << YAML::Value
      << target.camera_properties_.denoiser_strength;
  out << YAML::Key << "camera_properties_.focal_length" << YAML::Value << target.camera_properties_.focal_length;
  out << YAML::Key << "camera_properties_.aperture" << YAML::Value << target.camera_properties_.aperture;
}

RayTracerCamera &RayTracerCamera::operator=(const RayTracerCamera &source) {
  main_camera_ = source.main_camera_;

  camera_properties_.accumulate = source.camera_properties_.accumulate;
  camera_properties_.fov = source.camera_properties_.fov;
  camera_properties_.inverse_projection_view = source.camera_properties_.inverse_projection_view;
  camera_properties_.horizontal_direction = source.camera_properties_.horizontal_direction;
  camera_properties_.output_type = source.camera_properties_.output_type;
  camera_properties_.gamma = source.camera_properties_.gamma;
  camera_properties_.denoiser_strength = source.camera_properties_.denoiser_strength;
  camera_properties_.aperture = source.camera_properties_.aperture;
  camera_properties_.focal_length = source.camera_properties_.focal_length;
  camera_properties_.modified = true;

  camera_properties_.target_frame.size = glm::vec2(0, 0);
  ray_properties = source.ray_properties;
  frame_size = source.frame_size;
  allow_auto_resize = source.allow_auto_resize;
  rendered_ = false;
  return *this;
}

void RayTracerCamera::Render() {
  if (!CudaModule::GetRayTracer()->instances.empty()) {
    auto global_transform = GetScene()->GetDataComponent<GlobalTransform>(GetOwner()).value;
    Ready(global_transform[3], glm::quat_cast(global_transform));
    rendered_ = CudaModule::GetRayTracer()->RenderToCamera(
        ApplicationContext::Get().GetLayer<RayTracerLayer>()->environment_properties, camera_properties_,
        ray_properties);
  }
}

void RayTracerCamera::Render(const RayProperties &ray_properties) {
  if (!CudaModule::GetRayTracer()->instances.empty()) {
    auto global_transform = GetScene()->GetDataComponent<GlobalTransform>(GetOwner()).value;
    Ready(global_transform[3], glm::quat_cast(global_transform));
    rendered_ = CudaModule::GetRayTracer()->RenderToCamera(
        ApplicationContext::Get().GetLayer<RayTracerLayer>()->environment_properties, camera_properties_,
        ray_properties);
  }
}

void RayTracerCamera::Render(const RayProperties &ray_properties, const EnvironmentProperties &environment_properties) {
  if (!CudaModule::GetRayTracer()->instances.empty()) {
    auto global_transform = GetScene()->GetDataComponent<GlobalTransform>(GetOwner()).value;
    Ready(global_transform[3], glm::quat_cast(global_transform));
    rendered_ = CudaModule::GetRayTracer()->RenderToCamera(environment_properties, camera_properties_, ray_properties);
  }
}

void RayTracerCamera::RenderSpectral() {
  if (!CudaModule::GetRayTracer()->instances.empty()) {
    auto global_transform = GetScene()->GetDataComponent<GlobalTransform>(GetOwner()).value;
    Ready(global_transform[3], glm::quat_cast(global_transform));
    rendered_ = CudaModule::GetRayTracer()->RenderToCameraSpectral(
        ApplicationContext::Get().GetLayer<RayTracerLayer>()->environment_properties, camera_properties_,
        ray_properties);
  }
}

void RayTracerCamera::RenderSpectral(const RayProperties &ray_properties) {
  if (!CudaModule::GetRayTracer()->instances.empty()) {
    auto global_transform = GetScene()->GetDataComponent<GlobalTransform>(GetOwner()).value;
    Ready(global_transform[3], glm::quat_cast(global_transform));
    rendered_ = CudaModule::GetRayTracer()->RenderToCameraSpectral(
        ApplicationContext::Get().GetLayer<RayTracerLayer>()->environment_properties, camera_properties_,
        ray_properties);
  }
}

void RayTracerCamera::RenderSpectral(const RayProperties &ray_properties,
                                     const EnvironmentProperties &environment_properties) {
  if (!CudaModule::GetRayTracer()->instances.empty()) {
    auto global_transform = GetScene()->GetDataComponent<GlobalTransform>(GetOwner()).value;
    Ready(global_transform[3], glm::quat_cast(global_transform));
    rendered_ =
        CudaModule::GetRayTracer()->RenderToCameraSpectral(environment_properties, camera_properties_, ray_properties);
  }
}

void RayTracerCamera::SetFov(float value) {
  camera_properties_.SetFov(value);
}

void RayTracerCamera::SetAperture(float value) {
  camera_properties_.SetAperture(value);
}

void RayTracerCamera::SetFocalLength(float value) {
  camera_properties_.SetFocalLength(value);
}

void RayTracerCamera::SetDenoiserStrength(float value) {
  camera_properties_.SetDenoiserStrength(value);
}

void RayTracerCamera::SetGamma(float value) {
  camera_properties_.SetGamma(value);
}

void RayTracerCamera::ApplyCameraSettings(const CameraSettings &camera_settings) {
  camera_properties_.SetGamma(camera_settings.gamma);
  camera_properties_.SetFov(camera_settings.fov);

  camera_properties_.SetBackgroundColor(camera_settings.clear_color);
  if (camera_settings.use_clear_color) {
    camera_properties_.SetBackgroundType(BackgroundType::Color);
  }

  ray_properties.bounces = camera_settings.bounce;
  ray_properties.samples = camera_settings.sample_size;
}

void RayTracerCamera::SetOutputType(OutputType value) {
  camera_properties_.SetOutputType(value);
}

void RayTracerCamera::SetAccumulate(bool value) {
  camera_properties_.accumulate = value;
}

void RayTracerCamera::SetSkybox(const std::shared_ptr<Cubemap> &cubemap) {
  skybox_ = cubemap;
  const auto cudaImage = CudaModule::ImportCubemap(cubemap);
  camera_properties_.SetSkybox(cudaImage);
}

void RayTracerCamera::SetMainCamera(bool value) {
  if (GetScene()->IsEntityValid(GetOwner()))
    main_camera_ = value;
}

void RayTracerCamera::SetMaxDistance(float value) {
  camera_properties_.SetMaxDistance(value);
}

glm::mat4 RayTracerCamera::GetProjection() const {
  return glm::perspective(glm::radians(camera_properties_.fov * 0.5f), (float)frame_size.x / frame_size.y, 0.0001f,
                          100.0f);
}
