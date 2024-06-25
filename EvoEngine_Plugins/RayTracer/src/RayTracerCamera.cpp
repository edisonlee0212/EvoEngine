//
// Created by lllll on 11/15/2021.
//

#include "RayTracerCamera.hpp"
#include "Optix7.hpp"
#include "RayTracerLayer.hpp"
#include "IHandle.hpp"

#include "Application.hpp"
#include "Scene.hpp"
#include "TransformGraph.hpp"
using namespace evo_engine;

void RayTracerCamera::Ready(const glm::vec3 &position, const glm::quat &rotation) {
    if (camera_properties_.m_frame.m_size != frame_size) {
        frame_size = glm::max(glm::uvec2(512, 512), frame_size);
        camera_properties_.Resize(frame_size);
        VkExtent3D extent;
        extent.width = frame_size.x;
        extent.depth = 1;
        extent.height = frame_size.y;
        render_texture->Resize(extent);
    }

    camera_properties_.m_image = CudaModule::ImportRenderTexture(render_texture);
    camera_properties_.Set(position, rotation);

}

bool RayTracerCamera::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
    if (GetScene()->IsEntityValid(GetOwner())) ImGui::Checkbox("Main Camera", &main_camera_);

    camera_properties_.OnInspect();
    ray_properties.OnInspect();
    if (ImGui::TreeNode("Debug")) {
        static float debug_scale = 0.25f;
        ImGui::DragFloat("Scale", &debug_scale, 0.01f, 0.1f, 1.0f);
        debug_scale = glm::clamp(debug_scale, 0.1f, 1.0f);
        ImGui::Image(render_texture->GetColorImTextureId(),
                ImVec2(camera_properties_.m_frame.m_size.x * debug_scale,
                       camera_properties_.m_frame.m_size.y * debug_scale),
                ImVec2(0, 1),
                ImVec2(1, 0));
        ImGui::TreePop();
    }
    FileUtils::SaveFile("Export Screenshot", "Texture2D", {".png", ".jpg", ".hdr"},
                        [this](const std::filesystem::path &file_path) {
                            render_texture->Save(file_path);
                        }, false);
    ImGui::Checkbox("Allow auto resize", &allow_auto_resize);
    if (!allow_auto_resize) {
        glm::ivec2 resolution = { frame_size.x, frame_size.y };
        if (ImGui::DragInt2("Resolution", &resolution.x, 1, 1, 4096))
        {
            frame_size = { resolution.x, resolution.y };
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
    render_texture = std::make_unique<RenderTexture>(render_texture_create_info);
    Ready(glm::vec3(0), glm::vec3(0));
}

void RayTracerCamera::OnDestroy() {
    camera_properties_.m_frameBufferColor.Free();
    camera_properties_.m_frameBufferNormal.Free();
    camera_properties_.m_frameBufferAlbedo.Free();
    OPTIX_CHECK(optixDenoiserDestroy(camera_properties_.m_denoiser));
    camera_properties_.m_denoiserScratch.Free();
    camera_properties_.m_denoiserState.Free();
    camera_properties_.m_frameBufferColor.Free();
    camera_properties_.m_denoiserIntensity.Free();
}

void RayTracerCamera::Deserialize(const YAML::Node &in) {
    if (in["main_camera_"]) main_camera_ = in["main_camera_"].as<bool>();

    if (in["allow_auto_resize"]) allow_auto_resize = in["allow_auto_resize"].as<bool>();
    if (in["frame_size.x"]) frame_size.x = in["frame_size.x"].as<int>();
    if (in["frame_size.y"]) frame_size.y = in["frame_size.y"].as<int>();

    if (in["ray_properties.m_samples"]) ray_properties.m_samples = in["ray_properties.m_samples"].as<int>();
    if (in["ray_properties.m_bounces"]) ray_properties.m_bounces = in["ray_properties.m_bounces"].as<int>();

    if (in["camera_properties_.m_fov"]) camera_properties_.m_fov = in["camera_properties_.m_fov"].as<float>();
    if (in["camera_properties_.m_gamma"]) camera_properties_.m_gamma = in["camera_properties_.m_gamma"].as<float>();
    if (in["camera_properties_.m_accumulate"]) camera_properties_.m_accumulate = in["camera_properties_.m_accumulate"].as<bool>();
    if (in["camera_properties_.m_denoiserStrength"]) camera_properties_.m_denoiserStrength = in["camera_properties_.m_denoiserStrength"].as<float>();
    if (in["camera_properties_.m_focalLength"]) camera_properties_.m_focalLength = in["camera_properties_.m_focalLength"].as<float>();
    if (in["camera_properties_.m_aperture"]) camera_properties_.m_aperture = in["camera_properties_.m_aperture"].as<float>();
}

void RayTracerCamera::Serialize(YAML::Emitter &out) const {
    out << YAML::Key << "main_camera_" << YAML::Value << main_camera_;

    out << YAML::Key << "allow_auto_resize" << YAML::Value << allow_auto_resize;
    out << YAML::Key << "frame_size.x" << YAML::Value << frame_size.x;
    out << YAML::Key << "frame_size.y" << YAML::Value << frame_size.y;

    out << YAML::Key << "ray_properties.m_bounces" << YAML::Value << ray_properties.m_bounces;
    out << YAML::Key << "ray_properties.m_samples" << YAML::Value << ray_properties.m_samples;

    out << YAML::Key << "camera_properties_.m_fov" << YAML::Value << camera_properties_.m_fov;
    out << YAML::Key << "camera_properties_.m_gamma" << YAML::Value << camera_properties_.m_gamma;
    out << YAML::Key << "camera_properties_.m_accumulate" << YAML::Value << camera_properties_.m_accumulate;
    out << YAML::Key << "camera_properties_.m_denoiserStrength" << YAML::Value << camera_properties_.m_denoiserStrength;
    out << YAML::Key << "camera_properties_.m_focalLength" << YAML::Value << camera_properties_.m_focalLength;
    out << YAML::Key << "camera_properties_.m_aperture" << YAML::Value << camera_properties_.m_aperture;
}

RayTracerCamera &RayTracerCamera::operator=(const RayTracerCamera &source) {
    main_camera_ = source.main_camera_;

    camera_properties_.m_accumulate = source.camera_properties_.m_accumulate;
    camera_properties_.m_fov = source.camera_properties_.m_fov;
    camera_properties_.m_inverseProjectionView = source.camera_properties_.m_inverseProjectionView;
    camera_properties_.m_horizontal = source.camera_properties_.m_horizontal;
    camera_properties_.m_outputType = source.camera_properties_.m_outputType;
    camera_properties_.m_gamma = source.camera_properties_.m_gamma;
    camera_properties_.m_denoiserStrength = source.camera_properties_.m_denoiserStrength;
    camera_properties_.m_aperture = source.camera_properties_.m_aperture;
    camera_properties_.m_focalLength = source.camera_properties_.m_focalLength;
    camera_properties_.m_modified = true;

    camera_properties_.m_frame.m_size = glm::vec2(0, 0);
    ray_properties = source.ray_properties;
    frame_size = source.frame_size;
    allow_auto_resize = source.allow_auto_resize;
    rendered_ = false;
    return *this;
}

void RayTracerCamera::Render() {
    if (!CudaModule::GetRayTracer()->m_instances.empty()) {
        auto global_transform = GetScene()->GetDataComponent<GlobalTransform>(GetOwner()).value;
        Ready(global_transform[3], glm::quat_cast(global_transform));
        rendered_ = CudaModule::GetRayTracer()->RenderToCamera(
                Application::GetLayer<RayTracerLayer>()->environment_properties,
                camera_properties_,
                ray_properties);
    }
}

void RayTracerCamera::Render(const RayProperties &ray_properties) {
    if (!CudaModule::GetRayTracer()->m_instances.empty()) {
        auto global_transform = GetScene()->GetDataComponent<GlobalTransform>(GetOwner()).value;
        Ready(global_transform[3], glm::quat_cast(global_transform));
        rendered_ = CudaModule::GetRayTracer()->RenderToCamera(
                Application::GetLayer<RayTracerLayer>()->environment_properties,
                camera_properties_,
                ray_properties);
    }
}

void RayTracerCamera::Render(const RayProperties &ray_properties, const EnvironmentProperties &environment_properties) {
    if (!CudaModule::GetRayTracer()->m_instances.empty()) {
        auto global_transform = GetScene()->GetDataComponent<GlobalTransform>(GetOwner()).value;
        Ready(global_transform[3], glm::quat_cast(global_transform));
        rendered_ = CudaModule::GetRayTracer()->RenderToCamera(
                environment_properties,
                camera_properties_,
                ray_properties);
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

void RayTracerCamera::SetOutputType(OutputType value) {
    camera_properties_.SetOutputType(value);
}

void RayTracerCamera::SetAccumulate(bool value) {
    camera_properties_.m_accumulate = value;
}

void RayTracerCamera::SetSkybox(const std::shared_ptr<Cubemap>& cubemap)
{
    skybox_ = cubemap;
    const auto cudaImage = CudaModule::ImportCubemap(cubemap);
    camera_properties_.SetSkybox(cudaImage);
}

void RayTracerCamera::SetMainCamera(bool value) {
    if (GetScene()->IsEntityValid(GetOwner())) main_camera_ = value;
}

void RayTracerCamera::SetMaxDistance(float value) {
    camera_properties_.SetMaxDistance(value);
}

glm::mat4 RayTracerCamera::GetProjection() const {
    return glm::perspective(glm::radians(camera_properties_.m_fov * 0.5f), (float)frame_size.x / frame_size.y, 0.0001f, 100.0f);
}
