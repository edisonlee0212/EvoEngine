#include "Application.hpp"

#include "ApplicationContext.hpp"

#include "Animation.hpp"
#include "AnimationPlayer.hpp"
#include "Animator.hpp"
#include "AssetManager.hpp"
#include "AssetRef.hpp"
#include "Camera.hpp"
#include "Cubemap.hpp"
#include "DdgiVolume.hpp"
#include "EditorLayer.hpp"
#include "EnvironmentalMap.hpp"
#include "GaussianSplat.hpp"
#include "GaussianSplatRenderer.hpp"
#include "Input.hpp"
#include "InspectorRegistry.hpp"
#include "Jobs.hpp"
#include "Json.hpp"
#include "LightProbe.hpp"
#include "Lights.hpp"
#include "LodGroup.hpp"
#include "Mesh.hpp"
#include "MeshRenderer.hpp"
#include "PackageManager.hpp"
#include "Particles.hpp"
#include "Platform.hpp"
#include "PlayerController.hpp"
#include "PointCloud.hpp"
#include "PointCloudScanner.hpp"
#include "PostProcessingStack.hpp"
#include "Prefab.hpp"
#include "PrivateComponentRef.hpp"
#include "ProceduralNoise.hpp"
#include "Profiler.hpp"
#include "ProjectManager.hpp"
#include "ReflectionProbe.hpp"
#include "RenderLayer.hpp"
#include "Resources.hpp"
#include "SDKInspectionAdapters.hpp"
#include "Scene.hpp"
#include "Shader.hpp"
#include "SkinnedMesh.hpp"
#include "SkinnedMeshRenderer.hpp"
#include "Strands.hpp"
#include "StrandsRenderer.hpp"
#include "Texture2D.hpp"
#include "Times.hpp"
#include "TransformGraph.hpp"
#include "UnknownPrivateComponent.hpp"
#include "Utilities.hpp"
#include "WayPoints.hpp"
#include "WindowLayer.hpp"

#include <algorithm>
#include <chrono>
#include <cstring>

#ifdef EVOENGINE_WINDOWS
#  ifndef NOMINMAX
#    define NOMINMAX
#  endif
#  include <Windows.h>
#endif

using namespace evo_engine;

namespace {
constexpr auto kMainThreadAssetTaskFrameBudget = std::chrono::milliseconds(2);

glm::vec3 DeserializeDdgiProbeSpacing(const YAML::Node& in) {
  if (in.IsSequence()) {
    return in.as<glm::vec3>();
  }
  return glm::vec3(in.as<float>());
}

void AddUniqueStartupPackage(ApplicationInitializationSettings& settings, const std::string& package_name) {
  if (!package_name.empty() &&
      std::find(settings.startup_runtime_packages.begin(), settings.startup_runtime_packages.end(), package_name) ==
          settings.startup_runtime_packages.end()) {
    settings.startup_runtime_packages.emplace_back(package_name);
  }
}

void MergeProjectLaunchMetadata(ApplicationInitializationSettings& settings) {
  if (settings.project_path.empty()) {
    return;
  }

  const auto metadata = ProjectManager::LoadProjectLaunchMetadata(settings.project_path);
  for (const auto& package_name : metadata.startup_runtime_packages) {
    AddUniqueStartupPackage(settings, package_name);
  }
  if (!settings.startup_runtime_packages.empty()) {
    settings.enable_runtime_packages = true;
  }
}

void ConfigureConsoleWindow(const bool hide_console_window) {
#ifdef EVOENGINE_WINDOWS
  if (!hide_console_window) {
    if (!GetConsoleWindow() && !AttachConsole(ATTACH_PARENT_PROCESS)) {
      AllocConsole();
    }
    return;
  }

  const HWND console_window = GetConsoleWindow();
  if (!console_window) {
    return;
  }

  DWORD console_process_ids[2] = {};
  if (GetConsoleProcessList(console_process_ids, 2) <= 1) {
    ShowWindow(console_window, SW_HIDE);
  }
#else
  (void)hide_console_window;
#endif
}

template <typename T>
bool RegisterYamlStagedAssetIoHandler(const std::string& type_name) {
  return Serialization::RegisterAssetIoHandler<T>(
      {}, {},
      [](const T&, const std::filesystem::path&) {
        return true;
      },
      {}, {}, {}, type_name);
}

void RegisterBuiltInAssetIoHandlers() {
  RegisterYamlStagedAssetIoHandler<PostProcessingStack>("PostProcessingStack");
  RegisterYamlStagedAssetIoHandler<UnknownAsset>("UnknownAsset");
  RegisterYamlStagedAssetIoHandler<Material>("Material");
  RegisterYamlStagedAssetIoHandler<procedural_noise::ProceduralNoise2D>("ProceduralNoise2D");
  RegisterYamlStagedAssetIoHandler<procedural_noise::ProceduralNoise3D>("ProceduralNoise3D");
  RegisterYamlStagedAssetIoHandler<procedural_noise::ProceduralNoise4D>("ProceduralNoise4D");
  RegisterYamlStagedAssetIoHandler<Cubemap>("Cubemap");
  RegisterYamlStagedAssetIoHandler<LightProbe>("LightProbe");
  RegisterYamlStagedAssetIoHandler<ReflectionProbe>("ReflectionProbe");
  RegisterYamlStagedAssetIoHandler<EnvironmentalMap>("EnvironmentalMap");
  Shader::RegisterAssetIoHandlers();
  Mesh::RegisterAssetIoHandlers();
  Strands::RegisterAssetIoHandlers();
  Prefab::RegisterAssetIoHandlers();
  Texture2D::RegisterAssetIoHandlers();
  Scene::RegisterAssetIoHandlers();
  RegisterYamlStagedAssetIoHandler<ParticleInfoList>("ParticleInfoList");
  RegisterYamlStagedAssetIoHandler<Animation>("Animation");
  SkinnedMesh::RegisterAssetIoHandlers();
  PointCloud::RegisterAssetIoHandlers();
  GaussianSplat::RegisterAssetIoHandlers();
}

void SerializeAnimationPlayer(YAML::Emitter& out, const AnimationPlayer& player) {
  out << YAML::Key << "auto_play" << YAML::Value << player.auto_play;
  out << YAML::Key << "auto_play_speed" << YAML::Value << player.auto_play_speed;
}

void DeserializeAnimationPlayer(const YAML::Node& in, AnimationPlayer& player) {
  if (in["auto_play"])
    player.auto_play = in["auto_play"].as<bool>();
  if (in["auto_play_speed"])
    player.auto_play_speed = in["auto_play_speed"].as<float>();
}

void SerializeCamera(YAML::Emitter& out, const Camera& camera) {
  const auto size = camera.GetSize();
  out << YAML::Key << "x" << YAML::Value << size.x;
  out << YAML::Key << "y" << YAML::Value << size.y;
  out << YAML::Key << "use_clear_color" << YAML::Value << camera.camera_settings.use_clear_color;
  out << YAML::Key << "clear_color" << YAML::Value << camera.camera_settings.clear_color;
  out << YAML::Key << "near_distance" << YAML::Value << camera.camera_settings.near_distance;
  out << YAML::Key << "far_distance" << YAML::Value << camera.camera_settings.far_distance;
  out << YAML::Key << "fov" << YAML::Value << camera.camera_settings.fov;
  out << YAML::Key << "background_intensity" << YAML::Value << camera.camera_settings.background_intensity;
  out << YAML::Key << "fade_ratio" << YAML::Value << camera.camera_settings.fade_ratio;
  out << YAML::Key << "fade_factor" << YAML::Value << camera.camera_settings.fade_factor;
  out << YAML::Key << "sample_size" << YAML::Value << camera.camera_settings.sample_size;
  out << YAML::Key << "bounce" << YAML::Value << camera.camera_settings.bounce;
  out << YAML::Key << "gamma" << YAML::Value << camera.camera_settings.gamma;
  camera.skybox.Save("skybox", out);
  camera.post_processing_stack_ref.Save("post_processing_stack_ref", out);
}

void DeserializeCamera(const YAML::Node& in, Camera& camera) {
  if (in["use_clear_color"])
    camera.camera_settings.use_clear_color = in["use_clear_color"].as<bool>();
  if (in["clear_color"])
    camera.camera_settings.clear_color = in["clear_color"].as<glm::vec4>();
  if (in["near_distance"])
    camera.camera_settings.near_distance = in["near_distance"].as<float>();
  if (in["far_distance"])
    camera.camera_settings.far_distance = in["far_distance"].as<float>();
  if (in["fade_ratio"])
    camera.camera_settings.fade_ratio = in["fade_ratio"].as<float>();
  if (in["fade_factor"])
    camera.camera_settings.fade_factor = in["fade_factor"].as<float>();
  if (in["fov"])
    camera.camera_settings.fov = in["fov"].as<float>();
  if (in["x"] && in["y"])
    camera.Resize({in["x"].as<uint32_t>(), in["y"].as<uint32_t>()});
  camera.skybox.Load("skybox", in);
  camera.post_processing_stack_ref.Load("post_processing_stack_ref", in);
  camera.ResetRenderState();
  if (in["background_intensity"])
    camera.camera_settings.background_intensity = in["background_intensity"].as<float>();
  if (in["sample_size"])
    camera.camera_settings.sample_size = in["sample_size"].as<uint32_t>();
  if (in["bounce"])
    camera.camera_settings.bounce = in["bounce"].as<uint32_t>();
  if (in["gamma"])
    camera.camera_settings.gamma = in["gamma"].as<float>();
}

void SerializeAnimator(YAML::Emitter& out, const Animator& animator) {
  animator.PeekAnimationRef().Save("animation_", out);
  out << YAML::Key << "current_activated_animation_" << YAML::Value << animator.GetCurrentAnimationName();
  out << YAML::Key << "current_animation_time_" << YAML::Value << animator.GetCurrentAnimationTimePoint();

  const auto& transform_chain = animator.PeekTransformChain();
  if (!transform_chain.empty()) {
    out << YAML::Key << "transform_chain_" << YAML::Value
        << YAML::Binary(reinterpret_cast<const unsigned char*>(transform_chain.data()),
                        transform_chain.size() * sizeof(glm::mat4));
  }
  const auto& offset_matrices = animator.PeekOffsetMatrices();
  if (!offset_matrices.empty()) {
    out << YAML::Key << "offset_matrices_" << YAML::Value
        << YAML::Binary(reinterpret_cast<const unsigned char*>(offset_matrices.data()),
                        offset_matrices.size() * sizeof(glm::mat4));
  }
  if (const auto& names = animator.PeekBoneNames(); !names.empty()) {
    out << YAML::Key << "names_" << YAML::Value << YAML::BeginSeq;
    for (const auto& name : names) {
      out << YAML::BeginMap;
      out << YAML::Key << "Name" << YAML::Value << name;
      out << YAML::EndMap;
    }
    out << YAML::EndSeq;
  }
}

void DeserializeAnimator(const YAML::Node& in, Animator& animator) {
  animator.RefAnimationRef().Load("animation_", in);
  if (animator.GetAnimation()) {
    if (in["current_activated_animation_"] && in["current_animation_time_"]) {
      animator.RestorePlaybackState(in["current_activated_animation_"].as<std::string>(),
                                    in["current_animation_time_"].as<float>());
    }
    animator.RebuildAnimationState();
  }
  if (in["transform_chain_"]) {
    const auto chains = in["transform_chain_"].as<YAML::Binary>();
    auto& transform_chain = animator.RefTransformChain();
    transform_chain.resize(chains.size() / sizeof(glm::mat4));
    std::memcpy(transform_chain.data(), chains.data(), chains.size());
  }
  if (in["offset_matrices_"]) {
    const auto matrices = in["offset_matrices_"].as<YAML::Binary>();
    auto& offset_matrices = animator.RefOffsetMatrices();
    offset_matrices.resize(matrices.size() / sizeof(glm::mat4));
    std::memcpy(offset_matrices.data(), matrices.data(), matrices.size());
  }
  if (in["names_"]) {
    auto& names = animator.RefBoneNames();
    for (const auto& i : in["names_"]) {
      names.push_back(i["Name"].as<std::string>());
    }
  }
}

void SerializePostProcessingStack(YAML::Emitter& out, const PostProcessingStack& stack) {
  out << YAML::Key << "enable_screen_space_ambient_occlusion" << YAML::Value
      << stack.enable_screen_space_ambient_occlusion;
  out << YAML::Key << "enable_bloom" << YAML::Value << stack.enable_bloom;
  out << YAML::Key << "enable_screen_space_reflection" << YAML::Value << stack.enable_screen_space_reflection;
  out << YAML::Key << "enable_tone_mapping" << YAML::Value << stack.enable_tone_mapping;
  if (stack.screen_space_ambient_occlusion) {
    out << YAML::Key << "screen_space_ambient_occlusion" << YAML::Value << YAML::BeginMap;
    stack.screen_space_ambient_occlusion->Serialize(out);
    out << YAML::EndMap;
  }
  if (stack.bloom) {
    out << YAML::Key << "bloom" << YAML::Value << YAML::BeginMap;
    stack.bloom->Serialize(out);
    out << YAML::EndMap;
  }
  if (stack.screen_space_reflection) {
    out << YAML::Key << "screen_space_reflection" << YAML::Value << YAML::BeginMap;
    stack.screen_space_reflection->Serialize(out);
    out << YAML::EndMap;
  }
  if (stack.tone_mapping) {
    out << YAML::Key << "tone_mapping" << YAML::Value << YAML::BeginMap;
    stack.tone_mapping->Serialize(out);
    out << YAML::EndMap;
  }
}

void DeserializePostProcessingStack(const YAML::Node& in, PostProcessingStack& stack) {
  if (in["enable_screen_space_ambient_occlusion"])
    stack.enable_screen_space_ambient_occlusion = in["enable_screen_space_ambient_occlusion"].as<bool>();
  if (in["enable_bloom"])
    stack.enable_bloom = in["enable_bloom"].as<bool>();
  if (in["enable_screen_space_reflection"])
    stack.enable_screen_space_reflection = in["enable_screen_space_reflection"].as<bool>();
  if (in["enable_tone_mapping"])
    stack.enable_tone_mapping = in["enable_tone_mapping"].as<bool>();

  if (in["screen_space_ambient_occlusion"]) {
    if (!stack.screen_space_ambient_occlusion)
      stack.screen_space_ambient_occlusion = std::make_shared<ScreenSpaceAmbientOcclusion>();
    stack.screen_space_ambient_occlusion->Deserialize(in["screen_space_ambient_occlusion"]);
  }
  if (in["bloom"]) {
    if (!stack.bloom)
      stack.bloom = std::make_shared<Bloom>();
    stack.bloom->Deserialize(in["bloom"]);
  }
  if (in["screen_space_reflection"]) {
    if (!stack.screen_space_reflection)
      stack.screen_space_reflection = std::make_shared<ScreenSpaceReflection>();
    stack.screen_space_reflection->Deserialize(in["screen_space_reflection"]);
  }
  if (in["tone_mapping"]) {
    if (!stack.tone_mapping)
      stack.tone_mapping = std::make_shared<ToneMapping>();
    stack.tone_mapping->Deserialize(in["tone_mapping"]);
  }
}

void SaveMaterialProperties(const std::string& name, const MaterialProperties& material_properties,
                            YAML::Emitter& out) {
  out << YAML::Key << name << YAML::Value << YAML::BeginMap;
  out << YAML::Key << "albedo_color" << YAML::Value << material_properties.albedo_color;
  out << YAML::Key << "subsurface_color" << YAML::Value << material_properties.subsurface_color;
  out << YAML::Key << "subsurface_factor" << YAML::Value << material_properties.subsurface_factor;
  out << YAML::Key << "subsurface_radius" << YAML::Value << material_properties.subsurface_radius;
  out << YAML::Key << "metallic" << YAML::Value << material_properties.metallic;
  out << YAML::Key << "specular" << YAML::Value << material_properties.specular;
  out << YAML::Key << "specular_tint" << YAML::Value << material_properties.specular_tint;
  out << YAML::Key << "roughness" << YAML::Value << material_properties.roughness;
  out << YAML::Key << "sheen" << YAML::Value << material_properties.sheen;
  out << YAML::Key << "sheen_tint" << YAML::Value << material_properties.sheen_tint;
  out << YAML::Key << "clear_coat" << YAML::Value << material_properties.clear_coat;
  out << YAML::Key << "clear_coat_roughness" << YAML::Value << material_properties.clear_coat_roughness;
  out << YAML::Key << "ior" << YAML::Value << material_properties.ior;
  out << YAML::Key << "transmission" << YAML::Value << material_properties.transmission;
  out << YAML::Key << "transmission_roughness" << YAML::Value << material_properties.transmission_roughness;
  out << YAML::Key << "emission" << YAML::Value << material_properties.emission;
  out << YAML::EndMap;
}

void LoadMaterialProperties(const std::string& name, MaterialProperties& material_properties, const YAML::Node& in) {
  if (in[name]) {
    const auto& in_material_properties = in[name];
    if (in_material_properties["albedo_color"])
      material_properties.albedo_color = in_material_properties["albedo_color"].as<glm::vec3>();
    if (in_material_properties["subsurface_color"])
      material_properties.subsurface_color = in_material_properties["subsurface_color"].as<glm::vec3>();
    if (in_material_properties["subsurface_factor"])
      material_properties.subsurface_factor = in_material_properties["subsurface_factor"].as<float>();
    if (in_material_properties["subsurface_radius"])
      material_properties.subsurface_radius = in_material_properties["subsurface_radius"].as<glm::vec3>();
    if (in_material_properties["metallic"])
      material_properties.metallic = in_material_properties["metallic"].as<float>();
    if (in_material_properties["specular"])
      material_properties.specular = in_material_properties["specular"].as<float>();
    if (in_material_properties["specular_tint"])
      material_properties.specular_tint = in_material_properties["specular_tint"].as<float>();
    if (in_material_properties["roughness"])
      material_properties.roughness = in_material_properties["roughness"].as<float>();
    if (in_material_properties["m_sheen"])
      material_properties.sheen = in_material_properties["sheen"].as<float>();
    if (in_material_properties["sheen_tint"])
      material_properties.sheen_tint = in_material_properties["sheen_tint"].as<float>();
    if (in_material_properties["clear_coat"])
      material_properties.clear_coat = in_material_properties["clear_coat"].as<float>();
    if (in_material_properties["clear_coat_roughness"])
      material_properties.clear_coat_roughness = in_material_properties["clear_coat_roughness"].as<float>();
    if (in_material_properties["ior"])
      material_properties.ior = in_material_properties["ior"].as<float>();
    if (in_material_properties["transmission"])
      material_properties.transmission = in_material_properties["transmission"].as<float>();
    if (in_material_properties["transmission_roughness"])
      material_properties.transmission_roughness = in_material_properties["transmission_roughness"].as<float>();
    if (in_material_properties["emission"])
      material_properties.emission = in_material_properties["emission"].as<float>();
  }
}

void SerializeMaterial(YAML::Emitter& out, const Material& material) {
  material.PeekAlbedoTextureRef().Save("albedo_texture_", out);
  material.PeekNormalTextureRef().Save("normal_texture_", out);
  material.PeekMetallicTextureRef().Save("metallic_texture_", out);
  material.PeekRoughnessTextureRef().Save("roughness_texture_", out);
  material.PeekAoTextureRef().Save("ao_texture_", out);
  material.draw_settings.Save("draw_settings", out);
  SaveMaterialProperties("material_properties", material.material_properties, out);
  out << YAML::Key << "vertex_color_only" << YAML::Value << material.vertex_color_only;
}

void DeserializeMaterial(const YAML::Node& in, Material& material) {
  material.RefAlbedoTextureRef().Load("albedo_texture_", in);
  material.RefNormalTextureRef().Load("normal_texture_", in);
  material.RefMetallicTextureRef().Load("metallic_texture_", in);
  material.RefRoughnessTextureRef().Load("roughness_texture_", in);
  material.RefAoTextureRef().Load("ao_texture_", in);
  material.draw_settings.Load("draw_settings", in);
  LoadMaterialProperties("material_properties", material.material_properties, in);
  if (in["vertex_color_only"])
    material.vertex_color_only = in["vertex_color_only"].as<bool>();
}

void SerializeShader(YAML::Emitter& out, const Shader& shader) {
  out << YAML::Key << "shader_type" << YAML::Value << static_cast<unsigned>(shader.GetShaderType());
  out << YAML::Key << "shader_code" << YAML::Value << shader.PeekShaderCode();
}

void DeserializeShader(const YAML::Node& in, Shader& shader) {
  if (in["shader_code"])
    shader.RefShaderCode() = in["shader_code"].as<std::string>();
  if (in["shader_type"])
    shader.RefShaderType() = in["shader_type"].as<unsigned>();
}

template <typename T>
void SerializeProceduralNoise(YAML::Emitter& out, const T& noise) {
  procedural_noise::SaveProceduralNoiseGraph(out, noise);
}

template <typename T>
void DeserializeProceduralNoise(const YAML::Node& in, T& noise) {
  procedural_noise::LoadProceduralNoiseGraph(in, noise);
}

void SerializePointCloud(YAML::Emitter& out, const PointCloud& point_cloud) {
  out << YAML::Key << "offset" << point_cloud.offset;
  out << YAML::Key << "point_size" << point_cloud.point_size;
  out << YAML::Key << "compress_factor" << point_cloud.compress_factor;
  out << YAML::Key << "min_" << point_cloud.GetMinBound();
  out << YAML::Key << "max_" << point_cloud.GetMaxBound();
  if (!point_cloud.positions.empty()) {
    out << YAML::Key << "positions" << YAML::Value
        << YAML::Binary(reinterpret_cast<const unsigned char*>(point_cloud.positions.data()),
                        point_cloud.positions.size() * sizeof(glm::dvec3));
  }
  if (!point_cloud.normals.empty()) {
    out << YAML::Key << "normals" << YAML::Value
        << YAML::Binary(reinterpret_cast<const unsigned char*>(point_cloud.normals.data()),
                        point_cloud.normals.size() * sizeof(glm::dvec3));
  }
  if (!point_cloud.colors.empty()) {
    out << YAML::Key << "colors" << YAML::Value
        << YAML::Binary(reinterpret_cast<const unsigned char*>(point_cloud.colors.data()),
                        point_cloud.colors.size() * sizeof(glm::vec3));
  }
}

void DeserializePointCloud(const YAML::Node& in, PointCloud& point_cloud) {
  auto min_bound = point_cloud.GetMinBound();
  auto max_bound = point_cloud.GetMaxBound();
  if (in["offset"])
    point_cloud.offset = in["offset"].as<glm::dvec3>();
  if (in["point_size"])
    point_cloud.point_size = in["point_size"].as<float>();
  if (in["compress_factor"])
    point_cloud.compress_factor = in["compress_factor"].as<float>();
  if (in["min_"])
    min_bound = in["min_"].as<glm::dvec3>();
  if (in["max_"])
    max_bound = in["max_"].as<glm::dvec3>();
  point_cloud.SetBounds(min_bound, max_bound);
  if (in["positions"]) {
    const auto& vertex_data = in["positions"].as<YAML::Binary>();
    point_cloud.positions.resize(vertex_data.size() / sizeof(glm::dvec3));
    std::memcpy(point_cloud.positions.data(), vertex_data.data(), vertex_data.size());
  }
  if (in["colors"]) {
    const auto& vertex_data = in["colors"].as<YAML::Binary>();
    point_cloud.colors.resize(vertex_data.size() / sizeof(glm::vec3));
    std::memcpy(point_cloud.colors.data(), vertex_data.data(), vertex_data.size());
  }
  if (in["normals"]) {
    const auto& vertex_data = in["normals"].as<YAML::Binary>();
    point_cloud.normals.resize(vertex_data.size() / sizeof(glm::vec3));
    std::memcpy(point_cloud.normals.data(), vertex_data.data(), vertex_data.size());
  }
}

void SerializeGaussianSplat(YAML::Emitter& out, const GaussianSplat& gaussian_splat) {
  out << YAML::Key << "min_bound" << gaussian_splat.GetMinBound();
  out << YAML::Key << "max_bound" << gaussian_splat.GetMaxBound();
  out << YAML::Key << "spherical_harmonics_rest_float_count" << gaussian_splat.spherical_harmonics_rest_float_count;
  if (!gaussian_splat.positions.empty()) {
    out << YAML::Key << "positions" << YAML::Value
        << YAML::Binary(reinterpret_cast<const unsigned char*>(gaussian_splat.positions.data()),
                        gaussian_splat.positions.size() * sizeof(glm::vec3));
  }
  if (!gaussian_splat.scales.empty()) {
    out << YAML::Key << "scales" << YAML::Value
        << YAML::Binary(reinterpret_cast<const unsigned char*>(gaussian_splat.scales.data()),
                        gaussian_splat.scales.size() * sizeof(glm::vec3));
  }
  if (!gaussian_splat.rotations.empty()) {
    out << YAML::Key << "rotations" << YAML::Value
        << YAML::Binary(reinterpret_cast<const unsigned char*>(gaussian_splat.rotations.data()),
                        gaussian_splat.rotations.size() * sizeof(glm::vec4));
  }
  if (!gaussian_splat.opacities.empty()) {
    out << YAML::Key << "opacities" << YAML::Value
        << YAML::Binary(reinterpret_cast<const unsigned char*>(gaussian_splat.opacities.data()),
                        gaussian_splat.opacities.size() * sizeof(float));
  }
  if (!gaussian_splat.colors.empty()) {
    out << YAML::Key << "colors" << YAML::Value
        << YAML::Binary(reinterpret_cast<const unsigned char*>(gaussian_splat.colors.data()),
                        gaussian_splat.colors.size() * sizeof(glm::vec3));
  }
  if (!gaussian_splat.spherical_harmonics_rest.empty()) {
    out << YAML::Key << "spherical_harmonics_rest" << YAML::Value
        << YAML::Binary(reinterpret_cast<const unsigned char*>(gaussian_splat.spherical_harmonics_rest.data()),
                        gaussian_splat.spherical_harmonics_rest.size() * sizeof(float));
  }
}

void DeserializeGaussianSplat(const YAML::Node& in, GaussianSplat& gaussian_splat) {
  auto min_bound = gaussian_splat.GetMinBound();
  auto max_bound = gaussian_splat.GetMaxBound();
  if (in["min_bound"])
    min_bound = in["min_bound"].as<glm::vec3>();
  if (in["max_bound"])
    max_bound = in["max_bound"].as<glm::vec3>();
  if (in["spherical_harmonics_rest_float_count"])
    gaussian_splat.spherical_harmonics_rest_float_count = in["spherical_harmonics_rest_float_count"].as<uint32_t>();
  gaussian_splat.SetBounds(min_bound, max_bound);
  if (in["positions"]) {
    const auto& data = in["positions"].as<YAML::Binary>();
    gaussian_splat.positions.resize(data.size() / sizeof(glm::vec3));
    std::memcpy(gaussian_splat.positions.data(), data.data(), data.size());
  }
  if (in["scales"]) {
    const auto& data = in["scales"].as<YAML::Binary>();
    gaussian_splat.scales.resize(data.size() / sizeof(glm::vec3));
    std::memcpy(gaussian_splat.scales.data(), data.data(), data.size());
  }
  if (in["rotations"]) {
    const auto& data = in["rotations"].as<YAML::Binary>();
    gaussian_splat.rotations.resize(data.size() / sizeof(glm::vec4));
    std::memcpy(gaussian_splat.rotations.data(), data.data(), data.size());
  }
  if (in["opacities"]) {
    const auto& data = in["opacities"].as<YAML::Binary>();
    gaussian_splat.opacities.resize(data.size() / sizeof(float));
    std::memcpy(gaussian_splat.opacities.data(), data.data(), data.size());
  }
  if (in["colors"]) {
    const auto& data = in["colors"].as<YAML::Binary>();
    gaussian_splat.colors.resize(data.size() / sizeof(glm::vec3));
    std::memcpy(gaussian_splat.colors.data(), data.data(), data.size());
  }
  if (in["spherical_harmonics_rest"]) {
    const auto& data = in["spherical_harmonics_rest"].as<YAML::Binary>();
    gaussian_splat.spherical_harmonics_rest.resize(data.size() / sizeof(float));
    std::memcpy(gaussian_splat.spherical_harmonics_rest.data(), data.data(), data.size());
  }
  gaussian_splat.InvalidateGpuCaches();
}

void SerializeTexture2D(YAML::Emitter& out, const Texture2D& texture) {
  std::vector<glm::vec4> pixels;
  if (texture.PeekLocalData().empty())
    texture.GetRgbaChannelData(pixels);
  else
    pixels = texture.PeekLocalData();

  out << YAML::Key << "hdr" << YAML::Value << texture.hdr;
  out << YAML::Key << "red_channel" << YAML::Value << texture.red_channel;
  out << YAML::Key << "green_channel" << YAML::Value << texture.green_channel;
  out << YAML::Key << "blue_channel" << YAML::Value << texture.blue_channel;
  out << YAML::Key << "alpha_channel" << YAML::Value << texture.alpha_channel;
  const auto resolution = texture.GetResolution();
  out << YAML::Key << "resolution" << YAML::Value << resolution;
  if (resolution.x == 0 || resolution.y == 0) {
    return;
  }
  if (texture.hdr) {
    Serialization::SerializeVector("pixels", pixels, out);
    return;
  }

  size_t target_channel_size = 0;
  if (texture.red_channel)
    target_channel_size++;
  if (texture.green_channel)
    target_channel_size++;
  if (texture.blue_channel)
    target_channel_size++;
  if (texture.alpha_channel)
    target_channel_size++;

  std::vector<unsigned char> transferred_pixels;
  transferred_pixels.resize(resolution.x * resolution.y * target_channel_size);
  Jobs::RunParallelFor(resolution.x * resolution.y, [&](size_t i) {
    for (int channel = 0; channel < target_channel_size; channel++) {
      transferred_pixels[i * target_channel_size + channel] =
          static_cast<unsigned char>(glm::clamp(pixels[i][channel] * 255.9f, 0.f, 255.f));
    }
  });
  Serialization::SerializeVector("pixels", transferred_pixels, out);
}

void DeserializeTexture2D(const YAML::Node& in, Texture2D& texture) {
  std::vector<glm::vec4> pixels;
  glm::ivec2 resolution = glm::ivec2(0);
  if (in["red_channel"])
    texture.red_channel = in["red_channel"].as<bool>();
  if (in["green_channel"])
    texture.green_channel = in["green_channel"].as<bool>();
  if (in["blue_channel"])
    texture.blue_channel = in["blue_channel"].as<bool>();
  if (in["alpha_channel"])
    texture.alpha_channel = in["alpha_channel"].as<bool>();
  if (in["hdr"])
    texture.hdr = in["hdr"].as<bool>();
  if (in["resolution"])
    resolution = in["resolution"].as<glm::ivec2>();
  if (resolution.x == 0 || resolution.y == 0) {
    return;
  }
  if (texture.hdr) {
    Serialization::DeserializeVector("pixels", pixels, in);
    texture.SetRgbaChannelData(pixels, resolution);
    return;
  }

  size_t target_channel_size = 0;
  if (texture.red_channel)
    target_channel_size++;
  if (texture.green_channel)
    target_channel_size++;
  if (texture.blue_channel)
    target_channel_size++;
  if (texture.alpha_channel)
    target_channel_size++;

  std::vector<unsigned char> transferred_pixels;
  Serialization::DeserializeVector("pixels", transferred_pixels, in);
  transferred_pixels.resize(resolution.x * resolution.y * target_channel_size);
  pixels.resize(resolution.x * resolution.y);
  Jobs::RunParallelFor(pixels.size(), [&](size_t i) {
    for (int channel = 0; channel < target_channel_size; channel++) {
      pixels[i][channel] = glm::clamp(transferred_pixels[i * target_channel_size + channel] / 256.f, 0.f, 1.f);
    }
    if (target_channel_size < 4) {
      pixels[i][3] = 1.f;
    }
    if (target_channel_size < 3) {
      pixels[i][2] = 0.f;
    }
    if (target_channel_size < 2) {
      pixels[i][1] = 0.f;
    }
  });
  texture.SetRgbaChannelData(pixels, resolution);
}

void SerializeAnimation(YAML::Emitter& out, const Animation& animation) {
  out << YAML::Key << "bone_size" << YAML::Value << animation.bone_size;
  if (!animation.animation_length.empty()) {
    out << YAML::Key << "animation_length" << YAML::Value << YAML::BeginSeq;
    for (const auto& [name, length] : animation.animation_length) {
      out << YAML::BeginMap;
      out << YAML::Key << "Name" << YAML::Value << name;
      out << YAML::Key << "Length" << YAML::Value << length;
      out << YAML::EndMap;
    }
    out << YAML::EndSeq;
  }
  if (animation.root_bone) {
    out << YAML::Key << "root_bone" << YAML::Value << YAML::BeginMap;
    animation.root_bone->Serialize(out);
    out << YAML::EndMap;
  }
}

void DeserializeAnimation(const YAML::Node& in, Animation& animation) {
  animation.bone_size = in["bone_size"].as<size_t>();
  animation.animation_length.clear();
  if (const auto in_animation_name_and_length = in["animation_length"]) {
    for (const auto& i : in_animation_name_and_length) {
      animation.animation_length.insert({i["Name"].as<std::string>(), i["Length"].as<float>()});
    }
  }
  if (in["root_bone"]) {
    animation.root_bone = std::make_shared<Bone>();
    animation.root_bone->Deserialize(in["root_bone"]);
  }
}

void SerializeParticleInfoList(YAML::Emitter& out, const ParticleInfoList& list) {
  Serialization::SerializeVector("particle_infos", list.PeekParticleInfoList(), out);
}

void DeserializeParticleInfoList(const YAML::Node& in, ParticleInfoList& list) {
  if (in["particle_infos"]) {
    std::vector<ParticleInfo> particle_infos;
    Serialization::DeserializeVector("particle_infos", particle_infos, in);
    list.SetParticleInfos(particle_infos);
  }
}

VertexAttributes DefaultVertexAttributes() {
  VertexAttributes attributes{};
  attributes.normal = true;
  attributes.tangent = true;
  attributes.tex_coord = true;
  attributes.color = true;
  return attributes;
}

SkinnedVertexAttributes DefaultSkinnedVertexAttributes() {
  SkinnedVertexAttributes attributes{};
  attributes.normal = true;
  attributes.tangent = true;
  attributes.tex_coord = true;
  attributes.color = true;
  return attributes;
}

StrandPointAttributes DefaultStrandPointAttributes() {
  StrandPointAttributes attributes{};
  attributes.normal = true;
  attributes.tex_coord = true;
  attributes.color = true;
  return attributes;
}

void SerializeMesh(YAML::Emitter& out, const Mesh& mesh) {
  out << YAML::Key << "vertex_attributes_" << YAML::BeginMap;
  mesh.GetVertexAttributes().Serialize(out);
  out << YAML::EndMap;

  const auto& vertices = mesh.PeekVertices();
  const auto& triangles = mesh.PeekTriangles();
  if (!vertices.empty() && !triangles.empty()) {
    out << YAML::Key << "vertices_" << YAML::Value
        << YAML::Binary(reinterpret_cast<const unsigned char*>(vertices.data()), vertices.size() * sizeof(Vertex));
    out << YAML::Key << "triangles_" << YAML::Value
        << YAML::Binary(reinterpret_cast<const unsigned char*>(triangles.data()),
                        triangles.size() * sizeof(glm::uvec3));
  }
}

void DeserializeMesh(const YAML::Node& in, Mesh& mesh) {
  auto vertex_attributes = DefaultVertexAttributes();
  if (in["vertex_attributes_"]) {
    vertex_attributes.Deserialize(in["vertex_attributes_"]);
  }

  if (in["vertices_"] && in["triangles_"]) {
    const auto& vertex_data = in["vertices_"].as<YAML::Binary>();
    std::vector<Vertex> vertices;
    vertices.resize(vertex_data.size() / sizeof(Vertex));
    std::memcpy(vertices.data(), vertex_data.data(), vertex_data.size());

    const auto& triangle_data = in["triangles_"].as<YAML::Binary>();
    std::vector<glm::uvec3> triangles;
    triangles.resize(triangle_data.size() / sizeof(glm::uvec3));
    std::memcpy(triangles.data(), triangle_data.data(), triangle_data.size());

    mesh.SetVertices(vertex_attributes, vertices, triangles);
  }
}

void SerializeSkinnedMesh(YAML::Emitter& out, const SkinnedMesh& mesh) {
  if (!mesh.bone_animator_indices.empty()) {
    out << YAML::Key << "bone_animator_indices" << YAML::Value
        << YAML::Binary(reinterpret_cast<const unsigned char*>(mesh.bone_animator_indices.data()),
                        mesh.bone_animator_indices.size() * sizeof(unsigned));
  }

  out << YAML::Key << "skinned_vertex_attributes_" << YAML::BeginMap;
  mesh.GetSkinnedVertexAttributes().Serialize(out);
  out << YAML::EndMap;

  const auto& vertices = mesh.PeekSkinnedVertices();
  const auto& triangles = mesh.PeekTriangles();
  if (!vertices.empty() && !triangles.empty()) {
    out << YAML::Key << "skinned_vertices_" << YAML::Value
        << YAML::Binary(reinterpret_cast<const unsigned char*>(vertices.data()),
                        vertices.size() * sizeof(SkinnedVertex));
    out << YAML::Key << "skinned_triangles_" << YAML::Value
        << YAML::Binary(reinterpret_cast<const unsigned char*>(triangles.data()),
                        triangles.size() * sizeof(glm::uvec3));
  }
}

void DeserializeSkinnedMesh(const YAML::Node& in, SkinnedMesh& mesh) {
  if (in["bone_animator_indices"]) {
    const auto& bone_indices = in["bone_animator_indices"].as<YAML::Binary>();
    mesh.bone_animator_indices.resize(bone_indices.size() / sizeof(unsigned));
    std::memcpy(mesh.bone_animator_indices.data(), bone_indices.data(), bone_indices.size());
  }

  auto vertex_attributes = DefaultSkinnedVertexAttributes();
  if (in["skinned_vertex_attributes_"]) {
    vertex_attributes.Deserialize(in["skinned_vertex_attributes_"]);
  }

  if (in["skinned_vertices_"] && in["skinned_triangles_"]) {
    const auto& vertex_data = in["skinned_vertices_"].as<YAML::Binary>();
    std::vector<SkinnedVertex> vertices;
    vertices.resize(vertex_data.size() / sizeof(SkinnedVertex));
    std::memcpy(vertices.data(), vertex_data.data(), vertex_data.size());

    const auto& triangle_data = in["skinned_triangles_"].as<YAML::Binary>();
    std::vector<glm::uvec3> triangles;
    triangles.resize(triangle_data.size() / sizeof(glm::uvec3));
    std::memcpy(triangles.data(), triangle_data.data(), triangle_data.size());

    mesh.SetVertices(vertex_attributes, vertices, triangles);
  }
}

void SerializeStrands(YAML::Emitter& out, const Strands& strands) {
  const auto& segment_indices = strands.PeekSegments();
  const auto& points = strands.PeekStrandPoints();
  if (!segment_indices.empty() && !points.empty()) {
    out << YAML::Key << "segment_raw_indices_" << YAML::Value
        << YAML::Binary(reinterpret_cast<const unsigned char*>(segment_indices.data()),
                        segment_indices.size() * sizeof(glm::uint));
    out << YAML::Key << "strand_points_" << YAML::Value
        << YAML::Binary(reinterpret_cast<const unsigned char*>(points.data()), points.size() * sizeof(StrandPoint));
  }
}

void DeserializeStrands(const YAML::Node& in, Strands& strands) {
  if (in["segment_raw_indices_"] && in["strand_points_"]) {
    const auto& segment_data = in["segment_raw_indices_"].as<YAML::Binary>();
    std::vector<glm::uint> segment_indices;
    segment_indices.resize(segment_data.size() / sizeof(glm::uint));
    std::memcpy(segment_indices.data(), segment_data.data(), segment_data.size());

    const auto& point_data = in["strand_points_"].as<YAML::Binary>();
    std::vector<StrandPoint> points;
    points.resize(point_data.size() / sizeof(StrandPoint));
    std::memcpy(points.data(), point_data.data(), point_data.size());

    strands.SetSegments(DefaultStrandPointAttributes(), segment_indices, points);
  }
}

void SerializePrefab(YAML::Emitter& out, const Prefab& prefab) {
  out << YAML::Key << "in" << YAML::Value << prefab.instance_name;
  out << YAML::Key << "e" << YAML::Value << prefab.IsPrefabEnabled();
  out << YAML::Key << "eh" << YAML::Value << prefab.entity_handle.GetValue();

  if (!prefab.data_components.empty()) {
    out << YAML::Key << "dc" << YAML::BeginSeq;
    for (const auto& component : prefab.data_components) {
      out << YAML::BeginMap;
      component.Serialize(out);
      out << YAML::EndMap;
    }
    out << YAML::EndSeq;
  }

  if (!prefab.private_components.empty()) {
    out << YAML::Key << "pc" << YAML::BeginSeq;
    for (const auto& component : prefab.private_components) {
      out << YAML::BeginMap;
      component.Serialize(out);
      out << YAML::EndMap;
    }
    out << YAML::EndSeq;
  }

  if (!prefab.child_prefabs.empty()) {
    out << YAML::Key << "c" << YAML::BeginSeq;
    for (const auto& child : prefab.child_prefabs) {
      out << YAML::BeginMap;
      out << YAML::Key << "h" << child->GetHandle().GetValue();
      Serialization::SerializeObject(out, *child);
      out << YAML::EndMap;
    }
    out << YAML::EndSeq;
  }
}

void DeserializePrefab(const YAML::Node& in, Prefab& prefab) {
  prefab.instance_name = in["in"].as<std::string>();
  prefab.SetPrefabEnabled(in["e"].as<bool>());
  prefab.entity_handle = Handle(in["eh"].as<uint64_t>());
  if (in["dc"]) {
    for (const auto& i : in["dc"]) {
      DataComponentHolder holder;
      if (holder.Deserialize(i)) {
        prefab.data_components.push_back(holder);
      }
    }
  }
  std::vector<std::pair<int, std::shared_ptr<IAsset>>> local_assets;
  if (const auto in_local_assets = in["LocalAssets"]) {
    int index = 0;
    for (const auto& i : in_local_assets) {
      if (const auto type_name = i["TypeName"].as<std::string>(); Serialization::HasSerializableType(type_name)) {
        auto asset = AssetManager::CreateTemporaryAsset(type_name, Handle(i["Handle"].as<uint64_t>()));
        local_assets.emplace_back(index, asset);
      }
      index++;
    }

    for (const auto& i : local_assets) {
      Serialization::DeserializeObject(in_local_assets[i.first], *i.second);
    }
  }
#ifdef _DEBUG
  EVOENGINE_LOG(std::string("Prefab Deserialization: Loaded " + std::to_string(local_assets.size()) + " assets."))
#endif
  if (in["pc"]) {
    for (const auto& i : in["pc"]) {
      PrivateComponentHolder holder;
      holder.Deserialize(i);
      prefab.private_components.push_back(holder);
    }
  }

  if (in["c"]) {
    for (const auto& i : in["c"]) {
      auto child = std::dynamic_pointer_cast<Prefab>(
          AssetManager::CreateTemporaryAsset("Prefab", Handle(i["h"].as<uint64_t>())));
      if (!child) {
        continue;
      }
      Serialization::DeserializeObject(i, *child);
      prefab.child_prefabs.push_back(child);
    }
  }
}

void SerializeMeshRenderer(YAML::Emitter& out, const MeshRenderer& renderer) {
  out << YAML::Key << "cast_shadow" << YAML::Value << renderer.cast_shadow;
  renderer.mesh.Save("mesh", out);
  renderer.material.Save("material", out);
}

void DeserializeMeshRenderer(const YAML::Node& in, MeshRenderer& renderer) {
  renderer.cast_shadow = in["cast_shadow"].as<bool>();
  renderer.mesh.Load("mesh", in);
  renderer.material.Load("material", in);
}

void SerializeStrandsRenderer(YAML::Emitter& out, const StrandsRenderer& renderer) {
  out << YAML::Key << "cast_shadow" << YAML::Value << renderer.cast_shadow;
  renderer.strands.Save("strands", out);
  renderer.material.Save("material", out);
}

void DeserializeStrandsRenderer(const YAML::Node& in, StrandsRenderer& renderer) {
  renderer.cast_shadow = in["cast_shadow"].as<bool>();
  renderer.strands.Load("strands", in);
  renderer.material.Load("material", in);
}

void SerializeGaussianSplatRenderer(YAML::Emitter& out, const GaussianSplatRenderer& renderer) {
  renderer.gaussian_splat.Save("gaussian_splat", out);
  out << YAML::Key << "opacity_scale" << YAML::Value << renderer.opacity_scale;
  out << YAML::Key << "sh_degree" << YAML::Value << renderer.sh_degree;
  out << YAML::Key << "sort_mode" << YAML::Value << static_cast<int>(renderer.sort_mode);
  out << YAML::Key << "depth_mode" << YAML::Value << static_cast<int>(renderer.depth_mode);
}

void DeserializeGaussianSplatRenderer(const YAML::Node& in, GaussianSplatRenderer& renderer) {
  renderer.gaussian_splat.Load("gaussian_splat", in);
  if (in["opacity_scale"])
    renderer.opacity_scale = in["opacity_scale"].as<float>();
  if (in["sh_degree"])
    renderer.sh_degree = in["sh_degree"].as<int>();
  if (in["sort_mode"])
    renderer.sort_mode = static_cast<GaussianSplatSortMode>(in["sort_mode"].as<int>());
  if (in["depth_mode"])
    renderer.depth_mode = static_cast<GaussianSplatDepthMode>(in["depth_mode"].as<int>());
}

void SerializeSkinnedMeshRenderer(YAML::Emitter& out, const SkinnedMeshRenderer& renderer) {
  out << YAML::Key << "cast_shadow" << renderer.cast_shadow;
  renderer.animator.Save("animator", out);
  renderer.skinned_mesh.Save("skinned_mesh", out);
  renderer.material.Save("material", out);
  out << YAML::Key << "rag_doll_" << YAML::Value << renderer.RagDoll();
  out << YAML::Key << "rag_doll_freeze" << YAML::Value << renderer.rag_doll_freeze;

  if (const auto& bound_entities = renderer.PeekRagDollBoundEntities(); !bound_entities.empty()) {
    out << YAML::Key << "bound_entities_" << YAML::Value << YAML::BeginSeq;
    for (const auto& bound_entity : bound_entities) {
      out << YAML::BeginMap;
      bound_entity.Serialize(out);
      out << YAML::EndMap;
    }
    out << YAML::EndSeq;
  }

  const auto& transform_chain = renderer.PeekRagDollTransformChain();
  if (!transform_chain.empty()) {
    out << YAML::Key << "rag_doll_transform_chain_" << YAML::Value
        << YAML::Binary(reinterpret_cast<const unsigned char*>(transform_chain.data()),
                        transform_chain.size() * sizeof(glm::mat4));
  }
}

void DeserializeSkinnedMeshRenderer(const YAML::Node& in, SkinnedMeshRenderer& renderer) {
  renderer.cast_shadow = in["cast_shadow"].as<bool>();
  renderer.animator.Load("animator", in, renderer.GetScene());
  renderer.skinned_mesh.Load("skinned_mesh", in);
  renderer.material.Load("material", in);
  renderer.SetRagDollState(in["rag_doll_"].as<bool>());
  renderer.rag_doll_freeze = in["rag_doll_freeze"].as<bool>();
  if (const auto in_bound_entities = in["bound_entities_"]) {
    auto& bound_entities = renderer.RefRagDollBoundEntities();
    for (const auto& i : in_bound_entities) {
      EntityRef ref;
      ref.Deserialize(i);
      bound_entities.push_back(ref);
    }
  }

  if (in["rag_doll_transform_chain_"]) {
    const auto chains = in["rag_doll_transform_chain_"].as<YAML::Binary>();
    auto& transform_chain = renderer.RefRagDollTransformChain();
    transform_chain.resize(chains.size() / sizeof(glm::mat4));
    std::memcpy(transform_chain.data(), chains.data(), chains.size());
  }
}

void SerializeParticles(YAML::Emitter& out, const Particles& particles) {
  out << YAML::Key << "cast_shadow" << YAML::Value << particles.cast_shadow;
  particles.mesh.Save("mesh", out);
  particles.material.Save("material", out);
  particles.particle_info_list.Save("particle_info_list", out);
}

void DeserializeParticles(const YAML::Node& in, Particles& particles) {
  particles.cast_shadow = in["cast_shadow"].as<bool>();
  particles.mesh.Load("mesh", in);
  particles.material.Load("material", in);
  particles.particle_info_list.Load("particle_info_list", in);
}

void SerializeSpotLight(YAML::Emitter& out, const SpotLight& light) {
  out << YAML::Key << "cast_shadow" << YAML::Value << light.cast_shadow;
  out << YAML::Key << "shadow_distance" << YAML::Value << light.shadow_distance;
  out << YAML::Key << "inner_degrees" << YAML::Value << light.inner_degrees;
  out << YAML::Key << "outer_degrees" << YAML::Value << light.outer_degrees;
  out << YAML::Key << "constant" << YAML::Value << light.constant;
  out << YAML::Key << "linear" << YAML::Value << light.linear;
  out << YAML::Key << "quadratic" << YAML::Value << light.quadratic;
  out << YAML::Key << "bias" << YAML::Value << light.bias;
  out << YAML::Key << "diffuse" << YAML::Value << light.diffuse;
  out << YAML::Key << "diffuse_brightness" << YAML::Value << light.diffuse_brightness;
  out << YAML::Key << "light_size" << YAML::Value << light.light_size;
}

void DeserializeSpotLight(const YAML::Node& in, SpotLight& light) {
  light.cast_shadow = in["cast_shadow"].as<bool>();
  if (in["shadow_distance"])
    light.shadow_distance = in["shadow_distance"].as<float>();
  light.inner_degrees = in["inner_degrees"].as<float>();
  light.outer_degrees = in["outer_degrees"].as<float>();
  light.constant = in["constant"].as<float>();
  light.linear = in["linear"].as<float>();
  light.quadratic = in["quadratic"].as<float>();
  light.bias = in["bias"].as<float>();
  light.diffuse = in["diffuse"].as<glm::vec3>();
  light.diffuse_brightness = in["diffuse_brightness"].as<float>();
  light.light_size = in["light_size"].as<float>();
}

void SerializePointLight(YAML::Emitter& out, const PointLight& light) {
  out << YAML::Key << "cast_shadow" << YAML::Value << light.cast_shadow;
  out << YAML::Key << "shadow_distance" << YAML::Value << light.shadow_distance;
  out << YAML::Key << "constant" << YAML::Value << light.constant;
  out << YAML::Key << "linear" << YAML::Value << light.linear;
  out << YAML::Key << "quadratic" << YAML::Value << light.quadratic;
  out << YAML::Key << "bias" << YAML::Value << light.bias;
  out << YAML::Key << "diffuse" << YAML::Value << light.diffuse;
  out << YAML::Key << "diffuse_brightness" << YAML::Value << light.diffuse_brightness;
  out << YAML::Key << "light_size" << YAML::Value << light.light_size;
}

void DeserializePointLight(const YAML::Node& in, PointLight& light) {
  light.cast_shadow = in["cast_shadow"].as<bool>();
  if (in["shadow_distance"])
    light.shadow_distance = in["shadow_distance"].as<float>();
  light.constant = in["constant"].as<float>();
  light.linear = in["linear"].as<float>();
  light.quadratic = in["quadratic"].as<float>();
  light.bias = in["bias"].as<float>();
  light.diffuse = in["diffuse"].as<glm::vec3>();
  light.diffuse_brightness = in["diffuse_brightness"].as<float>();
  light.light_size = in["light_size"].as<float>();
}

void SerializeDirectionalLight(YAML::Emitter& out, const DirectionalLight& light) {
  out << YAML::Key << "cast_shadow" << YAML::Value << light.cast_shadow;
  out << YAML::Key << "bias" << YAML::Value << light.bias;
  out << YAML::Key << "diffuse" << YAML::Value << light.diffuse;
  out << YAML::Key << "diffuse_brightness" << YAML::Value << light.diffuse_brightness;
  out << YAML::Key << "light_size" << YAML::Value << light.light_size;
  out << YAML::Key << "normal_offset" << YAML::Value << light.normal_offset;
}

void DeserializeDirectionalLight(const YAML::Node& in, DirectionalLight& light) {
  light.cast_shadow = in["cast_shadow"].as<bool>();
  light.bias = in["bias"].as<float>();
  light.diffuse = in["diffuse"].as<glm::vec3>();
  light.diffuse_brightness = in["diffuse_brightness"].as<float>();
  light.light_size = in["light_size"].as<float>();
  light.normal_offset = in["normal_offset"].as<float>();
}

void SerializeDdgiVolume(YAML::Emitter& out, const DdgiVolume& volume) {
  out << YAML::Key << "probe_counts" << YAML::Value << volume.probe_counts;
  out << YAML::Key << "probe_spacing" << YAML::Value << volume.probe_spacing;
  out << YAML::Key << "volume_origin" << YAML::Value << volume.volume_origin;
  out << YAML::Key << "movement_type" << YAML::Value << volume.movement_type;
  out << YAML::Key << "enable_probe_relocation" << YAML::Value << volume.enable_probe_relocation;
  out << YAML::Key << "enable_probe_classification" << YAML::Value << volume.enable_probe_classification;
  out << YAML::Key << "enable_probe_variability" << YAML::Value << volume.enable_probe_variability;
  out << YAML::Key << "enable_probe_variability_gating" << YAML::Value << volume.enable_probe_variability_gating;
  out << YAML::Key << "relocation_distance" << YAML::Value << volume.relocation_distance;
  out << YAML::Key << "random_ray_backface_threshold" << YAML::Value << volume.random_ray_backface_threshold;
  out << YAML::Key << "fixed_ray_backface_threshold" << YAML::Value << volume.fixed_ray_backface_threshold;
  out << YAML::Key << "probe_variability_threshold" << YAML::Value << volume.probe_variability_threshold;
  out << YAML::Key << "probe_variability_min_samples" << YAML::Value << volume.probe_variability_min_samples;
  out << YAML::Key << "warmup_trigger_conditions" << YAML::Value << volume.warmup_trigger_conditions;
  out << YAML::Key << "variability_reset_trigger_conditions" << YAML::Value
      << volume.variability_reset_trigger_conditions;
  out << YAML::Key << "visualize_bounds" << YAML::Value << volume.visualize_bounds;
  out << YAML::Key << "visualize_probe_positions" << YAML::Value << volume.visualize_probe_positions;
  out << YAML::Key << "max_visualized_probes" << YAML::Value << volume.max_visualized_probes;
  out << YAML::Key << "probe_visualization_size" << YAML::Value << volume.probe_visualization_size;
}

void DeserializeDdgiVolume(const YAML::Node& in, DdgiVolume& volume) {
  if (in["probe_counts"])
    volume.probe_counts = in["probe_counts"].as<glm::ivec3>();
  if (in["probe_spacing"])
    volume.probe_spacing = DeserializeDdgiProbeSpacing(in["probe_spacing"]);
  if (in["volume_origin"])
    volume.volume_origin = in["volume_origin"].as<glm::vec3>();
  else if (in["probe_offset"])
    volume.volume_origin =
        in["probe_offset"].as<glm::vec3>() +
        glm::vec3(glm::clamp(volume.probe_counts.x, 1, 256) - 1, glm::clamp(volume.probe_counts.y, 1, 256) - 1,
                  glm::clamp(volume.probe_counts.z, 1, 256) - 1) *
            (glm::clamp(volume.probe_spacing, glm::vec3(0.05f), glm::vec3(10000.0f)) * 0.5f);
  if (in["movement_type"])
    volume.movement_type = in["movement_type"].as<int>();
  if (in["enable_probe_relocation"])
    volume.enable_probe_relocation = in["enable_probe_relocation"].as<bool>();
  if (in["enable_probe_classification"])
    volume.enable_probe_classification = in["enable_probe_classification"].as<bool>();
  if (in["enable_probe_variability"])
    volume.enable_probe_variability = in["enable_probe_variability"].as<bool>();
  if (in["enable_probe_variability_gating"])
    volume.enable_probe_variability_gating = in["enable_probe_variability_gating"].as<bool>();
  if (in["relocation_distance"])
    volume.relocation_distance = in["relocation_distance"].as<float>();
  if (in["random_ray_backface_threshold"])
    volume.random_ray_backface_threshold = in["random_ray_backface_threshold"].as<float>();
  if (in["fixed_ray_backface_threshold"])
    volume.fixed_ray_backface_threshold = in["fixed_ray_backface_threshold"].as<float>();
  if (in["probe_variability_threshold"])
    volume.probe_variability_threshold = in["probe_variability_threshold"].as<float>();
  if (in["probe_variability_min_samples"])
    volume.probe_variability_min_samples = in["probe_variability_min_samples"].as<int>();
  if (in["warmup_trigger_conditions"])
    volume.warmup_trigger_conditions = in["warmup_trigger_conditions"].as<int>();
  if (in["variability_reset_trigger_conditions"])
    volume.variability_reset_trigger_conditions = in["variability_reset_trigger_conditions"].as<int>();
  if (in["visualize_bounds"])
    volume.visualize_bounds = in["visualize_bounds"].as<bool>();
  if (in["visualize_probe_positions"])
    volume.visualize_probe_positions = in["visualize_probe_positions"].as<bool>();
  if (in["max_visualized_probes"])
    volume.max_visualized_probes = in["max_visualized_probes"].as<int>();
  if (in["probe_visualization_size"])
    volume.probe_visualization_size = in["probe_visualization_size"].as<float>();
  volume.ClampSettings();
}

void SerializePointCloudScanner(YAML::Emitter&, const PointCloudScanner&) {
}

void DeserializePointCloudScanner(const YAML::Node&, PointCloudScanner&) {
}

void SerializePlayerController(YAML::Emitter& out, const PlayerController& controller) {
  out << YAML::Key << "velocity" << YAML::Value << controller.velocity;
  out << YAML::Key << "sensitivity" << YAML::Value << controller.sensitivity;
  out << YAML::Key << "scene_camera_yaw_angle_" << YAML::Value << controller.GetSceneCameraYawAngle();
  out << YAML::Key << "scene_camera_pitch_angle_" << YAML::Value << controller.GetSceneCameraPitchAngle();
}

void DeserializePlayerController(const YAML::Node& in, PlayerController& controller) {
  if (in["velocity"])
    controller.velocity = in["velocity"].as<float>();
  if (in["sensitivity"])
    controller.sensitivity = in["sensitivity"].as<float>();
  if (in["scene_camera_yaw_angle_"])
    controller.RefSceneCameraYawAngle() = in["scene_camera_yaw_angle_"].as<float>();
  if (in["scene_camera_pitch_angle_"])
    controller.RefSceneCameraPitchAngle() = in["scene_camera_pitch_angle_"].as<float>();
}

void SerializeLodGroup(YAML::Emitter& out, const LodGroup& group) {
  out << YAML::Key << "override_lod_factor" << YAML::Value << group.override_lod_factor;
  out << YAML::Key << "lod_factor" << YAML::Value << group.lod_factor;
  if (group.lods.empty()) {
    return;
  }
  out << YAML::Key << "lods" << YAML::BeginSeq;
  for (const auto& lod : group.lods) {
    out << YAML::BeginMap;
    out << YAML::Key << "index" << YAML::Value << lod.index;
    out << YAML::Key << "lod_offset" << YAML::Value << lod.lod_offset;
    out << YAML::Key << "transition_width" << YAML::Value << lod.transition_width;
    if (!lod.renderers.empty()) {
      out << YAML::Key << "renderers" << YAML::BeginSeq;
      for (const auto& renderer : lod.renderers) {
        out << YAML::BeginMap;
        renderer.Serialize(out);
        out << YAML::EndMap;
      }
      out << YAML::EndSeq;
    }
    out << YAML::EndMap;
  }
  out << YAML::EndSeq;
}

void DeserializeLodGroup(const YAML::Node& in, LodGroup& group) {
  const auto scene = group.GetScene();
  if (in["override_lod_factor"])
    group.override_lod_factor = in["override_lod_factor"].as<bool>();
  if (in["lod_factor"])
    group.lod_factor = in["lod_factor"].as<float>();
  if (!in["lods"]) {
    return;
  }
  group.lods.clear();
  for (const auto& in_lod : in["lods"]) {
    group.lods.emplace_back();
    auto& lod = group.lods.back();
    lod.renderers.clear();
    if (in_lod["index"])
      lod.index = in_lod["index"].as<int>();
    if (in_lod["lod_offset"])
      lod.lod_offset = in_lod["lod_offset"].as<float>();
    if (in_lod["transition_width"])
      lod.transition_width = in_lod["transition_width"].as<float>();
    if (in_lod["renderers"]) {
      for (const auto& in_renderer : in_lod["renderers"]) {
        lod.renderers.emplace_back();
        lod.renderers.back().Deserialize(in_renderer, scene);
      }
    }
  }
}

void SerializeUnknownPayload(YAML::Emitter& out, const UnknownRuntimePayload& payload,
                             const std::vector<std::string>& skipped_keys) {
  const auto& node = payload.GetSerializedNode();
  if (!node || !node.IsMap()) {
    return;
  }
  for (const auto& item : node) {
    if (!item.first.IsScalar()) {
      continue;
    }
    const auto key = item.first.as<std::string>();
    if (std::find(skipped_keys.begin(), skipped_keys.end(), key) != skipped_keys.end()) {
      continue;
    }
    out << YAML::Key << key << YAML::Value << item.second;
  }
}

void DeserializeUnknownPayload(const YAML::Node& in, UnknownRuntimePayload& payload, const char* type_name_key) {
  if (in[type_name_key]) {
    payload.SetOriginalTypeName(in[type_name_key].as<std::string>());
  }
  payload.SetSerializedNode(in);
}

void SerializeUnknownPrivateComponent(YAML::Emitter& out, const UnknownPrivateComponent& component) {
  SerializeUnknownPayload(out, component, {"tn", "e"});
}

void DeserializeUnknownPrivateComponent(const YAML::Node& in, UnknownPrivateComponent& component) {
  DeserializeUnknownPayload(in, component, "tn");
}

void SerializeUnknownAsset(YAML::Emitter& out, const UnknownAsset& asset) {
  SerializeUnknownPayload(out, asset, {"type_name", "handle"});
}

void DeserializeUnknownAsset(const YAML::Node& in, UnknownAsset& asset) {
  DeserializeUnknownPayload(in, asset, "type_name");
}

void SerializeUnknownSystem(YAML::Emitter& out, const UnknownSystem& system) {
  SerializeUnknownPayload(out, system, {"type_name", "enabled_", "rank_", "handle_"});
}

void DeserializeUnknownSystem(const YAML::Node& in, UnknownSystem& system) {
  DeserializeUnknownPayload(in, system, "type_name");
}

void SerializeAssetRef(YAML::Emitter& out, const AssetRef& asset_ref) {
  asset_ref.Serialize(out);
}

void DeserializeAssetRef(const YAML::Node& in, AssetRef& asset_ref) {
  asset_ref.Deserialize(in);
}

void SerializePrivateComponentRef(YAML::Emitter& out, const PrivateComponentRef& private_component_ref) {
  private_component_ref.Serialize(out);
}

void DeserializePrivateComponentRef(const YAML::Node& in, PrivateComponentRef& private_component_ref) {
  private_component_ref.Deserialize(in);
}

void RegisterReferenceSerializationHandlers() {
  Serialization::RegisterSerializationHandler<AssetRef>(SerializeAssetRef, DeserializeAssetRef, {}, "AssetRef");
  Serialization::RegisterSerializationHandler<PrivateComponentRef>(
      SerializePrivateComponentRef, DeserializePrivateComponentRef, {}, "PrivateComponentRef");
}

void RegisterBuiltInSerializationHandlers() {
  Serialization::RegisterSerializationHandler<AnimationPlayer>(SerializeAnimationPlayer, DeserializeAnimationPlayer, {},
                                                               "AnimationPlayer");
  Serialization::RegisterSerializationHandler<Camera>(SerializeCamera, DeserializeCamera, {}, "Camera");
  Serialization::RegisterSerializationHandler<Animator>(SerializeAnimator, DeserializeAnimator, {}, "Animator");
  Serialization::RegisterSerializationHandler<PostProcessingStack>(
      SerializePostProcessingStack, DeserializePostProcessingStack, {}, "PostProcessingStack");
  Serialization::RegisterSerializationHandler<Material>(SerializeMaterial, DeserializeMaterial, {}, "Material");
  Serialization::RegisterSerializationHandler<Shader>(SerializeShader, DeserializeShader, {}, "Shader");
  Serialization::RegisterSerializationHandler<procedural_noise::ProceduralNoise2D>(
      SerializeProceduralNoise<procedural_noise::ProceduralNoise2D>,
      DeserializeProceduralNoise<procedural_noise::ProceduralNoise2D>, {}, "ProceduralNoise2D");
  Serialization::RegisterSerializationHandler<procedural_noise::ProceduralNoise3D>(
      SerializeProceduralNoise<procedural_noise::ProceduralNoise3D>,
      DeserializeProceduralNoise<procedural_noise::ProceduralNoise3D>, {}, "ProceduralNoise3D");
  Serialization::RegisterSerializationHandler<procedural_noise::ProceduralNoise4D>(
      SerializeProceduralNoise<procedural_noise::ProceduralNoise4D>,
      DeserializeProceduralNoise<procedural_noise::ProceduralNoise4D>, {}, "ProceduralNoise4D");
  Serialization::RegisterSerializationHandler<PointCloud>(SerializePointCloud, DeserializePointCloud, {}, "PointCloud");
  Serialization::RegisterSerializationHandler<GaussianSplat>(SerializeGaussianSplat, DeserializeGaussianSplat, {},
                                                             "GaussianSplat");
  Serialization::RegisterSerializationHandler<Texture2D>(SerializeTexture2D, DeserializeTexture2D, {}, "Texture2D");
  Serialization::RegisterSerializationHandler<Animation>(SerializeAnimation, DeserializeAnimation, {}, "Animation");
  Serialization::RegisterSerializationHandler<ParticleInfoList>(SerializeParticleInfoList, DeserializeParticleInfoList,
                                                                {}, "ParticleInfoList");
  Serialization::RegisterSerializationHandler<Mesh>(SerializeMesh, DeserializeMesh, {}, "Mesh");
  Serialization::RegisterSerializationHandler<SkinnedMesh>(SerializeSkinnedMesh, DeserializeSkinnedMesh, {},
                                                           "SkinnedMesh");
  Serialization::RegisterSerializationHandler<Strands>(SerializeStrands, DeserializeStrands, {}, "Strands");
  Serialization::RegisterSerializationHandler<Prefab>(SerializePrefab, DeserializePrefab, {}, "Prefab");
  Serialization::RegisterSerializationHandler<Scene>(SerializeScene, DeserializeScene, {}, "Scene");
  Serialization::RegisterSerializationHandler<MeshRenderer>(SerializeMeshRenderer, DeserializeMeshRenderer, {},
                                                            "MeshRenderer");
  Serialization::RegisterSerializationHandler<StrandsRenderer>(SerializeStrandsRenderer, DeserializeStrandsRenderer, {},
                                                               "StrandsRenderer");
  Serialization::RegisterSerializationHandler<GaussianSplatRenderer>(
      SerializeGaussianSplatRenderer, DeserializeGaussianSplatRenderer, {}, "GaussianSplatRenderer");
  Serialization::RegisterSerializationHandler<SkinnedMeshRenderer>(
      SerializeSkinnedMeshRenderer, DeserializeSkinnedMeshRenderer, {}, "SkinnedMeshRenderer");
  Serialization::RegisterSerializationHandler<Particles>(SerializeParticles, DeserializeParticles, {}, "Particles");
  Serialization::RegisterSerializationHandler<SpotLight>(SerializeSpotLight, DeserializeSpotLight, {}, "SpotLight");
  Serialization::RegisterSerializationHandler<PointLight>(SerializePointLight, DeserializePointLight, {}, "PointLight");
  Serialization::RegisterSerializationHandler<DirectionalLight>(SerializeDirectionalLight, DeserializeDirectionalLight,
                                                                {}, "DirectionalLight");
  Serialization::RegisterSerializationHandler<DdgiVolume>(SerializeDdgiVolume, DeserializeDdgiVolume, {}, "DdgiVolume");
  Serialization::RegisterSerializationHandler<PointCloudScanner>(SerializePointCloudScanner,
                                                                 DeserializePointCloudScanner, {}, "PointCloudScanner");
  Serialization::RegisterSerializationHandler<PlayerController>(SerializePlayerController, DeserializePlayerController,
                                                                {}, "PlayerController");
  Serialization::RegisterSerializationHandler<LodGroup>(SerializeLodGroup, DeserializeLodGroup, {}, "LodGroup");
  Serialization::RegisterSerializationHandler<UnknownPrivateComponent>(
      SerializeUnknownPrivateComponent, DeserializeUnknownPrivateComponent, {}, "UnknownPrivateComponent");
  Serialization::RegisterSerializationHandler<UnknownAsset>(SerializeUnknownAsset, DeserializeUnknownAsset, {},
                                                            "UnknownAsset");
  Serialization::RegisterSerializationHandler<UnknownSystem>(SerializeUnknownSystem, DeserializeUnknownSystem, {},
                                                             "UnknownSystem");
}
}  // namespace

Application::Application()
    : asset_manager_(std::make_unique<AssetManager>()),
      console_(std::make_unique<Console>()),
      entities_(std::make_unique<Entities>()),
      file_manager_(std::make_unique<FileManager>()),
      geometry_storage_(std::make_unique<GeometryStorage>()),
      input_(std::make_unique<Input>()),
      jobs_(std::make_unique<Jobs>()),
      package_manager_(std::make_unique<PackageManager>()),
      platform_(std::make_unique<Platform>()),
      project_manager_(std::make_unique<ProjectManager>()),
      resources_(std::make_unique<Resources>()),
      texture_storage_(std::make_unique<TextureStorage>()),
      times_(std::make_unique<Times>()),
      transform_graph_(std::make_unique<TransformGraph>()) {
  ApplicationContext::Set(this);
  Profiler::GetInstance().RegisterThread("MainThread");
}

Application::~Application() {
  if (execution_status_ != ExecutionStatus::Uninitialized) {
    Terminate();
  }
  if (ApplicationContext::TryGet() == this) {
    ApplicationContext::Set(nullptr);
  }
}

Serialization& Application::GetSerialization() {
  return serialization_registry_;
}

const Serialization& Application::GetSerialization() const {
  return serialization_registry_;
}

AssetManager& Application::GetAssetManager() {
  return *asset_manager_;
}

Console& Application::GetConsole() {
  return *console_;
}

Entities& Application::GetEntities() {
  return *entities_;
}

FileManager& Application::GetFileManager() {
  return *file_manager_;
}

GeometryStorage& Application::GetGeometryStorage() {
  return *geometry_storage_;
}

Input& Application::GetInput() {
  return *input_;
}

Jobs& Application::GetJobs() {
  return *jobs_;
}

PackageManager& Application::GetPackageManager() {
  return *package_manager_;
}

Platform& Application::GetPlatform() {
  return *platform_;
}

ProjectManager& Application::GetProjectManager() {
  return *project_manager_;
}

Resources& Application::GetResources() {
  return *resources_;
}

TextureStorage& Application::GetTextureStorage() {
  return *texture_storage_;
}

Times& Application::GetTimes() {
  return *times_;
}

TransformGraph& Application::GetTransformGraph() {
  return *transform_graph_;
}

void Application::PreUpdateInternal() {
  ApplicationContextScope application_scope(*this);
  const ProfilerScope profile_scope("Application::PreUpdate", "Frame");
  auto& times = GetTimes();
  const auto now = std::chrono::system_clock::now();
  const std::chrono::duration<double> delta_time = now - times.last_update_time_;
  times.delta_time_ = delta_time.count();
  times.last_update_time_ = std::chrono::system_clock::now();
  if (this->execution_status_ == ExecutionStatus::Uninitialized) {
    EVOENGINE_ERROR("Application uninitialized!")
    return;
  }
  if (this->execution_status_ == ExecutionStatus::OnDestroy)
    return;

  this->execution_order = ExecutionOrder::PreUpdate;
  Input::PreUpdate();
  if (const auto render_layer = GetLayer<RenderLayer>()) {
    Platform::PreUpdate();
  }
  const auto asset_task_budget_start = std::chrono::steady_clock::now();
  const auto run_asset_tasks = [&]() {
    const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() -
                                                                               asset_task_budget_start);
    if (elapsed >= kMainThreadAssetTaskFrameBudget) {
      return size_t{0};
    }
    return AssetManager::ExecuteMainThreadAssetTasksWithinBudget(1, kMainThreadAssetTaskFrameBudget - elapsed);
  };
  run_asset_tasks();
  ProjectManager::PreUpdate();
  run_asset_tasks();
  TryStartPendingPlayerAutoplay();
  if (this->active_scene_) {
    const ProfilerScope scene_scope("Application::ScenePreUpdate", "Scene");
    TransformGraph::CalculateTransformGraphs(this->active_scene_);
    for (const auto& i : this->external_pre_update_functions_)
      i();
    if (this->execution_status_ == ExecutionStatus::Playing || this->execution_status_ == ExecutionStatus::Step) {
      this->active_scene_->Start();
    }
  }

  {
    const ProfilerScope layers_scope("Application::LayerPreUpdate", "Layer");
    for (size_t layer_index = 0; layer_index < this->layers_.size();) {
      const auto layer = this->layers_[layer_index];
      layer->PreUpdate();
      if (layer_index < this->layers_.size() && this->layers_[layer_index] == layer) {
        ++layer_index;
      }
    }
  }
  if (times.steps_ == 0) {
    times.last_fixed_update_time_ = std::chrono::system_clock::now();
    times.steps_ = 1;
  }
  const auto last_fixed_update_time = times.last_fixed_update_time_;
  std::chrono::duration<double> duration = std::chrono::system_clock::now() - last_fixed_update_time;
  size_t step = 1;
  while (duration.count() >= step * times.time_step_) {
    for (const auto& i : this->external_fixed_update_functions_)
      i();
    for (size_t layer_index = 0; layer_index < this->layers_.size();) {
      const auto layer = this->layers_[layer_index];
      layer->FixedUpdate();
      if (layer_index < this->layers_.size() && this->layers_[layer_index] == layer) {
        ++layer_index;
      }
    }
    if (this->execution_status_ == ExecutionStatus::Playing || this->execution_status_ == ExecutionStatus::Step) {
      this->active_scene_->FixedUpdate();
    }
    duration = std::chrono::system_clock::now() - last_fixed_update_time;
    step++;
    const auto current_time = std::chrono::system_clock::now();
    const std::chrono::duration<double> fixed_delta_time = current_time - times.last_fixed_update_time_;
    times.fixed_delta_time_ = fixed_delta_time.count();
    times.last_fixed_update_time_ = std::chrono::system_clock::now();
    if (step > 10) {
      EVOENGINE_WARNING("Fixed update timeout!")
    }
    break;
  }
}

void Application::UpdateInternal() {
  ApplicationContextScope application_scope(*this);
  const ProfilerScope profile_scope("Application::Update", "Frame");
  if (this->execution_status_ == ExecutionStatus::Uninitialized) {
    EVOENGINE_ERROR("Application uninitialized!")
    return;
  }
  if (this->execution_status_ == ExecutionStatus::OnDestroy)
    return;

  this->execution_order = ExecutionOrder::Update;
  if (this->active_scene_) {
    const ProfilerScope scene_scope("Application::SceneUpdate", "Scene");
    if (this->execution_status_ == ExecutionStatus::Playing || this->execution_status_ == ExecutionStatus::Step) {
      this->active_scene_->Update();
    }
  }

  {
    const ProfilerScope layers_scope("Application::LayerUpdate", "Layer");
    for (size_t layer_index = 0; layer_index < this->layers_.size();) {
      const auto layer = this->layers_[layer_index];
      layer->Update();
      if (layer_index < this->layers_.size() && this->layers_[layer_index] == layer) {
        ++layer_index;
      }
    }
  }
  for (const auto& i : this->external_update_functions_)
    i();

  if (const auto render_layer = GetLayer<RenderLayer>()) {
    const ProfilerScope render_scope("Application::PrepareRendering", "Render");
    render_layer->PrepareForRendering();
    render_layer->ClearAllEditorCameras();
    render_layer->ClearAllCameras();
  }
}

void Application::LateUpdateInternal() {
  ApplicationContextScope application_scope(*this);
  const ProfilerScope profile_scope("Application::LateUpdate", "Frame");
  if (this->execution_status_ == ExecutionStatus::Uninitialized) {
    EVOENGINE_ERROR("Application uninitialized!")
    return;
  }
  if (this->execution_status_ == ExecutionStatus::OnDestroy)
    return;
  for (const auto& i : this->external_late_update_functions_)
    i();
  for (size_t layer_index = this->layers_.size(); layer_index > 0;) {
    --layer_index;
    if (layer_index >= this->layers_.size()) {
      continue;
    }
    this->layers_[layer_index]->LateUpdate();
  }

  const auto render_layer = GetLayer<RenderLayer>();
  const auto editor_layer = GetLayer<EditorLayer>();
  const auto window_layer = GetLayer<WindowLayer>();

  if (this->active_scene_) {
    this->execution_order = ExecutionOrder::LateUpdate;

    if (this->execution_status_ == ExecutionStatus::Playing || this->execution_status_ == ExecutionStatus::Step) {
      const ProfilerScope scene_scope("Application::SceneLateUpdate", "Scene");
      this->active_scene_->LateUpdate();
    }

    if (render_layer) {
      const ProfilerScope render_scope("Application::RenderSubmission", "Render");
      render_layer->RenderAll();
      render_layer->RenderGizmos();
    }
  }

  if (window_layer) {
    const ProfilerScope window_scope("Application::WindowRender", "Render");
    window_layer->Render();
  }
  if (render_layer) {
    Platform::LateUpdate();
  }
  if (this->execution_status_ == ExecutionStatus::Step)
    this->execution_status_ = ExecutionStatus::Pause;
}

const ApplicationInitializationSettings& Application::GetApplicationInfo() const {
  return this->initialization_settings;
}

const Application::ExecutionStatus& Application::GetApplicationStatus() const {
  return this->execution_status_;
}

std::shared_ptr<Scene> Application::GetActiveScene() const {
  return this->active_scene_;
}

void Application::Reset() {
  ApplicationContextScope application_scope(*this);
  this->execution_status_ = ExecutionStatus::NotPlaying;
  auto& times = GetTimes();
  times.steps_ = times.frames_ = 0;
}

void Application::Initialize(const ApplicationInitializationSettings& application_create_info) {
  ApplicationContextScope application_scope(*this);
#pragma region Reflection
  RegisterDataComponent<Transform>("Transform");
  RegisterDataComponent<GlobalTransform>("GlobalTransform");
  RegisterDataComponent<TransformUpdateFlag>("TransformUpdateFlag");
  RegisterDataComponent<Ray>("Ray");
  RegisterDataComponent<UnknownDataComponent>("UnknownDataComponent");

  RegisterReferenceSerializationHandlers();

  RegisterPrivateComponent<Camera>("Camera");
  RegisterPrivateComponent<AnimationPlayer>("AnimationPlayer");
  RegisterPrivateComponent<PlayerController>("PlayerController");
  RegisterPrivateComponent<Particles>("Particles");
  RegisterPrivateComponent<MeshRenderer>("MeshRenderer");
  RegisterPrivateComponent<StrandsRenderer>("StrandsRenderer");
  RegisterPrivateComponent<GaussianSplatRenderer>("GaussianSplatRenderer");
  RegisterPrivateComponent<SkinnedMeshRenderer>("SkinnedMeshRenderer");
  RegisterPrivateComponent<Animator>("Animator");
  RegisterPrivateComponent<PointLight>("PointLight");
  RegisterPrivateComponent<SpotLight>("SpotLight");
  RegisterPrivateComponent<DirectionalLight>("DirectionalLight");
  RegisterPrivateComponent<DdgiVolume>("DdgiVolume");
  RegisterPrivateComponent<WayPoints>("WayPoints");
  RegisterWayPointsHandlers();
  RegisterPrivateComponent<LodGroup>("LodGroup");
  RegisterPrivateComponent<PointCloudScanner>("PointCloudScanner");
  RegisterPrivateComponent<UnknownPrivateComponent>("UnknownPrivateComponent");
  RegisterSystem<UnknownSystem>("UnknownSystem");
  RegisterBuiltInSerializationHandlers();

  RegisterAsset<PostProcessingStack>("PostProcessingStack", {".evepostprocessingstack"});
  RegisterAsset<IAsset>("IAsset", {".eveasset"});
  RegisterAsset<UnknownAsset>("UnknownAsset", {".eveunknownasset"});
  RegisterAsset<Material>("Material", {".evematerial"});
  RegisterAsset<procedural_noise::ProceduralNoise2D>("ProceduralNoise2D", {".evenoise2d"});
  RegisterAsset<procedural_noise::ProceduralNoise3D>("ProceduralNoise3D", {".evenoise3d"});
  RegisterAsset<procedural_noise::ProceduralNoise4D>("ProceduralNoise4D", {".evenoise4d"});
  RegisterAsset<Cubemap>("Cubemap", {".evecubemap"});
  RegisterAsset<LightProbe>("LightProbe", {".evelightprobe"});
  RegisterAsset<ReflectionProbe>("ReflectionProbe", {".evereflectionprobe"});
  RegisterAsset<EnvironmentalMap>("EnvironmentalMap", {".eveenvironmentalmap"});
  RegisterAsset<Shader>(
      "Shader", {".eveshader", ".glsl", ".vert", ".frag", ".comp", ".geom", ".task", ".mesh", ".tesc", ".tese"});
  RegisterAsset<Mesh>("Mesh", {".evemesh"});
  RegisterAsset<Strands>("Strands", {".evestrands", ".hair"});
  RegisterAsset<Prefab>(
      "Prefab", {".eveprefab", ".obj", ".gltf", ".glb", ".blend", ".ply", ".fbx", ".dae", ".x3d", ".OBJ", ".FBX"});
  RegisterAsset<Texture2D>("Texture2D",
                           {".evetexture2d", ".png", ".jpg", ".jpeg", ".tga", ".hdr", ".TGA", ".PNG", ".JPG"});
  RegisterAsset<Scene>("Scene", {".evescene"});
  RegisterAsset<ParticleInfoList>("ParticleInfoList", {".eveparticleinfolist"});
  RegisterAsset<Animation>("Animation", {".eveanimation"});
  RegisterAsset<SkinnedMesh>("SkinnedMesh", {".eveskinnedmesh"});
  RegisterAsset<PointCloud>("PointCloud", {".evepointcloud"});
  RegisterAsset<GaussianSplat>("GaussianSplat", {".evegaussiansplat"});
  RegisterAsset<Json>("Json", {".json"});
  RegisterSdkInspectionAdapters();
  RegisterBuiltInAssetIoHandlers();
  RegisterJsonHandlers();
#pragma endregion

  if (this->execution_status_ != ExecutionStatus::Uninitialized) {
    EVOENGINE_ERROR("Application is not uninitialzed!")
    return;
  }
  this->initialization_settings = application_create_info;
  ConfigureConsoleWindow(this->initialization_settings.hide_console_window);
  if (this->initialization_settings.redirect_standard_streams_to_console) {
    console_->InstallStandardStreamRedirectors();
  } else {
    console_->RestoreStandardStreamRedirectors();
  }
  const auto render_layer = GetLayer<RenderLayer>();
  const auto window_layer = GetLayer<WindowLayer>();
  if (!this->initialization_settings.project_path.empty()) {
    if (this->initialization_settings.project_path.extension().string() != ".eveproj") {
      EVOENGINE_ERROR("Project file extension is not eveproj!")
      return;
    }
  } else if (!this->initialization_settings.allow_empty_project) {
    EVOENGINE_ERROR("Project filepath must be present unless empty project startup is explicitly allowed!")
    return;
  }
  MergeProjectLaunchMetadata(this->initialization_settings);
  const auto hardware_thread_size = std::thread::hardware_concurrency();
  const size_t default_thread_size = hardware_thread_size > 2 ? hardware_thread_size - 2 : 1;
  for (const auto& layer : this->layers_) {
    layer->RegisterTypes(*this);
  }
  Jobs::Initialize(default_thread_size);
  Entities::Initialize();
  TransformGraph::Initialize();
  AssetManager::Initialize();
  FileManager::Initialize();
  ProjectManager::Initialize();
  if (render_layer) {
    Platform::Initialize(this->initialization_settings);
  }
  if (this->initialization_settings.load_default_resources) {
    Resources::Initialize();
  }
  if (this->initialization_settings.enable_runtime_packages) {
    PackageManager::Initialize(this->initialization_settings.package_search_paths,
                               this->initialization_settings.startup_runtime_packages);
  }
  for (const auto& layer : this->layers_) {
    layer->OnCreate();
  }
  if (window_layer) {
    window_layer->ResizeWindow(this->initialization_settings.default_window_size.x,
                               this->initialization_settings.default_window_size.y);
    if (!this->initialization_settings.full_screen) {
      window_layer->CenterWindow();
    }
    // Texture loading flips STB globally; GLFW window icons need image-space orientation.
    stbi_set_flip_vertically_on_load(false);
    if (this->initialization_settings.icon_paths.empty()) {
      GLFWimage images[4];
      images[0].pixels = stbi_load(Resources::GetDefaultResourcePath("Icons/EvoEngine16.png").string().c_str(),
                                   &images[0].width, &images[0].height, nullptr, 4);  // rgba channels
      images[1].pixels = stbi_load(Resources::GetDefaultResourcePath("Icons/EvoEngine24.png").string().c_str(),
                                   &images[1].width, &images[1].height, nullptr, 4);  // rgba channels
      images[2].pixels = stbi_load(Resources::GetDefaultResourcePath("Icons/EvoEngine32.png").string().c_str(),
                                   &images[2].width, &images[2].height, nullptr, 4);  // rgba channels
      images[3].pixels = stbi_load(Resources::GetDefaultResourcePath("Icons/EvoEngine64.png").string().c_str(),
                                   &images[3].width, &images[3].height, nullptr, 4);  // rgba channels
      glfwSetWindowIcon(window_layer->window_, 4, images);
      stbi_image_free(images[0].pixels);
      stbi_image_free(images[1].pixels);
      stbi_image_free(images[2].pixels);
      stbi_image_free(images[3].pixels);
    } else {
      std::vector<GLFWimage> images;
      for (const auto& i : this->initialization_settings.icon_paths) {
        if (std::filesystem::exists(i)) {
          auto& image = images.emplace_back();
          image.pixels = stbi_load(std::filesystem::absolute(i).string().c_str(), &image.width, &image.height, nullptr,
                                   4);  // rgba channels
        }
      }
      glfwSetWindowIcon(window_layer->window_, images.size(), images.data());
      for (const auto& i : images) {
        stbi_image_free(i.pixels);
      }
    }
    window_layer->ShowWindow();
  }
  this->execution_status_ = ExecutionStatus::NotPlaying;

  if (!this->initialization_settings.project_path.empty()) {
    ProjectManager::GetOrCreateProject(this->initialization_settings.project_path);
  }
}

void Application::Start(const bool autoplay) {
  ApplicationContextScope application_scope(*this);
  auto& times = GetTimes();
  times.start_time_ = std::chrono::system_clock::now();
  times.steps_ = times.frames_ = 0;
  const bool runtime_autoplay_mode = initialization_settings.application_mode == ApplicationMode::Player ||
                                     initialization_settings.application_mode == ApplicationMode::Headless;
  pending_player_autoplay_ = autoplay && runtime_autoplay_mode && !GetLayer<EditorLayer>();
  TryStartPendingPlayerAutoplay();
}

void Application::Run() {
  while (Loop()) {
  }
}

bool Application::Loop() {
  const ApplicationContextScope application_scope(*this);
  if (this->execution_status_ != ExecutionStatus::OnDestroy) {
    const ProfilerFrameScope profiler_frame_scope;
    const ProfilerScope profiler_scope("Application::Loop", "Frame");
    PreUpdateInternal();
    UpdateInternal();
    LateUpdateInternal();
    ExecuteEndOfLoopActions();
    return true;
  }
  return false;
}

void Application::End() {
  ApplicationContextScope application_scope(*this);
  this->execution_status_ = ExecutionStatus::OnDestroy;
}

void Application::ExecuteEndOfLoopActions() {
  Jobs::ExecuteMainThreadJobs();
  if (this->end_of_loop_actions_.empty()) {
    return;
  }
  auto actions = std::move(this->end_of_loop_actions_);
  this->end_of_loop_actions_.clear();
  for (const auto& action : actions) {
    if (action) {
      action();
    }
  }
  Jobs::ExecuteMainThreadJobs();
}

void Application::Terminate() {
  ApplicationContextScope application_scope(*this);
  this->execution_status_ = ExecutionStatus::OnDestroy;
  pending_player_autoplay_ = false;
  const bool has_render_layer = GetLayer<RenderLayer>() != nullptr;
  for (auto i = this->layers_.rbegin(); i != this->layers_.rend(); ++i) {
    (*i)->OnDestroy();
  }
  this->layers_.clear();
  AssetManager::OnDestroy();
  ProjectManager::OnDestroy();
  FileManager::OnDestroy();
  Resources::OnDestroy();
  this->active_scene_.reset();
  TextureStorage::OnDestroy();
  GeometryStorage::OnDestroy();

  if (has_render_layer) {
    Platform::OnDestroy();
  }

  Jobs::OnDestroy();
  PackageManager::UnloadAll();
  Serialization::OnDestroy();

  this->execution_status_ = ExecutionStatus::Uninitialized;
}

const std::vector<std::shared_ptr<ILayer>>& Application::GetLayers() const {
  return this->layers_;
}

bool Application::RemoveLayersOwnedByPackage(const std::string& package_name) {
  ApplicationContextScope application_scope(*this);
  bool success = true;
  for (size_t layer_index = 0; layer_index < this->layers_.size();) {
    auto layer = this->layers_[layer_index];
    if (layer->package_owner_ != package_name) {
      ++layer_index;
      continue;
    }

    if (layer.use_count() > 2) {
      EVOENGINE_WARNING("Cannot unload runtime package because package layer is still in use: " + layer->layer_name_ +
                        " (" + package_name + ")")
      success = false;
      ++layer_index;
      continue;
    }

    layer->OnDestroy();
    layer->scene_.reset();
    layer->subsequent_layer_.reset();
    layer->self_.reset();
    layer->application_ = nullptr;
    layer->package_owner_.clear();
    this->layers_.erase(this->layers_.begin() + layer_index);
    if (layer.use_count() > 1) {
      EVOENGINE_WARNING("Cannot unload runtime package because package layer survived destruction: " +
                        layer->layer_name_ + " (" + package_name + ")")
      success = false;
    }
  }

  for (size_t layer_index = 0; layer_index < this->layers_.size(); ++layer_index) {
    this->layers_[layer_index]->subsequent_layer_.reset();
    if (layer_index + 1 < this->layers_.size()) {
      this->layers_[layer_index]->subsequent_layer_ = this->layers_[layer_index + 1];
    }
  }
  return success;
}

void Application::Attach(const std::shared_ptr<Scene>& scene) {
  ApplicationContextScope application_scope(*this);
  const ProfilerScope profiler_scope("Application::AttachScene", "Scene Sync");
  if (this->execution_status_ == ExecutionStatus::Playing) {
    EVOENGINE_ERROR("Stop Application to attach scene")
  }

  this->active_scene_ = scene;
  for (auto& func : this->post_attach_scene_functions_) {
    func(scene);
  }
  for (const auto& layer : this->layers_) {
    layer->scene_ = scene;
  }
  TryStartPendingPlayerAutoplay();
}

void Application::TryStartPendingPlayerAutoplay() {
  if (!pending_player_autoplay_ || !active_scene_ || execution_status_ != ExecutionStatus::NotPlaying ||
      !ProjectManager::IsProjectIdle()) {
    return;
  }
  pending_player_autoplay_ = false;
  Play();
}

void Application::Play() {
  ApplicationContextScope application_scope(*this);
  if (!this->active_scene_ || this->execution_status_ == ExecutionStatus::OnDestroy)
    return;
  if (this->execution_status_ != ExecutionStatus::Pause && this->execution_status_ != ExecutionStatus::NotPlaying)
    return;
  if (this->execution_status_ == ExecutionStatus::NotPlaying) {
    const auto copied_scene = AssetManager::CreateTemporaryAsset<Scene>();
    Scene::Clone(ProjectManager::GetStartScene().lock(), copied_scene);
    Attach(copied_scene);
  }
  this->execution_status_ = ExecutionStatus::Playing;
}
void Application::Stop() {
  ApplicationContextScope application_scope(*this);
  if (!this->active_scene_ || this->execution_status_ == ExecutionStatus::OnDestroy)
    return;
  if (this->execution_status_ == ExecutionStatus::NotPlaying)
    return;
  this->execution_status_ = ExecutionStatus::NotPlaying;
  Attach(ProjectManager::GetStartScene().lock());
}
void Application::Pause() {
  ApplicationContextScope application_scope(*this);
  if (!this->active_scene_ || this->execution_status_ == ExecutionStatus::OnDestroy)
    return;
  if (this->execution_status_ != ExecutionStatus::Playing)
    return;
  this->execution_status_ = ExecutionStatus::Pause;
}

void Application::Step() {
  ApplicationContextScope application_scope(*this);
  if (this->execution_status_ != ExecutionStatus::Pause && this->execution_status_ != ExecutionStatus::NotPlaying)
    return;
  if (this->execution_status_ == ExecutionStatus::NotPlaying) {
    const auto copied_scene = AssetManager::CreateTemporaryAsset<Scene>();
    Scene::Clone(ProjectManager::GetStartScene().lock(), copied_scene);
    Attach(copied_scene);
  }
  this->execution_status_ = ExecutionStatus::Step;
}

Application::ExecutionOrder Application::GetApplicationExecutionStatus() const {
  return this->execution_order;
}

void Application::RegisterPreUpdateFunction(const std::function<void()>& func) {
  ApplicationContextScope application_scope(*this);
  this->external_pre_update_functions_.push_back(func);
}

void Application::RegisterUpdateFunction(const std::function<void()>& func) {
  ApplicationContextScope application_scope(*this);
  this->external_update_functions_.push_back(func);
}

void Application::RegisterLateUpdateFunction(const std::function<void()>& func) {
  ApplicationContextScope application_scope(*this);
  this->external_late_update_functions_.push_back(func);
}

void Application::QueueEndOfLoopAction(const std::function<void()>& func) {
  ApplicationContextScope application_scope(*this);
  if (func) {
    this->end_of_loop_actions_.push_back(func);
  }
}

void Application::RegisterFixedUpdateFunction(const std::function<void()>& func) {
  ApplicationContextScope application_scope(*this);
  this->external_fixed_update_functions_.push_back(func);
}

void Application::RegisterPostAttachSceneFunction(
    const std::function<void(const std::shared_ptr<Scene>& new_scene)>& func) {
  ApplicationContextScope application_scope(*this);
  this->post_attach_scene_functions_.push_back(func);
}

bool Application::IsPlaying() const {
  return this->execution_status_ == ExecutionStatus::Playing;
}
