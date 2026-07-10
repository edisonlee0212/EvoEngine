#include "DemoScene.hpp"

#include "AnimationPlayer.hpp"
#include "Animator.hpp"
#include "Application.hpp"
#include "DdgiVolume.hpp"
#include "EditorLayer.hpp"
#include "GaussianSplat.hpp"
#include "GaussianSplatRenderer.hpp"
#include "Lights.hpp"
#include "Material.hpp"
#include "Mesh.hpp"
#include "MeshRenderer.hpp"
#include "PathUtils.hpp"
#include "PlayerController.hpp"
#include "PostProcessingStack.hpp"
#include "Prefab.hpp"
#include "ProjectManager.hpp"
#include "RenderLayer.hpp"
#include "Resources.hpp"
#include "SkinnedMeshRenderer.hpp"
#include "Times.hpp"
#include "TransformGraph.hpp"

#include <cmath>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <unordered_set>

using namespace evo_engine;

namespace {
constexpr uint64_t kSpatialDragonGaussianSplatHandle = 9739484957885691067ull;
constexpr uint64_t kBicycleGaussianSplatHandle = 14453709846752502031ull;
// VK's benchmark preset 1 maps to the first imported INRIA camera because preset 0 is VK's default camera.
const glm::vec3 kBicycleDemoCamera0Position = glm::vec3(-3.0026817f, 1.4007727f, -2.2284005f);
const glm::vec3 kBicycleDemoCamera0Front = glm::vec3(0.7710113f, -0.08249339f, 0.6314558f);
const glm::vec3 kBicycleDemoCamera0ImageDown = glm::vec3(-0.03804422f, 0.9838366f, 0.17498063f);
// vk_gltf_renderer's Bistro benchmark uses --gltfCamera 0, which resolves to this zeux/niagara_bistro camera node.
const glm::vec3 kBistroReferenceCameraPosition =
    glm::vec3(-26.43077278137207f, 3.156161069869995f, 11.170342445373535f);
const glm::quat kBistroReferenceCameraNodeRotation =
    glm::normalize(glm::quat(0.5971105694770813f, 0.004824427422136068f, -0.8017998337745667f, -0.023510245606303215f));
const glm::vec3 kBistroReferenceCameraFront =
    glm::normalize(kBistroReferenceCameraNodeRotation * glm::vec3(0.0f, 0.0f, -1.0f));
const glm::quat kBistroReferenceCameraRotation =
    glm::quatLookAt(kBistroReferenceCameraFront, glm::vec3(0.0f, 1.0f, 0.0f));
constexpr float kBistroReferenceCameraYFov = 35.98339890412515f;
constexpr float kBistroReferenceCameraEvoEngineFov = kBistroReferenceCameraYFov * 2.0f;
constexpr float kBistroReferenceCameraNearDistance = 0.1f;
constexpr float kBistroReferenceCameraFarDistance = 1000.0f;
constexpr int kBistroReferencePathTraceMaxDepth = 5;
constexpr int kBistroDdgiMaxProbeCount = 4096;
constexpr int kBistroDdgiMaxAxisProbeCount = 32;
constexpr int kBistroDdgiTargetLongestAxisProbeCount = 24;
constexpr float kBistroDdgiMinProbeSpacing = 0.05f;
constexpr float kBistroDdgiBoundsPadding = 1.05f;
constexpr float kBistroDirectionalLightIntensity = 10.0f;
constexpr const char* kRenderingRegressionRootName = "M42 Rendering Regression Root";
constexpr const char* kBistroDdgiVolumeName = "DDGI Probe Volume";
constexpr const char* kBistroImportedSunLightName = "Sun directional light";

struct RenderingRegressionTemporalMotionState {
  std::weak_ptr<Scene> scene;
  Entity rigid_entity;
  Entity transparent_entity;
  Entity skinned_entity;
  uint64_t frame = 0;
  bool enabled = false;
};

std::shared_ptr<RenderingRegressionTemporalMotionState> rendering_regression_temporal_motion_state;
bool rendering_regression_temporal_motion_registered = false;

void RegisterRenderingRegressionTemporalMotionUpdate() {
  if (rendering_regression_temporal_motion_registered) {
    return;
  }
  rendering_regression_temporal_motion_registered = true;
  ApplicationContext::Get().RegisterUpdateFunction([] {
    const auto state = rendering_regression_temporal_motion_state;
    if (!state || !state->enabled) {
      return;
    }
    const auto scene = state->scene.lock();
    auto& application = ApplicationContext::Get();
    if (!scene || application.GetActiveScene() != scene) {
      return;
    }

    const float phase = static_cast<float>(state->frame % 240u) * (2.0f * glm::pi<float>() / 240.0f);
    ++state->frame;
    if (scene->IsEntityValid(state->rigid_entity)) {
      Transform transform;
      transform.SetValue(glm::vec3(glm::sin(phase) * 1.65f, -0.14f, -1.55f), glm::vec3(0.0f, phase * 1.5f, 0.0f),
                         glm::vec3(0.24f));
      scene->SetDataComponent(state->rigid_entity, transform);
    }
    if (scene->IsEntityValid(state->transparent_entity)) {
      Transform transform;
      transform.SetValue(glm::vec3(glm::cos(phase * 0.8f) * 1.35f, 0.42f, -1.15f), glm::vec3(phase * 0.7f, phase, 0.0f),
                         glm::vec3(0.28f));
      scene->SetDataComponent(state->transparent_entity, transform);
    }
    if (scene->IsEntityValid(state->skinned_entity) && scene->HasPrivateComponent<Animator>(state->skinned_entity)) {
      const auto animator = scene->GetOrSetPrivateComponent<Animator>(state->skinned_entity).lock();
      const auto animation = animator ? animator->GetAnimation() : nullptr;
      if (animation) {
        const auto animation_name = animator->GetCurrentAnimationName();
        const auto animation_length = animation->GetAnimationLength(animation_name);
        if (animation_length > 0.0f) {
          animator->Animate(glm::mod(static_cast<float>(state->frame), animation_length));
        }
      }
    }

    const glm::vec3 base_position(0.0f, 1.15f, 5.6f);
    const glm::vec3 camera_position =
        base_position +
        glm::vec3(glm::sin(phase * 0.5f) * 0.12f, glm::sin(phase) * 0.035f, glm::cos(phase * 0.5f) * 0.08f);
    const glm::vec3 camera_target(glm::sin(phase * 0.4f) * 0.08f, 0.35f, -2.4f);
    const auto camera_rotation =
        glm::quatLookAt(glm::normalize(camera_target - camera_position), glm::vec3(0.0f, 1.0f, 0.0f));
    if (const auto main_camera = scene->main_camera.Get<Camera>()) {
      Transform transform;
      transform.SetValue(camera_position, camera_rotation, glm::vec3(1.0f));
      scene->SetDataComponent(main_camera->GetOwner(), transform);
    }
    if (const auto editor_layer = application.GetLayer<EditorLayer>()) {
      editor_layer->SetSceneCameraPosition(camera_position);
      editor_layer->SetSceneCameraRotation(camera_rotation);
    }
  });
}

struct GaussianSplatDemoCameraPose {
  glm::vec3 position = glm::vec3(0.0f);
  glm::quat rotation = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
};

size_t CountPrefabRenderers(const std::shared_ptr<Prefab>& prefab) {
  if (!prefab) {
    return 0;
  }
  size_t count = 0;
  for (const auto& private_component : prefab->private_components) {
    if (std::dynamic_pointer_cast<MeshRenderer>(private_component.private_component) ||
        std::dynamic_pointer_cast<SkinnedMeshRenderer>(private_component.private_component)) {
      ++count;
    }
  }
  for (const auto& child : prefab->child_prefabs) {
    count += CountPrefabRenderers(child);
  }
  return count;
}

struct CameraFrame {
  glm::vec3 position = glm::vec3(0.0f);
  glm::quat rotation = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
  float fov = kBistroReferenceCameraEvoEngineFov;
  float near_distance = kBistroReferenceCameraNearDistance;
  float far_distance = kBistroReferenceCameraFarDistance;
};

struct BistroParityStats {
  size_t mesh_renderer_count = 0;
  size_t skinned_mesh_renderer_count = 0;
  size_t mesh_primitive_count = 0;
  size_t alpha_opaque_render_primitive_count = 0;
  size_t alpha_mask_render_primitive_count = 0;
  size_t alpha_blend_render_primitive_count = 0;
  size_t material_count = 0;
  size_t texture_count = 0;
  size_t triangle_count = 0;
  size_t directional_light_count = 0;
  size_t point_light_count = 0;
  size_t spot_light_count = 0;
};

CameraFrame CalculateBistroCameraFrame(const Bound& bound) {
  CameraFrame frame;
  const auto center = bound.Center();
  const auto half_extents = glm::max(bound.Size(), glm::vec3(0.5f));
  const auto radius = glm::max(glm::length(half_extents), 4.0f);
  const auto distance = radius / glm::tan(glm::radians(kBistroReferenceCameraYFov * 0.5f)) * 1.2f;
  frame.position = center + glm::vec3(0.0f, radius * 0.2f, distance);
  const auto front = glm::normalize(center - frame.position);
  frame.rotation = glm::quatLookAt(front, glm::vec3(0.0f, 1.0f, 0.0f));
  frame.far_distance = glm::max(kBistroReferenceCameraFarDistance, distance + radius * 3.0f);
  return frame;
}

void ConfigureBistroCameraPostProcessing(const std::shared_ptr<Camera>& camera);
Entity LoadRenderingScene(const std::shared_ptr<Scene>& scene, const std::string& base_entity_name, bool add_spheres);
std::optional<Entity> FindEntityNamed(const std::shared_ptr<Scene>& scene, const std::string& name);

void ConfigureMaterial(const std::shared_ptr<Material>& material, const glm::vec3& albedo, const float roughness = 1.0f,
                       const float metallic = 1.0f, const float emission = 0.0f, const float transmission = 0.0f) {
  auto& shade_material = material->material_data.shade_material;
  shade_material.pbr_base_color_factor = glm::vec4(albedo, 1.0f);
  shade_material.pbr_roughness_factor = roughness;
  shade_material.pbr_metallic_factor = metallic;
  shade_material.transmission_factor = transmission;
  const auto emissive_tint = glm::max(albedo, glm::vec3(0.0f));
  const auto emissive_length = glm::length(emissive_tint);
  shade_material.emissive_factor =
      emission > 0.0f && emissive_length > 0.0f ? emissive_tint / emissive_length * emission : glm::vec3(0.0f);
  material->MarkDirty();
}

Entity CreateRenderingRegressionProbe(const std::shared_ptr<Scene>& scene, const Entity& root, const std::string& name,
                                      const std::shared_ptr<Mesh>& mesh, const glm::vec3& position,
                                      const glm::vec3& scale, const glm::vec3& albedo, const float roughness,
                                      const float metallic, const float emission = 0.0f, const bool cast_shadow = true,
                                      const float transmission = 0.0f) {
  const auto entity = scene->CreateEntity(name);
  const auto renderer = scene->GetOrSetPrivateComponent<MeshRenderer>(entity).lock();
  renderer->mesh = mesh;
  renderer->material = AssetManager::CreateTemporaryAsset<Material>();
  renderer->cast_shadow = cast_shadow;
  ConfigureMaterial(renderer->material.Get<Material>(), albedo, roughness, metallic, emission, transmission);
  Transform transform;
  transform.SetValue(position, glm::vec3(0.0f), scale);
  scene->SetDataComponent(entity, transform);
  scene->SetParent(entity, root);
  return entity;
}

void ConfigureRenderingRegressionCamera(const std::shared_ptr<Scene>& scene) {
  const glm::vec3 camera_position(0.0f, 1.15f, 5.6f);
  const glm::vec3 camera_target(0.0f, 0.35f, -2.4f);
  const auto camera_rotation = glm::quatLookAt(glm::normalize(camera_target - camera_position), glm::vec3(0, 1, 0));
  if (const auto main_camera = scene->main_camera.Get<Camera>()) {
    main_camera->Resize({1920, 1080});
    main_camera->camera_render_mode = Camera::CameraRenderMode::Rasterization;
    main_camera->camera_settings.fov = 55.0f;
    main_camera->camera_settings.near_distance = 0.05f;
    main_camera->camera_settings.far_distance = 250.0f;
    main_camera->camera_settings.sample_size = 4;
    main_camera->camera_settings.bounce = 5;
    main_camera->camera_settings.firefly_clamp_enabled = true;
    main_camera->camera_settings.firefly_clamp_threshold = 10.0f;
    main_camera->camera_settings.auto_spp_enabled = false;
    main_camera->camera_settings.auto_spp_min_samples = 4;
    main_camera->camera_settings.auto_spp_max_samples = 32;
    main_camera->camera_settings.auto_spp_convergence_threshold = 0.01f;
    main_camera->post_processing_stack_ref = AssetManager::CreateTemporaryAsset<PostProcessingStack>();
    ConfigureBistroCameraPostProcessing(main_camera);
    const auto main_camera_entity = main_camera->GetOwner();
    Transform main_camera_transform;
    main_camera_transform.SetValue(camera_position, camera_rotation, glm::vec3(1.0f));
    scene->SetDataComponent(main_camera_entity, main_camera_transform);
    scene->GetOrSetPrivateComponent<PlayerController>(main_camera_entity);
    main_camera->ResetFrameCount();
  }
  if (const auto editor_layer = ApplicationContext::Get().GetLayer<EditorLayer>()) {
    editor_layer->enable_gizmos = false;
    editor_layer->velocity = 4.0f;
    editor_layer->SetSelectedEntity({});
    editor_layer->SetSceneCameraPosition(camera_position);
    editor_layer->SetSceneCameraRotation(camera_rotation);
    if (const auto scene_camera = editor_layer->GetSceneCamera()) {
      scene_camera->camera_settings.fov = 55.0f;
      scene_camera->camera_settings.near_distance = 0.05f;
      scene_camera->camera_settings.far_distance = 250.0f;
      scene_camera->camera_settings.sample_size = 4;
      scene_camera->camera_settings.bounce = 5;
      scene_camera->camera_settings.firefly_clamp_enabled = true;
      scene_camera->camera_settings.firefly_clamp_threshold = 10.0f;
      scene_camera->camera_settings.auto_spp_enabled = false;
      scene_camera->camera_settings.auto_spp_min_samples = 4;
      scene_camera->camera_settings.auto_spp_max_samples = 32;
      scene_camera->camera_settings.auto_spp_convergence_threshold = 0.01f;
      ConfigureBistroCameraPostProcessing(scene_camera);
      scene_camera->ResetFrameCount();
    }
  }
}

void ConfigureRenderingRegressionLights(const std::shared_ptr<Scene>& scene, const Entity& root) {
  const auto& primitives = Resources::GetInstance().GetPrimitives();
  const auto directional_entity = scene->CreateEntity("M42 Punctual Light Probe Directional");
  const auto directional_light = scene->GetOrSetPrivateComponent<DirectionalLight>(directional_entity).lock();
  directional_light->diffuse = glm::vec3(1.0f, 0.96f, 0.86f);
  directional_light->diffuse_brightness = 2.0f;
  directional_light->light_size = 0.025f;
  directional_light->bias = 0.02f;
  directional_light->normal_offset = 0.02f;
  Transform directional_transform;
  const auto directional_light_dir = glm::normalize(glm::vec3(0.35f, -0.8f, -0.48f));
  directional_transform.SetRotation(glm::quatLookAt(-directional_light_dir, glm::vec3(0, 1, 0)));
  scene->SetDataComponent(directional_entity, directional_transform);
  scene->SetParent(directional_entity, root);

  const auto point_entity = CreateRenderingRegressionProbe(
      scene, root, "M42 Punctual Light Probe Point", primitives.sphere, glm::vec3(-2.3f, 1.25f, -1.7f),
      glm::vec3(0.08f), glm::vec3(1.0f, 0.75f, 0.25f), 0.7f, 0.0f, 8.0f, false);
  const auto point_light = scene->GetOrSetPrivateComponent<PointLight>(point_entity).lock();
  point_light->diffuse = glm::vec3(1.0f, 0.72f, 0.25f);
  point_light->diffuse_brightness = 18.0f;
  point_light->range = 7.0f;
  point_light->light_size = 0.025f;
  point_light->constant = 1.0f;
  point_light->linear = 0.18f;
  point_light->quadratic = 0.045f;

  const auto spot_entity = CreateRenderingRegressionProbe(scene, root, "M42 Punctual Light Probe Spot", primitives.cone,
                                                          glm::vec3(2.3f, 1.7f, -0.45f), glm::vec3(0.18f),
                                                          glm::vec3(0.35f, 0.62f, 1.0f), 0.8f, 0.0f, 3.0f, false);
  const auto spot_light = scene->GetOrSetPrivateComponent<SpotLight>(spot_entity).lock();
  spot_light->diffuse = glm::vec3(0.45f, 0.65f, 1.0f);
  spot_light->diffuse_brightness = 16.0f;
  spot_light->inner_degrees = 14.0f;
  spot_light->outer_degrees = 28.0f;
  spot_light->range = 8.0f;
  spot_light->light_size = 0.02f;
  Transform spot_transform = scene->GetDataComponent<Transform>(spot_entity);
  const auto spot_target = glm::vec3(0.4f, -0.05f, -2.8f);
  spot_transform.SetRotation(
      glm::quatLookAt(glm::normalize(spot_target - spot_transform.GetPosition()), glm::vec3(0, 1, 0)));
  scene->SetDataComponent(spot_entity, spot_transform);
}

void ConfigureRenderingRegressionImportedProbes(const std::shared_ptr<Scene>& scene, const Entity& root) {
  const auto imported_root = LoadRenderingScene(scene, "M42 Imported Material Texture Probe", false);
  scene->SetParent(imported_root, root);
  Transform imported_transform;
  imported_transform.SetValue(glm::vec3(8.0f, -1.0f, -8.0f), glm::vec3(0.0f), glm::vec3(0.08f));
  scene->SetDataComponent(imported_root, imported_transform);
  if (const auto capoeira_entity = FindEntityNamed(scene, "Capoeira")) {
    scene->SetEntityName(*capoeira_entity, "M42 Skinned Capoeira Probe");
    scene->SetEnable(*capoeira_entity, true);
    scene->SetParent(*capoeira_entity, root);
    Transform capoeira_transform;
    capoeira_transform.SetValue(glm::vec3(-1.35f, -0.25f, -2.2f), glm::radians(glm::vec3(0.0f, 25.0f, 0.0f)),
                                glm::vec3(0.018f));
    scene->SetDataComponent(*capoeira_entity, capoeira_transform);
  }
}

Transform CalculateBistroRootTransformForCameraFrame(const CameraFrame& frame) {
  const auto root_rotation = glm::normalize(frame.rotation * glm::inverse(kBistroReferenceCameraRotation));
  Transform transform;
  transform.SetValue(frame.position - root_rotation * kBistroReferenceCameraPosition, root_rotation, glm::vec3(1.0f));
  return transform;
}

void AccumulateBistroMaterialStats(const std::shared_ptr<Material>& material, std::unordered_set<Handle>& materials,
                                   std::unordered_set<Handle>& textures) {
  if (!material) {
    return;
  }
  materials.insert(material->GetHandle());
  for (const auto& texture_ref : material->PeekTextureRefs()) {
    const auto texture_handle = texture_ref.GetAssetHandle();
    if (texture_handle.GetValue() != 0u) {
      textures.insert(texture_handle);
    }
  }
}

void AccumulateBistroAlphaModeStats(const std::shared_ptr<Material>& material, BistroParityStats& stats) {
  if (!material) {
    return;
  }
  const auto alpha_mode = material->material_data.shade_material.alpha_mode;
  if (alpha_mode == static_cast<int32_t>(GltfAlphaMode::Mask)) {
    ++stats.alpha_mask_render_primitive_count;
  } else if (alpha_mode == static_cast<int32_t>(GltfAlphaMode::Blend)) {
    ++stats.alpha_blend_render_primitive_count;
  } else {
    ++stats.alpha_opaque_render_primitive_count;
  }
}

void AccumulateBistroPrefabStats(const std::shared_ptr<Prefab>& prefab, BistroParityStats& stats,
                                 std::unordered_set<Handle>& materials, std::unordered_set<Handle>& textures,
                                 std::unordered_set<Handle>& meshes) {
  if (!prefab || !prefab->IsPrefabEnabled()) {
    return;
  }
  for (const auto& private_component : prefab->private_components) {
    if (!private_component.enabled) {
      continue;
    }
    if (const auto mesh_renderer = std::dynamic_pointer_cast<MeshRenderer>(private_component.private_component)) {
      ++stats.mesh_renderer_count;
      ++stats.mesh_primitive_count;
      const auto material = mesh_renderer->material.Get<Material>();
      if (const auto mesh = mesh_renderer->mesh.Get<Mesh>()) {
        if (meshes.insert(mesh->GetHandle()).second) {
          stats.triangle_count += mesh->UnsafeGetTriangles().size();
        }
      }
      AccumulateBistroAlphaModeStats(material, stats);
      AccumulateBistroMaterialStats(material, materials, textures);
    } else if (const auto skinned_mesh_renderer =
                   std::dynamic_pointer_cast<SkinnedMeshRenderer>(private_component.private_component)) {
      ++stats.skinned_mesh_renderer_count;
      ++stats.mesh_primitive_count;
      const auto material = skinned_mesh_renderer->material.Get<Material>();
      if (const auto skinned_mesh = skinned_mesh_renderer->skinned_mesh.Get<SkinnedMesh>()) {
        if (meshes.insert(skinned_mesh->GetHandle()).second) {
          stats.triangle_count += skinned_mesh->UnsafeGetTriangles().size();
        }
      }
      AccumulateBistroAlphaModeStats(material, stats);
      AccumulateBistroMaterialStats(material, materials, textures);
    }
  }
  for (const auto& child : prefab->child_prefabs) {
    AccumulateBistroPrefabStats(child, stats, materials, textures, meshes);
  }
}

template <typename T>
size_t CountEnabledBistroSceneComponents(const std::shared_ptr<Scene>& scene) {
  if (!scene) {
    return 0;
  }
  size_t count = 0;
  if (const auto* owners = scene->UnsafeGetPrivateComponentOwnersList<T>()) {
    for (const auto& owner : *owners) {
      const auto component = scene->GetOrSetPrivateComponent<T>(owner).lock();
      if (component && component->IsEnabled() && scene->IsEntityEnabled(owner)) {
        ++count;
      }
    }
  }
  return count;
}

BistroParityStats GatherBistroParityStats(const std::shared_ptr<Scene>& scene, const std::shared_ptr<Prefab>& bistro) {
  BistroParityStats stats;
  std::unordered_set<Handle> materials;
  std::unordered_set<Handle> textures;
  std::unordered_set<Handle> meshes;
  AccumulateBistroPrefabStats(bistro, stats, materials, textures, meshes);
  stats.material_count = materials.size();
  stats.texture_count = textures.size();
  stats.directional_light_count = CountEnabledBistroSceneComponents<DirectionalLight>(scene);
  stats.point_light_count = CountEnabledBistroSceneComponents<PointLight>(scene);
  stats.spot_light_count = CountEnabledBistroSceneComponents<SpotLight>(scene);
  return stats;
}

void LogBistroRasterAlphaPolicy(const BistroParityStats& stats) {
  std::ostringstream stream;
  stream << "Bistro raster alpha policy: alpha_opaque_render_primitives=" << stats.alpha_opaque_render_primitive_count
         << ", alpha_mask_render_primitives=" << stats.alpha_mask_render_primitive_count
         << ", alpha_blend_render_primitives=" << stats.alpha_blend_render_primitive_count
         << ", mask_policy=deferred_gbuffer_alpha_cutoff"
         << ", blend_policy=sorted_back_to_front_transparent_pass"
         << ", reference_policy=sorted_back_to_front_blend_pass";
  EVOENGINE_LOG(stream.str())
}

const char* BistroEnvironmentTypeName(const Scene::EnvironmentType environment_type) {
  switch (environment_type) {
    case Scene::EnvironmentType::EnvironmentalMap:
      return "EnvironmentalMap";
    case Scene::EnvironmentType::Color:
      return "Color";
  }
  return "Unknown";
}

void ApplyBistroParityRendererState(const std::shared_ptr<Scene>& scene) {
  if (!scene) {
    return;
  }
  scene->environment.ddgi_settings.runtime.enabled = false;
  scene->environment.ddgi_settings.debug.enabled = false;
  scene->environment.volumetric_cloud_settings.enabled = false;
}

void ConfigureBistroCameraPostProcessing(const std::shared_ptr<Camera>& camera) {
  if (!camera) {
    return;
  }
  auto post_processing_stack = camera->post_processing_stack_ref.Get<PostProcessingStack>();
  if (!post_processing_stack) {
    post_processing_stack = AssetManager::CreateTemporaryAsset<PostProcessingStack>();
    camera->post_processing_stack_ref = post_processing_stack;
  }
  post_processing_stack->enable_bloom = false;
  post_processing_stack->enable_screen_space_reflection = false;
}

void ConfigureBistroReferenceToneMapping(const std::shared_ptr<Camera>& camera) {
  ConfigureBistroCameraPostProcessing(camera);
  const auto post_processing_stack = camera->post_processing_stack_ref.Get<PostProcessingStack>();
  if (!post_processing_stack) {
    return;
  }
  post_processing_stack->enable_tone_mapping = true;
  if (!post_processing_stack->tone_mapping) {
    post_processing_stack->tone_mapping = std::make_shared<ToneMapping>();
  }
  auto& tone_mapping = *post_processing_stack->tone_mapping;
  tone_mapping.method = ToneMapping::ToneMapMethod::Filmic;
  tone_mapping.exposure = 1.0f;
  tone_mapping.brightness = 1.0f;
  tone_mapping.contrast = 1.0f;
  tone_mapping.saturation = 1.0f;
  tone_mapping.vignette = 0.0f;
  tone_mapping.auto_exposure = true;
  tone_mapping.auto_exposure_speed = 5.0f;
  tone_mapping.ev_min_value = -5.0f;
  tone_mapping.ev_max_value = 10.0f;
  tone_mapping.enable_center_metering = false;
  tone_mapping.center_metering_size = 0.5f;
  tone_mapping.average_mode = 1;
  tone_mapping.dither = true;
}

void DisableImportedLightsRecursive(const std::shared_ptr<Scene>& scene, Entity entity);

Entity LoadRenderingScene(const std::shared_ptr<Scene>& scene, const std::string& base_entity_name, bool add_spheres) {
  auto base_entity = scene->CreateEntity(base_entity_name);

  if (add_spheres) {
    const int amount = 5;
    const auto collection = scene->CreateEntity("Spheres");
    const auto spheres = scene->CreateEntities(amount * amount * amount, "Instance");

    for (int i = 0; i < amount; i++) {
      for (int j = 0; j < amount; j++) {
        for (int k = 0; k < amount; k++) {
          constexpr float scale_factor = 0.03f;
          auto& sphere = spheres[i * amount * amount + j * amount + k];
          Transform transform;
          glm::vec3 position = glm::vec3(i + 0.5f - amount / 2.0f, j + 0.5f - amount / 2.0f, k + 0.5f - amount / 2.0f);
          position += glm::linearRand(glm::vec3(-0.5f), glm::vec3(0.5f)) * scale_factor;
          transform.SetPosition(position * 5.f * scale_factor);
          transform.SetScale(glm::vec3(4.0f * scale_factor));
          scene->SetDataComponent(sphere, transform);
          const auto mesh_renderer = scene->GetOrSetPrivateComponent<MeshRenderer>(sphere).lock();
          mesh_renderer->mesh = Resources::GetInstance().GetPrimitives().sphere;
          const auto material = AssetManager::CreateTemporaryAsset<Material>();
          mesh_renderer->material = material;
          ConfigureMaterial(material, glm::vec3(1.0f), static_cast<float>(i) / (amount - 1),
                            static_cast<float>(j) / (amount - 1));
          scene->SetParent(sphere, collection);
        }
      }
    }
    scene->SetParent(collection, base_entity);
    Transform physics_demo_transform;
    physics_demo_transform.SetPosition(glm::vec3(0.0f, 0.0f, -3.5f));
    physics_demo_transform.SetScale(glm::vec3(3.0f));
    scene->SetDataComponent(collection, physics_demo_transform);
  }

  const auto ground = scene->CreateEntity("Ground");
  const auto ground_mesh_renderer = scene->GetOrSetPrivateComponent<MeshRenderer>(ground).lock();
  const auto ground_mat = AssetManager::CreateTemporaryAsset<Material>();
  ground_mesh_renderer->material = ground_mat;
  ground_mesh_renderer->mesh = Resources::GetInstance().GetPrimitives().cube;
  Transform ground_transform;
  ground_transform.SetValue(glm::vec3(0, -2.05f, 0), glm::vec3(0), glm::vec3(30, 1, 60));
  scene->SetDataComponent(ground, ground_transform);
  scene->SetParent(ground, base_entity);

  const auto sponza =
      std::dynamic_pointer_cast<Prefab>(ProjectManager::GetOrCreateAsset("Models/Sponza_FBX/Sponza.fbx"));
  const auto sponza_entity = sponza->ToEntity(scene);
  DisableImportedLightsRecursive(scene, sponza_entity);
  Transform sponza_transform;
  sponza_transform.SetValue(glm::vec3(0, -1.5f, -6), glm::radians(glm::vec3(0, -90, 0)), glm::vec3(0.01f));
  scene->SetDataComponent(sponza_entity, sponza_transform);
  scene->SetParent(sponza_entity, base_entity);

  const auto title = std::dynamic_pointer_cast<Prefab>(ProjectManager::GetOrCreateAsset("Models/EvoEngine.obj"));
  const auto title_entity = title->ToEntity(scene);
  scene->SetEntityName(title_entity, "Title");
  Transform title_transform;
  title_transform.SetValue(glm::vec3(-1.4f, 6.9f, -16), glm::radians(glm::vec3(0, 0, 0)), glm::vec3(0.02f));
  scene->SetDataComponent(title_entity, title_transform);
  scene->SetParent(title_entity, base_entity);

  const auto title_material =
      scene->GetOrSetPrivateComponent<MeshRenderer>(scene->GetChildren(scene->GetChildren(title_entity)[0])[0])
          .lock()
          ->material.Get<Material>();
  ConfigureMaterial(title_material, glm::vec3(1, 0.2f, 0.5f), 1.0f, 1.0f, 4.0f);

  const auto capoeira = std::dynamic_pointer_cast<Prefab>(ProjectManager::GetOrCreateAsset("Models/Capoeira.fbx"));
  const auto capoeira_entity = capoeira->ToEntity(scene);
  const auto capoeira_animation_player = scene->GetOrSetPrivateComponent<AnimationPlayer>(capoeira_entity).lock();
  capoeira_animation_player->auto_play = true;
  capoeira_animation_player->auto_play_speed = 60;
  scene->SetEntityName(capoeira_entity, "Capoeira");
  Transform capoeira_transform;
  capoeira_transform.SetValue(glm::vec3(0.5f, 2.7f, -18), glm::vec3(0), glm::vec3(0.02f));
  scene->SetDataComponent(capoeira_entity, capoeira_transform);
  const auto capoeira_body_material =
      scene
          ->GetOrSetPrivateComponent<SkinnedMeshRenderer>(scene->GetChildren(scene->GetChildren(capoeira_entity)[1])[0])
          .lock()
          ->material.Get<Material>();
  ConfigureMaterial(capoeira_body_material, glm::vec3(0, 1, 1), 0.0f, 1.0f);
  const auto capoeira_joints_material =
      scene
          ->GetOrSetPrivateComponent<SkinnedMeshRenderer>(scene->GetChildren(scene->GetChildren(capoeira_entity)[0])[0])
          .lock()
          ->material.Get<Material>();
  ConfigureMaterial(capoeira_joints_material, glm::vec3(0.3f, 1.0f, 0.5f), 0.0f, 1.0f, 6.0f);
  scene->SetParent(capoeira_entity, base_entity);
  scene->SetEnable(capoeira_entity, false);

  return base_entity;
}

void ConfigureRenderingDemoDdgi(const std::shared_ptr<Scene>& scene) {
  auto& settings = scene->environment.ddgi_settings;
  settings.runtime.enabled = true;
  settings.runtime.pause_updates = false;
  settings.runtime.ray_count = 64;
  settings.runtime.normal_bias = 0.02f;
  settings.runtime.visibility_moment_bias = 0.02f;
  settings.storage.max_probe_count = 8192;
  settings.debug.enabled = true;
  settings.debug.visualize_volume_bounds = true;
  settings.debug.visualize_probe_positions = true;
  settings.debug.visualize_selected_probe = true;
  settings.debug.visualization_scale = 2.0f;

  const auto ddgi_volume_entity = scene->CreateEntity("DDGI Probe Volume");
  const auto ddgi_volume = scene->GetOrSetPrivateComponent<DdgiVolume>(ddgi_volume_entity).lock();
  ddgi_volume->probe_counts = {10, 6, 16};
  ddgi_volume->probe_spacing = glm::vec3(1.5f);
  ddgi_volume->volume_origin = {0.0f, 3.0f, 3.0f};
  ddgi_volume->relocation_distance = 0.25f;
  ddgi_volume->enable_probe_relocation = true;
  ddgi_volume->enable_probe_classification = false;
  ddgi_volume->visualize_bounds = true;
  ddgi_volume->visualize_probe_positions = true;
  ddgi_volume->max_visualized_probes = 8192;
  ddgi_volume->probe_visualization_size = 0.06f;
  ddgi_volume->ClampSettings();

  Transform ddgi_volume_transform;
  ddgi_volume_transform.SetPosition(glm::vec3(0.0f, 0.0f, -6.0f));
  scene->SetDataComponent(ddgi_volume_entity, ddgi_volume_transform);
}

std::shared_ptr<Material> CreateCornellMaterial(const glm::vec3& albedo, const float emission = 0.0f) {
  const auto material = AssetManager::CreateTemporaryAsset<Material>();
  ConfigureMaterial(material, albedo, 0.85f, 0.0f, emission);
  return material;
}

Entity CreateCornellBox(const std::shared_ptr<Scene>& scene, const Entity parent, const std::string& name,
                        const glm::vec3& position, const glm::vec3& scale, const std::shared_ptr<Material>& material) {
  const auto entity = scene->CreateEntity(name);
  const auto mesh_renderer = scene->GetOrSetPrivateComponent<MeshRenderer>(entity).lock();
  mesh_renderer->mesh = Resources::GetInstance().GetPrimitives().cube;
  mesh_renderer->material = material;
  Transform transform;
  transform.SetPosition(position);
  transform.SetScale(scale);
  scene->SetDataComponent(entity, transform);
  scene->SetParent(entity, parent);
  return entity;
}

template <typename LightComponent>
void DisableLightIfPresent(const std::shared_ptr<Scene>& scene, const Entity entity) {
  if (scene->HasPrivateComponent<LightComponent>(entity)) {
    if (const auto light = scene->GetOrSetPrivateComponent<LightComponent>(entity).lock()) {
      light->SetEnabled(false);
    }
  }
}

void DisableImportedLightsRecursive(const std::shared_ptr<Scene>& scene, const Entity entity) {
  if (!scene->IsEntityValid(entity)) {
    return;
  }
  DisableLightIfPresent<DirectionalLight>(scene, entity);
  DisableLightIfPresent<PointLight>(scene, entity);
  DisableLightIfPresent<SpotLight>(scene, entity);
  for (const auto child : scene->GetChildren(entity)) {
    DisableImportedLightsRecursive(scene, child);
  }
}

void ConfigureCornellBoxDdgi(const std::shared_ptr<Scene>& scene) {
  auto& settings = scene->environment.ddgi_settings;
  settings.runtime.enabled = true;
  settings.runtime.pause_updates = false;
  settings.runtime.ray_count = 256;
  settings.runtime.normal_bias = 0.02f;
  settings.runtime.visibility_moment_bias = 0.02f;
  settings.runtime.indirect_intensity = 1.0f;
  settings.storage.max_probe_count = 1024;
  settings.debug.enabled = true;
  settings.debug.visualize_volume_bounds = true;
  settings.debug.visualize_probe_positions = true;
  settings.debug.visualize_selected_probe = true;
  settings.debug.visualization_scale = 2.0f;

  const auto ddgi_volume_entity = scene->CreateEntity("DDGI Probe Volume");
  const auto ddgi_volume = scene->GetOrSetPrivateComponent<DdgiVolume>(ddgi_volume_entity).lock();
  ddgi_volume->probe_counts = {9, 9, 9};
  ddgi_volume->probe_spacing = glm::vec3(0.3f);
  ddgi_volume->volume_origin = {0.0f, 0.0f, 0.0f};
  ddgi_volume->relocation_distance = 0.1f;
  ddgi_volume->enable_probe_relocation = true;
  ddgi_volume->enable_probe_classification = false;
  ddgi_volume->visualize_bounds = true;
  ddgi_volume->visualize_probe_positions = true;
  ddgi_volume->max_visualized_probes = 512;
  ddgi_volume->probe_visualization_size = 0.03f;
  ddgi_volume->ClampSettings();

  Transform ddgi_volume_transform;
  ddgi_volume_transform.SetPosition(glm::vec3(0.0f, 0.0f, -3.0f));
  scene->SetDataComponent(ddgi_volume_entity, ddgi_volume_transform);
}

void ConfigureCornellBoxScene(const std::shared_ptr<Scene>& scene) {
  scene->environment.ambient_light_intensity = 0.0f;
  if (const auto* directional_light_owners = scene->UnsafeGetPrivateComponentOwnersList<DirectionalLight>()) {
    for (const auto& owner : *directional_light_owners) {
      if (const auto directional_light = scene->GetOrSetPrivateComponent<DirectionalLight>(owner).lock()) {
        directional_light->SetEnabled(false);
      }
    }
  }

  const auto main_camera = scene->main_camera.Get<Camera>();
  main_camera->Resize({1920, 1080});
  main_camera->post_processing_stack_ref = AssetManager::CreateTemporaryAsset<PostProcessingStack>();
  const auto main_camera_entity = main_camera->GetOwner();
  Transform main_camera_transform;
  main_camera_transform.SetPosition(glm::vec3(0.0f, 0.0f, 1.6f));
  scene->SetDataComponent(main_camera_entity, main_camera_transform);
  scene->GetOrSetPrivateComponent<PlayerController>(main_camera_entity);
  if (const auto editor_layer = ApplicationContext::Get().GetLayer<EditorLayer>()) {
    editor_layer->SetSceneCameraPosition(glm::vec3(0.0f, 0.0f, 1.6f));
  }

  const auto base_entity = scene->CreateEntity("Cornell Box");
  const auto white = CreateCornellMaterial(glm::vec3(0.78f));
  const auto red = CreateCornellMaterial(glm::vec3(0.9f, 0.08f, 0.05f));
  const auto green = CreateCornellMaterial(glm::vec3(0.05f, 0.65f, 0.12f));
  const auto light_material = CreateCornellMaterial(glm::vec3(1.0f), 8.0f);

  CreateCornellBox(scene, base_entity, "Floor", {0.0f, -1.0f, -3.0f}, {2.0f, 0.04f, 2.0f}, white);
  CreateCornellBox(scene, base_entity, "Ceiling", {0.0f, 1.0f, -3.0f}, {2.0f, 0.04f, 2.0f}, white);
  CreateCornellBox(scene, base_entity, "Back Wall", {0.0f, 0.0f, -4.0f}, {2.0f, 2.0f, 0.04f}, white);
  CreateCornellBox(scene, base_entity, "Left Wall", {-1.0f, 0.0f, -3.0f}, {0.04f, 2.0f, 2.0f}, red);
  CreateCornellBox(scene, base_entity, "Right Wall", {1.0f, 0.0f, -3.0f}, {0.04f, 2.0f, 2.0f}, green);
  CreateCornellBox(scene, base_entity, "Tall Box", {0.42f, -0.48f, -3.24f}, {0.45f, 1.0f, 0.45f}, white);
  CreateCornellBox(scene, base_entity, "Short Box", {-0.42f, -0.68f, -2.6f}, {0.55f, 0.62f, 0.55f}, white);
  CreateCornellBox(scene, base_entity, "Ceiling Light Mesh", {0.0f, 0.94f, -3.0f}, {0.42f, 0.02f, 0.42f},
                   light_material);

  const auto light_entity = scene->CreateEntity("Cornell Ceiling Light");
  const auto point_light = scene->GetOrSetPrivateComponent<PointLight>(light_entity).lock();
  point_light->cast_shadow = true;
  point_light->diffuse = glm::vec3(1.0f);
  point_light->diffuse_brightness = 45.0f;
  point_light->light_size = 0.08f;
  point_light->constant = 1.0f;
  point_light->linear = 0.08f;
  point_light->quadratic = 0.02f;
  Transform light_transform;
  light_transform.SetPosition(glm::vec3(0.0f, 0.82f, -3.0f));
  scene->SetDataComponent(light_entity, light_transform);
  scene->SetParent(light_entity, base_entity);

  ConfigureCornellBoxDdgi(scene);
}

void ConfigureThinWallDdgi(const std::shared_ptr<Scene>& scene) {
  auto& settings = scene->environment.ddgi_settings;
  settings.runtime.enabled = true;
  settings.runtime.pause_updates = false;
  settings.runtime.ray_count = 64;
  settings.runtime.normal_bias = 0.015f;
  settings.runtime.visibility_moment_bias = 0.02f;
  settings.runtime.indirect_intensity = 1.0f;
  settings.storage.max_probe_count = 1024;
  settings.debug.enabled = true;
  settings.debug.visualize_volume_bounds = true;
  settings.debug.visualize_probe_positions = true;
  settings.debug.visualize_selected_probe = true;
  settings.debug.visualization_scale = 2.0f;

  const auto ddgi_volume_entity = scene->CreateEntity("DDGI Probe Volume");
  const auto ddgi_volume = scene->GetOrSetPrivateComponent<DdgiVolume>(ddgi_volume_entity).lock();
  ddgi_volume->probe_counts = {8, 6, 8};
  ddgi_volume->probe_spacing = glm::vec3(0.35f);
  ddgi_volume->volume_origin = {0.0f, 0.0f, 0.0f};
  ddgi_volume->relocation_distance = 0.25f;
  ddgi_volume->enable_probe_relocation = true;
  ddgi_volume->enable_probe_classification = false;
  ddgi_volume->visualize_bounds = true;
  ddgi_volume->visualize_probe_positions = true;
  ddgi_volume->max_visualized_probes = 512;
  ddgi_volume->probe_visualization_size = 0.03f;
  ddgi_volume->ClampSettings();

  Transform ddgi_volume_transform;
  ddgi_volume_transform.SetPosition(glm::vec3(0.0f, 0.0f, -3.0f));
  scene->SetDataComponent(ddgi_volume_entity, ddgi_volume_transform);
}

void ConfigureThinWallScene(const std::shared_ptr<Scene>& scene) {
  scene->environment.ambient_light_intensity = 0.0f;
  if (const auto* directional_light_owners = scene->UnsafeGetPrivateComponentOwnersList<DirectionalLight>()) {
    for (const auto& owner : *directional_light_owners) {
      if (const auto directional_light = scene->GetOrSetPrivateComponent<DirectionalLight>(owner).lock()) {
        directional_light->SetEnabled(false);
      }
    }
  }

  const auto main_camera = scene->main_camera.Get<Camera>();
  main_camera->Resize({1920, 1080});
  main_camera->post_processing_stack_ref = AssetManager::CreateTemporaryAsset<PostProcessingStack>();
  const auto main_camera_entity = main_camera->GetOwner();
  Transform main_camera_transform;
  main_camera_transform.SetPosition(glm::vec3(0.0f, 0.0f, 0.9f));
  scene->SetDataComponent(main_camera_entity, main_camera_transform);
  scene->GetOrSetPrivateComponent<PlayerController>(main_camera_entity);
  if (const auto editor_layer = ApplicationContext::Get().GetLayer<EditorLayer>()) {
    editor_layer->SetSceneCameraPosition(glm::vec3(0.0f, 0.0f, 0.9f));
  }

  const auto base_entity = scene->CreateEntity("Thin Wall DDGI Room");
  const auto white = CreateCornellMaterial(glm::vec3(0.78f));
  const auto warm = CreateCornellMaterial(glm::vec3(0.9f, 0.65f, 0.18f));
  const auto cool = CreateCornellMaterial(glm::vec3(0.15f, 0.32f, 0.9f));
  const auto blocker = CreateCornellMaterial(glm::vec3(0.82f));
  const auto light_material = CreateCornellMaterial(glm::vec3(1.0f, 0.86f, 0.28f), 8.0f);

  CreateCornellBox(scene, base_entity, "Floor", {0.0f, -1.0f, -3.0f}, {2.4f, 0.04f, 2.0f}, white);
  CreateCornellBox(scene, base_entity, "Ceiling", {0.0f, 1.0f, -3.0f}, {2.4f, 0.04f, 2.0f}, white);
  CreateCornellBox(scene, base_entity, "Back Wall", {0.0f, 0.0f, -4.0f}, {2.4f, 2.0f, 0.04f}, white);
  CreateCornellBox(scene, base_entity, "Left Wall", {-1.2f, 0.0f, -3.0f}, {0.04f, 2.0f, 2.0f}, warm);
  CreateCornellBox(scene, base_entity, "Right Wall", {1.2f, 0.0f, -3.0f}, {0.04f, 2.0f, 2.0f}, cool);
  CreateCornellBox(scene, base_entity, "Thin Wall Blocker", {0.0f, 0.0f, -3.0f}, {0.035f, 1.85f, 1.85f}, blocker);
  CreateCornellBox(scene, base_entity, "Thin Wall Light Marker", {-0.72f, 0.94f, -3.0f}, {0.18f, 0.02f, 0.18f},
                   light_material);

  const auto light_entity = scene->CreateEntity("Thin Wall Left Light");
  const auto point_light = scene->GetOrSetPrivateComponent<PointLight>(light_entity).lock();
  point_light->cast_shadow = true;
  point_light->diffuse = glm::vec3(1.0f, 0.82f, 0.25f);
  point_light->diffuse_brightness = 80.0f;
  point_light->light_size = 0.04f;
  point_light->constant = 1.0f;
  point_light->linear = 0.08f;
  point_light->quadratic = 0.02f;
  Transform light_transform;
  light_transform.SetPosition(glm::vec3(-0.72f, 0.72f, -3.0f));
  scene->SetDataComponent(light_entity, light_transform);
  scene->SetParent(light_entity, base_entity);

  ConfigureThinWallDdgi(scene);
}

void RemoveGeneratedFiles(const std::filesystem::path& root, const std::unordered_set<std::string>& extensions) {
  if (!std::filesystem::exists(root))
    return;
  for (const auto& i : std::filesystem::recursive_directory_iterator(root)) {
    if (i.is_directory())
      continue;
    if (extensions.find(i.path().extension().string()) != extensions.end()) {
      std::filesystem::remove(i.path());
    }
  }
}

void RemoveGeneratedDemoProjectFiles(const std::filesystem::path& resource_root) {
  const auto demo_projects_root = resource_root / "EvoEngine-DemoProjects";
  const auto gaussian_splat_demo_root = demo_projects_root / "3DGS";
  const auto bicycle_demo_root = demo_projects_root / "Bicycle";
  if (!std::filesystem::exists(demo_projects_root)) {
    return;
  }
  std::filesystem::recursive_directory_iterator iterator(demo_projects_root);
  for (const auto end = std::filesystem::recursive_directory_iterator(); iterator != end; ++iterator) {
    const auto path = iterator->path();
    if (iterator->is_directory()) {
      if (path == gaussian_splat_demo_root || path == bicycle_demo_root) {
        iterator.disable_recursion_pending();
      }
      continue;
    }
    const auto extension = path.extension().string();
    if (extension == ".evescene" || extension == ".eveproj" || extension == ".evefilemeta" ||
        extension == ".evefoldermeta") {
      std::filesystem::remove(path);
    }
  }
}

void RemoveGeneratedProceduralGalaxyProjectFiles(const std::filesystem::path& resource_root) {
  RemoveGeneratedFiles(resource_root / "EvoEngine-DemoProjects" / "Universe",
                       {".evescene", ".eveproj", ".evefilemeta", ".evefoldermeta"});
}

void RemoveGeneratedStandaloneGaussianSplatProjectFiles(const std::filesystem::path& root) {
  if (!std::filesystem::exists(root)) {
    return;
  }
  for (const auto& i : std::filesystem::recursive_directory_iterator(root)) {
    if (i.is_directory()) {
      continue;
    }
    const auto path = i.path();
    if (path.extension() == ".evescene" || path.extension() == ".eveproj" ||
        (path.extension() == ".evefilemeta" && path.stem().extension() == ".evescene")) {
      std::filesystem::remove(path);
    }
  }
}

void RemoveGeneratedGaussianSplatProjectFiles(const std::filesystem::path& resource_root) {
  RemoveGeneratedStandaloneGaussianSplatProjectFiles(resource_root / "EvoEngine-DemoProjects" / "3DGS");
}

void RemoveGeneratedBicycleProjectFiles(const std::filesystem::path& resource_root) {
  RemoveGeneratedStandaloneGaussianSplatProjectFiles(resource_root / "EvoEngine-DemoProjects" / "Bicycle");
}

bool NeedsGeneratedAssetFileCopy(const std::filesystem::path& source, const std::filesystem::path& target) {
  if (!std::filesystem::exists(target) || !std::filesystem::is_regular_file(target)) {
    return true;
  }
  return std::filesystem::file_size(source) != std::filesystem::file_size(target);
}

void CopyGeneratedAssetFileIfNeeded(const std::filesystem::path& source, const std::filesystem::path& target) {
  if (!std::filesystem::exists(source) || !std::filesystem::is_regular_file(source)) {
    throw std::runtime_error("Missing Rendering Regression source asset: " + source.string());
  }
  if (!NeedsGeneratedAssetFileCopy(source, target)) {
    return;
  }
  std::filesystem::create_directories(target.parent_path());
  std::filesystem::copy_file(source, target, std::filesystem::copy_options::overwrite_existing);
}

void CopyGeneratedAssetDirectoryIfNeeded(const std::filesystem::path& source, const std::filesystem::path& target,
                                         const std::filesystem::path& sentinel_file) {
  if (!std::filesystem::exists(source) || !std::filesystem::is_directory(source)) {
    throw std::runtime_error("Missing Rendering Regression source asset folder: " + source.string());
  }
  const auto source_sentinel = source / sentinel_file;
  const auto target_sentinel = target / sentinel_file;
  if (!std::filesystem::exists(source_sentinel) || !std::filesystem::is_regular_file(source_sentinel)) {
    throw std::runtime_error("Missing Rendering Regression source asset: " + source_sentinel.string());
  }
  if (!NeedsGeneratedAssetFileCopy(source_sentinel, target_sentinel)) {
    return;
  }
  std::filesystem::create_directories(target.parent_path());
  std::filesystem::copy(source, target,
                        std::filesystem::copy_options::recursive | std::filesystem::copy_options::overwrite_existing);
}

void PrepareRenderingRegressionGeneratedAssets(const std::filesystem::path& resource_root) {
  const auto source_models = resource_root / "EvoEngine-DemoProjects" / "Rendering" / "Assets" / "Models";
  const auto target_models =
      resource_root / ".generated" / "EvoEngine-DemoProjects" / "RenderingRegression" / "Assets" / "Models";
  CopyGeneratedAssetFileIfNeeded(source_models / "Capoeira.fbx", target_models / "Capoeira.fbx");
  CopyGeneratedAssetFileIfNeeded(source_models / "EvoEngine.obj", target_models / "EvoEngine.obj");
  CopyGeneratedAssetDirectoryIfNeeded(source_models / "Sponza_FBX", target_models / "Sponza_FBX", "Sponza.fbx");
}

void ConfigureProceduralGalaxyScene(const std::shared_ptr<Scene>& scene) {
  scene->environment.environment_type = Scene::EnvironmentType::Color;
  scene->environment.background_color = glm::vec3(0.0f);
  scene->environment.background_intensity = 0.0f;
  scene->environment.ambient_light_intensity = 0.0f;

  const auto main_camera = scene->main_camera.Get<Camera>();
  main_camera->Resize({1920, 1080});
  main_camera->skybox.Clear();
  main_camera->camera_settings.use_clear_color = true;
  main_camera->camera_settings.clear_color = glm::vec4(0.0f, 0.0f, 0.0f, 1.0f);
  main_camera->camera_settings.background_intensity = 0.0f;
  main_camera->camera_settings.far_distance = 1000.0f;
  main_camera->post_processing_stack_ref = AssetManager::CreateTemporaryAsset<PostProcessingStack>();

  const auto main_camera_entity = main_camera->GetOwner();
  Transform main_camera_transform;
  main_camera_transform.SetPosition(glm::vec3(0.0f, 100.0f, 100.0f));
  main_camera_transform.SetEulerRotation(glm::radians(glm::vec3(-50.0f, 0.0f, 0.0f)));
  scene->SetDataComponent(main_camera_entity, main_camera_transform);
  scene->GetOrSetPrivateComponent<PlayerController>(main_camera_entity);

  if (const auto editor_layer = ApplicationContext::Get().GetLayer<EditorLayer>()) {
    editor_layer->SetSceneCameraPosition(glm::vec3(0.0f, 100.0f, 100.0f));
    editor_layer->SetSceneCameraRotation(glm::quat(glm::radians(glm::vec3(-50.0f, 0.0f, 0.0f))));
    if (const auto scene_camera = editor_layer->GetSceneCamera()) {
      scene_camera->skybox.Clear();
      scene_camera->camera_settings.use_clear_color = true;
      scene_camera->camera_settings.clear_color = glm::vec4(0.0f, 0.0f, 0.0f, 1.0f);
      scene_camera->camera_settings.background_intensity = 0.0f;
      scene_camera->camera_settings.far_distance = 1000.0f;
      scene_camera->ResetFrameCount();
    }
  }
}

Entity GetOrCreateGaussianSplatDemoEntity(const std::shared_ptr<Scene>& scene, const char* entity_name) {
  if (const auto* owners = scene->UnsafeGetPrivateComponentOwnersList<GaussianSplatRenderer>()) {
    for (const auto& owner : *owners) {
      if (scene->IsEntityValid(owner) && scene->GetEntityName(owner) == entity_name) {
        return owner;
      }
    }
  }
  return scene->CreateEntity(entity_name);
}

void RemoveRayTracingTlasSeedEntity(const std::shared_ptr<Scene>& scene) {
  bool removed = false;
  do {
    removed = false;
    if (const auto* owners = scene->UnsafeGetPrivateComponentOwnersList<MeshRenderer>()) {
      for (const auto& owner : *owners) {
        if (scene->IsEntityValid(owner) && scene->GetEntityName(owner) == "Ray Tracing TLAS Seed") {
          scene->DeleteEntity(owner);
          removed = true;
          break;
        }
      }
    }
  } while (removed);
}

GaussianSplatDemoCameraPose GetBicycleDemoCameraPose(const glm::vec3& center) {
  GaussianSplatDemoCameraPose pose;
  pose.position = kBicycleDemoCamera0Position - center;
  // EvoEngine keeps the INRIA PLY coordinates unflipped; invert the image-down axis for editor/main camera up.
  pose.rotation =
      glm::quatLookAt(glm::normalize(kBicycleDemoCamera0Front), glm::normalize(-kBicycleDemoCamera0ImageDown));
  return pose;
}

void ConfigureGaussianSplatDemoSceneImpl(const std::shared_ptr<Scene>& scene, const uint64_t gaussian_splat_handle,
                                         const char* asset_name, const char* entity_name, const int sh_degree,
                                         const GaussianSplatSortMode sort_mode = GaussianSplatSortMode::GpuRadix,
                                         const bool bake_bicycle_camera_pose_into_splat_transform = false) {
  if (!scene) {
    return;
  }
  scene->environment.environment_type = Scene::EnvironmentType::EnvironmentalMap;
  scene->environment.environmental_map = Resources::GetInstance().GetDefaultEnvironmentalMap();
  scene->environment.background_color = glm::vec3(0.01f, 0.012f, 0.016f);
  scene->environment.background_intensity = 0.6f;
  scene->environment.ambient_light_intensity = 0.25f;

  std::shared_ptr<GaussianSplat> gaussian_splat;
  try {
    gaussian_splat = AssetManager::GetAsset<GaussianSplat>(Handle(gaussian_splat_handle));
  } catch (const std::exception& error) {
    EVOENGINE_ERROR("Failed to load " + std::string(asset_name) + " Gaussian splat asset: " + std::string(error.what()))
    return;
  }
  if (!gaussian_splat || gaussian_splat->Empty()) {
    EVOENGINE_ERROR(std::string(asset_name) + " Gaussian splat asset is empty or unavailable.")
    return;
  }

  const auto min_bound = gaussian_splat->GetMinBound();
  const auto max_bound = gaussian_splat->GetMaxBound();
  const auto center = (min_bound + max_bound) * 0.5f;
  const auto size = glm::max(max_bound - min_bound, glm::vec3(0.001f));
  const float radius = std::max(glm::length(size) * 0.5f, 0.5f);
  auto camera_position = glm::vec3(0.0f, radius * 0.05f, radius * 2.4f);
  auto camera_rotation = glm::quat(glm::radians(glm::vec3(-3.0f, 0.0f, 0.0f)));
  auto camera_fov = 55.0f;
  auto gaussian_position = glm::vec3(-center);
  auto gaussian_rotation = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
  if (bake_bicycle_camera_pose_into_splat_transform) {
    const auto camera_pose = GetBicycleDemoCameraPose(center);
    const auto inverse_pose_rotation = glm::inverse(camera_pose.rotation);
    gaussian_rotation = glm::normalize(camera_rotation * inverse_pose_rotation);
    gaussian_position =
        camera_position + camera_rotation * (inverse_pose_rotation * (glm::vec3(-center) - camera_pose.position));
  }

  const auto main_camera = scene->main_camera.Get<Camera>();
  main_camera->Resize({1920, 1080});
  main_camera->skybox = Resources::GetInstance().GetDefaultSkybox();
  main_camera->camera_settings.use_clear_color = false;
  main_camera->camera_settings.clear_color = glm::vec4(0.01f, 0.012f, 0.016f, 1.0f);
  main_camera->camera_settings.background_intensity = 0.6f;
  main_camera->camera_settings.near_distance = std::max(radius * 0.01f, 0.01f);
  main_camera->camera_settings.far_distance = std::max(radius * 10.0f, 100.0f);
  main_camera->camera_settings.fov = camera_fov;
  main_camera->camera_render_mode = Camera::CameraRenderMode::Rasterization;
  main_camera->post_processing_stack_ref = AssetManager::CreateTemporaryAsset<PostProcessingStack>();

  const auto main_camera_entity = main_camera->GetOwner();
  Transform main_camera_transform;
  main_camera_transform.SetPosition(camera_position);
  main_camera_transform.SetRotation(camera_rotation);
  scene->SetDataComponent(main_camera_entity, main_camera_transform);
  scene->GetOrSetPrivateComponent<PlayerController>(main_camera_entity);

  const auto gaussian_entity = GetOrCreateGaussianSplatDemoEntity(scene, entity_name);
  const auto gaussian_renderer = scene->GetOrSetPrivateComponent<GaussianSplatRenderer>(gaussian_entity).lock();
  gaussian_renderer->gaussian_splat.Set<GaussianSplat>(gaussian_splat);
  gaussian_renderer->opacity_scale = 1.0f;
  gaussian_renderer->sh_degree = sh_degree;
  gaussian_renderer->sort_mode = sort_mode;
  gaussian_renderer->depth_mode = GaussianSplatDepthMode::SceneDepth;
  Transform gaussian_transform;
  gaussian_transform.SetValue(gaussian_position, gaussian_rotation, glm::vec3(1.0f));
  scene->SetDataComponent(gaussian_entity, gaussian_transform);
  RemoveRayTracingTlasSeedEntity(scene);

  if (const auto editor_layer = ApplicationContext::Get().GetLayer<EditorLayer>()) {
    editor_layer->velocity = std::max(radius * 0.25f, 0.5f);
    editor_layer->default_scene_camera_position = camera_position;
    editor_layer->SetSceneCameraPosition(camera_position);
    editor_layer->SetSceneCameraRotation(camera_rotation);
    if (const auto scene_camera = editor_layer->GetSceneCamera()) {
      scene_camera->skybox = Resources::GetInstance().GetDefaultSkybox();
      scene_camera->camera_settings.use_clear_color = false;
      scene_camera->camera_settings.clear_color = glm::vec4(0.01f, 0.012f, 0.016f, 1.0f);
      scene_camera->camera_settings.background_intensity = 0.6f;
      scene_camera->camera_settings.near_distance = main_camera->camera_settings.near_distance;
      scene_camera->camera_settings.far_distance = main_camera->camera_settings.far_distance;
      scene_camera->camera_settings.fov = main_camera->camera_settings.fov;
      scene_camera->camera_render_mode = Camera::CameraRenderMode::Rasterization;
      scene_camera->ResetFrameCount();
    }
  }

  scene->Save();
  ProjectManager::SaveProject();
}

std::optional<Entity> FindEntityNamed(const std::shared_ptr<Scene>& scene, const std::string& name) {
  if (!scene) {
    return std::nullopt;
  }
  for (const auto& entity : scene->UnsafeGetAllEntities()) {
    if (scene->IsEntityValid(entity) && scene->GetEntityName(entity) == name) {
      return entity;
    }
  }
  return std::nullopt;
}

void RemoveDefaultDirectionalLight(const std::shared_ptr<Scene>& scene) {
  const auto default_light_entity = FindEntityNamed(scene, "Directional Light");
  if (default_light_entity && scene->HasPrivateComponent<DirectionalLight>(*default_light_entity)) {
    scene->DeleteEntity(*default_light_entity);
  }
}

struct BistroDdgiVolumeConfig {
  glm::ivec3 probe_counts = glm::ivec3(1);
  glm::vec3 probe_spacing = glm::vec3(1.0f);
  glm::vec3 volume_origin = glm::vec3(0.0f);
};

int BistroDdgiProbeCount(const glm::ivec3& probe_counts) {
  return probe_counts.x * probe_counts.y * probe_counts.z;
}

glm::ivec3 CalculateBistroDdgiProbeCounts(const glm::vec3& padded_extent) {
  const auto max_extent = std::max(padded_extent.x, std::max(padded_extent.y, padded_extent.z));
  auto spacing =
      std::max(kBistroDdgiMinProbeSpacing, max_extent / static_cast<float>(kBistroDdgiTargetLongestAxisProbeCount - 1));
  const auto calculate_axis_count = [](const float extent, const float spacing) {
    return glm::clamp(static_cast<int>(std::ceil(extent / spacing)) + 1, 2, kBistroDdgiMaxAxisProbeCount);
  };

  glm::ivec3 probe_counts;
  do {
    probe_counts = {calculate_axis_count(padded_extent.x, spacing), calculate_axis_count(padded_extent.y, spacing),
                    calculate_axis_count(padded_extent.z, spacing)};
    spacing *= 1.1f;
  } while (BistroDdgiProbeCount(probe_counts) > kBistroDdgiMaxProbeCount);
  return probe_counts;
}

BistroDdgiVolumeConfig CalculateBistroDdgiVolumeConfig(const Bound& bistro_world_bound) {
  const auto extent = glm::max(bistro_world_bound.max - bistro_world_bound.min, glm::vec3(1.0f));
  const auto padded_extent = extent * kBistroDdgiBoundsPadding;
  BistroDdgiVolumeConfig config;
  config.probe_counts = CalculateBistroDdgiProbeCounts(padded_extent);
  config.probe_spacing = padded_extent / glm::vec3(config.probe_counts - glm::ivec3(1));
  config.volume_origin = bistro_world_bound.Center();
  return config;
}

void ConfigureBistroDemoDdgi(const std::shared_ptr<Scene>& scene, const Bound& bistro_world_bound) {
  const auto config = CalculateBistroDdgiVolumeConfig(bistro_world_bound);
  const auto probe_count = BistroDdgiProbeCount(config.probe_counts);
  const auto min_spacing = std::min(config.probe_spacing.x, std::min(config.probe_spacing.y, config.probe_spacing.z));

  auto& settings = scene->environment.ddgi_settings;
  settings.runtime.enabled = true;
  settings.runtime.pause_updates = false;
  settings.runtime.ray_count = 64;
  settings.runtime.normal_bias = std::max(0.02f, min_spacing * 0.02f);
  settings.runtime.view_bias = std::max(0.05f, min_spacing * 0.04f);
  settings.runtime.reset_probe_history = true;
  settings.runtime.indirect_intensity = 1.0f;
  settings.volume_defaults.probe_counts = config.probe_counts;
  settings.volume_defaults.probe_spacing = config.probe_spacing;
  settings.volume_defaults.volume_origin = config.volume_origin;
  settings.volume_defaults.relocation_distance = std::min(min_spacing * 0.25f, 50.0f);
  settings.volume_defaults.enable_probe_relocation = true;
  settings.volume_defaults.enable_probe_classification = false;
  settings.storage.max_probe_count = kBistroDdgiMaxProbeCount;
  settings.debug.enabled = false;
  settings.debug.visualize_volume_bounds = false;
  settings.debug.visualize_probe_positions = false;
  settings.debug.visualize_selected_probe = false;

  const auto existing_ddgi_volume = FindEntityNamed(scene, kBistroDdgiVolumeName);
  const auto ddgi_volume_entity =
      existing_ddgi_volume ? *existing_ddgi_volume : scene->CreateEntity(kBistroDdgiVolumeName);
  const auto ddgi_volume = scene->GetOrSetPrivateComponent<DdgiVolume>(ddgi_volume_entity).lock();
  ddgi_volume->probe_counts = config.probe_counts;
  ddgi_volume->probe_spacing = config.probe_spacing;
  ddgi_volume->volume_origin = config.volume_origin;
  ddgi_volume->relocation_distance = settings.volume_defaults.relocation_distance;
  ddgi_volume->enable_probe_relocation = settings.volume_defaults.enable_probe_relocation;
  ddgi_volume->enable_probe_classification = settings.volume_defaults.enable_probe_classification;
  ddgi_volume->visualize_bounds = false;
  ddgi_volume->visualize_probe_positions = false;
  ddgi_volume->max_visualized_probes = std::min(probe_count, 1024);
  ddgi_volume->probe_visualization_size = std::max(0.08f, min_spacing * 0.02f);
  ddgi_volume->ClampSettings();

  Transform ddgi_volume_transform;
  scene->SetDataComponent(ddgi_volume_entity, ddgi_volume_transform);

  std::ostringstream stream;
  stream << "Bistro DDGI setup: enabled=" << settings.runtime.enabled << ", probe_counts=(" << config.probe_counts.x
         << "," << config.probe_counts.y << "," << config.probe_counts.z << "), probe_count=" << probe_count
         << ", probe_spacing=(" << config.probe_spacing.x << "," << config.probe_spacing.y << ","
         << config.probe_spacing.z << "), volume_origin=(" << config.volume_origin.x << "," << config.volume_origin.y
         << "," << config.volume_origin.z << "), normal_bias=" << settings.runtime.normal_bias
         << ", view_bias=" << settings.runtime.view_bias
         << ", storage_max_probe_count=" << settings.storage.max_probe_count
         << ", debug_enabled=" << settings.debug.enabled << ", world_bound_min=(" << bistro_world_bound.min.x << ","
         << bistro_world_bound.min.y << "," << bistro_world_bound.min.z << "), world_bound_max=("
         << bistro_world_bound.max.x << "," << bistro_world_bound.max.y << "," << bistro_world_bound.max.z << ")";
  EVOENGINE_LOG(stream.str())
}

void ApplyBistroDirectionalLightIntensity(const std::shared_ptr<Scene>& scene) {
  if (!scene) {
    return;
  }
  const auto* owners = scene->UnsafeGetPrivateComponentOwnersList<DirectionalLight>();
  if (!owners) {
    EVOENGINE_WARNING("Bistro directional light policy: no directional light components found.")
    return;
  }
  for (const auto& owner : *owners) {
    if (scene->GetEntityName(owner) != kBistroImportedSunLightName) {
      continue;
    }
    const auto light = scene->GetOrSetPrivateComponent<DirectionalLight>(owner).lock();
    if (!light) {
      continue;
    }
    const auto previous_color = light->diffuse * light->diffuse_brightness;
    const auto previous_brightness = light->diffuse_brightness;
    light->diffuse_brightness = kBistroDirectionalLightIntensity;
    const auto effective_color = light->diffuse * light->diffuse_brightness;

    std::ostringstream stream;
    stream << "Bistro directional light policy: entity=\"" << kBistroImportedSunLightName
           << "\", previous_brightness=" << previous_brightness
           << ", target_brightness=" << kBistroDirectionalLightIntensity
           << ", effective_brightness=" << light->diffuse_brightness << ", previous_to_effective_ratio="
           << (kBistroDirectionalLightIntensity > 0.0f ? previous_brightness / kBistroDirectionalLightIntensity : 0.0f)
           << ", previous_color=(" << previous_color.x << "," << previous_color.y << "," << previous_color.z
           << "), effective_color=(" << effective_color.x << "," << effective_color.y << "," << effective_color.z
           << ")";
    EVOENGINE_LOG(stream.str())
    return;
  }
  EVOENGINE_WARNING("Bistro directional light policy: imported Sun directional light was not found.")
}
}  // namespace

void evo_engine::SetRenderingRegressionTemporalMotionEnabled(const bool enabled) {
  if (!rendering_regression_temporal_motion_state) {
    return;
  }
  rendering_regression_temporal_motion_state->enabled = enabled;
  rendering_regression_temporal_motion_state->frame = 0;
}

void evo_engine::ConfigureGaussianSplatDemoScene(const std::shared_ptr<Scene>& scene) {
  ConfigureGaussianSplatDemoSceneImpl(scene, kSpatialDragonGaussianSplatHandle, "Spatial Dragon", "Spatial Dragon 3DGS",
                                      0);
}

void evo_engine::ConfigureBicycleDemoScene(const std::shared_ptr<Scene>& scene) {
  ConfigureGaussianSplatDemoSceneImpl(scene, kBicycleGaussianSplatHandle, "Bicycle", "Bicycle 3DGS", 3,
                                      GaussianSplatSortMode::GpuRadix, true);
}

void evo_engine::ConfigureRenderingRegressionDemoScene(const std::shared_ptr<Scene>& scene) {
  if (!scene) {
    return;
  }
  ApplyBistroParityRendererState(scene);
  RemoveDefaultDirectionalLight(scene);
  if (const auto existing_root = FindEntityNamed(scene, kRenderingRegressionRootName)) {
    scene->DeleteEntity(*existing_root);
  }
  if (const auto default_ground = FindEntityNamed(scene, "Ground")) {
    scene->DeleteEntity(*default_ground);
  }

  scene->environment.environment_type = Scene::EnvironmentType::Color;
  scene->environment.background_color = glm::vec3(0.025f, 0.028f, 0.034f);
  scene->environment.background_intensity = 1.0f;
  scene->environment.ambient_light_intensity = 0.025f;

  const auto root = scene->CreateEntity(kRenderingRegressionRootName);
  const auto& primitives = Resources::GetInstance().GetPrimitives();
  CreateRenderingRegressionProbe(scene, root, "M42 Material Probe Ground", primitives.cube,
                                 glm::vec3(0.0f, -0.62f, -2.1f), glm::vec3(4.8f, 0.08f, 4.2f),
                                 glm::vec3(0.48f, 0.50f, 0.52f), 0.85f, 0.0f);
  CreateRenderingRegressionProbe(scene, root, "M42 Material Probe Dielectric", primitives.sphere,
                                 glm::vec3(-1.8f, -0.08f, -1.0f), glm::vec3(0.32f), glm::vec3(0.95f, 0.38f, 0.32f),
                                 0.55f, 0.0f);
  CreateRenderingRegressionProbe(scene, root, "M42 Material Probe Metallic", primitives.sphere,
                                 glm::vec3(-0.65f, -0.08f, -1.0f), glm::vec3(0.32f), glm::vec3(0.85f, 0.78f, 0.62f),
                                 0.22f, 1.0f);
  CreateRenderingRegressionProbe(scene, root, "M42 Material Probe Rough", primitives.sphere,
                                 glm::vec3(0.55f, -0.08f, -1.0f), glm::vec3(0.32f), glm::vec3(0.42f, 0.72f, 0.58f),
                                 0.95f, 0.0f);
  CreateRenderingRegressionProbe(scene, root, "M42 Material Probe Transmission", primitives.sphere,
                                 glm::vec3(1.35f, -0.08f, -1.65f), glm::vec3(0.32f), glm::vec3(0.55f, 0.82f, 1.0f),
                                 0.08f, 0.0f, 0.0f, true, 0.65f);
  CreateRenderingRegressionProbe(scene, root, "M42 Material Probe Emissive", primitives.cube,
                                 glm::vec3(1.75f, 0.02f, -1.0f), glm::vec3(0.28f), glm::vec3(0.35f, 0.65f, 1.0f), 0.8f,
                                 0.0f, 3.0f);
  CreateRenderingRegressionProbe(scene, root, "M42 Firefly Clamp Probe", primitives.sphere,
                                 glm::vec3(2.15f, 0.85f, -2.35f), glm::vec3(0.075f), glm::vec3(1.0f, 0.96f, 0.72f),
                                 0.2f, 0.0f, 80.0f, false);
  CreateRenderingRegressionProbe(scene, root, "M42 Auto SPP Convergence Probe Low Contrast", primitives.cube,
                                 glm::vec3(-0.48f, -0.42f, -0.28f), glm::vec3(0.18f, 0.18f, 0.05f), glm::vec3(0.42f),
                                 0.9f, 0.0f);
  CreateRenderingRegressionProbe(scene, root, "M42 Auto SPP Convergence Probe High Contrast", primitives.cube,
                                 glm::vec3(0.0f, -0.42f, -0.28f), glm::vec3(0.18f, 0.18f, 0.05f), glm::vec3(0.9f), 0.9f,
                                 0.0f);
  CreateRenderingRegressionProbe(scene, root, "M42 Punctual Light Shadow Blocker", primitives.cube,
                                 glm::vec3(0.95f, -0.08f, -2.15f), glm::vec3(0.18f, 0.62f, 0.18f),
                                 glm::vec3(0.18f, 0.20f, 0.24f), 0.8f, 0.0f);
  CreateRenderingRegressionProbe(scene, root, "M42 Temporal Depth Discontinuity", primitives.cube,
                                 glm::vec3(0.0f, 0.05f, -2.45f), glm::vec3(1.9f, 0.72f, 0.08f), glm::vec3(0.02f), 1.0f,
                                 0.0f);
  CreateRenderingRegressionProbe(scene, root, "M42 Saturated Red Probe", primitives.cube,
                                 glm::vec3(-2.75f, 0.35f, -2.0f), glm::vec3(0.22f, 0.85f, 0.08f),
                                 glm::vec3(1.0f, 0.0f, 0.0f), 0.8f, 0.0f);
  CreateRenderingRegressionProbe(scene, root, "M42 High Contrast White Probe", primitives.cube,
                                 glm::vec3(2.55f, 0.35f, -2.0f), glm::vec3(0.20f, 0.85f, 0.08f), glm::vec3(1.0f), 0.8f,
                                 0.0f);
  const auto moving_rigid = CreateRenderingRegressionProbe(
      scene, root, "M42 Temporal Moving Rigid Probe", primitives.cube, glm::vec3(0.0f, -0.14f, -1.55f),
      glm::vec3(0.24f), glm::vec3(1.0f, 0.85f, 0.05f), 0.28f, 0.1f);
  const auto moving_transparent = CreateRenderingRegressionProbe(
      scene, root, "M42 Temporal Moving Transparent Probe", primitives.sphere, glm::vec3(1.35f, 0.42f, -1.15f),
      glm::vec3(0.28f), glm::vec3(0.25f, 0.65f, 1.0f), 0.12f, 0.0f, 0.0f, false, 0.72f);

  ConfigureRenderingRegressionImportedProbes(scene, root);
  ConfigureRenderingRegressionLights(scene, root);
  ConfigureRenderingRegressionCamera(scene);

  rendering_regression_temporal_motion_state = std::make_shared<RenderingRegressionTemporalMotionState>();
  rendering_regression_temporal_motion_state->scene = scene;
  rendering_regression_temporal_motion_state->rigid_entity = moving_rigid;
  rendering_regression_temporal_motion_state->transparent_entity = moving_transparent;
  if (const auto skinned_entity = FindEntityNamed(scene, "M42 Skinned Capoeira Probe")) {
    rendering_regression_temporal_motion_state->skinned_entity = *skinned_entity;
  }
  RegisterRenderingRegressionTemporalMotionUpdate();
}

void evo_engine::ConfigureBistroRayTracingPostProcessing(const std::shared_ptr<Camera>& camera) {
  ConfigureBistroReferenceToneMapping(camera);
}

void evo_engine::ConfigureBistroParityCapture(const std::shared_ptr<Scene>& scene,
                                              const std::shared_ptr<Camera>& camera) {
  ApplyBistroParityRendererState(scene);
  ConfigureBistroReferenceToneMapping(camera);
  if (camera) {
    camera->camera_settings.bounce = kBistroReferencePathTraceMaxDepth;
  }
}

void evo_engine::LogBistroParityCaptureState(const std::shared_ptr<Scene>& scene, const std::shared_ptr<Camera>& camera,
                                             const int width, const int height, const std::string& render_mode_name,
                                             const std::filesystem::path& output_path) {
  if (!scene || !camera) {
    return;
  }
  const auto bistro = std::dynamic_pointer_cast<Prefab>(ProjectManager::GetOrCreateAsset("Models/Bistro/bistro.gltf"));
  const auto stats = GatherBistroParityStats(scene, bistro);
  const auto light_count = stats.directional_light_count + stats.point_light_count + stats.spot_light_count;
  std::ostringstream stream;
  stream << "Bistro parity scene: output=" << output_path.string() << ", render_mode=" << render_mode_name
         << ", resolution=" << width << "x" << height << ", mesh_primitives=" << stats.mesh_primitive_count
         << ", mesh_renderers=" << stats.mesh_renderer_count
         << ", skinned_mesh_renderers=" << stats.skinned_mesh_renderer_count
         << ", alpha_opaque_render_primitives=" << stats.alpha_opaque_render_primitive_count
         << ", alpha_mask_render_primitives=" << stats.alpha_mask_render_primitive_count
         << ", alpha_blend_render_primitives=" << stats.alpha_blend_render_primitive_count
         << ", raster_mask_policy=deferred_gbuffer_alpha_cutoff"
         << ", raster_blend_policy=sorted_back_to_front_transparent_pass"
         << ", triangle_count=" << stats.triangle_count << ", material_count=" << stats.material_count
         << ", texture_count=" << stats.texture_count << ", light_count=" << light_count
         << ", directional_lights=" << stats.directional_light_count << ", point_lights=" << stats.point_light_count
         << ", spot_lights=" << stats.spot_light_count << ", camera_fov=" << camera->camera_settings.fov
         << ", camera_near=" << camera->camera_settings.near_distance
         << ", camera_far=" << camera->camera_settings.far_distance
         << ", camera_samples=" << camera->camera_settings.sample_size
         << ", camera_bounces=" << camera->camera_settings.bounce << ", camera_gamma=" << camera->camera_settings.gamma
         << ", environment_type=" << BistroEnvironmentTypeName(scene->environment.environment_type)
         << ", background_intensity=" << scene->environment.background_intensity
         << ", ambient_light_intensity=" << scene->environment.ambient_light_intensity
         << ", ddgi_enabled=" << scene->environment.ddgi_settings.runtime.enabled
         << ", volumetric_clouds_enabled=" << scene->environment.volumetric_cloud_settings.enabled;
  if (const auto editor_layer = ApplicationContext::Get().GetLayer<EditorLayer>()) {
    const auto position = editor_layer->GetSceneCameraPosition();
    const auto rotation = editor_layer->GetSceneCameraRotation();
    stream << ", camera_position=(" << position.x << "," << position.y << "," << position.z << "), camera_rotation=("
           << rotation.w << "," << rotation.x << "," << rotation.y << "," << rotation.z << ")";
  }
  if (const auto post_processing_stack = camera->post_processing_stack_ref.Get<PostProcessingStack>();
      post_processing_stack && post_processing_stack->tone_mapping) {
    const auto& tone_mapping = *post_processing_stack->tone_mapping;
    stream << ", tone_mapping_enabled=" << post_processing_stack->enable_tone_mapping
           << ", tone_mapping_method=" << static_cast<int>(tone_mapping.method)
           << ", tone_mapping_exposure=" << tone_mapping.exposure
           << ", tone_mapping_brightness=" << tone_mapping.brightness
           << ", tone_mapping_contrast=" << tone_mapping.contrast
           << ", tone_mapping_saturation=" << tone_mapping.saturation
           << ", tone_mapping_auto_exposure=" << tone_mapping.auto_exposure
           << ", tone_mapping_average_mode=" << tone_mapping.average_mode;
  }
  EVOENGINE_LOG(stream.str())
}

void evo_engine::ConfigureBistroDemoScene(const std::shared_ptr<Scene>& scene) {
  if (!scene) {
    return;
  }
  const auto bistro = std::dynamic_pointer_cast<Prefab>(ProjectManager::GetOrCreateAsset("Models/Bistro/bistro.gltf"));
  if (!bistro) {
    EVOENGINE_ERROR("Failed to load Bistro prefab asset.")
    return;
  }
  const auto renderer_count = CountPrefabRenderers(bistro);
  if (renderer_count == 0) {
    throw std::runtime_error("Bistro prefab import produced no mesh renderers.");
  }
  LogBistroRasterAlphaPolicy(GatherBistroParityStats(scene, bistro));
  RemoveDefaultDirectionalLight(scene);
  ApplyBistroParityRendererState(scene);
  const auto existing_bistro_entity = FindEntityNamed(scene, "Bistro");
  const auto bistro_entity = existing_bistro_entity ? *existing_bistro_entity : bistro->ToEntity(scene, false, false);
  if (!existing_bistro_entity) {
    scene->SetEntityName(bistro_entity, "Bistro");
  }
  const auto camera_frame = CalculateBistroCameraFrame(bistro->GetBoundingBox());
  const auto bistro_root_transform = CalculateBistroRootTransformForCameraFrame(camera_frame);
  scene->SetDataComponent(bistro_entity, bistro_root_transform);
  TransformGraph::CalculateTransformGraphForDescendants(scene, bistro_entity);
  const auto bistro_world_bound = scene->GetEntityBoundingBox(bistro_entity);
  ApplyBistroDirectionalLightIntensity(scene);

  scene->environment.environment_type = Scene::EnvironmentType::Color;
  scene->environment.background_color = glm::vec3(0.0f);
  scene->environment.background_intensity = 0.0f;
  scene->environment.ambient_light_intensity = 0.0f;
  ConfigureBistroDemoDdgi(scene, bistro_world_bound);
  if (const auto main_camera = scene->main_camera.Get<Camera>()) {
    main_camera->Resize({1920, 1080});
    main_camera->skybox.Clear();
    main_camera->camera_render_mode = Camera::CameraRenderMode::RayTracing;
    main_camera->camera_settings.use_clear_color = true;
    main_camera->camera_settings.clear_color = glm::vec4(0.0f, 0.0f, 0.0f, 1.0f);
    main_camera->camera_settings.background_intensity = 0.0f;
    main_camera->camera_settings.fov = camera_frame.fov;
    main_camera->camera_settings.near_distance = camera_frame.near_distance;
    main_camera->camera_settings.far_distance = camera_frame.far_distance;
    main_camera->camera_settings.bounce = kBistroReferencePathTraceMaxDepth;
    main_camera->post_processing_stack_ref = AssetManager::CreateTemporaryAsset<PostProcessingStack>();
    ConfigureBistroRayTracingPostProcessing(main_camera);
    const auto main_camera_entity = main_camera->GetOwner();
    Transform main_camera_transform;
    main_camera_transform.SetValue(camera_frame.position, camera_frame.rotation, glm::vec3(1.0f));
    scene->SetDataComponent(main_camera_entity, main_camera_transform);
    scene->GetOrSetPrivateComponent<PlayerController>(main_camera_entity);
  }
  if (const auto editor_layer = ApplicationContext::Get().GetLayer<EditorLayer>()) {
    editor_layer->enable_gizmos = false;
    editor_layer->show_scene_info = true;
    editor_layer->SetSelectedEntity({});
    editor_layer->SetSceneCameraPosition(camera_frame.position);
    editor_layer->SetSceneCameraRotation(camera_frame.rotation);
    if (const auto scene_camera = editor_layer->GetSceneCamera()) {
      scene_camera->skybox.Clear();
      scene_camera->camera_render_mode = Camera::CameraRenderMode::Rasterization;
      scene_camera->camera_settings.use_clear_color = true;
      scene_camera->camera_settings.clear_color = glm::vec4(0.0f, 0.0f, 0.0f, 1.0f);
      scene_camera->camera_settings.background_intensity = 0.0f;
      scene_camera->camera_settings.fov = camera_frame.fov;
      scene_camera->camera_settings.near_distance = camera_frame.near_distance;
      scene_camera->camera_settings.far_distance = camera_frame.far_distance;
      scene_camera->camera_settings.bounce = kBistroReferencePathTraceMaxDepth;
      ConfigureBistroReferenceToneMapping(scene_camera);
      scene_camera->ResetFrameCount();
    }
  }
}

std::filesystem::path evo_engine::FindDemoResourcesRoot(const std::filesystem::path& preferred_root) {
  if (!preferred_root.empty() && std::filesystem::exists(preferred_root)) {
    return path_utils::NormalizeAbsolutePath(preferred_root);
  }

  return path_utils::FindAncestorChildPath("Resources", std::filesystem::current_path(), 8);
}

void evo_engine::ClearGeneratedDemoProjectFiles(const std::filesystem::path& resource_folder_path) {
  const auto resource_root = FindDemoResourcesRoot(resource_folder_path);
  if (resource_root.empty()) {
    return;
  }

  RemoveGeneratedDemoProjectFiles(resource_root);
  RemoveGeneratedFiles(resource_root, {".uescene", ".ueproj"});
}

void evo_engine::ClearGeneratedProceduralGalaxyProjectFiles(const std::filesystem::path& resource_folder_path) {
  const auto resource_root = FindDemoResourcesRoot(resource_folder_path);
  if (resource_root.empty()) {
    return;
  }
  RemoveGeneratedProceduralGalaxyProjectFiles(resource_root);
}

void evo_engine::SetupDemoScene(const DemoSetup demo_setup, ApplicationInitializationSettings& application_info,
                                const std::filesystem::path& resource_folder_path,
                                const bool clear_generated_project_files) {
  const auto resource_root = FindDemoResourcesRoot(resource_folder_path);
  if (demo_setup != DemoSetup::Empty && resource_root.empty()) {
    EVOENGINE_ERROR("Failed to locate Resources folder for DemoApp scene setup.")
    return;
  }

  if (demo_setup == DemoSetup::ProceduralGalaxy && clear_generated_project_files) {
    RemoveGeneratedProceduralGalaxyProjectFiles(resource_root);
  } else if (demo_setup == DemoSetup::GaussianSplat && clear_generated_project_files) {
    RemoveGeneratedGaussianSplatProjectFiles(resource_root);
  } else if (demo_setup == DemoSetup::Bicycle && clear_generated_project_files) {
    RemoveGeneratedBicycleProjectFiles(resource_root);
  } else if (demo_setup != DemoSetup::Empty && clear_generated_project_files) {
    ClearGeneratedDemoProjectFiles(resource_root);
  }

  if (demo_setup == DemoSetup::RenderingRegression) {
    PrepareRenderingRegressionGeneratedAssets(resource_root);
  }

  switch (demo_setup) {
    case DemoSetup::Rendering: {
      application_info.application_name = "Rendering Demo";
      application_info.project_path = resource_root / "EvoEngine-DemoProjects/Rendering/Rendering.eveproj";
      application_info.default_window_size = {1920, 1080};
      ProjectManager::SetActionAfterNewScene([](const std::shared_ptr<Scene>& scene) {
        scene->environment.ambient_light_intensity = 0.0f;

        const auto main_camera = scene->main_camera.Get<Camera>();
        main_camera->Resize({1920, 1080});
        main_camera->post_processing_stack_ref = AssetManager::CreateTemporaryAsset<PostProcessingStack>();
        const auto main_camera_entity = main_camera->GetOwner();
        auto main_camera_transform = scene->GetDataComponent<Transform>(main_camera_entity);
        main_camera_transform.SetPosition(glm::vec3(0, 0, 3));
        scene->SetDataComponent(main_camera_entity, main_camera_transform);
        scene->GetOrSetPrivateComponent<PlayerController>(main_camera_entity);

        if (const auto editor_layer = ApplicationContext::Get().GetLayer<EditorLayer>()) {
          editor_layer->SetSceneCameraPosition(glm::vec3(0, 0, 3));
        }

        const auto demo_scene = LoadRenderingScene(scene, "Rendering Demo", true);
        Transform demo_transform;
        demo_transform.SetScale(glm::vec3(0.5f));
        scene->SetDataComponent(demo_scene, demo_transform);
        ConfigureRenderingDemoDdgi(scene);

        const auto directional_light_entity = scene->CreateEntity("Top Down Directional Light");
        const auto directional_light =
            scene->GetOrSetPrivateComponent<DirectionalLight>(directional_light_entity).lock();
        directional_light->diffuse = glm::vec3(1.0f);
        directional_light->diffuse_brightness = 5.0f;
        directional_light->light_size = 0.01f;
        Transform directional_light_transform;
        directional_light_transform.SetEulerRotation(glm::radians(glm::vec3(90.0f, 0.0f, 0.0f)));
        scene->SetDataComponent(directional_light_entity, directional_light_transform);

        const auto left_point_light_right_entity = scene->CreateEntity("Left Point Light");
        const auto point_light_right_renderer =
            scene->GetOrSetPrivateComponent<MeshRenderer>(left_point_light_right_entity).lock();
        point_light_right_renderer->cast_shadow = false;
        const auto point_light_right_material = AssetManager::CreateTemporaryAsset<Material>();
        point_light_right_renderer->material.Set<Material>(point_light_right_material);
        ConfigureMaterial(point_light_right_material, glm::vec3(1.0f, 0.8f, 0.0f), 1.0f, 1.0f, 2.0f);
        point_light_right_renderer->mesh = Resources::GetInstance().GetPrimitives().sphere;
        const auto point_light_right =
            scene->GetOrSetPrivateComponent<PointLight>(left_point_light_right_entity).lock();
        point_light_right->diffuse_brightness = 24.0f;
        point_light_right->light_size = 0.005f;
        point_light_right->constant = 2.5f;
        point_light_right->linear = 0.5f;
        point_light_right->quadratic = 0.1f;
        point_light_right->diffuse = glm::vec3(1.0f, 0.8f, 0.0f);

        Transform left_point_light_right_transform;
        left_point_light_right_transform.SetPosition(glm::vec3(3, 0, -2.5f));
        left_point_light_right_transform.SetScale(glm::vec3(0.1f));
        scene->SetDataComponent(left_point_light_right_entity, left_point_light_right_transform);

        ApplicationContext::Get().RegisterUpdateFunction([=]() {
          static bool last_frame_playing = false;
          auto& application = ApplicationContext::Get();
          const auto playing = application.IsPlaying();
          if (!playing) {
            last_frame_playing = false;
            return;
          }
          const auto current_scene = application.GetActiveScene();
          if (!current_scene) {
            last_frame_playing = playing;
            return;
          }
          auto moving_light_entity = left_point_light_right_entity;
          if (!current_scene->IsEntityValid(moving_light_entity)) {
            for (const auto& entity : current_scene->UnsafeGetAllEntities()) {
              if (current_scene->IsEntityValid(entity) && current_scene->GetEntityName(entity) == "Left Point Light") {
                moving_light_entity = entity;
                break;
              }
            }
          }
          if (!current_scene->IsEntityValid(moving_light_entity)) {
            last_frame_playing = playing;
            return;
          }
          static float start_time;
          if (!last_frame_playing)
            start_time = application.GetTimes().Now();
          const float current_time = application.GetTimes().Now() - start_time;
          const float cos_time = glm::cos(current_time / 2.5f);

          Transform current_left_point_light_transform;
          current_left_point_light_transform.SetPosition(glm::vec3(3, 0, cos_time * 2.5f - 2.5f));
          current_left_point_light_transform.SetScale(glm::vec3(0.1f));
          current_scene->SetDataComponent(moving_light_entity, current_left_point_light_transform);

          last_frame_playing = playing;
        });
      });
    } break;
    case DemoSetup::CornellBox: {
      application_info.application_name = "Cornell Box";
      application_info.project_path = resource_root / "EvoEngine-DemoProjects/CornellBox/CornellBox.eveproj";
      application_info.default_window_size = {1920, 1080};
      ProjectManager::SetActionAfterNewScene([](const std::shared_ptr<Scene>& scene) {
        ConfigureCornellBoxScene(scene);
      });
    } break;
    case DemoSetup::ThinWall: {
      application_info.application_name = "Thin Wall DDGI";
      application_info.project_path = resource_root / "EvoEngine-DemoProjects/ThinWall/ThinWall.eveproj";
      application_info.default_window_size = {1920, 1080};
      ProjectManager::SetActionAfterNewScene([](const std::shared_ptr<Scene>& scene) {
        ConfigureThinWallScene(scene);
      });
    } break;
    case DemoSetup::ProceduralGalaxy: {
      application_info.application_name = "Procedural Galaxy";
      application_info.project_path = resource_root / "EvoEngine-DemoProjects/Universe/ProceduralGalaxy.eveproj";
      application_info.default_window_size = {1920, 1080};
      application_info.enable_runtime_packages = true;
      application_info.startup_runtime_packages = {"Universe"};
      ProjectManager::SetActionAfterNewScene([](const std::shared_ptr<Scene>& scene) {
        ConfigureProceduralGalaxyScene(scene);
      });
    } break;
    case DemoSetup::RenderingRegression: {
      application_info.application_name = "Rendering Regression";
      application_info.project_path =
          resource_root / ".generated/EvoEngine-DemoProjects/RenderingRegression/RenderingRegression.eveproj";
      application_info.default_window_size = {1920, 1080};
      ProjectManager::SetActionAfterNewScene([](const std::shared_ptr<Scene>& scene) {
        ConfigureRenderingRegressionDemoScene(scene);
      });
    } break;
    case DemoSetup::GaussianSplat: {
      application_info.application_name = "3D Gaussian Splatting";
      application_info.project_path = resource_root / "EvoEngine-DemoProjects/3DGS/3DGS.eveproj";
      application_info.default_window_size = {1920, 1080};
      ProjectManager::SetActionAfterNewScene([](const std::shared_ptr<Scene>& scene) {
        ConfigureGaussianSplatDemoScene(scene);
      });
    } break;
    case DemoSetup::Bicycle: {
      application_info.application_name = "Bicycle";
      application_info.project_path = resource_root / "EvoEngine-DemoProjects/Bicycle/Bicycle.eveproj";
      application_info.default_window_size = {1920, 1080};
      ProjectManager::SetActionAfterNewScene([](const std::shared_ptr<Scene>& scene) {
        ConfigureBicycleDemoScene(scene);
      });
    } break;
    case DemoSetup::Bistro: {
      application_info.application_name = "Bistro";
      application_info.project_path = resource_root / ".generated/EvoEngine-DemoProjects/Bistro/Bistro.eveproj";
      application_info.default_window_size = {1920, 1080};
      ProjectManager::SetActionAfterNewScene([](const std::shared_ptr<Scene>& scene) {
        ConfigureBistroDemoScene(scene);
      });
    } break;
    case DemoSetup::Universe:
    case DemoSetup::Empty:
    default: {
    } break;
  }
}
