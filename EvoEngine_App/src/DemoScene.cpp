#include "DemoScene.hpp"

#include "AnimationPlayer.hpp"
#include "Animator.hpp"
#include "Application.hpp"
#include "AssetManager.hpp"
#include "Camera.hpp"
#include "EditorLayer.hpp"
#include "EnvironmentalLighting.hpp"
#include "EnvironmentalLightingResolver.hpp"
#include "EnvironmentalMap.hpp"
#include "GaussianSplat.hpp"
#include "GaussianSplatRenderer.hpp"
#include "GeometryStorage.hpp"
#include "GlobalReflectionProbe.hpp"
#include "Lights.hpp"
#include "Material.hpp"
#include "Mesh.hpp"
#include "MeshRenderer.hpp"
#include "Particles.hpp"
#include "PathUtils.hpp"
#include "Platform.hpp"
#include "PlayerController.hpp"
#include "PostProcessingStack.hpp"
#include "Prefab.hpp"
#include "ProjectManager.hpp"
#include "RenderInstanceStorage.hpp"
#include "RenderLayer.hpp"
#include "Resources.hpp"
#include "SkinnedMeshRenderer.hpp"
#include "StrandsRenderer.hpp"
#include "Texture2D.hpp"
#include "TextureStorage.hpp"
#include "Times.hpp"
#include "TransformGraph.hpp"
#include "WindowLayer.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <limits>
#include <map>
#include <optional>
#include <set>
#include <sstream>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>

#include <glm/gtc/packing.hpp>

using namespace evo_engine;

namespace {
constexpr uint64_t kSpatialDragonGaussianSplatHandle = 9739484957885691067ull;
constexpr uint64_t kBicycleGaussianSplatHandle = 14453709846752502031ull;
constexpr const char* kStrandValidationRootName = "Strand Mesh Shader Validation";
constexpr const char* kStrandValidationDynamicName = "Strand Validation Multi Meshlet";
constexpr const char* kStrandPunctualValidationRootName = "Strand Punctual Shadow Validation";
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
constexpr int kBistroDdgiProbeCount = 22 * 7 * 26;
const glm::ivec3 kBistroDdgiProbeCounts = glm::ivec3(22, 7, 26);
const glm::vec3 kBistroDdgiProbeSpacing = glm::vec3(4.0f);
const glm::vec3 kBistroDdgiVolumeOrigin = glm::vec3(-10.0f, 10.0f, -15.0f);
constexpr float kBistroDirectionalLightIntensity = 10.0f;
constexpr float kBistroDirectionalLightSize = 0.01f;
// DDGI_VALIDATION_FIXTURE_CONSTANTS_BEGIN
constexpr const char* kRenderingRegressionRootName = "M42 Rendering Regression Root";
constexpr float kDdgiValidationNeutralProbeVariabilityThreshold = 0.2f;
constexpr float kDdgiValidationHighContrastProbeVariabilityThreshold = 0.5f;
constexpr int kDdgiValidationProbeVariabilityMinSamples = 16;
constexpr float kDdgiValidationEmitterRadiance = 45.0f;
constexpr float DdgiValidationBoxSurfaceArea(const float x, const float y, const float z) {
  return 2.0f * (x * y + x * z + y * z);
}
constexpr float kDdgiValidationSmallEmitterArea = DdgiValidationBoxSurfaceArea(0.16f, 0.025f, 0.16f);
constexpr float kDdgiValidationLargeEmitterArea = DdgiValidationBoxSurfaceArea(1.35f, 0.025f, 1.1f);
constexpr float kDdgiValidationEqualPowerSmallEmitterRadiance =
    kDdgiValidationEmitterRadiance * kDdgiValidationLargeEmitterArea / kDdgiValidationSmallEmitterArea;
static_assert(kDdgiValidationSmallEmitterArea > 0.0f && kDdgiValidationLargeEmitterArea > 0.0f);
// DDGI_VALIDATION_FIXTURE_CONSTANTS_END
constexpr const char* kBistroDdgiVolumeName = "DDGI Probe Volume";
constexpr const char* kBistroImportedSunLightName = "Sun directional light";
constexpr const char* kSponzaLightingDirectory = "Lighting/Sponza";
constexpr const char* kSponzaEnvironmentPath = "Lighting/Sponza/SponzaEnvironment.eveenvironmentalmap";
constexpr const char* kSponzaGlobalProbePath = "Lighting/Sponza/SponzaGlobal.evereflectionprobe";
constexpr const char* kSponzaLocalProbePackPath = "Lighting/Sponza/SponzaLocal.evereflectionprobepack";
constexpr std::array<const char*, 5> kSponzaLocalProbeNames = {
    "Sponza Left Gallery Reflection Probe", "Sponza Right Gallery Reflection Probe",
    "Sponza Central Front Reflection Probe", "Sponza Central Middle Reflection Probe",
    "Sponza Central Rear Reflection Probe"};

void SyncTemporaryEnvironmentalLightingSettingsFromScene(const std::shared_ptr<Scene>& scene);

void SetDdgiUpdatesPaused(const bool paused) {
  if (const auto render_layer = ApplicationContext::Get().GetLayer<RenderLayer>()) {
    render_layer->GetDdgiSessionState().pause_updates = paused;
  }
}

void RequestDdgiHistoryReset() {
  if (const auto render_layer = ApplicationContext::Get().GetLayer<RenderLayer>()) {
    render_layer->RequestDdgiHistoryReset();
  }
}

void DisableDdgiDebugVisualization() {
  if (const auto render_layer = ApplicationContext::Get().GetLayer<RenderLayer>()) {
    auto& session = render_layer->GetDdgiSessionState();
    session.show_probes = false;
    session.show_selected_probe = false;
    session.show_selected_probe_state = false;
    session.show_rays = false;
  }
}

std::vector<glm::vec4> ReadAndStoreValidationCapture(const std::shared_ptr<RenderTexture>& render_texture,
                                                     const glm::uvec2 resolution, const std::filesystem::path& path,
                                                     const char* label) {
  if (!render_texture || render_texture->GetExtent().width != resolution.x ||
      render_texture->GetExtent().height != resolution.y) {
    throw std::runtime_error(std::string(label) + " capture resolution is invalid.");
  }
  std::vector<glm::vec4> pixels;
  render_texture->GetRgbaChannelData(pixels);
  if (pixels.size() != static_cast<size_t>(resolution.x) * resolution.y) {
    throw std::runtime_error(std::string(label) + " capture readback size is invalid.");
  }
  std::vector<float> rgba(pixels.size() * 4u);
  for (size_t index = 0; index < pixels.size(); ++index) {
    rgba[index * 4u] = pixels[index].x;
    rgba[index * 4u + 1u] = pixels[index].y;
    rgba[index * 4u + 2u] = pixels[index].z;
    rgba[index * 4u + 3u] = 1.0f;
  }
  std::error_code error;
  std::filesystem::remove(path, error);
  if (error) {
    throw std::runtime_error("Failed to replace validation capture: " + path.string());
  }
  Texture2D::StoreToPng(path, rgba, static_cast<int>(resolution.x), static_cast<int>(resolution.y), 4, 4);
  if (!std::filesystem::exists(path, error) || error || std::filesystem::file_size(path, error) == 0u || error) {
    throw std::runtime_error("Failed to save validation capture: " + path.string());
  }
  return pixels;
}

struct RenderingRegressionTemporalMotionState {
  std::weak_ptr<Scene> scene;
  Entity rigid_entity;
  Entity transparent_entity;
  Entity skinned_entity;
  Entity light_entity;
  Entity secondary_geometry_entity;
  uint64_t frame = 0;
  bool geometry_camera_enabled = false;
  bool moving_light_enabled = false;
  bool secondary_geometry_enabled = false;
  bool suffix_light_fixture = false;
};

std::shared_ptr<RenderingRegressionTemporalMotionState> rendering_regression_temporal_motion_state;
bool rendering_regression_temporal_motion_registered = false;

template <typename LightComponent>
void DisableRenderingRegressionFixtureLightsExcept(const std::shared_ptr<Scene>& scene, const Entity keep_entity) {
  if (const auto* owners = scene->UnsafeGetPrivateComponentOwnersList<LightComponent>()) {
    for (const auto& owner : *owners) {
      if (owner != keep_entity) {
        if (const auto light = scene->GetOrSetPrivateComponent<LightComponent>(owner).lock()) {
          light->SetEnabled(false);
        }
      }
    }
  }
}

struct StrandGizmoValidationState {
  std::weak_ptr<Scene> scene;
  std::shared_ptr<Strands> strands;
};

std::shared_ptr<StrandGizmoValidationState> strand_gizmo_validation_state;
bool strand_gizmo_validation_registered = false;

void RegisterStrandGizmoValidationUpdate() {
  if (strand_gizmo_validation_registered) {
    return;
  }
  strand_gizmo_validation_registered = true;
  ApplicationContext::Get().RegisterUpdateFunction([] {
    const auto state = strand_gizmo_validation_state;
    const auto scene = state ? state->scene.lock() : nullptr;
    auto& application = ApplicationContext::Get();
    const auto editor_layer = application.GetLayer<EditorLayer>();
    const auto camera = editor_layer ? editor_layer->GetSceneCamera() : nullptr;
    if (!state || !state->strands || !scene || application.GetActiveScene() != scene || !camera) {
      return;
    }

    constexpr std::array modes = {GizmoSettings::ColorMode::Default, GizmoSettings::ColorMode::VertexColor,
                                  GizmoSettings::ColorMode::NormalColor};
    for (size_t mode = 0; mode < modes.size(); ++mode) {
      GizmoSettings settings;
      settings.color_mode = modes[mode];
      const auto x = (static_cast<float>(mode) - 1.0f) * 1.5f;
      const auto model = glm::translate(glm::mat4(1.0f), glm::vec3(x, 0.0f, -2.5f));
      editor_layer->DrawGizmoStrands(state->strands, camera, glm::vec4(1.0f, 0.45f, 0.12f, 1.0f), model, 1.0f,
                                     settings);
    }
  });
}

void RegisterRenderingRegressionTemporalMotionUpdate() {
  if (rendering_regression_temporal_motion_registered) {
    return;
  }
  rendering_regression_temporal_motion_registered = true;
  ApplicationContext::Get().RegisterUpdateFunction([] {
    const auto state = rendering_regression_temporal_motion_state;
    if (!state ||
        (!state->geometry_camera_enabled && !state->moving_light_enabled && !state->secondary_geometry_enabled)) {
      return;
    }
    const auto scene = state->scene.lock();
    auto& application = ApplicationContext::Get();
    if (!scene || application.GetActiveScene() != scene) {
      return;
    }

    if (state->suffix_light_fixture || state->secondary_geometry_enabled) {
      DisableRenderingRegressionFixtureLightsExcept<DirectionalLight>(scene, state->light_entity);
      DisableRenderingRegressionFixtureLightsExcept<PointLight>(scene, state->light_entity);
      DisableRenderingRegressionFixtureLightsExcept<SpotLight>(scene, state->light_entity);
    }

    const uint64_t motion_period = state->suffix_light_fixture || state->secondary_geometry_enabled ? 24u : 240u;
    const float phase = static_cast<float>(state->frame % motion_period) *
                        (2.0f * glm::pi<float>() / static_cast<float>(motion_period));
    ++state->frame;
    if (state->geometry_camera_enabled && scene->IsEntityValid(state->rigid_entity)) {
      Transform transform;
      transform.SetValue(glm::vec3(glm::sin(phase) * 1.65f, -0.14f, -1.55f), glm::vec3(0.0f, phase * 1.5f, 0.0f),
                         glm::vec3(0.24f));
      scene->SetDataComponent(state->rigid_entity, transform);
    }
    if (state->geometry_camera_enabled && scene->IsEntityValid(state->transparent_entity)) {
      Transform transform;
      transform.SetValue(glm::vec3(glm::cos(phase * 0.8f) * 1.35f, 0.42f, -1.15f), glm::vec3(phase * 0.7f, phase, 0.0f),
                         glm::vec3(0.28f));
      scene->SetDataComponent(state->transparent_entity, transform);
    }
    if (state->geometry_camera_enabled && scene->IsEntityValid(state->skinned_entity) &&
        scene->HasPrivateComponent<Animator>(state->skinned_entity)) {
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

    if (state->geometry_camera_enabled) {
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
    }
    if (state->moving_light_enabled && scene->IsEntityValid(state->light_entity)) {
      Transform transform = scene->GetDataComponent<Transform>(state->light_entity);
      transform.SetPosition(state->suffix_light_fixture
                                ? glm::vec3(glm::sin(phase) * 1.85f, 2.25f, -3.25f + glm::cos(phase) * 0.65f)
                                : glm::vec3(-2.3f + glm::sin(phase) * 1.2f, 1.25f + glm::cos(phase * 0.7f) * 0.3f,
                                            -1.7f + glm::cos(phase) * 0.8f));
      scene->SetDataComponent(state->light_entity, transform);
    }
    if (state->secondary_geometry_enabled && scene->IsEntityValid(state->secondary_geometry_entity)) {
      Transform transform;
      transform.SetValue(glm::vec3(glm::sin(phase) * 1.35f, 1.6f + glm::cos(phase * 0.5f) * 0.15f, -2.8f),
                         glm::vec3(0.0f, phase * 0.25f, 0.0f), glm::vec3(1.55f, 0.08f, 1.25f));
      scene->SetDataComponent(state->secondary_geometry_entity, transform);
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

// DDGI_VALIDATION_MATERIAL_HELPER_BEGIN
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
// DDGI_VALIDATION_MATERIAL_HELPER_END

std::shared_ptr<Strands> CreateStrandValidationGeometry(const size_t point_count, const glm::vec4& color,
                                                        const float thickness = 0.075f) {
  StrandPointAttributes attributes;
  attributes.normal = true;
  attributes.tex_coord = true;
  attributes.color = true;
  std::vector<StrandPoint> points(point_count);
  for (size_t index = 0; index < point_count; ++index) {
    const float t = static_cast<float>(index) / static_cast<float>(point_count - 1);
    auto& point = points[index];
    point.position = glm::vec3(0.18f * glm::sin(t * glm::two_pi<float>() * 1.5f), t * 1.8f - 0.9f,
                               0.08f * glm::cos(t * glm::two_pi<float>()));
    point.thickness = thickness * (1.0f - 0.25f * t);
    point.normal = glm::normalize(glm::vec3(0.15f * glm::sin(t * glm::two_pi<float>()), 0.0f, 1.0f));
    point.tex_coord = t;
    point.color = color;
  }
  const auto strands = AssetManager::CreateTemporaryAsset<Strands>();
  if (point_count == 4) {
    strands->SetSegments(attributes, {0}, points);
  } else {
    strands->SetStrands(attributes, {0, static_cast<glm::uint>(point_count)}, points);
  }
  return strands;
}

Entity CreateStrandValidationRenderer(const std::shared_ptr<Scene>& scene, const Entity root, const std::string& name,
                                      const std::shared_ptr<Strands>& strands,
                                      const std::shared_ptr<Material>& material, const glm::vec3& position,
                                      const glm::vec3& scale, const bool cast_shadow,
                                      const glm::vec3& rotation = glm::vec3(0.0f, 0.0f, glm::radians(8.0f))) {
  const auto entity = scene->CreateEntity(name);
  const auto renderer = scene->GetOrSetPrivateComponent<StrandsRenderer>(entity).lock();
  renderer->strands = strands;
  renderer->material = material;
  renderer->cast_shadow = cast_shadow;
  Transform transform;
  transform.SetValue(position, rotation, scale);
  scene->SetDataComponent(entity, transform);
  scene->SetParent(entity, root);
  return entity;
}

// DDGI_VALIDATION_GEOMETRY_HELPERS_BEGIN
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

std::shared_ptr<Mesh> CreateRenderingRegressionMaterialQuad(const std::array<glm::vec4, 4>& colors) {
  std::vector<Vertex> vertices(4);
  vertices[0].position = glm::vec3(-0.5f, -0.5f, 0.0f);
  vertices[1].position = glm::vec3(0.5f, -0.5f, 0.0f);
  vertices[2].position = glm::vec3(0.5f, 0.5f, 0.0f);
  vertices[3].position = glm::vec3(-0.5f, 0.5f, 0.0f);
  const std::array<glm::vec2, 4> tex_coords_0 = {glm::vec2(0.0f, 0.0f), glm::vec2(1.0f, 0.0f), glm::vec2(1.0f, 1.0f),
                                                 glm::vec2(0.0f, 1.0f)};
  const std::array<glm::vec2, 4> tex_coords_1 = {glm::vec2(0.0f, 1.0f), glm::vec2(0.0f, 0.0f), glm::vec2(1.0f, 0.0f),
                                                 glm::vec2(1.0f, 1.0f)};
  const std::array<glm::vec2, 4> tex_coords_3 = {glm::vec2(1.0f, 1.0f), glm::vec2(0.0f, 1.0f), glm::vec2(0.0f, 0.0f),
                                                 glm::vec2(1.0f, 0.0f)};
  for (size_t i = 0; i < vertices.size(); ++i) {
    vertices[i].normal = glm::vec3(0.0f, 0.0f, 1.0f);
    vertices[i].color = colors[i];
    vertices[i].tex_coord = tex_coords_0[i];
    vertices[i].tex_coord_1 = tex_coords_1[i];
    vertices[i].tex_coord_2 = tex_coords_0[3 - i];
    vertices[i].tex_coord_3 = tex_coords_3[i];
  }
  VertexAttributes attributes;
  attributes.normal = true;
  attributes.tex_coord = true;
  attributes.tex_coord_1 = true;
  attributes.tex_coord_2 = true;
  attributes.tex_coord_3 = true;
  attributes.color = true;
  const auto mesh = AssetManager::CreateTemporaryAsset<Mesh>();
  mesh->SetVertices(attributes, vertices, {glm::uvec3(0, 1, 2), glm::uvec3(0, 2, 3)}, 3);
  return mesh;
}

std::shared_ptr<Mesh> CreateRenderingRegressionMirroredTangentSeam() {
  std::vector<Vertex> vertices(4);
  const std::array positions = {glm::vec3(-1.0f, -0.5f, 0.0f), glm::vec3(1.0f, -0.5f, 0.0f),
                                glm::vec3(-1.0f, 0.5f, 0.0f), glm::vec3(1.0f, 0.5f, 0.0f)};
  const std::array tex_coords = {glm::vec2(0.0f, 0.0f), glm::vec2(1.0f, 0.0f), glm::vec2(0.0f, 1.0f),
                                 glm::vec2(0.0f, 0.0f)};
  for (size_t index = 0; index < vertices.size(); ++index) {
    vertices[index].position = positions[index];
    vertices[index].normal = glm::vec3(0.0f, 0.0f, 1.0f);
    vertices[index].tex_coord = tex_coords[index];
  }
  VertexAttributes attributes;
  attributes.normal = true;
  attributes.tex_coord = true;
  const auto mesh = AssetManager::CreateTemporaryAsset<Mesh>();
  mesh->SetVertices(attributes, vertices, {glm::uvec3(0, 1, 2), glm::uvec3(2, 1, 3)});
  return mesh;
}

void SetRenderingRegressionSampler(const std::shared_ptr<Texture2D>& texture, const VkFilter filter,
                                   const VkSamplerAddressMode address_mode) {
  Texture2DSamplerSettings settings;
  settings.mag_filter = filter;
  settings.min_filter = filter;
  settings.mipmap_mode = filter == VK_FILTER_NEAREST ? VK_SAMPLER_MIPMAP_MODE_NEAREST : VK_SAMPLER_MIPMAP_MODE_LINEAR;
  settings.address_mode_u = address_mode;
  settings.address_mode_v = address_mode;
  texture->SetSamplerSettings(settings);
  texture->UnsafeUploadDataImmediately();
}

Entity CreateRenderingRegressionMaterialQuadEntity(const std::shared_ptr<Scene>& scene, const Entity& root,
                                                   const std::string& name, const std::shared_ptr<Mesh>& mesh,
                                                   const std::shared_ptr<Material>& material, const glm::vec3& position,
                                                   const glm::vec3& rotation, const glm::vec3& scale) {
  const auto entity = scene->CreateEntity(name);
  const auto renderer = scene->GetOrSetPrivateComponent<MeshRenderer>(entity).lock();
  renderer->mesh = mesh;
  renderer->material = material;
  Transform transform;
  transform.SetValue(position, rotation, scale);
  scene->SetDataComponent(entity, transform);
  scene->SetParent(entity, root);
  return entity;
}
// DDGI_VALIDATION_GEOMETRY_HELPERS_END

void ConfigureRenderingRegressionGltfMaterialProbes(const std::shared_ptr<Scene>& scene, const Entity& root) {
  const auto textured_mesh =
      CreateRenderingRegressionMaterialQuad({glm::vec4(1.0f, 0.35f, 0.2f, 1.0f), glm::vec4(0.2f, 1.0f, 0.35f, 1.0f),
                                             glm::vec4(0.25f, 0.45f, 1.0f, 1.0f), glm::vec4(1.0f, 0.9f, 0.25f, 1.0f)});
  const auto textured_material = AssetManager::CreateTemporaryAsset<Material>();
  auto color_texture = AssetManager::CreateTemporaryAsset<Texture2D>();
  color_texture->SetRgbaChannelData({glm::vec4(1.0f, 0.15f, 0.04f, 1.0f), glm::vec4(0.04f, 0.25f, 1.0f, 1.0f),
                                     glm::vec4(0.04f, 1.0f, 0.18f, 1.0f), glm::vec4(1.0f, 0.8f, 0.04f, 1.0f)},
                                    glm::uvec2(2, 2));
  auto normal_texture = AssetManager::CreateTemporaryAsset<Texture2D>();
  normal_texture->SetRgbaChannelData({glm::vec4(0.72f, 0.50f, 0.95f, 1.0f), glm::vec4(0.28f, 0.50f, 0.95f, 1.0f),
                                      glm::vec4(0.50f, 0.72f, 0.95f, 1.0f), glm::vec4(0.50f, 0.28f, 0.95f, 1.0f)},
                                     glm::uvec2(2, 2));
  const float rotation = glm::radians(30.0f);
  const glm::mat3x2 texture_transform(1.35f * std::cos(rotation), 1.35f * std::sin(rotation),
                                      -0.85f * std::sin(rotation), 0.85f * std::cos(rotation), 0.15f, 0.1f);
  const auto base_color_slot =
      textured_material->SetTexture(&GltfShadeMaterial::pbr_base_color_texture, color_texture, 3, texture_transform);
  textured_material->material_data.texture_infos[base_color_slot].color_space =
      static_cast<int32_t>(GltfTextureColorSpace::Srgb);
  textured_material->SetTexture(&GltfShadeMaterial::normal_texture, normal_texture, 3, texture_transform);
  ConfigureMaterial(textured_material, glm::vec3(1.0f), 0.45f, 0.0f);
  CreateRenderingRegressionMaterialQuadEntity(
      scene, root, "M9 UV3 Transform Vertex Color Probe", textured_mesh, textured_material,
      glm::vec3(-1.75f, 0.85f, -1.35f), glm::radians(glm::vec3(8.0f, 22.0f, 0.0f)), glm::vec3(0.75f, 0.48f, 1.0f));

  const auto mirrored_normal_material = AssetManager::CreateTemporaryAsset<Material>();
  mirrored_normal_material->SetTexture(&GltfShadeMaterial::normal_texture, normal_texture, 1, texture_transform);
  ConfigureMaterial(mirrored_normal_material, glm::vec3(0.25f, 0.65f, 1.0f), 0.45f, 0.0f);
  mirrored_normal_material->material_data.shade_material.double_sided = 1;
  mirrored_normal_material->MarkDirty();
  CreateRenderingRegressionMaterialQuadEntity(
      scene, root, "M3a Mirrored Double Sided Normal Probe", textured_mesh, mirrored_normal_material,
      glm::vec3(-0.6f, 1.55f, -1.55f), glm::radians(glm::vec3(8.0f, 18.0f, 0.0f)), glm::vec3(-0.55f, 0.32f, 1.0f));

  const auto instanced_entity = scene->CreateEntity("M3a Mixed Mirrored Instanced Probe");
  const auto particles = scene->GetOrSetPrivateComponent<Particles>(instanced_entity).lock();
  particles->mesh = textured_mesh;
  particles->material = textured_material;
  const auto particle_info_list = particles->particle_info_list.Get<ParticleInfoList>();
  std::vector<ParticleInfo> particle_infos(2);
  particle_infos[0].instance_matrix.SetValue(glm::vec3(-1.55f, 1.55f, -1.55f),
                                             glm::radians(glm::vec3(5.0f, 12.0f, 0.0f)), glm::vec3(0.38f, 0.25f, 1.0f));
  particle_infos[1].instance_matrix.SetValue(
      glm::vec3(1.55f, 1.55f, -1.55f), glm::radians(glm::vec3(-5.0f, -12.0f, 0.0f)), glm::vec3(-0.38f, 0.25f, 1.0f));
  particle_info_list->SetParticleInfos(particle_infos);
  particles->RecalculateBoundingBox();
  scene->SetParent(instanced_entity, root);

  const auto spec_gloss_mesh =
      CreateRenderingRegressionMaterialQuad({glm::vec4(1.0f, 0.45f, 0.45f, 1.0f), glm::vec4(0.45f, 1.0f, 0.45f, 1.0f),
                                             glm::vec4(0.45f, 0.55f, 1.0f, 1.0f), glm::vec4(1.0f, 0.85f, 0.45f, 1.0f)});
  const auto spec_gloss_material = AssetManager::CreateTemporaryAsset<Material>();
  auto& spec_gloss = spec_gloss_material->material_data.shade_material;
  spec_gloss.pbr_model = static_cast<int32_t>(GltfPbrModel::SpecularGlossiness);
  spec_gloss.pbr_diffuse_factor = glm::vec4(0.9f, 0.75f, 0.55f, 1.0f);
  spec_gloss.pbr_specular_factor = glm::vec3(0.78f, 0.16f, 0.06f);
  spec_gloss.pbr_glossiness_factor = 0.72f;
  spec_gloss_material->MarkDirty();
  CreateRenderingRegressionMaterialQuadEntity(
      scene, root, "M3a Specular Glossiness F0 Probe", spec_gloss_mesh, spec_gloss_material,
      glm::vec3(-0.55f, 0.85f, -1.35f), glm::radians(glm::vec3(-5.0f, -18.0f, 0.0f)), glm::vec3(0.75f, 0.48f, 1.0f));

  const auto opaque_mesh =
      CreateRenderingRegressionMaterialQuad({glm::vec4(0.1f, 1.0f, 0.3f, 0.0f), glm::vec4(0.1f, 1.0f, 0.3f, 0.0f),
                                             glm::vec4(0.1f, 1.0f, 0.3f, 0.0f), glm::vec4(0.1f, 1.0f, 0.3f, 0.0f)});
  const auto opaque_material = AssetManager::CreateTemporaryAsset<Material>();
  opaque_material->material_data.shade_material.pbr_base_color_factor = glm::vec4(1.0f, 0.8f, 0.25f, 0.0f);
  opaque_material->material_data.shade_material.alpha_mode = static_cast<int32_t>(GltfAlphaMode::Opaque);
  opaque_material->MarkDirty();
  CreateRenderingRegressionMaterialQuadEntity(scene, root, "M3a Opaque Ignores Alpha Probe", opaque_mesh,
                                              opaque_material, glm::vec3(0.65f, 0.85f, -1.35f), glm::vec3(0.0f),
                                              glm::vec3(0.75f, 0.48f, 1.0f));
  CreateRenderingRegressionMaterialQuadEntity(
      scene, root, "M3a Mirrored Single Sided Visibility Probe", opaque_mesh, opaque_material,
      glm::vec3(0.65f, 1.55f, -1.55f), glm::radians(glm::vec3(-5.0f, -18.0f, 0.0f)), glm::vec3(-0.55f, 0.32f, 1.0f));

  const auto alpha_mesh =
      CreateRenderingRegressionMaterialQuad({glm::vec4(1.0f, 0.3f, 0.1f, 0.0f), glm::vec4(1.0f, 0.3f, 0.1f, 1.0f),
                                             glm::vec4(0.2f, 0.55f, 1.0f, 1.0f), glm::vec4(0.2f, 0.55f, 1.0f, 0.0f)});
  const auto alpha_material = AssetManager::CreateTemporaryAsset<Material>();
  alpha_material->material_data.shade_material.alpha_mode = static_cast<int32_t>(GltfAlphaMode::Mask);
  alpha_material->material_data.shade_material.alpha_cutoff = 0.5f;
  alpha_material->MarkDirty();
  CreateRenderingRegressionMaterialQuadEntity(scene, root, "M3a Vertex Alpha Mask Probe", alpha_mesh, alpha_material,
                                              glm::vec3(1.85f, 0.85f, -1.35f), glm::vec3(0.0f),
                                              glm::vec3(0.75f, 0.48f, 1.0f));

  const auto blend_material = AssetManager::CreateTemporaryAsset<Material>();
  blend_material->material_data.shade_material.pbr_base_color_factor = glm::vec4(0.35f, 0.65f, 1.0f, 0.55f);
  blend_material->material_data.shade_material.alpha_mode = static_cast<int32_t>(GltfAlphaMode::Blend);
  blend_material->MarkDirty();
  CreateRenderingRegressionMaterialQuadEntity(scene, root, "M3a Vertex Alpha Blend Probe", alpha_mesh, blend_material,
                                              glm::vec3(2.75f, 0.85f, -1.35f), glm::vec3(0.0f),
                                              glm::vec3(0.55f, 0.48f, 1.0f));

  const auto sampler_pixels = std::vector{glm::vec4(1.0f, 0.05f, 0.02f, 1.0f), glm::vec4(0.02f, 0.1f, 1.0f, 1.0f),
                                          glm::vec4(0.02f, 1.0f, 0.08f, 1.0f), glm::vec4(1.0f, 0.8f, 0.02f, 1.0f)};
  const glm::mat3x2 out_of_range_transform(2.0f, 0.0f, 0.0f, 2.0f, -0.5f, -0.5f);
  const auto create_sampler_probe = [&](const std::string& name, const glm::vec3& position, const VkFilter filter,
                                        const VkSamplerAddressMode address_mode) {
    const auto texture = AssetManager::CreateTemporaryAsset<Texture2D>();
    texture->SetRgbaChannelData(sampler_pixels, glm::uvec2(2));
    SetRenderingRegressionSampler(texture, filter, address_mode);
    const auto material = AssetManager::CreateTemporaryAsset<Material>();
    const auto slot =
        material->SetTexture(&GltfShadeMaterial::pbr_base_color_texture, texture, 0, out_of_range_transform);
    material->material_data.texture_infos[slot].color_space = static_cast<int32_t>(GltfTextureColorSpace::Srgb);
    ConfigureMaterial(material, glm::vec3(1.0f), 0.65f, 0.0f);
    CreateRenderingRegressionMaterialQuadEntity(scene, root, name, textured_mesh, material, position, glm::vec3(0.0f),
                                                glm::vec3(0.55f, 0.34f, 1.0f));
  };
  create_sampler_probe("M7 Repeat Nearest Sampler Probe", glm::vec3(-1.55f, 2.45f, -1.4f), VK_FILTER_NEAREST,
                       VK_SAMPLER_ADDRESS_MODE_REPEAT);
  create_sampler_probe("M7 Clamp Linear Sampler Probe", glm::vec3(-0.55f, 2.45f, -1.4f), VK_FILTER_LINEAR,
                       VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE);

  const auto mip_texture = AssetManager::CreateTemporaryAsset<Texture2D>();
  mip_texture->srgb = true;
  std::vector<glm::vec4> mip_pixels(32 * 32);
  for (uint32_t y = 0; y < 32; ++y) {
    for (uint32_t x = 0; x < 32; ++x) {
      const float value = (x + y) % 2 == 0 ? 1.0f : 0.0f;
      mip_pixels[y * 32 + x] = glm::vec4(value, value, value, 1.0f);
    }
  }
  mip_texture->SetRgbaChannelData(mip_pixels, glm::uvec2(32));
  SetRenderingRegressionSampler(mip_texture, VK_FILTER_LINEAR, VK_SAMPLER_ADDRESS_MODE_REPEAT);
  const auto mip_material = AssetManager::CreateTemporaryAsset<Material>();
  const glm::mat3x2 minification_transform(32.0f, 0.0f, 0.0f, 32.0f, 0.0f, 0.0f);
  const auto mip_slot =
      mip_material->SetTexture(&GltfShadeMaterial::pbr_base_color_texture, mip_texture, 2, minification_transform);
  mip_material->material_data.texture_infos[mip_slot].color_space = static_cast<int32_t>(GltfTextureColorSpace::Linear);
  ConfigureMaterial(mip_material, glm::vec3(1.0f), 1.0f, 0.0f);
  mip_material->material_data.shade_material.unlit = 1;
  mip_material->MarkDirty();
  CreateRenderingRegressionMaterialQuadEntity(scene, root, "M9 Linear sRGB Mip Probe", textured_mesh, mip_material,
                                              glm::vec3(2.35f, 2.45f, -1.4f), glm::vec3(0.0f),
                                              glm::vec3(0.5f, 0.34f, 1.0f));

  const auto seam_normal_texture = AssetManager::CreateTemporaryAsset<Texture2D>();
  seam_normal_texture->SetRgbaChannelData({glm::vec4(0.75f, 0.5f, 0.9330127f, 1.0f)}, glm::uvec2(1));
  const auto seam_material = AssetManager::CreateTemporaryAsset<Material>();
  seam_material->SetTexture(&GltfShadeMaterial::normal_texture, seam_normal_texture);
  ConfigureMaterial(seam_material, glm::vec3(0.72f), 0.5f, 0.0f);
  CreateRenderingRegressionMaterialQuadEntity(
      scene, root, "M9 Mikk Mirrored UV Tangent Seam Probe", CreateRenderingRegressionMirroredTangentSeam(),
      seam_material, glm::vec3(1.15f, 2.45f, -1.4f), glm::vec3(0.0f), glm::vec3(0.5f, 0.34f, 1.0f));
}

void ConfigureRenderingRegressionAdvancedRayMaterialProbes(const std::shared_ptr<Scene>& scene, const Entity& root) {
  const auto& primitives = Resources::GetInstance().GetPrimitives();
  const auto create_probe = [&](const std::string& name, const glm::vec3& position, const glm::vec3& color,
                                const float roughness, const float metallic) {
    const auto entity = CreateRenderingRegressionProbe(scene, root, name, primitives.sphere, position, glm::vec3(0.16f),
                                                       color, roughness, metallic);
    return scene->GetOrSetPrivateComponent<MeshRenderer>(entity).lock()->material.Get<Material>();
  };

  const glm::mat3x2 data_texture_transform(0.75f, 0.0f, 0.0f, 1.25f, 0.125f, 0.25f);
  const auto iridescence_data_texture = AssetManager::CreateTemporaryAsset<Texture2D>();
  iridescence_data_texture->SetRgbaChannelData({glm::vec4(1.0f, 1.0f, 0.17f, 0.0f)}, glm::uvec2(1));
  const auto anisotropy_data_texture = AssetManager::CreateTemporaryAsset<Texture2D>();
  anisotropy_data_texture->SetRgbaChannelData({glm::vec4(1.0f, 0.5f, 1.0f, 0.0f)}, glm::uvec2(1));
  const auto retroreflection_data_texture = AssetManager::CreateTemporaryAsset<Texture2D>();
  retroreflection_data_texture->SetRgbaChannelData({glm::vec4(1.0f, 0.07f, 0.83f, 0.0f)}, glm::uvec2(1));

  auto iridescence =
      create_probe("M3b Iridescence Colored F0 Probe", glm::vec3(2.0f, 1.75f, -1.4f), glm::vec3(0.62f), 0.16f, 0.0f);
  iridescence->material_data.shade_material.iridescence_factor = 1.0f;
  iridescence->material_data.shade_material.iridescence_ior = 1.3f;
  iridescence->material_data.shade_material.iridescence_thickness_minimum = 100.0f;
  iridescence->material_data.shade_material.iridescence_thickness_maximum = 400.0f;
  iridescence->material_data.shade_material.specular_color_factor = glm::vec3(0.55f, 0.85f, 1.0f);
  iridescence->SetTexture(&GltfShadeMaterial::iridescence_texture, iridescence_data_texture, 1, data_texture_transform);
  iridescence->SetTexture(&GltfShadeMaterial::iridescence_thickness_texture, iridescence_data_texture, 1,
                          data_texture_transform);
  iridescence->MarkDirty();

  auto anisotropy_zero = create_probe("M3b Anisotropy Rotation 0 Probe", glm::vec3(-2.0f, 1.75f, -1.4f),
                                      glm::vec3(0.95f, 0.72f, 0.25f), 0.35f, 1.0f);
  anisotropy_zero->material_data.shade_material.anisotropy_strength = 1.0f;
  anisotropy_zero->material_data.shade_material.anisotropy_rotation = glm::vec2(1.0f, 0.0f);
  anisotropy_zero->SetTexture(&GltfShadeMaterial::anisotropy_texture, anisotropy_data_texture, 1,
                              data_texture_transform);
  anisotropy_zero->MarkDirty();

  auto anisotropy_ninety = create_probe("M3b Anisotropy Rotation 90 CCW Probe", glm::vec3(-1.6f, 1.75f, -1.4f),
                                        glm::vec3(0.95f, 0.72f, 0.25f), 0.35f, 1.0f);
  anisotropy_ninety->material_data.shade_material.anisotropy_strength = 1.0f;
  anisotropy_ninety->material_data.shade_material.anisotropy_rotation = glm::vec2(0.0f, 1.0f);
  anisotropy_ninety->SetTexture(&GltfShadeMaterial::anisotropy_texture, anisotropy_data_texture, 1,
                                data_texture_transform);
  anisotropy_ninety->MarkDirty();

  const auto configure_dispersion = [](const std::shared_ptr<Material>& material, const float dispersion) {
    auto& shade = material->material_data.shade_material;
    shade.pbr_base_color_factor = glm::vec4(1.0f);
    shade.pbr_roughness_factor = 0.025f;
    shade.pbr_metallic_factor = 0.0f;
    shade.transmission_factor = 1.0f;
    shade.thickness_factor = 1.0f;
    shade.ior = 1.5f;
    shade.dispersion = dispersion;
    material->MarkDirty();
  };
  auto dispersion_off =
      create_probe("M3b Dispersion 0 Probe", glm::vec3(-1.2f, 1.75f, -1.4f), glm::vec3(1.0f), 0.025f, 0.0f);
  auto dispersion_on =
      create_probe("M3b Dispersion 1 Probe", glm::vec3(-0.8f, 1.75f, -1.4f), glm::vec3(1.0f), 0.025f, 0.0f);
  configure_dispersion(dispersion_off, 0.0f);
  configure_dispersion(dispersion_on, 1.0f);

  const auto configure_retroreflection = [](const std::shared_ptr<Material>& material, const float factor) {
    auto& shade = material->material_data.shade_material;
    shade.retroreflection_factor = factor;
    material->MarkDirty();
  };
  auto retroreflection_off = create_probe("M3b Retroreflection 0 Probe", glm::vec3(-0.4f, 1.75f, -1.4f),
                                          glm::vec3(0.78f, 0.82f, 0.9f), 0.48f, 1.0f);
  auto retroreflection_half = create_probe("M3b Retroreflection 0.5 Energy Probe", glm::vec3(0.0f, 1.75f, -1.4f),
                                           glm::vec3(0.78f, 0.82f, 0.9f), 0.48f, 1.0f);
  auto retroreflection_full = create_probe("M3b Retroreflection 1 Probe", glm::vec3(0.4f, 1.75f, -1.4f),
                                           glm::vec3(0.78f, 0.82f, 0.9f), 0.48f, 1.0f);
  for (const auto& material : {retroreflection_off, retroreflection_half, retroreflection_full}) {
    material->SetTexture(&GltfShadeMaterial::retroreflection_texture, retroreflection_data_texture, 1,
                         data_texture_transform);
  }
  configure_retroreflection(retroreflection_off, 0.0f);
  configure_retroreflection(retroreflection_half, 0.5f);
  configure_retroreflection(retroreflection_full, 1.0f);

  auto zero_specular = create_probe("M3b Explicit Specular Factor 0 Probe", glm::vec3(0.8f, 1.75f, -1.4f),
                                    glm::vec3(0.72f, 0.28f, 0.16f), 0.12f, 0.0f);
  zero_specular->material_data.shade_material.specular_factor = 0.0f;
  zero_specular->MarkDirty();

  auto half_specular = create_probe("M3b Specular Factor 0.5 F90 Probe", glm::vec3(1.2f, 1.75f, -1.4f),
                                    glm::vec3(0.72f, 0.28f, 0.16f), 0.12f, 0.0f);
  half_specular->material_data.shade_material.specular_factor = 0.5f;
  half_specular->material_data.shade_material.specular_color_factor = glm::vec3(2.0f, 1.0f, 0.5f);
  half_specular->MarkDirty();

  const auto unlit_entity = CreateRenderingRegressionProbe(
      scene, root, "M3b Unlit Ignores Emissive Probe", primitives.cube, glm::vec3(1.6f, 1.75f, -1.4f), glm::vec3(0.15f),
      glm::vec3(0.12f, 0.82f, 0.34f), 0.7f, 0.0f);
  const auto unlit = scene->GetOrSetPrivateComponent<MeshRenderer>(unlit_entity).lock()->material.Get<Material>();
  unlit->material_data.shade_material.unlit = 1;
  unlit->material_data.shade_material.emissive_factor = glm::vec3(8.0f, 0.0f, 0.0f);
  unlit->MarkDirty();

  const auto clearcoat_normal_texture = AssetManager::CreateTemporaryAsset<Texture2D>();
  clearcoat_normal_texture->SetRgbaChannelData({glm::vec4(0.8f, 0.35f, 0.9f, 1.0f)}, glm::uvec2(1));
  const auto configure_clearcoat_normal = [&](const std::shared_ptr<Material>& material, const float scale) {
    auto& shade = material->material_data.shade_material;
    shade.clearcoat_factor = 1.0f;
    shade.clearcoat_roughness = 0.12f;
    shade.clearcoat_normal_texture_scale = scale;
    shade.pbr_base_color_factor = glm::vec4(0.0f, 0.0f, 0.0f, 1.0f);
    shade.specular_factor = 0.0f;
    shade.emissive_factor = glm::vec3(12.0f, 4.0f, 1.0f);
    shade.alpha_mode = static_cast<int32_t>(GltfAlphaMode::Blend);
    material->SetTexture(&GltfShadeMaterial::clearcoat_normal_texture, clearcoat_normal_texture);
    material->MarkDirty();
  };
  auto clearcoat_normal_zero = create_probe("M8 Clearcoat Normal Scale 0 Probe", glm::vec3(-2.0f, 2.15f, -1.4f),
                                            glm::vec3(0.64f, 0.72f, 0.9f), 0.28f, 0.0f);
  auto clearcoat_normal_one = create_probe("M8 Clearcoat Normal Scale 1 Probe", glm::vec3(-1.6f, 2.15f, -1.4f),
                                           glm::vec3(0.64f, 0.72f, 0.9f), 0.28f, 0.0f);
  configure_clearcoat_normal(clearcoat_normal_zero, 0.0f);
  configure_clearcoat_normal(clearcoat_normal_one, 1.0f);

  const auto configure_coated_emitter = [](const std::shared_ptr<Material>& material, const float clearcoat) {
    auto& shade = material->material_data.shade_material;
    shade.pbr_base_color_factor = glm::vec4(0.0f, 0.0f, 0.0f, 1.0f);
    shade.specular_factor = 0.0f;
    shade.emissive_factor = glm::vec3(12.0f, 4.0f, 1.0f);
    shade.clearcoat_factor = clearcoat;
    shade.clearcoat_roughness = 0.08f;
    material->MarkDirty();
  };
  auto uncoated_emitter =
      create_probe("M8 Uncoated Emission Probe", glm::vec3(-1.2f, 2.15f, -1.4f), glm::vec3(0.4f), 0.3f, 0.0f);
  auto coated_emitter =
      create_probe("M8 Coated Emission Probe", glm::vec3(-0.8f, 2.15f, -1.4f), glm::vec3(0.4f), 0.3f, 0.0f);
  configure_coated_emitter(uncoated_emitter, 0.0f);
  configure_coated_emitter(coated_emitter, 1.0f);

  auto half_specular_iridescence = create_probe("M8 Specular 0.5 Iridescence F90 Probe", glm::vec3(-0.4f, 2.15f, -1.4f),
                                                glm::vec3(0.72f, 0.28f, 0.16f), 0.12f, 0.0f);
  half_specular_iridescence->material_data.shade_material.specular_factor = 0.5f;
  half_specular_iridescence->material_data.shade_material.specular_color_factor = glm::vec3(2.0f, 1.0f, 0.5f);
  half_specular_iridescence->material_data.shade_material.iridescence_factor = 1.0f;
  half_specular_iridescence->material_data.shade_material.iridescence_thickness_minimum = 350.0f;
  half_specular_iridescence->material_data.shade_material.iridescence_thickness_maximum = 350.0f;
  half_specular_iridescence->MarkDirty();

  auto half_specular_control = create_probe("M8 Specular 0.5 Iridescence Control Probe", glm::vec3(0.0f, 2.15f, -1.4f),
                                            glm::vec3(0.72f, 0.28f, 0.16f), 0.12f, 0.0f);
  half_specular_control->material_data.shade_material.specular_factor = 0.5f;
  half_specular_control->material_data.shade_material.specular_color_factor = glm::vec3(2.0f, 1.0f, 0.5f);
  half_specular_control->MarkDirty();

  CreateRenderingRegressionProbe(scene, root, "M3b Dispersion Backdrop Red", primitives.cube,
                                 glm::vec3(-1.3f, 1.75f, -2.2f), glm::vec3(0.06f, 0.3f, 0.04f),
                                 glm::vec3(1.0f, 0.03f, 0.03f), 0.8f, 0.0f, 4.0f, false);
  CreateRenderingRegressionProbe(scene, root, "M3b Dispersion Backdrop Blue", primitives.cube,
                                 glm::vec3(-0.7f, 1.75f, -2.2f), glm::vec3(0.06f, 0.3f, 0.04f),
                                 glm::vec3(0.03f, 0.12f, 1.0f), 0.8f, 0.0f, 4.0f, false);
}

void ConfigureRenderingRegressionEmissiveNeeProbes(const std::shared_ptr<Scene>& scene, const Entity& root) {
  const auto cube = Resources::GetInstance().GetPrimitives().cube;
  CreateRenderingRegressionProbe(scene, root, "M4 Emissive NEE Receiver Floor", cube, glm::vec3(0.0f, 3.75f, -2.4f),
                                 glm::vec3(2.0f, 0.05f, 1.5f), glm::vec3(0.72f), 0.9f, 0.0f);
  CreateRenderingRegressionProbe(scene, root, "M4 Emissive NEE Receiver Back Wall", cube, glm::vec3(0.0f, 4.4f, -3.85f),
                                 glm::vec3(2.0f, 0.7f, 0.05f), glm::vec3(0.62f, 0.66f, 0.72f), 0.9f, 0.0f);

  const auto constant_emitter = CreateRenderingRegressionProbe(
      scene, root, "M4 Emissive NEE Constant Emitter", cube, glm::vec3(-0.72f, 5.15f, -2.4f),
      glm::vec3(0.22f, 0.025f, 0.22f), glm::vec3(1.0f, 0.48f, 0.14f), 0.8f, 0.0f, 50.0f);
  const auto constant_material =
      scene->GetOrSetPrivateComponent<MeshRenderer>(constant_emitter).lock()->material.Get<Material>();
  constant_material->material_data.shade_material.double_sided = 0;
  constant_material->MarkDirty();

  const auto textured_emitter = CreateRenderingRegressionProbe(
      scene, root, "M4 Emissive NEE Textured Emitter", cube, glm::vec3(0.72f, 5.15f, -2.4f),
      glm::vec3(0.22f, 0.025f, 0.22f), glm::vec3(0.24f, 0.62f, 1.0f), 0.8f, 0.0f, 50.0f);
  const auto textured_material =
      scene->GetOrSetPrivateComponent<MeshRenderer>(textured_emitter).lock()->material.Get<Material>();
  const auto emissive_texture = AssetManager::CreateTemporaryAsset<Texture2D>();
  constexpr uint32_t kEmissiveTextureResolution = 32;
  const std::array emissive_palette = {glm::vec4(1.0f, 0.18f, 0.04f, 1.0f), glm::vec4(0.04f, 0.25f, 1.0f, 1.0f),
                                       glm::vec4(0.05f, 1.0f, 0.22f, 1.0f), glm::vec4(1.0f, 0.85f, 0.08f, 1.0f)};
  std::vector<glm::vec4> emissive_pixels;
  emissive_pixels.reserve(kEmissiveTextureResolution * kEmissiveTextureResolution);
  for (uint32_t y = 0; y < kEmissiveTextureResolution; ++y) {
    for (uint32_t x = 0; x < kEmissiveTextureResolution; ++x) {
      emissive_pixels.emplace_back(emissive_palette[(x + y * 3u) % emissive_palette.size()]);
    }
  }
  emissive_texture->SetRgbaChannelData(emissive_pixels, glm::uvec2(kEmissiveTextureResolution));
  const float texture_rotation = glm::radians(22.5f);
  const glm::mat3x2 texture_transform(1.25f * std::cos(texture_rotation), 1.25f * std::sin(texture_rotation),
                                      -0.8f * std::sin(texture_rotation), 0.8f * std::cos(texture_rotation), 0.1f,
                                      0.15f);
  const auto emissive_slot =
      textured_material->SetTexture(&GltfShadeMaterial::emissive_texture, emissive_texture, 1, texture_transform);
  textured_material->material_data.texture_infos[emissive_slot].color_space =
      static_cast<int32_t>(GltfTextureColorSpace::Srgb);
  textured_material->MarkDirty();
}

void ConfigureRenderingRegressionCamera(const std::shared_ptr<Scene>& scene) {
  const glm::vec3 camera_position(0.0f, 1.15f, 5.6f);
  const glm::vec3 camera_target(0.0f, 0.35f, -2.4f);
  const auto camera_rotation = glm::quatLookAt(glm::normalize(camera_target - camera_position), glm::vec3(0, 1, 0));
  const auto configure_post_processing = [](const std::shared_ptr<Camera>& camera) {
    ConfigureBistroCameraPostProcessing(camera);
    if (const auto stack = camera->post_processing_stack_ref.Get<PostProcessingStack>(); stack && stack->tone_mapping) {
      stack->tone_mapping->auto_exposure = false;
      stack->tone_mapping->exposure = 1.0f;
      stack->tone_mapping->dither = false;
    }
  };
  if (const auto main_camera = scene->main_camera.Get<Camera>()) {
    main_camera->Resize({1920, 1080});
    main_camera->skybox.Clear();
    main_camera->camera_render_mode = Camera::CameraRenderMode::Rasterization;
    main_camera->camera_settings.background_source = Camera::BackgroundSource::ClearColor;
    main_camera->camera_settings.clear_color = glm::vec4(0.0f, 0.0f, 0.0f, 1.0f);
    main_camera->camera_settings.background_intensity = 0.0f;
    main_camera->camera_settings.fov = 55.0f;
    main_camera->camera_settings.near_distance = 0.05f;
    main_camera->camera_settings.far_distance = 250.0f;
    main_camera->camera_settings.sample_size = 4;
    main_camera->camera_settings.bounce = 5;
    main_camera->camera_settings.firefly_clamp_threshold = 10.0f;
    main_camera->camera_settings.auto_spp_enabled = false;
    main_camera->camera_settings.auto_spp_min_samples = 4;
    main_camera->camera_settings.auto_spp_max_samples = 32;
    main_camera->camera_settings.auto_spp_convergence_threshold = 0.01f;
    main_camera->post_processing_stack_ref = AssetManager::CreateTemporaryAsset<PostProcessingStack>();
    configure_post_processing(main_camera);
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
      scene_camera->skybox.Clear();
      scene_camera->camera_settings.background_source = Camera::BackgroundSource::ClearColor;
      scene_camera->camera_settings.clear_color = glm::vec4(0.0f, 0.0f, 0.0f, 1.0f);
      scene_camera->camera_settings.background_intensity = 0.0f;
      scene_camera->camera_settings.fov = 55.0f;
      scene_camera->camera_settings.near_distance = 0.05f;
      scene_camera->camera_settings.far_distance = 250.0f;
      scene_camera->camera_settings.sample_size = 4;
      scene_camera->camera_settings.bounce = 5;
      scene_camera->camera_settings.firefly_clamp_threshold = 10.0f;
      scene_camera->camera_settings.auto_spp_enabled = false;
      scene_camera->camera_settings.auto_spp_min_samples = 4;
      scene_camera->camera_settings.auto_spp_max_samples = 32;
      scene_camera->camera_settings.auto_spp_convergence_threshold = 0.01f;
      configure_post_processing(scene_camera);
      scene_camera->ResetFrameCount();
    }
  }
  SyncTemporaryEnvironmentalLightingSettingsFromScene(scene);
}

Entity ConfigureRenderingRegressionLights(const std::shared_ptr<Scene>& scene, const Entity& root) {
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
                                                          glm::vec3(2.9f, 1.2f, -0.45f), glm::vec3(0.18f),
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

  const auto retroreflection_light_entity = scene->CreateEntity("M3b Retroreflection Camera Light");
  const auto retroreflection_light = scene->GetOrSetPrivateComponent<PointLight>(retroreflection_light_entity).lock();
  retroreflection_light->diffuse = glm::vec3(1.0f);
  retroreflection_light->diffuse_brightness = 5.0f;
  retroreflection_light->range = 16.0f;
  retroreflection_light->light_size = 0.02f;
  retroreflection_light->constant = 1.0f;
  retroreflection_light->linear = 0.08f;
  retroreflection_light->quadratic = 0.02f;
  Transform retroreflection_light_transform;
  retroreflection_light_transform.SetPosition(glm::vec3(0.0f, 1.15f, 5.2f));
  scene->SetDataComponent(retroreflection_light_entity, retroreflection_light_transform);
  scene->SetParent(retroreflection_light_entity, root);
  return point_entity;
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

void ApplyBistroParityRendererState(const std::shared_ptr<Scene>& scene) {
  if (!scene) {
    return;
  }
  if (const auto lighting = scene->environmental_lighting.Get<EnvironmentalLighting>()) {
    lighting->ddgi_settings.runtime.enabled = false;
    DisableDdgiDebugVisualization();
  }
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

void ConfigureBistroRasterizationPostProcessing(const std::shared_ptr<Camera>& camera) {
  if (!camera) {
    return;
  }
  camera->post_processing_stack_ref = AssetManager::CreateTemporaryAsset<PostProcessingStack>();
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
  tone_mapping.auto_exposure_speed = 10.0f;
  tone_mapping.ev_min_value = -20.0f;
  tone_mapping.ev_max_value = 20.0f;
  tone_mapping.enable_center_metering = false;
  tone_mapping.center_metering_size = 0.5f;
  tone_mapping.average_mode = 1;
  tone_mapping.dither = true;
}

void DisableImportedLightsRecursive(const std::shared_ptr<Scene>& scene, Entity entity);

glm::mat4 MakeAuthoringTransform(const glm::vec3& position, const glm::vec3& rotation = glm::vec3(0.0f),
                                 const glm::vec3& scale = glm::vec3(1.0f)) {
  Transform transform;
  transform.SetValue(position, rotation, scale);
  return transform.value;
}

uint64_t StableEnvironmentalLightingId(const std::string& name) {
  uint64_t hash = 1469598103934665603ull;
  for (const char value : name) {
    hash ^= static_cast<uint8_t>(value);
    hash *= 1099511628211ull;
  }
  return hash == 0u ? 1u : hash;
}

std::shared_ptr<EnvironmentalLighting> GetOrCreateTemporaryEnvironmentalLighting(const std::shared_ptr<Scene>& scene) {
  if (!scene) {
    return {};
  }
  auto lighting = scene->environmental_lighting.Get<EnvironmentalLighting>();
  if (!lighting || !lighting->IsTemporary()) {
    lighting = AssetManager::CreateTemporaryAsset<EnvironmentalLighting>();
    scene->environmental_lighting = lighting;
  }
  return lighting;
}

void ConfigureEnvironmentalLightingColorSource(EnvironmentalLighting& lighting, const glm::vec3& color,
                                               const float environment_lighting_intensity,
                                               const float diffuse_fallback_intensity,
                                               const float specular_fallback_intensity = 1.0f, const float gamma = 2.2f,
                                               const float rotation = 0.0f) {
  lighting.indirect_environment_source = {};
  lighting.indirect_environment_source.kind = EnvironmentalLighting::IndirectEnvironmentSourceKind::Color;
  lighting.indirect_environment_source.color = color;
  lighting.indirect_environment_source.gamma = glm::max(gamma, 0.0f);
  lighting.indirect_environment_source.rotation = rotation;
  lighting.environment_lighting_intensity = glm::max(environment_lighting_intensity, 0.0f);
  lighting.diffuse_fallback_intensity = glm::max(diffuse_fallback_intensity, 0.0f);
  lighting.specular_fallback_intensity = glm::max(specular_fallback_intensity, 0.0f);
}

void ConfigureEnvironmentalLightingMapSource(EnvironmentalLighting& lighting,
                                             const std::shared_ptr<EnvironmentalMap>& environmental_map,
                                             const float environment_lighting_intensity,
                                             const float diffuse_fallback_intensity,
                                             const float specular_fallback_intensity = 1.0f,
                                             const glm::vec3& color = glm::vec3(0.0f), const float gamma = 2.2f,
                                             const float rotation = 0.0f) {
  lighting.indirect_environment_source = {};
  lighting.indirect_environment_source.kind = EnvironmentalLighting::IndirectEnvironmentSourceKind::EnvironmentalMap;
  lighting.indirect_environment_source.environmental_map = environmental_map;
  lighting.indirect_environment_source.color = color;
  lighting.indirect_environment_source.gamma = glm::max(gamma, 0.0f);
  lighting.indirect_environment_source.rotation = rotation;
  if (lighting.indirect_environment_source.environmental_map.GetAssetHandle().GetValue() == 0u) {
    lighting.indirect_environment_source.kind = EnvironmentalLighting::IndirectEnvironmentSourceKind::EngineDefault;
  }
  lighting.environment_lighting_intensity = glm::max(environment_lighting_intensity, 0.0f);
  lighting.diffuse_fallback_intensity = glm::max(diffuse_fallback_intensity, 0.0f);
  lighting.specular_fallback_intensity = glm::max(specular_fallback_intensity, 0.0f);
}

void SetEnvironmentalLightingIntensity(const std::shared_ptr<Scene>& scene, const float intensity) {
  if (const auto lighting = GetOrCreateTemporaryEnvironmentalLighting(scene)) {
    lighting->environment_lighting_intensity = glm::max(intensity, 0.0f);
  }
}

void SetEnvironmentalLightingDiffuseFallback(const std::shared_ptr<Scene>& scene, const float intensity) {
  if (const auto lighting = GetOrCreateTemporaryEnvironmentalLighting(scene)) {
    lighting->diffuse_fallback_intensity = glm::max(intensity, 0.0f);
  }
}

DdgiSettings& RequireEnvironmentalLightingDdgiSettings(const std::shared_ptr<Scene>& scene) {
  const auto lighting = GetOrCreateTemporaryEnvironmentalLighting(scene);
  if (!lighting) {
    throw std::runtime_error("EnvironmentalLighting asset is required for DDGI settings.");
  }
  return lighting->ddgi_settings;
}

void SetEnvironmentalLightingFallbackIntensities(EnvironmentalLighting& lighting, const float diffuse,
                                                 const float specular) {
  lighting.diffuse_fallback_intensity = glm::max(diffuse, 0.0f);
  lighting.specular_fallback_intensity = glm::max(specular, 0.0f);
}

void ConfigureStandardDdgiRuntime(DdgiSettings& settings, const int max_probe_count, const float normal_bias,
                                  const float visibility_moment_bias = 0.02f) {
  settings.runtime.enabled = true;
  SetDdgiUpdatesPaused(false);
  settings.runtime.enable_emissive_mesh_sampling = true;
  settings.runtime.ray_count = 192;
  settings.runtime.emissive_ray_count = 64;
  settings.runtime.normal_bias = normal_bias;
  settings.runtime.visibility_moment_bias = visibility_moment_bias;
  settings.storage.max_probe_count = max_probe_count;
  if (const auto render_layer = ApplicationContext::Get().GetLayer<RenderLayer>()) {
    auto& session = render_layer->GetDdgiSessionState();
    session.show_probes = true;
    session.show_selected_probe = true;
  }
}

EnvironmentalLighting::DdgiVolume& AddEnvironmentalLightingDdgiVolume(
    EnvironmentalLighting& lighting, const std::string& name, const glm::mat4& transform,
    const glm::ivec3& probe_counts, const glm::vec3& probe_spacing, const glm::vec3& volume_origin,
    const int artist_priority = 0) {
  auto& volume = lighting.GetOrCreateDdgiVolumePack()->volumes.emplace_back();
  volume.name = name;
  volume.stable_id = StableEnvironmentalLightingId(name);
  volume.transform = transform;
  volume.probe_counts = probe_counts;
  volume.probe_spacing = probe_spacing;
  volume.volume_origin = volume_origin;
  volume.artist_priority = artist_priority;
  return volume;
}

EnvironmentalLighting::DdgiVolume& ResetEnvironmentalLightingDdgiVolume(
    EnvironmentalLighting& lighting, const std::string& name, const glm::vec3& position, const glm::ivec3& probe_counts,
    const glm::vec3& probe_spacing, const glm::vec3& volume_origin, const int artist_priority = 0) {
  Transform transform;
  transform.SetPosition(position);
  lighting.GetOrCreateDdgiVolumePack()->volumes.clear();
  return AddEnvironmentalLightingDdgiVolume(lighting, name, transform.value, probe_counts, probe_spacing, volume_origin,
                                            artist_priority);
}

EnvironmentalLighting::DdgiVolume* FindEnvironmentalLightingDdgiVolume(const std::shared_ptr<Scene>& scene,
                                                                       const std::string& name) {
  const auto lighting = scene ? scene->environmental_lighting.Get<EnvironmentalLighting>() : nullptr;
  if (!lighting) {
    return nullptr;
  }
  const auto pack = lighting->GetDdgiVolumePack();
  if (!pack)
    return nullptr;
  for (auto& volume : pack->volumes) {
    if (volume.name == name) {
      return &volume;
    }
  }
  return nullptr;
}

void SyncTemporaryEnvironmentalLightingSettingsFromScene(const std::shared_ptr<Scene>& scene) {
  (void)GetOrCreateTemporaryEnvironmentalLighting(scene);
}

void ConfigureDdgiValidationVolume(EnvironmentalLighting::DdgiVolume& volume, const std::string& fixture_id,
                                   const float probe_variability_threshold) {
  volume.probe_counts = {8, 6, 8};
  volume.probe_spacing = glm::vec3(0.6f);
  volume.volume_origin = glm::vec3(0.0f);
  volume.movement_type = fixture_id == "scrolling" ? static_cast<int>(DdgiVolumeMovementType::Scrolling)
                                                   : static_cast<int>(DdgiVolumeMovementType::Default);
  volume.relocation_distance = 0.2f;
  volume.enable_probe_relocation = true;
  volume.enable_probe_classification = true;
  volume.enable_probe_variability = true;
  volume.enable_probe_variability_gating = true;
  volume.probe_variability_threshold = probe_variability_threshold;
  volume.probe_variability_min_samples = kDdgiValidationProbeVariabilityMinSamples;
}

EnvironmentalLighting::LocalReflectionProbe& AddEnvironmentalLightingLocalReflectionProbe(
    EnvironmentalLighting& lighting, const std::string& name, const glm::mat4& transform,
    const std::shared_ptr<GlobalReflectionProbe>& asset, const int priority,
    const EnvironmentalLighting::LocalReflectionProbeShape shape, const float sphere_radius, const float blend_distance,
    const bool box_projection = false) {
  auto& probe = lighting.GetOrCreateReflectionProbePack()->probes.emplace_back();
  probe.name = name;
  probe.stable_id = StableEnvironmentalLightingId(name);
  probe.transform = transform;
  probe.payload = asset;
  probe.artist_priority = priority;
  probe.shape = static_cast<int>(shape);
  probe.box_projection_extents = glm::vec3(0.5f);
  probe.sphere_radius = sphere_radius;
  probe.blend_distance = blend_distance;
  probe.reflection_intensity = 1.0f;
  probe.box_projection = box_projection;
  return probe;
}

EnvironmentalLighting::LocalReflectionProbe& AddReflectionProbeValidationLocalProbe(
    EnvironmentalLighting& lighting, const std::string& name, const glm::vec3& position,
    const std::shared_ptr<GlobalReflectionProbe>& asset, const int priority,
    const EnvironmentalLighting::LocalReflectionProbeShape shape, const glm::vec3& box_size, const float sphere_radius,
    const float blend_distance, const bool box_projection = false, const glm::vec3& rotation = glm::vec3(0.0f),
    const glm::vec3& scale = glm::vec3(1.0f)) {
  const auto influence_scale =
      shape == EnvironmentalLighting::LocalReflectionProbeShape::Box ? scale * box_size : scale;
  return AddEnvironmentalLightingLocalReflectionProbe(
      lighting, name, MakeAuthoringTransform(position, rotation, influence_scale), asset, priority, shape,
      sphere_radius, blend_distance, box_projection);
}

EnvironmentalLighting::LocalReflectionProbe* FindEnvironmentalLightingLocalReflectionProbe(
    const std::shared_ptr<Scene>& scene, const std::string& name) {
  const auto lighting = scene ? scene->environmental_lighting.Get<EnvironmentalLighting>() : nullptr;
  if (!lighting) {
    return nullptr;
  }
  const auto pack = lighting->GetReflectionProbePack();
  if (!pack)
    return nullptr;
  for (auto& probe : pack->probes) {
    if (probe.name == name) {
      return &probe;
    }
  }
  return nullptr;
}

EnvironmentalLighting::LocalReflectionProbe& RequireEnvironmentalLightingLocalReflectionProbe(
    EnvironmentalLighting& lighting, const std::string& name) {
  const auto pack = lighting.GetReflectionProbePack();
  if (!pack)
    throw std::runtime_error("Missing environmental lighting reflection probe pack.");
  for (auto& probe : pack->probes) {
    if (probe.name == name) {
      return probe;
    }
  }
  throw std::runtime_error("Missing environmental lighting local reflection probe: " + name);
}

const EnvironmentalLighting::LocalReflectionProbe& RequireEnvironmentalLightingLocalReflectionProbe(
    const EnvironmentalLighting& lighting, const std::string& name) {
  const auto pack = lighting.GetReflectionProbePack();
  if (!pack)
    throw std::runtime_error("Missing environmental lighting reflection probe pack.");
  for (const auto& probe : pack->probes) {
    if (probe.name == name) {
      return probe;
    }
  }
  throw std::runtime_error("Missing environmental lighting local reflection probe: " + name);
}

void ClampEnvironmentalLightingLocalReflectionProbe(EnvironmentalLighting::LocalReflectionProbe& probe) {
  const auto finite_or = [](const float value, const float fallback) {
    return std::isfinite(value) ? value : fallback;
  };
  const auto finite_vec3_or = [&](const glm::vec3& value, const glm::vec3& fallback) {
    return glm::vec3(finite_or(value.x, fallback.x), finite_or(value.y, fallback.y), finite_or(value.z, fallback.z));
  };
  constexpr float minimum_extent = 0.001f;
  probe.artist_priority =
      glm::clamp(probe.artist_priority, -EnvironmentalLighting::kMaxExactLocalReflectionProbePriority,
                 EnvironmentalLighting::kMaxExactLocalReflectionProbePriority);
  probe.shape = glm::clamp(probe.shape, static_cast<int>(EnvironmentalLighting::LocalReflectionProbeShape::Box),
                           static_cast<int>(EnvironmentalLighting::LocalReflectionProbeShape::Sphere));
  probe.box_projection_extents =
      glm::max(finite_vec3_or(probe.box_projection_extents, glm::vec3(0.5f)), glm::vec3(minimum_extent));
  probe.sphere_radius = glm::max(finite_or(probe.sphere_radius, 5.0f), minimum_extent);
  probe.reflection_intensity = glm::max(finite_or(probe.reflection_intensity, 1.0f), 0.0f);
  probe.blend_distance = glm::max(finite_or(probe.blend_distance, 1.0f), 0.0f);
  const float maximum_blend = probe.shape == static_cast<int>(EnvironmentalLighting::LocalReflectionProbeShape::Box)
                                  ? 0.5f
                                  : probe.sphere_radius;
  probe.blend_distance = glm::min(probe.blend_distance, maximum_blend);
}

std::string GetEnvironmentalLightingLocalProbeBakeStatus(const EnvironmentalLighting::LocalReflectionProbe& probe) {
  const auto& payload = probe.payload;
  if (!payload) {
    return "Missing GlobalReflectionProbe";
  }
  if (payload->GetSourceKind() != GlobalReflectionProbe::SourceKind::Baked) {
    return payload->GetSourceKind() == GlobalReflectionProbe::SourceKind::Imported ? "Imported" : "Not baked";
  }
  return payload->IsRuntimeReady() ? "Ready" : "Not ready";
}

void RunEnvironmentalLightingLocalProbeBake(const std::shared_ptr<Scene>& scene, RenderLayer& render_layer,
                                            const EnvironmentalLighting::LocalReflectionProbe& probe,
                                            const char* context) {
  const auto lighting = scene ? scene->environmental_lighting.Get<EnvironmentalLighting>() : nullptr;
  const auto pack = lighting ? lighting->GetReflectionProbePack() : nullptr;
  const auto& payload = probe.payload;
  if (!payload || !pack ||
      render_layer.QueueGlobalReflectionProbeBakeBatch(
          scene, {{glm::vec3(probe.transform[3]), payload, pack, probe.stable_id}}) != 1u) {
    throw std::runtime_error(std::string(context) + " could not queue its reflection probe bake.");
  }
  for (size_t frame = 0; frame < 600u; ++frame) {
    if (!ApplicationContext::Get().Loop()) {
      throw std::runtime_error(std::string("Application ended during ") + context + " reflection probe baking.");
    }
    if (!render_layer.IsGlobalReflectionProbeBakePending(payload) &&
        GetEnvironmentalLightingLocalProbeBakeStatus(probe) == "Ready" && payload->IsRuntimeReady()) {
      return;
    }
  }
  throw std::runtime_error(std::string(context) + " reflection probe bake timed out.");
}

uint32_t RunEnvironmentalLightingLocalProbeBakeBatch(
    const std::shared_ptr<Scene>& scene, RenderLayer& render_layer,
    const std::vector<const EnvironmentalLighting::LocalReflectionProbe*>& probes, const char* context) {
  std::vector<RenderLayer::ReflectionProbeBakeRequest> requests;
  std::vector<std::shared_ptr<GlobalReflectionProbe>> payloads;
  requests.reserve(probes.size());
  payloads.reserve(probes.size());
  const auto lighting = scene ? scene->environmental_lighting.Get<EnvironmentalLighting>() : nullptr;
  const auto pack = lighting ? lighting->GetReflectionProbePack() : nullptr;
  for (const auto* probe : probes) {
    const auto& payload = probe->payload;
    if (!payload || !pack) {
      throw std::runtime_error(std::string(context) + " has a missing reflection probe payload.");
    }
    requests.push_back({glm::vec3(probe->transform[3]), payload, pack, probe->stable_id});
    payloads.emplace_back(payload);
  }
  const auto queued_count = render_layer.QueueGlobalReflectionProbeBakeBatch(scene, requests);
  if (queued_count != requests.size()) {
    throw std::runtime_error(std::string(context) + " could not queue its reflection probe bake batch.");
  }
  for (size_t frame = 0; frame < 600u; ++frame) {
    if (!ApplicationContext::Get().Loop()) {
      throw std::runtime_error(std::string("Application ended during ") + context + " reflection probe batch.");
    }
    const bool ready = std::all_of(payloads.begin(), payloads.end(), [&](const auto& payload) {
      return !render_layer.IsGlobalReflectionProbeBakePending(payload) && !payload->Saved() &&
             payload->GetSourceKind() == GlobalReflectionProbe::SourceKind::Baked && payload->IsRuntimeReady();
    });
    if (ready) {
      return queued_count;
    }
  }
  throw std::runtime_error(std::string(context) + " reflection probe batch timed out.");
}

void ConfigureMainSceneCamera(const std::shared_ptr<Scene>& scene, const glm::vec3& position) {
  const auto main_camera = scene->main_camera.Get<Camera>();
  main_camera->Resize({1920, 1080});
  main_camera->post_processing_stack_ref = AssetManager::CreateTemporaryAsset<PostProcessingStack>();
  const auto main_camera_entity = main_camera->GetOwner();
  auto main_camera_transform = scene->GetDataComponent<Transform>(main_camera_entity);
  main_camera_transform.SetPosition(position);
  scene->SetDataComponent(main_camera_entity, main_camera_transform);
  scene->GetOrSetPrivateComponent<PlayerController>(main_camera_entity);
  if (const auto editor_layer = ApplicationContext::Get().GetLayer<EditorLayer>()) {
    editor_layer->SetSceneCameraPosition(position);
  }
}

void DisableSceneDirectionalLights(const std::shared_ptr<Scene>& scene) {
  if (const auto* directional_light_owners = scene->UnsafeGetPrivateComponentOwnersList<DirectionalLight>()) {
    for (const auto& owner : *directional_light_owners) {
      if (const auto directional_light = scene->GetOrSetPrivateComponent<DirectionalLight>(owner).lock()) {
        directional_light->SetEnabled(false);
      }
    }
  }
}

// DDGI_VALIDATION_RENDERING_SCENE_BEGIN
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
  scene->SetEnable(capoeira_entity, true);

  return base_entity;
}

void ConfigureRenderingDemoDdgi(const std::shared_ptr<Scene>& scene) {
  const auto lighting = GetOrCreateTemporaryEnvironmentalLighting(scene);
  if (!lighting) {
    return;
  }
  auto& settings = lighting->ddgi_settings;
  ConfigureStandardDdgiRuntime(settings, 8192, 0.02f);
  auto& ddgi_volume = ResetEnvironmentalLightingDdgiVolume(*lighting, "DDGI Probe Volume", glm::vec3(0.0f, 0.0f, -6.0f),
                                                           {10, 8, 16}, glm::vec3(1.5f), glm::vec3(0.0f, 3.0f, 3.0f));
  ddgi_volume.relocation_distance = 0.25f;
  ddgi_volume.enable_probe_relocation = true;
  ddgi_volume.enable_probe_classification = false;
}

bool SponzaProbeAuthoringRequested() {
  const auto* value = std::getenv("EVOENGINE_SPONZA_PROBE_AUTHORING");
  return value && (std::string(value) == "overwrite" || std::string(value) == "benchmark");
}

std::shared_ptr<EnvironmentalMap> LoadSponzaEnvironment() {
  const auto environment =
      std::dynamic_pointer_cast<EnvironmentalMap>(ProjectManager::GetOrCreateAsset(kSponzaEnvironmentPath));
  if (!environment || environment->environment_source_type != EnvironmentalMap::SourceType::SkyIllumination) {
    throw std::runtime_error("Rendering/Sponza is missing its tracked sky environment asset.");
  }
  environment->EnsureEnvironmentSource();
  auto cubemap_ref = environment->environment_cubemap;
  const auto environment_cubemap = cubemap_ref.Get<Cubemap>();
  const auto generated_global = AssetManager::CreateTemporaryAsset<GlobalReflectionProbe>();
  const auto persistent_global =
      std::dynamic_pointer_cast<GlobalReflectionProbe>(ProjectManager::GetOrCreateAsset(kSponzaGlobalProbePath));
  if (!environment_cubemap || !generated_global || !generated_global->ConstructFromCubemap(environment_cubemap) ||
      !generated_global->IsRuntimeReady() || !persistent_global) {
    throw std::runtime_error("Rendering/Sponza could not build its tracked global reflection source.");
  }
  if (SponzaProbeAuthoringRequested() && (!persistent_global->IsRuntimeReady() ||
                                          persistent_global->GetPayloadHash() != generated_global->GetPayloadHash())) {
    if (!persistent_global->SetCanonicalPayload(generated_global->GetCanonicalPayload()) ||
        !persistent_global->Save()) {
      throw std::runtime_error("Rendering/Sponza could not publish its tracked global reflection probe.");
    }
  }
  if (!persistent_global->IsRuntimeReady()) {
    throw std::runtime_error("Rendering/Sponza global reflection probe is missing; run the probe authoring hook.");
  }
  return environment;
}

std::shared_ptr<GlobalReflectionProbe> LoadSponzaGlobalReflectionProbe() {
  return std::dynamic_pointer_cast<GlobalReflectionProbe>(ProjectManager::GetOrCreateAsset(kSponzaGlobalProbePath));
}

void ConfigureSponzaReflectionProbes(const std::shared_ptr<Scene>& scene) {
  const auto lighting = GetOrCreateTemporaryEnvironmentalLighting(scene);
  if (!lighting) {
    return;
  }
  struct ProbeDefinition {
    glm::vec3 position;
    glm::vec3 size;
    int priority;
  };
  constexpr float blend_distance = 0.03f;
  constexpr std::array definitions = {ProbeDefinition{{-2.5f, 0.75f, -3.32f}, {4.0f, 6.2f, 14.5f}, 50},
                                      ProbeDefinition{{2.2f, 1.41f, -3.32f}, {3.0f, 5.0f, 14.5f}, 40},
                                      ProbeDefinition{{0.175f, 1.41f, 1.23f}, {3.4f, 5.0f, 5.8f}, 30},
                                      ProbeDefinition{{0.175f, 1.41f, -3.32f}, {3.4f, 5.0f, 5.8f}, 20},
                                      ProbeDefinition{{0.175f, 1.41f, -7.88f}, {3.4f, 5.0f, 5.8f}, 10}};

  if (!SponzaProbeAuthoringRequested()) {
    const auto pack =
        std::dynamic_pointer_cast<ReflectionProbePack>(ProjectManager::GetOrCreateAsset(kSponzaLocalProbePackPath));
    if (!pack || pack->probes.size() != definitions.size()) {
      throw std::runtime_error("Rendering/Sponza is missing its local reflection probe pack.");
    }
    for (size_t index = 0; index < definitions.size(); ++index) {
      const auto& probe = pack->probes[index];
      if (probe.name != kSponzaLocalProbeNames[index] ||
          probe.stable_id != StableEnvironmentalLightingId(kSponzaLocalProbeNames[index]) || !probe.payload ||
          !probe.payload->IsRuntimeReady()) {
        throw std::runtime_error("Rendering/Sponza local reflection probe pack is incomplete or invalid.");
      }
    }
    lighting->reflection_probe_pack = pack;
    return;
  }

  const auto pack = AssetManager::CreateTemporaryAsset<ReflectionProbePack>();
  lighting->reflection_probe_pack = pack;
  for (size_t index = 0; index < definitions.size(); ++index) {
    const auto& definition = definitions[index];
    Transform probe_transform;
    probe_transform.SetPosition(definition.position);
    probe_transform.SetScale(definition.size);
    AddEnvironmentalLightingLocalReflectionProbe(
        *lighting, kSponzaLocalProbeNames[index], probe_transform.value,
        AssetManager::CreateTemporaryAsset<GlobalReflectionProbe>(), definition.priority,
        EnvironmentalLighting::LocalReflectionProbeShape::Box, 1.0f, blend_distance, true);
  }
}

void AddRenderingDemoReflectionProbeComparisonSpheres(const std::shared_ptr<Scene>& scene, const Entity& root) {
  constexpr float root_scale = 0.5f;
  constexpr float sphere_world_scale = 0.28f;
  constexpr float bottom_y = -0.42f;
  constexpr float top_y = 0.14f;
  constexpr float sphere_z = -2.55f;
  const auto collection = scene->CreateEntity("Reflection Probe Comparison Spheres");
  scene->SetParent(collection, root);
  const auto& primitives = Resources::GetInstance().GetPrimitives();
  const auto create_sphere = [&](const std::string& name, const glm::vec3& world_position, const glm::vec3& albedo,
                                 const float roughness, const float metallic) {
    CreateRenderingRegressionProbe(scene, collection, name, primitives.sphere, world_position / root_scale,
                                   glm::vec3(sphere_world_scale / root_scale), albedo, roughness, metallic);
  };
  create_sphere("Reflection Probe Comparison Left Rough Dielectric", {-2.35f, bottom_y, sphere_z}, glm::vec3(0.78f),
                0.88f, 0.0f);
  create_sphere("Reflection Probe Comparison Left Smooth Metal", {-2.35f, top_y, sphere_z}, glm::vec3(0.92f), 0.05f,
                1.0f);
  create_sphere("Reflection Probe Comparison Right Rough Dielectric", {2.2f, bottom_y, sphere_z}, glm::vec3(0.78f),
                0.88f, 0.0f);
  create_sphere("Reflection Probe Comparison Right Smooth Metal", {2.2f, top_y, sphere_z}, glm::vec3(0.92f), 0.05f,
                1.0f);
}

void ConfigureRenderingDemoScene(const std::shared_ptr<Scene>& scene) {
  const auto environment = LoadSponzaEnvironment();
  scene->global_reflection_probe_fallback = LoadSponzaGlobalReflectionProbe();
  if (const auto lighting = GetOrCreateTemporaryEnvironmentalLighting(scene)) {
    ConfigureEnvironmentalLightingMapSource(*lighting, environment, 1.0f, lighting->diffuse_fallback_intensity,
                                            lighting->specular_fallback_intensity);
  }

  ConfigureMainSceneCamera(scene, glm::vec3(0, 0, 3));

  const auto demo_scene = LoadRenderingScene(scene, "Rendering Demo", true);
  Transform demo_transform;
  demo_transform.SetScale(glm::vec3(0.5f));
  scene->SetDataComponent(demo_scene, demo_transform);
  AddRenderingDemoReflectionProbeComparisonSpheres(scene, demo_scene);
  ConfigureRenderingDemoDdgi(scene);
  ConfigureSponzaReflectionProbes(scene);

  const auto directional_light_entity = scene->CreateEntity("Top Down Directional Light");
  const auto directional_light = scene->GetOrSetPrivateComponent<DirectionalLight>(directional_light_entity).lock();
  directional_light->diffuse = glm::vec3(1.0f);
  directional_light->diffuse_brightness = 5.0f;
  directional_light->light_size = 0.01f;
  Transform directional_light_transform;
  directional_light_transform.SetEulerRotation(glm::radians(glm::vec3(90.0f, 0.0f, 0.0f)));
  scene->SetDataComponent(directional_light_entity, directional_light_transform);

  const auto moving_light_entity = scene->CreateEntity("Left Point Light");
  const auto point_light_right_renderer = scene->GetOrSetPrivateComponent<MeshRenderer>(moving_light_entity).lock();
  point_light_right_renderer->cast_shadow = false;
  const auto moving_light_material = AssetManager::CreateTemporaryAsset<Material>();
  point_light_right_renderer->material.Set<Material>(moving_light_material);
  ConfigureMaterial(moving_light_material, glm::vec3(1.0f, 0.8f, 0.0f), 1.0f, 1.0f, 2.0f);
  point_light_right_renderer->mesh = Resources::GetInstance().GetPrimitives().sphere;
  const auto moving_light = scene->GetOrSetPrivateComponent<PointLight>(moving_light_entity).lock();
  moving_light->diffuse_brightness = 24.0f;
  moving_light->light_size = 0.005f;
  moving_light->constant = 2.5f;
  moving_light->linear = 0.5f;
  moving_light->quadratic = 0.1f;
  moving_light->diffuse = glm::vec3(1.0f, 0.8f, 0.0f);

  Transform moving_light_transform;
  moving_light_transform.SetPosition(glm::vec3(3, 0, -2.5f));
  moving_light_transform.SetScale(glm::vec3(0.1f));
  scene->SetDataComponent(moving_light_entity, moving_light_transform);

  ApplicationContext::Get().RegisterUpdateFunction([moving_light_entity]() {
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
    auto current_moving_light_entity = moving_light_entity;
    if (!current_scene->IsEntityValid(current_moving_light_entity)) {
      for (const auto& entity : current_scene->UnsafeGetAllEntities()) {
        if (current_scene->IsEntityValid(entity) && current_scene->GetEntityName(entity) == "Left Point Light") {
          current_moving_light_entity = entity;
          break;
        }
      }
    }
    if (!current_scene->IsEntityValid(current_moving_light_entity)) {
      last_frame_playing = playing;
      return;
    }
    static float start_time;
    if (!last_frame_playing) {
      start_time = application.GetTimes().Now();
    }
    const float current_time = application.GetTimes().Now() - start_time;
    const float cos_time = glm::cos(current_time / 2.5f);

    Transform current_moving_light_transform;
    current_moving_light_transform.SetPosition(glm::vec3(3, 0, cos_time * 2.5f - 2.5f));
    current_moving_light_transform.SetScale(glm::vec3(0.1f));
    current_scene->SetDataComponent(current_moving_light_entity, current_moving_light_transform);

    last_frame_playing = playing;
  });
}
// DDGI_VALIDATION_RENDERING_SCENE_END

// DDGI_VALIDATION_CORNELL_SCENE_BEGIN
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
  const auto lighting = GetOrCreateTemporaryEnvironmentalLighting(scene);
  if (!lighting) {
    return;
  }
  auto& settings = lighting->ddgi_settings;
  ConfigureStandardDdgiRuntime(settings, 1024, 0.02f);
  auto& ddgi_volume = ResetEnvironmentalLightingDdgiVolume(*lighting, "DDGI Probe Volume", glm::vec3(0.0f, 0.0f, -3.0f),
                                                           {9, 9, 9}, glm::vec3(0.3f), glm::vec3(0.0f));
  ddgi_volume.relocation_distance = 0.1f;
  ddgi_volume.enable_probe_relocation = true;
  ddgi_volume.enable_probe_classification = false;
}

void ConfigureCornellBoxScene(const std::shared_ptr<Scene>& scene) {
  if (const auto lighting = GetOrCreateTemporaryEnvironmentalLighting(scene)) {
    ConfigureEnvironmentalLightingColorSource(*lighting, glm::vec3(0.0f), 0.0f, 1.0f);
  }
  DisableSceneDirectionalLights(scene);
  ConfigureMainSceneCamera(scene, glm::vec3(0.0f, 0.0f, 1.6f));

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
// DDGI_VALIDATION_CORNELL_SCENE_END

void ConfigureThinWallDdgi(const std::shared_ptr<Scene>& scene) {
  const auto lighting = GetOrCreateTemporaryEnvironmentalLighting(scene);
  if (!lighting) {
    return;
  }
  auto& settings = lighting->ddgi_settings;
  ConfigureStandardDdgiRuntime(settings, 1024, 0.015f);
  auto& ddgi_volume = ResetEnvironmentalLightingDdgiVolume(*lighting, "DDGI Probe Volume", glm::vec3(0.0f, 0.0f, -3.0f),
                                                           {8, 6, 8}, glm::vec3(0.35f), glm::vec3(0.0f));
  ddgi_volume.relocation_distance = 0.25f;
  ddgi_volume.enable_probe_relocation = true;
  ddgi_volume.enable_probe_classification = false;
}

void ConfigureThinWallScene(const std::shared_ptr<Scene>& scene) {
  if (const auto lighting = GetOrCreateTemporaryEnvironmentalLighting(scene)) {
    ConfigureEnvironmentalLightingColorSource(*lighting, glm::vec3(0.0f), 0.0f, 1.0f);
  }
  DisableSceneDirectionalLights(scene);
  ConfigureMainSceneCamera(scene, glm::vec3(0.0f, 0.0f, 0.9f));

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
  if (std::filesystem::file_size(source) != std::filesystem::file_size(target)) {
    return true;
  }
  std::ifstream source_stream(source, std::ios::binary);
  std::ifstream target_stream(target, std::ios::binary);
  return !std::equal(std::istreambuf_iterator<char>(source_stream), std::istreambuf_iterator<char>(),
                     std::istreambuf_iterator<char>(target_stream));
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
  if (!std::filesystem::exists(source_sentinel) || !std::filesystem::is_regular_file(source_sentinel)) {
    throw std::runtime_error("Missing Rendering Regression source asset: " + source_sentinel.string());
  }
  for (const auto& entry : std::filesystem::recursive_directory_iterator(source)) {
    if (entry.is_regular_file()) {
      CopyGeneratedAssetFileIfNeeded(entry.path(), target / std::filesystem::relative(entry.path(), source));
    }
  }
}

void PrepareRenderingRegressionGeneratedAssets(const std::filesystem::path& resource_root) {
  const auto source_models = resource_root / "EvoEngine-DemoProjects" / "Rendering" / "Assets" / "Models";
  const auto target_models =
      resource_root / ".generated" / "EvoEngine-DemoProjects" / "RenderingRegression" / "Assets" / "Models";
  CopyGeneratedAssetFileIfNeeded(source_models / "Capoeira.fbx", target_models / "Capoeira.fbx");
  CopyGeneratedAssetFileIfNeeded(source_models / "EvoEngine.obj", target_models / "EvoEngine.obj");
  CopyGeneratedAssetDirectoryIfNeeded(source_models / "Sponza_FBX", target_models / "Sponza_FBX", "Sponza.fbx");
  const auto source_lighting =
      resource_root / "EvoEngine-DemoProjects" / "Rendering" / "Assets" / kSponzaLightingDirectory;
  const auto target_lighting = resource_root / ".generated" / "EvoEngine-DemoProjects" / "RenderingRegression" /
                               "Assets" / kSponzaLightingDirectory;
  CopyGeneratedAssetDirectoryIfNeeded(source_lighting, target_lighting, "SponzaEnvironment.eveenvironmentalmap");
}

void ConfigureProceduralGalaxyScene(const std::shared_ptr<Scene>& scene) {
  if (const auto lighting = GetOrCreateTemporaryEnvironmentalLighting(scene)) {
    ConfigureEnvironmentalLightingColorSource(*lighting, glm::vec3(0.0f), 0.0f, 1.0f);
  }

  const auto main_camera = scene->main_camera.Get<Camera>();
  main_camera->Resize({1920, 1080});
  main_camera->skybox.Clear();
  main_camera->camera_settings.background_source = Camera::BackgroundSource::ClearColor;
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
      scene_camera->camera_settings.background_source = Camera::BackgroundSource::ClearColor;
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
  scene->global_reflection_probe_fallback = Resources::GetInstance().GetDefaultGlobalReflectionProbe();
  if (const auto lighting = GetOrCreateTemporaryEnvironmentalLighting(scene)) {
    ConfigureEnvironmentalLightingMapSource(*lighting, Resources::GetInstance().GetDefaultEnvironmentalMap(), 0.25f,
                                            1.0f, 0.0f, glm::vec3(0.01f, 0.012f, 0.016f));
  }

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
  main_camera->skybox.Clear();
  main_camera->camera_settings.background_source = Camera::BackgroundSource::ClearColor;
  main_camera->camera_settings.clear_color = glm::vec4(0.0f, 0.0f, 0.0f, 1.0f);
  main_camera->camera_settings.background_intensity = 0.0f;
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
      scene_camera->skybox.Clear();
      scene_camera->camera_settings.background_source = Camera::BackgroundSource::ClearColor;
      scene_camera->camera_settings.clear_color = glm::vec4(0.0f, 0.0f, 0.0f, 1.0f);
      scene_camera->camera_settings.background_intensity = 0.0f;
      scene_camera->camera_settings.near_distance = main_camera->camera_settings.near_distance;
      scene_camera->camera_settings.far_distance = main_camera->camera_settings.far_distance;
      scene_camera->camera_settings.fov = main_camera->camera_settings.fov;
      scene_camera->camera_render_mode = Camera::CameraRenderMode::Rasterization;
      scene_camera->ResetFrameCount();
    }
  }

  SyncTemporaryEnvironmentalLightingSettingsFromScene(scene);
  scene->Save();
  ProjectManager::SaveProject();
}

// DDGI_VALIDATION_ENTITY_LOOKUP_HELPER_BEGIN
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
// DDGI_VALIDATION_ENTITY_LOOKUP_HELPER_END

void RemoveDefaultDirectionalLight(const std::shared_ptr<Scene>& scene) {
  const auto default_light_entity = FindEntityNamed(scene, "Directional Light");
  if (default_light_entity && scene->HasPrivateComponent<DirectionalLight>(*default_light_entity)) {
    scene->DeleteEntity(*default_light_entity);
  }
}

void ConfigureBistroDemoDdgi(const std::shared_ptr<Scene>& scene, const Bound& bistro_world_bound) {
  const auto lighting = GetOrCreateTemporaryEnvironmentalLighting(scene);
  if (!lighting) {
    return;
  }
  constexpr auto min_spacing = 4.0f;

  lighting->environment_lighting_intensity = EnvironmentalLighting::kDefaultEnvironmentLightingIntensity;
  SetEnvironmentalLightingFallbackIntensities(*lighting, 0.0f, 0.0f);
  auto& settings = lighting->ddgi_settings;
  settings = DdgiSettings{};
  settings.runtime.enabled = true;
  SetDdgiUpdatesPaused(false);
  settings.runtime.enable_emissive_mesh_sampling = true;
  settings.runtime.ray_count = 192;
  settings.runtime.emissive_ray_count = 64;
  settings.runtime.normal_bias = std::max(0.02f, min_spacing * 0.02f);
  settings.runtime.view_bias = std::max(0.05f, min_spacing * 0.04f);
  RequestDdgiHistoryReset();
  settings.volume_defaults.probe_counts = kBistroDdgiProbeCounts;
  settings.volume_defaults.probe_spacing = kBistroDdgiProbeSpacing;
  settings.volume_defaults.volume_origin = kBistroDdgiVolumeOrigin;
  settings.volume_defaults.relocation_distance = std::min(min_spacing * 0.25f, 50.0f);
  settings.volume_defaults.enable_probe_relocation = true;
  settings.volume_defaults.enable_probe_classification = false;
  settings.storage.max_probe_count = kBistroDdgiMaxProbeCount;
  DisableDdgiDebugVisualization();

  lighting->GetOrCreateReflectionProbePack()->probes.clear();
  lighting->GetOrCreateDdgiVolumePack()->volumes.clear();
  auto& ddgi_volume =
      AddEnvironmentalLightingDdgiVolume(*lighting, kBistroDdgiVolumeName, glm::mat4(1.0f), kBistroDdgiProbeCounts,
                                         kBistroDdgiProbeSpacing, kBistroDdgiVolumeOrigin);
  ddgi_volume.relocation_distance = settings.volume_defaults.relocation_distance;
  ddgi_volume.enable_probe_relocation = settings.volume_defaults.enable_probe_relocation;
  ddgi_volume.enable_probe_classification = settings.volume_defaults.enable_probe_classification;

  if (const auto existing_ddgi_volume = FindEntityNamed(scene, kBistroDdgiVolumeName)) {
    scene->DeleteEntity(*existing_ddgi_volume);
  }

  std::ostringstream stream;
  stream << "Bistro DDGI setup: enabled=" << settings.runtime.enabled << ", probe_counts=(" << kBistroDdgiProbeCounts.x
         << "," << kBistroDdgiProbeCounts.y << "," << kBistroDdgiProbeCounts.z
         << "), probe_count=" << kBistroDdgiProbeCount << ", probe_spacing=(" << kBistroDdgiProbeSpacing.x << ","
         << kBistroDdgiProbeSpacing.y << "," << kBistroDdgiProbeSpacing.z << "), volume_origin=("
         << kBistroDdgiVolumeOrigin.x << "," << kBistroDdgiVolumeOrigin.y << "," << kBistroDdgiVolumeOrigin.z
         << "), normal_bias=" << settings.runtime.normal_bias << ", view_bias=" << settings.runtime.view_bias
         << ", storage_max_probe_count=" << settings.storage.max_probe_count << ", world_bound_min=("
         << bistro_world_bound.min.x << "," << bistro_world_bound.min.y << "," << bistro_world_bound.min.z
         << "), world_bound_max=(" << bistro_world_bound.max.x << "," << bistro_world_bound.max.y << ","
         << bistro_world_bound.max.z << ")";
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
    const auto previous_light_size = light->light_size;
    light->diffuse_brightness = kBistroDirectionalLightIntensity;
    light->light_size = kBistroDirectionalLightSize;
    const auto effective_color = light->diffuse * light->diffuse_brightness;

    std::ostringstream stream;
    stream << "Bistro directional light policy: entity=\"" << kBistroImportedSunLightName
           << "\", previous_brightness=" << previous_brightness
           << ", target_brightness=" << kBistroDirectionalLightIntensity
           << ", effective_brightness=" << light->diffuse_brightness << ", previous_to_effective_ratio="
           << (kBistroDirectionalLightIntensity > 0.0f ? previous_brightness / kBistroDirectionalLightIntensity : 0.0f)
           << ", previous_light_size=" << previous_light_size << ", target_light_size=" << kBistroDirectionalLightSize
           << ", effective_light_size=" << light->light_size << ", previous_color=(" << previous_color.x << ","
           << previous_color.y << "," << previous_color.z << "), effective_color=(" << effective_color.x << ","
           << effective_color.y << "," << effective_color.z << ")";
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
  rendering_regression_temporal_motion_state->geometry_camera_enabled = enabled;
  rendering_regression_temporal_motion_state->frame = 0;
}

void evo_engine::SetRenderingRegressionMovingLightEnabled(const bool enabled) {
  if (!rendering_regression_temporal_motion_state) {
    return;
  }
  rendering_regression_temporal_motion_state->moving_light_enabled = enabled;
  rendering_regression_temporal_motion_state->frame = 0;
}

void evo_engine::SetRenderingRegressionSecondaryGeometryMotionEnabled(const bool enabled) {
  if (!rendering_regression_temporal_motion_state) {
    return;
  }
  rendering_regression_temporal_motion_state->secondary_geometry_enabled = enabled;
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

  scene->global_reflection_probe_fallback = Resources::GetInstance().GetDefaultGlobalReflectionProbe();
  if (const auto lighting = GetOrCreateTemporaryEnvironmentalLighting(scene)) {
    ConfigureEnvironmentalLightingMapSource(*lighting, Resources::GetInstance().GetDefaultEnvironmentalMap(), 0.25f,
                                            1.0f, 0.0f, glm::vec3(0.025f, 0.028f, 0.034f));
  }

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

  ConfigureRenderingRegressionGltfMaterialProbes(scene, root);
  ConfigureRenderingRegressionAdvancedRayMaterialProbes(scene, root);
  ConfigureRenderingRegressionEmissiveNeeProbes(scene, root);
  ConfigureRenderingRegressionImportedProbes(scene, root);
  const auto moving_light = ConfigureRenderingRegressionLights(scene, root);
  ConfigureRenderingRegressionCamera(scene);

  rendering_regression_temporal_motion_state = std::make_shared<RenderingRegressionTemporalMotionState>();
  rendering_regression_temporal_motion_state->scene = scene;
  rendering_regression_temporal_motion_state->rigid_entity = moving_rigid;
  rendering_regression_temporal_motion_state->transparent_entity = moving_transparent;
  rendering_regression_temporal_motion_state->light_entity = moving_light;
  if (const auto skinned_entity = FindEntityNamed(scene, "M42 Skinned Capoeira Probe")) {
    rendering_regression_temporal_motion_state->skinned_entity = *skinned_entity;
  }
  RegisterRenderingRegressionTemporalMotionUpdate();
  SyncTemporaryEnvironmentalLightingSettingsFromScene(scene);
}

void evo_engine::ConfigureEnvironmentLightingValidationScene(const std::shared_ptr<Scene>& scene) {
  ConfigureDdgiValidationFixture(scene, "furnace");
  if (!scene) {
    return;
  }

  if (const auto furnace_center = FindEntityNamed(scene, "DDGI Furnace Center")) {
    scene->DeleteEntity(*furnace_center);
  }
  const auto fixture_root = FindEntityNamed(scene, "DDGI Validation Fixture");
  const auto root = fixture_root ? *fixture_root : scene->CreateEntity("Environment Lighting Validation Fixture");
  const auto& sphere = Resources::GetInstance().GetPrimitives().sphere;
  CreateRenderingRegressionProbe(scene, root, "Environment Lighting Dielectric", sphere,
                                 glm::vec3(-0.95f, 0.85f, -2.4f), glm::vec3(0.72f), glm::vec3(0.8f), 0.28f, 0.0f);
  CreateRenderingRegressionProbe(scene, root, "Environment Lighting Exact Metal", sphere,
                                 glm::vec3(0.95f, 0.85f, -2.4f), glm::vec3(0.72f), glm::vec3(0.8f), 0.28f, 1.0f);

  const auto ibl_diffuse_receiver = CreateRenderingRegressionProbe(
      scene, root, "Environment Lighting IBL Diffuse Receiver", Resources::GetInstance().GetPrimitives().cube,
      glm::vec3(0.8f, 1.5f, 0.6f), glm::vec3(0.24f), glm::vec3(0.25f), 0.8f, 0.0f, 0.0f, false);
  if (const auto renderer = scene->GetOrSetPrivateComponent<MeshRenderer>(ibl_diffuse_receiver).lock()) {
    if (const auto material = renderer->material.Get<Material>()) {
      auto& shade = material->material_data.shade_material;
      shade.occlusion_strength = 1.0f;
      shade.specular_factor = 0.0f;
      material->MarkDirty();
    }
  }

  const auto direct_receiver = CreateRenderingRegressionProbe(scene, root, "Environment Lighting Direct Receiver",
                                                              sphere, glm::vec3(-0.28f, 1.85f, -2.4f), glm::vec3(0.42f),
                                                              glm::vec3(0.82f), 0.5f, 0.0f, 0.0f, false);
  if (const auto renderer = scene->GetOrSetPrivateComponent<MeshRenderer>(direct_receiver).lock()) {
    if (const auto material = renderer->material.Get<Material>()) {
      auto& shade = material->material_data.shade_material;
      shade.occlusion_strength = 1.0f;
      shade.specular_factor = 0.0f;
      material->MarkDirty();
    }
  }
  const auto direct_light_entity = scene->CreateEntity("Environment Lighting Direct Light");
  scene->SetParent(direct_light_entity, root);
  Transform direct_light_transform;
  direct_light_transform.SetPosition(glm::vec3(-0.28f, 1.85f, -1.6f));
  scene->SetDataComponent(direct_light_entity, direct_light_transform);
  const auto direct_light = scene->GetOrSetPrivateComponent<PointLight>(direct_light_entity).lock();
  direct_light->cast_shadow = false;
  direct_light->diffuse = glm::vec3(1.0f, 0.82f, 0.58f);
  direct_light->diffuse_brightness = 8.0f;
  direct_light->range = 1.2f;
  direct_light->light_size = 0.0f;
  direct_light->constant = 1.0f;
  direct_light->linear = 0.0f;
  direct_light->quadratic = 0.0f;

  const auto emission_marker = CreateRenderingRegressionProbe(scene, root, "Environment Lighting Emission Marker",
                                                              sphere, glm::vec3(0.28f, 1.85f, -2.4f), glm::vec3(0.42f),
                                                              glm::vec3(1.0f, 0.32f, 0.06f), 0.5f, 0.0f, 6.0f, false);
  if (const auto renderer = scene->GetOrSetPrivateComponent<MeshRenderer>(emission_marker).lock()) {
    if (const auto material = renderer->material.Get<Material>()) {
      auto& shade = material->material_data.shade_material;
      shade.pbr_base_color_factor = glm::vec4(0.0f, 0.0f, 0.0f, 1.0f);
      shade.occlusion_strength = 0.0f;
      shade.specular_factor = 0.0f;
      material->MarkDirty();
    }
  }

  const auto lighting = GetOrCreateTemporaryEnvironmentalLighting(scene);
  if (!lighting) {
    throw std::runtime_error("Environment lighting validation requires an EnvironmentalLighting asset.");
  }
  ConfigureEnvironmentalLightingColorSource(*lighting, glm::vec3(1.0f), 1.0f, 1.0f, 1.0f);

  auto& ddgi = lighting->ddgi_settings;
  ddgi.runtime.enabled = true;
  SetDdgiUpdatesPaused(false);
  ddgi.runtime.ray_count = 16;
  ddgi.runtime.warmup_frames = 1;
  ddgi.runtime.deterministic_ray_seed_enabled = true;
  ddgi.runtime.deterministic_ray_seed = 0x6d2b79f5u;
  ddgi.storage.max_probe_count = 64;
  DisableDdgiDebugVisualization();
  ddgi.volume_defaults.enable_probe_relocation = false;
  ddgi.volume_defaults.enable_probe_classification = false;
  ddgi.volume_defaults.enable_probe_variability = true;
  ddgi.volume_defaults.enable_probe_variability_gating = true;
  ddgi.volume_defaults.probe_variability_threshold = 1.0f;
  ddgi.volume_defaults.probe_variability_min_samples = 1;
  if (auto* volume = FindEnvironmentalLightingDdgiVolume(scene, "DDGI Validation Volume")) {
    volume->probe_counts = {3, 2, 3};
    volume->probe_spacing = glm::vec3(1.2f);
    volume->volume_origin = glm::vec3(0.0f);
    volume->enable_probe_relocation = false;
    volume->enable_probe_classification = false;
    volume->enable_probe_variability = true;
    volume->enable_probe_variability_gating = true;
    volume->probe_variability_threshold = 1.0f;
    volume->probe_variability_min_samples = 1;
  }

  const auto configure_camera = [](const std::shared_ptr<Camera>& camera) {
    if (!camera) {
      return;
    }
    camera->Resize({1920, 1080});
    camera->skybox.Clear();
    camera->camera_render_mode = Camera::CameraRenderMode::Rasterization;
    camera->camera_settings.background_source = Camera::BackgroundSource::ClearColor;
    camera->camera_settings.clear_color = glm::vec4(0.12f, 0.24f, 0.48f, 1.0f);
    camera->camera_settings.background_intensity = 0.0f;
    camera->camera_settings.sample_size = 1;
    camera->camera_settings.bounce = 4;
    camera->camera_settings.ray_debug_view = CameraSettings::RayDebugView::Beauty;
    camera->camera_settings.auto_spp_enabled = false;
    camera->post_processing_stack_ref = AssetManager::CreateTemporaryAsset<PostProcessingStack>();
    if (const auto stack = camera->post_processing_stack_ref.Get<PostProcessingStack>()) {
      stack->enable_ambient_occlusion = false;
      stack->enable_bloom = false;
      stack->enable_screen_space_reflection = false;
      stack->enable_anti_aliasing = false;
      stack->enable_tone_mapping = true;
      if (stack->tone_mapping) {
        stack->tone_mapping->method = ToneMapping::ToneMapMethod::Filmic;
        stack->tone_mapping->exposure = 1.0f;
        stack->tone_mapping->brightness = 1.0f;
        stack->tone_mapping->contrast = 1.0f;
        stack->tone_mapping->saturation = 1.0f;
        stack->tone_mapping->vignette = 0.0f;
        stack->tone_mapping->auto_exposure = false;
        stack->tone_mapping->dither = false;
      }
    }
    camera->ResetFrameCount();
  };
  configure_camera(scene->main_camera.Get<Camera>());
  if (const auto editor_layer = ApplicationContext::Get().GetLayer<EditorLayer>()) {
    configure_camera(editor_layer->GetSceneCamera());
  }
  SyncTemporaryEnvironmentalLightingSettingsFromScene(scene);
  if (const auto lighting = scene->environmental_lighting.Get<EnvironmentalLighting>()) {
    SetEnvironmentalLightingFallbackIntensities(*lighting, 1.0f, 1.0f);
  }
}

namespace {
std::shared_ptr<GlobalReflectionProbe> CreateSyntheticReflectionProbe(
    const std::array<glm::vec3, 6>& face_colors,
    const std::array<float, GlobalReflectionProbe::kMipLevels>& mip_scale) {
  std::vector<uint16_t> payload(GlobalReflectionProbe::kCanonicalTexelCount * 4);
  size_t offset = 0;
  for (uint32_t face = 0; face < 6; ++face) {
    uint32_t size = GlobalReflectionProbe::kResolution;
    for (uint32_t mip = 0; mip < GlobalReflectionProbe::kMipLevels; ++mip) {
      const auto color = glm::max(face_colors[face] * mip_scale[mip], glm::vec3(0.0f));
      const std::array packed = {glm::packHalf1x16(color.x), glm::packHalf1x16(color.y), glm::packHalf1x16(color.z),
                                 glm::packHalf1x16(1.0f)};
      const size_t texel_count = static_cast<size_t>(size) * size;
      for (size_t texel = 0; texel < texel_count; ++texel) {
        std::copy(packed.begin(), packed.end(), payload.begin() + static_cast<ptrdiff_t>(offset));
        offset += 4;
      }
      size = glm::max(size / 2u, 1u);
    }
  }
  if (offset != payload.size()) {
    throw std::runtime_error("Synthetic reflection probe payload size is invalid.");
  }
  const auto asset = AssetManager::CreateTemporaryAsset<GlobalReflectionProbe>();
  if (!asset || !asset->SetCanonicalPayload(payload)) {
    throw std::runtime_error("Synthetic reflection probe upload failed.");
  }
  return asset;
}

std::shared_ptr<GlobalReflectionProbe> CreateConstantReflectionProbe(const glm::vec3& color) {
  std::array<glm::vec3, 6> faces;
  faces.fill(color);
  std::array<float, GlobalReflectionProbe::kMipLevels> mip_scale;
  mip_scale.fill(1.0f);
  return CreateSyntheticReflectionProbe(faces, mip_scale);
}

}  // namespace

void evo_engine::ConfigureReflectionProbeValidationScene(const std::shared_ptr<Scene>& scene) {
  ConfigureDdgiValidationFixture(scene, "furnace");
  if (!scene) {
    return;
  }
  const auto main_camera = scene->main_camera.Get<Camera>();
  if (!main_camera) {
    throw std::runtime_error("Reflection probe validation requires a main camera.");
  }
  const auto main_camera_entity = main_camera->GetOwner();
  std::vector<Entity> entities;
  scene->GetAllEntities(entities);
  for (const auto& entity : entities) {
    if (entity != main_camera_entity) {
      scene->DeleteEntity(entity);
    }
  }

  const auto lighting = GetOrCreateTemporaryEnvironmentalLighting(scene);
  if (!lighting) {
    throw std::runtime_error("Reflection probe validation requires an EnvironmentalLighting asset.");
  }
  ConfigureEnvironmentalLightingColorSource(*lighting, glm::vec3(0.18f), 1.0f, 0.0f);
  lighting->ddgi_settings = {};
  lighting->ddgi_settings.runtime.enabled = false;
  DisableDdgiDebugVisualization();
  lighting->GetOrCreateReflectionProbePack()->probes.clear();
  lighting->GetOrCreateDdgiVolumePack()->volumes.clear();

  const glm::vec3 camera_position(0.0f, 1.0f, 7.0f);
  const glm::vec3 camera_target(0.0f, 0.85f, -2.4f);
  const auto camera_rotation =
      glm::quatLookAt(glm::normalize(camera_target - camera_position), glm::vec3(0.0f, 1.0f, 0.0f));
  const auto configure_camera = [&](const std::shared_ptr<Camera>& camera) {
    if (!camera) {
      return;
    }
    camera->Resize({1920, 1080});
    camera->skybox.Clear();
    camera->camera_render_mode = Camera::CameraRenderMode::Rasterization;
    camera->camera_settings.fov = 90.0f;
    camera->camera_settings.near_distance = 0.05f;
    camera->camera_settings.far_distance = 100.0f;
    camera->camera_settings.background_source = Camera::BackgroundSource::ClearColor;
    camera->camera_settings.clear_color = glm::vec4(0.0f, 0.0f, 0.0f, 1.0f);
    camera->camera_settings.background_intensity = 0.0f;
    camera->camera_settings.sample_size = 1;
    camera->post_processing_stack_ref = AssetManager::CreateTemporaryAsset<PostProcessingStack>();
    if (const auto stack = camera->post_processing_stack_ref.Get<PostProcessingStack>()) {
      stack->enable_ambient_occlusion = false;
      stack->enable_bloom = false;
      stack->enable_screen_space_reflection = false;
      stack->enable_anti_aliasing = false;
      stack->enable_tone_mapping = true;
      if (stack->tone_mapping) {
        stack->tone_mapping->method = ToneMapping::ToneMapMethod::Filmic;
        stack->tone_mapping->exposure = 1.0f;
        stack->tone_mapping->brightness = 1.0f;
        stack->tone_mapping->contrast = 1.0f;
        stack->tone_mapping->saturation = 1.0f;
        stack->tone_mapping->vignette = 0.0f;
        stack->tone_mapping->auto_exposure = false;
        stack->tone_mapping->dither = false;
      }
    }
    camera->ResetFrameCount();
  };
  configure_camera(main_camera);
  Transform main_camera_transform;
  main_camera_transform.SetValue(camera_position, camera_rotation, glm::vec3(1.0f));
  scene->SetDataComponent(main_camera_entity, main_camera_transform);
  if (const auto editor_layer = ApplicationContext::Get().GetLayer<EditorLayer>()) {
    editor_layer->enable_gizmos = false;
    editor_layer->SetSelectedEntity({});
    editor_layer->default_scene_camera_position = camera_position;
    editor_layer->default_scene_camera_rotation = camera_rotation;
    editor_layer->SetSceneCameraPosition(camera_position);
    editor_layer->SetSceneCameraRotation(camera_rotation);
    configure_camera(editor_layer->GetSceneCamera());
  }

  const auto root = scene->CreateEntity("Reflection Probe Validation Fixture");
  const auto& primitives = Resources::GetInstance().GetPrimitives();
  CreateRenderingRegressionProbe(scene, root, "Reflection Probe Left Room Receiver", primitives.cube,
                                 {-1.45f, 0.8f, -3.25f}, {1.4f, 1.45f, 0.05f}, glm::vec3(0.82f), 0.34f, 1.0f, 0.0f,
                                 false);
  CreateRenderingRegressionProbe(scene, root, "Reflection Probe Right Room Receiver", primitives.cube,
                                 {1.45f, 0.8f, -3.25f}, {1.4f, 1.45f, 0.05f}, glm::vec3(0.82f), 0.34f, 1.0f, 0.0f,
                                 false);
  const auto create_metal = [&](const char* name, const glm::vec3& position, const glm::vec3& scale,
                                const float roughness) {
    return CreateRenderingRegressionProbe(scene, root, name, primitives.sphere, position, scale, glm::vec3(0.82f),
                                          roughness, 1.0f, 0.0f, false);
  };
  create_metal("Reflection Probe Left Metal", {-2.0f, 0.8f, -2.4f}, glm::vec3(0.62f), 0.16f);
  create_metal("Reflection Probe Center Metal", {0.0f, 0.8f, -2.4f}, glm::vec3(0.48f), 0.16f);
  create_metal("Reflection Probe Boundary Metal", {0.72f, 0.8f, -2.4f}, glm::vec3(0.34f), 0.16f);
  create_metal("Reflection Probe Right Metal", {2.0f, 0.8f, -2.4f}, glm::vec3(0.62f), 0.16f);
  create_metal("Reflection Probe Smooth Metal", {-0.8f, 2.05f, -2.4f}, glm::vec3(0.5f), 0.0f);
  create_metal("Reflection Probe Rough Metal", {0.8f, 2.05f, -2.4f}, glm::vec3(0.5f), 1.0f);
  create_metal("Reflection Probe Fallback Metal", {0.0f, -0.55f, -2.4f}, glm::vec3(0.5f), 0.2f);
  CreateRenderingRegressionProbe(scene, root, "Reflection Probe Bake Emitter", primitives.cube, {0.0f, 1.2f, -4.1f},
                                 {0.18f, 0.18f, 0.06f}, {1.0f, 0.35f, 0.08f}, 0.5f, 0.0f, 8.0f, false);

  const auto red = CreateConstantReflectionProbe({3.0f, 0.03f, 0.03f});
  const auto blue = CreateConstantReflectionProbe({0.03f, 0.03f, 3.0f});
  const auto green = CreateConstantReflectionProbe({0.03f, 3.0f, 0.03f});
  const auto magenta = CreateConstantReflectionProbe({2.5f, 0.03f, 2.5f});
  const std::array<glm::vec3, 6> directional_faces = {glm::vec3(4.0f, 0.03f, 0.03f), glm::vec3(0.03f, 0.03f, 4.0f),
                                                      glm::vec3(0.03f, 4.0f, 0.03f), glm::vec3(4.0f, 4.0f, 0.03f),
                                                      glm::vec3(4.0f, 0.03f, 4.0f),  glm::vec3(0.03f, 4.0f, 4.0f)};
  const std::array<float, GlobalReflectionProbe::kMipLevels> mip_scale = {1.0f,  0.82f, 0.64f, 0.48f, 0.34f,
                                                                          0.24f, 0.16f, 0.10f, 0.05f};
  const auto directional = CreateSyntheticReflectionProbe(directional_faces, mip_scale);

  AddReflectionProbeValidationLocalProbe(*lighting, "Reflection Probe Red Box", {-1.6f, 0.8f, -2.4f}, red, 10,
                                         EnvironmentalLighting::LocalReflectionProbeShape::Box, {4.0f, 2.0f, 3.0f},
                                         1.0f, 0.6f);
  AddReflectionProbeValidationLocalProbe(*lighting, "Reflection Probe Blue Box", {1.6f, 0.8f, -2.4f}, blue, 5,
                                         EnvironmentalLighting::LocalReflectionProbeShape::Box, {4.0f, 2.0f, 3.0f},
                                         1.0f, 0.6f);
  AddReflectionProbeValidationLocalProbe(*lighting, "Reflection Probe Nested Sphere", {0.0f, 0.8f, -2.4f}, green, 10,
                                         EnvironmentalLighting::LocalReflectionProbeShape::Sphere, glm::vec3(1.0f),
                                         0.58f, 0.16f);
  AddReflectionProbeValidationLocalProbe(*lighting, "Reflection Probe Directional Box", {0.0f, 2.05f, -2.4f},
                                         directional, 15, EnvironmentalLighting::LocalReflectionProbeShape::Box,
                                         {3.04f, 1.44f, 3.5f}, 1.0f, 0.3f, true,
                                         glm::radians(glm::vec3(0.0f, 18.0f, 0.0f)), {1.25f, 1.0f, 0.8f});
  AddReflectionProbeValidationLocalProbe(*lighting, "Reflection Probe Fallback", {0.0f, -0.55f, -2.4f}, magenta, 100,
                                         EnvironmentalLighting::LocalReflectionProbeShape::Sphere, glm::vec3(1.0f),
                                         0.72f, 0.15f);
  SyncTemporaryEnvironmentalLightingSettingsFromScene(scene);
}

bool evo_engine::RunRenderingSponzaProbeAuthoringFromEnvironment() {
  if (!SponzaProbeAuthoringRequested()) {
    return false;
  }

  const auto resource_root = FindDemoResourcesRoot();
  const auto expected_project =
      resource_root / ".generated/EvoEngine-DemoProjects/RenderingRegression/RenderingRegression.eveproj";
  if (resource_root.empty() || !std::filesystem::exists(resource_root.parent_path() / ".git") ||
      path_utils::NormalizeAbsolutePath(ProjectManager::GetProjectPath()) !=
          path_utils::NormalizeAbsolutePath(expected_project)) {
    throw std::runtime_error(
        "Sponza probe authoring must run from a source checkout with the rendering-regression demo.");
  }

  const auto scene = ApplicationContext::Get().GetActiveScene();
  const auto render_layer = ApplicationContext::Get().GetLayer<RenderLayer>();
  if (!scene || !render_layer) {
    throw std::runtime_error("Sponza probe authoring requires an active scene and RenderLayer.");
  }
  const auto main_camera = scene->main_camera.Get<Camera>();
  if (!main_camera) {
    throw std::runtime_error("Sponza probe authoring requires a main camera.");
  }
  const auto main_camera_entity = main_camera->GetOwner();
  if (const auto parent = scene->GetParent(main_camera_entity); scene->IsEntityValid(parent)) {
    scene->RemoveChild(main_camera_entity, parent);
  }
  std::vector<Entity> entities;
  scene->GetAllEntities(entities);
  for (const auto& entity : entities) {
    if (entity != main_camera_entity) {
      scene->DeleteEntity(entity);
    }
  }
  ConfigureRenderingDemoScene(scene);
  if (const auto lighting = scene->environmental_lighting.Get<EnvironmentalLighting>()) {
    lighting->dynamic_reflection_probe_settings.enabled = false;
  }

  auto& ddgi = RequireEnvironmentalLightingDdgiSettings(scene);
  ddgi.runtime.enabled = true;
  SetDdgiUpdatesPaused(false);
  RequestDdgiHistoryReset();
  ddgi.runtime.deterministic_ray_seed_enabled = true;
  ddgi.runtime.deterministic_ray_seed = 0x6d2b79f5u;
  DisableDdgiDebugVisualization();

  constexpr size_t kMaximumWaitFrames = 30000;
  size_t ready_input_frames = 0;
  for (size_t frame = 0; frame < kMaximumWaitFrames && ready_input_frames < 4u; ++frame) {
    const bool ready = ProjectManager::IsProjectIdle() && !AssetManager::GetAssetLoadSnapshot().Active() &&
                       !TextureStorage::HasPendingUploads() && !GeometryStorage::HasPendingUploads() &&
                       !BottomLevelAccelerationStructure::HasPendingStaticBuilds();
    ready_input_frames = ready ? ready_input_frames + 1u : 0u;
    if (ready_input_frames < 4u && !ApplicationContext::Get().Loop()) {
      throw std::runtime_error("Application ended before Sponza authoring inputs were ready.");
    }
  }
  if (ready_input_frames < 4u) {
    throw std::runtime_error("Sponza authoring input readiness timed out.");
  }

  size_t ready_ddgi_frames = 0;
  for (size_t frame = 0; frame < kMaximumWaitFrames && ready_ddgi_frames < 4u; ++frame) {
    if (!ApplicationContext::Get().Loop()) {
      throw std::runtime_error("Application ended during Sponza authoring DDGI convergence.");
    }
    const auto runtime = render_layer->GetDdgiInspectorSnapshot().volumes;
    const auto& performance = render_layer->GetDdgiInspectorSnapshot().aggregate;
    const bool volume_ready = runtime.size() == 1u && runtime[0].probe_count == 960u && runtime[0].resources_ready &&
                              runtime[0].has_valid_probe_history && runtime[0].contributes_lighting;
    const bool ready = volume_ready && performance.active_probe_count == 960u &&
                       performance.lighting_descriptors_bound && performance.probe_variability_converged &&
                       !performance.probe_warmup_active;
    ready_ddgi_frames = ready ? ready_ddgi_frames + 1u : 0u;
  }
  if (ready_ddgi_frames < 4u) {
    throw std::runtime_error("Sponza authoring DDGI convergence timed out.");
  }
  SetDdgiUpdatesPaused(true);

  const auto lighting = scene->environmental_lighting.Get<EnvironmentalLighting>();
  if (!lighting) {
    throw std::runtime_error("Sponza authoring requires an EnvironmentalLighting asset.");
  }
  std::array<EnvironmentalLighting::LocalReflectionProbe*, kSponzaLocalProbeNames.size()> probes;
  for (size_t index = 0; index < probes.size(); ++index) {
    probes[index] = FindEnvironmentalLightingLocalReflectionProbe(scene, kSponzaLocalProbeNames[index]);
    if (!probes[index]) {
      throw std::runtime_error(std::string("Sponza authoring probe is missing: ") + kSponzaLocalProbeNames[index]);
    }
  }
  std::vector<const EnvironmentalLighting::LocalReflectionProbe*> batch_probes(probes.begin(), probes.end());
  Platform::SetGpuTimestampCaptureEnabled(true);
  Platform::ResetGpuTimestampStats();
  const auto bake_start = std::chrono::steady_clock::now();
  RunEnvironmentalLightingLocalProbeBakeBatch(scene, *render_layer, batch_probes, "Sponza");
  const double bake_wall_milliseconds =
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - bake_start).count();
  for (int frame = 0; frame < Platform::kMaxFramesInFlight; ++frame) {
    if (!ApplicationContext::Get().Loop()) {
      throw std::runtime_error("Application ended while resolving Sponza reflection probe timing scopes.");
    }
  }
  const auto find_timing = [](const std::vector<GpuTimestampStats>& timings, const std::string& name) {
    const auto found = std::find_if(timings.begin(), timings.end(), [&](const auto& timing) {
      return timing.name == name;
    });
    return found == timings.end() ? GpuTimestampStats{} : *found;
  };
  const auto gpu_total = find_timing(Platform::GetGpuTimestampStats(), "Reflection Probe Bake GPU Total");
  const auto cpu_timings = Platform::GetCpuTimingStats();
  const auto cpu_prepare = find_timing(cpu_timings, "Reflection Probe Bake Prepare CPU");
  const auto cpu_record = find_timing(cpu_timings, "Reflection Probe Bake Record CPU");
  std::cout << "EVOENGINE_SPONZA_LOCAL_PROBE_BATCH_TIMING probes=" << probes.size()
            << " wall_ms=" << bake_wall_milliseconds
            << " cpu_total_ms=" << cpu_prepare.last_milliseconds + cpu_record.last_milliseconds
            << " cpu_prepare_ms=" << cpu_prepare.last_milliseconds << " cpu_record_ms=" << cpu_record.last_milliseconds
            << " gpu_total_ms=" << gpu_total.last_milliseconds << std::endl;
  if (std::string(std::getenv("EVOENGINE_SPONZA_PROBE_AUTHORING")) == "benchmark") {
    std::cout << "EVOENGINE_SPONZA_PROBE_AUTHORING_COMPLETE assets=0" << std::endl;
    return true;
  }

  const auto output_directory = resource_root / "EvoEngine-DemoProjects/Rendering/Assets" / kSponzaLightingDirectory;
  std::filesystem::create_directories(output_directory);
  const auto environment_source = output_directory / std::filesystem::path(kSponzaEnvironmentPath).filename();
  if (!std::filesystem::exists(environment_source) || std::filesystem::file_size(environment_source) == 0u) {
    throw std::runtime_error("The tracked Sponza sky environment is missing.");
  }
  const auto export_probe = [&](const std::shared_ptr<GlobalReflectionProbe>& asset,
                                const std::filesystem::path& relative_path) {
    if (!asset || !asset->IsRuntimeReady() || !asset->Export(output_directory / relative_path.filename()) ||
        asset->GetCanonicalPayloadByteSize() != GlobalReflectionProbe::kCanonicalPayloadByteSize ||
        asset->GetPayloadHash() == 0u) {
      throw std::runtime_error("Sponza reflection probe export failed: " + relative_path.string());
    }
  };
  export_probe(scene->GetGlobalReflectionProbeFallback(false), kSponzaGlobalProbePath);
  const auto pack = lighting->GetReflectionProbePack();
  if (!pack || pack->probes.size() != probes.size()) {
    throw std::runtime_error("Sponza reflection probe pack is incomplete.");
  }
  for (const auto& probe : pack->probes) {
    if (!probe.payload || probe.payload->GetSourceKind() != GlobalReflectionProbe::SourceKind::Baked ||
        !probe.payload->IsRuntimeReady()) {
      throw std::runtime_error("Sponza reflection probe pack contains an incomplete bake.");
    }
  }
  const auto pack_path = output_directory / std::filesystem::path(kSponzaLocalProbePackPath).filename();
  if (!pack->Export(pack_path) ||
      std::filesystem::file_size(pack_path) <= probes.size() * GlobalReflectionProbe::kCanonicalPayloadByteSize) {
    throw std::runtime_error("Sponza reflection probe pack export failed.");
  }
  std::cout << "EVOENGINE_SPONZA_PROBE_AUTHORING_COMPLETE assets=3" << std::endl;
  return true;
}

// DDGI_VALIDATION_FIXTURE_IMPLEMENTATION_BEGIN
void evo_engine::ConfigureDdgiValidationFixture(const std::shared_ptr<Scene>& scene, const std::string& fixture_id) {
  if (!scene) {
    return;
  }

  const auto main_camera = scene->main_camera.Get<Camera>();
  if (!main_camera) {
    throw std::runtime_error("DDGI validation fixture requires a main camera.");
  }
  const auto main_camera_entity = main_camera->GetOwner();
  if (const auto parent = scene->GetParent(main_camera_entity); scene->IsEntityValid(parent)) {
    scene->RemoveChild(main_camera_entity, parent);
  }
  std::vector<Entity> existing_entities;
  scene->GetAllEntities(existing_entities);
  for (const auto& entity : existing_entities) {
    if (entity != main_camera_entity) {
      scene->DeleteEntity(entity);
    }
  }
  rendering_regression_temporal_motion_state.reset();

  const bool furnace_fixture = fixture_id == "furnace";
  const auto lighting = GetOrCreateTemporaryEnvironmentalLighting(scene);
  if (!lighting) {
    throw std::runtime_error("DDGI validation fixture requires an environmental lighting asset.");
  }
  ConfigureEnvironmentalLightingColorSource(*lighting, furnace_fixture ? glm::vec3(1.0f) : glm::vec3(0.0f), 0.0f, 1.0f);

  const auto configure_validation_camera = [&](const glm::vec3& position, const glm::quat& rotation, const float fov,
                                               const float near_distance, const float far_distance) {
    const auto configure_camera = [&](const std::shared_ptr<Camera>& camera) {
      if (!camera) {
        return;
      }
      camera->Resize({1920, 1080});
      camera->skybox.Clear();
      camera->camera_render_mode = Camera::CameraRenderMode::Rasterization;
      camera->camera_settings.fov = fov;
      camera->camera_settings.near_distance = near_distance;
      camera->camera_settings.far_distance = far_distance;
      camera->camera_settings.background_source = Camera::BackgroundSource::Cubemap;
      camera->camera_settings.clear_color = glm::vec4(0.0f, 0.0f, 0.0f, 1.0f);
      camera->camera_settings.background_intensity = 1.0f;
      camera->camera_settings.sample_size = 1;
      camera->camera_settings.bounce = 4;
      camera->camera_settings.auto_spp_enabled = false;
      camera->post_processing_stack_ref = AssetManager::CreateTemporaryAsset<PostProcessingStack>();
      if (const auto stack = camera->post_processing_stack_ref.Get<PostProcessingStack>()) {
        stack->enable_ambient_occlusion = false;
        stack->enable_bloom = false;
        stack->enable_screen_space_reflection = false;
        stack->enable_anti_aliasing = false;
        stack->enable_tone_mapping = true;
        if (stack->tone_mapping) {
          stack->tone_mapping->method = ToneMapping::ToneMapMethod::Filmic;
          stack->tone_mapping->exposure = 1.0f;
          stack->tone_mapping->brightness = 1.0f;
          stack->tone_mapping->contrast = 1.0f;
          stack->tone_mapping->saturation = 1.0f;
          stack->tone_mapping->vignette = 0.0f;
          stack->tone_mapping->auto_exposure = false;
          stack->tone_mapping->dither = false;
        }
      }
      camera->ResetFrameCount();
    };
    configure_camera(main_camera);
    Transform camera_transform;
    camera_transform.SetValue(position, rotation, glm::vec3(1.0f));
    scene->SetDataComponent(main_camera_entity, camera_transform);
    if (const auto editor_layer = ApplicationContext::Get().GetLayer<EditorLayer>()) {
      editor_layer->enable_gizmos = false;
      editor_layer->SetSelectedEntity({});
      editor_layer->default_scene_camera_position = position;
      editor_layer->default_scene_camera_rotation = rotation;
      editor_layer->SetSceneCameraPosition(position);
      editor_layer->SetSceneCameraRotation(rotation);
      configure_camera(editor_layer->GetSceneCamera());
    }
  };

  auto& ddgi = lighting->ddgi_settings;
  ddgi = DdgiSettings{};
  const bool high_contrast_fixture = fixture_id == "alpha-tested" || fixture_id == "scrolling" ||
                                     (fixture_id.rfind("emissive-", 0) == 0 && fixture_id != "emissive-empty");
  const float probe_variability_threshold = high_contrast_fixture ? kDdgiValidationHighContrastProbeVariabilityThreshold
                                                                  : kDdgiValidationNeutralProbeVariabilityThreshold;
  const auto configure_validation_ddgi_settings = [&](const int ray_count) {
    ddgi.runtime.enabled = true;
    SetDdgiUpdatesPaused(false);
    ddgi.runtime.ray_count = ray_count;
    ddgi.runtime.warmup_frames = 32;
    RequestDdgiHistoryReset();
    DisableDdgiDebugVisualization();
    ddgi.volume_defaults.enable_probe_relocation = true;
    ddgi.volume_defaults.enable_probe_classification = true;
    ddgi.volume_defaults.enable_probe_variability = true;
    ddgi.volume_defaults.enable_probe_variability_gating = true;
    ddgi.volume_defaults.probe_variability_threshold = probe_variability_threshold;
    ddgi.volume_defaults.probe_variability_min_samples = kDdgiValidationProbeVariabilityMinSamples;
  };
  configure_validation_ddgi_settings(128);

  const auto configure_assigned_lighting_volumes = [&]() {
    if (const auto assigned_lighting = scene->environmental_lighting.Get<EnvironmentalLighting>()) {
      assigned_lighting->ddgi_settings = ddgi;
      for (auto& volume : assigned_lighting->GetOrCreateDdgiVolumePack()->volumes) {
        volume.enable_probe_relocation = true;
        volume.enable_probe_classification = true;
        volume.enable_probe_variability = true;
        volume.enable_probe_variability_gating = true;
        volume.probe_variability_threshold = probe_variability_threshold;
        volume.probe_variability_min_samples = kDdgiValidationProbeVariabilityMinSamples;
      }
    }
  };

  if (fixture_id == "cornell") {
    ConfigureCornellBoxScene(scene);
    configure_validation_ddgi_settings(128);
    configure_assigned_lighting_volumes();
    configure_validation_camera(glm::vec3(0.0f, 0.0f, 1.6f), glm::quat(glm::vec3(0.0f)), 120.0f, 0.1f, 200.0f);
    const auto lighting = scene->environmental_lighting.Get<EnvironmentalLighting>();
    if (!FindEntityNamed(scene, "Cornell Box") || !lighting || lighting->GetOrCreateDdgiVolumePack()->volumes.empty()) {
      throw std::runtime_error("DDGI Cornell fixture failed to construct its canonical scene and probe volume.");
    }
    return;
  }
  if (fixture_id == "sponza") {
    ConfigureRenderingDemoScene(scene);
    for (const auto& name : {"Spheres", "Title"}) {
      if (const auto entity = FindEntityNamed(scene, name)) {
        scene->SetEnable(*entity, false);
      }
    }
    configure_validation_ddgi_settings(256);
    configure_assigned_lighting_volumes();
    configure_validation_camera(glm::vec3(0.0f, 0.0f, 3.0f), glm::quat(glm::vec3(0.0f)), 120.0f, 0.1f, 200.0f);
    const auto lighting = scene->environmental_lighting.Get<EnvironmentalLighting>();
    if (!FindEntityNamed(scene, "Rendering Demo") || !lighting ||
        lighting->GetOrCreateDdgiVolumePack()->volumes.empty()) {
      throw std::runtime_error("DDGI Sponza fixture failed to construct its canonical scene and probe volume.");
    }
    return;
  }

  const auto root = scene->CreateEntity("DDGI Validation Fixture");
  Transform volume_transform;
  volume_transform.SetPosition(glm::vec3(0.0f, 0.8f, -2.4f));
  lighting->GetOrCreateReflectionProbePack()->probes.clear();
  lighting->GetOrCreateDdgiVolumePack()->volumes.clear();
  auto& volume = AddEnvironmentalLightingDdgiVolume(*lighting, "DDGI Validation Volume", volume_transform.value,
                                                    {8, 6, 8}, glm::vec3(0.6f), glm::vec3(0.0f));
  ConfigureDdgiValidationVolume(volume, fixture_id, probe_variability_threshold);

  const glm::vec3 camera_position(0.0f, 1.25f, 5.0f);
  const glm::vec3 camera_target(0.0f, 0.8f, -2.4f);
  const auto camera_rotation =
      glm::quatLookAt(glm::normalize(camera_target - camera_position), glm::vec3(0.0f, 1.0f, 0.0f));
  configure_validation_camera(camera_position, camera_rotation, 48.0f, 0.05f, 250.0f);

  const auto& primitives = Resources::GetInstance().GetPrimitives();
  if (fixture_id != "furnace") {
    CreateRenderingRegressionProbe(scene, root, "DDGI Validation Receiver Floor", primitives.cube,
                                   glm::vec3(0.0f, -0.05f, -2.4f), glm::vec3(2.2f, 0.05f, 2.0f), glm::vec3(0.72f), 0.9f,
                                   0.0f);
    CreateRenderingRegressionProbe(scene, root, "DDGI Validation Receiver Wall", primitives.cube,
                                   glm::vec3(0.0f, 1.25f, -4.35f), glm::vec3(2.2f, 1.3f, 0.05f),
                                   glm::vec3(0.64f, 0.68f, 0.74f), 0.9f, 0.0f);
  }

  const auto create_emitter = [&](const std::string& name, const glm::vec3& scale, const bool double_sided,
                                  const int uv_set = -1, const bool alpha_masked = false, const bool face_away = false,
                                  const float emission = kDdgiValidationEmitterRadiance) {
    const auto mesh = uv_set >= 0 || alpha_masked
                          ? CreateRenderingRegressionMaterialQuad(
                                {glm::vec4(1.0f, 1.0f, 1.0f, alpha_masked ? 0.0f : 1.0f), glm::vec4(1.0f),
                                 glm::vec4(1.0f), glm::vec4(1.0f, 1.0f, 1.0f, alpha_masked ? 0.0f : 1.0f)})
                          : primitives.cube;
    const auto entity = CreateRenderingRegressionProbe(scene, root, name, mesh, glm::vec3(0.0f, 2.45f, -2.4f), scale,
                                                       glm::vec3(1.0f, 0.48f, 0.16f), 0.85f, 0.0f, emission, false);
    const auto material = scene->GetOrSetPrivateComponent<MeshRenderer>(entity).lock()->material.Get<Material>();
    auto& shade = material->material_data.shade_material;
    shade.double_sided = double_sided ? 1 : 0;
    if (uv_set >= 0) {
      const auto texture = AssetManager::CreateTemporaryAsset<Texture2D>();
      texture->SetRgbaChannelData({glm::vec4(1.0f, 0.08f, 0.02f, 1.0f), glm::vec4(0.02f, 0.2f, 1.0f, 1.0f),
                                   glm::vec4(0.02f, 1.0f, 0.12f, 1.0f), glm::vec4(1.0f, 0.8f, 0.04f, 1.0f)},
                                  glm::uvec2(2));
      const auto slot = material->SetTexture(&GltfShadeMaterial::emissive_texture, texture, uv_set);
      material->material_data.texture_infos[slot].color_space = static_cast<int32_t>(GltfTextureColorSpace::Srgb);
    }
    if (alpha_masked) {
      shade.alpha_mode = static_cast<int32_t>(GltfAlphaMode::Mask);
      shade.alpha_cutoff = 0.5f;
    }
    material->MarkDirty();
    if (uv_set >= 0 || alpha_masked) {
      Transform transform;
      const float rotation = face_away ? -90.0f : 90.0f;
      transform.SetValue(glm::vec3(0.0f, 2.45f, -2.4f), glm::radians(glm::vec3(rotation, 0.0f, 0.0f)), scale);
      scene->SetDataComponent(entity, transform);
    }
    return entity;
  };

  if (fixture_id == "furnace") {
    CreateRenderingRegressionProbe(scene, root, "DDGI Furnace Center", primitives.sphere, glm::vec3(0.0f, 0.85f, -2.4f),
                                   glm::vec3(1.6f), glm::vec3(1.0f), 1.0f, 0.0f);
  } else if (fixture_id == "alpha-tested") {
    create_emitter("DDGI Alpha Fixture Emitter", glm::vec3(0.8f, 0.02f, 0.8f), true);
    const auto blocker_mesh = CreateRenderingRegressionMaterialQuad(
        {glm::vec4(1.0f, 1.0f, 1.0f, 0.0f), glm::vec4(1.0f), glm::vec4(1.0f), glm::vec4(1.0f, 1.0f, 1.0f, 0.0f)});
    const auto blocker_material = AssetManager::CreateTemporaryAsset<Material>();
    blocker_material->material_data.shade_material.alpha_mode = static_cast<int32_t>(GltfAlphaMode::Mask);
    blocker_material->material_data.shade_material.alpha_cutoff = 0.5f;
    blocker_material->MarkDirty();
    CreateRenderingRegressionMaterialQuadEntity(scene, root, "DDGI Alpha Fixture Blocker", blocker_mesh,
                                                blocker_material, glm::vec3(0.0f, 1.25f, -2.4f),
                                                glm::radians(glm::vec3(90.0f, 0.0f, 0.0f)), glm::vec3(1.2f));
  } else if (fixture_id == "emissive-large") {
    create_emitter("DDGI Large Area Emitter", glm::vec3(1.35f, 0.025f, 1.1f), true);
  } else if (fixture_id.rfind("emissive-textured-uv", 0) == 0) {
    const int uv_set = fixture_id.back() - '0';
    create_emitter("DDGI Textured Emitter", glm::vec3(1.0f, 0.8f, 1.0f), true, uv_set);
  } else if (fixture_id == "emissive-one-sided") {
    create_emitter("DDGI One Sided Emitter", glm::vec3(1.0f, 0.8f, 1.0f), false, 0, false, true);
  } else if (fixture_id == "emissive-double-sided") {
    create_emitter("DDGI Double Sided Emitter", glm::vec3(1.0f, 0.8f, 1.0f), true, 0, false, true);
  } else if (fixture_id == "emissive-alpha-cutout") {
    create_emitter("DDGI Alpha Cutout Emitter", glm::vec3(1.0f, 0.8f, 1.0f), true, 0, true);
  } else if (fixture_id == "emissive-multi") {
    const auto left = create_emitter("DDGI Multi Emitter Left", glm::vec3(0.45f, 0.025f, 0.45f), true);
    const auto right = create_emitter("DDGI Multi Emitter Right", glm::vec3(0.25f, 0.35f, 0.25f), true, 3);
    Transform left_transform;
    left_transform.SetValue(glm::vec3(-0.85f, 2.3f, -2.4f), glm::vec3(0.0f), glm::vec3(0.45f, 0.025f, 0.45f));
    scene->SetDataComponent(left, left_transform);
    Transform right_transform;
    right_transform.SetValue(glm::vec3(0.85f, 1.8f, -2.4f), glm::radians(glm::vec3(90.0f, 0.0f, 0.0f)),
                             glm::vec3(0.25f, 0.35f, 0.25f));
    scene->SetDataComponent(right, right_transform);
  } else if (fixture_id == "emissive-moving-rigid") {
    create_emitter("DDGI Moving Rigid Emitter", glm::vec3(0.5f, 0.05f, 0.5f), true);
  } else if (fixture_id == "emissive-enable") {
    create_emitter("DDGI Enabled Emitter", glm::vec3(0.16f, 0.025f, 0.16f), true, -1, false, false, 0.0f);
  } else if (fixture_id == "emissive-disable") {
    create_emitter("DDGI Disabled Emitter", glm::vec3(0.16f, 0.025f, 0.16f), true, -1, false, false,
                   kDdgiValidationEqualPowerSmallEmitterRadiance);
  } else if (fixture_id == "emissive-enable-hdr") {
    create_emitter("DDGI HDR Enabled Emitter", glm::vec3(0.16f, 0.025f, 0.16f), true, -1, false, false, 0.0f);
  } else if (fixture_id == "geometry-moving") {
    create_emitter("DDGI Geometry Motion Emitter", glm::vec3(0.4f, 0.025f, 0.4f), true);
    CreateRenderingRegressionProbe(scene, root, "DDGI Moving Occluder", primitives.cube, glm::vec3(-0.6f, 1.2f, -2.4f),
                                   glm::vec3(0.18f, 0.4f, 0.18f), glm::vec3(0.35f), 0.9f, 0.0f);
  } else if (fixture_id == "analytic-light") {
    const auto light_entity = scene->CreateEntity("DDGI Analytic Point Light");
    const auto point_light = scene->GetOrSetPrivateComponent<PointLight>(light_entity).lock();
    point_light->cast_shadow = true;
    point_light->diffuse = glm::vec3(1.0f, 0.48f, 0.16f);
    point_light->diffuse_brightness = 45.0f;
    point_light->light_size = 0.08f;
    point_light->constant = 1.0f;
    point_light->linear = 0.08f;
    point_light->quadratic = 0.02f;
    Transform transform;
    transform.SetPosition(glm::vec3(0.0f, 2.2f, -2.4f));
    scene->SetDataComponent(light_entity, transform);
    scene->SetParent(light_entity, root);
  } else if (fixture_id == "emissive-direct-hit") {
    const auto emitter = create_emitter("DDGI Direct Hit Emitter", glm::vec3(0.9f), true, 0);
    Transform transform;
    transform.SetValue(glm::vec3(0.0f, 1.0f, -2.8f), glm::vec3(0.0f), glm::vec3(0.9f));
    scene->SetDataComponent(emitter, transform);
  } else if (fixture_id == "emissive-small-equal-power") {
    create_emitter("DDGI Equal Power Small Emitter", glm::vec3(0.16f, 0.025f, 0.16f), true, -1, false, false,
                   kDdgiValidationEqualPowerSmallEmitterRadiance);
  } else if (fixture_id != "emissive-empty") {
    create_emitter("DDGI Small Bright Emitter", glm::vec3(0.16f, 0.025f, 0.16f), true);
  }
  SyncTemporaryEnvironmentalLightingSettingsFromScene(scene);
}

bool evo_engine::AdvanceDdgiValidationFixture(const std::shared_ptr<Scene>& scene, const std::string& fixture_id) {
  if (!scene) {
    return false;
  }
  if (fixture_id == "scrolling") {
    if (auto* volume = FindEnvironmentalLightingDdgiVolume(scene, "DDGI Validation Volume")) {
      Transform transform;
      transform.value = volume->transform;
      transform.SetPosition(transform.GetPosition() + glm::vec3(0.6f, 0.0f, 0.0f));
      volume->transform = transform.value;
      SyncTemporaryEnvironmentalLightingSettingsFromScene(scene);
      return true;
    }
    return false;
  }
  if (fixture_id == "emissive-moving-rigid") {
    const auto entity = FindEntityNamed(scene, "DDGI Moving Rigid Emitter");
    if (!entity) {
      return false;
    }
    auto transform = scene->GetDataComponent<Transform>(*entity);
    transform.SetPosition(transform.GetPosition() + glm::vec3(0.6f, 0.0f, 0.0f));
    scene->SetDataComponent(*entity, transform);
    SyncTemporaryEnvironmentalLightingSettingsFromScene(scene);
    return true;
  }
  if (fixture_id == "emissive-enable") {
    const auto entity = FindEntityNamed(scene, "DDGI Enabled Emitter");
    if (!entity) {
      return false;
    }
    const auto renderer = scene->GetOrSetPrivateComponent<MeshRenderer>(*entity).lock();
    const auto material = renderer ? renderer->material.Get<Material>() : nullptr;
    if (!material) {
      return false;
    }
    ConfigureMaterial(material, glm::vec3(1.0f, 0.48f, 0.16f), 0.85f, 0.0f,
                      kDdgiValidationEqualPowerSmallEmitterRadiance);
    SyncTemporaryEnvironmentalLightingSettingsFromScene(scene);
    return true;
  }
  if (fixture_id == "emissive-disable") {
    const auto entity = FindEntityNamed(scene, "DDGI Disabled Emitter");
    const auto renderer = entity ? scene->GetOrSetPrivateComponent<MeshRenderer>(*entity).lock() : nullptr;
    const auto material = renderer ? renderer->material.Get<Material>() : nullptr;
    if (!material) {
      return false;
    }
    ConfigureMaterial(material, glm::vec3(1.0f, 0.48f, 0.16f), 0.85f, 0.0f, 0.0f);
    SyncTemporaryEnvironmentalLightingSettingsFromScene(scene);
    return true;
  }
  if (fixture_id == "emissive-enable-hdr") {
    const auto entity = FindEntityNamed(scene, "DDGI HDR Enabled Emitter");
    const auto renderer = entity ? scene->GetOrSetPrivateComponent<MeshRenderer>(*entity).lock() : nullptr;
    const auto material = renderer ? renderer->material.Get<Material>() : nullptr;
    if (!material) {
      return false;
    }
    ConfigureMaterial(material, glm::vec3(1.0f, 0.48f, 0.16f), 0.85f, 0.0f,
                      4.0f * kDdgiValidationEqualPowerSmallEmitterRadiance);
    SyncTemporaryEnvironmentalLightingSettingsFromScene(scene);
    return true;
  }
  if (fixture_id == "geometry-moving") {
    const auto entity = FindEntityNamed(scene, "DDGI Moving Occluder");
    if (!entity) {
      return false;
    }
    auto transform = scene->GetDataComponent<Transform>(*entity);
    transform.SetPosition(transform.GetPosition() + glm::vec3(1.2f, 0.0f, 0.0f));
    scene->SetDataComponent(*entity, transform);
    SyncTemporaryEnvironmentalLightingSettingsFromScene(scene);
    return true;
  }
  return false;
}
// DDGI_VALIDATION_FIXTURE_IMPLEMENTATION_END

bool evo_engine::RunReflectionProbeValidationFromEnvironment(const int width, const int height) {
  const auto* evidence_path = std::getenv("EVOENGINE_REFLECTION_PROBE_EVIDENCE");
  if (!evidence_path) {
    return false;
  }
  if (width != 1920 || height != 1080) {
    throw std::runtime_error("Reflection probe validation requires 1920x1080.");
  }
  if (!Platform::GraphicsValidationEnabled()) {
    throw std::runtime_error("Reflection probe validation requires Vulkan validation.");
  }
  Platform::SetGpuTimestampCaptureEnabled(true);
  if (!Platform::GpuTimestampCaptureAvailable() || !Platform::GpuTimestampCaptureEnabled()) {
    throw std::runtime_error("Reflection probe validation requires GPU timestamp capture.");
  }
  const auto output_directory = std::filesystem::path(evidence_path);
  if (output_directory.empty()) {
    throw std::runtime_error("Reflection probe validation evidence path is empty.");
  }
  std::filesystem::create_directories(output_directory);

  const auto scene = ApplicationContext::Get().GetActiveScene();
  const auto editor_layer = ApplicationContext::Get().GetLayer<EditorLayer>();
  const auto render_layer = ApplicationContext::Get().GetLayer<RenderLayer>();
  if (!scene || !editor_layer || !render_layer) {
    throw std::runtime_error("Reflection probe validation requires an active scene, EditorLayer, and RenderLayer.");
  }
  ConfigureReflectionProbeValidationScene(scene);
  if (const auto window_layer = ApplicationContext::Get().GetLayer<WindowLayer>()) {
    window_layer->ResizeWindow(width, height);
    window_layer->CenterWindow();
  }
  const glm::uvec2 resolution(static_cast<uint32_t>(width), static_cast<uint32_t>(height));
  editor_layer->show_camera_window = false;
  editor_layer->RequestSceneCameraPreviewWindow(resolution);
  editor_layer->SetSceneCameraResolutionOverride(resolution);
  const auto scene_camera = editor_layer->GetSceneCamera();
  if (!scene_camera) {
    throw std::runtime_error("Reflection probe validation requires a scene camera.");
  }
  scene_camera->SetRequireRendering(true);
  scene_camera->Resize(resolution);
  scene_camera->ResetFrameCount();
  if (const auto main_camera = scene->main_camera.Get<Camera>(); main_camera && main_camera != scene_camera) {
    main_camera->SetEnabled(false);
  }
  constexpr size_t max_wait_frames = 30000;
  size_t ready_frames = 0;
  for (size_t frame = 0; frame < max_wait_frames && ready_frames < 4u; ++frame) {
    const bool ready = ProjectManager::IsProjectIdle() && !AssetManager::GetAssetLoadSnapshot().Active() &&
                       !TextureStorage::HasPendingUploads() && !GeometryStorage::HasPendingUploads() &&
                       !BottomLevelAccelerationStructure::HasPendingStaticBuilds();
    ready_frames = ready ? ready_frames + 1u : 0u;
    if (ready_frames < 4u && !ApplicationContext::Get().Loop()) {
      throw std::runtime_error("Application ended before reflection probe inputs were ready.");
    }
  }
  if (ready_frames < 4u) {
    throw std::runtime_error("Reflection probe scene input readiness timed out.");
  }

  const auto lighting = scene->environmental_lighting.Get<EnvironmentalLighting>();
  if (!lighting) {
    throw std::runtime_error("Reflection probe validation requires an EnvironmentalLighting asset.");
  }
  const bool dynamic_validation_requested = std::getenv("EVOENGINE_DYNAMIC_REFLECTION_PROBE_VALIDATION") != nullptr;
  lighting->dynamic_reflection_probe_settings.enabled = dynamic_validation_requested;
  bool dynamic_initial_b_sweep = !dynamic_validation_requested;
  bool dynamic_transition_progress = !dynamic_validation_requested;
  bool dynamic_a_sweep = !dynamic_validation_requested;
  bool dynamic_b_sweep = !dynamic_validation_requested;
  bool dynamic_contribution_resume = !dynamic_validation_requested;
  bool dynamic_reset = !dynamic_validation_requested;
  bool dynamic_returned_to_static = !dynamic_validation_requested;
  size_t dynamic_initial_latency_frames = 0u;
  size_t dynamic_bab_cycle_frames = 0u;
  double dynamic_bab_cycle_milliseconds = 0.0;
  double dynamic_blend_frame_milliseconds = 0.0;
  uint64_t dynamic_published_generations_observed = 0u;
  uint64_t dynamic_transient_gpu_bytes = 0u;
  double dynamic_update_gpu_ms = 0.0;
  double dynamic_capture_gpu_ms = 0.0;
  double dynamic_prefilter_gpu_ms = 0.0;
  if (dynamic_validation_requested) {
    lighting->ddgi_settings.runtime.enabled = false;
    lighting->dynamic_reflection_probe_settings.enabled = true;
    lighting->dynamic_reflection_probe_settings.faces_per_frame = 1;
    const auto dynamic_cycle_start = std::chrono::steady_clock::now();
    size_t dynamic_cycle_frame_count = 0u;
    size_t dynamic_blend_frame_count = 0u;
    double dynamic_blend_frame_milliseconds_total = 0.0;
    bool measure_dynamic_blend_frame_time = false;
    const auto run_dynamic_frame = [&]() {
      const auto frame_start = std::chrono::steady_clock::now();
      const bool running = ApplicationContext::Get().Loop();
      const double frame_milliseconds =
          std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - frame_start).count();
      ++dynamic_cycle_frame_count;
      if (measure_dynamic_blend_frame_time) {
        dynamic_blend_frame_milliseconds_total += frame_milliseconds;
        ++dynamic_blend_frame_count;
      }
      return running;
    };
    const auto wait_for_dynamic_generation = [&](const uint64_t minimum_generation, const size_t maximum_frames) {
      for (size_t frame = 0; frame < maximum_frames; ++frame) {
        const auto stats = render_layer->GetDynamicReflectionProbeStats();
        if (stats.active && stats.published_generation_count >= minimum_generation) {
          return stats;
        }
        if (!run_dynamic_frame()) {
          throw std::runtime_error("Application ended during dynamic reflection-probe validation.");
        }
      }
      return render_layer->GetDynamicReflectionProbeStats();
    };
    const auto record_dynamic_stats = [&](const RenderLayer::DynamicReflectionProbeStats& stats) {
      dynamic_published_generations_observed =
          std::max(dynamic_published_generations_observed, stats.published_generation_count);
      dynamic_transient_gpu_bytes = std::max(dynamic_transient_gpu_bytes, stats.transient_gpu_bytes);
      dynamic_update_gpu_ms = std::max(dynamic_update_gpu_ms, stats.last_update_gpu_ms);
      dynamic_capture_gpu_ms = std::max(dynamic_capture_gpu_ms, stats.last_capture_gpu_ms);
      dynamic_prefilter_gpu_ms = std::max(dynamic_prefilter_gpu_ms, stats.last_prefilter_gpu_ms);
    };
    const auto initial = wait_for_dynamic_generation(5u, 240u);
    dynamic_initial_latency_frames = dynamic_cycle_frame_count;
    record_dynamic_stats(initial);
    dynamic_initial_b_sweep = initial.active && initial.published_generation_count >= 5u &&
                              initial.generation_a_probe_count == 0u && initial.generation_b_probe_count == 5u &&
                              initial.transient_gpu_bytes >= 10u * GlobalReflectionProbe::kCanonicalPayloadByteSize;
    measure_dynamic_blend_frame_time = true;
    for (size_t frame = 0; frame < 120u && !dynamic_transition_progress; ++frame) {
      const auto stats = render_layer->GetDynamicReflectionProbeStats();
      record_dynamic_stats(stats);
      dynamic_transition_progress = stats.transitioning_probe_count > 0u && stats.maximum_transition_weight > 0.0f &&
                                    stats.minimum_transition_weight < 1.0f;
      if (!dynamic_transition_progress && !run_dynamic_frame()) {
        throw std::runtime_error("Application ended during dynamic reflection-probe transition validation.");
      }
    }
    const auto after_a = wait_for_dynamic_generation(10u, 240u);
    measure_dynamic_blend_frame_time = false;
    dynamic_blend_frame_milliseconds =
        dynamic_blend_frame_count == 0u
            ? 0.0
            : dynamic_blend_frame_milliseconds_total / static_cast<double>(dynamic_blend_frame_count);
    record_dynamic_stats(after_a);
    dynamic_a_sweep = after_a.published_generation_count >= 10u && after_a.generation_a_probe_count == 5u &&
                      after_a.generation_b_probe_count == 0u;
    const auto after_b = wait_for_dynamic_generation(15u, 240u);
    dynamic_bab_cycle_frames = dynamic_cycle_frame_count;
    dynamic_bab_cycle_milliseconds =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - dynamic_cycle_start).count();
    record_dynamic_stats(after_b);
    dynamic_b_sweep = after_b.published_generation_count >= 15u && after_b.generation_a_probe_count == 0u &&
                      after_b.generation_b_probe_count == 5u;

    lighting->local_reflection_probes_enabled = false;
    for (size_t frame = 0; frame < 4u; ++frame) {
      if (!ApplicationContext::Get().Loop()) {
        throw std::runtime_error("Application ended while disabling dynamic reflection probes.");
      }
    }
    const auto contribution_suspended = render_layer->GetDynamicReflectionProbeStats();
    const auto publications_match = [](const RenderLayer::DynamicReflectionProbeStats& lhs,
                                       const RenderLayer::DynamicReflectionProbeStats& rhs) {
      return lhs.published_generation_count == rhs.published_generation_count &&
             lhs.generation_a_probe_count == rhs.generation_a_probe_count &&
             lhs.generation_b_probe_count == rhs.generation_b_probe_count;
    };
    const bool contribution_suspended_without_reset =
        !contribution_suspended.active && publications_match(contribution_suspended, after_b);
    lighting->local_reflection_probes_enabled = true;
    const auto after_contribution = wait_for_dynamic_generation(after_b.published_generation_count, 8u);
    record_dynamic_stats(after_contribution);
    dynamic_contribution_resume = contribution_suspended_without_reset && after_contribution.active &&
                                  publications_match(after_contribution, after_b);

    render_layer->ResetDynamicReflectionProbeHistory();
    const auto after_reset = wait_for_dynamic_generation(5u, 240u);
    record_dynamic_stats(after_reset);
    dynamic_reset = after_reset.active && after_reset.published_generation_count >= 5u;
    lighting->dynamic_reflection_probe_settings.enabled = false;
    for (size_t frame = 0; frame < 4u; ++frame) {
      if (!ApplicationContext::Get().Loop()) {
        throw std::runtime_error("Application ended while returning to static reflection probes.");
      }
    }
    dynamic_returned_to_static = !render_layer->GetDynamicReflectionProbeStats().active;
  }
  auto reflection_pack = lighting->GetOrCreateReflectionProbePack();
  reflection_pack->probes.reserve(reflection_pack->probes.size() + 2u);
  auto& red_probe = RequireEnvironmentalLightingLocalReflectionProbe(*lighting, "Reflection Probe Red Box");
  auto& blue_probe = RequireEnvironmentalLightingLocalReflectionProbe(*lighting, "Reflection Probe Blue Box");
  auto& directional_probe =
      RequireEnvironmentalLightingLocalReflectionProbe(*lighting, "Reflection Probe Directional Box");
  (void)RequireEnvironmentalLightingLocalReflectionProbe(*lighting, "Reflection Probe Nested Sphere");
  (void)RequireEnvironmentalLightingLocalReflectionProbe(*lighting, "Reflection Probe Fallback");
  const auto red_asset = red_probe.payload;
  const auto blue_asset = blue_probe.payload;
  if (!red_asset || !blue_asset) {
    throw std::runtime_error("Reflection probe validation synthetic assets are unavailable.");
  }
  const auto get_material = [&](const char* name) {
    const auto entity = FindEntityNamed(scene, name);
    if (!entity || !scene->HasPrivateComponent<MeshRenderer>(*entity)) {
      throw std::runtime_error(std::string("Missing reflection probe material fixture: ") + name);
    }
    const auto renderer = scene->GetOrSetPrivateComponent<MeshRenderer>(*entity).lock();
    const auto material = renderer ? renderer->material.Get<Material>() : nullptr;
    if (!material) {
      throw std::runtime_error(std::string("Missing reflection probe fixture material: ") + name);
    }
    return std::pair{*entity, material};
  };
  const auto smooth_metal_fixture = get_material("Reflection Probe Smooth Metal");
  const auto rough_metal_fixture = get_material("Reflection Probe Rough Metal");
  const auto boundary_metal_fixture = get_material("Reflection Probe Boundary Metal");
  const auto smooth_metal_material = smooth_metal_fixture.second;
  const auto rough_metal_material = rough_metal_fixture.second;
  const auto boundary_metal_entity = boundary_metal_fixture.first;
  const auto boundary_metal_material = boundary_metal_fixture.second;
  const auto post_processing_stack = scene_camera->post_processing_stack_ref.Get<PostProcessingStack>();
  if (!post_processing_stack || !post_processing_stack->ambient_occlusion) {
    throw std::runtime_error("Reflection probe validation requires ambient-occlusion controls.");
  }

  struct TemporaryProjectFolder {
    std::shared_ptr<Folder> folder;
    ~TemporaryProjectFolder() {
      if (folder) {
        (void)ProjectManager::DeleteFolder(folder->GetHandle());
      }
    }
  } temporary_folder{ProjectManager::CreateFolder(ProjectManager::GetAssetsFolder(), "M14 Reflection Validation")};
  if (!temporary_folder.folder) {
    throw std::runtime_error("Could not create the temporary reflection probe bake folder.");
  }
  const auto first_bake = std::dynamic_pointer_cast<GlobalReflectionProbe>(
      ProjectManager::CreateAsset(temporary_folder.folder, "GlobalReflectionProbe"));
  const auto second_bake = std::dynamic_pointer_cast<GlobalReflectionProbe>(
      ProjectManager::CreateAsset(temporary_folder.folder, "GlobalReflectionProbe"));
  if (!first_bake || !second_bake) {
    throw std::runtime_error("Could not create persistent reflection probe bake assets.");
  }
  if (!first_bake->Save() || !second_bake->Save()) {
    throw std::runtime_error("Could not initialize persistent reflection probe bake assets.");
  }
  const std::array bake_assets = {first_bake, second_bake};
  const size_t bake_probe_base_index = reflection_pack->probes.size();
  std::array<size_t, 2> bake_probe_indices{};
  for (size_t index = 0; index < bake_probe_indices.size(); ++index) {
    bake_probe_indices[index] = reflection_pack->probes.size();
    auto& bake_probe = AddEnvironmentalLightingLocalReflectionProbe(
        *lighting, "Reflection Probe Bake Origin " + std::to_string(index + 1u),
        MakeAuthoringTransform({0.0f, 1.0f, -0.6f}), bake_assets[index], -100,
        EnvironmentalLighting::LocalReflectionProbeShape::Sphere, 0.1f, 0.0f);
    ClampEnvironmentalLightingLocalReflectionProbe(bake_probe);
  }
  const auto get_bake_probe =
      [&reflection_pack, &bake_probe_indices](const size_t index) -> EnvironmentalLighting::LocalReflectionProbe& {
    return reflection_pack->probes[bake_probe_indices[index]];
  };
  const bool unique_bake_entries =
      bake_probe_indices[0] != bake_probe_indices[1] && get_bake_probe(0).stable_id != get_bake_probe(1).stable_id;
  const bool unique_bake_assets = first_bake->GetHandle() != second_bake->GetHandle() &&
                                  first_bake->GetAbsolutePath() != second_bake->GetAbsolutePath();
  SetEnvironmentalLightingIntensity(scene, 0.75f);
  SetEnvironmentalLightingDiffuseFallback(scene, 0.5f);
  SyncTemporaryEnvironmentalLightingSettingsFromScene(scene);
  if (!ApplicationContext::Get().Loop()) {
    throw std::runtime_error("Application ended while preparing reflection probe bake controls.");
  }
  const uint64_t first_persisted_hash_before =
      YAML::LoadFile(first_bake->GetAbsolutePath().string())["payload_hash"].as<uint64_t>();
  RunEnvironmentalLightingLocalProbeBake(scene, *render_layer, get_bake_probe(0), "Reflection probe validation");
  const bool first_bake_deferred =
      !first_bake->Saved() && first_bake->GetPayloadHash() == 0u && first_bake->GetCanonicalPayloadByteSize() == 0u &&
      first_bake->IsRuntimeReady() &&
      YAML::LoadFile(first_bake->GetAbsolutePath().string())["payload_hash"].as<uint64_t>() ==
          first_persisted_hash_before;
  if (!first_bake->Save()) {
    throw std::runtime_error("Reflection probe validation could not explicitly save its first baked payload.");
  }
  const uint64_t first_payload_hash = first_bake->GetPayloadHash();
  const auto first_bake_document = YAML::LoadFile(first_bake->GetAbsolutePath().string());
  const bool persisted_payload_is_canonical =
      first_bake_document["pixels"] &&
      first_bake_document["pixels"].as<YAML::Binary>().size() == GlobalReflectionProbe::kCanonicalPayloadByteSize;
  red_probe.payload = blue_asset;
  const uint64_t second_persisted_hash_before =
      YAML::LoadFile(second_bake->GetAbsolutePath().string())["payload_hash"].as<uint64_t>();
  RunEnvironmentalLightingLocalProbeBake(scene, *render_layer, get_bake_probe(1), "Reflection probe validation");
  const bool second_bake_deferred =
      !second_bake->Saved() && second_bake->GetPayloadHash() == 0u &&
      second_bake->GetCanonicalPayloadByteSize() == 0u && second_bake->IsRuntimeReady() &&
      YAML::LoadFile(second_bake->GetAbsolutePath().string())["payload_hash"].as<uint64_t>() ==
          second_persisted_hash_before;
  if (!second_bake->Save()) {
    throw std::runtime_error("Reflection probe validation could not explicitly save its second baked payload.");
  }
  red_probe.payload = red_asset;
  const bool deferred_bake_persistence = first_bake_deferred && second_bake_deferred;
  const bool bake_contract =
      first_bake->GetSourceKind() == GlobalReflectionProbe::SourceKind::Baked &&
      second_bake->GetSourceKind() == GlobalReflectionProbe::SourceKind::Baked &&
      first_payload_hash == second_bake->GetPayloadHash() &&
      first_bake->GetCanonicalPayloadByteSize() == GlobalReflectionProbe::kCanonicalPayloadByteSize &&
      second_bake->GetCanonicalPayloadByteSize() == GlobalReflectionProbe::kCanonicalPayloadByteSize &&
      first_bake->GetRuntimeFormat() == GlobalReflectionProbe::kCanonicalFormat && first_bake->GetCubemap() &&
      first_bake->GetCubemap()->GetResolution() == GlobalReflectionProbe::kResolution &&
      first_bake->GetCubemap()->GetMipLevels() == GlobalReflectionProbe::kMipLevels && persisted_payload_is_canonical;
  bool baked_nonblack = false;
  const auto& baked_payload = first_bake->GetCanonicalPayload();
  for (size_t index = 0; index < baked_payload.size(); index += 4) {
    baked_nonblack |= glm::unpackHalf1x16(baked_payload[index]) > 0.0f ||
                      glm::unpackHalf1x16(baked_payload[index + 1]) > 0.0f ||
                      glm::unpackHalf1x16(baked_payload[index + 2]) > 0.0f;
  }

  const auto refresh_bake_statuses = [&] {
    SyncTemporaryEnvironmentalLightingSettingsFromScene(scene);
  };
  const auto all_bake_probes = [&] {
    return std::array<EnvironmentalLighting::LocalReflectionProbe*, 2>{&get_bake_probe(0), &get_bake_probe(1)};
  };
  const auto all_bake_probe_statuses_are = [&](const std::string& expected_status) {
    const auto probes = all_bake_probes();
    return std::all_of(probes.begin(), probes.end(), [&](const auto* probe) {
      return GetEnvironmentalLightingLocalProbeBakeStatus(*probe) == expected_status;
    });
  };
  refresh_bake_statuses();
  const bool initial_bakes_ready = all_bake_probe_statuses_are("Ready");
  const std::string imported_status_before = GetEnvironmentalLightingLocalProbeBakeStatus(red_probe);
  const uint64_t imported_payload_hash_before = red_asset->GetPayloadHash();
  const glm::vec3 original_background_color = lighting->indirect_environment_source.color;
  SetEnvironmentalLightingIntensity(scene, 1.0f);
  SetEnvironmentalLightingDiffuseFallback(scene, 0.0f);
  lighting->indirect_environment_source.color = original_background_color + glm::vec3(0.03125f, 0.0f, 0.0f);
  refresh_bake_statuses();
  const std::array no_auto_payload_hashes_before = {first_bake->GetPayloadHash(), second_bake->GetPayloadHash()};
  bool explicit_payload_active_without_bake = true;
  for (size_t frame = 0; frame < 4u; ++frame) {
    if (!ApplicationContext::Get().Loop()) {
      throw std::runtime_error("Application ended during reflection probe no-auto-rebake observation.");
    }
    refresh_bake_statuses();
    explicit_payload_active_without_bake &= first_bake->IsRuntimeReady() && second_bake->IsRuntimeReady() &&
                                            get_bake_probe(0).payload == first_bake &&
                                            get_bake_probe(1).payload == second_bake;
  }
  const std::array no_auto_payload_hashes_after = {first_bake->GetPayloadHash(), second_bake->GetPayloadHash()};
  const bool no_auto_payload_hashes_unchanged = no_auto_payload_hashes_before == no_auto_payload_hashes_after;
  const uint64_t imported_payload_hash_after_observation = red_asset->GetPayloadHash();
  const bool imported_probe_unchanged = imported_status_before == "Imported" &&
                                        GetEnvironmentalLightingLocalProbeBakeStatus(red_probe) == "Imported" &&
                                        red_asset->GetSourceKind() == GlobalReflectionProbe::SourceKind::Imported &&
                                        imported_payload_hash_before == imported_payload_hash_after_observation;

  const auto capture_explicit_payload = [&](const char* name) {
    scene_camera->ResetFrameCount();
    for (size_t frame = 0; frame < 3u; ++frame) {
      if (!ApplicationContext::Get().Loop()) {
        throw std::runtime_error(std::string("Application ended during explicit reflection payload capture: ") + name);
      }
    }
    return ReadAndStoreValidationCapture(scene_camera->GetRenderTexture(), resolution, output_directory / name,
                                         "Explicit reflection payload");
  };
  get_bake_probe(0).artist_priority = 100;
  get_bake_probe(0).sphere_radius = 12.0f;
  get_bake_probe(0).blend_distance = 0.0f;
  get_bake_probe(0).reflection_intensity = 8.0f;
  ClampEnvironmentalLightingLocalReflectionProbe(get_bake_probe(0));
  const auto explicit_payload_enabled = capture_explicit_payload("explicit-payload-enabled.png");
  get_bake_probe(0).enabled = false;
  const auto explicit_payload_disabled = capture_explicit_payload("explicit-payload-disabled.png");
  get_bake_probe(0).enabled = true;
  get_bake_probe(0).artist_priority = -100;
  get_bake_probe(0).sphere_radius = 0.1f;
  get_bake_probe(0).reflection_intensity = 1.0f;
  ClampEnvironmentalLightingLocalReflectionProbe(get_bake_probe(0));
  const auto normalized_rms = [](const std::vector<glm::vec4>& lhs, const std::vector<glm::vec4>& rhs) {
    if (lhs.size() != rhs.size() || lhs.empty()) {
      return std::numeric_limits<double>::infinity();
    }
    double squared_error = 0.0;
    double squared_signal = 0.0;
    for (size_t index = 0; index < lhs.size(); ++index) {
      const glm::dvec3 a(lhs[index]);
      const glm::dvec3 b(rhs[index]);
      const auto difference = a - b;
      squared_error += glm::dot(difference, difference);
      squared_signal += glm::dot(a, a);
    }
    return squared_signal > 0.0 ? std::sqrt(squared_error / squared_signal) : std::sqrt(squared_error);
  };
  const double explicit_payload_render_delta = normalized_rms(explicit_payload_enabled, explicit_payload_disabled);
  const bool explicit_payload_rendered = explicit_payload_render_delta > 0.0001;

  const uint64_t single_payload_hash_before = first_bake->GetPayloadHash();
  const uint64_t single_persisted_hash_before =
      YAML::LoadFile(first_bake->GetAbsolutePath().string())["payload_hash"].as<uint64_t>();
  RunEnvironmentalLightingLocalProbeBake(scene, *render_layer, get_bake_probe(0), "Reflection probe validation");
  const bool single_rebake_deferred =
      !first_bake->Saved() && first_bake->GetPayloadHash() == 0u && first_bake->GetCanonicalPayloadByteSize() == 0u &&
      first_bake->IsRuntimeReady() &&
      YAML::LoadFile(first_bake->GetAbsolutePath().string())["payload_hash"].as<uint64_t>() ==
          single_persisted_hash_before;
  if (!first_bake->Save()) {
    throw std::runtime_error("Reflection probe validation could not explicitly save its single rebake.");
  }
  const uint64_t single_payload_hash_after = first_bake->GetPayloadHash();
  const bool single_rebake_ready = GetEnvironmentalLightingLocalProbeBakeStatus(get_bake_probe(0)) == "Ready" &&
                                   first_bake->IsRuntimeReady() && single_payload_hash_after != 0u &&
                                   single_payload_hash_after != single_payload_hash_before;

  lighting->indirect_environment_source.color = original_background_color + glm::vec3(0.015625f, 0.0f, 0.0f);
  refresh_bake_statuses();

  std::vector<uint32_t> batch_pending_counts;
  std::array<bool, 2> batch_observed_pending{false, false};
  uint32_t batch_max_pending = 0;
  uint32_t batch_queued_count = 0;
  bool batch_bakes_deferred = true;
  Platform::ResetGpuTimestampStats();
  const auto batch_start = std::chrono::steady_clock::now();
  std::vector<const EnvironmentalLighting::LocalReflectionProbe*> batch_probes;
  batch_probes.reserve(bake_probe_indices.size());
  for (size_t index = 0; index < bake_probe_indices.size(); ++index) {
    batch_probes.emplace_back(&get_bake_probe(index));
    batch_observed_pending[index] = true;
  }
  batch_pending_counts.emplace_back(static_cast<uint32_t>(batch_probes.size()));
  batch_max_pending = static_cast<uint32_t>(batch_probes.size());
  batch_queued_count = RunEnvironmentalLightingLocalProbeBakeBatch(scene, *render_layer, batch_probes,
                                                                   "Reflection probe validation batch");
  const double batch_wall_milliseconds =
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - batch_start).count();
  batch_pending_counts.emplace_back(0u);
  for (size_t index = 0; index < bake_probe_indices.size(); ++index) {
    batch_bakes_deferred &= !bake_assets[index]->Saved() && bake_assets[index]->GetPayloadHash() == 0u &&
                            bake_assets[index]->GetCanonicalPayloadByteSize() == 0u &&
                            bake_assets[index]->IsRuntimeReady();
    if (!bake_assets[index]->Save()) {
      throw std::runtime_error("Reflection probe validation could not explicitly save its batch rebake.");
    }
    refresh_bake_statuses();
  }
  for (int frame = 0; frame < Platform::kMaxFramesInFlight; ++frame) {
    if (!ApplicationContext::Get().Loop()) {
      throw std::runtime_error("Application ended while resolving reflection probe batch timing scopes.");
    }
  }
  const auto batch_gpu_timings = Platform::GetGpuTimestampStats();
  const auto batch_cpu_timings = Platform::GetCpuTimingStats();
  const auto find_timing = [](const std::vector<GpuTimestampStats>& timings, const std::string& name) {
    const auto found = std::find_if(timings.begin(), timings.end(), [&](const auto& timing) {
      return timing.name == name;
    });
    return found == timings.end() ? GpuTimestampStats{} : *found;
  };
  const auto batch_gpu_total = find_timing(batch_gpu_timings, "Reflection Probe Bake GPU Total");
  const auto batch_face_capture = find_timing(batch_gpu_timings, "Reflection Probe Face Capture");
  const auto batch_prefilter = find_timing(batch_gpu_timings, "Reflection Probe GGX Prefilter");
  const auto batch_cpu_prepare = find_timing(batch_cpu_timings, "Reflection Probe Bake Prepare CPU");
  const auto batch_cpu_record = find_timing(batch_cpu_timings, "Reflection Probe Bake Record CPU");
  const double batch_cpu_total_milliseconds = batch_cpu_prepare.last_milliseconds + batch_cpu_record.last_milliseconds;
  const bool batch_finished = all_bake_probe_statuses_are("Ready");
  const uint64_t imported_payload_hash_after_batch = red_asset->GetPayloadHash();
  const bool imported_probe_unchanged_after_batch =
      imported_probe_unchanged && GetEnvironmentalLightingLocalProbeBakeStatus(red_probe) == "Imported" &&
      imported_payload_hash_after_batch == imported_payload_hash_before;
  const std::array batch_payload_hashes = {first_bake->GetPayloadHash(), second_bake->GetPayloadHash()};
  const bool batch_payloads_nonblack = std::all_of(bake_assets.begin(), bake_assets.end(), [](const auto& asset) {
    const auto& payload = asset->GetCanonicalPayload();
    for (size_t index = 0; index < payload.size(); index += 4u) {
      if (glm::unpackHalf1x16(payload[index]) > 0.0f || glm::unpackHalf1x16(payload[index + 1u]) > 0.0f ||
          glm::unpackHalf1x16(payload[index + 2u]) > 0.0f) {
        return true;
      }
    }
    return false;
  });
  const bool batch_final_ready =
      batch_finished && batch_payload_hashes[0] != 0u && batch_payload_hashes[0] == batch_payload_hashes[1] &&
      first_bake->IsRuntimeReady() && second_bake->IsRuntimeReady() &&
      first_bake->GetCanonicalPayloadByteSize() == GlobalReflectionProbe::kCanonicalPayloadByteSize &&
      second_bake->GetCanonicalPayloadByteSize() == GlobalReflectionProbe::kCanonicalPayloadByteSize &&
      get_bake_probe(0).payload == first_bake && get_bake_probe(1).payload == second_bake && batch_payloads_nonblack;
  reflection_pack->probes.resize(bake_probe_base_index);
  SetEnvironmentalLightingFallbackIntensities(*lighting, 0.0f, 1.0f);

  struct RegionEvidence {
    glm::dvec3 average = glm::dvec3(0.0);
    double luminance = 0.0;
    uint64_t count = 0;
  };
  const std::array regions = {std::pair{"left", std::array{glm::vec2(0.29f, 0.38f), glm::vec2(0.41f, 0.60f)}},
                              std::pair{"center", std::array{glm::vec2(0.46f, 0.42f), glm::vec2(0.53f, 0.58f)}},
                              std::pair{"boundary", std::array{glm::vec2(0.53f, 0.43f), glm::vec2(0.58f, 0.57f)}},
                              std::pair{"right", std::array{glm::vec2(0.59f, 0.38f), glm::vec2(0.71f, 0.60f)}},
                              std::pair{"smooth", std::array{glm::vec2(0.40f, 0.24f), glm::vec2(0.48f, 0.39f)}},
                              std::pair{"rough", std::array{glm::vec2(0.52f, 0.24f), glm::vec2(0.60f, 0.39f)}},
                              std::pair{"fallback", std::array{glm::vec2(0.46f, 0.61f), glm::vec2(0.54f, 0.76f)}}};
  const auto summarize = [&](const std::vector<glm::vec4>& pixels, const std::array<glm::vec2, 2>& region) {
    RegionEvidence evidence;
    const auto begin = glm::uvec2(glm::floor(region[0] * glm::vec2(resolution)));
    const auto end = glm::uvec2(glm::ceil(region[1] * glm::vec2(resolution)));
    for (uint32_t y = begin.y; y < end.y; ++y) {
      const auto source_y = resolution.y - 1u - y;
      for (uint32_t x = begin.x; x < end.x; ++x) {
        const auto color =
            glm::max(glm::dvec3(pixels[static_cast<size_t>(source_y) * resolution.x + x]), glm::dvec3(0.0));
        evidence.average += color;
        evidence.luminance += glm::dot(color, glm::dvec3(0.2126, 0.7152, 0.0722));
        ++evidence.count;
      }
    }
    if (evidence.count != 0u) {
      evidence.average /= static_cast<double>(evidence.count);
      evidence.luminance /= static_cast<double>(evidence.count);
    }
    return evidence;
  };
  struct CaptureEvidence {
    std::string name;
    std::string ambient_occlusion;
    std::vector<glm::vec4> pixels;
    std::map<std::string, RegionEvidence> regions;
    std::vector<uint64_t> ordered_ids;
    std::vector<uint32_t> validity;
    std::vector<RenderInstanceStorage::ReflectionProbeInfoBlock> probe_infos;
    uint32_t probe_count = 0;
    int indirect_debug_view = 0;
    float material_occlusion = 1.0f;
    float indirect_lighting_intensity = 0.0f;
    bool finite = true;
  };
  const auto capture = [&](const char* name) {
    SyncTemporaryEnvironmentalLightingSettingsFromScene(scene);
    scene_camera->ResetFrameCount();
    for (size_t frame = 0; frame < 3u; ++frame) {
      if (!ApplicationContext::Get().Loop()) {
        throw std::runtime_error(std::string("Application ended during reflection probe phase: ") + name);
      }
    }
    CaptureEvidence evidence;
    evidence.name = name;
    evidence.indirect_debug_view = static_cast<int>(render_layer->render_settings.indirect_lighting_debug_view);
    evidence.material_occlusion = rough_metal_material->material_data.shade_material.occlusion_strength;
    evidence.indirect_lighting_intensity = lighting->diffuse_fallback_intensity;
    if (!post_processing_stack->enable_ambient_occlusion) {
      evidence.ambient_occlusion = "disabled";
    } else if (!post_processing_stack->ambient_occlusion) {
      evidence.ambient_occlusion = "unavailable";
    } else {
      evidence.ambient_occlusion =
          post_processing_stack->ambient_occlusion->algorithm == AmbientOcclusion::Algorithm::Gtao ? "gtao" : "ssao";
    }
    const auto capture_path = output_directory / (evidence.name + ".png");
    evidence.pixels =
        ReadAndStoreValidationCapture(scene_camera->GetRenderTexture(), resolution, capture_path, "Reflection probe");
    for (const auto& pixel : evidence.pixels) {
      evidence.finite &=
          std::isfinite(pixel.x) && std::isfinite(pixel.y) && std::isfinite(pixel.z) && std::isfinite(pixel.w);
    }
    for (const auto& region : regions) {
      evidence.regions.emplace(region.first, summarize(evidence.pixels, region.second));
    }
    const auto storage = render_layer->GetCurrentRenderInstanceStorage();
    if (!storage) {
      throw std::runtime_error("Reflection probe capture has no render instance storage.");
    }
    evidence.probe_count = storage->GetReflectionProbeCount();
    const auto& probe_infos = storage->GetReflectionProbeInfoBlocks();
    for (uint32_t index = 0; index < evidence.probe_count; ++index) {
      evidence.ordered_ids.emplace_back(static_cast<uint64_t>(probe_infos[index].identity_and_flags.z) |
                                        static_cast<uint64_t>(probe_infos[index].identity_and_flags.w) << 32u);
      evidence.validity.emplace_back(probe_infos[index].identity_and_flags.y);
      evidence.probe_infos.emplace_back(probe_infos[index]);
    }
    return evidence;
  };
  std::array<CaptureEvidence, 26> captures;
  captures[0] = capture("baseline");
  SetEnvironmentalLightingIntensity(scene, 0.0f);
  captures[1] = capture("sky-off-local");
  SetEnvironmentalLightingIntensity(scene, 1.0f);
  constexpr size_t timing_warmup_frames = 8;
  constexpr size_t timing_measure_frames = 120;
  for (size_t frame = 0; frame < timing_warmup_frames; ++frame) {
    if (!ApplicationContext::Get().Loop()) {
      throw std::runtime_error("Application ended during reflection probe timing warmup.");
    }
  }
  Platform::WaitForFrameSubmissions("Reflection Probe Timing Warmup Fence Wait");
  Platform::ResetGpuTimestampStats();
  for (size_t frame = 0; frame < timing_measure_frames; ++frame) {
    if (!ApplicationContext::Get().Loop()) {
      throw std::runtime_error("Application ended during reflection probe timing measurement.");
    }
  }
  Platform::WaitForFrameSubmissions("Reflection Probe Timing Completion Fence Wait");
  const auto timing_stats = Platform::GetGpuTimestampStats();
  const auto deferred_lighting_timing = std::find_if(timing_stats.begin(), timing_stats.end(), [](const auto& timing) {
    return timing.name == "Deferred Lighting";
  });
  if (deferred_lighting_timing == timing_stats.end()) {
    throw std::runtime_error("Reflection probe validation did not record Deferred Lighting GPU timing.");
  }
  const uint64_t deferred_lighting_sample_count = deferred_lighting_timing->sample_count;
  const double deferred_lighting_median_ms = deferred_lighting_timing->MedianMilliseconds();
  const double deferred_lighting_p95_ms = deferred_lighting_timing->PercentileMilliseconds(0.95);
  const std::vector<double> deferred_lighting_samples_ms = deferred_lighting_timing->samples_milliseconds;

  const auto saved_ambient_occlusion = post_processing_stack->ambient_occlusion;
  const float smooth_occlusion = smooth_metal_material->material_data.shade_material.occlusion_strength;
  const float rough_occlusion = rough_metal_material->material_data.shade_material.occlusion_strength;
  const auto set_material_occlusion = [&](const float value) {
    smooth_metal_material->material_data.shade_material.occlusion_strength = value;
    rough_metal_material->material_data.shade_material.occlusion_strength = value;
    smooth_metal_material->MarkDirty();
    rough_metal_material->MarkDirty();
  };
  const auto set_ambient_occlusion = [&](const bool enabled, const AmbientOcclusion::Algorithm algorithm) {
    post_processing_stack->ambient_occlusion = saved_ambient_occlusion;
    post_processing_stack->enable_ambient_occlusion = enabled;
    saved_ambient_occlusion->algorithm = algorithm;
  };
  const auto set_indirect_debug = [&](const RenderSettings::IndirectLightingDebugView view) {
    render_layer->render_settings.indirect_lighting_debug_view = view;
  };

  set_material_occlusion(1.0f);
  set_ambient_occlusion(false, AmbientOcclusion::Algorithm::Gtao);
  set_indirect_debug(RenderSettings::IndirectLightingDebugView::UnoccludedProbeSpecular);
  captures[7] = capture("m15-unoccluded");
  set_indirect_debug(RenderSettings::IndirectLightingDebugView::SpecularVisibility);
  captures[8] = capture("m15-visibility-off");
  set_indirect_debug(RenderSettings::IndirectLightingDebugView::OccludedProbeSpecular);
  captures[9] = capture("m15-occluded-off");

  set_material_occlusion(0.2f);
  set_indirect_debug(RenderSettings::IndirectLightingDebugView::SpecularVisibility);
  captures[10] = capture("m15-visibility-material");
  set_indirect_debug(RenderSettings::IndirectLightingDebugView::OccludedProbeSpecular);
  captures[11] = capture("m15-occluded-material");

  set_material_occlusion(1.0f);
  set_ambient_occlusion(true, AmbientOcclusion::Algorithm::Gtao);
  saved_ambient_occlusion->gtao_radius = 0.8f;
  saved_ambient_occlusion->gtao_intensity = 1.5f;
  set_indirect_debug(RenderSettings::IndirectLightingDebugView::SpecularVisibility);
  captures[12] = capture("m15-visibility-gtao");
  set_indirect_debug(RenderSettings::IndirectLightingDebugView::OccludedProbeSpecular);
  captures[13] = capture("m15-occluded-gtao");

  set_material_occlusion(0.2f);
  set_indirect_debug(RenderSettings::IndirectLightingDebugView::SpecularVisibility);
  captures[14] = capture("m15-visibility-material-gtao");
  set_indirect_debug(RenderSettings::IndirectLightingDebugView::OccludedProbeSpecular);
  captures[15] = capture("m15-occluded-material-gtao");

  set_material_occlusion(1.0f);
  post_processing_stack->enable_ambient_occlusion = true;
  post_processing_stack->ambient_occlusion.reset();
  set_indirect_debug(RenderSettings::IndirectLightingDebugView::SpecularVisibility);
  captures[16] = capture("m15-visibility-unavailable");
  set_indirect_debug(RenderSettings::IndirectLightingDebugView::OccludedProbeSpecular);
  captures[17] = capture("m15-occluded-unavailable");

  set_ambient_occlusion(true, AmbientOcclusion::Algorithm::Ssao);
  set_indirect_debug(RenderSettings::IndirectLightingDebugView::OccludedProbeSpecular);
  captures[18] = capture("m15-occluded-ssao");

  set_ambient_occlusion(false, AmbientOcclusion::Algorithm::Gtao);
  set_material_occlusion(0.2f);
  SetEnvironmentalLightingDiffuseFallback(scene, 2.0f);
  captures[19] = capture("m15-occluded-indirect-double");
  SetEnvironmentalLightingDiffuseFallback(scene, 0.0f);

  const auto boundary_transform = scene->GetDataComponent<Transform>(boundary_metal_entity);
  const float boundary_roughness = boundary_metal_material->material_data.shade_material.pbr_roughness_factor;
  const float boundary_occlusion = boundary_metal_material->material_data.shade_material.occlusion_strength;
  boundary_metal_material->material_data.shade_material.pbr_roughness_factor = 1.0f;
  boundary_metal_material->material_data.shade_material.occlusion_strength = 0.2f;
  boundary_metal_material->MarkDirty();
  for (size_t index = 0; index < 3u; ++index) {
    auto transform = boundary_transform;
    transform.SetPosition(glm::vec3(0.70f + 0.02f * static_cast<float>(index), 0.8f, -2.4f));
    scene->SetDataComponent(boundary_metal_entity, transform);
    captures[20u + index] = capture(index == 0u   ? "m15-boundary-left"
                                    : index == 1u ? "m15-boundary-center"
                                                  : "m15-boundary-right");
  }
  scene->SetDataComponent(boundary_metal_entity, boundary_transform);
  boundary_metal_material->material_data.shade_material.pbr_roughness_factor = boundary_roughness;
  boundary_metal_material->material_data.shade_material.occlusion_strength = boundary_occlusion;
  boundary_metal_material->MarkDirty();

  const glm::vec3 m15_moved_position(1.3f, 1.35f, 7.2f);
  const glm::vec3 m15_moved_target(0.0f, 0.85f, -2.4f);
  editor_layer->SetSceneCameraPosition(m15_moved_position);
  editor_layer->SetSceneCameraRotation(
      glm::quatLookAt(glm::normalize(m15_moved_target - m15_moved_position), glm::vec3(0.0f, 1.0f, 0.0f)));
  captures[23] = capture("m15-camera-moved-occluded");
  const glm::vec3 m15_original_position(0.0f, 1.0f, 7.0f);
  const glm::vec3 m15_original_target(0.0f, 0.85f, -2.4f);
  editor_layer->SetSceneCameraPosition(m15_original_position);
  editor_layer->SetSceneCameraRotation(
      glm::quatLookAt(glm::normalize(m15_original_target - m15_original_position), glm::vec3(0.0f, 1.0f, 0.0f)));

  set_material_occlusion(rough_occlusion);
  smooth_metal_material->material_data.shade_material.occlusion_strength = smooth_occlusion;
  smooth_metal_material->MarkDirty();
  set_ambient_occlusion(false, AmbientOcclusion::Algorithm::Gtao);
  set_indirect_debug(RenderSettings::IndirectLightingDebugView::Beauty);
  directional_probe.box_projection = false;
  captures[2] = capture("box-unprojected");
  directional_probe.box_projection = true;

  const auto red_settings = red_probe;
  const auto red_iter =
      std::find_if(reflection_pack->probes.begin(), reflection_pack->probes.end(), [](const auto& probe) {
        return probe.name == "Reflection Probe Red Box";
      });
  if (red_iter == reflection_pack->probes.end()) {
    throw std::runtime_error("Reflection probe validation red asset entry is missing.");
  }
  reflection_pack->probes.erase(red_iter);
  reflection_pack->probes.push_back(red_settings);
  captures[3] = capture("owner-order-reversed");
  auto& reordered_directional_probe =
      RequireEnvironmentalLightingLocalReflectionProbe(*lighting, "Reflection Probe Directional Box");
  auto& reordered_fallback_probe =
      RequireEnvironmentalLightingLocalReflectionProbe(*lighting, "Reflection Probe Fallback");

  const glm::vec3 moved_position(1.3f, 1.35f, 7.2f);
  const glm::vec3 moved_target(0.0f, 0.85f, -2.4f);
  editor_layer->SetSceneCameraPosition(moved_position);
  editor_layer->SetSceneCameraRotation(
      glm::quatLookAt(glm::normalize(moved_target - moved_position), glm::vec3(0.0f, 1.0f, 0.0f)));
  captures[4] = capture("camera-moved");
  const glm::vec3 original_position(0.0f, 1.0f, 7.0f);
  const glm::vec3 original_target(0.0f, 0.85f, -2.4f);
  editor_layer->SetSceneCameraPosition(original_position);
  editor_layer->SetSceneCameraRotation(
      glm::quatLookAt(glm::normalize(original_target - original_position), glm::vec3(0.0f, 1.0f, 0.0f)));

  const auto empty_asset = AssetManager::CreateTemporaryAsset<GlobalReflectionProbe>();
  reordered_fallback_probe.payload = empty_asset;
  captures[5] = capture("missing-asset");
  const auto fallback_iter =
      std::find_if(reflection_pack->probes.begin(), reflection_pack->probes.end(), [](const auto& probe) {
        return probe.name == "Reflection Probe Fallback";
      });
  if (fallback_iter == reflection_pack->probes.end()) {
    throw std::runtime_error("Reflection probe validation fallback asset entry is missing.");
  }
  reflection_pack->probes.erase(fallback_iter);
  captures[6] = capture("removed");

  auto& debug_box_probe =
      RequireEnvironmentalLightingLocalReflectionProbe(*lighting, "Reflection Probe Directional Box");
  auto& debug_sphere_probe =
      RequireEnvironmentalLightingLocalReflectionProbe(*lighting, "Reflection Probe Nested Sphere");
  const auto original_directional_transform = debug_box_probe.transform;
  debug_box_probe.transform = MakeAuthoringTransform(
      {-1.35f, 1.65f, -2.4f}, glm::radians(glm::vec3(14.0f, -24.0f, 9.0f)), {1.68f, 0.85f, 1.32f});
  debug_sphere_probe.transform =
      MakeAuthoringTransform({1.15f, 1.1f, -2.4f}, glm::radians(glm::vec3(12.0f, 28.0f, -9.0f)), {1.35f, 0.8f, 1.1f});
  editor_layer->enable_gizmos = true;
  editor_layer->OpenAssetInspector(lighting);
  captures[24] = capture("debug-bounds-off");
  debug_box_probe.debug_draw_bounds = true;
  debug_sphere_probe.debug_draw_bounds = true;
  captures[25] = capture("debug-bounds-on");

  const std::vector<uint64_t> expected_order = {StableEnvironmentalLightingId("Reflection Probe Fallback"),
                                                StableEnvironmentalLightingId("Reflection Probe Directional Box"),
                                                StableEnvironmentalLightingId("Reflection Probe Nested Sphere"),
                                                StableEnvironmentalLightingId("Reflection Probe Red Box"),
                                                StableEnvironmentalLightingId("Reflection Probe Blue Box")};
  const auto& baseline = captures[0];
  const auto& sky_off_local = captures[1];
  const auto& unprojected = captures[2];
  const auto& reordered = captures[3];
  const auto& camera_moved = captures[4];
  const auto& missing = captures[5];
  const auto& removed = captures[6];
  const auto& m15_unoccluded = captures[7];
  const auto& m15_visibility_off = captures[8];
  const auto& m15_occluded_off = captures[9];
  const auto& m15_visibility_material = captures[10];
  const auto& m15_occluded_material = captures[11];
  const auto& m15_visibility_gtao = captures[12];
  const auto& m15_occluded_gtao = captures[13];
  const auto& m15_visibility_combined = captures[14];
  const auto& m15_occluded_combined = captures[15];
  const auto& m15_visibility_unavailable = captures[16];
  const auto& m15_occluded_unavailable = captures[17];
  const auto& m15_occluded_ssao = captures[18];
  const auto& m15_occluded_indirect_double = captures[19];
  const auto& m15_boundary_left = captures[20];
  const auto& m15_boundary_center = captures[21];
  const auto& m15_boundary_right = captures[22];
  const auto& m15_camera_moved = captures[23];
  const auto& debug_bounds_off = captures[24];
  const auto& debug_bounds_on = captures[25];
  const auto dominant = [](const glm::dvec3& color, const int channel) {
    const double selected = color[channel];
    return selected > 0.005 && selected > color[(channel + 1) % 3] * 1.25 && selected > color[(channel + 2) % 3] * 1.25;
  };
  const auto& fallback_color = missing.regions.at("fallback").average;
  const bool neutral_fallback = missing.regions.at("fallback").luminance > 0.005 &&
                                std::max({fallback_color.x, fallback_color.y, fallback_color.z}) -
                                        std::min({fallback_color.x, fallback_color.y, fallback_color.z}) <
                                    std::max(0.03, missing.regions.at("fallback").luminance * 0.2);
  const double box_projection_delta = normalized_rms(baseline.pixels, unprojected.pixels);
  const double reorder_delta = normalized_rms(baseline.pixels, reordered.pixels);
  const double missing_removal_delta = normalized_rms(missing.pixels, removed.pixels);
  const double debug_bounds_delta = normalized_rms(debug_bounds_off.pixels, debug_bounds_on.pixels);
  const double m15_off_bypass_delta = normalized_rms(m15_unoccluded.pixels, m15_occluded_off.pixels);
  const double m15_material_delta = normalized_rms(m15_unoccluded.pixels, m15_occluded_material.pixels);
  const double m15_gtao_delta = normalized_rms(m15_unoccluded.pixels, m15_occluded_gtao.pixels);
  const double m15_combined_delta = normalized_rms(m15_unoccluded.pixels, m15_occluded_combined.pixels);
  const double m15_unavailable_delta = normalized_rms(m15_unoccluded.pixels, m15_occluded_unavailable.pixels);
  const double m15_unavailable_visibility_delta =
      normalized_rms(m15_visibility_off.pixels, m15_visibility_unavailable.pixels);
  const double m15_ssao_delta = normalized_rms(m15_unoccluded.pixels, m15_occluded_ssao.pixels);
  const double m15_indirect_delta = normalized_rms(m15_occluded_material.pixels, m15_occluded_indirect_double.pixels);
  const auto scalar_channel_error = [](const CaptureEvidence& value) {
    double maximum = 0.0;
    for (const auto& pixel : value.pixels) {
      maximum =
          std::max(maximum, static_cast<double>(glm::max(glm::abs(pixel.r - pixel.g), glm::abs(pixel.r - pixel.b))));
    }
    return maximum;
  };
  const double m15_scalar_error =
      std::max({scalar_channel_error(m15_visibility_off), scalar_channel_error(m15_visibility_material),
                scalar_channel_error(m15_visibility_gtao), scalar_channel_error(m15_visibility_combined),
                scalar_channel_error(m15_visibility_unavailable)});
  const auto maximum_amplification = [](const CaptureEvidence& unoccluded, const CaptureEvidence& occluded) {
    double maximum = 0.0;
    for (size_t index = 0; index < unoccluded.pixels.size(); ++index) {
      const auto difference = glm::vec3(occluded.pixels[index] - unoccluded.pixels[index]);
      maximum = std::max(maximum, static_cast<double>(std::max({difference.r, difference.g, difference.b})));
    }
    return maximum;
  };
  const double m15_maximum_amplification = std::max({maximum_amplification(m15_unoccluded, m15_occluded_material),
                                                     maximum_amplification(m15_unoccluded, m15_occluded_gtao),
                                                     maximum_amplification(m15_unoccluded, m15_occluded_combined)});
  const auto boundary_luminance = [](const CaptureEvidence& value) {
    return value.regions.at("boundary").luminance;
  };
  const double minimum_boundary_luminance =
      std::min({boundary_luminance(m15_boundary_left), boundary_luminance(m15_boundary_center),
                boundary_luminance(m15_boundary_right)});
  const double maximum_boundary_luminance =
      std::max({boundary_luminance(m15_boundary_left), boundary_luminance(m15_boundary_center),
                boundary_luminance(m15_boundary_right)});
  const auto local_region_channel_invariant = [&](const char* name, const int channel) {
    const double enabled = baseline.regions.at(name).average[channel];
    const double sky_off = sky_off_local.regions.at(name).average[channel];
    return std::abs(enabled - sky_off) <= std::max(0.01, std::max(enabled, sky_off) * 0.1);
  };
  const auto matrices_nearly_equal = [](const glm::mat4& lhs, const glm::mat4& rhs) {
    for (int column = 0; column < 4; ++column) {
      for (int row = 0; row < 4; ++row) {
        if (glm::abs(lhs[column][row] - rhs[column][row]) > 1.0e-5f) {
          return false;
        }
      }
    }
    return true;
  };
  const auto expected_directional_world_to_probe = glm::inverse(original_directional_transform);
  float synthetic_packed_nrmse = 0.0f;
  float synthetic_packed_peak = 0.0f;
  float baked_packed_nrmse = 0.0f;
  float baked_packed_peak = 0.0f;
  const auto directional_asset = reordered_directional_probe.payload;
  const bool packed_quality_valid =
      directional_asset &&
      GlobalReflectionProbe::EvaluatePackedRuntimeQuality(directional_asset->GetCanonicalPayload(),
                                                          synthetic_packed_nrmse, synthetic_packed_peak) &&
      GlobalReflectionProbe::EvaluatePackedRuntimeQuality(first_bake->GetCanonicalPayload(), baked_packed_nrmse,
                                                          baked_packed_peak);
  const float packed_nrmse = glm::max(synthetic_packed_nrmse, baked_packed_nrmse);
  const float packed_peak = glm::max(synthetic_packed_peak, baked_packed_peak);
  const double packed_memory_saving = 1.0 - static_cast<double>(GlobalReflectionProbe::kPackedRuntimeByteSize) /
                                                static_cast<double>(GlobalReflectionProbe::kCanonicalPayloadByteSize);
  const bool packed_adopted = false;
  const std::vector<std::pair<std::string, bool>> checks{
      {"dynamic_initial_b_sweep", dynamic_initial_b_sweep},
      {"dynamic_transition_progress", dynamic_transition_progress},
      {"dynamic_a_sweep", dynamic_a_sweep},
      {"dynamic_b_sweep", dynamic_b_sweep},
      {"dynamic_contribution_resume", dynamic_contribution_resume},
      {"dynamic_reset", dynamic_reset},
      {"dynamic_returned_to_static", dynamic_returned_to_static},
      {"captures_finite", std::all_of(captures.begin(), captures.end(),
                                      [](const auto& capture_evidence) {
                                        return capture_evidence.finite;
                                      })},
      {"fixed_raster_contract",
       scene_camera->camera_render_mode == Camera::CameraRenderMode::Rasterization &&
           lighting->environment_lighting_intensity == 1.0f && lighting->diffuse_fallback_intensity == 0.0f &&
           lighting->specular_fallback_intensity == 1.0f && !post_processing_stack->enable_ambient_occlusion &&
           render_layer->render_settings.indirect_lighting_debug_view ==
               RenderSettings::IndirectLightingDebugView::Beauty},
      {"canonical_nonrecursive_bake", bake_contract && baked_nonblack},
      {"explicit_bake_defers_persistence", deferred_bake_persistence && single_rebake_deferred && batch_bakes_deferred},
      {"bake_targets_are_unique", unique_bake_entries && unique_bake_assets && initial_bakes_ready},
      {"explicit_bakes_do_not_run_automatically",
       explicit_payload_active_without_bake && no_auto_payload_hashes_unchanged},
      {"explicit_bakes_keep_last_valid_payload", explicit_payload_active_without_bake && explicit_payload_rendered},
      {"single_explicit_rebake_finishes_ready", single_rebake_ready},
      {"imported_probe_is_not_invalidated", imported_probe_unchanged_after_batch},
      {"batch_explicit_rebake_queues_both", batch_queued_count == 2u},
      {"batch_explicit_rebake_is_one_request", batch_max_pending == 2u && batch_observed_pending[0] &&
                                                   batch_observed_pending[1] &&
                                                   std::all_of(batch_pending_counts.begin(), batch_pending_counts.end(),
                                                               [](const uint32_t count) {
                                                                 return count <= 2u;
                                                               })},
      {"batch_explicit_rebake_finishes_ready", batch_final_ready},
      {"deferred_lighting_timing_is_complete",
       deferred_lighting_sample_count == timing_measure_frames &&
           deferred_lighting_samples_ms.size() == timing_measure_frames && std::isfinite(deferred_lighting_median_ms) &&
           deferred_lighting_median_ms >= 0.0 && std::isfinite(deferred_lighting_p95_ms) &&
           deferred_lighting_p95_ms >= deferred_lighting_median_ms},
      {"deterministic_runtime_order", baseline.probe_count == 5u && baseline.ordered_ids == expected_order &&
                                          std::all_of(baseline.validity.begin(), baseline.validity.end(),
                                                      [](const uint32_t valid) {
                                                        return valid == 1u;
                                                      })},
      {"scaled_oriented_probe_transform",
       baseline.probe_infos.size() == 5u &&
           matrices_nearly_equal(baseline.probe_infos[1].world_to_probe, expected_directional_world_to_probe) &&
           glm::all(glm::lessThanEqual(glm::abs(glm::vec3(baseline.probe_infos[1].shape_parameters) - glm::vec3(0.5f)),
                                       glm::vec3(1.0e-5f))) &&
           baseline.probe_infos[1].lighting_parameters.z ==
               static_cast<float>(EnvironmentalLighting::LocalReflectionProbeShape::Box)},
      {"adjacent_red_region", dominant(baseline.regions.at("left").average, 0)},
      {"nested_sphere_priority", dominant(baseline.regions.at("center").average, 1)},
      {"adjacent_blue_region", dominant(baseline.regions.at("right").average, 2)},
      {"lower_priority_boundary_blend",
       baseline.regions.at("boundary").average.x > 0.005 && baseline.regions.at("boundary").average.z > 0.005},
      {"exact_metal_local_specular", baseline.regions.at("left").luminance > 0.01 &&
                                         baseline.regions.at("center").luminance > 0.01 &&
                                         baseline.regions.at("right").luminance > 0.01},
      {"local_probe_payloads_are_sky_scale_invariant", local_region_channel_invariant("left", 0) &&
                                                           local_region_channel_invariant("center", 1) &&
                                                           local_region_channel_invariant("right", 2)},
      {"roughness_selects_prefiltered_mip",
       baseline.regions.at("smooth").luminance > baseline.regions.at("rough").luminance * 1.2},
      {"box_projection_changes_only_spatial_lookup", box_projection_delta > 0.0001},
      {"owner_order_is_deterministic", reordered.probe_count == baseline.probe_count &&
                                           reordered.ordered_ids == baseline.ordered_ids && reorder_delta < 1.0e-6},
      {"camera_outside_probes_keeps_surface_selection", camera_moved.probe_count == baseline.probe_count &&
                                                            camera_moved.ordered_ids == baseline.ordered_ids &&
                                                            dominant(camera_moved.regions.at("center").average, 1)},
      {"missing_asset_uses_global",
       missing.probe_count == 5u && !missing.validity.empty() && missing.validity.front() == 0u && neutral_fallback},
      {"removal_uses_same_global_fallback", removed.probe_count == 4u && missing_removal_delta < 1.0e-6},
      {"rough_specular_visibility_is_scalar", m15_scalar_error < 1.0e-6},
      {"rough_specular_disabled_bypasses_visibility", m15_off_bypass_delta < 1.0e-6},
      {"rough_specular_unavailable_falls_back",
       m15_unavailable_visibility_delta < 1.0e-6 && m15_unavailable_delta < 1.0e-6 &&
           m15_occluded_unavailable.regions.at("rough").luminance > 0.0001},
      {"ssao_remains_diffuse_only", m15_ssao_delta < 1.0e-6},
      {"material_ao_suppresses_rough_specular",
       m15_material_delta > 0.0001 &&
           m15_occluded_material.regions.at("rough").luminance < m15_unoccluded.regions.at("rough").luminance * 0.95 &&
           std::abs(m15_occluded_material.regions.at("smooth").luminance -
                    m15_unoccluded.regions.at("smooth").luminance) < 0.001},
      {"gtao_contributes_scalar_visibility", m15_gtao_delta > 0.000001},
      {"combined_visibility_is_bounded", m15_combined_delta + 1.0e-6 >= std::max(m15_material_delta, m15_gtao_delta) &&
                                             m15_maximum_amplification < 1.0e-5},
      {"probe_specular_is_indirect_intensity_invariant", m15_indirect_delta < 1.0e-6},
      {"rough_boundary_transition_is_finite_nonblack",
       minimum_boundary_luminance > 0.0001 && maximum_boundary_luminance < minimum_boundary_luminance * 2.0 + 0.01},
      {"camera_motion_keeps_occluded_probe_specular", m15_camera_moved.regions.at("center").luminance > 0.0001},
      {"transformed_debug_bounds_are_visible", debug_bounds_delta > 0.0001},
      {"packed_format_no_adopt",
       packed_quality_valid && packed_memory_saving >= GlobalReflectionProbe::kPackedMinMemorySaving &&
           !packed_adopted && directional_asset->GetRuntimeFormat() == GlobalReflectionProbe::kCanonicalFormat}};

  const auto report_path = output_directory / "report.json";
  std::ofstream report(report_path, std::ios::trunc);
  if (!report) {
    throw std::runtime_error("Failed to open reflection probe report: " + report_path.string());
  }
  report << std::setprecision(17);
  report << "{\n  \"schema_version\": 3,\n  \"contract\": {\"resolution\": [1920, 1080], "
            "\"render_mode\": \"Rasterization\", \"graphics_validation\": true, \"launch_count\": 1, "
            "\"probe_resolution\": 256, \"probe_mips\": 9, \"probe_format\": \"RGBA16F\", "
            "\"canonical_texels\": 524286, \"canonical_payload_bytes\": 4194288, "
            "\"packed_payload_bytes\": 2097144, \"max_enabled_probes\": 32, "
            "\"bake_near\": 0.1, \"bake_far\": 1000.0, \"screen_space_reflections\": false, "
            "\"ray_traced_reflections\": false, \"gpu_timestamps\": true, \"no_auto_bake_observation_frames\": 4, "
            "\"timing_warmup_frames\": 8, \"timing_measure_frames\": 120, "
            "\"batch_explicit_rebake_policy\": \"serial\", \"specular_visibility_model\": "
            "\"1-roughness^2*mix(0.04*tanh((1-min(material_ao,gtao))/0.04),1-min(material_ao,gtao),"
            "smoothstep(0.8,1,NdotV))\"},\n  \"captures\": [";
  for (size_t capture_index = 0; capture_index < captures.size(); ++capture_index) {
    const auto& capture_evidence = captures[capture_index];
    report << (capture_index == 0u ? "" : ",") << "\n    {\"name\": \"" << capture_evidence.name << "\", \"image\": \""
           << capture_evidence.name << ".png\", \"probe_count\": " << capture_evidence.probe_count
           << ", \"finite\": " << (capture_evidence.finite ? "true" : "false")
           << ", \"indirect_debug_view\": " << capture_evidence.indirect_debug_view << ", \"ambient_occlusion\": \""
           << capture_evidence.ambient_occlusion
           << "\", \"material_occlusion\": " << capture_evidence.material_occlusion
           << ", \"indirect_lighting_intensity\": " << capture_evidence.indirect_lighting_intensity
           << ", \"regions\": {";
    size_t region_index = 0;
    for (const auto& [name, region] : capture_evidence.regions) {
      report << (region_index++ == 0u ? "" : ",") << "\"" << name << "\": {\"average\": [" << region.average.x << ", "
             << region.average.y << ", " << region.average.z << "], \"luminance\": " << region.luminance << "}";
    }
    report << "}}";
  }
  report << "\n  ],\n  \"bake\": {\"payload_hash\": " << first_payload_hash
         << ", \"nonrecursive_equal\": " << (bake_contract ? "true" : "false")
         << ", \"nonblack\": " << (baked_nonblack ? "true" : "false")
         << ", \"persisted_payload_bytes\": " << GlobalReflectionProbe::kCanonicalPayloadByteSize
         << "},\n  \"dynamic_update\": {\"requested\": " << (dynamic_validation_requested ? "true" : "false")
         << ", \"policy\": \"Continuous A/B\", \"faces_per_frame\": 1"
         << ", \"initial_sweep_latency_frames\": " << dynamic_initial_latency_frames
         << ", \"complete_bab_cycle_frames\": " << dynamic_bab_cycle_frames
         << ", \"complete_bab_cycle_wall_ms\": " << dynamic_bab_cycle_milliseconds
         << ", \"two_sample_blend_frame_ms_average\": " << dynamic_blend_frame_milliseconds
         << ", \"published_generations_observed\": " << dynamic_published_generations_observed
         << ", \"transient_gpu_bytes\": " << dynamic_transient_gpu_bytes
         << ", \"gpu_update_ms\": " << dynamic_update_gpu_ms << ", \"gpu_face_capture_ms\": " << dynamic_capture_gpu_ms
         << ", \"gpu_ggx_prefilter_ms\": " << dynamic_prefilter_gpu_ms
         << "},\n  \"explicit_rebake\": {\"asset_entry_count\": 2, \"unique_entries\": "
         << (unique_bake_entries ? "true" : "false")
         << ", \"unique_assets\": " << (unique_bake_assets ? "true" : "false")
         << ", \"initial_controls\": {\"sky\": 0.75, \"indirect\": 0.5}, "
            "\"final_controls\": {\"sky\": 1.0, \"indirect\": 0.0}, "
            "\"no_auto_rebake\": {\"frames\": 4, \"payload_active\": "
         << (explicit_payload_active_without_bake ? "true" : "false")
         << ", \"payload_hashes_unchanged\": " << (no_auto_payload_hashes_unchanged ? "true" : "false")
         << ", \"last_valid_payload_rendered\": " << (explicit_payload_rendered ? "true" : "false")
         << ", \"render_nrmse\": " << explicit_payload_render_delta
         << ", \"enabled_image\": \"explicit-payload-enabled.png\", \"disabled_image\": "
            "\"explicit-payload-disabled.png\""
         << ", \"payload_hashes_before\": [" << no_auto_payload_hashes_before[0] << ", "
         << no_auto_payload_hashes_before[1] << "], \"payload_hashes_after\": [" << no_auto_payload_hashes_after[0]
         << ", " << no_auto_payload_hashes_after[1]
         << "]}, \"single\": {\"ready\": " << (single_rebake_ready ? "true" : "false")
         << ", \"payload_hash_before\": " << single_payload_hash_before
         << ", \"payload_hash_after\": " << single_payload_hash_after << "}, \"imported_probe\": {\"status\": \""
         << imported_status_before << "\", \"payload_hash_before\": " << imported_payload_hash_before
         << ", \"payload_hash_after\": " << imported_payload_hash_after_batch
         << ", \"unchanged\": " << (imported_probe_unchanged_after_batch ? "true" : "false")
         << "}, \"batch\": {\"queued_count\": " << batch_queued_count << ", \"pending_counts\": [";
  for (size_t index = 0; index < batch_pending_counts.size(); ++index) {
    report << (index == 0u ? "" : ", ") << batch_pending_counts[index];
  }
  report << "], \"max_pending\": " << batch_max_pending << ", \"both_observed_pending\": "
         << (batch_observed_pending[0] && batch_observed_pending[1] ? "true" : "false")
         << ", \"final_ready\": " << (batch_final_ready ? "true" : "false") << ", \"payload_hashes\": ["
         << batch_payload_hashes[0] << ", " << batch_payload_hashes[1]
         << "], \"timing\": {\"wall_ms\": " << batch_wall_milliseconds
         << ", \"gpu_total_ms\": " << batch_gpu_total.last_milliseconds
         << ", \"gpu_total_samples\": " << batch_gpu_total.sample_count
         << ", \"face_capture_ms\": " << batch_face_capture.total_milliseconds
         << ", \"face_capture_samples\": " << batch_face_capture.sample_count
         << ", \"ggx_prefilter_ms\": " << batch_prefilter.total_milliseconds
         << ", \"ggx_prefilter_samples\": " << batch_prefilter.sample_count
         << ", \"cpu_total_ms\": " << batch_cpu_total_milliseconds
         << ", \"cpu_prepare_ms\": " << batch_cpu_prepare.last_milliseconds
         << ", \"cpu_record_ms\": " << batch_cpu_record.last_milliseconds
         << ", \"cpu_total_samples\": " << std::min(batch_cpu_prepare.sample_count, batch_cpu_record.sample_count)
         << "}}},\n  \"gpu_timing\": {\"scope\": \"Deferred Lighting\", "
            "\"warmup_frames\": 8, \"measure_frames\": 120, \"sample_count\": "
         << deferred_lighting_sample_count << ", \"median_ms\": " << deferred_lighting_median_ms
         << ", \"p95_ms\": " << deferred_lighting_p95_ms
         << ", \"m14_baseline_median_ms\": 0.087536, \"m14_baseline_p95_ms\": 0.0970064, "
            "\"median_limit_ms\": 0.137536, \"p95_limit_ms\": 0.1970064, \"relative_mad_limit\": 0.25, "
            "\"samples_ms\": [";
  for (size_t index = 0; index < deferred_lighting_samples_ms.size(); ++index) {
    report << (index == 0u ? "" : ", ") << deferred_lighting_samples_ms[index];
  }
  report << "]},\n  \"rough_specular\": {\"material_ao_nrmse\": " << m15_material_delta
         << ", \"gtao_nrmse\": " << m15_gtao_delta << ", \"combined_nrmse\": " << m15_combined_delta
         << ", \"disabled_bypass_nrmse\": " << m15_off_bypass_delta
         << ", \"unavailable_bypass_nrmse\": " << m15_unavailable_delta
         << ", \"unavailable_visibility_nrmse\": " << m15_unavailable_visibility_delta
         << ", \"ssao_specular_nrmse\": " << m15_ssao_delta << ", \"indirect_intensity_nrmse\": " << m15_indirect_delta
         << ", \"scalar_channel_error\": " << m15_scalar_error
         << ", \"maximum_amplification\": " << m15_maximum_amplification
         << ", \"boundary_luminance_min\": " << minimum_boundary_luminance
         << ", \"boundary_luminance_max\": " << maximum_boundary_luminance
         << "},\n  \"abi\": {"
            "\"new_descriptors\": 0, \"lighting_sampler_count\": 37, \"reflection_probe_info_bytes\": 128, "
            "\"render_info_bytes\": 6032, \"camera_info_bytes\": 704},\n  \"image_deltas\": {\"box_projection_nrmse\": "
         << box_projection_delta << ", \"owner_reorder_nrmse\": " << reorder_delta
         << ", \"missing_removal_nrmse\": " << missing_removal_delta
         << ", \"debug_bounds_nrmse\": " << debug_bounds_delta << "},\n  \"packed_runtime\": {\"capability\": "
         << (directional_asset->PackedRuntimeFormatSupported() ? "true" : "false")
         << ", \"normalized_rms_error\": " << packed_nrmse << ", \"relative_peak_error\": " << packed_peak
         << ", \"memory_saving\": " << packed_memory_saving
         << ", \"minimum_gpu_time_improvement\": " << GlobalReflectionProbe::kPackedMinGpuTimeImprovement
         << ", \"gpu_time_improvement_measured\": false, \"adopted\": false, "
            "\"active_format\": \"RGBA16F\", \"quality_cases\": [\"bright_glossy\", \"roughness_extremes\", "
            "\"dark_saturated_gradients\", \"small_intense_emitter\"]},\n  \"memory\": "
            "{\"canonical_gpu_image_logical_bytes\": "
         << GlobalReflectionProbe::kCanonicalPayloadByteSize
         << ", \"canonical_steady_cpu_payload_bytes\": " << GlobalReflectionProbe::kCanonicalPayloadByteSize
         << ", \"canonical_serialized_payload_bytes\": " << GlobalReflectionProbe::kCanonicalPayloadByteSize
         << ", \"packed_gpu_image_logical_bytes\": " << GlobalReflectionProbe::kPackedRuntimeByteSize
         << ", \"packed_steady_cpu_payload_bytes\": 0, \"packed_serialized_payload_bytes\": 0},\n  \"checks\": {";
  bool passed = true;
  for (size_t index = 0; index < checks.size(); ++index) {
    report << (index == 0u ? "" : ",") << "\n    \"" << checks[index].first
           << "\": " << (checks[index].second ? "true" : "false");
    passed &= checks[index].second;
  }
  report << "\n  },\n  \"passed\": " << (passed ? "true" : "false") << "\n}\n";
  report.close();

  std::cout << "EVOENGINE_REFLECTION_PROBE_REPORT path=\"" << report_path.string()
            << "\" passed=" << (passed ? "true" : "false") << " box_projection_nrmse=" << box_projection_delta
            << " owner_reorder_nrmse=" << reorder_delta << " missing_removal_nrmse=" << missing_removal_delta
            << std::endl;
  if (!passed) {
    throw std::runtime_error("Reflection probe validation report contains failed checks.");
  }
  editor_layer->SetSceneCameraResolutionOverride(std::nullopt);
  return true;
}

bool evo_engine::RunEnvironmentLightingValidationFromEnvironment(const int width, const int height) {
  const auto* evidence_path = std::getenv("EVOENGINE_ENVIRONMENT_LIGHTING_EVIDENCE");
  if (!evidence_path) {
    return false;
  }
  if (width != 1920 || height != 1080) {
    throw std::runtime_error("Environment lighting validation requires 1920x1080.");
  }
  if (!Platform::RayTracingEnabled()) {
    throw std::runtime_error("Environment lighting validation requires ray tracing for DDGI invalidation telemetry.");
  }
  if (!Platform::GraphicsValidationEnabled()) {
    throw std::runtime_error("Environment lighting validation requires Vulkan validation.");
  }

  const auto output_directory = std::filesystem::path(evidence_path);
  if (output_directory.empty()) {
    throw std::runtime_error("Environment lighting validation evidence path is empty.");
  }
  std::filesystem::create_directories(output_directory);

  const auto scene = ApplicationContext::Get().GetActiveScene();
  const auto editor_layer = ApplicationContext::Get().GetLayer<EditorLayer>();
  const auto render_layer = ApplicationContext::Get().GetLayer<RenderLayer>();
  if (!scene || !editor_layer || !render_layer) {
    throw std::runtime_error("Environment lighting validation requires an active scene, EditorLayer, and RenderLayer.");
  }
  ConfigureEnvironmentLightingValidationScene(scene);

  if (const auto window_layer = ApplicationContext::Get().GetLayer<WindowLayer>()) {
    window_layer->ResizeWindow(width, height);
    window_layer->CenterWindow();
  }
  const glm::uvec2 resolution(static_cast<uint32_t>(width), static_cast<uint32_t>(height));
  editor_layer->show_camera_window = false;
  editor_layer->RequestSceneCameraPreviewWindow(resolution);
  editor_layer->SetSceneCameraResolutionOverride(resolution);
  const auto scene_camera = editor_layer->GetSceneCamera();
  if (!scene_camera) {
    throw std::runtime_error("Environment lighting validation requires a scene camera.");
  }
  scene_camera->SetRequireRendering(true);
  scene_camera->Resize(resolution);
  scene_camera->ResetFrameCount();
  if (const auto main_camera = scene->main_camera.Get<Camera>(); main_camera && main_camera != scene_camera) {
    main_camera->SetEnabled(false);
  }
  const auto initial_fixed_post = scene_camera->post_processing_stack_ref.Get<PostProcessingStack>();
  const bool fixed_post_processing =
      initial_fixed_post && !initial_fixed_post->enable_ambient_occlusion && !initial_fixed_post->enable_bloom &&
      !initial_fixed_post->enable_screen_space_reflection && !initial_fixed_post->enable_anti_aliasing &&
      initial_fixed_post->enable_tone_mapping && initial_fixed_post->tone_mapping &&
      !initial_fixed_post->tone_mapping->auto_exposure && !initial_fixed_post->tone_mapping->dither;

  const auto wait_for_scene_inputs = [](const char* phase) {
    constexpr size_t max_wait_frames = 30000;
    constexpr size_t settled_frame_count = 4;
    size_t stable_input_frames = 0;
    for (size_t frame = 0; frame < max_wait_frames && stable_input_frames < settled_frame_count; ++frame) {
      const bool ready = ProjectManager::IsProjectIdle() && !AssetManager::GetAssetLoadSnapshot().Active() &&
                         !TextureStorage::HasPendingUploads() && !GeometryStorage::HasPendingUploads() &&
                         !BottomLevelAccelerationStructure::HasPendingStaticBuilds();
      stable_input_frames = ready ? stable_input_frames + 1u : 0u;
      if (stable_input_frames < settled_frame_count && !ApplicationContext::Get().Loop()) {
        throw std::runtime_error(std::string("Application ended while waiting for ") + phase + ".");
      }
    }
    if (stable_input_frames < settled_frame_count) {
      throw std::runtime_error(std::string("Environment lighting input readiness timed out during ") + phase + ".");
    }
  };
  wait_for_scene_inputs("the control-matrix fixture");

  constexpr size_t max_convergence_frames = 256;
  size_t convergence_frames = 0;
  for (; convergence_frames < max_convergence_frames; ++convergence_frames) {
    if (!ApplicationContext::Get().Loop()) {
      throw std::runtime_error("Application ended during environment lighting DDGI convergence.");
    }
    if (render_layer->GetDdgiInspectorSnapshot().aggregate.probe_variability_converged) {
      ++convergence_frames;
      break;
    }
  }
  if (!render_layer->GetDdgiInspectorSnapshot().aggregate.probe_variability_converged) {
    throw std::runtime_error("Environment lighting DDGI invalidation fixture did not converge.");
  }

  struct RegionEvidence {
    glm::dvec3 average_color = glm::dvec3(0.0);
    double average_luminance = 0.0;
    double linear_luminance = 0.0;
    uint64_t sample_count = 0;
  };
  const std::array regions = {
      std::pair{"background", std::array{glm::vec2(0.05f, 0.05f), glm::vec2(0.20f, 0.20f)}},
      std::pair{"dielectric", std::array{glm::vec2(0.30f, 0.44f), glm::vec2(0.36f, 0.53f)}},
      std::pair{"metal", std::array{glm::vec2(0.64f, 0.44f), glm::vec2(0.70f, 0.53f)}},
      std::pair{"ibl_diffuse", std::array{glm::vec2(0.72f, 0.19f), glm::vec2(0.76f, 0.25f)}},
      std::pair{"direct", std::array{glm::vec2(0.43f, 0.12f), glm::vec2(0.47f, 0.20f)}},
      std::pair{"emission", std::array{glm::vec2(0.53f, 0.12f), glm::vec2(0.57f, 0.20f)}},
      std::pair{"sponza_exact_metal", std::array{glm::vec2(0.435f, 0.375f), glm::vec2(0.465f, 0.425f)}},
      std::pair{"sponza_rough_metal", std::array{glm::vec2(0.535f, 0.375f), glm::vec2(0.565f, 0.425f)}},
      std::pair{"sponza_smooth_dielectric", std::array{glm::vec2(0.435f, 0.565f), glm::vec2(0.465f, 0.615f)}},
      std::pair{"sponza_rough_dielectric", std::array{glm::vec2(0.535f, 0.565f), glm::vec2(0.565f, 0.615f)}},
      std::pair{"sponza_room", std::array{glm::vec2(0.20f, 0.20f), glm::vec2(0.80f, 0.82f)}}};
  struct CaptureEvidence {
    std::string name;
    std::string render_mode;
    std::string ambient_occlusion;
    float background_intensity = 0.0f;
    float environment_intensity = 0.0f;
    float diffuse_fallback_intensity = 0.0f;
    float specular_fallback_intensity = 0.0f;
    float local_probe_intensity = 0.0f;
    size_t settle_frames = 0;
    int indirect_debug_view = 0;
    uint32_t local_probe_count = 0;
    uint32_t valid_local_probe_count = 0;
    bool ddgi_enabled = false;
    std::vector<glm::vec4> pixels;
    std::map<std::string, RegionEvidence> regions;
    bool finite = true;
  };

  const auto inverse_filmic = [](const double mapped) {
    double low = 0.0;
    double high = 64.0;
    for (int iteration = 0; iteration < 32; ++iteration) {
      const double color = (low + high) * 0.5;
      const double temporary = std::max(0.0, color - 0.004);
      const double filmic = temporary * (6.2 * temporary + 0.5) / (temporary * (6.2 * temporary + 1.7) + 0.06);
      if (filmic < mapped) {
        low = color;
      } else {
        high = color;
      }
    }
    return (low + high) * 0.5;
  };
  const auto summarize_region = [&](const std::vector<glm::vec4>& pixels, const std::array<glm::vec2, 2>& region) {
    RegionEvidence evidence;
    const auto begin = glm::uvec2(glm::floor(region[0] * glm::vec2(resolution)));
    const auto end = glm::uvec2(glm::ceil(region[1] * glm::vec2(resolution)));
    for (uint32_t y = begin.y; y < end.y; ++y) {
      const uint32_t source_y = resolution.y - 1u - y;
      for (uint32_t x = begin.x; x < end.x; ++x) {
        const auto color =
            glm::max(glm::dvec3(pixels[static_cast<size_t>(source_y) * resolution.x + x]), glm::dvec3(0.0));
        evidence.average_color += color;
        evidence.average_luminance += glm::dot(color, glm::dvec3(0.2126, 0.7152, 0.0722));
        evidence.linear_luminance +=
            glm::dot(glm::dvec3(inverse_filmic(color.x), inverse_filmic(color.y), inverse_filmic(color.z)),
                     glm::dvec3(0.2126, 0.7152, 0.0722));
        ++evidence.sample_count;
      }
    }
    if (evidence.sample_count != 0u) {
      evidence.average_color /= static_cast<double>(evidence.sample_count);
      evidence.average_luminance /= static_cast<double>(evidence.sample_count);
      evidence.linear_luminance /= static_cast<double>(evidence.sample_count);
    }
    return evidence;
  };

  const auto store_capture = [&](const char* name, const Camera::CameraRenderMode render_mode,
                                 const float background_intensity, const float environment_intensity,
                                 const float diffuse_fallback_intensity, const float specular_fallback_intensity,
                                 const size_t settle_frames) {
    CaptureEvidence evidence;
    evidence.name = name;
    evidence.render_mode = Camera::GetCameraRenderModeName(render_mode);
    const auto capture_path = output_directory / (evidence.name + ".png");
    evidence.pixels = ReadAndStoreValidationCapture(scene_camera->GetRenderTexture(), resolution, capture_path,
                                                    "Environment lighting");
    evidence.background_intensity = background_intensity;
    evidence.environment_intensity = environment_intensity;
    evidence.diffuse_fallback_intensity = diffuse_fallback_intensity;
    evidence.specular_fallback_intensity = specular_fallback_intensity;
    evidence.settle_frames = settle_frames;
    evidence.indirect_debug_view = static_cast<int>(render_layer->render_settings.indirect_lighting_debug_view);
    const auto lighting = scene->environmental_lighting.Get<EnvironmentalLighting>();
    evidence.ddgi_enabled = lighting && lighting->ddgi_settings.runtime.enabled;
    const auto post_processing_stack = scene_camera->post_processing_stack_ref.Get<PostProcessingStack>();
    if (!post_processing_stack || !post_processing_stack->enable_ambient_occlusion) {
      evidence.ambient_occlusion = "disabled";
    } else if (!post_processing_stack->ambient_occlusion) {
      evidence.ambient_occlusion = "unavailable";
    } else {
      evidence.ambient_occlusion =
          post_processing_stack->ambient_occlusion->algorithm == AmbientOcclusion::Algorithm::Gtao ? "gtao" : "ssao";
    }
    if (const auto storage = render_layer->GetCurrentRenderInstanceStorage()) {
      evidence.local_probe_count = storage->GetReflectionProbeCount();
      const auto& probe_infos = storage->GetReflectionProbeInfoBlocks();
      for (uint32_t index = 0; index < evidence.local_probe_count; ++index) {
        evidence.valid_local_probe_count += probe_infos[index].identity_and_flags.y != 0u ? 1u : 0u;
      }
    }
    if (const auto* probe = FindEnvironmentalLightingLocalReflectionProbe(scene, kSponzaLocalProbeNames.front())) {
      evidence.local_probe_intensity = probe->reflection_intensity;
    }
    for (const auto& pixel : evidence.pixels) {
      evidence.finite = evidence.finite && std::isfinite(pixel.x) && std::isfinite(pixel.y) && std::isfinite(pixel.z) &&
                        std::isfinite(pixel.w);
    }
    for (const auto& [region_name, region] : regions) {
      evidence.regions.emplace(region_name, summarize_region(evidence.pixels, region));
    }
    return evidence;
  };
  const auto capture = [&](const char* name, const Camera::CameraRenderMode render_mode,
                           const float background_intensity, const float environment_intensity,
                           const float diffuse_fallback_intensity, const float specular_fallback_intensity,
                           const size_t settle_frames) {
    SetEnvironmentalLightingIntensity(scene, environment_intensity);
    if (const auto lighting = GetOrCreateTemporaryEnvironmentalLighting(scene)) {
      SetEnvironmentalLightingFallbackIntensities(*lighting, diffuse_fallback_intensity, specular_fallback_intensity);
    }
    SyncTemporaryEnvironmentalLightingSettingsFromScene(scene);
    scene_camera->camera_settings.background_intensity = background_intensity;
    scene_camera->camera_render_mode = render_mode;
    scene_camera->ResetFrameCount();
    const bool wait_for_accumulation = Camera::IsRayCameraRenderMode(render_mode);
    constexpr size_t max_capture_frame_slack = 30000;
    size_t frame = 0;
    while ((wait_for_accumulation && scene_camera->GetFrameCount() < settle_frames) ||
           (!wait_for_accumulation && frame < settle_frames)) {
      if (!ApplicationContext::Get().Loop()) {
        throw std::runtime_error(std::string("Application ended during environment lighting phase: ") + name);
      }
      if (++frame >= settle_frames + max_capture_frame_slack) {
        throw std::runtime_error(std::string("Environment lighting phase timed out: ") + name);
      }
    }
    return store_capture(name, render_mode, background_intensity, environment_intensity, diffuse_fallback_intensity,
                         specular_fallback_intensity, settle_frames);
  };

  std::vector<CaptureEvidence> captures;
  captures.reserve(35);
  captures.emplace_back(capture("default", Camera::CameraRenderMode::Rasterization, 0.0f, 1.0f, 1.0f, 1.0f, 4u));

  SetEnvironmentalLightingDiffuseFallback(scene, 0.0f);
  SyncTemporaryEnvironmentalLightingSettingsFromScene(scene);
  if (!ApplicationContext::Get().Loop()) {
    throw std::runtime_error("Application ended during the indirect-intensity DDGI invalidation check.");
  }
  const auto indirect_update_reasons = render_layer->GetDdgiInspectorSnapshot().last_probe_update_reasons;
  const auto indirect_update_stats = render_layer->GetDdgiInspectorSnapshot().aggregate;
  captures.emplace_back(
      capture("diffuse-fallback-off", Camera::CameraRenderMode::Rasterization, 0.0f, 1.0f, 0.0f, 1.0f, 4u));

  SetEnvironmentalLightingIntensity(scene, 0.0f);
  SyncTemporaryEnvironmentalLightingSettingsFromScene(scene);
  if (!ApplicationContext::Get().Loop()) {
    throw std::runtime_error("Application ended during the sky-scale DDGI invalidation check.");
  }
  const auto sky_update_snapshot = render_layer->GetDdgiInspectorSnapshot();
  const auto sky_update_reasons = sky_update_snapshot.last_probe_update_reasons;
  const auto sky_update_stats = sky_update_snapshot.aggregate;
  captures.emplace_back(capture("all-off", Camera::CameraRenderMode::Rasterization, 0.0f, 0.0f, 0.0f, 0.0f, 4u));
  captures.emplace_back(
      capture("environment-off", Camera::CameraRenderMode::Rasterization, 0.0f, 0.0f, 1.0f, 1.0f, 4u));
  captures.emplace_back(
      capture("background-only", Camera::CameraRenderMode::Rasterization, 1.0f, 0.0f, 0.0f, 0.0f, 4u));
  captures.emplace_back(capture("background-lit", Camera::CameraRenderMode::Rasterization, 1.0f, 1.0f, 1.0f, 1.0f, 4u));
  captures.emplace_back(
      capture("environment-double", Camera::CameraRenderMode::Rasterization, 0.0f, 2.0f, 1.0f, 1.0f, 4u));
  captures.emplace_back(
      capture("diffuse-fallback-double", Camera::CameraRenderMode::Rasterization, 0.0f, 1.0f, 2.0f, 1.0f, 4u));
  captures.emplace_back(
      capture("specular-fallback-off", Camera::CameraRenderMode::Rasterization, 0.0f, 1.0f, 1.0f, 0.0f, 4u));
  captures.emplace_back(
      capture("specular-fallback-double", Camera::CameraRenderMode::Rasterization, 0.0f, 1.0f, 1.0f, 2.0f, 4u));
  captures.emplace_back(
      capture("fallbacks-double", Camera::CameraRenderMode::Rasterization, 0.0f, 1.0f, 2.0f, 2.0f, 4u));
  captures.emplace_back(
      capture("ray-neutral-reference", Camera::CameraRenderMode::RayTracing, 0.0f, 1.0f, 1.0f, 1.0f, 48u));
  captures.emplace_back(
      capture("ray-environment-double", Camera::CameraRenderMode::RayTracing, 0.0f, 2.0f, 1.0f, 1.0f, 48u));
  captures.emplace_back(
      capture("ray-background-lit", Camera::CameraRenderMode::RayTracing, 1.0f, 1.0f, 1.0f, 1.0f, 48u));
  captures.emplace_back(
      capture("ray-fallbacks-off", Camera::CameraRenderMode::RayTracing, 0.0f, 1.0f, 0.0f, 0.0f, 48u));
  captures.emplace_back(
      capture("ray-fallbacks-double", Camera::CameraRenderMode::RayTracing, 0.0f, 1.0f, 2.0f, 2.0f, 48u));
  captures.emplace_back(
      capture("ray-environment-off", Camera::CameraRenderMode::RayTracing, 0.0f, 0.0f, 1.0f, 1.0f, 48u));
  captures.emplace_back(capture("ray-all-off", Camera::CameraRenderMode::RayTracing, 0.0f, 0.0f, 0.0f, 0.0f, 48u));

  ConfigureDdgiValidationFixture(scene, "sponza");
  const auto sponza_spheres = FindEntityNamed(scene, "Spheres");
  if (!sponza_spheres) {
    throw std::runtime_error("Environment lighting Sponza validation is missing the material spheres.");
  }
  scene->SetEnable(*sponza_spheres, true);
  wait_for_scene_inputs("the Sponza exact-metal fixture");
  captures.emplace_back(capture("sponza-default", Camera::CameraRenderMode::Rasterization, 0.0f, 1.0f, 1.0f, 1.0f, 4u));

  auto& m15_sponza_ddgi = RequireEnvironmentalLightingDdgiSettings(scene);
  m15_sponza_ddgi.runtime.deterministic_ray_seed_enabled = true;
  m15_sponza_ddgi.runtime.deterministic_ray_seed = 0x6d2b79f5u;
  SyncTemporaryEnvironmentalLightingSettingsFromScene(scene);
  constexpr size_t m15_sponza_warmup_frames = 64;
  for (size_t frame = 0; frame < m15_sponza_warmup_frames; ++frame) {
    if (!ApplicationContext::Get().Loop()) {
      throw std::runtime_error("Application ended during M15 Sponza warmup.");
    }
  }
  RequestDdgiHistoryReset();
  SyncTemporaryEnvironmentalLightingSettingsFromScene(scene);
  constexpr size_t m15_max_sponza_convergence_frames = 1024;
  size_t m15_sponza_convergence_frames = 0;
  for (; m15_sponza_convergence_frames < m15_max_sponza_convergence_frames; ++m15_sponza_convergence_frames) {
    if (!ApplicationContext::Get().Loop()) {
      throw std::runtime_error("Application ended during M15 Sponza DDGI convergence.");
    }
    if (render_layer->GetDdgiInspectorSnapshot().aggregate.probe_variability_converged) {
      ++m15_sponza_convergence_frames;
      break;
    }
  }
  if (!render_layer->GetDdgiInspectorSnapshot().aggregate.probe_variability_converged) {
    throw std::runtime_error("M15 Sponza DDGI fixture did not converge.");
  }
  m15_sponza_ddgi.volume_defaults.enable_probe_variability_gating = false;
  if (const auto lighting = scene->environmental_lighting.Get<EnvironmentalLighting>()) {
    lighting->ddgi_settings = m15_sponza_ddgi;
    for (auto& volume : lighting->GetOrCreateDdgiVolumePack()->volumes) {
      volume.enable_probe_variability_gating = false;
    }
  }
  SetDdgiUpdatesPaused(true);
  constexpr size_t m15_sponza_history_settle_frames = 4;
  for (size_t frame = 0; frame < m15_sponza_history_settle_frames; ++frame) {
    if (!ApplicationContext::Get().Loop()) {
      throw std::runtime_error("Application ended while freezing M15 Sponza DDGI history.");
    }
  }

  const auto sponza_post_processing = scene_camera->post_processing_stack_ref.Get<PostProcessingStack>();
  if (!sponza_post_processing || !sponza_post_processing->ambient_occlusion) {
    throw std::runtime_error("M15 Sponza validation requires ambient occlusion resources.");
  }
  const auto set_sponza_ambient_occlusion = [&](const bool enabled, const AmbientOcclusion::Algorithm algorithm) {
    sponza_post_processing->enable_ambient_occlusion = enabled;
    sponza_post_processing->ambient_occlusion->algorithm = algorithm;
    sponza_post_processing->ambient_occlusion->gtao_radius = 0.8f;
    sponza_post_processing->ambient_occlusion->gtao_intensity = 1.5f;
  };
  const auto set_sponza_indirect_debug = [&](const RenderSettings::IndirectLightingDebugView view) {
    render_layer->render_settings.indirect_lighting_debug_view = view;
  };
  auto* first_sponza_probe = FindEnvironmentalLightingLocalReflectionProbe(scene, kSponzaLocalProbeNames.front());
  if (!first_sponza_probe) {
    throw std::runtime_error("M15 Sponza validation is missing its persistent local reflection probes.");
  }
  const float first_sponza_probe_intensity = first_sponza_probe->reflection_intensity;

  set_sponza_ambient_occlusion(false, AmbientOcclusion::Algorithm::Gtao);
  set_sponza_indirect_debug(RenderSettings::IndirectLightingDebugView::DiffuseIndirect);
  captures.emplace_back(capture("sponza-diffuse", Camera::CameraRenderMode::Rasterization, 0.0f, 1.0f, 1.0f, 1.0f, 4u));
  set_sponza_indirect_debug(RenderSettings::IndirectLightingDebugView::UnoccludedProbeSpecular);
  captures.emplace_back(
      capture("sponza-unoccluded", Camera::CameraRenderMode::Rasterization, 0.0f, 1.0f, 1.0f, 1.0f, 4u));

  set_sponza_ambient_occlusion(true, AmbientOcclusion::Algorithm::Gtao);
  set_sponza_indirect_debug(RenderSettings::IndirectLightingDebugView::Beauty);
  captures.emplace_back(
      capture("sponza-gtao-beauty", Camera::CameraRenderMode::Rasterization, 0.0f, 1.0f, 1.0f, 1.0f, 4u));
  set_sponza_indirect_debug(RenderSettings::IndirectLightingDebugView::DiffuseIndirect);
  captures.emplace_back(
      capture("sponza-gtao-diffuse", Camera::CameraRenderMode::Rasterization, 0.0f, 1.0f, 1.0f, 1.0f, 4u));
  first_sponza_probe->reflection_intensity = 2.0f;
  ClampEnvironmentalLightingLocalReflectionProbe(*first_sponza_probe);
  captures.emplace_back(
      capture("sponza-gtao-diffuse-probe-double", Camera::CameraRenderMode::Rasterization, 0.0f, 1.0f, 1.0f, 1.0f, 4u));
  first_sponza_probe->reflection_intensity = first_sponza_probe_intensity;
  ClampEnvironmentalLightingLocalReflectionProbe(*first_sponza_probe);
  set_sponza_indirect_debug(RenderSettings::IndirectLightingDebugView::SpecularVisibility);
  captures.emplace_back(
      capture("sponza-gtao-visibility", Camera::CameraRenderMode::Rasterization, 0.0f, 1.0f, 1.0f, 1.0f, 4u));
  set_sponza_indirect_debug(RenderSettings::IndirectLightingDebugView::OccludedProbeSpecular);
  captures.emplace_back(
      capture("sponza-gtao-occluded", Camera::CameraRenderMode::Rasterization, 0.0f, 1.0f, 1.0f, 1.0f, 4u));

  set_sponza_ambient_occlusion(true, AmbientOcclusion::Algorithm::Ssao);
  set_sponza_indirect_debug(RenderSettings::IndirectLightingDebugView::DiffuseIndirect);
  captures.emplace_back(
      capture("sponza-ssao-diffuse", Camera::CameraRenderMode::Rasterization, 0.0f, 1.0f, 1.0f, 1.0f, 4u));
  set_sponza_indirect_debug(RenderSettings::IndirectLightingDebugView::OccludedProbeSpecular);
  captures.emplace_back(
      capture("sponza-ssao-occluded", Camera::CameraRenderMode::Rasterization, 0.0f, 1.0f, 1.0f, 1.0f, 4u));

  set_sponza_ambient_occlusion(true, AmbientOcclusion::Algorithm::Gtao);
  captures.emplace_back(capture("sponza-gtao-occluded-indirect-double", Camera::CameraRenderMode::Rasterization, 0.0f,
                                1.0f, 2.0f, 1.0f, 4u));
  m15_sponza_ddgi.runtime.enabled = false;
  captures.emplace_back(capture("sponza-gtao-occluded-ddgi-disabled", Camera::CameraRenderMode::Rasterization, 0.0f,
                                1.0f, 1.0f, 1.0f, 4u));
  m15_sponza_ddgi.runtime.enabled = true;

  std::vector<std::pair<size_t, glm::mat4>> sponza_volume_transforms;
  if (const auto lighting = scene->environmental_lighting.Get<EnvironmentalLighting>()) {
    auto ddgi_pack = lighting->GetOrCreateDdgiVolumePack();
    sponza_volume_transforms.reserve(ddgi_pack->volumes.size());
    for (size_t index = 0; index < ddgi_pack->volumes.size(); ++index) {
      auto& volume = ddgi_pack->volumes[index];
      sponza_volume_transforms.emplace_back(index, volume.transform);
      Transform transform;
      transform.value = volume.transform;
      transform.SetPosition(transform.GetPosition() + glm::vec3(1000.0f));
      volume.transform = transform.value;
    }
  }
  captures.emplace_back(capture("sponza-gtao-occluded-ddgi-outside", Camera::CameraRenderMode::Rasterization, 0.0f,
                                1.0f, 1.0f, 1.0f, 4u));
  if (const auto lighting = scene->environmental_lighting.Get<EnvironmentalLighting>()) {
    const auto ddgi_pack = lighting->GetOrCreateDdgiVolumePack();
    for (const auto& [index, transform] : sponza_volume_transforms) {
      if (index < ddgi_pack->volumes.size()) {
        ddgi_pack->volumes[index].transform = transform;
      }
    }
  }

  const auto capture_sponza_room = [&](const char* name, const glm::vec3& position, const glm::vec3& target) {
    editor_layer->SetSceneCameraPosition(position);
    editor_layer->SetSceneCameraRotation(
        glm::quatLookAt(glm::normalize(target - position), glm::vec3(0.0f, 1.0f, 0.0f)));
    captures.emplace_back(capture(name, Camera::CameraRenderMode::Rasterization, 0.0f, 1.0f, 1.0f, 1.0f, 4u));
  };
  capture_sponza_room("sponza-gtao-left-room", glm::vec3(-1.85f, 1.25f, 0.0f), glm::vec3(-1.85f, 1.25f, -3.4f));
  capture_sponza_room("sponza-gtao-right-room", glm::vec3(2.2f, 1.25f, 0.0f), glm::vec3(2.2f, 1.25f, -3.4f));

  editor_layer->SetSceneCameraPosition(glm::vec3(0.0f, 0.0f, 3.0f));
  editor_layer->SetSceneCameraRotation(glm::quat(glm::vec3(0.0f)));
  m15_sponza_ddgi.runtime.enabled = true;
  SetDdgiUpdatesPaused(false);
  SetEnvironmentalLightingDiffuseFallback(scene, 1.0f);
  SyncTemporaryEnvironmentalLightingSettingsFromScene(scene);
  set_sponza_ambient_occlusion(false, AmbientOcclusion::Algorithm::Gtao);
  set_sponza_indirect_debug(RenderSettings::IndirectLightingDebugView::Beauty);

  ConfigureDdgiValidationFixture(scene, "sponza");
  auto& canonical_sponza_ddgi = RequireEnvironmentalLightingDdgiSettings(scene);
  canonical_sponza_ddgi.runtime.deterministic_ray_seed_enabled = true;
  canonical_sponza_ddgi.runtime.deterministic_ray_seed = 0x6d2b79f5u;
  SyncTemporaryEnvironmentalLightingSettingsFromScene(scene);
  const auto canonical_sponza_spheres = FindEntityNamed(scene, "Spheres");
  const auto canonical_sponza_title = FindEntityNamed(scene, "Title");
  const bool canonical_sponza_entities_disabled = canonical_sponza_spheres && canonical_sponza_title &&
                                                  !scene->IsEntityEnabled(*canonical_sponza_spheres) &&
                                                  !scene->IsEntityEnabled(*canonical_sponza_title);
  if (!canonical_sponza_entities_disabled) {
    throw std::runtime_error("Canonical Sponza validation entities are not disabled.");
  }
  wait_for_scene_inputs("the canonical Sponza repeatability fixture");
  scene_camera->camera_settings.background_intensity = 1.0f;
  scene_camera->ResetFrameCount();
  if (const auto stack = scene_camera->post_processing_stack_ref.Get<PostProcessingStack>()) {
    stack->enable_ambient_occlusion = false;
    stack->enable_bloom = false;
    stack->enable_screen_space_reflection = false;
    stack->enable_anti_aliasing = false;
    stack->enable_tone_mapping = true;
    if (stack->tone_mapping) {
      stack->tone_mapping->auto_exposure = false;
      stack->tone_mapping->dither = false;
    }
  }
  constexpr size_t canonical_warmup_frames = 64;
  constexpr size_t canonical_measure_frames = 120;
  for (size_t frame = 0; frame < canonical_warmup_frames; ++frame) {
    if (!ApplicationContext::Get().Loop()) {
      throw std::runtime_error("Application ended during canonical Sponza warmup.");
    }
  }
  RequestDdgiHistoryReset();
  SyncTemporaryEnvironmentalLightingSettingsFromScene(scene);
  constexpr size_t max_sponza_convergence_frames = 1024;
  size_t sponza_convergence_frames = 0;
  for (; sponza_convergence_frames < max_sponza_convergence_frames; ++sponza_convergence_frames) {
    if (!ApplicationContext::Get().Loop()) {
      throw std::runtime_error("Application ended during canonical Sponza DDGI convergence.");
    }
    if (render_layer->GetDdgiInspectorSnapshot().aggregate.probe_variability_converged) {
      ++sponza_convergence_frames;
      break;
    }
  }
  if (!render_layer->GetDdgiInspectorSnapshot().aggregate.probe_variability_converged) {
    throw std::runtime_error("Canonical Sponza DDGI fixture did not converge.");
  }
  canonical_sponza_ddgi.volume_defaults.enable_probe_variability_gating = false;
  if (const auto lighting = scene->environmental_lighting.Get<EnvironmentalLighting>()) {
    lighting->ddgi_settings = canonical_sponza_ddgi;
    for (auto& volume : lighting->GetOrCreateDdgiVolumePack()->volumes) {
      volume.enable_probe_variability_gating = false;
    }
  }
  SyncTemporaryEnvironmentalLightingSettingsFromScene(scene);
  constexpr size_t canonical_preparation_frames = 8;
  for (size_t frame = 0; frame < canonical_preparation_frames; ++frame) {
    if (!ApplicationContext::Get().Loop()) {
      throw std::runtime_error("Application ended during canonical Sponza measurement preparation.");
    }
  }
  for (size_t frame = 0; frame < canonical_measure_frames; ++frame) {
    if (!ApplicationContext::Get().Loop()) {
      throw std::runtime_error("Application ended during canonical Sponza measurement.");
    }
  }
  const bool canonical_sponza_anchor_contract =
      canonical_sponza_entities_disabled && canonical_sponza_ddgi.runtime.enabled &&
      canonical_sponza_ddgi.runtime.deterministic_ray_seed_enabled &&
      canonical_sponza_ddgi.runtime.deterministic_ray_seed == 0x6d2b79f5u &&
      scene_camera->camera_render_mode == Camera::CameraRenderMode::Rasterization &&
      scene_camera->camera_settings.sample_size == 1 && scene_camera->camera_settings.background_intensity == 1.0f &&
      scene->environmental_lighting.Get<EnvironmentalLighting>() &&
      scene->environmental_lighting.Get<EnvironmentalLighting>()->environment_lighting_intensity == 1.0f &&
      scene->environmental_lighting.Get<EnvironmentalLighting>()->diffuse_fallback_intensity == 1.0f;
  captures.emplace_back(store_capture("sponza-repeatability-anchor", Camera::CameraRenderMode::Rasterization, 1.0f,
                                      1.0f, 1.0f, 1.0f, 0u));

  ConfigureDdgiValidationFixture(scene, "sponza");
  auto& reference_sponza_ddgi = RequireEnvironmentalLightingDdgiSettings(scene);
  reference_sponza_ddgi.runtime.deterministic_ray_seed_enabled = true;
  reference_sponza_ddgi.runtime.deterministic_ray_seed = 0x6d2b79f5u;
  const auto reference_sponza_spheres = FindEntityNamed(scene, "Spheres");
  const auto reference_sponza_title = FindEntityNamed(scene, "Title");
  const bool reference_sponza_entities_disabled = reference_sponza_spheres && reference_sponza_title &&
                                                  !scene->IsEntityEnabled(*reference_sponza_spheres) &&
                                                  !scene->IsEntityEnabled(*reference_sponza_title);
  if (!reference_sponza_entities_disabled) {
    throw std::runtime_error("Canonical Sponza reference entities are not disabled.");
  }
  reference_sponza_ddgi.runtime.enabled = false;
  scene_camera->camera_settings.sample_size = 4;
  scene_camera->camera_settings.bounce = 4;
  scene_camera->camera_settings.ray_debug_view = CameraSettings::RayDebugView::Beauty;
  scene_camera->camera_settings.shader_execution_reordering_mode =
      CameraSettings::ShaderExecutionReorderingMode::Disabled;
  scene_camera->camera_settings.firefly_clamp_threshold = 10.0f;
  scene_camera->camera_settings.auto_spp_enabled = false;
  scene_camera->camera_settings.background_intensity = 1.0f;
  scene_camera->camera_render_mode = Camera::CameraRenderMode::RayTracing;
  SetEnvironmentalLightingIntensity(scene, 1.0f);
  SetEnvironmentalLightingDiffuseFallback(scene, 1.0f);
  SyncTemporaryEnvironmentalLightingSettingsFromScene(scene);
  wait_for_scene_inputs("the canonical Sponza ray reference fixture");
  if (Camera::ResolveCameraRenderMode(scene_camera->camera_render_mode) != Camera::CameraRenderMode::RayTracing) {
    throw std::runtime_error("Canonical Sponza reference requires the Vulkan ray-tracing pipeline.");
  }
  constexpr size_t max_ray_variant_wait_frames = 30000;
  size_t ray_variant_wait_frames = 0;
  while (!render_layer->IsRayCameraShaderVariantReady(RayCameraShaderTechnique::RayTracing)) {
    const auto stats = render_layer->GetRayCameraShaderVariantStats(RayCameraShaderTechnique::RayTracing);
    if (stats.failed) {
      throw std::runtime_error("Canonical Sponza ray shader variant failed: " + stats.last_error);
    }
    if (!ApplicationContext::Get().Loop()) {
      throw std::runtime_error("Application ended before the canonical Sponza ray shader became ready.");
    }
    if (++ray_variant_wait_frames >= max_ray_variant_wait_frames) {
      throw std::runtime_error("Canonical Sponza ray shader variant timed out.");
    }
  }
  captures.emplace_back(
      capture("sponza-ray-reference", Camera::CameraRenderMode::RayTracing, 1.0f, 1.0f, 1.0f, 1.0f, 64u));
  const bool canonical_sponza_ray_contract =
      reference_sponza_entities_disabled && !reference_sponza_ddgi.runtime.enabled &&
      reference_sponza_ddgi.runtime.deterministic_ray_seed_enabled &&
      reference_sponza_ddgi.runtime.deterministic_ray_seed == 0x6d2b79f5u && scene_camera->GetFrameCount() == 64u &&
      Camera::ResolveCameraRenderMode(scene_camera->camera_render_mode) == Camera::CameraRenderMode::RayTracing &&
      scene_camera->camera_settings.sample_size == 4 && scene_camera->camera_settings.bounce == 4 &&
      scene_camera->camera_settings.ray_debug_view == CameraSettings::RayDebugView::Beauty &&
      scene_camera->camera_settings.shader_execution_reordering_mode ==
          CameraSettings::ShaderExecutionReorderingMode::Disabled &&
      scene_camera->camera_settings.firefly_clamp_threshold == 10.0f &&
      !scene_camera->camera_settings.auto_spp_enabled && scene_camera->camera_settings.background_intensity == 1.0f &&
      scene->environmental_lighting.Get<EnvironmentalLighting>() &&
      scene->environmental_lighting.Get<EnvironmentalLighting>()->environment_lighting_intensity == 1.0f &&
      scene->environmental_lighting.Get<EnvironmentalLighting>()->diffuse_fallback_intensity == 1.0f;
  if (!canonical_sponza_ray_contract) {
    throw std::runtime_error("Canonical Sponza ray reference contract changed before publication.");
  }
  std::cout << "EVOENGINE_DDGI_REFERENCE fixture=sponza render_mode=RayTracing resolution=1920x1080 "
               "frames=64 spp_per_frame=4 total_spp=256 output=.png"
            << std::endl;

  const auto& defaults = captures[0];
  const auto& diffuse_fallback_off = captures[1];
  const auto& all_off = captures[2];
  const auto& environment_off = captures[3];
  const auto& background_only = captures[4];
  const auto& background_lit = captures[5];
  const auto& environment_double = captures[6];
  const auto& diffuse_fallback_double = captures[7];
  const auto& specular_fallback_off = captures[8];
  const auto& specular_fallback_double = captures[9];
  const auto& fallbacks_double = captures[10];
  const auto& ray_neutral = captures[11];
  const auto& ray_environment_double = captures[12];
  const auto& ray_background_lit = captures[13];
  const auto& ray_fallbacks_off = captures[14];
  const auto& ray_fallbacks_double = captures[15];
  const auto& ray_environment_off = captures[16];
  const auto& ray_all_off = captures[17];
  const auto& sponza_default = captures[18];
  const auto& sponza_diffuse = captures[19];
  const auto& sponza_unoccluded = captures[20];
  const auto& sponza_gtao_beauty = captures[21];
  const auto& sponza_gtao_diffuse = captures[22];
  const auto& sponza_gtao_diffuse_probe_double = captures[23];
  const auto& sponza_gtao_visibility = captures[24];
  const auto& sponza_gtao_occluded = captures[25];
  const auto& sponza_ssao_diffuse = captures[26];
  const auto& sponza_ssao_occluded = captures[27];
  const auto& sponza_gtao_occluded_indirect_double = captures[28];
  const auto& sponza_gtao_occluded_ddgi_disabled = captures[29];
  const auto& sponza_gtao_occluded_ddgi_outside = captures[30];
  const auto& sponza_gtao_left_room = captures[31];
  const auto& sponza_gtao_right_room = captures[32];
  const auto& sponza_repeatability_anchor = captures[33];
  const auto& sponza_ray_reference = captures[34];
  const auto luminance = [](const CaptureEvidence& capture_evidence, const char* name) {
    return capture_evidence.regions.at(name).average_luminance;
  };
  const auto linear_luminance = [](const CaptureEvidence& capture_evidence, const char* name) {
    return capture_evidence.regions.at(name).linear_luminance;
  };
  const uint32_t reset_reasons = DdgiUpdateReasonSource | DdgiUpdateReasonManualReset;
  const auto near_black = [](const double luminance) {
    return luminance < 0.005;
  };
  const auto nearly_equal = [](const double a, const double b) {
    return std::abs(a - b) <= std::max(0.01, std::max(a, b) * 0.1);
  };
  const auto ray_nearly_equal = [](const double a, const double b) {
    return std::abs(a - b) <= std::max(0.04, std::max(a, b) * 0.2);
  };
  const auto scales_twice = [](const double off, const double once, const double twice) {
    const double unit_response = once - off;
    const double double_response = twice - off;
    return unit_response > 0.002 &&
           std::abs(double_response - 2.0 * unit_response) <= std::max(0.01, 0.25 * 2.0 * unit_response);
  };
  const auto normalized_rms = [](const std::vector<glm::vec4>& lhs, const std::vector<glm::vec4>& rhs) {
    if (lhs.size() != rhs.size() || lhs.empty()) {
      return std::numeric_limits<double>::infinity();
    }
    double squared_error = 0.0;
    double squared_reference = 0.0;
    for (size_t index = 0; index < lhs.size(); ++index) {
      const auto difference = glm::dvec3(lhs[index]) - glm::dvec3(rhs[index]);
      squared_error += glm::dot(difference, difference);
      const auto reference = glm::dvec3(lhs[index]);
      squared_reference += glm::dot(reference, reference);
    }
    return std::sqrt(squared_error / std::max(squared_reference, 1.0e-12));
  };
  const auto scalar_channel_error = [](const CaptureEvidence& value) {
    double maximum = 0.0;
    for (const auto& pixel : value.pixels) {
      maximum =
          std::max(maximum, static_cast<double>(glm::max(glm::abs(pixel.r - pixel.g), glm::abs(pixel.r - pixel.b))));
    }
    return maximum;
  };
  const double sponza_diffuse_probe_intensity_nrmse =
      normalized_rms(sponza_gtao_diffuse.pixels, sponza_gtao_diffuse_probe_double.pixels);
  const double sponza_ssao_specular_nrmse = normalized_rms(sponza_unoccluded.pixels, sponza_ssao_occluded.pixels);
  const double sponza_indirect_specular_nrmse =
      normalized_rms(sponza_gtao_occluded.pixels, sponza_gtao_occluded_indirect_double.pixels);
  const double sponza_ddgi_disabled_specular_nrmse =
      normalized_rms(sponza_gtao_occluded.pixels, sponza_gtao_occluded_ddgi_disabled.pixels);
  const double sponza_ddgi_outside_specular_nrmse =
      normalized_rms(sponza_gtao_occluded.pixels, sponza_gtao_occluded_ddgi_outside.pixels);
  const double sponza_gtao_specular_nrmse = normalized_rms(sponza_unoccluded.pixels, sponza_gtao_occluded.pixels);
  const double sponza_gtao_diffuse_nrmse = normalized_rms(sponza_diffuse.pixels, sponza_gtao_diffuse.pixels);
  const double sponza_ssao_diffuse_nrmse = normalized_rms(sponza_diffuse.pixels, sponza_ssao_diffuse.pixels);
  const double sponza_visibility_scalar_error = scalar_channel_error(sponza_gtao_visibility);
  const auto maximum_amplification = [](const CaptureEvidence& unoccluded, const CaptureEvidence& occluded) {
    double maximum = 0.0;
    for (size_t index = 0; index < unoccluded.pixels.size(); ++index) {
      const auto difference = glm::vec3(occluded.pixels[index] - unoccluded.pixels[index]);
      maximum = std::max(maximum, static_cast<double>(std::max({difference.r, difference.g, difference.b})));
    }
    return maximum;
  };
  const double sponza_maximum_amplification = maximum_amplification(sponza_unoccluded, sponza_gtao_occluded);
  const auto raster_region_invariant = [&](const char* name) {
    return std::all_of(captures.begin() + 1, captures.begin() + 11, [&](const auto& capture_evidence) {
      return nearly_equal(luminance(defaults, name), luminance(capture_evidence, name));
    });
  };
  const std::vector<std::pair<std::string, bool>> checks{
      {"captures_finite", std::all_of(captures.begin(), captures.end(),
                                      [&](const auto& value) {
                                        return value.finite && value.regions.size() == regions.size() &&
                                               std::all_of(value.regions.begin(), value.regions.end(),
                                                           [](const auto& region) {
                                                             return region.second.sample_count > 0u;
                                                           });
                                      })},
      {"fixed_post_processing", fixed_post_processing},
      {"render_mode_matrix", std::all_of(captures.begin(), captures.begin() + 11,
                                         [](const auto& value) {
                                           return value.render_mode == "Rasterization" && value.settle_frames == 4u;
                                         }) &&
                                 std::all_of(captures.begin() + 11, captures.begin() + 18,
                                             [](const auto& value) {
                                               return value.render_mode == "RayTracing" && value.settle_frames == 48u;
                                             }) &&
                                 sponza_default.render_mode == "Rasterization" && sponza_default.settle_frames == 4u &&
                                 std::all_of(captures.begin() + 19, captures.begin() + 33,
                                             [](const auto& value) {
                                               return value.render_mode == "Rasterization" && value.settle_frames == 4u;
                                             }) &&
                                 sponza_repeatability_anchor.render_mode == "Rasterization" &&
                                 sponza_repeatability_anchor.settle_frames == 0u &&
                                 sponza_ray_reference.render_mode == "RayTracing" &&
                                 sponza_ray_reference.settle_frames == 64u},
      {"background_is_camera_only",
       luminance(background_only, "background") > luminance(all_off, "background") + 0.01 &&
           nearly_equal(luminance(background_only, "background"), luminance(background_lit, "background")) &&
           nearly_equal(luminance(background_only, "dielectric"), luminance(all_off, "dielectric")) &&
           nearly_equal(luminance(background_only, "metal"), luminance(all_off, "metal")) &&
           luminance(ray_background_lit, "background") > luminance(ray_neutral, "background") + 0.01 &&
           ray_nearly_equal(luminance(ray_background_lit, "dielectric"), luminance(ray_neutral, "dielectric")) &&
           ray_nearly_equal(luminance(ray_background_lit, "metal"), luminance(ray_neutral, "metal")) &&
           ray_nearly_equal(luminance(ray_background_lit, "ibl_diffuse"), luminance(ray_neutral, "ibl_diffuse")) &&
           std::all_of(captures.begin(), captures.begin() + 18,
                       [&](const auto& value) {
                         return value.name == "background-only" || value.name == "background-lit" ||
                                value.name == "ray-background-lit" || near_black(luminance(value, "background"));
                       })},
      {"environment_intensity_owns_ddgi_source",
       luminance(defaults, "dielectric") > luminance(environment_off, "dielectric") + 0.005 &&
           luminance(environment_double, "dielectric") > luminance(defaults, "dielectric") + 0.005 &&
           nearly_equal(luminance(environment_off, "ibl_diffuse"), luminance(defaults, "ibl_diffuse")) &&
           nearly_equal(luminance(defaults, "ibl_diffuse"), luminance(environment_double, "ibl_diffuse")) &&
           nearly_equal(luminance(environment_off, "metal"), luminance(defaults, "metal")) &&
           nearly_equal(luminance(defaults, "metal"), luminance(environment_double, "metal"))},
      {"diffuse_fallback_owns_diffuse_result",
       luminance(defaults, "ibl_diffuse") > luminance(diffuse_fallback_off, "ibl_diffuse") + 0.005 &&
           luminance(diffuse_fallback_double, "ibl_diffuse") > luminance(defaults, "ibl_diffuse") + 0.005 &&
           nearly_equal(luminance(diffuse_fallback_off, "metal"), luminance(defaults, "metal")) &&
           nearly_equal(luminance(defaults, "metal"), luminance(diffuse_fallback_double, "metal"))},
      {"specular_fallback_owns_global_specular",
       luminance(defaults, "metal") > luminance(specular_fallback_off, "metal") + 0.005 &&
           luminance(specular_fallback_double, "metal") > luminance(defaults, "metal") + 0.005 &&
           nearly_equal(luminance(specular_fallback_off, "ibl_diffuse"), luminance(defaults, "ibl_diffuse")) &&
           nearly_equal(luminance(defaults, "ibl_diffuse"), luminance(specular_fallback_double, "ibl_diffuse"))},
      {"diffuse_fallback_is_applied_once",
       scales_twice(linear_luminance(diffuse_fallback_off, "ibl_diffuse"), linear_luminance(defaults, "ibl_diffuse"),
                    linear_luminance(diffuse_fallback_double, "ibl_diffuse"))},
      {"specular_fallback_is_applied_once",
       scales_twice(linear_luminance(specular_fallback_off, "metal"), linear_luminance(defaults, "metal"),
                    linear_luminance(specular_fallback_double, "metal"))},
      {"ray_environment_intensity_is_applied_once",
       scales_twice(linear_luminance(ray_environment_off, "metal"), linear_luminance(ray_neutral, "metal"),
                    linear_luminance(ray_environment_double, "metal")) &&
           scales_twice(linear_luminance(ray_environment_off, "ibl_diffuse"),
                        linear_luminance(ray_neutral, "ibl_diffuse"),
                        linear_luminance(ray_environment_double, "ibl_diffuse"))},
      {"ray_fallbacks_are_ignored",
       ray_nearly_equal(luminance(ray_fallbacks_off, "dielectric"), luminance(ray_neutral, "dielectric")) &&
           ray_nearly_equal(luminance(ray_fallbacks_double, "dielectric"), luminance(ray_neutral, "dielectric")) &&
           ray_nearly_equal(luminance(ray_fallbacks_off, "metal"), luminance(ray_neutral, "metal")) &&
           ray_nearly_equal(luminance(ray_fallbacks_double, "metal"), luminance(ray_neutral, "metal")) &&
           ray_nearly_equal(luminance(ray_fallbacks_off, "ibl_diffuse"), luminance(ray_neutral, "ibl_diffuse")) &&
           ray_nearly_equal(luminance(ray_fallbacks_double, "ibl_diffuse"), luminance(ray_neutral, "ibl_diffuse"))},
      {"probe_specular_is_diffuse_fallback_invariant",
       nearly_equal(luminance(diffuse_fallback_off, "metal"), luminance(defaults, "metal")) &&
           nearly_equal(luminance(defaults, "metal"), luminance(diffuse_fallback_double, "metal"))},
      {"direct_and_emission_are_control_invariant",
       luminance(all_off, "direct") > 0.01 && luminance(defaults, "emission") > 0.01 &&
           nearly_equal(luminance(all_off, "direct"), luminance(environment_off, "direct")) &&
           nearly_equal(luminance(all_off, "direct"), luminance(diffuse_fallback_off, "direct")) &&
           raster_region_invariant("emission")},
      {"ray_camera_ownership_matches_raster",
       luminance(ray_all_off, "direct") > 0.01 && luminance(ray_neutral, "emission") > 0.01 &&
           luminance(ray_neutral, "dielectric") > luminance(ray_environment_off, "dielectric") + 0.005 &&
           luminance(ray_neutral, "metal") > luminance(ray_environment_off, "metal") + 0.005 &&
           ray_nearly_equal(luminance(ray_all_off, "direct"), luminance(ray_fallbacks_off, "direct")) &&
           ray_nearly_equal(luminance(ray_all_off, "direct"), luminance(ray_fallbacks_double, "direct")) &&
           ray_nearly_equal(luminance(ray_all_off, "direct"), luminance(ray_environment_double, "direct")) &&
           ray_nearly_equal(luminance(ray_all_off, "direct"), luminance(ray_environment_off, "direct")) &&
           ray_nearly_equal(luminance(ray_neutral, "emission"), luminance(ray_fallbacks_off, "emission")) &&
           ray_nearly_equal(luminance(ray_neutral, "emission"), luminance(ray_fallbacks_double, "emission")) &&
           ray_nearly_equal(luminance(ray_neutral, "emission"), luminance(ray_environment_double, "emission")) &&
           ray_nearly_equal(luminance(ray_neutral, "emission"), luminance(ray_environment_off, "emission")) &&
           ray_nearly_equal(luminance(ray_neutral, "emission"), luminance(ray_all_off, "emission"))},
      {"sponza_exact_metal_is_lit", luminance(sponza_default, "sponza_exact_metal") > 0.01},
      {"sponza_persistent_probe_payloads_are_valid",
       std::all_of(captures.begin() + 18, captures.begin() + 33,
                   [](const auto& value) {
                     return value.local_probe_count == kSponzaLocalProbeNames.size() &&
                            value.valid_local_probe_count == kSponzaLocalProbeNames.size();
                   })},
      {"sponza_rough_specular_debug_contract",
       sponza_diffuse.indirect_debug_view ==
               static_cast<int>(RenderSettings::IndirectLightingDebugView::DiffuseIndirect) &&
           sponza_unoccluded.indirect_debug_view ==
               static_cast<int>(RenderSettings::IndirectLightingDebugView::UnoccludedProbeSpecular) &&
           sponza_gtao_beauty.indirect_debug_view ==
               static_cast<int>(RenderSettings::IndirectLightingDebugView::Beauty) &&
           sponza_gtao_visibility.indirect_debug_view ==
               static_cast<int>(RenderSettings::IndirectLightingDebugView::SpecularVisibility) &&
           sponza_gtao_occluded.indirect_debug_view ==
               static_cast<int>(RenderSettings::IndirectLightingDebugView::OccludedProbeSpecular) &&
           sponza_diffuse.ambient_occlusion == "disabled" && sponza_unoccluded.ambient_occlusion == "disabled" &&
           sponza_gtao_occluded.ambient_occlusion == "gtao" && sponza_ssao_occluded.ambient_occlusion == "ssao"},
      {"sponza_visibility_is_scalar", sponza_visibility_scalar_error < 1.0e-6},
      {"sponza_visibility_does_not_amplify", sponza_maximum_amplification < 1.0e-5},
      {"sponza_gtao_affects_diffuse_and_rough_specular",
       sponza_gtao_diffuse_nrmse > 1.0e-6 && sponza_gtao_specular_nrmse > 1.0e-6 &&
           luminance(sponza_gtao_occluded, "sponza_rough_metal") < luminance(sponza_unoccluded, "sponza_rough_metal") &&
           luminance(sponza_gtao_occluded, "sponza_rough_dielectric") <
               luminance(sponza_unoccluded, "sponza_rough_dielectric")},
      {"sponza_smooth_specular_is_retained", luminance(sponza_gtao_occluded, "sponza_exact_metal") >
                                                     luminance(sponza_unoccluded, "sponza_exact_metal") * 0.8 &&
                                                 luminance(sponza_gtao_occluded, "sponza_smooth_dielectric") >
                                                     luminance(sponza_unoccluded, "sponza_smooth_dielectric") * 0.8},
      {"sponza_ssao_retains_material_and_ddgi_specular_visibility",
       sponza_ssao_diffuse_nrmse > 1.0e-6 && sponza_ssao_specular_nrmse > 1.0e-6},
      {"sponza_probe_intensity_is_diffuse_invariant",
       sponza_diffuse_probe_intensity_nrmse < 1.0e-6 && sponza_gtao_diffuse_probe_double.local_probe_intensity == 2.0f},
      {"sponza_probe_specular_is_indirect_invariant", sponza_indirect_specular_nrmse < 1.0e-6},
      {"sponza_probe_specular_uses_ddgi_visibility",
       sponza_ddgi_disabled_specular_nrmse > 1.0e-6 && sponza_ddgi_outside_specular_nrmse < 1.0e-6},
      {"sponza_adjacent_rooms_are_nonblack", luminance(sponza_gtao_left_room, "sponza_room") > 0.0001 &&
                                                 luminance(sponza_gtao_right_room, "sponza_room") > 0.0001},
      {"canonical_sponza_reference_contract", canonical_sponza_anchor_contract && canonical_sponza_ray_contract &&
                                                  sponza_convergence_frames > 0u &&
                                                  sponza_convergence_frames <= max_sponza_convergence_frames},
      {"indirect_edit_keeps_ddgi_history", (indirect_update_reasons & reset_reasons) == 0u &&
                                               indirect_update_stats.recorded_probe_update_count == 0u &&
                                               indirect_update_stats.recorded_ray_sample_count == 0u},
      {"sky_edit_activates_hysteresis_boost",
       (sky_update_reasons & DdgiUpdateReasonSceneChange) != 0u &&
           (sky_update_reasons & DdgiUpdateReasonWarmup) == 0u && !sky_update_snapshot.last_probe_history_cleared &&
           sky_update_stats.recorded_probe_update_count > 0u && sky_update_stats.recorded_ray_sample_count > 0u}};

  const auto report_path = output_directory / "report.json";
  std::ofstream report(report_path, std::ios::trunc);
  if (!report) {
    throw std::runtime_error("Failed to open environment lighting report: " + report_path.string());
  }
  report << std::setprecision(17);
  report << "{\n  \"schema_version\": 4,\n  \"contract\": {\"resolution\": [1920, 1080], "
            "\"render_modes\": [\"Rasterization\", \"RayTracing\"], \"graphics_validation\": true, "
            "\"launch_count\": 1, \"raster_settle_frames\": 4, \"ray_settle_frames\": 48, "
            "\"screen_space_reflections\": false, \"ray_traced_reflections\": false, "
            "\"metallic_values\": [0.0, 1.0], \"physical_reference\": \"ray-neutral-reference\", "
            "\"specular_visibility_model\": "
            "\"1-roughness^2*mix(0.04*tanh((1-min(material_ao,gtao))/0.04),1-min(material_ao,gtao),"
            "smoothstep(0.8,1,NdotV))\", "
            "\"sponza_local_probe_count\": 5, "
            "\"canonical_sponza\": {\"fixture_id\": \"sponza\", \"deterministic_seed\": 1831565813, "
            "\"spheres_enabled\": false, \"title_enabled\": false, "
            "\"raster\": {\"render_mode\": \"Rasterization\", \"ddgi_enabled\": true, \"samples_per_frame\": 1, "
            "\"warmup_frames\": 64, \"max_convergence_frames\": 1024, \"preparation_frames\": 8, "
            "\"measure_frames\": 120, \"image\": \"sponza-repeatability-anchor.png\"}, "
            "\"ray\": {\"render_mode\": \"RayTracing\", \"ddgi_enabled\": false, \"frames\": 64, "
            "\"samples_per_frame\": 4, \"total_spp\": 256, \"debug_view\": \"Beauty\", \"ser\": \"disabled\", "
            "\"firefly_clamp_threshold\": 10.0, \"auto_spp\": false, "
            "\"image\": \"sponza-ray-reference.png\"}}},\n"
            "  \"captures\": [";
  const auto write_region = [&](const RegionEvidence& region) {
    report << "{\"average_color\": [" << region.average_color.x << ", " << region.average_color.y << ", "
           << region.average_color.z << "], \"average_luminance\": " << region.average_luminance
           << ", \"linear_luminance\": " << region.linear_luminance << ", \"sample_count\": " << region.sample_count
           << "}";
  };
  for (size_t index = 0; index < captures.size(); ++index) {
    const auto& value = captures[index];
    report << (index == 0u ? "" : ",") << "\n    {\"name\": \"" << value.name << "\", \"render_mode\": \""
           << value.render_mode << "\", \"settle_frames\": " << value.settle_frames
           << ", \"controls\": {\"background\": " << value.background_intensity
           << ", \"environment\": " << value.environment_intensity
           << ", \"diffuse_fallback\": " << value.diffuse_fallback_intensity
           << ", \"specular_fallback\": " << value.specular_fallback_intensity << "}, \"ambient_occlusion\": \""
           << value.ambient_occlusion << "\", \"indirect_debug_view\": " << value.indirect_debug_view
           << ", \"ddgi_enabled\": " << (value.ddgi_enabled ? "true" : "false")
           << ", \"local_probe_count\": " << value.local_probe_count
           << ", \"valid_local_probe_count\": " << value.valid_local_probe_count
           << ", \"local_probe_intensity\": " << value.local_probe_intensity << ", \"image\": \"" << value.name
           << ".png\", \"finite\": " << (value.finite ? "true" : "false") << ", \"regions\": {";
    size_t region_index = 0;
    for (const auto& [region_name, region] : value.regions) {
      report << (region_index++ == 0u ? "" : ",") << "\"" << region_name << "\": ";
      write_region(region);
    }
    report << "}}";
  }
  report << "\n  ],\n  \"rough_specular\": {\"gtao_specular_nrmse\": " << sponza_gtao_specular_nrmse
         << ", \"gtao_diffuse_nrmse\": " << sponza_gtao_diffuse_nrmse
         << ", \"ssao_diffuse_nrmse\": " << sponza_ssao_diffuse_nrmse
         << ", \"ssao_specular_nrmse\": " << sponza_ssao_specular_nrmse
         << ", \"probe_intensity_diffuse_nrmse\": " << sponza_diffuse_probe_intensity_nrmse
         << ", \"indirect_intensity_specular_nrmse\": " << sponza_indirect_specular_nrmse
         << ", \"ddgi_disabled_specular_nrmse\": " << sponza_ddgi_disabled_specular_nrmse
         << ", \"ddgi_outside_specular_nrmse\": " << sponza_ddgi_outside_specular_nrmse
         << ", \"scalar_channel_error\": " << sponza_visibility_scalar_error
         << ", \"maximum_amplification\": " << sponza_maximum_amplification
         << "},\n  \"ddgi_invalidation\": "
            "{\"indirect_edit\": {\"update_reasons\": "
         << indirect_update_reasons
         << ", \"recorded_probe_updates\": " << indirect_update_stats.recorded_probe_update_count
         << ", \"recorded_ray_samples\": " << indirect_update_stats.recorded_ray_sample_count
         << "}, \"sky_edit\": {\"update_reasons\": " << sky_update_reasons
         << ", \"recorded_probe_updates\": " << sky_update_stats.recorded_probe_update_count
         << ", \"recorded_ray_samples\": " << sky_update_stats.recorded_ray_sample_count
         << "}, \"convergence_frames\": " << convergence_frames
         << ", \"m15_sponza_convergence_frames\": " << m15_sponza_convergence_frames
         << ", \"canonical_sponza_convergence_frames\": " << sponza_convergence_frames << "},\n  \"checks\": {";
  bool all_checks_passed = true;
  for (size_t index = 0; index < checks.size(); ++index) {
    report << (index == 0u ? "" : ",") << "\n    \"" << checks[index].first
           << "\": " << (checks[index].second ? "true" : "false");
    all_checks_passed &= checks[index].second;
  }
  report << "\n  },\n  \"passed\": " << (all_checks_passed ? "true" : "false") << "\n}\n";
  report.close();

  std::cout << "EVOENGINE_ENVIRONMENT_LIGHTING_REPORT path=\"" << report_path.string()
            << "\" passed=" << (all_checks_passed ? "true" : "false")
            << " indirect_update_reasons=" << indirect_update_reasons << " sky_update_reasons=" << sky_update_reasons
            << std::endl;
  if (!all_checks_passed) {
    throw std::runtime_error("Environment lighting validation report contains failed checks.");
  }
  editor_layer->SetSceneCameraResolutionOverride(std::nullopt);
  return true;
}

bool evo_engine::RunDdgiEmissiveValidationFromEnvironment(const int width, const int height) {
  const auto* evidence_path = std::getenv("EVOENGINE_DDGI_EMISSIVE_EVIDENCE");
  if (!evidence_path) {
    return false;
  }
  if (width != 1920 || height != 1080) {
    throw std::runtime_error("DDGI emissive validation requires 1920x1080.");
  }
  if (!Platform::RayTracingEnabled()) {
    throw std::runtime_error("DDGI emissive validation requires the Vulkan ray-tracing pipeline.");
  }
  if (!Platform::GraphicsValidationEnabled()) {
    throw std::runtime_error("DDGI emissive validation requires Vulkan validation.");
  }
  Platform::SetGpuTimestampCaptureEnabled(true);
  if (!Platform::GpuTimestampCaptureAvailable() || !Platform::GpuTimestampCaptureEnabled()) {
    throw std::runtime_error("DDGI emissive validation requires GPU timestamp capture.");
  }

  const auto output_directory = std::filesystem::path(evidence_path);
  if (output_directory.empty()) {
    throw std::runtime_error("DDGI emissive validation evidence path is empty.");
  }
  std::filesystem::create_directories(output_directory);

  const auto scene = ApplicationContext::Get().GetActiveScene();
  const auto editor_layer = ApplicationContext::Get().GetLayer<EditorLayer>();
  const auto render_layer = ApplicationContext::Get().GetLayer<RenderLayer>();
  if (!scene || !editor_layer || !render_layer) {
    throw std::runtime_error("DDGI emissive validation requires an active scene, EditorLayer, and RenderLayer.");
  }
  ConfigureDdgiValidationFixture(scene, "emissive-alpha-cutout");
  auto& settings = RequireEnvironmentalLightingDdgiSettings(scene);
  settings.runtime.enabled = true;
  SetDdgiUpdatesPaused(false);
  settings.runtime.deterministic_ray_seed_enabled = true;
  settings.runtime.deterministic_ray_seed = 0x6d2b79f5u;
  DisableDdgiDebugVisualization();
  if (const auto lighting = scene->environmental_lighting.Get<EnvironmentalLighting>()) {
    lighting->ddgi_settings = settings;
    for (auto& volume : lighting->GetOrCreateDdgiVolumePack()->volumes) {
      volume.emissive_mesh_sampling_mode = static_cast<int>(DdgiEmissiveMeshSamplingMode::Inherit);
    }
  }
  SyncTemporaryEnvironmentalLightingSettingsFromScene(scene);

  if (const auto window_layer = ApplicationContext::Get().GetLayer<WindowLayer>()) {
    window_layer->ResizeWindow(width, height);
    window_layer->CenterWindow();
  }
  const glm::uvec2 resolution(static_cast<uint32_t>(width), static_cast<uint32_t>(height));
  editor_layer->show_camera_window = false;
  editor_layer->RequestSceneCameraPreviewWindow(resolution);
  editor_layer->SetSceneCameraResolutionOverride(resolution);
  const auto scene_camera = editor_layer->GetSceneCamera();
  if (!scene_camera) {
    throw std::runtime_error("DDGI emissive validation requires a scene camera.");
  }
  scene_camera->camera_render_mode = Camera::CameraRenderMode::Rasterization;
  scene_camera->SetRequireRendering(true);
  scene_camera->Resize(resolution);
  scene_camera->ResetFrameCount();
  if (const auto main_camera = scene->main_camera.Get<Camera>(); main_camera && main_camera != scene_camera) {
    main_camera->SetEnabled(false);
  }

  constexpr size_t max_wait_frames = 30000;
  constexpr size_t settled_frame_count = 4;
  size_t stable_input_frames = 0;
  for (size_t frame = 0; frame < max_wait_frames && stable_input_frames < settled_frame_count; ++frame) {
    const bool ready = ProjectManager::IsProjectIdle() && !AssetManager::GetAssetLoadSnapshot().Active() &&
                       !TextureStorage::HasPendingUploads() && !GeometryStorage::HasPendingUploads() &&
                       !BottomLevelAccelerationStructure::HasPendingStaticBuilds();
    stable_input_frames = ready ? stable_input_frames + 1u : 0u;
    if (stable_input_frames < settled_frame_count && !ApplicationContext::Get().Loop()) {
      throw std::runtime_error("Application ended before DDGI emissive scene inputs were ready.");
    }
  }
  if (stable_input_frames < settled_frame_count) {
    throw std::runtime_error("DDGI emissive scene input readiness timed out.");
  }

  struct PhaseEvidence {
    std::string name;
    bool requested_enabled = false;
    bool effective_enabled = false;
    size_t convergence_frames = 0;
    float variability = 0.0f;
    uint32_t variability_samples = 0;
    uint32_t update_reasons = DdgiUpdateReasonNone;
    uint32_t emissive_triangle_count = 0;
    uint32_t enabled_volume_count = 0;
    uint64_t candidate_ray_count = 0;
    uint64_t probe_trace_sample_count = 0;
    double probe_trace_median_ms = 0.0;
    double probe_trace_p95_ms = 0.0;
    std::vector<double> probe_trace_samples_ms;
    double luminance_sum = 0.0;
  };

  const auto save_capture = [&](const std::filesystem::path& path) {
    const auto pixels =
        ReadAndStoreValidationCapture(scene_camera->GetRenderTexture(), resolution, path, "DDGI emissive");
    double luminance_sum = 0.0;
    for (const auto& pixel : pixels) {
      if (!std::isfinite(pixel.x) || !std::isfinite(pixel.y) || !std::isfinite(pixel.z) || !std::isfinite(pixel.w)) {
        throw std::runtime_error("DDGI emissive capture contains non-finite pixels.");
      }
      luminance_sum +=
          0.2126 * std::max(0.0f, pixel.x) + 0.7152 * std::max(0.0f, pixel.y) + 0.0722 * std::max(0.0f, pixel.z);
    }
    if (!(luminance_sum > 1e-6)) {
      throw std::runtime_error("DDGI emissive capture is black.");
    }
    return luminance_sum;
  };

  constexpr size_t max_convergence_frames = 1024;
  constexpr size_t timing_prepare_frames = 8;
  constexpr size_t measure_frames = 120;
  const auto run_phase = [&](const std::string& name, const bool enabled) {
    PhaseEvidence phase;
    phase.name = name;
    phase.requested_enabled = enabled;
    settings.runtime.enable_emissive_mesh_sampling = enabled;
    RequestDdgiHistoryReset();
    settings.volume_defaults.enable_probe_variability = true;
    settings.volume_defaults.enable_probe_variability_gating = true;
    if (const auto lighting = scene->environmental_lighting.Get<EnvironmentalLighting>()) {
      lighting->ddgi_settings = settings;
      for (auto& volume : lighting->GetOrCreateDdgiVolumePack()->volumes) {
        volume.emissive_mesh_sampling_mode = static_cast<int>(DdgiEmissiveMeshSamplingMode::Inherit);
        volume.enable_probe_variability = true;
        volume.enable_probe_variability_gating = true;
      }
    }
    SyncTemporaryEnvironmentalLightingSettingsFromScene(scene);

    bool converged = false;
    for (; phase.convergence_frames < max_convergence_frames; ++phase.convergence_frames) {
      if (!ApplicationContext::Get().Loop()) {
        throw std::runtime_error("Application ended during DDGI emissive convergence.");
      }
      phase.update_reasons |= render_layer->GetDdgiInspectorSnapshot().last_probe_update_reasons;
      const auto runtime = render_layer->GetDdgiInspectorSnapshot().volumes;
      const auto& performance = render_layer->GetDdgiInspectorSnapshot().aggregate;
      if (runtime.size() == 1u && runtime.front().emissive_mesh_sampling_enabled == enabled &&
          runtime.front().has_valid_probe_history && runtime.front().contributes_lighting &&
          performance.probe_variability_converged) {
        ++phase.convergence_frames;
        converged = true;
        break;
      }
    }
    if (!converged) {
      throw std::runtime_error("DDGI emissive phase did not converge: " + name);
    }

    settings.volume_defaults.enable_probe_variability_gating = false;
    if (const auto lighting = scene->environmental_lighting.Get<EnvironmentalLighting>()) {
      lighting->ddgi_settings = settings;
      for (auto& volume : lighting->GetOrCreateDdgiVolumePack()->volumes) {
        volume.enable_probe_variability_gating = false;
      }
    }
    for (size_t frame = 0; frame < timing_prepare_frames; ++frame) {
      if (!ApplicationContext::Get().Loop()) {
        throw std::runtime_error("Application ended during DDGI emissive timing preparation.");
      }
      phase.update_reasons |= render_layer->GetDdgiInspectorSnapshot().last_probe_update_reasons;
    }
    Platform::WaitForFrameSubmissions("DDGI Emissive Timing Warmup Fence Wait");
    Platform::ResetGpuTimestampStats();
    for (size_t frame = 0; frame < measure_frames; ++frame) {
      if (!ApplicationContext::Get().Loop()) {
        throw std::runtime_error("Application ended during DDGI emissive timing measurement.");
      }
      phase.update_reasons |= render_layer->GetDdgiInspectorSnapshot().last_probe_update_reasons;
    }
    Platform::WaitForFrameSubmissions("DDGI Emissive Timing Completion Fence Wait");

    const auto runtime = render_layer->GetDdgiInspectorSnapshot().volumes;
    if (runtime.size() != 1u) {
      throw std::runtime_error("DDGI emissive validation expected exactly one runtime volume.");
    }
    const auto& performance = render_layer->GetDdgiInspectorSnapshot().aggregate;
    phase.effective_enabled = runtime.front().emissive_mesh_sampling_enabled;
    phase.variability = performance.probe_variability_average;
    phase.variability_samples = performance.probe_variability_sample_count;
    phase.emissive_triangle_count = performance.emissive_triangle_count;
    phase.enabled_volume_count = performance.emissive_sampling_enabled_volume_count;
    phase.candidate_ray_count = performance.emissive_sampling_candidate_ray_count;
    const auto gpu_timestamps = Platform::GetGpuTimestampStats();
    const auto probe_trace = std::find_if(gpu_timestamps.begin(), gpu_timestamps.end(), [](const auto& timestamp) {
      return timestamp.name == "DDGI Probe Trace";
    });
    if (probe_trace == gpu_timestamps.end()) {
      throw std::runtime_error("DDGI emissive validation did not record DDGI Probe Trace timing.");
    }
    phase.probe_trace_sample_count = probe_trace->sample_count;
    phase.probe_trace_median_ms = probe_trace->MedianMilliseconds();
    phase.probe_trace_p95_ms = probe_trace->PercentileMilliseconds(0.95);
    phase.probe_trace_samples_ms = probe_trace->samples_milliseconds;
    phase.luminance_sum = save_capture(output_directory / (name + ".png"));
    return phase;
  };

  const std::array<PhaseEvidence, 3> phases{run_phase("enabled", true), run_phase("disabled", false),
                                            run_phase("enabled-repeat", true)};
  const auto phase_valid = [&](const PhaseEvidence& phase) {
    const uint32_t expected_enabled_volumes = phase.requested_enabled ? 1u : 0u;
    return phase.effective_enabled == phase.requested_enabled && phase.emissive_triangle_count > 0u &&
           phase.enabled_volume_count == expected_enabled_volumes &&
           (phase.requested_enabled ? phase.candidate_ray_count > 0u : phase.candidate_ray_count == 0u) &&
           phase.probe_trace_sample_count == measure_frames && phase.probe_trace_samples_ms.size() == measure_frames &&
           phase.luminance_sum > 1e-6;
  };
  const std::vector<std::pair<std::string, bool>> checks{
      {"enabled_phase", phase_valid(phases[0])},
      {"disabled_phase", phase_valid(phases[1])},
      {"enabled_repeat_phase", phase_valid(phases[2])},
      {"source_reset_on_disable", (phases[1].update_reasons & DdgiUpdateReasonSource) != 0u},
      {"source_reset_on_reenable", (phases[2].update_reasons & DdgiUpdateReasonSource) != 0u}};

  const auto report_path = output_directory / "report.json";
  std::ofstream report(report_path, std::ios::trunc);
  if (!report) {
    throw std::runtime_error("Failed to open DDGI emissive report: " + report_path.string());
  }
  report << std::setprecision(17);
  report << "{\n  \"schema_version\": 1,\n  \"contract\": {\"resolution\": [1920, 1080], "
            "\"vulkan_rt_pipeline\": true, \"graphics_validation\": true, \"launch_count\": 1, "
            "\"measure_frames\": 120, \"deterministic_seed\": 1831565813},\n  \"phases\": [";
  for (size_t phase_index = 0; phase_index < phases.size(); ++phase_index) {
    const auto& phase = phases[phase_index];
    report << (phase_index == 0u ? "" : ",") << "\n    {\"name\": \"" << phase.name
           << "\", \"requested_enabled\": " << (phase.requested_enabled ? "true" : "false")
           << ", \"effective_enabled\": " << (phase.effective_enabled ? "true" : "false")
           << ", \"convergence_frames\": " << phase.convergence_frames << ", \"variability\": " << phase.variability
           << ", \"variability_samples\": " << phase.variability_samples
           << ", \"update_reasons\": " << phase.update_reasons
           << ", \"emissive_triangle_count\": " << phase.emissive_triangle_count
           << ", \"enabled_volume_count\": " << phase.enabled_volume_count
           << ", \"candidate_ray_count\": " << phase.candidate_ray_count
           << ", \"probe_trace\": {\"sample_count\": " << phase.probe_trace_sample_count
           << ", \"median_ms\": " << phase.probe_trace_median_ms << ", \"p95_ms\": " << phase.probe_trace_p95_ms
           << ", \"samples_ms\": [";
    for (size_t sample_index = 0; sample_index < phase.probe_trace_samples_ms.size(); ++sample_index) {
      report << (sample_index == 0u ? "" : ", ") << phase.probe_trace_samples_ms[sample_index];
    }
    report << "]}, \"capture\": \"" << phase.name << ".png\", \"luminance_sum\": " << phase.luminance_sum << "}";
  }
  report << "\n  ],\n  \"checks\": {";
  bool all_checks_passed = true;
  for (size_t i = 0; i < checks.size(); ++i) {
    report << (i == 0u ? "" : ",") << "\n    \"" << checks[i].first << "\": " << (checks[i].second ? "true" : "false");
    all_checks_passed &= checks[i].second;
  }
  report << "\n  },\n  \"passed\": " << (all_checks_passed ? "true" : "false") << "\n}\n";
  report.close();

  std::cout << "EVOENGINE_DDGI_EMISSIVE_REPORT path=\"" << report_path.string()
            << "\" passed=" << (all_checks_passed ? "true" : "false")
            << " enabled_luminance=" << phases[0].luminance_sum << " disabled_luminance=" << phases[1].luminance_sum
            << std::endl;
  if (!all_checks_passed) {
    throw std::runtime_error("DDGI emissive validation report contains failed checks.");
  }
  editor_layer->SetSceneCameraResolutionOverride(std::nullopt);
  return true;
}

bool evo_engine::RunDdgiMultiVolumeValidationFromEnvironment(const int width, const int height) {
  const auto* evidence_path = std::getenv("EVOENGINE_DDGI_MULTI_VOLUME_EVIDENCE");
  if (!evidence_path) {
    return false;
  }
  if (width != 1920 || height != 1080) {
    throw std::runtime_error("DDGI multi-volume validation requires 1920x1080.");
  }
  if (!Platform::RayTracingEnabled()) {
    throw std::runtime_error("DDGI multi-volume validation requires the Vulkan ray-tracing pipeline.");
  }
  if (!Platform::GraphicsValidationEnabled()) {
    throw std::runtime_error("DDGI multi-volume validation requires Vulkan validation.");
  }

  const auto output_directory = std::filesystem::path(evidence_path);
  if (output_directory.empty()) {
    throw std::runtime_error("DDGI multi-volume validation evidence path is empty.");
  }
  std::filesystem::create_directories(output_directory);

  const auto scene = ApplicationContext::Get().GetActiveScene();
  const auto editor_layer = ApplicationContext::Get().GetLayer<EditorLayer>();
  const auto render_layer = ApplicationContext::Get().GetLayer<RenderLayer>();
  if (!scene || !editor_layer || !render_layer) {
    throw std::runtime_error("DDGI multi-volume validation requires an active scene, EditorLayer, and RenderLayer.");
  }

  const auto lighting = GetOrCreateTemporaryEnvironmentalLighting(scene);
  if (!lighting) {
    throw std::runtime_error("DDGI multi-volume validation requires an EnvironmentalLighting asset.");
  }
  lighting->GetOrCreateReflectionProbePack()->probes.clear();
  lighting->GetOrCreateDdgiVolumePack()->volumes.clear();

  constexpr std::array<const char*, 8> kMultiVolumeNames = {
      "M7 Overlap Volume",   "M7 Disjoint Scrolling Volume", "M7 Coarse Nested Volume", "M7 Dense Nested Volume",
      "M7 Slot Four Volume", "M7 Slot Five Volume",          "M7 Slot Six Volume",      "M7 Slot Seven Volume"};
  for (const auto* name : kMultiVolumeNames) {
    if (const auto existing_volume = FindEntityNamed(scene, name)) {
      scene->DeleteEntity(*existing_volume);
    }
  }

  auto& settings = lighting->ddgi_settings;
  settings.runtime.enabled = true;
  SetDdgiUpdatesPaused(false);
  RequestDdgiHistoryReset();
  settings.runtime.ray_count = 32;
  settings.runtime.warmup_frames = 4;
  render_layer->render_settings.ddgi_hysteresis = 0.95f;
  settings.runtime.deterministic_ray_seed_enabled = true;
  settings.runtime.deterministic_ray_seed = 0x4d37564fu;
  settings.storage.max_probe_count = static_cast<int>(DdgiRuntime::kMaxResidentProbeCount);
  DisableDdgiDebugVisualization();
  if (const auto render_layer = ApplicationContext::Get().GetLayer<RenderLayer>()) {
    render_layer->RequestDdgiGatherTimingCapture();
  }
  Platform::SetGpuTimestampCaptureEnabled(true);

  const auto create_volume = [&](const char* name, const glm::ivec3 probe_counts, const float probe_spacing,
                                 const glm::vec3 volume_origin, const int artist_priority,
                                 const DdgiVolumeMovementType movement_type) -> EnvironmentalLighting::DdgiVolume& {
    auto& volume = AddEnvironmentalLightingDdgiVolume(*lighting, name, glm::mat4(1.0f), probe_counts,
                                                      glm::vec3(probe_spacing), volume_origin, artist_priority);
    volume.movement_type = static_cast<int>(movement_type);
    volume.enable_probe_relocation = true;
    volume.enable_probe_classification = true;
    volume.enable_probe_variability = false;
    volume.enable_probe_variability_gating = false;
    return volume;
  };

  // Deliberately create the volumes in an order different from their runtime sort order.
  const auto overlap_id =
      create_volume("M7 Overlap Volume", {5, 5, 5}, 0.5f, {1.75f, 0.0f, -1.5f}, 10, DdgiVolumeMovementType::Default)
          .stable_id;
  const auto scrolling_id = create_volume("M7 Disjoint Scrolling Volume", {5, 5, 5}, 0.5f, {-4.0f, 0.0f, -1.5f}, 10,
                                          DdgiVolumeMovementType::Scrolling)
                                .stable_id;
  const auto coarse_id = create_volume("M7 Coarse Nested Volume", {5, 5, 5}, 0.8f, {0.0f, 0.0f, -1.5f}, 20,
                                       DdgiVolumeMovementType::Default)
                             .stable_id;
  const auto dense_id =
      create_volume("M7 Dense Nested Volume", {7, 7, 7}, 0.4f, {0.0f, 0.0f, -1.5f}, 20, DdgiVolumeMovementType::Default)
          .stable_id;
  const auto slot_four_id =
      create_volume("M7 Slot Four Volume", {2, 2, 2}, 0.5f, {20.0f, 0.0f, -1.5f}, -1, DdgiVolumeMovementType::Default)
          .stable_id;
  const auto slot_five_id =
      create_volume("M7 Slot Five Volume", {2, 2, 2}, 0.5f, {24.0f, 0.0f, -1.5f}, -2, DdgiVolumeMovementType::Default)
          .stable_id;
  const auto slot_six_id =
      create_volume("M7 Slot Six Volume", {2, 2, 2}, 0.5f, {28.0f, 0.0f, -1.5f}, -3, DdgiVolumeMovementType::Default)
          .stable_id;
  const auto slot_seven_id =
      create_volume("M7 Slot Seven Volume", {2, 2, 2}, 0.5f, {32.0f, 0.0f, -1.5f}, -4, DdgiVolumeMovementType::Default)
          .stable_id;
  const std::vector<uint64_t> created_order{overlap_id,   scrolling_id, coarse_id,   dense_id,
                                            slot_four_id, slot_five_id, slot_six_id, slot_seven_id};
  const auto first_low_priority_id = std::min(overlap_id, scrolling_id);
  const auto second_low_priority_id = std::max(overlap_id, scrolling_id);
  const std::vector<uint64_t> expected_order{dense_id,     coarse_id,    first_low_priority_id, second_low_priority_id,
                                             slot_four_id, slot_five_id, slot_six_id,           slot_seven_id};
  const auto find_volume = [&](const uint64_t stable_id) -> EnvironmentalLighting::DdgiVolume* {
    for (auto& volume : lighting->GetOrCreateDdgiVolumePack()->volumes) {
      if (volume.stable_id == stable_id) {
        return &volume;
      }
    }
    return nullptr;
  };

  if (const auto window_layer = ApplicationContext::Get().GetLayer<WindowLayer>()) {
    window_layer->ResizeWindow(width, height);
    window_layer->CenterWindow();
  }
  const glm::uvec2 resolution(static_cast<uint32_t>(width), static_cast<uint32_t>(height));
  editor_layer->show_camera_window = false;
  editor_layer->RequestSceneCameraPreviewWindow(resolution);
  editor_layer->SetSceneCameraResolutionOverride(resolution);
  const auto scene_camera = editor_layer->GetSceneCamera();
  if (!scene_camera) {
    throw std::runtime_error("DDGI multi-volume validation requires a scene camera.");
  }
  scene_camera->camera_render_mode = Camera::CameraRenderMode::Rasterization;
  scene_camera->SetRequireRendering(true);
  scene_camera->Resize(resolution);
  scene_camera->ResetFrameCount();
  if (const auto main_camera = scene->main_camera.Get<Camera>(); main_camera && main_camera != scene_camera) {
    main_camera->SetEnabled(false);
  }

  constexpr size_t max_wait_frames = 30000;
  constexpr size_t settled_frame_count = 4;
  size_t stable_input_frames = 0;
  for (size_t frame = 0; frame < max_wait_frames && stable_input_frames < settled_frame_count; ++frame) {
    const bool ready = ProjectManager::IsProjectIdle() && !AssetManager::GetAssetLoadSnapshot().Active() &&
                       !TextureStorage::HasPendingUploads() && !GeometryStorage::HasPendingUploads() &&
                       !BottomLevelAccelerationStructure::HasPendingStaticBuilds();
    stable_input_frames = ready ? stable_input_frames + 1u : 0u;
    if (stable_input_frames < settled_frame_count && !ApplicationContext::Get().Loop()) {
      throw std::runtime_error("Application ended before DDGI multi-volume scene inputs were ready.");
    }
  }
  if (stable_input_frames < settled_frame_count) {
    throw std::runtime_error("DDGI multi-volume scene input readiness timed out.");
  }

  const auto all_runtime_ready = [](const std::vector<DdgiVolumeRuntimeStats>& stats) {
    return std::all_of(stats.begin(), stats.end(), [](const auto& volume) {
      return volume.resources_ready && volume.has_valid_probe_history && volume.contributes_lighting;
    });
  };
  const auto wait_for_runtime = [&](const size_t expected_count, const uint64_t removed_id = 0u) {
    for (size_t frame = 0; frame < max_wait_frames; ++frame) {
      if (!ApplicationContext::Get().Loop()) {
        throw std::runtime_error("Application ended during DDGI multi-volume runtime validation.");
      }
      const auto stats = render_layer->GetDdgiInspectorSnapshot().volumes;
      const bool removed = removed_id == 0u || std::none_of(stats.begin(), stats.end(), [&](const auto& volume) {
                             return volume.stable_entity_id == removed_id;
                           });
      if (stats.size() == expected_count && removed && all_runtime_ready(stats) &&
          render_layer->GetDdgiInspectorSnapshot().aggregate.lighting_descriptors_bound && scene_camera->Rendered()) {
        return stats;
      }
    }
    throw std::runtime_error("DDGI multi-volume runtime readiness timed out.");
  };
  const auto aggregate_probe_count = [](const std::vector<DdgiVolumeRuntimeStats>& stats) {
    uint32_t result = 0;
    for (const auto& volume : stats) {
      result += volume.probe_count;
    }
    return result;
  };
  const auto has_id_order = [](const std::vector<DdgiVolumeRuntimeStats>& stats, const std::vector<uint64_t>& ids) {
    if (stats.size() != ids.size()) {
      return false;
    }
    for (size_t i = 0; i < ids.size(); ++i) {
      if (stats[i].stable_entity_id != ids[i] || stats[i].sorted_index != i) {
        return false;
      }
    }
    return true;
  };
  const auto resources_are_unique = [](const std::vector<DdgiVolumeRuntimeStats>& stats) {
    std::set<uint64_t> resources;
    for (const auto& volume : stats) {
      for (const auto resource_id : volume.resource_ids) {
        if (resource_id == 0u || !resources.insert(resource_id).second) {
          return false;
        }
      }
    }
    return true;
  };
  const auto resource_map = [](const std::vector<DdgiVolumeRuntimeStats>& stats) {
    std::unordered_map<uint64_t, std::array<uint64_t, 5>> result;
    for (const auto& volume : stats) {
      result.emplace(volume.stable_entity_id, volume.resource_ids);
    }
    return result;
  };
  const auto resources_preserved = [](const std::unordered_map<uint64_t, std::array<uint64_t, 5>>& previous,
                                      const std::vector<DdgiVolumeRuntimeStats>& current,
                                      const uint64_t excluded_id = 0u) {
    for (const auto& volume : current) {
      if (volume.stable_entity_id == excluded_id) {
        continue;
      }
      const auto found = previous.find(volume.stable_entity_id);
      if (found == previous.end() || found->second != volume.resource_ids) {
        return false;
      }
    }
    return true;
  };
  const auto save_capture = [&](const std::filesystem::path& path) {
    const auto pixels =
        ReadAndStoreValidationCapture(scene_camera->GetRenderTexture(), resolution, path, "DDGI multi-volume");
    double luminance_sum = 0.0;
    for (const auto& pixel : pixels) {
      if (!std::isfinite(pixel.x) || !std::isfinite(pixel.y) || !std::isfinite(pixel.z) || !std::isfinite(pixel.w)) {
        throw std::runtime_error("DDGI multi-volume capture contains non-finite pixels.");
      }
      luminance_sum +=
          0.2126 * std::max(0.0f, pixel.x) + 0.7152 * std::max(0.0f, pixel.y) + 0.0722 * std::max(0.0f, pixel.z);
    }
    if (!(luminance_sum > 1e-6)) {
      throw std::runtime_error("DDGI multi-volume capture is black.");
    }
    return luminance_sum;
  };

  const auto initial_stats = wait_for_runtime(8u);
  const auto initial_resources = resource_map(initial_stats);
  constexpr size_t timing_measure_frames = 120;
  Platform::WaitForFrameSubmissions("DDGI Multi-Volume Timing Warmup Fence Wait");
  Platform::ResetGpuTimestampStats();
  for (size_t frame = 0; frame < timing_measure_frames; ++frame) {
    if (!ApplicationContext::Get().Loop()) {
      throw std::runtime_error("Application ended during DDGI multi-volume timing measurement.");
    }
  }
  Platform::WaitForFrameSubmissions("DDGI Multi-Volume Timing Completion Fence Wait");
  const auto multi_volume_gpu_timestamps = Platform::GetGpuTimestampStats();
  const auto initial_luminance = save_capture(output_directory / "initial.png");

  const auto infos = CollectDdgiVolumeRuntimeInfos(ResolveEnvironmentalLighting(scene));
  auto reverse_infos = infos;
  std::reverse(reverse_infos.begin(), reverse_infos.end());
  const auto selection_equal = [](const DdgiVolumeSelection& lhs, const DdgiVolumeSelection& rhs) {
    return lhs.valid == rhs.valid && lhs.primary_entity_id == rhs.primary_entity_id &&
           lhs.secondary_entity_id == rhs.secondary_entity_id &&
           std::abs(lhs.primary_weight - rhs.primary_weight) < 1e-6f &&
           std::abs(lhs.secondary_weight - rhs.secondary_weight) < 1e-6f &&
           std::abs(lhs.ibl_weight - rhs.ibl_weight) < 1e-6f;
  };
  const std::array<glm::vec3, 4> selection_points{
      {{-4.0f, 0.0f, -1.5f}, {0.0f, 0.0f, -1.5f}, {0.75f, 0.0f, -1.5f}, {1.15f, 0.0f, -1.5f}}};
  std::array<DdgiVolumeSelection, selection_points.size()> selections{};
  bool storage_order_independent = true;
  for (size_t i = 0; i < selection_points.size(); ++i) {
    selections[i] = DdgiRuntime::SelectVolumes(infos, selection_points[i]);
    storage_order_independent &=
        selection_equal(selections[i], DdgiRuntime::SelectVolumes(reverse_infos, selection_points[i]));
  }
  const bool disjoint_selection =
      selections[0].valid && selections[0].primary_entity_id == scrolling_id && selections[0].secondary_entity_id == 0u;
  const bool nested_selection =
      selections[1].valid && selections[1].primary_entity_id == dense_id && selections[1].secondary_entity_id == 0u;
  const bool deep_overlap_selection =
      selections[2].valid && selections[2].primary_entity_id == dense_id && selections[2].secondary_entity_id == 0u;
  const bool boundary_selection =
      selections[3].valid && selections[3].primary_entity_id == dense_id &&
      selections[3].secondary_entity_id == coarse_id && selections[3].primary_weight > 0.0f &&
      selections[3].secondary_weight > 0.0f &&
      std::abs(selections[3].primary_weight + selections[3].secondary_weight - 1.0f) < 1e-6f &&
      selections[3].ibl_weight == 0.0f;

  if (auto* scrolling_volume = find_volume(scrolling_id)) {
    scrolling_volume->transform = glm::translate(scrolling_volume->transform, glm::vec3(0.5f, 0.0f, 0.0f));
  } else {
    throw std::runtime_error("DDGI multi-volume validation lost its scrolling volume.");
  }
  std::vector<DdgiVolumeRuntimeStats> scrolled_stats;
  for (size_t frame = 0; frame < max_wait_frames; ++frame) {
    if (!ApplicationContext::Get().Loop()) {
      throw std::runtime_error("Application ended while scrolling a DDGI volume.");
    }
    const auto stats = render_layer->GetDdgiInspectorSnapshot().volumes;
    const auto moved = std::find_if(stats.begin(), stats.end(), [&](const auto& volume) {
      return volume.stable_entity_id == scrolling_id;
    });
    if (stats.size() == 8u && moved != stats.end() && moved->last_probe_scroll_delta == glm::ivec3(1, 0, 0) &&
        all_runtime_ready(stats) && render_layer->GetDdgiInspectorSnapshot().aggregate.lighting_descriptors_bound) {
      scrolled_stats = stats;
      break;
    }
  }
  if (scrolled_stats.empty()) {
    throw std::runtime_error("DDGI scrolling volume did not report its one-cell delta.");
  }
  const bool only_scrolling_volume_moved =
      std::all_of(scrolled_stats.begin(), scrolled_stats.end(), [&](const auto& volume) {
        const bool scrolling = volume.stable_entity_id == scrolling_id;
        return volume.last_probe_scroll_delta == (scrolling ? glm::ivec3(1, 0, 0) : glm::ivec3(0)) &&
               volume.probe_scroll_offset == (scrolling ? glm::ivec3(1, 0, 0) : glm::ivec3(0));
      });
  const bool scrolling_resources_preserved = resources_preserved(initial_resources, scrolled_stats);
  const auto scrolled_luminance = save_capture(output_directory / "scrolled.png");

  auto ddgi_pack = lighting->GetOrCreateDdgiVolumePack();
  const auto previous_volume_count = ddgi_pack->volumes.size();
  ddgi_pack->volumes.erase(std::remove_if(ddgi_pack->volumes.begin(), ddgi_pack->volumes.end(),
                                          [&](const auto& volume) {
                                            return volume.stable_id == dense_id;
                                          }),
                           ddgi_pack->volumes.end());
  if (ddgi_pack->volumes.size() == previous_volume_count) {
    throw std::runtime_error("DDGI multi-volume validation lost its removable dense volume.");
  }
  const auto removed_stats = wait_for_runtime(7u, dense_id);
  const auto removed_luminance = save_capture(output_directory / "removed.png");
  const std::vector<uint64_t> expected_removed_order{
      coarse_id, first_low_priority_id, second_low_priority_id, slot_four_id, slot_five_id, slot_six_id, slot_seven_id};
  const bool survivor_resources_preserved = resources_preserved(initial_resources, removed_stats, dense_id);
  const auto post_removal_selection = DdgiRuntime::SelectVolumes(
      CollectDdgiVolumeRuntimeInfos(ResolveEnvironmentalLighting(scene)), {0.0f, 0.0f, -1.5f});

  const std::vector<std::pair<std::string, bool>> checks{
      {"all_eight_runtime_slots_ready", initial_stats.size() == DdgiRuntime::kMaxVolumeCount},
      {"initial_probe_total", aggregate_probe_count(initial_stats) == 750u},
      {"priority_density_stable_id_order",
       created_order != expected_order && has_id_order(initial_stats, expected_order)},
      {"distinct_ready_resources", all_runtime_ready(initial_stats) && resources_are_unique(initial_stats)},
      {"disjoint_selection", disjoint_selection},
      {"nested_selection", nested_selection},
      {"deep_overlap_single_primary", deep_overlap_selection},
      {"boundary_single_secondary_normalized", boundary_selection},
      {"storage_order_independent", storage_order_independent},
      {"one_cell_scroll_isolated", only_scrolling_volume_moved},
      {"scrolling_resources_preserved", scrolling_resources_preserved},
      {"removal_count_and_probe_total", removed_stats.size() == 7u && aggregate_probe_count(removed_stats) == 407u &&
                                            has_id_order(removed_stats, expected_removed_order)},
      {"removal_preserves_survivors", survivor_resources_preserved && all_runtime_ready(removed_stats)},
      {"removal_reselects_primary",
       post_removal_selection.valid && post_removal_selection.primary_entity_id == coarse_id},
      {"lighting_descriptors_bound", render_layer->GetDdgiInspectorSnapshot().aggregate.lighting_descriptors_bound},
      {"captures_nonblank", initial_luminance > 1e-6 && scrolled_luminance > 1e-6 && removed_luminance > 1e-6}};

  const auto report_path = output_directory / "report.json";
  std::ofstream report(report_path, std::ios::trunc);
  if (!report) {
    throw std::runtime_error("Failed to open DDGI multi-volume report: " + report_path.string());
  }
  const auto write_ids = [&](const std::vector<uint64_t>& ids) {
    report << "[";
    for (size_t i = 0; i < ids.size(); ++i) {
      report << (i == 0u ? "" : ", ") << ids[i];
    }
    report << "]";
  };
  const auto write_runtime_stats = [&](const std::vector<DdgiVolumeRuntimeStats>& stats) {
    report << "[";
    for (size_t i = 0; i < stats.size(); ++i) {
      const auto& volume = stats[i];
      report << (i == 0u ? "" : ",") << "\n      {\"stable_entity_id\": " << volume.stable_entity_id
             << ", \"sorted_index\": " << volume.sorted_index << ", \"artist_priority\": " << volume.artist_priority
             << ", \"probe_density\": " << volume.probe_density << ", \"probe_counts\": [" << volume.probe_counts.x
             << ", " << volume.probe_counts.y << ", " << volume.probe_counts.z
             << "], \"probe_count\": " << volume.probe_count << ", \"scroll_offset\": [" << volume.probe_scroll_offset.x
             << ", " << volume.probe_scroll_offset.y << ", " << volume.probe_scroll_offset.z
             << "], \"last_scroll_delta\": [" << volume.last_probe_scroll_delta.x << ", "
             << volume.last_probe_scroll_delta.y << ", " << volume.last_probe_scroll_delta.z
             << "], \"history_valid\": " << (volume.has_valid_probe_history ? "true" : "false")
             << ", \"contributes_lighting\": " << (volume.contributes_lighting ? "true" : "false")
             << ", \"resources_ready\": " << (volume.resources_ready ? "true" : "false") << ", \"resource_ids\": [";
      for (size_t resource_index = 0; resource_index < volume.resource_ids.size(); ++resource_index) {
        report << (resource_index == 0u ? "" : ", ") << volume.resource_ids[resource_index];
      }
      report << "]}";
    }
    report << (stats.empty() ? "" : "\n    ") << "]";
  };
  report << std::setprecision(17);
  report << "{\n  \"schema_version\": 1,\n  \"contract\": {\"resolution\": [1920, 1080], "
            "\"vulkan_rt_pipeline\": true, \"graphics_validation\": true, \"launch_count\": 1},\n";
  report << "  \"created_order\": ";
  write_ids(created_order);
  report << ",\n  \"expected_runtime_order\": ";
  write_ids(expected_order);
  report << ",\n  \"runtime\": {\n    \"initial\": ";
  write_runtime_stats(initial_stats);
  report << ",\n    \"scrolled\": ";
  write_runtime_stats(scrolled_stats);
  report << ",\n    \"removed\": ";
  write_runtime_stats(removed_stats);
  report << "\n  },\n  \"gpu_timestamps\": {";
  bool wrote_gpu_timestamp = false;
  for (const auto& timestamp : multi_volume_gpu_timestamps) {
    if (timestamp.name.rfind("DDGI ", 0u) != 0u || timestamp.sample_count == 0u) {
      continue;
    }
    report << (wrote_gpu_timestamp ? "," : "") << "\n    \"" << timestamp.name
           << "\": {\"sample_count\": " << timestamp.sample_count
           << ", \"median_ms\": " << timestamp.MedianMilliseconds()
           << ", \"p95_ms\": " << timestamp.PercentileMilliseconds(0.95) << "}";
    wrote_gpu_timestamp = true;
  }
  report << (wrote_gpu_timestamp ? "\n  " : "")
         << "},\n  \"captures\": {\"initial.png\": {\"luminance_sum\": " << initial_luminance
         << "}, \"scrolled.png\": {\"luminance_sum\": " << scrolled_luminance
         << "}, \"removed.png\": {\"luminance_sum\": " << removed_luminance << "}},\n  \"checks\": {";
  bool all_checks_passed = true;
  for (size_t i = 0; i < checks.size(); ++i) {
    report << (i == 0u ? "" : ",") << "\n    \"" << checks[i].first << "\": " << (checks[i].second ? "true" : "false");
    all_checks_passed &= checks[i].second;
  }
  report << "\n  },\n  \"passed\": " << (all_checks_passed ? "true" : "false") << "\n}\n";
  report.close();

  std::cout << "EVOENGINE_DDGI_MULTI_VOLUME_REPORT path=\"" << report_path.string()
            << "\" passed=" << (all_checks_passed ? "true" : "false")
            << " initial_probes=" << aggregate_probe_count(initial_stats)
            << " remaining_probes=" << aggregate_probe_count(removed_stats) << std::endl;
  if (!all_checks_passed) {
    throw std::runtime_error("DDGI multi-volume validation report contains failed checks.");
  }
  editor_layer->SetSceneCameraResolutionOverride(std::nullopt);
  return true;
}

void evo_engine::ConfigureStrandMeshShaderValidation(const std::shared_ptr<Scene>& scene) {
  if (!scene) {
    return;
  }
  if (const auto regression_root = FindEntityNamed(scene, kRenderingRegressionRootName)) {
    scene->SetEnable(*regression_root, false);
  }
  if (const auto existing_root = FindEntityNamed(scene, kStrandValidationRootName)) {
    scene->DeleteEntity(*existing_root);
  }

  if (const auto lighting = GetOrCreateTemporaryEnvironmentalLighting(scene)) {
    ConfigureEnvironmentalLightingColorSource(*lighting, glm::vec3(0.025f, 0.03f, 0.04f), 0.12f, 1.0f);
  }

  const auto root = scene->CreateEntity(kStrandValidationRootName);
  const auto ground_material = AssetManager::CreateTemporaryAsset<Material>();
  ConfigureMaterial(ground_material, glm::vec3(0.42f, 0.45f, 0.5f), 0.9f, 0.0f);
  const auto ground = scene->CreateEntity("Strand Validation Ground");
  const auto ground_renderer = scene->GetOrSetPrivateComponent<MeshRenderer>(ground).lock();
  ground_renderer->mesh = Resources::GetInstance().GetPrimitives().cube;
  ground_renderer->material = ground_material;
  Transform ground_transform;
  ground_transform.SetValue(glm::vec3(0.0f, -1.0f, -125.0f), glm::vec3(0.0f), glm::vec3(32.0f, 0.08f, 250.0f));
  scene->SetDataComponent(ground, ground_transform);
  scene->SetParent(ground, root);

  const auto single_material = AssetManager::CreateTemporaryAsset<Material>();
  const auto multi_material = AssetManager::CreateTemporaryAsset<Material>();
  ConfigureMaterial(single_material, glm::vec3(0.95f, 0.28f, 0.12f), 0.48f, 0.0f);
  ConfigureMaterial(multi_material, glm::vec3(0.12f, 0.55f, 0.95f), 0.42f, 0.0f);
  CreateStrandValidationRenderer(scene, root, "Strand Validation Single Segment",
                                 CreateStrandValidationGeometry(4, glm::vec4(1.0f, 0.45f, 0.18f, 1.0f)),
                                 single_material, glm::vec3(-0.75f, 0.0f, -2.5f), glm::vec3(1.0f), false);
  CreateStrandValidationRenderer(scene, root, kStrandValidationDynamicName,
                                 CreateStrandValidationGeometry(58, glm::vec4(0.18f, 0.65f, 1.0f, 1.0f)),
                                 multi_material, glm::vec3(0.75f, 0.0f, -2.5f), glm::vec3(-1.15f, 0.8f, 1.35f), true);
  const auto cascade_geometry = CreateStrandValidationGeometry(4, glm::vec4(1.0f, 0.82f, 0.18f, 1.0f));
  const std::array cascade_positions = {glm::vec3(-4.0f, 0.0f, -70.0f), glm::vec3(0.0f, 0.0f, -130.0f),
                                        glm::vec3(12.0f, 0.0f, -190.0f)};
  for (size_t cascade = 0; cascade < cascade_positions.size(); ++cascade) {
    const float scale = static_cast<float>(cascade + 2);
    CreateStrandValidationRenderer(scene, root, "Strand Validation Cascade " + std::to_string(cascade + 1),
                                   cascade_geometry, single_material, cascade_positions[cascade],
                                   glm::vec3(scale, scale * 2.0f, scale), true);
  }

  const auto light_entity = scene->CreateEntity("Strand Validation Directional Light");
  const auto light = scene->GetOrSetPrivateComponent<DirectionalLight>(light_entity).lock();
  light->cast_shadow = true;
  light->diffuse = glm::vec3(1.0f, 0.96f, 0.88f);
  light->diffuse_brightness = 3.5f;
  light->light_size = 0.02f;
  light->bias = 0.02f;
  light->normal_offset = 0.02f;
  Transform light_transform;
  const auto direction = glm::normalize(glm::vec3(0.45f, -0.85f, -0.32f));
  light_transform.SetRotation(glm::quatLookAt(-direction, glm::vec3(0.0f, 1.0f, 0.0f)));
  scene->SetDataComponent(light_entity, light_transform);
  scene->SetParent(light_entity, root);
  SyncTemporaryEnvironmentalLightingSettingsFromScene(scene);
}

void evo_engine::UpdateStrandMeshShaderValidationGeometry(const std::shared_ptr<Scene>& scene) {
  const auto entity = scene ? FindEntityNamed(scene, kStrandValidationDynamicName) : std::nullopt;
  if (!entity) {
    throw std::runtime_error("Strand mesh-shader validation was not configured before its geometry update.");
  }
  const auto renderer = scene->GetOrSetPrivateComponent<StrandsRenderer>(*entity).lock();
  const auto strands = renderer ? renderer->strands.Get<Strands>() : nullptr;
  if (!strands || strands->GetStrandPointAmount() != 58) {
    throw std::runtime_error("Strand mesh-shader validation dynamic geometry is invalid.");
  }
  auto points = strands->PeekStrandPoints();
  for (size_t index = 0; index < points.size(); ++index) {
    const float t = static_cast<float>(index) / static_cast<float>(points.size() - 1);
    points[index].position.x += 0.04f * glm::sin(t * glm::two_pi<float>() * 3.0f);
  }
  StrandPointAttributes attributes;
  attributes.normal = true;
  attributes.tex_coord = true;
  attributes.color = true;
  strands->SetStrands(attributes, {0, static_cast<glm::uint>(points.size())}, points);
}

void evo_engine::ConfigureStrandGizmoValidation(const std::shared_ptr<Scene>& scene) {
  if (!scene) {
    return;
  }
  for (const auto* root_name :
       {kRenderingRegressionRootName, kStrandValidationRootName, kStrandPunctualValidationRootName}) {
    if (const auto root = FindEntityNamed(scene, root_name)) {
      scene->SetEnable(*root, false);
    }
  }

  if (const auto lighting = GetOrCreateTemporaryEnvironmentalLighting(scene)) {
    ConfigureEnvironmentalLightingColorSource(*lighting, glm::vec3(0.025f, 0.03f, 0.04f), 0.0f, 1.0f);
  }

  const auto strands = CreateStrandValidationGeometry(4, glm::vec4(1.0f), 0.12f);
  auto points = strands->PeekStrandPoints();
  constexpr std::array colors = {glm::vec4(1.0f, 0.12f, 0.08f, 1.0f), glm::vec4(0.1f, 1.0f, 0.18f, 1.0f),
                                 glm::vec4(0.08f, 0.3f, 1.0f, 1.0f), glm::vec4(1.0f, 0.9f, 0.08f, 1.0f)};
  constexpr std::array normals = {glm::vec3(0.25f, 0.25f, 1.0f), glm::vec3(0.65f, 0.25f, 0.8f),
                                  glm::vec3(0.25f, 0.75f, 0.7f), glm::vec3(0.8f, 0.55f, 0.35f)};
  for (size_t index = 0; index < points.size(); ++index) {
    points[index].color = colors[index];
    points[index].normal = glm::normalize(normals[index]);
  }
  StrandPointAttributes attributes;
  attributes.normal = true;
  attributes.tex_coord = true;
  attributes.color = true;
  strands->SetSegments(attributes, {0}, points);

  strand_gizmo_validation_state = std::make_shared<StrandGizmoValidationState>();
  strand_gizmo_validation_state->scene = scene;
  strand_gizmo_validation_state->strands = strands;
  RegisterStrandGizmoValidationUpdate();
  SyncTemporaryEnvironmentalLightingSettingsFromScene(scene);
}

void evo_engine::ConfigureStrandPunctualShadowValidation(const std::shared_ptr<Scene>& scene) {
  if (!scene) {
    return;
  }
  if (const auto regression_root = FindEntityNamed(scene, kRenderingRegressionRootName)) {
    scene->SetEnable(*regression_root, false);
  }
  if (const auto strand_root = FindEntityNamed(scene, kStrandValidationRootName)) {
    scene->SetEnable(*strand_root, false);
  }
  if (const auto existing_root = FindEntityNamed(scene, kStrandPunctualValidationRootName)) {
    scene->DeleteEntity(*existing_root);
  }
  if (const auto* owners = scene->UnsafeGetPrivateComponentOwnersList<DirectionalLight>()) {
    for (const auto& owner : *owners) {
      if (const auto light = scene->GetOrSetPrivateComponent<DirectionalLight>(owner).lock()) {
        light->cast_shadow = false;
        light->SetEnabled(false);
      }
    }
  }
  if (const auto* owners = scene->UnsafeGetPrivateComponentOwnersList<PointLight>()) {
    for (const auto& owner : *owners) {
      if (const auto light = scene->GetOrSetPrivateComponent<PointLight>(owner).lock()) {
        light->cast_shadow = false;
        light->SetEnabled(false);
      }
    }
  }
  if (const auto* owners = scene->UnsafeGetPrivateComponentOwnersList<SpotLight>()) {
    for (const auto& owner : *owners) {
      if (const auto light = scene->GetOrSetPrivateComponent<SpotLight>(owner).lock()) {
        light->cast_shadow = false;
        light->SetEnabled(false);
      }
    }
  }

  if (const auto lighting = GetOrCreateTemporaryEnvironmentalLighting(scene)) {
    ConfigureEnvironmentalLightingColorSource(*lighting, glm::vec3(0.025f, 0.03f, 0.04f), 0.0f, 1.0f);
  }

  const auto root = scene->CreateEntity(kStrandPunctualValidationRootName);
  const auto point_material = AssetManager::CreateTemporaryAsset<Material>();
  const auto spot_material = AssetManager::CreateTemporaryAsset<Material>();
  ConfigureMaterial(point_material, glm::vec3(1.0f, 0.45f, 0.12f), 0.45f, 0.0f);
  ConfigureMaterial(spot_material, glm::vec3(0.18f, 0.55f, 1.0f), 0.45f, 0.0f);
  const auto point_geometry = CreateStrandValidationGeometry(4, glm::vec4(1.0f, 0.45f, 0.12f, 1.0f));
  const auto spot_geometry = CreateStrandValidationGeometry(4, glm::vec4(0.18f, 0.55f, 1.0f, 1.0f), 0.24f);

  const glm::vec3 point_position(-5.0f, 2.0f, -12.0f);
  const std::array point_directions = {glm::vec3(1.0f, 0.0f, 0.0f), glm::vec3(-1.0f, 0.0f, 0.0f),
                                       glm::vec3(0.0f, 1.0f, 0.0f), glm::vec3(0.0f, -1.0f, 0.0f),
                                       glm::vec3(0.0f, 0.0f, 1.0f), glm::vec3(0.0f, 0.0f, -1.0f)};
  const std::array point_panel_scales = {glm::vec3(0.08f, 1.2f, 1.2f), glm::vec3(0.08f, 1.2f, 1.2f),
                                         glm::vec3(1.2f, 0.08f, 1.2f), glm::vec3(1.2f, 0.08f, 1.2f),
                                         glm::vec3(1.2f, 1.2f, 0.08f), glm::vec3(1.2f, 1.2f, 0.08f)};
  for (size_t face = 0; face < point_directions.size(); ++face) {
    const bool y_face = face == 2 || face == 3;
    const auto rotation =
        y_face ? glm::vec3(0.0f, 0.0f, glm::half_pi<float>()) : glm::vec3(0.0f, 0.0f, glm::radians(8.0f));
    const auto visible_offset = face == 5 ? glm::vec3(0.8f, 0.0f, 0.0f) : glm::vec3(0.0f);
    CreateStrandValidationRenderer(scene, root, "Strand Point Face " + std::to_string(face), point_geometry,
                                   point_material, point_position + point_directions[face] * 1.25f + visible_offset,
                                   glm::vec3(0.55f), true, rotation);
    CreateRenderingRegressionProbe(scene, root, "Strand Point Receiver " + std::to_string(face),
                                   Resources::GetInstance().GetPrimitives().cube,
                                   point_position + point_directions[face] * 2.5f + visible_offset,
                                   point_panel_scales[face], glm::vec3(0.42f, 0.44f, 0.48f), 0.9f, 0.0f, 0.0f, false);
  }
  CreateStrandValidationRenderer(scene, root, "Strand Point Cast False", point_geometry, point_material,
                                 point_position + glm::vec3(1.25f, 0.0f, 0.65f), glm::vec3(0.55f), false);

  const auto point_light_entity = scene->CreateEntity("Strand Validation Point Light");
  const auto point_light = scene->GetOrSetPrivateComponent<PointLight>(point_light_entity).lock();
  point_light->cast_shadow = true;
  point_light->diffuse = glm::vec3(1.0f, 0.55f, 0.25f);
  point_light->diffuse_brightness = 20.0f;
  point_light->range = 6.0f;
  point_light->shadow_distance = 6.0f;
  point_light->constant = 1.0f;
  point_light->linear = 0.2f;
  point_light->quadratic = 0.08f;
  point_light->bias = 0.002f;
  Transform point_light_transform;
  point_light_transform.SetPosition(point_position);
  scene->SetDataComponent(point_light_entity, point_light_transform);
  scene->SetParent(point_light_entity, root);

  const glm::vec3 spot_position(5.0f, 4.0f, -9.0f);
  const glm::vec3 spot_target(5.0f, -0.75f, -14.0f);
  const auto spot_direction = glm::normalize(spot_target - spot_position);
  CreateStrandValidationRenderer(scene, root, "Strand Spot Cast True", spot_geometry, spot_material,
                                 spot_position + spot_direction * 3.2f, glm::vec3(1.0f), true);
  CreateStrandValidationRenderer(scene, root, "Strand Spot Cast False", spot_geometry, spot_material,
                                 spot_position + spot_direction * 3.2f + glm::vec3(0.85f, 0.0f, 0.0f), glm::vec3(1.0f),
                                 false);
  const auto spot_receiver_position = spot_position + spot_direction * 6.0f;
  const auto spot_receiver = CreateRenderingRegressionProbe(
      scene, root, "Strand Spot Receiver", Resources::GetInstance().GetPrimitives().cube, spot_receiver_position,
      glm::vec3(2.8f, 2.8f, 0.08f), glm::vec3(0.38f, 0.42f, 0.5f), 0.9f, 0.0f, 0.0f, false);
  Transform spot_receiver_transform;
  spot_receiver_transform.SetPosition(spot_receiver_position);
  spot_receiver_transform.SetRotation(glm::quatLookAt(spot_direction, glm::vec3(0.0f, 1.0f, 0.0f)));
  spot_receiver_transform.SetScale(glm::vec3(2.8f, 2.8f, 0.08f));
  scene->SetDataComponent(spot_receiver, spot_receiver_transform);

  const auto spot_light_entity = scene->CreateEntity("Strand Validation Spot Light");
  const auto spot_light = scene->GetOrSetPrivateComponent<SpotLight>(spot_light_entity).lock();
  spot_light->cast_shadow = true;
  spot_light->diffuse = glm::vec3(0.3f, 0.6f, 1.0f);
  spot_light->diffuse_brightness = 20.0f;
  spot_light->inner_degrees = 18.0f;
  spot_light->outer_degrees = 28.0f;
  spot_light->range = 10.0f;
  spot_light->shadow_distance = 10.0f;
  spot_light->constant = 1.0f;
  spot_light->linear = 0.2f;
  spot_light->quadratic = 0.08f;
  spot_light->bias = 0.0005f;
  Transform spot_light_transform;
  spot_light_transform.SetPosition(spot_position);
  spot_light_transform.SetRotation(glm::quatLookAt(spot_direction, glm::vec3(0.0f, 1.0f, 0.0f)));
  scene->SetDataComponent(spot_light_entity, spot_light_transform);
  scene->SetParent(spot_light_entity, root);
  SyncTemporaryEnvironmentalLightingSettingsFromScene(scene);
}

void evo_engine::ConfigureBistroRayTracingPostProcessing(const std::shared_ptr<Camera>& camera) {
  ConfigureBistroReferenceToneMapping(camera);
}

void evo_engine::ConfigureBistroParityCapture(const std::shared_ptr<Scene>& scene,
                                              const std::shared_ptr<Camera>& camera) {
  if (!scene || !camera) {
    return;
  }
  if (!Camera::IsRayCameraRenderMode(Camera::ResolveCameraRenderMode(camera->camera_render_mode))) {
    auto& ddgi = RequireEnvironmentalLightingDdgiSettings(scene);
    ddgi.runtime.enabled = true;
    DisableDdgiDebugVisualization();
    SyncTemporaryEnvironmentalLightingSettingsFromScene(scene);
    ConfigureBistroRasterizationPostProcessing(camera);
    camera->ResetFrameCount();
    return;
  }
  ApplyBistroParityRendererState(scene);
  SyncTemporaryEnvironmentalLightingSettingsFromScene(scene);
  ConfigureBistroReferenceToneMapping(camera);
  camera->camera_settings.bounce = kBistroReferencePathTraceMaxDepth;
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
  const auto& graphics_settings = ApplicationContext::Get().GetApplicationInfo().graphics_settings;
  const auto lighting = scene->environmental_lighting.Get<EnvironmentalLighting>();
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
         << ", directional_shadow_map_resolution=" << graphics_settings.directional_light_shadow_map_resolution
         << ", environment_source_kind="
         << (lighting ? static_cast<int>(lighting->indirect_environment_source.kind) : -1)
         << ", background_intensity=" << camera->camera_settings.background_intensity
         << ", environment_lighting_intensity=" << (lighting ? lighting->environment_lighting_intensity : 0.0f)
         << ", diffuse_fallback_intensity=" << (lighting ? lighting->diffuse_fallback_intensity : 0.0f)
         << ", specular_fallback_intensity=" << (lighting ? lighting->specular_fallback_intensity : 0.0f)
         << ", ddgi_enabled=" << (lighting && lighting->ddgi_settings.runtime.enabled);
  const auto resolved_render_mode = Camera::ResolveCameraRenderMode(camera->camera_render_mode);
  if (Camera::IsRayCameraRenderMode(resolved_render_mode)) {
    const auto technique = resolved_render_mode == Camera::CameraRenderMode::RayQuery
                               ? RayCameraShaderTechnique::RayQuery
                               : RayCameraShaderTechnique::RayTracing;
    if (const auto render_layer = ApplicationContext::Get().GetLayer<RenderLayer>()) {
      const auto shader_variant = render_layer->GetRayCameraShaderVariantStats(technique);
      stream << ", ray_shader_requested_key=" << shader_variant.requested_key
             << ", ray_shader_active_key=" << shader_variant.active_key
             << ", ray_shader_cache_source=" << shader_variant.cache_source
             << ", ray_shader_ready=" << shader_variant.ready;
      if (const auto render_instances = render_layer->GetCurrentRenderInstanceStorage()) {
        std::map<uint32_t, size_t> feature_mask_counts;
        const auto& texture_infos = render_instances->GetGltfTextureInfos();
        for (const auto& material : render_instances->GetGltfShadeMaterials()) {
          ++feature_mask_counts[DetectGltfSceneFeatures({material}, texture_infos)];
        }
        stream << ", ray_material_feature_masks=[";
        bool first_mask = true;
        for (const auto& [mask, count] : feature_mask_counts) {
          stream << (first_mask ? "" : ";") << FormatGltfSceneFeatureMask(mask) << ":" << count;
          first_mask = false;
        }
        size_t ray_camera_count = 0;
        size_t ray_debug_camera_count = 0;
        for (const auto& [transform, collected_camera] : render_instances->cameras) {
          if (!collected_camera ||
              !Camera::IsRayCameraRenderMode(Camera::ResolveCameraRenderMode(collected_camera->camera_render_mode))) {
            continue;
          }
          ++ray_camera_count;
          ray_debug_camera_count +=
              collected_camera->camera_settings.ray_debug_view != CameraSettings::RayDebugView::Beauty;
        }
        stream << "], ray_camera_count=" << ray_camera_count << ", ray_debug_camera_count=" << ray_debug_camera_count;
      }
    }
  }
  if (const auto editor_layer = ApplicationContext::Get().GetLayer<EditorLayer>()) {
    const auto position = editor_layer->GetSceneCameraPosition();
    const auto rotation = editor_layer->GetSceneCameraRotation();
    stream << ", camera_position=(" << position.x << "," << position.y << "," << position.z << "), camera_rotation=("
           << rotation.w << "," << rotation.x << "," << rotation.y << "," << rotation.z << ")";
  }
  if (const auto post_processing_stack = camera->post_processing_stack_ref.Get<PostProcessingStack>()) {
    stream << ", ambient_occlusion_enabled=" << post_processing_stack->enable_ambient_occlusion
           << ", bloom_enabled=" << post_processing_stack->enable_bloom
           << ", screen_space_reflection_enabled=" << post_processing_stack->enable_screen_space_reflection
           << ", anti_aliasing_enabled=" << post_processing_stack->enable_anti_aliasing
           << ", tone_mapping_enabled=" << post_processing_stack->enable_tone_mapping;
    if (post_processing_stack->ambient_occlusion) {
      stream << ", ambient_occlusion_algorithm="
             << static_cast<int>(post_processing_stack->ambient_occlusion->algorithm);
    }
    if (post_processing_stack->anti_aliasing) {
      stream << ", anti_aliasing_algorithm=" << static_cast<int>(post_processing_stack->anti_aliasing->algorithm);
    }
    if (post_processing_stack->tone_mapping) {
      const auto& tone_mapping = *post_processing_stack->tone_mapping;
      stream << ", tone_mapping_method=" << static_cast<int>(tone_mapping.method)
             << ", tone_mapping_exposure=" << tone_mapping.exposure
             << ", tone_mapping_brightness=" << tone_mapping.brightness
             << ", tone_mapping_contrast=" << tone_mapping.contrast
             << ", tone_mapping_saturation=" << tone_mapping.saturation
             << ", tone_mapping_auto_exposure=" << tone_mapping.auto_exposure
             << ", tone_mapping_average_mode=" << tone_mapping.average_mode;
    }
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

  if (const auto lighting = GetOrCreateTemporaryEnvironmentalLighting(scene)) {
    ConfigureEnvironmentalLightingMapSource(*lighting, Resources::GetInstance().GetDefaultEnvironmentalMap(), 1.0f,
                                            0.0f);
  }
  scene->global_reflection_probe_fallback = Resources::GetInstance().GetDefaultGlobalReflectionProbe();
  ConfigureBistroDemoDdgi(scene, bistro_world_bound);
  if (const auto main_camera = scene->main_camera.Get<Camera>()) {
    main_camera->Resize({1920, 1080});
    main_camera->skybox.Clear();
    main_camera->camera_render_mode = Camera::CameraRenderMode::RayTracing;
    main_camera->camera_settings.background_source = Camera::BackgroundSource::InheritEnvironmentalLighting;
    main_camera->camera_settings.background_intensity = 1.0f;
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
      scene_camera->camera_settings.background_source = Camera::BackgroundSource::InheritEnvironmentalLighting;
      scene_camera->camera_settings.background_intensity = 1.0f;
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
      ProjectManager::SetActionAfterNewScene(ConfigureRenderingDemoScene);
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
