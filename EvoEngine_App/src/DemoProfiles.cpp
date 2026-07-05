#include "EvoEngine_SDK_PCH.hpp"

#include "DemoProfiles.hpp"

#include "Application.hpp"
#include "ApplicationInitializationSettings.hpp"
#include "AssetManager.hpp"
#include "Camera.hpp"
#include "DdgiVolume.hpp"
#include "EditorLayer.hpp"
#include "Entity.hpp"
#include "Lights.hpp"
#include "Material.hpp"
#include "Mesh.hpp"
#include "MeshRenderer.hpp"
#include "PathUtils.hpp"
#include "Scene.hpp"
#include "SkinnedMesh.hpp"
#include "SkinnedMeshRenderer.hpp"
#include "Transform.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <filesystem>
#include <optional>

namespace evo_engine {
namespace {
const glm::ivec2 kDdgiCornellBoxExtent = {1024, 1024};
const glm::vec3 kDdgiCornellBoxCameraPosition = {0.0f, 0.0f, 0.8f};
const glm::ivec3 kDdgiCornellBoxProbeCounts = {13, 13, 14};
const glm::vec3 kDdgiCornellBoxVolumeOrigin = {0.0f, 0.0f, 0.0f};
constexpr float kDdgiCornellBoxProbeSpacing = 0.14333334f;
constexpr float kDdgiCornellBoxCeilingLightEmission = 6.0f;
constexpr float kDdgiCornellBoxNormalBias = 0.02f;
constexpr float kDdgiCornellBoxViewBias = 0.05f;

std::filesystem::path ExistingResourcesRoot(const std::filesystem::path& preferred_root) {
  if (!preferred_root.empty() && std::filesystem::exists(preferred_root)) {
    return path_utils::NormalizeAbsolutePath(preferred_root);
  }
  return path_utils::FindAncestorChildPath("Resources", std::filesystem::current_path(), 8);
}

std::filesystem::path StableResourcesRoot(const std::filesystem::path& preferred_root) {
  if (const auto resources_root = ExistingResourcesRoot(preferred_root); !resources_root.empty()) {
    return resources_root;
  }
  return path_utils::NormalizeAbsolutePath(preferred_root.empty() ? std::filesystem::path("Resources")
                                                                  : preferred_root);
}

std::string Lowercase(std::string value) {
  std::transform(value.begin(), value.end(), value.begin(), [](const unsigned char character) {
    return static_cast<char>(std::tolower(character));
  });
  return value;
}

struct ShowcaseInspectorTarget {
  Entity entity;
  std::shared_ptr<IAsset> mesh;
  std::shared_ptr<IAsset> material;
  int score = -1;
};

int ScoreAssetTitle(const Handle& asset_handle, const std::string& needle, const int weight) {
  if (asset_handle.GetValue() == 0) {
    return 0;
  }
  const auto asset = AssetManager::GetAsset(asset_handle);
  if (!asset) {
    return 0;
  }
  return Lowercase(asset->GetTitle()).find(needle) != std::string::npos ? weight : 0;
}

int ScoreMaterialTextures(const std::shared_ptr<Material>& material) {
  if (!material) {
    return 0;
  }
  const auto& texture_refs = material->PeekTextureRefs();
  const auto score_texture = [&](const uint16_t texture_info_slot, const std::string& needle, const int weight) {
    if (texture_info_slot == 0 || texture_info_slot >= texture_refs.size()) {
      return 0;
    }
    return ScoreAssetTitle(texture_refs[texture_info_slot].GetAssetHandle(), needle, weight);
  };
  int score = 0;
  const auto& shade_material = material->material_data.shade_material;
  score += score_texture(shade_material.pbr_base_color_texture, "curtain", 5);
  score += score_texture(shade_material.normal_texture, "curtain", 3);
  score += score_texture(shade_material.pbr_base_color_texture, "blue", 1);
  score += score_texture(shade_material.pbr_base_color_texture, "green", 1);
  return score;
}

void ConsiderShowcaseTarget(const std::shared_ptr<Scene>& scene, const Entity& entity,
                            const std::shared_ptr<IAsset>& mesh, const std::shared_ptr<Material>& material,
                            ShowcaseInspectorTarget& target) {
  if (!mesh || !material) {
    return;
  }
  const auto entity_name = Lowercase(scene->GetEntityName(entity));
  const auto mesh_title = Lowercase(mesh->GetTitle());
  const auto material_title = Lowercase(material->GetTitle());
  int score = 0;
  if (entity_name.find("curtain") != std::string::npos) {
    score += 8;
  }
  if (mesh_title.find("curtain") != std::string::npos) {
    score += 4;
  }
  if (material_title.find("curtain") != std::string::npos) {
    score += 4;
  }
  score += ScoreMaterialTextures(material);
  if (entity_name.find("blue") != std::string::npos || mesh_title.find("blue") != std::string::npos ||
      material_title.find("blue") != std::string::npos) {
    score += 1;
  }
  if (score > target.score) {
    target = {entity, mesh, material, score};
  }
}

ShowcaseInspectorTarget FindShowcaseInspectorTarget(const std::shared_ptr<Scene>& scene) {
  ShowcaseInspectorTarget target;
  for (const auto& entity : scene->UnsafeGetAllEntities()) {
    if (!scene->IsEntityValid(entity)) {
      continue;
    }
    if (scene->HasPrivateComponent<MeshRenderer>(entity)) {
      const auto mesh_renderer = scene->GetOrSetPrivateComponent<MeshRenderer>(entity).lock();
      ConsiderShowcaseTarget(scene, entity, mesh_renderer->mesh.Get<Mesh>(), mesh_renderer->material.Get<Material>(),
                             target);
    }
    if (scene->HasPrivateComponent<SkinnedMeshRenderer>(entity)) {
      const auto skinned_mesh_renderer = scene->GetOrSetPrivateComponent<SkinnedMeshRenderer>(entity).lock();
      ConsiderShowcaseTarget(scene, entity, skinned_mesh_renderer->skinned_mesh.Get<SkinnedMesh>(),
                             skinned_mesh_renderer->material.Get<Material>(), target);
    }
  }
  return target;
}

void OpenShowcaseAssetInspector(const std::shared_ptr<EditorLayer>& editor_layer,
                                const std::shared_ptr<IAsset>& material) {
  editor_layer->ClearAssetInspectors();
  if (material) {
    editor_layer->OpenAssetInspector(material);
  }
}

void FrameSceneCameraOnTarget(const std::shared_ptr<EditorLayer>& editor_layer, const std::shared_ptr<Scene>& scene,
                              const Entity& entity) {
  if (!scene->IsEntityValid(entity)) {
    return;
  }
  editor_layer->SetSceneCameraPosition(glm::vec3(0.0f, 0.0f, 3.0f));
  editor_layer->SetSceneCameraRotation(glm::quat(glm::vec3(0.0f)));
}

std::optional<Entity> FindEntityByName(const std::shared_ptr<Scene>& scene, const std::string& name) {
  if (!scene) {
    return {};
  }
  for (const auto& entity : scene->UnsafeGetAllEntities()) {
    if (scene->IsEntityValid(entity) && scene->GetEntityName(entity) == name) {
      return entity;
    }
  }
  return {};
}

void EnableMainCameraRayTracing(const std::shared_ptr<Scene>& scene) {
  if (const auto main_camera = scene->main_camera.Get<Camera>()) {
    main_camera->camera_render_mode = Camera::CameraRenderMode::RayTracing;
    main_camera->ResetFrameCount();
  }
}

bool PrepareDdgiShowcase(const std::shared_ptr<EditorLayer>& editor_layer, const std::shared_ptr<Scene>& scene) {
  const auto ddgi_volume_entity = FindEntityByName(scene, "DDGI Probe Volume");
  if (!ddgi_volume_entity) {
    return false;
  }

  auto& settings = scene->environment.ddgi_settings;
  settings.runtime.enabled = true;
  settings.debug.enabled = true;
  settings.debug.visualize_volume_bounds = true;
  settings.debug.visualize_probe_positions = true;
  settings.debug.visualize_selected_probe = true;
  settings.debug.visualization_scale = 2.0f;
  settings.debug.selected_probe_index = 129;

  if (const auto volume = scene->GetOrSetPrivateComponent<DdgiVolume>(*ddgi_volume_entity).lock()) {
    volume->visualize_bounds = true;
    volume->visualize_probe_positions = true;
    volume->max_visualized_probes = 512;
    volume->probe_visualization_size = 0.14f;
    volume->ClampSettings();
  }

  const glm::vec3 camera_position(0.0f, 0.0f, 3.0f);
  const glm::quat camera_rotation(glm::vec3(0.0f));
  if (const auto main_camera = scene->main_camera.Get<Camera>()) {
    main_camera->Resize({1920, 1080});
    auto camera_transform = scene->GetDataComponent<Transform>(main_camera->GetOwner());
    camera_transform.SetPosition(camera_position);
    camera_transform.SetRotation(camera_rotation);
    scene->SetDataComponent(main_camera->GetOwner(), camera_transform);
  }
  EnableMainCameraRayTracing(scene);
  editor_layer->main_camera_allow_auto_resize = true;
  editor_layer->main_camera_resolution_x = 1920;
  editor_layer->main_camera_resolution_y = 1080;
  editor_layer->SetSceneCameraPosition(camera_position);
  editor_layer->SetSceneCameraRotation(camera_rotation);
  if (const auto scene_camera = editor_layer->GetSceneCamera()) {
    scene_camera->camera_render_mode = Camera::CameraRenderMode::Rasterization;
    scene_camera->ResetFrameCount();
  }
  editor_layer->SetSelectedEntity(*ddgi_volume_entity, false);
  OpenShowcaseAssetInspector(editor_layer, FindShowcaseInspectorTarget(scene).material);
  return true;
}

void PrepareRenderingDemoShowcase(const std::shared_ptr<EditorLayer>& editor_layer) {
  const auto scene = ApplicationContext::Get().GetActiveScene();
  if (!scene) {
    return;
  }
  EnableMainCameraRayTracing(scene);
  if (!editor_layer) {
    return;
  }

  if (PrepareDdgiShowcase(editor_layer, scene)) {
    return;
  }

  const auto target = FindShowcaseInspectorTarget(scene);
  if (target.score >= 0) {
    if (target.score >= 4) {
      scene->SetEntityName(target.entity, "Blur Curtain");
    }
    editor_layer->SetSelectedEntity(target.entity, false);
    FrameSceneCameraOnTarget(editor_layer, scene, target.entity);
    OpenShowcaseAssetInspector(editor_layer, target.material);
  }
}

EditorLayoutSettings CreateRenderingDemoEditorLayout() {
  EditorLayoutSettings settings;
  settings.panels.scene = true;
  settings.panels.camera = true;
  settings.panels.scene_info = true;
  settings.panels.camera_info = true;
  settings.panels.entity_explorer = true;
  settings.panels.entity_inspector = true;
  settings.panels.console = true;
  settings.panels.project = true;
  settings.panels.resources = false;
  settings.panels.runtime_package_manager = true;
  settings.panels.render_layer_inspection = false;

  EditorDockLayoutSettings dock_layout;
  dock_layout.left_fraction = 0.15f;
  dock_layout.right_fraction = 0.19f;
  dock_layout.bottom_fraction = 0.28f;
  dock_layout.camera_fraction = 0.50f;
  settings.dock_layout = dock_layout;

  EditorFloatingWindowLayout asset_inspector_window;
  asset_inspector_window.anchor = EditorFloatingWindowLayout::Anchor::LowerLeft;
  asset_inspector_window.size = {292.0f, 530.0f};
  asset_inspector_window.margin = {24.0f, 24.0f};
  settings.asset_inspector_window = asset_inspector_window;

  EditorRuntimePackageManagerLayoutSettings package_manager;
  package_manager.floating_window.anchor = EditorFloatingWindowLayout::Anchor::LowerRight;
  package_manager.floating_window.size = {600.0f, 530.0f};
  package_manager.floating_window.margin = {24.0f, 24.0f};
  package_manager.list_width_fraction = 0.30f;
  package_manager.list_width_min = 220.0f;
  package_manager.list_width_max = 320.0f;
  settings.runtime_package_manager = package_manager;

  EditorProjectBrowserLayoutSettings project_browser;
  project_browser.hierarchy_width = 320.0f;
  project_browser.reveal_folder = std::filesystem::path("Models") / "Sponza" / "textures";
  settings.project_browser = project_browser;

  return settings;
}

void DisablePostProcessing(const std::shared_ptr<Scene>& scene) {
  if (const auto main_camera = scene->main_camera.Get<Camera>()) {
    main_camera->post_processing_stack_ref.Clear();
  }
  if (const auto* camera_owners = scene->UnsafeGetPrivateComponentOwnersList<Camera>()) {
    for (const auto& owner : *camera_owners) {
      if (const auto camera = scene->GetOrSetPrivateComponent<Camera>(owner).lock()) {
        camera->post_processing_stack_ref.Clear();
      }
    }
  }
}

float NonNegative(const float value) {
  return value < 0.0f ? 0.0f : value;
}
}  // namespace

const std::vector<DemoProfileDescriptor>& GetDemoProfiles() {
  static const std::vector<DemoProfileDescriptor> profiles = {
      {DemoProfileId::Rendering,
       "rendering",
       "Rendering",
       "DemoApp",
       "Lighting, deferred rendering, DDGI, and material stress scene.",
       "Launcher/DemoPreviews/rendering.png",
       ApplicationMode::Editor,
       {ApplicationMode::Editor},
       {}},
      {DemoProfileId::RenderingRegression,
       "rendering-regression",
       "Rendering Regression",
       "EvoEngineEditor",
       "Cross-technique regression scene for materials, lights, skinning, and path tracing controls.",
       "Launcher/DemoPreviews/rendering.png",
       ApplicationMode::Editor,
       {ApplicationMode::Editor},
       {}},
      {DemoProfileId::ProceduralGalaxy,
       "procedural-galaxy",
       "Procedural Galaxy",
       "Universe",
       "Generated 50,000-star galaxy project with the Universe runtime package loaded.",
       "Launcher/DemoPreviews/procedural-galaxy.png",
       ApplicationMode::Editor,
       {ApplicationMode::Editor},
       {"Universe"}},
      {DemoProfileId::GaussianSplat,
       "3dgs",
       "3D Gaussian Splatting",
       "EvoEngineEditor",
       "Spatial Dragon Gaussian splat demo scene.",
       "Launcher/DemoPreviews/3dgs.png",
       ApplicationMode::Editor,
       {ApplicationMode::Editor},
       {}},
      {DemoProfileId::Bicycle,
       "bicycle",
       "Bicycle",
       "EvoEngineEditor",
       "INRIA Bicycle Gaussian splat demo scene.",
       "Launcher/DemoPreviews/bicycle.png",
       ApplicationMode::Editor,
       {ApplicationMode::Editor},
       {}},
      {DemoProfileId::Bistro,
       "bistro",
       "Bistro",
       "EvoEngineEditor",
       "Amazon Lumberyard Bistro glTF scene for path tracing validation.",
       "Launcher/DemoPreviews/bistro.png",
       ApplicationMode::Editor,
       {ApplicationMode::Editor},
       {}},
      {DemoProfileId::EcoSysLab,
       "ecosyslab",
       "EcoSysLab",
       "EcoSysLabApp",
       "EcoSysLab project with its runtime package loaded.",
       "Launcher/DemoPreviews/ecosyslab.png",
       ApplicationMode::Editor,
       {ApplicationMode::Editor},
       {"EcoSysLab"}},
      {DemoProfileId::LSystem,
       "lsystem",
       "LSystem",
       "LSystemApp",
       "LSystem project with DigitalAgriculture compatibility package loaded.",
       "Launcher/DemoPreviews/lsystem.png",
       ApplicationMode::Editor,
       {ApplicationMode::Editor},
       {"LSystem", "DigitalAgriculture"}},
      {DemoProfileId::DigitalAgriculture,
       "digital-agriculture",
       "DigitalAgriculture",
       "DigitalAgricultureApp",
       "Digital Agriculture project with its runtime package loaded.",
       "Launcher/DemoPreviews/digital-agriculture.png",
       ApplicationMode::Editor,
       {ApplicationMode::Editor},
       {"DigitalAgriculture"}},

      {DemoProfileId::Ddgi,
       "ddgi",
       "DDGI",
       "DDGIApp",
       "Cornell Box DDGI scene.",
       "Launcher/DemoPreviews/ddgi.png",
       ApplicationMode::Editor,
       {ApplicationMode::Editor},
       {}},

  };
  return profiles;
}

const DemoProfileDescriptor* FindDemoProfile(const std::string_view id_name) {
  const auto& profiles = GetDemoProfiles();
  for (const auto& profile : profiles) {
    if (std::string_view(profile.id_name) == id_name) {
      return &profile;
    }
  }
  return nullptr;
}

const DemoProfileDescriptor& GetDemoProfile(const DemoProfileId id) {
  const auto& profiles = GetDemoProfiles();
  for (const auto& profile : profiles) {
    if (profile.id == id) {
      return profile;
    }
  }
  return profiles.front();
}

const char* GetDemoProfileIdName(const DemoProfileId id) {
  return GetDemoProfile(id).id_name;
}

bool IsDemoProfileApplicationModeSupported(const DemoProfileId id, const ApplicationMode mode) {
  const auto& profile = GetDemoProfile(id);
  return std::find(profile.supported_application_modes.begin(), profile.supported_application_modes.end(), mode) !=
         profile.supported_application_modes.end();
}

std::filesystem::path FindDemoProfileResourcesRoot(const std::filesystem::path& preferred_root) {
  return ExistingResourcesRoot(preferred_root);
}

std::filesystem::path ResolveDemoProfileProjectPath(const DemoProfileId id,
                                                    const std::filesystem::path& preferred_resource_root) {
  const auto resource_root = StableResourcesRoot(preferred_resource_root);
  switch (id) {
    case DemoProfileId::Rendering:
      return path_utils::NormalizeAbsolutePath(resource_root / "EvoEngine-DemoProjects" / "Rendering" /
                                               "Rendering.eveproj");
    case DemoProfileId::RenderingRegression:
      return path_utils::NormalizeAbsolutePath(resource_root / ".generated" / "EvoEngine-DemoProjects" /
                                               "RenderingRegression" / "RenderingRegression.eveproj");
    case DemoProfileId::Ddgi:
      return path_utils::NormalizeAbsolutePath(resource_root / "EvoEngine-DemoProjects" / "CornellBox" /
                                               "CornellBox.eveproj");
    case DemoProfileId::EcoSysLab:
      return path_utils::NormalizeAbsolutePath(resource_root / "EcoSysLabProject" / "test.eveproj");
    case DemoProfileId::DigitalAgriculture:
      return path_utils::NormalizeAbsolutePath(resource_root / "DigitalAgricultureProject" / "test.eveproj");
    case DemoProfileId::LSystem: {
      const std::array<std::filesystem::path, 2> candidates = {resource_root / "LSystemProject" / "test.eveproj",
                                                               resource_root / "LSystemProjectAssets" / "test.eveproj"};
      for (const auto& candidate : candidates) {
        if (std::filesystem::exists(candidate)) {
          return path_utils::NormalizeAbsolutePath(candidate);
        }
      }
      return path_utils::NormalizeAbsolutePath(candidates.front());
    }
    case DemoProfileId::ProceduralGalaxy:
      return path_utils::NormalizeAbsolutePath(resource_root / "EvoEngine-DemoProjects" / "Universe" /
                                               "ProceduralGalaxy.eveproj");
    case DemoProfileId::GaussianSplat:
      return path_utils::NormalizeAbsolutePath(resource_root / "EvoEngine-DemoProjects" / "3DGS" / "3DGS.eveproj");
    case DemoProfileId::Bicycle:
      return path_utils::NormalizeAbsolutePath(resource_root / "EvoEngine-DemoProjects" / "Bicycle" /
                                               "Bicycle.eveproj");
    case DemoProfileId::Bistro:
      return path_utils::NormalizeAbsolutePath(resource_root / ".generated" / "EvoEngine-DemoProjects" / "Bistro" /
                                               "Bistro.eveproj");
  }
  return {};
}

std::vector<std::string> MissingDemoProfileResourceRequirements(const DemoProfileId id,
                                                                const std::filesystem::path& preferred_resource_root) {
  std::vector<std::string> missing;
  const auto resource_root = FindDemoProfileResourcesRoot(preferred_resource_root);
  if (resource_root.empty()) {
    missing.emplace_back("Resources folder");
    return missing;
  }
  if (id == DemoProfileId::Rendering || id == DemoProfileId::RenderingRegression || id == DemoProfileId::Ddgi ||
      id == DemoProfileId::ProceduralGalaxy) {
    return missing;
  }
  if (id == DemoProfileId::GaussianSplat) {
    const auto asset_path =
        resource_root / "EvoEngine-DemoProjects" / "3DGS" / "Assets" / "GaussianSplats" / "spatial_dragon.ply";
    const auto asset_metadata_path = std::filesystem::path(asset_path.string() + ".evefilemeta");
    if (!std::filesystem::exists(asset_path) || std::filesystem::is_directory(asset_path)) {
      missing.emplace_back("spatial_dragon.ply");
    }
    if (!std::filesystem::exists(asset_metadata_path) || std::filesystem::is_directory(asset_metadata_path)) {
      missing.emplace_back("spatial_dragon.ply.evefilemeta");
    }
    return missing;
  }
  if (id == DemoProfileId::Bicycle) {
    const auto demo_root = resource_root / "EvoEngine-DemoProjects" / "Bicycle" / "Assets";
    const auto asset_path = demo_root / "GaussianSplats" / "bicycle.ply";
    const auto asset_metadata_path = std::filesystem::path(asset_path.string() + ".evefilemeta");
    if (!std::filesystem::exists(asset_path) || std::filesystem::is_directory(asset_path)) {
      missing.emplace_back("bicycle.ply");
    }
    if (!std::filesystem::exists(asset_metadata_path) || std::filesystem::is_directory(asset_metadata_path)) {
      missing.emplace_back("bicycle.ply.evefilemeta");
    }
    return missing;
  }
  if (id == DemoProfileId::Bistro) {
    const auto demo_root = resource_root / ".generated" / "EvoEngine-DemoProjects" / "Bistro";
    const auto asset_root = demo_root / "Assets" / "Models" / "Bistro";
    const auto asset_path = asset_root / "bistro.gltf";
    const auto asset_metadata_path = std::filesystem::path(asset_path.string() + ".evefilemeta");
    if (!std::filesystem::exists(demo_root / "Bistro.eveproj") ||
        std::filesystem::is_directory(demo_root / "Bistro.eveproj")) {
      missing.emplace_back("Bistro.eveproj");
    }
    if (!std::filesystem::exists(asset_path) || std::filesystem::is_directory(asset_path)) {
      missing.emplace_back("bistro.gltf");
    }
    if (!std::filesystem::exists(asset_root / "bistro.bin") ||
        std::filesystem::is_directory(asset_root / "bistro.bin")) {
      missing.emplace_back("bistro.bin");
    }
    if (!std::filesystem::exists(asset_root / "textures") || !std::filesystem::is_directory(asset_root / "textures")) {
      missing.emplace_back("textures");
    }
    if (!std::filesystem::exists(asset_root / "objects") || !std::filesystem::is_directory(asset_root / "objects")) {
      missing.emplace_back("objects");
    }
    if (!std::filesystem::exists(asset_metadata_path) || std::filesystem::is_directory(asset_metadata_path)) {
      missing.emplace_back("bistro.gltf.evefilemeta");
    }
    return missing;
  }

  const auto project_path = ResolveDemoProfileProjectPath(id, resource_root);
  if (!std::filesystem::exists(project_path) || std::filesystem::is_directory(project_path)) {
    missing.emplace_back(project_path.filename().string());
  }
  return missing;
}

void NormalizeLegacyResourceExtensions(const std::filesystem::path& resource_root) {
  if (resource_root.empty() || !std::filesystem::exists(resource_root)) {
    return;
  }
  for (const auto& entry : std::filesystem::recursive_directory_iterator(resource_root)) {
    if (entry.is_directory()) {
      continue;
    }
    const auto old_path = entry.path();
    auto new_path = old_path;
    bool replace = false;
    if (old_path.extension() == ".uescene") {
      new_path.replace_extension(".evescene");
      replace = true;
    } else if (old_path.extension() == ".umeta") {
      new_path.replace_extension(".evefilemeta");
      replace = true;
    } else if (old_path.extension() == ".ueproj") {
      new_path.replace_extension(".eveproj");
      replace = true;
    } else if (old_path.extension() == ".ufmeta") {
      new_path.replace_extension(".evefoldermeta");
      replace = true;
    }
    if (replace) {
      std::filesystem::copy(old_path, new_path);
      std::filesystem::remove(old_path);
    }
  }
}

void ApplyRenderingDemoEditorSetup() {
  const auto editor_layer = ApplicationContext::Get().GetLayer<EditorLayer>();
  if (editor_layer) {
    editor_layer->RequestEditorLayout(CreateRenderingDemoEditorLayout());
  }
  PrepareRenderingDemoShowcase(editor_layer);
}

void ConfigureDdgiCornellBoxApplication(ApplicationInitializationSettings& application_info,
                                        const ApplicationMode application_mode) {
  application_info.application_mode = application_mode;
  application_info.application_name = "DDGI Cornell Box";
  application_info.default_window_size = kDdgiCornellBoxExtent;
  application_info.use_custom_title_bar = false;
}

void ConfigureDdgiCornellBoxScene(const std::shared_ptr<Scene>& scene, const DdgiCornellBoxDemoSettings& settings) {
  if (!scene) {
    return;
  }
  DisablePostProcessing(scene);

  scene->environment.environment_type = Scene::EnvironmentType::Color;
  scene->environment.background_color = glm::vec3(0.0f);
  scene->environment.background_intensity = 0.0f;
  scene->environment.ambient_light_intensity = 0.0f;

  auto& ddgi_settings = scene->environment.ddgi_settings;
  ddgi_settings.runtime.enabled = true;
  ddgi_settings.runtime.pause_updates = false;
  ddgi_settings.runtime.ray_count = 256;
  ddgi_settings.runtime.normal_bias = kDdgiCornellBoxNormalBias;
  ddgi_settings.runtime.view_bias = kDdgiCornellBoxViewBias;
  ddgi_settings.runtime.reset_probe_history = true;
  ddgi_settings.runtime.indirect_intensity = NonNegative(settings.ddgi_indirect_intensity);
  ddgi_settings.storage.max_probe_count =
      kDdgiCornellBoxProbeCounts.x * kDdgiCornellBoxProbeCounts.y * kDdgiCornellBoxProbeCounts.z;
  ddgi_settings.debug.enabled = false;
  ddgi_settings.debug.visualize_volume_bounds = false;
  ddgi_settings.debug.visualize_probe_positions = false;
  ddgi_settings.debug.visualize_selected_probe = false;
  ddgi_settings.debug.visualize_probe_state = false;
  ddgi_settings.debug.visualize_probe_illumination = false;
  ddgi_settings.debug.show_atlas_preview = false;
  ddgi_settings.debug.show_update_age = false;
  ddgi_settings.debug.show_rays = false;
  ddgi_settings.debug.show_irradiance = false;
  ddgi_settings.debug.show_visibility = false;
  ddgi_settings.debug.show_sampling_weights = false;

  if (const auto main_camera = scene->main_camera.Get<Camera>()) {
    main_camera->Resize(kDdgiCornellBoxExtent);
    main_camera->skybox.Clear();
    main_camera->camera_settings.use_clear_color = true;
    main_camera->camera_settings.clear_color = glm::vec4(0.0f, 0.0f, 0.0f, 1.0f);
    main_camera->camera_settings.background_intensity = 0.0f;

    auto camera_transform = scene->GetDataComponent<Transform>(main_camera->GetOwner());
    camera_transform.SetPosition(kDdgiCornellBoxCameraPosition);
    scene->SetDataComponent(main_camera->GetOwner(), camera_transform);
  }

  if (const auto* volume_owners = scene->UnsafeGetPrivateComponentOwnersList<DdgiVolume>()) {
    for (const auto& owner : *volume_owners) {
      if (const auto volume = scene->GetOrSetPrivateComponent<DdgiVolume>(owner).lock()) {
        volume->visualize_bounds = false;
        volume->visualize_probe_positions = false;
        volume->probe_counts = kDdgiCornellBoxProbeCounts;
        volume->probe_spacing = glm::vec3(kDdgiCornellBoxProbeSpacing);
        volume->volume_origin = kDdgiCornellBoxVolumeOrigin;
        volume->relocation_distance = kDdgiCornellBoxProbeSpacing * 0.5f;
        volume->enable_probe_relocation = settings.enable_probe_relocation;
        volume->enable_probe_classification = settings.enable_probe_classification;
        volume->max_visualized_probes = 0;
        volume->ClampSettings();
      }
    }
  }

  if (const auto* point_light_owners = scene->UnsafeGetPrivateComponentOwnersList<PointLight>()) {
    for (const auto& owner : *point_light_owners) {
      if (const auto point_light = scene->GetOrSetPrivateComponent<PointLight>(owner).lock()) {
        point_light->diffuse_brightness = NonNegative(settings.point_light_brightness);
      }
    }
  }

  if (const auto* mesh_renderer_owners = scene->UnsafeGetPrivateComponentOwnersList<MeshRenderer>()) {
    for (const auto& owner : *mesh_renderer_owners) {
      if (scene->GetEntityName(owner) != "Ceiling Light Mesh") {
        continue;
      }
      const auto mesh_renderer = scene->GetOrSetPrivateComponent<MeshRenderer>(owner).lock();
      const auto material = mesh_renderer ? mesh_renderer->material.Get<Material>() : nullptr;
      if (material) {
        material->material_data.shade_material.emissive_factor = glm::vec3(kDdgiCornellBoxCeilingLightEmission);
        material->MarkDirty();
      }
    }
  }
}
}  // namespace evo_engine
