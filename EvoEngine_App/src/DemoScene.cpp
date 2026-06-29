#include "DemoScene.hpp"

#include "AnimationPlayer.hpp"
#include "Application.hpp"
#include "DdgiVolume.hpp"
#include "EditorLayer.hpp"
#include "GaussianSplat.hpp"
#include "GaussianSplatRenderer.hpp"
#include "Json.hpp"
#include "Lights.hpp"
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

#include <unordered_set>

using namespace evo_engine;

namespace {
constexpr uint64_t kSpatialDragonGaussianSplatHandle = 9739484957885691067ull;
constexpr uint64_t kBicycleGaussianSplatHandle = 14453709846752502031ull;
constexpr uint64_t kBicycleCamerasHandle = 6692924907754216903ull;
// VK's benchmark preset 1 maps to the first imported INRIA camera because preset 0 is VK's default camera.
constexpr int kBicycleDemoCameraId = 0;

struct GaussianSplatDemoCameraPose {
  glm::vec3 position = glm::vec3(0.0f);
  glm::quat rotation = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
  float fov = 55.0f;
};

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
          material->material_properties.roughness = static_cast<float>(i) / (amount - 1);
          material->material_properties.metallic = static_cast<float>(j) / (amount - 1);
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
  title_material->material_properties.emission = 4;
  title_material->material_properties.albedo_color = glm::vec3(1, 0.2f, 0.5f);

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
  capoeira_body_material->material_properties.albedo_color = glm::vec3(0, 1, 1);
  capoeira_body_material->material_properties.metallic = 1;
  capoeira_body_material->material_properties.roughness = 0;
  const auto capoeira_joints_material =
      scene
          ->GetOrSetPrivateComponent<SkinnedMeshRenderer>(scene->GetChildren(scene->GetChildren(capoeira_entity)[0])[0])
          .lock()
          ->material.Get<Material>();
  capoeira_joints_material->material_properties.albedo_color = glm::vec3(0.3f, 1.0f, 0.5f);
  capoeira_joints_material->material_properties.metallic = 1;
  capoeira_joints_material->material_properties.roughness = 0;
  capoeira_joints_material->material_properties.emission = 6;
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
  material->material_properties.albedo_color = albedo;
  material->material_properties.roughness = 0.85f;
  material->material_properties.metallic = 0.0f;
  material->material_properties.emission = emission;
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

Entity GetOrCreateRayTracingTlasSeedEntity(const std::shared_ptr<Scene>& scene) {
  if (const auto* owners = scene->UnsafeGetPrivateComponentOwnersList<MeshRenderer>()) {
    for (const auto& owner : *owners) {
      if (scene->IsEntityValid(owner) && scene->GetEntityName(owner) == "Ray Tracing TLAS Seed") {
        return owner;
      }
    }
  }
  return scene->CreateEntity("Ray Tracing TLAS Seed");
}

std::optional<GaussianSplatDemoCameraPose> LoadGaussianSplatDemoCameraPose(const uint64_t cameras_handle,
                                                                           const glm::vec3& center,
                                                                           const int preferred_camera_id) {
  std::shared_ptr<Json> cameras_asset;
  try {
    cameras_asset = AssetManager::GetAsset<Json>(Handle(cameras_handle));
  } catch (const std::exception& error) {
    EVOENGINE_WARNING("Failed to load Gaussian splat demo camera JSON: " + std::string(error.what()))
    return {};
  }
  if (!cameras_asset || !cameras_asset->m_json.is_array() || cameras_asset->m_json.empty()) {
    return {};
  }

  const nlohmann::json* selected_camera = &cameras_asset->m_json.front();
  for (const auto& camera : cameras_asset->m_json) {
    if (camera.value("id", -1) == preferred_camera_id) {
      selected_camera = &camera;
      break;
    }
  }

  try {
    const auto& position = selected_camera->at("position");
    const auto& rotation = selected_camera->at("rotation");
    GaussianSplatDemoCameraPose pose;
    pose.position =
        glm::vec3(position.at(0).get<float>(), position.at(1).get<float>(), position.at(2).get<float>()) - center;
    // EvoEngine keeps the INRIA PLY coordinates unflipped; invert the image-down axis for editor/main camera up.
    const auto front = glm::normalize(glm::vec3(rotation.at(0).at(2).get<float>(), rotation.at(1).at(2).get<float>(),
                                                rotation.at(2).at(2).get<float>()));
    const auto up = glm::normalize(-glm::vec3(rotation.at(0).at(1).get<float>(), rotation.at(1).at(1).get<float>(),
                                              rotation.at(2).at(1).get<float>()));
    pose.rotation = glm::quatLookAt(front, up);

    const float height = selected_camera->value("height", 0.0f);
    const float fy = selected_camera->value("fy", 0.0f);
    if (height > 0.0f && fy > 0.0f) {
      pose.fov = glm::clamp(glm::degrees(4.0f * std::atan(height / (2.0f * fy))), 20.0f, 120.0f);
    }
    return pose;
  } catch (const std::exception& error) {
    EVOENGINE_WARNING("Failed to parse Gaussian splat demo camera JSON: " + std::string(error.what()))
    return {};
  }
}

void ConfigureGaussianSplatDemoSceneImpl(const std::shared_ptr<Scene>& scene, const uint64_t gaussian_splat_handle,
                                         const char* asset_name, const char* entity_name, const int sh_degree,
                                         const std::optional<uint64_t> cameras_handle = {},
                                         const int preferred_camera_id = 0) {
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
  if (cameras_handle) {
    if (const auto camera_pose = LoadGaussianSplatDemoCameraPose(*cameras_handle, center, preferred_camera_id)) {
      camera_position = camera_pose->position;
      camera_rotation = camera_pose->rotation;
      camera_fov = camera_pose->fov;
    }
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
  gaussian_renderer->sort_mode = GaussianSplatSortMode::CpuDepth;
  gaussian_renderer->depth_mode = GaussianSplatDepthMode::SceneDepth;
  Transform gaussian_transform;
  gaussian_transform.SetPosition(-center);
  scene->SetDataComponent(gaussian_entity, gaussian_transform);

  const auto tlas_seed_entity = GetOrCreateRayTracingTlasSeedEntity(scene);
  const auto tlas_seed_renderer = scene->GetOrSetPrivateComponent<MeshRenderer>(tlas_seed_entity).lock();
  tlas_seed_renderer->mesh = Resources::GetInstance().GetPrimitives().cube;
  auto tlas_seed_material = tlas_seed_renderer->material.Get<Material>();
  if (!tlas_seed_material) {
    tlas_seed_material = AssetManager::CreateTemporaryAsset<Material>();
    tlas_seed_renderer->material.Set<Material>(tlas_seed_material);
  }
  tlas_seed_material->material_properties.albedo_color = glm::vec3(0.0f);
  tlas_seed_material->material_properties.roughness = 1.0f;
  tlas_seed_material->material_properties.metallic = 0.0f;
  Transform tlas_seed_transform;
  tlas_seed_transform.SetPosition(camera_position + camera_rotation * glm::vec3(0.0f, 0.0f, radius * 100.0f));
  tlas_seed_transform.SetScale(glm::vec3(glm::max(radius * 0.001f, 0.001f)));
  scene->SetDataComponent(tlas_seed_entity, tlas_seed_transform);

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
      scene_camera->ResetFrameCount();
    }
  }

  scene->Save();
  ProjectManager::SaveProject();
}
}  // namespace

void evo_engine::ConfigureGaussianSplatDemoScene(const std::shared_ptr<Scene>& scene) {
  ConfigureGaussianSplatDemoSceneImpl(scene, kSpatialDragonGaussianSplatHandle, "Spatial Dragon", "Spatial Dragon 3DGS",
                                      0);
}

void evo_engine::ConfigureBicycleDemoScene(const std::shared_ptr<Scene>& scene) {
  ConfigureGaussianSplatDemoSceneImpl(scene, kBicycleGaussianSplatHandle, "Bicycle", "Bicycle 3DGS", 3,
                                      kBicycleCamerasHandle, kBicycleDemoCameraId);
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

        const auto left_point_light_right_entity = scene->CreateEntity("Left Point Light");
        const auto point_light_right_renderer =
            scene->GetOrSetPrivateComponent<MeshRenderer>(left_point_light_right_entity).lock();
        point_light_right_renderer->cast_shadow = false;
        const auto point_light_right_material = AssetManager::CreateTemporaryAsset<Material>();
        point_light_right_renderer->material.Set<Material>(point_light_right_material);
        point_light_right_material->material_properties.albedo_color = glm::vec3(1.0f, 0.8f, 0.0f);
        point_light_right_material->material_properties.emission = 2.0f;
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
    case DemoSetup::Universe:
    case DemoSetup::Empty:
    default: {
    } break;
  }
}
