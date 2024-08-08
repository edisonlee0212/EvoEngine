#include "AnimationPlayer.hpp"
#include "Application.hpp"
#include "ClassRegistry.hpp"

#include "EditorLayer.hpp"
#include "MeshRenderer.hpp"
#include "PlanetTerrainSystem.hpp"
#include "StarClusterSystem.hpp"
#include "PerlinNoiseStage.hpp"
#include "PlayerController.hpp"
#include "Prefab.hpp"
#include "RenderLayer.hpp"
#include "Times.hpp"
#include "WindowLayer.hpp"

#include "PostProcessingStack.hpp"

#ifdef OPTIX_RAY_TRACER_PLUGIN
#include <CUDAModule.hpp>
#include <RayTracerLayer.hpp>
#endif
#ifdef PHYSICS_PLUGIN
#include "PhysicsLayer.hpp"
#include "RigidBody.hpp"
#ifdef RAY_TRACER_PLUGIN
#include "CpuRayTracerCamera.hpp"
#endif

#endif
using namespace evo_engine;
using namespace planet;
using namespace Galaxy;
#pragma region Helpers
#ifdef PHYSICS_PLUGIN
Entity CreateDynamicCube(const float& mass, const glm::vec3& color, const glm::vec3& position,
                         const glm::vec3& rotation, const glm::vec3& scale, const std::string& name);

Entity CreateSolidCube(const float& mass, const glm::vec3& color, const glm::vec3& position, const glm::vec3& rotation,
                       const glm::vec3& scale, const std::string& name);

Entity CreateCube(const glm::vec3& color, const glm::vec3& position, const glm::vec3& rotation, const glm::vec3& scale,
                  const std::string& name);

Entity CreateDynamicSphere(const float& mass, const glm::vec3& color, const glm::vec3& position,
                           const glm::vec3& rotation, const float& scale, const std::string& name);

Entity CreateSolidSphere(const float& mass, const glm::vec3& color, const glm::vec3& position,
                         const glm::vec3& rotation, const float& scale, const std::string& name);

Entity CreateSphere(const glm::vec3& color, const glm::vec3& position, const glm::vec3& rotation, const float& scale,
                    const std::string& name);
#endif

enum class DemoSetup { Empty, Rendering, Galaxy, Planets };
Entity LoadScene(const std::shared_ptr<Scene>& scene, const std::string& base_entity_name, bool add_spheres);
void SetupDemoScene(DemoSetup demo_setup, ApplicationInfo& application_info);
Entity LoadPhysicsScene(const std::shared_ptr<Scene>& scene, const std::string& base_entity_name);
#pragma endregion

int main() {
  constexpr DemoSetup demo_setup = DemoSetup::Rendering;
  Application::PushLayer<WindowLayer>();
  Application::PushLayer<EditorLayer>();
  Application::PushLayer<RenderLayer>();

#ifdef OPTIX_RAY_TRACER_PLUGIN
  Application::PushLayer<RayTracerLayer>();
#endif
#ifdef PHYSICS_PLUGIN
  Application::PushLayer<PhysicsLayer>();
#endif
  
#ifdef RAY_TRACER_PLUGIN
  PrivateComponentRegistration<CpuRayTracerCamera>("CpuRayTracerCamera");
#endif
  ApplicationInfo application_info;
  SetupDemoScene(demo_setup, application_info);

  Application::Initialize(application_info);

#ifdef OPTIX_RAY_TRACER_PLUGIN
  const auto ray_tracer_layer = Application::GetLayer<RayTracerLayer>();
  ray_tracer_layer->show_camera_window = false;
  ray_tracer_layer->show_scene_window = false;
  ray_tracer_layer->show_ray_tracer_settings_window = false;
#endif

  Application::Start();
  Application::Run();
  Application::Terminate();
  return 0;
}
#pragma region Helpers
Entity LoadScene(const std::shared_ptr<Scene>& scene, const std::string& base_entity_name, bool add_spheres) {
  auto base_entity = scene->CreateEntity(base_entity_name);

  if (add_spheres) {
#pragma region Create 9 spheres in different PBR properties
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
          mesh_renderer->mesh = Resources::GetResource<Mesh>("PRIMITIVE_SPHERE");
          const auto material = ProjectManager::CreateTemporaryAsset<Material>();
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
#pragma endregion
  }

#pragma region Create ground
  auto ground = scene->CreateEntity("Ground");
  std::shared_ptr<MeshRenderer> ground_mesh_renderer;
  ground_mesh_renderer = scene->GetOrSetPrivateComponent<MeshRenderer>(ground).lock();
  auto ground_mat = ProjectManager::CreateTemporaryAsset<Material>();

  ground_mesh_renderer->material = ground_mat;
  ground_mesh_renderer->mesh = Resources::GetResource<Mesh>("PRIMITIVE_CUBE");
  Transform ground_transform;
  ground_transform.SetValue(glm::vec3(0, -2.05, -0), glm::vec3(0), glm::vec3(30, 1, 60));
  scene->SetDataComponent(ground, ground_transform);
  scene->SetParent(ground, base_entity);
#pragma endregion
#pragma region Load models and display
  const auto sponza =
      std::dynamic_pointer_cast<Prefab>(ProjectManager::GetOrCreateAsset("Models/Sponza_FBX/Sponza.fbx"));
  const auto sponza_entity = sponza->ToEntity(scene);
  Transform sponza_transform;
  sponza_transform.SetValue(glm::vec3(0, -1.5, -6), glm::radians(glm::vec3(0, -90, 0)), glm::vec3(0.01));
  scene->SetDataComponent(sponza_entity, sponza_transform);
  scene->SetParent(sponza_entity, base_entity);

  auto title = std::dynamic_pointer_cast<Prefab>(ProjectManager::GetOrCreateAsset("Models/UniEngine.obj"));
  auto title_entity = title->ToEntity(scene);
  scene->SetEntityName(title_entity, "Title");
  Transform title_transform;
  title_transform.SetValue(glm::vec3(0.35, 7, -16), glm::radians(glm::vec3(0, 0, 0)), glm::vec3(0.005));
  scene->SetDataComponent(title_entity, title_transform);
  scene->SetParent(title_entity, base_entity);

  auto title_material =
      scene->GetOrSetPrivateComponent<MeshRenderer>(scene->GetChildren(scene->GetChildren(title_entity)[0])[0])
          .lock()
          ->material.Get<Material>();
  title_material->material_properties.emission = 4;
  title_material->material_properties.albedo_color = glm::vec3(1, 0.2, 0.5);

  auto dancing_storm_trooper = std::dynamic_pointer_cast<Prefab>(
      ProjectManager::GetOrCreateAsset("Models/dancing-stormtrooper/silly_dancing.fbx"));
  auto dancing_storm_trooper_entity = dancing_storm_trooper->ToEntity(scene);
  const auto dancing_storm_trooper_animation_player =
      scene->GetOrSetPrivateComponent<AnimationPlayer>(dancing_storm_trooper_entity).lock();
  dancing_storm_trooper_animation_player->auto_play = true;
  dancing_storm_trooper_animation_player->auto_play_speed = 30;
  scene->SetEntityName(dancing_storm_trooper_entity, "StormTrooper");
  Transform dancing_storm_trooper_transform;
  dancing_storm_trooper_transform.SetValue(glm::vec3(1.2, -1.5, 0), glm::vec3(0), glm::vec3(0.4));
  scene->SetDataComponent(dancing_storm_trooper_entity, dancing_storm_trooper_transform);
  scene->SetParent(dancing_storm_trooper_entity, base_entity);

  const auto capoeira = std::dynamic_pointer_cast<Prefab>(ProjectManager::GetOrCreateAsset("Models/Capoeira.fbx"));
  const auto capoeira_entity = capoeira->ToEntity(scene);
  const auto capoeira_animation_player = scene->GetOrSetPrivateComponent<AnimationPlayer>(capoeira_entity).lock();
  capoeira_animation_player->auto_play = true;
  capoeira_animation_player->auto_play_speed = 60;
  scene->SetEntityName(capoeira_entity, "Capoeira");
  Transform capoeira_transform;
  capoeira_transform.SetValue(glm::vec3(0.5, 2.7, -18), glm::vec3(0), glm::vec3(0.02));
  scene->SetDataComponent(capoeira_entity, capoeira_transform);
  auto capoeira_body_material =
      scene
          ->GetOrSetPrivateComponent<SkinnedMeshRenderer>(scene->GetChildren(scene->GetChildren(capoeira_entity)[1])[0])
          .lock()
          ->material.Get<Material>();
  capoeira_body_material->material_properties.albedo_color = glm::vec3(0, 1, 1);
  capoeira_body_material->material_properties.metallic = 1;
  capoeira_body_material->material_properties.roughness = 0;
  auto capoeira_joints_material =
      scene
          ->GetOrSetPrivateComponent<SkinnedMeshRenderer>(scene->GetChildren(scene->GetChildren(capoeira_entity)[0])[0])
          .lock()
          ->material.Get<Material>();
  capoeira_joints_material->material_properties.albedo_color = glm::vec3(0.3, 1.0, 0.5);
  capoeira_joints_material->material_properties.metallic = 1;
  capoeira_joints_material->material_properties.roughness = 0;
  capoeira_joints_material->material_properties.emission = 6;
  scene->SetParent(capoeira_entity, base_entity);

#pragma endregion

  return base_entity;
}

void SetupDemoScene(DemoSetup demo_setup, ApplicationInfo& application_info) {
  std::filesystem::path resource_folder_path("../../../../../Resources");
  if (!std::filesystem::exists(resource_folder_path)) {
    resource_folder_path = "../../../../Resources";
  }
  if (!std::filesystem::exists(resource_folder_path)) {
    resource_folder_path = "../../../Resources";
  }
  if (!std::filesystem::exists(resource_folder_path)) {
    resource_folder_path = "../../Resources";
  }
  if (!std::filesystem::exists(resource_folder_path)) {
    resource_folder_path = "../Resources";
  }
#pragma region Demo scene setup
  if (demo_setup != DemoSetup::Empty && std::filesystem::exists(resource_folder_path)) {
    for (const auto i : std::filesystem::recursive_directory_iterator(resource_folder_path / "Example Projects")) {
      if (i.is_directory())
        continue;
      if (i.path().extension().string() == ".evescene" || i.path().extension().string() == ".evefilemeta" ||
          i.path().extension().string() == ".eveproj" || i.path().extension().string() == ".evefoldermeta") {
        std::filesystem::remove(i.path());
      }
    }
    for (const auto i : std::filesystem::recursive_directory_iterator(resource_folder_path)) {
      if (i.is_directory())
        continue;
      if (i.path().extension().string() == ".uescene" || i.path().extension().string() == ".umeta" ||
          i.path().extension().string() == ".ueproj" || i.path().extension().string() == ".ufmeta") {
        std::filesystem::remove(i.path());
      }
    }
  }
  switch (demo_setup) {
    case DemoSetup::Rendering: {
      application_info.application_name = "Rendering Demo";
      application_info.project_path = resource_folder_path / "Example Projects/Rendering/Rendering.eveproj";
      ProjectManager::SetActionAfterNewScene([&](const std::shared_ptr<Scene>& scene) {
        scene->environment.ambient_light_intensity = 0.1f;
#pragma region Set main camera to correct position and rotation
        const auto main_camera = scene->main_camera.Get<Camera>();
        main_camera->Resize({640, 480});
        main_camera->post_processing_stack = ProjectManager::CreateTemporaryAsset<PostProcessingStack>();
        const auto main_camera_entity = main_camera->GetOwner();
        auto main_camera_transform = scene->GetDataComponent<Transform>(main_camera_entity);
        main_camera_transform.SetPosition(glm::vec3(0, 0, 4));
        scene->SetDataComponent(main_camera_entity, main_camera_transform);
        auto camera = scene->GetOrSetPrivateComponent<Camera>(main_camera_entity).lock();
        scene->GetOrSetPrivateComponent<PlayerController>(main_camera_entity);
#pragma endregion

#ifdef PHYSICS_PLUGIN
        LoadScene(scene, "Rendering Demo", false);
        const auto physics_demo = LoadPhysicsScene(scene, "Physics Demo");
        Transform physics_demo_transform;
        physics_demo_transform.SetPosition(glm::vec3(-0.5f, -0.5f, -1.0f));
        scene->SetDataComponent(physics_demo, physics_demo_transform);
        scene->GetOrCreateSystem<PhysicsSystem>(0.0f);
#else
        LoadScene(scene, "Rendering Demo", true);
#endif

#pragma region Dynamic Lighting
        const auto point_light_right_entity = scene->CreateEntity("Left Point Light");
        const auto point_light_right_renderer =
            scene->GetOrSetPrivateComponent<MeshRenderer>(point_light_right_entity).lock();
        const auto point_light_right_material = ProjectManager::CreateTemporaryAsset<Material>();
        point_light_right_renderer->material.Set<Material>(point_light_right_material);
        point_light_right_material->material_properties.albedo_color = glm::vec3(1.0, 0.8, 0.0);
        point_light_right_material->material_properties.emission = 100.0f;
        point_light_right_renderer->mesh = Resources::GetResource<Mesh>("PRIMITIVE_SPHERE");
        const auto point_light_right = scene->GetOrSetPrivateComponent<PointLight>(point_light_right_entity).lock();
        point_light_right->diffuse_brightness = 4;
        point_light_right->light_size = 0.02f;
        point_light_right->linear = 0.5f;
        point_light_right->quadratic = 0.1f;
        point_light_right->diffuse = glm::vec3(1.0, 0.8, 0.0);

        Transform point_light_right_transform;
        point_light_right_transform.SetPosition(glm::vec3(4, 1.2, -5));
        point_light_right_transform.SetScale({1.0f, 1.0f, 1.0f});
        scene->SetDataComponent(point_light_right_entity, point_light_right_transform);

        Application::RegisterUpdateFunction([=]() {
          static bool last_frame_playing = false;
          if (!Application::IsPlaying()) {
            last_frame_playing = Application::IsPlaying();
            return;
          }
          const auto current_scene = Application::GetActiveScene();
          static float start_time;
          if (!last_frame_playing)
            start_time = Times::Now();
          const float current_time = Times::Now() - start_time;
          const float cos_time = glm::cos(current_time / 5.0f);
          // Transform dir_light_transform;
          // dir_light_transform.SetEulerRotation(glm::radians(glm::vec3(105.0f, current_time * 20, 0.0f)));
          Transform point_light_right_transform;
          point_light_right_transform.SetPosition(glm::vec3(4, 1.2, cos_time * 5 - 5));
          point_light_right_transform.SetScale({1.0f, 1.0f, 1.0f});
          point_light_right_transform.SetPosition(glm::vec3(4, 1.2, cos_time * 5 - 5));
          current_scene->SetDataComponent(point_light_right_entity, point_light_right_transform);

          last_frame_playing = Application::IsPlaying();
        });
#pragma endregion
      });
    } break;
    case DemoSetup::Galaxy: {
      application_info.application_name = "Galaxy Demo";
      SystemRegistration<StarClusterSystem>("StarClusterSystem");
      application_info.project_path = resource_folder_path / "Example Projects/Galaxy/Galaxy.eveproj";
      ProjectManager::SetActionAfterNewScene([&](const std::shared_ptr<Scene>& scene) {
        const auto main_camera = scene->main_camera.Get<Camera>();
        main_camera->Resize({640, 480});
        const auto main_camera_entity = main_camera->GetOwner();
        scene->GetOrSetPrivateComponent<PlayerController>(main_camera_entity);
#pragma region Star System
        auto star_cluster_system = scene->GetOrCreateSystem<StarClusterSystem>(SystemGroup::SimulationSystemGroup);
#pragma endregion
        main_camera->use_clear_color = true;
      });
    } break;
    case DemoSetup::Planets: {
      application_info.application_name = "Planets Demo";
      SystemRegistration<PlanetTerrainSystem>("PlanetTerrainSystem");
      PrivateComponentRegistration<PlanetTerrain>("PlanetTerrain");

      application_info.project_path = resource_folder_path / "Example Projects/Planet/Planet.eveproj";
      ProjectManager::SetActionAfterNewScene([&](const std::shared_ptr<Scene>& scene) {
#pragma region Preparations
        const auto main_camera = scene->main_camera.Get<Camera>();
        main_camera->Resize({640, 480});
        const auto main_camera_entity = main_camera->GetOwner();
        auto main_camera_transform = scene->GetDataComponent<Transform>(main_camera_entity);
        main_camera_transform.SetPosition(glm::vec3(0, -4, 25));
        scene->SetDataComponent(main_camera_entity, main_camera_transform);
        scene->GetOrSetPrivateComponent<PlayerController>(main_camera_entity);
        // auto postProcessing = scene->GetOrSetPrivateComponent<PostProcessing>(mainCameraEntity).lock();

        const auto surface_material = ProjectManager::CreateTemporaryAsset<Material>();
        const auto border_texture =
            std::dynamic_pointer_cast<Texture2D>(ProjectManager::GetOrCreateAsset("Textures/border.png"));
        surface_material->SetAlbedoTexture(border_texture);

        auto pts = scene->GetOrCreateSystem<planet::PlanetTerrainSystem>(SystemGroup::SimulationSystemGroup);

        pts->Enable();

        PlanetInfo pi;
        Transform planet_transform;

        planet_transform.SetPosition(glm::vec3(0.0f, 0.0f, 0.0f));
        planet_transform.SetEulerRotation(glm::vec3(0.0f));
        pi.max_lod_level = 8;
        pi.lod_distance = 7.0;
        pi.radius = 10.0;
        pi.index = 0;
        pi.resolution = 8;

        // Serialization not implemented.

        auto planet1 = scene->CreateEntity("Planet 1");
        const auto planet_terrain1 = scene->GetOrSetPrivateComponent<PlanetTerrain>(planet1).lock();
        planet_terrain1->terrain_construction_stages.emplace_back(std::make_shared<PerlinNoiseStage>());
        planet_terrain1->surface_material = surface_material;
        planet_terrain1->SetPlanetInfo(pi);
        scene->SetDataComponent(planet1, planet_transform);
        planet_transform.SetPosition(glm::vec3(35.0f, 0.0f, 0.0f));
        pi.max_lod_level = 20;
        pi.lod_distance = 7.0;
        pi.radius = 15.0;
        pi.index = 1;

        auto planet2 = scene->CreateEntity("Planet 2");
        const auto planet_terrain2 = scene->GetOrSetPrivateComponent<PlanetTerrain>(planet2).lock();
        planet_terrain2->surface_material = surface_material;
        planet_terrain2->SetPlanetInfo(pi);
        scene->SetDataComponent(planet2, planet_transform);
        planet_transform.SetPosition(glm::vec3(-20.0f, 0.0f, 0.0f));
        pi.max_lod_level = 4;
        pi.lod_distance = 7.0;
        pi.radius = 5.0;
        pi.index = 2;

        auto planet3 = scene->CreateEntity("Planet 3");
        const auto planet_terrain3 = scene->GetOrSetPrivateComponent<PlanetTerrain>(planet3).lock();
        planet_terrain3->surface_material = surface_material;
        planet_terrain3->SetPlanetInfo(pi);
        scene->SetDataComponent(planet3, planet_transform);
#pragma endregion

#pragma region Lights
        const auto shared_mat = ProjectManager::CreateTemporaryAsset<Material>();
        Transform ltw;

        Entity dle = scene->CreateEntity("Directional Light");
        auto dlc = scene->GetOrSetPrivateComponent<DirectionalLight>(dle).lock();
        dlc->diffuse = glm::vec3(1.0f);
        ltw.SetScale(glm::vec3(0.5f));

        Entity ple = scene->CreateEntity("Point Light 1");
        auto plmmc = scene->GetOrSetPrivateComponent<MeshRenderer>(ple).lock();
        plmmc->mesh = Resources::GetResource<Mesh>("PRIMITIVE_SPHERE");
        plmmc->material.Set<Material>(shared_mat);
        auto plc = scene->GetOrSetPrivateComponent<PointLight>(ple).lock();
        plc->constant = 1.0f;
        plc->linear = 0.09f;
        plc->quadratic = 0.032f;
        plc->diffuse = glm::vec3(1.0f);
        plc->diffuse_brightness = 5;

        scene->SetDataComponent(ple, ltw);

        Entity ple2 = scene->CreateEntity("Point Light 2");
        auto plc2 = scene->GetOrSetPrivateComponent<PointLight>(ple2).lock();
        plc2->constant = 1.0f;
        plc2->linear = 0.09f;
        plc2->quadratic = 0.032f;
        plc2->diffuse = glm::vec3(1.0f);
        plc2->diffuse_brightness = 5;

        scene->SetDataComponent(ple2, ltw);
        scene->SetEntityName(ple2, "Point Light 2");
        auto plmmc2 = scene->GetOrSetPrivateComponent<MeshRenderer>(ple2).lock();
        plmmc2->mesh = Resources::GetResource<Mesh>("PRIMITIVE_SPHERE");
        plmmc2->material.Set<Material>(shared_mat);

#pragma endregion
        Application::RegisterLateUpdateFunction([=]() {
          auto scene = Application::GetActiveScene();
          Transform ltw;
          ltw.SetScale(glm::vec3(0.5f));
#pragma region LightsPosition
          ltw.SetPosition(glm::vec4(
              glm::vec3(0.0f, 20.0f * glm::sin(Times::Now() / 2.0f), -20.0f * glm::cos(Times::Now() / 2.0f)), 0.0f));
          scene->SetDataComponent(dle, ltw);
          ltw.SetPosition(glm::vec4(
              glm::vec3(-20.0f * glm::cos(Times::Now() / 2.0f), 20.0f * glm::sin(Times::Now() / 2.0f), 0.0f), 0.0f));
          scene->SetDataComponent(ple, ltw);
          ltw.SetPosition(glm::vec4(
              glm::vec3(20.0f * glm::cos(Times::Now() / 2.0f), 15.0f, 20.0f * glm::sin(Times::Now() / 2.0f)), 0.0f));
          scene->SetDataComponent(ple2, ltw);
#pragma endregion
        });
      });
    } break;
    default: {
    } break;
  }
#pragma endregion
}

Entity LoadPhysicsScene(const std::shared_ptr<Scene>& scene, const std::string& base_entity_name) {
  const auto base_entity = scene->CreateEntity(base_entity_name);
#pragma region Create 9 spheres in different PBR properties
  const int amount = 5;
  constexpr float scale_factor = 0.03f;
  const auto collection = scene->CreateEntity("Spheres");
  const auto spheres = scene->CreateEntities(amount * amount * amount, "Instance");

  for (int i = 0; i < amount; i++) {
    for (int j = 0; j < amount; j++) {
      for (int k = 0; k < amount; k++) {
        auto& sphere = spheres[i * amount * amount + j * amount + k];
        Transform transform;
        glm::vec3 position = glm::vec3(i + 0.5f - amount / 2.0f, j + 0.5f - amount / 2.0f, k + 0.5f - amount / 2.0f);
        position += glm::linearRand(glm::vec3(-0.5f), glm::vec3(0.5f)) * scale_factor;
        transform.SetPosition(position * 5.f * scale_factor);
        transform.SetScale(glm::vec3(4.0f * scale_factor));
        scene->SetDataComponent(sphere, transform);
        const auto mesh_renderer = scene->GetOrSetPrivateComponent<MeshRenderer>(sphere).lock();
        mesh_renderer->mesh = Resources::GetResource<Mesh>("PRIMITIVE_SPHERE");
        const auto material = ProjectManager::CreateTemporaryAsset<Material>();
        mesh_renderer->material = material;
        material->material_properties.roughness = static_cast<float>(i) / (amount - 1);
        material->material_properties.metallic = static_cast<float>(j) / (amount - 1);

        const auto rigid_body = scene->GetOrSetPrivateComponent<RigidBody>(sphere).lock();
        rigid_body->SetEnabled(true);
        rigid_body->SetDensityAndMassCenter(0.1f);
        auto sphere_collider = ProjectManager::CreateTemporaryAsset<Collider>();
        sphere_collider->SetShapeType(ShapeType::Sphere);
        sphere_collider->SetShapeParam(glm::vec3(2.0f * scale_factor));
        rigid_body->AttachCollider(sphere_collider);
        scene->SetParent(sphere, collection);
      }
    }
  }
  scene->SetParent(collection, base_entity);
#pragma endregion
#pragma region Create Boundaries
  {
#ifdef PHYSICS_PLUGIN
    const auto ground = CreateSolidCube(1.0, glm::vec3(1.0f), glm::vec3(0, -35, 0) * scale_factor, glm::vec3(0),
                                        glm::vec3(30, 1, 60) * scale_factor, "Ground");

    const auto right_wall = CreateSolidCube(1.0, glm::vec3(1.0f), glm::vec3(30, -20, 0) * scale_factor, glm::vec3(0),
                                            glm::vec3(1, 15, 60) * scale_factor, "LeftWall");
    const auto left_wall = CreateSolidCube(1.0, glm::vec3(1.0f), glm::vec3(-30, -20, 0) * scale_factor, glm::vec3(0),
                                           glm::vec3(1, 15, 60) * scale_factor, "RightWall");
    const auto front_wall = CreateSolidCube(1.0, glm::vec3(1.0f), glm::vec3(0, -30, 60) * scale_factor, glm::vec3(0),
                                            glm::vec3(30, 5, 1) * scale_factor, "FrontWall");
    const auto back_wall = CreateSolidCube(1.0, glm::vec3(1.0f), glm::vec3(0, -20, -60) * scale_factor, glm::vec3(0),
                                           glm::vec3(30, 15, 1) * scale_factor, "BackWall");
    scene->SetParent(right_wall, collection);
    scene->SetParent(ground, collection);
    scene->SetParent(back_wall, collection);
    scene->SetParent(left_wall, collection);
    scene->SetParent(front_wall, collection);
#endif
    /*
    const auto b1 = CreateDynamicCube(
            1.0, glm::vec3(0.0f, 0.0f, 1.0f), glm::vec3(-5, -7.5, 0) * scaleFactor, glm::vec3(0, 0, 45), glm::vec3(0.5)
    * scaleFactor, "Block 1"); const auto b2 = CreateDynamicCube(1.0, glm::vec3(1.0f), glm::vec3(0, -10, 0) *
    scaleFactor, glm::vec3(0, 0, 45), glm::vec3(1) * scaleFactor, "Block 2"); const auto b3 = CreateDynamicCube( 1.0,
    glm::vec3(0.0f, 0.0f, 1.0f), glm::vec3(5, -7.5, 0) * scaleFactor, glm::vec3(0, 0, 45), glm::vec3(1) * scaleFactor,
    "Block 3");

    auto b1j = scene->GetOrSetPrivateComponent<Joint>(b1).lock();
    b1j->SetType(JointType::Fixed);
    b1j->Link(b2);
    auto b3j = scene->GetOrSetPrivateComponent<Joint>(b3).lock();
    b3j->SetType(JointType::Fixed);
    b3j->Link(b2);
    auto b2j = scene->GetOrSetPrivateComponent<Joint>(b2).lock();
    b2j->SetType(JointType::Fixed);
    b2j->Link(ground);

    const auto anchor = CreateDynamicCube(
            1.0, glm::vec3(1.0f, 0.0f, 0.0f), glm::vec3(-10, 0, 0) * scaleFactor, glm::vec3(0, 0, 45), glm::vec3(2.0f) *
    scaleFactor, "Anchor"); scene->GetOrSetPrivateComponent<RigidBody>(anchor).lock()->SetKinematic(true); auto lastLink
    = anchor; for (int i = 1; i < 10; i++)
    {
            const auto link = CreateDynamicSphere(
                    1.0, glm::vec3(1.0f, 0.0f, 0.0f), glm::vec3(-10 - i, 0, 0) * scaleFactor, glm::vec3(0, 0, 45), 2.0f
    * scaleFactor, "Link"); auto joint = scene->GetOrSetPrivateComponent<Joint>(link).lock();
            joint->SetType(JointType::D6);
            joint->Link(lastLink);
            // joint->SetMotion(MotionAxis::SwingY, MotionType::Limited);
            scene->SetParent(link, anchor);
            lastLink = link;
    }

    const auto freeSphere = CreateDynamicCube(
            0.01, glm::vec3(0.0f, 1.0f, 0.0f), glm::vec3(-20, 0, 0) * scaleFactor, glm::vec3(0, 0, 45), glm::vec3(5.0f)
    * scaleFactor, "Free Cube"); auto joint = scene->GetOrSetPrivateComponent<Joint>(freeSphere).lock();
    joint->SetType(JointType::D6);
    joint->Link(lastLink);
    // joint->SetMotion(MotionAxis::TwistX, MotionType::Free);
    joint->SetMotion(MotionAxis::SwingY, MotionType::Free);
    joint->SetMotion(MotionAxis::SwingZ, MotionType::Free);
    */
  }
#pragma endregion
  return base_entity;
}

#ifdef PHYSICS_PLUGIN
Entity CreateSolidCube(const float& mass, const glm::vec3& color, const glm::vec3& position, const glm::vec3& rotation,
                       const glm::vec3& scale, const std::string& name) {
  auto scene = Application::GetActiveScene();
  auto cube = CreateCube(color, position, rotation, scale, name);
  const auto rigid_body = scene->GetOrSetPrivateComponent<RigidBody>(cube).lock();
  rigid_body->SetStatic(true);
  // The rigidbody can only apply mesh bound after it's attached to an entity with mesh renderer.
  rigid_body->SetEnabled(true);

  auto collider = ProjectManager::CreateTemporaryAsset<Collider>();
  collider->SetShapeType(ShapeType::Box);
  collider->SetShapeParam(scale);
  rigid_body->AttachCollider(collider);
  return cube;
}

Entity CreateDynamicCube(const float& mass, const glm::vec3& color, const glm::vec3& position,
                         const glm::vec3& rotation, const glm::vec3& scale, const std::string& name) {
  auto scene = Application::GetActiveScene();
  auto cube = CreateCube(color, position, rotation, scale, name);
  const auto rigid_body = scene->GetOrSetPrivateComponent<RigidBody>(cube).lock();
  rigid_body->SetStatic(false);
  rigid_body->SetDensityAndMassCenter(1);
  // The rigidbody can only apply mesh bound after it's attached to an entity with mesh renderer.
  rigid_body->SetEnabled(true);
  rigid_body->SetDensityAndMassCenter(mass / scale.x / scale.y / scale.z);

  auto collider = ProjectManager::CreateTemporaryAsset<Collider>();
  collider->SetShapeType(ShapeType::Box);
  collider->SetShapeParam(scale);
  rigid_body->AttachCollider(collider);
  return cube;
}

Entity CreateCube(const glm::vec3& color, const glm::vec3& position, const glm::vec3& rotation, const glm::vec3& scale,
                  const std::string& name) {
  auto scene = Application::GetActiveScene();
  auto cube = scene->CreateEntity(name);
  const auto ground_mesh_renderer = scene->GetOrSetPrivateComponent<MeshRenderer>(cube).lock();
  auto material = ProjectManager::CreateTemporaryAsset<Material>();
  ground_mesh_renderer->material = material;
  material->material_properties.albedo_color = color;
  ground_mesh_renderer->mesh = Resources::GetResource<Mesh>("PRIMITIVE_CUBE");
  Transform ground_transform;
  ground_transform.SetValue(position, glm::radians(rotation), scale * 2.0f);
  scene->SetDataComponent(cube, ground_transform);
  /*
  GlobalTransform groundGlobalTransform;
  groundGlobalTransform.SetValue(position, glm::radians(rotation), scale);
  scene->SetDataComponent(cube, groundGlobalTransform);
  */
  return cube;
}

Entity CreateDynamicSphere(const float& mass, const glm::vec3& color, const glm::vec3& position,
                           const glm::vec3& rotation, const float& scale, const std::string& name) {
  auto scene = Application::GetActiveScene();
  auto sphere = CreateSphere(color, position, rotation, scale, name);
  const auto rigid_body = scene->GetOrSetPrivateComponent<RigidBody>(sphere).lock();
  rigid_body->SetStatic(false);
  rigid_body->SetDensityAndMassCenter(1);
  // The rigidbody can only apply mesh bound after it's attached to an entity with mesh renderer.
  rigid_body->SetEnabled(true);
  rigid_body->SetDensityAndMassCenter(mass / scale / scale / scale);

  auto collider = ProjectManager::CreateTemporaryAsset<Collider>();
  collider->SetShapeType(ShapeType::Sphere);
  collider->SetShapeParam(glm::vec3(scale));
  rigid_body->AttachCollider(collider);
  return sphere;
}

Entity CreateSolidSphere(const float& mass, const glm::vec3& color, const glm::vec3& position,
                         const glm::vec3& rotation, const float& scale, const std::string& name) {
  auto scene = Application::GetActiveScene();
  auto sphere = CreateSphere(color, position, rotation, scale, name);
  const auto rigid_body = scene->GetOrSetPrivateComponent<RigidBody>(sphere).lock();
  rigid_body->SetStatic(true);
  // The rigidbody can only apply mesh bound after it's attached to an entity with mesh renderer.
  rigid_body->SetEnabled(true);

  auto collider = ProjectManager::CreateTemporaryAsset<Collider>();
  collider->SetShapeType(ShapeType::Sphere);
  collider->SetShapeParam(glm::vec3(scale));
  rigid_body->AttachCollider(collider);
  return sphere;
}

Entity CreateSphere(const glm::vec3& color, const glm::vec3& position, const glm::vec3& rotation, const float& scale,
                    const std::string& name) {
  auto scene = Application::GetActiveScene();
  auto sphere = scene->CreateEntity(name);
  const auto ground_mesh_renderer = scene->GetOrSetPrivateComponent<MeshRenderer>(sphere).lock();
  auto material = ProjectManager::CreateTemporaryAsset<Material>();
  ground_mesh_renderer->material = material;
  material->material_properties.albedo_color = color;
  ground_mesh_renderer->mesh = Resources::GetResource<Mesh>("PRIMITIVE_CUBE");
  Transform ground_transform;
  ground_transform.SetValue(position, glm::radians(rotation), glm::vec3(scale));
  // groundTransform.SetValue(glm::vec3(0, -15, 0), glm::vec3(0), glm::vec3(30, 1, 30));
  scene->SetDataComponent(sphere, ground_transform);

  GlobalTransform ground_global_transform;
  ground_global_transform.SetValue(position, glm::radians(rotation), glm::vec3(scale));
  scene->SetDataComponent(sphere, ground_global_transform);
  return sphere;
}
#endif
#pragma endregion
