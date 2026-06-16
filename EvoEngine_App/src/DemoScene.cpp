#include "DemoScene.hpp"

#include "AnimationPlayer.hpp"
#include "Application.hpp"
#include "EditorLayer.hpp"
#include "Lights.hpp"
#include "MeshRenderer.hpp"
#include "PathUtils.hpp"
#include "PlayerController.hpp"
#include "PostProcessingStack.hpp"
#include "Prefab.hpp"
#include "ProjectManager.hpp"
#include "Resources.hpp"
#include "SkinnedMeshRenderer.hpp"
#include "Times.hpp"

using namespace evo_engine;

namespace {
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

  return base_entity;
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
}  // namespace

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

  RemoveGeneratedFiles(resource_root / "EvoEngine-DemoProjects", {".evescene", ".eveproj"});
  RemoveGeneratedFiles(resource_root, {".uescene", ".ueproj"});
}

void evo_engine::SetupDemoScene(const DemoSetup demo_setup, ApplicationInitializationSettings& application_info,
                                const std::filesystem::path& resource_folder_path,
                                const bool clear_generated_project_files) {
  const auto resource_root = FindDemoResourcesRoot(resource_folder_path);
  if (demo_setup != DemoSetup::Empty && resource_root.empty()) {
    EVOENGINE_ERROR("Failed to locate Resources folder for DemoApp scene setup.")
    return;
  }

  if (demo_setup != DemoSetup::Empty && clear_generated_project_files) {
    ClearGeneratedDemoProjectFiles(resource_root);
  }

  switch (demo_setup) {
    case DemoSetup::Rendering: {
      application_info.application_name = "Rendering Demo";
      application_info.project_path = resource_root / "EvoEngine-DemoProjects/Rendering/Rendering.eveproj";
      ProjectManager::SetActionAfterNewScene([](const std::shared_ptr<Scene>& scene) {
        scene->environment.ambient_light_intensity = 0.5f;

        const auto main_camera = scene->main_camera.Get<Camera>();
        main_camera->Resize({640, 480});
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

        const auto left_point_light_right_entity = scene->CreateEntity("Left Point Light");
        const auto point_light_right_renderer =
            scene->GetOrSetPrivateComponent<MeshRenderer>(left_point_light_right_entity).lock();
        const auto point_light_right_material = AssetManager::CreateTemporaryAsset<Material>();
        point_light_right_renderer->material.Set<Material>(point_light_right_material);
        point_light_right_material->material_properties.albedo_color = glm::vec3(1.0f, 0.8f, 0.0f);
        point_light_right_material->material_properties.emission = 10.0f;
        point_light_right_renderer->mesh = Resources::GetInstance().GetPrimitives().sphere;
        const auto point_light_right =
            scene->GetOrSetPrivateComponent<PointLight>(left_point_light_right_entity).lock();
        point_light_right->diffuse_brightness = 100;
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
          if (!ApplicationContext::Get().IsPlaying()) {
            last_frame_playing = ApplicationContext::Get().IsPlaying();
            return;
          }
          const auto current_scene = ApplicationContext::Get().GetActiveScene();
          static float start_time;
          if (!last_frame_playing)
            start_time = ApplicationContext::Get().GetTimes().Now();
          const float current_time = ApplicationContext::Get().GetTimes().Now() - start_time;
          const float cos_time = glm::cos(current_time / 2.5f);

          Transform current_left_point_light_transform;
          current_left_point_light_transform.SetPosition(glm::vec3(3, 0, cos_time * 2.5f - 2.5f));
          current_left_point_light_transform.SetScale(glm::vec3(0.1f));
          current_scene->SetDataComponent(left_point_light_right_entity, current_left_point_light_transform);

          last_frame_playing = ApplicationContext::Get().IsPlaying();
        });
      });
    } break;
    case DemoSetup::Universe:
    case DemoSetup::Empty:
    default: {
    } break;
  }
}
