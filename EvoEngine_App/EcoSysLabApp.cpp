// PlantFactory.cpp : This file contains the 'main' function. Program execution
// begins and ends there.
//
#include <Application.hpp>

#include "PostProcessingStack.hpp"
#include "Times.hpp"
#ifdef ECOSYSLAB_PLUGIN
#  include "EcoSysLabLayer.hpp"
#  include "FungusTest.hpp"
#  include "ObjectRotator.hpp"
#  include "ParticlePhysics2DDemo.hpp"
#  include "Physics2DDemo.hpp"
using namespace eco_sys_lab_plugin;
#endif
#include "ClassRegistry.hpp"

#include "ProjectManager.hpp"

#include "EditorLayer.hpp"
#include "WindowLayer.hpp"
#ifdef TEXTURE_BAKING_PLUGIN
#  include "TextureBaking.hpp"
using namespace texture_baking_plugin;
#endif

using namespace evo_engine;
void EngineSetup();

int main() {
  Application application;
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
  if (std::filesystem::exists(resource_folder_path)) {
    for (auto i : std::filesystem::recursive_directory_iterator(resource_folder_path)) {
      if (i.is_directory())
        continue;
      auto old_path = i.path();
      auto new_path = i.path();
      bool remove = false;
      if (i.path().extension().string() == ".uescene") {
        new_path.replace_extension(".evescene");
        remove = true;
      }
      if (i.path().extension().string() == ".umeta") {
        new_path.replace_extension(".evefilemeta");
        remove = true;
      }
      if (i.path().extension().string() == ".ueproj") {
        new_path.replace_extension(".eveproj");
        remove = true;
      }
      if (i.path().extension().string() == ".ufmeta") {
        new_path.replace_extension(".evefoldermeta");
        remove = true;
      }
      if (remove) {
        std::filesystem::copy(old_path, new_path);
        std::filesystem::remove(old_path);
      }
    }
  }

  EngineSetup();

  ApplicationContext::Get().PushLayer<RenderLayer>("Render Layer");
  ApplicationContext::Get().PushLayer<WindowLayer>("Window Layer");
  ApplicationContext::Get().PushLayer<EditorLayer>("Editor Layer");

#ifdef PHYSX_PHYSICS_PLUGIN
  ApplicationContext::Get().PushLayer<PhysicsLayer>();
#endif
#ifdef ECOSYSLAB_PLUGIN
  ApplicationContext::Get().PushLayer<EcoSysLabLayer>("EcoSysLab Layer")->enable_inspection = true;
  application.RegisterPrivateComponent<Physics2DDemo>("Physics2DDemo");
  application.RegisterPrivateComponent<ParticlePhysics2DDemo>("ParticlePhysics2DDemo");
  application.RegisterPrivateComponent<ObjectRotator>("ObjectRotator");
  application.RegisterPrivateComponent<FungusTest>("FungusTest");
#endif
#ifdef TEXTURE_BAKING_PLUGIN
  application.RegisterPrivateComponent<TextureBaking>("TextureBaking");
#endif

  ApplicationInitializationSettings application_configs;
  application_configs.application_name = "EcoSysLab";
  application_configs.project_path =
      std::filesystem::absolute(resource_folder_path / "EcoSysLabProject" / "test.eveproj");
  ApplicationContext::Get().Initialize(application_configs);

#ifdef PHYSX_PHYSICS_PLUGIN
  ApplicationContext::Get().GetActiveScene()->GetOrCreateSystem<PhysicsSystem>(1);
#endif
  // adjust default camera speed
  const auto editor_layer = ApplicationContext::Get().GetLayer<EditorLayer>();
  editor_layer->velocity = 2.f;
  auto& camera_settings = editor_layer->GetSceneCamera()->camera_settings;
  camera_settings.use_clear_color = true;
  camera_settings.clear_color = glm::vec4(1.f);
  camera_settings.background_intensity = 3.f;
  const auto post_processing_stack =
      editor_layer->GetSceneCamera()->post_processing_stack_ref.Get<PostProcessingStack>();
  post_processing_stack->enable_bloom = false;
  auto render_layer = ApplicationContext::Get().GetLayer<RenderLayer>();
#pragma region Engine Loop
  ApplicationContext::Get().Start();
  ApplicationContext::Get().Run();
#pragma endregion
  ApplicationContext::Get().Terminate();
}

void EngineSetup() {
  ProjectManager::SetActionAfterNewScene([=](const std::shared_ptr<Scene>& scene) {
#pragma region Engine Setup
    Transform transform;
    transform.SetEulerRotation(glm::radians(glm::vec3(150, 30, 0)));
#pragma region Preparations
    Times::SetTimeStep(0.016f);
    transform = Transform();
    transform.SetPosition(glm::vec3(0, 2, 35));
    transform.SetEulerRotation(glm::radians(glm::vec3(15, 0, 0)));
    if (const auto main_camera = ApplicationContext::Get().GetActiveScene()->main_camera.Get<Camera>()) {
      scene->SetDataComponent(main_camera->GetOwner(), transform);
      main_camera->camera_settings.use_clear_color = true;
      main_camera->camera_settings.clear_color = glm::vec4(0.5f, 0.5f, 0.5f, 1.f);
    }
#pragma endregion
#pragma endregion
  });
}
