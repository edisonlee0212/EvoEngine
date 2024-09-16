// PlantFactory.cpp : This file contains the 'main' function. Program execution
// begins and ends there.
//
#include <Application.hpp>

#ifdef OPTIX_RAY_TRACER_PLUGIN
#  include <CUDAModule.hpp>
#  include <RayTracerLayer.hpp>
#endif

#include "Times.hpp"
#ifdef ECOSYSLAB_PLUGIN
#  include "EcoSysLabLayer.hpp"
#  include "ObjectRotator.hpp"
#  include "ParticlePhysics2DDemo.hpp"
#  include "Physics2DDemo.hpp"
using namespace eco_sys_lab_plugin;
#endif
#include "ClassRegistry.hpp"

#include "ProjectManager.hpp"

#include "WindowLayer.hpp"
#ifdef TEXTURE_BAKING_PLUGIN
#  include "TextureBaking.hpp"
using namespace texture_baking_plugin;
#endif


void EngineSetup();

int main() {
  std::filesystem::path resourceFolderPath("../../../../../Resources");
  if (!std::filesystem::exists(resourceFolderPath)) {
    resourceFolderPath = "../../../../Resources";
  }
  if (!std::filesystem::exists(resourceFolderPath)) {
    resourceFolderPath = "../../../Resources";
  }
  if (!std::filesystem::exists(resourceFolderPath)) {
    resourceFolderPath = "../../Resources";
  }
  if (!std::filesystem::exists(resourceFolderPath)) {
    resourceFolderPath = "../Resources";
  }
  if (std::filesystem::exists(resourceFolderPath)) {
    for (auto i : std::filesystem::recursive_directory_iterator(resourceFolderPath)) {
      if (i.is_directory())
        continue;
      auto oldPath = i.path();
      auto newPath = i.path();
      bool remove = false;
      if (i.path().extension().string() == ".uescene") {
        newPath.replace_extension(".evescene");
        remove = true;
      }
      if (i.path().extension().string() == ".umeta") {
        newPath.replace_extension(".evefilemeta");
        remove = true;
      }
      if (i.path().extension().string() == ".ueproj") {
        newPath.replace_extension(".eveproj");
        remove = true;
      }
      if (i.path().extension().string() == ".ufmeta") {
        newPath.replace_extension(".evefoldermeta");
        remove = true;
      }
      if (remove) {
        std::filesystem::copy(oldPath, newPath);
        std::filesystem::remove(oldPath);
      }
    }
  }

  EngineSetup();

  Application::PushLayer<WindowLayer>();
  Application::PushLayer<EditorLayer>();
  Application::PushLayer<RenderLayer>();
#ifdef OPTIX_RAY_TRACER_PLUGIN
  Application::PushLayer<RayTracerLayer>();
#endif

#ifdef PHYSX_PHYSICS_PLUGIN
  Application::PushLayer<PhysicsLayer>();
#endif
#ifdef ECOSYSLAB_PLUGIN
  Application::PushLayer<EcoSysLabLayer>();
  PrivateComponentRegistration<Physics2DDemo>("Physics2DDemo");
  PrivateComponentRegistration<ParticlePhysics2DDemo>("ParticlePhysics2DDemo");
  PrivateComponentRegistration<ObjectRotator>("ObjectRotator");
#endif
#ifdef TEXTURE_BAKING_PLUGIN
  PrivateComponentRegistration<TextureBaking>("TextureBaking");
#endif

  ApplicationInfo application_configs;
  application_configs.application_name = "EcoSysLab";
  application_configs.project_path =
      std::filesystem::absolute(resourceFolderPath / "EcoSysLabProject" / "test.eveproj");
  Application::Initialize(application_configs);

#ifdef OPTIX_RAY_TRACER_PLUGIN
  auto ray_tracer_layer = Application::GetLayer<RayTracerLayer>();
#endif
#ifdef PHYSX_PHYSICS_PLUGIN
  Application::GetActiveScene()->GetOrCreateSystem<PhysicsSystem>(1);
#endif
  // adjust default camera speed
  const auto editor_layer = Application::GetLayer<EditorLayer>();
  editor_layer->velocity = 2.f;
  editor_layer->default_scene_camera_position = glm::vec3(1.124, 0.218, 14.089);
  // override default scene camera position etc.
  editor_layer->show_camera_window = false;
  editor_layer->show_scene_window = true;
  editor_layer->show_entity_explorer_window = true;
  editor_layer->show_entity_inspector_window = true;
  const auto render_layer = Application::GetLayer<RenderLayer>();
#pragma region Engine Loop
  Application::Start();
  Application::Run();
#pragma endregion
  Application::Terminate();
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
    if (const auto main_camera = Application::GetActiveScene()->main_camera.Get<Camera>()) {
      scene->SetDataComponent(main_camera->GetOwner(), transform);
      main_camera->use_clear_color = true;
      main_camera->clear_color = glm::vec3(0.5f);
    }
#pragma endregion
#pragma endregion
  });
}
