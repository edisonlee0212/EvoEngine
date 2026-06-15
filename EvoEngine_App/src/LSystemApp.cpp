// LSystemApp.cpp - standalone L-system editor shell.
//
// Boots into an LSystem project by default. We resolve several candidate
// locations so local developer layouts (Resources vs Temp) still open into
// LSystem content without manual project selection.
#include <Application.hpp>

#include <array>
#include <filesystem>

#include "ClassRegistry.hpp"
#include "Times.hpp"

#include "ProjectManager.hpp"

#include "EditorLayer.hpp"
#include "ImGuiLayer.hpp"
#include "RenderLayer.hpp"
#include "WindowLayer.hpp"

using namespace evo_engine;

namespace {

std::filesystem::path FindResourceFolder() {
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
  return resource_folder_path;
}

std::filesystem::path ResolveDefaultLSystemProjectPath(const std::filesystem::path& resource_folder_path) {
  const std::array<std::filesystem::path, 2> candidates = {
      resource_folder_path / "LSystemProject" / "test.eveproj",
      resource_folder_path / "LSystemProjectAssets" / "test.eveproj"};

  for (const auto& candidate : candidates) {
    if (std::filesystem::exists(candidate)) {
      return std::filesystem::absolute(candidate);
    }
  }

  // Keep a stable fallback target even when assets are not present yet.
  return std::filesystem::absolute(resource_folder_path / "LSystemProject" / "test.eveproj");
}

}  // namespace

int main() {
  Application application;
  const auto resource_folder_path = FindResourceFolder();
  const auto lsystem_project_path = ResolveDefaultLSystemProjectPath(resource_folder_path);

  ApplicationContext::Get().PushLayer<RenderLayer>("Render Layer");
  ApplicationContext::Get().PushLayer<WindowLayer>("Window Layer");
  ApplicationContext::Get().PushLayer<ImGuiLayer>("ImGui Layer");
  ApplicationContext::Get().PushLayer<EditorLayer>("Editor Layer");

  ApplicationInitializationSettings application_configs;
  application_configs.application_name = "LSystem";
  application_configs.project_path = lsystem_project_path;
  application_configs.enable_runtime_packages = true;
  application_configs.use_custom_title_bar = true;
  // Keep LSystem first while also loading DigitalAgriculture so copied
  // default-scene components deserialize with maximal compatibility.
  application_configs.startup_runtime_packages = {"LSystem", "DigitalAgriculture"};
  ApplicationContext::Get().Initialize(application_configs);

  const auto editor_layer = ApplicationContext::Get().GetLayer<EditorLayer>();
  if (editor_layer) {
    editor_layer->velocity = 2.f;
    editor_layer->default_scene_camera_position = glm::vec3(0.0f, 1.0f, 5.0f);
  }

  ApplicationContext::Get().Start();
  ApplicationContext::Get().Run();
  ApplicationContext::Get().Terminate();
  return 0;
}
