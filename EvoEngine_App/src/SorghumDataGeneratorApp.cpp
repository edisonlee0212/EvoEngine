#include "Application.hpp"
#include "ClassRegistry.hpp"
#include "EditorLayer.hpp"
#include "ImGuiLayer.hpp"
#include "ProjectManager.hpp"
#include "RenderLayer.hpp"
#include "WindowLayer.hpp"

using namespace evo_engine;

namespace {
std::filesystem::path FindResourceFolder() {
  std::filesystem::path resource_folder_path("../../../../../Resources");
  if (!std::filesystem::exists(resource_folder_path))
    resource_folder_path = "../../../../Resources";
  if (!std::filesystem::exists(resource_folder_path))
    resource_folder_path = "../../../Resources";
  if (!std::filesystem::exists(resource_folder_path))
    resource_folder_path = "../../Resources";
  if (!std::filesystem::exists(resource_folder_path))
    resource_folder_path = "../Resources";
  return std::filesystem::absolute(resource_folder_path);
}
}  // namespace

int main() {
  Application application;

  ApplicationContext::Get().PushLayer<RenderLayer>("Render Layer");
  ApplicationContext::Get().PushLayer<WindowLayer>("Window Layer");
  ApplicationContext::Get().PushLayer<ImGuiLayer>("ImGui Layer");
  ApplicationContext::Get().PushLayer<EditorLayer>("Editor Layer");

  ApplicationInitializationSettings application_info{};
  application_info.application_name = "Sorghum Data Generator";
  application_info.project_path = FindResourceFolder() / "DigitalAgricultureProject" / "test.eveproj";
  application_info.enable_runtime_packages = true;
  application_info.startup_runtime_packages = {"DatasetGeneration"};

  ApplicationContext::Get().Initialize(application_info);
  EVOENGINE_LOG(
      "SorghumDataGeneratorApp now bootstraps DatasetGeneration as a runtime package. Direct generation "
      "commands need a package API before they can run from this app.")
  ApplicationContext::Get().Start();
  ApplicationContext::Get().Run();
  ApplicationContext::Get().Terminate();
  return 0;
}
