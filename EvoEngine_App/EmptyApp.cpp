#include "AnimationPlayer.hpp"
#include "Application.hpp"
#include "ClassRegistry.hpp"

#include "EditorLayer.hpp"
#include "MeshRenderer.hpp"

#include "PlayerController.hpp"
#include "Prefab.hpp"
#include "RenderLayer.hpp"

#include "Times.hpp"
#include "WindowLayer.hpp"
using namespace evo_engine;
int main() {
  Application::PushLayer<RenderLayer>("Render Layer");
  Application::PushLayer<WindowLayer>("Window Layer");
  Application::PushLayer<EditorLayer>("Editor Layer");

  ApplicationInitializationSettings application_info{};
  Application::Initialize(application_info);

  Application::Start();
  Application::Run();
  Application::Terminate();
  return 0;
}
