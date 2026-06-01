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
  Application application;
  ApplicationContext::Get().PushLayer<RenderLayer>("Render Layer");
  ApplicationContext::Get().PushLayer<WindowLayer>("Window Layer");
  ApplicationContext::Get().PushLayer<EditorLayer>("Editor Layer");

  ApplicationInitializationSettings application_info{};
  ApplicationContext::Get().Initialize(application_info);

  ApplicationContext::Get().Start();
  ApplicationContext::Get().Run();
  ApplicationContext::Get().Terminate();
  return 0;
}
