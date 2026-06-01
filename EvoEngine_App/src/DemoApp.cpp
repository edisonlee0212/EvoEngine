#include "Application.hpp"
#include "DemoScene.hpp"
#include "EditorLayer.hpp"
#include "RenderLayer.hpp"
#include "WindowLayer.hpp"

#ifdef PHYSX_PHYSICS_SERVICE
#  include "PhysicsLayer.hpp"
#endif

using namespace evo_engine;

int main() {
  Application application;
  constexpr DemoSetup demo_setup = DemoSetup::Rendering;
  ApplicationContext::Get().PushLayer<RenderLayer>("Render Layer");
  ApplicationContext::Get().PushLayer<WindowLayer>("Window Layer");
  ApplicationContext::Get().PushLayer<EditorLayer>("Editor Layer");
#ifdef PHYSX_PHYSICS_SERVICE
  ApplicationContext::Get().PushLayer<PhysicsLayer>();
#endif

  ApplicationInitializationSettings application_info;
  SetupDemoScene(demo_setup, application_info);

  ApplicationContext::Get().Initialize(application_info);

  ApplicationContext::Get().Start();
  ApplicationContext::Get().Run();
  ApplicationContext::Get().Terminate();
  return 0;
}
