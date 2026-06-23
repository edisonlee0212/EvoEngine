#include "EvoEngine_SDK_PCH.hpp"

#ifdef min
#  undef min
#endif
#ifdef max
#  undef max
#endif

#include <gtest/gtest.h>

#include "Application.hpp"
#include "ApplicationContext.hpp"
#include "AssetManager.hpp"
#include "Camera.hpp"
#include "Platform.hpp"
#include "ProjectManager.hpp"
#include "RenderLayer.hpp"

#include "DynamicTreeStrands.hpp"
#include "EcoSysLabLayer.hpp"
#include "Tree.hpp"
#include "TreeDescriptor.hpp"

#include <cmath>
#include <filesystem>

using namespace eco_sys_lab_package;
using namespace evo_engine;

namespace {
constexpr uint64_t kAppleTreeDescriptorHandle = 2019738788707375090ull;
constexpr float kGrowthYears = 4.0f;
constexpr float kDaysPerYear = 365.0f;
constexpr float kTwoPi = 6.28318530717958647692f;
constexpr int kMaxGrowthSteps = 128;
constexpr int kShakeSteps = 24;

std::filesystem::path EcoSysLabProjectPath() {
  return std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / "Resources" / "EcoSysLabProject" / "test.eveproj";
}

ApplicationInitializationSettings MakeHeadlessTestSettings() {
  ApplicationInitializationSettings settings;
  settings.allow_empty_project = true;
  settings.default_window_size = {256, 256};
  settings.application_mode = ApplicationMode::Headless;
  settings.load_project_assets = false;
  settings.redirect_standard_streams_to_console = false;
  settings.enable_runtime_packages = false;
  settings.graphics_settings.use_mesh_shader = false;
  settings.graphics_settings.use_ray_tracing = false;
  return settings;
}

void WaitForGpuIdleIfInitialized() {
  if (Platform::Initialized() && Platform::GetGpuService().Initialized()) {
    Platform::GetGpuService().WaitIdle();
  }
}
}  // namespace

TEST(EcoSysLab, AppleTreeDynamicStrandsSurviveSlowShake) {
  Application application;
  ASSERT_NE(application.PushLayer<RenderLayer>("Render Layer"), nullptr);
  const auto eco_sys_lab_layer = application.PushLayer<EcoSysLabLayer>("EcoSysLab Layer");
  ASSERT_NE(eco_sys_lab_layer, nullptr);

  ASSERT_NO_THROW(application.Initialize(MakeHeadlessTestSettings()));
  ASSERT_NO_THROW(ProjectManager::GetOrCreateProject(EcoSysLabProjectPath()));
  ASSERT_TRUE(ProjectManager::IsProjectLoaded());
  const auto scene = application.GetActiveScene();
  ASSERT_NE(scene, nullptr);
  ASSERT_FALSE(EcoSysLabLayer::FindClimate().expired());
  ASSERT_FALSE(EcoSysLabLayer::FindSoil().expired());

  eco_sys_lab_layer->ResetAllTrees(nullptr);
  const auto tree_descriptor = AssetManager::GetAsset<TreeDescriptor>(Handle(kAppleTreeDescriptorHandle));
  ASSERT_NE(tree_descriptor, nullptr);

  const Entity tree_entity = tree_descriptor->Instantiate();
  ASSERT_TRUE(scene->IsEntityValid(tree_entity));
  const auto tree = scene->GetOrSetPrivateComponent<Tree>(tree_entity).lock();
  ASSERT_NE(tree, nullptr);

  SimulationSettings simulation_settings = eco_sys_lab_layer->simulation_settings;
  simulation_settings.delta_time = 30.0f;
  simulation_settings.soil_simulation = false;
  SimulationStats simulation_stats;
  const float target_simulated_days = kGrowthYears * kDaysPerYear;
  int growth_steps = 0;
  while (eco_sys_lab_layer->GetSimulatedTime() < target_simulated_days) {
    ASSERT_LT(growth_steps, kMaxGrowthSteps);
    eco_sys_lab_layer->Simulate(simulation_settings, simulation_stats);
    ++growth_steps;
  }
  EXPECT_GE(eco_sys_lab_layer->GetSimulatedTime(), target_simulated_days);
  const auto grown_node_count = tree->shoot_model.PeekShootSkeleton().PeekSortedNodeList().size();
  EXPECT_GT(grown_node_count, 1u);

  const auto dynamic_tree_strands = scene->GetOrSetPrivateComponent<DynamicTreeStrands>(tree_entity).lock();
  ASSERT_NE(dynamic_tree_strands, nullptr);
  dynamic_tree_strands->initialize_parameters.min_segment_length = 0.12f;
  dynamic_tree_strands->initialize_parameters.max_segment_length = 0.2f;
  dynamic_tree_strands->initialize_parameters.uniform_subdivision = 1;
  ASSERT_NO_THROW(dynamic_tree_strands->InitializeFromTree(tree));
  ASSERT_TRUE(dynamic_tree_strands->initialized_from_tree);
  ASSERT_NE(dynamic_tree_strands->dynamic_strands, nullptr);
  ASSERT_FALSE(dynamic_tree_strands->dynamic_strands->strands.empty());
  ASSERT_FALSE(dynamic_tree_strands->dynamic_strands->segments.empty());
  WaitForGpuIdleIfInitialized();

  DynamicStrands::PhysicsParameters physics_parameters;
  physics_parameters.time_step = 0.01f;
  physics_parameters.sub_step = 4;
  const auto initial_transform = scene->GetDataComponent<GlobalTransform>(tree_entity);
  const glm::vec3 initial_position = initial_transform.GetPosition();
  for (int i = 0; i < kShakeSteps; ++i) {
    auto tree_transform = initial_transform;
    const float phase = static_cast<float>(i) / static_cast<float>(kShakeSteps - 1);
    tree_transform.SetPosition(initial_position + glm::vec3(std::sin(phase * kTwoPi) * 0.1f, 0.0f, 0.0f));
    scene->SetDataComponent(tree_entity, tree_transform);
    ASSERT_NO_THROW(dynamic_tree_strands->PhysicsStep(physics_parameters));
  }
  WaitForGpuIdleIfInitialized();

  EXPECT_FALSE(dynamic_tree_strands->dynamic_strands->strands.empty());
  EXPECT_FALSE(dynamic_tree_strands->dynamic_strands->segments.empty());
}
