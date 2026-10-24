#include <gtest/gtest.h>
#include <algorithm>
#include <chrono>
#include <glm/gtx/quaternion.hpp>
#include <iostream>

#include "Application.hpp"
#include "ApplicationInitializationSettings.hpp"
#include "GeometryStorage.hpp"
#include "GpuTreePhysics.hpp"
#include "Jobs.hpp"
#include "Platform.hpp"
#include "RenderLayer.hpp"
#include "Shader.hpp"
#include "TextureStorage.hpp"

namespace evo_engine {
class PlatformLifecycleTestAccess final {
 public:
  static void Initialize(const ApplicationInitializationSettings& settings) {
    Platform::Initialize(settings);
  }
  static void PreUpdate() {
    Platform::PreUpdate();
  }
  static void LateUpdate() {
    Platform::LateUpdate();
  }
  static void OnDestroy() {
    Platform::OnDestroy();
  }
};
}  // namespace evo_engine

using namespace eco_sys_lab_package;

namespace {
class ScopedTreeGpuPlatform {
 public:
  explicit ScopedTreeGpuPlatform(const bool capture_timestamps = false)
      : previous_path_(std::filesystem::current_path()) {
    application_ = std::make_unique<Application>();
    application_->PushLayer<RenderLayer>("Render Layer");
    ApplicationInitializationSettings settings;
    settings.project_path = std::filesystem::temp_directory_path() / "EvoEngineGpuTreePhysicsTest.eveproj";
    settings.load_default_resources = false;
    settings.load_project_assets = false;
    settings.load_project_start_scene = false;
    settings.enable_runtime_packages = false;
    settings.graphics_settings.use_mesh_shader = false;
    settings.graphics_settings.use_ray_tracing = false;
    settings.enable_gpu_timestamp_capture = capture_timestamps;
    const_cast<ApplicationInitializationSettings&>(application_->GetApplicationInfo()) = settings;
    Jobs::Initialize(2);
    evo_engine::PlatformLifecycleTestAccess::Initialize(settings);
    if (capture_timestamps)
      Platform::SetGpuTimestampCaptureEnabled(true);
    ApplicationContext::Get().RegisterAsset<Shader>("Shader", {".eveshader", ".slang"});
    std::filesystem::current_path(std::filesystem::path(__FILE__).parent_path().parent_path().parent_path() /
                                  "EvoEngine_Packages/EcoSysLab/Internals");
  }

  ~ScopedTreeGpuPlatform() {
    std::filesystem::current_path(previous_path_);
    application_->PopLayer<RenderLayer>();
    Platform::DrainGpuResourceWork();
    TextureStorage::OnDestroy();
    GeometryStorage::OnDestroy();
    evo_engine::PlatformLifecycleTestAccess::OnDestroy();
    Jobs::OnDestroy();
    ApplicationContext::Set(nullptr);
    (void)application_.release();
  }

 private:
  std::filesystem::path previous_path_;
  std::unique_ptr<Application> application_;
};

BaseSkeleton MakeGpuPhysicsFork() {
  BaseSkeleton skeleton;
  auto& trunk = skeleton.RefNode(0).info;
  trunk.length = 1.0f;
  trunk.thickness = 0.1f;
  trunk.global_rotation = glm::rotation(glm::vec3(0, 0, -1), glm::vec3(0, 1, 0));
  for (int i = 0; i < 2; ++i) {
    const auto handle = skeleton.Extend(0, false);
    auto& info = skeleton.RefNode(handle).info;
    info.global_position = {0, 1, 0};
    info.length = 0.8f;
    info.thickness = 0.05f;
    info.global_rotation = glm::rotation(glm::vec3(0, 0, -1), glm::normalize(glm::vec3(i == 0 ? -1.0f : 1.0f, 1, 0)));
  }
  skeleton.SortLists();
  return skeleton;
}

BaseSkeleton MakeGpuPhysicsChain() {
  BaseSkeleton skeleton;
  auto& root = skeleton.RefNode(0).info;
  root.length = 0.7f;
  root.thickness = 0.1f;
  root.global_rotation = glm::rotation(glm::vec3(0, 0, -1), glm::vec3(0, 1, 0));
  int parent = 0;
  for (int i = 0; i < 4; ++i) {
    const glm::vec3 start = skeleton.PeekNode(parent).info.GetGlobalEndPosition();
    const int handle = skeleton.Extend(parent, false);
    auto& info = skeleton.RefNode(handle).info;
    info.global_position = start;
    info.length = 0.6f;
    info.thickness = 0.08f - 0.012f * i;
    info.global_rotation = glm::rotation(glm::vec3(0, 0, -1), glm::normalize(glm::vec3(0.4f + 0.1f * i, 1.0f, 0)));
    parent = handle;
  }
  skeleton.SortLists();
  return skeleton;
}

BaseSkeleton MakeGpuPhysicsCanopy() {
  BaseSkeleton skeleton;
  auto& root = skeleton.RefNode(0).info;
  root.length = 0.7f;
  root.thickness = 0.1f;
  root.global_rotation = glm::rotation(glm::vec3(0, 0, -1), glm::vec3(0, 1, 0));
  std::vector<int> level{0};
  for (int depth = 0; depth < 6; ++depth) {
    std::vector<int> next;
    for (const int parent : level) {
      const auto start = skeleton.PeekNode(parent).info.GetGlobalEndPosition();
      for (int side : {-1, 1}) {
        const int node = skeleton.Extend(parent, false);
        auto& info = skeleton.RefNode(node).info;
        info.global_position = start;
        info.length = 0.6f / (1.0f + depth * 0.2f);
        info.thickness = 0.08f / (1.0f + depth * 0.5f);
        info.global_rotation =
            glm::rotation(glm::vec3(0, 0, -1), glm::normalize(glm::vec3(side * 0.25f, 1.0f, depth * 0.08f)));
        next.push_back(node);
      }
    }
    level = std::move(next);
  }
  skeleton.SortLists();
  return skeleton;
}
}  // namespace

TEST(GpuTreePhysics, PinnedRodRespondsToGravity) {
  ScopedTreeGpuPlatform platform;

  DynamicTreeSkeleton::GpuRestState state;
  state.vertices = {{{0, 0, 0, 0}, {0, 0, 0, 0.1f}, {0, 0, 0, 0}, {0, 1, 0, 0}},
                    {{1, 0, 0, 1}, {0, 0, 0, 0.1f}, {1, 0, 0, 0}, {1, 1, 0, 0}}};
  state.rods = {{{0, 1, 0, 1}, {1, 1, 1, 0.1f}, {0, 0, 0, 1}, {0, 0, 0, 1}, {0, 0, 0, 0}}};
  state.vertex_rods = {0, 0};
  state.node_rods = {0};
  GpuTreePhysics physics;
  ASSERT_TRUE(physics.Initialize(state));
  DynamicTreeSkeleton::PhysicsSettings parameters;
  for (int i = 0; i < 16; ++i) {
    evo_engine::PlatformLifecycleTestAccess::PreUpdate();
    physics.Step(parameters);
    evo_engine::PlatformLifecycleTestAccess::LateUpdate();
  }
  Platform::WaitForFrameSubmissions("GPU tree physics pinned rod test");
  std::vector<DynamicTreeSkeleton::GpuRestState::Vertex> vertices;
  physics.GetVertexBuffer()->DownloadVector(vertices, 2);
  EXPECT_EQ(glm::vec3(vertices[0].position_inverse_mass), glm::vec3(0));
  EXPECT_LT(vertices[1].position_inverse_mass.y, -0.001f);
  EXPECT_GT(vertices[1].position_inverse_mass.x, 0.8f);
}

TEST(GpuTreePhysics, PacksMultipleTreesWithRebasedIndices) {
  Tree tree;
  tree.shoot_model.Initialize(MakeGpuPhysicsFork());
  DynamicTreeSkeleton physics;
  ASSERT_TRUE(physics.InitializePhysics(tree));
  const auto rest = physics.ExportGpuRestState();
  const auto packed = GpuTreePhysics::PackRestStates({rest, rest});
  ASSERT_EQ(packed.ranges.size(), 2);
  const auto& range = packed.ranges[1];
  EXPECT_EQ(range.vertex_base, rest.vertices.size());
  EXPECT_EQ(range.rod_base, rest.rods.size());
  EXPECT_EQ(range.node_rod_base, rest.node_rods.size());
  EXPECT_EQ(packed.state.vertices[range.vertex_base].incident_rods.x,
            rest.vertices[0].incident_rods.x + rest.vertex_rods.size());
  EXPECT_EQ(packed.state.vertices[range.vertex_base].incident_rods.z, 1u);
  EXPECT_EQ(packed.state.rods[range.rod_base].links.x, rest.rods[0].links.x + range.vertex_base);
  EXPECT_EQ(packed.state.rods[range.rod_base].incident_angles.z, 1u);
  EXPECT_EQ(packed.state.vertex_rods[rest.vertex_rods.size()], rest.vertex_rods[0] + range.rod_base);
  EXPECT_EQ(packed.state.node_rods[range.node_rod_base], rest.node_rods[0] + range.rod_base);
  ASSERT_FALSE(rest.angles.empty());
  EXPECT_EQ(packed.state.angles[rest.angles.size()].rods.x, rest.angles[0].rods.x + range.rod_base);
  EXPECT_EQ(packed.state.rod_angles[rest.rod_angles.size()], rest.rod_angles[0] + rest.angles.size());
}

TEST(GpuTreePhysics, SharedSolverKeepsTreeSettingsIndependent) {
  ScopedTreeGpuPlatform platform;
  DynamicTreeSkeleton::GpuRestState rest;
  rest.vertices = {{{0, 0, 0, 0}, {0, 0, 0, 0.1f}, {0, 0, 0, 0}, {0, 1, 0, 0}},
                   {{1, 0, 0, 1}, {0, 0, 0, 0.1f}, {1, 0, 0, 0}, {1, 1, 0, 0}}};
  rest.rods = {{{0, 1, 0, 1}, {1, 1, 1, 0.1f}, {0, 0, 0, 1}, {0, 0, 0, 1}, {0, 0, 0, 0}}};
  rest.vertex_rods = {0, 0};
  rest.node_rods = {0};
  auto second = rest;
  for (auto& vertex : second.vertices) {
    vertex.position_inverse_mass.x += 5.0f;
    vertex.rest_local.x += 5.0f;
  }
  GpuTreePhysics physics;
  ASSERT_TRUE(physics.Initialize(std::vector<DynamicTreeSkeleton::GpuRestState>{rest, second}));
  DynamicTreeSkeleton::PhysicsSettings settings;
  auto calm = settings;
  calm.gravity = {};
  calm.substeps = 2;
  calm.iterations = 3;
  for (int i = 0; i < 16; ++i) {
    evo_engine::PlatformLifecycleTestAccess::PreUpdate();
    physics.Step(std::vector<GpuTreePhysics::TreeStep>{{settings, true}, {calm, true}});
    evo_engine::PlatformLifecycleTestAccess::LateUpdate();
  }
  Platform::WaitForFrameSubmissions("GPU forest physics first tree");
  std::vector<DynamicTreeSkeleton::GpuRestState::Vertex> vertices;
  physics.GetVertexBuffer()->DownloadVector(vertices, 4);
  EXPECT_LT(vertices[1].position_inverse_mass.y, -0.001f);
  EXPECT_FLOAT_EQ(vertices[3].position_inverse_mass.y, 0.0f);
  const float first_y = vertices[1].position_inverse_mass.y;
  for (int i = 0; i < 16; ++i) {
    evo_engine::PlatformLifecycleTestAccess::PreUpdate();
    physics.Step(std::vector<GpuTreePhysics::TreeStep>{{settings, false}, {settings, true}});
    evo_engine::PlatformLifecycleTestAccess::LateUpdate();
  }
  Platform::WaitForFrameSubmissions("GPU forest physics second tree");
  physics.GetVertexBuffer()->DownloadVector(vertices, 4);
  EXPECT_FLOAT_EQ(vertices[1].position_inverse_mass.y, first_y);
  EXPECT_LT(vertices[3].position_inverse_mass.y, -0.001f);
}

TEST(GpuTreePhysics, PausedForestMotionMovesOnlyItsTreeRigidly) {
  ScopedTreeGpuPlatform platform;
  Tree tree;
  tree.shoot_model.Initialize(MakeGpuPhysicsFork());
  DynamicTreeSkeleton component;
  ASSERT_TRUE(component.InitializePhysics(tree));
  const auto rest = component.ExportGpuRestState();
  GpuTreePhysics physics;
  ASSERT_TRUE(physics.Initialize(std::vector<DynamicTreeSkeleton::GpuRestState>{rest, rest}));
  GlobalTransform moved;
  moved.SetPosition({2.0f, 0.4f, -1.0f});
  moved.SetRotation(glm::angleAxis(0.3f, glm::vec3(0, 0, 1)));
  evo_engine::PlatformLifecycleTestAccess::PreUpdate();
  physics.Step(std::vector<GpuTreePhysics::TreeStep>{{component.physics_settings, false, moved},
                                                     {component.physics_settings, false, GlobalTransform{}}});
  evo_engine::PlatformLifecycleTestAccess::LateUpdate();
  Platform::WaitForFrameSubmissions("paused GPU forest parent motion");
  std::vector<DynamicTreeSkeleton::GpuRestState::Vertex> vertices;
  physics.GetVertexBuffer()->DownloadVector(vertices, rest.vertices.size() * 2);
  for (size_t i = 0; i < rest.vertices.size(); ++i) {
    const glm::vec3 expected =
        glm::vec3(moved.value * glm::vec4(glm::vec3(rest.vertices[i].position_inverse_mass), 1.0f));
    EXPECT_LT(glm::length(glm::vec3(vertices[i].position_inverse_mass) - expected), 1e-4f);
    EXPECT_LT(glm::length(glm::vec3(vertices[i + rest.vertices.size()].position_inverse_mass) -
                          glm::vec3(rest.vertices[i].position_inverse_mass)),
              1e-4f);
  }
}

TEST(GpuTreePhysics, RunningTreeRootFollowsMovedForest) {
  ScopedTreeGpuPlatform platform;
  Tree tree;
  tree.shoot_model.Initialize(MakeGpuPhysicsFork());
  DynamicTreeSkeleton component;
  ASSERT_TRUE(component.InitializePhysics(tree));
  const auto rest = component.ExportGpuRestState();
  GpuTreePhysics physics;
  ASSERT_TRUE(physics.Initialize(rest));
  GlobalTransform moved;
  moved.SetPosition({1.0f, 0.0f, 0.0f});
  evo_engine::PlatformLifecycleTestAccess::PreUpdate();
  physics.Step(std::vector<GpuTreePhysics::TreeStep>{{component.physics_settings, true, moved}});
  evo_engine::PlatformLifecycleTestAccess::LateUpdate();
  Platform::WaitForFrameSubmissions("running GPU forest parent motion");
  std::vector<DynamicTreeSkeleton::GpuRestState::Vertex> vertices;
  physics.GetVertexBuffer()->DownloadVector(vertices, rest.vertices.size());
  EXPECT_NEAR(vertices[0].position_inverse_mass.x, 1.0f, 1e-4f);
  EXPECT_LT(vertices[1].position_inverse_mass.x, 0.99f);
}

TEST(GpuTreePhysics, SimulationDoesNotChangeGrowthSkeleton) {
  ScopedTreeGpuPlatform platform;
  Tree tree;
  tree.shoot_model.Initialize(MakeGpuPhysicsFork());
  DynamicTreeSkeleton component;
  component.gpu_physics_enabled = true;
  ASSERT_TRUE(component.SyncGpuPhysics(tree, GlobalTransform{}, false));
  const auto rest = component.ExportGpuRestState();
  const auto tip = rest.rods[rest.node_rods[1]].links.y;
  const glm::vec3 original_tip = rest.vertices[tip].position_inverse_mass;
  GpuTreePhysics gpu;
  ASSERT_TRUE(gpu.Initialize(rest));
  for (int i = 0; i < 16; ++i) {
    evo_engine::PlatformLifecycleTestAccess::PreUpdate();
    gpu.Step(component.physics_settings);
    evo_engine::PlatformLifecycleTestAccess::LateUpdate();
  }
  Platform::WaitForFrameSubmissions("GPU growth pose isolation");
  std::vector<DynamicTreeSkeleton::GpuRestState::Vertex> vertices;
  gpu.GetVertexBuffer()->DownloadVector(vertices, rest.vertices.size());
  EXPECT_GT(glm::length(glm::vec3(vertices[tip].position_inverse_mass) - original_tip), 1e-4f);
  const auto& original_node = tree.shoot_model.PeekShootSkeleton().PeekNode(1).info;
  EXPECT_LT(glm::length(original_node.GetGlobalEndPosition() - original_tip), 1e-5f);

  component.ResetPhysics(tree);
  ASSERT_TRUE(component.SyncGpuPhysics(tree, GlobalTransform{}, true));
  EXPECT_FALSE(component.physics_running);
  const auto restored = component.ExportGpuRestState();
  EXPECT_LT(glm::length(glm::vec3(restored.vertices[tip].position_inverse_mass) - original_tip), 1e-5f);
}

TEST(GpuTreePhysics, ForkMotionTracksCpuReference) {
  ScopedTreeGpuPlatform platform;
  Tree tree;
  tree.shoot_model.Initialize(MakeGpuPhysicsFork());
  DynamicTreeSkeleton cpu;
  ASSERT_TRUE(cpu.InitializePhysics(tree));
  const auto rest = cpu.ExportGpuRestState();
  GpuTreePhysics gpu;
  ASSERT_TRUE(gpu.Initialize(rest));
  cpu.physics_running = true;
  for (int i = 0; i < 16; ++i) {
    evo_engine::PlatformLifecycleTestAccess::PreUpdate();
    gpu.Step(cpu.physics_settings);
    evo_engine::PlatformLifecycleTestAccess::LateUpdate();
    cpu.StepPhysics(tree);
  }
  Platform::WaitForFrameSubmissions("GPU tree physics fork comparison");
  std::vector<DynamicTreeSkeleton::GpuRestState::Vertex> vertices;
  gpu.GetVertexBuffer()->DownloadVector(vertices, rest.vertices.size());
  const auto& pose = tree.shoot_model.PeekPhysicsPose();
  for (uint32_t node = 1; node <= 2; ++node) {
    const uint32_t tip = rest.rods[rest.node_rods[node]].links.y;
    const glm::vec3 cpu_tip = pose[node].position + pose[node].rotation * glm::vec3(0, 0, -1) * pose[node].length;
    EXPECT_LT(glm::length(glm::vec3(vertices[tip].position_inverse_mass) - cpu_tip), 0.05f);
    EXPECT_LT(vertices[tip].position_inverse_mass.y, rest.vertices[tip].position_inverse_mass.y);
  }
}

TEST(GpuTreePhysics, DeepTreeMotionTracksCpuReference) {
  ScopedTreeGpuPlatform platform;
  Tree tree;
  tree.shoot_model.Initialize(MakeGpuPhysicsChain());
  DynamicTreeSkeleton cpu;
  ASSERT_TRUE(cpu.InitializePhysics(tree));
  const auto rest = cpu.ExportGpuRestState();
  GpuTreePhysics gpu;
  ASSERT_TRUE(gpu.Initialize(rest));
  cpu.physics_running = true;
  for (int i = 0; i < 16; ++i) {
    evo_engine::PlatformLifecycleTestAccess::PreUpdate();
    gpu.Step(cpu.physics_settings);
    evo_engine::PlatformLifecycleTestAccess::LateUpdate();
    cpu.StepPhysics(tree);
  }
  Platform::WaitForFrameSubmissions("GPU tree physics deep tree comparison");
  std::vector<DynamicTreeSkeleton::GpuRestState::Vertex> vertices;
  gpu.GetVertexBuffer()->DownloadVector(vertices, rest.vertices.size());
  const auto& pose = tree.shoot_model.PeekPhysicsPose();
  for (uint32_t node = 1; node < 5; ++node) {
    const uint32_t tip = rest.rods[rest.node_rods[node]].links.y;
    const glm::vec3 cpu_tip = pose[node].position + pose[node].rotation * glm::vec3(0, 0, -1) * pose[node].length;
    EXPECT_LT(glm::length(glm::vec3(vertices[tip].position_inverse_mass) - cpu_tip), 0.08f);
  }
}

TEST(GpuTreePhysics, DISABLED_ForestScaleTiming) {
  ScopedTreeGpuPlatform platform(true);
  Tree tree;
  tree.shoot_model.Initialize(MakeGpuPhysicsCanopy());
  DynamicTreeSkeleton component;
  ASSERT_TRUE(component.InitializePhysics(tree));
  const auto rest = component.ExportGpuRestState();
  for (const int count : {1, 4, 16, 32}) {
    GpuTreePhysics physics;
    ASSERT_TRUE(physics.Initialize(std::vector<DynamicTreeSkeleton::GpuRestState>(count, rest)));
    std::vector<GpuTreePhysics::TreeStep> steps(count, {component.physics_settings, true});
    Platform::ResetGpuTimestampStats();
    const auto start = std::chrono::steady_clock::now();
    for (int frame = 0; frame < 120; ++frame) {
      evo_engine::PlatformLifecycleTestAccess::PreUpdate();
      physics.Step(steps);
      evo_engine::PlatformLifecycleTestAccess::LateUpdate();
    }
    Platform::WaitForFrameSubmissions("GPU forest physics timing");
    const auto wall_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
    const auto stats = Platform::GetGpuTimestampStats();
    const auto timing = std::find_if(stats.begin(), stats.end(), [](const auto& stat) {
      return stat.name == "EcoSysLab Tree Physics";
    });
    ASSERT_NE(timing, stats.end());
    std::cout << count << " trees, " << count * rest.rods.size() << " rods: solver GPU median "
              << timing->MedianMilliseconds() << " ms, 120-step wall " << wall_ms << " ms\n";
  }
}
