#include <gtest/gtest.h>
#include <algorithm>
#include <chrono>
#include <glm/gtx/quaternion.hpp>
#include <iostream>

#include "Application.hpp"
#include "ApplicationInitializationSettings.hpp"
#include "GeometryStorage.hpp"
#include "GpuSpatialGrid.hpp"
#include "GpuTreePhysics.hpp"
#include "Jobs.hpp"
#include "Platform.hpp"
#include "RenderLayer.hpp"
#include "Shader.hpp"
#include "TextureStorage.hpp"
#include "WindParticleField.hpp"

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
    Shader::RegisterShaderIncludePath("./EcoSysLabResources/Shaders/Modules");
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

  component.ResetPhysics();
  ASSERT_TRUE(component.SyncGpuPhysics(tree, GlobalTransform{}, true));
  EXPECT_FALSE(component.physics_running);
  const auto restored = component.ExportGpuRestState();
  EXPECT_LT(glm::length(glm::vec3(restored.vertices[tip].position_inverse_mass) - original_tip), 1e-5f);
}

TEST(GpuTreePhysics, RegisteredAirCellsDriveBranchesAfterParticleDeposition) {
  ScopedTreeGpuPlatform platform;
  Tree tree;
  tree.shoot_model.Initialize(MakeGpuPhysicsChain());
  DynamicTreeSkeleton component;
  ASSERT_TRUE(component.InitializePhysics(tree));
  const auto rest = component.ExportGpuRestState();
  auto physics = std::make_shared<GpuTreePhysics>();
  ASSERT_TRUE(physics->Initialize({rest, rest, rest}));
  GpuSpatialGrid air;
  glm::vec3 lower(rest.vertices.front().position_inverse_mass);
  glm::vec3 upper = lower;
  for (const auto& vertex : rest.vertices) {
    const glm::vec3 point(vertex.position_inverse_mass);
    lower = glm::min(lower, point);
    upper = glm::max(upper, point);
  }
  ASSERT_TRUE(air.RegisterBox(lower - glm::vec3(1), upper + glm::vec3(1)));
  const uint32_t origin_cell = air.Find({0, 0, 0});
  ASSERT_NE(origin_cell, UINT32_MAX);
  const auto first_chunk = air.GetValueChunk(0);
  const auto first_heads = air.GetHeads();
  struct Addresses {
    uint64_t cells, values;
  };
  std::vector<Addresses> addresses;
  air.GetAddresses()->DownloadVector(addresses, 1);
  ASSERT_TRUE(air.RegisterBox({100, 0, 0}, {101, 1, 1}));
  EXPECT_LE(air.GetLowerCell().x, 0);
  EXPECT_GE(air.GetUpperCell().x, 101);
  std::vector<Addresses> grown;
  air.GetAddresses()->DownloadVector(grown, 1);
  EXPECT_EQ(grown[0].cells, addresses[0].cells);
  EXPECT_EQ(grown[0].values, addresses[0].values);
  EXPECT_EQ(air.GetValueChunk(0), first_chunk);
  EXPECT_NE(air.GetHeads(), first_heads);
  EXPECT_EQ(air.Find({0, 0, 0}), origin_cell);
  GpuSpatialGrid independent(2.0f, sizeof(glm::vec4));
  ASSERT_TRUE(independent.RegisterBox({0, 0, 0}, {1, 1, 1}));
  EXPECT_EQ(independent.GetCellCount(), 1u);
  EXPECT_EQ(independent.GetValueChunk(0)->GetSize(), GpuSpatialGrid::chunk_capacity * sizeof(glm::vec4));
  independent.Reset();
  ASSERT_TRUE(independent.RegisterBox({-2, 0, 0}, {-2, 0, 0}));
  EXPECT_EQ(independent.GetLowerCell().x, -1);
  EXPECT_EQ(independent.GetUpperCell().x, -1);
  GpuSpatialGrid fine(0.01f);
  ASSERT_TRUE(fine.RegisterBox({0, 0, 0}, {0.02f, 0.02f, 0.02f}));
  EXPECT_EQ(fine.GetCellCount(), 27u);
  EXPECT_NE(fine.Find({1, 1, 1}), UINT32_MAX);

  struct Particle {
    glm::vec4 position_age, velocity_lifetime, collision;
  };
  std::vector<Particle> particles;
  constexpr uint32_t bucket_count = 128;
  std::vector<uint32_t> heads(bucket_count, UINT32_MAX);
  std::vector<uint32_t> next;
  for (const auto& rod : rest.rods) {
    const glm::vec3 a(rest.vertices[rod.links.x].position_inverse_mass);
    const glm::vec3 b(rest.vertices[rod.links.y].position_inverse_mass);
    const glm::vec3 position = (a + b) * 0.5f;
    const glm::ivec3 cell(glm::floor(position));
    const uint32_t hash = (static_cast<uint32_t>(cell.x) * 73856093u ^ static_cast<uint32_t>(cell.y) * 19349663u ^
                           static_cast<uint32_t>(cell.z) * 83492791u) %
                          bucket_count;
    next.push_back(heads[hash]);
    heads[hash] = static_cast<uint32_t>(particles.size());
    particles.push_back({glm::vec4(position, 0), {0, 0, 5, 3}, {0.35f, 1.0f, 0.007f, 293.15f}});
  }
  const auto make_buffer = [](const auto& data) {
    VkBufferCreateInfo info{VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO};
    info.size = data.size() * sizeof(data[0]);
    info.usage = VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT;
    info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
    VmaAllocationCreateInfo allocation{};
    allocation.usage = VMA_MEMORY_USAGE_AUTO_PREFER_DEVICE;
    auto buffer = std::make_shared<Buffer>(info, allocation);
    buffer->UploadVector(data);
    return buffer;
  };
  auto layout = std::make_shared<DescriptorSetLayout>();
  for (uint32_t i = 0; i < 5; ++i)
    layout->PushDescriptorBinding(i, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_COMPUTE_BIT, 0);
  layout->Initialize();
  auto set = std::make_shared<DescriptorSet>(layout);
  const auto particle_buffer = make_buffer(particles);
  const auto head_buffer = make_buffer(heads);
  const auto next_buffer = make_buffer(next);
  set->UpdateBufferDescriptorBinding(0, particle_buffer);
  set->UpdateBufferDescriptorBinding(1, head_buffer);
  set->UpdateBufferDescriptorBinding(2, next_buffer);
  set->UpdateBufferDescriptorBinding(3, air.GetHeads());
  set->UpdateBufferDescriptorBinding(4, air.GetAddresses());
  auto update = std::make_shared<ComputePipeline>();
  update->compute_shader = Shader::CreateTemporary(ShaderType::Compute, Platform::GetShaderGlobalDefines(),
                                                   "./EcoSysLabResources/Shaders/Compute/Wind/UpdateAir.slang");
  update->descriptor_set_layouts = {layout};
  struct Parameters {
    uint32_t cell_count, bucket_count;
    float cell_size, time_step, coupling_rate, friction, temperature, humidity, pressure, particle_cell_size;
  };
  update->push_constant_ranges.push_back({VK_SHADER_STAGE_COMPUTE_BIT, 0, sizeof(Parameters)});
  update->Initialize();
  ASSERT_TRUE(update->Initialized());
  evo_engine::PlatformLifecycleTestAccess::PreUpdate();
  Platform::RecordCommandsMainQueue([&](VkCommandBuffer command) {
    update->Bind(command);
    update->BindDescriptorSet(command, 0, set->GetVkDescriptorSet());
    update->PushConstant(command, 0,
                         Parameters{air.GetCellCount(), bucket_count, 1, 0.016f, 64, 0, 293.15f, 0.007f, 101325.0f, 1});
    update->Dispatch(command, (air.GetCellCount() + 63) / 64, 1, 1);
    Platform::EverythingBarrier(command);
  });
  evo_engine::PlatformLifecycleTestAccess::LateUpdate();
  Platform::WaitForFrameSubmissions("air cell deposition");
  auto wind_set = std::make_shared<DescriptorSet>(physics->GetWindDescriptorSetLayout());
  wind_set->UpdateBufferDescriptorBinding(0, air.GetHeads());
  wind_set->UpdateBufferDescriptorBinding(1, air.GetAddresses());
  auto windy = component.physics_settings;
  windy.gravity = {};
  windy.wind_drag = 20.0f;
  auto calm = windy;
  calm.wind_drag = 0;
  for (int i = 0; i < 60; ++i) {
    evo_engine::PlatformLifecycleTestAccess::PreUpdate();
    physics->Step({{windy, true}, {calm, true}, {windy, false}},
                  {wind_set, air.GetCellCount(), GpuSpatialGrid::bucket_count, 1});
    evo_engine::PlatformLifecycleTestAccess::LateUpdate();
  }
  Platform::WaitForFrameSubmissions("air cell branch force");
  std::vector<DynamicTreeSkeleton::GpuRestState::Vertex> vertices;
  physics->GetVertexBuffer()->DownloadVector(vertices, rest.vertices.size() * 3);
  const uint32_t tip = rest.rods[rest.node_rods.back()].links.y;
  EXPECT_GT(vertices[tip].position_inverse_mass.z - vertices[tip + rest.vertices.size()].position_inverse_mass.z,
            0.001f);
  EXPECT_NEAR(vertices[tip + 2 * rest.vertices.size()].position_inverse_mass.z,
              rest.vertices[tip].position_inverse_mass.z, 1e-4f);
  EXPECT_FLOAT_EQ(tree.shoot_model.PeekShootSkeleton().PeekNode(4).info.GetGlobalEndPosition().z,
                  rest.vertices[tip].position_inverse_mass.z);
  WindParticleField field;
  field.ResetAirGrid({rest, rest, rest}, nullptr, 0.2f);
  const uint32_t coarse_cells = field.GetAirCellCount();
  field.ResetAirGrid({rest, rest, rest}, physics, 0.1f);
  EXPECT_FLOAT_EQ(field.GetAirCellSize(), 0.1f);
  EXPECT_GT(field.GetAirCellCount(), coarse_cells);
  EXPECT_GT(field.GetAirCellCount(), 0u);
  std::vector<DynamicTreeSkeleton::GpuRestState::Vertex> after_reset;
  physics->GetVertexBuffer()->DownloadVector(after_reset, vertices.size());
  for (size_t i = 0; i < vertices.size(); ++i)
    EXPECT_EQ(after_reset[i].position_inverse_mass, vertices[i].position_inverse_mass);
  air.Reset();
  EXPECT_EQ(air.GetCellCount(), 0u);
}

TEST(GpuTreePhysics, AirGridResetRegistersEveryTreeAtFineResolution) {
  ScopedTreeGpuPlatform platform;
  Tree tree;
  tree.shoot_model.Initialize(MakeGpuPhysicsChain());
  DynamicTreeSkeleton component;
  ASSERT_TRUE(component.InitializePhysics(tree));
  const auto rest = component.ExportGpuRestState();
  std::vector<DynamicTreeSkeleton::GpuRestState> states;
  for (int i = 0; i < 4; ++i) {
    auto state = rest;
    const float offset = 20.0f * i;
    state.initial_model[3].x += offset;
    for (auto& vertex : state.vertices)
      vertex.position_inverse_mass.x += offset;
    states.push_back(std::move(state));
  }
  auto physics = std::make_shared<GpuTreePhysics>();
  ASSERT_TRUE(physics->Initialize(states));
  WindParticleField single;
  single.ResetAirGrid({states.front()}, nullptr, 0.1f);
  WindParticleField fine;
  fine.ResetAirGrid({}, nullptr, 0.01f);
  EXPECT_FLOAT_EQ(fine.GetAirCellSize(), 0.05f);
  fine.ResetAirGrid({states.front()}, nullptr, 0.05f);
  EXPECT_FLOAT_EQ(fine.GetAirCellSize(), 0.05f);
  EXPECT_GT(fine.GetAirCellCount(), single.GetAirCellCount() * 3);
  const uint32_t initial_fine_cells = fine.GetAirCellCount();
  glm::mat4 moved = states.front().initial_model;
  moved[3].x += 5.0f;
  fine.RegisterTreeModels({moved});
  EXPECT_GT(fine.GetAirCellCount(), initial_fine_cells);
  WindParticleField forest;
  forest.ResetAirGrid(states, physics, 0.1f);
  EXPECT_GT(forest.GetAirCellCount(), single.GetAirCellCount() * 3.5f);
}

TEST(GpuTreePhysics, FineAirCellReadsParticleGridBucket) {
  ScopedTreeGpuPlatform platform;
  GpuSpatialGrid air(0.1f);
  ASSERT_TRUE(air.RegisterBox({2.05f, 0.05f, 0.05f}, {2.05f, 0.05f, 0.05f}));
  struct Particle {
    glm::vec4 position_age, velocity_lifetime, collision;
  };
  VkBufferCreateInfo info{VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO};
  info.usage = VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT;
  info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
  VmaAllocationCreateInfo allocation{};
  allocation.usage = VMA_MEMORY_USAGE_AUTO_PREFER_DEVICE;
  const auto make_buffer = [&](const auto& data) {
    info.size = data.size() * sizeof(data[0]);
    auto buffer = std::make_shared<Buffer>(info, allocation);
    buffer->UploadVector(data);
    return buffer;
  };
  const auto particles =
      make_buffer(std::vector<Particle>{{{2.05f, 0.05f, 0.05f, 0}, {0, 0, 5, 3}, {0.1f, 1, 0.007f, 293.15f}}});
  constexpr uint32_t bucket_count = 128;
  std::vector<uint32_t> heads(bucket_count, UINT32_MAX);
  heads[(2u * 73856093u) % bucket_count] = 0;
  const auto head_buffer = make_buffer(heads);
  const auto next_buffer = make_buffer(std::vector<uint32_t>{UINT32_MAX});
  auto layout = std::make_shared<DescriptorSetLayout>();
  for (uint32_t i = 0; i < 5; ++i)
    layout->PushDescriptorBinding(i, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_COMPUTE_BIT, 0);
  layout->Initialize();
  auto set = std::make_shared<DescriptorSet>(layout);
  set->UpdateBufferDescriptorBinding(0, particles);
  set->UpdateBufferDescriptorBinding(1, head_buffer);
  set->UpdateBufferDescriptorBinding(2, next_buffer);
  set->UpdateBufferDescriptorBinding(3, air.GetHeads());
  set->UpdateBufferDescriptorBinding(4, air.GetAddresses());
  auto update = std::make_shared<ComputePipeline>();
  update->compute_shader = Shader::CreateTemporary(ShaderType::Compute, Platform::GetShaderGlobalDefines(),
                                                   "./EcoSysLabResources/Shaders/Compute/Wind/UpdateAir.slang");
  update->descriptor_set_layouts = {layout};
  struct Parameters {
    uint32_t cell_count, bucket_count;
    float cell_size, time_step, coupling_rate, friction, temperature, humidity, pressure, particle_cell_size;
  };
  update->push_constant_ranges.push_back({VK_SHADER_STAGE_COMPUTE_BIT, 0, sizeof(Parameters)});
  update->Initialize();
  ASSERT_TRUE(update->Initialized());
  evo_engine::PlatformLifecycleTestAccess::PreUpdate();
  Platform::RecordCommandsMainQueue([&](VkCommandBuffer command) {
    update->Bind(command);
    update->BindDescriptorSet(command, 0, set->GetVkDescriptorSet());
    update->PushConstant(command, 0, Parameters{1, bucket_count, 0.1f, 0.016f, 64, 0, 293.15f, 0.007f, 101325.0f, 1});
    update->Dispatch(command, 1, 1, 1);
  });
  evo_engine::PlatformLifecycleTestAccess::LateUpdate();
  Platform::WaitForFrameSubmissions("fine air cell deposition");
  std::vector<glm::vec4> value;
  air.GetValueChunk(0)->DownloadVector(value, 2);
  EXPECT_GT(value[0].z, 1.0f);
}

TEST(GpuTreePhysics, WindEmissionFillsTheSpaceBehindBlower) {
  ScopedTreeGpuPlatform platform;
  struct Particle {
    glm::vec4 position_age;
    glm::vec4 velocity_lifetime;
    glm::vec4 collision;
  };
  struct EmitParameters {
    glm::mat4 model{1.0f};
    glm::vec4 speed_spread_lifetime{10.0f, 0.0f, 3.0f, 0.1f};
    glm::vec4 collision{0.3f, 2.0f, 0.01f, 303.15f};
    glm::uvec4 base_count_seed_capacity{0, 64, 17, 64};
  };
  auto layout = std::make_shared<DescriptorSetLayout>();
  layout->PushDescriptorBinding(0, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_COMPUTE_BIT, 0);
  layout->Initialize();
  VkBufferCreateInfo info{VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO};
  info.size = 64 * sizeof(Particle);
  info.usage = VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT;
  info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
  VmaAllocationCreateInfo allocation{};
  allocation.usage = VMA_MEMORY_USAGE_AUTO_PREFER_DEVICE;
  auto buffer = std::make_shared<Buffer>(info, allocation);
  auto descriptor = std::make_shared<DescriptorSet>(layout);
  descriptor->UpdateBufferDescriptorBinding(0, buffer);
  auto pipeline = std::make_shared<ComputePipeline>();
  pipeline->compute_shader = Shader::CreateTemporary(ShaderType::Compute, Platform::GetShaderGlobalDefines(),
                                                     "./EcoSysLabResources/Shaders/Compute/Wind/Emit.slang");
  pipeline->descriptor_set_layouts = {layout};
  pipeline->push_constant_ranges.push_back({VK_SHADER_STAGE_COMPUTE_BIT, 0, sizeof(EmitParameters)});
  pipeline->Initialize();
  ASSERT_TRUE(pipeline->Initialized());
  evo_engine::PlatformLifecycleTestAccess::PreUpdate();
  Platform::RecordCommandsMainQueue([&](VkCommandBuffer command) {
    pipeline->Bind(command);
    pipeline->BindDescriptorSet(command, 0, descriptor->GetVkDescriptorSet());
    pipeline->PushConstant(command, 0, EmitParameters{});
    pipeline->Dispatch(command, 1, 1, 1);
  });
  evo_engine::PlatformLifecycleTestAccess::LateUpdate();
  Platform::WaitForFrameSubmissions("GPU wind emission spacing");
  std::vector<Particle> particles;
  buffer->DownloadVector(particles, 64);
  bool near_surface = false;
  bool far_behind = false;
  for (const auto& particle : particles) {
    EXPECT_GE(particle.position_age.z, -1.0f);
    EXPECT_LE(particle.position_age.z, 0.0f);
    EXPECT_FLOAT_EQ(particle.collision.x, 0.3f);
    EXPECT_FLOAT_EQ(particle.collision.y, 2.0f);
    EXPECT_FLOAT_EQ(particle.collision.z, 0.01f);
    EXPECT_FLOAT_EQ(particle.collision.w, 303.15f);
    near_surface |= particle.position_age.z > -0.2f;
    far_behind |= particle.position_age.z < -0.8f;
  }
  EXPECT_TRUE(near_surface);
  EXPECT_TRUE(far_behind);
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
