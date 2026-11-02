#include <gtest/gtest.h>
#include <algorithm>
#include <chrono>
#include <fstream>
#include <glm/gtx/quaternion.hpp>
#include <iostream>

#include "Application.hpp"
#include "ApplicationInitializationSettings.hpp"
#include "GeometryStorage.hpp"
#include "GpuSpatialGrid.hpp"
#include "GpuTreePhysics.hpp"
#include "Jobs.hpp"
#include "MeshFormation/GpuTreeMeshData.hpp"
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

TEST(TreeDebugRendering, SharedMeshStagesCompileWithDebugOutputs) {
  ScopedTreeGpuPlatform platform;
  const std::vector<std::pair<ShaderType, std::filesystem::path>> stages{
      {ShaderType::Task, "EcoSysLabResources/Shaders/Graphics/Task/TreeMesh.slang"},
      {ShaderType::Mesh, "EcoSysLabResources/Shaders/Graphics/Mesh/TreeMesh.slang"},
      {ShaderType::Fragment, "EcoSysLabResources/Shaders/Graphics/Fragment/TreeDebug.slang"}};
  for (const auto& [type, path] : stages) {
    std::ifstream file(path);
    ASSERT_TRUE(file.is_open()) << path;
    const std::string source = "#define EE_TREE_DEBUG 1\n" +
                               std::string(std::istreambuf_iterator<char>(file), std::istreambuf_iterator<char>());
    std::vector<uint32_t> binaries;
    ASSERT_TRUE(Shader::CompileToSpirv(type, source, binaries, path)) << path;
    EXPECT_FALSE(binaries.empty());
  }
}

TEST(TreeDebugRendering, PickPixelReadbackUsesSdkDispatch) {
  ScopedTreeGpuPlatform platform;
  VkImageCreateInfo image_info{VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO};
  image_info.imageType = VK_IMAGE_TYPE_2D;
  image_info.format = VK_FORMAT_R32G32B32A32_UINT;
  image_info.extent = {2, 2, 1};
  image_info.mipLevels = image_info.arrayLayers = 1;
  image_info.samples = VK_SAMPLE_COUNT_1_BIT;
  image_info.usage = VK_IMAGE_USAGE_TRANSFER_SRC_BIT | VK_IMAGE_USAGE_TRANSFER_DST_BIT;
  Image image(image_info);
  VkBufferCreateInfo buffer_info{VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO};
  buffer_info.size = sizeof(glm::uvec4);
  buffer_info.usage = VK_BUFFER_USAGE_TRANSFER_DST_BIT;
  VmaAllocationCreateInfo host{};
  host.usage = VMA_MEMORY_USAGE_AUTO_PREFER_HOST;
  host.flags = VMA_ALLOCATION_CREATE_HOST_ACCESS_RANDOM_BIT | VMA_ALLOCATION_CREATE_MAPPED_BIT;
  host.requiredFlags = VK_MEMORY_PROPERTY_HOST_COHERENT_BIT;
  Buffer readback(buffer_info, host);
  std::shared_ptr<FrameSubmissionState> submission;
  evo_engine::PlatformLifecycleTestAccess::PreUpdate();
  Platform::RecordCommandsMainQueue([&](VkCommandBuffer command) {
    image.TransitImageLayout(command, VK_IMAGE_LAYOUT_GENERAL);
    VkClearColorValue color{};
    color.uint32[0] = 17;
    color.uint32[1] = 1;
    color.uint32[2] = 0x3f000000;
    const VkImageSubresourceRange range{VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1};
    Platform::ClearColorImage(command, image, color, 1, &range);
    image.TransitImageLayout(command, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL);
    VkBufferImageCopy copy{};
    copy.imageSubresource = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 0, 1};
    copy.imageOffset = {1, 1, 0};
    copy.imageExtent = {1, 1, 1};
    Platform::CopyImageToBuffer(command, image, readback, copy);
    Platform::EverythingBarrier(command);
    submission = Platform::TrackCurrentFrameSubmission();
  });
  evo_engine::PlatformLifecycleTestAccess::LateUpdate();
  Platform::WaitForFrameSubmissions("Tree debug pick readback test");
  ASSERT_EQ(submission->status, FrameSubmissionState::Status::Submitted);
  ASSERT_NE(readback.GetVmaAllocationInfo().pMappedData, nullptr);
  EXPECT_EQ(*static_cast<const glm::uvec4*>(readback.GetVmaAllocationInfo().pMappedData),
            glm::uvec4(17, 1, 0x3f000000, 0));
}

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

TEST(GpuTreePhysics, FineAirCellsFollowCurrentBentPose) {
  ScopedTreeGpuPlatform platform;
  DynamicTreeSkeleton::GpuRestState state;
  state.vertices = {{{2, 0, 0, 0}, {0, 0, 0, 0.1f}, {0, 0, 0, 0}, {0, 1, 0, 0}},
                    {{3, 0, 0, 1}, {0, 0, 0, 0.1f}, {1, 0, 0, 0}, {1, 1, 0, 0}}};
  state.rods = {{{0, 1, 0, 1}, {1, 1, 1, 0.1f}, {0, 0, 0, 1}, {0, 0, 0, 1}, {0, 0, 0, 0}}};
  state.vertex_rods = {0, 0};
  state.node_rods = {0};
  GpuTreePhysics physics;
  ASSERT_TRUE(physics.Initialize(state));
  GpuSpatialGrid air(0.05f);
  ASSERT_TRUE(air.RegisterBox({2.4f, -0.1f, -0.1f}, {2.6f, 0.1f, 0.1f}));
  struct AirValue {
    glm::vec4 velocity_temperature;
    glm::vec4 humidity_strength_density;
  };
  air.GetValueChunk(0)->UploadVector(
      std::vector<AirValue>(GpuSpatialGrid::chunk_capacity, {{0, 0, 5, 293.15f}, {0, 1, 1.225f, 0}}));
  auto wind_set = std::make_shared<DescriptorSet>(physics.GetWindDescriptorSetLayout());
  wind_set->UpdateBufferDescriptorBinding(0, air.GetHeads());
  wind_set->UpdateBufferDescriptorBinding(1, air.GetAddresses());
  DynamicTreeSkeleton::PhysicsSettings settings;
  settings.gravity = {};
  settings.stretch_stiffness = 0;
  evo_engine::PlatformLifecycleTestAccess::PreUpdate();
  physics.Step({{settings, true}}, {wind_set, air.GetCellCount(), GpuSpatialGrid::bucket_count, air.GetCellSize()});
  evo_engine::PlatformLifecycleTestAccess::LateUpdate();
  Platform::WaitForFrameSubmissions("fine air follows bent tree");
  std::vector<DynamicTreeSkeleton::GpuRestState::Vertex> vertices;
  physics.GetVertexBuffer()->DownloadVector(vertices, 2);
  EXPECT_GT(vertices[1].position_inverse_mass.z, 1e-5f);
}

TEST(GpuTreePhysics, ForestTreesOwnLocalTopologyBuffers) {
  ScopedTreeGpuPlatform platform;
  Tree tree;
  tree.shoot_model.Initialize(MakeGpuPhysicsFork());
  DynamicTreeSkeleton component;
  ASSERT_TRUE(component.InitializePhysics(tree));
  const auto rest = component.ExportGpuRestState();
  GpuTreePhysics physics;
  ASSERT_TRUE(physics.Initialize(std::vector<DynamicTreeSkeleton::GpuRestState>{rest, rest}));
  EXPECT_NE(physics.GetVertexBuffer(0), physics.GetVertexBuffer(1));
  EXPECT_NE(physics.GetRodBuffer(0)->GetDeviceAddress(), physics.GetRodBuffer(1)->GetDeviceAddress());
  for (uint32_t id = 0; id < 2; ++id) {
    std::vector<DynamicTreeSkeleton::GpuRestState::Rod> rods;
    physics.GetRodBuffer(id)->DownloadVector(rods, rest.rods.size());
    for (size_t i = 0; i < rods.size(); ++i)
      EXPECT_EQ(rods[i].links, rest.rods[i].links);
    std::vector<uint32_t> mapping;
    physics.GetNodeRodBuffer(id)->DownloadVector(mapping, rest.node_rods.size());
    EXPECT_EQ(mapping, rest.node_rods);
  }
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
  physics.GetVertexBuffer(0)->DownloadVector(vertices, 2);
  std::vector<DynamicTreeSkeleton::GpuRestState::Vertex> second_vertices;
  physics.GetVertexBuffer(1)->DownloadVector(second_vertices, 2);
  EXPECT_LT(vertices[1].position_inverse_mass.y, -0.001f);
  EXPECT_FLOAT_EQ(second_vertices[1].position_inverse_mass.y, 0.0f);
  const float first_y = vertices[1].position_inverse_mass.y;
  for (int i = 0; i < 16; ++i) {
    evo_engine::PlatformLifecycleTestAccess::PreUpdate();
    physics.Step(std::vector<GpuTreePhysics::TreeStep>{{settings, false}, {settings, true}});
    evo_engine::PlatformLifecycleTestAccess::LateUpdate();
  }
  Platform::WaitForFrameSubmissions("GPU forest physics second tree");
  physics.GetVertexBuffer(0)->DownloadVector(vertices, 2);
  physics.GetVertexBuffer(1)->DownloadVector(second_vertices, 2);
  EXPECT_FLOAT_EQ(vertices[1].position_inverse_mass.y, first_y);
  EXPECT_LT(second_vertices[1].position_inverse_mass.y, -0.001f);
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
  physics.GetVertexBuffer(0)->DownloadVector(vertices, rest.vertices.size());
  std::vector<DynamicTreeSkeleton::GpuRestState::Vertex> stationary;
  physics.GetVertexBuffer(1)->DownloadVector(stationary, rest.vertices.size());
  for (size_t i = 0; i < rest.vertices.size(); ++i) {
    const glm::vec3 expected =
        glm::vec3(moved.value * glm::vec4(glm::vec3(rest.vertices[i].position_inverse_mass), 1.0f));
    EXPECT_LT(glm::length(glm::vec3(vertices[i].position_inverse_mass) - expected), 1e-4f);
    EXPECT_LT(
        glm::length(glm::vec3(stationary[i].position_inverse_mass) - glm::vec3(rest.vertices[i].position_inverse_mass)),
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
  for (uint32_t id = 0; id < 3; ++id) {
    std::vector<DynamicTreeSkeleton::GpuRestState::Vertex> local;
    physics->GetVertexBuffer(id)->DownloadVector(local, rest.vertices.size());
    vertices.insert(vertices.end(), local.begin(), local.end());
  }
  const uint32_t tip = rest.rods[rest.node_rods.back()].links.y;
  EXPECT_GT(vertices[tip].position_inverse_mass.z - vertices[tip + rest.vertices.size()].position_inverse_mass.z,
            0.001f);
  EXPECT_NEAR(vertices[tip + 2 * rest.vertices.size()].position_inverse_mass.z,
              rest.vertices[tip].position_inverse_mass.z, 1e-4f);
  EXPECT_FLOAT_EQ(tree.shoot_model.PeekShootSkeleton().PeekNode(4).info.GetGlobalEndPosition().z,
                  rest.vertices[tip].position_inverse_mass.z);
  WindParticleField field;
  field.ResetAirGrid({rest, rest, rest}, nullptr, {}, 0.2f);
  const uint32_t coarse_cells = field.GetAirCellCount();
  field.ResetAirGrid({rest, rest, rest}, physics, {rest.initial_model, rest.initial_model, rest.initial_model}, 0.1f);
  EXPECT_FLOAT_EQ(field.GetAirCellSize(), 0.1f);
  EXPECT_GT(field.GetAirCellCount(), coarse_cells);
  EXPECT_GT(field.GetAirCellCount(), 0u);
  std::vector<DynamicTreeSkeleton::GpuRestState::Vertex> after_reset;
  for (uint32_t id = 0; id < 3; ++id) {
    std::vector<DynamicTreeSkeleton::GpuRestState::Vertex> local;
    physics->GetVertexBuffer(id)->DownloadVector(local, rest.vertices.size());
    after_reset.insert(after_reset.end(), local.begin(), local.end());
  }
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
  single.ResetAirGrid({states.front()}, nullptr, {}, 0.1f);
  WindParticleField fine;
  fine.ResetAirGrid({}, nullptr, {}, 0.01f);
  EXPECT_FLOAT_EQ(fine.GetAirCellSize(), 0.05f);
  fine.ResetAirGrid({states.front()}, nullptr, {}, 0.05f);
  EXPECT_FLOAT_EQ(fine.GetAirCellSize(), 0.05f);
  EXPECT_GT(fine.GetAirCellCount(), single.GetAirCellCount() * 3);
  const uint32_t initial_fine_cells = fine.GetAirCellCount();
  glm::mat4 moved = states.front().initial_model;
  moved[3].x += 5.0f;
  fine.RegisterTreeModels({moved});
  EXPECT_GT(fine.GetAirCellCount(), initial_fine_cells);
  WindParticleField forest;
  forest.ResetAirGrid(
      states, physics,
      {states[0].initial_model, states[1].initial_model, states[2].initial_model, states[3].initial_model}, 0.1f);
  EXPECT_GT(forest.GetAirCellCount(), single.GetAirCellCount() * 3.5f);
}

TEST(GpuTreePhysics, AirGridResetUsesCurrentModelForDownloadedPose) {
  ScopedTreeGpuPlatform platform;
  Tree tree;
  tree.shoot_model.Initialize(MakeGpuPhysicsChain());
  DynamicTreeSkeleton component;
  ASSERT_TRUE(component.InitializePhysics(tree));
  const auto rest = component.ExportGpuRestState();
  auto physics = std::make_shared<GpuTreePhysics>();
  ASSERT_TRUE(physics->Initialize(rest));
  GlobalTransform moved;
  moved.SetPosition({5.0f, 0.0f, 0.0f});
  evo_engine::PlatformLifecycleTestAccess::PreUpdate();
  physics->Step(std::vector<GpuTreePhysics::TreeStep>{{component.physics_settings, false, moved}});
  evo_engine::PlatformLifecycleTestAccess::LateUpdate();
  Platform::WaitForFrameSubmissions("moved air grid reset");

  WindParticleField field;
  field.ResetAirGrid({rest}, physics, {moved.value}, 0.2f);
  const uint32_t cells = field.GetAirCellCount();
  ASSERT_GT(cells, 0u);
  field.RegisterTreeModels({moved.value});
  EXPECT_EQ(field.GetAirCellCount(), cells);
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
TEST(GpuTreePhysics, RodDrivenAirMatchesGatherAndClearsContributions) {
  ScopedTreeGpuPlatform platform;
  using State = DynamicTreeSkeleton::GpuRestState;
  std::vector<State::Vertex> vertices;
  std::vector<State::Rod> rods;
  const auto add = [&](glm::vec3 a, glm::vec3 b, float radius) {
    const auto base = static_cast<uint32_t>(vertices.size());
    vertices.push_back({glm::vec4(a, 1), {1, -2, 3, 0}, {}, {}});
    vertices.push_back({glm::vec4(b, 1), {-3, 1, 2, 0}, {}, {}});
    rods.push_back({{base, base + 1, 0, 0}, {glm::length(b - a), 1, 1, radius}, {}, {}, {}});
  };
  add({-0.7f, -0.2f, -0.2f}, {0.7f, 0.2f, 0.2f}, 0.08f);
  add({-0.1f, -0.3f, 0.1f}, {-0.1f, 0.3f, 0.1f}, 0.22f);
  add({0, 0, 0}, {0, 0, 0}, 0.05f);
  add({20, 20, 20}, {21, 21, 21}, 0.1f);  // Unregistered cells must not be created.
  for (int i = 0; i < 64; ++i)
    add({0.001f * i, 0, 0}, {0.001f * i, 0.2f, 0}, 0.01f);
  const auto buffer = [](const auto& data) {
    VkBufferCreateInfo info{VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO};
    info.size = data.size() * sizeof(data[0]);
    info.usage = VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT;
    VmaAllocationCreateInfo allocation{};
    allocation.usage = VMA_MEMORY_USAGE_AUTO_PREFER_DEVICE;
    auto result = std::make_shared<Buffer>(info, allocation);
    result->UploadVector(data);
    return result;
  };
  State local_state;
  local_state.vertices = vertices;
  local_state.rods = rods;
  auto neighbor_state = local_state;
  for (auto& vertex : neighbor_state.vertices)
    vertex.velocity_radius += glm::vec4(2, -3, 4, 0);
  GpuTreePhysics forest;
  ASSERT_TRUE(forest.Initialize(std::vector<State>{local_state, neighbor_state}));
  const auto vertex_base = static_cast<uint32_t>(vertices.size());
  vertices.insert(vertices.end(), neighbor_state.vertices.begin(), neighbor_state.vertices.end());
  for (auto rod : neighbor_state.rods) {
    rod.links.x += vertex_base;
    rod.links.y += vertex_base;
    rods.push_back(rod);
  }
  const auto vertex_buffer = buffer(vertices), rod_buffer = buffer(rods);
  auto layout = std::make_shared<DescriptorSetLayout>();
  for (uint32_t i = 0; i < 6; ++i)
    layout->PushDescriptorBinding(i, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_COMPUTE_BIT, 0);
  layout->Initialize();
  auto forest_layout = std::make_shared<DescriptorSetLayout>();
  for (uint32_t i = 0; i < 8; ++i)
    forest_layout->PushDescriptorBinding(i, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_COMPUTE_BIT, 0);
  forest_layout->Initialize();
  struct Parameters {
    uint32_t cells, rods, buckets, phase;
    float size, dt;
  };
  const auto pipeline = [&](const char* file, uint32_t size, bool batched = false) {
    auto result = std::make_shared<ComputePipeline>();
    result->compute_shader = Shader::CreateTemporary(
        ShaderType::Compute, Platform::GetShaderGlobalDefines() + (batched ? "\n#define EE_FOREST_PHYSICS 1\n" : ""),
        std::filesystem::path("./EcoSysLabResources/Shaders/Compute/Wind") / file);
    result->descriptor_set_layouts = {batched ? forest_layout : layout};
    result->push_constant_ranges.push_back({VK_SHADER_STAGE_COMPUTE_BIT, 0, size});
    result->Initialize();
    return result;
  };
  const auto scatter = pipeline("RodAirScatter.slang", sizeof(Parameters));
  const auto gather = pipeline("RodAir.slang", sizeof(Parameters));
  const auto batched_scatter = pipeline("RodAirScatter.slang", sizeof(Parameters), true);
  ASSERT_TRUE(batched_scatter->Initialized());
  ASSERT_TRUE(pipeline("RodAir.slang", sizeof(Parameters), true)->Initialized());
  ASSERT_TRUE(pipeline("BuildRods.slang", 16, true)->Initialized());
  const auto clear = pipeline("ClearRods.slang", 16), build = pipeline("BuildRods.slang", 16);
  ASSERT_TRUE(scatter->Initialized());
  ASSERT_TRUE(gather->Initialized());
  for (float size : {0.05f, 0.1f, 0.2f}) {
    GpuSpatialGrid grid(size);
    ASSERT_TRUE(grid.RegisterBox({-0.8f, -0.4f, -0.4f}, {0.8f, 0.4f, 0.4f}));
    const auto cell_count = grid.GetCellCount();
    auto sums = buffer(std::vector<glm::vec4>(cell_count, glm::vec4(123)));
    auto heads = buffer(std::vector<uint32_t>(128)), next = buffer(std::vector<uint32_t>(rods.size()));
    auto extent = buffer(std::vector<uint32_t>(1));
    auto set = std::make_shared<DescriptorSet>(layout);
    auto forest_set = std::make_shared<DescriptorSet>(forest_layout);
    set->UpdateBufferDescriptorBinding(0, grid.GetAddresses());
    set->UpdateBufferDescriptorBinding(3, extent);
    set->UpdateBufferDescriptorBinding(4, vertex_buffer);
    set->UpdateBufferDescriptorBinding(5, rod_buffer);
    struct Air {
      glm::vec4 velocity_temperature, other;
    };
    const Air initial{{2, 3, -1, 300}, {0.01f, 1, 1.2f, 0}};
    std::vector<Air> reference;
    for (int mode = 0; mode < 4; ++mode) {
      // The third run has no rods and must discard the previous scatter sums.
      const auto count = mode == 2 ? 0u : static_cast<uint32_t>(rods.size());
      for (uint32_t chunk = 0; chunk * GpuSpatialGrid::chunk_capacity < grid.GetCellCount(); ++chunk)
        grid.GetValueChunk(chunk)->UploadVector(std::vector<Air>(GpuSpatialGrid::chunk_capacity, initial));
      set->UpdateBufferDescriptorBinding(1, mode == 0 ? heads : grid.GetHeads());
      set->UpdateBufferDescriptorBinding(2, mode == 0 ? next : sums);
      evo_engine::PlatformLifecycleTestAccess::PreUpdate();
      auto dispatch_set = set;
      if (mode == 3) {
        forest.PrepareRegistry();
        dispatch_set = forest_set;
        forest_set->UpdateBufferDescriptorBinding(0, grid.GetAddresses());
        forest_set->UpdateBufferDescriptorBinding(1, grid.GetHeads());
        forest_set->UpdateBufferDescriptorBinding(2, sums);
        forest_set->UpdateBufferDescriptorBinding(3, extent);
        forest_set->UpdateBufferDescriptorBinding(4, vertex_buffer);
        forest_set->UpdateBufferDescriptorBinding(5, rod_buffer);
        forest_set->UpdateBufferDescriptorBinding(6, forest.GetRegistryBuffer());
        forest_set->UpdateBufferDescriptorBinding(7, forest.GetRodGroupBuffer());
      }
      const auto dispatch_count = mode == 3 ? forest.GetRodGroupCount() * 64 : count;

      Platform::RecordCommandsMainQueue([&](VkCommandBuffer command) {
        const auto dispatch = [&](const auto& p, const auto& parameters, uint32_t count) {
          p->Bind(command);
          p->BindDescriptorSet(command, 0, dispatch_set->GetVkDescriptorSet());
          p->PushConstant(command, 0, parameters);
          p->Dispatch(command, (count + 63) / 64, 1, 1);
          Platform::EverythingBarrier(command);
        };
        if (mode == 0) {
          struct RodParameters {
            uint32_t particles, rods, buckets;
            float size;
          };
          dispatch(clear, RodParameters{0, count, 128, 1}, 128);
          dispatch(build, RodParameters{0, count, 128, 1}, count);
          struct GatherParameters {
            uint32_t cells, rods, buckets;
            float rod_size, size, dt;
          };
          dispatch(gather, GatherParameters{grid.GetCellCount(), dispatch_count, 128, 1, size, 0.016f},
                   grid.GetCellCount());
        } else {
          for (uint32_t phase = 0; phase < 3; ++phase)
            dispatch(mode == 3 ? batched_scatter : scatter,
                     Parameters{grid.GetCellCount(), dispatch_count, GpuSpatialGrid::bucket_count, phase, size, 0.016f},
                     phase == 1 ? std::max(dispatch_count, 1u) : grid.GetCellCount());
        }
      });
      evo_engine::PlatformLifecycleTestAccess::LateUpdate();
      Platform::WaitForFrameSubmissions("rod air equivalence");
      std::vector<Air> actual;
      for (uint32_t chunk = 0; chunk * GpuSpatialGrid::chunk_capacity < grid.GetCellCount(); ++chunk) {
        std::vector<Air> values;
        grid.GetValueChunk(chunk)->DownloadVector(values, GpuSpatialGrid::chunk_capacity);
        actual.insert(actual.end(), values.begin(), values.end());
      }
      if (mode == 0) {
        reference = actual;
        continue;
      }
      uint32_t affected = 0;
      for (uint32_t cell = 0; cell < grid.GetCellCount(); ++cell) {
        const auto expected = mode != 2 ? reference[cell] : initial;
        for (int axis = 0; axis < 4; ++axis) {
          ASSERT_NEAR(actual[cell].velocity_temperature[axis], expected.velocity_temperature[axis], 2e-5f)
              << "size=" << size << " mode=" << mode << " cell=" << cell;
          EXPECT_EQ(actual[cell].other[axis], initial.other[axis]);
        }
        affected += glm::length(glm::vec3(actual[cell].velocity_temperature - initial.velocity_temperature)) > 1e-5f;
      }
      if (mode != 2)
        EXPECT_GT(affected, 0u);
      EXPECT_EQ(grid.GetCellCount(), cell_count);
    }
  }
}

TEST(GpuTreePhysics, AnalyticBranchShadowDepthHandlesTaperAxialAndMissRays) {
  ScopedTreeGpuPlatform platform;
  for (const auto& stage : {std::pair{ShaderType::Task, "Task"}, std::pair{ShaderType::Mesh, "Mesh"},
                            std::pair{ShaderType::Fragment, "Fragment"}}) {
    const auto shader = Shader::CreateTemporary(
        stage.first, Platform::GetShaderGlobalDefines(),
        std::filesystem::path("./EcoSysLabResources/Shaders/Graphics") / stage.second / "TreeShadow.slang");
    ASSERT_TRUE(shader->Compiled()) << stage.second;
  }
  VkBufferCreateInfo info{VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO};
  info.size = 10 * sizeof(float);
  info.usage = VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT;
  VmaAllocationCreateInfo allocation{};
  allocation.usage = VMA_MEMORY_USAGE_AUTO_PREFER_DEVICE;
  auto values = std::make_shared<Buffer>(info, allocation);
  auto layout = std::make_shared<DescriptorSetLayout>();
  layout->PushDescriptorBinding(0, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_COMPUTE_BIT, 0);
  layout->Initialize();
  auto set = std::make_shared<DescriptorSet>(layout);
  set->UpdateBufferDescriptorBinding(0, values);
  auto pipeline = std::make_shared<ComputePipeline>();
  pipeline->compute_shader = Shader::CreateTemporary(
      ShaderType::Compute, Platform::GetShaderGlobalDefines(),
      std::filesystem::path(__FILE__).parent_path().parent_path() / "Resources/Shaders/Compute/TreeShadowProbe.slang");
  pipeline->descriptor_set_layouts = {layout};
  pipeline->Initialize();
  ASSERT_TRUE(pipeline->Initialized());
  evo_engine::PlatformLifecycleTestAccess::PreUpdate();
  Platform::RecordCommandsMainQueue([&](VkCommandBuffer command) {
    pipeline->Bind(command);
    pipeline->BindDescriptorSet(command, 0, set->GetVkDescriptorSet());
    pipeline->Dispatch(command, 1, 1, 1);
    Platform::EverythingBarrier(command);
  });
  evo_engine::PlatformLifecycleTestAccess::LateUpdate();
  Platform::WaitForFrameSubmissions("analytic branch shadow depth");
  std::vector<float> result;
  values->DownloadVector(result, 10);
  EXPECT_NEAR(result[0], 1.8f, 1e-5f);
  EXPECT_NEAR(result[1], 1.8f, 1e-5f);
  EXPECT_GT(result[2], 1e29f);
  EXPECT_NEAR(result[3], 1.8f, 1e-5f);
  EXPECT_NEAR(result[4], 1.85f, 1e-5f);
  EXPECT_GT(result[5], 1e29f);
  EXPECT_NEAR(result[6], 2.0f - std::sqrt(0.15f * 0.15f - 0.1f * 0.1f), 1e-5f);
  EXPECT_NEAR(result[7], 1.9f, 1e-5f);
  EXPECT_EQ(result[8], 0.0f);
  EXPECT_NEAR(result[9], 1.85f, 1e-5f);
}

TEST(GpuTreePhysics, TreeAddressAccessPreservesLocalIndicesAndLayouts) {
  ScopedTreeGpuPlatform platform;
  const auto make = [](const auto& data) {
    VkBufferCreateInfo info{VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO};
    info.size = data.size() * sizeof(data[0]);
    info.usage = VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT |
                 VK_BUFFER_USAGE_TRANSFER_SRC_BIT | VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT;
    VmaAllocationCreateInfo allocation{};
    allocation.usage = VMA_MEMORY_USAGE_AUTO_PREFER_DEVICE;
    auto result = std::make_shared<Buffer>(info, allocation);
    result->UploadVector(data);
    return result;
  };
  using Vertex = DynamicTreeSkeleton::GpuRestState::Vertex;
  const auto vertices =
      make(std::vector<Vertex>(2, Vertex{{1, 2, 3, 4}, {5, 6, 7, 8}, {9, 10, 11, 12}, {13, 14, 15, 16}}));
  const auto indices = make(std::vector<uint32_t>{19, 23, 29});
  const auto addresses = make(std::vector<uint64_t>{vertices->GetDeviceAddress(), indices->GetDeviceAddress()});
  auto layout = std::make_shared<DescriptorSetLayout>();
  layout->PushDescriptorBinding(0, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_COMPUTE_BIT, 0);
  layout->Initialize();
  auto set = std::make_shared<DescriptorSet>(layout);
  set->UpdateBufferDescriptorBinding(0, addresses);
  auto pipeline = std::make_shared<ComputePipeline>();
  pipeline->compute_shader = Shader::CreateTemporary(ShaderType::Compute, Platform::GetShaderGlobalDefines(),
                                                     std::filesystem::path(__FILE__).parent_path().parent_path() /
                                                         "Resources/Shaders/Compute/TreePhysicsAddressProbe.slang");
  pipeline->descriptor_set_layouts = {layout};
  pipeline->Initialize();
  ASSERT_TRUE(pipeline->Initialized());
  evo_engine::PlatformLifecycleTestAccess::PreUpdate();
  Platform::RecordCommandsMainQueue([&](VkCommandBuffer command) {
    pipeline->Bind(command);
    pipeline->BindDescriptorSet(command, 0, set->GetVkDescriptorSet());
    pipeline->Dispatch(command, 1, 1, 1);
    Platform::EverythingBarrier(command);
  });
  evo_engine::PlatformLifecycleTestAccess::LateUpdate();
  Platform::WaitForFrameSubmissions("tree physics addresses");
  std::vector<Vertex> actual;
  vertices->DownloadVector(actual, 2);
  EXPECT_EQ(actual[0].position_inverse_mass, glm::vec4(1, 2, 3, 4));
  EXPECT_EQ(actual[1].position_inverse_mass, glm::vec4(1, 5, 3, 4));
  EXPECT_EQ(actual[1].incident_rods, glm::uvec4(13, 14, 15, 16));
  std::vector<uint32_t> values;
  indices->DownloadVector(values, 3);
  EXPECT_EQ(values, (std::vector<uint32_t>{19, 23, 30}));
}

TEST(GpuTreePhysics, RebuildingAndRemovingOneTreePreservesNeighborBuffersAndPose) {
  ScopedTreeGpuPlatform platform;
  Tree tree;
  tree.shoot_model.Initialize(MakeGpuPhysicsFork());
  DynamicTreeSkeleton component;
  ASSERT_TRUE(component.InitializePhysics(tree));
  const auto rest = component.ExportGpuRestState();
  GpuTreePhysics physics;
  ASSERT_TRUE(physics.Initialize(std::vector<DynamicTreeSkeleton::GpuRestState>{rest, rest}));
  for (int i = 0; i < 8; ++i) {
    evo_engine::PlatformLifecycleTestAccess::PreUpdate();
    physics.Step(
        std::vector<GpuTreePhysics::TreeStep>{{component.physics_settings, true}, {component.physics_settings, true}});
    evo_engine::PlatformLifecycleTestAccess::LateUpdate();
  }
  Platform::WaitForFrameSubmissions("neighbor pose before rebuild");
  const auto neighbor = physics.GetTreeState(1);
  std::vector<DynamicTreeSkeleton::GpuRestState::Vertex> before, after;
  physics.GetVertexBuffer(1)->DownloadVector(before, rest.vertices.size());
  EXPECT_EQ(physics.RegisterTree(rest, 0), 0u);
  EXPECT_EQ(physics.GetTreeState(1), neighbor);
  physics.GetVertexBuffer(1)->DownloadVector(after, rest.vertices.size());
  for (size_t i = 0; i < before.size(); ++i) {
    EXPECT_EQ(before[i].position_inverse_mass, after[i].position_inverse_mass);
    EXPECT_EQ(before[i].velocity_radius, after[i].velocity_radius);
  }
  physics.RemoveTree(0);
  EXPECT_EQ(physics.GetActiveTreeIds(), std::vector<uint32_t>{1});
  EXPECT_EQ(physics.GetTreeState(1), neighbor);
  const auto added = physics.RegisterTree(rest);
  EXPECT_EQ(added, 2u);
  physics.SetActiveTrees({1, added});
  evo_engine::PlatformLifecycleTestAccess::PreUpdate();
  physics.Step(
      std::vector<GpuTreePhysics::TreeStep>{{component.physics_settings, false}, {component.physics_settings, true}});
  evo_engine::PlatformLifecycleTestAccess::LateUpdate();
  Platform::WaitForFrameSubmissions("neighbor pose after membership changes");
  physics.GetVertexBuffer(1)->DownloadVector(after, rest.vertices.size());
  for (size_t i = 0; i < before.size(); ++i)
    EXPECT_EQ(before[i].position_inverse_mass, after[i].position_inverse_mass);
}

TEST(GpuTreePhysics, InactiveTreeKeepsItsOwnPingPongStateAcrossOddIterations) {
  ScopedTreeGpuPlatform platform;
  Tree tree;
  tree.shoot_model.Initialize(MakeGpuPhysicsFork());
  DynamicTreeSkeleton component;
  ASSERT_TRUE(component.InitializePhysics(tree));
  const auto rest = component.ExportGpuRestState();
  GpuTreePhysics physics;
  ASSERT_TRUE(physics.Initialize(std::vector<DynamicTreeSkeleton::GpuRestState>{rest, rest}));
  auto settings = component.physics_settings;
  settings.substeps = 1;
  settings.iterations = 3;
  physics.SetActiveTrees({0});
  evo_engine::PlatformLifecycleTestAccess::PreUpdate();
  physics.Step(std::vector<GpuTreePhysics::TreeStep>{{settings, true}});
  evo_engine::PlatformLifecycleTestAccess::LateUpdate();
  Platform::WaitForFrameSubmissions("one active tree odd parity");
  EXPECT_EQ(physics.GetTreeState(0)->vertex_index, 1u);
  EXPECT_EQ(physics.GetTreeState(1)->vertex_index, 0u);
  physics.SetActiveTrees({0, 1});
  evo_engine::PlatformLifecycleTestAccess::PreUpdate();
  physics.Step(std::vector<GpuTreePhysics::TreeStep>{{settings, true}, {settings, false}});
  evo_engine::PlatformLifecycleTestAccess::LateUpdate();
  Platform::WaitForFrameSubmissions("rejoined tree parity");
  std::vector<DynamicTreeSkeleton::GpuRestState::Vertex> actual;
  physics.GetVertexBuffer(1)->DownloadVector(actual, rest.vertices.size());
  for (size_t i = 0; i < actual.size(); ++i)
    EXPECT_EQ(actual[i].position_inverse_mass, rest.vertices[i].position_inverse_mass);
}

TEST(GpuTreePhysics, DeformationUsesEachTreesLocalNodeMapAndDynamicPose) {
  ScopedTreeGpuPlatform platform;
  Tree tree;
  tree.shoot_model.Initialize(MakeGpuPhysicsFork());
  DynamicTreeSkeleton component;
  GlobalTransform model;
  model.value = glm::translate(glm::mat4(1), glm::vec3(7, 2, -3));
  ASSERT_TRUE(component.InitializePhysics(tree, model));
  const auto rest = component.ExportGpuRestState();
  GpuTreePhysics physics;
  ASSERT_TRUE(physics.Initialize(std::vector<DynamicTreeSkeleton::GpuRestState>{rest, rest}));
  std::vector<GpuTreeNode> nodes;
  AppendGpuTreeNodes(tree.shoot_model.PeekShootSkeleton(), tree.tree_mesh_generator_settings, nullptr, 0, true, nodes);
  const auto make = [&](const auto& data) {
    VkBufferCreateInfo info{VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO};
    info.size = data.size() * sizeof(data[0]);
    info.usage =
        VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_TRANSFER_SRC_BIT;
    VmaAllocationCreateInfo allocation{};
    allocation.usage = VMA_MEMORY_USAGE_AUTO_PREFER_DEVICE;
    auto buffer = std::make_shared<Buffer>(info, allocation);
    buffer->UploadVector(data);
    return buffer;
  };
  const auto rest_nodes = make(nodes);
  std::array<std::shared_ptr<Buffer>, 2> output{make(nodes), make(nodes)};
  auto layout = std::make_shared<DescriptorSetLayout>();
  for (uint32_t i = 0; i < 5; ++i)
    layout->PushDescriptorBinding(i, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_COMPUTE_BIT, 0);
  layout->Initialize();
  auto pipeline = std::make_shared<ComputePipeline>();
  pipeline->compute_shader = Shader::CreateTemporary(ShaderType::Compute, Platform::GetShaderGlobalDefines(),
                                                     "./EcoSysLabResources/Shaders/Compute/DeformTreeNodes.slang");
  pipeline->descriptor_set_layouts = {layout};
  pipeline->push_constant_ranges.push_back({VK_SHADER_STAGE_COMPUTE_BIT, 0, sizeof(GpuTreeDeformParameters)});
  pipeline->Initialize();
  ASSERT_TRUE(pipeline->Initialized());
  auto moving = component.physics_settings;
  moving.gravity = {4, -9.81f, 0};
  std::array<std::shared_ptr<DescriptorSet>, 2> sets;
  evo_engine::PlatformLifecycleTestAccess::PreUpdate();
  physics.Step(std::vector<GpuTreePhysics::TreeStep>{{moving, true, model}, {moving, false, model}});
  for (uint32_t id = 0; id < 2; ++id) {
    sets[id] = std::make_shared<DescriptorSet>(layout);
    sets[id]->UpdateBufferDescriptorBinding(0, output[id]);
    sets[id]->UpdateBufferDescriptorBinding(1, physics.GetVertexBuffer(id));
    sets[id]->UpdateBufferDescriptorBinding(2, physics.GetRodBuffer(id));
    sets[id]->UpdateBufferDescriptorBinding(3, physics.GetNodeRodBuffer(id));
    sets[id]->UpdateBufferDescriptorBinding(4, rest_nodes);
    const GpuTreeDeformParameters push{
        0, static_cast<uint32_t>(nodes.size()), 0, 0, glm::vec4(model.GetPosition(), 0), glm::vec4(1), {0, 0, 0, 1}};
    Platform::RecordCommandsMainQueue([&](VkCommandBuffer command) {
      pipeline->Bind(command);
      pipeline->BindDescriptorSet(command, 0, sets[id]->GetVkDescriptorSet());
      pipeline->PushConstant(command, 0, push);
      pipeline->Dispatch(command, 1, 1, 1);
      Platform::EverythingBarrier(command);
    });
  }
  evo_engine::PlatformLifecycleTestAccess::LateUpdate();
  Platform::WaitForFrameSubmissions("tree-local deformation");
  for (uint32_t id = 0; id < 2; ++id) {
    std::vector<GpuTreeNode> actual;
    output[id]->DownloadVector(actual, nodes.size());
    std::vector<DynamicTreeSkeleton::GpuRestState::Vertex> vertices;
    physics.GetVertexBuffer(id)->DownloadVector(vertices, rest.vertices.size());
    bool changed = false;
    for (size_t i = 0; i < vertices.size(); ++i)
      changed |=
          glm::length(glm::vec3(vertices[i].position_inverse_mass - rest.vertices[i].position_inverse_mass)) > 1e-7f;
    EXPECT_EQ(changed, id == 0);
    for (size_t i = 0; i < nodes.size(); ++i) {
      const auto rod = rest.node_rods[nodes[i].foliage_meta.x];
      if (rod == UINT32_MAX)
        continue;
      const auto& links = rest.rods[rod].links;
      const glm::vec3 start(vertices[links.x].position_inverse_mass), end(vertices[links.y].position_inverse_mass);
      EXPECT_LT(glm::length(glm::vec3(actual[i].position_thickness) - (start - model.GetPosition())), 1e-5f);
      EXPECT_LT(glm::length(glm::vec3(actual[i].direction_length) * actual[i].direction_length.w - (end - start)),
                1e-5f);
      EXPECT_EQ(actual[i].position_thickness.w, nodes[i].position_thickness.w);
    }
  }
}
