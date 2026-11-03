#include <gtest/gtest.h>
#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <glm/gtx/quaternion.hpp>
#include <iostream>
#include <optional>

#include "Application.hpp"
#include "ApplicationInitializationSettings.hpp"
#include "EcoSysLabSystem.hpp"
#include "EvoEngine_EigenDense.hpp"
#include "GeometryStorage.hpp"
#include "GpuSpatialGrid.hpp"
#include "GpuTreeClusterPhysics.hpp"
#include "GpuTreePhysics.hpp"
#include "GpuTreeSubtreePhysics.hpp"
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
class ScopedTreePhysicsWrites {
 public:
  explicit ScopedTreePhysicsWrites(bool full) {
    if (const auto value = std::getenv(name_))
      previous_ = value;
    Set(full ? "1" : nullptr);
  }
  ~ScopedTreePhysicsWrites() {
    Set(previous_ ? previous_->c_str() : nullptr);
  }

 private:
  static constexpr const char* name_ = "EVOENGINE_TREE_PHYSICS_FULL_WRITES";
  std::optional<std::string> previous_;
  static void Set(const char* value) {
#ifdef _WIN32
    _putenv_s(name_, value ? value : "");
#else
    if (value)
      setenv(name_, value, 1);
    else
      unsetenv(name_);
#endif
  }
};

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

BaseSkeleton MakeGpuPhysicsCanopy(int levels = 6) {
  BaseSkeleton skeleton;
  auto& root = skeleton.RefNode(0).info;
  root.length = 0.7f;
  root.thickness = 0.1f;
  root.global_rotation = glm::rotation(glm::vec3(0, 0, -1), glm::vec3(0, 1, 0));
  std::vector<int> level{0};
  for (int depth = 0; depth < levels; ++depth) {
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
  physics.DownloadVertices(vertices);
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
  physics.DownloadVertices(vertices);
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
    physics.DownloadRods(rods, id);
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
  physics.DownloadVertices(vertices, 0);
  std::vector<DynamicTreeSkeleton::GpuRestState::Vertex> second_vertices;
  physics.DownloadVertices(second_vertices, 1);
  EXPECT_LT(vertices[1].position_inverse_mass.y, -0.001f);
  EXPECT_FLOAT_EQ(second_vertices[1].position_inverse_mass.y, 0.0f);
  const float first_y = vertices[1].position_inverse_mass.y;
  for (int i = 0; i < 16; ++i) {
    evo_engine::PlatformLifecycleTestAccess::PreUpdate();
    physics.Step(std::vector<GpuTreePhysics::TreeStep>{{settings, false}, {settings, true}});
    evo_engine::PlatformLifecycleTestAccess::LateUpdate();
  }
  Platform::WaitForFrameSubmissions("GPU forest physics second tree");
  physics.DownloadVertices(vertices, 0);
  physics.DownloadVertices(second_vertices, 1);
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
  physics.DownloadVertices(vertices, 0);
  std::vector<DynamicTreeSkeleton::GpuRestState::Vertex> stationary;
  physics.DownloadVertices(stationary, 1);
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
  physics.DownloadVertices(vertices);
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
  gpu.DownloadVertices(vertices);
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
    physics->DownloadVertices(local, id);
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
    physics->DownloadVertices(local, id);
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

TEST(GpuTreePhysics, AttributeArraysAndCachedCoefficientsMatchReferenceAndPreserveImmutableData) {
  ScopedTreeGpuPlatform platform;
  using Rest = DynamicTreeSkeleton::GpuRestState;
  std::vector<Rest> states;
  for (const auto& skeleton : {MakeGpuPhysicsFork(), MakeGpuPhysicsChain(), MakeGpuPhysicsCanopy(7)}) {
    Tree tree;
    tree.shoot_model.Initialize(skeleton);
    DynamicTreeSkeleton component;
    ASSERT_TRUE(component.InitializePhysics(tree));
    auto rest = component.ExportGpuRestState();
    for (auto& vertex : rest.vertices)
      vertex.velocity_radius = glm::vec4(0.1f, -0.2f, 0.3f, vertex.velocity_radius.w);
    states.push_back(std::move(rest));
  }
  GpuTreePhysics motion, full;
  {
    const ScopedTreePhysicsWrites mode(false);
    ASSERT_TRUE(motion.Initialize(states));
  }
  {
    const ScopedTreePhysicsWrites mode(true);
    ASSERT_TRUE(full.Initialize(states));
  }
  GpuSpatialGrid air;
  ASSERT_TRUE(air.RegisterBox({-3, -3, -3}, {4, 9, 4}));
  ASSERT_LE(air.GetCellCount(), GpuSpatialGrid::chunk_capacity);
  std::vector<glm::vec4> values(2 * GpuSpatialGrid::chunk_capacity);
  for (size_t i = 0; i < values.size(); i += 2) {
    values[i] = {4, 1, 3, 293.15f};
    values[i + 1] = {0.007f, 1, 1.225f, 0};
  }
  air.GetValueChunk(0)->UploadVector(values);
  auto wind_set = std::make_shared<DescriptorSet>(motion.GetWindDescriptorSetLayout());
  wind_set->UpdateBufferDescriptorBinding(0, air.GetHeads());
  wind_set->UpdateBufferDescriptorBinding(1, air.GetAddresses());
  std::vector<GpuTreePhysics::TreeStep> steps(3);
  steps[0].settings.substeps = 1;
  steps[0].settings.iterations = 3;
  steps[1].settings.substeps = 4;
  steps[1].settings.iterations = 8;
  steps[2].settings.substeps = 2;
  steps[2].settings.iterations = 5;
  for (int frame = 0; frame < 12; ++frame) {
    SCOPED_TRACE(frame);
    steps[0].model.SetPosition({0.02f * frame, 0, 0});
    if (frame == 5) {
      steps[0].settings.time_step = 0.008f;
      steps[1].settings.stretch_stiffness = 4000;
      steps[1].settings.bend_stiffness = 250;
      steps[2].settings.substeps = 3;
    }
    if (frame == 9) {
      steps[0].settings.stretch_stiffness = 0;
      steps[0].settings.bend_stiffness = 0;
    }
    steps[1].running = frame % 3 != 0;
    steps[2].running = false;
    steps[2].model.SetRotation(glm::angleAxis(0.02f * frame, glm::vec3(0, 0, 1)));
    const GpuTreePhysics::WindInput wind =
        frame < 3 ? GpuTreePhysics::WindInput{}
                  : GpuTreePhysics::WindInput{wind_set, air.GetCellCount(), GpuSpatialGrid::bucket_count, 1};
    evo_engine::PlatformLifecycleTestAccess::PreUpdate();
    motion.Step(steps, wind);
    full.Step(steps, wind);
    evo_engine::PlatformLifecycleTestAccess::LateUpdate();
    Platform::WaitForFrameSubmissions("motion-only/full-record equivalence");
    for (uint32_t id = 0; id < states.size(); ++id) {
      SCOPED_TRACE(id);
      const auto& a = motion.GetTreeState(id);
      const auto& b = full.GetTreeState(id);
      ASSERT_EQ(a->vertex_index, b->vertex_index);
      ASSERT_EQ(a->rod_index, b->rod_index);
      for (uint32_t copy = 0; copy < 2; ++copy) {
        std::vector<Rest::Vertex> av, bv;
        motion.DownloadVertices(av, id, copy);
        full.DownloadVertices(bv, id, copy);
        for (size_t i = 0; i < av.size(); ++i) {
          EXPECT_EQ(av[i].rest_local, states[id].vertices[i].rest_local);
          EXPECT_EQ(av[i].incident_rods, states[id].vertices[i].incident_rods);
          EXPECT_EQ(av[i].position_inverse_mass.w, states[id].vertices[i].position_inverse_mass.w);
          EXPECT_EQ(av[i].velocity_radius.w, states[id].vertices[i].velocity_radius.w);
          EXPECT_LT(glm::length(av[i].position_inverse_mass - bv[i].position_inverse_mass), 1e-5f);
          EXPECT_LT(glm::length(av[i].velocity_radius - bv[i].velocity_radius), 1e-4f);
        }
        std::vector<Rest::Rod> ar, br;
        motion.DownloadRods(ar, id, copy);
        full.DownloadRods(br, id, copy);
        for (size_t i = 0; i < ar.size(); ++i) {
          EXPECT_EQ(ar[i].links, states[id].rods[i].links);
          EXPECT_EQ(ar[i].length_scale_mass_radius, states[id].rods[i].length_scale_mass_radius);
          EXPECT_EQ(ar[i].rest_rotation, states[id].rods[i].rest_rotation);
          EXPECT_EQ(ar[i].incident_angles, states[id].rods[i].incident_angles);
          EXPECT_LT(glm::length(ar[i].rotation - br[i].rotation), 1e-5f);
        }
      }
    }
  }
}

TEST(GpuTreePhysics, DISABLED_ForestScaleTiming) {
  ScopedTreeGpuPlatform platform(true);
  Tree tree;
  tree.shoot_model.Initialize(MakeGpuPhysicsCanopy(13));
  DynamicTreeSkeleton component;
  ASSERT_TRUE(component.InitializePhysics(tree));
  const auto rest = component.ExportGpuRestState();
  for (const int count : {1, 4, 16, 32}) {
    GpuTreePhysics physics;
    ASSERT_TRUE(physics.Initialize(std::vector<DynamicTreeSkeleton::GpuRestState>(count, rest)));
    std::vector<GpuTreePhysics::TreeStep> steps(count, {component.physics_settings, true});
    for (int frame = 0; frame < 16; ++frame) {
      evo_engine::PlatformLifecycleTestAccess::PreUpdate();
      physics.Step(steps);
      evo_engine::PlatformLifecycleTestAccess::LateUpdate();
    }
    Platform::WaitForFrameSubmissions("GPU forest physics warmup");
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
    for (const auto& stat : stats)
      if (stat.name.rfind("EcoSysLab Tree ", 0) == 0)
        std::cout << "  " << stat.name << ": " << stat.total_milliseconds / 120 << " ms/step, " << stat.sample_count
                  << " samples\n";
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
  info.size = 22 * sizeof(float);
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
  values->DownloadVector(result, 22);
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
  EXPECT_EQ(result[10], 1);
  for (int i = 11; i <= 16; ++i)
    EXPECT_EQ(result[i], 0) << i;
  for (int i = 17; i <= 20; ++i)
    EXPECT_EQ(result[i], 1) << i;
  EXPECT_EQ(result[21], 0);
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
  physics.DownloadVertices(before, 1);
  EXPECT_EQ(physics.RegisterTree(rest, 0), 0u);
  EXPECT_EQ(physics.GetTreeState(1), neighbor);
  physics.DownloadVertices(after, 1);
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
  physics.DownloadVertices(after, 1);
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
  physics.DownloadVertices(actual, 1);
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
    const GpuTreeDeformParameters push{0,
                                       static_cast<uint32_t>(nodes.size()),
                                       0,
                                       physics.GetTreeState(id)->attribute_arrays ? 1u : 0u,
                                       glm::vec4(model.GetPosition(), 0),
                                       glm::vec4(1),
                                       {0, 0, 0, 1}};
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
    physics.DownloadVertices(vertices, id);
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

TEST(TreeTessellation, ProjectedDetailPreservesJunctionsAndBoundsCurveError) {
  ScopedTreeGpuPlatform platform;
  for (const auto& stage : {std::pair{ShaderType::Task, "Task/TreeMeshBatched.slang"},
                            std::pair{ShaderType::Mesh, "Mesh/TreeMeshIndexed.slang"}}) {
    const auto shader =
        Shader::CreateTemporary(stage.first, Platform::GetShaderGlobalDefines(),
                                std::filesystem::path("./EcoSysLabResources/Shaders/Graphics") / stage.second);
    ASSERT_TRUE(shader->Compiled()) << stage.second;
  }
  std::vector<GpuTreeNode> nodes(5);
  for (auto& node : nodes) {
    node.position_thickness = {0, -0.5f, 0.5f, 0.2f};
    node.direction_length = {0, 1, 0, 1};
    node.radial_axis_distance = {1, 0, 0, 0};
    node.subdivisions = {0.005f, 0.01f, 0.33f, 0.33f};
    node.bark = node.bark_base = glm::vec4(0);
    node.links = {UINT32_MAX, 0, 0, UINT32_MAX};
  }
  for (uint32_t i = 1; i < 3; ++i) {
    nodes[i].position_thickness = {0, 0.5f, 0.5f, i == 1 ? 0.04f : 0.02f};
    nodes[i].direction_length = glm::vec4(glm::normalize(glm::vec3(i == 1 ? 1 : -1, 1, 0)), i == 1 ? 0.4f : 0.6f);
    nodes[i].links.x = 0;
  }
  nodes[3].bark.z = 0.1f;
  nodes[4] = nodes[1];
  nodes[4].position_thickness.x = 3;
  VkBufferCreateInfo info{VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO};
  info.size = nodes.size() * sizeof(GpuTreeNode);
  info.usage = VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_TRANSFER_SRC_BIT;
  VmaAllocationCreateInfo allocation{};
  allocation.usage = VMA_MEMORY_USAGE_AUTO_PREFER_DEVICE;
  auto input = std::make_shared<Buffer>(info, allocation);
  input->UploadVector(nodes);
  info.size = 15 * sizeof(glm::vec4);
  auto output = std::make_shared<Buffer>(info, allocation);
  auto layout = std::make_shared<DescriptorSetLayout>();
  layout->PushDescriptorBinding(0, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_COMPUTE_BIT, 0);
  layout->Initialize();
  auto empty = std::make_shared<DescriptorSetLayout>();
  empty->Initialize();
  auto input_set = std::make_shared<DescriptorSet>(layout), output_set = std::make_shared<DescriptorSet>(layout);
  input_set->UpdateBufferDescriptorBinding(0, input);
  output_set->UpdateBufferDescriptorBinding(0, output);
  auto pipeline = std::make_shared<ComputePipeline>();
  pipeline->compute_shader = Shader::CreateTemporary(ShaderType::Compute, Platform::GetShaderGlobalDefines(),
                                                     std::filesystem::path(__FILE__).parent_path().parent_path() /
                                                         "Resources/Shaders/Compute/TreeTessellationProbe.slang");
  pipeline->descriptor_set_layouts = {layout, empty, empty, layout};
  pipeline->Initialize();
  ASSERT_TRUE(pipeline->Initialized());
  evo_engine::PlatformLifecycleTestAccess::PreUpdate();
  Platform::RecordCommandsMainQueue([&](VkCommandBuffer command) {
    pipeline->Bind(command);
    pipeline->BindDescriptorSet(command, 0, output_set->GetVkDescriptorSet());
    pipeline->BindDescriptorSet(command, 3, input_set->GetVkDescriptorSet());
    pipeline->Dispatch(command, 1, 1, 1);
    Platform::EverythingBarrier(command);
  });
  evo_engine::PlatformLifecycleTestAccess::LateUpdate();
  Platform::WaitForFrameSubmissions("projected branch tessellation");
  std::vector<glm::vec4> result;
  output->DownloadVector(result, 15);
  for (const auto& v : result)
    for (int i = 0; i < 4; ++i)
      EXPECT_TRUE(std::isfinite(v[i]));
  EXPECT_GE(result[0].x, result[0].y);
  EXPECT_GT(result[0].y, result[0].z);
  EXPECT_EQ(result[0].z, 4);
  EXPECT_EQ(result[0].w, result[0].x);
  EXPECT_GT(result[1].x, result[1].y);
  EXPECT_GT(result[1].y, result[1].z);
  EXPECT_EQ(result[1].z, 1);
  EXPECT_EQ(result[1].w, result[1].x);
  EXPECT_EQ(result[2], glm::vec4(result[0].x, result[1].x, result[0].x, result[1].x));
  EXPECT_EQ(result[3].x, result[3].y);
  EXPECT_EQ(result[3].x, result[3].w);
  EXPECT_GT(result[3].x, result[3].z);
  EXPECT_EQ(result[4], glm::vec4(0));
  EXPECT_FLOAT_EQ(result[5].x, result[5].y);
  EXPECT_EQ(result[5].z, 1);
  EXPECT_GE(result[5].w, 2);
  EXPECT_LT(result[5].w, result[8].z);
  EXPECT_LE(result[6].x, 0.5f);
  EXPECT_GE(result[6].y, 1);
  EXPECT_LE(result[6].y, 1024);
  EXPECT_EQ(result[6].z, 126);
  EXPECT_EQ(result[6].w, 3);
  EXPECT_GT(result[7].x, result[7].y);
  EXPECT_GE(result[7].z, result[0].y);
  EXPECT_EQ(result[7].z, result[7].w);
  EXPECT_GE(result[8].x, result[0].y);
  EXPECT_GE(result[8].y, result[1].y);
  EXPECT_GT(result[9].x, result[9].y);
  EXPECT_EQ(result[9].z, 0);
  EXPECT_EQ(result[9].w, 0);
  for (int i = 0; i < 4; ++i)
    EXPECT_LT(result[10][i], 1e-6f);
  EXPECT_GT(result[11].x, result[11].y);
  EXPECT_EQ(result[11].z, 1);
  EXPECT_LT(result[11].w, 1e-6f);
  EXPECT_EQ(result[12], glm::vec4(1, 0, 1, 1));
  EXPECT_EQ(result[13], glm::vec4(0, 1, 1, 1));
  EXPECT_EQ(result[14], glm::vec4(6, 1, 1, 0));
}

namespace {
DynamicTreeSkeleton::GpuRestState MakeLodChain(uint32_t count = 129) {
  DynamicTreeSkeleton::GpuRestState rest;
  for (uint32_t i = 0; i <= count; ++i) {
    auto offset = static_cast<uint32_t>(rest.vertex_rods.size());
    if (i)
      rest.vertex_rods.push_back(i - 1);
    if (i < count)
      rest.vertex_rods.push_back(i);
    rest.vertices.push_back({{0.02f * i, 0, 0, i == 0 ? 0.0f : 1.0f},
                             {0, 0, 0, 0.05f},
                             {0.02f * i, 0, 0, 0},
                             {offset, static_cast<uint32_t>(rest.vertex_rods.size()) - offset, 0, 0}});
  }
  for (uint32_t i = 0; i < count; ++i) {
    auto offset = static_cast<uint32_t>(rest.rod_angles.size());
    if (i)
      rest.rod_angles.push_back(i - 1);
    if (i + 1 < count)
      rest.rod_angles.push_back(i);
    rest.rods.push_back({{i, i + 1, i, i == 0 ? 1u : 0u},
                         {0.02f, 1, 1, 0.05f},
                         {0, 0, 0, 1},
                         {0, 0, 0, 1},
                         {offset, static_cast<uint32_t>(rest.rod_angles.size()) - offset, 0, 0}});
    rest.node_rods.push_back(i);
    if (i)
      rest.angles.push_back({{i - 1, i, 0, 0}, {0, 0, 0, 1}, {1, 0, 0, 0}});
  }
  return rest;
}
void StepLod(GpuTreePhysics& physics, const std::vector<GpuTreePhysics::TreeStep>& settings,
             const GpuTreePhysics::WindInput& wind = {}) {
  evo_engine::PlatformLifecycleTestAccess::PreUpdate();
  physics.Step(settings, wind);
  evo_engine::PlatformLifecycleTestAccess::LateUpdate();
  Platform::WaitForFrameSubmissions("Physics LOD test");
}
uint32_t SelectedLod(const GpuTreePhysics& physics, uint32_t tree = 0) {
  std::vector<uint32_t> state;
  physics.GetTreeState(tree)->lod->state->DownloadVector(state, 6);
  return state[0];
}
}  // namespace
TEST(GpuTreePhysicsLod, TransfersCurrentMotionAndPreservesFullTopology) {
  if (std::getenv("EVOENGINE_TREE_PHYSICS_REFERENCE"))
    GTEST_SKIP();
  ScopedTreeGpuPlatform platform;
  auto rest = MakeLodChain();
  GpuTreePhysics physics;
  ASSERT_TRUE(physics.Initialize(std::vector<DynamicTreeSkeleton::GpuRestState>{rest, rest}));
  const auto lod = physics.GetTreeState(0)->lod;
  ASSERT_GT(lod->level_count, 2u);
  const auto scratch = lod->scratch->vertices;
  DynamicTreeSkeleton::PhysicsSettings a, b;
  a.gravity = b.gravity = {};
  a.approximation_tolerance = 0.1f;
  a.substeps = 1;
  a.iterations = 3;
  b.substeps = 2;
  b.iterations = 2;
  StepLod(physics, {{a, true}, {b, true}});
  EXPECT_GT(SelectedLod(physics), 0u);
  EXPECT_EQ(SelectedLod(physics, 1), 0u);
  std::vector<DynamicTreeSkeleton::GpuRestState::Vertex> vertices;
  physics.DownloadVertices(vertices);
  for (size_t i = 0; i < vertices.size(); ++i)
    EXPECT_LT(glm::length(glm::vec3(vertices[i].position_inverse_mass - rest.vertices[i].position_inverse_mass)),
              1e-5f);
  GlobalTransform moved;
  moved.value = glm::translate(glm::mat4(1), glm::vec3(0, 0.3f, 0));
  StepLod(physics, {{a, false, moved}, {b, true}});
  physics.DownloadVertices(vertices);
  for (size_t i = 0; i < vertices.size(); ++i)
    EXPECT_NEAR(vertices[i].position_inverse_mass.y, 0.3f, 2e-5f);
  a.approximation_tolerance = 0;
  StepLod(physics, {{a, false, moved}, {b, true}});
  EXPECT_EQ(SelectedLod(physics), 0u);
  physics.DownloadVertices(vertices);
  for (const auto& v : vertices)
    EXPECT_NEAR(v.position_inverse_mass.y, 0.3f, 2e-5f);
  a.approximation_tolerance = 0.1f;
  StepLod(physics, {{a, true, moved}, {b, true}});
  EXPECT_GT(SelectedLod(physics), 0u);
  EXPECT_EQ(lod->scratch->vertices, scratch);
  std::vector<DynamicTreeSkeleton::GpuRestState::Rod> rods;
  physics.DownloadRods(rods);
  for (size_t i = 0; i < rods.size(); ++i) {
    EXPECT_EQ(rods[i].links, rest.rods[i].links);
    EXPECT_EQ(rods[i].length_scale_mass_radius, rest.rods[i].length_scale_mass_radius);
    EXPECT_NEAR(glm::length(rods[i].rotation), 1, 1e-5f);
  }
}
TEST(GpuTreePhysicsLod, RejectsCoarseningThatCannotRepresentCurrentBend) {
  if (std::getenv("EVOENGINE_TREE_PHYSICS_REFERENCE"))
    GTEST_SKIP();
  ScopedTreeGpuPlatform platform;
  auto rest = MakeLodChain(17);
  GpuTreePhysics physics;
  ASSERT_TRUE(physics.Initialize(rest));
  auto bent = rest.vertices;
  for (uint32_t i = 2; i < 17; ++i)
    bent[i].position_inverse_mass.y = i % 2 ? 0.08f : -0.08f;
  std::vector<glm::uvec4> packed(bent.size() * 4);
  for (size_t i = 0; i < bent.size(); ++i)
    for (size_t f = 0; f < 4; ++f)
      std::memcpy(&packed[f * bent.size() + i], reinterpret_cast<const char*>(&bent[i]) + f * 16, 16);
  physics.GetVertexBuffer()->UploadVector(packed);
  DynamicTreeSkeleton::PhysicsSettings settings;
  settings.approximation_tolerance = 0.25f;
  settings.gravity = {};
  settings.stretch_stiffness = 0;
  settings.bend_stiffness = 0;
  StepLod(physics, {{settings, true}});
  EXPECT_EQ(SelectedLod(physics), 0u);
  std::vector<DynamicTreeSkeleton::GpuRestState::Vertex> result;
  physics.DownloadVertices(result);
  for (size_t i = 0; i < result.size(); ++i)
    EXPECT_LT(glm::length(result[i].position_inverse_mass - bent[i].position_inverse_mass), 1e-5f);
}
TEST(GpuTreePhysicsLod, WindSamplesOriginalMembersAndGravityAcceleratesCoarseState) {
  if (std::getenv("EVOENGINE_TREE_PHYSICS_REFERENCE"))
    GTEST_SKIP();
  ScopedTreeGpuPlatform platform;
  auto rest = MakeLodChain(33);
  GpuTreePhysics physics;
  ASSERT_TRUE(physics.Initialize(rest));
  GpuSpatialGrid air(0.05f);
  ASSERT_TRUE(air.RegisterBox({0.08f, -0.1f, -0.1f}, {0.16f, 0.1f, 0.1f}));
  struct AirValue {
    glm::vec4 velocity_temperature, humidity_strength_density;
  };
  air.GetValueChunk(0)->UploadVector(
      std::vector<AirValue>(GpuSpatialGrid::chunk_capacity, {{0, 0, 5, 293.15f}, {0, 1, 1.225f, 0}}));
  auto wind = std::make_shared<DescriptorSet>(physics.GetWindDescriptorSetLayout());
  wind->UpdateBufferDescriptorBinding(0, air.GetHeads());
  wind->UpdateBufferDescriptorBinding(1, air.GetAddresses());
  DynamicTreeSkeleton::PhysicsSettings settings;
  settings.approximation_tolerance = 0.1f;
  for (int i = 0; i < 6; ++i)
    StepLod(physics, {{settings, true}}, {wind, air.GetCellCount(), GpuSpatialGrid::bucket_count, air.GetCellSize()});
  EXPECT_GT(SelectedLod(physics), 0u);
  std::vector<DynamicTreeSkeleton::GpuRestState::Vertex> result;
  physics.DownloadVertices(result);
  float displacement = 0;
  for (const auto& v : result)
    displacement = std::max(displacement, v.position_inverse_mass.z);
  EXPECT_GT(displacement, 1e-6f);
  EXPECT_LT(result.back().position_inverse_mass.y, -1e-4f);
  EXPECT_LT(result.back().velocity_radius.y, -0.01f);
  EXPECT_EQ(result.front().position_inverse_mass, rest.vertices.front().position_inverse_mass);
}

TEST(GpuTreePhysicsLod, DISABLED_CompareCantileverMotion) {
  if (std::getenv("EVOENGINE_TREE_PHYSICS_REFERENCE"))
    GTEST_SKIP();
  ScopedTreeGpuPlatform platform;
  const auto rest = MakeLodChain(33);
  GpuTreePhysics physics;
  ASSERT_TRUE(physics.Initialize(std::vector<DynamicTreeSkeleton::GpuRestState>{rest, rest}));
  DynamicTreeSkeleton::PhysicsSettings fine, coarse;
  coarse.approximation_tolerance = 0.25f;

  for (int frame = 0; frame < 120; ++frame) {
    StepLod(physics, {{fine, true}, {coarse, true}});
    if (frame == 0 || frame == 29 || frame == 59 || frame == 119) {
      std::vector<DynamicTreeSkeleton::GpuRestState::Vertex> a, b;
      physics.DownloadVertices(a, 0);
      physics.DownloadVertices(b, 1);
      std::cout << "LOD_CANTILEVER frame=" << frame << " level=" << SelectedLod(physics, 1)
                << " fine_tip_y=" << a.back().position_inverse_mass.y
                << " coarse_tip_y=" << b.back().position_inverse_mass.y << std::endl;
      EXPECT_TRUE(std::isfinite(b.back().position_inverse_mass.y));
    }
  }
}

TEST(GpuTreePhysicsLod, DensityEditsKeepPoseStorageAndHierarchy) {
  if (std::getenv("EVOENGINE_TREE_PHYSICS_REFERENCE"))
    GTEST_SKIP();
  ScopedTreeGpuPlatform platform;
  const auto rest = MakeLodChain(33);
  GpuTreePhysics physics;
  ASSERT_TRUE(physics.Initialize(std::vector<DynamicTreeSkeleton::GpuRestState>{rest, rest}));
  const auto state = physics.GetTreeState(0);
  const auto lod = state->lod;
  const auto vertices = state->vertices;
  DynamicTreeSkeleton::PhysicsSettings a, b;
  a.approximation_tolerance = b.approximation_tolerance = 0.1f;
  StepLod(physics, {{a, true}, {b, true}});
  a.density *= 2;
  for (int i = 0; i < 16; ++i)
    StepLod(physics, {{a, true}, {b, true}});
  EXPECT_EQ(physics.GetTreeState(0), state);
  EXPECT_EQ(state->lod, lod);
  EXPECT_EQ(state->vertices, vertices);
  EXPECT_FLOAT_EQ(state->coefficient_mechanics.w, 2);
  std::vector<DynamicTreeSkeleton::GpuRestState::Vertex> heavy, light;
  physics.DownloadVertices(heavy, 0);
  physics.DownloadVertices(light, 1);
  EXPECT_LT(heavy.back().position_inverse_mass.y, light.back().position_inverse_mass.y);
  EXPECT_TRUE(std::isfinite(heavy.back().position_inverse_mass.y));
}

TEST(GpuTreePhysicsLod, DISABLED_ChainForestTiming) {
  if (std::getenv("EVOENGINE_TREE_PHYSICS_REFERENCE"))
    GTEST_SKIP();
  ScopedTreeGpuPlatform platform(true);
  const auto rest = MakeLodChain(4097);
  for (const float tolerance : {0.0f, 0.1f, 0.25f, 1.0f}) {
    GpuTreePhysics physics;
    ASSERT_TRUE(physics.Initialize(std::vector<DynamicTreeSkeleton::GpuRestState>(16, rest)));
    DynamicTreeSkeleton::PhysicsSettings settings;
    settings.gravity = {};
    settings.approximation_tolerance = tolerance;
    std::vector<GpuTreePhysics::TreeStep> steps(16, {settings, true});
    for (int frame = 0; frame < 136; ++frame) {
      if (frame == 16) {
        Platform::WaitForFrameSubmissions("LOD timing warmup");
        Platform::ResetGpuTimestampStats();
      }
      evo_engine::PlatformLifecycleTestAccess::PreUpdate();
      physics.Step(steps);
      evo_engine::PlatformLifecycleTestAccess::LateUpdate();
    }
    Platform::WaitForFrameSubmissions("LOD timing readback");
    const auto stats = Platform::GetGpuTimestampStats();
    const auto timing = std::find_if(stats.begin(), stats.end(), [](const auto& stat) {
      return stat.name == "EcoSysLab Tree Physics";
    });
    ASSERT_NE(timing, stats.end());
    uint32_t count = 0;
    for (auto id : physics.GetActiveTreeIds()) {
      const auto level = SelectedLod(physics, id);
      const auto& state = physics.GetTreeState(id);
      count += level ? state->lod->templates[level - 1]->rod_count : state->rod_count;
    }
    std::cout << "LOD_TIMING tolerance=" << tolerance << " active_rods=" << count
              << " median_ms=" << timing->MedianMilliseconds() << std::endl;
  }
}

TEST(GpuTreePhysicsLod, PoseChecksCoverMoreThanOneDispatchDimension) {
  if (std::getenv("EVOENGINE_TREE_PHYSICS_REFERENCE"))
    GTEST_SKIP();
  ScopedTreeGpuPlatform platform;
  auto rest = MakeLodChain(4097);
  GpuTreePhysics physics;
  ASSERT_TRUE(physics.Initialize(rest));
  const auto lod = physics.GetTreeState(0)->lod;
  std::vector<GpuTreePhysicsLodLevel> levels;
  lod->levels->DownloadVector(levels, lod->level_count);
  ASSERT_GT(levels.size(), 1u);
  // Reuse immutable maps to stress scheduling without allocating a giant forest.
  levels.resize(1024, levels[1]);
  lod->level_count = static_cast<uint32_t>(levels.size());
  lod->levels->UploadVector(levels);
  std::vector<uint32_t> state(8 + levels.size(), 0);
  state[2] = UINT32_MAX;
  lod->state->UploadVector(state);
  lod->error_group_count = ((static_cast<uint32_t>(rest.vertices.size()) + 63) / 64) * (lod->level_count - 1);
  ASSERT_GT(lod->error_group_count, 65535u);
  rest.vertices[2].position_inverse_mass.y = 0.2f;
  std::vector<glm::uvec4> packed(rest.vertices.size() * 4);
  for (size_t i = 0; i < rest.vertices.size(); ++i)
    for (size_t field = 0; field < 4; ++field)
      std::memcpy(&packed[field * rest.vertices.size() + i],
                  reinterpret_cast<const char*>(&rest.vertices[i]) + field * 16, 16);
  physics.GetVertexBuffer()->UploadVector(packed);
  DynamicTreeSkeleton::PhysicsSettings settings;
  settings.approximation_tolerance = 0.1f;
  settings.gravity = {};
  settings.stretch_stiffness = settings.bend_stiffness = 0;
  StepLod(physics, {{settings, true}});
  EXPECT_EQ(SelectedLod(physics), 0u);
  std::vector<uint32_t> errors;
  lod->state->DownloadVector(errors, 8 + lod->level_count);
  float last_error;
  std::memcpy(&last_error, &errors.back(), sizeof(float));
  EXPECT_GT(last_error, settings.approximation_tolerance);
}

TEST(GpuTreePhysicsLod, TransferPreservesIndependentRodFrames) {
  if (std::getenv("EVOENGINE_TREE_PHYSICS_REFERENCE"))
    GTEST_SKIP();
  ScopedTreeGpuPlatform platform;
  auto rest = MakeLodChain(33);
  GpuTreePhysics physics;
  ASSERT_TRUE(physics.Initialize(rest));
  const auto q = glm::angleAxis(glm::radians(45.0f), glm::vec3(0, 0, 1));
  for (size_t i = 1; i < rest.rods.size(); ++i)
    rest.rods[i].rotation = {q.x, q.y, q.z, q.w};
  std::vector<glm::uvec4> packed(rest.rods.size() * 5);
  for (size_t i = 0; i < rest.rods.size(); ++i)
    for (size_t field = 0; field < 5; ++field)
      std::memcpy(&packed[field * rest.rods.size() + i], reinterpret_cast<const char*>(&rest.rods[i]) + field * 16, 16);
  physics.GetRodBuffer()->UploadVector(packed);
  DynamicTreeSkeleton::PhysicsSettings settings;
  settings.approximation_tolerance = 0.25f;
  settings.gravity = {};
  settings.stretch_stiffness = settings.bend_stiffness = 0;
  for (int step = 0; step < 33; ++step)
    StepLod(physics, {{settings, true}});
  EXPECT_GT(SelectedLod(physics), 0u);
  std::vector<DynamicTreeSkeleton::GpuRestState::Rod> rods;
  physics.DownloadRods(rods);
  for (size_t i = 1; i < rods.size(); ++i)
    EXPECT_GT(std::abs(glm::dot(rods[i].rotation, rest.rods[i].rotation)), 0.99999f);
}

TEST(GpuTreePhysicsLod, DeferredGrowthHierarchyUsesFullState) {
  if (std::getenv("EVOENGINE_TREE_PHYSICS_REFERENCE"))
    GTEST_SKIP();
  ScopedTreeGpuPlatform platform;
  auto rest = MakeLodChain();
  GpuTreePhysics physics;
  ASSERT_TRUE(physics.Initialize(std::vector<DynamicTreeSkeleton::GpuRestState>{rest, rest}));
  DynamicTreeSkeleton::PhysicsSettings settings;
  settings.gravity = {};
  settings.approximation_tolerance = 0.1f;
  StepLod(physics, {{settings, true}, {settings, true}});
  physics.RegisterTree(rest, 0, false);
  ASSERT_FALSE(physics.GetTreeState(0)->lod);
  GlobalTransform moved;
  moved.value = glm::translate(glm::mat4(1), glm::vec3(0, 0.3f, 0));
  StepLod(physics, {{settings, false, moved}, {settings, true}});
  std::vector<DynamicTreeSkeleton::GpuRestState::Vertex> vertices;
  physics.DownloadVertices(vertices);
  for (const auto& vertex : vertices)
    EXPECT_NEAR(vertex.position_inverse_mass.y, 0.3f, 2e-5f);
  EXPECT_GT(SelectedLod(physics, 1), 0u);
  physics.InitializeLod(0, rest);
  StepLod(physics, {{settings, false, moved}, {settings, true}});
  physics.DownloadVertices(vertices);
  for (const auto& vertex : vertices)
    EXPECT_NEAR(vertex.position_inverse_mass.y, 0.3f, 2e-5f);
}

namespace {
DynamicTreeSkeleton::GpuRestState MakeSubtreeFork() {
  DynamicTreeSkeleton::GpuRestState rest;
  const std::vector<glm::vec3> points{{0, 0, 0}, {1, 0, 0}, {2, 0, 0}, {3, 0.8f, 0.3f}, {2.7f, -0.5f, -0.4f},
                                      {1, 1, 0}};
  const std::vector<glm::uvec2> links{{0, 1}, {1, 2}, {2, 3}, {2, 4}, {1, 5}};
  std::vector<float> mass(points.size(), 0);
  for (uint32_t i = 0; i < links.size(); ++i) {
    const auto link = links[i];
    const glm::vec3 delta = points[link.y] - points[link.x];
    const auto q = glm::rotation(glm::vec3(1, 0, 0), glm::normalize(delta));
    rest.rods.push_back({{link.x, link.y, i, i == 0 ? 1u : 0u},
                         {glm::length(delta), 1, 1, 0.05f},
                         {q.x, q.y, q.z, q.w},
                         {q.x, q.y, q.z, q.w},
                         {}});
    rest.node_rods.push_back(i);
    mass[link.x] += 0.5f;
    mass[link.y] += 0.5f;
  }
  for (uint32_t i = 0; i < points.size(); ++i) {
    const auto start = static_cast<uint32_t>(rest.vertex_rods.size());
    for (uint32_t r = 0; r < links.size(); ++r)
      if (links[r].x == i || links[r].y == i)
        rest.vertex_rods.push_back(r);
    rest.vertices.push_back({{points[i], i < 2 ? 0 : 1 / mass[i]},
                             {0, 0, 0, 0.05f},
                             {points[i], 0},
                             {start, static_cast<uint32_t>(rest.vertex_rods.size()) - start, 0, 0}});
  }
  for (uint32_t i = 1; i < links.size(); ++i)
    for (uint32_t j = 0; j < i; ++j)
      if (links[j].y == links[i].x) {
        const auto a = rest.rods[j].rotation, b = rest.rods[i].rotation;
        const auto q = glm::inverse(glm::quat(a.w, a.x, a.y, a.z)) * glm::quat(b.w, b.x, b.y, b.z);
        rest.angles.push_back({{j, i, 0, 0}, {q.x, q.y, q.z, q.w}, {1, 0, 0, 0}});
      }
  for (uint32_t r = 0; r < links.size(); ++r) {
    const auto start = static_cast<uint32_t>(rest.rod_angles.size());
    for (uint32_t a = 0; a < rest.angles.size(); ++a)
      if (rest.angles[a].rods.x == r || rest.angles[a].rods.y == r)
        rest.rod_angles.push_back(a);
    rest.rods[r].incident_angles = {start, static_cast<uint32_t>(rest.rod_angles.size()) - start, 0, 0};
  }
  return rest;
}
GpuTreeSubtreePhysics::Motion SubtreeMotion(const GpuTreeSubtreePhysics& physics) {
  std::vector<GpuTreeSubtreePhysics::Motion> motion;
  physics.GetMotionBuffer()->DownloadVector(motion, 1);
  return motion.front();
}
void StepSubtree(GpuTreeSubtreePhysics& physics, const GpuTreeSubtreePhysics::Settings& settings,
                 const glm::vec3& anchor = {1, 0, 0}, const glm::quat& target = {1, 0, 0, 0}) {
  evo_engine::PlatformLifecycleTestAccess::PreUpdate();
  physics.Step(settings, anchor, target);
  evo_engine::PlatformLifecycleTestAccess::LateUpdate();
  Platform::WaitForFrameSubmissions("Subtree physics test");
}
}  // namespace

TEST(TreeSubtreePhysics, AggregatesCylinderMassAndFullInertia) {
  auto rest = MakeSubtreeFork();
  ASSERT_FALSE(BuildTreeSubtreeBody(rest, 0));
  ASSERT_FALSE(BuildTreeSubtreeBody(rest, 99));
  const auto body = BuildTreeSubtreeBody(rest, 1);
  ASSERT_TRUE(body);
  EXPECT_EQ(body->rods.size(), 3u);
  EXPECT_EQ(body->vertices.size(), 4u);
  EXPECT_FLOAT_EQ(body->mass, 3);
  const glm::vec3 expected_center =
      (glm::vec3(0.5f, 0, 0) + glm::vec3(1.5f, 0.4f, 0.15f) + glm::vec3(1.35f, -0.25f, -0.2f)) / 3.0f;
  EXPECT_LT(glm::length(body->center_of_mass - expected_center), 1e-6f);
  EXPECT_GT(glm::determinant(body->inertia_center), 0);
  EXPECT_GT(std::abs(body->inertia_center[0][1]), 0.01f);
  const auto shift = body->mass * (glm::dot(expected_center, expected_center) * glm::mat3(1) -
                                   glm::outerProduct(expected_center, expected_center));
  for (int i = 0; i < 3; ++i)
    EXPECT_LT(glm::length(body->inertia_center[i] + shift[i] - body->inertia_pivot[i]), 1e-6f);
  auto single = MakeLodChain(2);
  single.vertices[2].position_inverse_mass.x = 2.02f;
  single.rods[1].length_scale_mass_radius = {2, 1, 6, 0.1f};
  const auto cylinder = BuildTreeSubtreeBody(single, 1);
  ASSERT_TRUE(cylinder);
  EXPECT_NEAR(cylinder->inertia_center[0][0], 0.03f, 1e-6f);
  EXPECT_NEAR(cylinder->inertia_center[1][1], 2.015f, 1e-5f);
  EXPECT_NEAR(cylinder->inertia_pivot[1][1], 8.015f, 1e-5f);
}

TEST(TreeSubtreePhysics, GravityAndMovingPivotProduceAnalyticalAngularAcceleration) {
  ScopedTreeGpuPlatform platform;
  const auto rest = MakeSubtreeFork();
  const auto body = BuildTreeSubtreeBody(rest, 1).value();
  for (bool shaking : {false, true}) {
    GpuTreeSubtreePhysics physics;
    ASSERT_TRUE(physics.Initialize(rest, 1));
    GpuTreeSubtreePhysics::Settings settings;
    settings.time_step = 0.0001f;
    settings.substeps = 1;
    settings.angular_stiffness = {};
    settings.air_density = 0;
    glm::vec3 anchor = body.anchor, acceleration{};
    if (shaking) {
      settings.gravity = {};
      anchor.y += 1e-7f;
      acceleration = (anchor - body.anchor) / (settings.time_step * settings.time_step);
    }
    const auto expected = settings.time_step * glm::inverse(body.inertia_pivot) *
                          glm::cross(body.center_of_mass, body.mass * (settings.gravity - acceleration));
    StepSubtree(physics, settings, anchor);
    const auto motion = SubtreeMotion(physics);
    EXPECT_LT(glm::length(glm::vec3(motion.angular_velocity) - expected), 1e-6f);
    EXPECT_LT(motion.angular_velocity.z, 0);
  }
}

TEST(TreeSubtreePhysics, MemberWindProducesForceAndTorque) {
  ScopedTreeGpuPlatform platform;
  const auto rest = MakeSubtreeFork();
  const auto body = BuildTreeSubtreeBody(rest, 1).value();
  GpuTreeSubtreePhysics physics;
  ASSERT_TRUE(physics.Initialize(rest, 1));
  GpuTreeSubtreePhysics::Settings settings;
  settings.gravity = {};
  settings.angular_stiffness = {};
  settings.time_step = 0.0001f;
  settings.substeps = 1;
  settings.wind_velocity = {0, 0, 2};
  glm::vec3 force{}, torque{};
  for (const auto& rod : body.rods) {
    const auto axis = glm::normalize(glm::vec3(rod.end_length - rod.start_radius));
    const auto crosswind = settings.wind_velocity - glm::dot(settings.wind_velocity, axis) * axis;
    const auto endpoint_force =
        0.6f * settings.wind_drag * rod.start_radius.w * rod.end_length.w * glm::length(crosswind) * crosswind;
    force += 2.0f * endpoint_force;
    torque += glm::cross(glm::vec3(rod.start_radius + rod.end_length), endpoint_force);
  }
  StepSubtree(physics, settings);
  const auto motion = SubtreeMotion(physics);
  EXPECT_LT(glm::length(glm::vec3(motion.external_force) - force), 1e-5f);
  EXPECT_LT(glm::length(glm::vec3(motion.external_torque) - torque), 1e-5f);
  const auto expected = settings.time_step * glm::inverse(body.inertia_pivot) * torque;
  EXPECT_LT(glm::length(glm::vec3(motion.angular_velocity) - expected), 1e-6f);
  EXPECT_LT(motion.angular_velocity.y, 0);
}

TEST(TreeSubtreePhysics, PreservesGeometryAttachmentAndPersistentBuffers) {
  ScopedTreeGpuPlatform platform;
  const auto rest = MakeSubtreeFork();
  const auto body = BuildTreeSubtreeBody(rest, 1).value();
  GpuTreeSubtreePhysics physics;
  ASSERT_TRUE(physics.Initialize(rest, 1));
  const auto vertex_buffers = physics.GetTreeState()->vertices;
  const auto motion_buffer = physics.GetMotionBuffer();
  GpuTreeSubtreePhysics::Settings settings;
  settings.angular_stiffness = {0.3f, 1, 1};
  settings.wind_velocity = {0, 0, 1};
  for (int i = 0; i < 120; ++i)
    StepSubtree(physics, settings, {1, 0.05f * std::sin(i * 0.05f), 0});
  const auto motion = SubtreeMotion(physics);
  EXPECT_NEAR(glm::length(motion.rotation), 1, 1e-5f);
  EXPECT_EQ(vertex_buffers, physics.GetTreeState()->vertices);
  EXPECT_EQ(motion_buffer, physics.GetMotionBuffer());
  std::vector<glm::vec4> a, b, rods;
  physics.GetTreeState()->vertices[0]->DownloadVector(a, rest.vertices.size() * 4);
  physics.GetTreeState()->vertices[1]->DownloadVector(b, rest.vertices.size() * 4);
  physics.GetTreeState()->rods[0]->DownloadVector(rods, rest.rods.size() * 5);
  EXPECT_EQ(a, b);
  EXPECT_LT(glm::length(glm::vec3(a[1] - motion.anchor)), 1e-7f);
  EXPECT_EQ(a[0], rest.vertices[0].position_inverse_mass);
  EXPECT_EQ(a[5], rest.vertices[5].position_inverse_mass);
  const auto q = glm::quat(motion.rotation.w, motion.rotation.x, motion.rotation.y, motion.rotation.z);
  for (const auto& member : body.vertices) {
    const auto i = member.indices.x;
    const auto offset = q * glm::vec3(member.offset);
    EXPECT_LT(glm::length(glm::vec3(a[i]) - glm::vec3(motion.anchor) - offset), 2e-6f);
    EXPECT_LT(glm::length(glm::vec3(a[rest.vertices.size() + i]) - glm::vec3(motion.anchor_velocity) -
                          glm::cross(glm::vec3(motion.angular_velocity), offset)),
              2e-5f);
    EXPECT_EQ(a[i].w, rest.vertices[i].position_inverse_mass.w);
  }
  for (const auto& member : body.rods) {
    const auto rotation = rods[2 * rest.rods.size() + member.indices.x];
    const auto r = member.relative_rotation;
    const auto expected = q * glm::quat(r.w, r.x, r.y, r.z);
    EXPECT_NEAR(std::abs(glm::dot(expected, glm::quat(rotation.w, rotation.x, rotation.y, rotation.z))), 1, 1e-5f);
  }
}

TEST(TreeSubtreePhysics, CompliantAttachmentRestoresBendingAndTwist) {
  ScopedTreeGpuPlatform platform;
  for (const glm::vec3 axis : {glm::vec3(1, 0, 0), glm::vec3(0, 0, 1)}) {
    GpuTreeSubtreePhysics physics;
    ASSERT_TRUE(physics.Initialize(MakeSubtreeFork(), 1));
    GpuTreeSubtreePhysics::Settings settings;
    settings.gravity = {};
    settings.air_density = 0;
    settings.damping = 5;
    settings.angular_stiffness = glm::vec3(30);
    const auto target = glm::angleAxis(0.2f, axis);
    for (int i = 0; i < 300; ++i)
      StepSubtree(physics, settings, {1, 0, 0}, target);
    auto motion = SubtreeMotion(physics);
    EXPECT_GT(std::abs(glm::dot(target,
                                glm::quat(motion.rotation.w, motion.rotation.x, motion.rotation.y, motion.rotation.z))),
              0.9999f);
    settings.angular_stiffness = glm::vec3(100000);
    for (int i = 0; i < 30; ++i)
      StepSubtree(physics, settings, {1, 0, 0});
    motion = SubtreeMotion(physics);
    EXPECT_NEAR(std::abs(motion.rotation.w), 1, 1e-5f);
    EXPECT_LT(glm::length(glm::vec3(motion.angular_velocity)), 0.001f);
  }
}

TEST(TreeSubtreePhysics, CompareFullResolutionGravityShakingAndWind) {
  ScopedTreeGpuPlatform platform;
  const auto rest = MakeSubtreeFork();
  const auto body = BuildTreeSubtreeBody(rest, 1).value();
  for (int scenario = 0; scenario < 3; ++scenario) {
    GpuTreePhysics full;
    GpuTreeSubtreePhysics cluster;
    ASSERT_TRUE(full.Initialize(rest));
    ASSERT_TRUE(cluster.Initialize(rest, 1));
    DynamicTreeSkeleton::PhysicsSettings fine;
    GpuTreeSubtreePhysics::Settings coarse;
    // Nominal boundary stiffness scale; no claim of equal modal response.
    coarse.angular_stiffness = glm::vec3(fine.bend_stiffness);
    if (scenario != 0)
      fine.gravity = coarse.gravity = {};
    fine.wind_drag = coarse.wind_drag = 1;
    coarse.wind_velocity = scenario == 2 ? glm::vec3(0, 0, 1) : glm::vec3(0);
    GpuSpatialGrid air(1.0f);
    ASSERT_TRUE(air.RegisterBox({-4, -4, -4}, {6, 4, 4}));
    struct AirValue {
      glm::vec4 velocity_temperature, humidity_strength_density;
    };
    air.GetValueChunk(0)->UploadVector(
        std::vector<AirValue>(GpuSpatialGrid::chunk_capacity, {{coarse.wind_velocity, 293.15f}, {0, 1, 1.225f, 0}}));
    auto wind = std::make_shared<DescriptorSet>(full.GetWindDescriptorSetLayout());
    wind->UpdateBufferDescriptorBinding(0, air.GetHeads());
    wind->UpdateBufferDescriptorBinding(1, air.GetAddresses());
    for (int frame = 0; frame < 120; ++frame) {
      GlobalTransform model;
      const float displacement = scenario == 1 ? 0.05f * std::sin(frame * fine.time_step * 6.2831853f) : 0;
      model.value = glm::translate(glm::mat4(1), glm::vec3(0, displacement, 0));
      evo_engine::PlatformLifecycleTestAccess::PreUpdate();
      full.Step({{fine, true, model}}, {wind, air.GetCellCount(), GpuSpatialGrid::bucket_count, air.GetCellSize()});
      cluster.Step(coarse, body.anchor + glm::vec3(0, displacement, 0), body.rotation);
      evo_engine::PlatformLifecycleTestAccess::LateUpdate();
      Platform::WaitForFrameSubmissions("Subtree comparison");
      if (frame == 29 || frame == 119) {
        std::vector<DynamicTreeSkeleton::GpuRestState::Vertex> vertices;
        full.DownloadVertices(vertices);
        glm::vec3 fine_center{};
        for (const auto& member : body.rods) {
          const auto& rod = rest.rods[member.indices.x];
          fine_center +=
              rod.length_scale_mass_radius.z * 0.5f *
              glm::vec3(vertices[rod.links.x].position_inverse_mass + vertices[rod.links.y].position_inverse_mass);
        }
        fine_center /= body.mass;
        const auto motion = SubtreeMotion(cluster);
        const auto q = glm::quat(motion.rotation.w, motion.rotation.x, motion.rotation.y, motion.rotation.z);
        const auto coarse_center = glm::vec3(motion.anchor) + q * body.center_of_mass;
        EXPECT_TRUE(std::isfinite(glm::length(fine_center)));
        EXPECT_TRUE(std::isfinite(glm::length(coarse_center)));
        if (scenario == 0) {
          EXPECT_LT(fine_center.y, body.center_of_mass.y);
          EXPECT_LT(coarse_center.y, body.center_of_mass.y);
        } else if (scenario == 2) {
          EXPECT_GT(fine_center.z, body.center_of_mass.z);
          EXPECT_GT(coarse_center.z, body.center_of_mass.z);
        }
        std::cout << "SUBTREE_COMPARE scenario=" << scenario << " frame=" << frame << " full_com=" << fine_center.x
                  << ',' << fine_center.y << ',' << fine_center.z << " cluster_com=" << coarse_center.x << ','
                  << coarse_center.y << ',' << coarse_center.z << std::endl;
      }
    }
  }
}

TEST(TreeSubtreePhysics, RotationCovarianceAndTorqueFreeMomentum) {
  ScopedTreeGpuPlatform platform;
  auto rest = MakeSubtreeFork();
  const auto body = BuildTreeSubtreeBody(rest, 1).value();
  const auto frame = glm::angleAxis(0.7f, glm::normalize(glm::vec3(1, 2, 3)));
  const glm::vec3 translation(2, 4, -1);
  GpuTreeSubtreePhysics first, rotated;
  ASSERT_TRUE(first.Initialize(rest, 1));
  for (auto& vertex : rest.vertices)
    vertex.position_inverse_mass = {translation + frame * glm::vec3(vertex.position_inverse_mass),
                                    vertex.position_inverse_mass.w};
  for (auto& rod : rest.rods) {
    const auto q = frame * glm::quat(rod.rotation.w, rod.rotation.x, rod.rotation.y, rod.rotation.z);
    rod.rotation = rod.rest_rotation = {q.x, q.y, q.z, q.w};
  }
  ASSERT_TRUE(rotated.Initialize(rest, 1));
  GpuTreeSubtreePhysics::Settings settings;
  settings.angular_stiffness = {};
  settings.air_density = 0;
  settings.time_step = 0.001f;
  StepSubtree(first, settings);
  settings.gravity = frame * settings.gravity;
  StepSubtree(rotated, settings, translation + frame * body.anchor, frame);
  const auto a = SubtreeMotion(first), b = SubtreeMotion(rotated);
  EXPECT_LT(glm::length(glm::vec3(b.angular_velocity) - frame * glm::vec3(a.angular_velocity)), 2e-5f);
  const auto expected = frame * glm::quat(a.rotation.w, a.rotation.x, a.rotation.y, a.rotation.z);
  EXPECT_NEAR(std::abs(glm::dot(expected, glm::quat(b.rotation.w, b.rotation.x, b.rotation.y, b.rotation.z))), 1,
              1e-6f);

  GpuTreeSubtreePhysics free;
  ASSERT_TRUE(free.Initialize(MakeSubtreeFork(), 1));
  auto motion = SubtreeMotion(free);
  motion.angular_velocity = {1, 2, 3, 0};
  free.GetMotionBuffer()->UploadVector(std::vector<GpuTreeSubtreePhysics::Motion>{motion});
  const auto initial_momentum = body.inertia_pivot * glm::vec3(motion.angular_velocity);
  const float initial_energy = 0.5f * glm::dot(glm::vec3(motion.angular_velocity), initial_momentum);
  settings.gravity = {};
  for (int i = 0; i < 100; ++i)
    StepSubtree(free, settings);
  motion = SubtreeMotion(free);
  const auto q = glm::quat(motion.rotation.w, motion.rotation.x, motion.rotation.y, motion.rotation.z);
  const auto omega = glm::inverse(q) * glm::vec3(motion.angular_velocity);
  const auto momentum = q * (body.inertia_pivot * omega);
  const auto energy = 0.5f * glm::dot(omega, body.inertia_pivot * omega);
  EXPECT_LT(glm::length(momentum - initial_momentum) / glm::length(initial_momentum), 0.005f);
  EXPECT_LT(std::abs(energy - initial_energy) / initial_energy, 0.005f);
}

namespace {
DynamicTreeSkeleton::GpuRestState MakeSubtreePair() {
  auto rest = MakeLodChain(3);
  for (uint32_t i = 0; i < rest.vertices.size(); ++i) {
    rest.vertices[i].position_inverse_mass.x = static_cast<float>(i);
    rest.vertices[i].rest_local.x = static_cast<float>(i);
  }
  for (auto& rod : rest.rods)
    rod.length_scale_mass_radius.x = 1;
  return rest;
}
std::vector<GpuTreeSubtreePhysics::Motion> PairMotion(const GpuTreeSubtreePhysics& physics) {
  std::vector<GpuTreeSubtreePhysics::Motion> result;
  physics.GetMotionBuffer()->DownloadVector(result, 2);
  return result;
}
glm::quat MotionRotation(const GpuTreeSubtreePhysics::Motion& motion) {
  return {motion.rotation.w, motion.rotation.x, motion.rotation.y, motion.rotation.z};
}
std::pair<glm::vec3, float> PairMomentumEnergy(const std::array<TreeSubtreeBody, 2>& bodies,
                                               const std::vector<GpuTreeSubtreePhysics::Motion>& motions) {
  glm::vec3 momentum{};
  float energy = 0;
  for (int i = 0; i < 2; ++i) {
    const auto q = MotionRotation(motions[i]);
    const auto omega = glm::vec3(motions[i].angular_velocity);
    const auto arm = q * bodies[i].center_of_mass;
    const auto position = glm::vec3(motions[i].anchor - motions[0].anchor) + arm;
    const auto velocity = glm::vec3(motions[i].anchor_velocity) + glm::cross(omega, arm);
    const auto spin = q * (bodies[i].inertia_center * (glm::inverse(q) * omega));
    momentum += spin + glm::cross(position, bodies[i].mass * velocity);
    energy += 0.5f * (glm::dot(omega, spin) + bodies[i].mass * glm::dot(velocity, velocity));
  }
  return {momentum, energy};
}
}  // namespace

TEST(TreeSubtreePair, PartitionsMassWithoutOverlappingRods) {
  const auto rest = MakeSubtreeFork();
  EXPECT_FALSE(BuildTreeSubtreePair(rest, 1, 1));
  EXPECT_FALSE(BuildTreeSubtreePair(rest, 1, 4));
  EXPECT_FALSE(BuildTreeSubtreePair(rest, 0, 2));
  EXPECT_FALSE(BuildTreeSubtreePair(rest, 2, 1));
  const auto pair = BuildTreeSubtreePair(rest, 1, 2);
  ASSERT_TRUE(pair);
  const auto whole = BuildTreeSubtreeBody(rest, 1).value();
  EXPECT_EQ(pair->at(0).rods.size(), 2u);
  EXPECT_EQ(pair->at(1).rods.size(), 1u);
  EXPECT_FLOAT_EQ(pair->at(0).mass + pair->at(1).mass, whole.mass);
  glm::vec3 center{};
  glm::mat3 inertia(0);
  std::vector<bool> owned(rest.rods.size(), false);
  for (const auto& body : *pair) {
    const auto q = glm::inverse(whole.rotation) * body.rotation;
    const auto c = glm::inverse(whole.rotation) * (body.anchor - whole.anchor) + q * body.center_of_mass;
    center += body.mass * c;
    inertia += glm::mat3_cast(q) * body.inertia_center * glm::transpose(glm::mat3_cast(q)) +
               body.mass * (glm::dot(c, c) * glm::mat3(1) - glm::outerProduct(c, c));
    for (const auto& rod : body.rods) {
      EXPECT_FALSE(owned[rod.indices.x]);
      owned[rod.indices.x] = true;
    }
  }
  EXPECT_LT(glm::length(center / whole.mass - whole.center_of_mass), 1e-6f);
  for (int i = 0; i < 3; ++i)
    EXPECT_LT(glm::length(inertia[i] - whole.inertia_pivot[i]), 2e-6f);
}

TEST(TreeSubtreePair, ChildMassLoadsParentWithAnalyticalCoupledAcceleration) {
  ScopedTreeGpuPlatform platform;
  for (float child_mass : {1.0f, 4.0f}) {
    auto rest = MakeSubtreePair();
    rest.rods[2].length_scale_mass_radius.z = child_mass;
    GpuTreeSubtreePhysics physics;
    ASSERT_TRUE(physics.InitializeCoupled(rest, 1, 2));
    GpuTreeSubtreePhysics::Settings settings;
    settings.time_step = 0.0001f;
    settings.substeps = 1;
    settings.air_density = 0;
    settings.angular_stiffness = settings.base_angular_stiffness = {};
    const float inertia = 1.0f / 3 + 0.05f * 0.05f / 4;
    const float a = inertia + child_mass, b = 0.5f * child_mass, d = inertia * child_mass;
    const float t0 = (0.5f + child_mass) * settings.gravity.y, t1 = 0.5f * child_mass * settings.gravity.y;
    const glm::vec2 expected = settings.time_step / (a * d - b * b) * glm::vec2(d * t0 - b * t1, a * t1 - b * t0);
    StepSubtree(physics, settings);
    const auto motion = PairMotion(physics);
    EXPECT_NEAR(motion[0].angular_velocity.z, expected.x, 2e-6f);
    EXPECT_NEAR(motion[1].angular_velocity.z, expected.y, 2e-6f);
    EXPECT_LT(motion[0].angular_velocity.z, 0);
    EXPECT_GT(motion[1].angular_velocity.z, 0);
  }
}

TEST(TreeSubtreePair, JointSpringHasReciprocalReaction) {
  ScopedTreeGpuPlatform platform;
  GpuTreeSubtreePhysics physics;
  const auto rest = MakeSubtreePair();
  ASSERT_TRUE(physics.InitializeCoupled(rest, 1, 2));
  auto motion = PairMotion(physics);
  const auto q = glm::angleAxis(0.1f, glm::vec3(0, 0, 1));
  motion[1].rotation = {q.x, q.y, q.z, q.w};
  physics.GetMotionBuffer()->UploadVector(motion);
  GpuTreeSubtreePhysics::Settings settings;
  settings.gravity = {};
  settings.air_density = 0;
  settings.base_angular_stiffness = {};
  settings.time_step = 0.0001f;
  settings.substeps = 1;
  StepSubtree(physics, settings);
  motion = PairMotion(physics);
  EXPECT_GT(motion[0].angular_velocity.z, 0);
  EXPECT_LT(motion[1].angular_velocity.z, 0);
  const auto conserved = PairMomentumEnergy(BuildTreeSubtreePair(rest, 1, 2).value(), motion);
  EXPECT_LT(glm::length(conserved.first), 1e-7f);
}

TEST(TreeSubtreePair, MaintainsJointAndGeometryDuringShakingAndWind) {
  ScopedTreeGpuPlatform platform;
  const auto rest = MakeSubtreeFork();
  const auto pair = BuildTreeSubtreePair(rest, 1, 2).value();
  GpuTreeSubtreePhysics physics;
  ASSERT_TRUE(physics.InitializeCoupled(rest, 1, 2));
  const auto buffers = physics.GetTreeState()->vertices;
  const auto state = physics.GetMotionBuffer();
  GpuTreeSubtreePhysics::Settings settings;
  settings.wind_velocity = {0, 0, 1};
  settings.base_angular_stiffness = glm::vec3(30);
  settings.angular_stiffness = {0.3f, 1, 1};
  const auto joint = glm::inverse(pair[0].rotation) * (pair[1].anchor - pair[0].anchor);
  for (int frame = 0; frame < 120; ++frame) {
    const glm::vec3 anchor(1, 0.05f * std::sin(frame * 0.05f), 0);
    StepSubtree(physics, settings, anchor);
    const auto motion = PairMotion(physics);
    const auto arm = MotionRotation(motion[0]) * joint;
    EXPECT_LT(glm::length(glm::vec3(motion[1].anchor) - anchor - arm), 2e-6f);
    EXPECT_LT(glm::length(glm::vec3(motion[1].anchor_velocity - motion[0].anchor_velocity) -
                          glm::cross(glm::vec3(motion[0].angular_velocity), arm)),
              2e-5f);
  }
  EXPECT_EQ(buffers, physics.GetTreeState()->vertices);
  EXPECT_EQ(state, physics.GetMotionBuffer());
  std::vector<glm::vec4> a, b;
  buffers[0]->DownloadVector(a, rest.vertices.size() * 4);
  buffers[1]->DownloadVector(b, rest.vertices.size() * 4);
  EXPECT_EQ(a, b);
  const auto motion = PairMotion(physics);
  for (int i = 0; i < 2; ++i)
    for (const auto& vertex : pair[i].vertices) {
      const auto expected = glm::vec3(motion[i].anchor) + MotionRotation(motion[i]) * glm::vec3(vertex.offset);
      EXPECT_LT(glm::length(glm::vec3(a[vertex.indices.x]) - expected), 2e-6f);
    }
  EXPECT_EQ(a[0], rest.vertices[0].position_inverse_mass);
  EXPECT_EQ(a[5], rest.vertices[5].position_inverse_mass);
}

TEST(TreeSubtreePair, ConservesTorqueFreeMomentumAndEnergy) {
  ScopedTreeGpuPlatform platform;
  const auto rest = MakeSubtreeFork();
  const auto pair = BuildTreeSubtreePair(rest, 1, 2).value();
  GpuTreeSubtreePhysics physics;
  ASSERT_TRUE(physics.InitializeCoupled(rest, 1, 2));
  auto motion = PairMotion(physics);
  motion[0].angular_velocity = {0.1f, 0.2f, 0.3f, 0};
  motion[1].angular_velocity = {-0.2f, 0.4f, -0.1f, 0};
  motion[1].anchor_velocity = {glm::cross(glm::vec3(motion[0].angular_velocity), pair[1].anchor - pair[0].anchor), 0};
  physics.GetMotionBuffer()->UploadVector(motion);
  const auto initial = PairMomentumEnergy(pair, motion);
  GpuTreeSubtreePhysics::Settings settings;
  settings.time_step = 0.001f;
  settings.gravity = settings.angular_stiffness = settings.base_angular_stiffness = {};
  settings.air_density = 0;
  for (int i = 0; i < 100; ++i)
    StepSubtree(physics, settings);
  const auto final = PairMomentumEnergy(pair, PairMotion(physics));
  EXPECT_LT(glm::length(final.first - initial.first) / glm::length(initial.first), 0.005f);
  EXPECT_LT(std::abs(final.second - initial.second) / initial.second, 0.005f);
  std::cout << "SUBTREE_PAIR momentum_relative_error="
            << glm::length(final.first - initial.first) / glm::length(initial.first)
            << " energy_relative_error=" << std::abs(final.second - initial.second) / initial.second << std::endl;
}

TEST(TreeSubtreePair, RotatedFramesAndStiffAttachmentsRemainConsistent) {
  ScopedTreeGpuPlatform platform;
  auto rest = MakeSubtreeFork();
  const auto pair = BuildTreeSubtreePair(rest, 1, 2).value();
  const auto frame = glm::angleAxis(0.7f, glm::normalize(glm::vec3(1, 2, 3)));
  const glm::vec3 translation(2, 4, -1);
  GpuTreeSubtreePhysics original, rotated;
  ASSERT_TRUE(original.InitializeCoupled(rest, 1, 2));
  for (auto& vertex : rest.vertices)
    vertex.position_inverse_mass = {translation + frame * glm::vec3(vertex.position_inverse_mass),
                                    vertex.position_inverse_mass.w};
  for (auto& rod : rest.rods) {
    const auto q = frame * glm::quat(rod.rotation.w, rod.rotation.x, rod.rotation.y, rod.rotation.z);
    rod.rotation = rod.rest_rotation = {q.x, q.y, q.z, q.w};
  }
  ASSERT_TRUE(rotated.InitializeCoupled(rest, 1, 2));
  GpuTreeSubtreePhysics::Settings settings;
  settings.air_density = 0;
  settings.time_step = 0.001f;
  settings.angular_stiffness = {1, 2, 3};
  settings.base_angular_stiffness = {3, 2, 1};
  for (int i = 0; i < 20; ++i) {
    StepSubtree(original, settings);
    auto transformed = settings;
    transformed.gravity = frame * settings.gravity;
    StepSubtree(rotated, transformed, translation + frame * pair[0].anchor, frame);
  }
  const auto a = PairMotion(original), b = PairMotion(rotated);
  for (int i = 0; i < 2; ++i) {
    EXPECT_LT(glm::length(glm::vec3(b[i].angular_velocity) - frame * glm::vec3(a[i].angular_velocity)), 3e-4f);
    EXPECT_LT(glm::length(glm::vec3(b[i].anchor) - translation - frame * glm::vec3(a[i].anchor)), 1e-5f);
    EXPECT_NEAR(std::abs(glm::dot(frame * MotionRotation(a[i]), MotionRotation(b[i]))), 1, 1e-5f);
  }
  settings.time_step = 0.016f;
  settings.gravity = {};
  settings.damping = 5;
  settings.angular_stiffness = settings.base_angular_stiffness = glm::vec3(100000);
  const auto target = glm::angleAxis(0.2f, glm::normalize(glm::vec3(1, 1, 1)));
  for (int i = 0; i < 100; ++i)
    StepSubtree(original, settings, pair[0].anchor, target);
  const auto result = PairMotion(original);
  EXPECT_NEAR(std::abs(glm::dot(MotionRotation(result[0]), target)), 1, 1e-5f);
  EXPECT_NEAR(std::abs(glm::dot(MotionRotation(result[1]), target * glm::inverse(pair[0].rotation) * pair[1].rotation)),
              1, 1e-5f);
  for (const auto& motion : result)
    EXPECT_LT(glm::length(glm::vec3(motion.angular_velocity)), 0.005f);
}

namespace {
void StepClusters(GpuTreeClusterPhysics& physics, const GpuTreeClusterPhysics::Settings& settings,
                  const GlobalTransform& model = {},
                  GpuTreeClusterPhysics::ExecutionMode mode = GpuTreeClusterPhysics::ExecutionMode::LevelParallel) {
  evo_engine::PlatformLifecycleTestAccess::PreUpdate();
  physics.Step(settings, model, mode);
  evo_engine::PlatformLifecycleTestAccess::LateUpdate();
  Platform::WaitForFrameSubmissions("Articulated cluster test");
}
std::vector<GpuTreeClusterPhysics::Motion> ClusterMotion(const GpuTreeClusterPhysics& physics) {
  std::vector<GpuTreeClusterPhysics::Motion> result;
  physics.GetMotionBuffer()->DownloadVector(result, physics.GetBodies().size());
  return result;
}
Eigen::Vector3d E(const glm::vec3& v) {
  return {v.x, v.y, v.z};
}
Eigen::Matrix3d CrossJacobian(const glm::vec3& arm) {
  Eigen::Matrix3d result;
  for (int i = 0; i < 3; ++i) {
    glm::vec3 axis(0);
    axis[i] = 1;
    result.col(i) = E(glm::cross(axis, arm));
  }
  return result;
}
}  // namespace

TEST(TreeClusterPhysics, MatchesTwoBodySolverAtSmallSteps) {
  ScopedTreeGpuPlatform platform;
  const auto rest = MakeSubtreePair();
  GpuTreeSubtreePhysics pair;
  GpuTreeClusterPhysics tree;
  ASSERT_TRUE(pair.InitializeCoupled(rest, 1, 2));
  ASSERT_TRUE(tree.Initialize(rest, {}));
  GpuTreeClusterPhysics::Settings settings;
  settings.time_step = 0.001f;
  settings.air_density = 0;
  settings.angular_stiffness = glm::vec3(0.5f);
  settings.base_angular_stiffness = glm::vec3(0.8f);
  for (int i = 0; i < 60; ++i) {
    StepSubtree(pair, settings);
    StepClusters(tree, settings);
  }
  const auto a = PairMotion(pair), b = ClusterMotion(tree);
  ASSERT_EQ(b.size(), 3u);
  for (int i = 0; i < 2; ++i) {
    EXPECT_LT(glm::length(glm::vec3(a[i].angular_velocity - b[i + 1].angular_velocity)), 0.002f);
    EXPECT_LT(glm::length(glm::vec3(a[i].anchor - b[i + 1].anchor)), 0.0001f);
    EXPECT_NEAR(std::abs(glm::dot(MotionRotation(a[i]), MotionRotation(b[i + 1]))), 1, 1e-5f);
  }
}

TEST(TreeClusterPhysics, BranchingAccelerationMatchesIndependentDenseMassMatrix) {
  ScopedTreeGpuPlatform platform;
  GpuTreeClusterPhysics physics;
  ASSERT_TRUE(physics.Initialize(MakeSubtreeFork(), {}));
  const auto& bodies = physics.GetBodies();
  const auto& elements = physics.GetSelection().elements;
  const int dynamic = static_cast<int>(bodies.size()) - 1;
  Eigen::MatrixXd mass = Eigen::MatrixXd::Zero(3 * dynamic, 3 * dynamic);
  Eigen::VectorXd force = Eigen::VectorXd::Zero(3 * dynamic);
  GpuTreeClusterPhysics::Settings settings;
  settings.time_step = 0.0001f;
  settings.substeps = 1;
  settings.air_density = 0;
  settings.angular_stiffness = settings.base_angular_stiffness = {};
  for (uint32_t j = 1; j < bodies.size(); ++j) {
    const auto& body = bodies[j];
    std::vector<Eigen::Matrix3d> jacobian(dynamic, Eigen::Matrix3d::Zero());
    jacobian[j - 1] = CrossJacobian(body.rotation * body.center_of_mass);
    for (uint32_t child = j, parent = elements[child].parent; parent != 0;
         child = parent, parent = elements[child].parent)
      jacobian[parent - 1] = CrossJacobian(bodies[child].anchor - bodies[parent].anchor);
    const auto world_inertia =
        glm::mat3_cast(body.rotation) * body.inertia_center * glm::transpose(glm::mat3_cast(body.rotation));
    for (int a = 0; a < 3; ++a)
      for (int b = 0; b < 3; ++b)
        mass(3 * (j - 1) + a, 3 * (j - 1) + b) += world_inertia[b][a];
    for (int a = 0; a < dynamic; ++a) {
      force.segment<3>(3 * a) += jacobian[a].transpose() * E(body.mass * settings.gravity);
      for (int b = 0; b < dynamic; ++b)
        mass.block<3, 3>(3 * a, 3 * b) += body.mass * jacobian[a].transpose() * jacobian[b];
    }
  }
  const Eigen::VectorXd acceleration = mass.ldlt().solve(force);
  EXPECT_LT((mass * acceleration - force).norm(), 1e-8);
  StepClusters(physics, settings);
  const auto motion = ClusterMotion(physics);
  for (int i = 0; i < dynamic; ++i)
    EXPECT_LT(
        (E(glm::vec3(motion[i + 1].angular_velocity)) - settings.time_step * acceleration.segment<3>(3 * i)).norm(),
        1e-5);
}

TEST(TreeClusterPhysics, PolicyLayoutsReconstructAllRodsWithMovingRoots) {
  ScopedTreeGpuPlatform platform;
  auto rest = MakeSubtreeFork();
  rest.branch_orders = {0, 1, 2, 2, 1};
  TreeClusterRule rule;
  rule.policy = TreeClusterPolicy::BranchOrder;
  rule.minimum_branch_order = 1;
  for (bool coarse : {false, true}) {
    GpuTreeClusterPhysics physics;
    ASSERT_TRUE(physics.Initialize(rest, coarse ? std::vector<TreeClusterRule>{rule} : std::vector<TreeClusterRule>{}));
    EXPECT_EQ(physics.GetBodies().size(), coarse ? 3u : 5u);
    const auto buffers = physics.GetTreeState()->vertices;
    const auto state = physics.GetMotionBuffer();
    GpuTreeClusterPhysics::Settings settings;
    settings.angular_stiffness = glm::vec3(5);
    settings.base_angular_stiffness = glm::vec3(50);
    settings.wind_velocity = {0, 0, 0.5f};
    GlobalTransform model;
    for (int i = 0; i < 80; ++i) {
      model.value = glm::translate(glm::mat4(1), glm::vec3(0, 0.03f * std::sin(i * 0.03f), 0)) *
                    glm::mat4_cast(glm::angleAxis(0.05f * std::sin(i * 0.03f), glm::vec3(0, 0, 1)));
      StepClusters(physics, settings, model);
    }
    EXPECT_EQ(buffers, physics.GetTreeState()->vertices);
    EXPECT_EQ(state, physics.GetMotionBuffer());
    const auto motion = ClusterMotion(physics);
    std::vector<glm::vec4> a, b;
    buffers[0]->DownloadVector(a, rest.vertices.size() * 4);
    buffers[1]->DownloadVector(b, rest.vertices.size() * 4);
    EXPECT_EQ(a, b);
    EXPECT_LT(glm::length(glm::vec3(a[0]) - model.GetPosition()), 1e-6f);
    EXPECT_LT(glm::length(glm::vec3(a[1]) - glm::vec3(model.value * glm::vec4(1, 0, 0, 1))), 1e-5f);
    for (uint32_t i = 0; i < motion.size(); ++i) {
      EXPECT_NEAR(glm::length(motion[i].rotation), 1, 1e-5f);
      for (const auto& vertex : physics.GetBodies()[i].vertices) {
        const auto arm = MotionRotation(motion[i]) * glm::vec3(vertex.offset);
        EXPECT_LT(glm::length(glm::vec3(a[vertex.indices.x]) - glm::vec3(motion[i].anchor) - arm), 1e-5f);
        EXPECT_LT(
            glm::length(glm::vec3(a[rest.vertices.size() + vertex.indices.x]) - glm::vec3(motion[i].anchor_velocity) -
                        glm::cross(glm::vec3(motion[i].angular_velocity), arm)),
            2e-4f);
      }
    }
  }
}

TEST(TreeClusterPhysics, BranchingMomentumAndEnergyRemainBounded) {
  ScopedTreeGpuPlatform platform;
  GpuTreeClusterPhysics physics;
  ASSERT_TRUE(physics.Initialize(MakeSubtreeFork(), {}));
  auto motion = ClusterMotion(physics);
  const auto& bodies = physics.GetBodies();
  const auto& elements = physics.GetSelection().elements;
  for (uint32_t i = 1; i < motion.size(); ++i) {
    motion[i].angular_velocity = {0.1f * i, 0.2f, -0.1f, 0};
    const auto parent = elements[i].parent;
    motion[i].anchor_velocity = {
        glm::vec3(motion[parent].anchor_velocity) +
            glm::cross(glm::vec3(motion[parent].angular_velocity), bodies[i].anchor - bodies[parent].anchor),
        0};
  }
  physics.GetMotionBuffer()->UploadVector(motion);
  const auto measure = [&](const auto& states) {
    glm::vec3 momentum{};
    float energy = 0;
    for (uint32_t i = 1; i < states.size(); ++i) {
      const auto q = MotionRotation(states[i]);
      const auto omega = glm::vec3(states[i].angular_velocity), arm = q * bodies[i].center_of_mass;
      const auto velocity = glm::vec3(states[i].anchor_velocity) + glm::cross(omega, arm);
      const auto spin = q * (bodies[i].inertia_center * (glm::inverse(q) * omega));
      momentum += spin + glm::cross(glm::vec3(states[i].anchor) + arm - glm::vec3(1, 0, 0), bodies[i].mass * velocity);
      energy += 0.5f * (glm::dot(omega, spin) + bodies[i].mass * glm::dot(velocity, velocity));
    }
    return std::make_pair(momentum, energy);
  };
  const auto initial = measure(motion);
  GpuTreeClusterPhysics::Settings settings;
  settings.gravity = settings.angular_stiffness = settings.base_angular_stiffness = {};
  settings.air_density = 0;
  settings.time_step = 0.001f;
  for (int i = 0; i < 100; ++i)
    StepClusters(physics, settings);
  const auto final = measure(ClusterMotion(physics));
  EXPECT_LT(glm::length(final.first - initial.first) / glm::length(initial.first), 0.005f);
  EXPECT_LT(std::abs(final.second - initial.second) / initial.second, 0.005f);
}

TEST(TreeClusterPhysics, DeepTreeAndStrongJointsStayFinite) {
  ScopedTreeGpuPlatform platform;
  auto rest = MakeLodChain(129);
  GpuTreeClusterPhysics physics;
  ASSERT_TRUE(physics.Initialize(rest, {}));
  GpuTreeClusterPhysics::Settings settings;
  settings.angular_stiffness = settings.base_angular_stiffness = glm::vec3(100000);
  settings.air_density = 0;
  settings.damping = 5;
  for (int i = 0; i < 10; ++i)
    StepClusters(physics, settings);
  for (const auto& motion : ClusterMotion(physics)) {
    EXPECT_TRUE(std::isfinite(glm::length(motion.angular_velocity)));
    EXPECT_NEAR(glm::length(motion.rotation), 1, 1e-5f);
  }
}

namespace {
DynamicTreeSkeleton::GpuRestState MakeClusterBinaryTree(uint32_t count) {
  auto rest = MakeLodChain(count);
  rest.vertex_rods.clear();
  rest.rod_angles.clear();
  rest.angles.clear();
  std::vector<std::vector<uint32_t>> incident_rods(count + 1), incident_angles(count);
  rest.branch_orders.resize(count);
  for (uint32_t i = 0; i < count; ++i) {
    const uint32_t start = i ? (i - 1) / 2 + 1 : 0;
    rest.rods[i].links.x = start;
    const auto point = glm::vec3(rest.vertices[start].position_inverse_mass) + glm::vec3(0.02f, 0, 0);
    rest.vertices[i + 1].position_inverse_mass = glm::vec4(point, 1);
    rest.vertices[i + 1].rest_local = glm::vec4(point, 0);
    incident_rods[start].push_back(i);
    incident_rods[i + 1].push_back(i);
    if (i) {
      const uint32_t parent = start - 1;
      rest.branch_orders[i] = rest.branch_orders[parent] + 1;
      const auto angle = static_cast<uint32_t>(rest.angles.size());
      rest.angles.push_back({{parent, i, 0, 0}, {0, 0, 0, 1}, {1, 0, 0, 0}});
      incident_angles[parent].push_back(angle);
      incident_angles[i].push_back(angle);
    }
  }
  for (uint32_t i = 0; i <= count; ++i) {
    rest.vertices[i].incident_rods = {rest.vertex_rods.size(), incident_rods[i].size(), 0, 0};
    rest.vertex_rods.insert(rest.vertex_rods.end(), incident_rods[i].begin(), incident_rods[i].end());
  }
  for (uint32_t i = 0; i < count; ++i) {
    rest.rods[i].incident_angles = {rest.rod_angles.size(), incident_angles[i].size(), 0, 0};
    rest.rod_angles.insert(rest.rod_angles.end(), incident_angles[i].begin(), incident_angles[i].end());
  }
  return rest;
}
}  // namespace
TEST(TreeClusterPhysics, ParallelMatchesSerialAcrossPolicies) {
  ScopedTreeGpuPlatform platform;
  const auto rest = MakeClusterBinaryTree(255);
  TreeClusterRule rule;
  rule.policy = TreeClusterPolicy::BranchOrder;
  rule.minimum_branch_order = 4;
  for (bool coarse : {false, true}) {
    GpuTreeClusterPhysics serial, parallel;
    const auto rules = coarse ? std::vector<TreeClusterRule>{rule} : std::vector<TreeClusterRule>{};
    ASSERT_TRUE(serial.Initialize(rest, rules));
    ASSERT_TRUE(parallel.Initialize(rest, rules));
    GpuTreeClusterPhysics::Settings settings;
    settings.time_step = 0.001f;
    settings.wind_velocity = {0, 0, 0.2f};
    settings.angular_stiffness = glm::vec3(10);
    settings.base_angular_stiffness = glm::vec3(100);
    for (int frame = 0; frame < 30; ++frame) {
      GlobalTransform model;
      model.value = glm::translate(glm::mat4(1), glm::vec3(0, 0.001f * std::sin(frame * 0.1f), 0));
      StepClusters(serial, settings, model, GpuTreeClusterPhysics::ExecutionMode::Serial);
      StepClusters(parallel, settings, model);
    }
    const auto a = ClusterMotion(serial), b = ClusterMotion(parallel);
    ASSERT_EQ(a.size(), b.size());
    for (size_t i = 0; i < a.size(); ++i) {
      EXPECT_LT(glm::length(a[i].anchor - b[i].anchor), 1e-5f);
      EXPECT_LT(glm::length(a[i].angular_velocity - b[i].angular_velocity), 1e-3f);
      EXPECT_NEAR(std::abs(glm::dot(a[i].rotation, b[i].rotation)), 1, 1e-5f);
    }
  }
}
TEST(TreeClusterPhysics, DISABLED_SchedulingTiming) {
  ScopedTreeGpuPlatform platform(true);
  for (uint32_t count : {127u, 1023u, 4095u}) {
    for (bool broad : {true, false}) {
      if (!broad && count > 127)
        continue;
      const auto rest = broad ? MakeClusterBinaryTree(count) : MakeLodChain(count);
      for (auto mode :
           {GpuTreeClusterPhysics::ExecutionMode::Serial, GpuTreeClusterPhysics::ExecutionMode::LevelParallel}) {
        GpuTreeClusterPhysics physics;
        ASSERT_TRUE(physics.Initialize(rest, {}));
        GpuTreeClusterPhysics::Settings settings;
        settings.gravity = {};
        settings.air_density = 0;
        for (int frame = 0; frame < 76; ++frame) {
          if (frame == 16) {
            Platform::WaitForFrameSubmissions("Cluster warmup");
            Platform::ResetGpuTimestampStats();
          }
          evo_engine::PlatformLifecycleTestAccess::PreUpdate();
          physics.Step(settings, {}, mode);
          evo_engine::PlatformLifecycleTestAccess::LateUpdate();
        }
        Platform::WaitForFrameSubmissions("Cluster timing");
        const auto stats = Platform::GetGpuTimestampStats();
        const auto timing = std::find_if(stats.begin(), stats.end(), [](const auto& stat) {
          return stat.name == "EcoSysLab Cluster Physics";
        });
        ASSERT_NE(timing, stats.end());
        std::cout << "CLUSTER_TIMING bodies=" << count << " broad=" << broad
                  << " parallel=" << (mode == GpuTreeClusterPhysics::ExecutionMode::LevelParallel)
                  << " median_ms=" << timing->MedianMilliseconds() << std::endl;
      }
    }
  }
}

TEST(TreeClusterForest, MatchesIndependentTreesWithDifferentSettingsAndPauses) {
  ScopedTreeGpuPlatform platform;
  std::vector<std::shared_ptr<GpuTreeClusterPhysics>> trees, references;
  std::vector<GpuTreeClusterForest::TreeStep> steps(16);
  std::vector<std::shared_ptr<Buffer>> buffers;
  for (uint32_t i = 0; i < 16; ++i) {
    const auto rest = i % 3 == 0 ? MakeSubtreeFork() : i % 3 == 1 ? MakeLodChain(17 + i) : MakeClusterBinaryTree(31);
    TreeClusterRule rule;
    rule.policy = TreeClusterPolicy::BranchOrder;
    rule.minimum_branch_order = 2;
    const auto rules = i % 2 ? std::vector<TreeClusterRule>{rule} : std::vector<TreeClusterRule>{};
    trees.push_back(std::make_shared<GpuTreeClusterPhysics>());
    references.push_back(std::make_shared<GpuTreeClusterPhysics>());
    ASSERT_TRUE(trees.back()->Initialize(rest, rules));
    ASSERT_TRUE(references.back()->Initialize(rest, rules));
    buffers.push_back(trees.back()->GetMotionBuffer());
    steps[i].settings.time_step = 0.001f;
    steps[i].settings.substeps = 1 + i % 4;
    steps[i].settings.angular_stiffness = glm::vec3(1 + i);
    steps[i].settings.base_angular_stiffness = glm::vec3(10 + i);
    steps[i].settings.wind_velocity = {0, 0, 0.01f * i};
  }
  GpuTreeClusterForest forest;
  EXPECT_FALSE(forest.Initialize({trees[0], trees[0]}));
  ASSERT_TRUE(forest.Initialize(trees));
  for (uint32_t frame = 0; frame < 20; ++frame) {
    for (uint32_t i = 0; i < 16; ++i) {
      steps[i].running = (frame + i) % 4 != 0;
      steps[i].model.value = glm::translate(glm::mat4(1), glm::vec3(0, 0.0001f * i * std::sin(frame * 0.1f), 0));
    }
    steps[0].settings.time_step = 0;
    evo_engine::PlatformLifecycleTestAccess::PreUpdate();
    for (uint32_t i = 0; i < 16; ++i)
      if (steps[i].running)
        references[i]->Step(steps[i].settings, steps[i].model);
    forest.Step(steps);
    evo_engine::PlatformLifecycleTestAccess::LateUpdate();
    Platform::WaitForFrameSubmissions("Forest cluster equivalence");
  }
  for (uint32_t i = 0; i < 16; ++i) {
    EXPECT_EQ(buffers[i], trees[i]->GetMotionBuffer());
    const auto actual = ClusterMotion(*trees[i]), expected = ClusterMotion(*references[i]);
    ASSERT_EQ(actual.size(), expected.size());
    for (size_t j = 0; j < actual.size(); ++j) {
      EXPECT_LT(glm::length(actual[j].anchor - expected[j].anchor), 1e-5f);
      EXPECT_LT(glm::length(actual[j].angular_velocity - expected[j].angular_velocity), 1e-3f);
      EXPECT_NEAR(std::abs(glm::dot(actual[j].rotation, expected[j].rotation)), 1, 1e-5f);
    }
    for (uint32_t copy = 0; copy < 2; ++copy) {
      std::vector<glm::vec4> a, b;
      const auto count = trees[i]->GetTreeState()->vertex_count * 4;
      trees[i]->GetTreeState()->vertices[copy]->DownloadVector(a, count);
      references[i]->GetTreeState()->vertices[copy]->DownloadVector(b, count);
      ASSERT_EQ(a.size(), b.size());
      for (size_t v = 0; v < a.size(); ++v)
        EXPECT_LT(glm::length(a[v] - b[v]), 1e-4f);
    }
  }
}
TEST(TreeClusterForest, DISABLED_SchedulingTiming) {
  ScopedTreeGpuPlatform platform(true);
  for (bool broad : {false, true}) {
    const auto rest = broad ? MakeClusterBinaryTree(127) : MakeLodChain(127);
    for (bool batched : {false, true}) {
      std::vector<std::shared_ptr<GpuTreeClusterPhysics>> trees;
      for (int i = 0; i < 16; ++i) {
        trees.push_back(std::make_shared<GpuTreeClusterPhysics>());
        ASSERT_TRUE(trees.back()->Initialize(rest, {}));
      }
      GpuTreeClusterForest forest;
      ASSERT_TRUE(forest.Initialize(trees));
      std::vector<GpuTreeClusterForest::TreeStep> steps(16);
      for (auto& step : steps) {
        step.settings.gravity = {};
        step.settings.air_density = 0;
      }
      for (int frame = 0; frame < 76; ++frame) {
        if (frame == 16) {
          Platform::WaitForFrameSubmissions("Forest cluster warmup");
          Platform::ResetGpuTimestampStats();
        }
        evo_engine::PlatformLifecycleTestAccess::PreUpdate();
        if (batched)
          forest.Step(steps);
        else
          for (size_t i = 0; i < trees.size(); ++i)
            trees[i]->Step(steps[i].settings);
        evo_engine::PlatformLifecycleTestAccess::LateUpdate();
      }
      Platform::WaitForFrameSubmissions("Forest cluster timing");
      const auto stats = Platform::GetGpuTimestampStats();
      const auto timing = std::find_if(stats.begin(), stats.end(), [&](const auto& stat) {
        return stat.name == (batched ? "EcoSysLab Cluster Forest" : "EcoSysLab Cluster Physics");
      });
      ASSERT_NE(timing, stats.end());
      ASSERT_GT(timing->sample_count, 0u);
      std::cout << "CLUSTER_FOREST_TIMING trees=16 bodies_each=127 broad=" << broad << " batched=" << batched
                << " mean_gpu_ms=" << timing->total_milliseconds / timing->sample_count << std::endl;
    }
  }
}

TEST(TreeClusterForest, ClimateGridMatchesUniformLoadsAndHandlesMissingCells) {
  ScopedTreeGpuPlatform platform;
  auto rest = MakeSubtreeFork();
  rest.branch_orders = {0, 1, 2, 2, 1};
  GpuSpatialGrid grid;
  ASSERT_TRUE(grid.RegisterBox({-2, -2, -2}, {5, 3, 3}));
  ASSERT_LE(grid.GetCellCount(), GpuSpatialGrid::chunk_capacity);
  std::vector<glm::vec4> values(2 * GpuSpatialGrid::chunk_capacity);
  for (size_t i = 0; i < values.size(); i += 2) {
    values[i] = {0, 0, 2, 293.15f};
    values[i + 1] = {0.01f, 0.25f, 2.45f, 0};
  }
  grid.GetValueChunk(0)->UploadVector(values);
  std::vector<std::shared_ptr<GpuTreeClusterPhysics>> trees, reference;
  for (bool coarse : {false, true}) {
    TreeClusterRule rule;
    rule.policy = TreeClusterPolicy::BranchOrder;
    rule.minimum_branch_order = 1;
    const auto rules = coarse ? std::vector<TreeClusterRule>{rule} : std::vector<TreeClusterRule>{};
    trees.push_back(std::make_shared<GpuTreeClusterPhysics>());
    reference.push_back(std::make_shared<GpuTreeClusterPhysics>());
    ASSERT_TRUE(trees.back()->Initialize(rest, rules));
    ASSERT_TRUE(reference.back()->Initialize(rest, rules));
  }
  auto set = std::make_shared<DescriptorSet>(trees[0]->GetWindDescriptorSetLayout());
  set->UpdateBufferDescriptorBinding(0, grid.GetHeads());
  set->UpdateBufferDescriptorBinding(1, grid.GetAddresses());
  GpuTreePhysics::WindInput wind{set, grid.GetCellCount(), GpuSpatialGrid::bucket_count, 1};
  GpuTreeClusterForest forest;
  ASSERT_TRUE(forest.Initialize(trees));
  std::vector<GpuTreeClusterForest::TreeStep> steps(2);
  for (auto& step : steps) {
    step.settings.time_step = 0.001f;
    step.settings.substeps = 1;
    step.settings.gravity = step.settings.angular_stiffness = step.settings.base_angular_stiffness = {};
    step.settings.wind_velocity = {0, 0, 2};
    step.settings.air_density = 0.25f * 2.45f;
  }
  for (int frame = 0; frame < 10; ++frame) {
    evo_engine::PlatformLifecycleTestAccess::PreUpdate();
    for (size_t i = 0; i < trees.size(); ++i)
      reference[i]->Step(steps[i].settings);
    forest.Step(steps, wind);
    evo_engine::PlatformLifecycleTestAccess::LateUpdate();
    Platform::WaitForFrameSubmissions("Cluster climate load equivalence");
  }
  for (size_t i = 0; i < trees.size(); ++i) {
    const auto a = ClusterMotion(*trees[i]), b = ClusterMotion(*reference[i]);
    ASSERT_EQ(a.size(), b.size());
    EXPECT_GT(glm::length(a[1].angular_velocity), 1e-4f);
    for (size_t j = 0; j < a.size(); ++j) {
      EXPECT_LT(glm::length(a[j].external_force - b[j].external_force), 1e-5f);
      EXPECT_LT(glm::length(a[j].external_torque - b[j].external_torque), 1e-5f);
      EXPECT_LT(glm::length(a[j].angular_velocity - b[j].angular_velocity), 1e-4f);
    }
  }
  GpuSpatialGrid absent;
  ASSERT_TRUE(absent.RegisterBox({20, 20, 20}, {21, 21, 21}));
  auto missing = std::make_shared<DescriptorSet>(trees[0]->GetWindDescriptorSetLayout());
  missing->UpdateBufferDescriptorBinding(0, absent.GetHeads());
  missing->UpdateBufferDescriptorBinding(1, absent.GetAddresses());
  for (bool empty : {false, true}) {
    if (!empty) {
      for (size_t i = 1; i < values.size(); i += 2)
        values[i].y = 0;
      grid.GetValueChunk(0)->UploadVector(values);
    }
    const GpuTreePhysics::WindInput input =
        empty ? GpuTreePhysics::WindInput{missing, absent.GetCellCount(), GpuSpatialGrid::bucket_count, 1} : wind;
    evo_engine::PlatformLifecycleTestAccess::PreUpdate();
    // Exercise both within-tree scheduling and the forest batch with a climate descriptor.
    trees[0]->Step(steps[0].settings, {}, GpuTreeClusterPhysics::ExecutionMode::LevelParallel, input);
    steps[0].running = false;
    forest.Step(steps, input);
    evo_engine::PlatformLifecycleTestAccess::LateUpdate();
    Platform::WaitForFrameSubmissions("Cluster empty climate");
    for (const auto& tree : trees)
      for (const auto& motion : ClusterMotion(*tree)) {
        EXPECT_LT(glm::length(motion.external_force), 1e-6f);
        EXPECT_LT(glm::length(motion.external_torque), 1e-6f);
      }
  }
}

TEST(TreeClusterForest, LiveGpuSelectionPreservesCurrentPoseAcrossAllPolicies) {
  ScopedTreeGpuPlatform platform;
  auto rest = MakeSubtreeFork();
  rest.branch_orders = {0, 1, 2, 2, 1};
  const auto hierarchy = BuildTreeClusterHierarchy(rest);
  ASSERT_TRUE(hierarchy);
  auto tree = std::make_shared<GpuTreeClusterPhysics>();
  ASSERT_TRUE(tree->InitializeDynamic(rest));
  GpuTreeClusterForest forest;
  ASSERT_TRUE(forest.Initialize({tree}));
  GpuTreeClusterForest::TreeStep step;
  step.settings.time_step = 0.001f;
  step.settings.substeps = 2;
  step.settings.wind_velocity = {0, 0, 0.5f};
  step.settings.angular_stiffness = step.settings.base_angular_stiffness = glm::vec3(2);
  const auto update = [&] {
    evo_engine::PlatformLifecycleTestAccess::PreUpdate();
    forest.Step({step});
    evo_engine::PlatformLifecycleTestAccess::LateUpdate();
    Platform::WaitForFrameSubmissions("Live cluster LOD");
  };
  for (int i = 0; i < 30; ++i)
    update();
  std::vector<glm::vec4> initial;
  tree->GetTreeState()->vertices[0]->DownloadVector(initial, rest.vertices.size() * 4);
  EXPECT_GT(glm::length(glm::vec3(initial[3]) - glm::vec3(rest.vertices[3].position_inverse_mass)), 1e-4f);
  const auto vertices = tree->GetTreeState()->vertices;
  const auto motions = tree->GetMotionBuffer();
  const auto topology = tree->GetLodSelectionBuffer();
  uint32_t transitions = 1;
  for (uint32_t mask : {2u, 0u, 1u, 3u, 4u, 5u, 6u, 7u, 0u}) {
    step.running = false;
    step.lod.enabled_policies = mask;
    step.lod.rules = {glm::vec4(10, 1, 1, 2), glm::vec4(0.15f, 0.25f, 1, 2), glm::vec4(10, 1, 2, 2)};
    std::vector<glm::vec4> before, after;
    vertices[0]->DownloadVector(before, rest.vertices.size() * 4);
    update();
    std::vector<glm::uvec4> state, selection;
    tree->GetLodStateBuffer()->DownloadVector(state, 6);
    std::vector<TreeClusterRule> rules;
    for (uint32_t p = 0; p < 3; ++p)
      if (mask & (1u << p)) {
        const auto values = step.lod.rules[p];
        rules.push_back({static_cast<TreeClusterPolicy>(p), static_cast<uint32_t>(values.z),
                         static_cast<uint32_t>(values.w), values.x, values.y});
      }
    const auto expected = SelectTreeClusters(*hierarchy, rules);
    ASSERT_EQ(state[0].x, expected.elements.size());
    EXPECT_EQ(state[0].w, ++transitions);
    topology->DownloadVector(selection, state[0].x);
    for (size_t i = 0; i < selection.size(); ++i) {
      const auto& e = expected.elements[i];
      EXPECT_EQ(selection[i], glm::uvec4(e.root_rod, e.parent, e.begin, e.end));
    }
    vertices[0]->DownloadVector(after, rest.vertices.size() * 4);
    EXPECT_EQ(before, after);
    update();
    tree->GetLodStateBuffer()->DownloadVector(state, 6);
    EXPECT_EQ(state[0].w, transitions);
    step.running = true;
    step.settings.time_step = 1e-8f;
    update();
    vertices[0]->DownloadVector(after, rest.vertices.size() * 4);
    for (size_t i = 0; i < rest.vertices.size(); ++i)
      EXPECT_LT(glm::length(glm::vec3(before[i] - after[i])), 1e-4f);
    EXPECT_EQ(vertices, tree->GetTreeState()->vertices);
    EXPECT_EQ(motions, tree->GetMotionBuffer());
    EXPECT_EQ(topology, tree->GetLodSelectionBuffer());
  }
}

TEST(TreeClusterForest, WorkgroupMatchesSerialWhileChangingGpuTopology) {
  ScopedTreeGpuPlatform platform;
  auto rest = MakeClusterBinaryTree(255);
  auto serial = std::make_shared<GpuTreeClusterPhysics>(), parallel = std::make_shared<GpuTreeClusterPhysics>();
  ASSERT_TRUE(serial->InitializeDynamic(rest));
  ASSERT_TRUE(parallel->InitializeDynamic(rest));
  GpuTreeClusterForest forest;
  ASSERT_TRUE(forest.Initialize({serial, parallel}));
  GpuTreeClusterForest::TreeStep input;
  input.settings.time_step = 0.001f;
  input.settings.substeps = 2;
  input.settings.wind_velocity = {0, 0, 0.2f};
  input.settings.angular_stiffness = input.settings.base_angular_stiffness = glm::vec3(3);
  input.mass_scale = 1.5f;
  for (int frame = 0; frame < 40; ++frame) {
    input.lod.enabled_policies = frame >= 10 && frame < 25 ? 2 : 0;
    input.lod.rules[1].z = 3;
    input.model.SetPosition({0, 0.001f * std::sin(frame * 0.1f), 0});
    auto other = input;
    input.scheduling = GpuTreeClusterForest::Scheduling::Serial;
    other.scheduling = GpuTreeClusterForest::Scheduling::Workgroup;
    evo_engine::PlatformLifecycleTestAccess::PreUpdate();
    forest.Step({input, other});
    evo_engine::PlatformLifecycleTestAccess::LateUpdate();
    Platform::WaitForFrameSubmissions("Cluster workgroup equivalence");
  }
  std::vector<glm::uvec4> state;
  serial->GetLodStateBuffer()->DownloadVector(state, 6);
  EXPECT_EQ(state[0].x, 255u);
  EXPECT_EQ(state[0].w, 3u);
  const auto a = ClusterMotion(*serial), b = ClusterMotion(*parallel);
  for (size_t i = 0; i < state[0].x; ++i) {
    EXPECT_LT(glm::length(a[i].anchor - b[i].anchor), 1e-5f);
    EXPECT_LT(glm::length(a[i].angular_velocity - b[i].angular_velocity), 1e-3f);
  }
}
TEST(TreeClusterForest, SharesExistingBentGpuStateWithoutRestoringRestPose) {
  ScopedTreeGpuPlatform platform;
  const auto rest = MakeLodChain(17);
  GpuTreePhysics original;
  ASSERT_TRUE(original.Initialize(rest));
  DynamicTreeSkeleton::PhysicsSettings settings;
  settings.time_step = 0.001f;
  settings.substeps = 1;
  settings.iterations = 3;
  for (int i = 0; i < 15; ++i)
    StepLod(original, {{settings, true}});
  std::vector<DynamicTreeSkeleton::GpuRestState::Vertex> before;
  original.DownloadVertices(before);
  auto tree = std::make_shared<GpuTreeClusterPhysics>();
  ASSERT_TRUE(tree->InitializeDynamic(rest, original.GetTreeState(0)));
  EXPECT_EQ(tree->GetTreeState(), original.GetTreeState(0));
  GpuTreeClusterForest forest;
  ASSERT_TRUE(forest.Initialize({tree}));
  GpuTreeClusterForest::TreeStep step;
  step.running = false;
  step.lod.enabled_policies = 1;
  step.lod.rules[0] = {100, 1, 0, 2};
  const auto update = [&] {
    evo_engine::PlatformLifecycleTestAccess::PreUpdate();
    forest.Step({step});
    evo_engine::PlatformLifecycleTestAccess::LateUpdate();
    Platform::WaitForFrameSubmissions("Cluster shared state");
  };
  update();
  step.running = true;
  step.settings.time_step = 1e-8f;
  update();
  std::vector<DynamicTreeSkeleton::GpuRestState::Vertex> after;
  original.DownloadVertices(after);
  ASSERT_EQ(before.size(), after.size());
  for (size_t i = 0; i < before.size(); ++i)
    EXPECT_LT(glm::length(glm::vec3(before[i].position_inverse_mass - after[i].position_inverse_mass)), 1e-5f);
}

TEST(TreeClusterForest, SceneLifecycleRetainsNeighborsAndRebuildsAfterGrowth) {
  ScopedTreeGpuPlatform platform;
  EcoSysLabSystem::Dynamics dynamics;
  dynamics.tree_physics = std::make_shared<GpuTreePhysics>();
  std::vector<std::shared_ptr<Tree>> trees;
  std::vector<std::shared_ptr<DynamicTreeSkeleton>> components;
  for (int i = 0; i < 2; ++i) {
    auto tree = std::make_shared<Tree>();
    tree->shoot_model.Initialize(MakeGpuPhysicsFork());
    auto component = std::make_shared<DynamicTreeSkeleton>();
    ASSERT_TRUE(component->InitializePhysics(*tree));
    const auto id = dynamics.tree_physics->RegisterTree(component->ExportGpuRestState());
    ASSERT_NE(id, UINT32_MAX);
    component->SetGpuPhysics(dynamics.tree_physics, id);
    dynamics.entries.push_back({{}, tree, component, tree->GetShootModelRevision().content, id});
    trees.push_back(tree);
    components.push_back(component);
  }
  const auto sync = [&](bool growing) {
    evo_engine::PlatformLifecycleTestAccess::PreUpdate();
    dynamics.SyncClusters(growing);
    if (dynamics.cluster_forest) {
      std::vector<GpuTreeClusterForest::TreeStep> steps(2);
      for (size_t i = 0; i < 2; ++i) {
        steps[i].settings.time_step = 1e-5f;
        steps[i].lod = components[i]->physics_settings.cluster_lod;
        steps[i].pose_revision = dynamics.entries[i].pose_revision;
      }
      dynamics.cluster_forest->Step(steps);
    }
    evo_engine::PlatformLifecycleTestAccess::LateUpdate();
    Platform::WaitForFrameSubmissions("Cluster lifecycle");
  };
  dynamics.use_subtree_clusters = true;
  sync(false);
  ASSERT_TRUE(dynamics.cluster_forest);
  const auto first = dynamics.entries[0].cluster, neighbor = dynamics.entries[1].cluster;
  const auto forest = dynamics.cluster_forest;
  const auto neighbor_vertices = neighbor->GetTreeState()->vertices;
  dynamics.cluster_lod.enabled_policies = 3;
  dynamics.cluster_lod.rules[0] = {10, 1, 0, 2};
  sync(false);
  EXPECT_EQ(forest, dynamics.cluster_forest);
  EXPECT_EQ(first, dynamics.entries[0].cluster);
  EXPECT_EQ(components[0]->physics_settings.cluster_lod.enabled_policies, 3u);
  dynamics.use_subtree_clusters = false;
  sync(false);
  EXPECT_FALSE(dynamics.cluster_forest);
  EXPECT_FALSE(components[0]->physics_settings.use_subtree_clusters);
  dynamics.use_subtree_clusters = true;
  sync(false);
  EXPECT_EQ(first, dynamics.entries[0].cluster);
  sync(true);
  EXPECT_FALSE(dynamics.cluster_forest);
  trees[0]->shoot_model.Initialize(MakeGpuPhysicsChain());
  trees[0]->NotifyShootModelChanged();
  ASSERT_TRUE(components[0]->InitializePhysics(*trees[0]));
  ASSERT_EQ(dynamics.tree_physics->RegisterTree(components[0]->ExportGpuRestState(), dynamics.entries[0].gpu_id),
            dynamics.entries[0].gpu_id);
  components[0]->SetGpuPhysics(dynamics.tree_physics, dynamics.entries[0].gpu_id);
  sync(false);
  EXPECT_NE(first, dynamics.entries[0].cluster);
  EXPECT_EQ(neighbor, dynamics.entries[1].cluster);
  EXPECT_EQ(neighbor_vertices, dynamics.entries[1].cluster->GetTreeState()->vertices);
  dynamics.Reset();
  EXPECT_FALSE(components[0]->GetGpuPhysics());
  EXPECT_FALSE(dynamics.cluster_forest);
}

TEST(TreeClusterForest, RefinementUsesCurrentRigidMotionInsteadOfOldFineHistory) {
  ScopedTreeGpuPlatform platform;
  auto rest = MakeSubtreeFork();
  rest.branch_orders = {0, 1, 2, 2, 1};
  auto tree = std::make_shared<GpuTreeClusterPhysics>();
  ASSERT_TRUE(tree->InitializeDynamic(rest));
  GpuTreeClusterForest forest;
  ASSERT_TRUE(forest.Initialize({tree}));
  GpuTreeClusterForest::TreeStep step;
  step.running = false;
  const auto update = [&] {
    evo_engine::PlatformLifecycleTestAccess::PreUpdate();
    forest.Step({step});
    evo_engine::PlatformLifecycleTestAccess::LateUpdate();
    Platform::WaitForFrameSubmissions("Cluster momentum transfer");
  };
  update();
  const auto assign_rigid_motion = [&](const glm::vec3& omega, const glm::vec3& velocity) {
    std::vector<glm::uvec4> state;
    tree->GetLodStateBuffer()->DownloadVector(state, 6);
    auto motions = ClusterMotion(*tree);
    for (uint32_t i = 0; i < state[0].x; ++i) {
      motions[i].angular_velocity = {omega, 0};
      motions[i].anchor_velocity = {velocity + glm::cross(omega, glm::vec3(motions[i].anchor)), 0};
    }
    tree->GetMotionBuffer()->UploadVector(motions);
    for (uint32_t copy = 0; copy < 2; ++copy) {
      std::vector<glm::vec4> values;
      auto buffer = tree->GetTreeState()->vertices[copy];
      buffer->DownloadVector(values, rest.vertices.size() * 4);
      for (size_t i = 0; i < rest.vertices.size(); ++i)
        values[rest.vertices.size() + i] = {velocity + glm::cross(omega, glm::vec3(values[i])),
                                            values[rest.vertices.size() + i].w};
      buffer->UploadVector(values);
    }
  };
  const auto verify = [&](const glm::vec3& omega, const glm::vec3& velocity, uint32_t count) {
    std::vector<glm::uvec4> state;
    tree->GetLodStateBuffer()->DownloadVector(state, 6);
    ASSERT_EQ(state[0].x, count);
    const auto motions = ClusterMotion(*tree);
    for (uint32_t i = 0; i < count; ++i) {
      EXPECT_LT(glm::length(glm::vec3(motions[i].angular_velocity) - omega), 1e-4f);
      EXPECT_LT(glm::length(glm::vec3(motions[i].anchor_velocity) - velocity -
                            glm::cross(omega, glm::vec3(motions[i].anchor))),
                1e-4f);
    }
  };
  const glm::vec3 old_omega(0.2f, -0.3f, 0.4f), old_velocity(0.1f, 0.2f, -0.1f);
  assign_rigid_motion(old_omega, old_velocity);
  step.lod.enabled_policies = 2;
  step.lod.rules[1].z = 1;
  update();
  verify(old_omega, old_velocity, 3);
  const glm::vec3 current_omega(-0.5f, 0.4f, 0.1f), current_velocity(-0.2f, 0.1f, 0.3f);
  assign_rigid_motion(current_omega, current_velocity);
  step.lod.enabled_policies = 0;
  update();
  verify(current_omega, current_velocity, 5);
}
