#include "EvoEngine_SDK_PCH.hpp"

#include <gtest/gtest.h>

#include "Application.hpp"
#include "ApplicationContext.hpp"
#include "ComputePipeline.hpp"
#include "Cubemap.hpp"
#include "GpuService.hpp"
#include "GraphicsResources.hpp"
#include "Jobs.hpp"
#include "Mesh.hpp"
#include "Platform.hpp"
#include "RenderLayer.hpp"
#include "Shader.hpp"
#include "Texture2D.hpp"

#include <array>
#include <atomic>
#include <chrono>
#include <filesystem>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <vector>

using namespace evo_engine;

namespace evo_engine {
class PlatformLifecycleTestAccess final {
 public:
  static void Initialize(const ApplicationInitializationSettings& settings) {
    Platform::Initialize(settings);
  }

  static void PreUpdate() {
    Platform::PreUpdate();
  }

  static void OnDestroy() {
    Platform::OnDestroy();
  }
};
}  // namespace evo_engine

namespace {
TEST(StaticBlasBuilder, PlansDestinationBudgetAndOversizedSingletons) {
  constexpr VkDeviceSize budget = 512;
  const auto passes = PlanStaticBlasBuildPasses({{256, 64}, {256, 64}, {1, 64}, {600, 64}, {100, 64}}, 64, budget);
  ASSERT_EQ(passes.size(), 4u);
  EXPECT_EQ(passes[0].begin, 0u);
  EXPECT_EQ(passes[0].count, 2u);
  EXPECT_EQ(passes[0].destination_size, budget);
  EXPECT_FALSE(passes[0].oversized_singleton);
  EXPECT_EQ(passes[1].begin, 2u);
  EXPECT_EQ(passes[1].count, 1u);
  EXPECT_EQ(passes[2].begin, 3u);
  EXPECT_EQ(passes[2].count, 1u);
  EXPECT_EQ(passes[2].destination_size, 600u);
  EXPECT_TRUE(passes[2].oversized_singleton);
  EXPECT_EQ(passes[3].begin, 4u);
  EXPECT_EQ(passes[3].count, 1u);
}

TEST(StaticBlasBuilder, SizesSharedScratchAndWavesIndependently) {
  constexpr VkDeviceSize budget = 512;
  const auto compact = PlanStaticBlasBuildPasses({{1, 200}, {1, 200}}, 64, budget);
  ASSERT_EQ(compact.size(), 1u);
  EXPECT_EQ(compact[0].scratch_size, 512u);
  EXPECT_EQ(compact[0].scratch_wave_count, 1u);

  const auto waved = PlanStaticBlasBuildPasses({{1, 300}, {1, 300}}, 64, budget);
  ASSERT_EQ(waved.size(), 1u);
  EXPECT_EQ(waved[0].scratch_size, budget);
  EXPECT_EQ(waved[0].scratch_wave_count, 2u);

  const auto oversized = PlanStaticBlasBuildPasses({{1, 700}}, 64, budget);
  ASSERT_EQ(oversized.size(), 1u);
  EXPECT_EQ(oversized[0].scratch_size, 704u);
  EXPECT_EQ(oversized[0].scratch_wave_count, 1u);
  const auto isolated = PlanStaticBlasBuildPasses({{1, 64}, {1, 700}, {1, 64}}, 64, budget);
  ASSERT_EQ(isolated.size(), 3u);
  EXPECT_EQ(isolated[1].count, 1u);
  EXPECT_GT(isolated[1].scratch_size, budget);
  EXPECT_THROW(static_cast<void>(PlanStaticBlasBuildPasses({{1, 1}}, 1, 0)), std::invalid_argument);
}

class TempProject {
 public:
  TempProject() {
    const auto now = std::chrono::steady_clock::now().time_since_epoch().count();
    root_ = std::filesystem::temp_directory_path() / ("EvoEngineGpuServiceTest_" + std::to_string(now));
    std::filesystem::create_directories(root_ / "Assets");
  }

  ~TempProject() {
    std::error_code error;
    std::filesystem::remove_all(root_, error);
  }

  [[nodiscard]] std::filesystem::path ProjectPath() const {
    return root_ / "GpuServiceTest.eveproj";
  }

 private:
  std::filesystem::path root_;
};

ApplicationInitializationSettings TestApplicationSettings(const TempProject& project,
                                                          const bool use_ray_tracing = false) {
  ApplicationInitializationSettings settings;
  settings.project_path = project.ProjectPath();
  settings.load_default_resources = false;
  settings.load_project_assets = false;
  settings.load_project_start_scene = false;
  settings.enable_runtime_packages = false;
  settings.graphics_settings.use_mesh_shader = false;
  settings.graphics_settings.use_ray_tracing = use_ray_tracing;
  return settings;
}

class ScopedJobsRuntime {
 public:
  ScopedJobsRuntime() {
    Jobs::Initialize(2);
  }

  ~ScopedJobsRuntime() {
    Jobs::OnDestroy();
  }

  ScopedJobsRuntime(const ScopedJobsRuntime&) = delete;
  ScopedJobsRuntime& operator=(const ScopedJobsRuntime&) = delete;

 private:
  Application application_;
};

class ScopedGpuPlatform {
 public:
  explicit ScopedGpuPlatform(const bool use_ray_tracing = false) {
    project_ = std::make_unique<TempProject>();
    application_ = std::make_unique<Application>();
    application_->PushLayer<RenderLayer>("Render Layer");
    Jobs::Initialize(2);
    jobs_initialized_ = true;
    try {
      PlatformLifecycleTestAccess::Initialize(TestApplicationSettings(*project_, use_ray_tracing));
      platform_initialized_ = true;
    } catch (...) {
      if (jobs_initialized_) {
        Jobs::OnDestroy();
      }
      throw;
    }
  }

  ~ScopedGpuPlatform() {
    application_->PopLayer<RenderLayer>();
    if (platform_initialized_) {
      PlatformLifecycleTestAccess::OnDestroy();
    }
    if (jobs_initialized_) {
      Jobs::OnDestroy();
    }
    ApplicationContext::Set(nullptr);
    // This fixture initializes only Jobs and Platform. Avoid running Application's full destructor on a partial app.
    (void)application_.release();
    project_.reset();
  }

  ScopedGpuPlatform(const ScopedGpuPlatform&) = delete;
  ScopedGpuPlatform& operator=(const ScopedGpuPlatform&) = delete;

 private:
  std::unique_ptr<TempProject> project_;
  std::unique_ptr<Application> application_;
  bool jobs_initialized_ = false;
  bool platform_initialized_ = false;
};
}  // namespace

TEST(StaticBlasBuilder, PublishesCompactedSharedGeometryAfterGpuCompletion) {
  ScopedGpuPlatform platform(true);
  if (!Platform::RayAccelerationStructureEnabled()) {
    GTEST_SKIP() << "Acceleration structures are unavailable on the selected test device.";
  }

  Mesh mesh;
  mesh.OnCreate();
  VertexAttributes attributes{};
  attributes.normal = true;
  attributes.tangent = true;
  std::vector<Vertex> vertices(3);
  vertices[0].position = glm::vec3(0.0f, 0.0f, 0.0f);
  vertices[1].position = glm::vec3(1.0f, 0.0f, 0.0f);
  vertices[2].position = glm::vec3(0.0f, 1.0f, 0.0f);
  vertices[0].normal = vertices[1].normal = vertices[2].normal = glm::vec3(0.0f, 0.0f, 1.0f);
  vertices[0].tangent = vertices[1].tangent = vertices[2].tangent = glm::vec3(1.0f, 0.0f, 0.0f);

  const auto before = BottomLevelAccelerationStructure::GetStaticBuildTelemetry();
  mesh.SetVertices(attributes, vertices, {glm::uvec3(0, 1, 2)});
  const auto blas = mesh.GetBlas();
  ASSERT_TRUE(blas);
  EXPECT_FALSE(blas->IsReady());
  EXPECT_TRUE(BottomLevelAccelerationStructure::HasPendingStaticBuilds());

  BottomLevelAccelerationStructure::WaitForStaticBuilds();
  ASSERT_TRUE(blas->IsReady());
  EXPECT_NE(blas->GetDeviceAddress(), 0u);
  const auto after = BottomLevelAccelerationStructure::GetStaticBuildTelemetry();
  EXPECT_TRUE(after.complete);
  EXPECT_EQ(after.pending_count, 0u);
  EXPECT_EQ(after.static_eligible_count, before.static_eligible_count + 1);
  EXPECT_EQ(after.shared_input_count, before.shared_input_count + 1);
  EXPECT_EQ(after.cumulative_built_static_count, before.cumulative_built_static_count + 1);
  EXPECT_GT(after.cumulative_uncompacted_bytes, before.cumulative_uncompacted_bytes);
  EXPECT_GT(after.cumulative_compacted_bytes, before.cumulative_compacted_bytes);
  EXPECT_GT(after.eligible_static_uncompacted_bytes, before.eligible_static_uncompacted_bytes);
  EXPECT_GT(after.eligible_static_compacted_bytes, before.eligible_static_compacted_bytes);
  EXPECT_GT(after.final_compacted_storage_bytes, before.final_compacted_storage_bytes);
}

TEST(StaticBlasBuilder, GeometryWaitBarrierCommitsChangesDuringActiveBuild) {
  ScopedGpuPlatform platform(true);
  if (!Platform::RayAccelerationStructureEnabled()) {
    GTEST_SKIP() << "Acceleration structures are unavailable on the selected test device.";
  }

  Mesh mesh;
  mesh.OnCreate();
  VertexAttributes attributes{};
  attributes.normal = true;
  attributes.tangent = true;
  std::vector<Vertex> vertices(3);
  vertices[0].position = glm::vec3(0.0f, 0.0f, 0.0f);
  vertices[1].position = glm::vec3(1.0f, 0.0f, 0.0f);
  vertices[2].position = glm::vec3(0.0f, 1.0f, 0.0f);
  vertices[0].normal = vertices[1].normal = vertices[2].normal = glm::vec3(0.0f, 0.0f, 1.0f);
  vertices[0].tangent = vertices[1].tangent = vertices[2].tangent = glm::vec3(1.0f, 0.0f, 0.0f);
  const std::vector<glm::uvec3> triangles = {glm::uvec3(0, 1, 2)};

  mesh.SetVertices(attributes, vertices, triangles);
  GeometryStorage::WaitForPendingUploads();
  ASSERT_TRUE(BottomLevelAccelerationStructure::StaticBuildInProgress());

  vertices[0].position.x = 2.0f;
  mesh.SetVertices(attributes, vertices, triangles);
  ASSERT_TRUE(GeometryStorage::HasPendingUploads());
  const auto replacement_blas = mesh.GetBlas();
  const auto replacement_range = mesh.GetTriangleRange();

  GeometryStorage::WaitForPendingUploads();
  EXPECT_FALSE(GeometryStorage::HasPendingUploads());
  EXPECT_EQ(replacement_range->prev_frame_offset, replacement_range->offset);
  EXPECT_EQ(replacement_range->prev_frame_range, replacement_range->range);
  EXPECT_EQ(replacement_range->prev_frame_index_count, replacement_range->index_count);

  BottomLevelAccelerationStructure::WaitForStaticBuilds();
  ASSERT_TRUE(replacement_blas);
  EXPECT_TRUE(replacement_blas->IsReady());
}

TEST(StaticBlasBuilder, KeepsBuildHistoryAfterLiveStorageIsReleased) {
  ScopedGpuPlatform platform(true);
  if (!Platform::RayAccelerationStructureEnabled()) {
    GTEST_SKIP() << "Acceleration structures are unavailable on the selected test device.";
  }

  const auto before = BottomLevelAccelerationStructure::GetStaticBuildTelemetry();
  {
    Mesh mesh;
    mesh.OnCreate();
    VertexAttributes attributes{};
    attributes.normal = true;
    attributes.tangent = true;
    std::vector<Vertex> vertices(3);
    vertices[0].position = glm::vec3(0.0f, 0.0f, 0.0f);
    vertices[1].position = glm::vec3(1.0f, 0.0f, 0.0f);
    vertices[2].position = glm::vec3(0.0f, 1.0f, 0.0f);
    vertices[0].normal = vertices[1].normal = vertices[2].normal = glm::vec3(0.0f, 0.0f, 1.0f);
    vertices[0].tangent = vertices[1].tangent = vertices[2].tangent = glm::vec3(1.0f, 0.0f, 0.0f);
    mesh.SetVertices(attributes, vertices, {glm::uvec3(0, 1, 2)});
    BottomLevelAccelerationStructure::WaitForStaticBuilds();

    const auto built = BottomLevelAccelerationStructure::GetStaticBuildTelemetry();
    EXPECT_EQ(built.static_eligible_count, before.static_eligible_count + 1);
    EXPECT_EQ(built.cumulative_built_static_count, before.cumulative_built_static_count + 1);
    EXPECT_GT(built.cumulative_uncompacted_bytes, before.cumulative_uncompacted_bytes);
    EXPECT_GT(built.cumulative_compacted_bytes, before.cumulative_compacted_bytes);
  }

  const auto released = BottomLevelAccelerationStructure::GetStaticBuildTelemetry();
  EXPECT_EQ(released.static_eligible_count, before.static_eligible_count);
  EXPECT_EQ(released.shared_input_count, before.shared_input_count);
  EXPECT_EQ(released.eligible_static_uncompacted_bytes, before.eligible_static_uncompacted_bytes);
  EXPECT_EQ(released.eligible_static_compacted_bytes, before.eligible_static_compacted_bytes);
  EXPECT_EQ(released.final_compacted_storage_bytes, before.final_compacted_storage_bytes);
  EXPECT_EQ(released.cumulative_built_static_count, before.cumulative_built_static_count + 1);
  EXPECT_GT(released.cumulative_uncompacted_bytes, before.cumulative_uncompacted_bytes);
  EXPECT_GT(released.cumulative_compacted_bytes, before.cumulative_compacted_bytes);
  EXPECT_GT(released.pass_count, before.pass_count);
}

TEST(GpuService, EnqueuedWorkRunsOnGpuExecutor) {
  ScopedGpuPlatform platform;
  auto& gpu_service = Platform::GetGpuService();

  const auto main_thread_id = std::this_thread::get_id();
  std::thread::id executed_thread_id;
  bool executed_on_gpu_thread = false;
  bool executed_on_render_thread = false;

  const auto handle = gpu_service.Enqueue([&]() {
    executed_thread_id = std::this_thread::get_id();
    executed_on_gpu_thread = gpu_service.IsGpuThread();
    executed_on_render_thread = Jobs::IsExecutorThread(JobExecutorType::Render);
  });
  gpu_service.Wait(handle);

  EXPECT_TRUE(executed_on_gpu_thread);
  EXPECT_FALSE(executed_on_render_thread);
  EXPECT_NE(executed_thread_id, main_thread_id);
}

TEST(GpuService, RejectsSubmitAfterShutdown) {
  ScopedGpuPlatform platform;
  auto& gpu_service = Platform::GetGpuService();
  ASSERT_TRUE(gpu_service.Initialized());

  gpu_service.Shutdown();

  EXPECT_EQ(gpu_service.GetLifecycleState(), GpuService::LifecycleState::Stopped);
  EXPECT_THROW(
      {
        const auto handle = gpu_service.Enqueue([]() {
        });
        (void)handle;
      },
      std::runtime_error);
  EXPECT_THROW(gpu_service.SubmitImmediate([](VkCommandBuffer) {
  }),
               std::runtime_error);
}

TEST(GpuService, RejectsNestedImmediateSubmit) {
  ScopedGpuPlatform platform;
  auto& gpu_service = Platform::GetGpuService();

  EXPECT_NO_THROW(gpu_service.SubmitImmediate([&](VkCommandBuffer) {
    EXPECT_THROW(gpu_service.SubmitImmediate([](VkCommandBuffer) {
    }),
                 std::runtime_error);
  }));
}

TEST(GpuService, PropagatesGpuWorkExceptions) {
  ScopedGpuPlatform platform;
  auto& gpu_service = Platform::GetGpuService();

  const auto handle = gpu_service.Enqueue([]() {
    throw std::runtime_error("gpu work failure");
  });

  EXPECT_THROW(gpu_service.Wait(handle), std::runtime_error);
}

TEST(GpuService, ConcurrentSubmitRunsAllWork) {
  ScopedGpuPlatform platform;
  auto& gpu_service = Platform::GetGpuService();

  constexpr size_t submit_count = 16;
  std::atomic_size_t executed_count = 0;
  std::vector<std::thread> submit_threads;
  std::vector<GpuWorkHandle> handles(submit_count);
  std::mutex handles_mutex;

  for (size_t i = 0; i < submit_count; ++i) {
    submit_threads.emplace_back([&gpu_service, &executed_count, &handles, &handles_mutex, i]() {
      auto handle = gpu_service.Enqueue([&executed_count]() {
        ++executed_count;
      });
      std::lock_guard lock(handles_mutex);
      handles[i] = handle;
    });
  }

  for (auto& thread : submit_threads) {
    thread.join();
  }
  for (const auto& handle : handles) {
    gpu_service.Wait(handle);
  }

  EXPECT_EQ(executed_count.load(), submit_count);
}

TEST(GpuService, BufferUploadReadbackRoundTrip) {
  ScopedGpuPlatform platform;
  ASSERT_TRUE(Platform::Initialized());
  auto& gpu_service = Platform::GetGpuService();
  ASSERT_TRUE(gpu_service.Initialized());

  constexpr std::array<uint32_t, 8> input = {0x12345678u, 0x90abcdefu, 0x0u,        0xffffffffu,
                                             0x31415926u, 0x27182818u, 0xfeedbeefu, 0xc001d00du};
  constexpr auto byte_size = static_cast<VkDeviceSize>(input.size() * sizeof(input[0]));

  VkBufferCreateInfo buffer_create_info{};
  buffer_create_info.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
  buffer_create_info.size = byte_size;
  buffer_create_info.usage =
      VK_BUFFER_USAGE_TRANSFER_SRC_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_STORAGE_BUFFER_BIT;
  buffer_create_info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;

  VmaAllocationCreateInfo allocation_create_info{};
  allocation_create_info.usage = VMA_MEMORY_USAGE_AUTO;

  {
    Buffer buffer(buffer_create_info, allocation_create_info);
    const auto upload = buffer.UploadDataAsync(static_cast<size_t>(byte_size), input.data());
    gpu_service.Wait(upload);

    const auto downloaded_bytes = buffer.DownloadDataAsync(static_cast<size_t>(byte_size)).get();
    ASSERT_EQ(downloaded_bytes.size(), static_cast<size_t>(byte_size));

    std::array<uint32_t, input.size()> output{};
    memcpy(output.data(), downloaded_bytes.data(), downloaded_bytes.size());
    EXPECT_EQ(output, input);
  }
}

TEST(GpuService, CubemapFaceMipUploadReadbackRoundTrip) {
  ScopedGpuPlatform platform;
  Cubemap cubemap;
  constexpr uint32_t resolution = 2;
  constexpr uint32_t mip_levels = 2;
  std::vector<glm::vec4> expected(30);
  for (size_t index = 0; index < expected.size(); ++index) {
    expected[index] =
        glm::vec4(static_cast<float>(index), static_cast<float>(index) + 0.25f, static_cast<float>(index) + 0.5f, 1.0f);
  }

  ASSERT_TRUE(cubemap.SetRgbaChannelData(expected, resolution, mip_levels));
  EXPECT_EQ(cubemap.GetResolution(), resolution);
  EXPECT_EQ(cubemap.GetMipLevels(), mip_levels);
  std::vector<glm::vec4> restored;
  cubemap.GetRgbaChannelData(restored, true);
  ASSERT_EQ(restored.size(), expected.size());
  for (size_t index = 0; index < expected.size(); ++index) {
    for (int channel = 0; channel < 4; ++channel) {
      EXPECT_FLOAT_EQ(restored[index][channel], expected[index][channel]);
    }
  }
  ASSERT_TRUE(cubemap.GetImage());
  EXPECT_EQ(cubemap.GetImage()->GetLayout(), VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
}

TEST(GpuService, CubemapUninitializedGpuStorageRejectsReadback) {
  ScopedGpuPlatform platform;
  Cubemap cubemap;
  cubemap.Initialize(2u);
  ASSERT_TRUE(cubemap.GetImage());
  ASSERT_EQ(cubemap.GetImage()->GetLayout(), VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);

  std::vector<glm::vec4> pixels = {glm::vec4(1.0f)};
  cubemap.GetRgbaChannelData(pixels);
  EXPECT_TRUE(pixels.empty());
  EXPECT_TRUE(cubemap.PeekLocalData().empty());
  EXPECT_EQ(cubemap.GetImage()->GetLayout(), VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
}

TEST(GpuService, GltfRayTracingNumericalProbeMatchesAnalyticValues) {
  ScopedGpuPlatform platform;
  const auto shader_root =
      std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / "EvoEngine_SDK" / "Internals" / "DefaultResources" / "Shaders";
  Shader::RegisterShaderIncludePath(shader_root / "Includes");

  auto descriptor_layout = std::make_shared<DescriptorSetLayout>();
  descriptor_layout->PushDescriptorBinding(0, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_COMPUTE_BIT, 0);
  descriptor_layout->Initialize();

  constexpr size_t value_count = 38;
  VkBufferCreateInfo buffer_create_info{};
  buffer_create_info.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
  buffer_create_info.size = value_count * sizeof(float);
  buffer_create_info.usage =
      VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_SRC_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT;
  buffer_create_info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
  VmaAllocationCreateInfo allocation_create_info{};
  allocation_create_info.usage = VMA_MEMORY_USAGE_AUTO_PREFER_DEVICE;
  auto output = std::make_shared<Buffer>(buffer_create_info, allocation_create_info);
  const std::array<float, value_count> zero{};
  output->Upload(zero);

  auto descriptor_set = std::make_shared<DescriptorSet>(descriptor_layout);
  descriptor_set->UpdateBufferDescriptorBinding(0, output);
  auto shader = std::make_shared<Shader>();
  ASSERT_TRUE(shader->TryCompile(ShaderType::Compute, Platform::GetShaderGlobalDefines(),
                                 shader_root / "Compute" / "GltfRayTracingNumericalProbe.comp"));
  auto pipeline = std::make_shared<ComputePipeline>();
  pipeline->compute_shader = shader;
  pipeline->descriptor_set_layouts.emplace_back(descriptor_layout);
  pipeline->Initialize();
  ASSERT_TRUE(pipeline->Initialized());

  Platform::ImmediateSubmit([&](const VkCommandBuffer command_buffer) {
    pipeline->Bind(command_buffer);
    pipeline->BindDescriptorSet(command_buffer, 0, descriptor_set->GetVkDescriptorSet());
    pipeline->Dispatch(command_buffer, 1);
    Platform::EverythingBarrier(command_buffer);
  });
  std::array<float, value_count> values{};
  output->Download(values);
  EXPECT_NEAR(values[0], 0.04f, 1.0e-6f);
  EXPECT_NEAR(values[1], 0.04f, 1.0e-6f);
  EXPECT_NEAR(values[2], 1.0f, 1.0e-6f);
  EXPECT_NEAR(values[3], 1.0f, 1.0e-6f);
  EXPECT_GE(values[4], 0.0f);
  EXPECT_NEAR(values[5], values[6], 1.0e-6f);
  EXPECT_NEAR(values[7], 1.5f, 1.0e-5f);
  EXPECT_FLOAT_EQ(values[8], 1.0f);
  EXPECT_NEAR(values[9], 0.04f, 1.0e-6f);
  EXPECT_NEAR(values[10], 0.01f, 1.0e-6f);
  EXPECT_NEAR(values[11], 0.02f, 1.0e-6f);
  EXPECT_NEAR(values[12], 0.5f, 1.0e-6f);
  EXPECT_NEAR(values[13], 0.5f, 1.0e-6f);
  EXPECT_NEAR(values[14], 0.3f / std::sqrt(1.18f), 1.0e-6f);
  EXPECT_NEAR(values[15], -0.3f / std::sqrt(1.18f), 1.0e-6f);
  EXPECT_NEAR(values[16], 1.0f, 1.0e-6f);
  EXPECT_NEAR(values[17], 1.92f, 1.0e-6f);
  EXPECT_NEAR(values[18], 0.0f, 1.0e-6f);
  EXPECT_FLOAT_EQ(values[19], 1.0f);
  for (const auto offset : {20u, 23u, 32u, 35u}) {
    EXPECT_NEAR(values[offset], 0.0f, 1.0e-6f);
    EXPECT_NEAR(values[offset + 1], 0.0f, 1.0e-6f);
    EXPECT_NEAR(values[offset + 2], 1.0f, 1.0e-6f);
  }
  const glm::vec3 normal_fallback = glm::normalize(glm::vec3(1.0f, 2.0f, 3.0f));
  for (const auto offset : {26u, 29u}) {
    EXPECT_NEAR(values[offset], normal_fallback.x, 1.0e-6f);
    EXPECT_NEAR(values[offset + 1], normal_fallback.y, 1.0e-6f);
    EXPECT_NEAR(values[offset + 2], normal_fallback.z, 1.0e-6f);
  }
}

TEST(GpuService, M10CameraRayTransportShadersCompile) {
  ScopedGpuPlatform platform;
  const auto shader_root =
      std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / "EvoEngine_SDK" / "Internals" / "DefaultResources" / "Shaders";
  Shader::RegisterShaderIncludePath(shader_root / "Includes");
  const auto header = Platform::GetShaderGlobalDefines();

  Shader raygen;
  Shader any_hit;
  Shader closest_hit;
  Shader miss;
  Shader ray_query;
  EXPECT_TRUE(raygen.TryCompile(ShaderType::RayGen, header, shader_root / "RayTracing/RayGen/Camera.rgen"));
  EXPECT_TRUE(any_hit.TryCompile(ShaderType::AnyHit, header, shader_root / "RayTracing/AnyHit/Camera.rahit"));
  EXPECT_TRUE(
      closest_hit.TryCompile(ShaderType::ClosestHit, header, shader_root / "RayTracing/ClosestHit/Camera.rchit"));
  EXPECT_TRUE(miss.TryCompile(ShaderType::Miss, header, shader_root / "RayTracing/Miss/Camera.rmiss"));
  EXPECT_TRUE(ray_query.TryCompile(ShaderType::Compute, header, shader_root / "Compute/RayQueryCamera.comp"));
}

TEST(GpuService, BufferUploadSubrangeRoundTrip) {
  ScopedGpuPlatform platform;
  auto& gpu_service = Platform::GetGpuService();

  constexpr std::array<uint32_t, 8> input = {0u, 1u, 2u, 3u, 4u, 5u, 6u, 7u};
  constexpr std::array<uint32_t, 2> patch = {42u, 43u};
  constexpr auto byte_size = static_cast<VkDeviceSize>(input.size() * sizeof(input[0]));
  constexpr auto patch_offset = static_cast<VkDeviceSize>(3 * sizeof(input[0]));

  VkBufferCreateInfo buffer_create_info{};
  buffer_create_info.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
  buffer_create_info.size = byte_size;
  buffer_create_info.usage =
      VK_BUFFER_USAGE_TRANSFER_SRC_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_STORAGE_BUFFER_BIT;
  buffer_create_info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;

  VmaAllocationCreateInfo allocation_create_info{};
  allocation_create_info.usage = VMA_MEMORY_USAGE_AUTO;

  Buffer buffer(buffer_create_info, allocation_create_info);
  gpu_service.Wait(buffer.UploadDataAsync(static_cast<size_t>(byte_size), input.data()));
  gpu_service.Wait(buffer.UploadSubDataAsync(patch.size() * sizeof(patch[0]), patch.data(), patch_offset));

  const auto downloaded_bytes = buffer.DownloadDataAsync(static_cast<size_t>(byte_size)).get();
  std::array<uint32_t, input.size()> output{};
  memcpy(output.data(), downloaded_bytes.data(), downloaded_bytes.size());

  auto expected = input;
  expected[3] = patch[0];
  expected[4] = patch[1];
  EXPECT_EQ(output, expected);
}

TEST(GpuService, Texture2DAsyncUploadProducesReadyImage) {
  ScopedGpuPlatform platform;
  auto& gpu_service = Platform::GetGpuService();

  Texture2D texture;
  std::vector<glm::vec4> pixels = {
      glm::vec4(1.0f, 0.0f, 0.0f, 1.0f),
      glm::vec4(0.0f, 1.0f, 0.0f, 1.0f),
      glm::vec4(0.0f, 0.0f, 1.0f, 1.0f),
      glm::vec4(1.0f, 1.0f, 1.0f, 1.0f),
  };
  auto& texture_storage = texture.RefTexture2DStorage();
  const auto version_before_upload = TextureStorage::GetVersion();

  const auto upload = texture_storage.SetDataAsync(pixels, glm::uvec2(2, 2));
  ASSERT_TRUE(upload.Valid());
  EXPECT_TRUE(texture_storage.IsGpuUploadPending());
  gpu_service.Wait(upload);
  TextureStorage::DeviceSync();

  EXPECT_FALSE(texture_storage.IsGpuUploadPending());
  EXPECT_GT(TextureStorage::GetVersion(), version_before_upload);
  EXPECT_NE(texture_storage.GetVkImage(), VK_NULL_HANDLE);
  EXPECT_NE(texture_storage.GetVkImageView(), VK_NULL_HANDLE);
  EXPECT_NE(texture_storage.GetVkSampler(), VK_NULL_HANDLE);
  EXPECT_EQ(texture_storage.GetLayout(), VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
  EXPECT_EQ(texture_storage.GetMipLevels(), 2u);

  Buffer mip_readback(sizeof(glm::vec4), true);
  VkBufferImageCopy mip_copy{};
  mip_copy.imageSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
  mip_copy.imageSubresource.mipLevel = 1;
  mip_copy.imageSubresource.layerCount = 1;
  mip_copy.imageExtent = {1, 1, 1};
  mip_readback.CopyFromImage(*texture_storage.GetImage(), mip_copy);
  const auto mip_bytes = mip_readback.DownloadDataAsync(sizeof(glm::vec4)).get();
  ASSERT_EQ(mip_bytes.size(), sizeof(glm::vec4));
  glm::vec4 mip_pixel;
  memcpy(&mip_pixel, mip_bytes.data(), sizeof(mip_pixel));
  EXPECT_NEAR(mip_pixel.r, 0.5f, 0.001f);
  EXPECT_NEAR(mip_pixel.g, 0.5f, 0.001f);
  EXPECT_NEAR(mip_pixel.b, 0.5f, 0.001f);
  EXPECT_NEAR(mip_pixel.a, 1.0f, 0.001f);
}

TEST(GpuService, Texture2DSharedColorSpaceViewReusesCompressedImage) {
  ScopedGpuPlatform platform;
  if (!Platform::GetSelectedPhysicalDevice()->features.textureCompressionBC) {
    GTEST_SKIP() << "BC texture compression is unavailable on the selected test device.";
  }
  Texture2D source;
  std::vector<std::byte> block(16);
  const auto upload =
      source.RefTexture2DStorage().SetCompressedDataAsync(block, glm::uvec2(4), VK_FORMAT_BC7_UNORM_BLOCK);
  ASSERT_TRUE(upload.Valid());
  Platform::GetGpuService().Wait(upload);
  TextureStorage::DeviceSync();
  const auto version_before_share = TextureStorage::GetVersion();

  Texture2DSamplerSettings sampler;
  sampler.address_mode_u = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
  const auto source_image = source.GetVkImage();
  const auto source_view = source.GetVkImageView();
  EXPECT_FALSE(source.ShareGpuImage(source, true, sampler));
  EXPECT_EQ(source.GetVkImage(), source_image);
  EXPECT_EQ(source.GetVkImageView(), source_view);

  Texture2D view;
  ASSERT_TRUE(view.ShareGpuImage(source, true, sampler));
  EXPECT_GT(TextureStorage::GetVersion(), version_before_share);
  EXPECT_NE(view.GetVkImageView(), VK_NULL_HANDLE);
  EXPECT_NE(view.GetVkSampler(), VK_NULL_HANDLE);
  EXPECT_EQ(view.GetVkImage(), source.GetVkImage());
  EXPECT_NE(view.GetVkImageView(), source.GetVkImageView());
  EXPECT_EQ(source.RefTexture2DStorage().GetFormat(), VK_FORMAT_BC7_UNORM_BLOCK);
  EXPECT_EQ(view.RefTexture2DStorage().GetFormat(), VK_FORMAT_BC7_SRGB_BLOCK);
  EXPECT_TRUE(view.SamplesLinearSrgb());
  EXPECT_EQ(view.GetSamplerSettings(), sampler);

  const auto shared_image = view.GetVkImage();
  source.RefTexture2DStorage().Clear();
  EXPECT_EQ(view.GetVkImage(), shared_image);
}

TEST(GpuService, Texture2DSharedPredecodedSrgbImagePreservesSamplingSemantics) {
  ScopedGpuPlatform platform;
  Texture2D source;
  source.srgb = true;
  const auto upload = source.RefTexture2DStorage().SetDataAsync({glm::vec4(0.5f, 0.25f, 0.125f, 1.0f)}, glm::uvec2(1),
                                                                VK_FORMAT_UNDEFINED, true);
  ASSERT_TRUE(upload.Valid());
  Platform::GetGpuService().Wait(upload);

  Texture2D view;
  ASSERT_TRUE(view.ShareGpuImage(source, true, {}));
  EXPECT_EQ(view.GetVkImage(), source.GetVkImage());
  EXPECT_EQ(view.RefTexture2DStorage().GetFormat(), Platform::Constants::texture_2d);
  EXPECT_TRUE(view.SamplesLinearSrgb());
}

TEST(GpuService, Texture2DRuntimeUpdateTracksGpuReadiness) {
  ScopedGpuPlatform platform;

  Texture2D texture;
  const std::vector<glm::vec4> pixels = {
      glm::vec4(0.2f, 0.0f, 0.0f, 1.0f),
      glm::vec4(0.0f, 0.4f, 0.0f, 1.0f),
      glm::vec4(0.0f, 0.0f, 0.6f, 1.0f),
      glm::vec4(0.8f, 0.8f, 0.8f, 1.0f),
  };
  auto& texture_storage = texture.RefTexture2DStorage();

  texture.SetRgbaChannelData(pixels, glm::uvec2(2, 2));
  EXPECT_TRUE(texture_storage.IsGpuUploadPending());
  Platform::GetGpuService().WaitIdle();

  EXPECT_FALSE(texture_storage.IsGpuUploadPending());
  EXPECT_EQ(texture_storage.GetLayout(), VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);

  std::vector<glm::vec4> readback;
  texture.GetRgbaChannelData(readback);
  ASSERT_EQ(readback.size(), pixels.size());
  for (size_t i = 0; i < pixels.size(); ++i) {
    EXPECT_NEAR(readback[i].r, pixels[i].r, 0.001f);
    EXPECT_NEAR(readback[i].g, pixels[i].g, 0.001f);
    EXPECT_NEAR(readback[i].b, pixels[i].b, 0.001f);
    EXPECT_NEAR(readback[i].a, pixels[i].a, 0.001f);
  }
}

TEST(GpuService, Texture2DRejectsUndersizedUpload) {
  ScopedGpuPlatform platform;
  Texture2D texture;
  auto& texture_storage = texture.RefTexture2DStorage();
  const auto original_image = texture_storage.GetVkImage();
  ASSERT_NE(original_image, VK_NULL_HANDLE);
  EXPECT_FALSE(texture_storage.SetDataAsync({glm::vec4(1.0f)}, glm::uvec2(2, 2)).Valid());
  EXPECT_EQ(texture_storage.GetVkImage(), original_image);
}

TEST(GpuService, Texture2DClearAfterImGuiShutdownSkipsBackendRemoval) {
  ScopedGpuPlatform platform;
  ASSERT_EQ(ImGui::GetCurrentContext(), nullptr);
  Texture2D texture;
  auto& texture_storage = texture.RefTexture2DStorage();
  texture_storage.im_texture_id = static_cast<ImTextureID>(1);
  texture_storage.Clear();
  EXPECT_EQ(texture_storage.im_texture_id, 0);
}

TEST(GpuService, TextureStorageDeviceSyncUsesAsyncUploadPath) {
  ScopedGpuPlatform platform;
  auto& gpu_service = Platform::GetGpuService();

  Texture2D texture;
  const std::vector<glm::vec4> pixels = {
      glm::vec4(1.0f, 0.25f, 0.0f, 1.0f),
      glm::vec4(0.0f, 1.0f, 0.25f, 1.0f),
      glm::vec4(0.25f, 0.0f, 1.0f, 1.0f),
      glm::vec4(1.0f, 1.0f, 0.25f, 1.0f),
  };
  auto& texture_storage = texture.RefTexture2DStorage();

  texture_storage.SetData(pixels, glm::uvec2(2, 2));
  TextureStorage::DeviceSync();
  gpu_service.WaitIdle();

  EXPECT_FALSE(texture_storage.IsGpuUploadPending());
  EXPECT_NE(texture_storage.GetVkImage(), VK_NULL_HANDLE);
  EXPECT_NE(texture_storage.GetVkImageView(), VK_NULL_HANDLE);
  EXPECT_NE(texture_storage.GetVkSampler(), VK_NULL_HANDLE);
  EXPECT_EQ(texture_storage.GetLayout(), VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
}

TEST(GpuService, GeometryStorageCommitsMeshRangesAfterAsyncUpload) {
  ScopedGpuPlatform platform;
  auto& gpu_service = Platform::GetGpuService();

  Mesh mesh;
  mesh.OnCreate();
  VertexAttributes attributes{};
  attributes.normal = true;
  attributes.tangent = true;
  std::vector<Vertex> vertices(3);
  vertices[0].position = glm::vec3(0.0f, 0.0f, 0.0f);
  vertices[1].position = glm::vec3(1.0f, 0.0f, 0.0f);
  vertices[2].position = glm::vec3(0.0f, 1.0f, 0.0f);
  vertices[0].normal = vertices[1].normal = vertices[2].normal = glm::vec3(0.0f, 0.0f, 1.0f);
  vertices[0].tangent = vertices[1].tangent = vertices[2].tangent = glm::vec3(1.0f, 0.0f, 0.0f);
  const std::vector<glm::uvec3> triangles = {glm::uvec3(0, 1, 2)};

  mesh.SetVertices(attributes, vertices, triangles);
  const auto& triangle_range = mesh.GetTriangleRange();
  ASSERT_TRUE(triangle_range);
  EXPECT_EQ(triangle_range->prev_frame_index_count, 0u);
  EXPECT_EQ(triangle_range->prev_frame_range, 0u);

  const auto version_before_sync = GeometryStorage::GetVersion();
  PlatformLifecycleTestAccess::PreUpdate();
  gpu_service.WaitIdle();
  PlatformLifecycleTestAccess::PreUpdate();

  EXPECT_GT(GeometryStorage::GetVersion(), version_before_sync);
  EXPECT_EQ(triangle_range->prev_frame_index_count, 1u);
  EXPECT_EQ(triangle_range->prev_frame_range, triangle_range->range);
  EXPECT_EQ(triangle_range->prev_frame_offset, triangle_range->offset);
}

TEST(GpuService, GeometryStorageUploadsCompactedMeshTail) {
  ScopedGpuPlatform platform;
  auto& gpu_service = Platform::GetGpuService();

  VertexAttributes attributes{};
  attributes.normal = true;
  attributes.tangent = true;
  const std::vector<glm::uvec3> triangles = {glm::uvec3(0, 1, 2)};

  auto first_mesh = std::make_unique<Mesh>();
  first_mesh->OnCreate();
  std::vector<Vertex> first_vertices(3);
  first_vertices[0].position = glm::vec3(1.0f, 0.0f, 0.0f);
  first_vertices[1].position = glm::vec3(2.0f, 0.0f, 0.0f);
  first_vertices[2].position = glm::vec3(1.0f, 1.0f, 0.0f);
  first_mesh->SetVertices(attributes, first_vertices, triangles);

  auto second_mesh = std::make_unique<Mesh>();
  second_mesh->OnCreate();
  std::vector<Vertex> second_vertices(3);
  second_vertices[0].position = glm::vec3(10.0f, 0.0f, 0.0f);
  second_vertices[1].position = glm::vec3(11.0f, 0.0f, 0.0f);
  second_vertices[2].position = glm::vec3(10.0f, 1.0f, 0.0f);
  second_mesh->SetVertices(attributes, second_vertices, triangles);

  PlatformLifecycleTestAccess::PreUpdate();
  gpu_service.WaitIdle();
  PlatformLifecycleTestAccess::PreUpdate();
  ASSERT_EQ(second_mesh->GetTriangleRange()->prev_frame_offset, 1u);

  first_mesh.reset();
  BottomLevelAccelerationStructure::WaitForStaticBuilds();
  GeometryStorage::WaitForPendingUploads();

  ASSERT_FALSE(GeometryStorage::HasPendingUploads());
  ASSERT_EQ(second_mesh->GetTriangleRange()->prev_frame_offset, 0u);

  const auto downloaded_bytes = GeometryStorage::GetVertexBuffer()->DownloadDataAsync(sizeof(VertexDataChunk)).get();
  VertexDataChunk output{};
  memcpy(&output, downloaded_bytes.data(), downloaded_bytes.size());
  EXPECT_FLOAT_EQ(output.vertex_data[0].position.x, second_vertices[0].position.x);
  EXPECT_FLOAT_EQ(output.vertex_data[0].position.y, second_vertices[0].position.y);
  EXPECT_FLOAT_EQ(output.vertex_data[0].position.z, second_vertices[0].position.z);
}

TEST(GpuService, ConcurrentBufferUploadsRoundTrip) {
  ScopedGpuPlatform platform;
  auto& gpu_service = Platform::GetGpuService();

  constexpr size_t buffer_count = 6;
  constexpr size_t element_count = 8;
  constexpr auto byte_size = static_cast<VkDeviceSize>(element_count * sizeof(uint32_t));

  VkBufferCreateInfo buffer_create_info{};
  buffer_create_info.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
  buffer_create_info.size = byte_size;
  buffer_create_info.usage =
      VK_BUFFER_USAGE_TRANSFER_SRC_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_STORAGE_BUFFER_BIT;
  buffer_create_info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;

  VmaAllocationCreateInfo allocation_create_info{};
  allocation_create_info.usage = VMA_MEMORY_USAGE_AUTO;

  std::vector<std::array<uint32_t, element_count>> inputs(buffer_count);
  std::vector<std::unique_ptr<Buffer>> buffers;
  std::vector<GpuWorkHandle> uploads;
  for (size_t i = 0; i < buffer_count; ++i) {
    for (size_t j = 0; j < element_count; ++j) {
      inputs[i][j] = static_cast<uint32_t>(i * 100 + j);
    }
    buffers.emplace_back(std::make_unique<Buffer>(buffer_create_info, allocation_create_info));
    uploads.emplace_back(buffers.back()->UploadDataAsync(static_cast<size_t>(byte_size), inputs[i].data()));
  }

  for (const auto& upload : uploads) {
    gpu_service.Wait(upload);
  }

  for (size_t i = 0; i < buffer_count; ++i) {
    const auto downloaded_bytes = buffers[i]->DownloadDataAsync(static_cast<size_t>(byte_size)).get();
    std::array<uint32_t, element_count> output{};
    memcpy(output.data(), downloaded_bytes.data(), downloaded_bytes.size());
    EXPECT_EQ(output, inputs[i]);
  }
}

TEST(GpuService, BufferDestroyWaitsForPendingUpload) {
  ScopedGpuPlatform platform;

  constexpr std::array<uint32_t, 8> input = {0u, 1u, 2u, 3u, 4u, 5u, 6u, 7u};
  constexpr auto byte_size = static_cast<VkDeviceSize>(input.size() * sizeof(input[0]));

  VkBufferCreateInfo buffer_create_info{};
  buffer_create_info.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
  buffer_create_info.size = byte_size;
  buffer_create_info.usage =
      VK_BUFFER_USAGE_TRANSFER_SRC_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_STORAGE_BUFFER_BIT;
  buffer_create_info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;

  VmaAllocationCreateInfo allocation_create_info{};
  allocation_create_info.usage = VMA_MEMORY_USAGE_AUTO;

  EXPECT_NO_THROW({
    Buffer buffer(buffer_create_info, allocation_create_info);
    const auto upload = buffer.UploadDataAsync(static_cast<size_t>(byte_size), input.data());
    (void)upload;
  });
}

TEST(GpuService, StagingBudgetThrottlesQueuedUploads) {
  ScopedGpuPlatform platform;
  auto& gpu_service = Platform::GetGpuService();

  constexpr std::array<uint32_t, 4> input = {9u, 8u, 7u, 6u};
  constexpr auto byte_size = static_cast<VkDeviceSize>(input.size() * sizeof(input[0]));
  constexpr size_t upload_count = 8;

  VkBufferCreateInfo buffer_create_info{};
  buffer_create_info.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
  buffer_create_info.size = byte_size;
  buffer_create_info.usage =
      VK_BUFFER_USAGE_TRANSFER_SRC_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_STORAGE_BUFFER_BIT;
  buffer_create_info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;

  VmaAllocationCreateInfo allocation_create_info{};
  allocation_create_info.usage = VMA_MEMORY_USAGE_AUTO;

  std::vector<std::unique_ptr<Buffer>> buffers;
  std::vector<GpuWorkHandle> uploads;
  for (size_t i = 0; i < upload_count; ++i) {
    buffers.emplace_back(std::make_unique<Buffer>(buffer_create_info, allocation_create_info));
    uploads.emplace_back(buffers.back()->UploadDataAsync(static_cast<size_t>(byte_size), input.data()));
  }

  for (const auto& upload : uploads) {
    gpu_service.Wait(upload);
  }

  SUCCEED();
}
