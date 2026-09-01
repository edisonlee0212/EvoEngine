#include "EvoEngine_SDK_PCH.hpp"

#include <gtest/gtest.h>

#include "Application.hpp"
#include "ApplicationContext.hpp"
#include "ComputePipeline.hpp"
#include "Cubemap.hpp"
#include "GeometryStorage.hpp"
#include "GltfMaterial.hpp"
#include "GpuService.hpp"
#include "GraphicsResources.hpp"
#include "Jobs.hpp"
#include "Lights.hpp"
#include "Mesh.hpp"
#include "Platform.hpp"
#include "RenderLayer.hpp"
#include "Shader.hpp"
#include "Strands.hpp"
#include "Texture2D.hpp"
#include "TextureStorage.hpp"

#include <array>
#include <atomic>
#include <chrono>
#include <cstring>
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
      const auto settings = TestApplicationSettings(*project_, use_ray_tracing);
      const_cast<ApplicationInitializationSettings&>(application_->GetApplicationInfo()) = settings;
      PlatformLifecycleTestAccess::Initialize(settings);
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
      Platform::DrainGpuResourceWork();
      TextureStorage::OnDestroy();
      GeometryStorage::OnDestroy();
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

TEST(GpuService, PresentationReadinessAllowsDynamicParticleUpdatesAfterFirstUpload) {
  ScopedGpuPlatform platform;
  ParticleInfoList particles;
  particles.OnCreate();
  particles.SetParticleInfos({ParticleInfo{}});

  EXPECT_TRUE(GeometryStorage::HasPendingPresentationUploads());
  PlatformLifecycleTestAccess::PreUpdate();
  EXPECT_FALSE(GeometryStorage::HasPendingPresentationUploads());

  particles.SetParticleInfos({ParticleInfo{}});
  EXPECT_TRUE(GeometryStorage::HasPendingUploads());
  EXPECT_FALSE(GeometryStorage::HasPendingPresentationUploads());

  particles.SetParticleInfos({ParticleInfo{}, ParticleInfo{}});
  EXPECT_TRUE(GeometryStorage::HasPendingPresentationUploads());
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

TEST(GpuService, RasterOnlyConfigurationSamplesSharedBindlessTextureArrays) {
  ScopedGpuPlatform platform;
  ASSERT_FALSE(Platform::RayTracingEnabled());
  ASSERT_FALSE(Platform::RayQueryEnabled());

  Texture2D texture_2d_a;
  Texture2D texture_2d_b;
  const auto texture_2d_a_upload =
      texture_2d_a.RefTexture2DStorage().SetDataAsync({glm::vec4(1.0f, 0.0f, 0.0f, 1.0f)}, {1, 1});
  const auto texture_2d_b_upload =
      texture_2d_b.RefTexture2DStorage().SetDataAsync({glm::vec4(0.0f, 1.0f, 0.0f, 1.0f)}, {1, 1});
  ASSERT_TRUE(texture_2d_a_upload.Valid());
  ASSERT_TRUE(texture_2d_b_upload.Valid());
  Platform::GetGpuService().Wait(texture_2d_a_upload);
  Platform::GetGpuService().Wait(texture_2d_b_upload);
  TextureStorage::DeviceSync();
  Cubemap cubemap_a;
  Cubemap cubemap_b;
  ASSERT_TRUE(cubemap_a.SetRgbaChannelData(std::vector<glm::vec4>(6, glm::vec4(0.0f, 0.0f, 1.0f, 1.0f)), 1));
  ASSERT_TRUE(cubemap_b.SetRgbaChannelData(std::vector<glm::vec4>(6, glm::vec4(1.0f, 1.0f, 0.0f, 1.0f)), 1));

  const GraphicsInitializationSettings settings;
  auto layout = std::make_shared<DescriptorSetLayout>();
  layout->PushDescriptorBinding(9, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, VK_SHADER_STAGE_COMPUTE_BIT,
                                VK_DESCRIPTOR_BINDING_PARTIALLY_BOUND_BIT, settings.max_texture_2d_resource_size);
  layout->PushDescriptorBinding(10, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, VK_SHADER_STAGE_COMPUTE_BIT,
                                VK_DESCRIPTOR_BINDING_PARTIALLY_BOUND_BIT, settings.max_cubemap_resource_size);
  layout->PushDescriptorBinding(11, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_COMPUTE_BIT, 0);
  layout->Initialize();
  auto descriptor_set = std::make_shared<DescriptorSet>(layout);
  TextureStorage::BindTexture2DToDescriptorSet(descriptor_set, 9);
  TextureStorage::BindCubemapToDescriptorSet(descriptor_set, 10);

  constexpr size_t output_count = 4;
  VkBufferCreateInfo buffer_info{};
  buffer_info.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
  buffer_info.size = output_count * sizeof(glm::vec4);
  buffer_info.usage =
      VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_SRC_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT;
  buffer_info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
  VmaAllocationCreateInfo allocation_info{};
  allocation_info.usage = VMA_MEMORY_USAGE_AUTO_PREFER_DEVICE;
  auto output = std::make_shared<Buffer>(buffer_info, allocation_info);
  output->Upload(std::array<glm::vec4, output_count>{});
  descriptor_set->UpdateBufferDescriptorBinding(11, output);

  const auto shader_path = std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / "EvoEngine_Tests" / "Resources" /
                           "Shaders" / "Compute" / "BindlessTextureArrayProbe.slang";
  auto shader = std::make_shared<Shader>();
  ASSERT_TRUE(shader->TryCompile(ShaderType::Compute, Platform::GetShaderGlobalDefines(), shader_path));
  auto pipeline = std::make_shared<ComputePipeline>();
  pipeline->compute_shader = shader;
  pipeline->descriptor_set_layouts.emplace_back(layout);
  pipeline->push_constant_ranges.emplace_back(VkPushConstantRange{VK_SHADER_STAGE_COMPUTE_BIT, 0, 16});
  pipeline->Initialize();
  ASSERT_TRUE(pipeline->Initialized());

  struct ProbePushConstant {
    glm::uvec2 texture_2d_indices;
    glm::uvec2 cubemap_indices;
  };
  const ProbePushConstant push_constant{{texture_2d_a.GetTextureStorageIndex(), texture_2d_b.GetTextureStorageIndex()},
                                        {cubemap_a.GetTextureStorageIndex(), cubemap_b.GetTextureStorageIndex()}};
  Platform::ImmediateSubmit([&](const VkCommandBuffer command_buffer) {
    pipeline->Bind(command_buffer);
    pipeline->BindDescriptorSet(command_buffer, 0, descriptor_set->GetVkDescriptorSet());
    pipeline->PushConstant(command_buffer, 0, push_constant);
    pipeline->Dispatch(command_buffer, 1);
    Platform::EverythingBarrier(command_buffer);
  });

  std::array<glm::vec4, output_count> values{};
  output->Download(values);
  const std::array<glm::vec4, output_count> expected = {
      glm::vec4(1.0f, 0.0f, 0.0f, 1.0f), glm::vec4(0.0f, 1.0f, 0.0f, 1.0f), glm::vec4(0.0f, 0.0f, 1.0f, 1.0f),
      glm::vec4(1.0f, 1.0f, 0.0f, 1.0f)};
  for (size_t value_index = 0; value_index < output_count; ++value_index) {
    for (int channel = 0; channel < 4; ++channel) {
      EXPECT_NEAR(values[value_index][channel], expected[value_index][channel], 1.0e-5f)
          << value_index << ":" << channel;
    }
  }
}

TEST(GpuService, SampledViewInspectionAndDualBindingShareOneImageAllocation) {
  ScopedGpuPlatform platform;

  Texture2D source;
  Texture2DSamplerSettings nearest_sampler;
  nearest_sampler.mag_filter = VK_FILTER_NEAREST;
  nearest_sampler.min_filter = VK_FILTER_NEAREST;
  nearest_sampler.address_mode_u = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
  nearest_sampler.address_mode_v = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
  source.SetSamplerSettings(nearest_sampler);
  const auto upload = source.RefTexture2DStorage().SetDataAsync(
      {glm::vec4(1.0f, 0.0f, 0.0f, 1.0f), glm::vec4(0.0f, 0.0f, 1.0f, 1.0f)}, {2, 1});
  ASSERT_TRUE(upload.Valid());
  Platform::GetGpuService().Wait(upload);
  TextureStorage::DeviceSync();

  Texture2D alternate_view;
  Texture2DSamplerSettings linear_sampler;
  linear_sampler.address_mode_u = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
  linear_sampler.address_mode_v = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
  ASSERT_TRUE(alternate_view.ShareGpuImage(source, false, linear_sampler));

  const auto inspections = TextureStorage::InspectTexture2DSampledViews();
  const auto source_index = source.GetTextureStorageIndex();
  const auto alternate_index = alternate_view.GetTextureStorageIndex();
  ASSERT_LT(source_index, inspections.size());
  ASSERT_LT(alternate_index, inspections.size());
  const auto& source_inspection = inspections[source_index];
  const auto& alternate_inspection = inspections[alternate_index];
  EXPECT_EQ(source_inspection.storage_handle, static_cast<int>(source_index));
  EXPECT_EQ(alternate_inspection.storage_handle, static_cast<int>(alternate_index));
  EXPECT_EQ(source_inspection.asset_type, SampledTextureAssetType::Texture2D);
  EXPECT_EQ(alternate_inspection.asset_type, SampledTextureAssetType::Texture2D);
  EXPECT_TRUE(source_inspection.ready);
  EXPECT_TRUE(alternate_inspection.ready);
  EXPECT_TRUE(source_inspection.bindless_registered);
  EXPECT_TRUE(alternate_inspection.bindless_registered);
  EXPECT_TRUE(source_inspection.traditional_binding_compatible);
  EXPECT_TRUE(alternate_inspection.traditional_binding_compatible);
  EXPECT_NE(source_inspection.image_allocation, VK_NULL_HANDLE);
  EXPECT_EQ(source_inspection.image_allocation, alternate_inspection.image_allocation);
  EXPECT_EQ(source_inspection.image, alternate_inspection.image);
  EXPECT_NE(source_inspection.image_view, alternate_inspection.image_view);
  EXPECT_NE(source_inspection.sampler, alternate_inspection.sampler);

  const GraphicsInitializationSettings settings;
  auto bindless_layout = std::make_shared<DescriptorSetLayout>();
  bindless_layout->PushDescriptorBinding(9, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, VK_SHADER_STAGE_COMPUTE_BIT,
                                         VK_DESCRIPTOR_BINDING_PARTIALLY_BOUND_BIT,
                                         settings.max_texture_2d_resource_size);
  bindless_layout->PushDescriptorBinding(11, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_COMPUTE_BIT, 0);
  bindless_layout->Initialize();
  auto bindless_set = std::make_shared<DescriptorSet>(bindless_layout);
  TextureStorage::BindTexture2DToDescriptorSet(bindless_set, 9);

  auto fixed_layout = std::make_shared<DescriptorSetLayout>();
  fixed_layout->PushDescriptorBinding(0, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, VK_SHADER_STAGE_COMPUTE_BIT, 0);
  fixed_layout->PushDescriptorBinding(1, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, VK_SHADER_STAGE_COMPUTE_BIT, 0);
  fixed_layout->Initialize();
  auto fixed_set = std::make_shared<DescriptorSet>(fixed_layout);
  VkDescriptorImageInfo source_info{};
  source_info.imageLayout = source.GetLayout();
  source_info.imageView = source.GetVkImageView();
  source_info.sampler = source.GetVkSampler();
  fixed_set->UpdateImageDescriptorBinding(0, source_info);
  VkDescriptorImageInfo alternate_info{};
  alternate_info.imageLayout = alternate_view.GetLayout();
  alternate_info.imageView = alternate_view.GetVkImageView();
  alternate_info.sampler = alternate_view.GetVkSampler();
  fixed_set->UpdateImageDescriptorBinding(1, alternate_info);

  constexpr size_t output_count = 4;
  VkBufferCreateInfo buffer_info{};
  buffer_info.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
  buffer_info.size = output_count * sizeof(glm::vec4);
  buffer_info.usage =
      VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_SRC_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT;
  buffer_info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
  VmaAllocationCreateInfo allocation_info{};
  allocation_info.usage = VMA_MEMORY_USAGE_AUTO_PREFER_DEVICE;
  auto output = std::make_shared<Buffer>(buffer_info, allocation_info);
  output->Upload(std::array<glm::vec4, output_count>{});
  bindless_set->UpdateBufferDescriptorBinding(11, output);

  const auto shader_path = std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / "EvoEngine_Tests" / "Resources" /
                           "Shaders" / "Compute" / "TextureDualAccessProbe.slang";
  auto shader = std::make_shared<Shader>();
  ASSERT_TRUE(shader->TryCompile(ShaderType::Compute, Platform::GetShaderGlobalDefines(), shader_path));
  auto pipeline = std::make_shared<ComputePipeline>();
  pipeline->compute_shader = shader;
  pipeline->descriptor_set_layouts = {bindless_layout, fixed_layout};
  pipeline->push_constant_ranges.emplace_back(VkPushConstantRange{VK_SHADER_STAGE_COMPUTE_BIT, 0, 8});
  pipeline->Initialize();
  ASSERT_TRUE(pipeline->Initialized());

  const glm::uvec2 texture_indices(source_index, alternate_index);
  Platform::ImmediateSubmit([&](const VkCommandBuffer command_buffer) {
    pipeline->Bind(command_buffer);
    pipeline->BindDescriptorSet(command_buffer, 0, bindless_set->GetVkDescriptorSet());
    pipeline->BindDescriptorSet(command_buffer, 1, fixed_set->GetVkDescriptorSet());
    pipeline->PushConstant(command_buffer, 0, texture_indices);
    pipeline->Dispatch(command_buffer, 1);
    Platform::EverythingBarrier(command_buffer);
  });

  std::array<glm::vec4, output_count> values{};
  output->Download(values);
  for (int channel = 0; channel < 4; ++channel) {
    EXPECT_NEAR(values[0][channel], values[1][channel], 1.0e-5f);
    EXPECT_NEAR(values[2][channel], values[3][channel], 1.0e-5f);
  }
  EXPECT_GT(glm::distance(values[0], values[2]), 0.1f);
}

TEST(GpuService, SharedBindlessGltfTextureAccessPreservesIndicesFallbacksAndGradients) {
  ScopedGpuPlatform platform;

  Texture2D red;
  Texture2D green;
  Texture2D pending;
  const auto red_upload = red.RefTexture2DStorage().SetDataAsync({glm::vec4(1.0f, 0.0f, 0.0f, 1.0f)}, {1, 1});
  const auto green_upload = green.RefTexture2DStorage().SetDataAsync({glm::vec4(0.0f, 1.0f, 0.0f, 1.0f)}, {1, 1});
  ASSERT_TRUE(red_upload.Valid());
  ASSERT_TRUE(green_upload.Valid());
  Platform::GetGpuService().Wait(red_upload);
  Platform::GetGpuService().Wait(green_upload);
  TextureStorage::DeviceSync();

  std::array<GltfShadeMaterial, 4> materials{};
  materials[0].pbr_base_color_texture = 1;
  materials[1].pbr_base_color_texture = 2;
  materials[3].pbr_base_color_texture = 3;
  std::array<GltfTextureInfo, 4> texture_infos{};
  texture_infos[1].index = static_cast<int32_t>(red.GetTextureStorageIndex());
  texture_infos[1].tex_coord = 1;
  texture_infos[1].uv_transform = glm::mat3x2(glm::vec2(2.0f, 0.0f), glm::vec2(0.0f, 3.0f), glm::vec2(0.25f, 0.5f));
  texture_infos[2].index = static_cast<int32_t>(green.GetTextureStorageIndex());
  texture_infos[2].tex_coord = 0;
  texture_infos[3].index = static_cast<int32_t>(pending.GetTextureStorageIndex());

  constexpr size_t output_count = 6;
  const auto create_storage_buffer = [](const VkDeviceSize size) {
    VkBufferCreateInfo buffer_info{};
    buffer_info.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
    buffer_info.size = size;
    buffer_info.usage =
        VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_SRC_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT;
    buffer_info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
    VmaAllocationCreateInfo allocation_info{};
    allocation_info.usage = VMA_MEMORY_USAGE_AUTO_PREFER_DEVICE;
    return std::make_shared<Buffer>(buffer_info, allocation_info);
  };
  auto material_buffer = create_storage_buffer(sizeof(materials));
  auto texture_info_buffer = create_storage_buffer(sizeof(texture_infos));
  auto output_buffer = create_storage_buffer(output_count * sizeof(glm::vec4));
  material_buffer->Upload(materials);
  texture_info_buffer->Upload(texture_infos);
  output_buffer->Upload(std::array<glm::vec4, output_count>{});

  const GraphicsInitializationSettings settings;
  auto layout = std::make_shared<DescriptorSetLayout>();
  layout->PushDescriptorBinding(9, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, VK_SHADER_STAGE_COMPUTE_BIT,
                                VK_DESCRIPTOR_BINDING_PARTIALLY_BOUND_BIT, settings.max_texture_2d_resource_size);
  layout->PushDescriptorBinding(11, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_COMPUTE_BIT, 0);
  layout->PushDescriptorBinding(12, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_COMPUTE_BIT, 0);
  layout->PushDescriptorBinding(13, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_COMPUTE_BIT, 0);
  layout->Initialize();
  auto descriptor_set = std::make_shared<DescriptorSet>(layout);
  TextureStorage::BindTexture2DToDescriptorSet(descriptor_set, 9);
  descriptor_set->UpdateBufferDescriptorBinding(11, material_buffer);
  descriptor_set->UpdateBufferDescriptorBinding(12, texture_info_buffer);
  descriptor_set->UpdateBufferDescriptorBinding(13, output_buffer);

  const auto shader_root =
      std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / "EvoEngine_SDK" / "Internals" / "DefaultResources" / "Shaders";
  Shader::RegisterShaderIncludePath(shader_root / "Modules");
  const auto shader_path = std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / "EvoEngine_Tests" / "Resources" /
                           "Shaders" / "Compute" / "GltfBindlessTextureAccessProbe.slang";
  auto shader = std::make_shared<Shader>();
  ASSERT_TRUE(shader->TryCompile(ShaderType::Compute, Platform::GetShaderGlobalDefines(), shader_path));
  const auto fragment_shader_path = std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / "EvoEngine_Tests" / "Resources" /
                                    "Shaders" / "Fragment" / "GltfBindlessRasterAccessProbe.slang";
  auto fragment_shader = std::make_shared<Shader>();
  ASSERT_TRUE(
      fragment_shader->TryCompile(ShaderType::Fragment, Platform::GetShaderGlobalDefines(), fragment_shader_path));
  auto pipeline = std::make_shared<ComputePipeline>();
  pipeline->compute_shader = shader;
  pipeline->descriptor_set_layouts.emplace_back(layout);
  pipeline->push_constant_ranges.emplace_back(VkPushConstantRange{VK_SHADER_STAGE_COMPUTE_BIT, 0, 16});
  pipeline->Initialize();
  ASSERT_TRUE(pipeline->Initialized());

  const glm::uvec4 material_indices(0, 1, 2, 3);
  Platform::ImmediateSubmit([&](const VkCommandBuffer command_buffer) {
    pipeline->Bind(command_buffer);
    pipeline->BindDescriptorSet(command_buffer, 0, descriptor_set->GetVkDescriptorSet());
    pipeline->PushConstant(command_buffer, 0, material_indices);
    pipeline->Dispatch(command_buffer, 1);
    Platform::EverythingBarrier(command_buffer);
  });

  std::array<glm::vec4, output_count> values{};
  output_buffer->Download(values);
  EXPECT_NEAR(values[0].r, 1.0f, 1.0e-5f);
  EXPECT_NEAR(values[0].g, 0.0f, 1.0e-5f);
  EXPECT_NEAR(values[1].r, 0.0f, 1.0e-5f);
  EXPECT_NEAR(values[1].g, 1.0f, 1.0e-5f);
  EXPECT_EQ(values[2], glm::vec4(0.125f, 0.25f, 0.5f, 1.0f));
  EXPECT_EQ(values[3], glm::vec4(1.0f));
  EXPECT_NEAR(values[4].x, 0.22f, 1.0e-5f);
  EXPECT_NEAR(values[4].y, 0.36f, 1.0e-5f);
  EXPECT_NEAR(values[4].z, 0.42f, 1.0e-5f);
  EXPECT_NEAR(values[4].w, 0.66f, 1.0e-5f);
  EXPECT_NEAR(values[5].x, 0.85f, 1.0e-5f);
  EXPECT_NEAR(values[5].y, 1.7f, 1.0e-5f);
  EXPECT_NEAR(values[5].z, 0.3f, 1.0e-5f);
  EXPECT_NEAR(values[5].w, 0.4f, 1.0e-5f);
}

TEST(GpuService, TextureStorageSlotsRemainStableAndRevisionedAcrossLifecycleStress) {
  ScopedGpuPlatform platform;

  const auto initial_texture_registration_revision = TextureStorage::GetTexture2DRegistrationRevision();
  const auto initial_cubemap_registration_revision = TextureStorage::GetCubemapRegistrationRevision();
  auto texture_a = std::make_unique<Texture2D>();
  auto texture_retired = std::make_unique<Texture2D>();
  auto texture_tail = std::make_unique<Texture2D>();
  const auto texture_a_index = texture_a->GetTextureStorageIndex();
  const auto texture_retired_index = texture_retired->GetTextureStorageIndex();
  const auto texture_tail_index = texture_tail->GetTextureStorageIndex();
  const auto texture_revision_before_retire = TextureStorage::GetTexture2DDescriptorRevision();
  EXPECT_EQ(TextureStorage::GetTexture2DRegistrationRevision(), initial_texture_registration_revision + 3u);

  texture_retired.reset();
  EXPECT_EQ(TextureStorage::GetTexture2DRegistrationRevision(), initial_texture_registration_revision + 4u);
  auto texture_inspections = TextureStorage::InspectTexture2DSampledViews();
  ASSERT_LT(texture_retired_index, texture_inspections.size());
  EXPECT_EQ(texture_inspections[texture_retired_index].slot_state, SampledViewSlotState::Retiring);
  EXPECT_FALSE(texture_inspections[texture_retired_index].bindless_registered);
  EXPECT_EQ(texture_a->GetTextureStorageIndex(), texture_a_index);
  EXPECT_EQ(texture_tail->GetTextureStorageIndex(), texture_tail_index);
  EXPECT_GT(TextureStorage::GetTexture2DDescriptorRevision(), texture_revision_before_retire);

  TextureStorage::DeviceSync();
  texture_inspections = TextureStorage::InspectTexture2DSampledViews();
  EXPECT_EQ(texture_inspections[texture_retired_index].slot_state, SampledViewSlotState::Reusable);
  EXPECT_EQ(texture_inspections[texture_retired_index].storage_handle, -1);
  EXPECT_EQ(texture_tail->GetTextureStorageIndex(), texture_tail_index);

  auto texture_replacement = std::make_unique<Texture2D>();
  EXPECT_EQ(TextureStorage::GetTexture2DRegistrationRevision(), initial_texture_registration_revision + 5u);
  EXPECT_EQ(texture_replacement->GetTextureStorageIndex(), texture_retired_index);
  EXPECT_EQ(texture_a->GetTextureStorageIndex(), texture_a_index);
  EXPECT_EQ(texture_tail->GetTextureStorageIndex(), texture_tail_index);

  auto cubemap_a = std::make_unique<Cubemap>();
  auto cubemap_retired = std::make_unique<Cubemap>();
  auto cubemap_tail = std::make_unique<Cubemap>();
  EXPECT_EQ(TextureStorage::GetCubemapRegistrationRevision(), initial_cubemap_registration_revision + 3u);
  const auto cubemap_a_index = cubemap_a->GetTextureStorageIndex();
  const auto cubemap_retired_index = cubemap_retired->GetTextureStorageIndex();
  const auto cubemap_tail_index = cubemap_tail->GetTextureStorageIndex();
  cubemap_retired.reset();
  EXPECT_EQ(TextureStorage::GetCubemapRegistrationRevision(), initial_cubemap_registration_revision + 4u);
  auto cubemap_inspections = TextureStorage::InspectCubemapSampledViews();
  ASSERT_LT(cubemap_retired_index, cubemap_inspections.size());
  EXPECT_EQ(cubemap_inspections[cubemap_retired_index].slot_state, SampledViewSlotState::Retiring);
  TextureStorage::DeviceSync();
  cubemap_inspections = TextureStorage::InspectCubemapSampledViews();
  EXPECT_EQ(cubemap_inspections[cubemap_retired_index].slot_state, SampledViewSlotState::Reusable);
  auto cubemap_replacement = std::make_unique<Cubemap>();
  EXPECT_EQ(TextureStorage::GetCubemapRegistrationRevision(), initial_cubemap_registration_revision + 5u);
  EXPECT_EQ(cubemap_replacement->GetTextureStorageIndex(), cubemap_retired_index);
  EXPECT_EQ(cubemap_a->GetTextureStorageIndex(), cubemap_a_index);
  EXPECT_EQ(cubemap_tail->GetTextureStorageIndex(), cubemap_tail_index);

  Texture2D pending_texture;
  const auto pending_upload =
      pending_texture.RefTexture2DStorage().SetDataAsync({glm::vec4(0.25f, 0.5f, 0.75f, 1.0f)}, glm::uvec2(1));
  ASSERT_TRUE(pending_upload.Valid());
  texture_inspections = TextureStorage::InspectTexture2DSampledViews();
  ASSERT_LT(pending_texture.GetTextureStorageIndex(), texture_inspections.size());
  EXPECT_EQ(texture_inspections[pending_texture.GetTextureStorageIndex()].slot_state,
            SampledViewSlotState::AllocatedPending);
  EXPECT_FALSE(texture_inspections[pending_texture.GetTextureStorageIndex()].ready);
  VkDescriptorImageInfo pending_info{};
  EXPECT_FALSE(
      TextureStorage::TryGetTexture2DDescriptorImageInfo(pending_texture.GetTextureStorageIndex(), pending_info));

  const GraphicsInitializationSettings settings;
  auto layout = std::make_shared<DescriptorSetLayout>();
  layout->PushDescriptorBinding(9, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, VK_SHADER_STAGE_COMPUTE_BIT,
                                VK_DESCRIPTOR_BINDING_PARTIALLY_BOUND_BIT, settings.max_texture_2d_resource_size);
  layout->PushDescriptorBinding(10, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, VK_SHADER_STAGE_COMPUTE_BIT,
                                VK_DESCRIPTOR_BINDING_PARTIALLY_BOUND_BIT, settings.max_cubemap_resource_size);
  layout->Initialize();
  auto descriptor_set = std::make_shared<DescriptorSet>(layout);
  TextureStorage::ResetDescriptorUpdateStats();
  uint64_t applied_revision = 0;
  const auto current_revision = TextureStorage::GetTexture2DDescriptorRevision();
  if (applied_revision != current_revision) {
    EXPECT_EQ(TextureStorage::BindTexture2DToDescriptorSet(descriptor_set, 9), settings.max_texture_2d_resource_size);
    applied_revision = current_revision;
  }
  EXPECT_EQ(TextureStorage::BindCubemapToDescriptorSet(descriptor_set, 10), settings.max_cubemap_resource_size);
  const auto first_update_stats = TextureStorage::GetDescriptorUpdateStats();
  EXPECT_EQ(first_update_stats.texture_2d_full_rebuilds, 1u);
  EXPECT_EQ(first_update_stats.texture_2d_descriptors_written, settings.max_texture_2d_resource_size);
  EXPECT_EQ(first_update_stats.cubemap_full_rebuilds, 1u);
  EXPECT_EQ(first_update_stats.cubemap_descriptors_written, settings.max_cubemap_resource_size);
  const auto texture_diagnostics = TextureStorage::GetTexture2DArrayDiagnostics();
  EXPECT_EQ(texture_diagnostics.capacity, settings.max_texture_2d_resource_size);
  EXPECT_GE(texture_diagnostics.occupancy, 4u);
  EXPECT_GE(texture_diagnostics.high_water_mark, texture_diagnostics.occupancy);
  EXPECT_GE(texture_diagnostics.pending_count, 1u);
  EXPECT_EQ(texture_diagnostics.full_rebuilds, 1u);
  EXPECT_EQ(texture_diagnostics.descriptors_written, settings.max_texture_2d_resource_size);
  EXPECT_EQ(texture_diagnostics.descriptor_metadata_bytes_per_mirror,
            static_cast<uint64_t>(settings.max_texture_2d_resource_size) * sizeof(VkDescriptorImageInfo));
  const auto cubemap_diagnostics = TextureStorage::GetCubemapArrayDiagnostics();
  EXPECT_EQ(cubemap_diagnostics.capacity, settings.max_cubemap_resource_size);
  EXPECT_GE(cubemap_diagnostics.occupancy, 3u);
  EXPECT_GE(cubemap_diagnostics.high_water_mark, cubemap_diagnostics.occupancy);
  EVOENGINE_LOG("EVOENGINE_BINDLESS_SMALL_SCENE texture_2d_occupancy=" + std::to_string(texture_diagnostics.occupancy) +
                " cubemap_occupancy=" + std::to_string(cubemap_diagnostics.occupancy) + " descriptor_update_cpu_ms=" +
                std::to_string(static_cast<double>(texture_diagnostics.descriptor_update_cpu_nanoseconds +
                                                   cubemap_diagnostics.descriptor_update_cpu_nanoseconds) /
                               1.0e6) +
                " descriptors_written=" +
                std::to_string(texture_diagnostics.descriptors_written + cubemap_diagnostics.descriptors_written) +
                " descriptor_metadata_bytes_per_mirror=" +
                std::to_string(texture_diagnostics.descriptor_metadata_bytes_per_mirror +
                               cubemap_diagnostics.descriptor_metadata_bytes_per_mirror))
  if (applied_revision != TextureStorage::GetTexture2DDescriptorRevision()) {
    TextureStorage::BindTexture2DToDescriptorSet(descriptor_set, 9);
  }
  const auto unchanged_update_stats = TextureStorage::GetDescriptorUpdateStats();
  EXPECT_EQ(unchanged_update_stats.texture_2d_full_rebuilds, first_update_stats.texture_2d_full_rebuilds);
  EXPECT_EQ(unchanged_update_stats.texture_2d_descriptors_written, first_update_stats.texture_2d_descriptors_written);

  Platform::GetGpuService().Wait(pending_upload);
  TextureStorage::DeviceSync();
  texture_inspections = TextureStorage::InspectTexture2DSampledViews();
  EXPECT_EQ(texture_inspections[pending_texture.GetTextureStorageIndex()].slot_state, SampledViewSlotState::Ready);
  EXPECT_TRUE(texture_inspections[pending_texture.GetTextureStorageIndex()].ready);
  EXPECT_GT(TextureStorage::GetTexture2DDescriptorRevision(), current_revision);
}

TEST(GpuService, GltfRayTracingNumericalProbeMatchesAnalyticValues) {
  ScopedGpuPlatform platform;
  const auto shader_root =
      std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / "EvoEngine_SDK" / "Internals" / "DefaultResources" / "Shaders";
  const auto probe_path = std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / "EvoEngine_Tests" / "Resources" /
                          "Shaders" / "Compute" / "GltfRayTracingNumericalProbe.slang";
  Shader::RegisterShaderIncludePath(shader_root / "Modules");

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
  ASSERT_TRUE(shader->TryCompile(ShaderType::Compute, Platform::GetShaderGlobalDefines(), probe_path));
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

TEST(GpuService, EcoSysLabComputeMigrationMatchesDeterministicContracts) {
  ScopedGpuPlatform platform;
  const auto shader_root = std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / "EvoEngine_Packages" / "EcoSysLab" /
                           "Internals" / "EcoSysLabResources" / "Shaders";
  Shader::RegisterShaderIncludePath(shader_root / "Modules");

  constexpr uint32_t segment_count = 4;
  constexpr size_t segment_stride = 480;
  constexpr size_t segment_group_offset = 316;
  constexpr size_t particle_stride = 96;
  constexpr size_t particle_position_offset = 16;
  constexpr size_t particle_last_position_offset = 32;
  constexpr size_t particle_velocity_offset = 48;
  constexpr size_t particle_acceleration_offset = 64;
  constexpr size_t segment_data_stride = 48;
  constexpr size_t segment_connection_handles_stride = 256;
  constexpr uint32_t cell_count = 2u << 15u;

  auto make_buffer = [](const size_t size) {
    VkBufferCreateInfo info{};
    info.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
    info.size = size;
    info.usage =
        VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_SRC_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT;
    info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
    VmaAllocationCreateInfo allocation{};
    allocation.usage = VMA_MEMORY_USAGE_AUTO_PREFER_DEVICE;
    return std::make_shared<Buffer>(info, allocation);
  };
  const auto write = [](std::vector<std::byte>& bytes, const size_t offset, const auto& value) {
    ASSERT_LE(offset + sizeof(value), bytes.size());
    std::memcpy(bytes.data() + offset, &value, sizeof(value));
  };
  const auto read_int32 = [](const std::vector<std::byte>& bytes, const size_t offset) {
    int32_t value{};
    EXPECT_LE(offset + sizeof(value), bytes.size());
    std::memcpy(&value, bytes.data() + offset, sizeof(value));
    return value;
  };
  const auto read_float = [](const std::vector<std::byte>& bytes, const size_t offset) {
    float value{};
    EXPECT_LE(offset + sizeof(value), bytes.size());
    std::memcpy(&value, bytes.data() + offset, sizeof(value));
    return value;
  };

  const std::array<size_t, 13> buffer_sizes = {
      48,
      16,
      segment_stride * segment_count,
      144,
      segment_data_stride * segment_count,
      16 * segment_count,
      16 * cell_count,
      384,
      160 * 4,
      160,
      particle_stride * segment_count,
      particle_stride * segment_count,
      segment_connection_handles_stride * segment_count,
  };
  std::array<std::shared_ptr<Buffer>, 13> buffers;
  auto strands_layout = std::make_shared<DescriptorSetLayout>();
  for (uint32_t binding = 0; binding < buffers.size(); ++binding) {
    strands_layout->PushDescriptorBinding(binding, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_COMPUTE_BIT, 0);
    buffers[binding] = make_buffer(buffer_sizes[binding]);
    std::vector<std::byte> zero(buffer_sizes[binding]);
    buffers[binding]->UploadData(zero.size(), zero.data());
  }
  strands_layout->Initialize();
  auto strands_set = std::make_shared<DescriptorSet>(strands_layout);
  for (uint32_t binding = 0; binding < buffers.size(); ++binding)
    strands_set->UpdateBufferDescriptorBinding(binding, buffers[binding]);

  auto make_auxiliary_set = [&](const uint32_t binding_count, const std::vector<size_t>& sizes,
                                std::vector<std::shared_ptr<Buffer>>& auxiliary_buffers) {
    auto layout = std::make_shared<DescriptorSetLayout>();
    for (uint32_t binding = 0; binding < binding_count; ++binding) {
      layout->PushDescriptorBinding(binding, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_COMPUTE_BIT, 0);
      auxiliary_buffers.emplace_back(make_buffer(sizes[binding]));
      std::vector<std::byte> zero(sizes[binding]);
      auxiliary_buffers.back()->UploadData(zero.size(), zero.data());
    }
    layout->Initialize();
    auto set = std::make_shared<DescriptorSet>(layout);
    for (uint32_t binding = 0; binding < binding_count; ++binding)
      set->UpdateBufferDescriptorBinding(binding, auxiliary_buffers[binding]);
    return std::pair{layout, set};
  };

  auto make_pipeline = [&](const std::filesystem::path& relative_path,
                           std::vector<std::shared_ptr<DescriptorSetLayout>> layouts, const uint32_t push_size) {
    auto shader = std::make_shared<Shader>();
    const auto path = shader_root / relative_path;
    EXPECT_TRUE(shader->TryCompile(ShaderType::Compute, Platform::GetShaderGlobalDefines(), path)) << path.string();
    auto pipeline = std::make_shared<ComputePipeline>();
    pipeline->compute_shader = shader;
    pipeline->descriptor_set_layouts = std::move(layouts);
    pipeline->push_constant_ranges.emplace_back(VkPushConstantRange{VK_SHADER_STAGE_COMPUTE_BIT, 0, push_size});
    pipeline->Initialize();
    EXPECT_TRUE(pipeline->Initialized()) << path.string();
    return pipeline;
  };
  const auto dispatch = [&](const std::shared_ptr<ComputePipeline>& pipeline, const auto& constants,
                            const std::shared_ptr<DescriptorSet>& auxiliary_set = nullptr) {
    Platform::ImmediateSubmit([&](const VkCommandBuffer command_buffer) {
      pipeline->Bind(command_buffer);
      pipeline->BindDescriptorSet(command_buffer, 0, strands_set->GetVkDescriptorSet());
      if (auxiliary_set)
        pipeline->BindDescriptorSet(command_buffer, 1, auxiliary_set->GetVkDescriptorSet());
      pipeline->PushConstant(command_buffer, 0, constants);
      pipeline->Dispatch(command_buffer, 1);
      Platform::EverythingBarrier(command_buffer);
    });
  };

  struct SegmentCountConstants {
    uint32_t segment_size;
  };
  struct PartitionConstants {
    uint32_t segment_size;
    float cell_size;
  };
  struct SortConstants {
    uint32_t segment_size;
    uint32_t group_size;
  };
  struct HashedGridElement {
    uint32_t cell_id;
    uint32_t segment_handle;
    uint32_t padding0;
    uint32_t padding1;
  };
  struct CellRange {
    uint32_t start_index;
    uint32_t end_index;
    uint32_t padding0;
    uint32_t padding1;
  };

  std::vector<std::byte> segments(segment_stride * segment_count);
  std::vector<std::byte> particle0s(particle_stride * segment_count);
  std::vector<std::byte> particle1s(particle_stride * segment_count);
  const auto upload_strands = [&] {
    buffers[2]->UploadData(segments.size(), segments.data());
    buffers[10]->UploadData(particle0s.size(), particle0s.data());
    buffers[11]->UploadData(particle1s.size(), particle1s.data());
  };
  const auto download_strands = [&] {
    buffers[2]->DownloadData(segments.size(), segments.data());
    buffers[10]->DownloadData(particle0s.size(), particle0s.data());
    buffers[11]->DownloadData(particle1s.size(), particle1s.data());
  };
  const std::array<int32_t, segment_count> initial_groups = {3, 1, 2, 0};
  for (uint32_t index = 0; index < segment_count; ++index) {
    write(segments, index * segment_stride + segment_group_offset, initial_groups[index]);
    const glm::vec3 position(static_cast<float>(index), 0.0f, 0.0f);
    write(particle0s, index * particle_stride + particle_position_offset, position);
    write(particle1s, index * particle_stride + particle_position_offset, position);
  }
  upload_strands();

  const auto reset_pipeline = make_pipeline("Compute/DynamicStrands/Grouping/Reset.slang", {strands_layout}, 4);
  dispatch(reset_pipeline, SegmentCountConstants{segment_count});
  buffers[2]->DownloadData(segments.size(), segments.data());
  for (uint32_t index = 0; index < segment_count; ++index)
    EXPECT_EQ(read_int32(segments, index * segment_stride + segment_group_offset), index);

  buffers[2]->UploadData(segments.size(), segments.data());
  std::vector<std::byte> segment_pairs(144);
  write(segment_pairs, 0, int32_t{0});
  write(segment_pairs, 4, int32_t{1});
  write(segment_pairs, 8, 1.0f);
  write(segment_pairs, 12, 1.0f);
  buffers[3]->UploadData(segment_pairs.size(), segment_pairs.data());
  std::vector<std::byte> segment_data(segment_data_stride * segment_count);
  std::vector<std::byte> segment_connection_handles(segment_connection_handles_stride * segment_count, std::byte{0xff});
  write(segment_connection_handles, 0, int32_t{0});
  write(segment_connection_handles, segment_connection_handles_stride, int32_t{0});
  buffers[4]->UploadData(segment_data.size(), segment_data.data());
  buffers[12]->UploadData(segment_connection_handles.size(), segment_connection_handles.data());

  std::vector<std::shared_ptr<Buffer>> grouping_buffers;
  auto [grouping_layout, grouping_set] =
      make_auxiliary_set(2, {sizeof(int32_t) * segment_count, sizeof(uint32_t)}, grouping_buffers);
  const auto dynamic_step_pipeline =
      make_pipeline("Compute/DynamicStrands/Grouping/DynamicStep.slang", {strands_layout, grouping_layout}, 4);
  dispatch(dynamic_step_pipeline, SegmentCountConstants{segment_count}, grouping_set);
  std::array<int32_t, segment_count> new_groups{};
  grouping_buffers[0]->Download(new_groups);
  EXPECT_EQ(new_groups, (std::array<int32_t, segment_count>{0, 0, 2, 3}));
  uint32_t workgroup_updated = 0;
  grouping_buffers[1]->Download(workgroup_updated);
  EXPECT_EQ(workgroup_updated, 1u);

  const auto apply_pipeline =
      make_pipeline("Compute/DynamicStrands/Grouping/Apply.slang", {strands_layout, grouping_layout}, 4);
  dispatch(apply_pipeline, SegmentCountConstants{segment_count}, grouping_set);
  buffers[2]->DownloadData(segments.size(), segments.data());
  for (uint32_t index = 0; index < segment_count; ++index)
    EXPECT_EQ(read_int32(segments, index * segment_stride + segment_group_offset), new_groups[index]);

  const auto& uniform_particles = buffers[8];
  const auto& tetrahedrons = buffers[9];
  std::vector<std::byte> uniform_particle(160 * 4);
  write(uniform_particle, 120, int32_t{2});
  uniform_particles->UploadData(uniform_particle.size(), uniform_particle.data());
  std::array<std::byte, 160> empty_tetrahedron{};
  tetrahedrons->Upload(empty_tetrahedron);
  const auto normal_pipeline =
      make_pipeline("Compute/DynamicStrands/Initialization/AlphaShapeMeshing/Normal.slang", {strands_layout}, 4);
  dispatch(normal_pipeline, SegmentCountConstants{1});
  uniform_particles->DownloadData(uniform_particle.size(), uniform_particle.data());
  EXPECT_EQ(read_int32(uniform_particle, 120), 0);
  for (size_t offset = 16; offset < 32; offset += sizeof(float))
    EXPECT_TRUE(std::isfinite(read_float(uniform_particle, offset)));

  const auto partition_pipeline =
      make_pipeline("Compute/DynamicStrands/DynamicHashedGrid/Partition.slang", {strands_layout}, 8);
  dispatch(partition_pipeline, PartitionConstants{segment_count, 1.0f});
  std::array<HashedGridElement, segment_count> hashed{};
  buffers[5]->Download(hashed);
  for (uint32_t index = 0; index < segment_count; ++index) {
    const uint32_t expected_cell = (73856093u * index) % cell_count;
    EXPECT_EQ(hashed[index].cell_id, expected_cell);
    EXPECT_EQ(hashed[index].segment_handle, index);
  }

  hashed = {{{30, 0, 0, 0}, {10, 1, 0, 0}, {20, 2, 0, 0}, {0, 3, 0, 0}}};
  buffers[5]->Upload(hashed);
  const auto sort_pipeline =
      make_pipeline("Compute/DynamicStrands/DynamicHashedGrid/Sort/LocalMergeSort.slang", {strands_layout}, 8);
  dispatch(sort_pipeline,
           SortConstants{segment_count, Platform::GetInstance().GetCapabilities().compute_work_group_invocations});
  buffers[5]->Download(hashed);
  EXPECT_EQ(
      (std::array<uint32_t, segment_count>{hashed[0].cell_id, hashed[1].cell_id, hashed[2].cell_id, hashed[3].cell_id}),
      (std::array<uint32_t, segment_count>{0, 10, 20, 30}));

  hashed = {{{1, 0, 0, 0}, {1, 1, 0, 0}, {2, 2, 0, 0}, {2, 3, 0, 0}}};
  buffers[5]->Upload(hashed);
  std::vector<CellRange> cell_ranges(cell_count, CellRange{0xffffffffu, 0xffffffffu, 0xffffffffu, 0xffffffffu});
  buffers[6]->UploadVector(cell_ranges);
  const auto offset_pipeline =
      make_pipeline("Compute/DynamicStrands/DynamicHashedGrid/Offset.slang", {strands_layout}, 4);
  dispatch(offset_pipeline, SegmentCountConstants{segment_count});
  buffers[6]->DownloadVector(cell_ranges, cell_ranges.size());
  EXPECT_EQ(cell_ranges[1].start_index, 0u);
  EXPECT_EQ(cell_ranges[1].end_index, 1u);
  EXPECT_EQ(cell_ranges[2].start_index, 2u);
  EXPECT_EQ(cell_ranges[2].end_index, 3u);

  struct GroundConstants {
    uint32_t segment_size;
    float ground_height;
    float ground_softness;
    float ground_friction;
  };
  segments.assign(segment_stride * segment_count, std::byte{});
  particle0s.assign(particle_stride * segment_count, std::byte{});
  particle1s.assign(particle_stride * segment_count, std::byte{});
  write(segments, 12, 0.0f);
  write(segments, segment_stride + 12, 1.0f);
  for (uint32_t index = 0; index < 2; ++index) {
    write(particle0s, index * particle_stride + particle_position_offset, glm::vec3(0.0f, -1.0f, 0.0f));
    write(particle1s, index * particle_stride + particle_position_offset, glm::vec3(0.0f, -1.0f, 0.0f));
  }
  upload_strands();
  const auto ground_pipeline = make_pipeline("Compute/DynamicStrands/Constraints/Position/SegmentGroundPlane.slang",
                                             {strands_layout}, sizeof(GroundConstants));
  dispatch(ground_pipeline, GroundConstants{2, 0.0f, 0.0f, 0.0f});
  download_strands();
  EXPECT_FLOAT_EQ(read_float(particle0s, particle_position_offset + sizeof(float)), -1.0f);
  EXPECT_FLOAT_EQ(read_float(particle0s, particle_stride + particle_position_offset + sizeof(float)), 0.0f);
  EXPECT_FLOAT_EQ(read_float(particle1s, particle_stride + particle_position_offset + sizeof(float)), 0.0f);

  struct PreStepConstants {
    glm::vec3 acceleration;
    uint32_t segment_size;
    float time_step;
    float inv_time_step;
  };
  struct PredictionConstants {
    uint32_t segment_size;
    float time_step;
    float inv_time_step;
  };
  for (uint32_t index = 0; index < 2; ++index) {
    const size_t base = index * segment_stride;
    write(segments, base + 48, glm::vec4(0.0f, 0.0f, 0.0f, 1.0f));
    write(segments, base + 64, glm::vec4(0.0f, 0.0f, 0.0f, 1.0f));
    write(segments, base + 108, 1.0f);
    write(segments, base + 120, 1.0f);
    write(segments, base + 156, 1.0f);
    write(segments, base + 312, 2.0f);
    write(segments, base + 320, 1.0f);
    const size_t particle_base = index * particle_stride;
    write(particle0s, particle_base + particle_position_offset, glm::vec3(0.0f, 0.0f, 0.0f));
    write(particle1s, particle_base + particle_position_offset, glm::vec3(0.0f, 1.0f, 0.0f));
    write(particle0s, particle_base + particle_velocity_offset, glm::vec3(1.0f, 0.0f, 0.0f));
    write(particle1s, particle_base + particle_velocity_offset, glm::vec3(1.0f, 0.0f, 0.0f));
  }
  upload_strands();
  const auto pre_step_pipeline =
      make_pipeline("Compute/DynamicStrands/PreStep/Segment.slang", {strands_layout}, sizeof(PreStepConstants));
  dispatch(pre_step_pipeline, PreStepConstants{glm::vec3(0.0f, -10.0f, 0.0f), 2, 0.5f, 2.0f});
  download_strands();
  EXPECT_NEAR(read_float(segments, 12), 1.0f / 3.0f, 1.0e-6f);
  EXPECT_FLOAT_EQ(read_float(segments, 320), 0.0f);
  EXPECT_FLOAT_EQ(read_float(particle0s, particle_acceleration_offset + sizeof(float)), -15.0f);

  const auto prediction_pipeline =
      make_pipeline("Compute/DynamicStrands/Prediction/Segment.slang", {strands_layout}, sizeof(PredictionConstants));
  dispatch(prediction_pipeline, PredictionConstants{2, 0.5f, 2.0f});
  download_strands();
  EXPECT_FLOAT_EQ(read_float(particle0s, particle_last_position_offset), 0.0f);
  EXPECT_FLOAT_EQ(read_float(particle0s, particle_position_offset), 0.5f);
  EXPECT_FLOAT_EQ(read_float(particle1s, particle_position_offset), 0.5f);

  struct VelocityConstants {
    glm::vec3 max_angular_velocity;
    float time_step;
    glm::vec3 max_velocity;
    float inv_time_step;
    uint32_t segment_size;
    float angular_velocity_damping;
    float velocity_damping;
  };
  const auto velocity_pipeline =
      make_pipeline("Compute/DynamicStrands/VelocityUpdate/Segment.slang", {strands_layout}, sizeof(VelocityConstants));
  dispatch(velocity_pipeline, VelocityConstants{glm::vec3(100.0f), 0.5f, glm::vec3(100.0f), 2.0f, 2, 0.0f, 0.0f});
  download_strands();
  for (const size_t offset : {80u, 84u, 88u})
    EXPECT_TRUE(std::isfinite(read_float(segments, offset))) << offset;
  for (const auto* particles : {&particle0s, &particle1s})
    for (const size_t offset : {particle_velocity_offset, particle_velocity_offset + 4, particle_velocity_offset + 8})
      EXPECT_TRUE(std::isfinite(read_float(*particles, offset))) << offset;

  struct LeafBreakingConstants {
    uint32_t leaf_size;
    uint32_t leaf_break_from_moisture;
    float leaf_break_threshold;
  };
  std::vector<std::byte> leaf(384);
  write(leaf, 12, int32_t{0});
  write(leaf, 28, 1.0f);
  write(leaf, 140, 2.0f);
  write(leaf, 220, 1.0f);
  buffers[7]->UploadData(leaf.size(), leaf.data());
  const auto breaking_pipeline =
      make_pipeline("Compute/DynamicStrands/Breaking/Leaf.slang", {strands_layout}, sizeof(LeafBreakingConstants));
  dispatch(breaking_pipeline, LeafBreakingConstants{1, 0, 0.5f});
  buffers[7]->DownloadData(leaf.size(), leaf.data());
  EXPECT_FLOAT_EQ(read_float(leaf, 28), 0.0f);

  download_strands();
  for (const size_t offset : {80u, 84u, 88u})
    write(segments, offset, 7.0f);
  for (auto* particles : {&particle0s, &particle1s})
    for (const size_t offset : {particle_velocity_offset, particle_velocity_offset + 4, particle_velocity_offset + 8})
      write(*particles, offset, 7.0f);
  upload_strands();
  const auto stop_pipeline =
      make_pipeline("Compute/DynamicStrands/Operators/SegmentStopAll.slang", {strands_layout}, 4);
  dispatch(stop_pipeline, SegmentCountConstants{1});
  download_strands();
  for (const size_t offset : {80u, 84u, 88u})
    EXPECT_FLOAT_EQ(read_float(segments, offset), 0.0f) << offset;
  for (const auto* particles : {&particle0s, &particle1s})
    for (const size_t offset : {particle_velocity_offset, particle_velocity_offset + 4, particle_velocity_offset + 8})
      EXPECT_FLOAT_EQ(read_float(*particles, offset), 0.0f) << offset;

  write(segments, 12, 0.5f);
  for (auto* particles : {&particle0s, &particle1s})
    for (const size_t offset :
         {particle_acceleration_offset, particle_acceleration_offset + 4, particle_acceleration_offset + 8})
      write(*particles, offset, 0.0f);
  upload_strands();
  std::vector<std::shared_ptr<Buffer>> force_buffers;
  auto [force_layout, force_set] = make_auxiliary_set(1, {sizeof(glm::vec4)}, force_buffers);
  force_buffers[0]->Upload(glm::vec4(2.0f, 4.0f, 6.0f, 0.0f));
  const auto force_pipeline =
      make_pipeline("Compute/DynamicStrands/Operators/ExternalForce.slang", {strands_layout, force_layout}, 4);
  dispatch(force_pipeline, SegmentCountConstants{1}, force_set);
  download_strands();
  for (const auto* particles : {&particle0s, &particle1s}) {
    EXPECT_FLOAT_EQ(read_float(*particles, particle_acceleration_offset), 1.0f);
    EXPECT_FLOAT_EQ(read_float(*particles, particle_acceleration_offset + 4), 2.0f);
    EXPECT_FLOAT_EQ(read_float(*particles, particle_acceleration_offset + 8), 3.0f);
  }

  struct FungusNodeConstants {
    uint32_t segment_size;
    float dt;
    float aw;
    float ab;
    float bw;
    float bb;
    float ycw;
    float ycb;
    float ylw;
    float pc;
    float pl;
    float k;
    float delta;
    float ll;
    float lc;
    float bo;
    float kc;
    float brw;
    float brb;
    float msr;
    float bd_offset;
    float cpb;
    float cpw;
  };
  static_assert(sizeof(FungusNodeConstants) == 92);
  segments.assign(segment_stride * segment_count, std::byte{});
  write(segments, 124, 1.0f);
  write(segments, 356, 0.2f);
  write(segments, 360, 1.0f);
  write(segments, 364, 1.0f);
  write(segments, 368, 0.0f);
  write(segments, 372, 0.0f);
  write(segments, 380, 3.0f);
  write(segments, 384, 4.0f);
  write(segments, 388, 5.0f);
  write(segments, 392, int32_t{7});
  write(segments, 400, 0.5f);
  write(segments, 404, 6.0f);
  buffers[2]->UploadData(segments.size(), segments.data());
  const auto fungus_node_pipeline = make_pipeline("Compute/DynamicStrands/Fungus/FungusDiffusion_node.slang",
                                                  {strands_layout}, sizeof(FungusNodeConstants));
  dispatch(fungus_node_pipeline,
           FungusNodeConstants{1,    0.1f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.2f,
                               0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f});
  buffers[2]->DownloadData(segments.size(), segments.data());
  for (const size_t offset : {336u, 340u, 344u, 348u, 352u, 396u}) {
    EXPECT_TRUE(std::isfinite(read_float(segments, offset))) << offset;
    EXPECT_GE(read_float(segments, offset), 0.0f) << offset;
    EXPECT_LE(read_float(segments, offset), 1.0f) << offset;
  }
  for (const size_t offset : {380u, 384u, 388u, 404u})
    EXPECT_FLOAT_EQ(read_float(segments, offset), 0.0f) << offset;
  EXPECT_EQ(read_int32(segments, 392), 0);

  struct TetrahedronFilteringConstants {
    uint32_t tetrahedron_size;
    float alpha;
    float bifurcation_alpha;
    float max_dist_squared;
    int32_t render_complex;
    float degenerate_triangle_threshold;
    float break_threshold;
    int32_t persistent_damage;
  };
  static_assert(sizeof(TetrahedronFilteringConstants) == 32);
  uniform_particle.assign(160 * 4, std::byte{});
  const std::array<glm::vec3, 4> tetrahedron_positions = {glm::vec3(0.0f, 0.0f, 0.0f), glm::vec3(1.0f, 0.0f, 0.0f),
                                                          glm::vec3(0.0f, 1.0f, 0.0f), glm::vec3(0.0f, 0.0f, 1.0f)};
  for (uint32_t index = 0; index < tetrahedron_positions.size(); ++index) {
    const size_t base = index * 160;
    write(uniform_particle, base, tetrahedron_positions[index]);
    write(uniform_particle, base + 80, int32_t{0});
  }
  uniform_particles->UploadData(uniform_particle.size(), uniform_particle.data());
  std::vector<std::byte> tetrahedron(160, std::byte{});
  for (uint32_t index = 0; index < 4; ++index) {
    write(tetrahedron, index * sizeof(int32_t), static_cast<int32_t>(index));
    write(tetrahedron, 16 + index * sizeof(int32_t), int32_t{-1});
  }
  write(tetrahedron, 88, int32_t{1});
  for (uint32_t edge = 0; edge < 6; ++edge) {
    write(tetrahedron, 96 + edge * sizeof(float), 2.0f);
    write(tetrahedron, 128 + edge * sizeof(int32_t), int32_t{-1});
  }
  write(tetrahedron, 152, int32_t{1});
  tetrahedrons->UploadData(tetrahedron.size(), tetrahedron.data());
  const auto tetrahedron_filter_pipeline =
      make_pipeline("Compute/DynamicStrands/Rendering/AlphaShapeMeshing/TetrahedronFiltering.slang", {strands_layout},
                    sizeof(TetrahedronFilteringConstants));
  dispatch(tetrahedron_filter_pipeline, TetrahedronFilteringConstants{1, 0.1f, 0.1f, 4.0f, 1, 0.0f, 0.1f, 0});
  tetrahedrons->DownloadData(tetrahedron.size(), tetrahedron.data());
  uniform_particles->DownloadData(uniform_particle.size(), uniform_particle.data());
  EXPECT_EQ(read_int32(tetrahedron, 88), 1);
  for (uint32_t index = 0; index < 4; ++index)
    EXPECT_EQ(read_int32(uniform_particle, index * 160 + 112), 1);

  const auto triangle_filter_pipeline =
      make_pipeline("Compute/DynamicStrands/Rendering/AlphaShapeMeshing/TriangleFiltering.slang", {strands_layout},
                    sizeof(TetrahedronFilteringConstants));
  dispatch(triangle_filter_pipeline, TetrahedronFilteringConstants{1, 0.1f, 0.1f, 4.0f, 1, 0.0f, 0.1f, 0});
  tetrahedrons->DownloadData(tetrahedron.size(), tetrahedron.data());
  uniform_particles->DownloadData(uniform_particle.size(), uniform_particle.data());
  for (uint32_t index = 0; index < 4; ++index) {
    EXPECT_EQ(read_int32(tetrahedron, 32 + index * sizeof(int32_t)), 1);
    EXPECT_EQ(read_int32(uniform_particle, index * 160 + 112), 0);
    for (uint32_t component = 0; component < 3; ++component)
      EXPECT_TRUE(std::isfinite(read_float(uniform_particle, index * 160 + 16 + component * sizeof(float))));
  }
}

TEST(GpuService, DirectionalShadowComparisonSamplerFiltersDepthStep) {
  ScopedGpuPlatform platform;
  constexpr auto format = Platform::Constants::shadow_map;
  const auto format_properties = Platform::GetPhysicalDeviceFormatProperties(format);
  constexpr VkFormatFeatureFlags2 required_features =
      VK_FORMAT_FEATURE_2_DEPTH_STENCIL_ATTACHMENT_BIT | VK_FORMAT_FEATURE_2_SAMPLED_IMAGE_BIT |
      VK_FORMAT_FEATURE_2_SAMPLED_IMAGE_FILTER_LINEAR_BIT | VK_FORMAT_FEATURE_2_TRANSFER_DST_BIT |
      VK_FORMAT_FEATURE_2_SAMPLED_IMAGE_DEPTH_COMPARISON_BIT;
  ASSERT_EQ(format_properties.optimalTilingFeatures & required_features, required_features)
      << "Selected devices must support the production directional comparison-PCF path.";

  VkImageCreateInfo image_info{};
  image_info.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
  image_info.imageType = VK_IMAGE_TYPE_2D;
  image_info.format = format;
  image_info.extent = {2, 1, 1};
  image_info.mipLevels = 1;
  image_info.arrayLayers = 1;
  image_info.samples = VK_SAMPLE_COUNT_1_BIT;
  image_info.tiling = VK_IMAGE_TILING_OPTIMAL;
  image_info.usage = VK_IMAGE_USAGE_TRANSFER_DST_BIT | VK_IMAGE_USAGE_SAMPLED_BIT;
  image_info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
  image_info.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
  auto image = std::make_shared<Image>(image_info);

  VkImageViewCreateInfo view_info{};
  view_info.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
  view_info.image = image->GetVkImage();
  view_info.viewType = VK_IMAGE_VIEW_TYPE_2D_ARRAY;
  view_info.format = format;
  view_info.subresourceRange.aspectMask = VK_IMAGE_ASPECT_DEPTH_BIT;
  view_info.subresourceRange.levelCount = 1;
  view_info.subresourceRange.layerCount = 1;
  auto image_view = std::make_shared<ImageView>(view_info, image);
  auto sampler = std::make_shared<Sampler>(Lighting::GetDirectionalShadowSamplerCreateInfo());

  constexpr std::array<float, 2> depth_step = {0.25f, 0.75f};
  Buffer staging_buffer(sizeof(depth_step));
  staging_buffer.UploadData(sizeof(depth_step), depth_step.data());

  constexpr size_t value_count = 7;
  VkBufferCreateInfo buffer_info{};
  buffer_info.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
  buffer_info.size = value_count * sizeof(float);
  buffer_info.usage =
      VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_SRC_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT;
  buffer_info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
  VmaAllocationCreateInfo allocation_info{};
  allocation_info.usage = VMA_MEMORY_USAGE_AUTO_PREFER_DEVICE;
  auto output = std::make_shared<Buffer>(buffer_info, allocation_info);
  const std::array<float, value_count> zero{};
  output->Upload(zero);

  auto descriptor_layout = std::make_shared<DescriptorSetLayout>();
  descriptor_layout->PushDescriptorBinding(0, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, VK_SHADER_STAGE_COMPUTE_BIT,
                                           0);
  descriptor_layout->PushDescriptorBinding(1, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_COMPUTE_BIT, 0);
  descriptor_layout->Initialize();
  auto descriptor_set = std::make_shared<DescriptorSet>(descriptor_layout);
  VkDescriptorImageInfo descriptor_image_info{};
  descriptor_image_info.sampler = sampler->GetVkSampler();
  descriptor_image_info.imageView = image_view->GetVkImageView();
  descriptor_image_info.imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
  descriptor_set->UpdateImageDescriptorBinding(0, descriptor_image_info);
  descriptor_set->UpdateBufferDescriptorBinding(1, output);

  const auto probe_path = std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / "EvoEngine_Tests" / "Resources" /
                          "Shaders" / "Compute" / "DirectionalShadowComparisonProbe.slang";
  auto shader = std::make_shared<Shader>();
  ASSERT_TRUE(shader->TryCompile(ShaderType::Compute, Platform::GetShaderGlobalDefines(), probe_path));
  auto pipeline = std::make_shared<ComputePipeline>();
  pipeline->compute_shader = shader;
  pipeline->descriptor_set_layouts.emplace_back(descriptor_layout);
  pipeline->Initialize();
  ASSERT_TRUE(pipeline->Initialized());

  Platform::ImmediateSubmit([&](const VkCommandBuffer command_buffer) {
    image->TransitImageLayout(command_buffer, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL);
    VkBufferImageCopy copy_region{};
    copy_region.imageSubresource.aspectMask = VK_IMAGE_ASPECT_DEPTH_BIT;
    copy_region.imageSubresource.layerCount = 1;
    copy_region.imageExtent = {2, 1, 1};
    image->CopyFromBuffer(command_buffer, staging_buffer.GetVkBuffer(), {copy_region});
    image->TransitImageLayout(command_buffer, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
    Platform::EverythingBarrier(command_buffer);
    pipeline->Bind(command_buffer);
    pipeline->BindDescriptorSet(command_buffer, 0, descriptor_set->GetVkDescriptorSet());
    pipeline->Dispatch(command_buffer, 1);
    Platform::EverythingBarrier(command_buffer);
  });

  std::array<float, value_count> values{};
  output->Download(values);
  constexpr std::array<float, value_count> expected = {0.0f, 0.25f, 0.5f, 0.75f, 1.0f, 1.0f, 0.0f};
  for (size_t index = 0; index < value_count; ++index) {
    EXPECT_NEAR(values[index], expected[index], 0.02f) << index;
  }
  EXPECT_LT(values[0], values[1]);
  EXPECT_LT(values[1], values[2]);
  EXPECT_LT(values[2], values[3]);
  EXPECT_LT(values[3], values[4]);
  EXPECT_GT(values[2], 0.0f);
  EXPECT_LT(values[2], 1.0f);
}

TEST(GpuService, CameraRayTransportShadersCompile) {
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
  EXPECT_TRUE(raygen.TryCompile(ShaderType::RayGen, header, shader_root / "RayTracing/RayGen/Camera.slang"));
  EXPECT_TRUE(any_hit.TryCompile(ShaderType::AnyHit, header, shader_root / "RayTracing/AnyHit/Camera.slang"));
  EXPECT_TRUE(
      closest_hit.TryCompile(ShaderType::ClosestHit, header, shader_root / "RayTracing/ClosestHit/Camera.slang"));
  EXPECT_TRUE(miss.TryCompile(ShaderType::Miss, header, shader_root / "RayTracing/Miss/Camera.slang"));
  EXPECT_TRUE(ray_query.TryCompile(ShaderType::Compute, header, shader_root / "Compute/RayQueryCamera.slang"));
}

TEST(GpuService, LinearSweptSphereCapabilityMatchesShaderDefine) {
  ScopedGpuPlatform platform(true);
  const bool enabled = Platform::RayTracingLinearSweptSpheresEnabled();
  const auto expected_define =
      std::string("#define EE_RAY_TRACING_LINEAR_SWEPT_SPHERES_SUPPORTED ") + (enabled ? "1" : "0");
  EXPECT_NE(Platform::GetShaderGlobalDefines().find(expected_define), std::string::npos);

#ifdef VK_NV_ray_tracing_linear_swept_spheres
  const auto& physical_device = Platform::GetSelectedPhysicalDevice();
  const bool supported =
      Platform::GetInstance().GetCapabilities().support_ray_tracing &&
      physical_device->CheckExtensionSupport(VK_NV_RAY_TRACING_LINEAR_SWEPT_SPHERES_EXTENSION_NAME) &&
      physical_device->ray_tracing_linear_swept_spheres_features_nv.linearSweptSpheres == VK_TRUE;
  EXPECT_EQ(enabled, supported);
#else
  EXPECT_FALSE(enabled);
#endif
}

TEST(GpuService, LinearSweptSphereCameraShadersCompile) {
  ScopedGpuPlatform platform(true);
  if (!Platform::RayTracingLinearSweptSpheresEnabled()) {
    GTEST_SKIP() << "Linear swept spheres are unavailable on the selected test device.";
  }
  const auto shader_root =
      std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / "EvoEngine_SDK" / "Internals" / "DefaultResources" / "Shaders";
  Shader::RegisterShaderIncludePath(shader_root / "Includes");
  const auto header = Platform::GetShaderGlobalDefines() + "\n#define EE_GLTF_COMPILED_FEATURE_MASK 32767u\n";

  Shader raygen;
  Shader any_hit;
  Shader closest_hit;
  Shader ray_query;
  EXPECT_TRUE(raygen.TryCompile(ShaderType::RayGen, header, shader_root / "RayTracing/RayGen/Camera.slang"));
  EXPECT_TRUE(any_hit.TryCompile(ShaderType::AnyHit, header, shader_root / "RayTracing/AnyHit/Camera.slang"));
  EXPECT_TRUE(
      closest_hit.TryCompile(ShaderType::ClosestHit, header, shader_root / "RayTracing/ClosestHit/Camera.slang"));
  EXPECT_TRUE(ray_query.TryCompile(ShaderType::Compute, header, shader_root / "Compute/RayQueryCamera.slang"));
}

TEST(GpuService, EnvironmentLightingShadersCompile) {
  ScopedGpuPlatform platform;
  const auto shader_root =
      std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / "EvoEngine_SDK" / "Internals" / "DefaultResources" / "Shaders";
  Shader::RegisterShaderIncludePath(shader_root / "Includes");
  const auto header = Platform::GetShaderGlobalDefines();

  Shader point_cloud_raygen;
  Shader point_cloud_miss;
  Shader point_cloud_closest_hit;
  Shader volumetric_clouds;
  Shader volumetric_clouds_composite;
  EXPECT_TRUE(
      point_cloud_raygen.TryCompile(ShaderType::RayGen, header, shader_root / "RayTracing/RayGen/PointCloud.slang"));
  EXPECT_TRUE(point_cloud_miss.TryCompile(ShaderType::Miss, header, shader_root / "RayTracing/Miss/PointCloud.slang"));
  EXPECT_TRUE(point_cloud_closest_hit.TryCompile(ShaderType::ClosestHit, header,
                                                 shader_root / "RayTracing/ClosestHit/PointCloud.slang"));
  EXPECT_TRUE(
      volumetric_clouds.TryCompile(ShaderType::Compute, header, shader_root / "Compute/VolumetricClouds.slang"));
  EXPECT_TRUE(volumetric_clouds_composite.TryCompile(ShaderType::Compute, header,
                                                     shader_root / "Compute/VolumetricCloudsComposite.slang"));
}

TEST(GpuService, DdgiMaterialShadersCompile) {
  ScopedGpuPlatform platform;
  const auto shader_root =
      std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / "EvoEngine_SDK" / "Internals" / "DefaultResources" / "Shaders";
  Shader::RegisterShaderIncludePath(shader_root / "Includes");
  const auto header = Platform::GetShaderGlobalDefines();

  Shader raygen;
  Shader any_hit;
  Shader closest_hit;
  Shader miss;
  Shader transparent;
  Shader gtao;
  Shader ambient_occlusion_blur;
  EXPECT_TRUE(raygen.TryCompile(ShaderType::RayGen, header, shader_root / "RayTracing/RayGen/DDGIProbeTrace.slang"));
  EXPECT_TRUE(any_hit.TryCompile(ShaderType::AnyHit, header, shader_root / "RayTracing/AnyHit/DDGIProbeTrace.slang"));
  EXPECT_TRUE(closest_hit.TryCompile(ShaderType::ClosestHit, header,
                                     shader_root / "RayTracing/ClosestHit/DDGIProbeTrace.slang"));
  EXPECT_TRUE(miss.TryCompile(ShaderType::Miss, header, shader_root / "RayTracing/Miss/DDGIProbeTrace.slang"));
  EXPECT_TRUE(transparent.TryCompile(ShaderType::Fragment, header,
                                     shader_root / "Graphics/Fragment/Standard/StandardTransparent.slang"));
  EXPECT_TRUE(gtao.TryCompile(ShaderType::Compute, header, shader_root / "Compute/PostProcessing/GTAO.slang"));
  EXPECT_TRUE(ambient_occlusion_blur.TryCompile(ShaderType::Compute, header,
                                                shader_root / "Compute/PostProcessing/AmbientOcclusionBlur.slang"));
}

TEST(GpuService, DdgiProbeUpdateVariantsCompile) {
  ScopedGpuPlatform platform;
  const auto shader_root =
      std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / "EvoEngine_SDK" / "Internals" / "DefaultResources" / "Shaders";
  Shader::RegisterShaderIncludePath(shader_root / "Includes");
  const auto shader_path = shader_root / "Compute/DDGIProbeUpdate.slang";
  const auto base_header = Platform::GetShaderGlobalDefines();
  const auto compile_variant = [&](const uint32_t mode, const bool shared) {
    Shader shader;
    const auto header = base_header + "\n#define EE_DDGI_PROBE_UPDATE_MODE " + std::to_string(mode) +
                        "\n#define EE_DDGI_PROBE_USE_SHARED_RAYS " + (shared ? "1\n" : "0\n");
    return shader.TryCompile(ShaderType::Compute, header, shader_path);
  };

  EXPECT_TRUE(compile_variant(0u, false));
  EXPECT_TRUE(compile_variant(1u, false));
  EXPECT_TRUE(compile_variant(2u, false));
  EXPECT_TRUE(compile_variant(1u, true));
  EXPECT_TRUE(compile_variant(2u, true));
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

TEST(GpuService, BufferUploadBatchRoundTrip) {
  ScopedGpuPlatform platform;
  constexpr std::array<uint32_t, 4> first_input = {1u, 2u, 3u, 4u};
  constexpr std::array<uint32_t, 4> second_input = {5u, 6u, 7u, 8u};
  constexpr auto byte_size = static_cast<VkDeviceSize>(first_input.size() * sizeof(first_input[0]));

  VkBufferCreateInfo buffer_create_info{};
  buffer_create_info.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
  buffer_create_info.size = byte_size;
  buffer_create_info.usage =
      VK_BUFFER_USAGE_TRANSFER_SRC_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_STORAGE_BUFFER_BIT;
  buffer_create_info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
  VmaAllocationCreateInfo allocation_create_info{};
  allocation_create_info.usage = VMA_MEMORY_USAGE_AUTO;
  auto first = std::make_shared<Buffer>(buffer_create_info, allocation_create_info);
  buffer_create_info.size = sizeof(uint32_t);
  auto second = std::make_shared<Buffer>(buffer_create_info, allocation_create_info);

  BufferUploadBatch batch;
  batch.Add(first, first_input.data(), sizeof(first_input));
  BufferUploadOptions grow_options;
  grow_options.capacity_policy = BufferUploadCapacityPolicy::GrowIfNeeded;
  batch.Add(second, second_input.data(), sizeof(second_input), 0, grow_options);
  EXPECT_EQ(batch.GetEntryCount(), 2u);
  EXPECT_EQ(batch.GetStagingSize(), byte_size * 2);
  batch.SubmitImmediate();
  EXPECT_EQ(batch.GetEntryCount(), 0u);
  EXPECT_EQ(second->GetSize(), byte_size);

  const auto download = [](const std::shared_ptr<Buffer>& buffer) {
    std::array<uint32_t, 4> output{};
    const auto bytes = buffer->DownloadDataAsync(sizeof(output)).get();
    memcpy(output.data(), bytes.data(), bytes.size());
    return output;
  };
  EXPECT_EQ(download(first), first_input);
  EXPECT_EQ(download(second), second_input);
}

TEST(GpuService, BufferUploadBatchRejectsInvalidRanges) {
  ScopedGpuPlatform platform;
  VkBufferCreateInfo buffer_create_info{};
  buffer_create_info.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
  buffer_create_info.size = 16;
  buffer_create_info.usage = VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_STORAGE_BUFFER_BIT;
  buffer_create_info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
  auto buffer = std::make_shared<Buffer>(buffer_create_info);
  constexpr std::array<uint32_t, 2> input = {1u, 2u};

  BufferUploadBatch batch;
  batch.Add(buffer, input.data(), sizeof(input), 0);
  EXPECT_THROW(batch.Add(buffer, input.data(), sizeof(input), sizeof(uint32_t)), std::invalid_argument);
  EXPECT_THROW(batch.Add(buffer, input.data(), sizeof(input), 12), std::out_of_range);
  EXPECT_THROW(batch.Add(buffer, input.data(), sizeof(input) - 1, 8), std::invalid_argument);
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

TEST(GpuService, RayTracingStrandStorageIsAbsentWhenFeatureIsDisabled) {
  ScopedGpuPlatform platform;
  ASSERT_FALSE(Platform::RayTracingLinearSweptSpheresEnabled());
  EXPECT_FALSE(GeometryStorage::GetRayTracingStrandPointBuffer());
  EXPECT_FALSE(GeometryStorage::GetRayTracingStrandIndexBuffer());

  Strands strands;
  strands.OnCreate();
  StrandPointAttributes attributes{};
  attributes.normal = true;
  std::vector<StrandPoint> points(4);
  for (size_t index = 0; index < points.size(); ++index) {
    points[index].position = glm::vec3(static_cast<float>(index), 0.0f, 0.0f);
    points[index].normal = glm::vec3(0.0f, 1.0f, 0.0f);
    points[index].thickness = 0.1f;
  }
  strands.SetSegments(attributes, {0}, points);
  EXPECT_FALSE(GeometryStorage::HasPendingUploads());
}

TEST(GpuService, RayTracingStrandStorageCompactionAdjustsAbsoluteIndices) {
  ScopedGpuPlatform platform(true);
  if (!Platform::RayTracingLinearSweptSpheresEnabled()) {
    GTEST_SKIP() << "Linear swept spheres are unavailable on the selected test device.";
  }

  StrandPointAttributes attributes{};
  attributes.normal = true;
  const auto create_strands = [&](const float offset) {
    auto strands = std::make_unique<Strands>();
    strands->OnCreate();
    std::vector<StrandPoint> points(4);
    for (size_t index = 0; index < points.size(); ++index) {
      points[index].position = glm::vec3(offset + static_cast<float>(index), 0.0f, 0.0f);
      points[index].normal = glm::vec3(0.0f, 1.0f, 0.0f);
      points[index].thickness = 0.1f;
    }
    strands->SetSegments(attributes, {0}, points);
    return strands;
  };

  auto first = create_strands(0.0f);
  auto second = create_strands(10.0f);
  GeometryStorage::WaitForPendingUploads();
  EXPECT_EQ(GeometryStorage::PeekRayTracingStrandIndex(8), 9u);

  first.reset();
  GeometryStorage::WaitForPendingUploads();
  for (uint32_t index = 0; index < 8; ++index) {
    EXPECT_EQ(GeometryStorage::PeekRayTracingStrandIndex(index), index);
  }
  EXPECT_FLOAT_EQ(GeometryStorage::PeekRayTracingStrandPoint(0).position.x, 11.0f);

  const auto downloaded =
      GeometryStorage::GetRayTracingStrandIndexBuffer()->DownloadDataAsync(8 * sizeof(uint32_t)).get();
  ASSERT_EQ(downloaded.size(), 8 * sizeof(uint32_t));
  std::array<uint32_t, 8> indices{};
  memcpy(indices.data(), downloaded.data(), downloaded.size());
  for (uint32_t index = 0; index < indices.size(); ++index) {
    EXPECT_EQ(indices[index], index);
  }
}

TEST(GpuService, LinearSweptSphereBlasBuildsFromAssetOwnedInput) {
  ScopedGpuPlatform platform(true);
  if (!Platform::RayTracingLinearSweptSpheresEnabled()) {
    GTEST_SKIP() << "Linear swept spheres are unavailable on the selected test device.";
  }

  Strands strands;
  strands.OnCreate();
  StrandPointAttributes attributes{};
  attributes.normal = true;
  std::vector<StrandPoint> points(4);
  for (size_t index = 0; index < points.size(); ++index) {
    points[index].position = glm::vec3(static_cast<float>(index), 0.25f * static_cast<float>(index % 2), 0.0f);
    points[index].normal = glm::vec3(0.0f, 0.0f, 1.0f);
    points[index].thickness = 0.05f + 0.01f * static_cast<float>(index);
  }
  strands.SetSegments(attributes, {0}, points);

  ASSERT_TRUE(strands.GetBlas());
  EXPECT_TRUE(strands.GetBlas()->IsReady());
  EXPECT_NE(strands.GetBlas()->GetDeviceAddress(), 0u);
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
