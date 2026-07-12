#include "EvoEngine_SDK_PCH.hpp"

#include <gtest/gtest.h>

#include "Application.hpp"
#include "ApplicationContext.hpp"
#include "ComputePipeline.hpp"
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

ApplicationInitializationSettings TestApplicationSettings(const TempProject& project) {
  ApplicationInitializationSettings settings;
  settings.project_path = project.ProjectPath();
  settings.load_default_resources = false;
  settings.load_project_assets = false;
  settings.load_project_start_scene = false;
  settings.enable_runtime_packages = false;
  settings.graphics_settings.use_mesh_shader = false;
  settings.graphics_settings.use_ray_tracing = false;
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
  ScopedGpuPlatform() {
    project_ = std::make_unique<TempProject>();
    application_ = std::make_unique<Application>();
    application_->PushLayer<RenderLayer>("Render Layer");
    Jobs::Initialize(2);
    jobs_initialized_ = true;
    try {
      PlatformLifecycleTestAccess::Initialize(TestApplicationSettings(*project_));
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

TEST(GpuService, GltfRayTracingNumericalProbeMatchesAnalyticValues) {
  ScopedGpuPlatform platform;
  const auto shader_root =
      std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / "EvoEngine_SDK" / "Internals" / "DefaultResources" / "Shaders";
  Shader::RegisterShaderIncludePath(shader_root / "Includes");

  auto descriptor_layout = std::make_shared<DescriptorSetLayout>();
  descriptor_layout->PushDescriptorBinding(0, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_COMPUTE_BIT, 0);
  descriptor_layout->Initialize();

  constexpr size_t value_count = 20;
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
  PlatformLifecycleTestAccess::PreUpdate();
  gpu_service.WaitIdle();
  PlatformLifecycleTestAccess::PreUpdate();

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
