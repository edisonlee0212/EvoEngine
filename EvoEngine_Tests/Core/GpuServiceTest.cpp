#include "EvoEngine_SDK_PCH.hpp"

#include <gtest/gtest.h>

#include "Application.hpp"
#include "ApplicationContext.hpp"
#include "ComputePipeline.hpp"
#include "Cubemap.hpp"
#include "DemoScene.hpp"
#include "EnvironmentalLighting.hpp"
#include "GeometryStorage.hpp"
#include "GltfMaterial.hpp"
#include "GpuService.hpp"
#include "GraphicsResources.hpp"
#include "HddagiCamera.hpp"
#include "HddagiGather.hpp"
#include "HddagiLight.hpp"
#include "HddagiProbe.hpp"
#include "HddagiResources.hpp"
#include "HddagiTypes.hpp"
#include "HddagiVoxelizer.hpp"
#include "Jobs.hpp"
#include "Lights.hpp"
#include "Mesh.hpp"
#include "MeshRenderer.hpp"
#include "Platform.hpp"
#include "ProjectManager.hpp"
#include "RenderLayer.hpp"
#include "Resources.hpp"
#include "Scene.hpp"
#include "SdfgiCapabilities.hpp"
#include "SdfgiDebug.hpp"
#include "SdfgiGather.hpp"
#include "SdfgiLight.hpp"
#include "SdfgiPreprocess.hpp"
#include "SdfgiProbe.hpp"
#include "SdfgiProbeLayout.hpp"
#include "SdfgiResources.hpp"
#include "SdfgiRuntime.hpp"
#include "SdfgiVoxelizer.hpp"
#include "Serialization.hpp"
#include "Shader.hpp"
#include "Strands.hpp"
#include "Texture2D.hpp"
#include "TextureStorage.hpp"

#include <array>
#include <atomic>
#include <chrono>
#include <cstring>
#include <filesystem>
#include <glm/gtc/packing.hpp>
#include <memory>
#include <mutex>
#include <numeric>
#include <stdexcept>
#include <thread>
#include <vector>

using namespace evo_engine;

namespace evo_engine {
class SdfgiTestAccess {
 public:
  static void ValidateGiSettings(RenderLayer& render, const std::shared_ptr<Scene>& scene) {
    render.ValidateSceneGiSettings(scene);
  }

  static std::vector<std::shared_ptr<DescriptorSetLayout>> HostLayouts(const RenderLayer& render) {
    return {render.per_frame_layout_, render.camera_g_buffer_layout_, render.lighting_layout_,
            render.raster_lighting_texture_layout_, render.deferred_compute_lighting_layout_};
  }
  static void ExecuteSceneFrame(RenderLayer& render, const std::shared_ptr<Scene>& scene) {
    render.per_frame_descriptor_sets_.resize(Platform::GetMaxFramesInFlight());
    render.render_graph_transient_resource_stores_.resize(Platform::GetMaxFramesInFlight());
    render.ExecuteSceneFramePasses(scene);
  }

  static bool HasDdgiResources(const RenderLayer& render) {
    return !render.ddgi_cascade_runtime_states_.empty() || render.ddgi_probe_update_pipeline_ ||
           render.ddgi_probe_update_layout_ || render.ddgi_probe_ray_output_layout_ || render.ddgi_atlas_sampler_;
  }

  static void RetireHddagiFrame(RenderLayer& render) {
    const auto slot = Platform::GetCurrentFrameIndex();
    if (slot < render.hddagi_frame_resources_.size())
      render.hddagi_frame_resources_[slot].clear();
  }
  static std::shared_ptr<const HddagiResources> LatestHddagiCapture(const RenderLayer& render) {
    const auto& slot = render.hddagi_frame_resources_.at(Platform::GetCurrentFrameIndex());
    for (auto i = slot.rbegin(); i != slot.rend(); ++i)
      if ((*i)->capture_source)
        return *i;
    return {};
  }
  static uint64_t CaptureImmediately(RenderLayer& render, const std::shared_ptr<Scene>& scene,
                                     const glm::vec3 position) {
    const auto camera = render.GetOrCreateReflectionProbeCaptureCameras(1).front();
    GlobalTransform transform;
    transform.SetPosition(position);
    render.RenderSceneToCameraImmediately(scene, transform, camera, true);
    return camera->GetHandle().GetValue();
  }
};

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

TEST(SdfgiCapabilities, ValidatesConfigurationAndReferenceShapes) {
  EXPECT_TRUE(GetSdfgiImageRequirements(0, 30).empty());
  EXPECT_TRUE(GetSdfgiImageRequirements(9, 30).empty());
  EXPECT_TRUE(GetSdfgiImageRequirements(4, 0).empty());
  EXPECT_TRUE(GetSdfgiImageRequirements(4, 31).empty());
  EXPECT_TRUE(GetSdfgiImageRequirements(4, 7).empty());
  const auto requirements = GetSdfgiImageRequirements(8, 30, 128, 128, 8);
  ASSERT_EQ(requirements.size(), 14u);
  for (const auto& requirement : requirements) {
    if (std::string(requirement.name) == "occlusion") {
      EXPECT_EQ(requirement.extent.width, 256u);
      EXPECT_EQ(requirement.extent.depth, 1024u);
      EXPECT_EQ(requirement.sampled_format, VK_FORMAT_R4G4B4A4_UNORM_PACK16);
    }
    if (std::string(requirement.name) == "probe_atlas") {
      EXPECT_EQ(requirement.extent.width, 2312u);
      EXPECT_EQ(requirement.extent.height, 136u);
      EXPECT_EQ(requirement.layers, 16u);
    }
    EXPECT_EQ(requirement.CreateFlags() != 0, requirement.storage_format != requirement.sampled_format);
  }
}

TEST(SdfgiCapabilities, RejectsMissingFeaturesAndInsufficientLimits) {
  VkPhysicalDeviceFeatures features{};
  VkPhysicalDeviceLimits limits{};
  SdfgiCapabilityReport report;
  EXPECT_FALSE(report.Supported());
  report.checks = EvaluateSdfgiDeviceLimits(features, limits);
  EXPECT_FALSE(report.Supported());
  EXPECT_NE(report.ToString().find("fragmentStoresAndAtomics"), std::string::npos);
  EXPECT_NE(report.ToString().find("maxBoundDescriptorSets required=6 available=0"), std::string::npos);
  EXPECT_NE(report.ToString().find("maxPerStageDescriptorStorageBuffers required=5 available=0"), std::string::npos);
  features.fragmentStoresAndAtomics = VK_TRUE;
  limits.maxBoundDescriptorSets = 6;
  report.checks = EvaluateSdfgiDeviceLimits(features, limits);
  EXPECT_TRUE(report.checks.front().supported);
  EXPECT_EQ(report.ToString().find("unavailable: maxBoundDescriptorSets"), std::string::npos);
  EXPECT_FALSE(report.Supported());
}

TEST(SdfgiCapabilities, ChecksImageUsageDimensionsLayersAndAllocationLimit) {
  constexpr auto all_features = ~VkFormatFeatureFlags2{0};
  VkImageFormatProperties properties{};
  properties.maxExtent = {4096, 4096, 4096};
  properties.maxMipLevels = 1;
  properties.maxArrayLayers = 256;
  properties.sampleCounts = VK_SAMPLE_COUNT_1_BIT;
  properties.maxResourceSize = ~VkDeviceSize{0};
  for (const auto& requirement : GetSdfgiImageRequirements(4, 30, 128, 128, 8)) {
    SCOPED_TRACE(requirement.name);
    EXPECT_TRUE(SupportsSdfgiImage(requirement, all_features, all_features, VK_SUCCESS, properties));
    EXPECT_FALSE(
        SupportsSdfgiImage(requirement, all_features, all_features, VK_ERROR_FORMAT_NOT_SUPPORTED, properties));
    EXPECT_FALSE(SupportsSdfgiImage(requirement, all_features & ~VK_FORMAT_FEATURE_2_STORAGE_IMAGE_BIT, all_features,
                                    VK_SUCCESS, properties));
    EXPECT_FALSE(SupportsSdfgiImage(requirement, all_features, 0, VK_SUCCESS, properties));
    if (requirement.linear_filter) {
      EXPECT_FALSE(SupportsSdfgiImage(requirement, all_features,
                                      all_features & ~VK_FORMAT_FEATURE_2_SAMPLED_IMAGE_FILTER_LINEAR_BIT, VK_SUCCESS,
                                      properties));
    }
    if (requirement.atomic) {
      EXPECT_FALSE(SupportsSdfgiImage(requirement, all_features & ~VK_FORMAT_FEATURE_2_STORAGE_IMAGE_ATOMIC_BIT,
                                      all_features, VK_SUCCESS, properties));
    }
    auto insufficient = properties;
    insufficient.maxExtent.depth = requirement.extent.depth - 1;
    EXPECT_FALSE(SupportsSdfgiImage(requirement, all_features, all_features, VK_SUCCESS, insufficient));
    insufficient = properties;
    insufficient.maxArrayLayers = requirement.layers - 1;
    EXPECT_FALSE(SupportsSdfgiImage(requirement, all_features, all_features, VK_SUCCESS, insufficient));
    insufficient = properties;
    insufficient.maxResourceSize = 1;
    EXPECT_FALSE(SupportsSdfgiImage(requirement, all_features, all_features, VK_SUCCESS, insufficient));
  }
}

TEST(SdfgiCapabilities, PackedSampledViewsDoNotRequireStorageOrLinearIntegerFiltering) {
  const SdfgiImageRequirement requirement{
      "radiance", VK_FORMAT_R32_UINT, VK_FORMAT_E5B9G9R9_UFLOAT_PACK32, VK_IMAGE_TYPE_3D, {128, 128, 128}, 1, 4, true};
  const VkImageFormatProperties properties{{128, 128, 128}, 1, 1, VK_SAMPLE_COUNT_1_BIT, 8u * 1024u * 1024u};
  EXPECT_TRUE(
      SupportsSdfgiImage(requirement,
                         VK_FORMAT_FEATURE_2_STORAGE_IMAGE_BIT | VK_FORMAT_FEATURE_2_TRANSFER_SRC_BIT |
                             VK_FORMAT_FEATURE_2_TRANSFER_DST_BIT,
                         VK_FORMAT_FEATURE_2_SAMPLED_IMAGE_BIT | VK_FORMAT_FEATURE_2_SAMPLED_IMAGE_FILTER_LINEAR_BIT,
                         VK_SUCCESS, properties));
}

TEST(GiSettings, CurrentGpuRejectsEditsAtomicallyAndIgnoresInactiveHistory) {
  ScopedGpuPlatform platform(false);
  EnvironmentalLighting lighting;
  std::string error;
  auto candidate = lighting.GetGiSettings();
  ASSERT_TRUE(lighting.TrySetGiSettings(candidate, error)) << error;
  const auto accepted = lighting.GetGiSettings();
  candidate.gi_probe_settings.probe_count_x = 64;
  EXPECT_FALSE(lighting.TrySetGiSettings(candidate, error));
  EXPECT_FALSE(error.empty());
  EXPECT_EQ(lighting.GetGiSettings(), accepted);
  candidate = accepted;
  candidate.indirect_gi_provider = IndirectGiProvider::AutomaticDdgi;
  EXPECT_FALSE(lighting.TrySetGiSettings(candidate, error));
  EXPECT_EQ(lighting.GetGiSettings(), accepted);
  candidate = accepted;
  candidate.ddgi_settings.runtime.history_count = 5;
  ASSERT_TRUE(lighting.TrySetGiSettings(candidate, error)) << error;
  candidate.sdfgi_settings.probe_spacing_cells = 8;
  ASSERT_TRUE(lighting.TrySetGiSettings(candidate, error)) << error;
  EXPECT_EQ(lighting.GetGiSettings().sdfgi_settings.probe_spacing_cells, 8u);
}

TEST(GiSettings, RawRuntimeEditsRollbackButUnsupportedLoadedAssetsRetainValues) {
  ScopedGpuPlatform platform(false);
  const auto render = ApplicationContext::Get().GetLayer<RenderLayer>();
  ASSERT_TRUE(render);
  const auto scene = std::make_shared<Scene>();
  const auto lighting = std::make_shared<EnvironmentalLighting>();
  scene->environmental_lighting = lighting;
  SdfgiTestAccess::ValidateGiSettings(*render, scene);
  const auto accepted = lighting->GetGiSettings();
  lighting->gi_probe_settings.probe_count_x = 64;
  lighting->ddgi_settings.runtime.history_count = 7;
  SdfgiTestAccess::ValidateGiSettings(*render, scene);
  EXPECT_EQ(lighting->GetGiSettings(), accepted);

  const auto replacement = std::make_shared<EnvironmentalLighting>();
  replacement->gi_probe_settings.probe_count_y = 18;
  scene->environmental_lighting = replacement;
  SdfgiTestAccess::ValidateGiSettings(*render, scene);
  EXPECT_EQ(replacement->gi_probe_settings.probe_count_y, 18u);
  SdfgiTestAccess::ExecuteSceneFrame(*render, scene);
  ASSERT_TRUE(scene->GetSdfgiRuntime());
  EXPECT_FALSE(scene->GetSdfgiRuntime()->resources);
  EXPECT_FALSE(scene->GetSdfgiRuntime()->fallback_reason.empty());
  replacement->gi_probe_settings.probe_count_y = 17;
  SdfgiTestAccess::ValidateGiSettings(*render, scene);
  replacement->gi_probe_settings.probe_count_y = 18;
  SdfgiTestAccess::ValidateGiSettings(*render, scene);
  EXPECT_EQ(replacement->gi_probe_settings.probe_count_y, 17u);

  const auto next_scene = std::make_shared<Scene>();
  const auto next_lighting = std::make_shared<EnvironmentalLighting>();
  next_lighting->indirect_gi_provider = IndirectGiProvider::AutomaticDdgi;
  next_scene->environmental_lighting = next_lighting;
  SdfgiTestAccess::ValidateGiSettings(*render, next_scene);
  EXPECT_EQ(next_lighting->indirect_gi_provider, IndirectGiProvider::AutomaticDdgi);
  EXPECT_FALSE(SdfgiTestAccess::HasDdgiResources(*render));
}

TEST(SdfgiCapabilities, CurrentGpuPreflightWithRayFeaturesDisabled) {
  ScopedGpuPlatform platform(false);
  const auto report = QuerySdfgiCapabilities();
  std::cout << report.ToString() << std::endl;
  EXPECT_FALSE(report.ray_tracing_enabled);
  EXPECT_FALSE(report.ray_query_enabled);
  EXPECT_FALSE(report.acceleration_structures_enabled);
  EXPECT_FALSE(Platform::RayAccelerationStructureEnabled());
  EXPECT_TRUE(report.Supported()) << report.ToString();
  EXPECT_FALSE(QuerySdfgiCapabilities(0, 30).Supported());
}

TEST(SdfgiRuntime, SceneFrameBoundaryRunsExternalPassOnceWithoutDdgiOrRayFeatures) {
  ScopedGpuPlatform platform(false);
  const auto render = ApplicationContext::Get().GetLayer<RenderLayer>();
  ASSERT_TRUE(render);
  const auto scene = std::make_shared<Scene>();
  const auto lighting = std::make_shared<EnvironmentalLighting>();
  scene->environmental_lighting = lighting;
  lighting->indirect_gi_provider = IndirectGiProvider::AutomaticSdfgi;
  lighting->ddgi_settings.runtime.enabled = true;
  uint32_t calls = 0;
  bool expect_sdfgi = true;
  render->RegisterFrameRenderPass({"SdfgiFrameBoundaryTest", RenderPassQueue::Graphics, RenderPassScope::Frame},
                                  [&](const VkCommandBuffer) -> uint32_t {
                                    ++calls;
                                    if (expect_sdfgi) {
                                      EXPECT_TRUE(scene->GetSdfgiRuntime());
                                      if (const auto runtime = scene->GetSdfgiRuntime())
                                        EXPECT_EQ(runtime->maintenance_count, 1u);
                                    }
                                    return 0;
                                  });
  SdfgiTestAccess::ExecuteSceneFrame(*render, scene);
  EXPECT_EQ(calls, 1u);
  ASSERT_TRUE(scene->GetSdfgiRuntime());
  EXPECT_TRUE(scene->GetSdfgiRuntime()->capabilities.Supported());
  EXPECT_FALSE(scene->GetSdfgiRuntime()->published);
  EXPECT_TRUE(scene->GetSdfgiRuntime()->missing_anchor);
  EXPECT_FALSE(SdfgiTestAccess::HasDdgiResources(*render));
  EXPECT_FALSE(Platform::RayTracingEnabled());
  EXPECT_FALSE(Platform::RayQueryEnabled());
  EXPECT_FALSE(Platform::RayAccelerationStructureEnabled());
  lighting->indirect_gi_provider = IndirectGiProvider::Environment;
  expect_sdfgi = false;
  SdfgiTestAccess::ExecuteSceneFrame(*render, scene);
  EXPECT_EQ(calls, 2u);
  EXPECT_FALSE(scene->GetSdfgiRuntime());
  lighting->indirect_gi_provider = IndirectGiProvider::AutomaticSdfgi;
  expect_sdfgi = true;
  SdfgiTestAccess::ExecuteSceneFrame(*render, scene);
  const auto replacement = std::make_shared<Scene>();
  const auto replacement_lighting = std::make_shared<EnvironmentalLighting>();
  replacement_lighting->indirect_gi_provider = IndirectGiProvider::Environment;
  replacement->environmental_lighting = replacement_lighting;
  expect_sdfgi = false;
  SdfgiTestAccess::ExecuteSceneFrame(*render, replacement);
  EXPECT_FALSE(scene->GetSdfgiRuntime());
  EXPECT_FALSE(replacement->GetSdfgiRuntime());
  EXPECT_EQ(calls, 4u);
  EXPECT_FALSE(SdfgiTestAccess::HasDdgiResources(*render));
}

TEST(SdfgiResources, AllocatesClearsPackedViewsAndRetiresOnTheMainQueueWithoutRayFeatures) {
  ScopedGpuPlatform platform(false);
  SdfgiSettings reference;
  reference.voxel_count_x = reference.voxel_count_y = 128;
  reference.probe_spacing_cells = 8;
  ApplicationContext::Get().RegisterAsset<Shader>("Shader", {".eveshader", ".slang"});
  const auto render = ApplicationContext::Get().GetLayer<RenderLayer>();
  ASSERT_TRUE(render);
  const auto host_layouts = SdfgiTestAccess::HostLayouts(*render);
  std::string failure;
  EXPECT_FALSE(SdfgiResources::TryCreate(reference, host_layouts, failure, 3));
  EXPECT_NE(failure.find("forced allocation failure"), std::string::npos) << failure;
  EXPECT_FALSE(Platform::RayTracingEnabled());
  EXPECT_FALSE(Platform::RayQueryEnabled());
  EXPECT_FALSE(Platform::RayAccelerationStructureEnabled());
  auto field = SdfgiResources::TryCreate(reference, host_layouts, failure);
  ASSERT_TRUE(field) << failure;
  EXPECT_EQ(field->textures.size(), 46u);
  EXPECT_EQ(field->buffers.size(), 19u + Platform::GetMaxFramesInFlight() * 10u);
  EXPECT_EQ(field->buffers.at("ProbePlacement").buffer->GetSize(), sizeof(glm::vec4));
  EXPECT_EQ(field->buffers.at("ProbePlacementScroll").buffer->GetSize(), sizeof(glm::vec4));
  EXPECT_EQ(field->pipelines.size(), 20u);
  EXPECT_TRUE(field->voxel_pipeline->Initialized());
  EXPECT_FALSE(field->initialization_recorded);
  for (const auto& [name, texture] : field->textures) {
    SCOPED_TRACE(name);
    EXPECT_EQ(texture.image->GetFormat(), texture.requirement.storage_format);
    EXPECT_EQ(texture.image->GetExtent().depth, texture.requirement.extent.depth);
    EXPECT_EQ(texture.storage_view->GetImage(), texture.sampled_view->GetImage());
    EXPECT_GT(texture.image->GetVmaAllocationInfo().size, 0u);
  }
  EXPECT_EQ(field->textures.at("Atlas").requirement.layers, 8u);
  EXPECT_EQ(field->textures.at("Cascade0.History").requirement.layers, 30u);
  EXPECT_EQ(field->textures.at("Occlusion").requirement.extent.depth, 512u);
  for (size_t i = 0; i < 4; ++i)
    std::cout << "SDFGI allocated memory class " << i << ": " << field->allocated_bytes[i] << " bytes\n";

  const auto make_check = [&](const char* filename, const SdfgiLayout layout, const uint32_t push_size,
                              const bool sky) {
    auto pipeline = std::make_shared<ComputePipeline>();
    pipeline->compute_shader =
        Shader::CreateTemporary(ShaderType::Compute, "#define EE_SDFGI_ABI_ONLY 1\n#define EE_SDFGI_RESOURCE_CHECK 1\n",
                                Resources::GetDefaultResourcesPath() / "Shaders/Compute" / filename);
    pipeline->descriptor_set_layouts = {field->layouts[static_cast<size_t>(layout)]};
    if (sky)
      pipeline->descriptor_set_layouts.push_back(field->layouts[static_cast<size_t>(SdfgiLayout::Sky)]);
    pipeline->push_constant_ranges.push_back({VK_SHADER_STAGE_COMPUTE_BIT, 0, push_size});
    pipeline->Initialize();
    return pipeline;
  };
  const auto light_check =
      make_check("SdfgiDirectLight.slang", SdfgiLayout::DirectLight, sizeof(SdfgiDirectLightPushConstant), false);
  const auto integrate_check =
      make_check("SdfgiIntegrate.slang", SdfgiLayout::Integrate, sizeof(SdfgiIntegratePushConstant), true);
  ASSERT_TRUE(light_check->Initialized());
  ASSERT_TRUE(integrate_check->Initialized());
  const auto gather_check = std::make_shared<ComputePipeline>();
  gather_check->descriptor_set_layouts = field->pipelines.at("GatherAbi")->descriptor_set_layouts;
  gather_check->compute_shader =
      Shader::CreateTemporary(ShaderType::Compute, "#define EE_SDFGI_ABI_ONLY 1\n#define EE_SDFGI_RESOURCE_CHECK 1\n",
                              Resources::GetDefaultResourcesPath() / "Shaders/Compute/SdfgiGatherAbi.slang");
  gather_check->Initialize();
  ASSERT_TRUE(gather_check->Initialized());
  const auto export_spirv = [&](const std::string& name, const std::shared_ptr<Shader>& shader) {
    if (const char* directory = std::getenv("EVOENGINE_SDFGI_SPIRV_OUTPUT_DIR"); directory && directory[0]) {
      std::vector<uint32_t> words;
      ASSERT_TRUE(Shader::CompileToSpirv(shader->GetShaderType(), shader->PeekShaderCode(), words,
                                         std::filesystem::path(directory) / (name + ".slang")));
      std::filesystem::create_directories(directory);
      std::ofstream out(std::filesystem::path(directory) / (name + ".spv"), std::ios::binary);
      out.write(reinterpret_cast<const char*>(words.data()), words.size() * sizeof(uint32_t));
      ASSERT_TRUE(out.good());
    }
  };
  for (const auto& [name, pipeline] : field->pipelines)
    export_spirv(name, pipeline->compute_shader);
  export_spirv("LightResourceCheck", light_check->compute_shader);
  export_spirv("IntegrateResourceCheck", integrate_check->compute_shader);
  export_spirv("GatherResourceCheck", gather_check->compute_shader);
  auto environment = std::make_shared<ComputePipeline>();
  environment->descriptor_set_layouts = host_layouts;
  environment->push_constant_ranges = field->pipelines.at("DeferredSdfgi")->push_constant_ranges;
  environment->compute_shader =
      Shader::CreateTemporary(ShaderType::Compute, Platform::GetShaderGlobalDefines(),
                              Resources::GetDefaultResourcesPath() / "Shaders/Compute/DeferredComputeLighting.slang");
  environment->Initialize();
  ASSERT_TRUE(environment->Initialized());
  EXPECT_EQ(environment->descriptor_set_layouts.size(), 5u);
  EXPECT_EQ(field->pipelines.at("DeferredSdfgi")->descriptor_set_layouts.size(), 6u);
  export_spirv("DeferredEnvironment", environment->compute_shader);
  export_spirv("VoxelVertexAbi", field->voxel_pipeline->vertex_shader);
  export_spirv("VoxelFragmentAbi", field->voxel_pipeline->fragment_shader);
  const auto debug_renderer = std::make_shared<SdfgiDebugRenderer>();
  ASSERT_TRUE(debug_renderer->sdf->Initialized());
  export_spirv("DebugSdf", debug_renderer->sdf->compute_shader);
  for (uint32_t i = 0; i < debug_renderer->graphics.size(); ++i) {
    ASSERT_TRUE(debug_renderer->graphics[i]->Initialized());
    export_spirv("DebugVertex" + std::to_string(i), debug_renderer->graphics[i]->vertex_shader);
    export_spirv("DebugFragment" + std::to_string(i), debug_renderer->graphics[i]->fragment_shader);
  }
  BufferUploadArena uploads(4096);
  for (uint32_t iteration = 0; iteration < 2; ++iteration) {
    PlatformLifecycleTestAccess::PreUpdate();
    RenderGraph graph;
    RenderGraphResourceRegistry registry;
    field->Import(graph, registry);
    graph.AddPass({RenderPassNames::sdfgi_maintenance, RenderPassQueue::Graphics, RenderPassScope::Frame},
                  [](const RenderGraphExecutionContext&) {
                  });
    graph.AddPass(field->ClearDescriptor(), [&](const RenderGraphExecutionContext& context) {
      Platform::RecordCommandsMainQueue([&](const VkCommandBuffer command) {
        field->Clear(command, context);
      });
    });
    const auto plan = graph.Compile({});
    ASSERT_TRUE(plan.valid);
    EXPECT_FALSE(plan.uses_compute_queue);
    EXPECT_FALSE(plan.uses_ray_tracing_queue);
    EXPECT_TRUE(plan.allocations.empty());
    graph.Execute(plan, registry);
    Platform::RecordCommandsMainQueue([&](const VkCommandBuffer command) {
      field->OrderAccess(command, VK_PIPELINE_STAGE_2_TRANSFER_BIT, VK_ACCESS_2_TRANSFER_WRITE_BIT);
      for (const auto& [name, texture] : field->textures) {
        if (name != "Atlas" && name != "Occlusion" && name.find(".Light") == std::string::npos)
          continue;
        VkClearColorValue color{};
        color.uint32[0] = name == "Occlusion" ? 0xf0a5 : 0x84020100;
        const VkImageSubresourceRange range{VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, texture.requirement.layers};
        Platform::ClearColorImage(command, *texture.image, color, 1, &range);
      }
    });
    SdfgiCascadeBlock cascade_data{};
    cascade_data.data[7].pad2[3] = 42.5f;
    std::array<SdfgiLight, 2> lights{};
    lights[1].host_photometry[3] = 13.25f;
    BufferUploadBatch upload;
    BufferUploadOptions uniform;
    uniform.usage = BufferUploadUsage::Uniform;
    upload.Add(field->buffers.at("Frame0.Cascades").buffer, cascade_data, uniform);
    SdfgiGatherData gather_data{};
    gather_data.cascades[7].exposure_normalization = 2;
    gather_data.anchor_origin[2] = 3;
    gather_data.generation = 5;
    upload.Add(field->buffers.at("Frame0.Gather").buffer, gather_data, uniform);
    upload.Add(field->buffers.at("Frame0.Cascade0.StaticLights").buffer, lights);
    upload.Record(uploads);
    Platform::RecordCommandsMainQueue([&](const VkCommandBuffer command) {
      field->OrderAccess(command, VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT,
                         VK_ACCESS_2_SHADER_READ_BIT | VK_ACCESS_2_SHADER_WRITE_BIT);
      light_check->Bind(command);
      light_check->BindDescriptorSet(command, 0, field->sets.at("Frame0.Cascade0.StaticLights")->GetVkDescriptorSet());
      SdfgiDirectLightPushConstant light_params{};
      light_params.max_cascades = 8;
      light_check->PushConstant(command, 0, light_params);
      light_check->Dispatch(command, 1);
      field->OrderAccess(command, VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT,
                         VK_ACCESS_2_SHADER_READ_BIT | VK_ACCESS_2_SHADER_WRITE_BIT);
      integrate_check->Bind(command);
      integrate_check->BindDescriptorSet(command, 1, field->sets.at("Sky")->GetVkDescriptorSet());
      for (uint32_t c = 0; c < 4; ++c) {
        integrate_check->BindDescriptorSet(
            command, 0, field->sets.at("Frame0.Cascade" + std::to_string(c) + ".Integrate")->GetVkDescriptorSet());
        SdfgiIntegratePushConstant params{};
        params.history_size = 30;
        params.max_cascades = 4;
        integrate_check->PushConstant(command, 0, params);
        integrate_check->Dispatch(command, 1);
        field->OrderAccess(command, VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT,
                           VK_ACCESS_2_SHADER_READ_BIT | VK_ACCESS_2_SHADER_WRITE_BIT);
      }
      gather_check->Bind(command);
      gather_check->BindDescriptorSet(command, 5, field->sets.at("Frame0.Gather")->GetVkDescriptorSet());
      gather_check->Dispatch(command, 1);
      field->OrderAccess(command, VK_PIPELINE_STAGE_2_TRANSFER_BIT, VK_ACCESS_2_TRANSFER_READ_BIT);
    });
    PlatformLifecycleTestAccess::LateUpdate();
  }
  EXPECT_TRUE(field->initialization_recorded);
  EXPECT_GT(Platform::GetPendingFrameSubmissionCount(), 0u);
  const std::weak_ptr<SdfgiResources> weak_field = field;
  auto retained = field;
  field.reset();
  EXPECT_FALSE(weak_field.expired());
  Platform::WaitForFrameSubmissions("SDFGI focused lifetime test");
  SdfgiFieldStatus status{};
  retained->buffers.at("Status").buffer->Download(status);
  EXPECT_EQ(status.ready, 0u);
  EXPECT_EQ(status.failure_flags, 0u);
  EXPECT_EQ(status.solid_cell_capacity, retained->settings.SolidCellCapacity());
  EXPECT_EQ(status.generation, glm::floatBitsToUint(42.5f));
  retained.reset();
  EXPECT_TRUE(weak_field.expired());

  field = SdfgiResources::TryCreate(reference, host_layouts, failure);
  ASSERT_TRUE(field) << failure;
  EXPECT_FALSE(field->initialization_recorded);
  field.reset();
}

TEST(SdfgiVoxelization, ThreeAxesPayloadCoverageAndRepeatedScratchWithRtDisabled) {
  ScopedGpuPlatform platform(false);
  ApplicationContext::Get().RegisterAsset<Shader>("Shader", {".eveshader", ".slang"});
  const auto render = ApplicationContext::Get().GetLayer<RenderLayer>();
  SdfgiSettings settings;
  settings.voxel_count_x = settings.voxel_count_y = 128;
  settings.probe_spacing_cells = 8;
  settings.cascade_count = 1;
  settings.min_cell_size = 1;
  settings.vertical_scale = SdfgiSettings::VerticalScale::Percent100;
  std::string failure;
  auto field = SdfgiResources::TryCreate(settings, SdfgiTestAccess::HostLayouts(*render), failure);
  ASSERT_TRUE(field) << failure;
  EXPECT_FALSE(Platform::RayTracingEnabled());
  EXPECT_FALSE(Platform::RayQueryEnabled());
  EXPECT_FALSE(Platform::RayAccelerationStructureEnabled());
  std::vector<SdfgiCascade> cascades;
  ASSERT_TRUE(UpdateSdfgiCascades(settings, glm::vec3(0), cascades).empty());
  const auto pending = GetSdfgiPendingRegions(cascades, 1);
  const auto mask_texture = std::make_shared<Texture2D>();
  mask_texture->SetRgbaChannelData({glm::vec4(1, 1, 1, 0), glm::vec4(1)}, {2, 1});
  Platform::GetGpuService().WaitIdle();
  TextureStorage::DeviceSync();
  SdfgiTextureInput texture;
  texture.texture = mask_texture;
  texture.image = mask_texture->GetImage();
  texture.image_view = mask_texture->PeekTexture2DStorage().image_view;
  texture.sampler = mask_texture->PeekTexture2DStorage().sampler;
  texture.mapping.tex_coord = 1;
  texture.mapping.uv_transform[2].x = 0.5f;
  SdfgiContributorRegistry contributors;
  for (uint32_t axis = 0; axis < 3; ++axis)
    for (uint32_t kind = 0; kind < 4; ++kind) {
      SdfgiContributor contributor;
      contributor.id = {axis * 4 + kind + 1, 1};
      contributor.mesh = std::make_shared<Mesh>();
      contributor.mesh->OnCreate();
      const auto right = (axis + 1) % 3, up = (axis + 2) % 3;
      std::vector<Vertex> vertices(4);
      for (uint32_t corner = 0; corner < 4; ++corner) {
        auto& vertex = vertices[corner];
        vertex.position[axis] = 0.5f;
        vertex.position[right] = -52.0f + 12 * kind + (corner & 1 ? 8 : 0);
        vertex.position[up] = corner & 2 ? 4 : -4;
        vertex.normal[axis] = kind == 2 ? -1 : 1;
        vertex.color = glm::vec4(0.5f, 0.5f, 0.5f, 1);
        vertex.tex_coord_1 = glm::vec2(corner & 1 ? 1 : 0, 0.5f);
      }
      VertexAttributes attributes{};
      attributes.normal = attributes.color = true;
      attributes.tex_coord_1 = true;
      const std::vector<glm::uvec3> triangles =
          kind == 2 ? std::vector<glm::uvec3>{{0, 2, 1}, {1, 2, 3}} : std::vector<glm::uvec3>{{0, 1, 2}, {1, 3, 2}};
      contributor.mesh->SetVertices(attributes, vertices, triangles);
      contributor.world_bounds = contributor.mesh->GetBound();
      contributor.material.base_color = glm::vec4(0.8f, 0.4f, 0.2f, 0.1f);
      contributor.material.masked = kind == 1;
      if (kind == 1) {
        contributor.material.base_color.a = 1;
        contributor.material.base_texture = texture;
      }
      contributor.material.double_sided = kind == 2;
      contributor.material.cull_mode = VK_CULL_MODE_BACK_BIT;
      contributor.material.emission = kind == 3 ? glm::vec3(2, 0, 0) : glm::vec3(0);
      contributors.entries.emplace(contributor.id, std::move(contributor));
    }
  std::array<std::shared_ptr<SdfgiVoxelFrame>, 2> frames;
  GeometryStorage::WaitForPendingUploads();
  for (uint32_t iteration = 0; iteration < frames.size(); ++iteration) {
    PlatformLifecycleTestAccess::PreUpdate();
    if (iteration == 1)
      for (auto& [id, contributor] : contributors.entries) {
        contributor.material.masked = false;
        contributor.material.emission = glm::vec3(0);
      }
    auto& frame = frames[iteration];
    ASSERT_NO_THROW(frame = SdfgiVoxelFrame::Create(*field, contributors, cascades, pending));
    frame->debug = std::make_shared<SdfgiVoxelDebug>(0, 64);
    RenderGraph graph;
    RenderGraphResourceRegistry registry;
    field->Import(graph, registry);
    graph.AddPass({RenderPassNames::sdfgi_maintenance, RenderPassQueue::Graphics, RenderPassScope::Frame},
                  [](const RenderGraphExecutionContext&) {
                  });
    if (!field->initialization_recorded)
      graph.AddPass(field->ClearDescriptor(), [field](const RenderGraphExecutionContext& context) {
        Platform::RecordCommandsMainQueue([&](const VkCommandBuffer command) {
          field->Clear(command, context);
        });
      });
    frame->AddPasses(graph, registry, field);
    const auto plan = graph.Compile({});
    ASSERT_TRUE(plan.valid);
    EXPECT_FALSE(plan.uses_compute_queue);
    graph.Execute(plan, registry);
    PlatformLifecycleTestAccess::LateUpdate();
  }
  for (uint32_t iteration = 0; iteration < frames.size(); ++iteration) {
    const auto data = frames[iteration]->debug->Read();
    for (uint32_t axis = 0; axis < 3; ++axis)
      for (uint32_t kind = 0; kind < 4; ++kind) {
        SCOPED_TRACE(::testing::Message() << "frame=" << iteration << " axis=" << axis << " kind=" << kind);
        const uint32_t right = 16 + 12 * kind, up = 65;
        const uint32_t index = axis * 128 * 128 + (axis == 1 ? up + right * 128 : right + up * 128);
        const bool empty = iteration == 0 && kind == 1;
        EXPECT_EQ(data[0][index], empty ? 0u : 1u | (12u << 11) | (6u << 6) | (3u << 1));
        EXPECT_EQ(data[3][index], empty ? 0u : 1u << (axis + (kind == 2 ? 3 : 0)));
        EXPECT_EQ(data[1][index], iteration == 0 && kind == 3 ? (17u << 25) | 128u : 0u);
        EXPECT_EQ(data[2][index], iteration == 0 && kind == 3 ? 31u << (axis * 5) : 0u);
        if (kind == 1) {
          const uint32_t visible_right = 13 + 12 * kind;
          const uint32_t visible = axis * 128 * 128 + (axis == 1 ? up + visible_right * 128 : visible_right + up * 128);
          EXPECT_EQ(data[0][visible], 1u | (12u << 11) | (6u << 6) | (3u << 1));
        }
      }
  }
}

TEST(GpuTimestampFrames, FirstScopeInsideRenderingAfterUntimedFrames) {
  ScopedGpuPlatform platform(false);
  Platform::SetGpuTimestampCaptureEnabled(true);
  for (uint32_t phase = 0; phase < 4; ++phase) {
    PlatformLifecycleTestAccess::PreUpdate();
    Platform::RecordCommandsMainQueue([&](const VkCommandBuffer command) {
      VkRenderingInfo info{VK_STRUCTURE_TYPE_RENDERING_INFO};
      info.renderArea.extent = {1, 1};
      info.layerCount = 1;
      Platform::RecordRenderCommands(info, command, [&]() {
        if (phase % 2) {
          const auto token = Platform::BeginGpuTimestampScope(command, "FirstInsideRendering");
          EXPECT_TRUE(token.valid);
          Platform::EndGpuTimestampScope(command, token);
        }
      });
    });
    PlatformLifecycleTestAccess::LateUpdate();
    Platform::WaitForFrameSubmissions("Focused timestamp render-boundary check");
  }
  const auto stats = Platform::GetGpuTimestampStats();
  ASSERT_EQ(stats.size(), 1u);
  EXPECT_EQ(stats[0].name, "FirstInsideRendering");
  EXPECT_EQ(stats[0].sample_count, 2u);
  Platform::SetGpuTimestampCaptureEnabled(false);
}

TEST(SdfgiEdits, PayloadPreservesTopologyHistoryAndStaticSeedAndRecoversWithoutRt) {
  ScopedGpuPlatform platform(false);
  ApplicationContext::Get().RegisterAsset<Shader>("Shader", {".eveshader", ".slang"});
  EXPECT_FALSE(Platform::RayTracingEnabled());
  EXPECT_FALSE(Platform::RayQueryEnabled());
  EXPECT_FALSE(Platform::RayAccelerationStructureEnabled());
  SdfgiSettings settings;
  settings.voxel_count_x = settings.voxel_count_y = 128;
  settings.probe_spacing_cells = 8;
  settings.cascade_count = 1;
  settings.min_cell_size = 1;
  settings.history_size = 5;
  settings.vertical_scale = SdfgiSettings::VerticalScale::Percent100;
  std::string failure;
  const auto render = ApplicationContext::Get().GetLayer<RenderLayer>();
  auto field = SdfgiResources::TryCreate(settings, SdfgiTestAccess::HostLayouts(*render), failure);
  ASSERT_TRUE(field) << failure;
  auto runtime = std::make_shared<SdfgiRuntime>(settings, SdfgiCapabilityReport{});
  runtime->Maintain(0, {1, glm::vec3(0)});
  SdfgiContributor input;
  input.id = {1, 1};
  input.mesh = std::make_shared<Mesh>();
  input.mesh->OnCreate();
  std::vector<Vertex> vertices(4);
  for (uint32_t i = 0; i < 4; ++i) {
    vertices[i].position = {0.5f, i & 1 ? 4.0f : -4.0f, i & 2 ? 4.0f : -4.0f};
    vertices[i].normal = {1, 0, 0};
    vertices[i].color = glm::vec4(1);
  }
  VertexAttributes attributes{};
  attributes.normal = attributes.color = true;
  input.mesh->SetVertices(attributes, vertices, std::vector<glm::uvec3>{{0, 1, 2}, {1, 3, 2}});
  input.world_bounds = input.mesh->GetBound();
  input.material.cull_mode = VK_CULL_MODE_BACK_BIT;
  GeometryStorage::WaitForPendingUploads();
  SdfgiLightInput light;
  light.id = 1;
  light.dynamic = false;
  light.type = SdfgiLightInput::Type::Point;
  light.position = {10, 0, 0};
  light.range = 30;
  light.color = glm::vec3(2);
  light.world_bounds = {glm::vec3(-30), glm::vec3(30)};
  std::vector<uint8_t> initial_sdf;
  std::vector<uint16_t> initial_occlusion;
  std::vector<SdfgiSolidCell> previous_cells;
  std::shared_ptr<SdfgiPreprocessReadback> failed_readback;
  for (uint32_t phase = 0; phase < 7; ++phase) {
    SCOPED_TRACE(phase);
    PlatformLifecycleTestAccess::PreUpdate();
    runtime->Maintain(Platform::GetFrameCount(), {1, glm::vec3(0)});
    SdfgiSceneSnapshot snapshot;
    if (phase < 5) {
      input.material.base_color = phase ? glm::vec4(0.4f, 0.8f, 0.2f, 1) : glm::vec4(1);
      input.material.emission = phase ? glm::vec3(0.5f) : glm::vec3(0);
      input.material.masked = phase == 3;
      input.material.base_color.a = phase == 3 ? 0 : 1;
      snapshot.contributors.push_back(input);
    }
    runtime->UpdateSceneSnapshot(snapshot);
    runtime->PrepareUpdates(phase != 0, phase == 4 || phase == 5);
    // Force a payload retry and then an intentionally invalid proof to exercise GPU fallback.
    if (phase == 2 || phase == 3 || phase == 6) {
      auto raster = runtime->cascades;
      raster[0].full_redraw = true;
      runtime->pending_regions = GetSdfgiPendingRegions(raster, 1);
      runtime->payload_cascades = 1;
    }
    const auto slot = Platform::GetCurrentFrameIndex();
    const auto voxel =
        SdfgiVoxelFrame::Create(*field, runtime->contributors, runtime->cascades, runtime->pending_regions);
    voxel->payload_cascades = runtime->payload_cascades;
    voxel->reset_failure = phase == 4;
    field->voxel_frames[slot] = voxel;
    RenderGraph graph;
    RenderGraphResourceRegistry registry;
    field->Import(graph, registry);
    graph.AddPass({RenderPassNames::sdfgi_maintenance, RenderPassQueue::Graphics, RenderPassScope::Frame},
                  [](const RenderGraphExecutionContext&) {
                  });
    if (phase == 0)
      graph.AddPass(field->ClearDescriptor(), [&](const RenderGraphExecutionContext& context) {
        Platform::RecordCommandsMainQueue([&](const VkCommandBuffer command) {
          field->Clear(command, context);
        });
      });
    voxel->AddPasses(graph, registry, field, runtime);
    auto lighting = SdfgiLightFrame::Create(*field, runtime->cascades, {light}, Platform::GetFrameCount(), 1);
    field->light_frames[slot] = lighting;
    lighting->AddPasses(graph, field, "SdfgiVoxelComplete");
    const auto plan = graph.Compile({});
    ASSERT_TRUE(plan.valid);
    EXPECT_FALSE(plan.uses_compute_queue);
    graph.Execute(plan, registry);
    if (phase == 0)
      Platform::RecordCommandsMainQueue([&](const VkCommandBuffer command) {
        field->OrderAccess(command, VK_PIPELINE_STAGE_2_TRANSFER_BIT, VK_ACCESS_2_TRANSFER_WRITE_BIT);
        VkClearColorValue clear{};
        for (auto& value : clear.int32)
          value = 123;
        const VkImageSubresourceRange range{VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 5};
        Platform::ClearColorImage(command, *field->textures.at("Cascade0.History").image, clear, 1, &range);
        field->OrderAccess(command, VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT, VK_ACCESS_2_SHADER_READ_BIT);
      });
    PlatformLifecycleTestAccess::LateUpdate();
    Platform::WaitForFrameSubmissions("SDFGI focused material edit readback");
    voxel->preprocess_readback->ReadAfterFrameFence(*field);
    EXPECT_EQ(field->preprocess_status.failure_flags, phase == 3 ? kSdfgiFailurePayloadCoverage : 0u);
    EXPECT_EQ(runtime->pending_changes[0], 0u);
    EXPECT_EQ(field->geometry_update_count, phase < 4 ? 1u : std::min(phase - 2, 3u));
    EXPECT_EQ(field->payload_update_count, phase == 6 ? 4u : std::min(phase, 3u));
    if (phase == 3)
      failed_readback = voxel->preprocess_readback;
    if (phase == 4) {
      failed_readback->consumed = false;
      failed_readback->ReadAfterFrameFence(*field);
      EXPECT_EQ(field->preprocess_status.failure_flags, 0u);
    }
    VkBufferImageCopy copy{};
    copy.imageSubresource = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 0, 1};
    copy.imageExtent = {128, 128, 128};
    Buffer sdf_buffer(128 * 128 * 128);
    sdf_buffer.CopyFromImage(*field->textures.at("Cascade0.Sdf").image, copy);
    std::vector<uint8_t> sdf;
    sdf_buffer.DownloadVector(sdf, 128 * 128 * 128);
    copy.imageExtent.width = 256;
    Buffer occlusion_buffer(256 * 128 * 128 * 2);
    occlusion_buffer.CopyFromImage(*field->textures.at("Occlusion").image, copy);
    std::vector<uint16_t> occlusion;
    occlusion_buffer.DownloadVector(occlusion, 256 * 128 * 128);
    if (phase == 0 || phase == 5) {
      initial_sdf = sdf;
      initial_occlusion = occlusion;
    } else if (phase < 4 || phase == 6) {
      EXPECT_EQ(sdf, initial_sdf);
      EXPECT_EQ(occlusion, initial_occlusion);
    }
    copy.imageExtent = {1, 1, 1};
    copy.imageSubresource.layerCount = 5;
    Buffer history_buffer(5 * 8);
    history_buffer.CopyFromImage(*field->textures.at("Cascade0.History").image, copy);
    std::vector<int16_t> history;
    history_buffer.DownloadVector(history, 20);
    EXPECT_EQ(history, std::vector<int16_t>(20, 123));
    const auto count = field->solid_cell_dispatch[0].total_count;
    ASSERT_GT(count, 0u);
    std::vector<SdfgiSolidCell> cells;
    field->buffers.at("Cascade0.SolidCells").buffer->DownloadVector(cells, count);
    if (phase >= 5) {
      ASSERT_EQ(count, 1u);  // Matches Godot's empty-field origin cell, not retained geometry.
      EXPECT_EQ(cells[0].position & 0x1fffffu, 0u);
      EXPECT_EQ(cells[0].albedo & 0x1fffffu, 0u);
      EXPECT_EQ(cells[0].light & 0x3fffffffu, 0u);
      EXPECT_EQ(cells[0].light_aniso & 0x3fffffffu, 0u);
      continue;
    }
    if (phase == 1 || phase == 2) {
      ASSERT_EQ(cells.size(), previous_cells.size());
      for (size_t i = 0; i < cells.size(); ++i) {
        EXPECT_EQ(cells[i].position, previous_cells[i].position);
        EXPECT_EQ(cells[i].albedo & 0xffff8000u, previous_cells[i].albedo & 0xffff8000u);
        if (phase == 1)
          EXPECT_NE(cells[i].albedo & 0x7fff, previous_cells[i].albedo & 0x7fff);
        if (phase == 2)
          EXPECT_EQ(std::memcmp(&cells[i], &previous_cells[i], sizeof(SdfgiSolidCell)), 0);
      }
    }
    previous_cells = std::move(cells);
  }
}

TEST(SdfgiEdits, NonStaticSelectionMovementAndRemovalWithoutRt) {
  TempProject project;
  Application app;
  struct Terminate {
    Application& app;
    ~Terminate() {
      app.Terminate();
    }
  } terminate{app};
  app.PushLayer<RenderLayer>("Render Layer");
  auto initialization = TestApplicationSettings(project);
  initialization.load_default_resources = true;
  app.Initialize(initialization);
  EXPECT_FALSE(Platform::RayTracingEnabled());
  EXPECT_FALSE(Platform::RayQueryEnabled());
  EXPECT_FALSE(Platform::RayAccelerationStructureEnabled());
  const auto scene = AssetManager::CreateTemporaryAsset<Scene>();
  const auto entity = scene->CreateEntity("Non-static contributor");
  const auto renderer = scene->GetOrSetPrivateComponent<MeshRenderer>(entity).lock();
  const auto mesh = AssetManager::CreateTemporaryAsset<Mesh>();
  renderer->mesh = mesh;
  renderer->material = AssetManager::CreateTemporaryAsset<Material>();
  std::vector<Vertex> vertices(4);
  for (uint32_t i = 0; i < 4; ++i) {
    vertices[i].position = {0.5f, i & 1 ? 4.0f : -4.0f, i & 2 ? 4.0f : -4.0f};
    vertices[i].normal = {1, 0, 0};
    vertices[i].color = glm::vec4(1);
  }
  VertexAttributes attributes{};
  attributes.normal = attributes.color = true;
  mesh->SetVertices(attributes, vertices, std::vector<glm::uvec3>{{0, 1, 2}, {1, 3, 2}});
  GeometryStorage::WaitForPendingUploads();
  ResolvedEnvironmentalLighting lighting;
  auto& settings = lighting.sdfgi_settings;
  settings.voxel_count_x = settings.voxel_count_y = 64;
  settings.cascade_count = 1;
  settings.min_cell_size = 1;
  settings.history_size = 5;
  settings.vertical_scale = SdfgiSettings::VerticalScale::Percent100;
  const auto render = ApplicationContext::Get().GetLayer<RenderLayer>();
  std::string failure;
  auto field = SdfgiResources::TryCreate(settings, SdfgiTestAccess::HostLayouts(*render), failure);
  ASSERT_TRUE(field) << failure;
  auto runtime = std::make_shared<SdfgiRuntime>(settings, SdfgiCapabilityReport{});
  std::vector<uint8_t> empty_sdf, previous_sdf;
  for (uint32_t phase = 0; phase < 5; ++phase) {
    SCOPED_TRACE(phase);
    PlatformLifecycleTestAccess::PreUpdate();
    settings.static_entities_only = phase == 0 || phase == 4;
    runtime->settings = settings;
    runtime->Maintain(Platform::GetFrameCount(), {1, glm::vec3(0)});
    GlobalTransform transform;
    transform.SetPosition({phase >= 2 ? 12.0f : 0.0f, 0, 0});
    scene->SetDataComponent(entity, transform);
    runtime->UpdateSceneSnapshot(SnapshotSdfgiScene(scene, lighting));
    runtime->PrepareUpdates(phase != 0, false);
    EXPECT_FALSE(scene->IsEntityStatic(entity));
    if (phase == 3) {
      EXPECT_TRUE(runtime->contributors.changes.empty());
      EXPECT_TRUE(runtime->pending_regions.empty());
      PlatformLifecycleTestAccess::LateUpdate();
      continue;
    }
    EXPECT_EQ(runtime->contributors.entries.size(), settings.static_entities_only ? 0u : 1u);
    const auto voxel =
        SdfgiVoxelFrame::Create(*field, runtime->contributors, runtime->cascades, runtime->pending_regions);
    field->voxel_frames[Platform::GetCurrentFrameIndex()] = voxel;
    RenderGraph graph;
    RenderGraphResourceRegistry registry;
    field->Import(graph, registry);
    graph.AddPass({RenderPassNames::sdfgi_maintenance, RenderPassQueue::Graphics, RenderPassScope::Frame},
                  [](const RenderGraphExecutionContext&) {
                  });
    if (phase == 0)
      graph.AddPass(field->ClearDescriptor(), [&](const RenderGraphExecutionContext& context) {
        Platform::RecordCommandsMainQueue([&](const VkCommandBuffer command) {
          field->Clear(command, context);
        });
      });
    voxel->AddPasses(graph, registry, field, runtime);
    const auto plan = graph.Compile({});
    ASSERT_TRUE(plan.valid);
    EXPECT_FALSE(plan.uses_compute_queue);
    graph.Execute(plan, registry);
    PlatformLifecycleTestAccess::LateUpdate();
    Platform::WaitForFrameSubmissions("SDFGI contributor selection readback");
    voxel->preprocess_readback->ReadAfterFrameFence(*field);
    EXPECT_EQ(field->preprocess_status.failure_flags, 0u);
    VkBufferImageCopy copy{};
    copy.imageSubresource = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 0, 1};
    copy.imageExtent = {64, 64, 64};
    Buffer buffer(64 * 64 * 64);
    buffer.CopyFromImage(*field->textures.at("Cascade0.Sdf").image, copy);
    std::vector<uint8_t> sdf;
    buffer.DownloadVector(sdf, 64 * 64 * 64);
    if (phase == 0)
      empty_sdf = sdf;
    else if (phase == 4)
      EXPECT_EQ(sdf, empty_sdf);
    else {
      EXPECT_NE(sdf, empty_sdf);
      EXPECT_NE(sdf, previous_sdf);
    }
    previous_sdf = std::move(sdf);
  }
  EXPECT_EQ(field->geometry_update_count, 4u);
}

TEST(SdfgiScene, CornellContributesWithoutChangingAuthoredStaticFlags) {
  TempProject project;
  Application app;
  struct Terminate {
    Application& app;
    ~Terminate() {
      app.Terminate();
    }
  } terminate{app};
  app.PushLayer<RenderLayer>("Render Layer");
  auto settings = TestApplicationSettings(project);
  settings.load_default_resources = true;
  app.Initialize(settings);
  const auto scene = AssetManager::CreateTemporaryAsset<Scene>();
  ConfigureDdgiValidationFixture(scene, "cornell");
  ResolvedEnvironmentalLighting lighting;
  SdfgiContributorRegistry registry;
  registry.Update(SnapshotSdfgiScene(scene, lighting).contributors);
  ASSERT_EQ(registry.entries.size(), 8u);
  const auto owners = scene->UnsafeGetPrivateComponentOwnersList<MeshRenderer>();
  ASSERT_TRUE(owners);
  for (const auto owner : *owners)
    EXPECT_FALSE(scene->IsEntityStatic(owner));
  lighting.sdfgi_settings.static_entities_only = true;
  registry.Update(SnapshotSdfgiScene(scene, lighting).contributors);
  EXPECT_TRUE(registry.entries.empty());
  EXPECT_EQ(registry.changes.size(), 8u);
  EXPECT_FLOAT_EQ(lighting.sdfgi_settings.min_cell_size, 0.1f);
}

TEST(SdfgiRelocation, ClearanceHistoryGeometryAndSignedScrollWithoutRt) {
  ScopedGpuPlatform platform(false);
  ApplicationContext::Get().RegisterAsset<Shader>("Shader", {".eveshader", ".slang"});
  EXPECT_FALSE(Platform::RayTracingEnabled());
  EXPECT_FALSE(Platform::RayQueryEnabled());
  EXPECT_FALSE(Platform::RayAccelerationStructureEnabled());
  SdfgiSettings settings;
  settings.voxel_count_x = settings.voxel_count_y = 64;
  settings.cascade_count = 2;
  settings.history_size = 5;
  settings.probe_relocation = true;
  settings.probe_spacing_cells = 4;
  settings.bounce_feedback = 0;
  const auto layout = SdfgiProbeLayout::Create(64, 64, 4, 32768);
  const auto render = ApplicationContext::Get().GetLayer<RenderLayer>();
  std::string failure;
  auto field = SdfgiResources::TryCreate(settings, SdfgiTestAccess::HostLayouts(*render), failure);
  ASSERT_TRUE(field) << failure;
  auto seed = std::make_shared<ComputePipeline>();
  seed->descriptor_set_layouts = {field->layouts[static_cast<size_t>(SdfgiLayout::Store)]};
  seed->push_constant_ranges = {{VK_SHADER_STAGE_COMPUTE_BIT, 0, 4}};
  seed->compute_shader = Shader::CreateTemporary(ShaderType::Compute, std::string(R"(
[[vk::binding(7,0)]] [vk::image_format("r8")] RWTexture3D<float> sdf;
[[vk::push_constant]] ConstantBuffer<uint> phase;
[numthreads(8,8,8)]
void main(uint3 id : SV_DispatchThreadID) {
  if (phase == 8) {
    float d = min(min(float(id.x), float(id.y)), abs(float(id.x) - 32));
    sdf[id] = d == 0 ? 0 : (1 + d) / 255.0;
    return;
  }
  sdf[id] = phase == 2 ? 0.0 : phase >= 3 ? 1.0 :
      (floor(max(0.0, abs(float(id.x) + 0.5 - 32.0) - 0.5)) + 1.0) / 255.0;
}
)"));
  seed->Initialize();
  ASSERT_TRUE(seed->Initialized());
  auto output_layout = std::make_shared<DescriptorSetLayout>();
  output_layout->PushDescriptorBinding(0, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_COMPUTE_BIT, 0);
  output_layout->Initialize();
  VkBufferCreateInfo info{VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO};
  info.size = 10 * sizeof(glm::vec4);
  info.usage = VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_SRC_BIT;
  auto output = std::make_shared<Buffer>(info);
  auto output_set = std::make_shared<DescriptorSet>(output_layout);
  output_set->UpdateBufferDescriptorBinding(0, output);
  auto query = std::make_shared<ComputePipeline>();
  query->descriptor_set_layouts = {field->layouts[static_cast<size_t>(SdfgiLayout::Integrate)], output_layout};
  query->compute_shader = Shader::CreateTemporary(ShaderType::Compute, std::string(R"(
import EvoEngine.SdfgiRelocation;
import EvoEngine.SdfgiTypes;
[[vk::binding(1,0)]] Texture3D<float> sdf[8];
[[vk::binding(6,0)]] SamplerState linear_sampler;
[[vk::binding(8,0)]] [vk::image_format("r32ui")] RWTexture2DArray<uint> atlas;
[[vk::binding(9,0)]] [vk::image_format("rgba16i")] RWTexture2DArray<int4> history;
[[vk::binding(10,0)]] [vk::image_format("rgba32i")] RWTexture2D<int4> average;
[[vk::binding(16,0)]] StructuredBuffer<float4> placements;
[[vk::binding(0,1)]] RWStructuredBuffer<float4> result;
[numthreads(1,1,1)]
void main(uint3 id : SV_DispatchThreadID) {
  uint width, height; average.GetDimensions(width, height);
  int2 p = SdfgiProbeTexel(int3(8), 17, width);
  result[0] = float4(average[int2(p.x,p.y*16)].x, history[int3(p.x,p.y*16,0)].x,
      atlas[int3(p*8,0)], atlas[int3(p*8,2)]);
  result[1] = float4(SdfgiSegmentVisibility(sdf[0],linear_sampler,float3(16,32,32),float3(48,32,32),64),
      SdfgiSegmentVisibility(sdf[0],linear_sampler,float3(16,32,32),float3(24,32,32),64),
      SdfgiSegmentVisibility(sdf[0],linear_sampler,float3(16,32,32),float3(64,32,32),64),0);
  result[8] = float4(
      SdfgiSegmentVisibility(sdf[1],linear_sampler,float3(8.5),float3(1.6,0.6,8.5),64,float3(1,0,0)),
      SdfgiSegmentVisibility(sdf[1],linear_sampler,float3(8.5),float3(0.6,8.5,8.5),64,float3(1,0,0)),
      SdfgiSegmentVisibility(sdf[1],linear_sampler,float3(40,8.5,8.5),float3(1.6,0.6,8.5),64,float3(1,0,0)),
      SdfgiSegmentVisibility(sdf[1],linear_sampler,float3(0.2,8.5,8.5),float3(1.6,8.5,8.5),64,float3(1,0,0)));
  result[9] = float4(
      SdfgiSegmentVisibility(sdf[1],linear_sampler,float3(1.6,8.5,8.5),float3(1.6,0.6,8.5),64,float3(1,0,0)),
      SdfgiSegmentVisibility(sdf[1],linear_sampler,float3(31,8.5,8.5),float3(33.6,8.5,8.5),64,float3(-1,0,0)),
      SdfgiSegmentVisibility(sdf[1],linear_sampler,float3(32.5,16.5,8.5),float3(32.5,8.5,8.5),64,float3(0,1,0)),0);
  for (uint i=0; i<4; ++i) {
    float spacing = float(1u << i);
    float4 p = SdfgiFindPlacement(sdf[0],linear_sampler,float3(32),64,spacing);
    result[2+i] = float4(length(p.xyz),p.w,
        SdfgiClearance(sdf[0],linear_sampler,float3(32)+p.xyz*spacing,64),
        all(p == SdfgiFindPlacement(sdf[0],linear_sampler,float3(32),64,spacing)) ? 1.0 : 0.0);
  }
}
)"));
  query->Initialize();
  ASSERT_TRUE(query->Initialized());
  auto gather = std::make_shared<ComputePipeline>();
  gather->descriptor_set_layouts = SdfgiTestAccess::HostLayouts(*render);
  gather->descriptor_set_layouts[4] = output_layout;
  gather->descriptor_set_layouts.push_back(field->layouts[static_cast<size_t>(SdfgiLayout::Gather)]);
  gather->compute_shader = Shader::CreateTemporary(ShaderType::Compute, std::string(R"(
import EvoEngine.Sdfgi;
[[vk::binding(0,4)]] RWStructuredBuffer<float4> result;
[numthreads(1,1,1)]
void main(uint3 id : SV_DispatchThreadID) {
  EeSdfgiLighting value = EE_SDFGI_GATHER(float3(-2,-1,-2),float3(1,0,0),float3(0,1,0),0.5);
  result[6] = float4(value.diffuse,value.weight);
  result[7] = float4(value.specular,all(isfinite(value.specular)) && all(isfinite(value.diffuse)) ? 1.0 : 0.0);
}
)"));
  gather->Initialize();
  ASSERT_TRUE(gather->Initialized());
  std::vector<SdfgiCascade> cascades;
  ASSERT_TRUE(UpdateSdfgiCascades(settings, glm::vec3(0), cascades).empty());
  BufferUploadArena uploads;
  std::vector<glm::vec4> previous;
  for (uint32_t phase = 0; phase < 8; ++phase) {
    SCOPED_TRACE(phase);
    PlatformLifecycleTestAccess::PreUpdate();
    const auto slot = Platform::GetCurrentFrameIndex();
    BufferUploadBatch upload;
    const auto metadata = BuildSdfgiGatherData(settings, cascades, glm::vec3(0), 1);
    upload.Add(field->buffers.at("Frame" + std::to_string(slot) + ".Gather").buffer, metadata,
               {BufferUploadUsage::Uniform});
    RenderGraph graph;
    RenderGraphResourceRegistry registry;
    field->Import(graph, registry);
    graph.AddPass({RenderPassNames::sdfgi_maintenance, RenderPassQueue::Graphics, RenderPassScope::Frame},
                  [](const RenderGraphExecutionContext&) {
                  });
    if (phase == 0)
      graph.AddPass(field->ClearDescriptor(), [&](const RenderGraphExecutionContext& context) {
        Platform::RecordCommandsMainQueue([&](VkCommandBuffer command) {
          field->Clear(command, context);
        });
      });
    graph.AddPass(
        {"Relocate",
         RenderPassQueue::Graphics,
         RenderPassScope::Frame,
         {},
         {phase == 0 ? "SdfgiInitialize" : RenderPassNames::sdfgi_maintenance}},
        [&](const RenderGraphExecutionContext&) {
          upload.Record(uploads);
          Platform::RecordCommandsMainQueue([&](VkCommandBuffer command) {
            field->OrderAccess(command, VK_PIPELINE_STAGE_2_TRANSFER_BIT, VK_ACCESS_2_TRANSFER_WRITE_BIT);
            field->buffers.at("Status").buffer->Fill(command, offsetof(SdfgiFieldStatus, ready), 4, 1);
            field->buffers.at("Status").buffer->Fill(command, offsetof(SdfgiFieldStatus, generation), 4, 1);
            for (const auto& name : {"Cascade0.History", "Cascade0.Average", "Cascade1.Average", "Atlas"}) {
              const auto& texture = field->textures.at(name);
              VkClearColorValue value{};
              value.int32[0] = std::string(name) == "Cascade0.Average" ? 35 : 7;
              const VkImageSubresourceRange range{VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, texture.requirement.layers};
              Platform::ClearColorImage(command, *texture.image, value, 1, &range);
            }
            field->OrderAccess(command, VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT,
                               VK_ACCESS_2_SHADER_READ_BIT | VK_ACCESS_2_SHADER_WRITE_BIT);
            seed->Bind(command);
            seed->BindDescriptorSet(command, 0, field->sets.at("Cascade1.Store")->GetVkDescriptorSet());
            seed->PushConstant(command, 0, 8u);
            seed->Dispatch(command, 8, 8, 8);
            seed->BindDescriptorSet(command, 0, field->sets.at("Cascade0.Store")->GetVkDescriptorSet());
            seed->PushConstant(command, 0, phase);
            seed->Dispatch(command, 8, 8, 8);
            if (phase != 4 && phase != 6) {
              RecordSdfgiProbeRelocation(command, *field, 0, glm::ivec3(phase == 5 ? 4 : phase == 7 ? -4 : 0, 0, 0));
            } else {
              RecordSdfgiScroll(command, *field, 0, glm::ivec3(0), glm::ivec3(phase == 4 ? 4 : -4, 0, 0), slot);
            }
            query->Bind(command);
            query->BindDescriptorSet(
                command, 0,
                field->sets.at("Frame" + std::to_string(slot) + ".Cascade0.Integrate")->GetVkDescriptorSet());
            query->BindDescriptorSet(command, 1, output_set->GetVkDescriptorSet());
            query->Dispatch(command, 1, 1, 1);
            field->OrderAccess(command, VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT,
                               VK_ACCESS_2_SHADER_READ_BIT | VK_ACCESS_2_SHADER_WRITE_BIT);
            gather->Bind(command);
            gather->BindDescriptorSet(command, 4, output_set->GetVkDescriptorSet());
            gather->BindDescriptorSet(command, 5,
                                      field->sets.at("Frame" + std::to_string(slot) + ".Gather")->GetVkDescriptorSet());
            gather->Dispatch(command, 1, 1, 1);
          });
        });
    const auto plan = graph.Compile({});
    ASSERT_TRUE(plan.valid);
    graph.Execute(plan, registry);
    PlatformLifecycleTestAccess::LateUpdate();
    Platform::WaitForFrameSubmissions("SDFGI relocation fixture");
    std::vector<glm::vec4> placements, result;
    field->buffers.at("ProbePlacement").buffer->DownloadVector(placements, layout.ProbeCount() * 2);
    output->DownloadVector(result, 10);
    EXPECT_EQ(result[8], glm::vec4(1, 1, 0, 0));
    EXPECT_EQ(result[9], glm::vec4(1, 0, 0, 0));
    EXPECT_EQ(result[1].y, phase == 2 ? 0 : 1);
    if (phase == 4 || phase == 6) {
      const auto index = layout.Index(phase == 4 ? 0 : 16, 8, 8);
      VkBufferImageCopy copy{};
      copy.imageSubresource = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 0, 1};
      copy.imageOffset = {static_cast<int32_t>(index % layout.columns),
                          static_cast<int32_t>(index / layout.columns * 16), 0};
      copy.imageExtent = {1, 1, 1};
      Buffer readback(sizeof(glm::ivec4));
      readback.CopyFromImage(*field->textures.at("Cascade0.Average").image, copy);
      glm::ivec4 value;
      readback.Download(value);
      EXPECT_EQ(value, glm::ivec4(0));
    }
    EXPECT_EQ(result[6].w, phase == 2 ? 0 : 1);
    EXPECT_EQ(result[7].w, 1);
    EXPECT_EQ(result[0], phase == 1 || phase >= 4 ? glm::vec4(35, 7, 7, 7) : glm::vec4(0));
    if (phase == 0) {
      EXPECT_EQ(result[1], glm::vec4(0, 1, 0, 0));
      for (uint32_t i = 0; i < 4; ++i) {
        EXPECT_LE(result[2 + i].x, 0.45001f);
        EXPECT_EQ(result[2 + i].w, 1);
        if (result[2 + i].y > 0)
          EXPECT_GE(result[2 + i].z, std::min(1.0f, 0.25f * (1u << i)));
      }
      EXPECT_EQ(placements[layout.Index(8, 8, 8)].w, 1);
      EXPECT_GT(glm::length(glm::vec3(placements[layout.Index(8, 8, 8)])), 0);
    }
    for (uint32_t y = 0; y < 17; ++y)
      for (uint32_t z = 0; z < 17; ++z)
        for (uint32_t x = 0; x < 17; ++x) {
          const auto index = layout.Index(x, y, z);
          const auto p = placements[index];
          if (phase == 1)
            EXPECT_EQ(p, previous[index]);
          if (phase == 2)
            EXPECT_EQ(p.w, -1);
          if (phase == 4 || phase == 6) {
            const int source_x = int(x) + (phase == 4 ? -1 : 1);
            EXPECT_EQ(p, source_x >= 0 && source_x < 17 ? previous[layout.Index(source_x, y, z)] : glm::vec4(0));
          }
          if (p.w > 0 && phase != 4 && phase != 6) {
            EXPECT_LE(glm::length(glm::vec3(p)), 0.45001f);
            const auto position = (glm::vec3(x, y, z) + glm::vec3(p)) * 4.0f;
            EXPECT_TRUE(glm::all(glm::greaterThanEqual(position, glm::vec3(0))));
            EXPECT_TRUE(glm::all(glm::lessThan(position, glm::vec3(64))));
          }
        }
    previous = std::move(placements);
  }
}

TEST(SdfgiGather, PublicationWeightsCoverageAndTwoCameraGraphsWithoutRt) {
  ScopedGpuPlatform platform(false);
  ApplicationContext::Get().RegisterAsset<Shader>("Shader", {".eveshader", ".slang"});
  EXPECT_FALSE(Platform::RayTracingEnabled());
  EXPECT_FALSE(Platform::RayQueryEnabled());
  EXPECT_FALSE(Platform::RayAccelerationStructureEnabled());
  SdfgiSettings settings;
  settings.voxel_count_x = settings.voxel_count_y = 128;
  settings.probe_spacing_cells = 8;
  settings.cascade_count = 2;
  settings.min_cell_size = 1;
  settings.vertical_scale = SdfgiSettings::VerticalScale::Percent100;
  settings.normal_bias = 0;
  auto runtime = std::make_shared<SdfgiRuntime>(settings, SdfgiCapabilityReport{});
  ASSERT_TRUE(UpdateSdfgiCascades(settings, glm::vec3(0), runtime->cascades).empty());
  const auto render = ApplicationContext::Get().GetLayer<RenderLayer>();
  std::string failure;
  auto field = SdfgiResources::TryCreate(settings, SdfgiTestAccess::HostLayouts(*render), failure);
  ASSERT_TRUE(field) << failure;
  runtime->resources = field;
  auto seed = std::make_shared<ComputePipeline>();
  seed->descriptor_set_layouts = {field->layouts[static_cast<size_t>(SdfgiLayout::Integrate)]};
  seed->push_constant_ranges = {{VK_SHADER_STAGE_COMPUTE_BIT, 0, 4}};
  seed->compute_shader = Shader::CreateTemporary(ShaderType::Compute, std::string(R"(
[[vk::binding(8,0)]] [vk::image_format("r32ui")] RWTexture2DArray<uint> atlas;
[[vk::push_constant]] ConstantBuffer<uint> gradient;
[numthreads(8,8,1)]
void main(uint3 id : SV_DispatchThreadID) {
  if (id.x >= 2312 || id.y >= 136) return;
  uint value = gradient != 0 ? (id.x / 8 % 17) + 3 * (id.y / 8) + 5 * (id.x / 136) + 1 : (id.z % 2 == 0 ? 1 : 3);
  if (id.z >= 2) value *= 2;
  atlas[id] = value | (value << 9) | (value << 18) | (24u << 27);
}
)"));
  seed->Initialize();
  ASSERT_TRUE(seed->Initialized());
  auto wall = std::make_shared<ComputePipeline>();
  wall->descriptor_set_layouts = {field->layouts[static_cast<size_t>(SdfgiLayout::Store)]};
  wall->push_constant_ranges = {{VK_SHADER_STAGE_COMPUTE_BIT, 0, 4}};
  wall->compute_shader = Shader::CreateTemporary(ShaderType::Compute, std::string(R"(
[[vk::binding(7,0)]] [vk::image_format("r8")] RWTexture3D<float> sdf;
[[vk::push_constant]] ConstantBuffer<float> cell_size;
[numthreads(8,8,8)]
void main(uint3 id : SV_DispatchThreadID) {
  float distance = abs((float(id.z) + 0.5 - 64.0) * cell_size) - 85.0;
  sdf[id] = distance >= 0 ? 0.0 : min(255.0, floor(-distance / cell_size) + 1.0) / 255.0;
}
)"));
  wall->Initialize();
  ASSERT_TRUE(wall->Initialized());
  auto output_layout = std::make_shared<DescriptorSetLayout>();
  output_layout->PushDescriptorBinding(0, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_COMPUTE_BIT, 0);
  output_layout->Initialize();
  VkBufferCreateInfo output_info{VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO};
  output_info.size = 44 * sizeof(glm::vec4);
  output_info.usage = VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_SRC_BIT;
  auto output = std::make_shared<Buffer>(output_info);
  auto output_set = std::make_shared<DescriptorSet>(output_layout);
  output_set->UpdateBufferDescriptorBinding(0, output);
  auto gather = std::make_shared<ComputePipeline>();
  gather->descriptor_set_layouts = SdfgiTestAccess::HostLayouts(*render);
  gather->descriptor_set_layouts[4] = output_layout;
  gather->descriptor_set_layouts.push_back(field->layouts[static_cast<size_t>(SdfgiLayout::Gather)]);
  gather->push_constant_ranges = {{VK_SHADER_STAGE_COMPUTE_BIT, 0, 4}};
  gather->compute_shader = Shader::CreateTemporary(ShaderType::Compute, std::string(R"(
import EvoEngine.Sdfgi;
[[vk::binding(0,4)]] RWStructuredBuffer<float4> result;
[[vk::push_constant]] ConstantBuffer<uint> camera;
[numthreads(1,1,1)]
void main(uint3 id : SV_DispatchThreadID) {
  const float3 positions[11] = {float3(0), float3(52,0,0), float3(112,0,0), float3(140,0,0), float3(2.4,3.2,4.8),
      float3(4,0,0), float3(4,0,0), float3(4,0,0), float3(4,0,0), float3(80,0,0), float3(124,0,0)};
  const float roughness[11] = {0.5, 0.75, 0.5, 0.5, 0.5, 0, 0.1, 0.1999, 0.2, 0, 0.1};
  EeSdfgiLighting value = EE_SDFGI_GATHER(positions[id.x], float3(1,0,0), float3(0,0,camera == 0 ? 1 : -1), roughness[id.x]);
  result[camera * 22 + id.x * 2] = float4(value.diffuse, value.weight);
  result[camera * 22 + id.x * 2 + 1] = float4(value.specular, value.specular_weight);
}
)"));
  gather->Initialize();
  ASSERT_TRUE(gather->Initialized());
  for (uint32_t phase = 0; phase < 7; ++phase) {
    SCOPED_TRACE(phase);
    PlatformLifecycleTestAccess::PreUpdate();
    const uint32_t slot = Platform::GetCurrentFrameIndex(), generation = phase + 1;
    runtime->settings.use_occlusion = phase == 2;
    runtime->published = false;
    auto publication = SdfgiGatherFrame::Create(*runtime, generation);
    field->gather_frames[slot] = publication;
    RenderGraph scene_graph;
    RenderGraphResourceRegistry scene_registry;
    field->Import(scene_graph, scene_registry);
    scene_graph.AddPass({RenderPassNames::sdfgi_maintenance, RenderPassQueue::Graphics, RenderPassScope::Frame},
                        [](const RenderGraphExecutionContext&) {
                        });
    if (phase == 0)
      scene_graph.AddPass(field->ClearDescriptor(), [&](const RenderGraphExecutionContext& context) {
        Platform::RecordCommandsMainQueue([&](const VkCommandBuffer command) {
          field->Clear(command, context);
        });
      });
    scene_graph.AddPass(
        {"SdfgiProbeStore",
         RenderPassQueue::Graphics,
         RenderPassScope::Frame,
         {},
         {phase == 0 ? "SdfgiInitialize" : RenderPassNames::sdfgi_maintenance}},
        [&](const RenderGraphExecutionContext&) {
          Platform::RecordCommandsMainQueue([&](const VkCommandBuffer command) {
            field->OrderAccess(command, VK_PIPELINE_STAGE_2_TRANSFER_BIT, VK_ACCESS_2_TRANSFER_WRITE_BIT);
            field->buffers.at("Status").buffer->Fill(command, offsetof(SdfgiFieldStatus, ready), 4, 0);
            field->buffers.at("Status").buffer->Fill(command, offsetof(SdfgiFieldStatus, failure_flags), 4,
                                                     phase == 4 ? 1 : 0);
            field->buffers.at("Status").buffer->Fill(command, offsetof(SdfgiFieldStatus, generation), 4,
                                                     phase == 3 ? generation - 1 : generation);
            const VkImageSubresourceRange range{VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1};
            VkClearColorValue clear{};
            clear.uint32[0] = 0x0248;
            Platform::ClearColorImage(command, *field->textures.at("Occlusion").image, clear, 1, &range);
            for (uint32_t c = 0; c < 2; ++c) {
              const auto name = "Cascade" + std::to_string(c) + ".";
              clear = {};
              clear.float32[0] = phase == 5 ? 1.0f : 0.0f;
              Platform::ClearColorImage(command, *field->textures.at(name + "Sdf").image, clear, 1, &range);
              const uint32_t value = c == 0 ? 8 : 16;
              clear.uint32[0] = value | (value << 9) | (value << 18) | (24u << 27);
              Platform::ClearColorImage(command, *field->textures.at(name + "Light").image, clear, 1, &range);
            }
            field->OrderAccess(command, VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT, VK_ACCESS_2_SHADER_WRITE_BIT);
            if (phase == 6) {
              wall->Bind(command);
              for (uint32_t c = 0; c < 2; ++c) {
                wall->BindDescriptorSet(command, 0,
                                        field->sets.at("Cascade" + std::to_string(c) + ".Store")->GetVkDescriptorSet());
                wall->PushConstant(command, 0, c == 0 ? 1.0f : 2.0f);
                wall->Dispatch(command, 16, 16, 16);
              }
            }
            seed->Bind(command);
            seed->BindDescriptorSet(
                command, 0,
                field->sets.at("Frame" + std::to_string(slot) + ".Cascade0.Integrate")->GetVkDescriptorSet());
            seed->PushConstant(command, 0, phase == 2 ? 1u : 0u);
            seed->Dispatch(command, 289, 17, 4);
          });
          field->transport_recorded = true;
          field->transport_pass = generation;
          if (phase == 0)
            publication->input_upload.Record(publication->uploads);
        });
    if (phase != 0)
      publication->AddPublication(scene_graph, runtime);
    const auto scene_plan = scene_graph.Compile({});
    ASSERT_TRUE(scene_plan.valid);
    EXPECT_FALSE(scene_plan.uses_compute_queue);
    scene_graph.Execute(scene_plan, scene_registry);
    for (uint32_t order = 0; order < 2; ++order) {
      const uint32_t camera = order ^ (phase & 1);
      RenderGraph camera_graph;
      RenderGraphResourceRegistry camera_registry;
      publication->ImportCamera(camera_graph, camera_registry, *field);
      EXPECT_EQ(publication->CameraReads().size(), 9u);
      EXPECT_TRUE(camera_graph.HasResource("Frame.SDFGI.ProbePlacement"));
      EXPECT_TRUE(camera_graph.HasResource("Frame.SDFGI.Cascade1.Sdf"));
      EXPECT_TRUE(camera_graph.HasResource("Frame.SDFGI.Cascade1.Light"));
      RenderResourceDescriptor target;
      target.name = "SdfgiGatherTestOutput";
      target.type = RenderResourceType::Buffer;
      target.lifetime = RenderResourceLifetime::Persistent;
      target.byte_size = output_info.size;
      camera_graph.AddResource(target);
      camera_registry.BindBuffer(target.name, output);
      RenderPassDescriptor pass{"SdfgiGatherTestCamera", RenderPassQueue::Graphics, RenderPassScope::Camera};
      pass.resources = publication->CameraReads();
      pass.resources.push_back({target.name, RenderResourceUsage::Write, RenderResourceState::General});
      camera_graph.AddPass(pass, [&](const RenderGraphExecutionContext&) {
        Platform::RecordCommandsMainQueue([&](const VkCommandBuffer command) {
          field->OrderAccess(command, VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT,
                             VK_ACCESS_2_SHADER_READ_BIT | VK_ACCESS_2_SHADER_WRITE_BIT);
          gather->Bind(command);
          gather->BindDescriptorSet(command, 4, output_set->GetVkDescriptorSet());
          gather->BindDescriptorSet(command, 5, publication->descriptor_set->GetVkDescriptorSet());
          gather->PushConstant(command, 0, camera);
          gather->Dispatch(command, 11);
        });
      });
      const auto plan = camera_graph.Compile({});
      ASSERT_TRUE(plan.valid);
      EXPECT_FALSE(plan.uses_compute_queue);
      camera_graph.Execute(plan, camera_registry);
    }
    PlatformLifecycleTestAccess::LateUpdate();
    Platform::WaitForFrameSubmissions("SDFGI two-camera gather fixture readback");
    std::vector<glm::vec4> values;
    output->DownloadVector(values, 44);
    SdfgiFieldStatus status;
    field->buffers.at("Status").buffer->Download(status);
    const bool ready = phase == 1 || phase == 2 || phase == 5 || phase == 6;
    EXPECT_EQ(status.ready, ready ? 1u : 0u);
    for (uint32_t i = 0; i < 22; ++i)
      for (uint32_t axis = 0; axis < 4; ++axis)
        EXPECT_NEAR(values[i][axis], values[22 + i][axis], 1e-4f);
    if (!ready) {
      for (const auto value : values)
        EXPECT_EQ(value, glm::vec4(0));
    } else if (phase != 2) {
      EXPECT_NEAR(values[0].x, 1, 1e-5f);
      EXPECT_NEAR(values[1].x, 1.625f, 1e-5f);
      EXPECT_NEAR(values[2].x, 2, 1e-5f);
      EXPECT_NEAR(values[3].x, 2.625f, 1e-5f);
      EXPECT_NEAR(values[4].x, 3, 1e-5f);
      EXPECT_NEAR(values[4].w, 0.15625f, 1e-5f);
      EXPECT_EQ(values[6], glm::vec4(0));
      const float bias = (1 + 4.0f / 60) * 1.1f;
      const float sharp = phase == 6 ? 8 : 4 + 4 * glm::length(glm::vec3(4 + 1.4f * bias, 0, bias)) / 60;
      EXPECT_NEAR(values[11].x, phase == 5 ? 0 : sharp, 1e-4f);
      EXPECT_FLOAT_EQ(values[11].w, phase == 5 ? 0 : 1);
      EXPECT_FLOAT_EQ(values[10].w, 1);  // A sharp miss must not discard diffuse GI.
      EXPECT_NEAR(values[13].x, phase == 5 ? 2 : 0.5f * sharp + 1, 1e-4f);
      EXPECT_FLOAT_EQ(values[13].w, phase == 5 ? 0.5f : 1);
      EXPECT_NEAR(values[15].x, phase == 5 ? 2 : glm::mix(sharp, 2.0f, 0.1999f * 5), 1e-4f);
      EXPECT_NEAR(values[15].w, phase == 5 ? 0.9995f : 1, 1e-6f);
      EXPECT_NEAR(values[17].x, 2, 1e-5f);
      EXPECT_FLOAT_EQ(values[17].w, 1);
      EXPECT_NEAR(values[19].x, phase == 5 ? 0 : 8, 1e-4f);
      EXPECT_FLOAT_EQ(values[20].w, 0);
      EXPECT_NEAR(values[21].x, 6, 1e-5f);
      EXPECT_FLOAT_EQ(values[21].w, 0.5f);  // Reference trace alpha is independent of the outer diffuse fade.
    } else {
      float total = 0, weighted = 0;
      for (uint32_t j = 0; j < 8; ++j) {
        const glm::ivec3 offset(j & 1, (j >> 1) & 1, (j >> 2) & 1);
        const glm::vec3 delta = glm::vec3(0.3f, 0.4f, 0.6f) - glm::vec3(offset);
        const glm::vec3 trilinear = 1.0f - glm::abs(delta);
        const float occlusion[]{0, 2.0f / 15, 4.0f / 15, 8.0f / 15};
        const float weight = trilinear.x * trilinear.y * trilinear.z * std::max(0.005f, glm::normalize(-delta).x) *
                             std::max(0.01f, occlusion[j & 3]);
        total += weight;
        weighted += (73 + offset.x + 3 * offset.y + 5 * offset.z) * weight;
      }
      EXPECT_NEAR(values[8].x, weighted / total, 0.02f);
      EXPECT_NEAR(values[9].x, weighted / total * 1.625f, 0.04f);
      EXPECT_FLOAT_EQ(values[8].w, 1);
    }
  }
}

void CheckSdfgiRectangularGather(const uint32_t voxel_x, const uint32_t voxel_y, const bool capture = false) {
  ScopedGpuPlatform platform(false);
  ApplicationContext::Get().RegisterAsset<Shader>("Shader", {".eveshader", ".slang"});
  SdfgiSettings settings;
  settings.probe_spacing_cells = 8;
  settings.voxel_count_x = voxel_x;
  settings.voxel_count_y = voxel_y;
  settings.cascade_count = 1;
  settings.history_size = 5;
  settings.min_cell_size = 1;
  settings.vertical_scale = SdfgiSettings::VerticalScale::Percent100;
  const auto render = ApplicationContext::Get().GetLayer<RenderLayer>();
  auto runtime = std::make_shared<SdfgiRuntime>(
      settings, QuerySdfgiCapabilities(1, 5, settings.voxel_count_x, settings.voxel_count_y));
  ASSERT_TRUE(runtime->capabilities.Supported());
  EXPECT_FALSE(Platform::RayTracingEnabled());
  EXPECT_FALSE(Platform::RayQueryEnabled());
  EXPECT_FALSE(Platform::RayAccelerationStructureEnabled());
  ASSERT_TRUE(UpdateSdfgiCascades(settings, glm::vec3(0), runtime->cascades).empty());
  std::string failure;
  auto field = SdfgiResources::TryCreate(settings, SdfgiTestAccess::HostLayouts(*render), failure);
  ASSERT_TRUE(field) << failure;
  runtime->resources = field;
  auto output_layout = std::make_shared<DescriptorSetLayout>();
  output_layout->PushDescriptorBinding(0, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_COMPUTE_BIT, 0);
  output_layout->Initialize();
  VkBufferCreateInfo info{VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO};
  info.size = 14 * sizeof(glm::vec4);
  info.usage = VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_SRC_BIT;
  auto output = std::make_shared<Buffer>(info);
  auto output_set = std::make_shared<DescriptorSet>(output_layout);
  output_set->UpdateBufferDescriptorBinding(0, output);
  auto gather = std::make_shared<ComputePipeline>();
  gather->descriptor_set_layouts = SdfgiTestAccess::HostLayouts(*render);
  gather->descriptor_set_layouts[4] = output_layout;
  gather->descriptor_set_layouts.push_back(field->layouts[static_cast<size_t>(SdfgiLayout::Gather)]);
  gather->compute_shader = Shader::CreateTemporary(
      ShaderType::Compute, "#define TEST_GRID_X " + std::to_string(voxel_x) + "\n#define TEST_GRID_Y " +
                               std::to_string(voxel_y) + "\n#define TEST_CAPTURE " + (capture ? "true" : "false") +
                               "\n" + std::string(R"(
import EvoEngine.Sdfgi;
[[vk::binding(0,4)]] RWStructuredBuffer<float4> result;
[numthreads(1,1,1)]
void main(uint3 id : SV_DispatchThreadID) {
  float3 half_grid = float3(TEST_GRID_X, TEST_GRID_Y, TEST_GRID_X) * 0.5;
  float inner = max(0.0, half_grid.x - 48);
  const float3 positions[7] = {float3(inner,0,0), float3(0,0,inner), float3(0,half_grid.y-12,0),
      float3(0,half_grid.y+16,0), float3(half_grid.x-12,0,0), float3(0,0,half_grid.z+4), float3(half_grid.x+72,0,0)};
  EeSdfgiLighting value = EE_SDFGI_GATHER(positions[id.x], float3(0,1,0), float3(1,0,0), id.x < 2 ? 0.0 : 0.5, !TEST_CAPTURE);
  result[id.x * 2] = float4(value.diffuse, value.weight);
  result[id.x * 2 + 1] = float4(value.specular, value.specular_weight);
}
)"));
  gather->Initialize();
  ASSERT_TRUE(gather->Initialized());
  PlatformLifecycleTestAccess::PreUpdate();
  auto publication = SdfgiGatherFrame::Create(*runtime, 1);
  RenderGraph graph;
  RenderGraphResourceRegistry registry;
  field->Import(graph, registry);
  graph.AddPass({RenderPassNames::sdfgi_maintenance, RenderPassQueue::Graphics, RenderPassScope::Frame},
                [](const RenderGraphExecutionContext&) {
                });
  graph.AddPass(field->ClearDescriptor(), [&](const RenderGraphExecutionContext& context) {
    Platform::RecordCommandsMainQueue([&](const VkCommandBuffer command) {
      field->Clear(command, context);
    });
  });
  graph.AddPass({"SdfgiProbeStore", RenderPassQueue::Graphics, RenderPassScope::Frame, {}, {"SdfgiInitialize"}},
                [&](const RenderGraphExecutionContext&) {
                  Platform::RecordCommandsMainQueue([&](const VkCommandBuffer command) {
                    field->OrderAccess(command, VK_PIPELINE_STAGE_2_TRANSFER_BIT, VK_ACCESS_2_TRANSFER_WRITE_BIT);
                    VkClearColorValue clear{};
                    clear.uint32[0] = 2 | (2 << 9) | (2 << 18) | (24u << 27);
                    VkImageSubresourceRange range{VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 2};
                    Platform::ClearColorImage(command, *field->textures.at("Atlas").image, clear, 1, &range);
                    range.layerCount = 1;
                    clear.uint32[0] = 8 | (8 << 9) | (8 << 18) | (24u << 27);
                    Platform::ClearColorImage(command, *field->textures.at("Cascade0.Light").image, clear, 1, &range);
                    clear.uint32[0] = 0xffff;
                    Platform::ClearColorImage(command, *field->textures.at("Occlusion").image, clear, 1, &range);
                    field->buffers.at("Status").buffer->Fill(command, offsetof(SdfgiFieldStatus, generation), 4, 1);
                  });
                  field->transport_recorded = true;
                  field->transport_pass = 1;
                });
  publication->AddPublication(graph, runtime);
  RenderResourceDescriptor target;
  target.name = "WideGatherOutput";
  target.type = RenderResourceType::Buffer;
  target.lifetime = RenderResourceLifetime::Persistent;
  target.byte_size = info.size;
  graph.AddResource(target);
  registry.BindBuffer(target.name, output);
  RenderPassDescriptor pass{"WideGather", RenderPassQueue::Graphics, RenderPassScope::Camera, {}, {"SdfgiPublish"}};
  pass.resources = publication->CameraReads();
  pass.resources.push_back({target.name, RenderResourceUsage::Write, RenderResourceState::General});
  graph.AddPass(pass, [&](const RenderGraphExecutionContext&) {
    Platform::RecordCommandsMainQueue([&](const VkCommandBuffer command) {
      field->OrderAccess(command, VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT,
                         VK_ACCESS_2_SHADER_READ_BIT | VK_ACCESS_2_SHADER_WRITE_BIT);
      gather->Bind(command);
      gather->BindDescriptorSet(command, 4, output_set->GetVkDescriptorSet());
      gather->BindDescriptorSet(command, 5, publication->descriptor_set->GetVkDescriptorSet());
      gather->Dispatch(command, 7);
    });
  });
  const auto plan = graph.Compile({});
  ASSERT_TRUE(plan.valid);
  EXPECT_FALSE(plan.uses_compute_queue);
  graph.Execute(plan, registry);
  PlatformLifecycleTestAccess::LateUpdate();
  Platform::WaitForFrameSubmissions("SDFGI rectangular coverage fixture");
  std::vector<glm::vec4> values;
  output->DownloadVector(values, 14);
  const float weights[]{1, 1, 0.5f, 0, 0.5f, 0, 0};
  for (uint32_t i = 0; i < 7; ++i) {
    EXPECT_NEAR(values[i * 2].w, weights[i], 1e-5f);
    EXPECT_NEAR(values[i * 2].x, weights[i] > 0 ? 2.0f : 0.0f, 1e-5f);
    for (uint32_t component = 0; component < 4; ++component) {
      EXPECT_TRUE(std::isfinite(values[i * 2][component]));
      EXPECT_TRUE(std::isfinite(values[i * 2 + 1][component]));
    }
  }
  for (uint32_t i : {1u, 3u}) {
    EXPECT_NEAR(values[i].x, capture ? 0 : 4, 1e-5f);
    EXPECT_FLOAT_EQ(values[i].w, capture ? 0 : 1);
  }
  if (capture)
    for (uint32_t i = 0; i < 7; ++i)
      EXPECT_EQ(values[i * 2 + 1], glm::vec4(0));
}

TEST(SdfgiGather, ReflectionCaptureKeepsDiffuseAndCoverageWithoutSpecularOrSharpTracing) {
  CheckSdfgiRectangularGather(80, 144, true);
}

TEST(SdfgiReflectionCapture, LiveBakeDynamicUpdatesAndLayoutResetWithoutRt) {
  TempProject project;
  Application app;
  struct Terminate {
    Application& app;
    ~Terminate() {
      app.Terminate();
    }
  } terminate{app};
  app.PushLayer<RenderLayer>("Render Layer");
  auto initialization = TestApplicationSettings(project);
  initialization.load_default_resources = true;
  initialization.load_project_assets = true;
  initialization.load_project_start_scene = true;
  const auto* sponza_resources = std::getenv("EVOENGINE_SDFGI_CAPTURE_TEST_RESOURCES");
  if (sponza_resources)
    SetupDemoScene(DemoSetup::Rendering, initialization, sponza_resources, false);
  initialization.graphics_settings.use_ray_tracing = false;
  app.Initialize(initialization);
  app.Start();
  for (uint32_t frame = 0; frame < 30000 && !app.GetActiveScene(); ++frame)
    ASSERT_TRUE(app.Loop());
  const auto scene = app.GetActiveScene();
  ASSERT_TRUE(scene);
  auto camera = scene->main_camera.Get<Camera>();
  if (!camera) {
    camera = scene->GetOrSetPrivateComponent<Camera>(scene->CreateEntity("Capture anchor")).lock();
    scene->main_camera = camera;
  }
  camera->camera_render_mode = Camera::CameraRenderMode::Rasterization;
  camera->SetRequireRendering(true);
  camera->Resize(sponza_resources ? glm::uvec2(2560, 1440) : glm::uvec2(128));
  auto lighting = scene->environmental_lighting.Get<EnvironmentalLighting>();
  if (!lighting) {
    lighting = AssetManager::CreateTemporaryAsset<EnvironmentalLighting>();
    scene->environmental_lighting = lighting;
  }
  if (!sponza_resources) {
    scene->SetDataComponent(camera->GetOwner(), Transform{});
    const auto entity = scene->CreateEntity("Diffuse capture receiver");
    const auto renderer = scene->GetOrSetPrivateComponent<MeshRenderer>(entity).lock();
    renderer->mesh = Resources::GetInstance().GetPrimitives().cube;
    renderer->material = AssetManager::CreateTemporaryAsset<Material>();
    Transform transform;
    transform.SetValue(glm::vec3(0, 0, -3), glm::vec3(0), glm::vec3(2));
    scene->SetDataComponent(entity, transform);
    scene->SetEntityStatic(entity, true);
    lighting->indirect_environment_source.kind = EnvironmentalLighting::IndirectEnvironmentSourceKind::Color;
    lighting->indirect_environment_source.color = glm::vec3(0.5f, 0.25f, 0.125f);
    lighting->gi_probe_settings.probe_count_x = lighting->gi_probe_settings.probe_count_y = 17;
    lighting->gi_probe_settings.cascade_count = 1;
  }
  lighting->indirect_gi_provider = IndirectGiProvider::AutomaticSdfgi;
  lighting->dynamic_reflection_probe_settings.enabled = false;
  const auto render = ApplicationContext::Get().GetLayer<RenderLayer>();
  ASSERT_TRUE(render);
  EXPECT_FALSE(Platform::RayTracingEnabled());
  EXPECT_FALSE(Platform::RayQueryEnabled());
  EXPECT_FALSE(Platform::RayAccelerationStructureEnabled());
  const auto loop_until = [&](const auto& ready) {
    for (uint32_t frame = 0; frame < 360; ++frame) {
      if (!app.Loop())
        return false;
      if (ready())
        return true;
    }
    return false;
  };
  ASSERT_TRUE(loop_until([&] {
    const auto runtime = scene->GetSdfgiRuntime();
    return ProjectManager::IsProjectIdle() && runtime && runtime->published && runtime->resources &&
           runtime->resources->transport_pass >= 90;
  }));
  const auto anchor = scene->GetSdfgiRuntime()->anchor;
  const auto original_probe_settings = lighting->gi_probe_settings;
  const auto original_pack = lighting->reflection_probe_pack;
  auto pack = AssetManager::CreateTemporaryAsset<ReflectionProbePack>();
  lighting->reflection_probe_pack = pack;
  pack->probes.emplace_back();
  auto& probe = pack->probes.front();
  probe.stable_id = 123;
  probe.transform = glm::translate(glm::mat4(1), anchor.world_position);
  probe.box_projection_extents = glm::vec3(30);
  const auto payload = probe.GetOrCreatePayload();
  const auto bake = [&] {
    if (render->QueueGlobalReflectionProbeBakeBatch(scene, {{anchor.world_position, payload, pack, probe.stable_id}}) !=
        1)
      return false;
    return loop_until([&] {
             return !render->HasPendingGlobalReflectionProbeBake();
           }) &&
           payload->IsRuntimeReady();
  };
  lighting->sdfgi_settings.energy = 0;
  ASSERT_TRUE(app.Loop());
  ASSERT_TRUE(bake());
  std::vector<uint16_t> dark, lit;
  ASSERT_TRUE(payload->ReadCanonicalPayload(dark));
  lighting->sdfgi_settings.energy = 2;
  ASSERT_TRUE(app.Loop());
  ASSERT_TRUE(bake());
  ASSERT_TRUE(payload->ReadCanonicalPayload(lit));
  ASSERT_EQ(dark.size(), lit.size());
  EXPECT_NE(dark, lit);
  for (const uint16_t value : lit)
    ASSERT_TRUE(std::isfinite(glm::unpackHalf1x16(value)));
  const auto baked_hash = GlobalReflectionProbe::CalculatePayloadHash(lit);
  const auto immediate_id = SdfgiTestAccess::CaptureImmediately(*render, scene, anchor.world_position);
  const auto& immediate_reads = scene->GetSdfgiRuntime()->resources->gather_camera_ids;
  EXPECT_NE(std::find(immediate_reads.begin(), immediate_reads.end(), immediate_id), immediate_reads.end());
  lighting->dynamic_reflection_probe_settings.faces_per_frame = 1;
  lighting->dynamic_reflection_probe_settings.enabled = true;
  bool saw_capture_read = false;
  ASSERT_TRUE(loop_until([&] {
    const auto& ids = scene->GetSdfgiRuntime()->resources->gather_camera_ids;
    saw_capture_read |= std::any_of(ids.begin(), ids.end(), [&](uint64_t id) {
      return id != anchor.camera_id;
    });
    return render->GetDynamicReflectionProbeStats().published_generation_count >= 1;
  }));
  EXPECT_TRUE(saw_capture_read);
  ASSERT_TRUE(loop_until([&] {
    return render->GetDynamicReflectionProbeStats().completed_face_count == 1;
  }));
  std::weak_ptr<SdfgiResources> retired = scene->GetSdfgiRuntime()->resources;
  lighting->gi_probe_settings.probe_count_x = 21;
  lighting->gi_probe_settings.probe_count_y = 19;
  const auto generation = render->GetDynamicReflectionProbeStats().published_generation_count;
  ASSERT_TRUE(app.Loop());
  EXPECT_FALSE(retired.expired());
  ASSERT_TRUE(loop_until([&] {
    return scene->GetSdfgiRuntime()->published &&
           render->GetDynamicReflectionProbeStats().published_generation_count > generation;
  }));
  EXPECT_TRUE(retired.expired());
  EXPECT_EQ(scene->GetSdfgiRuntime()->resources->settings.ProbeSize(), glm::ivec3(21, 19, 21));
  EXPECT_EQ(scene->GetSdfgiRuntime()->anchor.camera_id, anchor.camera_id);
  EXPECT_EQ(scene->GetSdfgiRuntime()->anchor.world_position, anchor.world_position);
  ASSERT_TRUE(payload->ReadCanonicalPayload(lit));
  EXPECT_EQ(GlobalReflectionProbe::CalculatePayloadHash(lit), baked_hash);
  if (sponza_resources) {
    lighting->reflection_probe_pack = original_pack;
    lighting->sdfgi_settings.energy = 1;
    lighting->gi_probe_settings = original_probe_settings;
    ASSERT_TRUE(loop_until([&] {
      const auto runtime = scene->GetSdfgiRuntime();
      return runtime->published && runtime->resources->transport_pass >= 90 &&
             render->GetDynamicReflectionProbeStats().published_generation_count >= 5;
    }));
    const auto path = std::filesystem::path(sponza_resources).parent_path() / "m12d-sponza.png";
    camera->GetRenderTexture()->StoreToPng(path);
    std::vector<glm::vec4> pixels;
    camera->GetRenderTexture()->GetRgbaChannelData(pixels);
    ASSERT_EQ(pixels.size(), 2560u * 1440u);
    for (const auto pixel : pixels)
      ASSERT_TRUE(std::isfinite(pixel.x) && std::isfinite(pixel.y) && std::isfinite(pixel.z) && std::isfinite(pixel.w));
    std::cout << "SDFGI reflection Sponza capture: " << path << " RT pipeline/query/AS disabled\n";
  }
}

TEST(SdfgiWide, GatherExtendsOnlyHorizontalCoverageAndSharpReflectionsWithoutRt) {
  CheckSdfgiRectangularGather(256, 128);
}

TEST(SdfgiConfigurable, GatherTallNonPowerOfTwoWithoutRt) {
  CheckSdfgiRectangularGather(80, 144);
}

SdfgiSolidCell SdfgiTestCell(const glm::ivec3 p) {
  SdfgiSolidCell cell{};
  cell.position = (p.x & 127) | ((p.y & 127) << 7) | ((p.z & 127) << 14);
  cell.position_high = (p.x >> 7) | ((p.z >> 7) << 1) | ((p.y >> 7) << 2);
  return cell;
}

TEST(SdfgiConfigurable, CompactCellCoordinateBitsPreservePayload) {
  for (int x : {0, 127, 128, 255})
    for (int y : {0, 127, 128, 255})
      for (int z : {0, 127, 128, 255}) {
        auto cell = SdfgiTestCell({x, y, z});
        cell.position |= 0xffe00000u;
        const glm::ivec3 unpacked((cell.position & 127) | ((cell.position_high & 1) << 7),
                                  ((cell.position >> 7) & 127) | ((cell.position_high & 4) << 5),
                                  ((cell.position >> 14) & 127) | ((cell.position_high & 2) << 6));
        EXPECT_EQ(unpacked, glm::ivec3(x, y, z));
        EXPECT_EQ(cell.position & 0xffe00000u, 0xffe00000u);
      }
}

void CheckSdfgiScrolling(const uint32_t voxel_x, const uint32_t voxel_y = 128, const uint32_t spacing = 8) {
  ScopedGpuPlatform platform(false);
  ApplicationContext::Get().RegisterAsset<Shader>("Shader", {".eveshader", ".slang"});
  EXPECT_FALSE(Platform::RayTracingEnabled());
  EXPECT_FALSE(Platform::RayQueryEnabled());
  EXPECT_FALSE(Platform::RayAccelerationStructureEnabled());
  SdfgiSettings settings;
  settings.probe_spacing_cells = spacing;
  settings.voxel_count_x = voxel_x;
  settings.voxel_count_y = voxel_y;
  const auto layout = SdfgiProbeLayout::Create(
      voxel_x, voxel_y, spacing, Platform::GetSelectedPhysicalDevice()->properties.limits.maxImageDimension2D);
  const uint32_t axis = settings.ProbeSize().x, columns = layout.columns, atlas_width = columns * 8;
  const auto grid = settings.GridSize();
  const uint32_t rows = layout.rows, vertical = settings.ProbeSize().y;
  settings.cascade_count = 2;
  settings.min_cell_size = 1;
  settings.vertical_scale = SdfgiSettings::VerticalScale::Percent100;
  settings.history_size = 5;
  std::string failure;
  const auto render = ApplicationContext::Get().GetLayer<RenderLayer>();
  auto field = SdfgiResources::TryCreate(settings, SdfgiTestAccess::HostLayouts(*render), failure);
  ASSERT_TRUE(field) << failure;
  std::vector<SdfgiCascade> cascades;
  ASSERT_TRUE(UpdateSdfgiCascades(settings, glm::vec3(0), cascades).empty());
  const auto cascade_data = BuildSdfgiCascadeBlock(cascades);
  auto seed = std::make_shared<ComputePipeline>();
  seed->descriptor_set_layouts = {field->layouts[static_cast<size_t>(SdfgiLayout::Integrate)]};
  seed->push_constant_ranges.push_back({VK_SHADER_STAGE_COMPUTE_BIT, 0, sizeof(uint32_t)});
  seed->compute_shader = Shader::CreateTemporary(
      ShaderType::Compute, "#define EE_PROBE_AXIS " + std::to_string(axis) + "\n" + std::string(R"(
import EvoEngine.SdfgiTypes;
[[vk::binding(9,0)]] [vk::image_format("rgba16i")] RWTexture2DArray<int4> history;
[[vk::binding(10,0)]] [vk::image_format("rgba32i")] RWTexture2D<int4> average;
[[vk::push_constant]] ConstantBuffer<uint> cascade;
[numthreads(8,8,1)]
void main(uint3 id : SV_DispatchThreadID) {
  uint width, height;
  average.GetDimensions(width, height);
  if (id.x >= width || id.y >= height) return;
  int3 probe = SdfgiProbeCell(int2(id.x, id.y / 16), EE_PROBE_AXIS, width);
  int base = int(cascade) * 500 + probe.x + probe.y * 2 + probe.z * 4 + int(id.y % 16) * 16;
  int4 sum = 0;
  for (int h = 0; h < 5; ++h) {
    int v = base + h;
    int4 value = int4(v, -v, v * 2, 1024);
    history[int3(id.xy, h)] = value;
    sum += value;
  }
  average[id.xy] = sum;
}
)"));
  seed->Initialize();
  ASSERT_TRUE(seed->Initialized());
  glm::ivec3 shifts[]{{8, 0, 0}, {-8, 0, 0}, {0, 8, 0}, {0, -8, 0}, {0, 0, 8}, {0, 0, -8}, {8, -16, 8}, {8, 0, 0}};
  for (auto& shift : shifts)
    shift = shift / 8 * static_cast<int>(spacing);
  const glm::ivec3 cell_position(voxel_x == 256 ? 192 : grid.x / 2, voxel_y > 128 ? 132 : grid.y / 2,
                                 voxel_x == 256 ? 192 : grid.z / 2);
  auto cell = SdfgiTestCell(cell_position);
  cell.albedo = 0x4321 | (13 << 15);
  cell.light = 0x12345678;
  cell.light_aniso = 0x23456789;
  const SdfgiDispatchData dispatch{1, 1, 1, 1};
  const auto value_at = [](const glm::vec3 p, const uint32_t cascade, const int coefficient, const int history) {
    const float v = cascade * 500 + p.x + 2 * p.y + 4 * p.z + coefficient * 16 + history;
    return glm::ivec4(v, -v, 2 * v, 1024);
  };
  BufferUploadArena arena;
  for (uint32_t phase = 0; phase < std::size(shifts); ++phase) {
    SCOPED_TRACE(phase);
    PlatformLifecycleTestAccess::PreUpdate();
    const auto slot = Platform::GetCurrentFrameIndex();
    const bool failed = phase == 7;
    field->settings.bounce_feedback = phase & 1 ? 0.0f : 0.5f;
    BufferUploadBatch upload;
    upload.Add(field->buffers.at("Frame" + std::to_string(slot) + ".Cascades").buffer, cascade_data,
               {BufferUploadUsage::Uniform});
    for (uint32_t c = 0; c < 2; ++c) {
      const auto prefix = "Cascade" + std::to_string(c) + ".";
      upload.Add(field->buffers.at(prefix + "UnlitCells").buffer, cell);
      upload.Add(field->buffers.at(prefix + "Dispatch").buffer, dispatch);
      upload.Add(field->buffers.at(prefix + "Indirect").buffer, dispatch);
    }
    RenderGraph graph;
    RenderGraphResourceRegistry registry;
    field->Import(graph, registry);
    graph.AddPass({RenderPassNames::sdfgi_maintenance, RenderPassQueue::Graphics, RenderPassScope::Frame},
                  [](const RenderGraphExecutionContext&) {
                  });
    graph.AddPass(field->ClearDescriptor(), [&](const RenderGraphExecutionContext& context) {
      Platform::RecordCommandsMainQueue([&](const VkCommandBuffer command) {
        field->Clear(command, context);
      });
    });
    graph.AddPass(
        {"ScrollSeed", RenderPassQueue::Graphics, RenderPassScope::Frame, {}, {"SdfgiInitialize"}},
        [&](const RenderGraphExecutionContext&) {
          upload.Record(arena);
          Platform::RecordCommandsMainQueue([&](const VkCommandBuffer command) {
            field->OrderAccess(command, VK_PIPELINE_STAGE_2_TRANSFER_BIT, VK_ACCESS_2_TRANSFER_WRITE_BIT);
            VkClearColorValue clear{};
            clear.uint32[0] = 0x3210;
            const VkImageSubresourceRange range{VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1};
            Platform::ClearColorImage(command, *field->textures.at("Occlusion").image, clear, 1, &range);
            if (failed)
              field->buffers.at("Status").buffer->Fill(command, offsetof(SdfgiFieldStatus, failure_flags), 4, 1);
            field->OrderAccess(command, VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT, VK_ACCESS_2_SHADER_WRITE_BIT);
            seed->Bind(command);
            for (uint32_t c = 0; c < 2; ++c) {
              seed->BindDescriptorSet(
                  command, 0,
                  field->sets.at("Frame" + std::to_string(slot) + ".Cascade" + std::to_string(c) + ".Integrate")
                      ->GetVkDescriptorSet());
              seed->PushConstant(command, 0, c);
              seed->Dispatch(command, (columns + 7) / 8, rows * 2);
            }
          });
        });
    const auto readback = std::make_shared<SdfgiPreprocessReadback>(2);
    auto previous = std::string("ScrollSeed");
    for (uint32_t c = 0; c < 2; ++c) {
      // Exercise the production graph pass, shared scratch, scroll/store and subsequent topology rebuild.
      previous = AddSdfgiPreprocessPass(graph, registry, field, readback, c, -shifts[phase], previous, shifts[phase]);
    }
    const auto plan = graph.Compile({});
    ASSERT_TRUE(plan.valid);
    EXPECT_FALSE(plan.uses_compute_queue);
    graph.Execute(plan, registry);
    PlatformLifecycleTestAccess::LateUpdate();
    Platform::WaitForFrameSubmissions("SDFGI focused scroll mapping readback");
    readback->ReadAfterFrameFence(*field);
    EXPECT_EQ(field->preprocess_status.failure_flags, failed ? 1u : 0u);
    for (uint32_t c = 0; c < 2; ++c) {
      SCOPED_TRACE(c);
      const auto prefix = "Cascade" + std::to_string(c) + ".";
      VkBufferImageCopy copy{};
      copy.imageSubresource = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 0, 1};
      copy.imageExtent = {columns, (rows * 16), 1};
      Buffer average_buffer(columns * (rows * 16) * sizeof(glm::ivec4));
      average_buffer.CopyFromImage(*field->textures.at(prefix + "Average").image, copy);
      std::vector<glm::ivec4> average;
      average_buffer.DownloadVector(average, columns * (rows * 16));
      copy.imageSubresource.layerCount = 5;
      Buffer history_buffer(columns * (rows * 16) * 5 * 8);
      history_buffer.CopyFromImage(*field->textures.at(prefix + "History").image, copy);
      std::vector<int16_t> history;
      history_buffer.DownloadVector(history, columns * (rows * 16) * 5 * 4);
      for (const glm::ivec3 p : {glm::ivec3(0), settings.ProbeSize() - 1, settings.ProbeSize() / 2,
                                 glm::ivec3(0, vertical - 1, axis / 2), glm::ivec3(axis - 1, 0, axis / 2),
                                 glm::ivec3(axis / 2, vertical / 2, 0), glm::ivec3(axis / 2, vertical / 2, axis - 1)}) {
        const glm::ivec3 read = p - shifts[phase] / static_cast<int>(spacing);
        const bool retained =
            glm::all(glm::greaterThanEqual(read, glm::ivec3(0))) && glm::all(glm::lessThan(read, settings.ProbeSize()));
        const bool parent = !failed && !retained && c == 0;
        const glm::vec3 source = failed     ? glm::vec3(p)
                                 : retained ? glm::vec3(read)
                                 : parent   ? glm::vec3(p) * 0.5f + glm::vec3(settings.ProbeSize() - 1) * 0.25f
                                            : glm::vec3(p);
        for (int coefficient = 0; coefficient < 16; ++coefficient) {
          const auto index = layout.Index(p.x, p.y, p.z);
          const auto pixel = (index / columns * 16 + coefficient) * columns + index % columns;
          glm::ivec4 sum(0);
          for (int h = 0; h < 5; ++h) {
            const auto expected = value_at(source, parent ? 1 : c, coefficient, parent ? 2 : h);
            sum += expected;
            for (int channel = 0; channel < 4; ++channel)
              EXPECT_NEAR(history[(h * columns * (rows * 16) + pixel) * 4 + channel], expected[channel],
                          parent ? 1 : 0);
          }
          for (int channel = 0; channel < 4; ++channel)
            EXPECT_NEAR(average[pixel][channel], sum[channel], parent ? 5 : 0);
        }
      }
      if (!failed) {
        ASSERT_EQ(field->solid_cell_dispatch[c].total_count, 1u);
        SdfgiSolidCell retained;
        field->buffers.at(prefix + "UnlitCells").buffer->Download(retained);
        const glm::ivec3 write = cell_position + shifts[phase];
        EXPECT_EQ(retained.position & 0x1fffffu, (write.x & 127) | ((write.y & 127) << 7) | ((write.z & 127) << 14));
        EXPECT_EQ(retained.position_high, uint32_t((write.x >> 7) | ((write.z >> 7) << 1) | ((write.y >> 7) << 2)));
        EXPECT_EQ(retained.albedo & 0x1fffffu, cell.albedo);
        EXPECT_EQ(retained.light & 0x3fffffffu, cell.light);
        EXPECT_EQ(retained.light_aniso & 0x3fffffffu, cell.light_aniso);
        copy.imageSubresource.layerCount = 1;
        copy.imageOffset = {32, 32, static_cast<int>(c * grid.z + 32)};
        copy.imageExtent = {1, 1, 1};
        Buffer packed_buffer(2);
        packed_buffer.CopyFromImage(*field->textures.at("Occlusion").image, copy);
        uint16_t packed;
        packed_buffer.Download(packed);
        EXPECT_EQ(packed, 0x3210);  // Untouched neighborhoods must keep packed visibility, not recompute it.
      }
    }
    VkBufferImageCopy copy{};
    copy.imageSubresource = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 0, 4};
    copy.imageExtent = {atlas_width, (rows * 8), 1};
    Buffer atlas_buffer(atlas_width * (rows * 8) * 4 * 4);
    atlas_buffer.CopyFromImage(*field->textures.at("Atlas").image, copy);
    std::vector<uint32_t> atlas;
    atlas_buffer.DownloadVector(atlas, atlas_width * (rows * 8) * 4);
    EXPECT_EQ(std::any_of(atlas.begin(), atlas.end(),
                          [](uint32_t value) {
                            return value != 0;
                          }),
              field->settings.bounce_feedback > 0 && !failed);
  }
}

TEST(SdfgiScrolling, SignedRetentionParentHistoryOcclusionAndFailureWithoutRt) {
  CheckSdfgiScrolling(128);
}

TEST(SdfgiDensity, PackedScrollingAndParentHistoryWithoutRt) {
  CheckSdfgiScrolling(128, 64, 2);
}

TEST(SdfgiWide, SignedRetentionParentHistoryOcclusionAndFailureWithoutRt) {
  CheckSdfgiScrolling(256);
}

TEST(SdfgiConfigurable, ScrollingTallNonPowerOfTwoWithoutRt) {
  CheckSdfgiScrolling(80, 144);
}

void CheckSdfgiTransport(const uint32_t voxel_x, const uint32_t voxel_y = 128, const uint32_t spacing = 8) {
  ScopedGpuPlatform platform(false);
  ApplicationContext::Get().RegisterAsset<Shader>("Shader", {".eveshader", ".slang"});
  EXPECT_FALSE(Platform::RayTracingEnabled());
  EXPECT_FALSE(Platform::RayQueryEnabled());
  EXPECT_FALSE(Platform::RayAccelerationStructureEnabled());
  SdfgiSettings settings;
  settings.probe_spacing_cells = spacing;
  settings.voxel_count_x = voxel_x;
  settings.voxel_count_y = voxel_y;
  const auto layout = SdfgiProbeLayout::Create(
      voxel_x, voxel_y, spacing, Platform::GetSelectedPhysicalDevice()->properties.limits.maxImageDimension2D);
  const uint32_t axis = settings.ProbeSize().x, columns = layout.columns, atlas_width = columns * 8;
  const auto grid = settings.GridSize();
  const uint32_t rows = layout.rows;
  const uint32_t center_probe = layout.Index(axis / 2, settings.ProbeSize().y / 2, axis / 2);
  const uint32_t center_column = center_probe % columns, center_row = center_probe / columns;
  settings.cascade_count = 2;
  settings.min_cell_size = 1;
  settings.vertical_scale = SdfgiSettings::VerticalScale::Percent100;
  settings.history_size = 5;
  settings.ray_count = 4;
  const auto render = ApplicationContext::Get().GetLayer<RenderLayer>();
  std::string failure;
  const auto field = SdfgiResources::TryCreate(settings, SdfgiTestAccess::HostLayouts(*render), failure);
  ASSERT_TRUE(field) << failure;
  std::vector<SdfgiCascade> cascades;
  ASSERT_TRUE(UpdateSdfgiCascades(settings, glm::vec3(0), cascades).empty());
  const auto cascade_data = BuildSdfgiCascadeBlock(cascades);
  auto shell = std::make_shared<ComputePipeline>();
  shell->descriptor_set_layouts = {field->layouts[static_cast<size_t>(SdfgiLayout::Store)]};
  shell->compute_shader = Shader::CreateTemporary(ShaderType::Compute, std::string(R"(
[[vk::binding(7,0)]] [vk::image_format("r8")] RWTexture3D<float> sdf;
[numthreads(4,4,4)]
void main(uint3 id : SV_DispatchThreadID) {
  uint width, height, depth;
  sdf.GetDimensions(width, height, depth);
  float3 p = abs(float3(id) + 0.5 - float3(width, height, depth) * 0.5);
  float d = max(0.0, abs(max(p.x, max(p.y, p.z)) - float(min(width, min(height, depth))) * 0.375) - 1);
  sdf[id] = d == 0 ? 0 : (d + 1) / 255.0;
}
)"));
  shell->Initialize();
  ASSERT_TRUE(shell->Initialized());
  auto cube = std::make_shared<Cubemap>();
  std::vector<glm::vec4> cube_data(6 * 21);
  for (uint32_t face = 0; face < 6; ++face) {
    std::fill_n(cube_data.begin() + face * 21, 16, glm::vec4(1, 0, 0, 1));
    std::fill_n(cube_data.begin() + face * 21 + 16, 4, glm::vec4(0, 1, 0, 1));
    cube_data[face * 21 + 20] = {0, 0, 4, 1};
  }
  ASSERT_TRUE(cube->SetRgbaChannelData(cube_data, 4, 3));
  ASSERT_TRUE(cube->GetImage());  // Ordinary asset upload precedes SDFGI maintenance.
  SdfgiSkyInput sky;
  sky.constant_color = true;
  sky.color = {1, 2, 100};
  std::vector<glm::ivec4> first_average;
  std::vector<uint32_t> first_atlas;
  std::shared_ptr<SdfgiProbeFrame> retained_sky;
  std::shared_ptr<SdfgiProbeDebug> retained_debug;
  BufferUploadArena upload_arena;
  for (uint32_t iteration = 0; iteration < 15; ++iteration) {
    SCOPED_TRACE(iteration);
    if (iteration == 10) {
      sky.constant_color = false;
      sky.cubemap = cube;
      sky.gamma = 2;
      sky.energy = 3;
      sky.rotation = glm::half_pi<float>();
    }
    if (iteration == 11) {
      ASSERT_TRUE(cube->SetRgbaChannelData(std::vector<glm::vec4>(6, glm::vec4(4, 0, 0, 1)), 1));
      ASSERT_TRUE(cube->GetImage());
      sky.energy = 0.5f;
    }
    if (iteration >= 12)
      field->settings.read_sky_light = false;
    const bool reset = iteration == 0 || iteration >= 10;
    if (reset)
      field->transport_pass = 0;
    PlatformLifecycleTestAccess::PreUpdate();
    const auto slot = Platform::GetCurrentFrameIndex();
    auto frame = SdfgiProbeFrame::Create(*field, cascades, sky, iteration);
    field->probe_frames[slot] = frame;
    EXPECT_EQ(frame->constants[0].history_index, iteration < 10 ? iteration % 5 : 0);
    EXPECT_EQ(frame->constants[0].world_offset[0], 0);
    if (iteration == 10) {
      retained_sky = frame;
      EXPECT_FLOAT_EQ(frame->constants[0].sky_lod_inverse_gamma[0], 2);
      EXPECT_FLOAT_EQ(frame->constants[0].sky_lod_inverse_gamma[1], 0.5f);
      EXPECT_EQ(frame->constants[0].sky_flags, 6u);
      const auto* q = frame->constants[0].sky_color_or_orientation;
      const auto rotated =
          glm::quat(std::sqrt(1 - q[0] * q[0] - q[1] * q[1] - q[2] * q[2]), q[0], q[1], q[2]) * glm::vec3(1, 0, 0);
      EXPECT_NEAR(rotated.x, 0, 1e-6f);
      EXPECT_NEAR(rotated.z, 1, 1e-6f);
    }
    if (iteration == 11) {
      EXPECT_FLOAT_EQ(frame->constants[0].sky_lod_inverse_gamma[0], 0);
      EXPECT_NE(frame->sky_image, retained_sky->sky_image);
      EXPECT_EQ(retained_sky->sky_image->GetExtent().width, 4u);
    }
    if (iteration >= 12)
      EXPECT_EQ(frame->constants[0].sky_flags, 0u);
    BufferUploadBatch upload;
    upload.Add(field->buffers.at("Frame" + std::to_string(slot) + ".Cascades").buffer, cascade_data,
               {BufferUploadUsage::Uniform});
    RenderGraph graph;
    RenderGraphResourceRegistry registry;
    field->Import(graph, registry);
    graph.AddPass({RenderPassNames::sdfgi_maintenance, RenderPassQueue::Graphics, RenderPassScope::Frame},
                  [](const RenderGraphExecutionContext&) {
                  });
    if (reset)
      graph.AddPass(field->ClearDescriptor(), [&](const RenderGraphExecutionContext& context) {
        Platform::RecordCommandsMainQueue([&](const VkCommandBuffer command) {
          field->Clear(command, context);
        });
      });
    graph.AddPass(
        {"SdfgiTransportSeed",
         RenderPassQueue::Graphics,
         RenderPassScope::Frame,
         {},
         {reset ? "SdfgiInitialize" : RenderPassNames::sdfgi_maintenance}},
        [&](const RenderGraphExecutionContext&) {
          upload.Record(upload_arena);
          if (!reset)
            return;
          Platform::RecordCommandsMainQueue([&](const VkCommandBuffer command) {
            field->OrderAccess(command, VK_PIPELINE_STAGE_2_TRANSFER_BIT, VK_ACCESS_2_TRANSFER_WRITE_BIT);
            const VkImageSubresourceRange range{VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1};
            VkClearColorValue clear{};
            clear.float32[0] = 1;
            for (uint32_t c = 0; c < 2; ++c)
              Platform::ClearColorImage(command, *field->textures.at("Cascade" + std::to_string(c) + ".Sdf").image,
                                        clear, 1, &range);
            if (iteration == 14) {
              field->buffers.at("Status").buffer->Fill(command, offsetof(SdfgiFieldStatus, failure_flags), 4, 1);
              for (auto& value : clear.int32)
                value = 1234;
              Platform::ClearColorImage(command, *field->textures.at("Cascade0.Average").image, clear, 1, &range);
            }
            if (iteration == 13) {
              clear.uint32[0] = 256 | (256 << 9) | (256 << 18) | (17u << 27);
              Platform::ClearColorImage(command, *field->textures.at("Cascade1.Light").image, clear, 1, &range);
              for (auto& value : clear.float32)
                value = 1;
              Platform::ClearColorImage(command, *field->textures.at("Cascade1.Aniso0").image, clear, 1, &range);
              Platform::ClearColorImage(command, *field->textures.at("Cascade1.Aniso1").image, clear, 1, &range);
              field->OrderAccess(command, VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT, VK_ACCESS_2_SHADER_WRITE_BIT);
              shell->Bind(command);
              shell->BindDescriptorSet(command, 0, field->sets.at("Cascade1.Store")->GetVkDescriptorSet());
              shell->Dispatch(command, grid.x / 4, grid.y / 4, grid.z / 4);
            }
          });
        });
    field->lighting_recorded = true;
    frame->AddPasses(graph, registry, field, "SdfgiTransportSeed");
    if (iteration == 4) {
      retained_debug = std::make_shared<SdfgiProbeDebug>(settings, 0, center_probe);
      retained_debug->AddPass(graph, registry, field, "SdfgiProbeStore");
    }
    const auto plan = graph.Compile({});
    ASSERT_TRUE(plan.valid);
    EXPECT_FALSE(plan.uses_compute_queue);
    graph.Execute(plan, registry);
    PlatformLifecycleTestAccess::LateUpdate();
    Platform::WaitForFrameSubmissions("SDFGI focused transport readback");
    EXPECT_TRUE(field->transport_recorded);
    EXPECT_EQ(field->transport_pass, iteration < 10 ? iteration + 1 : 1);
    SdfgiFieldStatus status;
    field->buffers.at("Status").buffer->Download(status);
    EXPECT_EQ(status.failure_flags, iteration == 14 ? 1u : 0u);
    EXPECT_EQ(status.ready, 0u);
    EXPECT_EQ(status.generation, field->transport_pass);
    VkBufferImageCopy copy{};
    copy.imageSubresource = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 0, 1};
    copy.imageOffset = {int32_t(center_column), int32_t(center_row * 16), 0};
    copy.imageExtent = {1, 1, 1};
    Buffer sample(sizeof(glm::ivec4));
    sample.CopyFromImage(*field->textures.at("Cascade0.Average").image, copy);
    glm::ivec4 average;
    sample.Download(average);
    if (iteration < 10) {
      const int count = std::min(iteration + 1, 5u);
      EXPECT_EQ(average, glm::ivec4(1155, 2310, 32767, 1024) * count);
    } else if (iteration == 10) {
      EXPECT_EQ(average, glm::ivec4(0, 0, static_cast<int>(6 * 0.282095f * 4 * 1024), 1024));
    } else if (iteration == 11) {
      EXPECT_EQ(average, glm::ivec4(1155, 0, 0, 1024));
    } else if (iteration == 12) {
      EXPECT_EQ(average, glm::ivec4(0, 0, 0, 1024));
    } else if (iteration == 13) {
      EXPECT_GT(average.x, 1000);
      EXPECT_EQ(average.x, average.y);
      EXPECT_EQ(average.y, average.z);
    } else {
      EXPECT_EQ(average, glm::ivec4(1234));
      EXPECT_EQ(retained_debug->ReadStatus().generation, 5u);
      EXPECT_EQ(retained_debug->ReadStatus().failure_flags, 0u);
      std::vector<glm::ivec4> old_average;
      retained_debug->data[1]->DownloadVector(old_average, columns * (rows * 16));
      EXPECT_EQ(old_average, first_average);
    }
    if (iteration == 4 || iteration == 9) {
      copy.imageOffset = {0, 0, 0};
      copy.imageExtent = {columns, (rows * 16), 1};
      Buffer averages(columns * (rows * 16) * sizeof(glm::ivec4));
      averages.CopyFromImage(*field->textures.at("Cascade0.Average").image, copy);
      std::vector<glm::ivec4> values;
      averages.DownloadVector(values, columns * (rows * 16));
      if (iteration == 4)
        first_average = values;
      else
        EXPECT_EQ(values, first_average);
      EXPECT_TRUE(std::any_of(values.begin(), values.end(), [](const auto& value) {
        return value.x < 0;
      }));
      copy.imageExtent = {atlas_width, (rows * 8), 1};
      copy.imageSubresource.layerCount = 4;
      Buffer atlas(atlas_width * (rows * 8) * 4 * sizeof(uint32_t));
      atlas.CopyFromImage(*field->textures.at("Atlas").image, copy);
      std::vector<uint32_t> packed;
      atlas.DownloadVector(packed, atlas_width * (rows * 8) * 4);
      if (iteration == 4)
        first_atlas = packed;
      else
        EXPECT_EQ(packed, first_atlas);
      bool distinct_layers = false;
      for (uint32_t layer : {0u, 2u}) {
        const auto at = [&](const uint32_t x, const uint32_t y) {
          return packed[layer * atlas_width * (rows * 8) + (center_row * 8 + y) * atlas_width + center_column * 8 + x];
        };
        for (uint32_t i = 1; i <= 6; ++i) {
          EXPECT_EQ(at(i, 0), at(7 - i, 1));
          EXPECT_EQ(at(i, 7), at(7 - i, 6));
          EXPECT_EQ(at(0, i), at(1, 7 - i));
          EXPECT_EQ(at(7, i), at(6, 7 - i));
        }
        EXPECT_EQ(at(0, 0), at(6, 6));
        EXPECT_EQ(at(7, 0), at(1, 6));
        EXPECT_EQ(at(0, 7), at(6, 1));
        EXPECT_EQ(at(7, 7), at(1, 1));
      }
      for (uint32_t i = 0; i < atlas_width * (rows * 8); ++i)
        distinct_layers |= packed[i] != packed[i + 2 * atlas_width * (rows * 8)];
      EXPECT_TRUE(distinct_layers);
    }
  }
}

TEST(SdfgiTransport, HistorySkyMipOrientationCrossCascadeAndAtlasBordersWithoutRt) {
  CheckSdfgiTransport(128);
}

TEST(SdfgiWide, HistorySkyMipOrientationCrossCascadeAndAtlasBordersWithoutRt) {
  CheckSdfgiTransport(256);
}

TEST(SdfgiConfigurable, TransportTallNonPowerOfTwoWithoutRt) {
  CheckSdfgiTransport(80, 144);
}

TEST(SdfgiDensity, PackedTransportHistoryAndAtlasWithoutRt) {
  CheckSdfgiTransport(128, 64, 2);
}

TEST(SdfgiLighting, InjectionCadenceStaticReseedShadowsAndOverflowWithoutRt) {
  ScopedGpuPlatform platform(false);
  ApplicationContext::Get().RegisterAsset<Shader>("Shader", {".eveshader", ".slang"});
  EXPECT_FALSE(Platform::RayTracingEnabled());
  EXPECT_FALSE(Platform::RayQueryEnabled());
  EXPECT_FALSE(Platform::RayAccelerationStructureEnabled());
  SdfgiSettings settings;
  settings.voxel_count_x = settings.voxel_count_y = 128;
  settings.probe_spacing_cells = 8;
  settings.cascade_count = 1;
  settings.min_cell_size = 1;
  settings.vertical_scale = SdfgiSettings::VerticalScale::Percent100;
  std::string failure;
  const auto render = ApplicationContext::Get().GetLayer<RenderLayer>();
  auto field = SdfgiResources::TryCreate(settings, SdfgiTestAccess::HostLayouts(*render), failure);
  ASSERT_TRUE(field) << failure;
  std::vector<SdfgiCascade> cascades;
  ASSERT_TRUE(UpdateSdfgiCascades(settings, glm::vec3(0), cascades).empty());
  SdfgiSolidCell cell{};
  cell.position = 64 | (64 << 7) | (64 << 14) | (0x7ffu << 21);
  cell.albedo = 0x7fff | (1 << 15) | (0x7ffu << 21);
  cell.light = 128 | (256 << 8) | (128 << 17) | (14 << 25) | (3u << 30);
  cell.light_aniso = 31 | (3u << 30);
  SdfgiDispatchData dispatch{1, 1, 1, 1};
  BufferUploadBatch initial_upload;
  BufferUploadArena initial_arena;
  initial_upload.Add(field->buffers.at("Cascade0.SolidCells").buffer, cell);
  initial_upload.Add(field->buffers.at("Cascade0.UnlitCells").buffer, cell);
  initial_upload.Add(field->buffers.at("Cascade0.Dispatch").buffer, dispatch);
  initial_upload.Add(field->buffers.at("Cascade0.Indirect").buffer, dispatch);
  auto wall = std::make_shared<ComputePipeline>();
  wall->descriptor_set_layouts = {field->layouts[static_cast<size_t>(SdfgiLayout::Store)]};
  wall->compute_shader = Shader::CreateTemporary(ShaderType::Compute, std::string(R"(
[[vk::binding(7,0)]] [vk::image_format("r8")] RWTexture3D<float> sdf;
[numthreads(4,4,4)]
void main(uint3 id : SV_DispatchThreadID) {
  float d = abs(int(id.x) - 70);
  sdf[id] = d == 0 ? 0 : (d + 1) / 255.0;
}
)"));
  wall->Initialize();
  ASSERT_TRUE(wall->Initialized());
  std::shared_ptr<SdfgiLightDebug> debug;
  for (uint32_t phase = 0; phase < 21; ++phase) {
    SCOPED_TRACE(phase);
    PlatformLifecycleTestAccess::PreUpdate();
    std::vector<SdfgiLightInput> inputs(1);
    auto& light = inputs[0];
    light.id = 1;
    light.color = glm::vec3(phase == 0 || phase >= 11 ? 2 : 4);
    light.direction = {-1, 0, 0};
    light.position = {10.5f, 0.5f, 0.5f};
    light.range = 20;
    light.world_bounds = {glm::vec3(-20), glm::vec3(20)};
    light.attenuation = {1, 0.1f, 0.01f};
    float expected = phase == 0 || phase == 1 ? 2.25f : 4.25f;
    if (phase >= 3 && phase <= 9) {
      light.type = SdfgiLightInput::Type::Point;
      expected = 4.0f / 3.0f + 0.25f;
    }
    if (phase == 4 || phase == 5) {
      light.type = SdfgiLightInput::Type::Spot;
      light.direction = phase == 4 ? glm::vec3(-0.75f, std::sqrt(1 - 0.75f * 0.75f), 0) : glm::vec3(1, 0, 0);
      light.cos_inner = 0.9f;
      light.cos_outer = 0.6f;
      expected = phase == 4 ? 2.0f / 3.0f + 0.25f : 0.25f;
    }
    if (phase == 6) {
      light.range = 5;
      expected = 0.25f;
    }
    if (phase >= 7 && phase <= 9) {
      light.dynamic = false;
      light.attenuation = {1, 0, 0};
      light.color = glm::vec3(phase < 9 ? 3 : 1);
      expected = phase < 9 ? 3.25f : 1.25f;
    }
    if (phase == 10) {
      inputs.clear();
      expected = 0.25f;
    }
    if (phase == 11 || phase == 14)
      expected = 0.25f;
    if (phase == 13) {
      const auto repeated = light;
      inputs.resize(129, repeated);
      for (uint32_t i = 0; i < inputs.size(); ++i)
        inputs[i].id = i;
    }
    if (phase == 15) {
      inputs.clear();
      cell.light &= 3u << 30;
      cell.light_aniso &= 3u << 30;
      initial_upload.Add(field->buffers.at("Cascade0.SolidCells").buffer, cell);
      initial_upload.Add(field->buffers.at("Cascade0.UnlitCells").buffer, cell);
    }
    if (phase == 16) {
      light.dynamic = false;
      light.type = SdfgiLightInput::Type::Point;
      light.color = glm::vec3(0);
    }
    if (phase >= 15)
      expected = 0;
    if (phase >= 17) {
      inputs.clear();
      field->transport_recorded = phase >= 18;
      field->settings.bounce_feedback = phase == 20 ? SdfgiSettings{}.bounce_feedback : phase == 18 ? 0.0f : 0.5f;
      expected = phase == 20 ? 1.0f : phase == 19 ? 0.5f : 0;
    }
    const auto scene_frame = phase == 1 ? 1 : phase * 4;
    auto frame = SdfgiLightFrame::Create(*field, cascades, inputs, scene_frame, phase == 0 || phase == 15 ? 1 : 0);
    field->light_frames[Platform::GetCurrentFrameIndex()] = frame;
    EXPECT_FLOAT_EQ(frame->bounce_feedback, phase == 20 ? 1.0f : phase == 19 ? 0.5f : 0.0f);
    if (phase == 1 || phase == 8)
      EXPECT_EQ(frame->static_refresh, 0u);
    RenderGraph graph;
    RenderGraphResourceRegistry registry;
    field->Import(graph, registry);
    graph.AddPass({RenderPassNames::sdfgi_maintenance, RenderPassQueue::Graphics, RenderPassScope::Frame},
                  [](const RenderGraphExecutionContext&) {
                  });
    if (phase == 0)
      graph.AddPass(field->ClearDescriptor(), [&](const RenderGraphExecutionContext& context) {
        Platform::RecordCommandsMainQueue([&](const VkCommandBuffer command) {
          field->Clear(command, context);
        });
      });
    graph.AddPass(
        {"SdfgiLightSeed",
         RenderPassQueue::Graphics,
         RenderPassScope::Frame,
         {},
         {phase == 0 ? "SdfgiInitialize" : RenderPassNames::sdfgi_maintenance}},
        [&](const RenderGraphExecutionContext&) {
          if (phase == 0 || phase == 15)
            initial_upload.Record(initial_arena);
          Platform::RecordCommandsMainQueue([&](const VkCommandBuffer command) {
            field->OrderAccess(command, VK_PIPELINE_STAGE_2_TRANSFER_BIT, VK_ACCESS_2_TRANSFER_WRITE_BIT);
            const VkImageSubresourceRange range{VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1};
            if (phase == 0) {
              VkClearColorValue clear{};
              clear.float32[0] = 1;
              Platform::ClearColorImage(command, *field->textures.at("Cascade0.Sdf").image, clear, 1, &range);
              clear.uint32[0] = 0xffff;
              Platform::ClearColorImage(command, *field->textures.at("Occlusion").image, clear, 1, &range);
            }
            if (phase == 12 || phase == 13) {
              field->buffers.at("Status").buffer->Fill(command, offsetof(SdfgiFieldStatus, failure_flags), 4,
                                                       phase == 12 ? 1 : 0);
              VkClearColorValue clear{};
              clear.uint32[0] = 0xdeadbeefu;
              Platform::ClearColorImage(command, *field->textures.at("Cascade0.Light").image, clear, 1, &range);
            }
            if (phase == 17) {
              const VkImageSubresourceRange atlas_range{VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 2};
              VkClearColorValue clear{};
              clear.uint32[0] = 256 | (256 << 9) | (256 << 18) | (16u << 27);
              Platform::ClearColorImage(command, *field->textures.at("Atlas").image, clear, 1, &atlas_range);
            }
            if (phase == 11) {
              field->OrderAccess(command, VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT, VK_ACCESS_2_SHADER_WRITE_BIT);
              wall->Bind(command);
              wall->BindDescriptorSet(command, 0, field->sets.at("Cascade0.Store")->GetVkDescriptorSet());
              wall->Dispatch(command, 32, 32, 32);
            }
          });
        });
    frame->AddPasses(graph, field, "SdfgiLightSeed");
    if (phase == 0) {
      debug = std::make_shared<SdfgiLightDebug>(0, 64);
      debug->AddPass(graph, registry, field, "SdfgiDirectLight");
    }
    const auto plan = graph.Compile({});
    ASSERT_TRUE(plan.valid);
    EXPECT_FALSE(plan.uses_compute_queue);
    graph.Execute(plan, registry);
    PlatformLifecycleTestAccess::LateUpdate();
    Platform::WaitForFrameSubmissions("SDFGI direct-light fixture readback");
    EXPECT_EQ(field->light_failure.empty(), phase != 13);
    VkBufferImageCopy copy{};
    copy.imageSubresource = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 0, 1};
    copy.imageOffset = {63, 63, 63};
    copy.imageExtent = {3, 3, 3};
    Buffer pixels(27 * sizeof(uint32_t));
    pixels.CopyFromImage(*field->textures.at("Cascade0.Light").image, copy);
    std::vector<uint32_t> packed;
    pixels.DownloadVector(packed, 27);
    for (const auto value : packed) {
      if (phase == 12 || phase == 13)
        EXPECT_EQ(value, 0xdeadbeefu);
      else {
        const float scale = std::ldexp(1.0f, static_cast<int>(value >> 27) - 24);
        for (const auto shift : {0, 9, 18})
          EXPECT_NEAR(((value >> shift) & 511) * scale, expected, 0.016f);
      }
    }
    if (phase != 12 && phase != 13) {
      pixels.CopyFromImage(*field->textures.at("Cascade0.Aniso0").image, copy);
      pixels.DownloadVector(packed, 27);
      for (const auto value : packed)
        EXPECT_EQ(value, phase >= 15 && phase < 19 ? 0u : 255u);
    }
    if (phase == 16) {
      ASSERT_TRUE(debug->recorded);
      std::vector<uint32_t> old_planes;
      debug->planes[1]->DownloadVector(old_planes, 3 * 128 * 128);
      for (uint32_t axis = 0; axis < 3; ++axis)
        EXPECT_EQ(old_planes[axis * 128 * 128 + 64 * 128 + 64], 255u);
    }
  }
}

void CheckSdfgiPreprocess(const uint32_t voxel_x, const uint32_t voxel_y = 128, const uint32_t spacing = 8) {
  ScopedGpuPlatform platform(false);
  ApplicationContext::Get().RegisterAsset<Shader>("Shader", {".eveshader", ".slang"});
  SdfgiSettings settings;
  settings.probe_spacing_cells = spacing;
  settings.history_size = 5;
  settings.voxel_count_x = voxel_x;
  settings.voxel_count_y = voxel_y;
  const auto grid = settings.GridSize();
  const uint32_t volume = grid.x * grid.y * grid.z;
  const uint32_t width = grid.x;
  const glm::ivec3 center(voxel_x == 256 ? 192 : grid.x / 2, voxel_y > 128 ? voxel_y - 8 : grid.y / 2,
                          voxel_x == 256 ? 192 : grid.z / 2);
  settings.cascade_count = 1;
  const auto render = ApplicationContext::Get().GetLayer<RenderLayer>();
  std::string failure;
  auto field = SdfgiResources::TryCreate(settings, SdfgiTestAccess::HostLayouts(*render), failure);
  ASSERT_TRUE(field) << failure;
  EXPECT_FALSE(Platform::RayTracingEnabled());
  EXPECT_FALSE(Platform::RayQueryEnabled());
  EXPECT_FALSE(Platform::RayAccelerationStructureEnabled());
  auto seed = std::make_shared<ComputePipeline>();
  seed->descriptor_set_layouts = {field->layouts[static_cast<size_t>(SdfgiLayout::Store)]};
  seed->push_constant_ranges = {{VK_SHADER_STAGE_COMPUTE_BIT, 0, sizeof(uint32_t)}};
  seed->compute_shader = Shader::CreateTemporary(ShaderType::Compute, std::string(R"(
import EvoEngine.SdfgiTypes;
[[vk::push_constant]] ConstantBuffer<uint> pattern;
[[vk::binding(2,0)]] [vk::image_format("r16ui")] RWTexture3D<uint> albedo;
[[vk::binding(4,0)]] [vk::image_format("r32ui")] RWTexture3D<uint> emission;
[[vk::binding(5,0)]] [vk::image_format("r32ui")] RWTexture3D<uint> aniso;
[[vk::binding(6,0)]] [vk::image_format("r32ui")] RWTexture3D<uint> facing;
[[vk::binding(12,0)]] RWStructuredBuffer<SdfgiFieldStatus> status;
[numthreads(4,4,4)]
void main(uint3 id : SV_DispatchThreadID) {
  uint width, height, depth;
  albedo.GetDimensions(width, height, depth);
  bool solid = pattern == 0 ? all(id == uint3(width == 256 ? 192 : width / 2, height > 128 ? height - 8 : height / 2, depth == 256 ? 192 : depth / 2)) : pattern == 2 && all(id >= uint3(32)) && all(id < uint3(64));
  if (solid) {
    albedo[id] = 1u | (12u << 11) | (6u << 6) | (3u << 1);
    emission[id] = (17u << 25) | 128u;
    aniso[id] = 31;
    facing[id] = 63;
  }
  if (all(id == uint3(0))) status[0].solid_cell_capacity = pattern == 2 ? 64 : width * height * depth / 4;
}
)"));
  seed->Initialize();
  ASSERT_TRUE(seed->Initialized());
  auto check_layout = std::make_shared<DescriptorSetLayout>();
  check_layout->PushDescriptorBinding(0, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_COMPUTE_BIT, 0);
  check_layout->Initialize();
  VkBufferCreateInfo info{VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO};
  info.size = sizeof(uint32_t);
  info.usage = VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_SRC_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT;
  const auto counter = std::make_shared<Buffer>(info);
  auto check_set = std::make_shared<DescriptorSet>(check_layout);
  check_set->UpdateBufferDescriptorBinding(0, counter);
  auto check = std::make_shared<ComputePipeline>();
  check->descriptor_set_layouts = {check_layout};
  check->compute_shader = Shader::CreateTemporary(ShaderType::Compute, std::string(R"(
[[vk::binding(0,0)]] RWStructuredBuffer<uint> count;
[numthreads(64,1,1)]
void main(uint3 id : SV_DispatchThreadID) { InterlockedAdd(count[0], 1); }
)"));
  check->Initialize();
  ASSERT_TRUE(check->Initialized());
  for (uint32_t pattern = 0; pattern < 3; ++pattern) {
    SCOPED_TRACE(pattern);
    PlatformLifecycleTestAccess::PreUpdate();
    auto readback = std::make_shared<SdfgiPreprocessReadback>(1);
    RenderGraph graph;
    RenderGraphResourceRegistry registry;
    field->Import(graph, registry);
    graph.AddPass({RenderPassNames::sdfgi_maintenance, RenderPassQueue::Graphics, RenderPassScope::Frame},
                  [](const RenderGraphExecutionContext&) {
                  });
    graph.AddPass(field->ClearDescriptor(), [&](const RenderGraphExecutionContext& context) {
      Platform::RecordCommandsMainQueue([&](const VkCommandBuffer command) {
        field->Clear(command, context);
      });
    });
    graph.AddPass({"SdfgiSeed", RenderPassQueue::Graphics, RenderPassScope::Frame, {}, {"SdfgiInitialize"}},
                  [&](const RenderGraphExecutionContext&) {
                    Platform::RecordCommandsMainQueue([&](const VkCommandBuffer command) {
                      field->OrderAccess(command, VK_PIPELINE_STAGE_2_TRANSFER_BIT, VK_ACCESS_2_TRANSFER_WRITE_BIT);
                      field->buffers.at("Cascade0.SolidCells").buffer->Fill(command, 0, VK_WHOLE_SIZE, 0xdeadbeefu);
                      counter->Fill(command, 0, VK_WHOLE_SIZE, 0);
                      field->OrderAccess(command, VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT, VK_ACCESS_2_SHADER_WRITE_BIT);
                      seed->Bind(command);
                      seed->BindDescriptorSet(command, 0, field->sets.at("Cascade0.Store")->GetVkDescriptorSet());
                      seed->PushConstant(command, 0, pattern);
                      seed->Dispatch(command, grid.x / 4, grid.y / 4, grid.z / 4);
                    });
                  });
    const auto previous = AddSdfgiPreprocessPass(graph, registry, field, readback, 0, glm::ivec3(0), "SdfgiSeed");
    graph.AddPass({"SdfgiIndirectCheck", RenderPassQueue::Graphics, RenderPassScope::Frame, {}, {previous}},
                  [&](const RenderGraphExecutionContext&) {
                    Platform::RecordCommandsMainQueue([&](const VkCommandBuffer command) {
                      field->OrderAccess(command,
                                         VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT | VK_PIPELINE_STAGE_2_DRAW_INDIRECT_BIT,
                                         VK_ACCESS_2_SHADER_READ_BIT | VK_ACCESS_2_SHADER_WRITE_BIT |
                                             VK_ACCESS_2_INDIRECT_COMMAND_READ_BIT);
                      check->Bind(command);
                      check->BindDescriptorSet(command, 0, check_set->GetVkDescriptorSet());
                      check->DispatchIndirect(command, *field->buffers.at("Cascade0.Indirect").buffer);
                    });
                  });
    const auto plan = graph.Compile({});
    ASSERT_TRUE(plan.valid);
    EXPECT_FALSE(plan.uses_compute_queue);
    graph.Execute(plan, registry);
    PlatformLifecycleTestAccess::LateUpdate();
    Platform::WaitForFrameSubmissions("SDFGI focused preprocess readback");
    readback->ReadAfterFrameFence(*field);
    ASSERT_TRUE(field->preprocess_status_available);
    EXPECT_EQ(field->preprocess_status.failure_flags, pattern == 2 ? kSdfgiFailureSolidOverflow : 0u);
    EXPECT_EQ(field->preprocess_status.ready, 0u);
    ASSERT_EQ(field->solid_cell_dispatch.size(), 1u);
    const auto& dispatch = field->solid_cell_dispatch[0];
    EXPECT_EQ(dispatch.total_count, pattern == 2 ? 32768u : 1u);
    EXPECT_EQ(dispatch.x, 1u);
    EXPECT_EQ(dispatch.y, 1u);
    EXPECT_EQ(dispatch.z, 1u);
    uint32_t invocations = 0;
    counter->Download(invocations);
    EXPECT_EQ(invocations, 64u);
    std::vector<SdfgiSolidCell> cells;
    field->buffers.at("Cascade0.SolidCells").buffer->DownloadVector(cells, 65);
    std::vector<SdfgiSolidCell> unlit;
    field->buffers.at("Cascade0.UnlitCells").buffer->DownloadVector(unlit, 65);
    EXPECT_EQ(std::memcmp(cells.data(), unlit.data(), 65 * sizeof(SdfgiSolidCell)), 0);
    EXPECT_EQ(cells[pattern == 2 ? 64 : 1].position, 0xdeadbeefu);
    if (pattern == 0) {
      EXPECT_EQ(cells[0].position & 0x1fffffu, SdfgiTestCell(center).position);
      EXPECT_EQ(cells[0].position_high, SdfgiTestCell(center).position_high);
      EXPECT_EQ(cells[0].albedo & 0x7fffu, (12u << 10) | (6u << 5) | 3u);
      EXPECT_EQ((cells[0].albedo >> 15) & 63, 63u);
      EXPECT_EQ((cells[0].albedo >> 21) | ((cells[0].position >> 21) << 11) | ((cells[0].light >> 30) << 22) |
                    ((cells[0].light_aniso >> 30) << 24),
                0x3ffffffu);
      EXPECT_EQ(cells[0].light & 0x3fffffffu, (17u << 25) | 128u);
      EXPECT_EQ(cells[0].light_aniso & 0x3fffffffu, 31u);
      Buffer pixels(volume);
      VkBufferImageCopy copy{};
      copy.imageSubresource = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 0, 1};
      copy.imageExtent = {uint32_t(grid.x), uint32_t(grid.y), uint32_t(grid.z)};
      pixels.CopyFromImage(*field->textures.at("Cascade0.Sdf").image, copy);
      std::vector<uint8_t> sdf;
      pixels.DownloadVector(sdf, volume);
      for (int z = 0; z < grid.z; z += 7)
        for (int y = 0; y < grid.y; y += 7)
          for (int x = 0; x < grid.x; x += 7) {
            const float distance = glm::length(glm::vec3(glm::ivec3(x, y, z) - center));
            // Vulkan permits either adjacent integer for floating-point to UNORM conversion.
            const float encoded = distance == 0 ? 0 : std::min(255.0f, distance + 1);
            EXPECT_GE(sdf[x + y * width + z * width * grid.y], std::floor(encoded));
            EXPECT_LE(sdf[x + y * width + z * width * grid.y], std::ceil(encoded));
          }
    }
    if (pattern == 1) {
      // Godot STORE deliberately tests the xyz position, not w; its empty-grid origin sentinel is preserved.
      EXPECT_EQ(cells[0].position & 0x1fffffu, 0u);
      EXPECT_EQ(cells[0].position_high, 0u);
      EXPECT_EQ(cells[0].albedo & 0x1fffffu, 0u);
      Buffer pixels(volume * 2 * 2);
      VkBufferImageCopy copy{};
      copy.imageSubresource = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 0, 1};
      copy.imageExtent = {2 * width, uint32_t(grid.y), width};
      pixels.CopyFromImage(*field->textures.at("Occlusion").image, copy);
      std::vector<uint16_t> occlusion;
      pixels.DownloadVector(occlusion, volume * 2);
      for (uint32_t i = 0; i < occlusion.size(); i += 1331)
        EXPECT_EQ(occlusion[i], 0xffffu);
    }
  }
  PlatformLifecycleTestAccess::PreUpdate();
  Platform::RecordCommandsMainQueue([&](const VkCommandBuffer command) {
    field->OrderAccess(command, VK_PIPELINE_STAGE_2_TRANSFER_BIT, VK_ACCESS_2_TRANSFER_WRITE_BIT);
    field->buffers.at("Cascade0.Dispatch").buffer->Fill(command, 0, VK_WHOLE_SIZE, 0);
    const VkImageSubresourceRange range{VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1};
    const std::array<float, 8> values{0, 1.0f / 3, 2.0f / 3, 1, 1, 2.0f / 3, 1.0f / 3, 0};
    for (uint32_t i = 0; i < 8; ++i) {
      VkClearColorValue color{};
      color.float32[0] = values[i];
      Platform::ClearColorImage(command, *field->textures.at("OcclusionScratch" + std::to_string(i)).image, color, 1,
                                &range);
    }
    field->OrderAccess(command, VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT,
                       VK_ACCESS_2_SHADER_READ_BIT | VK_ACCESS_2_SHADER_WRITE_BIT);
    const auto& store = field->pipelines.at("Store");
    store->Bind(command);
    store->BindDescriptorSet(command, 0, field->sets.at("Cascade0.Store")->GetVkDescriptorSet());
    SdfgiPreprocessPushConstant params{};
    params.grid_size = width;
    params.grid_size_y = grid.y;
    store->PushConstant(command, 0, params);
    store->Dispatch(command, grid.x / 4, grid.y / 4, grid.z / 4);
  });
  PlatformLifecycleTestAccess::LateUpdate();
  Platform::WaitForFrameSubmissions("SDFGI occlusion channel packing readback");
  Buffer pixels(volume * 2 * 2);
  VkBufferImageCopy copy{};
  copy.imageSubresource = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 0, 1};
  copy.imageExtent = {2 * width, uint32_t(grid.y), width};
  pixels.CopyFromImage(*field->textures.at("Occlusion").image, copy);
  std::vector<uint16_t> occlusion;
  pixels.DownloadVector(occlusion, volume * 2);
  const uint32_t sample = width / 2 + (grid.y / 2) * width * 2 + (width / 2) * width * 2 * grid.y;
  EXPECT_EQ(occlusion[sample], 0x05afu);
  EXPECT_EQ(occlusion[sample + width], 0xfa50u);
}

TEST(SdfgiPreprocess, DistanceOcclusionPackingAndBoundedIndirectOverflowWithoutRt) {
  CheckSdfgiPreprocess(128);
}

TEST(SdfgiWide, DistanceOcclusionPackingAndBoundedIndirectOverflowWithoutRt) {
  CheckSdfgiPreprocess(256);
}

TEST(SdfgiConfigurable, PreprocessTallNonPowerOfTwoWithoutRt) {
  CheckSdfgiPreprocess(80, 144);
}

TEST(SdfgiDensity, SmallSpacingPreprocessWithoutRt) {
  for (const uint32_t spacing : {1u, 2u, 4u}) {
    SCOPED_TRACE(spacing);
    CheckSdfgiPreprocess(64, 64, spacing);
  }
}

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
  EXPECT_EQ(cubemap.GetImage()->GetLayout(), VK_IMAGE_LAYOUT_GENERAL);
}

TEST(GpuService, CubemapUninitializedGpuStorageRejectsReadback) {
  ScopedGpuPlatform platform;
  Cubemap cubemap;
  cubemap.Initialize(2u);
  ASSERT_TRUE(cubemap.GetImage());
  ASSERT_EQ(cubemap.GetImage()->GetLayout(), VK_IMAGE_LAYOUT_GENERAL);

  std::vector<glm::vec4> pixels = {glm::vec4(1.0f)};
  cubemap.GetRgbaChannelData(pixels);
  EXPECT_TRUE(pixels.empty());
  EXPECT_TRUE(cubemap.PeekLocalData().empty());
  EXPECT_EQ(cubemap.GetImage()->GetLayout(), VK_IMAGE_LAYOUT_GENERAL);
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
  descriptor_image_info.imageLayout = VK_IMAGE_LAYOUT_GENERAL;
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
    image->TransitImageLayout(command_buffer, VK_IMAGE_LAYOUT_GENERAL);
    VkBufferImageCopy copy_region{};
    copy_region.imageSubresource.aspectMask = VK_IMAGE_ASPECT_DEPTH_BIT;
    copy_region.imageSubresource.layerCount = 1;
    copy_region.imageExtent = {2, 1, 1};
    image->CopyFromBuffer(command_buffer, staging_buffer.GetVkBuffer(), {copy_region});
    image->TransitImageLayout(command_buffer, VK_IMAGE_LAYOUT_GENERAL);
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

TEST(GpuService, DdgiQuantizedHistoryStartupReplacementRejectionAndReset) {
  ScopedGpuPlatform platform(false);
  ApplicationContext::Get().RegisterAsset<Shader>("Shader", {".eveshader", ".slang"});
  DdgiHistoryLayout layout;
  ASSERT_TRUE(DdgiHistoryLayout::Calculate(6, 1, 1, 30, layout));
  GiHistoryBudget budget;
  std::string error;
  ASSERT_TRUE(DdgiRuntime::AddDeviceHistoryAllocations(layout, budget, error)) << error;
  EXPECT_GE(budget.bytes, std::accumulate(layout.buffer_bytes.begin(), layout.buffer_bytes.end(), uint64_t{0}));
  const auto make_buffer = [](const uint64_t size) {
    VkBufferCreateInfo info{VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO};
    info.size = size;
    info.usage =
        VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_SRC_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT;
    info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
    VmaAllocationCreateInfo allocation{};
    allocation.usage = VMA_MEMORY_USAGE_AUTO;
    return std::make_shared<Buffer>(info, allocation);
  };
  const auto history_layout = std::make_shared<DescriptorSetLayout>();
  for (uint32_t binding = 7; binding < 7 + DdgiHistoryLayout::BufferCount; ++binding)
    history_layout->PushDescriptorBinding(binding, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_COMPUTE_BIT, 0);
  history_layout->Initialize();
  const auto output_layout = std::make_shared<DescriptorSetLayout>();
  output_layout->PushDescriptorBinding(31, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_COMPUTE_BIT, 0);
  output_layout->Initialize();
  const auto history_set = std::make_shared<DescriptorSet>(history_layout);
  std::array<std::shared_ptr<Buffer>, DdgiHistoryLayout::BufferCount> history;
  for (size_t i = 0; i < history.size(); ++i) {
    history[i] = make_buffer(layout.buffer_bytes[i]);
    history_set->UpdateBufferDescriptorBinding(static_cast<uint32_t>(7 + i), history[i]);
  }
  const auto output = make_buffer(6 * sizeof(glm::uvec4));
  const auto output_set = std::make_shared<DescriptorSet>(output_layout);
  output_set->UpdateBufferDescriptorBinding(31, output);
  const auto pipeline = std::make_shared<ComputePipeline>();
  pipeline->descriptor_set_layouts = {output_layout, history_layout};
  pipeline->compute_shader =
      Shader::CreateTemporary(ShaderType::Compute, Platform::GetShaderGlobalDefines(),
                              std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) /
                                  "EvoEngine_Tests/Resources/Shaders/Compute/DdgiRollingHistoryProbe.slang");
  pipeline->Initialize();
  ASSERT_TRUE(pipeline->Initialized());
  Platform::GetGpuService().SubmitImmediate([&](const VkCommandBuffer command) {
    pipeline->Bind(command);
    pipeline->BindDescriptorSet(command, 0, output_set->GetVkDescriptorSet());
    pipeline->BindDescriptorSet(command, 1, history_set->GetVkDescriptorSet());
    pipeline->Dispatch(command, 6);
  });
  std::vector<glm::uvec4> results;
  output->DownloadVector(results, 6);
  for (uint32_t i = 0; i < 6; ++i) {
    const auto count = (i + 1) * 5;
    uint32_t irradiance_sum = 0;
    for (uint32_t value = 1; value <= count; ++value) {
      irradiance_sum += QuantizeDdgiHistorySample(static_cast<float>(value), 64.0f);
    }
    EXPECT_EQ(results[i].x, 0u) << count;
    EXPECT_EQ(results[i].y, irradiance_sum);
    EXPECT_NEAR(glm::uintBitsToFloat(results[i].z), (count + 1.0f) * 0.5f, 0.001f);
    EXPECT_NEAR(glm::uintBitsToFloat(results[i].w), 1.0f / count, 0.001f);
  }
}

TEST(GpuService, DdgiProductionHistoryInvalidatesMovedAndReactivatedProbesOnly) {
  ScopedGpuPlatform platform(false);
  ApplicationContext::Get().RegisterAsset<Shader>("Shader", {".eveshader", ".slang"});
  const auto make_buffer = [](uint64_t bytes) {
    VkBufferCreateInfo info{VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO};
    info.size = bytes;
    info.usage =
        VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_SRC_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT;
    return std::make_shared<Buffer>(info);
  };
  DdgiHistoryLayout layout;
  ASSERT_TRUE(DdgiHistoryLayout::Calculate(3, 1, 1, 5, layout));
  const auto descriptors = std::make_shared<DescriptorSetLayout>();
  for (uint32_t binding = 0; binding < 7 + DdgiHistoryLayout::BufferCount; ++binding)
    if (binding != 5)
      descriptors->PushDescriptorBinding(
          binding, binding == 1 || binding == 2 ? VK_DESCRIPTOR_TYPE_STORAGE_IMAGE : VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,
          VK_SHADER_STAGE_COMPUTE_BIT, 0);
  descriptors->Initialize();
  const auto set = std::make_shared<DescriptorSet>(descriptors);
  const auto empty_layout = std::make_shared<DescriptorSetLayout>();
  empty_layout->Initialize();
  const auto empty_set = std::make_shared<DescriptorSet>(empty_layout);
  const auto state = make_buffer(3 * sizeof(glm::vec4));
  const std::array<glm::vec4, 3> origins{glm::vec4(0), glm::vec4(0), glm::vec4(0, 0, 0, 1)};
  const std::array<glm::vec4, 3> moved{glm::vec4(0), glm::vec4(0.2f, 0, 0, 0), glm::vec4(0)};
  state->Upload(moved);
  for (const uint32_t binding : {0u, 3u, 4u, 6u})
    set->UpdateBufferDescriptorBinding(binding, state);
  std::array<std::shared_ptr<Buffer>, DdgiHistoryLayout::BufferCount> history;
  for (size_t i = 0; i < history.size(); ++i) {
    history[i] = make_buffer(layout.buffer_bytes[i]);
    set->UpdateBufferDescriptorBinding(static_cast<uint32_t>(7 + i), history[i]);
    history[i]->UploadVector(std::vector<uint32_t>(layout.buffer_bytes[i] / 4, 7));
  }
  history[DdgiHistoryLayout::ProbeOrigins]->Upload(origins);
  std::array<std::shared_ptr<Image>, 2> images;
  std::array<std::shared_ptr<ImageView>, 2> views;
  for (size_t i = 0; i < images.size(); ++i) {
    VkImageCreateInfo info{VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO};
    info.imageType = VK_IMAGE_TYPE_2D;
    info.format = i == 0 ? VK_FORMAT_R16G16B16A16_SFLOAT : VK_FORMAT_R16G16_SFLOAT;
    info.extent = {9, 3, 1};
    info.mipLevels = info.arrayLayers = 1;
    info.samples = VK_SAMPLE_COUNT_1_BIT;
    info.usage = VK_IMAGE_USAGE_STORAGE_BIT | VK_IMAGE_USAGE_TRANSFER_DST_BIT | VK_IMAGE_USAGE_TRANSFER_SRC_BIT;
    images[i] = std::make_shared<Image>(info);
    VkImageViewCreateInfo view{VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO};
    view.image = images[i]->GetVkImage();
    view.viewType = VK_IMAGE_VIEW_TYPE_2D;
    view.format = info.format;
    view.subresourceRange = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1};
    views[i] = std::make_shared<ImageView>(view, images[i]);
    VkDescriptorImageInfo descriptor{};
    descriptor.imageLayout = VK_IMAGE_LAYOUT_GENERAL;
    descriptor.imageView = views[i]->GetVkImageView();
    set->UpdateImageDescriptorBinding(static_cast<uint32_t>(1 + i), descriptor);
  }
  const auto pipeline = std::make_shared<ComputePipeline>();
  pipeline->descriptor_set_layouts = {empty_layout, descriptors};
  pipeline->push_constant_ranges = {{VK_SHADER_STAGE_COMPUTE_BIT, 0, sizeof(DdgiProbeAtlasUpdatePushConstant)}};
  pipeline->compute_shader =
      Shader::CreateTemporary(ShaderType::Compute, Platform::GetShaderGlobalDefines(),
                              std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) /
                                  "EvoEngine_SDK/Internals/DefaultResources/Shaders/Compute/DDGIProbeUpdate.slang");
  pipeline->Initialize();
  ASSERT_TRUE(pipeline->Initialized());
  DdgiProbeAtlasUpdatePushConstant constants{};
  constants.probe_count_ray_count_and_tile_sizes = {3, 1, 1, 1};
  constants.atlas_columns_fixed_ray_count_and_update_mode = {3, 3, 0, 3};
  constants.probe_counts_and_rotation = {3, 1, 1, 0};
  constants.blend_parameters.z = 5;
  // Signed scrolling must still invalidate each physical slot exactly once.
  constants.probe_scroll_offset = {-1, 0, 0, 1};
  Platform::GetGpuService().SubmitImmediate([&](VkCommandBuffer command) {
    for (const auto& image : images) {
      image->TransitImageLayout(command, VK_IMAGE_LAYOUT_GENERAL);
      VkClearColorValue color{{1, 1, 1, 1}};
      const VkImageSubresourceRange range{VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1};
      Platform::ClearColorImage(command, *image, color, 1, &range);
    }
    Platform::EverythingBarrier(command);
    pipeline->Bind(command);
    pipeline->BindDescriptorSet(command, 0, empty_set->GetVkDescriptorSet());
    pipeline->BindDescriptorSet(command, 1, set->GetVkDescriptorSet());
    pipeline->PushConstant(command, 0, constants);
    pipeline->Dispatch(command, 1);
    Platform::EverythingBarrier(command);
    pipeline->Dispatch(command, 1);
    Platform::EverythingBarrier(command);
  });
  for (size_t i = 0; i < DdgiHistoryLayout::BufferCount; ++i) {
    if (i == DdgiHistoryLayout::ProbeOrigins)
      continue;
    std::vector<uint32_t> values;
    history[i]->DownloadVector(values, layout.buffer_bytes[i] / 4);
    for (size_t word = 0; word < values.size(); ++word)
      EXPECT_EQ(values[word], word < values.size() / 3 ? 7u : 0u) << i << ":" << word;
  }
  std::array<glm::vec4, 3> recorded{};
  history[DdgiHistoryLayout::ProbeOrigins]->Download(recorded);
  EXPECT_EQ(recorded, moved);
  for (size_t i = 0; i < images.size(); ++i) {
    const uint32_t components = i == 0 ? 4 : 2;
    const auto output = make_buffer(9 * 3 * components * sizeof(uint16_t));
    output->CopyFromImage(*images[i], components * sizeof(uint16_t));
    std::vector<uint16_t> values;
    output->DownloadVector(values, 9 * 3 * components);
    for (size_t word = 0; word < values.size(); ++word)
      EXPECT_EQ(values[word], (word / components) % 9 < 3 ? 0x3c00u : 0u);
  }
  const auto rays = make_buffer(3 * sizeof(glm::vec4));
  rays->UploadVector(std::vector<glm::vec4>(3, glm::vec4(std::numeric_limits<float>::quiet_NaN())));
  const auto metadata = make_buffer(9 * sizeof(glm::vec4));
  set->UpdateBufferDescriptorBinding(0, rays);
  set->UpdateBufferDescriptorBinding(3, metadata);
  constants.update_parameters = {10, 1, 1, 0};
  Platform::GetGpuService().SubmitImmediate([&](VkCommandBuffer command) {
    for (const auto& image : images)
      image->TransitImageLayout(command, VK_IMAGE_LAYOUT_GENERAL);
    pipeline->Bind(command);
    pipeline->BindDescriptorSet(command, 0, empty_set->GetVkDescriptorSet());
    pipeline->BindDescriptorSet(command, 1, set->GetVkDescriptorSet());
    for (uint32_t mode : {1u, 2u}) {
      constants.atlas_columns_fixed_ray_count_and_update_mode.w = mode;
      pipeline->PushConstant(command, 0, constants);
      pipeline->Dispatch(command, 1);
      Platform::EverythingBarrier(command);
    }
  });
  for (size_t i = 0; i < DdgiHistoryLayout::BufferCount; ++i) {
    if (i == DdgiHistoryLayout::ProbeOrigins)
      continue;
    std::vector<uint32_t> values;
    history[i]->DownloadVector(values, layout.buffer_bytes[i] / 4);
    for (size_t word = 0; word < values.size(); ++word)
      EXPECT_EQ(values[word], word < values.size() / 3 ? 7u : 0u)
          << "Rejected sample changed history " << i << ":" << word;
  }
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
  EXPECT_EQ(texture_storage.GetLayout(), VK_IMAGE_LAYOUT_GENERAL);
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
  EXPECT_EQ(texture_storage.GetLayout(), VK_IMAGE_LAYOUT_GENERAL);

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
  EXPECT_EQ(texture_storage.GetLayout(), VK_IMAGE_LAYOUT_GENERAL);
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
  ASSERT_EQ(downloaded_bytes.size(), sizeof(VertexDataChunk));
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

TEST(GpuService, BloomBoundedSourcePreservesHueAndLimitsExtremeEmission) {
  ScopedGpuPlatform platform;
  Shader::RegisterShaderIncludePath(std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) /
                                    "EvoEngine_SDK/Internals/DefaultResources/Shaders/Modules");
  auto layout = std::make_shared<DescriptorSetLayout>();
  layout->PushDescriptorBinding(0, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_COMPUTE_BIT, 0);
  layout->Initialize();
  constexpr size_t count = 21;
  VkBufferCreateInfo info{VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO};
  info.size = count * sizeof(glm::vec4);
  info.usage = VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_SRC_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT;
  VmaAllocationCreateInfo allocation{};
  allocation.usage = VMA_MEMORY_USAGE_AUTO_PREFER_DEVICE;
  auto output = std::make_shared<Buffer>(info, allocation);
  auto descriptor = std::make_shared<DescriptorSet>(layout);
  descriptor->UpdateBufferDescriptorBinding(0, output);
  auto shader = std::make_shared<Shader>();
  ASSERT_TRUE(shader->TryCompile(ShaderType::Compute, std::string(R"(
import EvoEngine.PostProcessing;
[[vk::binding(0, 0)]] RWStructuredBuffer<float4> output;
[shader("compute")]
[numthreads(1, 1, 1)]
void main() {
  const float brightness[12] = {0, 1, 2, 2.001, 3, 8, 20, 100, 10000, 1e20, 1e30, 1e38};
  for (uint i = 0; i < 12; ++i)
    output[i] = float4(EeLimitBloomSource(float3(1, 0.25, 0.5) * brightness[i], 2, 8), 1);
  output[12] = float4(EeLimitBloomSource(float3(20, 5, 10), 4, 4), 1);
  output[13] = float4(EeLimitBloomSource(float3(20, 5, 10), 0, 0), 1);
  output[14] = float4(EeLimitBloomSource(float3(-1, 2, 0), 2, 8), 1);
  output[15] = float4(EePrefilterBloom(float3(0.5, 0.25, 0.125), 1, 0.1, 2, 8), 1);
  output[16] = float4(EePrefilterBloom(float3(4, 2, 1), 1, 0.1, 2, 8), 1);
  output[17] = float4(EeLimitBloomSource(float3(20, 5, 10), 2, 4), 1);
  output[18] = float4(EeLimitBloomSource(float3(20, 5, 10), 9, -1), 1);
  output[19] = float4(EeLimitBloomSource(float3(asfloat(0x7f800000u), 0, 0), 2, 8), 1);
  output[20] = float4(EePrefilterBloom(float3(asfloat(0x7f800000u), 0, 0), 1, 0.1, 2, 8), 1);
})")));
  ComputePipeline pipeline;
  pipeline.compute_shader = shader;
  pipeline.descriptor_set_layouts.push_back(layout);
  pipeline.Initialize();
  ASSERT_TRUE(pipeline.Initialized());
  Platform::ImmediateSubmit([&](const VkCommandBuffer command) {
    pipeline.Bind(command);
    pipeline.BindDescriptorSet(command, 0, descriptor->GetVkDescriptorSet());
    pipeline.Dispatch(command, 1);
    Platform::BufferMemoryBarrier(command, *output, VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT,
                                  VK_ACCESS_2_SHADER_WRITE_BIT, VK_PIPELINE_STAGE_2_TRANSFER_BIT,
                                  VK_ACCESS_2_TRANSFER_READ_BIT);
  });
  std::array<glm::vec4, count> values{};
  output->Download(values);
  const std::array<double, 12> brightness{0, 1, 2, 2.001, 3, 8, 20, 100, 10000, 1e20, 1e30, 1e38};
  for (size_t i = 0; i < brightness.size(); ++i) {
    const double x = brightness[i];
    const double expected = x <= 2 ? x : 2 + 6 * ((x - 2) / (6 + x - 2));
    EXPECT_NEAR(values[i].x, expected, 2e-6);
    EXPECT_NEAR(values[i].y, expected * 0.25, 2e-6);
    EXPECT_NEAR(values[i].z, expected * 0.5, 2e-6);
    EXPECT_LE(values[i].x, 8);
    if (i != 0)
      EXPECT_GE(values[i].x, values[i - 1].x);
  }
  for (const auto& value : values)
    for (int channel = 0; channel < 4; ++channel)
      EXPECT_TRUE(std::isfinite(value[channel]));
  EXPECT_EQ(values[12], glm::vec4(4, 1, 2, 1));
  EXPECT_EQ(values[13], glm::vec4(0, 0, 0, 1));
  EXPECT_EQ(values[14], glm::vec4(0, 2, 0, 1));
  EXPECT_EQ(values[15], glm::vec4(0, 0, 0, 1));
  EXPECT_NEAR(values[16].x, 2 + 6.0 / 7, 2e-6);
  EXPECT_NEAR(values[16].y, values[16].x * 0.5, 2e-6);
  EXPECT_NEAR(values[16].z, values[16].x * 0.25, 2e-6);
  EXPECT_LT(values[17].x, values[6].x);
  EXPECT_LT(values[17].x, 4);
  EXPECT_EQ(values[18], glm::vec4(0, 0, 0, 1));
  EXPECT_EQ(values[19], glm::vec4(8, 0, 0, 1));
  EXPECT_EQ(values[20], glm::vec4(8, 0, 0, 1));
}

TEST(HddagiResources, PreflightAndPartialAllocationWithoutRayTracing) {
  ScopedGpuPlatform platform(false);
  GiProbeSettings probes;
  probes.probe_count_x = probes.probe_count_y = 9;
  probes.cascade_count = 1;
  HddagiSettings settings;
  const auto report = QueryHddagiCapabilities(probes, settings);
  ASSERT_TRUE(report.Supported()) << report.failure;
  const auto default_report = QueryHddagiCapabilities({}, settings);
  ASSERT_TRUE(default_report.Supported()) << default_report.failure;
  std::cout << "HDDAGI device: " << report.device_name << "; default image bytes: " << default_report.image_bytes
            << "; temporal bytes: " << default_report.temporal_bytes
            << "; buffer bytes: " << default_report.buffer_bytes << std::endl;
  EXPECT_FALSE(Platform::RayTracingEnabled());
  EXPECT_FALSE(Platform::RayQueryEnabled());
  EXPECT_FALSE(Platform::RayAccelerationStructureEnabled());
  EXPECT_GE(report.temporal_bytes, HddagiLogicalTemporalBytes(probes, settings));
  std::string failure;
  EXPECT_FALSE(HddagiResources::TryCreate(probes, settings, failure, 3));
  EXPECT_NE(failure.find("Injected"), std::string::npos);
  EXPECT_FALSE(
      HddagiResources::TryCreate(probes, settings, failure, GetHddagiImageRequirements(probes, settings).size() + 1));
  EXPECT_NE(failure.find("Injected"), std::string::npos);
  auto field = HddagiResources::TryCreate(probes, settings, failure);
  ASSERT_TRUE(field) << failure;
  EXPECT_EQ(field->images.size(), GetHddagiImageRequirements(probes, settings).size());
  EXPECT_GE(field->temporal_bytes, HddagiLogicalTemporalBytes(probes, settings));
  EXPECT_EQ(field->buffers.size(), 5u);
  EXPECT_GE(field->AllocationBytes(), report.image_bytes + report.buffer_bytes);
  const std::weak_ptr<HddagiResources> weak = field;
  auto retained = field;
  field.reset();
  EXPECT_FALSE(weak.expired());
  retained.reset();
  EXPECT_TRUE(weak.expired());
}

TEST(HddagiRuntime, ProviderSwitchRetainsSettingsAndNeverPublishesUnreadyTransport) {
  ScopedGpuPlatform platform(false);
  const auto render = ApplicationContext::Get().GetLayer<RenderLayer>();
  const auto scene = std::make_shared<Scene>();
  const auto lighting = std::make_shared<EnvironmentalLighting>();
  lighting->indirect_gi_provider = IndirectGiProvider::AutomaticHddagi;
  scene->environmental_lighting = lighting;
  SdfgiTestAccess::ExecuteSceneFrame(*render, scene);
  ASSERT_TRUE(scene->GetHddagiRuntime());
  EXPECT_FALSE(scene->GetHddagiRuntime()->published);
  EXPECT_FALSE(scene->GetSdfgiRuntime());
  EXPECT_FALSE(SdfgiTestAccess::HasDdgiResources(*render));
  EXPECT_FALSE(scene->GetHddagiRuntime()->fallback_reason.empty());
  const std::weak_ptr<const HddagiRuntime> previous = scene->GetHddagiRuntime();
  lighting->indirect_gi_provider = IndirectGiProvider::Environment;
  SdfgiTestAccess::ExecuteSceneFrame(*render, scene);
  EXPECT_FALSE(scene->GetHddagiRuntime());
  EXPECT_TRUE(previous.expired());
  EXPECT_EQ(lighting->hddagi_settings.history_size, 12u);
  lighting->indirect_gi_provider = IndirectGiProvider::AutomaticHddagi;
  SdfgiTestAccess::ValidateGiSettings(*render, scene);
  const auto accepted = lighting->GetGiSettings();
  lighting->hddagi_settings.history_size = 7;
  SdfgiTestAccess::ValidateGiSettings(*render, scene);
  EXPECT_EQ(lighting->GetGiSettings(), accepted);
  SdfgiTestAccess::ExecuteSceneFrame(*render, scene);
  ASSERT_TRUE(scene->GetHddagiRuntime());
  const auto next_scene = std::make_shared<Scene>();
  const auto unsupported = std::make_shared<EnvironmentalLighting>();
  unsupported->indirect_gi_provider = IndirectGiProvider::AutomaticHddagi;
  unsupported->gi_probe_settings.probe_count_x = 7;
  next_scene->environmental_lighting = unsupported;
  SdfgiTestAccess::ExecuteSceneFrame(*render, next_scene);
  EXPECT_FALSE(scene->GetHddagiRuntime());
  ASSERT_TRUE(next_scene->GetHddagiRuntime());
  EXPECT_FALSE(next_scene->GetHddagiRuntime()->capabilities.Supported());
  EXPECT_FALSE(next_scene->GetHddagiRuntime()->resources);
  EXPECT_FALSE(next_scene->GetHddagiRuntime()->published);
  EXPECT_EQ(unsupported->gi_probe_settings.probe_count_x, 7u);
}

TEST(HddagiResources, GraphClearsEveryImageAndArrayLayerBeforeReadback) {
  ScopedGpuPlatform platform(false);
  GiProbeSettings probes;
  probes.probe_count_x = probes.probe_count_y = 9;
  probes.cascade_count = 2;
  std::string failure;
  auto field = HddagiResources::TryCreate(probes, {}, failure);
  ASSERT_TRUE(field) << failure;
  VkBufferCreateInfo buffer_info{VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO};
  buffer_info.size = field->images.size() * 8;
  buffer_info.usage = VK_BUFFER_USAGE_TRANSFER_DST_BIT;
  VmaAllocationCreateInfo readback{};
  readback.usage = VMA_MEMORY_USAGE_AUTO;
  readback.flags = VMA_ALLOCATION_CREATE_HOST_ACCESS_RANDOM_BIT;
  readback.requiredFlags = VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT;
  Buffer output(buffer_info, readback);
  PlatformLifecycleTestAccess::PreUpdate();
  RenderGraph graph;
  RenderGraphResourceRegistry registry;
  field->Import(graph, registry);
  graph.AddPass(field->ClearDescriptor(), [&](const RenderGraphExecutionContext& context) {
    Platform::RecordCommandsMainQueue([&](const VkCommandBuffer command) {
      field->Clear(command, context);
    });
  });
  const auto plan = graph.Compile({});
  ASSERT_TRUE(plan.valid);
  EXPECT_FALSE(plan.uses_compute_queue);
  EXPECT_FALSE(plan.uses_ray_tracing_queue);
  EXPECT_TRUE(plan.allocations.empty());
  graph.Execute(plan, registry);
  Platform::RecordCommandsMainQueue([&](const VkCommandBuffer command) {
    field->OrderAccess(command, VK_PIPELINE_STAGE_2_TRANSFER_BIT, VK_ACCESS_2_TRANSFER_WRITE_BIT);
    VkClearColorValue sentinel{};
    for (auto& channel : sentinel.uint32)
      channel = 0x3f800001;
    for (const auto& [name, texture] : field->images) {
      const VkImageSubresourceRange range{VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, texture.requirement.layers};
      Platform::ClearColorImage(command, *texture.image, sentinel, 1, &range);
    }
  });
  graph.Execute(plan, registry);
  PlatformLifecycleTestAccess::LateUpdate();
  Platform::WaitForFrameSubmissions("HDDAGI initialized image readback");
  size_t copy_index = 0;
  for (const auto& [name, texture] : field->images) {
    const auto& r = texture.requirement;
    VkBufferImageCopy copy{};
    copy.bufferOffset = copy_index++ * 8;
    copy.imageSubresource = {VK_IMAGE_ASPECT_COLOR_BIT, 0, r.layers - 1, 1};
    copy.imageOffset = {int32_t(r.extent.width - 1), int32_t(r.extent.height - 1), int32_t(r.extent.depth - 1)};
    copy.imageExtent = {1, 1, 1};
    output.CopyFromImage(*texture.image, copy);
  }
  EXPECT_TRUE(field->initialization_recorded);
  std::vector<uint8_t> values;
  output.DownloadVector(values, buffer_info.size);
  size_t index = 0;
  for (const auto& [name, texture] : field->images) {
    const auto format = texture.requirement.storage_format;
    const uint32_t bytes = format == VK_FORMAT_R8_UINT || format == VK_FORMAT_R8_UNORM                  ? 1
                           : format == VK_FORMAT_R16_UINT                                               ? 2
                           : format == VK_FORMAT_R32G32_UINT || format == VK_FORMAT_R16G16B16A16_SFLOAT ? 8
                                                                                                        : 4;
    for (uint32_t byte = 0; byte < bytes; ++byte)
      EXPECT_EQ(values[index * 8 + byte], 0) << name << " byte " << byte;
    ++index;
  }
}

TEST(HddagiRuntime, SubmittedFieldSurvivesProviderSwitchUntilItsFrameSlotRetires) {
  TempProject project;
  Application app;
  struct Terminate {
    Application& app;
    ~Terminate() {
      app.Terminate();
    }
  } terminate{app};
  app.PushLayer<RenderLayer>("Render Layer");
  auto initialization = TestApplicationSettings(project);
  initialization.load_default_resources = true;
  app.Initialize(initialization);
  const auto render = ApplicationContext::Get().GetLayer<RenderLayer>();
  const auto scene = AssetManager::CreateTemporaryAsset<Scene>();
  const auto lighting = std::make_shared<EnvironmentalLighting>();
  lighting->indirect_gi_provider = IndirectGiProvider::AutomaticHddagi;
  lighting->gi_probe_settings.probe_count_x = lighting->gi_probe_settings.probe_count_y = 9;
  lighting->gi_probe_settings.cascade_count = 1;
  scene->environmental_lighting = lighting;
  scene->main_camera = scene->GetOrSetPrivateComponent<Camera>(scene->CreateEntity("GI anchor")).lock();
  PlatformLifecycleTestAccess::PreUpdate();
  SdfgiTestAccess::ExecuteSceneFrame(*render, scene);
  ASSERT_TRUE(scene->GetHddagiRuntime());
  ASSERT_TRUE(scene->GetHddagiRuntime()->resources) << scene->GetHddagiRuntime()->fallback_reason;
  EXPECT_TRUE(scene->GetHddagiRuntime()->resources->initialization_recorded);
  EXPECT_FALSE(scene->GetHddagiRuntime()->published);
  const std::weak_ptr<HddagiResources> field = scene->GetHddagiRuntime()->resources;
  PlatformLifecycleTestAccess::LateUpdate();
  EXPECT_GT(Platform::GetPendingFrameSubmissionCount(), 0u);
  lighting->indirect_gi_provider = IndirectGiProvider::Environment;
  SdfgiTestAccess::ExecuteSceneFrame(*render, scene);
  EXPECT_FALSE(scene->GetHddagiRuntime());
  EXPECT_FALSE(field.expired());
  for (uint32_t frame = 0; frame < Platform::GetMaxFramesInFlight(); ++frame) {
    PlatformLifecycleTestAccess::PreUpdate();
    SdfgiTestAccess::RetireHddagiFrame(*render);
    PlatformLifecycleTestAccess::LateUpdate();
  }
  EXPECT_TRUE(field.expired());
}

TEST(HddagiTraversal, HierarchyMatchesVoxelDdaAcrossSupportedDimensionsAndCircularOffsets) {
  ScopedGpuPlatform platform(false);
  const auto root =
      std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / "EvoEngine_SDK/Internals/DefaultResources/Shaders";
  Shader::RegisterShaderIncludePath(root / "Modules");
  const auto layout = [](std::initializer_list<VkDescriptorType> types) {
    auto result = std::make_shared<DescriptorSetLayout>();
    uint32_t binding = 0;
    for (const auto type : types)
      result->PushDescriptorBinding(binding++, type, VK_SHADER_STAGE_COMPUTE_BIT, 0);
    result->Initialize();
    return result;
  };
  const auto fill_layout = layout({VK_DESCRIPTOR_TYPE_STORAGE_IMAGE});
  const auto region_layout = layout({VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE, VK_DESCRIPTOR_TYPE_STORAGE_IMAGE,
                                     VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, VK_DESCRIPTOR_TYPE_STORAGE_IMAGE});
  const auto trace_layout =
      layout({VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE, VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER,
              VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER});
  auto fill_shader = std::make_shared<Shader>();
  ASSERT_TRUE(fill_shader->TryCompile(ShaderType::Compute, std::string(R"(
struct Params { int3 grid; uint pattern; };
[[vk::push_constant]] ConstantBuffer<Params> params;
[[vk::binding(0, 0)]] [vk::image_format("r32ui")] RWTexture3D<uint> normals;
[numthreads(8, 8, 8)]
void main(uint3 id : SV_DispatchThreadID) {
  bool solid = params.pattern == 1 || (params.pattern == 2 && (id.x == params.grid.x / 2 || id.y == params.grid.y / 3)) ||
               (params.pattern == 3 && all(id == uint3(params.grid - 1))) ||
               (params.pattern == 4 && id.x == uint(params.grid.x * 3 / 4));
  normals[id] = solid ? 1u : 0u;
})")));
  auto region_shader = std::make_shared<Shader>();
  auto trace_shader = std::make_shared<Shader>();
  ASSERT_TRUE(region_shader->TryCompile(ShaderType::Compute, root / "Compute/HddagiRegionStore.slang"));
  ASSERT_TRUE(trace_shader->TryCompile(ShaderType::Compute, root / "Compute/HddagiTraceRays.slang"));
  const auto pipeline = [](const std::shared_ptr<Shader>& shader, const std::shared_ptr<DescriptorSetLayout>& set,
                           const uint32_t push_size) {
    auto result = std::make_shared<ComputePipeline>();
    result->compute_shader = shader;
    result->descriptor_set_layouts = {set};
    result->push_constant_ranges.push_back({VK_SHADER_STAGE_COMPUTE_BIT, 0, push_size});
    result->Initialize();
    return result;
  };
  const auto fill = pipeline(fill_shader, fill_layout, 16);
  const auto region_store = pipeline(region_shader, region_layout, sizeof(HddagiRegionParams));
  const auto trace = pipeline(trace_shader, trace_layout, sizeof(HddagiTraceParams));
  ASSERT_TRUE(fill->Initialized() && region_store->Initialized() && trace->Initialized());
  const auto buffer = [](const size_t bytes, const VkBufferUsageFlags usage) {
    VkBufferCreateInfo info{VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO};
    info.size = bytes;
    info.usage = usage | VK_BUFFER_USAGE_TRANSFER_SRC_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT;
    return std::make_shared<Buffer>(info);
  };
  for (const auto grid : {glm::ivec3(64), glm::ivec3(128), glm::ivec3(192, 80, 192), glm::ivec3(256, 128, 256)}) {
    SCOPED_TRACE("grid " + std::to_string(grid.x) + "x" + std::to_string(grid.y));
    GiProbeSettings probes;
    probes.probe_count_x = grid.x / 8 + 1;
    probes.probe_count_y = grid.y / 8 + 1;
    probes.cascade_count = 2;
    std::string failure;
    auto field = HddagiResources::TryCreate(probes, {}, failure);
    ASSERT_TRUE(field) << failure;
    PlatformLifecycleTestAccess::PreUpdate();
    RenderGraph graph;
    RenderGraphResourceRegistry registry;
    field->Import(graph, registry);
    graph.AddPass(field->ClearDescriptor(), [&](const RenderGraphExecutionContext& context) {
      Platform::RecordCommandsMainQueue([&](const VkCommandBuffer command) {
        field->Clear(command, context);
      });
    });
    graph.Execute(graph.Compile({}), registry);
    PlatformLifecycleTestAccess::LateUpdate();
    Platform::WaitForFrameSubmissions("HDDAGI traversal fixture initialization");
    const auto fill_set = std::make_shared<DescriptorSet>(fill_layout);
    const auto region_set = std::make_shared<DescriptorSet>(region_layout);
    const auto trace_set = std::make_shared<DescriptorSet>(trace_layout);
    const auto image_binding = [&](const std::shared_ptr<DescriptorSet>& set, const uint32_t binding, const char* name,
                                   const bool sampled) {
      const auto& image = field->images.at(name);
      VkDescriptorImageInfo info{};
      info.imageLayout = VK_IMAGE_LAYOUT_GENERAL;
      info.imageView = (sampled ? image.sampled : image.storage)->GetVkImageView();
      set->UpdateImageDescriptorBinding(binding, info);
    };
    image_binding(fill_set, 0, "NormalBits", false);
    image_binding(region_set, 0, "NormalBits", true);
    image_binding(region_set, 1, "VoxelBits", false);
    image_binding(region_set, 2, "Regions", false);
    image_binding(region_set, 3, "Versions", false);
    image_binding(trace_set, 0, "VoxelBits", true);
    image_binding(trace_set, 1, "Regions", true);
    HddagiCascadeBlock cascades;
    cascades.data[0].offset = {-37, 11, -23};
    cascades.data[0].region_world_offset = {-17, 23, -5};
    cascades.data[1].offset = cascades.data[0].offset - glm::vec3(grid) * 0.5f;
    cascades.data[1].to_cell = 0.5f;
    cascades.data[1].region_world_offset = {-9, 11, -3};
    std::vector<HddagiRay> rays;
    const auto add_ray = [&](const glm::vec3 local, const glm::vec3 direction) {
      rays.push_back({local + cascades.data[0].offset, 0, glm::normalize(direction), 0});
    };
    for (int x = -1; x <= 1; ++x)
      for (int y = -1; y <= 1; ++y)
        for (int z = -1; z <= 1; ++z)
          if (x || y || z) {
            add_ray(glm::vec3(grid) * 0.25f + 0.25f, glm::vec3(x, y, z));
            add_ray(glm::vec3(grid) - 0.25f, glm::vec3(x, y, z));
          }
    add_ray({0.5f, 0.5f, 0.5f}, {1, 1e-8f, -1e-8f});
    add_ray(glm::vec3(grid) - 0.5f, {-1, -1e-8f, 1e-8f});
    add_ray({-1e-5f, 0.5f, 0.5f}, {1, 0, 0});
    add_ray({1e30f, 0.5f, 0.5f}, {-1, 0, 0});
    rays.push_back({cascades.data[0].offset + 0.5f, 0, glm::vec3(0), 0});
    for (uint32_t bit = 0; bit < 512; ++bit)
      add_ray(glm::vec3(grid / 2 / 8 * 8) + glm::vec3(bit % 8, bit / 8 % 8, bit / 64) + 0.25f, {1, 0, 0});
    auto cascade_buffer = buffer(sizeof(cascades), VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT);
    auto ray_buffer = buffer(rays.size() * sizeof(HddagiRay), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT);
    auto result_buffer = buffer(rays.size() * 3 * sizeof(glm::uvec4), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT);
    cascade_buffer->Upload(cascades);
    ray_buffer->UploadVector(rays);
    trace_set->UpdateBufferDescriptorBinding(2, cascade_buffer);
    trace_set->UpdateBufferDescriptorBinding(3, ray_buffer);
    trace_set->UpdateBufferDescriptorBinding(4, result_buffer);
    for (uint32_t pattern = 0; pattern < 4; ++pattern) {
      SCOPED_TRACE("pattern " + std::to_string(pattern));
      const auto solid = [&](const glm::ivec3 cell) {
        return pattern == 1 || (pattern == 2 && (cell.x == grid.x / 2 || cell.y == grid.y / 3)) ||
               (pattern == 3 && cell == grid - 1);
      };
      const auto oracle = [&](const HddagiRay& ray) {
        const glm::dvec3 origin = glm::dvec3(ray.origin) - glm::dvec3(cascades.data[0].offset);
        const glm::dvec3 direction(ray.direction);
        if (glm::any(glm::lessThan(origin, glm::dvec3(0))) ||
            glm::any(glm::greaterThanEqual(origin, glm::dvec3(grid))) || direction == glm::dvec3(0))
          return std::make_pair(glm::ivec4(0), glm::dvec3(0));
        auto cell = glm::ivec3(glm::floor(origin));
        const auto step = glm::ivec3(glm::sign(direction));
        glm::dvec3 next, delta;
        double entry = 0;
        for (int axis = 0; axis < 3; ++axis) {
          next[axis] = direction[axis] == 0 ? INFINITY
                                            : (cell[axis] + (step[axis] > 0 ? 1 : 0) - origin[axis]) / direction[axis];
          delta[axis] = direction[axis] == 0 ? INFINITY : std::abs(1.0 / direction[axis]);
        }
        while (glm::all(glm::greaterThanEqual(cell, glm::ivec3(0))) && glm::all(glm::lessThan(cell, grid))) {
          if (solid(cell))
            return std::make_pair(glm::ivec4(cell, 1), origin + direction * entry);
          const auto t = std::min(next.x, std::min(next.y, next.z));
          entry = t;
          for (int axis = 0; axis < 3; ++axis)
            if (next[axis] <= t + 1e-9) {
              cell[axis] += step[axis];
              next[axis] += delta[axis];
            }
        }
        return std::make_pair(glm::ivec4(0), glm::dvec3(0));
      };
      for (const uint32_t fractional_bits : {8u, 10u}) {
        SCOPED_TRACE("fractional bits " + std::to_string(fractional_bits));
        Platform::ImmediateSubmit([&](const VkCommandBuffer command) {
          field->OrderAccess(command, VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT, VK_ACCESS_2_SHADER_WRITE_BIT);
          fill->Bind(command);
          fill->BindDescriptorSet(command, 0, fill_set->GetVkDescriptorSet());
          fill->PushConstant(command, 0, glm::uvec4(glm::uvec3(grid), pattern));
          fill->Dispatch(command, grid.x / 8, grid.y / 8, grid.z / 8);
          field->OrderAccess(command, VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT,
                             VK_ACCESS_2_SHADER_READ_BIT | VK_ACCESS_2_SHADER_WRITE_BIT);
          region_store->Bind(command);
          region_store->BindDescriptorSet(command, 0, region_set->GetVkDescriptorSet());
          HddagiRegionParams region_params;
          region_params.grid = grid;
          region_params.region_world_offset = cascades.data[0].region_world_offset;
          region_params.version = pattern + 1;
          region_store->PushConstant(command, 0, region_params);
          region_store->Dispatch(command, grid.x / 8, grid.y / 8, grid.z / 8);
          field->OrderAccess(command, VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT, VK_ACCESS_2_SHADER_READ_BIT);
          trace->Bind(command);
          trace->BindDescriptorSet(command, 0, trace_set->GetVkDescriptorSet());
          HddagiTraceParams trace_params;
          trace_params.grid = grid;
          trace_params.ray_count = rays.size();
          trace_params.fractional_bits = fractional_bits;
          trace->PushConstant(command, 0, trace_params);
          trace->Dispatch(command, Platform::DivUp(trace_params.ray_count, 64));
          Platform::BufferMemoryBarrier(command, *result_buffer, VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT,
                                        VK_ACCESS_2_SHADER_WRITE_BIT, VK_PIPELINE_STAGE_2_TRANSFER_BIT,
                                        VK_ACCESS_2_TRANSFER_READ_BIT);
        });
        std::vector<glm::uvec4> results;
        result_buffer->DownloadVector(results, rays.size() * 3);
        for (size_t i = 0; i < rays.size(); ++i) {
          const auto [expected, intersection] = oracle(rays[i]);
          ASSERT_EQ(results[i * 3 + 1].w, uint32_t(expected.w)) << "ray " << i;
          if (expected.w) {
            const auto actual = glm::ivec3(results[i * 3]);
            EXPECT_TRUE(solid(actual)) << "ray " << i;
            // At an exact edge, either adjacent solid voxel can contain the same first intersection.
            EXPECT_TRUE(glm::all(glm::greaterThanEqual(intersection, glm::dvec3(actual) - 1e-8)) &&
                        glm::all(glm::lessThanEqual(intersection, glm::dvec3(actual) + 1.0 + 1e-8)))
                << "ray " << i;
          }
          EXPECT_LT(results[i * 3 + 2].x, 10000u);
        }
      }
    }
    std::vector<HddagiRay> crossing{{cascades.data[0].offset + glm::vec3(grid) * 0.5f, 0, {1, 0, 0}, 0},
                                    {cascades.data[0].offset + glm::vec3(grid) * 1.125f, 0, {-1, 0, 0}, 0},
                                    {{1e30f, 0, 0}, 0, {-1, 0, 0}, 0}};
    ray_buffer->UploadVector(crossing);
    for (const uint32_t fractional_bits : {8u, 10u}) {
      Platform::ImmediateSubmit([&](const VkCommandBuffer command) {
        field->OrderAccess(command, VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT, VK_ACCESS_2_SHADER_WRITE_BIT);
        fill->Bind(command);
        fill->BindDescriptorSet(command, 0, fill_set->GetVkDescriptorSet());
        fill->PushConstant(command, 0, glm::uvec4(glm::uvec3(grid), 4));
        fill->Dispatch(command, grid.x / 8, grid.y / 8, grid.z / 8);
        field->OrderAccess(command, VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT,
                           VK_ACCESS_2_SHADER_READ_BIT | VK_ACCESS_2_SHADER_WRITE_BIT);
        region_store->Bind(command);
        region_store->BindDescriptorSet(command, 0, region_set->GetVkDescriptorSet());
        HddagiRegionParams region_params;
        region_params.grid = grid;
        region_params.cascade = 1;
        region_params.region_world_offset = cascades.data[1].region_world_offset;
        region_store->PushConstant(command, 0, region_params);
        region_store->Dispatch(command, grid.x / 8, grid.y / 8, grid.z / 8);
        field->OrderAccess(command, VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT, VK_ACCESS_2_SHADER_READ_BIT);
        trace->Bind(command);
        trace->BindDescriptorSet(command, 0, trace_set->GetVkDescriptorSet());
        HddagiTraceParams trace_params;
        trace_params.grid = grid;
        trace_params.ray_count = crossing.size();
        trace_params.cascade_count = 2;
        trace_params.fractional_bits = fractional_bits;
        trace->PushConstant(command, 0, trace_params);
        trace->Dispatch(command, 1);
        Platform::BufferMemoryBarrier(command, *result_buffer, VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT,
                                      VK_ACCESS_2_SHADER_WRITE_BIT, VK_PIPELINE_STAGE_2_TRANSFER_BIT,
                                      VK_ACCESS_2_TRANSFER_READ_BIT);
      });
      std::vector<glm::uvec4> results;
      result_buffer->DownloadVector(results, crossing.size() * 3);
      EXPECT_EQ(results[0], glm::uvec4(grid.x * 3 / 4, grid.y / 2, grid.z / 2, 1));
      EXPECT_EQ(results[1], glm::uvec4(uint32_t(-1), 0, 0, 1));
      EXPECT_EQ(results[3], glm::uvec4(grid.x * 3 / 4, grid.y * 13 / 16, grid.z * 13 / 16, 1));
      EXPECT_EQ(results[4], glm::uvec4(1, 0, 0, 1));
      EXPECT_EQ(results[7].w, 0u);
    }
    std::vector<HddagiRay> segments;
    for (uint32_t first_cascade : {0u, 1u})
      for (const auto direction :
           {glm::vec3(1, 0, 0), glm::normalize(glm::vec3(1, 0, 1)), glm::normalize(glm::vec3(1, 0, 0.00001f))}) {
        const float hit_distance = grid.x * 0.5f / direction.x;
        for (const float delta : {-0.01f, 0.01f})
          segments.push_back({crossing[0].origin, first_cascade, direction, hit_distance + delta});
        const float reverse_distance = (grid.x * 0.125f - 2) / direction.x;
        for (const float delta : {-0.01f, 0.01f})
          segments.push_back({crossing[1].origin, first_cascade, -direction, reverse_distance + delta});
      }
    segments.push_back({crossing[0].origin, 0, {1, 0, 0}, 0});
    ray_buffer->UploadVector(segments);
    Platform::ImmediateSubmit([&](const VkCommandBuffer command) {
      field->OrderAccess(command, VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT, VK_ACCESS_2_SHADER_READ_BIT);
      trace->Bind(command);
      trace->BindDescriptorSet(command, 0, trace_set->GetVkDescriptorSet());
      HddagiTraceParams params;
      params.grid = grid;
      params.ray_count = segments.size();
      params.cascade_count = 2;
      params.fractional_bits = 10;
      params.padding.x = 1;
      trace->PushConstant(command, 0, params);
      trace->Dispatch(command, 1);
      Platform::BufferMemoryBarrier(command, *result_buffer, VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT,
                                    VK_ACCESS_2_SHADER_WRITE_BIT, VK_PIPELINE_STAGE_2_TRANSFER_BIT,
                                    VK_ACCESS_2_TRANSFER_READ_BIT);
    });
    std::vector<glm::uvec4> segment_results;
    result_buffer->DownloadVector(segment_results, segments.size() * 3);
    for (size_t i = 0; i < segments.size(); ++i)
      EXPECT_EQ(segment_results[i * 3 + 1].w, i + 1 < segments.size() ? i % 2 : 0u) << "segment " << i;
  }
}

TEST(HddagiVoxelization, ThreeAxesPayloadCoverageAndRepeatedScratchWithRtDisabled) {
  ScopedGpuPlatform platform(false);
  ApplicationContext::Get().RegisterAsset<Shader>("Shader", {".eveshader", ".slang"});
  const auto render = ApplicationContext::Get().GetLayer<RenderLayer>();
  SdfgiSettings settings;
  settings.voxel_count_x = settings.voxel_count_y = 128;
  settings.probe_spacing_cells = 8;
  settings.cascade_count = 1;
  settings.min_cell_size = 1;
  settings.vertical_scale = SdfgiSettings::VerticalScale::Percent100;
  std::string failure;
  auto field = HddagiResources::TryCreate(GiProbesFromSdfgi(settings), HddagiSettings{}, failure);
  ASSERT_TRUE(field) << failure;
  EXPECT_FALSE(Platform::RayTracingEnabled());
  EXPECT_FALSE(Platform::RayQueryEnabled());
  EXPECT_FALSE(Platform::RayAccelerationStructureEnabled());
  std::vector<SdfgiCascade> cascades;
  ASSERT_TRUE(UpdateSdfgiCascades(settings, glm::vec3(0), cascades).empty());
  const auto pending = GetSdfgiPendingRegions(cascades, 1);
  const auto mask_texture = std::make_shared<Texture2D>();
  mask_texture->SetRgbaChannelData({glm::vec4(1, 1, 1, 0), glm::vec4(1)}, {2, 1});
  Platform::GetGpuService().WaitIdle();
  TextureStorage::DeviceSync();
  SdfgiTextureInput texture;
  texture.texture = mask_texture;
  texture.image = mask_texture->GetImage();
  texture.image_view = mask_texture->PeekTexture2DStorage().image_view;
  texture.sampler = mask_texture->PeekTexture2DStorage().sampler;
  texture.mapping.tex_coord = 1;
  texture.mapping.uv_transform[2].x = 0.5f;
  SdfgiContributorRegistry contributors;
  for (uint32_t axis = 0; axis < 3; ++axis)
    for (uint32_t kind = 0; kind < 4; ++kind) {
      SdfgiContributor contributor;
      contributor.id = {axis * 4 + kind + 1, 1};
      contributor.mesh = std::make_shared<Mesh>();
      contributor.mesh->OnCreate();
      const auto right = (axis + 1) % 3, up = (axis + 2) % 3;
      std::vector<Vertex> vertices(4);
      for (uint32_t corner = 0; corner < 4; ++corner) {
        auto& vertex = vertices[corner];
        vertex.position[axis] = 0.5f;
        vertex.position[right] = -52.0f + 12 * kind + (corner & 1 ? 8 : 0);
        vertex.position[up] = corner & 2 ? 4 : -4;
        vertex.normal[axis] = kind == 2 ? -1 : 1;
        vertex.color = glm::vec4(0.5f, 0.5f, 0.5f, 1);
        vertex.tex_coord_1 = glm::vec2(corner & 1 ? 1 : 0, 0.5f);
      }
      VertexAttributes attributes{};
      attributes.normal = attributes.color = true;
      attributes.tex_coord_1 = true;
      const std::vector<glm::uvec3> triangles =
          kind == 2 ? std::vector<glm::uvec3>{{0, 2, 1}, {1, 2, 3}} : std::vector<glm::uvec3>{{0, 1, 2}, {1, 3, 2}};
      contributor.mesh->SetVertices(attributes, vertices, triangles);
      contributor.world_bounds = contributor.mesh->GetBound();
      contributor.material.base_color = glm::vec4(0.8f, 0.4f, 0.2f, 0.1f);
      contributor.material.masked = kind == 1;
      if (kind == 1) {
        contributor.material.base_color.a = 1;
        contributor.material.base_texture = texture;
      }
      contributor.material.double_sided = kind == 2;
      contributor.material.cull_mode = VK_CULL_MODE_BACK_BIT;
      contributor.material.emission = kind == 3 ? glm::vec3(2, 0, 0) : glm::vec3(0);
      contributors.entries.emplace(contributor.id, std::move(contributor));
    }
  GeometryStorage::WaitForPendingUploads();
  for (uint32_t iteration = 0; iteration < 3; ++iteration) {
    PlatformLifecycleTestAccess::PreUpdate();
    if (iteration == 1)
      for (auto& [id, contributor] : contributors.entries) {
        contributor.material.masked = false;
        contributor.material.emission = glm::vec3(0);
      }
    if (iteration == 2) {
      field->light_cell_capacity = 1;
      Platform::ImmediateSubmit([&](const VkCommandBuffer command) {
        field->OrderAccess(command, VK_PIPELINE_STAGE_2_TRANSFER_BIT, VK_ACCESS_2_TRANSFER_WRITE_BIT);
        field->buffers.at("ProcessSpare0")->Fill(command, 0, 32, 0xdeadbeefu);
      });
    }
    auto frame = HddagiVoxelFrame::Create(*field, SdfgiTestAccess::HostLayouts(*render)[0], contributors, cascades,
                                          pending, iteration + 1);
    RenderGraph graph;
    RenderGraphResourceRegistry registry;
    field->Import(graph, registry);
    if (!field->initialization_recorded)
      graph.AddPass(field->ClearDescriptor(), [field](const RenderGraphExecutionContext& context) {
        Platform::RecordCommandsMainQueue([&](const VkCommandBuffer command) {
          field->Clear(command, context);
        });
      });
    frame->AddPasses(graph, registry, field);
    const auto plan = graph.Compile({});
    ASSERT_TRUE(plan.valid);
    EXPECT_FALSE(plan.uses_compute_queue);
    EXPECT_FALSE(plan.uses_ray_tracing_queue);
    graph.Execute(plan, registry);
    PlatformLifecycleTestAccess::LateUpdate();
    Platform::WaitForFrameSubmissions("HDDAGI raster payload readback");
    std::map<std::string, std::vector<uint32_t>> data;
    for (const auto name : {"Albedo", "Emission", "EmissionAniso", "NormalBits", "VoxelBits", "Regions", "Versions"}) {
      const auto& texture = field->images.at(name);
      const auto extent = texture.requirement.extent;
      const size_t count = size_t(extent.width) * extent.height * extent.depth;
      const size_t bytes = std::string(name) == "Albedo" || std::string(name) == "Versions" ? 2
                           : std::string(name) == "Regions"                                 ? 1
                           : std::string(name) == "VoxelBits"                               ? 8
                                                                                            : 4;
      VkBufferCreateInfo info{VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO};
      info.size = count * bytes;
      info.usage = VK_BUFFER_USAGE_TRANSFER_DST_BIT;
      VmaAllocationCreateInfo allocation{};
      allocation.usage = VMA_MEMORY_USAGE_AUTO;
      allocation.flags = VMA_ALLOCATION_CREATE_HOST_ACCESS_RANDOM_BIT;
      allocation.requiredFlags = VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT;
      Buffer output(info, allocation);
      VkBufferImageCopy copy{};
      copy.imageSubresource = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 0, 1};
      copy.imageExtent = extent;
      output.CopyFromImage(*texture.image, copy);
      if (bytes == 1) {
        std::vector<uint8_t> values;
        output.DownloadVector(values, count);
        data[name].assign(values.begin(), values.end());
      } else if (bytes == 2) {
        std::vector<uint16_t> values;
        output.DownloadVector(values, count);
        data[name].assign(values.begin(), values.end());
      } else
        output.DownloadVector(data[name], count * bytes / 4);
    }
    frame->ReadStatusAfterFence(*field);
    std::vector<uint32_t> dispatch;
    field->buffers.at("Dispatch0")->DownloadVector(dispatch, 5);
    EXPECT_EQ(field->failure_flags, dispatch[4]);
    ASSERT_EQ(field->light_cell_counts.size(), 1u);
    EXPECT_EQ(field->light_cell_counts[0], dispatch[3]);
    ASSERT_EQ(dispatch[4], iteration == 2 ? 1u : 0u);
    EXPECT_EQ(dispatch[0], (std::min(dispatch[3], field->light_cell_capacity) + 63) / 64);
    EXPECT_EQ(dispatch[1], 1u);
    EXPECT_EQ(dispatch[2], 1u);
    std::vector<HddagiProcessVoxel> payload;
    field->buffers.at("Process0")->DownloadVector(payload, std::min(dispatch[3], field->light_cell_capacity));
    std::map<uint32_t, HddagiProcessVoxel> payload_by_cell;
    for (const auto& voxel : payload) {
      EXPECT_EQ(voxel.position >> 30, 3u);
      EXPECT_TRUE(payload_by_cell.emplace(voxel.position & 0xffffffu, voxel).second);
      EXPECT_EQ(voxel.albedo_normal & 0xffffu, 12u | (12u << 5) | (3u << 11));
      uint32_t total_weight = 0;
      for (uint32_t neighbor = 0; neighbor < 8; ++neighbor)
        total_weight += (voxel.occlusion >> (neighbor * 4)) & 15;
      EXPECT_LE(total_weight, 15u);
      if (iteration == 1)
        EXPECT_EQ(voxel.emission, 0u);
    }
    const auto value = [&](const std::string& name, const glm::ivec3 cell) {
      const auto e = field->images.at(name).requirement.extent;
      return data.at(name).at(cell.x + e.width * (cell.y + e.height * cell.z));
    };
    size_t expected_payload_count = 0;
    for (int z = 0; z < 128; ++z)
      for (int y = 0; y < 128; ++y)
        for (int x = 0; x < 128; ++x) {
          const glm::ivec3 cell(x, y, z);
          if (value("NormalBits", cell))
            continue;
          bool expected = false;
          for (int face = 0; face < 6; ++face) {
            auto neighbor = cell;
            neighbor[face / 2] += face % 2 ? -1 : 1;
            if (glm::any(glm::lessThan(neighbor, glm::ivec3(0))) ||
                glm::any(glm::greaterThanEqual(neighbor, glm::ivec3(128))))
              continue;
            expected |= (value("NormalBits", neighbor) & (1u << face)) != 0;
          }
          if (!expected)
            continue;
          ++expected_payload_count;
          if (iteration < 2)
            EXPECT_EQ(payload_by_cell.count(uint32_t(x | (y << 8) | (z << 16))), 1u);
        }
    EXPECT_EQ(dispatch[3], expected_payload_count);
    EXPECT_EQ(payload.size(), std::min<size_t>(expected_payload_count, field->light_cell_capacity));
    if (iteration == 2) {
      std::vector<HddagiProcessVoxel> prefix;
      field->buffers.at("Process0")->DownloadVector(prefix, 2);
      EXPECT_EQ(prefix[1].position, 0xdeadbeefu);
      EXPECT_EQ(prefix[1].albedo_normal, 0xdeadbeefu);
      EXPECT_EQ(prefix[1].emission, 0xdeadbeefu);
      EXPECT_EQ(prefix[1].occlusion, 0xdeadbeefu);
    }

    for (uint32_t axis = 0; axis < 3; ++axis)
      for (uint32_t kind = 0; kind < 4; ++kind) {
        SCOPED_TRACE(::testing::Message() << "frame=" << iteration << " axis=" << axis << " kind=" << kind);
        glm::ivec3 cell(64);
        cell[(axis + 1) % 3] = 16 + 12 * kind;
        cell[(axis + 2) % 3] = 65;
        const bool empty = iteration == 0 && kind == 1;
        const uint32_t face = axis * 2 + (kind == 2 ? 0 : 1);
        const uint32_t exclusive[6] = {(1u << 6) | (1u << 13) | (1u << 15) | (1u << 16) | (1u << 18),
                                       (1u << 7) | (1u << 25) | (1u << 27) | (1u << 28) | (1u << 30),
                                       (1u << 8) | (1u << 13) | (1u << 20) | (1u << 21) | (1u << 25),
                                       (1u << 9) | (1u << 18) | (1u << 22) | (1u << 23) | (1u << 30),
                                       (1u << 10) | (1u << 15) | (1u << 20) | (1u << 22) | (1u << 27),
                                       (1u << 11) | (1u << 16) | (1u << 21) | (1u << 23) | (1u << 28)};
        EXPECT_EQ(value("NormalBits", cell), empty ? 0u : (1u << face) | exclusive[face]);
        if (kind == 1) {
          auto visible = cell;
          visible[(axis + 1) % 3] = 25;
          EXPECT_EQ(value("NormalBits", visible), (1u << face) | exclusive[face]);
        }
        auto albedo_cell = cell / 2;
        albedo_cell.z = albedo_cell.z * 6 + face;
        EXPECT_EQ(value("Albedo", albedo_cell), empty ? 0u : 12u | (12u << 5) | (3u << 11));
        EXPECT_EQ(value("Emission", cell / 2), iteration == 0 && kind == 3 ? (17u << 27) | 256u : 0u);
        EXPECT_EQ(value("EmissionAniso", cell / 2), iteration == 0 && kind == 3 ? 31u << (face * 5) : 0u);
        const auto circular = (cell + glm::ivec3(64)) % 128;
        EXPECT_EQ(value("Versions", circular / 8), iteration + 1);
        const auto block = circular / 4;
        const auto local = circular % 4;
        const uint32_t bit = local.x + 4 * local.y + 16 * local.z;
        const size_t index = 2 * (block.x + 32 * (block.y + 32 * block.z)) + bit / 32;
        EXPECT_EQ((data.at("VoxelBits")[index] >> (bit % 32)) & 1u, empty ? 0u : 1u);
      }
  }
}

TEST(HddagiRuntime, SceneContributorParityMovementRemovalAndUnchangedFrameWithoutRt) {
  TempProject project;
  Application app;
  struct Terminate {
    Application& app;
    ~Terminate() {
      app.Terminate();
    }
  } terminate{app};
  app.PushLayer<RenderLayer>("Render Layer");
  auto initialization = TestApplicationSettings(project);
  initialization.load_default_resources = true;
  app.Initialize(initialization);
  EXPECT_FALSE(Platform::RayTracingEnabled());
  EXPECT_FALSE(Platform::RayQueryEnabled());
  EXPECT_FALSE(Platform::RayAccelerationStructureEnabled());
  const auto scene = AssetManager::CreateTemporaryAsset<Scene>();
  const auto entity = scene->CreateEntity("Non-static contributor");
  const auto renderer = scene->GetOrSetPrivateComponent<MeshRenderer>(entity).lock();
  const auto mesh = AssetManager::CreateTemporaryAsset<Mesh>();
  renderer->mesh = mesh;
  renderer->material = AssetManager::CreateTemporaryAsset<Material>();
  std::vector<Vertex> vertices(4);
  for (uint32_t i = 0; i < 4; ++i) {
    vertices[i].position = {0.5f, i & 1 ? 4.0f : -4.0f, i & 2 ? 4.0f : -4.0f};
    vertices[i].normal = {1, 0, 0};
    vertices[i].color = glm::vec4(1);
  }
  VertexAttributes attributes{};
  attributes.normal = attributes.color = true;
  mesh->SetVertices(attributes, vertices, std::vector<glm::uvec3>{{0, 1, 2}, {1, 3, 2}});
  GeometryStorage::WaitForPendingUploads();
  const auto lighting = std::make_shared<EnvironmentalLighting>();
  lighting->indirect_gi_provider = IndirectGiProvider::AutomaticHddagi;
  lighting->gi_probe_settings.probe_count_x = lighting->gi_probe_settings.probe_count_y = 9;
  lighting->gi_probe_settings.cascade_count = 1;
  lighting->gi_probe_settings.base_probe_distance = 8;
  scene->environmental_lighting = lighting;
  scene->main_camera = scene->GetOrSetPrivateComponent<Camera>(scene->CreateEntity("GI anchor")).lock();
  const auto render = ApplicationContext::Get().GetLayer<RenderLayer>();
  uint32_t previous_version = 0;
  for (uint32_t phase = 0; phase < 6; ++phase) {
    SCOPED_TRACE(phase);
    PlatformLifecycleTestAccess::PreUpdate();
    SdfgiTestAccess::RetireHddagiFrame(*render);
    lighting->hddagi_settings.static_entities_only = phase == 1;
    if (phase == 3) {
      GlobalTransform transform;
      transform.SetPosition({12, 0, 0});
      scene->SetDataComponent(entity, transform);
    }
    if (phase == 4)
      scene->DeleteEntity(entity);
    SdfgiTestAccess::ExecuteSceneFrame(*render, scene);
    const auto runtime = scene->GetHddagiRuntime();
    ASSERT_TRUE(runtime);
    ASSERT_TRUE(runtime->resources) << runtime->fallback_reason;
    EXPECT_TRUE(runtime->voxel_failure.empty()) << runtime->voxel_failure;
    EXPECT_TRUE(runtime->resources->voxelization_recorded);
    if (phase != 5)
      EXPECT_GT(runtime->resources->AllocationBytes(), runtime->resources->allocation_bytes);
    EXPECT_FALSE(runtime->published);
    EXPECT_FALSE(scene->GetSdfgiRuntime());
    EXPECT_FALSE(SdfgiTestAccess::HasDdgiResources(*render));
    ResolvedEnvironmentalLighting reference;
    reference.sdfgi_settings.static_entities_only = lighting->hddagi_settings.static_entities_only;
    const auto snapshot = SnapshotSdfgiScene(scene, reference);
    SdfgiContributorRegistry registry;
    registry.Update(snapshot.contributors);
    EXPECT_EQ(runtime->contributors.entries.size(), registry.entries.size());
    EXPECT_EQ(runtime->contributors.entries.size(), phase == 1 || phase >= 4 ? 0u : 1u);
    if (phase == 5)
      EXPECT_EQ(runtime->region_version, previous_version);
    previous_version = runtime->region_version;
    const auto probe_frame = runtime->resources->probe_frames[Platform::GetCurrentFrameIndex()];
    PlatformLifecycleTestAccess::LateUpdate();
    Platform::WaitForFrameSubmissions("HDDAGI scene occupancy readback");
    ASSERT_TRUE(runtime->transport_failure.empty()) << runtime->transport_failure;
    ASSERT_TRUE(runtime->resources->transport_recorded);
    ASSERT_TRUE(probe_frame);
    probe_frame->ReadStatusAfterFence(*runtime->resources);
    EXPECT_TRUE(runtime->resources->transport_ready);
    EXPECT_EQ(runtime->resources->transport_failure_flags, 0u);
    const auto& texture = runtime->resources->images.at("Regions");
    VkBufferCreateInfo info{VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO};
    info.size = 512;
    info.usage = VK_BUFFER_USAGE_TRANSFER_DST_BIT;
    VmaAllocationCreateInfo allocation{};
    allocation.usage = VMA_MEMORY_USAGE_AUTO;
    allocation.flags = VMA_ALLOCATION_CREATE_HOST_ACCESS_RANDOM_BIT;
    allocation.requiredFlags = VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT;
    Buffer output(info, allocation);
    VkBufferImageCopy copy{};
    copy.imageSubresource = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 0, 1};
    copy.imageExtent = {8, 8, 8};
    output.CopyFromImage(*texture.image, copy);
    std::vector<uint8_t> occupancy;
    output.DownloadVector(occupancy, 512);
    EXPECT_EQ(std::any_of(occupancy.begin(), occupancy.end(),
                          [](uint8_t v) {
                            return v != 0;
                          }),
              phase != 1 && phase < 4);
  }
}

TEST(HddagiVoxelization, CompactPayloadPreservesMaximumCoordinatesAcrossRectangularFields) {
  ScopedGpuPlatform platform(false);
  ApplicationContext::Get().RegisterAsset<Shader>("Shader", {".eveshader", ".slang"});
  const auto render = ApplicationContext::Get().GetLayer<RenderLayer>();
  for (const auto grid : {glm::ivec3(64), glm::ivec3(192, 80, 192), glm::ivec3(256, 128, 256), glm::ivec3(256)}) {
    SCOPED_TRACE(::testing::Message() << grid.x << "x" << grid.y << "x" << grid.z);
    SdfgiSettings settings;
    settings.voxel_count_x = grid.x;
    settings.voxel_count_y = grid.y;
    settings.probe_spacing_cells = 8;
    settings.cascade_count = 1;
    settings.min_cell_size = 1;
    settings.vertical_scale = SdfgiSettings::VerticalScale::Percent100;
    std::string failure;
    auto field = HddagiResources::TryCreate(GiProbesFromSdfgi(settings), HddagiSettings{}, failure);
    ASSERT_TRUE(field) << failure;
    std::vector<SdfgiCascade> cascades;
    ASSERT_TRUE(UpdateSdfgiCascades(settings, glm::vec3(0), cascades).empty());
    SdfgiContributorRegistry contributors;
    for (uint32_t axis = 0; axis < 3; ++axis) {
      SdfgiContributor contributor;
      contributor.id = {axis + 1, 1};
      contributor.mesh = std::make_shared<Mesh>();
      contributor.mesh->OnCreate();
      std::vector<Vertex> vertices(4);
      for (uint32_t corner = 0; corner < 4; ++corner) {
        auto& vertex = vertices[corner];
        vertex.position[axis] = grid[axis] * 0.5f - 1.5f;
        vertex.position[(axis + 1) % 3] = corner & 1 ? 2.25f : 0.25f;
        vertex.position[(axis + 2) % 3] = corner & 2 ? 2.25f : 0.25f;
        vertex.normal[axis] = 1;
        vertex.color = glm::vec4(1);
      }
      VertexAttributes attributes{};
      attributes.normal = attributes.color = true;
      contributor.mesh->SetVertices(attributes, vertices, std::vector<glm::uvec3>{{0, 1, 2}, {1, 3, 2}});
      contributor.world_bounds = contributor.mesh->GetBound();
      contributors.entries.emplace(contributor.id, std::move(contributor));
    }
    GeometryStorage::WaitForPendingUploads();
    PlatformLifecycleTestAccess::PreUpdate();
    const auto frame = HddagiVoxelFrame::Create(*field, SdfgiTestAccess::HostLayouts(*render)[0], contributors,
                                                cascades, GetSdfgiPendingRegions(cascades, 1), 1);
    RenderGraph graph;
    RenderGraphResourceRegistry registry;
    field->Import(graph, registry);
    graph.AddPass(field->ClearDescriptor(), [field](const RenderGraphExecutionContext& context) {
      Platform::RecordCommandsMainQueue([&](const VkCommandBuffer command) {
        field->Clear(command, context);
      });
    });
    frame->AddPasses(graph, registry, field);
    const auto plan = graph.Compile({});
    ASSERT_TRUE(plan.valid);
    graph.Execute(plan, registry);
    PlatformLifecycleTestAccess::LateUpdate();
    Platform::WaitForFrameSubmissions("HDDAGI maximum-coordinate payload readback");
    frame->ReadStatusAfterFence(*field);
    ASSERT_EQ(field->failure_flags, 0u);
    ASSERT_EQ(field->light_cell_counts.size(), 1u);
    ASSERT_EQ(field->light_cell_counts[0], 12u);
    std::vector<HddagiProcessVoxel> payload;
    field->buffers.at("Process0")->DownloadVector(payload, 12);
    glm::uvec3 counts(0);
    for (const auto& voxel : payload) {
      EXPECT_EQ(voxel.position >> 30, 3u);
      const glm::ivec3 cell(voxel.position & 255u, (voxel.position >> 8) & 255u, (voxel.position >> 16) & 255u);
      EXPECT_TRUE(glm::all(glm::lessThan(cell, grid)));
      EXPECT_EQ(voxel.albedo_normal & 65535u, 65535u);
      EXPECT_EQ(voxel.emission, 0u);
      uint32_t total_weight = 0;
      for (uint32_t neighbor = 0; neighbor < 8; ++neighbor)
        total_weight += (voxel.occlusion >> (neighbor * 4)) & 15;
      EXPECT_LE(total_weight, 15u);
      for (int axis = 0; axis < 3; ++axis)
        if (cell[axis] == grid[axis] - 1)
          ++counts[axis];
    }
    EXPECT_EQ(counts, glm::uvec3(4));
  }
}

TEST(HddagiTraversal, SignedWorldCoordinatesWrapBijectivelyInRectangularFieldsAndProbeAtlases) {
  ScopedGpuPlatform platform(false);
  Shader::RegisterShaderIncludePath(std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) /
                                    "EvoEngine_SDK/Internals/DefaultResources/Shaders/Modules");
  auto shader = std::make_shared<Shader>();
  ASSERT_TRUE(shader->TryCompile(ShaderType::Compute, std::string(R"(
import EvoEngine.HddagiTrace;
struct Params { uint count; };
[[vk::push_constant]] ConstantBuffer<Params> params;
[[vk::binding(0, 0)]] StructuredBuffer<int4> inputs;
[[vk::binding(1, 0)]] RWStructuredBuffer<int4> outputs;
[numthreads(64, 1, 1)]
void main(uint3 id : SV_DispatchThreadID) {
  if (id.x >= params.count) return;
  int4 input = inputs[id.x];
  outputs[id.x] = int4(HddagiWrap(int3(input.x), input.yzw), 0);
})")));
  auto layout = std::make_shared<DescriptorSetLayout>();
  layout->PushDescriptorBinding(0, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_COMPUTE_BIT, 0);
  layout->PushDescriptorBinding(1, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_COMPUTE_BIT, 0);
  layout->Initialize();
  ComputePipeline pipeline;
  pipeline.compute_shader = shader;
  pipeline.descriptor_set_layouts = {layout};
  pipeline.push_constant_ranges.push_back({VK_SHADER_STAGE_COMPUTE_BIT, 0, 4});
  pipeline.Initialize();
  ASSERT_TRUE(pipeline.Initialized());
  std::vector<glm::ivec4> inputs;
  for (const auto size : {glm::ivec3(9, 11, 33), glm::ivec3(8, 10, 24), glm::ivec3(64, 80, 192), glm::ivec3(256)}) {
    for (int value = -1024; value <= 1024; ++value)
      inputs.emplace_back(value, size.x, size.y, size.z);
    inputs.emplace_back(INT32_MIN, size.x, size.y, size.z);
    inputs.emplace_back(INT32_MAX, size.x, size.y, size.z);
  }
  VkBufferCreateInfo info{VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO};
  info.size = inputs.size() * sizeof(glm::ivec4);
  info.usage = VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_SRC_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT;
  auto source = std::make_shared<Buffer>(info);
  auto destination = std::make_shared<Buffer>(info);
  source->UploadVector(inputs);
  auto set = std::make_shared<DescriptorSet>(layout);
  set->UpdateBufferDescriptorBinding(0, source);
  set->UpdateBufferDescriptorBinding(1, destination);
  Platform::ImmediateSubmit([&](const VkCommandBuffer command) {
    pipeline.Bind(command);
    pipeline.BindDescriptorSet(command, 0, set->GetVkDescriptorSet());
    pipeline.PushConstant(command, 0, static_cast<uint32_t>(inputs.size()));
    pipeline.Dispatch(command, (inputs.size() + 63) / 64, 1, 1);
    Platform::BufferMemoryBarrier(command, *destination);
  });
  std::vector<glm::ivec4> actual;
  destination->DownloadVector(actual, inputs.size());
  for (size_t i = 0; i < inputs.size(); ++i)
    for (int axis = 0; axis < 3; ++axis) {
      const int64_t size = inputs[i][axis + 1];
      EXPECT_EQ(actual[i][axis], (int64_t(inputs[i].x) % size + size) % size)
          << "value " << inputs[i].x << " dimension " << size;
    }
}

TEST(HddagiUpdates, ScrollingAndEditsMatchFreshHierarchyAndCompactPayloadWithoutRt) {
  ScopedGpuPlatform platform(false);
  ApplicationContext::Get().RegisterAsset<Shader>("Shader", {".eveshader", ".slang"});
  const auto render = ApplicationContext::Get().GetLayer<RenderLayer>();
  GiProbeSettings probes;
  probes.probe_count_x = 9;
  probes.probe_count_y = 11;
  probes.cascade_count = 2;
  probes.base_probe_distance = 8;
  std::string failure;
  auto incremental = HddagiResources::TryCreate(probes, {}, failure);
  ASSERT_TRUE(incremental) << failure;
  auto fresh = HddagiResources::TryCreate(probes, {}, failure);
  ASSERT_TRUE(fresh) << failure;
  const auto shader_root =
      std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / "EvoEngine_SDK/Internals/DefaultResources/Shaders";
  Shader::RegisterShaderIncludePath(shader_root / "Modules");
  auto cache_layout = std::make_shared<DescriptorSetLayout>();
  const std::array cache_types{VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE,  VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE,
                               VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,
                               VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE,
                               VK_DESCRIPTOR_TYPE_STORAGE_IMAGE,  VK_DESCRIPTOR_TYPE_STORAGE_IMAGE};
  for (uint32_t i = 0; i < cache_types.size(); ++i)
    cache_layout->PushDescriptorBinding(i, cache_types[i], VK_SHADER_STAGE_COMPUTE_BIT, 0);
  cache_layout->Initialize();
  auto cache_shader = std::make_shared<Shader>();
  ASSERT_TRUE(cache_shader->TryCompile(ShaderType::Compute, shader_root / "Compute/HddagiTraceCachedRays.slang"));
  auto cache_pipeline = std::make_shared<ComputePipeline>();
  cache_pipeline->compute_shader = cache_shader;
  cache_pipeline->descriptor_set_layouts = {cache_layout};
  cache_pipeline->push_constant_ranges.push_back({VK_SHADER_STAGE_COMPUTE_BIT, 0, sizeof(HddagiTraceParams)});
  cache_pipeline->Initialize();
  ASSERT_TRUE(cache_pipeline->Initialized());
  auto cache_set = std::make_shared<DescriptorSet>(cache_layout);
  for (const auto& [binding, name] : std::map<uint32_t, std::string>{
           {0, "VoxelBits"}, {1, "Regions"}, {5, "Versions"}, {6, "HitCache"}, {7, "HitVersions"}}) {
    VkDescriptorImageInfo info{};
    info.imageLayout = VK_IMAGE_LAYOUT_GENERAL;
    const auto& image = incremental->images.at(name);
    info.imageView = (binding < 6 ? image.sampled : image.storage)->GetVkImageView();
    cache_set->UpdateImageDescriptorBinding(binding, info);
  }
  const auto buffer = [](const size_t size, VkBufferUsageFlags usage) {
    VkBufferCreateInfo info{VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO};
    info.size = size;
    info.usage = usage | VK_BUFFER_USAGE_TRANSFER_SRC_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT;
    return std::make_shared<Buffer>(info);
  };
  const auto read_image = [](const HddagiResources& field, const std::string& name) {
    const auto& image = field.images.at(name);
    const auto extent = image.requirement.extent;
    VkBufferCreateInfo info{VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO};
    info.size = size_t(extent.width) * extent.height * extent.depth * image.requirement.layers * 4;
    info.usage = VK_BUFFER_USAGE_TRANSFER_DST_BIT;
    VmaAllocationCreateInfo allocation{};
    allocation.usage = VMA_MEMORY_USAGE_AUTO;
    allocation.flags = VMA_ALLOCATION_CREATE_HOST_ACCESS_RANDOM_BIT;
    allocation.requiredFlags = VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT;
    Buffer output(info, allocation);
    VkBufferImageCopy copy{};
    copy.imageSubresource = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 0, image.requirement.layers};
    copy.imageExtent = extent;
    output.CopyFromImage(*image.image, copy);
    std::vector<uint32_t> values;
    output.DownloadVector(values, info.size / 4);
    return values;
  };
  std::vector<SdfgiContributor> snapshot;
  for (uint32_t i = 0; i < 3; ++i) {
    SdfgiContributor contributor;
    contributor.id = {i + 1, 1};
    contributor.mesh = std::make_shared<Mesh>();
    contributor.mesh->OnCreate();
    const uint32_t axis = i == 2 ? 1 : 0;
    std::vector<Vertex> vertices(4);
    for (uint32_t corner = 0; corner < 4; ++corner) {
      auto& vertex = vertices[corner];
      vertex.position[axis] = i == 0 ? -24.5f : i == 1 ? 24.5f : -8.5f;
      vertex.position[(axis + 1) % 3] = corner & 1 ? 80 : -80;
      vertex.position[(axis + 2) % 3] = corner & 2 ? 80 : -80;
      vertex.normal[axis] = 1;
      vertex.color = glm::vec4(1);
    }
    VertexAttributes attributes{};
    attributes.normal = attributes.color = true;
    contributor.mesh->SetVertices(attributes, vertices, std::vector<glm::uvec3>{{0, 1, 2}, {1, 3, 2}});
    contributor.world_bounds = contributor.mesh->GetBound();
    snapshot.push_back(std::move(contributor));
  }
  GeometryStorage::WaitForPendingUploads();
  SdfgiContributorRegistry contributors;
  std::vector<SdfgiCascade> previous;
  const std::array anchors{glm::vec3(0),        glm::vec3(8, 0, 0), glm::vec3(-8, 8, 0),
                           glm::vec3(8, -8, 8), glm::vec3(0),       glm::vec3(1000)};
  for (uint32_t phase = 0; phase < anchors.size(); ++phase) {
    SCOPED_TRACE(phase);
    if (phase == 3)
      snapshot.erase(snapshot.begin() + 1);
    if (phase == 4)
      snapshot.back().material.emission = {0, 2, 0};
    contributors.Update(snapshot);
    HddagiUpdatePlan update, reset;
    ASSERT_TRUE(
        BuildHddagiUpdatePlan(probes, {}, anchors[phase], previous, contributors.changes, false, update).empty());
    ASSERT_TRUE(BuildHddagiUpdatePlan(probes, {}, anchors[phase], {}, {}, true, reset).empty());
    previous = update.cascades;
    PlatformLifecycleTestAccess::PreUpdate();
    std::array<std::shared_ptr<HddagiVoxelFrame>, 2> frames;
    const std::array fields{incremental, fresh};
    for (uint32_t kind = 0; kind < 2; ++kind) {
      const auto& field = fields[kind];
      frames[kind] = HddagiVoxelFrame::Create(*field, SdfgiTestAccess::HostLayouts(*render)[0], contributors,
                                              kind == 0 ? update : reset,
                                              phase == 4   ? UINT16_MAX
                                              : phase == 5 ? 1
                                                           : phase + 1);
      RenderGraph graph;
      RenderGraphResourceRegistry registry;
      field->Import(graph, registry);
      if (!field->initialization_recorded || kind == 1)
        graph.AddPass(field->ClearDescriptor(), [field](const RenderGraphExecutionContext& context) {
          Platform::RecordCommandsMainQueue([&](const VkCommandBuffer command) {
            field->Clear(command, context);
          });
        });
      frames[kind]->AddPasses(graph, registry, field);
      const auto plan = graph.Compile({});
      ASSERT_TRUE(plan.valid);
      graph.Execute(plan, registry);
    }
    PlatformLifecycleTestAccess::LateUpdate();
    Platform::WaitForFrameSubmissions("HDDAGI retained versus fresh comparison");
    for (uint32_t kind = 0; kind < 2; ++kind)
      frames[kind]->ReadStatusAfterFence(*fields[kind]);
    ASSERT_EQ(incremental->failure_flags, 0u);
    ASSERT_EQ(fresh->failure_flags, 0u);
    for (const auto name : {"VoxelBits", "Regions", "LightNeighbors", "Disocclusion", "Occlusion0", "Occlusion1"}) {
      std::array<std::vector<uint8_t>, 2> images;
      for (uint32_t kind = 0; kind < 2; ++kind) {
        const auto& image = fields[kind]->images.at(name);
        const auto extent = image.requirement.extent;
        const auto pixel = image.requirement.storage_format == VK_FORMAT_R16_UINT      ? 2
                           : image.requirement.storage_format == VK_FORMAT_R8_UINT     ? 1
                           : image.requirement.storage_format == VK_FORMAT_R32G32_UINT ? 8
                                                                                       : 4;
        VkBufferCreateInfo info{VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO};
        info.size = size_t(extent.width) * extent.height * extent.depth * pixel;
        info.usage = VK_BUFFER_USAGE_TRANSFER_DST_BIT;
        VmaAllocationCreateInfo allocation{};
        allocation.usage = VMA_MEMORY_USAGE_AUTO;
        allocation.flags = VMA_ALLOCATION_CREATE_HOST_ACCESS_RANDOM_BIT;
        allocation.requiredFlags = VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT;
        Buffer output(info, allocation);
        VkBufferImageCopy copy{};
        copy.imageSubresource = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 0, 1};
        copy.imageExtent = extent;
        output.CopyFromImage(*image.image, copy);
        output.DownloadVector(images[kind], info.size);
      }
      const auto mismatch = std::mismatch(images[0].begin(), images[0].end(), images[1].begin());
      EXPECT_EQ(mismatch.first, images[0].end())
          << name << " first differing byte " << std::distance(images[0].begin(), mismatch.first);
    }
    const auto cache_contents = read_image(*incremental, "HitCache");
    EXPECT_TRUE(std::all_of(cache_contents.begin(), cache_contents.end(), [](uint32_t v) {
      return v == 0;
    }));
    const glm::ivec3 probe_size = probes.ProbeSize();
    for (const auto name : {"History", "HistorySum", "Diffuse"}) {
      const auto values = read_image(*incremental, name);
      const auto& requirement = incremental->images.at(name).requirement;
      const int tile = std::string(name) == "Diffuse" ? 7 : 5;
      const int channels = std::string(name) == "HistorySum" ? 3 : 1;
      const bool history = std::string(name) == "History";
      size_t mismatches = 0;
      for (uint32_t layer = 0; layer < requirement.layers; ++layer) {
        const uint32_t cascade = history ? layer / incremental->settings.history_size : layer;
        const auto offset = (update.cascades[cascade].position - update.cascades[cascade].size / 2) / 8;
        for (uint32_t y = 0; y < requirement.extent.height; ++y)
          for (uint32_t x = 0; x < requirement.extent.width; ++x) {
            const glm::ivec3 physical(x / (tile * channels), (y / tile) % probe_size.y, (y / tile) / probe_size.y);
            const auto local = ((physical - offset) % probe_size + probe_size) % probe_size;
            const auto old = local + update.scroll[cascade] / 8;
            const bool retained = phase && !(update.reset_history_cascades & (1u << cascade)) &&
                                  glm::all(glm::greaterThanEqual(old, glm::ivec3(0))) &&
                                  glm::all(glm::lessThan(old, probe_size));
            const size_t index = (size_t(layer) * requirement.extent.height + y) * requirement.extent.width + x;
            mismatches += values[index] != (retained ? 123u : 0u);
          }
      }
      EXPECT_EQ(mismatches, 0u) << name;
    }
    HddagiCascadeBlock cascade_data;
    for (uint32_t c = 0; c < probes.cascade_count; ++c) {
      const auto& cascade = update.cascades[c];
      cascade_data.data[c].offset = glm::vec3(cascade.position - cascade.size / 2) * cascade.cell_size;
      cascade_data.data[c].to_cell = 1.0f / cascade.cell_size;
      cascade_data.data[c].region_world_offset = (cascade.position - cascade.size / 2) / 8;
    }
    std::vector<HddagiRay> rays;
    for (uint32_t c = 0; c < probes.cascade_count; ++c)
      for (int x = -1; x <= 1; ++x)
        for (int y = -1; y <= 1; ++y)
          for (int z = -1; z <= 1; ++z)
            if (x || y || z)
              rays.push_back({anchors[phase] + glm::vec3(0.25f), c, glm::normalize(glm::vec3(x, y, z)), 0});
    auto cascade_buffer = buffer(sizeof(cascade_data), VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT);
    auto ray_buffer = buffer(rays.size() * sizeof(HddagiRay), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT);
    auto result_buffer = buffer(rays.size() * 2 * sizeof(glm::uvec4), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT);
    cascade_buffer->Upload(cascade_data);
    ray_buffer->UploadVector(rays);
    cache_set->UpdateBufferDescriptorBinding(2, cascade_buffer);
    cache_set->UpdateBufferDescriptorBinding(3, ray_buffer);
    cache_set->UpdateBufferDescriptorBinding(4, result_buffer);
    std::vector<glm::uvec4> uncached;
    for (uint32_t mode = 0; mode < 4; ++mode) {
      Platform::ImmediateSubmit([&](const VkCommandBuffer command) {
        incremental->OrderAccess(command, VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT,
                                 VK_ACCESS_2_SHADER_READ_BIT | VK_ACCESS_2_SHADER_WRITE_BIT);
        cache_pipeline->Bind(command);
        cache_pipeline->BindDescriptorSet(command, 0, cache_set->GetVkDescriptorSet());
        HddagiTraceParams params;
        params.grid = update.cascades[0].size;
        params.ray_count = rays.size();
        params.cascade_count = probes.cascade_count;
        params.fractional_bits = mode != 0;
        cache_pipeline->PushConstant(command, 0, params);
        cache_pipeline->Dispatch(command, (rays.size() + 63) / 64, 1, 1);
        incremental->OrderAccess(command, VK_PIPELINE_STAGE_2_ALL_COMMANDS_BIT,
                                 VK_ACCESS_2_MEMORY_READ_BIT | VK_ACCESS_2_MEMORY_WRITE_BIT);
      });
      std::vector<glm::uvec4> results;
      result_buffer->DownloadVector(results, rays.size() * 2);
      if (!mode)
        uncached = results;
      for (size_t r = 0; r < rays.size(); ++r) {
        EXPECT_EQ(results[r * 2], uncached[r * 2]);
        EXPECT_EQ(results[r * 2 + 1].x, uncached[r * 2 + 1].x);
        const bool reused = mode == 3 || (mode == 2 && uncached[r * 2 + 1].x == 0);
        EXPECT_EQ(results[r * 2 + 1].y, reused ? 1u : 0u);
        if (reused)
          EXPECT_EQ(results[r * 2 + 1].z, 0u);
      }
      if (mode == 1) {
        Platform::ImmediateSubmit([&](const VkCommandBuffer command) {
          incremental->OrderAccess(command, VK_PIPELINE_STAGE_2_TRANSFER_BIT, VK_ACCESS_2_TRANSFER_WRITE_BIT);
          const VkImageSubresourceRange range{VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1};
          VkClearColorValue version{};
          version.uint32[0] = 777 + phase;
          Platform::ClearColorImage(command, *incremental->images.at("Versions").image, version, 1, &range);
          incremental->OrderAccess(command, VK_PIPELINE_STAGE_2_ALL_COMMANDS_BIT,
                                   VK_ACCESS_2_MEMORY_READ_BIT | VK_ACCESS_2_MEMORY_WRITE_BIT);
        });
      }
    }
    Platform::ImmediateSubmit([&](const VkCommandBuffer command) {
      incremental->OrderAccess(command, VK_PIPELINE_STAGE_2_TRANSFER_BIT, VK_ACCESS_2_TRANSFER_WRITE_BIT);
      for (const auto name : {"History", "HistorySum", "Diffuse"}) {
        const auto& image = incremental->images.at(name);
        const VkImageSubresourceRange range{VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, image.requirement.layers};
        VkClearColorValue sentinel{};
        sentinel.uint32[0] = 123;
        Platform::ClearColorImage(command, *image.image, sentinel, 1, &range);
      }
      incremental->OrderAccess(command, VK_PIPELINE_STAGE_2_ALL_COMMANDS_BIT,
                               VK_ACCESS_2_MEMORY_READ_BIT | VK_ACCESS_2_MEMORY_WRITE_BIT);
    });
    ASSERT_EQ(incremental->light_cell_counts, fresh->light_cell_counts);
    for (uint32_t cascade = 0; cascade < probes.cascade_count; ++cascade) {
      std::array<std::vector<HddagiProcessVoxel>, 2> payload;
      for (uint32_t kind = 0; kind < 2; ++kind) {
        fields[kind]
            ->buffers.at("Process" + std::to_string(cascade))
            ->DownloadVector(payload[kind], fields[kind]->light_cell_counts[cascade]);
        std::sort(payload[kind].begin(), payload[kind].end(), [](const auto& a, const auto& b) {
          return a.position < b.position;
        });
      }
      ASSERT_EQ(payload[0].size(), payload[1].size());
      if (!payload[0].empty())
        EXPECT_EQ(std::memcmp(payload[0].data(), payload[1].data(), payload[0].size() * sizeof(HddagiProcessVoxel)), 0);
    }
  }
}

TEST(HddagiCapture, SnapshotRetainsGenerationAcrossProviderReplacement) {
  ScopedGpuPlatform platform(false);
  auto runtime = std::make_shared<HddagiRuntime>();
  runtime->probes.probe_count_x = runtime->probes.probe_count_y = 9;
  runtime->probes.cascade_count = 1;
  std::string failure;
  runtime->resources = HddagiResources::TryCreate(runtime->probes, runtime->settings, failure);
  ASSERT_TRUE(runtime->resources) << failure;
  const auto field = runtime->resources;
  HddagiUpdatePlan placement;
  ASSERT_TRUE(BuildHddagiUpdatePlan(runtime->probes, {}, glm::vec3(0), {}, {}, true, placement).empty());
  runtime->cascades = placement.cascades;
  PlatformLifecycleTestAccess::PreUpdate();
  field->last_voxel_frame = Platform::GetFrameCount();
  const auto generation = uint32_t(field->last_voxel_frame + 1);
  field->submission = Platform::TrackCurrentFrameSubmission();
  RenderGraph graph;
  RenderGraphResourceRegistry registry;
  field->Import(graph, registry);
  graph.AddPass(field->ClearDescriptor(), [field](const RenderGraphExecutionContext& context) {
    Platform::RecordCommandsMainQueue([&](VkCommandBuffer command) {
      field->Clear(command, context);
    });
  });
  graph.Execute(graph.Compile({}), registry);
  PlatformLifecycleTestAccess::LateUpdate();
  Platform::WaitForFrameSubmissions("HDDAGI capture source initialization");
  const auto fill = [&](uint32_t value) {
    Platform::ImmediateSubmit([&](VkCommandBuffer command) {
      field->OrderAccess(command, VK_PIPELINE_STAGE_2_TRANSFER_BIT, VK_ACCESS_2_TRANSFER_WRITE_BIT);
      VkClearColorValue color{};
      color.uint32[0] = value;
      const VkImageSubresourceRange range{VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1};
      Platform::ClearColorImage(command, *field->images.at("FilteredDiffuse").image, color, 1, &range);
      field->OrderAccess(command, VK_PIPELINE_STAGE_2_ALL_COMMANDS_BIT,
                         VK_ACCESS_2_MEMORY_READ_BIT | VK_ACCESS_2_MEMORY_WRITE_BIT);
    });
  };
  field->buffers.at("Status")->Upload(std::array<uint32_t, 4>{generation, 0, 1, 0});
  field->transport_ready = field->transport_recorded = true;
  const uint32_t radiance = (16u << 27) | (64u << 18) | (128u << 9) | 256u;
  fill(radiance);
  field->submission->status = FrameSubmissionState::Status::Pending;
  EXPECT_FALSE(SnapshotHddagiCapture(runtime, true));
  field->submission->status = FrameSubmissionState::Status::Submitted;
  PlatformLifecycleTestAccess::PreUpdate();
  EXPECT_TRUE(SnapshotHddagiCapture(runtime, true));
  auto snapshot = SnapshotHddagiCapture(runtime);
  ASSERT_TRUE(snapshot);
  EXPECT_NE(snapshot->resources->images.at("FilteredDiffuse").image, field->images.at("FilteredDiffuse").image);
  EXPECT_EQ(snapshot->frame.anchor.camera_id, runtime->frame.anchor.camera_id);
  PlatformLifecycleTestAccess::LateUpdate();
  Platform::WaitForFrameSubmissions("HDDAGI capture snapshot copy");
  fill(0);
  field->buffers.at("Status")->Upload(std::array<uint32_t, 4>{generation + 1, 2, 0, 0});
  runtime.reset();
  std::array<uint32_t, 4> status;
  snapshot->resources->buffers.at("Status")->Download(status);
  EXPECT_EQ(status, (std::array<uint32_t, 4>{generation, 0, 1, 0}));
  VkBufferCreateInfo info{VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO};
  info.size = 4;
  info.usage = VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_TRANSFER_SRC_BIT;
  Buffer output(info);
  VkBufferImageCopy copy{};
  copy.imageSubresource = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 0, 1};
  copy.imageExtent = {1, 1, 1};
  output.CopyFromImage(*snapshot->resources->images.at("FilteredDiffuse").image, copy);
  uint32_t actual;
  output.Download(actual);
  EXPECT_EQ(actual, radiance);
}

TEST(HddagiCamera, ResizesRetainLiveImagesAndKeepSceneProbeStorage) {
  ScopedGpuPlatform platform(false);
  ApplicationContext::Get().RegisterAsset<Shader>("Shader", {".eveshader", ".slang"});
  HddagiRuntime runtime;
  runtime.probes.probe_count_x = runtime.probes.probe_count_y = 9;
  runtime.probes.cascade_count = 1;
  std::string failure;
  runtime.resources = HddagiResources::TryCreate(runtime.probes, runtime.settings, failure);
  ASSERT_TRUE(runtime.resources) << failure;
  HddagiUpdatePlan plan;
  ASSERT_TRUE(BuildHddagiUpdatePlan(runtime.probes, runtime.settings, glm::vec3(0), {}, {}, true, plan).empty());
  runtime.cascades = plan.cascades;
  VkSamplerCreateInfo sampler{VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO};
  sampler.magFilter = sampler.minFilter = VK_FILTER_LINEAR;
  runtime.resources->linear_sampler = std::make_shared<Sampler>(sampler);
  RenderInstanceStorage instances;
  const auto layouts = SdfgiTestAccess::HostLayouts(*ApplicationContext::Get().GetLayer<RenderLayer>());
  const auto probe_image = runtime.resources->images.at("Diffuse").image;
  auto first = HddagiCameraFrame::Create(runtime, instances, layouts, {65, 33}, 1, false);
  EXPECT_EQ(first->params.gi_size, glm::uvec2(32, 16));
  const auto shared = HddagiCameraFrame::Create(runtime, instances, layouts, {65, 33}, 1, false);
  EXPECT_EQ(shared->images, first->images);
  const auto second = HddagiCameraFrame::Create(runtime, instances, layouts, {1, 1}, 1, false);
  EXPECT_EQ(second->params.gi_size, glm::uvec2(1));
  EXPECT_NE(second->images, first->images);
  EXPECT_EQ(runtime.resources->images.at("Diffuse").image, probe_image);
  EXPECT_EQ(first->images->images.at("Surface").requirement.extent.width, 65u);
  EXPECT_THROW(HddagiCameraFrame::Create(runtime, instances, layouts, {UINT32_MAX, 1}, 1, false),
               std::invalid_argument);
  EXPECT_EQ(runtime.resources->camera_images.at(1), second->images);
  const auto capture = HddagiCameraFrame::Create(runtime, instances, layouts, {33, 17}, 2, true);
  EXPECT_EQ(capture->params.reflection_capture, 1u);
  EXPECT_NE(capture->images, second->images);
}

TEST(HddagiGather, ConstantRadianceAndZeroVisibilityAtProbeAndCascadeEdgesWithoutRt) {
  ScopedGpuPlatform platform(false);
  ApplicationContext::Get().RegisterAsset<Shader>("Shader", {".eveshader", ".slang"});
  const auto source =
      std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / "EvoEngine_SDK/Internals/DefaultResources/Shaders";
  Shader::RegisterShaderIncludePath(source / "Modules");
  GiProbeSettings probes;
  probes.probe_count_x = 9;
  probes.probe_count_y = 11;
  probes.cascade_count = 2;
  std::string failure;
  auto field = HddagiResources::TryCreate(probes, {}, failure);
  ASSERT_TRUE(field) << failure;
  HddagiUpdatePlan placement;
  ASSERT_TRUE(BuildHddagiUpdatePlan(probes, {}, {-17, 2, -35}, {}, {}, true, placement).empty());
  auto data = BuildHddagiGatherData(probes, {}, placement.cascades, {-17, 2, -35});
  data.occlusion_bias = 0;
  struct Input {
    glm::vec3 position;
    uint32_t cascade;
    glm::vec3 normal;
    float roughness;
    glm::vec3 reflection;
    uint32_t dynamic_receiver;
  };
  static_assert(sizeof(Input) == 48);
  std::vector<Input> inputs;
  for (uint32_t c = 0; c < 2; ++c)
    for (uint32_t dynamic = 0; dynamic < 2; ++dynamic)
      for (const auto position : {glm::vec3(0), glm::vec3(8, 16, 24), glm::vec3(63.99f, 79.99f, 63.99f)})
        for (const auto roughness : {0.1f, 0.3f, 0.7f, 1.0f})
          inputs.push_back({position, c, glm::normalize(glm::vec3(1, 2, -3)), roughness, {0, 0, 1}, dynamic});
  const auto buffer = [](size_t size, VkBufferUsageFlags usage) {
    VkBufferCreateInfo info{VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO};
    info.size = size;
    info.usage = usage | VK_BUFFER_USAGE_TRANSFER_SRC_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT;
    return std::make_shared<Buffer>(info);
  };
  const auto metadata = buffer(sizeof(data), VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT);
  const auto input = buffer(inputs.size() * sizeof(Input), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT);
  const auto output = buffer(inputs.size() * sizeof(glm::vec4) * 2, VK_BUFFER_USAGE_STORAGE_BUFFER_BIT);
  metadata->Upload(data);
  input->UploadVector(inputs);
  auto layout = std::make_shared<DescriptorSetLayout>();
  const std::array types{
      VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE,  VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE,
      VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE,  VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE,  VK_DESCRIPTOR_TYPE_SAMPLER,
      VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE,
      VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE,  VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE,  VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE,
      VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE};
  for (uint32_t i = 0; i < types.size(); ++i)
    layout->PushDescriptorBinding(i, types[i], VK_SHADER_STAGE_COMPUTE_BIT, 0);
  layout->Initialize();
  ComputePipeline pipeline;
  pipeline.descriptor_set_layouts = {layout};
  pipeline.compute_shader = Shader::CreateTemporary(ShaderType::Compute, "", source / "Compute/HddagiGatherAbi.slang");
  pipeline.Initialize();
  ASSERT_TRUE(pipeline.Initialized());
  auto set = std::make_shared<DescriptorSet>(layout);
  set->UpdateBufferDescriptorBinding(0, metadata);
  set->UpdateBufferDescriptorBinding(6, input);
  set->UpdateBufferDescriptorBinding(7, output);
  VkSamplerCreateInfo sampler_info{VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO};
  sampler_info.magFilter = sampler_info.minFilter = VK_FILTER_LINEAR;
  sampler_info.addressModeU = sampler_info.addressModeV = sampler_info.addressModeW =
      VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
  Sampler sampler(sampler_info);
  VkDescriptorImageInfo info{};
  info.sampler = sampler.GetVkSampler();
  set->UpdateImageDescriptorBinding(5, info);
  info.sampler = VK_NULL_HANDLE;
  info.imageLayout = VK_IMAGE_LAYOUT_GENERAL;
  uint32_t binding = 1;
  for (const auto name : {"Diffuse", "Specular", "Occlusion0", "Occlusion1"}) {
    info.imageView = field->images.at(name).sampled->GetVkImageView();
    set->UpdateImageDescriptorBinding(binding++, info);
  }
  binding = 8;
  for (const auto name : {"VoxelBits", "Regions", "Light", "Disocclusion", "LightNeighbors"}) {
    info.imageView = field->images.at(name).sampled->GetVkImageView();
    set->UpdateImageDescriptorBinding(binding++, info);
  }
  for (uint32_t phase = 0; phase < 2; ++phase) {
    Platform::ImmediateSubmit([&](const VkCommandBuffer command) {
      for (const auto name : {"Diffuse", "Specular", "Occlusion0", "Occlusion1"}) {
        const auto& texture = field->images.at(name);
        texture.image->TransitImageLayout(command, VK_IMAGE_LAYOUT_GENERAL);
        const VkImageSubresourceRange range{VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, texture.requirement.layers};
        VkClearColorValue value{};
        value.uint32[0] = std::string(name) == "Diffuse"    ? (16u << 27) | (64u << 18) | (128u << 9) | 256u
                          : std::string(name) == "Specular" ? (16u << 27) | (256u << 18) | (64u << 9) | 128u
                          : phase == 0                      ? 0xffffu
                                                            : 0;
        Platform::ClearColorImage(command, *texture.image, value, 1, &range);
      }
      field->OrderAccess(command, VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT, VK_ACCESS_2_SHADER_READ_BIT);
      pipeline.Bind(command);
      pipeline.BindDescriptorSet(command, 0, set->GetVkDescriptorSet());
      pipeline.Dispatch(command, (inputs.size() + 63) / 64, 1, 1);
      Platform::BufferMemoryBarrier(command, *output);
    });
    std::vector<glm::vec4> actual;
    output->DownloadVector(actual, inputs.size() * 2);
    for (size_t i = 0; i < inputs.size(); ++i) {
      SCOPED_TRACE(i);
      const glm::vec3 diffuse = phase ? glm::vec3(0) : glm::vec3(1, 0.5f, 0.25f);
      const glm::vec3 specular =
          phase ? glm::vec3(0)
                : glm::mix(glm::vec3(0.5f, 0.25f, 1), diffuse, glm::smoothstep(0.25f, 1.0f, inputs[i].roughness));
      EXPECT_LT(glm::length(glm::vec3(actual[i * 2]) - diffuse), 0.001f);
      EXPECT_LT(glm::length(glm::vec3(actual[i * 2 + 1]) - specular), 0.001f);
    }
  }
  ComputePipeline full;
  full.descriptor_set_layouts = {layout};
  full.compute_shader = Shader::CreateTemporary(ShaderType::Compute, "#define MODE_FULL_GATHER\n",
                                                source / "Compute/HddagiGatherAbi.slang");
  full.Initialize();
  ASSERT_TRUE(full.Initialized());
  const glm::vec3 world_anchor = data.anchor_origin / glm::vec3(1, data.y_mult, 1);
  for (size_t i = 0; i < inputs.size(); ++i) {
    inputs[i].position = world_anchor + (i % 3 == 0 ? glm::vec3(10000) : glm::vec3(0));
    inputs[i].roughness = i % 3 == 2 ? 0 : 1;
  }
  input->UploadVector(inputs);
  Platform::ImmediateSubmit([&](const VkCommandBuffer command) {
    for (const auto name :
         {"VoxelBits", "Regions", "Light", "Disocclusion", "LightNeighbors", "Occlusion0", "Occlusion1"}) {
      const auto& texture = field->images.at(name);
      texture.image->TransitImageLayout(command, VK_IMAGE_LAYOUT_GENERAL);
      const VkImageSubresourceRange range{VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, texture.requirement.layers};
      VkClearColorValue value{};
      if (std::string(name).find("Occlusion") == 0)
        value.uint32[0] = 0xffffu;
      Platform::ClearColorImage(command, *texture.image, value, 1, &range);
    }
    field->OrderAccess(command, VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT, VK_ACCESS_2_SHADER_READ_BIT);
    full.Bind(command);
    full.BindDescriptorSet(command, 0, set->GetVkDescriptorSet());
    full.Dispatch(command, (inputs.size() + 63) / 64, 1, 1);
    Platform::BufferMemoryBarrier(command, *output);
  });
  std::vector<glm::vec4> actual;
  output->DownloadVector(actual, inputs.size() * 2);
  for (size_t i = 0; i < inputs.size(); ++i) {
    const glm::vec4 expected = i % 3 == 0 ? glm::vec4(0) : glm::vec4(1, 0.5f, 0.25f, 1);
    EXPECT_LT(glm::length(actual[i * 2] - expected), 0.001f);
    EXPECT_LT(glm::length(actual[i * 2 + 1] - (i % 3 == 2 ? glm::vec4(0) : expected)), 0.001f);
  }
  for (auto& value : inputs) {
    value.position = world_anchor;
    value.roughness = 0;
  }
  input->UploadVector(inputs);
  for (const uint32_t mask : {0u, 63u}) {
    Platform::ImmediateSubmit([&](const VkCommandBuffer command) {
      field->OrderAccess(command, VK_PIPELINE_STAGE_2_TRANSFER_BIT, VK_ACCESS_2_TRANSFER_WRITE_BIT);
      for (const auto name : {"VoxelBits", "Regions", "Light", "Disocclusion"}) {
        const auto& texture = field->images.at(name);
        const VkImageSubresourceRange range{VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, texture.requirement.layers};
        VkClearColorValue value{};
        value.uint32[0] = std::string(name) == "Light"          ? (16u << 27) | (64u << 18) | (128u << 9) | 256u
                          : std::string(name) == "Disocclusion" ? mask
                                                                : UINT32_MAX;
        value.uint32[1] = UINT32_MAX;
        Platform::ClearColorImage(command, *texture.image, value, 1, &range);
      }
      field->OrderAccess(command, VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT, VK_ACCESS_2_SHADER_READ_BIT);
      full.Bind(command);
      full.BindDescriptorSet(command, 0, set->GetVkDescriptorSet());
      full.Dispatch(command, (inputs.size() + 63) / 64, 1, 1);
      Platform::BufferMemoryBarrier(command, *output);
    });
    output->DownloadVector(actual, inputs.size() * 2);
    for (size_t i = 0; i < inputs.size(); ++i)
      EXPECT_LT(glm::length(actual[i * 2 + 1] - glm::vec4(1, 0.5f, 0.25f, 1)), 0.001f);
  }
}

TEST(HddagiLighting, StaticEditsEmissionAndNativePhotometryWithoutRayTracing) {
  ScopedGpuPlatform platform(false);
  ApplicationContext::Get().RegisterAsset<Shader>("Shader", {".eveshader", ".slang"});
  GiProbeSettings probes;
  probes.probe_count_x = probes.probe_count_y = 9;
  probes.cascade_count = 1;
  probes.base_probe_distance = 8;
  probes.vertical_scale = GiProbeSettings::VerticalScale::Percent50;
  std::string failure;
  auto field = HddagiResources::TryCreate(probes, {}, failure);
  ASSERT_TRUE(field) << failure;
  HddagiUpdatePlan placement;
  ASSERT_TRUE(BuildHddagiUpdatePlan(probes, {}, glm::vec3(0), {}, {}, true, placement).empty());
  PlatformLifecycleTestAccess::PreUpdate();
  RenderGraph initialize;
  RenderGraphResourceRegistry initial_registry;
  field->Import(initialize, initial_registry);
  initialize.AddPass(field->ClearDescriptor(), [field](const RenderGraphExecutionContext& context) {
    Platform::RecordCommandsMainQueue([&](const VkCommandBuffer command) {
      field->Clear(command, context);
    });
  });
  initialize.Execute(initialize.Compile({}), initial_registry);
  PlatformLifecycleTestAccess::LateUpdate();
  Platform::WaitForFrameSubmissions("HDDAGI light fixture initialization");
  const auto encode = [](glm::vec3 v) {
    const int exponent = std::max(-16, int(std::floor(std::log2(std::max(v.x, std::max(v.y, v.z)))))) + 16;
    const float scale = std::ldexp(1.0f, exponent - 24);
    const auto mantissa = glm::uvec3(glm::floor(v / scale + 0.5f));
    return (uint32_t(exponent) << 27) | (mantissa.z << 18) | (mantissa.y << 9) | mantissa.x;
  };
  const auto decode = [](uint32_t packed) {
    return glm::vec3(packed & 511, (packed >> 9) & 511, (packed >> 18) & 511) *
           std::ldexp(1.0f, int(packed >> 27) - 24);
  };
  HddagiProcessVoxel voxel;
  voxel.position = 32 | (32 << 8) | (32 << 16) | 0xc0000000u;
  voxel.albedo_normal = 0xffffu | (128u << 16) | (128u << 24);
  voxel.emission = encode({0.125f, 0.25f, 0.5f});
  voxel.occlusion = 15;
  field->buffers.at("Process0")->Upload(voxel);
  field->buffers.at("Dispatch0")->Upload(std::array<uint32_t, 5>{1, 1, 1, 1, 0});
  const glm::vec3 emission = decode(voxel.emission);
  const glm::vec3 normal = glm::normalize(glm::vec3(1, 1, 253));
  SdfgiLightInput positional;
  positional.id = 1;
  positional.type = SdfgiLightInput::Type::Point;
  positional.dynamic = false;
  positional.color = {0.25f, 0.5f, 1};
  positional.position = {0.5f, 0.25f, 4.5f};
  positional.range = 10;
  positional.world_bounds = {positional.position - positional.range, positional.position + positional.range};
  SdfgiLightInput directional;
  directional.id = 2;
  directional.color = {0.5f, 0.25f, 0.125f};
  directional.direction = {0, 0, -1};
  VkBufferCreateInfo output_info{VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO};
  output_info.size = 4;
  output_info.usage = VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_TRANSFER_SRC_BIT;
  Buffer output(output_info);
  for (uint32_t phase = 0; phase < 9; ++phase) {
    SCOPED_TRACE(phase);
    std::vector<SdfgiLightInput> lights;
    glm::vec3 expected = emission;
    if (phase <= 1) {
      positional.color = phase == 0 ? glm::vec3(0.25f, 0.5f, 1) : glm::vec3(1, 0.25f, 0.5f);
      lights.push_back(positional);
      expected += positional.color * normal.z;
      if (phase == 0) {
        lights.push_back(directional);
        expected += directional.color * normal.z;
      }
    } else if (phase >= 3 && phase < 7) {
      positional.dynamic = true;
      positional.attenuation = {1, 0, 1};
      positional.type = phase == 3 ? SdfgiLightInput::Type::Point : SdfgiLightInput::Type::Spot;
      positional.cos_inner = 0.95f;
      positional.cos_outer = 0.8f;
      positional.direction = {0, 0, -1};
      positional.casts_shadow = phase == 5;
      lights.push_back(positional);
      if (phase != 5)
        expected += positional.color * normal.z / 17.0f;
    }
    if (phase >= 7) {
      field->transport_recorded = true;
      field->buffers.at("Status")->Upload(std::array<uint32_t, 4>{1, 0, 1, phase == 7 ? 1u : 0u});
      const glm::vec3 bounce(0.25f, 0.5f, 0.125f);
      if (phase == 7)
        expected += bounce;
      Platform::ImmediateSubmit([&](const VkCommandBuffer command) {
        field->OrderAccess(command, VK_PIPELINE_STAGE_2_TRANSFER_BIT, VK_ACCESS_2_TRANSFER_WRITE_BIT);
        const VkImageSubresourceRange range{VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1};
        VkClearColorValue value{};
        value.uint32[0] = encode(bounce);
        Platform::ClearColorImage(command, *field->images.at("FilteredDiffuse").image, value, 1, &range);
        field->OrderAccess(command, VK_PIPELINE_STAGE_2_ALL_COMMANDS_BIT,
                           VK_ACCESS_2_MEMORY_READ_BIT | VK_ACCESS_2_MEMORY_WRITE_BIT);
      });
    }
    if (phase == 5) {
      Platform::ImmediateSubmit([&](const VkCommandBuffer command) {
        field->OrderAccess(command, VK_PIPELINE_STAGE_2_TRANSFER_BIT, VK_ACCESS_2_TRANSFER_WRITE_BIT);
        const VkImageSubresourceRange range{VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1};
        VkClearColorValue occupied{};
        occupied.uint32[0] = occupied.uint32[1] = UINT32_MAX;
        for (const auto name : {"VoxelBits", "Regions"})
          Platform::ClearColorImage(command, *field->images.at(name).image, occupied, 1, &range);
        field->OrderAccess(command, VK_PIPELINE_STAGE_2_ALL_COMMANDS_BIT,
                           VK_ACCESS_2_MEMORY_READ_BIT | VK_ACCESS_2_MEMORY_WRITE_BIT);
      });
    }
    PlatformLifecycleTestAccess::PreUpdate();
    auto frame = HddagiLightFrame::Create(*field, placement.cascades, lights, phase, 0);
    RenderGraph graph;
    RenderGraphResourceRegistry registry;
    field->Import(graph, registry);
    frame->AddPasses(graph, registry, field, "");
    const auto plan = graph.Compile({});
    ASSERT_TRUE(plan.valid);
    graph.Execute(plan, registry);
    PlatformLifecycleTestAccess::LateUpdate();
    Platform::WaitForFrameSubmissions("HDDAGI direct-light reference fixture");
    VkBufferImageCopy copy{};
    copy.imageSubresource = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 0, 1};
    copy.imageExtent = {1, 1, 1};
    output.CopyFromImage(*field->images.at("Light").image, copy);
    uint32_t packed;
    output.Download(packed);
    const auto actual = decode(packed);
    for (uint32_t axis = 0; axis < 3; ++axis)
      EXPECT_NEAR(actual[axis], expected[axis], 0.009f);
    HddagiProcessVoxel retained;
    field->buffers.at("Process0")->Download(retained);
    EXPECT_EQ(retained.emission, voxel.emission);
    EXPECT_EQ(retained.position & 0xc0000000u, 0u);
  }
}

TEST(HddagiLighting, ProbeHistoryRingsConserveSkyEnergyAcrossCascadesAndRemoveOldRadiance) {
  ScopedGpuPlatform platform(false);
  ApplicationContext::Get().RegisterAsset<Shader>("Shader", {".eveshader", ".slang"});
  GiProbeSettings probes;
  probes.probe_count_x = 9;
  probes.probe_count_y = 11;
  probes.cascade_count = 2;
  HddagiSettings settings;
  settings.history_size = 6;
  std::string failure;
  auto field = HddagiResources::TryCreate(probes, settings, failure);
  ASSERT_TRUE(field) << failure;
  HddagiUpdatePlan placement;
  ASSERT_TRUE(BuildHddagiUpdatePlan(probes, settings, glm::vec3(-3), {}, {}, true, placement).empty());
  const auto read = [&](const char* name, const uint32_t bytes) {
    const auto& image = field->images.at(name);
    const auto extent = image.requirement.extent;
    VkBufferCreateInfo info{VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO};
    info.size = size_t(extent.width) * extent.height * image.requirement.layers * bytes;
    info.usage = VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_TRANSFER_SRC_BIT;
    Buffer output(info);
    VkBufferImageCopy copy{};
    copy.imageSubresource = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 0, image.requirement.layers};
    copy.imageExtent = extent;
    output.CopyFromImage(*image.image, copy);
    std::vector<uint32_t> data;
    output.DownloadVector(data, info.size / 4);
    return data;
  };
  SdfgiSkyInput sky;
  sky.constant_color = true;
  sky.color = {0.25f, 0.5f, 1};
  for (uint32_t frame_index = 0; frame_index < 12; ++frame_index) {
    SCOPED_TRACE(frame_index);
    if (frame_index == 6)
      sky.color = glm::vec3(0);
    PlatformLifecycleTestAccess::PreUpdate();
    auto frame = HddagiProbeFrame::Create(*field, placement.cascades, sky, frame_index, true);
    RenderGraph graph;
    RenderGraphResourceRegistry registry;
    field->Import(graph, registry);
    if (!frame_index)
      graph.AddPass(field->ClearDescriptor(), [field](const RenderGraphExecutionContext& context) {
        Platform::RecordCommandsMainQueue([&](const VkCommandBuffer command) {
          field->Clear(command, context);
        });
      });
    frame->AddBeginPass(graph, registry, field, frame_index ? "" : "HddagiInitialize");
    frame->AddPasses(graph, registry, field, "HddagiTransportBegin");
    const auto plan = graph.Compile({});
    ASSERT_TRUE(plan.valid);
    graph.Execute(plan, registry);
    PlatformLifecycleTestAccess::LateUpdate();
    Platform::WaitForFrameSubmissions("HDDAGI probe temporal fixture");
    frame->ReadStatusAfterFence(*field);
    ASSERT_TRUE(field->transport_ready);
    ASSERT_EQ(field->transport_failure_flags, 0u);
    EXPECT_EQ(field->transport_generation, frame_index + 1);
    const auto history = read("History", 4);
    const auto sum = read("HistorySum", 4);
    const auto ambient = read("Ambient", 8);
    const auto specular = read("Specular", 4);
    const auto diffuse = read("Diffuse", 4);
    EXPECT_EQ(read("FilteredDiffuse", 4), diffuse);
    const auto counters = read("ProcessFrame", 4);
    const uint32_t width = probes.probe_count_x * 5;
    const uint32_t height = probes.probe_count_y * probes.probe_count_x * 5;
    for (uint32_t c = 0; c < probes.cascade_count; ++c) {
      for (uint32_t y = 0; y < height; ++y)
        for (uint32_t x = 0; x < width; ++x) {
          glm::uvec3 expected(0);
          for (uint32_t phase = 0; phase < settings.history_size; ++phase) {
            const uint32_t packed = history[((c * settings.history_size + phase) * height + y) * width + x];
            glm::uvec3 mantissa(packed & 511, (packed >> 9) & 511, (packed >> 18) & 511);
            const int shift = int(packed >> 27) - 10;
            expected += shift >= 0 ? mantissa << uint32_t(shift) : mantissa >> uint32_t(-shift);
          }
          for (uint32_t channel = 0; channel < 3; ++channel)
            ASSERT_EQ(sum[((c * height + y) * width + x) * 3 + channel], expected[channel]);
        }
      const uint32_t probe_count = probes.probe_count_x * probes.probe_count_y * probes.probe_count_x;
      const float fill = frame_index < 6 ? float(frame_index + 1) / 6 : float(11 - frame_index) / 6;
      for (uint32_t probe = 0; probe < probe_count; ++probe) {
        ASSERT_EQ(counters[c * probe_count + probe], frame_index + 1);
        const auto rg = glm::unpackHalf2x16(ambient[(c * probe_count + probe) * 2]);
        const auto ba = glm::unpackHalf2x16(ambient[(c * probe_count + probe) * 2 + 1]);
        ASSERT_NEAR(rg.x, 0.25f * fill, 0.003f);
        ASSERT_NEAR(rg.y, 0.5f * fill, 0.003f);
        ASSERT_NEAR(ba.x, fill, 0.003f);
        ASSERT_EQ(ba.y, 1);
      }
    }
    if (frame_index == 11) {
      const auto dark = [](uint32_t packed) {
        return (packed & 0x07ffffffu) == 0;
      };
      EXPECT_TRUE(std::all_of(specular.begin(), specular.end(), dark));
      EXPECT_TRUE(std::all_of(diffuse.begin(), diffuse.end(), dark));
      EXPECT_TRUE(std::all_of(sum.begin(), sum.end(), [](uint32_t value) {
        return value == 0;
      }));
    }
  }
  for (uint32_t phase = 0; phase < 3; ++phase) {
    SCOPED_TRACE(phase);
    field->buffers.at("Dispatch1")->Upload(std::array<uint32_t, 5>{0, 0, 0, 0, phase == 0 ? 1u : 0u});
    sky.color = phase == 1 ? glm::vec3(NAN) : glm::vec3(1);
    PlatformLifecycleTestAccess::PreUpdate();
    auto frame = HddagiProbeFrame::Create(*field, placement.cascades, sky, 12 + phase, true);
    RenderGraph graph;
    RenderGraphResourceRegistry registry;
    field->Import(graph, registry);
    frame->AddBeginPass(graph, registry, field, "");
    frame->AddPasses(graph, registry, field, "HddagiTransportBegin");
    const auto plan = graph.Compile({});
    ASSERT_TRUE(plan.valid);
    graph.Execute(plan, registry);
    PlatformLifecycleTestAccess::LateUpdate();
    Platform::WaitForFrameSubmissions("HDDAGI publication failure and recovery fixture");
    frame->ReadStatusAfterFence(*field);
    EXPECT_EQ(field->transport_ready, phase == 2);
    EXPECT_EQ(field->transport_failure_flags, phase == 0 ? 1u : phase == 1 ? 2u : 0u);
    if (phase == 2) {
      const auto counters = read("ProcessFrame", 4);
      EXPECT_TRUE(std::all_of(counters.begin(), counters.end(), [](uint32_t value) {
        return value == 1;
      }));
    }
  }
}

TEST(HddagiLighting, OcclusionSeparatesWallSidesAndPreservesWrappedMarginsAndPackedChannels) {
  ScopedGpuPlatform platform(false);
  ApplicationContext::Get().RegisterAsset<Shader>("Shader", {".eveshader", ".slang"});
  GiProbeSettings probes;
  probes.probe_count_x = probes.probe_count_y = 9;
  probes.cascade_count = 1;
  std::string failure;
  auto field = HddagiResources::TryCreate(probes, {}, failure);
  ASSERT_TRUE(field) << failure;
  HddagiUpdatePlan placement;
  ASSERT_TRUE(BuildHddagiUpdatePlan(probes, {}, glm::vec3(0), {}, {}, true, placement).empty());
  auto frame = HddagiVoxelFrame::Create(
      *field, SdfgiTestAccess::HostLayouts(*ApplicationContext::Get().GetLayer<RenderLayer>())[0], {}, placement, 1);
  auto layout = std::make_shared<DescriptorSetLayout>();
  layout->PushDescriptorBinding(0, VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, VK_SHADER_STAGE_COMPUTE_BIT, 0);
  layout->Initialize();
  auto shader = std::make_shared<Shader>();
  ASSERT_TRUE(shader->TryCompile(ShaderType::Compute, std::string(R"(
struct Params { uint pattern; };
[[vk::push_constant]] ConstantBuffer<Params> params;
[[vk::binding(0, 0)]] [vk::image_format("r32ui")] RWTexture3D<uint> normals;
[numthreads(8, 8, 8)]
void main(uint3 id : SV_DispatchThreadID) { normals[id] = params.pattern == 1 || (params.pattern == 2 && id.x % 8 == 4) ? 1 : 0;
})")));
  ComputePipeline fill;
  fill.compute_shader = shader;
  fill.descriptor_set_layouts = {layout};
  fill.push_constant_ranges.push_back({VK_SHADER_STAGE_COMPUTE_BIT, 0, 4});
  fill.Initialize();
  auto set = std::make_shared<DescriptorSet>(layout);
  VkDescriptorImageInfo image_info{};
  image_info.imageLayout = VK_IMAGE_LAYOUT_GENERAL;
  image_info.imageView = field->images.at("NormalBits").storage->GetVkImageView();
  set->UpdateImageDescriptorBinding(0, image_info);
  PlatformLifecycleTestAccess::PreUpdate();
  RenderGraph graph;
  RenderGraphResourceRegistry registry;
  field->Import(graph, registry);
  graph.AddPass(field->ClearDescriptor(), [field](const RenderGraphExecutionContext& context) {
    Platform::RecordCommandsMainQueue([&](const VkCommandBuffer command) {
      field->Clear(command, context);
    });
  });
  graph.Execute(graph.Compile({}), registry);
  PlatformLifecycleTestAccess::LateUpdate();
  Platform::WaitForFrameSubmissions("HDDAGI occlusion fixture initialization");
  for (uint32_t pattern = 0; pattern < 3; ++pattern) {
    SCOPED_TRACE(pattern);
    Platform::ImmediateSubmit([&](const VkCommandBuffer command) {
      field->OrderAccess(command, VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT, VK_ACCESS_2_SHADER_WRITE_BIT);
      fill.Bind(command);
      fill.BindDescriptorSet(command, 0, set->GetVkDescriptorSet());
      fill.PushConstant(command, 0, pattern);
      fill.Dispatch(command, 8, 8, 8);
      field->OrderAccess(command, VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT,
                         VK_ACCESS_2_SHADER_READ_BIT | VK_ACCESS_2_SHADER_WRITE_BIT);
      frame->occlusion->Bind(command);
      for (uint32_t plane = 0; plane < 2; ++plane) {
        frame->occlusion->BindDescriptorSet(command, 0, frame->occlusion_sets[plane]->GetVkDescriptorSet());
        HddagiOcclusionParams params;
        params.grid = glm::ivec3(64);
        params.region_world_offset = glm::ivec3(-4);
        params.layer_offset = (int32_t(plane) - 1) * 4;
        frame->occlusion->PushConstant(command, 0, params);
        frame->occlusion->Dispatch(command, 8, 8, 8);
      }
      field->OrderAccess(command, VK_PIPELINE_STAGE_2_ALL_COMMANDS_BIT,
                         VK_ACCESS_2_MEMORY_READ_BIT | VK_ACCESS_2_MEMORY_WRITE_BIT);
    });
    for (uint32_t plane = 0; plane < 2; ++plane) {
      VkBufferCreateInfo info{VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO};
      info.size = 66 * 66 * 66 * 2;
      info.usage = VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_TRANSFER_SRC_BIT;
      Buffer output(info);
      VkBufferImageCopy copy{};
      copy.imageSubresource = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 0, 1};
      copy.imageExtent = {66, 66, 66};
      output.CopyFromImage(*field->images.at("Occlusion" + std::to_string(plane)).image, copy);
      std::vector<uint16_t> data;
      output.DownloadVector(data, 66 * 66 * 66);
      for (uint32_t z = 0; z < 66; ++z)
        for (uint32_t y = 0; y < 66; ++y)
          for (uint32_t x = 0; x < 66; ++x) {
            const uint32_t physical_x = (x + 63) % 64;
            if (pattern == 2 && (physical_x % 8 == 3 || physical_x % 8 == 4))
              continue;
            const bool high_x = (physical_x % 8 >= 5) != (physical_x / 8 % 2 != 0);
            const uint16_t expected = pattern == 0 ? 0xffff : pattern == 1 ? 0 : high_x ? 0x0f0f : 0xf0f0;
            ASSERT_EQ(data[x + 66 * (y + 66 * z)], expected) << "xyz " << x << "," << y << "," << z;
          }
    }
  }
}

TEST(HddagiReflectionCapture, LiveBakeDynamicUpdatesAndLayoutResetWithoutRt) {
  TempProject project;
  Application app;
  struct Terminate {
    Application& app;
    ~Terminate() {
      app.Terminate();
    }
  } terminate{app};
  app.PushLayer<RenderLayer>("Render Layer");
  auto initialization = TestApplicationSettings(project);
  initialization.load_default_resources = true;
  initialization.load_project_assets = true;
  initialization.load_project_start_scene = true;
  const auto* sponza_resources = static_cast<const char*>(nullptr);
  if (sponza_resources)
    SetupDemoScene(DemoSetup::Rendering, initialization, sponza_resources, false);
  initialization.graphics_settings.use_ray_tracing = false;
  app.Initialize(initialization);
  app.Start();
  for (uint32_t frame = 0; frame < 30000 && !app.GetActiveScene(); ++frame)
    ASSERT_TRUE(app.Loop());
  const auto scene = app.GetActiveScene();
  ASSERT_TRUE(scene);
  auto camera = scene->main_camera.Get<Camera>();
  if (!camera) {
    camera = scene->GetOrSetPrivateComponent<Camera>(scene->CreateEntity("Capture anchor")).lock();
    scene->main_camera = camera;
  }
  camera->camera_render_mode = Camera::CameraRenderMode::Rasterization;
  camera->SetRequireRendering(true);
  camera->Resize(sponza_resources ? glm::uvec2(2560, 1440) : glm::uvec2(128));
  auto lighting = scene->environmental_lighting.Get<EnvironmentalLighting>();
  if (!lighting) {
    lighting = AssetManager::CreateTemporaryAsset<EnvironmentalLighting>();
    scene->environmental_lighting = lighting;
  }
  if (!sponza_resources) {
    scene->SetDataComponent(camera->GetOwner(), Transform{});
    const auto entity = scene->CreateEntity("Diffuse capture receiver");
    const auto renderer = scene->GetOrSetPrivateComponent<MeshRenderer>(entity).lock();
    renderer->mesh = Resources::GetInstance().GetPrimitives().cube;
    renderer->material = AssetManager::CreateTemporaryAsset<Material>();
    Transform transform;
    transform.SetValue(glm::vec3(0, 0, -3), glm::vec3(0), glm::vec3(2));
    scene->SetDataComponent(entity, transform);
    scene->SetEntityStatic(entity, true);
    lighting->indirect_environment_source.kind = EnvironmentalLighting::IndirectEnvironmentSourceKind::Color;
    lighting->indirect_environment_source.color = glm::vec3(0.5f, 0.25f, 0.125f);
    lighting->gi_probe_settings.probe_count_x = lighting->gi_probe_settings.probe_count_y = 9;
    lighting->gi_probe_settings.cascade_count = 1;
  }
  lighting->indirect_gi_provider = IndirectGiProvider::AutomaticHddagi;
  lighting->dynamic_reflection_probe_settings.enabled = false;
  const auto render = ApplicationContext::Get().GetLayer<RenderLayer>();
  ASSERT_TRUE(render);
  EXPECT_FALSE(Platform::RayTracingEnabled());
  EXPECT_FALSE(Platform::RayQueryEnabled());
  EXPECT_FALSE(Platform::RayAccelerationStructureEnabled());
  const auto loop_until = [&](const auto& ready) {
    for (uint32_t frame = 0; frame < 360; ++frame) {
      if (!app.Loop())
        return false;
      if (ready())
        return true;
    }
    return false;
  };
  ASSERT_TRUE(loop_until([&] {
    const auto runtime = scene->GetHddagiRuntime();
    return ProjectManager::IsProjectIdle() && runtime && runtime->published && runtime->resources &&
           runtime->resources->transport_ready && runtime->resources->transport_generation >= 32;
  }));
  const auto anchor = scene->GetHddagiRuntime()->frame.anchor;
  auto original_field = scene->GetHddagiRuntime()->resources;
  camera->Resize({65, 33});
  lighting->hddagi_settings.half_resolution = false;
  lighting->hddagi_settings.filter_reflections = false;
  ASSERT_TRUE(app.Loop());
  EXPECT_EQ(scene->GetHddagiRuntime()->resources, original_field);
  EXPECT_EQ(original_field->camera_images.at(camera->GetHandle().GetValue())->layout.gi, glm::uvec2(65, 33));
  lighting->hddagi_settings.half_resolution = true;
  lighting->hddagi_settings.filter_reflections = true;
  const auto second = scene->GetOrSetPrivateComponent<Camera>(scene->CreateEntity("Second HDDAGI camera")).lock();
  second->camera_render_mode = Camera::CameraRenderMode::Rasterization;
  second->SetRequireRendering(true);
  second->Resize({1, 1});
  ASSERT_TRUE(app.Loop());
  EXPECT_EQ(original_field->camera_images.at(camera->GetHandle().GetValue())->layout.gi, glm::uvec2(32, 16));
  EXPECT_EQ(original_field->camera_images.at(second->GetHandle().GetValue())->layout.gi, glm::uvec2(1));
  EXPECT_EQ(scene->GetHddagiRuntime()->frame.anchor.camera_id, anchor.camera_id);
  camera->Resize({128, 128});
  const auto original_probe_settings = lighting->gi_probe_settings;
  const auto original_pack = lighting->reflection_probe_pack;
  auto pack = AssetManager::CreateTemporaryAsset<ReflectionProbePack>();
  lighting->reflection_probe_pack = pack;
  pack->probes.emplace_back();
  auto& probe = pack->probes.front();
  probe.stable_id = 123;
  probe.transform = glm::translate(glm::mat4(1), anchor.world_position);
  probe.box_projection_extents = glm::vec3(30);
  const auto payload = probe.GetOrCreatePayload();
  const auto bake = [&] {
    if (render->QueueGlobalReflectionProbeBakeBatch(scene, {{anchor.world_position, payload, pack, probe.stable_id}}) !=
        1)
      return false;
    return loop_until([&] {
             return !render->HasPendingGlobalReflectionProbeBake();
           }) &&
           payload->IsRuntimeReady();
  };
  lighting->hddagi_settings.energy = 0;
  ASSERT_TRUE(app.Loop());
  ASSERT_TRUE(bake());
  std::vector<uint16_t> dark, lit;
  ASSERT_TRUE(payload->ReadCanonicalPayload(dark));
  lighting->hddagi_settings.energy = 2;
  ASSERT_TRUE(app.Loop());
  ASSERT_TRUE(bake());
  ASSERT_TRUE(payload->ReadCanonicalPayload(lit));
  ASSERT_EQ(dark.size(), lit.size());
  EXPECT_NE(dark, lit);
  for (const uint16_t value : lit)
    ASSERT_TRUE(std::isfinite(glm::unpackHalf1x16(value)));
  const auto baked_hash = GlobalReflectionProbe::CalculatePayloadHash(lit);
  const auto immediate_id = SdfgiTestAccess::CaptureImmediately(*render, scene, anchor.world_position);
  auto immediate_snapshot = SdfgiTestAccess::LatestHddagiCapture(*render);
  ASSERT_TRUE(immediate_snapshot);
  EXPECT_EQ(immediate_snapshot->camera_images.count(immediate_id), 1);

  lighting->dynamic_reflection_probe_settings.faces_per_frame = 1;
  lighting->dynamic_reflection_probe_settings.enabled = true;
  ASSERT_TRUE(loop_until([&] {
    return render->GetDynamicReflectionProbeStats().published_generation_count >= 1;
  }));
  ASSERT_TRUE(loop_until([&] {
    return render->GetDynamicReflectionProbeStats().completed_face_count == 1;
  }));
  original_field.reset();
  immediate_snapshot.reset();
  std::weak_ptr<HddagiResources> retired = scene->GetHddagiRuntime()->resources;
  lighting->gi_probe_settings.probe_count_x = 11;
  lighting->gi_probe_settings.probe_count_y = 11;
  const auto generation = render->GetDynamicReflectionProbeStats().published_generation_count;
  ASSERT_TRUE(app.Loop());
  EXPECT_FALSE(retired.expired());
  ASSERT_TRUE(loop_until([&] {
    return scene->GetHddagiRuntime()->published &&
           render->GetDynamicReflectionProbeStats().published_generation_count > generation;
  }));
  EXPECT_TRUE(retired.expired());
  EXPECT_EQ(scene->GetHddagiRuntime()->resources->probes.probe_count_x, 11);
  EXPECT_EQ(scene->GetHddagiRuntime()->frame.anchor.camera_id, anchor.camera_id);
  EXPECT_EQ(scene->GetHddagiRuntime()->frame.anchor.world_position, anchor.world_position);
  ASSERT_TRUE(payload->ReadCanonicalPayload(lit));
  EXPECT_EQ(GlobalReflectionProbe::CalculatePayloadHash(lit), baked_hash);
  ASSERT_TRUE(loop_until([&] {
    return render->GetDynamicReflectionProbeStats().completed_face_count == 1;
  }));
  const auto before_switch = render->GetDynamicReflectionProbeStats().published_generation_count;
  lighting->indirect_gi_provider = IndirectGiProvider::Environment;
  ASSERT_TRUE(app.Loop());
  EXPECT_FALSE(scene->GetHddagiRuntime());
  ASSERT_TRUE(loop_until([&] {
    return render->GetDynamicReflectionProbeStats().published_generation_count > before_switch;
  }));
  lighting->indirect_gi_provider = IndirectGiProvider::AutomaticHddagi;
  ASSERT_TRUE(loop_until([&] {
    return scene->GetHddagiRuntime() && scene->GetHddagiRuntime()->published;
  }));
}

TEST(HddagiCamera, CompactVisualFixturesAndReceiverOnlyMovementWithoutRt) {
  TempProject project;
  Application app;
  struct Terminate {
    Application& app;
    ~Terminate() {
      app.Terminate();
    }
  } terminate{app};
  app.PushLayer<RenderLayer>("Render Layer");
  auto initialization = TestApplicationSettings(project);
  initialization.load_default_resources = true;
  initialization.load_project_assets = true;
  initialization.load_project_start_scene = true;
  initialization.graphics_settings.use_ray_tracing = false;
  app.Initialize(initialization);
  app.Start();
  for (uint32_t i = 0; i < 30000 && !app.GetActiveScene(); ++i)
    ASSERT_TRUE(app.Loop());
  const auto scene = app.GetActiveScene();
  ASSERT_TRUE(scene);
  const auto camera = scene->GetOrSetPrivateComponent<Camera>(scene->CreateEntity("Fixture camera")).lock();
  scene->main_camera = camera;
  camera->camera_render_mode = Camera::CameraRenderMode::Rasterization;
  camera->SetRequireRendering(true);
  camera->Resize({960, 540});
  Transform camera_transform;
  camera_transform.SetPosition({0, 1.5f, 8});
  scene->SetDataComponent(camera->GetOwner(), camera_transform);
  const auto lighting = AssetManager::CreateTemporaryAsset<EnvironmentalLighting>();
  scene->environmental_lighting = lighting;
  lighting->indirect_gi_provider = IndirectGiProvider::AutomaticHddagi;
  lighting->gi_probe_settings.probe_count_x = lighting->gi_probe_settings.probe_count_y = 17;
  lighting->gi_probe_settings.cascade_count = 2;
  lighting->gi_probe_settings.base_probe_distance = 1;
  lighting->hddagi_settings.static_entities_only = true;
  lighting->indirect_environment_source.kind = EnvironmentalLighting::IndirectEnvironmentSourceKind::Color;
  lighting->indirect_environment_source.color = glm::vec3(0.15f);
  const auto material = [](glm::vec3 color, float roughness, float metallic) {
    auto result = AssetManager::CreateTemporaryAsset<Material>();
    auto& data = result->material_data.shade_material;
    data.pbr_base_color_factor = glm::vec4(color, 1);
    data.pbr_roughness_factor = roughness;
    data.pbr_metallic_factor = metallic;
    return result;
  };
  const auto shape = [&](const char* name, glm::vec3 position, glm::vec3 scale,
                         const std::shared_ptr<Material>& surface, bool sphere = false, bool is_static = true) {
    const auto entity = scene->CreateEntity(name);
    Transform transform;
    transform.SetPosition(position);
    transform.SetScale(scale * 2.0f);
    scene->SetDataComponent(entity, transform);
    scene->SetEntityStatic(entity, is_static);
    const auto renderer = scene->GetOrSetPrivateComponent<MeshRenderer>(entity).lock();
    renderer->mesh =
        sphere ? Resources::GetInstance().GetPrimitives().sphere : Resources::GetInstance().GetPrimitives().cube;
    renderer->material = surface;
    return entity;
  };
  const auto white = material(glm::vec3(0.65f), 1, 0);
  shape("Floor", {0, -0.125f, 0}, {5, 0.125f, 4}, white);
  shape("Back corner", {0, 1.5f, -4}, {5, 1.5f, 0.125f}, white);
  shape("Thin wall", {-3.2f, 1.5f, -1}, {0.0625f, 1.5f, 2}, white);
  const auto emissive = material({1, 0.05f, 0.02f}, 1, 0);
  emissive->material_data.shade_material.emissive_factor = {4, 0.1f, 0.05f};
  shape("Emissive panel", {-4, 1.5f, -2}, {0.15f, 1, 1}, emissive);
  for (int i = 0; i < 3; ++i)
    shape("Curved roughness receiver", {-2.0f + i * 2, 0.7f, -0.5f}, glm::vec3(0.7f),
          material(glm::vec3(0.8f), i * 0.5f, 1), true);
  const auto checker = AssetManager::CreateTemporaryAsset<Texture2D>();
  std::vector<glm::vec4> texels;
  for (int y = 0; y < 8; ++y)
    for (int x = 0; x < 8; ++x)
      texels.emplace_back(0.1f, 0.8f, 0.1f, (x + y) % 2 ? 1.0f : 0.0f);
  checker->SetRgbaChannelData(texels, {8, 8});
  const auto masked = material(glm::vec3(1), 1, 0);
  masked->material_data.shade_material.alpha_mode = static_cast<int32_t>(GltfAlphaMode::Mask);
  masked->SetTexture(&GltfShadeMaterial::pbr_base_color_texture, checker);
  shape("Masked checker", {3.2f, 1.5f, -2}, {1, 1.5f, 0.05f}, masked);
  const auto receiver =
      shape("Receiver only", {1, 0.6f, 2}, glm::vec3(0.6f), material({0.1f, 0.2f, 0.9f}, 0.8f, 0), true, false);
  for (uint32_t frame = 0; frame < 96; ++frame)
    ASSERT_TRUE(app.Loop());
  const auto runtime = scene->GetHddagiRuntime();
  ASSERT_TRUE(runtime && runtime->published);
  EXPECT_FALSE(Platform::RayAccelerationStructureEnabled());
  const auto update_count = runtime->update_count;
  for (uint32_t phase = 0; phase < 3; ++phase) {
    if (phase == 1) {
      auto transform = scene->GetDataComponent<Transform>(receiver);
      transform.SetPosition({-1, 0.6f, 2});
      scene->SetDataComponent(receiver, transform);
      lighting->hddagi_settings.filter_reflections = true;
    } else if (phase == 2) {
      emissive->material_data.shade_material.emissive_factor = glm::vec3(0);
      emissive->SetUnsaved();
      lighting->hddagi_settings.half_resolution = false;
    }
    for (uint32_t frame = 0; frame < 64; ++frame)
      ASSERT_TRUE(app.Loop());
    ASSERT_TRUE(runtime->published);
    EXPECT_EQ(runtime->resources->transport_failure_flags, 0);
    if (phase == 1)
      EXPECT_EQ(runtime->update_count, update_count);
    if (phase == 2)
      EXPECT_GT(runtime->update_count, update_count);
    std::vector<glm::vec4> pixels;
    camera->GetRenderTexture()->GetRgbaChannelData(pixels);
    for (const auto value : pixels)
      ASSERT_TRUE(std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z));
    if (const auto directory = std::getenv("EVOENGINE_HDDAGI_CAPTURE_DIRECTORY")) {
      std::filesystem::create_directories(directory);
      camera->GetRenderTexture()->StoreToPng(std::filesystem::path(directory) /
                                             ("compact-" + std::to_string(phase) + ".png"));
    }
  }
}
