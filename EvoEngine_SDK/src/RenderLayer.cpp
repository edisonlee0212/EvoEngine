#include "RenderLayer.hpp"
#include "Application.hpp"
#include "AssetManager.hpp"
#include "ComputePipeline.hpp"
#include "DdgiProbeRayData.hpp"
#include "EditorLayer.hpp"
#include "EnvironmentalLighting.hpp"
#include "EnvironmentalLightingResolver.hpp"
#include "EnvironmentalMap.hpp"
#include "GeometryStorage.hpp"
#include "GlobalReflectionProbe.hpp"
#include "GltfSceneFeatures.hpp"
#include "GpuService.hpp"
#include "GraphicsPipeline.hpp"
#include "GraphicsResources.hpp"
#include "Jobs.hpp"
#include "LodGroup.hpp"
#include "MeshRenderer.hpp"
#include "Particles.hpp"
#include "Platform.hpp"
#include "PointCloudSample.hpp"
#include "PostProcessingStack.hpp"
#include "Profiler.hpp"
#include "ProjectManager.hpp"
#include "RenderGraph.hpp"
#include "RenderPasses/DdgiAtlasPreparePass.hpp"
#include "RenderPasses/DdgiGatherTimingPass.hpp"
#include "RenderPasses/DdgiPassUtilities.hpp"
#include "RenderPasses/DdgiProbeClassificationPass.hpp"
#include "RenderPasses/DdgiProbeRayVisualizationPass.hpp"
#include "RenderPasses/DdgiProbeRelocationPass.hpp"
#include "RenderPasses/DdgiProbeScrollPass.hpp"
#include "RenderPasses/DdgiProbeUpdatePass.hpp"
#include "RenderPasses/DdgiProbeVariabilityPass.hpp"
#include "RenderPasses/DdgiProbeVisualizationPass.hpp"
#include "RenderPasses/DdgiRayDiagnosticsPass.hpp"
#include "RenderPasses/DeferredGeometryPass.hpp"
#include "RenderPasses/DeferredLightingPass.hpp"
#include "RenderPasses/DepthPyramidPass.hpp"
#include "RenderPasses/DirectionalLightShadowPass.hpp"
#include "RenderPasses/GaussianSplatPass.hpp"
#include "RenderPasses/MotionCoveragePass.hpp"
#include "RenderPasses/MotionVectorPass.hpp"
#include "RenderPasses/PostProcessingPass.hpp"
#include "RenderPasses/RayTracingCameraPass.hpp"
#include "RenderPasses/RenderPassUtilities.hpp"
#include "RenderPasses/TransparentGeometryPass.hpp"
#include "RenderPasses/VolumetricCloudsPass.hpp"
#include "Resources.hpp"
#include "Serialization.hpp"
#include "Shader.hpp"
#include "SkinnedMeshRenderer.hpp"
#include "StrandsRenderer.hpp"
#include "TextureStorage.hpp"
#include "Times.hpp"
#include "Utilities.hpp"
#include "WindowLayer.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <functional>
#include <initializer_list>
#include <limits>
#include <numeric>
#include <sstream>
#include <unordered_map>
#include <unordered_set>
#include <vector>
using namespace evo_engine;

namespace {
using DdgiPerformanceClock = std::chrono::steady_clock;
constexpr int kDdgiProbeScrollClearXBit = 1 << 0;
constexpr int kDdgiProbeScrollClearYBit = 1 << 1;
constexpr int kDdgiProbeScrollClearZBit = 1 << 2;
constexpr int kDdgiProbeScrollPositiveXBit = 1 << 3;
constexpr int kDdgiProbeScrollPositiveYBit = 1 << 4;
constexpr int kDdgiProbeScrollPositiveZBit = 1 << 5;
constexpr uint32_t kDdgiSceneInputSettleFrameCount = 1;
constexpr uint32_t kDdgiLightingIrradianceBinding = 17;
constexpr uint32_t kDdgiLightingVisibilityBinding = 18;
constexpr uint32_t kDdgiLightingProbeStateBinding = 19;
constexpr uint32_t kRasterLightingBrdfLutBinding = 0;
constexpr uint32_t kRasterLightingSkyboxBinding = 1;
constexpr uint32_t kRasterLightingIrradianceBinding = 2;
constexpr uint32_t kRasterLightingPrefilteredBinding = 3;
constexpr uint32_t kRasterLightingAmbientOcclusionBinding = 4;
constexpr uint32_t kRasterLightingReflectionProbesBinding = 5;
constexpr uint32_t kRasterLightingDescriptorSamplerCount = 37;
constexpr uint32_t kRasterLightingMaxPerStageSamplerCount = 64;
constexpr uint32_t kStandaloneReflectionProbeBakeMaxRetryFrames = 600;

std::shared_ptr<GlobalReflectionProbe> GetAssignedGlobalReflectionProbe(const std::shared_ptr<Scene>& scene) {
  if (!scene) {
    return {};
  }
  return scene->GetGlobalReflectionProbeFallback(false);
}

std::shared_ptr<EnvironmentalLighting> GetAssignedEnvironmentalLighting(const std::shared_ptr<Scene>& scene) {
  if (!scene) {
    return {};
  }
  auto lighting_ref = scene->environmental_lighting;
  return lighting_ref.Get<EnvironmentalLighting>();
}

bool IsFiniteReflectionProbeBakePosition(const glm::vec3& position) {
  return std::isfinite(position.x) && std::isfinite(position.y) && std::isfinite(position.z);
}

bool ValidateGlobalReflectionProbeBakeRequest(const std::shared_ptr<Scene>& scene, const glm::vec3& position,
                                              const std::shared_ptr<GlobalReflectionProbe>& target,
                                              std::string& error) {
  if (!scene) {
    error = "Assign the EnvironmentalLighting asset to a scene before baking.";
    return false;
  }
  if (!IsFiniteReflectionProbeBakePosition(position)) {
    error = "The reflection probe capture position is not finite.";
    return false;
  }
  if (!target) {
    error = "Assign a persistent GlobalReflectionProbe asset before baking.";
    return false;
  }
  if (const auto global = GetAssignedGlobalReflectionProbe(scene);
      global && (global == target || global->GetHandle() == target->GetHandle())) {
    error = "The scene's global reflection probe cannot also be a local bake output.";
    return false;
  }
  if (target->IsTemporary()) {
    error = "Assign a persistent GlobalReflectionProbe asset before baking.";
    return false;
  }
  return true;
}

std::shared_ptr<EnvironmentalMap> ResolveIndirectEnvironmentMap(
    const ResolvedEnvironmentalLighting::IndirectEnvironmentSource& source) {
  if (source.kind == ResolvedEnvironmentalLighting::IndirectEnvironmentSourceKind::EngineDefault) {
    return Resources::GetInstance().GetDefaultEnvironmentalMap();
  }
  if (source.kind != ResolvedEnvironmentalLighting::IndirectEnvironmentSourceKind::EnvironmentalMap) {
    return {};
  }
  auto map_ref = source.environmental_map;
  return map_ref.Get<EnvironmentalMap>();
}

std::vector<VkFormat> CreateDeferredGBufferColorAttachmentFormats() {
  return {Platform::Constants::g_buffer_attribute, Platform::Constants::g_buffer_attribute,
          Platform::Constants::g_buffer_attribute, Platform::Constants::g_buffer_attribute,
          Platform::Constants::g_buffer_utility};
}

float DdgiElapsedMilliseconds(const DdgiPerformanceClock::time_point start) {
  return std::chrono::duration<float, std::milli>(DdgiPerformanceClock::now() - start).count();
}

uint32_t HashDdgiProbeRayRotationSeed(uint32_t value) {
  value ^= value >> 16u;
  value *= 0x7feb352du;
  value ^= value >> 15u;
  value *= 0x846ca68bu;
  value ^= value >> 16u;
  return value;
}

float DdgiProbeRayRotationUnitFloat(const uint32_t seed) {
  return static_cast<float>(HashDdgiProbeRayRotationSeed(seed) >> 8u) * (1.0f / 16777216.0f);
}

glm::vec4 CreateDdgiProbeRayRotationQuaternion(const uint32_t frame_index, const uint32_t volume_index,
                                               const uint32_t base_seed) {
  constexpr float two_pi = 6.28318530718f;
  const auto seed = base_seed ^ frame_index ^ (volume_index * 0x9e3779b9u);
  const float u1 = DdgiProbeRayRotationUnitFloat(seed ^ 0x68bc21ebu);
  const float u2 = DdgiProbeRayRotationUnitFloat(seed ^ 0x02e5be93u);
  const float u3 = DdgiProbeRayRotationUnitFloat(seed ^ 0x967a889bu);
  const float r1 = std::sqrt(glm::max(0.0f, 1.0f - u1));
  const float r2 = std::sqrt(glm::max(0.0f, u1));
  return {r1 * std::sin(two_pi * u2), r1 * std::cos(two_pi * u2), r2 * std::sin(two_pi * u3),
          r2 * std::cos(two_pi * u3)};
}

RenderLayer::DdgiProbeVariabilityObservation ReadDdgiProbeVariabilityObservation(
    const std::shared_ptr<Buffer>& buffer) {
  if (!buffer || buffer->GetSize() < sizeof(glm::vec2)) {
    return {};
  }
  glm::vec2 result(0.0f);
  buffer->Download(result);
  if (!std::isfinite(result.x) || !std::isfinite(result.y) || result.x < 0.0f || result.y <= 0.0f) {
    return {};
  }
  return {true, result.x, result.y};
}

void AddExternalRenderResources(RenderGraph& graph, const std::vector<RenderResourceDescriptor>& descriptors) {
  for (const auto& descriptor : descriptors) {
    graph.AddResource(descriptor);
  }
}

void ImportMissingPassResources(RenderGraph& graph, const RenderPassDescriptor& descriptor) {
  for (const auto& resource : descriptor.resources) {
    if (!graph.HasResource(resource.resource_name)) {
      graph.AddResource({resource.resource_name, RenderResourceType::External, RenderResourceLifetime::Imported});
    }
  }
}

class ScopedRenderCameraDrawScope {
 public:
  ScopedRenderCameraDrawScope(const uint32_t frame_index, const std::shared_ptr<Scene>& scene,
                              const std::shared_ptr<Camera>& camera, const bool scene_camera) {
    if (!camera) {
      return;
    }
    uint32_t entity_index = 0;
    if (!scene_camera && scene) {
      if (const auto owner = camera->GetOwner(); scene->IsEntityValid(owner)) {
        entity_index = owner.GetIndex();
      }
    }
    Platform::BeginRenderCameraDrawScope(frame_index, camera->GetHandle().GetValue(), entity_index, scene_camera);
    active_ = true;
  }

  ~ScopedRenderCameraDrawScope() {
    if (active_) {
      Platform::EndRenderCameraDrawScope();
    }
  }

  ScopedRenderCameraDrawScope(const ScopedRenderCameraDrawScope&) = delete;
  ScopedRenderCameraDrawScope& operator=(const ScopedRenderCameraDrawScope&) = delete;

 private:
  bool active_ = false;
};

RenderGraphCompileContext CreateFrameRenderGraphCompileContext() {
  RenderGraphCompileContext context;
  if (Platform::Initialized()) {
    if (const auto& swapchain = Platform::GetSwapchain()) {
      const auto extent = swapchain->GetImageExtent();
      context.frame_width = extent.width;
      context.frame_height = extent.height;
    }
  }
  return context;
}

std::string CreateRasterMaterialShaderDefines() {
  return Platform::GetShaderGlobalDefines() + "\n#define EE_GLTF_RASTER_FIXED_MATERIAL_TEXTURES 1\n";
}

std::string CreateRasterNoBindlessTextureShaderDefines() {
  return Platform::GetShaderGlobalDefines() + "\n#define EE_SKIP_PER_FRAME_BINDLESS_TEXTURES 1\n";
}

std::string CreateRasterMaterialNoBindlessShaderDefines() {
  return CreateRasterNoBindlessTextureShaderDefines() + "#define EE_GLTF_RASTER_FIXED_MATERIAL_TEXTURES 1\n";
}

std::string CreateRasterFixedLightingShaderDefines(const uint32_t lighting_texture_set) {
  return CreateRasterNoBindlessTextureShaderDefines() +
         "#define EE_RASTER_FIXED_LIGHTING_TEXTURES 1\n#define "
         "EE_RASTER_FIXED_LIGHTING_TEXTURE_SET " +
         std::to_string(lighting_texture_set) + "\n";
}

std::string CreateRasterMaterialFixedLightingShaderDefines(const uint32_t lighting_texture_set) {
  return CreateRasterFixedLightingShaderDefines(lighting_texture_set) +
         "#define EE_GLTF_RASTER_FIXED_MATERIAL_TEXTURES 1\n";
}

bool ShouldCreatePerFrameBindlessTextureDescriptors() {
  return Platform::RayTracingEnabled() || Platform::RayQueryEnabled();
}

void PushPerFrameSceneDescriptorBindings(const std::shared_ptr<DescriptorSetLayout>& layout) {
  layout->PushDescriptorBinding(0, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, VK_SHADER_STAGE_ALL, 0);
  layout->PushDescriptorBinding(1, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, VK_SHADER_STAGE_ALL, 0);
  layout->PushDescriptorBinding(2, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_ALL, 0);
  layout->PushDescriptorBinding(4, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_ALL, 0);
  layout->PushDescriptorBinding(5, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, VK_SHADER_STAGE_ALL, 0);
  layout->PushDescriptorBinding(6, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_ALL, 0);
  layout->PushDescriptorBinding(7, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_ALL, 0);
  layout->PushDescriptorBinding(8, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_ALL, 0);
}

void PushPerFrameBindlessTextureDescriptorBindings(
    const std::shared_ptr<DescriptorSetLayout>& layout,
    const ApplicationInitializationSettings& application_initialization_settings) {
  auto texture_2d_stages = VK_SHADER_STAGE_FRAGMENT_BIT | VK_SHADER_STAGE_COMPUTE_BIT;
  auto cubemap_stages = texture_2d_stages;
  if (Platform::RayTracingEnabled()) {
    texture_2d_stages |=
        VK_SHADER_STAGE_RAYGEN_BIT_KHR | VK_SHADER_STAGE_CLOSEST_HIT_BIT_KHR | VK_SHADER_STAGE_ANY_HIT_BIT_KHR;
    cubemap_stages |= texture_2d_stages | VK_SHADER_STAGE_MISS_BIT_KHR;
  }
  layout->PushDescriptorBinding(9, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, texture_2d_stages,
                                VK_DESCRIPTOR_BINDING_PARTIALLY_BOUND_BIT,
                                application_initialization_settings.graphics_settings.max_texture_2d_resource_size);
  layout->PushDescriptorBinding(10, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, cubemap_stages,
                                VK_DESCRIPTOR_BINDING_PARTIALLY_BOUND_BIT,
                                application_initialization_settings.graphics_settings.max_cubemap_resource_size);
}

void PushPerFrameMaterialBufferDescriptorBindings(const std::shared_ptr<DescriptorSetLayout>& layout) {
  layout->PushDescriptorBinding(11, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_ALL, 0);
  layout->PushDescriptorBinding(12, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_ALL, 0);
}

std::shared_ptr<GraphicsPipeline> CreateShadowVertexPipeline(
    const std::filesystem::path& vertex_shader_path, const std::filesystem::path& fragment_shader_path,
    const GeometryType geometry_type,
    const std::initializer_list<std::shared_ptr<DescriptorSetLayout>>& descriptor_set_layouts) {
  auto pipeline = std::make_shared<GraphicsPipeline>();
  pipeline->vertex_shader =
      Shader::CreateTemporary(ShaderType::Vertex, Platform::GetShaderGlobalDefines(), vertex_shader_path);
  pipeline->fragment_shader =
      Shader::CreateTemporary(ShaderType::Fragment, Platform::GetShaderGlobalDefines(), fragment_shader_path);
  pipeline->geometry_type = geometry_type;
  pipeline->vertex_input_attribute_set = VertexInputAttributeSet::Position;
  for (const auto& descriptor_set_layout : descriptor_set_layouts) {
    pipeline->descriptor_set_layouts.emplace_back(descriptor_set_layout);
  }
  pipeline->depth_attachment_format = Platform::Constants::shadow_map;
  pipeline->stencil_attachment_format = VK_FORMAT_UNDEFINED;
  auto& push_constant_range = pipeline->push_constant_ranges.emplace_back();
  push_constant_range.size = sizeof(RenderInstancePushConstant);
  push_constant_range.offset = 0;
  push_constant_range.stageFlags = VK_SHADER_STAGE_ALL;
  pipeline->Initialize();
  return pipeline;
}

std::shared_ptr<GraphicsPipeline> CreateStrandShadowMeshPipeline(
    const std::filesystem::path& mesh_shader_path, const std::shared_ptr<DescriptorSetLayout>& per_frame_layout,
    const std::shared_ptr<DescriptorSetLayout>& strand_meshlet_layout) {
  auto pipeline = std::make_shared<GraphicsPipeline>();
  pipeline->task_shader = Shader::CreateTemporary(
      ShaderType::Task, Platform::GetShaderGlobalDefines(),
      Resources::GetDefaultResourcesPath() / "Shaders/Graphics/Task/Lighting/StrandsShadowMap.slang");
  pipeline->mesh_shader =
      Shader::CreateTemporary(ShaderType::Mesh, Platform::GetShaderGlobalDefines(), mesh_shader_path);
  pipeline->fragment_shader =
      Shader::CreateTemporary(ShaderType::Fragment, Platform::GetShaderGlobalDefines(),
                              Resources::GetDefaultResourcesPath() / "Shaders/Graphics/Fragment/Empty.slang");
  pipeline->descriptor_set_layouts = {per_frame_layout, strand_meshlet_layout};
  pipeline->depth_attachment_format = Platform::Constants::shadow_map;
  pipeline->stencil_attachment_format = VK_FORMAT_UNDEFINED;
  auto& push_constant_range = pipeline->push_constant_ranges.emplace_back();
  push_constant_range.size = sizeof(RenderInstancePushConstant);
  push_constant_range.offset = 0;
  push_constant_range.stageFlags = VK_SHADER_STAGE_ALL;
  pipeline->Initialize();
  return pipeline;
}

std::shared_ptr<GraphicsPipeline> CreateGaussianSplatPipeline(
    const std::shared_ptr<DescriptorSetLayout>& per_frame_layout,
    const std::shared_ptr<DescriptorSetLayout>& gaussian_splat_layout, const VkFormat depth_attachment_format,
    const bool use_mesh_shader = false) {
  auto pipeline = std::make_shared<GraphicsPipeline>();
  if (use_mesh_shader) {
    pipeline->mesh_shader = Shader::CreateTemporary(
        ShaderType::Mesh, Platform::GetShaderGlobalDefines(),
        Resources::GetDefaultResourcesPath() / "Shaders/Graphics/Mesh/GaussianSplat/GaussianSplat.slang");
  } else {
    pipeline->vertex_shader = Shader::CreateTemporary(
        ShaderType::Vertex, Platform::GetShaderGlobalDefines(),
        Resources::GetDefaultResourcesPath() / "Shaders/Graphics/Vertex/GaussianSplat/GaussianSplat.slang");
  }
  pipeline->fragment_shader = Shader::CreateTemporary(
      ShaderType::Fragment, Platform::GetShaderGlobalDefines(),
      Resources::GetDefaultResourcesPath() / "Shaders/Graphics/Fragment/GaussianSplat/GaussianSplat.slang");
  pipeline->geometry_type = GeometryType::Mesh;
  pipeline->vertex_input_enabled = false;
  pipeline->primitive_topology = VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST;
  pipeline->depth_attachment_format = depth_attachment_format;
  pipeline->stencil_attachment_format = VK_FORMAT_UNDEFINED;
  pipeline->color_attachment_formats = {1, Platform::Constants::render_texture_color};
  pipeline->descriptor_set_layouts.emplace_back(per_frame_layout);
  pipeline->descriptor_set_layouts.emplace_back(gaussian_splat_layout);
  auto& push_constant_range = pipeline->push_constant_ranges.emplace_back();
  push_constant_range.size = sizeof(GaussianSplatPushConstant);
  push_constant_range.offset = 0;
  push_constant_range.stageFlags =
      (use_mesh_shader ? VK_SHADER_STAGE_MESH_BIT_EXT : VK_SHADER_STAGE_VERTEX_BIT) | VK_SHADER_STAGE_FRAGMENT_BIT;
  pipeline->Initialize();
  return pipeline;
}

bool LightCastsShadow(const glm::vec4& diffuse) {
  return diffuse.w > 0.5f;
}

bool IsFiniteBound(const Bound& bound) {
  return std::isfinite(bound.min.x) && std::isfinite(bound.min.y) && std::isfinite(bound.min.z) &&
         std::isfinite(bound.max.x) && std::isfinite(bound.max.y) && std::isfinite(bound.max.z) &&
         bound.min.x <= bound.max.x && bound.min.y <= bound.max.y && bound.min.z <= bound.max.z;
}

bool BoundIntersectsClipSpace(const Bound& bound, const glm::mat4& matrix) {
  if (!IsFiniteBound(bound)) {
    return true;
  }
  std::vector<glm::vec3> corners;
  bound.PopulateCorners(corners);
  uint32_t outside_left = 0;
  uint32_t outside_right = 0;
  uint32_t outside_bottom = 0;
  uint32_t outside_top = 0;
  uint32_t outside_near = 0;
  uint32_t outside_far = 0;
  for (const auto& corner : corners) {
    const auto clip = matrix * glm::vec4(corner, 1.0f);
    if (!std::isfinite(clip.x) || !std::isfinite(clip.y) || !std::isfinite(clip.z) || !std::isfinite(clip.w)) {
      return true;
    }
    outside_left += clip.x < -clip.w ? 1 : 0;
    outside_right += clip.x > clip.w ? 1 : 0;
    outside_bottom += clip.y < -clip.w ? 1 : 0;
    outside_top += clip.y > clip.w ? 1 : 0;
    outside_near += clip.z < -clip.w ? 1 : 0;
    outside_far += clip.z > clip.w ? 1 : 0;
  }
  return outside_left != corners.size() && outside_right != corners.size() && outside_bottom != corners.size() &&
         outside_top != corners.size() && outside_near != corners.size() && outside_far != corners.size();
}

bool ShouldRenderShadowInstance(const std::shared_ptr<RenderInstanceStorage::IRenderInstance>& render_instance,
                                const glm::mat4& light_space_matrix) {
  return render_instance->cast_shadow && BoundIntersectsClipSpace(render_instance->world_bound, light_space_matrix);
}

bool HasVisibleShadowInstance(const std::shared_ptr<RenderInstanceStorage::IRenderInstanceCollection>& collection,
                              const glm::mat4& light_space_matrix) {
  bool has_visible_instance = false;
  collection->ForEachRenderInstance([&](const auto& render_instance) {
    if (!has_visible_instance && ShouldRenderShadowInstance(render_instance, light_space_matrix)) {
      has_visible_instance = true;
    }
  });
  return has_visible_instance;
}

RenderGraphCompileContext CreateCameraRenderGraphCompileContext(const std::shared_ptr<Camera>& camera) {
  auto context = CreateFrameRenderGraphCompileContext();
  if (camera) {
    const auto size = camera->GetSize();
    context.camera_width = size.x;
    context.camera_height = size.y;
  }
  return context;
}

RenderGraphResourceRegistry CreateFrameRenderGraphResourceRegistry(
    const std::shared_ptr<DescriptorSet>& per_frame_descriptor_set) {
  RenderGraphResourceRegistry registry;
  registry.BindDescriptorSet(RenderResourceNames::frame_per_frame_descriptor_set, per_frame_descriptor_set);
  return registry;
}

RenderGraphResourceRegistry CreateCameraRenderGraphResourceRegistry(
    const std::shared_ptr<DescriptorSet>& per_frame_descriptor_set,
    const std::shared_ptr<DescriptorSet>& ray_tracing_descriptor_set, const std::shared_ptr<Camera>& camera) {
  auto registry = CreateFrameRenderGraphResourceRegistry(per_frame_descriptor_set);
  if (ray_tracing_descriptor_set) {
    registry.BindDescriptorSet(RenderResourceNames::frame_ray_tracing_descriptor_set, ray_tracing_descriptor_set);
  }
  if (camera) {
    registry.BindDescriptorSet(RenderResourceNames::camera_g_buffer, camera->GetGBufferDescriptorSet());
    if (const auto& render_texture = camera->GetRenderTexture()) {
      registry.BindRenderTexture(RenderResourceNames::camera_color, render_texture);
      registry.BindImage(RenderResourceNames::camera_color, render_texture->GetColorImage());
      registry.BindImage(RenderResourceNames::camera_depth, render_texture->GetDepthImage());
    }
  }
  return registry;
}

bool ShouldUseDdgiFrameResources(const RenderLayer::DdgiSettings& settings) {
  return RenderLayer::ResolveDdgiRuntimePolicy(settings).use_resources;
}

bool ShouldTraceDdgiProbeRays(const RenderLayer::DdgiSettings& settings) {
  return RenderLayer::ResolveDdgiRuntimePolicy(settings).trace_probe_rays;
}

bool ShouldRenderDdgiProbeVisualization(const RenderLayer::DdgiSettings& settings) {
  return settings.debug.enabled && settings.debug.visualize_probe_illumination &&
         settings.debug.visualize_probe_positions;
}

uint32_t GetDdgiProbeVisualizationMode(const RenderLayer::DdgiSettings& settings) {
  return static_cast<uint32_t>(glm::clamp(settings.debug.probe_visualization_mode, 0, 3));
}

glm::ivec3 ClampDdgiProbeCounts(const glm::ivec3& value) {
  return {glm::clamp(value.x, 1, 256), glm::clamp(value.y, 1, 256), glm::clamp(value.z, 1, 256)};
}

glm::ivec3 WrapDdgiProbeGrid(const glm::ivec3& probe_grid, const glm::ivec3& probe_counts) {
  const auto safe_counts = glm::max(probe_counts, glm::ivec3(1));
  return (probe_grid % safe_counts + safe_counts) % safe_counts;
}

uint32_t GetDdgiProbeIndexFromGrid(const glm::ivec3& probe_grid, const glm::ivec3& probe_counts) {
  const auto safe_counts = glm::max(probe_counts, glm::ivec3(1));
  const auto wrapped_grid = WrapDdgiProbeGrid(probe_grid, safe_counts);
  return static_cast<uint32_t>(wrapped_grid.x + wrapped_grid.y * safe_counts.x +
                               wrapped_grid.z * safe_counts.x * safe_counts.y);
}

uint32_t GetScrolledDdgiProbeIndex(const glm::uvec3& logical_probe_grid, const glm::ivec3& probe_scroll_offset,
                                   const glm::ivec3& probe_counts) {
  return GetDdgiProbeIndexFromGrid(glm::ivec3(logical_probe_grid) + probe_scroll_offset, probe_counts);
}

glm::vec3 ClampDdgiProbeSpacing(const glm::vec3& value) {
  return glm::clamp(value, glm::vec3(0.05f), glm::vec3(10000.0f));
}

RenderResourceDescriptor CreateDdgiImageResourceDescriptor(const std::string& name,
                                                           const RenderLayer::DdgiAtlasLayout& layout,
                                                           const std::string& format_name) {
  return {name,
          RenderResourceType::Image,
          RenderResourceLifetime::Frame,
          {RenderResourceSizeMode::Absolute, layout.resolution.x, layout.resolution.y, 1, 1, 1},
          format_name,
          1,
          1,
          true};
}

RenderResourceDescriptor CreateDdgiImageResourceDescriptor(const std::string& name, const glm::uvec2 extent,
                                                           const std::string& format_name) {
  return {name,
          RenderResourceType::Image,
          RenderResourceLifetime::Frame,
          {RenderResourceSizeMode::Absolute, glm::max(extent.x, 1u), glm::max(extent.y, 1u), 1, 1, 1},
          format_name,
          1,
          1,
          true};
}

RenderResourceDescriptor CreateDdgiImportedImageResourceDescriptor(const std::string& name,
                                                                   const RenderLayer::DdgiAtlasLayout& layout,
                                                                   const std::string& format_name) {
  auto descriptor = CreateDdgiImageResourceDescriptor(name, layout, format_name);
  descriptor.lifetime = RenderResourceLifetime::Persistent;
  descriptor.managed_by_graph = false;
  return descriptor;
}

std::shared_ptr<Image> CreateDdgiAtlasImage(const RenderLayer::DdgiAtlasLayout& layout, const VkFormat format) {
  if (!Platform::Initialized() || layout.resolution.x == 0 || layout.resolution.y == 0) {
    return {};
  }
  const uint32_t queue_family_indices[2] = {Platform::GetGraphicsAndComputeQueueFamilyIndex(),
                                            Platform::GetComputeQueueFamilyIndex()};
  VkImageCreateInfo image_info{};
  image_info.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
  image_info.imageType = VK_IMAGE_TYPE_2D;
  image_info.extent = {layout.resolution.x, layout.resolution.y, 1};
  image_info.mipLevels = 1;
  image_info.arrayLayers = 1;
  image_info.format = format;
  image_info.tiling = VK_IMAGE_TILING_OPTIMAL;
  image_info.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
  image_info.usage = VK_IMAGE_USAGE_TRANSFER_DST_BIT | VK_IMAGE_USAGE_SAMPLED_BIT | VK_IMAGE_USAGE_STORAGE_BIT;
  image_info.samples = VK_SAMPLE_COUNT_1_BIT;
  if (queue_family_indices[0] != queue_family_indices[1]) {
    image_info.sharingMode = VK_SHARING_MODE_CONCURRENT;
    image_info.queueFamilyIndexCount = 2;
    image_info.pQueueFamilyIndices = queue_family_indices;
  } else {
    image_info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
  }
  return std::make_shared<Image>(image_info);
}

std::shared_ptr<Sampler> CreateDdgiAtlasSampler() {
  if (!Platform::Initialized()) {
    return {};
  }
  VkSamplerCreateInfo sampler_info{};
  sampler_info.sType = VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO;
  sampler_info.magFilter = VK_FILTER_LINEAR;
  sampler_info.minFilter = VK_FILTER_LINEAR;
  sampler_info.addressModeU = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
  sampler_info.addressModeV = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
  sampler_info.addressModeW = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
  sampler_info.borderColor = VK_BORDER_COLOR_FLOAT_OPAQUE_BLACK;
  sampler_info.unnormalizedCoordinates = VK_FALSE;
  sampler_info.compareEnable = VK_FALSE;
  sampler_info.compareOp = VK_COMPARE_OP_ALWAYS;
  sampler_info.mipmapMode = VK_SAMPLER_MIPMAP_MODE_NEAREST;
  sampler_info.minLod = 0.0f;
  sampler_info.maxLod = 0.0f;
  return std::make_shared<Sampler>(sampler_info);
}

bool HasDdgiAtlasImageLayout(const std::shared_ptr<Image>& image, const RenderLayer::DdgiAtlasLayout& layout,
                             const VkFormat format) {
  if (!image || image->GetFormat() != format) {
    return false;
  }
  const auto extent = image->GetExtent();
  return extent.width == layout.resolution.x && extent.height == layout.resolution.y && extent.depth == 1;
}

struct DdgiProbeRayDiagnosticSource {
  glm::ivec3 probe_counts = {1, 1, 1};
  glm::vec3 first_probe = glm::vec3(0.0f);
  glm::vec3 probe_step_x = glm::vec3(0.0f);
  glm::vec3 probe_step_y = glm::vec3(0.0f);
  glm::vec3 probe_step_z = glm::vec3(0.0f);
  uint32_t selected_volume_index = 0;
  float relocation_distance = 0.0f;
  float random_ray_backface_threshold = 0.1f;
  float fixed_ray_backface_threshold = 0.25f;
  float probe_variability_threshold = 0.2f;
  int probe_variability_min_samples = 16;
  int auto_invalidate_trigger_conditions = DdgiVolumeTriggerConditionAll;
  int warmup_trigger_conditions = DdgiVolumeTriggerConditionLightEnableChanged;
  int variability_reset_trigger_conditions =
      DdgiVolumeTriggerConditionLightingConditionChanged | DdgiVolumeTriggerConditionGeometryChanged;
  int movement_type = static_cast<int>(DdgiVolumeMovementType::Default);
  glm::ivec3 probe_scroll_offset = glm::ivec3(0);
  glm::ivec3 probe_scroll_clear = glm::ivec3(0);
  glm::ivec3 probe_scroll_directions = glm::ivec3(1);
  glm::ivec3 probe_scroll_delta = glm::ivec3(0);
  bool enable_probe_relocation = false;
  bool enable_probe_classification = false;
  bool enable_probe_variability = true;
  bool enable_probe_variability_gating = true;
  bool emissive_mesh_sampling_enabled = true;
};

glm::vec3 TransformPoint(const glm::mat4& transform, const glm::vec3& point) {
  return glm::vec3(transform * glm::vec4(point, 1.0f));
}

glm::vec3 TransformVector(const glm::mat4& transform, const glm::vec3& vector) {
  return glm::vec3(transform * glm::vec4(vector, 0.0f));
}

float AxisCoordinate(const glm::vec3& delta, const glm::vec3& axis) {
  const auto axis_length_squared = glm::dot(axis, axis);
  return axis_length_squared > 1e-6f ? glm::dot(delta, axis) / axis_length_squared : 0.0f;
}

glm::ivec3 CalculateDdgiProbeScrollDelta(const glm::vec3& first_probe_delta, const glm::vec3& probe_step_x,
                                         const glm::vec3& probe_step_y, const glm::vec3& probe_step_z) {
  const auto abs_floor = [](const float value) {
    return value >= 0.0f ? static_cast<int>(std::floor(value)) : static_cast<int>(std::ceil(value));
  };
  return {abs_floor(AxisCoordinate(first_probe_delta, probe_step_x)),
          abs_floor(AxisCoordinate(first_probe_delta, probe_step_y)),
          abs_floor(AxisCoordinate(first_probe_delta, probe_step_z))};
}

glm::vec3 CalculateDdgiEffectiveFirstProbe(const glm::vec3& base_first_probe, const glm::vec3& probe_step_x,
                                           const glm::vec3& probe_step_y, const glm::vec3& probe_step_z,
                                           const glm::ivec3& probe_scroll_offset) {
  return base_first_probe + probe_step_x * static_cast<float>(probe_scroll_offset.x) +
         probe_step_y * static_cast<float>(probe_scroll_offset.y) +
         probe_step_z * static_cast<float>(probe_scroll_offset.z);
}

void NormalizeDdgiProbeScrollOrigin(glm::vec3& base_first_probe, glm::ivec3& probe_scroll_offset,
                                    const glm::ivec3& probe_counts, const glm::vec3& probe_step_x,
                                    const glm::vec3& probe_step_y, const glm::vec3& probe_step_z) {
  const auto counts = ClampDdgiProbeCounts(probe_counts);
  const glm::vec3 probe_steps[3] = {probe_step_x, probe_step_y, probe_step_z};
  for (int axis = 0; axis < 3; ++axis) {
    const int complete_wraps = probe_scroll_offset[axis] / counts[axis];
    if (complete_wraps != 0) {
      const int normalized_probes = complete_wraps * counts[axis];
      base_first_probe += probe_steps[axis] * static_cast<float>(normalized_probes);
      probe_scroll_offset[axis] -= normalized_probes;
    }
  }
}

int PackDdgiProbeScrollFlags(const glm::ivec3& probe_scroll_clear, const glm::ivec3& probe_scroll_directions) {
  int flags = 0;
  flags |= probe_scroll_clear.x != 0 ? kDdgiProbeScrollClearXBit : 0;
  flags |= probe_scroll_clear.y != 0 ? kDdgiProbeScrollClearYBit : 0;
  flags |= probe_scroll_clear.z != 0 ? kDdgiProbeScrollClearZBit : 0;
  flags |= probe_scroll_directions.x > 0 ? kDdgiProbeScrollPositiveXBit : 0;
  flags |= probe_scroll_directions.y > 0 ? kDdgiProbeScrollPositiveYBit : 0;
  flags |= probe_scroll_directions.z > 0 ? kDdgiProbeScrollPositiveZBit : 0;
  return flags;
}

glm::ivec4 CreateDdgiProbeScrollPushConstant(const DdgiProbeRayDiagnosticSource& source) {
  return {source.probe_scroll_offset,
          PackDdgiProbeScrollFlags(source.probe_scroll_clear, source.probe_scroll_directions)};
}

DdgiProbeRayDiagnosticSource CreateDdgiProbeRayDiagnosticSourceFromResolvedVolume(
    const ResolvedEnvironmentalLighting::DdgiVolume& volume, const RenderLayer::DdgiSettings& settings) {
  DdgiProbeRayDiagnosticSource source;
  source.probe_counts = volume.probe_counts;
  const auto counts = ClampDdgiProbeCounts(volume.probe_counts);
  const auto spacing = ClampDdgiProbeSpacing(volume.probe_spacing);
  const auto first_probe_local = volume.volume_origin - glm::vec3(counts - glm::ivec3(1)) * spacing * 0.5f;
  source.first_probe = TransformPoint(volume.transform, first_probe_local);
  source.probe_step_x = TransformVector(volume.transform, {spacing.x, 0.0f, 0.0f});
  source.probe_step_y = TransformVector(volume.transform, {0.0f, spacing.y, 0.0f});
  source.probe_step_z = TransformVector(volume.transform, {0.0f, 0.0f, spacing.z});
  source.relocation_distance = glm::max(volume.relocation_distance, 0.0f);
  source.random_ray_backface_threshold = glm::clamp(volume.random_ray_backface_threshold, 0.0f, 1.0f);
  source.fixed_ray_backface_threshold = glm::clamp(volume.fixed_ray_backface_threshold, 0.0f, 1.0f);
  source.probe_variability_threshold = glm::clamp(volume.probe_variability_threshold, 0.0f, 10.0f);
  source.probe_variability_min_samples = glm::clamp(volume.probe_variability_min_samples, 0, 4096);
  source.auto_invalidate_trigger_conditions = volume.auto_invalidate_trigger_conditions & DdgiVolumeTriggerConditionAll;
  source.warmup_trigger_conditions = volume.warmup_trigger_conditions & DdgiVolumeTriggerConditionAll;
  source.variability_reset_trigger_conditions =
      volume.variability_reset_trigger_conditions & DdgiVolumeTriggerConditionAll;
  source.movement_type = glm::clamp(volume.movement_type, static_cast<int>(DdgiVolumeMovementType::Default),
                                    static_cast<int>(DdgiVolumeMovementType::Scrolling));
  source.enable_probe_relocation = volume.enable_probe_relocation;
  source.enable_probe_classification = volume.enable_probe_classification;
  source.enable_probe_variability = volume.enable_probe_variability;
  source.enable_probe_variability_gating = volume.enable_probe_variability_gating;
  source.emissive_mesh_sampling_enabled = RenderLayer::ResolveDdgiEmissiveMeshSampling(
      settings.runtime.enable_emissive_mesh_sampling, volume.emissive_mesh_sampling_mode);
  return source;
}

float CalculateDdgiEffectiveMaxRayDistance(const RenderLayer::DdgiSettings& settings) {
  return glm::clamp(settings.runtime.max_ray_distance, 0.05f, kDdgiProbeRayMissDistance);
}

uint32_t GetDdgiEnvironmentCubemapIndex(const std::shared_ptr<Scene>& scene) {
  const auto resolved_lighting = ResolveEnvironmentalLighting(scene);
  const auto environmental_map = ResolveIndirectEnvironmentMap(resolved_lighting.indirect_environment_source);
  if (!environmental_map) {
    return 0u;
  }
  environmental_map->EnsureEnvironmentSource();
  auto cubemap_ref = environmental_map->environment_cubemap;
  const auto cubemap = cubemap_ref.Get<Cubemap>();
  return cubemap ? cubemap->GetTextureStorageIndex() : 0u;
}

DdgiProbeRayTracingPushConstant CreateDdgiProbeRayTracingPushConstant(
    const RenderLayer::DdgiSettings& settings, const DdgiProbeRayDiagnosticSource& source,
    const bool skip_inactive_probe_trace, const bool skip_recursive_ddgi, const uint32_t volume_index,
    const uint32_t environment_cubemap_index, const uint32_t deterministic_sequence_index) {
  DdgiProbeRayTracingPushConstant push_constant;
  const auto frame_index = settings.runtime.deterministic_ray_seed_enabled
                               ? deterministic_sequence_index
                               : static_cast<uint32_t>(Platform::GetFrameCount() & 0x00ffffffu);
  const auto base_seed = settings.runtime.deterministic_ray_seed_enabled ? settings.runtime.deterministic_ray_seed : 0u;
  const auto emissive_dispatch_seed = HashDdgiProbeRayRotationSeed(base_seed ^ (frame_index * 0x9e3779b9u) ^
                                                                   (deterministic_sequence_index * 0x85ebca6bu) ^
                                                                   (source.selected_volume_index * 0xc2b2ae35u));
  const auto ray_rotation = CreateDdgiProbeRayRotationQuaternion(frame_index, source.selected_volume_index, base_seed);
  const auto ray_count = static_cast<uint32_t>(glm::max(settings.runtime.ray_count, 1));
  const auto fixed_ray_count = RenderLayer::GetDdgiFixedRayCount(
      ray_count, source.enable_probe_relocation || source.enable_probe_classification);
  push_constant.first_probe = glm::vec4(source.first_probe, ray_rotation.x);
  push_constant.probe_step_x = glm::vec4(source.probe_step_x, ray_rotation.y);
  push_constant.probe_step_y = glm::vec4(source.probe_step_y, ray_rotation.z);
  push_constant.probe_step_z = glm::vec4(source.probe_step_z, ray_rotation.w);
  push_constant.probe_counts_and_ray_count =
      glm::uvec4(glm::uvec3(ClampDdgiProbeCounts(source.probe_counts)), ray_count);
  const auto ray_flags =
      RenderLayer::GetDdgiProbeRayFlags(skip_inactive_probe_trace, source.emissive_mesh_sampling_enabled);
  push_constant.selected_probe_volume_flags_environment = {(std::numeric_limits<uint32_t>::max)(), volume_index,
                                                           ray_flags, environment_cubemap_index};
  push_constant.trace_parameters = {CalculateDdgiEffectiveMaxRayDistance(settings),
                                    glm::max(settings.runtime.normal_bias, 0.001f), static_cast<float>(fixed_ray_count),
                                    skip_recursive_ddgi ? 1.0f : 0.0f};
  push_constant.probe_scroll_offset = CreateDdgiProbeScrollPushConstant(source);
  push_constant.probe_scroll_offset.w = static_cast<int32_t>(emissive_dispatch_seed & 0x7fffffffu);
  return push_constant;
}

DdgiProbeAtlasUpdatePushConstant CreateDdgiProbeAtlasUpdatePushConstant(
    const RenderLayer::DdgiSettings& settings, const RenderLayer::DdgiFrameResourceLayout& layout,
    const DdgiProbeRayDiagnosticSource& source, const float history_hysteresis, const float brightness_threshold) {
  const auto total_probe_count = layout.probe_count;
  const auto history_weight = glm::clamp(history_hysteresis, 0.0f, 1.0f);
  DdgiProbeAtlasUpdatePushConstant push_constant;
  const auto ray_count = static_cast<uint32_t>(glm::max(settings.runtime.ray_count, 1));
  const auto fixed_ray_count = RenderLayer::GetDdgiFixedRayCount(
      ray_count, source.enable_probe_relocation || source.enable_probe_classification);
  push_constant.probe_count_ray_count_and_tile_sizes = {
      total_probe_count, ray_count, layout.irradiance_atlas.tile_resolution, layout.visibility_atlas.tile_resolution};
  push_constant.atlas_columns_fixed_ray_count_and_update_mode = {layout.irradiance_atlas.columns,
                                                                 layout.visibility_atlas.columns, fixed_ray_count, 0u};
  push_constant.probe_counts_and_rotation = glm::uvec4(glm::uvec3(ClampDdgiProbeCounts(source.probe_counts)), 0u);
  push_constant.update_parameters = {CalculateDdgiEffectiveMaxRayDistance(settings), history_weight,
                                     glm::max(settings.runtime.irradiance_gamma, 1.0f),
                                     glm::max(source.relocation_distance, 0.0f)};
  push_constant.blend_parameters = {
      glm::clamp(source.random_ray_backface_threshold, 0.0f, 1.0f), glm::max(settings.runtime.distance_exponent, 0.0f),
      glm::max(brightness_threshold, 0.0f), glm::clamp(settings.runtime.irradiance_threshold, 0.0f, 1.0f)};
  push_constant.probe_scroll_offset = CreateDdgiProbeScrollPushConstant(source);
  push_constant.probe_scroll_delta = glm::ivec4(source.probe_scroll_delta, 0);
  push_constant.probe_step_x = glm::vec4(source.probe_step_x, 0.0f);
  push_constant.probe_step_y = glm::vec4(source.probe_step_y, 0.0f);
  push_constant.probe_step_z = glm::vec4(source.probe_step_z, 0.0f);
  return push_constant;
}

DdgiProbeScrollPushConstant CreateDdgiProbeScrollClearPushConstant(const RenderLayer::DdgiFrameResourceLayout& layout,
                                                                   const DdgiProbeRayDiagnosticSource& source) {
  DdgiProbeScrollPushConstant push_constant;
  push_constant.probe_counts_and_irradiance_tile_size = {
      static_cast<uint32_t>(source.probe_counts.x), static_cast<uint32_t>(source.probe_counts.y),
      static_cast<uint32_t>(source.probe_counts.z), layout.irradiance_atlas.tile_resolution};
  push_constant.atlas_columns_visibility_tile_and_probe_count = {
      layout.irradiance_atlas.columns, layout.visibility_atlas.columns, layout.visibility_atlas.tile_resolution,
      layout.probe_count};
  push_constant.probe_scroll_offset = glm::ivec4(source.probe_scroll_offset, 0);
  push_constant.probe_scroll_delta = glm::ivec4(source.probe_scroll_delta, 0);
  return push_constant;
}

DdgiProbeRelocationPushConstant CreateDdgiProbeRelocationPushConstant(const RenderLayer::DdgiSettings& settings,
                                                                      const uint32_t total_probe_count,
                                                                      const DdgiProbeRayDiagnosticSource& source,
                                                                      const bool reset_offsets) {
  DdgiProbeRelocationPushConstant push_constant;
  const auto ray_count = static_cast<uint32_t>(glm::max(settings.runtime.ray_count, 1));
  const auto fixed_ray_count = RenderLayer::GetDdgiFixedRayCount(
      ray_count, source.enable_probe_relocation || source.enable_probe_classification);
  push_constant.probe_count_ray_count_and_flags = {total_probe_count, ray_count, 0u, reset_offsets ? 1u : 0u};
  push_constant.probe_counts = glm::uvec4(glm::uvec3(ClampDdgiProbeCounts(source.probe_counts)), fixed_ray_count);
  push_constant.relocation_parameters = {glm::max(source.relocation_distance, 0.0f),
                                         glm::clamp(source.fixed_ray_backface_threshold, 0.0f, 1.0f), 0.0f, 0.0f};
  push_constant.probe_scroll_offset = CreateDdgiProbeScrollPushConstant(source);
  push_constant.probe_step_x = glm::vec4(source.probe_step_x, 0.0f);
  push_constant.probe_step_y = glm::vec4(source.probe_step_y, 0.0f);
  push_constant.probe_step_z = glm::vec4(source.probe_step_z, 0.0f);
  return push_constant;
}

DdgiProbeClassificationPushConstant CreateDdgiProbeClassificationPushConstant(
    const RenderLayer::DdgiSettings& settings, const uint32_t total_probe_count,
    const DdgiProbeRayDiagnosticSource& source, const bool reset_classification) {
  DdgiProbeClassificationPushConstant push_constant;
  const auto ray_count = static_cast<uint32_t>(glm::max(settings.runtime.ray_count, 1));
  const auto fixed_ray_count = RenderLayer::GetDdgiFixedRayCount(
      ray_count, source.enable_probe_relocation || source.enable_probe_classification);
  push_constant.probe_count_ray_count_and_flags = {total_probe_count, ray_count, 0u, reset_classification ? 1u : 0u};
  push_constant.probe_counts = glm::uvec4(glm::uvec3(ClampDdgiProbeCounts(source.probe_counts)), fixed_ray_count);
  push_constant.classification_parameters = {glm::clamp(source.fixed_ray_backface_threshold, 0.0f, 1.0f), 0.0f, 0.0f,
                                             0.0f};
  push_constant.probe_scroll_offset = CreateDdgiProbeScrollPushConstant(source);
  push_constant.probe_step_x = glm::vec4(source.probe_step_x, 0.0f);
  push_constant.probe_step_y = glm::vec4(source.probe_step_y, 0.0f);
  push_constant.probe_step_z = glm::vec4(source.probe_step_z, 0.0f);
  return push_constant;
}

bool BindDdgiLightingDescriptors(const std::shared_ptr<DescriptorSet>& lighting_descriptor_set,
                                 const VkDescriptorImageInfo& irradiance_info,
                                 const VkDescriptorImageInfo& visibility_info,
                                 const std::shared_ptr<Buffer>& probe_state_buffer, const uint32_t volume_slot) {
  if (!lighting_descriptor_set || !IsValidDescriptorImageInfo(irradiance_info) ||
      !IsValidDescriptorImageInfo(visibility_info) || !probe_state_buffer) {
    return false;
  }
  lighting_descriptor_set->UpdateImageDescriptorBinding(kDdgiLightingIrradianceBinding, irradiance_info, volume_slot);
  lighting_descriptor_set->UpdateImageDescriptorBinding(kDdgiLightingVisibilityBinding, visibility_info, volume_slot);
  lighting_descriptor_set->UpdateBufferDescriptorBinding(kDdgiLightingProbeStateBinding, probe_state_buffer,
                                                         volume_slot);
  return true;
}

void BindDdgiFallbackLightingDescriptors(const std::shared_ptr<DescriptorSet>& lighting_descriptor_set,
                                         const std::shared_ptr<Buffer>& fallback_probe_state_buffer) {
  const auto image_info = CreateDdgiFallbackImageInfo();
  for (uint32_t slot = 0; slot < RenderInstanceStorage::kDdgiMaxVolumeCount; ++slot) {
    BindDdgiLightingDescriptors(lighting_descriptor_set, image_info, image_info, fallback_probe_state_buffer, slot);
  }
}

bool BindDdgiAtlasLightingDescriptors(const RenderGraphResourceRegistry& registry,
                                      RenderGraphTransientResourceStore& transient_resources,
                                      const std::shared_ptr<DescriptorSet>& lighting_descriptor_set,
                                      const std::shared_ptr<Sampler>& atlas_sampler,
                                      const std::shared_ptr<Buffer>& fallback_probe_state_buffer,
                                      const uint32_t volume_slot) {
  if (!lighting_descriptor_set || !fallback_probe_state_buffer) {
    return false;
  }
  const auto fallback_info = CreateDdgiFallbackImageInfo();
  if (!IsValidDescriptorImageInfo(fallback_info)) {
    return false;
  }
  const auto* state_binding = registry.GetResourceBinding(RenderResourceNames::frame_ddgi_probe_state);
  const auto* irradiance_binding = registry.GetResourceBinding(RenderResourceNames::frame_ddgi_irradiance_atlas);
  const auto* visibility_binding = registry.GetResourceBinding(RenderResourceNames::frame_ddgi_visibility_atlas);
  if (!irradiance_binding || !irradiance_binding->image || !visibility_binding || !visibility_binding->image) {
    return false;
  }
  const auto irradiance_view = CreateGraphImageMipView(irradiance_binding->image, 0);
  const auto visibility_view = CreateGraphImageMipView(visibility_binding->image, 0);
  if (!irradiance_view || !visibility_view) {
    return false;
  }
  transient_resources.RetainImageView(irradiance_view);
  transient_resources.RetainImageView(visibility_view);

  VkDescriptorImageInfo image_info{};
  image_info.imageLayout = VK_IMAGE_LAYOUT_GENERAL;
  image_info.sampler = atlas_sampler ? atlas_sampler->GetVkSampler() : fallback_info.sampler;
  image_info.imageView = irradiance_view->GetVkImageView();
  auto irradiance_info = image_info;
  image_info.imageView = visibility_view->GetVkImageView();
  auto visibility_info = image_info;
  const auto probe_state_buffer =
      state_binding && state_binding->buffer ? state_binding->buffer : fallback_probe_state_buffer;
  return BindDdgiLightingDescriptors(lighting_descriptor_set, irradiance_info, visibility_info, probe_state_buffer,
                                     volume_slot);
}

void PreserveDdgiRenderInfo(RenderInstanceStorage::RenderInfoBlock& render_info,
                            const RenderInstanceStorage::RenderInfoBlock& previous_render_info) {
  render_info.ddgi_volume_header = previous_render_info.ddgi_volume_header;
  render_info.ddgi_volumes = previous_render_info.ddgi_volumes;
}

uint64_t MakeDdgiLightKey(const uint32_t type_index, const Entity& owner) {
  return (static_cast<uint64_t>(type_index) << 56u) | (static_cast<uint64_t>(owner.GetVersion()) << 32u) |
         static_cast<uint64_t>(owner.GetIndex());
}

uint64_t MixDdgiSignature(const uint64_t seed, const uint64_t value) {
  return seed ^ (value + 0x9e3779b97f4a7c15ull + (seed << 6u) + (seed >> 2u));
}

constexpr uint64_t kReflectionProbeCaptureFingerprintVersion = 2u;

uint64_t MixDdgiFloat(const uint64_t seed, const float value) {
  return MixDdgiSignature(seed, static_cast<uint64_t>(glm::floatBitsToUint(value)));
}

uint64_t MixDdgiVec3(uint64_t seed, const glm::vec3& value) {
  seed = MixDdgiFloat(seed, value.x);
  seed = MixDdgiFloat(seed, value.y);
  return MixDdgiFloat(seed, value.z);
}

uint64_t MixDdgiVec4(uint64_t seed, const glm::vec4& value) {
  seed = MixDdgiVec3(seed, glm::vec3(value));
  return MixDdgiFloat(seed, value.w);
}

uint64_t MixDdgiMat4(uint64_t seed, const glm::mat4& value) {
  for (int column = 0; column < 4; ++column) {
    for (int row = 0; row < 4; ++row) {
      seed = MixDdgiFloat(seed, value[column][row]);
    }
  }
  return seed;
}

uint64_t MixStableText(uint64_t seed, const std::string& value) {
  for (const unsigned char character : value) {
    seed = MixDdgiSignature(seed, character);
  }
  return seed;
}

uint64_t MixStableYaml(uint64_t seed, const YAML::Node& node) {
  seed = MixDdgiSignature(seed, static_cast<uint64_t>(node.Type()));
  if (node.IsScalar()) {
    return MixStableText(seed, node.Scalar());
  }
  if (node.IsSequence()) {
    seed = MixDdgiSignature(seed, node.size());
    for (const auto& value : node) {
      seed = MixStableYaml(seed, value);
    }
    return seed;
  }
  if (!node.IsMap()) {
    return seed;
  }
  std::vector<std::string> keys;
  keys.reserve(node.size());
  for (const auto& value : node) {
    keys.emplace_back(value.first.Scalar());
  }
  std::sort(keys.begin(), keys.end());
  seed = MixDdgiSignature(seed, keys.size());
  for (const auto& key : keys) {
    seed = MixStableText(seed, key);
    if (key == "asset_handle_" || key == "entity_handle_") {
      seed = MixDdgiSignature(seed, 0u);
    } else {
      seed = MixStableYaml(seed, node[key]);
    }
  }
  return seed;
}

uint64_t GetStableSerializedObjectHash(const ISerializable& object, uint64_t seed = 0x535441424c45594dull) {
  YAML::Emitter out;
  out << YAML::BeginMap;
  Serialization::SerializeObject(out, object);
  out << YAML::EndMap;
  return MixStableYaml(seed, YAML::Load(out.c_str()));
}

uint64_t GetStableSerializedAssetHash(const std::shared_ptr<IAsset>& asset) {
  if (!asset) {
    return 0;
  }
  struct CacheEntry {
    std::weak_ptr<IAsset> asset;
    uint32_t version = 0;
    uint64_t hash = 0;
  };
  static std::unordered_map<const IAsset*, CacheEntry> cache;
  const bool cacheable = std::dynamic_pointer_cast<Mesh>(asset) || std::dynamic_pointer_cast<SkinnedMesh>(asset) ||
                         std::dynamic_pointer_cast<Strands>(asset) || std::dynamic_pointer_cast<Texture2D>(asset);
  if (cacheable) {
    if (const auto found = cache.find(asset.get());
        found != cache.end() && found->second.asset.lock() == asset && found->second.version == asset->GetVersion()) {
      return found->second.hash;
    }
  }
  auto hash = MixStableText(0x4153534554434f4eull, asset->GetTypeName());
  hash = GetStableSerializedObjectHash(*asset, hash);
  if (cacheable) {
    cache[asset.get()] = {asset, asset->GetVersion(), hash};
  }
  return hash;
}

uint64_t GetStableAssetGraphHash(const std::shared_ptr<IAsset>& root) {
  std::unordered_set<const IAsset*> visiting;
  const std::function<uint64_t(const std::shared_ptr<IAsset>&)> recurse =
      [&](const std::shared_ptr<IAsset>& asset) -> uint64_t {
    if (!asset) {
      return 0;
    }
    if (!visiting.insert(asset.get()).second) {
      return 0x4359434c45415354ull;
    }
    std::vector<AssetRef> references;
    Serialization::CollectAssetRefs(*asset, references);
    auto hash = GetStableSerializedAssetHash(asset);
    for (auto& reference : references) {
      const auto referenced_asset = reference.Get<IAsset>();
      if (referenced_asset) {
        hash = MixStableText(hash, referenced_asset->GetTypeName());
      }
      hash = MixDdgiSignature(hash, recurse(referenced_asset));
    }
    visiting.erase(asset.get());
    return hash;
  };
  return recurse(root);
}

uint64_t GetStableSceneComponentHash(const std::shared_ptr<Scene>& scene, const Entity owner,
                                     const std::shared_ptr<IPrivateComponent>& component) {
  auto hash = MixStableText(0x434f4d504f4e454eull, component->GetTypeName());
  hash = MixDdgiSignature(hash, scene->IsEntityEnabled(owner) ? 1u : 0u);
  hash = MixDdgiSignature(hash, component->IsEnabled() ? 1u : 0u);
  hash = MixDdgiMat4(hash, scene->GetDataComponent<GlobalTransform>(owner).value);
  hash = GetStableSerializedObjectHash(*component, hash);
  std::vector<AssetRef> references;
  Serialization::CollectAssetRefs(*component, references);
  for (auto& reference : references) {
    const auto referenced_asset = reference.Get<IAsset>();
    if (referenced_asset) {
      hash = MixStableText(hash, referenced_asset->GetTypeName());
    }
    hash = MixDdgiSignature(hash, GetStableAssetGraphHash(referenced_asset));
  }
  if (const auto skinned_renderer = std::dynamic_pointer_cast<SkinnedMeshRenderer>(component);
      skinned_renderer && skinned_renderer->bone_matrices) {
    for (const auto& matrix : skinned_renderer->bone_matrices->value) {
      hash = MixDdgiMat4(hash, matrix);
    }
  }
  return hash;
}

template <typename Component>
void AppendStableSceneComponentHashes(const std::shared_ptr<Scene>& scene, std::vector<uint64_t>& hashes) {
  if (const auto* owners = scene->UnsafeGetPrivateComponentOwnersList<Component>()) {
    for (const auto& owner : *owners) {
      if (const auto component = scene->GetOrSetPrivateComponent<Component>(owner).lock()) {
        hashes.emplace_back(GetStableSceneComponentHash(scene, owner, component));
      }
    }
  }
}

void AppendStableLodGroupHashes(const std::shared_ptr<Scene>& scene, std::vector<uint64_t>& hashes) {
  if (const auto* owners = scene->UnsafeGetPrivateComponentOwnersList<LodGroup>()) {
    for (const auto& owner : *owners) {
      const auto group = scene->GetOrSetPrivateComponent<LodGroup>(owner).lock();
      if (!group) {
        continue;
      }
      auto hash = 0x4c4f4447524f5550ull;
      hash = MixDdgiSignature(hash, scene->IsEntityEnabled(owner) ? 1u : 0u);
      hash = MixDdgiSignature(hash, group->IsEnabled() ? 1u : 0u);
      hash = MixDdgiMat4(hash, scene->GetDataComponent<GlobalTransform>(owner).value);
      hash = MixDdgiSignature(hash, group->override_lod_factor ? 1u : 0u);
      if (group->override_lod_factor) {
        hash = MixDdgiFloat(hash, group->lod_factor);
      }
      for (const auto& lod : group->lods) {
        hash = MixDdgiSignature(hash, static_cast<uint32_t>(lod.index));
        hash = MixDdgiFloat(hash, lod.lod_offset);
        hash = MixDdgiFloat(hash, lod.transition_width);
        for (auto renderer : lod.renderers) {
          if (const auto component = renderer.Get<IPrivateComponent>()) {
            hash = MixDdgiSignature(hash, GetStableSceneComponentHash(scene, component->GetOwner(), component));
          } else {
            hash = MixDdgiSignature(hash, 0u);
          }
        }
      }
      hashes.emplace_back(hash);
    }
  }
}

std::vector<uint64_t> CollectStableReflectionProbeSourceHashes(const std::shared_ptr<Scene>& scene) {
  std::vector<uint64_t> hashes;
  AppendStableSceneComponentHashes<MeshRenderer>(scene, hashes);
  AppendStableSceneComponentHashes<SkinnedMeshRenderer>(scene, hashes);
  AppendStableSceneComponentHashes<Particles>(scene, hashes);
  AppendStableSceneComponentHashes<StrandsRenderer>(scene, hashes);
  AppendStableSceneComponentHashes<DirectionalLight>(scene, hashes);
  AppendStableSceneComponentHashes<PointLight>(scene, hashes);
  AppendStableSceneComponentHashes<SpotLight>(scene, hashes);
  AppendStableLodGroupHashes(scene, hashes);
  std::sort(hashes.begin(), hashes.end());
  return hashes;
}

uint64_t GetReflectionProbeSceneSourceFingerprint(const std::shared_ptr<Scene>& scene) {
  if (!scene) {
    return 0u;
  }
  const auto signatures = CollectStableReflectionProbeSourceHashes(scene);
  auto fingerprint = MixDdgiSignature(0x52504653434e4553ull, signatures.size());
  for (const auto signature : signatures) {
    fingerprint = MixDdgiSignature(fingerprint, signature);
  }
  return fingerprint;
}

uint64_t GetStableCubemapHash(const std::shared_ptr<Cubemap>& cubemap) {
  if (!cubemap) {
    return 0;
  }
  struct CacheEntry {
    std::weak_ptr<Cubemap> cubemap;
    uint32_t version = 0;
    uint64_t generation = 0;
    uint64_t hash = 0;
  };
  static std::unordered_map<const Cubemap*, CacheEntry> cache;
  const auto generation = cubemap->PeekStorage().content_generation;
  if (const auto found = cache.find(cubemap.get()); found != cache.end() && found->second.cubemap.lock() == cubemap &&
                                                    found->second.version == cubemap->GetVersion() &&
                                                    found->second.generation == generation) {
    return found->second.hash;
  }
  auto hash = MixDdgiSignature(0x4355424553544142ull, cubemap->GetResolution());
  hash = MixDdgiSignature(hash, cubemap->GetMipLevels());
  hash = MixDdgiSignature(hash, static_cast<uint32_t>(cubemap->GetFormat()));
  if (cubemap->GetFormat() == VK_FORMAT_R16G16B16A16_SFLOAT) {
    std::vector<uint16_t> pixels;
    cubemap->GetRgba16fData(pixels);
    for (const auto pixel : pixels) {
      hash = MixDdgiSignature(hash, pixel);
    }
  } else {
    std::vector<glm::vec4> pixels;
    cubemap->GetRgbaChannelData(pixels);
    for (const auto& pixel : pixels) {
      hash = MixDdgiVec4(hash, pixel);
    }
  }
  cache[cubemap.get()] = {cubemap, cubemap->GetVersion(), generation, hash};
  return hash;
}

uint64_t MakeDdgiEnvironmentSignature(const RenderInstanceStorage::EnvironmentInfoBlock& environment,
                                      const uint32_t selected_cubemap_index) {
  auto signature = MixDdgiFloat(0u, environment.diffuse_sky_intensity);
  signature = MixDdgiFloat(signature, environment.diffuse_fallback_intensity);
  signature = MixDdgiFloat(signature, environment.background_color.w);
  if (environment.background_color.w == 1.0f) {
    return MixDdgiVec3(signature, glm::vec3(environment.background_color));
  }
  signature = MixDdgiFloat(signature, environment.environmental_map_gamma);
  signature = MixDdgiFloat(signature, environment.environment_rotation);
  signature = MixDdgiSignature(signature, selected_cubemap_index);
  if (environment.diffuse_fallback_intensity <= 0.0f) {
    return signature;
  }
  uint64_t cubemap_signature = 0;
  if (TextureStorage::TryGetCubemapContentSignature(selected_cubemap_index, cubemap_signature)) {
    signature = MixDdgiSignature(signature, cubemap_signature);
  }
  return signature;
}

struct DdgiMaterialInputSignatures {
  std::vector<uint64_t> keys;
  std::vector<uint64_t> materials;
  std::vector<uint64_t> textures;
  bool inputs_pending = false;
};

uint64_t MakeDdgiMaterialSignature(const GltfShadeMaterial& material) {
  auto signature = MixDdgiVec4(0u, material.pbr_base_color_factor);
  signature = MixDdgiVec3(signature, material.emissive_factor);
  signature = MixDdgiFloat(signature, material.normal_texture_scale);
  signature = MixDdgiFloat(signature, material.pbr_metallic_factor);
  signature = MixDdgiSignature(signature, static_cast<uint32_t>(material.alpha_mode));
  signature = MixDdgiFloat(signature, material.alpha_cutoff);
  signature = MixDdgiSignature(signature, static_cast<uint32_t>(material.double_sided));
#if MAT_EXT_IOR
  signature = MixDdgiFloat(signature, material.ior);
#endif
#if MAT_EXT_TRANSMISSION
  signature = MixDdgiFloat(signature, material.transmission_factor);
#endif
#if MAT_EXT_VOLUME
  signature = MixDdgiFloat(signature, material.thickness_factor);
#endif
#if MAT_EXT_CLEARCOAT
  signature = MixDdgiFloat(signature, material.clearcoat_factor);
  signature = MixDdgiFloat(signature, material.clearcoat_normal_texture_scale);
#endif
#if MAT_EXT_SPECULAR
  signature = MixDdgiVec3(signature, material.specular_color_factor);
  signature = MixDdgiFloat(signature, material.specular_factor);
#endif
#if MAT_EXT_UNLIT
  signature = MixDdgiSignature(signature, static_cast<uint32_t>(material.unlit));
#endif
#if MAT_EXT_SPECULAR_GLOSSINESS
  signature = MixDdgiSignature(signature, static_cast<uint32_t>(material.pbr_model));
  signature = MixDdgiVec4(signature, material.pbr_diffuse_factor);
  signature = MixDdgiVec3(signature, material.pbr_specular_factor);
#endif
#if MAT_EXT_DIFFUSE_TRANSMISSION
  signature = MixDdgiFloat(signature, material.diffuse_transmission_factor);
#endif
  return signature;
}

std::vector<uint16_t> GetDdgiMaterialTextureSlots(const GltfShadeMaterial& material) {
  std::vector<uint16_t> slots{material.pbr_base_color_texture, material.normal_texture,
                              material.pbr_metallic_roughness_texture, material.emissive_texture};
#if MAT_EXT_CLEARCOAT
  slots.push_back(material.clearcoat_texture);
  slots.push_back(material.clearcoat_normal_texture);
#endif
#if MAT_EXT_SPECULAR
  slots.push_back(material.specular_texture);
  slots.push_back(material.specular_color_texture);
#endif
#if MAT_EXT_SPECULAR_GLOSSINESS
  slots.push_back(material.pbr_diffuse_texture);
  slots.push_back(material.pbr_specular_glossiness_texture);
#endif
  return slots;
}

DdgiMaterialInputSignatures CollectDdgiMaterialInputSignatures(
    const std::shared_ptr<RenderInstanceStorage>& render_instances,
    std::vector<std::pair<int32_t, uint64_t>> material_references) {
  DdgiMaterialInputSignatures result;
  if (!render_instances) {
    return result;
  }
  std::sort(material_references.begin(), material_references.end(), [](const auto& lhs, const auto& rhs) {
    return lhs.second < rhs.second || (lhs.second == rhs.second && lhs.first < rhs.first);
  });
  material_references.erase(std::unique(material_references.begin(), material_references.end(),
                                        [](const auto& lhs, const auto& rhs) {
                                          return lhs.second == rhs.second;
                                        }),
                            material_references.end());
  const auto& materials = render_instances->GetGltfShadeMaterials();
  const auto& texture_infos = render_instances->GetGltfTextureInfos();
  for (const auto& [material_index, material_key] : material_references) {
    if (material_index < 0 || static_cast<size_t>(material_index) >= materials.size()) {
      continue;
    }
    result.keys.push_back(material_key);
    const auto& material = materials[material_index];
    result.materials.push_back(MakeDdgiMaterialSignature(material));
    auto texture_signature = material_key;
    const auto texture_slots = GetDdgiMaterialTextureSlots(material);
    for (size_t slot_index = 0; slot_index < texture_slots.size(); ++slot_index) {
      const auto texture_info_index = texture_slots[slot_index];
      texture_signature = MixDdgiSignature(texture_signature, slot_index);
      if (texture_info_index == 0u) {
        texture_signature = MixDdgiSignature(texture_signature, 0u);
        continue;
      }
      if (texture_info_index >= texture_infos.size()) {
        texture_signature = MixDdgiSignature(texture_signature, texture_info_index);
        continue;
      }
      const auto& texture_info = texture_infos[texture_info_index];
#if MAT_EXT_TEXTURE_TRANSFORM
      for (int column = 0; column < 3; ++column) {
        for (int row = 0; row < 2; ++row) {
          texture_signature = MixDdgiFloat(texture_signature, texture_info.uv_transform[column][row]);
        }
      }
#endif
      texture_signature = MixDdgiSignature(texture_signature, static_cast<uint32_t>(texture_info.tex_coord));
      texture_signature = MixDdgiSignature(texture_signature, static_cast<uint32_t>(texture_info.color_space));
      uint64_t content_signature = 0;
      if (texture_info.index >= 0) {
        const auto texture_index = static_cast<uint32_t>(texture_info.index);
        result.inputs_pending |= TextureStorage::HasPendingTexture2DUpload(texture_index);
        if (TextureStorage::TryGetTexture2DContentSignature(texture_index, content_signature)) {
          texture_signature = MixDdgiSignature(texture_signature, content_signature);
          continue;
        }
      }
      texture_signature = MixDdgiSignature(texture_signature, static_cast<uint32_t>(texture_info.index));
    }
    result.textures.push_back(texture_signature);
  }
  return result;
}

uint64_t MakeDdgiLightSignature(const std::shared_ptr<Scene>& scene, const uint32_t type_index, const Entity& owner,
                                const DirectionalLight& light) {
  const auto rotation = scene->GetDataComponent<GlobalTransform>(owner).GetRotation();
  auto signature =
      MixDdgiVec3(MakeDdgiLightKey(type_index, owner), glm::normalize(rotation * glm::vec3(0.0f, 0.0f, 1.0f)));
  signature = MixDdgiSignature(signature, light.cast_shadow ? 1u : 0u);
  return MixDdgiVec3(signature, light.diffuse * light.diffuse_brightness);
}

uint64_t MakeDdgiLightSignature(const std::shared_ptr<Scene>& scene, const uint32_t type_index, const Entity& owner,
                                const PointLight& light) {
  auto signature =
      MixDdgiVec3(MakeDdgiLightKey(type_index, owner), scene->GetDataComponent<GlobalTransform>(owner).GetPosition());
  signature = MixDdgiSignature(signature, light.cast_shadow ? 1u : 0u);
  signature = MixDdgiFloat(signature, light.constant);
  signature = MixDdgiFloat(signature, light.linear);
  signature = MixDdgiFloat(signature, light.quadratic);
  signature = MixDdgiVec3(signature, light.diffuse * light.diffuse_brightness);
  return MixDdgiFloat(signature, light.range > 0.0f ? light.range : light.GetFarPlane());
}

uint64_t MakeDdgiLightSignature(const std::shared_ptr<Scene>& scene, const uint32_t type_index, const Entity& owner,
                                const SpotLight& light) {
  const auto transform = scene->GetDataComponent<GlobalTransform>(owner);
  auto signature = MixDdgiVec3(MakeDdgiLightKey(type_index, owner), transform.GetPosition());
  signature = MixDdgiVec3(signature, glm::normalize(transform.GetRotation() * glm::vec3(0.0f, 0.0f, -1.0f)));
  signature = MixDdgiSignature(signature, light.cast_shadow ? 1u : 0u);
  signature = MixDdgiFloat(signature, light.inner_degrees);
  signature = MixDdgiFloat(signature, light.outer_degrees);
  signature = MixDdgiFloat(signature, light.constant);
  signature = MixDdgiFloat(signature, light.linear);
  signature = MixDdgiFloat(signature, light.quadratic);
  signature = MixDdgiVec3(signature, light.diffuse * light.diffuse_brightness);
  return MixDdgiFloat(signature, light.range > 0.0f ? light.range : light.GetFarPlane());
}

template <typename LightComponent>
void CollectDdgiActiveLightKeys(const std::shared_ptr<Scene>& scene, const uint32_t type_index,
                                std::vector<uint64_t>& keys,
                                const uint32_t maximum_count = (std::numeric_limits<uint32_t>::max)()) {
  uint32_t count = 0;
  if (const auto* owners = scene->UnsafeGetPrivateComponentOwnersList<LightComponent>()) {
    for (const auto& owner : *owners) {
      if (!scene->IsEntityEnabled(owner)) {
        continue;
      }
      const auto light = scene->GetOrSetPrivateComponent<LightComponent>(owner).lock();
      if (light && light->IsEnabled()) {
        if (count >= maximum_count) {
          break;
        }
        keys.push_back(MakeDdgiLightKey(type_index, owner));
        ++count;
      }
    }
  }
}

std::vector<uint64_t> CollectDdgiActiveLightKeys(const std::shared_ptr<Scene>& scene) {
  std::vector<uint64_t> keys;
  if (!scene) {
    return keys;
  }
  const auto max_directional_lights =
      ApplicationContext::Get().GetApplicationInfo().graphics_settings.max_directional_light_size;
  CollectDdgiActiveLightKeys<DirectionalLight>(scene, 1u, keys, max_directional_lights);
  CollectDdgiActiveLightKeys<PointLight>(scene, 2u, keys);
  CollectDdgiActiveLightKeys<SpotLight>(scene, 3u, keys);
  std::sort(keys.begin(), keys.end());
  return keys;
}

template <typename LightComponent>
void CollectDdgiLightSignatures(const std::shared_ptr<Scene>& scene, const uint32_t type_index,
                                std::vector<uint64_t>& signatures,
                                const uint32_t maximum_count = (std::numeric_limits<uint32_t>::max)()) {
  uint32_t count = 0;
  if (const auto* owners = scene->UnsafeGetPrivateComponentOwnersList<LightComponent>()) {
    for (const auto& owner : *owners) {
      if (!scene->IsEntityEnabled(owner)) {
        continue;
      }
      const auto light = scene->GetOrSetPrivateComponent<LightComponent>(owner).lock();
      if (light && light->IsEnabled()) {
        if (count >= maximum_count) {
          break;
        }
        signatures.push_back(MakeDdgiLightSignature(scene, type_index, owner, *light));
        ++count;
      }
    }
  }
}

std::vector<uint64_t> CollectDdgiLightSignatures(const std::shared_ptr<Scene>& scene) {
  std::vector<uint64_t> signatures;
  if (!scene) {
    return signatures;
  }
  const auto max_directional_lights =
      ApplicationContext::Get().GetApplicationInfo().graphics_settings.max_directional_light_size;
  CollectDdgiLightSignatures<DirectionalLight>(scene, 1u, signatures, max_directional_lights);
  CollectDdgiLightSignatures<PointLight>(scene, 2u, signatures);
  CollectDdgiLightSignatures<SpotLight>(scene, 3u, signatures);
  std::sort(signatures.begin(), signatures.end());
  return signatures;
}

bool IsDdgiAccelerationStructureTransformValid(const glm::mat4& transform) {
  for (glm::length_t column = 0; column < 4; ++column) {
    for (glm::length_t row = 0; row < 4; ++row) {
      if (!std::isfinite(transform[column][row])) {
        return false;
      }
    }
  }
  const auto determinant = glm::determinant(glm::mat3(transform));
  return std::isfinite(determinant) && determinant != 0.0f;
}

bool IsDdgiTlasContributor(const std::shared_ptr<RenderInstanceStorage::IRenderInstance>& render_instance) {
  if (!render_instance) {
    return false;
  }
  if (const auto mesh = std::dynamic_pointer_cast<RenderInstanceStorage::MeshRenderInstance>(render_instance)) {
    if (!mesh->mesh) {
      return false;
    }
    const auto blas = mesh->ray_tracing_blas ? mesh->ray_tracing_blas : (mesh->mesh ? mesh->mesh->GetBlas() : nullptr);
    return blas && blas->IsReady() && IsDdgiAccelerationStructureTransformValid(mesh->model.value);
  }
  if (const auto skinned =
          std::dynamic_pointer_cast<RenderInstanceStorage::SkinnedMeshRenderInstance>(render_instance)) {
    if (!skinned->skinned_mesh) {
      return false;
    }
    const auto blas = skinned->ray_tracing_blas ? skinned->ray_tracing_blas
                                                : (skinned->skinned_mesh ? skinned->skinned_mesh->GetBlas() : nullptr);
    return blas && blas->IsReady() && IsDdgiAccelerationStructureTransformValid(skinned->model.value);
  }
  if (const auto instanced =
          std::dynamic_pointer_cast<RenderInstanceStorage::InstancedRenderInstance>(render_instance)) {
    if (!instanced->mesh || !instanced->mesh->GetBlas() || !instanced->mesh->GetBlas()->IsReady() ||
        !instanced->particle_infos) {
      return false;
    }
    const auto& particle_infos = instanced->particle_infos->PeekParticleInfoList();
    return std::any_of(particle_infos.begin(), particle_infos.end(), [&](const auto& particle_info) {
      return IsDdgiAccelerationStructureTransformValid(instanced->model.value * particle_info.instance_matrix.value);
    });
  }
  if (const auto external = std::dynamic_pointer_cast<RenderInstanceStorage::ExternalRenderInstance>(render_instance)) {
    const auto& blas = external->ddgi_geometry.bottom_level_acceleration_structure;
    return external->HasDdgiRayTracingGeometry() && blas && blas->IsReady() &&
           IsDdgiAccelerationStructureTransformValid(external->model.value);
  }
  return false;
}

uint64_t MakeDdgiGeometrySignature(const std::shared_ptr<RenderInstanceStorage::IRenderInstance>& render_instance,
                                   const bool include_shading_inputs) {
  if (!render_instance) {
    return 0;
  }
  const auto mix_range = [](uint64_t signature, const std::shared_ptr<RangeDescriptor>& range) {
    if (!range) {
      return MixDdgiSignature(signature, 0u);
    }
    signature = MixDdgiSignature(signature, range->offset);
    signature = MixDdgiSignature(signature, range->range);
    signature = MixDdgiSignature(signature, range->prev_frame_offset);
    signature = MixDdgiSignature(signature, range->index_count);
    return MixDdgiSignature(signature, range->prev_frame_index_count);
  };
  const auto mix_blas = [](uint64_t signature, const std::shared_ptr<BottomLevelAccelerationStructure>& blas) {
    if (!blas) {
      return MixDdgiSignature(signature, 0u);
    }
    signature = MixDdgiSignature(signature, reinterpret_cast<uintptr_t>(blas.get()));
    signature = MixDdgiSignature(signature, blas->GetDeviceAddress());
    return MixDdgiSignature(signature, blas->GetContentVersion());
  };
  auto signature = static_cast<uint64_t>(render_instance->command_type);
  signature = MixDdgiSignature(signature, render_instance->owner.GetIndex());
  signature = MixDdgiSignature(signature, render_instance->owner.GetVersion());
  signature = MixDdgiSignature(signature, static_cast<uint64_t>(render_instance->entity_handle));
  signature = MixDdgiSignature(signature, static_cast<uint64_t>(render_instance->renderer_handle));
  if (include_shading_inputs) {
    signature = MixDdgiSignature(signature,
                                 render_instance->material
                                     ? render_instance->material->GetHandle().GetValue()
                                     : static_cast<uint64_t>(static_cast<uint32_t>(render_instance->material_index)));
    signature = MixDdgiSignature(signature, render_instance->cast_shadow ? 1u : 0u);
  }
  signature = MixDdgiSignature(signature, render_instance->geometry_version);
  if (const auto mesh = std::dynamic_pointer_cast<RenderInstanceStorage::MeshRenderInstance>(render_instance)) {
    signature = MixDdgiSignature(signature, mesh->mesh ? mesh->mesh->GetHandle().GetValue() : 0u);
    signature = MixDdgiSignature(signature, mesh->ray_tracing_geometry_version);
    signature = MixDdgiSignature(signature, mesh->morph_weights_version);
    signature =
        mix_range(signature, mesh->ray_tracing_triangle_range
                                 ? mesh->ray_tracing_triangle_range
                                 : (mesh->mesh ? mesh->mesh->GetTriangleRange() : std::shared_ptr<RangeDescriptor>{}));
    signature = mix_blas(
        signature, mesh->ray_tracing_blas
                       ? mesh->ray_tracing_blas
                       : (mesh->mesh ? mesh->mesh->GetBlas() : std::shared_ptr<BottomLevelAccelerationStructure>{}));
  }
  if (const auto skinned =
          std::dynamic_pointer_cast<RenderInstanceStorage::SkinnedMeshRenderInstance>(render_instance)) {
    signature = MixDdgiSignature(signature, skinned->skinned_mesh ? skinned->skinned_mesh->GetHandle().GetValue() : 0u);
    signature = MixDdgiSignature(signature, skinned->ray_tracing_geometry_version);
    signature = MixDdgiSignature(signature, skinned->morph_weights_version);
    signature = mix_range(signature, skinned->ray_tracing_triangle_range
                                         ? skinned->ray_tracing_triangle_range
                                         : (skinned->skinned_mesh ? skinned->skinned_mesh->GetRayTracingTriangleRange()
                                                                  : std::shared_ptr<RangeDescriptor>{}));
    signature =
        mix_blas(signature, skinned->ray_tracing_blas
                                ? skinned->ray_tracing_blas
                                : (skinned->skinned_mesh ? skinned->skinned_mesh->GetBlas()
                                                         : std::shared_ptr<BottomLevelAccelerationStructure>{}));
  }
  if (const auto instanced =
          std::dynamic_pointer_cast<RenderInstanceStorage::InstancedRenderInstance>(render_instance)) {
    signature = MixDdgiSignature(signature, instanced->mesh ? instanced->mesh->GetHandle().GetValue() : 0u);
    signature = mix_range(signature,
                          instanced->mesh ? instanced->mesh->GetTriangleRange() : std::shared_ptr<RangeDescriptor>{});
    signature = mix_blas(
        signature, instanced->mesh ? instanced->mesh->GetBlas() : std::shared_ptr<BottomLevelAccelerationStructure>{});
    if (instanced->particle_infos) {
      const auto& particle_infos = instanced->particle_infos->PeekParticleInfoList();
      std::vector<uint64_t> transform_signatures;
      transform_signatures.reserve(particle_infos.size());
      for (const auto& particle_info : particle_infos) {
        const auto transform = instanced->model.value * particle_info.instance_matrix.value;
        if (IsDdgiAccelerationStructureTransformValid(transform)) {
          transform_signatures.push_back(MixDdgiMat4(0u, transform));
        }
      }
      std::sort(transform_signatures.begin(), transform_signatures.end());
      signature = MixDdgiSignature(signature, transform_signatures.size());
      for (const auto transform_signature : transform_signatures) {
        signature = MixDdgiSignature(signature, transform_signature);
      }
    }
  }
  if (const auto external = std::dynamic_pointer_cast<RenderInstanceStorage::ExternalRenderInstance>(render_instance)) {
    signature = MixDdgiSignature(signature, external->ddgi_geometry.geometry_version);
    signature = MixDdgiSignature(signature, static_cast<uint32_t>(external->ddgi_geometry.triangle_offset));
    signature = mix_blas(signature, external->ddgi_geometry.bottom_level_acceleration_structure);
  }
  return std::dynamic_pointer_cast<RenderInstanceStorage::InstancedRenderInstance>(render_instance)
             ? signature
             : MixDdgiMat4(signature, render_instance->model.value);
}

bool DdgiTriggerConditionEnabled(const int conditions, const int condition) {
  return (conditions & condition) != 0;
}

RenderResourceDescriptor CreateDdgiBufferResourceDescriptor(const std::string& name, const uint64_t byte_size) {
  return {name, RenderResourceType::Buffer, RenderResourceLifetime::Frame, {}, {}, 1, 1, true, byte_size};
}

RenderResourceDescriptor CreateDdgiImportedBufferResourceDescriptor(const std::string& name, const uint64_t byte_size) {
  return {name, RenderResourceType::Buffer, RenderResourceLifetime::Persistent, {}, {}, 1, 1, false, byte_size};
}

std::shared_ptr<Buffer> CreateDdgiProbeStateBuffer(const uint64_t byte_size) {
  if (byte_size == 0 || !Platform::Initialized()) {
    return {};
  }
  VkBufferCreateInfo buffer_create_info{};
  buffer_create_info.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
  buffer_create_info.size = byte_size;
  buffer_create_info.usage = VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_SRC_BIT |
                             VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT;
  buffer_create_info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;

  VmaAllocationCreateInfo allocation_create_info{};
  allocation_create_info.usage = VMA_MEMORY_USAGE_AUTO;
  return std::make_shared<Buffer>(buffer_create_info, allocation_create_info);
}

std::shared_ptr<Buffer> CreateDdgiFallbackProbeStateBuffer(const uint64_t byte_size) {
  const auto fallback_byte_size = glm::max(byte_size, static_cast<uint64_t>(sizeof(glm::vec4)));
  auto buffer = CreateDdgiProbeStateBuffer(fallback_byte_size);
  if (buffer) {
    const std::vector<glm::vec4> inactive_state(fallback_byte_size / sizeof(glm::vec4),
                                                glm::vec4(0.0f, 0.0f, 0.0f, 1.0f));
    buffer->UploadVector(inactive_state);
  }
  return buffer;
}

void AddDdgiFrameResources(RenderGraph& graph, const RenderLayer::DdgiFrameResourceLayout& layout) {
  graph.AddResource(CreateDdgiImportedBufferResourceDescriptor(RenderResourceNames::frame_ddgi_probe_metadata,
                                                               layout.probe_metadata_byte_size));
  graph.AddResource(CreateDdgiImportedBufferResourceDescriptor(RenderResourceNames::frame_ddgi_probe_state,
                                                               layout.probe_state_byte_size));
  graph.AddResource(
      CreateDdgiBufferResourceDescriptor(RenderResourceNames::frame_ddgi_ray_output, layout.ray_output_byte_size));
  graph.AddResource(CreateDdgiImportedBufferResourceDescriptor(RenderResourceNames::frame_ddgi_selected_ray_diagnostics,
                                                               layout.selected_ray_diagnostics_byte_size));
  graph.AddResource(CreateDdgiImportedImageResourceDescriptor(RenderResourceNames::frame_ddgi_irradiance_atlas,
                                                              layout.irradiance_atlas, "RGBA16F"));
  graph.AddResource(CreateDdgiImportedImageResourceDescriptor(RenderResourceNames::frame_ddgi_visibility_atlas,
                                                              layout.visibility_atlas, "RG16F"));
  graph.AddResource(CreateDdgiImportedImageResourceDescriptor(RenderResourceNames::frame_ddgi_variability_atlas,
                                                              layout.variability_atlas, "R16F"));
  graph.AddResource(CreateDdgiImageResourceDescriptor(RenderResourceNames::frame_ddgi_variability_reduction_a,
                                                      layout.variability_reduction_extent, "RG32F"));
  graph.AddResource(CreateDdgiImageResourceDescriptor(RenderResourceNames::frame_ddgi_variability_reduction_b,
                                                      layout.variability_reduction_extent, "RG32F"));
}

void AddDdgiProbeVisualizationFrameResources(RenderGraph& graph, const RenderLayer::DdgiFrameResourceLayout& layout) {
  graph.AddResource(CreateDdgiImportedBufferResourceDescriptor(RenderResourceNames::frame_ddgi_probe_metadata,
                                                               layout.probe_metadata_byte_size));
  graph.AddResource(CreateDdgiImportedBufferResourceDescriptor(RenderResourceNames::frame_ddgi_probe_state,
                                                               layout.probe_state_byte_size));
  graph.AddResource(CreateDdgiImportedImageResourceDescriptor(RenderResourceNames::frame_ddgi_irradiance_atlas,
                                                              layout.irradiance_atlas, "RGBA16F"));
}

void AddDdgiProbeRayVisualizationFrameResources(RenderGraph& graph,
                                                const RenderLayer::DdgiFrameResourceLayout& layout) {
  graph.AddResource(CreateDdgiImportedBufferResourceDescriptor(RenderResourceNames::frame_ddgi_selected_ray_diagnostics,
                                                               layout.selected_ray_diagnostics_byte_size));
}

void AddDdgiRayTracingFrameResources(RenderGraph& graph) {
  graph.AddResource({RenderResourceNames::frame_ray_tracing_descriptor_set, RenderResourceType::DescriptorSet,
                     RenderResourceLifetime::Frame});
  graph.AddResource(
      {RenderResourceNames::scene_mesh_tlas, RenderResourceType::AccelerationStructure, RenderResourceLifetime::Frame});
}

std::string CameraVariantShaderHeader(const uint32_t feature_mask) {
  return Platform::GetShaderGlobalDefines() + "\n#define EE_CAMERA_ENABLE_DEBUG_VIEWS " +
         ((feature_mask & kRayCameraDebugViewsFeature) != 0u ? "1\n" : "0\n") +
         BuildGltfSceneFeatureDefines(feature_mask);
}

std::shared_ptr<RayTracingPipeline> CreateRayTracingCameraPipeline(
    const std::shared_ptr<DescriptorSetLayout>& per_frame_layout,
    const std::shared_ptr<DescriptorSetLayout>& ray_tracing_layout,
    const std::shared_ptr<DescriptorSetLayout>& camera_output_layout, const std::string& shader_header,
    const std::shared_ptr<Shader>& shared_miss_shader = {},
    const std::shared_ptr<Shader>& shared_closest_hit_shader = {}) {
  auto pipeline = std::make_shared<RayTracingPipeline>();
  pipeline->SetMaxRecursionDepth(1);
  pipeline->raygen_shader =
      Shader::CreateTemporary(ShaderType::RayGen, shader_header,
                              Resources::GetDefaultResourcesPath() / "Shaders/RayTracing/RayGen/Camera.slang");
  pipeline->miss_shader =
      shared_miss_shader
          ? shared_miss_shader
          : Shader::CreateTemporary(ShaderType::Miss, Platform::GetShaderGlobalDefines(),
                                    Resources::GetDefaultResourcesPath() / "Shaders/RayTracing/Miss/Camera.slang");
  pipeline->closest_hit_shader =
      shared_closest_hit_shader ? shared_closest_hit_shader
                                : Shader::CreateTemporary(ShaderType::ClosestHit, Platform::GetShaderGlobalDefines(),
                                                          Resources::GetDefaultResourcesPath() /
                                                              "Shaders/RayTracing/ClosestHit/Camera.slang");
  pipeline->any_hit_shader =
      Shader::CreateTemporary(ShaderType::AnyHit, shader_header,
                              Resources::GetDefaultResourcesPath() / "Shaders/RayTracing/AnyHit/Camera.slang");
  pipeline->descriptor_set_layouts = {per_frame_layout, ray_tracing_layout, camera_output_layout};
  auto& push_constant_range = pipeline->push_constant_ranges.emplace_back();
  push_constant_range.size = sizeof(RayTracingCameraPushConstant);
  push_constant_range.offset = 0;
  push_constant_range.stageFlags = VK_SHADER_STAGE_RAYGEN_BIT_KHR | VK_SHADER_STAGE_MISS_BIT_KHR |
                                   VK_SHADER_STAGE_CLOSEST_HIT_BIT_KHR | VK_SHADER_STAGE_ANY_HIT_BIT_KHR;
  pipeline->Initialize();
  return pipeline;
}

std::shared_ptr<ComputePipeline> CreateRayQueryCameraPipeline(
    const std::shared_ptr<DescriptorSetLayout>& per_frame_layout,
    const std::shared_ptr<DescriptorSetLayout>& ray_tracing_layout,
    const std::shared_ptr<DescriptorSetLayout>& camera_output_layout, const std::string& shader_header) {
  auto pipeline = std::make_shared<ComputePipeline>();
  pipeline->compute_shader =
      Shader::CreateTemporary(ShaderType::Compute, shader_header,
                              Resources::GetDefaultResourcesPath() / "Shaders/Compute/RayQueryCamera.slang");
  pipeline->descriptor_set_layouts = {per_frame_layout, ray_tracing_layout, camera_output_layout};
  auto& push_constant_range = pipeline->push_constant_ranges.emplace_back();
  push_constant_range.size = sizeof(RayTracingCameraPushConstant);
  push_constant_range.offset = 0;
  push_constant_range.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
  pipeline->Initialize();
  return pipeline;
}

}  // namespace

uint32_t RenderLayer::GetDdgiProbeCount(const glm::ivec3& probe_counts) {
  return DdgiRuntime::GetProbeCount(probe_counts);
}

uint32_t RenderLayer::GetDdgiFixedRayCount(const uint32_t ray_count, const bool fixed_rays_enabled) {
  return DdgiRuntime::GetFixedRayCount(ray_count, fixed_rays_enabled);
}

RenderLayer::DdgiProbeUpdateVariant RenderLayer::ParseDdgiProbeUpdateVariant(const std::string_view value) {
  return DdgiRuntime::ParseProbeUpdateVariant(value);
}

RenderLayer::DdgiProbeConvergenceUpdate RenderLayer::AdvanceDdgiProbeConvergence(
    const DdgiProbeConvergenceState& state, const DdgiProbeVariabilityObservation& observation,
    const uint32_t minimum_sample_count, const float entry_threshold) {
  return DdgiRuntime::AdvanceProbeConvergence(state, observation, minimum_sample_count, entry_threshold);
}

bool RenderLayer::IsDdgiPeriodicRefreshDue(const bool gating_enabled, const bool converged,
                                           const bool waiting_for_observation, const uint32_t refresh_age) {
  return DdgiRuntime::IsPeriodicRefreshDue(gating_enabled, converged, waiting_for_observation, refresh_age);
}

bool RenderLayer::IsDdgiReflectionProbeRuntimeReady(const bool has_valid_history, const bool lighting_descriptors_bound,
                                                    const bool variability_gating_enabled,
                                                    const bool variability_converged) {
  return DdgiRuntime::IsReflectionProbeRuntimeReady(has_valid_history, lighting_descriptors_bound,
                                                    variability_gating_enabled, variability_converged);
}

RenderLayer::DdgiProbeUpdateVariant RenderLayer::ResolveDdgiProbeUpdateVariant(
    const DdgiProbeUpdateVariant requested, const DdgiProbeUpdateDeviceLimits& limits, const uint32_t probe_count,
    const bool irradiance_pipeline_ready, const bool visibility_pipeline_ready) {
  return DdgiRuntime::ResolveProbeUpdateVariant(requested, limits, probe_count, irradiance_pipeline_ready,
                                                visibility_pipeline_ready);
}

uint32_t RenderLayer::GetDdgiAllocatedProbeCount(const DdgiSettings& settings) {
  return DdgiRuntime::GetAllocatedProbeCount(settings);
}

uint32_t RenderLayer::GetDdgiAllocatedProbeCount(const DdgiSettings& settings, const uint32_t probe_count) {
  return DdgiRuntime::GetAllocatedProbeCount(settings, probe_count);
}

bool RenderLayer::ValidateDdgiProbeGrid(const glm::ivec3& probe_counts, const uint32_t max_probe_count,
                                        std::string* error) {
  return DdgiRuntime::ValidateProbeGrid(probe_counts, max_probe_count, error);
}

RenderLayer::DdgiRuntimePolicy RenderLayer::ResolveDdgiRuntimePolicy(const DdgiSettings& settings) {
  return DdgiRuntime::ResolveRuntimePolicy(settings);
}

bool RenderLayer::ResolveDdgiEmissiveMeshSampling(const bool global_enabled, const int volume_mode) {
  return DdgiRuntime::ResolveEmissiveMeshSampling(global_enabled, volume_mode);
}

uint32_t RenderLayer::GetDdgiProbeRayFlags(const bool skip_inactive_probes, const bool emissive_mesh_sampling) {
  return DdgiRuntime::GetProbeRayFlags(skip_inactive_probes, emissive_mesh_sampling);
}

uint64_t RenderLayer::CalculateDdgiEmissiveSamplingCandidateRayCount(const uint32_t updated_probe_count,
                                                                     const uint32_t ray_count,
                                                                     const uint32_t fixed_ray_count,
                                                                     const bool emissive_mesh_sampling,
                                                                     const bool trace_probe_rays) {
  return DdgiRuntime::CalculateEmissiveSamplingCandidateRayCount(updated_probe_count, ray_count, fixed_ray_count,
                                                                 emissive_mesh_sampling, trace_probe_rays);
}

bool RenderLayer::RequiresDdgiFullScrollReset(const glm::ivec3& probe_counts, const glm::ivec3& scroll_delta) {
  return DdgiRuntime::RequiresFullScrollReset(probe_counts, scroll_delta);
}

glm::uvec3 RenderLayer::GetDdgiProbeGridIndex(const glm::ivec3& probe_counts, const uint32_t probe_index) {
  return DdgiRuntime::GetProbeGridIndex(probe_counts, probe_index);
}

RenderLayer::DdgiAtlasLayout RenderLayer::CalculateDdgiAtlasLayout(const uint32_t probe_count,
                                                                   const uint32_t tile_resolution,
                                                                   const uint32_t preferred_columns) {
  return DdgiRuntime::CalculateAtlasLayout(probe_count, tile_resolution, preferred_columns);
}

RenderLayer::DdgiAtlasLayout RenderLayer::CalculateDdgiAtlasLayout(const uint32_t probe_count,
                                                                   const uint32_t tile_resolution,
                                                                   const uint32_t preferred_columns,
                                                                   const uint32_t max_image_dimension_2d) {
  return DdgiRuntime::CalculateAtlasLayout(probe_count, tile_resolution, preferred_columns, max_image_dimension_2d);
}

RenderLayer::DdgiFrameResourceLayout RenderLayer::CalculateDdgiFrameResourceLayout(const DdgiSettings& settings) {
  return DdgiRuntime::CalculateFrameResourceLayout(settings);
}

RenderLayer::DdgiFrameResourceLayout RenderLayer::CalculateDdgiFrameResourceLayout(const DdgiSettings& settings,
                                                                                   const uint32_t probe_count) {
  return DdgiRuntime::CalculateFrameResourceLayout(settings, probe_count);
}

RenderLayer::DdgiFrameResourceLayout RenderLayer::CalculateDdgiFrameResourceLayout(
    const DdgiSettings& settings, const uint32_t probe_count, const uint32_t max_image_dimension_2d) {
  return DdgiRuntime::CalculateFrameResourceLayout(settings, probe_count, max_image_dimension_2d);
}

RenderLayer::DdgiFrameResourceLayout RenderLayer::CalculateDdgiFrameResourceLayout(
    const DdgiSettings& settings, const uint32_t probe_count, const uint32_t max_image_dimension_2d,
    const uint64_t max_storage_buffer_range) {
  return DdgiRuntime::CalculateFrameResourceLayout(settings, probe_count, max_image_dimension_2d,
                                                   max_storage_buffer_range);
}

bool RenderLayer::AreDdgiPersistentLayoutsCompatible(const DdgiFrameResourceLayout& previous,
                                                     const DdgiFrameResourceLayout& current) {
  return DdgiRuntime::ArePersistentLayoutsCompatible(previous, current);
}

float RenderLayer::CalculateDdgiUpdateHysteresis(const DdgiSettings& settings, const uint32_t update_reasons) {
  return DdgiRuntime::CalculateUpdateHysteresis(settings, update_reasons);
}

float RenderLayer::CalculateDdgiUpdateHysteresis(const DdgiSettings& settings, const uint32_t update_reasons,
                                                 const uint32_t warmup_frame_index) {
  return DdgiRuntime::CalculateUpdateHysteresis(settings, update_reasons, warmup_frame_index);
}

float RenderLayer::CalculateDdgiUpdateBrightnessThreshold(const DdgiSettings& settings) {
  return DdgiRuntime::CalculateUpdateBrightnessThreshold(settings);
}

std::string RenderLayer::FormatDdgiUpdateReasons(const uint32_t reasons) {
  return DdgiRuntime::FormatUpdateReasons(reasons);
}

float RenderLayer::CalculateDdgiVolumeBlendWeight(const glm::vec3& probe_coordinate, const glm::ivec3& probe_counts,
                                                  const glm::vec3& probe_step_lengths) {
  return DdgiRuntime::CalculateVolumeBlendWeight(probe_coordinate, probe_counts, probe_step_lengths);
}

float RenderLayer::CalculateDdgiProbeDensity(const glm::vec3& probe_step_x, const glm::vec3& probe_step_y,
                                             const glm::vec3& probe_step_z) {
  return DdgiRuntime::CalculateProbeDensity(probe_step_x, probe_step_y, probe_step_z);
}

void RenderLayer::SortDdgiVolumeRuntimeInfos(std::vector<DdgiVolumeRuntimeInfo>& infos) {
  DdgiRuntime::SortVolumeRuntimeInfos(infos);
}

RenderLayer::DdgiVolumeSetValidation RenderLayer::ValidateDdgiVolumeSet(const std::vector<DdgiVolumeRuntimeInfo>& infos,
                                                                        const uint32_t configured_probe_limit) {
  return DdgiRuntime::ValidateVolumeSet(infos, configured_probe_limit);
}

RenderLayer::DdgiVolumeSelection RenderLayer::SelectDdgiVolumes(const std::vector<DdgiVolumeRuntimeInfo>& infos,
                                                                const glm::vec3& world_position) {
  return DdgiRuntime::SelectVolumes(infos, world_position);
}

std::vector<RenderLayer::DdgiVolumeRuntimeInfo> RenderLayer::CollectDdgiVolumeRuntimeInfos(
    const std::shared_ptr<Scene>& scene, const DdgiSettings& settings) {
  (void)settings;
  const auto resolved_lighting = ResolveEnvironmentalLighting(scene);
  std::vector<DdgiVolumeRuntimeInfo> infos;
  infos.reserve(resolved_lighting.ddgi_volumes.size());
  for (size_t i = 0; i < resolved_lighting.ddgi_volumes.size(); ++i) {
    const auto& volume = resolved_lighting.ddgi_volumes[i];
    const auto source = CreateDdgiProbeRayDiagnosticSourceFromResolvedVolume(volume, resolved_lighting.ddgi_settings);
    auto& info = infos.emplace_back();
    info.sorted_index = static_cast<uint32_t>(i);
    info.stable_entity_id = volume.stable_id;
    info.artist_priority = volume.artist_priority;
    info.probe_density = CalculateDdgiProbeDensity(source.probe_step_x, source.probe_step_y, source.probe_step_z);
    info.probe_counts = source.probe_counts;
    info.probe_count = GetDdgiProbeCount(source.probe_counts);
    info.first_probe = source.first_probe;
    info.probe_step_x = source.probe_step_x;
    info.probe_step_y = source.probe_step_y;
    info.probe_step_z = source.probe_step_z;
  }
  return infos;
}

const std::shared_ptr<DescriptorSetLayout>& RenderLayer::GetPerFrameDescriptorSetLayout() const {
  return per_frame_layout_;
}

const std::shared_ptr<DescriptorSetLayout>& RenderLayer::GetMeshletDescriptorSetLayout() const {
  return meshlet_layout_;
}

const std::shared_ptr<DescriptorSetLayout>& RenderLayer::GetLightingDescriptorSetLayout() const {
  return lighting_layout_;
}

const std::shared_ptr<DescriptorSetLayout>& RenderLayer::GetRayTracingDescriptorSetLayout() const {
  return ray_tracing_layout_;
}

const std::shared_ptr<DescriptorSetLayout>& RenderLayer::GetRayTracingPointCloudDescriptorSetLayout() const {
  return ray_tracing_point_cloud_layout_;
}

const std::shared_ptr<DescriptorSetLayout>& RenderLayer::GetParticleInstancedDataDescriptorSetLayout() const {
  return particle_instanced_data_layout_;
}

const std::shared_ptr<DescriptorSetLayout>& RenderLayer::GetBoneMatricesDescriptorSetLayout() const {
  return bone_matrices_layout_;
}

const std::shared_ptr<DescriptorSetLayout>& RenderLayer::GetCameraGBufferDescriptorSetLayout() const {
  return camera_g_buffer_layout_;
}

const std::shared_ptr<DescriptorSetLayout>& RenderLayer::GetRenderTextureStorageDescriptorSetLayout() const {
  return render_texture_storage_layout_;
}

const std::shared_ptr<DescriptorSetLayout>& RenderLayer::GetRenderTexturePresentDescriptorSetLayout() const {
  return render_texture_present_layout_;
}

const std::shared_ptr<DescriptorSetLayout>& RenderLayer::GetRasterMaterialDescriptorSetLayout() const {
  return raster_material_layout_;
}

void RenderLayer::InitializeCommonDescriptorSetLayouts(
    const ApplicationInitializationSettings& application_initialization_settings) {
  if (!empty_descriptor_set_layout_) {
    empty_descriptor_set_layout_ = std::make_shared<DescriptorSetLayout>();
    empty_descriptor_set_layout_->Initialize();
  }
  if (!render_texture_present_layout_) {
    render_texture_present_layout_ = std::make_shared<DescriptorSetLayout>();
    render_texture_present_layout_->PushDescriptorBinding(
        0, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, VK_SHADER_STAGE_FRAGMENT_BIT | VK_SHADER_STAGE_COMPUTE_BIT, 0);
    render_texture_present_layout_->Initialize();
  }
  if (!per_frame_layout_) {
    per_frame_layout_ = std::make_shared<DescriptorSetLayout>();
    PushPerFrameSceneDescriptorBindings(per_frame_layout_);
    per_frame_bindless_texture_descriptors_enabled_ = ShouldCreatePerFrameBindlessTextureDescriptors();
    if (per_frame_bindless_texture_descriptors_enabled_) {
      PushPerFrameBindlessTextureDescriptorBindings(per_frame_layout_, application_initialization_settings);
    }
    PushPerFrameMaterialBufferDescriptorBindings(per_frame_layout_);
    per_frame_layout_->Initialize();
  }
  if (!raster_material_per_frame_layout_) {
    raster_material_per_frame_layout_ = std::make_shared<DescriptorSetLayout>();
    PushPerFrameSceneDescriptorBindings(raster_material_per_frame_layout_);
    PushPerFrameMaterialBufferDescriptorBindings(raster_material_per_frame_layout_);
    raster_material_per_frame_layout_->Initialize();
  }
  if (!raster_material_layout_) {
    raster_material_layout_ = std::make_shared<DescriptorSetLayout>();
    for (uint32_t binding = 0; binding < RenderInstanceStorage::kRasterMaterialTextureSlotCount; binding++) {
      raster_material_layout_->PushDescriptorBinding(binding, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER,
                                                     VK_SHADER_STAGE_FRAGMENT_BIT, 0);
    }
    raster_material_layout_->Initialize();
  }
  if (!raster_lighting_texture_layout_) {
    const auto& limits = Platform::GetSelectedPhysicalDevice()->properties.limits;
    if (limits.maxPerStageDescriptorSamplers < kRasterLightingMaxPerStageSamplerCount ||
        limits.maxDescriptorSetSamplers < kRasterLightingDescriptorSamplerCount ||
        limits.maxPerStageDescriptorSampledImages < kRasterLightingMaxPerStageSamplerCount ||
        limits.maxDescriptorSetSampledImages < kRasterLightingDescriptorSamplerCount) {
      throw std::runtime_error("The selected Vulkan device cannot bind 32 spatial reflection probes.");
    }
    raster_lighting_texture_layout_ = std::make_shared<DescriptorSetLayout>();
    for (uint32_t binding = 0; binding < 5; binding++) {
      raster_lighting_texture_layout_->PushDescriptorBinding(binding, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER,
                                                             VK_SHADER_STAGE_FRAGMENT_BIT, 0);
    }
    raster_lighting_texture_layout_->PushDescriptorBinding(
        kRasterLightingReflectionProbesBinding, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, VK_SHADER_STAGE_FRAGMENT_BIT,
        0, RenderInstanceStorage::kReflectionProbeMaxCount);
    raster_lighting_texture_layout_->Initialize();
  }
  if (!meshlet_layout_) {
    meshlet_layout_ = std::make_shared<DescriptorSetLayout>();
    meshlet_layout_->PushDescriptorBinding(0, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,
                                           VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_MESH_BIT_EXT, 0);
    meshlet_layout_->PushDescriptorBinding(1, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,
                                           VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_MESH_BIT_EXT, 0);
    meshlet_layout_->Initialize();
  }
  if (Platform::MeshShaderEnabled() && !strand_meshlet_layout_) {
    strand_meshlet_layout_ = std::make_shared<DescriptorSetLayout>();
    constexpr VkShaderStageFlags strand_mesh_stages = VK_SHADER_STAGE_TASK_BIT_EXT | VK_SHADER_STAGE_MESH_BIT_EXT;
    strand_meshlet_layout_->PushDescriptorBinding(0, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, strand_mesh_stages, 0);
    strand_meshlet_layout_->PushDescriptorBinding(1, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, strand_mesh_stages, 0);
    strand_meshlet_layout_->Initialize();
  }
  if (!lighting_layout_) {
    lighting_layout_ = std::make_shared<DescriptorSetLayout>();
    lighting_layout_->PushDescriptorBinding(14, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, VK_SHADER_STAGE_FRAGMENT_BIT,
                                            0);
    lighting_layout_->PushDescriptorBinding(15, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, VK_SHADER_STAGE_FRAGMENT_BIT,
                                            0);
    lighting_layout_->PushDescriptorBinding(16, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, VK_SHADER_STAGE_FRAGMENT_BIT,
                                            0);
    lighting_layout_->PushDescriptorBinding(17, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, VK_SHADER_STAGE_FRAGMENT_BIT,
                                            0, RenderInstanceStorage::kDdgiMaxVolumeCount);
    lighting_layout_->PushDescriptorBinding(18, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, VK_SHADER_STAGE_FRAGMENT_BIT,
                                            0, RenderInstanceStorage::kDdgiMaxVolumeCount);
    lighting_layout_->PushDescriptorBinding(19, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_FRAGMENT_BIT, 0,
                                            RenderInstanceStorage::kDdgiMaxVolumeCount);
    lighting_layout_->Initialize();
  }
  if (Platform::RayAccelerationStructureEnabled() && !ray_tracing_layout_) {
    ray_tracing_layout_ = std::make_shared<DescriptorSetLayout>();
    VkShaderStageFlags ray_camera_geometry_stages = 0;
    if (Platform::RayQueryEnabled()) {
      ray_camera_geometry_stages |= VK_SHADER_STAGE_COMPUTE_BIT;
    }
    if (Platform::RayTracingEnabled()) {
      ray_camera_geometry_stages |=
          VK_SHADER_STAGE_RAYGEN_BIT_KHR | VK_SHADER_STAGE_ANY_HIT_BIT_KHR | VK_SHADER_STAGE_CLOSEST_HIT_BIT_KHR;
    }
    ray_tracing_layout_->PushDescriptorBinding(0, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, ray_camera_geometry_stages, 0);
    ray_tracing_layout_->PushDescriptorBinding(1, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, ray_camera_geometry_stages, 0);
    ray_tracing_layout_->PushDescriptorBinding(2, VK_DESCRIPTOR_TYPE_ACCELERATION_STRUCTURE_KHR,
                                               ray_camera_geometry_stages, 0);
    ray_tracing_layout_->PushDescriptorBinding(3, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, ray_camera_geometry_stages, 0);
    ray_tracing_layout_->Initialize();
  }
  if (Platform::RayAccelerationStructureEnabled() && !ray_tracing_camera_output_layout_) {
    ray_tracing_camera_output_layout_ = std::make_shared<DescriptorSetLayout>();
    VkShaderStageFlags ray_camera_output_stages = 0;
    if (Platform::RayQueryEnabled()) {
      ray_camera_output_stages |= VK_SHADER_STAGE_COMPUTE_BIT;
    }
    if (Platform::RayTracingEnabled()) {
      ray_camera_output_stages |= VK_SHADER_STAGE_RAYGEN_BIT_KHR;
    }
    ray_tracing_camera_output_layout_->PushDescriptorBinding(0, VK_DESCRIPTOR_TYPE_STORAGE_IMAGE,
                                                             ray_camera_output_stages, 0);
    ray_tracing_camera_output_layout_->PushDescriptorBinding(1, VK_DESCRIPTOR_TYPE_STORAGE_IMAGE,
                                                             ray_camera_output_stages, 0);
    ray_tracing_camera_output_layout_->PushDescriptorBinding(2, VK_DESCRIPTOR_TYPE_STORAGE_IMAGE,
                                                             ray_camera_output_stages, 0);
    ray_tracing_camera_output_layout_->PushDescriptorBinding(3, VK_DESCRIPTOR_TYPE_STORAGE_IMAGE,
                                                             ray_camera_output_stages, 0);
    ray_tracing_camera_output_layout_->Initialize();
  }
  if (Platform::RayTracingEnabled() && !ray_tracing_point_cloud_layout_) {
    ray_tracing_point_cloud_layout_ = std::make_shared<DescriptorSetLayout>();
    ray_tracing_point_cloud_layout_->PushDescriptorBinding(0, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,
                                                           VK_SHADER_STAGE_RAYGEN_BIT_KHR, 0);
    ray_tracing_point_cloud_layout_->Initialize();
  }
  if (Platform::RayTracingEnabled() && !ddgi_probe_ray_output_layout_) {
    ddgi_probe_ray_output_layout_ = std::make_shared<DescriptorSetLayout>();
    ddgi_probe_ray_output_layout_->PushDescriptorBinding(0, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,
                                                         VK_SHADER_STAGE_RAYGEN_BIT_KHR, 0);
    ddgi_probe_ray_output_layout_->PushDescriptorBinding(
        1, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_RAYGEN_BIT_KHR | VK_SHADER_STAGE_CLOSEST_HIT_BIT_KHR, 0);
    ddgi_probe_ray_output_layout_->PushDescriptorBinding(2, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,
                                                         VK_SHADER_STAGE_RAYGEN_BIT_KHR, 0);
    ddgi_probe_ray_output_layout_->PushDescriptorBinding(17, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER,
                                                         VK_SHADER_STAGE_CLOSEST_HIT_BIT_KHR, 0);
    ddgi_probe_ray_output_layout_->PushDescriptorBinding(18, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER,
                                                         VK_SHADER_STAGE_CLOSEST_HIT_BIT_KHR, 0);
    ddgi_probe_ray_output_layout_->Initialize();
  }
  if (!particle_instanced_data_layout_) {
    particle_instanced_data_layout_ = std::make_shared<DescriptorSetLayout>();
    particle_instanced_data_layout_->PushDescriptorBinding(0, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_ALL,
                                                           0);
    particle_instanced_data_layout_->Initialize();
  }
  if (!bone_matrices_layout_) {
    bone_matrices_layout_ = std::make_shared<DescriptorSetLayout>();
    bone_matrices_layout_->PushDescriptorBinding(0, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_VERTEX_BIT, 0);
    bone_matrices_layout_->Initialize();
  }
  if (!camera_g_buffer_layout_) {
    camera_g_buffer_layout_ = std::make_shared<DescriptorSetLayout>();
    camera_g_buffer_layout_->PushDescriptorBinding(17, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER,
                                                   VK_SHADER_STAGE_FRAGMENT_BIT | VK_SHADER_STAGE_COMPUTE_BIT, 0);
    camera_g_buffer_layout_->PushDescriptorBinding(20, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER,
                                                   VK_SHADER_STAGE_FRAGMENT_BIT | VK_SHADER_STAGE_COMPUTE_BIT, 0);
    camera_g_buffer_layout_->PushDescriptorBinding(21, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER,
                                                   VK_SHADER_STAGE_FRAGMENT_BIT | VK_SHADER_STAGE_COMPUTE_BIT, 0);
    camera_g_buffer_layout_->PushDescriptorBinding(22, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER,
                                                   VK_SHADER_STAGE_FRAGMENT_BIT | VK_SHADER_STAGE_COMPUTE_BIT, 0);
    camera_g_buffer_layout_->PushDescriptorBinding(23, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER,
                                                   VK_SHADER_STAGE_FRAGMENT_BIT | VK_SHADER_STAGE_COMPUTE_BIT, 0);
    camera_g_buffer_layout_->PushDescriptorBinding(24, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER,
                                                   VK_SHADER_STAGE_FRAGMENT_BIT | VK_SHADER_STAGE_COMPUTE_BIT, 0);
    camera_g_buffer_layout_->Initialize();
  }
  if (!render_texture_storage_layout_) {
    render_texture_storage_layout_ = std::make_shared<DescriptorSetLayout>();
    render_texture_storage_layout_->PushDescriptorBinding(0, VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, VK_SHADER_STAGE_ALL, 0);
    render_texture_storage_layout_->Initialize();
  }
  if (!depth_pyramid_layout_) {
    depth_pyramid_layout_ = std::make_shared<DescriptorSetLayout>();
    depth_pyramid_layout_->PushDescriptorBinding(0, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER,
                                                 VK_SHADER_STAGE_COMPUTE_BIT, 0);
    depth_pyramid_layout_->PushDescriptorBinding(1, VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, VK_SHADER_STAGE_COMPUTE_BIT, 0);
    depth_pyramid_layout_->Initialize();
  }
  if (!motion_vectors_layout_) {
    motion_vectors_layout_ = std::make_shared<DescriptorSetLayout>();
    motion_vectors_layout_->PushDescriptorBinding(0, VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, VK_SHADER_STAGE_COMPUTE_BIT, 0);
    motion_vectors_layout_->PushDescriptorBinding(1, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_COMPUTE_BIT, 0);
    motion_vectors_layout_->Initialize();
  }
  if (!motion_coverage_layout_) {
    motion_coverage_layout_ = std::make_shared<DescriptorSetLayout>();
    motion_coverage_layout_->PushDescriptorBinding(0, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_VERTEX_BIT, 0);
    motion_coverage_layout_->PushDescriptorBinding(1, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_VERTEX_BIT, 0);
    motion_coverage_layout_->Initialize();
  }
  if (!volumetric_clouds_layout_) {
    volumetric_clouds_layout_ = std::make_shared<DescriptorSetLayout>();
    volumetric_clouds_layout_->PushDescriptorBinding(0, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER,
                                                     VK_SHADER_STAGE_COMPUTE_BIT, 0);
    volumetric_clouds_layout_->PushDescriptorBinding(1, VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, VK_SHADER_STAGE_COMPUTE_BIT,
                                                     0);
    volumetric_clouds_layout_->PushDescriptorBinding(2, VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, VK_SHADER_STAGE_COMPUTE_BIT,
                                                     0);
    volumetric_clouds_layout_->PushDescriptorBinding(3, VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, VK_SHADER_STAGE_COMPUTE_BIT,
                                                     0);
    volumetric_clouds_layout_->PushDescriptorBinding(4, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER,
                                                     VK_SHADER_STAGE_COMPUTE_BIT, 0);
    volumetric_clouds_layout_->PushDescriptorBinding(5, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER,
                                                     VK_SHADER_STAGE_COMPUTE_BIT, 0);
    volumetric_clouds_layout_->PushDescriptorBinding(6, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER,
                                                     VK_SHADER_STAGE_COMPUTE_BIT, 0);
    volumetric_clouds_layout_->PushDescriptorBinding(7, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER,
                                                     VK_SHADER_STAGE_COMPUTE_BIT, 0);
    volumetric_clouds_layout_->PushDescriptorBinding(8, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER,
                                                     VK_SHADER_STAGE_COMPUTE_BIT, 0);
    volumetric_clouds_layout_->PushDescriptorBinding(9, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER,
                                                     VK_SHADER_STAGE_COMPUTE_BIT, 0);
    volumetric_clouds_layout_->PushDescriptorBinding(10, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER,
                                                     VK_SHADER_STAGE_COMPUTE_BIT, 0);
    volumetric_clouds_layout_->PushDescriptorBinding(11, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER,
                                                     VK_SHADER_STAGE_COMPUTE_BIT, 0);
    volumetric_clouds_layout_->PushDescriptorBinding(12, VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, VK_SHADER_STAGE_COMPUTE_BIT,
                                                     0);
    volumetric_clouds_layout_->PushDescriptorBinding(13, VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, VK_SHADER_STAGE_COMPUTE_BIT,
                                                     0);
    volumetric_clouds_layout_->Initialize();
  }
  if (!ddgi_probe_update_layout_) {
    ddgi_probe_update_layout_ = std::make_shared<DescriptorSetLayout>();
    ddgi_probe_update_layout_->PushDescriptorBinding(0, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_COMPUTE_BIT,
                                                     0);
    ddgi_probe_update_layout_->PushDescriptorBinding(1, VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, VK_SHADER_STAGE_COMPUTE_BIT,
                                                     0);
    ddgi_probe_update_layout_->PushDescriptorBinding(2, VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, VK_SHADER_STAGE_COMPUTE_BIT,
                                                     0);
    ddgi_probe_update_layout_->PushDescriptorBinding(3, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_COMPUTE_BIT,
                                                     0);
    ddgi_probe_update_layout_->PushDescriptorBinding(4, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_COMPUTE_BIT,
                                                     0);
    ddgi_probe_update_layout_->PushDescriptorBinding(5, VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, VK_SHADER_STAGE_COMPUTE_BIT,
                                                     0);
    ddgi_probe_update_layout_->Initialize();
  }
  if (!ddgi_probe_relocation_layout_) {
    ddgi_probe_relocation_layout_ = std::make_shared<DescriptorSetLayout>();
    ddgi_probe_relocation_layout_->PushDescriptorBinding(0, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,
                                                         VK_SHADER_STAGE_COMPUTE_BIT, 0);
    ddgi_probe_relocation_layout_->PushDescriptorBinding(1, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,
                                                         VK_SHADER_STAGE_COMPUTE_BIT, 0);
    ddgi_probe_relocation_layout_->Initialize();
  }
  if (!ddgi_probe_classification_layout_) {
    ddgi_probe_classification_layout_ = std::make_shared<DescriptorSetLayout>();
    ddgi_probe_classification_layout_->PushDescriptorBinding(0, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,
                                                             VK_SHADER_STAGE_COMPUTE_BIT, 0);
    ddgi_probe_classification_layout_->PushDescriptorBinding(1, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,
                                                             VK_SHADER_STAGE_COMPUTE_BIT, 0);
    ddgi_probe_classification_layout_->Initialize();
  }
  if (!ddgi_probe_variability_layout_) {
    ddgi_probe_variability_layout_ = std::make_shared<DescriptorSetLayout>();
    ddgi_probe_variability_layout_->PushDescriptorBinding(0, VK_DESCRIPTOR_TYPE_STORAGE_IMAGE,
                                                          VK_SHADER_STAGE_COMPUTE_BIT, 0);
    ddgi_probe_variability_layout_->PushDescriptorBinding(1, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,
                                                          VK_SHADER_STAGE_COMPUTE_BIT, 0);
    ddgi_probe_variability_layout_->PushDescriptorBinding(2, VK_DESCRIPTOR_TYPE_STORAGE_IMAGE,
                                                          VK_SHADER_STAGE_COMPUTE_BIT, 0);
    ddgi_probe_variability_layout_->Initialize();
  }
  if (!ddgi_probe_visualization_layout_) {
    ddgi_probe_visualization_layout_ = std::make_shared<DescriptorSetLayout>();
    ddgi_probe_visualization_layout_->PushDescriptorBinding(0, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,
                                                            VK_SHADER_STAGE_VERTEX_BIT, 0);
    ddgi_probe_visualization_layout_->PushDescriptorBinding(1, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,
                                                            VK_SHADER_STAGE_VERTEX_BIT, 0);
    ddgi_probe_visualization_layout_->PushDescriptorBinding(17, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER,
                                                            VK_SHADER_STAGE_FRAGMENT_BIT, 0);
    ddgi_probe_visualization_layout_->Initialize();
  }
  if (!ddgi_probe_ray_visualization_layout_) {
    ddgi_probe_ray_visualization_layout_ = std::make_shared<DescriptorSetLayout>();
    ddgi_probe_ray_visualization_layout_->PushDescriptorBinding(0, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,
                                                                VK_SHADER_STAGE_VERTEX_BIT, 0);
    ddgi_probe_ray_visualization_layout_->Initialize();
  }
  if (!gaussian_splat_layout_) {
    gaussian_splat_layout_ = std::make_shared<DescriptorSetLayout>();
    gaussian_splat_layout_->PushDescriptorBinding(
        0, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,
        VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_MESH_BIT_EXT | VK_SHADER_STAGE_COMPUTE_BIT, 0);
    gaussian_splat_layout_->PushDescriptorBinding(
        1, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,
        VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_MESH_BIT_EXT | VK_SHADER_STAGE_COMPUTE_BIT, 0);
    gaussian_splat_layout_->PushDescriptorBinding(
        2, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,
        VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_MESH_BIT_EXT | VK_SHADER_STAGE_COMPUTE_BIT, 0);
    gaussian_splat_layout_->PushDescriptorBinding(3, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,
                                                  VK_SHADER_STAGE_MESH_BIT_EXT | VK_SHADER_STAGE_COMPUTE_BIT, 0);
    gaussian_splat_layout_->PushDescriptorBinding(4, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_COMPUTE_BIT, 0);
    gaussian_splat_layout_->Initialize();
  }
  if (!gaussian_splat_radix_sort_layout_) {
    gaussian_splat_radix_sort_layout_ = std::make_shared<DescriptorSetLayout>();
    for (uint32_t binding = 0; binding < 7u; ++binding) {
      gaussian_splat_radix_sort_layout_->PushDescriptorBinding(binding, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,
                                                               VK_SHADER_STAGE_COMPUTE_BIT, 0);
    }
    gaussian_splat_radix_sort_layout_->Initialize();
  }
}

void RenderLayer::RenderToPointLightShadowMap(
    std::function<uint32_t(VkCommandBuffer vk_command_buffer, const PointLightShadowMapView& shadow_map_view)>&& func) {
  point_light_shadow_map_external_functions.emplace_back(func);
}

void RenderLayer::RenderToSpotLightShadowMap(
    std::function<uint32_t(VkCommandBuffer vk_command_buffer, const SpotLightShadowMapView& shadow_map_view)>&& func) {
  spot_light_shadow_map_external_functions.emplace_back(func);
}

void RenderLayer::RenderToDirectionalLightShadowMap(
    std::function<uint32_t(VkCommandBuffer vk_command_buffer, const DirectionalLightShadowMapView& shadow_map_view)>&&
        func) {
  directional_light_shadow_map_external_functions.emplace_back(func);
}

void RenderLayer::DeferredRenderingAllCameras(
    std::function<uint32_t(VkCommandBuffer vk_command_buffer,
                           const std::vector<VkRenderingAttachmentInfo>& geometry_pass_color_attachment_infos,
                           const DeferredRenderingView& forward_rendering_view)>&& func) {
  deferred_rendering_external_functions.emplace_back(func);
}

void RenderLayer::ForwardRenderingAllCameras(
    std::function<uint32_t(VkCommandBuffer vk_command_buffer, const std::shared_ptr<Camera>& target_camera,
                           const ForwardRenderingView& forward_rendering_view)>&& func) {
  forward_rendering_external_functions.emplace_back(func);
}

void RenderLayer::RegisterRenderResource(RenderResourceDescriptor descriptor) {
  external_render_resource_descriptors.emplace_back(std::move(descriptor));
}

void RenderLayer::RegisterFrameRenderPass(RenderPassDescriptor descriptor,
                                          std::function<uint32_t(VkCommandBuffer vk_command_buffer)>&& func) {
  descriptor.scope = RenderPassScope::Frame;
  frame_render_pass_external_functions.push_back({std::move(descriptor), std::move(func), {}});
}

void RenderLayer::RegisterFrameRenderPass(
    RenderPassDescriptor descriptor,
    std::function<uint32_t(VkCommandBuffer vk_command_buffer, const RenderGraphExecutionContext& context)>&& func) {
  descriptor.scope = RenderPassScope::Frame;
  frame_render_pass_external_functions.push_back({std::move(descriptor), {}, std::move(func)});
}

void RenderLayer::RegisterCameraRenderPass(
    RenderPassDescriptor descriptor,
    std::function<uint32_t(VkCommandBuffer vk_command_buffer, const std::shared_ptr<Camera>& target_camera,
                           const ForwardRenderingView& forward_rendering_view)>&& func) {
  descriptor.scope = RenderPassScope::Camera;
  camera_render_pass_external_functions.push_back({std::move(descriptor), std::move(func), {}});
}

void RenderLayer::RegisterCameraRenderPass(
    RenderPassDescriptor descriptor,
    std::function<uint32_t(VkCommandBuffer vk_command_buffer, const std::shared_ptr<Camera>& target_camera,
                           const ForwardRenderingView& forward_rendering_view,
                           const RenderGraphExecutionContext& context)>&& func) {
  descriptor.scope = RenderPassScope::Camera;
  camera_render_pass_external_functions.push_back({std::move(descriptor), {}, std::move(func)});
}

void RenderLayer::OnCreate() {
  const auto startup_begin = std::chrono::steady_clock::now();
  const auto shader_stats_begin = Shader::GetCompileCacheStats();
  const auto initialize_render_instance_storage = [&] {
    const auto max_frames_in_flight = Platform::GetMaxFramesInFlight();
    render_instances_list_.resize(max_frames_in_flight);
    for (auto& i : render_instances_list_) {
      i = std::make_shared<RenderInstanceStorage>();
    }
  };
  const auto log_startup = [&](const bool prewarm_enabled) {
    const auto shader_stats = Shader::GetCompileCacheStats();
    std::ostringstream stream;
    stream << "EVOENGINE_RENDER_LAYER_STARTUP prewarm=" << (prewarm_enabled ? "enabled" : "disabled") << " elapsed_ms="
           << std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - startup_begin).count()
           << " shader_memory_hits=" << (shader_stats.memory_hits - shader_stats_begin.memory_hits)
           << " shader_disk_hits=" << (shader_stats.disk_hits - shader_stats_begin.disk_hits)
           << " shader_disk_misses=" << (shader_stats.disk_misses - shader_stats_begin.disk_misses)
           << " shader_compilations=" << (shader_stats.compilations - shader_stats_begin.compilations)
           << " shader_slang_frontend="
           << (shader_stats.slang_frontend_invocations - shader_stats_begin.slang_frontend_invocations);
    EVOENGINE_LOG(stream.str())
  };
  post_processing_renderer_resources_ = std::make_shared<PostProcessingRendererResources>();
  render_graph_transient_resource_stores_.clear();
  render_graph_transient_resource_stores_.resize(Platform::GetMaxFramesInFlight());
  ray_camera_render_graph_plan_cache_.Clear();
  ray_camera_history_cameras_.clear();
  retired_ray_camera_history_stats_ = {};
  peak_live_ray_camera_history_count_ = 0;
  peak_live_ray_camera_history_byte_size_ = 0;
  peak_live_ray_camera_output_descriptor_count_ = 0;
  enable_inspection = false;
  if (!ddgi_atlas_sampler_) {
    ddgi_atlas_sampler_ = CreateDdgiAtlasSampler();
  }
  if (!ApplicationContext::Get().GetApplicationInfo().prewarm_render_pipelines) {
    initialize_render_instance_storage();
    log_startup(false);
    return;
  }
  if (!depth_pyramid_pipeline_) {
    depth_pyramid_pipeline_ = std::make_shared<ComputePipeline>();
    depth_pyramid_pipeline_->compute_shader =
        Shader::CreateTemporary(ShaderType::Compute, Platform::GetShaderGlobalDefines(),
                                Resources::GetDefaultResourcesPath() / "Shaders/Compute/DepthPyramid.slang");
    depth_pyramid_pipeline_->descriptor_set_layouts.emplace_back(depth_pyramid_layout_);
    auto& push_constant_range = depth_pyramid_pipeline_->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(glm::uvec4);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
    depth_pyramid_pipeline_->Initialize();
  }
  if (!motion_vectors_pipeline_) {
    motion_vectors_pipeline_ = std::make_shared<ComputePipeline>();
    motion_vectors_pipeline_->compute_shader =
        Shader::CreateTemporary(ShaderType::Compute, Platform::GetShaderGlobalDefines(),
                                Resources::GetDefaultResourcesPath() / "Shaders/Compute/MotionVectors.slang");
    motion_vectors_pipeline_->descriptor_set_layouts.emplace_back(per_frame_layout_);
    motion_vectors_pipeline_->descriptor_set_layouts.emplace_back(camera_g_buffer_layout_);
    motion_vectors_pipeline_->descriptor_set_layouts.emplace_back(motion_vectors_layout_);
    auto& push_constant_range = motion_vectors_pipeline_->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(glm::ivec2);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
    motion_vectors_pipeline_->Initialize();
  }
  if (!volumetric_clouds_pipeline_) {
    volumetric_clouds_pipeline_ = std::make_shared<ComputePipeline>();
    volumetric_clouds_pipeline_->compute_shader =
        Shader::CreateTemporary(ShaderType::Compute, Platform::GetShaderGlobalDefines(),
                                Resources::GetDefaultResourcesPath() / "Shaders/Compute/VolumetricClouds.slang");
    volumetric_clouds_pipeline_->descriptor_set_layouts.emplace_back(per_frame_layout_);
    volumetric_clouds_pipeline_->descriptor_set_layouts.emplace_back(volumetric_clouds_layout_);
    auto& push_constant_range = volumetric_clouds_pipeline_->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(VolumetricCloudsPushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
    volumetric_clouds_pipeline_->Initialize();
  }
  if (!volumetric_clouds_composite_pipeline_) {
    volumetric_clouds_composite_pipeline_ = std::make_shared<ComputePipeline>();
    volumetric_clouds_composite_pipeline_->compute_shader = Shader::CreateTemporary(
        ShaderType::Compute, Platform::GetShaderGlobalDefines(),
        Resources::GetDefaultResourcesPath() / "Shaders/Compute/VolumetricCloudsComposite.slang");
    volumetric_clouds_composite_pipeline_->descriptor_set_layouts.emplace_back(per_frame_layout_);
    volumetric_clouds_composite_pipeline_->descriptor_set_layouts.emplace_back(volumetric_clouds_layout_);
    auto& push_constant_range = volumetric_clouds_composite_pipeline_->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(VolumetricCloudsPushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
    volumetric_clouds_composite_pipeline_->Initialize();
  }
  if (!gaussian_splat_cull_pipeline_) {
    gaussian_splat_cull_pipeline_ = std::make_shared<ComputePipeline>();
    gaussian_splat_cull_pipeline_->compute_shader =
        Shader::CreateTemporary(ShaderType::Compute, Platform::GetShaderGlobalDefines(),
                                Resources::GetDefaultResourcesPath() / "Shaders/Compute/GaussianSplatCull.slang");
    gaussian_splat_cull_pipeline_->descriptor_set_layouts.emplace_back(per_frame_layout_);
    gaussian_splat_cull_pipeline_->descriptor_set_layouts.emplace_back(gaussian_splat_layout_);
    auto& push_constant_range = gaussian_splat_cull_pipeline_->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(GaussianSplatCullPushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
    gaussian_splat_cull_pipeline_->Initialize();
  }
  const auto create_gaussian_splat_radix_pipeline = [&](std::shared_ptr<ComputePipeline>& pipeline,
                                                        const std::filesystem::path& shader_path) {
    if (pipeline) {
      return;
    }
    pipeline = std::make_shared<ComputePipeline>();
    pipeline->compute_shader =
        Shader::CreateTemporary(ShaderType::Compute, Platform::GetShaderGlobalDefines(), shader_path);
    pipeline->descriptor_set_layouts.emplace_back(gaussian_splat_radix_sort_layout_);
    auto& push_constant_range = pipeline->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(GaussianSplatRadixSortPushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
    pipeline->Initialize();
  };
  const auto gaussian_splat_compute_path = Resources::GetDefaultResourcesPath() / "Shaders/Compute";
  create_gaussian_splat_radix_pipeline(gaussian_splat_radix_upsweep_pipeline_,
                                       gaussian_splat_compute_path / "GaussianSplatRadixUpsweep.slang");
  create_gaussian_splat_radix_pipeline(gaussian_splat_radix_spine_pipeline_,
                                       gaussian_splat_compute_path / "GaussianSplatRadixSpine.slang");
  create_gaussian_splat_radix_pipeline(gaussian_splat_radix_downsweep_pipeline_,
                                       gaussian_splat_compute_path / "GaussianSplatRadixDownsweep.slang");
  if (!ddgi_probe_scroll_pipeline_) {
    ddgi_probe_scroll_pipeline_ = std::make_shared<ComputePipeline>();
    ddgi_probe_scroll_pipeline_->compute_shader =
        Shader::CreateTemporary(ShaderType::Compute, Platform::GetShaderGlobalDefines(),
                                Resources::GetDefaultResourcesPath() / "Shaders/Compute/DDGIProbeScroll.slang");
    ddgi_probe_scroll_pipeline_->descriptor_set_layouts.emplace_back(ddgi_probe_update_layout_);
    auto& push_constant_range = ddgi_probe_scroll_pipeline_->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(DdgiProbeScrollPushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
    ddgi_probe_scroll_pipeline_->Initialize();
  }
  if (!ddgi_probe_update_pipeline_) {
    ddgi_probe_update_pipeline_ = std::make_shared<ComputePipeline>();
    ddgi_probe_update_pipeline_->compute_shader =
        Shader::CreateTemporary(ShaderType::Compute, Platform::GetShaderGlobalDefines(),
                                Resources::GetDefaultResourcesPath() / "Shaders/Compute/DDGIProbeUpdate.slang");
    ddgi_probe_update_pipeline_->descriptor_set_layouts.emplace_back(per_frame_layout_);
    ddgi_probe_update_pipeline_->descriptor_set_layouts.emplace_back(ddgi_probe_update_layout_);
    auto& push_constant_range = ddgi_probe_update_pipeline_->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(DdgiProbeAtlasUpdatePushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
    ddgi_probe_update_pipeline_->Initialize();
  }
  {
    const char* environment_variant = std::getenv("EVOENGINE_DDGI_PROBE_UPDATE_VARIANT");
    const std::string requested_name =
        environment_variant && environment_variant[0] != '\0' ? environment_variant : "parallel-shared";
    const auto requested_variant = ParseDdgiProbeUpdateVariant(requested_name);
    DdgiProbeUpdateDeviceLimits limits;
    const auto& capabilities = Platform::GetInstance().GetCapabilities();
    limits.max_work_group_invocations = capabilities.max_compute_work_group_invocations;
    limits.max_shared_memory_bytes = capabilities.max_shared_memory_size;
    if (const auto physical_device = Platform::GetSelectedPhysicalDevice()) {
      limits.max_work_group_size_x = physical_device->properties.limits.maxComputeWorkGroupSize[0];
      limits.max_work_group_count_x = physical_device->properties.limits.maxComputeWorkGroupCount[0];
      limits.max_work_group_count_y = physical_device->properties.limits.maxComputeWorkGroupCount[1];
    }

    const auto supported_variant = ResolveDdgiProbeUpdateVariant(requested_variant, limits, 1u, true, true);
    const auto create_parallel_pipeline = [&](std::shared_ptr<ComputePipeline>& pipeline, const uint32_t mode) {
      pipeline = std::make_shared<ComputePipeline>();
      const bool use_shared_rays = requested_variant == DdgiProbeUpdateVariant::ParallelShared;
      const auto shader_header = Platform::GetShaderGlobalDefines() + "\n#define EE_DDGI_PROBE_UPDATE_MODE " +
                                 std::to_string(mode) + "\n#define EE_DDGI_PROBE_USE_SHARED_RAYS " +
                                 (use_shared_rays ? "1\n" : "0\n");
      pipeline->compute_shader =
          Shader::CreateTemporary(ShaderType::Compute, shader_header,
                                  Resources::GetDefaultResourcesPath() / "Shaders/Compute/DDGIProbeUpdate.slang");
      pipeline->descriptor_set_layouts.emplace_back(per_frame_layout_);
      pipeline->descriptor_set_layouts.emplace_back(ddgi_probe_update_layout_);
      auto& push_constant_range = pipeline->push_constant_ranges.emplace_back();
      push_constant_range.size = sizeof(DdgiProbeAtlasUpdatePushConstant);
      push_constant_range.offset = 0;
      push_constant_range.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
      pipeline->Initialize();
    };
    if (supported_variant != DdgiProbeUpdateVariant::Serial) {
      create_parallel_pipeline(ddgi_probe_update_irradiance_pipeline_, 1u);
      create_parallel_pipeline(ddgi_probe_update_visibility_pipeline_, 2u);
    } else {
      ddgi_probe_update_irradiance_pipeline_.reset();
      ddgi_probe_update_visibility_pipeline_.reset();
    }
    ddgi_probe_update_variant_ = ResolveDdgiProbeUpdateVariant(
        requested_variant, limits, 1u,
        ddgi_probe_update_irradiance_pipeline_ && ddgi_probe_update_irradiance_pipeline_->Initialized(),
        ddgi_probe_update_visibility_pipeline_ && ddgi_probe_update_visibility_pipeline_->Initialized());
    if (ddgi_probe_update_variant_ == DdgiProbeUpdateVariant::Serial) {
      ddgi_probe_update_irradiance_pipeline_.reset();
      ddgi_probe_update_visibility_pipeline_.reset();
    }
    ddgi_probe_update_path_reported_ = false;
    const auto variant_name = [](const DdgiProbeUpdateVariant variant) {
      switch (variant) {
        case DdgiProbeUpdateVariant::Serial:
          return "serial";
        case DdgiProbeUpdateVariant::ParallelShared:
          return "parallel-shared";
        default:
          return "parallel-direct";
      }
    };
    EVOENGINE_LOG("EVOENGINE_DDGI_PROBE_UPDATE_VARIANT requested=" + requested_name +
                  " selected=" + variant_name(ddgi_probe_update_variant_) +
                  " required_shared_bytes=" + std::to_string(kDdgiProbeUpdateSharedMemoryBytes))
  }
  if (!ddgi_probe_relocation_pipeline_) {
    ddgi_probe_relocation_pipeline_ = std::make_shared<ComputePipeline>();
    ddgi_probe_relocation_pipeline_->compute_shader =
        Shader::CreateTemporary(ShaderType::Compute, Platform::GetShaderGlobalDefines(),
                                Resources::GetDefaultResourcesPath() / "Shaders/Compute/DDGIProbeRelocation.slang");
    ddgi_probe_relocation_pipeline_->descriptor_set_layouts.emplace_back(ddgi_probe_relocation_layout_);
    auto& push_constant_range = ddgi_probe_relocation_pipeline_->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(DdgiProbeRelocationPushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
    ddgi_probe_relocation_pipeline_->Initialize();
  }
  if (!ddgi_probe_classification_pipeline_) {
    ddgi_probe_classification_pipeline_ = std::make_shared<ComputePipeline>();
    ddgi_probe_classification_pipeline_->compute_shader =
        Shader::CreateTemporary(ShaderType::Compute, Platform::GetShaderGlobalDefines(),
                                Resources::GetDefaultResourcesPath() / "Shaders/Compute/DDGIProbeClassification.slang");
    ddgi_probe_classification_pipeline_->descriptor_set_layouts.emplace_back(ddgi_probe_classification_layout_);
    auto& push_constant_range = ddgi_probe_classification_pipeline_->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(DdgiProbeClassificationPushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
    ddgi_probe_classification_pipeline_->Initialize();
  }
  if (!ddgi_probe_variability_reduce_pipeline_) {
    ddgi_probe_variability_reduce_pipeline_ = std::make_shared<ComputePipeline>();
    ddgi_probe_variability_reduce_pipeline_->compute_shader = Shader::CreateTemporary(
        ShaderType::Compute, Platform::GetShaderGlobalDefines(),
        Resources::GetDefaultResourcesPath() / "Shaders/Compute/DDGIProbeVariabilityReduce.slang");
    ddgi_probe_variability_reduce_pipeline_->descriptor_set_layouts.emplace_back(ddgi_probe_variability_layout_);
    auto& push_constant_range = ddgi_probe_variability_reduce_pipeline_->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(DdgiProbeVariabilityPushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
    ddgi_probe_variability_reduce_pipeline_->Initialize();
  }
  if (!ddgi_probe_variability_extra_reduce_pipeline_) {
    ddgi_probe_variability_extra_reduce_pipeline_ = std::make_shared<ComputePipeline>();
    ddgi_probe_variability_extra_reduce_pipeline_->compute_shader = Shader::CreateTemporary(
        ShaderType::Compute, Platform::GetShaderGlobalDefines(),
        Resources::GetDefaultResourcesPath() / "Shaders/Compute/DDGIProbeVariabilityExtraReduce.slang");
    ddgi_probe_variability_extra_reduce_pipeline_->descriptor_set_layouts.emplace_back(ddgi_probe_variability_layout_);
    auto& push_constant_range = ddgi_probe_variability_extra_reduce_pipeline_->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(DdgiProbeVariabilityPushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
    ddgi_probe_variability_extra_reduce_pipeline_->Initialize();
  }
#pragma region Graphics Pipelines
  const auto shadow_empty_fragment_shader_path =
      Resources::GetDefaultResourcesPath() / "Shaders/Graphics/Fragment/Empty.slang";
  if (!point_light_shadow_pipeline_normal_opaque) {
    point_light_shadow_pipeline_normal_opaque = CreateShadowVertexPipeline(
        Resources::GetDefaultResourcesPath() / "Shaders/Graphics/Vertex/Lighting/PointLightShadowMap.slang",
        shadow_empty_fragment_shader_path, GeometryType::Mesh, {per_frame_layout_});
  }
  if (Platform::GetInstance().GetCapabilities().support_mesh_shader && !point_light_shadow_pipeline_mesh_shader) {
    point_light_shadow_pipeline_mesh_shader = std::make_shared<GraphicsPipeline>();
    point_light_shadow_pipeline_mesh_shader->task_shader = Shader::CreateTemporary(
        ShaderType::Task, Platform::GetShaderGlobalDefines(),
        Resources::GetDefaultResourcesPath() / "Shaders/Graphics/Task/Lighting/PointLightShadowMap.slang");
    point_light_shadow_pipeline_mesh_shader->mesh_shader = Shader::CreateTemporary(
        ShaderType::Mesh, Platform::GetShaderGlobalDefines(),
        Resources::GetDefaultResourcesPath() / "Shaders/Graphics/Mesh/Lighting/PointLightShadowMap.slang");
    point_light_shadow_pipeline_mesh_shader->fragment_shader =
        Shader::CreateTemporary(ShaderType::Fragment, Platform::GetShaderGlobalDefines(),
                                Resources::GetDefaultResourcesPath() / "Shaders/Graphics/Fragment/Empty.slang");
    point_light_shadow_pipeline_mesh_shader->geometry_type = GeometryType::Mesh;
    point_light_shadow_pipeline_mesh_shader->descriptor_set_layouts.emplace_back(per_frame_layout_);
    point_light_shadow_pipeline_mesh_shader->descriptor_set_layouts.emplace_back(meshlet_layout_);
    point_light_shadow_pipeline_mesh_shader->depth_attachment_format = Platform::Constants::shadow_map;
    point_light_shadow_pipeline_mesh_shader->stencil_attachment_format = VK_FORMAT_UNDEFINED;
    auto& push_constant_range = point_light_shadow_pipeline_mesh_shader->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(RenderInstancePushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_ALL;
    point_light_shadow_pipeline_mesh_shader->Initialize();
  }
  if (!spot_light_shadow_pipeline_normal_opaque) {
    spot_light_shadow_pipeline_normal_opaque = CreateShadowVertexPipeline(
        Resources::GetDefaultResourcesPath() / "Shaders/Graphics/Vertex/Lighting/SpotLightShadowMap.slang",
        shadow_empty_fragment_shader_path, GeometryType::Mesh, {per_frame_layout_});
  }
  if (Platform::GetInstance().GetCapabilities().support_mesh_shader && !spot_light_shadow_pipeline_mesh_shader) {
    spot_light_shadow_pipeline_mesh_shader = std::make_shared<GraphicsPipeline>();
    spot_light_shadow_pipeline_mesh_shader->task_shader = Shader::CreateTemporary(
        ShaderType::Task, Platform::GetShaderGlobalDefines(),
        Resources::GetDefaultResourcesPath() / "Shaders/Graphics/Task/Lighting/SpotLightShadowMap.slang");
    spot_light_shadow_pipeline_mesh_shader->mesh_shader = Shader::CreateTemporary(
        ShaderType::Mesh, Platform::GetShaderGlobalDefines(),
        Resources::GetDefaultResourcesPath() / "Shaders/Graphics/Mesh/Lighting/SpotLightShadowMap.slang");
    spot_light_shadow_pipeline_mesh_shader->fragment_shader =
        Shader::CreateTemporary(ShaderType::Fragment, Platform::GetShaderGlobalDefines(),
                                Resources::GetDefaultResourcesPath() / "Shaders/Graphics/Fragment/Empty.slang");
    spot_light_shadow_pipeline_mesh_shader->geometry_type = GeometryType::Mesh;
    spot_light_shadow_pipeline_mesh_shader->descriptor_set_layouts.emplace_back(per_frame_layout_);
    spot_light_shadow_pipeline_mesh_shader->descriptor_set_layouts.emplace_back(meshlet_layout_);
    spot_light_shadow_pipeline_mesh_shader->depth_attachment_format = Platform::Constants::shadow_map;
    spot_light_shadow_pipeline_mesh_shader->stencil_attachment_format = VK_FORMAT_UNDEFINED;
    auto& push_constant_range = spot_light_shadow_pipeline_mesh_shader->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(RenderInstancePushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_ALL;
    spot_light_shadow_pipeline_mesh_shader->Initialize();
  }
  if (!directional_light_shadow_pipeline_normal_opaque) {
    directional_light_shadow_pipeline_normal_opaque = CreateShadowVertexPipeline(
        Resources::GetDefaultResourcesPath() / "Shaders/Graphics/Vertex/Lighting/DirectionalLightShadowMap.slang",
        shadow_empty_fragment_shader_path, GeometryType::Mesh, {per_frame_layout_});
  }
  if (Platform::GetInstance().GetCapabilities().support_mesh_shader && !directional_light_shadow_pipeline_mesh_shader) {
    directional_light_shadow_pipeline_mesh_shader = std::make_shared<GraphicsPipeline>();
    directional_light_shadow_pipeline_mesh_shader->task_shader = Shader::CreateTemporary(
        ShaderType::Task, Platform::GetShaderGlobalDefines(),
        Resources::GetDefaultResourcesPath() / "Shaders/Graphics/Task/Lighting/DirectionalLightShadowMap.slang");
    directional_light_shadow_pipeline_mesh_shader->mesh_shader = Shader::CreateTemporary(
        ShaderType::Mesh, Platform::GetShaderGlobalDefines(),
        Resources::GetDefaultResourcesPath() / "Shaders/Graphics/Mesh/Lighting/DirectionalLightShadowMap.slang");
    directional_light_shadow_pipeline_mesh_shader->fragment_shader =
        Shader::CreateTemporary(ShaderType::Fragment, Platform::GetShaderGlobalDefines(),
                                Resources::GetDefaultResourcesPath() / "Shaders/Graphics/Fragment/Empty.slang");
    directional_light_shadow_pipeline_mesh_shader->geometry_type = GeometryType::Mesh;
    directional_light_shadow_pipeline_mesh_shader->descriptor_set_layouts.emplace_back(per_frame_layout_);
    directional_light_shadow_pipeline_mesh_shader->descriptor_set_layouts.emplace_back(meshlet_layout_);
    directional_light_shadow_pipeline_mesh_shader->depth_attachment_format = Platform::Constants::shadow_map;
    directional_light_shadow_pipeline_mesh_shader->stencil_attachment_format = VK_FORMAT_UNDEFINED;
    auto& push_constant_range = directional_light_shadow_pipeline_mesh_shader->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(RenderInstancePushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_ALL;
    directional_light_shadow_pipeline_mesh_shader->Initialize();
  }
  if (Platform::MeshShaderEnabled() && !strands_directional_light_shadow_pipeline) {
    strands_directional_light_shadow_pipeline = CreateStrandShadowMeshPipeline(
        Resources::GetDefaultResourcesPath() / "Shaders/Graphics/Mesh/Lighting/DirectionalLightStrandsShadowMap.slang",
        per_frame_layout_, strand_meshlet_layout_);
  }
  if (!instanced_point_light_shadow_pipeline_opaque) {
    instanced_point_light_shadow_pipeline_opaque = CreateShadowVertexPipeline(
        Resources::GetDefaultResourcesPath() / "Shaders/Graphics/Vertex/Lighting/PointLightShadowMapInstanced.slang",
        shadow_empty_fragment_shader_path, GeometryType::Mesh, {per_frame_layout_, particle_instanced_data_layout_});
  }
  if (!instanced_spot_light_shadow_pipeline_opaque) {
    instanced_spot_light_shadow_pipeline_opaque = CreateShadowVertexPipeline(
        Resources::GetDefaultResourcesPath() / "Shaders/Graphics/Vertex/Lighting/SpotLightShadowMapInstanced.slang",
        shadow_empty_fragment_shader_path, GeometryType::Mesh, {per_frame_layout_, particle_instanced_data_layout_});
  }
  if (!instanced_directional_light_shadow_pipeline_opaque) {
    instanced_directional_light_shadow_pipeline_opaque = CreateShadowVertexPipeline(
        Resources::GetDefaultResourcesPath() /
            "Shaders/Graphics/Vertex/Lighting/DirectionalLightShadowMapInstanced.slang",
        shadow_empty_fragment_shader_path, GeometryType::Mesh, {per_frame_layout_, particle_instanced_data_layout_});
  }
  if (!skinned_point_light_shadow_pipeline_opaque) {
    skinned_point_light_shadow_pipeline_opaque = CreateShadowVertexPipeline(
        Resources::GetDefaultResourcesPath() / "Shaders/Graphics/Vertex/Lighting/PointLightShadowMapSkinned.slang",
        shadow_empty_fragment_shader_path, GeometryType::SkinnedMesh, {per_frame_layout_, bone_matrices_layout_});
  }
  if (!skinned_spot_light_shadow_pipeline_opaque) {
    skinned_spot_light_shadow_pipeline_opaque = CreateShadowVertexPipeline(
        Resources::GetDefaultResourcesPath() / "Shaders/Graphics/Vertex/Lighting/SpotLightShadowMapSkinned.slang",
        shadow_empty_fragment_shader_path, GeometryType::SkinnedMesh, {per_frame_layout_, bone_matrices_layout_});
  }
  if (!skinned_directional_light_shadow_pipeline_opaque) {
    skinned_directional_light_shadow_pipeline_opaque = CreateShadowVertexPipeline(
        Resources::GetDefaultResourcesPath() /
            "Shaders/Graphics/Vertex/Lighting/DirectionalLightShadowMapSkinned.slang",
        shadow_empty_fragment_shader_path, GeometryType::SkinnedMesh, {per_frame_layout_, bone_matrices_layout_});
  }
  if (Platform::MeshShaderEnabled() && !strands_point_light_shadow_pipeline) {
    strands_point_light_shadow_pipeline = CreateStrandShadowMeshPipeline(
        Resources::GetDefaultResourcesPath() / "Shaders/Graphics/Mesh/Lighting/PointLightStrandsShadowMap.slang",
        per_frame_layout_, strand_meshlet_layout_);
  }
  if (Platform::MeshShaderEnabled() && !strands_spot_light_shadow_pipeline) {
    strands_spot_light_shadow_pipeline = CreateStrandShadowMeshPipeline(
        Resources::GetDefaultResourcesPath() / "Shaders/Graphics/Mesh/Lighting/SpotLightStrandsShadowMap.slang",
        per_frame_layout_, strand_meshlet_layout_);
  }
  if (!deferred_prepass_pipeline_normal) {
    deferred_prepass_pipeline_normal = std::make_shared<GraphicsPipeline>();
    deferred_prepass_pipeline_normal->vertex_shader = Shader::CreateTemporary(
        ShaderType::Vertex, CreateRasterNoBindlessTextureShaderDefines(),
        Resources::GetDefaultResourcesPath() / "Shaders/Graphics/Vertex/Standard/Standard.slang");
    deferred_prepass_pipeline_normal->fragment_shader = Shader::CreateTemporary(
        ShaderType::Fragment, CreateRasterMaterialNoBindlessShaderDefines(),
        Resources::GetDefaultResourcesPath() / "Shaders/Graphics/Fragment/Standard/StandardDeferred.slang");
    deferred_prepass_pipeline_normal->geometry_type = GeometryType::Mesh;
    deferred_prepass_pipeline_normal->descriptor_set_layouts.emplace_back(raster_material_per_frame_layout_);
    deferred_prepass_pipeline_normal->descriptor_set_layouts.emplace_back(empty_descriptor_set_layout_);
    deferred_prepass_pipeline_normal->descriptor_set_layouts.emplace_back(empty_descriptor_set_layout_);
    deferred_prepass_pipeline_normal->descriptor_set_layouts.emplace_back(raster_material_layout_);
    deferred_prepass_pipeline_normal->depth_attachment_format = Platform::Constants::render_texture_depth;
    deferred_prepass_pipeline_normal->stencil_attachment_format = VK_FORMAT_UNDEFINED;
    deferred_prepass_pipeline_normal->color_attachment_formats = CreateDeferredGBufferColorAttachmentFormats();
    auto& push_constant_range = deferred_prepass_pipeline_normal->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(RenderInstancePushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_ALL;
    deferred_prepass_pipeline_normal->Initialize();
  }
  if (Platform::GetInstance().GetCapabilities().support_mesh_shader && !deferred_prepass_pipeline_mesh) {
    deferred_prepass_pipeline_mesh = std::make_shared<GraphicsPipeline>();
    deferred_prepass_pipeline_mesh->task_shader =
        Shader::CreateTemporary(ShaderType::Task, CreateRasterNoBindlessTextureShaderDefines(),
                                Resources::GetDefaultResourcesPath() / "Shaders/Graphics/Task/Standard/Standard.slang");
    deferred_prepass_pipeline_mesh->mesh_shader =
        Shader::CreateTemporary(ShaderType::Mesh, CreateRasterNoBindlessTextureShaderDefines(),
                                Resources::GetDefaultResourcesPath() / "Shaders/Graphics/Mesh/Standard/Standard.slang");
    deferred_prepass_pipeline_mesh->fragment_shader = Shader::CreateTemporary(
        ShaderType::Fragment, CreateRasterMaterialNoBindlessShaderDefines(),
        Resources::GetDefaultResourcesPath() / "Shaders/Graphics/Fragment/Standard/StandardDeferred.slang");
    deferred_prepass_pipeline_mesh->geometry_type = GeometryType::Mesh;
    deferred_prepass_pipeline_mesh->descriptor_set_layouts.emplace_back(raster_material_per_frame_layout_);
    deferred_prepass_pipeline_mesh->descriptor_set_layouts.emplace_back(meshlet_layout_);
    deferred_prepass_pipeline_mesh->descriptor_set_layouts.emplace_back(empty_descriptor_set_layout_);
    deferred_prepass_pipeline_mesh->descriptor_set_layouts.emplace_back(raster_material_layout_);
    deferred_prepass_pipeline_mesh->depth_attachment_format = Platform::Constants::render_texture_depth;
    deferred_prepass_pipeline_mesh->stencil_attachment_format = VK_FORMAT_UNDEFINED;
    deferred_prepass_pipeline_mesh->color_attachment_formats = CreateDeferredGBufferColorAttachmentFormats();
    auto& push_constant_range = deferred_prepass_pipeline_mesh->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(RenderInstancePushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_ALL;
    deferred_prepass_pipeline_mesh->Initialize();
  }
  if (!instanced_deferred_prepass_pipeline) {
    instanced_deferred_prepass_pipeline = std::make_shared<GraphicsPipeline>();
    instanced_deferred_prepass_pipeline->vertex_shader = Shader::CreateTemporary(
        ShaderType::Vertex, CreateRasterNoBindlessTextureShaderDefines(),
        Resources::GetDefaultResourcesPath() / "Shaders/Graphics/Vertex/Standard/StandardInstanced.slang");
    instanced_deferred_prepass_pipeline->fragment_shader = Shader::CreateTemporary(
        ShaderType::Fragment, CreateRasterMaterialNoBindlessShaderDefines(),
        Resources::GetDefaultResourcesPath() / "Shaders/Graphics/Fragment/Standard/StandardDeferred.slang");
    instanced_deferred_prepass_pipeline->geometry_type = GeometryType::Mesh;
    instanced_deferred_prepass_pipeline->descriptor_set_layouts.emplace_back(raster_material_per_frame_layout_);
    instanced_deferred_prepass_pipeline->descriptor_set_layouts.emplace_back(particle_instanced_data_layout_);
    instanced_deferred_prepass_pipeline->descriptor_set_layouts.emplace_back(empty_descriptor_set_layout_);
    instanced_deferred_prepass_pipeline->descriptor_set_layouts.emplace_back(raster_material_layout_);
    instanced_deferred_prepass_pipeline->depth_attachment_format = Platform::Constants::render_texture_depth;
    instanced_deferred_prepass_pipeline->stencil_attachment_format = VK_FORMAT_UNDEFINED;
    instanced_deferred_prepass_pipeline->color_attachment_formats = CreateDeferredGBufferColorAttachmentFormats();
    auto& push_constant_range = instanced_deferred_prepass_pipeline->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(RenderInstancePushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_ALL;
    instanced_deferred_prepass_pipeline->Initialize();
  }
  if (!skinned_deferred_prepass_pipeline) {
    skinned_deferred_prepass_pipeline = std::make_shared<GraphicsPipeline>();
    skinned_deferred_prepass_pipeline->vertex_shader = Shader::CreateTemporary(
        ShaderType::Vertex, CreateRasterNoBindlessTextureShaderDefines(),
        Resources::GetDefaultResourcesPath() / "Shaders/Graphics/Vertex/Standard/StandardSkinned.slang");
    skinned_deferred_prepass_pipeline->fragment_shader = Shader::CreateTemporary(
        ShaderType::Fragment, CreateRasterMaterialNoBindlessShaderDefines(),
        Resources::GetDefaultResourcesPath() / "Shaders/Graphics/Fragment/Standard/StandardDeferred.slang");
    skinned_deferred_prepass_pipeline->geometry_type = GeometryType::SkinnedMesh;
    skinned_deferred_prepass_pipeline->descriptor_set_layouts.emplace_back(raster_material_per_frame_layout_);
    skinned_deferred_prepass_pipeline->descriptor_set_layouts.emplace_back(bone_matrices_layout_);
    skinned_deferred_prepass_pipeline->descriptor_set_layouts.emplace_back(empty_descriptor_set_layout_);
    skinned_deferred_prepass_pipeline->descriptor_set_layouts.emplace_back(raster_material_layout_);
    skinned_deferred_prepass_pipeline->depth_attachment_format = Platform::Constants::render_texture_depth;
    skinned_deferred_prepass_pipeline->stencil_attachment_format = VK_FORMAT_UNDEFINED;
    skinned_deferred_prepass_pipeline->color_attachment_formats = CreateDeferredGBufferColorAttachmentFormats();
    auto& push_constant_range = skinned_deferred_prepass_pipeline->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(RenderInstancePushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_ALL;
    skinned_deferred_prepass_pipeline->Initialize();
  }
  if (!skinned_motion_vectors_pipeline_) {
    skinned_motion_vectors_pipeline_ = std::make_shared<GraphicsPipeline>();
    skinned_motion_vectors_pipeline_->vertex_shader = Shader::CreateTemporary(
        ShaderType::Vertex, CreateRasterNoBindlessTextureShaderDefines(),
        Resources::GetDefaultResourcesPath() / "Shaders/Graphics/Vertex/Standard/SkinnedMotionVectors.slang");
    skinned_motion_vectors_pipeline_->fragment_shader = Shader::CreateTemporary(
        ShaderType::Fragment, CreateRasterMaterialNoBindlessShaderDefines(),
        Resources::GetDefaultResourcesPath() / "Shaders/Graphics/Fragment/Standard/SkinnedMotionVectors.slang");
    skinned_motion_vectors_pipeline_->geometry_type = GeometryType::SkinnedMesh;
    skinned_motion_vectors_pipeline_->vertex_input_attribute_set = VertexInputAttributeSet::MotionVectors;
    skinned_motion_vectors_pipeline_->descriptor_set_layouts.emplace_back(raster_material_per_frame_layout_);
    skinned_motion_vectors_pipeline_->descriptor_set_layouts.emplace_back(bone_matrices_layout_);
    skinned_motion_vectors_pipeline_->descriptor_set_layouts.emplace_back(motion_coverage_layout_);
    skinned_motion_vectors_pipeline_->descriptor_set_layouts.emplace_back(raster_material_layout_);
    skinned_motion_vectors_pipeline_->depth_attachment_format = Platform::Constants::render_texture_depth;
    skinned_motion_vectors_pipeline_->stencil_attachment_format = VK_FORMAT_UNDEFINED;
    skinned_motion_vectors_pipeline_->color_attachment_formats = {VK_FORMAT_R16G16B16A16_SFLOAT};
    auto& push_constant_range = skinned_motion_vectors_pipeline_->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(RenderInstancePushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_ALL;
    skinned_motion_vectors_pipeline_->Initialize();
  }
  if (!transparent_motion_vectors_pipeline_) {
    transparent_motion_vectors_pipeline_ = std::make_shared<GraphicsPipeline>();
    transparent_motion_vectors_pipeline_->vertex_shader = Shader::CreateTemporary(
        ShaderType::Vertex, CreateRasterNoBindlessTextureShaderDefines(),
        Resources::GetDefaultResourcesPath() / "Shaders/Graphics/Vertex/Standard/TransparentMotionVectors.slang");
    transparent_motion_vectors_pipeline_->fragment_shader = Shader::CreateTemporary(
        ShaderType::Fragment, CreateRasterMaterialNoBindlessShaderDefines(),
        Resources::GetDefaultResourcesPath() / "Shaders/Graphics/Fragment/Standard/TransparentMotionVectors.slang");
    transparent_motion_vectors_pipeline_->geometry_type = GeometryType::Mesh;
    transparent_motion_vectors_pipeline_->vertex_input_attribute_set = VertexInputAttributeSet::MotionVectors;
    transparent_motion_vectors_pipeline_->descriptor_set_layouts.emplace_back(raster_material_per_frame_layout_);
    transparent_motion_vectors_pipeline_->descriptor_set_layouts.emplace_back(empty_descriptor_set_layout_);
    transparent_motion_vectors_pipeline_->descriptor_set_layouts.emplace_back(motion_coverage_layout_);
    transparent_motion_vectors_pipeline_->descriptor_set_layouts.emplace_back(raster_material_layout_);
    transparent_motion_vectors_pipeline_->depth_attachment_format = Platform::Constants::render_texture_depth;
    transparent_motion_vectors_pipeline_->stencil_attachment_format = VK_FORMAT_UNDEFINED;
    transparent_motion_vectors_pipeline_->color_attachment_formats = {VK_FORMAT_R16G16B16A16_SFLOAT};
    auto& push_constant_range = transparent_motion_vectors_pipeline_->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(RenderInstancePushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_ALL;
    transparent_motion_vectors_pipeline_->Initialize();
  }
  if (Platform::MeshShaderEnabled() && !strands_deferred_prepass_pipeline) {
    strands_deferred_prepass_pipeline = std::make_shared<GraphicsPipeline>();
    strands_deferred_prepass_pipeline->task_shader = Shader::CreateTemporary(
        ShaderType::Task, CreateRasterNoBindlessTextureShaderDefines(),
        Resources::GetDefaultResourcesPath() / "Shaders/Graphics/Task/Standard/StandardStrands.slang");
    strands_deferred_prepass_pipeline->mesh_shader = Shader::CreateTemporary(
        ShaderType::Mesh, CreateRasterNoBindlessTextureShaderDefines(),
        Resources::GetDefaultResourcesPath() / "Shaders/Graphics/Mesh/Standard/StandardStrands.slang");
    strands_deferred_prepass_pipeline->fragment_shader = Shader::CreateTemporary(
        ShaderType::Fragment, CreateRasterMaterialNoBindlessShaderDefines(),
        Resources::GetDefaultResourcesPath() / "Shaders/Graphics/Fragment/Standard/StandardDeferred.slang");
    strands_deferred_prepass_pipeline->vertex_input_enabled = false;
    strands_deferred_prepass_pipeline->descriptor_set_layouts.emplace_back(raster_material_per_frame_layout_);
    strands_deferred_prepass_pipeline->descriptor_set_layouts.emplace_back(strand_meshlet_layout_);
    strands_deferred_prepass_pipeline->descriptor_set_layouts.emplace_back(empty_descriptor_set_layout_);
    strands_deferred_prepass_pipeline->descriptor_set_layouts.emplace_back(raster_material_layout_);
    strands_deferred_prepass_pipeline->depth_attachment_format = Platform::Constants::render_texture_depth;
    strands_deferred_prepass_pipeline->stencil_attachment_format = VK_FORMAT_UNDEFINED;
    strands_deferred_prepass_pipeline->color_attachment_formats = CreateDeferredGBufferColorAttachmentFormats();
    auto& push_constant_range = strands_deferred_prepass_pipeline->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(RenderInstancePushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_ALL;
    strands_deferred_prepass_pipeline->Initialize();
  }
  if (!deferred_lighting_pass_pipeline) {
    deferred_lighting_pass_pipeline = std::make_shared<GraphicsPipeline>();
    deferred_lighting_pass_pipeline->vertex_shader = Shader::CreateTemporary(
        ShaderType::Vertex, Resources::GetDefaultResourcesPath() / "Shaders/Graphics/Vertex/TexturePassThrough.slang");
    deferred_lighting_pass_pipeline->fragment_shader = Shader::CreateTemporary(
        ShaderType::Fragment, CreateRasterFixedLightingShaderDefines(3),
        Resources::GetDefaultResourcesPath() / "Shaders/Graphics/Fragment/Standard/StandardDeferredLighting.slang");
    deferred_lighting_pass_pipeline->geometry_type = GeometryType::Mesh;
    deferred_lighting_pass_pipeline->vertex_input_attribute_set = VertexInputAttributeSet::PositionTexCoord;
    deferred_lighting_pass_pipeline->descriptor_set_layouts.emplace_back(raster_material_per_frame_layout_);
    deferred_lighting_pass_pipeline->descriptor_set_layouts.emplace_back(camera_g_buffer_layout_);
    deferred_lighting_pass_pipeline->descriptor_set_layouts.emplace_back(lighting_layout_);
    deferred_lighting_pass_pipeline->descriptor_set_layouts.emplace_back(raster_lighting_texture_layout_);
    deferred_lighting_pass_pipeline->depth_attachment_format = Platform::Constants::render_texture_depth;
    deferred_lighting_pass_pipeline->stencil_attachment_format = VK_FORMAT_UNDEFINED;
    deferred_lighting_pass_pipeline->color_attachment_formats = {1, Platform::Constants::render_texture_color};
    auto& push_constant_range = deferred_lighting_pass_pipeline->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(RenderInstancePushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_ALL;
    deferred_lighting_pass_pipeline->Initialize();
  }
  if (!ddgi_gather_timing_pipeline_) {
    ddgi_gather_timing_pipeline_ = std::make_shared<GraphicsPipeline>();
    ddgi_gather_timing_pipeline_->vertex_shader = Shader::CreateTemporary(
        ShaderType::Vertex, Resources::GetDefaultResourcesPath() / "Shaders/Graphics/Vertex/TexturePassThrough.slang");
    ddgi_gather_timing_pipeline_->fragment_shader = Shader::CreateTemporary(
        ShaderType::Fragment, CreateRasterFixedLightingShaderDefines(3),
        Resources::GetDefaultResourcesPath() / "Shaders/Graphics/Fragment/Standard/DDGIGatherTiming.slang");
    ddgi_gather_timing_pipeline_->geometry_type = GeometryType::Mesh;
    ddgi_gather_timing_pipeline_->vertex_input_attribute_set = VertexInputAttributeSet::PositionTexCoord;
    ddgi_gather_timing_pipeline_->descriptor_set_layouts.emplace_back(raster_material_per_frame_layout_);
    ddgi_gather_timing_pipeline_->descriptor_set_layouts.emplace_back(camera_g_buffer_layout_);
    ddgi_gather_timing_pipeline_->descriptor_set_layouts.emplace_back(lighting_layout_);
    ddgi_gather_timing_pipeline_->descriptor_set_layouts.emplace_back(raster_lighting_texture_layout_);
    ddgi_gather_timing_pipeline_->depth_attachment_format = VK_FORMAT_UNDEFINED;
    ddgi_gather_timing_pipeline_->stencil_attachment_format = VK_FORMAT_UNDEFINED;
    ddgi_gather_timing_pipeline_->color_attachment_formats = {VK_FORMAT_R16G16B16A16_SFLOAT};
    auto& push_constant_range = ddgi_gather_timing_pipeline_->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(RenderInstancePushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_ALL;
    ddgi_gather_timing_pipeline_->Initialize();
  }
  if (!deferred_lighting_pass_pipeline_scene_camera) {
    deferred_lighting_pass_pipeline_scene_camera = std::make_shared<GraphicsPipeline>();
    deferred_lighting_pass_pipeline_scene_camera->vertex_shader = Shader::CreateTemporary(
        ShaderType::Vertex, Resources::GetDefaultResourcesPath() / "Shaders/Graphics/Vertex/TexturePassThrough.slang");
    deferred_lighting_pass_pipeline_scene_camera->fragment_shader =
        Shader::CreateTemporary(ShaderType::Fragment, CreateRasterFixedLightingShaderDefines(3),
                                Resources::GetDefaultResourcesPath() /
                                    "Shaders/Graphics/Fragment/Standard/StandardDeferredLightingSceneCamera.slang");
    deferred_lighting_pass_pipeline_scene_camera->geometry_type = GeometryType::Mesh;
    deferred_lighting_pass_pipeline_scene_camera->vertex_input_attribute_set =
        VertexInputAttributeSet::PositionTexCoord;
    deferred_lighting_pass_pipeline_scene_camera->descriptor_set_layouts.emplace_back(
        raster_material_per_frame_layout_);
    deferred_lighting_pass_pipeline_scene_camera->descriptor_set_layouts.emplace_back(camera_g_buffer_layout_);
    deferred_lighting_pass_pipeline_scene_camera->descriptor_set_layouts.emplace_back(lighting_layout_);
    deferred_lighting_pass_pipeline_scene_camera->descriptor_set_layouts.emplace_back(raster_lighting_texture_layout_);
    deferred_lighting_pass_pipeline_scene_camera->depth_attachment_format = Platform::Constants::render_texture_depth;
    deferred_lighting_pass_pipeline_scene_camera->stencil_attachment_format = VK_FORMAT_UNDEFINED;
    deferred_lighting_pass_pipeline_scene_camera->color_attachment_formats = {
        1, Platform::Constants::render_texture_color};
    auto& push_constant_range = deferred_lighting_pass_pipeline_scene_camera->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(RenderInstancePushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_ALL;
    deferred_lighting_pass_pipeline_scene_camera->Initialize();
  }
  if (!transparent_geometry_pipeline_normal) {
    transparent_geometry_pipeline_normal = std::make_shared<GraphicsPipeline>();
    transparent_geometry_pipeline_normal->vertex_shader = Shader::CreateTemporary(
        ShaderType::Vertex, CreateRasterNoBindlessTextureShaderDefines(),
        Resources::GetDefaultResourcesPath() / "Shaders/Graphics/Vertex/Standard/Standard.slang");
    transparent_geometry_pipeline_normal->fragment_shader = Shader::CreateTemporary(
        ShaderType::Fragment, CreateRasterMaterialFixedLightingShaderDefines(4),
        Resources::GetDefaultResourcesPath() / "Shaders/Graphics/Fragment/Standard/StandardTransparent.slang");
    transparent_geometry_pipeline_normal->geometry_type = GeometryType::Mesh;
    transparent_geometry_pipeline_normal->descriptor_set_layouts.emplace_back(raster_material_per_frame_layout_);
    transparent_geometry_pipeline_normal->descriptor_set_layouts.emplace_back(empty_descriptor_set_layout_);
    transparent_geometry_pipeline_normal->descriptor_set_layouts.emplace_back(lighting_layout_);
    transparent_geometry_pipeline_normal->descriptor_set_layouts.emplace_back(raster_material_layout_);
    transparent_geometry_pipeline_normal->descriptor_set_layouts.emplace_back(raster_lighting_texture_layout_);
    transparent_geometry_pipeline_normal->depth_attachment_format = Platform::Constants::render_texture_depth;
    transparent_geometry_pipeline_normal->stencil_attachment_format = VK_FORMAT_UNDEFINED;
    transparent_geometry_pipeline_normal->color_attachment_formats = {1, Platform::Constants::render_texture_color};
    auto& push_constant_range = transparent_geometry_pipeline_normal->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(RenderInstancePushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_ALL;
    transparent_geometry_pipeline_normal->Initialize();
  }
  if (!gizmos) {
    gizmos = std::make_shared<GraphicsPipeline>();
    gizmos->vertex_shader =
        Shader::CreateTemporary(ShaderType::Vertex, Platform::GetShaderGlobalDefines(),
                                Resources::GetDefaultResourcesPath() / "Shaders/Graphics/Vertex/Gizmos/Gizmos.slang");
    gizmos->fragment_shader =
        Shader::CreateTemporary(ShaderType::Fragment, Platform::GetShaderGlobalDefines(),
                                Resources::GetDefaultResourcesPath() / "Shaders/Graphics/Fragment/Gizmos/Gizmos.slang");
    gizmos->geometry_type = GeometryType::Mesh;
    gizmos->vertex_input_attribute_set = VertexInputAttributeSet::Position;
    gizmos->depth_attachment_format = Platform::Constants::render_texture_depth;
    gizmos->stencil_attachment_format = VK_FORMAT_UNDEFINED;
    gizmos->color_attachment_formats = {1, Platform::Constants::render_texture_color};
    gizmos->descriptor_set_layouts.emplace_back(per_frame_layout_);
    auto& push_constant_range = gizmos->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(GizmosPushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_ALL;

    gizmos->Initialize();
  }
  if (!gizmos_normal_colored) {
    gizmos_normal_colored = std::make_shared<GraphicsPipeline>();
    gizmos_normal_colored->vertex_shader = Shader::CreateTemporary(
        ShaderType::Vertex, Platform::GetShaderGlobalDefines(),
        Resources::GetDefaultResourcesPath() / "Shaders/Graphics/Vertex/Gizmos/GizmosNormalColored.slang");
    gizmos_normal_colored->fragment_shader = Shader::CreateTemporary(
        ShaderType::Fragment, Platform::GetShaderGlobalDefines(),
        Resources::GetDefaultResourcesPath() / "Shaders/Graphics/Fragment/Gizmos/GizmosColored.slang");
    gizmos_normal_colored->geometry_type = GeometryType::Mesh;
    gizmos_normal_colored->vertex_input_attribute_set = VertexInputAttributeSet::PositionNormal;
    gizmos_normal_colored->depth_attachment_format = Platform::Constants::render_texture_depth;
    gizmos_normal_colored->stencil_attachment_format = VK_FORMAT_UNDEFINED;
    gizmos_normal_colored->color_attachment_formats = {1, Platform::Constants::render_texture_color};
    gizmos_normal_colored->descriptor_set_layouts.emplace_back(per_frame_layout_);
    gizmos_normal_colored->tessellation_patch_control_points = 4;
    auto& push_constant_range = gizmos_normal_colored->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(GizmosPushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_ALL;
    gizmos_normal_colored->Initialize();
  }
  if (!gizmos_vertex_colored) {
    gizmos_vertex_colored = std::make_shared<GraphicsPipeline>();
    gizmos_vertex_colored->vertex_shader = Shader::CreateTemporary(
        ShaderType::Vertex, Platform::GetShaderGlobalDefines(),
        Resources::GetDefaultResourcesPath() / "Shaders/Graphics/Vertex/Gizmos/GizmosVertexColored.slang");
    gizmos_vertex_colored->fragment_shader = Shader::CreateTemporary(
        ShaderType::Fragment, Platform::GetShaderGlobalDefines(),
        Resources::GetDefaultResourcesPath() / "Shaders/Graphics/Fragment/Gizmos/GizmosColored.slang");
    gizmos_vertex_colored->geometry_type = GeometryType::Mesh;
    gizmos_vertex_colored->vertex_input_attribute_set = VertexInputAttributeSet::PositionColor;
    gizmos_vertex_colored->depth_attachment_format = Platform::Constants::render_texture_depth;
    gizmos_vertex_colored->stencil_attachment_format = VK_FORMAT_UNDEFINED;
    gizmos_vertex_colored->color_attachment_formats = {1, Platform::Constants::render_texture_color};
    gizmos_vertex_colored->descriptor_set_layouts.emplace_back(per_frame_layout_);
    auto& push_constant_range = gizmos_vertex_colored->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(GizmosPushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_ALL;
    gizmos_vertex_colored->Initialize();
  }
  if (!gizmos_instanced_colored) {
    gizmos_instanced_colored = std::make_shared<GraphicsPipeline>();
    gizmos_instanced_colored->vertex_shader = Shader::CreateTemporary(
        ShaderType::Vertex, Platform::GetShaderGlobalDefines(),
        Resources::GetDefaultResourcesPath() / "Shaders/Graphics/Vertex/Gizmos/GizmosInstancedColored.slang");
    gizmos_instanced_colored->fragment_shader = Shader::CreateTemporary(
        ShaderType::Fragment, Platform::GetShaderGlobalDefines(),
        Resources::GetDefaultResourcesPath() / "Shaders/Graphics/Fragment/Gizmos/GizmosColored.slang");
    gizmos_instanced_colored->geometry_type = GeometryType::Mesh;
    gizmos_instanced_colored->vertex_input_attribute_set = VertexInputAttributeSet::Position;
    gizmos_instanced_colored->depth_attachment_format = Platform::Constants::render_texture_depth;
    gizmos_instanced_colored->stencil_attachment_format = VK_FORMAT_UNDEFINED;
    gizmos_instanced_colored->color_attachment_formats = {1, Platform::Constants::render_texture_color};
    gizmos_instanced_colored->descriptor_set_layouts.emplace_back(per_frame_layout_);
    gizmos_instanced_colored->descriptor_set_layouts.emplace_back(particle_instanced_data_layout_);
    auto& push_constant_range = gizmos_instanced_colored->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(GizmosPushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_ALL;

    gizmos_instanced_colored->Initialize();
  }
  if (!ddgi_probe_visualization_pipeline_) {
    ddgi_probe_visualization_pipeline_ = std::make_shared<GraphicsPipeline>();
    ddgi_probe_visualization_pipeline_->vertex_shader = Shader::CreateTemporary(
        ShaderType::Vertex, Platform::GetShaderGlobalDefines(),
        Resources::GetDefaultResourcesPath() / "Shaders/Graphics/Vertex/DDGI/DDGIProbeVisualization.slang");
    ddgi_probe_visualization_pipeline_->fragment_shader = Shader::CreateTemporary(
        ShaderType::Fragment, Platform::GetShaderGlobalDefines(),
        Resources::GetDefaultResourcesPath() / "Shaders/Graphics/Fragment/DDGI/DDGIProbeVisualization.slang");
    ddgi_probe_visualization_pipeline_->geometry_type = GeometryType::Mesh;
    ddgi_probe_visualization_pipeline_->vertex_input_attribute_set = VertexInputAttributeSet::PositionNormal;
    ddgi_probe_visualization_pipeline_->depth_attachment_format = Platform::Constants::render_texture_depth;
    ddgi_probe_visualization_pipeline_->stencil_attachment_format = VK_FORMAT_UNDEFINED;
    ddgi_probe_visualization_pipeline_->color_attachment_formats = {1, Platform::Constants::render_texture_color};
    ddgi_probe_visualization_pipeline_->descriptor_set_layouts.emplace_back(per_frame_layout_);
    ddgi_probe_visualization_pipeline_->descriptor_set_layouts.emplace_back(ddgi_probe_visualization_layout_);
    auto& push_constant_range = ddgi_probe_visualization_pipeline_->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(DdgiProbeVisualizationPushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT;
    ddgi_probe_visualization_pipeline_->Initialize();
  }
  if (!ddgi_probe_ray_visualization_pipeline_) {
    ddgi_probe_ray_visualization_pipeline_ = std::make_shared<GraphicsPipeline>();
    ddgi_probe_ray_visualization_pipeline_->vertex_shader = Shader::CreateTemporary(
        ShaderType::Vertex, Platform::GetShaderGlobalDefines(),
        Resources::GetDefaultResourcesPath() / "Shaders/Graphics/Vertex/DDGI/DDGIProbeRayVisualization.slang");
    ddgi_probe_ray_visualization_pipeline_->fragment_shader = Shader::CreateTemporary(
        ShaderType::Fragment, Platform::GetShaderGlobalDefines(),
        Resources::GetDefaultResourcesPath() / "Shaders/Graphics/Fragment/DDGI/DDGIProbeRayVisualization.slang");
    ddgi_probe_ray_visualization_pipeline_->geometry_type = GeometryType::Mesh;
    ddgi_probe_ray_visualization_pipeline_->vertex_input_enabled = false;
    ddgi_probe_ray_visualization_pipeline_->primitive_topology = VK_PRIMITIVE_TOPOLOGY_LINE_LIST;
    ddgi_probe_ray_visualization_pipeline_->depth_attachment_format = Platform::Constants::render_texture_depth;
    ddgi_probe_ray_visualization_pipeline_->stencil_attachment_format = VK_FORMAT_UNDEFINED;
    ddgi_probe_ray_visualization_pipeline_->color_attachment_formats = {1, Platform::Constants::render_texture_color};
    ddgi_probe_ray_visualization_pipeline_->descriptor_set_layouts.emplace_back(per_frame_layout_);
    ddgi_probe_ray_visualization_pipeline_->descriptor_set_layouts.emplace_back(ddgi_probe_ray_visualization_layout_);
    auto& push_constant_range = ddgi_probe_ray_visualization_pipeline_->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(DdgiProbeRayVisualizationPushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT;
    ddgi_probe_ray_visualization_pipeline_->Initialize();
  }
  if (!gaussian_splat_pipeline_) {
    gaussian_splat_pipeline_ = CreateGaussianSplatPipeline(per_frame_layout_, gaussian_splat_layout_,
                                                           Platform::Constants::render_texture_depth);
  }
  if (!gaussian_splat_overlay_pipeline_) {
    gaussian_splat_overlay_pipeline_ =
        CreateGaussianSplatPipeline(per_frame_layout_, gaussian_splat_layout_, VK_FORMAT_UNDEFINED);
  }
  if (Platform::MeshShaderEnabled()) {
    if (!gaussian_splat_mesh_pipeline_) {
      gaussian_splat_mesh_pipeline_ = CreateGaussianSplatPipeline(per_frame_layout_, gaussian_splat_layout_,
                                                                  Platform::Constants::render_texture_depth, true);
    }
    if (!gaussian_splat_mesh_overlay_pipeline_) {
      gaussian_splat_mesh_overlay_pipeline_ =
          CreateGaussianSplatPipeline(per_frame_layout_, gaussian_splat_layout_, VK_FORMAT_UNDEFINED, true);
    }
  }
  if (Platform::MeshShaderEnabled()) {
    const auto create_gizmo_strands_pipeline = [&](const std::string& mesh_define,
                                                   const std::filesystem::path& fragment_shader_path) {
      auto pipeline = std::make_shared<GraphicsPipeline>();
      pipeline->task_shader = Shader::CreateTemporary(
          ShaderType::Task, Platform::GetShaderGlobalDefines(),
          Resources::GetDefaultResourcesPath() / "Shaders/Graphics/Task/Gizmos/GizmosStrands.slang");
      pipeline->mesh_shader = Shader::CreateTemporary(
          ShaderType::Mesh, Platform::GetShaderGlobalDefines() + mesh_define,
          Resources::GetDefaultResourcesPath() / "Shaders/Graphics/Mesh/Gizmos/GizmosStrands.slang");
      pipeline->fragment_shader =
          Shader::CreateTemporary(ShaderType::Fragment, Platform::GetShaderGlobalDefines(), fragment_shader_path);
      pipeline->vertex_input_enabled = false;
      pipeline->depth_attachment_format = Platform::Constants::render_texture_depth;
      pipeline->stencil_attachment_format = VK_FORMAT_UNDEFINED;
      pipeline->color_attachment_formats = {1, Platform::Constants::render_texture_color};
      pipeline->descriptor_set_layouts = {per_frame_layout_, strand_meshlet_layout_};
      auto& push_constant_range = pipeline->push_constant_ranges.emplace_back();
      push_constant_range.size = sizeof(GizmosPushConstant);
      push_constant_range.offset = 0;
      push_constant_range.stageFlags = VK_SHADER_STAGE_ALL;
      pipeline->Initialize();
      return pipeline;
    };
    const auto gizmos_fragment_path =
        Resources::GetDefaultResourcesPath() / "Shaders/Graphics/Fragment/Gizmos/Gizmos.slang";
    const auto colored_fragment_path =
        Resources::GetDefaultResourcesPath() / "Shaders/Graphics/Fragment/Gizmos/GizmosColored.slang";
    if (!gizmos_strands) {
      gizmos_strands = create_gizmo_strands_pipeline({}, gizmos_fragment_path);
    }
    if (!gizmos_strands_normal_colored) {
      gizmos_strands_normal_colored =
          create_gizmo_strands_pipeline("#define EE_GIZMO_STRAND_NORMAL_COLOR 1\n", colored_fragment_path);
    }
    if (!gizmos_strands_vertex_colored) {
      gizmos_strands_vertex_colored =
          create_gizmo_strands_pipeline("#define EE_GIZMO_STRAND_VERTEX_COLOR 1\n", colored_fragment_path);
    }
  }
#pragma endregion
#pragma region Ray Tracing Pipelines
  constexpr auto ray_tracing_push_constant_stages = VK_SHADER_STAGE_RAYGEN_BIT_KHR | VK_SHADER_STAGE_MISS_BIT_KHR |
                                                    VK_SHADER_STAGE_CLOSEST_HIT_BIT_KHR |
                                                    VK_SHADER_STAGE_ANY_HIT_BIT_KHR;
  if (Platform::RayTracingEnabled() && !ray_tracing_camera_fallback_pipeline_) {
    ray_tracing_camera_fallback_pipeline_ = CreateRayTracingCameraPipeline(
        per_frame_layout_, ray_tracing_layout_, ray_tracing_camera_output_layout_, Platform::GetShaderGlobalDefines());
    ray_tracing_camera_pipeline = ray_tracing_camera_fallback_pipeline_;
  }
  if (Platform::RayQueryEnabled() && !ray_query_camera_fallback_pipeline_) {
    ray_query_camera_fallback_pipeline_ = CreateRayQueryCameraPipeline(
        per_frame_layout_, ray_tracing_layout_, ray_tracing_camera_output_layout_, Platform::GetShaderGlobalDefines());
    ray_query_camera_pipeline_ = ray_query_camera_fallback_pipeline_;
  }
  if (!ray_camera_shader_variant_cache_ &&
      (ray_tracing_camera_fallback_pipeline_ || ray_query_camera_fallback_pipeline_)) {
    const auto ray_tracing_factory =
        ray_tracing_camera_fallback_pipeline_
            ? RayCameraShaderVariantCache::RayTracingFactory(
                  [per_frame_layout = per_frame_layout_, ray_tracing_layout = ray_tracing_layout_,
                   camera_output_layout = ray_tracing_camera_output_layout_,
                   miss_shader = ray_tracing_camera_fallback_pipeline_->miss_shader,
                   closest_hit_shader =
                       ray_tracing_camera_fallback_pipeline_->closest_hit_shader](const uint32_t feature_mask) {
                    return CreateRayTracingCameraPipeline(per_frame_layout, ray_tracing_layout, camera_output_layout,
                                                          CameraVariantShaderHeader(feature_mask), miss_shader,
                                                          closest_hit_shader);
                  })
            : RayCameraShaderVariantCache::RayTracingFactory{};
    const auto ray_query_factory =
        ray_query_camera_fallback_pipeline_
            ? RayCameraShaderVariantCache::RayQueryFactory(
                  [per_frame_layout = per_frame_layout_, ray_tracing_layout = ray_tracing_layout_,
                   camera_output_layout = ray_tracing_camera_output_layout_](const uint32_t feature_mask) {
                    return CreateRayQueryCameraPipeline(per_frame_layout, ray_tracing_layout, camera_output_layout,
                                                        CameraVariantShaderHeader(feature_mask));
                  })
            : RayCameraShaderVariantCache::RayQueryFactory{};
    ray_camera_shader_variant_cache_ = std::make_shared<RayCameraShaderVariantCache>(
        ray_tracing_camera_fallback_pipeline_, ray_query_camera_fallback_pipeline_, ray_tracing_factory,
        ray_query_factory);
  }
  if (Platform::RayTracingEnabled() && !ray_tracing_point_cloud_pipeline) {
    ray_tracing_point_cloud_pipeline = std::make_shared<RayTracingPipeline>();
    ray_tracing_point_cloud_pipeline->raygen_shader =
        Shader::CreateTemporary(ShaderType::RayGen, Platform::GetShaderGlobalDefines(),
                                Resources::GetDefaultResourcesPath() / "Shaders/RayTracing/RayGen/PointCloud.slang");
    ray_tracing_point_cloud_pipeline->miss_shader =
        Shader::CreateTemporary(ShaderType::Miss, Platform::GetShaderGlobalDefines(),
                                Resources::GetDefaultResourcesPath() / "Shaders/RayTracing/Miss/PointCloud.slang");
    ray_tracing_point_cloud_pipeline->closest_hit_shader = Shader::CreateTemporary(
        ShaderType::ClosestHit, Platform::GetShaderGlobalDefines(),
        Resources::GetDefaultResourcesPath() / "Shaders/RayTracing/ClosestHit/PointCloud.slang");
    ray_tracing_point_cloud_pipeline->descriptor_set_layouts.emplace_back(per_frame_layout_);
    ray_tracing_point_cloud_pipeline->descriptor_set_layouts.emplace_back(ray_tracing_layout_);
    ray_tracing_point_cloud_pipeline->descriptor_set_layouts.emplace_back(ray_tracing_point_cloud_layout_);
    ray_tracing_point_cloud_pipeline->Initialize();
  }
  if (Platform::RayTracingEnabled() && !ddgi_probe_ray_diagnostic_pipeline_) {
    ddgi_probe_ray_diagnostic_pipeline_ = std::make_shared<RayTracingPipeline>();
    ddgi_probe_ray_diagnostic_pipeline_->raygen_shader = Shader::CreateTemporary(
        ShaderType::RayGen, Platform::GetShaderGlobalDefines(),
        Resources::GetDefaultResourcesPath() / "Shaders/RayTracing/RayGen/DDGIProbeDiagnostics.slang");
    ddgi_probe_ray_diagnostic_pipeline_->miss_shader = Shader::CreateTemporary(
        ShaderType::Miss, Platform::GetShaderGlobalDefines(),
        Resources::GetDefaultResourcesPath() / "Shaders/RayTracing/Miss/DDGIProbeDiagnostics.slang");
    ddgi_probe_ray_diagnostic_pipeline_->closest_hit_shader = Shader::CreateTemporary(
        ShaderType::ClosestHit, Platform::GetShaderGlobalDefines(),
        Resources::GetDefaultResourcesPath() / "Shaders/RayTracing/ClosestHit/DDGIProbeDiagnostics.slang");
    ddgi_probe_ray_diagnostic_pipeline_->any_hit_shader = Shader::CreateTemporary(
        ShaderType::AnyHit, Platform::GetShaderGlobalDefines(),
        Resources::GetDefaultResourcesPath() / "Shaders/RayTracing/AnyHit/DDGIProbeDiagnostics.slang");
    ddgi_probe_ray_diagnostic_pipeline_->descriptor_set_layouts.emplace_back(per_frame_layout_);
    ddgi_probe_ray_diagnostic_pipeline_->descriptor_set_layouts.emplace_back(ray_tracing_layout_);
    ddgi_probe_ray_diagnostic_pipeline_->descriptor_set_layouts.emplace_back(ddgi_probe_ray_output_layout_);
    auto& push_constant_range = ddgi_probe_ray_diagnostic_pipeline_->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(DdgiProbeRayTracingPushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = ray_tracing_push_constant_stages;
    ddgi_probe_ray_diagnostic_pipeline_->Initialize();
  }
#pragma endregion

  const auto max_frames_in_flight = Platform::GetMaxFramesInFlight();
  initialize_render_instance_storage();
  kernel_descriptor_buffers_.clear();
  VkBufferCreateInfo buffer_create_info{};
  buffer_create_info.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
  buffer_create_info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
  VmaAllocationCreateInfo buffer_vma_allocation_create_info{};
  buffer_vma_allocation_create_info.usage = VMA_MEMORY_USAGE_AUTO_PREFER_DEVICE;
  const auto max_frame_in_flight = Platform::GetMaxFramesInFlight();
  for (size_t i = 0; i < max_frame_in_flight; i++) {
    buffer_create_info.usage = VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT;
    buffer_create_info.size = sizeof(glm::vec4) * Platform::Constants::max_kernel_amount * 2;
    kernel_descriptor_buffers_.emplace_back(
        std::make_shared<Buffer>(buffer_create_info, buffer_vma_allocation_create_info));
  }
  per_frame_descriptor_sets_.clear();
  for (size_t i = 0; i < max_frames_in_flight; i++) {
    auto descriptor_set = std::make_shared<DescriptorSet>(per_frame_layout_);
    per_frame_descriptor_sets_.emplace_back(descriptor_set);
  }

  raster_material_per_frame_descriptor_sets_.clear();
  for (size_t i = 0; i < max_frames_in_flight; i++) {
    auto descriptor_set = std::make_shared<DescriptorSet>(raster_material_per_frame_layout_);
    raster_material_per_frame_descriptor_sets_.emplace_back(descriptor_set);
  }

  raster_lighting_texture_descriptor_sets_.clear();
  raster_lighting_texture_descriptor_sets_.resize(max_frames_in_flight);

  meshlet_descriptor_sets_.clear();
  for (size_t i = 0; i < max_frames_in_flight; i++) {
    auto descriptor_set = std::make_shared<DescriptorSet>(meshlet_layout_);
    meshlet_descriptor_sets_.emplace_back(descriptor_set);
  }

  strand_meshlet_descriptor_sets_.clear();
  if (strand_meshlet_layout_) {
    for (size_t i = 0; i < max_frames_in_flight; i++) {
      strand_meshlet_descriptor_sets_.emplace_back(std::make_shared<DescriptorSet>(strand_meshlet_layout_));
    }
  }

  ray_tracing_descriptor_sets_.clear();
  if (Platform::RayAccelerationStructureEnabled()) {
    for (size_t i = 0; i < max_frames_in_flight; i++) {
      auto descriptor_set = std::make_shared<DescriptorSet>(ray_tracing_layout_);
      ray_tracing_descriptor_sets_.emplace_back(descriptor_set);
    }
  }

  std::vector<glm::vec4> kernels;
  for (uint32_t i = 0; i < Platform::Constants::max_kernel_amount; i++) {
    kernels.emplace_back(glm::ballRand(1.0f), 1.0f);
  }
  for (uint32_t i = 0; i < Platform::Constants::max_kernel_amount; i++) {
    kernels.emplace_back(glm::gaussRand(0.0f, 1.0f), glm::gaussRand(0.0f, 1.0f), glm::gaussRand(0.0f, 1.0f),
                         glm::gaussRand(0.0f, 1.0f));
  }
  for (int i = 0; i < Platform::GetMaxFramesInFlight(); i++) {
    kernel_descriptor_buffers_[i]->UploadVector(kernels);
  }
  PrepareEnvironmentalBrdfLut();
  lighting_ = std::make_unique<Lighting>();
  lighting_->Initialize();
  log_startup(true);
}

void RenderLayer::EnsureRasterMaterialFallbackTextures() const {
  const auto create_fallback_texture = [](const glm::vec4& color) {
    auto texture = AssetManager::CreateTemporaryAsset<Texture2D>();
    texture->SetRgbaChannelData({color}, {1, 1}, false);
    texture->UnsafeUploadDataImmediately();
    return texture;
  };
  if (!raster_material_white_fallback_texture_) {
    raster_material_white_fallback_texture_ = create_fallback_texture(glm::vec4(1.0f));
  }
  if (!raster_material_black_fallback_texture_) {
    raster_material_black_fallback_texture_ = create_fallback_texture(glm::vec4(0.0f, 0.0f, 0.0f, 1.0f));
  }
  if (!raster_material_flat_normal_fallback_texture_) {
    raster_material_flat_normal_fallback_texture_ = create_fallback_texture(glm::vec4(0.5f, 0.5f, 1.0f, 1.0f));
  }
}

std::array<VkDescriptorImageInfo, RenderInstanceStorage::kRasterMaterialTextureSlotCount>
RenderLayer::GetRasterMaterialFallbackDescriptorImageInfos() const {
  EnsureRasterMaterialFallbackTextures();
  std::array<VkDescriptorImageInfo, RenderInstanceStorage::kRasterMaterialTextureSlotCount> image_infos{};
  TextureStorage::TryGetTexture2DDescriptorImageInfo(raster_material_white_fallback_texture_->GetTextureStorageIndex(),
                                                     image_infos[0]);
  TextureStorage::TryGetTexture2DDescriptorImageInfo(raster_material_white_fallback_texture_->GetTextureStorageIndex(),
                                                     image_infos[1]);
  TextureStorage::TryGetTexture2DDescriptorImageInfo(
      raster_material_flat_normal_fallback_texture_->GetTextureStorageIndex(), image_infos[2]);
  TextureStorage::TryGetTexture2DDescriptorImageInfo(raster_material_black_fallback_texture_->GetTextureStorageIndex(),
                                                     image_infos[3]);
  TextureStorage::TryGetTexture2DDescriptorImageInfo(raster_material_white_fallback_texture_->GetTextureStorageIndex(),
                                                     image_infos[4]);
  TextureStorage::TryGetTexture2DDescriptorImageInfo(raster_material_white_fallback_texture_->GetTextureStorageIndex(),
                                                     image_infos[5]);
  TextureStorage::TryGetTexture2DDescriptorImageInfo(raster_material_white_fallback_texture_->GetTextureStorageIndex(),
                                                     image_infos[6]);
  TextureStorage::TryGetTexture2DDescriptorImageInfo(
      raster_material_flat_normal_fallback_texture_->GetTextureStorageIndex(), image_infos[7]);
  return image_infos;
}

void RenderLayer::ClearAllEditorCameras() const {
  const auto scene = GetScene();
  if (!scene)
    return;
  std::vector<std::pair<GlobalTransform, std::shared_ptr<Camera>>> cameras;
  RenderInstanceStorage::CollectEditorCameras(scene, cameras);

  Platform::RecordCommandsMainQueue([&](const VkCommandBuffer vk_command_buffer) {
    for (const auto& i : cameras) {
      if (i.second->prev_global_transform_ != i.first.value) {
        i.second->frame_count_ = 0;
        i.second->InvalidateRayCameraHistory();
        i.second->prev_global_transform_ = i.first.value;
      }
      if ((i.second->camera_render_mode == Camera::CameraRenderMode::Rasterization && i.second->rendered_) ||
          (Camera::IsRayCameraRenderMode(i.second->camera_render_mode) && i.second->frame_count_ == 0)) {
        if (const auto render_texture = i.second->GetRenderTexture()) {
          render_texture->Clear(vk_command_buffer);
        }
      }
    }
  });
}

RenderLayer::DdgiSettings& RenderLayer::GetDdgiSettings() {
  const auto scene = GetScene();
  if (const auto lighting = GetAssignedEnvironmentalLighting(scene)) {
    return lighting->ddgi_settings;
  }
  return fallback_ddgi_settings_;
}

const RenderLayer::DdgiSettings& RenderLayer::GetDdgiSettings() const {
  const auto scene = GetScene();
  if (const auto lighting = GetAssignedEnvironmentalLighting(scene)) {
    return lighting->ddgi_settings;
  }
  return fallback_ddgi_settings_;
}

glm::ivec3 RenderLayer::GetDdgiProbeScrollOffset() const {
  const auto* runtime = GetPrimaryDdgiVolumeRuntimeState();
  return runtime ? runtime->probe_scroll_offset : glm::ivec3(0);
}

glm::ivec3 RenderLayer::GetDdgiLastProbeScrollDelta() const {
  const auto* runtime = GetPrimaryDdgiVolumeRuntimeState();
  return runtime ? runtime->last_probe_scroll_delta : glm::ivec3(0);
}

RenderLayer::DdgiPerformanceStats RenderLayer::GetDdgiLastPerformanceStats() const {
  return ddgi_last_performance_stats_;
}

std::vector<RenderLayer::DdgiVolumeRuntimeStats> RenderLayer::GetDdgiVolumeRuntimeStats() const {
  std::vector<DdgiVolumeRuntimeStats> stats;
  stats.reserve(ddgi_ordered_volume_ids_.size());
  for (const auto volume_id : ddgi_ordered_volume_ids_) {
    const auto runtime = ddgi_volume_runtime_states_.find(volume_id);
    if (runtime == ddgi_volume_runtime_states_.end()) {
      continue;
    }
    const auto& state = *runtime->second;
    const bool resources_ready =
        state.frame_resource_layout.valid &&
        std::all_of(state.resource_ids.begin(), state.resource_ids.end(),
                    [](const auto id) {
                      return id != 0u;
                    }) &&
        state.probe_metadata_buffer &&
        state.probe_metadata_buffer->GetSize() >= state.frame_resource_layout.probe_metadata_byte_size &&
        state.probe_state_buffer &&
        state.probe_state_buffer->GetSize() >= state.frame_resource_layout.probe_state_byte_size &&
        state.irradiance_atlas && state.visibility_atlas && state.variability_atlas;
    stats.emplace_back(DdgiVolumeRuntimeStats{
        state.stable_entity_id, state.sorted_index, state.artist_priority, state.probe_density,
        state.previous_probe_counts, state.frame_resource_layout.valid ? state.frame_resource_layout.probe_count : 0u,
        state.probe_scroll_offset, state.last_probe_scroll_delta, state.has_valid_probe_history,
        state.contributes_lighting, resources_ready, state.emissive_mesh_sampling_enabled, state.resource_ids});
  }
  return stats;
}

uint32_t RenderLayer::GetDdgiLastProbeUpdateReasons() const {
  const auto* runtime = GetPrimaryDdgiVolumeRuntimeState();
  return runtime ? runtime->last_probe_update_reasons : DdgiUpdateReasonNone;
}

std::string RenderLayer::GetDdgiLastProbeUpdateReasonText() const {
  return FormatDdgiUpdateReasons(GetDdgiLastProbeUpdateReasons());
}

void RenderLayer::ResetDdgiRuntimeFrameState(DdgiVolumeRuntimeState& runtime_state) {
  runtime_state.frame_trace_probe_rays = false;
  runtime_state.frame_clear_scrolled_probes = false;
  runtime_state.frame_ray_push_constant = {};
  runtime_state.frame_probe_scroll_push_constant = {};
  runtime_state.frame_probe_update_push_constant = {};
  runtime_state.frame_probe_relocation_reset_push_constant = {};
  runtime_state.frame_probe_relocation_update_push_constant = {};
  runtime_state.frame_probe_classification_reset_push_constant = {};
  runtime_state.frame_probe_classification_update_push_constant = {};
  runtime_state.clear_probe_atlas_this_frame = false;
  runtime_state.frame_probe_relocation_reset = false;
  runtime_state.frame_probe_relocation_enabled = false;
  runtime_state.frame_probe_classification_reset = false;
  runtime_state.frame_probe_classification_enabled = false;
  runtime_state.frame_probe_variability_enabled = false;
  runtime_state.frame_probe_warmup_frame_index = 0u;
  runtime_state.frame_probe_warmup_frame_count = 0u;
  runtime_state.frame_probe_warmup_active = false;
  runtime_state.frame_probe_update_hysteresis = 0.0f;
  runtime_state.frame_variability_readback_generation = 0u;
  runtime_state.last_probe_update_reasons = DdgiUpdateReasonNone;
  runtime_state.frame_selected_probe_ray_sample_count = 0u;
  runtime_state.frame_selected_probe_ray_logical_index = 0u;
  runtime_state.frame_selected_probe_ray_physical_index = 0u;
}

uint64_t RenderLayer::NextDdgiResourceId() {
  if (++next_ddgi_resource_id_ == 0u) {
    ++next_ddgi_resource_id_;
  }
  return next_ddgi_resource_id_;
}

const RenderLayer::DdgiVolumeRuntimeState* RenderLayer::GetPrimaryDdgiVolumeRuntimeState() const {
  if (ddgi_ordered_volume_ids_.empty()) {
    return nullptr;
  }
  const auto runtime = ddgi_volume_runtime_states_.find(ddgi_ordered_volume_ids_.front());
  return runtime != ddgi_volume_runtime_states_.end() ? runtime->second.get() : nullptr;
}

RenderLayer::DdgiProbeDebugDataView RenderLayer::GetDdgiProbeDebugData(const bool refresh_readback) {
  if (ddgi_ordered_volume_ids_.empty()) {
    return {};
  }
  const auto runtime_entry = ddgi_volume_runtime_states_.find(ddgi_ordered_volume_ids_.front());
  if (runtime_entry == ddgi_volume_runtime_states_.end()) {
    return {};
  }
  auto& runtime = *runtime_entry->second;
  if (refresh_readback) {
    const auto readback_ready = [](DdgiReadbackTicket& ticket) {
      if (!ticket.buffer || !ticket.submission || ticket.generation == 0u ||
          ticket.generation <= ticket.consumed_generation) {
        return false;
      }
      if (ticket.submission->status == FrameSubmissionState::Status::Pending) {
        Platform::WaitForFrameSubmission(ticket.frame_index, "DDGI Debug Readback Fence Wait");
      }
      if (ticket.submission->status == FrameSubmissionState::Status::Discarded) {
        ticket.consumed_generation = ticket.generation;
        return false;
      }
      return ticket.submission->status == FrameSubmissionState::Status::Submitted;
    };
    auto& metadata_ticket = runtime.metadata_readback_ticket;
    if (readback_ready(metadata_ticket)) {
      if (metadata_ticket.byte_size != 0u && metadata_ticket.byte_size % sizeof(glm::vec4) == 0u &&
          metadata_ticket.buffer->GetSize() >= metadata_ticket.byte_size) {
        metadata_ticket.buffer->DownloadVector(runtime.probe_debug_metadata,
                                               static_cast<size_t>(metadata_ticket.byte_size / sizeof(glm::vec4)));
      }
      metadata_ticket.consumed_generation = metadata_ticket.generation;
    }
    auto& ray_ticket = runtime.ray_readback_ticket;
    if (readback_ready(ray_ticket)) {
      if (ray_ticket.element_count != 0u &&
          ray_ticket.byte_size == ray_ticket.element_count * sizeof(PointCloudSample) &&
          ray_ticket.buffer->GetSize() >= ray_ticket.byte_size) {
        ray_ticket.buffer->DownloadVector(runtime.probe_debug_ray_samples, ray_ticket.element_count);
        runtime.probe_debug_ray_probe_index = ray_ticket.logical_probe_index;
        runtime.probe_debug_ray_physical_probe_index = ray_ticket.physical_probe_index;
      }
      ray_ticket.consumed_generation = ray_ticket.generation;
    }
  }
  return {&runtime.probe_debug_metadata,
          &runtime.probe_debug_ray_samples,
          static_cast<uint32_t>(runtime.probe_debug_metadata.size() / 3u),
          runtime.probe_debug_ray_probe_index,
          runtime.probe_debug_ray_physical_probe_index,
          static_cast<uint32_t>(runtime.probe_debug_ray_samples.size()),
          !runtime.probe_debug_ray_samples.empty()};
}

void RenderLayer::ClearAllCameras() const {
  const auto scene = GetScene();
  if (!scene)
    return;
  std::vector<std::pair<GlobalTransform, std::shared_ptr<Camera>>> cameras;
  RenderInstanceStorage::CollectCameras(scene, cameras);

  Platform::RecordCommandsMainQueue([&](const VkCommandBuffer vk_command_buffer) {
    for (const auto& i : cameras) {
      if (i.second->prev_global_transform_ != i.first.value) {
        i.second->frame_count_ = 0;
        i.second->InvalidateRayCameraHistory();
        i.second->prev_global_transform_ = i.first.value;
      }
      if ((i.second->camera_render_mode == Camera::CameraRenderMode::Rasterization && i.second->rendered_) ||
          (Camera::IsRayCameraRenderMode(i.second->camera_render_mode) && i.second->frame_count_ == 0)) {
        if (const auto render_texture = i.second->GetRenderTexture()) {
          render_texture->Clear(vk_command_buffer);
        }
      }
    }
  });
}

void RenderLayer::PrepareForRendering() {
  const ProfilerScope profiler_scope("RenderLayer::PrepareForRendering", "Render");
  const auto scene = GetScene();
  PrepareSceneForRendering(scene);
}

void RenderLayer::PrepareSceneForRendering(const std::shared_ptr<Scene>& scene, const bool include_editor_cameras,
                                           const bool update_editor_selection, const bool update_ray_tracing,
                                           const bool track_ddgi_scene_inputs,
                                           const std::pair<GlobalTransform, std::shared_ptr<Camera>>* injected_camera,
                                           const bool include_reflection_probes) {
  if (!scene)
    return;
  const ProfilerScope profiler_scope("RenderLayer::PrepareSceneForRendering", "Render");
  const auto current_frame_index = Platform::GetCurrentFrameIndex();
  Platform::ResetRenderPassDrawStats(current_frame_index);
  const auto current_render_instances = render_instances_list_[current_frame_index];
  if (update_editor_selection) {
    ApplyAnimators();
  }
  if (GeometryStorage::HasPendingMeshUploads()) {
    Platform::WaitForFrameSubmissions("Required Geometry Upload Fence Wait");
    const auto geometry_upload_wait_start = std::chrono::steady_clock::now();
    GeometryStorage::WaitForPendingUploads();
    Platform::RecordCpuTimingSample(
        "Required Geometry Upload Wait",
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - geometry_upload_wait_start)
            .count());
  }
  const bool render_instance_updated =
      UpdateRenderInstanceStorage(scene, current_frame_index, include_editor_cameras, update_editor_selection,
                                  track_ddgi_scene_inputs, injected_camera, include_reflection_probes);

  const bool update_ray_tracing_resources = update_ray_tracing && Platform::RayAccelerationStructureEnabled();
  if (update_ray_tracing_resources) {
    if (ray_camera_shader_variant_cache_) {
      bool need_ray_tracing = false;
      bool need_ray_query = false;
      bool need_ray_tracing_debug_views = false;
      bool need_ray_query_debug_views = false;
      for (const auto& [transform, camera] : current_render_instances->cameras) {
        if (!camera)
          continue;
        const auto mode = Camera::ResolveCameraRenderMode(camera->camera_render_mode);
        need_ray_tracing |= mode == Camera::CameraRenderMode::RayTracing;
        need_ray_query |= mode == Camera::CameraRenderMode::RayQuery;
        const bool debug_views = camera->camera_settings.ray_debug_view != CameraSettings::RayDebugView::Beauty;
        need_ray_tracing_debug_views |= mode == Camera::CameraRenderMode::RayTracing && debug_views;
        need_ray_query_debug_views |= mode == Camera::CameraRenderMode::RayQuery && debug_views;
      }
      auto feature_mask = force_full_ray_camera_shader_variant
                              ? kGltfSceneAllFeatures
                              : DetectGltfSceneFeatures(current_render_instances->GetGltfShadeMaterials(),
                                                        current_render_instances->GetGltfTextureInfos());
      const auto variant_update =
          ray_camera_shader_variant_cache_->Update(feature_mask, need_ray_tracing, need_ray_query,
                                                   need_ray_tracing_debug_views || force_full_ray_camera_shader_variant,
                                                   need_ray_query_debug_views || force_full_ray_camera_shader_variant);
      ray_tracing_camera_pipeline = ray_camera_shader_variant_cache_->GetRayTracingPipeline();
      ray_query_camera_pipeline_ = ray_camera_shader_variant_cache_->GetRayQueryPipeline();
      for (const auto& [transform, camera] : current_render_instances->cameras) {
        if (!camera)
          continue;
        const auto mode = Camera::ResolveCameraRenderMode(camera->camera_render_mode);
        if ((variant_update.ray_tracing_activated && mode == Camera::CameraRenderMode::RayTracing) ||
            (variant_update.ray_query_activated && mode == Camera::CameraRenderMode::RayQuery)) {
          camera->ResetFrameCount();
        }
      }
    }
    current_render_instances->UpdateTopLevelAccelerationStructure();

    if (current_render_instances->mesh_top_level_acceleration_structure) {
      ray_tracing_descriptor_sets_[current_frame_index]->UpdateBufferDescriptorBinding(
          0, GeometryStorage::GetVertexBuffer());
      ray_tracing_descriptor_sets_[current_frame_index]->UpdateBufferDescriptorBinding(
          1, GeometryStorage::GetTriangleBuffer());
      ray_tracing_descriptor_sets_[current_frame_index]->UpdateAccelerationStructureDescriptorBinding(
          2, current_render_instances->mesh_top_level_acceleration_structure);
    }
  }
  if (track_ddgi_scene_inputs) {
    PrepareDdgiFrameState(scene, current_render_instances);
  }
  current_render_instances->BuildPreviousInstanceInfoBlocks(GetPreviousRenderInstanceStorage());
  current_render_instances->Upload();
  BindRenderInstanceStorage(current_frame_index, current_render_instances);
}

void RenderLayer::PrepareDdgiFrameState(const std::shared_ptr<Scene>& scene,
                                        const std::shared_ptr<RenderInstanceStorage>& render_instances) {
  const auto resolved_lighting = ResolveEnvironmentalLighting(scene);
  auto ddgi_settings = resolved_lighting.ddgi_settings;
  const auto assigned_lighting = GetAssignedEnvironmentalLighting(scene);
  if (!render_instances) {
    return;
  }
  if (ddgi_runtime_scene_.lock().get() != scene.get()) {
    ddgi_volume_runtime_states_.clear();
    ddgi_ordered_volume_ids_.clear();
    ddgi_volume_set_validation_error_.clear();
    ddgi_runtime_scene_ = scene;
  }

  render_instances->render_info_block.ddgi_volume_header = glm::uvec4(0u);
  render_instances->render_info_block.ddgi_volumes = {};
  const auto reject_volume_set = [&](const std::string& error) {
    if (error != ddgi_volume_set_validation_error_) {
      EVOENGINE_ERROR("DDGI volume set rejected atomically: " + error)
      ddgi_volume_set_validation_error_ = error;
    }
    for (auto& [stable_entity_id, runtime] : ddgi_volume_runtime_states_) {
      (void)stable_entity_id;
      if (runtime) {
        ResetDdgiRuntimeFrameState(*runtime);
      }
    }
    if (!ddgi_volume_runtime_states_.empty()) {
      if (const auto previous_render_instances = GetPreviousRenderInstanceStorage()) {
        PreserveDdgiRenderInfo(render_instances->render_info_block, previous_render_instances->render_info_block);
      }
    }
  };

  std::vector<DdgiVolumeRuntimeInfo> infos;
  infos.reserve(resolved_lighting.ddgi_volumes.size());
  for (const auto& volume : resolved_lighting.ddgi_volumes) {
    const auto source = CreateDdgiProbeRayDiagnosticSourceFromResolvedVolume(volume, ddgi_settings);
    auto& info = infos.emplace_back();
    info.sorted_index = static_cast<uint32_t>(infos.size() - 1u);
    info.stable_entity_id = volume.stable_id;
    info.artist_priority = volume.artist_priority;
    info.probe_counts = source.probe_counts;
    info.probe_count = GetDdgiProbeCount(source.probe_counts);
    info.first_probe = source.first_probe;
    info.probe_step_x = source.probe_step_x;
    info.probe_step_y = source.probe_step_y;
    info.probe_step_z = source.probe_step_z;
    info.probe_density = CalculateDdgiProbeDensity(source.probe_step_x, source.probe_step_y, source.probe_step_z);
  }
  const auto configured_probe_limit =
      ddgi_settings.storage.max_probe_count > 0 ? static_cast<uint32_t>(ddgi_settings.storage.max_probe_count) : 0u;
  const auto validation = ValidateDdgiVolumeSet(infos, configured_probe_limit);
  if (!validation.valid) {
    reject_volume_set(validation.error);
    return;
  }
  const auto selected_physical_device = Platform::GetSelectedPhysicalDevice();
  const auto max_image_dimension_2d =
      selected_physical_device ? selected_physical_device->properties.limits.maxImageDimension2D : 0u;
  const auto max_storage_buffer_range =
      selected_physical_device ? selected_physical_device->properties.limits.maxStorageBufferRange : 0u;
  std::vector<DdgiFrameResourceLayout> preflight_layouts;
  preflight_layouts.reserve(infos.size());
  for (const auto& info : infos) {
    auto& layout = preflight_layouts.emplace_back(CalculateDdgiFrameResourceLayout(
        ddgi_settings, info.probe_count, max_image_dimension_2d, max_storage_buffer_range));
    if (!layout.valid) {
      reject_volume_set("volume " + std::to_string(info.stable_entity_id) + ": " + layout.error);
      return;
    }
  }
  ddgi_volume_set_validation_error_.clear();

  std::vector<uint64_t> current_ids;
  current_ids.reserve(infos.size());
  for (const auto& info : infos) {
    current_ids.push_back(info.stable_entity_id);
  }
  for (auto it = ddgi_volume_runtime_states_.begin(); it != ddgi_volume_runtime_states_.end();) {
    if (std::find(current_ids.begin(), current_ids.end(), it->first) == current_ids.end()) {
      it = ddgi_volume_runtime_states_.erase(it);
    } else {
      ++it;
    }
  }
  ddgi_ordered_volume_ids_ = current_ids;

  const bool reset_probe_history = ddgi_settings.runtime.reset_probe_history;
  const auto prepare_runtime = [&](const DdgiVolumeRuntimeInfo& info,
                                   const ResolvedEnvironmentalLighting::DdgiVolume& volume,
                                   const DdgiFrameResourceLayout& preflight_layout) {
    auto& runtime = ddgi_volume_runtime_states_[info.stable_entity_id];
    if (!runtime) {
      runtime = std::make_unique<DdgiVolumeRuntimeState>();
    }
    runtime->stable_entity_id = info.stable_entity_id;
    runtime->sorted_index = info.sorted_index;
    runtime->artist_priority = info.artist_priority;
    runtime->probe_density = info.probe_density;
    runtime->latched_scene_change_triggers |= ddgi_latched_scene_change_triggers_;
    runtime->latched_scene_geometry_changed |= ddgi_latched_scene_geometry_changed_;
    runtime->deferred_scene_readiness_refresh |= ddgi_deferred_scene_readiness_refresh_;
    runtime->manual_reset_pending |= reset_probe_history;
    PrepareDdgiVolumeFrameState(scene, render_instances, *runtime, volume, ddgi_settings, preflight_layout,
                                info.sorted_index, runtime->manual_reset_pending);
    if (runtime->frame_trace_probe_rays) {
      runtime->manual_reset_pending = false;
    }

    auto source = CreateDdgiProbeRayDiagnosticSourceFromResolvedVolume(volume, ddgi_settings);
    source.selected_volume_index = info.sorted_index;
    if (runtime->has_previous_ray_source) {
      source.probe_counts = runtime->previous_probe_counts;
      source.probe_step_x = runtime->previous_probe_step_x;
      source.probe_step_y = runtime->previous_probe_step_y;
      source.probe_step_z = runtime->previous_probe_step_z;
      source.first_probe =
          runtime->previous_movement_type == static_cast<int>(DdgiVolumeMovementType::Scrolling)
              ? CalculateDdgiEffectiveFirstProbe(runtime->probe_scroll_base_first_probe, runtime->previous_probe_step_x,
                                                 runtime->previous_probe_step_y, runtime->previous_probe_step_z,
                                                 runtime->probe_scroll_offset)
              : runtime->previous_first_probe;
    }
    const auto& layout = runtime->frame_resource_layout;
    auto& gpu_info = runtime->gpu_info;
    gpu_info = {};
    gpu_info.first_probe = glm::vec4(source.first_probe, 0.0f);
    gpu_info.probe_step_x = glm::vec4(source.probe_step_x, 0.0f);
    gpu_info.probe_step_y = glm::vec4(source.probe_step_y, 0.0f);
    gpu_info.probe_step_z = glm::vec4(source.probe_step_z, 0.0f);
    gpu_info.probe_counts = glm::vec4(glm::vec3(ClampDdgiProbeCounts(source.probe_counts)),
                                      glm::max(ddgi_settings.runtime.irradiance_gamma, 1.0f));
    gpu_info.probe_scroll_and_priority = glm::ivec4(runtime->probe_scroll_offset, info.artist_priority);
    gpu_info.atlas_parameters =
        layout.valid ? glm::uvec4(layout.irradiance_atlas.tile_resolution, layout.irradiance_atlas.columns,
                                  layout.visibility_atlas.tile_resolution, layout.visibility_atlas.columns)
                     : glm::uvec4(1u);
    gpu_info.volume_parameters =
        glm::vec4(0.0f, glm::max(ddgi_settings.runtime.visibility_moment_bias, 0.0f),
                  glm::max(ddgi_settings.runtime.normal_bias, 0.001f), glm::max(ddgi_settings.runtime.view_bias, 0.0f));
    runtime->contributes_lighting = ddgi_settings.runtime.enabled && runtime->has_valid_probe_history && layout.valid;
    gpu_info.lighting_parameters =
        glm::vec4(runtime->contributes_lighting ? 1.0f : 0.0f, info.probe_density, 0.0f, 0.0f);
    gpu_info.identity_and_flags =
        glm::uvec4(static_cast<uint32_t>(info.stable_entity_id), static_cast<uint32_t>(info.stable_entity_id >> 32u),
                   (runtime->has_valid_probe_history ? 1u : 0u) | (ddgi_settings.runtime.pause_updates ? 2u : 0u),
                   info.sorted_index);
  };

  for (size_t i = 0; i < infos.size(); ++i) {
    prepare_runtime(infos[i], resolved_lighting.ddgi_volumes[i], preflight_layouts[i]);
  }

  if (assigned_lighting && (!reset_probe_history || !infos.empty())) {
    assigned_lighting->ddgi_settings.runtime.reset_probe_history = false;
  }
  ddgi_latched_scene_change_triggers_ = DdgiVolumeTriggerConditionNone;
  ddgi_latched_scene_geometry_changed_ = false;
  ddgi_deferred_scene_readiness_refresh_ = false;
  render_instances->render_info_block.ddgi_volume_header =
      glm::uvec4(static_cast<uint32_t>(ddgi_ordered_volume_ids_.size()), validation.aggregate_probe_count,
                 kDdgiMaxVolumeCount, 0u);
  render_instances->render_info_block.ddgi_volumes = {};
  for (size_t i = 0; i < ddgi_ordered_volume_ids_.size(); ++i) {
    render_instances->render_info_block.ddgi_volumes[i] =
        ddgi_volume_runtime_states_.at(ddgi_ordered_volume_ids_[i])->gpu_info;
  }
}

void RenderLayer::PrepareDdgiVolumeFrameState(const std::shared_ptr<Scene>& scene,
                                              const std::shared_ptr<RenderInstanceStorage>& render_instances,
                                              DdgiVolumeRuntimeState& runtime_state,
                                              const ResolvedEnvironmentalLighting::DdgiVolume& volume,
                                              const DdgiSettings& ddgi_settings,
                                              const DdgiFrameResourceLayout& preflight_layout,
                                              const uint32_t sorted_index, const bool reset_probe_history) {
  const bool prepare_debug_data = sorted_index == 0u;
  ResetDdgiRuntimeFrameState(runtime_state);
  if (!render_instances) {
    return;
  }
  auto ddgi_ray_source = CreateDdgiProbeRayDiagnosticSourceFromResolvedVolume(volume, ddgi_settings);
  ddgi_ray_source.selected_volume_index = sorted_index;
  runtime_state.emissive_mesh_sampling_enabled = ddgi_ray_source.emissive_mesh_sampling_enabled;
  runtime_state.probe_variability_gating_enabled =
      ddgi_ray_source.enable_probe_variability && ddgi_ray_source.enable_probe_variability_gating;
  const auto& layout = preflight_layout;
  const auto ddgi_total_probe_count = layout.probe_count;
  const auto previous_render_instances = GetPreviousRenderInstanceStorage();
  const bool compatible_history =
      runtime_state.has_valid_probe_history && previous_render_instances &&
      AreDdgiPersistentLayoutsCompatible(runtime_state.frame_resource_layout, layout) &&
      runtime_state.probe_metadata_buffer &&
      runtime_state.probe_metadata_buffer->GetSize() == layout.probe_metadata_byte_size &&
      runtime_state.probe_state_buffer && runtime_state.probe_state_buffer->GetSize() == layout.probe_state_byte_size &&
      HasDdgiAtlasImageLayout(runtime_state.irradiance_atlas, layout.irradiance_atlas, VK_FORMAT_R16G16B16A16_SFLOAT) &&
      HasDdgiAtlasImageLayout(runtime_state.visibility_atlas, layout.visibility_atlas, VK_FORMAT_R16G16_SFLOAT) &&
      HasDdgiAtlasImageLayout(runtime_state.variability_atlas, layout.variability_atlas, VK_FORMAT_R16_SFLOAT);
  const auto track_ddgi_environment_signature = [&] {
    const auto& environment = render_instances->environment_info_block;
    const auto cubemap_index = environment.background_color.w != 1.0f && environment.diffuse_sky_intensity > 0.0f
                                   ? GetDdgiEnvironmentCubemapIndex(scene)
                                   : 0u;
    const auto signature = MakeDdgiEnvironmentSignature(environment, cubemap_index);
    if (runtime_state.has_previous_environment_signature && signature != runtime_state.previous_environment_signature) {
      runtime_state.latched_scene_change_triggers |= DdgiVolumeTriggerConditionLightingConditionChanged;
    }
    runtime_state.has_previous_environment_signature = true;
    runtime_state.previous_environment_signature = signature;
    return cubemap_index;
  };
  if (ddgi_settings.runtime.pause_updates) {
    if (compatible_history) {
      (void)track_ddgi_environment_signature();
    } else {
      runtime_state.frame_resource_layout = {};
    }
    return;
  }
  runtime_state.frame_resource_layout = layout;
  const auto current_frame_index = Platform::GetCurrentFrameIndex();
  const auto max_frames_in_flight = static_cast<size_t>(Platform::GetMaxFramesInFlight());
  runtime_state.probe_metadata_readback_buffers.resize(max_frames_in_flight);
  runtime_state.probe_ray_readback_buffers.resize(max_frames_in_flight);
  runtime_state.selected_ray_diagnostics_buffers.resize(max_frames_in_flight);
  runtime_state.variability_readback_tickets.resize(max_frames_in_flight);
  auto& ddgi_probe_metadata_readback_buffer = runtime_state.probe_metadata_readback_buffers.at(current_frame_index);
  auto& ddgi_probe_ray_readback_buffer = runtime_state.probe_ray_readback_buffers.at(current_frame_index);
  auto& ddgi_selected_ray_diagnostics_buffer = runtime_state.selected_ray_diagnostics_buffers.at(current_frame_index);
  auto& ddgi_variability_readback_ticket = runtime_state.variability_readback_tickets.at(current_frame_index);
  auto& ddgi_variability_readback_buffer = ddgi_variability_readback_ticket.buffer;

  bool ddgi_persistent_resource_changed = false;
  if (ShouldUseDdgiFrameResources(ddgi_settings)) {
    if (!runtime_state.probe_metadata_buffer ||
        runtime_state.probe_metadata_buffer->GetSize() != layout.probe_metadata_byte_size) {
      runtime_state.probe_metadata_buffer = CreateDdgiProbeStateBuffer(layout.probe_metadata_byte_size);
      runtime_state.resource_ids[0] = runtime_state.probe_metadata_buffer ? NextDdgiResourceId() : 0u;
      runtime_state.clear_probe_atlas_this_frame = true;
      runtime_state.has_valid_probe_history = false;
      ddgi_persistent_resource_changed = true;
    }
    if (!HasDdgiAtlasImageLayout(runtime_state.irradiance_atlas, layout.irradiance_atlas,
                                 VK_FORMAT_R16G16B16A16_SFLOAT)) {
      runtime_state.irradiance_atlas = CreateDdgiAtlasImage(layout.irradiance_atlas, VK_FORMAT_R16G16B16A16_SFLOAT);
      runtime_state.resource_ids[2] = runtime_state.irradiance_atlas ? NextDdgiResourceId() : 0u;
      runtime_state.clear_probe_atlas_this_frame = true;
      runtime_state.has_valid_probe_history = false;
      ddgi_persistent_resource_changed = true;
    }
    if (!HasDdgiAtlasImageLayout(runtime_state.visibility_atlas, layout.visibility_atlas, VK_FORMAT_R16G16_SFLOAT)) {
      runtime_state.visibility_atlas = CreateDdgiAtlasImage(layout.visibility_atlas, VK_FORMAT_R16G16_SFLOAT);
      runtime_state.resource_ids[3] = runtime_state.visibility_atlas ? NextDdgiResourceId() : 0u;
      runtime_state.clear_probe_atlas_this_frame = true;
      runtime_state.has_valid_probe_history = false;
      ddgi_persistent_resource_changed = true;
    }
    if (!HasDdgiAtlasImageLayout(runtime_state.variability_atlas, layout.variability_atlas, VK_FORMAT_R16_SFLOAT)) {
      runtime_state.variability_atlas = CreateDdgiAtlasImage(layout.variability_atlas, VK_FORMAT_R16_SFLOAT);
      runtime_state.resource_ids[4] = runtime_state.variability_atlas ? NextDdgiResourceId() : 0u;
      runtime_state.clear_probe_atlas_this_frame = true;
      runtime_state.has_valid_probe_history = false;
      ddgi_persistent_resource_changed = true;
    }
    if (!ddgi_variability_readback_buffer || ddgi_variability_readback_buffer->GetSize() != sizeof(glm::vec2)) {
      ddgi_variability_readback_ticket = {};
      ddgi_variability_readback_buffer = std::make_shared<Buffer>(sizeof(glm::vec2), true);
    }
  }
  if (!ShouldTraceDdgiProbeRays(ddgi_settings) || !Platform::RayTracingEnabled() ||
      !render_instances->mesh_top_level_acceleration_structure) {
    return;
  }
  const bool startup_scene_inputs_pending =
      !compatible_history &&
      (TextureStorage::HasPendingUploads() || (ProjectManager::HasProject() && !ProjectManager::IsProjectIdle()));
  const bool scene_inputs_pending = ddgi_referenced_scene_inputs_pending_ || startup_scene_inputs_pending;
  if (scene_inputs_pending) {
    runtime_state.deferred_scene_readiness_refresh = true;
    runtime_state.scene_input_settle_frame_count = 0;
    runtime_state.last_probe_update_reasons = DdgiUpdateReasonSceneInput;
    return;
  }
  if (runtime_state.deferred_scene_readiness_refresh &&
      runtime_state.scene_input_settle_frame_count < kDdgiSceneInputSettleFrameCount) {
    ++runtime_state.scene_input_settle_frame_count;
    runtime_state.last_probe_update_reasons = DdgiUpdateReasonSceneInput;
    return;
  }

  const auto ray_count = static_cast<uint32_t>(glm::max(ddgi_settings.runtime.ray_count, 1));
  const auto fixed_ray_count = RenderLayer::GetDdgiFixedRayCount(
      ray_count, ddgi_ray_source.enable_probe_relocation || ddgi_ray_source.enable_probe_classification);
  const auto effective_max_ray_distance = CalculateDdgiEffectiveMaxRayDistance(ddgi_settings);
  const glm::vec4 trace_parameters{effective_max_ray_distance, glm::max(ddgi_settings.runtime.normal_bias, 0.001f),
                                   static_cast<float>(fixed_ray_count), 0.0f};
  const glm::vec4 update_parameters{effective_max_ray_distance,
                                    glm::clamp(ddgi_settings.runtime.hysteresis, 0.0f, 1.0f),
                                    glm::max(ddgi_settings.runtime.visibility_moment_bias, 0.0f),
                                    glm::max(ddgi_settings.runtime.irradiance_gamma, 1.0f)};
  const glm::vec4 probe_state_parameters{glm::max(ddgi_ray_source.relocation_distance, 0.0f),
                                         ddgi_ray_source.enable_probe_relocation ? 1.0f : 0.0f,
                                         ddgi_ray_source.enable_probe_classification ? 1.0f : 0.0f,
                                         glm::clamp(ddgi_settings.runtime.irradiance_threshold, 0.0f, 1.0f)};
  const glm::vec4 probe_blend_parameters{glm::clamp(ddgi_ray_source.random_ray_backface_threshold, 0.0f, 1.0f),
                                         glm::clamp(ddgi_ray_source.fixed_ray_backface_threshold, 0.0f, 1.0f),
                                         glm::max(ddgi_settings.runtime.distance_exponent, 0.0f),
                                         glm::clamp(ddgi_settings.runtime.brightness_threshold, 0.0f, 1.0f)};
  const glm::vec4 probe_variability_parameters{
      glm::max(ddgi_ray_source.probe_variability_threshold, 0.0f),
      static_cast<float>(glm::max(ddgi_ray_source.probe_variability_min_samples, 0)),
      ddgi_ray_source.enable_probe_variability ? 1.0f : 0.0f,
      ddgi_ray_source.enable_probe_variability_gating ? 1.0f : 0.0f};
  const bool had_previous_ray_source = runtime_state.has_previous_ray_source;
  const bool ddgi_variability_policy_changed =
      had_previous_ray_source && runtime_state.previous_probe_variability_parameters != probe_variability_parameters;
  runtime_state.previous_probe_variability_parameters = probe_variability_parameters;
  const auto ddgi_ray_source_common_changed =
      !had_previous_ray_source || runtime_state.previous_probe_counts != ddgi_ray_source.probe_counts ||
      runtime_state.previous_probe_step_x != ddgi_ray_source.probe_step_x ||
      runtime_state.previous_probe_step_y != ddgi_ray_source.probe_step_y ||
      runtime_state.previous_probe_step_z != ddgi_ray_source.probe_step_z ||
      runtime_state.previous_ray_count != ray_count || runtime_state.previous_trace_parameters != trace_parameters ||
      runtime_state.previous_update_parameters != update_parameters ||
      runtime_state.previous_probe_state_parameters != probe_state_parameters ||
      runtime_state.previous_probe_blend_parameters != probe_blend_parameters ||
      runtime_state.previous_deterministic_ray_seed_enabled != ddgi_settings.runtime.deterministic_ray_seed_enabled ||
      runtime_state.previous_deterministic_ray_seed != ddgi_settings.runtime.deterministic_ray_seed ||
      runtime_state.previous_movement_type != ddgi_ray_source.movement_type ||
      runtime_state.previous_emissive_mesh_sampling_enabled != runtime_state.emissive_mesh_sampling_enabled;
  const auto first_probe_changed =
      runtime_state.has_previous_ray_source && runtime_state.previous_first_probe != ddgi_ray_source.first_probe;
  const auto ddgi_scrolling_enabled =
      ddgi_ray_source.movement_type == static_cast<int>(DdgiVolumeMovementType::Scrolling);
  bool ddgi_ray_source_changed = ddgi_ray_source_common_changed;
  bool ddgi_full_scroll_reset = false;
  runtime_state.probe_scroll_clear = glm::ivec3(0);
  runtime_state.last_probe_scroll_delta = glm::ivec3(0);
  if (ddgi_ray_source_common_changed) {
    runtime_state.has_previous_ray_source = true;
    runtime_state.probe_scroll_base_first_probe = ddgi_ray_source.first_probe;
    runtime_state.probe_scroll_offset = glm::ivec3(0);
    runtime_state.probe_scroll_directions = glm::ivec3(1);
    runtime_state.previous_probe_counts = ddgi_ray_source.probe_counts;
    runtime_state.previous_first_probe = ddgi_ray_source.first_probe;
    runtime_state.previous_probe_step_x = ddgi_ray_source.probe_step_x;
    runtime_state.previous_probe_step_y = ddgi_ray_source.probe_step_y;
    runtime_state.previous_probe_step_z = ddgi_ray_source.probe_step_z;
    runtime_state.previous_ray_count = ray_count;
    runtime_state.previous_trace_parameters = trace_parameters;
    runtime_state.previous_update_parameters = update_parameters;
    runtime_state.previous_probe_state_parameters = probe_state_parameters;
    runtime_state.previous_probe_blend_parameters = probe_blend_parameters;
    runtime_state.previous_deterministic_ray_seed_enabled = ddgi_settings.runtime.deterministic_ray_seed_enabled;
    runtime_state.previous_deterministic_ray_seed = ddgi_settings.runtime.deterministic_ray_seed;
    runtime_state.previous_movement_type = ddgi_ray_source.movement_type;
    runtime_state.previous_emissive_mesh_sampling_enabled = runtime_state.emissive_mesh_sampling_enabled;
  } else if (first_probe_changed) {
    if (ddgi_scrolling_enabled) {
      NormalizeDdgiProbeScrollOrigin(runtime_state.probe_scroll_base_first_probe, runtime_state.probe_scroll_offset,
                                     ddgi_ray_source.probe_counts, ddgi_ray_source.probe_step_x,
                                     ddgi_ray_source.probe_step_y, ddgi_ray_source.probe_step_z);
      const auto effective_first_probe = CalculateDdgiEffectiveFirstProbe(
          runtime_state.probe_scroll_base_first_probe, ddgi_ray_source.probe_step_x, ddgi_ray_source.probe_step_y,
          ddgi_ray_source.probe_step_z, runtime_state.probe_scroll_offset);
      const auto first_probe_delta = ddgi_ray_source.first_probe - effective_first_probe;
      runtime_state.probe_scroll_directions = {
          AxisCoordinate(first_probe_delta, ddgi_ray_source.probe_step_x) >= 0.0f ? 1 : -1,
          AxisCoordinate(first_probe_delta, ddgi_ray_source.probe_step_y) >= 0.0f ? 1 : -1,
          AxisCoordinate(first_probe_delta, ddgi_ray_source.probe_step_z) >= 0.0f ? 1 : -1};
      runtime_state.last_probe_scroll_delta = CalculateDdgiProbeScrollDelta(
          first_probe_delta, ddgi_ray_source.probe_step_x, ddgi_ray_source.probe_step_y, ddgi_ray_source.probe_step_z);
      ddgi_full_scroll_reset =
          RequiresDdgiFullScrollReset(ddgi_ray_source.probe_counts, runtime_state.last_probe_scroll_delta);
      for (int axis = 0; axis < 3; ++axis) {
        runtime_state.probe_scroll_offset[axis] += runtime_state.last_probe_scroll_delta[axis];
        runtime_state.probe_scroll_clear[axis] = glm::abs(runtime_state.last_probe_scroll_delta[axis]);
      }
      NormalizeDdgiProbeScrollOrigin(runtime_state.probe_scroll_base_first_probe, runtime_state.probe_scroll_offset,
                                     ddgi_ray_source.probe_counts, ddgi_ray_source.probe_step_x,
                                     ddgi_ray_source.probe_step_y, ddgi_ray_source.probe_step_z);
      runtime_state.previous_first_probe = ddgi_ray_source.first_probe;
    } else {
      ddgi_ray_source_changed = true;
      runtime_state.probe_scroll_base_first_probe = ddgi_ray_source.first_probe;
      runtime_state.probe_scroll_offset = glm::ivec3(0);
      runtime_state.probe_scroll_directions = glm::ivec3(1);
      runtime_state.previous_first_probe = ddgi_ray_source.first_probe;
    }
  }
  if (ddgi_scrolling_enabled) {
    ddgi_ray_source.first_probe = CalculateDdgiEffectiveFirstProbe(
        runtime_state.probe_scroll_base_first_probe, ddgi_ray_source.probe_step_x, ddgi_ray_source.probe_step_y,
        ddgi_ray_source.probe_step_z, runtime_state.probe_scroll_offset);
  }
  ddgi_ray_source.probe_scroll_offset = runtime_state.probe_scroll_offset;
  ddgi_ray_source.probe_scroll_clear = runtime_state.probe_scroll_clear;
  ddgi_ray_source.probe_scroll_directions = runtime_state.probe_scroll_directions;
  ddgi_ray_source.probe_scroll_delta = ddgi_full_scroll_reset ? glm::ivec3(0) : runtime_state.last_probe_scroll_delta;
  const bool ddgi_scroll_clear_this_frame = runtime_state.probe_scroll_clear.x != 0 ||
                                            runtime_state.probe_scroll_clear.y != 0 ||
                                            runtime_state.probe_scroll_clear.z != 0;
  runtime_state.frame_clear_scrolled_probes = ddgi_scroll_clear_this_frame && !ddgi_full_scroll_reset;
  const auto ddgi_environment_cubemap_index = track_ddgi_environment_signature();
  const bool scene_readiness_refresh = runtime_state.deferred_scene_readiness_refresh;
  const auto contributor_triggers = runtime_state.latched_scene_change_triggers & DdgiVolumeTriggerConditionAll;
  const auto auto_invalidate_triggers =
      contributor_triggers & (ddgi_ray_source.auto_invalidate_trigger_conditions & DdgiVolumeTriggerConditionAll);
  const bool scene_auto_invalidate = auto_invalidate_triggers != DdgiVolumeTriggerConditionNone;
  const bool scene_change_triggers_pending =
      contributor_triggers != DdgiVolumeTriggerConditionNone || runtime_state.latched_scene_geometry_changed;
  uint32_t full_refresh_reasons = DdgiUpdateReasonNone;
  if (ddgi_ray_source_changed) {
    full_refresh_reasons |= DdgiUpdateReasonSource;
  }
  if (ddgi_variability_policy_changed) {
    full_refresh_reasons |= DdgiUpdateReasonVariabilityPolicy;
  }
  if (reset_probe_history) {
    full_refresh_reasons |= DdgiUpdateReasonManualReset;
  }
  if (scene_auto_invalidate || scene_readiness_refresh) {
    full_refresh_reasons |= DdgiUpdateReasonSceneInput;
  }
  runtime_state.last_probe_update_reasons =
      full_refresh_reasons == DdgiUpdateReasonNone ? DdgiUpdateReasonSteadyState : full_refresh_reasons;
  runtime_state.clear_probe_atlas_this_frame = runtime_state.clear_probe_atlas_this_frame || ddgi_ray_source_changed ||
                                               reset_probe_history || ddgi_full_scroll_reset || scene_auto_invalidate;
  if (runtime_state.clear_probe_atlas_this_frame) {
    runtime_state.has_valid_probe_history = false;
  }
  const auto probe_variability_enabled = ddgi_ray_source.enable_probe_variability;
  const auto probe_variability_gating_enabled =
      probe_variability_enabled && ddgi_ray_source.enable_probe_variability_gating;
  const auto probe_variability_min_samples =
      static_cast<uint32_t>(glm::max(ddgi_ray_source.probe_variability_min_samples, 0));
  const auto readback_generation = ddgi_variability_readback_ticket.generation;
  const bool has_unconsumed_variability_readback =
      readback_generation > runtime_state.last_consumed_variability_generation &&
      ddgi_variability_readback_ticket.submission;
  const bool has_fresh_variability_readback =
      has_unconsumed_variability_readback && !ddgi_persistent_resource_changed &&
      ddgi_variability_readback_ticket.submission->status == FrameSubmissionState::Status::Submitted;
  const auto variability_observation =
      has_fresh_variability_readback ? ReadDdgiProbeVariabilityObservation(ddgi_variability_readback_ticket.buffer)
                                     : DdgiProbeVariabilityObservation{};
  const bool discarded_variability_readback =
      has_unconsumed_variability_readback &&
      ddgi_variability_readback_ticket.submission->status == FrameSubmissionState::Status::Discarded;
  if (has_fresh_variability_readback || discarded_variability_readback ||
      (has_unconsumed_variability_readback && ddgi_persistent_resource_changed)) {
    runtime_state.last_consumed_variability_generation = readback_generation;
  }
  if (discarded_variability_readback && runtime_state.probe_variability_refresh_waiting &&
      readback_generation >= runtime_state.probe_variability_refresh_generation) {
    runtime_state.probe_variability_refresh_waiting = false;
    runtime_state.probe_variability_refresh_generation = 0u;
    runtime_state.probe_variability_refresh_age = kDdgiProbeRefreshInterval;
  }
  const bool hard_ddgi_refresh = ddgi_ray_source_changed || reset_probe_history || ddgi_persistent_resource_changed ||
                                 ddgi_full_scroll_reset || scene_auto_invalidate;
  const bool transport_refresh = hard_ddgi_refresh || ddgi_variability_policy_changed || scene_readiness_refresh;
  if (transport_refresh) {
    runtime_state.probe_ray_sequence_index = 0u;
  }
  const bool reset_ddgi_warmup_state =
      hard_ddgi_refresh || DdgiTriggerConditionEnabled(contributor_triggers, ddgi_ray_source.warmup_trigger_conditions &
                                                                                 DdgiVolumeTriggerConditionAll);
  const bool variability_trigger_refresh = DdgiTriggerConditionEnabled(
      contributor_triggers, ddgi_ray_source.variability_reset_trigger_conditions & DdgiVolumeTriggerConditionAll);
  const bool reset_ddgi_variability_state = hard_ddgi_refresh || ddgi_variability_policy_changed ||
                                            variability_trigger_refresh || ddgi_scroll_clear_this_frame;
  if (!probe_variability_enabled || reset_ddgi_variability_state) {
    runtime_state.probe_variability_sample_count = 0;
    runtime_state.probe_variability_stable_sample_count = 0;
    runtime_state.probe_variability_average = 0.0f;
    runtime_state.probe_variability_converged = false;
    runtime_state.probe_variability_refresh_age = 0u;
    runtime_state.probe_variability_refresh_waiting = false;
    runtime_state.probe_variability_refresh_generation = 0u;
    runtime_state.last_consumed_variability_generation = runtime_state.next_variability_generation;
  }
  if (reset_ddgi_warmup_state) {
    runtime_state.probe_warmup_frame_index = 0;
  }
  runtime_state.frame_probe_warmup_frame_count =
      static_cast<uint32_t>(glm::max(ddgi_settings.runtime.warmup_frames, 0));
  runtime_state.frame_probe_warmup_frame_index = runtime_state.probe_warmup_frame_index;
  runtime_state.frame_probe_warmup_active =
      runtime_state.frame_probe_warmup_frame_count != 0u &&
      runtime_state.probe_warmup_frame_index < runtime_state.frame_probe_warmup_frame_count;
  const auto ddgi_first_warmup_frame =
      runtime_state.frame_probe_warmup_active && runtime_state.probe_warmup_frame_index == 0u;
  if (runtime_state.frame_probe_warmup_active && full_refresh_reasons == DdgiUpdateReasonNone) {
    runtime_state.last_probe_update_reasons |= DdgiUpdateReasonWarmup;
  }

  bool entered_convergence = false;
  if (!reset_ddgi_variability_state && !scene_auto_invalidate && has_fresh_variability_readback) {
    const bool completed_periodic_refresh = runtime_state.probe_variability_refresh_waiting &&
                                            readback_generation >= runtime_state.probe_variability_refresh_generation;
    if (completed_periodic_refresh) {
      runtime_state.probe_variability_refresh_waiting = false;
      runtime_state.probe_variability_refresh_generation = 0u;
      runtime_state.probe_variability_refresh_age = 0u;
    }
    if (variability_observation.valid) {
      runtime_state.probe_variability_average = variability_observation.average;
    }
    if (runtime_state.probe_variability_converged) {
      const auto update = AdvanceDdgiProbeConvergence(
          {runtime_state.probe_variability_sample_count, runtime_state.probe_variability_stable_sample_count,
           runtime_state.probe_variability_converged},
          variability_observation, probe_variability_min_samples, ddgi_ray_source.probe_variability_threshold);
      runtime_state.probe_variability_sample_count = update.state.sample_count;
      runtime_state.probe_variability_stable_sample_count = update.state.stable_sample_count;
      runtime_state.probe_variability_converged = update.state.converged;
    } else if (!probe_variability_gating_enabled || runtime_state.frame_probe_warmup_active) {
      if (variability_observation.valid) {
        ++runtime_state.probe_variability_sample_count;
      }
      runtime_state.probe_variability_stable_sample_count = 0u;
    } else {
      const auto update = AdvanceDdgiProbeConvergence(
          {runtime_state.probe_variability_sample_count, runtime_state.probe_variability_stable_sample_count, false},
          variability_observation, probe_variability_min_samples, ddgi_ray_source.probe_variability_threshold);
      runtime_state.probe_variability_sample_count = update.state.sample_count;
      runtime_state.probe_variability_stable_sample_count = update.state.stable_sample_count;
      runtime_state.probe_variability_converged = update.state.converged;
      entered_convergence = update.entered_convergence;
    }
    if (completed_periodic_refresh && !variability_observation.valid) {
      runtime_state.probe_variability_refresh_age = kDdgiProbeRefreshInterval;
    }
  }
  if (!probe_variability_gating_enabled) {
    runtime_state.probe_variability_converged = false;
    runtime_state.probe_variability_stable_sample_count = 0u;
    runtime_state.probe_variability_refresh_age = 0u;
    runtime_state.probe_variability_refresh_waiting = false;
    runtime_state.probe_variability_refresh_generation = 0u;
  } else if (runtime_state.probe_variability_converged && !runtime_state.probe_variability_refresh_waiting) {
    runtime_state.probe_variability_refresh_age =
        entered_convergence || scene_auto_invalidate
            ? 0u
            : glm::min(runtime_state.probe_variability_refresh_age + 1u, kDdgiProbeRefreshInterval);
  }
  const bool periodic_refresh_due = IsDdgiPeriodicRefreshDue(
      probe_variability_gating_enabled, runtime_state.probe_variability_converged,
      runtime_state.probe_variability_refresh_waiting, runtime_state.probe_variability_refresh_age);

  const bool scene_probe_state_reset =
      runtime_state.latched_scene_geometry_changed &&
      DdgiTriggerConditionEnabled(auto_invalidate_triggers, DdgiVolumeTriggerConditionGeometryChanged);
  bool reset_probe_state = hard_ddgi_refresh || scene_probe_state_reset || !runtime_state.probe_state_buffer ||
                           runtime_state.probe_state_buffer->GetSize() != layout.probe_state_byte_size;
  if (reset_probe_state) {
    runtime_state.probe_state_buffer = CreateDdgiProbeStateBuffer(layout.probe_state_byte_size);
    runtime_state.resource_ids[1] = runtime_state.probe_state_buffer ? NextDdgiResourceId() : 0u;
    if (runtime_state.probe_state_buffer) {
      const std::vector<glm::vec4> zero_state(layout.probe_count, glm::vec4(0.0f));
      runtime_state.probe_state_buffer->UploadVector(zero_state);
    }
  }

  const bool wait_for_periodic_observation = runtime_state.probe_variability_refresh_waiting;
  const bool forced_probe_trace = scene_auto_invalidate || scene_readiness_refresh ||
                                  runtime_state.frame_probe_warmup_active || ddgi_variability_policy_changed;
  const bool should_trace_probe_rays =
      forced_probe_trace ||
      (!wait_for_periodic_observation && (!runtime_state.probe_variability_converged || periodic_refresh_due));
  if (!should_trace_probe_rays) {
    runtime_state.last_probe_update_reasons = DdgiUpdateReasonConverged;
  } else if (periodic_refresh_due && full_refresh_reasons == DdgiUpdateReasonNone &&
             !runtime_state.frame_probe_warmup_active) {
    runtime_state.last_probe_update_reasons = DdgiUpdateReasonPeriodicRefresh;
  }
  if (!should_trace_probe_rays) {
    if (scene_change_triggers_pending) {
      runtime_state.latched_scene_change_triggers = DdgiVolumeTriggerConditionNone;
      runtime_state.latched_scene_geometry_changed = false;
    }
    return;
  }
  const auto skip_inactive_probe_trace = ddgi_ray_source.enable_probe_classification;
  const auto ddgi_update_hysteresis = CalculateDdgiUpdateHysteresis(
      ddgi_settings, runtime_state.last_probe_update_reasons, runtime_state.probe_warmup_frame_index);
  const auto ddgi_update_brightness_threshold = ddgi_first_warmup_frame
                                                    ? (std::numeric_limits<float>::max)()
                                                    : CalculateDdgiUpdateBrightnessThreshold(ddgi_settings);
  runtime_state.frame_probe_update_hysteresis = ddgi_update_hysteresis;
  runtime_state.frame_ray_push_constant = CreateDdgiProbeRayTracingPushConstant(
      ddgi_settings, ddgi_ray_source, skip_inactive_probe_trace, ddgi_first_warmup_frame, sorted_index,
      ddgi_environment_cubemap_index, runtime_state.probe_ray_sequence_index);
  runtime_state.frame_probe_update_push_constant = CreateDdgiProbeAtlasUpdatePushConstant(
      ddgi_settings, layout, ddgi_ray_source, ddgi_update_hysteresis, ddgi_update_brightness_threshold);
  runtime_state.frame_probe_update_push_constant.probe_step_x.w = runtime_state.frame_ray_push_constant.first_probe.w;
  runtime_state.frame_probe_update_push_constant.probe_step_y.w = runtime_state.frame_ray_push_constant.probe_step_x.w;
  runtime_state.frame_probe_update_push_constant.probe_step_z.w = runtime_state.frame_ray_push_constant.probe_step_y.w;
  runtime_state.frame_probe_update_push_constant.probe_counts_and_rotation.w =
      glm::floatBitsToUint(runtime_state.frame_ray_push_constant.probe_step_z.w);
  runtime_state.frame_probe_scroll_push_constant = CreateDdgiProbeScrollClearPushConstant(layout, ddgi_ray_source);
  runtime_state.frame_probe_relocation_reset_push_constant =
      CreateDdgiProbeRelocationPushConstant(ddgi_settings, ddgi_total_probe_count, ddgi_ray_source, true);
  runtime_state.frame_probe_relocation_update_push_constant =
      CreateDdgiProbeRelocationPushConstant(ddgi_settings, ddgi_total_probe_count, ddgi_ray_source, false);
  runtime_state.frame_probe_classification_reset_push_constant =
      CreateDdgiProbeClassificationPushConstant(ddgi_settings, ddgi_total_probe_count, ddgi_ray_source, true);
  runtime_state.frame_probe_classification_update_push_constant =
      CreateDdgiProbeClassificationPushConstant(ddgi_settings, ddgi_total_probe_count, ddgi_ray_source, false);
  if (!ddgi_selected_ray_diagnostics_buffer ||
      ddgi_selected_ray_diagnostics_buffer->GetSize() != layout.selected_ray_diagnostics_byte_size) {
    ddgi_selected_ray_diagnostics_buffer = CreateDdgiProbeStateBuffer(layout.selected_ray_diagnostics_byte_size);
  }
  if (!ddgi_selected_ray_diagnostics_buffer) {
    return;
  }
  if (prepare_debug_data && ddgi_settings.debug.enabled && ddgi_settings.debug.visualize_probe_state) {
    if (!ddgi_probe_metadata_readback_buffer ||
        ddgi_probe_metadata_readback_buffer->GetSize() != layout.probe_metadata_byte_size) {
      ddgi_probe_metadata_readback_buffer = std::make_shared<Buffer>(layout.probe_metadata_byte_size, true);
      runtime_state.metadata_readback_ticket = {};
      runtime_state.probe_debug_metadata.clear();
    }
  }
  if (prepare_debug_data && ddgi_settings.debug.enabled && ddgi_settings.debug.show_rays) {
    const auto selected_logical_probe = glm::min(
        static_cast<uint32_t>(glm::max(ddgi_settings.debug.selected_probe_index, 0)), ddgi_total_probe_count - 1u);
    const auto selected_probe_grid =
        RenderLayer::GetDdgiProbeGridIndex(ddgi_ray_source.probe_counts, selected_logical_probe);
    const auto selected_physical_probe =
        GetScrolledDdgiProbeIndex(selected_probe_grid, runtime_state.probe_scroll_offset, ddgi_ray_source.probe_counts);
    const auto selected_ray_byte_size =
        static_cast<uint64_t>(ray_count) * static_cast<uint64_t>(sizeof(PointCloudSample));
    if (selected_ray_byte_size != 0u) {
      if (!ddgi_probe_ray_readback_buffer || ddgi_probe_ray_readback_buffer->GetSize() != selected_ray_byte_size) {
        ddgi_probe_ray_readback_buffer = std::make_shared<Buffer>(selected_ray_byte_size, true);
        runtime_state.ray_readback_ticket = {};
        runtime_state.probe_debug_ray_samples.clear();
      }
      runtime_state.frame_selected_probe_ray_sample_count = ray_count;
      runtime_state.frame_ray_push_constant.selected_probe_volume_flags_environment.x = selected_logical_probe;
      runtime_state.frame_selected_probe_ray_logical_index = selected_logical_probe;
      runtime_state.frame_selected_probe_ray_physical_index = selected_physical_probe;
    }
  }

  if (probe_variability_enabled && (!runtime_state.probe_variability_converged || periodic_refresh_due)) {
    runtime_state.frame_probe_variability_enabled = true;
    const auto generation = ++runtime_state.next_variability_generation;
    runtime_state.frame_variability_readback_generation = generation;
    if (periodic_refresh_due) {
      runtime_state.probe_variability_refresh_waiting = true;
      runtime_state.probe_variability_refresh_generation = generation;
    }
  }
  if (runtime_state.frame_probe_warmup_active) {
    runtime_state.probe_warmup_frame_index =
        glm::min(runtime_state.probe_warmup_frame_index + 1u, runtime_state.frame_probe_warmup_frame_count);
  }
  runtime_state.frame_probe_relocation_reset = reset_probe_state;
  runtime_state.frame_probe_relocation_enabled = ddgi_ray_source.enable_probe_relocation;
  runtime_state.frame_probe_classification_reset = reset_probe_state;
  runtime_state.frame_probe_classification_enabled = ddgi_ray_source.enable_probe_classification;
  if (scene_change_triggers_pending) {
    runtime_state.latched_scene_change_triggers = DdgiVolumeTriggerConditionNone;
    runtime_state.latched_scene_geometry_changed = false;
  }
  runtime_state.deferred_scene_readiness_refresh = false;
  runtime_state.scene_input_settle_frame_count = 0;
  runtime_state.frame_trace_probe_rays = true;
  runtime_state.has_valid_probe_history = true;
  ++runtime_state.probe_ray_sequence_index;
}

void RenderLayer::BindRenderInstanceStorage(const uint32_t current_frame_index,
                                            const std::shared_ptr<RenderInstanceStorage>& render_instances) const {
  render_instances->RefreshRasterMaterialDescriptorSets(raster_material_layout_,
                                                        GetRasterMaterialFallbackDescriptorImageInfos());

  const auto update_per_frame_buffers = [&](const std::shared_ptr<DescriptorSet>& descriptor_set) {
    descriptor_set->UpdateBufferDescriptorBinding(0, render_instances->render_info_descriptor_buffer);
    descriptor_set->UpdateBufferDescriptorBinding(1, render_instances->environment_info_descriptor_buffer);
    descriptor_set->UpdateBufferDescriptorBinding(2, render_instances->camera_info_descriptor_buffer);
    descriptor_set->UpdateBufferDescriptorBinding(4, render_instances->instance_info_descriptor_buffer);
    descriptor_set->UpdateBufferDescriptorBinding(5, kernel_descriptor_buffers_[current_frame_index]);
    descriptor_set->UpdateBufferDescriptorBinding(6, render_instances->directional_light_info_descriptor_buffer);
    descriptor_set->UpdateBufferDescriptorBinding(7, render_instances->point_light_info_descriptor_buffer);
    descriptor_set->UpdateBufferDescriptorBinding(8, render_instances->spot_light_info_descriptor_buffer);
    descriptor_set->UpdateBufferDescriptorBinding(11, render_instances->gltf_material_descriptor_buffer);
    descriptor_set->UpdateBufferDescriptorBinding(12, render_instances->gltf_texture_info_descriptor_buffer);
  };
  update_per_frame_buffers(per_frame_descriptor_sets_[current_frame_index]);
  update_per_frame_buffers(raster_material_per_frame_descriptor_sets_[current_frame_index]);

  meshlet_descriptor_sets_[current_frame_index]->UpdateBufferDescriptorBinding(0, GeometryStorage::GetVertexBuffer());
  meshlet_descriptor_sets_[current_frame_index]->UpdateBufferDescriptorBinding(1, GeometryStorage::GetMeshletBuffer());
  if (current_frame_index < strand_meshlet_descriptor_sets_.size()) {
    strand_meshlet_descriptor_sets_[current_frame_index]->UpdateBufferDescriptorBinding(
        0, GeometryStorage::GetStrandPointBuffer());
    strand_meshlet_descriptor_sets_[current_frame_index]->UpdateBufferDescriptorBinding(
        1, GeometryStorage::GetStrandMeshletBuffer());
  }

  if (per_frame_bindless_texture_descriptors_enabled_) {
    TextureStorage::BindTexture2DToDescriptorSet(per_frame_descriptor_sets_[current_frame_index], 9);
    TextureStorage::BindCubemapToDescriptorSet(per_frame_descriptor_sets_[current_frame_index], 10);
  }
  if (Platform::RayAccelerationStructureEnabled() && current_frame_index < ray_tracing_descriptor_sets_.size()) {
    ray_tracing_descriptor_sets_[current_frame_index]->UpdateBufferDescriptorBinding(
        3, render_instances->emissive_triangle_info_descriptor_buffer);
  }
}

std::shared_ptr<DescriptorSet> RenderLayer::GetRasterLightingTextureDescriptorSet(
    const uint32_t current_frame_index, const int camera_index,
    const std::shared_ptr<RenderInstanceStorage>& render_instances) const {
  if (current_frame_index >= raster_lighting_texture_descriptor_sets_.size() || camera_index < 0 ||
      !raster_lighting_texture_layout_ || !render_instances ||
      static_cast<size_t>(camera_index) >= render_instances->camera_info_blocks_.size()) {
    return {};
  }

  auto& frame_descriptor_sets = raster_lighting_texture_descriptor_sets_[current_frame_index];
  if (frame_descriptor_sets.size() <= static_cast<size_t>(camera_index)) {
    frame_descriptor_sets.resize(static_cast<size_t>(camera_index) + 1);
  }
  auto& descriptor_set = frame_descriptor_sets[camera_index];
  if (!descriptor_set) {
    descriptor_set = std::make_shared<DescriptorSet>(raster_lighting_texture_layout_);
  }
  EnsureRasterMaterialFallbackTextures();

  const auto bind_texture_2d = [&](const uint32_t binding, const uint32_t texture_index,
                                   const std::shared_ptr<Texture2D>& fallback) {
    VkDescriptorImageInfo image_info{};
    if (TextureStorage::TryGetTexture2DDescriptorImageInfo(texture_index, image_info) ||
        (fallback &&
         TextureStorage::TryGetTexture2DDescriptorImageInfo(fallback->GetTextureStorageIndex(), image_info))) {
      descriptor_set->UpdateImageDescriptorBinding(binding, image_info);
    }
  };
  const auto bind_cubemap = [&](const uint32_t binding, const int texture_index,
                                const std::shared_ptr<Cubemap>& fallback) {
    VkDescriptorImageInfo image_info{};
    if ((texture_index >= 0 &&
         TextureStorage::TryGetCubemapDescriptorImageInfo(static_cast<uint32_t>(texture_index), image_info)) ||
        (fallback &&
         TextureStorage::TryGetCubemapDescriptorImageInfo(fallback->GetTextureStorageIndex(), image_info))) {
      descriptor_set->UpdateImageDescriptorBinding(binding, image_info);
    }
  };

  auto default_skybox = Resources::GetInstance().GetDefaultSkybox();
  std::shared_ptr<Cubemap> default_irradiance;
  std::shared_ptr<Cubemap> default_prefiltered;
  if (const auto default_environment = Resources::GetInstance().GetDefaultEnvironmentalMap()) {
    if (const auto light_probe = default_environment->light_probe.Get<LightProbe>()) {
      default_irradiance = light_probe->GetCubemap();
    }
  }
  if (const auto reflection_probe = Resources::GetInstance().GetDefaultGlobalReflectionProbe()) {
    default_prefiltered = reflection_probe->GetCubemap();
  }
  if (!default_irradiance) {
    default_irradiance = default_skybox;
  }
  if (!default_prefiltered) {
    default_prefiltered = default_skybox;
  }

  const auto& camera_info = render_instances->camera_info_blocks_[camera_index];
  bind_texture_2d(kRasterLightingBrdfLutBinding,
                  environmental_brdf_lut_ ? environmental_brdf_lut_->GetTextureStorageIndex() : 0u,
                  raster_material_white_fallback_texture_);
  bind_cubemap(kRasterLightingSkyboxBinding, camera_info.skybox_texture_index, default_skybox);
  bind_cubemap(kRasterLightingIrradianceBinding, camera_info.environmental_irradiance_texture_index,
               default_irradiance);
  VkDescriptorImageInfo global_prefiltered_info{};
  const bool has_global_prefiltered =
      (camera_info.environmental_prefiltered_index >= 0 &&
       TextureStorage::TryGetCubemapDescriptorImageInfo(
           static_cast<uint32_t>(camera_info.environmental_prefiltered_index), global_prefiltered_info)) ||
      (default_prefiltered && TextureStorage::TryGetCubemapDescriptorImageInfo(
                                  default_prefiltered->GetTextureStorageIndex(), global_prefiltered_info));
  if (has_global_prefiltered) {
    descriptor_set->UpdateImageDescriptorBinding(kRasterLightingPrefilteredBinding, global_prefiltered_info);
  }
  bind_texture_2d(kRasterLightingAmbientOcclusionBinding, (std::numeric_limits<uint32_t>::max)(),
                  raster_material_white_fallback_texture_);
  for (uint32_t slot = 0; slot < RenderInstanceStorage::kReflectionProbeMaxCount; ++slot) {
    VkDescriptorImageInfo image_info{};
    bool bound = false;
    if (slot < render_instances->render_info_block.reflection_probe_header.x) {
      const auto& probe = render_instances->render_info_block.reflection_probes[slot];
      bound = probe.identity_and_flags.y != 0u &&
              TextureStorage::TryGetCubemapDescriptorImageInfo(probe.identity_and_flags.x, image_info);
    }
    if (!bound && has_global_prefiltered) {
      image_info = global_prefiltered_info;
      bound = true;
    }
    if (bound) {
      descriptor_set->UpdateImageDescriptorBinding(kRasterLightingReflectionProbesBinding, image_info, slot);
    }
  }
  return descriptor_set;
}

void RenderLayer::RenderSceneToCameraImmediately(const std::shared_ptr<Scene>& scene,
                                                 const GlobalTransform& camera_global_transform,
                                                 const std::shared_ptr<Camera>& camera,
                                                 const bool reflection_probe_capture) {
  if (!scene || !camera || !Platform::Initialized()) {
    return;
  }

  const auto current_frame_index = Platform::GetCurrentFrameIndex();
  const auto previous_render_instances = render_instances_list_[current_frame_index];
  const bool previous_need_fade = need_fade_;
  const auto restore = [&] {
    render_instances_list_[current_frame_index] = previous_render_instances;
    need_fade_ = previous_need_fade;
    if (previous_render_instances) {
      BindRenderInstanceStorage(current_frame_index, previous_render_instances);
    }
  };
  try {
    render_instances_list_[current_frame_index] = std::make_shared<RenderInstanceStorage>();
    const std::pair<GlobalTransform, std::shared_ptr<Camera>> injected_camera{camera_global_transform, camera};
    PrepareSceneForRendering(scene, false, false, false, false, reflection_probe_capture ? &injected_camera : nullptr,
                             !reflection_probe_capture);
    const auto capture_render_instances = render_instances_list_[current_frame_index];
    bool render_info_changed = false;
    if (capture_render_instances && capture_render_instances->render_info_block.shadow_fade_parameters.y != 0.0f) {
      capture_render_instances->render_info_block.shadow_fade_parameters.y = 0.0f;
      render_info_changed = true;
    }
    if (reflection_probe_capture && capture_render_instances) {
      capture_render_instances->environment_info_block.diffuse_fallback_intensity = 0.0f;
      capture_render_instances->environment_info_block.specular_fallback_intensity = 0.0f;
      if (previous_render_instances) {
        PreserveDdgiRenderInfo(capture_render_instances->render_info_block,
                               previous_render_instances->render_info_block);
      }
      capture_render_instances->render_info_block.reflection_probe_header = glm::uvec4(0u);
      capture_render_instances->render_info_block.reflection_probes = {};
      capture_render_instances->render_info_block.debug_visualization = 0;
      capture_render_instances->render_info_block.shadow_debug_parameters =
          glm::ivec4(0, 0, 0, glm::clamp(capture_render_instances->render_info_block.shadow_debug_parameters.w, 1, 64));
      render_info_changed = true;
    }
    if (render_info_changed) {
      capture_render_instances->Upload();
      BindRenderInstanceStorage(current_frame_index, capture_render_instances);
    }
    if (reflection_probe_capture && previous_render_instances && capture_render_instances) {
      PreparePointAndSpotLightShadowMap(true, false);
    }
    RenderToCamera(scene, camera_global_transform, camera, true, reflection_probe_capture);
  } catch (...) {
    restore();
    throw;
  }
  restore();
}

uint64_t RenderLayer::GetReflectionProbeCaptureFingerprint(const std::shared_ptr<Scene>& scene,
                                                           const glm::vec3& position) const {
  if (!scene || scene.get() != GetScene().get() || render_instances_list_.empty()) {
    return 0;
  }
  const auto render_instances = render_instances_list_[Platform::GetCurrentFrameIndex()];
  if (!render_instances) {
    return 0;
  }

  uint64_t fingerprint = 0x52504650524f4245ull;
  fingerprint = MixDdgiSignature(fingerprint, kReflectionProbeCaptureFingerprintVersion);
  fingerprint = MixDdgiSignature(fingerprint, GlobalReflectionProbe::kSchemaVersion);
  fingerprint = MixDdgiSignature(fingerprint, GlobalReflectionProbe::kResolution);
  fingerprint = MixDdgiSignature(fingerprint, GlobalReflectionProbe::kMipLevels);
  fingerprint = MixDdgiSignature(fingerprint, static_cast<uint32_t>(GlobalReflectionProbe::kCanonicalFormat));
  fingerprint = MixDdgiFloat(fingerprint, GlobalReflectionProbe::kBakeNearPlane);
  fingerprint = MixDdgiFloat(fingerprint, GlobalReflectionProbe::kBakeFarPlane);
  fingerprint = MixDdgiVec3(fingerprint, position);

  fingerprint = MixDdgiFloat(fingerprint, render_settings.max_shadow_distance);
  fingerprint = MixDdgiFloat(fingerprint, render_settings.shadow_cascade_split_lambda);
  fingerprint = MixDdgiSignature(fingerprint, static_cast<uint32_t>(render_settings.shadow_cascade_fit_mode));
  fingerprint = MixDdgiSignature(fingerprint, static_cast<uint32_t>(render_settings.pcf_sample_amount));
  fingerprint = MixDdgiSignature(fingerprint, static_cast<uint32_t>(render_settings.directional_pcf_sample_amount));
  fingerprint = MixDdgiFloat(fingerprint, render_settings.shadow_cascade_transition_width);
  fingerprint = MixDdgiFloat(fingerprint, render_settings.shadow_distance_fade);
  fingerprint = MixDdgiFloat(fingerprint, render_settings.strands_subdivision_x_factor);
  fingerprint = MixDdgiFloat(fingerprint, render_settings.strands_subdivision_y_factor);
  fingerprint = MixDdgiSignature(fingerprint, static_cast<uint32_t>(render_settings.strands_subdivision_max_x));
  fingerprint = MixDdgiSignature(fingerprint, static_cast<uint32_t>(render_settings.strands_subdivision_max_y));
  const auto& graphics_settings = ApplicationContext::Get().GetApplicationInfo().graphics_settings;
  fingerprint = MixDdgiSignature(fingerprint, graphics_settings.directional_light_shadow_map_resolution);
  fingerprint = MixDdgiSignature(fingerprint, graphics_settings.point_light_shadow_map_resolution);
  fingerprint = MixDdgiSignature(fingerprint, graphics_settings.spot_light_shadow_map_resolution);
  fingerprint = MixDdgiSignature(fingerprint, graphics_settings.max_directional_light_size);
  fingerprint = MixDdgiSignature(fingerprint, graphics_settings.max_point_light_size);
  fingerprint = MixDdgiSignature(fingerprint, graphics_settings.max_spot_light_size);

  const auto resolved_lighting = ResolveEnvironmentalLighting(scene);
  const auto& indirect_source = resolved_lighting.indirect_environment_source;
  fingerprint = MixDdgiFloat(fingerprint, resolved_lighting.environment_lighting_intensity);
  fingerprint = MixDdgiSignature(fingerprint, static_cast<uint32_t>(indirect_source.kind));
  const auto mix_cubemap = [&](const std::shared_ptr<Cubemap>& cubemap) {
    fingerprint = MixDdgiSignature(fingerprint, GetStableCubemapHash(cubemap));
  };
  if (indirect_source.kind == ResolvedEnvironmentalLighting::IndirectEnvironmentSourceKind::Color) {
    fingerprint = MixDdgiVec3(fingerprint, indirect_source.color);
  } else {
    fingerprint = MixDdgiFloat(fingerprint, indirect_source.gamma);
    fingerprint = MixDdgiFloat(fingerprint, indirect_source.rotation);
    if (const auto environmental_map = ResolveIndirectEnvironmentMap(indirect_source)) {
      environmental_map->EnsureEnvironmentSource();
      auto cubemap_ref = environmental_map->environment_cubemap;
      mix_cubemap(cubemap_ref.Get<Cubemap>());
      auto light_probe_ref = environmental_map->light_probe;
      if (const auto light_probe = light_probe_ref.Get<LightProbe>()) {
        mix_cubemap(light_probe->GetCubemap());
      } else {
        mix_cubemap({});
      }
    } else {
      mix_cubemap({});
      mix_cubemap({});
    }
  }
  if (const auto reflection_probe = scene->GetGlobalReflectionProbeFallback(false)) {
    fingerprint = MixDdgiSignature(fingerprint, reflection_probe->GetPayloadHash());
  } else {
    fingerprint = MixDdgiSignature(fingerprint, 0u);
  }

  fingerprint = MixDdgiSignature(fingerprint, GetReflectionProbeSceneSourceFingerprint(scene));

  const auto& render_info = render_instances->render_info_block;
  fingerprint = MixDdgiSignature(fingerprint, render_info.ddgi_volume_header.x);
  for (uint32_t index = 0;
       index < glm::min(render_info.ddgi_volume_header.x, RenderInstanceStorage::kDdgiMaxVolumeCount); ++index) {
    const auto& volume = render_info.ddgi_volumes[index];
    fingerprint = MixDdgiVec4(fingerprint, volume.first_probe);
    fingerprint = MixDdgiVec4(fingerprint, volume.probe_step_x);
    fingerprint = MixDdgiVec4(fingerprint, volume.probe_step_y);
    fingerprint = MixDdgiVec4(fingerprint, volume.probe_step_z);
    fingerprint = MixDdgiVec4(fingerprint, volume.probe_counts);
    fingerprint = MixDdgiVec4(fingerprint, volume.volume_parameters);
    fingerprint = MixDdgiVec4(fingerprint, volume.lighting_parameters);
    fingerprint = MixDdgiSignature(fingerprint, volume.identity_and_flags.z);
    fingerprint = MixDdgiSignature(fingerprint, volume.identity_and_flags.w);
  }
  return fingerprint;
}

bool RenderLayer::QueueGlobalReflectionProbeBake(const std::shared_ptr<Scene>& scene, const glm::vec3& position,
                                                 const std::shared_ptr<GlobalReflectionProbe>& target) {
  std::string error;
  if (!ValidateGlobalReflectionProbeBakeRequest(scene, position, target, error)) {
    EVOENGINE_ERROR("Environmental lighting reflection probe bake was not queued: " + error)
    return false;
  }
  if (scene.get() != GetScene().get()) {
    EVOENGINE_ERROR(
        "Environmental lighting reflection probe bake was not queued: reflection probes can only bake the active "
        "scene.")
    return false;
  }
  QueueGlobalReflectionProbeBakeAttempt(scene, position, target, 0u);
  return true;
}

void RenderLayer::QueueGlobalReflectionProbeBakeAttempt(const std::shared_ptr<Scene>& scene, const glm::vec3& position,
                                                        const std::shared_ptr<GlobalReflectionProbe>& target,
                                                        const uint32_t retry_count) {
  ApplicationContext::Get().QueueEndOfLoopAction([scene, position, target, retry_count] {
    const auto render_layer = ApplicationContext::Get().GetLayer<RenderLayer>();
    std::string error;
    if (!render_layer) {
      EVOENGINE_ERROR("Environmental lighting reflection probe bake failed: the render layer became unavailable.")
      return;
    }
    if (!ValidateGlobalReflectionProbeBakeRequest(scene, position, target, error)) {
      EVOENGINE_ERROR("Environmental lighting reflection probe bake failed: " + error)
      return;
    }
    bool retry = false;
    uint64_t source_fingerprint = 0u;
    if (render_layer->BakeReflectionProbe(scene, position, target, source_fingerprint, error, retry)) {
      EVOENGINE_LOG("Baked environmental lighting reflection probe payload.")
      return;
    }
    if (retry && retry_count + 1u < kStandaloneReflectionProbeBakeMaxRetryFrames) {
      render_layer->QueueGlobalReflectionProbeBakeAttempt(scene, position, target, retry_count + 1u);
      return;
    }
    const auto message = retry ? "Environmental lighting reflection probe bake timed out: " + error
                               : "Environmental lighting reflection probe bake failed: " + error;
    EVOENGINE_ERROR(message)
  });
}

bool RenderLayer::BakeReflectionProbe(const std::shared_ptr<Scene>& scene, const glm::vec3& position,
                                      const std::shared_ptr<GlobalReflectionProbe>& target,
                                      uint64_t& source_fingerprint, std::string& error, bool& retry) {
  retry = false;
  source_fingerprint = 0;
  error.clear();
  if (!scene || scene.get() != GetScene().get()) {
    error = "Reflection probes can only bake the active scene.";
    return false;
  }
  if (!target) {
    error = "Assign a persistent GlobalReflectionProbe asset before baking.";
    return false;
  }
  if (const auto global = GetAssignedGlobalReflectionProbe(scene);
      global && (global == target || global->GetHandle() == target->GetHandle())) {
    error = "The scene's global reflection probe cannot also be a local bake output.";
    return false;
  }
  if (target->IsTemporary()) {
    error = "Assign a persistent GlobalReflectionProbe asset before baking.";
    return false;
  }
  if (!Platform::Initialized()) {
    error = "The Vulkan renderer is not initialized.";
    return false;
  }
  if (!ProjectManager::IsProjectIdle() || AssetManager::GetAssetLoadSnapshot().Active() ||
      TextureStorage::HasPendingUploads() || GeometryStorage::HasPendingUploads()) {
    retry = true;
    error = "Waiting for pending project, asset, texture, or geometry work.";
    return false;
  }
  const auto ddgi_settings = GetDdgiSettings();
  const auto ddgi_capture_pending =
      ddgi_settings.runtime.enabled &&
      std::any_of(ddgi_ordered_volume_ids_.begin(), ddgi_ordered_volume_ids_.end(),
                  [&](const uint64_t stable_entity_id) {
                    const auto found = ddgi_volume_runtime_states_.find(stable_entity_id);
                    if (found == ddgi_volume_runtime_states_.end()) {
                      return true;
                    }
                    const auto& runtime = *found->second;
                    return !IsDdgiReflectionProbeRuntimeReady(
                        runtime.has_valid_probe_history, runtime.last_performance_stats.lighting_descriptors_bound,
                        runtime.probe_variability_gating_enabled, runtime.probe_variability_converged);
                  });
  if (ddgi_capture_pending) {
    if (ddgi_settings.runtime.pause_updates) {
      error = "Active DDGI is paused before reflection-probe capture became ready.";
      return false;
    }
    retry = true;
    error = "Waiting for active DDGI history and convergence.";
    return false;
  }
  source_fingerprint = GetReflectionProbeCaptureFingerprint(scene, position);
  if (source_fingerprint == 0u) {
    retry = true;
    error = "Waiting for the active scene render snapshot.";
    return false;
  }

  Platform::WaitForFrameSubmissions("Reflection Probe Bake Fence Wait");
  try {
    const auto camera = Serialization::ProduceSerializable<Camera>();
    if (!camera) {
      error = "Failed to create the reflection probe capture camera.";
      return false;
    }
    camera->OnCreate();
    camera->camera_render_mode = Camera::CameraRenderMode::Rasterization;
    camera->camera_settings.fov = 180.0f;
    camera->camera_settings.near_distance = GlobalReflectionProbe::kBakeNearPlane;
    camera->camera_settings.far_distance = GlobalReflectionProbe::kBakeFarPlane;
    camera->camera_settings.fade_ratio = 1.0f;
    const auto resolved_lighting = ResolveEnvironmentalLighting(scene);
    camera->camera_settings.background_intensity = glm::max(resolved_lighting.environment_lighting_intensity, 0.0f);
    const auto& indirect_source = resolved_lighting.indirect_environment_source;
    if (indirect_source.kind == ResolvedEnvironmentalLighting::IndirectEnvironmentSourceKind::Color) {
      camera->camera_settings.background_source = Camera::BackgroundSource::ClearColor;
      camera->camera_settings.clear_color = glm::vec4(indirect_source.color, 1.0f);
    } else {
      camera->camera_settings.background_source = Camera::BackgroundSource::Cubemap;
      if (const auto environmental_map = ResolveIndirectEnvironmentMap(indirect_source)) {
        environmental_map->EnsureEnvironmentSource();
        auto cubemap_ref = environmental_map->environment_cubemap;
        if (const auto environment_cubemap = cubemap_ref.Get<Cubemap>()) {
          camera->skybox.Set(environment_cubemap);
        }
      }
    }
    camera->Resize({GlobalReflectionProbe::kResolution, GlobalReflectionProbe::kResolution});
    if (!camera->GetRenderTexture() || !camera->GetRenderTexture()->GetColorImage()) {
      error = "Failed to allocate the reflection probe capture target.";
      return false;
    }

    const auto raw_cubemap = AssetManager::CreateTemporaryAsset<Cubemap>();
    raw_cubemap->Initialize(GlobalReflectionProbe::kResolution, GlobalReflectionProbe::kMipLevels,
                            camera->GetRenderTexture()->GetColorImage()->GetFormat());
    if (!raw_cubemap->GetImage()) {
      error = "Failed to allocate the reflection probe cubemap.";
      return false;
    }
    raw_cubemap->BeginGpuWrite();
    Platform::ImmediateSubmit([&](const VkCommandBuffer command_buffer) {
      raw_cubemap->GetImage()->TransitImageLayout(command_buffer, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL);
    });

    constexpr std::array<glm::vec3, 6> directions = {glm::vec3(1, 0, 0),  glm::vec3(-1, 0, 0), glm::vec3(0, 1, 0),
                                                     glm::vec3(0, -1, 0), glm::vec3(0, 0, 1),  glm::vec3(0, 0, -1)};
    constexpr std::array<glm::vec3, 6> up_directions = {glm::vec3(0, -1, 0), glm::vec3(0, -1, 0), glm::vec3(0, 0, 1),
                                                        glm::vec3(0, 0, -1), glm::vec3(0, -1, 0), glm::vec3(0, -1, 0)};
    for (uint32_t face = 0; face < directions.size(); ++face) {
      GlobalTransform camera_transform;
      camera_transform.SetValue(position, glm::quatLookAt(directions[face], up_directions[face]), glm::vec3(1.0f));
      RenderSceneToCameraImmediately(scene, camera_transform, camera, true);

      const auto source_image = camera->GetRenderTexture()->GetColorImage();
      const auto source_layout = source_image->GetLayout();
      Platform::ImmediateSubmit([&](const VkCommandBuffer command_buffer) {
        source_image->TransitImageLayout(command_buffer, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL);
        VkImageCopy copy{};
        copy.srcSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
        copy.srcSubresource.layerCount = 1;
        copy.dstSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
        copy.dstSubresource.baseArrayLayer = face;
        copy.dstSubresource.layerCount = 1;
        copy.extent = {GlobalReflectionProbe::kResolution, GlobalReflectionProbe::kResolution, 1};
        vkCmdCopyImage(command_buffer, source_image->GetVkImage(), VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,
                       raw_cubemap->GetImage()->GetVkImage(), VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, 1, &copy);
        source_image->TransitImageLayout(command_buffer, source_layout);
      });
    }
    Platform::ImmediateSubmit([&](const VkCommandBuffer command_buffer) {
      raw_cubemap->GetImage()->GenerateMipmaps(command_buffer);
    });
    raw_cubemap->MarkGpuContentValid();

    const auto filtered = AssetManager::CreateTemporaryAsset<GlobalReflectionProbe>();
    if (!filtered->ConstructFromCubemap(raw_cubemap)) {
      error = "Failed to prefilter the baked reflection probe payload.";
      return false;
    }
    const auto& payload = filtered->GetCanonicalPayload();
    if (payload.size() * sizeof(uint16_t) != GlobalReflectionProbe::kCanonicalPayloadByteSize) {
      error = "The baked reflection probe payload has an invalid byte count.";
      return false;
    }
    if (!GlobalReflectionProbe::ValidateCanonicalPayload(payload, error)) {
      return false;
    }
    filtered->MarkBaked(source_fingerprint);
    if (!Serialization::SaveAsset(*filtered, target->GetAbsolutePath())) {
      error = "The reflection probe bake could not be saved; the previous asset remains active.";
      return false;
    }
    if (!target->Load()) {
      error = "The reflection probe was saved but could not be published to the active asset.";
      return false;
    }
    return true;
  } catch (const std::exception& exception) {
    error = exception.what();
    return false;
  }
}

void RenderLayer::RenderAll() {
  const ProfilerScope profiler_scope("RenderLayer::RenderAll", "Render");
  const auto scene = GetScene();
  const auto current_frame_index = Platform::GetCurrentFrameIndex();
  const auto current_render_instances = render_instances_list_[current_frame_index];
  const auto lighting_descriptor_set =
      lighting_ ? lighting_->lighting_descriptor_sets_.at(current_frame_index) : nullptr;
  const auto ddgi_settings = ResolveEnvironmentalLighting(scene).ddgi_settings;
  PruneRayCameraHistories(current_render_instances);
  auto& current_frame_transient_resources = render_graph_transient_resource_stores_.at(current_frame_index);
  current_frame_transient_resources.clear();
  if (!ddgi_fallback_probe_state_buffer_) {
    ddgi_fallback_probe_state_buffer_ =
        CreateDdgiFallbackProbeStateBuffer(kDdgiMaxResidentProbeCount * sizeof(glm::vec4));
  }
  BindDdgiFallbackLightingDescriptors(lighting_descriptor_set, ddgi_fallback_probe_state_buffer_);
  ddgi_last_performance_stats_ = {};
  bool has_ddgi_performance_stats = false;
  const auto accumulate_ddgi_performance_stats = [&](const DdgiPerformanceStats& volume_stats) {
    if (!has_ddgi_performance_stats) {
      ddgi_last_performance_stats_ = volume_stats;
      has_ddgi_performance_stats = true;
      return;
    }
    auto& aggregate = ddgi_last_performance_stats_;
    const auto old_variability_sample_count = aggregate.probe_variability_sample_count;
    const auto variability_sample_count = old_variability_sample_count + volume_stats.probe_variability_sample_count;
    if (variability_sample_count > 0u) {
      aggregate.probe_variability_average =
          (aggregate.probe_variability_average * static_cast<float>(old_variability_sample_count) +
           volume_stats.probe_variability_average * static_cast<float>(volume_stats.probe_variability_sample_count)) /
          static_cast<float>(variability_sample_count);
    }
    aggregate.active_probe_count += volume_stats.active_probe_count;
    aggregate.storage_probe_count += volume_stats.storage_probe_count;
    aggregate.updated_probe_count += volume_stats.updated_probe_count;
    aggregate.ray_count = glm::max(aggregate.ray_count, volume_stats.ray_count);
    aggregate.ray_sample_count += volume_stats.ray_sample_count;
    aggregate.emissive_triangle_count =
        glm::max(aggregate.emissive_triangle_count, volume_stats.emissive_triangle_count);
    aggregate.emissive_sampling_enabled_volume_count += volume_stats.emissive_sampling_enabled_volume_count;
    aggregate.emissive_sampling_candidate_ray_count += volume_stats.emissive_sampling_candidate_ray_count;
    aggregate.recorded_ray_sample_count += volume_stats.recorded_ray_sample_count;
    aggregate.recorded_probe_update_count += volume_stats.recorded_probe_update_count;
    aggregate.selected_ray_sample_count += volume_stats.selected_ray_sample_count;
    aggregate.probe_metadata_byte_size += volume_stats.probe_metadata_byte_size;
    aggregate.probe_state_byte_size += volume_stats.probe_state_byte_size;
    aggregate.ray_output_byte_size += volume_stats.ray_output_byte_size;
    aggregate.selected_ray_diagnostics_byte_size += volume_stats.selected_ray_diagnostics_byte_size;
    aggregate.irradiance_atlas_byte_size += volume_stats.irradiance_atlas_byte_size;
    aggregate.visibility_atlas_byte_size += volume_stats.visibility_atlas_byte_size;
    aggregate.variability_atlas_byte_size += volume_stats.variability_atlas_byte_size;
    aggregate.variability_reduction_byte_size += volume_stats.variability_reduction_byte_size;
    aggregate.persistent_byte_size += volume_stats.persistent_byte_size;
    aggregate.per_frame_transient_byte_size += volume_stats.per_frame_transient_byte_size;
    aggregate.peak_resident_byte_size += volume_stats.peak_resident_byte_size;
    aggregate.probe_variability_sample_count = variability_sample_count;
    aggregate.probe_variability_stable_sample_count =
        glm::min(aggregate.probe_variability_stable_sample_count, volume_stats.probe_variability_stable_sample_count);
    aggregate.probe_variability_required_stable_sample_count =
        glm::max(aggregate.probe_variability_required_stable_sample_count,
                 volume_stats.probe_variability_required_stable_sample_count);
    aggregate.probe_variability_converged &= volume_stats.probe_variability_converged;
    aggregate.probe_variability_refresh_age =
        glm::max(aggregate.probe_variability_refresh_age, volume_stats.probe_variability_refresh_age);
    aggregate.probe_variability_refresh_waiting |= volume_stats.probe_variability_refresh_waiting;
    aggregate.probe_warmup_active |= volume_stats.probe_warmup_active;
    aggregate.lighting_descriptors_bound &= volume_stats.lighting_descriptors_bound;
    aggregate.atlas_prepare_record_ms += volume_stats.atlas_prepare_record_ms;
    aggregate.ray_diagnostics_record_ms += volume_stats.ray_diagnostics_record_ms;
    aggregate.probe_update_record_ms += volume_stats.probe_update_record_ms;
    aggregate.probe_relocation_record_ms += volume_stats.probe_relocation_record_ms;
    aggregate.probe_classification_record_ms += volume_stats.probe_classification_record_ms;
    aggregate.probe_variability_record_ms += volume_stats.probe_variability_record_ms;
    aggregate.frame_graph_execute_ms += volume_stats.frame_graph_execute_ms;
  };

  const auto execute_ddgi_runtime = [&](DdgiVolumeRuntimeState& ddgi_runtime, const uint32_t volume_slot,
                                        const bool include_external_passes) {
    ddgi_runtime.frame_selected_ray_diagnostics_buffer.reset();
    ddgi_runtime.last_performance_stats = {};
    ddgi_runtime.last_performance_stats.storage_probe_count = ddgi_runtime.frame_resource_layout.probe_count;
    ddgi_runtime.last_performance_stats.updated_probe_count =
        ddgi_runtime.frame_trace_probe_rays ? ddgi_runtime.frame_resource_layout.probe_count : 0u;
    ddgi_runtime.last_performance_stats.selected_ray_sample_count = ddgi_runtime.frame_selected_probe_ray_sample_count;
    ddgi_runtime.last_performance_stats.emissive_triangle_count =
        current_render_instances ? current_render_instances->render_info_block.emissive_triangle_parameters.x : 0u;
    ddgi_runtime.last_performance_stats.emissive_sampling_enabled_volume_count =
        ddgi_runtime.emissive_mesh_sampling_enabled ? 1u : 0u;
    ddgi_runtime.last_performance_stats.probe_metadata_byte_size =
        ddgi_runtime.frame_resource_layout.probe_metadata_byte_size;
    ddgi_runtime.last_performance_stats.probe_state_byte_size =
        ddgi_runtime.frame_resource_layout.probe_state_byte_size;
    ddgi_runtime.last_performance_stats.ray_output_byte_size = ddgi_runtime.frame_resource_layout.ray_output_byte_size;
    ddgi_runtime.last_performance_stats.selected_ray_diagnostics_byte_size =
        ddgi_runtime.frame_resource_layout.selected_ray_diagnostics_byte_size;
    ddgi_runtime.last_performance_stats.irradiance_atlas_byte_size =
        ddgi_runtime.frame_resource_layout.irradiance_atlas_byte_size;
    ddgi_runtime.last_performance_stats.visibility_atlas_byte_size =
        ddgi_runtime.frame_resource_layout.visibility_atlas_byte_size;
    ddgi_runtime.last_performance_stats.variability_atlas_byte_size =
        ddgi_runtime.frame_resource_layout.variability_atlas_byte_size;
    ddgi_runtime.last_performance_stats.variability_reduction_byte_size =
        ddgi_runtime.frame_resource_layout.variability_reduction_byte_size;
    ddgi_runtime.last_performance_stats.persistent_byte_size = ddgi_runtime.frame_resource_layout.persistent_byte_size;
    ddgi_runtime.last_performance_stats.per_frame_transient_byte_size =
        ddgi_runtime.frame_resource_layout.per_frame_transient_byte_size;
    ddgi_runtime.last_performance_stats.peak_resident_byte_size =
        ddgi_runtime.frame_resource_layout.peak_resident_byte_size;
    ddgi_runtime.last_performance_stats.irradiance_atlas_extent =
        ddgi_runtime.frame_resource_layout.irradiance_atlas.resolution;
    ddgi_runtime.last_performance_stats.visibility_atlas_extent =
        ddgi_runtime.frame_resource_layout.visibility_atlas.resolution;
    ddgi_runtime.last_performance_stats.variability_atlas_extent =
        ddgi_runtime.frame_resource_layout.variability_atlas.resolution;
    ddgi_runtime.last_performance_stats.variability_reduction_extent =
        ddgi_runtime.frame_resource_layout.variability_reduction_extent;
    ddgi_runtime.last_performance_stats.probe_variability_average = ddgi_runtime.probe_variability_average;
    ddgi_runtime.last_performance_stats.probe_variability_sample_count = ddgi_runtime.probe_variability_sample_count;
    ddgi_runtime.last_performance_stats.probe_variability_stable_sample_count =
        ddgi_runtime.probe_variability_stable_sample_count;
    ddgi_runtime.last_performance_stats.probe_variability_required_stable_sample_count =
        kDdgiProbeVariabilityStableSampleCount;
    ddgi_runtime.last_performance_stats.probe_variability_converged = ddgi_runtime.probe_variability_converged;
    ddgi_runtime.last_performance_stats.probe_variability_refresh_age = ddgi_runtime.probe_variability_refresh_age;
    ddgi_runtime.last_performance_stats.probe_variability_refresh_waiting =
        ddgi_runtime.probe_variability_refresh_waiting;
    ddgi_runtime.last_performance_stats.probe_warmup_frame_index = ddgi_runtime.frame_probe_warmup_frame_index;
    ddgi_runtime.last_performance_stats.probe_warmup_frame_count = ddgi_runtime.frame_probe_warmup_frame_count;
    ddgi_runtime.last_performance_stats.probe_warmup_active = ddgi_runtime.frame_probe_warmup_active;
    ddgi_runtime.last_performance_stats.probe_update_hysteresis = ddgi_runtime.frame_probe_update_hysteresis;
    ddgi_runtime.last_performance_stats.active_probe_count =
        ddgi_runtime.contributes_lighting ? ddgi_runtime.frame_resource_layout.probe_count : 0u;
    if (ddgi_runtime.frame_trace_probe_rays) {
      ddgi_runtime.last_performance_stats.ray_count =
          static_cast<uint32_t>(glm::max(ddgi_settings.runtime.ray_count, 1));
      ddgi_runtime.last_performance_stats.ray_sample_count =
          ddgi_runtime.last_performance_stats.updated_probe_count * ddgi_runtime.last_performance_stats.ray_count;
      const auto fixed_ray_count =
          static_cast<uint32_t>(glm::max(ddgi_runtime.frame_ray_push_constant.trace_parameters.z, 0.0f));
      ddgi_runtime.last_performance_stats.emissive_sampling_candidate_ray_count =
          CalculateDdgiEmissiveSamplingCandidateRayCount(
              ddgi_runtime.last_performance_stats.updated_probe_count, ddgi_runtime.last_performance_stats.ray_count,
              fixed_ray_count, ddgi_runtime.emissive_mesh_sampling_enabled, ddgi_runtime.frame_trace_probe_rays);
    }
    RenderGraph frame_render_graph;
    RenderGraphTransientResourceStore* active_frame_transient_resources = nullptr;
    AddDefaultFrameResources(frame_render_graph);
    AddAdvancedFrameResources(frame_render_graph);
    const auto trace_ddgi_probe_rays = ddgi_runtime.frame_trace_probe_rays;
    const auto use_ddgi_frame_resources =
        ShouldUseDdgiFrameResources(ddgi_settings) && ddgi_runtime.frame_resource_layout.valid;
    if (use_ddgi_frame_resources) {
      const auto clear_ddgi_probe_atlas = ddgi_runtime.clear_probe_atlas_this_frame;
      const auto clear_scrolled_probes = ddgi_runtime.frame_clear_scrolled_probes;
      const auto ddgi_probe_scroll_push_constant = ddgi_runtime.frame_probe_scroll_push_constant;
      AddDdgiFrameResources(frame_render_graph, ddgi_runtime.frame_resource_layout);
      if (clear_ddgi_probe_atlas) {
        frame_render_graph.AddPass(
            DdgiAtlasPreparePass::CreateDescriptor(), [&](const RenderGraphExecutionContext& context) {
              DdgiAtlasPreparePass::Execute(context, {&ddgi_runtime.last_performance_stats.atlas_prepare_record_ms});
            });
      }
      if (clear_scrolled_probes) {
        frame_render_graph.AddPass(DdgiProbeScrollPass::CreateDescriptor(),
                                   [&, ddgi_probe_scroll_push_constant](const RenderGraphExecutionContext& context) {
                                     DdgiProbeScrollPass::Execute(
                                         context, {ddgi_probe_scroll_pipeline_, ddgi_probe_update_layout_,
                                                   active_frame_transient_resources, ddgi_probe_scroll_push_constant,
                                                   &ddgi_runtime.last_performance_stats.atlas_prepare_record_ms});
                                   });
      }
    }
    const auto reset_ddgi_probe_relocation = ddgi_runtime.frame_probe_relocation_reset;
    const auto relocate_ddgi_probes = ddgi_runtime.frame_probe_relocation_enabled;
    const auto reset_ddgi_probe_classification = ddgi_runtime.frame_probe_classification_reset;
    const auto classify_ddgi_probes = ddgi_runtime.frame_probe_classification_enabled;
    const auto reduce_ddgi_probe_variability = ddgi_runtime.frame_probe_variability_enabled;
    const auto ddgi_ray_push_constant = ddgi_runtime.frame_ray_push_constant;
    const auto ddgi_probe_update_push_constant = ddgi_runtime.frame_probe_update_push_constant;
    const auto ddgi_probe_relocation_reset_push_constant = ddgi_runtime.frame_probe_relocation_reset_push_constant;
    const auto ddgi_probe_relocation_update_push_constant = ddgi_runtime.frame_probe_relocation_update_push_constant;
    const auto ddgi_probe_classification_reset_push_constant =
        ddgi_runtime.frame_probe_classification_reset_push_constant;
    const auto ddgi_probe_classification_update_push_constant =
        ddgi_runtime.frame_probe_classification_update_push_constant;
    const DdgiProbeVariabilityPass::DdgiAtlasLayout ddgi_probe_variability_layout{
        ddgi_runtime.frame_resource_layout.probe_count,
        ddgi_runtime.frame_resource_layout.variability_atlas.tile_resolution,
        ddgi_runtime.frame_resource_layout.variability_atlas.columns,
        ddgi_runtime.frame_resource_layout.variability_atlas.resolution,
        ddgi_runtime.frame_resource_layout.variability_reduction_extent};
    const auto ddgi_probe_metadata_readback_buffer =
        ddgi_settings.debug.enabled && ddgi_settings.debug.visualize_probe_state &&
                current_frame_index < ddgi_runtime.probe_metadata_readback_buffers.size()
            ? ddgi_runtime.probe_metadata_readback_buffers[current_frame_index]
            : nullptr;
    const auto ddgi_probe_ray_readback_buffer =
        ddgi_settings.debug.enabled && ddgi_settings.debug.show_rays &&
                ddgi_runtime.frame_selected_probe_ray_sample_count != 0u &&
                current_frame_index < ddgi_runtime.probe_ray_readback_buffers.size()
            ? ddgi_runtime.probe_ray_readback_buffers[current_frame_index]
            : nullptr;
    const auto ddgi_selected_ray_diagnostics_buffer =
        current_frame_index < ddgi_runtime.selected_ray_diagnostics_buffers.size()
            ? ddgi_runtime.selected_ray_diagnostics_buffers[current_frame_index]
            : nullptr;
    const auto ddgi_variability_readback_buffer =
        current_frame_index < ddgi_runtime.variability_readback_tickets.size()
            ? ddgi_runtime.variability_readback_tickets[current_frame_index].buffer
            : nullptr;
    bool metadata_readback_recorded = false;
    bool selected_ray_readback_recorded = false;
    bool variability_readback_recorded = false;
    if (trace_ddgi_probe_rays) {
      AddDdgiRayTracingFrameResources(frame_render_graph);
      frame_render_graph.AddPass(
          DdgiRayDiagnosticsPass::CreateDescriptor(),
          [&, ddgi_ray_push_constant](const RenderGraphExecutionContext& context) {
            DdgiRayDiagnosticsPass::Execute(
                context, {ddgi_probe_ray_diagnostic_pipeline_, per_frame_descriptor_sets_[current_frame_index],
                          ray_tracing_descriptor_sets_[current_frame_index], ddgi_probe_ray_output_layout_,
                          active_frame_transient_resources, ddgi_atlas_sampler_, ddgi_ray_push_constant,
                          ddgi_runtime.frame_resource_layout.probe_count, ddgi_probe_ray_readback_buffer,
                          ddgi_runtime.frame_selected_probe_ray_sample_count, &selected_ray_readback_recorded,
                          &ddgi_runtime.last_performance_stats.recorded_ray_sample_count,
                          &ddgi_runtime.last_performance_stats.ray_diagnostics_record_ms});
          });
      frame_render_graph.AddPass(
          DdgiProbeUpdatePass::CreateDescriptor(),
          [&, ddgi_probe_update_push_constant](const RenderGraphExecutionContext& context) {
            DdgiProbeUpdatePass::Execute(
                context,
                {ddgi_probe_update_pipeline_, ddgi_probe_update_irradiance_pipeline_,
                 ddgi_probe_update_visibility_pipeline_, ddgi_probe_update_variant_ != DdgiProbeUpdateVariant::Serial,
                 per_frame_descriptor_sets_[current_frame_index], ddgi_probe_update_layout_,
                 active_frame_transient_resources, ddgi_probe_update_push_constant, ddgi_probe_metadata_readback_buffer,
                 &metadata_readback_recorded, &ddgi_runtime.last_performance_stats.recorded_probe_update_count,
                 &ddgi_runtime.last_performance_stats.probe_update_record_ms, &ddgi_probe_update_path_reported_});
          });
      if (reset_ddgi_probe_relocation || relocate_ddgi_probes) {
        frame_render_graph.AddPass(
            DdgiProbeRelocationPass::CreateDescriptor(),
            [&, ddgi_probe_relocation_reset_push_constant, ddgi_probe_relocation_update_push_constant,
             reset_ddgi_probe_relocation, relocate_ddgi_probes](const RenderGraphExecutionContext& context) {
              DdgiProbeRelocationPass::Execute(
                  context, {ddgi_probe_relocation_pipeline_, ddgi_probe_relocation_layout_,
                            active_frame_transient_resources, ddgi_probe_relocation_reset_push_constant,
                            ddgi_probe_relocation_update_push_constant, reset_ddgi_probe_relocation,
                            relocate_ddgi_probes, &ddgi_runtime.last_performance_stats.probe_relocation_record_ms});
            });
      }
      if (reset_ddgi_probe_classification || classify_ddgi_probes) {
        frame_render_graph.AddPass(
            DdgiProbeClassificationPass::CreateDescriptor(),
            [&, ddgi_probe_classification_reset_push_constant, ddgi_probe_classification_update_push_constant,
             reset_ddgi_probe_classification, classify_ddgi_probes](const RenderGraphExecutionContext& context) {
              DdgiProbeClassificationPass::Execute(
                  context, {ddgi_probe_classification_pipeline_, ddgi_probe_classification_layout_,
                            active_frame_transient_resources, ddgi_probe_classification_reset_push_constant,
                            ddgi_probe_classification_update_push_constant, reset_ddgi_probe_classification,
                            classify_ddgi_probes, &ddgi_runtime.last_performance_stats.probe_classification_record_ms});
            });
      }
      if (reduce_ddgi_probe_variability) {
        frame_render_graph.AddPass(
            DdgiProbeVariabilityPass::CreateDescriptor(),
            [&, ddgi_probe_variability_layout](const RenderGraphExecutionContext& context) {
              DdgiProbeVariabilityPass::Execute(
                  context,
                  {ddgi_probe_variability_reduce_pipeline_, ddgi_probe_variability_extra_reduce_pipeline_,
                   ddgi_probe_variability_layout_, active_frame_transient_resources, ddgi_probe_variability_layout,
                   ddgi_variability_readback_buffer, &variability_readback_recorded,
                   &ddgi_runtime.last_performance_stats.probe_variability_record_ms});
            });
      }
    }
    if (include_external_passes) {
      RenderPassDescriptor ddgi_volumes_complete{RenderPassNames::ddgi_volumes_complete, RenderPassQueue::Graphics,
                                                 RenderPassScope::Frame};
      for (const auto& pass : frame_render_graph.GetPasses()) {
        ddgi_volumes_complete.dependencies.push_back(pass.name);
      }
      frame_render_graph.AddPass(ddgi_volumes_complete, [](const RenderGraphExecutionContext&) {
      });
      AddExternalRenderResources(frame_render_graph, external_render_resource_descriptors);
      for (const auto& external_pass : frame_render_pass_external_functions) {
        auto descriptor = external_pass.descriptor;
        if (std::find(descriptor.dependencies.begin(), descriptor.dependencies.end(),
                      RenderPassNames::ddgi_volumes_complete) == descriptor.dependencies.end()) {
          descriptor.dependencies.emplace_back(RenderPassNames::ddgi_volumes_complete);
        }
        ImportMissingPassResources(frame_render_graph, descriptor);
        frame_render_graph.AddPass(descriptor, [&, external_pass](const RenderGraphExecutionContext& context) {
          Platform::RecordCommandsMainQueue([&, external_pass](VkCommandBuffer vk_command_buffer) {
            uint32_t prim_count = 0;
            if (external_pass.context_func) {
              ApplyGraphResourceBarriers(vk_command_buffer, context);
              prim_count = external_pass.context_func(vk_command_buffer, context);
              ApplyGraphResourceReleaseBarriers(vk_command_buffer, context, RenderPassQueue::Graphics);
            } else if (external_pass.func) {
              prim_count = external_pass.func(vk_command_buffer);
            }
            if (count_shadow_rendering_draw_calls) {
              Platform::CountRenderPassDraw(RenderPassDrawBucket::FrameExternal, RenderDrawCallKind::Direct,
                                            current_frame_index, prim_count);
            }
          });
        });
      }
    }
    if (!frame_render_graph.Validate()) {
      EVOENGINE_ERROR("Invalid frame render graph.")
    }
    const auto frame_render_graph_plan = frame_render_graph.Compile(CreateFrameRenderGraphCompileContext());
    auto frame_render_graph_resources =
        CreateFrameRenderGraphResourceRegistry(per_frame_descriptor_sets_[current_frame_index]);
    if (use_ddgi_frame_resources) {
      frame_render_graph_resources.BindBuffer(RenderResourceNames::frame_ddgi_probe_metadata,
                                              ddgi_runtime.probe_metadata_buffer);
      frame_render_graph_resources.BindBuffer(
          RenderResourceNames::frame_ddgi_probe_state,
          ddgi_runtime.probe_state_buffer ? ddgi_runtime.probe_state_buffer : ddgi_fallback_probe_state_buffer_);
      frame_render_graph_resources.BindImage(RenderResourceNames::frame_ddgi_irradiance_atlas,
                                             ddgi_runtime.irradiance_atlas);
      frame_render_graph_resources.BindImage(RenderResourceNames::frame_ddgi_visibility_atlas,
                                             ddgi_runtime.visibility_atlas);
      frame_render_graph_resources.BindImage(RenderResourceNames::frame_ddgi_variability_atlas,
                                             ddgi_runtime.variability_atlas);
    }
    if (trace_ddgi_probe_rays) {
      frame_render_graph_resources.BindDescriptorSet(RenderResourceNames::frame_ray_tracing_descriptor_set,
                                                     ray_tracing_descriptor_sets_[current_frame_index]);
      frame_render_graph_resources.BindBuffer(RenderResourceNames::frame_ddgi_probe_state,
                                              ddgi_runtime.probe_state_buffer);
      frame_render_graph_resources.BindBuffer(RenderResourceNames::frame_ddgi_selected_ray_diagnostics,
                                              ddgi_selected_ray_diagnostics_buffer);
    }
    auto& frame_transient_resources = current_frame_transient_resources.emplace_back();
    active_frame_transient_resources = &frame_transient_resources;
    frame_transient_resources.Allocate(frame_render_graph.GetResources(), frame_render_graph_plan);
    frame_transient_resources.Bind(frame_render_graph_resources);
    const bool synchronize_ddgi_frame_resources =
        use_ddgi_frame_resources && (ddgi_runtime.clear_probe_atlas_this_frame ||
                                     ddgi_runtime.frame_clear_scrolled_probes || trace_ddgi_probe_rays);
    std::shared_ptr<Image> frame_ddgi_irradiance_atlas;
    std::shared_ptr<Image> frame_ddgi_visibility_atlas;
    std::shared_ptr<Image> frame_ddgi_variability_atlas;
    std::shared_ptr<Buffer> frame_ddgi_probe_state;
    std::shared_ptr<Buffer> frame_ddgi_probe_metadata;
    std::shared_ptr<Buffer> frame_ddgi_selected_ray_diagnostics;
    if (synchronize_ddgi_frame_resources) {
      const auto* irradiance_binding =
          frame_render_graph_resources.GetResourceBinding(RenderResourceNames::frame_ddgi_irradiance_atlas);
      const auto* visibility_binding =
          frame_render_graph_resources.GetResourceBinding(RenderResourceNames::frame_ddgi_visibility_atlas);
      const auto* variability_binding =
          frame_render_graph_resources.GetResourceBinding(RenderResourceNames::frame_ddgi_variability_atlas);
      const auto* state_binding =
          frame_render_graph_resources.GetResourceBinding(RenderResourceNames::frame_ddgi_probe_state);
      const auto* metadata_binding =
          frame_render_graph_resources.GetResourceBinding(RenderResourceNames::frame_ddgi_probe_metadata);
      const auto* diagnostics_binding =
          frame_render_graph_resources.GetResourceBinding(RenderResourceNames::frame_ddgi_selected_ray_diagnostics);
      frame_ddgi_irradiance_atlas = irradiance_binding ? irradiance_binding->image : nullptr;
      frame_ddgi_visibility_atlas = visibility_binding ? visibility_binding->image : nullptr;
      frame_ddgi_variability_atlas = variability_binding ? variability_binding->image : nullptr;
      frame_ddgi_probe_state = state_binding ? state_binding->buffer : nullptr;
      frame_ddgi_probe_metadata = metadata_binding ? metadata_binding->buffer : nullptr;
      frame_ddgi_selected_ray_diagnostics = diagnostics_binding ? diagnostics_binding->buffer : nullptr;
      Platform::RecordCommandsMainQueue([=](const VkCommandBuffer command_buffer) {
        AcquireDdgiFrameResources(command_buffer, frame_ddgi_irradiance_atlas, frame_ddgi_visibility_atlas,
                                  frame_ddgi_variability_atlas, frame_ddgi_probe_state, frame_ddgi_probe_metadata,
                                  frame_ddgi_selected_ray_diagnostics);
      });
    }
    const auto ddgi_frame_graph_timer = DdgiPerformanceClock::now();
    frame_render_graph.Execute(frame_render_graph_plan, frame_render_graph_resources);
    ddgi_runtime.last_performance_stats.frame_graph_execute_ms = DdgiElapsedMilliseconds(ddgi_frame_graph_timer);
    std::shared_ptr<FrameSubmissionState> readback_submission;
    if (metadata_readback_recorded || selected_ray_readback_recorded || variability_readback_recorded) {
      readback_submission = Platform::TrackCurrentFrameSubmission();
    }
    if (metadata_readback_recorded) {
      auto& ticket = ddgi_runtime.metadata_readback_ticket;
      ticket = {};
      ticket.buffer = ddgi_probe_metadata_readback_buffer;
      ticket.submission = readback_submission;
      ticket.generation = ++ddgi_runtime.next_debug_readback_generation;
      ticket.byte_size = ddgi_runtime.frame_resource_layout.probe_metadata_byte_size;
      ticket.frame_index = current_frame_index;
      ticket.element_count = ddgi_runtime.frame_resource_layout.probe_count;
    }
    if (selected_ray_readback_recorded) {
      auto& ticket = ddgi_runtime.ray_readback_ticket;
      ticket = {};
      ticket.buffer = ddgi_probe_ray_readback_buffer;
      ticket.submission = readback_submission;
      ticket.generation = ++ddgi_runtime.next_debug_readback_generation;
      ticket.byte_size =
          static_cast<uint64_t>(ddgi_runtime.frame_selected_probe_ray_sample_count) * sizeof(PointCloudSample);
      ticket.frame_index = current_frame_index;
      ticket.element_count = ddgi_runtime.frame_selected_probe_ray_sample_count;
      ticket.logical_probe_index = ddgi_runtime.frame_selected_probe_ray_logical_index;
      ticket.physical_probe_index = ddgi_runtime.frame_selected_probe_ray_physical_index;
    }
    if (variability_readback_recorded && current_frame_index < ddgi_runtime.variability_readback_tickets.size()) {
      auto& ticket = ddgi_runtime.variability_readback_tickets[current_frame_index];
      ticket = {};
      ticket.buffer = ddgi_variability_readback_buffer;
      ticket.submission = readback_submission;
      ticket.generation = ddgi_runtime.frame_variability_readback_generation;
      ticket.byte_size = sizeof(glm::vec2);
      ticket.frame_index = current_frame_index;
      ticket.element_count = 1u;
    } else if (ddgi_runtime.frame_variability_readback_generation != 0u) {
      ddgi_runtime.last_consumed_variability_generation = std::max(ddgi_runtime.last_consumed_variability_generation,
                                                                   ddgi_runtime.frame_variability_readback_generation);
      if (ddgi_runtime.probe_variability_refresh_waiting &&
          ddgi_runtime.frame_variability_readback_generation >= ddgi_runtime.probe_variability_refresh_generation) {
        ddgi_runtime.probe_variability_refresh_waiting = false;
        ddgi_runtime.probe_variability_refresh_generation = 0u;
        ddgi_runtime.probe_variability_refresh_age = kDdgiProbeRefreshInterval;
      }
    }
    if (trace_ddgi_probe_rays && !reduce_ddgi_probe_variability && Platform::GpuTimestampCaptureEnabled()) {
      Platform::RecordCommandsMainQueue([](const VkCommandBuffer command_buffer) {
        const auto gpu_timestamp = Platform::BeginGpuTimestampScope(command_buffer, "DDGI Variability Reduction");
        Platform::EndGpuTimestampScope(command_buffer, gpu_timestamp);
      });
    }
    if (synchronize_ddgi_frame_resources) {
      Platform::RecordCommandsMainQueue([=](const VkCommandBuffer command_buffer) {
        PublishDdgiFrameResources(command_buffer, frame_ddgi_irradiance_atlas, frame_ddgi_visibility_atlas,
                                  frame_ddgi_probe_state, frame_ddgi_probe_metadata,
                                  frame_ddgi_selected_ray_diagnostics);
      });
    }
    if (use_ddgi_frame_resources) {
      ddgi_runtime.last_performance_stats.lighting_descriptors_bound = BindDdgiAtlasLightingDescriptors(
          frame_render_graph_resources, frame_transient_resources, lighting_descriptor_set, ddgi_atlas_sampler_,
          ddgi_fallback_probe_state_buffer_, volume_slot);
    }
    if (trace_ddgi_probe_rays) {
      if (ddgi_settings.debug.enabled && ddgi_settings.debug.show_rays) {
        if (const auto* diagnostics_binding = frame_render_graph_resources.GetResourceBinding(
                RenderResourceNames::frame_ddgi_selected_ray_diagnostics);
            diagnostics_binding && diagnostics_binding->buffer) {
          ddgi_runtime.frame_selected_ray_diagnostics_buffer = diagnostics_binding->buffer;
        }
      }
    }
    active_frame_transient_resources = nullptr;
  };

  if (ddgi_ordered_volume_ids_.empty()) {
    DdgiVolumeRuntimeState empty_runtime;
    execute_ddgi_runtime(empty_runtime, 0u, true);
  } else {
    for (size_t i = ddgi_ordered_volume_ids_.size(); i-- > 0u;) {
      const auto runtime = ddgi_volume_runtime_states_.find(ddgi_ordered_volume_ids_[i]);
      if (runtime == ddgi_volume_runtime_states_.end()) {
        continue;
      }
      execute_ddgi_runtime(*runtime->second, static_cast<uint32_t>(i), i == 0u);
    }
    for (const auto volume_id : ddgi_ordered_volume_ids_) {
      if (const auto runtime = ddgi_volume_runtime_states_.find(volume_id);
          runtime != ddgi_volume_runtime_states_.end()) {
        accumulate_ddgi_performance_stats(runtime->second->last_performance_stats);
      }
    }
  }
  PreparePointAndSpotLightShadowMap();
  for (const auto& [cameraGlobalTransform, camera] : current_render_instances->cameras) {
    camera->rendered_ = false;
    if (camera->require_rendering_) {
      RenderToCamera(scene, cameraGlobalTransform, camera);
    }
  }

  if (Platform::RayAccelerationStructureEnabled() && current_render_instances->mesh_top_level_acceleration_structure) {
    for (const auto& [cameraGlobalTransform, camera] : current_render_instances->cameras) {
      if (camera->require_rendering_) {
        RenderToCameraRayTracing(scene, cameraGlobalTransform, camera);
      }
    }
  }

  external_render_resource_descriptors.clear();
  frame_render_pass_external_functions.clear();
  point_light_shadow_map_external_functions.clear();
  spot_light_shadow_map_external_functions.clear();
  directional_light_shadow_map_external_functions.clear();
  deferred_rendering_external_functions.clear();
  forward_rendering_external_functions.clear();
  camera_render_pass_external_functions.clear();
}

void RenderLayer::RenderGizmos() const {
  if (const auto scene = GetScene(); !scene)
    return;
  if (const auto editor_layer = ApplicationContext::Get().GetLayer<EditorLayer>()) {
    const auto current_frame_index = Platform::GetCurrentFrameIndex();
    const auto current_render_instances = render_instances_list_[Platform::GetCurrentFrameIndex()];
    for (const auto& i : editor_layer->gizmo_mesh_tasks_) {
      if (editor_layer->editor_cameras_.find(i.editor_camera_component->GetHandle()) ==
          editor_layer->editor_cameras_.end()) {
        EVOENGINE_ERROR("Target camera not registered in editor!");
        return;
      }
      if (i.editor_camera_component && i.editor_camera_component->IsEnabled()) {
        Platform::RecordCommandsMainQueue([&](VkCommandBuffer vk_command_buffer) {
          std::shared_ptr<GraphicsPipeline> gizmos_pipeline;
          switch (i.gizmo_settings.color_mode) {
            case GizmoSettings::ColorMode::Default: {
              gizmos_pipeline = gizmos;
            } break;
            case GizmoSettings::ColorMode::VertexColor: {
              gizmos_pipeline = gizmos_vertex_colored;
            } break;
            case GizmoSettings::ColorMode::NormalColor: {
              gizmos_pipeline = gizmos_normal_colored;
            } break;
          }
          i.editor_camera_component->GetRenderTexture()->ApplyGraphicsPipelineStates(gizmos_pipeline->states);
          i.gizmo_settings.ApplySettings(gizmos_pipeline->states);

          gizmos_pipeline->Bind(vk_command_buffer);
          gizmos_pipeline->BindDescriptorSet(vk_command_buffer, 0,
                                             per_frame_descriptor_sets_[current_frame_index]->GetVkDescriptorSet());

          i.editor_camera_component->GetRenderTexture()->Render(
              vk_command_buffer, VK_ATTACHMENT_LOAD_OP_LOAD, VK_ATTACHMENT_STORE_OP_STORE, [&]() {
                GizmosPushConstant push_constant;
                push_constant.model = i.model;
                push_constant.color = i.color;
                push_constant.size = i.size;
                push_constant.camera_index =
                    current_render_instances->GetCameraIndex(i.editor_camera_component->GetHandle());
                push_constant.strand_meshlet_offset = 0;
                gizmos_pipeline->PushConstant(vk_command_buffer, 0, push_constant);
                GeometryStorage::BindVertices(vk_command_buffer);
                i.mesh->DrawIndexed(vk_command_buffer, gizmos_pipeline->states, 1);
              });
        });
      }
    }
    if (Platform::MeshShaderEnabled()) {
      for (const auto& i : editor_layer->gizmo_strands_tasks_) {
        if (!i.strands || !i.editor_camera_component || !i.editor_camera_component->IsEnabled() ||
            !i.strands->strand_meshlet_range_ || !i.strands->segment_range_ ||
            i.strands->strand_meshlet_range_->prev_frame_range == 0) {
          continue;
        }
        if (editor_layer->editor_cameras_.find(i.editor_camera_component->GetHandle()) ==
            editor_layer->editor_cameras_.end()) {
          EVOENGINE_ERROR("Target camera not registered in editor!");
          return;
        }
        Platform::RecordCommandsMainQueue([&](VkCommandBuffer vk_command_buffer) {
          std::shared_ptr<GraphicsPipeline> gizmos_pipeline;
          switch (i.gizmo_settings.color_mode) {
            case GizmoSettings::ColorMode::Default:
              gizmos_pipeline = gizmos_strands;
              break;
            case GizmoSettings::ColorMode::VertexColor:
              gizmos_pipeline = gizmos_strands_vertex_colored;
              break;
            case GizmoSettings::ColorMode::NormalColor:
              gizmos_pipeline = gizmos_strands_normal_colored;
              break;
          }
          i.editor_camera_component->GetRenderTexture()->ApplyGraphicsPipelineStates(gizmos_pipeline->states);
          i.gizmo_settings.ApplySettings(gizmos_pipeline->states);
          gizmos_pipeline->Bind(vk_command_buffer);
          gizmos_pipeline->BindDescriptorSet(vk_command_buffer, 0,
                                             per_frame_descriptor_sets_[current_frame_index]->GetVkDescriptorSet());
          gizmos_pipeline->BindDescriptorSet(
              vk_command_buffer, 1, strand_meshlet_descriptor_sets_[current_frame_index]->GetVkDescriptorSet());

          i.editor_camera_component->GetRenderTexture()->Render(
              vk_command_buffer, VK_ATTACHMENT_LOAD_OP_LOAD, VK_ATTACHMENT_STORE_OP_STORE, [&]() {
                GizmosPushConstant push_constant;
                push_constant.model = i.model;
                push_constant.color = i.color;
                push_constant.size = i.size;
                push_constant.camera_index =
                    current_render_instances->GetCameraIndex(i.editor_camera_component->GetHandle());
                push_constant.strand_meshlet_offset = i.strands->strand_meshlet_range_->prev_frame_offset;
                gizmos_pipeline->PushConstant(vk_command_buffer, 0, push_constant);
                gizmos_pipeline->DrawMeshTasks(vk_command_buffer, i.strands->strand_meshlet_range_->prev_frame_range);
                Platform::CountRenderPassDraw(RenderPassDrawBucket::EditorGizmos, RenderDrawCallKind::Direct,
                                              current_frame_index, i.strands->segment_range_->prev_frame_index_count);
              });
        });
      }
    }
    for (const auto& i : editor_layer->gizmo_instanced_mesh_tasks_) {
      if (editor_layer->editor_cameras_.find(i.editor_camera_component->GetHandle()) ==
          editor_layer->editor_cameras_.end()) {
        EVOENGINE_ERROR("Target camera not registered in editor!")
        return;
      }
      if (i.editor_camera_component && i.editor_camera_component->IsEnabled()) {
        Platform::RecordCommandsMainQueue([&](VkCommandBuffer vk_command_buffer) {
          i.editor_camera_component->GetRenderTexture()->ApplyGraphicsPipelineStates(gizmos_instanced_colored->states);
          i.gizmo_settings.ApplySettings(gizmos_instanced_colored->states);

          gizmos_instanced_colored->Bind(vk_command_buffer);
          gizmos_instanced_colored->BindDescriptorSet(
              vk_command_buffer, 0, per_frame_descriptor_sets_[current_frame_index]->GetVkDescriptorSet());
          gizmos_instanced_colored->BindDescriptorSet(vk_command_buffer, 1,
                                                      i.particle_info_list->GetDescriptorSet()->GetVkDescriptorSet());

          i.editor_camera_component->GetRenderTexture()->Render(
              vk_command_buffer, VK_ATTACHMENT_LOAD_OP_LOAD, VK_ATTACHMENT_STORE_OP_STORE, [&] {
                GizmosPushConstant push_constant;
                push_constant.model = i.model;
                push_constant.color = glm::vec4(0.0f);
                push_constant.size = i.size;
                push_constant.camera_index =
                    current_render_instances->GetCameraIndex(i.editor_camera_component->GetHandle());
                push_constant.strand_meshlet_offset = 0;
                gizmos_instanced_colored->PushConstant(vk_command_buffer, 0, push_constant);
                GeometryStorage::BindVertices(vk_command_buffer);
                i.mesh->DrawIndexed(vk_command_buffer, gizmos_instanced_colored->states,
                                    i.particle_info_list->PeekParticleInfoList().size());
              });
        });
      }
    }
  }
}

void RenderLayer::ForEachCollectedCamera(
    const std::function<void(const std::shared_ptr<Camera>& camera)>& action) const {
  const auto current_render_instances = render_instances_list_[Platform::GetCurrentFrameIndex()];
  for (const auto& camera : current_render_instances->cameras)
    action(camera.second);
}

std::shared_ptr<RenderInstanceStorage> RenderLayer::GetCurrentRenderInstanceStorage() const {
  const auto index = Platform::GetCurrentFrameIndex();
  if (index >= render_instances_list_.size())
    return {};
  return render_instances_list_[index];
}

std::shared_ptr<RenderInstanceStorage> RenderLayer::GetPreviousRenderInstanceStorage() const {
  const auto index =
      (Platform::GetMaxFramesInFlight() + Platform::GetCurrentFrameIndex() - 1) % Platform::GetMaxFramesInFlight();
  if (index >= render_instances_list_.size())
    return {};
  return render_instances_list_[index];
}

bool RenderLayer::RequiresCameraWideTemporalHistoryRejection() const {
  const auto render_instances = GetCurrentRenderInstanceStorage();
  return (render_instances && render_instances->RequiresCameraWideTemporalHistoryRejection()) ||
         !deferred_rendering_external_functions.empty() || !forward_rendering_external_functions.empty();
}

RayCameraShaderVariantStats RenderLayer::GetRayCameraShaderVariantStats(
    const RayCameraShaderTechnique technique) const {
  return ray_camera_shader_variant_cache_ ? ray_camera_shader_variant_cache_->GetStats(technique)
                                          : RayCameraShaderVariantStats{};
}

RayCameraHistoryStats RenderLayer::GetRayCameraHistoryStats() const {
  auto result = retired_ray_camera_history_stats_;
  const auto append = [&](const RayCameraHistoryStats& stats) {
    result.live_camera_count += stats.live_camera_count;
    result.live_history_count += stats.live_history_count;
    result.live_ray_tracing_history_count += stats.live_ray_tracing_history_count;
    result.live_ray_query_history_count += stats.live_ray_query_history_count;
    result.valid_history_count += stats.valid_history_count;
    result.radiance_image_count += stats.radiance_image_count;
    result.convergence_image_count += stats.convergence_image_count;
    result.radiance_view_count += stats.radiance_view_count;
    result.convergence_view_count += stats.convergence_view_count;
    result.live_byte_size += stats.live_byte_size;
    result.creation_count += stats.creation_count;
    result.reuse_count += stats.reuse_count;
    result.invalidation_count += stats.invalidation_count;
    result.retirement_count += stats.retirement_count;
    result.live_output_descriptor_count += stats.live_output_descriptor_count;
    result.output_descriptor_creation_count += stats.output_descriptor_creation_count;
    result.output_descriptor_reuse_count += stats.output_descriptor_reuse_count;
  };
  for (const auto& [handle, camera] : ray_camera_history_cameras_) {
    if (const auto locked_camera = camera.lock()) {
      append(locked_camera->GetRayCameraHistoryStats());
    }
  }
  result.peak_live_history_count = peak_live_ray_camera_history_count_;
  result.peak_live_byte_size = peak_live_ray_camera_history_byte_size_;
  result.peak_live_output_descriptor_count = peak_live_ray_camera_output_descriptor_count_;
  return result;
}

RenderLayer::RayCameraFramePathStats RenderLayer::GetRayCameraFramePathStats() const {
  const auto history = GetRayCameraHistoryStats();
  RayCameraFramePathStats result;
  result.render_graph_plan_cache = ray_camera_render_graph_plan_cache_.GetStats();
  result.live_output_descriptor_count = history.live_output_descriptor_count;
  result.peak_live_output_descriptor_count = history.peak_live_output_descriptor_count;
  result.output_descriptor_creation_count = history.output_descriptor_creation_count;
  result.output_descriptor_reuse_count = history.output_descriptor_reuse_count;
  result.retained_frame_slot_count =
      static_cast<uint32_t>(std::count_if(render_graph_transient_resource_stores_.begin(),
                                          render_graph_transient_resource_stores_.end(), [](const auto& stores) {
                                            return !stores.empty();
                                          }));
  return result;
}

bool RenderLayer::IsRayCameraShaderVariantReady(const RayCameraShaderTechnique technique) const {
  return !ray_camera_shader_variant_cache_ || ray_camera_shader_variant_cache_->IsReady(technique);
}

void RenderLayer::ApplyAnimators() const {
  const auto scene = GetScene();
  if (const auto* owners = scene->UnsafeGetPrivateComponentOwnersList<Animator>()) {
    Jobs::RunParallelFor(owners->size(), [&](size_t i) {
      const auto entity = owners->at(i);
      if (!scene->IsEntityEnabled(entity))
        return;
      const auto animator = scene->GetOrSetPrivateComponent<Animator>(owners->at(i)).lock();
      if (!animator->IsEnabled())
        return;
      animator->Apply();
    });
  }
  if (const auto* owners = scene->UnsafeGetPrivateComponentOwnersList<SkinnedMeshRenderer>()) {
    Jobs::RunParallelFor(owners->size(), [&](size_t i) {
      const auto entity = owners->at(i);
      if (!scene->IsEntityEnabled(entity))
        return;
      const auto skinned_mesh_renderer = scene->GetOrSetPrivateComponent<SkinnedMeshRenderer>(entity).lock();
      if (!skinned_mesh_renderer->IsEnabled())
        return;
      skinned_mesh_renderer->UpdateBoneMatrices();
    });
    for (const auto& i : *owners) {
      if (!scene->IsEntityEnabled(i))
        continue;
      const auto skinned_mesh_renderer = scene->GetOrSetPrivateComponent<SkinnedMeshRenderer>(i).lock();
      if (!skinned_mesh_renderer->IsEnabled())
        continue;
      skinned_mesh_renderer->UpdateRayTracingGeometry();
      skinned_mesh_renderer->bone_matrices->UploadData();
    }
  }
  if (const auto* owners = scene->UnsafeGetPrivateComponentOwnersList<MeshRenderer>()) {
    for (const auto& entity : *owners) {
      if (!scene->IsEntityEnabled(entity))
        continue;
      const auto mesh_renderer = scene->GetOrSetPrivateComponent<MeshRenderer>(entity).lock();
      if (mesh_renderer->IsEnabled()) {
        mesh_renderer->UpdateRayTracingGeometry();
      }
    }
  }
}

void RenderLayer::PreparePointAndSpotLightShadowMap(const bool immediate, const bool include_external) const {
  const bool count_draw_calls = count_shadow_rendering_draw_calls;
  const bool use_mesh_shader = Platform::MeshShaderEnabled() && enable_meshlet;
  const auto current_frame_index = Platform::GetCurrentFrameIndex();
  const auto& point_light_shadow_opaque_pipeline =
      use_mesh_shader ? point_light_shadow_pipeline_mesh_shader : point_light_shadow_pipeline_normal_opaque;
  const auto& spot_light_shadow_opaque_pipeline =
      use_mesh_shader ? spot_light_shadow_pipeline_mesh_shader : spot_light_shadow_pipeline_normal_opaque;
  const auto current_render_instances = render_instances_list_[current_frame_index];
  const auto record_commands = [&](const std::function<void(VkCommandBuffer)>& action) {
    if (immediate) {
      Platform::ImmediateSubmit(action);
    } else {
      Platform::RecordCommandsMainQueue(action);
    }
  };
  record_commands([&](const VkCommandBuffer vk_command_buffer) {
    const auto account_draw = [&](const RenderPassDrawBucket bucket, const RenderDrawCallKind kind,
                                  const size_t prim_count, const size_t indirect_draw_commands = 0) {
      if (count_draw_calls) {
        Platform::CountRenderPassDraw(bucket, kind, current_frame_index, prim_count, indirect_draw_commands);
      }
    };
    const auto prepare_graphics_pipeline = [&](const std::shared_ptr<GraphicsPipeline>& target_pipeline,
                                               const glm::ivec4& view_port) {
      if (!target_pipeline) {
        return false;
      }
      target_pipeline->states.ResetAllStates(0);
      target_pipeline->Bind(vk_command_buffer);
      target_pipeline->BindDescriptorSet(vk_command_buffer, 0,
                                         per_frame_descriptor_sets_[current_frame_index]->GetVkDescriptorSet());
      if (use_mesh_shader && (target_pipeline == point_light_shadow_opaque_pipeline ||
                              target_pipeline == spot_light_shadow_opaque_pipeline)) {
        target_pipeline->BindDescriptorSet(vk_command_buffer, 1,
                                           meshlet_descriptor_sets_[current_frame_index]->GetVkDescriptorSet());
      } else if (target_pipeline == strands_point_light_shadow_pipeline ||
                 target_pipeline == strands_spot_light_shadow_pipeline) {
        target_pipeline->BindDescriptorSet(vk_command_buffer, 1,
                                           strand_meshlet_descriptor_sets_[current_frame_index]->GetVkDescriptorSet());
      }
      target_pipeline->states.SetViewportScissor(view_port);
      return true;
    };
    const auto render_shadow_collection =
        [&](const RenderPassDrawBucket bucket,
            const std::shared_ptr<RenderInstanceStorage::IRenderInstanceCollection>& collection,
            const std::shared_ptr<GraphicsPipeline>& opaque_pipeline, const glm::mat4& light_space_matrix,
            const int light_index, const int split_index, const glm::ivec4& viewport) {
          if (!prepare_graphics_pipeline(opaque_pipeline, viewport)) {
            return;
          }
          collection->ForEachRenderInstance([&](const auto& render_instance) {
            if (!ShouldRenderShadowInstance(render_instance, light_space_matrix)) {
              return;
            }
            RenderInstancePushConstant push_constant;
            push_constant.camera_index = light_index;
            push_constant.light_split_index = split_index;
            push_constant.instance_index = render_instance->instance_index;
            const auto prim_count = render_instance->Render(vk_command_buffer, push_constant, opaque_pipeline);
            account_draw(bucket, RenderDrawCallKind::Direct, prim_count);
          });
        };
    const auto draw_shadow_indirect = [&](const RenderPassDrawBucket bucket,
                                          const std::shared_ptr<GraphicsPipeline>& target_pipeline,
                                          const uint32_t prim_count, const std::shared_ptr<Buffer>& indexed_buffer,
                                          const std::vector<VkDrawIndexedIndirectCommand>& indexed_commands,
                                          const std::shared_ptr<Buffer>& mesh_task_buffer,
                                          const std::vector<VkDrawMeshTasksIndirectCommandEXT>& mesh_task_commands,
                                          const glm::mat4& light_space_matrix, const int light_index,
                                          const int split_index, const glm::ivec4& viewport) {
      if (prim_count == 0 ||
          !HasVisibleShadowInstance(current_render_instances->deferred_render_instances, light_space_matrix) ||
          !prepare_graphics_pipeline(target_pipeline, viewport)) {
        return;
      }
      RenderInstancePushConstant push_constant;
      push_constant.camera_index = light_index;
      push_constant.light_split_index = split_index;
      push_constant.instance_index = 0;
      target_pipeline->PushConstant(vk_command_buffer, 0, push_constant);
      target_pipeline->states.ApplyAllStates(vk_command_buffer);
      account_draw(bucket, RenderDrawCallKind::Indirect, prim_count,
                   use_mesh_shader ? mesh_task_commands.size() : indexed_commands.size());
      if (use_mesh_shader) {
        Platform::DrawMeshTasksIndirect(vk_command_buffer, *mesh_task_buffer, 0, mesh_task_commands.size(),
                                        sizeof(VkDrawMeshTasksIndirectCommandEXT));
      } else {
        Platform::DrawIndexedIndirect(vk_command_buffer, *indexed_buffer, 0, indexed_commands.size(),
                                      sizeof(VkDrawIndexedIndirectCommand));
      }
    };
    const auto render_strands_shadow_collection =
        [&](const RenderPassDrawBucket bucket,
            const std::shared_ptr<RenderInstanceStorage::IRenderInstanceCollection>& collection,
            const std::shared_ptr<GraphicsPipeline>& pipeline, const glm::mat4& light_space_matrix,
            const int light_index, const int split_index, const glm::ivec4& viewport) {
          if (!prepare_graphics_pipeline(pipeline, viewport)) {
            return;
          }
          collection->ForEachRenderInstance([&](const auto& render_instance) {
            if (!ShouldRenderShadowInstance(render_instance, light_space_matrix)) {
              return;
            }
            RenderInstancePushConstant push_constant;
            push_constant.camera_index = light_index;
            push_constant.light_split_index = split_index;
            push_constant.instance_index = render_instance->instance_index;
            pipeline->states.cull_mode = render_instance->cull_mode;
            const auto prim_count = render_instance->Render(vk_command_buffer, push_constant, pipeline);
            if (count_draw_calls) {
              Platform::CountRenderPassDraw(bucket, RenderDrawCallKind::Direct, current_frame_index, prim_count);
            }
          });
        };

    VkRect2D render_area;
    render_area.offset = {0, 0};
    render_area.extent.width = lighting_->point_light_shadow_map_->GetExtent().width;
    render_area.extent.height = lighting_->point_light_shadow_map_->GetExtent().height;
    lighting_->point_light_shadow_map_->TransitImageLayout(vk_command_buffer, VK_IMAGE_LAYOUT_ATTACHMENT_OPTIMAL);

    for (int face = 0; face < 6; face++) {
      VkRenderingInfo render_info{};
      auto depth_attachment = lighting_->GetLayeredPointLightDepthAttachmentInfo(face, VK_ATTACHMENT_LOAD_OP_CLEAR,
                                                                                 VK_ATTACHMENT_STORE_OP_STORE);
      render_info.sType = VK_STRUCTURE_TYPE_RENDERING_INFO;
      render_info.renderArea = render_area;
      render_info.layerCount = 1;
      render_info.colorAttachmentCount = 0;
      render_info.pColorAttachments = nullptr;
      render_info.pDepthAttachment = &depth_attachment;
      Platform::RecordRenderCommands(render_info, vk_command_buffer, [&]() {
        for (int i = 0; i < current_render_instances->point_light_info_blocks_.size(); i++) {
          const auto& point_light_info_block = current_render_instances->point_light_info_blocks_[i];
          if (!LightCastsShadow(point_light_info_block.diffuse)) {
            continue;
          }
          const auto& light_space_matrix = point_light_info_block.light_space_matrix[face];
          GeometryStorage::BindVertices(vk_command_buffer);
          {
            if (enable_indirect_rendering &&
                !current_render_instances->opaque_shadow_mesh_draw_indexed_indirect_commands.empty()) {
              draw_shadow_indirect(
                  RenderPassDrawBucket::PointLightShadow, point_light_shadow_opaque_pipeline,
                  current_render_instances->total_opaque_shadow_mesh_triangles,
                  current_render_instances->opaque_shadow_mesh_draw_indexed_indirect_commands_buffer,
                  current_render_instances->opaque_shadow_mesh_draw_indexed_indirect_commands,
                  current_render_instances->opaque_shadow_mesh_draw_mesh_tasks_indirect_commands_buffer,
                  current_render_instances->opaque_shadow_mesh_draw_mesh_tasks_indirect_commands, light_space_matrix, i,
                  face, point_light_info_block.viewport);
            } else {
              render_shadow_collection(
                  RenderPassDrawBucket::PointLightShadow, current_render_instances->deferred_render_instances,
                  point_light_shadow_opaque_pipeline, light_space_matrix, i, face, point_light_info_block.viewport);
            }
          }
          {
            render_shadow_collection(RenderPassDrawBucket::PointLightShadow,
                                     current_render_instances->deferred_instanced_render_instances,
                                     instanced_point_light_shadow_pipeline_opaque, light_space_matrix, i, face,
                                     point_light_info_block.viewport);
          }
          GeometryStorage::BindSkinnedVertices(vk_command_buffer);
          {
            render_shadow_collection(RenderPassDrawBucket::PointLightShadow,
                                     current_render_instances->deferred_skinned_render_instances,
                                     skinned_point_light_shadow_pipeline_opaque, light_space_matrix, i, face,
                                     point_light_info_block.viewport);
          }
          render_strands_shadow_collection(RenderPassDrawBucket::PointLightShadow,
                                           current_render_instances->deferred_strands_render_instances,
                                           use_mesh_shader ? strands_point_light_shadow_pipeline : nullptr,
                                           light_space_matrix, i, face, point_light_info_block.viewport);
          if (include_external) {
            for (const auto& func : point_light_shadow_map_external_functions) {
              const auto prim_count = func(vk_command_buffer, {i, face, point_light_info_block.viewport});
              account_draw(RenderPassDrawBucket::PointLightShadow, RenderDrawCallKind::Direct, prim_count);
            }
          }
        }
      });
    }
#pragma region Viewport and scissor

    render_area.offset = {0, 0};
    render_area.extent.width = lighting_->spot_light_shadow_map_->GetExtent().width;
    render_area.extent.height = lighting_->spot_light_shadow_map_->GetExtent().height;

#pragma endregion
    lighting_->spot_light_shadow_map_->TransitImageLayout(vk_command_buffer, VK_IMAGE_LAYOUT_ATTACHMENT_OPTIMAL);
    VkRenderingInfo render_info{};
    const auto depth_attachment =
        lighting_->GetSpotLightDepthAttachmentInfo(VK_ATTACHMENT_LOAD_OP_CLEAR, VK_ATTACHMENT_STORE_OP_STORE);
    render_info.sType = VK_STRUCTURE_TYPE_RENDERING_INFO;
    render_info.renderArea = render_area;
    render_info.layerCount = 1;
    render_info.colorAttachmentCount = 0;
    render_info.pColorAttachments = nullptr;
    render_info.pDepthAttachment = &depth_attachment;
    Platform::RecordRenderCommands(render_info, vk_command_buffer, [&]() {
      for (int i = 0; i < current_render_instances->spot_light_info_blocks_.size(); i++) {
        const auto& spot_light_info_block = current_render_instances->spot_light_info_blocks_[i];
        if (!LightCastsShadow(spot_light_info_block.diffuse)) {
          continue;
        }
        const auto& light_space_matrix = spot_light_info_block.light_space_matrix;
        GeometryStorage::BindVertices(vk_command_buffer);
        {
          if (enable_indirect_rendering &&
              !current_render_instances->opaque_shadow_mesh_draw_indexed_indirect_commands.empty()) {
            draw_shadow_indirect(RenderPassDrawBucket::SpotLightShadow, spot_light_shadow_opaque_pipeline,
                                 current_render_instances->total_opaque_shadow_mesh_triangles,
                                 current_render_instances->opaque_shadow_mesh_draw_indexed_indirect_commands_buffer,
                                 current_render_instances->opaque_shadow_mesh_draw_indexed_indirect_commands,
                                 current_render_instances->opaque_shadow_mesh_draw_mesh_tasks_indirect_commands_buffer,
                                 current_render_instances->opaque_shadow_mesh_draw_mesh_tasks_indirect_commands,
                                 light_space_matrix, i, 0, spot_light_info_block.viewport);
          } else {
            render_shadow_collection(
                RenderPassDrawBucket::SpotLightShadow, current_render_instances->deferred_render_instances,
                spot_light_shadow_opaque_pipeline, light_space_matrix, i, 0, spot_light_info_block.viewport);
          }
        }
        {
          render_shadow_collection(
              RenderPassDrawBucket::SpotLightShadow, current_render_instances->deferred_instanced_render_instances,
              instanced_spot_light_shadow_pipeline_opaque, light_space_matrix, i, 0, spot_light_info_block.viewport);
        }
        GeometryStorage::BindSkinnedVertices(vk_command_buffer);
        {
          render_shadow_collection(
              RenderPassDrawBucket::SpotLightShadow, current_render_instances->deferred_skinned_render_instances,
              skinned_spot_light_shadow_pipeline_opaque, light_space_matrix, i, 0, spot_light_info_block.viewport);
        }
        render_strands_shadow_collection(RenderPassDrawBucket::SpotLightShadow,
                                         current_render_instances->deferred_strands_render_instances,
                                         use_mesh_shader ? strands_spot_light_shadow_pipeline : nullptr,
                                         light_space_matrix, i, 0, spot_light_info_block.viewport);
        if (include_external) {
          for (const auto& func : spot_light_shadow_map_external_functions) {
            const auto prim_count = func(vk_command_buffer, {i, spot_light_info_block.viewport});
            account_draw(RenderPassDrawBucket::SpotLightShadow, RenderDrawCallKind::Direct, prim_count);
          }
        }
      }
    });
    lighting_->point_light_shadow_map_->TransitImageLayout(vk_command_buffer, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
    lighting_->spot_light_shadow_map_->TransitImageLayout(vk_command_buffer, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
  });
}

bool RenderLayer::UpdateRenderInstanceStorage(
    const std::shared_ptr<Scene>& scene, const uint32_t current_frame_index, const bool include_editor_cameras,
    const bool update_editor_selection, const bool track_ddgi_scene_inputs,
    const std::pair<GlobalTransform, std::shared_ptr<Camera>>* injected_camera, const bool include_reflection_probes) {
  const ProfilerScope profiler_scope("RenderLayer::UpdateRenderInstanceStorage", "Render");
  auto lod_center = glm::vec3(0.f);
  float lod_max_distance = FLT_MAX;
  bool lod_set = false;
  if (const auto main_camera = scene->main_camera.Get<Camera>()) {
    if (const auto main_camera_owner = main_camera->GetOwner(); scene->IsEntityValid(main_camera_owner)) {
      lod_center = scene->GetDataComponent<GlobalTransform>(main_camera_owner).GetPosition();
      lod_max_distance = main_camera->camera_settings.far_distance;
      lod_set = true;
    }
  }
  if (injected_camera) {
    lod_center = injected_camera->first.GetPosition();
    lod_max_distance = injected_camera->second ? injected_camera->second->camera_settings.far_distance : FLT_MAX;
    lod_set = true;
  } else if (!lod_set) {
    if (const auto editor_layer = ApplicationContext::Get().GetLayer<EditorLayer>()) {
      if (const auto scene_camera = editor_layer->GetSceneCamera()) {
        lod_center = editor_layer->GetSceneCameraPosition();
        lod_max_distance = scene_camera->camera_settings.far_distance;
      }
    }
  }
  RenderInstanceStorage::CalculateLodFactor(scene, lod_center, lod_max_distance);
  auto world_bound = scene->GetBound();
  need_fade_ = false;
  const auto current_render_instances = render_instances_list_[current_frame_index];
  current_render_instances->BuildFromScene(render_settings, scene, world_bound, include_editor_cameras, injected_camera,
                                           include_reflection_probes);
  const auto previous_render_instances =
      render_instances_list_[(current_frame_index + Platform::GetMaxFramesInFlight() - 1) %
                             Platform::GetMaxFramesInFlight()];
  bool render_instance_updated = false;
  if (track_ddgi_scene_inputs) {
    const auto current_render_info = current_render_instances->render_info_block;
    PreserveDdgiRenderInfo(current_render_instances->render_info_block, previous_render_instances->render_info_block);
    render_instance_updated = *current_render_instances != *previous_render_instances;
    auto active_light_keys = CollectDdgiActiveLightKeys(scene);
    auto light_signatures = CollectDdgiLightSignatures(scene);
    std::vector<uint64_t> geometry_signatures;
    std::vector<uint64_t> probe_state_geometry_signatures;
    std::vector<std::pair<int32_t, uint64_t>> material_references;
    const auto collect_ddgi_geometry_signatures = [&](const auto& collection) {
      if (!collection) {
        return;
      }
      collection->ForEachRenderInstance([&](const auto& render_instance) {
        if (IsDdgiTlasContributor(render_instance)) {
          geometry_signatures.push_back(MakeDdgiGeometrySignature(render_instance, true));
          probe_state_geometry_signatures.push_back(MakeDdgiGeometrySignature(render_instance, false));
          const auto material_key = render_instance->material
                                        ? render_instance->material->GetHandle().GetValue()
                                        : static_cast<uint64_t>(static_cast<uint32_t>(render_instance->material_index));
          material_references.emplace_back(render_instance->material_index, material_key);
        }
      });
    };
    collect_ddgi_geometry_signatures(current_render_instances->deferred_render_instances);
    collect_ddgi_geometry_signatures(current_render_instances->deferred_skinned_render_instances);
    collect_ddgi_geometry_signatures(current_render_instances->deferred_instanced_render_instances);
    collect_ddgi_geometry_signatures(current_render_instances->forward_render_instances);
    collect_ddgi_geometry_signatures(current_render_instances->forward_skinned_render_instances);
    collect_ddgi_geometry_signatures(current_render_instances->forward_instanced_render_instances);
    collect_ddgi_geometry_signatures(current_render_instances->transparent_render_instances);
    collect_ddgi_geometry_signatures(current_render_instances->transparent_skinned_render_instances);
    collect_ddgi_geometry_signatures(current_render_instances->transparent_instanced_render_instances);
    collect_ddgi_geometry_signatures(current_render_instances->external_render_instances);
    std::sort(geometry_signatures.begin(), geometry_signatures.end());
    std::sort(probe_state_geometry_signatures.begin(), probe_state_geometry_signatures.end());
    auto material_inputs = CollectDdgiMaterialInputSignatures(current_render_instances, std::move(material_references));
    ddgi_referenced_scene_inputs_pending_ = material_inputs.inputs_pending;
    const auto emissive_inventory_signature = current_render_instances->GetDdgiEmissiveInventorySignature();
    const bool had_previous_scene_inputs = ddgi_has_previous_scene_inputs_;
    int scene_change_triggers = DdgiVolumeTriggerConditionNone;
    if (had_previous_scene_inputs) {
      const bool light_membership_changed = active_light_keys != ddgi_previous_active_light_keys_;
      if (light_membership_changed) {
        scene_change_triggers |= DdgiVolumeTriggerConditionLightEnableChanged;
      }
      if (!light_membership_changed && light_signatures != ddgi_previous_light_signatures_) {
        scene_change_triggers |= DdgiVolumeTriggerConditionLightingConditionChanged;
      }
      const bool material_inputs_changed = material_inputs.keys != ddgi_previous_material_keys_ ||
                                           material_inputs.materials != ddgi_previous_material_signatures_ ||
                                           material_inputs.textures != ddgi_previous_material_texture_signatures_;
      const bool emissive_inventory_changed =
          emissive_inventory_signature != ddgi_previous_emissive_inventory_signature_;
      const bool geometry_inputs_changed = geometry_signatures != ddgi_previous_geometry_signatures_;
      const bool probe_state_geometry_changed =
          probe_state_geometry_signatures != ddgi_previous_probe_state_geometry_signatures_;
      if (material_inputs_changed || emissive_inventory_changed || geometry_inputs_changed) {
        scene_change_triggers |= DdgiVolumeTriggerConditionGeometryChanged;
      }
      ddgi_latched_scene_change_triggers_ |= scene_change_triggers;
      ddgi_latched_scene_geometry_changed_ |= probe_state_geometry_changed;
    } else {
      ddgi_deferred_scene_readiness_refresh_ = true;
    }
    ddgi_has_previous_scene_inputs_ = true;
    ddgi_previous_material_keys_ = std::move(material_inputs.keys);
    ddgi_previous_material_signatures_ = std::move(material_inputs.materials);
    ddgi_previous_material_texture_signatures_ = std::move(material_inputs.textures);
    ddgi_previous_emissive_inventory_signature_ = emissive_inventory_signature;
    ddgi_previous_active_light_keys_ = std::move(active_light_keys);
    ddgi_previous_light_signatures_ = std::move(light_signatures);
    ddgi_previous_geometry_signatures_ = std::move(geometry_signatures);
    ddgi_previous_probe_state_geometry_signatures_ = std::move(probe_state_geometry_signatures);
    PreserveDdgiRenderInfo(current_render_instances->render_info_block, current_render_info);
  } else {
    render_instance_updated = *current_render_instances != *previous_render_instances;
  }
  const auto camera_info_changed = [&](const std::shared_ptr<Camera>& camera) {
    if (!camera || !previous_render_instances) {
      return true;
    }
    const auto current_index_search = current_render_instances->camera_indices_.find(camera->GetHandle());
    const auto previous_index_search = previous_render_instances->camera_indices_.find(camera->GetHandle());
    if (current_index_search == current_render_instances->camera_indices_.end() ||
        previous_index_search == previous_render_instances->camera_indices_.end()) {
      return true;
    }
    const auto current_index = static_cast<size_t>(current_index_search->second);
    const auto previous_index = static_cast<size_t>(previous_index_search->second);
    if (current_index >= current_render_instances->camera_info_blocks_.size() ||
        previous_index >= previous_render_instances->camera_info_blocks_.size()) {
      return true;
    }
    return current_render_instances->camera_info_blocks_[current_index] !=
           previous_render_instances->camera_info_blocks_[previous_index];
  };
  if (update_editor_selection) {
    if (const auto editor_layer = ApplicationContext::Get().GetLayer<EditorLayer>()) {
      if (scene->IsEntityValid(editor_layer->GetSelectedEntity())) {
        for (const auto& i : current_render_instances->instance_info_blocks_) {
          if (i.info_index) {
            need_fade_ = true;
          }
        }
      }
      editor_layer->MouseEntitySelection();
    }
  }
  if (render_instance_updated) {
    world_bound.min -= glm::vec3(0.1f);
    world_bound.max += glm::vec3(0.1f);
    scene->SetBound(world_bound);
    for (const auto& camera_entry : current_render_instances->cameras) {
      if (const auto& camera = camera_entry.second) {
        camera->frame_count_ = 0;
        camera->InvalidateRayCameraHistory();
      }
    }
  } else {
    for (const auto& camera_entry : current_render_instances->cameras) {
      if (const auto& camera = camera_entry.second; camera && camera_info_changed(camera)) {
        camera->frame_count_ = 0;
        camera->InvalidateRayCameraHistory();
      }
    }
  }
  return render_instance_updated;
}

void RenderLayer::PrepareEnvironmentalBrdfLut() {
  environmental_brdf_lut_.reset();
  environmental_brdf_lut_ = AssetManager::CreateTemporaryAsset<Texture2D>();
  auto& environmental_brdf_lut_texture_storage = environmental_brdf_lut_->RefTexture2DStorage();
  constexpr auto brdf_lut_resolution = 512;
  {
    VkImageCreateInfo image_info{};
    image_info.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
    image_info.imageType = VK_IMAGE_TYPE_2D;
    image_info.extent.width = brdf_lut_resolution;
    image_info.extent.height = brdf_lut_resolution;
    image_info.extent.depth = 1;
    image_info.mipLevels = 1;
    image_info.arrayLayers = 1;
    image_info.format = VK_FORMAT_R16G16_SFLOAT;
    image_info.tiling = VK_IMAGE_TILING_OPTIMAL;
    image_info.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
    image_info.usage = VK_IMAGE_USAGE_SAMPLED_BIT | VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT;
    image_info.samples = VK_SAMPLE_COUNT_1_BIT;
    image_info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;

    environmental_brdf_lut_texture_storage.image = std::make_unique<Image>(image_info);

    VkImageViewCreateInfo view_info{};
    view_info.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
    view_info.image = environmental_brdf_lut_->GetVkImage();
    view_info.viewType = VK_IMAGE_VIEW_TYPE_2D;
    view_info.format = VK_FORMAT_R16G16_SFLOAT;
    view_info.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
    view_info.subresourceRange.baseMipLevel = 0;
    view_info.subresourceRange.levelCount = 1;
    view_info.subresourceRange.baseArrayLayer = 0;
    view_info.subresourceRange.layerCount = 1;

    environmental_brdf_lut_texture_storage.image_view = std::make_unique<ImageView>(view_info);

    VkSamplerCreateInfo sampler_info{};
    sampler_info.sType = VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO;
    sampler_info.magFilter = VK_FILTER_LINEAR;
    sampler_info.minFilter = VK_FILTER_LINEAR;
    sampler_info.addressModeU = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
    sampler_info.addressModeV = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
    sampler_info.addressModeW = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
    sampler_info.anisotropyEnable = VK_TRUE;
    sampler_info.maxAnisotropy = Platform::GetSelectedPhysicalDevice()->properties.limits.maxSamplerAnisotropy;
    sampler_info.borderColor = VK_BORDER_COLOR_INT_OPAQUE_BLACK;
    sampler_info.unnormalizedCoordinates = VK_FALSE;
    sampler_info.compareEnable = VK_FALSE;
    sampler_info.compareOp = VK_COMPARE_OP_ALWAYS;
    sampler_info.mipmapMode = VK_SAMPLER_MIPMAP_MODE_LINEAR;

    environmental_brdf_lut_texture_storage.sampler = std::make_unique<Sampler>(sampler_info);
  }
  const auto environmental_brdf_pipeline = std::make_shared<GraphicsPipeline>();
  environmental_brdf_pipeline->vertex_shader = Shader::CreateTemporary(
      ShaderType::Vertex, Resources::GetDefaultResourcesPath() / "Shaders/Graphics/Vertex/TexturePassThrough.slang");
  environmental_brdf_pipeline->fragment_shader = Shader::CreateTemporary(
      ShaderType::Fragment,
      Resources::GetDefaultResourcesPath() / "Shaders/Graphics/Fragment/Lighting/EnvironmentalMapBrdf.slang");
  environmental_brdf_pipeline->geometry_type = GeometryType::Mesh;
  environmental_brdf_pipeline->vertex_input_attribute_set = VertexInputAttributeSet::PositionTexCoord;
  environmental_brdf_pipeline->depth_attachment_format = Platform::Constants::render_texture_depth;
  environmental_brdf_pipeline->stencil_attachment_format = VK_FORMAT_UNDEFINED;
  environmental_brdf_pipeline->color_attachment_formats = {1, VK_FORMAT_R16G16_SFLOAT};
  environmental_brdf_pipeline->Initialize();

  Platform::ImmediateSubmit([&](VkCommandBuffer vk_command_buffer) {
    environmental_brdf_lut_texture_storage.image->TransitImageLayout(vk_command_buffer,
                                                                     VK_IMAGE_LAYOUT_ATTACHMENT_OPTIMAL);
#pragma region Viewport and scissor
    VkRect2D render_area;
    render_area.offset = {0, 0};
    render_area.extent.width = brdf_lut_resolution;
    render_area.extent.height = brdf_lut_resolution;
    VkViewport viewport;
    viewport.x = 0.0f;
    viewport.y = 0.0f;
    viewport.width = static_cast<float>(brdf_lut_resolution);
    viewport.height = static_cast<float>(brdf_lut_resolution);
    viewport.minDepth = 0.0f;
    viewport.maxDepth = 1.0f;

    VkRect2D scissor;
    scissor.offset = {0, 0};
    scissor.extent.width = brdf_lut_resolution;
    scissor.extent.height = brdf_lut_resolution;
    environmental_brdf_pipeline->states.view_port = viewport;
    environmental_brdf_pipeline->states.scissor = scissor;
#pragma endregion
#pragma region Lighting pass
    {
      VkRenderingAttachmentInfo attachment{};
      attachment.sType = VK_STRUCTURE_TYPE_RENDERING_ATTACHMENT_INFO;

      attachment.imageLayout = VK_IMAGE_LAYOUT_ATTACHMENT_OPTIMAL;
      attachment.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
      attachment.storeOp = VK_ATTACHMENT_STORE_OP_STORE;

      attachment.clearValue = {0, 0, 0, 1};
      attachment.imageView = environmental_brdf_lut_texture_storage.image_view->GetVkImageView();

      VkRenderingInfo render_info{};
      render_info.sType = VK_STRUCTURE_TYPE_RENDERING_INFO;
      render_info.renderArea = render_area;
      render_info.layerCount = 1;
      render_info.colorAttachmentCount = 1;
      render_info.pColorAttachments = &attachment;
      environmental_brdf_pipeline->states.depth_test = false;
      environmental_brdf_pipeline->states.color_blend_attachment_states.clear();
      environmental_brdf_pipeline->states.color_blend_attachment_states.resize(1);
      for (auto& i : environmental_brdf_pipeline->states.color_blend_attachment_states) {
        i.colorWriteMask = VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT;
        i.blendEnable = VK_FALSE;
      }
      Platform::RecordRenderCommands(render_info, vk_command_buffer, [&]() {
        environmental_brdf_pipeline->Bind(vk_command_buffer);
        const auto mesh = Resources::GetInstance().GetTexturePassThroughQuad();
        GeometryStorage::BindVertices(vk_command_buffer);
        mesh->DrawIndexed(vk_command_buffer, environmental_brdf_pipeline->states, 1);
      });
#pragma endregion
    }
    environmental_brdf_lut_texture_storage.image->TransitImageLayout(vk_command_buffer,
                                                                     VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
  });
}
void RenderLayer::RenderToCamera(const std::shared_ptr<Scene>& scene, const GlobalTransform& camera_global_transform,
                                 const std::shared_ptr<Camera>& camera, const bool immediate,
                                 const bool reflection_probe_capture) const {
  const ProfilerScope profiler_scope("RenderLayer::RenderToCamera", "Render");
  const auto current_frame_index = Platform::GetCurrentFrameIndex();
  const auto current_render_instances = render_instances_list_[current_frame_index];
  const auto lighting_descriptor_set =
      lighting_ ? lighting_->lighting_descriptor_sets_.at(current_frame_index) : nullptr;
  const int camera_index = current_render_instances->GetCameraIndex(camera->GetHandle());
  bool ambient_occlusion_enabled = false;
  if (!immediate) {
    if (const auto post_processing_stack = camera->post_processing_stack_ref.Get<PostProcessingStack>();
        post_processing_stack && post_processing_stack->enable_ambient_occlusion &&
        post_processing_stack->ambient_occlusion && post_processing_renderer_resources_) {
      post_processing_stack->ambient_occlusion->BuildPipelines(*post_processing_renderer_resources_);
      const auto& resources = post_processing_renderer_resources_->ambient_occlusion;
      ambient_occlusion_enabled = resources.geometry_pipeline && resources.geometry_pipeline->Initialized() &&
                                  resources.blur_pipeline && resources.blur_pipeline->Initialized() &&
                                  resources.sampler;
    }
  }
  const auto raster_lighting_texture_descriptor_set =
      GetRasterLightingTextureDescriptorSet(current_frame_index, camera_index, current_render_instances);
  const auto editor_layer = ApplicationContext::Get().GetLayer<EditorLayer>();
  const bool is_scene_camera = editor_layer && camera.get() == editor_layer->GetSceneCamera().get();
  VolumetricCloudSettings volumetric_cloud_settings{};
  const bool volumetric_clouds_enabled = !reflection_probe_capture && volumetric_cloud_settings.enabled;
  const auto ddgi_settings = ResolveEnvironmentalLighting(scene).ddgi_settings;
  const auto* ddgi_debug_runtime = GetPrimaryDdgiVolumeRuntimeState();
  const auto ddgi_visualization_probe_count =
      ddgi_debug_runtime ? ddgi_debug_runtime->frame_resource_layout.probe_count : 0u;
  const bool ddgi_probe_visualization_enabled =
      !reflection_probe_capture && is_scene_camera && ddgi_debug_runtime &&
      ddgi_debug_runtime->frame_resource_layout.valid && ShouldRenderDdgiProbeVisualization(ddgi_settings) &&
      ddgi_debug_runtime->probe_metadata_buffer &&
      ddgi_debug_runtime->probe_metadata_buffer->GetSize() >=
          ddgi_debug_runtime->frame_resource_layout.probe_metadata_byte_size &&
      ddgi_debug_runtime->probe_state_buffer &&
      ddgi_debug_runtime->probe_state_buffer->GetSize() >=
          ddgi_debug_runtime->frame_resource_layout.probe_state_byte_size &&
      ddgi_debug_runtime->irradiance_atlas && ddgi_visualization_probe_count != 0u;
  DdgiProbeVisualizationPushConstant ddgi_probe_visualization_push_constant;
  ddgi_probe_visualization_push_constant.camera_selected_mode = {
      static_cast<uint32_t>(glm::max(camera_index, 0)),
      glm::min(static_cast<uint32_t>(glm::max(ddgi_settings.debug.selected_probe_index, 0)),
               ddgi_visualization_probe_count > 0u ? ddgi_visualization_probe_count - 1u : 0u),
      GetDdgiProbeVisualizationMode(ddgi_settings), 0u};
  ddgi_probe_visualization_push_constant.radius_intensity_alpha_selected_scale = {
      glm::max(ddgi_settings.debug.probe_visualization_radius, 0.001f) *
          glm::max(ddgi_settings.debug.visualization_scale, 0.01f),
      glm::max(ddgi_settings.debug.probe_visualization_intensity, 0.0f),
      glm::clamp(ddgi_settings.debug.probe_visualization_alpha, 0.0f, 1.0f),
      ddgi_settings.debug.visualize_selected_probe
          ? glm::max(ddgi_settings.debug.selected_probe_visualization_scale, 1.0f)
          : 1.0f};
  const bool ddgi_probe_visualization_depth_test = ddgi_settings.debug.probe_visualization_depth_mode == 0;
  const auto selected_ray_sample_count =
      ddgi_debug_runtime ? ddgi_debug_runtime->frame_selected_probe_ray_sample_count : 0u;
  const auto selected_ray_byte_size =
      static_cast<uint64_t>(selected_ray_sample_count) * static_cast<uint64_t>(sizeof(PointCloudSample));
  const bool ddgi_probe_ray_visualization_enabled =
      !reflection_probe_capture && is_scene_camera && ddgi_settings.debug.enabled && ddgi_settings.debug.show_rays &&
      ddgi_debug_runtime && ddgi_debug_runtime->frame_selected_ray_diagnostics_buffer &&
      selected_ray_sample_count != 0u &&
      ddgi_debug_runtime->frame_selected_ray_diagnostics_buffer->GetSize() >= selected_ray_byte_size;
  const auto ray_visualization_miss_distance =
      ddgi_debug_runtime && ddgi_debug_runtime->frame_ray_push_constant.trace_parameters.x > 0.0f
          ? ddgi_debug_runtime->frame_ray_push_constant.trace_parameters.x
          : ddgi_settings.runtime.max_ray_distance;
  DdgiProbeRayVisualizationPushConstant ddgi_probe_ray_visualization_push_constant;
  ddgi_probe_ray_visualization_push_constant.camera_ray_count = {static_cast<uint32_t>(glm::max(camera_index, 0)),
                                                                 selected_ray_sample_count};
  ddgi_probe_ray_visualization_push_constant.miss_distance_alpha = {
      glm::max(ray_visualization_miss_distance, 0.001f),
      glm::clamp(ddgi_settings.debug.probe_visualization_alpha, 0.0f, 1.0f)};
  const auto record_commands = [&](const std::function<void(VkCommandBuffer vk_command_buffer)>& action) {
    if (immediate) {
      Platform::ImmediateSubmit(action);
    } else {
      Platform::RecordCommandsMainQueue(action);
    }
  };
  if (Camera::ResolveCameraRenderMode(camera->camera_render_mode) == Camera::CameraRenderMode::Rasterization) {
    const auto& graphics_settings = ApplicationContext::Get().GetApplicationInfo().graphics_settings;

    const bool count_draw_calls = !reflection_probe_capture && count_shadow_rendering_draw_calls;
    const bool use_mesh_shader = Platform::MeshShaderEnabled() && enable_meshlet;
    RenderGraphTransientResourceStore* active_camera_transient_resources = nullptr;
    RenderGraph camera_render_graph;
    AddDefaultRasterCameraResources(camera_render_graph);
    AddAdvancedFrameResources(camera_render_graph);
    AddAdvancedCameraResources(camera_render_graph);
    AddGaussianSplatCameraResources(camera_render_graph);
    if (volumetric_clouds_enabled) {
      AddVolumetricCloudCameraResources(camera_render_graph,
                                        static_cast<uint32_t>(volumetric_cloud_settings.resolution_divisor));
    }
    if (!reflection_probe_capture) {
      AddExternalRenderResources(camera_render_graph, external_render_resource_descriptors);
    }
    if (ddgi_probe_visualization_enabled) {
      AddDdgiProbeVisualizationFrameResources(camera_render_graph, ddgi_debug_runtime->frame_resource_layout);
    }
    if (ddgi_probe_ray_visualization_enabled) {
      AddDdgiProbeRayVisualizationFrameResources(camera_render_graph, ddgi_debug_runtime->frame_resource_layout);
    }

    camera_render_graph.AddPass(
        DirectionalLightShadowPass::CreateDescriptor(), [&](const RenderGraphExecutionContext& context) {
          const auto& directional_light_shadow_opaque_pipeline = use_mesh_shader
                                                                     ? directional_light_shadow_pipeline_mesh_shader
                                                                     : directional_light_shadow_pipeline_normal_opaque;
          const auto shadow_extent = lighting_->directional_light_shadow_map_->GetExtent();
          DirectionalLightShadowPass::Execute(
              context,
              {current_render_instances,
               directional_light_shadow_opaque_pipeline,
               instanced_directional_light_shadow_pipeline_opaque,
               skinned_directional_light_shadow_pipeline_opaque,
               use_mesh_shader ? strands_directional_light_shadow_pipeline : nullptr,
               per_frame_descriptor_sets_[current_frame_index],
               meshlet_descriptor_sets_[current_frame_index],
               use_mesh_shader ? strand_meshlet_descriptor_sets_[current_frame_index] : nullptr,
               camera_index,
               static_cast<int>(graphics_settings.max_directional_light_size),
               current_frame_index,
               {shadow_extent.width, shadow_extent.height},
               use_mesh_shader,
               enable_indirect_rendering,
               count_draw_calls,
               [&](const uint32_t split, const VkAttachmentLoadOp load_op, const VkAttachmentStoreOp store_op) {
                 return lighting_->GetLayeredDirectionalLightDepthAttachmentInfo(split, load_op, store_op);
               },
               [&](const VkCommandBuffer vk_command_buffer, const int light_index, const int split_index,
                   const glm::ivec4& viewport, const glm::mat4& light_space_matrix) {
                 if (!reflection_probe_capture) {
                   for (const auto& func : directional_light_shadow_map_external_functions) {
                     const auto prim_count =
                         func(vk_command_buffer, {light_index, split_index, viewport, light_space_matrix});
                     if (count_draw_calls) {
                       Platform::CountRenderPassDraw(RenderPassDrawBucket::DirectionalLightShadow,
                                                     RenderDrawCallKind::Direct, current_frame_index, prim_count);
                     }
                   }
                 }
               },
               record_commands});
        });
    camera_render_graph.AddPass(
        DeferredGeometryPass::CreateDescriptor(), [&](const RenderGraphExecutionContext& context) {
          const auto& deferred_prepass_pipeline =
              use_mesh_shader ? deferred_prepass_pipeline_mesh : deferred_prepass_pipeline_normal;
          DeferredGeometryPass::Execute(
              context,
              {camera, current_render_instances, deferred_prepass_pipeline, instanced_deferred_prepass_pipeline,
               skinned_deferred_prepass_pipeline, use_mesh_shader ? strands_deferred_prepass_pipeline : nullptr,
               raster_material_per_frame_descriptor_sets_[current_frame_index],
               meshlet_descriptor_sets_[current_frame_index],
               use_mesh_shader ? strand_meshlet_descriptor_sets_[current_frame_index] : nullptr, camera_index,
               current_frame_index, use_mesh_shader, enable_indirect_rendering, true, count_draw_calls,
               reflection_probe_capture ? false : wire_frame,
               [&](const VkCommandBuffer vk_command_buffer,
                   const std::vector<VkRenderingAttachmentInfo>& color_attachment_infos, const glm::ivec4& viewport) {
                 if (!reflection_probe_capture) {
                   for (const auto& func : deferred_rendering_external_functions) {
                     const auto prim_count = func(vk_command_buffer, color_attachment_infos, {camera_index, viewport});
                     if (count_draw_calls) {
                       Platform::CountRenderPassDraw(RenderPassDrawBucket::DeferredGeometry, RenderDrawCallKind::Direct,
                                                     current_frame_index, prim_count);
                     }
                   }
                 }
               },
               record_commands});
        });
    camera_render_graph.AddPass(MotionVectorPass::CreateDescriptor(), [&](const RenderGraphExecutionContext& context) {
      MotionVectorPass::Execute(
          context,
          {camera, current_render_instances, per_frame_descriptor_sets_[current_frame_index], motion_vectors_pipeline_,
           motion_vectors_layout_, active_camera_transient_resources, camera_index, record_commands});
    });
    camera_render_graph.AddPass(
        MotionCoveragePass::CreateDescriptor(), [&](const RenderGraphExecutionContext& context) {
          MotionCoveragePass::Execute(
              context,
              {camera, current_render_instances, raster_material_per_frame_descriptor_sets_[current_frame_index],
               skinned_motion_vectors_pipeline_, transparent_motion_vectors_pipeline_, motion_coverage_layout_,
               active_camera_transient_resources, camera_index, reflection_probe_capture ? false : wire_frame,
               record_commands});
        });
    camera_render_graph.AddPass(DepthPyramidPass::CreateDescriptor(), [&](const RenderGraphExecutionContext& context) {
      DepthPyramidPass::Execute(context, {camera, depth_pyramid_pipeline_, depth_pyramid_layout_,
                                          active_camera_transient_resources, record_commands});
    });
    if (ambient_occlusion_enabled) {
      camera_render_graph.AddPass(AmbientOcclusionPass::CreateDescriptor(),
                                  [&](const RenderGraphExecutionContext& context) {
                                    AmbientOcclusionPass::Execute(context, {camera, active_camera_transient_resources});
                                  });
    }
    if (is_scene_camera && Platform::GpuTimestampCaptureEnabled() &&
        ddgi_last_performance_stats_.lighting_descriptors_bound) {
      camera_render_graph.AddPass(
          DdgiGatherTimingPass::CreateDescriptor(), [&](const RenderGraphExecutionContext& context) {
            DdgiGatherTimingPass::Execute(context, {camera, ddgi_gather_timing_pipeline_,
                                                    raster_material_per_frame_descriptor_sets_[current_frame_index],
                                                    lighting_descriptor_set, raster_lighting_texture_descriptor_set,
                                                    active_camera_transient_resources, camera_index, record_commands});
          });
    }
    camera_render_graph.AddPass(
        DeferredLightingPass::CreateDescriptor(ambient_occlusion_enabled),
        [&](const RenderGraphExecutionContext& context) {
          const bool fade_selection =
              !reflection_probe_capture && need_fade_ && editor_layer && editor_layer->highlight_selection_;
          const int selection_alpha = editor_layer ? editor_layer->selection_alpha_ : 0;
          const auto& deferred_lighting_pipeline =
              is_scene_camera ? deferred_lighting_pass_pipeline_scene_camera : deferred_lighting_pass_pipeline;
          DeferredLightingPass::Execute(
              context,
              {camera, deferred_lighting_pipeline, raster_material_per_frame_descriptor_sets_[current_frame_index],
               lighting_descriptor_set, raster_lighting_texture_descriptor_set, camera_index, current_frame_index,
               count_draw_calls, fade_selection, selection_alpha,
               [&](const VkCommandBuffer vk_command_buffer, const glm::ivec4& viewport) {
                 if (!reflection_probe_capture) {
                   for (const auto& func : forward_rendering_external_functions) {
                     const auto prim_count = func(vk_command_buffer, camera, {camera_index, viewport});
                     if (count_draw_calls) {
                       Platform::CountRenderPassDraw(RenderPassDrawBucket::ForwardExternal, RenderDrawCallKind::Direct,
                                                     current_frame_index, prim_count);
                     }
                   }
                 }
               },
               record_commands});
        });
    if (!reflection_probe_capture) {
      for (const auto& external_pass : camera_render_pass_external_functions) {
        ImportMissingPassResources(camera_render_graph, external_pass.descriptor);
        camera_render_graph.AddPass(
            external_pass.descriptor, [&, external_pass](const RenderGraphExecutionContext& context) {
              record_commands([&, external_pass](VkCommandBuffer vk_command_buffer) {
                glm::ivec4 view_port;
                view_port.x = 0;
                view_port.y = 0;
                view_port.z = camera->GetSize().x;
                view_port.w = camera->GetSize().y;
                uint32_t prim_count = 0;
                if (external_pass.context_func) {
                  ApplyGraphResourceBarriers(vk_command_buffer, context);
                  prim_count =
                      external_pass.context_func(vk_command_buffer, camera, {camera_index, view_port}, context);
                  ApplyGraphResourceReleaseBarriers(vk_command_buffer, context, RenderPassQueue::Graphics);
                } else if (external_pass.func) {
                  prim_count = external_pass.func(vk_command_buffer, camera, {camera_index, view_port});
                }
                if (count_draw_calls) {
                  Platform::CountRenderPassDraw(RenderPassDrawBucket::CameraExternal, RenderDrawCallKind::Direct,
                                                current_frame_index, prim_count);
                }
              });
            });
      }
    }
    const bool gaussian_splat_rendering_enabled = !reflection_probe_capture && current_render_instances &&
                                                  current_render_instances->total_gaussian_splats != 0u &&
                                                  current_render_instances->gaussian_splat_render_instances &&
                                                  !current_render_instances->gaussian_splat_render_instances->Empty();
    const bool transparent_mesh_rendering_enabled = !reflection_probe_capture && current_render_instances &&
                                                    current_render_instances->transparent_render_instances &&
                                                    !current_render_instances->transparent_render_instances->Empty();
    const char* post_lighting_dependency = RenderPassNames::deferred_camera;
    if (volumetric_clouds_enabled) {
      camera_render_graph.AddPass(
          VolumetricCloudsPass::CreateRasterDescriptor(post_lighting_dependency),
          [&](const RenderGraphExecutionContext& context) {
            const auto time_seconds = static_cast<float>(ApplicationContext::Get().GetTimes().Now());
            VolumetricCloudsPass::Execute(
                context, {camera, record_commands, volumetric_clouds_pipeline_, volumetric_clouds_composite_pipeline_,
                          per_frame_descriptor_sets_[current_frame_index], volumetric_clouds_layout_,
                          active_camera_transient_resources, volumetric_cloud_settings, camera_index,
                          static_cast<uint32_t>(glm::max(0.0f, std::floor(time_seconds * 60.0f))), time_seconds,
                          camera->camera_settings.far_distance});
          });
      post_lighting_dependency = RenderPassNames::volumetric_clouds;
    }
    if (transparent_mesh_rendering_enabled) {
      camera_render_graph.AddPass(
          TransparentGeometryPass::CreateDescriptor(post_lighting_dependency),
          [&](const RenderGraphExecutionContext& context) {
            TransparentGeometryPass::Execute(
                context, {camera, current_render_instances, transparent_geometry_pipeline_normal,
                          raster_material_per_frame_descriptor_sets_[current_frame_index], lighting_descriptor_set,
                          raster_lighting_texture_descriptor_set, camera_index, current_frame_index, count_draw_calls,
                          reflection_probe_capture ? false : wire_frame, record_commands});
          });
      post_lighting_dependency = RenderPassNames::transparent_geometry;
    }
    if (gaussian_splat_rendering_enabled) {
      camera_render_graph.AddPass(GaussianSplatCullPass::CreateDescriptor(post_lighting_dependency),
                                  [&](const RenderGraphExecutionContext& context) {
                                    GaussianSplatCullPass::Execute(
                                        context, {camera, current_render_instances, gaussian_splat_cull_pipeline_,
                                                  per_frame_descriptor_sets_[current_frame_index],
                                                  gaussian_splat_layout_, active_camera_transient_resources,
                                                  static_cast<uint32_t>(glm::max(camera_index, 0)), record_commands});
                                  });
      camera_render_graph.AddPass(
          GaussianSplatSortPass::CreateDescriptor(RenderPassNames::gaussian_splat_cull),
          [&](const RenderGraphExecutionContext& context) {
            GaussianSplatSortPass::Execute(
                context, {camera, current_render_instances, gaussian_splat_radix_upsweep_pipeline_,
                          gaussian_splat_radix_spine_pipeline_, gaussian_splat_radix_downsweep_pipeline_,
                          gaussian_splat_radix_sort_layout_, active_camera_transient_resources, record_commands});
          });
      camera_render_graph.AddPass(
          GaussianSplatPass::CreateDescriptor(RenderPassNames::gaussian_splat_sort),
          [&](const RenderGraphExecutionContext& context) {
            GaussianSplatPass::Execute(
                context, {camera, current_render_instances, gaussian_splat_pipeline_, gaussian_splat_mesh_pipeline_,
                          per_frame_descriptor_sets_[current_frame_index], gaussian_splat_layout_,
                          active_camera_transient_resources, static_cast<uint32_t>(glm::max(camera_index, 0)), true,
                          Platform::MeshShaderEnabled() && enable_meshlet, record_commands});
          });
      post_lighting_dependency = RenderPassNames::gaussian_splat;
    }
    if (ddgi_probe_visualization_enabled) {
      camera_render_graph.AddPass(
          DdgiProbeVisualizationPass::CreateDescriptor(),
          [&, ddgi_probe_visualization_push_constant](const RenderGraphExecutionContext& context) {
            DdgiProbeVisualizationPass::Execute(
                context, {ddgi_probe_visualization_pipeline_, per_frame_descriptor_sets_[current_frame_index],
                          ddgi_probe_visualization_layout_, active_camera_transient_resources, ddgi_atlas_sampler_,
                          camera, ddgi_visualization_probe_count, ddgi_probe_visualization_depth_test,
                          ddgi_probe_visualization_push_constant,
                          &ddgi_last_performance_stats_.probe_visualization_record_ms, record_commands});
            ddgi_last_performance_stats_.visualized_probe_count += ddgi_visualization_probe_count;
            if (count_draw_calls) {
              Platform::CountRenderPassDraw(RenderPassDrawBucket::DdgiProbeVisualization, RenderDrawCallKind::Direct,
                                            current_frame_index, ddgi_visualization_probe_count);
            }
          });
    }
    if (ddgi_probe_ray_visualization_enabled) {
      const auto* dependency =
          ddgi_probe_visualization_enabled ? RenderPassNames::ddgi_probe_visualization : post_lighting_dependency;
      camera_render_graph.AddPass(
          DdgiProbeRayVisualizationPass::CreateDescriptor(dependency),
          [&, ddgi_probe_ray_visualization_push_constant](const RenderGraphExecutionContext& context) {
            DdgiProbeRayVisualizationPass::Execute(
                context, {ddgi_probe_ray_visualization_pipeline_, per_frame_descriptor_sets_[current_frame_index],
                          ddgi_probe_ray_visualization_layout_, active_camera_transient_resources, camera,
                          ddgi_probe_visualization_depth_test, ddgi_probe_ray_visualization_push_constant,
                          &ddgi_last_performance_stats_.probe_ray_visualization_record_ms, record_commands});
            if (count_draw_calls) {
              Platform::CountRenderPassDraw(RenderPassDrawBucket::DdgiProbeRayVisualization, RenderDrawCallKind::Direct,
                                            current_frame_index, selected_ray_sample_count);
            }
          });
    }
    const auto* ddgi_debug_post_processing_dependency =
        ddgi_probe_ray_visualization_enabled
            ? RenderPassNames::ddgi_probe_ray_visualization
            : (ddgi_probe_visualization_enabled ? RenderPassNames::ddgi_probe_visualization : post_lighting_dependency);
    camera_render_graph.AddPass(PostProcessingPass::CreateDescriptor(ddgi_debug_post_processing_dependency),
                                [&](const RenderGraphExecutionContext& context) {
                                  PostProcessingPass::Execute(context,
                                                              {camera, active_camera_transient_resources, immediate});
                                });
    if (!camera_render_graph.Validate()) {
      EVOENGINE_ERROR("Invalid camera render graph.")
    }
    const auto camera_render_graph_plan = camera_render_graph.Compile(CreateCameraRenderGraphCompileContext(camera));
    auto camera_render_graph_resources =
        CreateCameraRenderGraphResourceRegistry(per_frame_descriptor_sets_[current_frame_index], {}, camera);
    if (camera) {
      camera_render_graph_resources.BindImages(
          RenderResourceNames::camera_g_buffer,
          {camera->g_buffer_base_color_ao_, camera->g_buffer_normal_roughness_, camera->g_buffer_pbr_flags_,
           camera->g_buffer_emissive_, camera->g_buffer_utility_});
    }
    if (lighting_ && lighting_->directional_light_shadow_map_) {
      camera_render_graph_resources.BindImage(RenderResourceNames::lighting_directional_shadow_map,
                                              lighting_->directional_light_shadow_map_);
    }
    if (ddgi_probe_visualization_enabled) {
      camera_render_graph_resources.BindBuffer(RenderResourceNames::frame_ddgi_probe_metadata,
                                               ddgi_debug_runtime->probe_metadata_buffer);
      camera_render_graph_resources.BindBuffer(RenderResourceNames::frame_ddgi_probe_state,
                                               ddgi_debug_runtime->probe_state_buffer);
      camera_render_graph_resources.BindImage(RenderResourceNames::frame_ddgi_irradiance_atlas,
                                              ddgi_debug_runtime->irradiance_atlas);
    }
    if (ddgi_probe_ray_visualization_enabled) {
      camera_render_graph_resources.BindBuffer(RenderResourceNames::frame_ddgi_selected_ray_diagnostics,
                                               ddgi_debug_runtime->frame_selected_ray_diagnostics_buffer);
    }
    auto& camera_transient_resources = render_graph_transient_resource_stores_.at(current_frame_index).emplace_back();
    active_camera_transient_resources = &camera_transient_resources;
    camera_transient_resources.Allocate(camera_render_graph.GetResources(), camera_render_graph_plan);
    camera_transient_resources.Bind(camera_render_graph_resources);
    if (ambient_occlusion_enabled && raster_lighting_texture_descriptor_set &&
        post_processing_renderer_resources_->ambient_occlusion.sampler) {
      const auto* binding =
          camera_render_graph_resources.GetResourceBinding(RenderResourceNames::camera_ambient_occlusion);
      if (binding && binding->image) {
        const auto image_view = CreateGraphImageMipView(binding->image, 0);
        VkDescriptorImageInfo image_info{};
        image_info.imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
        image_info.imageView = image_view->GetVkImageView();
        image_info.sampler = post_processing_renderer_resources_->ambient_occlusion.sampler->GetVkSampler();
        raster_lighting_texture_descriptor_set->UpdateImageDescriptorBinding(kRasterLightingAmbientOcclusionBinding,
                                                                             image_info);
        camera_transient_resources.RetainImageView(image_view);
      }
    }
    const ScopedRenderCameraDrawScope camera_draw_scope(current_frame_index, scene, camera, is_scene_camera);
    camera_render_graph.Execute(camera_render_graph_plan, camera_render_graph_resources);
    camera->rendered_ = true;
    camera->require_rendering_ = false;
    camera->frame_count_++;
  }
}

void RenderLayer::RenderToCameraRayTracing(const std::shared_ptr<Scene>& scene,
                                           const GlobalTransform& camera_global_transform,
                                           const std::shared_ptr<Camera>& camera) const {
  const ProfilerScope profiler_scope("RenderLayer::RenderToCameraRayTracing", "Render");
  if (!camera || !camera->ray_camera_history_owner_alive_) {
    return;
  }
  const auto render_texture = camera->GetRenderTexture();
  if (!render_texture) {
    return;
  }
  const auto current_frame_index = Platform::GetCurrentFrameIndex();
  const auto current_render_instances = render_instances_list_[current_frame_index];
  const int camera_index = current_render_instances->GetCameraIndex(camera->GetHandle());
  const auto editor_layer = ApplicationContext::Get().GetLayer<EditorLayer>();
  const bool is_scene_camera = editor_layer && camera.get() == editor_layer->GetSceneCamera().get();

  const auto resolved_render_mode = Camera::ResolveCameraRenderMode(camera->camera_render_mode);
  if (resolved_render_mode == Camera::CameraRenderMode::RayTracing ||
      resolved_render_mode == Camera::CameraRenderMode::RayQuery) {
    const bool use_ray_query = resolved_render_mode == Camera::CameraRenderMode::RayQuery;
    const auto ray_query_pipeline = ray_query_camera_pipeline_;
    const auto ray_tracing_pipeline = ray_tracing_camera_pipeline;
    if ((use_ray_query && (!ray_query_pipeline || !ray_query_pipeline->Initialized())) ||
        (!use_ray_query && (!ray_tracing_pipeline || !ray_tracing_pipeline->Initialized()))) {
      return;
    }
    const auto camera_handle = camera->GetHandle().GetValue();
    ray_camera_history_cameras_[camera_handle] = camera;
    auto& ray_camera_history = camera->AcquireRayCameraHistory(
        use_ray_query ? RayCameraHistoryTechnique::RayQuery : RayCameraHistoryTechnique::RayTracing,
        scene ? scene->GetHandle().GetValue() : 0u, render_texture->GetExtent());
    const auto ray_camera_output_descriptor = camera->AcquireRayCameraOutputDescriptor(
        current_frame_index, Platform::GetFrameCount(), ray_tracing_camera_output_layout_);
    UpdateRayCameraHistoryPeaks();
    if (!ray_camera_output_descriptor) {
      return;
    }
    const char* ray_camera_pass_name =
        use_ray_query ? RenderPassNames::ray_query_camera : RenderPassNames::ray_tracing_camera;
    VolumetricCloudSettings volumetric_cloud_settings{};
    const bool volumetric_clouds_enabled = volumetric_cloud_settings.enabled;
    const bool gaussian_splat_rendering_enabled = current_render_instances &&
                                                  current_render_instances->total_gaussian_splats != 0u &&
                                                  current_render_instances->gaussian_splat_render_instances &&
                                                  !current_render_instances->gaussian_splat_render_instances->Empty();
    const auto record_commands = [](const std::function<void(VkCommandBuffer vk_command_buffer)>& action) {
      Platform::RecordCommandsMainQueue(action);
    };
    RenderGraphTransientResourceStore* active_camera_transient_resources = nullptr;
    RenderGraph camera_render_graph;
    AddDefaultRayTracingCameraResources(camera_render_graph);
    AddAdvancedFrameResources(camera_render_graph);
    AddAdvancedCameraResources(camera_render_graph);
    AddGaussianSplatCameraResources(camera_render_graph);
    if (volumetric_clouds_enabled) {
      AddVolumetricCloudCameraResources(camera_render_graph,
                                        static_cast<uint32_t>(volumetric_cloud_settings.resolution_divisor));
    }
    AddExternalRenderResources(camera_render_graph, external_render_resource_descriptors);
    if (use_ray_query) {
      camera_render_graph.AddPass(
          RayQueryCameraPass::CreateDescriptor(), [&](const RenderGraphExecutionContext& context) {
            RayQueryCameraPass::Execute(
                context, {camera, ray_query_pipeline, per_frame_descriptor_sets_[current_frame_index],
                          ray_tracing_descriptor_sets_[current_frame_index], camera_index, record_commands,
                          ray_camera_output_descriptor, active_camera_transient_resources, &ray_camera_history});
          });
    } else {
      camera_render_graph.AddPass(
          RayTracingCameraPass::CreateDescriptor(), [&](const RenderGraphExecutionContext& context) {
            RayTracingCameraPass::Execute(
                context, {camera, ray_tracing_pipeline, per_frame_descriptor_sets_[current_frame_index],
                          ray_tracing_descriptor_sets_[current_frame_index], camera_index, record_commands,
                          ray_camera_output_descriptor, active_camera_transient_resources, &ray_camera_history});
          });
    }
    const char* post_ray_tracing_dependency = ray_camera_pass_name;
    if (volumetric_clouds_enabled) {
      camera_render_graph.AddPass(
          VolumetricCloudsPass::CreateRayTracingDescriptor(ray_camera_pass_name),
          [&](const RenderGraphExecutionContext& context) {
            const auto time_seconds = static_cast<float>(ApplicationContext::Get().GetTimes().Now());
            VolumetricCloudsPass::Execute(
                context, {camera, record_commands, volumetric_clouds_pipeline_, volumetric_clouds_composite_pipeline_,
                          per_frame_descriptor_sets_[current_frame_index], volumetric_clouds_layout_,
                          active_camera_transient_resources, volumetric_cloud_settings, camera_index,
                          camera->frame_count_, time_seconds, camera->camera_settings.far_distance,
                          RenderResourceNames::camera_ray_hit_distance, true});
          });
      post_ray_tracing_dependency = RenderPassNames::volumetric_clouds;
    }
    if (gaussian_splat_rendering_enabled) {
      camera_render_graph.AddPass(GaussianSplatCullPass::CreateDescriptor(post_ray_tracing_dependency),
                                  [&](const RenderGraphExecutionContext& context) {
                                    GaussianSplatCullPass::Execute(
                                        context, {camera, current_render_instances, gaussian_splat_cull_pipeline_,
                                                  per_frame_descriptor_sets_[current_frame_index],
                                                  gaussian_splat_layout_, active_camera_transient_resources,
                                                  static_cast<uint32_t>(glm::max(camera_index, 0)), record_commands});
                                  });
      camera_render_graph.AddPass(
          GaussianSplatSortPass::CreateDescriptor(RenderPassNames::gaussian_splat_cull),
          [&](const RenderGraphExecutionContext& context) {
            GaussianSplatSortPass::Execute(
                context, {camera, current_render_instances, gaussian_splat_radix_upsweep_pipeline_,
                          gaussian_splat_radix_spine_pipeline_, gaussian_splat_radix_downsweep_pipeline_,
                          gaussian_splat_radix_sort_layout_, active_camera_transient_resources, record_commands});
          });
      camera_render_graph.AddPass(
          GaussianSplatPass::CreateOverlayDescriptor(RenderPassNames::gaussian_splat_sort),
          [&](const RenderGraphExecutionContext& context) {
            GaussianSplatPass::Execute(
                context, {camera, current_render_instances, gaussian_splat_overlay_pipeline_,
                          gaussian_splat_mesh_overlay_pipeline_, per_frame_descriptor_sets_[current_frame_index],
                          gaussian_splat_layout_, active_camera_transient_resources,
                          static_cast<uint32_t>(glm::max(camera_index, 0)), false,
                          Platform::MeshShaderEnabled() && enable_meshlet, record_commands});
          });
      post_ray_tracing_dependency = RenderPassNames::gaussian_splat;
    }
    camera_render_graph.AddPass(PostProcessingPass::CreateRayTracingDescriptor(post_ray_tracing_dependency),
                                [&](const RenderGraphExecutionContext& context) {
                                  PostProcessingPass::Execute(context,
                                                              {camera, active_camera_transient_resources, false, true});
                                });
    if (!camera_render_graph.Validate()) {
      EVOENGINE_ERROR("Invalid ray tracing camera render graph.")
    }
    const auto& camera_render_graph_plan = ray_camera_render_graph_plan_cache_.GetOrCompile(
        camera_render_graph, CreateCameraRenderGraphCompileContext(camera));
    auto camera_render_graph_resources = CreateCameraRenderGraphResourceRegistry(
        per_frame_descriptor_sets_[current_frame_index], ray_tracing_descriptor_sets_[current_frame_index], camera);
    auto& camera_transient_resources = render_graph_transient_resource_stores_.at(current_frame_index).emplace_back();
    active_camera_transient_resources = &camera_transient_resources;
    camera_transient_resources.Allocate(camera_render_graph.GetResources(), camera_render_graph_plan);
    camera_transient_resources.Bind(camera_render_graph_resources);
    const ScopedRenderCameraDrawScope camera_draw_scope(current_frame_index, scene, camera, is_scene_camera);
    camera_render_graph.Execute(camera_render_graph_plan, camera_render_graph_resources);
    if (ray_camera_shader_variant_cache_) {
      ray_camera_shader_variant_cache_->RecordActiveUse(use_ray_query ? RayCameraShaderTechnique::RayQuery
                                                                      : RayCameraShaderTechnique::RayTracing);
    }
    camera->rendered_ = true;
    camera->require_rendering_ = false;
    camera->frame_count_++;
  }
}

void RenderLayer::PreUpdate() {
  const ProfilerScope profiler_scope("RenderLayer::PreUpdate", "Render");
  const auto scene = GetScene();
  if (!scene)
    return;
  const auto current_frame_index = Platform::GetCurrentFrameIndex();
  const auto current_render_instances = render_instances_list_[current_frame_index];
  current_render_instances->Clear();
  scene->SetBound({});
}

void RenderLayer::OnDestroy() {
  if (ray_camera_shader_variant_cache_)
    ray_camera_shader_variant_cache_->WaitForJobs();
  Platform::DrainGpuResourceWork();
  post_processing_renderer_resources_.reset();
  ray_camera_shader_variant_cache_.reset();
  ray_tracing_camera_pipeline.reset();
  ray_tracing_camera_fallback_pipeline_.reset();
  ray_query_camera_pipeline_.reset();
  ray_query_camera_fallback_pipeline_.reset();
  render_graph_transient_resource_stores_.clear();
  ray_camera_render_graph_plan_cache_.Clear();
  ClearRayCameraHistories();
}

const std::shared_ptr<PostProcessingRendererResources>& RenderLayer::GetPostProcessingRendererResources() const {
  return post_processing_renderer_resources_;
}

void RenderLayer::PruneRayCameraHistories(const std::shared_ptr<RenderInstanceStorage>& render_instances) const {
  std::unordered_map<uint64_t, const Camera*> active_cameras;
  if (render_instances) {
    active_cameras.reserve(render_instances->cameras.size());
    for (const auto& [transform, camera] : render_instances->cameras) {
      if (camera) {
        active_cameras[camera->GetHandle().GetValue()] = camera.get();
      }
    }
  }
  for (auto iterator = ray_camera_history_cameras_.begin(); iterator != ray_camera_history_cameras_.end();) {
    const auto camera = iterator->second.lock();
    const auto active_search = active_cameras.find(iterator->first);
    if (!camera || active_search == active_cameras.end() || active_search->second != camera.get()) {
      if (camera) {
        ArchiveRayCameraHistory(camera);
      }
      iterator = ray_camera_history_cameras_.erase(iterator);
    } else {
      ++iterator;
    }
  }
}

void RenderLayer::ForgetRayCameraHistoryCamera(const uint64_t camera_handle, const Camera* camera) const {
  if (const auto search = ray_camera_history_cameras_.find(camera_handle);
      search != ray_camera_history_cameras_.end()) {
    const auto locked_camera = search->second.lock();
    if (!locked_camera) {
      ray_camera_history_cameras_.erase(search);
    } else if (locked_camera.get() == camera) {
      ArchiveRayCameraHistory(locked_camera);
      ray_camera_history_cameras_.erase(search);
    }
  }
}

void RenderLayer::ArchiveRayCameraHistory(const std::shared_ptr<Camera>& camera) const {
  camera->ReleaseRayCameraHistory();
  const auto stats = camera->GetRayCameraHistoryStats();
  retired_ray_camera_history_stats_.creation_count += stats.creation_count;
  retired_ray_camera_history_stats_.reuse_count += stats.reuse_count;
  retired_ray_camera_history_stats_.invalidation_count += stats.invalidation_count;
  retired_ray_camera_history_stats_.retirement_count += stats.retirement_count;
  retired_ray_camera_history_stats_.output_descriptor_creation_count += stats.output_descriptor_creation_count;
  retired_ray_camera_history_stats_.output_descriptor_reuse_count += stats.output_descriptor_reuse_count;
  retired_ray_camera_history_stats_.peak_live_output_descriptor_count = std::max(
      retired_ray_camera_history_stats_.peak_live_output_descriptor_count, stats.peak_live_output_descriptor_count);
  camera->ray_camera_history_counters_ = {};
}

void RenderLayer::UpdateRayCameraHistoryPeaks() const {
  uint64_t live_history_count = 0;
  uint64_t live_byte_size = 0;
  uint64_t live_output_descriptor_count = 0;
  for (const auto& [handle, camera] : ray_camera_history_cameras_) {
    if (const auto locked_camera = camera.lock()) {
      const auto stats = locked_camera->GetRayCameraHistoryStats();
      live_history_count += stats.live_history_count;
      live_byte_size += stats.live_byte_size;
      live_output_descriptor_count += stats.live_output_descriptor_count;
    }
  }
  peak_live_ray_camera_history_count_ = std::max(peak_live_ray_camera_history_count_, live_history_count);
  peak_live_ray_camera_history_byte_size_ = std::max(peak_live_ray_camera_history_byte_size_, live_byte_size);
  peak_live_ray_camera_output_descriptor_count_ =
      std::max(peak_live_ray_camera_output_descriptor_count_, live_output_descriptor_count);
}

void RenderLayer::ClearRayCameraHistories() const {
  for (const auto& [handle, camera] : ray_camera_history_cameras_) {
    if (const auto locked_camera = camera.lock()) {
      ArchiveRayCameraHistory(locked_camera);
    }
  }
  ray_camera_history_cameras_.clear();
}

uint32_t RenderLayer::DrawMesh(const std::shared_ptr<Mesh>& mesh, const std::shared_ptr<Material>& material,
                               const GlobalTransform& global_transform, const bool cast_shadow) const {
  const auto current_frame_index = Platform::GetCurrentFrameIndex();
  const auto current_render_instances = render_instances_list_[current_frame_index];
  return current_render_instances->RegisterMeshDrawCommand(mesh, material, global_transform, cast_shadow);
}

uint32_t RenderLayer::DrawMeshInstanced(const std::shared_ptr<Mesh>& mesh, const std::shared_ptr<Material>& material,
                                        const GlobalTransform& global_transform,
                                        const std::shared_ptr<ParticleInfoList>& particle_info_list,
                                        const bool cast_shadow) const {
  const auto current_frame_index = Platform::GetCurrentFrameIndex();
  const auto current_render_instances = render_instances_list_[current_frame_index];
  return current_render_instances->RegisterMeshDrawInstancedCommand(mesh, material, global_transform,
                                                                    particle_info_list, cast_shadow);
}

const std::shared_ptr<DescriptorSet>& RenderLayer::GetPerFrameDescriptorSet() {
  return ApplicationContext::Get()
      .GetLayer<RenderLayer>()
      ->per_frame_descriptor_sets_[Platform::GetCurrentFrameIndex()];
}

const std::shared_ptr<DescriptorSet>& RenderLayer::GetLightingDescriptorSet() {
  return ApplicationContext::Get().GetLayer<RenderLayer>()->lighting_->lighting_descriptor_sets_.at(
      Platform::GetCurrentFrameIndex());
}
