#include "RenderLayer.hpp"
#include "Application.hpp"
#include "AssetManager.hpp"
#include "ComputePipeline.hpp"
#include "DdgiVolume.hpp"
#include "EditorLayer.hpp"
#include "EnvironmentalMap.hpp"
#include "GeometryStorage.hpp"
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
#include "RenderPasses/DdgiPassUtilities.hpp"
#include "RenderPasses/DdgiProbeClassificationPass.hpp"
#include "RenderPasses/DdgiProbeRayVisualizationPass.hpp"
#include "RenderPasses/DdgiProbeRelocationPass.hpp"
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
#include <numeric>
#include <vector>
using namespace evo_engine;

namespace {
using DdgiPerformanceClock = std::chrono::steady_clock;
constexpr uint32_t kDdgiProbeVariabilityStableSampleCount = 1;
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

glm::vec4 CreateDdgiProbeRayRotationQuaternion(const uint32_t frame_index, const uint32_t volume_index) {
  constexpr float two_pi = 6.28318530718f;
  const auto seed = frame_index ^ (volume_index * 0x9e3779b9u);
  const float u1 = DdgiProbeRayRotationUnitFloat(seed ^ 0x68bc21ebu);
  const float u2 = DdgiProbeRayRotationUnitFloat(seed ^ 0x02e5be93u);
  const float u3 = DdgiProbeRayRotationUnitFloat(seed ^ 0x967a889bu);
  const float r1 = std::sqrt(glm::max(0.0f, 1.0f - u1));
  const float r2 = std::sqrt(glm::max(0.0f, u1));
  return {r1 * std::sin(two_pi * u2), r1 * std::cos(two_pi * u2), r2 * std::sin(two_pi * u3),
          r2 * std::cos(two_pi * u3)};
}

float ReadDdgiProbeVariabilityAverage(const std::shared_ptr<Buffer>& buffer) {
  if (!buffer || buffer->GetSize() < sizeof(glm::vec2)) {
    return 0.0f;
  }
  glm::vec2 result(0.0f);
  buffer->Download(result);
  return std::isfinite(result.x) ? glm::max(result.x, 0.0f) : 0.0f;
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

std::shared_ptr<GraphicsPipeline> CreateGaussianSplatPipeline(
    const std::shared_ptr<DescriptorSetLayout>& per_frame_layout,
    const std::shared_ptr<DescriptorSetLayout>& gaussian_splat_layout, const VkFormat depth_attachment_format,
    const bool use_mesh_shader = false) {
  auto pipeline = std::make_shared<GraphicsPipeline>();
  if (use_mesh_shader) {
    pipeline->mesh_shader = Shader::CreateTemporary(
        ShaderType::Mesh, Platform::GetShaderGlobalDefines(),
        Resources::GetDefaultResourcesPath() / "Shaders/Graphics/Mesh/GaussianSplat/GaussianSplat.mesh");
  } else {
    pipeline->vertex_shader = Shader::CreateTemporary(
        ShaderType::Vertex, Platform::GetShaderGlobalDefines(),
        Resources::GetDefaultResourcesPath() / "Shaders/Graphics/Vertex/GaussianSplat/GaussianSplat.vert");
  }
  pipeline->fragment_shader = Shader::CreateTemporary(
      ShaderType::Fragment, Platform::GetShaderGlobalDefines(),
      Resources::GetDefaultResourcesPath() / "Shaders/Graphics/Fragment/GaussianSplat/GaussianSplat.frag");
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
  return settings.runtime.enabled ||
         (settings.debug.enabled &&
          (settings.debug.show_atlas_preview || settings.debug.show_irradiance || settings.debug.show_visibility ||
           settings.debug.show_rays || settings.debug.visualize_probe_illumination));
}

bool ShouldTraceDdgiProbeRays(const RenderLayer::DdgiSettings& settings) {
  return !settings.runtime.pause_updates &&
         (settings.runtime.enabled || (settings.debug.enabled && settings.debug.show_rays));
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
  uint32_t enabled_volume_count = 0;
  uint32_t selected_volume_index = 0;
  float relocation_distance = 0.0f;
  float random_ray_backface_threshold = 0.1f;
  float fixed_ray_backface_threshold = 0.25f;
  float probe_variability_threshold = 0.2f;
  int probe_variability_min_samples = 16;
  int warmup_trigger_conditions = DdgiVolumeTriggerConditionLightEnableChanged;
  int variability_reset_trigger_conditions =
      DdgiVolumeTriggerConditionLightingConditionChanged | DdgiVolumeTriggerConditionGeometryChanged;
  int movement_type = static_cast<int>(DdgiVolumeMovementType::Default);
  glm::ivec3 probe_scroll_offset = glm::ivec3(0);
  glm::ivec3 probe_scroll_clear = glm::ivec3(0);
  glm::ivec3 probe_scroll_directions = glm::ivec3(1);
  bool enable_probe_relocation = false;
  bool enable_probe_classification = false;
  bool enable_probe_variability = true;
  bool enable_probe_variability_gating = true;
};

struct DdgiVolumeCandidate {
  std::shared_ptr<DdgiVolume> volume{};
  Entity owner{};
  glm::mat4 transform = glm::mat4(1.0f);
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

void ResetDdgiProbeScrollOrigin(glm::vec3& base_first_probe, glm::ivec3& probe_scroll_offset,
                                const glm::ivec3& probe_scroll_directions, const glm::ivec3& probe_counts,
                                const glm::vec3& probe_step_x, const glm::vec3& probe_step_y,
                                const glm::vec3& probe_step_z) {
  const auto counts = ClampDdgiProbeCounts(probe_counts);
  const glm::vec3 probe_steps[3] = {probe_step_x, probe_step_y, probe_step_z};
  for (int axis = 0; axis < 3; ++axis) {
    if (probe_scroll_offset[axis] != 0 && probe_scroll_offset[axis] % counts[axis] == 0) {
      base_first_probe += probe_steps[axis] * static_cast<float>(counts[axis] * probe_scroll_directions[axis]);
      probe_scroll_offset[axis] = 0;
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

DdgiProbeRayDiagnosticSource CreateDdgiProbeRayDiagnosticSourceFromSettings(const RenderLayer::DdgiSettings& settings) {
  DdgiProbeRayDiagnosticSource source;
  source.probe_counts = ClampDdgiProbeCounts(settings.volume_defaults.probe_counts);
  const auto spacing = ClampDdgiProbeSpacing(settings.volume_defaults.probe_spacing);
  source.first_probe =
      settings.volume_defaults.volume_origin - glm::vec3(source.probe_counts - glm::ivec3(1)) * spacing * 0.5f;
  source.probe_step_x = {spacing.x, 0.0f, 0.0f};
  source.probe_step_y = {0.0f, spacing.y, 0.0f};
  source.probe_step_z = {0.0f, 0.0f, spacing.z};
  source.relocation_distance = glm::max(settings.volume_defaults.relocation_distance, 0.0f);
  source.random_ray_backface_threshold = glm::clamp(settings.volume_defaults.random_ray_backface_threshold, 0.0f, 1.0f);
  source.fixed_ray_backface_threshold = glm::clamp(settings.volume_defaults.fixed_ray_backface_threshold, 0.0f, 1.0f);
  source.probe_variability_threshold = glm::clamp(settings.volume_defaults.probe_variability_threshold, 0.0f, 10.0f);
  source.probe_variability_min_samples = glm::clamp(settings.volume_defaults.probe_variability_min_samples, 0, 4096);
  source.movement_type =
      glm::clamp(settings.volume_defaults.movement_type, static_cast<int>(DdgiVolumeMovementType::Default),
                 static_cast<int>(DdgiVolumeMovementType::Scrolling));
  source.enable_probe_relocation = settings.volume_defaults.enable_probe_relocation;
  source.enable_probe_classification = settings.volume_defaults.enable_probe_classification;
  source.enable_probe_variability = settings.volume_defaults.enable_probe_variability;
  source.enable_probe_variability_gating = settings.volume_defaults.enable_probe_variability_gating;
  return source;
}

DdgiProbeRayDiagnosticSource CreateDdgiProbeRayDiagnosticSourceFromVolume(const DdgiVolume& volume,
                                                                          const glm::mat4& transform) {
  DdgiProbeRayDiagnosticSource source;
  source.probe_counts = ClampDdgiProbeCounts(volume.probe_counts);
  const auto first_probe = volume.GetProbeLocalPosition({0, 0, 0});
  const auto spacing = ClampDdgiProbeSpacing(volume.probe_spacing);
  source.first_probe = TransformPoint(transform, first_probe);
  source.probe_step_x = TransformVector(transform, {spacing.x, 0.0f, 0.0f});
  source.probe_step_y = TransformVector(transform, {0.0f, spacing.y, 0.0f});
  source.probe_step_z = TransformVector(transform, {0.0f, 0.0f, spacing.z});
  source.relocation_distance = glm::max(volume.relocation_distance, 0.0f);
  source.random_ray_backface_threshold = glm::clamp(volume.random_ray_backface_threshold, 0.0f, 1.0f);
  source.fixed_ray_backface_threshold = glm::clamp(volume.fixed_ray_backface_threshold, 0.0f, 1.0f);
  source.probe_variability_threshold = glm::clamp(volume.probe_variability_threshold, 0.0f, 10.0f);
  source.probe_variability_min_samples = glm::clamp(volume.probe_variability_min_samples, 0, 4096);
  source.warmup_trigger_conditions = volume.warmup_trigger_conditions & DdgiVolumeTriggerConditionAll;
  source.variability_reset_trigger_conditions =
      volume.variability_reset_trigger_conditions & DdgiVolumeTriggerConditionAll;
  source.movement_type = glm::clamp(volume.movement_type, static_cast<int>(DdgiVolumeMovementType::Default),
                                    static_cast<int>(DdgiVolumeMovementType::Scrolling));
  source.enable_probe_relocation = volume.enable_probe_relocation;
  source.enable_probe_classification = volume.enable_probe_classification;
  source.enable_probe_variability = volume.enable_probe_variability;
  source.enable_probe_variability_gating = volume.enable_probe_variability_gating;
  return source;
}

std::vector<DdgiVolumeCandidate> CollectDdgiVolumeCandidates(const std::shared_ptr<Scene>& scene,
                                                             const RenderLayer::DdgiSettings& settings) {
  (void)settings;
  std::vector<DdgiVolumeCandidate> candidates;
  if (!scene) {
    return candidates;
  }
  if (const auto* owners = scene->UnsafeGetPrivateComponentOwnersList<DdgiVolume>()) {
    candidates.reserve(owners->size());
    for (const auto& owner : *owners) {
      if (!scene->IsEntityEnabled(owner)) {
        continue;
      }
      const auto volume = scene->GetOrSetPrivateComponent<DdgiVolume>(owner).lock();
      if (volume && volume->IsEnabled()) {
        candidates.push_back({volume, owner, scene->GetDataComponent<GlobalTransform>(owner).value});
      }
    }
  }
  std::stable_sort(candidates.begin(), candidates.end(), [](const auto& lhs, const auto& rhs) {
    return lhs.owner.GetIndex() < rhs.owner.GetIndex();
  });
  return candidates;
}

DdgiProbeRayDiagnosticSource CreateDdgiProbeRayDiagnosticSource(const std::shared_ptr<Scene>& scene,
                                                                const RenderLayer::DdgiSettings& settings) {
  const auto candidates = CollectDdgiVolumeCandidates(scene, settings);
  if (candidates.empty()) {
    return CreateDdgiProbeRayDiagnosticSourceFromSettings(settings);
  }

  constexpr size_t selected_index = 0;
  const auto& candidate = candidates[selected_index];
  auto source = CreateDdgiProbeRayDiagnosticSourceFromVolume(*candidate.volume, candidate.transform);
  source.enabled_volume_count = static_cast<uint32_t>(candidates.size());
  source.selected_volume_index = static_cast<uint32_t>(selected_index);
  return source;
}

float CalculateDdgiEffectiveMaxRayDistance(const RenderLayer::DdgiSettings& settings,
                                           const DdgiProbeRayDiagnosticSource& source) {
  (void)source;
  return glm::max(settings.runtime.max_ray_distance, 0.05f);
}

uint32_t GetDdgiEnvironmentCubemapIndex(const std::shared_ptr<Scene>& scene, const glm::vec3& position) {
  std::shared_ptr<ReflectionProbe> reflection_probe;
  if (scene) {
    reflection_probe = scene->environment.GetReflectionProbe(position);
  }
  if (!reflection_probe) {
    if (const auto default_environment = Resources::GetInstance().GetDefaultEnvironmentalMap()) {
      reflection_probe = default_environment->reflection_probe.Get<ReflectionProbe>();
    }
  }
  if (!reflection_probe) {
    return 0u;
  }

  const auto cubemap = reflection_probe->GetCubemap();
  return cubemap ? cubemap->GetTextureStorageIndex() : 0u;
}

DdgiProbeRayTracingPushConstant CreateDdgiProbeRayTracingPushConstant(
    const RenderLayer::DdgiSettings& settings, const DdgiProbeRayDiagnosticSource& source,
    const RenderLayer::DdgiProbeUpdateWindow& update_window, const bool skip_inactive_probe_trace,
    const bool skip_recursive_ddgi, const uint32_t environment_cubemap_index) {
  DdgiProbeRayTracingPushConstant push_constant;
  const auto frame_index = static_cast<uint32_t>(Platform::GetFrameCount() & 0x00ffffffu);
  const auto ray_rotation = CreateDdgiProbeRayRotationQuaternion(frame_index, source.selected_volume_index);
  push_constant.first_probe = glm::vec4(source.first_probe, ray_rotation.x);
  push_constant.probe_step_x = glm::vec4(source.probe_step_x, ray_rotation.y);
  push_constant.probe_step_y = glm::vec4(source.probe_step_y, ray_rotation.z);
  push_constant.probe_step_z = glm::vec4(source.probe_step_z, ray_rotation.w);
  push_constant.probe_counts_and_ray_count = glm::uvec4(glm::uvec3(ClampDdgiProbeCounts(source.probe_counts)),
                                                        static_cast<uint32_t>(glm::max(settings.runtime.ray_count, 1)));
  push_constant.probe_offset_and_update_count = {update_window.start_probe_index,
                                                 glm::max(update_window.probe_count, 1u),
                                                 skip_inactive_probe_trace ? 1u : 0u, environment_cubemap_index};
  push_constant.trace_parameters = {CalculateDdgiEffectiveMaxRayDistance(settings, source),
                                    glm::max(settings.runtime.normal_bias, 0.001f),
                                    source.enable_probe_relocation || source.enable_probe_classification ? 1.0f : 0.0f,
                                    skip_recursive_ddgi ? 1.0f : 0.0f};
  push_constant.probe_scroll_offset = CreateDdgiProbeScrollPushConstant(source);
  return push_constant;
}

DdgiProbeAtlasUpdatePushConstant CreateDdgiProbeAtlasUpdatePushConstant(
    const RenderLayer::DdgiSettings& settings, const RenderLayer::DdgiProbeUpdateWindow& update_window,
    const uint32_t total_probe_count, const DdgiProbeRayDiagnosticSource& source, const float history_hysteresis,
    const float brightness_threshold) {
  const auto layout = RenderLayer::CalculateDdgiFrameResourceLayout(settings, total_probe_count);
  const auto probe_count = glm::max(1u, glm::min(layout.probe_count, update_window.probe_count));
  const auto history_weight = glm::clamp(history_hysteresis, 0.0f, 1.0f);
  DdgiProbeAtlasUpdatePushConstant push_constant;
  push_constant.probe_count_ray_count_and_tile_sizes = {
      probe_count, static_cast<uint32_t>(glm::max(settings.runtime.ray_count, 1)),
      layout.irradiance_atlas.tile_resolution, layout.visibility_atlas.tile_resolution};
  push_constant.atlas_columns_and_rows = {layout.irradiance_atlas.columns, layout.irradiance_atlas.rows,
                                          layout.visibility_atlas.columns, layout.visibility_atlas.rows};
  push_constant.probe_offset_and_total_count = {update_window.start_probe_index, glm::max(total_probe_count, 1u), 0u,
                                                0u};
  push_constant.probe_counts = glm::uvec4(glm::uvec3(ClampDdgiProbeCounts(source.probe_counts)), 0u);
  push_constant.update_parameters = {CalculateDdgiEffectiveMaxRayDistance(settings, source), history_weight,
                                     glm::max(settings.runtime.visibility_moment_bias, 0.0f),
                                     glm::max(settings.runtime.irradiance_gamma, 1.0f)};
  push_constant.probe_state_parameters = {
      glm::max(source.relocation_distance, 0.0f), source.enable_probe_relocation ? 1.0f : 0.0f,
      source.enable_probe_classification ? 1.0f : 0.0f, glm::clamp(settings.runtime.irradiance_threshold, 0.0f, 1.0f)};
  push_constant.probe_blend_parameters = {glm::clamp(source.random_ray_backface_threshold, 0.0f, 1.0f),
                                          glm::clamp(source.fixed_ray_backface_threshold, 0.0f, 1.0f),
                                          glm::max(settings.runtime.distance_exponent, 0.0f),
                                          glm::max(brightness_threshold, 0.0f)};
  push_constant.probe_scroll_offset = CreateDdgiProbeScrollPushConstant(source);
  push_constant.probe_step_x = glm::vec4(source.probe_step_x, 0.0f);
  push_constant.probe_step_y = glm::vec4(source.probe_step_y, 0.0f);
  push_constant.probe_step_z = glm::vec4(source.probe_step_z, 0.0f);
  return push_constant;
}

DdgiProbeRelocationPushConstant CreateDdgiProbeRelocationPushConstant(
    const RenderLayer::DdgiSettings& settings, const RenderLayer::DdgiProbeUpdateWindow& update_window,
    const uint32_t total_probe_count, const DdgiProbeRayDiagnosticSource& source, const bool reset_offsets) {
  DdgiProbeRelocationPushConstant push_constant;
  push_constant.probe_count_ray_count_and_flags = {reset_offsets ? total_probe_count : update_window.probe_count,
                                                   static_cast<uint32_t>(glm::max(settings.runtime.ray_count, 1)),
                                                   glm::max(total_probe_count, 1u), reset_offsets ? 1u : 0u};
  push_constant.probe_counts = glm::uvec4(glm::uvec3(ClampDdgiProbeCounts(source.probe_counts)), 0u);
  push_constant.relocation_parameters = {glm::max(source.relocation_distance, 0.0f),
                                         glm::clamp(source.fixed_ray_backface_threshold, 0.0f, 1.0f), 0.0f, 0.0f};
  push_constant.probe_scroll_offset = CreateDdgiProbeScrollPushConstant(source);
  push_constant.probe_step_x = glm::vec4(source.probe_step_x, 0.0f);
  push_constant.probe_step_y = glm::vec4(source.probe_step_y, 0.0f);
  push_constant.probe_step_z = glm::vec4(source.probe_step_z, 0.0f);
  return push_constant;
}

DdgiProbeClassificationPushConstant CreateDdgiProbeClassificationPushConstant(
    const RenderLayer::DdgiSettings& settings, const RenderLayer::DdgiProbeUpdateWindow& update_window,
    const uint32_t total_probe_count, const DdgiProbeRayDiagnosticSource& source, const bool reset_classification) {
  DdgiProbeClassificationPushConstant push_constant;
  push_constant.probe_count_ray_count_and_flags = {reset_classification ? total_probe_count : update_window.probe_count,
                                                   static_cast<uint32_t>(glm::max(settings.runtime.ray_count, 1)),
                                                   glm::max(total_probe_count, 1u), reset_classification ? 1u : 0u};
  push_constant.probe_counts = glm::uvec4(glm::uvec3(ClampDdgiProbeCounts(source.probe_counts)), 0u);
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
                                 const std::shared_ptr<Buffer>& probe_state_buffer) {
  if (!lighting_descriptor_set || !IsValidDescriptorImageInfo(irradiance_info) ||
      !IsValidDescriptorImageInfo(visibility_info) || !probe_state_buffer) {
    return false;
  }
  lighting_descriptor_set->UpdateImageDescriptorBinding(kDdgiLightingIrradianceBinding, irradiance_info);
  lighting_descriptor_set->UpdateImageDescriptorBinding(kDdgiLightingVisibilityBinding, visibility_info);
  lighting_descriptor_set->UpdateBufferDescriptorBinding(kDdgiLightingProbeStateBinding, probe_state_buffer);
  return true;
}

void BindDdgiFallbackLightingDescriptors(const std::shared_ptr<DescriptorSet>& lighting_descriptor_set,
                                         const std::shared_ptr<Buffer>& fallback_probe_state_buffer) {
  const auto image_info = CreateDdgiFallbackImageInfo();
  BindDdgiLightingDescriptors(lighting_descriptor_set, image_info, image_info, fallback_probe_state_buffer);
}

bool BindDdgiAtlasLightingDescriptors(const RenderGraphResourceRegistry& registry,
                                      RenderGraphTransientResourceStore& transient_resources,
                                      const std::shared_ptr<DescriptorSet>& lighting_descriptor_set,
                                      const std::shared_ptr<Sampler>& atlas_sampler,
                                      const std::shared_ptr<Buffer>& fallback_probe_state_buffer) {
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
  return BindDdgiLightingDescriptors(lighting_descriptor_set, irradiance_info, visibility_info, probe_state_buffer);
}

void ApplyDdgiRenderInfo(RenderInstanceStorage::RenderInfoBlock& render_info, const RenderLayer::DdgiSettings& settings,
                         const DdgiProbeRayDiagnosticSource& source,
                         const DdgiProbeRayTracingPushConstant& ray_push_constant,
                         const DdgiProbeAtlasUpdatePushConstant& update_push_constant) {
  render_info.ddgi_indirect_intensity = glm::max(settings.runtime.indirect_intensity, 0.0f);
  render_info.ddgi_first_probe = glm::vec4(source.first_probe, 0.0f);
  render_info.ddgi_probe_step_x = glm::vec4(source.probe_step_x, 0.0f);
  render_info.ddgi_probe_step_y = glm::vec4(source.probe_step_y, 0.0f);
  render_info.ddgi_probe_step_z = glm::vec4(source.probe_step_z, 0.0f);
  render_info.ddgi_probe_counts =
      glm::vec4(glm::vec3(ray_push_constant.probe_counts_and_ray_count), update_push_constant.update_parameters.w);
  render_info.ddgi_probe_scroll_offset = glm::vec4(
      static_cast<float>(source.probe_scroll_offset.x), static_cast<float>(source.probe_scroll_offset.y),
      static_cast<float>(source.probe_scroll_offset.z),
      static_cast<float>(PackDdgiProbeScrollFlags(source.probe_scroll_clear, source.probe_scroll_directions)));
  render_info.ddgi_atlas_parameters = glm::vec4(
      update_push_constant.probe_count_ray_count_and_tile_sizes.z, update_push_constant.atlas_columns_and_rows.x,
      update_push_constant.probe_count_ray_count_and_tile_sizes.w, update_push_constant.atlas_columns_and_rows.z);
  render_info.ddgi_volume_parameters =
      glm::vec4(0.0f, update_push_constant.update_parameters.z, ray_push_constant.trace_parameters.y,
                glm::max(settings.runtime.view_bias, 0.0f));
  render_info.ddgi_sampling_parameters = glm::vec4(1.0f, 0.0f, 0.0f, 0.0f);
}

void PreserveDdgiRenderInfo(RenderInstanceStorage::RenderInfoBlock& render_info,
                            const RenderInstanceStorage::RenderInfoBlock& previous_render_info) {
  render_info.ddgi_indirect_intensity = previous_render_info.ddgi_indirect_intensity;
  render_info.ddgi_first_probe = previous_render_info.ddgi_first_probe;
  render_info.ddgi_probe_step_x = previous_render_info.ddgi_probe_step_x;
  render_info.ddgi_probe_step_y = previous_render_info.ddgi_probe_step_y;
  render_info.ddgi_probe_step_z = previous_render_info.ddgi_probe_step_z;
  render_info.ddgi_probe_counts = previous_render_info.ddgi_probe_counts;
  render_info.ddgi_probe_scroll_offset = previous_render_info.ddgi_probe_scroll_offset;
  render_info.ddgi_atlas_parameters = previous_render_info.ddgi_atlas_parameters;
  render_info.ddgi_volume_parameters = previous_render_info.ddgi_volume_parameters;
  render_info.ddgi_sampling_parameters = previous_render_info.ddgi_sampling_parameters;
}

uint64_t MakeDdgiLightKey(const uint32_t type_index, const Entity& owner) {
  return (static_cast<uint64_t>(type_index) << 56u) | (static_cast<uint64_t>(owner.GetVersion()) << 32u) |
         static_cast<uint64_t>(owner.GetIndex());
}

uint64_t MixDdgiSignature(const uint64_t seed, const uint64_t value) {
  return seed ^ (value + 0x9e3779b97f4a7c15ull + (seed << 6u) + (seed >> 2u));
}

uint64_t MixDdgiFloat(const uint64_t seed, const float value) {
  return MixDdgiSignature(seed, static_cast<uint64_t>(glm::floatBitsToUint(value)));
}

uint64_t MixDdgiVec3(uint64_t seed, const glm::vec3& value) {
  seed = MixDdgiFloat(seed, value.x);
  seed = MixDdgiFloat(seed, value.y);
  return MixDdgiFloat(seed, value.z);
}

uint64_t MixDdgiMat4(uint64_t seed, const glm::mat4& value) {
  for (int column = 0; column < 4; ++column) {
    for (int row = 0; row < 4; ++row) {
      seed = MixDdgiFloat(seed, value[column][row]);
    }
  }
  return seed;
}

uint64_t MakeDdgiLightSignatureBase(const std::shared_ptr<Scene>& scene, const uint32_t type_index,
                                    const Entity& owner) {
  return MixDdgiMat4(MakeDdgiLightKey(type_index, owner), scene->GetDataComponent<GlobalTransform>(owner).value);
}

uint64_t MakeDdgiLightSignature(const std::shared_ptr<Scene>& scene, const uint32_t type_index, const Entity& owner,
                                const DirectionalLight& light) {
  auto signature = MakeDdgiLightSignatureBase(scene, type_index, owner);
  signature = MixDdgiSignature(signature, light.cast_shadow ? 1u : 0u);
  signature = MixDdgiVec3(signature, light.diffuse);
  signature = MixDdgiFloat(signature, light.diffuse_brightness);
  signature = MixDdgiFloat(signature, light.bias);
  signature = MixDdgiFloat(signature, light.normal_offset);
  return MixDdgiFloat(signature, light.light_size);
}

uint64_t MakeDdgiLightSignature(const std::shared_ptr<Scene>& scene, const uint32_t type_index, const Entity& owner,
                                const PointLight& light) {
  auto signature = MakeDdgiLightSignatureBase(scene, type_index, owner);
  signature = MixDdgiSignature(signature, light.cast_shadow ? 1u : 0u);
  signature = MixDdgiFloat(signature, light.constant);
  signature = MixDdgiFloat(signature, light.linear);
  signature = MixDdgiFloat(signature, light.quadratic);
  signature = MixDdgiFloat(signature, light.bias);
  signature = MixDdgiVec3(signature, light.diffuse);
  signature = MixDdgiFloat(signature, light.diffuse_brightness);
  signature = MixDdgiFloat(signature, light.light_size);
  return MixDdgiFloat(signature, light.shadow_distance);
}

uint64_t MakeDdgiLightSignature(const std::shared_ptr<Scene>& scene, const uint32_t type_index, const Entity& owner,
                                const SpotLight& light) {
  auto signature = MakeDdgiLightSignatureBase(scene, type_index, owner);
  signature = MixDdgiSignature(signature, light.cast_shadow ? 1u : 0u);
  signature = MixDdgiFloat(signature, light.inner_degrees);
  signature = MixDdgiFloat(signature, light.outer_degrees);
  signature = MixDdgiFloat(signature, light.constant);
  signature = MixDdgiFloat(signature, light.linear);
  signature = MixDdgiFloat(signature, light.quadratic);
  signature = MixDdgiFloat(signature, light.bias);
  signature = MixDdgiVec3(signature, light.diffuse);
  signature = MixDdgiFloat(signature, light.diffuse_brightness);
  signature = MixDdgiFloat(signature, light.light_size);
  return MixDdgiFloat(signature, light.shadow_distance);
}

template <typename LightComponent>
void CollectDdgiActiveLightKeys(const std::shared_ptr<Scene>& scene, const uint32_t type_index,
                                std::vector<uint64_t>& keys) {
  if (const auto* owners = scene->UnsafeGetPrivateComponentOwnersList<LightComponent>()) {
    for (const auto& owner : *owners) {
      if (!scene->IsEntityEnabled(owner)) {
        continue;
      }
      const auto light = scene->GetOrSetPrivateComponent<LightComponent>(owner).lock();
      if (light && light->IsEnabled()) {
        keys.push_back(MakeDdgiLightKey(type_index, owner));
      }
    }
  }
}

std::vector<uint64_t> CollectDdgiActiveLightKeys(const std::shared_ptr<Scene>& scene) {
  std::vector<uint64_t> keys;
  if (!scene) {
    return keys;
  }
  CollectDdgiActiveLightKeys<DirectionalLight>(scene, 1u, keys);
  CollectDdgiActiveLightKeys<PointLight>(scene, 2u, keys);
  CollectDdgiActiveLightKeys<SpotLight>(scene, 3u, keys);
  std::sort(keys.begin(), keys.end());
  return keys;
}

template <typename LightComponent>
void CollectDdgiLightSignatures(const std::shared_ptr<Scene>& scene, const uint32_t type_index,
                                std::vector<uint64_t>& signatures) {
  if (const auto* owners = scene->UnsafeGetPrivateComponentOwnersList<LightComponent>()) {
    for (const auto& owner : *owners) {
      if (!scene->IsEntityEnabled(owner)) {
        continue;
      }
      const auto light = scene->GetOrSetPrivateComponent<LightComponent>(owner).lock();
      if (light && light->IsEnabled()) {
        signatures.push_back(MakeDdgiLightSignature(scene, type_index, owner, *light));
      }
    }
  }
}

std::vector<uint64_t> CollectDdgiLightSignatures(const std::shared_ptr<Scene>& scene) {
  std::vector<uint64_t> signatures;
  if (!scene) {
    return signatures;
  }
  CollectDdgiLightSignatures<DirectionalLight>(scene, 1u, signatures);
  CollectDdgiLightSignatures<PointLight>(scene, 2u, signatures);
  CollectDdgiLightSignatures<SpotLight>(scene, 3u, signatures);
  std::sort(signatures.begin(), signatures.end());
  return signatures;
}

uint64_t MakeDdgiGeometrySignature(const std::shared_ptr<RenderInstanceStorage::IRenderInstance>& render_instance) {
  if (!render_instance) {
    return 0;
  }
  auto signature = static_cast<uint64_t>(render_instance->command_type);
  signature = MixDdgiSignature(signature, render_instance->owner.GetIndex());
  signature = MixDdgiSignature(signature, render_instance->owner.GetVersion());
  signature = MixDdgiSignature(signature, static_cast<uint64_t>(render_instance->entity_handle));
  signature = MixDdgiSignature(signature, static_cast<uint64_t>(render_instance->material_index));
  signature = MixDdgiSignature(signature, render_instance->cast_shadow ? 1u : 0u);
  signature = MixDdgiSignature(signature, render_instance->material_version);
  signature = MixDdgiSignature(signature, render_instance->geometry_version);
  signature = MixDdgiSignature(signature, render_instance->cull_mode);
  signature = MixDdgiSignature(signature, render_instance->polygon_mode);
  if (const auto instanced =
          std::dynamic_pointer_cast<RenderInstanceStorage::InstancedRenderInstance>(render_instance)) {
    signature = MixDdgiSignature(signature, instanced->particle_info_list_version);
    if (instanced->particle_infos) {
      const auto& particle_infos = instanced->particle_infos->PeekParticleInfoList();
      signature = MixDdgiSignature(signature, particle_infos.size());
      for (const auto& particle_info : particle_infos) {
        signature = MixDdgiMat4(signature, particle_info.instance_matrix.value);
        signature = MixDdgiFloat(signature, particle_info.instance_color.x);
        signature = MixDdgiFloat(signature, particle_info.instance_color.y);
        signature = MixDdgiFloat(signature, particle_info.instance_color.z);
        signature = MixDdgiFloat(signature, particle_info.instance_color.w);
      }
    }
  }
  if (const auto skinned =
          std::dynamic_pointer_cast<RenderInstanceStorage::SkinnedMeshRenderInstance>(render_instance)) {
    signature = MixDdgiSignature(signature, skinned->bone_matrices_snapshot.size());
    for (const auto& bone_matrix : skinned->bone_matrices_snapshot) {
      signature = MixDdgiMat4(signature, bone_matrix);
    }
  }
  return MixDdgiMat4(signature, render_instance->model.value);
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

std::shared_ptr<Buffer> CreateDdgiFallbackProbeStateBuffer() {
  auto buffer = CreateDdgiProbeStateBuffer(sizeof(glm::vec4));
  if (buffer) {
    const std::vector<glm::vec4> zero_state(1, glm::vec4(0.0f));
    buffer->UploadVector(zero_state);
  }
  return buffer;
}

void AddDdgiFrameResources(RenderGraph& graph, const RenderLayer::DdgiFrameResourceLayout& layout) {
  graph.AddResource(CreateDdgiImportedBufferResourceDescriptor(RenderResourceNames::frame_ddgi_probe_metadata,
                                                               layout.probe_metadata_byte_size));
  graph.AddResource(CreateDdgiImportedBufferResourceDescriptor(RenderResourceNames::frame_ddgi_probe_state,
                                                               layout.probe_state_byte_size));
  graph.AddResource(CreateDdgiImportedBufferResourceDescriptor(RenderResourceNames::frame_ddgi_probe_update_indices,
                                                               layout.probe_update_index_byte_size));
  graph.AddResource(
      CreateDdgiBufferResourceDescriptor(RenderResourceNames::frame_ddgi_ray_output, layout.ray_output_byte_size));
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
  graph.AddResource(CreateDdgiImportedImageResourceDescriptor(RenderResourceNames::frame_ddgi_visibility_atlas,
                                                              layout.visibility_atlas, "RG16F"));
}

void AddDdgiProbeRayVisualizationFrameResources(RenderGraph& graph,
                                                const RenderLayer::DdgiFrameResourceLayout& layout) {
  graph.AddResource(CreateDdgiImportedBufferResourceDescriptor(RenderResourceNames::frame_ddgi_ray_output,
                                                               layout.ray_output_byte_size));
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
  pipeline->raygen_shader =
      Shader::CreateTemporary(ShaderType::RayGen, shader_header,
                              Resources::GetDefaultResourcesPath() / "Shaders/RayTracing/RayGen/Camera.rgen");
  pipeline->miss_shader =
      shared_miss_shader
          ? shared_miss_shader
          : Shader::CreateTemporary(ShaderType::Miss, Platform::GetShaderGlobalDefines(),
                                    Resources::GetDefaultResourcesPath() / "Shaders/RayTracing/Miss/Camera.rmiss");
  pipeline->closest_hit_shader =
      shared_closest_hit_shader ? shared_closest_hit_shader
                                : Shader::CreateTemporary(ShaderType::ClosestHit, Platform::GetShaderGlobalDefines(),
                                                          Resources::GetDefaultResourcesPath() /
                                                              "Shaders/RayTracing/ClosestHit/Camera.rchit");
  pipeline->any_hit_shader =
      Shader::CreateTemporary(ShaderType::AnyHit, shader_header,
                              Resources::GetDefaultResourcesPath() / "Shaders/RayTracing/AnyHit/Camera.rahit");
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
  pipeline->compute_shader = Shader::CreateTemporary(
      ShaderType::Compute, shader_header, Resources::GetDefaultResourcesPath() / "Shaders/Compute/RayQueryCamera.comp");
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
  const auto counts = glm::ivec3(glm::clamp(probe_counts.x, 1, 256), glm::clamp(probe_counts.y, 1, 256),
                                 glm::clamp(probe_counts.z, 1, 256));
  return static_cast<uint32_t>(counts.x) * static_cast<uint32_t>(counts.y) * static_cast<uint32_t>(counts.z);
}

uint32_t RenderLayer::GetDdgiAllocatedProbeCount(const DdgiSettings& settings) {
  return GetDdgiAllocatedProbeCount(settings, GetDdgiProbeCount(settings.volume_defaults.probe_counts));
}

uint32_t RenderLayer::GetDdgiAllocatedProbeCount(const DdgiSettings& settings, const uint32_t probe_count) {
  const auto max_probe_count = static_cast<uint32_t>(glm::max(settings.storage.max_probe_count, 1));
  return glm::min(glm::max(probe_count, 1u), max_probe_count);
}

glm::uvec3 RenderLayer::GetDdgiProbeGridIndex(const glm::ivec3& probe_counts, const uint32_t probe_index) {
  const auto counts = glm::ivec3(glm::clamp(probe_counts.x, 1, 256), glm::clamp(probe_counts.y, 1, 256),
                                 glm::clamp(probe_counts.z, 1, 256));
  const auto x_count = static_cast<uint32_t>(counts.x);
  const auto y_count = static_cast<uint32_t>(counts.y);
  const auto safe_index = glm::min(probe_index, GetDdgiProbeCount(counts) - 1u);
  return {safe_index % x_count, (safe_index / x_count) % y_count, safe_index / (x_count * y_count)};
}

RenderLayer::DdgiAtlasLayout RenderLayer::CalculateDdgiAtlasLayout(const uint32_t probe_count,
                                                                   const uint32_t tile_resolution,
                                                                   const uint32_t preferred_columns) {
  DdgiAtlasLayout layout;
  layout.probe_count = glm::max(1u, probe_count);
  layout.tile_resolution = glm::max(1u, tile_resolution);
  layout.columns = glm::min(layout.probe_count, glm::max(1u, preferred_columns));
  layout.rows = (layout.probe_count + layout.columns - 1u) / layout.columns;
  const auto tile_stride = layout.tile_resolution + 2u;
  layout.resolution = {layout.columns * tile_stride, layout.rows * tile_stride};
  return layout;
}

RenderLayer::DdgiProbeDebugCoordinates RenderLayer::CalculateDdgiProbeDebugCoordinates(const DdgiSettings& settings,
                                                                                       const uint32_t tile_resolution) {
  DdgiProbeDebugCoordinates coordinates;
  const auto probe_count = GetDdgiAllocatedProbeCount(settings);
  coordinates.probe_index =
      glm::min(static_cast<uint32_t>(glm::max(settings.debug.selected_probe_index, 0)), probe_count - 1u);
  coordinates.grid_index = GetDdgiProbeGridIndex(settings.volume_defaults.probe_counts, coordinates.probe_index);
  coordinates.atlas_layout =
      CalculateDdgiAtlasLayout(probe_count, tile_resolution, glm::max(settings.storage.atlas_probe_columns, 1));
  const auto tile_stride = coordinates.atlas_layout.tile_resolution + 2u;
  coordinates.atlas_tile_offset = {(coordinates.probe_index % coordinates.atlas_layout.columns) * tile_stride,
                                   (coordinates.probe_index / coordinates.atlas_layout.columns) * tile_stride};
  return coordinates;
}

RenderLayer::DdgiFrameResourceLayout RenderLayer::CalculateDdgiFrameResourceLayout(const DdgiSettings& settings) {
  return CalculateDdgiFrameResourceLayout(settings, GetDdgiProbeCount(settings.volume_defaults.probe_counts));
}

RenderLayer::DdgiFrameResourceLayout RenderLayer::CalculateDdgiFrameResourceLayout(const DdgiSettings& settings,
                                                                                   const uint32_t probe_count) {
  DdgiFrameResourceLayout layout;
  layout.probe_count = GetDdgiAllocatedProbeCount(settings, probe_count);
  layout.irradiance_atlas = CalculateDdgiAtlasLayout(
      layout.probe_count, static_cast<uint32_t>(glm::max(settings.storage.irradiance_tile_resolution, 1)),
      static_cast<uint32_t>(glm::max(settings.storage.atlas_probe_columns, 1)));
  layout.visibility_atlas = CalculateDdgiAtlasLayout(
      layout.probe_count, static_cast<uint32_t>(glm::max(settings.storage.visibility_tile_resolution, 1)),
      static_cast<uint32_t>(glm::max(settings.storage.atlas_probe_columns, 1)));
  layout.variability_atlas = layout.irradiance_atlas;
  layout.variability_atlas.resolution = {layout.variability_atlas.columns * layout.variability_atlas.tile_resolution,
                                         layout.variability_atlas.rows * layout.variability_atlas.tile_resolution};
  layout.variability_reduction_extent = {glm::max(1u, (layout.variability_atlas.resolution.x + 15u) / 16u),
                                         glm::max(1u, (layout.variability_atlas.resolution.y + 15u) / 16u)};
  layout.probe_metadata_byte_size = static_cast<uint64_t>(layout.probe_count) * sizeof(glm::vec4) * 3ull;
  layout.probe_state_byte_size = static_cast<uint64_t>(layout.probe_count) * sizeof(glm::vec4);
  layout.probe_update_index_byte_size = static_cast<uint64_t>(layout.probe_count) * sizeof(uint32_t);
  layout.ray_output_byte_size = static_cast<uint64_t>(layout.probe_count) *
                                static_cast<uint64_t>(glm::max(settings.runtime.ray_count, 1)) *
                                sizeof(PointCloudSample);
  layout.variability_atlas_byte_size = static_cast<uint64_t>(layout.variability_atlas.resolution.x) *
                                       static_cast<uint64_t>(layout.variability_atlas.resolution.y) * sizeof(uint16_t);
  layout.variability_reduction_byte_size = static_cast<uint64_t>(layout.variability_reduction_extent.x) *
                                           static_cast<uint64_t>(layout.variability_reduction_extent.y) *
                                           sizeof(glm::vec2);
  layout.variability_readback_byte_size = sizeof(glm::vec2);
  return layout;
}

float RenderLayer::CalculateDdgiUpdateHysteresis(const DdgiSettings& settings, const uint32_t update_reasons) {
  return CalculateDdgiUpdateHysteresis(settings, update_reasons, (std::numeric_limits<uint32_t>::max)());
}

float RenderLayer::CalculateDdgiUpdateHysteresis(const DdgiSettings& settings, const uint32_t update_reasons,
                                                 const uint32_t warmup_frame_index) {
  if ((update_reasons & (DdgiUpdateReasonSource | DdgiUpdateReasonManualReset | DdgiUpdateReasonSceneInput)) != 0u) {
    return 0.0f;
  }
  const auto hysteresis = glm::clamp(settings.runtime.hysteresis, 0.0f, 1.0f);
  const auto warmup_frame_count = static_cast<uint32_t>(glm::max(settings.runtime.warmup_frames, 0));
  if ((update_reasons & DdgiUpdateReasonWarmup) == 0u || warmup_frame_count == 0u ||
      warmup_frame_index >= warmup_frame_count) {
    return hysteresis;
  }
  const auto denominator = static_cast<float>(glm::max(warmup_frame_count - 1u, 1u));
  return hysteresis * (static_cast<float>(warmup_frame_index) / denominator);
}

float RenderLayer::CalculateDdgiUpdateBrightnessThreshold(const DdgiSettings& settings, const uint32_t update_reasons) {
  (void)update_reasons;
  const auto brightness_threshold = glm::clamp(settings.runtime.brightness_threshold, 0.0f, 1.0f);
  return brightness_threshold;
}

std::string RenderLayer::FormatDdgiUpdateReasons(const uint32_t reasons) {
  if (reasons == DdgiUpdateReasonNone) {
    return "None";
  }
  std::string result;
  const auto append_reason = [&](const uint32_t reason, const char* label) {
    if ((reasons & reason) == 0u) {
      return;
    }
    if (!result.empty()) {
      result += ", ";
    }
    result += label;
  };
  append_reason(DdgiUpdateReasonSource, "DDGI source");
  append_reason(DdgiUpdateReasonManualReset, "Manual reset");
  append_reason(DdgiUpdateReasonSteadyState, "Steady state");
  append_reason(DdgiUpdateReasonConverged, "Converged");
  append_reason(DdgiUpdateReasonWarmup, "Warm up");
  append_reason(DdgiUpdateReasonSceneInput, "Scene input");
  return result.empty() ? "Unknown" : result;
}

float RenderLayer::CalculateDdgiVolumeBlendWeight(const glm::vec3& probe_coordinate, const glm::ivec3& probe_counts,
                                                  const glm::vec3& probe_step_lengths) {
  const auto counts = ClampDdgiProbeCounts(probe_counts);
  const auto max_probe_coordinate = glm::vec3(counts - glm::ivec3(1));
  const auto inside_volume = !glm::any(glm::lessThan(probe_coordinate, glm::vec3(0.0f))) &&
                             !glm::any(glm::greaterThan(probe_coordinate, max_probe_coordinate));
  if (inside_volume) {
    return 1.0f;
  }

  const auto step_lengths = glm::max(probe_step_lengths, glm::vec3(0.0001f));
  const auto lower_distance = glm::max(-probe_coordinate * step_lengths, glm::vec3(0.0f));
  const auto upper_distance = glm::max((probe_coordinate - max_probe_coordinate) * step_lengths, glm::vec3(0.0f));
  const auto outside_distance = glm::max(lower_distance, upper_distance);
  const auto axis_weight =
      glm::vec3(1.0f) - glm::clamp(outside_distance / step_lengths, glm::vec3(0.0f), glm::vec3(1.0f));
  return axis_weight.x * axis_weight.y * axis_weight.z;
}

std::vector<RenderLayer::DdgiVolumeRuntimeInfo> RenderLayer::CollectDdgiVolumeRuntimeInfos(
    const std::shared_ptr<Scene>& scene, const DdgiSettings& settings) {
  const auto candidates = CollectDdgiVolumeCandidates(scene, settings);
  std::vector<DdgiVolumeRuntimeInfo> infos;
  infos.reserve(candidates.size());
  for (size_t i = 0; i < candidates.size(); ++i) {
    auto& info = infos.emplace_back();
    info.sorted_index = static_cast<uint32_t>(i);
    info.owner_index = candidates[i].owner.GetIndex();
    info.probe_counts = ClampDdgiProbeCounts(candidates[i].volume->probe_counts);
    info.probe_count = candidates[i].volume->GetProbeAmount();
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
    raster_lighting_texture_layout_ = std::make_shared<DescriptorSetLayout>();
    for (uint32_t binding = 0; binding < 4; binding++) {
      raster_lighting_texture_layout_->PushDescriptorBinding(binding, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER,
                                                             VK_SHADER_STAGE_FRAGMENT_BIT, 0);
    }
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
  if (!lighting_layout_) {
    lighting_layout_ = std::make_shared<DescriptorSetLayout>();
    lighting_layout_->PushDescriptorBinding(14, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, VK_SHADER_STAGE_FRAGMENT_BIT,
                                            0);
    lighting_layout_->PushDescriptorBinding(15, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, VK_SHADER_STAGE_FRAGMENT_BIT,
                                            0);
    lighting_layout_->PushDescriptorBinding(16, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, VK_SHADER_STAGE_FRAGMENT_BIT,
                                            0);
    lighting_layout_->PushDescriptorBinding(17, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, VK_SHADER_STAGE_FRAGMENT_BIT,
                                            0);
    lighting_layout_->PushDescriptorBinding(18, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, VK_SHADER_STAGE_FRAGMENT_BIT,
                                            0);
    lighting_layout_->PushDescriptorBinding(19, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_FRAGMENT_BIT, 0);
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
    ddgi_probe_update_layout_->PushDescriptorBinding(5, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_COMPUTE_BIT,
                                                     0);
    ddgi_probe_update_layout_->PushDescriptorBinding(6, VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, VK_SHADER_STAGE_COMPUTE_BIT,
                                                     0);
    ddgi_probe_update_layout_->Initialize();
  }
  if (!ddgi_probe_relocation_layout_) {
    ddgi_probe_relocation_layout_ = std::make_shared<DescriptorSetLayout>();
    ddgi_probe_relocation_layout_->PushDescriptorBinding(0, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,
                                                         VK_SHADER_STAGE_COMPUTE_BIT, 0);
    ddgi_probe_relocation_layout_->PushDescriptorBinding(1, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,
                                                         VK_SHADER_STAGE_COMPUTE_BIT, 0);
    ddgi_probe_relocation_layout_->PushDescriptorBinding(2, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,
                                                         VK_SHADER_STAGE_COMPUTE_BIT, 0);
    ddgi_probe_relocation_layout_->Initialize();
  }
  if (!ddgi_probe_classification_layout_) {
    ddgi_probe_classification_layout_ = std::make_shared<DescriptorSetLayout>();
    ddgi_probe_classification_layout_->PushDescriptorBinding(0, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,
                                                             VK_SHADER_STAGE_COMPUTE_BIT, 0);
    ddgi_probe_classification_layout_->PushDescriptorBinding(1, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,
                                                             VK_SHADER_STAGE_COMPUTE_BIT, 0);
    ddgi_probe_classification_layout_->PushDescriptorBinding(2, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,
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
    ddgi_probe_visualization_layout_->PushDescriptorBinding(
        0, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT, 0);
    ddgi_probe_visualization_layout_->PushDescriptorBinding(
        1, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT, 0);
    ddgi_probe_visualization_layout_->PushDescriptorBinding(17, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER,
                                                            VK_SHADER_STAGE_FRAGMENT_BIT, 0);
    ddgi_probe_visualization_layout_->PushDescriptorBinding(18, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER,
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
  enable_inspection = false;
  if (!ddgi_atlas_sampler_) {
    ddgi_atlas_sampler_ = CreateDdgiAtlasSampler();
  }
  if (!depth_pyramid_pipeline_) {
    depth_pyramid_pipeline_ = std::make_shared<ComputePipeline>();
    depth_pyramid_pipeline_->compute_shader =
        Shader::CreateTemporary(ShaderType::Compute, Platform::GetShaderGlobalDefines(),
                                Resources::GetDefaultResourcesPath() / "Shaders/Compute/DepthPyramid.comp");
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
                                Resources::GetDefaultResourcesPath() / "Shaders/Compute/MotionVectors.comp");
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
                                Resources::GetDefaultResourcesPath() / "Shaders/Compute/VolumetricClouds.comp");
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
        Resources::GetDefaultResourcesPath() / "Shaders/Compute/VolumetricCloudsComposite.comp");
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
                                Resources::GetDefaultResourcesPath() / "Shaders/Compute/GaussianSplatCull.comp");
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
                                       gaussian_splat_compute_path / "GaussianSplatRadixUpsweep.comp");
  create_gaussian_splat_radix_pipeline(gaussian_splat_radix_spine_pipeline_,
                                       gaussian_splat_compute_path / "GaussianSplatRadixSpine.comp");
  create_gaussian_splat_radix_pipeline(gaussian_splat_radix_downsweep_pipeline_,
                                       gaussian_splat_compute_path / "GaussianSplatRadixDownsweep.comp");
  if (!ddgi_probe_update_pipeline_) {
    ddgi_probe_update_pipeline_ = std::make_shared<ComputePipeline>();
    ddgi_probe_update_pipeline_->compute_shader =
        Shader::CreateTemporary(ShaderType::Compute, Platform::GetShaderGlobalDefines(),
                                Resources::GetDefaultResourcesPath() / "Shaders/Compute/DDGIProbeUpdate.comp");
    ddgi_probe_update_pipeline_->descriptor_set_layouts.emplace_back(per_frame_layout_);
    ddgi_probe_update_pipeline_->descriptor_set_layouts.emplace_back(ddgi_probe_update_layout_);
    auto& push_constant_range = ddgi_probe_update_pipeline_->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(DdgiProbeAtlasUpdatePushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
    ddgi_probe_update_pipeline_->Initialize();
  }
  if (!ddgi_probe_relocation_pipeline_) {
    ddgi_probe_relocation_pipeline_ = std::make_shared<ComputePipeline>();
    ddgi_probe_relocation_pipeline_->compute_shader =
        Shader::CreateTemporary(ShaderType::Compute, Platform::GetShaderGlobalDefines(),
                                Resources::GetDefaultResourcesPath() / "Shaders/Compute/DDGIProbeRelocation.comp");
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
                                Resources::GetDefaultResourcesPath() / "Shaders/Compute/DDGIProbeClassification.comp");
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
        Resources::GetDefaultResourcesPath() / "Shaders/Compute/DDGIProbeVariabilityReduce.comp");
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
        Resources::GetDefaultResourcesPath() / "Shaders/Compute/DDGIProbeVariabilityExtraReduce.comp");
    ddgi_probe_variability_extra_reduce_pipeline_->descriptor_set_layouts.emplace_back(ddgi_probe_variability_layout_);
    auto& push_constant_range = ddgi_probe_variability_extra_reduce_pipeline_->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(DdgiProbeVariabilityPushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
    ddgi_probe_variability_extra_reduce_pipeline_->Initialize();
  }
#pragma region Graphics Pipelines
  const auto shadow_empty_fragment_shader_path =
      Resources::GetDefaultResourcesPath() / "Shaders/Graphics/Fragment/Empty.frag";
  if (!point_light_shadow_pipeline_normal_opaque) {
    point_light_shadow_pipeline_normal_opaque = CreateShadowVertexPipeline(
        Resources::GetDefaultResourcesPath() / "Shaders/Graphics/Vertex/Lighting/PointLightShadowMap.vert",
        shadow_empty_fragment_shader_path, GeometryType::Mesh, {per_frame_layout_});
  }
  if (Platform::GetInstance().GetCapabilities().support_mesh_shader && !point_light_shadow_pipeline_mesh_shader) {
    point_light_shadow_pipeline_mesh_shader = std::make_shared<GraphicsPipeline>();
    point_light_shadow_pipeline_mesh_shader->task_shader = Shader::CreateTemporary(
        ShaderType::Task, Platform::GetShaderGlobalDefines(),
        Resources::GetDefaultResourcesPath() / "Shaders/Graphics/Task/Lighting/PointLightShadowMap.task");
    point_light_shadow_pipeline_mesh_shader->mesh_shader = Shader::CreateTemporary(
        ShaderType::Mesh, Platform::GetShaderGlobalDefines(),
        Resources::GetDefaultResourcesPath() / "Shaders/Graphics/Mesh/Lighting/PointLightShadowMap.mesh");
    point_light_shadow_pipeline_mesh_shader->fragment_shader =
        Shader::CreateTemporary(ShaderType::Fragment, Platform::GetShaderGlobalDefines(),
                                Resources::GetDefaultResourcesPath() / "Shaders/Graphics/Fragment/Empty.frag");
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
        Resources::GetDefaultResourcesPath() / "Shaders/Graphics/Vertex/Lighting/SpotLightShadowMap.vert",
        shadow_empty_fragment_shader_path, GeometryType::Mesh, {per_frame_layout_});
  }
  if (Platform::GetInstance().GetCapabilities().support_mesh_shader && !spot_light_shadow_pipeline_mesh_shader) {
    spot_light_shadow_pipeline_mesh_shader = std::make_shared<GraphicsPipeline>();
    spot_light_shadow_pipeline_mesh_shader->task_shader = Shader::CreateTemporary(
        ShaderType::Task, Platform::GetShaderGlobalDefines(),
        Resources::GetDefaultResourcesPath() / "Shaders/Graphics/Task/Lighting/SpotLightShadowMap.task");
    spot_light_shadow_pipeline_mesh_shader->mesh_shader = Shader::CreateTemporary(
        ShaderType::Mesh, Platform::GetShaderGlobalDefines(),
        Resources::GetDefaultResourcesPath() / "Shaders/Graphics/Mesh/Lighting/SpotLightShadowMap.mesh");
    spot_light_shadow_pipeline_mesh_shader->fragment_shader =
        Shader::CreateTemporary(ShaderType::Fragment, Platform::GetShaderGlobalDefines(),
                                Resources::GetDefaultResourcesPath() / "Shaders/Graphics/Fragment/Empty.frag");
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
        Resources::GetDefaultResourcesPath() / "Shaders/Graphics/Vertex/Lighting/DirectionalLightShadowMap.vert",
        shadow_empty_fragment_shader_path, GeometryType::Mesh, {per_frame_layout_});
  }
  if (Platform::GetInstance().GetCapabilities().support_mesh_shader && !directional_light_shadow_pipeline_mesh_shader) {
    directional_light_shadow_pipeline_mesh_shader = std::make_shared<GraphicsPipeline>();
    directional_light_shadow_pipeline_mesh_shader->task_shader = Shader::CreateTemporary(
        ShaderType::Task, Platform::GetShaderGlobalDefines(),
        Resources::GetDefaultResourcesPath() / "Shaders/Graphics/Task/Lighting/DirectionalLightShadowMap.task");
    directional_light_shadow_pipeline_mesh_shader->mesh_shader = Shader::CreateTemporary(
        ShaderType::Mesh, Platform::GetShaderGlobalDefines(),
        Resources::GetDefaultResourcesPath() / "Shaders/Graphics/Mesh/Lighting/DirectionalLightShadowMap.mesh");
    directional_light_shadow_pipeline_mesh_shader->fragment_shader =
        Shader::CreateTemporary(ShaderType::Fragment, Platform::GetShaderGlobalDefines(),
                                Resources::GetDefaultResourcesPath() / "Shaders/Graphics/Fragment/Empty.frag");
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
  if (!instanced_point_light_shadow_pipeline_opaque) {
    instanced_point_light_shadow_pipeline_opaque = CreateShadowVertexPipeline(
        Resources::GetDefaultResourcesPath() / "Shaders/Graphics/Vertex/Lighting/PointLightShadowMapInstanced.vert",
        shadow_empty_fragment_shader_path, GeometryType::Mesh, {per_frame_layout_, particle_instanced_data_layout_});
  }
  if (!instanced_spot_light_shadow_pipeline_opaque) {
    instanced_spot_light_shadow_pipeline_opaque = CreateShadowVertexPipeline(
        Resources::GetDefaultResourcesPath() / "Shaders/Graphics/Vertex/Lighting/SpotLightShadowMapInstanced.vert",
        shadow_empty_fragment_shader_path, GeometryType::Mesh, {per_frame_layout_, particle_instanced_data_layout_});
  }
  if (!instanced_directional_light_shadow_pipeline_opaque) {
    instanced_directional_light_shadow_pipeline_opaque = CreateShadowVertexPipeline(
        Resources::GetDefaultResourcesPath() /
            "Shaders/Graphics/Vertex/Lighting/DirectionalLightShadowMapInstanced.vert",
        shadow_empty_fragment_shader_path, GeometryType::Mesh, {per_frame_layout_, particle_instanced_data_layout_});
  }
  if (!skinned_point_light_shadow_pipeline_opaque) {
    skinned_point_light_shadow_pipeline_opaque = CreateShadowVertexPipeline(
        Resources::GetDefaultResourcesPath() / "Shaders/Graphics/Vertex/Lighting/PointLightShadowMapSkinned.vert",
        shadow_empty_fragment_shader_path, GeometryType::SkinnedMesh, {per_frame_layout_, bone_matrices_layout_});
  }
  if (!skinned_spot_light_shadow_pipeline_opaque) {
    skinned_spot_light_shadow_pipeline_opaque = CreateShadowVertexPipeline(
        Resources::GetDefaultResourcesPath() / "Shaders/Graphics/Vertex/Lighting/SpotLightShadowMapSkinned.vert",
        shadow_empty_fragment_shader_path, GeometryType::SkinnedMesh, {per_frame_layout_, bone_matrices_layout_});
  }
  if (!skinned_directional_light_shadow_pipeline_opaque) {
    skinned_directional_light_shadow_pipeline_opaque = CreateShadowVertexPipeline(
        Resources::GetDefaultResourcesPath() / "Shaders/Graphics/Vertex/Lighting/DirectionalLightShadowMapSkinned.vert",
        shadow_empty_fragment_shader_path, GeometryType::SkinnedMesh, {per_frame_layout_, bone_matrices_layout_});
  }
#ifdef EVOENGINE_WINDOWS
  if (!strands_point_light_shadow_pipeline) {
    strands_point_light_shadow_pipeline = std::make_shared<GraphicsPipeline>();
    strands_point_light_shadow_pipeline->vertex_shader = Shader::CreateTemporary(
        ShaderType::Vertex, Platform::GetShaderGlobalDefines(),
        Resources::GetDefaultResourcesPath() / "Shaders/Graphics/Vertex/Lighting/PointLightShadowMapStrands.vert");
    strands_point_light_shadow_pipeline->tessellation_control_shader = Shader::CreateTemporary(
        ShaderType::TessellationControl, Platform::GetShaderGlobalDefines(),
        Resources::GetDefaultResourcesPath() / "Shaders/Graphics/TessellationControl/Lighting/ShadowMapStrands.tesc");
    strands_point_light_shadow_pipeline->tessellation_evaluation_shader =
        Shader::CreateTemporary(ShaderType::TessellationEvaluation, Platform::GetShaderGlobalDefines(),
                                Resources::GetDefaultResourcesPath() /
                                    "Shaders/Graphics/TessellationEvaluation/Lighting/ShadowMapStrands.tese");
    strands_point_light_shadow_pipeline->geometry_shader = Shader::CreateTemporary(
        ShaderType::Geometry, Platform::GetShaderGlobalDefines(),
        Resources::GetDefaultResourcesPath() / "Shaders/Graphics/Geometry/Lighting/PointLightShadowMapStrands.geom");
    strands_point_light_shadow_pipeline->fragment_shader =
        Shader::CreateTemporary(ShaderType::Fragment, Platform::GetShaderGlobalDefines(),
                                Resources::GetDefaultResourcesPath() / "Shaders/Graphics/Fragment/Empty.frag");
    strands_point_light_shadow_pipeline->geometry_type = GeometryType::Strands;
    strands_point_light_shadow_pipeline->descriptor_set_layouts.emplace_back(per_frame_layout_);
    strands_point_light_shadow_pipeline->descriptor_set_layouts.emplace_back(particle_instanced_data_layout_);
    strands_point_light_shadow_pipeline->depth_attachment_format = Platform::Constants::shadow_map;
    strands_point_light_shadow_pipeline->stencil_attachment_format = VK_FORMAT_UNDEFINED;
    strands_point_light_shadow_pipeline->tessellation_patch_control_points = 4;
    auto& push_constant_range = strands_point_light_shadow_pipeline->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(RenderInstancePushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_ALL;
    strands_point_light_shadow_pipeline->Initialize();
  }
  if (!strands_spot_light_shadow_pipeline) {
    strands_spot_light_shadow_pipeline = std::make_shared<GraphicsPipeline>();
    strands_spot_light_shadow_pipeline->vertex_shader = Shader::CreateTemporary(
        ShaderType::Vertex, Platform::GetShaderGlobalDefines(),
        Resources::GetDefaultResourcesPath() / "Shaders/Graphics/Vertex/Lighting/SpotLightShadowMapStrands.vert");
    strands_spot_light_shadow_pipeline->tessellation_control_shader = Shader::CreateTemporary(
        ShaderType::TessellationControl, Platform::GetShaderGlobalDefines(),
        Resources::GetDefaultResourcesPath() / "Shaders/Graphics/TessellationControl/Lighting/ShadowMapStrands.tesc");
    strands_spot_light_shadow_pipeline->tessellation_evaluation_shader =
        Shader::CreateTemporary(ShaderType::TessellationEvaluation, Platform::GetShaderGlobalDefines(),
                                Resources::GetDefaultResourcesPath() /
                                    "Shaders/Graphics/TessellationEvaluation/Lighting/ShadowMapStrands.tese");
    strands_spot_light_shadow_pipeline->geometry_shader = Shader::CreateTemporary(
        ShaderType::Geometry, Platform::GetShaderGlobalDefines(),
        Resources::GetDefaultResourcesPath() / "Shaders/Graphics/Geometry/Lighting/SpotLightShadowMapStrands.geom");
    strands_spot_light_shadow_pipeline->fragment_shader =
        Shader::CreateTemporary(ShaderType::Fragment, Platform::GetShaderGlobalDefines(),
                                Resources::GetDefaultResourcesPath() / "Shaders/Graphics/Fragment/Empty.frag");
    strands_spot_light_shadow_pipeline->geometry_type = GeometryType::Strands;
    strands_spot_light_shadow_pipeline->descriptor_set_layouts.emplace_back(per_frame_layout_);
    strands_spot_light_shadow_pipeline->descriptor_set_layouts.emplace_back(particle_instanced_data_layout_);
    strands_spot_light_shadow_pipeline->depth_attachment_format = Platform::Constants::shadow_map;
    strands_spot_light_shadow_pipeline->stencil_attachment_format = VK_FORMAT_UNDEFINED;
    strands_spot_light_shadow_pipeline->tessellation_patch_control_points = 4;
    auto& push_constant_range = strands_spot_light_shadow_pipeline->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(RenderInstancePushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_ALL;
    strands_spot_light_shadow_pipeline->Initialize();
  }
  if (!strands_directional_light_shadow_pipeline) {
    strands_directional_light_shadow_pipeline = std::make_shared<GraphicsPipeline>();
    strands_directional_light_shadow_pipeline->vertex_shader =
        Shader::CreateTemporary(ShaderType::Vertex, Platform::GetShaderGlobalDefines(),
                                Resources::GetDefaultResourcesPath() /
                                    "Shaders/Graphics/Vertex/Lighting/DirectionalLightShadowMapStrands.vert");
    strands_directional_light_shadow_pipeline->tessellation_control_shader = Shader::CreateTemporary(
        ShaderType::TessellationControl, Platform::GetShaderGlobalDefines(),
        Resources::GetDefaultResourcesPath() / "Shaders/Graphics/TessellationControl/Lighting/ShadowMapStrands.tesc");
    strands_directional_light_shadow_pipeline->tessellation_evaluation_shader =
        Shader::CreateTemporary(ShaderType::TessellationEvaluation, Platform::GetShaderGlobalDefines(),
                                Resources::GetDefaultResourcesPath() /
                                    "Shaders/Graphics/TessellationEvaluation/Lighting/ShadowMapStrands.tese");
    strands_directional_light_shadow_pipeline->geometry_shader =
        Shader::CreateTemporary(ShaderType::Geometry, Platform::GetShaderGlobalDefines(),
                                Resources::GetDefaultResourcesPath() /
                                    "Shaders/Graphics/Geometry/Lighting/DirectionalLightShadowMapStrands.geom");
    strands_directional_light_shadow_pipeline->fragment_shader =
        Shader::CreateTemporary(ShaderType::Fragment, Platform::GetShaderGlobalDefines(),
                                Resources::GetDefaultResourcesPath() / "Shaders/Graphics/Fragment/Empty.frag");
    strands_directional_light_shadow_pipeline->geometry_type = GeometryType::Strands;
    strands_directional_light_shadow_pipeline->descriptor_set_layouts.emplace_back(per_frame_layout_);
    strands_directional_light_shadow_pipeline->descriptor_set_layouts.emplace_back(particle_instanced_data_layout_);
    strands_directional_light_shadow_pipeline->depth_attachment_format = Platform::Constants::shadow_map;
    strands_directional_light_shadow_pipeline->stencil_attachment_format = VK_FORMAT_UNDEFINED;
    strands_directional_light_shadow_pipeline->tessellation_patch_control_points = 4;
    auto& push_constant_range = strands_directional_light_shadow_pipeline->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(RenderInstancePushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_ALL;
    strands_directional_light_shadow_pipeline->Initialize();
  }
#endif
  if (!deferred_prepass_pipeline_normal) {
    deferred_prepass_pipeline_normal = std::make_shared<GraphicsPipeline>();
    deferred_prepass_pipeline_normal->vertex_shader = Shader::CreateTemporary(
        ShaderType::Vertex, CreateRasterNoBindlessTextureShaderDefines(),
        Resources::GetDefaultResourcesPath() / "Shaders/Graphics/Vertex/Standard/Standard.vert");
    deferred_prepass_pipeline_normal->fragment_shader = Shader::CreateTemporary(
        ShaderType::Fragment, CreateRasterMaterialNoBindlessShaderDefines(),
        Resources::GetDefaultResourcesPath() / "Shaders/Graphics/Fragment/Standard/StandardDeferred.frag");
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
                                Resources::GetDefaultResourcesPath() / "Shaders/Graphics/Task/Standard/Standard.task");
    deferred_prepass_pipeline_mesh->mesh_shader =
        Shader::CreateTemporary(ShaderType::Mesh, CreateRasterNoBindlessTextureShaderDefines(),
                                Resources::GetDefaultResourcesPath() / "Shaders/Graphics/Mesh/Standard/Standard.mesh");
    deferred_prepass_pipeline_mesh->fragment_shader = Shader::CreateTemporary(
        ShaderType::Fragment, CreateRasterMaterialNoBindlessShaderDefines(),
        Resources::GetDefaultResourcesPath() / "Shaders/Graphics/Fragment/Standard/StandardDeferred.frag");
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
        Resources::GetDefaultResourcesPath() / "Shaders/Graphics/Vertex/Standard/StandardInstanced.vert");
    instanced_deferred_prepass_pipeline->fragment_shader = Shader::CreateTemporary(
        ShaderType::Fragment, CreateRasterMaterialNoBindlessShaderDefines(),
        Resources::GetDefaultResourcesPath() / "Shaders/Graphics/Fragment/Standard/StandardDeferred.frag");
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
        Resources::GetDefaultResourcesPath() / "Shaders/Graphics/Vertex/Standard/StandardSkinned.vert");
    skinned_deferred_prepass_pipeline->fragment_shader = Shader::CreateTemporary(
        ShaderType::Fragment, CreateRasterMaterialNoBindlessShaderDefines(),
        Resources::GetDefaultResourcesPath() / "Shaders/Graphics/Fragment/Standard/StandardDeferred.frag");
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
        Resources::GetDefaultResourcesPath() / "Shaders/Graphics/Vertex/Standard/SkinnedMotionVectors.vert");
    skinned_motion_vectors_pipeline_->fragment_shader = Shader::CreateTemporary(
        ShaderType::Fragment, CreateRasterMaterialNoBindlessShaderDefines(),
        Resources::GetDefaultResourcesPath() / "Shaders/Graphics/Fragment/Standard/SkinnedMotionVectors.frag");
    skinned_motion_vectors_pipeline_->geometry_type = GeometryType::SkinnedMesh;
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
        Resources::GetDefaultResourcesPath() / "Shaders/Graphics/Vertex/Standard/TransparentMotionVectors.vert");
    transparent_motion_vectors_pipeline_->fragment_shader = Shader::CreateTemporary(
        ShaderType::Fragment, CreateRasterMaterialNoBindlessShaderDefines(),
        Resources::GetDefaultResourcesPath() / "Shaders/Graphics/Fragment/Standard/TransparentMotionVectors.frag");
    transparent_motion_vectors_pipeline_->geometry_type = GeometryType::Mesh;
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
#ifdef EVOENGINE_WINDOWS
  if (!strands_deferred_prepass_pipeline) {
    strands_deferred_prepass_pipeline = std::make_shared<GraphicsPipeline>();
    strands_deferred_prepass_pipeline->vertex_shader = Shader::CreateTemporary(
        ShaderType::Vertex, CreateRasterNoBindlessTextureShaderDefines(),
        Resources::GetDefaultResourcesPath() / "Shaders/Graphics/Vertex/Standard/StandardStrands.vert");
    strands_deferred_prepass_pipeline->tessellation_control_shader = Shader::CreateTemporary(
        ShaderType::TessellationControl, CreateRasterNoBindlessTextureShaderDefines(),
        Resources::GetDefaultResourcesPath() / "Shaders/Graphics/TessellationControl/Standard/StandardStrands.tesc");
    strands_deferred_prepass_pipeline->tessellation_evaluation_shader = Shader::CreateTemporary(
        ShaderType::TessellationEvaluation, CreateRasterNoBindlessTextureShaderDefines(),
        Resources::GetDefaultResourcesPath() / "Shaders/Graphics/TessellationEvaluation/Standard/StandardStrands.tese");
    strands_deferred_prepass_pipeline->geometry_shader = Shader::CreateTemporary(
        ShaderType::Geometry, CreateRasterNoBindlessTextureShaderDefines(),
        Resources::GetDefaultResourcesPath() / "Shaders/Graphics/Geometry/Standard/StandardStrands.geom");
    strands_deferred_prepass_pipeline->fragment_shader = Shader::CreateTemporary(
        ShaderType::Fragment, CreateRasterMaterialNoBindlessShaderDefines(),
        Resources::GetDefaultResourcesPath() / "Shaders/Graphics/Fragment/Standard/StandardDeferred.frag");
    strands_deferred_prepass_pipeline->geometry_type = GeometryType::Strands;
    strands_deferred_prepass_pipeline->descriptor_set_layouts.emplace_back(raster_material_per_frame_layout_);
    strands_deferred_prepass_pipeline->descriptor_set_layouts.emplace_back(particle_instanced_data_layout_);
    strands_deferred_prepass_pipeline->descriptor_set_layouts.emplace_back(empty_descriptor_set_layout_);
    strands_deferred_prepass_pipeline->descriptor_set_layouts.emplace_back(raster_material_layout_);
    strands_deferred_prepass_pipeline->tessellation_patch_control_points = 4;
    strands_deferred_prepass_pipeline->depth_attachment_format = Platform::Constants::render_texture_depth;
    strands_deferred_prepass_pipeline->stencil_attachment_format = VK_FORMAT_UNDEFINED;
    strands_deferred_prepass_pipeline->color_attachment_formats = CreateDeferredGBufferColorAttachmentFormats();
    auto& push_constant_range = strands_deferred_prepass_pipeline->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(RenderInstancePushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_ALL;
    strands_deferred_prepass_pipeline->Initialize();
  }
#endif
  if (!deferred_lighting_pass_pipeline) {
    deferred_lighting_pass_pipeline = std::make_shared<GraphicsPipeline>();
    deferred_lighting_pass_pipeline->vertex_shader = Shader::CreateTemporary(
        ShaderType::Vertex, Resources::GetDefaultResourcesPath() / "Shaders/Graphics/Vertex/TexturePassThrough.vert");
    deferred_lighting_pass_pipeline->fragment_shader = Shader::CreateTemporary(
        ShaderType::Fragment, CreateRasterFixedLightingShaderDefines(3),
        Resources::GetDefaultResourcesPath() / "Shaders/Graphics/Fragment/Standard/StandardDeferredLighting.frag");
    deferred_lighting_pass_pipeline->geometry_type = GeometryType::Mesh;
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
  if (!deferred_lighting_pass_pipeline_scene_camera) {
    deferred_lighting_pass_pipeline_scene_camera = std::make_shared<GraphicsPipeline>();
    deferred_lighting_pass_pipeline_scene_camera->vertex_shader = Shader::CreateTemporary(
        ShaderType::Vertex, Resources::GetDefaultResourcesPath() / "Shaders/Graphics/Vertex/TexturePassThrough.vert");
    deferred_lighting_pass_pipeline_scene_camera->fragment_shader =
        Shader::CreateTemporary(ShaderType::Fragment, CreateRasterFixedLightingShaderDefines(3),
                                Resources::GetDefaultResourcesPath() /
                                    "Shaders/Graphics/Fragment/Standard/StandardDeferredLightingSceneCamera.frag");
    deferred_lighting_pass_pipeline_scene_camera->geometry_type = GeometryType::Mesh;
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
        Resources::GetDefaultResourcesPath() / "Shaders/Graphics/Vertex/Standard/Standard.vert");
    transparent_geometry_pipeline_normal->fragment_shader = Shader::CreateTemporary(
        ShaderType::Fragment, CreateRasterMaterialFixedLightingShaderDefines(4),
        Resources::GetDefaultResourcesPath() / "Shaders/Graphics/Fragment/Standard/StandardTransparent.frag");
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
                                Resources::GetDefaultResourcesPath() / "Shaders/Graphics/Vertex/Gizmos/Gizmos.vert");
    gizmos->fragment_shader =
        Shader::CreateTemporary(ShaderType::Fragment, Platform::GetShaderGlobalDefines(),
                                Resources::GetDefaultResourcesPath() / "Shaders/Graphics/Fragment/Gizmos/Gizmos.frag");
    gizmos->geometry_type = GeometryType::Mesh;
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
        Resources::GetDefaultResourcesPath() / "Shaders/Graphics/Vertex/Gizmos/GizmosNormalColored.vert");
    gizmos_normal_colored->fragment_shader = Shader::CreateTemporary(
        ShaderType::Fragment, Platform::GetShaderGlobalDefines(),
        Resources::GetDefaultResourcesPath() / "Shaders/Graphics/Fragment/Gizmos/GizmosColored.frag");
    gizmos_normal_colored->geometry_type = GeometryType::Mesh;
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
        Resources::GetDefaultResourcesPath() / "Shaders/Graphics/Vertex/Gizmos/GizmosVertexColored.vert");
    gizmos_vertex_colored->fragment_shader = Shader::CreateTemporary(
        ShaderType::Fragment, Platform::GetShaderGlobalDefines(),
        Resources::GetDefaultResourcesPath() / "Shaders/Graphics/Fragment/Gizmos/GizmosColored.frag");
    gizmos_vertex_colored->geometry_type = GeometryType::Mesh;
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
        Resources::GetDefaultResourcesPath() / "Shaders/Graphics/Vertex/Gizmos/GizmosInstancedColored.vert");
    gizmos_instanced_colored->fragment_shader = Shader::CreateTemporary(
        ShaderType::Fragment, Platform::GetShaderGlobalDefines(),
        Resources::GetDefaultResourcesPath() / "Shaders/Graphics/Fragment/Gizmos/GizmosColored.frag");
    gizmos_instanced_colored->geometry_type = GeometryType::Mesh;
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
        Resources::GetDefaultResourcesPath() / "Shaders/Graphics/Vertex/DDGI/DDGIProbeVisualization.vert");
    ddgi_probe_visualization_pipeline_->fragment_shader = Shader::CreateTemporary(
        ShaderType::Fragment, Platform::GetShaderGlobalDefines(),
        Resources::GetDefaultResourcesPath() / "Shaders/Graphics/Fragment/DDGI/DDGIProbeVisualization.frag");
    ddgi_probe_visualization_pipeline_->geometry_type = GeometryType::Mesh;
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
        Resources::GetDefaultResourcesPath() / "Shaders/Graphics/Vertex/DDGI/DDGIProbeRayVisualization.vert");
    ddgi_probe_ray_visualization_pipeline_->fragment_shader = Shader::CreateTemporary(
        ShaderType::Fragment, Platform::GetShaderGlobalDefines(),
        Resources::GetDefaultResourcesPath() / "Shaders/Graphics/Fragment/DDGI/DDGIProbeRayVisualization.frag");
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
#ifdef EVOENGINE_WINDOWS
  if (!gizmos_strands) {
    gizmos_strands = std::make_shared<GraphicsPipeline>();
    gizmos_strands->vertex_shader = Shader::CreateTemporary(
        ShaderType::Vertex, Platform::GetShaderGlobalDefines(),
        Resources::GetDefaultResourcesPath() / "Shaders/Graphics/Vertex/Gizmos/GizmosStrands.vert");
    gizmos_strands->tessellation_control_shader = Shader::CreateTemporary(
        ShaderType::TessellationControl, Platform::GetShaderGlobalDefines(),
        Resources::GetDefaultResourcesPath() / "Shaders/Graphics/TessellationControl/Gizmos/GizmosStrands.tesc");
    gizmos_strands->tessellation_evaluation_shader = Shader::CreateTemporary(
        ShaderType::TessellationEvaluation, Platform::GetShaderGlobalDefines(),
        Resources::GetDefaultResourcesPath() / "Shaders/Graphics/TessellationEvaluation/Gizmos/GizmosStrands.tese");
    gizmos_strands->geometry_shader = Shader::CreateTemporary(
        ShaderType::Geometry, Platform::GetShaderGlobalDefines(),
        Resources::GetDefaultResourcesPath() / "Shaders/Graphics/Geometry/Gizmos/GizmosStrands.geom");
    gizmos_strands->fragment_shader =
        Shader::CreateTemporary(ShaderType::Fragment, Platform::GetShaderGlobalDefines(),
                                Resources::GetDefaultResourcesPath() / "Shaders/Graphics/Fragment/Gizmos/Gizmos.frag");
    gizmos_strands->geometry_type = GeometryType::Strands;
    gizmos_strands->depth_attachment_format = Platform::Constants::render_texture_depth;
    gizmos_strands->stencil_attachment_format = VK_FORMAT_UNDEFINED;
    gizmos_strands->tessellation_patch_control_points = 4;
    gizmos_strands->color_attachment_formats = {1, Platform::Constants::render_texture_color};
    gizmos_strands->descriptor_set_layouts.emplace_back(per_frame_layout_);
    auto& push_constant_range = gizmos_strands->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(GizmosPushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_ALL;

    gizmos_strands->Initialize();
  }
  if (!gizmos_strands_normal_colored) {
    gizmos_strands_normal_colored = std::make_shared<GraphicsPipeline>();
    gizmos_strands_normal_colored->vertex_shader = Shader::CreateTemporary(
        ShaderType::Vertex, Platform::GetShaderGlobalDefines(),
        Resources::GetDefaultResourcesPath() / "Shaders/Graphics/Vertex/Gizmos/GizmosStrandsNormalColored.vert");
    gizmos_strands_normal_colored->tessellation_control_shader = Shader::CreateTemporary(
        ShaderType::TessellationControl, Platform::GetShaderGlobalDefines(),
        Resources::GetDefaultResourcesPath() / "Shaders/Graphics/TessellationControl/Gizmos/GizmosStrandsColored.tesc");
    gizmos_strands_normal_colored->tessellation_evaluation_shader =
        Shader::CreateTemporary(ShaderType::TessellationEvaluation, Platform::GetShaderGlobalDefines(),
                                Resources::GetDefaultResourcesPath() /
                                    "Shaders/Graphics/TessellationEvaluation/Gizmos/GizmosStrandsColored.tese");
    gizmos_strands_normal_colored->geometry_shader = Shader::CreateTemporary(
        ShaderType::Geometry, Platform::GetShaderGlobalDefines(),
        Resources::GetDefaultResourcesPath() / "Shaders/Graphics/Geometry/Gizmos/GizmosStrandsColored.geom");
    gizmos_strands_normal_colored->fragment_shader = Shader::CreateTemporary(
        ShaderType::Fragment, Platform::GetShaderGlobalDefines(),
        Resources::GetDefaultResourcesPath() / "Shaders/Graphics/Fragment/Gizmos/GizmosColored.frag");
    gizmos_strands_normal_colored->geometry_type = GeometryType::Strands;
    gizmos_strands_normal_colored->depth_attachment_format = Platform::Constants::render_texture_depth;
    gizmos_strands_normal_colored->stencil_attachment_format = VK_FORMAT_UNDEFINED;
    gizmos_strands_normal_colored->color_attachment_formats = {1, Platform::Constants::render_texture_color};
    gizmos_strands_normal_colored->descriptor_set_layouts.emplace_back(per_frame_layout_);
    gizmos_strands_normal_colored->tessellation_patch_control_points = 4;
    auto& push_constant_range = gizmos_strands_normal_colored->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(GizmosPushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_ALL;

    gizmos_strands_normal_colored->Initialize();
  }
  if (!gizmos_strands_vertex_colored) {
    gizmos_strands_vertex_colored = std::make_shared<GraphicsPipeline>();
    gizmos_strands_vertex_colored->vertex_shader = Shader::CreateTemporary(
        ShaderType::Vertex, Platform::GetShaderGlobalDefines(),
        Resources::GetDefaultResourcesPath() / "Shaders/Graphics/Vertex/Gizmos/GizmosStrandsVertexColored.vert");
    gizmos_strands_vertex_colored->tessellation_control_shader = Shader::CreateTemporary(
        ShaderType::TessellationControl, Platform::GetShaderGlobalDefines(),
        Resources::GetDefaultResourcesPath() / "Shaders/Graphics/TessellationControl/Gizmos/GizmosStrandsColored.tesc");
    gizmos_strands_vertex_colored->tessellation_evaluation_shader =
        Shader::CreateTemporary(ShaderType::TessellationEvaluation, Platform::GetShaderGlobalDefines(),
                                Resources::GetDefaultResourcesPath() /
                                    "Shaders/Graphics/TessellationEvaluation/Gizmos/GizmosStrandsColored.tese");
    gizmos_strands_vertex_colored->geometry_shader = Shader::CreateTemporary(
        ShaderType::Geometry, Platform::GetShaderGlobalDefines(),
        Resources::GetDefaultResourcesPath() / "Shaders/Graphics/Geometry/Gizmos/GizmosStrandsColored.geom");
    gizmos_strands_vertex_colored->fragment_shader = Shader::CreateTemporary(
        ShaderType::Fragment, Platform::GetShaderGlobalDefines(),
        Resources::GetDefaultResourcesPath() / "Shaders/Graphics/Fragment/Gizmos/GizmosColored.frag");
    gizmos_strands_vertex_colored->geometry_type = GeometryType::Strands;
    gizmos_strands_vertex_colored->tessellation_patch_control_points = 4;
    gizmos_strands_vertex_colored->depth_attachment_format = Platform::Constants::render_texture_depth;
    gizmos_strands_vertex_colored->stencil_attachment_format = VK_FORMAT_UNDEFINED;
    gizmos_strands_vertex_colored->color_attachment_formats = {1, Platform::Constants::render_texture_color};
    gizmos_strands_vertex_colored->descriptor_set_layouts.emplace_back(per_frame_layout_);
    gizmos_strands_vertex_colored->tessellation_patch_control_points = 4;
    auto& push_constant_range = gizmos_strands_vertex_colored->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(GizmosPushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_ALL;
    gizmos_strands_vertex_colored->Initialize();
  }

#endif
#pragma endregion
#pragma region Ray Tracing Pipelines
  constexpr auto ray_tracing_push_constant_stages = VK_SHADER_STAGE_RAYGEN_BIT_KHR | VK_SHADER_STAGE_MISS_BIT_KHR |
                                                    VK_SHADER_STAGE_CLOSEST_HIT_BIT_KHR |
                                                    VK_SHADER_STAGE_ANY_HIT_BIT_KHR;
  double ray_tracing_fallback_build_milliseconds = 0.0;
  double ray_query_fallback_build_milliseconds = 0.0;
  if (Platform::RayTracingEnabled() && !ray_tracing_camera_fallback_pipeline_) {
    const auto build_start = std::chrono::steady_clock::now();
    ray_tracing_camera_fallback_pipeline_ = CreateRayTracingCameraPipeline(
        per_frame_layout_, ray_tracing_layout_, ray_tracing_camera_output_layout_, Platform::GetShaderGlobalDefines());
    ray_tracing_fallback_build_milliseconds =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - build_start).count();
    ray_tracing_camera_pipeline = ray_tracing_camera_fallback_pipeline_;
  }
  if (Platform::RayQueryEnabled() && !ray_query_camera_fallback_pipeline_) {
    const auto build_start = std::chrono::steady_clock::now();
    ray_query_camera_fallback_pipeline_ = CreateRayQueryCameraPipeline(
        per_frame_layout_, ray_tracing_layout_, ray_tracing_camera_output_layout_, Platform::GetShaderGlobalDefines());
    ray_query_fallback_build_milliseconds =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - build_start).count();
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
        ray_query_factory, ray_tracing_fallback_build_milliseconds, ray_query_fallback_build_milliseconds);
  }
  if (Platform::RayTracingEnabled() && !ray_tracing_point_cloud_pipeline) {
    ray_tracing_point_cloud_pipeline = std::make_shared<RayTracingPipeline>();
    ray_tracing_point_cloud_pipeline->raygen_shader =
        Shader::CreateTemporary(ShaderType::RayGen, Platform::GetShaderGlobalDefines(),
                                Resources::GetDefaultResourcesPath() / "Shaders/RayTracing/RayGen/PointCloud.rgen");
    ray_tracing_point_cloud_pipeline->miss_shader =
        Shader::CreateTemporary(ShaderType::Miss, Platform::GetShaderGlobalDefines(),
                                Resources::GetDefaultResourcesPath() / "Shaders/RayTracing/Miss/PointCloud.rmiss");
    ray_tracing_point_cloud_pipeline->closest_hit_shader = Shader::CreateTemporary(
        ShaderType::ClosestHit, Platform::GetShaderGlobalDefines(),
        Resources::GetDefaultResourcesPath() / "Shaders/RayTracing/ClosestHit/PointCloud.rchit");
    ray_tracing_point_cloud_pipeline->descriptor_set_layouts.emplace_back(per_frame_layout_);
    ray_tracing_point_cloud_pipeline->descriptor_set_layouts.emplace_back(ray_tracing_layout_);
    ray_tracing_point_cloud_pipeline->descriptor_set_layouts.emplace_back(ray_tracing_point_cloud_layout_);
    auto& push_constant_range = ray_tracing_point_cloud_pipeline->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(RayTracingPointCloudPushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = ray_tracing_push_constant_stages;
    ray_tracing_point_cloud_pipeline->Initialize();
  }
  if (Platform::RayTracingEnabled() && !ddgi_probe_ray_diagnostic_pipeline_) {
    ddgi_probe_ray_diagnostic_pipeline_ = std::make_shared<RayTracingPipeline>();
    ddgi_probe_ray_diagnostic_pipeline_->raygen_shader = Shader::CreateTemporary(
        ShaderType::RayGen, Platform::GetShaderGlobalDefines(),
        Resources::GetDefaultResourcesPath() / "Shaders/RayTracing/RayGen/DDGIProbeDiagnostics.rgen");
    ddgi_probe_ray_diagnostic_pipeline_->miss_shader = Shader::CreateTemporary(
        ShaderType::Miss, Platform::GetShaderGlobalDefines(),
        Resources::GetDefaultResourcesPath() / "Shaders/RayTracing/Miss/DDGIProbeDiagnostics.rmiss");
    ddgi_probe_ray_diagnostic_pipeline_->closest_hit_shader = Shader::CreateTemporary(
        ShaderType::ClosestHit, Platform::GetShaderGlobalDefines(),
        Resources::GetDefaultResourcesPath() / "Shaders/RayTracing/ClosestHit/DDGIProbeDiagnostics.rchit");
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
  render_instances_list_.resize(max_frames_in_flight);
  for (auto& i : render_instances_list_) {
    i = std::make_shared<RenderInstanceStorage>();
  }
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
  return scene ? scene->environment.ddgi_settings : fallback_ddgi_settings_;
}

const RenderLayer::DdgiSettings& RenderLayer::GetDdgiSettings() const {
  const auto scene = GetScene();
  return scene ? scene->environment.ddgi_settings : fallback_ddgi_settings_;
}

glm::ivec3 RenderLayer::GetDdgiProbeScrollOffset() const {
  return ddgi_probe_scroll_offset_;
}

glm::ivec3 RenderLayer::GetDdgiLastProbeScrollDelta() const {
  return ddgi_last_probe_scroll_delta_;
}

uint32_t RenderLayer::GetDdgiPendingProbeUpdateCount() const {
  return ddgi_pending_probe_update_count_;
}

RenderLayer::DdgiProbeUpdateStats RenderLayer::GetDdgiLastProbeUpdateStats() const {
  if (ddgi_frame_probe_update_indices_.empty()) {
    return {};
  }
  return {static_cast<uint32_t>(ddgi_frame_probe_update_indices_.size()), ddgi_frame_probe_update_indices_.front(),
          ddgi_frame_probe_update_indices_.back()};
}

RenderLayer::DdgiPerformanceStats RenderLayer::GetDdgiLastPerformanceStats() const {
  return ddgi_last_performance_stats_;
}

uint32_t RenderLayer::GetDdgiLastProbeUpdateReasons() const {
  return ddgi_last_probe_update_reasons_;
}

std::string RenderLayer::GetDdgiLastProbeUpdateReasonText() const {
  return FormatDdgiUpdateReasons(ddgi_last_probe_update_reasons_);
}

RenderLayer::DdgiProbeDebugDataView RenderLayer::GetDdgiProbeDebugData(const bool refresh_readback) const {
  if (refresh_readback && ddgi_probe_metadata_readback_buffer_ && ddgi_probe_debug_metadata_byte_size_ != 0 &&
      ddgi_probe_metadata_readback_buffer_->GetSize() >= ddgi_probe_debug_metadata_byte_size_) {
    ddgi_probe_metadata_readback_buffer_->DownloadVector(
        ddgi_probe_debug_metadata_, static_cast<size_t>(ddgi_probe_debug_metadata_byte_size_ / sizeof(glm::vec4)));
  }
  if (refresh_readback && ddgi_probe_debug_ray_samples_available_ && ddgi_probe_ray_readback_buffer_ &&
      ddgi_probe_debug_ray_sample_count_ != 0u &&
      ddgi_probe_ray_readback_buffer_->GetSize() >=
          static_cast<uint64_t>(ddgi_probe_debug_ray_sample_count_) * sizeof(PointCloudSample)) {
    ddgi_probe_ray_readback_buffer_->DownloadVector(ddgi_probe_debug_ray_samples_, ddgi_probe_debug_ray_sample_count_);
  }
  return {&ddgi_probe_debug_metadata_,        &ddgi_probe_debug_update_ages_,
          &ddgi_probe_debug_ray_samples_,     ddgi_probe_debug_metadata_probe_count_,
          ddgi_probe_debug_ray_probe_index_,  ddgi_probe_debug_ray_physical_probe_index_,
          ddgi_probe_debug_ray_sample_count_, ddgi_probe_debug_ray_samples_available_};
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
                                           const bool track_ddgi_scene_inputs) {
  if (!scene)
    return;
  const ProfilerScope profiler_scope("RenderLayer::PrepareSceneForRendering", "Render");
  const auto current_frame_index = Platform::GetCurrentFrameIndex();
  Platform::ResetRenderPassDrawStats(current_frame_index);
  const auto current_render_instances = render_instances_list_[current_frame_index];
  if (update_editor_selection) {
    ApplyAnimators();
  }
  if (GeometryStorage::HasPendingUploads()) {
    GeometryStorage::WaitForPendingUploads();
  }
  const bool render_instance_updated = UpdateRenderInstanceStorage(scene, current_frame_index, include_editor_cameras,
                                                                   update_editor_selection, track_ddgi_scene_inputs);

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
      bool reset_ray_tracing = false;
      bool reset_ray_query = false;
      for (const auto& [transform, camera] : current_render_instances->cameras) {
        if (!camera)
          continue;
        const auto mode = Camera::ResolveCameraRenderMode(camera->camera_render_mode);
        if ((variant_update.ray_tracing_activated && mode == Camera::CameraRenderMode::RayTracing) ||
            (variant_update.ray_query_activated && mode == Camera::CameraRenderMode::RayQuery)) {
          camera->ResetFrameCount();
          reset_ray_tracing |= mode == Camera::CameraRenderMode::RayTracing;
          reset_ray_query |= mode == Camera::CameraRenderMode::RayQuery;
        }
      }
      if (reset_ray_tracing)
        ray_camera_shader_variant_cache_->RecordAccumulationReset(RayCameraShaderTechnique::RayTracing);
      if (reset_ray_query)
        ray_camera_shader_variant_cache_->RecordAccumulationReset(RayCameraShaderTechnique::RayQuery);
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
    PrepareDdgiFrameState(scene, current_render_instances, ddgi_scene_change_triggers_);
  }
  current_render_instances->BuildPreviousInstanceInfoBlocks(GetPreviousRenderInstanceStorage());
  current_render_instances->Upload();
  BindRenderInstanceStorage(current_frame_index, current_render_instances);
}

void RenderLayer::PrepareDdgiFrameState(const std::shared_ptr<Scene>& scene,
                                        const std::shared_ptr<RenderInstanceStorage>& render_instances,
                                        const int ddgi_scene_change_triggers) {
  auto& ddgi_settings = scene ? scene->environment.ddgi_settings : fallback_ddgi_settings_;
  ddgi_frame_trace_probe_rays_ = false;
  ddgi_frame_ray_push_constant_ = {};
  ddgi_frame_probe_update_push_constant_ = {};
  ddgi_frame_probe_relocation_reset_push_constant_ = {};
  ddgi_frame_probe_relocation_update_push_constant_ = {};
  ddgi_frame_probe_classification_reset_push_constant_ = {};
  ddgi_frame_probe_classification_update_push_constant_ = {};
  ddgi_clear_probe_atlas_this_frame_ = false;
  ddgi_frame_probe_relocation_reset_ = false;
  ddgi_frame_probe_relocation_enabled_ = false;
  ddgi_frame_probe_classification_reset_ = false;
  ddgi_frame_probe_classification_enabled_ = false;
  ddgi_frame_probe_variability_enabled_ = false;
  ddgi_frame_probe_warmup_frame_index_ = 0;
  ddgi_frame_probe_warmup_frame_count_ = 0;
  ddgi_frame_probe_warmup_active_ = false;
  ddgi_frame_probe_update_hysteresis_ = 0.0f;
  ddgi_last_probe_update_reasons_ = DdgiUpdateReasonNone;
  ddgi_frame_selected_probe_ray_local_index_ = (std::numeric_limits<uint32_t>::max)();
  ddgi_frame_selected_probe_ray_sample_count_ = 0;
  ddgi_probe_debug_ray_samples_available_ = false;
  if (!render_instances) {
    return;
  }

  render_instances->render_info_block.ddgi_indirect_intensity = 0.0f;
  auto ddgi_ray_source = CreateDdgiProbeRayDiagnosticSource(scene, ddgi_settings);
  const auto source_probe_count = RenderLayer::GetDdgiProbeCount(ddgi_ray_source.probe_counts);
  const auto ddgi_total_probe_count = RenderLayer::GetDdgiAllocatedProbeCount(ddgi_settings, source_probe_count);
  const auto layout = RenderLayer::CalculateDdgiFrameResourceLayout(ddgi_settings, ddgi_total_probe_count);
  ddgi_frame_resource_layout_ = layout;
  bool ddgi_resource_changed = false;
  if (ShouldUseDdgiFrameResources(ddgi_settings)) {
    if (!ddgi_probe_metadata_buffer_ || ddgi_probe_metadata_buffer_->GetSize() != layout.probe_metadata_byte_size) {
      ddgi_probe_metadata_buffer_ = CreateDdgiProbeStateBuffer(layout.probe_metadata_byte_size);
      ddgi_clear_probe_atlas_this_frame_ = true;
      ddgi_resource_changed = true;
    }
    if (!HasDdgiAtlasImageLayout(ddgi_irradiance_atlas_, layout.irradiance_atlas, VK_FORMAT_R16G16B16A16_SFLOAT)) {
      ddgi_irradiance_atlas_ = CreateDdgiAtlasImage(layout.irradiance_atlas, VK_FORMAT_R16G16B16A16_SFLOAT);
      ddgi_clear_probe_atlas_this_frame_ = true;
      ddgi_resource_changed = true;
    }
    if (!HasDdgiAtlasImageLayout(ddgi_visibility_atlas_, layout.visibility_atlas, VK_FORMAT_R16G16_SFLOAT)) {
      ddgi_visibility_atlas_ = CreateDdgiAtlasImage(layout.visibility_atlas, VK_FORMAT_R16G16_SFLOAT);
      ddgi_clear_probe_atlas_this_frame_ = true;
      ddgi_resource_changed = true;
    }
    if (!HasDdgiAtlasImageLayout(ddgi_variability_atlas_, layout.variability_atlas, VK_FORMAT_R16_SFLOAT)) {
      ddgi_variability_atlas_ = CreateDdgiAtlasImage(layout.variability_atlas, VK_FORMAT_R16_SFLOAT);
      ddgi_clear_probe_atlas_this_frame_ = true;
      ddgi_resource_changed = true;
    }
    if (!ddgi_variability_readback_buffer_ ||
        ddgi_variability_readback_buffer_->GetSize() != layout.variability_readback_byte_size) {
      ddgi_variability_readback_buffer_ = std::make_shared<Buffer>(layout.variability_readback_byte_size, true);
      ddgi_resource_changed = true;
    }
  }
  if (!ShouldTraceDdgiProbeRays(ddgi_settings) || !Platform::RayTracingEnabled() ||
      !render_instances->mesh_top_level_acceleration_structure) {
    return;
  }
  const bool texture_uploads_pending = TextureStorage::HasPendingUploads();
  const bool project_scene_inputs_pending = ProjectManager::HasProject() && !ProjectManager::IsProjectIdle();
  const bool scene_inputs_pending = texture_uploads_pending || project_scene_inputs_pending;
  if (scene_inputs_pending) {
    ddgi_deferred_scene_readiness_refresh_ = true;
    ddgi_scene_input_settle_frame_count_ = 0;
    ddgi_clear_probe_atlas_this_frame_ = true;
    ddgi_last_probe_update_reasons_ = DdgiUpdateReasonSceneInput;
    ddgi_active_probe_update_reasons_ = DdgiUpdateReasonNone;
    ddgi_pending_probe_update_indices_.clear();
    ddgi_pending_probe_update_cursor_ = 0;
    ddgi_pending_probe_update_count_ = 0;
    ddgi_frame_probe_update_indices_.clear();
    return;
  }
  if (ddgi_deferred_scene_readiness_refresh_ &&
      ddgi_scene_input_settle_frame_count_ < kDdgiSceneInputSettleFrameCount) {
    ++ddgi_scene_input_settle_frame_count_;
    ddgi_clear_probe_atlas_this_frame_ = true;
    ddgi_last_probe_update_reasons_ = DdgiUpdateReasonSceneInput;
    ddgi_active_probe_update_reasons_ = DdgiUpdateReasonNone;
    ddgi_pending_probe_update_indices_.clear();
    ddgi_pending_probe_update_cursor_ = 0;
    ddgi_pending_probe_update_count_ = 0;
    ddgi_frame_probe_update_indices_.clear();
    return;
  }

  const auto ray_count = static_cast<uint32_t>(glm::max(ddgi_settings.runtime.ray_count, 1));
  const auto effective_max_ray_distance = CalculateDdgiEffectiveMaxRayDistance(ddgi_settings, ddgi_ray_source);
  const glm::vec4 trace_parameters{
      effective_max_ray_distance, glm::max(ddgi_settings.runtime.normal_bias, 0.001f),
      ddgi_ray_source.enable_probe_relocation || ddgi_ray_source.enable_probe_classification ? 1.0f : 0.0f, 0.0f};
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
  const auto ddgi_ray_source_common_changed =
      !ddgi_has_previous_ray_source_ || ddgi_previous_probe_counts_ != ddgi_ray_source.probe_counts ||
      ddgi_previous_probe_step_x_ != ddgi_ray_source.probe_step_x ||
      ddgi_previous_probe_step_y_ != ddgi_ray_source.probe_step_y ||
      ddgi_previous_probe_step_z_ != ddgi_ray_source.probe_step_z || ddgi_previous_ray_count_ != ray_count ||
      ddgi_previous_trace_parameters_ != trace_parameters || ddgi_previous_update_parameters_ != update_parameters ||
      ddgi_previous_probe_state_parameters_ != probe_state_parameters ||
      ddgi_previous_probe_blend_parameters_ != probe_blend_parameters ||
      ddgi_previous_movement_type_ != ddgi_ray_source.movement_type;
  const auto first_probe_changed =
      ddgi_has_previous_ray_source_ && ddgi_previous_first_probe_ != ddgi_ray_source.first_probe;
  const auto ddgi_scrolling_enabled =
      ddgi_ray_source.movement_type == static_cast<int>(DdgiVolumeMovementType::Scrolling);
  bool ddgi_ray_source_changed = ddgi_ray_source_common_changed;
  const auto reset_probe_history = ddgi_settings.runtime.reset_probe_history;
  ddgi_probe_scroll_clear_ = glm::ivec3(0);
  ddgi_last_probe_scroll_delta_ = glm::ivec3(0);
  if (ddgi_ray_source_common_changed) {
    ddgi_has_previous_ray_source_ = true;
    ddgi_probe_scroll_base_first_probe_ = ddgi_ray_source.first_probe;
    ddgi_probe_scroll_offset_ = glm::ivec3(0);
    ddgi_probe_scroll_directions_ = glm::ivec3(1);
    ddgi_previous_probe_counts_ = ddgi_ray_source.probe_counts;
    ddgi_previous_first_probe_ = ddgi_ray_source.first_probe;
    ddgi_previous_probe_step_x_ = ddgi_ray_source.probe_step_x;
    ddgi_previous_probe_step_y_ = ddgi_ray_source.probe_step_y;
    ddgi_previous_probe_step_z_ = ddgi_ray_source.probe_step_z;
    ddgi_previous_ray_count_ = ray_count;
    ddgi_previous_trace_parameters_ = trace_parameters;
    ddgi_previous_update_parameters_ = update_parameters;
    ddgi_previous_probe_state_parameters_ = probe_state_parameters;
    ddgi_previous_probe_blend_parameters_ = probe_blend_parameters;
    ddgi_previous_movement_type_ = ddgi_ray_source.movement_type;
  } else if (first_probe_changed) {
    if (ddgi_scrolling_enabled) {
      ResetDdgiProbeScrollOrigin(ddgi_probe_scroll_base_first_probe_, ddgi_probe_scroll_offset_,
                                 ddgi_probe_scroll_directions_, ddgi_ray_source.probe_counts,
                                 ddgi_ray_source.probe_step_x, ddgi_ray_source.probe_step_y,
                                 ddgi_ray_source.probe_step_z);
      const auto effective_first_probe = CalculateDdgiEffectiveFirstProbe(
          ddgi_probe_scroll_base_first_probe_, ddgi_ray_source.probe_step_x, ddgi_ray_source.probe_step_y,
          ddgi_ray_source.probe_step_z, ddgi_probe_scroll_offset_);
      const auto first_probe_delta = ddgi_ray_source.first_probe - effective_first_probe;
      ddgi_probe_scroll_directions_ = {
          AxisCoordinate(first_probe_delta, ddgi_ray_source.probe_step_x) >= 0.0f ? 1 : -1,
          AxisCoordinate(first_probe_delta, ddgi_ray_source.probe_step_y) >= 0.0f ? 1 : -1,
          AxisCoordinate(first_probe_delta, ddgi_ray_source.probe_step_z) >= 0.0f ? 1 : -1};
      ddgi_last_probe_scroll_delta_ = CalculateDdgiProbeScrollDelta(
          first_probe_delta, ddgi_ray_source.probe_step_x, ddgi_ray_source.probe_step_y, ddgi_ray_source.probe_step_z);
      if (ddgi_last_probe_scroll_delta_.x != 0) {
        ddgi_probe_scroll_offset_.x += ddgi_last_probe_scroll_delta_.x;
        ddgi_probe_scroll_clear_.x = 1;
      }
      if (ddgi_last_probe_scroll_delta_.y != 0) {
        ddgi_probe_scroll_offset_.y += ddgi_last_probe_scroll_delta_.y;
        ddgi_probe_scroll_clear_.y = 1;
      }
      if (ddgi_last_probe_scroll_delta_.z != 0) {
        ddgi_probe_scroll_offset_.z += ddgi_last_probe_scroll_delta_.z;
        ddgi_probe_scroll_clear_.z = 1;
      }
      ddgi_previous_first_probe_ = ddgi_ray_source.first_probe;
    } else {
      ddgi_ray_source_changed = true;
      ddgi_probe_scroll_base_first_probe_ = ddgi_ray_source.first_probe;
      ddgi_probe_scroll_offset_ = glm::ivec3(0);
      ddgi_probe_scroll_directions_ = glm::ivec3(1);
      ddgi_previous_first_probe_ = ddgi_ray_source.first_probe;
    }
  }
  if (ddgi_scrolling_enabled) {
    ddgi_ray_source.first_probe = CalculateDdgiEffectiveFirstProbe(
        ddgi_probe_scroll_base_first_probe_, ddgi_ray_source.probe_step_x, ddgi_ray_source.probe_step_y,
        ddgi_ray_source.probe_step_z, ddgi_probe_scroll_offset_);
  }
  ddgi_ray_source.probe_scroll_offset = ddgi_probe_scroll_offset_;
  ddgi_ray_source.probe_scroll_clear = ddgi_probe_scroll_clear_;
  ddgi_ray_source.probe_scroll_directions = ddgi_probe_scroll_directions_;
  const bool ddgi_scroll_clear_this_frame =
      ddgi_probe_scroll_clear_.x != 0 || ddgi_probe_scroll_clear_.y != 0 || ddgi_probe_scroll_clear_.z != 0;
  const bool scene_readiness_refresh = ddgi_deferred_scene_readiness_refresh_;
  const bool scene_material_refresh =
      ddgi_scene_material_inputs_changed_ &&
      DdgiTriggerConditionEnabled(ddgi_scene_change_triggers, (ddgi_ray_source.warmup_trigger_conditions |
                                                               ddgi_ray_source.variability_reset_trigger_conditions) &
                                                                  DdgiVolumeTriggerConditionAll);
  uint32_t full_refresh_reasons = DdgiUpdateReasonNone;
  if (ddgi_ray_source_changed) {
    full_refresh_reasons |= DdgiUpdateReasonSource;
  }
  if (reset_probe_history) {
    full_refresh_reasons |= DdgiUpdateReasonManualReset;
  }
  if (scene_material_refresh || scene_readiness_refresh) {
    full_refresh_reasons |= DdgiUpdateReasonSceneInput;
  }
  ddgi_last_probe_update_reasons_ =
      full_refresh_reasons == DdgiUpdateReasonNone ? DdgiUpdateReasonSteadyState : full_refresh_reasons;
  ddgi_settings.runtime.reset_probe_history = false;
  ddgi_pending_probe_update_indices_.clear();
  ddgi_pending_probe_update_cursor_ = 0;
  ddgi_pending_probe_update_count_ = 0;
  ddgi_clear_probe_atlas_this_frame_ = ddgi_clear_probe_atlas_this_frame_ || ddgi_ray_source_changed ||
                                       reset_probe_history || scene_material_refresh || scene_readiness_refresh;
  const auto probe_variability_enabled = ddgi_ray_source.enable_probe_variability;
  if (probe_variability_enabled && ddgi_probe_variability_sample_count_ != 0u) {
    ddgi_probe_variability_average_ = ReadDdgiProbeVariabilityAverage(ddgi_variability_readback_buffer_);
  }
  const bool internal_ddgi_refresh = ddgi_ray_source_changed || reset_probe_history || ddgi_resource_changed ||
                                     ddgi_scroll_clear_this_frame || scene_material_refresh || scene_readiness_refresh;
  const bool reset_ddgi_warmup_state =
      internal_ddgi_refresh ||
      DdgiTriggerConditionEnabled(ddgi_scene_change_triggers,
                                  ddgi_ray_source.warmup_trigger_conditions & DdgiVolumeTriggerConditionAll);
  const bool reset_ddgi_variability_state =
      internal_ddgi_refresh ||
      DdgiTriggerConditionEnabled(ddgi_scene_change_triggers,
                                  ddgi_ray_source.variability_reset_trigger_conditions & DdgiVolumeTriggerConditionAll);
  if (!probe_variability_enabled || reset_ddgi_variability_state) {
    ddgi_probe_variability_sample_count_ = 0;
    ddgi_probe_variability_stable_sample_count_ = 0;
    ddgi_probe_variability_average_ = 0.0f;
    ddgi_probe_variability_converged_ = false;
  }
  if (reset_ddgi_warmup_state) {
    ddgi_probe_warmup_frame_index_ = 0;
  }
  ddgi_frame_probe_warmup_frame_count_ = static_cast<uint32_t>(glm::max(ddgi_settings.runtime.warmup_frames, 0));
  ddgi_frame_probe_warmup_frame_index_ = ddgi_probe_warmup_frame_index_;
  ddgi_frame_probe_warmup_active_ = ddgi_frame_probe_warmup_frame_count_ != 0u &&
                                    ddgi_probe_warmup_frame_index_ < ddgi_frame_probe_warmup_frame_count_;
  const auto ddgi_first_warmup_frame = ddgi_frame_probe_warmup_active_ && ddgi_probe_warmup_frame_index_ == 0u;
  if (ddgi_frame_probe_warmup_active_ && full_refresh_reasons == DdgiUpdateReasonNone) {
    ddgi_last_probe_update_reasons_ |= DdgiUpdateReasonWarmup;
  }

  DdgiProbeUpdateWindow ddgi_update_window;
  ddgi_update_window.start_probe_index = 0;
  ddgi_update_window.probe_count = ddgi_total_probe_count;
  ddgi_update_window.next_start_probe_index = 0;
  ddgi_update_window.remaining_probe_count = 0;
  bool reset_probe_state = ddgi_ray_source_changed || reset_probe_history || scene_material_refresh ||
                           scene_readiness_refresh || !ddgi_probe_state_buffer_ ||
                           ddgi_probe_state_buffer_->GetSize() != layout.probe_state_byte_size;
  if (reset_probe_state) {
    ddgi_probe_state_buffer_ = CreateDdgiProbeStateBuffer(layout.probe_state_byte_size);
    if (ddgi_probe_state_buffer_) {
      const std::vector<glm::vec4> zero_state(layout.probe_count, glm::vec4(0.0f));
      ddgi_probe_state_buffer_->UploadVector(zero_state);
    }
  }

  const auto skip_inactive_probe_trace = ddgi_ray_source.enable_probe_classification;
  const auto ddgi_update_hysteresis =
      CalculateDdgiUpdateHysteresis(ddgi_settings, ddgi_last_probe_update_reasons_, ddgi_probe_warmup_frame_index_);
  const auto ddgi_update_brightness_threshold =
      ddgi_first_warmup_frame ? (std::numeric_limits<float>::max)()
                              : CalculateDdgiUpdateBrightnessThreshold(ddgi_settings, ddgi_last_probe_update_reasons_);
  ddgi_frame_probe_update_hysteresis_ = ddgi_update_hysteresis;
  const auto ddgi_environment_cubemap_index = GetDdgiEnvironmentCubemapIndex(scene, ddgi_ray_source.first_probe);
  ddgi_frame_ray_push_constant_ = CreateDdgiProbeRayTracingPushConstant(
      ddgi_settings, ddgi_ray_source, ddgi_update_window, skip_inactive_probe_trace, ddgi_first_warmup_frame,
      ddgi_environment_cubemap_index);
  ddgi_frame_probe_update_push_constant_ =
      CreateDdgiProbeAtlasUpdatePushConstant(ddgi_settings, ddgi_update_window, ddgi_total_probe_count, ddgi_ray_source,
                                             ddgi_update_hysteresis, ddgi_update_brightness_threshold);
  ddgi_frame_probe_relocation_reset_push_constant_ = CreateDdgiProbeRelocationPushConstant(
      ddgi_settings, ddgi_update_window, ddgi_total_probe_count, ddgi_ray_source, true);
  ddgi_frame_probe_relocation_update_push_constant_ = CreateDdgiProbeRelocationPushConstant(
      ddgi_settings, ddgi_update_window, ddgi_total_probe_count, ddgi_ray_source, false);
  ddgi_frame_probe_classification_reset_push_constant_ = CreateDdgiProbeClassificationPushConstant(
      ddgi_settings, ddgi_update_window, ddgi_total_probe_count, ddgi_ray_source, true);
  ddgi_frame_probe_classification_update_push_constant_ = CreateDdgiProbeClassificationPushConstant(
      ddgi_settings, ddgi_update_window, ddgi_total_probe_count, ddgi_ray_source, false);
  ApplyDdgiRenderInfo(render_instances->render_info_block, ddgi_settings, ddgi_ray_source,
                      ddgi_frame_ray_push_constant_, ddgi_frame_probe_update_push_constant_);

  const auto has_pending_refresh_work = ddgi_pending_probe_update_count_ != 0u || ddgi_clear_probe_atlas_this_frame_;
  const auto probe_variability_gating_enabled =
      probe_variability_enabled && ddgi_ray_source.enable_probe_variability_gating;
  const auto probe_variability_min_samples =
      static_cast<uint32_t>(glm::max(ddgi_ray_source.probe_variability_min_samples, 0));
  const auto probe_variability_sample_count_complete =
      ddgi_probe_variability_sample_count_ > probe_variability_min_samples;
  const auto probe_variability_below_threshold =
      ddgi_probe_variability_average_ < ddgi_ray_source.probe_variability_threshold;
  ddgi_probe_variability_converged_ = probe_variability_gating_enabled && !reset_ddgi_variability_state &&
                                      !has_pending_refresh_work && !ddgi_frame_probe_warmup_active_ &&
                                      probe_variability_sample_count_complete && probe_variability_below_threshold;
  ddgi_probe_variability_stable_sample_count_ =
      ddgi_probe_variability_converged_ ? kDdgiProbeVariabilityStableSampleCount : 0u;
  if (ddgi_probe_variability_converged_) {
    ddgi_last_probe_update_reasons_ = DdgiUpdateReasonConverged;
    ddgi_frame_probe_update_indices_.clear();
    ddgi_next_probe_update_index_ = 0u;
    ddgi_active_probe_update_reasons_ = DdgiUpdateReasonNone;
    return;
  }

  ddgi_frame_probe_update_indices_.clear();
  ddgi_frame_probe_update_indices_.resize(ddgi_total_probe_count);
  std::iota(ddgi_frame_probe_update_indices_.begin(), ddgi_frame_probe_update_indices_.end(), 0u);
  ddgi_next_probe_update_index_ = 0u;
  ddgi_active_probe_update_reasons_ = DdgiUpdateReasonNone;
  if (!ddgi_probe_update_index_buffer_ ||
      ddgi_probe_update_index_buffer_->GetSize() != layout.probe_update_index_byte_size) {
    ddgi_probe_update_index_buffer_ = CreateDdgiProbeStateBuffer(layout.probe_update_index_byte_size);
  }
  if (!ddgi_probe_update_index_buffer_ || ddgi_frame_probe_update_indices_.empty()) {
    return;
  }
  ddgi_probe_update_index_buffer_->UploadVector(ddgi_frame_probe_update_indices_);
  if (ddgi_settings.debug.enabled && ddgi_settings.debug.visualize_probe_state) {
    if (!ddgi_probe_metadata_readback_buffer_ ||
        ddgi_probe_metadata_readback_buffer_->GetSize() != layout.probe_metadata_byte_size) {
      ddgi_probe_metadata_readback_buffer_ = std::make_shared<Buffer>(layout.probe_metadata_byte_size, true);
      ddgi_probe_debug_metadata_.assign(static_cast<size_t>(layout.probe_count) * 3ull, glm::vec4(0.0f));
      ddgi_probe_debug_metadata_byte_size_ = layout.probe_metadata_byte_size;
    }
    ddgi_probe_debug_metadata_probe_count_ = ddgi_total_probe_count;
    if (ddgi_probe_debug_update_ages_.size() != ddgi_total_probe_count) {
      ddgi_probe_debug_update_ages_.assign(ddgi_total_probe_count, static_cast<float>(ddgi_total_probe_count));
    }
    for (auto& update_age : ddgi_probe_debug_update_ages_) {
      update_age = glm::min(update_age + 1.0f, 65535.0f);
    }
    for (const auto probe_index : ddgi_frame_probe_update_indices_) {
      if (probe_index < ddgi_probe_debug_update_ages_.size()) {
        ddgi_probe_debug_update_ages_[probe_index] = 0.0f;
      }
    }
  }
  if (ddgi_settings.debug.enabled && ddgi_settings.debug.show_rays) {
    const auto selected_logical_probe =
        glm::min(static_cast<uint32_t>(glm::max(ddgi_settings.debug.selected_probe_index, 0)), source_probe_count - 1u);
    const auto selected_probe_grid =
        RenderLayer::GetDdgiProbeGridIndex(ddgi_ray_source.probe_counts, selected_logical_probe);
    const auto selected_physical_probe =
        GetScrolledDdgiProbeIndex(selected_probe_grid, ddgi_probe_scroll_offset_, ddgi_ray_source.probe_counts);
    const auto selected_update_iter = std::find(ddgi_frame_probe_update_indices_.begin(),
                                                ddgi_frame_probe_update_indices_.end(), selected_logical_probe);
    const auto selected_ray_byte_size =
        static_cast<uint64_t>(ray_count) * static_cast<uint64_t>(sizeof(PointCloudSample));
    if (selected_update_iter != ddgi_frame_probe_update_indices_.end() && selected_ray_byte_size != 0u) {
      if (!ddgi_probe_ray_readback_buffer_ || ddgi_probe_ray_readback_buffer_->GetSize() != selected_ray_byte_size) {
        ddgi_probe_ray_readback_buffer_ = std::make_shared<Buffer>(selected_ray_byte_size, true);
        ddgi_probe_debug_ray_samples_.assign(ray_count, {});
      }
      ddgi_frame_selected_probe_ray_local_index_ =
          static_cast<uint32_t>(std::distance(ddgi_frame_probe_update_indices_.begin(), selected_update_iter));
      ddgi_frame_selected_probe_ray_sample_count_ = ray_count;
      ddgi_probe_debug_ray_probe_index_ = selected_logical_probe;
      ddgi_probe_debug_ray_physical_probe_index_ = selected_physical_probe;
      ddgi_probe_debug_ray_sample_count_ = ray_count;
      ddgi_probe_debug_ray_samples_available_ = true;
    }
  }

  if (probe_variability_enabled) {
    ddgi_frame_probe_variability_enabled_ = true;
    ddgi_probe_variability_sample_count_++;
  }
  if (ddgi_frame_probe_warmup_active_) {
    ddgi_probe_warmup_frame_index_ =
        glm::min(ddgi_probe_warmup_frame_index_ + 1u, ddgi_frame_probe_warmup_frame_count_);
  }
  ddgi_frame_probe_relocation_reset_ = reset_probe_state;
  ddgi_frame_probe_relocation_enabled_ = ddgi_ray_source.enable_probe_relocation;
  ddgi_frame_probe_classification_reset_ = reset_probe_state;
  ddgi_frame_probe_classification_enabled_ = ddgi_ray_source.enable_probe_classification;
  ddgi_deferred_scene_readiness_refresh_ = false;
  ddgi_scene_input_settle_frame_count_ = 0;
  ddgi_frame_trace_probe_rays_ = true;
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
    if (const auto reflection_probe = default_environment->reflection_probe.Get<ReflectionProbe>()) {
      default_prefiltered = reflection_probe->GetCubemap();
    }
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
  bind_cubemap(kRasterLightingPrefilteredBinding, camera_info.environmental_prefiltered_index, default_prefiltered);
  return descriptor_set;
}

void RenderLayer::RenderSceneToCameraImmediately(const std::shared_ptr<Scene>& scene,
                                                 const GlobalTransform& camera_global_transform,
                                                 const std::shared_ptr<Camera>& camera) {
  if (!scene || !camera || !Platform::Initialized()) {
    return;
  }

  const auto current_frame_index = Platform::GetCurrentFrameIndex();
  const auto previous_render_instances = render_instances_list_[current_frame_index];
  const bool previous_need_fade = need_fade_;
  render_instances_list_[current_frame_index] = std::make_shared<RenderInstanceStorage>();
  PrepareSceneForRendering(scene, false, false, false, false);
  RenderToCamera(scene, camera_global_transform, camera, true);
  render_instances_list_[current_frame_index] = previous_render_instances;
  need_fade_ = previous_need_fade;
  if (previous_render_instances) {
    BindRenderInstanceStorage(current_frame_index, previous_render_instances);
  }
}

void RenderLayer::RenderAll() {
  const ProfilerScope profiler_scope("RenderLayer::RenderAll", "Render");
  const auto scene = GetScene();
  const auto current_frame_index = Platform::GetCurrentFrameIndex();
  const auto current_render_instances = render_instances_list_[current_frame_index];
  const auto& ddgi_settings = scene ? scene->environment.ddgi_settings : fallback_ddgi_settings_;
  auto& platform = Platform::GetInstance();
  render_graph_transient_resource_stores_.clear();
  if (!ddgi_fallback_probe_state_buffer_) {
    ddgi_fallback_probe_state_buffer_ = CreateDdgiFallbackProbeStateBuffer();
  }
  BindDdgiFallbackLightingDescriptors(lighting_ ? lighting_->lighting_descriptor_set : nullptr,
                                      ddgi_fallback_probe_state_buffer_);
  ddgi_frame_ray_output_visualization_buffer_.reset();
  ddgi_last_performance_stats_ = {};
  ddgi_last_performance_stats_.storage_probe_count = ddgi_frame_resource_layout_.probe_count;
  ddgi_last_performance_stats_.updated_probe_count = static_cast<uint32_t>(ddgi_frame_probe_update_indices_.size());
  ddgi_last_performance_stats_.pending_probe_count = ddgi_pending_probe_update_count_;
  ddgi_last_performance_stats_.selected_ray_sample_count = ddgi_frame_selected_probe_ray_sample_count_;
  ddgi_last_performance_stats_.probe_metadata_byte_size = ddgi_frame_resource_layout_.probe_metadata_byte_size;
  ddgi_last_performance_stats_.probe_state_byte_size = ddgi_frame_resource_layout_.probe_state_byte_size;
  ddgi_last_performance_stats_.ray_output_byte_size = ddgi_frame_resource_layout_.ray_output_byte_size;
  ddgi_last_performance_stats_.variability_atlas_byte_size = ddgi_frame_resource_layout_.variability_atlas_byte_size;
  ddgi_last_performance_stats_.variability_reduction_byte_size =
      ddgi_frame_resource_layout_.variability_reduction_byte_size;
  ddgi_last_performance_stats_.irradiance_atlas_extent = ddgi_frame_resource_layout_.irradiance_atlas.resolution;
  ddgi_last_performance_stats_.visibility_atlas_extent = ddgi_frame_resource_layout_.visibility_atlas.resolution;
  ddgi_last_performance_stats_.variability_atlas_extent = ddgi_frame_resource_layout_.variability_atlas.resolution;
  ddgi_last_performance_stats_.variability_reduction_extent = ddgi_frame_resource_layout_.variability_reduction_extent;
  ddgi_last_performance_stats_.probe_variability_average = ddgi_probe_variability_average_;
  ddgi_last_performance_stats_.probe_variability_sample_count = ddgi_probe_variability_sample_count_;
  ddgi_last_performance_stats_.probe_variability_stable_sample_count = ddgi_probe_variability_stable_sample_count_;
  ddgi_last_performance_stats_.probe_variability_required_stable_sample_count = kDdgiProbeVariabilityStableSampleCount;
  ddgi_last_performance_stats_.probe_variability_converged = ddgi_probe_variability_converged_;
  ddgi_last_performance_stats_.probe_warmup_frame_index = ddgi_frame_probe_warmup_frame_index_;
  ddgi_last_performance_stats_.probe_warmup_frame_count = ddgi_frame_probe_warmup_frame_count_;
  ddgi_last_performance_stats_.probe_warmup_active = ddgi_frame_probe_warmup_active_;
  ddgi_last_performance_stats_.probe_update_hysteresis = ddgi_frame_probe_update_hysteresis_;
  const auto ddgi_perf_probe_counts =
      glm::ivec3(glm::max(current_render_instances->render_info_block.ddgi_probe_counts, glm::vec4(1.0f)));
  ddgi_last_performance_stats_.active_probe_count =
      glm::min(GetDdgiProbeCount(ddgi_perf_probe_counts), ddgi_frame_resource_layout_.probe_count);
  if (ddgi_frame_trace_probe_rays_) {
    ddgi_last_performance_stats_.ray_count = static_cast<uint32_t>(glm::max(ddgi_settings.runtime.ray_count, 1));
    ddgi_last_performance_stats_.ray_sample_count =
        ddgi_last_performance_stats_.updated_probe_count * ddgi_last_performance_stats_.ray_count;
  }
  RenderGraph frame_render_graph;
  RenderGraphTransientResourceStore* active_frame_transient_resources = nullptr;
  AddDefaultFrameResources(frame_render_graph);
  AddAdvancedFrameResources(frame_render_graph);
  const auto use_ddgi_frame_resources = ShouldUseDdgiFrameResources(ddgi_settings);
  if (use_ddgi_frame_resources) {
    const auto clear_ddgi_probe_atlas = ddgi_clear_probe_atlas_this_frame_;
    AddDdgiFrameResources(frame_render_graph, ddgi_frame_resource_layout_);
    frame_render_graph.AddPass(DdgiAtlasPreparePass::CreateDescriptor(),
                               [&, clear_ddgi_probe_atlas](const RenderGraphExecutionContext& context) {
                                 DdgiAtlasPreparePass::Execute(
                                     context,
                                     {clear_ddgi_probe_atlas, &ddgi_last_performance_stats_.atlas_prepare_record_ms});
                               });
  }
  const auto trace_ddgi_probe_rays = ddgi_frame_trace_probe_rays_;
  const auto reset_ddgi_probe_relocation = ddgi_frame_probe_relocation_reset_;
  const auto relocate_ddgi_probes = ddgi_frame_probe_relocation_enabled_;
  const auto reset_ddgi_probe_classification = ddgi_frame_probe_classification_reset_;
  const auto classify_ddgi_probes = ddgi_frame_probe_classification_enabled_;
  const auto reduce_ddgi_probe_variability = ddgi_frame_probe_variability_enabled_;
  const auto ddgi_ray_push_constant = ddgi_frame_ray_push_constant_;
  const auto ddgi_probe_update_push_constant = ddgi_frame_probe_update_push_constant_;
  const auto ddgi_probe_relocation_reset_push_constant = ddgi_frame_probe_relocation_reset_push_constant_;
  const auto ddgi_probe_relocation_update_push_constant = ddgi_frame_probe_relocation_update_push_constant_;
  const auto ddgi_probe_classification_reset_push_constant = ddgi_frame_probe_classification_reset_push_constant_;
  const auto ddgi_probe_classification_update_push_constant = ddgi_frame_probe_classification_update_push_constant_;
  const DdgiProbeVariabilityPass::DdgiAtlasLayout ddgi_probe_variability_layout{
      ddgi_frame_resource_layout_.probe_count, ddgi_frame_resource_layout_.variability_atlas.tile_resolution,
      ddgi_frame_resource_layout_.variability_atlas.columns, ddgi_frame_resource_layout_.variability_atlas.resolution,
      ddgi_frame_resource_layout_.variability_reduction_extent};
  const auto ddgi_probe_metadata_readback_buffer =
      ddgi_settings.debug.enabled && ddgi_settings.debug.visualize_probe_state ? ddgi_probe_metadata_readback_buffer_
                                                                               : nullptr;
  const auto ddgi_probe_ray_readback_buffer =
      ddgi_settings.debug.enabled && ddgi_settings.debug.show_rays && ddgi_probe_debug_ray_samples_available_
          ? ddgi_probe_ray_readback_buffer_
          : nullptr;
  if (trace_ddgi_probe_rays) {
    AddDdgiRayTracingFrameResources(frame_render_graph);
    frame_render_graph.AddPass(
        DdgiRayDiagnosticsPass::CreateDescriptor(),
        [&, ddgi_ray_push_constant](const RenderGraphExecutionContext& context) {
          DdgiRayDiagnosticsPass::Execute(
              context, {ddgi_probe_ray_diagnostic_pipeline_, per_frame_descriptor_sets_[current_frame_index],
                        ray_tracing_descriptor_sets_[current_frame_index], ddgi_probe_ray_output_layout_,
                        active_frame_transient_resources, ddgi_atlas_sampler_, ddgi_ray_push_constant,
                        &ddgi_last_performance_stats_.ray_diagnostics_record_ms});
        });
    frame_render_graph.AddPass(
        DdgiProbeUpdatePass::CreateDescriptor(),
        [&, ddgi_probe_update_push_constant](const RenderGraphExecutionContext& context) {
          DdgiProbeUpdatePass::Execute(
              context,
              {ddgi_probe_update_pipeline_, per_frame_descriptor_sets_[current_frame_index], ddgi_probe_update_layout_,
               active_frame_transient_resources, ddgi_probe_update_push_constant, ddgi_probe_metadata_readback_buffer,
               ddgi_probe_ray_readback_buffer, ddgi_frame_selected_probe_ray_local_index_,
               ddgi_frame_selected_probe_ray_sample_count_, &ddgi_last_performance_stats_.probe_update_record_ms});
        });
    if (reset_ddgi_probe_relocation || relocate_ddgi_probes) {
      frame_render_graph.AddPass(
          DdgiProbeRelocationPass::CreateDescriptor(),
          [&, ddgi_probe_relocation_reset_push_constant, ddgi_probe_relocation_update_push_constant,
           reset_ddgi_probe_relocation, relocate_ddgi_probes](const RenderGraphExecutionContext& context) {
            DdgiProbeRelocationPass::Execute(
                context, {ddgi_probe_relocation_pipeline_, ddgi_probe_relocation_layout_,
                          active_frame_transient_resources, ddgi_probe_relocation_reset_push_constant,
                          ddgi_probe_relocation_update_push_constant, reset_ddgi_probe_relocation, relocate_ddgi_probes,
                          &ddgi_last_performance_stats_.probe_relocation_record_ms});
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
                          classify_ddgi_probes, &ddgi_last_performance_stats_.probe_classification_record_ms});
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
                 ddgi_variability_readback_buffer_, &ddgi_last_performance_stats_.probe_variability_record_ms});
          });
    }
  }
  AddExternalRenderResources(frame_render_graph, external_render_resource_descriptors);
  for (const auto& external_pass : frame_render_pass_external_functions) {
    ImportMissingPassResources(frame_render_graph, external_pass.descriptor);
    frame_render_graph.AddPass(
        external_pass.descriptor, [&, external_pass](const RenderGraphExecutionContext& context) {
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
  if (!frame_render_graph.Validate()) {
    EVOENGINE_ERROR("Invalid frame render graph.")
  }
  const auto frame_render_graph_plan = frame_render_graph.Compile(CreateFrameRenderGraphCompileContext());
  auto frame_render_graph_resources =
      CreateFrameRenderGraphResourceRegistry(per_frame_descriptor_sets_[current_frame_index]);
  if (use_ddgi_frame_resources) {
    frame_render_graph_resources.BindBuffer(RenderResourceNames::frame_ddgi_probe_metadata,
                                            ddgi_probe_metadata_buffer_);
    frame_render_graph_resources.BindBuffer(
        RenderResourceNames::frame_ddgi_probe_state,
        ddgi_probe_state_buffer_ ? ddgi_probe_state_buffer_ : ddgi_fallback_probe_state_buffer_);
    frame_render_graph_resources.BindImage(RenderResourceNames::frame_ddgi_irradiance_atlas, ddgi_irradiance_atlas_);
    frame_render_graph_resources.BindImage(RenderResourceNames::frame_ddgi_visibility_atlas, ddgi_visibility_atlas_);
    frame_render_graph_resources.BindImage(RenderResourceNames::frame_ddgi_variability_atlas, ddgi_variability_atlas_);
  }
  if (trace_ddgi_probe_rays) {
    frame_render_graph_resources.BindDescriptorSet(RenderResourceNames::frame_ray_tracing_descriptor_set,
                                                   ray_tracing_descriptor_sets_[current_frame_index]);
    frame_render_graph_resources.BindBuffer(RenderResourceNames::frame_ddgi_probe_state, ddgi_probe_state_buffer_);
    frame_render_graph_resources.BindBuffer(RenderResourceNames::frame_ddgi_probe_update_indices,
                                            ddgi_probe_update_index_buffer_);
  }
  auto& frame_transient_resources = render_graph_transient_resource_stores_.emplace_back();
  active_frame_transient_resources = &frame_transient_resources;
  frame_transient_resources.Allocate(frame_render_graph.GetResources(), frame_render_graph_plan);
  frame_transient_resources.Bind(frame_render_graph_resources);
  const auto ddgi_frame_graph_timer = DdgiPerformanceClock::now();
  frame_render_graph.Execute(frame_render_graph_plan, frame_render_graph_resources);
  ddgi_last_performance_stats_.frame_graph_execute_ms = DdgiElapsedMilliseconds(ddgi_frame_graph_timer);
  if (use_ddgi_frame_resources) {
    BindDdgiAtlasLightingDescriptors(frame_render_graph_resources, frame_transient_resources,
                                     lighting_ ? lighting_->lighting_descriptor_set : nullptr, ddgi_atlas_sampler_,
                                     ddgi_fallback_probe_state_buffer_);
  }
  if (trace_ddgi_probe_rays) {
    if (ddgi_settings.debug.enabled && ddgi_settings.debug.show_rays) {
      if (const auto* ray_output_binding =
              frame_render_graph_resources.GetResourceBinding(RenderResourceNames::frame_ddgi_ray_output);
          ray_output_binding && ray_output_binding->buffer) {
        ddgi_frame_ray_output_visualization_buffer_ = ray_output_binding->buffer;
      }
    }
  }
  active_frame_transient_resources = nullptr;
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
                gizmos_pipeline->PushConstant(vk_command_buffer, 0, push_constant);
                GeometryStorage::BindVertices(vk_command_buffer);
                i.mesh->DrawIndexed(vk_command_buffer, gizmos_pipeline->states, 1);
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
                gizmos_instanced_colored->PushConstant(vk_command_buffer, 0, push_constant);
                GeometryStorage::BindVertices(vk_command_buffer);
                i.mesh->DrawIndexed(vk_command_buffer, gizmos_instanced_colored->states,
                                    i.particle_info_list->PeekParticleInfoList().size());
              });
        });
      }
    }
#ifdef EVOENGINE_WINDOWS
    for (const auto& i : editor_layer->gizmo_strands_tasks_) {
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
              gizmos_pipeline = gizmos_strands;
            } break;
            case GizmoSettings::ColorMode::VertexColor: {
              gizmos_pipeline = gizmos_strands_vertex_colored;
            } break;
            case GizmoSettings::ColorMode::NormalColor: {
              gizmos_pipeline = gizmos_strands_normal_colored;
            } break;
          }
          i.editor_camera_component->GetRenderTexture()->ApplyGraphicsPipelineStates(gizmos_pipeline->states);
          i.gizmo_settings.ApplySettings(gizmos_pipeline->states);

          gizmos_pipeline->Bind(vk_command_buffer);
          gizmos_pipeline->BindDescriptorSet(vk_command_buffer, 0,
                                             per_frame_descriptor_sets_[current_frame_index]->GetVkDescriptorSet());

          i.editor_camera_component->GetRenderTexture()->Render(
              vk_command_buffer, VK_ATTACHMENT_LOAD_OP_LOAD, VK_ATTACHMENT_STORE_OP_STORE, [&] {
                GizmosPushConstant push_constant;
                push_constant.model = i.model;
                push_constant.color = i.color;
                push_constant.size = i.size;
                push_constant.camera_index =
                    current_render_instances->GetCameraIndex(i.editor_camera_component->GetHandle());
                gizmos_pipeline->PushConstant(vk_command_buffer, 0, push_constant);
                GeometryStorage::BindStrandPoints(vk_command_buffer);
                i.strands->DrawIndexed(vk_command_buffer, gizmos_pipeline->states, 1);
              });
        });
      }
    }
#endif
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
}

void RenderLayer::PreparePointAndSpotLightShadowMap() const {
  const bool count_draw_calls = count_shadow_rendering_draw_calls;
  const bool use_mesh_shader = Platform::MeshShaderEnabled() && enable_meshlet;
  const auto current_frame_index = Platform::GetCurrentFrameIndex();
  const auto& point_light_shadow_opaque_pipeline =
      use_mesh_shader ? point_light_shadow_pipeline_mesh_shader : point_light_shadow_pipeline_normal_opaque;
  const auto& spot_light_shadow_opaque_pipeline =
      use_mesh_shader ? spot_light_shadow_pipeline_mesh_shader : spot_light_shadow_pipeline_normal_opaque;
  const auto current_render_instances = render_instances_list_[current_frame_index];
  Platform::RecordCommandsMainQueue([&](const VkCommandBuffer vk_command_buffer) {
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
            const auto prim_count = render_instance->Render(vk_command_buffer, push_constant, pipeline);
            account_draw(bucket, RenderDrawCallKind::Direct, prim_count);
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
#ifdef EVOENGINE_WINDOWS
          GeometryStorage::BindStrandPoints(vk_command_buffer);
          {
            render_strands_shadow_collection(
                RenderPassDrawBucket::PointLightShadow, current_render_instances->deferred_strands_render_instances,
                strands_point_light_shadow_pipeline, light_space_matrix, i, face, point_light_info_block.viewport);
          }
#endif
          for (const auto& func : point_light_shadow_map_external_functions) {
            const auto prim_count = func(vk_command_buffer, {i, face, point_light_info_block.viewport});
            account_draw(RenderPassDrawBucket::PointLightShadow, RenderDrawCallKind::Direct, prim_count);
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
#ifdef EVOENGINE_WINDOWS
        GeometryStorage::BindStrandPoints(vk_command_buffer);
        {
          render_strands_shadow_collection(
              RenderPassDrawBucket::SpotLightShadow, current_render_instances->deferred_strands_render_instances,
              strands_spot_light_shadow_pipeline, light_space_matrix, i, 0, spot_light_info_block.viewport);
        }
#endif
        for (const auto& func : spot_light_shadow_map_external_functions) {
          const auto prim_count = func(vk_command_buffer, {i, spot_light_info_block.viewport});
          account_draw(RenderPassDrawBucket::SpotLightShadow, RenderDrawCallKind::Direct, prim_count);
        }
      }
    });
    lighting_->point_light_shadow_map_->TransitImageLayout(vk_command_buffer, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
    lighting_->spot_light_shadow_map_->TransitImageLayout(vk_command_buffer, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
  });
}

bool RenderLayer::UpdateRenderInstanceStorage(const std::shared_ptr<Scene>& scene, const uint32_t current_frame_index,
                                              const bool include_editor_cameras, const bool update_editor_selection,
                                              const bool track_ddgi_scene_inputs) {
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
  if (!lod_set) {
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
  current_render_instances->BuildFromScene(render_settings, scene, world_bound, include_editor_cameras);
  const auto previous_render_instances =
      render_instances_list_[(current_frame_index + Platform::GetMaxFramesInFlight() - 1) %
                             Platform::GetMaxFramesInFlight()];
  bool render_instance_updated = false;
  if (track_ddgi_scene_inputs) {
    const auto current_render_info = current_render_instances->render_info_block;
    PreserveDdgiRenderInfo(current_render_instances->render_info_block, previous_render_instances->render_info_block);
    ddgi_scene_material_inputs_changed_ = false;
    const auto blocks_changed = [](const auto& current_blocks, const auto& previous_blocks) {
      if (current_blocks.size() != previous_blocks.size()) {
        return true;
      }
      for (size_t i = 0; i < current_blocks.size(); ++i) {
        if (current_blocks[i] != previous_blocks[i]) {
          return true;
        }
      }
      return false;
    };
    auto active_light_keys = CollectDdgiActiveLightKeys(scene);
    auto light_signatures = CollectDdgiLightSignatures(scene);
    std::vector<uint64_t> geometry_signatures;
    const auto collect_ddgi_geometry_signatures = [&](const auto& collection) {
      if (!collection) {
        return;
      }
      collection->ForEachRenderInstance([&](const auto& render_instance) {
        geometry_signatures.push_back(MakeDdgiGeometrySignature(render_instance));
      });
    };
    collect_ddgi_geometry_signatures(current_render_instances->deferred_render_instances);
    collect_ddgi_geometry_signatures(current_render_instances->deferred_skinned_render_instances);
    collect_ddgi_geometry_signatures(current_render_instances->deferred_instanced_render_instances);
    collect_ddgi_geometry_signatures(current_render_instances->deferred_strands_render_instances);
    collect_ddgi_geometry_signatures(current_render_instances->forward_render_instances);
    collect_ddgi_geometry_signatures(current_render_instances->forward_skinned_render_instances);
    collect_ddgi_geometry_signatures(current_render_instances->forward_instanced_render_instances);
    collect_ddgi_geometry_signatures(current_render_instances->forward_strands_render_instances);
    collect_ddgi_geometry_signatures(current_render_instances->transparent_render_instances);
    collect_ddgi_geometry_signatures(current_render_instances->transparent_skinned_render_instances);
    collect_ddgi_geometry_signatures(current_render_instances->transparent_instanced_render_instances);
    collect_ddgi_geometry_signatures(current_render_instances->transparent_strands_render_instances);
    collect_ddgi_geometry_signatures(current_render_instances->external_render_instances);
    std::sort(geometry_signatures.begin(), geometry_signatures.end());
    const bool had_previous_scene_inputs = ddgi_has_previous_scene_inputs_;
    bool scene_render_inputs_updated =
        current_render_instances->render_info_block != previous_render_instances->render_info_block;
    ddgi_scene_change_triggers_ = DdgiVolumeTriggerConditionNone;
    if (had_previous_scene_inputs) {
      if (active_light_keys != ddgi_previous_active_light_keys_) {
        ddgi_scene_change_triggers_ |= DdgiVolumeTriggerConditionLightEnableChanged;
      }
      if (current_render_instances->environment_info_block != ddgi_previous_environment_info_block_ ||
          light_signatures != ddgi_previous_light_signatures_) {
        ddgi_scene_change_triggers_ |= DdgiVolumeTriggerConditionLightingConditionChanged;
      }
      ddgi_scene_material_inputs_changed_ =
          blocks_changed(current_render_instances->GetGltfShadeMaterials(), ddgi_previous_gltf_shade_materials_) ||
          blocks_changed(current_render_instances->GetGltfTextureInfos(), ddgi_previous_gltf_texture_infos_) ||
          current_render_instances->texture_storage_version != ddgi_previous_texture_storage_version_;
      if (ddgi_scene_material_inputs_changed_ || geometry_signatures != ddgi_previous_geometry_signatures_) {
        ddgi_scene_change_triggers_ |= DdgiVolumeTriggerConditionGeometryChanged;
      }
      scene_render_inputs_updated =
          scene_render_inputs_updated || ddgi_scene_change_triggers_ != DdgiVolumeTriggerConditionNone;
    } else {
      scene_render_inputs_updated = true;
      ddgi_deferred_scene_readiness_refresh_ = true;
      ddgi_scene_input_settle_frame_count_ = 0;
    }
    ddgi_has_previous_scene_inputs_ = true;
    ddgi_previous_environment_info_block_ = current_render_instances->environment_info_block;
    ddgi_previous_gltf_shade_materials_ = current_render_instances->GetGltfShadeMaterials();
    ddgi_previous_gltf_texture_infos_ = current_render_instances->GetGltfTextureInfos();
    ddgi_previous_texture_storage_version_ = current_render_instances->texture_storage_version;
    ddgi_previous_active_light_keys_ = std::move(active_light_keys);
    ddgi_previous_light_signatures_ = std::move(light_signatures);
    ddgi_previous_geometry_signatures_ = std::move(geometry_signatures);
    render_instance_updated = scene_render_inputs_updated;
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
      }
    }
  } else {
    for (const auto& camera_entry : current_render_instances->cameras) {
      if (const auto& camera = camera_entry.second; camera && camera_info_changed(camera)) {
        camera->frame_count_ = 0;
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
      ShaderType::Vertex, Resources::GetDefaultResourcesPath() / "Shaders/Graphics/Vertex/TexturePassThrough.vert");
  environmental_brdf_pipeline->fragment_shader =
      Shader::CreateTemporary(ShaderType::Fragment, Resources::GetDefaultResourcesPath() /
                                                        "Shaders/Graphics/Fragment/Lighting/EnvironmentalMapBrdf.frag");
  environmental_brdf_pipeline->geometry_type = GeometryType::Mesh;
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
                                 const std::shared_ptr<Camera>& camera, const bool immediate) const {
  const ProfilerScope profiler_scope("RenderLayer::RenderToCamera", "Render");
  const auto current_frame_index = Platform::GetCurrentFrameIndex();
  const auto current_render_instances = render_instances_list_[current_frame_index];
  const int camera_index = current_render_instances->GetCameraIndex(camera->GetHandle());
  const auto raster_lighting_texture_descriptor_set =
      GetRasterLightingTextureDescriptorSet(current_frame_index, camera_index, current_render_instances);
  const auto editor_layer = ApplicationContext::Get().GetLayer<EditorLayer>();
  const bool is_scene_camera = editor_layer && camera.get() == editor_layer->GetSceneCamera().get();
  VolumetricCloudSettings volumetric_cloud_settings{};
  if (scene) {
    volumetric_cloud_settings = scene->environment.volumetric_cloud_settings;
    volumetric_cloud_settings.ClampSettings();
  }
  const bool volumetric_clouds_enabled = volumetric_cloud_settings.enabled;
  const auto& ddgi_settings = scene ? scene->environment.ddgi_settings : fallback_ddgi_settings_;
  const auto render_info_probe_counts =
      glm::ivec3(glm::max(current_render_instances->render_info_block.ddgi_probe_counts, glm::vec4(1.0f)));
  const auto ddgi_visualization_probe_count =
      glm::min(GetDdgiProbeCount(render_info_probe_counts), ddgi_frame_resource_layout_.probe_count);
  const bool ddgi_probe_visualization_enabled =
      is_scene_camera && ShouldRenderDdgiProbeVisualization(ddgi_settings) && ddgi_probe_metadata_buffer_ &&
      ddgi_probe_metadata_buffer_->GetSize() >= ddgi_frame_resource_layout_.probe_metadata_byte_size &&
      ddgi_probe_state_buffer_ &&
      ddgi_probe_state_buffer_->GetSize() >= ddgi_frame_resource_layout_.probe_state_byte_size &&
      ddgi_irradiance_atlas_ && ddgi_visibility_atlas_ && ddgi_visualization_probe_count != 0u;
  DdgiProbeVisualizationPushConstant ddgi_probe_visualization_push_constant;
  ddgi_probe_visualization_push_constant.camera_probe_selected_mode = {
      static_cast<uint32_t>(glm::max(camera_index, 0)), ddgi_visualization_probe_count,
      glm::min(static_cast<uint32_t>(glm::max(ddgi_settings.debug.selected_probe_index, 0)),
               ddgi_visualization_probe_count > 0u ? ddgi_visualization_probe_count - 1u : 0u),
      GetDdgiProbeVisualizationMode(ddgi_settings)};
  ddgi_probe_visualization_push_constant.radius_intensity_alpha_selected_scale = {
      glm::max(ddgi_settings.debug.probe_visualization_radius, 0.001f) *
          glm::max(ddgi_settings.debug.visualization_scale, 0.01f),
      glm::max(ddgi_settings.debug.probe_visualization_intensity, 0.0f),
      glm::clamp(ddgi_settings.debug.probe_visualization_alpha, 0.0f, 1.0f),
      ddgi_settings.debug.visualize_selected_probe
          ? glm::max(ddgi_settings.debug.selected_probe_visualization_scale, 1.0f)
          : 1.0f};
  const bool ddgi_probe_visualization_depth_test = ddgi_settings.debug.probe_visualization_depth_mode == 0;
  const auto selected_ray_local_probe_index = ddgi_frame_selected_probe_ray_local_index_;
  const auto selected_ray_sample_count = ddgi_frame_selected_probe_ray_sample_count_;
  const auto selected_ray_byte_size =
      static_cast<uint64_t>(selected_ray_sample_count) * static_cast<uint64_t>(sizeof(PointCloudSample));
  const auto selected_ray_src_offset =
      selected_ray_local_probe_index == (std::numeric_limits<uint32_t>::max)()
          ? 0ull
          : static_cast<uint64_t>(selected_ray_local_probe_index) * selected_ray_byte_size;
  const bool ddgi_probe_ray_visualization_enabled =
      is_scene_camera && ddgi_settings.debug.enabled && ddgi_settings.debug.show_rays &&
      ddgi_frame_ray_output_visualization_buffer_ && selected_ray_sample_count != 0u &&
      selected_ray_local_probe_index != (std::numeric_limits<uint32_t>::max)() &&
      ddgi_frame_ray_output_visualization_buffer_->GetSize() >= selected_ray_src_offset + selected_ray_byte_size;
  const auto ray_visualization_miss_distance = ddgi_frame_ray_push_constant_.trace_parameters.x > 0.0f
                                                   ? ddgi_frame_ray_push_constant_.trace_parameters.x
                                                   : ddgi_settings.runtime.max_ray_distance;
  DdgiProbeRayVisualizationPushConstant ddgi_probe_ray_visualization_push_constant;
  ddgi_probe_ray_visualization_push_constant.camera_selected_probe_ray_count_flags = {
      static_cast<uint32_t>(glm::max(camera_index, 0)), selected_ray_local_probe_index, selected_ray_sample_count, 0u};
  ddgi_probe_ray_visualization_push_constant.miss_distance_alpha_padding = {
      glm::max(ray_visualization_miss_distance, 0.001f),
      glm::clamp(ddgi_settings.debug.probe_visualization_alpha, 0.0f, 1.0f), 0.0f, 0.0f};
  const auto record_commands = [&](const std::function<void(VkCommandBuffer vk_command_buffer)>& action) {
    if (immediate) {
      Platform::ImmediateSubmit(action);
    } else {
      Platform::RecordCommandsMainQueue(action);
    }
  };
  if (Camera::ResolveCameraRenderMode(camera->camera_render_mode) == Camera::CameraRenderMode::Rasterization) {
    const auto& graphics_settings = ApplicationContext::Get().GetApplicationInfo().graphics_settings;

    const bool count_draw_calls = count_shadow_rendering_draw_calls;
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
    AddExternalRenderResources(camera_render_graph, external_render_resource_descriptors);
    if (ddgi_probe_visualization_enabled) {
      AddDdgiProbeVisualizationFrameResources(camera_render_graph, ddgi_frame_resource_layout_);
    }
    if (ddgi_probe_ray_visualization_enabled) {
      AddDdgiProbeRayVisualizationFrameResources(camera_render_graph, ddgi_frame_resource_layout_);
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
               strands_directional_light_shadow_pipeline,
               per_frame_descriptor_sets_[current_frame_index],
               meshlet_descriptor_sets_[current_frame_index],
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
                   const glm::ivec4& viewport) {
                 for (const auto& func : directional_light_shadow_map_external_functions) {
                   const auto prim_count = func(vk_command_buffer, {light_index, split_index, viewport});
                   if (count_draw_calls) {
                     Platform::CountRenderPassDraw(RenderPassDrawBucket::DirectionalLightShadow,
                                                   RenderDrawCallKind::Direct, current_frame_index, prim_count);
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
               skinned_deferred_prepass_pipeline, strands_deferred_prepass_pipeline,
               raster_material_per_frame_descriptor_sets_[current_frame_index],
               meshlet_descriptor_sets_[current_frame_index], camera_index, current_frame_index, use_mesh_shader,
               enable_indirect_rendering, true, count_draw_calls, wire_frame,
               [&](const VkCommandBuffer vk_command_buffer,
                   const std::vector<VkRenderingAttachmentInfo>& color_attachment_infos, const glm::ivec4& viewport) {
                 for (const auto& func : deferred_rendering_external_functions) {
                   const auto prim_count = func(vk_command_buffer, color_attachment_infos, {camera_index, viewport});
                   if (count_draw_calls) {
                     Platform::CountRenderPassDraw(RenderPassDrawBucket::DeferredGeometry, RenderDrawCallKind::Direct,
                                                   current_frame_index, prim_count);
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
               active_camera_transient_resources, camera_index, wire_frame, record_commands});
        });
    camera_render_graph.AddPass(DepthPyramidPass::CreateDescriptor(), [&](const RenderGraphExecutionContext& context) {
      DepthPyramidPass::Execute(context, {camera, depth_pyramid_pipeline_, depth_pyramid_layout_,
                                          active_camera_transient_resources, record_commands});
    });
    camera_render_graph.AddPass(
        DeferredLightingPass::CreateDescriptor(), [&](const RenderGraphExecutionContext& context) {
          const bool fade_selection = need_fade_ && editor_layer && editor_layer->highlight_selection_;
          const int selection_alpha = editor_layer ? editor_layer->selection_alpha_ : 0;
          const auto& deferred_lighting_pipeline =
              is_scene_camera ? deferred_lighting_pass_pipeline_scene_camera : deferred_lighting_pass_pipeline;
          DeferredLightingPass::Execute(
              context,
              {camera, deferred_lighting_pipeline, raster_material_per_frame_descriptor_sets_[current_frame_index],
               lighting_ ? lighting_->lighting_descriptor_set : nullptr, raster_lighting_texture_descriptor_set,
               camera_index, current_frame_index, count_draw_calls, fade_selection, selection_alpha,
               [&](const VkCommandBuffer vk_command_buffer, const glm::ivec4& viewport) {
                 for (const auto& func : forward_rendering_external_functions) {
                   const auto prim_count = func(vk_command_buffer, camera, {camera_index, viewport});
                   if (count_draw_calls) {
                     Platform::CountRenderPassDraw(RenderPassDrawBucket::ForwardExternal, RenderDrawCallKind::Direct,
                                                   current_frame_index, prim_count);
                   }
                 }
               },
               record_commands});
        });
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
                prim_count = external_pass.context_func(vk_command_buffer, camera, {camera_index, view_port}, context);
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
    const bool gaussian_splat_rendering_enabled = current_render_instances &&
                                                  current_render_instances->total_gaussian_splats != 0u &&
                                                  current_render_instances->gaussian_splat_render_instances &&
                                                  !current_render_instances->gaussian_splat_render_instances->Empty();
    const bool transparent_mesh_rendering_enabled = current_render_instances &&
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
                context,
                {camera, current_render_instances, transparent_geometry_pipeline_normal,
                 raster_material_per_frame_descriptor_sets_[current_frame_index],
                 lighting_ ? lighting_->lighting_descriptor_set : nullptr, raster_lighting_texture_descriptor_set,
                 camera_index, current_frame_index, count_draw_calls, wire_frame, record_commands});
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
                          camera, static_cast<uint32_t>(glm::max(camera_index, 0)), ddgi_visualization_probe_count,
                          ddgi_probe_visualization_depth_test, ddgi_probe_visualization_push_constant,
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
                          static_cast<uint32_t>(glm::max(camera_index, 0)), ddgi_probe_visualization_depth_test,
                          ddgi_probe_ray_visualization_push_constant,
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
                                               ddgi_probe_metadata_buffer_);
      camera_render_graph_resources.BindBuffer(RenderResourceNames::frame_ddgi_probe_state, ddgi_probe_state_buffer_);
      camera_render_graph_resources.BindImage(RenderResourceNames::frame_ddgi_irradiance_atlas, ddgi_irradiance_atlas_);
      camera_render_graph_resources.BindImage(RenderResourceNames::frame_ddgi_visibility_atlas, ddgi_visibility_atlas_);
    }
    if (ddgi_probe_ray_visualization_enabled) {
      camera_render_graph_resources.BindBuffer(RenderResourceNames::frame_ddgi_ray_output,
                                               ddgi_frame_ray_output_visualization_buffer_);
    }
    auto& camera_transient_resources = render_graph_transient_resource_stores_.emplace_back();
    active_camera_transient_resources = &camera_transient_resources;
    camera_transient_resources.Allocate(camera_render_graph.GetResources(), camera_render_graph_plan);
    camera_transient_resources.Bind(camera_render_graph_resources);
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
    if (ray_camera_shader_variant_cache_) {
      ray_camera_shader_variant_cache_->RecordFallbackFrame(use_ray_query ? RayCameraShaderTechnique::RayQuery
                                                                          : RayCameraShaderTechnique::RayTracing);
    }
    const char* ray_camera_pass_name =
        use_ray_query ? RenderPassNames::ray_query_camera : RenderPassNames::ray_tracing_camera;
    VolumetricCloudSettings volumetric_cloud_settings{};
    if (scene) {
      volumetric_cloud_settings = scene->environment.volumetric_cloud_settings;
      volumetric_cloud_settings.ClampSettings();
    }
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
                          ray_tracing_descriptor_sets_[current_frame_index], camera_index, camera->frame_count_,
                          record_commands, ray_tracing_camera_output_layout_, active_camera_transient_resources});
          });
    } else {
      camera_render_graph.AddPass(
          RayTracingCameraPass::CreateDescriptor(), [&](const RenderGraphExecutionContext& context) {
            RayTracingCameraPass::Execute(
                context, {camera, ray_tracing_pipeline, per_frame_descriptor_sets_[current_frame_index],
                          ray_tracing_descriptor_sets_[current_frame_index], camera_index, camera->frame_count_,
                          record_commands, ray_tracing_camera_output_layout_, active_camera_transient_resources});
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
                                  PostProcessingPass::Execute(context, {camera, nullptr, false, true});
                                });
    if (!camera_render_graph.Validate()) {
      EVOENGINE_ERROR("Invalid ray tracing camera render graph.")
    }
    const auto camera_render_graph_plan = camera_render_graph.Compile(CreateCameraRenderGraphCompileContext(camera));
    auto camera_render_graph_resources = CreateCameraRenderGraphResourceRegistry(
        per_frame_descriptor_sets_[current_frame_index], ray_tracing_descriptor_sets_[current_frame_index], camera);
    auto& camera_transient_resources = render_graph_transient_resource_stores_.emplace_back();
    active_camera_transient_resources = &camera_transient_resources;
    camera_transient_resources.Allocate(camera_render_graph.GetResources(), camera_render_graph_plan);
    camera_transient_resources.Bind(camera_render_graph_resources);
    const ScopedRenderCameraDrawScope camera_draw_scope(current_frame_index, scene, camera, is_scene_camera);
    camera_render_graph.Execute(camera_render_graph_plan, camera_render_graph_resources);
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
  return ApplicationContext::Get().GetLayer<RenderLayer>()->lighting_->lighting_descriptor_set;
}
