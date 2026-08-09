#pragma once
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include "CameraSettings.hpp"

namespace evo_engine {
class Buffer;
class DescriptorSet;
class Image;
class ImageView;
class IAsset;
class RenderTexture;
class Sampler;

enum class RenderPassQueue { Graphics, Compute, RayTracing };

enum class RenderPassScope { Frame, Camera };

enum class RenderResourceType { Buffer, Image, DescriptorSet, AccelerationStructure, External };

enum class RenderResourceLifetime { Frame, Camera, Persistent, History, Imported };

enum class RenderResourceUsage { Read, Write, ReadWrite };

enum class RenderResourceSizeMode { None, Absolute, FrameRelative, CameraRelative };

enum class RenderGraphBarrierType { ImageLayout, ImageMemory, BufferMemory, GlobalMemory };

enum class RenderResourceState {
  Undefined,
  ColorAttachment,
  DepthAttachment,
  ShaderRead,
  StorageReadWrite,
  Present,
  TransferSource,
  TransferDestination,
  TransferDestinationGeneral,
  AccelerationStructureRead,
  General
};

struct RenderResourceDimensions {
  RenderResourceSizeMode size_mode = RenderResourceSizeMode::None;
  uint32_t width = 0;
  uint32_t height = 0;
  uint32_t depth = 1;
  uint32_t layers = 1;
  uint32_t mip_levels = 1;
};

struct RenderGraphCompileContext {
  uint32_t frame_width = 0;
  uint32_t frame_height = 0;
  uint32_t camera_width = 0;
  uint32_t camera_height = 0;
};

struct RenderResourceDescriptor {
  std::string name;
  RenderResourceType type = RenderResourceType::External;
  RenderResourceLifetime lifetime = RenderResourceLifetime::Frame;
  RenderResourceDimensions dimensions;
  std::string format_name;
  uint32_t sample_count = 1;
  uint32_t history_length = 1;
  bool managed_by_graph = false;
  uint64_t byte_size = 0;
};

struct RenderResourceAccess {
  std::string resource_name;
  RenderResourceUsage usage = RenderResourceUsage::Read;
  RenderResourceState state = RenderResourceState::Undefined;
};

struct RenderPassDescriptor {
  std::string name;
  RenderPassQueue queue = RenderPassQueue::Graphics;
  RenderPassScope scope = RenderPassScope::Frame;
  std::vector<RenderResourceAccess> resources;
  std::vector<std::string> dependencies;
};

namespace RenderResourceNames {
inline constexpr const char* frame_render_instances = "Frame.RenderInstances";
inline constexpr const char* frame_per_frame_descriptor_set = "Frame.PerFrameDescriptorSet";
inline constexpr const char* frame_ray_tracing_descriptor_set = "Frame.RayTracingDescriptorSet";
inline constexpr const char* scene_mesh_tlas = "Scene.MeshTLAS";
inline constexpr const char* lighting_directional_shadow_map = "Lighting.DirectionalShadowMap";
inline constexpr const char* camera_depth = "Camera.Depth";
inline constexpr const char* camera_g_buffer = "Camera.GBuffer";
inline constexpr const char* camera_color = "Camera.Color";
inline constexpr const char* camera_motion_vectors = "Camera.MotionVectors";
inline constexpr const char* camera_object_id = "Camera.ObjectId";
inline constexpr const char* camera_material_id = "Camera.MaterialId";
inline constexpr const char* camera_depth_pyramid = "Camera.DepthPyramid";
inline constexpr const char* camera_ambient_occlusion = "Camera.AmbientOcclusion";
inline constexpr const char* camera_ambient_occlusion_scratch = "Camera.AmbientOcclusionScratch";
inline constexpr const char* camera_ddgi_gather_timing = "Camera.DDGI.GatherTiming";
inline constexpr const char* camera_color_history = "Camera.ColorHistory";
inline constexpr const char* camera_radiance_history = "Camera.RadianceHistory";
inline constexpr const char* camera_ray_hit_distance = "Camera.RayHitDistance";
inline constexpr const char* camera_ray_albedo = "Camera.RayAlbedo";
inline constexpr const char* camera_ray_normal = "Camera.RayNormal";
inline constexpr const char* camera_ray_count = "Camera.RayCount";
inline constexpr const char* camera_ray_path_length = "Camera.RayPathLength";
inline constexpr const char* camera_ray_time = "Camera.RayTime";
inline constexpr const char* camera_ray_debug = "Camera.RayDebug";
inline constexpr const char* camera_volumetric_cloud_accumulation = "Camera.VolumetricCloudAccumulation";
inline constexpr const char* camera_volumetric_cloud_transmittance = "Camera.VolumetricCloudTransmittance";
inline constexpr const char* camera_gaussian_splat_prepass = "Camera.GaussianSplatPrepass";
inline constexpr const char* frame_visibility_buffer = "Frame.VisibilityBuffer";
inline constexpr const char* frame_ddgi_probe_metadata = "Frame.DDGI.ProbeMetadata";
inline constexpr const char* frame_ddgi_probe_state = "Frame.DDGI.ProbeState";
inline constexpr const char* frame_ddgi_ray_output = "Frame.DDGI.RayOutput";
inline constexpr const char* frame_ddgi_selected_ray_diagnostics = "Frame.DDGI.SelectedRayDiagnostics";
inline constexpr const char* frame_ddgi_irradiance_atlas = "Frame.DDGI.IrradianceAtlas";
inline constexpr const char* frame_ddgi_visibility_atlas = "Frame.DDGI.VisibilityAtlas";
inline constexpr const char* frame_ddgi_variability_atlas = "Frame.DDGI.VariabilityAtlas";
inline constexpr const char* frame_ddgi_variability_reduction_a = "Frame.DDGI.VariabilityReductionA";
inline constexpr const char* frame_ddgi_variability_reduction_b = "Frame.DDGI.VariabilityReductionB";
}  // namespace RenderResourceNames

namespace RenderPassNames {
inline constexpr const char* frame_external = "Frame.External";
inline constexpr const char* ddgi_atlas_prepare = "DDGIAtlasPrepare";
inline constexpr const char* ddgi_probe_scroll = "DDGIProbeScroll";
inline constexpr const char* ddgi_ray_diagnostics = "DDGIRayDiagnostics";
inline constexpr const char* ddgi_probe_update = "DDGIProbeUpdate";
inline constexpr const char* ddgi_probe_relocation = "DDGIProbeRelocation";
inline constexpr const char* ddgi_probe_classification = "DDGIProbeClassification";
inline constexpr const char* ddgi_probe_variability = "DDGIProbeVariability";
inline constexpr const char* ddgi_volumes_complete = "DDGIVolumesComplete";
inline constexpr const char* ddgi_probe_visualization = "DDGIProbeVisualization";
inline constexpr const char* ddgi_probe_ray_visualization = "DDGIProbeRayVisualization";
inline constexpr const char* directional_light_shadow = "DirectionalLightShadow";
inline constexpr const char* deferred_geometry = "DeferredGeometry";
inline constexpr const char* motion_vectors = "MotionVectors";
inline constexpr const char* motion_coverage = "MotionCoverage";
inline constexpr const char* depth_pyramid = "DepthPyramid";
inline constexpr const char* ambient_occlusion = "AmbientOcclusion";
inline constexpr const char* ddgi_gather_timing = "DDGIGatherTiming";
inline constexpr const char* deferred_camera = "DeferredCamera";
inline constexpr const char* transparent_geometry = "TransparentGeometry";
inline constexpr const char* volumetric_clouds = "VolumetricClouds";
inline constexpr const char* gaussian_splat_cull = "GaussianSplatCull";
inline constexpr const char* gaussian_splat_sort = "GaussianSplatSort";
inline constexpr const char* gaussian_splat = "GaussianSplat";
inline constexpr const char* post_processing = "PostProcessing";
inline constexpr const char* ray_tracing_camera = "RayTracingCamera";
inline constexpr const char* ray_query_camera = "RayQueryCamera";
}  // namespace RenderPassNames

namespace RenderGraphConstants {
inline constexpr size_t invalid_pass_index = static_cast<size_t>(-1);
inline constexpr size_t invalid_resource_index = static_cast<size_t>(-1);
inline constexpr size_t invalid_allocation_slot_index = static_cast<size_t>(-1);
}  // namespace RenderGraphConstants

struct RenderResourceUsagePlan {
  size_t resource_index = 0;
  size_t first_pass_index = RenderGraphConstants::invalid_pass_index;
  size_t last_pass_index = RenderGraphConstants::invalid_pass_index;
  std::vector<size_t> reader_pass_indices;
  std::vector<size_t> writer_pass_indices;
  bool used = false;
  bool imported = true;
  bool can_alias = false;
  size_t allocation_slot_index = RenderGraphConstants::invalid_allocation_slot_index;
  RenderResourceDimensions resolved_dimensions;
  std::vector<RenderResourceState> required_states;
};

struct RenderPassExecutionPlan {
  size_t pass_index = 0;
  RenderPassQueue queue = RenderPassQueue::Graphics;
  std::vector<size_t> dependency_indices;
  std::vector<size_t> resource_dependency_indices;
  std::vector<size_t> schedule_dependency_indices;
  size_t schedule_step_index = 0;
};

struct RenderResourceAllocationPlan {
  size_t allocation_slot_index = 0;
  RenderResourceType type = RenderResourceType::External;
  RenderResourceDimensions dimensions;
  RenderResourceDimensions resolved_dimensions;
  std::string format_name;
  uint32_t sample_count = 1;
  uint64_t byte_size = 0;
  std::vector<RenderResourceState> required_states;
  std::vector<size_t> resource_indices;
};

struct RenderResourceTransitionPlan {
  size_t resource_index = 0;
  size_t pass_index = 0;
  size_t previous_pass_index = RenderGraphConstants::invalid_pass_index;
  RenderPassQueue previous_queue = RenderPassQueue::Graphics;
  RenderPassQueue next_queue = RenderPassQueue::Graphics;
  RenderResourceUsage previous_usage = RenderResourceUsage::Read;
  RenderResourceUsage next_usage = RenderResourceUsage::Read;
  RenderResourceState previous_state = RenderResourceState::Undefined;
  RenderResourceState next_state = RenderResourceState::Undefined;
  bool initial_use = false;
  bool queue_change = false;
  bool memory_dependency = false;
};

struct RenderResourceBarrierPlan {
  size_t transition_index = RenderGraphConstants::invalid_pass_index;
  size_t resource_index = 0;
  size_t pass_index = 0;
  size_t previous_pass_index = RenderGraphConstants::invalid_pass_index;
  RenderResourceType resource_type = RenderResourceType::External;
  RenderGraphBarrierType barrier_type = RenderGraphBarrierType::GlobalMemory;
  RenderPassQueue previous_queue = RenderPassQueue::Graphics;
  RenderPassQueue next_queue = RenderPassQueue::Graphics;
  RenderResourceUsage previous_usage = RenderResourceUsage::Read;
  RenderResourceUsage next_usage = RenderResourceUsage::Read;
  RenderResourceState previous_state = RenderResourceState::Undefined;
  RenderResourceState next_state = RenderResourceState::Undefined;
  bool queue_change = false;
  bool memory_dependency = false;
};

struct RenderGraphScheduleStep {
  std::vector<size_t> graphics_pass_indices;
  std::vector<size_t> compute_pass_indices;
  std::vector<size_t> ray_tracing_pass_indices;
};

struct RenderGraphExecutionPlan {
  bool valid = false;
  std::vector<RenderPassExecutionPlan> passes;
  std::vector<RenderResourceUsagePlan> resources;
  std::vector<RenderResourceAllocationPlan> allocations;
  std::vector<RenderResourceTransitionPlan> transitions;
  std::vector<RenderResourceBarrierPlan> barriers;
  std::vector<RenderGraphScheduleStep> schedule_steps;
  bool uses_graphics_queue = false;
  bool uses_compute_queue = false;
  bool uses_ray_tracing_queue = false;
};

struct RenderGraphResourceBinding {
  std::string resource_name;
  std::shared_ptr<Buffer> buffer;
  std::shared_ptr<Image> image;
  std::shared_ptr<DescriptorSet> descriptor_set;
  std::shared_ptr<RenderTexture> render_texture;
  std::vector<std::shared_ptr<Image>> images;
};

class RenderGraphResourceRegistry {
 public:
  void Clear();
  void BindBuffer(const std::string& resource_name, std::shared_ptr<Buffer> buffer);
  void BindImage(const std::string& resource_name, std::shared_ptr<Image> image);
  void BindImages(const std::string& resource_name, std::vector<std::shared_ptr<Image>> images);
  void BindDescriptorSet(const std::string& resource_name, std::shared_ptr<DescriptorSet> descriptor_set);
  void BindRenderTexture(const std::string& resource_name, std::shared_ptr<RenderTexture> render_texture);

  [[nodiscard]] bool HasResourceBinding(const std::string& resource_name) const;
  [[nodiscard]] const RenderGraphResourceBinding* GetResourceBinding(const std::string& resource_name) const;
  [[nodiscard]] const std::vector<RenderGraphResourceBinding>& GetResourceBindings() const;

 private:
  [[nodiscard]] RenderGraphResourceBinding& GetOrAddResourceBinding(const std::string& resource_name);

  std::vector<RenderGraphResourceBinding> resource_bindings_;
};

class RenderGraphTransientResourceStore {
 public:
  void Clear();
  void Allocate(const std::vector<RenderResourceDescriptor>& resources, const RenderGraphExecutionPlan& execution_plan);
  void Bind(RenderGraphResourceRegistry& resource_registry);
  void RetainDescriptorSet(std::shared_ptr<DescriptorSet> descriptor_set);
  void RetainBuffer(std::shared_ptr<Buffer> buffer);
  void RetainAsset(std::shared_ptr<IAsset> asset);
  void RetainRenderTextureResources(std::shared_ptr<RenderTexture> render_texture);
  void RetainImage(std::shared_ptr<Image> image);
  void RetainImageView(std::shared_ptr<ImageView> image_view);

  [[nodiscard]] const std::vector<RenderGraphResourceBinding>& GetResourceBindings() const;

 private:
  std::vector<std::shared_ptr<Buffer>> buffers_;
  std::vector<std::shared_ptr<Image>> images_;
  std::vector<std::shared_ptr<ImageView>> image_views_;
  std::vector<std::shared_ptr<DescriptorSet>> descriptor_sets_;
  std::vector<std::shared_ptr<IAsset>> assets_;
  std::vector<std::shared_ptr<Sampler>> samplers_;
  std::vector<std::shared_ptr<RenderTexture>> render_textures_;
  std::vector<RenderGraphResourceBinding> resource_bindings_;
};

class RenderGraphExecutionContext {
 public:
  RenderGraphExecutionContext(const std::vector<RenderResourceDescriptor>& resources,
                              const std::vector<RenderPassDescriptor>& passes,
                              const RenderGraphExecutionPlan& execution_plan,
                              const RenderGraphResourceRegistry* resource_registry, size_t current_pass_index);

  [[nodiscard]] size_t GetCurrentPassIndex() const;
  [[nodiscard]] const RenderPassDescriptor* GetCurrentPassDescriptor() const;
  [[nodiscard]] const RenderGraphExecutionPlan& GetExecutionPlan() const;
  [[nodiscard]] const RenderResourceDescriptor* GetResourceDescriptor(size_t resource_index) const;
  [[nodiscard]] const RenderResourceDescriptor* GetResourceDescriptor(const std::string& resource_name) const;
  [[nodiscard]] std::vector<const RenderResourceTransitionPlan*> GetCurrentPassTransitions() const;
  [[nodiscard]] std::vector<const RenderResourceBarrierPlan*> GetCurrentPassBarriers() const;
  [[nodiscard]] std::vector<const RenderResourceBarrierPlan*> GetCurrentPassReleaseBarriers() const;
  [[nodiscard]] const RenderResourceUsagePlan* GetResourceUsagePlan(const std::string& resource_name) const;
  [[nodiscard]] const RenderResourceAllocationPlan* GetResourceAllocationPlan(const std::string& resource_name) const;
  [[nodiscard]] const RenderGraphResourceBinding* GetResourceBinding(const std::string& resource_name) const;

 private:
  const std::vector<RenderResourceDescriptor>& resources_;
  const std::vector<RenderPassDescriptor>& passes_;
  const RenderGraphExecutionPlan& execution_plan_;
  const RenderGraphResourceRegistry* resource_registry_ = nullptr;
  size_t current_pass_index_ = RenderGraphConstants::invalid_pass_index;
};

class RenderGraph {
 public:
  using ExecuteFunction = std::function<void(const RenderGraphExecutionContext&)>;

  void Clear();
  void AddResource(const RenderResourceDescriptor& descriptor);
  void AddPass(const RenderPassDescriptor& descriptor, ExecuteFunction execute);
  [[nodiscard]] RenderGraphExecutionPlan Compile() const;
  [[nodiscard]] RenderGraphExecutionPlan Compile(const RenderGraphCompileContext& context) const;
  void Execute() const;
  void Execute(const RenderGraphResourceRegistry& resource_registry) const;
  void Execute(const RenderGraphExecutionPlan& execution_plan,
               const RenderGraphResourceRegistry& resource_registry) const;

  [[nodiscard]] bool Validate() const;
  [[nodiscard]] bool HasResource(const std::string& name) const;
  [[nodiscard]] bool UsesQueue(RenderPassQueue queue) const;
  [[nodiscard]] std::vector<size_t> GetPassIndices(RenderPassQueue queue) const;
  [[nodiscard]] const std::vector<RenderResourceDescriptor>& GetResources() const;
  [[nodiscard]] const std::vector<RenderPassDescriptor>& GetPasses() const;

 private:
  std::vector<RenderResourceDescriptor> resources_;
  std::vector<RenderPassDescriptor> passes_;
  std::vector<ExecuteFunction> execute_functions_;
};

struct RenderGraphPlanCacheStats {
  size_t entry_count = 0;
  size_t capacity = 0;
  uint64_t hit_count = 0;
  uint64_t miss_count = 0;
  uint64_t eviction_count = 0;
  uint64_t compilation_count = 0;
  double compilation_milliseconds = 0.0;
};

class RenderGraphPlanCache final {
 public:
  explicit RenderGraphPlanCache(size_t capacity = 16);
  [[nodiscard]] const RenderGraphExecutionPlan& GetOrCompile(const RenderGraph& graph,
                                                             const RenderGraphCompileContext& context);
  void Clear();
  [[nodiscard]] RenderGraphPlanCacheStats GetStats() const;

 private:
  struct Entry {
    RenderGraphCompileContext context{};
    std::vector<RenderResourceDescriptor> resources{};
    std::vector<RenderPassDescriptor> passes{};
    RenderGraphExecutionPlan plan{};
    uint64_t last_use = 0;
  };

  size_t capacity_ = 16;
  uint64_t use_counter_ = 0;
  RenderGraphPlanCacheStats stats_{};
  std::vector<Entry> entries_{};
};

void AddDefaultFrameResources(RenderGraph& graph);
void AddDefaultRasterCameraResources(RenderGraph& graph);
void AddDefaultRayTracingCameraResources(RenderGraph& graph);
void AddRayCameraOptionalOutputResources(RenderGraph& graph, const CameraSettings::RayOutputSettings& outputs);
void AddAdvancedFrameResources(RenderGraph& graph);
void AddAdvancedCameraResources(RenderGraph& graph);
void AddVolumetricCloudCameraResources(RenderGraph& graph, uint32_t resolution_divisor = 1);
void AddGaussianSplatCameraResources(RenderGraph& graph);
}  // namespace evo_engine
