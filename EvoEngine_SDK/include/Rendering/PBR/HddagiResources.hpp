#pragma once

#include "GraphicsResources.hpp"
#include "HddagiSettings.hpp"
#include "RenderGraph.hpp"
#include "SdfgiScene.hpp"

#include <map>
#include <memory>

namespace evo_engine {

// All VMA allocations, including shared scene resources: allocated bytes, reserved block bytes.
EVOENGINE_API std::array<uint64_t, 2> QueryGiValidationMemoryBytes();

class HddagiVoxelFrame;
class HddagiLightFrame;
class HddagiProbeFrame;
class HddagiCameraFrame;
struct HddagiCameraImages;
class GraphicsPipeline;
class ComputePipeline;

struct HddagiImageRequirement {
  std::string name;
  VkFormat storage_format;
  VkFormat sampled_format;
  VkImageType type;
  VkExtent3D extent;
  uint32_t layers = 1;
  bool temporal = false;
  bool filtered = false;
  bool atomic = false;
  bool cube = false;
};

struct EVOENGINE_API HddagiCapabilityReport {
  std::string device_name;
  std::string failure = "HDDAGI capabilities have not been queried";
  uint64_t temporal_bytes = 0;
  uint64_t image_bytes = 0;
  uint64_t buffer_bytes = 0;
  [[nodiscard]] bool Supported() const {
    return failure.empty();
  }
};

EVOENGINE_API std::vector<HddagiImageRequirement> GetHddagiImageRequirements(const GiProbeSettings& probes,
                                                                             const HddagiSettings& settings,
                                                                             bool occlusion_only = false);
EVOENGINE_API HddagiCapabilityReport QueryHddagiCapabilities(const GiProbeSettings& probes,
                                                             const HddagiSettings& settings,
                                                             bool occlusion_only = false);
EVOENGINE_API uint64_t HddagiLogicalTemporalBytes(const GiProbeSettings& probes, const HddagiSettings& settings);

struct HddagiImage {
  HddagiImageRequirement requirement;
  std::shared_ptr<Image> image;
  std::shared_ptr<ImageView> storage;
  std::shared_ptr<ImageView> sampled;
};

class EVOENGINE_API HddagiResources {
 public:
  GiProbeSettings probes;
  HddagiSettings settings;
  bool occlusion_only = false;
  std::map<std::string, HddagiImage> images;
  std::map<std::string, std::shared_ptr<Buffer>> buffers;
  uint32_t light_cell_capacity = 0;
  uint64_t allocation_bytes = 0;
  uint64_t temporal_bytes = 0;
  bool initialization_recorded = false;
  std::shared_ptr<GraphicsPipeline> voxel_pipeline;
  std::shared_ptr<ComputePipeline> region_pipeline;
  std::shared_ptr<ComputePipeline> light_store_pipeline;
  std::shared_ptr<ComputePipeline> light_scroll_pipeline;
  std::shared_ptr<ComputePipeline> reset_probes_pipeline;
  std::shared_ptr<ComputePipeline> occlusion_pipeline;
  std::shared_ptr<ComputePipeline> metadata_pipeline;
  std::vector<std::shared_ptr<HddagiVoxelFrame>> voxel_frames;
  std::vector<std::shared_ptr<HddagiLightFrame>> light_frames;
  std::array<std::shared_ptr<ComputePipeline>, 2> direct_pipelines;
  std::shared_ptr<Sampler> linear_sampler;
  std::vector<std::array<std::vector<SdfgiLight>, 2>> light_inputs;
  std::vector<std::array<uint32_t, 2>> light_overflow;
  bool transport_recorded = false;
  SdfgiSkyInput sky_input;
  uint32_t force_probe_frames = 0;
  bool transport_ready = false;
  uint32_t transport_generation = 0;
  uint32_t transport_failure_flags = 0;
  uint64_t last_transport_status_frame = UINT64_MAX;
  std::array<std::shared_ptr<ComputePipeline>, 2> transport_status_pipelines;
  std::shared_ptr<ComputePipeline> integrate_pipeline;
  std::shared_ptr<ComputePipeline> filter_pipeline;
  std::shared_ptr<Sampler> mip_sampler;
  std::vector<std::shared_ptr<HddagiProbeFrame>> probe_frames;
  std::map<uint64_t, std::shared_ptr<HddagiCameraImages>> camera_images;
  std::vector<std::vector<std::shared_ptr<HddagiCameraFrame>>> camera_frames;
  std::map<std::string, std::shared_ptr<ComputePipeline>> camera_pipelines;
  std::shared_ptr<const HddagiResources> capture_source;
  std::shared_ptr<struct FrameSubmissionState> submission;
  uint64_t last_voxel_frame = UINT64_MAX;
  bool voxelization_recorded = false;
  uint64_t last_status_frame = UINT64_MAX;
  std::vector<uint32_t> light_cell_counts;
  uint32_t failure_flags = 0;

  [[nodiscard]] uint64_t AllocationBytes() const;
  void CaptureToPng(const std::filesystem::path& path, const std::string& image_name, uint32_t layer = 0,
                    uint32_t z_slice = 0) const;
  void Import(RenderGraph& graph, RenderGraphResourceRegistry& registry) const;
  [[nodiscard]] RenderPassDescriptor ClearDescriptor() const;
  void Clear(VkCommandBuffer command, const RenderGraphExecutionContext& context);
  void OrderAccess(VkCommandBuffer command, VkPipelineStageFlags2 stages, VkAccessFlags2 access) const;

  static std::shared_ptr<HddagiResources> TryCreate(const GiProbeSettings& probes, const HddagiSettings& settings,
                                                    std::string& failure, uint32_t fail_after_allocations = UINT32_MAX,
                                                    bool occlusion_only = false);
};

struct EVOENGINE_API HddagiRuntime {
  GiProbeSettings probes;
  HddagiSettings settings;
  GiProbeFrame frame;
  HddagiCapabilityReport capabilities;
  std::shared_ptr<HddagiResources> resources;
  uint64_t retiring_bytes = 0;
  SdfgiContributorRegistry contributors;
  std::vector<SdfgiCascade> cascades;
  uint32_t region_version = 0;
  bool force_full_update = true;
  uint64_t last_updated_regions = 0;
  uint64_t total_updated_regions = 0;
  uint64_t update_count = 0;
  std::string voxel_failure;
  std::string transport_failure;
  bool allocation_attempted = false;
  bool published = false;
  std::string fallback_reason = "HDDAGI transport is not ready";
};

}  // namespace evo_engine
