#pragma once

#include "ComputePipeline.hpp"
#include "GraphicsPipeline.hpp"
#include "RenderGraph.hpp"
#include "SdfgiCapabilities.hpp"
#include "SdfgiSettings.hpp"
#include "SdfgiTypes.hpp"

#include <array>
#include <map>
#include <optional>

namespace evo_engine {

class SdfgiVoxelFrame;
class SdfgiVoxelDebug;
class SdfgiPreprocessDebug;
class SdfgiLightFrame;
class SdfgiLightDebug;
class SdfgiProbeFrame;
class SdfgiProbeDebug;
class SdfgiGatherFrame;

enum class SdfgiMemoryClass { Field, Scratch, Upload, Diagnostic, Count };
enum class SdfgiLayout {
  Initialize,
  JumpFlood,
  Upscale,
  Occlusion,
  Store,
  Scroll,
  ScrollOcclusion,
  DirectLight,
  Integrate,
  Sky,
  Voxel,
  Gather,
  Count
};

struct SdfgiTexture {
  SdfgiImageRequirement requirement;
  std::shared_ptr<Image> image;
  std::shared_ptr<ImageView> storage_view;
  std::shared_ptr<ImageView> sampled_view;
  SdfgiMemoryClass memory_class;
};

struct SdfgiBuffer {
  std::shared_ptr<Buffer> buffer;
  SdfgiMemoryClass memory_class;
};

// One allocation owner is retained by every submitting frame slot, including its immutable input/descriptor versions.
class EVOENGINE_API SdfgiResources {
 public:
  SdfgiSettings settings;
  std::map<std::string, SdfgiTexture> textures;
  std::map<std::string, SdfgiBuffer> buffers;
  std::array<uint64_t, static_cast<size_t>(SdfgiMemoryClass::Count)> allocated_bytes{};
  std::array<std::shared_ptr<DescriptorSetLayout>, static_cast<size_t>(SdfgiLayout::Count)> layouts;
  std::map<std::string, std::shared_ptr<DescriptorSet>> sets;
  std::map<std::string, std::shared_ptr<ComputePipeline>> pipelines;
  std::shared_ptr<GraphicsPipeline> voxel_pipeline;
  std::shared_ptr<Sampler> linear_sampler;
  std::shared_ptr<Sampler> mip_sampler;
  bool initialization_recorded = false;
  bool voxelization_recorded = false;
  uint32_t last_voxel_frame = UINT32_MAX;
  std::vector<std::shared_ptr<SdfgiVoxelFrame>> voxel_frames;
  std::optional<glm::uvec2> voxel_debug_request;
  std::shared_ptr<SdfgiVoxelDebug> voxel_debug;
  std::string voxel_failure;
  uint32_t preprocessed_cascades = 0;
  SdfgiFieldStatus preprocess_status{};
  bool preprocess_status_available = false;
  std::vector<SdfgiDispatchData> solid_cell_dispatch;
  std::string preprocess_failure;
  std::optional<glm::uvec2> preprocess_debug_request;
  std::shared_ptr<SdfgiPreprocessDebug> preprocess_debug;
  std::string preprocess_debug_failure;
  std::vector<std::shared_ptr<SdfgiPreprocessDebug>> preprocess_debug_frames;
  uint32_t last_light_frame = UINT32_MAX;
  bool lighting_recorded = false;
  std::string light_failure;
  std::vector<std::vector<SdfgiLight>> static_light_inputs;
  std::vector<std::shared_ptr<SdfgiLightFrame>> light_frames;
  std::optional<glm::uvec2> light_debug_request;
  std::shared_ptr<SdfgiLightDebug> light_debug;
  std::string light_debug_failure;
  std::vector<std::shared_ptr<SdfgiLightDebug>> light_debug_frames;
  uint32_t last_transport_frame = UINT32_MAX;
  uint32_t transport_pass = 0;
  bool transport_recorded = false;
  std::string transport_failure;
  std::vector<std::shared_ptr<SdfgiProbeFrame>> probe_frames;
  std::optional<glm::uvec2> probe_debug_request;
  std::shared_ptr<SdfgiProbeDebug> probe_debug;
  std::string probe_debug_failure;
  std::vector<std::shared_ptr<SdfgiProbeDebug>> probe_debug_frames;
  std::vector<std::shared_ptr<SdfgiGatherFrame>> gather_frames;
  std::shared_ptr<SdfgiGatherFrame> publication;
  std::vector<uint64_t> gather_camera_ids;
  [[nodiscard]] uint64_t GetAllocationBytes(SdfgiMemoryClass memory_class) const;

  // fail_after_allocations is a deterministic partial-allocation failure seam for focused tests only.
  static std::shared_ptr<SdfgiResources> TryCreate(
      const SdfgiSettings& settings, const std::vector<std::shared_ptr<DescriptorSetLayout>>& deferred_host_layouts,
      std::string& failure, uint32_t fail_after_allocations = UINT32_MAX);
  [[nodiscard]] static std::string ValidateDescriptorLimits(
      const std::vector<std::shared_ptr<DescriptorSetLayout>>& pipeline_layouts, const VkPhysicalDeviceLimits& limits);

  void Import(RenderGraph& graph, RenderGraphResourceRegistry& registry) const;
  [[nodiscard]] RenderPassDescriptor ClearDescriptor() const;
  void Clear(VkCommandBuffer command_buffer, const RenderGraphExecutionContext& context);
  void OrderAccess(VkCommandBuffer command_buffer, VkPipelineStageFlags2 destination_stages,
                   VkAccessFlags2 destination_access) const;

 private:
  void Allocate(const std::vector<std::shared_ptr<DescriptorSetLayout>>& deferred_host_layouts,
                uint32_t fail_after_allocations);
  void CreateLayouts(const std::vector<std::shared_ptr<DescriptorSetLayout>>& deferred_host_layouts);
  void CreateDescriptors();
  void CreatePipelines(const std::vector<std::shared_ptr<DescriptorSetLayout>>& deferred_host_layouts);
};

}  // namespace evo_engine
