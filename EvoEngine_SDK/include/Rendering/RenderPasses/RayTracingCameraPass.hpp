#pragma once
#include "RenderGraph.hpp"

#include <functional>

namespace evo_engine {
class EVOENGINE_API Camera;
class EVOENGINE_API ComputePipeline;
class EVOENGINE_API DescriptorSet;
class EVOENGINE_API DescriptorSetLayout;
struct RayCameraHistoryResources;
class EVOENGINE_API RayTracingPipeline;

class EVOENGINE_API RayTracingCameraPass final {
 public:
  using RecordCommands = std::function<void(const std::function<void(VkCommandBuffer vk_command_buffer)>& action)>;

  struct Parameters {
    std::shared_ptr<Camera> camera;
    std::shared_ptr<RayTracingPipeline> pipeline;
    std::shared_ptr<DescriptorSet> per_frame_descriptor_set;
    std::shared_ptr<DescriptorSet> ray_tracing_descriptor_set;
    int camera_index = -1;
    RecordCommands record_commands;
    std::shared_ptr<DescriptorSet> output_descriptor_set;
    RenderGraphTransientResourceStore* transient_resources = nullptr;
    RayCameraHistoryResources* history_resources = nullptr;
    std::shared_ptr<Buffer> primary_surface_buffer;
    bool write_nrd_signals = false;
  };

  [[nodiscard]] static RenderPassDescriptor CreateDescriptor(
      const char* pass_name = RenderPassNames::ray_tracing_camera, CameraSettings::RayOutputSettings outputs = {});
  static void Execute(const RenderGraphExecutionContext& context, const Parameters& parameters);
};

class EVOENGINE_API RayQueryCameraPass final {
 public:
  using RecordCommands = RayTracingCameraPass::RecordCommands;

  struct Parameters {
    std::shared_ptr<Camera> camera;
    std::shared_ptr<ComputePipeline> pipeline;
    std::shared_ptr<DescriptorSet> per_frame_descriptor_set;
    std::shared_ptr<DescriptorSet> ray_tracing_descriptor_set;
    int camera_index = -1;
    RecordCommands record_commands;
    std::shared_ptr<DescriptorSet> output_descriptor_set;
    RenderGraphTransientResourceStore* transient_resources = nullptr;
    RayCameraHistoryResources* history_resources = nullptr;
    std::shared_ptr<Buffer> candidate_buffer;
    std::shared_ptr<Buffer> primary_surface_buffer;
    bool commit_history = true;
    std::shared_ptr<Buffer> resolved_buffer;
    std::shared_ptr<Buffer> shift_buffer;
    std::shared_ptr<Buffer> generated_buffer;
    std::shared_ptr<Buffer> previous_history_buffer;
    std::shared_ptr<Buffer> previous_surface_buffer;
    std::shared_ptr<Buffer> temporal_forward_buffer;
    std::shared_ptr<Buffer> duplication_buffer;
    std::shared_ptr<Buffer> next_history_buffer;
    std::shared_ptr<Buffer> pairing_buffer;
    bool commit_restir_history = false;
    std::shared_ptr<Buffer> previous_instance_buffer;
    bool bind_nrd_guides = false;
    bool write_nrd_signals = false;
    bool read_nrd_denoised = false;
    bool bind_nrd_lobe_hit_distance = false;
    bool bind_nrd_material_factors = false;
    bool bind_nrd_accumulated_input = false;
    bool bind_nrd_temporal_aa = false;
    bool commit_nrd_temporal_aa = false;
    uint32_t nrd_taa_write_slot = 0;
    uint32_t restir_sample_index = 0;
    std::shared_ptr<Buffer> nrd_sample_accumulation_buffer;
    bool sss_temporal = false;
    bool sss_history_valid = false;
    bool commit_sss_history = true;
    bool sss_spatial_resolve = false;
  };

  [[nodiscard]] static RenderPassDescriptor CreateDescriptor(CameraSettings::RayOutputSettings outputs = {},
                                                             const char* pass_name = RenderPassNames::ray_query_camera);
  static void Execute(const RenderGraphExecutionContext& context, const Parameters& parameters);
};

}  // namespace evo_engine
