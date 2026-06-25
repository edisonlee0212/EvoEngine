#pragma once
#include "RenderGraph.hpp"
#include "VolumetricCloudSettings.hpp"

#include <functional>

namespace evo_engine {
class Camera;
class ComputePipeline;
class DescriptorSet;
class DescriptorSetLayout;

struct VolumetricCloudsPushConstant {
  glm::vec4 altitude_coverage_density = glm::vec4(0.0f);
  glm::vec4 wind_time = glm::vec4(0.0f);
  glm::vec4 lighting_phase_max_distance = glm::vec4(0.0f);
  glm::vec4 noise_extinction_march_distance = glm::vec4(0.0f);
  glm::vec4 atmosphere_cloud_type_curl = glm::vec4(0.0f);
  glm::vec4 march_control = glm::vec4(0.0f);
  glm::ivec4 camera_frame_steps = glm::ivec4(0);
  glm::ivec4 flags = glm::ivec4(0);
};

class VolumetricCloudsPass final {
 public:
  using RecordCommands = std::function<void(const std::function<void(VkCommandBuffer vk_command_buffer)>& action)>;

  struct Parameters {
    std::shared_ptr<Camera> camera;
    RecordCommands record_commands;
    std::shared_ptr<ComputePipeline> pipeline;
    std::shared_ptr<ComputePipeline> composite_pipeline;
    std::shared_ptr<DescriptorSet> per_frame_descriptor_set;
    std::shared_ptr<DescriptorSetLayout> descriptor_set_layout;
    RenderGraphTransientResourceStore* transient_resources = nullptr;
    VolumetricCloudSettings settings;
    int camera_index = 0;
    uint32_t frame_index = 0;
    float time_seconds = 0.0f;
    float max_distance = 0.0f;
    const char* input_resource_name = RenderResourceNames::camera_depth;
    bool input_is_ray_hit_distance = false;
  };

  [[nodiscard]] static RenderPassDescriptor CreateRasterDescriptor(const char* dependency);
  [[nodiscard]] static RenderPassDescriptor CreateRayTracingDescriptor(const char* dependency);
  static void Execute(const RenderGraphExecutionContext& context, const Parameters& parameters);
};
}  // namespace evo_engine
