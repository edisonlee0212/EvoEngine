#pragma once

#include <unordered_map>

#include "ComputePipeline.hpp"
#include "GraphicsPipeline.hpp"
#include "Scene.hpp"

namespace eco_sys_lab_package {
using namespace evo_engine;

class WindParticleField : public std::enable_shared_from_this<WindParticleField> {
 public:
  static constexpr uint32_t capacity = 65536;

  void Step(const std::shared_ptr<Scene>& scene, float time_step);
  void RegisterRender(bool visible);

 private:
  struct Particle {
    glm::vec4 position_age{};
    glm::vec4 velocity_lifetime{};
  };
  struct EmitParameters {
    glm::mat4 model{1.0f};
    glm::vec4 speed_spread_lifetime{};
    glm::uvec4 base_count_seed_capacity{};
  };
  struct AdvectParameters {
    float time_step = 0;
    uint32_t count = 0;
    glm::vec2 padding{};
  };
  struct RenderParameters {
    uint32_t camera_index = 0;
    glm::vec2 viewport_size{};
    float point_radius = 3.0f;
  };

  bool Prepare();
  uint32_t Draw(VkCommandBuffer command_buffer, const std::shared_ptr<Camera>& camera, const glm::ivec4& viewport,
                uint32_t camera_index);

  std::weak_ptr<Scene> scene_;
  std::shared_ptr<Buffer> particles_;
  std::shared_ptr<DescriptorSetLayout> layout_;
  std::shared_ptr<DescriptorSet> descriptor_set_;
  std::shared_ptr<ComputePipeline> advect_pipeline_;
  std::shared_ptr<ComputePipeline> emit_pipeline_;
  std::shared_ptr<GraphicsPipeline> render_pipeline_;
  std::unordered_map<uint64_t, float> emission_remainders_;
  uint64_t emitted_ = 0;
};
}  // namespace eco_sys_lab_package
