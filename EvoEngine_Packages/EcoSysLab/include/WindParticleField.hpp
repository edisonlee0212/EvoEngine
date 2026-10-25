#pragma once

#include <algorithm>
#include <unordered_map>

#include "ComputePipeline.hpp"
#include "GraphicsPipeline.hpp"
#include "Scene.hpp"

namespace eco_sys_lab_package {
using namespace evo_engine;
class GpuTreePhysics;

class WindParticleField : public std::enable_shared_from_this<WindParticleField> {
 public:
  static constexpr uint32_t capacity = 65536;
  static constexpr uint32_t grid_bucket_count = 32768;
  static constexpr float grid_cell_size = 1.0f;

  void Step(const std::shared_ptr<Scene>& scene, float time_step, const std::shared_ptr<GpuTreePhysics>& physics);
  void RegisterRender(bool visible);
  [[nodiscard]] uint32_t GetParticleCount() const {
    return static_cast<uint32_t>(std::min<uint64_t>(emitted_, capacity));
  }
  [[nodiscard]] float GetMaxParticleRadius() const {
    return max_particle_radius_;
  }
  [[nodiscard]] const std::shared_ptr<DescriptorSet>& GetDescriptorSet() const {
    return descriptor_set_;
  }

 private:
  struct Particle {
    glm::vec4 position_age{};
    glm::vec4 velocity_lifetime{};
    glm::vec4 collision{};
  };
  static_assert(sizeof(Particle) == 48);
  struct EmitParameters {
    glm::mat4 model{1.0f};
    glm::vec4 speed_spread_lifetime{};
    glm::vec4 collision{};
    glm::uvec4 base_count_seed_capacity{};
  };
  static_assert(sizeof(EmitParameters) == 112);
  struct AdvectParameters {
    float time_step = 0;
    uint32_t count = 0;
    glm::vec2 padding{};
  };
  struct GridParameters {
    uint32_t count = 0;
    uint32_t bucket_count = 0;
    float cell_size = 1.0f;
    float padding = 0;
  };
  struct RodCollisionParameters {
    uint32_t particle_count = 0;
    uint32_t rod_count = 0;
    uint32_t bucket_count = 0;
    float cell_size = 1.0f;
  };
  struct FlowParameters {
    uint32_t particle_count = 0;
    uint32_t rod_count = 0;
    uint32_t bucket_count = 0;
    float cell_size = 1.0f;
    float time_step = 0.0f;
    float max_support = 0.0f;
  };
  struct RenderParameters {
    uint32_t camera_index = 0;
    glm::vec2 viewport_size{};
    float point_radius = 3.0f;
  };
  struct RodCollisionFrame {
    uint32_t frame_count = UINT32_MAX;
    std::vector<std::shared_ptr<DescriptorSet>> descriptor_sets;
    std::vector<std::shared_ptr<GpuTreePhysics>> physics;
    std::vector<std::shared_ptr<Buffer>> rod_next_buffers;
  };

  bool Prepare();
  uint32_t Draw(VkCommandBuffer command_buffer, const std::shared_ptr<Camera>& camera, const glm::ivec4& viewport,
                uint32_t camera_index);

  std::weak_ptr<Scene> scene_;
  std::shared_ptr<Buffer> particles_;
  std::shared_ptr<Buffer> grid_heads_;
  std::shared_ptr<Buffer> grid_next_;
  std::shared_ptr<Buffer> rod_heads_;
  std::shared_ptr<Buffer> rod_next_;
  std::shared_ptr<Buffer> rod_max_extent_;
  std::shared_ptr<Buffer> particle_density_;
  std::shared_ptr<Buffer> particle_velocity_;
  std::shared_ptr<DescriptorSetLayout> layout_;
  std::shared_ptr<DescriptorSet> descriptor_set_;
  std::shared_ptr<DescriptorSetLayout> rod_collision_layout_;
  std::vector<RodCollisionFrame> rod_collision_frames_;
  std::shared_ptr<ComputePipeline> advect_pipeline_;
  std::shared_ptr<ComputePipeline> emit_pipeline_;
  std::shared_ptr<ComputePipeline> clear_grid_pipeline_;
  std::shared_ptr<ComputePipeline> grid_pipeline_;
  std::shared_ptr<ComputePipeline> clear_rods_pipeline_;
  std::shared_ptr<ComputePipeline> build_rods_pipeline_;
  std::shared_ptr<ComputePipeline> collide_rods_pipeline_;
  std::shared_ptr<ComputePipeline> density_pipeline_;
  std::shared_ptr<ComputePipeline> flow_pipeline_;
  std::shared_ptr<ComputePipeline> apply_flow_pipeline_;
  std::shared_ptr<GraphicsPipeline> render_pipeline_;
  std::unordered_map<uint64_t, float> emission_remainders_;
  uint64_t emitted_ = 0;
  float max_particle_radius_ = 0.0f;
};
}  // namespace eco_sys_lab_package
