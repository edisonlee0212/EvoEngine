#pragma once

#include <algorithm>
#include <unordered_map>

#include "Climate.hpp"
#include "ComputePipeline.hpp"
#include "DynamicTreeSkeleton.hpp"
#include "GpuSpatialGrid.hpp"
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

  WindParticleField() : air_grid_(Climate::default_air_cell_size) {
  }

  void Step(const std::shared_ptr<Scene>& scene, float time_step, const std::shared_ptr<GpuTreePhysics>& physics);
  void RegisterRender(bool particles_visible, bool grid_visible, bool coverage_visible);
  void RegisterTreeCoverage(const std::vector<DynamicTreeSkeleton::GpuRestState>& trees);
  void RegisterTreeModels(const std::vector<glm::mat4>& models);
  void ResetAirGrid(const std::vector<DynamicTreeSkeleton::GpuRestState>& trees,
                    const std::shared_ptr<GpuTreePhysics>& physics, float cell_size = -1.0f);
  [[nodiscard]] uint32_t GetParticleCount() const {
    return static_cast<uint32_t>(std::min<uint64_t>(emitted_, capacity));
  }
  [[nodiscard]] const std::shared_ptr<DescriptorSet>& GetAirDescriptorSet() const {
    return air_descriptor_set_;
  }
  [[nodiscard]] uint32_t GetAirCellCount() const {
    return air_grid_.GetCellCount();
  }
  [[nodiscard]] float GetAirCellSize() const {
    return air_grid_.GetCellSize();
  }

  float ambient_temperature_kelvin = 293.15f;
  float ambient_specific_humidity = 0.007f;
  float ambient_pressure_pa = 101325.0f;
  float air_coupling_rate = 4.0f;
  float air_friction = 4.0f;

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
  struct RenderParameters {
    uint32_t camera_index = 0;
    glm::vec2 viewport_size{};
    float point_radius = 3.0f;
  };
  struct GridRenderParameters {
    uint32_t camera_index = 0;
    float cell_size = 1.0f;
    float ambient_temperature = 293.15f;
    uint32_t bucket_count = GpuSpatialGrid::bucket_count;
    glm::vec4 lower{};  // w is the optional registered-cell opacity density.
    glm::vec4 upper{};
  };
  struct AirParameters {
    uint32_t cell_count = 0;
    uint32_t bucket_count = 0;
    float cell_size = 1;
    float time_step = 0;
    float coupling_rate = 0;
    float friction = 0;
    float ambient_temperature = 293.15f;
    float ambient_humidity = 0.007f;
    float ambient_pressure = 101325.0f;
    float particle_cell_size = grid_cell_size;
  };
  struct TraceParameters {
    uint32_t particle_count = 0;
    uint32_t bucket_count = 0;
    float cell_size = 1;
    float time_step = 0;
    float coupling_rate = 0;
  };
  struct RodParameters {
    uint32_t particle_count = 0;
    uint32_t rod_count = 0;
    uint32_t bucket_count = 0;
    float cell_size = 1;
  };
  struct RodAirParameters {
    uint32_t cell_count = 0;
    uint32_t rod_count = 0;
    uint32_t bucket_count = 0;
    float rod_cell_size = grid_cell_size;
    float air_cell_size = 1;
    float time_step = 0;
  };
  struct RodFrame {
    uint32_t frame_count = UINT32_MAX;
    size_t used = 0;
    std::vector<std::shared_ptr<DescriptorSet>> sets;
    std::vector<std::shared_ptr<GpuTreePhysics>> physics;
    std::vector<std::shared_ptr<Buffer>> buffers;
  };
  static_assert(sizeof(AirParameters) == 40);
  static_assert(sizeof(TraceParameters) == 20);
  static_assert(sizeof(RodAirParameters) == 24);
  struct TreeBounds {
    glm::vec3 lower_local{};
    glm::vec3 upper_local{};
    glm::mat4 model{1.0f};
  };

  bool Prepare();
  void InvalidateAirSets();
  uint32_t Draw(VkCommandBuffer command_buffer, const std::shared_ptr<Camera>& camera, const glm::ivec4& viewport,
                uint32_t camera_index);
  uint32_t DrawVolume(VkCommandBuffer command_buffer, const std::shared_ptr<Camera>& camera, const glm::ivec4& viewport,
                      uint32_t camera_index, bool coverage_visible);

  std::weak_ptr<Scene> scene_;
  std::shared_ptr<Buffer> particles_;
  std::shared_ptr<Buffer> grid_heads_;
  std::shared_ptr<Buffer> grid_next_;
  std::shared_ptr<Buffer> rod_heads_;
  std::shared_ptr<Buffer> rod_next_;
  std::shared_ptr<Buffer> rod_max_extent_;
  std::shared_ptr<DescriptorSetLayout> layout_;
  std::shared_ptr<DescriptorSet> descriptor_set_;
  GpuSpatialGrid air_grid_;
  std::vector<TreeBounds> tree_bounds_;
  std::shared_ptr<DescriptorSetLayout> air_layout_;
  std::shared_ptr<DescriptorSet> air_descriptor_set_;
  std::shared_ptr<DescriptorSetLayout> grid_render_layout_;
  std::shared_ptr<DescriptorSet> grid_render_set_;
  std::shared_ptr<DescriptorSetLayout> grid_depth_layout_;
  std::unordered_map<const RenderTexture*, std::pair<std::shared_ptr<ImageView>, std::shared_ptr<DescriptorSet>>>
      grid_depth_sets_;
  std::shared_ptr<DescriptorSetLayout> air_update_layout_;
  std::shared_ptr<DescriptorSet> air_update_set_;
  std::shared_ptr<ComputePipeline> air_update_pipeline_;
  std::shared_ptr<DescriptorSetLayout> trace_layout_;
  std::shared_ptr<DescriptorSet> trace_set_;
  std::shared_ptr<ComputePipeline> trace_pipeline_;
  std::shared_ptr<DescriptorSetLayout> rod_layout_;
  std::vector<RodFrame> rod_frames_;
  std::shared_ptr<ComputePipeline> clear_rods_pipeline_;
  std::shared_ptr<ComputePipeline> build_rods_pipeline_;
  std::shared_ptr<ComputePipeline> rod_air_pipeline_;
  std::vector<std::pair<uint32_t, std::shared_ptr<DescriptorSet>>> retired_air_sets_;
  std::shared_ptr<ComputePipeline> advect_pipeline_;
  std::shared_ptr<ComputePipeline> emit_pipeline_;
  std::shared_ptr<ComputePipeline> clear_grid_pipeline_;
  std::shared_ptr<ComputePipeline> grid_pipeline_;
  std::shared_ptr<GraphicsPipeline> render_pipeline_;
  std::shared_ptr<GraphicsPipeline> grid_render_pipeline_;
  std::unordered_map<uint64_t, float> emission_remainders_;
  uint64_t emitted_ = 0;
};
}  // namespace eco_sys_lab_package
