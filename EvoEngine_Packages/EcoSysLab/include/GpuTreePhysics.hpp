#pragma once

#include <array>

#include "ComputePipeline.hpp"
#include "DynamicTreeSkeleton.hpp"

namespace eco_sys_lab_package {
class GpuTreePhysics {
 public:
  struct TreeRange {
    uint32_t vertex_base = 0;
    uint32_t vertex_count = 0;
    uint32_t rod_base = 0;
    uint32_t rod_count = 0;
    uint32_t node_rod_base = 0;
  };
  struct PackedRestState {
    DynamicTreeSkeleton::GpuRestState state;
    std::vector<TreeRange> ranges;
  };
  struct TreeStep {
    DynamicTreeSkeleton::PhysicsSettings settings;
    bool running = true;
    GlobalTransform model{};
  };
  struct WindInput {
    std::shared_ptr<DescriptorSet> descriptor_set;
    uint32_t particle_count = 0;
    uint32_t bucket_count = 0;
    float cell_size = 1.0f;
    float max_particle_radius = 0.0f;
  };

  [[nodiscard]] static PackedRestState PackRestStates(const std::vector<DynamicTreeSkeleton::GpuRestState>& states);
  bool Initialize(const DynamicTreeSkeleton::GpuRestState& rest_state);
  bool Initialize(const std::vector<DynamicTreeSkeleton::GpuRestState>& rest_states);
  void Step(const DynamicTreeSkeleton::PhysicsSettings& settings);
  void Step(const std::vector<TreeStep>& trees, const WindInput& wind = {});
  [[nodiscard]] const std::shared_ptr<DescriptorSetLayout>& GetWindDescriptorSetLayout() const {
    return wind_layout_;
  }
  [[nodiscard]] const TreeRange& GetTreeRange(uint32_t index) const {
    return ranges_[index];
  }
  [[nodiscard]] const std::shared_ptr<Buffer>& GetVertexBuffer() const {
    return vertices_[vertex_index_];
  }
  [[nodiscard]] const std::shared_ptr<Buffer>& GetRodBuffer() const {
    return rods_[rod_index_];
  }
  [[nodiscard]] const std::shared_ptr<Buffer>& GetNodeRodBuffer() const {
    return node_rods_;
  }

 private:
  struct Parameters {
    uint32_t vertex_count = 0;
    uint32_t rod_count = 0;
    uint32_t substep = 0;
    uint32_t iteration = 0;
    uint32_t particle_count = 0;
    uint32_t bucket_count = 0;
    float cell_size = 1.0f;
    float max_particle_radius = 0.0f;
  };
  struct TreeParameters {
    glm::vec4 mechanics{};
    glm::vec4 gravity{};
    glm::uvec4 control{};
    glm::mat4 previous_model{1.0f};
    glm::mat4 current_model{1.0f};
    glm::mat4 model_delta{1.0f};
    glm::vec4 anchor_previous{0, 0, 0, 1};
    glm::vec4 anchor_current{0, 0, 0, 1};
    glm::vec4 rotation_delta{0, 0, 0, 1};
    glm::vec4 anchor_offset{};
  };
  static_assert(sizeof(Parameters) == 32);
  static_assert(sizeof(TreeParameters) == 304);

  std::shared_ptr<DescriptorSetLayout> layout_;
  std::shared_ptr<DescriptorSetLayout> wind_layout_;
  std::shared_ptr<DescriptorSet> empty_wind_set_;
  std::array<std::shared_ptr<Buffer>, 3> empty_wind_buffers_;
  std::array<std::shared_ptr<DescriptorSet>, 4> descriptor_sets_;
  std::array<std::shared_ptr<Buffer>, 2> vertices_;
  std::array<std::shared_ptr<Buffer>, 2> rods_;
  std::shared_ptr<Buffer> angles_;
  std::shared_ptr<Buffer> vertex_rods_;
  std::shared_ptr<Buffer> rod_angles_;
  std::shared_ptr<Buffer> node_rods_;
  std::shared_ptr<Buffer> predicted_;
  std::shared_ptr<Buffer> previous_;
  std::shared_ptr<Buffer> tree_parameters_;
  BufferUploadArena upload_arena_;
  std::shared_ptr<ComputePipeline> predict_;
  std::shared_ptr<ComputePipeline> solve_position_;
  std::shared_ptr<ComputePipeline> solve_rotation_;
  std::shared_ptr<ComputePipeline> update_velocity_;
  uint32_t vertex_count_ = 0;
  uint32_t rod_count_ = 0;
  uint32_t vertex_index_ = 0;
  uint32_t rod_index_ = 0;
  std::vector<TreeRange> ranges_;
  std::vector<glm::mat4> previous_models_;
  std::vector<glm::quat> initial_rotations_;
};
}  // namespace eco_sys_lab_package
