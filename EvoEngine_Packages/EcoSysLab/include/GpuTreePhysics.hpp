#pragma once

#include <array>

#include "ComputePipeline.hpp"
#include "DynamicTreeSkeleton.hpp"
#include "GpuTreePhysicsState.hpp"

namespace eco_sys_lab_package {
class GpuTreePhysics {
 public:
  struct TreeStep {
    DynamicTreeSkeleton::PhysicsSettings settings;
    bool running = true;
    GlobalTransform model{};
  };
  struct WindInput {
    std::shared_ptr<DescriptorSet> descriptor_set;
    uint32_t cell_count = 0;
    uint32_t bucket_count = 0;
    float cell_size = 1.0f;
  };

  bool Initialize(const DynamicTreeSkeleton::GpuRestState& rest_state);
  bool Initialize(const std::vector<DynamicTreeSkeleton::GpuRestState>& rest_states);
  uint32_t RegisterTree(const DynamicTreeSkeleton::GpuRestState& state, uint32_t id = UINT32_MAX);
  void RemoveTree(uint32_t id);
  void SetActiveTrees(const std::vector<uint32_t>& ids);
  void PrepareRegistry();
  void Step(const DynamicTreeSkeleton::PhysicsSettings& settings);
  void Step(const std::vector<TreeStep>& trees);
  void Step(const std::vector<TreeStep>& trees, const WindInput& wind);
  const std::shared_ptr<DescriptorSetLayout>& GetWindDescriptorSetLayout() const {
    return wind_layout_;
  }
  const std::vector<uint32_t>& GetActiveTreeIds() const {
    return active_ids_;
  }
  const std::shared_ptr<GpuTreePhysicsState>& GetTreeState(uint32_t id) const {
    return trees_[id];
  }
  uint32_t GetRodCount() const {
    uint32_t count = 0;
    for (const auto id : active_ids_)
      if (trees_[id])
        count += trees_[id]->rod_count;
    return count;
  }
  uint32_t GetRodGroupCount() const {
    return rod_group_count_;
  }
  const std::shared_ptr<Buffer>& GetRegistryBuffer() const {
    return registry_;
  }
  const std::shared_ptr<Buffer>& GetRodGroupBuffer() const {
    return rod_groups_;
  }
  const std::shared_ptr<Buffer>& GetVertexBuffer(uint32_t id = 0) const {
    const auto& t = *trees_[id];
    return t.vertices[t.vertex_index];
  }
  const std::shared_ptr<Buffer>& GetRodBuffer(uint32_t id = 0) const {
    const auto& t = *trees_[id];
    return t.rods[t.rod_index];
  }
  const std::shared_ptr<Buffer>& GetNodeRodBuffer(uint32_t id = 0) const {
    return trees_[id]->node_rods;
  }

 private:
  struct Parameters {
    uint32_t vertex_index = 0;
    uint32_t rod_index = 0;
    uint32_t substep = 0;
    uint32_t iteration = 0;
    uint32_t cell_count = 0;
    uint32_t bucket_count = 0;
    float cell_size = 1.0f;
    float padding = 0.0f;
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

  struct TreeAddresses {
    std::array<uint64_t, 2> vertices{}, rods{};
    uint64_t angles = 0, vertex_rods = 0, rod_angles = 0, node_rods = 0, predicted = 0, previous = 0;
    glm::uvec4 counts{};
  };
  static_assert(sizeof(TreeAddresses) == 96);
  bool Prepare();
  std::shared_ptr<DescriptorSetLayout> layout_, wind_layout_;
  std::shared_ptr<DescriptorSet> descriptor_set_, empty_wind_set_;
  std::array<std::shared_ptr<Buffer>, 2> empty_wind_buffers_;
  std::shared_ptr<Buffer> registry_, vertex_groups_, rod_groups_, tree_parameters_;
  BufferUploadArena upload_arena_;
  std::shared_ptr<ComputePipeline> predict_, solve_position_, solve_rotation_, update_velocity_;
  std::vector<std::shared_ptr<GpuTreePhysicsState>> trees_;
  std::vector<uint32_t> active_ids_;
  std::vector<TreeAddresses> cached_addresses_;
  bool groups_dirty_ = true;
  uint32_t vertex_group_count_ = 0, rod_group_count_ = 0;
  std::vector<std::pair<uint32_t, std::shared_ptr<GpuTreePhysicsState>>> retired_states_;
  std::vector<std::pair<uint32_t, std::shared_ptr<Buffer>>> retired_buffers_;
  std::vector<std::pair<uint32_t, std::shared_ptr<DescriptorSet>>> retired_sets_;
};
}  // namespace eco_sys_lab_package
