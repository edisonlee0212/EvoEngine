#pragma once

#include <array>

#include "ComputePipeline.hpp"
#include "DynamicTreeSkeleton.hpp"
#include "GpuTreePhysicsLod.hpp"
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
  uint32_t RegisterTree(const DynamicTreeSkeleton::GpuRestState& state, uint32_t id = UINT32_MAX,
                        bool build_lod = true);
  void InitializeLod(uint32_t id, const DynamicTreeSkeleton::GpuRestState& rest);
  void RemoveTree(uint32_t id);
  void SetActiveTrees(const std::vector<uint32_t>& ids);
  void PrepareRegistry();
  void ResetLodSelection(uint32_t id);
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

  void DownloadVertices(std::vector<DynamicTreeSkeleton::GpuRestState::Vertex>& output, uint32_t id = 0,
                        uint32_t copy = UINT32_MAX) const;
  void DownloadRods(std::vector<DynamicTreeSkeleton::GpuRestState::Rod>& output, uint32_t id = 0,
                    uint32_t copy = UINT32_MAX) const;

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
    glm::vec4 material{1, 0, 0, 0};
  };
  static_assert(sizeof(Parameters) == 32);
  static_assert(sizeof(TreeParameters) == 320);

  using TreeAddresses = GpuTreePhysicsAddresses;
  bool Prepare();
  std::shared_ptr<DescriptorSetLayout> layout_, wind_layout_;
  std::shared_ptr<DescriptorSet> descriptor_set_, empty_wind_set_;
  std::array<std::shared_ptr<Buffer>, 2> empty_wind_buffers_;
  std::shared_ptr<Buffer> registry_, vertex_groups_, rod_groups_, tree_parameters_, coefficient_groups_;
  BufferUploadArena upload_arena_;
  std::shared_ptr<ComputePipeline> predict_, solve_position_, solve_rotation_, update_velocity_, prepare_coefficients_;
  std::shared_ptr<DescriptorSetLayout> lod_layout_;
  std::shared_ptr<DescriptorSet> lod_set_, lod_solver_set_;
  std::array<std::shared_ptr<ComputePipeline>, 7> lod_pipelines_;
  std::shared_ptr<Buffer> selected_registry_, lod_active_ids_, lod_vertex_groups_, lod_rod_groups_,
      lod_coefficient_groups_, lod_error_groups_, lod_dispatch_;
  bool lod_was_requested_ = false;
  bool reference_ = false;
  std::vector<std::shared_ptr<GpuTreePhysicsState>> trees_;
  std::vector<uint32_t> active_ids_;
  std::vector<TreeAddresses> cached_addresses_;
  bool groups_dirty_ = true;
  uint32_t vertex_group_count_ = 0, rod_group_count_ = 0, coefficient_group_count_ = 0;
  std::vector<std::pair<uint32_t, std::shared_ptr<GpuTreePhysicsState>>> retired_states_;
  std::vector<std::pair<uint32_t, std::shared_ptr<Buffer>>> retired_buffers_;
  std::vector<std::pair<uint32_t, std::shared_ptr<DescriptorSet>>> retired_sets_;
};
}  // namespace eco_sys_lab_package
