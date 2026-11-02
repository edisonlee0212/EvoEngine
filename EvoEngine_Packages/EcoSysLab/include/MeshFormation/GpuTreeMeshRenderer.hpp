#pragma once

#include <array>
#include <unordered_map>
#include "ComputePipeline.hpp"
#include "GpuTreeMeshData.hpp"
#include "Material.hpp"
#include "RenderLayer.hpp"
#include "SubsurfaceProfile.hpp"

namespace eco_sys_lab_package {
class GpuTreePhysics;
struct GpuTreePhysicsState;
class Tree;

class GpuTreeMeshRenderer : public std::enable_shared_from_this<GpuTreeMeshRenderer> {
 public:
  bool Prepare();
  void Register(const std::shared_ptr<Scene>& scene);
  bool directional_shadow_impostors = true;

 private:
  void Initialize();
  struct TreeNodeCache;
  uint32_t Draw(const std::shared_ptr<TreeNodeCache>& tree, VkCommandBuffer command_buffer,
                const std::shared_ptr<GraphicsPipeline>& pipeline, const glm::ivec4& viewport, uint32_t geometry_pass,
                uint32_t pass, uint32_t camera, uint32_t light, uint32_t face, VkCullModeFlags cull_mode);

  struct TreeHandles {
    Handle branch{};
    Handle foliage{};
    bool has_branch = false;
    bool has_foliage = false;
  };

  struct TreeFrame {
    std::shared_ptr<Buffer> nodes, rest, instances, foliage;
    std::shared_ptr<DescriptorSet> draw_set, deform_set;
    std::shared_ptr<GpuTreePhysicsState> physics;
    TreeHandles handles;
    uint64_t generation = UINT64_MAX;
    int branch_material = -1, foliage_material = -1;
    uint32_t node_count = 0;
    bool indices_ready = false, params_ready = false;
    GpuTreeFoliageParams params{};
    glm::vec2 tessellation{};
  };
  struct TreeNodeCache {
    std::weak_ptr<Tree> tree;
    std::weak_ptr<GpuTreePhysicsState> physics;
    std::array<float, 18> parameters{};
    uint64_t shoot_revision = UINT64_MAX, root_revision = UINT64_MAX, generation = 0;
    uint32_t shoot_count = 0;
    std::vector<GpuTreeNode> nodes;
    Bound local_bound;
    float foliage_padding = -1;
    std::vector<TreeFrame> frames;
  };

  std::shared_ptr<DescriptorSetLayout> layout_;
  std::shared_ptr<DescriptorSetLayout> deform_layout_;
  std::shared_ptr<ComputePipeline> deform_pipeline_;
  std::shared_ptr<GraphicsPipeline> camera_pipeline_;
  std::shared_ptr<GraphicsPipeline> shadow_pipeline_;
  std::shared_ptr<GraphicsPipeline> directional_shadow_pipeline_;
  std::shared_ptr<GraphicsPipeline> foliage_opaque_pipeline_;
  std::shared_ptr<GraphicsPipeline> foliage_masked_pipeline_;
  std::shared_ptr<GraphicsPipeline> foliage_opaque_shadow_pipeline_;
  std::shared_ptr<GraphicsPipeline> foliage_masked_shadow_pipeline_;
  std::shared_ptr<Material> default_foliage_material_;
  std::shared_ptr<SubsurfaceProfile> default_foliage_profile_;
  std::weak_ptr<Scene> cached_scene_;
  std::unordered_map<Entity, std::shared_ptr<TreeNodeCache>, Entity> tree_node_cache_;
  std::vector<std::pair<uint32_t, std::shared_ptr<TreeNodeCache>>> retired_trees_;
  BufferUploadArena upload_arena_;
};

}  // namespace eco_sys_lab_package
