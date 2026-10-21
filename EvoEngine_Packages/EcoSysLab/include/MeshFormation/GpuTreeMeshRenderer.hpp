#pragma once

#include "GpuTreeMeshData.hpp"
#include "Material.hpp"
#include "RenderLayer.hpp"
#include "SubsurfaceProfile.hpp"

namespace eco_sys_lab_package {

class GpuTreeMeshRenderer : public std::enable_shared_from_this<GpuTreeMeshRenderer> {
 public:
  bool Prepare();
  void Register(const std::shared_ptr<Scene>& scene);

 private:
  void Initialize();
  uint32_t Draw(VkCommandBuffer command_buffer, const std::shared_ptr<GraphicsPipeline>& pipeline,
                const glm::ivec4& viewport, uint32_t geometry_pass, uint32_t pass, uint32_t camera, uint32_t light,
                uint32_t face, VkCullModeFlags cull_mode);

  struct TreeHandles {
    Handle branch{};
    Handle foliage{};
    bool has_branch = false;
    bool has_foliage = false;
  };

  std::shared_ptr<DescriptorSetLayout> layout_;
  std::vector<std::shared_ptr<DescriptorSet>> descriptor_sets_;
  std::vector<std::shared_ptr<Buffer>> node_buffers_;
  std::vector<std::shared_ptr<Buffer>> instance_index_buffers_;
  std::vector<std::shared_ptr<Buffer>> foliage_params_buffers_;
  std::vector<std::vector<TreeHandles>> tree_handles_;
  std::vector<bool> instance_indices_ready_;
  BufferUploadArena upload_arena_;
  std::shared_ptr<GraphicsPipeline> camera_pipeline_;
  std::shared_ptr<GraphicsPipeline> shadow_pipeline_;
  std::shared_ptr<GraphicsPipeline> foliage_opaque_pipeline_;
  std::shared_ptr<GraphicsPipeline> foliage_masked_pipeline_;
  std::shared_ptr<GraphicsPipeline> foliage_opaque_shadow_pipeline_;
  std::shared_ptr<GraphicsPipeline> foliage_masked_shadow_pipeline_;
  std::shared_ptr<Material> default_foliage_material_;
  std::shared_ptr<SubsurfaceProfile> default_foliage_profile_;
  uint32_t node_count_ = 0;
};

}  // namespace eco_sys_lab_package
