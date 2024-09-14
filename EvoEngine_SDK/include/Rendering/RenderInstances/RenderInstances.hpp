#pragma once
#include "Entity.hpp"
#include "MeshRenderer.hpp"
#include "SkinnedMeshRenderer.hpp"
#include "StrandsRenderer.hpp"

namespace evo_engine {
enum class RenderCommandType {
  Unknown,
  FromRenderer,
  FromApi,
};

struct InstanceInfoBlock {
  GlobalTransform model = {};
  uint32_t material_index = 0;
  uint32_t entity_selected = 0;
  uint32_t meshlet_index_offset = 0;
  uint32_t meshlet_size = 0;
};

struct RenderInstance {
  uint32_t instance_index = 0;
  RenderCommandType command_type = RenderCommandType::Unknown;
  Entity owner = Entity();
  std::shared_ptr<Mesh> mesh;
  bool cast_shadow = true;

  uint32_t meshlet_size = 0;

  float line_width = 1.0f;
  VkCullModeFlags cull_mode = VK_CULL_MODE_BACK_BIT;
  VkPolygonMode polygon_mode = VK_POLYGON_MODE_FILL;
};

struct SkinnedRenderInstance {
  uint32_t instance_index = 0;
  RenderCommandType command_type = RenderCommandType::Unknown;
  Entity m_owner = Entity();
  std::shared_ptr<SkinnedMesh> skinned_mesh;
  bool cast_shadow = true;
  std::shared_ptr<BoneMatrices> bone_matrices;  // We require the skinned mesh renderer to provide bones.

  uint32_t skinned_meshlet_size = 0;

  float line_width = 1.0f;
  VkCullModeFlags cull_mode = VK_CULL_MODE_BACK_BIT;
  VkPolygonMode polygon_mode = VK_POLYGON_MODE_FILL;
};

struct InstancedRenderInstance {
  uint32_t instance_index = 0;
  RenderCommandType command_type = RenderCommandType::Unknown;
  Entity owner = Entity();
  std::shared_ptr<Mesh> mesh;
  bool cast_shadow = true;
  std::shared_ptr<ParticleInfoList> particle_infos;

  uint32_t meshlet_size = 0;

  float line_width = 1.0f;
  VkCullModeFlags cull_mode = VK_CULL_MODE_BACK_BIT;
  VkPolygonMode polygon_mode = VK_POLYGON_MODE_FILL;
};

struct StrandsRenderInstance {
  uint32_t instance_index = 0;
  RenderCommandType command_type = RenderCommandType::Unknown;
  Entity m_owner = Entity();
  std::shared_ptr<Strands> m_strands;
  bool cast_shadow = true;

  uint32_t strand_meshlet_size = 0;

  float line_width = 1.0f;
  VkCullModeFlags cull_mode = VK_CULL_MODE_BACK_BIT;
  VkPolygonMode polygon_mode = VK_POLYGON_MODE_FILL;
};

struct RenderInstanceCollection {
  std::vector<RenderInstance> render_commands;
  void Dispatch(const std::function<void(const RenderInstance&)>& command_action) const;
};
struct SkinnedRenderInstanceCollection {
  std::vector<SkinnedRenderInstance> render_commands;
  void Dispatch(const std::function<void(const SkinnedRenderInstance&)>& command_action) const;
};
struct StrandsRenderInstanceCollection {
  std::vector<StrandsRenderInstance> render_commands;
  void Dispatch(const std::function<void(const StrandsRenderInstance&)>& command_action) const;
};
struct InstancedRenderInstanceCollection {
  std::vector<InstancedRenderInstance> render_commands;
  void Dispatch(const std::function<void(const InstancedRenderInstance&)>& command_action) const;
};

class RenderInstances {
  std::unordered_map<Handle, uint32_t> material_indices_;

  /**
   * \brief Use this to find render instance via entity handle.
   */
  std::unordered_map<Handle, uint32_t> instance_indices_;
  /**
   * \brief Use this to find entity via render instance index.
   */
  std::unordered_map<uint32_t, Handle> instance_handles_;

  std::vector<MaterialInfoBlock> material_info_blocks_{};
  std::vector<InstanceInfoBlock> instance_info_blocks_{};
  friend class RenderLayer;
  friend class RayTracer;

 public:
  std::shared_ptr<Scene> target_scene;
  RenderInstanceCollection deferred_render_instances;
  SkinnedRenderInstanceCollection deferred_skinned_render_instances;
  InstancedRenderInstanceCollection deferred_instanced_render_instances;
  StrandsRenderInstanceCollection deferred_strands_render_instances;

  RenderInstanceCollection transparent_render_instances;
  SkinnedRenderInstanceCollection transparent_skinned_render_instances;
  InstancedRenderInstanceCollection transparent_instanced_render_instances;
  StrandsRenderInstanceCollection transparent_strands_render_instances;

  std::vector<std::shared_ptr<Buffer>> material_info_descriptor_buffers = {};
  std::vector<std::shared_ptr<Buffer>> instance_info_descriptor_buffers = {};

  void Clear();

  RenderInstances(uint32_t max_frame_in_flight);

  bool TryRegisterRenderer(const Entity& owner, const std::shared_ptr<MeshRenderer>& mesh_renderer,
                           glm::vec3& min_bound, glm::vec3& max_bound);
  bool TryRegisterRenderer(const Entity& owner, const std::shared_ptr<SkinnedMeshRenderer>& skinned_mesh_renderer,
                           glm::vec3& min_bound, glm::vec3& max_bound);
  bool TryRegisterRenderer(const Entity& owner, const std::shared_ptr<Particles>& particles, glm::vec3& min_bound,
                           glm::vec3& max_bound);
  bool TryRegisterRenderer(const Entity& owner, const std::shared_ptr<StrandsRenderer>& strands_renderer,
                           glm::vec3& min_bound, glm::vec3& max_bound);

  static void CalculateLodFactor(const std::shared_ptr<Scene>& scene, const glm::vec3& view_position,
                                 float max_distance);
  [[nodiscard]] bool UpdateRenderInstances(const std::shared_ptr<Scene>& scene, Bound& world_bound);

  [[nodiscard]] uint32_t RegisterMaterialIndex(const Handle& handle, const MaterialInfoBlock& material_info_block);
  [[nodiscard]] uint32_t RegisterInstanceIndex(const Handle& handle, const InstanceInfoBlock& instance_info_block);

  [[nodiscard]] uint32_t GetMaterialIndex(const Handle& handle);
  [[nodiscard]] uint32_t GetInstanceIndex(const Handle& handle);
  [[nodiscard]] Handle GetInstanceHandle(uint32_t index);

  void Upload(uint32_t current_frame_index) const;

  const std::vector<MaterialInfoBlock>& GetMaterialInfoBlocks() const;
  const std::vector<InstanceInfoBlock>& GetInstanceInfoBlocks() const;
};
}  // namespace evo_engine