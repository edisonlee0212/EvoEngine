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
  uint32_t triangle_offset = 0;
  uint32_t meshlet_index_offset = 0;
  uint32_t meshlet_size = 0;

  uint32_t entity_selected = 0;
  uint32_t padding0 = 0;
  uint32_t padding1 = 0;
  uint32_t padding2 = 0;
};

struct MeshRenderInstance {
  uint32_t instance_index = 0;
  RenderCommandType command_type = RenderCommandType::Unknown;
  Entity owner = Entity();

  GlobalTransform model = {};
  uint32_t material_version;
  uint32_t mesh_version;
  std::shared_ptr<Material> material;
  std::shared_ptr<Mesh> mesh;


  bool cast_shadow = true;

  uint32_t meshlet_size = 0;

  float line_width = 1.0f;
  VkCullModeFlags cull_mode = VK_CULL_MODE_BACK_BIT;
  VkPolygonMode polygon_mode = VK_POLYGON_MODE_FILL;
  bool operator!=(const MeshRenderInstance& other) const;
};

struct SkinnedMeshRenderInstance {
  uint32_t instance_index = 0;
  RenderCommandType command_type = RenderCommandType::Unknown;
  Entity owner = Entity();

  GlobalTransform model = {};
  uint32_t material_version;
  uint32_t skinned_mesh_version;
  uint32_t bone_matrices_version;
  std::shared_ptr<Material> material;
  std::shared_ptr<SkinnedMesh> skinned_mesh;
  bool cast_shadow = true;
  std::shared_ptr<BoneMatrices> bone_matrices;  // We require the skinned mesh renderer to provide bones.

  uint32_t skinned_meshlet_size = 0;

  float line_width = 1.0f;
  VkCullModeFlags cull_mode = VK_CULL_MODE_BACK_BIT;
  VkPolygonMode polygon_mode = VK_POLYGON_MODE_FILL;
  bool operator!=(const SkinnedMeshRenderInstance& other) const;
};

struct InstancedRenderInstance {
  uint32_t instance_index = 0;
  RenderCommandType command_type = RenderCommandType::Unknown;
  Entity owner = Entity();

  GlobalTransform model = {};
  uint32_t material_version;
  uint32_t mesh_version;
  uint32_t particle_info_list_version;
  std::shared_ptr<Material> material;
  std::shared_ptr<Mesh> mesh;
  bool cast_shadow = true;
  std::shared_ptr<ParticleInfoList> particle_infos;

  uint32_t meshlet_size = 0;

  float line_width = 1.0f;
  VkCullModeFlags cull_mode = VK_CULL_MODE_BACK_BIT;
  VkPolygonMode polygon_mode = VK_POLYGON_MODE_FILL;
  bool operator!=(const InstancedRenderInstance& other) const;
};

struct StrandsRenderInstance {
  uint32_t instance_index = 0;
  RenderCommandType command_type = RenderCommandType::Unknown;
  Entity owner = Entity();

  GlobalTransform model = {};
  uint32_t material_version;
  uint32_t strands_version;
  std::shared_ptr<Material> material;
  std::shared_ptr<Strands> strands;
  bool cast_shadow = true;

  uint32_t strand_meshlet_size = 0;

  float line_width = 1.0f;
  VkCullModeFlags cull_mode = VK_CULL_MODE_BACK_BIT;
  VkPolygonMode polygon_mode = VK_POLYGON_MODE_FILL;
  bool operator!=(const StrandsRenderInstance& other) const;
};

struct MeshRenderInstanceCollection {
  std::vector<MeshRenderInstance> render_commands;
  bool operator!=(const MeshRenderInstanceCollection& other) const;
};
struct SkinnedMeshRenderInstanceCollection {
  std::vector<SkinnedMeshRenderInstance> render_commands;
  bool operator!=(const SkinnedMeshRenderInstanceCollection& other) const;
};
struct StrandsRenderInstanceCollection {
  std::vector<StrandsRenderInstance> render_commands;
  bool operator!=(const StrandsRenderInstanceCollection& other) const;
};
struct InstancedRenderInstanceCollection {
  std::vector<InstancedRenderInstance> render_commands;
  bool operator!=(const InstancedRenderInstanceCollection& other) const;
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
  friend class CpuRayTracer;

  void Collect(Bound& world_bound);

 public:
  RenderInstances();
  [[nodiscard]] bool MeshInstancesUpdated(const RenderInstances& other) const;
  [[nodiscard]] bool SkinnedMeshInstancesUpdated(const RenderInstances& other) const;
  [[nodiscard]] bool InstancedMeshInstancesUpdated(const RenderInstances& other) const;
  [[nodiscard]] bool StrandsInstancesUpdated(const RenderInstances& other) const;
  std::shared_ptr<Scene> target_scene;
  MeshRenderInstanceCollection deferred_render_instances;
  SkinnedMeshRenderInstanceCollection deferred_skinned_render_instances;
  InstancedRenderInstanceCollection deferred_instanced_render_instances;
  StrandsRenderInstanceCollection deferred_strands_render_instances;

  MeshRenderInstanceCollection transparent_render_instances;
  SkinnedMeshRenderInstanceCollection transparent_skinned_render_instances;
  InstancedRenderInstanceCollection transparent_instanced_render_instances;
  StrandsRenderInstanceCollection transparent_strands_render_instances;

  std::shared_ptr<Buffer> material_info_descriptor_buffer = {};
  std::shared_ptr<Buffer> instance_info_descriptor_buffer = {};

  std::shared_ptr<TopLevelAccelerationStructure> mesh_top_level_acceleration_structure{}; 


  std::vector<VkDrawIndexedIndirectCommand> mesh_draw_indexed_indirect_commands;
  std::shared_ptr<Buffer> mesh_draw_indexed_indirect_commands_buffer;

  std::vector<VkDrawMeshTasksIndirectCommandEXT> mesh_draw_mesh_tasks_indirect_commands;
  std::shared_ptr<Buffer> mesh_draw_mesh_tasks_indirect_commands_buffer;

  uint32_t total_mesh_triangles = 0;
  uint32_t total_skinned_mesh_triangles = 0;
  uint32_t total_instanced_mesh_triangles = 0;
  uint32_t total_strands_segments = 0;

  void Clear();

  bool TryRegisterRenderer(const Entity& owner,
                                  const std::shared_ptr<MeshRenderer>& mesh_renderer,
                           glm::vec3& min_bound, glm::vec3& max_bound);
  bool TryRegisterRenderer(const Entity& owner,
                                  const std::shared_ptr<SkinnedMeshRenderer>& skinned_mesh_renderer,
                           glm::vec3& min_bound, glm::vec3& max_bound);
  bool TryRegisterRenderer(const Entity& owner,
                                  const std::shared_ptr<Particles>& particles,
                                  glm::vec3& min_bound,
                           glm::vec3& max_bound);
  bool TryRegisterRenderer(const Entity& owner,
                                  const std::shared_ptr<StrandsRenderer>& strands_renderer,
                           glm::vec3& min_bound, glm::vec3& max_bound);

  static void CalculateLodFactor(const std::shared_ptr<Scene>& scene, const glm::vec3& view_position,
                                 float max_distance);
  [[nodiscard]] bool UpdateRenderInstances(const std::shared_ptr<Scene>& scene, Bound& world_bound);
  void UpdateTopLevelAccelerationStructure(const std::shared_ptr<Scene>& scene);
  [[nodiscard]] uint32_t RegisterMaterialIndex(const Handle& handle, const MaterialInfoBlock& material_info_block);
  [[nodiscard]] uint32_t RegisterInstanceIndex(const Handle& handle, const InstanceInfoBlock& instance_info_block);

  [[nodiscard]] uint32_t GetMaterialIndex(const Handle& handle);
  [[nodiscard]] uint32_t GetInstanceIndex(const Handle& handle);
  [[nodiscard]] Handle GetInstanceHandle(uint32_t index);

  void Upload() const;

  [[nodiscard]] const std::vector<MaterialInfoBlock>& GetMaterialInfoBlocks() const;
  [[nodiscard]] const std::vector<InstanceInfoBlock>& GetInstanceInfoBlocks() const;
};
}  // namespace evo_engine