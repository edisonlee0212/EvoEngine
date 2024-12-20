#pragma once
#include "Camera.hpp"
#include "Entity.hpp"
#include "Lights.hpp"
#include "MeshRenderer.hpp"
#include "SkinnedMeshRenderer.hpp"
#include "StrandsRenderer.hpp"
namespace evo_engine {
struct RenderSettings {
  bool stable_fit = true;
  float max_shadow_distance = 100;
  float shadow_cascade_split[4] = {0.075f, 0.15f, 0.3f, 1.0f};
  bool enable_debug_visualization = false;

  int pcf_sample_amount = 32;
  int blocker_search_amount = 8;
  float seam_fix_ratio = 0.1f;
  float gamma = 1.f;

  float strands_subdivision_x_factor = 50.0f;
  float strands_subdivision_y_factor = 50.0f;
  int strands_subdivision_max_x = 15;
  int strands_subdivision_max_y = 8;
  void OnInspect(const std::shared_ptr<EditorLayer>& editor_layer);
};

struct RenderInstancePushConstant {
  int instance_index = 0;
  int camera_index = 0;
  int light_split_index = 0;
};
struct RayTracingPushConstant {
  uint32_t camera_index = 0;
  uint32_t frame_id = 0;
};

enum class RenderInstanceType {
  Unknown,
  FromRenderer,
  FromApi,
};

struct IRenderInstance {
  uint32_t instance_index = 0;
  RenderInstanceType command_type = RenderInstanceType::Unknown;
  Entity owner = Entity();
  Handle renderer_handle = 0;
  bool entity_selected = false;
  GlobalTransform model = {};
  float line_width = 1.0f;
  VkCullModeFlags cull_mode = VK_CULL_MODE_BACK_BIT;
  VkPolygonMode polygon_mode = VK_POLYGON_MODE_FILL;
  bool cast_shadow = true;

  virtual uint32_t Render(VkCommandBuffer vk_command_buffer,
                          const RenderInstancePushConstant& render_instance_push_constant,
                          const std::shared_ptr<GraphicsPipeline>& graphics_pipeline) const = 0;
};

struct MeshRenderInstance : IRenderInstance {
  uint32_t material_version;
  uint32_t mesh_version;
  std::shared_ptr<Material> material;
  std::shared_ptr<Mesh> mesh;
  uint32_t meshlet_size = 0;
  bool operator!=(const MeshRenderInstance& other) const;
  uint32_t Render(VkCommandBuffer vk_command_buffer, const RenderInstancePushConstant& render_instance_push_constant,
                  const std::shared_ptr<GraphicsPipeline>& graphics_pipeline) const override;
};

struct SkinnedMeshRenderInstance : IRenderInstance {
  uint32_t material_version;
  uint32_t skinned_mesh_version;
  uint32_t bone_matrices_version;
  std::shared_ptr<Material> material;
  std::shared_ptr<SkinnedMesh> skinned_mesh;
  std::shared_ptr<BoneMatrices> bone_matrices;  // We require the skinned mesh renderer to provide bones.

  uint32_t skinned_meshlet_size = 0;
  bool operator!=(const SkinnedMeshRenderInstance& other) const;
  uint32_t Render(VkCommandBuffer vk_command_buffer, const RenderInstancePushConstant& render_instance_push_constant,
                  const std::shared_ptr<GraphicsPipeline>& graphics_pipeline) const override;
};

struct InstancedRenderInstance : IRenderInstance {
  uint32_t material_version;
  uint32_t mesh_version;
  uint32_t particle_info_list_version;
  std::shared_ptr<Material> material;
  std::shared_ptr<Mesh> mesh;
  std::shared_ptr<ParticleInfoList> particle_infos;

  uint32_t meshlet_size = 0;

  bool operator!=(const InstancedRenderInstance& other) const;
  uint32_t Render(VkCommandBuffer vk_command_buffer, const RenderInstancePushConstant& render_instance_push_constant,
                  const std::shared_ptr<GraphicsPipeline>& graphics_pipeline) const override;
};

struct StrandsRenderInstance : IRenderInstance {
  uint32_t material_version;
  uint32_t strands_version;
  std::shared_ptr<Material> material;
  std::shared_ptr<Strands> strands;
  uint32_t strand_meshlet_size = 0;
  bool operator!=(const StrandsRenderInstance& other) const;
  uint32_t Render(VkCommandBuffer vk_command_buffer, const RenderInstancePushConstant& render_instance_push_constant,
                  const std::shared_ptr<GraphicsPipeline>& graphics_pipeline) const override;
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

class RenderInstanceStorage {
  struct RenderInfoBlock {
    glm::vec4 split_distances = {};
    alignas(4) int pcf_sample_amount = 32;
    alignas(4) int blocker_search_amount = 8;
    alignas(4) float seam_fix_ratio = 0.1f;
    alignas(4) float gamma = 1.f;

    alignas(4) float strands_subdivision_x_factor = 50.0f;
    alignas(4) float strands_subdivision_y_factor = 50.0f;
    alignas(4) int strands_subdivision_max_x = 15;
    alignas(4) int strands_subdivision_max_y = 8;

    alignas(4) int directional_light_size = 0;
    alignas(4) int point_light_size = 0;
    alignas(4) int spot_light_size = 0;
    alignas(4) int brdflut_texture_index = 0;

    alignas(4) int debug_visualization = 0;
    alignas(4) int padding0 = 0;
    alignas(4) int padding1 = 0;
    alignas(4) int padding2 = 0;
    void Apply(const RenderSettings& target_render_settings);

    bool operator!=(const RenderInfoBlock& other) const;
  };
  struct EnvironmentInfoBlock {
    glm::vec4 background_color = glm::vec4(1.0f, 1.0f, 1.0f, 1.0f);
    alignas(4) float environmental_map_gamma = 2.2f;
    alignas(4) float environmental_lighting_intensity = 0.8f;
    alignas(4) float background_intensity = 1.0f;
    alignas(4) float environmental_padding2 = 0.0f;
    bool operator!=(const EnvironmentInfoBlock& other) const;
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
    bool operator!=(const InstanceInfoBlock& other) const;
  };
  struct MaterialInfoBlock {
    alignas(4) int albedo_texture_index = -1;
    alignas(4) int normal_texture_index = -1;
    alignas(4) int metallic_texture_index = -1;
    alignas(4) int roughness_texture_index = -1;

    alignas(4) int ao_texture_index = -1;
    alignas(4) int cast_shadow = true;
    alignas(4) int receive_shadow = true;
    alignas(4) int enable_shadow = true;

    glm::vec4 albedo_color_val = glm::vec4(1.0f);
    glm::vec4 subsurface_color = glm::vec4(1.0f, 1.0f, 1.0f, 0.0f);
    glm::vec4 subsurface_radius = glm::vec4(1.0f, 1.0f, 1.0f, 0.0f);

    alignas(4) float metallic_val = 0.5f;
    alignas(4) float roughness_val = 0.5f;
    alignas(4) float ao_val = 1.0f;
    alignas(4) float emission_val = 0.0f;
    void Apply(const std::shared_ptr<Material>& target_material);
    bool operator!=(const MaterialInfoBlock& other) const;
  };

  std::unordered_map<Handle, int> material_indices_;
  /**
   * \brief Use this to find render instance via entity handle.
   */
  std::unordered_map<Handle, int> instance_indices_;
  /**
   * \brief Use this to find entity via render instance index.
   */
  std::unordered_map<int, Handle> instance_handles_;
  /**
   * \brief Use this to find camera via camera index.
   */
  std::unordered_map<Handle, int> camera_indices_;

  std::vector<MaterialInfoBlock> material_info_blocks_{};
  std::vector<InstanceInfoBlock> instance_info_blocks_{};
  RenderInfoBlock render_info_block = {};
  EnvironmentInfoBlock environment_info_block = {};

  std::vector<DirectionalLightInfoBlock> directional_light_info_blocks_;
  std::vector<PointLightInfoBlock> point_light_info_blocks_;
  std::vector<SpotLightInfoBlock> spot_light_info_blocks_;

  std::vector<CameraInfoBlock> camera_info_blocks_{};
  MeshRenderInstanceCollection deferred_render_instances;
  SkinnedMeshRenderInstanceCollection deferred_skinned_render_instances;
  InstancedRenderInstanceCollection deferred_instanced_render_instances;
  StrandsRenderInstanceCollection deferred_strands_render_instances;

  MeshRenderInstanceCollection transparent_render_instances;
  SkinnedMeshRenderInstanceCollection transparent_skinned_render_instances;
  InstancedRenderInstanceCollection transparent_instanced_render_instances;
  StrandsRenderInstanceCollection transparent_strands_render_instances;

  friend class RenderLayer;
  friend class CpuRayTracer;
  void CollectEntityRenderers(const std::shared_ptr<Scene>& target_scene, Bound& world_bound);
  void CollectLights(const std::shared_ptr<Scene>& target_scene, const Bound& world_bound);
  void CollectEnvironment(const std::shared_ptr<Scene>& target_scene);
  uint32_t geometry_storage_version = 0;
  uint32_t texture_storage_version = 0;

  bool RegisterEntity(const std::shared_ptr<Scene>& target_scene, const Entity& owner,
                      const std::shared_ptr<MeshRenderer>& mesh_renderer, glm::vec3& min_bound, glm::vec3& max_bound);
  bool RegisterEntity(const std::shared_ptr<Scene>& target_scene, const Entity& owner,
                      const std::shared_ptr<SkinnedMeshRenderer>& skinned_mesh_renderer, glm::vec3& min_bound,
                      glm::vec3& max_bound);
  bool RegisterEntity(const std::shared_ptr<Scene>& target_scene, const Entity& owner,
                      const std::shared_ptr<Particles>& particles, glm::vec3& min_bound, glm::vec3& max_bound);
  bool RegisterEntity(const std::shared_ptr<Scene>& target_scene, const Entity& owner,
                      const std::shared_ptr<StrandsRenderer>& strands_renderer, glm::vec3& min_bound,
                      glm::vec3& max_bound);
  [[nodiscard]] int RegisterMaterial(const Handle& handle, const MaterialInfoBlock& material_info_block);
  [[nodiscard]] int RegisterInstance(const Handle& handle, const InstanceInfoBlock& instance_info_block);
  [[nodiscard]] int RegisterCamera(const Handle& handle, const CameraInfoBlock& camera_info_block);

 public:
  RenderInstanceStorage();
  bool operator!=(const RenderInstanceStorage& other) const;
  uint32_t RegisterMeshDrawCommand(const std::shared_ptr<Mesh>& mesh, const std::shared_ptr<Material>& material,
                                   const GlobalTransform& model, bool cast_shadow);
  [[nodiscard]] int RegisterMaterial(const std::shared_ptr<Material>& material);
  std::vector<std::pair<GlobalTransform, std::shared_ptr<Camera>>> cameras;
  RenderSettings render_settings{};
  std::shared_ptr<Buffer> material_info_descriptor_buffer = {};
  std::shared_ptr<Buffer> instance_info_descriptor_buffer = {};
  std::shared_ptr<Buffer> environment_info_descriptor_buffer = {};
  std::shared_ptr<Buffer> directional_light_info_descriptor_buffer = {};
  std::shared_ptr<Buffer> point_light_info_descriptor_buffer = {};
  std::shared_ptr<Buffer> spot_light_info_descriptor_buffer = {};
  std::shared_ptr<Buffer> render_info_descriptor_buffer = {};
  std::shared_ptr<Buffer> camera_info_descriptor_buffer = {};

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
  static void CollectCameras(const std::shared_ptr<Scene>& target_scene,
                             std::vector<std::pair<GlobalTransform, std::shared_ptr<Camera>>>& cameras);
  static void CalculateLodFactor(const std::shared_ptr<Scene>& scene, const glm::vec3& view_position,
                                 float max_distance);
  void BuildFromScene(const RenderSettings& render_settings, const std::shared_ptr<Scene>& scene, Bound& world_bound);
  void UpdateTopLevelAccelerationStructure(const std::shared_ptr<Scene>& scene);

  [[nodiscard]] int GetMaterialIndex(const Handle& handle);
  [[nodiscard]] int GetInstanceIndex(const Handle& handle);
  [[nodiscard]] int GetCameraIndex(const Handle& handle);
  [[nodiscard]] Handle GetInstanceHandle(int index);

  void Upload() const;

  [[nodiscard]] const std::vector<MaterialInfoBlock>& GetMaterialInfoBlocks() const;
  [[nodiscard]] const std::vector<InstanceInfoBlock>& GetInstanceInfoBlocks() const;
};
}  // namespace evo_engine