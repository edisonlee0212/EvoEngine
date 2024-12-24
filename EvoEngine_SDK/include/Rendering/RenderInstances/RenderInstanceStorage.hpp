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

class RenderInstanceStorage {
 public:
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

  struct IRenderInstance {
    uint32_t instance_index = 0;
    uint32_t material_index = 0;
    RenderInstanceType command_type = RenderInstanceType::Unknown;
    Entity owner = Entity();
    Handle entity_handle;
    Handle renderer_handle = 0;
    bool entity_selected = false;
    GlobalTransform model = {};
    float line_width = 1.0f;
    VkCullModeFlags cull_mode = VK_CULL_MODE_BACK_BIT;
    VkPolygonMode polygon_mode = VK_POLYGON_MODE_FILL;
    bool cast_shadow = true;
    uint32_t material_version;
    uint32_t geometry_version;
    std::shared_ptr<Material> material;
    virtual void Apply(InstanceInfoBlock& instance_info_block) const = 0;
    virtual uint32_t Render(VkCommandBuffer vk_command_buffer,
                            const RenderInstancePushConstant& render_instance_push_constant,
                            const std::shared_ptr<GraphicsPipeline>& graphics_pipeline) const = 0;
  };

  struct ExternalRenderInstance : IRenderInstance {
    bool operator!=(const ExternalRenderInstance& other) const;
    void Apply(InstanceInfoBlock& instance_info_block) const override;
    uint32_t Render(VkCommandBuffer vk_command_buffer, const RenderInstancePushConstant& render_instance_push_constant,
                    const std::shared_ptr<GraphicsPipeline>& graphics_pipeline) const override;
  };

  struct MeshRenderInstance : IRenderInstance {
    std::shared_ptr<Mesh> mesh;
    bool operator!=(const MeshRenderInstance& other) const;
    void Apply(InstanceInfoBlock& instance_info_block) const override;
    uint32_t Render(VkCommandBuffer vk_command_buffer, const RenderInstancePushConstant& render_instance_push_constant,
                    const std::shared_ptr<GraphicsPipeline>& graphics_pipeline) const override;
  };

  struct SkinnedMeshRenderInstance : IRenderInstance {
    uint32_t bone_matrices_version;
    std::shared_ptr<SkinnedMesh> skinned_mesh;
    std::shared_ptr<BoneMatrices> bone_matrices;  // We require the skinned mesh renderer to provide bones.
    bool operator!=(const SkinnedMeshRenderInstance& other) const;
    void Apply(InstanceInfoBlock& instance_info_block) const override;
    uint32_t Render(VkCommandBuffer vk_command_buffer, const RenderInstancePushConstant& render_instance_push_constant,
                    const std::shared_ptr<GraphicsPipeline>& graphics_pipeline) const override;
  };

  struct InstancedRenderInstance : IRenderInstance {
    uint32_t particle_info_list_version;
    std::shared_ptr<Mesh> mesh;
    std::shared_ptr<ParticleInfoList> particle_infos;
    bool operator!=(const InstancedRenderInstance& other) const;
    void Apply(InstanceInfoBlock& instance_info_block) const override;
    uint32_t Render(VkCommandBuffer vk_command_buffer, const RenderInstancePushConstant& render_instance_push_constant,
                    const std::shared_ptr<GraphicsPipeline>& graphics_pipeline) const override;
  };

  struct StrandsRenderInstance : IRenderInstance {
    std::shared_ptr<Strands> strands;
    bool operator!=(const StrandsRenderInstance& other) const;
    void Apply(InstanceInfoBlock& instance_info_block) const override;
    uint32_t Render(VkCommandBuffer vk_command_buffer, const RenderInstancePushConstant& render_instance_push_constant,
                    const std::shared_ptr<GraphicsPipeline>& graphics_pipeline) const override;
  };

  class IRenderInstanceCollection {
   public:
    virtual bool Empty() const = 0;
    virtual void Register(const std::shared_ptr<IRenderInstance>& render_instance) = 0;
    virtual void ForEachRenderInstance(const std::function<void(const std::shared_ptr<IRenderInstance>&)>& action) = 0;
  };

  class ExternalRenderInstanceCollection : public IRenderInstanceCollection {
    std::vector<std::shared_ptr<ExternalRenderInstance>> render_commands;

   public:
    bool operator!=(const ExternalRenderInstanceCollection& other) const;
    bool Empty() const override;
    void Register(const std::shared_ptr<IRenderInstance>& render_instance) override;
    void ForEachRenderInstance(const std::function<void(const std::shared_ptr<IRenderInstance>&)>& action) override;
  };

  class MeshRenderInstanceCollection : public IRenderInstanceCollection {
    std::vector<std::shared_ptr<MeshRenderInstance>> render_commands;

   public:
    bool operator!=(const MeshRenderInstanceCollection& other) const;
    bool Empty() const override;
    void Register(const std::shared_ptr<IRenderInstance>& render_instance) override;
    void ForEachRenderInstance(const std::function<void(const std::shared_ptr<IRenderInstance>&)>& action) override;
  };

  class SkinnedMeshRenderInstanceCollection : public IRenderInstanceCollection {
    std::vector<std::shared_ptr<SkinnedMeshRenderInstance>> render_commands;

   public:
    bool Empty() const override;
    void Register(const std::shared_ptr<IRenderInstance>& render_instance) override;
    bool operator!=(const SkinnedMeshRenderInstanceCollection& other) const;
    void ForEachRenderInstance(const std::function<void(const std::shared_ptr<IRenderInstance>&)>& action) override;
  };

  class StrandsRenderInstanceCollection : public IRenderInstanceCollection {
    std::vector<std::shared_ptr<StrandsRenderInstance>> render_commands;

   public:
    bool Empty() const override;
    void Register(const std::shared_ptr<IRenderInstance>& render_instance) override;
    bool operator!=(const StrandsRenderInstanceCollection& other) const;
    void ForEachRenderInstance(const std::function<void(const std::shared_ptr<IRenderInstance>&)>& action) override;
  };

  class InstancedRenderInstanceCollection : public IRenderInstanceCollection {
    std::vector<std::shared_ptr<InstancedRenderInstance>> render_commands;

   public:
    bool Empty() const override;
    void Register(const std::shared_ptr<IRenderInstance>& render_instance) override;
    bool operator!=(const InstancedRenderInstanceCollection& other) const;
    void ForEachRenderInstance(const std::function<void(const std::shared_ptr<IRenderInstance>&)>& action) override;
  };

  RenderInstanceStorage();
  bool operator!=(const RenderInstanceStorage& other) const;
  bool RegisterMeshDrawCommand(const std::shared_ptr<Mesh>& mesh, const std::shared_ptr<Material>& material,
                               const GlobalTransform& model, bool cast_shadow);
  bool RegisterMeshDrawInstancedCommand(const std::shared_ptr<Mesh>& mesh, const std::shared_ptr<Material>& material,
                                        const GlobalTransform& model,
                                        const std::shared_ptr<ParticleInfoList>& particle_info_list, bool cast_shadow);
  bool RegisterRenderInstance(const std::shared_ptr<Scene>& target_scene, const Entity& entity,
                              const Handle& renderer_handle, const std::shared_ptr<Material>& material);
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
  /**
   * \brief Find material index via material handle.
   * \param material_handle Target material's handle.
   * \return Index of the material.
   */
  [[nodiscard]] int GetMaterialIndex(const Handle& material_handle);
  /**
   * \brief Find renderer via render instance index.
   * \param renderer_handle Target renderer's handle.
   * \return Index of the render instance.
   */
  [[nodiscard]] int GetRenderInstanceIndex(const Handle& renderer_handle);
  /**
   * \brief Find camera via camera index.
   * \param camera_handle Target camera's handle
   * \return Index of the camera.
   */
  [[nodiscard]] int GetCameraIndex(const Handle& camera_handle);
  /**
   * \brief Find entity via render instance index.
   * \param render_instance_index Index of the render instance.
   * \return Handle of the entity.
   */
  [[nodiscard]] Handle GetInstanceEntityHandle(int render_instance_index);
  /**
   * \brief Find renderer via render instance index.
   * \param render_instance_index Index of the render instance.
   * \return Handle of the renderer.
   */
  [[nodiscard]] Handle GetInstanceRendererHandle(int render_instance_index);
  void Upload() const;

  [[nodiscard]] const std::vector<MaterialInfoBlock>& GetMaterialInfoBlocks() const;
  [[nodiscard]] const std::vector<InstanceInfoBlock>& GetInstanceInfoBlocks() const;

 private:
  /**
   * \brief Use this to find instance index via renderer handle.
   */
  std::unordered_map<Handle, int> renderer_indices_;
  /**
   * \brief Use this to find material index via material handle.
   */
  std::unordered_map<Handle, int> material_indices_;
  /**
   * \brief Use this to find entity via render instance index.
   */
  std::unordered_map<int, Handle> instance_entity_handles_;
  /**
   * \brief Use this to find renderer via render instance index.
   */
  std::unordered_map<int, Handle> instance_renderer_handles_;
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
  std::shared_ptr<MeshRenderInstanceCollection> deferred_render_instances;
  std::shared_ptr<SkinnedMeshRenderInstanceCollection> deferred_skinned_render_instances;
  std::shared_ptr<InstancedRenderInstanceCollection> deferred_instanced_render_instances;
  std::shared_ptr<StrandsRenderInstanceCollection> deferred_strands_render_instances;

  std::shared_ptr<MeshRenderInstanceCollection> forward_render_instances;
  std::shared_ptr<SkinnedMeshRenderInstanceCollection> forward_skinned_render_instances;
  std::shared_ptr<InstancedRenderInstanceCollection> forward_instanced_render_instances;
  std::shared_ptr<StrandsRenderInstanceCollection> forward_strands_render_instances;

  std::shared_ptr<MeshRenderInstanceCollection> transparent_render_instances;
  std::shared_ptr<SkinnedMeshRenderInstanceCollection> transparent_skinned_render_instances;
  std::shared_ptr<InstancedRenderInstanceCollection> transparent_instanced_render_instances;
  std::shared_ptr<StrandsRenderInstanceCollection> transparent_strands_render_instances;

  std::shared_ptr<ExternalRenderInstanceCollection> external_render_instances;
  friend class TopLevelAccelerationStructure;
  friend class RenderLayer;
  friend class CpuRayTracer;
  void CollectEntityRenderers(const std::shared_ptr<Scene>& target_scene, Bound& world_bound);
  void BuildRenderInstanceBlocks();
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
  [[nodiscard]] int RegisterCamera(const Handle& handle, const CameraInfoBlock& camera_info_block);
};
}  // namespace evo_engine