
#pragma once
#include "Camera.hpp"
#include "Entity.hpp"
#include "Lights.hpp"
#include "MeshRenderer.hpp"
#include "SkinnedMeshRenderer.hpp"
#include "StrandsRenderer.hpp"

namespace evo_engine {

/**
 * @brief Struct containing various render settings for the engine.
 */
struct RenderSettings {
  bool stable_fit = true;                                       ///< Indicates whether rendering should use stable fit.
  float max_shadow_distance = 100;                              ///< Maximum shadow distance in the scene.
  float shadow_cascade_split[4] = {0.075f, 0.15f, 0.3f, 1.0f};  ///< Splits for shadow cascades.
  bool enable_debug_visualization = false;                      ///< Whether debug visualization is enabled.

  int pcf_sample_amount = 32;   ///< Sample amount for PCF shadows.
  float seam_fix_ratio = 0.1f;  ///< Ratio for fixing seam issues in shadows.

  float strands_subdivision_x_factor = 50.0f;  ///< Subdivision factor for strands (in the X-axis).
  float strands_subdivision_y_factor = 50.0f;  ///< Subdivision factor for strands (in the Y-axis).
  int strands_subdivision_max_x = 15;          ///< Maximum subdivision in X-axis for strands.
  int strands_subdivision_max_y = 8;           ///< Maximum subdivision in Y-axis for strands.
};

/**
 * @brief Struct containing push constants for render instances.
 */
struct RenderInstancePushConstant {
  int instance_index = 0;     ///< Index of the instance to render.
  int camera_index = 0;       ///< Index of the camera.
  int light_split_index = 0;  ///< Index of the light split for rendering.
};

/**
 * @brief Struct containing push constants for ray tracing.
 */
struct RayTracingCameraPushConstant {
  uint32_t camera_index = 0;  ///< Index of the camera for ray tracing.
  uint32_t frame_id = 0;      ///< Frame ID for the current ray tracing operation.
};

/**
 * @brief Struct containing push constants for ray tracing.
 */
struct RayTracingPointCloudPushConstant {
  uint32_t bounce = 0;  ///< Current bounce count for ray tracing.
  uint32_t envIndex;
  uint32_t skybox_tex_index;
  uint32_t use_clear_color;
  glm::vec4 clear_color;
};

/**
 * @brief Enumeration for defining the type of render instance.
 */
enum class RenderInstanceType {
  Unknown,       ///< The render instance type is unknown.
  FromRenderer,  ///< Render instance comes from a renderer.
  FromApi        ///< Render instance comes from the API.
};

/**
 * @brief Class for managing render instance storage.
 */
class RenderInstanceStorage {
 public:
  /**
   * @brief Struct to hold information related to render settings applied.
   */
  struct RenderInfoBlock {
    glm::vec4 split_distances = {};          ///< Distances for shadow cascade splits.
    alignas(4) int pcf_sample_amount = 32;   ///< PCF sampling amount.
    alignas(4) int debug_visualization = 0;  ///< Debug visualization flag.
    alignas(4) float seam_fix_ratio = 0.1f;  ///< Ratio for seam fixes.
    alignas(4) float padding = 1.f;          ///< Padding to align struct properly.

    alignas(4) float strands_subdivision_x_factor = 50.0f;  ///< X factor for strands subdivision.
    alignas(4) float strands_subdivision_y_factor = 50.0f;  ///< Y factor for strands subdivision.
    alignas(4) int strands_subdivision_max_x = 15;          ///< Max subdivisions in X-axis for strands.
    alignas(4) int strands_subdivision_max_y = 8;           ///< Max subdivisions in Y-axis for strands.

    alignas(4) int directional_light_size = 0;  ///< Number of directional lights.
    alignas(4) int point_light_size = 0;        ///< Number of point lights.
    alignas(4) int spot_light_size = 0;         ///< Number of spot lights.
    alignas(4) int brdflut_texture_index = 0;   ///< Texture index for BRDF LUT.

    /**
     * @brief Applies the settings from the target RenderSettings.
     * @param target_render_settings Render settings to be applied.
     */
    void Apply(const RenderSettings& target_render_settings);

    /**
     * @brief Compares two RenderInfoBlock objects for inequality.
     * @param other The other RenderInfoBlock object to compare.
     * @return True if the objects are not equal.
     */
    bool operator!=(const RenderInfoBlock& other) const;
  };

  /**
   * @brief Struct to hold environment-related rendering information.
   */
  struct EnvironmentInfoBlock {
    glm::vec4 background_color = glm::vec4(1.0f, 1.0f, 1.0f, 1.0f);  ///< Background color of the environment.
    alignas(4) float environmental_map_gamma = 2.2f;                 ///< Gamma correction for the environmental map.
    alignas(4) float environmental_lighting_intensity = 0.8f;        ///< Intensity of the environmental lighting.
    alignas(4) float background_intensity = 1.0f;                    ///< Intensity of the background.
    alignas(4) float environmental_padding2 = 0.0f;                  ///< Padding for alignment.

    /**
     * @brief Compares two EnvironmentInfoBlock objects for inequality.
     * @param other The other EnvironmentInfoBlock object to compare.
     * @return True if the objects are not equal.
     */
    bool operator!=(const EnvironmentInfoBlock& other) const;
  };

  /**
   * @brief Struct to hold instance-related rendering information.
   */
  struct InstanceInfoBlock {
    GlobalTransform model = {};        ///< Global transform of the instance.
    int32_t material_index = 0;        ///< Material index used by the instance.
    int32_t triangle_offset = 0;       ///< Offset for triangles in the mesh.
    int32_t meshlet_index_offset = 0;  ///< Offset for meshlet indices.
    int32_t meshlet_size = 0;          ///< Size of the meshlet.

    int32_t info_index = 0;      ///< Index for additional instance information.
    uint32_t entity_index = 0;   ///< Entity index associated with the instance.
    Handle renderer_handle = 0;  ///< Handle for the renderer.

    /**
     * @brief Compares two InstanceInfoBlock objects for inequality.
     * @param other The other InstanceInfoBlock object to compare.
     * @return True if the objects are not equal.
     */
    bool operator!=(const InstanceInfoBlock& other) const;
  };

  /**
   * @brief Struct to hold material-related rendering information.
   */
  struct MaterialInfoBlock {
    alignas(4) int albedo_texture_index = -1;     ///< Albedo texture index.
    alignas(4) int normal_texture_index = -1;     ///< Normal texture index.
    alignas(4) int metallic_texture_index = -1;   ///< Metallic texture index.
    alignas(4) int roughness_texture_index = -1;  ///< Roughness texture index.

    alignas(4) int ao_texture_index = -1;  ///< Ambient occlusion texture index.
    alignas(4) int cast_shadow = true;     ///< Indicates if the material casts shadows.
    alignas(4) int receive_shadow = true;  ///< Indicates if the material receives shadows.
    alignas(4) int enable_shadow = true;   ///< Indicates if shadows are enabled for the material.

    glm::vec4 albedo_color_val = glm::vec4(1.0f);                     ///< Albedo color value.
    glm::vec4 subsurface_color = glm::vec4(1.0f, 1.0f, 1.0f, 0.0f);   ///< Subsurface color.
    glm::vec4 subsurface_radius = glm::vec4(1.0f, 1.0f, 1.0f, 0.0f);  ///< Subsurface radius.

    alignas(4) float metallic_val = 0.5f;   ///< Metallic value.
    alignas(4) float roughness_val = 0.5f;  ///< Roughness value.
    alignas(4) float ao_val = 1.0f;         ///< Ambient occlusion value.
    alignas(4) float emission_val = 0.0f;   ///< Emission value.

    /**
     * @brief Applies the material settings to the target material.
     * @param target_material Shared pointer to the material where settings will be applied.
     */
    void Apply(const std::shared_ptr<Material>& target_material);

    /**
     * @brief Compares two MaterialInfoBlock objects for inequality.
     * @param other The other MaterialInfoBlock object to compare.
     * @return True if the objects are not equal.
     */
    bool operator!=(const MaterialInfoBlock& other) const;
  };

  /**
   * @brief Abstract struct defining an interface for render instances.
   */
  struct IRenderInstance {
    int32_t instance_index = 0;                                     ///< Index of the render instance.
    int32_t material_index = 0;                                     ///< Material index used by the render instance.
    RenderInstanceType command_type = RenderInstanceType::Unknown;  ///< Type of the render instance.
    Entity owner = Entity();                                        ///< Entity owning the render instance.
    Handle entity_handle;                                           ///< Handle for the entity.
    Handle renderer_handle = 0;                                     ///< Handle for the renderer.
    bool entity_selected = false;                                   ///< Indicates if the entity is selected.
    GlobalTransform model = {};                                     ///< Global transform for the render instance.
    float line_width = 1.0f;                                        ///< Line width for rendering.
    VkCullModeFlags cull_mode = VK_CULL_MODE_BACK_BIT;              ///< Culling mode for rendering.
    VkPolygonMode polygon_mode = VK_POLYGON_MODE_FILL;              ///< Polygon rendering mode.
    bool cast_shadow = true;                                        ///< Indicates if the render instance casts shadows.
    uint32_t material_version;                                      ///< Material version used by the render instance.
    uint32_t geometry_version;                                      ///< Geometry version used by the render instance.
    std::shared_ptr<Material> material;                             ///< Material used by the render instance.

    /**
     * @brief Applies the rendering information to the given InstanceInfoBlock.
     * @param instance_info_block The instance information block to update.
     */
    virtual void Apply(InstanceInfoBlock& instance_info_block) const = 0;

    /**
     * @brief Renders the instance using the provided Vulkan command buffer and pipeline.
     * @param vk_command_buffer Vulkan command buffer for rendering.
     * @param render_instance_push_constant Push constants for the render instance.
     * @param graphics_pipeline Graphics pipeline for rendering.
     * @return A 32-bit unsigned integer indicating rendering success.
     */
    virtual uint32_t Render(VkCommandBuffer vk_command_buffer,
                            const RenderInstancePushConstant& render_instance_push_constant,
                            const std::shared_ptr<GraphicsPipeline>& graphics_pipeline) const = 0;
  };

  /**
   * @brief Struct for external render instance functionality.
   */
  struct ExternalRenderInstance : IRenderInstance {
    /**
     * @brief Compares two ExternalRenderInstance objects for inequality.
     * @param other The other ExternalRenderInstance object to compare.
     * @return True if the objects are not equal.
     */
    bool operator!=(const ExternalRenderInstance& other) const;

    /**
     * @brief Apply instance information to the given InstanceInfoBlock.
     * @param instance_info_block The instance information block to update.
     */
    void Apply(InstanceInfoBlock& instance_info_block) const override;

    /**
     * @brief Renders the external instance using the given Vulkan resources.
     * @param vk_command_buffer Vulkan command buffer to use.
     * @param render_instance_push_constant Push constants for the render instance.
     * @param graphics_pipeline Graphics pipeline for rendering.
     * @return Rendering result as a 32-bit unsigned integer.
     */
    uint32_t Render(VkCommandBuffer vk_command_buffer, const RenderInstancePushConstant& render_instance_push_constant,
                    const std::shared_ptr<GraphicsPipeline>& graphics_pipeline) const override;
  };

  /**
   * @brief Struct for mesh render instance functionality.
   */
  struct MeshRenderInstance : IRenderInstance {
    std::shared_ptr<Mesh> mesh;  ///< Shared pointer to the mesh rendered.

    /**
     * @brief Compares two MeshRenderInstance objects for inequality.
     * @param other The other MeshRenderInstance object to compare.
     * @return True if the objects are not equal.
     */
    bool operator!=(const MeshRenderInstance& other) const;

    /**
     * @brief Apply instance information to the given InstanceInfoBlock.
     * @param instance_info_block The instance information block to update.
     */
    void Apply(InstanceInfoBlock& instance_info_block) const override;

    /**
     * @brief Renders the mesh instance using the given Vulkan resources.
     * @param vk_command_buffer Vulkan command buffer to use.
     * @param render_instance_push_constant Push constants for the render instance.
     * @param graphics_pipeline Graphics pipeline for rendering.
     * @return Rendering result as a 32-bit unsigned integer.
     */
    uint32_t Render(VkCommandBuffer vk_command_buffer, const RenderInstancePushConstant& render_instance_push_constant,
                    const std::shared_ptr<GraphicsPipeline>& graphics_pipeline) const override;
  };

  /**
   * @brief Struct for skinned mesh render instance functionality.
   */
  struct SkinnedMeshRenderInstance : IRenderInstance {
    uint32_t bone_matrices_version;               ///< Version of the bone matrices for the skinned mesh.
    std::shared_ptr<SkinnedMesh> skinned_mesh;    ///< Shared pointer to the skinned mesh.
    std::shared_ptr<BoneMatrices> bone_matrices;  ///< Shared pointer to bone matrices needed for animation.

    /**
     * @brief Compares two SkinnedMeshRenderInstance objects for inequality.
     * @param other The other SkinnedMeshRenderInstance object to compare.
     * @return True if the objects are not equal.
     */
    bool operator!=(const SkinnedMeshRenderInstance& other) const;

    /**
     * @brief Apply instance information to the given InstanceInfoBlock.
     * @param instance_info_block The instance information block to update.
     */
    void Apply(InstanceInfoBlock& instance_info_block) const override;

    /**
     * @brief Renders the skinned mesh instance using the given Vulkan resources.
     * @param vk_command_buffer Vulkan command buffer to use.
     * @param render_instance_push_constant Push constants for the render instance.
     * @param graphics_pipeline Graphics pipeline for rendering.
     * @return Rendering result as a 32-bit unsigned integer.
     */
    uint32_t Render(VkCommandBuffer vk_command_buffer, const RenderInstancePushConstant& render_instance_push_constant,
                    const std::shared_ptr<GraphicsPipeline>& graphics_pipeline) const override;
  };

  /**
   * @brief Struct for instanced render instance functionality.
   */
  struct InstancedRenderInstance : IRenderInstance {
    uint32_t particle_info_list_version;               ///< Version of the particle information list.
    std::shared_ptr<Mesh> mesh;                        ///< Shared pointer to the mesh.
    std::shared_ptr<ParticleInfoList> particle_infos;  ///< Shared pointer to the particle information list.

    /**
     * @brief Compares two InstancedRenderInstance objects for inequality.
     * @param other The other InstancedRenderInstance object to compare.
     * @return True if the objects are not equal.
     */
    bool operator!=(const InstancedRenderInstance& other) const;

    /**
     * @brief Apply instance information to the given InstanceInfoBlock.
     * @param instance_info_block The instance information block to update.
     */
    void Apply(InstanceInfoBlock& instance_info_block) const override;

    /**
     * @brief Renders the instanced object using the provided Vulkan resources.
     * @param vk_command_buffer Vulkan command buffer to use.
     * @param render_instance_push_constant Push constants for the render instance.
     * @param graphics_pipeline Graphics pipeline for rendering.
     * @return Rendering result as a 32-bit unsigned integer.
     */
    uint32_t Render(VkCommandBuffer vk_command_buffer, const RenderInstancePushConstant& render_instance_push_constant,
                    const std::shared_ptr<GraphicsPipeline>& graphics_pipeline) const override;
  };

  /**
   * @brief Struct for strands render instance functionality.
   */
  struct StrandsRenderInstance : IRenderInstance {
    std::shared_ptr<Strands> strands;  ///< Shared pointer to the strands to be rendered.

    /**
     * @brief Compares two StrandsRenderInstance objects for inequality.
     * @param other The other StrandsRenderInstance object to compare.
     * @return True if the objects are not equal.
     */
    bool operator!=(const StrandsRenderInstance& other) const;

    /**
     * @brief Apply instance information to the given InstanceInfoBlock.
     * @param instance_info_block The instance information block to update.
     */
    void Apply(InstanceInfoBlock& instance_info_block) const override;

    /**
     * @brief Renders the strands instance using the given Vulkan resources.
     * @param vk_command_buffer Vulkan command buffer to use.
     * @param render_instance_push_constant Push constants for the render instance.
     * @param graphics_pipeline Graphics pipeline for rendering.
     * @return Rendering result as a 32-bit unsigned integer.
     */
    uint32_t Render(VkCommandBuffer vk_command_buffer, const RenderInstancePushConstant& render_instance_push_constant,
                    const std::shared_ptr<GraphicsPipeline>& graphics_pipeline) const override;
  };

  /**
   * @brief Interface for a collection of render instances.
   */
  class IRenderInstanceCollection {
   public:
    /**
     * @brief Checks whether the collection is empty.
     * @return True if the collection is empty, false otherwise.
     */
    virtual bool Empty() const = 0;

    /**
     * @brief Registers a render instance in the collection.
     * @param render_instance The render instance to register.
     */
    virtual void Register(const std::shared_ptr<IRenderInstance>& render_instance) = 0;

    /**
     * @brief Applies an action to each render instance in the collection.
     * @param action The action to apply.
     */
    virtual void ForEachRenderInstance(const std::function<void(const std::shared_ptr<IRenderInstance>&)>& action) = 0;
  };

  /**
   * @brief Collection of external render instances.
   */
  class ExternalRenderInstanceCollection : public IRenderInstanceCollection {
    std::vector<std::shared_ptr<ExternalRenderInstance>> render_commands;  ///< Commands for external rendering.

   public:
    /**
     * @brief Compares two ExternalRenderInstanceCollection objects for inequality.
     * @param other The other ExternalRenderInstanceCollection object to compare.
     * @return True if the collections are not equal.
     */
    bool operator!=(const ExternalRenderInstanceCollection& other) const;

    /**
     * @brief Checks whether the collection is empty.
     * @return True if the collection is empty, false otherwise.
     */
    bool Empty() const override;

    /**
     * @brief Registers a render instance in the collection.
     * @param render_instance The render instance to register.
     */
    void Register(const std::shared_ptr<IRenderInstance>& render_instance) override;

    /**
     * @brief Applies an action to each render instance in the collection.
     * @param action The action to apply.
     */
    void ForEachRenderInstance(const std::function<void(const std::shared_ptr<IRenderInstance>&)>& action) override;
  };

  /**
   * @brief Collection of mesh render instances.
   */
  class MeshRenderInstanceCollection : public IRenderInstanceCollection {
    std::vector<std::shared_ptr<MeshRenderInstance>> render_commands;  ///< Commands for mesh rendering.

   public:
    /**
     * @brief Compares two MeshRenderInstanceCollection objects for inequality.
     * @param other The other MeshRenderInstanceCollection object to compare.
     * @return True if the collections are not equal.
     */
    bool operator!=(const MeshRenderInstanceCollection& other) const;

    /**
     * @brief Checks whether the collection is empty.
     * @return True if the collection is empty, false otherwise.
     */
    bool Empty() const override;

    /**
     * @brief Registers a render instance in the collection.
     * @param render_instance The render instance to register.
     */
    void Register(const std::shared_ptr<IRenderInstance>& render_instance) override;

    /**
     * @brief Applies an action to each render instance in the collection.
     * @param action The action to apply.
     */
    void ForEachRenderInstance(const std::function<void(const std::shared_ptr<IRenderInstance>&)>& action) override;
  };

  /**
   * @brief Collection of skinned mesh render instances.
   */
  class SkinnedMeshRenderInstanceCollection : public IRenderInstanceCollection {
    std::vector<std::shared_ptr<SkinnedMeshRenderInstance>> render_commands;  ///< Commands for skinned mesh rendering.

   public:
    /**
     * @brief Checks whether the collection is empty.
     * @return True if the collection is empty, false otherwise.
     */
    bool Empty() const override;

    /**
     * @brief Registers a render instance in the collection.
     * @param render_instance The render instance to register.
     */
    void Register(const std::shared_ptr<IRenderInstance>& render_instance) override;

    /**
     * @brief Compares two SkinnedMeshRenderInstanceCollection objects for inequality.
     * @param other The other SkinnedMeshRenderInstanceCollection object to compare.
     * @return True if the collections are not equal.
     */
    bool operator!=(const SkinnedMeshRenderInstanceCollection& other) const;

    /**
     * @brief Applies an action to each render instance in the collection.
     * @param action The action to apply.
     */
    void ForEachRenderInstance(const std::function<void(const std::shared_ptr<IRenderInstance>&)>& action) override;
  };

  /**
   * @brief Collection of strands render instances.
   */
  class StrandsRenderInstanceCollection : public IRenderInstanceCollection {
    std::vector<std::shared_ptr<StrandsRenderInstance>> render_commands;  ///< Commands for strands rendering.

   public:
    /**
     * @brief Checks whether the collection is empty.
     * @return True if the collection is empty, false otherwise.
     */
    bool Empty() const override;

    /**
     * @brief Registers a render instance in the collection.
     * @param render_instance The render instance to register.
     */
    void Register(const std::shared_ptr<IRenderInstance>& render_instance) override;

    /**
     * @brief Compares two StrandsRenderInstanceCollection objects for inequality.
     * @param other The other StrandsRenderInstanceCollection object to compare.
     * @return True if the collections are not equal.
     */
    bool operator!=(const StrandsRenderInstanceCollection& other) const;

    /**
     * @brief Applies an action to each render instance in the collection.
     * @param action The action to apply.
     */
    void ForEachRenderInstance(const std::function<void(const std::shared_ptr<IRenderInstance>&)>& action) override;
  };

  /**
   * @brief Collection of instanced render instances.
   */
  class InstancedRenderInstanceCollection : public IRenderInstanceCollection {
    std::vector<std::shared_ptr<InstancedRenderInstance>> render_commands;  ///< Commands for instanced rendering.

   public:
    /**
     * @brief Checks whether the collection is empty.
     * @return True if the collection is empty, false otherwise.
     */
    bool Empty() const override;

    /**
     * @brief Registers a render instance in the collection.
     * @param render_instance The render instance to register.
     */
    void Register(const std::shared_ptr<IRenderInstance>& render_instance) override;

    /**
     * @brief Compares two InstancedRenderInstanceCollection objects for inequality.
     * @param other The other InstancedRenderInstanceCollection object to compare.
     * @return True if the collections are not equal.
     */
    bool operator!=(const InstancedRenderInstanceCollection& other) const;

    /**
     * @brief Applies an action to each render instance in the collection.
     * @param action The action to apply.
     */
    void ForEachRenderInstance(const std::function<void(const std::shared_ptr<IRenderInstance>&)>& action) override;
  };

  /**
   * @brief Default constructor for RenderInstanceStorage.
   */
  RenderInstanceStorage();

  /**
   * @brief Compares two RenderInstanceStorage objects for inequality.
   * @param other The other RenderInstanceStorage object to compare.
   * @return True if the objects are not equal.
   */
  bool operator!=(const RenderInstanceStorage& other) const;

  /**
   * @brief Registers a mesh draw command.
   * @param mesh The mesh to draw.
   * @param material The material used for rendering the mesh.
   * @param model The global transform of the mesh.
   * @param cast_shadow Indicates whether the mesh casts shadows.
   * @return True if the registration was successful.
   */
  bool RegisterMeshDrawCommand(const std::shared_ptr<Mesh>& mesh, const std::shared_ptr<Material>& material,
                               const GlobalTransform& model, bool cast_shadow);

  /**
   * @brief Registers a mesh draw instanced command.
   * @param mesh The mesh to draw.
   * @param material The material used for rendering the mesh.
   * @param model The global transform of the mesh.
   * @param particle_info_list The particle information list for the mesh instances.
   * @param cast_shadow Indicates whether the mesh casts shadows.
   * @return True if the registration was successful.
   */
  bool RegisterMeshDrawInstancedCommand(const std::shared_ptr<Mesh>& mesh, const std::shared_ptr<Material>& material,
                                        const GlobalTransform& model,
                                        const std::shared_ptr<ParticleInfoList>& particle_info_list, bool cast_shadow);

  /**
   * @brief Registers a render instance for the specified entity in the target scene.
   * @param target_scene The scene containing the render instance.
   * @param entity The entity owning the render instance.
   * @param renderer_handle The handle of the renderer.
   * @param material The material associated with the render instance.
   * @return True if the registration was successful.
   */
  bool RegisterRenderInstance(const std::shared_ptr<Scene>& target_scene, const Entity& entity,
                              const Handle& renderer_handle, const std::shared_ptr<Material>& material,
                              int* out_material_index = nullptr);

  /**
   * @brief Registers a material and returns its index.
   * @param material Shared pointer to the material being registered.
   * @return Index of the registered material.
   */
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
  /**
   * @brief Clears all the render instance data and collections.
   */
  void Clear();

  /**
   * @brief Collects editor cameras from the specified scene.
   * @param target_scene The scene from which to collect cameras.
   * @param cameras Vector to store collected cameras and their transforms.
   */
  static void CollectEditorCameras(const std::shared_ptr<Scene>& target_scene,
                                   std::vector<std::pair<GlobalTransform, std::shared_ptr<Camera>>>& cameras);

  /**
   * @brief Collects all cameras from the specified scene.
   * @param target_scene The scene from which to collect cameras.
   * @param cameras Vector to store collected cameras and their transforms.
   */
  static void CollectCameras(const std::shared_ptr<Scene>& target_scene,
                             std::vector<std::pair<GlobalTransform, std::shared_ptr<Camera>>>& cameras);

  /**
   * @brief Calculates the Level of Detail (LOD) factor for objects in the scene based on distance.
   * @param scene The scene containing objects.
   * @param view_position The position of the viewer.
   * @param max_distance The maximum distance for LOD calculations.
   */
  static void CalculateLodFactor(const std::shared_ptr<Scene>& scene, const glm::vec3& view_position,
                                 float max_distance);

  /**
   * @brief Builds render instances from the specified scene, applying render settings.
   * @param render_settings The render settings to apply.
   * @param scene The scene from which to build render instances.
   * @param world_bound The world bounds for the scene.
   */
  void BuildFromScene(const RenderSettings& render_settings, const std::shared_ptr<Scene>& scene, Bound& world_bound,
                      bool include_editor_cameras = true);

  /**
   * @brief Updates the top-level acceleration structure for ray tracing.
   * @param scene The scene for which to update the acceleration structure.
   */
  void UpdateTopLevelAccelerationStructure(const std::shared_ptr<Scene>& scene);

  /**
   * @brief Finds the material index via a material handle.
   * @param material_handle The handle of the target material.
   * @return Index of the material.
   */
  [[nodiscard]] int GetMaterialIndex(const Handle& material_handle);

  /**
   * @brief Finds the renderer index via a render instance index.
   * @param renderer_handle The handle of the target renderer.
   * @return Index of the render instance.
   */
  [[nodiscard]] int GetRenderInstanceIndex(const Handle& renderer_handle);

  /**
   * @brief Finds the camera index via a camera handle.
   * @param camera_handle The handle of the target camera.
   * @return Index of the camera.
   */
  [[nodiscard]] int GetCameraIndex(const Handle& camera_handle);

  /**
   * @brief Finds the entity handle via a render instance index.
   * @param render_instance_index The index of the render instance.
   * @return Handle of the entity associated with the instance.
   */
  [[nodiscard]] Handle GetInstanceEntityHandle(int render_instance_index);

  /**
   * @brief Finds the renderer handle via a render instance index.
   * @param render_instance_index The index of the render instance.
   * @return Handle of the renderer associated with the instance.
   */
  [[nodiscard]] Handle GetInstanceRendererHandle(int render_instance_index);

  /**
   * @brief Uploads all data and render instance information to the GPU.
   */
  void Upload() const;

  /**
   * @brief Retrieves the list of material information blocks.
   * @return Reference to the vector of MaterialInfoBlock objects.
   */
  [[nodiscard]] const std::vector<MaterialInfoBlock>& GetMaterialInfoBlocks() const;

  /**
   * @brief Retrieves the list of instance information blocks.
   * @return Reference to the vector of InstanceInfoBlock objects.
   */
  [[nodiscard]] const std::vector<InstanceInfoBlock>& GetInstanceInfoBlocks() const;

 private:
  /**
   * @brief Finds the instance index via a renderer handle.
   */
  std::unordered_map<Handle, int> renderer_indices_;

  /**
   * @brief Finds the material index via a material handle.
   */
  std::unordered_map<Handle, int> material_indices_;

  /**
   * @brief Finds the entity handle via a render instance index.
   */
  std::unordered_map<int, Handle> instance_entity_handles_;

  /**
   * @brief Finds the renderer handle via a render instance index.
   */
  std::unordered_map<int, Handle> instance_renderer_handles_;

  /**
   * @brief Finds the camera index via a camera handle.
   */
  std::unordered_map<Handle, int> camera_indices_;

  /**
   * @brief Holds the material information blocks.
   */
  std::vector<MaterialInfoBlock> material_info_blocks_{};

  /**
   * @brief Holds the instance information blocks.
   */
  std::vector<InstanceInfoBlock> instance_info_blocks_{};

  /**
   * @brief Stores rendering-related information like shadow splits and lighting.
   */
  RenderInfoBlock render_info_block = {};

  /**
   * @brief Stores environment-related rendering information such as background color.
   */
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
  uint32_t geometry_storage_version = 0;
  uint32_t texture_storage_version = 0;

  friend class TopLevelAccelerationStructure;
  friend class RenderLayer;
  friend class CpuRayTracer;
  /**
   * @brief Collects entity renderers and calculates the world bounding box.
   * @param target_scene The scene containing entities.
   * @param world_bound Output bounding box for the world.
   */
  void CollectEntityRenderers(const std::shared_ptr<Scene>& target_scene, Bound& world_bound);

  /**
   * @brief Builds render instance blocks for rendering.
   */
  void BuildRenderInstanceBlocks();

  /**
   * @brief Collects lighting information from the scene.
   * @param target_scene The scene from which to collect lights.
   * @param world_bound The world bounds of the scene.
   */
  void CollectLights(const std::shared_ptr<Scene>& target_scene, const Bound& world_bound);

  /**
   * @brief Collects environment-related settings and updates their information.
   * @param target_scene The scene containing the environment settings.
   */
  void CollectEnvironment(const std::shared_ptr<Scene>& target_scene);

  /**
   * @brief Registers an entity with a mesh renderer.
   * @param target_scene The scene containing the entity.
   * @param owner The entity to register.
   * @param mesh_renderer The mesh renderer associated with the entity.
   * @param min_bound Minimum bounds of the entity.
   * @param max_bound Maximum bounds of the entity.
   * @return True if the registration was successful.
   */
  bool RegisterEntity(const std::shared_ptr<Scene>& target_scene, const Entity& owner,
                      const std::shared_ptr<MeshRenderer>& mesh_renderer, glm::vec3& min_bound, glm::vec3& max_bound);

  /**
   * @brief Registers an entity with a skinned mesh renderer.
   * @param target_scene The scene containing the entity.
   * @param owner The entity to register.
   * @param skinned_mesh_renderer The skinned mesh renderer associated with the entity.
   * @param min_bound Minimum bounds of the entity.
   * @param max_bound Maximum bounds of the entity.
   * @return True if the registration was successful.
   */
  bool RegisterEntity(const std::shared_ptr<Scene>& target_scene, const Entity& owner,
                      const std::shared_ptr<SkinnedMeshRenderer>& skinned_mesh_renderer, glm::vec3& min_bound,
                      glm::vec3& max_bound);

  /**
   * @brief Registers an entity with particle-based rendering.
   * @param target_scene The scene containing the entity.
   * @param owner The entity to register.
   * @param particles The particle system associated with the entity.
   * @param min_bound Minimum bounds of the entity.
   * @param max_bound Maximum bounds of the entity.
   * @return True if the registration was successful.
   */
  bool RegisterEntity(const std::shared_ptr<Scene>& target_scene, const Entity& owner,
                      const std::shared_ptr<Particles>& particles, glm::vec3& min_bound, glm::vec3& max_bound);

  /**
   * @brief Registers an entity with a strands renderer.
   * @param target_scene The scene containing the entity.
   * @param owner The entity to register.
   * @param strands_renderer The strands renderer associated with the entity.
   * @param min_bound Minimum bounds of the entity.
   * @param max_bound Maximum bounds of the entity.
   * @return True if the registration was successful.
   */
  bool RegisterEntity(const std::shared_ptr<Scene>& target_scene, const Entity& owner,
                      const std::shared_ptr<StrandsRenderer>& strands_renderer, glm::vec3& min_bound,
                      glm::vec3& max_bound);

  /**
   * @brief Registers material information and returns its index.
   * @param handle The handle associated with the material.
   * @param material_info_block The material information block to register.
   * @return Index of the registered material.
   */
  [[nodiscard]] int RegisterMaterial(const Handle& handle, const MaterialInfoBlock& material_info_block);

  /**
   * @brief Registers camera information and returns its index.
   * @param handle The handle associated with the camera.
   * @param camera_info_block The camera information block to register.
   * @return Index of the registered camera.
   */
  [[nodiscard]] int RegisterCamera(const Handle& handle, const CameraInfoBlock& camera_info_block);
};

};  // namespace evo_engine
