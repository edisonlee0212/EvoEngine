#pragma once
#include "Camera.hpp"
#include "IGeometry.hpp"
#include "ILayer.hpp"
#include "Lights.hpp"
#include "Material.hpp"
#include "Mesh.hpp"
#include "MeshRenderer.hpp"
#include "SkinnedMeshRenderer.hpp"
#include "Strands.hpp"

namespace evo_engine {
#pragma region Enums Structs
enum class RenderCommandType {
  Unknown,
  FromRenderer,
  FromApi,
};

struct RenderInstancePushConstant {
  int instance_index = 0;
  int camera_index = 0;
  int light_split_index = 0;
};

struct RenderInstance {
  uint32_t instance_index = 0;
  RenderCommandType command_type = RenderCommandType::Unknown;
  Entity m_owner = Entity();
  std::shared_ptr<Mesh> m_mesh;
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
};

struct EnvironmentInfoBlock {
  glm::vec4 background_color = glm::vec4(1.0f, 1.0f, 1.0f, 1.0f);
  alignas(4) float environmental_map_gamma = 2.2f;
  alignas(4) float environmental_lighting_intensity = 0.8f;
  alignas(4) float background_intensity = 1.0f;
  alignas(4) float environmental_padding2 = 0.0f;
};

struct InstanceInfoBlock {
  GlobalTransform model = {};
  uint32_t material_index = 0;
  uint32_t entity_selected = 0;
  uint32_t meshlet_index_offset = 0;
  uint32_t meshlet_size = 0;
};

#pragma endregion

class RenderLayer final : public ILayer {
  friend class Resources;
  friend class Camera;
  friend class GraphicsPipeline;
  friend class EditorLayer;
  friend class Material;
  friend class Lighting;
  friend class PostProcessingStack;
  friend class Application;
  void OnCreate() override;
  void OnDestroy() override;
  void OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) override;

  void PreparePointAndSpotLightShadowMap() const;

  void PrepareEnvironmentalBrdfLut();
  void RenderToCamera(const GlobalTransform& camera_global_transform, const std::shared_ptr<Camera>& camera);

  bool TryRegisterRenderer(const std::shared_ptr<Scene>& scene, const Entity& owner,
                           const std::shared_ptr<MeshRenderer>& mesh_renderer, glm::vec3& min_bound,
                           glm::vec3& max_bound, bool enable_selection_highlight);
  bool TryRegisterRenderer(const std::shared_ptr<Scene>& scene, const Entity& owner,
                           const std::shared_ptr<SkinnedMeshRenderer>& skinned_mesh_renderer, glm::vec3& min_bound,
                           glm::vec3& max_bound, bool enable_selection_highlight);
  bool TryRegisterRenderer(const std::shared_ptr<Scene>& scene, const Entity& owner,
                           const std::shared_ptr<Particles>& particles, glm::vec3& min_bound, glm::vec3& max_bound,
                           bool enable_selection_high_light);
  bool TryRegisterRenderer(const std::shared_ptr<Scene>& scene, const Entity& owner,
                           const std::shared_ptr<StrandsRenderer>& strands_renderer, glm::vec3& min_bound,
                           glm::vec3& max_bound, bool enable_selection_high_light);

  void ClearAllCameras();
  void RenderAllCameras();

 public:
  bool wire_frame = false;

  bool count_shadow_rendering_draw_calls = false;
  bool enable_indirect_rendering = false;
  bool enable_debug_visualization = false;
  bool enable_render_menu = false;
  bool stable_fit = true;
  float max_shadow_distance = 100;
  float shadow_cascade_split[4] = {0.075f, 0.15f, 0.3f, 1.0f};

  bool enable_strands_renderer = true;
  bool enable_mesh_renderer = true;
  bool enable_particles = true;
  bool enable_skinned_mesh_renderer = true;

  [[nodiscard]] uint32_t GetCameraIndex(const Handle& handle);
  [[nodiscard]] uint32_t GetMaterialIndex(const Handle& handle);
  [[nodiscard]] uint32_t GetInstanceIndex(const Handle& handle);
  [[nodiscard]] Handle GetInstanceHandle(uint32_t index);

  [[nodiscard]] uint32_t RegisterCameraIndex(const Handle& handle, const CameraInfoBlock& camera_info_block);
  [[nodiscard]] uint32_t RegisterMaterialIndex(const Handle& handle, const MaterialInfoBlock& material_info_block);
  [[nodiscard]] uint32_t RegisterInstanceIndex(const Handle& handle, const InstanceInfoBlock& instance_info_block);

  RenderInfoBlock render_info_block = {};
  EnvironmentInfoBlock environment_info_block = {};
  void DrawMesh(const std::shared_ptr<Mesh>& mesh, const std::shared_ptr<Material>& material, glm::mat4 model,
                bool cast_shadow);

  [[nodiscard]] const std::shared_ptr<DescriptorSet>& GetPerFrameDescriptorSet() const;

 private:
  bool need_fade_ = false;
#pragma region Render procedure
  RenderInstanceCollection deferred_render_instances_;
  SkinnedRenderInstanceCollection deferred_skinned_render_instances_;
  InstancedRenderInstanceCollection deferred_instanced_render_instances_;
  StrandsRenderInstanceCollection deferred_strands_render_instances_;

  RenderInstanceCollection transparent_render_instances_;
  SkinnedRenderInstanceCollection transparent_skinned_render_instances_;
  InstancedRenderInstanceCollection transparent_instanced_render_instances_;
  StrandsRenderInstanceCollection transparent_strands_render_instances_;

  void CollectCameras(std::vector<std::pair<GlobalTransform, std::shared_ptr<Camera>>>& cameras);

  void CalculateLodFactor(const glm::vec3& center, float max_distance) const;

  [[nodiscard]] bool CollectRenderInstances(Bound& world_bound);
  void CollectDirectionalLights(const std::vector<std::pair<GlobalTransform, std::shared_ptr<Camera>>>& cameras);
  void CollectPointLights();
  void CollectSpotLights();

  std::unique_ptr<Lighting> lighting_;
  std::shared_ptr<Texture2D> environmental_brdf_lut_ = {};

  void ApplyAnimator() const;

#pragma endregion
#pragma region Per Frame Descriptor Sets
  friend class TextureStorage;
  std::vector<std::shared_ptr<DescriptorSet>> per_frame_descriptor_sets_ = {};

  std::vector<std::shared_ptr<Buffer>> render_info_descriptor_buffers_ = {};
  std::vector<std::shared_ptr<Buffer>> environment_info_descriptor_buffers_ = {};
  std::vector<std::shared_ptr<Buffer>> camera_info_descriptor_buffers_ = {};
  std::vector<std::shared_ptr<Buffer>> material_info_descriptor_buffers_ = {};
  std::vector<std::shared_ptr<Buffer>> instance_info_descriptor_buffers_ = {};

  std::vector<std::shared_ptr<Buffer>> kernel_descriptor_buffers_ = {};
  std::vector<std::shared_ptr<Buffer>> directional_light_info_descriptor_buffers_ = {};
  std::vector<std::shared_ptr<Buffer>> point_light_info_descriptor_buffers_ = {};
  std::vector<std::shared_ptr<Buffer>> spot_light_info_descriptor_buffers_ = {};

  void CreateStandardDescriptorBuffers();
  void CreatePerFrameDescriptorSets();

  std::unordered_map<Handle, uint32_t> camera_indices_;
  std::unordered_map<Handle, uint32_t> material_indices_;
  std::unordered_map<Handle, uint32_t> instance_indices_;
  std::unordered_map<uint32_t, Handle> instance_handles_;

  std::vector<CameraInfoBlock> camera_info_blocks_{};
  std::vector<MaterialInfoBlock> material_info_blocks_{};
  std::vector<InstanceInfoBlock> instance_info_blocks_{};

  std::vector<std::shared_ptr<Buffer>> mesh_draw_indexed_indirect_commands_buffers_ = {};
  std::vector<VkDrawIndexedIndirectCommand> mesh_draw_indexed_indirect_commands_{};
  uint32_t total_mesh_triangles_ = 0;

  std::vector<std::shared_ptr<Buffer>> mesh_draw_mesh_tasks_indirect_commands_buffers_ = {};
  std::vector<VkDrawMeshTasksIndirectCommandEXT> mesh_draw_mesh_tasks_indirect_commands_{};

  std::vector<DirectionalLightInfo> directional_light_info_blocks_;
  std::vector<PointLightInfo> point_light_info_blocks_;
  std::vector<SpotLightInfo> spot_light_info_blocks_;

#pragma endregion
};
}  // namespace evo_engine
