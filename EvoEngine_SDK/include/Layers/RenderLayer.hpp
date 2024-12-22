#pragma once
#include "Camera.hpp"
#include "IGeometry.hpp"
#include "ILayer.hpp"

#include "Material.hpp"
#include "Mesh.hpp"
#include "RenderInstanceStorage.hpp"
namespace evo_engine {
class RenderLayer final : public ILayer {
 public:
  void ForEachCollectedCamera(const std::function<void(const std::shared_ptr<Camera>& camera)>& action) const;
  [[nodiscard]] std::shared_ptr<RenderInstanceStorage> GetCurrentRenderInstanceStorage() const;
  bool wire_frame = false;
  bool count_shadow_rendering_draw_calls = true;
  bool enable_indirect_rendering = true;
  bool enable_render_menu = false;
  RenderSettings render_settings{};
  [[nodiscard]] uint32_t DrawMesh(const std::shared_ptr<Mesh>& mesh, const std::shared_ptr<Material>& material,
                                  const GlobalTransform& global_transform, bool cast_shadow) const;
  [[nodiscard]] static const std::shared_ptr<DescriptorSet>& GetPerFrameDescriptorSet();
  [[nodiscard]] static const std::shared_ptr<DescriptorSet>& GetLightingDescriptorSet();

  struct PointLightShadowMapView {
    int light_index;
    int face_index;
    glm::ivec4 viewport;
  };
  struct SpotLightShadowMapView {
    int light_index;
    glm::ivec4 viewport;
  };
  struct DirectionalLightShadowMapView {
    int light_index;
    int split_index;
    glm::ivec4 viewport;
  };
  struct ForwardRenderingView {
    int camera_index;
    glm::ivec4 viewport;
  };
  /**
   * \brief Register per-frame function to render to all point light shadow maps.
   * \param func Render function targeting point light shadow map. Return primitive count.
   */
  void RenderToPointLightShadowMap(std::function<uint32_t(VkCommandBuffer vk_command_buffer,
                                                          const PointLightShadowMapView& shadow_map_view)>&& func);
  /**
   * \brief Register per-frame function to render to all spot light shadow maps.
   * \param func Render function targeting point light shadow map. Return primitive count.
   */
  void RenderToSpotLightShadowMap(
      std::function<uint32_t(VkCommandBuffer vk_command_buffer, const SpotLightShadowMapView& shadow_map_view)>&& func);

  /**
   * \brief Register per-frame function to render to all directional light shadow maps.
   * \param func Render function targeting point light shadow map. Return primitive count.
   */
  void RenderToDirectionalLightShadowMap(
      std::function<uint32_t(VkCommandBuffer vk_command_buffer, const DirectionalLightShadowMapView& shadow_map_view)>&&
          func);

  /**
   * \brief Register per-frame function to render to all cameras.
   * \param func Render function targeting all cameras. Return primitive count.
   */
  void ForwardRenderingAllCameras(
      std::function<uint32_t(VkCommandBuffer vk_command_buffer, const std::shared_ptr<Camera>& target_camera,
                             const ForwardRenderingView& forward_rendering_view)>&& func);

#pragma region DescriptorSet Layouts
  inline static std::shared_ptr<DescriptorSetLayout> per_frame_layout;
  inline static std::shared_ptr<DescriptorSetLayout> meshlet_layout;

  inline static std::shared_ptr<DescriptorSetLayout> lighting_layout;
  inline static std::shared_ptr<DescriptorSetLayout> ray_tracing_layout;

#pragma endregion
 private:
  std::vector<
      std::function<uint32_t(VkCommandBuffer vk_command_buffer, const PointLightShadowMapView& shadow_map_view)>>
      point_light_shadow_map_external_functions;
  std::vector<std::function<uint32_t(VkCommandBuffer vk_command_buffer, const SpotLightShadowMapView& shadow_map_view)>>
      spot_light_shadow_map_external_functions;
  std::vector<
      std::function<uint32_t(VkCommandBuffer vk_command_buffer, const DirectionalLightShadowMapView& shadow_map_view)>>
      directional_light_shadow_map_external_functions;
  std::vector<std::function<uint32_t(VkCommandBuffer vk_command_buffer, const std::shared_ptr<Camera>& target_camera,
                                     const ForwardRenderingView& forward_rendering_view)>>
      forward_rendering_external_functions;
  friend class Platform;
  friend class Resources;
  friend class Camera;
  friend class GraphicsPipeline;
  friend class EditorLayer;
  friend class Material;
  friend class Lighting;
  friend class PostProcessingStack;
  friend class Application;
  void OnCreate() override;
  void OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) override;
  void PreparePointAndSpotLightShadowMap() const;
  void PrepareEnvironmentalBrdfLut();
  void RenderToCamera(const GlobalTransform& camera_global_transform, const std::shared_ptr<Camera>& camera) const;
  void RenderToCameraRayTracing(const GlobalTransform& camera_global_transform,
                                const std::shared_ptr<Camera>& camera) const;
  void ClearAll() const;
  void PrepareForRendering();
  void RenderAll();
  void RenderGizmos() const;
  std::vector<std::shared_ptr<RenderInstanceStorage>> render_instances_list_;
  bool need_fade_ = false;
  bool UpdateRenderInstanceStorage(const std::shared_ptr<Scene>& scene, uint32_t current_frame_index);
  std::unique_ptr<Lighting> lighting_;
  friend class RenderInstanceStorage;
  std::shared_ptr<Texture2D> environmental_brdf_lut_ = {};
  void ApplyAnimators() const;
  friend class TextureStorage;
  std::vector<std::shared_ptr<DescriptorSet>> per_frame_descriptor_sets_ = {};
  std::vector<std::shared_ptr<DescriptorSet>> meshlet_descriptor_sets_ = {};
  std::vector<std::shared_ptr<DescriptorSet>> ray_tracing_descriptor_sets_ = {};
  std::vector<std::shared_ptr<Buffer>> kernel_descriptor_buffers_ = {};

#pragma region Graphics Pipelines
  // Shadow map pre-pass
  std::shared_ptr<GraphicsPipeline> point_light_shadow_pipeline_normal;
  std::shared_ptr<GraphicsPipeline> point_light_shadow_pipeline_mesh_shader;
  std::shared_ptr<GraphicsPipeline> spot_light_shadow_pipeline_normal;
  std::shared_ptr<GraphicsPipeline> spot_light_shadow_pipeline_mesh_shader;
  std::shared_ptr<GraphicsPipeline> directional_light_shadow_pipeline_normal;
  std::shared_ptr<GraphicsPipeline> directional_light_shadow_pipeline_mesh_shader;

  std::shared_ptr<GraphicsPipeline> instanced_point_light_shadow_pipeline;
  std::shared_ptr<GraphicsPipeline> instanced_spot_light_shadow_pipeline;
  std::shared_ptr<GraphicsPipeline> instanced_directional_light_shadow_pipeline;

  std::shared_ptr<GraphicsPipeline> skinned_point_light_shadow_pipeline;
  std::shared_ptr<GraphicsPipeline> skinned_spot_light_shadow_pipeline;
  std::shared_ptr<GraphicsPipeline> skinned_directional_light_shadow_pipeline;

  std::shared_ptr<GraphicsPipeline> strands_point_light_shadow_pipeline;
  std::shared_ptr<GraphicsPipeline> strands_spot_light_shadow_pipeline;
  std::shared_ptr<GraphicsPipeline> strands_directional_light_shadow_pipeline;

  // Deferred shading GBuffer pre-pass
  std::shared_ptr<GraphicsPipeline> deferred_prepass_pipeline_normal;
  std::shared_ptr<GraphicsPipeline> deferred_prepass_pipeline_mesh;
  std::shared_ptr<GraphicsPipeline> instanced_deferred_prepass_pipeline;
  std::shared_ptr<GraphicsPipeline> skinned_deferred_prepass_pipeline;
  std::shared_ptr<GraphicsPipeline> strands_deferred_prepass_pipeline;
  // Deferred shading lighting pass
  std::shared_ptr<GraphicsPipeline> deferred_lighting_pass_pipeline;
  std::shared_ptr<GraphicsPipeline> deferred_lighting_pass_pipeline_scene_camera;

  // Gizmos rendering
  std::shared_ptr<GraphicsPipeline> gizmos;
  std::shared_ptr<GraphicsPipeline> gizmos_normal_colored;
  std::shared_ptr<GraphicsPipeline> gizmos_vertex_colored;
  std::shared_ptr<GraphicsPipeline> gizmos_instanced_colored;
  std::shared_ptr<GraphicsPipeline> gizmos_strands;
  std::shared_ptr<GraphicsPipeline> gizmos_strands_normal_colored;
  std::shared_ptr<GraphicsPipeline> gizmos_strands_vertex_colored;

  std::shared_ptr<GraphicsPipeline> render_texture_present_pipeline;
#pragma endregion
#pragma region Ray Tracing Pipelines
  std::shared_ptr<RayTracingPipeline> ray_tracing_camera_pipeline;
#pragma endregion
};
}  // namespace evo_engine
