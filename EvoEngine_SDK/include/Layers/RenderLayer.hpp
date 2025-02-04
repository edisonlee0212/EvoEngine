
#pragma once
#include "Camera.hpp"
#include "IGeometry.hpp"
#include "ILayer.hpp"

#include "Material.hpp"
#include "Mesh.hpp"
#include "RenderInstanceStorage.hpp"
namespace evo_engine {

/**
 * \class RenderLayer
 * \brief Represents a render layer in the rendering engine.
 *
 * Handles various rendering tasks, such as camera iteration, shadow map rendering,
 * deferred rendering, forward rendering, and more.
 */
class RenderLayer final : public ILayer {
 public:
  /**
   * \brief Iterates through all collected cameras and applies the specified action.
   * \param action Function to apply to each camera.
   */
  void ForEachCollectedCamera(const std::function<void(const std::shared_ptr<Camera>& camera)>& action) const;

  /**
   * \brief Retrieves the current render instance storage.
   * \return A shared pointer to the current render instance storage.
   */
  [[nodiscard]] std::shared_ptr<RenderInstanceStorage> GetCurrentRenderInstanceStorage() const;

  /**
   * \brief Retrieves the previous render instance storage.
   * \return A shared pointer to the previous render instance storage.
   */
  [[nodiscard]] std::shared_ptr<RenderInstanceStorage> GetPreviousRenderInstanceStorage() const;

  /// Specifies whether wireframe rendering is enabled.
  bool wire_frame = false;

  /// Specifies whether shadow-rendering draw calls should be counted.
  bool count_shadow_rendering_draw_calls = true;

  /// Specifies whether indirect rendering is enabled.
  bool enable_indirect_rendering = true;

  /// Specifies the rendering settings.
  RenderSettings render_settings{};

  /**
   * \brief Draws a mesh.
   * \param mesh The mesh to draw.
   * \param material The material to use for rendering the mesh.
   * \param global_transform The global transform of the mesh.
   * \param cast_shadow Specifies whether the mesh casts a shadow.
   * \return Number of primitives drawn.
   */
  [[maybe_unused]] uint32_t DrawMesh(const std::shared_ptr<Mesh>& mesh, const std::shared_ptr<Material>& material,
                                     const GlobalTransform& global_transform, bool cast_shadow) const;

  /**
   * \brief Draws a mesh with instancing.
   * \param mesh The mesh to draw.
   * \param material The material to use for rendering the mesh.
   * \param global_transform The global transform of the mesh.
   * \param particle_info_list Instance particle information.
   * \param cast_shadow Specifies whether the mesh casts a shadow.
   * \return Number of primitives drawn.
   */
  [[maybe_unused]] uint32_t DrawMeshInstanced(const std::shared_ptr<Mesh>& mesh,
                                              const std::shared_ptr<Material>& material,
                                              const GlobalTransform& global_transform,
                                              const std::shared_ptr<ParticleInfoList>& particle_info_list,
                                              bool cast_shadow) const;

  /**
   * \brief Retrieves the per-frame descriptor set.
   * \return A shared pointer to the per-frame descriptor set.
   */
  [[nodiscard]] static const std::shared_ptr<DescriptorSet>& GetPerFrameDescriptorSet();

  /**
   * \brief Retrieves the lighting descriptor set.
   * \return A shared pointer to the lighting descriptor set.
   */
  [[nodiscard]] static const std::shared_ptr<DescriptorSet>& GetLightingDescriptorSet();

  /// Represents the view for a specific point light shadow map.
  struct PointLightShadowMapView {
    int light_index;      ///< Index of the light.
    int face_index;       ///< Index of the face for cube map rendering.
    glm::ivec4 viewport;  ///< The viewport rectangle for rendering.
  };

  /// Represents the view for a specific spot light shadow map.
  struct SpotLightShadowMapView {
    int light_index;      ///< Index of the light.
    glm::ivec4 viewport;  ///< The viewport rectangle for rendering.
  };

  /// Represents the view for a specific directional light shadow map.
  struct DirectionalLightShadowMapView {
    int light_index;      ///< Index of the light.
    int split_index;      ///< Index of the split for cascaded shadow maps.
    glm::ivec4 viewport;  ///< The viewport rectangle for rendering.
  };

  /// Represents the view for deferred rendering.
  struct DeferredRenderingView {
    int camera_index;     ///< Index of the target camera.
    glm::ivec4 viewport;  ///< The viewport rectangle for rendering.
  };

  /// Represents the view for forward rendering.
  struct ForwardRenderingView {
    int camera_index;     ///< Index of the target camera.
    glm::ivec4 viewport;  ///< The viewport rectangle for rendering.
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
   * \brief Register per-frame function to render to all cameras using deferred rendering.
   * \param func Render function targeting deferred rendering cameras. Return primitive count.
   */
  void DeferredRenderingAllCameras(
      std::function<uint32_t(VkCommandBuffer vk_command_buffer,
                             const std::vector<VkRenderingAttachmentInfo>& geometry_pass_color_attachment_infos,
                             const DeferredRenderingView& forward_rendering_view)>&& func);

  /**
   * \brief Register per-frame function to render to all cameras using forward rendering.
   * \param func Render function targeting forward rendering cameras. Return primitive count.
   */
  void ForwardRenderingAllCameras(
      std::function<uint32_t(VkCommandBuffer vk_command_buffer, const std::shared_ptr<Camera>& target_camera,
                             const ForwardRenderingView& forward_rendering_view)>&& func);

#pragma region DescriptorSet Layouts
  /// Descriptor set layout for per-frame data.
  inline static std::shared_ptr<DescriptorSetLayout> per_frame_layout;

  /// Descriptor set layout for meshlet data.
  inline static std::shared_ptr<DescriptorSetLayout> meshlet_layout;

  /// Descriptor set layout for lighting.
  inline static std::shared_ptr<DescriptorSetLayout> lighting_layout;

  /// Descriptor set layout for ray tracing data.
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

  std::vector<std::function<uint32_t(VkCommandBuffer vk_command_buffer,
                                     const std::vector<VkRenderingAttachmentInfo>& geometry_pass_color_attachment_infos,
                                     const DeferredRenderingView& forward_rendering_view)>>
      deferred_rendering_external_functions;

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
  friend class RenderInstanceStorage;
  friend class TextureStorage;
  std::vector<std::shared_ptr<RenderInstanceStorage>> render_instances_list_;
  bool need_fade_ = false;
  std::unique_ptr<Lighting> lighting_;
  std::shared_ptr<Texture2D> environmental_brdf_lut_ = {};
  /**
   * \brief Called after the RenderLayer object is created.
   */
  void OnCreate() override;

  /**
   * \brief Provides a user interface in the editor to inspect and modify this render layer.
   * \param editor_layer The current editor layer.
   */
  void OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) override;

  /**
   * \brief Prepares shadow maps for point and spot lights.
   */
  void PreparePointAndSpotLightShadowMap() const;

  /**
   * \brief Prepares the environmental BRDF LUT texture.
   */
  void PrepareEnvironmentalBrdfLut();

  /**
   * \brief Renders to the specified camera.
   * \param camera_global_transform The global transform of the camera.
   * \param camera The target camera to render to.
   */
  void RenderToCamera(const GlobalTransform& camera_global_transform, const std::shared_ptr<Camera>& camera) const;

  /**
   * \brief Renders to the specified camera using ray tracing.
   * \param camera_global_transform The global transform of the camera.
   * \param camera The target camera to render to.
   */
  void RenderToCameraRayTracing(const GlobalTransform& camera_global_transform,
                                const std::shared_ptr<Camera>& camera) const;

  /**
   * \brief Called before updating this render layer.
   */
  void PreUpdate() override;

  /**
   * \brief Clears all editor cameras associated with this render layer.
   */
  void ClearAllEditorCameras() const;

  /**
   * \brief Clears all cameras associated with this render layer.
   */
  void ClearAllCameras() const;

  /**
   * \brief Prepares the render layer for rendering.
   */
  void PrepareForRendering();

  /**
   * \brief Performs all rendering operations for this render layer.
   */
  void RenderAll();

  /**
   * \brief Renders all gizmos associated with this render layer.
   */
  void RenderGizmos() const;

  /**
   * \brief Updates the render instance storage based on the provided scene.
   * \param scene The scene associated with this render layer.
   * \param current_frame_index The index of the current frame.
   * \return True if the render instance storage was updated, otherwise false.
   */
  bool UpdateRenderInstanceStorage(const std::shared_ptr<Scene>& scene, uint32_t current_frame_index);

  /**
   * \brief Applies all animators associated with this render layer.
   */
  void ApplyAnimators() const;
  friend class TextureStorage;
  std::vector<std::shared_ptr<DescriptorSet>> per_frame_descriptor_sets_ = {};
  std::vector<std::shared_ptr<DescriptorSet>> meshlet_descriptor_sets_ = {};
  std::vector<std::shared_ptr<DescriptorSet>> ray_tracing_descriptor_sets_ = {};
  std::vector<std::shared_ptr<Buffer>> kernel_descriptor_buffers_ = {};

#pragma region Graphics Pipelines
  /// Graphics pipeline for rendering point light shadows with normal meshes.
  std::shared_ptr<GraphicsPipeline> point_light_shadow_pipeline_normal;

  /// Graphics pipeline for rendering point light shadows with mesh shaders.
  std::shared_ptr<GraphicsPipeline> point_light_shadow_pipeline_mesh_shader;

  /// Graphics pipeline for rendering spot light shadows with normal meshes.
  std::shared_ptr<GraphicsPipeline> spot_light_shadow_pipeline_normal;

  /// Graphics pipeline for rendering spot light shadows with mesh shaders.
  std::shared_ptr<GraphicsPipeline> spot_light_shadow_pipeline_mesh_shader;

  /// Graphics pipeline for rendering directional light shadows with normal meshes.
  std::shared_ptr<GraphicsPipeline> directional_light_shadow_pipeline_normal;

  /// Graphics pipeline for rendering directional light shadows with mesh shaders.
  std::shared_ptr<GraphicsPipeline> directional_light_shadow_pipeline_mesh_shader;

  /// Graphics pipeline for rendering instanced point light shadows.
  std::shared_ptr<GraphicsPipeline> instanced_point_light_shadow_pipeline;

  /// Graphics pipeline for rendering instanced spot light shadows.
  std::shared_ptr<GraphicsPipeline> instanced_spot_light_shadow_pipeline;

  /// Graphics pipeline for rendering instanced directional light shadows.
  std::shared_ptr<GraphicsPipeline> instanced_directional_light_shadow_pipeline;

  /// Graphics pipeline for rendering point light shadows with skinned meshes.
  std::shared_ptr<GraphicsPipeline> skinned_point_light_shadow_pipeline;

  /// Graphics pipeline for rendering spot light shadows with skinned meshes.
  std::shared_ptr<GraphicsPipeline> skinned_spot_light_shadow_pipeline;

  /// Graphics pipeline for rendering directional light shadows with skinned meshes.
  std::shared_ptr<GraphicsPipeline> skinned_directional_light_shadow_pipeline;

  /// Graphics pipeline for rendering point light shadows with hair strands.
  std::shared_ptr<GraphicsPipeline> strands_point_light_shadow_pipeline;

  /// Graphics pipeline for rendering spot light shadows with hair strands.
  std::shared_ptr<GraphicsPipeline> strands_spot_light_shadow_pipeline;

  /// Graphics pipeline for rendering directional light shadows with hair strands.
  std::shared_ptr<GraphicsPipeline> strands_directional_light_shadow_pipeline;

  /// Graphics pipeline for the deferred shading GBuffer pre-pass using normal meshes.
  std::shared_ptr<GraphicsPipeline> deferred_prepass_pipeline_normal;

  /// Graphics pipeline for the deferred shading GBuffer pre-pass using mesh shaders.
  std::shared_ptr<GraphicsPipeline> deferred_prepass_pipeline_mesh;

  /// Graphics pipeline for rendering instanced deferred shading GBuffer pre-pass.
  std::shared_ptr<GraphicsPipeline> instanced_deferred_prepass_pipeline;

  /// Graphics pipeline for rendering deferred shading GBuffer pre-pass with skinned meshes.
  std::shared_ptr<GraphicsPipeline> skinned_deferred_prepass_pipeline;

  /// Graphics pipeline for rendering deferred shading GBuffer pre-pass with hair strands.
  std::shared_ptr<GraphicsPipeline> strands_deferred_prepass_pipeline;

  /// Graphics pipeline for performing the deferred shading lighting pass.
  std::shared_ptr<GraphicsPipeline> deferred_lighting_pass_pipeline;

  /// Graphics pipeline for performing the deferred shading lighting pass with scene cameras.
  std::shared_ptr<GraphicsPipeline> deferred_lighting_pass_pipeline_scene_camera;

  /// Graphics pipeline for rendering gizmos.
  std::shared_ptr<GraphicsPipeline> gizmos;

  /// Graphics pipeline for rendering gizmos with normal-colored shaders.
  std::shared_ptr<GraphicsPipeline> gizmos_normal_colored;

  /// Graphics pipeline for rendering gizmos with vertex-colored shaders.
  std::shared_ptr<GraphicsPipeline> gizmos_vertex_colored;

  /// Graphics pipeline for rendering instanced gizmos.
  std::shared_ptr<GraphicsPipeline> gizmos_instanced_colored;

  /// Graphics pipeline for rendering gizmos on hair strands.
  std::shared_ptr<GraphicsPipeline> gizmos_strands;

  /// Graphics pipeline for rendering gizmos with normal-colored shaders on hair strands.
  std::shared_ptr<GraphicsPipeline> gizmos_strands_normal_colored;

  /// Graphics pipeline for rendering gizmos with vertex-colored shaders on hair strands.
  std::shared_ptr<GraphicsPipeline> gizmos_strands_vertex_colored;
#pragma endregion

#pragma region Ray Tracing Pipelines
  /// Ray tracing pipeline for rendering cameras with ray tracing.
  std::shared_ptr<RayTracingPipeline> ray_tracing_camera_pipeline;
#pragma endregion
};
}  // namespace evo_engine
