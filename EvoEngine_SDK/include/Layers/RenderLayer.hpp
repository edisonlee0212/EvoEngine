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
  [[nodiscard]] std::shared_ptr<RenderInstanceStorage> GetCurrentRenderInstances() const;
  bool wire_frame = false;
  bool count_shadow_rendering_draw_calls = true;
  bool enable_indirect_rendering = true;
  bool enable_render_menu = false;
  RenderSettings render_settings{};
  [[nodiscard]] uint32_t DrawMesh(const std::shared_ptr<Mesh>& mesh, const std::shared_ptr<Material>& material,
                                  const GlobalTransform& global_transform, bool cast_shadow) const;
  [[nodiscard]] const std::shared_ptr<DescriptorSet>& GetPerFrameDescriptorSet() const;
  [[nodiscard]] const std::shared_ptr<DescriptorSet>& GetLightingDescriptorSet() const;

  struct PointLightShadowMapView {
    int light_index;
    int face;
    glm::ivec4 viewport;
  };
  struct SpotLightShadowMapView {
    int light_index;
    glm::ivec4 viewport;
  };
  struct DirectionalLightShadowMapView {
    int light_index;
    int split;
    glm::ivec4 viewport;
  };
  void RenderToPointLightShadowMap(std::function<uint32_t(VkCommandBuffer vk_command_buffer,
                                                          const PointLightShadowMapView& shadow_map_view)>&& func);
  void RenderToSpotLightShadowMap(
      std::function<uint32_t(VkCommandBuffer vk_command_buffer, const SpotLightShadowMapView& shadow_map_view)>&& func);
  void RenderToDirectionalLightShadowMap(
      std::function<uint32_t(VkCommandBuffer vk_command_buffer, const DirectionalLightShadowMapView& shadow_map_view)>&&
          func);
 private:
  std::vector<
      std::function<uint32_t(VkCommandBuffer vk_command_buffer, const PointLightShadowMapView& shadow_map_view)>>
      point_light_shadow_map_external_functions;
  std::vector<std::function<uint32_t(VkCommandBuffer vk_command_buffer, const SpotLightShadowMapView& shadow_map_view)>>
      spot_light_shadow_map_external_functions;
  std::vector<
      std::function<uint32_t(VkCommandBuffer vk_command_buffer, const DirectionalLightShadowMapView& shadow_map_view)>>
      directional_light_shadow_map_external_functions;

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
};
}  // namespace evo_engine
