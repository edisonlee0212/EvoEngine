#pragma once
#include "Camera.hpp"
#include "IGeometry.hpp"
#include "ILayer.hpp"
#include "Lights.hpp"
#include "Material.hpp"
#include "Mesh.hpp"
#include "RenderInstances.hpp"
namespace evo_engine {
#pragma region Enums Structs

struct RenderInstancePushConstant {
  int instance_index = 0;
  int camera_index = 0;
  int light_split_index = 0;
};
struct RayTracingPushConstant {
  uint32_t camera_index = 0;
  uint32_t frame_id = 0;
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
  void RenderToCameraRayTracing(const GlobalTransform& camera_global_transform, const std::shared_ptr<Camera>& camera);
  
  void CollectCameras(const std::shared_ptr<Scene>& scene,
                      std::vector<std::pair<GlobalTransform, std::shared_ptr<Camera>>>& cameras);
  void ClearAllCameras();
  void RenderAllCameras();

 public:
  std::vector<std::shared_ptr<RenderInstances>> render_instances_list;

  bool wire_frame = false;

  bool count_shadow_rendering_draw_calls = true;
  bool enable_indirect_rendering = true;
  bool enable_debug_visualization = false;
  bool enable_render_menu = false;
  bool stable_fit = true;
  float max_shadow_distance = 100;
  float shadow_cascade_split[4] = {0.075f, 0.15f, 0.3f, 1.0f};

  [[nodiscard]] uint32_t GetCameraIndex(const Handle& handle);
  [[nodiscard]] uint32_t RegisterCameraIndex(const Handle& handle, const CameraInfoBlock& camera_info_block);

  RenderInfoBlock render_info_block = {};
  EnvironmentInfoBlock environment_info_block = {};
  void DrawMesh(const std::shared_ptr<Mesh>& mesh, const std::shared_ptr<Material>& material, glm::mat4 model,
                bool cast_shadow);

  [[nodiscard]] const std::shared_ptr<DescriptorSet>& GetPerFrameDescriptorSet() const;

 private:
  bool need_fade_ = false;
#pragma region Render procedure
  bool UpdateRenderInfo(const std::shared_ptr<Scene>& scene, uint32_t current_frame_index);
  bool UpdateEnvironmentInfo(const std::shared_ptr<Scene>& scene, uint32_t current_frame_index);
  bool UpdateCameras(const std::shared_ptr<Scene>& scene, uint32_t current_frame_index,
                     std::vector<std::pair<GlobalTransform, std::shared_ptr<Camera>>>& cameras);
  
  bool UpdateRenderInstances(const std::shared_ptr<Scene>& scene, uint32_t current_frame_index);
  bool UpdateLighting(const std::shared_ptr<Scene>& scene, uint32_t current_frame_index,
                      const std::vector<std::pair<GlobalTransform, std::shared_ptr<Camera>>>& cameras);

  void CollectDirectionalLights(const std::shared_ptr<Scene>& scene,
                                const std::vector<std::pair<GlobalTransform, std::shared_ptr<Camera>>>& cameras);
  void CollectPointLights(const std::shared_ptr<Scene>& scene, const GlobalTransform& view_point_gt);
  void CollectSpotLights(const std::shared_ptr<Scene>& scene, const GlobalTransform& view_point_gt);

  std::unique_ptr<Lighting> lighting_;
  std::shared_ptr<Texture2D> environmental_brdf_lut_ = {};

  void ApplyAnimators() const;

#pragma endregion
#pragma region Per Frame Descriptor Sets
  friend class TextureStorage;
  std::vector<std::shared_ptr<DescriptorSet>> per_frame_descriptor_sets_ = {};
  std::vector<std::shared_ptr<DescriptorSet>> meshlet_descriptor_sets_ = {};
  std::vector<std::shared_ptr<DescriptorSet>> ray_tracing_descriptor_sets_ = {};

  std::vector<std::shared_ptr<Buffer>> render_info_descriptor_buffers_ = {};
  std::vector<std::shared_ptr<Buffer>> environment_info_descriptor_buffers_ = {};
  std::vector<std::shared_ptr<Buffer>> camera_info_descriptor_buffers_ = {};

  std::vector<std::shared_ptr<Buffer>> kernel_descriptor_buffers_ = {};
  std::vector<std::shared_ptr<Buffer>> directional_light_info_descriptor_buffers_ = {};
  std::vector<std::shared_ptr<Buffer>> point_light_info_descriptor_buffers_ = {};
  std::vector<std::shared_ptr<Buffer>> spot_light_info_descriptor_buffers_ = {};

  void CreateStandardDescriptorBuffers();
  void CreateDescriptorSets();
  std::unordered_map<Handle, uint32_t> camera_indices_;

  std::vector<CameraInfoBlock> camera_info_blocks_{};
  std::vector<DirectionalLightInfo> directional_light_info_blocks_;
  std::vector<PointLightInfo> point_light_info_blocks_;
  std::vector<SpotLightInfo> spot_light_info_blocks_;

#pragma endregion
};
}  // namespace evo_engine
