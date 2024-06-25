#pragma once
#include "EvoEngine_SDK_PCH.hpp"

#include "CUDAModule.hpp"
#include "CompressedBTF.hpp"
#include "Cubemap.hpp"
#include "ILayer.hpp"

#include "memory"

#include "Material.hpp"
namespace evo_engine {
class RayTracerCamera;
class RayTracerLayer : public ILayer {
 protected:
  void UpdateMeshesStorage(const std::shared_ptr<Scene>& scene,
                           std::unordered_map<uint64_t, RayTracedMaterial>& material_storage,
                           std::unordered_map<uint64_t, RayTracedGeometry>& geometry_storage,
                           std::unordered_map<uint64_t, RayTracedInstance>& instance_storage, bool& rebuild_instances,
                           bool& update_shader_binding_table) const;

  void SceneCameraWindow();

  static void RayCameraWindow();

  friend class RayTracerCamera;

  static std::shared_ptr<RayTracerCamera> ray_tracer_camera_;

  bool CheckMaterial(RayTracedMaterial& ray_tracer_material, const std::shared_ptr<Material>& material) const;

  static bool CheckCompressedBtf(RayTracedMaterial& ray_tracer_material,
                                 const std::shared_ptr<CompressedBTF>& compressed_btf);

  glm::ivec2 scene_camera_resolution_ = glm::ivec2(0);

 public:
  bool show_scene_info = false;
  bool render_mesh_renderer = true;
  bool render_strands_renderer = true;
  bool render_particles = true;
  bool render_btf_mesh_renderer = true;
  bool render_skinned_mesh_renderer = false;
  [[nodiscard]] glm::ivec2 GetSceneCameraResolution() const;
  bool show_ray_tracer_settings_window = false;
  EnvironmentProperties environment_properties;
  std::shared_ptr<CudaImage> environmental_map_image;
  Handle environmental_map_handle = 0;

  bool show_scene_window = false;
  bool show_camera_window = false;

  bool rendering_enabled = true;
  float resolution_multiplier = 1.0f;
  std::shared_ptr<RayTracerCamera> scene_camera;

  bool UpdateScene(const std::shared_ptr<Scene>& scene);

  void OnCreate() override;

  void PreUpdate() override;
  void LateUpdate() override;

  void OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) override;

  void OnDestroy() override;
};
}  // namespace evo_engine