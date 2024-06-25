#pragma once
#ifdef BUILD_WITH_RAYTRACER
#  include <CUDAModule.hpp>
#endif
#include "ILayer.hpp"
#include "PointCloud.hpp"
#include "SorghumField.hpp"
#include "SorghumState.hpp"
using namespace evo_engine;
namespace eco_sys_lab {
class SorghumLayer : public ILayer {
  static void ObjExportHelper(glm::vec3 position, std::shared_ptr<Mesh> mesh, std::ofstream& of, unsigned& start_index);

 public:
#ifdef BUILD_WITH_RAYTRACER
#  pragma region Illumination
  int m_seed = 0;
  float push_distance = 0.001f;
  RayProperties ray_properties;

  bool enable_compressed_btf = false;
  std::vector<Entity> processing_entities;
  int processing_index;
  bool processing = false;
  float light_probe_size = 0.05f;
  float per_plant_calculation_time = 0.0f;
  void CalculateIlluminationFrameByFrame();
  void CalculateIllumination();

#  pragma endregion
#endif
  SorghumMeshGeneratorSettings sorghum_mesh_generator_settings;
  bool auto_refresh_sorghums = true;

  AssetRef panicle_material;

  AssetRef leaf_bottom_face_material;
  AssetRef leaf_material;
  AssetRef leaf_cbtf_group;

  AssetRef leaf_albedo_texture;
  AssetRef leaf_normal_texture;
  AssetRef segmented_leaf_materials[25];

  float vertical_subdivision_length = 0.01f;
  int horizontal_subdivision_step = 4;
  float skeleton_width = 0.0025f;

  glm::vec3 skeleton_color = glm::vec3(0);

  void OnCreate() override;
  void GenerateMeshForAllSorghums(const SorghumMeshGeneratorSettings& sorghum_mesh_generator_settings) const;
  void OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) override;
  void Update() override;
  void LateUpdate() override;

  static void ExportSorghum(const Entity& sorghum, std::ofstream& of, unsigned& start_index);
  void ExportAllSorghumsModel(const std::string& filename);
};

}  // namespace eco_sys_lab
