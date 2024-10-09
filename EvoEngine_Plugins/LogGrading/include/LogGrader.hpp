#pragma once
#include <Plot2D.hpp>

#include "BarkDescriptor.hpp"
#include "LogWood.hpp"
using namespace evo_engine;
using namespace eco_sys_lab_plugin;
namespace log_grading_plugin {
struct LogWoodMeshGenerationSettings {
  float m_y_subdivision = 0.02f;
};

class ProceduralLogParameters {
 public:
  bool m_bottom = true;
  bool m_sound_defect = false;
  float m_length_without_trim_in_feet = 16.0f;
  float m_length_step_in_inches = 1.f;
  float m_large_end_diameter_in_inches = 25.f;
  float m_small_end_diameter_in_inches = 20.f;

  unsigned m_mode = 0;
  float m_span_in_inches = 0.0f;
  float m_angle = 180.0f;
  float m_crook_ratio = 0.7f;

  bool OnInspect();
};

class LogGrader : public IPrivateComponent {
  std::shared_ptr<Mesh> m_tempCylinderMesh{};

  std::shared_ptr<ParticleInfoList> m_surface1;
  std::shared_ptr<ParticleInfoList> m_surface2;
  std::shared_ptr<ParticleInfoList> m_surface3;
  std::shared_ptr<ParticleInfoList> m_surface4;

  std::shared_ptr<Mesh> m_tempFlatMesh1{};
  std::shared_ptr<Mesh> m_tempFlatMesh2{};
  std::shared_ptr<Mesh> m_tempFlatMesh3{};
  std::shared_ptr<Mesh> m_tempFlatMesh4{};
  void RefreshMesh(const LogGrading& log_grading) const;

 public:
  int m_best_grading_index = 0;
  std::vector<LogGrading> m_available_best_grading{};
  ProceduralLogParameters m_procedural_log_parameters;
  AssetRef m_branch_shape{};
  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) override;
  void InitializeLogRandomly(const ProceduralLogParameters& procedural_log_parameters,
                             const std::shared_ptr<BarkDescriptor>& branch_shape);
  LogWoodMeshGenerationSettings m_log_wood_mesh_generation_settings{};
  LogWood m_log_wood{};
  void GenerateCylinderMesh(const std::shared_ptr<Mesh>& mesh,
                            const LogWoodMeshGenerationSettings& mesh_generator_settings) const;
  void GenerateFlatMesh(const std::shared_ptr<Mesh>& mesh, const LogWoodMeshGenerationSettings& mesh_generator_settings,
                        int start_x, int end_x) const;
  void GenerateSurface(const std::shared_ptr<ParticleInfoList>& surface,
                       const LogWoodMeshGenerationSettings& mesh_generator_settings, int start_x, int end_x) const;
  void OnCreate() override;
  void InitializeMeshRenderer(const LogWoodMeshGenerationSettings& mesh_generator_settings) const;
  void ClearMeshRenderer() const;
};
}  // namespace eco_sys_lab_plugin