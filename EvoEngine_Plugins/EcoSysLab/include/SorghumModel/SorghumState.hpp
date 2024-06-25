#pragma once
#include "SorghumSpline.hpp"
using namespace evo_engine;
namespace eco_sys_lab {
struct SorghumMeshGeneratorSettings {
  bool m_enablePanicle = true;
  bool m_enableStem = true;
  bool m_enableLeaves = true;
  bool m_bottomFace = true;
  bool m_leafSeparated = false;
  float m_leafThickness = 0.001f;
  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer);
};

class SorghumPanicleState {
 public:
  glm::vec3 m_panicleSize = glm::vec3(0, 0, 0);
  int m_seedAmount = 0;
  float m_seedRadius = 0.002f;
  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer);
  void Serialize(YAML::Emitter& out) const;
  void Deserialize(const YAML::Node& in);

  void GenerateGeometry(const glm::vec3& stem_tip, std::vector<Vertex>& vertices,
                        std::vector<unsigned int>& indices) const;
  void GenerateGeometry(const glm::vec3& stem_tip, std::vector<Vertex>& vertices, std::vector<unsigned int>& indices,
                        const std::shared_ptr<ParticleInfoList>& particle_info_list) const;
};

class SorghumStemState {
 public:
  SorghumSpline m_spline;
  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer);
  void Serialize(YAML::Emitter& out) const;
  void Deserialize(const YAML::Node& in);

  void GenerateGeometry(std::vector<Vertex>& vertices, std::vector<unsigned int>& indices) const;
};

class SorghumLeafState {
 public:
  int m_index = 0;
  SorghumSpline m_spline;
  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer);
  void Serialize(YAML::Emitter& out) const;
  void Deserialize(const YAML::Node& in);

  void GenerateGeometry(std::vector<Vertex>& vertices, std::vector<unsigned int>& indices, bool bottom_face = false,
                        float thickness = 0.0f) const;
};

class SorghumState : public IAsset {
 public:
  SorghumPanicleState m_panicle;
  SorghumStemState m_stem;
  std::vector<SorghumLeafState> m_leaves;
  bool OnInspect(const std::shared_ptr<EditorLayer>& editorLayer) override;
  void Serialize(YAML::Emitter& out) const override;
  void Deserialize(const YAML::Node& in) override;
};

}  // namespace eco_sys_lab