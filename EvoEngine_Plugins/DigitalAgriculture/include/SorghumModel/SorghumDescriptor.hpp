#pragma once
#include "SorghumSpline.hpp"
using namespace evo_engine;
namespace digital_agriculture_plugin {
struct SorghumMeshGeneratorSettings {
  bool enable_panicle = true;
  bool enable_stem = true;
  bool enable_leaves = true;
  bool enable_leaf_stem = false;
  int single_leaf_index = -1;
  bool bottom_face = true;
  bool leaf_separated = false;
  float leaf_thickness = 0.001f;
  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer);
};

class SorghumPanicleState {
 public:
  glm::vec3 panicle_size = glm::vec3(0, 0, 0);
  int seed_amount = 0;
  float seed_radius = 0.002f;
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
  SorghumSpline spline;
  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer);
  void Serialize(YAML::Emitter& out) const;
  void Deserialize(const YAML::Node& in);

  void GenerateGeometry(std::vector<Vertex>& vertices, std::vector<unsigned int>& indices) const;
};

class SorghumLeafState {
 public:
  int index = 0;
  SorghumSpline spline;
  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer);
  void Serialize(YAML::Emitter& out) const;
  void Deserialize(const YAML::Node& in);

  void GenerateGeometry(std::vector<Vertex>& vertices, std::vector<unsigned int>& indices,
                        const SorghumMeshGeneratorSettings& mesh_generator_settings, bool current_bottom_face = false) const;
};

class SorghumDescriptor : public IAsset {
 public:
  SorghumPanicleState panicle;
  SorghumStemState stem;
  std::vector<SorghumLeafState> leaves;
  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) override;
  void Serialize(YAML::Emitter& out) const override;
  void Deserialize(const YAML::Node& in) override;
  [[maybe_unused]] Entity CreateEntity(const std::string& name) const;
};

}  // namespace digital_agriculture_plugin