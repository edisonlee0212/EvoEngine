#pragma once
#include "Plot2D.hpp"
#include "SorghumDescriptor.hpp"
#include "Curve.hpp"
using namespace evo_engine;
namespace digital_agriculture_plugin {
#pragma region States
enum class StateMode { Default, CubicBezier };

struct SorghumPanicleState {
  glm::vec3 panicle_size = glm::vec3(0, 0, 0);
  int seed_amount = 0;
  float seed_radius = 0.002f;

  bool saved = false;
  SorghumPanicleState();
  bool OnInspectImpl();
  void Serialize(YAML::Emitter& out) const;
  void Deserialize(const YAML::Node& in);
  void Apply(SorghumPanicleDescriptor &target_sorghum_panicle_descriptor) const;
};
struct SorghumStemState {
  BezierSpline spline;
  glm::vec3 direction = {0, 1, 0};
  Plot2D<float> width_along_stem;
  float length = 0;

  bool saved = false;
  SorghumStemState();
  [[nodiscard]] glm::vec3 GetPoint(float point) const;
  void Serialize(YAML::Emitter& out) const;
  void Deserialize(const YAML::Node& in);
  bool OnInspectImpl(int mode);
  void Apply(SorghumStemDescriptor& target_sorghum_stem_descriptor) const;
};
struct SorghumLeafState {
  bool dead = false;
  BezierSpline spline;
  int index = 0;
  float starting_point = 0;
  float length = 0.35f;
  float roll_angle = 0;
  float branching_angle = 0;

  Plot2D<float> width_along_leaf;
  Plot2D<float> curling_along_leaf;
  Plot2D<float> bending_along_leaf;
  Plot2D<float> waviness_along_leaf;
  glm::vec2 waviness_period_start = glm::vec2(0.0f);
  float waviness_frequency = 0.0f;

  bool saved = false;
  SorghumLeafState();
  void CopyShape(const SorghumLeafState& another);
  void Serialize(YAML::Emitter& out) const;
  void Deserialize(const YAML::Node& in);
  bool OnInspectImpl(int mode);
  void Apply(const SorghumStemState& stem_state, SorghumLeafDescriptor& target_sorghum_leaf_descriptor) const;

  void ChangeWaviness(float factor,
                      const SorghumStemState& stem_state,
                      const SorghumMeshGeneratorSettings& mesh_generator_settings, SorghumLeafState& target_leaf_state) const;
};
#pragma endregion

class SorghumState : public IAsset {
  friend class SorghumGrowthStages;
 public:
  SorghumState();
  bool saved = false;
  std::string name = "Unnamed";
  SorghumPanicleState panicle;
  SorghumStemState stem;
  std::vector<SorghumLeafState> leaves;
  bool OnInspectImpl(int mode);
  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) override;
  void Apply(const std::shared_ptr<SorghumDescriptor>& target_sorghum_descriptor) const;
  void Serialize(YAML::Emitter& out) const override;
  void Deserialize(const YAML::Node& in) override;
  [[maybe_unused]] Entity CreateEntity(const std::string& name) const;


  void ChangeWaviness(float factor, const SorghumMeshGeneratorSettings& mesh_generator_settings, SorghumState& target_sorghum_state) const;
};
}  // namespace digital_agriculture_plugin