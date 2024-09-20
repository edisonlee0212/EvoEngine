#pragma once
#include "Plot2D.hpp"

#include "Curve.hpp"
using namespace evo_engine;
namespace digital_agriculture_plugin {
#pragma region States
enum class StateMode { Default, CubicBezier };

struct SorghumPanicleGrowthStage {
  glm::vec3 panicle_size = glm::vec3(0, 0, 0);
  int seed_amount = 0;
  float seed_radius = 0.002f;

  bool saved = false;
  SorghumPanicleGrowthStage();
  bool OnInspect();
  void Serialize(YAML::Emitter& out) const;
  void Deserialize(const YAML::Node& in);
};
struct SorghumStemGrowthStage {
  BezierSpline spline;
  glm::vec3 direction = {0, 1, 0};
  Plot2D<float> width_along_stem;
  float length = 0;

  bool saved = false;
  SorghumStemGrowthStage();
  [[nodiscard]] glm::vec3 GetPoint(float point) const;
  void Serialize(YAML::Emitter& out) const;
  void Deserialize(const YAML::Node& in);
  bool OnInspect(int mode);
};
struct SorghumLeafGrowthStage {
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
  SorghumLeafGrowthStage();
  void CopyShape(const SorghumLeafGrowthStage& another);
  void Serialize(YAML::Emitter& out) const;
  void Deserialize(const YAML::Node& in);
  bool OnInspect(int mode);
};
#pragma endregion

class SorghumGrowthStage {
  friend class SorghumGrowthStages;
  unsigned version_ = 0;

 public:
  SorghumGrowthStage();
  bool saved = false;
  std::string name = "Unnamed";
  SorghumPanicleGrowthStage panicle;
  SorghumStemGrowthStage stem;
  std::vector<SorghumLeafGrowthStage> leaves;
  bool OnInspect(int mode);

  void Serialize(YAML::Emitter& out) const;
  void Deserialize(const YAML::Node& in);
};
}  // namespace digital_agriculture_plugin