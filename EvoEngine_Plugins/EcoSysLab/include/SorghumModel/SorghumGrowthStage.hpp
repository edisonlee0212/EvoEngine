#pragma once
#include "Plot2D.hpp"

#include "Curve.hpp"
using namespace evo_engine;
namespace eco_sys_lab {
#pragma region States
enum class StateMode { Default, CubicBezier };

struct SorghumPanicleGrowthStage {
  glm::vec3 m_panicleSize = glm::vec3(0, 0, 0);
  int m_seedAmount = 0;
  float m_seedRadius = 0.002f;

  bool m_saved = false;
  SorghumPanicleGrowthStage();
  bool OnInspect();
  void Serialize(YAML::Emitter& out) const;
  void Deserialize(const YAML::Node& in);
};
struct SorghumStemGrowthStage {
  BezierSpline m_spline;
  glm::vec3 m_direction = {0, 1, 0};
  Plot2D<float> m_widthAlongStem;
  float m_length = 0;

  bool m_saved = false;
  SorghumStemGrowthStage();
  [[nodiscard]] glm::vec3 GetPoint(float point) const;
  void Serialize(YAML::Emitter& out) const;
  void Deserialize(const YAML::Node& in);
  bool OnInspect(int mode);
};
struct SorghumLeafGrowthStage {
  bool m_dead = false;
  BezierSpline m_spline;
  int m_index = 0;
  float m_startingPoint = 0;
  float m_length = 0.35f;
  float m_rollAngle = 0;
  float m_branchingAngle = 0;

  Plot2D<float> m_widthAlongLeaf;
  Plot2D<float> m_curlingAlongLeaf;
  Plot2D<float> m_bendingAlongLeaf;
  Plot2D<float> m_wavinessAlongLeaf;
  glm::vec2 m_wavinessPeriodStart = glm::vec2(0.0f);
  float m_wavinessFrequency = 0.0f;

  bool m_saved = false;
  SorghumLeafGrowthStage();
  void CopyShape(const SorghumLeafGrowthStage& another);
  void Serialize(YAML::Emitter& out) const;
  void Deserialize(const YAML::Node& in);
  bool OnInspect(int mode);
};
#pragma endregion

class SorghumGrowthStage {
  friend class SorghumGrowthStages;
  unsigned m_version = 0;

 public:
  SorghumGrowthStage();
  bool m_saved = false;
  std::string m_name = "Unnamed";
  SorghumPanicleGrowthStage m_panicle;
  SorghumStemGrowthStage m_stem;
  std::vector<SorghumLeafGrowthStage> m_leaves;
  bool OnInspect(int mode);

  void Serialize(YAML::Emitter& out) const;
  void Deserialize(const YAML::Node& in);
};
}  // namespace eco_sys_lab