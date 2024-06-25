#pragma once

#include "Plot2D.hpp"
#include "SorghumGrowthStages.hpp"
#include "SorghumState.hpp"
using namespace evo_engine;
namespace eco_sys_lab {

class SorghumStateGenerator : public IAsset {
 public:
  // Panicle
  SingleDistribution<glm::vec2> m_panicleSize;
  SingleDistribution<float> m_panicleSeedAmount;
  SingleDistribution<float> m_panicleSeedRadius;
  // Stem
  SingleDistribution<float> m_stemTiltAngle;
  SingleDistribution<float> m_internodeLength;
  SingleDistribution<float> m_stemWidth;
  // Leaf
  SingleDistribution<float> m_leafAmount;

  PlottedDistribution<float> m_leafStartingPoint;
  PlottedDistribution<float> m_leafCurling;
  PlottedDistribution<float> m_leafRollAngle;
  PlottedDistribution<float> m_leafBranchingAngle;

  PlottedDistribution<float> m_leafBending;
  PlottedDistribution<float> m_leafBendingAcceleration;
  PlottedDistribution<float> m_leafBendingSmoothness;

  // PlottedDistribution<float> m_leafSaggingBase;
  // PlottedDistribution<float> m_leafSaggingStrength;
  // PlottedDistribution<float> m_leafGravitropism;

  PlottedDistribution<float> m_leafWaviness;
  PlottedDistribution<float> m_leafWavinessFrequency;
  PlottedDistribution<float> m_leafLength;
  PlottedDistribution<float> m_leafWidth;

  // Finer control
  Curve2D m_widthAlongStem;
  Curve2D m_curlingAlongLeaf;
  Curve2D m_widthAlongLeaf;
  Curve2D m_wavinessAlongLeaf;
  void OnCreate() override;
  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) override;
  void Serialize(YAML::Emitter& out) const override;
  void Deserialize(const YAML::Node& in) override;

  [[nodiscard]] Entity CreateEntity(unsigned int seed = 0) const;
  void Apply(const std::shared_ptr<SorghumState>& target_state, unsigned int seed = 0) const;
};
}  // namespace eco_sys_lab