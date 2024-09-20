#pragma once

#include "Plot2D.hpp"
#include "SorghumGrowthStages.hpp"
#include "SorghumDescriptor.hpp"
using namespace evo_engine;
namespace digital_agriculture_plugin {

class SorghumDescriptorGenerator : public IAsset {
 public:
  // Panicle
  SingleDistribution<glm::vec2> panicle_size;
  SingleDistribution<float> panicle_seed_amount;
  SingleDistribution<float> panicle_seed_radius;
  // Stem
  SingleDistribution<float> stem_tilt_angle;
  SingleDistribution<float> internode_length;
  SingleDistribution<float> stem_width;
  // Leaf
  SingleDistribution<float> leaf_amount;

  PlottedDistribution<float> leaf_starting_point;
  PlottedDistribution<float> leaf_curling;
  PlottedDistribution<float> leaf_roll_angle;
  PlottedDistribution<float> leaf_branching_angle;

  PlottedDistribution<float> leaf_bending;
  PlottedDistribution<float> leaf_bending_acceleration;
  PlottedDistribution<float> leaf_bending_smoothness;

  // PlottedDistribution<float> m_leafSaggingBase;
  // PlottedDistribution<float> m_leafSaggingStrength;
  // PlottedDistribution<float> m_leafGravitropism;

  PlottedDistribution<float> leaf_waviness;
  PlottedDistribution<float> leaf_waviness_frequency;
  PlottedDistribution<float> leaf_length;
  PlottedDistribution<float> leaf_width;

  // Finer control
  Curve2D width_along_stem;
  Curve2D curling_along_leaf;
  Curve2D width_along_leaf;
  Curve2D waviness_along_leaf;
  void OnCreate() override;
  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) override;
  void Serialize(YAML::Emitter& out) const override;
  void Deserialize(const YAML::Node& in) override;

  [[nodiscard]] Entity CreateEntity(unsigned int seed = 0) const;
  void Apply(const std::shared_ptr<SorghumDescriptor>& target_state, unsigned int seed = 0) const;
};
}  // namespace digital_agriculture_plugin