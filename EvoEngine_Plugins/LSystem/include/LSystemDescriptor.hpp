#pragma once

#include <IAsset.hpp>
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include "LSystemGraph.hpp"

namespace l_system_plugin {

/**
 * @brief Serializable asset holding L-system derivation parameters.
 *
 * Stores the type-independent configuration for an L-system instance:
 * derivation step count, RNG seed, root transform, and display options.
 *
 * Since production rules and module types are defined in C++ templates
 * (Phase 1), this asset does not store the rules themselves — only the
 * parameters that control derivation and geometry. User code instantiates
 * the typed DerivationEngine and feeds it the parameters from this asset.
 */
class LSystemDescriptor : public evo_engine::IAsset {
 public:
  int derivation_steps = 5;          ///< Number of derivation iterations.
  unsigned int seed = 42;            ///< RNG seed for stochastic rules.
  glm::vec3 root_position = glm::vec3(0.0f);     ///< World-space root position.
  glm::quat root_rotation = kDefaultRootRotation;   ///< World-space root rotation (default: upward).
  float default_length = 1.0f;       ///< Default internode length.
  float default_thickness = 0.1f;    ///< Default internode thickness.
  bool auto_derive_on_change = true; ///< Re-derive when parameters change in editor.

  bool OnInspect(const std::shared_ptr<evo_engine::EditorLayer>& editor_layer) override;
  void Serialize(YAML::Emitter& out) const override;
  void Deserialize(const YAML::Node& in) override;
};

}  // namespace l_system_plugin
