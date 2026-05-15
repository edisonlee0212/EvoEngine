#pragma once
#include "ShootModel.hpp"
#include "SimulationSettings.hpp"
#include "TreeDescriptor.hpp"

namespace eco_sys_lab_package {
using namespace evo_engine;

class Climate;

/**
 * \class BasicPruningDescriptor
 * \brief Represents the parameters controlling procedural tree pruning.
 */
class BasicPruningDescriptor : public IPruningDescriptor {
 public:
  float low_branch_pruning = 0.0f;

  /**
   * \brief Flag indicating if trunk protection is enabled in pruning logic.
   */
  bool trunk_protection = false;

  /**
   * \brief The maximum allowed flow length for nutrient distribution.
   */
  int max_flow_length = 0;

  /**
   * \brief Factor determining pruning intensity due to lack of light.
   */
  float light_pruning_factor = 0.0f;

  /**
   * \brief Strength factor affecting branch endurance.
   */
  float branch_strength = 1.f;

  /**
   * \brief Effect of thickness on branch strength.
   */
  float branch_strength_thickness_factor = 3.f;

  /**
   * \brief Lighting threshold below which branches weaken.
   */
  float branch_strength_lighting_threshold = 0.f;

  /**
   * \brief Loss of branch strength over time due to insufficient light.
   */
  float branch_strength_lighting_loss = 0.f;

  /**
   * \brief Multiplier affecting how easily branches break.
   */
  float branch_breaking_multiplier = 1.f;

  /**
   * \brief Factor influencing the breaking probability of branches.
   */
  float branch_breaking_factor = 1.f;

  /**
   * \brief Prepares a ShootPruningController using current growth parameters.
   * \param simulation_settings Simulation settings.
   * \param shoot_pruning_controller The controller to configure.
   */
  void PrepareController(const SimulationSettings& simulation_settings,
                         ShootPruningController& shoot_pruning_controller) const override;

  /**
   * \brief Serializes the shoot descriptor to YAML format.
   * \param out The YAML emitter to write to.
   */
  void Serialize(YAML::Emitter& out) const override;

  /**
   * \brief Deserializes the shoot descriptor from YAML format.
   * \param in The YAML node containing serialized data.
   */
  void Deserialize(const YAML::Node& in) override;

  /**
   * \brief Inspects and modifies shoot descriptor parameters in the editor.
   * \param editor_layer The editor layer providing UI interaction.
   * \return True if the asset's content remains unmodified.
   */
  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) override;
};
}  // namespace eco_sys_lab_package