#pragma once
#include "TreeDescriptor.hpp"

namespace eco_sys_lab_plugin {
using namespace evo_engine;

namespace shoot_descriptor {
enum class NodeType {
  Input,
  Output,
};
}

/**
 * \class AdvancedShootDescriptor
 * \brief Represents the parameters controlling procedural tree growth.
 *
 * This class defines various properties affecting tree shoot growth, including
 * internode characteristics, bud behavior, pruning, and environmental influences.
 */
class AdvancedShootDescriptor : public IShootDescriptor {
 public:
  /**
   * \brief Prepares a ShootGrowthController using current growth parameters.
   * \param shoot_growth_controller The controller to configure.
   */
  void PrepareController(ShootGrowthController& shoot_growth_controller) const override;

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
}  // namespace eco_sys_lab_plugin