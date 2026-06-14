#pragma once
#include "TreeDescriptor.hpp"

namespace eco_sys_lab_package {
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
   * \brief Inspects and modifies shoot descriptor parameters in the editor.
   * \param editor_layer The editor layer providing UI interaction.
   * \return True if the asset's content remains unmodified.
   */
  bool DrawGui(const std::shared_ptr<EditorLayer>& editor_layer);
};
}  // namespace eco_sys_lab_package
