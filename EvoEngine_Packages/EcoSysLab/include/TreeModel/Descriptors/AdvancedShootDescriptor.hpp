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
};
}  // namespace eco_sys_lab_package
