#pragma once

namespace eco_sys_lab_package {
inline bool ShouldRunDynamicStrandsStage(const bool enabled, const int pending_steps) {
  return enabled || pending_steps > 0;
}

inline void ConsumePendingDynamicStrandsStep(int& pending_steps) {
  if (pending_steps > 0)
    --pending_steps;
}

inline int FungusStepsInPhysicsSubstep(const int fungus_steps, const int physics_substeps, const int substep_index) {
  if (fungus_steps <= 0 || physics_substeps <= 0)
    return 0;
  return (substep_index + 1) * fungus_steps / physics_substeps - substep_index * fungus_steps / physics_substeps;
}
}  // namespace eco_sys_lab_package
