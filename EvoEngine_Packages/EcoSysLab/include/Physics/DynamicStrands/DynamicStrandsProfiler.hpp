#pragma once

#include "PackageManager.hpp"

namespace eco_sys_lab_package::dynamic_strands_profiler {

struct Items {
  evo_engine::ProfilerItemHandle simulation_cpu;
  evo_engine::ProfilerItemHandle simulation_gpu;
  evo_engine::ProfilerItemHandle interaction;
  evo_engine::ProfilerItemHandle physics;
  evo_engine::ProfilerItemHandle pre_step;
  evo_engine::ProfilerItemHandle dynamic_grouping;
  evo_engine::ProfilerItemHandle segment_collision;
  evo_engine::ProfilerItemHandle render_compute;
};

[[nodiscard]] const Items& GetItems();
[[nodiscard]] bool RegisterItems(evo_engine::PackageRegistrar& registrar);

}  // namespace eco_sys_lab_package::dynamic_strands_profiler
