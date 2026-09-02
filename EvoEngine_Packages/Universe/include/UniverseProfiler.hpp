#pragma once

#include "PackageManager.hpp"

namespace universe_package::universe_profiler {
struct Items {
  evo_engine::ProfilerItemHandle compute{};
  evo_engine::ProfilerItemHandle parameters{};
  evo_engine::ProfilerItemHandle render_registration{};
  evo_engine::ProfilerItemHandle forward_render{};
  evo_engine::ProfilerItemHandle pick_intersection{}, pick_reduction{}, pick_input{}, pick_readback{};
  evo_engine::ProfilerItemHandle follow_cpu{}, hover_render{};
};

bool RegisterItems(evo_engine::PackageRegistrar& registrar);
const Items& GetItems();
}  // namespace universe_package::universe_profiler
