#pragma once

#include "PackageManager.hpp"

namespace universe_package::universe_profiler {
struct Items {
  evo_engine::ProfilerItemHandle compute{};
  evo_engine::ProfilerItemHandle inspection_copy{};
  evo_engine::ProfilerItemHandle readback{};
  evo_engine::ProfilerItemHandle render_registration{};
  evo_engine::ProfilerItemHandle forward_render{};
};

bool RegisterItems(evo_engine::PackageRegistrar& registrar);
const Items& GetItems();
}  // namespace universe_package::universe_profiler
