#include "UniverseProfiler.hpp"

using namespace evo_engine;

namespace universe_package::universe_profiler {
namespace {
Items items{};

ProfilerItemDescriptor CpuItem(const std::string& id, const std::string& name) {
  ProfilerItemDescriptor descriptor{};
  descriptor.local_id = id;
  descriptor.display_name = name;
  descriptor.cpu = true;
  descriptor.cpu_category = "Universe";
  return descriptor;
}

ProfilerItemDescriptor GpuItem(const std::string& id, const std::string& name) {
  auto descriptor = CpuItem(id, name);
  descriptor.cpu = false;
  descriptor.gpu = true;
  descriptor.gpu_group = "Universe";
  descriptor.gpu_queue = ProfilerGpuQueue::Graphics;
  descriptor.gpu_contributes_to_frame_total = true;
  return descriptor;
}
}  // namespace

bool RegisterItems(PackageRegistrar& registrar) {
  items.compute = registrar.RegisterProfilerItem(GpuItem("Universe.ClusterCompute", "Star Cluster Compute"));
  items.inspection_copy = registrar.RegisterProfilerItem(GpuItem("Universe.InspectionCopy", "Star Inspection Copy"));
  items.readback = registrar.RegisterProfilerItem(CpuItem("Universe.Readback", "Star Cluster Readback"));
  items.render_registration =
      registrar.RegisterProfilerItem(CpuItem("Universe.RenderRegistration", "Star Render Registration"));
  items.forward_render = registrar.RegisterProfilerItem(GpuItem("Universe.ForwardRender", "Star Forward Render"));
  return items.compute && items.inspection_copy && items.readback && items.render_registration && items.forward_render;
}

const Items& GetItems() {
  return items;
}
}  // namespace universe_package::universe_profiler
