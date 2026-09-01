#include "DynamicStrandsProfiler.hpp"

using namespace eco_sys_lab_package;
using namespace evo_engine;

namespace {
dynamic_strands_profiler::Items items;

ProfilerItemDescriptor CpuItem(std::string id, std::string name) {
  ProfilerItemDescriptor descriptor;
  descriptor.local_id = std::move(id);
  descriptor.display_name = std::move(name);
  descriptor.cpu_category = "EcoSysLab";
  descriptor.cpu = true;
  return descriptor;
}

ProfilerItemDescriptor GpuItem(std::string id, std::string name, const bool additive = true) {
  ProfilerItemDescriptor descriptor;
  descriptor.local_id = std::move(id);
  descriptor.display_name = std::move(name);
  descriptor.gpu_group = "DynamicStrands";
  descriptor.gpu_queue = ProfilerGpuQueue::Compute;
  descriptor.gpu = true;
  descriptor.gpu_contributes_to_frame_total = additive;
  return descriptor;
}

ProfilerItemDescriptor CpuGpuItem(std::string id, std::string name, const bool additive) {
  auto descriptor = GpuItem(std::move(id), std::move(name), additive);
  descriptor.cpu = true;
  descriptor.cpu_category = "EcoSysLab";
  return descriptor;
}
}  // namespace

const dynamic_strands_profiler::Items& dynamic_strands_profiler::GetItems() {
  return items;
}

bool dynamic_strands_profiler::RegisterItems(PackageRegistrar& registrar) {
  items.simulation_cpu =
      registrar.RegisterProfilerItem(CpuItem("DynamicStrands.SimulationCpu", "DynamicStrandSimulation"));
  items.simulation_gpu =
      registrar.RegisterProfilerItem(GpuItem("DynamicStrands.SimulationGpu", "DynamicStrands Simulation", false));
  items.interaction =
      registrar.RegisterProfilerItem(CpuGpuItem("DynamicStrands.Interaction", "Interaction / Operators", true));
  items.physics = registrar.RegisterProfilerItem(CpuGpuItem("DynamicStrands.Physics", "Physics", true));
  items.pre_step = registrar.RegisterProfilerItem(GpuItem("DynamicStrands.PreStep", "Pre-step"));
  items.dynamic_grouping =
      registrar.RegisterProfilerItem(GpuItem("DynamicStrands.DynamicGrouping", "Dynamic Grouping"));
  items.segment_collision =
      registrar.RegisterProfilerItem(GpuItem("DynamicStrands.SegmentCollision", "Segment Collision"));
  items.bundle_legacy = registrar.RegisterProfilerItem(GpuItem("DynamicStrands.Bundle.Legacy", "Bundle: Legacy"));
  items.bundle_pair_solve =
      registrar.RegisterProfilerItem(GpuItem("DynamicStrands.Bundle.PairSolveGather", "Bundle: Pair Solve / Gather"));
  items.bundle_topology_rebuild =
      registrar.RegisterProfilerItem(GpuItem("DynamicStrands.Bundle.TopologyRebuild", "Bundle: Topology Rebuild"));
  items.bundle_slice_fit_apply =
      registrar.RegisterProfilerItem(GpuItem("DynamicStrands.Bundle.SliceFitApply", "Bundle: Slice Fit / Apply"));
  items.bundle_coarse_edge_solve =
      registrar.RegisterProfilerItem(GpuItem("DynamicStrands.Bundle.CoarseEdgeSolve", "Bundle: Coarse Edge Solve"));
  items.render_compute =
      registrar.RegisterProfilerItem(CpuGpuItem("DynamicStrands.RenderCompute", "Render Compute", true));
  return items.simulation_cpu && items.simulation_gpu && items.interaction && items.physics && items.pre_step &&
         items.dynamic_grouping && items.segment_collision && items.bundle_legacy && items.bundle_pair_solve &&
         items.bundle_topology_rebuild && items.bundle_slice_fit_apply && items.bundle_coarse_edge_solve &&
         items.render_compute;
}
