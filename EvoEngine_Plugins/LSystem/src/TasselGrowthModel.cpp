#include "TasselGrowthModel.hpp"
#include "MaizeTasselDescriptor.hpp"
#include <Times.hpp>
#include <algorithm>
#include <cmath>
#include <functional>
#include <glm/gtc/constants.hpp>

using namespace l_system_plugin;

namespace {
// [deprecated] Legacy dual-axis spike reparent helpers are retained for backward compatibility
// with historical graphs/rules that still instantiate TasselSpikeApex.
bool IsSpikeAxisNode(const TasselNode& node) {
  if (node.data.Is<TasselSpikeApex>()) {
    return true;
  }
  if (node.data.Is<TasselInternode>()) {
    return node.data.Get<TasselInternode>().is_spike;
  }
  return false;
}

LNodeHandle FindMainSpikeRootHandle(const TasselGraph& graph) {
  for (const auto handle : graph.PeekSortedNodeList()) {
    const auto& node = graph.PeekNode(handle);
    if (!IsSpikeAxisNode(node)) {
      continue;
    }

    const auto parent_handle = node.GetParentHandle();
    if (parent_handle < 0) {
      return handle;
    }

    const auto& parent = graph.PeekNode(parent_handle);
    if (!IsSpikeAxisNode(parent)) {
      return handle;
    }
  }
  return -1;
}

LNodeHandle FindPeduncleTipInternodeHandle(const TasselGraph& graph) {
  for (const auto handle : graph.PeekSortedNodeList()) {
    const auto& node = graph.PeekNode(handle);
    if (!node.data.Is<TasselApex>()) {
      continue;
    }

    const auto& apex = node.data.Get<TasselApex>();
    if (apex.order != 0) {
      continue;
    }

    const auto parent_handle = node.GetParentHandle();
    if (parent_handle < 0) {
      continue;
    }

    const auto& parent = graph.PeekNode(parent_handle);
    if (!parent.data.Is<TasselInternode>()) {
      continue;
    }

    const auto& parent_internode = parent.data.Get<TasselInternode>();
    if (parent_internode.order == 0 && !parent_internode.is_spike) {
      return parent_handle;
    }
  }
  return -1;
}

bool AttachMainSpikeToPeduncleTip(TasselGraph& graph, const bool assume_sorted = false) {
  if (!assume_sorted) {
    graph.SortLists();
  }

  const auto spike_root_handle = FindMainSpikeRootHandle(graph);
  if (spike_root_handle < 0) {
    return false;
  }

  const auto peduncle_tip_handle = FindPeduncleTipInternodeHandle(graph);
  if (peduncle_tip_handle < 0 || peduncle_tip_handle == spike_root_handle) {
    return false;
  }

  const auto& spike_root = graph.PeekNode(spike_root_handle);
  if (spike_root.GetParentHandle() == peduncle_tip_handle) {
    return false;
  }

  graph.ReparentNode(spike_root_handle, peduncle_tip_handle);
  return true;
}
}  // namespace

// ---------------------------------------------------------------------------
// Initialize
// ---------------------------------------------------------------------------

void TasselGrowthModel::Initialize(const MaizeTasselDescriptor& descriptor, unsigned int seed,
                                   const glm::vec3& root_position,
                                   const glm::quat& root_rotation) {
  Reset();

  rng_.seed(seed);
  root_position_ = root_position;
  root_rotation_ = root_rotation;

  // Sample all distributions from the descriptor.
  sampled = descriptor.Sample(rng_);

  accumulated_gdd = 0.0f;
  gdd_per_growth_step = 1.0f;
  sampled.gdd_step = gdd_per_growth_step;
  sampled.base_temperature = std::clamp(sampled.base_temperature, 0.0f, 30.0f);
  sampled.plastochron_gdd = std::max(1.0f, sampled.plastochron_gdd);
  sampled.anthesis_gdd = std::max(0.0f, sampled.anthesis_gdd);
  sampled.maturity_gdd = std::max(sampled.anthesis_gdd + 1.0f, sampled.maturity_gdd);

  // Set up a single continuous main-axis apex.
  const int total_main_nodes = std::max(0, sampled.branch_node_count + sampled.spike_node_count);
  graph = TasselGraph(1);
  auto& root = graph.RefNode(0);
  root.symbol_id = TasselSymbol::Apex;
  TasselApex apex;
  apex.vigor = static_cast<float>(total_main_nodes);
  apex.age = 0;
  apex.order = 0;
  apex.age_gdd = ComputeInitiationPlastochronGdd(sampled, apex.order, false);
  apex.node_random = SampleUnit01(rng_);
  apex.phyllotaxis_phase = std::fmod(apex.node_random * 360.0f, 360.0f);
  root.data.Set<TasselApex>(apex);
  root.info.global_position = root_position;
  root.info.global_rotation = root_rotation;
  root.info.length = 0.0f;
  const float root_branch_thickness = sampled.branch_internode_thickness.mean.GetValue(0.0f);
  const float root_spike_thickness = sampled.spike_internode_thickness.mean.GetValue(0.0f);
  const float root_spike_weight = sampled.branch_node_count <= 0 ? 1.0f : 0.0f;
  root.info.thickness = std::max(0.01f, root_branch_thickness +
                                           (root_spike_thickness - root_branch_thickness) * root_spike_weight);

  // [deprecated] Legacy dual-axis SpikeApex remains supported by topology rules for backward compatibility,
  // but default initialization uses a single connected order-0 axis.

  // Build the derivation engine with sampled thermal timing parameters.
  engine_ = TasselEngine();
  engine_.topology_rules = CreateTasselTopologyRules(sampled);
  engine_.growth_rules = CreateTasselGrowthRules(sampled);

  // Topology steps needed: continuous main axis + lateral expansion.
  max_topology_steps_ = total_main_nodes * 3 + 10;

  initialized_ = true;
  topology_complete_ = false;
}

// ---------------------------------------------------------------------------
// Derive full topology
// ---------------------------------------------------------------------------

void TasselGrowthModel::DeriveTopology() {
  if (!initialized_)
    return;

  // Keep deriving until no more topology changes occur.
  for (int i = 0; i < max_topology_steps_; i++) {
    bool changed = engine_.ApplyTopologyRules(graph, rng_);
    if (!changed)
      break;
  }

  // Assign lengths/thicknesses from module data before geometry pass.
  graph.SortLists();
  for (const auto handle : graph.PeekSortedNodeList()) {
    auto& node = graph.RefNode(handle);
    if (node.data.Is<TasselInternode>()) {
      const auto& internode = node.data.Get<TasselInternode>();
      node.info.length = internode.length;
      node.info.thickness = internode.thickness;
    } else {
      node.info.length = 0.0f;
      node.info.thickness = 0.0f;
    }
  }

  PropagateGeometry();
  topology_complete_ = true;
}

// ---------------------------------------------------------------------------
// Growth step
// ---------------------------------------------------------------------------

void TasselGrowthModel::GrowStep() {
  if (!initialized_)
    return;

  const double step_start = evo_engine::Times::Now();
  last_growth_step_profile = {};
  bool topology_phase_ran = false;
  bool topology_changed = false;

  // Age and interpolate existing modules first.
  double phase_start = evo_engine::Times::Now();
  engine_.ApplyGrowthRules(graph, rng_);
  last_growth_step_profile.apply_growth_rules_seconds = evo_engine::Times::Now() - phase_start;

  // Once topology is complete, growth rules can no longer create topology symbols.
  if (!topology_complete_) {
    topology_phase_ran = true;
    phase_start = evo_engine::Times::Now();
    topology_changed = engine_.ApplyTopologyRules(graph, rng_);
    last_growth_step_profile.apply_topology_rules_seconds = evo_engine::Times::Now() - phase_start;

    phase_start = evo_engine::Times::Now();
    if (topology_changed) {
      graph.SortLists();
    }
    last_growth_step_profile.sort_lists_seconds += evo_engine::Times::Now() - phase_start;
  }

  accumulated_gdd += gdd_per_growth_step;

  const auto& sorted_nodes = graph.PeekSortedNodeList();
  phase_start = evo_engine::Times::Now();
  for (const auto handle : sorted_nodes) {
    auto& node = graph.RefNode(handle);
    if (node.data.Is<TasselInternode>()) {
      const auto& internode = node.data.Get<TasselInternode>();
      node.info.length = internode.length;
      node.info.thickness = internode.thickness;
    } else {
      node.info.length = 0.0f;
      node.info.thickness = 0.0f;
    }
  }
  last_growth_step_profile.update_node_info_seconds = evo_engine::Times::Now() - phase_start;

  phase_start = evo_engine::Times::Now();
  PropagateGeometry();
  last_growth_step_profile.propagate_geometry_seconds = evo_engine::Times::Now() - phase_start;

  if (topology_phase_ran) {
    // Topology is complete when no pending developmental symbols remain.
    phase_start = evo_engine::Times::Now();
    topology_complete_ = true;
    for (const auto handle : sorted_nodes) {
      const auto& node = graph.PeekNode(handle);
      if (node.data.Is<TasselApex>() || node.data.Is<TasselLateral>() || node.data.Is<TasselSpikeApex>()) {
        topology_complete_ = false;
        break;
      }
    }
    last_growth_step_profile.topology_scan_seconds = evo_engine::Times::Now() - phase_start;
  }

  last_growth_step_profile.total_seconds = evo_engine::Times::Now() - step_start;
}

// ---------------------------------------------------------------------------
// Grow to target GDD
// ---------------------------------------------------------------------------

void TasselGrowthModel::GrowToGDD(float target_gdd, const uint32_t max_growth_steps) {
  last_growth_steps = 0;
  last_grow_to_gdd_profile = {};
  while (accumulated_gdd < target_gdd &&
         (max_growth_steps == 0 || last_growth_steps < max_growth_steps)) {
    GrowStep();
    last_growth_steps++;

    last_grow_to_gdd_profile.total_seconds += last_growth_step_profile.total_seconds;
    last_grow_to_gdd_profile.apply_growth_rules_seconds +=
        last_growth_step_profile.apply_growth_rules_seconds;
    last_grow_to_gdd_profile.apply_topology_rules_seconds +=
        last_growth_step_profile.apply_topology_rules_seconds;
    last_grow_to_gdd_profile.sort_lists_seconds +=
        last_growth_step_profile.sort_lists_seconds;
    last_grow_to_gdd_profile.update_node_info_seconds +=
        last_growth_step_profile.update_node_info_seconds;
    last_grow_to_gdd_profile.propagate_geometry_seconds +=
        last_growth_step_profile.propagate_geometry_seconds;
    last_grow_to_gdd_profile.topology_scan_seconds +=
        last_growth_step_profile.topology_scan_seconds;
  }
  if (last_growth_steps == 0) {
    last_growth_step_profile = {};
    last_grow_to_gdd_profile = {};
  }
}

// ---------------------------------------------------------------------------
// Propagate geometry
// ---------------------------------------------------------------------------

void TasselGrowthModel::PropagateGeometry() {
  GeometryPass::Execute(graph, root_position_, root_rotation_,
      std::function<glm::quat(const LGraphNode<TasselModuleData>&,
                              const LGraphNode<TasselModuleData>&)>(
      [this](const LGraphNode<TasselModuleData>& node,
             const LGraphNode<TasselModuleData>& parent) -> glm::quat {
        if (node.data.Is<TasselInternode>()) {
          const auto& internode = node.data.Get<TasselInternode>();

          auto finite_quat = [](const glm::quat& q) {
            return std::isfinite(q.x) && std::isfinite(q.y) && std::isfinite(q.z) && std::isfinite(q.w);
          };

          glm::vec3 bend_axis = internode.bend_axis_local;
            if (!std::isfinite(bend_axis.x) || !std::isfinite(bend_axis.y) ||
              !std::isfinite(bend_axis.z) || glm::dot(bend_axis, bend_axis) < 1e-8f) {
            bend_axis = glm::vec3(1.0f, 0.0f, 0.0f);
          } else {
            bend_axis = glm::normalize(bend_axis);
          }

          float safe_curvature = std::clamp(internode.curvature, -89.0f, 89.0f);
          if (!std::isfinite(safe_curvature)) {
            safe_curvature = 0.0f;
          }

          glm::quat roll = glm::angleAxis(glm::radians(internode.roll_angle), glm::vec3(0, 0, -1));
          glm::quat pitch = glm::angleAxis(glm::radians(internode.branch_angle), glm::vec3(1, 0, 0));
          // Apply tropism curvature around a per-internode local axis.
          glm::quat bend = glm::angleAxis(glm::radians(safe_curvature), bend_axis);
          glm::quat local = glm::normalize(roll * pitch * bend);
          if (!finite_quat(local)) {
            return glm::quat(1, 0, 0, 0);
          }
          return local;
        }
        if (node.data.Is<TasselLateral>()) {
          const auto& lateral = node.data.Get<TasselLateral>();
          glm::quat pitch = glm::angleAxis(glm::radians(lateral.insertion_angle), glm::vec3(1, 0, 0));
          return pitch;
        }
        if (node.data.Is<TasselSpikeletPair>()) {
          const auto& pair = node.data.Get<TasselSpikeletPair>();
          const glm::quat roll = glm::angleAxis(glm::radians(pair.phyllotaxis_azimuth), glm::vec3(0, 0, -1));
          const glm::quat pitch = glm::angleAxis(glm::radians(pair.proximal_outward_angle), glm::vec3(1, 0, 0));
          return glm::normalize(roll * pitch);
        }
        if (node.data.Is<TasselSpikeApex>()) {
          return glm::quat(1, 0, 0, 0);
        }
        return glm::quat(1, 0, 0, 0);
      }));
}

// ---------------------------------------------------------------------------
// Reset
// ---------------------------------------------------------------------------

void TasselGrowthModel::Reset() {
  graph = TasselGraph(1);
  engine_ = TasselEngine();
  accumulated_gdd = 0.0f;
  initialized_ = false;
  topology_complete_ = false;
  max_topology_steps_ = 0;
  sampled = SampledTasselParams();
}
