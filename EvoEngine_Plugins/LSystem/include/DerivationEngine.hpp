#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <random>
#include <vector>
#include "LSystemGraph.hpp"
#include "ProductionRule.hpp"

namespace l_system_plugin {

/**
 * @brief Two-phase derivation engine for L-system graphs.
 *
 * Phase 1 (topology): applies topology_rules that may change graph structure
 *   (extend, branch, remove nodes).
 * Phase 2 (growth): applies growth_rules that modify node data in-place
 *   (no topology changes).
 *
 * @tparam GraphData  Whole-graph data type.
 * @tparam FlowData   Per-flow data type.
 * @tparam ModuleData Per-node module data type (typically ModuleVariant<...>).
 */
template <typename GraphData, typename FlowData, typename ModuleData>
class DerivationEngine {
 public:
  using GraphType = LSystemGraph<GraphData, FlowData, ModuleData>;
  using RuleType = ProductionRule<GraphType, ModuleData>;
  using ContextType = RuleContext<GraphType>;
  using ResultType = ProductionResult<ModuleData>;

  std::vector<RuleType> topology_rules;  ///< Rules that may alter graph topology.
  std::vector<RuleType> growth_rules;    ///< Rules that only modify node data in-place.

  /**
   * @brief Perform one full derivation step (topology + growth).
   * @param graph The L-system graph to derive.
   * @param rng Random number generator for stochastic rules.
   * @return true if topology changed, false if only growth updates occurred.
   */
  bool Derive(GraphType& graph, std::mt19937& rng) const;

  /**
   * @brief Perform n derivation steps.
   * @param n Number of steps.
   * @param graph The L-system graph.
   * @param rng Random number generator.
   * @return Number of steps where topology changed.
   */
  int DeriveN(int n, GraphType& graph, std::mt19937& rng) const;

 private:
  /**
   * @brief Find the highest-priority matching rule for a node.
   * @return Pointer to the matching rule, or nullptr if none matches.
   */
  const RuleType* FindMatchingRule(const std::vector<RuleType>& rules,
                                   const ContextType& ctx) const;

  /**
   * @brief Find the highest-priority matching rule from a prefiltered candidate list.
   * @return Pointer to the matching rule, or nullptr if none matches.
   */
  const RuleType* FindMatchingRule(const std::vector<const RuleType*>& rules,
                                   const ContextType& ctx) const;

 public:
  /**
   * @brief Apply topology phase: collect all rule matches, then apply extensions/removals.
   * @return true if any topology change occurred.
   */
  bool ApplyTopologyRules(GraphType& graph, std::mt19937& rng) const;

  /**
   * @brief Apply growth phase: in-place data modifications only.
   */
  void ApplyGrowthRules(GraphType& graph, std::mt19937& rng) const;
};

// =============================================================================
// Template implementation
// =============================================================================

template <typename GraphData, typename FlowData, typename ModuleData>
bool DerivationEngine<GraphData, FlowData, ModuleData>::Derive(GraphType& graph, std::mt19937& rng) const {
  bool topology_changed = ApplyTopologyRules(graph, rng);
  ApplyGrowthRules(graph, rng);
  return topology_changed;
}

template <typename GraphData, typename FlowData, typename ModuleData>
int DerivationEngine<GraphData, FlowData, ModuleData>::DeriveN(int n, GraphType& graph, std::mt19937& rng) const {
  int topology_changes = 0;
  for (int i = 0; i < n; i++) {
    if (Derive(graph, rng))
      topology_changes++;
  }
  return topology_changes;
}

// Two-stage stochastic dispatch (P&L 1990 / vlab `cpfg` semantics).
//
// Stage 1: filter rules to the highest matching `priority`.
// Stage 2:
//   - bucket size 1                                          -> return it (no RNG)
//   - bucket size > 1 AND all probabilities default (1.0f)   -> return first
//                                                               match (legacy
//                                                               "first wins";
//                                                               consumes no RNG
//                                                               state)
//   - bucket size > 1 AND any rule opts in (probability!=1)  -> weighted random
//                                                               pick using
//                                                               ctx.rng with all
//                                                               bucket weights
//                                                               (vlab `SelectProd`).
//
// Negative or non-finite probabilities are clamped to 0.

namespace derivation_engine_detail {

template <typename RuleType>
const RuleType* PickFromBucket(const std::vector<const RuleType*>& bucket,
                                std::mt19937& rng) {
  if (bucket.empty()) return nullptr;
  if (bucket.size() == 1) return bucket.front();

  bool any_stochastic = false;
  float total = 0.0f;
  for (const auto* r : bucket) {
    const float w = (r && std::isfinite(r->probability) && r->probability > 0.0f)
                        ? r->probability : 0.0f;
    if (r && r->probability != 1.0f) any_stochastic = true;
    total += w;
  }

  if (!any_stochastic) {
    // Legacy deterministic path: return first (preserves prior behavior of
    // the original `if (!best || rule.priority > best->priority)` loop,
    // which never replaced a same-priority earlier match).
    return bucket.front();
  }

  if (total <= 0.0f) return bucket.front();

  std::uniform_real_distribution<float> uni(0.0f, 1.0f);
  float draw = total * uni(rng);
  for (const auto* r : bucket) {
    const float w = (r && std::isfinite(r->probability) && r->probability > 0.0f)
                        ? r->probability : 0.0f;
    draw -= w;
    if (draw <= 0.0f) return r;
  }
  return bucket.back();
}

}  // namespace derivation_engine_detail

template <typename GraphData, typename FlowData, typename ModuleData>
const typename DerivationEngine<GraphData, FlowData, ModuleData>::RuleType*
DerivationEngine<GraphData, FlowData, ModuleData>::FindMatchingRule(
    const std::vector<RuleType>& rules, const ContextType& ctx) const {
  // Stage 1: find max priority among matching rules.
  int max_prio = std::numeric_limits<int>::min();
  bool any = false;
  for (const auto& rule : rules) {
    if (rule.predecessor_symbol >= 0 && rule.predecessor_symbol != ctx.self.symbol_id)
      continue;
    if (rule.condition && !rule.condition(ctx))
      continue;
    if (!any || rule.priority > max_prio) {
      max_prio = rule.priority;
      any = true;
    }
  }
  if (!any) return nullptr;

  // Stage 2: collect bucket of matches at max_prio (stable order).
  std::vector<const RuleType*> bucket;
  bucket.reserve(rules.size());
  for (const auto& rule : rules) {
    if (rule.priority != max_prio) continue;
    if (rule.predecessor_symbol >= 0 && rule.predecessor_symbol != ctx.self.symbol_id)
      continue;
    if (rule.condition && !rule.condition(ctx))
      continue;
    bucket.push_back(&rule);
  }
  return derivation_engine_detail::PickFromBucket(bucket, ctx.rng);
}

template <typename GraphData, typename FlowData, typename ModuleData>
const typename DerivationEngine<GraphData, FlowData, ModuleData>::RuleType*
DerivationEngine<GraphData, FlowData, ModuleData>::FindMatchingRule(
    const std::vector<const RuleType*>& rules, const ContextType& ctx) const {
  // Stage 1: find max priority among matching rules.
  int max_prio = std::numeric_limits<int>::min();
  bool any = false;
  for (const auto* rule : rules) {
    if (!rule) continue;
    if (rule->predecessor_symbol >= 0 && rule->predecessor_symbol != ctx.self.symbol_id)
      continue;
    if (rule->condition && !rule->condition(ctx))
      continue;
    if (!any || rule->priority > max_prio) {
      max_prio = rule->priority;
      any = true;
    }
  }
  if (!any) return nullptr;

  // Stage 2: collect bucket of matches at max_prio (stable order).
  std::vector<const RuleType*> bucket;
  bucket.reserve(rules.size());
  for (const auto* rule : rules) {
    if (!rule || rule->priority != max_prio) continue;
    if (rule->predecessor_symbol >= 0 && rule->predecessor_symbol != ctx.self.symbol_id)
      continue;
    if (rule->condition && !rule->condition(ctx))
      continue;
    bucket.push_back(rule);
  }
  return derivation_engine_detail::PickFromBucket(bucket, ctx.rng);
}

template <typename GraphData, typename FlowData, typename ModuleData>
bool DerivationEngine<GraphData, FlowData, ModuleData>::ApplyTopologyRules(
    GraphType& graph, std::mt19937& rng) const {
  if (topology_rules.empty())
    return false;

  graph.SortLists();
  const auto& sorted = graph.PeekSortedNodeList();

  int max_symbol = -1;
  std::vector<const RuleType*> wildcard_rules;
  wildcard_rules.reserve(topology_rules.size());
  for (const auto& rule : topology_rules) {
    if (rule.predecessor_symbol >= 0) {
      max_symbol = std::max(max_symbol, rule.predecessor_symbol);
    } else {
      wildcard_rules.push_back(&rule);
    }
  }

  std::vector<std::vector<const RuleType*>> candidate_rules_by_symbol;
  if (max_symbol >= 0) {
    candidate_rules_by_symbol.resize(static_cast<size_t>(max_symbol) + 1);
    for (int symbol = 0; symbol <= max_symbol; symbol++) {
      auto& candidates = candidate_rules_by_symbol[static_cast<size_t>(symbol)];
      candidates.reserve(topology_rules.size());
      for (const auto& rule : topology_rules) {
        if (rule.predecessor_symbol < 0 || rule.predecessor_symbol == symbol) {
          candidates.push_back(&rule);
        }
      }
    }
  }

  // Phase 1a: Collect all matches before modifying topology.
  // We store (handle, result) pairs. We iterate in BFS order (root→leaves).
  struct PendingAction {
    LNodeHandle handle;
    ResultType result;
  };
  std::vector<PendingAction> pending;

  for (const auto& node_handle : sorted) {
    const auto& node = graph.PeekNode(node_handle);
    const auto* parent_ptr = node.GetParentHandle() >= 0 ? &graph.PeekNode(node.GetParentHandle()) : nullptr;
    ContextType ctx{node, parent_ptr, node.PeekChildHandles(), graph, rng, node_handle};

    const RuleType* rule = nullptr;
    if (node.symbol_id >= 0 && node.symbol_id <= max_symbol) {
      const auto& candidates = candidate_rules_by_symbol[static_cast<size_t>(node.symbol_id)];
      rule = FindMatchingRule(candidates, ctx);
    } else if (!wildcard_rules.empty()) {
      rule = FindMatchingRule(wildcard_rules, ctx);
    }
    if (!rule)
      continue;

    // Need mutable context for produce.
    ContextType mut_ctx{node, parent_ptr, node.PeekChildHandles(), graph, rng, node_handle};
    auto result = rule->produce(mut_ctx);

    // Only collect if the result actually changes topology.
    if (result.IsDeath() || result.IsExtension())
      pending.push_back({node_handle, std::move(result)});
    else if (!result.successors.empty()) {
      // Size 1 = in-place update (topology rule that decided not to change topology this step).
      auto& mut_node = graph.RefNode(node_handle);
      mut_node.symbol_id = result.successors[0].symbol_id;
      mut_node.data = result.successors[0].data;
    }
  }

  if (pending.empty())
    return false;

  // Phase 1b: Apply topology changes.
  // Process deaths and extensions. Extensions create new nodes; deaths remove nodes.
  // We process extensions first (they don't invalidate existing handles via swap-pop),
  // then deaths (which do invalidate via swap-pop, but we batch them).

  std::vector<LNodeHandle> nodes_to_remove;

  for (auto& [handle, result] : pending) {
    if (result.IsDeath()) {
      nodes_to_remove.push_back(handle);
    } else if (result.IsExtension()) {
      // First successor: overwrite the existing node in-place.
      auto& existing_node = graph.RefNode(handle);
      existing_node.symbol_id = result.successors[0].symbol_id;
      existing_node.data = result.successors[0].data;

      // Remaining successors: extend from this node.
      for (size_t s = 1; s < result.successors.size(); s++) {
        const auto& succ = result.successors[s];
        auto new_handle = graph.Extend(handle, succ.is_branch);
        auto& new_node = graph.RefNode(new_handle);
        new_node.symbol_id = succ.symbol_id;
        new_node.data = succ.data;
      }
    }
  }

  if (!nodes_to_remove.empty()) {
    graph.RemoveNodes(nodes_to_remove);
  }

  graph.SortLists();
  return true;
}

template <typename GraphData, typename FlowData, typename ModuleData>
void DerivationEngine<GraphData, FlowData, ModuleData>::ApplyGrowthRules(
    GraphType& graph, std::mt19937& rng) const {
  if (growth_rules.empty())
    return;

  graph.SortLists();
  const auto& sorted = graph.PeekSortedNodeList();

  int max_symbol = -1;
  std::vector<const RuleType*> wildcard_rules;
  wildcard_rules.reserve(growth_rules.size());
  for (const auto& rule : growth_rules) {
    if (rule.predecessor_symbol >= 0) {
      max_symbol = std::max(max_symbol, rule.predecessor_symbol);
    } else {
      wildcard_rules.push_back(&rule);
    }
  }

  std::vector<std::vector<const RuleType*>> candidate_rules_by_symbol;
  if (max_symbol >= 0) {
    candidate_rules_by_symbol.resize(static_cast<size_t>(max_symbol) + 1);
    for (int symbol = 0; symbol <= max_symbol; symbol++) {
      auto& candidates = candidate_rules_by_symbol[static_cast<size_t>(symbol)];
      candidates.reserve(growth_rules.size());
      for (const auto& rule : growth_rules) {
        if (rule.predecessor_symbol < 0 || rule.predecessor_symbol == symbol) {
          candidates.push_back(&rule);
        }
      }
    }
  }

  for (const auto& node_handle : sorted) {
    const auto& node = graph.PeekNode(node_handle);
    const auto* parent_ptr = node.GetParentHandle() >= 0 ? &graph.PeekNode(node.GetParentHandle()) : nullptr;
    ContextType ctx{node, parent_ptr, node.PeekChildHandles(), graph, rng, node_handle};

    const RuleType* rule = nullptr;
    if (node.symbol_id >= 0 && node.symbol_id <= max_symbol) {
      const auto& candidates = candidate_rules_by_symbol[static_cast<size_t>(node.symbol_id)];
      rule = FindMatchingRule(candidates, ctx);
    } else if (!wildcard_rules.empty()) {
      rule = FindMatchingRule(wildcard_rules, ctx);
    }

    if (!rule)
      continue;

    ContextType mut_ctx{node, parent_ptr, node.PeekChildHandles(), graph, rng, node_handle};
    auto result = rule->produce(mut_ctx);

    // Growth rules can only do in-place modifications (size == 1).
    if (!result.successors.empty()) {
      auto& mut_node = graph.RefNode(node_handle);
      mut_node.symbol_id = result.successors[0].symbol_id;
      mut_node.data = result.successors[0].data;
    }
  }
}

}  // namespace l_system_plugin
