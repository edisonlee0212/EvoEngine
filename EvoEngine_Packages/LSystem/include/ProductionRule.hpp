#pragma once

#include <functional>
#include <random>
#include <vector>
#include "LSystemGraph.hpp"

namespace l_system_package {

/**
 * @brief Read-only context passed to production rule conditions and producers.
 *
 * Provides access to the current node, its parent, children, the whole graph,
 * and a per-derivation RNG.
 *
 * @tparam GraphType The fully instantiated LSystemGraph type.
 */
template <typename GraphType>
struct RuleContext {
  using NodeType = typename std::remove_reference_t<decltype(std::declval<GraphType>().RefNode(0))>;
  using FlowType = typename std::remove_reference_t<decltype(std::declval<GraphType>().RefFlow(0))>;

  const NodeType& self;                           ///< Current node being evaluated.
  const NodeType* parent;                         ///< Parent node (nullptr if root).
  const std::vector<LNodeHandle>& child_handles;  ///< Handles of child nodes.
  const GraphType& graph;                         ///< Whole graph (read access).
  std::mt19937& rng;                              ///< Per-derivation random number generator.
  LNodeHandle self_handle;                        ///< Handle to self for topology operations.
};

/**
 * @brief A single successor module produced by a production rule.
 *
 * @tparam ModuleData The module data type (typically ModuleVariant<...>).
 */
template <typename ModuleData>
struct Successor {
  int symbol_id = -1;                ///< Symbol ID of the new module.
  ModuleData data = {};              ///< Data for the new module.
  bool is_branch = false;            ///< false = prolong (Extend(false)), true = branch (Extend(true)).
  LNodeHandle parent_override = -1;  ///< Optional explicit attachment node.
};

/**
 * @brief Result of applying a production rule to a node.
 *
 * - Empty successors -> death (node will be removed)
 * - 1 successor -> in-place modification (data overwritten, no topology change)
 * - 2+ successors -> extension (first = prolong same flow, rest = branches)
 *
 * @tparam ModuleData The module data type.
 */
template <typename ModuleData>
struct ProductionResult {
  std::vector<Successor<ModuleData>> successors;

  [[nodiscard]] bool IsDeath() const {
    return successors.empty();
  }
  [[nodiscard]] bool IsIdentity() const {
    return successors.size() == 1 && !successors[0].is_branch;
  }
  [[nodiscard]] bool IsExtension() const {
    return successors.size() > 1;
  }
};

/**
 * @brief A single production rule in the L-system.
 *
 * Matches nodes by symbol_id, evaluates a condition, and produces successors.
 * Rules are separated into topology rules (may change graph structure) and
 * growth rules (in-place parameter updates only).
 *
 * @tparam GraphType  The fully instantiated LSystemGraph type.
 * @tparam ModuleData The module data type stored in nodes.
 */
template <typename GraphType, typename ModuleData>
struct ProductionRule {
  int predecessor_symbol = -1;  ///< Which symbol_id this rule matches (-1 = match any).

  /**
   * @brief Condition function: returns true if this rule should fire for this node.
   *        If nullptr, the rule always fires when the symbol matches.
   */
  std::function<bool(const RuleContext<GraphType>&)> condition;

  /**
   * @brief Producer function: generates the successor modules.
   */
  std::function<ProductionResult<ModuleData>(RuleContext<GraphType>&)> produce;

  int priority = 0;  ///< Higher priority rules are checked first.

  /**
   * @brief Stochastic-selection weight (P&L 1990 / vlab `cpfg` style).
   *
   * When two or more rules share the highest matching priority for a given
   * node, the engine performs a weighted random pick using these weights.
   *
   * Dispatch contract: weighted selection only fires when at least one rule in
   * the matching same-priority bucket has a `probability` that differs from
   * the default `1.0f`. If every rule in the bucket keeps the default, the
   * engine falls back to the legacy "first match wins" path and consumes no
   * RNG state, keeping existing single-rule-per-priority L-systems unchanged.
   *
   * To opt in, register multiple rules at the same priority and set their
   * `probability` fields (any positive scalar; weights are normalized at
   * dispatch time).
   */
  float probability = 1.0f;
};

}  // namespace l_system_package
