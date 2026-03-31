#pragma once

#include <functional>
#include <sstream>
#include <stack>
#include <string>
#include <vector>
#include "LSystemGraph.hpp"

namespace l_system_plugin {

/**
 * @brief A single module in an exported L-system string.
 */
struct ExportedModule {
  int symbol_id = -1;              ///< Module symbol ID.
  std::vector<float> params;       ///< Flattened float parameters.
  bool is_branch_start = false;    ///< True if this is a '[' bracket.
  bool is_branch_end = false;      ///< True if this is a ']' bracket.
};

/**
 * @brief A linearised L-system string exported from an LSystemGraph.
 *
 * Produced by DFS traversal of the graph, inserting '[' and ']' bracket
 * modules at branch points. This is a read-only export — modifying it
 * does not affect the source graph.
 */
struct ExportedString {
  std::vector<ExportedModule> modules;

  /**
   * @brief Convert to human-readable text representation.
   * @param symbol_names Optional function mapping symbol_id → name string.
   *                     If nullptr, uses the numeric ID.
   * @return String like "F(1.0) [ +(30.0) F(0.5) ] F(0.8)"
   */
  [[nodiscard]] std::string ToText(
      std::function<std::string(int)> symbol_names = nullptr) const;

  /**
   * @brief Convert to a flat token sequence for ML consumption.
   *
   * Format: each module becomes [symbol_id, param_count, param0, param1, ...].
   * Branch start = special token (-1), branch end = special token (-2).
   *
   * @return Vector of integer tokens (float params quantised to int via
   *         multiplier).
   */
  [[nodiscard]] std::vector<int> ToTokens(float quantisation_scale = 100.0f) const;

  /**
   * @brief Total number of non-bracket modules.
   */
  [[nodiscard]] int ModuleCount() const;
};

/**
 * @brief Export an LSystemGraph to a linear ExportedString via DFS traversal.
 *
 * Traverses the graph depth-first, emitting modules in order with '[' and ']'
 * bracket modules at branch points.
 *
 * @tparam GraphData   Graph-wide data type.
 * @tparam FlowData    Per-flow data type.
 * @tparam ModuleData  Per-node module data type.
 * @param graph        The graph to export.
 * @param param_extractor Function that extracts float parameters from a node's
 *                        ModuleData. If nullptr, no parameters are exported.
 * @return The exported string.
 */
template <typename GraphData, typename FlowData, typename ModuleData>
ExportedString ExportString(
    const LSystemGraph<GraphData, FlowData, ModuleData>& graph,
    std::function<std::vector<float>(int symbol_id, const ModuleData& data)> param_extractor = nullptr) {
  ExportedString result;

  const auto& base_nodes = graph.PeekRawNodes();
  if (base_nodes.empty())
    return result;

  // DFS traversal with bracket insertion.
  // We traverse from each base (root) node.
  struct StackEntry {
    LNodeHandle handle;
    int child_index;  // Which child we're about to visit next.
    bool emitted;     // Whether we've emitted this node's module.
  };

  // Find root nodes (nodes with no parent).
  std::vector<LNodeHandle> roots;
  for (const auto& node : base_nodes) {
    if (node.GetParentHandle() == -1)
      roots.push_back(node.GetHandle());
  }

  for (const auto& root_handle : roots) {
    std::stack<StackEntry> stack;
    stack.push({root_handle, 0, false});

    while (!stack.empty()) {
      auto& top = stack.top();
      const auto& node = graph.PeekNode(top.handle);

      if (!top.emitted) {
        // Emit this module.
        ExportedModule mod;
        mod.symbol_id = node.symbol_id;
        if (param_extractor) {
          mod.params = param_extractor(node.symbol_id, node.data);
        }
        result.modules.push_back(std::move(mod));
        top.emitted = true;
      }

      const auto& children = node.PeekChildHandles();
      if (top.child_index < static_cast<int>(children.size())) {
        auto child_handle = children[top.child_index];
        top.child_index++;

        // If this node has multiple children, non-first children are branches.
        if (children.size() > 1 && top.child_index > 1) {
          // Emit branch start.
          ExportedModule sb;
          sb.is_branch_start = true;
          sb.symbol_id = -1;
          result.modules.push_back(sb);
        }

        stack.push({child_handle, 0, false});
      } else {
        // All children visited. If we opened branches, close them.
        stack.pop();

        // If this node was a branch child (not the first child), emit branch end.
        if (!stack.empty()) {
          const auto& parent_entry = stack.top();
          const auto& parent_node = graph.PeekNode(parent_entry.handle);
          const auto& parent_children = parent_node.PeekChildHandles();
          // We just finished visiting a child. If parent has multiple children
          // and this wasn't the first child (apical continuation), close bracket.
          if (parent_children.size() > 1 && parent_entry.child_index > 1) {
            ExportedModule eb;
            eb.is_branch_end = true;
            eb.symbol_id = -2;
            result.modules.push_back(eb);
          }
        }
      }
    }
  }

  return result;
}

// =============================================================================
// Inline implementations for ExportedString
// =============================================================================

inline std::string ExportedString::ToText(std::function<std::string(int)> symbol_names) const {
  std::ostringstream oss;
  for (size_t i = 0; i < modules.size(); i++) {
    const auto& mod = modules[i];
    if (i > 0) oss << " ";
    if (mod.is_branch_start) {
      oss << "[";
    } else if (mod.is_branch_end) {
      oss << "]";
    } else {
      if (symbol_names) {
        oss << symbol_names(mod.symbol_id);
      } else {
        oss << "M" << mod.symbol_id;
      }
      if (!mod.params.empty()) {
        oss << "(";
        for (size_t p = 0; p < mod.params.size(); p++) {
          if (p > 0) oss << ",";
          oss << mod.params[p];
        }
        oss << ")";
      }
    }
  }
  return oss.str();
}

inline std::vector<int> ExportedString::ToTokens(float quantisation_scale) const {
  std::vector<int> tokens;
  for (const auto& mod : modules) {
    if (mod.is_branch_start) {
      tokens.push_back(-1);
    } else if (mod.is_branch_end) {
      tokens.push_back(-2);
    } else {
      tokens.push_back(mod.symbol_id);
      tokens.push_back(static_cast<int>(mod.params.size()));
      for (float p : mod.params) {
        tokens.push_back(static_cast<int>(p * quantisation_scale));
      }
    }
  }
  return tokens;
}

inline int ExportedString::ModuleCount() const {
  int count = 0;
  for (const auto& mod : modules) {
    if (!mod.is_branch_start && !mod.is_branch_end)
      count++;
  }
  return count;
}

}  // namespace l_system_plugin
