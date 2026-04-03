#pragma once

#include <functional>
#include "LSystemGraph.hpp"

namespace l_system_plugin {

/**
 * @brief Top-down geometry propagation pass for LSystemGraph.
 *
 * Traverses the graph in BFS (sorted) order, computing each node's
 * `global_position` and `global_rotation` from its parent's transform plus
 * its own local rotation and length.
 *
 * This is the graph-primary equivalent of a turtle interpreter. It does NOT
 * interpret a linear string — it operates directly on graph node references.
 *
 * Convention: forward direction is -Z in local space; default root rotation
 * maps this to world +Y (upward growth).
 */
struct GeometryPass {
  /**
   * @brief Execute the geometry propagation pass.
   *
   * For each node in sorted (BFS root→leaf) order:
   *   - Root nodes keep their existing global_position and global_rotation.
   *   - Non-root nodes:
   *       child.global_position = parent.info.GetGlobalEndPosition()
   *       child.global_rotation = parent.global_rotation × child_local_rotation
   *
   * The local rotation is obtained from the user-supplied callback, which
   * can derive it from the node's module data (e.g., branching angles stored
   * in the module parameters).
   *
   * @tparam GraphData   Graph-wide data type.
   * @tparam FlowData    Per-flow data type.
   * @tparam ModuleData  Per-node module data type.
   * @param graph        The graph to update in-place.
   * @param local_rotation_fn Optional callback returning the local rotation for
   *                          a node relative to its parent. If nullptr, identity
   *                          rotation is used (child inherits parent direction).
   */
  template <typename GraphData, typename FlowData, typename ModuleData>
  static void Execute(
      LSystemGraph<GraphData, FlowData, ModuleData>& graph,
      std::function<glm::quat(const LGraphNode<ModuleData>& node,
                              const LGraphNode<ModuleData>& parent)>
          local_rotation_fn = nullptr);

  /**
   * @brief Execute geometry propagation with a fixed root transform.
   *
   * Sets the first root's position and rotation before propagating.
   *
   * @param root_position  Position of the root node.
   * @param root_rotation  Global rotation of the root node.
   */
  template <typename GraphData, typename FlowData, typename ModuleData>
  static void Execute(
      LSystemGraph<GraphData, FlowData, ModuleData>& graph,
      const glm::vec3& root_position,
      const glm::quat& root_rotation,
      std::function<glm::quat(const LGraphNode<ModuleData>& node,
                              const LGraphNode<ModuleData>& parent)>
          local_rotation_fn = nullptr);
};

// =============================================================================
// Template implementations
// =============================================================================

template <typename GraphData, typename FlowData, typename ModuleData>
void GeometryPass::Execute(
    LSystemGraph<GraphData, FlowData, ModuleData>& graph,
    std::function<glm::quat(const LGraphNode<ModuleData>& node,
                            const LGraphNode<ModuleData>& parent)>
        local_rotation_fn) {
  const auto& sorted = graph.PeekSortedNodeList();
  for (const auto handle : sorted) {
    auto& node = graph.RefNode(handle);
    const auto parent_handle = node.GetParentHandle();
    if (parent_handle == -1) {
      // Root node: position and rotation stay as-is (set externally or default).
      continue;
    }

    const auto& parent = graph.PeekNode(parent_handle);

    // Child starts where parent ends.
    node.info.global_position = parent.info.GetGlobalEndPosition();

    // Child rotation = parent rotation composed with local rotation.
    if (local_rotation_fn) {
      glm::quat local_rot = local_rotation_fn(node, parent);
      node.info.global_rotation = glm::normalize(parent.info.global_rotation * local_rot);
    } else {
      // No local rotation — inherit parent direction.
      node.info.global_rotation = glm::normalize(parent.info.global_rotation);
    }
  }
}

template <typename GraphData, typename FlowData, typename ModuleData>
void GeometryPass::Execute(
    LSystemGraph<GraphData, FlowData, ModuleData>& graph,
    const glm::vec3& root_position,
    const glm::quat& root_rotation,
    std::function<glm::quat(const LGraphNode<ModuleData>& node,
                            const LGraphNode<ModuleData>& parent)>
        local_rotation_fn) {
  // Set root transform(s).
  const auto& sorted = graph.PeekSortedNodeList();
  for (const auto handle : sorted) {
    auto& node = graph.RefNode(handle);
    if (node.GetParentHandle() == -1) {
      node.info.global_position = root_position;
      node.info.global_rotation = glm::normalize(root_rotation);
    }
  }

  // Run standard propagation.
  Execute(graph, local_rotation_fn);
}

}  // namespace l_system_plugin
