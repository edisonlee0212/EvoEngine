#pragma once

#include <array>
#include <optional>
#include "DynamicTreeSkeleton.hpp"

namespace eco_sys_lab_package {
enum class TreeClusterPolicy { Size, BranchOrder, Combined };
struct TreeClusterRule {
  TreeClusterPolicy policy = TreeClusterPolicy::Combined;
  uint32_t minimum_branch_order = 2, minimum_rods = 2;
  float maximum_extent_ratio = 0.15f, maximum_attachment_radius_ratio = 0.25f;
};

struct TreeClusterHierarchy {
  struct Candidate {
    uint32_t parent = UINT32_MAX, begin = 0, end = 0, branch_order = UINT32_MAX;
    float extent = 0, attachment_radius = 0, total_length = 0, mass = 0;
    bool collapsible = true;
  };
  // Candidate indices are original rod indices; each subtree is a contiguous preorder interval.
  std::vector<Candidate> candidates;
  std::vector<uint32_t> preorder;
  float tree_height = 0, trunk_radius = 0;
  bool has_branch_orders = false;
};
struct TreeClusterSelection {
  struct Element {
    uint32_t root_rod = 0, parent = UINT32_MAX, begin = 0, end = 0, policy_mask = 0;
    bool IsCluster() const {
      return end - begin > 1;
    }
  };
  std::vector<Element> elements;
  std::vector<uint32_t> rod_elements;
};

std::optional<TreeClusterHierarchy> BuildTreeClusterHierarchy(const DynamicTreeSkeleton::GpuRestState& rest);
// Rules are alternatives. Use Combined to require both branch order and size limits.
TreeClusterSelection SelectTreeClusters(const TreeClusterHierarchy& hierarchy,
                                        const std::vector<TreeClusterRule>& rules);
}  // namespace eco_sys_lab_package
