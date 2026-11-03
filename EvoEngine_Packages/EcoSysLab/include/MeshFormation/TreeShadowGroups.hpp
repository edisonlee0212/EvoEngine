#pragma once

#include "GpuTreeMeshData.hpp"

namespace eco_sys_lab_package {
struct TreeShadowGroups {
  std::vector<uint32_t> node_groups, members;
  // Member offset, count, representative node, reserved.
  std::vector<glm::uvec4> groups;
};

inline TreeShadowGroups BuildTreeShadowGroups(const std::vector<GpuTreeNode>& nodes, const std::vector<bool>& apical) {
  TreeShadowGroups result;
  result.node_groups.resize(nodes.size(), UINT32_MAX);
  std::vector<std::vector<uint32_t>> members;
  for (uint32_t i = 0; i < apical.size(); ++i) {
    const auto parent = nodes[i].links.x;
    if (parent == UINT32_MAX)
      continue;
    auto group = result.node_groups[parent];
    if (group == UINT32_MAX && !apical[i]) {
      group = static_cast<uint32_t>(members.size());
      members.emplace_back();
    }
    result.node_groups[i] = group;
    if (group != UINT32_MAX)
      members[group].push_back(i);
  }
  for (const auto& group : members) {
    result.groups.emplace_back(result.members.size(), group.size(), group.front(), 0);
    result.members.insert(result.members.end(), group.begin(), group.end());
  }
  return result;
}
}  // namespace eco_sys_lab_package
