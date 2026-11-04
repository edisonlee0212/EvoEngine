#include "TreeClusterHierarchy.hpp"

#include <algorithm>
#include <cmath>
#include <glm/gtx/quaternion.hpp>
#include <limits>

using namespace eco_sys_lab_package;

std::optional<TreeClusterHierarchy> eco_sys_lab_package::BuildTreeClusterHierarchy(
    const DynamicTreeSkeleton::GpuRestState& rest) {
  TreeClusterHierarchy result;
  const auto count = static_cast<uint32_t>(rest.rods.size());
  result.candidates.resize(count);
  result.preorder.reserve(count);
  result.has_branch_orders = rest.branch_orders.size() == count;
  if (count == 0)
    return result;
  std::vector<uint32_t> incoming(rest.vertices.size(), UINT32_MAX);
  std::vector<std::vector<uint32_t>> children(count);
  for (uint32_t i = 0; i < count; ++i) {
    const auto& rod = rest.rods[i];
    if (rod.links.x >= incoming.size() || rod.links.y >= incoming.size() || rod.links.x == rod.links.y ||
        incoming[rod.links.y] != UINT32_MAX)
      return {};
    incoming[rod.links.y] = i;
  }
  std::vector<uint32_t> roots;
  for (uint32_t i = 0; i < count; ++i) {
    auto& candidate = result.candidates[i];
    candidate.parent = incoming[rest.rods[i].links.x];
    if (candidate.parent == UINT32_MAX) {
      roots.push_back(i);
      result.trunk_radius = std::max(result.trunk_radius, rest.rods[i].length_scale_mass_radius.w);
    } else {
      children[candidate.parent].push_back(i);
    }
  }
  std::vector<uint32_t> stack(roots.rbegin(), roots.rend());
  while (!stack.empty()) {
    const auto id = stack.back();
    stack.pop_back();
    result.candidates[id].begin = static_cast<uint32_t>(result.preorder.size());
    result.preorder.push_back(id);
    stack.insert(stack.end(), children[id].rbegin(), children[id].rend());
  }
  if (result.preorder.size() != count)
    return {};  // A rooted tree cannot contain disconnected cycles.

  GlobalTransform model;
  model.value = rest.initial_model;
  const auto inverse_rotation = glm::inverse(model.GetRotation());
  const auto origin = glm::vec3(rest.vertices[rest.rods[roots.front()].links.x].position_inverse_mass);
  std::vector<glm::vec3> minimum(count), maximum(count);
  for (auto it = result.preorder.rbegin(); it != result.preorder.rend(); ++it) {
    const auto id = *it;
    const auto& rod = rest.rods[id];
    auto& candidate = result.candidates[id];
    const auto a = inverse_rotation * (glm::vec3(rest.vertices[rod.links.x].position_inverse_mass) - origin);
    const auto b = inverse_rotation * (glm::vec3(rest.vertices[rod.links.y].position_inverse_mass) - origin);
    const float radius = rod.length_scale_mass_radius.w;
    if (!std::isfinite(glm::length(a)) || !std::isfinite(glm::length(b)) || !std::isfinite(radius) || radius < 0)
      return {};
    minimum[id] = glm::min(a, b) - radius;
    maximum[id] = glm::max(a, b) + radius;
    candidate.end = candidate.begin + 1;
    candidate.attachment_radius = radius;
    candidate.total_length = rod.length_scale_mass_radius.x;
    candidate.mass = rod.length_scale_mass_radius.z;
    candidate.branch_order = result.has_branch_orders ? rest.branch_orders[id] : UINT32_MAX;
    candidate.collapsible = rod.links.w == 0 && radius > 0 && candidate.total_length > 0 && candidate.mass > 0;
    for (const auto child : children[id]) {
      const auto& descendant = result.candidates[child];
      minimum[id] = glm::min(minimum[id], minimum[child]);
      maximum[id] = glm::max(maximum[id], maximum[child]);
      candidate.end += descendant.end - descendant.begin;
      candidate.total_length += descendant.total_length;
      candidate.mass += descendant.mass;
      candidate.collapsible = candidate.collapsible && descendant.collapsible;
    }
    candidate.extent = glm::length(maximum[id] - minimum[id]);
  }
  float bottom = std::numeric_limits<float>::max(), top = std::numeric_limits<float>::lowest();
  for (const auto root : roots) {
    bottom = std::min(bottom, minimum[root].y);
    top = std::max(top, maximum[root].y);
  }
  result.tree_height = top - bottom;
  return result;
}

TreeClusterSelection eco_sys_lab_package::SelectTreeClusters(const TreeClusterHierarchy& hierarchy,
                                                             const std::vector<TreeClusterRule>& rules) {
  TreeClusterSelection result;
  result.rod_elements.resize(hierarchy.candidates.size(), UINT32_MAX);
  for (uint32_t position = 0; position < hierarchy.preorder.size();) {
    const auto id = hierarchy.preorder[position];
    const auto& candidate = hierarchy.candidates[id];
    uint32_t policies = 0;
    if (candidate.collapsible && candidate.parent != UINT32_MAX)
      for (const auto& rule : rules) {
        if (candidate.end - candidate.begin < std::max(2u, rule.minimum_rods))
          continue;
        const bool order = hierarchy.has_branch_orders && candidate.branch_order >= rule.minimum_branch_order;
        const bool size = hierarchy.tree_height > 0 && hierarchy.trunk_radius > 0 &&
                          candidate.extent <= rule.maximum_extent_ratio * hierarchy.tree_height &&
                          candidate.attachment_radius <= rule.maximum_attachment_radius_ratio * hierarchy.trunk_radius;
        if ((rule.policy == TreeClusterPolicy::Size && size) ||
            (rule.policy == TreeClusterPolicy::BranchOrder && order) ||
            (rule.policy == TreeClusterPolicy::Combined && order && size))
          policies |= 1u << static_cast<uint32_t>(rule.policy);
      }
    const uint32_t end = policies ? candidate.end : position + 1;
    const auto element = static_cast<uint32_t>(result.elements.size());
    result.elements.push_back({id, candidate.parent == UINT32_MAX ? UINT32_MAX : result.rod_elements[candidate.parent],
                               position, end, policies});
    for (; position < end; ++position)
      result.rod_elements[hierarchy.preorder[position]] = element;
  }
  return result;
}
