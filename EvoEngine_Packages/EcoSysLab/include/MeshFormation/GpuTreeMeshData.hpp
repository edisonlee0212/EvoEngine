#pragma once

#include <cstdint>
#include <glm/glm.hpp>
#include <vector>
#include "BasicBarkDescriptor.hpp"
#include "ShootModel.hpp"
#include "TreeMeshGenerator.hpp"

namespace eco_sys_lab_package {

// Std430 layout shared with EcoSysLab.GpuTreeMesh. Parent indices address this packed buffer.
struct GpuTreeNode {
  glm::vec4 position_thickness;
  glm::vec4 direction_length;
  glm::vec4 radial_axis_distance;
  glm::vec4 subdivisions;
  glm::vec4 bark;
  glm::vec4 bark_base;
  glm::uvec4 links;
  glm::vec4 foliage;
  glm::vec4 foliage_rotation;
  glm::uvec4 foliage_meta;
};
static_assert(sizeof(GpuTreeNode) == 160);

struct GpuTreeFoliageParams {
  glm::vec4 size_count;
  glm::vec4 filters;
  glm::vec4 distributions;
  glm::vec4 tropism;
  glm::uvec4 flags;
};
static_assert(sizeof(GpuTreeFoliageParams) == 80);

struct GpuTreeInstanceIndices {
  uint32_t branch = 0;
  uint32_t foliage = 0;
};

template <typename SkeletonData, typename FlowData, typename NodeData>
void AppendGpuTreeNodes(const Skeleton<SkeletonData, FlowData, NodeData>& skeleton,
                        const TreeMeshGeneratorSettings& settings, const BasicBarkDescriptor* bark, uint32_t tree_index,
                        bool shoot, std::vector<GpuTreeNode>& output,
                        const std::vector<ShootModel::PhysicsPose>* physics_pose = nullptr) {
  const auto& sorted = skeleton.PeekSortedNodeList();
  const uint32_t base = static_cast<uint32_t>(output.size());
  std::vector<uint32_t> packed_indices(skeleton.PeekRawNodes().size(), UINT32_MAX);
  for (uint32_t i = 0; i < sorted.size(); ++i)
    packed_indices[sorted[i]] = base + i;
  for (const auto handle : sorted) {
    const auto& node = skeleton.PeekNode(handle);
    const auto& info = node.info;
    const auto* pose = physics_pose && handle < physics_pose->size() ? &(*physics_pose)[handle] : nullptr;
    const auto parent = node.GetParentHandle();
    const float thickness = (settings.override_radius ? settings.radius : info.thickness) * settings.radius_multiplier;
    GpuTreeNode packed{};
    packed.position_thickness = glm::vec4(pose ? pose->position : info.global_position, glm::max(0.0f, thickness));
    packed.direction_length = glm::vec4(pose ? pose->rotation * glm::vec3(0, 0, -1) : info.GetGlobalDirection(),
                                        glm::max(0.0f, pose ? pose->length : info.length));
    packed.radial_axis_distance = glm::vec4(
        (pose ? pose->regulated_rotation : info.regulated_global_rotation) * glm::vec3(1, 0, 0), info.root_distance);
    packed.subdivisions = {
        glm::max(0.0001f, settings.x_subdivision),
        glm::max(0.0001f, info.thickness >= settings.trunk_thickness ? settings.trunk_y_subdivision
                                                                     : settings.branch_y_subdivision),
        settings.smoothness ? settings.base_control_point_ratio : 0.0f,
        settings.smoothness ? settings.branch_control_point_ratio : 0.0f};
    if (bark) {
      packed.bark = {bark->bark_x_frequency, bark->bark_y_frequency, bark->bark_depth, bark->base_frequency};
      packed.bark_base = {bark->base_max_distance, bark->base_distance_decrease_factor, bark->base_depth, 0.0f};
    }
    packed.links = {parent < 0 ? UINT32_MAX : packed_indices[parent], tree_index, UINT32_MAX, UINT32_MAX};
    packed.foliage = {info.leaves, info.end_distance, info.thickness, 0.0f};
    const auto rotation = pose ? pose->rotation : info.global_rotation;
    packed.foliage_rotation = {rotation.x, rotation.y, rotation.z, rotation.w};
    packed.foliage_meta = {static_cast<uint32_t>(handle), shoot ? 1u : 0u, 0u, 0u};
    output.emplace_back(packed);
  }
}

}  // namespace eco_sys_lab_package
