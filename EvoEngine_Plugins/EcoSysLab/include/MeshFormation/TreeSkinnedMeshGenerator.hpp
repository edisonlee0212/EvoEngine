#pragma once

#include "TreeMeshGenerator.hpp"
#include "Vertex.hpp"
using namespace evo_engine;

namespace eco_sys_lab {

template <typename SkeletonData, typename FlowData, typename NodeData>
class CylindricalSkinnedMeshGenerator {
 public:
  static void GenerateBones(const Skeleton<SkeletonData, FlowData, NodeData>& skeleton,
                            const std::vector<SkeletonFlowHandle>& flow_handles,
                            std::vector<glm::mat4>& offset_matrices,
                            std::unordered_map<SkeletonFlowHandle, int>& flow_bone_id_map);
  static void Generate(
      const Skeleton<SkeletonData, FlowData, NodeData>& skeleton, std::vector<SkinnedVertex>& skinned_vertices,
      std::vector<unsigned int>& indices, std::vector<glm::mat4>& offset_matrices,
      const TreeMeshGeneratorSettings& settings,
      const std::function<void(glm::vec3& vertex_position, const glm::vec3& direction, float x_factor,
                               float distance_to_root)>& vertex_position_modifier,
      const std::function<void(glm::vec2& tex_coords, float x_factor, float distance_to_root)>& tex_coords_modifier);
};

template <typename SkeletonData, typename FlowData, typename NodeData>
void CylindricalSkinnedMeshGenerator<SkeletonData, FlowData, NodeData>::GenerateBones(
    const Skeleton<SkeletonData, FlowData, NodeData>& skeleton, const std::vector<SkeletonFlowHandle>& flow_handles,
    std::vector<glm::mat4>& offset_matrices, std::unordered_map<SkeletonFlowHandle, int>& flow_bone_id_map) {
  flow_bone_id_map.clear();
  int current_bone_index = 0;
  for (const auto& flow_handle : flow_handles) {
    flow_bone_id_map[flow_handle] = current_bone_index;
    current_bone_index++;
  }

  offset_matrices.resize(current_bone_index + 1);
  for (const auto& [flowHandle, matrixIndex] : flow_bone_id_map) {
    const auto& flow = skeleton.PeekFlow(flowHandle);
    offset_matrices[matrixIndex] = glm::inverse(glm::translate(flow.info.global_start_position) *
                                               glm::mat4_cast(flow.info.global_start_rotation));
  }
}

template <typename SkeletonData, typename FlowData, typename NodeData>
void CylindricalSkinnedMeshGenerator<SkeletonData, FlowData, NodeData>::Generate(
    const Skeleton<SkeletonData, FlowData, NodeData>& skeleton, std::vector<SkinnedVertex>& skinned_vertices,
    std::vector<unsigned int>& indices, std::vector<glm::mat4>& offset_matrices,
    const TreeMeshGeneratorSettings& settings,
    const std::function<void(glm::vec3& vertex_position, const glm::vec3& direction, float x_factor,
                             float distance_to_root)>& vertex_position_modifier,
    const std::function<void(glm::vec2& tex_coords, float x_factor, float distance_to_root)>& tex_coords_modifier) {
  const auto& sorted_flow_list = skeleton.PeekSortedFlowList();
  const auto& sorted_internode_list = skeleton.PeekSortedNodeList();

  std::vector<std::vector<RingSegment>> rings_list;
  std::vector<bool> main_child_status;

  std::unordered_map<SkeletonNodeHandle, int> steps{};
  rings_list.resize(sorted_internode_list.size());
  main_child_status.resize(sorted_internode_list.size());
  std::vector<std::shared_future<void>> results;
  std::vector<std::vector<std::pair<SkeletonNodeHandle, int>>> temp_steps{};
  temp_steps.resize(Jobs::GetWorkerSize());

  Jobs::RunParallelFor(sorted_internode_list.size(), [&](unsigned internode_index, unsigned thread_index) {
    auto internode_handle = sorted_internode_list[internode_index];
    const auto& internode = skeleton.PeekNode(internode_handle);
    const auto& internode_info = internode.info;

    auto& rings = rings_list[internode_index];
    rings.clear();

    glm::vec3 direction_start = internode_info.regulated_global_rotation * glm::vec3(0, 0, -1);
    glm::vec3 direction_end = direction_start;
    float root_distance_start = internode_info.root_distance;
    float root_distance_end = root_distance_start;

    glm::vec3 position_start = internode_info.global_position;
    glm::vec3 position_end;
    position_end = position_start + internode_info.length *
                  (settings.smoothness ? 1.0f - settings.base_control_point_ratio : 1.0f) *
                  internode_info.GetGlobalDirection();
    float thickness_start = internode_info.thickness;
    float thickness_end = internode_info.thickness;

    if (internode.GetParentHandle() != -1) {
      const auto& parent_internode = skeleton.PeekNode(internode.GetParentHandle());
      thickness_start = parent_internode.info.thickness;
      direction_start = parent_internode.info.regulated_global_rotation * glm::vec3(0, 0, -1);
      position_start =
          parent_internode.info.global_position +
          (parent_internode.info.length * (settings.smoothness ? 1.0f - settings.base_control_point_ratio : 1.0f)) *
              parent_internode.info.GetGlobalDirection();

      root_distance_start = parent_internode.info.root_distance;
    }

    if (settings.override_radius) {
      thickness_start = settings.radius;
      thickness_end = settings.radius;
    }

    if (settings.presentation_override && settings.presentation_override_settings.max_thickness != 0.0f) {
      thickness_start = glm::min(thickness_start, settings.presentation_override_settings.max_thickness);
      thickness_end = glm::min(thickness_end, settings.presentation_override_settings.max_thickness);
    }

#pragma region Subdivision internode here.
    const auto boundary_length = glm::max(thickness_start, thickness_end) * glm::pi<float>();
    int step = boundary_length / settings.x_subdivision;
    if (step < 4)
      step = 4;
    if (step % 2 != 0)
      ++step;

    temp_steps[thread_index].emplace_back(internode_handle, step);
    int amount = glm::max(
        1, static_cast<int>(glm::distance(position_start, position_end) /
                            (internode_info.thickness >= settings.trunk_thickness ? settings.trunk_y_subdivision
                                                                                    : settings.branch_y_subdivision)));
    if (amount % 2 != 0)
      ++amount;
    amount = glm::max(1, amount);
    BezierCurve curve = BezierCurve(
        position_start,
        position_start +
            (settings.smoothness ? internode_info.length * settings.base_control_point_ratio : 0.0f) * direction_start,
        position_end -
            (settings.smoothness ? internode_info.length * settings.branch_control_point_ratio : 0.0f) * direction_end,
        position_end);

    for (int ring_index = 1; ring_index <= amount; ring_index++) {
      const float a = static_cast<float>(ring_index - 1) / amount;
      const float b = static_cast<float>(ring_index) / amount;
      if (settings.smoothness) {
        rings.emplace_back(a, b, curve.GetPoint(a), curve.GetPoint(b), glm::mix(direction_start, direction_end, a),
                           glm::mix(direction_start, direction_end, b), glm::mix(thickness_start, thickness_end, a) * .5f,
                           glm::mix(thickness_start, thickness_end, b) * .5f,
                           glm::mix(root_distance_start, root_distance_end, a),
                           glm::mix(root_distance_start, root_distance_end, b));
      } else {
        rings.emplace_back(
            a, b, curve.GetPoint(a), curve.GetPoint(b), direction_end, direction_end,
            glm::mix(thickness_start, thickness_end, a) * .5f, glm::mix(thickness_start, thickness_end, b) * .5f,
            glm::mix(root_distance_start, root_distance_end, a), glm::mix(root_distance_start, root_distance_end, b));
      }
    }
#pragma endregion
  });

  for (const auto& list : temp_steps) {
    for (const auto& element : list) {
      steps[element.first] = element.second;
    }
  }

  std::unordered_map<SkeletonNodeHandle, int> vertex_last_ring_start_vertex_index{};

  int next_tree_part_index = 0;
  int next_line_index = 0;
  std::unordered_map<SkeletonNodeHandle, TreePartInfo> tree_part_infos{};

  std::unordered_map<SkeletonFlowHandle, int> flow_bone_id_map;

  GenerateBones(skeleton, sorted_flow_list, offset_matrices, flow_bone_id_map);

  for (int internode_index = 0; internode_index < sorted_internode_list.size(); internode_index++) {
    auto internode_handle = sorted_internode_list[internode_index];
    const auto& internode = skeleton.PeekNode(internode_handle);
    const auto& internode_info = internode.info;
    auto parent_internode_handle = internode.GetParentHandle();
    SkinnedVertex archetype{};
    const auto flow_handle = internode.GetFlowHandle();
    const auto& flow = skeleton.PeekFlow(flow_handle);
    const auto& chain_handles = flow.PeekNodeHandles();
    const auto parent_flow_handle = flow.GetParentHandle();
    float distance_to_chain_start = 0;
    float distance_to_chain_end = 0;
    const auto chain_size = chain_handles.size();
    for (int i = 0; i < chain_size; i++) {
      if (chain_handles[i] == internode_handle)
        break;
      distance_to_chain_start += skeleton.PeekNode(chain_handles[i]).info.length;
    }
    distance_to_chain_end = flow.info.flow_length - distance_to_chain_start;
    if (!internode.IsEndNode())
      distance_to_chain_end -= internode.info.length;
#pragma region TreePart
    if (settings.vertex_color_mode == static_cast<unsigned>(TreeMeshGeneratorSettings::VertexColorMode::Junction)) {
      const bool has_multiple_children = flow.PeekChildHandles().size() > 1;
      bool only_child = true;

      float compare_radius = internode.info.thickness;
      if (parent_flow_handle != -1) {
        const auto& parent_flow = skeleton.PeekFlow(parent_flow_handle);
        only_child = parent_flow.PeekChildHandles().size() <= 1;
        compare_radius = parent_flow.info.end_thickness;
      }
      int tree_part_type = 0;
      if (has_multiple_children && distance_to_chain_end <= settings.tree_part_base_distance * compare_radius) {
        tree_part_type = 1;
      } else if (!only_child && distance_to_chain_start <= settings.tree_part_end_distance * compare_radius) {
        tree_part_type = 2;
      }
      int current_tree_part_index = -1;
      int current_line_index = -1;
      archetype.vertex_info4.y = 0;
      if (tree_part_type == 0) {
        // IShape
        // If root or parent is Y Shape or length exceeds limit, create a new IShape from this node.
        bool restart_i_shape = parent_internode_handle == -1 || tree_part_infos[parent_internode_handle].tree_part_type != 0;
        if (!restart_i_shape) {
          if (const auto& parent_tree_part_info = tree_part_infos[parent_internode_handle];
            parent_tree_part_info.distance_to_start / internode_info.thickness > settings.tree_part_break_ratio)
            restart_i_shape = true;
        }
        if (restart_i_shape) {
          TreePartInfo tree_part_info;
          tree_part_info.tree_part_type = 0;
          tree_part_info.tree_part_index = next_tree_part_index;
          tree_part_info.line_index = next_line_index;
          tree_part_info.distance_to_start = 0.0f;
          tree_part_infos[internode_handle] = tree_part_info;
          current_tree_part_index = next_tree_part_index;
          next_tree_part_index++;

          current_line_index = next_line_index;
          next_line_index++;
        } else {
          auto& current_tree_part_info = tree_part_infos[internode_handle];
          current_tree_part_info = tree_part_infos[parent_internode_handle];
          current_tree_part_info.distance_to_start += internode_info.length;
          current_tree_part_info.tree_part_type = 0;
          current_tree_part_index = current_tree_part_info.tree_part_index;

          current_line_index = current_tree_part_info.line_index;
        }
        archetype.vertex_info4.y = 1;
        // archetype.color = glm::vec4(1, 1, 1, 1);
      } else if (tree_part_type == 1) {
        // Base of Y Shape
        if (parent_internode_handle == -1 || tree_part_infos[parent_internode_handle].tree_part_type != 1 ||
            tree_part_infos[parent_internode_handle].base_flow_handle != flow_handle) {
          TreePartInfo tree_part_info;
          tree_part_info.tree_part_type = 1;
          tree_part_info.tree_part_index = next_tree_part_index;
          tree_part_info.line_index = next_line_index;
          tree_part_info.distance_to_start = 0.0f;
          tree_part_info.base_flow_handle = flow_handle;
          tree_part_infos[internode_handle] = tree_part_info;
          current_tree_part_index = next_tree_part_index;
          next_tree_part_index++;

          current_line_index = next_line_index;
          next_line_index++;
        } else {
          auto& current_tree_part_info = tree_part_infos[internode_handle];
          current_tree_part_info = tree_part_infos[parent_internode_handle];
          current_tree_part_info.tree_part_type = 1;
          current_tree_part_index = current_tree_part_info.tree_part_index;
          current_line_index = current_tree_part_info.line_index;
        }
        archetype.vertex_info4.y = 2;
        // archetype.color = glm::vec4(1, 0, 0, 1);
      } else if (tree_part_type == 2) {
        // Branch of Y Shape
        if (parent_internode_handle == -1 || tree_part_infos[parent_internode_handle].tree_part_type == 0 ||
            tree_part_infos[parent_internode_handle].base_flow_handle != parent_flow_handle) {
        } else {
          auto& current_tree_part_info = tree_part_infos[internode_handle];
          current_tree_part_info = tree_part_infos[parent_internode_handle];
          if (current_tree_part_info.tree_part_type != 2) {
            current_tree_part_info.line_index = next_line_index;
            next_line_index++;
          }
          current_tree_part_info.tree_part_type = 2;
          current_tree_part_index = current_tree_part_info.tree_part_index;

          current_line_index = current_tree_part_info.line_index;
        }
        archetype.vertex_info4.y = 2;
        // archetype.color = glm::vec4(1, 0, 0, 1);
      }
      archetype.vertex_info3 = current_line_index + 1;
      archetype.vertex_info4.x = current_tree_part_index + 1;
    }
#pragma endregion
    const glm::vec3 up = internode_info.regulated_global_rotation * glm::vec3(0, 1, 0);
    glm::vec3 parent_up = up;
    bool need_stitching = false;
    if (parent_internode_handle != -1) {
      const auto& parent_internode = skeleton.PeekNode(parent_internode_handle);
      parent_up = parent_internode.info.regulated_global_rotation * glm::vec3(0, 1, 0);
      if (internode.IsApical() || parent_internode.PeekChildHandles().size() == 1)
        need_stitching = true;
      if (!need_stitching) {
        float max_child_thickness = -1;
        SkeletonNodeHandle max_child_handle = -1;
        for (const auto& child_handle : parent_internode.PeekChildHandles()) {
          const auto& child_internode = skeleton.PeekNode(child_handle);
          if (child_internode.IsApical())
            break;
          if (const float child_thickness = child_internode.info.thickness; child_thickness > max_child_thickness) {
            max_child_thickness = child_thickness;
            max_child_handle = child_handle;
          }
        }
        if (max_child_handle == internode_handle)
          need_stitching = true;
      }
    }

    if (internode.info.length == 0.0f) {
      // TODO: Model possible knots and wound here.
      continue;
    }
    auto& rings = rings_list[internode_index];
    if (rings.empty()) {
      continue;
    }
    // For stitching
    const int step = steps[internode_handle];
    int p_step = step;
    if (need_stitching) {
      p_step = steps[parent_internode_handle];
    }
    float angle_step = 360.0f / static_cast<float>(step);
    float p_angle_step = 360.0f / static_cast<float>(p_step);
    int vertex_index = skinned_vertices.size();

    archetype.vertex_info1 = internode_handle + 1;
    archetype.vertex_info2 = flow_handle + 1;

    if (!need_stitching) {
      for (int p = 0; p < p_step; p++) {
        float x_factor = static_cast<float>(p) / p_step;
        const auto& ring = rings.at(0);
        float y_factor = ring.start_distance_to_root;
        auto direction = ring.GetDirection(parent_up, p_angle_step * p, true);
        archetype.position = ring.start_position + direction * ring.start_radius;
        vertex_position_modifier(archetype.position, direction * ring.start_radius, x_factor, y_factor);
        assert(!glm::any(glm::isnan(archetype.position)));
        archetype.tex_coord = glm::vec2(x_factor, y_factor);
        tex_coords_modifier(archetype.tex_coord, x_factor, y_factor);
        if (settings.vertex_color_mode ==
            static_cast<unsigned>(TreeMeshGeneratorSettings::VertexColorMode::InternodeColor))
          archetype.color = internode_info.color;
        if (parent_flow_handle != -1)
          archetype.bond_id = glm::ivec4(flow_bone_id_map[parent_flow_handle], flow_bone_id_map[parent_flow_handle], -1, -1);
        else {
          archetype.bond_id = glm::ivec4(-1, flow_bone_id_map[0], -1, -1);
        }
        archetype.weight.x = 0.f;
        archetype.weight.y = 1.f;
        skinned_vertices.push_back(archetype);
      }
    }
    archetype.bond_id = glm::ivec4(flow_bone_id_map[flow_handle], flow_bone_id_map[flow_handle], -1, -1);
    archetype.bond_id2 = glm::ivec4(-1);
    archetype.weight = archetype.weight2 = glm::vec4(0.f);

    std::vector<float> angles;
    angles.resize(step);
    std::vector<float> p_angles;
    p_angles.resize(p_step);

    for (auto p = 0; p < p_step; p++) {
      p_angles[p] = p_angle_step * p;
    }
    for (auto s = 0; s < step; s++) {
      angles[s] = angle_step * s;
    }

    std::vector<unsigned> p_target;
    std::vector<unsigned> target;
    p_target.resize(p_step);
    target.resize(step);
    for (int p = 0; p < p_step; p++) {
      // First we allocate nearest skinnedVertices for parent.
      auto min_angle_diff = 360.0f;
      for (auto j = 0; j < step; j++) {
        const float diff = glm::abs(p_angles[p] - angles[j]);
        if (diff < min_angle_diff) {
          min_angle_diff = diff;
          p_target[p] = j;
        }
      }
    }
    for (int s = 0; s < step; s++) {
      // Second we allocate nearest skinnedVertices for child
      float min_angle_diff = 360.0f;
      for (int j = 0; j < p_step; j++) {
        const float diff = glm::abs(angles[s] - p_angles[j]);
        if (diff < min_angle_diff) {
          min_angle_diff = diff;
          target[s] = j;
        }
      }
    }

    int ring_size = rings.size();
    for (auto ring_index = 0; ring_index < ring_size; ring_index++) {
      for (auto s = 0; s < step; s++) {
        float x_factor = static_cast<float>(glm::min(s, step - s)) / step;
        auto& ring = rings.at(ring_index);
        float y_factor = ring.end_distance_to_root;
        auto direction = ring.GetDirection(up, angle_step * s, false);
        archetype.position = ring.end_position + direction * ring.end_radius;
        vertex_position_modifier(archetype.position, direction * ring.end_radius, x_factor, y_factor);
        assert(!glm::any(glm::isnan(archetype.position)));
        archetype.tex_coord = glm::vec2(x_factor, y_factor);
        tex_coords_modifier(archetype.tex_coord, x_factor, y_factor);
        if (settings.vertex_color_mode ==
            static_cast<unsigned>(TreeMeshGeneratorSettings::VertexColorMode::InternodeColor))
          archetype.color = internode_info.color;

        archetype.weight.x = glm::clamp((distance_to_chain_start + ring.end_a * internode.info.length) /
                                              (distance_to_chain_start + distance_to_chain_end),
                                          0.f, 1.f);
        archetype.weight.y = glm::clamp((distance_to_chain_end - ring.end_a * internode.info.length) /
                                              (distance_to_chain_start + distance_to_chain_end),
                                          0.f, 1.f);
        skinned_vertices.push_back(archetype);
      }
      if (ring_index == 0) {
        if (need_stitching) {
          int parent_last_ring_start_vertex_index = vertex_last_ring_start_vertex_index[parent_internode_handle];
          for (int p = 0; p < p_step; p++) {
            if (p_target[p] == p_target[p == p_step - 1 ? 0 : p + 1]) {
              auto a = parent_last_ring_start_vertex_index + p;
              auto b = parent_last_ring_start_vertex_index + (p == p_step - 1 ? 0 : p + 1);
              auto c = vertex_index + p_target[p];
              if (skinned_vertices[a].position != skinned_vertices[b].position &&
                  skinned_vertices[b].position != skinned_vertices[c].position &&
                  skinned_vertices[a].position != skinned_vertices[c].position &&
                  !glm::any(glm::isnan(skinned_vertices[a].position)) &&
                  !glm::any(glm::isnan(skinned_vertices[b].position)) &&
                  !glm::any(glm::isnan(skinned_vertices[c].position))) {
                indices.push_back(a);
                indices.push_back(b);
                indices.push_back(c);
              }
            } else {
              auto a = parent_last_ring_start_vertex_index + p;
              auto b = parent_last_ring_start_vertex_index + (p == p_step - 1 ? 0 : p + 1);
              auto c = vertex_index + p_target[p];
              if (skinned_vertices[a].position != skinned_vertices[b].position &&
                  skinned_vertices[b].position != skinned_vertices[c].position &&
                  skinned_vertices[a].position != skinned_vertices[c].position &&
                  !glm::any(glm::isnan(skinned_vertices[a].position)) &&
                  !glm::any(glm::isnan(skinned_vertices[b].position)) &&
                  !glm::any(glm::isnan(skinned_vertices[c].position))) {
                indices.push_back(a);
                indices.push_back(b);
                indices.push_back(c);
              }
              a = vertex_index + p_target[p == p_step - 1 ? 0 : p + 1];
              b = vertex_index + p_target[p];
              c = parent_last_ring_start_vertex_index + (p == p_step - 1 ? 0 : p + 1);
              if (skinned_vertices[a].position != skinned_vertices[b].position &&
                  skinned_vertices[b].position != skinned_vertices[c].position &&
                  skinned_vertices[a].position != skinned_vertices[c].position &&
                  !glm::any(glm::isnan(skinned_vertices[a].position)) &&
                  !glm::any(glm::isnan(skinned_vertices[b].position)) &&
                  !glm::any(glm::isnan(skinned_vertices[c].position))) {
                indices.push_back(a);
                indices.push_back(b);
                indices.push_back(c);
              }
            }
          }
        } else {
          for (int p = 0; p < p_step; p++) {
            if (p_target[p] == p_target[p == p_step - 1 ? 0 : p + 1]) {
              auto a = vertex_index + p;
              auto b = vertex_index + (p == p_step - 1 ? 0 : p + 1);
              auto c = vertex_index + p_step + p_target[p];
              if (skinned_vertices[a].position != skinned_vertices[b].position &&
                  skinned_vertices[b].position != skinned_vertices[c].position &&
                  skinned_vertices[a].position != skinned_vertices[c].position &&
                  !glm::any(glm::isnan(skinned_vertices[a].position)) &&
                  !glm::any(glm::isnan(skinned_vertices[b].position)) &&
                  !glm::any(glm::isnan(skinned_vertices[c].position))) {
                indices.push_back(a);
                indices.push_back(b);
                indices.push_back(c);
              }
            } else {
              auto a = vertex_index + p;
              auto b = vertex_index + (p == p_step - 1 ? 0 : p + 1);
              auto c = vertex_index + p_step + p_target[p];
              if (skinned_vertices[a].position != skinned_vertices[b].position &&
                  skinned_vertices[b].position != skinned_vertices[c].position &&
                  skinned_vertices[a].position != skinned_vertices[c].position &&
                  !glm::any(glm::isnan(skinned_vertices[a].position)) &&
                  !glm::any(glm::isnan(skinned_vertices[b].position)) &&
                  !glm::any(glm::isnan(skinned_vertices[c].position))) {
                indices.push_back(a);
                indices.push_back(b);
                indices.push_back(c);
              }
              a = vertex_index + p_step + p_target[p == p_step - 1 ? 0 : p + 1];
              b = vertex_index + p_step + p_target[p];
              c = vertex_index + (p == p_step - 1 ? 0 : p + 1);

              if (skinned_vertices[a].position != skinned_vertices[b].position &&
                  skinned_vertices[b].position != skinned_vertices[c].position &&
                  skinned_vertices[a].position != skinned_vertices[c].position &&
                  !glm::any(glm::isnan(skinned_vertices[a].position)) &&
                  !glm::any(glm::isnan(skinned_vertices[b].position)) &&
                  !glm::any(glm::isnan(skinned_vertices[c].position))) {
                indices.push_back(a);
                indices.push_back(b);
                indices.push_back(c);
              }
            }
          }
        }
        if (!need_stitching)
          vertex_index += p_step;
      } else {
        for (int s = 0; s < step - 1; s++) {
          // Down triangle
          auto a = vertex_index + (ring_index - 1) * step + s;
          auto b = vertex_index + (ring_index - 1) * step + s + 1;
          auto c = vertex_index + ring_index * step + s;
          if (skinned_vertices[a].position != skinned_vertices[b].position &&
              skinned_vertices[b].position != skinned_vertices[c].position &&
              skinned_vertices[a].position != skinned_vertices[c].position &&
              !glm::any(glm::isnan(skinned_vertices[a].position)) &&
              !glm::any(glm::isnan(skinned_vertices[b].position)) &&
              !glm::any(glm::isnan(skinned_vertices[c].position))) {
            indices.push_back(a);
            indices.push_back(b);
            indices.push_back(c);
          }

          // Up triangle
          a = vertex_index + ring_index * step + s + 1;
          b = vertex_index + ring_index * step + s;
          c = vertex_index + (ring_index - 1) * step + s + 1;
          if (skinned_vertices[a].position != skinned_vertices[b].position &&
              skinned_vertices[b].position != skinned_vertices[c].position &&
              skinned_vertices[a].position != skinned_vertices[c].position &&
              !glm::any(glm::isnan(skinned_vertices[a].position)) &&
              !glm::any(glm::isnan(skinned_vertices[b].position)) &&
              !glm::any(glm::isnan(skinned_vertices[c].position))) {
            indices.push_back(a);
            indices.push_back(b);
            indices.push_back(c);
          }
        }
        // Down triangle
        auto a = vertex_index + (ring_index - 1) * step + step - 1;
        auto b = vertex_index + (ring_index - 1) * step;
        auto c = vertex_index + ring_index * step + step - 1;
        if (skinned_vertices[a].position != skinned_vertices[b].position &&
            skinned_vertices[b].position != skinned_vertices[c].position &&
            skinned_vertices[a].position != skinned_vertices[c].position &&
            !glm::any(glm::isnan(skinned_vertices[a].position)) &&
            !glm::any(glm::isnan(skinned_vertices[b].position)) &&
            !glm::any(glm::isnan(skinned_vertices[c].position))) {
          indices.push_back(a);
          indices.push_back(b);
          indices.push_back(c);
        }
        // Up triangle
        a = vertex_index + ring_index * step;
        b = vertex_index + ring_index * step + step - 1;
        c = vertex_index + (ring_index - 1) * step;
        if (skinned_vertices[a].position != skinned_vertices[b].position &&
            skinned_vertices[b].position != skinned_vertices[c].position &&
            skinned_vertices[a].position != skinned_vertices[c].position &&
            !glm::any(glm::isnan(skinned_vertices[a].position)) &&
            !glm::any(glm::isnan(skinned_vertices[b].position)) &&
            !glm::any(glm::isnan(skinned_vertices[c].position))) {
          indices.push_back(a);
          indices.push_back(b);
          indices.push_back(c);
        }
      }
    }
    vertex_last_ring_start_vertex_index[internode_handle] = skinned_vertices.size() - step;
  }
}
}  // namespace eco_sys_lab
