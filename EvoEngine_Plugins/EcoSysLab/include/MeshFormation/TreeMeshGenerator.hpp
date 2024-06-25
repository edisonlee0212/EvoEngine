#pragma once

#include "Curve.hpp"
#include "Jobs.hpp"
#include "Octree.hpp"
#include "TreeModel.hpp"
#include "Vertex.hpp"
using namespace evo_engine;
namespace eco_sys_lab {
struct RingSegment {
  float start_a, end_a;
  glm::vec3 start_position, end_position;
  glm::vec3 start_axis, end_axis;
  float start_radius, end_radius;
  float start_distance_to_root;
  float end_distance_to_root;
  RingSegment() = default;

  RingSegment(float start_a, float end_a, glm::vec3 start_position, glm::vec3 end_position, glm::vec3 start_axis,
              glm::vec3 end_axis, float start_radius, float end_radius, float start_distance_to_root,
              float end_distance_to_root);

  void AppendPoints(std::vector<Vertex>& vertices, glm::vec3& normal_dir, int step);

  [[nodiscard]] glm::vec3 GetPoint(const glm::vec3& normal_dir, float angle, bool is_start,
                                   float multiplier = 0.0f) const;
  [[nodiscard]] glm::vec3 GetDirection(const glm::vec3& normal_dir, float angle, bool is_start) const;
};

struct PresentationOverrideSettings {
  float max_thickness = 0.0f;
};

struct TreeMeshGeneratorSettings {
  enum class VertexColorMode {
    InternodeColor,
    Junction,
  };
  unsigned vertex_color_mode = static_cast<unsigned>(VertexColorMode::InternodeColor);

  bool enable_foliage = true;
  bool foliage_instancing = true;
  bool enable_fruit = false;
  bool enable_branch = true;

  bool presentation_override = false;
  PresentationOverrideSettings presentation_override_settings = {};
  bool stitch_all_children = false;
  float trunk_thickness = 0.1f;
  float x_subdivision = 0.01f;
  float trunk_y_subdivision = 0.01f;
  float branch_y_subdivision = 0.01f;

  float radius_multiplier = 1.f;
  bool override_radius = false;
  float radius = 0.01f;
  float base_control_point_ratio = 0.3f;
  float branch_control_point_ratio = 0.3f;
  bool smoothness = true;

  bool auto_level = true;
  int voxel_subdivision_level = 10;
  int voxel_smooth_iteration = 5;
  bool remove_duplicate = true;

  unsigned branch_mesh_type = 0;

  float tree_part_base_distance = 0.5f;
  float tree_part_end_distance = 2.f;
  float tree_part_break_ratio = 4.0f;

  float marching_cube_radius = 0.01f;

  void OnInspect(const std::shared_ptr<EditorLayer>& editor_layer);

  void Save(const std::string& name, YAML::Emitter& out);

  void Load(const std::string& name, const YAML::Node& in);
};

template <typename SkeletonData, typename FlowData, typename NodeData>
class CylindricalMeshGenerator {
 public:
  static void Generate(
      const Skeleton<SkeletonData, FlowData, NodeData>& skeleton, std::vector<Vertex>& vertices,
      std::vector<unsigned int>& indices, const TreeMeshGeneratorSettings& settings,
      const std::function<void(glm::vec3& vertex_position, const glm::vec3& direction, float x_factor,
                               float distance_to_root)>& vertex_position_modifier,
      const std::function<void(glm::vec2& tex_coords, float x_factor, float distance_to_root)>& tex_coords_modifier);

  static void GeneratePartially(
      const std::unordered_set<SkeletonNodeHandle>& node_handles,
      const Skeleton<SkeletonData, FlowData, NodeData>& skeleton, std::vector<Vertex>& vertices,
      std::vector<unsigned int>& indices, const TreeMeshGeneratorSettings& settings,
      const std::function<void(glm::vec3& vertex_position, const glm::vec3& direction, float x_factor,
                               float distance_to_root)>& vertex_position_modifier,
      const std::function<void(glm::vec2& tex_coords, float x_factor, float distance_to_root)>& tex_coords_modifier);
};
template <typename SkeletonData, typename FlowData, typename NodeData>
class VoxelMeshGenerator {
 public:
  static void Generate(const Skeleton<SkeletonData, FlowData, NodeData>& tree_skeleton, std::vector<Vertex>& vertices,
                       std::vector<unsigned int>& indices, const TreeMeshGeneratorSettings& settings);
};

struct TreePartInfo {
  int tree_part_index = -1;
  int line_index = -1;
  int tree_part_type = 0;
  float distance_to_start = 0.0f;
  SkeletonFlowHandle base_flow_handle = -1;
};

template <typename SkeletonData, typename FlowData, typename NodeData>
void CylindricalMeshGenerator<SkeletonData, FlowData, NodeData>::Generate(
    const Skeleton<SkeletonData, FlowData, NodeData>& skeleton, std::vector<Vertex>& vertices,
    std::vector<unsigned int>& indices, const TreeMeshGeneratorSettings& settings,
    const std::function<void(glm::vec3& vertex_position, const glm::vec3& direction, float x_factor, float y_factor)>&
        vertex_position_modifier,
    const std::function<void(glm::vec2& tex_coords, float x_factor, float y_factor)>& tex_coords_modifier) {
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
    glm::vec3 position_end =
        position_start + internode_info.length *
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

    thickness_start *= settings.radius_multiplier;
    thickness_end *= settings.radius_multiplier;

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

  for (int internode_index = 0; internode_index < sorted_internode_list.size(); internode_index++) {
    auto internode_handle = sorted_internode_list[internode_index];
    const auto& internode = skeleton.PeekNode(internode_handle);
    const auto& internode_info = internode.info;
    auto parent_internode_handle = internode.GetParentHandle();
    Vertex archetype;
    const auto flow_handle = internode.GetFlowHandle();
    if (settings.vertex_color_mode == static_cast<unsigned>(TreeMeshGeneratorSettings::VertexColorMode::Junction)) {
#pragma region TreePart
      const auto& flow = skeleton.PeekFlow(flow_handle);
      const auto& chain_handles = flow.PeekNodeHandles();
      const bool has_multiple_children = flow.PeekChildHandles().size() > 1;
      bool only_child = true;
      const auto parent_flow_handle = flow.GetParentHandle();
      float distance_to_chain_start = 0;
      float distance_to_chain_end = 0;
      const auto chain_size = chain_handles.size();
      for (int i = 0; i < chain_size; i++) {
        if (chain_handles[i] == internode_handle)
          break;
        distance_to_chain_start += skeleton.PeekNode(chain_handles[i]).info.length;
      }
      distance_to_chain_end = flow.info.flow_length - distance_to_chain_start - internode.info.length;
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
          if (const auto& parent_tree_part_info = tree_part_infos[parent_internode_handle]; parent_tree_part_info.distance_to_start / internode_info.thickness > settings.tree_part_break_ratio)
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

#pragma endregion
    }
    const glm::vec3 up = internode_info.regulated_global_rotation * glm::vec3(0, 1, 0);
    glm::vec3 parent_up = up;
    bool need_stitching = false;
    if (parent_internode_handle != -1) {
      if (settings.stitch_all_children) {
        need_stitching = true;
      } else {
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
    int vertex_index = vertices.size();

    archetype.vertex_info1 = internode_handle + 1;
    archetype.vertex_info2 = flow_handle + 1;

    if (!need_stitching) {
      int parent_last_ring_start_vertex_index =
          parent_internode_handle == -1 ? -1 : vertex_last_ring_start_vertex_index[parent_internode_handle];
      for (int p = 0; p < p_step; p++) {
        if (parent_internode_handle != -1)
          vertices.push_back(vertices.at(parent_last_ring_start_vertex_index + p));
        else {
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
          vertices.push_back(archetype);
        }
      }
    }
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
      // First we allocate nearest vertices for parent.
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
      // Second we allocate nearest vertices for child
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
        vertices.push_back(archetype);
      }
      if (ring_index == 0) {
        if (need_stitching) {
          int parent_last_ring_start_vertex_index = vertex_last_ring_start_vertex_index[parent_internode_handle];
          for (int p = 0; p < p_step; p++) {
            if (p_target[p] == p_target[p == p_step - 1 ? 0 : p + 1]) {
              auto a = parent_last_ring_start_vertex_index + p;
              auto b = parent_last_ring_start_vertex_index + (p == p_step - 1 ? 0 : p + 1);
              auto c = vertex_index + p_target[p];
              if (vertices[a].position != vertices[b].position &&
                  vertices[b].position != vertices[c].position &&
                  vertices[a].position != vertices[c].position && !glm::any(glm::isnan(vertices[a].position)) &&
                  !glm::any(glm::isnan(vertices[b].position)) && !glm::any(glm::isnan(vertices[c].position))) {
                indices.push_back(a);
                indices.push_back(b);
                indices.push_back(c);
              }
            } else {
              auto a = parent_last_ring_start_vertex_index + p;
              auto b = parent_last_ring_start_vertex_index + (p == p_step - 1 ? 0 : p + 1);
              auto c = vertex_index + p_target[p];
              if (vertices[a].position != vertices[b].position &&
                  vertices[b].position != vertices[c].position &&
                  vertices[a].position != vertices[c].position && !glm::any(glm::isnan(vertices[a].position)) &&
                  !glm::any(glm::isnan(vertices[b].position)) && !glm::any(glm::isnan(vertices[c].position))) {
                indices.push_back(a);
                indices.push_back(b);
                indices.push_back(c);
              }
              a = vertex_index + p_target[p == p_step - 1 ? 0 : p + 1];
              b = vertex_index + p_target[p];
              c = parent_last_ring_start_vertex_index + (p == p_step - 1 ? 0 : p + 1);
              if (vertices[a].position != vertices[b].position &&
                  vertices[b].position != vertices[c].position &&
                  vertices[a].position != vertices[c].position && !glm::any(glm::isnan(vertices[a].position)) &&
                  !glm::any(glm::isnan(vertices[b].position)) && !glm::any(glm::isnan(vertices[c].position))) {
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
              if (vertices[a].position != vertices[b].position &&
                  vertices[b].position != vertices[c].position &&
                  vertices[a].position != vertices[c].position && !glm::any(glm::isnan(vertices[a].position)) &&
                  !glm::any(glm::isnan(vertices[b].position)) && !glm::any(glm::isnan(vertices[c].position))) {
                indices.push_back(a);
                indices.push_back(b);
                indices.push_back(c);
              }
            } else {
              auto a = vertex_index + p;
              auto b = vertex_index + (p == p_step - 1 ? 0 : p + 1);
              auto c = vertex_index + p_step + p_target[p];
              if (vertices[a].position != vertices[b].position &&
                  vertices[b].position != vertices[c].position &&
                  vertices[a].position != vertices[c].position && !glm::any(glm::isnan(vertices[a].position)) &&
                  !glm::any(glm::isnan(vertices[b].position)) && !glm::any(glm::isnan(vertices[c].position))) {
                indices.push_back(a);
                indices.push_back(b);
                indices.push_back(c);
              }
              a = vertex_index + p_step + p_target[p == p_step - 1 ? 0 : p + 1];
              b = vertex_index + p_step + p_target[p];
              c = vertex_index + (p == p_step - 1 ? 0 : p + 1);

              if (vertices[a].position != vertices[b].position &&
                  vertices[b].position != vertices[c].position &&
                  vertices[a].position != vertices[c].position && !glm::any(glm::isnan(vertices[a].position)) &&
                  !glm::any(glm::isnan(vertices[b].position)) && !glm::any(glm::isnan(vertices[c].position))) {
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
          if (vertices[a].position != vertices[b].position && vertices[b].position != vertices[c].position &&
              vertices[a].position != vertices[c].position && !glm::any(glm::isnan(vertices[a].position)) &&
              !glm::any(glm::isnan(vertices[b].position)) && !glm::any(glm::isnan(vertices[c].position))) {
            indices.push_back(a);
            indices.push_back(b);
            indices.push_back(c);
          }

          // Up triangle
          a = vertex_index + ring_index * step + s + 1;
          b = vertex_index + ring_index * step + s;
          c = vertex_index + (ring_index - 1) * step + s + 1;
          if (vertices[a].position != vertices[b].position && vertices[b].position != vertices[c].position &&
              vertices[a].position != vertices[c].position && !glm::any(glm::isnan(vertices[a].position)) &&
              !glm::any(glm::isnan(vertices[b].position)) && !glm::any(glm::isnan(vertices[c].position))) {
            indices.push_back(a);
            indices.push_back(b);
            indices.push_back(c);
          }
        }
        // Down triangle
        auto a = vertex_index + (ring_index - 1) * step + step - 1;
        auto b = vertex_index + (ring_index - 1) * step;
        auto c = vertex_index + ring_index * step + step - 1;
        if (vertices[a].position != vertices[b].position && vertices[b].position != vertices[c].position &&
            vertices[a].position != vertices[c].position && !glm::any(glm::isnan(vertices[a].position)) &&
            !glm::any(glm::isnan(vertices[b].position)) && !glm::any(glm::isnan(vertices[c].position))) {
          indices.push_back(a);
          indices.push_back(b);
          indices.push_back(c);
        }
        // Up triangle
        a = vertex_index + ring_index * step;
        b = vertex_index + ring_index * step + step - 1;
        c = vertex_index + (ring_index - 1) * step;
        if (vertices[a].position != vertices[b].position && vertices[b].position != vertices[c].position &&
            vertices[a].position != vertices[c].position && !glm::any(glm::isnan(vertices[a].position)) &&
            !glm::any(glm::isnan(vertices[b].position)) && !glm::any(glm::isnan(vertices[c].position))) {
          indices.push_back(a);
          indices.push_back(b);
          indices.push_back(c);
        }
      }
    }
    vertex_last_ring_start_vertex_index[internode_handle] = vertices.size() - step;
  }
}

template <typename SkeletonData, typename FlowData, typename NodeData>
void CylindricalMeshGenerator<SkeletonData, FlowData, NodeData>::GeneratePartially(
    const std::unordered_set<SkeletonNodeHandle>& node_handles,
    const Skeleton<SkeletonData, FlowData, NodeData>& skeleton, std::vector<Vertex>& vertices,
    std::vector<unsigned>& indices, const TreeMeshGeneratorSettings& settings,
    const std::function<void(glm::vec3& vertex_position, const glm::vec3& direction, float x_factor,
                             float distance_to_root)>& vertex_position_modifier,
    const std::function<void(glm::vec2& tex_coords, float x_factor, float distance_to_root)>& tex_coords_modifier) {
  const auto& sorted_internode_list = skeleton.PeekSortedNodeList();
  std::vector<std::vector<RingSegment>> rings_list;
  std::unordered_map<SkeletonNodeHandle, int> steps{};
  rings_list.resize(sorted_internode_list.size());
  std::vector<std::shared_future<void>> results;
  std::vector<std::vector<std::pair<SkeletonNodeHandle, int>>> temp_steps{};
  temp_steps.resize(Jobs::GetWorkerSize());

  Jobs::RunParallelFor(sorted_internode_list.size(), [&](unsigned internode_index, unsigned thread_index) {
    auto internode_handle = sorted_internode_list[internode_index];
    const auto& internode = skeleton.PeekNode(internode_handle);
    const auto& internode_info = internode.info;

    auto& rings = rings_list[internode_index];
    rings.clear();

    bool has_parent = node_handles.find(internode.GetParentHandle()) != node_handles.end();
    glm::vec3 direction_start = internode_info.regulated_global_rotation * glm::vec3(0, 0, -1);
    glm::vec3 direction_end = direction_start;
    float root_distance_start = internode_info.root_distance - 0.05f;  // this is a total hack, but whatever
    float root_distance_end = internode_info.root_distance;

    glm::vec3 position_start = internode_info.global_position;
    glm::vec3 position_end =
        position_start + internode_info.length *
                            (settings.smoothness ? 1.0f - settings.base_control_point_ratio : 1.0f) *
                            internode_info.GetGlobalDirection();
    float thickness_start = internode_info.thickness;
    float thickness_end = internode_info.thickness;

    if (has_parent) {
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
  std::unordered_map<SkeletonNodeHandle, TreePartInfo> tree_part_infos{};

  for (int internode_index = 0; internode_index < sorted_internode_list.size(); internode_index++) {
    auto internode_handle = sorted_internode_list[internode_index];
    if (node_handles.find(internode_handle) == node_handles.end())
      continue;
    const auto& internode = skeleton.PeekNode(internode_handle);
    const auto& internode_info = internode.info;
    auto parent_internode_handle = internode.GetParentHandle();

    bool has_parent = node_handles.find(internode.GetParentHandle()) != node_handles.end();
    bool need_stitching = false;

    const glm::vec3 up = internode_info.regulated_global_rotation * glm::vec3(0, 1, 0);
    glm::vec3 parent_up = up;

    if (has_parent) {
      if (settings.stitch_all_children) {
        need_stitching = true;
      } else {
        const auto& parent_internode = skeleton.PeekNode(parent_internode_handle);
        parent_up = parent_internode.info.regulated_global_rotation * glm::vec3(0, 1, 0);
        if (internode.IsApical() || parent_internode.PeekChildHandles().size() == 1)
          need_stitching = true;
        if (!need_stitching) {
          float max_child_thickness = -1;
          SkeletonNodeHandle max_child_handle = -1;
          for (const auto& child_handle : parent_internode.PeekChildHandles()) {
            if (node_handles.find(child_handle) == node_handles.end())
              continue;
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
    int vertex_index = vertices.size();
    Vertex archetype;
    const auto flow_handle = internode.GetFlowHandle();
    archetype.vertex_info1 = internode_handle + 1;
    archetype.vertex_info2 = flow_handle + 1;
    if (!need_stitching) {
      int parent_last_ring_start_vertex_index = has_parent ? vertex_last_ring_start_vertex_index[parent_internode_handle] : -1;
      for (int p = 0; p < p_step; p++) {
        if (has_parent) {
          vertices.push_back(vertices.at(parent_last_ring_start_vertex_index + p));
        } else {
          float x_factor = static_cast<float>(glm::min(p, p_step - p)) / p_step;
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
          vertices.push_back(archetype);
        }
      }
    }
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
      // First we allocate nearest vertices for parent.
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
      // Second we allocate nearest vertices for child
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
        float x_factor = static_cast<float>(glm::min(s, step - s)) / step;  // another hack
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
        vertices.push_back(archetype);
      }
      if (ring_index == 0) {
        if (need_stitching) {
          int parent_last_ring_start_vertex_index = vertex_last_ring_start_vertex_index[parent_internode_handle];
          for (int p = 0; p < p_step; p++) {
            if (p_target[p] == p_target[p == p_step - 1 ? 0 : p + 1]) {
              auto a = parent_last_ring_start_vertex_index + p;
              auto b = parent_last_ring_start_vertex_index + (p == p_step - 1 ? 0 : p + 1);
              auto c = vertex_index + p_target[p];
              if (vertices[a].position != vertices[b].position &&
                  vertices[b].position != vertices[c].position &&
                  vertices[a].position != vertices[c].position) {
                indices.push_back(a);
                indices.push_back(b);
                indices.push_back(c);
              }
            } else {
              auto a = parent_last_ring_start_vertex_index + p;
              auto b = parent_last_ring_start_vertex_index + (p == p_step - 1 ? 0 : p + 1);
              auto c = vertex_index + p_target[p];
              if (vertices[a].position != vertices[b].position &&
                  vertices[b].position != vertices[c].position &&
                  vertices[a].position != vertices[c].position) {
                indices.push_back(a);
                indices.push_back(b);
                indices.push_back(c);
              }
              a = vertex_index + p_target[p == p_step - 1 ? 0 : p + 1];
              b = vertex_index + p_target[p];
              c = parent_last_ring_start_vertex_index + (p == p_step - 1 ? 0 : p + 1);
              if (vertices[a].position != vertices[b].position &&
                  vertices[b].position != vertices[c].position &&
                  vertices[a].position != vertices[c].position) {
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
              if (vertices[a].position != vertices[b].position &&
                  vertices[b].position != vertices[c].position &&
                  vertices[a].position != vertices[c].position) {
                indices.push_back(a);
                indices.push_back(b);
                indices.push_back(c);
              }
            } else {
              auto a = vertex_index + p;
              auto b = vertex_index + (p == p_step - 1 ? 0 : p + 1);
              auto c = vertex_index + p_step + p_target[p];
              if (vertices[a].position != vertices[b].position &&
                  vertices[b].position != vertices[c].position &&
                  vertices[a].position != vertices[c].position) {
                indices.push_back(a);
                indices.push_back(b);
                indices.push_back(c);
              }
              a = vertex_index + p_step + p_target[p == p_step - 1 ? 0 : p + 1];
              b = vertex_index + p_step + p_target[p];
              c = vertex_index + (p == p_step - 1 ? 0 : p + 1);

              if (vertices[a].position != vertices[b].position &&
                  vertices[b].position != vertices[c].position &&
                  vertices[a].position != vertices[c].position) {
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
          if (vertices[a].position != vertices[b].position && vertices[b].position != vertices[c].position &&
              vertices[a].position != vertices[c].position) {
            indices.push_back(a);
            indices.push_back(b);
            indices.push_back(c);
          }

          // Up triangle
          a = vertex_index + ring_index * step + s + 1;
          b = vertex_index + ring_index * step + s;
          c = vertex_index + (ring_index - 1) * step + s + 1;
          if (vertices[a].position != vertices[b].position && vertices[b].position != vertices[c].position &&
              vertices[a].position != vertices[c].position) {
            indices.push_back(a);
            indices.push_back(b);
            indices.push_back(c);
          }
        }
        // Down triangle
        auto a = vertex_index + (ring_index - 1) * step + step - 1;
        auto b = vertex_index + (ring_index - 1) * step;
        auto c = vertex_index + ring_index * step + step - 1;
        if (vertices[a].position != vertices[b].position && vertices[b].position != vertices[c].position &&
            vertices[a].position != vertices[c].position) {
          indices.push_back(a);
          indices.push_back(b);
          indices.push_back(c);
        }
        // Up triangle
        a = vertex_index + ring_index * step;
        b = vertex_index + ring_index * step + step - 1;
        c = vertex_index + (ring_index - 1) * step;
        if (vertices[a].position != vertices[b].position && vertices[b].position != vertices[c].position &&
            vertices[a].position != vertices[c].position) {
          indices.push_back(a);
          indices.push_back(b);
          indices.push_back(c);
        }
      }
    }
    vertex_last_ring_start_vertex_index[internode_handle] = vertices.size() - step;
  }
}

template <typename SkeletonData, typename FlowData, typename NodeData>
void VoxelMeshGenerator<SkeletonData, FlowData, NodeData>::Generate(
    const Skeleton<SkeletonData, FlowData, NodeData>& tree_skeleton, std::vector<Vertex>& vertices,
    std::vector<unsigned>& indices, const TreeMeshGeneratorSettings& settings) {
  const auto box_size = tree_skeleton.max - tree_skeleton.min;
  Octree<bool> octree;
  if (settings.auto_level) {
    const float max_radius =
        glm::max(glm::max(box_size.x, box_size.y), box_size.z) * 0.5f + 2.0f * settings.marching_cube_radius;
    int subdivision_level = -1;
    float test_radius = settings.marching_cube_radius;
    while (test_radius <= max_radius) {
      subdivision_level++;
      test_radius *= 2.f;
    }
    EVOENGINE_LOG("Mesh formation: Auto set level to " + std::to_string(subdivision_level))

    octree.Reset(max_radius, subdivision_level, (tree_skeleton.min + tree_skeleton.max) * 0.5f);
  } else {
    octree.Reset(glm::max((box_size.x, box_size.y), glm::max(box_size.y, box_size.z)) * 0.5f,
                 glm::clamp(settings.voxel_subdivision_level, 4, 16), (tree_skeleton.min + tree_skeleton.max) / 2.0f);
  }
  auto& node_list = tree_skeleton.PeekSortedNodeList();
  for (const auto& node_index : node_list) {
    const auto& node = tree_skeleton.PeekNode(node_index);
    const auto& info = node.info;
    auto thickness = info.thickness;
    if (node.GetParentHandle() > 0) {
      thickness = (thickness + tree_skeleton.PeekNode(node.GetParentHandle()).info.thickness) / 2.0f;
    }
    octree.Occupy(info.global_position, info.global_rotation, info.length, thickness, [](OctreeNode&) {
    });
  }
  octree.TriangulateField(vertices, indices, settings.remove_duplicate);
}
}  // namespace eco_sys_lab
