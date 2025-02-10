#include "TreeStructor.hpp"
#include <unordered_set>
#include "EcoSysLabLayer.hpp"
#include "FoliageDescriptor.hpp"
#include "Platform.hpp"
#include "rapidcsv.h"
using namespace eco_sys_lab_plugin;

void TreeStructor::ApplyCurve(const OperatorBranch& branch) {
  auto& skeleton = skeletons[branch.skeleton_index];
  const auto chain_amount = branch.chain_node_handles.size();
  for (int i = 0; i < chain_amount; i++) {
    auto& node = skeleton.RefNode(branch.chain_node_handles[i]);
    node.data.global_start_position = branch.bezier_curve.GetPoint(static_cast<float>(i) / chain_amount);
    node.data.global_end_position = branch.bezier_curve.GetPoint(static_cast<float>(i + 1) / chain_amount);
    node.info.thickness = node.data.imported_thickness = branch.thickness;
    node.data.branch_handle = branch.handle;
    node.info.color = glm::vec4(branch.color, 1.0f);
    if (reconstruction_settings.use_foliage)
      node.info.leaves = branch.foliage;
    else
      node.info.leaves = 1.f;
  }
}

void TreeStructor::BuildVoxelGrid() {
  scatter_points_voxel_grid.Initialize(2.0f * connectivity_graph_settings.point_point_connection_detection_radius, min,
                                       max);
  allocated_points_voxel_grid.Initialize(2.0f * connectivity_graph_settings.point_point_connection_detection_radius,
                                         min, max);

  branch_ends_voxel_grid.Initialize(2.0f * connectivity_graph_settings.point_point_connection_detection_radius, min,
                                    max);
  for (auto& point : allocated_points) {
    point.branch_handle = point.node_handle = point.skeleton_index = -1;
  }
  for (auto& point : scattered_points) {
    point.neighbor_scatter_points.clear();
    point.p3.clear();
    point.p0.clear();
    PointData voxel;
    voxel.handle = point.handle;
    voxel.position = point.position;
    scatter_points_voxel_grid.Ref(point.position).emplace_back(voxel);
  }
  for (auto& point : allocated_points) {
    PointData voxel;
    voxel.handle = point.handle;
    voxel.position = point.position;
    allocated_points_voxel_grid.Ref(point.position).emplace_back(voxel);
  }
  for (auto& predicted_branch : predicted_branches) {
    predicted_branch.points_to_p3.clear();
    predicted_branch.p3_to_p0.clear();

    predicted_branch.points_to_p0.clear();
    predicted_branch.p3_to_p3.clear();
    predicted_branch.p0_to_p0.clear();
    predicted_branch.p0_to_p3.clear();

    BranchEndData voxel;
    voxel.branch_handle = predicted_branch.handle;
    voxel.position = predicted_branch.bezier_curve.p0;
    voxel.is_p0 = true;
    branch_ends_voxel_grid.Ref(predicted_branch.bezier_curve.p0).emplace_back(voxel);
    voxel.position = predicted_branch.bezier_curve.p3;
    voxel.is_p0 = false;
    branch_ends_voxel_grid.Ref(predicted_branch.bezier_curve.p3).emplace_back(voxel);
  }
}

bool TreeStructor::DirectConnectionCheck(const BezierCurve& parent_curve, const BezierCurve& child_curve,
                                         bool reverse) {
  const auto parent_pa = parent_curve.p0;
  const auto parent_pb = parent_curve.p3;
  const auto child_pa = reverse ? child_curve.p3 : child_curve.p0;
  const auto child_pb = reverse ? child_curve.p0 : child_curve.p3;
  if (const auto dot_p = glm::dot(glm::normalize(parent_pb - parent_pa), glm::normalize(child_pb - child_pa));
      dot_p < glm::cos(glm::radians(connectivity_graph_settings.direction_connection_angle_limit)))
    return false;
  if (connectivity_graph_settings.zigzag_check) {
    auto shortened_parent_p0 = parent_curve.GetPoint(connectivity_graph_settings.zigzag_branch_shortening);
    auto shortened_parent_p3 = parent_curve.GetPoint(1.0f - connectivity_graph_settings.zigzag_branch_shortening);
    auto shortened_child_p0 = child_curve.GetPoint(connectivity_graph_settings.zigzag_branch_shortening);
    auto shortened_child_p3 = child_curve.GetPoint(1.0f - connectivity_graph_settings.zigzag_branch_shortening);

    const auto dot_c0 = glm::dot(glm::normalize(shortened_child_p3 - shortened_child_p0),
                                 glm::normalize(shortened_parent_p3 - shortened_child_p0));
    // const auto dotC3 = glm::dot(glm::normalize(shortenedChildP0 - shortenedChildP3), glm::normalize(shortenedParentP3
    // - shortenedChildP3));
    const auto dot_p3 = glm::dot(glm::normalize(shortened_child_p0 - shortened_parent_p3),
                                 glm::normalize(shortened_parent_p0 - shortened_parent_p3));
    // const auto dotP0 = glm::dot(glm::normalize(shortenedChildP0 - shortenedParentP0),
    // glm::normalize(shortenedParentP3 - shortenedParentP0));
    if (dot_c0 > 0 || dot_p3 > 0 /* && dotP0 < 0*/)
      return false;
  }
  if (connectivity_graph_settings.parallel_shift_check &&
      parent_pb.y > connectivity_graph_settings.parallel_shift_check_height_limit &&
      child_pa.y > connectivity_graph_settings.parallel_shift_check_height_limit) {
    const auto parent_direction = glm::normalize(parent_pa - parent_pb);
    const auto projected_c0 =
        glm::closestPointOnLine(child_pa, parent_pa + 10.0f * parent_direction, parent_pb - 10.0f * parent_direction);
    const auto child_length = glm::distance(child_pa, child_pb);
    const auto parent_length = glm::distance(parent_pa, parent_pb);
    if (const auto projected_length = glm::distance(projected_c0, child_pa);
        projected_length > connectivity_graph_settings.parallel_shift_limit_range * child_length ||
        projected_length > connectivity_graph_settings.parallel_shift_limit_range * parent_length)
      return false;
  }
  if (connectivity_graph_settings.point_existence_check &&
      connectivity_graph_settings.point_existence_check_radius > 0.0f) {
    const auto middle_point = (child_pa + parent_pb) * 0.5f;
    if (!HasPoints(middle_point, allocated_points_voxel_grid,
                   connectivity_graph_settings.point_existence_check_radius) &&
        !HasPoints(middle_point, scatter_points_voxel_grid, connectivity_graph_settings.point_existence_check_radius))
      return false;
  }
  return true;
}

void TreeStructor::FindPoints(const glm::vec3& position, VoxelGrid<std::vector<PointData>>& point_voxel_grid,
                              const float radius, const std::function<void(const PointData& voxel)>& func) {
  point_voxel_grid.ForEach(position, radius, [&](const std::vector<PointData>& voxels) {
    for (const auto& voxel : voxels) {
      if (glm::distance(position, voxel.position) > radius)
        continue;
      func(voxel);
    }
  });
}

bool TreeStructor::HasPoints(const glm::vec3& position, VoxelGrid<std::vector<PointData>>& point_voxel_grid,
                             const float radius) {
  bool ret_val = false;
  point_voxel_grid.ForEach(position, radius, [&](const std::vector<PointData>& voxels) {
    if (ret_val)
      return;
    for (const auto& voxel : voxels) {
      if (glm::distance(position, voxel.position) <= radius)
        ret_val = true;
    }
  });
  return ret_val;
}

void TreeStructor::ForEachBranchEnd(const glm::vec3& position,
                                    VoxelGrid<std::vector<BranchEndData>>& branchEndsVoxelGrid, float radius,
                                    const std::function<void(const BranchEndData& voxel)>& func) {
  branchEndsVoxelGrid.ForEach(position, radius, [&](const std::vector<BranchEndData>& branchEnds) {
    for (const auto& branchEnd : branchEnds) {
      if (glm::distance(position, branchEnd.position) > radius)
        continue;
      func(branchEnd);
    }
  });
}

void TreeStructor::CalculateNodeTransforms(ReconstructionSkeleton& skeleton) {
  skeleton.min = glm::vec3(FLT_MAX);
  skeleton.max = glm::vec3(FLT_MIN);
  for (const auto& node_handle : skeleton.PeekSortedNodeList()) {
    auto& node = skeleton.RefNode(node_handle);
    auto& node_info = node.info;
    if (node.GetParentHandle() != -1) {
      auto& parent_info = skeleton.RefNode(node.GetParentHandle()).info;
      node_info.global_position = parent_info.global_position + parent_info.length * parent_info.GetGlobalDirection();
      auto parent_regulated_up = parent_info.regulated_global_rotation * glm::vec3(0, 1, 0);
      auto regulated_up = glm::normalize(
          glm::cross(glm::cross(node_info.GetGlobalDirection(), parent_regulated_up), node_info.GetGlobalDirection()));
      node_info.regulated_global_rotation = glm::quatLookAt(node_info.GetGlobalDirection(), regulated_up);
    }
    skeleton.min = glm::min(skeleton.min, node_info.global_position);
    skeleton.max = glm::max(skeleton.max, node_info.global_position);
    const auto end_position = node_info.global_position + node_info.length * node_info.GetGlobalDirection();
    skeleton.min = glm::min(skeleton.min, end_position);
    skeleton.max = glm::max(skeleton.max, end_position);
  }
}

void TreeStructor::BuildConnectionBranch(const BranchHandle processing_branch_handle,
                                         SkeletonNodeHandle& prev_node_handle) {
  operating_branches.emplace_back();
  auto& processing_branch = operating_branches[processing_branch_handle];
  auto& skeleton = skeletons[processing_branch.skeleton_index];
  auto& connection_branch = operating_branches.back();
  auto& parent_branch = operating_branches[processing_branch.parent_handle];
  assert(parent_branch.skeleton_index == processing_branch.skeleton_index);
  connection_branch.color = parent_branch.color;
  connection_branch.handle = operating_branches.size() - 1;
  connection_branch.skeleton_index = processing_branch.skeleton_index;
  connection_branch.thickness = (parent_branch.thickness + processing_branch.thickness) * 0.5f;
  connection_branch.child_handles.emplace_back(processing_branch.handle);
  connection_branch.parent_handle = processing_branch.parent_handle;
  processing_branch.parent_handle = connection_branch.handle;
  for (int& child_handle : parent_branch.child_handles) {
    if (child_handle == processing_branch.handle) {
      child_handle = connection_branch.handle;
      break;
    }
  }

  SkeletonNodeHandle best_prev_node_handle = parent_branch.chain_node_handles.back();
  float dot_max = -1.0f;
  glm::vec3 connection_branch_start_position = parent_branch.bezier_curve.p3;
  SkeletonNodeHandle back_track_walker = best_prev_node_handle;
  int branch_back_track_count = 0;
  BranchHandle prev_branch_handle = processing_branch.parent_handle;
  for (int i = 0; i < reconstruction_settings.node_back_track_limit; i++) {
    if (back_track_walker == -1)
      break;
    auto& node = skeleton.PeekNode(back_track_walker);
    if (node.data.branch_handle != prev_branch_handle) {
      branch_back_track_count++;
      prev_branch_handle = node.data.branch_handle;
    }
    if (branch_back_track_count > reconstruction_settings.branch_back_track_limit)
      break;
    const auto node_end_position = node.data.global_end_position;
    // fad
    if (const auto dot_val =
            glm::dot(glm::normalize(processing_branch.bezier_curve.p3 - processing_branch.bezier_curve.p0),
                     glm::normalize(processing_branch.bezier_curve.p0 - node_end_position));
        dot_val > dot_max) {
      dot_max = dot_val;
      best_prev_node_handle = back_track_walker;
      connection_branch_start_position = node_end_position;
    }
    back_track_walker = node.GetParentHandle();
  }

  connection_branch.bezier_curve.p0 = connection_branch_start_position;

  connection_branch.bezier_curve.p3 = processing_branch.bezier_curve.p0;

  connection_branch.bezier_curve.p1 =
      glm::mix(connection_branch.bezier_curve.p0, connection_branch.bezier_curve.p3, 0.25f);

  connection_branch.bezier_curve.p2 =
      glm::mix(connection_branch.bezier_curve.p0, connection_branch.bezier_curve.p3, 0.75f);

  prev_node_handle = best_prev_node_handle;

  auto connection_first_node_handle = skeleton.Extend(prev_node_handle, !processing_branch.apical);
  connection_branch.chain_node_handles.emplace_back(connection_first_node_handle);
  prev_node_handle = connection_first_node_handle;
  const float connection_chain_length = connection_branch.bezier_curve.GetLength();
  const int connection_chain_amount =
      glm::max(2, static_cast<int>(connection_chain_length / reconstruction_settings.internode_length));
  /*
  if(const auto treeDescriptor = tree_descriptor_ref.Get<TreeDescriptor>())
  {
          if(const auto shootDescriptor = treeDescriptor->m_shootDescriptor.Get<ShootDescriptor>())
          {
                  connectionChainAmount = glm::max(2, static_cast<int>(connectionChainLength /
                          shootDescriptor->internode_length));
          }
  }*/
  for (int i = 1; i < connection_chain_amount; i++) {
    prev_node_handle = skeleton.Extend(prev_node_handle, false);
    connection_branch.chain_node_handles.emplace_back(prev_node_handle);
  }
  ApplyCurve(connection_branch);
  prev_node_handle = skeleton.Extend(prev_node_handle, false);
  processing_branch.chain_node_handles.emplace_back(prev_node_handle);
}

void TreeStructor::Unlink(const BranchHandle child_handle, const BranchHandle parent_handle) {
  auto& child_branch = operating_branches[child_handle];
  auto& parent_branch = operating_branches[parent_handle];
  // Establish relationship
  child_branch.parent_handle = -1;
  child_branch.used = false;

  for (int i = 0; i < parent_branch.child_handles.size(); i++) {
    if (child_handle == parent_branch.child_handles[i]) {
      parent_branch.child_handles[i] = parent_branch.child_handles.back();
      parent_branch.child_handles.pop_back();
      break;
    }
  }
}

void TreeStructor::Link(const BranchHandle child_handle, const BranchHandle parent_handle) {
  auto& child_branch = operating_branches[child_handle];
  auto& parent_branch = operating_branches[parent_handle];
  // Establish relationship
  child_branch.parent_handle = parent_handle;
  child_branch.used = true;
  parent_branch.child_handles.emplace_back(child_handle);
}

void TreeStructor::GetSortedBranchList(BranchHandle branch_handle, std::vector<BranchHandle>& list) {
  const auto child_handles = operating_branches[branch_handle].child_handles;
  list.push_back(branch_handle);
  for (const auto& child_handle : child_handles) {
    GetSortedBranchList(child_handle, list);
  }
}

void TreeStructor::ConnectBranches(const BranchHandle branch_handle) {
  const auto child_handles = operating_branches[branch_handle].child_handles;
  for (const auto& child_handle : child_handles) {
    // Connect branches.
    SkeletonNodeHandle prev_node_handle = -1;
    BuildConnectionBranch(child_handle, prev_node_handle);
    auto& child_branch = operating_branches[child_handle];
    const float chain_length = child_branch.bezier_curve.GetLength();
    const int chain_amount = glm::max(2, static_cast<int>(chain_length / reconstruction_settings.internode_length));
    auto& skeleton = skeletons[child_branch.skeleton_index];
    for (int i = 1; i < chain_amount; i++) {
      prev_node_handle = skeleton.Extend(prev_node_handle, false);
      child_branch.chain_node_handles.emplace_back(prev_node_handle);
    }
    ApplyCurve(child_branch);
  }
  for (const auto& child_handle : child_handles) {
    ConnectBranches(child_handle);
  }
}

void TreeStructor::ImportGraph(const std::filesystem::path& path, float import_scale_factor) {
  if (!std::filesystem::exists(path)) {
    EVOENGINE_ERROR("Not exist!");
    return;
  }
  try {
    std::ifstream stream(path.string());
    std::stringstream string_stream;
    string_stream << stream.rdbuf();
    YAML::Node in = YAML::Load(string_stream.str());

    const auto& tree = in["Tree"];
    if (tree["Scatter Points"]) {
      const auto& scatter_points = tree["Scatter Points"];
      scattered_points.resize(scatter_points.size());

      for (int i = 0; i < scatter_points.size(); i++) {
        auto& point = scattered_points[i];
        point.position = scatter_points[i].as<glm::vec3>() * import_scale_factor;

        point.handle = i;
        point.neighbor_scatter_points.clear();
      }
    }

    const auto& in_tree_parts = tree["Tree Parts"];

    predicted_branches.clear();
    operating_branches.clear();
    tree_parts.clear();
    allocated_points.clear();
    skeletons.clear();
    scattered_point_to_branch_end_connections.clear();
    scattered_point_to_branch_start_connections.clear();
    scattered_points_connections.clear();
    candidate_branch_connections.clear();
    reversed_candidate_branch_connections.clear();
    filtered_branch_connections.clear();
    branch_connections.clear();
    min = glm::vec3(FLT_MAX);
    max = glm::vec3(FLT_MIN);
    float min_height = 999.0f;
    for (int i = 0; i < in_tree_parts.size(); i++) {
      const auto& in_tree_part = in_tree_parts[i];

      TreePart tree_part = {};
      tree_part.handle = tree_parts.size();
      try {
        if (in_tree_part["Color"])
          tree_part.color = in_tree_part["Color"].as<glm::vec3>() / 255.0f;
      } catch (const std::exception& e) {
        EVOENGINE_ERROR("Color is wrong at node " + std::to_string(i) + ": " + std::string(e.what()));
      }
      if (in_tree_part["foliage"])
        tree_part.foliage = in_tree_part["foliage"].as<float>();
      int branch_size = 0;
      for (const auto& in_branch : in_tree_part["Branches"]) {
        auto branch_start = in_branch["Start Pos"].as<glm::vec3>() * import_scale_factor;
        auto branch_end = in_branch["End Pos"].as<glm::vec3>() * import_scale_factor;
        auto start_dir = in_branch["Start Dir"].as<glm::vec3>();
        auto end_dir = in_branch["End Dir"].as<glm::vec3>();

        auto start_radius = in_branch["Start Radius"].as<float>() * import_scale_factor;
        auto end_radius = in_branch["End Radius"].as<float>() * import_scale_factor;
        if (branch_start == branch_end || glm::any(glm::isnan(start_dir)) || glm::any(glm::isnan(end_dir)) ||
            start_radius == 0.f || end_radius == 0.f) {
          continue;
        }
        branch_size++;
        auto& branch = predicted_branches.emplace_back();
        branch.foliage = tree_part.foliage;

        branch.bezier_curve.p0 = branch_start;
        branch.bezier_curve.p3 = branch_end;
        if (glm::distance(branch_start, branch_end) > 0.3f) {
          EVOENGINE_WARNING("Too long internode!")
        }
        branch.color = tree_part.color;
        auto c_p_length = glm::distance(branch.bezier_curve.p0, branch.bezier_curve.p3) * 0.3f;
        branch.bezier_curve.p1 = glm::normalize(start_dir) * c_p_length + branch.bezier_curve.p0;
        branch.bezier_curve.p2 = branch.bezier_curve.p3 - glm::normalize(end_dir) * c_p_length;
        if (glm::any(glm::isnan(branch.bezier_curve.p1))) {
          branch.bezier_curve.p1 = glm::mix(branch.bezier_curve.p0, branch.bezier_curve.p3, 0.25f);
        }
        if (glm::any(glm::isnan(branch.bezier_curve.p2))) {
          branch.bezier_curve.p2 = glm::mix(branch.bezier_curve.p0, branch.bezier_curve.p3, 0.75f);
        }
        branch.start_thickness = start_radius;
        branch.end_thickness = end_radius;
        branch.handle = predicted_branches.size() - 1;
        tree_part.branch_handles.emplace_back(branch.handle);
        branch.tree_part_handle = tree_part.handle;
        min_height = glm::min(min_height, branch.bezier_curve.p0.y);
        min_height = glm::min(min_height, branch.bezier_curve.p3.y);
      }
      if (branch_size == 0)
        continue;
      // auto& treePart = tree_parts.emplace_back();
      tree_parts.emplace_back(tree_part);
      for (const auto& in_allocated_point : in_tree_part["Allocated Points"]) {
        auto& allocated_point = allocated_points.emplace_back();
        allocated_point.color = tree_part.color;
        allocated_point.position = in_allocated_point.as<glm::vec3>() * import_scale_factor;
        allocated_point.handle = allocated_points.size() - 1;
        allocated_point.tree_part_handle = tree_part.handle;
        allocated_point.branch_handle = -1;
        tree_part.allocated_points.emplace_back(allocated_point.handle);
      }
    }
    for (auto& scatter_point : scattered_points) {
      scatter_point.position.y -= min_height;

      min = glm::min(min, scatter_point.position);
      max = glm::max(max, scatter_point.position);
    }
    for (auto& predicted_branch : predicted_branches) {
      predicted_branch.bezier_curve.p0.y -= min_height;
      predicted_branch.bezier_curve.p1.y -= min_height;
      predicted_branch.bezier_curve.p2.y -= min_height;
      predicted_branch.bezier_curve.p3.y -= min_height;
    }
    for (auto& allocated_point : allocated_points) {
      allocated_point.position.y -= min_height;

      min = glm::min(min, allocated_point.position);
      max = glm::max(max, allocated_point.position);

      const auto& tree_part = tree_parts[allocated_point.tree_part_handle];
      std::map<float, BranchHandle> distances;
      for (const auto& branch_handle : tree_part.branch_handles) {
        const auto& branch = predicted_branches[branch_handle];
        const auto distance0 = glm::distance(allocated_point.position, branch.bezier_curve.p0);
        const auto distance3 = glm::distance(allocated_point.position, branch.bezier_curve.p3);
        distances[distance0] = branch_handle;
        distances[distance3] = branch_handle;
      }
      allocated_point.branch_handle = distances.begin()->second;
      predicted_branches[allocated_point.branch_handle].allocated_points.emplace_back(allocated_point.handle);
    }
    for (auto& predicted_branch : predicted_branches) {
      min = glm::min(min, predicted_branch.bezier_curve.p0);
      max = glm::max(max, predicted_branch.bezier_curve.p0);
      min = glm::min(min, predicted_branch.bezier_curve.p3);
      max = glm::max(max, predicted_branch.bezier_curve.p3);

      if (!predicted_branch.allocated_points.empty()) {
        const auto& origin = predicted_branch.bezier_curve.p0;
        const auto normal = glm::normalize(predicted_branch.bezier_curve.p3 - origin);
        const auto x_axis = glm::vec3(normal.y, normal.z, normal.x);
        const auto y_axis = glm::vec3(normal.z, normal.x, normal.y);
        auto position_avg = glm::vec2(0.0f);
        for (const auto& point_handle : predicted_branch.allocated_points) {
          auto& point = allocated_points[point_handle];
          const auto v = predicted_branch.bezier_curve.p0 - point.position;
          const auto d = glm::dot(v, normal);
          const auto p = v + d * normal;
          const auto x = glm::distance(origin, glm::closestPointOnLine(p, origin, origin + 10.0f * x_axis));
          const auto y = glm::distance(origin, glm::closestPointOnLine(p, origin, origin + 10.0f * y_axis));
          point.plane_position = glm::vec2(x, y);
          position_avg += point.plane_position;
        }
        position_avg /= predicted_branch.allocated_points.size();
        auto distance_avg = 0.0f;
        for (const auto& point_handle : predicted_branch.allocated_points) {
          auto& point = allocated_points[point_handle];
          point.plane_position -= position_avg;
          distance_avg += glm::length(point.plane_position);
        }
        distance_avg /= predicted_branch.allocated_points.size();
        // predictedBranch.final_thickness = distanceAvg * 2.0f;
      } else {
        // predictedBranch.final_thickness = (predictedBranch.start_thickness + predictedBranch.end_thickness) *
        // 0.5f;
      }
      predicted_branch.final_thickness = 0.f;
    }

    auto center = (min + max) / 2.0f;
    auto new_min = center + (min - center) * 1.25f;
    auto new_max = center + (max - center) * 1.25f;
    min = new_min;
    max = new_max;

    BuildVoxelGrid();
  } catch (const std::exception& e) {
    EVOENGINE_ERROR("Failed to load: " + std::string(e.what()))
  }
}

void TreeStructor::ExportForestObj(const TreeMeshGeneratorSettings& mesh_generator_settings,
                                   const std::filesystem::path& path) {
  if (path.extension() == ".obj") {
    std::ofstream of;
    of.open(path.string(), std::ofstream::out | std::ofstream::trunc);
    if (of.is_open()) {
      std::string start = "#Forest OBJ exporter, by Bosheng Li";
      start += "\n";
      of.write(start.c_str(), start.size());
      of.flush();
      unsigned start_index = 1;
      if (const auto branch_meshes = GenerateForestBranchMeshes(mesh_generator_settings); !branch_meshes.empty()) {
        unsigned tree_index = 0;
        for (auto& mesh : branch_meshes) {
          auto& vertices = mesh->UnsafeGetVertices();
          auto& triangles = mesh->UnsafeGetTriangles();
          if (!vertices.empty() && !triangles.empty()) {
            std::string header =
                "#Vertices: " + std::to_string(vertices.size()) + ", tris: " + std::to_string(triangles.size());
            header += "\n";
            of.write(header.c_str(), header.size());
            of.flush();
            std::stringstream data;
            data << "o tree " + std::to_string(tree_index) + "\n";
#pragma region Data collection
            for (auto i = 0; i < vertices.size(); i++) {
              auto& vertex_position = vertices.at(i).position;
              auto& color = vertices.at(i).color;
              data << "v " + std::to_string(vertex_position.x) + " " + std::to_string(vertex_position.y) + " " +
                          std::to_string(vertex_position.z) + " " + std::to_string(color.x) + " " +
                          std::to_string(color.y) + " " + std::to_string(color.z) + "\n";
            }
            for (const auto& vertex : vertices) {
              data << "vt " + std::to_string(vertex.tex_coord.x) + " " + std::to_string(vertex.tex_coord.y) + "\n";
            }
            // data += "s off\n";
            data << "# List of indices for faces vertices, with (x, y, z).\n";
            for (auto i = 0; i < triangles.size(); i++) {
              const auto triangle = triangles[i];
              const auto f1 = triangle.x + start_index;
              const auto f2 = triangle.y + start_index;
              const auto f3 = triangle.z + start_index;
              data << "f " + std::to_string(f1) + "/" + std::to_string(f1) + "/" + std::to_string(f1) + " " +
                          std::to_string(f2) + "/" + std::to_string(f2) + "/" + std::to_string(f2) + " " +
                          std::to_string(f3) + "/" + std::to_string(f3) + "/" + std::to_string(f3) + "\n";
            }
#pragma endregion
            const auto result = data.str();
            of.write(result.c_str(), result.size());
            of.flush();
            start_index += vertices.size();
            tree_index++;
          }
        }
      }
      if (mesh_generator_settings.enable_foliage) {
        if (const auto foliage_meshes = GenerateFoliageMeshes(); !foliage_meshes.empty()) {
          unsigned tree_index = 0;
          for (auto& mesh : foliage_meshes) {
            auto& vertices = mesh->UnsafeGetVertices();
            auto& triangles = mesh->UnsafeGetTriangles();
            if (!vertices.empty() && !triangles.empty()) {
              std::string header =
                  "#Vertices: " + std::to_string(vertices.size()) + ", tris: " + std::to_string(triangles.size());
              header += "\n";
              of.write(header.c_str(), header.size());
              of.flush();
              std::stringstream data;
              data << "o tree " + std::to_string(tree_index) + "\n";
#pragma region Data collection
              for (auto i = 0; i < vertices.size(); i++) {
                auto& vertex_position = vertices.at(i).position;
                auto& color = vertices.at(i).color;
                data << "v " + std::to_string(vertex_position.x) + " " + std::to_string(vertex_position.y) + " " +
                            std::to_string(vertex_position.z) + " " + std::to_string(color.x) + " " +
                            std::to_string(color.y) + " " + std::to_string(color.z) + "\n";
              }
              for (const auto& vertex : vertices) {
                data << "vt " + std::to_string(vertex.tex_coord.x) + " " + std::to_string(vertex.tex_coord.y) + "\n";
              }
              // data += "s off\n";
              data << "# List of indices for faces vertices, with (x, y, z).\n";
              for (auto i = 0; i < triangles.size(); i++) {
                const auto triangle = triangles[i];
                const auto f1 = triangle.x + start_index;
                const auto f2 = triangle.y + start_index;
                const auto f3 = triangle.z + start_index;
                data << "f " + std::to_string(f1) + "/" + std::to_string(f1) + "/" + std::to_string(f1) + " " +
                            std::to_string(f2) + "/" + std::to_string(f2) + "/" + std::to_string(f2) + " " +
                            std::to_string(f3) + "/" + std::to_string(f3) + "/" + std::to_string(f3) + "\n";
              }
#pragma endregion
              const auto result = data.str();
              of.write(result.c_str(), result.size());
              of.flush();
              start_index += vertices.size();
              tree_index++;
            }
          }
        }
      }
      of.close();
    }
  }
}

bool TreeStructor::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  static Handle previous_handle = 0;
  bool changed = false;

  static std::vector<glm::vec3> scattered_point_connections_starts;
  static std::vector<glm::vec3> scattered_point_connections_ends;
  static std::vector<glm::vec4> scattered_point_connection_colors;

  static std::vector<glm::vec3> candidate_branch_connection_starts;
  static std::vector<glm::vec3> candidate_branch_connection_ends;
  static std::vector<glm::vec4> candidate_branch_connection_colors;

  static std::vector<glm::vec3> reversed_candidate_branch_connection_starts;
  static std::vector<glm::vec3> reversed_candidate_branch_connection_ends;
  static std::vector<glm::vec4> reversed_candidate_branch_connection_colors;

  static std::vector<glm::vec3> filtered_branch_connection_starts;
  static std::vector<glm::vec3> filtered_branch_connection_ends;
  static std::vector<glm::vec4> filtered_branch_connection_colors;

  static std::vector<glm::vec3> selected_branch_connection_starts;
  static std::vector<glm::vec3> selected_branch_connection_ends;
  static std::vector<glm::vec4> selected_branch_connection_colors;

  static std::vector<glm::vec3> scatter_point_to_branch_connection_starts;
  static std::vector<glm::vec3> scatter_point_to_branch_connection_ends;
  static std::vector<glm::vec4> scatter_point_to_branch_connection_colors;

  static std::vector<glm::vec3> predicted_branch_starts;
  static std::vector<glm::vec3> predicted_branch_ends;
  static std::vector<glm::vec4> predicted_branch_colors;
  static std::vector<float> predicted_branch_widths;

  if (!allocated_point_info_list)
    allocated_point_info_list = AssetManager::CreateTemporaryAsset<ParticleInfoList>();
  if (!scattered_point_info_list)
    scattered_point_info_list = AssetManager::CreateTemporaryAsset<ParticleInfoList>();
  if (!scattered_point_connection_info_list)
    scattered_point_connection_info_list = AssetManager::CreateTemporaryAsset<ParticleInfoList>();

  if (!candidate_branch_connection_info_list)
    candidate_branch_connection_info_list = AssetManager::CreateTemporaryAsset<ParticleInfoList>();
  if (!reversed_candidate_branch_connection_info_list)
    reversed_candidate_branch_connection_info_list = AssetManager::CreateTemporaryAsset<ParticleInfoList>();
  if (!filtered_branch_connection_info_list)
    filtered_branch_connection_info_list = AssetManager::CreateTemporaryAsset<ParticleInfoList>();
  if (!selected_branch_connection_info_list)
    selected_branch_connection_info_list = AssetManager::CreateTemporaryAsset<ParticleInfoList>();

  if (!scatter_point_to_branch_connection_info_list)
    scatter_point_to_branch_connection_info_list = AssetManager::CreateTemporaryAsset<ParticleInfoList>();
  if (!selected_branch_info_list)
    selected_branch_info_list = AssetManager::CreateTemporaryAsset<ParticleInfoList>();

  static std::vector<ParticleInfo> allocated_point_matrices;
  static std::vector<ParticleInfo> scatter_point_matrices;

  static bool enable_debug_rendering = true;

  static bool use_real_branch_width = true;
  static float predicted_branch_width = 0.005f;
  static float connection_width = 0.001f;
  static float point_size = 1.f;

  bool refresh_data = false;

  static int color_mode = 0;

  static float import_scale = 0.1f;
  editor_layer->DragAndDropButton<TreeDescriptor>(tree_descriptor_ref, "TreeDescriptor", true);

  ImGui::DragFloat("Import scale", &import_scale, 0.01f, 0.01f, 10.0f);
  FileUtils::OpenFile(
      "Load YAML", "YAML", {".yml"},
      [&](const std::filesystem::path& path) {
        ImportGraph(path, import_scale);
        refresh_data = true;
      },
      false);

  if (!tree_parts.empty()) {
    if (ImGui::TreeNodeEx("Graph Settings")) {
      connectivity_graph_settings.OnInspect();
      if (ImGui::Button("Rebuild Voxel Grid")) {
        BuildVoxelGrid();
      }
      ImGui::TreePop();
    }
    if (ImGui::TreeNodeEx("Reconstruction Settings")) {
      reconstruction_settings.OnInspect();
      ImGui::TreePop();
    }
    if (ImGui::Button("Build Skeletons")) {
      EstablishConnectivityGraph();
      BuildSkeletons();
      refresh_data = true;
    }
    if (ImGui::Button("Form forest")) {
      if (branch_connections.empty()) {
        skeletons.clear();
        EstablishConnectivityGraph();
        refresh_data = true;
      }
      if (skeletons.empty()) {
        BuildSkeletons();
        refresh_data = true;
      }
      GenerateForest();
    }
    ImGui::SameLine();
    if (ImGui::Button("Clear forest")) {
      ClearForest();
    }
    ImGui::Separator();
    const auto eco_sys_lab_layer = Application::GetLayer<EcoSysLabLayer>();
    FileUtils::SaveFile(
        "Export all forest as OBJ", "OBJ", {".obj"},
        [&](const std::filesystem::path& path) {
          ExportForestObj(eco_sys_lab_layer->mesh_generator_settings, path);
        },
        false);
  }

  ImGui::Checkbox("Debug Rendering", &enable_debug_rendering);
  if (enable_debug_rendering) {
    static GizmoSettings gizmo_settings{};
    if (ImGui::TreeNode("Debug rendering settings")) {
      if (ImGui::Combo("Color mode", {"TreePart", "Branch", "Node"}, color_mode))
        refresh_data = true;
      ImGui::Checkbox("Use skeleton width", &use_real_branch_width);
      if (!use_real_branch_width)
        if (ImGui::DragFloat("Branch width", &predicted_branch_width, 0.0001f, 0.0001f, 1.0f, "%.4f"))
          refresh_data = true;
      if (ImGui::DragFloat("Connection width", &connection_width, 0.0001f, 0.0001f, 1.0f, "%.4f"))
        refresh_data = true;
      if (ImGui::DragFloat("Point size", &point_size, 0.0001f, 0.0001f, 1.0f, "%.4f"))
        refresh_data = true;
      if (ImGui::Checkbox("Allocated points", &debug_allocated_points))
        refresh_data = true;
      if (ImGui::Checkbox("Scattered points", &debug_scattered_points))
        refresh_data = true;
      if (debug_scattered_points) {
        if (ImGui::ColorEdit4("Scatter Point color", &scatter_point_color.x))
          refresh_data = true;
        if (ImGui::Checkbox("Render Point-Point links", &debug_scattered_point_connections))
          refresh_data = true;
        if (ImGui::Checkbox("Render Point-Branch links", &debug_scatter_point_to_branch_connections))
          refresh_data = true;
        if (debug_scattered_point_connections &&
            ImGui::ColorEdit4("Point-Point links color", &scattered_point_connection_color.x))
          refresh_data = true;
        if (debug_scatter_point_to_branch_connections &&
            ImGui::ColorEdit4("Point-Branch links color", &scatter_point_to_branch_connection_color.x))
          refresh_data = true;
      }

      if (ImGui::Checkbox("Candidate connections", &debug_candidate_connections))
        refresh_data = true;
      if (debug_candidate_connections &&
          ImGui::ColorEdit4("Candidate connection color", &candidate_branch_connection_color.x))
        refresh_data = true;
      if (ImGui::Checkbox("Reversed candidate connections", &debug_reversed_candidate_connections))
        refresh_data = true;
      if (debug_reversed_candidate_connections &&
          ImGui::ColorEdit4("Reversed candidate connection color", &reversed_candidate_branch_connection_color.x))
        refresh_data = true;
      if (ImGui::Checkbox("Filtered connections", &debug_filtered_connections))
        refresh_data = true;
      if (debug_filtered_connections &&
          ImGui::ColorEdit4("Filtered Connection Color", &filtered_branch_connection_color.x))
        refresh_data = true;
      if (ImGui::Checkbox("Selected Branch connections", &debug_selected_branch_connections))
        refresh_data = true;
      if (debug_selected_branch_connections &&
          ImGui::ColorEdit4("Branch Connection Color", &selected_branch_connection_color.x))
        refresh_data = true;
      if (ImGui::Checkbox("Selected branches", &debug_selected_branches))
        refresh_data = true;

      gizmo_settings.draw_settings.OnInspect();

      ImGui::TreePop();
    }

    if (ImGui::Button("Refresh Data")) {
      refresh_data = true;
    }

    if (GetHandle() != previous_handle)
      refresh_data = true;

    if (refresh_data) {
      previous_handle = GetHandle();
      const auto eco_sys_lab_layer = Application::GetLayer<EcoSysLabLayer>();

      allocated_point_matrices.resize(allocated_points.size());

      predicted_branch_starts.resize(predicted_branches.size());
      predicted_branch_ends.resize(predicted_branches.size());
      predicted_branch_colors.resize(predicted_branches.size());
      predicted_branch_widths.resize(predicted_branches.size());
      switch (color_mode) {
        case 0: {
          // TreePart
          for (int i = 0; i < allocated_points.size(); i++) {
            allocated_point_matrices[i].instance_matrix.value =
                glm::translate(allocated_points[i].position) * glm::scale(glm::vec3(0.003f));
            allocated_point_matrices[i].instance_color = glm::vec4(allocated_points[i].color, 1.0f);
          }

          for (int i = 0; i < predicted_branches.size(); i++) {
            predicted_branch_starts[i] = predicted_branches[i].bezier_curve.p0;
            predicted_branch_ends[i] = predicted_branches[i].bezier_curve.p3;
            predicted_branch_colors[i] = glm::vec4(predicted_branches[i].color, 1.0f);
            if (use_real_branch_width)
              predicted_branch_widths[i] = predicted_branches[i].final_thickness;
            else
              predicted_branch_widths[i] = predicted_branch_width;
          }
          selected_branch_info_list->ApplyConnections(predicted_branch_starts, predicted_branch_ends,
                                                      predicted_branch_colors, predicted_branch_widths);

        } break;
        case 1: {
          // Branch
          for (int i = 0; i < allocated_points.size(); i++) {
            allocated_point_matrices[i].instance_matrix.value =
                glm::translate(allocated_points[i].position) * glm::scale(glm::vec3(0.003f));
            if (allocated_points[i].branch_handle >= 0) {
              allocated_point_matrices[i].instance_color =
                  glm::vec4(eco_sys_lab_layer->RandomColors()[allocated_points[i].branch_handle], 1.0f);
            } else {
              allocated_point_matrices[i].instance_color =
                  glm::vec4(eco_sys_lab_layer->RandomColors()[allocated_points[i].tree_part_handle], 1.0f);
            }
          }

          for (int i = 0; i < predicted_branches.size(); i++) {
            predicted_branch_starts[i] = predicted_branches[i].bezier_curve.p0;
            predicted_branch_ends[i] = predicted_branches[i].bezier_curve.p3;
            predicted_branch_colors[i] =
                glm::vec4(eco_sys_lab_layer->RandomColors()[predicted_branches[i].handle], 1.0f);
            if (use_real_branch_width)
              predicted_branch_widths[i] = predicted_branches[i].final_thickness;
            else
              predicted_branch_widths[i] = predicted_branch_width;
          }
          selected_branch_info_list->ApplyConnections(predicted_branch_starts, predicted_branch_ends,
                                                      predicted_branch_colors, predicted_branch_widths);

        } break;
        case 2: {
          // Node
          for (int i = 0; i < allocated_points.size(); i++) {
            allocated_point_matrices[i].instance_matrix.value =
                glm::translate(allocated_points[i].position) * glm::scale(glm::vec3(0.003f));
            if (allocated_points[i].node_handle >= 0) {
              allocated_point_matrices[i].instance_color =
                  glm::vec4(eco_sys_lab_layer->RandomColors()[allocated_points[i].node_handle], 1.0f);
            } else {
              allocated_point_matrices[i].instance_color =
                  glm::vec4(eco_sys_lab_layer->RandomColors()[allocated_points[i].tree_part_handle], 1.0f);
            }
          }

          for (int i = 0; i < predicted_branches.size(); i++) {
            predicted_branch_starts[i] = predicted_branches[i].bezier_curve.p0;
            predicted_branch_ends[i] = predicted_branches[i].bezier_curve.p3;
            predicted_branch_colors[i] = glm::vec4(1.0f);
            if (use_real_branch_width)
              predicted_branch_widths[i] = predicted_branches[i].final_thickness;
            else
              predicted_branch_widths[i] = predicted_branch_width;
          }
          selected_branch_info_list->ApplyConnections(predicted_branch_starts, predicted_branch_ends,
                                                      predicted_branch_colors, predicted_branch_widths);

        } break;
      }

      scatter_point_matrices.resize(scattered_points.size());
      for (int i = 0; i < scattered_points.size(); i++) {
        scatter_point_matrices[i].instance_matrix.value =
            glm::translate(scattered_points[i].position) * glm::scale(glm::vec3(0.004f));
        scatter_point_matrices[i].instance_color = scatter_point_color;
      }

      scattered_point_connections_starts.resize(scattered_points_connections.size());
      scattered_point_connections_ends.resize(scattered_points_connections.size());
      scattered_point_connection_colors.resize(scattered_points_connections.size());
      for (int i = 0; i < scattered_points_connections.size(); i++) {
        scattered_point_connections_starts[i] = scattered_points_connections[i].first;
        scattered_point_connections_ends[i] = scattered_points_connections[i].second;
        scattered_point_connection_colors[i] = scatter_point_to_branch_connection_color;
      }
      scattered_point_connection_info_list->ApplyConnections(scattered_point_connections_starts,
                                                             scattered_point_connections_ends,
                                                             scattered_point_connection_colors, connection_width);

      candidate_branch_connection_starts.resize(candidate_branch_connections.size());
      candidate_branch_connection_ends.resize(candidate_branch_connections.size());
      candidate_branch_connection_colors.resize(candidate_branch_connections.size());
      for (int i = 0; i < candidate_branch_connections.size(); i++) {
        candidate_branch_connection_starts[i] = candidate_branch_connections[i].first;
        candidate_branch_connection_ends[i] = candidate_branch_connections[i].second;
        candidate_branch_connection_colors[i] = candidate_branch_connection_color;
      }

      candidate_branch_connection_info_list->ApplyConnections(candidate_branch_connection_starts,
                                                              candidate_branch_connection_ends,
                                                              candidate_branch_connection_colors, connection_width);

      reversed_candidate_branch_connection_starts.resize(reversed_candidate_branch_connections.size());
      reversed_candidate_branch_connection_ends.resize(reversed_candidate_branch_connections.size());
      reversed_candidate_branch_connection_colors.resize(reversed_candidate_branch_connections.size());
      for (int i = 0; i < reversed_candidate_branch_connections.size(); i++) {
        reversed_candidate_branch_connection_starts[i] = reversed_candidate_branch_connections[i].first;
        reversed_candidate_branch_connection_ends[i] = reversed_candidate_branch_connections[i].second;
        reversed_candidate_branch_connection_colors[i] = reversed_candidate_branch_connection_color;
      }

      reversed_candidate_branch_connection_info_list->ApplyConnections(
          reversed_candidate_branch_connection_starts, reversed_candidate_branch_connection_ends,
          reversed_candidate_branch_connection_colors, connection_width);

      filtered_branch_connection_starts.resize(filtered_branch_connections.size());
      filtered_branch_connection_ends.resize(filtered_branch_connections.size());
      filtered_branch_connection_colors.resize(filtered_branch_connections.size());
      for (int i = 0; i < filtered_branch_connections.size(); i++) {
        filtered_branch_connection_starts[i] = filtered_branch_connections[i].first;
        filtered_branch_connection_ends[i] = filtered_branch_connections[i].second;
        filtered_branch_connection_colors[i] = filtered_branch_connection_color;
      }
      filtered_branch_connection_info_list->ApplyConnections(
          filtered_branch_connection_starts, filtered_branch_connection_ends, filtered_branch_connection_colors,
          connection_width * 1.1f);

      selected_branch_connection_starts.resize(branch_connections.size());
      selected_branch_connection_ends.resize(branch_connections.size());
      selected_branch_connection_colors.resize(branch_connections.size());
      for (int i = 0; i < branch_connections.size(); i++) {
        selected_branch_connection_starts[i] = branch_connections[i].first;
        selected_branch_connection_ends[i] = branch_connections[i].second;
        selected_branch_connection_colors[i] = selected_branch_connection_color;
      }
      selected_branch_connection_info_list->ApplyConnections(
          selected_branch_connection_starts, selected_branch_connection_ends, selected_branch_connection_colors,
          connection_width * 1.2f);

      scatter_point_to_branch_connection_starts.resize(scattered_point_to_branch_start_connections.size() +
                                                       scattered_point_to_branch_end_connections.size());
      scatter_point_to_branch_connection_ends.resize(scattered_point_to_branch_start_connections.size() +
                                                     scattered_point_to_branch_end_connections.size());
      scatter_point_to_branch_connection_colors.resize(scattered_point_to_branch_start_connections.size() +
                                                       scattered_point_to_branch_end_connections.size());
      for (int i = 0; i < scattered_point_to_branch_start_connections.size(); i++) {
        scatter_point_to_branch_connection_starts[i] = scattered_point_to_branch_start_connections[i].first;
        scatter_point_to_branch_connection_ends[i] = scattered_point_to_branch_start_connections[i].second;
        scatter_point_to_branch_connection_colors[i] = scatter_point_to_branch_connection_color;
      }
      for (int i = scattered_point_to_branch_start_connections.size();
           i < scattered_point_to_branch_start_connections.size() + scattered_point_to_branch_end_connections.size();
           i++) {
        scatter_point_to_branch_connection_starts[i] =
            scattered_point_to_branch_end_connections[i - scattered_point_to_branch_start_connections.size()].first;
        scatter_point_to_branch_connection_ends[i] =
            scattered_point_to_branch_end_connections[i - scattered_point_to_branch_start_connections.size()].second;
      }
      scatter_point_to_branch_connection_info_list->ApplyConnections(
          scatter_point_to_branch_connection_starts, scatter_point_to_branch_connection_ends,
          scatter_point_to_branch_connection_colors, connection_width);

      allocated_point_info_list->SetParticleInfos(allocated_point_matrices);
      scattered_point_info_list->SetParticleInfos(scatter_point_matrices);
    }
    if (debug_scattered_points) {
      editor_layer->DrawGizmoMeshInstancedColored(Resources::TryGetResource<Mesh>("PRIMITIVE_CUBE"),
                                                  scattered_point_info_list, glm::mat4(1.0f), point_size,
                                                  gizmo_settings);
    }
    if (debug_allocated_points) {
      editor_layer->DrawGizmoMeshInstancedColored(Resources::TryGetResource<Mesh>("PRIMITIVE_CUBE"),
                                                  allocated_point_info_list, glm::mat4(1.0f), point_size,
                                                  gizmo_settings);
    }
    if (debug_selected_branches)
      editor_layer->DrawGizmoMeshInstancedColored(Resources::TryGetResource<Mesh>("PRIMITIVE_CONE"),
                                                  selected_branch_info_list, glm::mat4(1.0f), 1.0f, gizmo_settings);
    if (debug_scattered_point_connections)
      editor_layer->DrawGizmoMeshInstancedColored(Resources::TryGetResource<Mesh>("PRIMITIVE_CYLINDER"),
                                                  scattered_point_connection_info_list, glm::mat4(1.0f), 1.0f,
                                                  gizmo_settings);

    if (debug_candidate_connections)
      editor_layer->DrawGizmoMeshInstancedColored(Resources::TryGetResource<Mesh>("PRIMITIVE_CONE"),
                                                  candidate_branch_connection_info_list, glm::mat4(1.0f), 1.0f,
                                                  gizmo_settings);

    if (debug_reversed_candidate_connections)
      editor_layer->DrawGizmoMeshInstancedColored(Resources::TryGetResource<Mesh>("PRIMITIVE_CYLINDER"),
                                                  reversed_candidate_branch_connection_info_list, glm::mat4(1.0f), 1.0f,
                                                  gizmo_settings);

    if (debug_filtered_connections)
      editor_layer->DrawGizmoMeshInstancedColored(Resources::TryGetResource<Mesh>("PRIMITIVE_CYLINDER"),
                                                  filtered_branch_connection_info_list, glm::mat4(1.0f), 1.0f,
                                                  gizmo_settings);
    if (debug_selected_branch_connections)
      editor_layer->DrawGizmoMeshInstancedColored(Resources::TryGetResource<Mesh>("PRIMITIVE_CYLINDER"),
                                                  selected_branch_connection_info_list, glm::mat4(1.0f), 1.0f,
                                                  gizmo_settings);

    if (debug_scatter_point_to_branch_connections)
      editor_layer->DrawGizmoMeshInstancedColored(Resources::TryGetResource<Mesh>("PRIMITIVE_CYLINDER"),
                                                  scatter_point_to_branch_connection_info_list, glm::mat4(1.0f), 1.0f,
                                                  gizmo_settings);
  }

  if (ImGui::TreeNode("Info settings")) {
    ImGui::Checkbox("Allocated points", &enable_allocated_points);
    ImGui::Checkbox("Scattered points", &enable_scattered_points);
    ImGui::Checkbox("Scatter-Branch connections", &enable_scatter_point_to_branch_connections);
    ImGui::Checkbox("Candidate connections", &enable_candidate_branch_connections);
    ImGui::Checkbox("Filtered branch connections", &enable_filtered_branch_connections);
    ImGui::Checkbox("Selected branch connections", &enable_selected_branch_connections);
    ImGui::Checkbox("Selected branches", &enable_selected_branches);
    ImGui::TreePop();
  }
  if (ImGui::Button("Build Info")) {
    FormInfoEntities();
  }

  return changed;
}

void TreeStructor::FormInfoEntities() const {
  const auto scene = GetScene();
  const auto owner = GetOwner();
  const auto children = scene->GetChildren(owner);
  for (const auto& i : children) {
    if (scene->GetEntityName(i) == "Info") {
      scene->DeleteEntity(i);
    }
  }

  const auto info_entity = scene->CreateEntity("Info");
  scene->SetParent(info_entity, owner);
  if (enable_allocated_points) {
    const auto allocated_point_info_entity = scene->CreateEntity("Allocated Points");
    scene->SetParent(allocated_point_info_entity, info_entity);
    const auto particles = scene->GetOrSetPrivateComponent<Particles>(allocated_point_info_entity).lock();
    particles->mesh = Resources::TryGetResource<Mesh>("PRIMITIVE_SPHERE");
    particles->particle_info_list = allocated_point_info_list;
    const auto material = AssetManager::CreateTemporaryAsset<Material>();
    particles->material = material;
    material->material_properties.albedo_color = allocated_point_color;
  }
  if (enable_scattered_points) {
    const auto scatter_point_info_entity = scene->CreateEntity("Scattered Points");
    scene->SetParent(scatter_point_info_entity, info_entity);
    const auto particles = scene->GetOrSetPrivateComponent<Particles>(scatter_point_info_entity).lock();
    particles->mesh = Resources::TryGetResource<Mesh>("PRIMITIVE_SPHERE");
    particles->particle_info_list = scattered_point_info_list;
    const auto material = AssetManager::CreateTemporaryAsset<Material>();
    particles->material = material;
    material->material_properties.albedo_color = scatter_point_color;
  }
  if (enable_scattered_point_connections) {
    const auto scattered_point_connection_info_entity = scene->CreateEntity("Scattered Point Connections");
    scene->SetParent(scattered_point_connection_info_entity, info_entity);
    scene->SetEnable(scattered_point_connection_info_entity, false);
    const auto particles = scene->GetOrSetPrivateComponent<Particles>(scattered_point_connection_info_entity).lock();
    particles->mesh = Resources::TryGetResource<Mesh>("PRIMITIVE_CYLINDER");
    particles->particle_info_list = scattered_point_connection_info_list;
    const auto material = AssetManager::CreateTemporaryAsset<Material>();
    particles->material = material;
    material->material_properties.albedo_color = scattered_point_connection_color;
  }
  if (enable_candidate_branch_connections) {
    const auto candidate_branch_connection_info_entity = scene->CreateEntity("Candidate Branch Connections");
    scene->SetEnable(candidate_branch_connection_info_entity, false);
    scene->SetParent(candidate_branch_connection_info_entity, info_entity);
    const auto particles = scene->GetOrSetPrivateComponent<Particles>(candidate_branch_connection_info_entity).lock();
    particles->mesh = Resources::TryGetResource<Mesh>("PRIMITIVE_CYLINDER");
    particles->particle_info_list = candidate_branch_connection_info_list;
    const auto material = AssetManager::CreateTemporaryAsset<Material>();
    particles->material = material;
    material->material_properties.albedo_color = candidate_branch_connection_color;
  }
  if (enable_reversed_candidate_branch_connections) {
    const auto reversed_candidate_branch_connection_info_entity =
        scene->CreateEntity("Reversed Candidate Branch Connections");
    scene->SetEnable(reversed_candidate_branch_connection_info_entity, false);
    scene->SetParent(reversed_candidate_branch_connection_info_entity, info_entity);
    const auto particles =
        scene->GetOrSetPrivateComponent<Particles>(reversed_candidate_branch_connection_info_entity).lock();
    particles->mesh = Resources::TryGetResource<Mesh>("PRIMITIVE_CYLINDER");
    particles->particle_info_list = reversed_candidate_branch_connection_info_list;
    const auto material = AssetManager::CreateTemporaryAsset<Material>();
    particles->material = material;
    material->material_properties.albedo_color = reversed_candidate_branch_connection_color;
  }
  if (enable_filtered_branch_connections) {
    const auto filtered_branch_connection_info_entity = scene->CreateEntity("Filtered Branch Connections");
    scene->SetEnable(filtered_branch_connection_info_entity, false);
    scene->SetParent(filtered_branch_connection_info_entity, info_entity);
    const auto particles = scene->GetOrSetPrivateComponent<Particles>(filtered_branch_connection_info_entity).lock();
    particles->mesh = Resources::TryGetResource<Mesh>("PRIMITIVE_CYLINDER");
    particles->particle_info_list = filtered_branch_connection_info_list;
    const auto material = AssetManager::CreateTemporaryAsset<Material>();
    particles->material = material;
    material->material_properties.albedo_color = filtered_branch_connection_color;
  }
  if (enable_selected_branch_connections) {
    const auto branch_connection_info_entity = scene->CreateEntity("Selected Branch Connections");
    scene->SetParent(branch_connection_info_entity, info_entity);
    const auto particles = scene->GetOrSetPrivateComponent<Particles>(branch_connection_info_entity).lock();
    particles->mesh = Resources::TryGetResource<Mesh>("PRIMITIVE_CYLINDER");
    particles->particle_info_list = selected_branch_connection_info_list;
    const auto material = AssetManager::CreateTemporaryAsset<Material>();
    particles->material = material;
    material->material_properties.albedo_color = selected_branch_connection_color;
  }
  if (enable_scatter_point_to_branch_connections) {
    const auto scatter_point_to_branch_connection = scene->CreateEntity("Scatter Point To Branch Connections");
    scene->SetParent(scatter_point_to_branch_connection, info_entity);
    scene->SetEnable(scatter_point_to_branch_connection, false);
    const auto particles = scene->GetOrSetPrivateComponent<Particles>(scatter_point_to_branch_connection).lock();
    particles->mesh = Resources::TryGetResource<Mesh>("PRIMITIVE_CYLINDER");
    particles->particle_info_list = scatter_point_to_branch_connection_info_list;
    const auto material = AssetManager::CreateTemporaryAsset<Material>();
    particles->material = material;
    material->material_properties.albedo_color = scatter_point_to_branch_connection_color;
  }
  if (enable_selected_branches) {
    const auto predicted_branch_connection_info_entity = scene->CreateEntity("Selected Branches");
    scene->SetParent(predicted_branch_connection_info_entity, info_entity);
    const auto particles = scene->GetOrSetPrivateComponent<Particles>(predicted_branch_connection_info_entity).lock();
    particles->mesh = Resources::TryGetResource<Mesh>("PRIMITIVE_CYLINDER");
    particles->particle_info_list = selected_branch_info_list;
    const auto material = AssetManager::CreateTemporaryAsset<Material>();
    particles->material = material;
    material->material_properties.albedo_color = selected_branch_color;
  }
}

void TreeStructor::EstablishConnectivityGraph() {
  scattered_points_connections.clear();
  reversed_candidate_branch_connections.clear();
  candidate_branch_connections.clear();
  filtered_branch_connections.clear();
  branch_connections.clear();
  scattered_point_to_branch_start_connections.clear();
  scattered_point_to_branch_end_connections.clear();

  for (auto& point : scattered_points) {
    point.neighbor_scatter_points.clear();
    point.p3.clear();
    point.p0.clear();
  }

  for (auto& predicted_branch : predicted_branches) {
    predicted_branch.points_to_p3.clear();
    predicted_branch.p3_to_p0.clear();

    predicted_branch.points_to_p0.clear();
    predicted_branch.p3_to_p3.clear();
    predicted_branch.p0_to_p0.clear();
    predicted_branch.p0_to_p3.clear();
  }

  // We establish connection between any 2 scatter points.
  for (auto& point : scattered_points) {
    if (scattered_points_connections.size() > 1000000) {
      EVOENGINE_ERROR("Too much connections!");
      break;
    }
    if (point.position.y > connectivity_graph_settings.max_scatter_point_connection_height)
      continue;
    FindPoints(point.position, scatter_points_voxel_grid,
               connectivity_graph_settings.point_point_connection_detection_radius, [&](const PointData& voxel) {
                 if (voxel.handle == point.handle)
                   return;
                 for (const auto& neighbor : point.neighbor_scatter_points) {
                   if (voxel.handle == neighbor)
                     return;
                 }
                 auto& other_point = scattered_points[voxel.handle];
                 point.neighbor_scatter_points.emplace_back(voxel.handle);
                 other_point.neighbor_scatter_points.emplace_back(point.handle);
                 scattered_points_connections.emplace_back(point.position, other_point.position);
               });
  }

  for (auto& branch : predicted_branches) {
    const auto& p0 = branch.bezier_curve.p0;
    const auto& p3 = branch.bezier_curve.p3;
    float branch_length = glm::distance(p0, p3);
    // We find scatter points close to the branch p0.
    if (p0.y <= connectivity_graph_settings.max_scatter_point_connection_height) {
      FindPoints(
          p0, scatter_points_voxel_grid, connectivity_graph_settings.point_branch_connection_detection_radius,
          [&](const PointData& voxel) {
            {
              auto& other_point = scattered_points[voxel.handle];
              bool duplicate = false;
              for (const auto& i : other_point.p0) {
                if (i.second == branch.handle) {
                  duplicate = true;
                  break;
                }
              }
              if (!duplicate)
                other_point.p0.emplace_back(glm::distance(branch.bezier_curve.p0, voxel.position), branch.handle);
            }
            if (connectivity_graph_settings.reverse_connection) {
              bool duplicate = false;
              for (const auto& i : branch.points_to_p0) {
                if (i.second == voxel.handle) {
                  duplicate = true;
                  break;
                }
              }
              if (!duplicate)
                branch.points_to_p0.emplace_back(glm::distance(branch.bezier_curve.p0, voxel.position), voxel.handle);
            }
            scattered_point_to_branch_start_connections.emplace_back(branch.bezier_curve.p0, voxel.position);
          });
    }
    if (p3.y <= connectivity_graph_settings.max_scatter_point_connection_height) {
      // We find branch p3 close to the scatter point.
      FindPoints(
          p3, scatter_points_voxel_grid, connectivity_graph_settings.point_branch_connection_detection_radius,
          [&](const PointData& voxel) {
            auto& other_point = scattered_points[voxel.handle];
            {
              bool duplicate = false;
              for (const auto& i : branch.points_to_p3) {
                if (i.second == voxel.handle) {
                  duplicate = true;
                  break;
                }
              }
              if (!duplicate)
                branch.points_to_p3.emplace_back(glm::distance(branch.bezier_curve.p3, voxel.position), voxel.handle);
            }
            if (connectivity_graph_settings.reverse_connection) {
              bool duplicate = false;
              for (const auto& i : other_point.p3) {
                if (i.second == branch.handle) {
                  duplicate = true;
                  break;
                }
              }
              if (!duplicate)
                other_point.p3.emplace_back(glm::distance(branch.bezier_curve.p3, voxel.position), branch.handle);
            }
            scattered_point_to_branch_end_connections.emplace_back(branch.bezier_curve.p3, voxel.position);
          });
    }
    // Connect P3 from other branch to this branch's P0
    ForEachBranchEnd(p0, branch_ends_voxel_grid,
                     branch_length * connectivity_graph_settings.branch_branch_connection_max_length_range,
                     [&](const BranchEndData& voxel) {
                       if (voxel.branch_handle == branch.handle)
                         return;
                       auto& other_branch = predicted_branches[voxel.branch_handle];
                       if (!voxel.is_p0) {
                         const auto other_branch_p0 = other_branch.bezier_curve.p0;
                         const auto other_branch_p3 = other_branch.bezier_curve.p3;
                         if (DirectConnectionCheck(other_branch.bezier_curve, branch.bezier_curve, false)) {
                           const auto distance = glm::distance(other_branch_p3, p0);
                           if (distance <= glm::distance(other_branch_p0, other_branch_p3) *
                                               connectivity_graph_settings.branch_branch_connection_max_length_range) {
                             const auto search = branch.p3_to_p0.find(other_branch.handle);
                             if (search == branch.p3_to_p0.end()) {
                               branch.p3_to_p0[other_branch.handle] = distance;
                               candidate_branch_connections.emplace_back(other_branch_p3, p0);
                               if (connectivity_graph_settings.reverse_connection) {
                                 other_branch.p0_to_p3[branch.handle] = distance;
                                 // reversed_candidate_branch_connections.emplace_back(p0, otherBranchP3);
                               }
                             }
                           }
                         }
                       } else if (connectivity_graph_settings.reverse_connection) {
                         const auto other_branch_p0 = other_branch.bezier_curve.p0;
                         const auto other_branch_p3 = other_branch.bezier_curve.p3;
                         if (DirectConnectionCheck(other_branch.bezier_curve, branch.bezier_curve, true)) {
                           const auto distance = glm::distance(other_branch_p0, p0);
                           if (distance <= glm::distance(other_branch_p0, other_branch_p3) *
                                               connectivity_graph_settings.branch_branch_connection_max_length_range) {
                             const auto search = branch.p0_to_p0.find(other_branch.handle);
                             if (search == branch.p0_to_p0.end()) {
                               branch.p0_to_p0[other_branch.handle] = distance;
                               other_branch.p0_to_p0[branch.handle] = distance;
                               reversed_candidate_branch_connections.emplace_back(p0, other_branch_p0);
                             }
                           }
                         }
                       }
                     });

    // Connect P0 from other branch to this branch's P3
    ForEachBranchEnd(p3, branch_ends_voxel_grid,
                     branch_length * connectivity_graph_settings.branch_branch_connection_max_length_range,
                     [&](const BranchEndData& voxel) {
                       if (voxel.branch_handle == branch.handle)
                         return;
                       auto& other_branch = predicted_branches[voxel.branch_handle];
                       if (voxel.is_p0) {
                         const auto other_branch_p0 = other_branch.bezier_curve.p0;
                         const auto other_branch_p3 = other_branch.bezier_curve.p3;
                         if (DirectConnectionCheck(branch.bezier_curve, other_branch.bezier_curve, false)) {
                           const auto distance = glm::distance(other_branch_p0, p3);
                           if (distance <= glm::distance(other_branch_p0, other_branch_p3) *
                                               connectivity_graph_settings.branch_branch_connection_max_length_range) {
                             {
                               const auto search = other_branch.p3_to_p0.find(branch.handle);
                               if (search == other_branch.p3_to_p0.end()) {
                                 other_branch.p3_to_p0[branch.handle] = distance;
                                 candidate_branch_connections.emplace_back(other_branch_p0, p3);
                                 if (connectivity_graph_settings.reverse_connection) {
                                   branch.p0_to_p3[other_branch.handle] = distance;
                                   // reversed_candidate_branch_connections.emplace_back(p3, otherBranchP0);
                                 }
                               }
                             }
                           }
                         }
                       } else if (connectivity_graph_settings.reverse_connection) {
                         const auto other_branch_p0 = other_branch.bezier_curve.p0;
                         const auto other_branch_p3 = other_branch.bezier_curve.p3;
                         if (DirectConnectionCheck(branch.bezier_curve, other_branch.bezier_curve, true)) {
                           const auto distance = glm::distance(other_branch_p3, p3);
                           if (distance <= glm::distance(other_branch_p0, other_branch_p3) *
                                               connectivity_graph_settings.branch_branch_connection_max_length_range) {
                             const auto search = other_branch.p3_to_p3.find(branch.handle);
                             if (search == other_branch.p3_to_p3.end()) {
                               other_branch.p3_to_p3[branch.handle] = distance;
                               branch.p3_to_p3[other_branch.handle] = distance;
                               reversed_candidate_branch_connections.emplace_back(p3, other_branch_p3);
                             }
                           }
                         }
                       }
                     });
  }
  // We search branch connections via scatter points start from p0.
  for (auto& predicted_branch : predicted_branches) {
    std::unordered_set<PointHandle> visited_points;
    std::vector<PointHandle> processing_points;
    float distance_l = FLT_MAX;
    for (const auto& i : predicted_branch.points_to_p3) {
      processing_points.emplace_back(i.second);
      auto distance = glm::distance(predicted_branch.bezier_curve.p3, scattered_points[i.second].position);
      if (distance < distance_l)
        distance_l = distance;
    }
    for (const auto& i : processing_points) {
      visited_points.emplace(i);
    }
    while (!processing_points.empty()) {
      auto current_point_handle = processing_points.back();
      visited_points.emplace(current_point_handle);
      processing_points.pop_back();
      auto& current_point = scattered_points[current_point_handle];
      for (const auto& branch_info : current_point.p0) {
        if (predicted_branch.handle == branch_info.second)
          continue;
        bool skip = false;
        for (const auto& i : predicted_branch.p3_to_p0) {
          if (branch_info.second == i.first) {
            skip = true;
            break;
          }
        }
        if (skip)
          continue;
        auto& other_branch = predicted_branches[branch_info.second];
        auto p_a = predicted_branch.bezier_curve.p3;
        auto p_b = predicted_branch.bezier_curve.p0;
        auto other_pa = other_branch.bezier_curve.p3;
        auto other_pb = other_branch.bezier_curve.p0;
        const auto dot_p = glm::dot(glm::normalize(other_pb - other_pa), glm::normalize(p_b - p_a));
        if (dot_p < glm::cos(glm::radians(connectivity_graph_settings.indirect_connection_angle_limit)))
          continue;
        float distance = distance_l + branch_info.first;
        const auto search = predicted_branch.p3_to_p0.find(branch_info.second);
        if (search == predicted_branch.p3_to_p0.end() || search->second > distance) {
          predicted_branch.p3_to_p0[branch_info.second] = distance;
          candidate_branch_connections.emplace_back(p_a, other_pb);
        }
      }

      for (const auto& neighbor_handle : current_point.neighbor_scatter_points) {
        if (visited_points.find(neighbor_handle) == visited_points.end())
          processing_points.emplace_back(neighbor_handle);
      }
    }
  }
}

void TreeStructor::BuildSkeletons() {
  skeletons.clear();
  std::unordered_set<BranchHandle> allocated_branch_handles;
  filtered_branch_connections.clear();
  branch_connections.clear();
  operating_branches.resize(predicted_branches.size());
  for (int i = 0; i < predicted_branches.size(); i++) {
    CloneOperatingBranch(reconstruction_settings, operating_branches[i], predicted_branches[i]);
  }
  std::vector<std::pair<glm::vec3, BranchHandle>> root_branch_handles{};
  std::unordered_set<BranchHandle> root_branch_handle_set{};
  // Branch is shortened after this.
  for (auto& branch : operating_branches) {
    branch.chain_node_handles.clear();
    branch.root_distance = 0.0f;
    branch.distance_to_parent_branch = 0.0f;
    branch.best_distance = FLT_MAX;
    const auto branch_start = branch.bezier_curve.p0;
    auto shortened_p0 = branch.bezier_curve.GetPoint(reconstruction_settings.branch_shortening);
    auto shortened_p3 = branch.bezier_curve.GetPoint(1.0f - reconstruction_settings.branch_shortening);
    auto shortened_length = glm::distance(shortened_p0, shortened_p3);
    if (branch_start.y <= reconstruction_settings.min_height) {
      // branch.bezier_curve.p0.y = 0.0f;
      root_branch_handles.emplace_back(branch.bezier_curve.p0, branch.handle);
      root_branch_handle_set.emplace(branch.handle);
    } else {
      branch.bezier_curve.p0 = shortened_p0;
    }
    branch.bezier_curve.p3 = shortened_p3;
    branch.bezier_curve.p1 =
        branch.bezier_curve.p0 +
        branch.bezier_curve.GetAxis(reconstruction_settings.branch_shortening) * shortened_length * 0.25f;
    branch.bezier_curve.p2 =
        branch.bezier_curve.p3 -
        branch.bezier_curve.GetAxis(1.0f - reconstruction_settings.branch_shortening) * shortened_length * 0.25f;
  }

  bool branch_removed = true;
  while (branch_removed) {
    branch_removed = false;
    std::vector<BranchHandle> remove_list{};
    for (int i = 0; i < operating_branches.size(); i++) {
      if (root_branch_handle_set.find(i) != root_branch_handle_set.end())
        continue;
      auto& operating_branch = operating_branches[i];
      if (!operating_branch.orphan && operating_branch.parent_candidates.empty()) {
        operating_branch.orphan = true;
        remove_list.emplace_back(i);
      }
    }
    if (!remove_list.empty()) {
      branch_removed = true;
      for (auto& operating_branch : operating_branches) {
        for (int i = 0; i < operating_branch.parent_candidates.size(); i++) {
          for (const auto& remove_handle : remove_list) {
            if (operating_branch.parent_candidates[i].first == remove_handle) {
              operating_branch.parent_candidates.erase(operating_branch.parent_candidates.begin() + i);
              i--;
              break;
            }
          }
        }
      }
    }
  }
  for (auto& operating_branch : operating_branches) {
    if (operating_branch.orphan)
      continue;
    for (const auto& parent_candidate : operating_branch.parent_candidates) {
      const auto& parent_branch = operating_branches[parent_candidate.first];
      if (parent_branch.orphan)
        continue;
      filtered_branch_connections.emplace_back(operating_branch.bezier_curve.p0, parent_branch.bezier_curve.p3);
    }
  }

  for (const auto& root_branch_handle : root_branch_handles) {
    auto& skeleton = skeletons.emplace_back();
    skeleton.data.root_position = root_branch_handle.first;
    auto& processing_branch = operating_branches[root_branch_handle.second];
    processing_branch.skeleton_index = skeletons.size() - 1;
    processing_branch.used = true;
    int prev_node_handle = 0;
    processing_branch.chain_node_handles.emplace_back(prev_node_handle);
    float chain_length = processing_branch.bezier_curve.GetLength();
    int chain_amount = glm::max(2, static_cast<int>(chain_length / reconstruction_settings.internode_length));
    for (int i = 1; i < chain_amount; i++) {
      prev_node_handle = skeleton.Extend(prev_node_handle, false);
      processing_branch.chain_node_handles.emplace_back(prev_node_handle);
    }
    ApplyCurve(processing_branch);
  }

  std::multimap<float, BranchHandle> height_sorted_branches{};
  for (const auto& i : operating_branches) {
    height_sorted_branches.insert({i.bezier_curve.p0.y, i.handle});
  }

  bool new_branch_allocated = true;
  while (new_branch_allocated) {
    new_branch_allocated = false;
    for (const auto& branch_pair : height_sorted_branches) {
      auto& child_branch = operating_branches[branch_pair.second];
      if (child_branch.orphan || child_branch.used || child_branch.parent_candidates.empty())
        continue;
      BranchHandle best_parent_handle = -1;
      float best_distance = FLT_MAX;
      float best_parent_distance = FLT_MAX;
      float best_root_distance = FLT_MAX;
      int max_index = -1;
      for (int i = 0; i < child_branch.parent_candidates.size(); i++) {
        const auto& parent_candidate = child_branch.parent_candidates[i];
        const auto& parent_branch = operating_branches[parent_candidate.first];
        if (!parent_branch.used || parent_branch.child_handles.size() >= reconstruction_settings.max_child_size)
          continue;
        float distance = parent_candidate.second;
        float root_distance = parent_branch.root_distance +
                              glm::distance(parent_branch.bezier_curve.p0, parent_branch.bezier_curve.p3) + distance;
        if (reconstruction_settings.use_root_distance) {
          distance = root_distance;
        }
        if (distance < best_distance) {
          best_parent_handle = parent_candidate.first;
          best_root_distance = root_distance;
          best_parent_distance = parent_candidate.second;
          max_index = i;
          best_distance = distance;
        }
      }
      if (max_index != -1) {
        child_branch.parent_candidates[max_index] = child_branch.parent_candidates.back();
        child_branch.parent_candidates.pop_back();
        new_branch_allocated = true;
        Link(branch_pair.second, best_parent_handle);
        child_branch.root_distance = best_root_distance;
        child_branch.best_distance = best_distance;
        child_branch.distance_to_parent_branch = best_parent_distance;
      }
    }
  }
  bool optimized = true;
  int iteration = 0;
  while (optimized && iteration < reconstruction_settings.optimization_timeout) {
    optimized = false;
    for (auto& child_branch : operating_branches) {
      if (child_branch.orphan || child_branch.parent_handle == -1 || child_branch.parent_candidates.empty())
        continue;
      BranchHandle best_parent_handle = -1;
      float best_distance = child_branch.best_distance;
      float best_parent_distance = FLT_MAX;
      float best_root_distance = child_branch.root_distance;
      int max_index = -1;
      for (int i = 0; i < child_branch.parent_candidates.size(); i++) {
        const auto& parent_candidate = child_branch.parent_candidates[i];
        const auto& parent_branch = operating_branches[parent_candidate.first];
        if (!parent_branch.used || parent_branch.child_handles.size() >= reconstruction_settings.max_child_size)
          continue;
        float distance = parent_candidate.second;
        float root_distance = parent_branch.root_distance +
                              glm::distance(parent_branch.bezier_curve.p0, parent_branch.bezier_curve.p3) + distance;
        if (reconstruction_settings.use_root_distance) {
          distance = root_distance;
        }
        if (distance < best_distance) {
          best_parent_handle = parent_candidate.first;
          best_root_distance = root_distance;
          best_parent_distance = parent_candidate.second;
          max_index = i;
          best_distance = distance;
        }
      }
      std::vector<BranchHandle> sub_tree_branch_list{};
      GetSortedBranchList(child_branch.handle, sub_tree_branch_list);
      if (max_index != -1) {
        for (const auto& branch_handle : sub_tree_branch_list) {
          if (branch_handle == best_parent_handle) {
            max_index = -1;
            break;
          }
        }
      }
      if (max_index != -1) {
        child_branch.parent_candidates[max_index] = child_branch.parent_candidates.back();
        child_branch.parent_candidates.pop_back();
        new_branch_allocated = true;
        Unlink(child_branch.handle, child_branch.parent_handle);
        Link(child_branch.handle, best_parent_handle);
        child_branch.root_distance = best_root_distance;
        child_branch.best_distance = best_distance;
        child_branch.distance_to_parent_branch = best_parent_distance;
        optimized = true;
      }
    }
    if (optimized) {
      CalculateBranchRootDistance(root_branch_handles);
    }
    iteration++;
  }
  CalculateBranchRootDistance(root_branch_handles);

  for (const auto& operating_branch : operating_branches) {
    if (operating_branch.parent_handle != -1) {
      branch_connections.emplace_back(predicted_branches[operating_branch.handle].bezier_curve.p0,
                                      predicted_branches[operating_branch.parent_handle].bezier_curve.p3);
    }
  }

  // Assign apical branch.
  for (const auto& root_branch_handle : root_branch_handles) {
    std::vector<BranchHandle> sorted_branch_list{};
    GetSortedBranchList(root_branch_handle.second, sorted_branch_list);
    for (auto it = sorted_branch_list.rbegin(); it != sorted_branch_list.rend(); ++it) {
      auto& operating_branch = operating_branches[*it];
      int max_descendant_size = -1;
      operating_branch.largest_child_handle = -1;
      for (const auto& child_handle : operating_branch.child_handles) {
        auto& child_branch = operating_branches[child_handle];
        operating_branch.descendant_size += child_branch.descendant_size + 1;
        if (child_branch.descendant_size >= max_descendant_size) {
          max_descendant_size = child_branch.descendant_size;
          operating_branch.largest_child_handle = child_handle;
        }
      }
      for (const auto& child_handle : operating_branch.child_handles) {
        auto& child_branch = operating_branches[child_handle];
        if (child_handle == operating_branch.largest_child_handle) {
          child_branch.apical = true;
        } else {
          child_branch.apical = false;
        }
      }
    }
  }

  // Smoothing
  for (int i = 0; i < reconstruction_settings.smooth_iteration; i++) {
    for (const auto& root_branch_handle : root_branch_handles) {
      std::vector<BranchHandle> sorted_branch_list{};
      GetSortedBranchList(root_branch_handle.second, sorted_branch_list);
      for (const auto& branch_handle : sorted_branch_list) {
        auto& branch = operating_branches[branch_handle];
        if (branch.parent_handle != -1 && branch.largest_child_handle != -1) {
          const auto& parent_branch = operating_branches[branch.parent_handle];
          if (parent_branch.largest_child_handle != branch_handle)
            continue;
          const auto& child_branch = operating_branches[branch.largest_child_handle];
          const auto parent_center = (parent_branch.bezier_curve.p0 + parent_branch.bezier_curve.p3) * 0.5f;
          const auto child_center = (child_branch.bezier_curve.p0 + child_branch.bezier_curve.p3) * 0.5f;
          const auto center = (branch.bezier_curve.p0 + branch.bezier_curve.p3) * 0.5f;
          const auto desired_center = (parent_center + child_center) * 0.5f;
          auto diff = (desired_center - center);
          branch.bezier_curve.p0 = branch.bezier_curve.p0 + diff * reconstruction_settings.position_smoothing;
          branch.bezier_curve.p1 = branch.bezier_curve.p1 + diff * reconstruction_settings.position_smoothing;
          branch.bezier_curve.p2 = branch.bezier_curve.p2 + diff * reconstruction_settings.position_smoothing;
          branch.bezier_curve.p3 = branch.bezier_curve.p3 + diff * reconstruction_settings.position_smoothing;
        }
      }
    }
    for (const auto& root_branch_handle : root_branch_handles) {
      std::vector<BranchHandle> sorted_branch_list{};
      GetSortedBranchList(root_branch_handle.second, sorted_branch_list);
      for (const auto& branch_handle : sorted_branch_list) {
        auto& branch = operating_branches[branch_handle];
        if (branch.parent_handle != -1 && branch.largest_child_handle != -1) {
          const auto& parent_branch = operating_branches[branch.parent_handle];
          const auto& child_branch = operating_branches[branch.largest_child_handle];
          const auto parent_center = (parent_branch.bezier_curve.p0 + parent_branch.bezier_curve.p3) * 0.5f;
          const auto child_center = (child_branch.bezier_curve.p0 + child_branch.bezier_curve.p3) * 0.5f;
          const auto center = (branch.bezier_curve.p0 + branch.bezier_curve.p3) * 0.5f;
          const auto length = glm::distance(branch.bezier_curve.p0, branch.bezier_curve.p3);
          // const auto currentDirection = glm::normalize(branch.bezier_curve.p3 - branch.bezier_curve.p0);

          const auto desired_direction = glm::normalize(parent_center - child_center);

          const auto desired_p0 = center + desired_direction * length * 0.5f;
          const auto desired_p3 = center - desired_direction * length * 0.5f;
          const auto desired_p1 = glm::mix(desired_p0, desired_p3, 0.25f);
          const auto desired_p2 = glm::mix(desired_p0, desired_p3, 0.75f);

          branch.bezier_curve.p0 =
              glm::mix(branch.bezier_curve.p0, desired_p0, reconstruction_settings.direction_smoothing);
          branch.bezier_curve.p1 =
              glm::mix(branch.bezier_curve.p1, desired_p1, reconstruction_settings.direction_smoothing);
          branch.bezier_curve.p2 =
              glm::mix(branch.bezier_curve.p2, desired_p2, reconstruction_settings.direction_smoothing);
          branch.bezier_curve.p3 =
              glm::mix(branch.bezier_curve.p3, desired_p3, reconstruction_settings.direction_smoothing);
        }
      }
    }
  }
  for (const auto& root_branch_handle : root_branch_handles) {
    ConnectBranches(root_branch_handle.second);
  }

  CalculateSkeletonGraphs();

  for (auto& allocated_point : allocated_points) {
    const auto& tree_part = tree_parts[allocated_point.tree_part_handle];
    float min_distance = 999.f;
    SkeletonNodeHandle closest_node_handle = -1;
    BranchHandle closest_branch_handle = -1;
    int closest_skeleton_index = -1;
    for (const auto& branch_handle : tree_part.branch_handles) {
      auto& branch = operating_branches[branch_handle];
      for (const auto& node_handle : branch.chain_node_handles) {
        auto& node = skeletons[branch.skeleton_index].RefNode(node_handle);
        auto distance = glm::distance(node.info.global_position, allocated_point.position);
        if (distance < min_distance) {
          min_distance = distance;
          closest_node_handle = node_handle;
          closest_skeleton_index = branch.skeleton_index;
          closest_branch_handle = branch_handle;
        }
      }
    }
    allocated_point.node_handle = closest_node_handle;
    allocated_point.branch_handle = closest_branch_handle;
    allocated_point.skeleton_index = closest_skeleton_index;
    if (allocated_point.skeleton_index != -1)
      skeletons[allocated_point.skeleton_index]
          .RefNode(closest_node_handle)
          .data.allocated_points.emplace_back(allocated_point.handle);
  }
  if (skeletons.size() > 1) {
    for (int i = 0; i < skeletons.size(); i++) {
      auto& skeleton = skeletons[i];
      bool remove = false;
      if (skeleton.PeekSortedNodeList().size() < reconstruction_settings.minimum_node_count) {
        remove = true;
      } else {
        for (int j = 0; j < skeletons.size(); j++) {
          if (j == i)
            continue;
          if (auto& other_skeleton = skeletons[j];
              glm::distance(skeleton.data.root_position, other_skeleton.data.root_position) <
              reconstruction_settings.minimum_tree_distance) {
            const auto& sorted_node_list = skeleton.PeekSortedNodeList();
            const auto& other_skeleton_sorted_node_list = other_skeleton.PeekSortedNodeList();
            if (sorted_node_list.size() < other_skeleton_sorted_node_list.size()) {
              remove = true;
              if (sorted_node_list.size() > reconstruction_settings.minimum_node_count) {
                std::unordered_map<SkeletonNodeHandle, SkeletonNodeHandle> node_handle_map;
                node_handle_map[0] = 0;
                for (const auto& node_handle : sorted_node_list) {
                  const auto& node = skeleton.PeekNode(node_handle);
                  SkeletonNodeHandle new_node_handle = -1;
                  if (node.GetParentHandle() == -1) {
                    continue;
                  }
                  new_node_handle = other_skeleton.Extend(node_handle_map.at(node.GetParentHandle()), !node.IsApical());
                  node_handle_map[node_handle] = new_node_handle;
                  auto& new_node = other_skeleton.RefNode(new_node_handle);
                  new_node.info = node.info;
                  new_node.data = node.data;
                }
              }
            }
          }
        }
      }

      if (remove) {
        skeletons.erase(skeletons.begin() + i);
        i--;
      }
    }
  }
  CalculateSkeletonGraphs();
  SpaceColonization();
}

Entity TreeStructor::GenerateForest() {
  const auto scene = GetScene();
  const auto owner = GetOwner();
  const auto children = scene->GetChildren(owner);
  for (const auto& i : children) {
    if (scene->GetEntityName(i) == "Forest") {
      scene->DeleteEntity(i);
    }
  }

  const auto forest_entity = scene->CreateEntity("Forest");
  scene->SetParent(forest_entity, owner);
  for (const auto& skeleton : skeletons) {
    const auto tree_entity = scene->CreateEntity("Tree");
    scene->SetParent(tree_entity, forest_entity);
    const auto tree = scene->GetOrSetPrivateComponent<Tree>(tree_entity).lock();
    tree->tree_descriptor_ref = tree_descriptor_ref;
    tree->FromSkeleton(skeleton);
    GlobalTransform gt{};
    gt.SetPosition(skeleton.data.root_position);
    scene->SetDataComponent(tree_entity, gt);
  }

  forest_ref = forest_entity;
  return forest_entity;
}

void TreeStructor::SpaceColonization() {
  if (reconstruction_settings.space_colonization_factor == 0.0f)
    return;

  Jobs::RunParallelFor(skeletons.size(), [&](unsigned i) {
    auto& skeleton = skeletons[i];
    const auto& sorted_internode_list = skeleton.PeekSortedNodeList();
    float max_end_distance = 0.0f;
    for (const auto& internode_handle : sorted_internode_list) {
      auto& internode = skeleton.RefNode(internode_handle);
      const auto distance = internode.info.end_distance + internode.info.length;
      if (internode.GetParentHandle() == -1) {
        skeleton.data.max_end_distance = distance;
        max_end_distance = distance * reconstruction_settings.space_colonization_factor;
      }
      internode.data.regrowth = distance <= max_end_distance;
    }
  });

  // Register voxel grid.
  const float removal_distance =
      reconstruction_settings.space_colonization_removal_distance_factor * reconstruction_settings.internode_length;
  const float detection_distance =
      reconstruction_settings.space_colonization_detection_distance_factor * reconstruction_settings.internode_length;

  space_colonization_voxel_grid.Initialize(removal_distance, min, max);
  for (auto& point : scattered_points) {
    PointData voxel;
    voxel.handle = -1;
    voxel.position = point.position;
    space_colonization_voxel_grid.Ref(voxel.position).emplace_back(voxel);
  }
  for (auto& point : allocated_points) {
    PointData voxel;
    voxel.handle = -1;
    voxel.position = point.position;
    space_colonization_voxel_grid.Ref(voxel.position).emplace_back(voxel);
  }

  VoxelGrid<std::vector<PointData>> internode_end_grid{};
  internode_end_grid.Initialize(detection_distance, min, max);
  for (int skeleton_index = 0; skeleton_index < skeletons.size(); skeleton_index++) {
    auto& skeleton = skeletons[skeleton_index];
    const auto& sorted_internode_list = skeleton.PeekSortedNodeList();
    for (const auto& internode_handle : sorted_internode_list) {
      auto& internode = skeleton.PeekNode(internode_handle);
      if (!internode.data.regrowth)
        continue;
      PointData voxel;
      voxel.handle = internode_handle;
      voxel.index = skeleton_index;
      voxel.position = internode.data.global_end_position;
      voxel.direction = internode.info.GetGlobalDirection();
      internode_end_grid.Ref(voxel.position).emplace_back(voxel);
    }
  }

  const auto dot_min = glm::cos(glm::radians(reconstruction_settings.space_colonization_theta));
  bool new_branch_grown = true;
  int timeout = 0;
  while (new_branch_grown && timeout < reconstruction_settings.space_colonization_timeout) {
    new_branch_grown = false;
    timeout++;
    // 1. Remove markers with occupancy zone.
    for (auto& skeleton : skeletons) {
      const auto& sorted_internode_list = skeleton.PeekSortedNodeList();
      for (const auto& internode_handle : sorted_internode_list) {
        auto& internode = skeleton.RefNode(internode_handle);
        internode.data.marker_size = 0;
        internode.data.regrow_direction = glm::vec3(0.0f);
        const auto internode_end_position = internode.data.global_end_position;
        space_colonization_voxel_grid.ForEach(internode_end_position, removal_distance,
                                              [&](std::vector<PointData>& voxels) {
                                                for (int i = 0; i < voxels.size(); i++) {
                                                  auto& marker = voxels[i];
                                                  const auto diff = marker.position - internode_end_position;
                                                  const auto distance = glm::length(diff);
                                                  if (distance < removal_distance) {
                                                    voxels[i] = voxels.back();
                                                    voxels.pop_back();
                                                    i--;
                                                  }
                                                }
                                              });
      }
    }

    // 2. Allocate markers to node with perception volume.
    for (auto& voxel : space_colonization_voxel_grid.RefData()) {
      for (auto& point : voxel) {
        point.min_distance = FLT_MAX;
        point.handle = -1;
        point.index = -1;
        point.direction = glm::vec3(0.0f);
        internode_end_grid.ForEach(point.position, detection_distance, [&](const std::vector<PointData>& voxels) {
          for (const auto& internode_end : voxels) {
            const auto diff = point.position - internode_end.position;
            const auto distance = glm::length(diff);
            const auto direction = glm::normalize(diff);
            if (distance < detection_distance && glm::dot(direction, internode_end.direction) > dot_min &&
                distance < point.min_distance) {
              point.min_distance = distance;
              point.handle = internode_end.handle;
              point.index = internode_end.index;
              point.direction = diff;
            }
          }
        });
      }
    }

    // 3. Calculate new direction for each internode.
    for (auto& voxel : space_colonization_voxel_grid.RefData()) {
      for (auto& point : voxel) {
        if (point.handle != -1) {
          auto& internode = skeletons[point.index].RefNode(point.handle);
          internode.data.marker_size++;
          internode.data.regrow_direction += point.direction;
        }
      }
    }

    // 4. Grow and add new internodes to the internodeEndGrid.
    for (int skeleton_index = 0; skeleton_index < skeletons.size(); skeleton_index++) {
      auto& skeleton = skeletons[skeleton_index];
      const auto& sorted_internode_list = skeleton.PeekSortedNodeList();
      for (const auto& internode_handle : sorted_internode_list) {
        auto& internode = skeleton.PeekNode(internode_handle);
        if (!internode.data.regrowth || internode.data.marker_size == 0)
          continue;
        if (internode.info.root_distance > skeleton.data.max_end_distance)
          continue;
        new_branch_grown = true;
        const auto new_internode_handle = skeleton.Extend(internode_handle, !internode.PeekChildHandles().empty());
        auto& old_internode = skeleton.RefNode(internode_handle);
        auto& new_internode = skeleton.RefNode(new_internode_handle);
        new_internode.info.global_position = old_internode.info.GetGlobalEndPosition();
        new_internode.info.length = reconstruction_settings.internode_length;
        new_internode.info.global_rotation =
            glm::quatLookAt(old_internode.data.regrow_direction,
                            glm::vec3(old_internode.data.regrow_direction.y, old_internode.data.regrow_direction.z,
                                      old_internode.data.regrow_direction.x));
        new_internode.data.global_end_position = old_internode.data.global_end_position +
                                                 new_internode.info.length * new_internode.info.GetGlobalDirection();
        new_internode.data.regrowth = true;
        PointData voxel;
        voxel.handle = new_internode_handle;
        voxel.index = skeleton_index;
        voxel.position = new_internode.data.global_end_position;
        voxel.direction = new_internode.info.GetGlobalDirection();
        internode_end_grid.Ref(voxel.position).emplace_back(voxel);
      }
    }
    for (auto& skeleton : skeletons) {
      skeleton.SortLists();
      skeleton.CalculateDistance();
    }
  }
  CalculateSkeletonGraphs();
}

void TreeStructor::CalculateBranchRootDistance(
    const std::vector<std::pair<glm::vec3, BranchHandle>>& root_branch_handles) {
  for (const auto& root_branch_handle : root_branch_handles) {
    std::vector<BranchHandle> sorted_branch_list{};
    GetSortedBranchList(root_branch_handle.second, sorted_branch_list);
    for (const auto branch_handle : sorted_branch_list) {
      auto& branch = operating_branches[branch_handle];
      if (branch.parent_handle == -1) {
        branch.root_distance = 0.0f;
      } else {
        const auto& parent_branch = operating_branches[branch.parent_handle];
        branch.root_distance = parent_branch.root_distance +
                               glm::distance(parent_branch.bezier_curve.p0, parent_branch.bezier_curve.p3) +
                               branch.distance_to_parent_branch;
        branch.skeleton_index = parent_branch.skeleton_index;
      }
    }
  }
}

void TreeStructor::CalculateSkeletonGraphs() {
  for (auto& skeleton : skeletons) {
    skeleton.SortLists();
    auto& sorted_node_list = skeleton.PeekSortedNodeList();
    auto& root_node = skeleton.RefNode(0);
    root_node.info.global_rotation = root_node.info.regulated_global_rotation =
        glm::quatLookAt(glm::vec3(0, 1, 0), glm::vec3(0, 0, -1));
    root_node.info.global_position = glm::vec3(0.0f);
    root_node.info.length = glm::length(root_node.data.global_end_position - root_node.data.global_start_position);
    for (const auto& node_handle : sorted_node_list) {
      auto& node = skeleton.RefNode(node_handle);
      auto& node_info = node.info;
      auto& node_data = node.data;
      if (node_handle == 0)
        continue;

      auto& parent_node = skeleton.RefNode(node.GetParentHandle());
      auto diff = node_data.global_end_position - parent_node.data.global_end_position;
      auto front = glm::normalize(diff);
      auto parent_up = parent_node.info.global_rotation * glm::vec3(0, 1, 0);
      auto regulated_up = glm::normalize(glm::cross(glm::cross(front, parent_up), front));
      node_info.global_rotation = glm::quatLookAt(front, regulated_up);
      node_info.length = glm::length(diff);
    }

    for (auto i = sorted_node_list.rbegin(); i != sorted_node_list.rend(); ++i) {
      auto& node = skeleton.RefNode(*i);
      auto& node_data = node.data;
      if (auto& child_handles = node.PeekChildHandles(); child_handles.empty()) {
        node_data.draft_thickness = reconstruction_settings.end_node_thickness;
      } else {
        float child_thickness_collection = 0.0f;
        for (const auto& child_handle : child_handles) {
          const auto& child_node = skeleton.RefNode(child_handle);
          child_thickness_collection +=
              glm::pow(child_node.data.draft_thickness + reconstruction_settings.thickness_accumulation_factor,
                       1.0f / reconstruction_settings.thickness_sum_factor);
        }
        node_data.draft_thickness = glm::pow(child_thickness_collection, reconstruction_settings.thickness_sum_factor);
      }
    }
    const auto root_node_thickness = skeleton.PeekNode(0).data.draft_thickness;
    if (reconstruction_settings.apply_root_thickness) {
      const float imported_root_thickness = skeleton.PeekNode(0).data.imported_thickness * 2.f;
      const float multiplier_factor = imported_root_thickness / root_node_thickness;
      for (const auto& handle : sorted_node_list) {
        auto& node_data = skeleton.RefNode(handle).data;
        node_data.draft_thickness *= multiplier_factor;
      }
    }
    if (root_node_thickness < reconstruction_settings.minimum_root_thickness) {
      const float multiplier_factor = reconstruction_settings.minimum_root_thickness / root_node_thickness;
      for (const auto& handle : sorted_node_list) {
        auto& node_data = skeleton.RefNode(handle).data;
        node_data.draft_thickness *= multiplier_factor;
      }
    }
    skeleton.CalculateDistance();
    for (auto i = sorted_node_list.rbegin(); i != sorted_node_list.rend(); ++i) {
      auto& node = skeleton.RefNode(*i);
      const auto& node_data = node.data;
      auto& node_info = node.info;
      if (node_info.root_distance >= reconstruction_settings.override_thickness_root_distance) {
        node_info.thickness = node_data.draft_thickness;
      }
      if (reconstruction_settings.limit_parent_thickness) {
        auto& child_handles = node.PeekChildHandles();
        float max_child_thickness = 0.0f;
        for (const auto& child_handle : child_handles) {
          max_child_thickness = glm::max(max_child_thickness, skeleton.PeekNode(child_handle).info.thickness);
        }
        node_info.thickness = glm::max(node_info.thickness, max_child_thickness);
      }

      if (node_data.branch_handle < predicted_branches.size())
        predicted_branches[node_data.branch_handle].final_thickness = node_info.thickness;
    }

    CalculateNodeTransforms(skeleton);
    skeleton.CalculateFlows();
  }
}

void TreeStructor::ClearForest() {
  const auto scene = GetScene();
  if (const auto forest_entity = forest_ref.Get(); scene->IsEntityValid(forest_entity)) {
    scene->DeleteEntity(forest_entity);
  }
  forest_ref.Clear();
}
void TreeStructor::ExportForestStatistics(const std::string& name, YAML::Emitter& out) const {
  const auto scene = GetScene();

  out << YAML::Key << name << YAML::Value << YAML::BeginSeq;
  int i = 0;
  for (const auto& skeleton : skeletons) {
    TreeStatistics tree_statistic{};
    tree_statistic.Calculate(skeleton);
    out << YAML::BeginMap;
    out << YAML::Key << "Index" << YAML::Value << i;
    tree_statistic.Serialize(out);
    out << YAML::EndMap;
    i++;
  }
  out << YAML::EndSeq;
}
void TreeStructor::ExportForestStatistics(const std::filesystem::path& path) const {
  try {
    YAML::Emitter out;
    out << YAML::BeginMap;
    ExportForestStatistics("Forest Statistics", out);
    out << YAML::EndMap;
    std::ofstream file_output(path.string());
    file_output << out.c_str();
    file_output.close();
  } catch (const std::exception& e) {
    EVOENGINE_ERROR("Failed to save: " + std::string(e.what()))
  }
}

void TreeStructor::OnCreate() {
}

std::vector<std::shared_ptr<Mesh>> TreeStructor::GenerateForestBranchMeshes(
    const TreeMeshGeneratorSettings& mesh_generator_settings) const {
  std::vector<std::shared_ptr<Mesh>> meshes{};
  for (const auto& skeleton : skeletons) {
    std::vector<Vertex> vertices;
    std::vector<unsigned int> indices;
    CylindricalMeshGenerator<ReconstructionSkeletonData, ReconstructionFlowData, ReconstructionNodeData>::Generate(
        skeleton, vertices, indices, mesh_generator_settings,
        [&](glm::vec3& vertex_position, const glm::vec3& direction, const float x_factor, const float y_factor) {
        },
        [&](glm::vec2& tex_coords, float x_factor, float y_factor) {
        });
    Jobs::RunParallelFor(vertices.size(), [&](const unsigned j) {
      vertices[j].position += skeleton.data.root_position;
    });
    auto mesh = AssetManager::CreateTemporaryAsset<Mesh>();
    VertexAttributes attributes{};
    attributes.tex_coord = true;
    mesh->SetVertices(attributes, vertices, indices);
    meshes.emplace_back(mesh);
  }
  return meshes;
}

std::vector<std::shared_ptr<Mesh>> TreeStructor::GenerateFoliageMeshes() {
  std::vector<std::shared_ptr<Mesh>> meshes{};
  for (auto& skeleton : skeletons) {
    std::vector<Vertex> vertices;
    std::vector<unsigned int> indices;

    auto quad_mesh = Resources::TryGetResource<Mesh>("PRIMITIVE_QUAD");
    auto& quad_triangles = quad_mesh->UnsafeGetTriangles();
    auto quad_vertices_size = quad_mesh->GetVerticesAmount();
    if (const auto tree_descriptor = this->tree_descriptor_ref.Get<TreeDescriptor>()) {
      size_t offset = 0;
      auto foliage_descriptor = tree_descriptor->foliage_descriptor.Get<FoliageDescriptor>();
      if (!foliage_descriptor)
        foliage_descriptor = AssetManager::CreateTemporaryAsset<FoliageDescriptor>();
      const auto& node_list = skeleton.PeekSortedNodeList();
      for (const auto& internode_handle : node_list) {
        const auto& internode = skeleton.PeekNode(internode_handle);
        const auto& internode_info = internode.info;

        if (internode_info.thickness < foliage_descriptor->max_node_thickness &&
            internode_info.root_distance > foliage_descriptor->min_root_distance &&
            internode_info.end_distance < foliage_descriptor->max_end_distance) {
          for (int i = 0; i < foliage_descriptor->leaf_count_per_internode; i++) {
            auto leaf_size = foliage_descriptor->leaf_size;
            glm::quat rotation = internode_info.GetGlobalDirection() *
                                 glm::quat(glm::radians(glm::linearRand(glm::vec3(0.0f), glm::vec3(360.0f))));
            auto front = rotation * glm::vec3(0, 0, -1);
            auto foliage_position =
                internode_info.global_position + front * (leaf_size.y * 1.5f) +
                glm::sphericalRand(1.0f) * glm::linearRand(0.0f, foliage_descriptor->position_variance);
            auto leaf_transform = glm::translate(foliage_position) * glm::mat4_cast(rotation) *
                                  glm::scale(glm::vec3(leaf_size.x, 1.0f, leaf_size.y));

            auto& matrix = leaf_transform;
            Vertex archetype;
            for (auto vertex_index = 0; vertex_index < quad_mesh->GetVerticesAmount(); vertex_index++) {
              archetype.position = matrix * glm::vec4(quad_mesh->UnsafeGetVertices()[vertex_index].position, 1.0f);
              archetype.normal = glm::normalize(
                  glm::vec3(matrix * glm::vec4(quad_mesh->UnsafeGetVertices()[vertex_index].normal, 0.0f)));
              archetype.tangent = glm::normalize(
                  glm::vec3(matrix * glm::vec4(quad_mesh->UnsafeGetVertices()[vertex_index].tangent, 0.0f)));
              archetype.tex_coord = quad_mesh->UnsafeGetVertices()[vertex_index].tex_coord;
              archetype.color = internode_info.color;
              vertices.push_back(archetype);
            }
            for (auto triangle : quad_triangles) {
              triangle.x += offset;
              triangle.y += offset;
              triangle.z += offset;
              indices.push_back(triangle.x);
              indices.push_back(triangle.y);
              indices.push_back(triangle.z);
            }
            offset += quad_vertices_size;
          }
        }
      }
    }
    Jobs::RunParallelFor(vertices.size(), [&](unsigned j) {
      vertices[j].position += skeleton.data.root_position;
    });
    auto mesh = AssetManager::CreateTemporaryAsset<Mesh>();
    VertexAttributes attributes{};
    attributes.tex_coord = true;
    mesh->SetVertices(attributes, vertices, indices);
    meshes.emplace_back(mesh);
  }
  return meshes;
}

void TreeStructor::Serialize(YAML::Emitter& out) const {
  tree_descriptor_ref.Save("tree_descriptor_ref", out);
}

void TreeStructor::Deserialize(const YAML::Node& in) {
  tree_descriptor_ref.Load("tree_descriptor_ref", in);
}

void TreeStructor::CollectAssetRef(std::vector<AssetRef>& list) {
  if (tree_descriptor_ref.Get<TreeDescriptor>())
    list.emplace_back(tree_descriptor_ref);
}
void TreeStructor::Relink(const std::unordered_map<Handle, Handle>& map, const std::shared_ptr<Scene>& scene) {
  forest_ref.Relink(map);
}

void ConnectivityGraphSettings::OnInspect() {
  // ImGui::Checkbox("Allow Reverse connections", &reverse_connection);
  if (ImGui::Button("Load reduced connection settings")) {
    point_point_connection_detection_radius = 0.05f;
    point_branch_connection_detection_radius = 0.1f;
    branch_branch_connection_max_length_range = 15.0f;
    direction_connection_angle_limit = 30.0f;
    indirect_connection_angle_limit = 15.0f;
    max_scatter_point_connection_height = 1.5f;
    parallel_shift_check = true;
  }
  if (ImGui::Button("Load default connection settings")) {
    *this = ConnectivityGraphSettings();
  }
  if (ImGui::Button("Load max connection settings")) {
    point_point_connection_detection_radius = 0.05f;
    point_branch_connection_detection_radius = 0.1f;
    branch_branch_connection_max_length_range = 10.0f;
    direction_connection_angle_limit = 90.0f;
    indirect_connection_angle_limit = 90.0f;
    max_scatter_point_connection_height = 10.f;
    parallel_shift_check = false;
  }

  ImGui::DragFloat("Point-point connection max height", &max_scatter_point_connection_height, 0.01f, 0.01f, 3.0f);
  ImGui::DragFloat("Point-point detection radius", &point_point_connection_detection_radius, 0.01f, 0.01f, 1.0f);
  ImGui::DragFloat("Point-branch detection radius", &point_branch_connection_detection_radius, 0.01f, 0.01f, 2.0f);
  ImGui::DragFloat("Branch-branch detection range", &branch_branch_connection_max_length_range, 0.01f, 0.01f, 2.0f);
  ImGui::DragFloat("Direct connection angle limit", &direction_connection_angle_limit, 0.01f, 0.0f, 180.0f);
  ImGui::DragFloat("Indirect connection angle limit", &indirect_connection_angle_limit, 0.01f, 0.0f, 180.0f);

  ImGui::Checkbox("Zigzag check", &zigzag_check);
  if (zigzag_check) {
    ImGui::DragFloat("Zigzag branch shortening", &zigzag_branch_shortening, 0.01f, 0.0f, 0.5f);
  }
  ImGui::Checkbox("Parallel shift check", &parallel_shift_check);
  if (parallel_shift_check)
    ImGui::DragFloat("Parallel Shift range limit", &parallel_shift_limit_range, 0.01f, 0.0f, 1.0f);

  ImGui::Checkbox("Point existence check", &point_existence_check);
  if (point_existence_check)
    ImGui::DragFloat("Point existence check radius", &point_existence_check_radius, 0.01f, 0.0f, 1.0f);
}

void TreeStructor::CloneOperatingBranch(const ReconstructionSettings& reconstruction_settings,
                                        OperatorBranch& operator_branch, const PredictedBranch& target) {
  operator_branch.color = target.color;
  operator_branch.tree_part_handle = target.tree_part_handle;
  operator_branch.handle = target.handle;
  operator_branch.bezier_curve = target.bezier_curve;
  operator_branch.thickness = target.final_thickness;
  operator_branch.parent_handle = -1;
  operator_branch.child_handles.clear();
  operator_branch.orphan = false;
  operator_branch.parent_candidates.clear();
  operator_branch.foliage = target.foliage;
  int count = 0;
  for (const auto& data : target.p3_to_p0) {
    operator_branch.parent_candidates.emplace_back(data.first, data.second);
    count++;
    if (count > reconstruction_settings.max_parent_candidate_size)
      break;
  }
  operator_branch.distance_to_parent_branch = 0.0f;
  operator_branch.best_distance = FLT_MAX;
  operator_branch.root_distance = 0.0f;
  operator_branch.descendant_size = 0;
  operator_branch.skeleton_index = -1;
  operator_branch.used = false;
  operator_branch.chain_node_handles.clear();
}

void ReconstructionSettings::OnInspect() {
  ImGui::DragFloat("Internode length", &internode_length, 0.01f, 0.01f, 1.0f);
  ImGui::DragFloat("Root node max height", &min_height, 0.01f, 0.01f, 1.0f);
  ImGui::DragFloat("Tree distance limit", &minimum_tree_distance, 0.01f, 0.01f, 1.0f);
  ImGui::DragFloat("Branch shortening", &branch_shortening, 0.01f, 0.01f, 0.4f);
  ImGui::DragInt("Max parent candidate size", &max_parent_candidate_size, 1, 2, 10);
  ImGui::DragInt("Max child size", &max_child_size, 1, 2, 10);

  ImGui::DragFloat("Override thickness root distance", &override_thickness_root_distance, 0.01f, 0.01f, 0.5f);
  ImGui::DragFloat("Space colonization factor", &space_colonization_factor, 0.01f, 0.f, 1.0f);
  if (space_colonization_factor > 0.0f) {
    ImGui::DragInt("Space colonization timeout", &space_colonization_timeout, 1, 0, 500);
    ImGui::DragFloat("Space colonization removal distance", &space_colonization_removal_distance_factor, 0.1f, 0.f,
                     10.0f);
    ImGui::DragFloat("Space colonization detection distance", &space_colonization_detection_distance_factor, 0.1f, 0.f,
                     20.0f);
    ImGui::DragFloat("Space colonization perception theta", &space_colonization_theta, 0.1f, 0.f, 90.0f);
  }
  ImGui::DragFloat("End node thickness", &end_node_thickness, 0.001f, 0.001f, 1.0f);
  ImGui::DragFloat("Thickness sum factor", &thickness_sum_factor, 0.01f, 0.0f, 2.0f);
  ImGui::DragFloat("Thickness accumulation factor", &thickness_accumulation_factor, 0.00001f, 0.0f, 1.0f, "%.5f");
  ImGui::Checkbox("Use imported root thickness", &apply_root_thickness);
  ImGui::Checkbox("Limit parent thickness", &limit_parent_thickness);
  ImGui::DragFloat("Minimum root thickness", &minimum_root_thickness, 0.001f, 0.0f, 1.0f, "%.3f");
  ImGui::DragInt("Minimum node count", &minimum_node_count, 1, 0, 100);

  ImGui::DragInt("Node back track limit", &node_back_track_limit, 1, 0, 100);
  ImGui::DragInt("Branch back track limit", &branch_back_track_limit, 1, 0, 10);

  ImGui::Checkbox("Use root distance", &use_root_distance);

  ImGui::DragInt("Optimization timeout", &optimization_timeout, 1, 0, 100);

  ImGui::DragFloat("Direction smoothing", &direction_smoothing, 0.01f, 0.0f, 1.0f);
  ImGui::DragFloat("Position smoothing", &position_smoothing, 0.01f, 0.0f, 1.0f);
  ImGui::DragInt("Smoothing iteration", &smooth_iteration, 1, 0, 100);

  ImGui::Checkbox("Use foliage", &use_foliage);
  /*
  ImGui::Checkbox("Candidate Search", &m_candidateSearch);
  if (m_candidateSearch) ImGui::DragInt("Candidate Search limit", &m_candidateSearchLimit, 1, 0, 10);
  ImGui::Checkbox("Force connect all branches", &m_forceConnectAllBranches);
  */
}
