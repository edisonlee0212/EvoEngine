#include "TreeStructor.hpp"
#include <unordered_set>
#include "EcoSysLabLayer.hpp"
#include "FoliageDescriptor.hpp"
#include "Graphics.hpp"
#include "rapidcsv.h"
using namespace eco_sys_lab;

void TreeStructor::ApplyCurve(const OperatorBranch& branch) {
  auto& skeleton = skeletons[branch.skeleton_index];
  const auto chainAmount = branch.chain_node_handles.size();
  for (int i = 0; i < chainAmount; i++) {
    auto& node = skeleton.RefNode(branch.chain_node_handles[i]);
    node.data.global_start_position = branch.bezier_curve.GetPoint(static_cast<float>(i) / chainAmount);
    node.data.global_end_position = branch.bezier_curve.GetPoint(static_cast<float>(i + 1) / chainAmount);
    node.info.thickness = branch.thickness;
    node.data.branch_handle = branch.handle;
    node.info.color = glm::vec4(branch.color, 1.0f);
    if(reconstruction_settings.use_foliage) node.info.leaves = branch.foliage;
    else node.info.leaves = 1.f;
  }
}

void TreeStructor::BuildVoxelGrid() {
  scatter_points_voxel_grid.Initialize(2.0f * connectivity_graph_settings.point_point_connection_detection_radius, min,
                                      max);
  allocated_points_voxel_grid.Initialize(2.0f * connectivity_graph_settings.point_point_connection_detection_radius, min,
                                        max);

  branch_ends_voxel_grid.Initialize(2.0f * connectivity_graph_settings.point_point_connection_detection_radius, min, max);
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
  for (auto& predictedBranch : predicted_branches) {
    predictedBranch.points_to_p3.clear();
    predictedBranch.p3_to_p0.clear();

    predictedBranch.points_to_p0.clear();
    predictedBranch.p3_to_p3.clear();
    predictedBranch.p0_to_p0.clear();
    predictedBranch.p0_to_p3.clear();

    BranchEndData voxel;
    voxel.branch_handle = predictedBranch.handle;
    voxel.position = predictedBranch.bezier_curve.p0;
    voxel.is_p0 = true;
    branch_ends_voxel_grid.Ref(predictedBranch.bezier_curve.p0).emplace_back(voxel);
    voxel.position = predictedBranch.bezier_curve.p3;
    voxel.is_p0 = false;
    branch_ends_voxel_grid.Ref(predictedBranch.bezier_curve.p3).emplace_back(voxel);
  }
}

bool TreeStructor::DirectConnectionCheck(const BezierCurve& parentCurve, const BezierCurve& childCurve, bool reverse) {
  const auto parentPA = parentCurve.p0;
  const auto parentPB = parentCurve.p3;
  const auto childPA = reverse ? childCurve.p3 : childCurve.p0;
  const auto childPB = reverse ? childCurve.p0 : childCurve.p3;
  const auto dotP = glm::dot(glm::normalize(parentPB - parentPA), glm::normalize(childPB - childPA));
  if (dotP < glm::cos(glm::radians(connectivity_graph_settings.direction_connection_angle_limit)))
    return false;
  if (connectivity_graph_settings.zigzag_check) {
    auto shortenedParentP0 = parentCurve.GetPoint(connectivity_graph_settings.zigzag_branch_shortening);
    auto shortenedParentP3 = parentCurve.GetPoint(1.0f - connectivity_graph_settings.zigzag_branch_shortening);
    auto shortenedChildP0 = childCurve.GetPoint(connectivity_graph_settings.zigzag_branch_shortening);
    auto shortenedChildP3 = childCurve.GetPoint(1.0f - connectivity_graph_settings.zigzag_branch_shortening);

    const auto dotC0 = glm::dot(glm::normalize(shortenedChildP3 - shortenedChildP0),
                                glm::normalize(shortenedParentP3 - shortenedChildP0));
    // const auto dotC3 = glm::dot(glm::normalize(shortenedChildP0 - shortenedChildP3), glm::normalize(shortenedParentP3
    // - shortenedChildP3));
    const auto dotP3 = glm::dot(glm::normalize(shortenedChildP0 - shortenedParentP3),
                                glm::normalize(shortenedParentP0 - shortenedParentP3));
    // const auto dotP0 = glm::dot(glm::normalize(shortenedChildP0 - shortenedParentP0),
    // glm::normalize(shortenedParentP3 - shortenedParentP0));
    if (dotC0 > 0 || dotP3 > 0 /* && dotP0 < 0*/)
      return false;
  }
  if (connectivity_graph_settings.parallel_shift_check &&
      parentPB.y > connectivity_graph_settings.parallel_shift_check_height_limit &&
      childPA.y > connectivity_graph_settings.parallel_shift_check_height_limit) {
    const auto parentDirection = glm::normalize(parentPA - parentPB);
    const auto projectedC0 =
        glm::closestPointOnLine(childPA, parentPA + 10.0f * parentDirection, parentPB - 10.0f * parentDirection);
    const auto childLength = glm::distance(childPA, childPB);
    const auto parentLength = glm::distance(parentPA, parentPB);
    const auto projectedLength = glm::distance(projectedC0, childPA);
    if (projectedLength > connectivity_graph_settings.parallel_shift_limit_range * childLength ||
        projectedLength > connectivity_graph_settings.parallel_shift_limit_range * parentLength)
      return false;
  }
  if (connectivity_graph_settings.point_existence_check &&
      connectivity_graph_settings.point_existence_check_radius > 0.0f) {
    const auto middlePoint = (childPA + parentPB) * 0.5f;
    if (!HasPoints(middlePoint, allocated_points_voxel_grid, connectivity_graph_settings.point_existence_check_radius) &&
        !HasPoints(middlePoint, scatter_points_voxel_grid, connectivity_graph_settings.point_existence_check_radius))
      return false;
  }
  return true;
}

void TreeStructor::FindPoints(const glm::vec3& position, VoxelGrid<std::vector<PointData>>& pointVoxelGrid,
                              float radius, const std::function<void(const PointData& voxel)>& func) {
  pointVoxelGrid.ForEach(position, radius, [&](const std::vector<PointData>& voxels) {
    for (const auto& voxel : voxels) {
      if (glm::distance(position, voxel.position) > radius)
        continue;
      func(voxel);
    }
  });
}

bool TreeStructor::HasPoints(const glm::vec3& position, VoxelGrid<std::vector<PointData>>& pointVoxelGrid,
                             float radius) {
  bool retVal = false;
  pointVoxelGrid.ForEach(position, radius, [&](const std::vector<PointData>& voxels) {
    if (retVal)
      return;
    for (const auto& voxel : voxels) {
      if (glm::distance(position, voxel.position) <= radius)
        retVal = true;
    }
  });
  return retVal;
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
  for (const auto& nodeHandle : skeleton.PeekSortedNodeList()) {
    auto& node = skeleton.RefNode(nodeHandle);
    auto& nodeInfo = node.info;
    auto& nodeData = node.data;
    if (node.GetParentHandle() != -1) {
      auto& parentInfo = skeleton.RefNode(node.GetParentHandle()).info;
      nodeInfo.global_position = parentInfo.global_position + parentInfo.length * parentInfo.GetGlobalDirection();
      auto parentRegulatedUp = parentInfo.regulated_global_rotation * glm::vec3(0, 1, 0);
      auto regulatedUp = glm::normalize(
          glm::cross(glm::cross(nodeInfo.GetGlobalDirection(), parentRegulatedUp), nodeInfo.GetGlobalDirection()));
      nodeInfo.regulated_global_rotation = glm::quatLookAt(nodeInfo.GetGlobalDirection(), regulatedUp);
    }
    skeleton.min = glm::min(skeleton.min, nodeInfo.global_position);
    skeleton.max = glm::max(skeleton.max, nodeInfo.global_position);
    const auto endPosition = nodeInfo.global_position + nodeInfo.length * nodeInfo.GetGlobalDirection();
    skeleton.min = glm::min(skeleton.min, endPosition);
    skeleton.max = glm::max(skeleton.max, endPosition);
  }
}

void TreeStructor::BuildConnectionBranch(const BranchHandle processingBranchHandle,
                                         SkeletonNodeHandle& prevNodeHandle) {
  operating_branches.emplace_back();
  auto& processingBranch = operating_branches[processingBranchHandle];
  auto& skeleton = skeletons[processingBranch.skeleton_index];
  auto& connectionBranch = operating_branches.back();
  auto& parentBranch = operating_branches[processingBranch.parent_handle];
  assert(parentBranch.skeleton_index == processingBranch.skeleton_index);
  connectionBranch.color = parentBranch.color;
  connectionBranch.handle = operating_branches.size() - 1;
  connectionBranch.skeleton_index = processingBranch.skeleton_index;
  connectionBranch.thickness = (parentBranch.thickness + processingBranch.thickness) * 0.5f;
  connectionBranch.child_handles.emplace_back(processingBranch.handle);
  connectionBranch.parent_handle = processingBranch.parent_handle;
  processingBranch.parent_handle = connectionBranch.handle;
  for (int& childHandle : parentBranch.child_handles) {
    if (childHandle == processingBranch.handle) {
      childHandle = connectionBranch.handle;
      break;
    }
  }

  SkeletonNodeHandle bestPrevNodeHandle = parentBranch.chain_node_handles.back();
  float dotMax = -1.0f;
  glm::vec3 connectionBranchStartPosition = parentBranch.bezier_curve.p3;
  SkeletonNodeHandle backTrackWalker = bestPrevNodeHandle;
  int branchBackTrackCount = 0;
  BranchHandle prevBranchHandle = processingBranch.parent_handle;
  for (int i = 0; i < reconstruction_settings.node_back_track_limit; i++) {
    if (backTrackWalker == -1)
      break;
    auto& node = skeleton.PeekNode(backTrackWalker);
    if (node.data.branch_handle != prevBranchHandle) {
      branchBackTrackCount++;
      prevBranchHandle = node.data.branch_handle;
    }
    if (branchBackTrackCount > reconstruction_settings.branch_back_track_limit)
      break;
    const auto nodeEndPosition = node.data.global_end_position;
    // fad
    const auto dotVal = glm::dot(glm::normalize(processingBranch.bezier_curve.p3 - processingBranch.bezier_curve.p0),
                                 glm::normalize(processingBranch.bezier_curve.p0 - nodeEndPosition));
    if (dotVal > dotMax) {
      dotMax = dotVal;
      bestPrevNodeHandle = backTrackWalker;
      connectionBranchStartPosition = nodeEndPosition;
    }
    backTrackWalker = node.GetParentHandle();
  }

  connectionBranch.bezier_curve.p0 = connectionBranchStartPosition;

  connectionBranch.bezier_curve.p3 = processingBranch.bezier_curve.p0;

  connectionBranch.bezier_curve.p1 =
      glm::mix(connectionBranch.bezier_curve.p0, connectionBranch.bezier_curve.p3, 0.25f);

  connectionBranch.bezier_curve.p2 =
      glm::mix(connectionBranch.bezier_curve.p0, connectionBranch.bezier_curve.p3, 0.75f);

  prevNodeHandle = bestPrevNodeHandle;

  auto connectionFirstNodeHandle = skeleton.Extend(prevNodeHandle, !processingBranch.apical);
  connectionBranch.chain_node_handles.emplace_back(connectionFirstNodeHandle);
  prevNodeHandle = connectionFirstNodeHandle;
  const float connectionChainLength = connectionBranch.bezier_curve.GetLength();
  int connectionChainAmount =
      glm::max(2, static_cast<int>(connectionChainLength / reconstruction_settings.internode_length));
  /*
  if(const auto treeDescriptor = tree_descriptor.Get<TreeDescriptor>())
  {
          if(const auto shootDescriptor = treeDescriptor->m_shootDescriptor.Get<ShootDescriptor>())
          {
                  connectionChainAmount = glm::max(2, static_cast<int>(connectionChainLength /
                          shootDescriptor->internode_length));
          }
  }*/
  for (int i = 1; i < connectionChainAmount; i++) {
    prevNodeHandle = skeleton.Extend(prevNodeHandle, false);
    connectionBranch.chain_node_handles.emplace_back(prevNodeHandle);
  }
  ApplyCurve(connectionBranch);
  prevNodeHandle = skeleton.Extend(prevNodeHandle, false);
  processingBranch.chain_node_handles.emplace_back(prevNodeHandle);
}

void TreeStructor::Unlink(const BranchHandle childHandle, const BranchHandle parentHandle) {
  auto& childBranch = operating_branches[childHandle];
  auto& parentBranch = operating_branches[parentHandle];
  // Establish relationship
  childBranch.parent_handle = -1;
  childBranch.used = false;

  for (int i = 0; i < parentBranch.child_handles.size(); i++) {
    if (childHandle == parentBranch.child_handles[i]) {
      parentBranch.child_handles[i] = parentBranch.child_handles.back();
      parentBranch.child_handles.pop_back();
      break;
    }
  }
}

void TreeStructor::Link(const BranchHandle childHandle, const BranchHandle parentHandle) {
  auto& childBranch = operating_branches[childHandle];
  auto& parentBranch = operating_branches[parentHandle];
  // Establish relationship
  childBranch.parent_handle = parentHandle;
  childBranch.used = true;
  parentBranch.child_handles.emplace_back(childHandle);
}

void TreeStructor::GetSortedBranchList(BranchHandle branchHandle, std::vector<BranchHandle>& list) {
  const auto childHandles = operating_branches[branchHandle].child_handles;
  list.push_back(branchHandle);
  for (const auto& childHandle : childHandles) {
    GetSortedBranchList(childHandle, list);
  }
}

void TreeStructor::ConnectBranches(const BranchHandle branchHandle) {
  const auto childHandles = operating_branches[branchHandle].child_handles;
  for (const auto& childHandle : childHandles) {
    // Connect branches.
    SkeletonNodeHandle prevNodeHandle = -1;
    BuildConnectionBranch(childHandle, prevNodeHandle);
    auto& childBranch = operating_branches[childHandle];
    const float chainLength = childBranch.bezier_curve.GetLength();
    const int chainAmount = glm::max(2, static_cast<int>(chainLength / reconstruction_settings.internode_length));
    auto& skeleton = skeletons[childBranch.skeleton_index];
    for (int i = 1; i < chainAmount; i++) {
      prevNodeHandle = skeleton.Extend(prevNodeHandle, false);
      childBranch.chain_node_handles.emplace_back(prevNodeHandle);
    }
    ApplyCurve(childBranch);
  }
  for (const auto& childHandle : childHandles) {
    ConnectBranches(childHandle);
  }
}

void TreeStructor::ImportGraph(const std::filesystem::path& path, float scaleFactor) {
  if (!std::filesystem::exists(path)) {
    EVOENGINE_ERROR("Not exist!");
    return;
  }
  try {
    std::ifstream stream(path.string());
    std::stringstream stringStream;
    stringStream << stream.rdbuf();
    YAML::Node in = YAML::Load(stringStream.str());

    const auto& tree = in["Tree"];
    if (tree["Scatter Points"]) {
      const auto& scatterPoints = tree["Scatter Points"];
      scattered_points.resize(scatterPoints.size());

      for (int i = 0; i < scatterPoints.size(); i++) {
        auto& point = scattered_points[i];
        point.position = scatterPoints[i].as<glm::vec3>() * scaleFactor;

        point.handle = i;
        point.neighbor_scatter_points.clear();
      }
    }

    const auto& treeParts = tree["Tree Parts"];

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
    float minHeight = 999.0f;
    for (int i = 0; i < treeParts.size(); i++) {
      const auto& inTreeParts = treeParts[i];

      TreePart treePart = {};
      treePart.handle = tree_parts.size();
      try {
        if (inTreeParts["Color"])
          treePart.color = inTreeParts["Color"].as<glm::vec3>() / 255.0f;
      } catch (const std::exception& e) {
        EVOENGINE_ERROR("Color is wrong at node " + std::to_string(i) + ": " + std::string(e.what()));
      }
      if(inTreeParts["foliage"]) treePart.foliage = inTreeParts["foliage"].as<float>();
      int branchSize = 0;
      for (const auto& inBranch : inTreeParts["Branches"]) {
        auto branchStart = inBranch["Start Pos"].as<glm::vec3>() * scaleFactor;
        auto branchEnd = inBranch["End Pos"].as<glm::vec3>() * scaleFactor;
        auto startDir = inBranch["Start Dir"].as<glm::vec3>();
        auto endDir = inBranch["End Dir"].as<glm::vec3>();

        auto startRadius = inBranch["Start Radius"].as<float>() * scaleFactor;
        auto endRadius = inBranch["End Radius"].as<float>() * scaleFactor;
        if (branchStart == branchEnd || glm::any(glm::isnan(startDir)) || glm::any(glm::isnan(endDir)) ||
            startRadius == 0.f || endRadius == 0.f) {
          continue;
        }
        branchSize++;
        auto& branch = predicted_branches.emplace_back();
        branch.foliage = treePart.foliage;

        branch.bezier_curve.p0 = branchStart;
        branch.bezier_curve.p3 = branchEnd;
        if (glm::distance(branchStart, branchEnd) > 0.3f) {
          EVOENGINE_WARNING("Too long internode!")
        }
        branch.color = treePart.color;
        auto cPLength = glm::distance(branch.bezier_curve.p0, branch.bezier_curve.p3) * 0.3f;
        branch.bezier_curve.p1 = glm::normalize(startDir) * cPLength + branch.bezier_curve.p0;
        branch.bezier_curve.p2 = branch.bezier_curve.p3 - glm::normalize(endDir) * cPLength;
        if (glm::any(glm::isnan(branch.bezier_curve.p1))) {
          branch.bezier_curve.p1 = glm::mix(branch.bezier_curve.p0, branch.bezier_curve.p3, 0.25f);
        }
        if (glm::any(glm::isnan(branch.bezier_curve.p2))) {
          branch.bezier_curve.p2 = glm::mix(branch.bezier_curve.p0, branch.bezier_curve.p3, 0.75f);
        }
        branch.start_thickness = startRadius;
        branch.end_thickness = endRadius;
        branch.handle = predicted_branches.size() - 1;
        treePart.branch_handles.emplace_back(branch.handle);
        branch.tree_part_handle = treePart.handle;
        minHeight = glm::min(minHeight, branch.bezier_curve.p0.y);
        minHeight = glm::min(minHeight, branch.bezier_curve.p3.y);
      }
      if (branchSize == 0)
        continue;
      // auto& treePart = tree_parts.emplace_back();
      tree_parts.emplace_back(treePart);
      for (const auto& inAllocatedPoint : inTreeParts["Allocated Points"]) {
        auto& allocatedPoint = allocated_points.emplace_back();
        allocatedPoint.color = treePart.color;
        allocatedPoint.position = inAllocatedPoint.as<glm::vec3>() * scaleFactor;
        allocatedPoint.handle = allocated_points.size() - 1;
        allocatedPoint.tree_part_handle = treePart.handle;
        allocatedPoint.branch_handle = -1;
        treePart.allocated_points.emplace_back(allocatedPoint.handle);
      }
    }
    for (auto& scatterPoint : scattered_points) {
      scatterPoint.position.y -= minHeight;

      min = glm::min(min, scatterPoint.position);
      max = glm::max(max, scatterPoint.position);
    }
    for (auto& predictedBranch : predicted_branches) {
      predictedBranch.bezier_curve.p0.y -= minHeight;
      predictedBranch.bezier_curve.p1.y -= minHeight;
      predictedBranch.bezier_curve.p2.y -= minHeight;
      predictedBranch.bezier_curve.p3.y -= minHeight;
    }
    for (auto& allocatedPoint : allocated_points) {
      allocatedPoint.position.y -= minHeight;

      min = glm::min(min, allocatedPoint.position);
      max = glm::max(max, allocatedPoint.position);

      const auto& treePart = tree_parts[allocatedPoint.tree_part_handle];
      std::map<float, BranchHandle> distances;
      for (const auto& branchHandle : treePart.branch_handles) {
        const auto& branch = predicted_branches[branchHandle];
        const auto distance0 = glm::distance(allocatedPoint.position, branch.bezier_curve.p0);
        const auto distance3 = glm::distance(allocatedPoint.position, branch.bezier_curve.p3);
        distances[distance0] = branchHandle;
        distances[distance3] = branchHandle;
      }
      allocatedPoint.branch_handle = distances.begin()->second;
      predicted_branches[allocatedPoint.branch_handle].allocated_points.emplace_back(allocatedPoint.handle);
    }
    for (auto& predictedBranch : predicted_branches) {
      min = glm::min(min, predictedBranch.bezier_curve.p0);
      max = glm::max(max, predictedBranch.bezier_curve.p0);
      min = glm::min(min, predictedBranch.bezier_curve.p3);
      max = glm::max(max, predictedBranch.bezier_curve.p3);

      if (!predictedBranch.allocated_points.empty()) {
        const auto& origin = predictedBranch.bezier_curve.p0;
        const auto normal = glm::normalize(predictedBranch.bezier_curve.p3 - origin);
        const auto xAxis = glm::vec3(normal.y, normal.z, normal.x);
        const auto yAxis = glm::vec3(normal.z, normal.x, normal.y);
        auto positionAvg = glm::vec2(0.0f);
        for (const auto& pointHandle : predictedBranch.allocated_points) {
          auto& point = allocated_points[pointHandle];
          const auto v = predictedBranch.bezier_curve.p0 - point.position;
          const auto d = glm::dot(v, normal);
          const auto p = v + d * normal;
          const auto x = glm::distance(origin, glm::closestPointOnLine(p, origin, origin + 10.0f * xAxis));
          const auto y = glm::distance(origin, glm::closestPointOnLine(p, origin, origin + 10.0f * yAxis));
          point.plane_position = glm::vec2(x, y);
          positionAvg += point.plane_position;
        }
        positionAvg /= predictedBranch.allocated_points.size();
        auto distanceAvg = 0.0f;
        for (const auto& pointHandle : predictedBranch.allocated_points) {
          auto& point = allocated_points[pointHandle];
          point.plane_position -= positionAvg;
          distanceAvg += glm::length(point.plane_position);
        }
        distanceAvg /= predictedBranch.allocated_points.size();
        // predictedBranch.final_thickness = distanceAvg * 2.0f;
      } else {
        // predictedBranch.final_thickness = (predictedBranch.start_thickness + predictedBranch.end_thickness) *
        // 0.5f;
      }
      predictedBranch.final_thickness = 0.f;
    }

    auto center = (min + max) / 2.0f;
    auto newMin = center + (min - center) * 1.25f;
    auto newMax = center + (max - center) * 1.25f;
    min = newMin;
    max = newMax;

    BuildVoxelGrid();
  } catch (std::exception e) {
    EVOENGINE_ERROR("Failed to load!");
  }
}

void TreeStructor::ExportForestOBJ(const TreeMeshGeneratorSettings& meshGeneratorSettings,
                                   const std::filesystem::path& path) {
  if (path.extension() == ".obj") {
    std::ofstream of;
    of.open(path.string(), std::ofstream::out | std::ofstream::trunc);
    if (of.is_open()) {
      std::string start = "#Forest OBJ exporter, by Bosheng Li";
      start += "\n";
      of.write(start.c_str(), start.size());
      of.flush();
      unsigned startIndex = 1;
      const auto branchMeshes = GenerateForestBranchMeshes(meshGeneratorSettings);
      if (!branchMeshes.empty()) {
        unsigned treeIndex = 0;
        for (auto& mesh : branchMeshes) {
          auto& vertices = mesh->UnsafeGetVertices();
          auto& triangles = mesh->UnsafeGetTriangles();
          if (!vertices.empty() && !triangles.empty()) {
            std::string header =
                "#Vertices: " + std::to_string(vertices.size()) + ", tris: " + std::to_string(triangles.size());
            header += "\n";
            of.write(header.c_str(), header.size());
            of.flush();
            std::stringstream data;
            data << "o tree " + std::to_string(treeIndex) + "\n";
#pragma region Data collection
            for (auto i = 0; i < vertices.size(); i++) {
              auto& vertexPosition = vertices.at(i).position;
              auto& color = vertices.at(i).color;
              data << "v " + std::to_string(vertexPosition.x) + " " + std::to_string(vertexPosition.y) + " " +
                          std::to_string(vertexPosition.z) + " " + std::to_string(color.x) + " " +
                          std::to_string(color.y) + " " + std::to_string(color.z) + "\n";
            }
            for (const auto& vertex : vertices) {
              data << "vt " + std::to_string(vertex.tex_coord.x) + " " + std::to_string(vertex.tex_coord.y) + "\n";
            }
            // data += "s off\n";
            data << "# List of indices for faces vertices, with (x, y, z).\n";
            for (auto i = 0; i < triangles.size(); i++) {
              const auto triangle = triangles[i];
              const auto f1 = triangle.x + startIndex;
              const auto f2 = triangle.y + startIndex;
              const auto f3 = triangle.z + startIndex;
              data << "f " + std::to_string(f1) + "/" + std::to_string(f1) + "/" + std::to_string(f1) + " " +
                          std::to_string(f2) + "/" + std::to_string(f2) + "/" + std::to_string(f2) + " " +
                          std::to_string(f3) + "/" + std::to_string(f3) + "/" + std::to_string(f3) + "\n";
            }
#pragma endregion
            const auto result = data.str();
            of.write(result.c_str(), result.size());
            of.flush();
            startIndex += vertices.size();
            treeIndex++;
          }
        }
      }
      if (meshGeneratorSettings.enable_foliage) {
        const auto foliageMeshes = GenerateFoliageMeshes();
        if (!foliageMeshes.empty()) {
          unsigned treeIndex = 0;
          for (auto& mesh : foliageMeshes) {
            auto& vertices = mesh->UnsafeGetVertices();
            auto& triangles = mesh->UnsafeGetTriangles();
            if (!vertices.empty() && !triangles.empty()) {
              std::string header =
                  "#Vertices: " + std::to_string(vertices.size()) + ", tris: " + std::to_string(triangles.size());
              header += "\n";
              of.write(header.c_str(), header.size());
              of.flush();
              std::stringstream data;
              data << "o tree " + std::to_string(treeIndex) + "\n";
#pragma region Data collection
              for (auto i = 0; i < vertices.size(); i++) {
                auto& vertexPosition = vertices.at(i).position;
                auto& color = vertices.at(i).color;
                data << "v " + std::to_string(vertexPosition.x) + " " + std::to_string(vertexPosition.y) + " " +
                            std::to_string(vertexPosition.z) + " " + std::to_string(color.x) + " " +
                            std::to_string(color.y) + " " + std::to_string(color.z) + "\n";
              }
              for (const auto& vertex : vertices) {
                data << "vt " + std::to_string(vertex.tex_coord.x) + " " + std::to_string(vertex.tex_coord.y) + "\n";
              }
              // data += "s off\n";
              data << "# List of indices for faces vertices, with (x, y, z).\n";
              for (auto i = 0; i < triangles.size(); i++) {
                const auto triangle = triangles[i];
                const auto f1 = triangle.x + startIndex;
                const auto f2 = triangle.y + startIndex;
                const auto f3 = triangle.z + startIndex;
                data << "f " + std::to_string(f1) + "/" + std::to_string(f1) + "/" + std::to_string(f1) + " " +
                            std::to_string(f2) + "/" + std::to_string(f2) + "/" + std::to_string(f2) + " " +
                            std::to_string(f3) + "/" + std::to_string(f3) + "/" + std::to_string(f3) + "\n";
              }
#pragma endregion
              const auto result = data.str();
              of.write(result.c_str(), result.size());
              of.flush();
              startIndex += vertices.size();
              treeIndex++;
            }
          }
        }
      }
      of.close();
    }
  }
}

bool TreeStructor::OnInspect(const std::shared_ptr<EditorLayer>& editorLayer) {
  static Handle previousHandle = 0;
  bool changed = false;

  static std::vector<glm::vec3> scatteredPointConnectionsStarts;
  static std::vector<glm::vec3> scatteredPointConnectionsEnds;
  static std::vector<glm::vec4> scatteredPointConnectionColors;

  static std::vector<glm::vec3> candidateBranchConnectionStarts;
  static std::vector<glm::vec3> candidateBranchConnectionEnds;
  static std::vector<glm::vec4> candidateBranchConnectionColors;

  static std::vector<glm::vec3> reversedCandidateBranchConnectionStarts;
  static std::vector<glm::vec3> reversedCandidateBranchConnectionEnds;
  static std::vector<glm::vec4> reversedCandidateBranchConnectionColors;

  static std::vector<glm::vec3> filteredBranchConnectionStarts;
  static std::vector<glm::vec3> filteredBranchConnectionEnds;
  static std::vector<glm::vec4> filteredBranchConnectionColors;

  static std::vector<glm::vec3> selectedBranchConnectionStarts;
  static std::vector<glm::vec3> selectedBranchConnectionEnds;
  static std::vector<glm::vec4> selectedBranchConnectionColors;

  static std::vector<glm::vec3> scatterPointToBranchConnectionStarts;
  static std::vector<glm::vec3> scatterPointToBranchConnectionEnds;
  static std::vector<glm::vec4> scatterPointToBranchConnectionColors;

  static std::vector<glm::vec3> predictedBranchStarts;
  static std::vector<glm::vec3> predictedBranchEnds;
  static std::vector<glm::vec4> predictedBranchColors;
  static std::vector<float> predictedBranchWidths;

  if (!allocated_point_info_list)
    allocated_point_info_list = ProjectManager::CreateTemporaryAsset<ParticleInfoList>();
  if (!scattered_point_info_list)
    scattered_point_info_list = ProjectManager::CreateTemporaryAsset<ParticleInfoList>();
  if (!scattered_point_connection_info_list)
    scattered_point_connection_info_list = ProjectManager::CreateTemporaryAsset<ParticleInfoList>();

  if (!candidate_branch_connection_info_list)
    candidate_branch_connection_info_list = ProjectManager::CreateTemporaryAsset<ParticleInfoList>();
  if (!reversed_candidate_branch_connection_info_list)
    reversed_candidate_branch_connection_info_list = ProjectManager::CreateTemporaryAsset<ParticleInfoList>();
  if (!filtered_branch_connection_info_list)
    filtered_branch_connection_info_list = ProjectManager::CreateTemporaryAsset<ParticleInfoList>();
  if (!selected_branch_connection_info_list)
    selected_branch_connection_info_list = ProjectManager::CreateTemporaryAsset<ParticleInfoList>();

  if (!scatter_point_to_branch_connection_info_list)
    scatter_point_to_branch_connection_info_list = ProjectManager::CreateTemporaryAsset<ParticleInfoList>();
  if (!selected_branch_info_list)
    selected_branch_info_list = ProjectManager::CreateTemporaryAsset<ParticleInfoList>();

  static std::vector<ParticleInfo> allocatedPointMatrices;
  static std::vector<ParticleInfo> scatterPointMatrices;

  static bool enableDebugRendering = true;

  static bool useRealBranchWidth = true;
  static float predictedBranchWidth = 0.005f;
  static float connectionWidth = 0.001f;
  static float pointSize = 1.f;

  bool refreshData = false;

  static int colorMode = 0;

  static float importScale = 0.1f;
  editorLayer->DragAndDropButton<TreeDescriptor>(tree_descriptor, "TreeDescriptor", true);

  ImGui::DragFloat("Import scale", &importScale, 0.01f, 0.01f, 10.0f);
  FileUtils::OpenFile(
      "Load YAML", "YAML", {".yml"},
      [&](const std::filesystem::path& path) {
        ImportGraph(path, importScale);
        refreshData = true;
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
      refreshData = true;
    }
    if (ImGui::Button("Form forest")) {
      if (branch_connections.empty()) {
        skeletons.clear();
        EstablishConnectivityGraph();
        refreshData = true;
      }
      if (skeletons.empty()) {
        BuildSkeletons();
        refreshData = true;
      }
      GenerateForest();
    }
    ImGui::SameLine();
    if (ImGui::Button("Clear forest")) {
      ClearForest();
    }
    ImGui::Separator();
    const auto ecoSysLabLayer = Application::GetLayer<EcoSysLabLayer>();
    FileUtils::SaveFile(
        "Export all forest as OBJ", "OBJ", {".obj"},
        [&](const std::filesystem::path& path) {
          ExportForestOBJ(ecoSysLabLayer->m_meshGeneratorSettings, path);
        },
        false);
  }

  ImGui::Checkbox("Debug Rendering", &enableDebugRendering);
  if (enableDebugRendering) {
    static GizmoSettings gizmoSettings{};
    if (ImGui::TreeNode("Debug rendering settings")) {
      if (ImGui::Combo("Color mode", {"TreePart", "Branch", "Node"}, colorMode))
        refreshData = true;
      ImGui::Checkbox("Use skeleton width", &useRealBranchWidth);
      if (!useRealBranchWidth)
        if (ImGui::DragFloat("Branch width", &predictedBranchWidth, 0.0001f, 0.0001f, 1.0f, "%.4f"))
          refreshData = true;
      if (ImGui::DragFloat("Connection width", &connectionWidth, 0.0001f, 0.0001f, 1.0f, "%.4f"))
        refreshData = true;
      if (ImGui::DragFloat("Point size", &pointSize, 0.0001f, 0.0001f, 1.0f, "%.4f"))
        refreshData = true;
      if (ImGui::Checkbox("Allocated points", &debug_allocated_points))
        refreshData = true;
      if (ImGui::Checkbox("Scattered points", &debug_scattered_points))
        refreshData = true;
      if (debug_scattered_points) {
        if (ImGui::ColorEdit4("Scatter Point color", &scatter_point_color.x))
          refreshData = true;
        if (ImGui::Checkbox("Render Point-Point links", &debug_scattered_point_connections))
          refreshData = true;
        if (ImGui::Checkbox("Render Point-Branch links", &debug_scatter_point_to_branch_connections))
          refreshData = true;
        if (debug_scattered_point_connections &&
            ImGui::ColorEdit4("Point-Point links color", &scattered_point_connection_color.x))
          refreshData = true;
        if (debug_scatter_point_to_branch_connections &&
            ImGui::ColorEdit4("Point-Branch links color", &scatter_point_to_branch_connection_color.x))
          refreshData = true;
      }

      if (ImGui::Checkbox("Candidate connections", &debug_candidate_connections))
        refreshData = true;
      if (debug_candidate_connections &&
          ImGui::ColorEdit4("Candidate connection color", &candidate_branch_connection_color.x))
        refreshData = true;
      if (ImGui::Checkbox("Reversed candidate connections", &debug_reversed_candidate_connections))
        refreshData = true;
      if (debug_reversed_candidate_connections &&
          ImGui::ColorEdit4("Reversed candidate connection color", &reversed_candidate_branch_connection_color.x))
        refreshData = true;
      if (ImGui::Checkbox("Filtered connections", &debug_filtered_connections))
        refreshData = true;
      if (debug_filtered_connections &&
          ImGui::ColorEdit4("Filtered Connection Color", &filtered_branch_connection_color.x))
        refreshData = true;
      if (ImGui::Checkbox("Selected Branch connections", &debug_selected_branch_connections))
        refreshData = true;
      if (debug_selected_branch_connections &&
          ImGui::ColorEdit4("Branch Connection Color", &selected_branch_connection_color.x))
        refreshData = true;
      if (ImGui::Checkbox("Selected branches", &debug_selected_branches))
        refreshData = true;

      gizmoSettings.draw_settings.OnInspect();

      ImGui::TreePop();
    }

    if (ImGui::Button("Refresh Data")) {
      refreshData = true;
    }

    if (GetHandle() != previousHandle)
      refreshData = true;

    if (refreshData) {
      previousHandle = GetHandle();
      const auto ecoSysLabLayer = Application::GetLayer<EcoSysLabLayer>();

      allocatedPointMatrices.resize(allocated_points.size());

      predictedBranchStarts.resize(predicted_branches.size());
      predictedBranchEnds.resize(predicted_branches.size());
      predictedBranchColors.resize(predicted_branches.size());
      predictedBranchWidths.resize(predicted_branches.size());
      switch (colorMode) {
        case 0: {
          // TreePart
          for (int i = 0; i < allocated_points.size(); i++) {
            allocatedPointMatrices[i].instance_matrix.value =
                glm::translate(allocated_points[i].position) * glm::scale(glm::vec3(0.003f));
            allocatedPointMatrices[i].instance_color = glm::vec4(allocated_points[i].color, 1.0f);
          }

          for (int i = 0; i < predicted_branches.size(); i++) {
            predictedBranchStarts[i] = predicted_branches[i].bezier_curve.p0;
            predictedBranchEnds[i] = predicted_branches[i].bezier_curve.p3;
            predictedBranchColors[i] = glm::vec4(predicted_branches[i].color, 1.0f);
            if (useRealBranchWidth)
              predictedBranchWidths[i] = predicted_branches[i].final_thickness;
            else
              predictedBranchWidths[i] = predictedBranchWidth;
          }
          selected_branch_info_list->ApplyConnections(predictedBranchStarts, predictedBranchEnds, predictedBranchColors,
                                                     predictedBranchWidths);

        } break;
        case 1: {
          // Branch
          for (int i = 0; i < allocated_points.size(); i++) {
            allocatedPointMatrices[i].instance_matrix.value =
                glm::translate(allocated_points[i].position) * glm::scale(glm::vec3(0.003f));
            if (allocated_points[i].branch_handle >= 0) {
              allocatedPointMatrices[i].instance_color =
                  glm::vec4(ecoSysLabLayer->RandomColors()[allocated_points[i].branch_handle], 1.0f);
            } else {
              allocatedPointMatrices[i].instance_color =
                  glm::vec4(ecoSysLabLayer->RandomColors()[allocated_points[i].tree_part_handle], 1.0f);
            }
          }

          for (int i = 0; i < predicted_branches.size(); i++) {
            predictedBranchStarts[i] = predicted_branches[i].bezier_curve.p0;
            predictedBranchEnds[i] = predicted_branches[i].bezier_curve.p3;
            predictedBranchColors[i] = glm::vec4(ecoSysLabLayer->RandomColors()[predicted_branches[i].handle], 1.0f);
            if (useRealBranchWidth)
              predictedBranchWidths[i] = predicted_branches[i].final_thickness;
            else
              predictedBranchWidths[i] = predictedBranchWidth;
          }
          selected_branch_info_list->ApplyConnections(predictedBranchStarts, predictedBranchEnds, predictedBranchColors,
                                                     predictedBranchWidths);

        } break;
        case 2: {
          // Node
          for (int i = 0; i < allocated_points.size(); i++) {
            allocatedPointMatrices[i].instance_matrix.value =
                glm::translate(allocated_points[i].position) * glm::scale(glm::vec3(0.003f));
            if (allocated_points[i].node_handle >= 0) {
              allocatedPointMatrices[i].instance_color =
                  glm::vec4(ecoSysLabLayer->RandomColors()[allocated_points[i].node_handle], 1.0f);
            } else {
              allocatedPointMatrices[i].instance_color =
                  glm::vec4(ecoSysLabLayer->RandomColors()[allocated_points[i].tree_part_handle], 1.0f);
            }
          }

          for (int i = 0; i < predicted_branches.size(); i++) {
            predictedBranchStarts[i] = predicted_branches[i].bezier_curve.p0;
            predictedBranchEnds[i] = predicted_branches[i].bezier_curve.p3;
            predictedBranchColors[i] = glm::vec4(1.0f);
            if (useRealBranchWidth)
              predictedBranchWidths[i] = predicted_branches[i].final_thickness;
            else
              predictedBranchWidths[i] = predictedBranchWidth;
          }
          selected_branch_info_list->ApplyConnections(predictedBranchStarts, predictedBranchEnds, predictedBranchColors,
                                                     predictedBranchWidths);

        } break;
      }

      scatterPointMatrices.resize(scattered_points.size());
      for (int i = 0; i < scattered_points.size(); i++) {
        scatterPointMatrices[i].instance_matrix.value =
            glm::translate(scattered_points[i].position) * glm::scale(glm::vec3(0.004f));
        scatterPointMatrices[i].instance_color = scatter_point_color;
      }

      scatteredPointConnectionsStarts.resize(scattered_points_connections.size());
      scatteredPointConnectionsEnds.resize(scattered_points_connections.size());
      scatteredPointConnectionColors.resize(scattered_points_connections.size());
      for (int i = 0; i < scattered_points_connections.size(); i++) {
        scatteredPointConnectionsStarts[i] = scattered_points_connections[i].first;
        scatteredPointConnectionsEnds[i] = scattered_points_connections[i].second;
        scatteredPointConnectionColors[i] = scatter_point_to_branch_connection_color;
      }
      scattered_point_connection_info_list->ApplyConnections(scatteredPointConnectionsStarts,
                                                           scatteredPointConnectionsEnds,
                                                           scatteredPointConnectionColors, connectionWidth);

      candidateBranchConnectionStarts.resize(candidate_branch_connections.size());
      candidateBranchConnectionEnds.resize(candidate_branch_connections.size());
      candidateBranchConnectionColors.resize(candidate_branch_connections.size());
      for (int i = 0; i < candidate_branch_connections.size(); i++) {
        candidateBranchConnectionStarts[i] = candidate_branch_connections[i].first;
        candidateBranchConnectionEnds[i] = candidate_branch_connections[i].second;
        candidateBranchConnectionColors[i] = candidate_branch_connection_color;
      }

      candidate_branch_connection_info_list->ApplyConnections(candidateBranchConnectionStarts,
                                                            candidateBranchConnectionEnds,
                                                            candidateBranchConnectionColors, connectionWidth);

      reversedCandidateBranchConnectionStarts.resize(reversed_candidate_branch_connections.size());
      reversedCandidateBranchConnectionEnds.resize(reversed_candidate_branch_connections.size());
      reversedCandidateBranchConnectionColors.resize(reversed_candidate_branch_connections.size());
      for (int i = 0; i < reversed_candidate_branch_connections.size(); i++) {
        reversedCandidateBranchConnectionStarts[i] = reversed_candidate_branch_connections[i].first;
        reversedCandidateBranchConnectionEnds[i] = reversed_candidate_branch_connections[i].second;
        reversedCandidateBranchConnectionColors[i] = reversed_candidate_branch_connection_color;
      }

      reversed_candidate_branch_connection_info_list->ApplyConnections(
          reversedCandidateBranchConnectionStarts, reversedCandidateBranchConnectionEnds,
          reversedCandidateBranchConnectionColors, connectionWidth);

      filteredBranchConnectionStarts.resize(filtered_branch_connections.size());
      filteredBranchConnectionEnds.resize(filtered_branch_connections.size());
      filteredBranchConnectionColors.resize(filtered_branch_connections.size());
      for (int i = 0; i < filtered_branch_connections.size(); i++) {
        filteredBranchConnectionStarts[i] = filtered_branch_connections[i].first;
        filteredBranchConnectionEnds[i] = filtered_branch_connections[i].second;
        filteredBranchConnectionColors[i] = filtered_branch_connection_color;
      }
      filtered_branch_connection_info_list->ApplyConnections(filteredBranchConnectionStarts, filteredBranchConnectionEnds,
                                                           filteredBranchConnectionColors, connectionWidth * 1.1f);

      selectedBranchConnectionStarts.resize(branch_connections.size());
      selectedBranchConnectionEnds.resize(branch_connections.size());
      selectedBranchConnectionColors.resize(branch_connections.size());
      for (int i = 0; i < branch_connections.size(); i++) {
        selectedBranchConnectionStarts[i] = branch_connections[i].first;
        selectedBranchConnectionEnds[i] = branch_connections[i].second;
        selectedBranchConnectionColors[i] = selected_branch_connection_color;
      }
      selected_branch_connection_info_list->ApplyConnections(selectedBranchConnectionStarts, selectedBranchConnectionEnds,
                                                           selectedBranchConnectionColors, connectionWidth * 1.2f);

      scatterPointToBranchConnectionStarts.resize(scattered_point_to_branch_start_connections.size() +
                                                  scattered_point_to_branch_end_connections.size());
      scatterPointToBranchConnectionEnds.resize(scattered_point_to_branch_start_connections.size() +
                                                scattered_point_to_branch_end_connections.size());
      scatterPointToBranchConnectionColors.resize(scattered_point_to_branch_start_connections.size() +
                                                  scattered_point_to_branch_end_connections.size());
      for (int i = 0; i < scattered_point_to_branch_start_connections.size(); i++) {
        scatterPointToBranchConnectionStarts[i] = scattered_point_to_branch_start_connections[i].first;
        scatterPointToBranchConnectionEnds[i] = scattered_point_to_branch_start_connections[i].second;
        scatterPointToBranchConnectionColors[i] = scatter_point_to_branch_connection_color;
      }
      for (int i = scattered_point_to_branch_start_connections.size();
           i < scattered_point_to_branch_start_connections.size() + scattered_point_to_branch_end_connections.size(); i++) {
        scatterPointToBranchConnectionStarts[i] =
            scattered_point_to_branch_end_connections[i - scattered_point_to_branch_start_connections.size()].first;
        scatterPointToBranchConnectionEnds[i] =
            scattered_point_to_branch_end_connections[i - scattered_point_to_branch_start_connections.size()].second;
      }
      scatter_point_to_branch_connection_info_list->ApplyConnections(scatterPointToBranchConnectionStarts,
                                                                 scatterPointToBranchConnectionEnds,
                                                                 scatterPointToBranchConnectionColors, connectionWidth);

      allocated_point_info_list->SetParticleInfos(allocatedPointMatrices);
      scattered_point_info_list->SetParticleInfos(scatterPointMatrices);
    }
    if (debug_scattered_points) {
      editorLayer->DrawGizmoMeshInstancedColored(Resources::GetResource<Mesh>("PRIMITIVE_CUBE"),
                                                 scattered_point_info_list, glm::mat4(1.0f), pointSize, gizmoSettings);
    }
    if (debug_allocated_points) {
      editorLayer->DrawGizmoMeshInstancedColored(Resources::GetResource<Mesh>("PRIMITIVE_CUBE"),
                                                 allocated_point_info_list, glm::mat4(1.0f), pointSize, gizmoSettings);
    }
    if (debug_selected_branches)
      editorLayer->DrawGizmoMeshInstancedColored(Resources::GetResource<Mesh>("PRIMITIVE_CONE"),
                                                 selected_branch_info_list, glm::mat4(1.0f), 1.0f, gizmoSettings);
    if (debug_scattered_point_connections)
      editorLayer->DrawGizmoMeshInstancedColored(Resources::GetResource<Mesh>("PRIMITIVE_CYLINDER"),
                                                 scattered_point_connection_info_list, glm::mat4(1.0f), 1.0f,
                                                 gizmoSettings);

    if (debug_candidate_connections)
      editorLayer->DrawGizmoMeshInstancedColored(Resources::GetResource<Mesh>("PRIMITIVE_CONE"),
                                                 candidate_branch_connection_info_list, glm::mat4(1.0f), 1.0f,
                                                 gizmoSettings);

    if (debug_reversed_candidate_connections)
      editorLayer->DrawGizmoMeshInstancedColored(Resources::GetResource<Mesh>("PRIMITIVE_CYLINDER"),
                                                 reversed_candidate_branch_connection_info_list, glm::mat4(1.0f), 1.0f,
                                                 gizmoSettings);

    if (debug_filtered_connections)
      editorLayer->DrawGizmoMeshInstancedColored(Resources::GetResource<Mesh>("PRIMITIVE_CYLINDER"),
                                                 filtered_branch_connection_info_list, glm::mat4(1.0f), 1.0f,
                                                 gizmoSettings);
    if (debug_selected_branch_connections)
      editorLayer->DrawGizmoMeshInstancedColored(Resources::GetResource<Mesh>("PRIMITIVE_CYLINDER"),
                                                 selected_branch_connection_info_list, glm::mat4(1.0f), 1.0f,
                                                 gizmoSettings);

    if (debug_scatter_point_to_branch_connections)
      editorLayer->DrawGizmoMeshInstancedColored(Resources::GetResource<Mesh>("PRIMITIVE_CYLINDER"),
                                                 scatter_point_to_branch_connection_info_list, glm::mat4(1.0f), 1.0f,
                                                 gizmoSettings);
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

  const auto infoEntity = scene->CreateEntity("Info");
  scene->SetParent(infoEntity, owner);
  if (enable_allocated_points) {
    const auto allocatedPointInfoEntity = scene->CreateEntity("Allocated Points");
    scene->SetParent(allocatedPointInfoEntity, infoEntity);
    const auto particles = scene->GetOrSetPrivateComponent<Particles>(allocatedPointInfoEntity).lock();
    particles->mesh = Resources::GetResource<Mesh>("PRIMITIVE_SPHERE");
    particles->particle_info_list = allocated_point_info_list;
    const auto material = ProjectManager::CreateTemporaryAsset<Material>();
    particles->material = material;
    material->material_properties.albedo_color = allocated_point_color;
  }
  if (enable_scattered_points) {
    const auto scatterPointInfoEntity = scene->CreateEntity("Scattered Points");
    scene->SetParent(scatterPointInfoEntity, infoEntity);
    const auto particles = scene->GetOrSetPrivateComponent<Particles>(scatterPointInfoEntity).lock();
    particles->mesh = Resources::GetResource<Mesh>("PRIMITIVE_SPHERE");
    particles->particle_info_list = scattered_point_info_list;
    const auto material = ProjectManager::CreateTemporaryAsset<Material>();
    particles->material = material;
    material->material_properties.albedo_color = scatter_point_color;
  }
  if (enable_scattered_point_connections) {
    const auto scatteredPointConnectionInfoEntity = scene->CreateEntity("Scattered Point Connections");
    scene->SetParent(scatteredPointConnectionInfoEntity, infoEntity);
    scene->SetEnable(scatteredPointConnectionInfoEntity, false);
    const auto particles = scene->GetOrSetPrivateComponent<Particles>(scatteredPointConnectionInfoEntity).lock();
    particles->mesh = Resources::GetResource<Mesh>("PRIMITIVE_CYLINDER");
    particles->particle_info_list = scattered_point_connection_info_list;
    const auto material = ProjectManager::CreateTemporaryAsset<Material>();
    particles->material = material;
    material->material_properties.albedo_color = scattered_point_connection_color;
  }
  if (enable_candidate_branch_connections) {
    const auto candidateBranchConnectionInfoEntity = scene->CreateEntity("Candidate Branch Connections");
    scene->SetEnable(candidateBranchConnectionInfoEntity, false);
    scene->SetParent(candidateBranchConnectionInfoEntity, infoEntity);
    const auto particles = scene->GetOrSetPrivateComponent<Particles>(candidateBranchConnectionInfoEntity).lock();
    particles->mesh = Resources::GetResource<Mesh>("PRIMITIVE_CYLINDER");
    particles->particle_info_list = candidate_branch_connection_info_list;
    const auto material = ProjectManager::CreateTemporaryAsset<Material>();
    particles->material = material;
    material->material_properties.albedo_color = candidate_branch_connection_color;
  }
  if (enable_reversed_candidate_branch_connections) {
    const auto reversedCandidateBranchConnectionInfoEntity =
        scene->CreateEntity("Reversed Candidate Branch Connections");
    scene->SetEnable(reversedCandidateBranchConnectionInfoEntity, false);
    scene->SetParent(reversedCandidateBranchConnectionInfoEntity, infoEntity);
    const auto particles =
        scene->GetOrSetPrivateComponent<Particles>(reversedCandidateBranchConnectionInfoEntity).lock();
    particles->mesh = Resources::GetResource<Mesh>("PRIMITIVE_CYLINDER");
    particles->particle_info_list = reversed_candidate_branch_connection_info_list;
    const auto material = ProjectManager::CreateTemporaryAsset<Material>();
    particles->material = material;
    material->material_properties.albedo_color = reversed_candidate_branch_connection_color;
  }
  if (enable_filtered_branch_connections) {
    const auto filteredBranchConnectionInfoEntity = scene->CreateEntity("Filtered Branch Connections");
    scene->SetEnable(filteredBranchConnectionInfoEntity, false);
    scene->SetParent(filteredBranchConnectionInfoEntity, infoEntity);
    const auto particles = scene->GetOrSetPrivateComponent<Particles>(filteredBranchConnectionInfoEntity).lock();
    particles->mesh = Resources::GetResource<Mesh>("PRIMITIVE_CYLINDER");
    particles->particle_info_list = filtered_branch_connection_info_list;
    const auto material = ProjectManager::CreateTemporaryAsset<Material>();
    particles->material = material;
    material->material_properties.albedo_color = filtered_branch_connection_color;
  }
  if (enable_selected_branch_connections) {
    const auto branchConnectionInfoEntity = scene->CreateEntity("Selected Branch Connections");
    scene->SetParent(branchConnectionInfoEntity, infoEntity);
    const auto particles = scene->GetOrSetPrivateComponent<Particles>(branchConnectionInfoEntity).lock();
    particles->mesh = Resources::GetResource<Mesh>("PRIMITIVE_CYLINDER");
    particles->particle_info_list = selected_branch_connection_info_list;
    const auto material = ProjectManager::CreateTemporaryAsset<Material>();
    particles->material = material;
    material->material_properties.albedo_color = selected_branch_connection_color;
  }
  if (enable_scatter_point_to_branch_connections) {
    const auto scatterPointToBranchConnection = scene->CreateEntity("Scatter Point To Branch Connections");
    scene->SetParent(scatterPointToBranchConnection, infoEntity);
    scene->SetEnable(scatterPointToBranchConnection, false);
    const auto particles = scene->GetOrSetPrivateComponent<Particles>(scatterPointToBranchConnection).lock();
    particles->mesh = Resources::GetResource<Mesh>("PRIMITIVE_CYLINDER");
    particles->particle_info_list = scatter_point_to_branch_connection_info_list;
    const auto material = ProjectManager::CreateTemporaryAsset<Material>();
    particles->material = material;
    material->material_properties.albedo_color = scatter_point_to_branch_connection_color;
  }
  if (enable_selected_branches) {
    const auto predictedBranchConnectionInfoEntity = scene->CreateEntity("Selected Branches");
    scene->SetParent(predictedBranchConnectionInfoEntity, infoEntity);
    const auto particles = scene->GetOrSetPrivateComponent<Particles>(predictedBranchConnectionInfoEntity).lock();
    particles->mesh = Resources::GetResource<Mesh>("PRIMITIVE_CYLINDER");
    particles->particle_info_list = selected_branch_info_list;
    const auto material = ProjectManager::CreateTemporaryAsset<Material>();
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

  for (auto& predictedBranch : predicted_branches) {
    predictedBranch.points_to_p3.clear();
    predictedBranch.p3_to_p0.clear();

    predictedBranch.points_to_p0.clear();
    predictedBranch.p3_to_p3.clear();
    predictedBranch.p0_to_p0.clear();
    predictedBranch.p0_to_p3.clear();
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
                 auto& otherPoint = scattered_points[voxel.handle];
                 point.neighbor_scatter_points.emplace_back(voxel.handle);
                 otherPoint.neighbor_scatter_points.emplace_back(point.handle);
                 scattered_points_connections.emplace_back(point.position, otherPoint.position);
               });
  }

  for (auto& branch : predicted_branches) {
    const auto& p0 = branch.bezier_curve.p0;
    const auto& p3 = branch.bezier_curve.p3;
    float branchLength = glm::distance(p0, p3);
    // We find scatter points close to the branch p0.
    if (p0.y <= connectivity_graph_settings.max_scatter_point_connection_height) {
      FindPoints(p0, scatter_points_voxel_grid, connectivity_graph_settings.point_branch_connection_detection_radius,
                 [&](const PointData& voxel) {
                   {
                     auto& otherPoint = scattered_points[voxel.handle];
                     bool duplicate = false;
                     for (const auto& i : otherPoint.p0) {
                       if (i.second == branch.handle) {
                         duplicate = true;
                         break;
                       }
                     }
                     if (!duplicate)
                       otherPoint.p0.emplace_back(glm::distance(branch.bezier_curve.p0, voxel.position),
                                                  branch.handle);
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
                       branch.points_to_p0.emplace_back(glm::distance(branch.bezier_curve.p0, voxel.position),
                                                        voxel.handle);
                   }
                   scattered_point_to_branch_start_connections.emplace_back(branch.bezier_curve.p0, voxel.position);
                 });
    }
    if (p3.y <= connectivity_graph_settings.max_scatter_point_connection_height) {
      // We find branch p3 close to the scatter point.
      FindPoints(p3, scatter_points_voxel_grid, connectivity_graph_settings.point_branch_connection_detection_radius,
                 [&](const PointData& voxel) {
                   auto& otherPoint = scattered_points[voxel.handle];
                   {
                     bool duplicate = false;
                     for (const auto& i : branch.points_to_p3) {
                       if (i.second == voxel.handle) {
                         duplicate = true;
                         break;
                       }
                     }
                     if (!duplicate)
                       branch.points_to_p3.emplace_back(glm::distance(branch.bezier_curve.p3, voxel.position),
                                                        voxel.handle);
                   }
                   if (connectivity_graph_settings.reverse_connection) {
                     bool duplicate = false;
                     for (const auto& i : otherPoint.p3) {
                       if (i.second == branch.handle) {
                         duplicate = true;
                         break;
                       }
                     }
                     if (!duplicate)
                       otherPoint.p3.emplace_back(glm::distance(branch.bezier_curve.p3, voxel.position),
                                                  branch.handle);
                   }
                   scattered_point_to_branch_end_connections.emplace_back(branch.bezier_curve.p3, voxel.position);
                 });
    }
    // Connect P3 from other branch to this branch's P0
    ForEachBranchEnd(p0, branch_ends_voxel_grid,
                     branchLength * connectivity_graph_settings.branch_branch_connection_max_length_range,
                     [&](const BranchEndData& voxel) {
                       if (voxel.branch_handle == branch.handle)
                         return;
                       auto& otherBranch = predicted_branches[voxel.branch_handle];
                       if (!voxel.is_p0) {
                         const auto otherBranchP0 = otherBranch.bezier_curve.p0;
                         const auto otherBranchP3 = otherBranch.bezier_curve.p3;
                         if (DirectConnectionCheck(otherBranch.bezier_curve, branch.bezier_curve, false)) {
                           const auto distance = glm::distance(otherBranchP3, p0);
                           if (distance <= glm::distance(otherBranchP0, otherBranchP3) *
                                               connectivity_graph_settings.branch_branch_connection_max_length_range) {
                             const auto search = branch.p3_to_p0.find(otherBranch.handle);
                             if (search == branch.p3_to_p0.end()) {
                               branch.p3_to_p0[otherBranch.handle] = distance;
                               candidate_branch_connections.emplace_back(otherBranchP3, p0);
                               if (connectivity_graph_settings.reverse_connection) {
                                 otherBranch.p0_to_p3[branch.handle] = distance;
                                 // reversed_candidate_branch_connections.emplace_back(p0, otherBranchP3);
                               }
                             }
                           }
                         }
                       } else if (connectivity_graph_settings.reverse_connection) {
                         const auto otherBranchP0 = otherBranch.bezier_curve.p0;
                         const auto otherBranchP3 = otherBranch.bezier_curve.p3;
                         if (DirectConnectionCheck(otherBranch.bezier_curve, branch.bezier_curve, true)) {
                           const auto distance = glm::distance(otherBranchP0, p0);
                           if (distance <= glm::distance(otherBranchP0, otherBranchP3) *
                                               connectivity_graph_settings.branch_branch_connection_max_length_range) {
                             const auto search = branch.p0_to_p0.find(otherBranch.handle);
                             if (search == branch.p0_to_p0.end()) {
                               branch.p0_to_p0[otherBranch.handle] = distance;
                               otherBranch.p0_to_p0[branch.handle] = distance;
                               reversed_candidate_branch_connections.emplace_back(p0, otherBranchP0);
                             }
                           }
                         }
                       }
                     });

    // Connect P0 from other branch to this branch's P3
    ForEachBranchEnd(p3, branch_ends_voxel_grid,
                     branchLength * connectivity_graph_settings.branch_branch_connection_max_length_range,
                     [&](const BranchEndData& voxel) {
                       if (voxel.branch_handle == branch.handle)
                         return;
                       auto& otherBranch = predicted_branches[voxel.branch_handle];
                       if (voxel.is_p0) {
                         const auto otherBranchP0 = otherBranch.bezier_curve.p0;
                         const auto otherBranchP3 = otherBranch.bezier_curve.p3;
                         if (DirectConnectionCheck(branch.bezier_curve, otherBranch.bezier_curve, false)) {
                           const auto distance = glm::distance(otherBranchP0, p3);
                           if (distance <= glm::distance(otherBranchP0, otherBranchP3) *
                                               connectivity_graph_settings.branch_branch_connection_max_length_range) {
                             {
                               const auto search = otherBranch.p3_to_p0.find(branch.handle);
                               if (search == otherBranch.p3_to_p0.end()) {
                                 otherBranch.p3_to_p0[branch.handle] = distance;
                                 candidate_branch_connections.emplace_back(otherBranchP0, p3);
                                 if (connectivity_graph_settings.reverse_connection) {
                                   branch.p0_to_p3[otherBranch.handle] = distance;
                                   // reversed_candidate_branch_connections.emplace_back(p3, otherBranchP0);
                                 }
                               }
                             }
                           }
                         }
                       } else if (connectivity_graph_settings.reverse_connection) {
                         const auto otherBranchP0 = otherBranch.bezier_curve.p0;
                         const auto otherBranchP3 = otherBranch.bezier_curve.p3;
                         if (DirectConnectionCheck(branch.bezier_curve, otherBranch.bezier_curve, true)) {
                           const auto distance = glm::distance(otherBranchP3, p3);
                           if (distance <= glm::distance(otherBranchP0, otherBranchP3) *
                                               connectivity_graph_settings.branch_branch_connection_max_length_range) {
                             const auto search = otherBranch.p3_to_p3.find(branch.handle);
                             if (search == otherBranch.p3_to_p3.end()) {
                               otherBranch.p3_to_p3[branch.handle] = distance;
                               branch.p3_to_p3[otherBranch.handle] = distance;
                               reversed_candidate_branch_connections.emplace_back(p3, otherBranchP3);
                             }
                           }
                         }
                       }
                     });
  }
  // We search branch connections via scatter points start from p0.
  for (auto& predictedBranch : predicted_branches) {
    std::unordered_set<PointHandle> visitedPoints;
    std::vector<PointHandle> processingPoints;
    float distanceL = FLT_MAX;
    for (const auto& i : predictedBranch.points_to_p3) {
      processingPoints.emplace_back(i.second);
      auto distance = glm::distance(predictedBranch.bezier_curve.p3, scattered_points[i.second].position);
      if (distance < distanceL)
        distanceL = distance;
    }
    for (const auto& i : processingPoints) {
      visitedPoints.emplace(i);
    }
    while (!processingPoints.empty()) {
      auto currentPointHandle = processingPoints.back();
      visitedPoints.emplace(currentPointHandle);
      processingPoints.pop_back();
      auto& currentPoint = scattered_points[currentPointHandle];
      for (const auto& branchInfo : currentPoint.p0) {
        if (predictedBranch.handle == branchInfo.second)
          continue;
        bool skip = false;
        for (const auto& i : predictedBranch.p3_to_p0) {
          if (branchInfo.second == i.first) {
            skip = true;
            break;
          }
        }
        if (skip)
          continue;
        auto& otherBranch = predicted_branches[branchInfo.second];
        auto pA = predictedBranch.bezier_curve.p3;
        auto pB = predictedBranch.bezier_curve.p0;
        auto otherPA = otherBranch.bezier_curve.p3;
        auto otherPB = otherBranch.bezier_curve.p0;
        const auto dotP = glm::dot(glm::normalize(otherPB - otherPA), glm::normalize(pB - pA));
        if (dotP < glm::cos(glm::radians(connectivity_graph_settings.indirect_connection_angle_limit)))
          continue;
        float distance = distanceL + branchInfo.first;
        const auto search = predictedBranch.p3_to_p0.find(branchInfo.second);
        if (search == predictedBranch.p3_to_p0.end() || search->second > distance) {
          predictedBranch.p3_to_p0[branchInfo.second] = distance;
          candidate_branch_connections.emplace_back(pA, otherPB);
        }
      }
      /*
              if (connectivity_graph_settings.reverse_connection)
              {
                      for (const auto& branchInfo : currentPoint.p3) {
                              if (predictedBranch.handle == branchInfo.second) continue;
                              bool skip = false;
                              for (const auto& i : predictedBranch.p3_to_p3) {
                                      if (branchInfo.second == i.first) {
                                              skip = true;
                                              break;
                                      }
                              }
                              if (skip) continue;
                              auto& otherBranch = predicted_branches[branchInfo.second];
                              auto pA = predictedBranch.bezier_curve.p3;
                              auto pB = predictedBranch.bezier_curve.p0;
                              auto otherPA = otherBranch.bezier_curve.p0;
                              auto otherPB = otherBranch.bezier_curve.p3;
                              const auto dotP = glm::dot(glm::normalize(otherPB - otherPA),
                                      glm::normalize(pB - pA));
                              if (dotP >
         glm::cos(glm::radians(connectivity_graph_settings.indirect_connection_angle_limit))) continue; const auto dotP2
         = glm::dot(glm::normalize(pB - pA), glm::normalize(otherPA - pA)); if (dotP2 > 3) continue;

                              float distance = distanceL + branchInfo.first;
                              const auto search = predictedBranch.p3_to_p3.find(branchInfo.second);
                              if (search == predictedBranch.p3_to_p3.end() || search->second > distance)
                              {
                                      predictedBranch.p3_to_p3[branchInfo.second] = distance;
                                      reversed_candidate_branch_connections.emplace_back(pA, otherPB);
                              }
                      }
              }
              */
      for (const auto& neighborHandle : currentPoint.neighbor_scatter_points) {
        if (visitedPoints.find(neighborHandle) == visitedPoints.end())
          processingPoints.emplace_back(neighborHandle);
      }
    }
  }
  /*
  if (connectivity_graph_settings.reverse_connection)
  {
          for (auto& predictedBranch : predicted_branches) {
                  std::unordered_set<PointHandle> visitedPoints;
                  std::vector<PointHandle> processingPoints;
                  float distanceL = FLT_MAX;
                  for (const auto& i : predictedBranch.points_to_p0)
                  {
                          processingPoints.emplace_back(i.second);
                          auto distance = glm::distance(predictedBranch.bezier_curve.p0,
  scattered_points[i.second].position); if (distance < distanceL) distanceL = distance;
                  }
                  for (const auto& i : processingPoints) {
                          visitedPoints.emplace(i);
                  }
                  while (!processingPoints.empty()) {
                          auto currentPointHandle = processingPoints.back();
                          visitedPoints.emplace(currentPointHandle);
                          processingPoints.pop_back();
                          auto& currentPoint = scattered_points[currentPointHandle];
                          for (const auto& neighborHandle : currentPoint.neighbor_scatter_points) {
                                  if (visitedPoints.find(neighborHandle) != visitedPoints.end()) continue;
                                  auto& neighbor = scattered_points[neighborHandle];
                                  //We stop search if the point is junction point.
                                  for (const auto& branchInfo : neighbor.p3) {
                                          if (predictedBranch.handle == branchInfo.second) continue;
                                          bool skip = false;
                                          for (const auto& i : predictedBranch.p0_to_p3) {
                                                  if (branchInfo.second == i.first) {
                                                          skip = true;
                                                          break;
                                                  }
                                          }
                                          if (skip) continue;
                                          auto& parentCandidateBranch = predicted_branches[branchInfo.second];
                                          auto pA = predictedBranch.bezier_curve.p0;
                                          auto pB = predictedBranch.bezier_curve.p3;
                                          auto otherPA = parentCandidateBranch.bezier_curve.p0;
                                          auto otherPB = parentCandidateBranch.bezier_curve.p3;
                                          const auto dotP = glm::dot(glm::normalize(otherPB - otherPA),
                                                  glm::normalize(pB - pA));
                                          if (dotP >
  glm::cos(glm::radians(connectivity_graph_settings.indirect_connection_angle_limit))) continue; const auto dotP2 =
  glm::dot(glm::normalize(pB - pA), glm::normalize(otherPA - pA)); if (dotP2 > 0) continue;

                                          float distance = distanceL + branchInfo.first;
                                          const auto search = predictedBranch.p0_to_p3.find(branchInfo.second);
                                          if (search == predictedBranch.p0_to_p3.end() || search->second > distance)
                                          {
                                                  predictedBranch.p0_to_p3[branchInfo.second] = distance;
                                                  candidate_branch_connections.emplace_back(pA, otherPB);
                                          }
                                  }
                                  if (connectivity_graph_settings.reverse_connection)
                                  {
                                          for (const auto& branchInfo : neighbor.p0) {
                                                  if (predictedBranch.handle == branchInfo.second) continue;
                                                  bool skip = false;
                                                  for (const auto& i : predictedBranch.p0_to_p0) {
                                                          if (branchInfo.second == i.first) {
                                                                  skip = true;
                                                                  break;
                                                          }
                                                  }
                                                  if (skip) continue;
                                                  auto& parentCandidateBranch = predicted_branches[branchInfo.second];
                                                  auto pA = predictedBranch.bezier_curve.p0;
                                                  auto pB = predictedBranch.bezier_curve.p3;
                                                  auto otherPA = parentCandidateBranch.bezier_curve.p3;
                                                  auto otherPB = parentCandidateBranch.bezier_curve.p0;
                                                  const auto dotP = glm::dot(glm::normalize(otherPB - otherPA),
                                                          glm::normalize(pB - pA));
                                                  if (dotP >
  glm::cos(glm::radians(connectivity_graph_settings.indirect_connection_angle_limit))) continue; const auto dotP2 =
  glm::dot(glm::normalize(pB - pA), glm::normalize(otherPA - pA)); if (dotP2 > 0) continue; float distance = distanceL +
  branchInfo.first; const auto search = predictedBranch.p0_to_p0.find(branchInfo.second); if (search ==
  predictedBranch.p0_to_p0.end() || search->second > distance)
                                                  {
                                                          predictedBranch.p0_to_p0[branchInfo.second] = distance;
                                                          reversed_candidate_branch_connections.emplace_back(pA,
  otherPB);
                                                  }
                                          }
                                  }
                                  processingPoints.emplace_back(neighborHandle);
                          }
                  }
          }
  }

  */
}

void TreeStructor::BuildSkeletons() {
  skeletons.clear();
  std::unordered_set<BranchHandle> allocatedBranchHandles;
  filtered_branch_connections.clear();
  branch_connections.clear();
  operating_branches.resize(predicted_branches.size());
  for (int i = 0; i < predicted_branches.size(); i++) {
    CloneOperatingBranch(reconstruction_settings, operating_branches[i], predicted_branches[i]);
  }
  std::vector<std::pair<glm::vec3, BranchHandle>> rootBranchHandles{};
  std::unordered_set<BranchHandle> rootBranchHandleSet{};
  // Branch is shortened after this.
  for (auto& branch : operating_branches) {
    branch.chain_node_handles.clear();
    branch.root_distance = 0.0f;
    branch.distance_to_parent_branch = 0.0f;
    branch.best_distance = FLT_MAX;
    const auto branchStart = branch.bezier_curve.p0;
    auto shortenedP0 = branch.bezier_curve.GetPoint(reconstruction_settings.branch_shortening);
    auto shortenedP3 = branch.bezier_curve.GetPoint(1.0f - reconstruction_settings.branch_shortening);
    auto shortenedLength = glm::distance(shortenedP0, shortenedP3);
    if (branchStart.y <= reconstruction_settings.min_height) {
      // branch.bezier_curve.p0.y = 0.0f;
      rootBranchHandles.emplace_back(branch.bezier_curve.p0, branch.handle);
      rootBranchHandleSet.emplace(branch.handle);
    } else {
      branch.bezier_curve.p0 = shortenedP0;
    }
    branch.bezier_curve.p3 = shortenedP3;
    branch.bezier_curve.p1 =
        branch.bezier_curve.p0 +
        branch.bezier_curve.GetAxis(reconstruction_settings.branch_shortening) * shortenedLength * 0.25f;
    branch.bezier_curve.p2 =
        branch.bezier_curve.p3 -
        branch.bezier_curve.GetAxis(1.0f - reconstruction_settings.branch_shortening) * shortenedLength * 0.25f;
  }

  bool branchRemoved = true;
  while (branchRemoved) {
    branchRemoved = false;
    std::vector<BranchHandle> removeList{};
    for (int i = 0; i < operating_branches.size(); i++) {
      if (rootBranchHandleSet.find(i) != rootBranchHandleSet.end())
        continue;
      auto& operatingBranch = operating_branches[i];
      if (!operatingBranch.orphan && operatingBranch.parent_candidates.empty()) {
        operatingBranch.orphan = true;
        removeList.emplace_back(i);
      }
    }
    if (!removeList.empty()) {
      branchRemoved = true;
      for (auto& operatingBranch : operating_branches) {
        for (int i = 0; i < operatingBranch.parent_candidates.size(); i++) {
          for (const auto& removeHandle : removeList) {
            if (operatingBranch.parent_candidates[i].first == removeHandle) {
              operatingBranch.parent_candidates.erase(operatingBranch.parent_candidates.begin() + i);
              i--;
              break;
            }
          }
        }
      }
    }
  }
  for (auto& operatingBranch : operating_branches) {
    if (operatingBranch.orphan)
      continue;
    for (const auto& parentCandidate : operatingBranch.parent_candidates) {
      const auto& parentBranch = operating_branches[parentCandidate.first];
      if (parentBranch.orphan)
        continue;
      filtered_branch_connections.emplace_back(operatingBranch.bezier_curve.p0, parentBranch.bezier_curve.p3);
    }
  }

  for (const auto& rootBranchHandle : rootBranchHandles) {
    auto& skeleton = skeletons.emplace_back();
    skeleton.data.root_position = rootBranchHandle.first;
    auto& processingBranch = operating_branches[rootBranchHandle.second];
    processingBranch.skeleton_index = skeletons.size() - 1;
    processingBranch.used = true;
    int prevNodeHandle = 0;
    processingBranch.chain_node_handles.emplace_back(prevNodeHandle);
    float chainLength = processingBranch.bezier_curve.GetLength();
    int chainAmount = glm::max(2, static_cast<int>(chainLength / reconstruction_settings.internode_length));
    for (int i = 1; i < chainAmount; i++) {
      prevNodeHandle = skeleton.Extend(prevNodeHandle, false);
      processingBranch.chain_node_handles.emplace_back(prevNodeHandle);
    }
    ApplyCurve(processingBranch);
  }

  std::multimap<float, BranchHandle> heightSortedBranches{};
  for (const auto& i : operating_branches) {
    heightSortedBranches.insert({i.bezier_curve.p0.y, i.handle});
  }

  bool newBranchAllocated = true;
  while (newBranchAllocated) {
    newBranchAllocated = false;
    for (const auto& branchPair : heightSortedBranches) {
      auto& childBranch = operating_branches[branchPair.second];
      if (childBranch.orphan || childBranch.used || childBranch.parent_candidates.empty())
        continue;
      BranchHandle bestParentHandle = -1;
      float bestDistance = FLT_MAX;
      float bestParentDistance = FLT_MAX;
      float bestRootDistance = FLT_MAX;
      int maxIndex = -1;
      for (int i = 0; i < childBranch.parent_candidates.size(); i++) {
        const auto& parentCandidate = childBranch.parent_candidates[i];
        const auto& parentBranch = operating_branches[parentCandidate.first];
        if (!parentBranch.used || parentBranch.child_handles.size() >= reconstruction_settings.max_child_size)
          continue;
        float distance = parentCandidate.second;
        float rootDistance = parentBranch.root_distance +
                             glm::distance(parentBranch.bezier_curve.p0, parentBranch.bezier_curve.p3) + distance;
        if (reconstruction_settings.use_root_distance) {
          distance = rootDistance;
        }
        if (distance < bestDistance) {
          bestParentHandle = parentCandidate.first;
          bestRootDistance = rootDistance;
          bestParentDistance = parentCandidate.second;
          maxIndex = i;
          bestDistance = distance;
        }
      }
      if (maxIndex != -1) {
        childBranch.parent_candidates[maxIndex] = childBranch.parent_candidates.back();
        childBranch.parent_candidates.pop_back();
        newBranchAllocated = true;
        Link(branchPair.second, bestParentHandle);
        childBranch.root_distance = bestRootDistance;
        childBranch.best_distance = bestDistance;
        childBranch.distance_to_parent_branch = bestParentDistance;
      }
    }
  }
  bool optimized = true;
  int iteration = 0;
  while (optimized && iteration < reconstruction_settings.optimization_timeout) {
    optimized = false;
    for (auto& childBranch : operating_branches) {
      if (childBranch.orphan || childBranch.parent_handle == -1 || childBranch.parent_candidates.empty())
        continue;
      BranchHandle bestParentHandle = -1;
      float bestDistance = childBranch.best_distance;
      float bestParentDistance = FLT_MAX;
      float bestRootDistance = childBranch.root_distance;
      int maxIndex = -1;
      for (int i = 0; i < childBranch.parent_candidates.size(); i++) {
        const auto& parentCandidate = childBranch.parent_candidates[i];
        const auto& parentBranch = operating_branches[parentCandidate.first];
        if (!parentBranch.used || parentBranch.child_handles.size() >= reconstruction_settings.max_child_size)
          continue;
        float distance = parentCandidate.second;
        float rootDistance = parentBranch.root_distance +
                             glm::distance(parentBranch.bezier_curve.p0, parentBranch.bezier_curve.p3) + distance;
        if (reconstruction_settings.use_root_distance) {
          distance = rootDistance;
        }
        if (distance < bestDistance) {
          bestParentHandle = parentCandidate.first;
          bestRootDistance = rootDistance;
          bestParentDistance = parentCandidate.second;
          maxIndex = i;
          bestDistance = distance;
        }
      }
      std::vector<BranchHandle> subTreeBranchList{};
      GetSortedBranchList(childBranch.handle, subTreeBranchList);
      if (maxIndex != -1) {
        for (const auto& branchHandle : subTreeBranchList) {
          if (branchHandle == bestParentHandle) {
            maxIndex = -1;
            break;
          }
        }
      }
      if (maxIndex != -1) {
        childBranch.parent_candidates[maxIndex] = childBranch.parent_candidates.back();
        childBranch.parent_candidates.pop_back();
        newBranchAllocated = true;
        Unlink(childBranch.handle, childBranch.parent_handle);
        Link(childBranch.handle, bestParentHandle);
        childBranch.root_distance = bestRootDistance;
        childBranch.best_distance = bestDistance;
        childBranch.distance_to_parent_branch = bestParentDistance;
        optimized = true;
      }
    }
    if (optimized) {
      CalculateBranchRootDistance(rootBranchHandles);
    }
    iteration++;
  }
  CalculateBranchRootDistance(rootBranchHandles);

  for (const auto& operatingBranch : operating_branches) {
    if (operatingBranch.parent_handle != -1) {
      branch_connections.emplace_back(predicted_branches[operatingBranch.handle].bezier_curve.p0,
                                       predicted_branches[operatingBranch.parent_handle].bezier_curve.p3);
    }
  }

  // Assign apical branch.
  for (const auto& rootBranchHandle : rootBranchHandles) {
    std::vector<BranchHandle> sortedBranchList{};
    GetSortedBranchList(rootBranchHandle.second, sortedBranchList);
    for (auto it = sortedBranchList.rbegin(); it != sortedBranchList.rend(); ++it) {
      auto& operatingBranch = operating_branches[*it];
      int maxDescendentSize = -1;
      operatingBranch.largest_child_handle = -1;
      for (const auto& childHandle : operatingBranch.child_handles) {
        auto& childBranch = operating_branches[childHandle];
        operatingBranch.descendant_size += childBranch.descendant_size + 1;
        if (childBranch.descendant_size >= maxDescendentSize) {
          maxDescendentSize = childBranch.descendant_size;
          operatingBranch.largest_child_handle = childHandle;
        }
      }
      for (const auto& childHandle : operatingBranch.child_handles) {
        auto& childBranch = operating_branches[childHandle];
        if (childHandle == operatingBranch.largest_child_handle) {
          childBranch.apical = true;
        } else {
          childBranch.apical = false;
        }
      }
    }
  }

  // Smoothing
  for (int i = 0; i < reconstruction_settings.smooth_iteration; i++) {
    for (const auto& rootBranchHandle : rootBranchHandles) {
      std::vector<BranchHandle> sortedBranchList{};
      GetSortedBranchList(rootBranchHandle.second, sortedBranchList);
      for (const auto& branchHandle : sortedBranchList) {
        auto& branch = operating_branches[branchHandle];
        if (branch.parent_handle != -1 && branch.largest_child_handle != -1) {
          const auto& parentBranch = operating_branches[branch.parent_handle];
          if (parentBranch.largest_child_handle != branchHandle)
            continue;
          const auto& childBranch = operating_branches[branch.largest_child_handle];
          const auto parentCenter = (parentBranch.bezier_curve.p0 + parentBranch.bezier_curve.p3) * 0.5f;
          const auto childCenter = (childBranch.bezier_curve.p0 + childBranch.bezier_curve.p3) * 0.5f;
          const auto center = (branch.bezier_curve.p0 + branch.bezier_curve.p3) * 0.5f;
          const auto desiredCenter = (parentCenter + childCenter) * 0.5f;
          auto diff = (desiredCenter - center);
          branch.bezier_curve.p0 = branch.bezier_curve.p0 + diff * reconstruction_settings.position_smoothing;
          branch.bezier_curve.p1 = branch.bezier_curve.p1 + diff * reconstruction_settings.position_smoothing;
          branch.bezier_curve.p2 = branch.bezier_curve.p2 + diff * reconstruction_settings.position_smoothing;
          branch.bezier_curve.p3 = branch.bezier_curve.p3 + diff * reconstruction_settings.position_smoothing;
        }
      }
    }
    for (const auto& rootBranchHandle : rootBranchHandles) {
      std::vector<BranchHandle> sortedBranchList{};
      GetSortedBranchList(rootBranchHandle.second, sortedBranchList);
      for (const auto& branchHandle : sortedBranchList) {
        auto& branch = operating_branches[branchHandle];
        if (branch.parent_handle != -1 && branch.largest_child_handle != -1) {
          const auto& parentBranch = operating_branches[branch.parent_handle];
          const auto& childBranch = operating_branches[branch.largest_child_handle];
          const auto parentCenter = (parentBranch.bezier_curve.p0 + parentBranch.bezier_curve.p3) * 0.5f;
          const auto childCenter = (childBranch.bezier_curve.p0 + childBranch.bezier_curve.p3) * 0.5f;
          const auto center = (branch.bezier_curve.p0 + branch.bezier_curve.p3) * 0.5f;
          const auto length = glm::distance(branch.bezier_curve.p0, branch.bezier_curve.p3);
          // const auto currentDirection = glm::normalize(branch.bezier_curve.p3 - branch.bezier_curve.p0);

          const auto desiredDirection = glm::normalize(parentCenter - childCenter);

          const auto desiredP0 = center + desiredDirection * length * 0.5f;
          const auto desiredP3 = center - desiredDirection * length * 0.5f;
          const auto desiredP1 = glm::mix(desiredP0, desiredP3, 0.25f);
          const auto desiredP2 = glm::mix(desiredP0, desiredP3, 0.75f);

          branch.bezier_curve.p0 =
              glm::mix(branch.bezier_curve.p0, desiredP0, reconstruction_settings.direction_smoothing);
          branch.bezier_curve.p1 =
              glm::mix(branch.bezier_curve.p1, desiredP1, reconstruction_settings.direction_smoothing);
          branch.bezier_curve.p2 =
              glm::mix(branch.bezier_curve.p2, desiredP2, reconstruction_settings.direction_smoothing);
          branch.bezier_curve.p3 =
              glm::mix(branch.bezier_curve.p3, desiredP3, reconstruction_settings.direction_smoothing);
        }
      }
    }
  }
  for (const auto& rootBranchHandle : rootBranchHandles) {
    ConnectBranches(rootBranchHandle.second);
  }

  CalculateSkeletonGraphs();

  for (auto& allocatedPoint : allocated_points) {
    const auto& treePart = tree_parts[allocatedPoint.tree_part_handle];
    float minDistance = 999.f;
    SkeletonNodeHandle closestNodeHandle = -1;
    BranchHandle closestBranchHandle = -1;
    int closestSkeletonIndex = -1;
    for (const auto& branchHandle : treePart.branch_handles) {
      auto& branch = operating_branches[branchHandle];
      for (const auto& nodeHandle : branch.chain_node_handles) {
        auto& node = skeletons[branch.skeleton_index].RefNode(nodeHandle);
        auto distance = glm::distance(node.info.global_position, allocatedPoint.position);
        if (distance < minDistance) {
          minDistance = distance;
          closestNodeHandle = nodeHandle;
          closestSkeletonIndex = branch.skeleton_index;
          closestBranchHandle = branchHandle;
        }
      }
    }
    allocatedPoint.node_handle = closestNodeHandle;
    allocatedPoint.branch_handle = closestBranchHandle;
    allocatedPoint.skeleton_index = closestSkeletonIndex;
    if (allocatedPoint.skeleton_index != -1)
      skeletons[allocatedPoint.skeleton_index]
          .RefNode(closestNodeHandle)
          .data.allocated_points.emplace_back(allocatedPoint.handle);
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
          auto& otherSkeleton = skeletons[j];
          if (glm::distance(skeleton.data.root_position, otherSkeleton.data.root_position) <
              reconstruction_settings.minimum_tree_distance) {
            const auto& sortedNodeList = skeleton.PeekSortedNodeList();
            const auto& otherSkeletonSortedNodeList = otherSkeleton.PeekSortedNodeList();
            if (sortedNodeList.size() < otherSkeletonSortedNodeList.size()) {
              remove = true;
              if (sortedNodeList.size() > reconstruction_settings.minimum_node_count) {
                std::unordered_map<SkeletonNodeHandle, SkeletonNodeHandle> nodeHandleMap;
                nodeHandleMap[0] = 0;
                for (const auto& nodeHandle : sortedNodeList) {
                  const auto& node = skeleton.PeekNode(nodeHandle);
                  SkeletonNodeHandle newNodeHandle = -1;
                  if (node.GetParentHandle() == -1) {
                    continue;
                  }
                  newNodeHandle = otherSkeleton.Extend(nodeHandleMap.at(node.GetParentHandle()), !node.IsApical());
                  nodeHandleMap[nodeHandle] = newNodeHandle;
                  auto& newNode = otherSkeleton.RefNode(newNodeHandle);
                  newNode.info = node.info;
                  newNode.data = node.data;
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
    for (int i = 0; i < skeletons.size(); i++) {
      auto& skeleton = skeletons[i];
      const auto& sortedList = skeleton.PeekSortedNodeList();
    }
  }
  CalculateSkeletonGraphs();
  SpaceColonization();
}

void TreeStructor::GenerateForest() const {
  const auto scene = GetScene();
  const auto owner = GetOwner();
  const auto children = scene->GetChildren(owner);
  for (const auto& i : children) {
    if (scene->GetEntityName(i) == "Forest") {
      scene->DeleteEntity(i);
    }
  }

  const auto forestEntity = scene->CreateEntity("Forest");
  scene->SetParent(forestEntity, owner);
  for (const auto& skeleton : skeletons) {
    const auto treeEntity = scene->CreateEntity("Tree");
    scene->SetParent(treeEntity, forestEntity);
    const auto tree = scene->GetOrSetPrivateComponent<Tree>(treeEntity).lock();
    tree->tree_descriptor = tree_descriptor;
    tree->FromSkeleton(skeleton);
    GlobalTransform gt{};
    gt.SetPosition(skeleton.data.root_position);
    scene->SetDataComponent(treeEntity, gt);
  }
}

void TreeStructor::SpaceColonization() {
  if (reconstruction_settings.space_colonization_factor == 0.0f)
    return;

  Jobs::RunParallelFor(skeletons.size(), [&](unsigned i) {
    auto& skeleton = skeletons[i];
    const auto& sortedInternodeList = skeleton.PeekSortedNodeList();
    float maxEndDistance = 0.0f;
    for (const auto& internodeHandle : sortedInternodeList) {
      auto& internode = skeleton.RefNode(internodeHandle);
      const auto distance = internode.info.end_distance + internode.info.length;
      if (internode.GetParentHandle() == -1) {
        skeleton.data.max_end_distance = distance;
        maxEndDistance = distance * reconstruction_settings.space_colonization_factor;
      }
      internode.data.regrowth = distance <= maxEndDistance;
    }
  });

  // Register voxel grid.
  const float removalDistance =
      reconstruction_settings.space_colonization_removal_distance_factor * reconstruction_settings.internode_length;
  const float detectionDistance =
      reconstruction_settings.space_colonization_detection_distance_factor * reconstruction_settings.internode_length;

  space_colonization_voxel_grid.Initialize(removalDistance, min, max);
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

  VoxelGrid<std::vector<PointData>> internodeEndGrid{};
  internodeEndGrid.Initialize(detectionDistance, min, max);
  for (int skeletonIndex = 0; skeletonIndex < skeletons.size(); skeletonIndex++) {
    auto& skeleton = skeletons[skeletonIndex];
    const auto& sortedInternodeList = skeleton.PeekSortedNodeList();
    for (const auto& internodeHandle : sortedInternodeList) {
      auto& internode = skeleton.PeekNode(internodeHandle);
      if (!internode.data.regrowth)
        continue;
      PointData voxel;
      voxel.handle = internodeHandle;
      voxel.index = skeletonIndex;
      voxel.position = internode.data.global_end_position;
      voxel.direction = internode.info.GetGlobalDirection();
      internodeEndGrid.Ref(voxel.position).emplace_back(voxel);
    }
  }

  const auto dotMin = glm::cos(glm::radians(reconstruction_settings.space_colonization_theta));
  bool newBranchGrown = true;
  int timeout = 0;
  while (newBranchGrown && timeout < reconstruction_settings.space_colonization_timeout) {
    newBranchGrown = false;
    timeout++;
    // 1. Remove markers with occupancy zone.
    for (auto& skeleton : skeletons) {
      const auto& sortedInternodeList = skeleton.PeekSortedNodeList();
      for (const auto& internodeHandle : sortedInternodeList) {
        auto& internode = skeleton.RefNode(internodeHandle);
        internode.data.marker_size = 0;
        internode.data.regrow_direction = glm::vec3(0.0f);
        const auto internodeEndPosition = internode.data.global_end_position;
        space_colonization_voxel_grid.ForEach(internodeEndPosition, removalDistance,
                                             [&](std::vector<PointData>& voxels) {
                                               for (int i = 0; i < voxels.size(); i++) {
                                                 auto& marker = voxels[i];
                                                 const auto diff = marker.position - internodeEndPosition;
                                                 const auto distance = glm::length(diff);
                                                 if (distance < removalDistance) {
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
        internodeEndGrid.ForEach(point.position, detectionDistance, [&](const std::vector<PointData>& voxels) {
          for (const auto& internodeEnd : voxels) {
            const auto diff = point.position - internodeEnd.position;
            const auto distance = glm::length(diff);
            const auto direction = glm::normalize(diff);
            if (distance < detectionDistance && glm::dot(direction, internodeEnd.direction) > dotMin &&
                distance < point.min_distance) {
              point.min_distance = distance;
              point.handle = internodeEnd.handle;
              point.index = internodeEnd.index;
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
    for (int skeletonIndex = 0; skeletonIndex < skeletons.size(); skeletonIndex++) {
      auto& skeleton = skeletons[skeletonIndex];
      const auto& sortedInternodeList = skeleton.PeekSortedNodeList();
      for (const auto& internodeHandle : sortedInternodeList) {
        auto& internode = skeleton.PeekNode(internodeHandle);
        if (!internode.data.regrowth || internode.data.marker_size == 0)
          continue;
        if (internode.info.root_distance > skeleton.data.max_end_distance)
          continue;
        newBranchGrown = true;
        const auto newInternodeHandle = skeleton.Extend(internodeHandle, !internode.PeekChildHandles().empty());
        auto& oldInternode = skeleton.RefNode(internodeHandle);
        auto& newInternode = skeleton.RefNode(newInternodeHandle);
        newInternode.info.global_position = oldInternode.info.GetGlobalEndPosition();
        newInternode.info.length = reconstruction_settings.internode_length;
        newInternode.info.global_rotation =
            glm::quatLookAt(oldInternode.data.regrow_direction,
                            glm::vec3(oldInternode.data.regrow_direction.y, oldInternode.data.regrow_direction.z,
                                      oldInternode.data.regrow_direction.x));
        newInternode.data.global_end_position =
            oldInternode.data.global_end_position + newInternode.info.length * newInternode.info.GetGlobalDirection();
        newInternode.data.regrowth = true;
        PointData voxel;
        voxel.handle = newInternodeHandle;
        voxel.index = skeletonIndex;
        voxel.position = newInternode.data.global_end_position;
        voxel.direction = newInternode.info.GetGlobalDirection();
        internodeEndGrid.Ref(voxel.position).emplace_back(voxel);
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
    const std::vector<std::pair<glm::vec3, BranchHandle>>& rootBranchHandles) {
  for (const auto& rootBranchHandle : rootBranchHandles) {
    std::vector<BranchHandle> sortedBranchList{};
    GetSortedBranchList(rootBranchHandle.second, sortedBranchList);
    for (const auto branchHandle : sortedBranchList) {
      auto& branch = operating_branches[branchHandle];
      if (branch.parent_handle == -1) {
        branch.root_distance = 0.0f;
      } else {
        const auto& parentBranch = operating_branches[branch.parent_handle];
        branch.root_distance = parentBranch.root_distance +
                                glm::distance(parentBranch.bezier_curve.p0, parentBranch.bezier_curve.p3) +
                                branch.distance_to_parent_branch;
        branch.skeleton_index = parentBranch.skeleton_index;
      }
    }
  }
}

void TreeStructor::CalculateSkeletonGraphs() {
  for (auto& skeleton : skeletons) {
    skeleton.SortLists();
    auto& sortedNodeList = skeleton.PeekSortedNodeList();
    auto& rootNode = skeleton.RefNode(0);
    rootNode.info.global_rotation = rootNode.info.regulated_global_rotation =
        glm::quatLookAt(glm::vec3(0, 1, 0), glm::vec3(0, 0, -1));
    rootNode.info.global_position = glm::vec3(0.0f);
    rootNode.info.length = glm::length(rootNode.data.global_end_position - rootNode.data.global_start_position);
    for (const auto& nodeHandle : sortedNodeList) {
      auto& node = skeleton.RefNode(nodeHandle);
      auto& nodeInfo = node.info;
      auto& nodeData = node.data;
      if (nodeHandle == 0)
        continue;

      auto& parentNode = skeleton.RefNode(node.GetParentHandle());
      auto diff = nodeData.global_end_position - parentNode.data.global_end_position;
      auto front = glm::normalize(diff);
      auto parentUp = parentNode.info.global_rotation * glm::vec3(0, 1, 0);
      auto regulatedUp = glm::normalize(glm::cross(glm::cross(front, parentUp), front));
      nodeInfo.global_rotation = glm::quatLookAt(front, regulatedUp);
      nodeInfo.length = glm::length(diff);
    }

    for (auto i = sortedNodeList.rbegin(); i != sortedNodeList.rend(); ++i) {
      auto& node = skeleton.RefNode(*i);
      auto& nodeData = node.data;
      auto& childHandles = node.PeekChildHandles();
      if (childHandles.empty()) {
        nodeData.draft_thickness = reconstruction_settings.end_node_thickness;
      } else {
        float childThicknessCollection = 0.0f;
        for (const auto& childHandle : childHandles) {
          const auto& childNode = skeleton.RefNode(childHandle);
          childThicknessCollection +=
              glm::pow(childNode.data.draft_thickness + reconstruction_settings.thickness_accumulation_factor,
                       1.0f / reconstruction_settings.thickness_sum_factor);
        }
        nodeData.draft_thickness = glm::pow(childThicknessCollection, reconstruction_settings.thickness_sum_factor);
      }
    }
    const auto rootNodeThickness = skeleton.PeekNode(0).data.draft_thickness;
    if (rootNodeThickness < reconstruction_settings.minimum_root_thickness) {
      float multiplierFactor = reconstruction_settings.minimum_root_thickness / rootNodeThickness;
      for (const auto& handle : sortedNodeList) {
        auto& nodeData = skeleton.RefNode(handle).data;
        nodeData.draft_thickness *= multiplierFactor;
      }
    }
    skeleton.CalculateDistance();
    for (auto i = sortedNodeList.rbegin(); i != sortedNodeList.rend(); ++i) {
      auto& node = skeleton.RefNode(*i);
      auto& nodeData = node.data;
      auto& nodeInfo = node.info;
      if (nodeInfo.root_distance >= reconstruction_settings.override_thickness_root_distance) {
        nodeInfo.thickness = nodeData.draft_thickness;
      }
      if (reconstruction_settings.limit_parent_thickness) {
        auto& childHandles = node.PeekChildHandles();
        float maxChildThickness = 0.0f;
        for (const auto& childHandle : childHandles) {
          maxChildThickness = glm::max(maxChildThickness, skeleton.PeekNode(childHandle).info.thickness);
        }
        nodeInfo.thickness = glm::max(nodeInfo.thickness, maxChildThickness);
      }

      if (nodeData.branch_handle < predicted_branches.size())
        predicted_branches[nodeData.branch_handle].final_thickness = nodeInfo.thickness;
    }

    CalculateNodeTransforms(skeleton);
    skeleton.CalculateFlows();
  }
}

void TreeStructor::ClearForest() const {
  const auto scene = GetScene();
  const auto owner = GetOwner();
  const auto children = scene->GetChildren(owner);
  for (const auto& i : children) {
    if (scene->GetEntityName(i) == "Forest") {
      scene->DeleteEntity(i);
    }
  }
}

void TreeStructor::OnCreate() {
}

std::vector<std::shared_ptr<Mesh>> TreeStructor::GenerateForestBranchMeshes(
    const TreeMeshGeneratorSettings& meshGeneratorSettings) const {
  std::vector<std::shared_ptr<Mesh>> meshes{};
  for (const auto& skeleton : skeletons) {
    std::vector<Vertex> vertices;
    std::vector<unsigned int> indices;
    CylindricalMeshGenerator<ReconstructionSkeletonData, ReconstructionFlowData, ReconstructionNodeData>::Generate(
        skeleton, vertices, indices, meshGeneratorSettings,
        [&](glm::vec3& vertexPosition, const glm::vec3& direction, const float xFactor, const float yFactor) {
        },
        [&](glm::vec2& texCoords, float xFactor, float yFactor) {
        });
    Jobs::RunParallelFor(vertices.size(), [&](unsigned j) {
      vertices[j].position += skeleton.data.root_position;
    });
    auto mesh = ProjectManager::CreateTemporaryAsset<Mesh>();
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

    auto quadMesh = Resources::GetResource<Mesh>("PRIMITIVE_QUAD");
    auto& quadTriangles = quadMesh->UnsafeGetTriangles();
    auto quadVerticesSize = quadMesh->GetVerticesAmount();
    if (const auto treeDescriptor = tree_descriptor.Get<TreeDescriptor>()) {
      size_t offset = 0;
      auto foliageDescriptor = treeDescriptor->foliage_descriptor.Get<FoliageDescriptor>();
      if (!foliageDescriptor)
        foliageDescriptor = ProjectManager::CreateTemporaryAsset<FoliageDescriptor>();
      const auto& nodeList = skeleton.PeekSortedNodeList();
      for (const auto& internodeHandle : nodeList) {
        const auto& internode = skeleton.PeekNode(internodeHandle);
        const auto& internodeInfo = internode.info;

        if (internodeInfo.thickness < foliageDescriptor->max_node_thickness &&
            internodeInfo.root_distance > foliageDescriptor->min_root_distance &&
            internodeInfo.end_distance < foliageDescriptor->max_end_distance) {
          for (int i = 0; i < foliageDescriptor->leaf_count_per_internode; i++) {
            auto leafSize = foliageDescriptor->leaf_size;
            glm::quat rotation = internodeInfo.GetGlobalDirection() *
                                 glm::quat(glm::radians(glm::linearRand(glm::vec3(0.0f), glm::vec3(360.0f))));
            auto front = rotation * glm::vec3(0, 0, -1);
            auto foliagePosition =
                internodeInfo.global_position + front * (leafSize.y * 1.5f) +
                glm::sphericalRand(1.0f) * glm::linearRand(0.0f, foliageDescriptor->position_variance);
            auto leafTransform = glm::translate(foliagePosition) * glm::mat4_cast(rotation) *
                                 glm::scale(glm::vec3(leafSize.x, 1.0f, leafSize.y));

            auto& matrix = leafTransform;
            Vertex archetype;
            for (auto vertex_index = 0; vertex_index < quadMesh->GetVerticesAmount(); vertex_index++) {
              archetype.position = matrix * glm::vec4(quadMesh->UnsafeGetVertices()[vertex_index].position, 1.0f);
              archetype.normal =
                  glm::normalize(glm::vec3(matrix * glm::vec4(quadMesh->UnsafeGetVertices()[vertex_index].normal, 0.0f)));
              archetype.tangent =
                  glm::normalize(glm::vec3(matrix * glm::vec4(quadMesh->UnsafeGetVertices()[vertex_index].tangent, 0.0f)));
              archetype.tex_coord = quadMesh->UnsafeGetVertices()[vertex_index].tex_coord;
              archetype.color = internodeInfo.color;
              vertices.push_back(archetype);
            }
            for (auto triangle : quadTriangles) {
              triangle.x += offset;
              triangle.y += offset;
              triangle.z += offset;
              indices.push_back(triangle.x);
              indices.push_back(triangle.y);
              indices.push_back(triangle.z);
            }
            offset += quadVerticesSize;
          }
        }
      }
    }
    Jobs::RunParallelFor(vertices.size(), [&](unsigned j) {
      vertices[j].position += skeleton.data.root_position;
    });
    auto mesh = ProjectManager::CreateTemporaryAsset<Mesh>();
    VertexAttributes attributes{};
    attributes.tex_coord = true;
    mesh->SetVertices(attributes, vertices, indices);
    meshes.emplace_back(mesh);
  }
  return meshes;
}

void TreeStructor::Serialize(YAML::Emitter& out) const {
  tree_descriptor.Save("tree_descriptor", out);
}

void TreeStructor::Deserialize(const YAML::Node& in) {
  tree_descriptor.Load("tree_descriptor", in);
}

void TreeStructor::CollectAssetRef(std::vector<AssetRef>& list) {
  if (tree_descriptor.Get<TreeDescriptor>())
    list.emplace_back(tree_descriptor);
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

void TreeStructor::CloneOperatingBranch(const ReconstructionSettings& reconstructionSettings,
                                        OperatorBranch& operatorBranch, const PredictedBranch& target) {
  operatorBranch.color = target.color;
  operatorBranch.tree_part_handle = target.tree_part_handle;
  operatorBranch.handle = target.handle;
  operatorBranch.bezier_curve = target.bezier_curve;
  operatorBranch.thickness = target.final_thickness;
  operatorBranch.parent_handle = -1;
  operatorBranch.child_handles.clear();
  operatorBranch.orphan = false;
  operatorBranch.parent_candidates.clear();
  operatorBranch.foliage = target.foliage;
  int count = 0;
  for (const auto& data : target.p3_to_p0) {
    operatorBranch.parent_candidates.emplace_back(data.first, data.second);
    count++;
    if (count > reconstructionSettings.max_parent_candidate_size)
      break;
  }
  operatorBranch.distance_to_parent_branch = 0.0f;
  operatorBranch.best_distance = FLT_MAX;
  operatorBranch.root_distance = 0.0f;
  operatorBranch.descendant_size = 0;
  operatorBranch.skeleton_index = -1;
  operatorBranch.used = false;
  operatorBranch.chain_node_handles.clear();
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
