#include "StrandModelMeshGenerator.hpp"
#include <glm/gtx/intersect.hpp>
#include <glm/gtx/io.hpp>
#include <queue>
#include "Jobs.hpp"
#include "MeshGenUtils.hpp"
#include "Octree.hpp"
#include "TreeMeshGenerator.hpp"
#include "AlphaShapeMeshGenerator.hpp"
#include "IterativeSlicingMeshGenerator.hpp"
#include "MarchingCubeMeshGenerator.hpp"
#include "EcoSysLabLayer.hpp"

using namespace eco_sys_lab_plugin;

void StrandModelMeshGeneratorSettings::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  ImGui::Combo("Mode", {"Iterative Slicing", "Marching Cube", "Alpha Shape"}, generator_type);
  if (generator_type == 0 && ImGui::TreeNode("Iterative Slicing settings")) {
    ImGui::DragInt("Steps per segment", &steps_per_segment, 1.0f, 1, 99);

    // ImGui::Checkbox("[DEBUG] Limit Profile Iterations", &m_limitProfileIterations);
    // ImGui::DragInt("[DEBUG] Limit", &m_maxProfileIterations);

    ImGui::DragFloat("[DEBUG] MaxParam", &max_param);
    // ImGui::Checkbox("Compute branch joints", &branch_connections);
    ImGui::DragInt("uCoord multiplier", &u_multiplier, 1, 1);
    ImGui::DragFloat("vCoord multiplier", &v_multiplier, 0.1f);
    ImGui::DragFloat("cluster distance factor", &cluster_distance, 0.1f, 1.0f, 10.0f);
    ImGui::TreePop();
  }

  if (generator_type == 1 && ImGui::TreeNode("Marching Cube settings")) {
    ImGui::Checkbox("Auto set level", &auto_level);
    if (!auto_level)
      ImGui::DragInt("Voxel subdivision level", &voxel_subdivision_level, 1, 5, 16);
    else
      ImGui::DragFloat("Min Cube size", &marching_cube_radius, 0.0001f, 0.001f, 1.0f);
    if (smooth_iteration == 0)
      ImGui::Checkbox("Remove duplicate", &remove_duplicate);
    ImGui::ColorEdit4("Marching cube color", &marching_cube_color.x);
    ImGui::ColorEdit4("Cylindrical color", &cylindrical_color.x);
    ImGui::DragInt("uCoord multiplier", &root_distance_multiplier, 1, 1, 100);
    ImGui::DragFloat("vCoord multiplier", &circle_multiplier, 0.1f);
    ImGui::TreePop();
  }

  if (generator_type == 2 && ImGui::TreeNode("Alpha Shape settings")) {
    ImGui::DragInt("Steps per segment", &steps_per_segment, 1.0f, 1, 99);
    ImGui::DragFloat("alpha factor", &cluster_distance, 0.1f, 1.0f, 10.0f);
    ImGui::DragInt("uCoord multiplier", &u_multiplier, 1, 1);
    ImGui::DragFloat("vCoord multiplier", &v_multiplier, 0.1f);
    ImGui::TreePop();
  }

  ImGui::DragInt("Major branch cell min", &min_cell_count_for_major_branches, 1, 0, 1000);
  ImGui::DragInt("Minor branch cell max", &max_cell_count_for_minor_branches, 1, 0, 1000);

  ImGui::Checkbox("Recalculate UV", &recalculate_uv);
  ImGui::Checkbox("Fast UV", &fast_uv);
  ImGui::DragInt("Smooth iteration", &smooth_iteration, 0, 0, 10);
  ImGui::Checkbox("Branch", &enable_branch);
  ImGui::Checkbox("Foliage", &enable_foliage);
}

 void StrandModelMeshGenerator::Generate(const StrandModel& strand_model, std::vector<Vertex>& vertices,
                                        std::vector<unsigned>& indices,
                                        const StrandModelMeshGeneratorSettings& settings) {
  switch (settings.generator_type) {
    case StrandModelMeshGeneratorType::RecursiveSlicing: {
      IterativeSlicingMeshGenerator::Generate(strand_model, vertices, indices, settings);
    } break;
    case StrandModelMeshGeneratorType::MarchingCube: {
      MarchingCubeMeshGenerator::Generate(strand_model, vertices, indices, settings);
    } break;
    case StrandModelMeshGeneratorType::AlphaShape: {
      AlphaShapeMeshGenerator::Generate(strand_model, vertices, indices, settings);
    } break;
  }

  if (settings.recalculate_uv ||
      settings.generator_type == static_cast<unsigned>(StrandModelMeshGeneratorType::MarchingCube)) {
    CalculateUv(strand_model, vertices, settings);
  }

  for (int i = 0; i < settings.smooth_iteration; i++) {
    MeshSmoothing(vertices, indices);
  }
  CylindricalMeshing(strand_model, vertices, indices, settings);

  CalculateNormal(vertices, indices);
}

void StrandModelMeshGenerator::Generate(const StrandModel& strand_model, std::vector<Vertex>& vertices,
                                        std::vector<glm::vec2>& tex_coords,
                                        std::vector<std::pair<unsigned, unsigned>>& index_pairs,
                                        const StrandModelMeshGeneratorSettings& settings) {
  
   switch (settings.generator_type) {
    case StrandModelMeshGeneratorType::RecursiveSlicing: {
       IterativeSlicingMeshGenerator::Generate(strand_model, vertices, tex_coords, index_pairs, settings);
    } break;
    case StrandModelMeshGeneratorType::MarchingCube: {
      MarchingCubeMeshGenerator::Generate(strand_model, vertices, tex_coords, index_pairs, settings);
    } break;
    case StrandModelMeshGeneratorType::AlphaShape: {
      AlphaShapeMeshGenerator::Generate(strand_model, vertices, tex_coords, index_pairs, settings);
    } break;
  }

  for (int i = 0; i < settings.smooth_iteration; i++) {
    MeshSmoothing(vertices, index_pairs);
  }
  std::vector<unsigned int> temp_indices{};
  CylindricalMeshing(strand_model, vertices, temp_indices, settings);
  for (const auto& index : temp_indices) {
    const auto& vertex = vertices.at(index);
    const auto tex_coords_index = tex_coords.size();
    tex_coords.emplace_back(vertex.tex_coord);
    index_pairs.emplace_back(index, tex_coords_index);
  }

  CalculateNormal(vertices, index_pairs);
}

void StrandModelMeshGenerator::CylindricalMeshing(const StrandModel& strand_model, std::vector<Vertex>& vertices,
                                                  std::vector<unsigned>& indices,
                                                  const StrandModelMeshGeneratorSettings& settings) {
  const auto& skeleton = strand_model.strand_model_skeleton;
  const auto& sorted_internode_list = skeleton.PeekSortedNodeList();
  std::unordered_set<SkeletonNodeHandle> node_handles;
  for (const auto& node_handle : sorted_internode_list) {
    const auto& internode = skeleton.PeekNode(node_handle);
    if (const int particle_size = internode.data.profile.PeekParticles().size();
        particle_size > settings.max_cell_count_for_minor_branches)
      continue;
    node_handles.insert(node_handle);
  }
  const auto current_vertices_size = vertices.size();
  const auto eco_sys_lab_layer = Application::GetLayer<EcoSysLabLayer>();
  CylindricalMeshGenerator<StrandModelSkeletonData, StrandModelFlowData, StrandModelNodeData>::GeneratePartially(
      node_handles, skeleton, vertices, indices, eco_sys_lab_layer->m_meshGeneratorSettings,
      [&](glm::vec3&, const glm::vec3&, const float, const float) {
      },
      [&](glm::vec2& tex_coords, const float, const float) {
        tex_coords.x *= 2.0f;
        tex_coords.y *= 12.0f * settings.v_multiplier;
      });
  for (auto i = current_vertices_size; i < vertices.size(); i++) {
    vertices.at(i).color = glm::vec4(0, 1, 0, 1);
  }
}

void StrandModelMeshGenerator::MeshSmoothing(std::vector<Vertex>& vertices, std::vector<unsigned>& indices) {
  std::vector<std::vector<unsigned>> connectivity;
  connectivity.resize(vertices.size());
  for (int i = 0; i < indices.size() / 3; i++) {
    auto a = indices[3 * i];
    auto b = indices[3 * i + 1];
    auto c = indices[3 * i + 2];
    // a
    {
      bool found1 = false;
      bool found2 = false;
      for (const auto& index : connectivity.at(a)) {
        if (b == index)
          found1 = true;
        if (c == index)
          found2 = true;
      }
      if (!found1) {
        connectivity.at(a).emplace_back(b);
      }
      if (!found2) {
        connectivity.at(a).emplace_back(c);
      }
    }
    // b
    {
      bool found1 = false;
      bool found2 = false;
      for (const auto& index : connectivity.at(b)) {
        if (a == index)
          found1 = true;
        if (c == index)
          found2 = true;
      }
      if (!found1) {
        connectivity.at(b).emplace_back(a);
      }
      if (!found2) {
        connectivity.at(b).emplace_back(c);
      }
    }
    // c
    {
      bool found1 = false;
      bool found2 = false;
      for (const auto& index : connectivity.at(c)) {
        if (a == index)
          found1 = true;
        if (b == index)
          found2 = true;
      }
      if (!found1) {
        connectivity.at(c).emplace_back(a);
      }
      if (!found2) {
        connectivity.at(c).emplace_back(b);
      }
    }
  }
  std::vector<glm::vec3> new_positions;
  std::vector<glm::vec2> new_uvs;
  for (int i = 0; i < vertices.size(); i++) {
    auto position = glm::vec3(0.0f);
    auto uv = glm::vec2(0.f);
    for (const auto& index : connectivity.at(i)) {
      const auto& vertex = vertices.at(index);
      position += vertex.position;
      uv += vertex.tex_coord;
    }
    new_positions.push_back(position / static_cast<float>(connectivity.at(i).size()));
    new_uvs.push_back(uv / static_cast<float>(connectivity.at(i).size()));
  }
  for (int i = 0; i < vertices.size(); i++) {
    if (vertices[i].position.y > 0.001f)
      vertices[i].position = new_positions[i];
    else {
      vertices[i].position.x = new_positions[i].x;
      vertices[i].position.z = new_positions[i].z;
    }
    vertices[i].tex_coord = new_uvs[i];
  }
}

void StrandModelMeshGenerator::MeshSmoothing(std::vector<Vertex>& vertices,
                                             std::vector<std::pair<unsigned, unsigned>>& indices) {
  std::vector<std::vector<unsigned>> connectivity;
  connectivity.resize(vertices.size());
  for (int i = 0; i < indices.size() / 3; i++) {
    auto a = indices[3 * i];
    auto b = indices[3 * i + 1];
    auto c = indices[3 * i + 2];
    // a
    {
      bool found1 = false;
      bool found2 = false;
      for (const auto& index : connectivity.at(a.first)) {
        if (b.first == index)
          found1 = true;
        if (c.first == index)
          found2 = true;
      }
      if (!found1) {
        connectivity.at(a.first).emplace_back(b.first);
      }
      if (!found2) {
        connectivity.at(a.first).emplace_back(c.first);
      }
    }
    // b
    {
      bool found1 = false;
      bool found2 = false;
      for (const auto& index : connectivity.at(b.first)) {
        if (a.first == index)
          found1 = true;
        if (c.first == index)
          found2 = true;
      }
      if (!found1) {
        connectivity.at(b.first).emplace_back(a.first);
      }
      if (!found2) {
        connectivity.at(b.first).emplace_back(c.first);
      }
    }
    // c
    {
      bool found1 = false;
      bool found2 = false;
      for (const auto& index : connectivity.at(c.first)) {
        if (a.first == index)
          found1 = true;
        if (b.first == index)
          found2 = true;
      }
      if (!found1) {
        connectivity.at(c.first).emplace_back(a.first);
      }
      if (!found2) {
        connectivity.at(c.first).emplace_back(b.first);
      }
    }
  }
  std::vector<glm::vec3> new_positions;
  std::vector<glm::vec2> new_uvs;
  for (int i = 0; i < vertices.size(); i++) {
    auto position = glm::vec3(0.0f);
    auto uv = glm::vec2(0.f);
    for (const auto& index : connectivity.at(i)) {
      const auto& vertex = vertices.at(index);
      position += vertex.position;
      uv += vertex.tex_coord;
    }
    new_positions.push_back(position / static_cast<float>(connectivity.at(i).size()));
    new_uvs.push_back(uv / static_cast<float>(connectivity.at(i).size()));
  }
  for (int i = 0; i < vertices.size(); i++) {
    if (vertices[i].position.y > 0.001f)
      vertices[i].position = new_positions[i];
    else {
      vertices[i].position.x = new_positions[i].x;
      vertices[i].position.z = new_positions[i].z;
    }
    vertices[i].tex_coord = new_uvs[i];
  }
}

void StrandModelMeshGenerator::CalculateNormal(std::vector<Vertex>& vertices, const std::vector<unsigned>& indices) {
  auto normal_lists = std::vector<std::vector<glm::vec3>>();
  const auto size = vertices.size();
  for (auto i = 0; i < size; i++) {
    normal_lists.emplace_back();
  }
  for (int i = 0; i < indices.size() / 3; i++) {
    const auto i1 = indices.at(i * 3);
    const auto i2 = indices.at(i * 3 + 1);
    const auto i3 = indices.at(i * 3 + 2);
    auto v1 = vertices[i1].position;
    auto v2 = vertices[i2].position;
    auto v3 = vertices[i3].position;
    auto normal = glm::normalize(glm::cross(v1 - v2, v1 - v3));
    normal_lists[i1].push_back(normal);
    normal_lists[i2].push_back(normal);
    normal_lists[i3].push_back(normal);
  }
  for (auto i = 0; i < size; i++) {
    auto normal = glm::vec3(0.0f);
    for (const auto j : normal_lists[i]) {
      normal += j;
    }
    vertices[i].normal = glm::normalize(normal);
  }
}

void StrandModelMeshGenerator::CalculateNormal(std::vector<Vertex>& vertices,
                                               const std::vector<std::pair<unsigned, unsigned>>& indices) {
  auto normal_lists = std::vector<std::vector<glm::vec3>>();
  const auto size = vertices.size();
  for (auto i = 0; i < size; i++) {
    normal_lists.emplace_back();
  }
  for (int i = 0; i < indices.size() / 3; i++) {
    const auto i1 = indices.at(i * 3);
    const auto i2 = indices.at(i * 3 + 1);
    const auto i3 = indices.at(i * 3 + 2);
    auto v1 = vertices[i1.first].position;
    auto v2 = vertices[i2.first].position;
    auto v3 = vertices[i3.first].position;
    auto normal = glm::normalize(glm::cross(v1 - v2, v1 - v3));
    normal_lists[i1.first].push_back(normal);
    normal_lists[i2.first].push_back(normal);
    normal_lists[i3.first].push_back(normal);
  }
  for (auto i = 0; i < size; i++) {
    auto normal = glm::vec3(0.0f);
    for (const auto j : normal_lists[i]) {
      normal += j;
    }
    vertices[i].normal = glm::normalize(normal);
  }
}

glm::vec3 ProjectVec3(const glm::vec3& a, const glm::vec3& dir) {
  return glm::normalize(dir) * glm::dot(a, dir) / glm::length(dir);
}

void StrandModelMeshGenerator::CalculateUv(const StrandModel& strand_model, std::vector<Vertex>& vertices,
                                           const StrandModelMeshGeneratorSettings& settings) {
  if (settings.fast_uv) {
    const auto& sorted_node_list = strand_model.strand_model_skeleton.PeekSortedNodeList();

    Jobs::RunParallelFor(vertices.size(), [&](unsigned vertex_index) {
      auto& vertex = vertices.at(vertex_index);

      float min_distance = FLT_MAX;
      SkeletonNodeHandle closest_node_handle = -1;

      for (const auto& node_handle : sorted_node_list) {
        const auto& node = strand_model.strand_model_skeleton.PeekNode(node_handle);
        const auto& profile = node.data.profile;
        if (profile.PeekParticles().size() < settings.min_cell_count_for_major_branches)
          continue;
        const auto node_start = node.info.global_position;
        const auto node_end = node.info.GetGlobalEndPosition();
        const auto closest_point = glm::closestPointOnLine(vertex.position, node_start, node_end);
        if (glm::dot(node_end - node_start, closest_point - node_start) <= 0.f ||
            glm::dot(node_start - node_end, closest_point - node_end) <= 0.f)
          continue;
        if (const auto current_distance = glm::distance(closest_point, vertex.position) / node.info.thickness;
            current_distance < min_distance) {
          min_distance = current_distance;
          closest_node_handle = node_handle;
        }
      }
      if (closest_node_handle != -1) {
        const auto closest_node = strand_model.strand_model_skeleton.PeekNode(closest_node_handle);
        const float end_point_root_distance = closest_node.info.root_distance;
        const float start_point_root_distance = closest_node.info.root_distance - closest_node.info.length;
        const auto closest_point = glm::closestPointOnLine(vertex.position, closest_node.info.global_position,
                                                           closest_node.info.GetGlobalEndPosition());
        const float distance_to_start = glm::distance(closest_point, closest_node.info.global_position);
        const float a = closest_node.info.length == 0 ? 1.f : distance_to_start / closest_node.info.length;
        const float root_distance = glm::mix(start_point_root_distance, end_point_root_distance, a);
        vertex.tex_coord.y = root_distance * settings.root_distance_multiplier;
        const auto v = glm::normalize(vertex.position - closest_point);
        const auto up = closest_node.info.regulated_global_rotation * glm::vec3(0, 1, 0);
        const auto left = closest_node.info.regulated_global_rotation * glm::vec3(1, 0, 0);
        const auto proj_up = ProjectVec3(v, up);
        const auto proj_left = ProjectVec3(v, left);
        const glm::vec2 position = glm::vec2(glm::length(proj_left) * (glm::dot(proj_left, left) > 0.f ? 1.f : -1.f),
                                             glm::length(proj_up) * (glm::dot(proj_up, up) > 0.f ? 1.f : -1.f));
        const float acos_val = glm::acos(position.x / glm::length(position));
        vertex.tex_coord.x = acos_val / glm::pi<float>();
      } else {
        vertex.tex_coord = glm::vec2(0.0f);
      }
    });
  } else {
    const auto& strand_group = strand_model.strand_model_skeleton.data.strand_group;
    auto min = glm::vec3(FLT_MAX);
    auto max = glm::vec3(FLT_MIN);
    for (StrandSegmentHandle strand_segment_handle = 0; strand_segment_handle < strand_group.PeekStrandSegments().size();
         strand_segment_handle++) {
      const auto& strand_segment = strand_group.PeekStrandSegment(strand_segment_handle);
      const auto& strand_segment_data = strand_group.PeekStrandSegmentData(strand_segment_handle);

      const auto& node = strand_model.strand_model_skeleton.PeekNode(strand_segment_data.node_handle);
      const auto& profile = node.data.profile;
      if (profile.PeekParticles().size() < settings.min_cell_count_for_major_branches)
        continue;
      const auto segment_start = strand_group.GetStrandSegmentStart(strand_segment_handle);
      const auto segment_end = strand_segment.end_position;
      min = glm::min(segment_start, min);
      min = glm::min(segment_start, min);
      max = glm::max(segment_end, max);
      max = glm::max(segment_end, max);
    }
    min -= glm::vec3(0.1f);
    max += glm::vec3(0.1f);
    VoxelGrid<std::vector<StrandSegmentHandle>> boundary_segments;
    boundary_segments.Initialize(0.01f, min, max, {});

    for (StrandSegmentHandle strand_segment_handle = 0; strand_segment_handle < strand_group.PeekStrandSegments().size();
         strand_segment_handle++) {
      const auto& strand_segment = strand_group.PeekStrandSegment(strand_segment_handle);
      const auto& strand_segment_data = strand_group.PeekStrandSegmentData(strand_segment_handle);
      const auto& node = strand_model.strand_model_skeleton.PeekNode(strand_segment_data.node_handle);
      const auto& profile = node.data.profile;
      if (profile.PeekParticles().size() < settings.min_cell_count_for_major_branches)
        continue;

      const auto segment_start = strand_group.GetStrandSegmentStart(strand_segment_handle);
      const auto segment_end = strand_segment.end_position;
      boundary_segments.Ref((segment_start + segment_end) * 0.5f).emplace_back(strand_segment_handle);
    }

    Jobs::RunParallelFor(vertices.size(), [&](unsigned vertex_index) {
      auto& vertex = vertices.at(vertex_index);
      float min_distance = FLT_MAX;
      StrandSegmentHandle closest_segment_handle = -1;
      boundary_segments.ForEach(vertex.position, 0.05f, [&](const std::vector<StrandSegmentHandle>& segment_handles) {
        for (const auto& segment_handle : segment_handles) {
          const auto& segment = strand_group.PeekStrandSegment(segment_handle);
          const auto segment_start = strand_group.GetStrandSegmentStart(segment_handle);
          const auto segment_end = segment.end_position;
          const auto closest_point = glm::closestPointOnLine(vertex.position, segment_start, segment_end);
          if (glm::dot(segment_end - segment_start, closest_point - segment_start) <= 0.f ||
              glm::dot(segment_start - segment_end, closest_point - segment_end) <= 0.f)
            continue;
          if (const auto current_distance = glm::distance(segment_end, vertex.position);
              current_distance < min_distance) {
            min_distance = current_distance;
            closest_segment_handle = segment_handle;
          }
        }
      });
      SkeletonNodeHandle closest_node_handle = -1;
      if (closest_segment_handle != -1) {
        const auto segment_data = strand_group.PeekStrandSegmentData(closest_segment_handle);
        closest_node_handle = segment_data.node_handle;
      }
      if (closest_node_handle != -1) {
        const auto closest_node = strand_model.strand_model_skeleton.PeekNode(closest_node_handle);
        const float end_point_root_distance = closest_node.info.root_distance;
        const float start_point_root_distance = closest_node.info.root_distance - closest_node.info.length;
        const auto closest_point = glm::closestPointOnLine(vertex.position, closest_node.info.global_position,
                                                           closest_node.info.GetGlobalEndPosition());
        const float distance_to_start = glm::distance(closest_point, closest_node.info.global_position);
        const float a = closest_node.info.length == 0 ? 1.f : distance_to_start / closest_node.info.length;
        const float root_distance = glm::mix(start_point_root_distance, end_point_root_distance, a);
        vertex.tex_coord.y = root_distance * settings.root_distance_multiplier;
        const auto v = glm::normalize(vertex.position - closest_point);
        const auto up = closest_node.info.regulated_global_rotation * glm::vec3(0, 1, 0);
        const auto left = closest_node.info.regulated_global_rotation * glm::vec3(1, 0, 0);
        const auto proj_up = ProjectVec3(v, up);
        const auto proj_left = ProjectVec3(v, left);
        const glm::vec2 position = glm::vec2(glm::length(proj_left) * (glm::dot(proj_left, left) > 0.f ? 1.f : -1.f),
                                             glm::length(proj_up) * (glm::dot(proj_up, up) > 0.f ? 1.f : -1.f));
        const float acos_val = glm::acos(position.x / glm::length(position));
        vertex.tex_coord.x = acos_val / glm::pi<float>();
      } else {
        vertex.tex_coord = glm::vec2(0.0f);
      }
    });
  }
}
