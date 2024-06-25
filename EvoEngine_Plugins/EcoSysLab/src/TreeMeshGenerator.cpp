//
// Created by lllll on 10/27/2022.
//

#include "TreeMeshGenerator.hpp"
#include "EditorLayer.hpp"
#include "Tree.hpp"

using namespace eco_sys_lab;

RingSegment::RingSegment(const float start_a, const float end_a, glm::vec3 start_position, glm::vec3 end_position,
                         glm::vec3 start_axis, glm::vec3 end_axis, float start_radius, float end_radius,
                         float start_distance_to_root, float end_distance_to_root)
    : start_a(start_a),
      end_a(end_a),
      start_position(start_position),
      end_position(end_position),
      start_axis(start_axis),
      end_axis(end_axis),
      start_radius(start_radius),
      end_radius(end_radius),
      start_distance_to_root(start_distance_to_root),
      end_distance_to_root(end_distance_to_root) {
}

void RingSegment::AppendPoints(std::vector<Vertex>& vertices, glm::vec3& normal_dir, int step) {
  std::vector<Vertex> start_ring;
  std::vector<Vertex> end_ring;

  float angle_step = 360.0f / (float)(step);
  Vertex archetype;
  for (int i = 0; i < step; i++) {
    archetype.position = GetPoint(normal_dir, angle_step * i, true);
    start_ring.push_back(archetype);
  }
  for (int i = 0; i < step; i++) {
    archetype.position = GetPoint(normal_dir, angle_step * i, false);
    end_ring.push_back(archetype);
  }
  float texture_x_step = 1.0f / step * 4;
  for (int i = 0; i < step - 1; i++) {
    float x = (i % step) * texture_x_step;
    start_ring[i].tex_coord = glm::vec2(x, 0.0f);
    start_ring[i + 1].tex_coord = glm::vec2(x + texture_x_step, 0.0f);
    end_ring[i].tex_coord = glm::vec2(x, 1.0f);
    end_ring[i + 1].tex_coord = glm::vec2(x + texture_x_step, 1.0f);
    vertices.push_back(start_ring[i]);
    vertices.push_back(start_ring[i + 1]);
    vertices.push_back(end_ring[i]);
    vertices.push_back(end_ring[i + 1]);
    vertices.push_back(end_ring[i]);
    vertices.push_back(start_ring[i + 1]);
  }
  start_ring[step - 1].tex_coord = glm::vec2(1.0f - texture_x_step, 0.0f);
  start_ring[0].tex_coord = glm::vec2(1.0f, 0.0f);
  end_ring[step - 1].tex_coord = glm::vec2(1.0f - texture_x_step, 1.0f);
  end_ring[0].tex_coord = glm::vec2(1.0f, 1.0f);
  vertices.push_back(start_ring[step - 1]);
  vertices.push_back(start_ring[0]);
  vertices.push_back(end_ring[step - 1]);
  vertices.push_back(end_ring[0]);
  vertices.push_back(end_ring[step - 1]);
  vertices.push_back(start_ring[0]);
}

glm::vec3 RingSegment::GetPoint(const glm::vec3& normal_dir, const float angle, const bool is_start,
                                const float multiplier) const {
  const auto direction = GetDirection(normal_dir, angle, is_start);
  const auto radius = is_start ? start_radius : end_radius;
  const glm::vec3 position = (is_start ? start_position : end_position) + direction * multiplier * radius;
  return position;
}

glm::vec3 RingSegment::GetDirection(const glm::vec3& normal_dir, float angle, const bool is_start) const {
  glm::vec3 direction = glm::cross(normal_dir, is_start ? this->start_axis : this->end_axis);
  direction = glm::rotate(direction, glm::radians(angle), is_start ? this->start_axis : this->end_axis);
  direction = glm::normalize(direction);
  return direction;
}

void TreeMeshGeneratorSettings::Save(const std::string& name, YAML::Emitter& out) {
  out << YAML::Key << name << YAML::Value << YAML::BeginMap;
  out << YAML::Key << "x_subdivision" << YAML::Value << x_subdivision;
  out << YAML::Key << "trunk_y_subdivision" << YAML::Value << trunk_y_subdivision;
  out << YAML::Key << "trunk_thickness" << YAML::Value << trunk_thickness;
  out << YAML::Key << "branch_y_subdivision" << YAML::Value << branch_y_subdivision;

  out << YAML::Key << "enable_foliage" << YAML::Value << enable_foliage;
  out << YAML::Key << "foliage_instancing" << YAML::Value << foliage_instancing;
  out << YAML::Key << "enable_branch" << YAML::Value << enable_branch;
  out << YAML::Key << "enable_fruit" << YAML::Value << enable_fruit;

  out << YAML::Key << "stitch_all_children" << YAML::Value << stitch_all_children;
  out << YAML::Key << "smoothness" << YAML::Value << smoothness;
  out << YAML::Key << "override_radius" << YAML::Value << override_radius;
  out << YAML::Key << "radius" << YAML::Value << radius;
  out << YAML::Key << "radius_multiplier" << YAML::Value << radius_multiplier;
  out << YAML::Key << "base_control_point_ratio" << YAML::Value << base_control_point_ratio;
  out << YAML::Key << "branch_control_point_ratio" << YAML::Value << branch_control_point_ratio;
  out << YAML::Key << "tree_part_end_distance" << YAML::Value << tree_part_end_distance;
  out << YAML::Key << "tree_part_base_distance" << YAML::Value << tree_part_base_distance;

  out << YAML::Key << "auto_level" << YAML::Value << auto_level;
  out << YAML::Key << "voxel_subdivision_level" << YAML::Value << voxel_subdivision_level;
  out << YAML::Key << "voxel_smooth_iteration" << YAML::Value << voxel_smooth_iteration;
  out << YAML::Key << "remove_duplicate" << YAML::Value << remove_duplicate;

  out << YAML::Key << "branch_mesh_type" << YAML::Value << branch_mesh_type;

  out << YAML::EndMap;
}

void TreeMeshGeneratorSettings::Load(const std::string& name, const YAML::Node& in) {
  if (in[name]) {
    const auto& ms = in[name];
    if (ms["x_subdivision"])
      x_subdivision = ms["x_subdivision"].as<float>();
    if (ms["trunk_y_subdivision"])
      trunk_y_subdivision = ms["trunk_y_subdivision"].as<float>();
    if (ms["trunk_thickness"])
      trunk_thickness = ms["trunk_thickness"].as<float>();
    if (ms["branch_y_subdivision"])
      branch_y_subdivision = ms["branch_y_subdivision"].as<float>();

    if (ms["enable_foliage"])
      enable_foliage = ms["enable_foliage"].as<bool>();
    if (ms["foliage_instancing"])
      foliage_instancing = ms["foliage_instancing"].as<bool>();
    if (ms["enable_branch"])
      enable_branch = ms["enable_branch"].as<bool>();
    if (ms["enable_fruit"])
      enable_fruit = ms["enable_fruit"].as<bool>();

    if (ms["stitch_all_children"])
      stitch_all_children = ms["stitch_all_children"].as<bool>();
    if (ms["smoothness"])
      smoothness = ms["smoothness"].as<bool>();
    if (ms["override_radius"])
      override_radius = ms["override_radius"].as<bool>();
    if (ms["radius_multiplier"])
      radius_multiplier = ms["radius_multiplier"].as<float>();
    if (ms["radius"])
      radius = ms["radius"].as<float>();
    if (ms["base_control_point_ratio"])
      base_control_point_ratio = ms["base_control_point_ratio"].as<float>();
    if (ms["branch_control_point_ratio"])
      branch_control_point_ratio = ms["branch_control_point_ratio"].as<float>();
    if (ms["tree_part_end_distance"])
      tree_part_end_distance = ms["tree_part_end_distance"].as<float>();
    if (ms["tree_part_base_distance"])
      tree_part_base_distance = ms["tree_part_base_distance"].as<float>();

    if (ms["auto_level"])
      auto_level = ms["auto_level"].as<bool>();
    if (ms["voxel_subdivision_level"])
      voxel_subdivision_level = ms["voxel_subdivision_level"].as<int>();
    if (ms["voxel_smooth_iteration"])
      voxel_smooth_iteration = ms["voxel_smooth_iteration"].as<int>();
    if (ms["remove_duplicate"])
      remove_duplicate = ms["remove_duplicate"].as<bool>();

    if (ms["branch_mesh_type"])
      branch_mesh_type = ms["branch_mesh_type"].as<unsigned>();
  }
}

void TreeMeshGeneratorSettings::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  if (ImGui::TreeNodeEx("Mesh Generator settings")) {
    ImGui::Checkbox("Branch", &enable_branch);
    ImGui::Checkbox("Fruit", &enable_fruit);
    ImGui::Checkbox("Foliage", &enable_foliage);
    ImGui::Checkbox("Foliage instancing", &foliage_instancing);
    ImGui::Combo("Branch mesh mode", {"Cylindrical", "Marching cubes"}, branch_mesh_type);

    ImGui::Combo("Branch color mode", {"Internode Color", "Junction"}, vertex_color_mode);

    if (ImGui::TreeNode("Cylindrical mesh settings")) {
      ImGui::Checkbox("Stitch all children", &stitch_all_children);
      ImGui::DragFloat("Trunk Thickness Threshold", &trunk_thickness, 1.0f, 0.0f, 16.0f);
      ImGui::DragFloat("X Step", &x_subdivision, 0.00001f, 0.00001f, 1.0f, "%.5f");
      ImGui::DragFloat("Trunk Y Step", &trunk_y_subdivision, 0.00001f, 0.00001f, 1.0f, "%.5f");
      ImGui::DragFloat("Branch Y Step", &branch_y_subdivision, 0.00001f, 0.00001f, 1.0f, "%.5f");

      ImGui::Checkbox("Smoothness", &smoothness);
      if (smoothness) {
        ImGui::DragFloat("Base control point ratio", &base_control_point_ratio, 0.001f, 0.0f, 1.0f);
        ImGui::DragFloat("Branch control point ratio", &branch_control_point_ratio, 0.001f, 0.0f, 1.0f);
      }
      ImGui::Checkbox("Override radius", &override_radius);
      if (override_radius)
        ImGui::DragFloat("Radius", &radius);
      ImGui::DragFloat("Radius multiplier", &radius_multiplier, 0.01f, 0.01f, 100.f);
      ImGui::DragFloat("Tree Part Base Distance", &tree_part_base_distance, 1, 0, 10);
      ImGui::DragFloat("Tree Part End Distance", &tree_part_end_distance, 1, 0, 10);
      ImGui::TreePop();
    }
    if (ImGui::TreeNode("Marching cubes settings")) {
      ImGui::Checkbox("Auto set level", &auto_level);
      if (!auto_level)
        ImGui::DragInt("Voxel subdivision level", &voxel_subdivision_level, 1, 5, 16);
      else
        ImGui::DragFloat("Min Cube size", &marching_cube_radius, 0.0001, 0.001f, 1.0f);
      ImGui::DragInt("Smooth iteration", &voxel_smooth_iteration, 0, 0, 10);
      if (voxel_smooth_iteration == 0)
        ImGui::Checkbox("Remove duplicate", &remove_duplicate);
      ImGui::TreePop();
    }
    if (enable_branch && ImGui::TreeNode("Branch settings")) {
      ImGui::TreePop();
    }
    if (enable_foliage && ImGui::TreeNode("Foliage settings")) {
      ImGui::TreePop();
    }

    ImGui::Checkbox("Mesh Override", &presentation_override);
    if (presentation_override && ImGui::TreeNodeEx("Override settings")) {
      ImGui::DragFloat("Max thickness", &presentation_override_settings.max_thickness, 0.01f, 0.0f, 1.0f);

      ImGui::TreePop();
    }
    ImGui::TreePop();
  }
}
