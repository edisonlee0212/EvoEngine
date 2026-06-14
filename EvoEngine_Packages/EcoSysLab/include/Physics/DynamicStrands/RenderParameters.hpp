#pragma once

namespace eco_sys_lab_package {
using namespace evo_engine;

struct BranchesRenderParameters {
  bool enabled = true;
  bool render_complex = false;
  bool use_cgal = false;
  bool solid = true;
  bool wireframe = false;
  float alpha = 0.00005f;
  float bifurcation_alpha = 0.00005f;
  float max_dist_squared = 1.0f;
  bool use_cubic_hermite_spline = true;

  enum VertexColors {
    Default,
    Normals,
    Tangents,
    Groups,
    Degree,
    Bark,
    NormalQuaternion,
    Up,
    InitUp,
    Axis,
    InitAxis,
    InitAngle
  };

  VertexColors vertex_colors = Default;

  float u_multiplier = 1;
  float v_multiplier = 0.025;
  float degen_triangle_threshold_logairthmic = 5.0f;
  float global_extrusion_distance = 0.002f;
  float break_threshold = 0.01f;

  bool persistent_damage = false;
  bool use_polar_coordinates_for_uv = true;
  bool DrawGui(const std::shared_ptr<EditorLayer>& editor_layer);
};

struct SmallSegmentsRenderParameters {
  bool enabled = true;
  bool cast_shadow = true;
  bool wireframe = false;
  float thickness_multiplier = 0.5f;
  glm::vec3 position_scale = glm::vec3(1.f);
  bool DrawGui(const std::shared_ptr<EditorLayer>& editor_layer);
};

struct SmallSegmentsVisualizationRenderParameters {
  bool enabled = true;
  float thickness_multiplier = 0.5f;

  glm::vec4 segment_color_min = glm::vec4(0, 0, 1, 1);
  glm::vec4 segment_color_max = glm::vec4(1, 0, 0, 1);
  glm::vec4 segment_color_main = glm::vec4(0.3, 0.15, 0.0, 0.5);
  uint32_t segment_render_mode = 6;
  float segment_boundary_distance_modular = 0.03f;
  glm::vec3 position_scale = glm::vec3(1.f);
  bool DrawGui(const std::shared_ptr<EditorLayer>& editor_layer);
};

struct SegmentPairsRenderParameters {
  bool enabled = false;
  float thickness_multiplier = 0.5f;
  uint32_t segment_pair_render_mode = 5;
  glm::vec3 position_scale = glm::vec3(1.f);

  glm::vec4 segment_pair_color_min = glm::vec4(0, 0, 1, 1);
  glm::vec4 segment_pair_color_max = glm::vec4(1, 0, 0, 1);
  glm::vec4 segment_pair_color_main = glm::vec4(0, 1, 1, 0.2);
  float segment_pair_radius_multiplier = 0.9f;

  bool DrawGui(const std::shared_ptr<EditorLayer>& editor_layer);
};

struct FoliageRenderParameters {
  bool enabled = true;
  bool wireframe = false;
  bool DrawGui(const std::shared_ptr<EditorLayer>& editor_layer);
};
};  // namespace eco_sys_lab_package