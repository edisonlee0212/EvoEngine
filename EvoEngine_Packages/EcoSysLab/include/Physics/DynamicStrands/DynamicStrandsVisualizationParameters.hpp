#pragma once

namespace eco_sys_lab_package {
using namespace evo_engine;

// TODO: move everything related to uniform particles to a separate struct
struct DynamicStrandsVisualizationParameters {
  enum class SegmentRenderMode {
    Default,
    NodeColor,
    GroupIndex,
    BoundaryDistance,
    Strength,
    ShearStretchStrain,
    StretchShearLimit,
    SegmentColor,
    StrandColor,
    Test
  };

  enum class SegmentPairRenderMode {
    Default,
    BendingStrain,
    TwistStrain,
    BundleStrain,
    BendingTwistingBundleStrain,
    ConnectivityStrain,

    BendingLimit,
    TwistLimit,
    BundleLimit,
    ConnectivityLimit,
    SegmentColor
  };

  bool render_segments = true;
  bool render_segment_pairs = false;
  bool render_foliage = false;

  uint32_t segment_render_mode = 9;
  uint32_t segment_pair_render_mode = 5;
  uint32_t foliage_render_mode = 0;

  glm::vec4 segment_color_min = glm::vec4(0, 0, 1, 1);
  glm::vec4 segment_color_max = glm::vec4(1, 0, 0, 1);
  glm::vec4 segment_color_main = glm::vec4(0.3, 0.15, 0.0, 0.5);
  float segment_radius_multiplier = 0.9f;
  float segment_boundary_distance_modular = 0.03f;
  float segment_length_multiplier = 1.0f;
  float general_factor = 1.0f;

  glm::vec4 segment_pair_color_min = glm::vec4(0, 0, 1, 1);
  glm::vec4 segment_pair_color_max = glm::vec4(1, 0, 0, 1);
  glm::vec4 segment_pair_color_main = glm::vec4(0, 1, 1, 0.2);
  float segment_pair_radius_multiplier = 0.9f;

  glm::vec4 foliage_color_min = glm::vec4(0, 0, 1, 1);
  glm::vec4 foliage_color_max = glm::vec4(1, 0, 0, 1);

  glm::vec4 foliage_color_main = glm::vec4(0, 1, 0, 1);

  bool DrawGui(const std::shared_ptr<EditorLayer>& editor_layer);
};
}  // namespace eco_sys_lab_package