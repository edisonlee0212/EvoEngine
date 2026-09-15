#pragma once
#include "CBTFGroup.hpp"
#include "CurveEditors.hpp"
#include "DigitalAgricultureInspectionAdapters.hpp"
#include "EditorLayer.hpp"
#include "PARSensorGroup.hpp"
#include "SkyIlluminance.hpp"
#include "Sorghum.hpp"
#include "SorghumField.hpp"
#include "SorghumGenerator.hpp"
#include "SorghumGrowthStages.hpp"
#include "SorghumState.hpp"
namespace digital_agriculture_package {
using namespace evo_engine;
struct CBTFGroupInspector {
  AssetRef ui_temp{};
  bool Inspect(evo_engine::InspectorContext& context, CBTFGroup& group);
};
struct PARSensorGroupInspector {
  glm::vec3 ui_min_range = glm::vec3(-25, 0, -25);
  glm::vec3 ui_max_range = glm::vec3(25, 3, 25);
  float ui_step = 3.0f;
  bool ui_draw = true;
  float ui_line_width = 0.05f;
  float ui_line_length_factor = 3.0f;
  float ui_point_size = 0.1f;
  std::vector<glm::vec3> ui_starts{};
  std::vector<glm::vec3> ui_ends{};
  std::shared_ptr<ParticleInfoList> ui_ray_particle_info_list{};
  std::shared_ptr<ParticleInfoList> ui_point_particle_info_list{};
  glm::vec4 ui_color = {0.0f, 1.0f, 0.0f, 0.5f};
  glm::vec4 ui_point_color = {1.0f, 0.0f, 0.0f, 0.75f};
  bool Inspect(evo_engine::InspectorContext& context, PARSensorGroup& group);
};
struct SkyIlluminanceInspector {
  float ui_time{};
  SkyIlluminanceSnapshot ui_snapshot{};
  bool Inspect(evo_engine::InspectorContext& context, SkyIlluminance& illuminance);
};
struct SorghumInspector {
  std::weak_ptr<IPrivateComponent> preview_owner;
  int ui_seed = 0;
  float ui_time = 0.0f;
  bool ui_debug_rendering = false;
  float ui_node_render_size = .5f;
  Entity ui_previous_referenced_entity{};
  std::shared_ptr<ParticleInfoList> ui_node_debug_info_list{};
  bool Inspect(evo_engine::InspectorContext& context, Sorghum& sorghum);
};
struct SorghumFieldInspector {
  int ui_index = 200;
  float ui_radius = 2.5f;
  AssetRef ui_temp_coordinates{};
  bool Inspect(evo_engine::InspectorContext& context, SorghumField& field);
};
struct SorghumGeneratorInspector {
  bool ui_auto_save = true;
  bool ui_intro = true;
  PlottedDistributionSettings ui_leaf_starting_point = {0.01f,
                                                        {0.01f, false, true, ""},
                                                        {0.01f, false, false, ""},
                                                        "The starting point of each leaf along stem. Default each leaf "
                                                        "located uniformly on stem."};
  PlottedDistributionSettings ui_leaf_curling = {
      0.01f, {0.01f, false, true, ""}, {0.01f, false, false, ""}, "The leaf curling."};
  PlottedDistributionSettings ui_leaf_roll_angle = {0.01f,
                                                    {},
                                                    {},
                                                    "The polar angle of leaf. Normally you should only change the "
                                                    "deviation. Values are in degrees"};
  PlottedDistributionSettings ui_leaf_branching_angle = {
      0.01f, {}, {}, "The branching angle of the leaf. Values are in degrees"};
  PlottedDistributionSettings ui_leaf_bending = {1.0f,
                                                 {1.0f, false, true, ""},
                                                 {},
                                                 "The bending of the leaf, controls how leaves bend because of "
                                                 "gravity. Positive value results in leaf bending towards the "
                                                 "ground, negative value results in leaf bend towards the sky"};
  PlottedDistributionSettings ui_leaf_bending_acceleration = {
      0.01f, {0.01f, false, true, ""}, {}, "The changes of bending along the leaf."};
  PlottedDistributionSettings ui_leaf_bending_smoothness = {
      0.01f, {0.01f, false, true, ""}, {}, "The smoothness of bending along the leaf."};
  double ui_last_auto_save_time = 0;
  float ui_auto_save_interval = 5;
  bool Inspect(evo_engine::InspectorContext& context, SorghumGenerator& generator);
};
struct SorghumGrowthStagesInspector {
  bool ui_auto_save = false;
  double ui_last_auto_save_time = 0;
  float ui_auto_save_interval = 10;
  const char* ui_state_modes[2]{"Default", "Cubic-Bezier"};
  char ui_new_name[256]{};
  int ui_seed = 0;
  AssetRef ui_descriptor{};
  bool Inspect(evo_engine::InspectorContext& context, SorghumGrowthStages& growth_stages);
};
struct SorghumStateInspector {
  float ui_target_waviness_factor = 1.0f;
  int ui_state_mode = static_cast<int>(StateMode::Default);
  const char* ui_state_modes[2]{"Default", "Cubic-Bezier"};
  bool Inspect(evo_engine::InspectorContext& context, SorghumState& state);
};
}  // namespace digital_agriculture_package
