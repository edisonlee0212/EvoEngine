#pragma once

#include "Climate.hpp"
#include "Soil.hpp"
#include "Strands.hpp"
#include "Tree.hpp"

using namespace evo_engine;
namespace eco_sys_lab {
class SimulationSettings {
 public:
  float delta_time = 0.0822f;
  bool soil_simulation = false;
  bool auto_clear_fruit_and_leaves = true;
  float crown_shyness_distance = 0.15f;
  int max_node_count = 0;

  float skylight_intensity = 1.f;

  float shadow_distance_loss = 1.f;
  float detection_radius = 0.5f;

  float environment_light_intensity = 0.01f;

  int blur_iteration = 0;

  void Save(const std::string& name, YAML::Emitter& out) const;
  void Load(const std::string& name, const YAML::Node& in);
  void Serialize(YAML::Emitter& out) const;
  void Deserialize(const YAML::Node& in);
  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer);
};
}  // namespace eco_sys_lab