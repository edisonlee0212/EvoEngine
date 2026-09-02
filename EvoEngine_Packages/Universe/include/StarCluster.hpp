#pragma once

#include "IPrivateComponent.hpp"

namespace evo_engine {
struct InspectorContext;
}

namespace universe_package {
using namespace evo_engine;

class StarCluster final : public IPrivateComponent {
  friend bool InspectStarCluster(InspectorContext& context, StarCluster& cluster);
  friend void SerializeStarCluster(YAML::Emitter& out, const StarCluster& cluster);
  friend void DeserializeStarCluster(const YAML::Node& in, StarCluster& cluster);

 public:
  uint64_t seed = 1;
  double y_spread = 0.05;
  double xz_spread = 0.015;
  double disk_diameter = 30000.0;
  double disk_eccentricity = 0.5;
  double core_proportion = 0.4;
  double core_eccentricity = 0.7;
  double center_diameter = 10.0;
  double center_eccentricity = 0.3;
  double disk_speed = 1.0;
  double core_speed = 5.0;
  double center_speed = 10.0;
  double disk_tilt_x = 0.0;
  double disk_tilt_z = 0.0;
  double core_tilt_x = 0.0;
  double core_tilt_z = 0.0;
  double center_tilt_x = 0.0;
  double center_tilt_z = 0.0;
  double twist = 360.0;
  glm::dvec3 center_offset{};
  glm::dvec3 center_position{};
  glm::vec3 disk_color{0.0f, 0.0f, 1.0f};
  glm::vec3 core_color{1.0f, 1.0f, 0.0f};
  glm::vec3 center_color{1.0f};
  float disk_emission_intensity = 8.0f;
  float core_emission_intensity = 8.0f;
  float center_emission_intensity = 8.0f;
  float alpha = 1.0f;
  double visual_radius = 1.0;
  double radius_standard_deviation = 0.0;
  double radius_min = 0.1;
  double radius_max = 2.0;
  double time_scale = 0.1;
  double phase = 1000000.0;
  bool paused = false;

  void SetStarCount(uint32_t count);
  [[nodiscard]] uint32_t GetStarCount() const;
  void OnCreate() override;
  void OnDestroy() override;

 private:
  uint32_t star_count_ = 0;
  void ResetAuthoringState();
};

bool InspectStarCluster(InspectorContext& context, StarCluster& cluster);
void SerializeStarCluster(YAML::Emitter& out, const StarCluster& cluster);
void DeserializeStarCluster(const YAML::Node& in, StarCluster& cluster);
}  // namespace universe_package
