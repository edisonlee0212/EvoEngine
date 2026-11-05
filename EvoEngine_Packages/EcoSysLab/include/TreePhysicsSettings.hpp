#pragma once

#include <array>
#include <glm/glm.hpp>
#include <tuple>

namespace YAML {
class Emitter;
class Node;
}  // namespace YAML
namespace eco_sys_lab_package {
// Size, branch-order, and combined rules are alternatives selected by mask bits 0..2.
struct TreeClusterLodSettings {
  uint32_t enabled_policies = 0;
  std::array<glm::vec4, 3> rules{glm::vec4(0.15f, 0.25f, 2, 2), glm::vec4(0.15f, 0.25f, 2, 2),
                                 glm::vec4(0.15f, 0.25f, 2, 2)};
};

struct TreePhysicsProperties {
  float stretch_stiffness = 8192.0f;
  float bend_stiffness = 512.0f;
  float bend_thickness_exponent = 3.5f;
  float bend_downstream_exponent = 1.5f;
  bool use_local_rod_mass = false;
  float density = 700.0f;
  float damping = 0.0f;
  float cluster_bend_damping_time = 0.0f;
  float wind_drag = 20.0f;
  float contact_friction = 0.6f;
  glm::vec3 gravity{0, -9.8f, 0};
  glm::vec3 anchor_offset{};
  bool fracture_enabled = false;
  float fracture_angle_degrees = 45.0f;
  float fracture_duration = 0.05f;
  bool operator==(const TreePhysicsProperties& other) const {
    return std::tie(stretch_stiffness, bend_stiffness, bend_thickness_exponent, bend_downstream_exponent,
                    use_local_rod_mass, density, damping, cluster_bend_damping_time, wind_drag, contact_friction,
                    gravity, anchor_offset, fracture_enabled, fracture_angle_degrees, fracture_duration) ==
           std::tie(other.stretch_stiffness, other.bend_stiffness, other.bend_thickness_exponent,
                    other.bend_downstream_exponent, other.use_local_rod_mass, other.density, other.damping,
                    other.cluster_bend_damping_time, other.wind_drag, other.contact_friction, other.gravity,
                    other.anchor_offset, other.fracture_enabled, other.fracture_angle_degrees, other.fracture_duration);
  }
};
struct TreePhysicsQuality {
  float time_step = 0.016f;
  int substeps = 4;
  int iterations = 8;
  float approximation_tolerance = 0.0f;
  bool use_subtree_clusters = false;
  TreeClusterLodSettings cluster_lod;
};
void SerializeTreeClusterLodSettings(YAML::Emitter& out, const TreeClusterLodSettings& settings);
void DeserializeTreeClusterLodSettings(const YAML::Node& in, TreeClusterLodSettings& settings);
void SerializeTreePhysicsProperties(YAML::Emitter& out, const TreePhysicsProperties& properties);
void DeserializeTreePhysicsProperties(const YAML::Node& in, TreePhysicsProperties& properties);
}  // namespace eco_sys_lab_package
