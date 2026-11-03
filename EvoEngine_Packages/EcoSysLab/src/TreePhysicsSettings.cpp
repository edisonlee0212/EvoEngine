#include "TreePhysicsSettings.hpp"
#include <algorithm>
#include "Serialization.hpp"

namespace eco_sys_lab_package {
using evo_engine::operator<<;
void SerializeTreeClusterLodSettings(YAML::Emitter& out, const TreeClusterLodSettings& settings) {
  out << YAML::BeginMap << YAML::Key << "enabled_policies" << YAML::Value << settings.enabled_policies;
  out << YAML::Key << "rules" << YAML::Value << YAML::BeginSeq;
  for (const auto& rule : settings.rules)
    out << rule;
  out << YAML::EndSeq << YAML::EndMap;
}
void DeserializeTreeClusterLodSettings(const YAML::Node& in, TreeClusterLodSettings& settings) {
  if (!in)
    return;
  if (in["enabled_policies"])
    settings.enabled_policies = in["enabled_policies"].as<uint32_t>() & 7u;
  if (const auto rules = in["rules"]; rules && rules.IsSequence())
    for (size_t i = 0; i < std::min<size_t>(rules.size(), 3); ++i) {
      auto value = rules[i].as<glm::vec4>();
      settings.rules[i] = glm::max(value, glm::vec4(0, 0, 0, 2));
    }
}
void SerializeTreePhysicsProperties(YAML::Emitter& out, const TreePhysicsProperties& p) {
  out << YAML::Key << "stretch_stiffness" << YAML::Value << p.stretch_stiffness;
  out << YAML::Key << "bend_stiffness" << YAML::Value << p.bend_stiffness;
  out << YAML::Key << "density" << YAML::Value << p.density;
  out << YAML::Key << "damping" << YAML::Value << p.damping;
  out << YAML::Key << "wind_drag" << YAML::Value << p.wind_drag;
  out << YAML::Key << "contact_friction" << YAML::Value << p.contact_friction;
  out << YAML::Key << "gravity" << YAML::Value << p.gravity;
  out << YAML::Key << "anchor_offset" << YAML::Value << p.anchor_offset;
  out << YAML::Key << "fracture_enabled" << YAML::Value << p.fracture_enabled;
  out << YAML::Key << "fracture_angle_degrees" << YAML::Value << p.fracture_angle_degrees;
  out << YAML::Key << "fracture_duration" << YAML::Value << p.fracture_duration;
}
void DeserializeTreePhysicsProperties(const YAML::Node& in, TreePhysicsProperties& p) {
  if (!in)
    return;
  if (in["stretch_stiffness"])
    p.stretch_stiffness = in["stretch_stiffness"].as<float>();
  if (in["bend_stiffness"])
    p.bend_stiffness = in["bend_stiffness"].as<float>();
  if (in["density"])
    p.density = in["density"].as<float>();
  if (in["damping"])
    p.damping = in["damping"].as<float>();
  if (in["wind_drag"])
    p.wind_drag = in["wind_drag"].as<float>();
  if (in["contact_friction"])
    p.contact_friction = in["contact_friction"].as<float>();
  if (in["gravity"])
    p.gravity = in["gravity"].as<glm::vec3>();
  if (in["anchor_offset"])
    p.anchor_offset = in["anchor_offset"].as<glm::vec3>();
  if (in["fracture_enabled"])
    p.fracture_enabled = in["fracture_enabled"].as<bool>();
  if (in["fracture_angle_degrees"])
    p.fracture_angle_degrees = in["fracture_angle_degrees"].as<float>();
  if (in["fracture_duration"])
    p.fracture_duration = in["fracture_duration"].as<float>();
}
}  // namespace eco_sys_lab_package
