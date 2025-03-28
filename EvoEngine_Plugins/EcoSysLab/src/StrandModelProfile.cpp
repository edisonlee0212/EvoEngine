#include "StrandModelProfile.hpp"

using namespace eco_sys_lab_plugin;
void ParticlePhysicsSettings::Save(const std::string& name, YAML::Emitter& out) const {
  out << YAML::Key << name << YAML::BeginMap;
  out << YAML::Key << "particle_softness" << YAML::Value << particle_softness;
  out << YAML::Key << "damping" << YAML::Value << damping;
  out << YAML::Key << "max_speed" << YAML::Value << max_speed;
  out << YAML::EndMap;
}
void ParticlePhysicsSettings::Load(const std::string& name, const YAML::Node& in) {
  if (in[name]) {
    const auto& in_settings = in[name];
    if (in_settings["particle_softness"])
      particle_softness = in_settings["particle_softness"].as<float>();
    if (in_settings["damping"])
      damping = in_settings["damping"].as<float>();
    if (in_settings["max_speed"])
      max_speed = in_settings["max_speed"].as<float>();
  }
}