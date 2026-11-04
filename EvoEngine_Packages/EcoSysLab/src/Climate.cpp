#include "Climate.hpp"
#include "EcoSysLabSerializationAdapters.hpp"

#include "AssetManager.hpp"
#include "EcoSysLabLayer.hpp"
#include "EcoSysLabSystem.hpp"
#include "Tree.hpp"

using namespace eco_sys_lab_package;

void eco_sys_lab_package::SerializeClimateDescriptor(YAML::Emitter& out, const ClimateDescriptor& target) {
}

void eco_sys_lab_package::DeserializeClimateDescriptor(const YAML::Node& in, ClimateDescriptor& target) {
}

void eco_sys_lab_package::SerializeClimate(YAML::Emitter& out, const Climate& target) {
  target.climate_descriptor_ref.Save("climate_descriptor_ref", out);
  out << YAML::Key << "air_temperature_c" << YAML::Value << target.air_temperature_c;
  out << YAML::Key << "air_relative_humidity" << YAML::Value << target.air_relative_humidity;
  out << YAML::Key << "air_pressure_pa" << YAML::Value << target.air_pressure_pa;
  out << YAML::Key << "air_coupling_rate" << YAML::Value << target.air_coupling_rate;
  out << YAML::Key << "air_friction" << YAML::Value << target.air_friction;
  out << YAML::Key << "air_cell_size" << YAML::Value << target.air_cell_size;
}

void Climate::CollectAssetRef(std::vector<AssetRef>& list) {
  list.push_back(climate_descriptor_ref);
}

void Climate::InitializeClimateModel() {
  if (const auto climate_descriptor = climate_descriptor_ref.Get<ClimateDescriptor>()) {
    const auto params = climate_descriptor->climate_parameters;
    climate_model.Initialize(params);
  }
}

void Climate::PrepareForGrowth() {
  const auto scene = GetScene();
  const auto system = scene->GetOrCreateSystem<EcoSysLabSystem>(0.0f);
  system->environment.EnsureConfiguration(scene);
  system->environment.InitializeModels();
  system->environment.PrepareForGrowth(scene, system->growth.settings);
}

void eco_sys_lab_package::DeserializeClimate(const YAML::Node& in, Climate& target) {
  target.climate_descriptor_ref.Load("climate_descriptor_ref", in);
  if (in["air_temperature_c"])
    target.air_temperature_c = in["air_temperature_c"].as<float>();
  if (in["air_relative_humidity"])
    target.air_relative_humidity = in["air_relative_humidity"].as<float>();
  if (in["air_pressure_pa"])
    target.air_pressure_pa = in["air_pressure_pa"].as<float>();
  if (in["air_coupling_rate"])
    target.air_coupling_rate = in["air_coupling_rate"].as<float>();
  if (in["air_friction"])
    target.air_friction = in["air_friction"].as<float>();
  if (in["air_cell_size"])
    target.air_cell_size = glm::clamp(in["air_cell_size"].as<float>(), Climate::minimum_air_cell_size, 0.2f);
}
