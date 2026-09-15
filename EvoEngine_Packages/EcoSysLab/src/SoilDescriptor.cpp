#include "EcoSysLabLayer.hpp"
#include "EcoSysLabSerializationAdapters.hpp"
#include "HeightField.hpp"
#include "Material.hpp"
using namespace eco_sys_lab_package;

void eco_sys_lab_package::SerializeSoilLayerDescriptor(YAML::Emitter& out, const SoilLayerDescriptor& target) {
  target.capacity_graph.Save("capacity_graph", out);
  target.permeability_graph.Save("permeability_graph", out);
  target.density_graph.Save("density_graph", out);
  target.initial_nutrients_graph.Save("initial_nutrients_graph", out);
  target.initial_water_graph.Save("initial_water_graph", out);

  target.thickness_graph.Save("thickness_graph", out);

  target.albedo_texture.Save("albedo_texture", out);
  target.roughness_texture.Save("roughness_texture", out);
  target.metallic_texture.Save("metallic_texture", out);
  target.normal_texture.Save("normal_texture", out);
  target.height_texture.Save("height_texture", out);
}

void eco_sys_lab_package::DeserializeSoilLayerDescriptor(const YAML::Node& in, SoilLayerDescriptor& target) {
  target.capacity_graph.Load("capacity_graph", in);
  target.permeability_graph.Load("permeability_graph", in);
  target.density_graph.Load("density_graph", in);
  target.initial_nutrients_graph.Load("initial_nutrients_graph", in);
  target.initial_water_graph.Load("initial_water_graph", in);
  target.thickness_graph.Load("thickness_graph", in);

  target.albedo_texture.Load("albedo_texture", in);
  target.roughness_texture.Load("roughness_texture", in);
  target.metallic_texture.Load("metallic_texture", in);
  target.normal_texture.Load("normal_texture", in);
  target.height_texture.Load("height_texture", in);
}

void SoilLayerDescriptor::CollectAssetRef(std::vector<AssetRef>& list) {
  list.push_back(albedo_texture);
  list.push_back(roughness_texture);
  list.push_back(metallic_texture);
  list.push_back(normal_texture);
  list.push_back(height_texture);
}

void SoilDescriptor::RandomOffset(const float min, const float max) {
  if (const auto hf = height_field.Get<HeightField>()) {
    hf->RandomOffset(min, max);
  }
}

void SerializeSoilParameters(const std::string& name, const SoilParameters& soil_parameters, YAML::Emitter& out) {
  out << YAML::Key << name << YAML::BeginMap;
  out << YAML::Key << "m_voxelResolution" << YAML::Value << soil_parameters.m_voxelResolution;
  out << YAML::Key << "m_deltaX" << YAML::Value << soil_parameters.m_deltaX;
  out << YAML::Key << "m_deltaTime" << YAML::Value << soil_parameters.m_deltaTime;
  out << YAML::Key << "m_boundingBoxMin" << YAML::Value << soil_parameters.m_boundingBoxMin;

  out << YAML::Key << "m_boundary_x" << YAML::Value << static_cast<int>(soil_parameters.m_boundary_x);
  out << YAML::Key << "m_boundary_y" << YAML::Value << static_cast<int>(soil_parameters.m_boundary_y);
  out << YAML::Key << "m_boundary_z" << YAML::Value << static_cast<int>(soil_parameters.m_boundary_z);

  out << YAML::Key << "m_diffusionForce" << YAML::Value << soil_parameters.m_diffusionForce;
  out << YAML::Key << "m_gravityForce" << YAML::Value << soil_parameters.m_gravityForce;
  out << YAML::EndMap;
}

void DeserializeSoilParameters(const std::string& name, SoilParameters& soil_parameters, const YAML::Node& in) {
  if (in[name]) {
    auto& param = in[name];
    if (param["m_voxelResolution"])
      soil_parameters.m_voxelResolution = param["m_voxelResolution"].as<glm::uvec3>();
    else {
      EVOENGINE_WARNING("DeserializeSoilParameters: m_voxelResolution not found!");
      // EVOENGINE_ERROR("DeserializeSoilParameters: m_voxelResolution not found!");
      // EVOENGINE_LOG("DeserializeSoilParameters: m_voxelResolution not found!");
    }
    if (param["m_deltaX"])
      soil_parameters.m_deltaX = param["m_deltaX"].as<float>();
    if (param["m_deltaTime"])
      soil_parameters.m_deltaTime = param["m_deltaTime"].as<float>();
    if (param["m_boundingBoxMin"])
      soil_parameters.m_boundingBoxMin = param["m_boundingBoxMin"].as<glm::vec3>();

    if (param["m_boundary_x"])
      soil_parameters.m_boundary_x = static_cast<VoxelSoilModel::Boundary>(param["m_boundary_x"].as<int>());
    if (param["m_boundary_y"])
      soil_parameters.m_boundary_y = static_cast<VoxelSoilModel::Boundary>(param["m_boundary_y"].as<int>());
    if (param["m_boundary_z"])
      soil_parameters.m_boundary_z = static_cast<VoxelSoilModel::Boundary>(param["m_boundary_z"].as<int>());

    if (param["m_diffusionForce"])
      soil_parameters.m_diffusionForce = param["m_diffusionForce"].as<float>();
    if (param["m_gravityForce"])
      soil_parameters.m_gravityForce = param["m_gravityForce"].as<glm::vec3>();
  }
}

void eco_sys_lab_package::SerializeSoilDescriptor(YAML::Emitter& out, const SoilDescriptor& target) {
  target.height_field.Save("height_field", out);
  SerializeSoilParameters("soil_parameters", target.soil_parameters, out);

  out << YAML::Key << "soil_layer_descriptors" << YAML::Value << YAML::BeginSeq;
  for (int i = 0; i < target.soil_layer_descriptors.size(); i++) {
    out << YAML::BeginMap;
    target.soil_layer_descriptors[i].Serialize(out);
    out << YAML::EndMap;
  }
  out << YAML::EndSeq;
}

void eco_sys_lab_package::DeserializeSoilDescriptor(const YAML::Node& in, SoilDescriptor& target) {
  target.height_field.Load("height_field", in);
  DeserializeSoilParameters("soil_parameters", target.soil_parameters, in);
  target.soil_layer_descriptors.clear();
  if (in["soil_layer_descriptors"]) {
    for (const auto& i : in["soil_layer_descriptors"]) {
      target.soil_layer_descriptors.emplace_back();
      target.soil_layer_descriptors.back().Deserialize(i);
    }
  }
}

void SoilDescriptor::CollectAssetRef(std::vector<AssetRef>& list) {
  list.push_back(height_field);

  for (int i = 0; i < soil_layer_descriptors.size(); i++) {
    if (auto soil_layer_descriptor = soil_layer_descriptors[i].Get<SoilLayerDescriptor>()) {
      list.push_back(soil_layer_descriptors[i]);
    } else {
      soil_layer_descriptors.erase(soil_layer_descriptors.begin() + i);
      i--;
    }
  }
}
