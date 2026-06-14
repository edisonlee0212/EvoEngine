#include "TreeStatistics.hpp"

using namespace eco_sys_lab_package;

void TreeStatistics::DrawGui(const std::shared_ptr<EditorLayer>& editor_layer) {
}

void TreeStatistics::Save(const std::string& name, YAML::Emitter& out) const {
  out << YAML::Key << name << YAML::Value << YAML::BeginMap;
  Serialize(out);
  out << YAML::EndMap;
}

void TreeStatistics::Load(const std::string& name, const YAML::Node& in) {
  if (in[name]) {
    const auto& cd = in[name];
    Deserialize(cd);
  }
}

void TreeStatistics::Export(const std::filesystem::path& path) const {
  try {
    YAML::Emitter out;
    out << YAML::BeginMap;
    Serialize(out);
    out << YAML::EndMap;
    std::ofstream file_output(path.string());
    file_output << out.c_str();
    file_output.close();
  } catch (const std::exception& e) {
    EVOENGINE_ERROR("Failed to save: " + std::string(e.what()))
  }
}

void TreeStatistics::Serialize(YAML::Emitter& out) const {
  out << YAML::Key << "dbh" << YAML::Value << dbh;
  out << YAML::Key << "volume" << YAML::Value << volume;
  out << YAML::Key << "height" << YAML::Value << height;
}

void TreeStatistics::Deserialize(const YAML::Node& in) {
  if (in["dbh"]) {
    dbh = in["dbh"].as<float>();
  }
  if (in["volume"]) {
    volume = in["volume"].as<float>();
  }
  if (in["height"]) {
    height = in["height"].as<float>();
  }
}