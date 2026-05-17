#include "LSystemDescriptorDefaults.hpp"

#include <Console.hpp>

#include <fstream>
#include <sstream>

using namespace l_system_plugin;

bool descriptor_defaults::LoadDefaultsYamlMap(
    const std::filesystem::path& file_path,
    YAML::Node& out_defaults,
    const std::string& descriptor_name_for_logging) {
  if (file_path.empty() || !std::filesystem::exists(file_path)) {
    return false;
  }

  try {
    const std::ifstream stream(file_path.string());
    std::stringstream string_stream;
    string_stream << stream.rdbuf();
    const YAML::Node defaults = YAML::Load(string_stream.str());
    if (!defaults || !defaults.IsMap()) {
      return false;
    }
    out_defaults = defaults;
    return true;
  } catch (const std::exception& e) {
    EVOENGINE_WARNING("Failed to load " + descriptor_name_for_logging + " defaults from " +
                      file_path.string() + ": " + std::string(e.what()));
    return false;
  }
}
