#include <yaml-cpp/yaml.h>
#include <array>
#include "LSystemDescriptorDefaults.hpp"
#include "LSystemSerializationAdapters.hpp"

using namespace l_system_package;
using namespace evo_engine;

namespace {

constexpr char kLSystemDescriptorName[] = "LSystemDescriptor";

// LSystem-only candidates. DigitalAgricultureProject paths intentionally
// removed so this package has no implicit data-path dependency on the
// DigitalAgriculture/EcoSysLab tree.
const std::array<std::filesystem::path, 4> kLSystemResourceCandidates = {
    std::filesystem::path("./LSystemResources/Defaults/LSystemDescriptor_Default.lsys"),
    std::filesystem::path("./EvoEngine_Packages/LSystem/Internals/LSystemResources/Defaults/") /
        "LSystemDescriptor_Default.lsys",
    std::filesystem::path("./EvoEngine_Plugins/LSystem/Internals/LSystemResources/Defaults/") /
        "LSystemDescriptor_Default.lsys",
    std::filesystem::path("./04_EvoEngine/EvoEngine_Packages/LSystem/Internals/") /
        "LSystemResources/Defaults/LSystemDescriptor_Default.lsys"};

const std::array<std::filesystem::path, 2> kLSystemProjectAssetCandidates = {
    std::filesystem::path("LSystem") / "New LSystemDescriptor.lsys", "New LSystemDescriptor.lsys"};

const std::array<std::filesystem::path, 1> kLSystemWritableTemplateCandidates = {
    std::filesystem::path("./EvoEngine_Packages/LSystem/Internals/LSystemResources/Defaults/") /
    "LSystemDescriptor_Default.lsys"};

const std::filesystem::path kLSystemFallbackDefaultsPath =
    std::filesystem::path("./LSystemResources/Defaults/LSystemDescriptor_Default.lsys");

std::filesystem::path ResolveDefaultLSystemDescriptorPath() {
  return descriptor_defaults::ResolveExistingDefaultsPath(kLSystemResourceCandidates, kLSystemProjectAssetCandidates);
}

std::filesystem::path ResolveWritableLSystemDescriptorDefaultsPath() {
  return descriptor_defaults::ResolveWritableDefaultsPath(kLSystemResourceCandidates, kLSystemProjectAssetCandidates,
                                                          kLSystemWritableTemplateCandidates,
                                                          kLSystemFallbackDefaultsPath);
}

bool LoadLSystemDescriptorDefaultsFromFile(LSystemDescriptor& descriptor, const std::filesystem::path& file_path) {
  YAML::Node defaults;
  if (!descriptor_defaults::LoadDefaultsYamlMap(file_path, defaults, kLSystemDescriptorName)) {
    return false;
  }
  DeserializeLSystemDescriptor(defaults, descriptor);
  return true;
}

}  // namespace

LSystemDescriptor::LSystemDescriptor() {
  const auto defaults_path = ResolveDefaultLSystemDescriptorPath();
  if (!LoadLSystemDescriptorDefaultsFromFile(*this, defaults_path)) {
    static bool warned_once = false;
    if (!warned_once) {
      warned_once = true;
      EVOENGINE_WARNING("LSystemDescriptor defaults file not found or invalid. Using inline member defaults.");
    }
  }
}

void l_system_package::SerializeLSystemDescriptor(YAML::Emitter& out, const LSystemDescriptor& target) {
  out << YAML::Key << "derivation_steps" << YAML::Value << target.derivation_steps;
  out << YAML::Key << "seed" << YAML::Value << target.seed;
  out << YAML::Key << "root_position" << YAML::Value << target.root_position;
  out << YAML::Key << "root_rotation" << YAML::Value << target.root_rotation;
  out << YAML::Key << "default_length" << YAML::Value << target.default_length;
  out << YAML::Key << "default_thickness" << YAML::Value << target.default_thickness;
  out << YAML::Key << "auto_derive_on_change" << YAML::Value << target.auto_derive_on_change;
}

void l_system_package::DeserializeLSystemDescriptor(const YAML::Node& in, LSystemDescriptor& target) {
  if (in["derivation_steps"])
    target.derivation_steps = in["derivation_steps"].as<int>();
  if (in["seed"])
    target.seed = in["seed"].as<unsigned int>();
  if (in["root_position"])
    target.root_position = in["root_position"].as<glm::vec3>();
  if (in["root_rotation"])
    target.root_rotation = in["root_rotation"].as<glm::quat>();
  if (in["default_length"])
    target.default_length = in["default_length"].as<float>();
  if (in["default_thickness"])
    target.default_thickness = in["default_thickness"].as<float>();
  if (in["auto_derive_on_change"])
    target.auto_derive_on_change = in["auto_derive_on_change"].as<bool>();
}
