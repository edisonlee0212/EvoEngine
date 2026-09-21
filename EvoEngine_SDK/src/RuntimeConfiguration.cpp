#include "RuntimeConfiguration.hpp"
#include "NativeBuildIdentity.hpp"
#include "PackageManager.hpp"
#include "PathUtils.hpp"
#include "RuntimePaths.hpp"

#include <yaml-cpp/yaml.h>
#include <array>
#include <fstream>
#include <set>
#include <stdexcept>

using namespace evo_engine;

namespace {
void CheckIdentity(const YAML::Node& node) {
  const std::array<const char*, 6> keys = {"sdk_source_id", "compiler_id", "compiler_version",
                                           "configuration", "platform",    "architecture"};
  std::array<std::string, 6> values;
  for (size_t i = 0; i < keys.size(); ++i) {
    values[i] = node[keys[i]].as<std::string>();
  }
  NativeBuildIdentity candidate{values[0].c_str(),
                                values[1].c_str(),
                                values[2].c_str(),
                                values[3].c_str(),
                                values[4].c_str(),
                                values[5].c_str(),
                                node["with_editor"].as<bool>()};
  if (candidate.with_editor)
    throw std::runtime_error("Runtime configuration must identify a runtime payload.");
  const auto& expected = GetNativeBuildIdentity();
  std::string reason;
  if (!IsNativeBuildCompatible(expected, candidate, &reason)) {
    throw std::runtime_error("Runtime configuration is incompatible: " + reason);
  }
}

WindowDisplayMode ReadWindowMode(const YAML::Node& node) {
  const auto value = node.as<std::string>();
  if (value == "windowed")
    return WindowDisplayMode::Windowed;
  if (value == "borderless_windowed")
    return WindowDisplayMode::BorderlessWindowed;
  if (value == "borderless_fullscreen")
    return WindowDisplayMode::BorderlessFullscreen;
  if (value == "exclusive_fullscreen")
    return WindowDisplayMode::ExclusiveFullscreen;
  throw std::runtime_error("Unknown runtime window mode: " + value);
}

template <typename T>
void ReadSetting(const YAML::Node& node, const char* key, T& value) {
  if (const auto field = node[key]) {
    value = field.as<T>();
  }
}
}  // namespace

RuntimeConfiguration RuntimeConfiguration::Load(const std::filesystem::path& path) {
  std::ifstream stream(path);
  if (!stream) {
    throw std::runtime_error("Required runtime.yaml is missing or unreadable.");
  }
  const auto node = YAML::Load(stream);
  if (!node.IsMap() || node["schema_version"].as<int>() != 1) {
    throw std::runtime_error("Unsupported runtime configuration format.");
  }
  CheckIdentity(node["identity"]);
  RuntimeConfiguration config;
  ReadSetting(node, "show_console", config.show_console);
  ReadSetting(node, "runtime_gui_layout_revision", config.runtime_gui_layout_revision);
  config.application_name = node["application_name"].as<std::string>();
  config.project = std::filesystem::u8path(node["project"].as<std::string>());
  if (config.application_name.empty() || config.project.extension() != ".eveproj") {
    throw std::runtime_error("Runtime configuration requires an application name and an existing .eveproj project.");
  }
  const auto project_path = runtime_paths::Resolve(config.project);
  if (!std::filesystem::is_regular_file(project_path)) {
    throw std::runtime_error("Runtime project is missing: " + config.project.string());
  }
  if (const auto window = node["window"]) {
    ReadSetting(window, "width", config.window_size.x);
    ReadSetting(window, "height", config.window_size.y);
    if (window["mode"])
      config.window_mode = ReadWindowMode(window["mode"]);
    ReadSetting(window, "allow_resize", config.allow_window_resize);
    ReadSetting(window, "allow_resolution_change", config.allow_resolution_change);
  }
  if (config.window_size.x <= 0 || config.window_size.y <= 0) {
    throw std::runtime_error("Runtime window dimensions must be positive.");
  }
  if (const auto graphics = node["graphics"]) {
    auto& settings = config.graphics;
    ReadSetting(graphics, "use_mesh_shader", settings.use_mesh_shader);
    ReadSetting(graphics, "use_ray_tracing", settings.use_ray_tracing);
    ReadSetting(graphics, "directional_light_shadow_map_resolution", settings.directional_light_shadow_map_resolution);
    ReadSetting(graphics, "point_light_shadow_map_resolution", settings.point_light_shadow_map_resolution);
    ReadSetting(graphics, "spot_light_shadow_map_resolution", settings.spot_light_shadow_map_resolution);
    ReadSetting(graphics, "max_texture_2d_resource_size", settings.max_texture_2d_resource_size);
    ReadSetting(graphics, "max_cubemap_resource_size", settings.max_cubemap_resource_size);
    ReadSetting(graphics, "max_directional_light_size", settings.max_directional_light_size);
    ReadSetting(graphics, "max_point_light_size", settings.max_point_light_size);
    ReadSetting(graphics, "max_spot_light_size", settings.max_spot_light_size);
  }
  const auto packages = node["packages"];
  if (!packages.IsSequence()) {
    throw std::runtime_error("Runtime packages must be an explicit list, including an empty list when none are used.");
  }
  std::set<std::string> names;
  for (const auto& entry : packages) {
    RuntimePackageRequirement package{entry["name"].as<std::string>(), entry["source_id"].as<std::string>()};
    if (!path_utils::IsValidFileName(package.name) || package.source_id.size() != 64 ||
        !names.insert(package.name).second) {
      throw std::runtime_error("Invalid or duplicate runtime package requirement: " + package.name);
    }
    const auto manifest = runtime_paths::Resolve(std::filesystem::path("Packages") / (package.name + ".evepackage"));
    if (!std::filesystem::is_regular_file(manifest)) {
      throw std::runtime_error("Required runtime package manifest is missing: " + package.name);
    }
    config.packages.emplace_back(std::move(package));
  }
  return config;
}

ApplicationInitializationSettings RuntimeConfiguration::ApplicationSettings() const {
  ApplicationInitializationSettings settings;
  settings.application_mode = ApplicationMode::Player;
  settings.strict_runtime = true;
  settings.project_path = runtime_paths::Resolve(project);
  settings.application_name = application_name;
  settings.runtime_gui_layout_revision = runtime_gui_layout_revision;
  settings.default_window_size = window_size;
  settings.window_mode = window_mode;
  settings.window_resizable = allow_window_resize;
  settings.allow_resolution_change = allow_resolution_change;
  settings.enable_docking = false;
  settings.enable_viewport = false;
  settings.redirect_standard_streams_to_console = false;
  settings.hide_console_window = !show_console;
  settings.load_project_assets = false;
  settings.enable_runtime_packages = !packages.empty();
  for (const auto& package : packages) {
    settings.startup_runtime_packages.emplace_back(package.name);
  }
  settings.graphics_settings = graphics;
  return settings;
}

void RuntimeConfiguration::ValidateLoadedPackages() const {
  const auto loaded = PackageManager::GetLoadedPackages();
  if (loaded.size() != packages.size()) {
    throw std::runtime_error("Runtime loaded a different package set from its build configuration.");
  }
  for (const auto& required : packages) {
    const auto found = std::find_if(loaded.begin(), loaded.end(), [&](const auto& package) {
      return package.name == required.name && package.package_source_id == required.source_id;
    });
    if (found == loaded.end()) {
      throw std::runtime_error("Required runtime package is missing or has different native sources: " + required.name);
    }
  }
}
