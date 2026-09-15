#include "ProjectBuildSettingsSerialization.hpp"
#include <stdexcept>
namespace evo_engine {
namespace {
const char* ModeName(WindowDisplayMode mode) {
  switch (mode) {
    case WindowDisplayMode::Windowed:
      return "windowed";
    case WindowDisplayMode::BorderlessWindowed:
      return "borderless_windowed";
    case WindowDisplayMode::BorderlessFullscreen:
      return "borderless_fullscreen";
    case WindowDisplayMode::ExclusiveFullscreen:
      return "exclusive_fullscreen";
  }
  return "windowed";
}
WindowDisplayMode ParseMode(const std::string& value) {
  if (value == "windowed")
    return WindowDisplayMode::Windowed;
  if (value == "borderless_windowed")
    return WindowDisplayMode::BorderlessWindowed;
  if (value == "borderless_fullscreen")
    return WindowDisplayMode::BorderlessFullscreen;
  if (value == "exclusive_fullscreen")
    return WindowDisplayMode::ExclusiveFullscreen;
  throw std::runtime_error("Unknown build window mode: " + value);
}
}  // namespace
void SerializeProjectBuildSettings(const ProjectBuildSettings& s, YAML::Emitter& out) {
  out << YAML::BeginMap << YAML::Key << "schema_version" << YAML::Value << 1 << YAML::Key << "application_name"
      << YAML::Value << s.application_name << YAML::Key << "output_directory" << YAML::Value
      << s.output_directory.generic_u8string() << YAML::Key << "startup_scene_handle" << YAML::Value
      << s.startup_scene_handle << YAML::Key << "target" << YAML::Value << "windows_x64" << YAML::Key << "window"
      << YAML::Value << YAML::BeginMap << YAML::Key << "mode" << YAML::Value << ModeName(s.window_mode) << YAML::Key
      << "width" << YAML::Value << s.window_width << YAML::Key << "height" << YAML::Value << s.window_height
      << YAML::Key << "allow_resize" << YAML::Value << s.allow_window_resize << YAML::Key << "allow_resolution_change"
      << YAML::Value << s.allow_resolution_change << YAML::EndMap << YAML::Key << "show_console" << YAML::Value
      << s.show_console << YAML::EndMap;
}
ProjectBuildSettings DeserializeProjectBuildSettings(const YAML::Node& node) {
  ProjectBuildSettings s;
  if (!node)
    return s;
  if (!node.IsMap() || node["schema_version"].as<int>() != 1 || node["target"].as<std::string>() != "windows_x64")
    throw std::runtime_error("Unsupported project build_settings format.");
  s.application_name = node["application_name"].as<std::string>();
  s.output_directory = std::filesystem::u8path(node["output_directory"].as<std::string>());
  s.startup_scene_handle = node["startup_scene_handle"].as<uint64_t>();
  const auto window = node["window"];
  s.window_mode = ParseMode(window["mode"].as<std::string>());
  s.window_width = window["width"].as<int>();
  s.window_height = window["height"].as<int>();
  s.allow_window_resize = window["allow_resize"].as<bool>();
  s.allow_resolution_change = window["allow_resolution_change"].as<bool>();
  if (node["show_console"])
    s.show_console = node["show_console"].as<bool>();
  return s;
}
}  // namespace evo_engine
