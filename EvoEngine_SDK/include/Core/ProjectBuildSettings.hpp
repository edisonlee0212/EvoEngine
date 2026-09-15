#pragma once
#include <cstdint>
#include <filesystem>
#include <string>
#include "EvoEngineAPI.hpp"
#include "WindowDisplayMode.hpp"
namespace evo_engine {
enum class RuntimeTarget { WindowsX64 };
struct EVOENGINE_API ProjectBuildSettings {
  uint32_t schema_version = 1;
  std::string application_name = "EvoEngine Application";
  std::filesystem::path output_directory;
  uint64_t startup_scene_handle = 0;
  RuntimeTarget target = RuntimeTarget::WindowsX64;
  WindowDisplayMode window_mode = WindowDisplayMode::Windowed;
  int window_width = 1280;
  int window_height = 720;
  bool allow_window_resize = true;
  bool allow_resolution_change = true;
  bool show_console = false;
  friend bool operator==(const ProjectBuildSettings& a, const ProjectBuildSettings& b) {
    return a.application_name == b.application_name && a.output_directory == b.output_directory &&
           a.startup_scene_handle == b.startup_scene_handle && a.target == b.target && a.window_mode == b.window_mode &&
           a.window_width == b.window_width && a.window_height == b.window_height &&
           a.allow_window_resize == b.allow_window_resize && a.allow_resolution_change == b.allow_resolution_change &&
           a.show_console == b.show_console;
  }
};
}  // namespace evo_engine
