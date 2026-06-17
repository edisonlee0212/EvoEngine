#pragma once

#include "Application.hpp"
#include "EditorLayer.hpp"
#include "ImGuiLayer.hpp"
#include "RenderLayer.hpp"
#include "WindowLayer.hpp"

#include <stdexcept>
#include <string>

namespace evo_engine {

[[nodiscard]] inline ApplicationMode ParseApplicationModeName(const std::string& mode_name) {
  if (mode_name == "Editor" || mode_name == "editor") {
    return ApplicationMode::Editor;
  }
  if (mode_name == "Player" || mode_name == "player") {
    return ApplicationMode::Player;
  }
  if (mode_name == "Headless" || mode_name == "headless") {
    return ApplicationMode::Headless;
  }
  throw std::invalid_argument("Unknown application mode: " + mode_name);
}

[[nodiscard]] inline const char* GetApplicationModeName(const ApplicationMode mode) {
  switch (mode) {
    case ApplicationMode::Editor:
      return "Editor";
    case ApplicationMode::Player:
      return "Player";
    case ApplicationMode::Headless:
      return "Headless";
  }
  return "Editor";
}

[[nodiscard]] inline const char* GetApplicationModeArgument(const ApplicationMode mode) {
  switch (mode) {
    case ApplicationMode::Editor:
      return "--editor";
    case ApplicationMode::Player:
      return "--player";
    case ApplicationMode::Headless:
      return "--headless";
  }
  return "--editor";
}

inline bool ConsumeApplicationModeArgument(const int argc, char** argv, int& arg_index, ApplicationMode& mode) {
  const std::string argument = argv[arg_index] ? argv[arg_index] : "";
  if (argument == "--editor") {
    mode = ApplicationMode::Editor;
    return true;
  }
  if (argument == "--player") {
    mode = ApplicationMode::Player;
    return true;
  }
  if (argument == "--headless") {
    mode = ApplicationMode::Headless;
    return true;
  }
  if (argument == "--application-mode" || argument == "--mode") {
    if (arg_index + 1 >= argc) {
      throw std::invalid_argument(argument + " requires Editor, Player, or Headless.");
    }
    mode = ParseApplicationModeName(argv[++arg_index]);
    return true;
  }
  return false;
}

[[nodiscard]] inline ApplicationMode ParseApplicationModeArguments(const int argc, char** argv) {
  auto mode = ApplicationMode::Editor;
  for (int arg_index = 1; arg_index < argc; ++arg_index) {
    if (!ConsumeApplicationModeArgument(argc, argv, arg_index, mode)) {
      throw std::invalid_argument("Unknown application argument: " +
                                  std::string(argv[arg_index] ? argv[arg_index] : ""));
    }
  }
  return mode;
}

inline void ApplyApplicationModeDefaults(ApplicationInitializationSettings& settings) {
  if (settings.application_mode != ApplicationMode::Editor) {
    settings.use_custom_title_bar = false;
  }
  if (settings.application_mode == ApplicationMode::Headless) {
    settings.enable_docking = false;
    settings.enable_viewport = false;
    settings.load_default_resources = false;
    settings.load_project_assets = false;
  }
}

inline void PushWindowAndUiLayers(const ApplicationMode mode) {
  if (mode == ApplicationMode::Headless) {
    return;
  }
  ApplicationContext::Get().PushLayer<WindowLayer>("Window Layer");
  if (mode == ApplicationMode::Editor) {
    ApplicationContext::Get().PushLayer<ImGuiLayer>("ImGui Layer");
    ApplicationContext::Get().PushLayer<EditorLayer>("Editor Layer");
  }
}

inline void PushStandardApplicationLayers(const ApplicationMode mode) {
  if (mode == ApplicationMode::Headless) {
    return;
  }
  ApplicationContext::Get().PushLayer<RenderLayer>("Render Layer");
  PushWindowAndUiLayers(mode);
}

}  // namespace evo_engine
