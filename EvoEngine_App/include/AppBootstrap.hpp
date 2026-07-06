#pragma once

#include "Application.hpp"
#include "EditorLayer.hpp"
#include "ImGuiLayer.hpp"
#include "RenderLayer.hpp"
#include "WindowLayer.hpp"

#include <algorithm>
#include <cctype>
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

[[nodiscard]] inline GraphicsInitializationSettings::ShadowMapResolutionQuality ParseShadowMapResolutionQualityName(
    std::string quality_name) {
  std::transform(quality_name.begin(), quality_name.end(), quality_name.begin(), [](const char character) {
    return static_cast<char>(std::tolower(static_cast<unsigned char>(character)));
  });
  if (quality_name == "low" || quality_name == "1k" || quality_name == "1024") {
    return GraphicsInitializationSettings::ShadowMapResolutionQuality::Low;
  }
  if (quality_name == "medium" || quality_name == "med" || quality_name == "2k" || quality_name == "2048") {
    return GraphicsInitializationSettings::ShadowMapResolutionQuality::Medium;
  }
  if (quality_name == "high" || quality_name == "4k" || quality_name == "4096") {
    return GraphicsInitializationSettings::ShadowMapResolutionQuality::High;
  }
  if (quality_name == "very-high" || quality_name == "veryhigh" || quality_name == "ultra" || quality_name == "8k" ||
      quality_name == "8192") {
    return GraphicsInitializationSettings::ShadowMapResolutionQuality::VeryHigh;
  }
  throw std::invalid_argument("Unknown shadow map resolution quality: " + quality_name);
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
