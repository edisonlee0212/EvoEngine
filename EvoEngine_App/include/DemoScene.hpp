#pragma once

#include <filesystem>

#include "ApplicationInitializationSettings.hpp"

namespace evo_engine {

enum class DemoSetup { Empty, Rendering, Universe };

[[nodiscard]] std::filesystem::path FindDemoResourcesRoot(const std::filesystem::path& preferred_root = {});

void ClearGeneratedDemoProjectFiles(const std::filesystem::path& resource_folder_path);

void SetupDemoScene(DemoSetup demo_setup, ApplicationInitializationSettings& application_info,
                    const std::filesystem::path& resource_folder_path = {}, bool clear_generated_project_files = true);

}  // namespace evo_engine
