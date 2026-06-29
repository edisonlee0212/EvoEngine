#pragma once

#include <filesystem>
#include <memory>

#include "ApplicationInitializationSettings.hpp"

namespace evo_engine {
class Scene;

enum class DemoSetup { Empty, Rendering, CornellBox, ThinWall, ProceduralGalaxy, Universe, GaussianSplat, Bicycle };

[[nodiscard]] std::filesystem::path FindDemoResourcesRoot(const std::filesystem::path& preferred_root = {});

void ClearGeneratedDemoProjectFiles(const std::filesystem::path& resource_folder_path);
void ClearGeneratedProceduralGalaxyProjectFiles(const std::filesystem::path& resource_folder_path);
void ConfigureGaussianSplatDemoScene(const std::shared_ptr<Scene>& scene);
void ConfigureBicycleDemoScene(const std::shared_ptr<Scene>& scene);

void SetupDemoScene(DemoSetup demo_setup, ApplicationInitializationSettings& application_info,
                    const std::filesystem::path& resource_folder_path = {}, bool clear_generated_project_files = true);

}  // namespace evo_engine
