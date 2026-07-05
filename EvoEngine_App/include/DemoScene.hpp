#pragma once

#include <filesystem>
#include <memory>
#include <string>

#include "ApplicationInitializationSettings.hpp"

namespace evo_engine {
class Camera;
class Scene;

enum class DemoSetup {
  Empty,
  Rendering,
  CornellBox,
  ThinWall,
  ProceduralGalaxy,
  Universe,
  RenderingRegression,
  GaussianSplat,
  Bicycle,
  Bistro
};

[[nodiscard]] std::filesystem::path FindDemoResourcesRoot(const std::filesystem::path& preferred_root = {});

void ClearGeneratedDemoProjectFiles(const std::filesystem::path& resource_folder_path);
void ClearGeneratedProceduralGalaxyProjectFiles(const std::filesystem::path& resource_folder_path);
void ConfigureRenderingRegressionDemoScene(const std::shared_ptr<Scene>& scene);
void ConfigureGaussianSplatDemoScene(const std::shared_ptr<Scene>& scene);
void ConfigureBicycleDemoScene(const std::shared_ptr<Scene>& scene);
void ConfigureBistroDemoScene(const std::shared_ptr<Scene>& scene);
void ConfigureBistroRayTracingPostProcessing(const std::shared_ptr<Camera>& camera);
void ConfigureBistroParityCapture(const std::shared_ptr<Scene>& scene, const std::shared_ptr<Camera>& camera);
void LogBistroParityCaptureState(const std::shared_ptr<Scene>& scene, const std::shared_ptr<Camera>& camera, int width,
                                 int height, const std::string& render_mode_name,
                                 const std::filesystem::path& output_path);

void SetupDemoScene(DemoSetup demo_setup, ApplicationInitializationSettings& application_info,
                    const std::filesystem::path& resource_folder_path = {}, bool clear_generated_project_files = true);

}  // namespace evo_engine
