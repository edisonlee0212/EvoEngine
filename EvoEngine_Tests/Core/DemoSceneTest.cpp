#include "EvoEngine_SDK_PCH.hpp"

#include <gtest/gtest.h>

#include "DemoScene.hpp"

#include <chrono>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <string>

using namespace evo_engine;

namespace {
class TempDemoResources {
 public:
  TempDemoResources() {
    const auto now = std::chrono::steady_clock::now().time_since_epoch().count();
    root_ = std::filesystem::temp_directory_path() / ("EvoEngineDemoResourcesTest_" + std::to_string(now));
    std::filesystem::create_directories(root_ / "EvoEngine-DemoProjects/Rendering/Assets");
  }

  ~TempDemoResources() {
    std::error_code error;
    std::filesystem::remove_all(root_, error);
  }

  [[nodiscard]] std::filesystem::path RootPath() const {
    return root_;
  }

  void WriteFile(const std::filesystem::path& relative_path) const {
    const auto path = root_ / relative_path;
    std::filesystem::create_directories(path.parent_path());
    std::ofstream file(path);
    file << "test";
  }

 private:
  std::filesystem::path root_;
};

std::string ReadTextFile(const std::filesystem::path& path) {
  std::ifstream file(path);
  EXPECT_TRUE(file.good()) << path.string();
  return {std::istreambuf_iterator<char>(file), std::istreambuf_iterator<char>()};
}

std::filesystem::path SourcePath(const std::filesystem::path& relative_path) {
  return std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / relative_path;
}
}  // namespace

TEST(DemoScene, ClearGeneratedDemoProjectFilesRemovesGeneratedDemoProjectMetadata) {
  TempDemoResources resources;
  resources.WriteFile("EvoEngine-DemoProjects/Rendering/Rendering.eveproj");
  resources.WriteFile("EvoEngine-DemoProjects/Rendering/Assets/New Scene.evescene");
  resources.WriteFile("EvoEngine-DemoProjects/Rendering/Assets/Texture.png.evefilemeta");
  resources.WriteFile("EvoEngine-DemoProjects/Rendering/Assets/Folder.evefoldermeta");
  resources.WriteFile("EvoEngine-DemoProjects/3DGS/3DGS.eveproj");
  resources.WriteFile("EvoEngine-DemoProjects/3DGS/Assets/GaussianSplats.evefoldermeta");
  resources.WriteFile("EvoEngine-DemoProjects/3DGS/Assets/GaussianSplats/spatial_dragon.ply.evefilemeta");
  resources.WriteFile("EvoEngine-DemoProjects/Bicycle/Bicycle.eveproj");
  resources.WriteFile("EvoEngine-DemoProjects/Bicycle/Assets/GaussianSplats.evefoldermeta");
  resources.WriteFile("EvoEngine-DemoProjects/Bicycle/Assets/GaussianSplats/bicycle.ply.evefilemeta");
  resources.WriteFile("Legacy.uescene");
  resources.WriteFile("Legacy.ueproj");
  resources.WriteFile("Legacy.umeta");
  resources.WriteFile("Legacy.ufmeta");

  ClearGeneratedDemoProjectFiles(resources.RootPath());

  EXPECT_FALSE(std::filesystem::exists(resources.RootPath() / "EvoEngine-DemoProjects/Rendering/Rendering.eveproj"));
  EXPECT_FALSE(
      std::filesystem::exists(resources.RootPath() / "EvoEngine-DemoProjects/Rendering/Assets/New Scene.evescene"));
  EXPECT_FALSE(std::filesystem::exists(resources.RootPath() /
                                       "EvoEngine-DemoProjects/Rendering/Assets/Texture.png.evefilemeta"));
  EXPECT_FALSE(
      std::filesystem::exists(resources.RootPath() / "EvoEngine-DemoProjects/Rendering/Assets/Folder.evefoldermeta"));
  EXPECT_TRUE(std::filesystem::exists(resources.RootPath() / "EvoEngine-DemoProjects/3DGS/3DGS.eveproj"));
  EXPECT_TRUE(std::filesystem::exists(resources.RootPath() /
                                      "EvoEngine-DemoProjects/3DGS/Assets/GaussianSplats.evefoldermeta"));
  EXPECT_TRUE(std::filesystem::exists(
      resources.RootPath() / "EvoEngine-DemoProjects/3DGS/Assets/GaussianSplats/spatial_dragon.ply.evefilemeta"));
  EXPECT_TRUE(std::filesystem::exists(resources.RootPath() / "EvoEngine-DemoProjects/Bicycle/Bicycle.eveproj"));
  EXPECT_TRUE(std::filesystem::exists(resources.RootPath() /
                                      "EvoEngine-DemoProjects/Bicycle/Assets/GaussianSplats.evefoldermeta"));
  EXPECT_TRUE(std::filesystem::exists(resources.RootPath() /
                                      "EvoEngine-DemoProjects/Bicycle/Assets/GaussianSplats/bicycle.ply.evefilemeta"));
  EXPECT_FALSE(std::filesystem::exists(resources.RootPath() / "Legacy.uescene"));
  EXPECT_FALSE(std::filesystem::exists(resources.RootPath() / "Legacy.ueproj"));
  EXPECT_TRUE(std::filesystem::exists(resources.RootPath() / "Legacy.umeta"));
  EXPECT_TRUE(std::filesystem::exists(resources.RootPath() / "Legacy.ufmeta"));
}

TEST(DemoScene, ClearGeneratedProceduralGalaxyProjectFilesOnlyRemovesUniverseGeneratedProjectFiles) {
  TempDemoResources resources;
  resources.WriteFile("EvoEngine-DemoProjects/Universe/ProceduralGalaxy.eveproj");
  resources.WriteFile("EvoEngine-DemoProjects/Universe/Assets/New Scene.evescene");
  resources.WriteFile("EvoEngine-DemoProjects/Universe/Assets/Texture.png.evefilemeta");
  resources.WriteFile("EvoEngine-DemoProjects/Universe/Assets/Folder.evefoldermeta");
  resources.WriteFile("EvoEngine-DemoProjects/Universe/Assets/Reference.png");
  resources.WriteFile("EvoEngine-DemoProjects/Rendering/Rendering.eveproj");
  resources.WriteFile("EvoEngine-DemoProjects/Rendering/Assets/New Scene.evescene");

  ClearGeneratedProceduralGalaxyProjectFiles(resources.RootPath());

  EXPECT_FALSE(
      std::filesystem::exists(resources.RootPath() / "EvoEngine-DemoProjects/Universe/ProceduralGalaxy.eveproj"));
  EXPECT_FALSE(
      std::filesystem::exists(resources.RootPath() / "EvoEngine-DemoProjects/Universe/Assets/New Scene.evescene"));
  EXPECT_FALSE(
      std::filesystem::exists(resources.RootPath() / "EvoEngine-DemoProjects/Universe/Assets/Texture.png.evefilemeta"));
  EXPECT_FALSE(
      std::filesystem::exists(resources.RootPath() / "EvoEngine-DemoProjects/Universe/Assets/Folder.evefoldermeta"));
  EXPECT_TRUE(std::filesystem::exists(resources.RootPath() / "EvoEngine-DemoProjects/Universe/Assets/Reference.png"));
  EXPECT_TRUE(std::filesystem::exists(resources.RootPath() / "EvoEngine-DemoProjects/Rendering/Rendering.eveproj"));
  EXPECT_TRUE(
      std::filesystem::exists(resources.RootPath() / "EvoEngine-DemoProjects/Rendering/Assets/New Scene.evescene"));
}

TEST(DemoScene, GaussianSplatDemoCamerasUseBlackClearBackground) {
  const auto demo_scene = ReadTextFile(SourcePath("EvoEngine_App/src/DemoScene.cpp"));
  const auto begin = demo_scene.find("void ConfigureGaussianSplatDemoSceneImpl");
  const auto end = demo_scene.find("// DDGI_VALIDATION_ENTITY_LOOKUP_HELPER_BEGIN", begin);
  ASSERT_NE(begin, std::string::npos);
  ASSERT_NE(end, std::string::npos);
  const auto gaussian_setup = demo_scene.substr(begin, end - begin);

  EXPECT_NE(gaussian_setup.find("main_camera->skybox.Clear()"), std::string::npos);
  EXPECT_NE(
      gaussian_setup.find("main_camera->camera_settings.background_source = Camera::BackgroundSource::ClearColor"),
      std::string::npos);
  EXPECT_NE(gaussian_setup.find("main_camera->camera_settings.clear_color = glm::vec4(0.0f, 0.0f, 0.0f, 1.0f)"),
            std::string::npos);
  EXPECT_NE(gaussian_setup.find("main_camera->camera_settings.background_intensity = 0.0f"), std::string::npos);
  EXPECT_NE(gaussian_setup.find("scene_camera->skybox.Clear()"), std::string::npos);
  EXPECT_NE(
      gaussian_setup.find("scene_camera->camera_settings.background_source = Camera::BackgroundSource::ClearColor"),
      std::string::npos);
  EXPECT_NE(gaussian_setup.find("scene_camera->camera_settings.clear_color = glm::vec4(0.0f, 0.0f, 0.0f, 1.0f)"),
            std::string::npos);
  EXPECT_NE(gaussian_setup.find("scene_camera->camera_settings.background_intensity = 0.0f"), std::string::npos);
  EXPECT_EQ(gaussian_setup.find("Camera::BackgroundSource::Cubemap"), std::string::npos);
}
