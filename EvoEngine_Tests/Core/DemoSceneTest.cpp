#include "EvoEngine_SDK_PCH.hpp"

#include <gtest/gtest.h>

#include "DemoScene.hpp"

#include <chrono>
#include <filesystem>
#include <fstream>

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
}  // namespace

TEST(DemoScene, ClearGeneratedDemoProjectFilesRemovesGeneratedDemoProjectMetadata) {
  TempDemoResources resources;
  resources.WriteFile("EvoEngine-DemoProjects/Rendering/Rendering.eveproj");
  resources.WriteFile("EvoEngine-DemoProjects/Rendering/Assets/New Scene.evescene");
  resources.WriteFile("EvoEngine-DemoProjects/Rendering/Assets/Texture.png.evefilemeta");
  resources.WriteFile("EvoEngine-DemoProjects/Rendering/Assets/Folder.evefoldermeta");
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
  EXPECT_FALSE(std::filesystem::exists(resources.RootPath() / "Legacy.uescene"));
  EXPECT_FALSE(std::filesystem::exists(resources.RootPath() / "Legacy.ueproj"));
  EXPECT_TRUE(std::filesystem::exists(resources.RootPath() / "Legacy.umeta"));
  EXPECT_TRUE(std::filesystem::exists(resources.RootPath() / "Legacy.ufmeta"));
}
