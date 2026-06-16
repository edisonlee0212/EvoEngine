#include "EvoEngine_SDK_PCH.hpp"

#include <gtest/gtest.h>

#include "PathUtils.hpp"

#include <chrono>
#include <filesystem>
#include <fstream>

using namespace evo_engine;

namespace {
class TempPathDirectory {
 public:
  TempPathDirectory() {
    const auto now = std::chrono::steady_clock::now().time_since_epoch().count();
    root_ = std::filesystem::temp_directory_path() / ("EvoEnginePathUtilsTest_" + std::to_string(now));
    std::filesystem::create_directories(root_);
  }

  ~TempPathDirectory() {
    std::error_code error;
    std::filesystem::remove_all(root_, error);
  }

  [[nodiscard]] std::filesystem::path RootPath() const {
    return root_;
  }

 private:
  std::filesystem::path root_;
};
}  // namespace

TEST(PathUtils, ContainmentUsesPathBoundaries) {
  TempPathDirectory temp;
  const auto assets = temp.RootPath() / "Assets";
  const auto sibling = temp.RootPath() / "AssetsBackup";
  std::filesystem::create_directories(assets);
  std::filesystem::create_directories(sibling);

  EXPECT_TRUE(path_utils::IsSameOrChildPath(assets / "Texture.bin", assets));
  EXPECT_TRUE(path_utils::IsSameOrChildPath(assets, assets));
  EXPECT_FALSE(path_utils::IsSameOrChildPath(sibling / "Texture.bin", assets));
}

TEST(PathUtils, NormalizeAbsolutePathUsesLexicalAbsolutePath) {
  const auto path = std::filesystem::path("MissingRoot") / ".." / "Project.eveproj";

  EXPECT_EQ(path_utils::NormalizeAbsolutePath(path), std::filesystem::absolute(path).lexically_normal());
}

TEST(PathUtils, CurrentExecutablePathReturnsAbsolutePath) {
  const auto executable_path = path_utils::CurrentExecutablePath("EvoEngine_Tests");

  ASSERT_FALSE(executable_path.empty());
  EXPECT_TRUE(executable_path.is_absolute());
}

TEST(PathUtils, RelativePathIfContainedReturnsContainedRelativePathOnly) {
  TempPathDirectory temp;
  const auto assets = temp.RootPath() / "Assets";
  const auto sibling = temp.RootPath() / "AssetsBackup";
  std::filesystem::create_directories(assets / "Textures");
  std::filesystem::create_directories(sibling);

  const auto relative_path = path_utils::RelativePathIfContained(assets / "Textures" / "Albedo.bin", assets);
  ASSERT_TRUE(relative_path);
  EXPECT_EQ(*relative_path, std::filesystem::path("Textures") / "Albedo.bin");
  EXPECT_FALSE(path_utils::RelativePathIfContained(sibling / "Albedo.bin", assets));
}

TEST(PathUtils, AddUniqueNormalizedPathSkipsDuplicateAbsolutePaths) {
  TempPathDirectory temp;
  std::vector<std::filesystem::path> paths;
  const auto packages = temp.RootPath() / "Packages";
  std::filesystem::create_directories(packages);

  path_utils::AddUniqueNormalizedPath(paths, packages);
  path_utils::AddUniqueNormalizedPath(paths, packages / ".");

  ASSERT_EQ(paths.size(), 1);
  EXPECT_EQ(paths.front(), path_utils::NormalizeAbsolutePath(packages));
}

TEST(PathUtils, FindExistingPathReturnsFirstExistingNormalizedPath) {
  TempPathDirectory temp;
  const auto first_missing = temp.RootPath() / "Missing";
  const auto first_existing = temp.RootPath() / "Existing";
  const auto second_existing = temp.RootPath() / "Second";
  std::filesystem::create_directories(first_existing);
  std::filesystem::create_directories(second_existing);

  EXPECT_EQ(path_utils::FindExistingPath({first_missing, first_existing, second_existing}),
            path_utils::NormalizeAbsolutePath(first_existing));
  EXPECT_TRUE(path_utils::FindExistingPath({first_missing}).empty());
}

TEST(PathUtils, FindAncestorChildPathWalksParentsWithinDepthLimit) {
  TempPathDirectory temp;
  const auto resources = temp.RootPath() / "Resources";
  const auto nested = temp.RootPath() / "Build" / "bin" / "RelWithDebInfo";
  std::filesystem::create_directories(resources);
  std::filesystem::create_directories(nested);

  EXPECT_EQ(path_utils::FindAncestorChildPath("Resources", nested, 4), path_utils::NormalizeAbsolutePath(resources));
  EXPECT_TRUE(path_utils::FindAncestorChildPath("Resources", nested, 1).empty());
}

TEST(PathUtils, GenerateUniqueChildPathUsesExistingFilesUnderRoot) {
  TempPathDirectory temp;
  const auto assets = temp.RootPath() / "Assets";
  std::filesystem::create_directories(assets / "Textures");
  std::ofstream(assets / "Textures" / "Albedo.png").close();
  std::ofstream(assets / "Textures" / "Albedo (1).png").close();

  EXPECT_EQ(path_utils::GenerateUniqueChildPath(assets, std::filesystem::path("Textures") / "Albedo", ".png"),
            std::filesystem::path("Textures") / "Albedo (2).png");
}

TEST(PathUtils, GenerateUniquePathUsesExistingAbsoluteFiles) {
  TempPathDirectory temp;
  const auto stem = temp.RootPath() / "Export";
  std::ofstream(stem.string() + ".eve").close();

  EXPECT_EQ(path_utils::GenerateUniquePath(stem, ".eve"), temp.RootPath() / "Export (1).eve");
}

TEST(PathUtils, ValidatesFilenameCharacters) {
  EXPECT_TRUE(path_utils::IsValidFileName("Project_01"));
  EXPECT_FALSE(path_utils::IsValidFileName(""));
  EXPECT_FALSE(path_utils::IsValidFileName("Bad/Name"));
  EXPECT_FALSE(path_utils::IsValidFileName("Bad*Name"));
}
