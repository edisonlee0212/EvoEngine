#include <gtest/gtest.h>
#include <stdexcept>
#include "PathUtils.hpp"
#include "RuntimeConfiguration.hpp"
#include "RuntimePaths.hpp"

using namespace evo_engine;

TEST(RuntimeConfiguration, ConsoleVisibilityPreservesFileLogging) {
  RuntimeConfiguration config;
  config.project = "Project/Test.eveproj";
  EXPECT_TRUE(config.ApplicationSettings().hide_console_window);
  config.show_console = true;
  EXPECT_FALSE(config.ApplicationSettings().hide_console_window);
  EXPECT_FALSE(config.ApplicationSettings().redirect_standard_streams_to_console);
}

TEST(RuntimePaths, ResolvesFromExecutableRatherThanWorkingDirectory) {
  struct RestoreWorkingDirectory {
    std::filesystem::path path = std::filesystem::current_path();
    ~RestoreWorkingDirectory() {
      std::filesystem::current_path(path);
    }
  } restore;
  std::filesystem::current_path(std::filesystem::temp_directory_path());
  EXPECT_EQ(runtime_paths::Root(), path_utils::CurrentExecutablePath().parent_path());
  EXPECT_EQ(runtime_paths::Resolve("UserData/save.dat"),
            std::filesystem::weakly_canonical(runtime_paths::Root() / "UserData/save.dat"));
}

TEST(RuntimePaths, DefaultsToNonStrictBeforeApplicationInitialization) {
  EXPECT_FALSE(runtime_paths::IsStrict());
}

TEST(RuntimePaths, RejectsPathsOutsideDistribution) {
  EXPECT_THROW(runtime_paths::Resolve("../save.dat"), std::runtime_error);
  EXPECT_THROW(runtime_paths::Resolve("UserData/../../save.dat"), std::runtime_error);
  EXPECT_THROW(runtime_paths::Resolve(runtime_paths::Root() / "save.dat"), std::runtime_error);
  EXPECT_THROW(runtime_paths::Resolve({}), std::runtime_error);
}
