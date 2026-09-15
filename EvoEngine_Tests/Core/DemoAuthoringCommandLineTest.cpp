#include <gtest/gtest.h>

#include "DemoAuthoringCommandLine.hpp"

#include <chrono>

using namespace evo_engine;

namespace {
class DemoAuthoringCommandLineTest : public testing::Test {
 protected:
  void SetUp() override {
    const auto suffix = std::chrono::steady_clock::now().time_since_epoch().count();
    root_ = std::filesystem::temp_directory_path() / ("EvoEngineDemoAuthoring_" + std::to_string(suffix));
    resources_ = root_ / "Resources";
    std::filesystem::create_directories(resources_);
  }

  void TearDown() override {
    std::error_code error;
    std::filesystem::remove_all(root_, error);
  }

  std::filesystem::path root_;
  std::filesystem::path resources_;
};
}  // namespace

TEST_F(DemoAuthoringCommandLineTest, ParsesExplicitRootAndCompleteWarmupCount) {
  const auto parsed = ParseDemoAuthoringArguments({"--demo", "rendering", "--resource-root", resources_.string(),
                                                   "--author-runtime-scene", "--author-warmup-frames", "3"});
  EXPECT_TRUE(parsed.enabled);
  ASSERT_TRUE(parsed.resource_root);
  EXPECT_EQ(*parsed.resource_root, std::filesystem::absolute(resources_));
  EXPECT_EQ(parsed.warmup_frames, 3u);
  EXPECT_NO_THROW(ValidateDemoAuthoringArguments(parsed, true, false));

  EXPECT_THROW(ParseDemoAuthoringArguments({"--author-warmup-frames", "3junk"}), std::invalid_argument);
  EXPECT_THROW(ParseDemoAuthoringArguments({"--author-warmup-frames", "-1"}), std::invalid_argument);
  EXPECT_THROW(ParseDemoAuthoringArguments({"--resource-root"}), std::invalid_argument);
}

TEST_F(DemoAuthoringCommandLineTest, ValidatesRequiredDemoRootAndIncompatibleModes) {
  const auto valid = ParseDemoAuthoringArguments({"--resource-root", resources_.string(), "--author-runtime-scene"});
  EXPECT_THROW(ValidateDemoAuthoringArguments(valid, false, false), std::invalid_argument);
  EXPECT_THROW(ValidateDemoAuthoringArguments(valid, true, true), std::invalid_argument);

  const auto missing_root = ParseDemoAuthoringArguments({"--author-runtime-scene"});
  EXPECT_THROW(ValidateDemoAuthoringArguments(missing_root, true, false), std::invalid_argument);

  const auto wrong_root = ParseDemoAuthoringArguments({"--resource-root", root_.string()});
  EXPECT_THROW(ValidateDemoAuthoringArguments(wrong_root, true, false), std::invalid_argument);

  const auto warmup_only = ParseDemoAuthoringArguments({"--author-warmup-frames", "1"});
  EXPECT_THROW(ValidateDemoAuthoringArguments(warmup_only, true, false), std::invalid_argument);
}
