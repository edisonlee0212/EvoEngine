#include "EvoEngine_SDK_PCH.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cctype>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <regex>
#include <string>
#include <vector>

namespace {
const std::vector<std::filesystem::path> kRenderingGuides = {
    "docs/rendering.md",         "docs/rendering-materials.md", "docs/ddgi.md",
    "docs/reflection-probes.md", "docs/rendering-demos.md",     "docs/rendering-validation.md",
};

std::filesystem::path RepoPath(const std::filesystem::path& relative_path) {
  return std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / relative_path;
}

std::string ReadTextFile(const std::filesystem::path& path) {
  std::ifstream file(path);
  EXPECT_TRUE(file.good()) << path.string();
  return {std::istreambuf_iterator<char>(file), std::istreambuf_iterator<char>()};
}

size_t CountWords(const std::string& text) {
  const std::regex word(R"(\b[[:alnum:]_]+\b)");
  return static_cast<size_t>(
      std::distance(std::sregex_iterator(text.begin(), text.end(), word), std::sregex_iterator()));
}
}  // namespace

TEST(RenderingDocumentation, OverviewLinksEveryFocusedGuide) {
  const auto overview = ReadTextFile(RepoPath("docs/rendering.md"));
  ASSERT_FALSE(overview.empty());

  for (auto guide = std::next(kRenderingGuides.begin()); guide != kRenderingGuides.end(); ++guide) {
    EXPECT_NE(overview.find(guide->filename().string()), std::string::npos) << guide->string();
  }
}

TEST(RenderingDocumentation, ExcludesDevelopmentHistory) {
  const std::regex milestone_id(R"(\bM[0-9]+\b)");
  const std::vector<std::string> rejected_terms = {"closeout", "frozen baseline", "this branch", "milestone"};

  for (const auto& guide : kRenderingGuides) {
    const auto contents = ReadTextFile(RepoPath(guide));
    ASSERT_FALSE(contents.empty()) << guide.string();
    EXPECT_FALSE(std::regex_search(contents, milestone_id)) << guide.string();

    auto lower = contents;
    std::transform(lower.begin(), lower.end(), lower.begin(), [](const unsigned char character) {
      return static_cast<char>(std::tolower(character));
    });
    for (const auto& term : rejected_terms) {
      EXPECT_EQ(lower.find(term), std::string::npos) << guide.string() << " contains " << term;
    }
  }
}

TEST(RenderingDocumentation, StaysWithinReadingBudget) {
  size_t total_words = 0;
  for (const auto& guide : kRenderingGuides) {
    total_words += CountWords(ReadTextFile(RepoPath(guide)));
  }

  EXPECT_LE(CountWords(ReadTextFile(RepoPath("docs/rendering.md"))), 2000u);
  EXPECT_LE(total_words, 9000u);
}

TEST(RenderingDocumentation, GraphicsValidationIsOptionalAndDisabledByDefault) {
  const auto root_cmake = ReadTextFile(RepoPath("CMakeLists.txt"));
  const auto sdk_cmake = ReadTextFile(RepoPath("EvoEngine_SDK/CMakeLists.txt"));
  const auto validation_guide = ReadTextFile(RepoPath("docs/rendering-validation.md"));

  EXPECT_NE(root_cmake.find(
                "option(EVOENGINE_ENABLE_GRAPHICS_VALIDATION \"Enable Vulkan validation in Debug and RelWithDebInfo "
                "builds.\" OFF)"),
            std::string::npos);
  EXPECT_NE(sdk_cmake.find("$<BOOL:${EVOENGINE_ENABLE_GRAPHICS_VALIDATION}>"), std::string::npos);
  EXPECT_NE(validation_guide.find("-DEVOENGINE_ENABLE_GRAPHICS_VALIDATION=ON"), std::string::npos);
}
