#include "EvoEngine_SDK_PCH.hpp"

#include <filesystem>
#include <fstream>
#include <iterator>
#include <string>

#include <gtest/gtest.h>

namespace {
std::string ReadTextFile(const std::filesystem::path& path) {
  std::ifstream file(path);
  EXPECT_TRUE(file.good()) << path.string();
  return {std::istreambuf_iterator<char>(file), std::istreambuf_iterator<char>()};
}
}  // namespace

TEST(VolumetricCloudShader, SharedLibraryDefinesV1Contract) {
  const auto shader_source =
      ReadTextFile(std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / "EvoEngine_SDK" / "Internals" /
                   "DefaultResources" / "Shaders" / "Includes" / "VolumetricClouds.glsl");
  ASSERT_FALSE(shader_source.empty());

  EXPECT_NE(shader_source.find("struct VolumetricCloudSettingsGpu"), std::string::npos);
  EXPECT_NE(shader_source.find("struct VolumetricCloudMarchResult"), std::string::npos);
  EXPECT_NE(shader_source.find("EE_VOLUMETRIC_CLOUD_SampleDensity"), std::string::npos);
  EXPECT_NE(shader_source.find("EE_VOLUMETRIC_CLOUD_HenyeyGreensteinPhase"), std::string::npos);
  EXPECT_NE(shader_source.find("EE_VOLUMETRIC_CLOUD_TemporalJitter"), std::string::npos);
  EXPECT_NE(shader_source.find("EE_VOLUMETRIC_CLOUD_LightTransmittance"), std::string::npos);
  EXPECT_NE(shader_source.find("EE_VOLUMETRIC_CLOUD_March"), std::string::npos);
  EXPECT_NE(shader_source.find("EE_VOLUMETRIC_CLOUD_Composite"), std::string::npos);
  EXPECT_NE(shader_source.find("EE_VOLUMETRIC_CLOUD_Debug"), std::string::npos);
}

TEST(VolumetricCloudShader, SharedLibraryStaysIndependentFromDdgiAndShaderForks) {
  const auto shader_source =
      ReadTextFile(std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / "EvoEngine_SDK" / "Internals" /
                   "DefaultResources" / "Shaders" / "Includes" / "VolumetricClouds.glsl");
  ASSERT_FALSE(shader_source.empty());

  EXPECT_EQ(shader_source.find("DDGI"), std::string::npos);
  EXPECT_EQ(shader_source.find("HLSL"), std::string::npos);
  EXPECT_EQ(shader_source.find(".hlsl"), std::string::npos);
  EXPECT_EQ(shader_source.find("#ifdef RASTER"), std::string::npos);
  EXPECT_EQ(shader_source.find("#ifdef RAY_TRACING"), std::string::npos);
}
