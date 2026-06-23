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

TEST(VolumetricCloudShader, RasterComputeUsesSharedLibraryAndRasterDepth) {
  const auto shader_source =
      ReadTextFile(std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / "EvoEngine_SDK" / "Internals" /
                   "DefaultResources" / "Shaders" / "Compute" / "VolumetricClouds.comp");
  ASSERT_FALSE(shader_source.empty());

  EXPECT_NE(shader_source.find("#include \"VolumetricClouds.glsl\""), std::string::npos);
  EXPECT_NE(shader_source.find("layout(set = 1, binding = 0) uniform sampler2D inDepth"), std::string::npos);
  EXPECT_NE(shader_source.find("layout(set = 1, binding = 1, rgba32f) uniform image2D inOutColor"), std::string::npos);
  EXPECT_NE(shader_source.find("layout(set = 1, binding = 2, rgba16f) uniform writeonly image2D outCloudAccumulation"),
            std::string::npos);
  EXPECT_NE(shader_source.find("layout(set = 1, binding = 3, r16f) uniform writeonly image2D outCloudTransmittance"),
            std::string::npos);
  EXPECT_NE(shader_source.find("EE_DEPTH_TO_WORLD_POS"), std::string::npos);
  EXPECT_NE(shader_source.find("EE_DIRECTIONAL_LIGHTS"), std::string::npos);
  EXPECT_NE(shader_source.find("EE_VOLUMETRIC_CLOUD_March"), std::string::npos);
  EXPECT_NE(shader_source.find("imageStore(inOutColor"), std::string::npos);
  EXPECT_EQ(shader_source.find("DDGI"), std::string::npos);
  EXPECT_EQ(shader_source.find("HLSL"), std::string::npos);
}

TEST(VolumetricCloudShader, RasterPassBindsComputePipelineAndGraphResources) {
  const auto pass_source = ReadTextFile(std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / "EvoEngine_SDK" / "src" /
                                        "RenderPasses" / "VolumetricCloudsPass.cpp");
  ASSERT_FALSE(pass_source.empty());

  EXPECT_NE(pass_source.find("CreateGraphImageMipView(accumulation_binding->image, 0)"), std::string::npos);
  EXPECT_NE(pass_source.find("CreateGraphImageMipView(transmittance_binding->image, 0)"), std::string::npos);
  EXPECT_NE(pass_source.find("descriptor_set->UpdateImageDescriptorBinding(0, image_info)"), std::string::npos);
  EXPECT_NE(pass_source.find("descriptor_set->UpdateImageDescriptorBinding(1, image_info)"), std::string::npos);
  EXPECT_NE(pass_source.find("parameters.pipeline->Dispatch"), std::string::npos);

  const auto render_layer_source =
      ReadTextFile(std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / "EvoEngine_SDK" / "src" / "RenderLayer.cpp");
  ASSERT_FALSE(render_layer_source.empty());
  EXPECT_NE(render_layer_source.find("Shaders/Compute/VolumetricClouds.comp"), std::string::npos);
  EXPECT_NE(render_layer_source.find("scene->environment.volumetric_cloud_settings"), std::string::npos);
  EXPECT_NE(render_layer_source.find("volumetric_clouds_pipeline_"), std::string::npos);
}
