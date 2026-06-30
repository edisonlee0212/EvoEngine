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

std::filesystem::path ShaderRoot() {
  return std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / "EvoEngine_SDK" / "Internals" / "DefaultResources" /
         "Shaders";
}
}  // namespace

TEST(GaussianSplatShader, VertexAndMeshShadersUseSharedProjectionLibrary) {
  const auto shader_root = ShaderRoot();
  const auto library = ReadTextFile(shader_root / "Includes" / "GaussianSplat.glsl");
  const auto vertex_shader = ReadTextFile(shader_root / "Graphics" / "Vertex" / "GaussianSplat" / "GaussianSplat.vert");
  const auto mesh_shader = ReadTextFile(shader_root / "Graphics" / "Mesh" / "GaussianSplat" / "GaussianSplat.mesh");

  ASSERT_FALSE(library.empty());
  ASSERT_FALSE(vertex_shader.empty());
  ASSERT_FALSE(mesh_shader.empty());

  EXPECT_NE(library.find("EE_GAUSSIAN_SPLAT_BUILD_VERTEX"), std::string::npos);
  EXPECT_NE(library.find("EE_GAUSSIAN_SPLAT_SCREEN_COVARIANCE"), std::string::npos);
  EXPECT_NE(library.find("EE_GAUSSIAN_SPLAT_SH_RADIANCE"), std::string::npos);
  EXPECT_NE(library.find("EE_GAUSSIAN_SPLAT_DRAW_COUNT"), std::string::npos);
  EXPECT_NE(vertex_shader.find("#include \"GaussianSplat.glsl\""), std::string::npos);
  EXPECT_NE(mesh_shader.find("#include \"GaussianSplat.glsl\""), std::string::npos);
  EXPECT_NE(mesh_shader.find("layout(max_vertices = 128, max_primitives = 64) out"), std::string::npos);
  EXPECT_NE(mesh_shader.find("SetMeshOutputsEXT(active_splats * 4u, active_splats * 2u)"), std::string::npos);
}

TEST(GaussianSplatShader, CullPassMaintainsVertexAndMeshIndirectCounts) {
  const auto cull_shader = ReadTextFile(ShaderRoot() / "Compute" / "GaussianSplatCull.comp");

  ASSERT_FALSE(cull_shader.empty());

  EXPECT_NE(cull_shader.find("DrawIndirectCommand"), std::string::npos);
  EXPECT_NE(cull_shader.find("MeshTasksIndirectCommand"), std::string::npos);
  EXPECT_NE(cull_shader.find("layout(set = 1, binding = 4)"), std::string::npos);
  EXPECT_NE(cull_shader.find("EE_GAUSSIAN_SPLAT_MESH_SPLATS_PER_TASK"), std::string::npos);
  EXPECT_NE(cull_shader.find("atomicAdd(EE_GAUSSIAN_SPLAT_MESH_TASK_DRAW_COMMAND.groupCountX, 1u)"), std::string::npos);
}

TEST(GaussianSplatShader, RenderLayerWiresMeshPipelineAsCapabilityGatedPath) {
  const auto source_root = std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / "EvoEngine_SDK";
  const auto render_layer = ReadTextFile(source_root / "src" / "RenderLayer.cpp");
  const auto gaussian_pass = ReadTextFile(source_root / "src" / "RenderPasses" / "GaussianSplatPass.cpp");

  ASSERT_FALSE(render_layer.empty());
  ASSERT_FALSE(gaussian_pass.empty());

  EXPECT_NE(render_layer.find("Shaders/Graphics/Mesh/GaussianSplat/GaussianSplat.mesh"), std::string::npos);
  EXPECT_NE(render_layer.find("Platform::MeshShaderEnabled() && enable_meshlet"), std::string::npos);
  EXPECT_NE(gaussian_pass.find("mesh_task_indirect_draw_buffer"), std::string::npos);
  EXPECT_NE(gaussian_pass.find("Platform::DrawMeshTasksIndirect"), std::string::npos);
  EXPECT_NE(gaussian_pass.find("gaussian_instance->raster_mode != GaussianSplatRasterMode::Vertex"), std::string::npos);
}
