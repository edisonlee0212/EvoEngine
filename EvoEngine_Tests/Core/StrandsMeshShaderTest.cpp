#include "EvoEngine_SDK_PCH.hpp"

#include "Application.hpp"
#include "ApplicationContext.hpp"
#include "GeometryStorage.hpp"
#include "Jobs.hpp"
#include "Strands.hpp"

#include <gtest/gtest.h>

#include <cstddef>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <string>

namespace {
std::string ReadRepoFile(const std::filesystem::path& path) {
  std::ifstream file(std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / path);
  EXPECT_TRUE(file.good()) << path.string();
  return {std::istreambuf_iterator<char>(file), std::istreambuf_iterator<char>()};
}

std::filesystem::path ShaderPath(const std::filesystem::path& path) {
  return std::filesystem::path("EvoEngine_SDK/Internals/DefaultResources/Shaders") / path;
}
}  // namespace

TEST(StrandsMeshShader, StorageAbiAndDispatchContract) {
  EXPECT_EQ(sizeof(evo_engine::StrandPoint), 48);
  EXPECT_EQ(sizeof(evo_engine::StrandPointDataChunk), 48 * evo_engine::Platform::Constants::meshlet_max_vertices_size);
  EXPECT_EQ(offsetof(evo_engine::StrandMeshlet, strand_points_size),
            4 * evo_engine::Platform::Constants::meshlet_max_triangles_size);
  EXPECT_EQ(sizeof(evo_engine::StrandMeshlet), 4 * evo_engine::Platform::Constants::meshlet_max_triangles_size + 12);
  EXPECT_EQ(evo_engine::Platform::Constants::meshlet_max_vertices_size, 64);
  EXPECT_EQ(evo_engine::Platform::Constants::meshlet_max_triangles_size, 40);

  const auto task = ReadRepoFile(ShaderPath("Graphics/Task/Standard/StandardStrands.task"));
  const auto mesh = ReadRepoFile(ShaderPath("Graphics/Mesh/Standard/StandardStrands.mesh"));
  const auto render_instances = ReadRepoFile("EvoEngine_SDK/src/RenderInstanceStorage.cpp");
  EXPECT_NE(task.find("layout(local_size_x = 1)"), std::string::npos);
  EXPECT_NE(task.find("EE_INSTANCES[instance_index].meshlet_offset + gl_WorkGroupID.x"), std::string::npos);
  EXPECT_NE(task.find("EmitMeshTasksEXT(interval_count, 1, 1)"), std::string::npos);
  EXPECT_NE(mesh.find("layout(max_vertices = 32, max_primitives = 30)"), std::string::npos);
  EXPECT_NE(mesh.find("uint STRAND_RING_MAX = 15"), std::string::npos);
  EXPECT_NE(render_instances.find("DrawMeshTasks(vk_command_buffer, strands->strand_meshlet_range_->prev_frame_range)"),
            std::string::npos);
  EXPECT_NE(render_instances.find("if (!graphics_pipeline->mesh_shader)"), std::string::npos);
}

TEST(StrandsMeshShader, BeautyPathUsesCorrectFrameAndBoundedSubdivision) {
  const auto include = ReadRepoFile(ShaderPath("Includes/StrandMeshlet.glsl"));
  const auto settings = ReadRepoFile(ShaderPath("Includes/Strands.glsl"));
  const auto mesh = ReadRepoFile(ShaderPath("Graphics/Mesh/Standard/StandardStrands.mesh"));
  EXPECT_NE(include.find("normal - t * dot(normal, t)"), std::string::npos);
  EXPECT_NE(mesh.find("transpose(inverse(mat3(model)))"), std::string::npos);
  EXPECT_NE(mesh.find("normal_matrix * radial"), std::string::npos);
  EXPECT_NE(mesh.find("mat3(model) * tangent"), std::string::npos);
  EXPECT_NE(mesh.find("(ring_size + 1) * 2"), std::string::npos);
  EXPECT_NE(settings.find("EE_RENDER_INFO.strand_subdivision_y"), std::string::npos);
  EXPECT_NE(settings.find("min(EE_RENDER_INFO.strand_subdivision_max_y, 15)"), std::string::npos);
  EXPECT_NE(settings.find("* 50.0 /"), std::string::npos);
}

TEST(StrandsMeshShader, ThicknessAwareBoundsUpdateWithGeometry) {
  evo_engine::Application application;
  evo_engine::ApplicationContextScope context(application);
  evo_engine::Jobs::Initialize(2);
  evo_engine::StrandPointAttributes attributes;
  attributes.normal = true;
  std::vector<evo_engine::StrandPoint> points(4);
  for (size_t i = 0; i < points.size(); ++i) {
    points[i].position = glm::vec3(static_cast<float>(i) / 3.0f, 0.0f, 0.0f);
    points[i].thickness = 0.25f;
    points[i].normal = glm::vec3(0.0f, 1.0f, 0.0f);
  }
  evo_engine::Strands strands;
  strands.OnCreate();
  strands.SetSegments(attributes, {0}, points);
  auto bound = strands.GetBound();
  EXPECT_EQ(bound.min, glm::vec3(-0.25f));
  EXPECT_EQ(bound.max, glm::vec3(1.25f, 0.25f, 0.25f));

  points[3].position.x = 2.0f;
  points[3].thickness = 0.5f;
  const auto version = strands.GetVersion();
  strands.SetSegments(attributes, {0}, points);
  bound = strands.GetBound();
  EXPECT_GT(strands.GetVersion(), version);
  EXPECT_EQ(bound.min, glm::vec3(-0.25f, -0.5f, -0.5f));
  EXPECT_EQ(bound.max, glm::vec3(2.5f, 0.5f, 0.5f));
  evo_engine::Jobs::OnDestroy();
}

TEST(StrandsMeshShader, UnmigratedLegacyDrawsAreNotReachable) {
  const auto render_layer = ReadRepoFile("EvoEngine_SDK/src/RenderLayer.cpp");
  const auto strands = ReadRepoFile("EvoEngine_SDK/src/Strands.cpp");
  const auto render_instances = ReadRepoFile("EvoEngine_SDK/src/RenderInstanceStorage.cpp");
  const auto deferred = ReadRepoFile("EvoEngine_SDK/src/RenderPasses/DeferredGeometryPass.cpp");
  EXPECT_EQ(deferred.find("BindStrandPoints"), std::string::npos);
  EXPECT_EQ(deferred.find("strands->DrawIndexed"), std::string::npos);
  EXPECT_EQ(render_layer.find("for (const auto& i : editor_layer->gizmo_strands_tasks_)"), std::string::npos);
  EXPECT_NE(render_layer.find("StandardStrands.task"), std::string::npos);
  EXPECT_NE(render_layer.find("StandardStrands.mesh"), std::string::npos);
  EXPECT_NE(strands.find("Platform::Initialized() && Platform::MeshShaderEnabled()"), std::string::npos);
  EXPECT_NE(render_instances.find("if (!Platform::MeshShaderEnabled() || !strands->strand_meshlet_range_"),
            std::string::npos);
}

TEST(StrandsMeshShader, ValidationFixtureCoversReuploadAndBothShadowFlags) {
  const auto scene = ReadRepoFile("EvoEngine_App/src/DemoScene.cpp");
  const auto editor = ReadRepoFile("EvoEngine_App/src/EvoEngineEditor.cpp");
  EXPECT_NE(scene.find("CreateStrandValidationGeometry(4"), std::string::npos);
  EXPECT_NE(scene.find("CreateStrandValidationGeometry(58"), std::string::npos);
  EXPECT_NE(scene.find("glm::vec3(-1.15f, 0.8f, 1.35f), true"), std::string::npos);
  EXPECT_NE(scene.find("glm::vec3(1.0f), false"), std::string::npos);
  EXPECT_NE(scene.find("UpdateStrandMeshShaderValidationGeometry"), std::string::npos);
  EXPECT_NE(editor.find("--preview-strand-fixture"), std::string::npos);
  EXPECT_NE(editor.find("StrandFixtureTelemetryJson"), std::string::npos);
  EXPECT_NE(editor.find("\"meshlet_count\""), std::string::npos);
  EXPECT_NE(editor.find("\"directional_shadow_strand_cascades\""), std::string::npos);
  EXPECT_NE(editor.find("GeometryStorage::HasPendingMeshUploads()"), std::string::npos);
}

TEST(StrandsMeshShader, DirectionalShadowUsesFixedMeshTopologyAndCasterAccounting) {
  const auto task = ReadRepoFile(ShaderPath("Graphics/Task/Lighting/StrandsShadowMap.task"));
  const auto mesh = ReadRepoFile(ShaderPath("Graphics/Mesh/Lighting/DirectionalLightStrandsShadowMap.mesh"));
  const auto common = ReadRepoFile(ShaderPath("Includes/StrandShadowMesh.glsl"));
  const auto render_layer = ReadRepoFile("EvoEngine_SDK/src/RenderLayer.cpp");
  const auto pass = ReadRepoFile("EvoEngine_SDK/src/RenderPasses/DirectionalLightShadowPass.cpp");
  EXPECT_NE(task.find("EmitMeshTasksEXT(EE_STRAND_MESHLETS[meshlet_index].segment_size, 1, 1)"), std::string::npos);
  EXPECT_NE(common.find("layout(max_vertices = 10, max_primitives = 8)"), std::string::npos);
  EXPECT_NE(common.find("const uint STRAND_SHADOW_RING_SIZE = 4"), std::string::npos);
  EXPECT_NE(mesh.find("EE_DIRECTIONAL_LIGHTS[EE_CAMERA_INDEX].light_space_matrix[EE_LIGHT_SPLIT_INDEX]"),
            std::string::npos);
  EXPECT_NE(render_layer.find("DirectionalLightStrandsShadowMap.mesh"), std::string::npos);
  EXPECT_NE(render_layer.find("strand_meshlet_descriptor_sets_[current_frame_index]"), std::string::npos);
  EXPECT_NE(pass.find("parameters.strand_meshlet_descriptor_set->GetVkDescriptorSet()"), std::string::npos);
  EXPECT_NE(pass.find("ForEachStrandsRenderInstance"), std::string::npos);
  EXPECT_NE(pass.find("DirectionalShadowCasterKind::Strands"), std::string::npos);
  EXPECT_NE(pass.find("DirectionalShadowCasterKind::Strands, RenderDrawCallKind::Direct, 0, split"), std::string::npos);
  EXPECT_NE(pass.find("states.cull_mode = render_instance->cull_mode"), std::string::npos);
}

TEST(StrandsMeshShader, PunctualShadowsUseMeshPipelinesAndExactFixtureTelemetry) {
  const auto point = ReadRepoFile(ShaderPath("Graphics/Mesh/Lighting/PointLightStrandsShadowMap.mesh"));
  const auto spot = ReadRepoFile(ShaderPath("Graphics/Mesh/Lighting/SpotLightStrandsShadowMap.mesh"));
  const auto common = ReadRepoFile(ShaderPath("Includes/StrandShadowMesh.glsl"));
  const auto render_layer = ReadRepoFile("EvoEngine_SDK/src/RenderLayer.cpp");
  const auto platform = ReadRepoFile("EvoEngine_SDK/include/Rendering/Platform/Platform.hpp");
  const auto scene = ReadRepoFile("EvoEngine_App/src/DemoScene.cpp");
  const auto editor = ReadRepoFile("EvoEngine_App/src/EvoEngineEditor.cpp");

  EXPECT_NE(common.find("layout(max_vertices = 10, max_primitives = 8)"), std::string::npos);
  EXPECT_NE(common.find("const uint STRAND_SHADOW_RING_SIZE = 4"), std::string::npos);
  EXPECT_NE(point.find("EE_POINT_LIGHTS[EE_CAMERA_INDEX].light_space_matrix[EE_LIGHT_SPLIT_INDEX]"), std::string::npos);
  EXPECT_NE(spot.find("EE_SPOT_LIGHTS[EE_CAMERA_INDEX].light_space_matrix"), std::string::npos);
  EXPECT_NE(render_layer.find("PointLightStrandsShadowMap.mesh"), std::string::npos);
  EXPECT_NE(render_layer.find("SpotLightStrandsShadowMap.mesh"), std::string::npos);
  EXPECT_EQ(render_layer.find("PointLightShadowMapStrands"), std::string::npos);
  EXPECT_EQ(render_layer.find("SpotLightShadowMapStrands"), std::string::npos);
  EXPECT_NE(render_layer.find("strand_meshlet_descriptor_sets_[current_frame_index]"), std::string::npos);
  EXPECT_NE(render_layer.find("use_mesh_shader ? strands_point_light_shadow_pipeline : nullptr"), std::string::npos);
  EXPECT_NE(render_layer.find("use_mesh_shader ? strands_spot_light_shadow_pipeline : nullptr"), std::string::npos);
  EXPECT_NE(render_layer.find("pipeline->states.cull_mode = render_instance->cull_mode"), std::string::npos);
  EXPECT_NE(render_layer.find("Platform::CountShadowStrandDraw"), std::string::npos);
  EXPECT_NE(platform.find("point_shadow_strand_face_draw_calls"), std::string::npos);
  EXPECT_NE(platform.find("spot_shadow_strand_draw_calls"), std::string::npos);
  EXPECT_NE(scene.find("ConfigureStrandPunctualShadowValidation"), std::string::npos);
  EXPECT_NE(scene.find("const std::array point_directions"), std::string::npos);
  EXPECT_NE(editor.find("--preview-strand-punctual-fixture"), std::string::npos);
  EXPECT_NE(editor.find("StrandPunctualFixtureTelemetryJson"), std::string::npos);
  EXPECT_NE(editor.find("\"point_shadow_strand_faces\""), std::string::npos);
  EXPECT_NE(editor.find("\"spot_shadow_strands\""), std::string::npos);
  EXPECT_NE(editor.find("\"draw_scope\"] = \"frame-global\""), std::string::npos);
  EXPECT_NE(editor.find("expected_point_face_draws = {1, 1, 1, 1, 1, 1}"), std::string::npos);
  EXPECT_NE(editor.find("result[\"pass\"] = failures.empty()"), std::string::npos);
  EXPECT_NE(editor.find("!scene->IsEntityEnabled(owner) || !renderer"), std::string::npos);
  EXPECT_NE(editor.find("Strand punctual-shadow validation failed"), std::string::npos);
}
