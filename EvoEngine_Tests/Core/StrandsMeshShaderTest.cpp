#include "EvoEngine_SDK_PCH.hpp"

#include "Application.hpp"
#include "ApplicationContext.hpp"
#include "EditorLayer.hpp"
#include "GeometryStorage.hpp"
#include "Jobs.hpp"
#include "Strands.hpp"

#include <gtest/gtest.h>

#include <cstddef>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <limits>
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
  EXPECT_EQ(offsetof(evo_engine::StrandMeshlet, bounding_sphere),
            4 * evo_engine::Platform::Constants::meshlet_max_triangles_size + 12);
  EXPECT_EQ(sizeof(evo_engine::StrandMeshlet), 4 * evo_engine::Platform::Constants::meshlet_max_triangles_size + 28);
  EXPECT_EQ(offsetof(evo_engine::Meshlet, bounding_sphere),
            3 * evo_engine::Platform::Constants::meshlet_max_triangles_size + 12);
  EXPECT_EQ(offsetof(evo_engine::Meshlet, normal_cone),
            3 * evo_engine::Platform::Constants::meshlet_max_triangles_size + 28);
  EXPECT_EQ(offsetof(evo_engine::Meshlet, normal_cone_apex),
            3 * evo_engine::Platform::Constants::meshlet_max_triangles_size + 44);
  EXPECT_EQ(sizeof(evo_engine::Meshlet), 3 * evo_engine::Platform::Constants::meshlet_max_triangles_size + 60);
  EXPECT_EQ(evo_engine::Platform::Constants::meshlet_max_vertices_size, 64);
  EXPECT_EQ(evo_engine::Platform::Constants::meshlet_max_triangles_size, 40);
  EXPECT_EQ(sizeof(evo_engine::GizmosPushConstant), 96u);
  EXPECT_EQ(offsetof(evo_engine::GizmosPushConstant, strand_color_mode), 92u);

  const auto task = ReadRepoFile(ShaderPath("Graphics/Task/Standard/StandardStrands.slang"));
  const auto mesh = ReadRepoFile(ShaderPath("Graphics/Mesh/Standard/StandardStrands.slang"));
  const auto render_instances = ReadRepoFile("EvoEngine_SDK/src/RenderInstanceStorage.cpp");
  EXPECT_NE(task.find("[numthreads(1, 1, 1)]"), std::string::npos);
  EXPECT_NE(task.find("EE_INSTANCES[instance_index].meshlet_offset + group_id.x"), std::string::npos);
  EXPECT_NE(task.find("DispatchMesh(interval_count, 1, 1, strand_task)"), std::string::npos);
  EXPECT_NE(mesh.find("OutputVertices<StandardRasterOutput, 32>"), std::string::npos);
  EXPECT_NE(mesh.find("OutputIndices<uint3, 30>"), std::string::npos);
  EXPECT_NE(mesh.find("uint STRAND_RING_MAX = 15u"), std::string::npos);
  EXPECT_NE(render_instances.find("DrawMeshTasks(vk_command_buffer, strands->strand_meshlet_range_->prev_frame_range)"),
            std::string::npos);
  EXPECT_NE(render_instances.find("if (!graphics_pipeline->mesh_shader)"), std::string::npos);
}

TEST(StrandsMeshShader, RigidAndStrandMeshletsStoreConservativeBounds) {
  evo_engine::Application application;
  evo_engine::ApplicationContextScope context(application);
  std::vector<evo_engine::Vertex> vertices(3);
  vertices[0].position = {-1.0f, 0.0f, 0.0f};
  vertices[1].position = {1.0f, 0.0f, 0.0f};
  vertices[2].position = {0.0f, 1.0f, 0.0f};
  std::vector<glm::uvec3> triangles{{0, 1, 2}};
  const auto meshlet_range = std::make_shared<evo_engine::RangeDescriptor>();
  const auto triangle_range = std::make_shared<evo_engine::RangeDescriptor>();
  evo_engine::GeometryStorage::AllocateMesh({}, vertices, triangles, meshlet_range, triangle_range);
  const auto& meshlet = evo_engine::GeometryStorage::PeekMeshlet(meshlet_range->offset);
  for (uint32_t index = 0; index < meshlet.vertices_size; ++index) {
    const auto& position = evo_engine::GeometryStorage::PeekVertex(
        meshlet.vertex_chunk_index * evo_engine::Platform::Constants::meshlet_max_vertices_size + index);
    EXPECT_LE(glm::distance(position.position, glm::vec3(meshlet.bounding_sphere)), meshlet.bounding_sphere.w + 1e-5f);
  }
  EXPECT_LT(meshlet.normal_cone.w, 1.0f);
  EXPECT_NEAR(glm::length(glm::vec3(meshlet.normal_cone)), 1.0f, 1e-5f);
  EXPECT_TRUE(std::isfinite(meshlet.normal_cone_apex.x));
  EXPECT_TRUE(std::isfinite(meshlet.normal_cone_apex.y));
  EXPECT_TRUE(std::isfinite(meshlet.normal_cone_apex.z));

  std::vector<evo_engine::StrandPoint> points(4);
  for (uint32_t index = 0; index < points.size(); ++index) {
    points[index].position = {static_cast<float>(index), 0.0f, 0.0f};
    points[index].thickness = index == 3 ? -0.5f : 0.25f;
  }
  const auto strand_meshlet_range = std::make_shared<evo_engine::RangeDescriptor>();
  const auto segment_range = std::make_shared<evo_engine::RangeDescriptor>();
  evo_engine::GeometryStorage::AllocateStrands({}, points, {{0, 1, 2, 3}}, strand_meshlet_range, segment_range);
  const auto& strand_meshlet = evo_engine::GeometryStorage::PeekStrandMeshlet(strand_meshlet_range->offset);
  for (uint32_t index = 0; index < strand_meshlet.strand_points_size; ++index) {
    const auto& point = evo_engine::GeometryStorage::PeekStrandPoint(
        strand_meshlet.strand_point_chunk_index * evo_engine::Platform::Constants::meshlet_max_vertices_size + index);
    EXPECT_LE(glm::distance(point.position, glm::vec3(strand_meshlet.bounding_sphere)) + std::abs(point.thickness),
              strand_meshlet.bounding_sphere.w + 1e-5f);
  }
  evo_engine::GeometryStorage::OnDestroy();
}

TEST(StrandsMeshShader, BeautyPathUsesCorrectFrameAndBoundedSubdivision) {
  const auto include = ReadRepoFile(ShaderPath("Modules/EvoEngine/StrandMeshlet.slang"));
  const auto settings = ReadRepoFile(ShaderPath("Modules/EvoEngine/Strands.slang"));
  const auto mesh = ReadRepoFile(ShaderPath("Graphics/Mesh/Standard/StandardStrands.slang"));
  EXPECT_NE(include.find("normal - normalized_tangent * dot(normal, normalized_tangent)"), std::string::npos);
  EXPECT_NE(mesh.find("EE_BUILD_STANDARD_RASTER_OUTPUT"), std::string::npos);
  EXPECT_NE(mesh.find("position + radial * thickness, radial, tangent"), std::string::npos);
  EXPECT_NE(mesh.find("(ring_size + 1u) * 2u"), std::string::npos);
  EXPECT_NE(settings.find("EE_RENDER_INFO.strand_subdivision_y"), std::string::npos);
  EXPECT_NE(settings.find("min(EE_RENDER_INFO.strand_subdivision_max_y, 15)"), std::string::npos);
  EXPECT_NE(settings.find("* 50.0f /"), std::string::npos);
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

TEST(StrandsMeshShader, RayTracingGeometryUsesEightDeterministicIntervalsPerSpan) {
  std::vector<evo_engine::StrandPoint> points(4);
  for (size_t i = 0; i < points.size(); ++i) {
    points[i].position = glm::vec3(static_cast<float>(i), 0.0f, 0.0f);
    points[i].thickness = -0.25f;
    points[i].normal = glm::vec3(0.0f, 1.0f, 0.0f);
    points[i].tex_coord = static_cast<float>(i);
    points[i].color = glm::vec4(static_cast<float>(i));
  }

  const auto geometry = evo_engine::Strands::BuildRayTracingGeometry(points, {glm::uvec4(0, 1, 2, 3)});
  ASSERT_EQ(geometry.points.size(), 9u);
  ASSERT_EQ(geometry.indices.size(), 8u);
  for (uint32_t index = 0; index < geometry.indices.size(); ++index) {
    EXPECT_EQ(geometry.indices[index], index);
  }
  EXPECT_FLOAT_EQ(geometry.points.front().position.x, 1.0f);
  EXPECT_FLOAT_EQ(geometry.points.back().position.x, 2.0f);
  EXPECT_FLOAT_EQ(geometry.points.front().thickness, 0.25f);
  EXPECT_FLOAT_EQ(geometry.points.back().thickness, 0.25f);
  EXPECT_EQ(geometry.points[4].normal, glm::vec3(0.0f, 1.0f, 0.0f));
  EXPECT_FLOAT_EQ(geometry.points[4].tex_coord, 1.5f);
  EXPECT_NEAR(geometry.points[4].color.x, 1.5f, 0.000001f);
  EXPECT_NEAR(geometry.points[4].color.y, 1.5f, 0.000001f);
  EXPECT_NEAR(geometry.points[4].color.z, 1.5f, 0.000001f);
  EXPECT_NEAR(geometry.points[4].color.w, 1.5f, 0.000001f);
}

TEST(StrandsMeshShader, RayTracingGeometryMergesSpansAndSeparatesChains) {
  std::vector<evo_engine::StrandPoint> points(8);
  for (size_t i = 0; i < points.size(); ++i) {
    points[i].position = glm::vec3(static_cast<float>(i), 0.0f, 0.0f);
    points[i].thickness = 0.1f;
  }

  const auto merged =
      evo_engine::Strands::BuildRayTracingGeometry(points, {glm::uvec4(0, 1, 2, 3), glm::uvec4(1, 2, 3, 4)});
  ASSERT_EQ(merged.points.size(), 17u);
  ASSERT_EQ(merged.indices.size(), 16u);
  for (uint32_t index = 0; index < merged.indices.size(); ++index) {
    EXPECT_EQ(merged.indices[index], index);
  }

  const auto separate =
      evo_engine::Strands::BuildRayTracingGeometry(points, {glm::uvec4(0, 1, 2, 3), glm::uvec4(4, 5, 6, 7)});
  ASSERT_EQ(separate.points.size(), 18u);
  ASSERT_EQ(separate.indices.size(), 16u);
  EXPECT_EQ(separate.indices[7], 7u);
  EXPECT_EQ(separate.indices[8], 9u);
}

TEST(StrandsMeshShader, RayTracingGeometryOmitsInvalidPrimitives) {
  std::vector<evo_engine::StrandPoint> points(8);
  for (auto& point : points) {
    point.position = glm::vec3(1.0f);
    point.thickness = 0.1f;
  }
  EXPECT_TRUE(evo_engine::Strands::BuildRayTracingGeometry(points, {glm::uvec4(0, 1, 2, 3)}).indices.empty());

  points[4].position.x = std::numeric_limits<float>::infinity();
  const auto invalid = evo_engine::Strands::BuildRayTracingGeometry(
      points, {glm::uvec4(0, 1, 2, 3), glm::uvec4(4, 5, 6, 7), glm::uvec4(8, 9, 10, 11)});
  EXPECT_TRUE(invalid.points.empty());
  EXPECT_TRUE(invalid.indices.empty());
}

TEST(StrandsMeshShader, LegacyBackendIsRemoved) {
  const auto render_layer = ReadRepoFile("EvoEngine_SDK/src/RenderLayer.cpp");
  const auto strands = ReadRepoFile("EvoEngine_SDK/src/Strands.cpp");
  const auto strands_header = ReadRepoFile("EvoEngine_SDK/include/Rendering/Geometry/Strands.hpp");
  const auto geometry_storage = ReadRepoFile("EvoEngine_SDK/src/GeometryStorage.cpp");
  const auto geometry = ReadRepoFile("EvoEngine_SDK/include/Rendering/Geometry/IGeometry.hpp");
  const auto render_instances = ReadRepoFile("EvoEngine_SDK/src/RenderInstanceStorage.cpp");
  const auto deferred = ReadRepoFile("EvoEngine_SDK/src/RenderPasses/DeferredGeometryPass.cpp");
  EXPECT_EQ(deferred.find("BindStrandPoints"), std::string::npos);
  EXPECT_EQ(deferred.find("strands->DrawIndexed"), std::string::npos);
  EXPECT_EQ(strands.find("Strands::DrawIndexed"), std::string::npos);
  EXPECT_EQ(strands_header.find("public IGeometry"), std::string::npos);
  EXPECT_EQ(geometry_storage.find("segment_buffer_"), std::string::npos);
  EXPECT_EQ(geometry.find("Strands"), std::string::npos);
  EXPECT_EQ(render_layer.find("PointLightShadowMapStrands"), std::string::npos);
  EXPECT_EQ(render_layer.find("SpotLightShadowMapStrands"), std::string::npos);
  EXPECT_EQ(render_layer.find("TessellationControl/Gizmos/GizmosStrands"), std::string::npos);
  EXPECT_NE(render_layer.find("StandardStrands.slang"), std::string::npos);
  EXPECT_NE(strands.find("Platform::Initialized() && Platform::MeshShaderEnabled()"), std::string::npos);
  EXPECT_NE(render_instances.find("const bool raster_ready = Platform::MeshShaderEnabled()"), std::string::npos);
  EXPECT_NE(render_instances.find("const bool ray_ready = Platform::RayTracingLinearSweptSpheresEnabled()"),
            std::string::npos);
  EXPECT_NE(render_instances.find("if (!raster_ready && !ray_ready)"), std::string::npos);
  EXPECT_NE(render_instances.find("strands->ray_tracing_index_range_ && strands->ray_tracing_point_range_"),
            std::string::npos);
}

TEST(StrandsMeshShader, GizmosUseMeshShadersAndThreeModeCaptureFixture) {
  const auto task = ReadRepoFile(ShaderPath("Graphics/Task/Gizmos/GizmosStrands.slang"));
  const auto mesh = ReadRepoFile(ShaderPath("Graphics/Mesh/Gizmos/GizmosStrands.slang"));
  const auto render_layer = ReadRepoFile("EvoEngine_SDK/src/RenderLayer.cpp");
  const auto inspection = ReadRepoFile("EvoEngine_SDK/src/Editor/SDKInspectionAdapters.cpp");
  const auto scene = ReadRepoFile("EvoEngine_App/src/DemoScene.cpp");
  const auto editor = ReadRepoFile("EvoEngine_App/src/EvoEngineEditor.cpp");

  EXPECT_NE(task.find("EE_GIZMOS_CONSTANTS.strand_meshlet_offset + group_id.x"), std::string::npos);
  EXPECT_NE(task.find("DispatchMesh(interval_count, 1, 1, strand_task)"), std::string::npos);
  EXPECT_NE(mesh.find("OutputVertices<EEGizmoStrandOutput, 32>"), std::string::npos);
  EXPECT_NE(mesh.find("OutputIndices<uint3, 30>"), std::string::npos);
  EXPECT_NE(mesh.find("strand_color_mode"), std::string::npos);
  EXPECT_NE(render_layer.find("Graphics/Task/Gizmos/GizmosStrands.slang"), std::string::npos);
  EXPECT_NE(render_layer.find("Graphics/Mesh/Gizmos/GizmosStrands.slang"), std::string::npos);
  EXPECT_NE(render_layer.find("for (const auto& i : editor_layer->gizmo_strands_tasks_)"), std::string::npos);
  EXPECT_NE(render_layer.find("gizmos_pipeline->DrawMeshTasks"), std::string::npos);
  EXPECT_NE(render_layer.find("Platform::CountRenderPassDraw(RenderPassDrawBucket::EditorGizmos"), std::string::npos);
  EXPECT_NE(inspection.find("Platform::MeshShaderEnabled() && ImGui::BeginTabItem(\"Strands\")"), std::string::npos);
  EXPECT_NE(scene.find("ConfigureStrandGizmoValidation"), std::string::npos);
  EXPECT_NE(scene.find("RegisterStrandGizmoValidationUpdate"), std::string::npos);
  EXPECT_NE(scene.find("GizmoSettings::ColorMode::VertexColor"), std::string::npos);
  EXPECT_NE(scene.find("GizmoSettings::ColorMode::NormalColor"), std::string::npos);
  EXPECT_NE(editor.find("--preview-strand-gizmo-fixture"), std::string::npos);
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
  EXPECT_NE(editor.find("UpdateStrandMeshShaderValidationGeometry"), std::string::npos);
}

TEST(StrandsMeshShader, DirectionalShadowUsesFixedMeshTopologyAndGenericAccounting) {
  const auto task = ReadRepoFile(ShaderPath("Graphics/Task/Lighting/StrandsShadowMap.slang"));
  const auto mesh = ReadRepoFile(ShaderPath("Graphics/Mesh/Lighting/DirectionalLightStrandsShadowMap.slang"));
  const auto render_layer = ReadRepoFile("EvoEngine_SDK/src/RenderLayer.cpp");
  const auto pass = ReadRepoFile("EvoEngine_SDK/src/RenderPasses/DirectionalLightShadowPass.cpp");
  EXPECT_NE(task.find("DispatchMesh(meshlet.segment_size, 1, 1, strand_shadow_task)"), std::string::npos);
  EXPECT_NE(mesh.find("SetMeshOutputCounts(10u, 8u)"), std::string::npos);
  EXPECT_NE(mesh.find("const uint STRAND_SHADOW_RING_SIZE = 4u"), std::string::npos);
  EXPECT_NE(mesh.find("OutputVertices<EEStrandShadowOutput, 10>"), std::string::npos);
  EXPECT_NE(mesh.find("OutputIndices<uint3, 8>"), std::string::npos);
  EXPECT_NE(mesh.find("EE_DIRECTIONAL_LIGHTS[EE_BASIC_CONSTANTS.camera_index]"), std::string::npos);
  EXPECT_NE(mesh.find("EE_BASIC_CONSTANTS.light_split_index"), std::string::npos);
  EXPECT_NE(mesh.find("mul(world_position, light_projection)"), std::string::npos);
  EXPECT_NE(render_layer.find("DirectionalLightStrandsShadowMap.slang"), std::string::npos);
  EXPECT_NE(render_layer.find("strand_meshlet_descriptor_sets_[current_frame_index]"), std::string::npos);
  EXPECT_NE(pass.find("parameters.strand_meshlet_descriptor_set->GetVkDescriptorSet()"), std::string::npos);
  EXPECT_NE(pass.find("ForEachStrandsRenderInstance"), std::string::npos);
  EXPECT_NE(pass.find("AccountDraws(parameters.count_draw_calls"), std::string::npos);
  EXPECT_NE(pass.find("Platform::CountRenderPassDraw(RenderPassDrawBucket::DirectionalLightShadow"), std::string::npos);
  EXPECT_NE(pass.find("states.cull_mode = render_instance->cull_mode"), std::string::npos);
}

TEST(StrandsMeshShader, PunctualShadowsUseMeshPipelinesAndCaptureFixture) {
  const auto point = ReadRepoFile(ShaderPath("Graphics/Mesh/Lighting/PointLightStrandsShadowMap.slang"));
  const auto spot = ReadRepoFile(ShaderPath("Graphics/Mesh/Lighting/SpotLightStrandsShadowMap.slang"));
  const auto render_layer = ReadRepoFile("EvoEngine_SDK/src/RenderLayer.cpp");
  const auto scene = ReadRepoFile("EvoEngine_App/src/DemoScene.cpp");
  const auto editor = ReadRepoFile("EvoEngine_App/src/EvoEngineEditor.cpp");

  EXPECT_NE(point.find("SetMeshOutputCounts(10u, 8u)"), std::string::npos);
  EXPECT_NE(point.find("const uint STRAND_SHADOW_RING_SIZE = 4u"), std::string::npos);
  EXPECT_NE(point.find("EE_POINT_LIGHTS[EE_BASIC_CONSTANTS.camera_index]"), std::string::npos);
  EXPECT_NE(point.find("EE_BASIC_CONSTANTS.light_split_index"), std::string::npos);
  EXPECT_NE(spot.find("EE_SPOT_LIGHTS[EE_BASIC_CONSTANTS.camera_index].light_space_matrix"), std::string::npos);
  EXPECT_NE(render_layer.find("PointLightStrandsShadowMap.slang"), std::string::npos);
  EXPECT_NE(render_layer.find("SpotLightStrandsShadowMap.slang"), std::string::npos);
  EXPECT_EQ(render_layer.find("PointLightShadowMapStrands"), std::string::npos);
  EXPECT_EQ(render_layer.find("SpotLightShadowMapStrands"), std::string::npos);
  EXPECT_NE(render_layer.find("strand_meshlet_descriptor_sets_[current_frame_index]"), std::string::npos);
  EXPECT_NE(render_layer.find("use_mesh_shader ? strands_point_light_shadow_pipeline : nullptr"), std::string::npos);
  EXPECT_NE(render_layer.find("use_mesh_shader ? strands_spot_light_shadow_pipeline : nullptr"), std::string::npos);
  EXPECT_NE(render_layer.find("pipeline->states.cull_mode = render_instance->cull_mode"), std::string::npos);
  EXPECT_NE(render_layer.find("render_strands_shadow_collection(RenderPassDrawBucket::PointLightShadow"),
            std::string::npos);
  EXPECT_NE(render_layer.find("render_strands_shadow_collection(RenderPassDrawBucket::SpotLightShadow"),
            std::string::npos);
  EXPECT_NE(render_layer.find("Platform::CountRenderPassDraw(bucket, RenderDrawCallKind::Direct"), std::string::npos);
  EXPECT_NE(scene.find("ConfigureStrandPunctualShadowValidation"), std::string::npos);
  EXPECT_NE(scene.find("const std::array point_directions"), std::string::npos);
  EXPECT_NE(editor.find("--preview-strand-punctual-fixture"), std::string::npos);
}
