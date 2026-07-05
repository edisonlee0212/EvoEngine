#include "EvoEngine_SDK_PCH.hpp"
#include "SkinnedMesh.hpp"

#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <iterator>
#include <string>

namespace {
std::string ReadTextFile(const std::filesystem::path& path) {
  std::ifstream file(path);
  EXPECT_TRUE(file.good()) << path.string();
  return {std::istreambuf_iterator<char>(file), std::istreambuf_iterator<char>()};
}

std::filesystem::path SourcePath(const std::filesystem::path& relative_path) {
  return std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / relative_path;
}
}  // namespace

TEST(RayTracingSkinned, RayTracingVertexBuilderAppliesAnimatedPose) {
  evo_engine::SkinnedVertex skinned_vertex{};
  skinned_vertex.position = glm::vec3(1.0f, 2.0f, 3.0f);
  skinned_vertex.normal = glm::vec3(0.0f, 1.0f, 0.0f);
  skinned_vertex.tangent = glm::vec3(1.0f, 0.0f, 0.0f);
  skinned_vertex.color = glm::vec4(0.25f, 0.5f, 0.75f, 1.0f);
  skinned_vertex.tex_coord = glm::vec2(0.2f, 0.8f);
  skinned_vertex.bond_id = glm::ivec4(0, -1, -1, -1);
  skinned_vertex.weight = glm::vec4(1.0f, 0.0f, 0.0f, 0.0f);
  skinned_vertex.bond_id2 = glm::ivec4(-1);
  skinned_vertex.weight2 = glm::vec4(0.0f);

  glm::mat4 translated_bone(1.0f);
  translated_bone[3] = glm::vec4(2.0f, 0.0f, -1.0f, 1.0f);
  const auto vertex = evo_engine::BuildSkinnedRayTracingVertex(skinned_vertex, {translated_bone});

  EXPECT_NEAR(vertex.position.x, 3.0f, 1e-5f);
  EXPECT_NEAR(vertex.position.y, 2.0f, 1e-5f);
  EXPECT_NEAR(vertex.position.z, 2.0f, 1e-5f);
  EXPECT_NEAR(vertex.normal.x, 0.0f, 1e-5f);
  EXPECT_NEAR(vertex.normal.y, 1.0f, 1e-5f);
  EXPECT_NEAR(vertex.normal.z, 0.0f, 1e-5f);
  EXPECT_NEAR(vertex.tangent.x, 1.0f, 1e-5f);
  EXPECT_NEAR(vertex.tangent.y, 0.0f, 1e-5f);
  EXPECT_NEAR(vertex.tangent.z, 0.0f, 1e-5f);
  EXPECT_EQ(vertex.color, skinned_vertex.color);
  EXPECT_EQ(vertex.tex_coord, skinned_vertex.tex_coord);
}

TEST(RayTracingSkinned, SkinnedMeshBuildsBindPoseFallbackWithSharedConversion) {
  const auto header = ReadTextFile(SourcePath("EvoEngine_SDK/include/Rendering/Geometry/SkinnedMesh.hpp"));
  const auto source = ReadTextFile(SourcePath("EvoEngine_SDK/src/SkinnedMesh.cpp"));
  ASSERT_FALSE(header.empty());
  ASSERT_FALSE(source.empty());

  EXPECT_NE(header.find("friend class TopLevelAccelerationStructure"), std::string::npos);
  EXPECT_NE(header.find("std::shared_ptr<BottomLevelAccelerationStructure>"), std::string::npos);
  EXPECT_NE(header.find("blas_"), std::string::npos);
  EXPECT_NE(header.find("ray_tracing_triangle_range_"), std::string::npos);
  EXPECT_NE(header.find("ray_tracing_meshlet_range_"), std::string::npos);
  EXPECT_NE(header.find("BuildSkinnedRayTracingVertex"), std::string::npos);
  EXPECT_NE(header.find("BuildSkinnedRayTracingVertices"), std::string::npos);

  EXPECT_NE(source.find("if (Platform::RayTracingEnabled())"), std::string::npos);
  EXPECT_NE(source.find("BuildSkinnedRayTracingVertices(skinned_vertices_, {})"), std::string::npos);
  EXPECT_NE(source.find("GeometryStorage::AllocateMesh(GetHandle(), vertices, triangles, ray_tracing_meshlet_range_"),
            std::string::npos);
  EXPECT_NE(source.find("ray_tracing_triangle_range_)"), std::string::npos);
  EXPECT_NE(source.find("blas_ = std::make_shared<BottomLevelAccelerationStructure>(vertices, triangles)"),
            std::string::npos);
}

TEST(RayTracingSkinned, SkinnedInstanceUsesStaticPayloadRangeForRayTracingShaders) {
  const auto source = ReadTextFile(SourcePath("EvoEngine_SDK/src/RenderInstanceStorage.cpp"));
  ASSERT_FALSE(source.empty());

  const auto apply = source.find("void RenderInstanceStorage::SkinnedMeshRenderInstance::Apply");
  ASSERT_NE(apply, std::string::npos);
  const auto animated_range = source.find("ray_tracing_triangle_range->prev_frame_offset", apply);
  const auto bind_pose_range = source.find("skinned_mesh->ray_tracing_triangle_range_", apply);
  const auto skinned_range = source.find("skinned_mesh->skinned_triangle_range_->prev_frame_offset", apply);
  ASSERT_NE(animated_range, std::string::npos);
  ASSERT_NE(bind_pose_range, std::string::npos);
  ASSERT_NE(skinned_range, std::string::npos);
  EXPECT_LT(animated_range, bind_pose_range);
  EXPECT_LT(bind_pose_range, skinned_range);
  EXPECT_NE(source.find("instance_info_block.triangle_offset = ray_tracing_triangle_range->prev_frame_offset"),
            std::string::npos);
  EXPECT_NE(source.find("instance_info_block.meshlet_index_offset = "
                        "skinned_mesh->skinned_meshlet_range_->prev_frame_offset"),
            std::string::npos);
}

TEST(RayTracingSkinned, SkinnedRendererBuildsAnimatedPayloadBeforeTlas) {
  const auto renderer_header =
      ReadTextFile(SourcePath("EvoEngine_SDK/include/Rendering/Renderer/SkinnedMeshRenderer.hpp"));
  const auto renderer_source = ReadTextFile(SourcePath("EvoEngine_SDK/src/SkinnedMeshRenderer.cpp"));
  const auto render_layer_source = ReadTextFile(SourcePath("EvoEngine_SDK/src/RenderLayer.cpp"));
  ASSERT_FALSE(renderer_header.empty());
  ASSERT_FALSE(renderer_source.empty());
  ASSERT_FALSE(render_layer_source.empty());

  EXPECT_NE(renderer_header.find("ray_tracing_blas_"), std::string::npos);
  EXPECT_NE(renderer_header.find("ray_tracing_bone_matrices_"), std::string::npos);
  EXPECT_NE(renderer_header.find("ray_tracing_geometry_version_"), std::string::npos);
  EXPECT_NE(renderer_header.find("void UpdateRayTracingGeometry()"), std::string::npos);

  EXPECT_NE(renderer_source.find("BoneMatricesMatch(ray_tracing_bone_matrices_, bone_matrices->value)"),
            std::string::npos);
  EXPECT_NE(renderer_source.find("GeometryStorage::FreeMesh(GetHandle())"), std::string::npos);
  EXPECT_NE(renderer_source.find("BuildSkinnedRayTracingVertices(mesh->skinned_vertices_, bone_matrices->value)"),
            std::string::npos);
  EXPECT_NE(renderer_source.find("ray_tracing_blas_ = std::make_shared<BottomLevelAccelerationStructure>"),
            std::string::npos);

  const auto prepare_scene = render_layer_source.find("void RenderLayer::PrepareSceneForRendering");
  const auto apply_animators = render_layer_source.find("ApplyAnimators();", prepare_scene);
  const auto wait_uploads = render_layer_source.find("GeometryStorage::WaitForPendingUploads()", prepare_scene);
  const auto update_tlas = render_layer_source.find("UpdateTopLevelAccelerationStructure(scene)", prepare_scene);
  ASSERT_NE(apply_animators, std::string::npos);
  ASSERT_NE(wait_uploads, std::string::npos);
  ASSERT_NE(update_tlas, std::string::npos);
  EXPECT_LT(apply_animators, wait_uploads);
  EXPECT_LT(wait_uploads, update_tlas);

  const auto apply_animators_def = render_layer_source.find("void RenderLayer::ApplyAnimators");
  ASSERT_NE(apply_animators_def, std::string::npos);
  const auto update_bones = render_layer_source.find("UpdateBoneMatrices()", apply_animators_def);
  const auto update_ray_geometry = render_layer_source.find("UpdateRayTracingGeometry()", apply_animators_def);
  const auto upload_bones = render_layer_source.find("bone_matrices->UploadData()", apply_animators_def);
  ASSERT_NE(update_bones, std::string::npos);
  ASSERT_NE(update_ray_geometry, std::string::npos);
  ASSERT_NE(upload_bones, std::string::npos);
  EXPECT_LT(update_bones, update_ray_geometry);
  EXPECT_LT(update_ray_geometry, upload_bones);
}

TEST(RayTracingSkinned, TopLevelAccelerationStructureRegistersSkinnedCollections) {
  const auto source = ReadTextFile(SourcePath("EvoEngine_SDK/src/GraphicsResources.cpp"));
  ASSERT_FALSE(source.empty());

  EXPECT_NE(source.find("#include \"SkinnedMesh.hpp\""), std::string::npos);
  const auto register_static = source.find("const auto register_mesh_render_instance");
  const auto register_skinned = source.find("const auto register_skinned_mesh_render_instance");
  const auto register_instanced = source.find("const auto register_instanced_mesh_render_instance");
  ASSERT_NE(register_static, std::string::npos);
  ASSERT_NE(register_skinned, std::string::npos);
  ASSERT_NE(register_instanced, std::string::npos);
  EXPECT_LT(register_static, register_skinned);
  EXPECT_LT(register_skinned, register_instanced);

  EXPECT_NE(source.find("std::dynamic_pointer_cast<RenderInstanceStorage::SkinnedMeshRenderInstance>"),
            std::string::npos);
  EXPECT_NE(source.find("skinned_render_instance->ray_tracing_blas ? skinned_render_instance->ray_tracing_blas"),
            std::string::npos);
  EXPECT_NE(source.find("skinned_render_instance->skinned_mesh->blas_"), std::string::npos);
  EXPECT_NE(source.find("blas->GetDeviceAddress()"), std::string::npos);
  EXPECT_NE(source.find("render_instance->model.value"), std::string::npos);
  EXPECT_NE(source.find("render_instance_storage.deferred_skinned_render_instances->ForEachRenderInstance"),
            std::string::npos);
  EXPECT_NE(source.find("render_instance_storage.forward_skinned_render_instances->ForEachRenderInstance"),
            std::string::npos);
  EXPECT_NE(source.find("render_instance_storage.transparent_skinned_render_instances->ForEachRenderInstance"),
            std::string::npos);
}

TEST(RayTracingSkinned, RenderInstanceStorageBuildsTlasForSkinnedOnlyScenes) {
  const auto source = ReadTextFile(SourcePath("EvoEngine_SDK/src/RenderInstanceStorage.cpp"));
  ASSERT_FALSE(source.empty());

  const auto update_tlas = source.find("void RenderInstanceStorage::UpdateTopLevelAccelerationStructure");
  ASSERT_NE(update_tlas, std::string::npos);
  const auto create_tlas = source.find("std::make_shared<TopLevelAccelerationStructure>(scene, *this)", update_tlas);
  ASSERT_NE(create_tlas, std::string::npos);

  const auto deferred_skinned = source.find("!deferred_skinned_render_instances->Empty()", update_tlas);
  const auto forward_skinned = source.find("!forward_skinned_render_instances->Empty()", update_tlas);
  const auto transparent_skinned = source.find("!transparent_skinned_render_instances->Empty()", update_tlas);
  ASSERT_NE(deferred_skinned, std::string::npos);
  ASSERT_NE(forward_skinned, std::string::npos);
  ASSERT_NE(transparent_skinned, std::string::npos);
  EXPECT_LT(deferred_skinned, create_tlas);
  EXPECT_LT(forward_skinned, create_tlas);
  EXPECT_LT(transparent_skinned, create_tlas);
}
