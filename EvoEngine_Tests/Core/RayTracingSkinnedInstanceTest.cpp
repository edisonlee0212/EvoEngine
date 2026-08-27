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

TEST(RayTracingSkinned, MorphTargetsAreAppliedBeforeSkinning) {
  evo_engine::SkinnedVertex base{};
  base.position = glm::vec3(1.0f, 2.0f, 3.0f);
  base.normal = glm::vec3(0.0f, 1.0f, 0.0f);
  base.tangent = glm::vec3(1.0f, 0.0f, 0.0f);
  base.bond_id = glm::ivec4(0, -1, -1, -1);
  base.weight = glm::vec4(1.0f, 0.0f, 0.0f, 0.0f);
  base.bond_id2 = glm::ivec4(-1);
  evo_engine::MorphTarget target;
  target.position_deltas = {glm::vec3(2.0f, 0.0f, 0.0f)};
  const auto morphed = evo_engine::BuildMorphedVertices(std::vector{base}, std::vector{target}, {0.25f}, {0.75f});

  glm::mat4 bone(1.0f);
  bone[3] = glm::vec4(0.0f, 4.0f, 0.0f, 1.0f);
  const auto skinned = evo_engine::BuildSkinnedRayTracingVertices(morphed, {bone});
  ASSERT_EQ(skinned.size(), 1);
  EXPECT_EQ(skinned[0].position, glm::vec3(2.0f, 6.0f, 3.0f));
}

TEST(RayTracingSkinned, PackedVertexRemapPreservesReorderingAndDuplication) {
  std::vector<evo_engine::SkinnedVertex> source_vertices(3);
  for (uint32_t index = 0; index < source_vertices.size(); index++) {
    auto& vertex = source_vertices[index];
    vertex.position = glm::vec3(static_cast<float>(index), static_cast<float>(index + 1), 0.0f);
    vertex.normal = glm::vec3(0.0f, 1.0f, 0.0f);
    vertex.tangent = glm::vec3(1.0f, 0.0f, 0.0f);
    vertex.color = glm::vec4(static_cast<float>(index), 0.5f, 0.25f, 1.0f);
    vertex.tex_coord = glm::vec2(static_cast<float>(index) * 0.1f, 0.75f);
  }
  const std::vector<uint32_t> remap = {2, 0, 2, 1};
  const auto packed = evo_engine::BuildSkinnedRayTracingVertices(source_vertices, {}, remap);
  ASSERT_EQ(packed.size(), remap.size());
  for (size_t index = 0; index < remap.size(); index++) {
    const auto expected = evo_engine::BuildSkinnedRayTracingVertex(source_vertices[remap[index]], {});
    EXPECT_EQ(packed[index].position, expected.position);
    EXPECT_EQ(packed[index].normal, expected.normal);
    EXPECT_EQ(packed[index].tangent, expected.tangent);
    EXPECT_EQ(packed[index].color, expected.color);
    EXPECT_EQ(packed[index].tex_coord, expected.tex_coord);
  }
  EXPECT_THROW(evo_engine::BuildSkinnedRayTracingVertices(source_vertices, {}, {3}), std::out_of_range);
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

  EXPECT_NE(source.find("if (Platform::RayAccelerationStructureEnabled())"), std::string::npos);
  EXPECT_NE(source.find("BuildSkinnedRayTracingVertices(skinned_vertices_, {})"), std::string::npos);
  EXPECT_NE(source.find("GeometryStorage::AllocateMesh(GetHandle(), vertices, triangles, ray_tracing_meshlet_range_"),
            std::string::npos);
  EXPECT_NE(source.find("ray_tracing_triangle_range_)"), std::string::npos);
  EXPECT_NE(source.find("BottomLevelAccelerationStructure::CreateStatic(ray_tracing_meshlet_range_,"),
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
  EXPECT_NE(renderer_header.find("ray_tracing_packed_source_vertex_indices_"), std::string::npos);
  EXPECT_NE(renderer_header.find("ray_tracing_payload_retry_required_"), std::string::npos);
  EXPECT_NE(renderer_header.find("ray_tracing_mesh_handle_"), std::string::npos);
  EXPECT_NE(renderer_header.find("ray_tracing_geometry_version_"), std::string::npos);
  EXPECT_NE(renderer_header.find("ray_tracing_morph_weights_"), std::string::npos);
  EXPECT_NE(renderer_header.find("morph_weights_version_"), std::string::npos);
  EXPECT_NE(renderer_header.find("void UpdateRayTracingGeometry()"), std::string::npos);

  const auto update_geometry = renderer_source.find("void SkinnedMeshRenderer::UpdateRayTracingGeometry");
  const auto update_geometry_end = renderer_source.find("void SkinnedMeshRenderer::OnCreate", update_geometry);
  ASSERT_NE(update_geometry, std::string::npos);
  ASSERT_NE(update_geometry_end, std::string::npos);
  const auto update_geometry_source = renderer_source.substr(update_geometry, update_geometry_end - update_geometry);
  EXPECT_NE(update_geometry_source.find("FrameSubmissionState::Status::Pending"), std::string::npos);
  EXPECT_NE(update_geometry_source.find("FrameSubmissionState::Status::Submitted"), std::string::npos);
  EXPECT_NE(update_geometry_source.find("ray_tracing_payload_retry_required_ = true"), std::string::npos);
  EXPECT_NE(update_geometry_source.find("mesh->BuildMorphedVertices(morph_weights)"), std::string::npos);
  EXPECT_NE(update_geometry_source.find("ray_tracing_mesh_handle_ != mesh_handle"), std::string::npos);
  EXPECT_NE(update_geometry_source.find("ray_tracing_mesh_handle_ = mesh_handle"), std::string::npos);
  EXPECT_NE(update_geometry_source.find("if (topology_changed)"), std::string::npos);
  EXPECT_NE(update_geometry_source.find("&ray_tracing_packed_source_vertex_indices_"), std::string::npos);
  EXPECT_NE(update_geometry_source.find("BottomLevelAccelerationStructure>(vertices, triangles, true)"),
            std::string::npos);
  const auto steady_update = update_geometry_source.find("const auto packed_vertices");
  ASSERT_NE(steady_update, std::string::npos);
  const auto steady_update_source = update_geometry_source.substr(steady_update);
  EXPECT_NE(steady_update_source.find("GeometryStorage::UpdateMeshVertices"), std::string::npos);
  EXPECT_NE(steady_update_source.find("ray_tracing_blas_->UpdateVertices(packed_vertices)"), std::string::npos);
  EXPECT_EQ(steady_update_source.find("GeometryStorage::FreeMesh"), std::string::npos);
  EXPECT_EQ(steady_update_source.find("make_shared<BottomLevelAccelerationStructure>"), std::string::npos);

  const auto prepare_scene = render_layer_source.find("void RenderLayer::PrepareSceneForRendering");
  const auto apply_animators = render_layer_source.find("ApplyAnimators();", prepare_scene);
  const auto wait_uploads = render_layer_source.find("GeometryStorage::WaitForPendingUploads()", prepare_scene);
  const auto update_tlas = render_layer_source.find("UpdateTopLevelAccelerationStructure()", prepare_scene);
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

  const auto apply_animators_end =
      render_layer_source.find("void RenderLayer::PreparePointAndSpotLightShadowMap", apply_animators_def);
  const auto apply_animators_source =
      render_layer_source.substr(apply_animators_def, apply_animators_end - apply_animators_def);
  const auto first_bone_update = apply_animators_source.find("UpdateBoneMatrices()");
  ASSERT_NE(first_bone_update, std::string::npos);
  EXPECT_EQ(apply_animators_source.find("UpdateBoneMatrices()", first_bone_update + 1), std::string::npos);
  const auto serial_loop = apply_animators_source.find("for (const auto& i : *owners)");
  ASSERT_NE(serial_loop, std::string::npos);
  const auto serial_loop_source = apply_animators_source.substr(serial_loop);
  EXPECT_NE(serial_loop_source.find("continue;"), std::string::npos);
  EXPECT_EQ(serial_loop_source.find("return;"), std::string::npos);
}

TEST(RayTracingSkinned, StaticMorphRendererReusesPersistentRayTracingGeometry) {
  const auto header = ReadTextFile(SourcePath("EvoEngine_SDK/include/Rendering/Renderer/MeshRenderer.hpp"));
  const auto renderer = ReadTextFile(SourcePath("EvoEngine_SDK/src/MeshRenderer.cpp"));
  const auto storage = ReadTextFile(SourcePath("EvoEngine_SDK/src/RenderInstanceStorage.cpp"));
  ASSERT_FALSE(header.empty());
  ASSERT_FALSE(renderer.empty());
  ASSERT_FALSE(storage.empty());

  EXPECT_NE(header.find("void SetMorphWeights"), std::string::npos);
  EXPECT_NE(header.find("ray_tracing_packed_source_vertex_indices_"), std::string::npos);
  EXPECT_NE(header.find("pending_ray_tracing_submission_state_"), std::string::npos);
  EXPECT_NE(header.find("ray_tracing_mesh_handle_"), std::string::npos);
  const auto update = renderer.find("void MeshRenderer::UpdateRayTracingGeometry");
  ASSERT_NE(update, std::string::npos);
  const auto update_source = renderer.substr(update);
  EXPECT_NE(update_source.find("FrameSubmissionState::Status::Submitted"), std::string::npos);
  EXPECT_NE(update_source.find("ray_tracing_payload_retry_required_ = true"), std::string::npos);
  EXPECT_NE(update_source.find("mesh_asset->BuildMorphedVertices(weights)"), std::string::npos);
  EXPECT_NE(update_source.find("ray_tracing_mesh_handle_ != mesh_handle"), std::string::npos);
  EXPECT_NE(update_source.find("ray_tracing_mesh_handle_ = mesh_handle"), std::string::npos);
  EXPECT_NE(update_source.find("GeometryStorage::UpdateMeshVertices"), std::string::npos);
  EXPECT_NE(update_source.find("ray_tracing_blas_->UpdateVertices(packed_vertices)"), std::string::npos);
  EXPECT_NE(renderer.find("void MeshRenderer::PostCloneAction"), std::string::npos);
  EXPECT_NE(renderer.find("ray_tracing_blas_.reset()"), std::string::npos);
  EXPECT_NE(ReadTextFile(SourcePath("EvoEngine_SDK/src/SkinnedMeshRenderer.cpp"))
                .find("cloned_bone_matrices->value = bone_matrices->value"),
            std::string::npos);
  EXPECT_NE(storage.find("ray_tracing_triangle_range && ray_tracing_triangle_range->prev_frame_index_count"),
            std::string::npos);
  EXPECT_NE(storage.find("render_instance->ray_tracing_blas ? render_instance->ray_tracing_blas"), std::string::npos);
}

TEST(RayTracingSkinned, DynamicBlasUsesPersistentMainQueueUpdates) {
  const auto header = ReadTextFile(SourcePath("EvoEngine_SDK/include/Rendering/Platform/GraphicsResources.hpp"));
  const auto source = ReadTextFile(SourcePath("EvoEngine_SDK/src/GraphicsResources.cpp"));
  const auto render_layer = ReadTextFile(SourcePath("EvoEngine_SDK/src/RenderLayer.cpp"));
  const auto render_storage = ReadTextFile(SourcePath("EvoEngine_SDK/src/RenderInstanceStorage.cpp"));
  ASSERT_FALSE(header.empty());
  ASSERT_FALSE(source.empty());
  ASSERT_FALSE(render_layer.empty());
  ASSERT_FALSE(render_storage.empty());

  EXPECT_NE(header.find("std::shared_ptr<FrameSubmissionState> UpdateVertices"), std::string::npos);
  EXPECT_NE(header.find("pending_content_version_"), std::string::npos);
  EXPECT_NE(source.find("std::max(build_sizes.buildScratchSize, build_sizes.updateScratchSize)"), std::string::npos);
  EXPECT_NE(source.find("VK_BUILD_ACCELERATION_STRUCTURE_ALLOW_UPDATE_BIT_KHR"), std::string::npos);
  EXPECT_NE(source.find("build_info.mode = VK_BUILD_ACCELERATION_STRUCTURE_MODE_UPDATE_KHR"), std::string::npos);
  EXPECT_NE(source.find("build_info.srcAccelerationStructure = vk_acceleration_structure_khr_"), std::string::npos);
  EXPECT_NE(source.find("build_info.dstAccelerationStructure = vk_acceleration_structure_khr_"), std::string::npos);
  EXPECT_NE(source.find("Platform::RecordCommandsMainQueue"), std::string::npos);
  EXPECT_NE(source.find("{\"BlasUpdate\", \"BLAS Update\", \"Ray Tracing\""), std::string::npos);
  EXPECT_NE(source.find("pending_content_version_ = content_version_ + 1"), std::string::npos);
  EXPECT_NE(source.find("pending_submission_state_ = Platform::TrackCurrentFrameSubmission()"), std::string::npos);
  EXPECT_NE(source.find("previous_blas_content_versions != current_blas_content_versions"), std::string::npos);
  EXPECT_NE(render_storage.find("bone_matrices_snapshot = skinned_mesh_renderer->bone_matrices->value"),
            std::string::npos);
}

TEST(RayTracingSkinned, TopLevelAccelerationStructureRegistersSkinnedCollections) {
  const auto source = ReadTextFile(SourcePath("EvoEngine_SDK/src/RenderInstanceStorage.cpp"));
  ASSERT_FALSE(source.empty());

  const auto register_static = source.find("const auto register_mesh_tlas_input");
  const auto register_skinned = source.find("const auto register_skinned_tlas_input");
  const auto register_instanced = source.find("const auto register_strands_tlas_input");
  ASSERT_NE(register_static, std::string::npos);
  ASSERT_NE(register_skinned, std::string::npos);
  ASSERT_NE(register_instanced, std::string::npos);
  EXPECT_LT(register_static, register_skinned);
  EXPECT_LT(register_skinned, register_instanced);

  EXPECT_NE(source.find("render_instance->ray_tracing_blas ? render_instance->ray_tracing_blas"), std::string::npos);
  EXPECT_NE(source.find("render_instance->skinned_mesh->blas_"), std::string::npos);
  EXPECT_NE(source.find("top_level_acceleration_structure_inputs_.push_back"), std::string::npos);
  EXPECT_NE(source.find("render_instance->model.value"), std::string::npos);
  EXPECT_NE(source.find("deferred_skinned_render_instances->ForEachSkinnedMeshRenderInstance"), std::string::npos);
  EXPECT_NE(source.find("forward_skinned_render_instances->ForEachSkinnedMeshRenderInstance"), std::string::npos);
  EXPECT_NE(source.find("transparent_skinned_render_instances->ForEachSkinnedMeshRenderInstance"), std::string::npos);
}

TEST(RayTracingSkinned, RenderInstanceStorageBuildsTlasForSkinnedOnlyScenes) {
  const auto source = ReadTextFile(SourcePath("EvoEngine_SDK/src/RenderInstanceStorage.cpp"));
  ASSERT_FALSE(source.empty());

  const auto update_tlas = source.find("void RenderInstanceStorage::UpdateTopLevelAccelerationStructure");
  ASSERT_NE(update_tlas, std::string::npos);
  const auto create_tlas = source.find("std::make_shared<TopLevelAccelerationStructure>()", update_tlas);
  ASSERT_NE(create_tlas, std::string::npos);
  EXPECT_NE(source.find("mesh_top_level_acceleration_structure->Update(*this)", create_tlas), std::string::npos);
}
