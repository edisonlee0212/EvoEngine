#include "EvoEngine_SDK_PCH.hpp"

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

std::filesystem::path ShaderPath(const std::filesystem::path& relative_path) {
  return std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / "EvoEngine_SDK" / "Internals" / "DefaultResources" /
         "Shaders" / relative_path;
}

std::filesystem::path SdkPath(const std::filesystem::path& relative_path) {
  return std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / "EvoEngine_SDK" / relative_path;
}

std::filesystem::path RepoPath(const std::filesystem::path& relative_path) {
  return std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / relative_path;
}

std::string ExtractSourceRange(const std::string& source, const std::string& begin, const std::string& end) {
  const auto begin_pos = source.find(begin);
  EXPECT_NE(begin_pos, std::string::npos) << begin;
  if (begin_pos == std::string::npos) {
    return {};
  }
  const auto end_pos = source.find(end, begin_pos);
  EXPECT_NE(end_pos, std::string::npos) << end;
  if (end_pos == std::string::npos) {
    return {};
  }
  return source.substr(begin_pos, end_pos - begin_pos);
}

size_t CountOccurrences(const std::string& source, const std::string& needle) {
  size_t count = 0;
  size_t pos = source.find(needle);
  while (pos != std::string::npos) {
    count++;
    pos = source.find(needle, pos + needle.size());
  }
  return count;
}
}  // namespace

TEST(GltfRasterMaterial, EvaluatorUsesCanonicalGltfTextureInfo) {
  const auto source = ReadTextFile(ShaderPath("Includes/GltfRasterMaterial.glsl"));
  ASSERT_FALSE(source.empty());

  EXPECT_NE(source.find("EE_GLTF_HAS_TEXTURE"), std::string::npos);
  EXPECT_NE(source.find("return uint(texture_info_slot) > 0u"), std::string::npos);
  EXPECT_NE(source.find("EE_GLTF_TEXTURE_INFOS[uint(texture_info_slot)]"), std::string::npos);
  EXPECT_NE(source.find("texture_info.uv_transform"), std::string::npos);
  EXPECT_NE(source.find("#ifndef EE_GLTF_RASTER_FIXED_MATERIAL_TEXTURES"), std::string::npos);
  EXPECT_NE(source.find("EE_GLTF_SAMPLE_BINDLESS_TEXTURE"), std::string::npos);
  EXPECT_NE(source.find("EE_TEXTURE_2DS[nonuniformEXT(texture_info.index)]"), std::string::npos);
  EXPECT_EQ(source.find(std::string("Material") + "Properties"), std::string::npos);
}

TEST(GltfRasterMaterial, EvaluatorCoversSpecGlossAndPbrTerms) {
  const auto source = ReadTextFile(ShaderPath("Includes/GltfRasterMaterial.glsl"));
  ASSERT_FALSE(source.empty());

  EXPECT_NE(source.find("EE_GLTF_PBR_MODEL_SPECULAR_GLOSSINESS"), std::string::npos);
  EXPECT_NE(source.find("pbr_specular_glossiness_texture"), std::string::npos);
  EXPECT_NE(source.find("EE_GLTF_CONVERT_SPEC_GLOSS_TO_METALLIC_ROUGHNESS"), std::string::npos);
  EXPECT_NE(source.find("pbr_metallic_roughness_texture"), std::string::npos);
  EXPECT_NE(source.find("surface.occlusion = 1.0 + surface.occlusion * (occlusion - 1.0)"), std::string::npos);
  EXPECT_NE(source.find("material.normal_texture_scale"), std::string::npos);
  EXPECT_NE(source.find("surface.emissive *="), std::string::npos);
  EXPECT_NE(source.find("EE_GLTF_ALPHA_MODE_MASK"), std::string::npos);
  EXPECT_NE(source.find("surface.transmission = material.transmission_factor"), std::string::npos);
  EXPECT_NE(source.find("surface.thickness = material.thickness_factor"), std::string::npos);
  EXPECT_NE(source.find("surface.diffuse_transmission_factor = material.diffuse_transmission_factor"),
            std::string::npos);
  EXPECT_NE(source.find("surface.diffuse_transmission_color = material.diffuse_transmission_color"), std::string::npos);
  EXPECT_NE(source.find("surface.multiscatter_color_factor = max(material.multiscatter_color_factor"),
            std::string::npos);
  EXPECT_NE(source.find("EE_GLTF_MULTI_TO_SINGLE_SCATTER_ALBEDO"), std::string::npos);
}

TEST(GltfRasterMaterial, PerFrameBindsCanonicalMaterialBuffers) {
  const auto per_frame = ReadTextFile(ShaderPath("Includes/PerFrame.glsl"));
  ASSERT_FALSE(per_frame.empty());

  EXPECT_NE(per_frame.find("EE_GLTF_MATERIALS_BLOCK_BINDING 11"), std::string::npos);
  EXPECT_NE(per_frame.find("EE_GLTF_TEXTURE_INFOS_BLOCK_BINDING 12"), std::string::npos);
  EXPECT_NE(per_frame.find("#ifndef EE_SKIP_PER_FRAME_BINDLESS_TEXTURES"), std::string::npos);
  EXPECT_NE(per_frame.find("#include \"Textures.glsl\""), std::string::npos);
  EXPECT_NE(per_frame.find("#include \"GltfMaterial.glsl\""), std::string::npos);

  const auto material = ReadTextFile(ShaderPath("Includes/GltfMaterial.glsl"));
  ASSERT_FALSE(material.empty());
  EXPECT_NE(material.find("#extension GL_EXT_scalar_block_layout : require"), std::string::npos);
  EXPECT_NE(material.find("layout(scalar, set = EE_GLTF_MATERIALS_BLOCK_SET"), std::string::npos);
  EXPECT_NE(material.find("layout(scalar, set = EE_GLTF_TEXTURE_INFOS_BLOCK_SET"), std::string::npos);
}

TEST(GltfRasterMaterial, RasterDescriptorMigrationContractIsDocumented) {
  const auto rendering_docs = ReadTextFile(RepoPath("docs/rendering.md"));
  ASSERT_FALSE(rendering_docs.empty());

  EXPECT_NE(rendering_docs.find("Raster material descriptors use descriptor set 3"), std::string::npos);
  EXPECT_NE(rendering_docs.find("set 1 remains available for"), std::string::npos);
  EXPECT_NE(rendering_docs.find("instanced/strand data"), std::string::npos);
  EXPECT_NE(rendering_docs.find("set 2 remains available for lighting or pass descriptors"), std::string::npos);
  EXPECT_NE(
      rendering_docs.find("Raster shaders must not use descriptor arrays such as `sampler2D[]` or `samplerCube[]`"),
      std::string::npos);
  EXPECT_NE(rendering_docs.find("Fixed-binding image arrays and atlases are allowed for raster"), std::string::npos);
  EXPECT_NE(rendering_docs.find("| 0 | Base color or diffuse | White. |"), std::string::npos);
  EXPECT_NE(rendering_docs.find("| 1 | Metallic-roughness or specular-glossiness | White. |"), std::string::npos);
  EXPECT_NE(rendering_docs.find("| 2 | Normal | Flat normal. |"), std::string::npos);
  EXPECT_NE(rendering_docs.find("| 3 | Emissive | Black. |"), std::string::npos);
  EXPECT_NE(rendering_docs.find("| 4 | Occlusion | White. |"), std::string::npos);
  EXPECT_NE(rendering_docs.find("renderer-owned runtime state keyed by material index"), std::string::npos);
  EXPECT_NE(rendering_docs.find("does not deduplicate descriptor sets across material indices"), std::string::npos);
  EXPECT_NE(rendering_docs.find("Each descriptor slot uses the texture's existing combined image sampler"),
            std::string::npos);
  EXPECT_NE(rendering_docs.find("bypasses all-in-one indirect deferred draws"), std::string::npos);
  EXPECT_NE(rendering_docs.find("Material-batched indirect buffers are the planned path"), std::string::npos);
  EXPECT_NE(rendering_docs.find("raster material per-frame descriptor set that keeps the shared per-frame buffers"),
            std::string::npos);
  EXPECT_NE(rendering_docs.find("omits bindless"), std::string::npos);
  EXPECT_NE(rendering_docs.find("texture and cubemap array bindings"), std::string::npos);
  EXPECT_NE(rendering_docs.find("Built-in shadow alpha and transparent mesh pipelines"), std::string::npos);
  EXPECT_NE(rendering_docs.find("alpha-tested shadow indirect draws are temporarily routed through direct submission"),
            std::string::npos);
  EXPECT_NE(rendering_docs.find("Alpha-tested built-in shadow pipelines also use the raster material per-frame"),
            std::string::npos);
  EXPECT_NE(rendering_docs.find("Transparent mesh lighting still uses bindless BRDF"), std::string::npos);
  EXPECT_NE(rendering_docs.find("lookups until the raster global/pass texture migration"), std::string::npos);
  EXPECT_NE(rendering_docs.find("Package or external forward callbacks"), std::string::npos);
  EXPECT_NE(rendering_docs.find("Bindless texture arrays are reserved for ray tracing and ray query paths"),
            std::string::npos);
  EXPECT_NE(rendering_docs.find("Raster-only or lower-end device mode must avoid"), std::string::npos);
  EXPECT_NE(rendering_docs.find("bindless descriptor layouts"), std::string::npos);
  EXPECT_NE(rendering_docs.find("tracing compute texture users also migrate to fixed descriptor sets"),
            std::string::npos);
}

TEST(GltfRasterMaterial, RasterMaterialDescriptorCompatibilityResourcesArePresent) {
  const auto render_layer_header = ReadTextFile(SdkPath("include/Layers/RenderLayer.hpp"));
  const auto render_layer = ReadTextFile(SdkPath("src/RenderLayer.cpp"));
  const auto render_instance_header =
      ReadTextFile(SdkPath("include/Rendering/RenderInstances/RenderInstanceStorage.hpp"));
  const auto render_instance = ReadTextFile(SdkPath("src/RenderInstanceStorage.cpp"));
  const auto texture_storage_header = ReadTextFile(SdkPath("include/Rendering/Texture/TextureStorage.hpp"));
  const auto texture_storage = ReadTextFile(SdkPath("src/TextureStorage.cpp"));
  ASSERT_FALSE(render_layer_header.empty());
  ASSERT_FALSE(render_layer.empty());
  ASSERT_FALSE(render_instance_header.empty());
  ASSERT_FALSE(render_instance.empty());
  ASSERT_FALSE(texture_storage_header.empty());
  ASSERT_FALSE(texture_storage.empty());

  EXPECT_NE(render_instance_header.find("kRasterMaterialTextureSlotCount = 5"), std::string::npos);
  EXPECT_NE(render_layer_header.find("GetRasterMaterialDescriptorSetLayout"), std::string::npos);
  EXPECT_NE(render_layer_header.find("raster_material_per_frame_layout_"), std::string::npos);
  EXPECT_NE(render_layer_header.find("raster_material_layout_"), std::string::npos);
  EXPECT_NE(render_layer_header.find("raster_material_per_frame_descriptor_sets_"), std::string::npos);
  EXPECT_NE(render_layer_header.find("raster_material_white_fallback_texture_"), std::string::npos);
  EXPECT_NE(render_layer_header.find("raster_material_black_fallback_texture_"), std::string::npos);
  EXPECT_NE(render_layer_header.find("raster_material_flat_normal_fallback_texture_"), std::string::npos);
  EXPECT_NE(render_layer.find("raster_material_layout_->PushDescriptorBinding"), std::string::npos);
  EXPECT_NE(render_layer.find("PushPerFrameSceneDescriptorBindings(raster_material_per_frame_layout_)"),
            std::string::npos);
  EXPECT_NE(render_layer.find("PushPerFrameMaterialBufferDescriptorBindings(raster_material_per_frame_layout_)"),
            std::string::npos);
  EXPECT_NE(render_layer.find("PushPerFrameBindlessTextureDescriptorBindings(per_frame_layout_"), std::string::npos);
  EXPECT_NE(render_layer.find("std::make_shared<DescriptorSet>(raster_material_per_frame_layout_)"), std::string::npos);
  EXPECT_NE(render_layer.find("VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER"), std::string::npos);
  EXPECT_NE(render_layer.find("VK_SHADER_STAGE_FRAGMENT_BIT"), std::string::npos);
  EXPECT_NE(render_layer.find("RefreshRasterMaterialDescriptorSets"), std::string::npos);
  EXPECT_NE(render_layer.find("glm::vec4(1.0f)"), std::string::npos);
  EXPECT_NE(render_layer.find("glm::vec4(0.0f, 0.0f, 0.0f, 1.0f)"), std::string::npos);
  EXPECT_NE(render_layer.find("glm::vec4(0.5f, 0.5f, 1.0f, 1.0f)"), std::string::npos);

  EXPECT_NE(render_instance_header.find("raster_material_descriptor_sets"), std::string::npos);
  EXPECT_NE(render_instance_header.find("RefreshRasterMaterialDescriptorSets"), std::string::npos);
  EXPECT_NE(render_instance.find("TextureStorage::GetVersion()"), std::string::npos);
  EXPECT_NE(render_instance.find("raster_material_descriptor_sets.resize(shade_materials.size())"), std::string::npos);
  EXPECT_NE(render_instance.find("std::make_shared<DescriptorSet>(raster_material_layout)"), std::string::npos);
  EXPECT_NE(render_instance.find("GltfPbrModel::SpecularGlossiness"), std::string::npos);
  for (uint32_t binding = 0; binding < 5; binding++) {
    EXPECT_NE(render_instance.find("UpdateImageDescriptorBinding(" + std::to_string(binding)), std::string::npos);
  }

  EXPECT_NE(texture_storage_header.find("TryGetTexture2DDescriptorImageInfo"), std::string::npos);
  EXPECT_NE(texture_storage.find("TextureStorage::TryGetTexture2DDescriptorImageInfo"), std::string::npos);
  EXPECT_NE(texture_storage.find("texture_storage.IsGpuUploadPending()"), std::string::npos);
}

TEST(GltfRasterMaterial, OpaqueDeferredPassBindsRasterMaterialDescriptors) {
  const auto render_layer = ReadTextFile(SdkPath("src/RenderLayer.cpp"));
  const auto pass_header = ReadTextFile(SdkPath("include/Rendering/RenderPasses/DeferredGeometryPass.hpp"));
  const auto pass = ReadTextFile(SdkPath("src/RenderPasses/DeferredGeometryPass.cpp"));
  const auto utilities = ReadTextFile(SdkPath("src/RenderPasses/RenderPassUtilities.cpp"));
  ASSERT_FALSE(render_layer.empty());
  ASSERT_FALSE(pass_header.empty());
  ASSERT_FALSE(pass.empty());
  ASSERT_FALSE(utilities.empty());

  EXPECT_NE(render_layer.find("std::string CreateRasterMaterialShaderDefines()"), std::string::npos);
  EXPECT_NE(render_layer.find("std::string CreateRasterMaterialNoBindlessShaderDefines()"), std::string::npos);
  EXPECT_NE(render_layer.find("#define EE_SKIP_PER_FRAME_BINDLESS_TEXTURES 1"), std::string::npos);
  EXPECT_NE(render_layer.find("#define EE_GLTF_RASTER_FIXED_MATERIAL_TEXTURES 1"), std::string::npos);

  const std::string normal_pipeline = ExtractSourceRange(render_layer, "if (!deferred_prepass_pipeline_normal)",
                                                         "deferred_prepass_pipeline_normal->depth_attachment_format");
  EXPECT_NE(normal_pipeline.find("CreateRasterNoBindlessTextureShaderDefines()"), std::string::npos);
  EXPECT_NE(normal_pipeline.find("CreateRasterMaterialNoBindlessShaderDefines()"), std::string::npos);
  EXPECT_EQ(CountOccurrences(normal_pipeline, "empty_descriptor_set_layout_"), 2);
  EXPECT_NE(normal_pipeline.find("raster_material_layout_"), std::string::npos);

  const std::string mesh_pipeline =
      ExtractSourceRange(render_layer,
                         "if (Platform::GetInstance().GetCapabilities().support_mesh_shader && "
                         "!deferred_prepass_pipeline_mesh)",
                         "deferred_prepass_pipeline_mesh->depth_attachment_format");
  EXPECT_NE(mesh_pipeline.find("CreateRasterNoBindlessTextureShaderDefines()"), std::string::npos);
  EXPECT_NE(mesh_pipeline.find("CreateRasterMaterialNoBindlessShaderDefines()"), std::string::npos);
  EXPECT_NE(mesh_pipeline.find("meshlet_layout_"), std::string::npos);
  EXPECT_EQ(CountOccurrences(mesh_pipeline, "empty_descriptor_set_layout_"), 1);
  EXPECT_NE(mesh_pipeline.find("raster_material_layout_"), std::string::npos);

  const std::string instanced_pipeline =
      ExtractSourceRange(render_layer, "if (!instanced_deferred_prepass_pipeline)",
                         "instanced_deferred_prepass_pipeline->depth_attachment_format");
  EXPECT_NE(instanced_pipeline.find("CreateRasterNoBindlessTextureShaderDefines()"), std::string::npos);
  EXPECT_NE(instanced_pipeline.find("CreateRasterMaterialNoBindlessShaderDefines()"), std::string::npos);
  EXPECT_NE(instanced_pipeline.find("particle_instanced_data_layout_"), std::string::npos);
  EXPECT_EQ(CountOccurrences(instanced_pipeline, "empty_descriptor_set_layout_"), 1);
  EXPECT_NE(instanced_pipeline.find("raster_material_layout_"), std::string::npos);

  const std::string skinned_pipeline = ExtractSourceRange(render_layer, "if (!skinned_deferred_prepass_pipeline)",
                                                          "skinned_deferred_prepass_pipeline->depth_attachment_format");
  EXPECT_NE(skinned_pipeline.find("CreateRasterNoBindlessTextureShaderDefines()"), std::string::npos);
  EXPECT_NE(skinned_pipeline.find("CreateRasterMaterialNoBindlessShaderDefines()"), std::string::npos);
  EXPECT_NE(skinned_pipeline.find("bone_matrices_layout_"), std::string::npos);
  EXPECT_EQ(CountOccurrences(skinned_pipeline, "empty_descriptor_set_layout_"), 1);
  EXPECT_NE(skinned_pipeline.find("raster_material_layout_"), std::string::npos);

  const std::string strands_pipeline =
      ExtractSourceRange(render_layer, "if (!strands_deferred_prepass_pipeline)",
                         "strands_deferred_prepass_pipeline->tessellation_patch_control_points");
  EXPECT_NE(strands_pipeline.find("CreateRasterNoBindlessTextureShaderDefines()"), std::string::npos);
  EXPECT_NE(strands_pipeline.find("CreateRasterMaterialNoBindlessShaderDefines()"), std::string::npos);
  EXPECT_NE(strands_pipeline.find("particle_instanced_data_layout_"), std::string::npos);
  EXPECT_EQ(CountOccurrences(strands_pipeline, "empty_descriptor_set_layout_"), 1);
  EXPECT_NE(strands_pipeline.find("raster_material_layout_"), std::string::npos);

  EXPECT_NE(render_layer.find("raster_material_per_frame_descriptor_sets_[current_frame_index]"), std::string::npos);
  EXPECT_NE(render_layer.find("enable_indirect_rendering, true, count_draw_calls, wire_frame"), std::string::npos);

  EXPECT_NE(pass_header.find("bind_raster_material_descriptor_sets"), std::string::npos);
  EXPECT_NE(pass.find("BindRasterMaterialDescriptorSet"), std::string::npos);
  EXPECT_NE(pass.find("parameters.enable_indirect_rendering && !parameters.bind_raster_material_descriptor_sets"),
            std::string::npos);
  EXPECT_EQ(CountOccurrences(pass, "BindRasterMaterialDescriptorSet(vk_command_buffer, parameters."), 4);
  EXPECT_NE(utilities.find("GetRasterMaterialDescriptorSet(static_cast<uint32_t>(material_index))"), std::string::npos);
  EXPECT_NE(utilities.find("BindDescriptorSet(vk_command_buffer, 3"), std::string::npos);
}

TEST(GltfRasterMaterial, ShadowAndTransparentPassesBindRasterMaterialDescriptors) {
  const auto render_layer = ReadTextFile(SdkPath("src/RenderLayer.cpp"));
  const auto directional_header =
      ReadTextFile(SdkPath("include/Rendering/RenderPasses/DirectionalLightShadowPass.hpp"));
  const auto directional = ReadTextFile(SdkPath("src/RenderPasses/DirectionalLightShadowPass.cpp"));
  const auto transparent = ReadTextFile(SdkPath("src/RenderPasses/TransparentGeometryPass.cpp"));
  const auto utilities_header = ReadTextFile(SdkPath("include/Rendering/RenderPasses/RenderPassUtilities.hpp"));
  ASSERT_FALSE(render_layer.empty());
  ASSERT_FALSE(directional_header.empty());
  ASSERT_FALSE(directional.empty());
  ASSERT_FALSE(transparent.empty());
  ASSERT_FALSE(utilities_header.empty());

  const char* alpha_shadow_pipelines[] = {
      "point_light_shadow_pipeline_normal",        "spot_light_shadow_pipeline_normal",
      "directional_light_shadow_pipeline_normal",  "instanced_point_light_shadow_pipeline",
      "instanced_spot_light_shadow_pipeline",      "instanced_directional_light_shadow_pipeline",
      "skinned_point_light_shadow_pipeline",       "skinned_spot_light_shadow_pipeline",
      "skinned_directional_light_shadow_pipeline",
  };
  for (const auto* pipeline : alpha_shadow_pipelines) {
    const std::string block =
        ExtractSourceRange(render_layer, std::string(pipeline) + "->fragment_shader = Shader::CreateTemporary",
                           std::string(pipeline) + "->depth_attachment_format");
    EXPECT_NE(block.find("CreateRasterMaterialNoBindlessShaderDefines()"), std::string::npos) << pipeline;
    EXPECT_NE(block.find("raster_material_per_frame_layout_"), std::string::npos) << pipeline;
    EXPECT_NE(block.find("raster_material_layout_"), std::string::npos) << pipeline;
  }

  const std::string shadow_mesh_helper =
      ExtractSourceRange(render_layer, "std::shared_ptr<GraphicsPipeline> CreateShadowMeshPipeline",
                         "std::shared_ptr<GraphicsPipeline> CreateGaussianSplatPipeline");
  EXPECT_NE(shadow_mesh_helper.find("CreateRasterNoBindlessTextureShaderDefines()"), std::string::npos);
  EXPECT_NE(shadow_mesh_helper.find("CreateRasterMaterialNoBindlessShaderDefines()"), std::string::npos);
  EXPECT_EQ(CountOccurrences(render_layer,
                             "{raster_material_per_frame_layout_, meshlet_layout_, empty_descriptor_set_layout_, "
                             "raster_material_layout_}"),
            3);

  const std::string transparent_pipeline = ExtractSourceRange(
      render_layer, "transparent_geometry_pipeline_normal->fragment_shader = Shader::CreateTemporary",
      "transparent_geometry_pipeline_normal->Initialize()");
  EXPECT_NE(transparent_pipeline.find("CreateRasterMaterialShaderDefines()"), std::string::npos);
  EXPECT_NE(transparent_pipeline.find("lighting_layout_"), std::string::npos);
  EXPECT_NE(transparent_pipeline.find("raster_material_layout_"), std::string::npos);
  EXPECT_NE(transparent.find("BindRasterMaterialDescriptorSet(vk_command_buffer, parameters.mesh_pipeline"),
            std::string::npos);
  EXPECT_NE(transparent.find("render_instance->material_index"), std::string::npos);

  EXPECT_NE(utilities_header.find("BindRasterMaterialDescriptorSet"), std::string::npos);
  EXPECT_NE(directional_header.find("bind_raster_material_descriptor_sets"), std::string::npos);
  EXPECT_NE(directional_header.find("raster_material_per_frame_descriptor_set"), std::string::npos);
  EXPECT_NE(
      directional.find("parameters.enable_indirect_rendering && !parameters.bind_raster_material_descriptor_sets"),
      std::string::npos);
  EXPECT_NE(directional.find("alpha_tested_pipeline && parameters.raster_material_per_frame_descriptor_set"),
            std::string::npos);
  EXPECT_EQ(CountOccurrences(directional, "alpha_tested && parameters.bind_raster_material_descriptor_sets"), 3);
  EXPECT_NE(render_layer.find("const bool bind_raster_material_descriptor_sets = true"), std::string::npos);
  EXPECT_NE(render_layer.find("enable_indirect_rendering && !bind_raster_material_descriptor_sets"), std::string::npos);
  EXPECT_NE(render_layer.find("const auto& per_frame_descriptor_set = alpha_tested_pipeline"), std::string::npos);
  EXPECT_NE(render_layer.find("raster_material_per_frame_descriptor_sets_[current_frame_index]"), std::string::npos);
  EXPECT_NE(render_layer.find("point_light_info_block.viewport, false, true"), std::string::npos);
  EXPECT_NE(render_layer.find("spot_light_info_block.viewport, false, true"), std::string::npos);
  EXPECT_NE(render_layer.find("BindRasterMaterialDescriptorSet(vk_command_buffer, target_pipeline, "
                              "current_render_instances"),
            std::string::npos);
  EXPECT_NE(render_layer.find("use_mesh_shader,\n               enable_indirect_rendering,\n               true,\n"
                              "               count_draw_calls"),
            std::string::npos);
}

TEST(GltfRasterMaterial, PreviewThumbnailsUseRenderLayerFixedRasterPath) {
  const auto docs = ReadTextFile(RepoPath("docs/rendering.md"));
  const auto thumbnail_provider = ReadTextFile(SdkPath("src/AssetThumbnailProvider.cpp"));
  const auto offscreen_preview = ReadTextFile(SdkPath("src/OffscreenPreviewRenderer.cpp"));
  const auto render_layer = ReadTextFile(SdkPath("src/RenderLayer.cpp"));
  ASSERT_FALSE(docs.empty());
  ASSERT_FALSE(thumbnail_provider.empty());
  ASSERT_FALSE(offscreen_preview.empty());
  ASSERT_FALSE(render_layer.empty());

  EXPECT_NE(docs.find("Material and mesh thumbnail rendering uses `AssetThumbnailProvider` and "
                      "`OffscreenPreviewRenderer`"),
            std::string::npos);
  EXPECT_NE(docs.find("do not own separate glTF raster"), std::string::npos);
  EXPECT_NE(docs.find("material pipelines"), std::string::npos);
  EXPECT_NE(docs.find("inherits the same fixed material descriptor layouts"), std::string::npos);

  EXPECT_NE(thumbnail_provider.find("RegisterAssetPreviewHandler<Material>"), std::string::npos);
  EXPECT_NE(thumbnail_provider.find("OffscreenPreviewRenderer::RenderMaterial(material, settings)"), std::string::npos);
  EXPECT_NE(thumbnail_provider.find("RegisterAssetPreviewHandler<Mesh>"), std::string::npos);
  EXPECT_NE(thumbnail_provider.find("OffscreenPreviewRenderer::RenderMesh(mesh, {}, settings)"), std::string::npos);
  EXPECT_NE(thumbnail_provider.find("Serialization::GenerateAssetThumbnail(asset, settings)"), std::string::npos);

  EXPECT_NE(offscreen_preview.find("RenderMeshWithMaterial(mesh, material"), std::string::npos);
  EXPECT_NE(offscreen_preview.find("scene->environment.volumetric_cloud_settings.enabled = false"), std::string::npos);
  EXPECT_NE(offscreen_preview.find("scene->environment.ddgi_settings.runtime.enabled = false"), std::string::npos);
  EXPECT_NE(offscreen_preview.find("UploadPreviewResources(mesh, material)"), std::string::npos);
  EXPECT_NE(offscreen_preview.find("camera->camera_render_mode = Camera::CameraRenderMode::Rasterization"),
            std::string::npos);
  EXPECT_NE(offscreen_preview.find("render_layer->RenderSceneToCameraImmediately(scene, camera_transform, camera)"),
            std::string::npos);
  EXPECT_EQ(offscreen_preview.find("GltfRasterMaterial.glsl"), std::string::npos);
  EXPECT_EQ(offscreen_preview.find("StandardDeferred.frag"), std::string::npos);
  EXPECT_EQ(offscreen_preview.find("StandardTransparent.frag"), std::string::npos);
  EXPECT_EQ(offscreen_preview.find("ShadowMapPassThrough.frag"), std::string::npos);
  EXPECT_EQ(offscreen_preview.find("Shader::CreateTemporary"), std::string::npos);

  const std::string immediate_render = ExtractSourceRange(
      render_layer, "void RenderLayer::RenderSceneToCameraImmediately", "void RenderLayer::RenderAll");
  EXPECT_NE(immediate_render.find("std::make_shared<RenderInstanceStorage>()"), std::string::npos);
  EXPECT_NE(immediate_render.find("PrepareSceneForRendering(scene, false, false, false, false)"), std::string::npos);
  EXPECT_NE(immediate_render.find("RenderToCamera(scene, camera_global_transform, camera, true)"), std::string::npos);
  EXPECT_NE(immediate_render.find("BindRenderInstanceStorage(current_frame_index, previous_render_instances)"),
            std::string::npos);
}

TEST(GltfRasterMaterial, FixedRasterBackendUsesIndividualTextureBindings) {
  const auto source = ReadTextFile(ShaderPath("Includes/GltfRasterMaterial.glsl"));
  ASSERT_FALSE(source.empty());

  EXPECT_NE(source.find("EE_GLTF_RASTER_FIXED_MATERIAL_TEXTURES"), std::string::npos);
  EXPECT_NE(source.find("EE_GLTF_RASTER_MATERIAL_SET 3"), std::string::npos);
  EXPECT_NE(source.find("EE_GLTF_RASTER_BASE_COLOR_TEXTURE_BINDING 0"), std::string::npos);
  EXPECT_NE(source.find("EE_GLTF_RASTER_METALLIC_ROUGHNESS_TEXTURE_BINDING 1"), std::string::npos);
  EXPECT_NE(source.find("EE_GLTF_RASTER_NORMAL_TEXTURE_BINDING 2"), std::string::npos);
  EXPECT_NE(source.find("EE_GLTF_RASTER_EMISSIVE_TEXTURE_BINDING 3"), std::string::npos);
  EXPECT_NE(source.find("EE_GLTF_RASTER_OCCLUSION_TEXTURE_BINDING 4"), std::string::npos);

  const std::string fixed_declarations =
      ExtractSourceRange(source, "#ifdef EE_GLTF_RASTER_FIXED_MATERIAL_TEXTURES", "struct GltfRasterMaterial");
  EXPECT_NE(fixed_declarations.find("uniform sampler2D"), std::string::npos);
  EXPECT_EQ(fixed_declarations.find("sampler2D[]"), std::string::npos);
  EXPECT_EQ(fixed_declarations.find("EE_TEXTURE_2DS"), std::string::npos);
  EXPECT_EQ(fixed_declarations.find("nonuniformEXT"), std::string::npos);

  const std::string fixed_sampling =
      ExtractSourceRange(source, "vec4 EE_GLTF_SAMPLE_FIXED_RASTER_TEXTURE", "vec4 EE_GLTF_SAMPLE_TEXTURE_SLOT");
  EXPECT_NE(fixed_sampling.find("texture(EE_GLTF_RASTER_BASE_COLOR_TEXTURE"), std::string::npos);
  EXPECT_NE(fixed_sampling.find("texture(EE_GLTF_RASTER_METALLIC_ROUGHNESS_TEXTURE"), std::string::npos);
  EXPECT_NE(fixed_sampling.find("texture(EE_GLTF_RASTER_NORMAL_TEXTURE"), std::string::npos);
  EXPECT_NE(fixed_sampling.find("texture(EE_GLTF_RASTER_EMISSIVE_TEXTURE"), std::string::npos);
  EXPECT_NE(fixed_sampling.find("texture(EE_GLTF_RASTER_OCCLUSION_TEXTURE"), std::string::npos);
  EXPECT_EQ(fixed_sampling.find("EE_TEXTURE_2DS"), std::string::npos);
  EXPECT_EQ(fixed_sampling.find("nonuniformEXT"), std::string::npos);
  EXPECT_EQ(fixed_sampling.find("sampler2D[]"), std::string::npos);

  EXPECT_NE(source.find("material.pbr_base_color_texture, EE_GLTF_RASTER_TEXTURE_BASE_COLOR"), std::string::npos);
  EXPECT_NE(source.find("material.pbr_diffuse_texture, EE_GLTF_RASTER_TEXTURE_BASE_COLOR"), std::string::npos);
  EXPECT_NE(source.find("material.pbr_metallic_roughness_texture, EE_GLTF_RASTER_TEXTURE_METALLIC_ROUGHNESS"),
            std::string::npos);
  EXPECT_NE(source.find("material.pbr_specular_glossiness_texture, EE_GLTF_RASTER_TEXTURE_METALLIC_ROUGHNESS"),
            std::string::npos);
  EXPECT_NE(source.find("material.normal_texture, EE_GLTF_RASTER_TEXTURE_NORMAL"), std::string::npos);
  EXPECT_NE(source.find("material.emissive_texture, EE_GLTF_RASTER_TEXTURE_EMISSIVE"), std::string::npos);
  EXPECT_NE(source.find("material.occlusion_texture, EE_GLTF_RASTER_TEXTURE_OCCLUSION"), std::string::npos);
  EXPECT_NE(source.find("EE_GLTF_RASTER_TEXTURE_UNSUPPORTED_EXTENSION"), std::string::npos);
}

TEST(GltfRasterMaterial, ActiveRasterShadersUseGltfEvaluator) {
  const std::filesystem::path paths[] = {
      ShaderPath("Graphics/Fragment/Standard/StandardDeferred.frag"),
      ShaderPath("Graphics/Fragment/Standard/StandardTransparent.frag"),
      ShaderPath("Graphics/Fragment/ShadowMapPassThrough.frag"),
  };

  for (const auto& path : paths) {
    const auto source = ReadTextFile(path);
    ASSERT_FALSE(source.empty()) << path.string();
    EXPECT_NE(source.find("#include \"GltfRasterMaterial.glsl\""), std::string::npos) << path.string();
    EXPECT_NE(source.find("EE_EVALUATE_GLTF_RASTER_SURFACE"), std::string::npos) << path.string();
    EXPECT_EQ(source.find(std::string("EE_MATERIAL") + "_PROPERTIES"), std::string::npos) << path.string();
    EXPECT_EQ(source.find(std::string("Material") + "Properties"), std::string::npos) << path.string();
  }
}

TEST(GltfRasterMaterial, PostProcessConsumersReadExpandedGBuffer) {
  const std::filesystem::path normal_paths[] = {
      ShaderPath("Compute/PostProcessing/SSRReflect.comp"),
      ShaderPath("Compute/PostProcessing/SSAOGeometry.comp"),
      ShaderPath("Graphics/Fragment/PostProcessing/SSRReflect.frag"),
  };
  for (const auto& path : normal_paths) {
    const auto source = ReadTextFile(path);
    ASSERT_FALSE(source.empty()) << path.string();
    EXPECT_NE(source.find("binding = 21) uniform sampler2D inNormalRoughness"), std::string::npos) << path.string();
    EXPECT_NE(source.find("texture(inNormalRoughness"), std::string::npos) << path.string();
    EXPECT_EQ(source.find("uniform sampler2D inMaterial"), std::string::npos) << path.string();
  }

  const std::filesystem::path combine_paths[] = {
      ShaderPath("Compute/PostProcessing/SSRCombine.comp"),
      ShaderPath("Graphics/Fragment/PostProcessing/SSRCombine.frag"),
  };
  for (const auto& path : combine_paths) {
    const auto source = ReadTextFile(path);
    ASSERT_FALSE(source.empty()) << path.string();
    EXPECT_EQ(source.find("#include \"GltfRasterMaterial.glsl\""), std::string::npos) << path.string();
    EXPECT_EQ(source.find("EE_EVALUATE_GLTF_RASTER_SURFACE"), std::string::npos) << path.string();
    EXPECT_NE(source.find("binding = 21) uniform sampler2D inNormalRoughness"), std::string::npos) << path.string();
    EXPECT_NE(source.find("binding = 22) uniform sampler2D inPbrFlags"), std::string::npos) << path.string();
    EXPECT_NE(source.find("float roughness = texture(inNormalRoughness, texCoord).a"), std::string::npos)
        << path.string();
    EXPECT_NE(source.find("float metallic = texture(inPbrFlags, texCoord).x"), std::string::npos) << path.string();
  }
}

TEST(GltfRasterMaterial, DeferredLightingReadsExpandedGBuffer) {
  const std::filesystem::path paths[] = {
      ShaderPath("Graphics/Fragment/Standard/StandardDeferredLighting.frag"),
      ShaderPath("Graphics/Fragment/Standard/StandardDeferredLightingSceneCamera.frag"),
  };

  for (const auto& path : paths) {
    const auto source = ReadTextFile(path);
    ASSERT_FALSE(source.empty()) << path.string();
    EXPECT_EQ(source.find("#include \"GltfRasterMaterial.glsl\""), std::string::npos) << path.string();
    EXPECT_EQ(source.find("EE_EVALUATE_GLTF_RASTER_SURFACE"), std::string::npos) << path.string();
    EXPECT_NE(source.find("binding = 20) uniform sampler2D inBaseColorAO"), std::string::npos) << path.string();
    EXPECT_NE(source.find("binding = 21) uniform sampler2D inNormalRoughness"), std::string::npos) << path.string();
    EXPECT_NE(source.find("binding = 22) uniform sampler2D inPbrFlags"), std::string::npos) << path.string();
    EXPECT_NE(source.find("binding = 23) uniform sampler2D inEmissive"), std::string::npos) << path.string();
    EXPECT_NE(source.find("float roughness = normalRoughness.a"), std::string::npos) << path.string();
    EXPECT_NE(source.find("float metallic = pbrFlags.x"), std::string::npos) << path.string();
    EXPECT_NE(source.find("float ao = baseColorAO.a"), std::string::npos) << path.string();
    EXPECT_NE(source.find("vec4 albedo = vec4(baseColorAO.rgb, 1.0)"), std::string::npos) << path.string();
  }
}

TEST(GltfRasterMaterial, DeferredPrepassDeclaresExpandedGBufferOutputs) {
  const auto deferred = ReadTextFile(ShaderPath("Graphics/Fragment/Standard/StandardDeferred.frag"));
  ASSERT_FALSE(deferred.empty());

  EXPECT_EQ(deferred.find("outNormal"), std::string::npos);
  EXPECT_EQ(deferred.find("outMaterial"), std::string::npos);
  EXPECT_NE(deferred.find("layout (location = 0) out vec4 outGBufferBaseColorAO"), std::string::npos);
  EXPECT_NE(deferred.find("layout (location = 1) out vec4 outGBufferNormalRoughness"), std::string::npos);
  EXPECT_NE(deferred.find("layout (location = 2) out vec4 outGBufferPbrFlags"), std::string::npos);
  EXPECT_NE(deferred.find("layout (location = 3) out vec4 outGBufferEmissive"), std::string::npos);
  EXPECT_NE(deferred.find("layout (location = 4) out vec4 outGBufferUtility"), std::string::npos);
  EXPECT_NE(deferred.find("outGBufferBaseColorAO = vec4(max(surface.base_color.rgb, vec3(0.0)), "
                          "max(surface.occlusion, 0.0))"),
            std::string::npos);
  EXPECT_NE(deferred.find("outGBufferNormalRoughness = vec4(world_normal, surface.roughness)"), std::string::npos);
  EXPECT_NE(deferred.find("outGBufferPbrFlags = vec4(surface.metallic, 0.0, 0.0, 0.0)"), std::string::npos);
  EXPECT_NE(deferred.find("outGBufferEmissive = vec4(surface.emissive, 0.0)"), std::string::npos);
  EXPECT_NE(deferred.find("outGBufferUtility = vec4(float(instance_index), float(instance.info_index), "
                          "float(instance.material_index), 0.0)"),
            std::string::npos);
}

TEST(GltfRasterMaterial, DeferredGBufferLegacyBindingsAreRetired) {
  const auto platform = ReadTextFile(SdkPath("include/Rendering/Platform/Platform.hpp"));
  const auto camera_header = ReadTextFile(SdkPath("include/Rendering/Camera.hpp"));
  const auto camera = ReadTextFile(SdkPath("src/Camera.cpp"));
  const auto editor = ReadTextFile(SdkPath("src/EditorLayer.cpp"));
  const auto inspection = ReadTextFile(SdkPath("src/Editor/SDKInspectionAdapters.cpp"));
  const auto render_layer = ReadTextFile(SdkPath("src/RenderLayer.cpp"));
  ASSERT_FALSE(platform.empty());
  ASSERT_FALSE(camera_header.empty());
  ASSERT_FALSE(camera.empty());
  ASSERT_FALSE(editor.empty());
  ASSERT_FALSE(inspection.empty());
  ASSERT_FALSE(render_layer.empty());

  EXPECT_NE(platform.find("g_buffer_attribute = VK_FORMAT_R16G16B16A16_SFLOAT"), std::string::npos);
  EXPECT_NE(platform.find("g_buffer_utility = VK_FORMAT_R32G32B32A32_SFLOAT"), std::string::npos);
  EXPECT_EQ(platform.find("g_buffer_color"), std::string::npos);
  EXPECT_EQ(platform.find("g_buffer_material"), std::string::npos);
  EXPECT_NE(render_layer.find("CreateDeferredGBufferColorAttachmentFormats"), std::string::npos);
  EXPECT_EQ(render_layer.find("camera_g_buffer_layout_->PushDescriptorBinding(18"), std::string::npos);
  EXPECT_EQ(render_layer.find("camera_g_buffer_layout_->PushDescriptorBinding(19"), std::string::npos);

  for (uint32_t binding = 20; binding <= 24; binding++) {
    EXPECT_NE(render_layer.find("PushDescriptorBinding(" + std::to_string(binding)), std::string::npos);
    EXPECT_NE(camera.find("UpdateImageDescriptorBinding(" + std::to_string(binding)), std::string::npos);
  }
  EXPECT_EQ(camera.find("UpdateImageDescriptorBinding(18"), std::string::npos);
  EXPECT_EQ(camera.find("UpdateImageDescriptorBinding(19"), std::string::npos);

  EXPECT_NE(camera.find("AppendGBufferAttachmentInfo(attachment_infos, attachment, g_buffer_base_color_ao_view_)"),
            std::string::npos);
  EXPECT_NE(camera.find("AppendGBufferAttachmentInfo(attachment_infos, attachment, g_buffer_utility_view_)"),
            std::string::npos);
  EXPECT_EQ(camera.find("g_buffer_material_"), std::string::npos);
  EXPECT_EQ(camera_header.find("g_buffer_material_"), std::string::npos);
  EXPECT_NE(editor.find("GetGBufferUtilityImage()"), std::string::npos);
  EXPECT_NE(editor.find("val = glm::round(ptr[0])"), std::string::npos);
  EXPECT_EQ(editor.find("GetGBufferNormalImage()"), std::string::npos);
  EXPECT_EQ(inspection.find("GetGBufferMaterialTexCoordImTextureId"), std::string::npos);
  EXPECT_EQ(inspection.find("GetGBufferMaterialIndicesImTextureId"), std::string::npos);
}

TEST(GltfRasterMaterial, EcoSysLabDeferredShadersWriteExpandedGBuffer) {
  const std::filesystem::path shader_paths[] = {
      RepoPath("EvoEngine_Packages/EcoSysLab/Internals/EcoSysLabResources/Shaders/Graphics/Fragment/DynamicStrands/"
               "Rendering/Foliage.frag"),
      RepoPath("EvoEngine_Packages/EcoSysLab/Internals/EcoSysLabResources/Shaders/Graphics/Fragment/DynamicStrands/"
               "Rendering/SmallSegments.frag"),
      RepoPath("EvoEngine_Packages/EcoSysLab/Internals/EcoSysLabResources/Shaders/Graphics/Fragment/DynamicStrands/"
               "Rendering/SmallSegmentsVisualization.frag"),
      RepoPath("EvoEngine_Packages/EcoSysLab/Internals/EcoSysLabResources/Shaders/Graphics/Fragment/DynamicStrands/"
               "Rendering/AlphaShapeMeshing/Branches.frag"),
      RepoPath("EvoEngine_Packages/EcoSysLab/Internals/EcoSysLabResources/Shaders/Graphics/Fragment/DynamicStrands/"
               "Rendering/KineticVoronoiMeshing/Branches.frag"),
  };

  for (const auto& path : shader_paths) {
    const auto source = ReadTextFile(path);
    ASSERT_FALSE(source.empty()) << path.string();
    EXPECT_EQ(source.find("outNormal"), std::string::npos) << path.string();
    EXPECT_EQ(source.find("outMaterial"), std::string::npos) << path.string();
    EXPECT_NE(source.find("layout(location = 0) out vec4 outGBufferBaseColorAO"), std::string::npos) << path.string();
    EXPECT_NE(source.find("layout(location = 1) out vec4 outGBufferNormalRoughness"), std::string::npos)
        << path.string();
    EXPECT_NE(source.find("layout(location = 2) out vec4 outGBufferPbrFlags"), std::string::npos) << path.string();
    EXPECT_NE(source.find("layout(location = 3) out vec4 outGBufferEmissive"), std::string::npos) << path.string();
    EXPECT_NE(source.find("layout(location = 4) out vec4 outGBufferUtility"), std::string::npos) << path.string();
    EXPECT_NE(source.find("outGBufferUtility = vec4(float(EE_INSTANCE_INDEX)"), std::string::npos) << path.string();
  }

  const std::filesystem::path pipeline_paths[] = {
      RepoPath("EvoEngine_Packages/EcoSysLab/src/DsAlphaShapeBranchesRendering.cpp"),
      RepoPath("EvoEngine_Packages/EcoSysLab/src/DsAlphaShapeSmallSegmentsRendering.cpp"),
      RepoPath("EvoEngine_Packages/EcoSysLab/src/DsKineticVoronoiMeshing.cpp"),
      RepoPath("EvoEngine_Packages/EcoSysLab/src/DynamicStrandsFoliageRendering.cpp"),
  };

  for (const auto& path : pipeline_paths) {
    const auto source = ReadTextFile(path);
    ASSERT_FALSE(source.empty()) << path.string();
    EXPECT_EQ(source.find("g_buffer_color"), std::string::npos) << path.string();
    EXPECT_NE(source.find("Platform::Constants::g_buffer_attribute"), std::string::npos) << path.string();
    EXPECT_NE(source.find("Platform::Constants::g_buffer_utility"), std::string::npos) << path.string();
  }
}

TEST(GltfRasterMaterial, ActiveRasterNormalMapsUseTangentHandedness) {
  const auto geometry = ReadTextFile(SdkPath("src/IGeometry.cpp"));
  const auto standard = ReadTextFile(ShaderPath("Graphics/Vertex/Standard/Standard.vert"));
  const auto standard_instanced = ReadTextFile(ShaderPath("Graphics/Vertex/Standard/StandardInstanced.vert"));
  const auto standard_skinned = ReadTextFile(ShaderPath("Graphics/Vertex/Standard/StandardSkinned.vert"));
  const auto standard_mesh = ReadTextFile(ShaderPath("Graphics/Mesh/Standard/Standard.mesh"));
  const auto standard_meshlet_colored = ReadTextFile(ShaderPath("Graphics/Mesh/Standard/StandardMeshletColored.mesh"));
  const auto deferred = ReadTextFile(ShaderPath("Graphics/Fragment/Standard/StandardDeferred.frag"));
  const auto raster_material = ReadTextFile(ShaderPath("Includes/GltfRasterMaterial.glsl"));

  ASSERT_FALSE(geometry.empty());
  ASSERT_FALSE(standard.empty());
  ASSERT_FALSE(standard_instanced.empty());
  ASSERT_FALSE(standard_skinned.empty());
  ASSERT_FALSE(standard_mesh.empty());
  ASSERT_FALSE(standard_meshlet_colored.empty());
  ASSERT_FALSE(deferred.empty());
  ASSERT_FALSE(raster_material.empty());

  EXPECT_NE(geometry.find("mesh[5].location = 9"), std::string::npos);
  EXPECT_NE(geometry.find("mesh[5].format = VK_FORMAT_R32_SFLOAT"), std::string::npos);
  EXPECT_NE(geometry.find("mesh[5].offset = offsetof(Vertex, vertex_info3)"), std::string::npos);
  EXPECT_NE(geometry.find("skinned_mesh[9].location = 9"), std::string::npos);
  EXPECT_NE(geometry.find("skinned_mesh[9].format = VK_FORMAT_R32_SFLOAT"), std::string::npos);
  EXPECT_NE(geometry.find("skinned_mesh[9].offset = offsetof(SkinnedVertex, vertex_info3)"), std::string::npos);

  for (const auto* source : {&standard, &standard_instanced, &standard_skinned}) {
    EXPECT_NE(source->find("layout (location = 9) in float inTangentHandedness"), std::string::npos);
    EXPECT_NE(source->find("flat float TangentHandedness"), std::string::npos);
    EXPECT_NE(source->find("vs_out.TangentHandedness = inTangentHandedness < 0.0 ? -1.0 : 1.0"), std::string::npos);
    EXPECT_EQ(source->find("T = normalize(T - dot(T, N) * N)"), std::string::npos);
  }

  for (const auto* source : {&standard_mesh, &standard_meshlet_colored}) {
    EXPECT_NE(source->find("flat float TangentHandedness"), std::string::npos);
    EXPECT_NE(source->find("ms_v_out[vert].TangentHandedness = v.vertex_info3 < 0.0 ? -1.0 : 1.0"), std::string::npos);
    EXPECT_EQ(source->find("T = normalize(T - dot(T, N) * N)"), std::string::npos);
  }

  EXPECT_NE(deferred.find("flat float TangentHandedness"), std::string::npos);
  EXPECT_NE(deferred.find("fs_in.TangentHandedness"), std::string::npos);
  EXPECT_EQ(deferred.find("EE_EVALUATE_GLTF_RASTER_NORMAL(material_index, tex_coord, tex_coord, fs_in.Normal, "
                          "fs_in.Tangent)"),
            std::string::npos);
  EXPECT_NE(raster_material.find("EE_GLTF_SAFE_NORMALIZE"), std::string::npos);
  EXPECT_NE(raster_material.find("tangent - n * dot(n, tangent)"), std::string::npos);
  EXPECT_NE(raster_material.find("EE_GLTF_FALLBACK_TANGENT(n)"), std::string::npos);
  EXPECT_NE(raster_material.find("vec3 b = cross(n, t) * bitangent_sign"), std::string::npos);
}

TEST(GltfRasterMaterial, PrefabImporterPreservesMissingTexCoordAttributes) {
  const auto prefab_source = ReadTextFile(SdkPath("src/Prefab.cpp"));
  ASSERT_FALSE(prefab_source.empty());

  EXPECT_NE(prefab_source.find("attributes.tex_coord = false"), std::string::npos);
  EXPECT_NE(prefab_source.find("skinned_vertex_attributes.tex_coord = false"), std::string::npos);
}
