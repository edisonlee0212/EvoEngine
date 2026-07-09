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
}  // namespace

TEST(GltfRasterMaterial, EvaluatorUsesCanonicalGltfTextureInfo) {
  const auto source = ReadTextFile(ShaderPath("Includes/GltfRasterMaterial.glsl"));
  ASSERT_FALSE(source.empty());

  EXPECT_NE(source.find("EE_GLTF_HAS_TEXTURE"), std::string::npos);
  EXPECT_NE(source.find("return uint(texture_info_slot) > 0u"), std::string::npos);
  EXPECT_NE(source.find("EE_GLTF_TEXTURE_INFOS[uint(texture_info_slot)]"), std::string::npos);
  EXPECT_NE(source.find("texture_info.uv_transform"), std::string::npos);
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
  EXPECT_NE(rendering_docs.find("Bindless texture arrays are reserved for ray tracing and ray query paths"),
            std::string::npos);
  EXPECT_NE(rendering_docs.find("Raster-only or lower-end device mode must avoid"), std::string::npos);
  EXPECT_NE(rendering_docs.find("bindless descriptor layouts"), std::string::npos);
  EXPECT_NE(rendering_docs.find("tracing compute texture users also migrate to fixed descriptor sets"),
            std::string::npos);
}

TEST(GltfRasterMaterial, ActiveRasterShadersUseGltfEvaluator) {
  const std::filesystem::path paths[] = {
      ShaderPath("Graphics/Fragment/Standard/StandardDeferred.frag"),
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
