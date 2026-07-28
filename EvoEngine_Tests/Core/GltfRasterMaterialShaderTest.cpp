#include "EvoEngine_SDK_PCH.hpp"

#include <gtest/gtest.h>

#include "EnvironmentalLighting.hpp"

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <initializer_list>
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

std::string RemoveAsciiWhitespace(const std::string& source) {
  std::string result;
  result.reserve(source.size());
  for (const char c : source) {
    if (c != ' ' && c != '\n' && c != '\r' && c != '\t') {
      result.push_back(c);
    }
  }
  return result;
}

bool ContainsIgnoringWhitespace(const std::string& source, const std::string& needle) {
  return RemoveAsciiWhitespace(source).find(RemoveAsciiWhitespace(needle)) != std::string::npos;
}

bool HasShaderInputLocation(const std::string& shader, const int location) {
  const auto suffix = std::to_string(location) + ") in";
  return shader.find("layout (location = " + suffix) != std::string::npos ||
         shader.find("layout(location = " + suffix) != std::string::npos;
}

bool HasSlangLocationAttribute(const std::string& source, const int location) {
  return source.find("[[vk::location(" + std::to_string(location) + ")]]") != std::string::npos;
}

bool HasFragmentOutputLocation(const std::string& source, const int location, const std::string& name) {
  const auto glsl_location = "layout(location = " + std::to_string(location) + ") out vec4 " + name;
  const auto slang_field = "vec4 " + name + ";";
  return source.find(glsl_location) != std::string::npos ||
         (HasSlangLocationAttribute(source, location) && source.find(slang_field) != std::string::npos);
}

void ExpectShaderInputLocations(const std::string& shader, const std::filesystem::path& shader_path,
                                const std::initializer_list<int> expected_locations) {
  for (int location = 0; location <= 12; ++location) {
    const bool expected =
        std::find(expected_locations.begin(), expected_locations.end(), location) != expected_locations.end();
    EXPECT_EQ(HasShaderInputLocation(shader, location), expected) << shader_path.string() << " location=" << location;
  }
}

void ExpectSlangInputLocations(const std::string& shader, const std::filesystem::path& shader_path,
                               const std::string& input_struct_name,
                               const std::initializer_list<int> expected_locations) {
  const auto input_block = ExtractSourceRange(shader, "struct " + input_struct_name + " {", "};");
  ASSERT_FALSE(input_block.empty()) << shader_path.string();
  for (int location = 0; location <= 12; ++location) {
    const bool expected =
        std::find(expected_locations.begin(), expected_locations.end(), location) != expected_locations.end();
    EXPECT_EQ(HasSlangLocationAttribute(input_block, location), expected)
        << shader_path.string() << " location=" << location;
  }
}
}  // namespace

TEST(GltfRasterMaterial, EvaluatorCoversCanonicalTextureInfoAndPbrTerms) {
  const auto source = ReadTextFile(ShaderPath("Includes/GltfRasterMaterial.slangh"));
  ASSERT_FALSE(source.empty());

  EXPECT_NE(source.find("EE_GLTF_HAS_TEXTURE"), std::string::npos);
  EXPECT_NE(source.find("return uint(texture_info_slot) > 0u"), std::string::npos);
  EXPECT_NE(source.find("EE_GLTF_TEXTURE_INFOS[uint(texture_info_slot)]"), std::string::npos);
  EXPECT_NE(source.find("texture_info.uv_transform"), std::string::npos);
  EXPECT_NE(source.find("#ifndef EE_GLTF_RASTER_FIXED_MATERIAL_TEXTURES"), std::string::npos);
  EXPECT_NE(source.find("EE_GLTF_SAMPLE_BINDLESS_TEXTURE"), std::string::npos);
  EXPECT_NE(source.find("EE_TEXTURE_2DS[nonuniformEXT(texture_info.index)]"), std::string::npos);
  EXPECT_EQ(source.find(std::string("Material") + "Properties"), std::string::npos);

  EXPECT_NE(source.find("EE_GLTF_PBR_MODEL_SPECULAR_GLOSSINESS"), std::string::npos);
  EXPECT_NE(source.find("pbr_specular_glossiness_texture"), std::string::npos);
  EXPECT_EQ(source.find("EE_GLTF_CONVERT_SPEC_GLOSS_TO_METALLIC_ROUGHNESS"), std::string::npos);
  EXPECT_NE(source.find("surface.specular_f0 = clamp(specular"), std::string::npos);
  EXPECT_NE(source.find("surface.base_color.rgb = diffuse.rgb * (1.0 - max(surface.specular_f0.r"), std::string::npos);
  EXPECT_NE(source.find("surface.roughness = max(1.0 - glossiness"), std::string::npos);
  EXPECT_NE(source.find("pbr_metallic_roughness_texture"), std::string::npos);
  EXPECT_NE(source.find("dielectric_f0 = pow((material_ior - 1.0)"), std::string::npos);
  EXPECT_NE(source.find("specular_weight = material.specular_factor"), std::string::npos);
  EXPECT_NE(source.find("material.ior == 0.0 ? 0.0"), std::string::npos);
  EXPECT_NE(source.find("dielectric_specular_f0 * max(specular_color"), std::string::npos);
  EXPECT_NE(source.find("surface.specular_f0 = mix(dielectric_specular_f0"), std::string::npos);
  EXPECT_NE(source.find("surface.specular_f90 = mix(dielectric_specular_f90, 1.0, surface.metallic)"),
            std::string::npos);
  EXPECT_NE(source.find("return f0 + (f90 - f0) * pow"), std::string::npos);
  EXPECT_NE(source.find("mix(dielectric_specular_f0, max(surface.base_color.rgb"), std::string::npos);
  EXPECT_EQ(source.find("return mix(vec3(0.04)"), std::string::npos);
  const auto rebase_specular_f0 =
      ExtractSourceRange(source, "vec3 EE_GLTF_RASTER_REBASE_SPECULAR_F0", "vec3 EE_GLTF_SAFE_NORMALIZE");
  EXPECT_NE(rebase_specular_f0.find("return surface.specular_f0"), std::string::npos);
  EXPECT_NE(rebase_specular_f0.find("const float metallic = clamp(surface.metallic"), std::string::npos);
  EXPECT_NE(rebase_specular_f0.find("surface.specular_f0 - original_base_color * metallic"), std::string::npos);
  EXPECT_NE(rebase_specular_f0.find("return mix(dielectric_specular_f0, rebased_base_color, metallic)"),
            std::string::npos);
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
  EXPECT_NE(source.find("material.pbr_diffuse_factor * vertex_color"), std::string::npos);
  EXPECT_NE(source.find("material.pbr_base_color_factor * vertex_color"), std::string::npos);
  EXPECT_NE(source.find("EE_GLTF_SRGB_TO_LINEAR"), std::string::npos);
  EXPECT_NE(source.find("lessThanEqual(encoded, vec3(0.04045))"), std::string::npos);
  EXPECT_NE(source.find("sample_value.rgb = EE_GLTF_SRGB_TO_LINEAR(sample_value.rgb)"), std::string::npos);
  EXPECT_NE(source.find("surface.alpha_mode == EE_GLTF_ALPHA_MODE_OPAQUE"), std::string::npos);
  EXPECT_NE(source.find("EE_GLTF_SAMPLE_TEXTURE_LOD0(material.transmission_texture"), std::string::npos);
  EXPECT_NE(source.find("effective_diffuse_transmission * diffuse_transmission_color"), std::string::npos);
  EXPECT_NE(source.find("EE_GLTF_RASTER_FRESNEL(specular_f0, vec3(specular_weight)"), std::string::npos);
  EXPECT_NE(source.find("const float remaining_energy = 1.0 - max(fresnel.r"), std::string::npos);
}

TEST(GltfRasterMaterial, FractionalSpecularF90AndUnlitSurviveDeferredAndTransparentPaths) {
  const auto lighting = ReadTextFile(ShaderPath("Includes/Lighting.slangh"));
  const auto deferred = ReadTextFile(ShaderPath("Graphics/Fragment/Standard/StandardDeferredLighting.slang"));
  const auto scene_deferred =
      ReadTextFile(ShaderPath("Graphics/Fragment/Standard/StandardDeferredLightingSceneCamera.slang"));
  const auto transparent = ReadTextFile(ShaderPath("Graphics/Fragment/Standard/StandardTransparent.slang"));
  ASSERT_FALSE(lighting.empty());
  ASSERT_FALSE(deferred.empty());
  ASSERT_FALSE(scene_deferred.empty());
  ASSERT_FALSE(transparent.empty());

  EXPECT_NE(lighting.find("vec3(F90) - F0"), std::string::npos);
  EXPECT_NE(lighting.find("vec3(F90 * (1.0f - roughness))"), std::string::npos);
  EXPECT_NE(lighting.find("vec3(F90 * brdf.y)"), std::string::npos);
  EXPECT_NE(deferred.find("vec4 emissiveSample = texture(inEmissive"), std::string::npos);
  const auto deferred_unlit =
      ExtractSourceRange(deferred, "if (emissiveSample.a < 0.0f) {", "vec3 normal = normalize(normalRoughness.xyz);");
  EXPECT_NE(
      deferred_unlit.find("FragColor = vec4(indirectLightingDebugView == 0 ? baseColorAO.rgb : vec3(0.0f), 1.0f)"),
      std::string::npos);
  EXPECT_NE(deferred_unlit.find("return;"), std::string::npos);
  EXPECT_EQ(deferred_unlit.find("EE_FUNC_CALCULATE_"), std::string::npos);
  EXPECT_NE(deferred.find("float F90 = emissiveSample.a"), std::string::npos);

  EXPECT_NE(scene_deferred.find("bool unlit = emissiveSample.a < 0.0"), std::string::npos);
  EXPECT_NE(scene_deferred.find("vec3 color = indirectLightingDebugView == 0 ? albedo.rgb : vec3(0.0f)"),
            std::string::npos);
  const auto scene_lit_only =
      ExtractSourceRange(scene_deferred, "if (!unlit) {", "\n    }\n\n    if (indirectLightingDebugView != 0) {");
  EXPECT_NE(scene_lit_only.find("EE_FUNC_CALCULATE_LIGHTS"), std::string::npos);
  EXPECT_NE(scene_lit_only.find("EE_FUNC_CALCULATE_DDGI_ENVIRONMENTAL_LIGHT"), std::string::npos);
  EXPECT_NE(scene_deferred.find("if (indirectLightingDebugView != 0) {\n"
                                "        FragColor = vec4(0.0f, 0.0f, 0.0f, 1.0f);\n"
                                "        return;\n"
                                "    }\n\n"
                                "    vec4 outputColor;"),
            std::string::npos);
  EXPECT_NE(scene_deferred.find("float F90 = emissiveSample.a"), std::string::npos);

  const auto transparent_unlit = ExtractSourceRange(transparent, "if (EE_GLTF_MATERIALS[material_index].unlit != 0) {",
                                                    "vec3 normal = EE_EVALUATE_GLTF_RASTER_NORMAL");
  EXPECT_NE(transparent_unlit.find("indirectLightingDebugView == 0 ? surface.base_color.rgb : vec3(0.0f)"),
            std::string::npos);
  EXPECT_NE(transparent_unlit.find("indirectLightingDebugView == 0 ? EE_GLTF_RASTER_OPACITY(surface) : 1.0f"),
            std::string::npos);
  EXPECT_NE(transparent_unlit.find("return;"), std::string::npos);
  EXPECT_EQ(transparent_unlit.find("EE_FUNC_CALCULATE_"), std::string::npos);
  EXPECT_NE(transparent.find("surface.specular_f90"), std::string::npos);
}

TEST(GltfRasterMaterial, PerFrameBindsCanonicalMaterialBuffers) {
  const auto per_frame = ReadTextFile(ShaderPath("Includes/PerFrame.slangh"));
  ASSERT_FALSE(per_frame.empty());

  EXPECT_NE(per_frame.find("EE_GLTF_MATERIALS_BLOCK_BINDING 11"), std::string::npos);
  EXPECT_NE(per_frame.find("EE_GLTF_TEXTURE_INFOS_BLOCK_BINDING 12"), std::string::npos);
  EXPECT_NE(per_frame.find("#ifndef EE_SKIP_PER_FRAME_BINDLESS_TEXTURES"), std::string::npos);
  EXPECT_NE(per_frame.find("#include \"Textures.slangh\""), std::string::npos);
  EXPECT_NE(per_frame.find("#include \"GltfMaterial.slangh\""), std::string::npos);

  const auto material = ReadTextFile(ShaderPath("Includes/GltfMaterial.slangh"));
  ASSERT_FALSE(material.empty());
  EXPECT_NE(material.find("#extension GL_EXT_scalar_block_layout : require"), std::string::npos);
  EXPECT_NE(material.find("layout(scalar, set = EE_GLTF_MATERIALS_BLOCK_SET"), std::string::npos);
  EXPECT_NE(material.find("layout(scalar, set = EE_GLTF_TEXTURE_INFOS_BLOCK_SET"), std::string::npos);
}

TEST(GltfRasterMaterial, RasterDescriptorContractIsDocumented) {
  const auto rendering_docs = ReadTextFile(RepoPath("docs/rendering.md"));
  ASSERT_FALSE(rendering_docs.empty());

  EXPECT_NE(rendering_docs.find("Raster material descriptors use descriptor set 3"), std::string::npos);
  EXPECT_NE(rendering_docs.find("set 1 remains available for"), std::string::npos);
  EXPECT_NE(rendering_docs.find("instanced/strand data"), std::string::npos);
  EXPECT_NE(rendering_docs.find("set 2 remains available for lighting or pass descriptors"), std::string::npos);
  EXPECT_NE(rendering_docs.find("Raster material shaders must not use runtime bindless descriptor arrays"),
            std::string::npos);
  EXPECT_NE(rendering_docs.find("Fixed-size image arrays and atlases are allowed"), std::string::npos);
  EXPECT_NE(rendering_docs.find("| 0 | Base color or diffuse | White. |"), std::string::npos);
  EXPECT_NE(rendering_docs.find("| 1 | Metallic-roughness or specular-glossiness | White. |"), std::string::npos);
  EXPECT_NE(rendering_docs.find("| 2 | Normal | Flat normal. |"), std::string::npos);
  EXPECT_NE(rendering_docs.find("| 3 | Emissive | Black. |"), std::string::npos);
  EXPECT_NE(rendering_docs.find("| 4 | Occlusion | White. |"), std::string::npos);
  EXPECT_NE(rendering_docs.find("| 5 | Clearcoat | White. |"), std::string::npos);
  EXPECT_NE(rendering_docs.find("| 6 | Clearcoat roughness | White. |"), std::string::npos);
  EXPECT_NE(rendering_docs.find("| 7 | Clearcoat normal | Flat normal. |"), std::string::npos);
  EXPECT_NE(rendering_docs.find("renderer-owned runtime state keyed by material index"), std::string::npos);
  EXPECT_NE(rendering_docs.find("Descriptor sets are not"), std::string::npos);
  EXPECT_NE(rendering_docs.find("deduplicated across material indices"), std::string::npos);
  EXPECT_NE(rendering_docs.find("texture's existing combined image sampler"), std::string::npos);
  EXPECT_NE(rendering_docs.find("uses material-batched indirect ranges"), std::string::npos);
  EXPECT_NE(rendering_docs.find("restores deferred mesh indirect rendering"), std::string::npos);
  EXPECT_NE(rendering_docs.find("material per-frame descriptor set that keeps the shared per-frame buffers"),
            std::string::npos);
  EXPECT_NE(rendering_docs.find("omits bindless"), std::string::npos);
  EXPECT_NE(rendering_docs.find("cubemap array"), std::string::npos);
  EXPECT_NE(rendering_docs.find("Built-in shadow-map passes treat all mesh materials as opaque"), std::string::npos);
  EXPECT_NE(rendering_docs.find("do not sample material textures for alpha discard"), std::string::npos);
  EXPECT_NE(rendering_docs.find("regular mesh shadow draws"), std::string::npos);
  EXPECT_NE(rendering_docs.find("opaque shadow indirect command path"), std::string::npos);
  EXPECT_NE(rendering_docs.find("Raster lighting uses a fixed raster-global texture descriptor set"),
            std::string::npos);
  EXPECT_NE(rendering_docs.find("prefiltered global environment cubemap, ambient occlusion, and a fixed array of 32"),
            std::string::npos);
  EXPECT_NE(rendering_docs.find("Set 2 still owns shared shadow-map and DDGI atlas bindings"), std::string::npos);
  EXPECT_NE(rendering_docs.find("external forward callbacks"), std::string::npos);
  EXPECT_NE(rendering_docs.find("Bindless texture arrays are reserved for ray tracing and ray query paths"),
            std::string::npos);
  EXPECT_NE(rendering_docs.find("without texture or cubemap descriptor arrays"), std::string::npos);
  EXPECT_NE(rendering_docs.find("skips binding the global texture storage arrays"), std::string::npos);
  EXPECT_NE(rendering_docs.find("non-ray-tracing compute texture inputs use fixed material, global, or"),
            std::string::npos);
}

TEST(GltfRasterMaterial, RasterMaterialDescriptorFallbackResourcesArePresent) {
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

  EXPECT_NE(render_instance_header.find("kRasterMaterialTextureSlotCount = 8"), std::string::npos);
  EXPECT_NE(render_layer_header.find("GetRasterMaterialDescriptorSetLayout"), std::string::npos);
  EXPECT_NE(render_layer_header.find("raster_material_per_frame_layout_"), std::string::npos);
  EXPECT_NE(render_layer_header.find("raster_material_layout_"), std::string::npos);
  EXPECT_NE(render_layer_header.find("raster_lighting_texture_layout_"), std::string::npos);
  EXPECT_NE(render_layer_header.find("raster_material_per_frame_descriptor_sets_"), std::string::npos);
  EXPECT_NE(render_layer_header.find("raster_lighting_texture_descriptor_sets_"), std::string::npos);
  EXPECT_NE(render_layer_header.find("raster_material_white_fallback_texture_"), std::string::npos);
  EXPECT_NE(render_layer_header.find("raster_material_black_fallback_texture_"), std::string::npos);
  EXPECT_NE(render_layer_header.find("raster_material_flat_normal_fallback_texture_"), std::string::npos);
  EXPECT_NE(render_layer.find("raster_material_layout_->PushDescriptorBinding"), std::string::npos);
  EXPECT_NE(render_layer.find("PushPerFrameSceneDescriptorBindings(raster_material_per_frame_layout_)"),
            std::string::npos);
  EXPECT_NE(render_layer.find("PushPerFrameMaterialBufferDescriptorBindings(raster_material_per_frame_layout_)"),
            std::string::npos);
  EXPECT_NE(render_layer.find("ShouldCreatePerFrameBindlessTextureDescriptors"), std::string::npos);
  EXPECT_NE(render_layer.find("per_frame_bindless_texture_descriptors_enabled_"), std::string::npos);
  EXPECT_NE(render_layer.find("PushPerFrameBindlessTextureDescriptorBindings(per_frame_layout_"), std::string::npos);
  EXPECT_NE(render_layer.find("if (per_frame_bindless_texture_descriptors_enabled_)"), std::string::npos);
  EXPECT_NE(render_layer.find("std::make_shared<DescriptorSet>(raster_material_per_frame_layout_)"), std::string::npos);
  EXPECT_NE(render_layer.find("std::make_shared<DescriptorSet>(raster_lighting_texture_layout_)"), std::string::npos);
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
  for (uint32_t binding = 0; binding < 8; binding++) {
    EXPECT_NE(render_instance.find("UpdateImageDescriptorBinding(" + std::to_string(binding)), std::string::npos);
  }

  EXPECT_NE(texture_storage_header.find("TryGetTexture2DDescriptorImageInfo"), std::string::npos);
  EXPECT_NE(texture_storage_header.find("TryGetCubemapDescriptorImageInfo"), std::string::npos);
  EXPECT_NE(texture_storage.find("TextureStorage::TryGetTexture2DDescriptorImageInfo"), std::string::npos);
  EXPECT_NE(texture_storage.find("TextureStorage::TryGetCubemapDescriptorImageInfo"), std::string::npos);
  EXPECT_NE(texture_storage.find("texture_storage.IsGpuUploadPending()"), std::string::npos);
  EXPECT_NE(texture_storage.find("bool IsSampledDescriptorImageLayout"), std::string::npos);
  EXPECT_EQ(CountOccurrences(texture_storage, "!IsSampledDescriptorImageLayout(layout)"), 2u);
  const auto bind_cubemaps = ExtractSourceRange(texture_storage, "void TextureStorage::BindCubemapToDescriptorSet",
                                                "const Texture2DStorage& TextureStorage::PeekTexture2DStorage");
  EXPECT_NE(bind_cubemaps.find("TryGetCubemapDescriptorImageInfo"), std::string::npos);
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
      ExtractSourceRange(render_layer, "if (Platform::MeshShaderEnabled() && !strands_deferred_prepass_pipeline)",
                         "strands_deferred_prepass_pipeline->depth_attachment_format");
  EXPECT_NE(strands_pipeline.find("CreateRasterNoBindlessTextureShaderDefines()"), std::string::npos);
  EXPECT_NE(strands_pipeline.find("CreateRasterMaterialNoBindlessShaderDefines()"), std::string::npos);
  EXPECT_NE(strands_pipeline.find("StandardStrands.slang"), std::string::npos);
  EXPECT_NE(strands_pipeline.find("strand_meshlet_layout_"), std::string::npos);
  EXPECT_EQ(strands_pipeline.find("tessellation_"), std::string::npos);
  EXPECT_EQ(strands_pipeline.find("geometry_shader"), std::string::npos);
  EXPECT_EQ(CountOccurrences(strands_pipeline, "empty_descriptor_set_layout_"), 1);
  EXPECT_NE(strands_pipeline.find("raster_material_layout_"), std::string::npos);

  EXPECT_NE(render_layer.find("raster_material_per_frame_descriptor_sets_[current_frame_index]"), std::string::npos);
  EXPECT_NE(render_layer.find("enable_indirect_rendering, true, count_draw_calls,"), std::string::npos);
  EXPECT_NE(render_layer.find("reflection_probe_capture ? false : wire_frame"), std::string::npos);

  EXPECT_NE(pass_header.find("bind_raster_material_descriptor_sets"), std::string::npos);
  EXPECT_NE(pass.find("BindRasterMaterialDescriptorSet"), std::string::npos);
  EXPECT_NE(pass.find("use_material_batched_indirect_deferred_draws"), std::string::npos);
  EXPECT_NE(pass.find("deferred_mesh_indirect_batches"), std::string::npos);
  EXPECT_NE(pass.find("batch.first_instance_index"), std::string::npos);
  EXPECT_NE(pass.find("batch.first_command * sizeof(VkDrawIndexedIndirectCommand)"), std::string::npos);
  EXPECT_EQ(CountOccurrences(pass, "BindRasterMaterialDescriptorSet(vk_command_buffer, parameters."), 5);
  EXPECT_NE(utilities.find("GetRasterMaterialDescriptorSet(static_cast<uint32_t>(material_index))"), std::string::npos);
  EXPECT_NE(utilities.find("BindDescriptorSet(vk_command_buffer, 3"), std::string::npos);
}

TEST(GltfRasterMaterial, DeferredIndirectUsesMaterialBatchedFixedDescriptors) {
  const auto render_instance_header =
      ReadTextFile(SdkPath("include/Rendering/RenderInstances/RenderInstanceStorage.hpp"));
  const auto render_instance = ReadTextFile(SdkPath("src/RenderInstanceStorage.cpp"));
  const auto pass = ReadTextFile(SdkPath("src/RenderPasses/DeferredGeometryPass.cpp"));
  ASSERT_FALSE(render_instance_header.empty());
  ASSERT_FALSE(render_instance.empty());
  ASSERT_FALSE(pass.empty());

  EXPECT_NE(render_instance_header.find("struct DeferredMeshIndirectBatch"), std::string::npos);
  EXPECT_NE(render_instance_header.find("std::vector<DeferredMeshIndirectBatch> deferred_mesh_indirect_batches"),
            std::string::npos);
  EXPECT_NE(render_instance.find("deferred_mesh_indirect_batches.clear()"), std::string::npos);
  EXPECT_NE(render_instance.find("batch.material_index == render_instance->material_index"), std::string::npos);
  EXPECT_NE(render_instance.find("batch.first_instance_index + static_cast<int32_t>(batch.command_count) =="),
            std::string::npos);
  EXPECT_NE(render_instance.find("batch.first_command = deferred_mesh_command_index"), std::string::npos);
  EXPECT_NE(render_instance.find("batch.command_count++"), std::string::npos);
  EXPECT_NE(render_instance.find("batch.triangle_count +="), std::string::npos);

  EXPECT_NE(pass.find("use_material_batched_indirect_deferred_draws"), std::string::npos);
  EXPECT_NE(pass.find("parameters.bind_raster_material_descriptor_sets"), std::string::npos);
  EXPECT_NE(pass.find("BindRasterMaterialDescriptorSet(vk_command_buffer, parameters.mesh_pipeline"),
            std::string::npos);
  EXPECT_NE(pass.find("push_constant.instance_index = batch.first_instance_index"), std::string::npos);
  EXPECT_NE(pass.find("ResolvePolygonMode(parameters.wire_frame, batch.polygon_mode)"), std::string::npos);
  EXPECT_NE(pass.find("batch.first_command * sizeof(VkDrawMeshTasksIndirectCommandEXT)"), std::string::npos);
  EXPECT_NE(pass.find("batch.first_command * sizeof(VkDrawIndexedIndirectCommand)"), std::string::npos);
}

TEST(GltfRasterMaterial, ShadowPassesUseOpaqueDepthPipelinesAndTransparentPassesBindRasterMaterialDescriptors) {
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

  const std::string shadow_pipeline_region =
      ExtractSourceRange(render_layer, "#pragma region Graphics Pipelines", "if (!deferred_prepass_pipeline_normal)");
  EXPECT_NE(shadow_pipeline_region.find("point_light_shadow_pipeline_normal_opaque"), std::string::npos);
  EXPECT_NE(shadow_pipeline_region.find("spot_light_shadow_pipeline_normal_opaque"), std::string::npos);
  EXPECT_NE(shadow_pipeline_region.find("directional_light_shadow_pipeline_normal_opaque"), std::string::npos);
  EXPECT_NE(shadow_pipeline_region.find("instanced_point_light_shadow_pipeline_opaque"), std::string::npos);
  EXPECT_NE(shadow_pipeline_region.find("skinned_point_light_shadow_pipeline_opaque"), std::string::npos);
  EXPECT_NE(shadow_pipeline_region.find("CreateShadowVertexPipeline"), std::string::npos);
  EXPECT_EQ(shadow_pipeline_region.find("CreateRasterMaterialNoBindlessShaderDefines()"), std::string::npos);
  EXPECT_EQ(shadow_pipeline_region.find("raster_material_per_frame_layout_"), std::string::npos);
  EXPECT_EQ(shadow_pipeline_region.find("raster_material_layout_"), std::string::npos);
  EXPECT_EQ(shadow_pipeline_region.find("ShadowMapPassThrough.slang"), std::string::npos);
  EXPECT_EQ(render_layer.find("CreateShadowMeshPipeline"), std::string::npos);

  const std::string transparent_pipeline = ExtractSourceRange(
      render_layer, "if (!transparent_geometry_pipeline_normal)", "transparent_geometry_pipeline_normal->Initialize()");
  EXPECT_NE(transparent_pipeline.find("CreateRasterMaterialFixedLightingShaderDefines(4)"), std::string::npos);
  EXPECT_NE(transparent_pipeline.find("CreateRasterNoBindlessTextureShaderDefines()"), std::string::npos);
  EXPECT_NE(transparent_pipeline.find("raster_material_per_frame_layout_"), std::string::npos);
  EXPECT_NE(transparent_pipeline.find("lighting_layout_"), std::string::npos);
  EXPECT_NE(transparent_pipeline.find("raster_material_layout_"), std::string::npos);
  EXPECT_NE(transparent_pipeline.find("raster_lighting_texture_layout_"), std::string::npos);
  EXPECT_NE(transparent.find("BindRasterMaterialDescriptorSet(vk_command_buffer, parameters.mesh_pipeline"),
            std::string::npos);
  EXPECT_NE(transparent.find("parameters.raster_lighting_texture_descriptor_set"), std::string::npos);
  EXPECT_NE(transparent.find("vk_command_buffer, 4, parameters.raster_lighting_texture_descriptor_set"),
            std::string::npos);
  EXPECT_NE(transparent.find("render_instance->material_index"), std::string::npos);

  EXPECT_NE(utilities_header.find("BindRasterMaterialDescriptorSet"), std::string::npos);
  EXPECT_EQ(directional_header.find("bind_raster_material_descriptor_sets"), std::string::npos);
  EXPECT_EQ(directional_header.find("raster_material_per_frame_descriptor_set"), std::string::npos);
  EXPECT_EQ(directional_header.find("directional_pipeline"), std::string::npos);
  EXPECT_EQ(directional_header.find("instanced_pipeline"), std::string::npos);
  EXPECT_EQ(directional_header.find("skinned_pipeline"), std::string::npos);
  EXPECT_EQ(directional.find("BindRasterMaterialDescriptorSet"), std::string::npos);
  EXPECT_EQ(directional.find("alpha_tested"), std::string::npos);
  EXPECT_NE(directional.find("draw_indirect(parameters.directional_opaque_pipeline"), std::string::npos);
  EXPECT_EQ(render_layer.find("const bool bind_raster_material_descriptor_sets = true"), std::string::npos);
  EXPECT_EQ(render_layer.find("use_alpha_tested_indirect_shadow"), std::string::npos);
  EXPECT_EQ(render_layer.find("const auto& per_frame_descriptor_set = alpha_tested_pipeline"), std::string::npos);
  EXPECT_NE(render_layer.find("raster_lighting_texture_descriptor_set"), std::string::npos);
  EXPECT_EQ(render_layer.find("point_light_info_block.viewport, false, true"), std::string::npos);
  EXPECT_EQ(render_layer.find("spot_light_info_block.viewport, false, true"), std::string::npos);
  EXPECT_EQ(render_layer.find("BindRasterMaterialDescriptorSet(vk_command_buffer, target_pipeline, "
                              "current_render_instances"),
            std::string::npos);
  const std::string directional_call =
      ExtractSourceRange(render_layer, "DirectionalLightShadowPass::Execute", "DeferredGeometryPass::CreateDescriptor");
  EXPECT_NE(directional_call.find("per_frame_descriptor_sets_[current_frame_index]"), std::string::npos);
  EXPECT_EQ(directional_call.find("raster_material_per_frame_descriptor_sets_[current_frame_index]"),
            std::string::npos);
}

TEST(GltfRasterMaterial, RasterLightingPassesUseFixedGlobalTextureDescriptors) {
  const auto lighting_shader = ReadTextFile(ShaderPath("Includes/Lighting.slangh"));
  const auto deferred_lighting_shader =
      ReadTextFile(ShaderPath("Graphics/Fragment/Standard/StandardDeferredLighting.slang"));
  const auto scene_camera_lighting_shader =
      ReadTextFile(ShaderPath("Graphics/Fragment/Standard/StandardDeferredLightingSceneCamera.slang"));
  const auto transparent_lighting_shader =
      ReadTextFile(ShaderPath("Graphics/Fragment/Standard/StandardTransparent.slang"));
  const auto render_info_shader = ReadTextFile(ShaderPath("Includes/RenderInfo.slangh"));
  const auto render_layer_header = ReadTextFile(SdkPath("include/Layers/RenderLayer.hpp"));
  const auto render_layer = ReadTextFile(SdkPath("src/RenderLayer.cpp"));
  const auto render_instance_storage = ReadTextFile(SdkPath("src/RenderInstanceStorage.cpp"));
  const auto deferred_header = ReadTextFile(SdkPath("include/Rendering/RenderPasses/DeferredLightingPass.hpp"));
  const auto deferred = ReadTextFile(SdkPath("src/RenderPasses/DeferredLightingPass.cpp"));
  const auto transparent_header = ReadTextFile(SdkPath("include/Rendering/RenderPasses/TransparentGeometryPass.hpp"));
  const auto transparent = ReadTextFile(SdkPath("src/RenderPasses/TransparentGeometryPass.cpp"));
  ASSERT_FALSE(lighting_shader.empty());
  ASSERT_FALSE(deferred_lighting_shader.empty());
  ASSERT_FALSE(scene_camera_lighting_shader.empty());
  ASSERT_FALSE(transparent_lighting_shader.empty());
  ASSERT_FALSE(render_info_shader.empty());
  ASSERT_FALSE(render_layer_header.empty());
  ASSERT_FALSE(render_layer.empty());
  ASSERT_FALSE(render_instance_storage.empty());
  ASSERT_FALSE(deferred_header.empty());
  ASSERT_FALSE(deferred.empty());
  ASSERT_FALSE(transparent_header.empty());
  ASSERT_FALSE(transparent.empty());

  EXPECT_NE(lighting_shader.find("#ifdef EE_RASTER_FIXED_LIGHTING_TEXTURES"), std::string::npos);
  EXPECT_NE(lighting_shader.find("EE_RASTER_BRDF_LUT"), std::string::npos);
  EXPECT_NE(lighting_shader.find("EE_RASTER_SKYBOX"), std::string::npos);
  EXPECT_NE(lighting_shader.find("EE_RASTER_IRRADIANCE_MAP"), std::string::npos);
  EXPECT_NE(lighting_shader.find("EE_RASTER_PREFILTERED_MAP"), std::string::npos);
  EXPECT_NE(lighting_shader.find("EE_CUBEMAPS[camera.skybox_tex_index]"), std::string::npos);
  EXPECT_NE(lighting_shader.find("EE_TEXTURE_2DS[EE_RENDER_INFO.brdf_lut_map_index]"), std::string::npos);
  EXPECT_NE(render_info_shader.find("const uint EE_REFLECTION_PROBE_MAX_COUNT = 32u"), std::string::npos);
  EXPECT_NE(lighting_shader.find("binding = 5"), std::string::npos);
  EXPECT_NE(lighting_shader.find("EE_RASTER_REFLECTION_PROBES[EE_REFLECTION_PROBE_MAX_COUNT]"), std::string::npos);
  const auto environmental_components =
      ExtractSourceRange(lighting_shader, "EeEnvironmentalLighting EE_FUNC_CALCULATE_ENVIRONMENTAL_COMPONENTS",
                         "vec3 EE_FUNC_CALCULATE_ENVIRONMENTAL_LIGHT");
  EXPECT_EQ(CountOccurrences(environmental_components, "vec3(1.0f / EE_ENVIRONMENT.gamma)"), 4u);
  EXPECT_NE(lighting_shader.find("case 31:"), std::string::npos);
  EXPECT_NE(lighting_shader.find("EE_REFLECTION_PROBE_INFLUENCE"), std::string::npos);
  EXPECT_NE(lighting_shader.find("EE_REFLECTION_PROBE_DIRECTION"), std::string::npos);
  EXPECT_NE(lighting_shader.find("any(greaterThan(abs(localPosition), projectionExtents))"), std::string::npos);
  EXPECT_NE(lighting_shader.find("primaryWeight < 1.0f"), std::string::npos);
  EXPECT_NE(lighting_shader.find("lighting_parameters.y >="), std::string::npos);
  EXPECT_NE(lighting_shader.find("globalWeight = (1.0f - primaryWeight) * (1.0f - secondaryRawWeight)"),
            std::string::npos);
  EXPECT_NE(lighting_shader.find("result += globalPrefiltered * primaryWeight"), std::string::npos);
  EXPECT_NE(lighting_shader.find("EE_SPATIAL_REFLECTION_PREFILTERED(fragPos, R, roughness, globalPrefiltered)"),
            std::string::npos);
  EXPECT_NE(environmental_components.find("EE_ENVIRONMENT.diffuse_fallback_intensity"), std::string::npos);
  EXPECT_NE(environmental_components.find("EE_ENVIRONMENT.specular_fallback_intensity"), std::string::npos);
  EXPECT_EQ(environmental_components.find("diffuse_environment_intensity"), std::string::npos);
  EXPECT_EQ(environmental_components.find("specular_reflection_intensity"), std::string::npos);
  EXPECT_NE(lighting_shader.find(
                "const float indirectVisibility = clamp(materialOcclusion * screenSpaceVisibility, 0.0f, 1.0f)"),
            std::string::npos);
  EXPECT_NE(lighting_shader.find("const vec3 diffuseIndirect = diffuse * indirectVisibility *"), std::string::npos);
  EXPECT_NE(lighting_shader.find("float EE_REFLECTION_PROBE_SCALAR_VISIBILITY"), std::string::npos);
  EXPECT_NE(
      lighting_shader.find("float EE_ROUGH_SPECULAR_VISIBILITY(float materialOcclusion, float screenSpaceVisibility, "
                           "float ddgiVisibility"),
      std::string::npos);
  EXPECT_NE(lighting_shader.find("EE_REFLECTION_PROBE_SCALAR_VISIBILITY(materialOcclusion, screenSpaceVisibility, "
                                 "ddgiVisibility)"),
            std::string::npos);
  EXPECT_NE(environmental_components.find("EE_ROUGH_SPECULAR_VISIBILITY"), std::string::npos);
  EXPECT_NE(environmental_components.find("result.unoccluded_specular"), std::string::npos);
  EXPECT_NE(environmental_components.find("result.specular = result.unoccluded_specular * result.specular_visibility"),
            std::string::npos);
  EXPECT_EQ(environmental_components.find("result.diffuse_lighting"), std::string::npos);
  EXPECT_EQ(lighting_shader.find("float EE_REFLECTION_LIGHTING_SCALE"), std::string::npos);
  EXPECT_EQ(lighting_shader.find("reflectionLightingScale"), std::string::npos);
  EXPECT_NE(lighting_shader.find("const float ddgiSpecularVisibility = "
                                 "mix(1.0f, EE_DDGI_GATHER_VISIBILITY(gather), gather_weight)"),
            std::string::npos);
  EXPECT_NE(lighting_shader.find("const vec3 specular = environment.specular"), std::string::npos);
  EXPECT_NE(deferred_lighting_shader.find("screenSpaceVisibility)"), std::string::npos);
  EXPECT_NE(scene_camera_lighting_shader.find("screenSpaceVisibility)"), std::string::npos);
  EXPECT_NE(transparent_lighting_shader.find("1.0f)"), std::string::npos);
  EXPECT_EQ(deferred_lighting_shader.find("screenSpaceVisibility, result)"), std::string::npos);
  EXPECT_EQ(scene_camera_lighting_shader.find("screenSpaceVisibility, direct)"), std::string::npos);
  EXPECT_EQ(transparent_lighting_shader.find("1.0f, direct)"), std::string::npos);

  EXPECT_NE(render_layer_header.find("raster_lighting_texture_layout_"), std::string::npos);
  EXPECT_NE(render_layer_header.find("raster_lighting_texture_descriptor_sets_"), std::string::npos);
  EXPECT_NE(render_layer_header.find("GetRasterLightingTextureDescriptorSet"), std::string::npos);
  EXPECT_NE(render_layer.find("CreateRasterFixedLightingShaderDefines(3)"), std::string::npos);
  EXPECT_NE(render_layer.find("CreateRasterMaterialFixedLightingShaderDefines(4)"), std::string::npos);
  EXPECT_NE(render_layer.find("raster_lighting_texture_layout_->PushDescriptorBinding"), std::string::npos);
  EXPECT_NE(render_layer.find("TextureStorage::TryGetCubemapDescriptorImageInfo"), std::string::npos);
  EXPECT_NE(render_layer.find("kRasterLightingBrdfLutBinding"), std::string::npos);
  EXPECT_NE(render_layer.find("kRasterLightingSkyboxBinding"), std::string::npos);
  EXPECT_NE(render_layer.find("kRasterLightingIrradianceBinding"), std::string::npos);
  EXPECT_NE(render_layer.find("kRasterLightingPrefilteredBinding"), std::string::npos);
  EXPECT_NE(render_layer.find("kRasterLightingReflectionProbesBinding = 5"), std::string::npos);
  EXPECT_NE(render_layer.find("kRasterLightingDescriptorSamplerCount = 37"), std::string::npos);
  EXPECT_NE(render_layer.find("kRasterLightingMaxPerStageSamplerCount = 64"), std::string::npos);
  EXPECT_NE(render_layer.find("maxDescriptorSetSampledImages < kRasterLightingDescriptorSamplerCount"),
            std::string::npos);
  EXPECT_NE(render_layer.find("RenderInstanceStorage::kReflectionProbeMaxCount"), std::string::npos);
  EXPECT_NE(render_layer.find("image_info = global_prefiltered_info"), std::string::npos);

  EXPECT_NE(render_instance_storage.find("ResolveEnvironmentalLighting(target_scene)"), std::string::npos);
  EXPECT_NE(render_instance_storage.find("resolved_lighting.local_reflection_probes"), std::string::npos);
  EXPECT_NE(render_instance_storage.find("std::min(resolved_lighting.local_reflection_probes.size()"),
            std::string::npos);
  EXPECT_NE(render_instance_storage.find("const auto& probe = resolved_lighting.local_reflection_probes[index]"),
            std::string::npos);
  EXPECT_NE(render_instance_storage.find("info.lighting_parameters = glm::vec4"), std::string::npos);
  EXPECT_NE(render_instance_storage.find("static_cast<float>(probe.artist_priority)"), std::string::npos);
  EXPECT_NE(render_instance_storage.find("asset && asset->IsRuntimeReady()"), std::string::npos);
  EXPECT_NE(render_instance_storage.find("auto probe_payload_ref = probe.global_reflection_probe"), std::string::npos);
  EXPECT_NE(render_instance_storage.find("info.world_to_probe = glm::inverse(probe.transform)"), std::string::npos);
  EXPECT_NE(lighting_shader.find("inverse(mat3(probe.world_to_probe)) * localHit"), std::string::npos);
  const std::string probe_collection =
      ExtractSourceRange(render_instance_storage, "void RenderInstanceStorage::CollectReflectionProbes",
                         "void RenderInstanceStorage::UpdateTopLevelAccelerationStructure");
  EXPECT_EQ(probe_collection.find("GetPrivateComponentOwnersList<ReflectionProbe>"), std::string::npos);
  EXPECT_EQ(probe_collection.find("GetSanitizedSettings("), std::string::npos);
  EXPECT_EQ(probe_collection.find("SetEnabled("), std::string::npos);
  EXPECT_EQ(probe_collection.find("ClampSettings("), std::string::npos);
  EXPECT_EQ(probe_collection.find("EnforceSceneLimit("), std::string::npos);

  const std::string deferred_pipeline = ExtractSourceRange(render_layer, "if (!deferred_lighting_pass_pipeline)",
                                                           "deferred_lighting_pass_pipeline->depth_attachment_format");
  EXPECT_NE(deferred_pipeline.find("CreateRasterFixedLightingShaderDefines(3)"), std::string::npos);
  EXPECT_NE(deferred_pipeline.find("raster_material_per_frame_layout_"), std::string::npos);
  EXPECT_NE(deferred_pipeline.find("lighting_layout_"), std::string::npos);
  EXPECT_NE(deferred_pipeline.find("raster_lighting_texture_layout_"), std::string::npos);

  const std::string scene_camera_pipeline =
      ExtractSourceRange(render_layer, "if (!deferred_lighting_pass_pipeline_scene_camera)",
                         "deferred_lighting_pass_pipeline_scene_camera->depth_attachment_format");
  EXPECT_NE(scene_camera_pipeline.find("CreateRasterFixedLightingShaderDefines(3)"), std::string::npos);
  EXPECT_NE(scene_camera_pipeline.find("raster_material_per_frame_layout_"), std::string::npos);
  EXPECT_NE(scene_camera_pipeline.find("raster_lighting_texture_layout_"), std::string::npos);

  EXPECT_NE(deferred_header.find("raster_lighting_texture_descriptor_set"), std::string::npos);
  EXPECT_NE(deferred.find("parameters.raster_lighting_texture_descriptor_set"), std::string::npos);
  EXPECT_NE(deferred.find("BindDescriptorSet(vk_command_buffer, 3"), std::string::npos);
  EXPECT_NE(transparent_header.find("raster_lighting_texture_descriptor_set"), std::string::npos);
  EXPECT_NE(transparent.find("parameters.raster_lighting_texture_descriptor_set"), std::string::npos);
  EXPECT_NE(transparent.find("vk_command_buffer, 4, parameters.raster_lighting_texture_descriptor_set"),
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
  EXPECT_EQ(offscreen_preview.find("scene->environment."), std::string::npos);
  EXPECT_NE(offscreen_preview.find("lighting->ddgi_settings.runtime.enabled = false"), std::string::npos);
  EXPECT_NE(offscreen_preview.find("UploadPreviewResources(mesh, material)"), std::string::npos);
  EXPECT_NE(offscreen_preview.find("camera->camera_render_mode = Camera::CameraRenderMode::Rasterization"),
            std::string::npos);
  EXPECT_NE(offscreen_preview.find("render_layer->RenderSceneToCameraImmediately(scene, camera_transform, camera)"),
            std::string::npos);
  EXPECT_EQ(offscreen_preview.find("GltfRasterMaterial.slangh"), std::string::npos);
  EXPECT_EQ(offscreen_preview.find("StandardDeferred.slang"), std::string::npos);
  EXPECT_EQ(offscreen_preview.find("StandardTransparent.slang"), std::string::npos);
  EXPECT_EQ(offscreen_preview.find("ShadowMapPassThrough.slang"), std::string::npos);
  EXPECT_EQ(offscreen_preview.find("Shader::CreateTemporary"), std::string::npos);

  const std::string immediate_render = ExtractSourceRange(
      render_layer, "void RenderLayer::RenderSceneToCameraImmediately", "void RenderLayer::RenderAll");
  EXPECT_NE(immediate_render.find("std::make_shared<RenderInstanceStorage>()"), std::string::npos);
  EXPECT_NE(immediate_render.find("PrepareSceneForRendering(scene, false, false, false, false,"), std::string::npos);
  EXPECT_NE(immediate_render.find("reflection_probe_capture ? &injected_camera : nullptr"), std::string::npos);
  EXPECT_NE(immediate_render.find("!reflection_probe_capture"), std::string::npos);
  EXPECT_NE(
      immediate_render.find("RenderToCamera(scene, camera_global_transform, camera, true, reflection_probe_capture)"),
      std::string::npos);
  EXPECT_NE(immediate_render.find("BindRenderInstanceStorage(current_frame_index, previous_render_instances)"),
            std::string::npos);
}

TEST(GltfRasterMaterial, FixedRasterBackendUsesIndividualTextureBindings) {
  const auto source = ReadTextFile(ShaderPath("Includes/GltfRasterMaterial.slangh"));
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
  EXPECT_NE(source.find("material.clearcoat_texture, EE_GLTF_RASTER_TEXTURE_CLEARCOAT"), std::string::npos);
  EXPECT_NE(source.find("material.clearcoat_roughness_texture, EE_GLTF_RASTER_TEXTURE_CLEARCOAT_ROUGHNESS"),
            std::string::npos);
  EXPECT_NE(source.find("material.clearcoat_normal_texture, EE_GLTF_RASTER_TEXTURE_CLEARCOAT_NORMAL"),
            std::string::npos);
  EXPECT_NE(source.find("normal_vector.xy *= material.clearcoat_normal_texture_scale"), std::string::npos);
  EXPECT_NE(source.find("EE_GLTF_RASTER_COATED_EMISSION"), std::string::npos);
  EXPECT_NE(source.find("EE_GLTF_RASTER_TEXTURE_UNSUPPORTED_EXTENSION"), std::string::npos);
}

TEST(GltfRasterMaterial, ActiveRasterShadersUseGltfEvaluator) {
  const std::filesystem::path paths[] = {
      ShaderPath("Graphics/Fragment/Standard/StandardDeferred.slang"),
      ShaderPath("Graphics/Fragment/Standard/StandardTransparent.slang"),
  };

  for (const auto& path : paths) {
    const auto source = ReadTextFile(path);
    ASSERT_FALSE(source.empty()) << path.string();
    EXPECT_NE(source.find("#include \"GltfRasterMaterial.slangh\""), std::string::npos) << path.string();
    EXPECT_NE(source.find("EE_EVALUATE_GLTF_RASTER_SURFACE"), std::string::npos) << path.string();
    EXPECT_EQ(source.find(std::string("EE_MATERIAL") + "_PROPERTIES"), std::string::npos) << path.string();
    EXPECT_EQ(source.find(std::string("Material") + "Properties"), std::string::npos) << path.string();
  }
}

TEST(GltfRasterMaterial, PostProcessConsumersReadExpandedGBuffer) {
  const std::filesystem::path normal_paths[] = {
      ShaderPath("Compute/PostProcessing/SSRReflect.slang"),
      ShaderPath("Compute/PostProcessing/AmbientOcclusionGeometry.slang"),
      ShaderPath("Graphics/Fragment/PostProcessing/SSRReflect.slang"),
  };
  for (const auto& path : normal_paths) {
    const auto source = ReadTextFile(path);
    ASSERT_FALSE(source.empty()) << path.string();
    EXPECT_NE(source.find("binding = 21) uniform sampler2D inNormalRoughness"), std::string::npos) << path.string();
    EXPECT_NE(source.find("texture(inNormalRoughness"), std::string::npos) << path.string();
    EXPECT_EQ(source.find("uniform sampler2D inMaterial"), std::string::npos) << path.string();
  }

  const std::filesystem::path combine_paths[] = {
      ShaderPath("Compute/PostProcessing/SSRCombine.slang"),
      ShaderPath("Graphics/Fragment/PostProcessing/SSRCombine.slang"),
  };
  for (const auto& path : combine_paths) {
    const auto source = ReadTextFile(path);
    ASSERT_FALSE(source.empty()) << path.string();
    EXPECT_EQ(source.find("#include \"GltfRasterMaterial.slangh\""), std::string::npos) << path.string();
    EXPECT_EQ(source.find("EE_EVALUATE_GLTF_RASTER_SURFACE"), std::string::npos) << path.string();
    EXPECT_NE(source.find("binding = 21) uniform sampler2D inNormalRoughness"), std::string::npos) << path.string();
    EXPECT_NE(source.find("binding = 22) uniform sampler2D inPbrFlags"), std::string::npos) << path.string();
    EXPECT_NE(source.find("float roughness = texture(inNormalRoughness, texCoord).a"), std::string::npos)
        << path.string();
    EXPECT_NE(source.find("float metallic = texture(inPbrFlags, texCoord).x"), std::string::npos) << path.string();
  }

  const auto ambient_occlusion_geometry =
      ReadTextFile(ShaderPath("Compute/PostProcessing/AmbientOcclusionGeometry.slang"));
  const auto ambient_occlusion_source = ReadTextFile(SdkPath("src/ScreenSpaceAmbientOcclusion.cpp"));
  ASSERT_FALSE(ambient_occlusion_geometry.empty());
  ASSERT_FALSE(ambient_occlusion_source.empty());
  EXPECT_EQ(std::filesystem::exists(ShaderPath("Compute/PostProcessing/AmbientOcclusionCombine.slang")), false);
  EXPECT_EQ(ambient_occlusion_geometry.find("inColor"), std::string::npos);
  EXPECT_EQ(ambient_occlusion_geometry.find("outSrcColor"), std::string::npos);
  EXPECT_EQ(ambient_occlusion_source.find("EverythingBarrier"), std::string::npos);

  const auto compose_lighting = [](const float direct, const float emission, const float indirect_diffuse,
                                   const float specular, const float material_ao, const float screen_ao,
                                   const float roughness) {
    const float visibility = evo_engine::EnvironmentalLighting::EvaluateRoughSpecularVisibility(material_ao, screen_ao,
                                                                                                1.0f, roughness, 1.0f);
    return direct + emission + indirect_diffuse * glm::clamp(material_ao * screen_ao, 0.0f, 1.0f) +
           specular * visibility;
  };
  EXPECT_FLOAT_EQ(compose_lighting(2.0f, 3.0f, 4.0f, 5.0f, 1.0f, 0.0f, 1.0f), 5.0f);
  EXPECT_FLOAT_EQ(compose_lighting(2.0f, 3.0f, 4.0f, 5.0f, 1.0f, 0.25f, 1.0f), 7.25f);
  EXPECT_FLOAT_EQ(compose_lighting(0.0f, 0.0f, 0.0f, 5.0f, 1.0f, 0.0f, 0.0f), 5.0f);
}

TEST(GltfRasterMaterial, DeferredPassesReadAndWriteExpandedGBuffer) {
  const std::filesystem::path paths[] = {
      ShaderPath("Graphics/Fragment/Standard/StandardDeferredLighting.slang"),
      ShaderPath("Graphics/Fragment/Standard/StandardDeferredLightingSceneCamera.slang"),
  };

  for (const auto& path : paths) {
    const auto source = ReadTextFile(path);
    ASSERT_FALSE(source.empty()) << path.string();
    EXPECT_EQ(source.find("#include \"GltfRasterMaterial.slangh\""), std::string::npos) << path.string();
    EXPECT_EQ(source.find("EE_EVALUATE_GLTF_RASTER_SURFACE"), std::string::npos) << path.string();
    EXPECT_NE(source.find("binding = 20) uniform sampler2D inBaseColorAO"), std::string::npos) << path.string();
    EXPECT_NE(source.find("binding = 21) uniform sampler2D inNormalRoughness"), std::string::npos) << path.string();
    EXPECT_NE(source.find("binding = 22) uniform sampler2D inPbrFlags"), std::string::npos) << path.string();
    EXPECT_NE(source.find("binding = 23) uniform sampler2D inEmissive"), std::string::npos) << path.string();
    EXPECT_NE(source.find("binding = 4) uniform sampler2D inAmbientOcclusion"), std::string::npos) << path.string();
    EXPECT_NE(source.find("float roughness = normalRoughness.a"), std::string::npos) << path.string();
    EXPECT_NE(source.find("float metallic = pbrFlags.x"), std::string::npos) << path.string();
    EXPECT_NE(source.find("vec3 F0 = pbrFlags.yzw"), std::string::npos) << path.string();
    EXPECT_NE(source.find("float materialOcclusion = baseColorAO.a"), std::string::npos) << path.string();
    EXPECT_NE(source.find("float screenSpaceVisibility = texture(inAmbientOcclusion, fs_in.TexCoord).r"),
              std::string::npos)
        << path.string();
    EXPECT_NE(source.find("vec4 albedo = vec4(baseColorAO.rgb, 1.0)"), std::string::npos) << path.string();
  }

  const auto deferred = ReadTextFile(ShaderPath("Graphics/Fragment/Standard/StandardDeferred.slang"));
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
  EXPECT_NE(deferred.find("outGBufferPbrFlags = vec4(surface.metallic, surface.specular_f0)"), std::string::npos);
  EXPECT_NE(deferred.find("float encoded_specular_f90"), std::string::npos);
  EXPECT_NE(deferred.find("outGBufferEmissive = vec4(coated_emissive, encoded_specular_f90)"), std::string::npos);
  EXPECT_NE(deferred.find("outGBufferUtility = vec4(float(instance_index), float(instance.info_index), "
                          "float(instance.material_index), 0.0)"),
            std::string::npos);
}

TEST(GltfRasterMaterial, DeferredGBufferUsesCurrentBindings) {
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
    EXPECT_TRUE(HasFragmentOutputLocation(source, 0, "outGBufferBaseColorAO")) << path.string();
    EXPECT_TRUE(HasFragmentOutputLocation(source, 1, "outGBufferNormalRoughness")) << path.string();
    EXPECT_TRUE(HasFragmentOutputLocation(source, 2, "outGBufferPbrFlags")) << path.string();
    EXPECT_TRUE(HasFragmentOutputLocation(source, 3, "outGBufferEmissive")) << path.string();
    EXPECT_TRUE(HasFragmentOutputLocation(source, 4, "outGBufferUtility")) << path.string();
    EXPECT_NE(source.find("EE_GLTF_RASTER_REBASE_SPECULAR_F0"), std::string::npos) << path.string();
    EXPECT_NE(source.find("EE_GLTF_RASTER_COATED_EMISSION"), std::string::npos) << path.string();
    EXPECT_NE(source.find("encoded_specular_f90"), std::string::npos) << path.string();
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
  const auto standard = ReadTextFile(ShaderPath("Graphics/Vertex/Standard/Standard.slang"));
  const auto standard_instanced = ReadTextFile(ShaderPath("Graphics/Vertex/Standard/StandardInstanced.slang"));
  const auto standard_skinned = ReadTextFile(ShaderPath("Graphics/Vertex/Standard/StandardSkinned.slang"));
  const auto standard_mesh = ReadTextFile(ShaderPath("Graphics/Mesh/Standard/Standard.slang"));
  const auto standard_meshlet_colored = ReadTextFile(ShaderPath("Graphics/Mesh/Standard/StandardMeshletColored.slang"));
  const auto standard_strands = ReadTextFile(ShaderPath("Graphics/Mesh/Standard/StandardStrands.slang"));
  const auto instances = ReadTextFile(ShaderPath("Includes/Instances.slangh"));
  const auto deferred = ReadTextFile(ShaderPath("Graphics/Fragment/Standard/StandardDeferred.slang"));
  const auto transparent = ReadTextFile(ShaderPath("Graphics/Fragment/Standard/StandardTransparent.slang"));
  const auto raster_material = ReadTextFile(ShaderPath("Includes/GltfRasterMaterial.slangh"));
  const auto render_instance_storage = ReadTextFile(SdkPath("src/RenderInstanceStorage.cpp"));
  const auto inspection_adapters = ReadTextFile(SdkPath("src/Editor/SDKInspectionAdapters.cpp"));
  const auto scots_pine = ReadTextFile(RepoPath("EvoEngine_Packages/LSystem/src/ScotsPine.cpp"));
  const auto soil = ReadTextFile(RepoPath("EvoEngine_Packages/EcoSysLab/src/Soil.cpp"));

  ASSERT_FALSE(geometry.empty());
  ASSERT_FALSE(standard.empty());
  ASSERT_FALSE(standard_instanced.empty());
  ASSERT_FALSE(standard_skinned.empty());
  ASSERT_FALSE(standard_mesh.empty());
  ASSERT_FALSE(standard_meshlet_colored.empty());
  ASSERT_FALSE(standard_strands.empty());
  ASSERT_FALSE(instances.empty());
  ASSERT_FALSE(deferred.empty());
  ASSERT_FALSE(transparent.empty());
  ASSERT_FALSE(raster_material.empty());
  ASSERT_FALSE(render_instance_storage.empty());
  ASSERT_FALSE(inspection_adapters.empty());
  ASSERT_FALSE(scots_pine.empty());
  ASSERT_FALSE(soil.empty());

  EXPECT_NE(geometry.find("mesh[5].location = 9"), std::string::npos);
  EXPECT_NE(geometry.find("mesh[5].format = VK_FORMAT_R32_SFLOAT"), std::string::npos);
  EXPECT_NE(geometry.find("mesh[5].offset = offsetof(Vertex, vertex_info3)"), std::string::npos);
  EXPECT_NE(geometry.find("skinned_mesh[9].location = 9"), std::string::npos);
  EXPECT_NE(geometry.find("skinned_mesh[9].format = VK_FORMAT_R32_SFLOAT"), std::string::npos);
  EXPECT_NE(geometry.find("skinned_mesh[9].offset = offsetof(SkinnedVertex, vertex_info3)"), std::string::npos);
  EXPECT_NE(geometry.find("mesh[6].location = 10"), std::string::npos);
  EXPECT_NE(geometry.find("mesh[6].offset = offsetof(Vertex, tex_coord_1)"), std::string::npos);
  EXPECT_NE(geometry.find("skinned_mesh[10].location = 10"), std::string::npos);
  EXPECT_NE(geometry.find("skinned_mesh[10].offset = offsetof(SkinnedVertex, tex_coord_1)"), std::string::npos);
  EXPECT_NE(geometry.find("mesh[7].location = 11"), std::string::npos);
  EXPECT_NE(geometry.find("mesh[7].offset = offsetof(Vertex, tex_coord_2)"), std::string::npos);
  EXPECT_NE(geometry.find("mesh[8].location = 12"), std::string::npos);
  EXPECT_NE(geometry.find("mesh[8].offset = offsetof(Vertex, tex_coord_3)"), std::string::npos);
  EXPECT_NE(geometry.find("skinned_mesh[11].location = 11"), std::string::npos);
  EXPECT_NE(geometry.find("skinned_mesh[12].location = 12"), std::string::npos);

  for (const auto* source : {&standard, &standard_instanced, &standard_skinned}) {
    EXPECT_NE(source->find("layout (location = 9) in float inTangentHandedness"), std::string::npos);
    EXPECT_NE(source->find("flat float TangentHandedness"), std::string::npos);
    EXPECT_NE(source->find("layout (location = 10) in vec2 inTexCoord1"), std::string::npos);
    EXPECT_NE(source->find("layout (location = 11) in vec2 inTexCoord2"), std::string::npos);
    EXPECT_NE(source->find("layout (location = 12) in vec2 inTexCoord3"), std::string::npos);
    EXPECT_NE(source->find("transpose(inverse(mat3("), std::string::npos);
    EXPECT_NE(source->find("vs_out.TexCoord01 = vec4(inTexCoord, inTexCoord1)"), std::string::npos);
    EXPECT_NE(source->find("vs_out.TexCoord23 = vec4(inTexCoord2, inTexCoord3)"), std::string::npos);
    EXPECT_NE(source->find("vs_out.Color = inColor"), std::string::npos);
    EXPECT_NE(source->find("transformHandedness = EE_TRANSFORM_HANDEDNESS("), std::string::npos);
    EXPECT_NE(source->find("* transformHandedness"), std::string::npos);
    EXPECT_EQ(source->find("T = normalize(T - dot(T, N) * N)"), std::string::npos);
  }

  for (const auto* source : {&standard_mesh, &standard_meshlet_colored}) {
    EXPECT_NE(source->find("[[vk::location(3)]]"), std::string::npos);
    EXPECT_NE(source->find("nointerpolation float TangentHandedness"), std::string::npos);
    EXPECT_NE(source->find("transpose(inverse(mat3(model)))"), std::string::npos);
    EXPECT_NE(source->find("vertices[vert].TexCoord01 = vec4(v.tex_coord, v.tex_coord_1)"), std::string::npos);
    EXPECT_NE(source->find("vertices[vert].TexCoord23 = vec4(v.tex_coord_2, v.tex_coord_3)"), std::string::npos);
    EXPECT_NE(source->find("vertices[vert].Color = v.color"), std::string::npos);
    EXPECT_NE(source->find("float handedness = EE_TRANSFORM_HANDEDNESS(model)"), std::string::npos);
    EXPECT_NE(source->find("vertices[vert].transformHandedness = handedness"), std::string::npos);
    EXPECT_NE(source->find("* handedness"), std::string::npos);
    EXPECT_NE(source->find("vec3 fragPos = vec3(model * vec4(v.position.xyz, 1.0))"), std::string::npos);
    EXPECT_NE(source->find("vec4(fragPos, 1.0)"), std::string::npos);
    EXPECT_EQ(source->find("* transformHandedness[vert]"), std::string::npos);
    EXPECT_EQ(source->find("vec4(ms_v_out[vert].FragPos"), std::string::npos);
    EXPECT_EQ(source->find("T = normalize(T - dot(T, N) * N)"), std::string::npos);
  }

  EXPECT_NE(deferred.find("flat float TangentHandedness"), std::string::npos);
  EXPECT_NE(deferred.find("fs_in.TangentHandedness"), std::string::npos);
  EXPECT_NE(deferred.find("fs_in.TexCoord01.zw"), std::string::npos);
  EXPECT_NE(deferred.find("fs_in.TexCoord23.xy"), std::string::npos);
  EXPECT_NE(deferred.find("fs_in.TexCoord23.zw"), std::string::npos);
  EXPECT_NE(deferred.find("fs_in.Color"), std::string::npos);
  EXPECT_EQ(deferred.find("EE_EVALUATE_GLTF_RASTER_NORMAL(material_index, tex_coord, tex_coord, fs_in.Normal, "
                          "fs_in.Tangent)"),
            std::string::npos);
  EXPECT_NE(raster_material.find("EE_GLTF_SAFE_NORMALIZE"), std::string::npos);
  EXPECT_NE(raster_material.find("tangent - n * dot(n, tangent)"), std::string::npos);
  EXPECT_NE(raster_material.find("EE_GLTF_FALLBACK_TANGENT(n)"), std::string::npos);
  EXPECT_NE(raster_material.find("vec3 b = cross(n, t) * bitangent_sign"), std::string::npos);
  EXPECT_NE(standard_strands.find("transpose(inverse(mat3(model)))"), std::string::npos);
  EXPECT_NE(standard_strands.find("vertices[vertex].TangentHandedness = handedness"), std::string::npos);
  EXPECT_NE(standard_strands.find("vertices[vertex].TexCoord01 = vec4(tex_coord, tex_coord)"), std::string::npos);
  EXPECT_NE(standard_strands.find("vertices[vertex].TexCoord23 = vec4(0.0)"), std::string::npos);
  for (const auto* source : {&deferred, &transparent}) {
    EXPECT_NE(source->find("layout(location = 8) in flat float transformHandedness"), std::string::npos);
    EXPECT_NE(source->find("(gl_FrontFacing ? 1.0 : -1.0) * transformHandedness"), std::string::npos);
    EXPECT_NE(source->find("EE_GLTF_MATERIALS[material_index].double_sided == 0 && facing_sign < 0.0"),
              std::string::npos);
  }
  EXPECT_NE(instances.find("float EE_TRANSFORM_HANDEDNESS(mat4 transform)"), std::string::npos);
  EXPECT_NE(instances.find("determinant(mat3(transform)) < 0.0 ? -1.0 : 1.0"), std::string::npos);
  EXPECT_NE(render_instance_storage.find("VkCullModeFlags ResolveCullModeForTransform"), std::string::npos);
  EXPECT_NE(render_instance_storage.find("VkCullModeFlags ResolveInstancedCullModeForTransforms"), std::string::npos);
  EXPECT_NE(render_instance_storage.find("has_positive_determinant && has_negative_determinant"), std::string::npos);
  EXPECT_NE(render_instance_storage.find("return VK_CULL_MODE_NONE"), std::string::npos);
  EXPECT_NE(render_instance_storage.find("return VK_CULL_MODE_FRONT_BIT"), std::string::npos);
  EXPECT_NE(render_instance_storage.find("return VK_CULL_MODE_BACK_BIT"), std::string::npos);
  EXPECT_NE(render_instance_storage.find("render_instance->cull_mode = ResolveCullModeForTransform"),
            std::string::npos);
  EXPECT_EQ(render_instance_storage.find("render_instance->cull_mode = material->draw_settings.cull_mode;"),
            std::string::npos);
  EXPECT_NE(inspection_adapters.find("InspectDrawSettings(material.draw_settings, false)"), std::string::npos);
  EXPECT_NE(scots_pine.find("needle_material->material_data.shade_material.double_sided = 1"), std::string::npos);
  EXPECT_EQ(CountOccurrences(soil, "material->material_data.shade_material.double_sided = 1"), 2);
}

TEST(GltfRasterMaterial, LightweightPipelinesUseCompactVertexInputLayouts) {
  const auto geometry_header = ReadTextFile(SdkPath("include/Rendering/Geometry/IGeometry.hpp"));
  const auto geometry_source = ReadTextFile(SdkPath("src/IGeometry.cpp"));
  const auto pipeline_header = ReadTextFile(SdkPath("include/Rendering/Platform/GraphicsPipeline.hpp"));
  const auto pipeline_source = ReadTextFile(SdkPath("src/GraphicsPipeline.cpp"));
  const auto platform = ReadTextFile(SdkPath("src/Platform.cpp"));
  const auto render_layer = ReadTextFile(SdkPath("src/RenderLayer.cpp"));
  const auto cubemap = ReadTextFile(SdkPath("src/Cubemap.cpp"));
  const auto global_reflection_probe = ReadTextFile(SdkPath("src/GlobalReflectionProbe.cpp"));
  const auto light_probe = ReadTextFile(SdkPath("src/LightProbe.cpp"));

  ASSERT_FALSE(geometry_header.empty());
  ASSERT_FALSE(geometry_source.empty());
  ASSERT_FALSE(pipeline_header.empty());
  ASSERT_FALSE(pipeline_source.empty());
  ASSERT_FALSE(platform.empty());
  ASSERT_FALSE(render_layer.empty());
  ASSERT_FALSE(cubemap.empty());
  ASSERT_FALSE(global_reflection_probe.empty());
  ASSERT_FALSE(light_probe.empty());

  EXPECT_NE(geometry_header.find("enum class VertexInputAttributeSet"), std::string::npos);
  EXPECT_NE(geometry_header.find("VertexInputAttributeSet::Full"), std::string::npos);
  EXPECT_NE(geometry_header.find("PositionTexCoord"), std::string::npos);
  EXPECT_NE(geometry_header.find("MotionVectors"), std::string::npos);
  EXPECT_NE(pipeline_header.find("VertexInputAttributeSet vertex_input_attribute_set"), std::string::npos);
  EXPECT_NE(pipeline_source.find("GetVertexAttributeDescriptions(geometry_type, vertex_input_attribute_set)"),
            std::string::npos);
  EXPECT_NE(geometry_source.find("mesh_base.assign(mesh.begin(), mesh.begin() + 5)"), std::string::npos);
  EXPECT_NE(geometry_source.find("skinned_mesh_base.assign(skinned_mesh.begin(), skinned_mesh.begin() + 9)"),
            std::string::npos);
  EXPECT_NE(geometry_source.find("mesh_position = {mesh[0]}"), std::string::npos);
  EXPECT_NE(geometry_source.find("mesh_position_tex_coord = {mesh[0], mesh[3]}"), std::string::npos);
  EXPECT_NE(geometry_source.find("mesh_motion_vectors = {mesh[0], mesh[3], mesh[4], mesh[6], mesh[7], mesh[8]}"),
            std::string::npos);

  const auto shadow_pipeline =
      ExtractSourceRange(render_layer, "std::shared_ptr<GraphicsPipeline> CreateShadowVertexPipeline",
                         "std::shared_ptr<GraphicsPipeline> CreateStrandShadowMeshPipeline");
  ASSERT_FALSE(shadow_pipeline.empty());
  EXPECT_NE(shadow_pipeline.find("pipeline->vertex_input_attribute_set = VertexInputAttributeSet::Position"),
            std::string::npos);

  EXPECT_NE(platform.find("graphics.render_texture_present_pipeline->vertex_input_attribute_set = "
                          "VertexInputAttributeSet::PositionTexCoord"),
            std::string::npos);
  for (const auto* pipeline : {"deferred_lighting_pass_pipeline", "ddgi_gather_timing_pipeline_",
                               "deferred_lighting_pass_pipeline_scene_camera", "environmental_brdf_pipeline"}) {
    EXPECT_TRUE(ContainsIgnoringWhitespace(render_layer, std::string(pipeline) +
                                                             "->vertex_input_attribute_set = "
                                                             "VertexInputAttributeSet::PositionTexCoord"))
        << pipeline;
  }
  for (const auto* pipeline : {"transparent_motion_vectors_pipeline_", "skinned_motion_vectors_pipeline_"}) {
    EXPECT_TRUE(ContainsIgnoringWhitespace(render_layer, std::string(pipeline) +
                                                             "->vertex_input_attribute_set = "
                                                             "VertexInputAttributeSet::MotionVectors"))
        << pipeline;
  }
  for (const auto* pipeline : {"gizmos", "gizmos_instanced_colored"}) {
    EXPECT_TRUE(ContainsIgnoringWhitespace(render_layer, std::string(pipeline) + "->vertex_input_attribute_set = "
                                                                                 "VertexInputAttributeSet::Position"))
        << pipeline;
  }
  EXPECT_NE(render_layer.find("gizmos_normal_colored->vertex_input_attribute_set = "
                              "VertexInputAttributeSet::PositionNormal"),
            std::string::npos);
  EXPECT_NE(render_layer.find("gizmos_vertex_colored->vertex_input_attribute_set = "
                              "VertexInputAttributeSet::PositionColor"),
            std::string::npos);
  EXPECT_NE(render_layer.find("ddgi_probe_visualization_pipeline_->vertex_input_attribute_set = "
                              "VertexInputAttributeSet::PositionNormal"),
            std::string::npos);
  EXPECT_EQ(CountOccurrences(cubemap, "vertex_input_attribute_set = VertexInputAttributeSet::Position"), 2);
  EXPECT_NE(global_reflection_probe.find("prefilter_construct_pipeline_->vertex_input_attribute_set = "
                                         "VertexInputAttributeSet::Position"),
            std::string::npos);
  EXPECT_NE(light_probe.find("irradiance_construct_pipeline_->vertex_input_attribute_set = "
                             "VertexInputAttributeSet::Position"),
            std::string::npos);

  const std::filesystem::path mesh_lightweight_shaders[] = {
      ShaderPath("Graphics/Vertex/Lighting/PointLightShadowMap.slang"),
      ShaderPath("Graphics/Vertex/Lighting/SpotLightShadowMap.slang"),
      ShaderPath("Graphics/Vertex/Lighting/DirectionalLightShadowMap.slang"),
      ShaderPath("Graphics/Vertex/Lighting/PointLightShadowMapInstanced.slang"),
      ShaderPath("Graphics/Vertex/Lighting/SpotLightShadowMapInstanced.slang"),
      ShaderPath("Graphics/Vertex/Lighting/DirectionalLightShadowMapInstanced.slang"),
      ShaderPath("Graphics/Vertex/Lighting/AtmosphereToCubemap.slang"),
  };
  for (const auto& shader_path : mesh_lightweight_shaders) {
    const auto shader = ReadTextFile(shader_path);
    ASSERT_FALSE(shader.empty()) << shader_path.string();
    ExpectShaderInputLocations(shader, shader_path, {0});
    EXPECT_EQ(shader.find("out VS_OUT"), std::string::npos) << shader_path.string();
    EXPECT_EQ(shader.find("currentInstanceIndex"), std::string::npos) << shader_path.string();
  }
  ExpectSlangInputLocations(ReadTextFile(ShaderPath("Graphics/Vertex/Lighting/CubemapProcess.slang")),
                            ShaderPath("Graphics/Vertex/Lighting/CubemapProcess.slang"), "CubemapProcessVertexInput",
                            {0});

  const std::filesystem::path skinned_shadow_shaders[] = {
      ShaderPath("Graphics/Vertex/Lighting/PointLightShadowMapSkinned.slang"),
      ShaderPath("Graphics/Vertex/Lighting/SpotLightShadowMapSkinned.slang"),
      ShaderPath("Graphics/Vertex/Lighting/DirectionalLightShadowMapSkinned.slang"),
  };
  for (const auto& shader_path : skinned_shadow_shaders) {
    const auto shader = ReadTextFile(shader_path);
    ASSERT_FALSE(shader.empty()) << shader_path.string();
    ExpectShaderInputLocations(shader, shader_path, {0, 5, 6, 7, 8});
    EXPECT_EQ(shader.find("out VS_OUT"), std::string::npos) << shader_path.string();
    EXPECT_EQ(shader.find("currentInstanceIndex"), std::string::npos) << shader_path.string();
  }

  ExpectSlangInputLocations(ReadTextFile(ShaderPath("Graphics/Vertex/TexturePassThrough.slang")),
                            ShaderPath("Graphics/Vertex/TexturePassThrough.slang"), "TexturePassThroughVertexInput",
                            {0, 3});
  ExpectShaderInputLocations(ReadTextFile(ShaderPath("Graphics/Vertex/Gizmos/Gizmos.slang")),
                             ShaderPath("Graphics/Vertex/Gizmos/Gizmos.slang"), {0});
  ExpectShaderInputLocations(ReadTextFile(ShaderPath("Graphics/Vertex/Gizmos/GizmosInstancedColored.slang")),
                             ShaderPath("Graphics/Vertex/Gizmos/GizmosInstancedColored.slang"), {0});
  ExpectShaderInputLocations(ReadTextFile(ShaderPath("Graphics/Vertex/Gizmos/GizmosNormalColored.slang")),
                             ShaderPath("Graphics/Vertex/Gizmos/GizmosNormalColored.slang"), {0, 1});
  ExpectShaderInputLocations(ReadTextFile(ShaderPath("Graphics/Vertex/Gizmos/GizmosVertexColored.slang")),
                             ShaderPath("Graphics/Vertex/Gizmos/GizmosVertexColored.slang"), {0, 4});
  ExpectShaderInputLocations(ReadTextFile(ShaderPath("Graphics/Vertex/DDGI/DDGIProbeVisualization.slang")),
                             ShaderPath("Graphics/Vertex/DDGI/DDGIProbeVisualization.slang"), {0, 1});

  const std::filesystem::path mesh_shadow_shaders[] = {
      ShaderPath("Graphics/Mesh/Lighting/PointLightShadowMap.slang"),
      ShaderPath("Graphics/Mesh/Lighting/SpotLightShadowMap.slang"),
      ShaderPath("Graphics/Mesh/Lighting/DirectionalLightShadowMap.slang"),
  };
  for (const auto& shader_path : mesh_shadow_shaders) {
    const auto shader = ReadTextFile(shader_path);
    ASSERT_FALSE(shader.empty()) << shader_path.string();
    EXPECT_EQ(shader.find("out MS_V_OUT"), std::string::npos) << shader_path.string();
    EXPECT_EQ(shader.find("currentInstanceIndex"), std::string::npos) << shader_path.string();
  }
}

TEST(GltfRasterMaterial, PrefabImporterPreservesMissingTexCoordAttributes) {
  const auto prefab_source = ReadTextFile(SdkPath("src/Prefab.cpp"));
  ASSERT_FALSE(prefab_source.empty());

  EXPECT_NE(prefab_source.find("attributes.tex_coord = false"), std::string::npos);
  EXPECT_NE(prefab_source.find("skinned_vertex_attributes.tex_coord = false"), std::string::npos);
  EXPECT_NE(prefab_source.find("attributes.tex_coord_1 = true"), std::string::npos);
  EXPECT_NE(prefab_source.find("skinned_vertex_attributes.tex_coord_1 = true"), std::string::npos);
  EXPECT_NE(prefab_source.find("attributes.tex_coord_2 = true"), std::string::npos);
  EXPECT_NE(prefab_source.find("attributes.tex_coord_3 = true"), std::string::npos);
  EXPECT_NE(prefab_source.find("skinned_vertex_attributes.tex_coord_2 = true"), std::string::npos);
  EXPECT_NE(prefab_source.find("skinned_vertex_attributes.tex_coord_3 = true"), std::string::npos);
  EXPECT_NE(prefab_source.find("glm::vec4(color.r, color.g, color.b, color.a)"), std::string::npos);
  EXPECT_NE(prefab_source.find("ReadImportedTexCoord(importer_mesh, 1"), std::string::npos);
  EXPECT_NE(prefab_source.find("bool parsed_gltf = false"), std::string::npos);
  EXPECT_NE(prefab_source.find("(extension == \".gltf\" || extension == \".glb\") && parsed_gltf"), std::string::npos);
  EXPECT_EQ(prefab_source.find("extension == \".gltf\" && !gltf_material_data.empty()"), std::string::npos);
}
