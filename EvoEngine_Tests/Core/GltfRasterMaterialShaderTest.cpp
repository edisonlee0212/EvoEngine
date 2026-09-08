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

bool HasSlangLocationAttribute(const std::string& source, const int location) {
  return source.find("[[vk::location(" + std::to_string(location) + ")]]") != std::string::npos;
}

bool HasFragmentOutputLocation(const std::string& source, const int location, const std::string& name) {
  const auto glsl_location = "layout(location = " + std::to_string(location) + ") out vec4 " + name;
  const auto slang_float_field = "float4 " + name + " ";
  const auto slang_uint_field = "uint4 " + name + " ";
  return source.find(glsl_location) != std::string::npos ||
         (HasSlangLocationAttribute(source, location) &&
          (source.find(slang_float_field) != std::string::npos || source.find(slang_uint_field) != std::string::npos));
}

std::string ExtractStructIgnoringWhitespace(const std::string& source, const std::string& name) {
  const auto begin = source.find("struct " + name);
  if (begin == std::string::npos) {
    return {};
  }
  const auto end = source.find("};", begin);
  if (end == std::string::npos) {
    return {};
  }
  return RemoveAsciiWhitespace(source.substr(begin, end + 2 - begin));
}

TEST(GltfRasterMaterial, MeshTaskShadersBoundWorkgroupsToInstanceMeshlets) {
  const std::filesystem::path shader_paths[] = {
      ShaderPath("Graphics/Task/Standard/Standard.slang"),
      ShaderPath("Graphics/Task/Lighting/DirectionalLightShadowMap.slang"),
      ShaderPath("Graphics/Task/Lighting/PointLightShadowMap.slang"),
      ShaderPath("Graphics/Task/Lighting/SpotLightShadowMap.slang"),
  };

  for (const auto& path : shader_paths) {
    const auto source = ReadTextFile(path);
    ASSERT_FALSE(source.empty()) << path.string();
    EXPECT_NE(source.find("uint group_meshlet_offset = group_id.x * EXT_TASK_WORK_GROUP_INVOCATIONS;"),
              std::string::npos)
        << path.string();
    EXPECT_NE(source.find("uint base_id = group_meshlet_offset + meshlet_offset;"), std::string::npos) << path.string();
    EXPECT_NE(source.find("bool render = group_meshlet_offset + meshlet_local < meshlet_count;"), std::string::npos)
        << path.string();
    EXPECT_EQ(source.find("bool render = meshlet_local < meshlet_count;"), std::string::npos) << path.string();
  }
}

void ExpectShaderInputLocations(const std::string& shader, const std::filesystem::path& shader_path,
                                const std::initializer_list<int> expected_locations) {
  const auto struct_begin = shader.find("struct ");
  ASSERT_NE(struct_begin, std::string::npos) << shader_path.string();
  const auto input_block = ExtractSourceRange(shader.substr(struct_begin), "struct ", "};");
  ASSERT_FALSE(input_block.empty()) << shader_path.string();
  for (int location = 0; location <= 12; ++location) {
    const bool expected =
        std::find(expected_locations.begin(), expected_locations.end(), location) != expected_locations.end();
    EXPECT_EQ(HasSlangLocationAttribute(input_block, location), expected)
        << shader_path.string() << " location=" << location;
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
  const auto source = ReadTextFile(ShaderPath("Modules/EvoEngine/GltfBindlessMaterial.slang"));
  ASSERT_FALSE(source.empty());

  EXPECT_NE(source.find("__exported import EvoEngine.GltfMaterial;"), std::string::npos);
  EXPECT_NE(source.find("import EvoEngine.Textures;"), std::string::npos);
  EXPECT_NE(source.find("EE_GLTF_HAS_TEXTURE"), std::string::npos);
  EXPECT_NE(source.find("return uint(texture_info_slot) > 0u"), std::string::npos);
  EXPECT_NE(source.find("EE_GLTF_TEXTURE_INFOS[uint(texture_info_slot)]"), std::string::npos);
  EXPECT_NE(source.find("EE_GLTF_TRANSFORM_UV(texture_info, float3(uv, 1.0))"), std::string::npos);
  EXPECT_EQ(source.find("EE_GLTF_SAMPLE_FIXED_RASTER_TEXTURE"), std::string::npos);
  EXPECT_NE(source.find("EE_TEXTURE_2DS[NonUniformResourceIndex(texture_info.index)]"), std::string::npos);
  EXPECT_EQ(source.find(std::string("Material") + "Properties"), std::string::npos);

  EXPECT_EQ(source.find("SPECULAR_GLOSSINESS"), std::string::npos);
  EXPECT_EQ(source.find("pbr_specular_glossiness_texture"), std::string::npos);
  EXPECT_NE(source.find("pbr_metallic_roughness_texture"), std::string::npos);
  EXPECT_NE(source.find("dielectric_f0 = pow((material_ior - 1.0)"), std::string::npos);
  EXPECT_NE(source.find("specular_weight = material.specular_factor"), std::string::npos);
  EXPECT_NE(source.find("material.ior == 0.0 ? 0.0"), std::string::npos);
  EXPECT_NE(source.find("dielectric_specular_f0 * max(specular_color"), std::string::npos);
  EXPECT_NE(source.find("surface.specular_f0 = lerp(dielectric_specular_f0"), std::string::npos);
  EXPECT_NE(source.find("surface.specular_f90 = lerp(dielectric_specular_f90, 1.0, surface.metallic)"),
            std::string::npos);
  EXPECT_NE(source.find("return f0 + (f90 - f0) * pow"), std::string::npos);
  EXPECT_NE(source.find("lerp(dielectric_specular_f0, max(surface.base_color.rgb"), std::string::npos);
  const auto rebase_specular_f0 =
      ExtractSourceRange(source, "float3 EE_GLTF_RASTER_REBASE_SPECULAR_F0", "float3 EE_GLTF_SAFE_NORMALIZE");
  EXPECT_EQ(rebase_specular_f0.find("pbr_model"), std::string::npos);
  EXPECT_NE(rebase_specular_f0.find("const float metallic = clamp(surface.metallic"), std::string::npos);
  EXPECT_NE(rebase_specular_f0.find("surface.specular_f0 - original_base_color * metallic"), std::string::npos);
  EXPECT_NE(rebase_specular_f0.find("return lerp(dielectric_specular_f0, rebased_base_color, metallic)"),
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
  EXPECT_EQ(source.find("material.pbr_diffuse_factor * vertex_color"), std::string::npos);
  EXPECT_NE(source.find("material.pbr_base_color_factor * vertex_color"), std::string::npos);
  EXPECT_NE(source.find("EE_GLTF_SRGB_TO_LINEAR"), std::string::npos);
  EXPECT_NE(source.find("encoded.x <= 0.04045"), std::string::npos);
  EXPECT_NE(source.find("sample_value.rgb = EE_GLTF_SRGB_TO_LINEAR(sample_value.rgb)"), std::string::npos);
  EXPECT_NE(source.find("surface.alpha_mode == EE_GLTF_ALPHA_MODE_OPAQUE"), std::string::npos);
  EXPECT_NE(source.find("EE_GLTF_SAMPLE_TEXTURE_LOD0_SPECIALIZED<feature_mask>(material.transmission_texture"),
            std::string::npos);
  EXPECT_NE(source.find("effective_diffuse_transmission * diffuse_transmission_color"), std::string::npos);
  EXPECT_NE(source.find("EE_GLTF_RASTER_FRESNEL(specular_f0, float3(specular_weight)"), std::string::npos);
  EXPECT_NE(source.find("const float remaining_energy = 1.0 - max(fresnel.r"), std::string::npos);
}

TEST(GltfRasterMaterial, FractionalSpecularF90AndUnlitSurviveDeferredAndTransparentPaths) {
  const auto lighting = ReadTextFile(ShaderPath("Modules/EvoEngine/Lighting.slang"));
  const auto deferred = ReadTextFile(ShaderPath("Compute/DeferredComputeLighting.slang"));
  const auto transparent = ReadTextFile(ShaderPath("Graphics/Fragment/Standard/StandardTransparent.slang"));
  ASSERT_FALSE(lighting.empty());
  ASSERT_FALSE(deferred.empty());
  ASSERT_FALSE(transparent.empty());

  EXPECT_NE(lighting.find("float3(F90) - F0"), std::string::npos);
  EXPECT_NE(lighting.find("float3(F90 * (1.0f - roughness))"), std::string::npos);
  EXPECT_NE(lighting.find("float3(F90 * brdf.y)"), std::string::npos);
  EXPECT_NE(deferred.find("surface.specular_f90"), std::string::npos);
  const auto deferred_unlit =
      ExtractSourceRange(deferred, "if (!scene_camera && encoded_specular_f90 < 0.0f)", "const float linear_depth");
  EXPECT_NE(deferred_unlit.find("indirect_lighting_debug_view == 0 ? resolved_base_color : float3(0.0f)"),
            std::string::npos);
  EXPECT_EQ(deferred_unlit.find("EE_FUNC_CALCULATE_"), std::string::npos);
  EXPECT_NE(deferred.find("EE_FUNC_CALCULATE_LIGHTS"), std::string::npos);
  EXPECT_NE(deferred.find("EE_FUNC_CALCULATE_DDGI_ENVIRONMENTAL_LIGHT"), std::string::npos);

  const auto transparent_unlit =
      ExtractSourceRange(transparent, "if (EE_GLTF_MATERIALS[material_index].unlit != 0) {", "float3 normal =");
  EXPECT_NE(transparent_unlit.find("indirect_lighting_debug_view == 0 ? surface.base_color.rgb : float3(0.0f)"),
            std::string::npos);
  EXPECT_NE(transparent_unlit.find("indirect_lighting_debug_view == 0 ? EE_GLTF_RASTER_OPACITY(surface) : 1.0f"),
            std::string::npos);
  EXPECT_NE(transparent_unlit.find("return float4"), std::string::npos);
  EXPECT_EQ(transparent_unlit.find("EE_FUNC_CALCULATE_"), std::string::npos);
  EXPECT_NE(transparent.find("surface.specular_f90"), std::string::npos);
}

TEST(GltfRasterMaterial, PerFrameBindsCanonicalMaterialBuffers) {
  const auto per_frame = ReadTextFile(ShaderPath("Modules/EvoEngine/PerFrame.slang"));
  const auto material = ReadTextFile(ShaderPath("Modules/EvoEngine/GltfMaterial.slang"));
  const auto textures = ReadTextFile(ShaderPath("Modules/EvoEngine/Textures.slang"));
  ASSERT_FALSE(per_frame.empty());
  ASSERT_FALSE(material.empty());
  ASSERT_FALSE(textures.empty());

  EXPECT_NE(per_frame.find("__exported import EvoEngine.Textures;"), std::string::npos);
  EXPECT_NE(per_frame.find("__exported import EvoEngine.GltfMaterial;"), std::string::npos);
  EXPECT_NE(material.find("[[vk::binding(11, 0)]]"), std::string::npos);
  EXPECT_NE(material.find("StructuredBuffer<GltfShadeMaterial, ScalarDataLayout>"), std::string::npos);
  EXPECT_NE(material.find("[[vk::binding(12, 0)]]"), std::string::npos);
  EXPECT_NE(material.find("StructuredBuffer<GltfTextureInfo, ScalarDataLayout>"), std::string::npos);
  EXPECT_NE(textures.find("[[vk::binding(9, 0)]]"), std::string::npos);
  EXPECT_NE(textures.find("[[vk::binding(10, 0)]]"), std::string::npos);
}

TEST(GltfRasterMaterial, SharedBindlessTextureAccessPreservesCanonicalMaterialSemantics) {
  const auto evaluator = ReadTextFile(ShaderPath("Modules/EvoEngine/GltfBindlessMaterial.slang"));
  const auto ray_compatibility = ReadTextFile(ShaderPath("Modules/EvoEngine/GltfRayTracingMaterial.slang"));
  const auto bsdf = ReadTextFile(ShaderPath("Modules/EvoEngine/GltfRayTracingBsdf.slang"));
  const auto textures = ReadTextFile(ShaderPath("Modules/EvoEngine/Textures.slang"));
  const auto raster_probe =
      ReadTextFile(RepoPath("EvoEngine_Tests/Resources/Shaders/Fragment/GltfBindlessRasterAccessProbe.slang"));
  ASSERT_FALSE(evaluator.empty());
  ASSERT_FALSE(ray_compatibility.empty());
  ASSERT_FALSE(bsdf.empty());
  ASSERT_FALSE(textures.empty());
  ASSERT_FALSE(raster_probe.empty());

  EXPECT_NE(ray_compatibility.find("__exported import EvoEngine.GltfBindlessMaterial;"), std::string::npos);
  EXPECT_NE(evaluator.find("import EvoEngine.Textures;"), std::string::npos);
  EXPECT_NE(textures.find("[[vk::binding(9, 0)]]"), std::string::npos);
  EXPECT_EQ(evaluator.find("[[vk::binding(0, 3)]]"), std::string::npos);
  EXPECT_EQ(evaluator.find("EE_GLTF_RASTER_BASE_COLOR_TEXTURE"), std::string::npos);
  EXPECT_NE(evaluator.find("EE_TEXTURE_2DS[NonUniformResourceIndex(texture_info.index)]"), std::string::npos);
  EXPECT_EQ(evaluator.find("NonUniformResourceIndex(texture_info_slot)"), std::string::npos);
  EXPECT_NE(evaluator.find("if (!EE_GLTF_HAS_TEXTURE(texture_info_slot))"), std::string::npos);
  EXPECT_NE(evaluator.find("if (texture_info.index < 0)"), std::string::npos);
  EXPECT_NE(evaluator.find("return fallback;"), std::string::npos);
  EXPECT_NE(evaluator.find("EE_GLTF_DECODE_TEXTURE_SAMPLE"), std::string::npos);
  EXPECT_NE(evaluator.find("EE_GLTF_SRGB_TO_LINEAR"), std::string::npos);

  EXPECT_NE(evaluator.find("float2 ddx_uv0"), std::string::npos);
  EXPECT_NE(evaluator.find("float2 ddx_uv1"), std::string::npos);
  EXPECT_EQ(evaluator.find("ddx_uv2"), std::string::npos);
  EXPECT_NE(evaluator.find("float2 ddy_uv0"), std::string::npos);
  EXPECT_NE(evaluator.find("float2 ddy_uv1"), std::string::npos);
  EXPECT_EQ(evaluator.find("ddy_uv2"), std::string::npos);
  EXPECT_NE(evaluator.find("uint gradient_mask"), std::string::npos);
  EXPECT_NE(evaluator.find("EE_GLTF_SELECT_TEX_COORD_DDX"), std::string::npos);
  EXPECT_NE(evaluator.find("EE_GLTF_SELECT_TEX_COORD_DDY"), std::string::npos);
  EXPECT_NE(evaluator.find("EE_GLTF_TRANSFORM_UV(texture_info, float3(ddx_uv, 0.0))"), std::string::npos);
  EXPECT_NE(evaluator.find("EE_GLTF_TRANSFORM_UV(texture_info, float3(ddy_uv, 0.0))"), std::string::npos);
  EXPECT_NE(evaluator.find(".SampleGrad(uv, ddx_uv, ddy_uv)"), std::string::npos);
  EXPECT_NE(evaluator.find(".SampleLevel(uv, 0.0f)"), std::string::npos);
  EXPECT_NE(raster_probe.find("import EvoEngine.GltfBindlessMaterial;"), std::string::npos);
  EXPECT_NE(raster_probe.find("ddx(input.uv0)"), std::string::npos);
  EXPECT_NE(raster_probe.find("ddx(input.uv1)"), std::string::npos);
  EXPECT_EQ(raster_probe.find("input.uv2"), std::string::npos);
  EXPECT_NE(raster_probe.find("ddy(input.uv0)"), std::string::npos);
  EXPECT_NE(raster_probe.find("ddy(input.uv1)"), std::string::npos);

  constexpr const char* texture_slots[] = {
      "pbr_base_color_texture",
      "normal_texture",
      "pbr_metallic_roughness_texture",
      "emissive_texture",
      "occlusion_texture",
      "transmission_texture",
      "thickness_texture",
      "clearcoat_texture",
      "clearcoat_roughness_texture",
      "clearcoat_normal_texture",
      "specular_texture",
      "specular_color_texture",
      "iridescence_texture",
      "iridescence_thickness_texture",
      "anisotropy_texture",
      "sheen_color_texture",
      "sheen_roughness_texture",
      "diffuse_transmission_texture",
      "diffuse_transmission_color_texture",
      "retroreflection_texture",
  };
  const auto material_evaluation = evaluator + bsdf;
  for (const auto* texture_slot : texture_slots) {
    EXPECT_NE(material_evaluation.find(std::string("material.") + texture_slot), std::string::npos) << texture_slot;
  }
}

TEST(GltfRasterMaterial, RasterMaterialDescriptorsAreRetiredWithoutAffectingPassLocalLighting) {
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

  EXPECT_EQ(render_instance_header.find("kRasterMaterialTextureSlotCount"), std::string::npos);
  EXPECT_EQ(render_layer_header.find("GetRasterMaterialDescriptorSetLayout"), std::string::npos);
  EXPECT_EQ(render_layer_header.find("raster_material_per_frame_layout_"), std::string::npos);
  EXPECT_EQ(render_layer_header.find("raster_material_layout_"), std::string::npos);
  EXPECT_NE(render_layer_header.find("raster_lighting_texture_layout_"), std::string::npos);
  EXPECT_EQ(render_layer_header.find("raster_material_per_frame_descriptor_sets_"), std::string::npos);
  EXPECT_NE(render_layer_header.find("raster_lighting_texture_descriptor_sets_"), std::string::npos);
  EXPECT_EQ(render_layer_header.find("raster_material_white_fallback_texture_"), std::string::npos);
  EXPECT_EQ(render_layer_header.find("raster_material_black_fallback_texture_"), std::string::npos);
  EXPECT_EQ(render_layer_header.find("raster_material_flat_normal_fallback_texture_"), std::string::npos);
  EXPECT_EQ(render_layer.find("raster_material_layout_->PushDescriptorBinding"), std::string::npos);
  EXPECT_EQ(render_layer.find("PushPerFrameSceneDescriptorBindings(raster_material_per_frame_layout_)"),
            std::string::npos);
  EXPECT_EQ(render_layer.find("PushPerFrameMaterialBufferDescriptorBindings(raster_material_per_frame_layout_)"),
            std::string::npos);
  EXPECT_NE(render_layer.find("ShouldCreatePerFrameBindlessTextureDescriptors"), std::string::npos);
  EXPECT_NE(render_layer.find("per_frame_bindless_texture_descriptors_enabled_"), std::string::npos);
  EXPECT_NE(render_layer.find("PushPerFrameBindlessTextureDescriptorBindings(per_frame_layout_"), std::string::npos);
  EXPECT_NE(render_layer.find("if (per_frame_bindless_texture_descriptors_enabled_)"), std::string::npos);
  EXPECT_EQ(render_layer.find("std::make_shared<DescriptorSet>(raster_material_per_frame_layout_)"), std::string::npos);
  EXPECT_NE(render_layer.find("std::make_shared<DescriptorSet>(raster_lighting_texture_layout_)"), std::string::npos);
  EXPECT_NE(render_layer.find("VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER"), std::string::npos);
  EXPECT_NE(render_layer.find("VK_SHADER_STAGE_FRAGMENT_BIT"), std::string::npos);
  EXPECT_EQ(render_layer.find("RefreshRasterMaterialDescriptorSets"), std::string::npos);
  EXPECT_NE(render_layer.find("glm::vec4(1.0f)"), std::string::npos);
  EXPECT_EQ(render_layer.find("glm::vec4(0.5f, 0.5f, 1.0f, 1.0f)"), std::string::npos);

  const auto capability_gate = ExtractSourceRange(render_layer, "bool ShouldCreatePerFrameBindlessTextureDescriptors",
                                                  "void PushPerFrameSceneDescriptorBindings");
  EXPECT_NE(capability_gate.find("return true;"), std::string::npos);

  EXPECT_EQ(render_instance_header.find("raster_material_descriptor_sets"), std::string::npos);
  EXPECT_EQ(render_instance_header.find("RefreshRasterMaterialDescriptorSets"), std::string::npos);
  EXPECT_EQ(render_instance.find("raster_material_descriptor_sets.resize(shade_materials.size())"), std::string::npos);
  EXPECT_EQ(render_instance.find("std::make_shared<DescriptorSet>(raster_material_layout)"), std::string::npos);
  EXPECT_EQ(render_instance.find("GltfPbrModel::SpecularGlossiness"), std::string::npos);
  EXPECT_EQ(render_instance.find("UpdateImageDescriptorBinding("), std::string::npos);

  EXPECT_NE(texture_storage_header.find("TryGetTexture2DDescriptorImageInfo"), std::string::npos);
  EXPECT_NE(texture_storage_header.find("TryGetCubemapDescriptorImageInfo"), std::string::npos);
  EXPECT_NE(texture_storage.find("TextureStorage::TryGetTexture2DDescriptorImageInfo"), std::string::npos);
  EXPECT_NE(texture_storage.find("TextureStorage::TryGetCubemapDescriptorImageInfo"), std::string::npos);
  EXPECT_NE(texture_storage.find("texture_storage.IsGpuUploadPending()"), std::string::npos);
  EXPECT_NE(texture_storage.find("bool IsSampledDescriptorImageLayout"), std::string::npos);
  EXPECT_EQ(CountOccurrences(texture_storage, "!IsSampledDescriptorImageLayout(layout)"), 2u);
  const auto bind_cubemaps = ExtractSourceRange(texture_storage, "uint32_t TextureStorage::BindCubemapToDescriptorSet",
                                                "const Texture2DStorage& TextureStorage::PeekTexture2DStorage");
  EXPECT_NE(bind_cubemaps.find("TryGetCubemapDescriptorImageInfo"), std::string::npos);
}

TEST(GltfRasterMaterial, StandardRendererRequiresBindlessSampledTextureCapabilities) {
  const auto platform_header = ReadTextFile(SdkPath("include/Rendering/Platform/Platform.hpp"));
  const auto platform = ReadTextFile(SdkPath("src/Platform.cpp"));
  const auto render_layer = ReadTextFile(SdkPath("src/RenderLayer.cpp"));
  ASSERT_FALSE(platform_header.empty());
  ASSERT_FALSE(platform.empty());
  ASSERT_FALSE(render_layer.empty());

  EXPECT_NE(platform_header.find("VkPhysicalDeviceVulkan12Features vulkan12_features"), std::string::npos);
  EXPECT_NE(platform.find("vulkan12_features.runtimeDescriptorArray != VK_TRUE"), std::string::npos);
  EXPECT_NE(platform.find("vulkan12_features.descriptorBindingPartiallyBound != VK_TRUE"), std::string::npos);
  EXPECT_NE(platform.find("vulkan12_features.shaderSampledImageArrayNonUniformIndexing != VK_TRUE"), std::string::npos);
  EXPECT_NE(platform.find("vk_physical_device_vulkan12_features.shaderSampledImageArrayNonUniformIndexing = VK_TRUE"),
            std::string::npos);
  EXPECT_NE(platform.find("max_texture_2d_resource_size"), std::string::npos);
  EXPECT_NE(platform.find("max_cubemap_resource_size"), std::string::npos);
  EXPECT_NE(platform.find("maxPerStageDescriptorSampledImages"), std::string::npos);
  EXPECT_NE(platform.find("maxDescriptorSetSampledImages"), std::string::npos);
  EXPECT_NE(render_layer.find("Shared sampled-texture arrays active"), std::string::npos);
}

TEST(GltfRasterMaterial, OpaqueDeferredPassUsesSharedBindlessMaterialDescriptors) {
  const auto render_layer = ReadTextFile(SdkPath("src/RenderLayer.cpp"));
  const auto pass_header = ReadTextFile(SdkPath("include/Rendering/RenderPasses/DeferredGeometryPass.hpp"));
  const auto pass = ReadTextFile(SdkPath("src/RenderPasses/DeferredGeometryPass.cpp"));
  const auto utilities = ReadTextFile(SdkPath("src/RenderPasses/RenderPassUtilities.cpp"));
  ASSERT_FALSE(render_layer.empty());
  ASSERT_FALSE(pass_header.empty());
  ASSERT_FALSE(pass.empty());
  ASSERT_FALSE(utilities.empty());

  EXPECT_EQ(render_layer.find("CreateRasterMaterialShaderDefines"), std::string::npos);
  EXPECT_EQ(render_layer.find("CreateRasterMaterialNoBindlessShaderDefines"), std::string::npos);
  EXPECT_EQ(render_layer.find("EE_SKIP_PER_FRAME_BINDLESS_TEXTURES"), std::string::npos);
  EXPECT_EQ(render_layer.find("EE_GLTF_RASTER_FIXED_MATERIAL_TEXTURES"), std::string::npos);

  const std::string normal_pipeline = ExtractSourceRange(render_layer, "if (!deferred_geometry_pipeline_normal)",
                                                         "deferred_geometry_pipeline_normal->depth_attachment_format");
  EXPECT_NE(normal_pipeline.find("Platform::GetShaderGlobalDefines()"), std::string::npos);
  EXPECT_EQ(CountOccurrences(normal_pipeline, "empty_descriptor_set_layout_"), 2);
  EXPECT_NE(normal_pipeline.find("per_frame_layout_"), std::string::npos);
  EXPECT_EQ(normal_pipeline.find("raster_material_layout_"), std::string::npos);

  const std::string mesh_pipeline =
      ExtractSourceRange(render_layer,
                         "if (Platform::GetInstance().GetCapabilities().support_mesh_shader && "
                         "!deferred_geometry_pipeline_mesh)",
                         "deferred_geometry_pipeline_mesh->depth_attachment_format");
  EXPECT_NE(mesh_pipeline.find("Platform::GetShaderGlobalDefines()"), std::string::npos);
  EXPECT_NE(mesh_pipeline.find("meshlet_layout_"), std::string::npos);
  EXPECT_EQ(CountOccurrences(mesh_pipeline, "empty_descriptor_set_layout_"), 1);
  EXPECT_NE(mesh_pipeline.find("per_frame_layout_"), std::string::npos);
  EXPECT_EQ(mesh_pipeline.find("raster_material_layout_"), std::string::npos);

  const std::string instanced_pipeline =
      ExtractSourceRange(render_layer, "if (!instanced_deferred_geometry_pipeline)",
                         "instanced_deferred_geometry_pipeline->depth_attachment_format");
  EXPECT_NE(instanced_pipeline.find("Platform::GetShaderGlobalDefines()"), std::string::npos);
  EXPECT_NE(instanced_pipeline.find("particle_instanced_data_layout_"), std::string::npos);
  EXPECT_EQ(CountOccurrences(instanced_pipeline, "empty_descriptor_set_layout_"), 1);
  EXPECT_NE(instanced_pipeline.find("per_frame_layout_"), std::string::npos);
  EXPECT_EQ(instanced_pipeline.find("raster_material_layout_"), std::string::npos);

  const std::string skinned_pipeline =
      ExtractSourceRange(render_layer, "if (!skinned_deferred_geometry_pipeline)",
                         "skinned_deferred_geometry_pipeline->depth_attachment_format");
  EXPECT_NE(skinned_pipeline.find("Platform::GetShaderGlobalDefines()"), std::string::npos);
  EXPECT_NE(skinned_pipeline.find("bone_matrices_layout_"), std::string::npos);
  EXPECT_EQ(CountOccurrences(skinned_pipeline, "empty_descriptor_set_layout_"), 1);
  EXPECT_NE(skinned_pipeline.find("per_frame_layout_"), std::string::npos);
  EXPECT_EQ(skinned_pipeline.find("raster_material_layout_"), std::string::npos);

  const std::string strands_pipeline =
      ExtractSourceRange(render_layer, "if (Platform::MeshShaderEnabled() && !strands_deferred_geometry_pipeline)",
                         "strands_deferred_geometry_pipeline->depth_attachment_format");
  EXPECT_NE(strands_pipeline.find("Platform::GetShaderGlobalDefines()"), std::string::npos);
  EXPECT_NE(strands_pipeline.find("StandardStrands.slang"), std::string::npos);
  EXPECT_NE(strands_pipeline.find("strand_meshlet_layout_"), std::string::npos);
  EXPECT_EQ(strands_pipeline.find("tessellation_"), std::string::npos);
  EXPECT_EQ(strands_pipeline.find("geometry_shader"), std::string::npos);
  EXPECT_EQ(CountOccurrences(strands_pipeline, "empty_descriptor_set_layout_"), 1);
  EXPECT_NE(strands_pipeline.find("per_frame_layout_"), std::string::npos);
  EXPECT_EQ(strands_pipeline.find("raster_material_layout_"), std::string::npos);

  EXPECT_NE(render_layer.find("per_frame_descriptor_sets_[current_frame_index]"), std::string::npos);
  EXPECT_EQ(render_layer.find("raster_material_per_frame_descriptor_sets_[current_frame_index]"), std::string::npos);
  EXPECT_NE(render_layer.find("enable_indirect_rendering,"), std::string::npos);
  EXPECT_NE(render_layer.find("count_draw_calls,"), std::string::npos);
  EXPECT_NE(render_layer.find("reflection_probe_capture ? false : wire_frame"), std::string::npos);

  EXPECT_EQ(pass.find("BindRasterMaterialDescriptorSet"), std::string::npos);
  EXPECT_NE(pass.find("deferred_mesh_indirect_batches"), std::string::npos);
  EXPECT_NE(pass.find("draw_instance_index_offset"), std::string::npos);
  EXPECT_NE(pass.find("batch.first_command * sizeof(VkDrawIndexedIndirectCommand)"), std::string::npos);
  EXPECT_EQ(CountOccurrences(pass, "BindRasterMaterialDescriptorSet(vk_command_buffer, parameters."), 0);
  EXPECT_EQ(utilities.find("GetRasterMaterialDescriptorSet(static_cast<uint32_t>(material_index))"), std::string::npos);
  EXPECT_EQ(utilities.find("BindDescriptorSet(vk_command_buffer, 3"), std::string::npos);
}

TEST(GltfRasterMaterial, DeferredIndirectUsesInstanceMetadataWithoutMaterialBatches) {
  const auto render_instance_header =
      ReadTextFile(SdkPath("include/Rendering/RenderInstances/RenderInstanceStorage.hpp"));
  const auto render_instance = ReadTextFile(SdkPath("src/RenderInstanceStorage.cpp"));
  const auto pass = ReadTextFile(SdkPath("src/RenderPasses/DeferredGeometryPass.cpp"));
  const auto instances = ReadTextFile(SdkPath("Internals/DefaultResources/Shaders/Modules/EvoEngine/Instances.slang"));
  const auto indexed_vertex =
      ReadTextFile(SdkPath("Internals/DefaultResources/Shaders/Graphics/Vertex/Standard/Standard.slang"));
  const auto mesh_task =
      ReadTextFile(SdkPath("Internals/DefaultResources/Shaders/Graphics/Task/Standard/Standard.slang"));
  const auto shadow_shaders =
      ReadTextFile(
          SdkPath("Internals/DefaultResources/Shaders/Graphics/Vertex/Lighting/DirectionalLightShadowMap.slang")) +
      ReadTextFile(SdkPath("Internals/DefaultResources/Shaders/Graphics/Vertex/Lighting/PointLightShadowMap.slang")) +
      ReadTextFile(SdkPath("Internals/DefaultResources/Shaders/Graphics/Vertex/Lighting/SpotLightShadowMap.slang")) +
      ReadTextFile(
          SdkPath("Internals/DefaultResources/Shaders/Graphics/Task/Lighting/DirectionalLightShadowMap.slang")) +
      ReadTextFile(SdkPath("Internals/DefaultResources/Shaders/Graphics/Task/Lighting/PointLightShadowMap.slang")) +
      ReadTextFile(SdkPath("Internals/DefaultResources/Shaders/Graphics/Task/Lighting/SpotLightShadowMap.slang"));
  ASSERT_FALSE(render_instance_header.empty());
  ASSERT_FALSE(render_instance.empty());
  ASSERT_FALSE(pass.empty());
  ASSERT_FALSE(instances.empty());
  ASSERT_FALSE(indexed_vertex.empty());
  ASSERT_FALSE(mesh_task.empty());
  ASSERT_FALSE(shadow_shaders.empty());

  EXPECT_NE(render_instance_header.find("struct DeferredMeshIndirectBatch"), std::string::npos);
  EXPECT_NE(render_instance_header.find("std::vector<DeferredMeshIndirectBatch> deferred_mesh_indirect_batches"),
            std::string::npos);
  EXPECT_NE(render_instance.find("deferred_mesh_indirect_batches.clear()"), std::string::npos);
  const auto batch_definition = ExtractSourceRange(render_instance_header, "struct DeferredMeshIndirectBatch",
                                                   "class EVOENGINE_API RasterSpatialIndex");
  EXPECT_EQ(batch_definition.find("material_index"), std::string::npos);
  EXPECT_EQ(render_instance.find("batch.material_index"), std::string::npos);
  EXPECT_EQ(render_instance_header.find("first_instance_index"), std::string::npos);
  EXPECT_NE(render_instance_header.find("std::vector<uint32_t> raster_draw_instance_indices"), std::string::npos);
  EXPECT_NE(render_instance.find(
                "raster_draw_instance_indices.emplace_back(static_cast<uint32_t>(render_instance->instance_index))"),
            std::string::npos);
  EXPECT_NE(render_instance.find("batch.first_command = deferred_mesh_command_index"), std::string::npos);
  EXPECT_NE(render_instance.find("batch.command_count++"), std::string::npos);
  EXPECT_NE(render_instance.find("batch.triangle_count +="), std::string::npos);

  EXPECT_EQ(pass.find("BindRasterMaterialDescriptorSet"), std::string::npos);
  EXPECT_NE(pass.find("RenderInstancePushConstant::kRasterDrawInstanceMappingBit"), std::string::npos);
  EXPECT_NE(pass.find("batch.first_command"), std::string::npos);
  EXPECT_NE(pass.find("ResolvePolygonMode(parameters.wire_frame, batch.polygon_mode)"), std::string::npos);
  EXPECT_NE(pass.find("batch.first_command * sizeof(VkDrawMeshTasksIndirectCommandEXT)"), std::string::npos);
  EXPECT_NE(pass.find("batch.first_command * sizeof(VkDrawIndexedIndirectCommand)"), std::string::npos);
  EXPECT_NE(instances.find("[[vk::binding(14, 0)]]"), std::string::npos);
  EXPECT_NE(instances.find("EE_RASTER_DRAW_INSTANCE_INDEX"), std::string::npos);
  EXPECT_NE(indexed_vertex.find("EE_RASTER_DRAW_INSTANCE_INDEX"), std::string::npos);
  EXPECT_NE(mesh_task.find("EE_RASTER_DRAW_INSTANCE_INDEX"), std::string::npos);
  EXPECT_EQ(CountOccurrences(shadow_shaders, "EE_RASTER_DRAW_INSTANCE_INDEX"), 6);
}

TEST(GltfRasterMaterial, ShadowAndTransparentPassesAvoidFixedMaterialDescriptors) {
  const auto render_layer = ReadTextFile(SdkPath("src/RenderLayer.cpp"));
  const auto directional_header =
      ReadTextFile(SdkPath("include/Rendering/RenderPasses/DirectionalLightShadowPass.hpp"));
  const auto directional = ReadTextFile(SdkPath("src/RenderPasses/DirectionalLightShadowPass.cpp"));
  const auto transparent = ReadTextFile(SdkPath("src/RenderPasses/TransparentGeometryPass.cpp"));
  const auto utilities_header = ReadTextFile(SdkPath("include/Rendering/RenderPasses/RenderPassUtilities.hpp"));
  const auto masked_shadow = ReadTextFile(ShaderPath("Graphics/Fragment/Lighting/AlphaMaskedShadow.slang"));
  ASSERT_FALSE(render_layer.empty());
  ASSERT_FALSE(directional_header.empty());
  ASSERT_FALSE(directional.empty());
  ASSERT_FALSE(transparent.empty());
  ASSERT_FALSE(utilities_header.empty());
  ASSERT_FALSE(masked_shadow.empty());

  const std::string shadow_pipeline_region =
      ExtractSourceRange(render_layer, "#pragma region Graphics Pipelines", "if (!deferred_geometry_pipeline_normal)");
  EXPECT_NE(shadow_pipeline_region.find("point_light_shadow_pipeline_normal_opaque"), std::string::npos);
  EXPECT_NE(shadow_pipeline_region.find("point_light_shadow_pipeline_normal_masked"), std::string::npos);
  EXPECT_NE(shadow_pipeline_region.find("spot_light_shadow_pipeline_normal_opaque"), std::string::npos);
  EXPECT_NE(shadow_pipeline_region.find("directional_light_shadow_pipeline_normal_opaque"), std::string::npos);
  EXPECT_NE(shadow_pipeline_region.find("instanced_point_light_shadow_pipeline_opaque"), std::string::npos);
  EXPECT_NE(shadow_pipeline_region.find("skinned_point_light_shadow_pipeline_opaque"), std::string::npos);
  EXPECT_NE(shadow_pipeline_region.find("CreateShadowVertexPipeline"), std::string::npos);
  EXPECT_NE(shadow_pipeline_region.find("CreateShadowMeshPipeline"), std::string::npos);
  EXPECT_NE(shadow_pipeline_region.find("shadow_masked_fragment_shader_path"), std::string::npos);
  EXPECT_EQ(shadow_pipeline_region.find("CreateRasterMaterialShaderDefines()"), std::string::npos);
  EXPECT_EQ(shadow_pipeline_region.find("raster_material_per_frame_layout_"), std::string::npos);
  EXPECT_EQ(shadow_pipeline_region.find("raster_material_layout_"), std::string::npos);
  EXPECT_EQ(shadow_pipeline_region.find("ShadowMapPassThrough.slang"), std::string::npos);
  EXPECT_NE(masked_shadow.find("import EvoEngine.GltfAlphaMask;"), std::string::npos);
  EXPECT_NE(masked_shadow.find("EE_GLTF_ALPHA_MASK_PASSES"), std::string::npos);
  EXPECT_EQ(masked_shadow.find("EE_EVALUATE_GLTF_RASTER_SURFACE"), std::string::npos);

  const std::string transparent_pipeline = ExtractSourceRange(
      render_layer, "if (!transparent_geometry_pipeline_normal)", "transparent_geometry_pipeline_normal->Initialize()");
  EXPECT_EQ(transparent_pipeline.find("CreateRasterMaterialShaderDefines()"), std::string::npos);
  EXPECT_EQ(transparent_pipeline.find("CreateRasterNoBindlessTextureShaderDefines()"), std::string::npos);
  EXPECT_NE(transparent_pipeline.find("per_frame_layout_"), std::string::npos);
  EXPECT_EQ(transparent_pipeline.find("raster_material_per_frame_layout_"), std::string::npos);
  EXPECT_NE(transparent_pipeline.find("lighting_layout_"), std::string::npos);
  EXPECT_EQ(transparent_pipeline.find("raster_material_layout_"), std::string::npos);
  EXPECT_NE(transparent_pipeline.find("raster_lighting_texture_layout_"), std::string::npos);
  EXPECT_EQ(transparent.find("BindRasterMaterialDescriptorSet"), std::string::npos);
  EXPECT_NE(transparent.find("parameters.raster_lighting_texture_descriptor_set"), std::string::npos);
  EXPECT_NE(transparent.find("vk_command_buffer, 3, parameters.raster_lighting_texture_descriptor_set"),
            std::string::npos);
  EXPECT_EQ(transparent.find("render_instance->material_index"), std::string::npos);

  EXPECT_EQ(utilities_header.find("BindRasterMaterialDescriptorSet"), std::string::npos);
  EXPECT_EQ(directional_header.find("bind_raster_material_descriptor_sets"), std::string::npos);
  EXPECT_EQ(directional_header.find("raster_material_per_frame_descriptor_set"), std::string::npos);
  EXPECT_NE(directional_header.find("directional_masked_pipeline"), std::string::npos);
  EXPECT_NE(directional_header.find("instanced_masked_pipeline"), std::string::npos);
  EXPECT_NE(directional_header.find("skinned_masked_pipeline"), std::string::npos);
  EXPECT_EQ(directional.find("BindRasterMaterialDescriptorSet"), std::string::npos);
  EXPECT_EQ(directional.find("alpha_tested"), std::string::npos);
  EXPECT_NE(directional.find("RenderInstancePushConstant::kRasterDrawInstanceMappingBit"), std::string::npos);
  EXPECT_NE(directional.find("render_bucket(false)"), std::string::npos);
  EXPECT_NE(directional.find("render_bucket(true)"), std::string::npos);
  EXPECT_NE(directional.find("masked_shadow_mesh_indirect_batches"), std::string::npos);
  EXPECT_EQ(render_layer.find("const bool bind_raster_material_descriptor_sets = true"), std::string::npos);
  EXPECT_EQ(render_layer.find("use_alpha_tested_indirect_shadow"), std::string::npos);
  EXPECT_NE(render_layer.find("RenderInstancePushConstant::kRasterDrawInstanceMappingBit"), std::string::npos);
  EXPECT_EQ(render_layer.find("const auto& per_frame_descriptor_set = alpha_tested_pipeline"), std::string::npos);
  EXPECT_NE(render_layer.find("raster_lighting_texture_descriptor_set"), std::string::npos);
  EXPECT_EQ(render_layer.find("point_light_info_block.viewport, false, true"), std::string::npos);
  EXPECT_EQ(render_layer.find("spot_light_info_block.viewport, false, true"), std::string::npos);
  EXPECT_EQ(render_layer.find("BindRasterMaterialDescriptorSet(vk_command_buffer, target_pipeline, "
                              "current_render_instances"),
            std::string::npos);
  const std::string directional_call =
      ExtractSourceRange(render_layer, "DirectionalLightShadowPass::Execute", "auto deferred_geometry_descriptor");
  EXPECT_NE(directional_call.find("per_frame_descriptor_sets_[current_frame_index]"), std::string::npos);
  EXPECT_EQ(directional_call.find("raster_material_per_frame_descriptor_sets_[current_frame_index]"),
            std::string::npos);
}

TEST(GltfRasterMaterial, RasterLightingUsesBindlessGlobalIblAndPassLocalAmbientOcclusion) {
  const auto lighting_shader = ReadTextFile(ShaderPath("Modules/EvoEngine/Lighting.slang")) +
                               ReadTextFile(ShaderPath("Modules/EvoEngine/LightingFixedSet3.slang"));
  const auto transparent_lighting_module = ReadTextFile(ShaderPath("Modules/EvoEngine/LightingFixedSet3.slang"));
  const auto deferred_lighting_shader = ReadTextFile(ShaderPath("Compute/DeferredComputeLighting.slang"));
  const auto transparent_lighting_shader =
      ReadTextFile(ShaderPath("Graphics/Fragment/Standard/StandardTransparent.slang"));
  const auto render_info_shader = ReadTextFile(ShaderPath("Modules/EvoEngine/RenderInfo.slang"));
  const auto render_layer_header = ReadTextFile(SdkPath("include/Layers/RenderLayer.hpp"));
  const auto render_layer = ReadTextFile(SdkPath("src/RenderLayer.cpp"));
  const auto render_instance_storage = ReadTextFile(SdkPath("src/RenderInstanceStorage.cpp"));
  const auto deferred_header = ReadTextFile(SdkPath("include/Rendering/RenderPasses/DeferredComputeLightingPass.hpp"));
  const auto deferred = ReadTextFile(SdkPath("src/RenderPasses/DeferredComputeLightingPass.cpp"));
  const auto transparent_header = ReadTextFile(SdkPath("include/Rendering/RenderPasses/TransparentGeometryPass.hpp"));
  const auto transparent = ReadTextFile(SdkPath("src/RenderPasses/TransparentGeometryPass.cpp"));
  ASSERT_FALSE(lighting_shader.empty());
  ASSERT_FALSE(transparent_lighting_module.empty());
  ASSERT_FALSE(deferred_lighting_shader.empty());
  ASSERT_FALSE(transparent_lighting_shader.empty());
  ASSERT_FALSE(render_info_shader.empty());
  ASSERT_FALSE(render_layer_header.empty());
  ASSERT_FALSE(render_layer.empty());
  ASSERT_FALSE(render_instance_storage.empty());
  ASSERT_FALSE(deferred_header.empty());
  ASSERT_FALSE(deferred.empty());
  ASSERT_FALSE(transparent_header.empty());
  ASSERT_FALSE(transparent.empty());

  EXPECT_EQ(lighting_shader.find("EE_RASTER_BRDF_LUT"), std::string::npos);
  EXPECT_EQ(lighting_shader.find("EE_RASTER_SKYBOX"), std::string::npos);
  EXPECT_EQ(lighting_shader.find("EE_RASTER_IRRADIANCE_MAP"), std::string::npos);
  EXPECT_EQ(lighting_shader.find("EE_RASTER_PREFILTERED_MAP"), std::string::npos);
  EXPECT_NE(lighting_shader.find("EE_CUBEMAPS[NonUniformResourceIndex(texture_index)]"), std::string::npos);
  EXPECT_NE(lighting_shader.find("EE_TEXTURE_2DS[NonUniformResourceIndex(EE_RENDER_INFO.brdf_lut_map_index)]"),
            std::string::npos);
  EXPECT_NE(lighting_shader.find(".skybox_tex_index"), std::string::npos);
  EXPECT_NE(lighting_shader.find(".irradiance_map_index"), std::string::npos);
  EXPECT_NE(lighting_shader.find(".prefiltered_map_index"), std::string::npos);
  EXPECT_EQ(lighting_shader.find("EE_RASTER_REFLECTION_PROBES"), std::string::npos);
  EXPECT_EQ(transparent_lighting_module.find("EE_RASTER_REFLECTION_PROBES"), std::string::npos);
  EXPECT_NE(render_info_shader.find("const uint EE_REFLECTION_PROBE_MAX_COUNT = 32u"), std::string::npos);
  EXPECT_NE(lighting_shader.find("sampleReflectionProbe(int texture_index"), std::string::npos);
  EXPECT_NE(lighting_shader.find("int(probe.identity_and_flags.x)"), std::string::npos);
  EXPECT_NE(lighting_shader.find("int(probe.transition_parameters.x)"), std::string::npos);
  const auto environmental_components =
      ExtractSourceRange(lighting_shader, "EeEnvironmentalLighting EE_FUNC_CALCULATE_ENVIRONMENTAL_COMPONENTS",
                         "float3 EE_FUNC_CALCULATE_ENVIRONMENTAL_LIGHT");
  EXPECT_GE(CountOccurrences(environmental_components, "float3(1.0f / EE_ENVIRONMENT.gamma)"), 2u);
  EXPECT_NE(lighting_shader.find("int(EE_REFLECTION_PROBE_MAX_COUNT)"), std::string::npos);
  EXPECT_NE(lighting_shader.find("EE_REFLECTION_PROBE_INFLUENCE"), std::string::npos);
  EXPECT_NE(lighting_shader.find("EE_REFLECTION_PROBE_DIRECTION"), std::string::npos);
  EXPECT_NE(lighting_shader.find("any(abs(local_position) > projection_extents)"), std::string::npos);
  EXPECT_NE(lighting_shader.find("primary_weight < 1.0f"), std::string::npos);
  EXPECT_NE(lighting_shader.find("lighting_parameters.y >="), std::string::npos);
  EXPECT_NE(lighting_shader.find("global_weight = (1.0f - primary_weight) * (1.0f - secondary_raw_weight)"),
            std::string::npos);
  EXPECT_NE(lighting_shader.find("result += global_prefiltered * primary_weight"), std::string::npos);
  EXPECT_NE(lighting_shader.find("EE_SPATIAL_REFLECTION_PREFILTERED(resources, fragPos, R, roughness,"),
            std::string::npos);
  EXPECT_NE(environmental_components.find("EE_ENVIRONMENT.diffuse_fallback_intensity"), std::string::npos);
  EXPECT_NE(environmental_components.find("EE_ENVIRONMENT.specular_fallback_intensity"), std::string::npos);
  EXPECT_EQ(environmental_components.find("diffuse_environment_intensity"), std::string::npos);
  EXPECT_EQ(environmental_components.find("specular_reflection_intensity"), std::string::npos);
  EXPECT_NE(lighting_shader.find(
                "const float indirectVisibility = clamp(materialOcclusion * screenSpaceVisibility, 0.0f, 1.0f)"),
            std::string::npos);
  EXPECT_NE(lighting_shader.find("const float3 diffuseIndirect = diffuse * indirectVisibility *"), std::string::npos);
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
  EXPECT_NE(environmental_components.find("prefilteredColor * (F * brdf.x + float3(F90 * brdf.y))"), std::string::npos);
  EXPECT_NE(environmental_components.find("EE_RECOVER_DIELECTRIC_F0(albedo, metallic, F0)"), std::string::npos);
  EXPECT_NE(environmental_components.find("EE_RECOVER_DIELECTRIC_F90(metallic, F90)"), std::string::npos);
  EXPECT_NE(environmental_components.find("(float3(1.0f) - dielectricF) *"), std::string::npos);
  EXPECT_EQ(environmental_components.find("(float3(1.0f) - F) * (1.0f - metallic)"), std::string::npos);
  EXPECT_EQ(CountOccurrences(lighting_shader, "float3 kS = F"), 0u);
  EXPECT_EQ(CountOccurrences(lighting_shader, "(float3(1.0f) - dielectricF) *"), 4u);
  EXPECT_NE(environmental_components.find("result.specular = result.unoccluded_specular * result.specular_visibility"),
            std::string::npos);
  EXPECT_EQ(environmental_components.find("result.diffuse_lighting"), std::string::npos);
  EXPECT_EQ(lighting_shader.find("float EE_REFLECTION_LIGHTING_SCALE"), std::string::npos);
  EXPECT_EQ(lighting_shader.find("reflectionLightingScale"), std::string::npos);
  EXPECT_NE(lighting_shader.find("const float ddgiSpecularVisibility = "
                                 "lerp(1.0f, EE_DDGI_GATHER_VISIBILITY(gather), gather_weight)"),
            std::string::npos);
  EXPECT_NE(lighting_shader.find("const float3 specular = environment.specular"), std::string::npos);
  EXPECT_NE(deferred_lighting_shader.find("inAmbientOcclusion.SampleLevel(tex_coord, 0.0f).r"), std::string::npos);
  EXPECT_NE(transparent_lighting_shader.find("surface.occlusion, 1.0f"), std::string::npos);
  EXPECT_EQ(deferred_lighting_shader.find("screenSpaceVisibility, result"), std::string::npos);
  EXPECT_EQ(transparent_lighting_shader.find("1.0f, direct"), std::string::npos);

  EXPECT_NE(render_layer_header.find("raster_lighting_texture_layout_"), std::string::npos);
  EXPECT_NE(render_layer_header.find("raster_lighting_texture_descriptor_sets_"), std::string::npos);
  EXPECT_NE(render_layer_header.find("GetRasterLightingTextureDescriptorSet"), std::string::npos);
  EXPECT_EQ(render_layer.find("EE_RASTER_FIXED_LIGHTING_TEXTURES"), std::string::npos);
  EXPECT_EQ(render_layer.find("EE_SKIP_PER_FRAME_BINDLESS_TEXTURES"), std::string::npos);
  EXPECT_NE(render_layer.find("raster_lighting_texture_layout_->PushDescriptorBinding"), std::string::npos);
  EXPECT_NE(render_layer.find("TextureStorage::TryGetCubemapDescriptorImageInfo"), std::string::npos);
  EXPECT_EQ(render_layer.find("kRasterLightingBrdfLutBinding"), std::string::npos);
  EXPECT_EQ(render_layer.find("kRasterLightingSkyboxBinding"), std::string::npos);
  EXPECT_EQ(render_layer.find("kRasterLightingIrradianceBinding"), std::string::npos);
  EXPECT_EQ(render_layer.find("kRasterLightingPrefilteredBinding"), std::string::npos);
  EXPECT_NE(render_layer.find("kRasterLightingAmbientOcclusionBinding = 4"), std::string::npos);
  EXPECT_EQ(render_layer.find("kRasterLightingReflectionProbesBinding"), std::string::npos);
  EXPECT_EQ(render_layer.find("kRasterLightingDescriptorSamplerCount"), std::string::npos);
  EXPECT_EQ(render_layer.find("kRasterLightingMaxPerStageSamplerCount"), std::string::npos);
  EXPECT_EQ(render_layer.find("source_info = global_prefiltered_info"), std::string::npos);
  EXPECT_EQ(render_layer.find("target_info = global_prefiltered_info"), std::string::npos);

  EXPECT_NE(render_instance_storage.find("ResolveEnvironmentalLighting(target_scene)"), std::string::npos);
  EXPECT_NE(render_instance_storage.find("resolved_lighting.local_reflection_probes"), std::string::npos);
  EXPECT_NE(render_instance_storage.find("std::min(resolved_lighting.local_reflection_probes.size()"),
            std::string::npos);
  EXPECT_NE(render_instance_storage.find("const auto& probe = resolved_lighting.local_reflection_probes[index]"),
            std::string::npos);
  EXPECT_NE(render_instance_storage.find("info.lighting_parameters = glm::vec4"), std::string::npos);
  EXPECT_NE(render_instance_storage.find("static_cast<float>(probe.artist_priority)"), std::string::npos);
  EXPECT_NE(render_instance_storage.find("asset && asset->IsRuntimeReady()"), std::string::npos);
  EXPECT_NE(render_instance_storage.find("const auto& asset = probe.payload"), std::string::npos);
  EXPECT_NE(render_instance_storage.find("info.world_to_probe = glm::inverse(probe.transform)"), std::string::npos);
  EXPECT_NE(lighting_shader.find("const float3 inverse_column0 = cross(world_to_probe[1], world_to_probe[2])"),
            std::string::npos);
  const std::string probe_collection =
      ExtractSourceRange(render_instance_storage, "void RenderInstanceStorage::CollectReflectionProbes",
                         "void RenderInstanceStorage::UpdateTopLevelAccelerationStructure");
  EXPECT_EQ(probe_collection.find("GetPrivateComponentOwnersList<ReflectionProbe>"), std::string::npos);
  EXPECT_EQ(probe_collection.find("GetSanitizedSettings("), std::string::npos);
  EXPECT_EQ(probe_collection.find("SetEnabled("), std::string::npos);
  EXPECT_EQ(probe_collection.find("ClampSettings("), std::string::npos);
  EXPECT_EQ(probe_collection.find("EnforceSceneLimit("), std::string::npos);

  const std::string deferred_pipeline = ExtractSourceRange(render_layer, "if (!deferred_compute_lighting_pipeline_)",
                                                           "deferred_compute_lighting_pipeline_->Initialize()");
  EXPECT_EQ(deferred_pipeline.find("CreateRasterFixedLightingShaderDefines"), std::string::npos);
  EXPECT_NE(deferred_pipeline.find("per_frame_layout_"), std::string::npos);
  EXPECT_NE(deferred_pipeline.find("lighting_layout_"), std::string::npos);
  EXPECT_NE(deferred_pipeline.find("raster_lighting_texture_layout_"), std::string::npos);

  EXPECT_NE(deferred_header.find("raster_lighting_texture_descriptor_set"), std::string::npos);
  EXPECT_NE(deferred.find("parameters.raster_lighting_texture_descriptor_set"), std::string::npos);
  EXPECT_NE(deferred.find("BindDescriptorSet(vk_command_buffer, 3"), std::string::npos);
  EXPECT_NE(transparent_header.find("raster_lighting_texture_descriptor_set"), std::string::npos);
  EXPECT_NE(transparent.find("parameters.raster_lighting_texture_descriptor_set"), std::string::npos);
  EXPECT_NE(transparent.find("vk_command_buffer, 3, parameters.raster_lighting_texture_descriptor_set"),
            std::string::npos);
}

TEST(GltfRasterMaterial, PreviewThumbnailsUseRenderLayerFixedRasterPath) {
  const auto thumbnail_provider = ReadTextFile(SdkPath("src/AssetThumbnailProvider.cpp"));
  const auto offscreen_preview = ReadTextFile(SdkPath("src/OffscreenPreviewRenderer.cpp"));
  const auto post_processing = ReadTextFile(SdkPath("src/PostProcessingStack.cpp"));
  const auto bloom_copy = ReadTextFile(ShaderPath("Compute/PostProcessing/BloomCopy.slang"));
  const auto bloom_module = ReadTextFile(ShaderPath("Modules/EvoEngine/PostProcessing.slang"));
  const auto bloom_downsampling = ReadTextFile(ShaderPath("Compute/PostProcessing/BloomDownsampling.slang"));
  const auto bloom_upsampling = ReadTextFile(ShaderPath("Compute/PostProcessing/BloomUpsampling.slang"));
  const auto bloom_mix = ReadTextFile(ShaderPath("Compute/PostProcessing/BloomMix.slang"));
  const auto tone_mapping = ReadTextFile(ShaderPath("Compute/PostProcessing/ToneMapping.slang"));
  const auto studio_generator = ReadTextFile(RepoPath("Scripts/generate_material_preview_hdr.py"));
  const auto render_layer = ReadTextFile(SdkPath("src/RenderLayer.cpp"));
  ASSERT_FALSE(thumbnail_provider.empty());
  ASSERT_FALSE(offscreen_preview.empty());
  ASSERT_FALSE(post_processing.empty());
  ASSERT_FALSE(bloom_copy.empty());
  ASSERT_FALSE(bloom_downsampling.empty());
  ASSERT_FALSE(bloom_upsampling.empty());
  ASSERT_FALSE(bloom_mix.empty());
  ASSERT_FALSE(tone_mapping.empty());
  ASSERT_FALSE(studio_generator.empty());
  ASSERT_FALSE(render_layer.empty());

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
  EXPECT_NE(offscreen_preview.find("std::unique_ptr<PreviewContext> preview_context"), std::string::npos);
  EXPECT_NE(offscreen_preview.find("Textures/MaterialPreview/neutral_studio.hdr"), std::string::npos);
  EXPECT_NE(offscreen_preview.find("Camera::BackgroundSource::InheritEnvironmentalLighting"), std::string::npos);
  EXPECT_NE(offscreen_preview.find("context->studio_environment ? 0.45f"), std::string::npos);
  EXPECT_NE(studio_generator.find("0.35 * base"), std::string::npos);
  EXPECT_NE(studio_generator.find("0.70 * base"), std::string::npos);
  EXPECT_NE(studio_generator.find("1.20 * base"), std::string::npos);
  EXPECT_NE(offscreen_preview.find("kMaterialPresentationZoom = 1.32f"), std::string::npos);
  EXPECT_NE(offscreen_preview.find("material_settings.camera_zoom *= kMaterialPresentationZoom"), std::string::npos);
  EXPECT_NE(offscreen_preview.find("mesh->GetBound(), material_settings, true"), std::string::npos);
  EXPECT_NE(offscreen_preview.find("mesh->GetBound(), settings, false"), std::string::npos);
  EXPECT_NE(offscreen_preview.find("kMaterialBloomThreshold = 1.25f"), std::string::npos);
  EXPECT_NE(offscreen_preview.find("kMaterialBloomIntensity = 0.5f"), std::string::npos);
  EXPECT_NE(offscreen_preview.find("ProcessBloomAndToneMappingImmediately(camera)"), std::string::npos);
  EXPECT_LT(offscreen_preview.find("RenderSceneToCameraImmediately(scene, camera_transform, camera)"),
            offscreen_preview.find("ProcessBloomAndToneMappingImmediately(camera)"));
  EXPECT_LT(offscreen_preview.find("ProcessBloomAndToneMappingImmediately(camera)"),
            offscreen_preview.find("return CopyColorTexture(camera, settings)"));
  EXPECT_NE(post_processing.find("void PostProcessingStack::ProcessBloomAndToneMappingImmediately"), std::string::npos);
  EXPECT_NE(post_processing.find("context.record_commands(action)"), std::string::npos);
  EXPECT_NE(bloom_module.find("brightness = max(color.x, max(color.y, color.z))"), std::string::npos);
  EXPECT_NE(bloom_module.find("knee = max(knee"), std::string::npos);
  EXPECT_NE(bloom_copy.find("EePrefilterBloom(inColor.SampleLevel"), std::string::npos);
  EXPECT_NE(bloom_copy.find("constants.compression_start, constants.source_ceiling"), std::string::npos);
  EXPECT_NE(bloom_copy.find("(g + h + i + a) * 0.125f"), std::string::npos);
  EXPECT_NE(bloom_copy.find("(a + i + j + k) * 0.125f"), std::string::npos);
  EXPECT_NE(bloom_copy.find("(m + a + k + l) * 0.125f"), std::string::npos);
  EXPECT_NE(bloom_downsampling.find("constants.source_resolution"), std::string::npos);
  EXPECT_NE(bloom_downsampling.find("constants.target_resolution"), std::string::npos);
  EXPECT_NE(bloom_upsampling.find("current + upsample"), std::string::npos);
  EXPECT_NE(bloom_mix.find("color.xyz + constants.intensity * bloom"), std::string::npos);
  EXPECT_EQ(bloom_mix.find("clamp(color.xyz"), std::string::npos);
  EXPECT_NE(post_processing.find("processed_mip_count = mip_levels > 2 ? mip_levels - 2 : 1"), std::string::npos);
  EXPECT_NE(post_processing.find("processed_mip_count > 1 ? camera.upsample_texture : camera.downsample_texture_a"),
            std::string::npos);
  EXPECT_NE(post_processing.find("VK_FORMAT_R16G16B16A16_SFLOAT"), std::string::npos);
  EXPECT_NE(post_processing.find("VK_FORMAT_R32G32B32A32_SFLOAT"), std::string::npos);
  EXPECT_NE(offscreen_preview.find("ToneMapping::ToneMapMethod::Aces"), std::string::npos);
  EXPECT_NE(offscreen_preview.find("tone_mapping.exposure = 1.0f"), std::string::npos);
  EXPECT_NE(tone_mapping.find("mul(input_matrix, color)"), std::string::npos);
  EXPECT_NE(tone_mapping.find("mul(output_matrix, color)"), std::string::npos);
  EXPECT_NE(tone_mapping.find("inColor.GetDimensions(resolution_x, resolution_y)"), std::string::npos);
  EXPECT_NE(tone_mapping.find("float4 output_color = inColor.Load(tex_coord)"), std::string::npos);
  EXPECT_NE(tone_mapping.find("inColor[tex_coord] = output_color"), std::string::npos);
  EXPECT_NE(offscreen_preview.find("vkCmdCopyImage"), std::string::npos);
  EXPECT_EQ(offscreen_preview.find("Buffer image_buffer"), std::string::npos);
  EXPECT_EQ(offscreen_preview.find("GltfRasterMaterial.slangh"), std::string::npos);
  EXPECT_EQ(offscreen_preview.find("StandardDeferred.slang"), std::string::npos);
  EXPECT_EQ(offscreen_preview.find("StandardTransparent.slang"), std::string::npos);
  EXPECT_EQ(offscreen_preview.find("ShadowMapPassThrough.slang"), std::string::npos);
  EXPECT_EQ(offscreen_preview.find("Shader::CreateTemporary"), std::string::npos);

  const std::string immediate_render = ExtractSourceRange(
      render_layer, "void RenderLayer::RenderSceneToCameraImmediately", "void RenderLayer::RenderAll");
  EXPECT_NE(immediate_render.find("std::make_shared<RenderInstanceStorage>()"), std::string::npos);
  EXPECT_NE(immediate_render.find("PrepareSceneForRendering(scene, false, false, false, false,"), std::string::npos);
  EXPECT_NE(immediate_render.find("reflection_probe_capture ? &injected_cameras : nullptr"), std::string::npos);
  EXPECT_NE(immediate_render.find("!reflection_probe_capture"), std::string::npos);
  EXPECT_NE(
      immediate_render.find("RenderToCamera(scene, camera_global_transform, camera, true, reflection_probe_capture)"),
      std::string::npos);
  EXPECT_NE(immediate_render.find("BindRenderInstanceStorage(current_frame_index, previous_render_instances)"),
            std::string::npos);
}

TEST(GltfRasterMaterial, PersistentSampledViewRegistrationIsAssetOwnedAndAbsentFromTransientPasses) {
  const auto texture_storage = ReadTextFile(SdkPath("include/Rendering/Texture/TextureStorage.hpp"));
  const auto private_registration =
      ExtractSourceRange(texture_storage, "private:\n  /**\n   * @brief Unregisters a 2D texture", "public:\n  /**");
  ASSERT_FALSE(private_registration.empty());
  EXPECT_NE(private_registration.find("RegisterTexture2D"), std::string::npos);
  EXPECT_NE(private_registration.find("RegisterCubemap"), std::string::npos);

  const auto source_root = SdkPath("src");
  for (const auto& entry : std::filesystem::recursive_directory_iterator(source_root)) {
    if (!entry.is_regular_file() || (entry.path().extension() != ".cpp" && entry.path().extension() != ".hpp")) {
      continue;
    }
    const auto file_name = entry.path().filename().string();
    if (file_name == "Texture2D.cpp" || file_name == "Cubemap.cpp" || file_name == "TextureStorage.cpp") {
      continue;
    }
    const auto source = ReadTextFile(entry.path());
    EXPECT_EQ(source.find("TextureStorage::RegisterTexture2D"), std::string::npos) << entry.path().string();
    EXPECT_EQ(source.find("TextureStorage::RegisterCubemap"), std::string::npos) << entry.path().string();
  }
}

TEST(GltfRasterMaterial, ProjectThumbnailCacheTracksMaterialDependenciesAndStaysOutsideAssets) {
  const auto thumbnail_provider = ReadTextFile(SdkPath("src/AssetThumbnailProvider.cpp"));
  const auto file_manager = ReadTextFile(SdkPath("src/FileManager.cpp"));
  const auto asset_manager = ReadTextFile(SdkPath("src/AssetManager.cpp"));
  ASSERT_FALSE(thumbnail_provider.empty());
  ASSERT_FALSE(file_manager.empty());
  ASSERT_FALSE(asset_manager.empty());

  EXPECT_NE(thumbnail_provider.find("project_folder / \"Cache/Thumbnail\""), std::string::npos);
  EXPECT_NE(thumbnail_provider.find("kThumbnailCacheVersion = 3"), std::string::npos);
  EXPECT_NE(thumbnail_provider.find("512ull * 1024ull * 1024ull"), std::string::npos);
  EXPECT_NE(thumbnail_provider.find("material->PeekTextureRefs()"), std::string::npos);
  EXPECT_NE(thumbnail_provider.find("std::filesystem::last_write_time"), std::string::npos);
  EXPECT_NE(thumbnail_provider.find("PruneThumbnailCache"), std::string::npos);
  EXPECT_NE(file_manager.find("kProjectThumbnailResolution = {256, 256}"), std::string::npos);
  EXPECT_NE(file_manager.find("kThumbnailGenerationBudget = std::chrono::milliseconds(8)"), std::string::npos);
  EXPECT_NE(asset_manager.find("OffscreenPreviewRenderer::Reset()"), std::string::npos);
  EXPECT_GT(
      std::filesystem::file_size(SdkPath("Internals/DefaultResources/Textures/MaterialPreview/neutral_studio.hdr")),
      1024u);
  EXPECT_NE(ReadTextFile(SdkPath("Internals/DefaultResources/Textures/MaterialPreview/LICENSE.txt")).find("CC0 1.0"),
            std::string::npos);
}

TEST(GltfRasterMaterial, RasterBackendUsesCanonicalBindlessTextureIndices) {
  const auto compatibility = ReadTextFile(ShaderPath("Modules/EvoEngine/GltfRasterMaterial.slang"));
  const auto source = ReadTextFile(ShaderPath("Modules/EvoEngine/GltfBindlessMaterial.slang"));
  ASSERT_FALSE(compatibility.empty());
  ASSERT_FALSE(source.empty());

  EXPECT_NE(compatibility.find("__exported import EvoEngine.GltfBindlessMaterial;"), std::string::npos);
  EXPECT_EQ(source.find("Sampler2D<float4> EE_GLTF_RASTER_"), std::string::npos);
  EXPECT_EQ(source.find("EE_GLTF_SAMPLE_FIXED_RASTER_TEXTURE"), std::string::npos);
  EXPECT_NE(source.find("import EvoEngine.Textures;"), std::string::npos);
  EXPECT_NE(source.find("EE_TEXTURE_2DS[NonUniformResourceIndex(texture_info.index)]"), std::string::npos);
  EXPECT_NE(source.find("material.pbr_base_color_texture"), std::string::npos);
  EXPECT_NE(source.find("material.pbr_metallic_roughness_texture"), std::string::npos);
  EXPECT_NE(source.find("material.normal_texture"), std::string::npos);
  EXPECT_NE(source.find("material.emissive_texture"), std::string::npos);
  EXPECT_NE(source.find("material.occlusion_texture"), std::string::npos);
  EXPECT_NE(source.find("material.clearcoat_texture"), std::string::npos);
  EXPECT_NE(source.find("material.clearcoat_roughness_texture"), std::string::npos);
  EXPECT_NE(source.find("material.clearcoat_normal_texture"), std::string::npos);
  EXPECT_NE(source.find("normal_vector.xy *= material.clearcoat_normal_texture_scale"), std::string::npos);
  EXPECT_NE(source.find("EE_GLTF_RASTER_COATED_EMISSION"), std::string::npos);
}

TEST(GltfRasterMaterial, ActiveRasterShadersUseGltfEvaluator) {
  const std::filesystem::path paths[] = {
      ShaderPath("Graphics/Fragment/Standard/StandardTransparent.slang"),
      ShaderPath("Graphics/Fragment/Standard/TransparentMotionVectors.slang"),
  };

  for (const auto& path : paths) {
    const auto source = ReadTextFile(path);
    ASSERT_FALSE(source.empty()) << path.string();
    EXPECT_NE(source.find("import EvoEngine.GltfRasterMaterial;"), std::string::npos) << path.string();
    EXPECT_TRUE(source.find("EE_EVALUATE_GLTF_RASTER_SURFACE") != std::string::npos ||
                source.find("EE_GLTF_RASTER_ALPHA_MASK_PASSES") != std::string::npos)
        << path.string();
    EXPECT_EQ(source.find(std::string("EE_MATERIAL") + "_PROPERTIES"), std::string::npos) << path.string();
    EXPECT_EQ(source.find(std::string("Material") + "Properties"), std::string::npos) << path.string();
  }
  const auto skinned_motion = ReadTextFile(ShaderPath("Graphics/Fragment/Standard/SkinnedMotionVectors.slang"));
  EXPECT_NE(skinned_motion.find("import EvoEngine.GltfAlphaMask;"), std::string::npos);
  EXPECT_NE(skinned_motion.find("EE_GLTF_ALPHA_MASK_PASSES"), std::string::npos);
  EXPECT_EQ(skinned_motion.find("EE_EVALUATE_GLTF_RASTER_SURFACE"), std::string::npos);
}

TEST(GltfRasterMaterial, PostProcessConsumersReadExpandedGBuffer) {
  const std::filesystem::path normal_paths[] = {
      ShaderPath("Compute/PostProcessing/SSRReflect.slang"),
      ShaderPath("Compute/PostProcessing/GTAO.slang"),
      ShaderPath("Graphics/Fragment/PostProcessing/SSRReflect.slang"),
  };
  for (const auto& path : normal_paths) {
    const auto source = ReadTextFile(path);
    ASSERT_FALSE(source.empty()) << path.string();
    EXPECT_NE(source.find("[[vk::binding(21,"), std::string::npos) << path.string();
    EXPECT_NE(source.find("Sampler2D inNormalRoughness"), std::string::npos) << path.string();
    EXPECT_TRUE(source.find("inNormalRoughness.SampleLevel") != std::string::npos ||
                source.find("inNormalRoughness.Sample") != std::string::npos)
        << path.string();
    EXPECT_EQ(source.find("Sampler2D inMaterial"), std::string::npos) << path.string();
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
    EXPECT_NE(source.find("[[vk::binding(21,"), std::string::npos) << path.string();
    EXPECT_NE(source.find("[[vk::binding(22,"), std::string::npos) << path.string();
    EXPECT_NE(source.find("Sampler2D inNormalRoughness"), std::string::npos) << path.string();
    EXPECT_NE(source.find("Sampler2D inPbrFlags"), std::string::npos) << path.string();
    EXPECT_NE(source.find("float4 normal_roughness = inNormalRoughness.SampleLevel"), std::string::npos)
        << path.string();
    EXPECT_NE(source.find("float4 pbr_flags = inPbrFlags.SampleLevel"), std::string::npos) << path.string();
    EXPECT_NE(source.find("float roughness = clamp(normal_roughness.a"), std::string::npos) << path.string();
    EXPECT_NE(source.find("float metallic = clamp(pbr_flags.x"), std::string::npos) << path.string();
    EXPECT_NE(source.find("reflected.rgb * (fresnel * brdf.x + float3(f90 * brdf.y))"), std::string::npos)
        << path.string();
    EXPECT_NE(source.find("original + (ssr_specular - environment.specular) * trace_confidence"), std::string::npos)
        << path.string();
    EXPECT_EQ(source.find("full_color_weight"), std::string::npos) << path.string();
    EXPECT_EQ(source.find("composition_mode"), std::string::npos) << path.string();
  }

  const auto ambient_occlusion_geometry = ReadTextFile(ShaderPath("Compute/PostProcessing/GTAO.slang"));
  const auto ambient_occlusion_source = ReadTextFile(SdkPath("src/AmbientOcclusion.cpp"));
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

TEST(GltfRasterMaterial, ScreenSpaceReflectionTraversalGuardsDegenerateRays) {
  const std::filesystem::path paths[] = {
      ShaderPath("Compute/PostProcessing/SSRReflect.slang"),
      ShaderPath("Graphics/Fragment/PostProcessing/SSRReflect.slang"),
  };

  for (const auto& path : paths) {
    const auto source = ReadTextFile(path);
    ASSERT_FALSE(source.empty()) << path.string();
    EXPECT_NE(source.find("float dominant_span = max(abs(pixel_delta.x), abs(pixel_delta.y))"), std::string::npos)
        << path.string();
    EXPECT_NE(source.find("EE_SSR_CONSTANTS.start_bias"), std::string::npos) << path.string();
    EXPECT_NE(source.find("EE_SSR_CONSTANTS.binary_search_iteration_count"), std::string::npos) << path.string();
    EXPECT_NE(source.find("bool bracket_found = false"), std::string::npos) << path.string();
    EXPECT_NE(source.find("all(isfinite(clip))"), std::string::npos) << path.string();
    EXPECT_EQ(source.find("(frag.y - start_frag.y) / delta_y"), std::string::npos) << path.string();
  }
}

TEST(GltfRasterMaterial, ScreenSpaceReflectionBuildsExplicitGeometricConfidence) {
  const std::filesystem::path reflect_paths[] = {
      ShaderPath("Compute/PostProcessing/SSRReflect.slang"),
      ShaderPath("Graphics/Fragment/PostProcessing/SSRReflect.slang"),
  };
  for (const auto& path : reflect_paths) {
    const auto source = ReadTextFile(path);
    ASSERT_FALSE(source.empty()) << path.string();
    EXPECT_NE(source.find("float thickness_confidence"), std::string::npos) << path.string();
    EXPECT_NE(source.find("float distance_confidence"), std::string::npos) << path.string();
    EXPECT_NE(source.find("float grazing_confidence"), std::string::npos) << path.string();
    EXPECT_NE(source.find("float edge_confidence"), std::string::npos) << path.string();
    EXPECT_NE(source.find("float normal_confidence"), std::string::npos) << path.string();
    EXPECT_NE(source.find("float hit_facing = dot(hit_view_normal, -pivot)"), std::string::npos) << path.string();
    EXPECT_NE(source.find("EncodeHit(sample_uv, normalized_distance, confidence"), std::string::npos) << path.string();
    EXPECT_NE(source.find("EE_SSR_CONSTANTS.debug_mode"), std::string::npos) << path.string();
  }
}

TEST(GltfRasterMaterial, ScreenSpaceReflectionUsesEdgeAwareSpatialResolve) {
  const auto source = ReadTextFile(ShaderPath("Compute/PostProcessing/SSRSpatialResolve.slang"));
  ASSERT_FALSE(source.empty());
  EXPECT_NE(source.find("center.a <= 0.0001f"), std::string::npos);
  EXPECT_NE(source.find("center.a >= 0.85f"), std::string::npos);
  EXPECT_NE(source.find("float3 radiance_sum = center.rgb"), std::string::npos);
  EXPECT_NE(source.find("float weight_sum = 1.0f"), std::string::npos);
  EXPECT_NE(source.find("float depth_weight"), std::string::npos);
  EXPECT_NE(source.find("float normal_weight"), std::string::npos);
  EXPECT_NE(source.find("float roughness_weight"), std::string::npos);
  EXPECT_NE(source.find("sample_reflection.a"), std::string::npos);

  const auto implementation = ReadTextFile(SdkPath("src/ScreenSpaceReflection.cpp"));
  ASSERT_FALSE(implementation.empty());
  EXPECT_NE(implementation.find("SSRSpatialResolve.slang"), std::string::npos);
  EXPECT_EQ(implementation.find("GaussianBlur("), std::string::npos);
}

TEST(GltfRasterMaterial, ScreenSpaceReflectionTemporalResolveRejectsInvalidHistory) {
  const auto source = ReadTextFile(ShaderPath("Compute/PostProcessing/SSRTemporalResolve.slang"));
  ASSERT_FALSE(source.empty());
  EXPECT_NE(source.find("motion.xy * texel_size"), std::string::npos);
  EXPECT_NE(source.find("expected_history_depth = linear_depth + motion.z"), std::string::npos);
  EXPECT_NE(source.find("dot(normal, history_geometry.xyz) >= 0.9f"), std::string::npos);
  EXPECT_NE(source.find("material_valid"), std::string::npos);
  EXPECT_NE(source.find("clamp(history.rgb, neighborhood_min, neighborhood_max)"), std::string::npos);
  EXPECT_NE(source.find("EE_SSR_CONSTANTS.temporal_history_valid"), std::string::npos);

  const auto pass = ReadTextFile(SdkPath("src/RenderPasses/PostProcessingPass.cpp"));
  ASSERT_FALSE(pass.empty());
  EXPECT_NE(pass.find("RenderResourceNames::camera_motion_vectors"), std::string::npos);
  EXPECT_NE(pass.find("motion_vectors_view"), std::string::npos);
}

TEST(GltfRasterMaterial, FusedDeferredComputeResolvesAndLightsExpandedGBuffer) {
  const auto opaque = ReadTextFile(ShaderPath("Graphics/Fragment/Standard/StandardDeferredRaw.slang"));
  const auto masked = ReadTextFile(ShaderPath("Graphics/Fragment/Standard/StandardDeferredMaskedRaw.slang"));
  const auto resolve = ReadTextFile(ShaderPath("Compute/DeferredComputeLighting.slang"));
  ASSERT_FALSE(opaque.empty());
  ASSERT_FALSE(masked.empty());
  ASSERT_FALSE(resolve.empty());

  EXPECT_NE(opaque.find("import EvoEngine.RawGBuffer;"), std::string::npos);
  EXPECT_NE(opaque.find("EE_BUILD_RAW_GBUFFER"), std::string::npos);
  EXPECT_EQ(opaque.find("GltfAlphaMask"), std::string::npos);
  EXPECT_EQ(opaque.find("EE_EVALUATE_GLTF"), std::string::npos);
  EXPECT_EQ(opaque.find("Sample"), std::string::npos);
  EXPECT_NE(masked.find("import EvoEngine.GltfAlphaMask;"), std::string::npos);
  EXPECT_NE(masked.find("EE_GLTF_ALPHA_MASK_PASSES"), std::string::npos);
  EXPECT_EQ(masked.find("EE_EVALUATE_GLTF_RASTER_SURFACE"), std::string::npos);
  EXPECT_EQ(masked.find("EE_EVALUATE_GLTF_RASTER_NORMAL"), std::string::npos);

  EXPECT_NE(resolve.find("EE_EVALUATE_GLTF_RASTER_SURFACE"), std::string::npos);
  EXPECT_NE(resolve.find("EE_EVALUATE_GLTF_RASTER_NORMAL"), std::string::npos);
  EXPECT_NE(resolve.find("EE_GLTF_RASTER_COATED_EMISSION"), std::string::npos);
  EXPECT_NE(resolve.find("surface.specular_f0 ="), std::string::npos);
  EXPECT_NE(resolve.find("EE_GLTF_RASTER_REBASE_SPECULAR_F0"), std::string::npos);
  EXPECT_NE(resolve.find("if (!isfinite(depth) || depth >= 1.0f)"), std::string::npos);
  EXPECT_EQ(resolve.find("packed_metadata.x == EE_GBUFFER_CLEAR_VALUE"), std::string::npos);
  for (uint32_t binding = 0; binding <= 5; ++binding) {
    EXPECT_NE(resolve.find("[[vk::binding(" + std::to_string(binding) + ", 4)]]"), std::string::npos) << binding;
  }
  EXPECT_NE(resolve.find("[[vk::binding(4, 3)]]"), std::string::npos);
  EXPECT_NE(resolve.find("EE_FUNC_CALCULATE_LIGHTS"), std::string::npos);
  EXPECT_NE(resolve.find("EE_FUNC_CALCULATE_DDGI_ENVIRONMENTAL_LIGHT"), std::string::npos);
  EXPECT_NE(resolve.find("outColor[pixel]"), std::string::npos);
  EXPECT_NE(resolve.find("inOutUvBaseColorAo[pixel] = float4(resolved_base_color"), std::string::npos);
  EXPECT_NE(resolve.find("inAmbientOcclusion.SampleLevel(tex_coord, 0.0f).r"), std::string::npos);
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
  EXPECT_NE(platform.find("g_buffer_utility = raw_g_buffer::kMetadataFormat"), std::string::npos);
  EXPECT_EQ(platform.find("g_buffer_color"), std::string::npos);
  EXPECT_EQ(platform.find("g_buffer_material"), std::string::npos);
  EXPECT_NE(render_layer.find("CreateDeferredGBufferColorAttachmentFormats"), std::string::npos);
  EXPECT_EQ(render_layer.find("camera_g_buffer_layout_->PushDescriptorBinding(18"), std::string::npos);
  EXPECT_EQ(render_layer.find("camera_g_buffer_layout_->PushDescriptorBinding(19"), std::string::npos);
  EXPECT_NE(render_layer.find("PushDescriptorBinding(24, VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE"), std::string::npos);

  for (uint32_t binding = 20; binding <= 24; binding++) {
    EXPECT_NE(render_layer.find("PushDescriptorBinding(" + std::to_string(binding)), std::string::npos);
    EXPECT_NE(camera.find("UpdateImageDescriptorBinding(" + std::to_string(binding)), std::string::npos);
  }
  EXPECT_EQ(camera.find("UpdateImageDescriptorBinding(18"), std::string::npos);
  EXPECT_EQ(camera.find("UpdateImageDescriptorBinding(19"), std::string::npos);

  EXPECT_NE(camera.find("AppendGBufferAttachmentInfo(attachment_infos, attachment, g_buffer_base_color_ao_view_, "
                        "clear_value)"),
            std::string::npos);
  EXPECT_NE(camera.find("AppendGBufferAttachmentInfo(attachment_infos, attachment, g_buffer_utility_view_, "
                        "clear_value)"),
            std::string::npos);
  EXPECT_NE(camera.find("clear_value.color.uint32[0] = raw_g_buffer::kClearValue"), std::string::npos);
  EXPECT_NE(camera.find("image_info.sampler = VK_NULL_HANDLE"), std::string::npos);
  EXPECT_EQ(camera.find("g_buffer_material_"), std::string::npos);
  EXPECT_EQ(camera_header.find("g_buffer_material_"), std::string::npos);
  EXPECT_NE(editor.find("GetGBufferUtilityImage()"), std::string::npos);
  EXPECT_NE(editor.find("static_cast<const uint32_t*>(mapped_entity_index_data_)[0]"), std::string::npos);
  EXPECT_NE(editor.find("instance_index != raw_g_buffer::kClearValue"), std::string::npos);
  EXPECT_EQ(editor.find("GetGBufferNormalImage()"), std::string::npos);
  EXPECT_EQ(inspection.find("GetGBufferMaterialTexCoordImTextureId"), std::string::npos);
  EXPECT_EQ(inspection.find("GetGBufferMaterialIndicesImTextureId"), std::string::npos);
  EXPECT_EQ(camera_header.find("GetGBufferUtilityImTextureId"), std::string::npos);
  EXPECT_NE(inspection.find("Integer instance, material, info, flags, and packed vertex-color data"),
            std::string::npos);

  const auto motion_vectors = ReadTextFile(ShaderPath("Compute/MotionVectors.slang"));
  const auto deferred_lighting = ReadTextFile(ShaderPath("Compute/DeferredComputeLighting.slang"));
  const auto selection = ReadTextFile(ShaderPath("Graphics/Fragment/PostProcessing/EntitySelectionHighlight.slang"));
  for (const auto* source : {&motion_vectors, &selection}) {
    EXPECT_NE(source->find("Texture2D<uint4> inUtility"), std::string::npos);
    EXPECT_EQ(source->find("Sampler2D inUtility"), std::string::npos);
    EXPECT_EQ(source->find("Sampler2D<float4> inUtility"), std::string::npos);
  }
  EXPECT_NE(motion_vectors.find("uint instance_index = inUtility.Load(int3(texel, 0)).x"), std::string::npos);
  EXPECT_NE(deferred_lighting.find("Texture2D<uint4> inMetadata"), std::string::npos);
  EXPECT_NE(deferred_lighting.find("const uint4 packed_metadata = inMetadata.Load"), std::string::npos);
  EXPECT_NE(deferred_lighting.find("metadata.material_index"), std::string::npos);
  EXPECT_NE(deferred_lighting.find("metadata.info_index"), std::string::npos);
  EXPECT_NE(selection.find("info_index != EE_GBUFFER_CLEAR_VALUE && (info_index & 1u) != 0u"), std::string::npos);

  const auto render_graph = ReadTextFile(SdkPath("src/RenderGraph.cpp"));
  EXPECT_NE(render_graph.find("format_name == \"GBufferMetadata\""), std::string::npos);
}

TEST(GltfRasterMaterial, EcoSysLabDeferredShadersWriteStrictRawGeometry) {
  const std::filesystem::path opaque_shader_paths[] = {
      RepoPath("EvoEngine_Packages/EcoSysLab/Internals/EcoSysLabResources/Shaders/Graphics/Fragment/DynamicStrands/"
               "Rendering/Foliage.slang"),
      RepoPath("EvoEngine_Packages/EcoSysLab/Internals/EcoSysLabResources/Shaders/Graphics/Fragment/DynamicStrands/"
               "Rendering/SmallSegments.slang"),
      RepoPath("EvoEngine_Packages/EcoSysLab/Internals/EcoSysLabResources/Shaders/Graphics/Fragment/DynamicStrands/"
               "Rendering/SmallSegmentsVisualization.slang"),
      RepoPath("EvoEngine_Packages/EcoSysLab/Internals/EcoSysLabResources/Shaders/Graphics/Fragment/DynamicStrands/"
               "Rendering/AlphaShapeMeshing/Branches.slang"),
      RepoPath("EvoEngine_Packages/EcoSysLab/Internals/EcoSysLabResources/Shaders/Graphics/Fragment/DynamicStrands/"
               "Rendering/KineticVoronoiMeshing/Branches.slang"),
  };

  for (const auto& path : opaque_shader_paths) {
    const auto source = ReadTextFile(path);
    ASSERT_FALSE(source.empty()) << path.string();
    EXPECT_NE(source.find("import EvoEngine.RawGBuffer;"), std::string::npos) << path.string();
    EXPECT_NE(source.find("EE_BUILD_RAW_GBUFFER"), std::string::npos) << path.string();
    EXPECT_EQ(source.find("GltfAlphaMask"), std::string::npos) << path.string();
    EXPECT_EQ(source.find("EE_EVALUATE_GLTF"), std::string::npos) << path.string();
    EXPECT_EQ(source.find("Sample"), std::string::npos) << path.string();
  }

  const std::filesystem::path masked_shader_paths[] = {
      RepoPath("EvoEngine_Packages/EcoSysLab/Internals/EcoSysLabResources/Shaders/Graphics/Fragment/DynamicStrands/"
               "Rendering/FoliageMasked.slang"),
      RepoPath("EvoEngine_Packages/EcoSysLab/Internals/EcoSysLabResources/Shaders/Graphics/Fragment/DynamicStrands/"
               "Rendering/SmallSegmentsMasked.slang"),
      RepoPath("EvoEngine_Packages/EcoSysLab/Internals/EcoSysLabResources/Shaders/Graphics/Fragment/DynamicStrands/"
               "Rendering/SmallSegmentsVisualizationMasked.slang"),
      RepoPath("EvoEngine_Packages/EcoSysLab/Internals/EcoSysLabResources/Shaders/Graphics/Fragment/DynamicStrands/"
               "Rendering/AlphaShapeMeshing/BranchesMasked.slang"),
      RepoPath("EvoEngine_Packages/EcoSysLab/Internals/EcoSysLabResources/Shaders/Graphics/Fragment/DynamicStrands/"
               "Rendering/KineticVoronoiMeshing/BranchesMasked.slang"),
  };
  for (const auto& path : masked_shader_paths) {
    const auto source = ReadTextFile(path);
    ASSERT_FALSE(source.empty()) << path.string();
    EXPECT_NE(source.find("import EvoEngine.RawGBuffer;"), std::string::npos) << path.string();
    EXPECT_NE(source.find("import EvoEngine.GltfAlphaMask;"), std::string::npos) << path.string();
    EXPECT_NE(source.find("EE_GLTF_ALPHA_MASK_PASSES"), std::string::npos) << path.string();
    EXPECT_NE(source.find("EE_BUILD_RAW_GBUFFER"), std::string::npos) << path.string();
    EXPECT_EQ(source.find("EE_EVALUATE_GLTF_RASTER_SURFACE"), std::string::npos) << path.string();
    EXPECT_EQ(source.find("EE_EVALUATE_GLTF_RASTER_NORMAL"), std::string::npos) << path.string();
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
    EXPECT_NE(source.find("Masked.slang"), std::string::npos) << path.string();
  }

  const auto alpha_shape_registration =
      ReadTextFile(RepoPath("EvoEngine_Packages/EcoSysLab/src/DsAlphaShapeMeshing.cpp"));
  const auto kinetic_registration =
      ReadTextFile(RepoPath("EvoEngine_Packages/EcoSysLab/src/DsKineticVoronoiMeshing.cpp"));
  const auto foliage_registration = ReadTextFile(RepoPath("EvoEngine_Packages/EcoSysLab/src/DynamicTreeStrands.cpp"));
  for (const auto* source : {&alpha_shape_registration, &kinetic_registration, &foliage_registration}) {
    ASSERT_FALSE(source->empty());
    EXPECT_NE(source->find("RawOpaqueRenderingAllCameras"), std::string::npos);
    EXPECT_NE(source->find("AlphaMaskedRenderingAllCameras"), std::string::npos);
    EXPECT_EQ(source->find("DeferredRenderingAllCameras"), std::string::npos);
  }
}

TEST(GltfRasterMaterial, EcoSysLabTaskMeshPayloadsAndGraphicsInterfacesMatch) {
  const auto root = RepoPath("EvoEngine_Packages/EcoSysLab/Internals/EcoSysLabResources/Shaders/Graphics");
  struct PipelineFamily {
    const char* task;
    const char* mesh;
  };
  const PipelineFamily families[] = {
      {"Task/DynamicStrands/Rendering/AlphaShapeMeshing/Branches.slang",
       "Mesh/DynamicStrands/Rendering/AlphaShapeMeshing/Branches/Rendering.slang"},
      {"Task/DynamicStrands/Rendering/AlphaShapeMeshing/SmallSegments.slang",
       "Mesh/DynamicStrands/Rendering/AlphaShapeMeshing/SmallSegments/Rendering.slang"},
      {"Task/DynamicStrands/Rendering/AlphaShapeMeshing/SmallSegmentsVisualization.slang",
       "Mesh/DynamicStrands/Rendering/AlphaShapeMeshing/SmallSegments/VisualizationRendering.slang"},
      {"Task/DynamicStrands/Rendering/Foliage.slang", "Mesh/DynamicStrands/Rendering/Foliage/Rendering.slang"},
      {"Task/DynamicStrands/Rendering/KineticVoronoiMeshing/SegmentMeshlet.slang",
       "Mesh/DynamicStrands/Rendering/KineticVoronoiMeshing/SegmentMeshlet/Rendering.slang"},
      {"Task/DynamicStrands/Rendering/SegmentPairs.slang",
       "Mesh/DynamicStrands/Rendering/SegmentPairs/Rendering.slang"},
      {"Task/DynamicStrands/Visualization/AlphaShapeMeshing/UniformParticles.slang",
       "Mesh/DynamicStrands/Visualization/AlphaShapeMeshing/UniformParticles.slang"},
      {"Task/DynamicStrands/Visualization/Foliage.slang", "Mesh/DynamicStrands/Visualization/Foliage.slang"},
      {"Task/DynamicStrands/Visualization/KineticVoronoiMeshing/Vertices.slang",
       "Mesh/DynamicStrands/Visualization/KineticVoronoiMeshing/Vertices.slang"},
      {"Task/DynamicStrands/Visualization/SegmentPairs.slang", "Mesh/DynamicStrands/Visualization/SegmentPairs.slang"},
      {"Task/DynamicStrands/Visualization/Segments.slang", "Mesh/DynamicStrands/Visualization/Segments.slang"},
  };

  for (const auto& family : families) {
    const auto task_path = root / family.task;
    const auto mesh_path = root / family.mesh;
    const auto task_source = ReadTextFile(task_path);
    const auto mesh_source = ReadTextFile(mesh_path);
    ASSERT_FALSE(task_source.empty()) << task_path.string();
    ASSERT_FALSE(mesh_source.empty()) << mesh_path.string();
    const auto task_payload = ExtractStructIgnoringWhitespace(task_source, "Task");
    ASSERT_FALSE(task_payload.empty()) << task_path.string();
    EXPECT_EQ(task_payload, ExtractStructIgnoringWhitespace(mesh_source, "Task"))
        << task_path.string() << " -> " << mesh_path.string();
    EXPECT_NE(task_source.find("DispatchMesh("), std::string::npos) << task_path.string();
    EXPECT_NE(mesh_source.find("OutputVertices<"), std::string::npos) << mesh_path.string();
    EXPECT_NE(mesh_source.find("OutputIndices<"), std::string::npos) << mesh_path.string();
  }

  size_t mesh_count = 0;
  size_t directional_shadow_count = 0;
  size_t point_shadow_count = 0;
  size_t spot_shadow_count = 0;
  for (const auto& entry : std::filesystem::recursive_directory_iterator(root / "Mesh")) {
    if (!entry.is_regular_file() || entry.path().extension() != ".slang") {
      continue;
    }
    const auto source = ReadTextFile(entry.path());
    ASSERT_FALSE(source.empty()) << entry.path().string();
    EXPECT_EQ(source.find("position = mul(transform, float4"), std::string::npos) << entry.path().string();
    EXPECT_EQ(source.find("position = mul(instance_matrix, float4"), std::string::npos) << entry.path().string();
    EXPECT_EQ(source.find("mul(mul(translate("), std::string::npos) << entry.path().string();
    ++mesh_count;
    const auto filename = entry.path().filename().string();
    directional_shadow_count += filename == "DirectionalLightShadowMap.slang";
    point_shadow_count += filename == "PointLightShadowMap.slang";
    spot_shadow_count += filename == "SpotLightShadowMap.slang";
  }
  EXPECT_EQ(mesh_count, 23u);
  EXPECT_EQ(directional_shadow_count, 4u);
  EXPECT_EQ(point_shadow_count, 4u);
  EXPECT_EQ(spot_shadow_count, 4u);

  const auto alpha_mesh =
      ReadTextFile(root / "Mesh/DynamicStrands/Rendering/AlphaShapeMeshing/Branches/Rendering.slang");
  const auto alpha_fragment = ReadTextFile(root / "Fragment/DynamicStrands/Rendering/AlphaShapeMeshing/Branches.slang");
  const auto kinetic_mesh =
      ReadTextFile(root / "Mesh/DynamicStrands/Rendering/KineticVoronoiMeshing/SegmentMeshlet/Rendering.slang");
  const auto kinetic_fragment =
      ReadTextFile(root / "Fragment/DynamicStrands/Rendering/KineticVoronoiMeshing/Branches.slang");
  for (const auto* source : {&alpha_mesh, &alpha_fragment, &kinetic_mesh, &kinetic_fragment}) {
    EXPECT_NE(source->find("[[vk::location(5)]] nointerpolation int MaterialIndex"), std::string::npos);
  }
  EXPECT_NE(alpha_fragment.find("input.MaterialIndex"), std::string::npos);
  EXPECT_NE(kinetic_fragment.find("input.MaterialIndex"), std::string::npos);
  EXPECT_EQ(alpha_fragment.find("EcoSysLabPrimitiveInput"), std::string::npos);
  EXPECT_EQ(kinetic_fragment.find("EcoSysLabPrimitiveInput"), std::string::npos);

  const auto branch_renderer =
      ReadTextFile(RepoPath("EvoEngine_Packages/EcoSysLab/src/DsAlphaShapeBranchesRendering.cpp"));
  EXPECT_NE(branch_renderer.find("render_push_constant.bark_material_index = bark_material_index;"), std::string::npos);
}

TEST(GltfRasterMaterial, ActiveRasterNormalMapsUseTangentHandedness) {
  const auto geometry = ReadTextFile(SdkPath("src/IGeometry.cpp"));
  const auto standard = ReadTextFile(ShaderPath("Graphics/Vertex/Standard/Standard.slang"));
  const auto standard_instanced = ReadTextFile(ShaderPath("Graphics/Vertex/Standard/StandardInstanced.slang"));
  const auto standard_skinned = ReadTextFile(ShaderPath("Graphics/Vertex/Standard/StandardSkinned.slang"));
  const auto standard_mesh = ReadTextFile(ShaderPath("Graphics/Mesh/Standard/Standard.slang"));
  const auto standard_meshlet_colored = ReadTextFile(ShaderPath("Graphics/Mesh/Standard/StandardMeshletColored.slang"));
  const auto standard_strands = ReadTextFile(ShaderPath("Graphics/Mesh/Standard/StandardStrands.slang"));
  const auto instances = ReadTextFile(ShaderPath("Modules/EvoEngine/Instances.slang"));
  const auto deferred = ReadTextFile(ShaderPath("Graphics/Fragment/Standard/StandardDeferredRaw.slang"));
  const auto transparent = ReadTextFile(ShaderPath("Graphics/Fragment/Standard/StandardTransparent.slang"));
  const auto raster_material = ReadTextFile(ShaderPath("Modules/EvoEngine/GltfBindlessMaterial.slang"));
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
  EXPECT_EQ(geometry.find("tex_coord_2"), std::string::npos);
  EXPECT_EQ(geometry.find("tex_coord_3"), std::string::npos);

  for (const auto* source : {&standard, &standard_instanced, &standard_skinned}) {
    EXPECT_NE(source->find("[[vk::location(9)]] float tangent_handedness"), std::string::npos);
    EXPECT_NE(source->find("[[vk::location(10)]] float2 tex_coord1"), std::string::npos);
    EXPECT_EQ(source->find("tex_coord2"), std::string::npos);
    EXPECT_EQ(source->find("tex_coord3"), std::string::npos);
    EXPECT_NE(source->find("input.tangent_handedness"), std::string::npos);
    EXPECT_NE(source->find("input.tex_coord1"), std::string::npos);
    EXPECT_NE(source->find("input.color"), std::string::npos);
    EXPECT_EQ(source->find("T = normalize(T - dot(T, N) * N)"), std::string::npos);
  }

  for (const auto* source : {&standard_mesh, &standard_meshlet_colored}) {
    EXPECT_NE(source->find("import EvoEngine.RasterVertex;"), std::string::npos);
    EXPECT_NE(source->find("EE_BUILD_STANDARD_RASTER_OUTPUT"), std::string::npos);
    EXPECT_NE(source->find("vertex.vertex_info3"), std::string::npos);
    EXPECT_NE(source->find("vertex.tex_coord_1"), std::string::npos);
    EXPECT_EQ(source->find("vertex.tex_coord_2"), std::string::npos);
    EXPECT_EQ(source->find("vertex.tex_coord_3"), std::string::npos);
    EXPECT_NE(source->find("vertex.color"), std::string::npos);
    EXPECT_EQ(source->find("T = normalize(T - dot(T, N) * N)"), std::string::npos);
  }

  EXPECT_NE(deferred.find("input.tangent_handedness"), std::string::npos);
  EXPECT_NE(deferred.find("input.tex_coord01.zw"), std::string::npos);
  EXPECT_EQ(deferred.find("input.tex_coord23"), std::string::npos);
  EXPECT_NE(deferred.find("input.color"), std::string::npos);
  EXPECT_EQ(deferred.find("EE_EVALUATE_GLTF_RASTER_NORMAL(material_index, tex_coord, tex_coord, fs_in.Normal, "
                          "fs_in.Tangent)"),
            std::string::npos);
  EXPECT_NE(raster_material.find("EE_GLTF_SAFE_NORMALIZE"), std::string::npos);
  EXPECT_NE(raster_material.find("tangent - n * dot(n, tangent)"), std::string::npos);
  EXPECT_NE(raster_material.find("EE_GLTF_FALLBACK_TANGENT(n)"), std::string::npos);
  EXPECT_NE(raster_material.find("float3 b = cross(n, t) *"), std::string::npos);
  EXPECT_NE(standard_strands.find("EE_BUILD_STANDARD_RASTER_OUTPUT"), std::string::npos);
  EXPECT_NE(standard_strands.find("vertices[vertex].tangent_handedness = handedness"), std::string::npos);
  EXPECT_NE(standard_strands.find("tex_coord, tex_coord, endpoint ? color1 : color0"), std::string::npos);
  for (const auto* source : {&deferred, &transparent}) {
    EXPECT_NE(source->find("bool is_front_face : SV_IsFrontFace"), std::string::npos);
    EXPECT_NE(source->find("(is_front_face ? 1.0f : -1.0f) * input.transform_handedness"), std::string::npos);
  }
  EXPECT_EQ(deferred.find("EE_GLTF_MATERIALS"), std::string::npos);
  EXPECT_EQ(deferred.find("EE_EVALUATE_GLTF"), std::string::npos);
  EXPECT_NE(transparent.find("EE_GLTF_MATERIALS[material_index].double_sided == 0 && facing_sign < 0.0"),
            std::string::npos);
  EXPECT_NE(instances.find("float EE_TRANSFORM_HANDEDNESS(float4x4 transform)"), std::string::npos);
  EXPECT_NE(instances.find("determinant(float3x3(transform)) < 0.0 ? -1.0 : 1.0"), std::string::npos);
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
  EXPECT_NE(geometry_source.find("mesh_motion_vectors = {mesh[0], mesh[3], mesh[4], mesh[6]}"), std::string::npos);

  const auto shadow_pipeline =
      ExtractSourceRange(render_layer, "std::shared_ptr<GraphicsPipeline> CreateShadowVertexPipeline",
                         "std::shared_ptr<GraphicsPipeline> CreateStrandShadowMeshPipeline");
  ASSERT_FALSE(shadow_pipeline.empty());
  EXPECT_TRUE(ContainsIgnoringWhitespace(
      shadow_pipeline, "alpha_masked ? VertexInputAttributeSet::MotionVectors : VertexInputAttributeSet::Position"));

  EXPECT_NE(platform.find("graphics.render_texture_present_pipeline->vertex_input_attribute_set = "
                          "VertexInputAttributeSet::PositionTexCoord"),
            std::string::npos);
  for (const auto* pipeline : {"environmental_brdf_pipeline"}) {
    EXPECT_TRUE(ContainsIgnoringWhitespace(render_layer, std::string(pipeline) +
                                                             "->vertex_input_attribute_set = "
                                                             "VertexInputAttributeSet::PositionTexCoord"))
        << pipeline;
  }
  for (const auto* pipeline : {"transparent_motion_vectors_pipeline_"}) {
    EXPECT_TRUE(ContainsIgnoringWhitespace(render_layer, std::string(pipeline) +
                                                             "->vertex_input_attribute_set = "
                                                             "VertexInputAttributeSet::MotionVectors"))
        << pipeline;
  }
  EXPECT_NE(render_layer.find("pipeline->vertex_input_attribute_set = VertexInputAttributeSet::MotionVectors"),
            std::string::npos);
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
  EXPECT_NE(global_reflection_probe.find("pipeline->vertex_input_attribute_set = VertexInputAttributeSet::Position"),
            std::string::npos);
  EXPECT_NE(light_probe.find("irradiance_construct_pipeline_->vertex_input_attribute_set = "
                             "VertexInputAttributeSet::Position"),
            std::string::npos);

  const std::filesystem::path rigid_shadow_shaders[] = {
      ShaderPath("Graphics/Vertex/Lighting/PointLightShadowMap.slang"),
      ShaderPath("Graphics/Vertex/Lighting/SpotLightShadowMap.slang"),
      ShaderPath("Graphics/Vertex/Lighting/DirectionalLightShadowMap.slang"),
      ShaderPath("Graphics/Vertex/Lighting/PointLightShadowMapInstanced.slang"),
      ShaderPath("Graphics/Vertex/Lighting/SpotLightShadowMapInstanced.slang"),
      ShaderPath("Graphics/Vertex/Lighting/DirectionalLightShadowMapInstanced.slang"),
  };
  for (const auto& shader_path : rigid_shadow_shaders) {
    const auto shader = ReadTextFile(shader_path);
    ASSERT_FALSE(shader.empty()) << shader_path.string();
    ExpectShaderInputLocations(shader, shader_path, {0, 3, 4, 10});
    EXPECT_EQ(shader.find("out VS_OUT"), std::string::npos) << shader_path.string();
    EXPECT_EQ(shader.find("currentInstanceIndex"), std::string::npos) << shader_path.string();
  }
  ExpectShaderInputLocations(ReadTextFile(ShaderPath("Graphics/Vertex/Lighting/AtmosphereToCubemap.slang")),
                             ShaderPath("Graphics/Vertex/Lighting/AtmosphereToCubemap.slang"), {0});
  ExpectShaderInputLocations(ReadTextFile(ShaderPath("Graphics/Vertex/Lighting/CubemapProcess.slang")),
                             ShaderPath("Graphics/Vertex/Lighting/CubemapProcess.slang"), {0});

  const std::filesystem::path skinned_shadow_shaders[] = {
      ShaderPath("Graphics/Vertex/Lighting/PointLightShadowMapSkinned.slang"),
      ShaderPath("Graphics/Vertex/Lighting/SpotLightShadowMapSkinned.slang"),
      ShaderPath("Graphics/Vertex/Lighting/DirectionalLightShadowMapSkinned.slang"),
  };
  for (const auto& shader_path : skinned_shadow_shaders) {
    const auto shader = ReadTextFile(shader_path);
    ASSERT_FALSE(shader.empty()) << shader_path.string();
    ExpectShaderInputLocations(shader, shader_path, {0, 3, 4, 5, 6, 7, 8, 10});
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
  EXPECT_EQ(prefab_source.find("HasTextureCoords(2)"), std::string::npos);
  EXPECT_EQ(prefab_source.find("HasTextureCoords(3)"), std::string::npos);
  EXPECT_NE(prefab_source.find("glm::vec4(color.r, color.g, color.b, color.a)"), std::string::npos);
  EXPECT_NE(prefab_source.find("ReadImportedTexCoord(importer_mesh, 1"), std::string::npos);
  EXPECT_NE(prefab_source.find("bool parsed_gltf = false"), std::string::npos);
  EXPECT_NE(prefab_source.find("(extension == \".gltf\" || extension == \".glb\") && parsed_gltf"), std::string::npos);
  EXPECT_EQ(prefab_source.find("extension == \".gltf\" && !gltf_material_data.empty()"), std::string::npos);
}
