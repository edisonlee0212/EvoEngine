#include "RenderInstanceStorage.hpp"

#include "Application.hpp"
#include "EditorLayer.hpp"
#include "EnvironmentalLightingResolver.hpp"
#include "EnvironmentalMap.hpp"
#include "GlobalReflectionProbe.hpp"
#include "LodGroup.hpp"
#include "Platform.hpp"
#include "RenderLayer.hpp"
#include "Resources.hpp"
#include "Texture2D.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <functional>
#include <glm/gtc/constants.hpp>
#include <limits>
#include <unordered_map>

using namespace evo_engine;

namespace {
uint64_t MixDdgiInventorySignature(const uint64_t seed, const uint64_t value) {
  return seed ^ (value + 0x9e3779b97f4a7c15ull + (seed << 6u) + (seed >> 2u));
}

float DecodeSrgbChannel(const float value) {
  return value <= 0.04045f ? value / 12.92f : std::pow((value + 0.055f) / 1.055f, 2.4f);
}

int WrapTextureCoordinate(const int coordinate, const int size, const VkSamplerAddressMode mode) {
  if (size <= 1) {
    return 0;
  }
  if (mode == VK_SAMPLER_ADDRESS_MODE_REPEAT) {
    return ((coordinate % size) + size) % size;
  }
  if (mode == VK_SAMPLER_ADDRESS_MODE_MIRRORED_REPEAT) {
    const int period = size * 2;
    const int wrapped = ((coordinate % period) + period) % period;
    return wrapped < size ? wrapped : period - wrapped - 1;
  }
  return glm::clamp(coordinate, 0, size - 1);
}

glm::vec2 SelectTexCoord(const Vertex& vertex, const int32_t tex_coord) {
  if (tex_coord == 1) {
    return vertex.tex_coord_1;
  }
  if (tex_coord == 2) {
    return vertex.tex_coord_2;
  }
  if (tex_coord == 3) {
    return vertex.tex_coord_3;
  }
  return vertex.tex_coord;
}

constexpr std::array<glm::vec3, 7> kTriangleImportanceSamples = {glm::vec3(1.0f / 3.0f),
                                                                 glm::vec3(2.0f / 3.0f, 1.0f / 6.0f, 1.0f / 6.0f),
                                                                 glm::vec3(1.0f / 6.0f, 2.0f / 3.0f, 1.0f / 6.0f),
                                                                 glm::vec3(1.0f / 6.0f, 1.0f / 6.0f, 2.0f / 3.0f),
                                                                 glm::vec3(0.5f, 0.5f, 0.0f),
                                                                 glm::vec3(0.0f, 0.5f, 0.5f),
                                                                 glm::vec3(0.5f, 0.0f, 0.5f)};

glm::vec4 SampleLocalTexture(const Texture2D& texture, const glm::vec2 uv) {
  const auto& pixels = texture.PeekLocalData();
  const auto resolution = texture.GetResolution();
  if (pixels.size() != static_cast<size_t>(resolution.x) * resolution.y || resolution.x == 0u || resolution.y == 0u) {
    return glm::vec4(1.0f);
  }
  const auto& sampler = texture.GetSamplerSettings();
  const glm::vec2 texel = uv * glm::vec2(resolution) - 0.5f;
  const glm::ivec2 base = glm::ivec2(glm::floor(texel));
  const glm::vec2 blend = glm::fract(texel);
  const auto fetch = [&](const int x, const int y) {
    const int wrapped_x = WrapTextureCoordinate(x, static_cast<int>(resolution.x), sampler.address_mode_u);
    const int wrapped_y = WrapTextureCoordinate(y, static_cast<int>(resolution.y), sampler.address_mode_v);
    glm::vec4 value = glm::max(pixels[static_cast<size_t>(wrapped_y) * resolution.x + wrapped_x], glm::vec4(0.0f));
    if (texture.SamplesLinearSrgb()) {
      value = {DecodeSrgbChannel(value.x), DecodeSrgbChannel(value.y), DecodeSrgbChannel(value.z), value.w};
    }
    return value;
  };
  const glm::vec4 row_0 = glm::mix(fetch(base.x, base.y), fetch(base.x + 1, base.y), blend.x);
  const glm::vec4 row_1 = glm::mix(fetch(base.x, base.y + 1), fetch(base.x + 1, base.y + 1), blend.x);
  return glm::mix(row_0, row_1, blend.y);
}

double EstimateTriangleEmissiveImportance(Material& material, const GltfShadeMaterial& shade_material, const Vertex& v0,
                                          const Vertex& v1, const Vertex& v2) {
  const glm::vec3 factor = glm::max(shade_material.emissive_factor, glm::vec3(0.0f));
  const double factor_luminance = glm::dot(factor, glm::vec3(0.2126f, 0.7152f, 0.0722f));
  if (shade_material.emissive_texture == 0u ||
      shade_material.emissive_texture >= material.material_data.texture_infos.size()) {
    return factor_luminance;
  }
  const auto texture = material.GetTexture(shade_material.emissive_texture);
  if (!texture || texture->PeekLocalData().empty()) {
    return factor_luminance;
  }
  const auto& texture_info = material.material_data.texture_infos[shade_material.emissive_texture];
  const glm::vec2 uv0 = SelectTexCoord(v0, texture_info.tex_coord);
  const glm::vec2 uv1 = SelectTexCoord(v1, texture_info.tex_coord);
  const glm::vec2 uv2 = SelectTexCoord(v2, texture_info.tex_coord);
  double luminance_sum = 0.0;
  for (const auto barycentric : kTriangleImportanceSamples) {
    const glm::vec2 uv =
        texture_info.uv_transform * glm::vec3(uv0 * barycentric.x + uv1 * barycentric.y + uv2 * barycentric.z, 1.0f);
    const glm::vec3 radiance = factor * glm::vec3(SampleLocalTexture(*texture, uv));
    luminance_sum += glm::dot(radiance, glm::vec3(0.2126f, 0.7152f, 0.0722f));
  }
  return glm::max(luminance_sum / static_cast<double>(kTriangleImportanceSamples.size()), factor_luminance * 1.0e-4);
}

double EstimateTriangleOpacityImportance(Material& material, const GltfShadeMaterial& shade_material, const Vertex& v0,
                                         const Vertex& v1, const Vertex& v2) {
  if (shade_material.alpha_mode == static_cast<int32_t>(GltfAlphaMode::Opaque)) {
    return 1.0;
  }
  const bool has_texture = shade_material.pbr_base_color_texture != 0u &&
                           shade_material.pbr_base_color_texture < material.material_data.texture_infos.size();
  const auto texture = has_texture ? material.GetTexture(shade_material.pbr_base_color_texture) : nullptr;
  const auto* texture_info =
      has_texture ? &material.material_data.texture_infos[shade_material.pbr_base_color_texture] : nullptr;
  const glm::vec2 uv0 = texture_info ? SelectTexCoord(v0, texture_info->tex_coord) : glm::vec2(0.0f);
  const glm::vec2 uv1 = texture_info ? SelectTexCoord(v1, texture_info->tex_coord) : glm::vec2(0.0f);
  const glm::vec2 uv2 = texture_info ? SelectTexCoord(v2, texture_info->tex_coord) : glm::vec2(0.0f);
  double opacity_sum = 0.0;
  for (const auto barycentric : kTriangleImportanceSamples) {
    float opacity =
        shade_material.pbr_base_color_factor.a * glm::dot(glm::vec3(v0.color.a, v1.color.a, v2.color.a), barycentric);
    if (texture && !texture->PeekLocalData().empty()) {
      const glm::vec2 uv =
          texture_info->uv_transform * glm::vec3(uv0 * barycentric.x + uv1 * barycentric.y + uv2 * barycentric.z, 1.0f);
      opacity *= SampleLocalTexture(*texture, uv).a;
    }
    opacity = glm::clamp(opacity, 0.0f, 1.0f);
    opacity_sum += shade_material.alpha_mode == static_cast<int32_t>(GltfAlphaMode::Mask)
                       ? static_cast<double>(opacity >= shade_material.alpha_cutoff)
                       : opacity;
  }
  return glm::max(opacity_sum / kTriangleImportanceSamples.size(), 1.0e-4);
}

uint64_t GetTextureSamplingSignature(Material& material, const uint16_t texture_slot) {
  if (texture_slot == 0u || texture_slot >= material.material_data.texture_infos.size()) {
    return 0u;
  }
  const auto texture = material.GetTexture(texture_slot);
  if (!texture) {
    return 0u;
  }
  uint64_t signature = texture->GetHandle().GetValue();
  uint64_t content_signature = signature;
  (void)TextureStorage::TryGetTexture2DContentSignature(texture->GetTextureStorageIndex(), content_signature);
  signature = MixDdgiInventorySignature(signature, content_signature);
  const auto& texture_info = material.material_data.texture_infos[texture_slot];
  signature = MixDdgiInventorySignature(signature, static_cast<uint32_t>(texture_info.tex_coord));
  for (int column = 0; column < 3; ++column) {
    for (int row = 0; row < 2; ++row) {
      const float component = texture_info.uv_transform[column][row];
      uint32_t bits = 0;
      std::memcpy(&bits, &component, sizeof(bits));
      signature = MixDdgiInventorySignature(signature, bits);
    }
  }
  const auto& sampler = texture->GetSamplerSettings();
  signature = MixDdgiInventorySignature(signature, sampler.address_mode_u);
  signature = MixDdgiInventorySignature(signature, sampler.address_mode_v);
  signature = MixDdgiInventorySignature(signature, texture->SamplesLinearSrgb());
  return signature;
}

uint64_t GetEmissiveSamplingSignature(Material& material, const GltfShadeMaterial& shade_material) {
  uint64_t signature = GetTextureSamplingSignature(material, shade_material.emissive_texture);
  signature = MixDdgiInventorySignature(signature, glm::floatBitsToUint(shade_material.emissive_factor.x));
  signature = MixDdgiInventorySignature(signature, glm::floatBitsToUint(shade_material.emissive_factor.y));
  signature = MixDdgiInventorySignature(signature, glm::floatBitsToUint(shade_material.emissive_factor.z));
  signature = MixDdgiInventorySignature(signature, static_cast<uint32_t>(shade_material.alpha_mode));
  if (shade_material.alpha_mode != static_cast<int32_t>(GltfAlphaMode::Opaque)) {
    signature = MixDdgiInventorySignature(signature,
                                          GetTextureSamplingSignature(material, shade_material.pbr_base_color_texture));
    signature = MixDdgiInventorySignature(signature, glm::floatBitsToUint(shade_material.pbr_base_color_factor.a));
    signature = MixDdgiInventorySignature(signature, glm::floatBitsToUint(shade_material.alpha_cutoff));
  }
  return signature;
}

template <typename Signatures>
uint64_t HashDdgiEmissiveInventorySignature(const Signatures& signatures) {
  std::vector<uint64_t> entry_hashes;
  entry_hashes.reserve(signatures.size());
  for (const auto& entry : signatures) {
    auto hash = MixDdgiInventorySignature(entry.mesh_handle, entry.renderer_handle);
    hash = MixDdgiInventorySignature(hash, entry.material_handle);
    hash = MixDdgiInventorySignature(hash, entry.emissive_sampling_signature);
    hash = MixDdgiInventorySignature(hash, entry.geometry_version);
    hash = MixDdgiInventorySignature(hash, entry.triangle_count);
    for (int column = 0; column < 4; ++column) {
      for (int row = 0; row < 4; ++row) {
        hash = MixDdgiInventorySignature(hash, glm::floatBitsToUint(entry.model.value[column][row]));
      }
    }
    uint64_t importance_bits = 0;
    std::memcpy(&importance_bits, &entry.importance, sizeof(importance_bits));
    entry_hashes.push_back(MixDdgiInventorySignature(hash, importance_bits));
  }
  std::sort(entry_hashes.begin(), entry_hashes.end());
  auto result = static_cast<uint64_t>(entry_hashes.size());
  for (const auto hash : entry_hashes) {
    result = MixDdgiInventorySignature(result, hash);
  }
  return result;
}

bool UsesTransparentRasterPass(const Material& material, const GltfShadeMaterial& shade_material) {
  return material.draw_settings.blending || GltfMaterialRequiresTransparentPass(shade_material);
}

std::shared_ptr<EnvironmentalMap> ResolveIndirectEnvironmentMap(
    const ResolvedEnvironmentalLighting::IndirectEnvironmentSource& source) {
  if (source.kind == ResolvedEnvironmentalLighting::IndirectEnvironmentSourceKind::EngineDefault) {
    return Resources::GetInstance().GetDefaultEnvironmentalMap();
  }
  if (source.kind != ResolvedEnvironmentalLighting::IndirectEnvironmentSourceKind::EnvironmentalMap) {
    return {};
  }
  auto map_ref = source.environmental_map;
  return map_ref.Get<EnvironmentalMap>();
}

VkCullModeFlags SwapCullModeFaces(const VkCullModeFlags cull_mode) {
  if (cull_mode == VK_CULL_MODE_BACK_BIT) {
    return VK_CULL_MODE_FRONT_BIT;
  }
  if (cull_mode == VK_CULL_MODE_FRONT_BIT) {
    return VK_CULL_MODE_BACK_BIT;
  }
  return cull_mode;
}

VkCullModeFlags ResolveCullModeForTransform(const VkCullModeFlags cull_mode, const glm::mat4& model) {
  if (glm::determinant(glm::mat3(model)) >= 0.0f) {
    return cull_mode;
  }
  return SwapCullModeFaces(cull_mode);
}

VkCullModeFlags ResolveInstancedCullModeForTransforms(const VkCullModeFlags cull_mode, const glm::mat4& model,
                                                      const std::vector<ParticleInfo>& particle_infos) {
  if (cull_mode == VK_CULL_MODE_NONE || cull_mode == VK_CULL_MODE_FRONT_AND_BACK || particle_infos.empty()) {
    return cull_mode;
  }
  bool has_positive_determinant = false;
  bool has_negative_determinant = false;
  for (const auto& particle_info : particle_infos) {
    if (glm::determinant(glm::mat3(model * particle_info.instance_matrix.value)) < 0.0f) {
      has_negative_determinant = true;
    } else {
      has_positive_determinant = true;
    }
    if (has_positive_determinant && has_negative_determinant) {
      return VK_CULL_MODE_NONE;
    }
  }
  return has_negative_determinant ? SwapCullModeFaces(cull_mode) : cull_mode;
}

VkDrawMeshTasksIndirectCommandEXT CreateMeshTaskCommand(const uint32_t meshlet_range) {
  VkDrawMeshTasksIndirectCommandEXT command{};
  const uint32_t task_work_group_invocations =
      Platform::GetSelectedPhysicalDevice()->mesh_shader_properties_ext.maxPreferredTaskWorkGroupInvocations;
  command.groupCountX = (meshlet_range + task_work_group_invocations - 1) / task_work_group_invocations;
  command.groupCountY = 1;
  command.groupCountZ = 1;
  return command;
}

VkDrawIndexedIndirectCommand CreateIndexedCommand(const uint32_t triangle_offset, const uint32_t triangle_index_count) {
  VkDrawIndexedIndirectCommand command{};
  command.instanceCount = 1;
  command.firstIndex = triangle_offset * 3;
  command.indexCount = triangle_index_count * 3;
  command.vertexOffset = 0;
  command.firstInstance = 0;
  return command;
}

void AppendMeshIndirectCommands(std::vector<VkDrawIndexedIndirectCommand>& indexed_commands,
                                std::vector<VkDrawMeshTasksIndirectCommandEXT>& mesh_task_commands,
                                const uint32_t triangle_offset, const uint32_t triangle_index_count,
                                const uint32_t meshlet_range) {
  mesh_task_commands.emplace_back(CreateMeshTaskCommand(meshlet_range));
  indexed_commands.emplace_back(CreateIndexedCommand(triangle_offset, triangle_index_count));
}

size_t CountRenderInstances(const std::shared_ptr<RenderInstanceStorage::IRenderInstanceCollection>& render_instances) {
  size_t count = 0;
  render_instances->ForEachRenderInstance([&](const auto&) {
    ++count;
  });
  return count;
}

void ValidateDeferredMeshIndirectCommandCount(
    const std::shared_ptr<RenderInstanceStorage::IRenderInstanceCollection>& deferred_render_instances,
    const std::vector<VkDrawIndexedIndirectCommand>& indexed_commands,
    const std::vector<VkDrawMeshTasksIndirectCommandEXT>& mesh_task_commands) {
  if (!ApplicationContext::Get().GetLayer<RenderLayer>()) {
    return;
  }
  const auto deferred_count = CountRenderInstances(deferred_render_instances);
  const auto indexed_count = indexed_commands.size();
  const auto mesh_task_count = mesh_task_commands.size();
  assert(indexed_count == deferred_count);
  assert(mesh_task_count == deferred_count);
  if (indexed_count != deferred_count || mesh_task_count != deferred_count) {
    EVOENGINE_ERROR("Deferred mesh indirect command count mismatch: deferred_instances=" +
                    std::to_string(deferred_count) + ", indexed_commands=" + std::to_string(indexed_count) +
                    ", mesh_task_commands=" + std::to_string(mesh_task_count))
  }
}

bool GaussianSplatGpuRadixSortSupported() {
  return Platform::Initialized() && Platform::GetInstance().GetCapabilities().subgroup_size >= 32u;
}

}  // namespace

float RenderSettings::GetShadowCascadeSplit(const int split, const float near_distance) const {
  const auto clamped_split = glm::clamp(split, 0, 3);
  if (clamped_split == 3) {
    return 1.0f;
  }
  const auto far_distance = glm::max(max_shadow_distance, 0.001f);
  const auto near_clip_distance = glm::clamp(near_distance, 0.001f, far_distance);
  const auto split_ratio = static_cast<float>(clamped_split + 1) / 4.0f;
  const auto uniform_split = split_ratio;
  const auto logarithmic_split =
      near_clip_distance * std::pow(far_distance / near_clip_distance, split_ratio) / far_distance;
  return glm::clamp(glm::mix(uniform_split, logarithmic_split, glm::clamp(shadow_cascade_split_lambda, 0.0f, 1.0f)),
                    0.0f, 1.0f);
}

float RenderSettings::GetShadowCascadeSplitDistance(const int split, const float near_distance) const {
  return max_shadow_distance * GetShadowCascadeSplit(split, near_distance);
}

glm::vec4 RenderSettings::GetShadowCascadeSplitDistances(const float near_distance) const {
  glm::vec4 result;
  for (int split = 0; split < 4; ++split) {
    result[split] = GetShadowCascadeSplitDistance(split, near_distance);
  }
  return result;
}

float RenderSettings::GetShadowCascadeTransitionHalfWidth(const int boundary, const float near_distance) const {
  const auto clamped_boundary = glm::clamp(boundary, 0, 2);
  const auto split_distances = GetShadowCascadeSplitDistances(near_distance);
  const auto split_distance = split_distances[clamped_boundary];
  const auto previous_split = clamped_boundary == 0 ? 0.0f : split_distances[clamped_boundary - 1];
  const auto next_split = split_distances[clamped_boundary + 1];
  const auto available_width = glm::max(glm::min(split_distance - previous_split, next_split - split_distance), 0.0f);
  return glm::min(glm::max(shadow_cascade_transition_width, 0.0f), available_width) * 0.5f;
}

const char* RenderSettings::GetShadowCascadeFitModeName(const ShadowCascadeFitMode mode) {
  switch (mode) {
    case ShadowCascadeFitMode::StableSphere:
      return "Stable Sphere";
    case ShadowCascadeFitMode::TightLightSpaceAabb:
      return "Tight Light-Space AABB";
  }
  return "Stable Sphere";
}

RenderInstanceStorage::DirectionalShadowCascadeFitResult RenderInstanceStorage::CalculateDirectionalShadowCascadeFit(
    const DirectionalShadowCascadeFitInput& input) {
  DirectionalShadowCascadeFitResult result;
  glm::vec3 frustum_center(0.0f);
  for (const auto& corner : input.frustum_corners) {
    frustum_center += corner;
  }
  frustum_center /= static_cast<float>(input.frustum_corners.size());

  const auto light_direction = glm::normalize(input.light_direction);
  const auto light_up = glm::normalize(input.light_up);
  const auto world_center = input.world_bound.Center();
  const auto light_view_center =
      frustum_center + glm::dot(world_center - frustum_center, light_direction) * light_direction;

  const std::array<glm::vec3, 8> world_corners = {
      glm::vec3(input.world_bound.min.x, input.world_bound.min.y, input.world_bound.min.z),
      glm::vec3(input.world_bound.min.x, input.world_bound.min.y, input.world_bound.max.z),
      glm::vec3(input.world_bound.min.x, input.world_bound.max.y, input.world_bound.min.z),
      glm::vec3(input.world_bound.min.x, input.world_bound.max.y, input.world_bound.max.z),
      glm::vec3(input.world_bound.max.x, input.world_bound.min.y, input.world_bound.min.z),
      glm::vec3(input.world_bound.max.x, input.world_bound.min.y, input.world_bound.max.z),
      glm::vec3(input.world_bound.max.x, input.world_bound.max.y, input.world_bound.min.z),
      glm::vec3(input.world_bound.max.x, input.world_bound.max.y, input.world_bound.max.z),
  };
  float minimum_depth = std::numeric_limits<float>::max();
  float maximum_depth = std::numeric_limits<float>::lowest();
  for (const auto& corner : world_corners) {
    const auto depth = glm::dot(corner - light_view_center, light_direction);
    minimum_depth = glm::min(minimum_depth, depth);
    maximum_depth = glm::max(maximum_depth, depth);
  }
  result.light_space_depth_half_extent = glm::max(maximum_depth - minimum_depth, 0.001f);

  const auto light_position = light_view_center - light_direction * result.light_space_depth_half_extent;
  const auto light_view = glm::lookAt(light_position, light_view_center, light_up);
  const bool stabilize = input.mode != RenderSettings::ShadowCascadeFitMode::TightLightSpaceAabb;
  const auto filter_radius_world = glm::max(input.filter_radius_world, 0.0f);
  const auto padded_half_extent = [&](const float raw_half_extent, const int viewport_extent) {
    const auto half_extent = glm::max(raw_half_extent, 0.001f);
    if (viewport_extent <= 0) {
      return half_extent;
    }
    const auto footprint_texels = stabilize ? 2.0f : 1.0f;
    if (static_cast<float>(viewport_extent) <= footprint_texels) {
      return half_extent + filter_radius_world;
    }
    return (half_extent + filter_radius_world) / (1.0f - footprint_texels / static_cast<float>(viewport_extent));
  };
  if (input.mode == RenderSettings::ShadowCascadeFitMode::TightLightSpaceAabb) {
    result.orthographic_min = glm::vec2(std::numeric_limits<float>::max());
    result.orthographic_max = glm::vec2(std::numeric_limits<float>::lowest());
    for (const auto& corner : input.frustum_corners) {
      const auto light_space_corner = light_view * glm::vec4(corner, 1.0f);
      result.orthographic_min = glm::min(result.orthographic_min, glm::vec2(light_space_corner));
      result.orthographic_max = glm::max(result.orthographic_max, glm::vec2(light_space_corner));
    }
    const auto extent_center = (result.orthographic_min + result.orthographic_max) * 0.5f;
    auto half_extent = (result.orthographic_max - result.orthographic_min) * 0.5f;
    half_extent.x = padded_half_extent(half_extent.x, input.viewport_extent.x);
    half_extent.y = padded_half_extent(half_extent.y, input.viewport_extent.y);
    result.orthographic_min = extent_center - half_extent;
    result.orthographic_max = extent_center + half_extent;
  } else {
    auto half_extent = 0.0f;
    for (const auto& corner : input.frustum_corners) {
      half_extent = glm::max(half_extent, glm::distance(corner, frustum_center));
    }
    half_extent = glm::max(padded_half_extent(half_extent, input.viewport_extent.x),
                           padded_half_extent(half_extent, input.viewport_extent.y));
    half_extent = glm::ceil(half_extent * 16.0f) / 16.0f;
    result.orthographic_min = glm::vec2(-half_extent);
    result.orthographic_max = glm::vec2(half_extent);
  }

  const auto extent_center = (result.orthographic_min + result.orthographic_max) * 0.5f;
  const auto half_extent = glm::max((result.orthographic_max - result.orthographic_min) * 0.5f, glm::vec2(0.001f));
  result.orthographic_min = extent_center - half_extent;
  result.orthographic_max = extent_center + half_extent;

  auto light_projection = glm::ortho(result.orthographic_min.x, result.orthographic_max.x, result.orthographic_min.y,
                                     result.orthographic_max.y, 0.0f, result.light_space_depth_half_extent * 2.0f);
  if (stabilize && input.viewport_extent.x > 0 && input.viewport_extent.y > 0) {
    const auto shadow_origin = light_projection * light_view * glm::vec4(0.0f, 0.0f, 0.0f, 1.0f);
    const auto viewport = glm::vec2(input.viewport_extent);
    const auto texel_origin = glm::vec2(shadow_origin) * viewport * 0.5f;
    const auto projection_offset = (glm::round(texel_origin) - texel_origin) * 2.0f / viewport;
    light_projection[3][0] += projection_offset.x;
    light_projection[3][1] += projection_offset.y;
  }
  result.light_space_matrix = light_projection * light_view;
  return result;
}

bool RenderInstanceStorage::ExternalRenderInstance::operator!=(const ExternalRenderInstance& other) const {
  if (entity_selected != other.entity_selected)
    return true;
  if (instance_index != other.instance_index)
    return true;
  if (command_type != other.command_type)
    return true;
  if (model.value != other.model.value)
    return true;
  if (owner != other.owner)
    return true;
  if (material != other.material)
    return true;
  if (geometry_version != other.geometry_version)
    return true;
  if (material_version != other.material_version)
    return true;
  if (cast_shadow != other.cast_shadow)
    return true;
  if (line_width != other.line_width)
    return true;
  if (cull_mode != other.cull_mode)
    return true;
  if (polygon_mode != other.polygon_mode)
    return true;
  if (ddgi_geometry.bottom_level_acceleration_structure != other.ddgi_geometry.bottom_level_acceleration_structure)
    return true;
  if (ddgi_geometry.triangle_offset != other.ddgi_geometry.triangle_offset)
    return true;
  if (ddgi_geometry.triangle_count != other.ddgi_geometry.triangle_count)
    return true;
  return false;
}

bool RenderInstanceStorage::ExternalRenderInstance::HasDdgiRayTracingGeometry() const {
  return ddgi_geometry.IsValid();
}

void RenderInstanceStorage::ExternalRenderInstance::Apply(InstanceInfoBlock& instance_info_block) const {
  instance_info_block.model = model;
  instance_info_block.material_index = material_index;
  instance_info_block.triangle_offset = HasDdgiRayTracingGeometry() ? ddgi_geometry.triangle_offset : 0;
  instance_info_block.meshlet_index_offset = 0;
  instance_info_block.meshlet_size = 0;
  instance_info_block.info_index = entity_selected ? 1 : 0;
  instance_info_block.entity_index = owner.GetIndex();
  instance_info_block.renderer_handle = renderer_handle;
}

uint32_t RenderInstanceStorage::ExternalRenderInstance::Render(
    VkCommandBuffer vk_command_buffer, const RenderInstancePushConstant& render_instance_push_constant,
    const std::shared_ptr<GraphicsPipeline>& graphics_pipeline) const {
  return 0;
}

bool RenderInstanceStorage::MeshRenderInstance::operator!=(const MeshRenderInstance& other) const {
  if (entity_selected != other.entity_selected)
    return true;
  if (instance_index != other.instance_index)
    return true;
  if (command_type != other.command_type)
    return true;
  if (model.value != other.model.value)
    return true;
  if (owner != other.owner)
    return true;
  if (mesh != other.mesh)
    return true;
  if (material != other.material)
    return true;
  if (ray_tracing_triangle_range != other.ray_tracing_triangle_range)
    return true;
  if (ray_tracing_blas != other.ray_tracing_blas)
    return true;
  if (geometry_version != other.geometry_version)
    return true;
  if (ray_tracing_geometry_version != other.ray_tracing_geometry_version)
    return true;
  if (morph_weights_version != other.morph_weights_version)
    return true;
  if (material_version != other.material_version)
    return true;
  if (cast_shadow != other.cast_shadow)
    return true;
  if (line_width != other.line_width)
    return true;
  if (cull_mode != other.cull_mode)
    return true;
  if (polygon_mode != other.polygon_mode)
    return true;
  return false;
}

void RenderInstanceStorage::MeshRenderInstance::Apply(InstanceInfoBlock& instance_info_block) const {
  instance_info_block.model = model;
  instance_info_block.material_index = material_index;
  instance_info_block.info_index = entity_selected ? 1 : 0;
  instance_info_block.triangle_offset =
      ray_tracing_triangle_range && ray_tracing_triangle_range->prev_frame_index_count != 0
          ? ray_tracing_triangle_range->prev_frame_offset
          : mesh->triangle_range_->prev_frame_offset;
  instance_info_block.meshlet_index_offset = mesh->meshlet_range_->prev_frame_offset;
  instance_info_block.meshlet_size = mesh->meshlet_range_->prev_frame_range;
  instance_info_block.entity_index = owner.GetIndex();
  instance_info_block.renderer_handle = renderer_handle;
}

uint32_t RenderInstanceStorage::MeshRenderInstance::Render(
    const VkCommandBuffer vk_command_buffer, const RenderInstancePushConstant& render_instance_push_constant,
    const std::shared_ptr<GraphicsPipeline>& graphics_pipeline) const {
  graphics_pipeline->PushConstant(vk_command_buffer, 0, render_instance_push_constant);

  if (graphics_pipeline->mesh_shader) {
    const uint32_t task_work_group_invocations =
        Platform::GetSelectedPhysicalDevice()->mesh_shader_properties_ext.maxPreferredTaskWorkGroupInvocations;
    graphics_pipeline->states.ApplyAllStates(vk_command_buffer);
    const uint32_t count =
        (mesh->meshlet_range_->prev_frame_range + task_work_group_invocations - 1) / task_work_group_invocations;
    graphics_pipeline->DrawMeshTasks(vk_command_buffer, count);
  } else {
    mesh->DrawIndexed(vk_command_buffer, graphics_pipeline->states, 1);
  }
  return mesh->triangle_range_->prev_frame_index_count;
}

bool RenderInstanceStorage::SkinnedMeshRenderInstance::operator!=(const SkinnedMeshRenderInstance& other) const {
  if (entity_selected != other.entity_selected)
    return true;
  if (instance_index != other.instance_index)
    return true;
  if (command_type != other.command_type)
    return true;
  if (model.value != other.model.value)
    return true;
  if (owner != other.owner)
    return true;
  if (material != other.material)
    return true;
  if (skinned_mesh != other.skinned_mesh)
    return true;
  if (bone_matrices != other.bone_matrices)
    return true;
  if (ray_tracing_triangle_range != other.ray_tracing_triangle_range)
    return true;
  if (ray_tracing_blas != other.ray_tracing_blas)
    return true;
  if (geometry_version != other.geometry_version)
    return true;
  if (ray_tracing_geometry_version != other.ray_tracing_geometry_version)
    return true;
  if (morph_weights_version != other.morph_weights_version)
    return true;
  if (material_version != other.material_version)
    return true;
  if (bone_matrices_version != other.bone_matrices_version)
    return true;
  if (cast_shadow != other.cast_shadow)
    return true;
  if (line_width != other.line_width)
    return true;
  if (cull_mode != other.cull_mode)
    return true;
  if (polygon_mode != other.polygon_mode)
    return true;
  return false;
}

void RenderInstanceStorage::SkinnedMeshRenderInstance::Apply(InstanceInfoBlock& instance_info_block) const {
  instance_info_block.model = model;
  instance_info_block.material_index = material_index;
  instance_info_block.info_index = entity_selected ? 1 : 0;
  if (ray_tracing_triangle_range && ray_tracing_triangle_range->prev_frame_index_count != 0) {
    instance_info_block.triangle_offset = ray_tracing_triangle_range->prev_frame_offset;
  } else if (skinned_mesh->ray_tracing_triangle_range_ &&
             skinned_mesh->ray_tracing_triangle_range_->prev_frame_index_count != 0) {
    instance_info_block.triangle_offset = skinned_mesh->ray_tracing_triangle_range_->prev_frame_offset;
  } else {
    instance_info_block.triangle_offset = skinned_mesh->skinned_triangle_range_->prev_frame_offset;
  }
  instance_info_block.meshlet_index_offset = skinned_mesh->skinned_meshlet_range_->prev_frame_offset;
  instance_info_block.meshlet_size = skinned_mesh->skinned_meshlet_range_->prev_frame_range;
  instance_info_block.entity_index = owner.GetIndex();
  instance_info_block.renderer_handle = renderer_handle;
}

uint32_t RenderInstanceStorage::SkinnedMeshRenderInstance::Render(
    VkCommandBuffer vk_command_buffer, const RenderInstancePushConstant& render_instance_push_constant,
    const std::shared_ptr<GraphicsPipeline>& graphics_pipeline) const {
  graphics_pipeline->BindDescriptorSet(vk_command_buffer, 1, bone_matrices->GetDescriptorSet()->GetVkDescriptorSet());
  graphics_pipeline->PushConstant(vk_command_buffer, 0, render_instance_push_constant);
  skinned_mesh->DrawIndexed(vk_command_buffer, graphics_pipeline->states, 1);
  return skinned_mesh->skinned_triangle_range_->prev_frame_index_count;
}

bool RenderInstanceStorage::InstancedRenderInstance::operator!=(const InstancedRenderInstance& other) const {
  if (entity_selected != other.entity_selected)
    return true;
  if (instance_index != other.instance_index)
    return true;
  if (command_type != other.command_type)
    return true;
  if (model.value != other.model.value)
    return true;
  if (owner != other.owner)
    return true;
  if (material != other.material)
    return true;
  if (mesh != other.mesh)
    return true;
  if (particle_infos != other.particle_infos)
    return true;
  if (geometry_version != other.geometry_version)
    return true;
  if (material_version != other.material_version)
    return true;
  if (particle_info_list_version != other.particle_info_list_version)
    return true;
  if (cast_shadow != other.cast_shadow)
    return true;
  if (line_width != other.line_width)
    return true;
  if (cull_mode != other.cull_mode)
    return true;
  if (polygon_mode != other.polygon_mode)
    return true;
  return false;
}

void RenderInstanceStorage::InstancedRenderInstance::Apply(InstanceInfoBlock& instance_info_block) const {
  instance_info_block.model = model;
  instance_info_block.material_index = material_index;
  instance_info_block.info_index = entity_selected ? 1 : 0;
  instance_info_block.triangle_offset = mesh->triangle_range_->prev_frame_offset;
  instance_info_block.meshlet_index_offset = mesh->meshlet_range_->prev_frame_offset;
  instance_info_block.meshlet_size = mesh->meshlet_range_->prev_frame_range;
  instance_info_block.entity_index = owner.GetIndex();
  instance_info_block.renderer_handle = renderer_handle;
}

uint32_t RenderInstanceStorage::InstancedRenderInstance::Render(
    const VkCommandBuffer vk_command_buffer, const RenderInstancePushConstant& render_instance_push_constant,
    const std::shared_ptr<GraphicsPipeline>& graphics_pipeline) const {
  graphics_pipeline->BindDescriptorSet(vk_command_buffer, 1, particle_infos->GetDescriptorSet()->GetVkDescriptorSet());
  graphics_pipeline->PushConstant(vk_command_buffer, 0, render_instance_push_constant);
  mesh->DrawIndexed(vk_command_buffer, graphics_pipeline->states, particle_infos->PeekParticleInfoList().size());
  return mesh->triangle_range_->prev_frame_index_count * particle_infos->PeekParticleInfoList().size();
}

bool RenderInstanceStorage::StrandsRenderInstance::operator!=(const StrandsRenderInstance& other) const {
  if (entity_selected != other.entity_selected)
    return true;
  if (instance_index != other.instance_index)
    return true;
  if (command_type != other.command_type)
    return true;
  if (model.value != other.model.value)
    return true;
  if (owner != other.owner)
    return true;
  if (material != other.material)
    return true;
  if (strands != other.strands)
    return true;

  if (geometry_version != other.geometry_version)
    return true;
  if (material_version != other.material_version)
    return true;

  if (cast_shadow != other.cast_shadow)
    return true;
  if (line_width != other.line_width)
    return true;
  if (cull_mode != other.cull_mode)
    return true;
  if (polygon_mode != other.polygon_mode)
    return true;
  return false;
}

void RenderInstanceStorage::StrandsRenderInstance::Apply(InstanceInfoBlock& instance_info_block) const {
  instance_info_block.model = model;
  instance_info_block.material_index = material_index;
  instance_info_block.info_index = entity_selected ? 1 : 0;
  instance_info_block.triangle_offset = strands->segment_range_->prev_frame_offset;
  instance_info_block.meshlet_index_offset = strands->strand_meshlet_range_->prev_frame_offset;
  instance_info_block.meshlet_size = strands->strand_meshlet_range_->prev_frame_range;
  instance_info_block.entity_index = owner.GetIndex();
  instance_info_block.renderer_handle = renderer_handle;
}

uint32_t RenderInstanceStorage::StrandsRenderInstance::Render(
    const VkCommandBuffer vk_command_buffer, const RenderInstancePushConstant& render_instance_push_constant,
    const std::shared_ptr<GraphicsPipeline>& graphics_pipeline) const {
  if (!graphics_pipeline->mesh_shader) {
    return 0;
  }
  graphics_pipeline->PushConstant(vk_command_buffer, 0, render_instance_push_constant);
  graphics_pipeline->states.ApplyAllStates(vk_command_buffer);
  graphics_pipeline->DrawMeshTasks(vk_command_buffer, strands->strand_meshlet_range_->prev_frame_range);
  return strands->segment_range_->prev_frame_index_count;
}

bool RenderInstanceStorage::GaussianSplatRenderInstance::operator!=(const GaussianSplatRenderInstance& other) const {
  if (entity_selected != other.entity_selected)
    return true;
  if (instance_index != other.instance_index)
    return true;
  if (command_type != other.command_type)
    return true;
  if (model.value != other.model.value)
    return true;
  if (owner != other.owner)
    return true;
  if (gaussian_splat != other.gaussian_splat)
    return true;
  if (geometry_version != other.geometry_version)
    return true;
  if (opacity_scale != other.opacity_scale)
    return true;
  if (sh_degree != other.sh_degree)
    return true;
  if (sort_mode != other.sort_mode)
    return true;
  if (depth_mode != other.depth_mode)
    return true;
  if (raster_mode != other.raster_mode)
    return true;
  return false;
}

void RenderInstanceStorage::GaussianSplatRenderInstance::Apply(InstanceInfoBlock& instance_info_block) const {
  instance_info_block.model = model;
  instance_info_block.material_index = -1;
  instance_info_block.triangle_offset = 0;
  instance_info_block.meshlet_index_offset = 0;
  instance_info_block.meshlet_size = 0;
  instance_info_block.info_index = entity_selected ? 1 : 0;
  instance_info_block.entity_index = owner.GetIndex();
  instance_info_block.renderer_handle = renderer_handle;
}

uint32_t RenderInstanceStorage::GaussianSplatRenderInstance::Render(VkCommandBuffer, const RenderInstancePushConstant&,
                                                                    const std::shared_ptr<GraphicsPipeline>&) const {
  return 0;
}

bool RenderInstanceStorage::ExternalRenderInstanceCollection::operator!=(
    const ExternalRenderInstanceCollection& other) const {
  if (render_commands.size() != other.render_commands.size())
    return true;
  for (uint32_t i = 0; i < render_commands.size(); i++) {
    if (render_commands[i] != other.render_commands[i])
      return true;
  }
  return false;
}

bool RenderInstanceStorage::ExternalRenderInstanceCollection::Empty() const {
  return render_commands.empty();
}

void RenderInstanceStorage::ExternalRenderInstanceCollection::Register(
    const std::shared_ptr<IRenderInstance>& render_instance) {
  render_commands.emplace_back(std::dynamic_pointer_cast<ExternalRenderInstance>(render_instance));
}

void RenderInstanceStorage::ExternalRenderInstanceCollection::ForEachRenderInstance(
    const std::function<void(const std::shared_ptr<IRenderInstance>&)>& action) {
  ForEachExternalRenderInstance([&](const auto& render_instance) {
    action(render_instance);
  });
}

void RenderInstanceStorage::ExternalRenderInstanceCollection::ForEachExternalRenderInstance(
    const std::function<void(const std::shared_ptr<ExternalRenderInstance>&)>& action) const {
  for (const auto& i : render_commands) {
    action(i);
  }
}

bool RenderInstanceStorage::ExternalRenderInstanceCollection::HasDdgiRayTracingGeometry() const {
  for (const auto& render_command : render_commands) {
    if (render_command && render_command->HasDdgiRayTracingGeometry()) {
      return true;
    }
  }
  return false;
}

bool RenderInstanceStorage::MeshRenderInstanceCollection::operator!=(const MeshRenderInstanceCollection& other) const {
  if (render_commands.size() != other.render_commands.size())
    return true;
  for (uint32_t i = 0; i < render_commands.size(); i++) {
    if (render_commands[i] != other.render_commands[i])
      return true;
  }
  return false;
}

bool RenderInstanceStorage::MeshRenderInstanceCollection::Empty() const {
  return render_commands.empty();
}

void RenderInstanceStorage::MeshRenderInstanceCollection::Register(
    const std::shared_ptr<IRenderInstance>& render_instance) {
  render_commands.emplace_back(std::dynamic_pointer_cast<MeshRenderInstance>(render_instance));
}

void RenderInstanceStorage::MeshRenderInstanceCollection::ForEachRenderInstance(
    const std::function<void(const std::shared_ptr<IRenderInstance>&)>& action) {
  ForEachMeshRenderInstance([&](const auto& render_instance) {
    action(render_instance);
  });
}

void RenderInstanceStorage::MeshRenderInstanceCollection::ForEachMeshRenderInstance(
    const std::function<void(const std::shared_ptr<MeshRenderInstance>&)>& action) const {
  for (const auto& i : render_commands) {
    action(i);
  }
}

bool RenderInstanceStorage::SkinnedMeshRenderInstanceCollection::Empty() const {
  return render_commands.empty();
}

void RenderInstanceStorage::SkinnedMeshRenderInstanceCollection::Register(
    const std::shared_ptr<IRenderInstance>& render_instance) {
  render_commands.emplace_back(std::dynamic_pointer_cast<SkinnedMeshRenderInstance>(render_instance));
}

bool RenderInstanceStorage::SkinnedMeshRenderInstanceCollection::operator!=(
    const SkinnedMeshRenderInstanceCollection& other) const {
  if (render_commands.size() != other.render_commands.size())
    return true;
  for (uint32_t i = 0; i < render_commands.size(); i++) {
    if (render_commands[i] != other.render_commands[i])
      return true;
  }
  return false;
}

void RenderInstanceStorage::SkinnedMeshRenderInstanceCollection::ForEachRenderInstance(
    const std::function<void(const std::shared_ptr<IRenderInstance>&)>& action) {
  ForEachSkinnedMeshRenderInstance([&](const auto& render_instance) {
    action(render_instance);
  });
}

void RenderInstanceStorage::SkinnedMeshRenderInstanceCollection::ForEachSkinnedMeshRenderInstance(
    const std::function<void(const std::shared_ptr<SkinnedMeshRenderInstance>&)>& action) const {
  for (const auto& i : render_commands) {
    action(i);
  }
}

bool RenderInstanceStorage::StrandsRenderInstanceCollection::Empty() const {
  return render_commands.empty();
}

void RenderInstanceStorage::StrandsRenderInstanceCollection::Register(
    const std::shared_ptr<IRenderInstance>& render_instance) {
  render_commands.emplace_back(std::dynamic_pointer_cast<StrandsRenderInstance>(render_instance));
}

bool RenderInstanceStorage::StrandsRenderInstanceCollection::operator!=(
    const StrandsRenderInstanceCollection& other) const {
  if (render_commands.size() != other.render_commands.size())
    return true;
  for (uint32_t i = 0; i < render_commands.size(); i++) {
    if (render_commands[i] != other.render_commands[i])
      return true;
  }
  return false;
}

void RenderInstanceStorage::StrandsRenderInstanceCollection::ForEachRenderInstance(
    const std::function<void(const std::shared_ptr<IRenderInstance>&)>& action) {
  ForEachStrandsRenderInstance([&](const auto& render_instance) {
    action(render_instance);
  });
}

void RenderInstanceStorage::StrandsRenderInstanceCollection::ForEachStrandsRenderInstance(
    const std::function<void(const std::shared_ptr<StrandsRenderInstance>&)>& action) const {
  for (const auto& i : render_commands) {
    action(i);
  }
}

bool RenderInstanceStorage::GaussianSplatRenderInstanceCollection::Empty() const {
  return render_commands.empty();
}

void RenderInstanceStorage::GaussianSplatRenderInstanceCollection::Register(
    const std::shared_ptr<IRenderInstance>& render_instance) {
  render_commands.emplace_back(std::dynamic_pointer_cast<GaussianSplatRenderInstance>(render_instance));
}

bool RenderInstanceStorage::GaussianSplatRenderInstanceCollection::operator!=(
    const GaussianSplatRenderInstanceCollection& other) const {
  if (render_commands.size() != other.render_commands.size())
    return true;
  for (uint32_t i = 0; i < render_commands.size(); i++) {
    if (render_commands[i] && other.render_commands[i]) {
      if (*render_commands[i] != *other.render_commands[i])
        return true;
    } else if (render_commands[i] != other.render_commands[i]) {
      return true;
    }
  }
  return false;
}

void RenderInstanceStorage::GaussianSplatRenderInstanceCollection::ForEachRenderInstance(
    const std::function<void(const std::shared_ptr<IRenderInstance>&)>& action) {
  ForEachGaussianSplatRenderInstance([&](const auto& render_instance) {
    action(render_instance);
  });
}

void RenderInstanceStorage::GaussianSplatRenderInstanceCollection::ForEachGaussianSplatRenderInstance(
    const std::function<void(const std::shared_ptr<GaussianSplatRenderInstance>&)>& action) const {
  for (const auto& i : render_commands) {
    action(i);
  }
}

bool RenderInstanceStorage::InstancedRenderInstanceCollection::Empty() const {
  return render_commands.empty();
}

void RenderInstanceStorage::InstancedRenderInstanceCollection::Register(
    const std::shared_ptr<IRenderInstance>& render_instance) {
  render_commands.emplace_back(std::dynamic_pointer_cast<InstancedRenderInstance>(render_instance));
}

bool RenderInstanceStorage::InstancedRenderInstanceCollection::operator!=(
    const InstancedRenderInstanceCollection& other) const {
  if (render_commands.size() != other.render_commands.size())
    return true;
  for (uint32_t i = 0; i < render_commands.size(); i++) {
    if (render_commands[i] != other.render_commands[i])
      return true;
  }
  return false;
}

void RenderInstanceStorage::InstancedRenderInstanceCollection::ForEachRenderInstance(
    const std::function<void(const std::shared_ptr<IRenderInstance>&)>& action) {
  ForEachInstancedRenderInstance([&](const auto& render_instance) {
    action(render_instance);
  });
}

void RenderInstanceStorage::InstancedRenderInstanceCollection::ForEachInstancedRenderInstance(
    const std::function<void(const std::shared_ptr<InstancedRenderInstance>&)>& action) const {
  for (const auto& i : render_commands) {
    action(i);
  }
}

void RenderInstanceStorage::RenderInfoBlock::Apply(const RenderSettings& target_render_settings) {
  for (int split = 0; split < 4; split++) {
    split_distances[split] = target_render_settings.GetShadowCascadeSplitDistance(split);
  }
  if (const auto render_layer = ApplicationContext::Get().GetLayer<RenderLayer>()) {
    brdflut_texture_index = render_layer->environmental_brdf_lut_->GetTextureStorageIndex();
  }
  if (target_render_settings.enable_debug_visualization)
    debug_visualization = 1;
  else
    debug_visualization = 0;

  pcf_sample_amount = target_render_settings.pcf_sample_amount;
  shadow_cascade_transition_width = glm::max(target_render_settings.shadow_cascade_transition_width, 0.0f);
  shadow_debug_parameters = glm::ivec4(glm::clamp(target_render_settings.shadow_debug_mode, 0, 5),
                                       glm::clamp(target_render_settings.shadow_debug_selected_cascade, 0, 3),
                                       glm::max(target_render_settings.shadow_debug_selected_light, 0),
                                       glm::clamp(target_render_settings.directional_pcf_sample_amount, 1, 64));
  shadow_fade_parameters = glm::vec4(
      glm::clamp(target_render_settings.shadow_distance_fade, 0.0f,
                 glm::max(target_render_settings.max_shadow_distance, 0.0f)),
      static_cast<float>(glm::clamp(static_cast<int>(target_render_settings.indirect_lighting_debug_view), 0, 4)), 0.0f,
      0.0f);
  strands_subdivision_x_factor = target_render_settings.strands_subdivision_x_factor;
  strands_subdivision_y_factor = target_render_settings.strands_subdivision_y_factor;
  strands_subdivision_max_x = target_render_settings.strands_subdivision_max_x;
  strands_subdivision_max_y = target_render_settings.strands_subdivision_max_y;
}

bool RenderInstanceStorage::RenderInfoBlock::operator!=(const RenderInfoBlock& other) const {
  if (split_distances != other.split_distances)
    return true;

  if (pcf_sample_amount != other.pcf_sample_amount)
    return true;

  if (shadow_cascade_transition_width != other.shadow_cascade_transition_width)
    return true;
  if (indirect_lighting_intensity != other.indirect_lighting_intensity)
    return true;

  if (strands_subdivision_x_factor != other.strands_subdivision_x_factor)
    return true;
  if (strands_subdivision_y_factor != other.strands_subdivision_y_factor)
    return true;
  if (strands_subdivision_max_x != other.strands_subdivision_max_x)
    return true;
  if (strands_subdivision_max_y != other.strands_subdivision_max_y)
    return true;

  if (directional_light_size != other.directional_light_size)
    return true;
  if (point_light_size != other.point_light_size)
    return true;
  if (spot_light_size != other.spot_light_size)
    return true;
  if (brdflut_texture_index != other.brdflut_texture_index)
    return true;

  if (debug_visualization != other.debug_visualization)
    return true;
  if (shadow_debug_parameters != other.shadow_debug_parameters)
    return true;
  if (shadow_fade_parameters != other.shadow_fade_parameters)
    return true;
  if (emissive_triangle_parameters != other.emissive_triangle_parameters)
    return true;
  if (ddgi_volume_header != other.ddgi_volume_header)
    return true;
  for (size_t i = 0; i < ddgi_volumes.size(); ++i) {
    if (ddgi_volumes[i] != other.ddgi_volumes[i])
      return true;
  }
  if (reflection_probe_header != other.reflection_probe_header)
    return true;
  for (size_t i = 0; i < reflection_probes.size(); ++i) {
    if (reflection_probes[i] != other.reflection_probes[i])
      return true;
  }

  return false;
}

bool RenderInstanceStorage::DdgiVolumeInfoBlock::operator!=(const DdgiVolumeInfoBlock& other) const {
  return first_probe != other.first_probe || probe_step_x != other.probe_step_x || probe_step_y != other.probe_step_y ||
         probe_step_z != other.probe_step_z || probe_counts != other.probe_counts ||
         probe_scroll_and_priority != other.probe_scroll_and_priority || atlas_parameters != other.atlas_parameters ||
         volume_parameters != other.volume_parameters || lighting_parameters != other.lighting_parameters ||
         identity_and_flags != other.identity_and_flags;
}

bool RenderInstanceStorage::ReflectionProbeInfoBlock::operator!=(const ReflectionProbeInfoBlock& other) const {
  return world_to_probe != other.world_to_probe || shape_parameters != other.shape_parameters ||
         projection_parameters != other.projection_parameters || lighting_parameters != other.lighting_parameters ||
         identity_and_flags != other.identity_and_flags || transition_parameters != other.transition_parameters;
}

bool RenderInstanceStorage::EmissiveTriangleInstanceSignature::operator==(
    const EmissiveTriangleInstanceSignature& other) const {
  return mesh_handle == other.mesh_handle && renderer_handle == other.renderer_handle &&
         material_handle == other.material_handle && emissive_sampling_signature == other.emissive_sampling_signature &&
         geometry_version == other.geometry_version && instance_index == other.instance_index &&
         material_index == other.material_index && triangle_offset == other.triangle_offset &&
         triangle_count == other.triangle_count && model == other.model && importance == other.importance;
}

std::vector<RenderInstanceStorage::EmissiveTriangleInfoBlock> RenderInstanceStorage::BuildEmissiveTriangleInfoBlocks(
    std::vector<EmissiveTriangleCandidate> candidates) {
  candidates.erase(std::remove_if(candidates.begin(), candidates.end(),
                                  [](const EmissiveTriangleCandidate& candidate) {
                                    return !std::isfinite(candidate.area) || !std::isfinite(candidate.importance) ||
                                           candidate.area <= 0.0 || candidate.importance <= 0.0;
                                  }),
                   candidates.end());
  std::sort(candidates.begin(), candidates.end(),
            [](const EmissiveTriangleCandidate& lhs, const EmissiveTriangleCandidate& rhs) {
              return lhs.instance_index < rhs.instance_index ||
                     (lhs.instance_index == rhs.instance_index && lhs.primitive_id < rhs.primitive_id);
            });

  double maximum_log_weight = -std::numeric_limits<double>::infinity();
  for (const auto& candidate : candidates) {
    maximum_log_weight = glm::max(maximum_log_weight, std::log(candidate.area) + std::log(candidate.importance));
  }
  if (!std::isfinite(maximum_log_weight)) {
    return {};
  }

  std::vector<double> probabilities(candidates.size());
  double scaled_weight_sum = 0.0;
  for (size_t i = 0; i < candidates.size(); ++i) {
    probabilities[i] = std::exp(std::log(candidates[i].area) + std::log(candidates[i].importance) - maximum_log_weight);
    scaled_weight_sum += probabilities[i];
  }
  if (!std::isfinite(scaled_weight_sum) || scaled_weight_sum <= 0.0) {
    return {};
  }

  std::vector<EmissiveTriangleInfoBlock> result;
  result.reserve(candidates.size());
  std::vector<double> alias_probabilities(candidates.size());
  std::vector<size_t> underfull_entries;
  std::vector<size_t> full_entries;
  for (size_t i = 0; i < candidates.size(); ++i) {
    probabilities[i] /= scaled_weight_sum;
    alias_probabilities[i] = probabilities[i] * static_cast<double>(candidates.size());
    result.push_back({candidates[i].instance_index, candidates[i].primitive_id, 1.0f, static_cast<uint32_t>(i), 0.0f});
    (alias_probabilities[i] < 1.0 ? underfull_entries : full_entries).emplace_back(i);
  }
  while (!underfull_entries.empty() && !full_entries.empty()) {
    const auto underfull_index = underfull_entries.back();
    underfull_entries.pop_back();
    const auto full_index = full_entries.back();
    full_entries.pop_back();
    result[underfull_index].alias_probability =
        static_cast<float>(glm::clamp(alias_probabilities[underfull_index], 0.0, 1.0));
    result[underfull_index].alias_index = static_cast<uint32_t>(full_index);
    alias_probabilities[full_index] += alias_probabilities[underfull_index] - 1.0;
    (alias_probabilities[full_index] < 1.0 ? underfull_entries : full_entries).emplace_back(full_index);
  }
  std::vector<double> quantized_probabilities(candidates.size());
  for (size_t i = 0; i < result.size(); ++i) {
    const double direct_probability = static_cast<double>(result[i].alias_probability) / result.size();
    quantized_probabilities[i] += direct_probability;
    quantized_probabilities[result[i].alias_index] += 1.0 / result.size() - direct_probability;
  }
  for (size_t i = 0; i < result.size(); ++i) {
    result[i].area_pdf = static_cast<float>(quantized_probabilities[i] / candidates[i].area);
  }
  return result;
}

std::vector<RenderInstanceStorage::DdgiEmissiveGuideInfoBlock> RenderInstanceStorage::BuildDdgiEmissiveGuideInfoBlocks(
    std::vector<DdgiEmissiveGuideCandidate> candidates, const uint32_t max_guide_count) {
  const auto finite_vector = [](const glm::vec3& value) {
    return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
  };
  candidates.erase(std::remove_if(candidates.begin(), candidates.end(),
                                  [&](const auto& candidate) {
                                    return !finite_vector(candidate.bound_min) || !finite_vector(candidate.bound_max) ||
                                           glm::any(glm::greaterThan(candidate.bound_min, candidate.bound_max)) ||
                                           !std::isfinite(candidate.estimated_power) ||
                                           candidate.estimated_power <= 0.0;
                                  }),
                   candidates.end());
  std::sort(candidates.begin(), candidates.end(), [](const auto& lhs, const auto& rhs) {
    if (lhs.estimated_power != rhs.estimated_power) {
      return lhs.estimated_power > rhs.estimated_power;
    }
    if (lhs.stable_id != rhs.stable_id) {
      return lhs.stable_id < rhs.stable_id;
    }
    if (lhs.source_revision != rhs.source_revision) {
      return lhs.source_revision < rhs.source_revision;
    }
    return lhs.instance_index < rhs.instance_index;
  });
  candidates.resize(glm::min(static_cast<size_t>(max_guide_count), candidates.size()));

  std::vector<DdgiEmissiveGuideInfoBlock> result;
  result.reserve(candidates.size());
  for (const auto& candidate : candidates) {
    const auto center = 0.5f * (candidate.bound_min + candidate.bound_max);
    const auto radius = 0.5f * glm::length(candidate.bound_max - candidate.bound_min);
    const auto power = static_cast<float>(
        glm::min(candidate.estimated_power, static_cast<double>((std::numeric_limits<float>::max)())));
    result.push_back({glm::vec4(center, radius), glm::vec4(power, 0.0f, 0.0f, 0.0f)});
  }
  return result;
}

bool RenderInstanceStorage::EnvironmentInfoBlock::operator!=(const EnvironmentInfoBlock& other) const {
  if (background_color != other.background_color)
    return true;
  if (environmental_map_gamma != other.environmental_map_gamma)
    return true;
  if (diffuse_sky_intensity != other.diffuse_sky_intensity)
    return true;
  if (global_reflection_intensity != other.global_reflection_intensity)
    return true;
  if (environment_type != other.environment_type)
    return true;
  if (environment_pdf_texture_index != other.environment_pdf_texture_index)
    return true;
  if (environment_cubemap_index != other.environment_cubemap_index)
    return true;
  if (environment_rotation != other.environment_rotation)
    return true;
  if (diffuse_fallback_intensity != other.diffuse_fallback_intensity)
    return true;
  if (specular_fallback_intensity != other.specular_fallback_intensity)
    return true;

  return false;
}

bool RenderInstanceStorage::InstanceInfoBlock::operator!=(const InstanceInfoBlock& other) const {
  if (model != other.model)
    return true;
  if (material_index != other.material_index)
    return true;
  if (triangle_offset != other.triangle_offset)
    return true;
  if (meshlet_index_offset != other.meshlet_index_offset)
    return true;
  if (meshlet_size != other.meshlet_size)
    return true;
  if (info_index != other.info_index)
    return true;
  if (entity_index != other.entity_index)
    return true;
  if (renderer_handle != other.renderer_handle)
    return true;
  return false;
}

void RenderInstanceStorage::CollectEntityRenderers(const std::shared_ptr<Scene>& target_scene, Bound& world_bound) {
  auto& min_bound = world_bound.min;
  auto& max_bound = world_bound.max;
  geometry_storage_version = GeometryStorage::GetVersion();
  texture_storage_version = TextureStorage::GetVersion();
  bool has_render_instance = false;
  std::unordered_set<Handle> lod_group_renderers{};
  if (const auto* owners = target_scene->UnsafeGetPrivateComponentOwnersList<LodGroup>()) {
    for (auto owner : *owners) {
      const auto lod_group = target_scene->GetOrSetPrivateComponent<LodGroup>(owner).lock();
      for (auto it = lod_group->lods.begin(); it != lod_group->lods.end(); ++it) {
        auto& lod = *it;
        bool render_current_level = true;
        if (lod_group->lod_factor > it->lod_offset) {
          render_current_level = false;
        }
        if (render_current_level && it != lod_group->lods.begin() && lod_group->lod_factor < (it - 1)->lod_offset) {
          render_current_level = false;
        }
        for (auto& renderer : lod.renderers) {
          if (const auto mesh_renderer = renderer.Get<MeshRenderer>()) {
            lod_group_renderers.insert(mesh_renderer->GetHandle());
            if (render_current_level && target_scene->IsEntityEnabled(owner) &&
                target_scene->IsEntityEnabled(mesh_renderer->GetOwner()) && mesh_renderer->IsEnabled()) {
              if (RegisterEntity(target_scene, owner, mesh_renderer, min_bound, max_bound)) {
                has_render_instance = true;
              }
            }
          } else if (const auto skinned_mesh_renderer = renderer.Get<SkinnedMeshRenderer>()) {
            lod_group_renderers.insert(skinned_mesh_renderer->GetHandle());
            if (render_current_level && target_scene->IsEntityEnabled(owner) &&
                target_scene->IsEntityEnabled(skinned_mesh_renderer->GetOwner()) &&
                skinned_mesh_renderer->IsEnabled()) {
              if (RegisterEntity(target_scene, owner, skinned_mesh_renderer, min_bound, max_bound)) {
                has_render_instance = true;
              }
            }
          } else if (const auto particles = renderer.Get<Particles>()) {
            lod_group_renderers.insert(particles->GetHandle());
            if (render_current_level && target_scene->IsEntityEnabled(owner) &&
                target_scene->IsEntityEnabled(particles->GetOwner()) && particles->IsEnabled()) {
              if (RegisterEntity(target_scene, owner, particles, min_bound, max_bound)) {
                has_render_instance = true;
              }
            }
          } else if (const auto strands_renderer = renderer.Get<StrandsRenderer>()) {
            lod_group_renderers.insert(strands_renderer->GetHandle());
            if (render_current_level && target_scene->IsEntityEnabled(owner) &&
                target_scene->IsEntityEnabled(strands_renderer->GetOwner()) && strands_renderer->IsEnabled()) {
              if (RegisterEntity(target_scene, owner, strands_renderer, min_bound, max_bound)) {
                has_render_instance = true;
              }
            }
          } else if (const auto gaussian_splat_renderer = renderer.Get<GaussianSplatRenderer>()) {
            lod_group_renderers.insert(gaussian_splat_renderer->GetHandle());
            if (render_current_level && target_scene->IsEntityEnabled(owner) &&
                target_scene->IsEntityEnabled(gaussian_splat_renderer->GetOwner()) &&
                gaussian_splat_renderer->IsEnabled()) {
              if (RegisterEntity(target_scene, owner, gaussian_splat_renderer, min_bound, max_bound)) {
                has_render_instance = true;
              }
            }
          }
        }
      }
    }
  }

  if (const auto* owners = target_scene->UnsafeGetPrivateComponentOwnersList<MeshRenderer>()) {
    for (auto owner : *owners) {
      if (!target_scene->IsEntityEnabled(owner))
        continue;
      auto mesh_renderer = target_scene->GetOrSetPrivateComponent<MeshRenderer>(owner).lock();
      if (lod_group_renderers.find(mesh_renderer->GetHandle()) != lod_group_renderers.end())
        continue;
      if (RegisterEntity(target_scene, owner, mesh_renderer, min_bound, max_bound)) {
        has_render_instance = true;
      }
    }
  }

  if (const auto* owners = target_scene->UnsafeGetPrivateComponentOwnersList<SkinnedMeshRenderer>()) {
    for (auto owner : *owners) {
      if (!target_scene->IsEntityEnabled(owner))
        continue;
      auto skinned_mesh_renderer = target_scene->GetOrSetPrivateComponent<SkinnedMeshRenderer>(owner).lock();
      if (lod_group_renderers.find(skinned_mesh_renderer->GetHandle()) != lod_group_renderers.end())
        continue;
      if (RegisterEntity(target_scene, owner, skinned_mesh_renderer, min_bound, max_bound)) {
        has_render_instance = true;
      }
    }
  }

  if (const auto* owners = target_scene->UnsafeGetPrivateComponentOwnersList<Particles>()) {
    for (auto owner : *owners) {
      if (!target_scene->IsEntityEnabled(owner))
        continue;
      auto particles = target_scene->GetOrSetPrivateComponent<Particles>(owner).lock();
      if (lod_group_renderers.find(particles->GetHandle()) != lod_group_renderers.end())
        continue;
      if (RegisterEntity(target_scene, owner, particles, min_bound, max_bound)) {
        has_render_instance = true;
      }
    }
  }

  if (const auto* owners = target_scene->UnsafeGetPrivateComponentOwnersList<StrandsRenderer>()) {
    for (auto owner : *owners) {
      if (!target_scene->IsEntityEnabled(owner))
        continue;
      auto strands_renderer = target_scene->GetOrSetPrivateComponent<StrandsRenderer>(owner).lock();
      if (lod_group_renderers.find(strands_renderer->GetHandle()) != lod_group_renderers.end())
        continue;
      if (RegisterEntity(target_scene, owner, strands_renderer, min_bound, max_bound)) {
        has_render_instance = true;
      }
    }
  }

  if (const auto* owners = target_scene->UnsafeGetPrivateComponentOwnersList<GaussianSplatRenderer>()) {
    for (auto owner : *owners) {
      if (!target_scene->IsEntityEnabled(owner))
        continue;
      auto gaussian_splat_renderer = target_scene->GetOrSetPrivateComponent<GaussianSplatRenderer>(owner).lock();
      if (lod_group_renderers.find(gaussian_splat_renderer->GetHandle()) != lod_group_renderers.end())
        continue;
      if (RegisterEntity(target_scene, owner, gaussian_splat_renderer, min_bound, max_bound)) {
        has_render_instance = true;
      }
    }
  }

  if (!has_render_instance) {
    min_bound = max_bound = glm::vec3(0.0f);
  }
}

void RenderInstanceStorage::BuildRenderInstanceBlocks() {
  total_opaque_shadow_mesh_triangles = 0;
  deferred_mesh_indirect_batches.clear();
  opaque_shadow_mesh_draw_indexed_indirect_commands.clear();
  opaque_shadow_mesh_draw_mesh_tasks_indirect_commands.clear();

  const auto register_render_instance = [&](const std::shared_ptr<IRenderInstance>& render_instance,
                                            const bool rigid_motion_supported) {
    render_instance->instance_index = instance_info_blocks_.size();
    if (render_instance->entity_handle != 0)
      instance_entity_handles_[render_instance->instance_index] = render_instance->entity_handle;
    if (render_instance->renderer_handle != 0) {
      instance_renderer_handles_[render_instance->instance_index] = render_instance->renderer_handle;
      renderer_indices_[render_instance->renderer_handle] = render_instance->instance_index;
    }
    auto& render_instance_block = instance_info_blocks_.emplace_back();
    render_instance->Apply(render_instance_block);
    rigid_motion_supported_.emplace_back(rigid_motion_supported ? 1u : 0u);
  };
  const auto prepare_gaussian_splat_render_instance =
      [&](const std::shared_ptr<GaussianSplatRenderInstance>& render_instance) {
        if (!render_instance || !render_instance->gaussian_splat) {
          return;
        }
        render_instance->geometry_version = render_instance->gaussian_splat->GetGpuDataRevision();
        const auto camera_count = std::min(cameras.size(), camera_info_blocks_.size());
        for (size_t camera_index = 0; camera_index < camera_count; ++camera_index) {
          const auto& camera = cameras[camera_index].second;
          if (!camera) {
            continue;
          }
          (void)render_instance->gaussian_splat->EnsureGpuPrepassCache(camera->GetHandle(),
                                                                       render_instance->renderer_handle);
          if (render_instance->sort_mode == GaussianSplatSortMode::CpuDepth ||
              (render_instance->sort_mode == GaussianSplatSortMode::GpuRadix &&
               !GaussianSplatGpuRadixSortSupported())) {
            (void)render_instance->gaussian_splat->EnsureSortedIndices(
                camera->GetHandle(), render_instance->renderer_handle, render_instance->model.value,
                camera_info_blocks_[camera_index].view);
          }
        }
      };
  const auto register_shadow_mesh_indirect_command = [&](const std::shared_ptr<MeshRenderInstance>& render_instance) {
    VkDrawIndexedIndirectCommand opaque_draw{};
    VkDrawMeshTasksIndirectCommandEXT opaque_mesh_task{};

    if (render_instance && render_instance->cast_shadow && render_instance->mesh) {
      const auto triangle_offset = render_instance->mesh->triangle_range_->prev_frame_offset;
      const auto triangle_index_count = render_instance->mesh->triangle_range_->prev_frame_index_count;
      const auto meshlet_range = render_instance->mesh->meshlet_range_->prev_frame_range;
      opaque_draw = CreateIndexedCommand(triangle_offset, triangle_index_count);
      opaque_mesh_task = CreateMeshTaskCommand(meshlet_range);
      total_opaque_shadow_mesh_triangles += triangle_index_count;
    }

    opaque_shadow_mesh_draw_indexed_indirect_commands.emplace_back(opaque_draw);
    opaque_shadow_mesh_draw_mesh_tasks_indirect_commands.emplace_back(opaque_mesh_task);
  };
  uint32_t deferred_mesh_command_index = 0;
  const auto register_deferred_mesh_indirect_batch = [&](const std::shared_ptr<MeshRenderInstance>& render_instance) {
    if (!render_instance || !render_instance->mesh ||
        deferred_mesh_command_index >= mesh_draw_indexed_indirect_commands.size() ||
        deferred_mesh_command_index >= mesh_draw_mesh_tasks_indirect_commands.size()) {
      deferred_mesh_command_index++;
      return;
    }
    const auto same_batch = [&](const DeferredMeshIndirectBatch& batch) {
      return batch.material_index == render_instance->material_index &&
             batch.line_width == render_instance->line_width && batch.cull_mode == render_instance->cull_mode &&
             batch.polygon_mode == render_instance->polygon_mode &&
             batch.first_instance_index + static_cast<int32_t>(batch.command_count) == render_instance->instance_index;
    };
    if (deferred_mesh_indirect_batches.empty() || !same_batch(deferred_mesh_indirect_batches.back())) {
      auto& batch = deferred_mesh_indirect_batches.emplace_back();
      batch.material_index = render_instance->material_index;
      batch.first_instance_index = render_instance->instance_index;
      batch.first_command = deferred_mesh_command_index;
      batch.line_width = render_instance->line_width;
      batch.cull_mode = render_instance->cull_mode;
      batch.polygon_mode = render_instance->polygon_mode;
    }
    auto& batch = deferred_mesh_indirect_batches.back();
    batch.command_count++;
    batch.triangle_count += render_instance->mesh->triangle_range_->prev_frame_index_count;
    deferred_mesh_command_index++;
  };
  deferred_render_instances->ForEachMeshRenderInstance([&](const auto& render_instance) {
    register_render_instance(render_instance, true);
    register_deferred_mesh_indirect_batch(render_instance);
    register_shadow_mesh_indirect_command(render_instance);
  });
  ValidateDeferredMeshIndirectCommandCount(deferred_render_instances, mesh_draw_indexed_indirect_commands,
                                           mesh_draw_mesh_tasks_indirect_commands);
  deferred_skinned_render_instances->ForEachSkinnedMeshRenderInstance([&](const auto& render_instance) {
    register_render_instance(render_instance, false);
  });
  deferred_instanced_render_instances->ForEachInstancedRenderInstance([&](const auto& render_instance) {
    register_render_instance(render_instance, false);
  });
  deferred_strands_render_instances->ForEachStrandsRenderInstance([&](const auto& render_instance) {
    register_render_instance(render_instance, false);
  });

  forward_render_instances->ForEachMeshRenderInstance([&](const auto& render_instance) {
    register_render_instance(render_instance, true);
  });
  forward_skinned_render_instances->ForEachSkinnedMeshRenderInstance([&](const auto& render_instance) {
    register_render_instance(render_instance, false);
  });
  forward_instanced_render_instances->ForEachInstancedRenderInstance([&](const auto& render_instance) {
    register_render_instance(render_instance, false);
  });
  forward_strands_render_instances->ForEachStrandsRenderInstance([&](const auto& render_instance) {
    register_render_instance(render_instance, false);
  });

  transparent_render_instances->ForEachMeshRenderInstance([&](const auto& render_instance) {
    register_render_instance(render_instance, true);
  });
  transparent_skinned_render_instances->ForEachSkinnedMeshRenderInstance([&](const auto& render_instance) {
    register_render_instance(render_instance, false);
  });
  transparent_instanced_render_instances->ForEachInstancedRenderInstance([&](const auto& render_instance) {
    register_render_instance(render_instance, false);
  });
  transparent_strands_render_instances->ForEachStrandsRenderInstance([&](const auto& render_instance) {
    register_render_instance(render_instance, false);
  });

  gaussian_splat_render_instances->ForEachGaussianSplatRenderInstance([&](const auto& render_instance) {
    prepare_gaussian_splat_render_instance(render_instance);
    register_render_instance(render_instance, false);
  });

  external_render_instances->ForEachExternalRenderInstance([&](const auto& render_instance) {
    register_render_instance(render_instance, false);
  });

  const auto append_particle_ray_instances = [&](const std::shared_ptr<InstancedRenderInstance>& render_instance) {
    render_instance->ray_tracing_instance_indices.clear();
    if (!Platform::RayAccelerationStructureEnabled() || !render_instance->particle_infos ||
        render_instance->instance_index < 0 ||
        static_cast<size_t>(render_instance->instance_index) >= instance_info_blocks_.size()) {
      return;
    }
    const auto base_block = instance_info_blocks_[render_instance->instance_index];
    for (const auto& particle_info : render_instance->particle_infos->PeekParticleInfoList()) {
      if (instance_info_blocks_.size() > 0x00ffffffu) {
        throw std::runtime_error("Ray tracing instance custom index exceeds 24 bits.");
      }
      const auto ray_instance_index = static_cast<uint32_t>(instance_info_blocks_.size());
      auto& ray_instance_block = instance_info_blocks_.emplace_back(base_block);
      ray_instance_block.model.value = render_instance->model.value * particle_info.instance_matrix.value;
      rigid_motion_supported_.emplace_back(0u);
      render_instance->ray_tracing_instance_indices.emplace_back(ray_instance_index);
      if (render_instance->entity_handle != 0) {
        instance_entity_handles_[ray_instance_index] = render_instance->entity_handle;
      }
      if (render_instance->renderer_handle != 0) {
        instance_renderer_handles_[ray_instance_index] = render_instance->renderer_handle;
      }
    }
  };
  deferred_instanced_render_instances->ForEachInstancedRenderInstance(append_particle_ray_instances);
  forward_instanced_render_instances->ForEachInstancedRenderInstance(append_particle_ray_instances);
  transparent_instanced_render_instances->ForEachInstancedRenderInstance(append_particle_ray_instances);
}

void RenderInstanceStorage::BuildEmissiveTriangleInfoBlocks() {
  if (!Platform::RayAccelerationStructureEnabled()) {
    emissive_triangle_info_dirty_ = !emissive_triangle_info_blocks_.empty();
    emissive_triangle_info_blocks_.clear();
    ddgi_emissive_guide_info_blocks_.clear();
    emissive_triangle_instance_signatures_.clear();
    ddgi_emissive_inventory_stats_ = {};
    ddgi_emissive_inventory_signature_ = 0;
    render_info_block.emissive_triangle_parameters = glm::uvec4(0u);
    return;
  }

  struct EmissiveTriangleInstance {
    std::shared_ptr<IRenderInstance> render_instance;
    uint32_t instance_index;
    uint32_t triangle_offset;
    uint32_t triangle_count;
    glm::mat4 model;
    double sidedness;
    uint64_t stable_id;
    uint32_t source_revision;
  };
  std::vector<EmissiveTriangleInstance> emissive_instances;
  std::vector<EmissiveTriangleInstanceSignature> signatures;
  EmissiveTriangleInventoryStats inventory_stats{};
  const auto& shade_materials = gltf_material_cache_.GetShadeMaterials();
  const auto record_excluded_if_emissive = [&](const std::shared_ptr<IRenderInstance>& render_instance) {
    if (!render_instance || render_instance->material_index < 0 ||
        static_cast<size_t>(render_instance->material_index) >= shade_materials.size()) {
      return;
    }
    const auto& material = shade_materials[render_instance->material_index];
    const glm::vec3 factor = glm::max(material.emissive_factor, glm::vec3(0.0f));
    if (std::isfinite(factor.x) && std::isfinite(factor.y) && std::isfinite(factor.z) &&
        glm::dot(factor, glm::vec3(0.2126f, 0.7152f, 0.0722f)) > 0.0f) {
      ++inventory_stats.excluded_emissive_instance_count;
    }
  };
  const auto append_instance = [&](const std::shared_ptr<IRenderInstance>& render_instance, const uint64_t mesh_handle,
                                   const uint32_t instance_index, const uint32_t triangle_offset,
                                   const uint32_t triangle_count, const glm::mat4& model,
                                   const bool has_ray_tracing_geometry) {
    if (!render_instance || !render_instance->material || render_instance->material_index < 0 ||
        static_cast<size_t>(render_instance->material_index) >= shade_materials.size()) {
      return;
    }
    const auto& material = shade_materials[render_instance->material_index];
    const glm::vec3 emissive_factor = glm::max(material.emissive_factor, glm::vec3(0.0f));
    double importance = static_cast<double>(glm::dot(emissive_factor, glm::vec3(0.2126f, 0.7152f, 0.0722f)));
    if (!std::isfinite(importance) || importance <= 0.0) {
      return;
    }
    if (!has_ray_tracing_geometry || instance_index >= instance_info_blocks_.size() || triangle_count == 0u ||
        render_instance->polygon_mode != VK_POLYGON_MODE_FILL || material.unlit != 0) {
      ++inventory_stats.excluded_emissive_instance_count;
      return;
    }
    const float transform_determinant = glm::determinant(glm::mat3(model));
    if (!std::isfinite(transform_determinant) || transform_determinant == 0.0f) {
      ++inventory_stats.excluded_emissive_instance_count;
      return;
    }
    const double sidedness = material.double_sided != 0 ? 2.0 : 1.0;
    importance *= sidedness;
    GlobalTransform signature_model{};
    signature_model.value = model;
    const auto emissive_sampling_signature = GetEmissiveSamplingSignature(*render_instance->material, material);
    signatures.push_back(
        {mesh_handle, render_instance->renderer_handle, render_instance->material->GetHandle().GetValue(),
         emissive_sampling_signature, render_instance->geometry_version, static_cast<int32_t>(instance_index),
         render_instance->material_index, triangle_offset, triangle_count, signature_model, importance});
    const uint64_t stable_id = render_instance->renderer_handle != 0u ? render_instance->renderer_handle.GetValue()
                                                                      : static_cast<uint64_t>(instance_index);
    const auto source_revision = static_cast<uint32_t>(MixDdgiInventorySignature(
        emissive_sampling_signature, static_cast<uint64_t>(render_instance->geometry_version)));
    emissive_instances.push_back({render_instance, instance_index, triangle_offset, triangle_count, model, sidedness,
                                  stable_id, source_revision});
    ++inventory_stats.eligible_instance_count;
  };
  const auto append_mesh_collection = [&](const std::shared_ptr<MeshRenderInstanceCollection>& collection) {
    collection->ForEachMeshRenderInstance([&](const std::shared_ptr<MeshRenderInstance>& render_instance) {
      if (!render_instance || !render_instance->mesh || !render_instance->mesh->triangle_range_) {
        record_excluded_if_emissive(render_instance);
        return;
      }
      const auto triangle_range = render_instance->ray_tracing_triangle_range &&
                                          render_instance->ray_tracing_triangle_range->prev_frame_index_count != 0u
                                      ? render_instance->ray_tracing_triangle_range
                                      : render_instance->mesh->triangle_range_;
      append_instance(render_instance, render_instance->mesh->GetHandle().GetValue(), render_instance->instance_index,
                      triangle_range->prev_frame_offset, triangle_range->prev_frame_index_count,
                      render_instance->model.value,
                      static_cast<bool>(render_instance->ray_tracing_blas ? render_instance->ray_tracing_blas
                                                                          : render_instance->mesh->blas_));
    });
  };
  const auto append_skinned_collection = [&](const std::shared_ptr<SkinnedMeshRenderInstanceCollection>& collection) {
    collection->ForEachSkinnedMeshRenderInstance(
        [&](const std::shared_ptr<SkinnedMeshRenderInstance>& render_instance) {
          if (!render_instance || !render_instance->skinned_mesh) {
            record_excluded_if_emissive(render_instance);
            return;
          }
          auto triangle_range = render_instance->ray_tracing_triangle_range;
          if (!triangle_range || triangle_range->prev_frame_index_count == 0u) {
            triangle_range = render_instance->skinned_mesh->ray_tracing_triangle_range_;
          }
          if (!triangle_range || triangle_range->prev_frame_index_count == 0u) {
            triangle_range = render_instance->skinned_mesh->skinned_triangle_range_;
          }
          if (!triangle_range) {
            record_excluded_if_emissive(render_instance);
            return;
          }
          append_instance(render_instance, render_instance->skinned_mesh->GetHandle().GetValue(),
                          render_instance->instance_index, triangle_range->prev_frame_offset,
                          triangle_range->prev_frame_index_count, render_instance->model.value,
                          static_cast<bool>(render_instance->ray_tracing_blas ? render_instance->ray_tracing_blas
                                                                              : render_instance->skinned_mesh->blas_));
        });
  };
  const auto append_instanced_collection = [&](const std::shared_ptr<InstancedRenderInstanceCollection>& collection) {
    collection->ForEachInstancedRenderInstance([&](const std::shared_ptr<InstancedRenderInstance>& render_instance) {
      if (!render_instance || !render_instance->mesh || !render_instance->mesh->triangle_range_) {
        record_excluded_if_emissive(render_instance);
        return;
      }
      if (render_instance->ray_tracing_instance_indices.empty()) {
        record_excluded_if_emissive(render_instance);
      }
      for (const auto instance_index : render_instance->ray_tracing_instance_indices) {
        if (instance_index >= instance_info_blocks_.size()) {
          continue;
        }
        append_instance(render_instance, render_instance->mesh->GetHandle().GetValue(), instance_index,
                        render_instance->mesh->triangle_range_->prev_frame_offset,
                        render_instance->mesh->triangle_range_->prev_frame_index_count,
                        instance_info_blocks_[instance_index].model.value,
                        static_cast<bool>(render_instance->mesh->blas_));
      }
    });
  };
  append_mesh_collection(deferred_render_instances);
  append_skinned_collection(deferred_skinned_render_instances);
  append_instanced_collection(deferred_instanced_render_instances);
  append_mesh_collection(forward_render_instances);
  append_skinned_collection(forward_skinned_render_instances);
  append_instanced_collection(forward_instanced_render_instances);
  append_mesh_collection(transparent_render_instances);
  append_skinned_collection(transparent_skinned_render_instances);
  append_instanced_collection(transparent_instanced_render_instances);
  external_render_instances->ForEachExternalRenderInstance(
      [&](const std::shared_ptr<ExternalRenderInstance>& render_instance) {
        if (!render_instance || !render_instance->HasDdgiRayTracingGeometry()) {
          record_excluded_if_emissive(render_instance);
          return;
        }
        append_instance(render_instance, 0u, render_instance->instance_index,
                        static_cast<uint32_t>(render_instance->ddgi_geometry.triangle_offset),
                        render_instance->ddgi_geometry.triangle_count, render_instance->model.value, true);
      });

  if (emissive_triangle_instance_signatures_ == signatures) {
    inventory_stats.unrepresentable_probability_count =
        ddgi_emissive_inventory_stats_.unrepresentable_probability_count;
    inventory_stats.estimated_emitted_power = ddgi_emissive_inventory_stats_.estimated_emitted_power;
    ddgi_emissive_inventory_stats_ = inventory_stats;
    render_info_block.emissive_triangle_parameters.x = static_cast<uint32_t>(emissive_triangle_info_blocks_.size());
    render_info_block.emissive_triangle_parameters.y = static_cast<uint32_t>(ddgi_emissive_guide_info_blocks_.size());
    return;
  }

  std::vector<EmissiveTriangleCandidate> candidates;
  std::vector<DdgiEmissiveGuideCandidate> guide_candidates;
  for (const auto& emissive_instance : emissive_instances) {
    const auto& render_instance = emissive_instance.render_instance;
    const auto& material = shade_materials[render_instance->material_index];
    glm::vec3 bound_min((std::numeric_limits<float>::max)());
    glm::vec3 bound_max((std::numeric_limits<float>::lowest)());
    double estimated_power = 0.0;
    bool has_finite_bounds = false;
    for (uint32_t primitive_id = 0; primitive_id < emissive_instance.triangle_count; ++primitive_id) {
      const auto& triangle = GeometryStorage::PeekTriangle(emissive_instance.triangle_offset + primitive_id);
      const auto& v0 = GeometryStorage::PeekVertex(triangle.x);
      const auto& v1 = GeometryStorage::PeekVertex(triangle.y);
      const auto& v2 = GeometryStorage::PeekVertex(triangle.z);
      const glm::vec3 p0 = glm::vec3(emissive_instance.model * glm::vec4(v0.position, 1.0f));
      const glm::vec3 p1 = glm::vec3(emissive_instance.model * glm::vec4(v1.position, 1.0f));
      const glm::vec3 p2 = glm::vec3(emissive_instance.model * glm::vec4(v2.position, 1.0f));
      const auto finite_position = [](const glm::vec3& value) {
        return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
      };
      if (finite_position(p0) && finite_position(p1) && finite_position(p2)) {
        bound_min = glm::min(bound_min, glm::min(p0, glm::min(p1, p2)));
        bound_max = glm::max(bound_max, glm::max(p0, glm::max(p1, p2)));
        has_finite_bounds = true;
      }
      const glm::vec3 weighted_normal = glm::cross(p1 - p0, p2 - p0);
      const double area = 0.5 * static_cast<double>(glm::length(weighted_normal));
      const double importance = EstimateTriangleEmissiveImportance(*render_instance->material, material, v0, v1, v2) *
                                EstimateTriangleOpacityImportance(*render_instance->material, material, v0, v1, v2) *
                                emissive_instance.sidedness;
      if (std::isfinite(area) && area > 0.0 && std::isfinite(importance) && importance > 0.0) {
        const auto triangle_power = glm::pi<double>() * area * importance;
        inventory_stats.estimated_emitted_power += triangle_power;
        estimated_power += triangle_power;
      }
      candidates.push_back({emissive_instance.instance_index, primitive_id, area, importance});
    }
    if (has_finite_bounds) {
      guide_candidates.push_back({bound_min, bound_max, estimated_power, emissive_instance.stable_id,
                                  emissive_instance.source_revision, emissive_instance.instance_index});
    }
  }
  emissive_triangle_info_blocks_ = BuildEmissiveTriangleInfoBlocks(std::move(candidates));
  ddgi_emissive_guide_info_blocks_ = BuildDdgiEmissiveGuideInfoBlocks(std::move(guide_candidates));
  inventory_stats.unrepresentable_probability_count = static_cast<uint32_t>(std::count_if(
      emissive_triangle_info_blocks_.begin(), emissive_triangle_info_blocks_.end(), [](const auto& record) {
        return record.area_pdf <= 0.0f;
      }));
  ddgi_emissive_inventory_stats_ = inventory_stats;
  emissive_triangle_instance_signatures_ = std::move(signatures);
  ddgi_emissive_inventory_signature_ = HashDdgiEmissiveInventorySignature(emissive_triangle_instance_signatures_);
  emissive_triangle_info_dirty_ = true;
  render_info_block.emissive_triangle_parameters.x = static_cast<uint32_t>(emissive_triangle_info_blocks_.size());
  render_info_block.emissive_triangle_parameters.y = static_cast<uint32_t>(ddgi_emissive_guide_info_blocks_.size());
}

void RenderInstanceStorage::CollectLights(const std::shared_ptr<Scene>& target_scene, const Bound& world_bound) {
#pragma region Directional Light
  const std::vector<Entity>* directional_light_entities =
      target_scene->UnsafeGetPrivateComponentOwnersList<DirectionalLight>();
  render_info_block.directional_light_size = 0;
  const auto& graphics_settings = ApplicationContext::Get().GetApplicationInfo().graphics_settings;
  const auto max_directional_light_size = graphics_settings.max_directional_light_size;

  if (directional_light_entities && !directional_light_entities->empty() && max_directional_light_size > 0) {
    directional_light_info_blocks_.resize(max_directional_light_size * cameras.size());
    uint32_t directional_light_size = 0;
    uint32_t directional_shadow_light_size = 0;
    for (const auto& light_entity : *directional_light_entities) {
      if (!target_scene->IsEntityEnabled(light_entity))
        continue;
      const auto dlc = target_scene->GetOrSetPrivateComponent<DirectionalLight>(light_entity).lock();
      if (!dlc->IsEnabled())
        continue;
      if (directional_light_size >= max_directional_light_size)
        break;
      directional_light_size++;
      if (dlc->cast_shadow) {
        directional_shadow_light_size++;
      }
    }
    render_info_block.directional_light_size = static_cast<int>(directional_light_size);
    std::vector<glm::uvec3> viewport_results;
    Lighting::AllocateAtlas(directional_shadow_light_size, graphics_settings.directional_light_shadow_map_resolution,
                            viewport_results);
    for (const auto& [cameraGlobalTransform, camera] : cameras) {
      auto camera_index = GetCameraIndex(camera->GetHandle());
      size_t directional_light_index = 0;
      size_t directional_shadow_light_index = 0;
      for (const auto& light_entity : *directional_light_entities) {
        if (!target_scene->IsEntityEnabled(light_entity))
          continue;
        const auto dlc = target_scene->GetOrSetPrivateComponent<DirectionalLight>(light_entity).lock();
        if (!dlc->IsEnabled())
          continue;
        if (directional_light_index >= max_directional_light_size)
          break;
        const auto block_index = camera_index * max_directional_light_size + directional_light_index;
        auto& viewport = directional_light_info_blocks_[block_index].viewport;
        viewport = glm::ivec4(0);
        if (dlc->cast_shadow && directional_shadow_light_index < viewport_results.size()) {
          viewport.x = viewport_results[directional_shadow_light_index].x;
          viewport.y = viewport_results[directional_shadow_light_index].y;
          viewport.z = viewport_results[directional_shadow_light_index].z;
          viewport.w = viewport_results[directional_shadow_light_index].z;
          directional_shadow_light_index++;
        }
        directional_light_index++;
      }
    }

    for (const auto& [cameraGlobalTransform, camera] : cameras) {
      size_t directional_light_index = 0;
      auto camera_index = GetCameraIndex(camera->GetHandle());
      const auto& split_distances = camera_info_blocks_[camera_index].shadow_split_distances;
      glm::vec3 main_camera_pos = cameraGlobalTransform.GetPosition();
      glm::quat main_camera_rot = cameraGlobalTransform.GetRotation();
      for (const auto& light_entity : *directional_light_entities) {
        if (!target_scene->IsEntityEnabled(light_entity))
          continue;
        const auto dlc = target_scene->GetOrSetPrivateComponent<DirectionalLight>(light_entity).lock();
        if (!dlc->IsEnabled())
          continue;
        if (directional_light_index >= max_directional_light_size)
          break;
        glm::quat rotation = target_scene->GetDataComponent<GlobalTransform>(light_entity).GetRotation();
        glm::vec3 light_dir = glm::normalize(rotation * glm::vec3(0, 0, 1));
        const auto block_index = camera_index * max_directional_light_size + directional_light_index;
        directional_light_info_blocks_[block_index].direction = glm::vec4(light_dir, 0.0f);
        directional_light_info_blocks_[block_index].diffuse =
            glm::vec4(dlc->diffuse * dlc->diffuse_brightness, dlc->cast_shadow);
        directional_light_info_blocks_[block_index].specular = glm::vec4(0.0f);
        const auto camera_near_distance = glm::max(camera->camera_settings.near_distance, 0.001f);
        for (int split = 0; split < 4; split++) {
          float split_start = camera_near_distance;
          const float split_end = split_distances[split];
          if (split != 0) {
            split_start = split_distances[split - 1];
          }
          auto fit_start = split_start;
          auto fit_end = split_end;
          if (split != 0) {
            fit_start = glm::max(
                camera_near_distance,
                split_start - render_settings.GetShadowCascadeTransitionHalfWidth(split - 1, camera_near_distance));
          }
          if (split != 3) {
            fit_end = glm::min(split_distances.w, split_end + render_settings.GetShadowCascadeTransitionHalfWidth(
                                                                  split, camera_near_distance));
          }
          std::array<glm::vec3, 8> frustum_corners{};
          Camera::CalculateFrustumPoints(camera, fit_start, fit_end, main_camera_pos, main_camera_rot,
                                         frustum_corners.data());
          const auto camera_size = glm::max(glm::vec2(camera->GetSize()), glm::vec2(1.0f));
          const auto far_half_height =
              glm::tan(glm::radians(camera->camera_settings.fov * 0.25f)) * glm::max(fit_end, 0.0f);
          const auto jitter_margin_world = glm::length(
              glm::vec2(far_half_height * camera->GetSizeRatio() / camera_size.x, far_half_height / camera_size.y));
          const auto& viewport = directional_light_info_blocks_[block_index].viewport;
          const auto fit = CalculateDirectionalShadowCascadeFit({
              render_settings.shadow_cascade_fit_mode,
              frustum_corners,
              world_bound,
              light_dir,
              glm::normalize(rotation * glm::vec3(0, 1, 0)),
              glm::ivec2(viewport.z, viewport.w),
              glm::max(dlc->light_size, 0.0f) + jitter_margin_world,
          });
          directional_light_info_blocks_[block_index].light_space_matrix[split] = fit.light_space_matrix;
          directional_light_info_blocks_[block_index].light_frustum_width[split] =
              (fit.orthographic_max.x - fit.orthographic_min.x) * 0.5f;
          directional_light_info_blocks_[block_index].light_frustum_height[split] =
              (fit.orthographic_max.y - fit.orthographic_min.y) * 0.5f;
          directional_light_info_blocks_[block_index].light_frustum_distance[split] = fit.light_space_depth_half_extent;
          if (split == 4 - 1)
            directional_light_info_blocks_[block_index].reserved_parameters =
                glm::vec4(dlc->light_size, dlc->slope_bias, dlc->bias, dlc->normal_offset);
        }
        directional_light_index++;
      }
    }
  }
#pragma endregion

  const auto main_camera = target_scene->main_camera.Get<Camera>();
  GlobalTransform main_camera_global_transform{};
  if (main_camera) {
    if (const auto main_camera_owner = main_camera->GetOwner(); target_scene->IsEntityValid(main_camera_owner)) {
      main_camera_global_transform = target_scene->GetDataComponent<GlobalTransform>(main_camera_owner);
    }
  }
  const glm::vec3 main_camera_position = main_camera_global_transform.GetPosition();
  const std::vector<Entity>* point_light_entities = target_scene->UnsafeGetPrivateComponentOwnersList<PointLight>();
  render_info_block.point_light_size = 0;
  if (point_light_entities && !point_light_entities->empty()) {
    point_light_info_blocks_.resize(point_light_entities->size());
    std::multimap<float, size_t> sorted_point_shadow_light_indices;
    uint32_t point_shadow_light_size = 0;
    for (int i = 0; i < point_light_entities->size(); i++) {
      Entity light_entity = point_light_entities->at(i);
      if (!target_scene->IsEntityEnabled(light_entity))
        continue;
      const auto plc = target_scene->GetOrSetPrivateComponent<PointLight>(light_entity).lock();
      if (!plc->IsEnabled())
        continue;
      glm::vec3 position = target_scene->GetDataComponent<GlobalTransform>(light_entity).value[3];
      point_light_info_blocks_[render_info_block.point_light_size].position = glm::vec4(position, 0);
      point_light_info_blocks_[render_info_block.point_light_size].constant_linear_quad_far_plane.x = plc->constant;
      point_light_info_blocks_[render_info_block.point_light_size].constant_linear_quad_far_plane.y = plc->linear;
      point_light_info_blocks_[render_info_block.point_light_size].constant_linear_quad_far_plane.z = plc->quadratic;
      point_light_info_blocks_[render_info_block.point_light_size].diffuse =
          glm::vec4(plc->diffuse * plc->diffuse_brightness, plc->cast_shadow);
      point_light_info_blocks_[render_info_block.point_light_size].specular = glm::vec4(0);
      point_light_info_blocks_[render_info_block.point_light_size].viewport = glm::ivec4(0);
      point_light_info_blocks_[render_info_block.point_light_size].constant_linear_quad_far_plane.w =
          plc->range > 0.0f ? plc->range : plc->GetFarPlane();

      glm::mat4 shadow_proj =
          glm::perspective(glm::radians(90.0f), 1.0f, plc->shadow_distance / 1000.f, plc->shadow_distance);
      point_light_info_blocks_[render_info_block.point_light_size].light_space_matrix[0] =
          shadow_proj * glm::lookAt(position, position + glm::vec3(1.0f, 0.0f, 0.0f), glm::vec3(0.0f, -1.0f, 0.0f));
      point_light_info_blocks_[render_info_block.point_light_size].light_space_matrix[1] =
          shadow_proj * glm::lookAt(position, position + glm::vec3(-1.0f, 0.0f, 0.0f), glm::vec3(0.0f, -1.0f, 0.0f));
      point_light_info_blocks_[render_info_block.point_light_size].light_space_matrix[2] =
          shadow_proj * glm::lookAt(position, position + glm::vec3(0.0f, 1.0f, 0.0f), glm::vec3(0.0f, 0.0f, 1.0f));
      point_light_info_blocks_[render_info_block.point_light_size].light_space_matrix[3] =
          shadow_proj * glm::lookAt(position, position + glm::vec3(0.0f, -1.0f, 0.0f), glm::vec3(0.0f, 0.0f, -1.0f));
      point_light_info_blocks_[render_info_block.point_light_size].light_space_matrix[4] =
          shadow_proj * glm::lookAt(position, position + glm::vec3(0.0f, 0.0f, 1.0f), glm::vec3(0.0f, -1.0f, 0.0f));
      point_light_info_blocks_[render_info_block.point_light_size].light_space_matrix[5] =
          shadow_proj * glm::lookAt(position, position + glm::vec3(0.0f, 0.0f, -1.0f), glm::vec3(0.0f, -1.0f, 0.0f));
      point_light_info_blocks_[render_info_block.point_light_size].reserved_parameters =
          glm::vec4(plc->bias, plc->light_size, 0, 0);

      if (plc->cast_shadow) {
        sorted_point_shadow_light_indices.insert(
            {glm::distance(main_camera_position, position), render_info_block.point_light_size});
        point_shadow_light_size++;
      }
      render_info_block.point_light_size++;
    }
    std::vector<glm::uvec3> view_port_results;
    Lighting::AllocateAtlas(point_shadow_light_size, graphics_settings.point_light_shadow_map_resolution,
                            view_port_results);
    int allocation_index = 0;
    for (const auto& point_light_index : sorted_point_shadow_light_indices) {
      auto& viewport = point_light_info_blocks_[point_light_index.second].viewport;
      viewport.x = view_port_results[allocation_index].x;
      viewport.y = view_port_results[allocation_index].y;
      viewport.z = view_port_results[allocation_index].z;
      viewport.w = view_port_results[allocation_index].z;

      allocation_index++;
    }
  }
  point_light_info_blocks_.resize(render_info_block.point_light_size);

  render_info_block.spot_light_size = 0;
  const std::vector<Entity>* spot_light_entities = target_scene->UnsafeGetPrivateComponentOwnersList<SpotLight>();
  if (spot_light_entities && !spot_light_entities->empty()) {
    spot_light_info_blocks_.resize(spot_light_entities->size());
    std::multimap<float, size_t> sorted_spot_shadow_light_indices;
    uint32_t spot_shadow_light_size = 0;
    for (auto light_entity : *spot_light_entities) {
      if (!target_scene->IsEntityEnabled(light_entity))
        continue;
      const auto slc = target_scene->GetOrSetPrivateComponent<SpotLight>(light_entity).lock();
      if (!slc->IsEnabled())
        continue;
      auto ltw = target_scene->GetDataComponent<GlobalTransform>(light_entity);
      glm::vec3 position = ltw.value[3];
      glm::vec3 front = ltw.GetRotation() * glm::vec3(0, 0, -1);
      glm::vec3 up = ltw.GetRotation() * glm::vec3(0, 1, 0);
      spot_light_info_blocks_[render_info_block.spot_light_size].position = glm::vec4(position, 0);
      spot_light_info_blocks_[render_info_block.spot_light_size].direction = glm::vec4(front, 0);
      spot_light_info_blocks_[render_info_block.spot_light_size].constant_linear_quad_far_plane.x = slc->constant;
      spot_light_info_blocks_[render_info_block.spot_light_size].constant_linear_quad_far_plane.y = slc->linear;
      spot_light_info_blocks_[render_info_block.spot_light_size].constant_linear_quad_far_plane.z = slc->quadratic;
      spot_light_info_blocks_[render_info_block.spot_light_size].constant_linear_quad_far_plane.w =
          slc->range > 0.0f ? slc->range : slc->GetFarPlane();
      spot_light_info_blocks_[render_info_block.spot_light_size].diffuse =
          glm::vec4(slc->diffuse * slc->diffuse_brightness, slc->cast_shadow);
      spot_light_info_blocks_[render_info_block.spot_light_size].specular = glm::vec4(0);
      spot_light_info_blocks_[render_info_block.spot_light_size].viewport = glm::ivec4(0);

      glm::mat4 shadow_proj = glm::perspective(glm::radians(slc->outer_degrees * 2.0f), 1.0f,
                                               slc->shadow_distance / 1000.f, slc->shadow_distance);
      spot_light_info_blocks_[render_info_block.spot_light_size].light_space_matrix =
          shadow_proj * glm::lookAt(position, position + front, up);
      spot_light_info_blocks_[render_info_block.spot_light_size].cut_off_outer_cut_off_light_size_bias =
          glm::vec4(glm::cos(glm::radians(slc->inner_degrees)), glm::cos(glm::radians(slc->outer_degrees)),
                    slc->light_size, slc->bias);

      if (slc->cast_shadow) {
        sorted_spot_shadow_light_indices.insert(
            {glm::distance(main_camera_position, position), render_info_block.spot_light_size});
        spot_shadow_light_size++;
      }
      render_info_block.spot_light_size++;
    }
    std::vector<glm::uvec3> view_port_results;
    Lighting::AllocateAtlas(spot_shadow_light_size, graphics_settings.spot_light_shadow_map_resolution,
                            view_port_results);
    int allocation_index = 0;
    for (const auto& spot_light_index : sorted_spot_shadow_light_indices) {
      auto& view_port = spot_light_info_blocks_[spot_light_index.second].viewport;
      view_port.x = view_port_results[allocation_index].x;
      view_port.y = view_port_results[allocation_index].y;
      view_port.z = view_port_results[allocation_index].z;
      view_port.w = view_port_results[allocation_index].z;
      allocation_index++;
    }
  }
  spot_light_info_blocks_.resize(render_info_block.spot_light_size);
}

void RenderInstanceStorage::CollectEnvironment(const std::shared_ptr<Scene>& target_scene) {
  environment_info_block = {};
  environment_info_block.environment_pdf_texture_index = -1.0f;
  environment_info_block.environment_cubemap_index = -1.0f;
  if (!target_scene) {
    render_info_block.indirect_lighting_intensity = 1.0f;
    return;
  }
  const auto resolved_lighting = ResolveEnvironmentalLighting(target_scene);
  const auto& source = resolved_lighting.indirect_environment_source;
  if (source.kind == ResolvedEnvironmentalLighting::IndirectEnvironmentSourceKind::Color) {
    environment_info_block.background_color = glm::vec4(source.color, 1.0f);
    environment_info_block.environment_type = 1.0f;
  } else {
    environment_info_block.background_color.w = 0.0f;
    environment_info_block.environment_type = 0.0f;
    if (const auto environmental_map = ResolveIndirectEnvironmentMap(source)) {
      environmental_map->EnsureEnvironmentSource();
      if (auto pdf_ref = environmental_map->environment_pdf_texture;
          const auto pdf_texture = pdf_ref.Get<Texture2D>()) {
        environment_info_block.environment_pdf_texture_index =
            static_cast<float>(pdf_texture->GetTextureStorageIndex());
      }
      if (auto cubemap_ref = environmental_map->environment_cubemap; const auto cubemap = cubemap_ref.Get<Cubemap>()) {
        environment_info_block.environment_cubemap_index = static_cast<float>(cubemap->GetTextureStorageIndex());
      }
    }
  }
  environment_info_block.environmental_map_gamma = source.gamma;
  environment_info_block.environment_rotation = source.rotation;
  const float environment_lighting_intensity = glm::max(resolved_lighting.environment_lighting_intensity, 0.0f);
  const float diffuse_fallback_intensity = glm::max(resolved_lighting.diffuse_fallback_intensity, 0.0f);
  const float specular_fallback_intensity = glm::max(resolved_lighting.specular_fallback_intensity, 0.0f);
  environment_info_block.diffuse_sky_intensity = environment_lighting_intensity;
  environment_info_block.global_reflection_intensity = environment_lighting_intensity;
  environment_info_block.diffuse_fallback_intensity = diffuse_fallback_intensity;
  environment_info_block.specular_fallback_intensity = specular_fallback_intensity;
  render_info_block.indirect_lighting_intensity = 1.0f;
}

void RenderInstanceStorage::CollectEditorCameras(
    const std::shared_ptr<Scene>& target_scene,
    std::vector<std::pair<GlobalTransform, std::shared_ptr<Camera>>>& cameras) {
  if (const auto editor_layer = ApplicationContext::Get().GetLayer<EditorLayer>()) {
    for (const auto& [cameraHandle, editorCamera] : editor_layer->editor_cameras_) {
      if (editorCamera.camera || editorCamera.camera->IsEnabled()) {
        GlobalTransform scene_camera_gt;
        scene_camera_gt.SetValue(editorCamera.position, editorCamera.rotation, glm::vec3(1.0f));
        cameras.emplace_back(scene_camera_gt, editorCamera.camera);
      }
    }
  }
}

void RenderInstanceStorage::CollectCameras(const std::shared_ptr<Scene>& target_scene,
                                           std::vector<std::pair<GlobalTransform, std::shared_ptr<Camera>>>& cameras) {
  if (const std::vector<Entity>* camera_entities = target_scene->UnsafeGetPrivateComponentOwnersList<Camera>()) {
    for (const auto& i : *camera_entities) {
      if (!target_scene->IsEntityEnabled(i))
        continue;
      assert(target_scene->HasPrivateComponent<Camera>(i));
      auto camera = target_scene->GetOrSetPrivateComponent<Camera>(i).lock();
      if (!camera || !camera->IsEnabled())
        continue;
      auto camera_global_transform = target_scene->GetDataComponent<GlobalTransform>(i);
      cameras.emplace_back(camera_global_transform, camera);
    }
  }
}

RenderInstanceStorage::RenderInstanceStorage() {
  VkBufferCreateInfo buffer_create_info{};
  buffer_create_info.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
  buffer_create_info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
  VmaAllocationCreateInfo buffer_vma_allocation_create_info{};
  buffer_vma_allocation_create_info.usage = VMA_MEMORY_USAGE_AUTO_PREFER_DEVICE;

  buffer_create_info.usage = VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT;
  buffer_create_info.size = sizeof(RenderInfoBlock);
  render_info_descriptor_buffer = std::make_shared<Buffer>(buffer_create_info, buffer_vma_allocation_create_info);
  buffer_create_info.size = sizeof(EnvironmentInfoBlock);
  environment_info_descriptor_buffer = std::make_shared<Buffer>(buffer_create_info, buffer_vma_allocation_create_info);

  const auto& graphics_settings = ApplicationContext::Get().GetApplicationInfo().graphics_settings;

  buffer_create_info.usage = VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_STORAGE_BUFFER_BIT;
  buffer_create_info.size = sizeof(CameraInfoBlock) * Platform::Constants::initial_camera_size;
  camera_info_descriptor_buffer = std::make_shared<Buffer>(buffer_create_info, buffer_vma_allocation_create_info);
  buffer_create_info.size = sizeof(DirectionalLightInfoBlock) * graphics_settings.max_directional_light_size *
                            Platform::Constants::initial_camera_size;
  directional_light_info_descriptor_buffer =
      std::make_shared<Buffer>(buffer_create_info, buffer_vma_allocation_create_info);
  buffer_create_info.size = sizeof(PointLightInfoBlock) * graphics_settings.max_point_light_size;
  point_light_info_descriptor_buffer = std::make_shared<Buffer>(buffer_create_info, buffer_vma_allocation_create_info);
  buffer_create_info.size = sizeof(SpotLightInfoBlock) * graphics_settings.max_spot_light_size;
  spot_light_info_descriptor_buffer = std::make_shared<Buffer>(buffer_create_info, buffer_vma_allocation_create_info);
  buffer_create_info.size =
      glm::max(static_cast<size_t>(1), sizeof(GltfShadeMaterial) * Platform::Constants::initial_material_size);
  gltf_material_descriptor_buffer = std::make_shared<Buffer>(buffer_create_info, buffer_vma_allocation_create_info);
  buffer_create_info.size =
      glm::max(static_cast<size_t>(1), sizeof(GltfTextureInfo) * Platform::Constants::initial_material_size);
  gltf_texture_info_descriptor_buffer = std::make_shared<Buffer>(buffer_create_info, buffer_vma_allocation_create_info);
  buffer_create_info.size =
      glm::max(static_cast<size_t>(1), sizeof(InstanceInfoBlock) * Platform::Constants::initial_instance_size);
  instance_info_descriptor_buffer = std::make_shared<Buffer>(buffer_create_info, buffer_vma_allocation_create_info);
  buffer_create_info.size =
      glm::max(static_cast<size_t>(1), sizeof(PreviousInstanceInfoBlock) * Platform::Constants::initial_instance_size);
  previous_instance_info_descriptor_buffer =
      std::make_shared<Buffer>(buffer_create_info, buffer_vma_allocation_create_info);
  buffer_create_info.size = sizeof(EmissiveTriangleInfoBlock);
  emissive_triangle_info_descriptor_buffer =
      std::make_shared<Buffer>(buffer_create_info, buffer_vma_allocation_create_info);
  buffer_create_info.usage = VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_INDIRECT_BUFFER_BIT;
  buffer_create_info.size = glm::max(static_cast<size_t>(1),
                                     sizeof(VkDrawIndexedIndirectCommand) * mesh_draw_indexed_indirect_commands.size());
  mesh_draw_indexed_indirect_commands_buffer =
      std::make_shared<Buffer>(buffer_create_info, buffer_vma_allocation_create_info);

  buffer_create_info.size = glm::max(static_cast<size_t>(1), sizeof(VkDrawMeshTasksIndirectCommandEXT) *
                                                                 mesh_draw_mesh_tasks_indirect_commands.size());
  mesh_draw_mesh_tasks_indirect_commands_buffer =
      std::make_shared<Buffer>(buffer_create_info, buffer_vma_allocation_create_info);

  buffer_create_info.size =
      glm::max(static_cast<size_t>(1),
               sizeof(VkDrawIndexedIndirectCommand) * opaque_shadow_mesh_draw_indexed_indirect_commands.size());
  opaque_shadow_mesh_draw_indexed_indirect_commands_buffer =
      std::make_shared<Buffer>(buffer_create_info, buffer_vma_allocation_create_info);

  buffer_create_info.size =
      glm::max(static_cast<size_t>(1),
               sizeof(VkDrawMeshTasksIndirectCommandEXT) * opaque_shadow_mesh_draw_mesh_tasks_indirect_commands.size());
  opaque_shadow_mesh_draw_mesh_tasks_indirect_commands_buffer =
      std::make_shared<Buffer>(buffer_create_info, buffer_vma_allocation_create_info);

  deferred_render_instances = std::make_shared<MeshRenderInstanceCollection>();
  deferred_skinned_render_instances = std::make_shared<SkinnedMeshRenderInstanceCollection>();
  deferred_instanced_render_instances = std::make_shared<InstancedRenderInstanceCollection>();
  deferred_strands_render_instances = std::make_shared<StrandsRenderInstanceCollection>();

  forward_render_instances = std::make_shared<MeshRenderInstanceCollection>();
  forward_skinned_render_instances = std::make_shared<SkinnedMeshRenderInstanceCollection>();
  forward_instanced_render_instances = std::make_shared<InstancedRenderInstanceCollection>();
  forward_strands_render_instances = std::make_shared<StrandsRenderInstanceCollection>();

  transparent_render_instances = std::make_shared<MeshRenderInstanceCollection>();
  transparent_skinned_render_instances = std::make_shared<SkinnedMeshRenderInstanceCollection>();
  transparent_instanced_render_instances = std::make_shared<InstancedRenderInstanceCollection>();
  transparent_strands_render_instances = std::make_shared<StrandsRenderInstanceCollection>();

  gaussian_splat_render_instances = std::make_shared<GaussianSplatRenderInstanceCollection>();
  external_render_instances = std::make_shared<ExternalRenderInstanceCollection>();
}

void RenderInstanceStorage::Clear() {
  total_mesh_triangles = 0;
  total_opaque_shadow_mesh_triangles = 0;
  total_skinned_mesh_triangles = 0;
  total_instanced_mesh_triangles = 0;
  total_strands_segments = 0;
  total_strand_meshlets = 0;
  total_gaussian_splats = 0;

  deferred_render_instances = std::make_shared<MeshRenderInstanceCollection>();
  deferred_skinned_render_instances = std::make_shared<SkinnedMeshRenderInstanceCollection>();
  deferred_instanced_render_instances = std::make_shared<InstancedRenderInstanceCollection>();
  deferred_strands_render_instances = std::make_shared<StrandsRenderInstanceCollection>();

  forward_render_instances = std::make_shared<MeshRenderInstanceCollection>();
  forward_skinned_render_instances = std::make_shared<SkinnedMeshRenderInstanceCollection>();
  forward_instanced_render_instances = std::make_shared<InstancedRenderInstanceCollection>();
  forward_strands_render_instances = std::make_shared<StrandsRenderInstanceCollection>();

  transparent_render_instances = std::make_shared<MeshRenderInstanceCollection>();
  transparent_skinned_render_instances = std::make_shared<SkinnedMeshRenderInstanceCollection>();
  transparent_instanced_render_instances = std::make_shared<InstancedRenderInstanceCollection>();
  transparent_strands_render_instances = std::make_shared<StrandsRenderInstanceCollection>();

  gaussian_splat_render_instances = std::make_shared<GaussianSplatRenderInstanceCollection>();
  external_render_instances = std::make_shared<ExternalRenderInstanceCollection>();

  instance_entity_handles_.clear();
  instance_renderer_handles_.clear();

  renderer_indices_.clear();
  camera_indices_.clear();
  material_indices_.clear();
  render_settings = {};

  camera_info_blocks_.clear();
  gltf_material_cache_.Clear();
  raster_material_descriptor_sets.clear();
  raster_material_descriptor_texture_storage_version_ = UINT32_MAX;
  instance_info_blocks_.clear();
  previous_instance_info_blocks_.clear();
  rigid_motion_supported_.clear();
  directional_light_info_blocks_.clear();
  point_light_info_blocks_.clear();
  spot_light_info_blocks_.clear();
  render_info_block = {};

  cameras.clear();

  mesh_draw_indexed_indirect_commands.clear();
  mesh_draw_mesh_tasks_indirect_commands.clear();
  deferred_mesh_indirect_batches.clear();
  opaque_shadow_mesh_draw_indexed_indirect_commands.clear();
  opaque_shadow_mesh_draw_mesh_tasks_indirect_commands.clear();
}

void RenderInstanceStorage::Upload() {
  if (!Platform::Initialized())
    return;
  camera_info_descriptor_buffer->UploadVector(camera_info_blocks_);
  gltf_material_descriptor_buffer->UploadVector(gltf_material_cache_.GetShadeMaterials());
  gltf_texture_info_descriptor_buffer->UploadVector(gltf_material_cache_.GetTextureInfos());
  instance_info_descriptor_buffer->UploadVector(instance_info_blocks_);
  previous_instance_info_descriptor_buffer->UploadVector(previous_instance_info_blocks_);
  if (emissive_triangle_info_dirty_) {
    emissive_triangle_info_descriptor_buffer->UploadVector(emissive_triangle_info_blocks_);
    emissive_triangle_info_dirty_ = false;
  }
  render_info_descriptor_buffer->Upload(render_info_block);
  directional_light_info_descriptor_buffer->UploadVector(directional_light_info_blocks_);
  point_light_info_descriptor_buffer->UploadVector(point_light_info_blocks_);
  spot_light_info_descriptor_buffer->UploadVector(spot_light_info_blocks_);

  mesh_draw_indexed_indirect_commands_buffer->UploadVector(mesh_draw_indexed_indirect_commands);
  mesh_draw_mesh_tasks_indirect_commands_buffer->UploadVector(mesh_draw_mesh_tasks_indirect_commands);
  opaque_shadow_mesh_draw_indexed_indirect_commands_buffer->UploadVector(
      opaque_shadow_mesh_draw_indexed_indirect_commands);
  opaque_shadow_mesh_draw_mesh_tasks_indirect_commands_buffer->UploadVector(
      opaque_shadow_mesh_draw_mesh_tasks_indirect_commands);

  environment_info_descriptor_buffer->Upload(environment_info_block);
}

const std::vector<GltfShadeMaterial>& RenderInstanceStorage::GetGltfShadeMaterials() const {
  return gltf_material_cache_.GetShadeMaterials();
}

const std::vector<GltfTextureInfo>& RenderInstanceStorage::GetGltfTextureInfos() const {
  return gltf_material_cache_.GetTextureInfos();
}

uint64_t RenderInstanceStorage::GetDdgiEmissiveInventorySignature() const {
  return ddgi_emissive_inventory_signature_;
}

const RenderInstanceStorage::EmissiveTriangleInventoryStats& RenderInstanceStorage::GetDdgiEmissiveInventoryStats()
    const {
  return ddgi_emissive_inventory_stats_;
}

const std::vector<RenderInstanceStorage::DdgiEmissiveGuideInfoBlock>&
RenderInstanceStorage::GetDdgiEmissiveGuideInfoBlocks() const {
  return ddgi_emissive_guide_info_blocks_;
}

const std::vector<RenderInstanceStorage::InstanceInfoBlock>& RenderInstanceStorage::GetInstanceInfoBlocks() const {
  return instance_info_blocks_;
}

const std::vector<RenderInstanceStorage::PreviousInstanceInfoBlock>&
RenderInstanceStorage::GetPreviousInstanceInfoBlocks() const {
  return previous_instance_info_blocks_;
}

void RenderInstanceStorage::BuildPreviousInstanceInfoBlocks(
    const std::shared_ptr<RenderInstanceStorage>& previous_render_instances) {
  previous_instance_info_blocks_.resize(instance_info_blocks_.size());
  std::unordered_map<uint32_t, size_t> previous_entity_indices;
  if (previous_render_instances) {
    const auto& previous_blocks = previous_render_instances->instance_info_blocks_;
    previous_entity_indices.reserve(previous_blocks.size());
    for (size_t index = 0; index < previous_blocks.size(); ++index) {
      const auto entity_index = previous_blocks[index].entity_index;
      if (entity_index != 0u) {
        previous_entity_indices.try_emplace(entity_index, index);
      }
    }
  }
  for (size_t index = 0; index < instance_info_blocks_.size(); ++index) {
    const auto& current = instance_info_blocks_[index];
    auto& previous = previous_instance_info_blocks_[index];
    previous.previous_model = current.model.value;
    previous.flags = {};
    if (!previous_render_instances) {
      continue;
    }
    int previous_index = -1;
    if (current.renderer_handle != 0) {
      const auto renderer_search = previous_render_instances->renderer_indices_.find(current.renderer_handle);
      if (renderer_search != previous_render_instances->renderer_indices_.end()) {
        previous_index = renderer_search->second;
      }
    }
    if (previous_index < 0 && current.entity_index != 0u) {
      const auto entity_search = previous_entity_indices.find(current.entity_index);
      if (entity_search != previous_entity_indices.end()) {
        previous_index = static_cast<int>(entity_search->second);
      }
    }
    if (previous_index < 0 ||
        static_cast<size_t>(previous_index) >= previous_render_instances->instance_info_blocks_.size()) {
      continue;
    }
    previous.previous_model = previous_render_instances->instance_info_blocks_[previous_index].model.value;
    if (index < rigid_motion_supported_.size() && rigid_motion_supported_[index] != 0u) {
      previous.flags.x = 1u;
    }
  }

  std::unordered_map<Handle, std::shared_ptr<SkinnedMeshRenderInstance>> previous_by_renderer;
  std::unordered_map<Handle, std::shared_ptr<SkinnedMeshRenderInstance>> previous_by_entity;
  const auto collect_previous = [&](const std::shared_ptr<SkinnedMeshRenderInstanceCollection>& collection) {
    if (!collection) {
      return;
    }
    collection->ForEachSkinnedMeshRenderInstance([&](const auto& render_instance) {
      if (render_instance->renderer_handle != 0) {
        previous_by_renderer.try_emplace(render_instance->renderer_handle, render_instance);
      }
      if (render_instance->entity_handle != 0) {
        previous_by_entity.try_emplace(render_instance->entity_handle, render_instance);
      }
    });
  };
  if (previous_render_instances) {
    collect_previous(previous_render_instances->deferred_skinned_render_instances);
    collect_previous(previous_render_instances->forward_skinned_render_instances);
    collect_previous(previous_render_instances->transparent_skinned_render_instances);
  }

  const auto build_previous_pose = [&](const std::shared_ptr<SkinnedMeshRenderInstanceCollection>& collection) {
    if (!collection) {
      return;
    }
    collection->ForEachSkinnedMeshRenderInstance([&](const auto& render_instance) {
      std::shared_ptr<SkinnedMeshRenderInstance> previous_render_instance;
      if (render_instance->renderer_handle != 0) {
        const auto search = previous_by_renderer.find(render_instance->renderer_handle);
        if (search != previous_by_renderer.end()) {
          previous_render_instance = search->second;
        }
      }
      if (!previous_render_instance && render_instance->entity_handle != 0) {
        const auto search = previous_by_entity.find(render_instance->entity_handle);
        if (search != previous_by_entity.end()) {
          previous_render_instance = search->second;
        }
      }
      const bool previous_pose_valid =
          previous_render_instance && render_instance->instance_index >= 0 &&
          static_cast<size_t>(render_instance->instance_index) < previous_instance_info_blocks_.size() &&
          previous_render_instance->geometry_version == render_instance->geometry_version &&
          !previous_render_instance->bone_matrices_snapshot.empty() &&
          previous_render_instance->bone_matrices_snapshot.size() == render_instance->bone_matrices_snapshot.size();
      if (!render_instance->bone_matrices || render_instance->bone_matrices_snapshot.empty()) {
        return;
      }
      render_instance->bone_matrices->UploadPreviousData(previous_pose_valid
                                                             ? previous_render_instance->bone_matrices_snapshot
                                                             : render_instance->bone_matrices_snapshot);
      if (previous_pose_valid) {
        previous_instance_info_blocks_[render_instance->instance_index].flags.y = 1u;
      }
    });
  };
  build_previous_pose(deferred_skinned_render_instances);
  build_previous_pose(forward_skinned_render_instances);
  build_previous_pose(transparent_skinned_render_instances);
}

bool RenderInstanceStorage::RequiresCameraWideTemporalHistoryRejection() const {
  return (forward_render_instances && !forward_render_instances->Empty()) ||
         (forward_skinned_render_instances && !forward_skinned_render_instances->Empty()) ||
         (forward_instanced_render_instances && !forward_instanced_render_instances->Empty()) ||
         (forward_strands_render_instances && !forward_strands_render_instances->Empty()) ||
         (transparent_skinned_render_instances && !transparent_skinned_render_instances->Empty()) ||
         (transparent_instanced_render_instances && !transparent_instanced_render_instances->Empty()) ||
         (transparent_strands_render_instances && !transparent_strands_render_instances->Empty()) ||
         (gaussian_splat_render_instances && !gaussian_splat_render_instances->Empty()) ||
         (external_render_instances && !external_render_instances->Empty());
}

void RenderInstanceStorage::RefreshRasterMaterialDescriptorSets(
    const std::shared_ptr<DescriptorSetLayout>& raster_material_layout,
    const std::array<VkDescriptorImageInfo, kRasterMaterialTextureSlotCount>& fallback_image_infos) {
  if (!Platform::Initialized() || !raster_material_layout) {
    return;
  }
  const auto& shade_materials = gltf_material_cache_.GetShadeMaterials();
  const auto current_texture_storage_version = TextureStorage::GetVersion();
  if (raster_material_descriptor_sets.size() == shade_materials.size() &&
      raster_material_descriptor_texture_storage_version_ == current_texture_storage_version) {
    return;
  }

  const auto& texture_infos = gltf_material_cache_.GetTextureInfos();
  raster_material_descriptor_sets.resize(shade_materials.size());
  const auto resolve_image_info = [&](const uint16_t texture_info_slot,
                                      const uint32_t fallback_binding) -> VkDescriptorImageInfo {
    auto image_info = fallback_image_infos[fallback_binding];
    if (texture_info_slot >= texture_infos.size()) {
      return image_info;
    }
    const auto texture_index = texture_infos[texture_info_slot].index;
    if (texture_index >= 0) {
      TextureStorage::TryGetTexture2DDescriptorImageInfo(static_cast<uint32_t>(texture_index), image_info);
    }
    return image_info;
  };

  for (uint32_t material_index = 0; material_index < shade_materials.size(); material_index++) {
    auto& descriptor_set = raster_material_descriptor_sets[material_index];
    if (!descriptor_set) {
      descriptor_set = std::make_shared<DescriptorSet>(raster_material_layout);
    }
    const auto& material = shade_materials[material_index];
#if MAT_EXT_SPECULAR_GLOSSINESS
    const bool specular_glossiness = material.pbr_model == static_cast<int32_t>(GltfPbrModel::SpecularGlossiness);
    const auto base_color_texture =
        specular_glossiness ? material.pbr_diffuse_texture : material.pbr_base_color_texture;
    const auto metallic_roughness_texture =
        specular_glossiness ? material.pbr_specular_glossiness_texture : material.pbr_metallic_roughness_texture;
#else
    const auto base_color_texture = material.pbr_base_color_texture;
    const auto metallic_roughness_texture = material.pbr_metallic_roughness_texture;
#endif
    descriptor_set->UpdateImageDescriptorBinding(0, resolve_image_info(base_color_texture, 0));
    descriptor_set->UpdateImageDescriptorBinding(1, resolve_image_info(metallic_roughness_texture, 1));
    descriptor_set->UpdateImageDescriptorBinding(2, resolve_image_info(material.normal_texture, 2));
    descriptor_set->UpdateImageDescriptorBinding(3, resolve_image_info(material.emissive_texture, 3));
    descriptor_set->UpdateImageDescriptorBinding(4, resolve_image_info(material.occlusion_texture, 4));
#if MAT_EXT_CLEARCOAT
    descriptor_set->UpdateImageDescriptorBinding(5, resolve_image_info(material.clearcoat_texture, 5));
    descriptor_set->UpdateImageDescriptorBinding(6, resolve_image_info(material.clearcoat_roughness_texture, 6));
    descriptor_set->UpdateImageDescriptorBinding(7, resolve_image_info(material.clearcoat_normal_texture, 7));
#else
    descriptor_set->UpdateImageDescriptorBinding(5, fallback_image_infos[5]);
    descriptor_set->UpdateImageDescriptorBinding(6, fallback_image_infos[6]);
    descriptor_set->UpdateImageDescriptorBinding(7, fallback_image_infos[7]);
#endif
  }
  raster_material_descriptor_texture_storage_version_ = current_texture_storage_version;
}

const std::shared_ptr<DescriptorSet>& RenderInstanceStorage::GetRasterMaterialDescriptorSet(
    const uint32_t material_index) const {
  if (material_index >= raster_material_descriptor_sets.size()) {
    throw std::runtime_error("Unable to find raster material descriptor set.");
  }
  return raster_material_descriptor_sets[material_index];
}

void RenderInstanceStorage::CalculateLodFactor(const std::shared_ptr<Scene>& scene, const glm::vec3& view_position,
                                               const float max_distance) {
  if (const auto* owners = scene->UnsafeGetPrivateComponentOwnersList<LodGroup>()) {
    for (auto owner : *owners) {
      if (const auto lod_group = scene->GetOrSetPrivateComponent<LodGroup>(owner).lock();
          !lod_group->override_lod_factor) {
        auto gt = scene->GetDataComponent<GlobalTransform>(owner);
        const auto distance = glm::distance(gt.GetPosition(), view_position);
        const auto distance_factor = glm::clamp(distance / max_distance, 0.f, 1.f);
        lod_group->lod_factor = glm::clamp(distance_factor * distance_factor, 0.f, 1.f);
      }
    }
  }
}
bool RenderInstanceStorage::operator!=(const RenderInstanceStorage& other) const {
  if (render_info_block != other.render_info_block)
    return true;

  if (environment_info_block != other.environment_info_block)
    return true;

  if (instance_info_blocks_.size() != other.instance_info_blocks_.size())
    return true;
  for (uint32_t i = 0; i < instance_info_blocks_.size(); i++) {
    if (instance_info_blocks_[i] != other.instance_info_blocks_[i])
      return true;
  }

  const auto directional_light_count = static_cast<size_t>(render_info_block.directional_light_size);
  const auto comparable_directional_light_count = std::min(
      {directional_light_count, directional_light_info_blocks_.size(), other.directional_light_info_blocks_.size()});
  for (size_t i = 0; i < comparable_directional_light_count; ++i) {
    const auto& current = directional_light_info_blocks_[i];
    const auto& previous = other.directional_light_info_blocks_[i];
    if (current.HasSceneLightingDifference(previous)) {
      return true;
    }
  }

  if (point_light_info_blocks_.size() != other.point_light_info_blocks_.size())
    return true;
  for (uint32_t i = 0; i < point_light_info_blocks_.size(); i++) {
    if (point_light_info_blocks_[i] != other.point_light_info_blocks_[i])
      return true;
  }

  if (spot_light_info_blocks_.size() != other.spot_light_info_blocks_.size())
    return true;
  for (uint32_t i = 0; i < spot_light_info_blocks_.size(); i++) {
    if (spot_light_info_blocks_[i] != other.spot_light_info_blocks_[i])
      return true;
  }

  if (GetGltfShadeMaterials() != other.GetGltfShadeMaterials())
    return true;
  if (GetGltfTextureInfos() != other.GetGltfTextureInfos())
    return true;

  if (emissive_triangle_instance_signatures_ != other.emissive_triangle_instance_signatures_)
    return true;

  if (*gaussian_splat_render_instances != *other.gaussian_splat_render_instances)
    return true;

  if (*external_render_instances != *other.external_render_instances)
    return true;

  if (geometry_storage_version != other.geometry_storage_version)
    return true;
  if (texture_storage_version != other.texture_storage_version)
    return true;

  return false;
}

bool RenderInstanceStorage::RegisterMeshDrawCommand(const std::shared_ptr<Mesh>& mesh,
                                                    const std::shared_ptr<Material>& material,
                                                    const GlobalTransform& model, bool cast_shadow) {
  if (!material || !mesh || !mesh->meshlet_range_ || !mesh->triangle_range_)
    return false;
  if (mesh->UnsafeGetVertices().empty() || mesh->UnsafeGetTriangles().empty())
    return false;
  if (mesh->triangle_range_->prev_frame_index_count == 0 || mesh->meshlet_range_->prev_frame_range == 0)
    return false;
  auto mesh_bound = mesh->GetBound();
  mesh_bound.ApplyTransform(model.value);
  const auto material_data = BuildMaterialGltfData(*material);
  const auto render_instance = std::make_shared<MeshRenderInstance>();
  render_instance->command_type = RenderInstanceType::FromApi;
  render_instance->owner = Entity();
  render_instance->mesh = mesh;
  render_instance->entity_handle = 0;
  render_instance->renderer_handle = 0;
  render_instance->material = material;
  render_instance->model = model;
  render_instance->renderer_handle = 0;
  render_instance->cast_shadow = cast_shadow;
  render_instance->world_bound = mesh_bound;
  render_instance->geometry_version = mesh->GetVersion();
  render_instance->material_version = material->GetVersion();
  render_instance->material_index = RegisterMaterial(material, material_data);
  render_instance->line_width = material->draw_settings.line_width;
  render_instance->cull_mode = ResolveCullModeForTransform(material->draw_settings.cull_mode, model.value);
  render_instance->polygon_mode = material->draw_settings.polygon_mode;
  render_instance->entity_selected = false;
  if (UsesTransparentRasterPass(*material, material_data.shade_material)) {
    transparent_render_instances->Register(render_instance);
  } else {
    deferred_render_instances->Register(render_instance);
    AppendMeshIndirectCommands(mesh_draw_indexed_indirect_commands, mesh_draw_mesh_tasks_indirect_commands,
                               mesh->triangle_range_->prev_frame_offset, mesh->triangle_range_->prev_frame_index_count,
                               mesh->meshlet_range_->prev_frame_range);
    total_mesh_triangles += mesh->triangle_range_->prev_frame_index_count;
  }

  return true;
}

bool RenderInstanceStorage::RegisterMeshDrawInstancedCommand(
    const std::shared_ptr<Mesh>& mesh, const std::shared_ptr<Material>& material, const GlobalTransform& model,
    const std::shared_ptr<ParticleInfoList>& particle_info_list, const bool cast_shadow) {
  if (!material || !mesh || !mesh->meshlet_range_ || !mesh->triangle_range_)
    return false;
  if (mesh->UnsafeGetVertices().empty() || mesh->UnsafeGetTriangles().empty())
    return false;
  if (mesh->triangle_range_->prev_frame_index_count == 0 || mesh->meshlet_range_->prev_frame_range == 0)
    return false;
  const auto material_data = BuildMaterialGltfData(*material);
  const auto render_instance = std::make_shared<InstancedRenderInstance>();
  render_instance->command_type = RenderInstanceType::FromApi;
  render_instance->owner = Entity();
  render_instance->mesh = mesh;
  render_instance->entity_handle = 0;
  render_instance->renderer_handle = 0;
  render_instance->material = material;
  render_instance->model = model;
  render_instance->particle_infos = particle_info_list;
  render_instance->particle_info_list_version = particle_info_list->GetVersion();
  render_instance->renderer_handle = 0;
  render_instance->cast_shadow = cast_shadow;
  render_instance->geometry_version = mesh->GetVersion();
  render_instance->material_version = material->GetVersion();
  render_instance->material_index = RegisterMaterial(material, material_data);
  render_instance->line_width = material->draw_settings.line_width;
  render_instance->cull_mode = ResolveInstancedCullModeForTransforms(material->draw_settings.cull_mode, model.value,
                                                                     particle_info_list->PeekParticleInfoList());
  render_instance->polygon_mode = material->draw_settings.polygon_mode;
  render_instance->entity_selected = false;
  if (UsesTransparentRasterPass(*material, material_data.shade_material)) {
    transparent_instanced_render_instances->Register(render_instance);
  } else {
    deferred_instanced_render_instances->Register(render_instance);
  }

  total_mesh_triangles +=
      mesh->triangle_range_->prev_frame_index_count * particle_info_list->PeekParticleInfoList().size();

  return true;
}

bool RenderInstanceStorage::RegisterRenderInstance(const std::shared_ptr<Scene>& target_scene, const Entity& entity,
                                                   const Handle& renderer_handle,
                                                   const std::shared_ptr<Material>& material, int* out_material_index) {
  return RegisterRenderInstance(target_scene, entity, renderer_handle, material, {}, out_material_index);
}

bool RenderInstanceStorage::RegisterRenderInstance(const std::shared_ptr<Scene>& target_scene, const Entity& entity,
                                                   const Handle& renderer_handle,
                                                   const std::shared_ptr<Material>& material,
                                                   const DdgiExternalGeometry& ddgi_geometry, int* out_material_index) {
  if (!material)
    return false;
  const auto gt = target_scene->GetDataComponent<GlobalTransform>(entity);
  const auto material_data = BuildMaterialGltfData(*material);
  const auto render_instance = std::make_shared<ExternalRenderInstance>();
  render_instance->command_type = RenderInstanceType::Unknown;
  render_instance->owner = entity;
  render_instance->model = gt;
  render_instance->entity_handle = target_scene->GetEntityHandle(entity);
  render_instance->renderer_handle = renderer_handle;
  render_instance->material = material;
  render_instance->cast_shadow = false;
  render_instance->ddgi_geometry = ddgi_geometry;
  render_instance->geometry_version = ddgi_geometry.IsValid() ? ddgi_geometry.geometry_version : 0;
  render_instance->material_version = material->GetVersion();
  render_instance->material_index = RegisterMaterial(material, material_data);
  render_instance->entity_selected = target_scene->IsEntityAncestorSelected(entity);
  render_instance->line_width = material->draw_settings.line_width;
  render_instance->cull_mode = ResolveCullModeForTransform(material->draw_settings.cull_mode, gt.value);
  render_instance->polygon_mode = material->draw_settings.polygon_mode;

  if (out_material_index) {
    *out_material_index = render_instance->material_index;
  }
  external_render_instances->Register(render_instance);

  return true;
}

int RenderInstanceStorage::RegisterMaterial(const std::shared_ptr<Material>& material) {
  if (!material)
    return -1;
  return RegisterMaterial(material, BuildMaterialGltfData(*material));
}

void RenderInstanceStorage::BuildFromScene(
    const RenderSettings& render_settings, const std::shared_ptr<Scene>& scene, Bound& world_bound,
    const bool include_editor_cameras,
    const std::vector<std::pair<GlobalTransform, std::shared_ptr<Camera>>>* injected_cameras,
    const bool include_reflection_probes,
    const std::unordered_map<uint64_t, ReflectionProbeTextureOverride>* reflection_probe_texture_overrides) {
  this->render_settings = render_settings;
  render_info_block.Apply(this->render_settings);
  CollectEnvironment(scene);
  if (include_reflection_probes) {
    CollectReflectionProbes(scene, reflection_probe_texture_overrides);
  } else {
    render_info_block.reflection_probe_header = glm::uvec4(0u);
    render_info_block.reflection_probes = {};
  }
  if (include_editor_cameras) {
    CollectEditorCameras(scene, cameras);
  }
  CollectCameras(scene, cameras);
  if (injected_cameras) {
    for (const auto& injected_camera : *injected_cameras) {
      if (injected_camera.second) {
        cameras.emplace_back(injected_camera);
      }
    }
  }
  for (const auto& camera_info : cameras) {
    CameraInfoBlock camera_info_block;
    camera_info.second->UpdateCameraInfoBlock(camera_info_block, camera_info.first);
    camera_info_block.shadow_split_distances =
        render_settings.GetShadowCascadeSplitDistances(camera_info.second->camera_settings.near_distance);
    const auto index = RegisterCamera(camera_info.second->GetHandle(), camera_info_block);
  }
  CollectEntityRenderers(scene, world_bound);
  BuildRenderInstanceBlocks();
  BuildEmissiveTriangleInfoBlocks();
  CollectLights(scene, world_bound);
}

void RenderInstanceStorage::CollectReflectionProbes(
    const std::shared_ptr<Scene>& target_scene,
    const std::unordered_map<uint64_t, ReflectionProbeTextureOverride>* texture_overrides) {
  render_info_block.reflection_probe_header = glm::uvec4(0u);
  render_info_block.reflection_probes = {};
  if (!target_scene) {
    return;
  }
  const auto resolved_lighting = ResolveEnvironmentalLighting(target_scene);
  render_info_block.reflection_probe_header.x = static_cast<uint32_t>(
      std::min(resolved_lighting.local_reflection_probes.size(), static_cast<size_t>(kReflectionProbeMaxCount)));
  for (size_t index = 0; index < render_info_block.reflection_probe_header.x; ++index) {
    const auto& probe = resolved_lighting.local_reflection_probes[index];
    auto& info = render_info_block.reflection_probes[index];
    info.world_to_probe = glm::inverse(probe.transform);
    info.shape_parameters = glm::vec4(glm::vec3(0.5f), probe.sphere_radius);
    info.projection_parameters = glm::vec4(probe.box_projection_extents, probe.blend_distance);
    info.lighting_parameters = glm::vec4(probe.reflection_intensity, static_cast<float>(probe.artist_priority),
                                         static_cast<float>(probe.shape), probe.box_projection ? 1.0f : 0.0f);
    info.identity_and_flags.z = static_cast<uint32_t>(probe.stable_id);
    info.identity_and_flags.w = static_cast<uint32_t>(probe.stable_id >> 32u);
    VkDescriptorImageInfo descriptor_info{};
    if (texture_overrides) {
      if (const auto found = texture_overrides->find(probe.stable_id); found != texture_overrides->end()) {
        const auto& override = found->second;
        VkDescriptorImageInfo target_descriptor{};
        if (override.target_valid &&
            TextureStorage::TryGetCubemapDescriptorImageInfo(override.target_texture_index, target_descriptor)) {
          if (override.source_valid &&
              TextureStorage::TryGetCubemapDescriptorImageInfo(override.source_texture_index, descriptor_info)) {
            info.identity_and_flags.x = override.source_texture_index;
            info.identity_and_flags.y = 1u;
          }
          info.transition_parameters.x = override.target_texture_index;
          info.transition_parameters.y = 1u;
          info.transition_parameters.z = glm::floatBitsToUint(glm::clamp(override.blend_weight, 0.0f, 1.0f));
          continue;
        }
      }
    }
    auto probe_payload_ref = probe.global_reflection_probe;
    if (const auto asset = probe_payload_ref.Get<GlobalReflectionProbe>(); asset && asset->IsRuntimeReady()) {
      if (const auto cubemap = asset->GetCubemap();
          cubemap &&
          TextureStorage::TryGetCubemapDescriptorImageInfo(cubemap->GetTextureStorageIndex(), descriptor_info)) {
        info.identity_and_flags.x = cubemap->GetTextureStorageIndex();
        info.identity_and_flags.y = 1u;
      }
    }
  }
}

uint32_t RenderInstanceStorage::GetReflectionProbeCount() const {
  return render_info_block.reflection_probe_header.x;
}

const std::array<RenderInstanceStorage::ReflectionProbeInfoBlock, RenderInstanceStorage::kReflectionProbeMaxCount>&
RenderInstanceStorage::GetReflectionProbeInfoBlocks() const {
  return render_info_block.reflection_probes;
}

void RenderInstanceStorage::UpdateTopLevelAccelerationStructure() {
  if (!mesh_top_level_acceleration_structure) {
    mesh_top_level_acceleration_structure = std::make_shared<TopLevelAccelerationStructure>();
  }
  mesh_top_level_acceleration_structure->Update(*this);
}

bool RenderInstanceStorage::RegisterEntity(const std::shared_ptr<Scene>& target_scene, const Entity& owner,
                                           const std::shared_ptr<StrandsRenderer>& strands_renderer,
                                           glm::vec3& min_bound, glm::vec3& max_bound) {
  auto material = strands_renderer->material.Get<Material>();
  auto strands = strands_renderer->strands.Get<Strands>();
  if (!strands_renderer->IsEnabled() || !material || !strands) {
    return false;
  }
  auto gt = target_scene->GetDataComponent<GlobalTransform>(owner);
  auto ltw = gt.value;
  auto mesh_bound = strands->bound_;
  mesh_bound.ApplyTransform(ltw);
  glm::vec3 center = mesh_bound.Center();

  glm::vec3 size = mesh_bound.Size();
  min_bound = glm::vec3((glm::min)(min_bound.x, center.x - size.x), (glm::min)(min_bound.y, center.y - size.y),
                        (glm::min)(min_bound.z, center.z - size.z));
  max_bound = glm::vec3(glm::max(max_bound.x, center.x + size.x), glm::max(max_bound.y, center.y + size.y),
                        glm::max(max_bound.z, center.z + size.z));

  if (!Platform::MeshShaderEnabled() || !strands->strand_meshlet_range_ || !strands->segment_range_ ||
      strands->segment_range_->prev_frame_index_count == 0 || strands->strand_meshlet_range_->prev_frame_range == 0) {
    return false;
  }

  const auto material_data = BuildMaterialGltfData(*material);

  const auto render_instance = std::make_shared<StrandsRenderInstance>();
  render_instance->command_type = RenderInstanceType::FromRenderer;
  render_instance->owner = owner;
  render_instance->entity_handle = target_scene->GetEntityHandle(owner);
  render_instance->renderer_handle = strands_renderer->GetHandle();
  render_instance->model = gt;
  render_instance->strands = strands;
  render_instance->material = material;
  render_instance->cast_shadow = strands_renderer->cast_shadow;
  render_instance->world_bound = mesh_bound;
  render_instance->geometry_version = strands->GetVersion();
  render_instance->material_version = material->GetVersion();
  render_instance->material_index = RegisterMaterial(material, material_data);
  render_instance->entity_selected = target_scene->IsEntityAncestorSelected(owner);
  render_instance->line_width = material->draw_settings.line_width;
  render_instance->cull_mode = ResolveCullModeForTransform(material->draw_settings.cull_mode, gt.value);
  render_instance->polygon_mode = material->draw_settings.polygon_mode;

  if (UsesTransparentRasterPass(*material, material_data.shade_material)) {
    transparent_strands_render_instances->Register(render_instance);
  } else {
    deferred_strands_render_instances->Register(render_instance);
  }

  total_strands_segments += strands->segment_range_->prev_frame_index_count;
  total_strand_meshlets += strands->strand_meshlet_range_->prev_frame_range;
  return true;
}

bool RenderInstanceStorage::RegisterEntity(const std::shared_ptr<Scene>& target_scene, const Entity& owner,
                                           const std::shared_ptr<GaussianSplatRenderer>& gaussian_splat_renderer,
                                           glm::vec3& min_bound, glm::vec3& max_bound) {
  auto gaussian_splat = gaussian_splat_renderer->gaussian_splat.Get<GaussianSplat>();
  if (!gaussian_splat_renderer->IsEnabled() || !gaussian_splat || gaussian_splat->Empty())
    return false;

  auto gt = target_scene->GetDataComponent<GlobalTransform>(owner);
  auto mesh_bound = Bound();
  mesh_bound.min = gaussian_splat->GetMinBound();
  mesh_bound.max = gaussian_splat->GetMaxBound();
  mesh_bound.ApplyTransform(gt.value);
  glm::vec3 center = mesh_bound.Center();

  glm::vec3 size = mesh_bound.Size();
  min_bound = glm::vec3((glm::min)(min_bound.x, center.x - size.x), (glm::min)(min_bound.y, center.y - size.y),
                        (glm::min)(min_bound.z, center.z - size.z));
  max_bound = glm::vec3(glm::max(max_bound.x, center.x + size.x), glm::max(max_bound.y, center.y + size.y),
                        glm::max(max_bound.z, center.z + size.z));

  const auto render_instance = std::make_shared<GaussianSplatRenderInstance>();
  (void)gaussian_splat->EnsureGpuData();
  render_instance->command_type = RenderInstanceType::FromRenderer;
  render_instance->owner = owner;
  render_instance->entity_handle = target_scene->GetEntityHandle(owner);
  render_instance->renderer_handle = gaussian_splat_renderer->GetHandle();
  render_instance->model = gt;
  render_instance->gaussian_splat = gaussian_splat;
  render_instance->world_bound = mesh_bound;
  render_instance->geometry_version = gaussian_splat->GetGpuDataRevision();
  render_instance->entity_selected = target_scene->IsEntityAncestorSelected(owner);
  render_instance->opacity_scale = gaussian_splat_renderer->opacity_scale;
  render_instance->sh_degree = gaussian_splat_renderer->sh_degree;
  render_instance->sort_mode = gaussian_splat_renderer->sort_mode;
  render_instance->depth_mode = gaussian_splat_renderer->depth_mode;
  render_instance->raster_mode = gaussian_splat_renderer->raster_mode;

  gaussian_splat_render_instances->Register(render_instance);
  total_gaussian_splats += gaussian_splat->GetSplatCount();
  return true;
}

bool RenderInstanceStorage::RegisterEntity(const std::shared_ptr<Scene>& target_scene, const Entity& owner,
                                           const std::shared_ptr<MeshRenderer>& mesh_renderer, glm::vec3& min_bound,
                                           glm::vec3& max_bound) {
  auto material = mesh_renderer->material.Get<Material>();
  auto mesh = mesh_renderer->mesh.Get<Mesh>();
  if (!mesh_renderer->IsEnabled() || !material || !mesh || !mesh->meshlet_range_ || !mesh->triangle_range_)
    return false;
  if (mesh->UnsafeGetVertices().empty() || mesh->UnsafeGetTriangles().empty())
    return false;
  if (mesh->triangle_range_->prev_frame_index_count == 0 || mesh->meshlet_range_->prev_frame_range == 0)
    return false;

  auto gt = target_scene->GetDataComponent<GlobalTransform>(owner);
  auto ltw = gt.value;
  auto mesh_bound = mesh->GetBound();
  if (mesh_renderer->ray_tracing_blas_) {
    mesh_bound.min = glm::min(mesh_bound.min, mesh_renderer->ray_tracing_bound_.min);
    mesh_bound.max = glm::max(mesh_bound.max, mesh_renderer->ray_tracing_bound_.max);
  }
  mesh_bound.ApplyTransform(ltw);
  glm::vec3 center = mesh_bound.Center();

  glm::vec3 size = mesh_bound.Size();
  min_bound = glm::vec3((glm::min)(min_bound.x, center.x - size.x), (glm::min)(min_bound.y, center.y - size.y),
                        (glm::min)(min_bound.z, center.z - size.z));
  max_bound = glm::vec3(glm::max(max_bound.x, center.x + size.x), glm::max(max_bound.y, center.y + size.y),
                        glm::max(max_bound.z, center.z + size.z));

  const auto material_data = BuildMaterialGltfData(*material);
  const auto render_instance = std::make_shared<MeshRenderInstance>();
  render_instance->command_type = RenderInstanceType::FromRenderer;
  render_instance->owner = owner;
  render_instance->mesh = mesh;
  render_instance->material = material;
  render_instance->model = gt;
  render_instance->entity_handle = target_scene->GetEntityHandle(owner);
  render_instance->renderer_handle = mesh_renderer->GetHandle();
  render_instance->cast_shadow = mesh_renderer->cast_shadow;
  render_instance->world_bound = mesh_bound;
  render_instance->geometry_version = mesh->GetVersion();
  render_instance->ray_tracing_geometry_version = mesh_renderer->ray_tracing_geometry_version_;
  render_instance->morph_weights_version = mesh_renderer->morph_weights_version_;
  render_instance->ray_tracing_triangle_range = mesh_renderer->ray_tracing_triangle_range_;
  render_instance->ray_tracing_blas = mesh_renderer->ray_tracing_blas_;
  render_instance->material_version = material->GetVersion();
  render_instance->material_index = RegisterMaterial(material, material_data);
  render_instance->line_width = material->draw_settings.line_width;
  render_instance->cull_mode = ResolveCullModeForTransform(material->draw_settings.cull_mode, gt.value);
  render_instance->polygon_mode = material->draw_settings.polygon_mode;
  render_instance->entity_selected = target_scene->IsEntityAncestorSelected(owner);
  if (UsesTransparentRasterPass(*material, material_data.shade_material)) {
    transparent_render_instances->Register(render_instance);
  } else {
    deferred_render_instances->Register(render_instance);
    if (ApplicationContext::Get().GetLayer<RenderLayer>()) {
      AppendMeshIndirectCommands(mesh_draw_indexed_indirect_commands, mesh_draw_mesh_tasks_indirect_commands,
                                 mesh->triangle_range_->prev_frame_offset,
                                 mesh->triangle_range_->prev_frame_index_count, mesh->meshlet_range_->prev_frame_range);
    }
    total_mesh_triangles += mesh->triangle_range_->prev_frame_index_count;
  }
  return true;
}

bool RenderInstanceStorage::RegisterEntity(const std::shared_ptr<Scene>& target_scene, const Entity& owner,
                                           const std::shared_ptr<SkinnedMeshRenderer>& skinned_mesh_renderer,
                                           glm::vec3& min_bound, glm::vec3& max_bound) {
  auto material = skinned_mesh_renderer->material.Get<Material>();
  auto skinned_mesh = skinned_mesh_renderer->skinned_mesh.Get<SkinnedMesh>();
  if (!skinned_mesh_renderer->IsEnabled() || !material || !skinned_mesh || !skinned_mesh->skinned_meshlet_range_ ||
      !skinned_mesh->skinned_triangle_range_)
    return false;
  if (skinned_mesh->skinned_vertices_.empty() || skinned_mesh->skinned_triangles_.empty())
    return false;
  if (skinned_mesh->skinned_triangle_range_->prev_frame_index_count == 0 ||
      skinned_mesh->skinned_meshlet_range_->prev_frame_range == 0)
    return false;
  GlobalTransform gt;
  if (auto animator = skinned_mesh_renderer->animator.Get<Animator>(); !animator) {
    return false;
  }
  if (!skinned_mesh_renderer->rag_doll_) {
    gt = target_scene->GetDataComponent<GlobalTransform>(owner);
  }
  auto ltw = gt.value;
  auto mesh_bound = skinned_mesh->GetBound();
  if (skinned_mesh_renderer->ray_tracing_blas_) {
    mesh_bound.min = glm::min(mesh_bound.min, skinned_mesh_renderer->ray_tracing_bound_.min);
    mesh_bound.max = glm::max(mesh_bound.max, skinned_mesh_renderer->ray_tracing_bound_.max);
  }
  mesh_bound.ApplyTransform(ltw);
  glm::vec3 center = mesh_bound.Center();

  glm::vec3 size = mesh_bound.Size();
  min_bound = glm::vec3((glm::min)(min_bound.x, center.x - size.x), (glm::min)(min_bound.y, center.y - size.y),
                        (glm::min)(min_bound.z, center.z - size.z));
  max_bound = glm::vec3(glm::max(max_bound.x, center.x + size.x), glm::max(max_bound.y, center.y + size.y),
                        glm::max(max_bound.z, center.z + size.z));

  const auto material_data = BuildMaterialGltfData(*material);
  const auto render_instance = std::make_shared<SkinnedMeshRenderInstance>();
  render_instance->command_type = RenderInstanceType::FromRenderer;
  render_instance->owner = owner;
  render_instance->entity_handle = target_scene->GetEntityHandle(owner);
  render_instance->renderer_handle = skinned_mesh_renderer->GetHandle();
  render_instance->model = gt;
  render_instance->skinned_mesh = skinned_mesh;
  render_instance->material = material;
  render_instance->cast_shadow = skinned_mesh_renderer->cast_shadow;
  render_instance->world_bound = mesh_bound;
  render_instance->bone_matrices = skinned_mesh_renderer->bone_matrices;
  render_instance->bone_matrices_snapshot = skinned_mesh_renderer->bone_matrices->value;
  render_instance->geometry_version = skinned_mesh->GetVersion();
  render_instance->ray_tracing_geometry_version = skinned_mesh_renderer->ray_tracing_geometry_version_;
  render_instance->morph_weights_version = skinned_mesh_renderer->morph_weights_version_;
  render_instance->ray_tracing_triangle_range = skinned_mesh_renderer->ray_tracing_triangle_range_;
  render_instance->ray_tracing_blas = skinned_mesh_renderer->ray_tracing_blas_;
  render_instance->material_version = material->GetVersion();
  render_instance->material_index = RegisterMaterial(material, material_data);
  render_instance->bone_matrices_version = skinned_mesh_renderer->bone_matrices->GetVersion();
  render_instance->entity_selected = target_scene->IsEntityAncestorSelected(owner);
  render_instance->line_width = material->draw_settings.line_width;
  render_instance->cull_mode = ResolveCullModeForTransform(material->draw_settings.cull_mode, gt.value);
  render_instance->polygon_mode = material->draw_settings.polygon_mode;

  if (UsesTransparentRasterPass(*material, material_data.shade_material)) {
    transparent_skinned_render_instances->Register(render_instance);
  } else {
    deferred_skinned_render_instances->Register(render_instance);
  }

  total_skinned_mesh_triangles += skinned_mesh->skinned_triangle_range_->prev_frame_index_count;
  return true;
}

bool RenderInstanceStorage::RegisterEntity(const std::shared_ptr<Scene>& target_scene, const Entity& owner,
                                           const std::shared_ptr<Particles>& particles, glm::vec3& min_bound,
                                           glm::vec3& max_bound) {
  auto material = particles->material.Get<Material>();
  auto mesh = particles->mesh.Get<Mesh>();
  auto particle_info_list = particles->particle_info_list.Get<ParticleInfoList>();
  if (!particles->IsEnabled() || !material || !mesh || !mesh->meshlet_range_ || !mesh->triangle_range_ ||
      !particle_info_list)
    return false;
  if (particle_info_list->PeekParticleInfoList().empty())
    return false;
  if (mesh->triangle_range_->prev_frame_index_count == 0 || mesh->meshlet_range_->prev_frame_range == 0)
    return false;
  auto gt = target_scene->GetDataComponent<GlobalTransform>(owner);
  auto ltw = gt.value;
  auto mesh_bound = mesh->GetBound();
  mesh_bound.ApplyTransform(ltw);
  glm::vec3 center = mesh_bound.Center();

  glm::vec3 size = mesh_bound.Size();
  min_bound = glm::vec3((glm::min)(min_bound.x, center.x - size.x), (glm::min)(min_bound.y, center.y - size.y),
                        (glm::min)(min_bound.z, center.z - size.z));

  max_bound = glm::vec3(glm::max(max_bound.x, center.x + size.x), glm::max(max_bound.y, center.y + size.y),
                        glm::max(max_bound.z, center.z + size.z));

  const auto material_data = BuildMaterialGltfData(*material);

  const auto render_instance = std::make_shared<InstancedRenderInstance>();
  render_instance->command_type = RenderInstanceType::FromRenderer;
  render_instance->model = gt;
  render_instance->owner = owner;
  render_instance->entity_handle = target_scene->GetEntityHandle(owner);
  render_instance->renderer_handle = particles->GetHandle();
  render_instance->mesh = mesh;
  render_instance->material = material;
  render_instance->cast_shadow = particles->cast_shadow;
  render_instance->particle_infos = particle_info_list;
  render_instance->geometry_version = mesh->GetVersion();
  render_instance->material_version = material->GetVersion();
  render_instance->material_index = RegisterMaterial(material, material_data);
  render_instance->particle_info_list_version = particle_info_list->GetVersion();
  render_instance->entity_selected = target_scene->IsEntityAncestorSelected(owner);
  render_instance->line_width = material->draw_settings.line_width;
  render_instance->cull_mode = ResolveInstancedCullModeForTransforms(material->draw_settings.cull_mode, gt.value,
                                                                     particle_info_list->PeekParticleInfoList());
  render_instance->polygon_mode = material->draw_settings.polygon_mode;

  if (UsesTransparentRasterPass(*material, material_data.shade_material)) {
    transparent_instanced_render_instances->Register(render_instance);
  } else {
    deferred_instanced_render_instances->Register(render_instance);
  }

  total_instanced_mesh_triangles +=
      mesh->triangle_range_->prev_frame_index_count * particle_info_list->PeekParticleInfoList().size();
  return true;
}

int RenderInstanceStorage::RegisterMaterial(const std::shared_ptr<Material>& material,
                                            const GltfMaterialData& material_data) {
  if (!material)
    return -1;
  const auto handle = material->GetHandle();
  const auto search = material_indices_.find(handle);
  if (search == material_indices_.end()) {
    const int index = static_cast<int>(gltf_material_cache_.GetShadeMaterials().size());
    material_indices_[handle] = index;
    const auto gltf_material_index = gltf_material_cache_.Append(material_data);
    if (gltf_material_index != static_cast<uint32_t>(index)) {
      throw std::runtime_error("glTF material cache drifted from render material indices.");
    }
    return index;
  }
  return search->second;
}

int RenderInstanceStorage::RegisterCamera(const Handle& handle, const CameraInfoBlock& camera_info_block) {
  const auto search = camera_indices_.find(handle);
  if (search == camera_indices_.end()) {
    const int index = camera_info_blocks_.size();
    camera_indices_[handle] = index;
    camera_info_blocks_.emplace_back(camera_info_block);
    return index;
  }
  return search->second;
}

int RenderInstanceStorage::GetMaterialIndex(const Handle& material_handle) {
  const auto search = material_indices_.find(material_handle);
  if (search == material_indices_.end()) {
    throw std::runtime_error("Unable to find material!");
  }
  return search->second;
}

int RenderInstanceStorage::GetRenderInstanceIndex(const Handle& renderer_handle) {
  const auto search = renderer_indices_.find(renderer_handle);
  if (search == renderer_indices_.end()) {
    throw std::runtime_error("Unable to find renderer!");
  }
  return search->second;
}

int RenderInstanceStorage::GetCameraIndex(const Handle& camera_handle) {
  const auto search = camera_indices_.find(camera_handle);
  if (search == camera_indices_.end()) {
    throw std::runtime_error("Unable to find camera!");
  }
  return search->second;
}

Handle RenderInstanceStorage::GetInstanceEntityHandle(const int render_instance_index) {
  const auto search = instance_entity_handles_.find(render_instance_index);
  if (search == instance_entity_handles_.end()) {
    return 0;
  }
  return search->second;
}

Handle RenderInstanceStorage::GetInstanceRendererHandle(const int render_instance_index) {
  const auto search = instance_renderer_handles_.find(render_instance_index);
  if (search == instance_renderer_handles_.end()) {
    return 0;
  }
  return search->second;
}
