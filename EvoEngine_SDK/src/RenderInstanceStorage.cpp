#include "RenderInstanceStorage.hpp"

#include "Application.hpp"
#include "EditorLayer.hpp"
#include "EnvironmentalLightingResolver.hpp"
#include "EnvironmentalMap.hpp"
#include "GlobalReflectionProbe.hpp"
#include "Jobs.hpp"
#include "LodGroup.hpp"
#include "Platform.hpp"
#include "Profiler.hpp"
#include "RenderLayer.hpp"
#include "Resources.hpp"
#include "Texture2D.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstring>
#include <functional>
#include <glm/gtc/constants.hpp>
#include <limits>
#include <numeric>
#include <thread>
#include <unordered_map>

using namespace evo_engine;

namespace {
VkDeviceSize StorageBufferAlignment() {
  return glm::max<VkDeviceSize>(
      4, Platform::GetSelectedPhysicalDevice()->properties.limits.minStorageBufferOffsetAlignment);
}

template <typename T>
VkDeviceSize AppendAligned(std::vector<T>& destination, const std::vector<T>& source, const VkDeviceSize alignment) {
  const size_t aligned_element_count = std::lcm(static_cast<size_t>(alignment), sizeof(T)) / sizeof(T);
  destination.resize(((destination.size() + aligned_element_count - 1) / aligned_element_count) *
                     aligned_element_count);
  const VkDeviceSize offset = destination.size() * sizeof(T);
  destination.insert(destination.end(), source.begin(), source.end());
  return offset;
}

void RunDedicatedRasterBatch(const size_t item_count, const std::function<void(size_t)>& function) {
  if (item_count == 0) {
    return;
  }
  std::atomic_size_t next_item = 0;
  const auto claim_items = [&] {
    while (true) {
      const size_t item = next_item.fetch_add(1, std::memory_order_relaxed);
      if (item >= item_count) {
        break;
      }
      function(item);
    }
  };
  const size_t worker_tasks = std::min(Jobs::GetWorkerSize(), item_count);
  std::vector<JobHandle> jobs;
  jobs.reserve(worker_tasks);
  for (size_t worker = 0; worker < worker_tasks; ++worker) {
    jobs.emplace_back(Jobs::Run([&] {
      claim_items();
    }));
  }
  const auto completed = Jobs::Combine(jobs);
  Jobs::Execute(completed);
  claim_items();
  while (!Jobs::IsCompleted(completed)) {
    std::this_thread::yield();
  }
  Jobs::Wait(completed);
}

template <typename T>
uint64_t VectorBytes(const std::vector<T>& values) {
  return static_cast<uint64_t>(values.size()) * sizeof(T);
}

uint64_t ByteSignature(const void* data, const size_t size) {
  auto signature = 1469598103934665603ull;
  const auto* bytes = static_cast<const uint8_t*>(data);
  for (size_t index = 0; index < size; ++index) {
    signature = (signature ^ bytes[index]) * 1099511628211ull;
  }
  return signature ^ size;
}

bool BoundsEqual(const Bound& left, const Bound& right) {
  return left.min == right.min && left.max == right.max;
}

Bound MergeBounds(const Bound& left, const Bound& right) {
  Bound result;
  result.min = glm::min(left.min, right.min);
  result.max = glm::max(left.max, right.max);
  return result;
}

bool ContainsBound(const Bound& container, const Bound& contained) {
  return glm::all(glm::lessThanEqual(container.min, contained.min)) &&
         glm::all(glm::greaterThanEqual(container.max, contained.max));
}

float BoundSurfaceArea(const Bound& bound) {
  const auto size = glm::max(bound.max - bound.min, glm::vec3(0.0f));
  return 2.0f * (size.x * size.y + size.y * size.z + size.z * size.x);
}

Bound FatBound(const Bound& bound) {
  const auto margin = glm::max((bound.max - bound.min) * 0.05f, glm::vec3(0.1f));
  Bound result;
  result.min = bound.min - margin;
  result.max = bound.max + margin;
  return result;
}

struct ClipSpaceBoundIntersector {
  std::array<glm::vec4, 6> planes{};

  [[nodiscard]] bool operator()(const Bound& bound) const {
    const auto finite_vec3 = [](const glm::vec3& value) {
      return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
    };
    if (!finite_vec3(bound.min) || !finite_vec3(bound.max) || glm::any(glm::greaterThan(bound.min, bound.max))) {
      return true;
    }
    for (const auto& plane : planes) {
      if (!std::isfinite(plane.x) || !std::isfinite(plane.y) || !std::isfinite(plane.z) || !std::isfinite(plane.w)) {
        return true;
      }
      const glm::vec3 vertex{plane.x >= 0.0f ? bound.max.x : bound.min.x, plane.y >= 0.0f ? bound.max.y : bound.min.y,
                             plane.z >= 0.0f ? bound.max.z : bound.min.z};
      if (glm::dot(plane, glm::vec4(vertex, 1.0f)) < 0.0f) {
        return false;
      }
    }
    return true;
  }
};

ClipSpaceBoundIntersector BuildClipSpaceBoundIntersector(const glm::mat4& projection_view, const bool zero_near_plane) {
  const glm::vec4 row_0{projection_view[0][0], projection_view[1][0], projection_view[2][0], projection_view[3][0]};
  const glm::vec4 row_1{projection_view[0][1], projection_view[1][1], projection_view[2][1], projection_view[3][1]};
  const glm::vec4 row_2{projection_view[0][2], projection_view[1][2], projection_view[2][2], projection_view[3][2]};
  const glm::vec4 row_3{projection_view[0][3], projection_view[1][3], projection_view[2][3], projection_view[3][3]};
  ClipSpaceBoundIntersector result;
  result.planes = {row_3 + row_0, row_3 - row_0, row_3 + row_1, row_3 - row_1, zero_near_plane ? row_2 : row_3 + row_2,
                   row_3 - row_2};
  return result;
}

template <typename T, typename Equal>
std::vector<RenderInstanceStorage::InstanceUploadRange> PlanUploadRanges(const std::vector<T>& previous,
                                                                         const std::vector<T>& current, Equal&& equal) {
  std::vector<RenderInstanceStorage::InstanceUploadRange> ranges;
  for (uint32_t index = 0; index < current.size(); ++index) {
    if (index < previous.size() && equal(previous[index], current[index])) {
      continue;
    }
    if (!ranges.empty() && ranges.back().first_instance + ranges.back().instance_count == index) {
      ranges.back().instance_count++;
    } else {
      ranges.push_back({index, 1u});
    }
  }
  return ranges;
}

template <typename T>
uint64_t AddUploadRanges(BufferUploadBatch& batch, const std::shared_ptr<Buffer>& buffer, const std::vector<T>& values,
                         const std::vector<RenderInstanceStorage::InstanceUploadRange>& ranges,
                         const BufferUploadOptions& options) {
  uint64_t uploaded_bytes = 0;
  for (const auto& range : ranges) {
    const auto byte_size = static_cast<size_t>(range.instance_count) * sizeof(T);
    const auto byte_offset = static_cast<size_t>(range.first_instance) * sizeof(T);
    if (byte_offset + byte_size > buffer->GetSize()) {
      batch.AddVector(buffer, values, options);
      return VectorBytes(values);
    }
    batch.Add(buffer, values.data() + range.first_instance, byte_size, byte_offset, options);
    uploaded_bytes += byte_size;
  }
  return uploaded_bytes;
}

template <typename T>
void CommitUploadedRanges(std::vector<T>& uploaded, const std::vector<T>& values,
                          const std::vector<RenderInstanceStorage::InstanceUploadRange>& ranges) {
  uploaded.resize(values.size());
  for (const auto& range : ranges) {
    std::copy_n(values.begin() + range.first_instance, range.instance_count, uploaded.begin() + range.first_instance);
  }
}

template <typename T>
void ResetCompactCollection(std::shared_ptr<T>& collection) {
  if (!collection) {
    collection = std::make_shared<T>();
  } else {
    collection->Clear();
  }
}

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

GltfRasterMaterialClass ResolveRasterMaterialClass(const Material& material, const GltfShadeMaterial& shade_material) {
  return ClassifyGltfRasterMaterial(shade_material, material.draw_settings.blending);
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

Bound CalculateInstancedWorldBound(const Bound& local_bound, const glm::mat4& model,
                                   const std::vector<ParticleInfo>& particle_infos) {
  Bound world_bound;
  for (const auto& particle_info : particle_infos) {
    auto instance_bound = local_bound;
    instance_bound.ApplyTransform(model * particle_info.instance_matrix.value);
    world_bound.min = glm::min(world_bound.min, instance_bound.min);
    world_bound.max = glm::max(world_bound.max, instance_bound.max);
  }
  return world_bound;
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

std::shared_ptr<Buffer> CreateIndirectBuffer() {
  VkBufferCreateInfo buffer_create_info{};
  buffer_create_info.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
  buffer_create_info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
  buffer_create_info.usage =
      VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_INDIRECT_BUFFER_BIT | VK_BUFFER_USAGE_STORAGE_BUFFER_BIT;
  buffer_create_info.size = 1;
  VmaAllocationCreateInfo allocation_create_info{};
  allocation_create_info.usage = VMA_MEMORY_USAGE_AUTO_PREFER_DEVICE;
  return std::make_shared<Buffer>(buffer_create_info, allocation_create_info);
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

void ValidateDeferredMeshIndirectCommandCount(const size_t deferred_count,
                                              const std::vector<VkDrawIndexedIndirectCommand>& indexed_commands,
                                              const std::vector<VkDrawMeshTasksIndirectCommandEXT>& mesh_task_commands,
                                              const std::vector<uint32_t>& draw_instance_indices) {
  if (!ApplicationContext::Get().GetLayer<RenderLayer>()) {
    return;
  }
  const auto indexed_count = indexed_commands.size();
  const auto mesh_task_count = mesh_task_commands.size();
  const auto draw_instance_count = draw_instance_indices.size();
  assert(indexed_count == deferred_count);
  assert(mesh_task_count == deferred_count);
  assert(draw_instance_count == deferred_count);
  if (indexed_count != deferred_count || mesh_task_count != deferred_count || draw_instance_count != deferred_count) {
    EVOENGINE_ERROR("Deferred mesh indirect command count mismatch: deferred_instances=" +
                    std::to_string(deferred_count) + ", indexed_commands=" + std::to_string(indexed_count) +
                    ", mesh_task_commands=" + std::to_string(mesh_task_count) +
                    ", draw_instance_indices=" + std::to_string(draw_instance_count))
  }
}

bool GaussianSplatGpuRadixSortSupported() {
  return Platform::Initialized() && Platform::GetInstance().GetCapabilities().subgroup_size >= 32u;
}

}  // namespace

void RenderInstanceStorage::EntitySelectionRenderSnapshot::Include(const Bound& bound) {
  if (!RenderInstanceStorage::IsFiniteBound(bound))
    return;
  if (!has_renderable_bounds) {
    world_bound = bound;
    has_renderable_bounds = true;
    return;
  }
  world_bound.min = glm::min(world_bound.min, bound.min);
  world_bound.max = glm::max(world_bound.max, bound.max);
}

bool RenderInstanceStorage::EntitySelectionRenderSnapshot::Matches(const std::shared_ptr<Scene>& target_scene,
                                                                   const uint64_t target_selection_revision,
                                                                   const uint64_t target_hierarchy_revision) const {
  return scene.lock() == target_scene && selection_revision == target_selection_revision &&
         hierarchy_revision == target_hierarchy_revision;
}

const RenderInstanceStorage::EntitySelectionRenderSnapshot& RenderInstanceStorage::GetEntitySelectionRenderSnapshot()
    const {
  return entity_selection_render_snapshot_;
}

bool RenderInstanceStorage::RasterSpatialIndex::Node::IsLeaf() const {
  return left < 0;
}

int32_t RenderInstanceStorage::RasterSpatialIndex::AllocateNode() {
  if (!free_nodes_.empty()) {
    const auto index = free_nodes_.back();
    free_nodes_.pop_back();
    nodes_[index] = {};
    nodes_[index].active = true;
    return index;
  }
  nodes_.emplace_back().active = true;
  return static_cast<int32_t>(nodes_.size() - 1u);
}

void RenderInstanceStorage::RasterSpatialIndex::ReleaseNode(const int32_t node_index) {
  nodes_[node_index] = {};
  free_nodes_.emplace_back(node_index);
}

void RenderInstanceStorage::RasterSpatialIndex::RefreshAncestors(int32_t node_index) {
  while (node_index >= 0) {
    auto& node = nodes_[node_index];
    if (!node.IsLeaf()) {
      node.bound = MergeBounds(nodes_[node.left].bound, nodes_[node.right].bound);
    }
    node_index = node.parent;
  }
}

void RenderInstanceStorage::RasterSpatialIndex::InsertLeaf(const int32_t leaf_index) {
  if (root_ < 0) {
    root_ = leaf_index;
    nodes_[leaf_index].parent = -1;
    return;
  }
  int32_t sibling = root_;
  while (!nodes_[sibling].IsLeaf()) {
    const auto& node = nodes_[sibling];
    const auto left_cost = BoundSurfaceArea(MergeBounds(nodes_[node.left].bound, nodes_[leaf_index].bound)) -
                           BoundSurfaceArea(nodes_[node.left].bound);
    const auto right_cost = BoundSurfaceArea(MergeBounds(nodes_[node.right].bound, nodes_[leaf_index].bound)) -
                            BoundSurfaceArea(nodes_[node.right].bound);
    sibling = left_cost <= right_cost ? node.left : node.right;
  }
  const int32_t previous_parent = nodes_[sibling].parent;
  const int32_t new_parent = AllocateNode();
  nodes_[new_parent].parent = previous_parent;
  nodes_[new_parent].left = sibling;
  nodes_[new_parent].right = leaf_index;
  nodes_[new_parent].bound = MergeBounds(nodes_[sibling].bound, nodes_[leaf_index].bound);
  nodes_[sibling].parent = new_parent;
  nodes_[leaf_index].parent = new_parent;
  if (previous_parent < 0) {
    root_ = new_parent;
  } else if (nodes_[previous_parent].left == sibling) {
    nodes_[previous_parent].left = new_parent;
  } else {
    nodes_[previous_parent].right = new_parent;
  }
  RefreshAncestors(new_parent);
}

void RenderInstanceStorage::RasterSpatialIndex::DetachLeaf(const int32_t leaf_index) {
  if (leaf_index == root_) {
    root_ = -1;
    nodes_[leaf_index].parent = -1;
    return;
  }
  const int32_t parent = nodes_[leaf_index].parent;
  const int32_t grand_parent = nodes_[parent].parent;
  const int32_t sibling = nodes_[parent].left == leaf_index ? nodes_[parent].right : nodes_[parent].left;
  if (grand_parent < 0) {
    root_ = sibling;
    nodes_[sibling].parent = -1;
  } else {
    if (nodes_[grand_parent].left == parent) {
      nodes_[grand_parent].left = sibling;
    } else {
      nodes_[grand_parent].right = sibling;
    }
    nodes_[sibling].parent = grand_parent;
    RefreshAncestors(grand_parent);
  }
  nodes_[leaf_index].parent = -1;
  ReleaseNode(parent);
}

void RenderInstanceStorage::RasterSpatialIndex::BeginUpdate() {
  seen_.clear();
  update_stats_ = {};
}

void RenderInstanceStorage::RasterSpatialIndex::Upsert(const Handle handle, const Bound& bound) {
  if (handle == 0) {
    return;
  }
  seen_.insert(handle);
  const auto found = leaves_.find(handle);
  if (found == leaves_.end()) {
    const int32_t leaf = AllocateNode();
    nodes_[leaf].handle = handle;
    nodes_[leaf].exact_bound = bound;
    nodes_[leaf].bound = FatBound(bound);
    leaves_[handle] = leaf;
    InsertLeaf(leaf);
    update_stats_.inserted_leaves++;
    return;
  }
  auto& leaf = nodes_[found->second];
  if (ContainsBound(leaf.bound, bound)) {
    leaf.exact_bound = bound;
    update_stats_.unchanged_leaves++;
    return;
  }
  DetachLeaf(found->second);
  leaf.exact_bound = bound;
  leaf.bound = FatBound(bound);
  InsertLeaf(found->second);
  update_stats_.reinserted_leaves++;
}

void RenderInstanceStorage::RasterSpatialIndex::RefreshTreeStats() {
  update_stats_.leaf_count = static_cast<uint32_t>(leaves_.size());
  update_stats_.node_count = update_stats_.leaf_count == 0u ? 0u : update_stats_.leaf_count * 2u - 1u;
  update_stats_.max_depth = 0;
  if (root_ < 0) {
    return;
  }
  std::vector<std::pair<int32_t, uint32_t>> pending{{root_, 1u}};
  while (!pending.empty()) {
    const auto [node_index, depth] = pending.back();
    pending.pop_back();
    update_stats_.max_depth = glm::max(update_stats_.max_depth, depth);
    const auto& node = nodes_[node_index];
    if (!node.IsLeaf()) {
      pending.emplace_back(node.left, depth + 1u);
      pending.emplace_back(node.right, depth + 1u);
    }
  }
}

void RenderInstanceStorage::RasterSpatialIndex::RebuildBalanced() {
  std::vector<int32_t> leaf_nodes;
  leaf_nodes.reserve(leaves_.size());
  for (int32_t index = 0; index < static_cast<int32_t>(nodes_.size()); ++index) {
    auto& node = nodes_[index];
    if (!node.active) {
      continue;
    }
    if (node.IsLeaf()) {
      node.parent = -1;
      leaf_nodes.emplace_back(index);
    } else {
      ReleaseNode(index);
    }
  }
  if (leaf_nodes.empty()) {
    root_ = -1;
    return;
  }
  const auto build = [&](auto&& self, const size_t begin, const size_t end) -> int32_t {
    if (end - begin == 1u) {
      return leaf_nodes[begin];
    }
    Bound centers;
    for (size_t index = begin; index < end; ++index) {
      const auto center = nodes_[leaf_nodes[index]].bound.Center();
      centers.min = glm::min(centers.min, center);
      centers.max = glm::max(centers.max, center);
    }
    const auto size = centers.Size();
    const int axis = size.x >= size.y && size.x >= size.z ? 0 : size.y >= size.z ? 1 : 2;
    const size_t middle = begin + (end - begin) / 2u;
    std::nth_element(leaf_nodes.begin() + begin, leaf_nodes.begin() + middle, leaf_nodes.begin() + end,
                     [&](const int32_t left, const int32_t right) {
                       return nodes_[left].bound.Center()[axis] < nodes_[right].bound.Center()[axis];
                     });
    const int32_t left = self(self, begin, middle);
    const int32_t right = self(self, middle, end);
    const int32_t parent = AllocateNode();
    nodes_[parent].left = left;
    nodes_[parent].right = right;
    nodes_[parent].bound = MergeBounds(nodes_[left].bound, nodes_[right].bound);
    nodes_[left].parent = parent;
    nodes_[right].parent = parent;
    return parent;
  };
  root_ = build(build, 0u, leaf_nodes.size());
  nodes_[root_].parent = -1;
}

void RenderInstanceStorage::RasterSpatialIndex::EndUpdate() {
  std::vector<Handle> removed;
  removed.reserve(leaves_.size());
  for (const auto& [handle, leaf] : leaves_) {
    if (seen_.find(handle) == seen_.end()) {
      removed.emplace_back(handle);
    }
  }
  for (const auto handle : removed) {
    const int32_t leaf = leaves_.at(handle);
    DetachLeaf(leaf);
    ReleaseNode(leaf);
    leaves_.erase(handle);
    update_stats_.removed_leaves++;
  }
  RefreshTreeStats();
  if (update_stats_.inserted_leaves + update_stats_.removed_leaves + update_stats_.reinserted_leaves != 0u &&
      update_stats_.leaf_count > 2u) {
    const auto target_depth = static_cast<uint32_t>(std::ceil(std::log2(update_stats_.leaf_count))) + 1u;
    if (update_stats_.max_depth > target_depth * 2u) {
      RebuildBalanced();
      RefreshTreeStats();
    }
  }
}

std::vector<Handle> RenderInstanceStorage::RasterSpatialIndex::Query(
    const std::function<bool(const Bound&)>& intersects, QueryStats* stats) const {
  QueryStats local_stats{};
  std::vector<Handle> result;
  if (root_ >= 0) {
    std::vector<int32_t> pending{root_};
    while (!pending.empty()) {
      const int32_t node_index = pending.back();
      pending.pop_back();
      const auto& node = nodes_[node_index];
      local_stats.visited_nodes++;
      if (!intersects(node.bound)) {
        continue;
      }
      if (node.IsLeaf()) {
        local_stats.tested_leaves++;
        if (intersects(node.exact_bound)) {
          result.emplace_back(node.handle);
          local_stats.accepted_leaves++;
        }
      } else {
        pending.emplace_back(node.left);
        pending.emplace_back(node.right);
      }
    }
  }
  if (stats) {
    *stats = local_stats;
  }
  return result;
}

const RenderInstanceStorage::RasterSpatialIndex::UpdateStats&
RenderInstanceStorage::RasterSpatialIndex::GetUpdateStats() const {
  return update_stats_;
}

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
  instance_info_block.triangle_offset = strands->segment_range_ ? strands->segment_range_->prev_frame_offset : 0;
  instance_info_block.meshlet_index_offset =
      strands->strand_meshlet_range_ ? strands->strand_meshlet_range_->prev_frame_offset : 0;
  instance_info_block.meshlet_size =
      strands->strand_meshlet_range_ ? strands->strand_meshlet_range_->prev_frame_range : 0;
  instance_info_block.entity_index = owner.GetIndex();
  instance_info_block.renderer_handle = renderer_handle;
  instance_info_block.ray_tracing_geometry = strands->ray_tracing_index_range_ && strands->ray_tracing_point_range_
                                                 ? glm::ivec4(1, strands->ray_tracing_index_range_->prev_frame_offset,
                                                              strands->ray_tracing_point_range_->prev_frame_offset,
                                                              strands->ray_tracing_index_range_->prev_frame_range)
                                                 : glm::ivec4(0);
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

void RenderInstanceStorage::ExternalRenderInstanceCollection::Clear() {
  render_commands.clear();
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

void RenderInstanceStorage::MeshRenderInstanceCollection::Clear() {
  render_commands.clear();
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

void RenderInstanceStorage::SkinnedMeshRenderInstanceCollection::Clear() {
  render_commands.clear();
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

void RenderInstanceStorage::StrandsRenderInstanceCollection::Clear() {
  render_commands.clear();
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

void RenderInstanceStorage::GaussianSplatRenderInstanceCollection::Clear() {
  render_commands.clear();
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

void RenderInstanceStorage::InstancedRenderInstanceCollection::Clear() {
  render_commands.clear();
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

  shadow_cascade_transition_width = glm::max(target_render_settings.shadow_cascade_transition_width, 0.0f);
  shadow_debug_parameters = glm::ivec4(glm::clamp(target_render_settings.shadow_debug_mode, 0, 5),
                                       glm::clamp(target_render_settings.shadow_debug_selected_cascade, 0, 3),
                                       glm::max(target_render_settings.shadow_debug_selected_light, 0), 0);
  shadow_fade_parameters = glm::vec4(
      glm::clamp(target_render_settings.shadow_distance_fade, 0.0f,
                 glm::max(target_render_settings.max_shadow_distance, 0.0f)),
      static_cast<float>(glm::clamp(static_cast<int>(target_render_settings.indirect_lighting_debug_view), 0, 5)), 0.0f,
      0.0f);
  strands_subdivision_x_factor = target_render_settings.strands_subdivision_x_factor;
  strands_subdivision_y_factor = target_render_settings.strands_subdivision_y_factor;
  strands_subdivision_max_x = target_render_settings.strands_subdivision_max_x;
  strands_subdivision_max_y = target_render_settings.strands_subdivision_max_y;
}

bool RenderInstanceStorage::RenderInfoBlock::operator!=(const RenderInfoBlock& other) const {
  if (split_distances != other.split_distances)
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

std::vector<RenderInstanceStorage::EmissiveAliasEntry> RenderInstanceStorage::BuildEmissiveAliasTable(
    const std::vector<double>& weights) {
  if (weights.empty()) {
    return {};
  }
  double maximum_weight = 0.0;
  for (const auto weight : weights) {
    if (std::isfinite(weight) && weight > maximum_weight) {
      maximum_weight = weight;
    }
  }
  if (!(maximum_weight > 0.0)) {
    return {};
  }

  std::vector<double> probabilities(weights.size());
  double scaled_weight_sum = 0.0;
  for (size_t index = 0; index < weights.size(); ++index) {
    if (std::isfinite(weights[index]) && weights[index] > 0.0) {
      probabilities[index] = weights[index] / maximum_weight;
      scaled_weight_sum += probabilities[index];
    }
  }
  if (!std::isfinite(scaled_weight_sum) || scaled_weight_sum <= 0.0) {
    return {};
  }

  std::vector<EmissiveAliasEntry> result(weights.size());
  std::vector<double> alias_probabilities(weights.size());
  std::vector<size_t> underfull_entries;
  std::vector<size_t> full_entries;
  underfull_entries.reserve(weights.size());
  full_entries.reserve(weights.size());
  for (size_t index = 0; index < weights.size(); ++index) {
    probabilities[index] /= scaled_weight_sum;
    alias_probabilities[index] = probabilities[index] * static_cast<double>(weights.size());
    result[index].alias_index = static_cast<uint32_t>(index);
    (alias_probabilities[index] < 1.0 ? underfull_entries : full_entries).emplace_back(index);
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

  const double inverse_size = 1.0 / static_cast<double>(weights.size());
  std::vector<double> quantized_probabilities(weights.size());
  for (size_t index = 0; index < result.size(); ++index) {
    const double direct_probability = static_cast<double>(result[index].alias_probability) * inverse_size;
    quantized_probabilities[index] += direct_probability;
    quantized_probabilities[result[index].alias_index] += inverse_size - direct_probability;
  }
  for (size_t index = 0; index < result.size(); ++index) {
    result[index].selection_probability = static_cast<float>(quantized_probabilities[index]);
  }
  return result;
}

std::vector<RenderInstanceStorage::EmissiveTriangleInfoBlock> RenderInstanceStorage::BuildEmissiveTriangleDistribution(
    std::vector<EmissiveTriangleCandidate> candidates) {
  candidates.erase(std::remove_if(candidates.begin(), candidates.end(),
                                  [](const EmissiveTriangleCandidate& candidate) {
                                    return !std::isfinite(candidate.area) || !std::isfinite(candidate.importance) ||
                                           candidate.area <= 0.0 || candidate.importance <= 0.0;
                                  }),
                   candidates.end());
  std::sort(candidates.begin(), candidates.end(),
            [](const EmissiveTriangleCandidate& lhs, const EmissiveTriangleCandidate& rhs) {
              return lhs.primitive_id < rhs.primitive_id;
            });

  double maximum_log_weight = -std::numeric_limits<double>::infinity();
  for (const auto& candidate : candidates) {
    maximum_log_weight = glm::max(maximum_log_weight, std::log(candidate.area) + std::log(candidate.importance));
  }
  if (!std::isfinite(maximum_log_weight)) {
    return {};
  }

  std::vector<double> weights(candidates.size());
  for (size_t i = 0; i < candidates.size(); ++i) {
    weights[i] = std::exp(std::log(candidates[i].area) + std::log(candidates[i].importance) - maximum_log_weight);
  }
  const auto alias_entries = BuildEmissiveAliasTable(weights);
  if (alias_entries.size() != candidates.size()) {
    return {};
  }
  std::vector<EmissiveTriangleInfoBlock> result;
  result.reserve(candidates.size());
  for (size_t i = 0; i < candidates.size(); ++i) {
    result.push_back({candidates[i].primitive_id, alias_entries[i].alias_probability, alias_entries[i].alias_index,
                      alias_entries[i].selection_probability});
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
  if (entity_index != other.entity_index)
    return true;
  if (renderer_handle != other.renderer_handle)
    return true;
  if (ray_tracing_geometry != other.ray_tracing_geometry)
    return true;
  if (world_bound_min != other.world_bound_min || world_bound_max != other.world_bound_max)
    return true;
  return false;
}

bool RenderInstanceStorage::IsFiniteBound(const Bound& bound) {
  return std::isfinite(bound.min.x) && std::isfinite(bound.min.y) && std::isfinite(bound.min.z) &&
         std::isfinite(bound.max.x) && std::isfinite(bound.max.y) && std::isfinite(bound.max.z) &&
         bound.min.x <= bound.max.x && bound.min.y <= bound.max.y && bound.min.z <= bound.max.z;
}

bool RenderInstanceStorage::BoundIntersectsCameraClipSpace(const Bound& bound, const glm::mat4& projection_view) {
  if (!IsFiniteBound(bound)) {
    return true;
  }
  const std::array corners = {
      glm::vec3{bound.min.x, bound.min.y, bound.min.z}, glm::vec3{bound.max.x, bound.min.y, bound.min.z},
      glm::vec3{bound.min.x, bound.max.y, bound.min.z}, glm::vec3{bound.max.x, bound.max.y, bound.min.z},
      glm::vec3{bound.min.x, bound.min.y, bound.max.z}, glm::vec3{bound.max.x, bound.min.y, bound.max.z},
      glm::vec3{bound.min.x, bound.max.y, bound.max.z}, glm::vec3{bound.max.x, bound.max.y, bound.max.z}};
  std::array<uint32_t, 6> outside{};
  for (const auto& corner : corners) {
    const auto clip = projection_view * glm::vec4(corner, 1.0f);
    if (!std::isfinite(clip.x) || !std::isfinite(clip.y) || !std::isfinite(clip.z) || !std::isfinite(clip.w)) {
      return true;
    }
    outside[0] += clip.x < -clip.w ? 1u : 0u;
    outside[1] += clip.x > clip.w ? 1u : 0u;
    outside[2] += clip.y < -clip.w ? 1u : 0u;
    outside[3] += clip.y > clip.w ? 1u : 0u;
    outside[4] += clip.z < 0.0f ? 1u : 0u;
    outside[5] += clip.z > clip.w ? 1u : 0u;
  }
  return std::none_of(outside.begin(), outside.end(), [&](const uint32_t count) {
    return count == corners.size();
  });
}

bool RenderInstanceStorage::BoundIntersectsShadowClipSpace(const Bound& bound, const glm::mat4& projection_view) {
  if (!IsFiniteBound(bound)) {
    return true;
  }
  const std::array corners = {
      glm::vec3{bound.min.x, bound.min.y, bound.min.z}, glm::vec3{bound.max.x, bound.min.y, bound.min.z},
      glm::vec3{bound.min.x, bound.max.y, bound.min.z}, glm::vec3{bound.max.x, bound.max.y, bound.min.z},
      glm::vec3{bound.min.x, bound.min.y, bound.max.z}, glm::vec3{bound.max.x, bound.min.y, bound.max.z},
      glm::vec3{bound.min.x, bound.max.y, bound.max.z}, glm::vec3{bound.max.x, bound.max.y, bound.max.z}};
  std::array<uint32_t, 6> outside{};
  for (const auto& corner : corners) {
    const auto clip = projection_view * glm::vec4(corner, 1.0f);
    if (!std::isfinite(clip.x) || !std::isfinite(clip.y) || !std::isfinite(clip.z) || !std::isfinite(clip.w)) {
      return true;
    }
    outside[0] += clip.x < -clip.w ? 1u : 0u;
    outside[1] += clip.x > clip.w ? 1u : 0u;
    outside[2] += clip.y < -clip.w ? 1u : 0u;
    outside[3] += clip.y > clip.w ? 1u : 0u;
    outside[4] += clip.z < -clip.w ? 1u : 0u;
    outside[5] += clip.z > clip.w ? 1u : 0u;
  }
  return std::none_of(outside.begin(), outside.end(), [&](const uint32_t count) {
    return count == corners.size();
  });
}

bool RenderInstanceStorage::MeshletSphereIntersectsClipSpace(const glm::vec4& local_sphere, const glm::mat4& model,
                                                             const glm::mat4& projection_view,
                                                             const bool zero_near_plane) {
  if (!std::isfinite(local_sphere.x) || !std::isfinite(local_sphere.y) || !std::isfinite(local_sphere.z) ||
      !std::isfinite(local_sphere.w) || local_sphere.w < 0.0f) {
    return true;
  }
  const glm::vec3 basis_x = model * glm::vec4(1.0f, 0.0f, 0.0f, 0.0f);
  const glm::vec3 basis_y = model * glm::vec4(0.0f, 1.0f, 0.0f, 0.0f);
  const glm::vec3 basis_z = model * glm::vec4(0.0f, 0.0f, 1.0f, 0.0f);
  const glm::vec3 basis_lengths{glm::length(basis_x), glm::length(basis_y), glm::length(basis_z)};
  const bool orthogonal = std::abs(glm::dot(basis_x, basis_y)) <= 1e-5f * basis_lengths.x * basis_lengths.y &&
                          std::abs(glm::dot(basis_x, basis_z)) <= 1e-5f * basis_lengths.x * basis_lengths.z &&
                          std::abs(glm::dot(basis_y, basis_z)) <= 1e-5f * basis_lengths.y * basis_lengths.z;
  const float world_scale =
      orthogonal ? glm::max(basis_lengths.x, glm::max(basis_lengths.y, basis_lengths.z)) : glm::length(basis_lengths);
  const glm::vec3 world_center = model * glm::vec4(local_sphere.x, local_sphere.y, local_sphere.z, 1.0f);
  const float world_radius = local_sphere.w * world_scale;
  if (!std::isfinite(world_center.x) || !std::isfinite(world_center.y) || !std::isfinite(world_center.z) ||
      !std::isfinite(world_radius)) {
    return true;
  }
  Bound world_bound;
  world_bound.min = world_center - world_radius;
  world_bound.max = world_center + world_radius;
  return zero_near_plane ? BoundIntersectsCameraClipSpace(world_bound, projection_view)
                         : BoundIntersectsShadowClipSpace(world_bound, projection_view);
}

RenderInstanceStorage::ShadowViewIndirectCommands RenderInstanceStorage::BuildShadowViewIndirectCommands(
    const glm::mat4& light_space_matrix) {
  const ProfilerScope profiler_scope("RenderInstanceStorage::BuildShadowViewIndirectCommands", "Render");
  ShadowViewIndirectCommands result;
  result.deferred_render_instances = std::make_shared<MeshRenderInstanceCollection>();
  result.deferred_skinned_render_instances = std::make_shared<SkinnedMeshRenderInstanceCollection>();
  result.deferred_instanced_render_instances = std::make_shared<InstancedRenderInstanceCollection>();
  result.deferred_strands_render_instances = std::make_shared<StrandsRenderInstanceCollection>();
  result.deferred_masked_render_instances = std::make_shared<MeshRenderInstanceCollection>();
  result.deferred_masked_skinned_render_instances = std::make_shared<SkinnedMeshRenderInstanceCollection>();
  result.deferred_masked_instanced_render_instances = std::make_shared<InstancedRenderInstanceCollection>();
  result.deferred_masked_strands_render_instances = std::make_shared<StrandsRenderInstanceCollection>();
  const auto visible_entries = QueryRasterSpatialEntries(BuildClipSpaceBoundIntersector(light_space_matrix, false));
  for (const auto& entry : visible_entries) {
    const auto& render_instance = entry.render_instance;
    if (!render_instance ||
        (entry.category != SpatialRenderCategory::Deferred &&
         entry.category != SpatialRenderCategory::DeferredMasked) ||
        !render_instance->cast_shadow) {
      continue;
    }
    if (entry.deferred_mesh_command_index < 0) {
      const bool masked = entry.category == SpatialRenderCategory::DeferredMasked;
      if (std::dynamic_pointer_cast<SkinnedMeshRenderInstance>(render_instance)) {
        (masked ? result.deferred_masked_skinned_render_instances : result.deferred_skinned_render_instances)
            ->Register(render_instance);
      } else if (std::dynamic_pointer_cast<InstancedRenderInstance>(render_instance)) {
        (masked ? result.deferred_masked_instanced_render_instances : result.deferred_instanced_render_instances)
            ->Register(render_instance);
      } else if (std::dynamic_pointer_cast<StrandsRenderInstance>(render_instance)) {
        (masked ? result.deferred_masked_strands_render_instances : result.deferred_strands_render_instances)
            ->Register(render_instance);
      }
    }
  }
  const auto append_mesh_commands = [&](const SpatialRenderCategory category,
                                        const std::shared_ptr<MeshRenderInstanceCollection>& collection,
                                        std::vector<DeferredMeshIndirectBatch>& batches) {
    for (const auto& entry : visible_entries) {
      if (entry.category != category || entry.deferred_mesh_command_index < 0 || !entry.render_instance ||
          !entry.render_instance->cast_shadow) {
        continue;
      }
      const auto render_instance = std::dynamic_pointer_cast<MeshRenderInstance>(entry.render_instance);
      const auto command_index = static_cast<size_t>(entry.deferred_mesh_command_index);
      if (!render_instance || command_index >= shadow_mesh_draw_indexed_indirect_commands.size() ||
          command_index >= shadow_mesh_draw_mesh_tasks_indirect_commands.size()) {
        continue;
      }
      const auto& indexed_command = shadow_mesh_draw_indexed_indirect_commands[command_index];
      if (indexed_command.indexCount == 0u) {
        continue;
      }
      const auto compact_command_index = static_cast<uint32_t>(result.indexed_commands.size());
      result.indexed_commands.emplace_back(indexed_command);
      result.mesh_task_commands.emplace_back(shadow_mesh_draw_mesh_tasks_indirect_commands[command_index]);
      result.draw_instance_indices.emplace_back(static_cast<uint32_t>(render_instance->instance_index));
      collection->Register(render_instance);
      const auto same_batch = [&](const DeferredMeshIndirectBatch& batch) {
        return batch.line_width == render_instance->line_width && batch.cull_mode == render_instance->cull_mode &&
               batch.polygon_mode == render_instance->polygon_mode;
      };
      if (batches.empty() || !same_batch(batches.back())) {
        auto& batch = batches.emplace_back();
        batch.first_command = compact_command_index;
        batch.line_width = render_instance->line_width;
        batch.cull_mode = render_instance->cull_mode;
        batch.polygon_mode = render_instance->polygon_mode;
      }
      auto& batch = batches.back();
      batch.command_count++;
      batch.triangle_count += indexed_command.indexCount / 3u;
    }
  };
  append_mesh_commands(SpatialRenderCategory::Deferred, result.deferred_render_instances,
                       result.opaque_mesh_indirect_batches);
  append_mesh_commands(SpatialRenderCategory::DeferredMasked, result.deferred_masked_render_instances,
                       result.masked_mesh_indirect_batches);
  return result;
}

void RenderInstanceStorage::MergeShadowRasterVisibilityResult(ShadowViewIndirectCommands& destination,
                                                              ShadowViewIndirectCommands&& visibility,
                                                              const bool use_mesh_shader) {
  visibility.draw_instance_index_offset = static_cast<uint32_t>(raster_draw_instance_indices.size());
  raster_draw_instance_indices.insert(raster_draw_instance_indices.end(), visibility.draw_instance_indices.begin(),
                                      visibility.draw_instance_indices.end());
  if (!visibility.indexed_commands.empty()) {
    if (use_mesh_shader) {
      visibility.indirect_buffer_offset = VectorBytes(packed_shadow_mesh_task_commands);
      packed_shadow_mesh_task_commands.insert(packed_shadow_mesh_task_commands.end(),
                                              visibility.mesh_task_commands.begin(),
                                              visibility.mesh_task_commands.end());
    } else {
      visibility.indirect_buffer_offset = VectorBytes(packed_shadow_indexed_commands);
      packed_shadow_indexed_commands.insert(packed_shadow_indexed_commands.end(), visibility.indexed_commands.begin(),
                                            visibility.indexed_commands.end());
    }
    visibility.indirect_buffer = packed_shadow_indirect_buffer;
  }
  destination = std::move(visibility);
}

const RenderInstanceStorage::ShadowViewIndirectCommands* RenderInstanceStorage::GetDirectionalShadowView(
    const int32_t camera_index, const int32_t light_index, const uint32_t split) const {
  if (camera_index < 0 || light_index < 0 || split >= 4u || directional_shadow_light_count_ == 0u) {
    return nullptr;
  }
  const size_t index = (static_cast<size_t>(camera_index) * 4u + split) * directional_shadow_light_count_ +
                       static_cast<size_t>(light_index);
  return index < directional_shadow_views_.size() ? &directional_shadow_views_[index] : nullptr;
}

const RenderInstanceStorage::ShadowViewIndirectCommands* RenderInstanceStorage::GetPointShadowView(
    const int32_t light_index, const uint32_t face) const {
  if (light_index < 0 || face >= 6u || point_light_info_blocks_.empty()) {
    return nullptr;
  }
  const size_t index = face * point_light_info_blocks_.size() + static_cast<size_t>(light_index);
  return index < point_shadow_views_.size() ? &point_shadow_views_[index] : nullptr;
}

const RenderInstanceStorage::ShadowViewIndirectCommands* RenderInstanceStorage::GetSpotShadowView(
    const int32_t light_index) const {
  return light_index >= 0 && static_cast<size_t>(light_index) < spot_shadow_views_.size()
             ? &spot_shadow_views_[light_index]
             : nullptr;
}

void RenderInstanceStorage::FinalizeShadowIndirectBuffers(const bool use_mesh_shader) {
  packed_shadow_uses_mesh_shader_ = use_mesh_shader;
}

RenderInstanceStorage::CameraRasterVisibility RenderInstanceStorage::BuildCameraRasterVisibilityResult(
    const size_t camera_index) const {
  const ProfilerScope profiler_scope("RenderInstanceStorage::BuildCameraRasterVisibilityResult", "Render");
  CameraRasterVisibility visibility;
  visibility.enabled = true;
  visibility.draw_instance_index_offset = deferred_mesh_draw_instance_index_offset;
  visibility.instance_visibility.assign(instance_info_blocks_.size(), 1u);
  visibility.deferred_mesh_indirect_batches.clear();
  visibility.deferred_masked_mesh_indirect_batches.clear();
  visibility.mesh_draw_indexed_indirect_commands.clear();
  visibility.mesh_draw_mesh_tasks_indirect_commands.clear();
  visibility.total_gaussian_splats = 0;
  ResetCompactCollection(visibility.deferred_render_instances);
  ResetCompactCollection(visibility.deferred_skinned_render_instances);
  ResetCompactCollection(visibility.deferred_instanced_render_instances);
  ResetCompactCollection(visibility.deferred_strands_render_instances);
  ResetCompactCollection(visibility.deferred_masked_render_instances);
  ResetCompactCollection(visibility.deferred_masked_skinned_render_instances);
  ResetCompactCollection(visibility.deferred_masked_instanced_render_instances);
  ResetCompactCollection(visibility.deferred_masked_strands_render_instances);
  ResetCompactCollection(visibility.forward_render_instances);
  ResetCompactCollection(visibility.forward_skinned_render_instances);
  ResetCompactCollection(visibility.forward_instanced_render_instances);
  ResetCompactCollection(visibility.forward_strands_render_instances);
  ResetCompactCollection(visibility.transparent_render_instances);
  ResetCompactCollection(visibility.transparent_skinned_render_instances);
  ResetCompactCollection(visibility.transparent_instanced_render_instances);
  ResetCompactCollection(visibility.transparent_strands_render_instances);
  ResetCompactCollection(visibility.gaussian_splat_render_instances);
  const auto& camera_info = camera_info_blocks_[camera_index];
  const auto unjittered_intersects = BuildClipSpaceBoundIntersector(camera_info.unjittered_projection_view, true);
  const auto jittered_intersects = BuildClipSpaceBoundIntersector(camera_info.projection_view, true);
  const auto intersects = [&](const Bound& bound) {
    return unjittered_intersects(bound) || jittered_intersects(bound);
  };
  for (const auto& [handle, entry] : spatial_render_entries_) {
    if (entry.render_instance && entry.render_instance->instance_index >= 0 &&
        static_cast<size_t>(entry.render_instance->instance_index) < visibility.instance_visibility.size()) {
      visibility.instance_visibility[entry.render_instance->instance_index] = 0u;
    }
  }
  const auto visible_entries = QueryRasterSpatialEntries(intersects);
  const auto register_geometry = [&](const SpatialRenderEntry& entry, const auto& mesh_collection,
                                     const auto& skinned_collection, const auto& instanced_collection,
                                     const auto& strands_collection) {
    if (std::dynamic_pointer_cast<MeshRenderInstance>(entry.render_instance)) {
      mesh_collection->Register(entry.render_instance);
    } else if (std::dynamic_pointer_cast<SkinnedMeshRenderInstance>(entry.render_instance)) {
      skinned_collection->Register(entry.render_instance);
    } else if (std::dynamic_pointer_cast<InstancedRenderInstance>(entry.render_instance)) {
      instanced_collection->Register(entry.render_instance);
    } else if (std::dynamic_pointer_cast<StrandsRenderInstance>(entry.render_instance)) {
      strands_collection->Register(entry.render_instance);
    }
  };
  for (const auto& entry : visible_entries) {
    const auto& render_instance = entry.render_instance;
    if (!render_instance || render_instance->instance_index < 0 ||
        static_cast<size_t>(render_instance->instance_index) >= visibility.instance_visibility.size()) {
      continue;
    }
    visibility.instance_visibility[render_instance->instance_index] = 1u;
    switch (entry.category) {
      case SpatialRenderCategory::Deferred:
        register_geometry(entry, visibility.deferred_render_instances, visibility.deferred_skinned_render_instances,
                          visibility.deferred_instanced_render_instances, visibility.deferred_strands_render_instances);
        break;
      case SpatialRenderCategory::DeferredMasked:
        register_geometry(
            entry, visibility.deferred_masked_render_instances, visibility.deferred_masked_skinned_render_instances,
            visibility.deferred_masked_instanced_render_instances, visibility.deferred_masked_strands_render_instances);
        break;
      case SpatialRenderCategory::Forward:
        register_geometry(entry, visibility.forward_render_instances, visibility.forward_skinned_render_instances,
                          visibility.forward_instanced_render_instances, visibility.forward_strands_render_instances);
        break;
      case SpatialRenderCategory::Transparent:
        register_geometry(
            entry, visibility.transparent_render_instances, visibility.transparent_skinned_render_instances,
            visibility.transparent_instanced_render_instances, visibility.transparent_strands_render_instances);
        break;
      case SpatialRenderCategory::Gaussian:
        visibility.gaussian_splat_render_instances->Register(render_instance);
        break;
    }
  }
  visibility.gaussian_splat_render_instances->ForEachGaussianSplatRenderInstance([&](const auto& render_instance) {
    if (render_instance && render_instance->gaussian_splat) {
      visibility.total_gaussian_splats += render_instance->gaussian_splat->GetSplatCount();
    }
  });
  visibility.draw_instance_indices.reserve(mesh_draw_indexed_indirect_commands.size());
  const auto append_mesh_commands = [&](const SpatialRenderCategory category,
                                        std::vector<DeferredMeshIndirectBatch>& batches) {
    for (const auto& entry : visible_entries) {
      if (entry.category != category || entry.deferred_mesh_command_index < 0) {
        continue;
      }
      const auto render_instance = std::dynamic_pointer_cast<MeshRenderInstance>(entry.render_instance);
      const auto command_index = static_cast<size_t>(entry.deferred_mesh_command_index);
      if (command_index >= mesh_draw_indexed_indirect_commands.size() ||
          command_index >= mesh_draw_mesh_tasks_indirect_commands.size() || !render_instance) {
        visibility.enabled = false;
        return;
      }
      const auto compact_command_index = static_cast<uint32_t>(visibility.mesh_draw_indexed_indirect_commands.size());
      visibility.mesh_draw_indexed_indirect_commands.emplace_back(mesh_draw_indexed_indirect_commands[command_index]);
      visibility.mesh_draw_mesh_tasks_indirect_commands.emplace_back(
          mesh_draw_mesh_tasks_indirect_commands[command_index]);
      visibility.draw_instance_indices.emplace_back(static_cast<uint32_t>(render_instance->instance_index));
      const auto same_batch = [&](const DeferredMeshIndirectBatch& batch) {
        return batch.line_width == render_instance->line_width && batch.cull_mode == render_instance->cull_mode &&
               batch.polygon_mode == render_instance->polygon_mode;
      };
      if (batches.empty() || !same_batch(batches.back())) {
        auto& batch = batches.emplace_back();
        batch.first_command = compact_command_index;
        batch.line_width = render_instance->line_width;
        batch.cull_mode = render_instance->cull_mode;
        batch.polygon_mode = render_instance->polygon_mode;
      }
      auto& batch = batches.back();
      batch.command_count++;
      batch.triangle_count += mesh_draw_indexed_indirect_commands[command_index].indexCount / 3u;
    }
  };
  append_mesh_commands(SpatialRenderCategory::Deferred, visibility.deferred_mesh_indirect_batches);
  if (visibility.enabled) {
    append_mesh_commands(SpatialRenderCategory::DeferredMasked, visibility.deferred_masked_mesh_indirect_batches);
  }
  if (!visibility.enabled) {
    return visibility;
  }
  return visibility;
}

void RenderInstanceStorage::MergeCameraRasterVisibilityResult(const size_t camera_index,
                                                              CameraRasterVisibility&& visibility) {
  visibility.draw_instance_index_offset = static_cast<uint32_t>(raster_draw_instance_indices.size());
  raster_draw_instance_indices.insert(raster_draw_instance_indices.end(), visibility.draw_instance_indices.begin(),
                                      visibility.draw_instance_indices.end());
  if (visibility.enabled) {
    const auto alignment = StorageBufferAlignment();
    visibility.indexed_indirect_buffer_offset =
        AppendAligned(packed_camera_indexed_commands, visibility.mesh_draw_indexed_indirect_commands, alignment);
    visibility.mesh_task_indirect_buffer_offset =
        AppendAligned(packed_camera_mesh_task_commands, visibility.mesh_draw_mesh_tasks_indirect_commands, alignment);
    visibility.mesh_draw_indexed_indirect_commands_buffer = packed_camera_indexed_buffer;
    visibility.mesh_draw_mesh_tasks_indirect_commands_buffer = packed_camera_mesh_task_buffer;
  }
  camera_raster_visibility_[camera_index] = std::move(visibility);
}

void RenderInstanceStorage::BuildRasterVisibility(const bool use_mesh_shader) {
  const auto canonical_mapping_end =
      deferred_mesh_draw_instance_index_offset + static_cast<uint32_t>(mesh_draw_indexed_indirect_commands.size());
  raster_draw_instance_indices.resize(canonical_mapping_end);
  packed_camera_indexed_commands.clear();
  packed_camera_mesh_task_commands.clear();
  camera_raster_visibility_.resize(camera_info_blocks_.size());

  directional_shadow_views_.clear();
  point_shadow_views_.clear();
  spot_shadow_views_.clear();
  packed_shadow_indexed_commands.clear();
  packed_shadow_mesh_task_commands.clear();
  directional_shadow_light_count_ = static_cast<uint32_t>(glm::max(render_info_block.directional_light_size, 0));
  struct ShadowWorkItem {
    ShadowViewIndirectCommands* destination = nullptr;
    glm::mat4 light_space_matrix{1.0f};
  };
  std::vector<ShadowWorkItem> shadow_work_items;
  const size_t camera_count = camera_info_blocks_.size();
  const size_t directional_stride = camera_count == 0 ? 0 : directional_light_info_blocks_.size() / camera_count;
  directional_shadow_views_.resize(camera_count * 4u * directional_shadow_light_count_);
  for (size_t camera_index = 0; camera_index < camera_count; ++camera_index) {
    for (uint32_t split = 0; split < 4u; ++split) {
      for (uint32_t light_index = 0; light_index < directional_shadow_light_count_; ++light_index) {
        const auto block_index = camera_index * directional_stride + light_index;
        if (block_index >= directional_light_info_blocks_.size() ||
            directional_light_info_blocks_[block_index].diffuse.w <= 0.5f) {
          continue;
        }
        shadow_work_items.push_back(
            {&directional_shadow_views_[(camera_index * 4u + split) * directional_shadow_light_count_ + light_index],
             directional_light_info_blocks_[block_index].light_space_matrix[split]});
      }
    }
  }
  const size_t point_light_count = point_light_info_blocks_.size();
  point_shadow_views_.resize(6u * point_light_count);
  for (uint32_t face = 0; face < 6u; ++face) {
    for (size_t light_index = 0; light_index < point_light_count; ++light_index) {
      if (point_light_info_blocks_[light_index].diffuse.w > 0.5f) {
        shadow_work_items.push_back({&point_shadow_views_[face * point_light_count + light_index],
                                     point_light_info_blocks_[light_index].light_space_matrix[face]});
      }
    }
  }
  spot_shadow_views_.resize(spot_light_info_blocks_.size());
  for (size_t light_index = 0; light_index < spot_light_info_blocks_.size(); ++light_index) {
    if (spot_light_info_blocks_[light_index].diffuse.w > 0.5f) {
      shadow_work_items.push_back(
          {&spot_shadow_views_[light_index], spot_light_info_blocks_[light_index].light_space_matrix});
    }
  }

  std::vector<CameraRasterVisibility> camera_results(camera_count);
  std::vector<ShadowViewIndirectCommands> shadow_results(shadow_work_items.size());
  const size_t work_item_count = camera_count + shadow_work_items.size();
  const auto build_item = [&](const size_t index) {
    if (index < camera_count) {
      camera_results[index] = BuildCameraRasterVisibilityResult(index);
    } else {
      const size_t shadow_index = index - camera_count;
      shadow_results[shadow_index] =
          BuildShadowViewIndirectCommands(shadow_work_items[shadow_index].light_space_matrix);
    }
  };
  if (work_item_count > 1u) {
    RunDedicatedRasterBatch(work_item_count, build_item);
  } else if (work_item_count == 1u) {
    build_item(0u);
  }

  for (size_t camera_index = 0; camera_index < camera_count; ++camera_index) {
    MergeCameraRasterVisibilityResult(camera_index, std::move(camera_results[camera_index]));
  }

  for (size_t shadow_index = 0; shadow_index < shadow_work_items.size(); ++shadow_index) {
    MergeShadowRasterVisibilityResult(*shadow_work_items[shadow_index].destination,
                                      std::move(shadow_results[shadow_index]), use_mesh_shader);
  }
  FinalizeShadowIndirectBuffers(use_mesh_shader);
}

bool RenderInstanceStorage::IsInstanceVisible(const int32_t camera_index, const int32_t instance_index) const {
  const auto* visibility = GetCameraRasterVisibility(camera_index);
  return !visibility || !visibility->enabled || instance_index < 0 ||
         static_cast<size_t>(instance_index) >= visibility->instance_visibility.size() ||
         visibility->instance_visibility[instance_index] != 0u;
}

const RenderInstanceStorage::CameraRasterVisibility* RenderInstanceStorage::GetCameraRasterVisibility(
    const int32_t camera_index) const {
  if (camera_index < 0 || static_cast<size_t>(camera_index) >= camera_raster_visibility_.size()) {
    return nullptr;
  }
  return &camera_raster_visibility_[camera_index];
}

const Bound* RenderInstanceStorage::FindPersistentWorldBound(const Handle renderer_handle, const GlobalTransform& model,
                                                             const Bound& local_bound,
                                                             const uint64_t content_signature) {
  if (renderer_handle == 0) {
    return nullptr;
  }
  persistent_transform_seen_.insert(renderer_handle);
  const auto found = persistent_transform_records_.find(renderer_handle);
  if (found == persistent_transform_records_.end() || found->second.model != model ||
      !BoundsEqual(found->second.local_bound, local_bound) || found->second.content_signature != content_signature) {
    return nullptr;
  }
  return &found->second.world_bound;
}

void RenderInstanceStorage::StorePersistentWorldBound(const Handle renderer_handle, const GlobalTransform& model,
                                                      const Bound& local_bound, const uint64_t content_signature,
                                                      const Bound& world_bound) {
  if (renderer_handle == 0) {
    return;
  }
  persistent_transform_seen_.insert(renderer_handle);
  persistent_transform_records_[renderer_handle] = {model, local_bound, world_bound, content_signature};
}

void RenderInstanceStorage::PrunePersistentTransformRecords() {
  for (auto iterator = persistent_transform_records_.begin(); iterator != persistent_transform_records_.end();) {
    if (persistent_transform_seen_.find(iterator->first) != persistent_transform_seen_.end()) {
      ++iterator;
      continue;
    }
    iterator = persistent_transform_records_.erase(iterator);
  }
}

void RenderInstanceStorage::PrepareStaticMeshCache(const std::shared_ptr<Scene>& scene) {
  const auto revision = scene->GetRenderStructureRevision();
  if (static_mesh_cache_scene_.lock() != scene || static_mesh_cache_structure_revision_ != revision) {
    static_mesh_render_instance_cache_.clear();
    static_mesh_cache_scene_ = scene;
    static_mesh_cache_structure_revision_ = revision;
  }
}

void RenderInstanceStorage::InvalidateStaticEntityCache(const std::shared_ptr<Scene>& scene, const Entity& entity) {
  if (!scene || static_mesh_cache_scene_.lock() != scene || !scene->IsEntityValid(entity))
    return;
  std::unordered_set<Entity, Entity> invalid_entities;
  scene->ForEachDescendant(
      entity,
      [&](const Entity& descendant) {
        invalid_entities.insert(descendant);
      },
      false);
  for (auto iterator = static_mesh_render_instance_cache_.begin();
       iterator != static_mesh_render_instance_cache_.end();) {
    const auto& record = iterator->second;
    if (invalid_entities.find(record.source_owner) != invalid_entities.end() ||
        (record.render_instance && invalid_entities.find(record.render_instance->owner) != invalid_entities.end())) {
      iterator = static_mesh_render_instance_cache_.erase(iterator);
    } else {
      ++iterator;
    }
  }
}

const GltfMaterialData& RenderInstanceStorage::ResolveMaterialData(const std::shared_ptr<Material>& material) {
  const auto handle = material->GetHandle();
  const auto version = material->GetVersion();
  if (const auto found = material_data_cache_.find(handle);
      found != material_data_cache_.end() && found->second.version == version) {
    return found->second.data;
  }
  auto& cached = material_data_cache_[handle];
  cached.version = version;
  cached.data = BuildMaterialGltfData(*material);
  return cached.data;
}

void RenderInstanceStorage::UpdateRasterSpatialIndex() {
  spatial_render_entries_.clear();
  spatial_always_visible_entries_.clear();
  raster_spatial_index_.BeginUpdate();
  const auto register_entry = [&](const std::shared_ptr<IRenderInstance>& render_instance,
                                  const SpatialRenderCategory category,
                                  const int32_t deferred_mesh_command_index = -1) {
    if (!render_instance) {
      return;
    }
    SpatialRenderEntry entry{render_instance, category, deferred_mesh_command_index};
    if (render_instance->renderer_handle == 0 || !IsFiniteBound(render_instance->world_bound)) {
      spatial_always_visible_entries_.emplace_back(std::move(entry));
      return;
    }
    spatial_render_entries_[render_instance->renderer_handle] = entry;
    raster_spatial_index_.Upsert(render_instance->renderer_handle, render_instance->world_bound);
  };
  int32_t deferred_mesh_command_index = 0;
  deferred_render_instances->ForEachRenderInstance([&](const auto& instance) {
    register_entry(instance, SpatialRenderCategory::Deferred, deferred_mesh_command_index++);
  });
  deferred_masked_render_instances->ForEachRenderInstance([&](const auto& instance) {
    register_entry(instance, SpatialRenderCategory::DeferredMasked, deferred_mesh_command_index++);
  });
  const auto register_collection = [&](const auto& collection, const SpatialRenderCategory category) {
    collection->ForEachRenderInstance([&](const auto& instance) {
      register_entry(instance, category);
    });
  };
  register_collection(deferred_skinned_render_instances, SpatialRenderCategory::Deferred);
  register_collection(deferred_instanced_render_instances, SpatialRenderCategory::Deferred);
  register_collection(deferred_strands_render_instances, SpatialRenderCategory::Deferred);
  register_collection(deferred_masked_skinned_render_instances, SpatialRenderCategory::DeferredMasked);
  register_collection(deferred_masked_instanced_render_instances, SpatialRenderCategory::DeferredMasked);
  register_collection(deferred_masked_strands_render_instances, SpatialRenderCategory::DeferredMasked);
  register_collection(forward_render_instances, SpatialRenderCategory::Forward);
  register_collection(forward_skinned_render_instances, SpatialRenderCategory::Forward);
  register_collection(forward_instanced_render_instances, SpatialRenderCategory::Forward);
  register_collection(forward_strands_render_instances, SpatialRenderCategory::Forward);
  register_collection(transparent_render_instances, SpatialRenderCategory::Transparent);
  register_collection(transparent_skinned_render_instances, SpatialRenderCategory::Transparent);
  register_collection(transparent_instanced_render_instances, SpatialRenderCategory::Transparent);
  register_collection(transparent_strands_render_instances, SpatialRenderCategory::Transparent);
  register_collection(gaussian_splat_render_instances, SpatialRenderCategory::Gaussian);
  raster_spatial_index_.EndUpdate();
}

std::vector<RenderInstanceStorage::SpatialRenderEntry> RenderInstanceStorage::QueryRasterSpatialEntries(
    const std::function<bool(const Bound&)>& intersects) const {
  std::vector<SpatialRenderEntry> result = spatial_always_visible_entries_;
  for (const auto handle : raster_spatial_index_.Query(intersects)) {
    if (const auto found = spatial_render_entries_.find(handle); found != spatial_render_entries_.end()) {
      result.emplace_back(found->second);
    }
  }
  std::sort(result.begin(), result.end(), [](const auto& left, const auto& right) {
    const auto left_index =
        left.render_instance ? left.render_instance->instance_index : std::numeric_limits<int32_t>::max();
    const auto right_index =
        right.render_instance ? right.render_instance->instance_index : std::numeric_limits<int32_t>::max();
    return left_index < right_index;
  });
  return result;
}

void RenderInstanceStorage::CollectEntityRenderers(const std::shared_ptr<Scene>& target_scene, Bound& world_bound) {
  auto& min_bound = world_bound.min;
  auto& max_bound = world_bound.max;
  geometry_storage_version = GeometryStorage::GetVersion();
  texture_storage_version = TextureStorage::GetVersion();
  PrepareStaticMeshCache(target_scene);
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

uint64_t RenderInstanceStorage::CalculateCanonicalStructureSignature() const {
  uint64_t signature = 0;
  uint64_t entry_count = 0;
  const auto append = [&](const std::shared_ptr<IRenderInstance>& instance, const uint64_t category) {
    if (!instance) {
      return;
    }
    entry_count++;
    signature = MixDdgiInventorySignature(signature, category);
    signature = MixDdgiInventorySignature(signature, static_cast<uint64_t>(instance->renderer_handle));
    signature = MixDdgiInventorySignature(signature, static_cast<uint64_t>(instance->entity_handle));
    signature = MixDdgiInventorySignature(signature, static_cast<uint64_t>(instance->command_type));
    signature = MixDdgiInventorySignature(signature, static_cast<uint32_t>(instance->material_index));
    signature = MixDdgiInventorySignature(signature, instance->geometry_version);
    signature = MixDdgiInventorySignature(signature, instance->cast_shadow ? 1u : 0u);
    signature = MixDdgiInventorySignature(signature, static_cast<uint64_t>(instance->cull_mode));
    signature = MixDdgiInventorySignature(signature, static_cast<uint64_t>(instance->polygon_mode));
    signature = MixDdgiInventorySignature(signature, glm::floatBitsToUint(instance->line_width));
    if (const auto mesh = std::dynamic_pointer_cast<MeshRenderInstance>(instance); mesh && mesh->mesh) {
      signature = MixDdgiInventorySignature(signature, mesh->mesh->GetHandle().GetValue());
      if (mesh->mesh->triangle_range_) {
        signature = MixDdgiInventorySignature(signature, mesh->mesh->triangle_range_->prev_frame_offset);
        signature = MixDdgiInventorySignature(signature, mesh->mesh->triangle_range_->prev_frame_index_count);
      }
      if (mesh->mesh->meshlet_range_) {
        signature = MixDdgiInventorySignature(signature, mesh->mesh->meshlet_range_->prev_frame_range);
      }
    } else if (const auto skinned = std::dynamic_pointer_cast<SkinnedMeshRenderInstance>(instance);
               skinned && skinned->skinned_mesh) {
      signature = MixDdgiInventorySignature(signature, skinned->skinned_mesh->GetHandle().GetValue());
      if (skinned->skinned_mesh->skinned_triangle_range_) {
        signature = MixDdgiInventorySignature(signature,
                                              skinned->skinned_mesh->skinned_triangle_range_->prev_frame_index_count);
      }
      if (skinned->skinned_mesh->skinned_meshlet_range_) {
        signature =
            MixDdgiInventorySignature(signature, skinned->skinned_mesh->skinned_meshlet_range_->prev_frame_range);
      }
    } else if (const auto instanced = std::dynamic_pointer_cast<InstancedRenderInstance>(instance); instanced) {
      signature = MixDdgiInventorySignature(signature, instanced->mesh ? instanced->mesh->GetHandle().GetValue() : 0u);
      signature = MixDdgiInventorySignature(signature, instanced->particle_info_list_version);
      signature = MixDdgiInventorySignature(
          signature, instanced->particle_infos ? instanced->particle_infos->PeekParticleInfoList().size() : 0u);
    } else if (const auto strands = std::dynamic_pointer_cast<StrandsRenderInstance>(instance);
               strands && strands->strands) {
      signature = MixDdgiInventorySignature(signature, strands->strands->GetHandle().GetValue());
      if (strands->strands->segment_range_) {
        signature = MixDdgiInventorySignature(signature, strands->strands->segment_range_->prev_frame_index_count);
      }
      if (strands->strands->strand_meshlet_range_) {
        signature = MixDdgiInventorySignature(signature, strands->strands->strand_meshlet_range_->prev_frame_range);
      }
    } else if (const auto gaussian = std::dynamic_pointer_cast<GaussianSplatRenderInstance>(instance); gaussian) {
      signature = MixDdgiInventorySignature(
          signature, gaussian->gaussian_splat ? gaussian->gaussian_splat->GetHandle().GetValue() : 0u);
      signature = MixDdgiInventorySignature(signature, static_cast<uint64_t>(gaussian->sort_mode));
      signature = MixDdgiInventorySignature(signature, static_cast<uint64_t>(gaussian->depth_mode));
      signature = MixDdgiInventorySignature(signature, static_cast<uint64_t>(gaussian->raster_mode));
    }
  };
  const auto append_collection = [&](const auto& collection, const uint64_t category) {
    collection->ForEachRenderInstance([&](const auto& instance) {
      append(instance, category);
    });
  };
  append_collection(deferred_render_instances, 0u);
  append_collection(deferred_skinned_render_instances, 1u);
  append_collection(deferred_instanced_render_instances, 2u);
  append_collection(deferred_strands_render_instances, 3u);
  append_collection(deferred_masked_render_instances, 4u);
  append_collection(deferred_masked_skinned_render_instances, 5u);
  append_collection(deferred_masked_instanced_render_instances, 6u);
  append_collection(deferred_masked_strands_render_instances, 7u);
  append_collection(forward_render_instances, 8u);
  append_collection(forward_skinned_render_instances, 9u);
  append_collection(forward_instanced_render_instances, 10u);
  append_collection(forward_strands_render_instances, 11u);
  append_collection(transparent_render_instances, 12u);
  append_collection(transparent_skinned_render_instances, 13u);
  append_collection(transparent_instanced_render_instances, 14u);
  append_collection(transparent_strands_render_instances, 15u);
  append_collection(gaussian_splat_render_instances, 16u);
  append_collection(external_render_instances, 17u);
  return MixDdgiInventorySignature(signature, entry_count);
}

void RenderInstanceStorage::BuildRenderInstanceBlocks() {
  const auto structure_signature = CalculateCanonicalStructureSignature();
  canonical_structure_changed_this_frame_ =
      !canonical_structure_initialized_ || structure_signature != canonical_structure_signature_;
  canonical_structure_initialized_ = true;
  canonical_structure_signature_ = structure_signature;
  raster_draw_instance_indices.clear();
  deferred_mesh_draw_instance_index_offset = 0;
  if (canonical_structure_changed_this_frame_) {
    deferred_mesh_indirect_batches.clear();
    deferred_masked_mesh_indirect_batches.clear();
    opaque_shadow_mesh_indirect_batches.clear();
    masked_shadow_mesh_indirect_batches.clear();
    mesh_draw_indexed_indirect_commands.clear();
    mesh_draw_mesh_tasks_indirect_commands.clear();
    shadow_mesh_draw_indexed_indirect_commands.clear();
    shadow_mesh_draw_mesh_tasks_indirect_commands.clear();
  }

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
    if (render_instance->entity_selected)
      entity_selection_render_snapshot_.Include(render_instance->world_bound);
    render_instance_block.world_bound_min = glm::vec4(render_instance->world_bound.min, 0.0f);
    render_instance_block.world_bound_max = glm::vec4(render_instance->world_bound.max, 0.0f);
    rigid_motion_supported_.emplace_back(rigid_motion_supported ? 1u : 0u);
  };
  const auto register_tlas_input =
      [&](const std::shared_ptr<IRenderInstance>& render_instance,
          const std::shared_ptr<BottomLevelAccelerationStructure>& bottom_level_acceleration_structure,
          const glm::mat4& model, const uint32_t custom_index, const bool linear_swept_spheres = false) {
        if (Platform::RayAccelerationStructureEnabled()) {
          top_level_acceleration_structure_inputs_.push_back(
              {render_instance, bottom_level_acceleration_structure, model, custom_index, linear_swept_spheres});
        }
      };
  const auto register_mesh_tlas_input = [&](const std::shared_ptr<MeshRenderInstance>& render_instance) {
    if (render_instance && render_instance->mesh) {
      register_tlas_input(
          render_instance,
          render_instance->ray_tracing_blas ? render_instance->ray_tracing_blas : render_instance->mesh->blas_,
          render_instance->model.value, static_cast<uint32_t>(render_instance->instance_index));
    }
  };
  const auto register_skinned_tlas_input = [&](const std::shared_ptr<SkinnedMeshRenderInstance>& render_instance) {
    if (render_instance && render_instance->skinned_mesh) {
      register_tlas_input(
          render_instance,
          render_instance->ray_tracing_blas ? render_instance->ray_tracing_blas : render_instance->skinned_mesh->blas_,
          render_instance->model.value, static_cast<uint32_t>(render_instance->instance_index));
    }
  };
  const auto register_strands_tlas_input = [&](const std::shared_ptr<StrandsRenderInstance>& render_instance) {
    if (Platform::RayTracingLinearSweptSpheresEnabled() && render_instance && render_instance->strands) {
      register_tlas_input(render_instance, render_instance->strands->blas_, render_instance->model.value,
                          static_cast<uint32_t>(render_instance->instance_index), true);
    }
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
  const auto register_shadow_mesh_indirect_command = [&](const std::shared_ptr<MeshRenderInstance>& render_instance,
                                                         std::vector<DeferredMeshIndirectBatch>& batches) {
    VkDrawIndexedIndirectCommand draw{};
    VkDrawMeshTasksIndirectCommandEXT mesh_task{};

    if (render_instance && render_instance->cast_shadow && render_instance->mesh) {
      const auto triangle_offset = render_instance->mesh->triangle_range_->prev_frame_offset;
      const auto triangle_index_count = render_instance->mesh->triangle_range_->prev_frame_index_count;
      const auto meshlet_range = render_instance->mesh->meshlet_range_->prev_frame_range;
      draw = CreateIndexedCommand(triangle_offset, triangle_index_count);
      mesh_task = CreateMeshTaskCommand(meshlet_range);
    }

    if (canonical_structure_changed_this_frame_) {
      const auto command_index = static_cast<uint32_t>(shadow_mesh_draw_indexed_indirect_commands.size());
      shadow_mesh_draw_indexed_indirect_commands.emplace_back(draw);
      shadow_mesh_draw_mesh_tasks_indirect_commands.emplace_back(mesh_task);
      const auto same_batch = [&](const DeferredMeshIndirectBatch& batch) {
        return batch.line_width == render_instance->line_width && batch.cull_mode == render_instance->cull_mode &&
               batch.polygon_mode == render_instance->polygon_mode;
      };
      if (batches.empty() || !same_batch(batches.back())) {
        auto& batch = batches.emplace_back();
        batch.first_command = command_index;
        batch.line_width = render_instance->line_width;
        batch.cull_mode = render_instance->cull_mode;
        batch.polygon_mode = render_instance->polygon_mode;
      }
      auto& batch = batches.back();
      batch.command_count++;
      batch.triangle_count += draw.indexCount / 3u;
    }
  };
  uint32_t deferred_mesh_command_index = 0;
  const auto register_deferred_mesh_indirect_batch = [&](const std::shared_ptr<MeshRenderInstance>& render_instance,
                                                         std::vector<DeferredMeshIndirectBatch>& batches) {
    if (!render_instance || !render_instance->mesh ||
        deferred_mesh_command_index >= mesh_draw_indexed_indirect_commands.size() ||
        deferred_mesh_command_index >= mesh_draw_mesh_tasks_indirect_commands.size()) {
      deferred_mesh_command_index++;
      return;
    }
    const auto same_batch = [&](const DeferredMeshIndirectBatch& batch) {
      return batch.line_width == render_instance->line_width && batch.cull_mode == render_instance->cull_mode &&
             batch.polygon_mode == render_instance->polygon_mode;
    };
    if (batches.empty() || !same_batch(batches.back())) {
      auto& batch = batches.emplace_back();
      batch.first_command = deferred_mesh_command_index;
      batch.line_width = render_instance->line_width;
      batch.cull_mode = render_instance->cull_mode;
      batch.polygon_mode = render_instance->polygon_mode;
    }
    auto& batch = batches.back();
    batch.command_count++;
    batch.triangle_count += render_instance->mesh->triangle_range_->prev_frame_index_count / 3u;
    deferred_mesh_command_index++;
  };
  const auto register_deferred_mesh_collection = [&](const std::shared_ptr<MeshRenderInstanceCollection>& collection,
                                                     std::vector<DeferredMeshIndirectBatch>& batches,
                                                     std::vector<DeferredMeshIndirectBatch>& shadow_batches) {
    collection->ForEachMeshRenderInstance([&](const auto& render_instance) {
      if (canonical_structure_changed_this_frame_ && render_instance && render_instance->mesh) {
        AppendMeshIndirectCommands(mesh_draw_indexed_indirect_commands, mesh_draw_mesh_tasks_indirect_commands,
                                   render_instance->mesh->triangle_range_->prev_frame_offset,
                                   render_instance->mesh->triangle_range_->prev_frame_index_count,
                                   render_instance->mesh->meshlet_range_->prev_frame_range);
      }
      register_render_instance(render_instance, true);
      register_mesh_tlas_input(render_instance);
      raster_draw_instance_indices.emplace_back(static_cast<uint32_t>(render_instance->instance_index));
      if (canonical_structure_changed_this_frame_) {
        register_deferred_mesh_indirect_batch(render_instance, batches);
      }
      register_shadow_mesh_indirect_command(render_instance, shadow_batches);
    });
  };
  register_deferred_mesh_collection(deferred_render_instances, deferred_mesh_indirect_batches,
                                    opaque_shadow_mesh_indirect_batches);
  register_deferred_mesh_collection(deferred_masked_render_instances, deferred_masked_mesh_indirect_batches,
                                    masked_shadow_mesh_indirect_batches);
  ValidateDeferredMeshIndirectCommandCount(
      CountRenderInstances(deferred_render_instances) + CountRenderInstances(deferred_masked_render_instances),
      mesh_draw_indexed_indirect_commands, mesh_draw_mesh_tasks_indirect_commands, raster_draw_instance_indices);
  deferred_skinned_render_instances->ForEachSkinnedMeshRenderInstance([&](const auto& render_instance) {
    register_render_instance(render_instance, false);
    register_skinned_tlas_input(render_instance);
  });
  deferred_instanced_render_instances->ForEachInstancedRenderInstance([&](const auto& render_instance) {
    register_render_instance(render_instance, false);
  });
  deferred_strands_render_instances->ForEachStrandsRenderInstance([&](const auto& render_instance) {
    register_render_instance(render_instance, false);
    register_strands_tlas_input(render_instance);
  });
  deferred_masked_skinned_render_instances->ForEachSkinnedMeshRenderInstance([&](const auto& render_instance) {
    register_render_instance(render_instance, false);
    register_skinned_tlas_input(render_instance);
  });
  deferred_masked_instanced_render_instances->ForEachInstancedRenderInstance([&](const auto& render_instance) {
    register_render_instance(render_instance, false);
  });
  deferred_masked_strands_render_instances->ForEachStrandsRenderInstance([&](const auto& render_instance) {
    register_render_instance(render_instance, false);
    register_strands_tlas_input(render_instance);
  });

  forward_render_instances->ForEachMeshRenderInstance([&](const auto& render_instance) {
    register_render_instance(render_instance, true);
    register_mesh_tlas_input(render_instance);
  });
  forward_skinned_render_instances->ForEachSkinnedMeshRenderInstance([&](const auto& render_instance) {
    register_render_instance(render_instance, false);
    register_skinned_tlas_input(render_instance);
  });
  forward_instanced_render_instances->ForEachInstancedRenderInstance([&](const auto& render_instance) {
    register_render_instance(render_instance, false);
  });
  forward_strands_render_instances->ForEachStrandsRenderInstance([&](const auto& render_instance) {
    register_render_instance(render_instance, false);
    register_strands_tlas_input(render_instance);
  });

  transparent_render_instances->ForEachMeshRenderInstance([&](const auto& render_instance) {
    register_render_instance(render_instance, true);
    register_mesh_tlas_input(render_instance);
  });
  transparent_skinned_render_instances->ForEachSkinnedMeshRenderInstance([&](const auto& render_instance) {
    register_render_instance(render_instance, false);
    register_skinned_tlas_input(render_instance);
  });
  transparent_instanced_render_instances->ForEachInstancedRenderInstance([&](const auto& render_instance) {
    register_render_instance(render_instance, false);
  });
  transparent_strands_render_instances->ForEachStrandsRenderInstance([&](const auto& render_instance) {
    register_render_instance(render_instance, false);
    register_strands_tlas_input(render_instance);
  });

  gaussian_splat_render_instances->ForEachGaussianSplatRenderInstance([&](const auto& render_instance) {
    prepare_gaussian_splat_render_instance(render_instance);
    register_render_instance(render_instance, false);
  });

  external_render_instances->ForEachExternalRenderInstance([&](const auto& render_instance) {
    register_render_instance(render_instance, false);
    if (render_instance && render_instance->HasDdgiRayTracingGeometry()) {
      register_tlas_input(render_instance, render_instance->ddgi_geometry.bottom_level_acceleration_structure,
                          render_instance->model.value, static_cast<uint32_t>(render_instance->instance_index));
    }
  });

  const auto append_particle_ray_instances = [&](const std::shared_ptr<InstancedRenderInstance>& render_instance) {
    render_instance->ray_tracing_instance_indices.clear();
    if (!Platform::RayAccelerationStructureEnabled() || !render_instance->mesh || !render_instance->particle_infos ||
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
      register_tlas_input(render_instance, render_instance->mesh->blas_, ray_instance_block.model.value,
                          ray_instance_index);
      if (render_instance->entity_handle != 0) {
        instance_entity_handles_[ray_instance_index] = render_instance->entity_handle;
      }
      if (render_instance->renderer_handle != 0) {
        instance_renderer_handles_[ray_instance_index] = render_instance->renderer_handle;
      }
    }
  };
  deferred_instanced_render_instances->ForEachInstancedRenderInstance(append_particle_ray_instances);
  deferred_masked_instanced_render_instances->ForEachInstancedRenderInstance(append_particle_ray_instances);
  forward_instanced_render_instances->ForEachInstancedRenderInstance(append_particle_ray_instances);
  transparent_instanced_render_instances->ForEachInstancedRenderInstance(append_particle_ray_instances);
}

void RenderInstanceStorage::BuildEmissiveTriangleInfoBlocks() {
  const auto build_started = std::chrono::steady_clock::now();
  if (!Platform::RayAccelerationStructureEnabled()) {
    emissive_instance_info_dirty_ = !emissive_instance_info_blocks_.empty();
    emissive_triangle_distribution_info_dirty_ = !emissive_triangle_distribution_info_blocks_.empty();
    emissive_triangle_info_dirty_ = !emissive_triangle_info_blocks_.empty();
    emissive_instance_info_blocks_.clear();
    emissive_triangle_distribution_info_blocks_.clear();
    emissive_triangle_info_blocks_.clear();
    emissive_triangle_instance_signatures_.clear();
    emissive_sampling_signature_ = 0;
    ddgi_emissive_inventory_stats_ = {};
    ddgi_emissive_inventory_stats_.build_ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - build_started).count();
    ddgi_emissive_inventory_signature_ = 0;
    render_info_block.emissive_triangle_parameters = glm::uvec4(0u);
    return;
  }

  struct EmissiveTriangleInstance {
    std::shared_ptr<IRenderInstance> render_instance;
    uint64_t mesh_handle;
    uint64_t material_handle;
    uint64_t emissive_sampling_signature;
    uint32_t geometry_version;
    uint32_t instance_index;
    uint32_t triangle_offset;
    uint32_t triangle_count;
    glm::mat4 model;
    double sidedness;
    double area_scale;
    bool unique_distribution;
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
                                   const bool has_ray_tracing_geometry, const bool deforming_geometry) {
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
    const glm::mat3 linear_transform(model);
    const glm::vec3 column_lengths(glm::length(linear_transform[0]), glm::length(linear_transform[1]),
                                   glm::length(linear_transform[2]));
    const float maximum_length = glm::max(column_lengths.x, glm::max(column_lengths.y, column_lengths.z));
    const float tolerance = glm::max(1.0f, maximum_length) * 1.0e-4f;
    const bool equal_lengths =
        glm::all(glm::lessThanEqual(glm::abs(column_lengths - glm::vec3(column_lengths.x)), glm::vec3(tolerance)));
    const float orthogonality_tolerance = glm::max(1.0f, maximum_length * maximum_length) * 1.0e-4f;
    const bool orthogonal = std::abs(glm::dot(linear_transform[0], linear_transform[1])) <= orthogonality_tolerance &&
                            std::abs(glm::dot(linear_transform[0], linear_transform[2])) <= orthogonality_tolerance &&
                            std::abs(glm::dot(linear_transform[1], linear_transform[2])) <= orthogonality_tolerance;
    const bool similarity_transform = equal_lengths && orthogonal && column_lengths.x > 0.0f;
    emissive_instances.push_back({render_instance, mesh_handle, render_instance->material->GetHandle().GetValue(),
                                  emissive_sampling_signature, render_instance->geometry_version, instance_index,
                                  triangle_offset, triangle_count, model, sidedness,
                                  similarity_transform ? static_cast<double>(column_lengths.x) * column_lengths.x : 1.0,
                                  deforming_geometry || !similarity_transform});
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
                                                                          : render_instance->mesh->blas_),
                      false);
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
                                                                              : render_instance->skinned_mesh->blas_),
                          true);
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
                        static_cast<bool>(render_instance->mesh->blas_), false);
      }
    });
  };
  append_mesh_collection(deferred_render_instances);
  append_skinned_collection(deferred_skinned_render_instances);
  append_instanced_collection(deferred_instanced_render_instances);
  append_mesh_collection(deferred_masked_render_instances);
  append_skinned_collection(deferred_masked_skinned_render_instances);
  append_instanced_collection(deferred_masked_instanced_render_instances);
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
                        render_instance->ddgi_geometry.triangle_count, render_instance->model.value, true, true);
      });

  std::sort(emissive_instances.begin(), emissive_instances.end(), [](const auto& lhs, const auto& rhs) {
    return lhs.instance_index < rhs.instance_index;
  });
  std::sort(signatures.begin(), signatures.end(), [](const auto& lhs, const auto& rhs) {
    return lhs.instance_index < rhs.instance_index;
  });

  uint64_t sampling_signature = emissive_instances.size();
  for (const auto& instance : emissive_instances) {
    sampling_signature = MixDdgiInventorySignature(sampling_signature, instance.mesh_handle);
    sampling_signature = MixDdgiInventorySignature(sampling_signature, instance.material_handle);
    sampling_signature = MixDdgiInventorySignature(sampling_signature, instance.emissive_sampling_signature);
    sampling_signature = MixDdgiInventorySignature(sampling_signature, instance.geometry_version);
    sampling_signature = MixDdgiInventorySignature(sampling_signature, instance.instance_index);
    sampling_signature = MixDdgiInventorySignature(sampling_signature, instance.triangle_offset);
    sampling_signature = MixDdgiInventorySignature(sampling_signature, instance.triangle_count);
    uint64_t area_scale_bits = 0;
    std::memcpy(&area_scale_bits, &instance.area_scale, sizeof(area_scale_bits));
    sampling_signature = MixDdgiInventorySignature(sampling_signature, area_scale_bits);
    sampling_signature = MixDdgiInventorySignature(sampling_signature, instance.unique_distribution);
    if (instance.unique_distribution) {
      for (int column = 0; column < 4; ++column) {
        for (int row = 0; row < 4; ++row) {
          sampling_signature =
              MixDdgiInventorySignature(sampling_signature, glm::floatBitsToUint(instance.model[column][row]));
        }
      }
    }
  }
  if (sampling_signature == emissive_sampling_signature_) {
    inventory_stats.distribution_count = ddgi_emissive_inventory_stats_.distribution_count;
    inventory_stats.fallback_distribution_count = ddgi_emissive_inventory_stats_.fallback_distribution_count;
    inventory_stats.logical_triangle_count = ddgi_emissive_inventory_stats_.logical_triangle_count;
    inventory_stats.stored_triangle_count = ddgi_emissive_inventory_stats_.stored_triangle_count;
    inventory_stats.unrepresentable_probability_count =
        ddgi_emissive_inventory_stats_.unrepresentable_probability_count;
    inventory_stats.estimated_emitted_power = ddgi_emissive_inventory_stats_.estimated_emitted_power;
    inventory_stats.upload_ms = 0.0;
    inventory_stats.build_ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - build_started).count();
    ddgi_emissive_inventory_stats_ = inventory_stats;
    emissive_triangle_instance_signatures_ = std::move(signatures);
    ddgi_emissive_inventory_signature_ = HashDdgiEmissiveInventorySignature(emissive_triangle_instance_signatures_);
    render_info_block.emissive_triangle_parameters =
        glm::uvec4(static_cast<uint32_t>(emissive_instance_info_blocks_.size()),
                   static_cast<uint32_t>(emissive_triangle_distribution_info_blocks_.size()),
                   static_cast<uint32_t>(emissive_triangle_info_blocks_.size()),
                   static_cast<uint32_t>(glm::min<uint64_t>(inventory_stats.logical_triangle_count, UINT32_MAX)));
    return;
  }

  struct DistributionKey {
    uint64_t mesh_handle;
    uint64_t material_handle;
    uint64_t sampling_signature;
    uint32_t geometry_version;
    uint32_t triangle_offset;
    uint32_t triangle_count;
    uint32_t unique_instance;

    bool operator==(const DistributionKey& other) const {
      return mesh_handle == other.mesh_handle && material_handle == other.material_handle &&
             sampling_signature == other.sampling_signature && geometry_version == other.geometry_version &&
             triangle_offset == other.triangle_offset && triangle_count == other.triangle_count &&
             unique_instance == other.unique_instance;
    }
  };
  struct DistributionKeyHash {
    size_t operator()(const DistributionKey& key) const {
      uint64_t hash = MixDdgiInventorySignature(key.mesh_handle, key.material_handle);
      hash = MixDdgiInventorySignature(hash, key.sampling_signature);
      hash = MixDdgiInventorySignature(hash, key.geometry_version);
      hash = MixDdgiInventorySignature(hash, key.triangle_offset);
      hash = MixDdgiInventorySignature(hash, key.triangle_count);
      return static_cast<size_t>(MixDdgiInventorySignature(hash, key.unique_instance));
    }
  };
  struct DistributionBuild {
    std::vector<EmissiveTriangleInfoBlock> triangles;
    double power = 0.0;
  };

  std::unordered_map<DistributionKey, uint32_t, DistributionKeyHash> distribution_indices;
  std::vector<DistributionBuild> distributions;
  std::vector<uint32_t> instance_distribution_indices;
  std::vector<double> instance_power_weights;
  std::vector<double> instance_uniform_weights;
  instance_distribution_indices.reserve(emissive_instances.size());
  instance_power_weights.reserve(emissive_instances.size());
  instance_uniform_weights.reserve(emissive_instances.size());

  for (const auto& emissive_instance : emissive_instances) {
    const DistributionKey key{emissive_instance.mesh_handle,
                              emissive_instance.material_handle,
                              emissive_instance.emissive_sampling_signature,
                              emissive_instance.geometry_version,
                              emissive_instance.triangle_offset,
                              emissive_instance.triangle_count,
                              emissive_instance.unique_distribution ? emissive_instance.instance_index : UINT32_MAX};
    if (distributions.size() >= UINT32_MAX) {
      throw std::runtime_error("Emissive triangle distribution count exceeds the GPU index range.");
    }
    auto [iterator, inserted] = distribution_indices.emplace(key, static_cast<uint32_t>(distributions.size()));
    if (inserted) {
      std::vector<EmissiveTriangleCandidate> candidates;
      candidates.reserve(emissive_instance.triangle_count);
      double power = 0.0;
      const auto& material = shade_materials[emissive_instance.render_instance->material_index];
      for (uint32_t primitive_id = 0; primitive_id < emissive_instance.triangle_count; ++primitive_id) {
        const auto& triangle = GeometryStorage::PeekTriangle(emissive_instance.triangle_offset + primitive_id);
        const auto& v0 = GeometryStorage::PeekVertex(triangle.x);
        const auto& v1 = GeometryStorage::PeekVertex(triangle.y);
        const auto& v2 = GeometryStorage::PeekVertex(triangle.z);
        glm::vec3 p0 = v0.position;
        glm::vec3 p1 = v1.position;
        glm::vec3 p2 = v2.position;
        if (emissive_instance.unique_distribution) {
          p0 = glm::vec3(emissive_instance.model * glm::vec4(p0, 1.0f));
          p1 = glm::vec3(emissive_instance.model * glm::vec4(p1, 1.0f));
          p2 = glm::vec3(emissive_instance.model * glm::vec4(p2, 1.0f));
        }
        const double area = 0.5 * static_cast<double>(glm::length(glm::cross(p1 - p0, p2 - p0)));
        const double importance =
            EstimateTriangleEmissiveImportance(*emissive_instance.render_instance->material, material, v0, v1, v2) *
            EstimateTriangleOpacityImportance(*emissive_instance.render_instance->material, material, v0, v1, v2) *
            emissive_instance.sidedness;
        if (std::isfinite(area) && area > 0.0 && std::isfinite(importance) && importance > 0.0) {
          candidates.push_back({primitive_id, area, importance});
          power += area * importance;
        }
      }
      distributions.push_back({BuildEmissiveTriangleDistribution(std::move(candidates)), power});
    }
    const auto distribution_index = iterator->second;
    const auto& distribution = distributions[distribution_index];
    instance_distribution_indices.push_back(distribution_index);
    inventory_stats.fallback_distribution_count += inserted && emissive_instance.unique_distribution ? 1u : 0u;
    const double scale = emissive_instance.unique_distribution ? 1.0 : emissive_instance.area_scale;
    instance_power_weights.push_back(distribution.power * scale);
    instance_uniform_weights.push_back(static_cast<double>(distribution.triangles.size()));
    inventory_stats.estimated_emitted_power += glm::pi<double>() * distribution.power * scale;
    inventory_stats.logical_triangle_count += distribution.triangles.size();
  }

  const auto power_alias = BuildEmissiveAliasTable(instance_power_weights);
  const auto uniform_alias = BuildEmissiveAliasTable(instance_uniform_weights);
  std::vector<EmissiveInstanceInfoBlock> instance_blocks;
  instance_blocks.reserve(emissive_instances.size());
  if (power_alias.size() == emissive_instances.size() && uniform_alias.size() == emissive_instances.size()) {
    for (size_t index = 0; index < emissive_instances.size(); ++index) {
      instance_blocks.push_back({emissive_instances[index].instance_index, instance_distribution_indices[index],
                                 power_alias[index].alias_probability, power_alias[index].alias_index,
                                 uniform_alias[index].alias_probability, uniform_alias[index].alias_index,
                                 power_alias[index].selection_probability, uniform_alias[index].selection_probability});
    }
  }

  std::vector<EmissiveTriangleDistributionInfoBlock> distribution_blocks;
  std::vector<EmissiveTriangleInfoBlock> triangle_blocks;
  distribution_blocks.reserve(distributions.size());
  for (const auto& distribution : distributions) {
    if (distribution.triangles.size() > UINT32_MAX ||
        triangle_blocks.size() > UINT32_MAX - distribution.triangles.size()) {
      throw std::runtime_error("Stored emissive triangle count exceeds the GPU index range.");
    }
    distribution_blocks.push_back(
        {static_cast<uint32_t>(triangle_blocks.size()), static_cast<uint32_t>(distribution.triangles.size())});
    triangle_blocks.insert(triangle_blocks.end(), distribution.triangles.begin(), distribution.triangles.end());
  }
  const auto instance_equal = [](const auto& lhs, const auto& rhs) {
    return lhs.instance_index == rhs.instance_index && lhs.distribution_index == rhs.distribution_index &&
           lhs.power_alias_probability == rhs.power_alias_probability &&
           lhs.power_alias_index == rhs.power_alias_index &&
           lhs.uniform_alias_probability == rhs.uniform_alias_probability &&
           lhs.uniform_alias_index == rhs.uniform_alias_index &&
           lhs.power_selection_probability == rhs.power_selection_probability &&
           lhs.uniform_selection_probability == rhs.uniform_selection_probability;
  };
  const auto distribution_equal = [](const auto& lhs, const auto& rhs) {
    return lhs.triangle_offset == rhs.triangle_offset && lhs.triangle_count == rhs.triangle_count;
  };
  const auto triangle_equal = [](const auto& lhs, const auto& rhs) {
    return lhs.primitive_id == rhs.primitive_id && lhs.alias_probability == rhs.alias_probability &&
           lhs.alias_index == rhs.alias_index && lhs.selection_probability == rhs.selection_probability;
  };
  const auto vectors_equal = [](const auto& lhs, const auto& rhs, const auto& predicate) {
    return lhs.size() == rhs.size() && std::equal(lhs.begin(), lhs.end(), rhs.begin(), predicate);
  };
  emissive_instance_info_dirty_ = !vectors_equal(emissive_instance_info_blocks_, instance_blocks, instance_equal);
  emissive_triangle_distribution_info_dirty_ =
      !vectors_equal(emissive_triangle_distribution_info_blocks_, distribution_blocks, distribution_equal);
  emissive_triangle_info_dirty_ = !vectors_equal(emissive_triangle_info_blocks_, triangle_blocks, triangle_equal);
  emissive_instance_info_blocks_ = std::move(instance_blocks);
  emissive_triangle_distribution_info_blocks_ = std::move(distribution_blocks);
  emissive_triangle_info_blocks_ = std::move(triangle_blocks);

  inventory_stats.distribution_count = static_cast<uint32_t>(emissive_triangle_distribution_info_blocks_.size());
  inventory_stats.stored_triangle_count = emissive_triangle_info_blocks_.size();
  inventory_stats.unrepresentable_probability_count = static_cast<uint32_t>(std::count_if(
      emissive_triangle_info_blocks_.begin(), emissive_triangle_info_blocks_.end(), [](const auto& record) {
        return record.selection_probability <= 0.0f;
      }));
  inventory_stats.build_ms =
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - build_started).count();
  ddgi_emissive_inventory_stats_ = inventory_stats;
  emissive_triangle_instance_signatures_ = std::move(signatures);
  emissive_sampling_signature_ = sampling_signature;
  ddgi_emissive_inventory_signature_ = HashDdgiEmissiveInventorySignature(emissive_triangle_instance_signatures_);
  render_info_block.emissive_triangle_parameters =
      glm::uvec4(static_cast<uint32_t>(emissive_instance_info_blocks_.size()),
                 static_cast<uint32_t>(emissive_triangle_distribution_info_blocks_.size()),
                 static_cast<uint32_t>(emissive_triangle_info_blocks_.size()),
                 static_cast<uint32_t>(glm::min<uint64_t>(inventory_stats.logical_triangle_count, UINT32_MAX)));
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
  buffer_create_info.size = sizeof(uint32_t) * Platform::Constants::initial_instance_size;
  raster_draw_instance_indices_buffer = std::make_shared<Buffer>(buffer_create_info, buffer_vma_allocation_create_info);
  buffer_create_info.size =
      glm::max(static_cast<size_t>(1), sizeof(PreviousInstanceInfoBlock) * Platform::Constants::initial_instance_size);
  previous_instance_info_descriptor_buffer =
      std::make_shared<Buffer>(buffer_create_info, buffer_vma_allocation_create_info);
  buffer_create_info.size = sizeof(EmissiveInstanceInfoBlock);
  emissive_instance_info_descriptor_buffer =
      std::make_shared<Buffer>(buffer_create_info, buffer_vma_allocation_create_info);
  buffer_create_info.size = sizeof(EmissiveTriangleDistributionInfoBlock);
  emissive_triangle_distribution_descriptor_buffer =
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

  buffer_create_info.size = glm::max(
      static_cast<size_t>(1), sizeof(VkDrawIndexedIndirectCommand) * shadow_mesh_draw_indexed_indirect_commands.size());
  shadow_mesh_draw_indexed_indirect_commands_buffer =
      std::make_shared<Buffer>(buffer_create_info, buffer_vma_allocation_create_info);

  buffer_create_info.size = glm::max(static_cast<size_t>(1), sizeof(VkDrawMeshTasksIndirectCommandEXT) *
                                                                 shadow_mesh_draw_mesh_tasks_indirect_commands.size());
  shadow_mesh_draw_mesh_tasks_indirect_commands_buffer =
      std::make_shared<Buffer>(buffer_create_info, buffer_vma_allocation_create_info);
  packed_shadow_indirect_buffer = CreateIndirectBuffer();
  packed_camera_indexed_buffer = CreateIndirectBuffer();
  packed_camera_mesh_task_buffer = CreateIndirectBuffer();

  deferred_render_instances = std::make_shared<MeshRenderInstanceCollection>();
  deferred_skinned_render_instances = std::make_shared<SkinnedMeshRenderInstanceCollection>();
  deferred_instanced_render_instances = std::make_shared<InstancedRenderInstanceCollection>();
  deferred_strands_render_instances = std::make_shared<StrandsRenderInstanceCollection>();

  deferred_masked_render_instances = std::make_shared<MeshRenderInstanceCollection>();
  deferred_masked_skinned_render_instances = std::make_shared<SkinnedMeshRenderInstanceCollection>();
  deferred_masked_instanced_render_instances = std::make_shared<InstancedRenderInstanceCollection>();
  deferred_masked_strands_render_instances = std::make_shared<StrandsRenderInstanceCollection>();

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
  total_skinned_mesh_triangles = 0;
  total_instanced_mesh_triangles = 0;
  total_strands_segments = 0;
  total_gaussian_splats = 0;
  directional_shadow_views_.clear();
  point_shadow_views_.clear();
  spot_shadow_views_.clear();
  directional_shadow_light_count_ = 0;
  persistent_transform_seen_.clear();
  active_material_handles_.clear();
  canonical_structure_changed_this_frame_ = false;
  material_cache_changed_this_frame_ = false;
  spatial_render_entries_.clear();
  spatial_always_visible_entries_.clear();

  deferred_render_instances->Clear();
  deferred_skinned_render_instances->Clear();
  deferred_instanced_render_instances->Clear();
  deferred_strands_render_instances->Clear();
  deferred_masked_render_instances->Clear();
  deferred_masked_skinned_render_instances->Clear();
  deferred_masked_instanced_render_instances->Clear();
  deferred_masked_strands_render_instances->Clear();
  forward_render_instances->Clear();
  forward_skinned_render_instances->Clear();
  forward_instanced_render_instances->Clear();
  forward_strands_render_instances->Clear();
  transparent_render_instances->Clear();
  transparent_skinned_render_instances->Clear();
  transparent_instanced_render_instances->Clear();
  transparent_strands_render_instances->Clear();
  gaussian_splat_render_instances->Clear();
  external_render_instances->Clear();
  top_level_acceleration_structure_inputs_.clear();

  instance_entity_handles_.clear();
  instance_renderer_handles_.clear();

  renderer_indices_.clear();
  camera_indices_.clear();
  render_settings = {};

  camera_info_blocks_.clear();
  for (auto& visibility : camera_raster_visibility_) {
    visibility.enabled = false;
    visibility.draw_instance_index_offset = 0;
    visibility.instance_visibility.clear();
    visibility.deferred_mesh_indirect_batches.clear();
    visibility.deferred_masked_mesh_indirect_batches.clear();
    visibility.mesh_draw_indexed_indirect_commands.clear();
    visibility.mesh_draw_mesh_tasks_indirect_commands.clear();
  }
  instance_info_blocks_.clear();
  previous_instance_info_blocks_.clear();
  rigid_motion_supported_.clear();
  directional_light_info_blocks_.clear();
  point_light_info_blocks_.clear();
  spot_light_info_blocks_.clear();
  render_info_block = {};

  cameras.clear();

  deferred_mesh_draw_instance_index_offset = 0;
  raster_draw_instance_indices.clear();
}

void RenderInstanceStorage::Upload(const bool immediate) {
  if (!Platform::Initialized())
    return;
  const ProfilerScope profiler_scope("RenderInstanceStorage::Upload", "Render");
  const auto instance_upload_ranges =
      PlanInstanceInfoUploadRanges(uploaded_instance_info_blocks_, instance_info_blocks_);
  const auto previous_instance_upload_ranges =
      PlanPreviousInstanceInfoUploadRanges(uploaded_previous_instance_info_blocks_, previous_instance_info_blocks_);
  const BufferUploadOptions uniform_options{BufferUploadUsage::Uniform, BufferUploadCapacityPolicy::GrowIfNeeded};
  const BufferUploadOptions storage_read_options{BufferUploadUsage::StorageRead,
                                                 BufferUploadCapacityPolicy::GrowIfNeeded};
  const BufferUploadOptions indirect_options{BufferUploadUsage::Indirect, BufferUploadCapacityPolicy::GrowIfNeeded};
  const BufferUploadOptions storage_indirect_options{
      BufferUploadUsage::Custom, BufferUploadCapacityPolicy::GrowIfNeeded, VK_PIPELINE_STAGE_2_ALL_COMMANDS_BIT,
      VK_ACCESS_2_SHADER_STORAGE_READ_BIT | VK_ACCESS_2_SHADER_STORAGE_WRITE_BIT |
          VK_ACCESS_2_INDIRECT_COMMAND_READ_BIT};
  BufferUploadBatch upload_batch;
  std::vector<std::pair<const Buffer*, uint64_t>> pending_signatures;
  const auto add_if_changed = [&](const std::shared_ptr<Buffer>& buffer, const void* data, const size_t size,
                                  const BufferUploadOptions& options) {
    if (size == 0) {
      return;
    }
    const auto signature = ByteSignature(data, size);
    if (const auto found = uploaded_payload_signatures_.find(buffer.get());
        found != uploaded_payload_signatures_.end() && found->second == signature) {
      return;
    }
    upload_batch.Add(buffer, data, size, 0, options);
    pending_signatures.emplace_back(buffer.get(), signature);
  };
  const auto add_vector_if_changed = [&](const std::shared_ptr<Buffer>& buffer, const auto& values,
                                         const BufferUploadOptions& options) {
    add_if_changed(buffer, values.data(), VectorBytes(values), options);
  };
  add_vector_if_changed(camera_info_descriptor_buffer, camera_info_blocks_, storage_read_options);
  if (material_cache_changed_this_frame_) {
    upload_batch.AddVector(gltf_material_descriptor_buffer, gltf_material_cache_.GetShadeMaterials(),
                           storage_read_options);
    upload_batch.AddVector(gltf_texture_info_descriptor_buffer, gltf_material_cache_.GetTextureInfos(),
                           storage_read_options);
  }
  AddUploadRanges(upload_batch, instance_info_descriptor_buffer, instance_info_blocks_, instance_upload_ranges,
                  storage_read_options);
  AddUploadRanges(upload_batch, previous_instance_info_descriptor_buffer, previous_instance_info_blocks_,
                  previous_instance_upload_ranges, storage_read_options);
  add_vector_if_changed(raster_draw_instance_indices_buffer, raster_draw_instance_indices, storage_read_options);
  const bool upload_emissive =
      emissive_instance_info_dirty_ || emissive_triangle_distribution_info_dirty_ || emissive_triangle_info_dirty_;
  const auto emissive_upload_started = std::chrono::steady_clock::now();
  if (emissive_instance_info_dirty_) {
    upload_batch.AddVector(emissive_instance_info_descriptor_buffer, emissive_instance_info_blocks_,
                           storage_read_options);
  }
  if (emissive_triangle_distribution_info_dirty_) {
    upload_batch.AddVector(emissive_triangle_distribution_descriptor_buffer,
                           emissive_triangle_distribution_info_blocks_, storage_read_options);
  }
  if (emissive_triangle_info_dirty_) {
    upload_batch.AddVector(emissive_triangle_info_descriptor_buffer, emissive_triangle_info_blocks_,
                           storage_read_options);
  }
  add_if_changed(render_info_descriptor_buffer, &render_info_block, sizeof(render_info_block), uniform_options);
  add_vector_if_changed(directional_light_info_descriptor_buffer, directional_light_info_blocks_, storage_read_options);
  add_vector_if_changed(point_light_info_descriptor_buffer, point_light_info_blocks_, storage_read_options);
  add_vector_if_changed(spot_light_info_descriptor_buffer, spot_light_info_blocks_, storage_read_options);

  if (canonical_structure_changed_this_frame_) {
    upload_batch.AddVector(mesh_draw_indexed_indirect_commands_buffer, mesh_draw_indexed_indirect_commands,
                           storage_indirect_options);
    upload_batch.AddVector(mesh_draw_mesh_tasks_indirect_commands_buffer, mesh_draw_mesh_tasks_indirect_commands,
                           storage_indirect_options);
    upload_batch.AddVector(shadow_mesh_draw_indexed_indirect_commands_buffer,
                           shadow_mesh_draw_indexed_indirect_commands, storage_indirect_options);
    upload_batch.AddVector(shadow_mesh_draw_mesh_tasks_indirect_commands_buffer,
                           shadow_mesh_draw_mesh_tasks_indirect_commands, storage_indirect_options);
  }
  add_vector_if_changed(packed_camera_indexed_buffer, packed_camera_indexed_commands, indirect_options);
  add_vector_if_changed(packed_camera_mesh_task_buffer, packed_camera_mesh_task_commands, indirect_options);
  if (packed_shadow_uses_mesh_shader_) {
    add_vector_if_changed(packed_shadow_indirect_buffer, packed_shadow_mesh_task_commands, indirect_options);
  } else {
    add_vector_if_changed(packed_shadow_indirect_buffer, packed_shadow_indexed_commands, indirect_options);
  }
  add_if_changed(environment_info_descriptor_buffer, &environment_info_block, sizeof(environment_info_block),
                 uniform_options);
  if (immediate) {
    upload_batch.SubmitImmediate();
  } else {
    upload_batch.Record(upload_arena_);
  }
  for (const auto& [buffer, signature] : pending_signatures) {
    uploaded_payload_signatures_[buffer] = signature;
  }
  ddgi_emissive_inventory_stats_.upload_ms =
      upload_emissive
          ? std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - emissive_upload_started)
                .count()
          : 0.0;
  CommitUploadedRanges(uploaded_instance_info_blocks_, instance_info_blocks_, instance_upload_ranges);
  CommitUploadedRanges(uploaded_previous_instance_info_blocks_, previous_instance_info_blocks_,
                       previous_instance_upload_ranges);
  emissive_instance_info_dirty_ = false;
  emissive_triangle_distribution_info_dirty_ = false;
  emissive_triangle_info_dirty_ = false;
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

const std::vector<RenderInstanceStorage::InstanceInfoBlock>& RenderInstanceStorage::GetInstanceInfoBlocks() const {
  return instance_info_blocks_;
}

const std::vector<RenderInstanceStorage::PreviousInstanceInfoBlock>&
RenderInstanceStorage::GetPreviousInstanceInfoBlocks() const {
  return previous_instance_info_blocks_;
}

std::vector<RenderInstanceStorage::InstanceUploadRange> RenderInstanceStorage::PlanInstanceInfoUploadRanges(
    const std::vector<InstanceInfoBlock>& previous, const std::vector<InstanceInfoBlock>& current) {
  return PlanUploadRanges(previous, current, [](const auto& left, const auto& right) {
    return left.info_index == right.info_index && !(left != right);
  });
}

std::vector<RenderInstanceStorage::InstanceUploadRange> RenderInstanceStorage::PlanPreviousInstanceInfoUploadRanges(
    const std::vector<PreviousInstanceInfoBlock>& previous, const std::vector<PreviousInstanceInfoBlock>& current) {
  return PlanUploadRanges(previous, current, [](const auto& left, const auto& right) {
    return left.previous_model == right.previous_model && left.flags == right.flags;
  });
}

void RenderInstanceStorage::BuildPreviousInstanceInfoBlocks(
    const std::shared_ptr<RenderInstanceStorage>& previous_render_instances) {
  const ProfilerScope profiler_scope("RenderInstanceStorage::BuildPreviousInstanceInfoBlocks", "Render");
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

bool RenderInstanceStorage::RegisterStrandsDrawCommand(const std::shared_ptr<Strands>& strands,
                                                       const std::shared_ptr<Material>& material,
                                                       const GlobalTransform& model, const bool cast_shadow) {
  if (!material || !strands || !Platform::MeshShaderEnabled() || !strands->strand_meshlet_range_ ||
      !strands->segment_range_ || strands->segment_range_->prev_frame_index_count == 0 ||
      strands->strand_meshlet_range_->prev_frame_range == 0)
    return false;
  auto bound = strands->GetBound();
  bound.ApplyTransform(model.value);
  const auto& material_data = ResolveMaterialData(material);
  const auto instance = std::make_shared<StrandsRenderInstance>();
  instance->command_type = RenderInstanceType::FromApi;
  instance->owner = Entity();
  instance->entity_handle = 0;
  instance->renderer_handle = 0;
  instance->strands = strands;
  instance->material = material;
  instance->model = model;
  instance->cast_shadow = cast_shadow;
  instance->world_bound = bound;
  instance->geometry_version = strands->GetVersion();
  instance->material_version = material->GetVersion();
  instance->material_index = RegisterMaterial(material, material_data);
  instance->line_width = material->draw_settings.line_width;
  instance->cull_mode = ResolveCullModeForTransform(material->draw_settings.cull_mode, model.value);
  instance->polygon_mode = material->draw_settings.polygon_mode;
  instance->entity_selected = false;
  switch (ResolveRasterMaterialClass(*material, material_data.shade_material)) {
    case GltfRasterMaterialClass::Forward:
      transparent_strands_render_instances->Register(instance);
      break;
    case GltfRasterMaterialClass::Masked:
      deferred_masked_strands_render_instances->Register(instance);
      break;
    case GltfRasterMaterialClass::Opaque:
      deferred_strands_render_instances->Register(instance);
      break;
  }
  total_strands_segments += strands->segment_range_->prev_frame_index_count;
  return true;
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
  const auto& material_data = ResolveMaterialData(material);
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
  switch (ResolveRasterMaterialClass(*material, material_data.shade_material)) {
    case GltfRasterMaterialClass::Forward:
      transparent_render_instances->Register(render_instance);
      break;
    case GltfRasterMaterialClass::Masked:
      deferred_masked_render_instances->Register(render_instance);
      break;
    case GltfRasterMaterialClass::Opaque:
      deferred_render_instances->Register(render_instance);
      break;
  }

  return true;
}

bool RenderInstanceStorage::RegisterMeshDrawInstancedCommand(
    const std::shared_ptr<Mesh>& mesh, const std::shared_ptr<Material>& material, const GlobalTransform& model,
    const std::shared_ptr<ParticleInfoList>& particle_info_list, const bool cast_shadow) {
  if (!material || !mesh || !particle_info_list || !mesh->meshlet_range_ || !mesh->triangle_range_)
    return false;
  if (mesh->UnsafeGetVertices().empty() || mesh->UnsafeGetTriangles().empty())
    return false;
  if (mesh->triangle_range_->prev_frame_index_count == 0 || mesh->meshlet_range_->prev_frame_range == 0)
    return false;
  const auto& material_data = ResolveMaterialData(material);
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
  render_instance->world_bound =
      CalculateInstancedWorldBound(mesh->GetBound(), model.value, particle_info_list->PeekParticleInfoList());
  render_instance->geometry_version = mesh->GetVersion();
  render_instance->material_version = material->GetVersion();
  render_instance->material_index = RegisterMaterial(material, material_data);
  render_instance->line_width = material->draw_settings.line_width;
  render_instance->cull_mode = ResolveInstancedCullModeForTransforms(material->draw_settings.cull_mode, model.value,
                                                                     particle_info_list->PeekParticleInfoList());
  render_instance->polygon_mode = material->draw_settings.polygon_mode;
  render_instance->entity_selected = false;
  switch (ResolveRasterMaterialClass(*material, material_data.shade_material)) {
    case GltfRasterMaterialClass::Forward:
      transparent_instanced_render_instances->Register(render_instance);
      break;
    case GltfRasterMaterialClass::Masked:
      deferred_masked_instanced_render_instances->Register(render_instance);
      break;
    case GltfRasterMaterialClass::Opaque:
      deferred_instanced_render_instances->Register(render_instance);
      break;
  }

  return true;
}

bool RenderInstanceStorage::IsEntitySelectionHighlighted(const Entity& entity) const {
  return entity_selection_highlight_coverage_ &&
         entity_selection_highlight_coverage_->find(entity) != entity_selection_highlight_coverage_->end();
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
  const auto& material_data = ResolveMaterialData(material);
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
  render_instance->entity_selected = IsEntitySelectionHighlighted(entity);
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
  const auto handle = material->GetHandle();
  if (const auto found = material_indices_.find(handle);
      found != material_indices_.end() && material_versions_[handle] == material->GetVersion()) {
    active_material_handles_.insert(handle);
    return found->second;
  }
  return RegisterMaterial(material, ResolveMaterialData(material));
}

void RenderInstanceStorage::BuildFromScene(
    const RenderSettings& render_settings, const std::shared_ptr<Scene>& scene, Bound& world_bound,
    const bool include_editor_cameras,
    const std::vector<std::pair<GlobalTransform, std::shared_ptr<Camera>>>* injected_cameras,
    const bool include_reflection_probes,
    const std::unordered_map<uint64_t, ReflectionProbeTextureOverride>* reflection_probe_texture_overrides,
    std::shared_ptr<const EntitySelectionHighlightCoverage> entity_selection_highlight_coverage,
    const uint64_t entity_selection_revision, const uint64_t scene_hierarchy_revision) {
  const ProfilerScope profiler_scope("RenderInstanceStorage::BuildFromScene", "Render");
  {
    const ProfilerScope selection_scope("RenderInstanceStorage::BindSelectionHighlightCoverage", "Render");
    entity_selection_highlight_coverage_ = std::move(entity_selection_highlight_coverage);
    entity_selection_render_snapshot_ = {scene, entity_selection_revision, scene_hierarchy_revision};
  }
  this->render_settings = render_settings;
  render_info_block.Apply(this->render_settings);
  {
    const ProfilerScope stage_scope("RenderInstanceStorage::CollectEnvironmentAndProbes", "Render");
    CollectEnvironment(scene);
    if (include_reflection_probes) {
      CollectReflectionProbes(scene, reflection_probe_texture_overrides);
    } else {
      render_info_block.reflection_probe_header = glm::uvec4(0u);
      render_info_block.reflection_probes = {};
    }
  }
  {
    const ProfilerScope stage_scope("RenderInstanceStorage::CollectCameras", "Render");
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
      (void)RegisterCamera(camera_info.second->GetHandle(), camera_info_block);
    }
  }
  {
    const ProfilerScope stage_scope("RenderInstanceStorage::CollectEntityRenderers", "Render");
    CollectEntityRenderers(scene, world_bound);
    PrunePersistentTransformRecords();
  }
  {
    const ProfilerScope stage_scope("RenderInstanceStorage::BuildRenderInstanceBlocks", "Render");
    BuildRenderInstanceBlocks();
  }
  UpdateRasterSpatialIndex();
  {
    const ProfilerScope stage_scope("RenderInstanceStorage::BuildEmissiveTriangleInfoBlocks", "Render");
    BuildEmissiveTriangleInfoBlocks();
  }
  {
    const ProfilerScope stage_scope("RenderInstanceStorage::CollectLights", "Render");
    CollectLights(scene, world_bound);
  }
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
    if (const auto& asset = probe.payload; asset && asset->IsRuntimeReady()) {
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
  const ProfilerScope profiler_scope("RenderInstanceStorage::UpdateTopLevelAccelerationStructure", "Render");
  if (!mesh_top_level_acceleration_structure) {
    mesh_top_level_acceleration_structure = std::make_shared<TopLevelAccelerationStructure>();
  }
  switch (mesh_top_level_acceleration_structure->Update(*this)) {
    case TopLevelAccelerationStructure::UpdateMode::NoOp:
      break;
    case TopLevelAccelerationStructure::UpdateMode::Build:
      break;
    case TopLevelAccelerationStructure::UpdateMode::Update:
      break;
  }
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
  const auto local_bound = strands->bound_;
  auto mesh_bound = local_bound;
  if (const auto* cached =
          FindPersistentWorldBound(strands_renderer->GetHandle(), gt, local_bound, strands->GetVersion())) {
    mesh_bound = *cached;
  } else {
    mesh_bound.ApplyTransform(ltw);
    StorePersistentWorldBound(strands_renderer->GetHandle(), gt, local_bound, strands->GetVersion(), mesh_bound);
  }
  glm::vec3 center = mesh_bound.Center();

  glm::vec3 size = mesh_bound.Size();
  min_bound = glm::vec3((glm::min)(min_bound.x, center.x - size.x), (glm::min)(min_bound.y, center.y - size.y),
                        (glm::min)(min_bound.z, center.z - size.z));
  max_bound = glm::vec3(glm::max(max_bound.x, center.x + size.x), glm::max(max_bound.y, center.y + size.y),
                        glm::max(max_bound.z, center.z + size.z));

  const bool raster_ready = Platform::MeshShaderEnabled() && strands->strand_meshlet_range_ &&
                            strands->segment_range_ && strands->segment_range_->prev_frame_index_count != 0 &&
                            strands->strand_meshlet_range_->prev_frame_range != 0;
  const bool ray_ready = Platform::RayTracingLinearSweptSpheresEnabled() && strands->blas_ && strands->blas_->IsReady();
  if (!raster_ready && !ray_ready) {
    return false;
  }

  const auto& material_data = ResolveMaterialData(material);

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
  render_instance->entity_selected = IsEntitySelectionHighlighted(owner);
  render_instance->line_width = material->draw_settings.line_width;
  render_instance->cull_mode = ResolveCullModeForTransform(material->draw_settings.cull_mode, gt.value);
  render_instance->polygon_mode = material->draw_settings.polygon_mode;

  switch (ResolveRasterMaterialClass(*material, material_data.shade_material)) {
    case GltfRasterMaterialClass::Forward:
      transparent_strands_render_instances->Register(render_instance);
      break;
    case GltfRasterMaterialClass::Masked:
      deferred_masked_strands_render_instances->Register(render_instance);
      break;
    case GltfRasterMaterialClass::Opaque:
      deferred_strands_render_instances->Register(render_instance);
      break;
  }

  if (raster_ready) {
    total_strands_segments += strands->segment_range_->prev_frame_index_count;
  }
  return true;
}

bool RenderInstanceStorage::RegisterEntity(const std::shared_ptr<Scene>& target_scene, const Entity& owner,
                                           const std::shared_ptr<GaussianSplatRenderer>& gaussian_splat_renderer,
                                           glm::vec3& min_bound, glm::vec3& max_bound) {
  auto gaussian_splat = gaussian_splat_renderer->gaussian_splat.Get<GaussianSplat>();
  if (!gaussian_splat_renderer->IsEnabled() || !gaussian_splat || gaussian_splat->Empty())
    return false;

  (void)gaussian_splat->EnsureGpuData();
  auto gt = target_scene->GetDataComponent<GlobalTransform>(owner);
  Bound local_bound;
  local_bound.min = gaussian_splat->GetMinBound();
  local_bound.max = gaussian_splat->GetMaxBound();
  auto mesh_bound = local_bound;
  if (const auto* cached = FindPersistentWorldBound(gaussian_splat_renderer->GetHandle(), gt, local_bound,
                                                    gaussian_splat->GetGpuDataRevision())) {
    mesh_bound = *cached;
  } else {
    mesh_bound.ApplyTransform(gt.value);
    StorePersistentWorldBound(gaussian_splat_renderer->GetHandle(), gt, local_bound,
                              gaussian_splat->GetGpuDataRevision(), mesh_bound);
  }
  glm::vec3 center = mesh_bound.Center();

  glm::vec3 size = mesh_bound.Size();
  min_bound = glm::vec3((glm::min)(min_bound.x, center.x - size.x), (glm::min)(min_bound.y, center.y - size.y),
                        (glm::min)(min_bound.z, center.z - size.z));
  max_bound = glm::vec3(glm::max(max_bound.x, center.x + size.x), glm::max(max_bound.y, center.y + size.y),
                        glm::max(max_bound.z, center.z + size.z));

  const auto render_instance = std::make_shared<GaussianSplatRenderInstance>();
  render_instance->command_type = RenderInstanceType::FromRenderer;
  render_instance->owner = owner;
  render_instance->entity_handle = target_scene->GetEntityHandle(owner);
  render_instance->renderer_handle = gaussian_splat_renderer->GetHandle();
  render_instance->model = gt;
  render_instance->gaussian_splat = gaussian_splat;
  render_instance->world_bound = mesh_bound;
  render_instance->geometry_version = gaussian_splat->GetGpuDataRevision();
  render_instance->entity_selected = IsEntitySelectionHighlighted(owner);
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
  auto local_bound = mesh->GetBound();
  if (mesh_renderer->ray_tracing_blas_) {
    local_bound.min = glm::min(local_bound.min, mesh_renderer->ray_tracing_bound_.min);
    local_bound.max = glm::max(local_bound.max, mesh_renderer->ray_tracing_bound_.max);
  }
  const auto renderer_handle = mesh_renderer->GetHandle();
  const auto source_owner = mesh_renderer->GetOwner();
  const auto entity_handle = target_scene->GetEntityHandle(owner);
  const auto mesh_version = mesh->GetVersion();
  const auto material_version = material->GetVersion();
  const bool cacheable = target_scene->IsEntityStatic(source_owner) && renderer_handle != 0;
  std::shared_ptr<MeshRenderInstance> render_instance;
  GltfRasterMaterialClass raster_class = GltfRasterMaterialClass::Opaque;
  if (cacheable) {
    if (const auto found = static_mesh_render_instance_cache_.find(renderer_handle);
        found != static_mesh_render_instance_cache_.end()) {
      const auto& record = found->second;
      const auto& cached = record.render_instance;
      if (cached && record.source_owner == source_owner && cached->owner == owner &&
          cached->entity_handle == entity_handle && cached->mesh == mesh && cached->material == material &&
          cached->model == gt && BoundsEqual(record.local_bound, local_bound) &&
          cached->geometry_version == mesh_version && cached->material_version == material_version &&
          cached->cast_shadow == mesh_renderer->cast_shadow &&
          cached->ray_tracing_geometry_version == mesh_renderer->ray_tracing_geometry_version_ &&
          cached->morph_weights_version == mesh_renderer->morph_weights_version_ &&
          cached->ray_tracing_triangle_range == mesh_renderer->ray_tracing_triangle_range_ &&
          cached->ray_tracing_blas == mesh_renderer->ray_tracing_blas_) {
        render_instance = cached;
        raster_class = record.raster_class;
      }
    }
  }
  auto mesh_bound = local_bound;
  if (render_instance) {
    mesh_bound = render_instance->world_bound;
  } else if (const auto* cached = FindPersistentWorldBound(renderer_handle, gt, local_bound, mesh_version)) {
    mesh_bound = *cached;
  } else {
    mesh_bound.ApplyTransform(ltw);
    StorePersistentWorldBound(renderer_handle, gt, local_bound, mesh_version, mesh_bound);
  }
  glm::vec3 center = mesh_bound.Center();

  glm::vec3 size = mesh_bound.Size();
  min_bound = glm::vec3((glm::min)(min_bound.x, center.x - size.x), (glm::min)(min_bound.y, center.y - size.y),
                        (glm::min)(min_bound.z, center.z - size.z));
  max_bound = glm::vec3(glm::max(max_bound.x, center.x + size.x), glm::max(max_bound.y, center.y + size.y),
                        glm::max(max_bound.z, center.z + size.z));

  if (!render_instance) {
    const auto& material_data = ResolveMaterialData(material);
    render_instance = std::make_shared<MeshRenderInstance>();
    render_instance->command_type = RenderInstanceType::FromRenderer;
    render_instance->owner = owner;
    render_instance->mesh = mesh;
    render_instance->material = material;
    render_instance->model = gt;
    render_instance->entity_handle = entity_handle;
    render_instance->renderer_handle = renderer_handle;
    render_instance->cast_shadow = mesh_renderer->cast_shadow;
    render_instance->world_bound = mesh_bound;
    render_instance->geometry_version = mesh_version;
    render_instance->ray_tracing_geometry_version = mesh_renderer->ray_tracing_geometry_version_;
    render_instance->morph_weights_version = mesh_renderer->morph_weights_version_;
    render_instance->ray_tracing_triangle_range = mesh_renderer->ray_tracing_triangle_range_;
    render_instance->ray_tracing_blas = mesh_renderer->ray_tracing_blas_;
    render_instance->material_version = material_version;
    render_instance->material_index = RegisterMaterial(material, material_data);
    render_instance->line_width = material->draw_settings.line_width;
    render_instance->cull_mode = ResolveCullModeForTransform(material->draw_settings.cull_mode, gt.value);
    render_instance->polygon_mode = material->draw_settings.polygon_mode;
    raster_class = ResolveRasterMaterialClass(*material, material_data.shade_material);
    if (cacheable) {
      static_mesh_render_instance_cache_[renderer_handle] = {source_owner, local_bound, render_instance, raster_class};
    }
  } else {
    render_instance->material_index = RegisterMaterial(material);
  }
  render_instance->entity_selected = IsEntitySelectionHighlighted(owner);
  switch (raster_class) {
    case GltfRasterMaterialClass::Forward:
      transparent_render_instances->Register(render_instance);
      break;
    case GltfRasterMaterialClass::Masked:
      deferred_masked_render_instances->Register(render_instance);
      break;
    case GltfRasterMaterialClass::Opaque:
      deferred_render_instances->Register(render_instance);
      break;
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
  auto local_bound = skinned_mesh->GetBound();
  if (skinned_mesh_renderer->ray_tracing_blas_) {
    local_bound.min = glm::min(local_bound.min, skinned_mesh_renderer->ray_tracing_bound_.min);
    local_bound.max = glm::max(local_bound.max, skinned_mesh_renderer->ray_tracing_bound_.max);
  }
  auto mesh_bound = local_bound;
  const uint64_t content_signature =
      static_cast<uint64_t>(skinned_mesh->GetVersion()) << 32u | skinned_mesh_renderer->morph_weights_version_;
  if (const auto* cached =
          FindPersistentWorldBound(skinned_mesh_renderer->GetHandle(), gt, local_bound, content_signature)) {
    mesh_bound = *cached;
  } else {
    mesh_bound.ApplyTransform(ltw);
    StorePersistentWorldBound(skinned_mesh_renderer->GetHandle(), gt, local_bound, content_signature, mesh_bound);
  }
  glm::vec3 center = mesh_bound.Center();

  glm::vec3 size = mesh_bound.Size();
  min_bound = glm::vec3((glm::min)(min_bound.x, center.x - size.x), (glm::min)(min_bound.y, center.y - size.y),
                        (glm::min)(min_bound.z, center.z - size.z));
  max_bound = glm::vec3(glm::max(max_bound.x, center.x + size.x), glm::max(max_bound.y, center.y + size.y),
                        glm::max(max_bound.z, center.z + size.z));

  const auto& material_data = ResolveMaterialData(material);
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
  render_instance->entity_selected = IsEntitySelectionHighlighted(owner);
  render_instance->line_width = material->draw_settings.line_width;
  render_instance->cull_mode = ResolveCullModeForTransform(material->draw_settings.cull_mode, gt.value);
  render_instance->polygon_mode = material->draw_settings.polygon_mode;

  switch (ResolveRasterMaterialClass(*material, material_data.shade_material)) {
    case GltfRasterMaterialClass::Forward:
      transparent_skinned_render_instances->Register(render_instance);
      break;
    case GltfRasterMaterialClass::Masked:
      deferred_masked_skinned_render_instances->Register(render_instance);
      break;
    case GltfRasterMaterialClass::Opaque:
      deferred_skinned_render_instances->Register(render_instance);
      break;
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
  const auto local_bound = mesh->GetBound();
  Bound mesh_bound;
  const uint64_t content_signature =
      static_cast<uint64_t>(mesh->GetVersion()) << 32u | particle_info_list->GetVersion();
  if (const auto* cached = FindPersistentWorldBound(particles->GetHandle(), gt, local_bound, content_signature)) {
    mesh_bound = *cached;
  } else {
    mesh_bound = CalculateInstancedWorldBound(local_bound, ltw, particle_info_list->PeekParticleInfoList());
    StorePersistentWorldBound(particles->GetHandle(), gt, local_bound, content_signature, mesh_bound);
  }
  glm::vec3 center = mesh_bound.Center();

  glm::vec3 size = mesh_bound.Size();
  min_bound = glm::vec3((glm::min)(min_bound.x, center.x - size.x), (glm::min)(min_bound.y, center.y - size.y),
                        (glm::min)(min_bound.z, center.z - size.z));

  max_bound = glm::vec3(glm::max(max_bound.x, center.x + size.x), glm::max(max_bound.y, center.y + size.y),
                        glm::max(max_bound.z, center.z + size.z));

  const auto& material_data = ResolveMaterialData(material);

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
  render_instance->world_bound = mesh_bound;
  render_instance->geometry_version = mesh->GetVersion();
  render_instance->material_version = material->GetVersion();
  render_instance->material_index = RegisterMaterial(material, material_data);
  render_instance->particle_info_list_version = particle_info_list->GetVersion();
  render_instance->entity_selected = IsEntitySelectionHighlighted(owner);
  render_instance->line_width = material->draw_settings.line_width;
  render_instance->cull_mode = ResolveInstancedCullModeForTransforms(material->draw_settings.cull_mode, gt.value,
                                                                     particle_info_list->PeekParticleInfoList());
  render_instance->polygon_mode = material->draw_settings.polygon_mode;

  switch (ResolveRasterMaterialClass(*material, material_data.shade_material)) {
    case GltfRasterMaterialClass::Forward:
      transparent_instanced_render_instances->Register(render_instance);
      break;
    case GltfRasterMaterialClass::Masked:
      deferred_masked_instanced_render_instances->Register(render_instance);
      break;
    case GltfRasterMaterialClass::Opaque:
      deferred_instanced_render_instances->Register(render_instance);
      break;
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
  active_material_handles_.insert(handle);
  const auto search = material_indices_.find(handle);
  if (search == material_indices_.end()) {
    const int index = static_cast<int>(gltf_material_cache_.GetShadeMaterials().size());
    material_indices_[handle] = index;
    material_versions_[handle] = material->GetVersion();
    const auto gltf_material_index = gltf_material_cache_.Append(material_data);
    if (gltf_material_index != static_cast<uint32_t>(index)) {
      throw std::runtime_error("glTF material cache drifted from render material indices.");
    }
    material_cache_changed_this_frame_ = true;
    return index;
  }
  const auto version = material->GetVersion();
  if (material_versions_[handle] != version) {
    gltf_material_cache_.Update(static_cast<uint32_t>(search->second), material_data);
    material_versions_[handle] = version;
    material_cache_changed_this_frame_ = true;
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

bool RenderInstanceStorage::HasSelectionHighlightRenderInstances() const {
  return std::any_of(instance_info_blocks_.begin(), instance_info_blocks_.end(), [](const auto& instance) {
    return instance.info_index & 1;
  });
}
