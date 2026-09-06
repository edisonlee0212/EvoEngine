// Allocation and bindings adapted from Godot renderer_rd/environment/gi.cpp::SDFGI::create,
// 34d06658a85845111a50db9e485ec4a0701d4298. See docs/licenses/Godot-MIT.txt.
#include "SdfgiResources.hpp"
#include "RenderInstanceStorage.hpp"
#include "SdfgiDebug.hpp"
#include "SdfgiGather.hpp"
#include "SdfgiLight.hpp"
#include "SdfgiPreprocess.hpp"
#include "SdfgiProbe.hpp"
#include "SdfgiVoxelizer.hpp"

#include "Platform.hpp"
#include "RenderPasses/RenderPassUtilities.hpp"
#include "Resources.hpp"
#include "Shader.hpp"

#include <algorithm>
#include <set>
#include <stdexcept>

using namespace evo_engine;

namespace {
constexpr VkImageUsageFlags kImageUsage = VK_IMAGE_USAGE_STORAGE_BIT | VK_IMAGE_USAGE_SAMPLED_BIT |
                                          VK_IMAGE_USAGE_TRANSFER_SRC_BIT | VK_IMAGE_USAGE_TRANSFER_DST_BIT;
constexpr VkPipelineStageFlags2 kFieldStages =
    VK_PIPELINE_STAGE_2_TRANSFER_BIT | VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT | VK_PIPELINE_STAGE_2_ALL_GRAPHICS_BIT |
    VK_PIPELINE_STAGE_2_DRAW_INDIRECT_BIT;
constexpr VkAccessFlags2 kFieldAccess = VK_ACCESS_2_TRANSFER_READ_BIT | VK_ACCESS_2_TRANSFER_WRITE_BIT |
                                        VK_ACCESS_2_SHADER_READ_BIT | VK_ACCESS_2_SHADER_WRITE_BIT |
                                        VK_ACCESS_2_INDIRECT_COMMAND_READ_BIT;

std::string CascadeName(uint32_t cascade, const std::string& resource) {
  return "Cascade" + std::to_string(cascade) + "." + resource;
}
std::string FrameName(uint32_t frame, const std::string& resource) {
  return "Frame" + std::to_string(frame) + "." + resource;
}
std::string GraphName(const std::string& resource) {
  return "Frame.SDFGI." + resource;
}
}  // namespace

std::string SdfgiResources::ValidateDescriptorLimits(
    const std::vector<std::shared_ptr<DescriptorSetLayout>>& pipeline_layouts, const VkPhysicalDeviceLimits& limits) {
  if (pipeline_layouts.size() > limits.maxBoundDescriptorSets)
    return "SDFGI pipeline exceeds maxBoundDescriptorSets";
  // Samplers, sampled images, storage images, uniform buffers, storage buffers, per-stage resources.
  std::array<uint64_t, 6> total{};
  std::array<std::array<uint64_t, 6>, 3> stages{};
  constexpr VkShaderStageFlags stage_flags[]{VK_SHADER_STAGE_VERTEX_BIT, VK_SHADER_STAGE_FRAGMENT_BIT,
                                             VK_SHADER_STAGE_COMPUTE_BIT};
  for (const auto& layout : pipeline_layouts) {
    if (!layout)
      return "SDFGI host descriptor layout is unavailable";
    for (const auto& [binding, value] : layout->GetDescriptorBindings()) {
      const auto& descriptor = value.binding;
      std::array<uint64_t, 6> count{};
      switch (descriptor.descriptorType) {
        case VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER:
          count[0] = count[1] = descriptor.descriptorCount;
          break;
        case VK_DESCRIPTOR_TYPE_SAMPLER:
          count[0] = descriptor.descriptorCount;
          break;
        case VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE:
          count[1] = descriptor.descriptorCount;
          break;
        case VK_DESCRIPTOR_TYPE_STORAGE_IMAGE:
          count[2] = descriptor.descriptorCount;
          break;
        case VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER:
          count[3] = descriptor.descriptorCount;
          break;
        case VK_DESCRIPTOR_TYPE_STORAGE_BUFFER:
          count[4] = descriptor.descriptorCount;
          break;
        default:
          break;
      }
      count[5] = descriptor.descriptorType == VK_DESCRIPTOR_TYPE_SAMPLER ? 0 : descriptor.descriptorCount;
      for (size_t i = 0; i < count.size(); ++i) {
        total[i] += count[i];
        for (size_t s = 0; s < stages.size(); ++s)
          if (descriptor.stageFlags & stage_flags[s])
            stages[s][i] += count[i];
      }
    }
  }
  const uint32_t set_limits[]{limits.maxDescriptorSetSamplers, limits.maxDescriptorSetSampledImages,
                              limits.maxDescriptorSetStorageImages, limits.maxDescriptorSetUniformBuffers,
                              limits.maxDescriptorSetStorageBuffers};
  const uint32_t stage_limits[]{limits.maxPerStageDescriptorSamplers,       limits.maxPerStageDescriptorSampledImages,
                                limits.maxPerStageDescriptorStorageImages,  limits.maxPerStageDescriptorUniformBuffers,
                                limits.maxPerStageDescriptorStorageBuffers, limits.maxPerStageResources};
  for (size_t i = 0; i < total.size(); ++i) {
    if (i < 5 && total[i] > set_limits[i])
      return "SDFGI combined descriptor limit exceeded, category " + std::to_string(i);
    for (size_t s = 0; s < stages.size(); ++s)
      if (stages[s][i] > stage_limits[i])
        return "SDFGI per-stage descriptor limit exceeded, stage " + std::to_string(s) + ", category " +
               std::to_string(i);
  }
  return {};
}

std::shared_ptr<SdfgiResources> SdfgiResources::TryCreate(
    const SdfgiSettings& settings, const std::vector<std::shared_ptr<DescriptorSetLayout>>& deferred_host_layouts,
    std::string& failure, const uint32_t fail_after_allocations) {
  failure = settings.Validate();
  if (!failure.empty())
    return {};
  const auto report =
      QuerySdfgiCapabilities(settings.cascade_count, settings.history_size, settings.wide_horizontal_field);
  if (!report.Supported()) {
    failure = report.ToString();
    return {};
  }
  try {
    auto result = std::make_shared<SdfgiResources>();
    result->settings = settings;
    result->Allocate(deferred_host_layouts, fail_after_allocations);
    return result;
  } catch (const std::exception& error) {
    failure = std::string("SDFGI resource initialization failed: ") + error.what();
    return {};
  }
}

void SdfgiResources::Allocate(const std::vector<std::shared_ptr<DescriptorSetLayout>>& deferred_host_layouts,
                              const uint32_t fail_after_allocations) {
  CreateLayouts(deferred_host_layouts);
  uint32_t allocations = 0;
  const auto checkpoint = [&] {
    if (allocations++ == fail_after_allocations)
      throw std::runtime_error("forced allocation failure");
  };
  const auto requirements =
      GetSdfgiImageRequirements(settings.cascade_count, settings.history_size, settings.wide_horizontal_field);
  const auto add_image = [&](const std::string& name, SdfgiImageRequirement requirement,
                             const SdfgiMemoryClass memory_class, const bool cube = false) {
    checkpoint();
    const std::array formats{requirement.storage_format, requirement.sampled_format};
    VkImageFormatListCreateInfo format_list{VK_STRUCTURE_TYPE_IMAGE_FORMAT_LIST_CREATE_INFO};
    format_list.viewFormatCount = formats[0] == formats[1] ? 1 : 2;
    format_list.pViewFormats = formats.data();
    VkImageCreateInfo info{VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO};
    info.pNext = &format_list;
    info.flags = requirement.CreateFlags() | (cube ? VK_IMAGE_CREATE_CUBE_COMPATIBLE_BIT : 0);
    info.imageType = requirement.type;
    info.format = requirement.storage_format;
    info.extent = requirement.extent;
    info.mipLevels = 1;
    info.arrayLayers = requirement.layers;
    info.samples = VK_SAMPLE_COUNT_1_BIT;
    info.tiling = VK_IMAGE_TILING_OPTIMAL;
    info.usage = kImageUsage;
    info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
    auto image = std::make_shared<Image>(info);
    const auto make_view = [&](const VkFormat format, const VkImageUsageFlags usage, const bool sampled) {
      VkImageViewUsageCreateInfo view_usage{VK_STRUCTURE_TYPE_IMAGE_VIEW_USAGE_CREATE_INFO};
      view_usage.usage = usage;
      VkImageViewCreateInfo view{VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO};
      view.pNext = &view_usage;
      view.image = image->GetVkImage();
      view.format = format;
      view.viewType = requirement.type == VK_IMAGE_TYPE_3D ? VK_IMAGE_VIEW_TYPE_3D
                      : cube ? (sampled ? VK_IMAGE_VIEW_TYPE_CUBE : VK_IMAGE_VIEW_TYPE_2D_ARRAY)
                      : (requirement.layers > 1 || std::string(requirement.name) == "ambient")
                          ? VK_IMAGE_VIEW_TYPE_2D_ARRAY
                          : VK_IMAGE_VIEW_TYPE_2D;
      view.subresourceRange = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, requirement.layers};
      auto result = std::make_shared<ImageView>(view, image);
      if (!result->GetVkImageView())
        throw std::runtime_error("image view creation failed: " + name);
      return result;
    };
    auto storage = make_view(requirement.storage_format, VK_IMAGE_USAGE_STORAGE_BIT, false);
    auto sampled = make_view(requirement.sampled_format, VK_IMAGE_USAGE_SAMPLED_BIT, true);
    allocated_bytes[static_cast<size_t>(memory_class)] += image->GetVmaAllocationInfo().size;
    textures.emplace(name, SdfgiTexture{requirement, image, storage, sampled, memory_class});
  };
  const auto add_buffer = [&](const std::string& name, const VkDeviceSize size, const VkBufferUsageFlags usage,
                              const SdfgiMemoryClass memory_class) {
    checkpoint();
    VkBufferCreateInfo info{VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO};
    info.size = size;
    info.usage = usage | VK_BUFFER_USAGE_TRANSFER_SRC_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT;
    info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
    auto buffer = std::make_shared<Buffer>(info);
    if (!buffer->GetVkBuffer())
      throw std::runtime_error("buffer allocation failed: " + name);
    allocated_bytes[static_cast<size_t>(memory_class)] += buffer->GetVmaAllocationInfo().size;
    buffers.emplace(name, SdfgiBuffer{buffer, memory_class});
  };
  add_image("Albedo", requirements[0], SdfgiMemoryClass::Scratch);
  add_image("Emission", requirements[1], SdfgiMemoryClass::Scratch);
  add_image("EmissionAniso", requirements[1], SdfgiMemoryClass::Scratch);
  add_image("Facing", requirements[2], SdfgiMemoryClass::Scratch);
  for (uint32_t i = 0; i < 8; ++i)
    add_image("OcclusionScratch" + std::to_string(i), requirements[5], SdfgiMemoryClass::Scratch);
  for (uint32_t i = 0; i < 2; ++i) {
    add_image("JumpFlood" + std::to_string(i), requirements[3], SdfgiMemoryClass::Scratch);
    add_image("JumpFloodHalf" + std::to_string(i), requirements[4], SdfgiMemoryClass::Scratch);
  }
  add_image("HistoryScroll", requirements[10], SdfgiMemoryClass::Scratch);
  add_image("AverageScroll", requirements[11], SdfgiMemoryClass::Scratch);
  add_image("Occlusion", requirements[9], SdfgiMemoryClass::Field);
  add_image("Atlas", requirements[12], SdfgiMemoryClass::Field);
  add_image("Ambient", requirements[13], SdfgiMemoryClass::Field);
  auto black_sky = requirements[13];
  black_sky.extent = {1, 1, 1};
  black_sky.layers = 6;
  add_image("BlackSky", black_sky, SdfgiMemoryClass::Field, true);
  for (uint32_t c = 0; c < settings.cascade_count; ++c) {
    add_image(CascadeName(c, "Sdf"), requirements[5], SdfgiMemoryClass::Field);
    add_image(CascadeName(c, "Light"), requirements[6], SdfgiMemoryClass::Field);
    add_image(CascadeName(c, "Aniso0"), requirements[7], SdfgiMemoryClass::Field);
    add_image(CascadeName(c, "Aniso1"), requirements[8], SdfgiMemoryClass::Field);
    add_image(CascadeName(c, "History"), requirements[10], SdfgiMemoryClass::Field);
    add_image(CascadeName(c, "Average"), requirements[11], SdfgiMemoryClass::Field);
    add_buffer(CascadeName(c, "SolidCells"), sizeof(SdfgiSolidCell) * settings.SolidCellCapacity(),
               VK_BUFFER_USAGE_STORAGE_BUFFER_BIT, SdfgiMemoryClass::Field);
    add_buffer(CascadeName(c, "UnlitCells"), sizeof(SdfgiSolidCell) * settings.SolidCellCapacity(),
               VK_BUFFER_USAGE_STORAGE_BUFFER_BIT, SdfgiMemoryClass::Field);
    add_buffer(CascadeName(c, "Dispatch"), sizeof(SdfgiDispatchData), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT,
               SdfgiMemoryClass::Field);
    add_buffer(CascadeName(c, "Indirect"), sizeof(SdfgiDispatchData),
               VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_INDIRECT_BUFFER_BIT, SdfgiMemoryClass::Field);
  }
  add_buffer("Status", sizeof(SdfgiFieldStatus), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT, SdfgiMemoryClass::Field);
  voxel_frames.resize(Platform::GetMaxFramesInFlight());
  light_frames.resize(Platform::GetMaxFramesInFlight());
  probe_frames.resize(Platform::GetMaxFramesInFlight());
  gather_frames.resize(Platform::GetMaxFramesInFlight());
  for (uint32_t f = 0; f < Platform::GetMaxFramesInFlight(); ++f) {
    add_buffer(FrameName(f, "Cascades"), sizeof(SdfgiCascadeBlock), VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT,
               SdfgiMemoryClass::Upload);
    add_buffer(FrameName(f, "Gather"), sizeof(SdfgiGatherData), VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT,
               SdfgiMemoryClass::Upload);
    for (uint32_t c = 0; c < settings.cascade_count; ++c) {
      add_buffer(FrameName(f, CascadeName(c, "StaticLights")), sizeof(SdfgiLight) * 1024,
                 VK_BUFFER_USAGE_STORAGE_BUFFER_BIT, SdfgiMemoryClass::Upload);
      add_buffer(FrameName(f, CascadeName(c, "DynamicLights")), sizeof(SdfgiLight) * 128,
                 VK_BUFFER_USAGE_STORAGE_BUFFER_BIT, SdfgiMemoryClass::Upload);
    }
  }
  VkSamplerCreateInfo sampler{VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO};
  sampler.magFilter = sampler.minFilter = VK_FILTER_LINEAR;
  sampler.mipmapMode = VK_SAMPLER_MIPMAP_MODE_NEAREST;
  sampler.addressModeU = sampler.addressModeV = sampler.addressModeW = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
  sampler.maxLod = 0;
  linear_sampler = std::make_shared<Sampler>(sampler);
  sampler.mipmapMode = VK_SAMPLER_MIPMAP_MODE_LINEAR;
  sampler.maxLod = VK_LOD_CLAMP_NONE;
  mip_sampler = std::make_shared<Sampler>(sampler);
  if (!linear_sampler->GetVkSampler() || !mip_sampler->GetVkSampler())
    throw std::runtime_error("sampler creation failed");
  CreateDescriptors();
  CreatePipelines(deferred_host_layouts);
}

void SdfgiResources::CreateLayouts(const std::vector<std::shared_ptr<DescriptorSetLayout>>& deferred_host_layouts) {
  if (deferred_host_layouts.size() != 5)
    throw std::runtime_error("SDFGI deferred variant requires the five host layouts");
  for (auto& layout : layouts)
    layout = std::make_shared<DescriptorSetLayout>();
  const auto binding = [&](const SdfgiLayout layout, const uint32_t index, const VkDescriptorType type,
                           const uint32_t count = 1, const VkShaderStageFlags stages = VK_SHADER_STAGE_COMPUTE_BIT) {
    layouts[static_cast<size_t>(layout)]->PushDescriptorBinding(index, type, stages, 0, count);
  };
  constexpr auto image = VK_DESCRIPTOR_TYPE_STORAGE_IMAGE;
  constexpr auto texture = VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE;
  constexpr auto buffer = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  constexpr auto uniform = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
  constexpr auto sampler = VK_DESCRIPTOR_TYPE_SAMPLER;
  for (auto layout : {SdfgiLayout::Initialize, SdfgiLayout::JumpFlood}) {
    binding(layout, 1, image);
    binding(layout, 2, image);
  }
  for (uint32_t i = 1; i <= 3; ++i)
    binding(SdfgiLayout::Upscale, i, image);
  binding(SdfgiLayout::Occlusion, 1, image);
  binding(SdfgiLayout::Occlusion, 2, image, 8);
  binding(SdfgiLayout::Occlusion, 3, image);
  for (uint32_t i = 1; i <= 8; ++i)
    binding(SdfgiLayout::Store, i, image, i == 3 ? 8 : 1);
  for (uint32_t i : {10u, 11u, 12u})
    binding(SdfgiLayout::Store, i, buffer);
  for (uint32_t i = 1; i <= 4; ++i)
    binding(SdfgiLayout::Scroll, i, image);
  binding(SdfgiLayout::Scroll, 5, buffer);
  binding(SdfgiLayout::Scroll, 6, buffer);
  binding(SdfgiLayout::Scroll, 7, buffer);
  for (uint32_t i = 1; i <= 4; ++i)
    binding(SdfgiLayout::PayloadRefresh, i, image);
  for (uint32_t i = 5; i <= 7; ++i)
    binding(SdfgiLayout::PayloadRefresh, i, buffer);
  binding(SdfgiLayout::ScrollOcclusion, 1, image, 8);
  binding(SdfgiLayout::ScrollOcclusion, 2, image);
  binding(SdfgiLayout::DirectLight, 1, texture, 8);
  binding(SdfgiLayout::DirectLight, 2, sampler);
  binding(SdfgiLayout::DirectLight, 3, sampler);
  binding(SdfgiLayout::DirectLight, 4, buffer);
  binding(SdfgiLayout::DirectLight, 5, buffer);
  for (uint32_t i = 6; i <= 8; ++i)
    binding(SdfgiLayout::DirectLight, i, image);
  binding(SdfgiLayout::DirectLight, 9, uniform);
  binding(SdfgiLayout::DirectLight, 10, buffer);
  binding(SdfgiLayout::DirectLight, 11, texture);
  binding(SdfgiLayout::DirectLight, 12, texture);
  binding(SdfgiLayout::DirectLight, 13, buffer);
  for (uint32_t i = 1; i <= 4; ++i)
    binding(SdfgiLayout::Integrate, i, texture, 8);
  binding(SdfgiLayout::Integrate, 6, sampler);
  binding(SdfgiLayout::Integrate, 7, uniform);
  for (uint32_t i = 8; i <= 14; ++i)
    binding(SdfgiLayout::Integrate, i, image);
  binding(SdfgiLayout::Integrate, 15, buffer);
  binding(SdfgiLayout::Sky, 0, texture);
  binding(SdfgiLayout::Sky, 1, sampler);
  constexpr VkShaderStageFlags voxel_stages = VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT;
  binding(SdfgiLayout::Voxel, 0, uniform, 1, voxel_stages);
  for (uint32_t i = 1; i <= 4; ++i)
    binding(SdfgiLayout::Voxel, i, image, 1, VK_SHADER_STAGE_FRAGMENT_BIT);
  binding(SdfgiLayout::Gather, 0, uniform);
  binding(SdfgiLayout::Gather, 1, texture);
  binding(SdfgiLayout::Gather, 2, texture);
  binding(SdfgiLayout::Gather, 3, sampler);
  binding(SdfgiLayout::Gather, 4, buffer);
  binding(SdfgiLayout::Gather, 5, texture, 8);
  binding(SdfgiLayout::Gather, 6, texture, 8);
  auto deferred = deferred_host_layouts;
  deferred.push_back(layouts[static_cast<size_t>(SdfgiLayout::Gather)]);
  const auto& limits = Platform::GetSelectedPhysicalDevice()->properties.limits;
  auto check = [&](const std::vector<std::shared_ptr<DescriptorSetLayout>>& list) {
    if (const auto error = ValidateDescriptorLimits(list, limits); !error.empty())
      throw std::runtime_error(error);
  };
  check(deferred);
  for (const auto& layout : layouts)
    check({layout});
  check({layouts[static_cast<size_t>(SdfgiLayout::Integrate)], layouts[static_cast<size_t>(SdfgiLayout::Sky)]});
  check({deferred_host_layouts[0], layouts[static_cast<size_t>(SdfgiLayout::Voxel)]});
  for (const auto& layout : layouts) {
    layout->Initialize();
    if (!layout->GetVkDescriptorSetLayout())
      throw std::runtime_error("descriptor layout creation failed");
  }
}

void SdfgiResources::CreateDescriptors() {
  const auto make_set = [&](const std::string& name, const SdfgiLayout layout) {
    auto result = std::make_shared<DescriptorSet>(layouts[static_cast<size_t>(layout)]);
    sets.emplace(name, result);
    return result;
  };
  const auto image = [&](const std::shared_ptr<DescriptorSet>& set, const uint32_t binding, const std::string& name,
                         const bool sampled = false, const uint32_t element = 0) {
    const auto& texture = textures.at(name);
    VkDescriptorImageInfo info{};
    info.imageLayout = VK_IMAGE_LAYOUT_GENERAL;
    info.imageView = (sampled ? texture.sampled_view : texture.storage_view)->GetVkImageView();
    set->UpdateImageDescriptorBinding(binding, info, element);
  };
  const auto buffer = [&](const std::shared_ptr<DescriptorSet>& set, const uint32_t binding, const std::string& name) {
    set->UpdateBufferDescriptorBinding(binding, buffers.at(name).buffer);
  };
  const auto sampler = [&](const std::shared_ptr<DescriptorSet>& set, const uint32_t binding, const bool mip = false) {
    VkDescriptorImageInfo info{};
    info.sampler = (mip ? mip_sampler : linear_sampler)->GetVkSampler();
    set->UpdateImageDescriptorBinding(binding, info);
  };
  for (const bool half : {false, true}) {
    const std::string suffix = half ? "Half" : "";
    auto set = make_set("Initialize" + suffix, SdfgiLayout::Initialize);
    image(set, 1, "Albedo");
    image(set, 2, "JumpFlood" + suffix + "0");
    for (uint32_t i = 0; i < 2; ++i) {
      set = make_set("JumpFlood" + suffix + std::to_string(i), SdfgiLayout::JumpFlood);
      image(set, 1, "JumpFlood" + suffix + std::to_string(i));
      image(set, 2, "JumpFlood" + suffix + std::to_string(1 - i));
    }
  }
  auto set = make_set("Upscale", SdfgiLayout::Upscale);
  image(set, 1, "Albedo");
  image(set, 2, settings.wide_horizontal_field ? "JumpFloodHalf1" : "JumpFloodHalf0");
  image(set, 3, "JumpFlood0");
  set = make_set("Occlusion", SdfgiLayout::Occlusion);
  image(set, 1, "Albedo");
  image(set, 3, "Facing");
  for (uint32_t i = 0; i < 8; ++i)
    image(set, 2, "OcclusionScratch" + std::to_string(i), false, i);
  set = make_set("ScrollOcclusion", SdfgiLayout::ScrollOcclusion);
  image(set, 2, "Occlusion");
  for (uint32_t i = 0; i < 8; ++i)
    image(set, 1, "OcclusionScratch" + std::to_string(i), false, i);
  set = make_set("Sky", SdfgiLayout::Sky);
  image(set, 0, "BlackSky", true);
  sampler(set, 1, true);
  for (uint32_t c = 0; c < settings.cascade_count; ++c) {
    set = make_set(CascadeName(c, "Store"), SdfgiLayout::Store);
    image(set, 1, "JumpFlood1");
    image(set, 2, "Albedo");
    for (uint32_t i = 0; i < 8; ++i)
      image(set, 3, "OcclusionScratch" + std::to_string(i), false, i);
    image(set, 4, "Emission");
    image(set, 5, "EmissionAniso");
    image(set, 6, "Facing");
    image(set, 7, CascadeName(c, "Sdf"));
    image(set, 8, "Occlusion");
    buffer(set, 10, CascadeName(c, "Dispatch"));
    buffer(set, 11, CascadeName(c, "SolidCells"));
    buffer(set, 12, "Status");
    set = make_set(CascadeName(c, "Scroll"), SdfgiLayout::Scroll);
    image(set, 1, "Albedo");
    image(set, 2, "Facing");
    image(set, 3, "Emission");
    image(set, 4, "EmissionAniso");
    buffer(set, 5, CascadeName(c, "Dispatch"));
    // Static-light refresh reseeds every rebuilt cascade; retain emission without baked static light.
    buffer(set, 6, CascadeName(c, "UnlitCells"));
    buffer(set, 7, "Status");
    set = make_set(CascadeName(c, "PayloadRefresh"), SdfgiLayout::PayloadRefresh);
    image(set, 1, "Albedo");
    image(set, 2, "Facing");
    image(set, 3, "Emission");
    image(set, 4, "EmissionAniso");
    buffer(set, 5, CascadeName(c, "Dispatch"));
    buffer(set, 6, CascadeName(c, "UnlitCells"));
    buffer(set, 7, "Status");
  }
  for (uint32_t f = 0; f < Platform::GetMaxFramesInFlight(); ++f) {
    set = make_set(FrameName(f, "Gather"), SdfgiLayout::Gather);
    buffer(set, 0, FrameName(f, "Gather"));
    image(set, 1, "Atlas", true);
    image(set, 2, "Occlusion", true);
    sampler(set, 3);
    buffer(set, 4, "Status");
    for (uint32_t i = 0; i < 8; ++i) {
      image(set, 5, CascadeName(std::min(i, settings.cascade_count - 1), "Sdf"), true, i);
      image(set, 6, CascadeName(std::min(i, settings.cascade_count - 1), "Light"), true, i);
    }
    for (uint32_t c = 0; c < settings.cascade_count; ++c) {
      for (const std::string kind : {"StaticLights", "DynamicLights"}) {
        set = make_set(FrameName(f, CascadeName(c, kind)), SdfgiLayout::DirectLight);
        for (uint32_t i = 0; i < 8; ++i)
          image(set, 1, CascadeName(std::min(i, settings.cascade_count - 1), "Sdf"), true, i);
        sampler(set, 2);
        sampler(set, 3, true);
        buffer(set, 4, CascadeName(c, "Dispatch"));
        buffer(set, 5, CascadeName(c, "SolidCells"));
        image(set, 6, CascadeName(c, "Light"));
        image(set, 7, CascadeName(c, "Aniso0"));
        image(set, 8, CascadeName(c, "Aniso1"));
        buffer(set, 9, FrameName(f, "Cascades"));
        buffer(set, 10, FrameName(f, CascadeName(c, kind)));
        image(set, 11, "Atlas", true);
        image(set, 12, "Occlusion", true);
        buffer(set, 13, "Status");
      }
      set = make_set(FrameName(f, CascadeName(c, "Integrate")), SdfgiLayout::Integrate);
      constexpr const char* sampled_cascades[]{"Sdf", "Light", "Aniso0", "Aniso1"};
      for (uint32_t b = 0; b < 4; ++b)
        for (uint32_t i = 0; i < 8; ++i)
          image(set, b + 1, CascadeName(std::min(i, settings.cascade_count - 1), sampled_cascades[b]), true, i);
      sampler(set, 6);
      buffer(set, 7, FrameName(f, "Cascades"));
      image(set, 8, "Atlas");
      image(set, 9, CascadeName(c, "History"));
      image(set, 10, CascadeName(c, "Average"));
      image(set, 11, "HistoryScroll");
      image(set, 12, "AverageScroll");
      image(set, 13, CascadeName(std::min(c + 1, settings.cascade_count - 1), "Average"));
      image(set, 14, "Ambient");
      buffer(set, 15, "Status");
    }
  }
}

void SdfgiResources::Import(RenderGraph& graph, RenderGraphResourceRegistry& registry) const {
  for (const auto& [name, texture] : textures) {
    const auto& r = texture.requirement;
    RenderResourceDescriptor descriptor;
    descriptor.name = GraphName(name);
    descriptor.type = RenderResourceType::Image;
    descriptor.lifetime = RenderResourceLifetime::Persistent;
    descriptor.dimensions = {
        RenderResourceSizeMode::Absolute, r.extent.width, r.extent.height, r.extent.depth, r.layers, 1};
    descriptor.format_name = std::to_string(r.storage_format);
    descriptor.byte_size = texture.image->GetVmaAllocationInfo().size;
    graph.AddResource(descriptor);
    registry.BindImage(descriptor.name, texture.image);
  }
  for (const auto& [name, resource] : buffers) {
    RenderResourceDescriptor descriptor;
    descriptor.name = GraphName(name);
    descriptor.type = RenderResourceType::Buffer;
    descriptor.lifetime = RenderResourceLifetime::Persistent;
    descriptor.byte_size = resource.buffer->GetSize();
    graph.AddResource(descriptor);
    registry.BindBuffer(descriptor.name, resource.buffer);
  }
}

uint64_t SdfgiResources::GetAllocationBytes(const SdfgiMemoryClass memory_class) const {
  uint64_t result = allocated_bytes[static_cast<size_t>(memory_class)];
  if (memory_class == SdfgiMemoryClass::Upload) {
    std::set<const SdfgiGatherFrame*> frames;
    if (publication)
      frames.insert(publication.get());
    for (const auto& frame : gather_frames)
      if (frame)
        frames.insert(frame.get());
    for (const auto* frame : frames)
      result += frame->uploads.GetAllocationBytes();
  }
  if (memory_class == SdfgiMemoryClass::Upload)
    for (const auto& frame : light_frames)
      if (frame)
        result += frame->uploads.GetAllocationBytes();
  if (memory_class == SdfgiMemoryClass::Upload)
    for (const auto& frame : voxel_frames)
      if (frame)
        result += frame->AllocationBytes();
  if (memory_class == SdfgiMemoryClass::Diagnostic) {
    result += GetSdfgiDebugAllocationBytes(*this);
    std::set<const SdfgiVoxelDebug*> snapshots;
    if (voxel_debug)
      snapshots.insert(voxel_debug.get());
    for (const auto& frame : voxel_frames)
      if (frame) {
        if (frame->debug)
          snapshots.insert(frame->debug.get());
        if (frame->preprocess_readback)
          result += frame->preprocess_readback->buffer->GetVmaAllocationInfo().size;
      }
    for (const auto* snapshot : snapshots)
      result += snapshot->AllocationBytes();
    std::set<const SdfgiPreprocessDebug*> preprocess_snapshots;
    if (preprocess_debug)
      preprocess_snapshots.insert(preprocess_debug.get());
    for (const auto& snapshot : preprocess_debug_frames)
      if (snapshot)
        preprocess_snapshots.insert(snapshot.get());
    for (const auto* snapshot : preprocess_snapshots)
      result += snapshot->AllocationBytes();
    std::set<const SdfgiLightDebug*> light_snapshots;
    if (light_debug)
      light_snapshots.insert(light_debug.get());
    for (const auto& snapshot : light_debug_frames)
      if (snapshot)
        light_snapshots.insert(snapshot.get());
    for (const auto* snapshot : light_snapshots)
      result += snapshot->AllocationBytes();
    std::set<const SdfgiProbeDebug*> probe_snapshots;
    if (probe_debug)
      probe_snapshots.insert(probe_debug.get());
    for (const auto& snapshot : probe_debug_frames)
      if (snapshot)
        probe_snapshots.insert(snapshot.get());
    for (const auto* snapshot : probe_snapshots)
      result += snapshot->AllocationBytes();
  }
  return result;
}

void SdfgiResources::CreatePipelines(const std::vector<std::shared_ptr<DescriptorSetLayout>>& deferred_host_layouts) {
  const auto root = Resources::GetDefaultResourcesPath() / "Shaders";
  Shader::RegisterShaderIncludePath(root / "Modules");
  const std::string header = "#define EE_SDFGI_ABI_ONLY 1\n";
  const auto compute = [&](const std::string& name, const std::string& file, const std::string& variant,
                           std::vector<std::shared_ptr<DescriptorSetLayout>> pipeline_layouts, const uint32_t push_size,
                           const bool source_layout_check = true) {
    auto pipeline = std::make_shared<ComputePipeline>();
    pipeline->descriptor_set_layouts = std::move(pipeline_layouts);
    if (push_size)
      pipeline->push_constant_ranges.push_back({VK_SHADER_STAGE_COMPUTE_BIT, 0, push_size});
    pipeline->compute_shader = Shader::CreateTemporary(ShaderType::Compute, variant, root / "Compute" / file);
    pipeline->Initialize();
    if (!pipeline->Initialized())
      throw std::runtime_error("pipeline creation failed: " + name);
    if (source_layout_check) {
      const auto validation = Shader::ValidateSlangPipelineLayout(
          ShaderType::Compute, pipeline->compute_shader->PeekShaderCode(), root / "Compute" / file,
          pipeline->descriptor_set_layouts, pipeline->push_constant_ranges);
      if (!validation.success)
        throw std::runtime_error(name + ": " + validation.diagnostics);
    }
    pipelines.emplace(name, pipeline);
  };
  struct PreprocessVariant {
    const char* name;
    const char* define;
    SdfgiLayout layout;
  };
  const PreprocessVariant preprocess[]{{"Initialize", "MODE_INITIALIZE_JUMP_FLOOD", SdfgiLayout::Initialize},
                                       {"InitializeHalf", "MODE_INITIALIZE_JUMP_FLOOD_HALF", SdfgiLayout::Initialize},
                                       {"JumpFlood", "MODE_JUMPFLOOD", SdfgiLayout::JumpFlood},
                                       {"JumpFloodOptimized", "MODE_JUMPFLOOD_OPTIMIZED", SdfgiLayout::JumpFlood},
                                       {"Upscale", "MODE_UPSCALE_JUMP_FLOOD", SdfgiLayout::Upscale},
                                       {"Occlusion", "MODE_OCCLUSION", SdfgiLayout::Occlusion},
                                       {"Store", "MODE_STORE", SdfgiLayout::Store},
                                       {"Scroll", "MODE_SCROLL", SdfgiLayout::Scroll},
                                       {"ScrollOcclusion", "MODE_SCROLL_OCCLUSION", SdfgiLayout::ScrollOcclusion}};
  for (const auto& variant : preprocess)
    compute(variant.name, "SdfgiPreprocess.slang", std::string("#define ") + variant.define + " 1\n",
            {layouts[static_cast<size_t>(variant.layout)]}, sizeof(SdfgiPreprocessPushConstant));
  for (const std::string mode : {"STATIC", "DYNAMIC"})
    compute("DirectLight" + mode, "SdfgiDirectLight.slang", "#define MODE_PROCESS_" + mode + " 1\n",
            {layouts[static_cast<size_t>(SdfgiLayout::DirectLight)]}, sizeof(SdfgiDirectLightPushConstant));
  compute("PayloadRefresh", "SdfgiPayloadRefresh.slang", "",
          {layouts[static_cast<size_t>(SdfgiLayout::PayloadRefresh)]}, 0);
  for (const std::string mode : {"PROCESS", "STORE", "SCROLL", "SCROLL_STORE"})
    compute("Integrate" + mode, "SdfgiIntegrate.slang", "#define MODE_" + mode + " 1\n",
            {layouts[static_cast<size_t>(SdfgiLayout::Integrate)], layouts[static_cast<size_t>(SdfgiLayout::Sky)]},
            sizeof(SdfgiIntegratePushConstant));
  auto deferred = deferred_host_layouts;
  deferred.push_back(layouts[static_cast<size_t>(SdfgiLayout::Gather)]);
  // Shared scene-module reflection includes unused vertex-only bindings; validate the emitted deferred SPIR-V instead.
  compute("DeferredSdfgi", "DeferredComputeLighting.slang", "#define EE_AUTOMATIC_SDFGI 1\n", deferred,
          sizeof(RenderInstancePushConstant), false);
  compute("Publish", "SdfgiPublish.slang", "", {layouts[static_cast<size_t>(SdfgiLayout::Gather)]}, 0);
  compute("GatherAbi", "SdfgiGatherAbi.slang", header, std::move(deferred), 0);
  voxel_pipeline = std::make_shared<GraphicsPipeline>();
  voxel_pipeline->vertex_input_enabled = true;
  voxel_pipeline->view_mask = 0;
  voxel_pipeline->depth_attachment_format = VK_FORMAT_UNDEFINED;
  voxel_pipeline->stencil_attachment_format = VK_FORMAT_UNDEFINED;
  voxel_pipeline->descriptor_set_layouts = {deferred_host_layouts[0], layouts[static_cast<size_t>(SdfgiLayout::Voxel)]};
  voxel_pipeline->push_constant_ranges.push_back(
      {VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT, 0, sizeof(SdfgiVoxelPushConstant)});
  voxel_pipeline->vertex_shader =
      Shader::CreateTemporary(ShaderType::Vertex, "", root / "Graphics/Vertex/SDFGI/SdfgiVoxelize.slang");
  voxel_pipeline->fragment_shader =
      Shader::CreateTemporary(ShaderType::Fragment, "", root / "Graphics/Fragment/SDFGI/SdfgiVoxelize.slang");
  voxel_pipeline->Initialize();
  if (!voxel_pipeline->Initialized())
    throw std::runtime_error("SDFGI voxel pipeline creation failed");
}

RenderPassDescriptor SdfgiResources::ClearDescriptor() const {
  RenderPassDescriptor result{"SdfgiInitialize", RenderPassQueue::Graphics, RenderPassScope::Frame};
  result.dependencies = {RenderPassNames::sdfgi_maintenance};
  result.profiler_group = RenderPassProfilerGroup::FramePreparation;
  result.profiler_display_name = "SDFGI Initialize";
  for (const auto& [name, texture] : textures)
    result.resources.push_back(
        {GraphName(name), RenderResourceUsage::Write, RenderResourceState::TransferDestinationGeneral});
  for (const auto& [name, buffer] : buffers)
    result.resources.push_back(
        {GraphName(name), RenderResourceUsage::Write, RenderResourceState::TransferDestinationGeneral});
  return result;
}

void SdfgiResources::OrderAccess(const VkCommandBuffer command_buffer, const VkPipelineStageFlags2 destination_stages,
                                 const VkAccessFlags2 destination_access) const {
  // Covers prior frame/camera graphs and sampled/storage aliases, including same-layout read-before-write reuse.
  VkMemoryBarrier2 barrier{VK_STRUCTURE_TYPE_MEMORY_BARRIER_2};
  barrier.srcStageMask = kFieldStages;
  barrier.srcAccessMask = kFieldAccess;
  barrier.dstStageMask = destination_stages;
  barrier.dstAccessMask = destination_access;
  VkDependencyInfo dependency{VK_STRUCTURE_TYPE_DEPENDENCY_INFO};
  dependency.memoryBarrierCount = 1;
  dependency.pMemoryBarriers = &barrier;
  vkCmdPipelineBarrier2(command_buffer, &dependency);
}

void SdfgiResources::Clear(const VkCommandBuffer command_buffer, const RenderGraphExecutionContext& context) {
  OrderAccess(command_buffer, VK_PIPELINE_STAGE_2_TRANSFER_BIT, VK_ACCESS_2_TRANSFER_WRITE_BIT);
  ApplyGraphResourceBarriers(command_buffer, context);
  const VkClearColorValue zero{};
  for (const auto& [name, texture] : textures) {
    // Explicit array coverage: the generic graph clear helper currently clears layer zero only.
    const VkImageSubresourceRange range{VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, texture.requirement.layers};
    Platform::ClearColorImage(command_buffer, *texture.image, zero, 1, &range);
  }
  for (const auto& [name, resource] : buffers)
    vkCmdFillBuffer(command_buffer, resource.buffer->GetVkBuffer(), 0, VK_WHOLE_SIZE, 0);
  OrderAccess(command_buffer, VK_PIPELINE_STAGE_2_TRANSFER_BIT, VK_ACCESS_2_TRANSFER_WRITE_BIT);
  const SdfgiFieldStatus status{0, 0, 0, settings.SolidCellCapacity()};
  vkCmdUpdateBuffer(command_buffer, buffers.at("Status").buffer->GetVkBuffer(), 0, sizeof(status), &status);
  OrderAccess(command_buffer, kFieldStages, kFieldAccess);
  initialization_recorded = true;
}
