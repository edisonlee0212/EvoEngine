// Three-axis rasterization adapted from Godot render_forward_clustered.cpp::_render_sdfgi,
// 34d06658a85845111a50db9e485ec4a0701d4298. See docs/licenses/Godot-MIT.txt.
#include "SdfgiVoxelizer.hpp"

#include "GeometryStorage.hpp"
#include "GltfMaterialCache.hpp"
#include "Mesh.hpp"
#include "Platform.hpp"
#include "RenderPasses/RenderPassUtilities.hpp"
#include "SdfgiPreprocess.hpp"
#include "Texture2D.hpp"

#include <stb_image_write.h>
#include <cstring>
#include <set>

using namespace evo_engine;

namespace {
std::string InputName(const std::string& name) {
  return "Frame.SDFGI.Voxel." + name;
}

bool Intersects(const Bound& a, const Bound& b) {
  return glm::all(glm::lessThanEqual(a.min, b.max)) && glm::all(glm::greaterThanEqual(a.max, b.min));
}

SdfgiVoxelData VoxelView(const SdfgiPendingRegion& region, const SdfgiCascade& cascade, const float y_mult,
                         const uint32_t axis) {
  const uint32_t right = (axis + 1) % 3;
  const uint32_t up = (axis + 2) % 3;
  const glm::vec3 extent = region.world_bounds.max - region.world_bounds.min;
  const glm::vec3 center = (region.world_bounds.max + region.world_bounds.min) * 0.5f;
  glm::mat4 projection(0);
  projection[right][0] = 2 / extent[right];
  projection[up][1] = -2 / extent[up];
  projection[axis][2] = -1 / extent[axis];
  projection[3][0] = -center[right] * projection[right][0];
  projection[3][1] = -center[up] * projection[up][1];
  projection[3][2] = region.world_bounds.max[axis] / extent[axis];
  projection[3][3] = 1;
  SdfgiVoxelData result{};
  std::memcpy(result.view_projection, &projection, sizeof(projection));
  for (int i = 0; i < 3; ++i) {
    result.cascade_min_cell[i] = (cascade.position[i] - 64) * cascade.cell_size;
    result.region_offset_y_mult[i] = static_cast<float>(region.offset[i]);
  }
  result.cascade_min_cell[3] = cascade.cell_size;
  result.region_offset_y_mult[3] = y_mult;
  return result;
}
}  // namespace

SdfgiVoxelDebug::SdfgiVoxelDebug(const uint32_t cascade_index, const uint32_t slice_index)
    : cascade(cascade_index), slice(slice_index) {
  if (slice >= 128)
    throw std::invalid_argument("SDFGI voxel slice must be in [0, 127]");
  for (size_t i = 0; i < planes.size(); ++i) {
    VkBufferCreateInfo info{VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO};
    info.size = 3 * 128 * 128 * (i == 0 ? 2 : 4);
    info.usage = VK_BUFFER_USAGE_TRANSFER_DST_BIT;
    VmaAllocationCreateInfo allocation{};
    allocation.usage = VMA_MEMORY_USAGE_AUTO;
    allocation.flags = VMA_ALLOCATION_CREATE_HOST_ACCESS_RANDOM_BIT;
    allocation.requiredFlags = VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT;
    planes[i] = std::make_shared<Buffer>(info, allocation);
  }
}

void SdfgiVoxelDebug::Record(const VkCommandBuffer command, const SdfgiResources& resources) {
  resources.OrderAccess(command, VK_PIPELINE_STAGE_2_TRANSFER_BIT, VK_ACCESS_2_TRANSFER_READ_BIT);
  const char* names[]{"Albedo", "Emission", "EmissionAniso", "Facing"};
  for (size_t i = 0; i < planes.size(); ++i) {
    std::array<VkBufferImageCopy, 3> copies{};
    for (uint32_t axis = 0; axis < 3; ++axis) {
      auto& copy = copies[axis];
      copy.bufferOffset = axis * 128 * 128 * (i == 0 ? 2 : 4);
      copy.imageSubresource = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 0, 1};
      copy.imageExtent = {128, 128, 128};
      if (axis == 0) {
        copy.imageOffset.x = slice;
        copy.imageExtent.width = 1;
      } else if (axis == 1) {
        copy.imageOffset.y = slice;
        copy.imageExtent.height = 1;
      } else {
        copy.imageOffset.z = slice;
        copy.imageExtent.depth = 1;
      }
    }
    vkCmdCopyImageToBuffer(command, resources.textures.at(names[i]).image->GetVkImage(), VK_IMAGE_LAYOUT_GENERAL,
                           planes[i]->GetVkBuffer(), copies.size(), copies.data());
  }
  VkMemoryBarrier2 barrier{VK_STRUCTURE_TYPE_MEMORY_BARRIER_2};
  barrier.srcStageMask = VK_PIPELINE_STAGE_2_TRANSFER_BIT;
  barrier.srcAccessMask = VK_ACCESS_2_TRANSFER_WRITE_BIT;
  barrier.dstStageMask = VK_PIPELINE_STAGE_2_HOST_BIT;
  barrier.dstAccessMask = VK_ACCESS_2_HOST_READ_BIT;
  VkDependencyInfo dependency{VK_STRUCTURE_TYPE_DEPENDENCY_INFO};
  dependency.memoryBarrierCount = 1;
  dependency.pMemoryBarriers = &barrier;
  vkCmdPipelineBarrier2(command, &dependency);
  recorded = true;
}

std::array<std::vector<uint32_t>, 4> SdfgiVoxelDebug::Read() const {
  if (!recorded)
    throw std::runtime_error("SDFGI voxel diagnostic has not been recorded");
  Platform::WaitForFrameSubmissions("SDFGI explicit voxel diagnostic readback");
  std::array<std::vector<uint32_t>, 4> result;
  std::vector<uint16_t> albedo;
  planes[0]->DownloadVector(albedo, 3 * 128 * 128);
  result[0].assign(albedo.begin(), albedo.end());
  for (size_t i = 1; i < planes.size(); ++i)
    planes[i]->DownloadVector(result[i], 3 * 128 * 128);
  return result;
}

void SdfgiVoxelDebug::StoreToPng(const std::filesystem::path& path) const {
  const auto data = Read();
  std::vector<uint8_t> pixels(2560 * 1440 * 4, 255);
  for (int y = 0; y < 1440; ++y)
    for (int x = 0; x < 2560; ++x) {
      const uint32_t axis = y / 480;
      const int u = x % 640 - 128, v = y % 480 - 48;
      glm::vec3 color(0.025f);
      if (u >= 0 && v >= 0 && u < 384 && v < 384) {
        const uint32_t right = u / 3, up = 127 - v / 3;
        const uint32_t index = axis * 128 * 128 + (axis == 1 ? up + right * 128 : right + up * 128);
        const auto albedo = data[0][index], emission = data[1][index], facing = data[3][index];
        color = glm::vec3(0);
        switch (x / 640) {
          case 0:
            color = glm::pow(glm::vec3((albedo >> 11) & 31, (albedo >> 6) & 31, (albedo >> 1) & 31) / 31.0f,
                             glm::vec3(1 / 2.2f));
            break;
          case 1: {
            const float scale = std::ldexp(1.0f, static_cast<int>(emission >> 25) - 24);
            const auto radiance =
                glm::vec3((emission & 255) * 2, (emission >> 8) & 511, ((emission >> 17) & 255) * 2) * scale;
            color = glm::pow(radiance / (glm::vec3(1) + radiance), glm::vec3(1 / 2.2f));
            break;
          }
          case 2:
            for (uint32_t direction = 0; direction < 6; ++direction)
              if (facing & (1u << direction))
                color[direction % 3] = std::max(color[direction % 3], direction < 3 ? 1.0f : 0.4f);
            break;
          case 3:
            color = glm::vec3(albedo & 1 ? 1 : 0);
            break;
        }
      }
      for (size_t channel = 0; channel < 3; ++channel)
        pixels[(y * 2560 + x) * 4 + channel] =
            static_cast<uint8_t>(glm::clamp(color[channel], 0.0f, 1.0f) * 255 + 0.5f);
    }
  stbi_flip_vertically_on_write(false);
  if (!stbi_write_png(path.string().c_str(), 2560, 1440, 4, pixels.data(), 2560 * 4))
    throw std::runtime_error("Could not write SDFGI voxel diagnostic PNG");
}

uint64_t SdfgiVoxelDebug::AllocationBytes() const {
  uint64_t result = 0;
  for (const auto& plane : planes)
    result += plane->GetVmaAllocationInfo().size;
  return result;
}

std::shared_ptr<SdfgiVoxelFrame> SdfgiVoxelFrame::Create(const SdfgiResources& resources,
                                                         const SdfgiContributorRegistry& contributors,
                                                         const std::vector<SdfgiCascade>& cascades,
                                                         const std::vector<SdfgiPendingRegion>& pending) {
  auto frame = std::make_shared<SdfgiVoxelFrame>();
  frame->cascades = cascades;
  frame->preprocess_readback = std::make_shared<SdfgiPreprocessReadback>(resources.settings.cascade_count);
  frame->vertex_buffer = GeometryStorage::GetVertexBuffer();
  frame->index_buffer = GeometryStorage::GetTriangleBuffer();
  frame->scene_set = std::make_shared<DescriptorSet>(resources.voxel_pipeline->descriptor_set_layouts[0]);
  auto& batch = frame->input_uploads;
  const auto make_buffer = [&](const std::string& name, const size_t size, const VkBufferUsageFlags usage) {
    VkBufferCreateInfo info{VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO};
    info.size = size;
    info.usage = usage | VK_BUFFER_USAGE_TRANSFER_DST_BIT;
    auto buffer = std::make_shared<Buffer>(info);
    frame->buffers.emplace(name, buffer);
    return buffer;
  };
  GltfMaterialCache materials;
  const auto texture_capacity =
      resources.voxel_pipeline->descriptor_set_layouts[0]->GetDescriptorBindings().at(9).binding.descriptorCount;
  for (const auto& [id, contributor] : contributors.entries) {
    const auto range = contributor.mesh->GetTriangleRange();
    if (!frame->vertex_buffer || !frame->index_buffer || !range || range->prev_frame_index_count == 0)
      throw std::runtime_error("SDFGI contributor raster geometry is not ready");
    Draw draw;
    draw.contributor = contributor;
    draw.first_index = range->prev_frame_offset * 3;
    draw.index_count = range->prev_frame_index_count * 3;
    std::memcpy(draw.constants.model, &contributor.transform, sizeof(contributor.transform));
    const auto normal = glm::transpose(glm::inverse(glm::mat3(contributor.transform)));
    for (int column = 0; column < 3; ++column)
      for (int row = 0; row < 3; ++row) {
        if (!std::isfinite(normal[column][row]))
          throw std::runtime_error("SDFGI contributor normal transform is singular or nonfinite");
        draw.constants.normal_basis[column * 3 + row] = normal[column][row];
      }
    GltfMaterialData material;
    auto& shade = material.shade_material;
    shade.pbr_base_color_factor = contributor.material.base_color;
    shade.emissive_factor = contributor.material.emission;
    shade.alpha_mode = contributor.material.masked ? static_cast<int32_t>(GltfAlphaMode::Mask)
                                                   : static_cast<int32_t>(GltfAlphaMode::Opaque);
    shade.alpha_cutoff = contributor.material.alpha_cutoff;
    shade.double_sided = contributor.material.double_sided;
    const auto texture = [&](const SdfgiTextureInput& input, uint16_t& slot) {
      if (!input.texture)
        return;
      const auto index = input.texture->GetTextureStorageIndex();
      VkDescriptorImageInfo info{};
      if (index >= texture_capacity || !TextureStorage::TryGetTexture2DDescriptorImageInfo(index, info) ||
          !input.image_view || info.imageView != input.image_view->GetVkImageView())
        throw std::runtime_error("SDFGI contributor texture is not ready in the frozen snapshot");
      auto mapping = input.mapping;
      mapping.index = static_cast<int32_t>(index);
      slot = AppendGltfTextureInfo(material, mapping);
      frame->scene_set->UpdateImageDescriptorBinding(9, info, index);
      frame->textures.push_back(input);
    };
    texture(contributor.material.base_texture, shade.pbr_base_color_texture);
    texture(contributor.material.emission_texture, shade.emissive_texture);
    if (materials.GetTextureInfos().size() + material.texture_infos.size() > UINT16_MAX)
      throw std::runtime_error("SDFGI material snapshot exceeds the host 16-bit texture-info range");
    draw.constants.material_index = materials.Append(material);
    frame->draws.push_back(std::move(draw));
  }
  if (frame->draws.empty())
    materials.Append(GltfMaterialData{});
  frame->material_data = materials.GetShadeMaterials();
  frame->texture_data = materials.GetTextureInfos();
  const auto material_size = materials.GetShadeMaterials().size() * sizeof(GltfShadeMaterial);
  const auto texture_info_size = materials.GetTextureInfos().size() * sizeof(GltfTextureInfo);
  const auto limit = Platform::GetSelectedPhysicalDevice()->properties.limits.maxStorageBufferRange;
  if (material_size > limit || texture_info_size > limit)
    throw std::runtime_error("SDFGI material snapshot exceeds the host storage-buffer range");
  auto buffer = make_buffer("Materials", material_size, VK_BUFFER_USAGE_STORAGE_BUFFER_BIT);
  batch.AddVector(buffer, frame->material_data);
  frame->scene_set->UpdateBufferDescriptorBinding(11, buffer);
  buffer = make_buffer("TextureInfos", texture_info_size, VK_BUFFER_USAGE_STORAGE_BUFFER_BIT);
  batch.AddVector(buffer, frame->texture_data);
  frame->scene_set->UpdateBufferDescriptorBinding(12, buffer);
  frame->regions.resize(pending.size());
  for (size_t i = 0; i < pending.size(); ++i) {
    auto& region = frame->regions[i];
    region.pending = pending[i];
    for (uint32_t axis = 0; axis < 3; ++axis) {
      region.constants[axis] = VoxelView(pending[i], cascades[pending[i].cascade],
                                         SdfgiYMultiplier(resources.settings.vertical_scale), axis);
      buffer = make_buffer("Region" + std::to_string(i) + "Axis" + std::to_string(axis), sizeof(SdfgiVoxelData),
                           VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT);
      batch.Add(buffer, region.constants[axis], {BufferUploadUsage::Uniform});
      auto& set = region.sets[axis];
      set = std::make_shared<DescriptorSet>(resources.layouts[static_cast<size_t>(SdfgiLayout::Voxel)]);
      set->UpdateBufferDescriptorBinding(0, buffer);
      uint32_t binding = 1;
      for (const auto name : {"Albedo", "Emission", "EmissionAniso", "Facing"}) {
        VkDescriptorImageInfo info{};
        info.imageLayout = VK_IMAGE_LAYOUT_GENERAL;
        info.imageView = resources.textures.at(name).storage_view->GetVkImageView();
        set->UpdateImageDescriptorBinding(binding++, info);
      }
    }
  }
  return frame;
}

void SdfgiVoxelFrame::AddPasses(RenderGraph& graph, RenderGraphResourceRegistry& registry,
                                const std::shared_ptr<SdfgiResources>& resources) {
  std::vector<RenderResourceAccess> inputs;
  const auto import_buffer = [&](const std::string& name, const std::shared_ptr<Buffer>& buffer) {
    if (!buffer)
      return;
    RenderResourceDescriptor descriptor;
    descriptor.name = InputName(name);
    descriptor.type = RenderResourceType::Buffer;
    descriptor.lifetime = RenderResourceLifetime::Persistent;
    descriptor.byte_size = buffer->GetSize();
    graph.AddResource(descriptor);
    registry.BindBuffer(descriptor.name, buffer);
    inputs.push_back({descriptor.name, RenderResourceUsage::Read, RenderResourceState::General});
  };
  for (const auto& [name, buffer] : buffers)
    import_buffer(name, buffer);
  RenderPassDescriptor upload{"SdfgiVoxelInputs", RenderPassQueue::Graphics, RenderPassScope::Frame};
  upload.dependencies = {resources->initialization_recorded ? RenderPassNames::sdfgi_maintenance : "SdfgiInitialize"};
  for (const auto& input : inputs)
    upload.resources.push_back(
        {input.resource_name, RenderResourceUsage::Write, RenderResourceState::TransferDestinationGeneral});
  graph.AddPass(upload, [frame = shared_from_this()](const RenderGraphExecutionContext&) {
    frame->input_uploads.Record(frame->uploads);
  });
  import_buffer("Vertices", vertex_buffer);
  import_buffer("Indices", index_buffer);
  std::set<const Image*> imported_images;
  for (const auto& texture : textures) {
    if (!texture.image || !imported_images.insert(texture.image.get()).second)
      continue;
    RenderResourceDescriptor descriptor;
    descriptor.name = InputName("Texture" + std::to_string(imported_images.size()));
    descriptor.type = RenderResourceType::Image;
    descriptor.lifetime = RenderResourceLifetime::Persistent;
    graph.AddResource(descriptor);
    registry.BindImage(descriptor.name, texture.image);
    inputs.push_back({descriptor.name, RenderResourceUsage::Read,
                      texture.image->GetLayout() == VK_IMAGE_LAYOUT_GENERAL ? RenderResourceState::General
                                                                            : RenderResourceState::ShaderRead});
  }
  std::string previous = "SdfgiVoxelInputs";
  for (uint32_t cascade = 0; cascade < resources->settings.cascade_count; ++cascade) {
    if (std::none_of(regions.begin(), regions.end(), [&](const auto& region) {
          return region.pending.cascade == cascade;
        }))
      continue;
    RenderPassDescriptor pass{"SdfgiVoxelCascade" + std::to_string(cascade), RenderPassQueue::Graphics,
                              RenderPassScope::Frame};
    pass.dependencies = {previous};
    pass.profiler_group = RenderPassProfilerGroup::FramePreparation;
    pass.profiler_display_name = "SDFGI Voxelize";
    pass.resources = inputs;
    for (const auto name : {"Albedo", "Emission", "EmissionAniso", "Facing"})
      pass.resources.push_back(
          {"Frame.SDFGI." + std::string(name), RenderResourceUsage::Write, RenderResourceState::General});
    previous = pass.name;
    graph.AddPass(pass, [frame = shared_from_this(), resources, cascade](const RenderGraphExecutionContext& context) {
      Platform::RecordCommandsMainQueue([&](const VkCommandBuffer command_buffer) {
        resources->OrderAccess(command_buffer, VK_PIPELINE_STAGE_2_TRANSFER_BIT, VK_ACCESS_2_TRANSFER_WRITE_BIT);
        ApplyGraphResourceBarriers(command_buffer, context);
        const VkImageSubresourceRange range{VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1};
        for (const auto name : {"Albedo", "Emission", "EmissionAniso", "Facing"})
          Platform::ClearColorImage(command_buffer, *resources->textures.at(name).image, VkClearColorValue{}, 1,
                                    &range);
        resources->OrderAccess(command_buffer, VK_PIPELINE_STAGE_2_ALL_GRAPHICS_BIT,
                               VK_ACCESS_2_SHADER_READ_BIT | VK_ACCESS_2_SHADER_WRITE_BIT |
                                   VK_ACCESS_2_VERTEX_ATTRIBUTE_READ_BIT | VK_ACCESS_2_INDEX_READ_BIT);
        auto& pipeline = *resources->voxel_pipeline;
        for (const auto& region : frame->regions) {
          if (region.pending.cascade != cascade)
            continue;
          for (uint32_t axis = 0; axis < 3; ++axis) {
            const uint32_t width = region.pending.size[(axis + 1) % 3];
            const uint32_t height = region.pending.size[(axis + 2) % 3];
            VkRenderingInfo rendering{VK_STRUCTURE_TYPE_RENDERING_INFO};
            rendering.renderArea.extent = {width, height};
            rendering.layerCount = 1;
            Platform::BeginRendering(command_buffer, rendering);
            pipeline.Bind(command_buffer);
            pipeline.states.SetViewportScissor({0, 0, width, height});
            pipeline.states.depth_test = pipeline.states.depth_write = false;
            pipeline.states.cull_mode = VK_CULL_MODE_NONE;
            pipeline.states.ApplyAllStates(command_buffer, true);
            pipeline.BindDescriptorSet(command_buffer, 0, frame->scene_set->GetVkDescriptorSet());
            pipeline.BindDescriptorSet(command_buffer, 1, region.sets[axis]->GetVkDescriptorSet());
            if (!frame->draws.empty()) {
              frame->vertex_buffer->BindVertex(command_buffer);
              frame->index_buffer->BindIndex(command_buffer);
              for (const auto& draw : frame->draws) {
                if (!Intersects(draw.contributor.world_bounds, region.pending.world_bounds))
                  continue;
                pipeline.PushConstant(command_buffer, 0, draw.constants);
                Platform::DrawIndexed(command_buffer, draw.index_count, 1, draw.first_index);
              }
            }
            Platform::EndRendering(command_buffer);
            resources->OrderAccess(command_buffer,
                                   VK_PIPELINE_STAGE_2_ALL_GRAPHICS_BIT | VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT,
                                   VK_ACCESS_2_SHADER_READ_BIT | VK_ACCESS_2_SHADER_WRITE_BIT);
          }
        }
      });
    });
    if (debug && debug->cascade == cascade) {
      RenderPassDescriptor capture{"SdfgiVoxelDebug", RenderPassQueue::Graphics, RenderPassScope::Frame};
      capture.dependencies = {previous};
      for (size_t i = 0; i < debug->planes.size(); ++i) {
        RenderResourceDescriptor descriptor;
        descriptor.name = InputName("Debug" + std::to_string(i));
        descriptor.type = RenderResourceType::Buffer;
        descriptor.lifetime = RenderResourceLifetime::Persistent;
        descriptor.byte_size = debug->planes[i]->GetSize();
        graph.AddResource(descriptor);
        registry.BindBuffer(descriptor.name, debug->planes[i]);
        capture.resources.push_back(
            {descriptor.name, RenderResourceUsage::Write, RenderResourceState::TransferDestinationGeneral});
      }
      for (const auto name : {"Albedo", "Emission", "EmissionAniso", "Facing"})
        capture.resources.push_back(
            {"Frame.SDFGI." + std::string(name), RenderResourceUsage::Read, RenderResourceState::General});
      previous = capture.name;
      graph.AddPass(capture, [snapshot = debug, resources](const RenderGraphExecutionContext& context) {
        Platform::RecordCommandsMainQueue([&](const VkCommandBuffer command) {
          ApplyGraphResourceBarriers(command, context);
          snapshot->Record(command, *resources);
        });
      });
    }
    const bool full_cascade = std::any_of(regions.begin(), regions.end(), [&](const auto& region) {
      return region.pending.cascade == cascade && region.pending.offset == glm::ivec3(0) &&
             region.pending.size == glm::ivec3(128);
    });
    if (full_cascade)
      previous = AddSdfgiPreprocessPass(graph, registry, resources, preprocess_readback, cascade,
                                        cascades[cascade].position, previous);
    else {
      resources->preprocessed_cascades &= ~(1u << cascade);
      resources->preprocess_failure = "SDFGI scrolling reconstruction is not implemented yet";
    }
  }
  graph.AddPass({"SdfgiVoxelComplete", RenderPassQueue::Graphics, RenderPassScope::Frame, {}, {previous}},
                [resources](const RenderGraphExecutionContext&) {
                  resources->voxelization_recorded = true;
                });
}

uint64_t SdfgiVoxelFrame::AllocationBytes() const {
  uint64_t result = uploads.GetAllocationBytes();
  for (const auto& [name, buffer] : buffers)
    result += buffer->GetVmaAllocationInfo().size;
  return result;
}
