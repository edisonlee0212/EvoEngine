// Godot HDDAGI raster/region ordering, da1410fa3516d08cc31b6e86bd6673b9ce776316.
// See docs/licenses/Godot-MIT.txt.
#include "HddagiVoxelizer.hpp"
#include <set>
#include "HddagiTypes.hpp"
#include "Platform.hpp"
#include "RenderPasses/RenderPassUtilities.hpp"
#include "Resources.hpp"
#include "Shader.hpp"

using namespace evo_engine;

std::shared_ptr<HddagiVoxelFrame> HddagiVoxelFrame::Create(HddagiResources& resources,
                                                           const std::shared_ptr<DescriptorSetLayout>& host_layout,
                                                           const SdfgiContributorRegistry& contributors,
                                                           const std::vector<SdfgiCascade>& cascades,
                                                           const std::vector<SdfgiPendingRegion>& pending,
                                                           const uint32_t version) {
  if (version == 0 || version > UINT16_MAX)
    throw std::invalid_argument("HDDAGI region version must be a nonzero 16-bit value");
  for (const auto& region : pending)
    if (region.cascade >= cascades.size() || glm::any(glm::lessThan(region.offset, glm::ivec3(0))) ||
        glm::any(glm::lessThanEqual(region.size, glm::ivec3(0))) ||
        glm::any(glm::greaterThan(region.offset + region.size, cascades[region.cascade].size)) ||
        glm::any(glm::notEqual(region.offset % 8, glm::ivec3(0))) ||
        glm::any(glm::notEqual(region.size % 8, glm::ivec3(0))))
      throw std::invalid_argument("HDDAGI raster regions must cover aligned whole regions inside their cascade");
  auto frame = std::make_shared<HddagiVoxelFrame>();
  frame->version = version;
  frame->raster = resources.voxel_pipeline;
  frame->region_store = resources.region_pipeline;
  if (!frame->raster || !frame->region_store) {
    auto layout = std::make_shared<DescriptorSetLayout>();
    const auto raster_stages = VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT;
    layout->PushDescriptorBinding(0, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, raster_stages, 0);
    for (uint32_t i = 1; i <= 4; ++i)
      layout->PushDescriptorBinding(i, VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, VK_SHADER_STAGE_FRAGMENT_BIT, 0);
    layout->Initialize();
    const auto failure = SdfgiResources::ValidateDescriptorLimits(
        {host_layout, layout}, Platform::GetSelectedPhysicalDevice()->properties.limits);
    if (!failure.empty())
      throw std::runtime_error(failure);
    const auto root = Resources::GetDefaultResourcesPath() / "Shaders";
    frame->raster = std::make_shared<GraphicsPipeline>();
    auto& pipeline = *frame->raster;
    pipeline.vertex_input_enabled = true;
    pipeline.view_mask = 0;
    pipeline.depth_attachment_format = pipeline.stencil_attachment_format = VK_FORMAT_UNDEFINED;
    pipeline.descriptor_set_layouts = {host_layout, layout};
    pipeline.push_constant_ranges.push_back({raster_stages, 0, sizeof(SdfgiVoxelPushConstant)});
    pipeline.vertex_shader =
        Shader::CreateTemporary(ShaderType::Vertex, "", root / "Graphics/Vertex/SDFGI/SdfgiVoxelize.slang");
    pipeline.fragment_shader =
        Shader::CreateTemporary(ShaderType::Fragment, "", root / "Graphics/Fragment/HDDAGI/HddagiVoxelize.slang");
    pipeline.Initialize();
    if (!pipeline.Initialized())
      throw std::runtime_error("HDDAGI raster pipeline creation failed");
    auto region_layout = std::make_shared<DescriptorSetLayout>();
    region_layout->PushDescriptorBinding(0, VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE, VK_SHADER_STAGE_COMPUTE_BIT, 0);
    for (uint32_t i = 1; i <= 3; ++i)
      region_layout->PushDescriptorBinding(i, VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, VK_SHADER_STAGE_COMPUTE_BIT, 0);
    region_layout->Initialize();
    frame->region_store = std::make_shared<ComputePipeline>();
    auto& compute = *frame->region_store;
    compute.descriptor_set_layouts = {region_layout};
    compute.push_constant_ranges.push_back({VK_SHADER_STAGE_COMPUTE_BIT, 0, sizeof(HddagiRegionParams)});
    compute.compute_shader = Shader::CreateTemporary(ShaderType::Compute, "", root / "Compute/HddagiRegionStore.slang");
    compute.Initialize();
    if (!compute.Initialized())
      throw std::runtime_error("HDDAGI region pipeline creation failed");
    resources.voxel_pipeline = frame->raster;
    resources.region_pipeline = frame->region_store;
  }
  const auto& layout = frame->raster->descriptor_set_layouts[1];
  const auto& region_layout = frame->region_store->descriptor_set_layouts[0];
  frame->inputs = SdfgiVoxelFrame::CreateRasterInputs(
      frame->raster, layout,
      {resources.images.at("Albedo").storage, resources.images.at("Emission").storage,
       resources.images.at("EmissionAniso").storage, resources.images.at("NormalBits").storage},
      SdfgiYMultiplier(static_cast<SdfgiSettings::VerticalScale>(resources.probes.vertical_scale)), contributors,
      cascades, pending);
  frame->region_set = std::make_shared<DescriptorSet>(region_layout);
  uint32_t binding = 0;
  for (const auto name : {"NormalBits", "VoxelBits", "Regions", "Versions"}) {
    VkDescriptorImageInfo info{};
    info.imageLayout = VK_IMAGE_LAYOUT_GENERAL;
    info.imageView =
        (binding == 0 ? resources.images.at(name).sampled : resources.images.at(name).storage)->GetVkImageView();
    frame->region_set->UpdateImageDescriptorBinding(binding++, info);
  }
  return frame;
}

void HddagiVoxelFrame::AddPasses(RenderGraph& graph, RenderGraphResourceRegistry& registry,
                                 const std::shared_ptr<HddagiResources>& resources) {
  std::vector<RenderResourceAccess> reads;
  const auto import_buffer = [&](const std::string& name, const std::shared_ptr<Buffer>& buffer) {
    if (!buffer)
      return;
    RenderResourceDescriptor descriptor;
    descriptor.name = "Frame.HDDAGI.Voxel." + name;
    descriptor.type = RenderResourceType::Buffer;
    descriptor.lifetime = RenderResourceLifetime::Persistent;
    descriptor.byte_size = buffer->GetSize();
    graph.AddResource(descriptor);
    registry.BindBuffer(descriptor.name, buffer);
    reads.push_back({descriptor.name, RenderResourceUsage::Read, RenderResourceState::General});
  };
  for (const auto& [name, buffer] : inputs->buffers)
    import_buffer(name, buffer);
  RenderPassDescriptor upload{"HddagiVoxelInputs", RenderPassQueue::Graphics, RenderPassScope::Frame};
  if (!resources->initialization_recorded)
    upload.dependencies = {"HddagiInitialize"};
  for (const auto& read : reads)
    upload.resources.push_back(
        {read.resource_name, RenderResourceUsage::Write, RenderResourceState::TransferDestinationGeneral});
  graph.AddPass(upload, [frame = shared_from_this()](const RenderGraphExecutionContext& context) {
    Platform::RecordCommandsMainQueue([&](const VkCommandBuffer command) {
      ApplyGraphResourceBarriers(command, context);
    });
    frame->inputs->input_uploads.Record(frame->inputs->uploads);
  });
  import_buffer("Vertices", inputs->vertex_buffer);
  import_buffer("Indices", inputs->index_buffer);
  std::set<const Image*> imported;
  for (const auto& texture : inputs->textures) {
    if (!texture.image || !imported.insert(texture.image.get()).second)
      continue;
    RenderResourceDescriptor descriptor;
    descriptor.name = "Frame.HDDAGI.Voxel.Texture" + std::to_string(imported.size());
    descriptor.type = RenderResourceType::Image;
    descriptor.lifetime = RenderResourceLifetime::Persistent;
    graph.AddResource(descriptor);
    registry.BindImage(descriptor.name, texture.image);
    reads.push_back({descriptor.name, RenderResourceUsage::Read,
                     texture.image->GetLayout() == VK_IMAGE_LAYOUT_GENERAL ? RenderResourceState::General
                                                                           : RenderResourceState::ShaderRead});
  }
  std::string previous = upload.name;
  for (uint32_t cascade = 0; cascade < inputs->cascades.size(); ++cascade) {
    if (std::none_of(inputs->regions.begin(), inputs->regions.end(), [cascade](const auto& r) {
          return r.pending.cascade == cascade;
        }))
      continue;
    RenderPassDescriptor pass{"HddagiVoxelCascade" + std::to_string(cascade), RenderPassQueue::Graphics,
                              RenderPassScope::Frame};
    pass.dependencies = {previous};
    previous = pass.name;
    pass.profiler_group = RenderPassProfilerGroup::FramePreparation;
    pass.profiler_display_name = "HDDAGI Voxelize";
    pass.resources = reads;
    for (const auto name : {"Albedo", "Emission", "EmissionAniso", "NormalBits", "VoxelBits", "Regions", "Versions"})
      pass.resources.push_back(
          {"Frame.HDDAGI." + std::string(name), RenderResourceUsage::Write, RenderResourceState::General});
    graph.AddPass(pass, [frame = shared_from_this(), resources, cascade](const RenderGraphExecutionContext& context) {
      Platform::RecordCommandsMainQueue([&](const VkCommandBuffer command) {
        const RenderPassGpuTimestampScope timing(command, context);
        resources->OrderAccess(command, VK_PIPELINE_STAGE_2_TRANSFER_BIT, VK_ACCESS_2_TRANSFER_WRITE_BIT);
        ApplyGraphResourceBarriers(command, context);
        const VkImageSubresourceRange range{VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1};
        for (const auto name : {"Albedo", "Emission", "EmissionAniso", "NormalBits"})
          Platform::ClearColorImage(command, *resources->images.at(name).image, VkClearColorValue{}, 1, &range);
        resources->OrderAccess(command, VK_PIPELINE_STAGE_2_ALL_GRAPHICS_BIT,
                               VK_ACCESS_2_SHADER_READ_BIT | VK_ACCESS_2_SHADER_WRITE_BIT |
                                   VK_ACCESS_2_VERTEX_ATTRIBUTE_READ_BIT | VK_ACCESS_2_INDEX_READ_BIT);
        for (const auto& region : frame->inputs->regions) {
          if (region.pending.cascade != cascade)
            continue;
          for (uint32_t axis = 0; axis < 3; ++axis) {
            frame->inputs->RecordAxis(command, *frame->raster, region, axis);
            resources->OrderAccess(command,
                                   VK_PIPELINE_STAGE_2_ALL_GRAPHICS_BIT | VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT,
                                   VK_ACCESS_2_SHADER_READ_BIT | VK_ACCESS_2_SHADER_WRITE_BIT);
          }
        }
        frame->region_store->Bind(command);
        frame->region_store->BindDescriptorSet(command, 0, frame->region_set->GetVkDescriptorSet());
        for (const auto& region : frame->inputs->regions) {
          if (region.pending.cascade != cascade)
            continue;
          HddagiRegionParams params;
          params.grid = frame->inputs->cascades[cascade].size;
          params.cascade = cascade;
          params.offset = region.pending.offset;
          params.version = frame->version;
          params.region_world_offset = (frame->inputs->cascades[cascade].position - params.grid / 2) / 8;
          frame->region_store->PushConstant(command, 0, params);
          frame->region_store->Dispatch(command, region.pending.size.x / 8, region.pending.size.y / 8,
                                        region.pending.size.z / 8);
        }
        resources->OrderAccess(command, VK_PIPELINE_STAGE_2_ALL_COMMANDS_BIT,
                               VK_ACCESS_2_MEMORY_READ_BIT | VK_ACCESS_2_MEMORY_WRITE_BIT);
      });
    });
  }
  graph.AddPass({"HddagiVoxelComplete", RenderPassQueue::Graphics, RenderPassScope::Frame, {}, {previous}},
                [resources](const RenderGraphExecutionContext&) {
                  resources->voxelization_recorded = true;
                });
}
