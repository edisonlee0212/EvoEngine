// Metadata from Godot gi.cpp::SDFGI::pre_process_gi, 34d06658a85845111a50db9e485ec4a0701d4298.
// See docs/licenses/Godot-MIT.txt. Publication/lifetime and shared-anchor origin are host adapters.
#include "SdfgiGather.hpp"
#include "Camera.hpp"
#include "Platform.hpp"
#include "RenderPasses/RenderPassUtilities.hpp"
#include "Scene.hpp"

using namespace evo_engine;

bool evo_engine::IsSdfgiCameraEligible(const std::shared_ptr<Scene>& scene, const std::shared_ptr<Camera>& camera,
                                       const std::shared_ptr<Camera>& editor_camera, const bool immediate,
                                       const bool reflection_capture, const bool custom_recorder) {
  return scene && camera && !immediate && !reflection_capture && !custom_recorder && camera->IsEnabled() &&
         camera->camera_render_mode == Camera::CameraRenderMode::Rasterization &&
         (camera == editor_camera || (camera->GetScene() == scene && scene->IsEntityValid(camera->GetOwner()) &&
                                      scene->IsEntityEnabled(camera->GetOwner())));
}

SdfgiGatherData evo_engine::BuildSdfgiGatherData(const SdfgiSettings& settings,
                                                 const std::vector<SdfgiCascade>& cascades, glm::vec3 anchor_world,
                                                 const uint32_t generation) {
  SdfgiGatherData result{};
  result.max_cascades = cascades.size();
  result.use_occlusion = settings.use_occlusion;
  result.probe_axis_size = 17;
  result.probe_to_uvw = 1.0f / 16;
  result.normal_bias = settings.normal_bias / 8;
  result.energy = settings.energy;
  result.y_mult = SdfgiYMultiplier(settings.vertical_scale);
  result.generation = generation;
  anchor_world.y *= result.y_mult;
  const glm::vec3 texel(1.0f / 2312, 1.0f / 136, 1);
  const glm::vec3 uv_offset(8 * texel.x, 8 * texel.y, 136 * texel.x);
  const glm::vec3 renormalize(0.5f, 1, 1.0f / result.max_cascades);
  for (uint32_t axis = 0; axis < 3; ++axis) {
    result.grid_size[axis] = 128;
    result.cascade_probe_size[axis] = 16;
    result.anchor_origin[axis] = anchor_world[axis];
    result.lightprobe_tex_pixel_size[axis] = texel[axis];
    result.lightprobe_uv_offset[axis] = uv_offset[axis];
    result.occlusion_clamp[axis] = 7.5f / 8;
    result.occlusion_renormalize[axis] = renormalize[axis];
  }
  for (uint32_t c = 0; c < cascades.size(); ++c) {
    auto& out = result.cascades[c];
    const auto& input = cascades[c];
    const glm::vec3 position = glm::vec3(input.position - glm::ivec3(64)) * input.cell_size - anchor_world;
    for (uint32_t axis = 0; axis < 3; ++axis) {
      out.position[axis] = position[axis];
      out.probe_world_offset[axis] = input.position[axis] / 8;
    }
    out.to_probe = 1 / (8 * input.cell_size);
    out.to_cell = 1 / input.cell_size;
    out.exposure_normalization = 1;
  }
  return result;
}

std::shared_ptr<SdfgiGatherFrame> SdfgiGatherFrame::Create(const SdfgiRuntime& runtime, const uint32_t generation) {
  auto frame = std::make_shared<SdfgiGatherFrame>();
  frame->frame_slot = Platform::GetCurrentFrameIndex();
  frame->metadata = BuildSdfgiGatherData(runtime.settings, runtime.cascades, runtime.anchor.world_position, generation);
  const auto name = "Frame" + std::to_string(frame->frame_slot) + ".Gather";
  frame->descriptor_set = runtime.resources->sets.at(name);
  frame->input_upload.Add(runtime.resources->buffers.at(name).buffer, frame->metadata, {BufferUploadUsage::Uniform});
  return frame;
}

void SdfgiGatherFrame::AddPublication(RenderGraph& graph, const std::shared_ptr<SdfgiRuntime>& runtime) {
  RenderPassDescriptor pass{"SdfgiPublish", RenderPassQueue::Graphics, RenderPassScope::Frame};
  pass.dependencies = {"SdfgiProbeStore"};
  pass.resources = CameraReads();
  for (auto& access : pass.resources)
    if (access.resource_name == "Frame.SDFGI.Status" || access.resource_name.find(".Gather") != std::string::npos)
      access.usage = RenderResourceUsage::ReadWrite;
  graph.AddPass(pass, [frame = shared_from_this(), runtime](const RenderGraphExecutionContext& context) {
    const auto& resources = runtime->resources;
    if (!resources->transport_recorded || resources->transport_pass != frame->metadata.generation ||
        !resources->light_failure.empty() || !resources->transport_failure.empty() ||
        resources->preprocess_status.failure_flags)
      return;
    Platform::RecordCommandsMainQueue([&](const VkCommandBuffer command) {
      resources->OrderAccess(command, VK_PIPELINE_STAGE_2_TRANSFER_BIT, VK_ACCESS_2_TRANSFER_WRITE_BIT);
    });
    frame->input_upload.Record(frame->uploads);
    Platform::RecordCommandsMainQueue([&](const VkCommandBuffer command) {
      resources->OrderAccess(command, VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT,
                             VK_ACCESS_2_SHADER_READ_BIT | VK_ACCESS_2_SHADER_WRITE_BIT);
      ApplyGraphResourceBarriers(command, context);
      const auto& pipeline = resources->pipelines.at("Publish");
      pipeline->Bind(command);
      pipeline->BindDescriptorSet(command, 0, frame->descriptor_set->GetVkDescriptorSet());
      pipeline->Dispatch(command, 1);
      resources->OrderAccess(command, VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT, VK_ACCESS_2_SHADER_READ_BIT);
    });
    resources->publication = frame;
    runtime->published = true;
    runtime->fallback_reason.clear();
  });
}

std::vector<RenderResourceAccess> SdfgiGatherFrame::CameraReads() const {
  std::vector<RenderResourceAccess> reads{
      {"Frame.SDFGI.Atlas", RenderResourceUsage::Read, RenderResourceState::General},
      {"Frame.SDFGI.Occlusion", RenderResourceUsage::Read, RenderResourceState::General},
      {"Frame.SDFGI.Status", RenderResourceUsage::Read, RenderResourceState::General},
      {"Frame.SDFGI.Frame" + std::to_string(frame_slot) + ".Gather", RenderResourceUsage::Read,
       RenderResourceState::General}};
  for (uint32_t c = 0; c < metadata.max_cascades; ++c)
    for (const std::string name : {"Sdf", "Light"})
      reads.push_back({"Frame.SDFGI.Cascade" + std::to_string(c) + "." + name, RenderResourceUsage::Read,
                       RenderResourceState::General});
  return reads;
}

void SdfgiGatherFrame::ImportCamera(RenderGraph& graph, RenderGraphResourceRegistry& registry,
                                    const SdfgiResources& resources) const {
  for (const auto& access : CameraReads()) {
    const auto name = access.resource_name.substr(std::string("Frame.SDFGI.").size());
    RenderResourceDescriptor descriptor;
    descriptor.name = access.resource_name;
    descriptor.lifetime = RenderResourceLifetime::Persistent;
    if (const auto texture = resources.textures.find(name); texture != resources.textures.end()) {
      const auto& r = texture->second.requirement;
      descriptor.type = RenderResourceType::Image;
      descriptor.dimensions = {
          RenderResourceSizeMode::Absolute, r.extent.width, r.extent.height, r.extent.depth, r.layers, 1};
      descriptor.format_name = std::to_string(r.storage_format);
      descriptor.byte_size = texture->second.image->GetVmaAllocationInfo().size;
      registry.BindImage(descriptor.name, texture->second.image);
    } else {
      descriptor.type = RenderResourceType::Buffer;
      descriptor.byte_size = resources.buffers.at(name).buffer->GetSize();
      registry.BindBuffer(descriptor.name, resources.buffers.at(name).buffer);
    }
    graph.AddResource(descriptor);
  }
}
