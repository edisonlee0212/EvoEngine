// Godot gi.cpp::SDFGI::{debug_draw,debug_probes}, 34d06658a85845111a50db9e485ec4a0701d4298.
// See docs/licenses/Godot-MIT.txt. Camera lifetime, bounds overlays, and bindings adapt EvoEngine.
#include <algorithm>
#include <cmath>
#include "Application.hpp"
#include "Camera.hpp"
#include "MeshRenderer.hpp"
#include "Platform.hpp"
#include "RenderPasses/RenderPassUtilities.hpp"
#include "RenderTexture.hpp"
#include "Resources.hpp"
#include "Scene.hpp"
#include "SdfgiDebug.hpp"
#include "Shader.hpp"
#include "SkinnedMesh.hpp"
#include "SkinnedMeshRenderer.hpp"

using namespace evo_engine;

namespace {
struct alignas(16) DebugCameraData {
  glm::mat4 view_projection{1};
  glm::mat4 inverse_view_projection{1};
  glm::vec4 origin_y_mult{};
  glm::uvec4 selection{};
  glm::uvec4 field{};
  SdfgiCascadeBlock cascades{};
};
static_assert(sizeof(DebugCameraData) == 560);
struct DebugBox {
  glm::vec4 minimum, maximum, color;
};
static_assert(sizeof(DebugBox) == 48);
std::shared_ptr<Buffer> MakeDebugBuffer(VkDeviceSize bytes, VkBufferUsageFlags usage) {
  VkBufferCreateInfo info{VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO};
  info.size = bytes;
  info.usage = usage | VK_BUFFER_USAGE_TRANSFER_DST_BIT;
  return std::make_shared<Buffer>(info);
}
glm::vec4 CascadeColor(uint32_t cascade) {
  constexpr glm::vec4 colors[]{{1, 0.2f, 0.1f, 1}, {0.2f, 1, 0.2f, 1}, {0.1f, 0.5f, 1, 1}, {1, 0.8f, 0.1f, 1},
                               {1, 0.2f, 1, 1},    {0.1f, 1, 1, 1},    {1, 0.6f, 0.3f, 1}, {0.7f, 0.7f, 1, 1}};
  return colors[cascade % 8];
}
}  // namespace

namespace evo_engine {
class SdfgiCameraDebugFrame {
 public:
  DebugCameraData data;
  std::vector<DebugBox> boxes;
  std::shared_ptr<Buffer> uniform;
  std::shared_ptr<Buffer> box_buffer;
  std::shared_ptr<DescriptorSet> descriptor;
  std::shared_ptr<RenderTexture> target;
  BufferUploadBatch input;
  BufferUploadArena uploads{64 * 1024};
};
}  // namespace evo_engine

SdfgiDebugRenderer::SdfgiDebugRenderer() {
  layout = std::make_shared<DescriptorSetLayout>();
  constexpr auto stages = VK_SHADER_STAGE_COMPUTE_BIT | VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT;
  for (uint32_t binding = 1; binding <= 6; ++binding)
    layout->PushDescriptorBinding(binding, VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE, stages, 0, binding <= 4 ? 8 : 1);
  layout->PushDescriptorBinding(7, VK_DESCRIPTOR_TYPE_SAMPLER, stages, 0, 1);
  layout->PushDescriptorBinding(8, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, stages, 0, 1);
  layout->PushDescriptorBinding(9, VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, VK_SHADER_STAGE_COMPUTE_BIT, 0, 1);
  layout->PushDescriptorBinding(10, VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE, stages, 0, 1);
  layout->PushDescriptorBinding(11, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, stages, 0, 1);
  layout->Initialize();
  const auto limits =
      SdfgiResources::ValidateDescriptorLimits({layout}, Platform::GetSelectedPhysicalDevice()->properties.limits);
  if (!limits.empty())
    throw std::runtime_error(limits);
  const auto root = Resources::GetDefaultResourcesPath() / "Shaders";
  sdf = std::make_shared<ComputePipeline>();
  sdf->descriptor_set_layouts = {layout};
  sdf->compute_shader = Shader::CreateTemporary(ShaderType::Compute, "", root / "Compute/SdfgiDebug.slang");
  sdf->Initialize();
  if (!sdf->Initialized())
    throw std::runtime_error("SDFGI SDF debug pipeline unavailable");
  const char* variants[]{"", "#define MODE_VISIBILITY 1\n", "#define MODE_BOXES 1\n"};
  for (uint32_t i = 0; i < graphics.size(); ++i) {
    auto& pipeline = graphics[i];
    pipeline = std::make_shared<GraphicsPipeline>();
    pipeline->vertex_input_enabled = false;
    pipeline->view_mask = 0;
    pipeline->depth_attachment_format = VK_FORMAT_UNDEFINED;
    pipeline->stencil_attachment_format = VK_FORMAT_UNDEFINED;
    pipeline->color_attachment_formats = {Platform::Constants::render_texture_color};
    pipeline->primitive_topology = i == 2 ? VK_PRIMITIVE_TOPOLOGY_LINE_LIST : VK_PRIMITIVE_TOPOLOGY_TRIANGLE_STRIP;
    pipeline->descriptor_set_layouts = {layout};
    pipeline->vertex_shader =
        Shader::CreateTemporary(ShaderType::Vertex, variants[i], root / "Graphics/Vertex/SDFGI/SdfgiDebugProbes.slang");
    pipeline->fragment_shader = Shader::CreateTemporary(ShaderType::Fragment, variants[i],
                                                        root / "Graphics/Fragment/SDFGI/SdfgiDebugProbes.slang");
    pipeline->Initialize();
    if (!pipeline->Initialized())
      throw std::runtime_error("SDFGI probe/bounds debug pipeline unavailable");
  }
  frames.resize(Platform::GetMaxFramesInFlight());
}

void evo_engine::RetireSdfgiDebugFrame(SdfgiResources& resources, const uint32_t frame_slot) {
  if (const auto& renderer = resources.debug_renderer;
      renderer && renderer->last_retirement != Platform::GetFrameCount()) {
    renderer->frames[frame_slot].clear();
    renderer->last_retirement = Platform::GetFrameCount();
  }
}

uint64_t evo_engine::GetSdfgiDebugAllocationBytes(const SdfgiResources& resources) {
  uint64_t result = 0;
  if (const auto& renderer = resources.debug_renderer)
    for (const auto& slot : renderer->frames)
      for (const auto& frame : slot)
        result += frame->uniform->GetVmaAllocationInfo().size + frame->box_buffer->GetVmaAllocationInfo().size +
                  frame->uploads.GetAllocationBytes();
  return result;
}

void evo_engine::AddSdfgiCameraDebug(RenderGraph& graph, RenderGraphResourceRegistry& registry,
                                     const std::shared_ptr<const SdfgiRuntime>& runtime,
                                     const std::shared_ptr<Camera>& camera, const CameraInfoBlock& camera_data,
                                     const char* dependency) {
  auto& debug = *runtime->debug;
  const auto resources = runtime->resources;
  debug.cascade = std::min(debug.cascade, static_cast<uint32_t>(std::max<size_t>(1, runtime->cascades.size()) - 1));
  const auto probes = runtime->settings.ProbeSize();
  const uint32_t probe_count = probes.x * probes.y * probes.z;
  debug.probe = std::min(debug.probe, probe_count - 1);
  debug.slice = std::min(debug.slice, uint32_t(runtime->settings.GridSize().z - 1));
  const auto remember = [runtime, resources, camera, camera_data, view = debug.view,
                         selection =
                             glm::uvec3(debug.cascade, debug.probe, debug.slice)](const RenderGraphExecutionContext&) {
    auto& state = *runtime->debug;
    state.rendered_camera = camera;
    state.rendered_field = resources;
    state.rendered_generation = resources ? resources->transport_pass : 0;
    state.rendered_view = view;
    state.rendered_selection = selection;
    state.rendered_boundary = state.last_boundary;
    state.rendered_view_projection = camera_data.projection_view;
    state.rendered_resolution = camera->GetSize();
    state.rendered_camera_selection = state.camera_id;
    state.rendered_depth_test = state.depth_test;
  };
  const bool overlay = debug.view == SdfgiDebugView::Cascades || debug.view == SdfgiDebugView::Probes ||
                       debug.view == SdfgiDebugView::Visibility || debug.view == SdfgiDebugView::DirtyRegions ||
                       debug.view == SdfgiDebugView::Contributors;
  const bool compute = debug.view == SdfgiDebugView::Sdf || debug.view == SdfgiDebugView::DistanceSlice;
  if ((!overlay && !compute) || !resources || !resources->initialization_recorded || runtime->cascades.empty()) {
    graph.AddPass({"SdfgiDebugSnapshot", RenderPassQueue::Graphics, RenderPassScope::Camera, {}, {dependency}},
                  remember);
    return;
  }
  if (!resources->debug_renderer)
    resources->debug_renderer = std::make_shared<SdfgiDebugRenderer>();
  const auto renderer = resources->debug_renderer;
  auto frame = std::make_shared<SdfgiCameraDebugFrame>();
  auto& data = frame->data;
  data.view_projection = camera_data.projection_view;
  data.inverse_view_projection = camera_data.inverse_projection_view;
  data.origin_y_mult =
      glm::vec4(glm::vec3(camera_data.inverse_view[3]), SdfgiYMultiplier(runtime->settings.vertical_scale));
  data.selection = {static_cast<uint32_t>(debug.view), std::min(debug.cascade, runtime->settings.cascade_count - 1),
                    std::min(debug.probe, probe_count - 1),
                    std::min(debug.slice, uint32_t(runtime->settings.GridSize().z - 1))};
  data.field = {runtime->settings.cascade_count, 0, debug.depth_test,
                runtime->settings.voxel_count_x | (runtime->settings.voxel_count_y << 16)};
  data.cascades = BuildSdfgiCascadeBlock(runtime->cascades);
  const auto box = [&](const Bound& bounds, const glm::vec4 color) {
    if (glm::all(glm::lessThanEqual(bounds.min, bounds.max)) &&
        std::isfinite(bounds.min.x + bounds.min.y + bounds.min.z + bounds.max.x + bounds.max.y + bounds.max.z))
      frame->boxes.push_back({glm::vec4(bounds.min, 0), glm::vec4(bounds.max, 0), color});
  };
  if (debug.view == SdfgiDebugView::Cascades)
    for (uint32_t c = 0; c < runtime->cascades.size(); ++c) {
      const auto bounds = runtime->cascades[c].WorldBounds(data.origin_y_mult.w);
      box(bounds, CascadeColor(c));
      // The reference fade width remains two probes on every axis.
      for (const float radius : {44.0f, 60.0f}) {
        const auto extent =
            ((glm::vec3(runtime->cascades[c].size) * 0.5f - 64.0f + radius) * runtime->cascades[c].cell_size) /
            glm::vec3(1, data.origin_y_mult.w, 1);
        box({runtime->anchor.world_position - extent, runtime->anchor.world_position + extent},
            CascadeColor(c) * glm::vec4(0.5f, 0.5f, 0.5f, 1));
      }
    }
  if (debug.view == SdfgiDebugView::DirtyRegions)
    for (const auto& region : runtime->pending_regions)
      box(region.world_bounds, CascadeColor(region.cascade));
  if (debug.view == SdfgiDebugView::Contributors) {
    const auto scene = ApplicationContext::Get().GetActiveScene();
    std::map<uint64_t, std::shared_ptr<MeshRenderer>> meshes;
    if (const auto owners = scene->UnsafeGetPrivateComponentOwnersList<MeshRenderer>())
      for (const auto owner : *owners) {
        const auto renderer = scene->GetOrSetPrivateComponent<MeshRenderer>(owner).lock();
        meshes.emplace(renderer->GetHandle().GetValue(), renderer);
      }
    for (const auto& contributor : runtime->scene_snapshot.contributors) {
      if (contributor.exclusion == SdfgiExclusion::None) {
        box(contributor.world_bounds, {0, 1, 0, 1});
        continue;
      }
      const auto renderer = meshes.find(contributor.id.first);
      const auto entity = scene->GetEntity(Handle(contributor.id.second));
      if (renderer == meshes.end() || !scene->IsEntityValid(entity))
        continue;
      if (const auto mesh = renderer->second->mesh.Get<Mesh>()) {
        auto bounds = mesh->GetBound();
        bounds.ApplyTransform(scene->GetDataComponent<GlobalTransform>(entity).value);
        box(bounds,
            contributor.exclusion == SdfgiExclusion::Dynamic || contributor.exclusion == SdfgiExclusion::Deforming
                ? glm::vec4(0, 0.7f, 1, 1)
                : glm::vec4(1, 0.2f, 0.1f, 1));
      }
    }
    if (const auto owners = scene->UnsafeGetPrivateComponentOwnersList<SkinnedMeshRenderer>())
      for (const auto owner : *owners) {
        const auto renderer = scene->GetOrSetPrivateComponent<SkinnedMeshRenderer>(owner).lock();
        if (!renderer->IsEnabled() || !scene->IsEntityEnabled(owner))
          continue;
        if (const auto mesh = renderer->skinned_mesh.Get<SkinnedMesh>()) {
          auto bounds = mesh->GetBound();  // Bind-pose diagnostic bounds, not a new deformation evaluation.
          bounds.ApplyTransform(scene->GetDataComponent<GlobalTransform>(owner).value);
          box(bounds, {0, 0.7f, 1, 1});
        }
      }
  }
  data.field.y = static_cast<uint32_t>(frame->boxes.size());
  frame->uniform = MakeDebugBuffer(sizeof(data), VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT);
  frame->box_buffer =
      MakeDebugBuffer(std::max<size_t>(1, frame->boxes.size()) * sizeof(DebugBox), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT);
  frame->input.Add(frame->uniform, data, {BufferUploadUsage::Uniform});
  frame->input.AddVector(frame->box_buffer, frame->boxes);
  frame->target = camera->GetRenderTexture();
  frame->descriptor = std::make_shared<DescriptorSet>(renderer->layout);
  const auto image = [&](uint32_t binding, const std::shared_ptr<ImageView>& view, uint32_t index = 0) {
    frame->descriptor->UpdateImageDescriptorBinding(
        binding, {VK_NULL_HANDLE, view->GetVkImageView(), VK_IMAGE_LAYOUT_GENERAL}, index);
  };
  for (uint32_t i = 0; i < 8; ++i) {
    const auto prefix = "Cascade" + std::to_string(std::min(i, runtime->settings.cascade_count - 1)) + ".";
    uint32_t binding = 1;
    for (const auto name : {"Sdf", "Light", "Aniso0", "Aniso1"})
      image(binding++, resources->textures.at(prefix + name).sampled_view, i);
  }
  image(5, resources->textures.at("Occlusion").sampled_view);
  image(6, resources->textures.at("Atlas").sampled_view);
  frame->descriptor->UpdateImageDescriptorBinding(
      7, {resources->linear_sampler->GetVkSampler(), VK_NULL_HANDLE, VK_IMAGE_LAYOUT_UNDEFINED});
  frame->descriptor->UpdateBufferDescriptorBinding(8, frame->uniform);
  image(9, frame->target->GetColorImageView());
  frame->descriptor->UpdateImageDescriptorBinding(
      10, {VK_NULL_HANDLE, frame->target->GetDepthImageView()->GetVkImageView(), VK_IMAGE_LAYOUT_GENERAL});
  frame->descriptor->UpdateBufferDescriptorBinding(11, frame->box_buffer);
  resources->Import(graph, registry);
  RenderPassDescriptor pass{"SdfgiDebugView", RenderPassQueue::Graphics, RenderPassScope::Camera};
  pass.dependencies = {dependency};
  pass.profiler_group = RenderPassProfilerGroup::EditorAndUi;
  pass.profiler_display_name = "SDFGI Debug View";
  for (const auto& [name, texture] : resources->textures)
    if (name == "Atlas" || name == "Occlusion" || name.find(".Sdf") != std::string::npos ||
        name.find(".Light") != std::string::npos || name.find(".Aniso") != std::string::npos)
      pass.resources.push_back({"Frame.SDFGI." + name, RenderResourceUsage::Read, RenderResourceState::General});
  pass.resources.push_back({RenderResourceNames::camera_color, RenderResourceUsage::ReadWrite,
                            compute ? RenderResourceState::StorageReadWrite : RenderResourceState::ColorAttachment});
  pass.resources.push_back(
      {RenderResourceNames::camera_depth, RenderResourceUsage::Read, RenderResourceState::ShaderRead});
  for (const auto& [name, buffer] : std::initializer_list<std::pair<const char*, std::shared_ptr<Buffer>>>{
           {"Camera.SDFGI.DebugUniform", frame->uniform}, {"Camera.SDFGI.DebugBoxes", frame->box_buffer}}) {
    RenderResourceDescriptor resource;
    resource.name = name;
    resource.type = RenderResourceType::Buffer;
    resource.lifetime = RenderResourceLifetime::Persistent;
    resource.byte_size = buffer->GetSize();
    graph.AddResource(resource);
    registry.BindBuffer(name, buffer);
    pass.resources.push_back({name, RenderResourceUsage::ReadWrite, RenderResourceState::General});
  }
  graph.AddPass(pass, [renderer, resources, frame, compute, remember](const RenderGraphExecutionContext& context) {
    frame->input.Record(frame->uploads);
    Platform::RecordCommandsMainQueue([&](const VkCommandBuffer command) {
      resources->OrderAccess(command, VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT | VK_PIPELINE_STAGE_2_ALL_GRAPHICS_BIT,
                             VK_ACCESS_2_SHADER_READ_BIT | VK_ACCESS_2_UNIFORM_READ_BIT);
      ApplyGraphResourceBarriers(command, context);
      const RenderPassGpuTimestampScope timing(command, context);
      const auto size = frame->target->GetExtent();
      if (compute) {
        renderer->sdf->Bind(command);
        renderer->sdf->BindDescriptorSet(command, 0, frame->descriptor->GetVkDescriptorSet());
        renderer->sdf->Dispatch(command, Platform::DivUp(size.width, 8), Platform::DivUp(size.height, 8));
      } else {
        std::vector<VkRenderingAttachmentInfo> attachments;
        frame->target->AppendColorAttachmentInfos(attachments, VK_ATTACHMENT_LOAD_OP_LOAD,
                                                  VK_ATTACHMENT_STORE_OP_STORE);
        VkRenderingInfo rendering{VK_STRUCTURE_TYPE_RENDERING_INFO};
        rendering.renderArea.extent = {size.width, size.height};
        rendering.layerCount = 1;
        rendering.colorAttachmentCount = static_cast<uint32_t>(attachments.size());
        rendering.pColorAttachments = attachments.data();
        Platform::RecordRenderCommands(rendering, command, [&]() {
          const auto draw = [&](uint32_t kind, uint32_t vertices, uint32_t instances) {
            auto& pipeline = renderer->graphics[kind];
            pipeline->states.ResetAllStates(attachments.size());
            pipeline->states.depth_test = pipeline->states.depth_write = false;
            pipeline->states.cull_mode = VK_CULL_MODE_NONE;
            pipeline->states.SetViewportScissor({0, 0, static_cast<int>(size.width), static_cast<int>(size.height)});
            pipeline->Bind(command);
            pipeline->BindDescriptorSet(command, 0, frame->descriptor->GetVkDescriptorSet());
            pipeline->states.ApplyAllStates(command);
            Platform::Draw(command, vertices, instances, 0, 0);
          };
          const uint32_t spacing = frame->data.cascades.data[frame->data.selection.y].pad;
          const uint32_t horizontal = (frame->data.field.w & 65535) / spacing + 1;
          if (frame->data.selection.x == static_cast<uint32_t>(SdfgiDebugView::Probes) ||
              frame->data.selection.x == static_cast<uint32_t>(SdfgiDebugView::Visibility))
            draw(0, 112, horizontal * horizontal * ((frame->data.field.w >> 16) / spacing + 1));
          if (frame->data.selection.x == static_cast<uint32_t>(SdfgiDebugView::Visibility))
            draw(1, 112, 8 * spacing * spacing * spacing);
          if (!frame->boxes.empty())
            draw(2, 24, static_cast<uint32_t>(frame->boxes.size()));
        });
      }
      ApplyGraphResourceReleaseBarriers(command, context, RenderPassQueue::Graphics);
    });
    remember(context);
  });
  renderer->frames[Platform::GetCurrentFrameIndex()].push_back(frame);
}
