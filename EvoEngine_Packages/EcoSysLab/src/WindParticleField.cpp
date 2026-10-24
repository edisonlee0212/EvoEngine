#include "WindParticleField.hpp"

#include <algorithm>
#include <cmath>

#include "Application.hpp"
#include "Blower.hpp"
#include "RenderLayer.hpp"
#include "RenderTexture.hpp"
#include "Shader.hpp"

using namespace eco_sys_lab_package;

bool WindParticleField::Prepare() {
  if (render_pipeline_)
    return advect_pipeline_->Initialized() && emit_pipeline_->Initialized() && clear_grid_pipeline_->Initialized() &&
           grid_pipeline_->Initialized() && render_pipeline_->Initialized();
  const auto render_layer = ApplicationContext::Get().GetLayer<RenderLayer>();
  if (!render_layer)
    return false;
  layout_ = std::make_shared<DescriptorSetLayout>();
  layout_->PushDescriptorBinding(0, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,
                                 VK_SHADER_STAGE_COMPUTE_BIT | VK_SHADER_STAGE_VERTEX_BIT, 0);
  layout_->PushDescriptorBinding(1, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_COMPUTE_BIT, 0);
  layout_->PushDescriptorBinding(2, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_COMPUTE_BIT, 0);
  layout_->Initialize();

  VkBufferCreateInfo buffer_info{VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO};
  buffer_info.size = capacity * sizeof(Particle);
  buffer_info.usage = VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT;
  buffer_info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
  VmaAllocationCreateInfo allocation_info{};
  allocation_info.usage = VMA_MEMORY_USAGE_AUTO_PREFER_DEVICE;
  particles_ = std::make_shared<Buffer>(buffer_info, allocation_info);
  particles_->UploadVector(std::vector<Particle>(capacity));
  buffer_info.size = grid_bucket_count * sizeof(uint32_t);
  grid_heads_ = std::make_shared<Buffer>(buffer_info, allocation_info);
  buffer_info.size = capacity * sizeof(uint32_t);
  grid_next_ = std::make_shared<Buffer>(buffer_info, allocation_info);
  descriptor_set_ = std::make_shared<DescriptorSet>(layout_);
  descriptor_set_->UpdateBufferDescriptorBinding(0, particles_);
  descriptor_set_->UpdateBufferDescriptorBinding(1, grid_heads_);
  descriptor_set_->UpdateBufferDescriptorBinding(2, grid_next_);

  const auto compute_shader = [](const char* path) {
    return Shader::CreateTemporary(ShaderType::Compute, Platform::GetShaderGlobalDefines(),
                                   std::filesystem::path("./EcoSysLabResources/Shaders/Compute/Wind") / path);
  };
  advect_pipeline_ = std::make_shared<ComputePipeline>();
  advect_pipeline_->compute_shader = compute_shader("Advect.slang");
  advect_pipeline_->descriptor_set_layouts = {layout_};
  advect_pipeline_->push_constant_ranges.push_back({VK_SHADER_STAGE_COMPUTE_BIT, 0, sizeof(AdvectParameters)});
  advect_pipeline_->Initialize();
  emit_pipeline_ = std::make_shared<ComputePipeline>();
  emit_pipeline_->compute_shader = compute_shader("Emit.slang");
  emit_pipeline_->descriptor_set_layouts = {layout_};
  emit_pipeline_->push_constant_ranges.push_back({VK_SHADER_STAGE_COMPUTE_BIT, 0, sizeof(EmitParameters)});
  emit_pipeline_->Initialize();
  clear_grid_pipeline_ = std::make_shared<ComputePipeline>();
  clear_grid_pipeline_->compute_shader = compute_shader("ClearGrid.slang");
  clear_grid_pipeline_->descriptor_set_layouts = {layout_};
  clear_grid_pipeline_->push_constant_ranges.push_back({VK_SHADER_STAGE_COMPUTE_BIT, 0, sizeof(GridParameters)});
  clear_grid_pipeline_->Initialize();
  grid_pipeline_ = std::make_shared<ComputePipeline>();
  grid_pipeline_->compute_shader = compute_shader("BuildGrid.slang");
  grid_pipeline_->descriptor_set_layouts = {layout_};
  grid_pipeline_->push_constant_ranges.push_back({VK_SHADER_STAGE_COMPUTE_BIT, 0, sizeof(GridParameters)});
  grid_pipeline_->Initialize();

  const auto graphics_shader = [](ShaderType type, const char* path) {
    return Shader::CreateTemporary(type, Platform::GetShaderGlobalDefines(),
                                   std::filesystem::path("./EcoSysLabResources/Shaders/Graphics") / path);
  };
  render_pipeline_ = std::make_shared<GraphicsPipeline>();
  render_pipeline_->vertex_shader = graphics_shader(ShaderType::Vertex, "WindParticles.vert.slang");
  render_pipeline_->fragment_shader = graphics_shader(ShaderType::Fragment, "WindParticles.frag.slang");
  render_pipeline_->geometry_type = GeometryType::Mesh;
  render_pipeline_->vertex_input_enabled = false;
  render_pipeline_->primitive_topology = VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST;
  render_pipeline_->descriptor_set_layouts = {render_layer->GetPerFrameDescriptorSetLayout(), layout_};
  render_pipeline_->depth_attachment_format = Platform::Constants::render_texture_depth;
  render_pipeline_->stencil_attachment_format = VK_FORMAT_UNDEFINED;
  render_pipeline_->color_attachment_formats = {Platform::Constants::render_texture_color};
  render_pipeline_->push_constant_ranges.push_back({VK_SHADER_STAGE_VERTEX_BIT, 0, sizeof(RenderParameters)});
  render_pipeline_->Initialize();
  return advect_pipeline_->Initialized() && emit_pipeline_->Initialized() && clear_grid_pipeline_->Initialized() &&
         grid_pipeline_->Initialized() && render_pipeline_->Initialized();
}

void WindParticleField::Step(const std::shared_ptr<Scene>& scene, const float time_step) {
  if (scene_.lock() != scene) {
    particles_.reset();
    grid_heads_.reset();
    grid_next_.reset();
    descriptor_set_.reset();
    layout_.reset();
    advect_pipeline_.reset();
    emit_pipeline_.reset();
    clear_grid_pipeline_.reset();
    grid_pipeline_.reset();
    render_pipeline_.reset();
    emission_remainders_.clear();
    emitted_ = 0;
    max_particle_radius_ = 0.0f;
    scene_ = scene;
  }
  const auto* owners = scene->UnsafeGetPrivateComponentOwnersList<Blower>();
  if ((!owners || owners->empty()) && !emitted_)
    return;
  if (!Prepare())
    return;
  const float dt = glm::clamp(time_step, 0.0f, 0.1f);
  if (dt <= 0.0f)
    return;
  if (emitted_) {
    const AdvectParameters parameters{dt, static_cast<uint32_t>(std::min<uint64_t>(emitted_, capacity))};
    Platform::RecordCommandsMainQueue([&](VkCommandBuffer command_buffer) {
      advect_pipeline_->Bind(command_buffer);
      advect_pipeline_->BindDescriptorSet(command_buffer, 0, descriptor_set_->GetVkDescriptorSet());
      advect_pipeline_->PushConstant(command_buffer, 0, parameters);
      advect_pipeline_->Dispatch(command_buffer, (parameters.count + 63) / 64, 1, 1);
      Platform::EverythingBarrier(command_buffer);
    });
  }
  if (owners)
    for (const auto entity : *owners) {
      if (!scene->IsEntityEnabled(entity))
        continue;
      const auto blower = scene->GetOrSetPrivateComponent<Blower>(entity).lock();
      if (!blower || !blower->IsEnabled())
        continue;
      const uint64_t key = (static_cast<uint64_t>(entity.GetVersion()) << 32) | entity.GetIndex();
      float& remainder = emission_remainders_[key];
      const float requested = remainder + glm::max(0.0f, blower->particle_rate) * dt;
      const uint32_t count = static_cast<uint32_t>(glm::min(std::floor(requested), 2048.0f));
      remainder = requested - std::floor(requested);
      if (!count)
        continue;
      EmitParameters parameters;
      parameters.model = scene->GetDataComponent<GlobalTransform>(entity).value;
      parameters.speed_spread_lifetime = {glm::max(0.0f, blower->speed),
                                          glm::radians(glm::clamp(blower->spread_degrees, 0.0f, 90.0f)), 3.0f, dt};
      parameters.collision.x = glm::clamp(blower->particle_radius, 0.0f, 1.0f);
      parameters.collision.y = glm::clamp(blower->air_density, 0.0f, 20.0f);
      max_particle_radius_ = glm::max(max_particle_radius_, parameters.collision.x);
      parameters.base_count_seed_capacity = {static_cast<uint32_t>(emitted_), count, blower->seed, capacity};
      Platform::RecordCommandsMainQueue([&](VkCommandBuffer command_buffer) {
        emit_pipeline_->Bind(command_buffer);
        emit_pipeline_->BindDescriptorSet(command_buffer, 0, descriptor_set_->GetVkDescriptorSet());
        emit_pipeline_->PushConstant(command_buffer, 0, parameters);
        emit_pipeline_->Dispatch(command_buffer, (count + 63) / 64, 1, 1);
        Platform::EverythingBarrier(command_buffer);
      });
      emitted_ += count;
    }
  if (emitted_)
    Platform::RecordCommandsMainQueue([&](VkCommandBuffer command_buffer) {
      const GridParameters parameters{GetParticleCount(), grid_bucket_count, grid_cell_size};
      clear_grid_pipeline_->Bind(command_buffer);
      clear_grid_pipeline_->BindDescriptorSet(command_buffer, 0, descriptor_set_->GetVkDescriptorSet());
      clear_grid_pipeline_->PushConstant(command_buffer, 0, parameters);
      clear_grid_pipeline_->Dispatch(command_buffer, (grid_bucket_count + 63) / 64, 1, 1);
      Platform::EverythingBarrier(command_buffer);
      grid_pipeline_->Bind(command_buffer);
      grid_pipeline_->BindDescriptorSet(command_buffer, 0, descriptor_set_->GetVkDescriptorSet());
      grid_pipeline_->PushConstant(command_buffer, 0, parameters);
      grid_pipeline_->Dispatch(command_buffer, (GetParticleCount() + 63) / 64, 1, 1);
      Platform::EverythingBarrier(command_buffer);
    });
}

void WindParticleField::RegisterRender(const bool visible) {
  if (!visible || !emitted_ || !render_pipeline_ || !render_pipeline_->Initialized())
    return;
  const auto render_layer = ApplicationContext::Get().GetLayer<RenderLayer>();
  if (!render_layer)
    return;
  const auto self = shared_from_this();
  render_layer->ForwardRenderingAllCameras([self](VkCommandBuffer command_buffer, const std::shared_ptr<Camera>& camera,
                                                  const RenderLayer::ForwardRenderingView& view) {
    return self->Draw(command_buffer, camera, view.viewport, view.camera_index);
  });
}

uint32_t WindParticleField::Draw(VkCommandBuffer command_buffer, const std::shared_ptr<Camera>& camera,
                                 const glm::ivec4& viewport, const uint32_t camera_index) {
  if (!camera || !camera->GetRenderTexture() || !RenderLayer::GetPerFrameDescriptorSet())
    return 0;
  std::vector<VkRenderingAttachmentInfo> colors;
  camera->GetRenderTexture()->AppendColorAttachmentInfos(colors, VK_ATTACHMENT_LOAD_OP_LOAD,
                                                         VK_ATTACHMENT_STORE_OP_STORE);
  const auto depth =
      camera->GetRenderTexture()->GetDepthAttachmentInfo(VK_ATTACHMENT_LOAD_OP_LOAD, VK_ATTACHMENT_STORE_OP_STORE);
  VkRenderingInfo rendering{VK_STRUCTURE_TYPE_RENDERING_INFO};
  rendering.renderArea = {{viewport.x, viewport.y},
                          {static_cast<uint32_t>(viewport.z), static_cast<uint32_t>(viewport.w)}};
  rendering.layerCount = 1;
  rendering.colorAttachmentCount = static_cast<uint32_t>(colors.size());
  rendering.pColorAttachments = colors.data();
  rendering.pDepthAttachment = &depth;
  Platform::BeginRendering(command_buffer, rendering);
  render_pipeline_->states.ResetAllStates(1);
  render_pipeline_->states.SetViewportScissor(viewport);
  render_pipeline_->states.cull_mode = VK_CULL_MODE_NONE;
  render_pipeline_->states.depth_test = true;
  render_pipeline_->states.depth_write = false;
  render_pipeline_->states.depth_compare = VK_COMPARE_OP_LESS_OR_EQUAL;
  render_pipeline_->states.ApplyAllStates(command_buffer);
  render_pipeline_->Bind(command_buffer);
  render_pipeline_->BindDescriptorSet(command_buffer, 0, RenderLayer::GetPerFrameDescriptorSet()->GetVkDescriptorSet());
  render_pipeline_->BindDescriptorSet(command_buffer, 1, descriptor_set_->GetVkDescriptorSet());
  render_pipeline_->PushConstant(command_buffer, 0, RenderParameters{camera_index, {viewport.z, viewport.w}, 3.0f});
  const uint32_t count = static_cast<uint32_t>(std::min<uint64_t>(emitted_, capacity));
  Platform::Draw(command_buffer, 6, count);
  Platform::EndRendering(command_buffer);
  return count * 2;
}
