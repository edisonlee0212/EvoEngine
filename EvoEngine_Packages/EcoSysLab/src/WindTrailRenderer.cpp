#include "WindTrailRenderer.hpp"

#include "Application.hpp"
#include "RenderLayer.hpp"
#include "Shader.hpp"

using namespace eco_sys_lab_package;
using namespace evo_engine;

namespace {
struct RenderParameters {
  uint32_t camera;
  float time;
  float duration;
  float width;
  glm::vec2 viewport;
  float opacity;
  float padding = 0;
};
}  // namespace

bool WindTrailRenderer::Prepare(const std::shared_ptr<Buffer>& particles, uint32_t particle_capacity) {
  if (update_)
    return update_->Initialized() && render_->Initialized();
  const auto layer = ApplicationContext::Get().GetLayer<RenderLayer>();
  if (!layer)
    return false;
  particle_capacity_ = particle_capacity;
  layout_ = std::make_shared<DescriptorSetLayout>();
  for (uint32_t i = 0; i < 3; ++i)
    layout_->PushDescriptorBinding(i, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,
                                   VK_SHADER_STAGE_COMPUTE_BIT | VK_SHADER_STAGE_VERTEX_BIT, 0);
  layout_->Initialize();
  const auto make = [](const auto& data) {
    VkBufferCreateInfo info{VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO};
    info.size = data.size() * sizeof(data[0]);
    info.usage = VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT;
    VmaAllocationCreateInfo allocation{};
    allocation.usage = VMA_MEMORY_USAGE_AUTO_PREFER_DEVICE;
    auto buffer = std::make_shared<Buffer>(info, allocation);
    buffer->UploadVector(data);
    return buffer;
  };
  trails_ = make(std::vector<Trail>(capacity));
  samples_ = make(std::vector<glm::vec4>(capacity * samples_per_trail));
  set_ = std::make_shared<DescriptorSet>(layout_);
  set_->UpdateBufferDescriptorBinding(0, particles);
  set_->UpdateBufferDescriptorBinding(1, trails_);
  set_->UpdateBufferDescriptorBinding(2, samples_);
  update_ = std::make_shared<ComputePipeline>();
  update_->compute_shader = Shader::CreateTemporary(ShaderType::Compute, Platform::GetShaderGlobalDefines(),
                                                    "EcoSysLabResources/Shaders/Compute/Wind/Trails.slang");
  update_->descriptor_set_layouts = {layout_};
  update_->push_constant_ranges.push_back({VK_SHADER_STAGE_COMPUTE_BIT, 0, sizeof(UpdateParameters)});
  update_->Initialize();
  render_ = std::make_shared<GraphicsPipeline>();
  render_->vertex_shader = Shader::CreateTemporary(ShaderType::Vertex, Platform::GetShaderGlobalDefines(),
                                                   "EcoSysLabResources/Shaders/Graphics/WindTrails.vert.slang");
  render_->fragment_shader = Shader::CreateTemporary(ShaderType::Fragment, Platform::GetShaderGlobalDefines(),
                                                     "EcoSysLabResources/Shaders/Graphics/WindTrails.frag.slang");
  render_->geometry_type = GeometryType::Mesh;
  render_->vertex_input_enabled = false;
  render_->primitive_topology = VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST;
  render_->descriptor_set_layouts = {layer->GetPerFrameDescriptorSetLayout(), layout_};
  render_->depth_attachment_format = Platform::Constants::render_texture_depth;
  render_->stencil_attachment_format = VK_FORMAT_UNDEFINED;
  render_->color_attachment_formats = {Platform::Constants::render_texture_color};
  render_->push_constant_ranges.push_back({VK_SHADER_STAGE_VERTEX_BIT, 0, sizeof(RenderParameters)});
  render_->Initialize();
  return update_->Initialized() && render_->Initialized();
}

void WindTrailRenderer::Dispatch(uint32_t begin, uint32_t count, uint32_t emitted, bool spawn) {
  const float duration = glm::clamp(settings.duration, 0.02f, 5.0f);
  const UpdateParameters parameters{
      {emitted, begin, count, static_cast<uint32_t>(glm::clamp(settings.emission_stride, 1, 65536))},
      {time_, duration, glm::max(settings.max_length, 0.01f), glm::clamp(settings.sample_interval, 0.0f, duration)},
      {spawn ? 1u : 0u, static_cast<uint32_t>(glm::clamp(settings.budget, 1, static_cast<int>(capacity))),
       particle_capacity_, 0}};
  Platform::RecordCommandsMainQueue([&](VkCommandBuffer command) {
    update_->Bind(command);
    update_->BindDescriptorSet(command, 0, set_->GetVkDescriptorSet());
    update_->PushConstant(command, 0, parameters);
    update_->Dispatch(command, (capacity + 63) / 64, 1, 1);
    Platform::EverythingBarrier(command);
  });
}

void WindTrailRenderer::Advance(float dt, uint32_t emitted) {
  if (dt <= 0)
    return;
  time_ += dt;
  Dispatch(0, 0, emitted, false);
}

void WindTrailRenderer::Emit(uint32_t begin, uint32_t count) {
  if (count)
    Dispatch(begin, count, begin + count, true);
}

uint32_t WindTrailRenderer::Draw(VkCommandBuffer command, const glm::ivec4& viewport, uint32_t camera_index) {
  if (!render_ || !render_->Initialized())
    return 0;
  render_->states.ResetAllStates(1);
  render_->states.SetViewportScissor(viewport);
  render_->states.cull_mode = VK_CULL_MODE_NONE;
  render_->states.depth_test = true;
  render_->states.depth_write = false;
  render_->states.depth_compare = VK_COMPARE_OP_LESS_OR_EQUAL;
  auto& blend = render_->states.color_blend_attachment_states[0];
  blend.blendEnable = VK_TRUE;
  blend.srcColorBlendFactor = VK_BLEND_FACTOR_SRC_ALPHA;
  blend.dstColorBlendFactor = VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA;
  blend.srcAlphaBlendFactor = VK_BLEND_FACTOR_ONE;
  blend.dstAlphaBlendFactor = VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA;
  render_->states.ApplyAllStates(command);
  render_->Bind(command);
  render_->BindDescriptorSet(command, 0, RenderLayer::GetPerFrameDescriptorSet()->GetVkDescriptorSet());
  render_->BindDescriptorSet(command, 1, set_->GetVkDescriptorSet());
  render_->PushConstant(command, 0,
                        RenderParameters{camera_index,
                                         time_,
                                         glm::clamp(settings.duration, 0.02f, 5.0f),
                                         glm::clamp(settings.width, 0.5f, 64.0f),
                                         {viewport.z, viewport.w},
                                         glm::clamp(settings.opacity, 0.0f, 1.0f)});
  Platform::Draw(command, 6 * (samples_per_trail - 1), capacity);
  return 2 * (samples_per_trail - 1) * capacity;
}
