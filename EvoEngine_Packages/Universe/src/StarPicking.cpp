#include "StarPicking.hpp"
#include "Camera.hpp"
#include "GpuProfiler.hpp"
#include "RenderTexture.hpp"
#include "Shader.hpp"
#include "UniverseProfiler.hpp"

using namespace universe_package;

namespace {
std::shared_ptr<Buffer> PickBuffer(const size_t bytes, const bool staging = false) {
  VkBufferCreateInfo info{VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO};
  info.size = bytes;
  info.usage = VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_SRC_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT;
  VmaAllocationCreateInfo allocation{};
  allocation.usage = VMA_MEMORY_USAGE_AUTO;
  if (staging) {
    allocation.flags = VMA_ALLOCATION_CREATE_HOST_ACCESS_RANDOM_BIT | VMA_ALLOCATION_CREATE_MAPPED_BIT;
    allocation.requiredFlags = VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT;
  }
  return std::make_shared<Buffer>(info, allocation);
}

bool StillActive(const StarPickSnapshot& hit, const std::vector<StarPickRange>& ranges) {
  if (!hit.result.valid || hit.cluster.expired())
    return false;
  return std::any_of(ranges.begin(), ranges.end(), [&](const StarPickRange& range) {
    return range.identity == hit.identity && range.seed == hit.seed && hit.ordinal < range.count &&
           range.cluster.lock() == hit.cluster.lock();
  });
}
}  // namespace

void StarPickState::SetInteractionLocked(const bool locked) {
  if (interaction_locked == locked)
    return;
  interaction_locked = locked;
  current.generation = ++generation;
  current.click = 0;
  ++latest_click;
  hovered = {};
}

void StarPickState::Update(StarPickRequest request, const bool clicked) {
  if (request.camera_handle != current.camera_handle || request.camera.lock() != current.camera.lock() ||
      request.display_size != current.display_size || request.image_origin != current.image_origin ||
      request.population_revision != current.population_revision || request.minimum_radius != current.minimum_radius ||
      request.reference_generation != current.reference_generation) {
    ++generation;
    hovered = {};
  }
  request.generation = generation;
  request.click = clicked && !interaction_locked ? ++latest_click : 0;
  current = std::move(request);
  if (!current.valid || interaction_locked)
    hovered = {};
}

void StarPickState::Complete(const StarPickRequest& request, const StarPickResult& result) {
  if (interaction_locked || request.generation != generation ||
      request.population_revision != current.population_revision)
    return;
  StarPickSnapshot hit;
  if (result.valid) {
    for (const auto& range : request.ranges) {
      if (result.index >= range.offset && result.index - range.offset < range.count) {
        hit = {result, range.identity, range.seed, request.frame, result.index - range.offset, range.cluster};
        break;
      }
    }
    if (!StillActive(hit, current.ranges))
      return;
  }
  if (current.valid && request.valid && request.frame >= hovered.frame) {
    hovered = hit;
    hovered.frame = request.frame;
  }
  if (request.click != 0 && request.click == latest_click && hit.result.valid)
    selected = hit;
}

bool StarPicker::Initialize(const std::shared_ptr<DescriptorSetLayout>& camera_layout,
                            const std::filesystem::path& shader_path) {
  layout_ = std::make_shared<DescriptorSetLayout>();
  for (uint32_t i = 0; i != 4; ++i)
    layout_->PushDescriptorBinding(i, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_COMPUTE_BIT, 0);
  layout_->PushDescriptorBinding(4, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, VK_SHADER_STAGE_COMPUTE_BIT, 0);
  layout_->Initialize();
  for (bool reduce : {false, true}) {
    auto shader = std::make_shared<Shader>();
    auto defines = Platform::GetShaderGlobalDefines();
    if (reduce)
      defines += "\n#define EE_STAR_PICK_REDUCE 1\n";
    if (!shader->TryCompile(ShaderType::Compute, defines, shader_path)) {
      status = "Picking shader compilation failed";
      return false;
    }
    auto& pipeline = reduce ? reduction_ : intersection_;
    pipeline = std::make_shared<ComputePipeline>();
    pipeline->compute_shader = shader;
    pipeline->descriptor_set_layouts = {camera_layout, layout_};
    pipeline->push_constant_ranges.push_back({VK_SHADER_STAGE_COMPUTE_BIT, 0, sizeof(StarPickPushConstant)});
    pipeline->Initialize();
  }
  status = Ready() ? "Ready" : "Picking pipeline initialization failed";
  return Ready();
}

void StarPicker::EnsureResources(const size_t frame_count, const size_t capacity) {
  if (!Ready() || (slots_.size() == frame_count && capacity_ >= capacity))
    return;
  // The layer has completed in-flight submissions before changing shared batch allocations.
  slots_.assign(frame_count, {});
  capacity_ = (std::max)(capacity, size_t{1});
  const size_t groups = (capacity_ + 255) / 256;
  for (auto& slot : slots_) {
    for (auto& scratch : slot.scratch)
      scratch = PickBuffer(groups * 16);
    slot.final_result = PickBuffer(sizeof(StarPickResult));
    slot.staging = PickBuffer(sizeof(StarPickResult), true);
    for (uint32_t i = 0; i != 2; ++i) {
      auto& descriptor = slot.descriptors[i];
      descriptor = std::make_shared<DescriptorSet>(layout_);
      descriptor->UpdateBufferDescriptorBinding(1, slot.scratch[i]);
      descriptor->UpdateBufferDescriptorBinding(2, slot.scratch[1 - i]);
      descriptor->UpdateBufferDescriptorBinding(3, slot.final_result);
    }
  }
}

void StarPicker::Reset() {
  slots_.clear();
  capacity_ = 0;
  const auto next_generation = state.generation + 1;
  state = {};
  state.generation = next_generation;
  status = Ready() ? "Scene reset" : "Not initialized";
}

bool StarPicker::Pending() const {
  return std::any_of(slots_.begin(), slots_.end(), [](const Slot& slot) {
    return slot.pending;
  });
}

bool StarPicker::PendingClick() const {
  return std::any_of(slots_.begin(), slots_.end(), [&](const Slot& slot) {
    return slot.pending && slot.request.click != 0 && slot.request.click == state.latest_click &&
           slot.request.generation == state.generation;
  });
}

void StarPicker::Consume(const uint32_t slot_index) {
  if (slot_index >= slots_.size() || !slots_[slot_index].pending)
    return;
  const ProfilerScope scope(universe_profiler::GetItems().pick_readback);
  auto& slot = slots_[slot_index];
  // Coherent persistent mapping: the engine already waited for this reused frame slot.
  StarPickResult result;
  memcpy(&result, slot.staging->GetVmaAllocationInfo().pMappedData, sizeof(result));
  state.Complete(slot.request, result);
  slot.pending = false;
  status = result.valid ? "Readback completed: hit" : "Readback completed: no hit";
}

void StarPicker::Record(const VkCommandBuffer command, const uint32_t slot_index, const std::shared_ptr<Buffer>& stars,
                        const std::shared_ptr<Camera>& camera, const int32_t camera_index,
                        const std::shared_ptr<DescriptorSet>& camera_descriptor, StarPickRequest request) {
  if (!Ready() || slot_index >= slots_.size() || request.camera.lock() != camera)
    return;
  auto& slot = slots_[slot_index];
  if (slot.pending)
    return;  // A camera may be rendered more than once in the frame.
  const auto target = camera->GetRenderTexture();
  const auto depth = target->GetDepthImage();
  slot.depth_view = target->GetDepthImageView();
  for (auto& descriptor : slot.descriptors) {
    descriptor->UpdateBufferDescriptorBinding(0, stars);
    descriptor->UpdateImageDescriptorBinding(
        4, {target->GetDepthSampler()->GetVkSampler(), slot.depth_view->GetVkImageView(), VK_IMAGE_LAYOUT_GENERAL});
  }
  // ForwardExternal's render graph owns ATTACHMENT_OPTIMAL; restore it before drawing stars.
  depth->TransitImageLayout(command, VK_IMAGE_LAYOUT_GENERAL, VK_IMAGE_LAYOUT_GENERAL, VK_QUEUE_FAMILY_IGNORED,
                            VK_QUEUE_FAMILY_IGNORED, true);
  Platform::BufferMemoryBarrier(command, *stars, VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT, VK_ACCESS_2_SHADER_WRITE_BIT,
                                VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT, VK_ACCESS_2_SHADER_READ_BIT);
  StarPickPushConstant constants;
  constants.camera_index = camera_index;
  for (const auto& range : request.ranges)
    constants.count += range.count;
  constants.cursor_uv = request.cursor_uv;
  constants.display_size = request.display_size;
  constants.minimum_radius = request.minimum_radius;
  constants.ray_valid = request.valid;
  uint32_t groups = (std::max)(1u, Platform::DivUp(constants.count, 256));
  constants.final_pass = groups == 1;
  {
    const GpuProfilerCommandScope scope(command, universe_profiler::GetItems().pick_intersection);
    intersection_->Bind(command);
    intersection_->BindDescriptorSet(command, 0, camera_descriptor->GetVkDescriptorSet());
    intersection_->BindDescriptorSet(command, 1, slot.descriptors[0]->GetVkDescriptorSet());
    intersection_->PushConstant(command, 0, constants);
    intersection_->Dispatch(command, groups);
  }
  uint32_t input = 1;
  {
    const GpuProfilerCommandScope scope(command, universe_profiler::GetItems().pick_reduction);
    while (groups > 1) {
      Platform::BufferMemoryBarrier(command, *slot.scratch[input], VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT,
                                    VK_ACCESS_2_SHADER_WRITE_BIT, VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT,
                                    VK_ACCESS_2_SHADER_READ_BIT);
      // The next output was an input two passes ago (WAR when ping-ponging).
      Platform::BufferMemoryBarrier(command, *slot.scratch[1 - input], VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT,
                                    VK_ACCESS_2_SHADER_READ_BIT, VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT,
                                    VK_ACCESS_2_SHADER_WRITE_BIT);
      constants.count = groups;
      groups = Platform::DivUp(groups, 256);
      constants.final_pass = groups == 1;
      reduction_->Bind(command);
      reduction_->BindDescriptorSet(command, 0, camera_descriptor->GetVkDescriptorSet());
      reduction_->BindDescriptorSet(command, 1, slot.descriptors[input]->GetVkDescriptorSet());
      reduction_->PushConstant(command, 0, constants);
      reduction_->Dispatch(command, groups);
      input = 1 - input;
    }
  }
  Platform::BufferMemoryBarrier(command, *slot.final_result, VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT,
                                VK_ACCESS_2_SHADER_WRITE_BIT, VK_PIPELINE_STAGE_2_TRANSFER_BIT,
                                VK_ACCESS_2_TRANSFER_READ_BIT);
  Platform::CopyBuffer(command, *slot.final_result, *slot.staging, sizeof(StarPickResult));
  Platform::BufferMemoryBarrier(command, *slot.staging, VK_PIPELINE_STAGE_2_TRANSFER_BIT,
                                VK_ACCESS_2_TRANSFER_WRITE_BIT, VK_PIPELINE_STAGE_2_HOST_BIT,
                                VK_ACCESS_2_HOST_READ_BIT);
  depth->TransitImageLayout(command, VK_IMAGE_LAYOUT_GENERAL, VK_IMAGE_LAYOUT_GENERAL, VK_QUEUE_FAMILY_IGNORED,
                            VK_QUEUE_FAMILY_IGNORED, true);
  slot.request = std::move(request);
  slot.pending = true;
}
