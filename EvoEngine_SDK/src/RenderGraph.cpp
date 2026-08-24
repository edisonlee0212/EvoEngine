#include "RenderGraph.hpp"

#include "GraphicsResources.hpp"
#include "Platform.hpp"
#include "RenderTexture.hpp"

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <utility>

using namespace evo_engine;

namespace {
bool IsWriteAccess(const RenderResourceUsage usage) {
  return usage == RenderResourceUsage::Write || usage == RenderResourceUsage::ReadWrite;
}

bool SameDimensions(const RenderResourceDimensions& left, const RenderResourceDimensions& right) {
  return left.size_mode == right.size_mode && left.width == right.width && left.height == right.height &&
         left.depth == right.depth && left.layers == right.layers && left.mip_levels == right.mip_levels;
}

bool SameResource(const RenderResourceDescriptor& left, const RenderResourceDescriptor& right) {
  return left.name == right.name && left.type == right.type && left.lifetime == right.lifetime &&
         SameDimensions(left.dimensions, right.dimensions) && left.format_name == right.format_name &&
         left.sample_count == right.sample_count && left.history_length == right.history_length &&
         left.managed_by_graph == right.managed_by_graph && left.byte_size == right.byte_size;
}

bool SameAccess(const RenderResourceAccess& left, const RenderResourceAccess& right) {
  return left.resource_name == right.resource_name && left.usage == right.usage && left.state == right.state;
}

bool SamePass(const RenderPassDescriptor& left, const RenderPassDescriptor& right) {
  return left.name == right.name && left.queue == right.queue && left.scope == right.scope &&
         std::equal(left.resources.begin(), left.resources.end(), right.resources.begin(), right.resources.end(),
                    SameAccess) &&
         left.resources.size() == right.resources.size() && left.dependencies == right.dependencies &&
         left.profiler_group == right.profiler_group && left.profiler_display_name == right.profiler_display_name;
}

bool SameCompileContext(const RenderGraphCompileContext& left, const RenderGraphCompileContext& right) {
  return left.frame_width == right.frame_width && left.frame_height == right.frame_height &&
         left.camera_width == right.camera_width && left.camera_height == right.camera_height;
}

bool CanAllocateGraphicsResources() {
  try {
    return Platform::Initialized();
  } catch (...) {
    return false;
  }
}

std::vector<uint32_t> GetGraphicsComputeQueueFamilyIndices() {
  try {
    if (Platform::Initialized()) {
      return Platform::GetGraphicsComputeQueueFamilyIndices();
    }
  } catch (...) {
  }
  return {};
}

void ApplyQueueSharing(VkImageCreateInfo& image_info, const std::vector<uint32_t>& queue_family_indices) {
  if (queue_family_indices.size() <= 1) {
    image_info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
    return;
  }
  image_info.sharingMode = VK_SHARING_MODE_CONCURRENT;
  image_info.queueFamilyIndexCount = static_cast<uint32_t>(queue_family_indices.size());
  image_info.pQueueFamilyIndices = queue_family_indices.data();
}

void ApplyQueueSharing(VkBufferCreateInfo& buffer_info, const std::vector<uint32_t>& queue_family_indices) {
  if (queue_family_indices.size() <= 1) {
    buffer_info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
    return;
  }
  buffer_info.sharingMode = VK_SHARING_MODE_CONCURRENT;
  buffer_info.queueFamilyIndexCount = static_cast<uint32_t>(queue_family_indices.size());
  buffer_info.pQueueFamilyIndices = queue_family_indices.data();
}

VkSampleCountFlagBits ToVkSampleCount(const uint32_t sample_count) {
  switch (sample_count) {
    case 2:
      return VK_SAMPLE_COUNT_2_BIT;
    case 4:
      return VK_SAMPLE_COUNT_4_BIT;
    case 8:
      return VK_SAMPLE_COUNT_8_BIT;
    case 16:
      return VK_SAMPLE_COUNT_16_BIT;
    case 32:
      return VK_SAMPLE_COUNT_32_BIT;
    case 64:
      return VK_SAMPLE_COUNT_64_BIT;
    case 1:
    default:
      return VK_SAMPLE_COUNT_1_BIT;
  }
}

VkFormat ToVkFormat(const std::string& format_name) {
  if (format_name == "Depth") {
    return Platform::Constants::render_texture_depth;
  }
  if (format_name == "DepthPyramid") {
    return VK_FORMAT_R32_SFLOAT;
  }
  if (format_name == "GBuffer") {
    return Platform::Constants::g_buffer_attribute;
  }
  if (format_name == "RG16F") {
    return VK_FORMAT_R16G16_SFLOAT;
  }
  if (format_name == "R16F") {
    return VK_FORMAT_R16_SFLOAT;
  }
  if (format_name == "RG32F") {
    return VK_FORMAT_R32G32_SFLOAT;
  }
  if (format_name == "R32F") {
    return VK_FORMAT_R32_SFLOAT;
  }
  if (format_name == "R32U") {
    return VK_FORMAT_R32_UINT;
  }
  if (format_name == "RGBA32UI") {
    return VK_FORMAT_R32G32B32A32_UINT;
  }
  if (format_name == "RGBA16F") {
    return VK_FORMAT_R16G16B16A16_SFLOAT;
  }
  if (format_name == "RGBA32F") {
    return VK_FORMAT_R32G32B32A32_SFLOAT;
  }
  if (format_name == "RGBA8") {
    return VK_FORMAT_R8G8B8A8_UNORM;
  }
  if (format_name == "Color" || format_name == "Radiance" || format_name.empty()) {
    return Platform::Constants::render_texture_color;
  }
  return VK_FORMAT_UNDEFINED;
}

bool IsDepthFormat(const VkFormat format) {
  return format == VK_FORMAT_D16_UNORM || format == VK_FORMAT_D32_SFLOAT || format == VK_FORMAT_D16_UNORM_S8_UINT ||
         format == VK_FORMAT_D24_UNORM_S8_UINT || format == VK_FORMAT_D32_SFLOAT_S8_UINT;
}

VkImageUsageFlags ToVkImageUsage(const std::vector<RenderResourceState>& required_states, const VkFormat format) {
  VkImageUsageFlags usage = VK_IMAGE_USAGE_TRANSFER_SRC_BIT | VK_IMAGE_USAGE_TRANSFER_DST_BIT |
                            VK_IMAGE_USAGE_SAMPLED_BIT | VK_IMAGE_USAGE_STORAGE_BIT;
  for (const auto state : required_states) {
    switch (state) {
      case RenderResourceState::ColorAttachment:
        usage |= VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT;
        break;
      case RenderResourceState::DepthAttachment:
        usage |= VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT;
        break;
      case RenderResourceState::ShaderRead:
        usage |= VK_IMAGE_USAGE_SAMPLED_BIT;
        break;
      case RenderResourceState::StorageReadWrite:
        usage |= VK_IMAGE_USAGE_STORAGE_BIT;
        break;
      case RenderResourceState::TransferSource:
        usage |= VK_IMAGE_USAGE_TRANSFER_SRC_BIT;
        break;
      case RenderResourceState::TransferDestination:
      case RenderResourceState::TransferDestinationGeneral:
        usage |= VK_IMAGE_USAGE_TRANSFER_DST_BIT;
        break;
      case RenderResourceState::Undefined:
      case RenderResourceState::Present:
      case RenderResourceState::AccelerationStructureRead:
      case RenderResourceState::General:
        break;
    }
  }
  if (IsDepthFormat(format)) {
    usage &= ~VK_IMAGE_USAGE_STORAGE_BIT;
    usage |= VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT;
  }
  return usage;
}

VkBufferUsageFlags ToVkBufferUsage(const std::vector<RenderResourceState>& required_states) {
  VkBufferUsageFlags usage = VK_BUFFER_USAGE_TRANSFER_SRC_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT |
                             VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT;
  for (const auto state : required_states) {
    switch (state) {
      case RenderResourceState::TransferSource:
        usage |= VK_BUFFER_USAGE_TRANSFER_SRC_BIT;
        break;
      case RenderResourceState::TransferDestination:
      case RenderResourceState::TransferDestinationGeneral:
        usage |= VK_BUFFER_USAGE_TRANSFER_DST_BIT;
        break;
      case RenderResourceState::ShaderRead:
      case RenderResourceState::StorageReadWrite:
      case RenderResourceState::General:
        usage |= VK_BUFFER_USAGE_STORAGE_BUFFER_BIT;
        break;
      case RenderResourceState::Undefined:
      case RenderResourceState::ColorAttachment:
      case RenderResourceState::DepthAttachment:
      case RenderResourceState::Present:
      case RenderResourceState::AccelerationStructureRead:
        break;
    }
  }
  return usage;
}

std::shared_ptr<Image> CreateTransientImage(const RenderResourceAllocationPlan& allocation) {
  const auto format = ToVkFormat(allocation.format_name);
  if (format == VK_FORMAT_UNDEFINED || allocation.resolved_dimensions.width == 0 ||
      allocation.resolved_dimensions.height == 0 || allocation.resolved_dimensions.mip_levels == 0) {
    return {};
  }
  VkImageCreateInfo image_info{};
  image_info.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
  image_info.imageType = allocation.resolved_dimensions.depth > 1 ? VK_IMAGE_TYPE_3D : VK_IMAGE_TYPE_2D;
  image_info.extent = {allocation.resolved_dimensions.width, allocation.resolved_dimensions.height,
                       allocation.resolved_dimensions.depth};
  image_info.mipLevels = allocation.resolved_dimensions.mip_levels;
  image_info.arrayLayers = allocation.resolved_dimensions.layers;
  image_info.format = format;
  image_info.tiling = VK_IMAGE_TILING_OPTIMAL;
  image_info.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
  image_info.usage = ToVkImageUsage(allocation.required_states, format);
  image_info.samples = ToVkSampleCount(allocation.sample_count);
  const auto queue_family_indices = GetGraphicsComputeQueueFamilyIndices();
  ApplyQueueSharing(image_info, queue_family_indices);
  return std::make_shared<Image>(image_info);
}

std::shared_ptr<Buffer> CreateTransientBuffer(const RenderResourceAllocationPlan& allocation) {
  if (allocation.byte_size == 0) {
    return {};
  }
  VkBufferCreateInfo buffer_info{};
  buffer_info.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
  buffer_info.size = allocation.byte_size;
  buffer_info.usage = ToVkBufferUsage(allocation.required_states);
  const auto queue_family_indices = GetGraphicsComputeQueueFamilyIndices();
  ApplyQueueSharing(buffer_info, queue_family_indices);
  return std::make_shared<Buffer>(buffer_info);
}

void AddRequiredState(std::vector<RenderResourceState>& states, const RenderResourceState state) {
  if (state == RenderResourceState::Undefined || std::find(states.begin(), states.end(), state) != states.end()) {
    return;
  }
  states.emplace_back(state);
}

uint32_t CalculateFullMipChainLevels(uint32_t width, uint32_t height) {
  uint32_t longest_edge = std::max(width, height);
  if (longest_edge == 0) {
    return 0;
  }
  uint32_t levels = 1;
  while (longest_edge > 1) {
    longest_edge /= 2;
    levels++;
  }
  return levels;
}

uint32_t ResolveRelativeDimension(const uint32_t base_dimension, const uint32_t divisor) {
  if (base_dimension == 0) {
    return 0;
  }
  const uint32_t safe_divisor = std::max(divisor, 1u);
  return std::max(1u, (base_dimension + safe_divisor - 1u) / safe_divisor);
}

RenderResourceDimensions ResolveDimensions(RenderResourceDimensions dimensions,
                                           const RenderGraphCompileContext& context) {
  switch (dimensions.size_mode) {
    case RenderResourceSizeMode::FrameRelative:
      if (context.frame_width != 0 && context.frame_height != 0) {
        dimensions.width = ResolveRelativeDimension(context.frame_width, dimensions.width);
        dimensions.height = ResolveRelativeDimension(context.frame_height, dimensions.height);
        dimensions.size_mode = RenderResourceSizeMode::Absolute;
      }
      break;
    case RenderResourceSizeMode::CameraRelative:
      if (context.camera_width != 0 && context.camera_height != 0) {
        dimensions.width = ResolveRelativeDimension(context.camera_width, dimensions.width);
        dimensions.height = ResolveRelativeDimension(context.camera_height, dimensions.height);
        dimensions.size_mode = RenderResourceSizeMode::Absolute;
      }
      break;
    case RenderResourceSizeMode::None:
    case RenderResourceSizeMode::Absolute:
      break;
  }
  if (dimensions.depth == 0) {
    dimensions.depth = 1;
  }
  if (dimensions.layers == 0) {
    dimensions.layers = 1;
  }
  if (dimensions.mip_levels == 0) {
    dimensions.mip_levels = CalculateFullMipChainLevels(dimensions.width, dimensions.height);
  }
  return dimensions;
}

bool AreDimensionsCompatible(const RenderResourceDimensions& lhs, const RenderResourceDimensions& rhs) {
  return lhs.size_mode == rhs.size_mode && lhs.width == rhs.width && lhs.height == rhs.height &&
         lhs.depth == rhs.depth && lhs.layers == rhs.layers && lhs.mip_levels == rhs.mip_levels;
}

bool AreAllocationResourcesCompatible(const RenderResourceDescriptor& lhs, const RenderResourceDescriptor& rhs) {
  return lhs.type == rhs.type && lhs.format_name == rhs.format_name && lhs.sample_count == rhs.sample_count &&
         lhs.byte_size == rhs.byte_size && AreDimensionsCompatible(lhs.dimensions, rhs.dimensions);
}

void AddRequiredStates(std::vector<RenderResourceState>& target, const std::vector<RenderResourceState>& source) {
  for (const auto state : source) {
    AddRequiredState(target, state);
  }
}

void AddUniquePassIndex(std::vector<size_t>& pass_indices, const size_t pass_index) {
  if (pass_index == RenderGraphConstants::invalid_pass_index ||
      std::find(pass_indices.begin(), pass_indices.end(), pass_index) != pass_indices.end()) {
    return;
  }
  pass_indices.emplace_back(pass_index);
}

void AddExplicitScheduleDependency(RenderPassExecutionPlan& pass_plan, const size_t dependency_index) {
  if (dependency_index == pass_plan.pass_index) {
    return;
  }
  AddUniquePassIndex(pass_plan.dependency_indices, dependency_index);
  AddUniquePassIndex(pass_plan.schedule_dependency_indices, dependency_index);
}

void AddResourceScheduleDependency(RenderPassExecutionPlan& pass_plan, const size_t dependency_index) {
  if (dependency_index == pass_plan.pass_index) {
    return;
  }
  AddUniquePassIndex(pass_plan.resource_dependency_indices, dependency_index);
  AddUniquePassIndex(pass_plan.schedule_dependency_indices, dependency_index);
}

void AddPassToScheduleStep(RenderGraphScheduleStep& schedule_step, const RenderPassQueue queue,
                           const size_t pass_index) {
  switch (queue) {
    case RenderPassQueue::Graphics:
      schedule_step.graphics_pass_indices.emplace_back(pass_index);
      break;
    case RenderPassQueue::Compute:
      schedule_step.compute_pass_indices.emplace_back(pass_index);
      break;
    case RenderPassQueue::RayTracing:
      schedule_step.ray_tracing_pass_indices.emplace_back(pass_index);
      break;
  }
}

std::vector<RenderGraphScheduleStep> BuildScheduleSteps(std::vector<RenderPassExecutionPlan>& pass_plans) {
  std::vector<RenderGraphScheduleStep> schedule_steps;
  for (auto& pass_plan : pass_plans) {
    size_t schedule_step_index = 0;
    for (const auto dependency_index : pass_plan.schedule_dependency_indices) {
      if (dependency_index < pass_plans.size()) {
        schedule_step_index = std::max(schedule_step_index, pass_plans[dependency_index].schedule_step_index + 1);
      }
    }
    pass_plan.schedule_step_index = schedule_step_index;
    if (schedule_step_index >= schedule_steps.size()) {
      schedule_steps.resize(schedule_step_index + 1);
    }
    AddPassToScheduleStep(schedule_steps[schedule_step_index], pass_plan.queue, pass_plan.pass_index);
  }
  return schedule_steps;
}

std::vector<size_t> GetScheduleStepPassIndices(const RenderGraphScheduleStep& schedule_step) {
  std::vector<size_t> pass_indices;
  pass_indices.reserve(schedule_step.graphics_pass_indices.size() + schedule_step.compute_pass_indices.size() +
                       schedule_step.ray_tracing_pass_indices.size());
  pass_indices.insert(pass_indices.end(), schedule_step.graphics_pass_indices.begin(),
                      schedule_step.graphics_pass_indices.end());
  pass_indices.insert(pass_indices.end(), schedule_step.compute_pass_indices.begin(),
                      schedule_step.compute_pass_indices.end());
  pass_indices.insert(pass_indices.end(), schedule_step.ray_tracing_pass_indices.begin(),
                      schedule_step.ray_tracing_pass_indices.end());
  std::sort(pass_indices.begin(), pass_indices.end());
  return pass_indices;
}

bool CanShareAllocationSlot(const RenderResourceAllocationPlan& allocation,
                            const std::vector<RenderResourceUsagePlan>& resource_plans,
                            const RenderResourceUsagePlan& candidate_plan) {
  for (const auto resource_index : allocation.resource_indices) {
    const auto& resource_plan = resource_plans[resource_index];
    if (candidate_plan.first_pass_index <= resource_plan.last_pass_index &&
        resource_plan.first_pass_index <= candidate_plan.last_pass_index) {
      return false;
    }
  }
  return true;
}

bool TryCreateBarrierPlan(RenderResourceBarrierPlan& barrier_plan, const RenderResourceDescriptor& resource,
                          const RenderResourceTransitionPlan& transition, const bool state_change,
                          const size_t transition_index) {
  if (resource.type == RenderResourceType::Image) {
    if (state_change && transition.next_state != RenderResourceState::Undefined) {
      barrier_plan.barrier_type = RenderGraphBarrierType::ImageLayout;
    } else if (transition.memory_dependency) {
      barrier_plan.barrier_type = RenderGraphBarrierType::ImageMemory;
    } else {
      return false;
    }
  } else if (resource.type == RenderResourceType::Buffer) {
    if (!transition.memory_dependency) {
      return false;
    }
    barrier_plan.barrier_type = RenderGraphBarrierType::BufferMemory;
  } else {
    if (!transition.memory_dependency) {
      return false;
    }
    barrier_plan.barrier_type = RenderGraphBarrierType::GlobalMemory;
  }
  barrier_plan.transition_index = transition_index;
  barrier_plan.resource_index = transition.resource_index;
  barrier_plan.pass_index = transition.pass_index;
  barrier_plan.previous_pass_index = transition.previous_pass_index;
  barrier_plan.resource_type = resource.type;
  barrier_plan.previous_queue = transition.previous_queue;
  barrier_plan.next_queue = transition.next_queue;
  barrier_plan.previous_usage = transition.previous_usage;
  barrier_plan.next_usage = transition.next_usage;
  barrier_plan.previous_state = transition.previous_state;
  barrier_plan.next_state = transition.next_state;
  barrier_plan.queue_change = transition.queue_change;
  barrier_plan.memory_dependency = transition.memory_dependency;
  return true;
}
}  // namespace

void RenderGraphResourceRegistry::Clear() {
  resource_bindings_.clear();
}

void RenderGraphResourceRegistry::BindBuffer(const std::string& resource_name, std::shared_ptr<Buffer> buffer) {
  GetOrAddResourceBinding(resource_name).buffer = std::move(buffer);
}

void RenderGraphResourceRegistry::BindImage(const std::string& resource_name, std::shared_ptr<Image> image) {
  auto& binding = GetOrAddResourceBinding(resource_name);
  binding.image = std::move(image);
  binding.images.clear();
  if (binding.image) {
    binding.images.emplace_back(binding.image);
  }
}

void RenderGraphResourceRegistry::BindImages(const std::string& resource_name,
                                             std::vector<std::shared_ptr<Image>> images) {
  auto& binding = GetOrAddResourceBinding(resource_name);
  binding.images = std::move(images);
  binding.image = binding.images.empty() ? nullptr : binding.images.front();
}

void RenderGraphResourceRegistry::BindDescriptorSet(const std::string& resource_name,
                                                    std::shared_ptr<DescriptorSet> descriptor_set) {
  GetOrAddResourceBinding(resource_name).descriptor_set = std::move(descriptor_set);
}

void RenderGraphResourceRegistry::BindRenderTexture(const std::string& resource_name,
                                                    std::shared_ptr<RenderTexture> render_texture) {
  GetOrAddResourceBinding(resource_name).render_texture = std::move(render_texture);
}

bool RenderGraphResourceRegistry::HasResourceBinding(const std::string& resource_name) const {
  return GetResourceBinding(resource_name) != nullptr;
}

const RenderGraphResourceBinding* RenderGraphResourceRegistry::GetResourceBinding(
    const std::string& resource_name) const {
  const auto binding =
      std::find_if(resource_bindings_.begin(), resource_bindings_.end(), [&](const RenderGraphResourceBinding& entry) {
        return entry.resource_name == resource_name;
      });
  if (binding == resource_bindings_.end()) {
    return nullptr;
  }
  return &*binding;
}

const std::vector<RenderGraphResourceBinding>& RenderGraphResourceRegistry::GetResourceBindings() const {
  return resource_bindings_;
}

RenderGraphResourceBinding& RenderGraphResourceRegistry::GetOrAddResourceBinding(const std::string& resource_name) {
  const auto binding =
      std::find_if(resource_bindings_.begin(), resource_bindings_.end(), [&](const RenderGraphResourceBinding& entry) {
        return entry.resource_name == resource_name;
      });
  if (binding != resource_bindings_.end()) {
    return *binding;
  }
  resource_bindings_.push_back({resource_name});
  return resource_bindings_.back();
}

void RenderGraphTransientResourceStore::Clear() {
  assets_.clear();
  descriptor_sets_.clear();
  samplers_.clear();
  image_views_.clear();
  render_textures_.clear();
  resource_bindings_.clear();
  images_.clear();
  buffers_.clear();
}

void RenderGraphTransientResourceStore::Allocate(const std::vector<RenderResourceDescriptor>& resources,
                                                 const RenderGraphExecutionPlan& execution_plan) {
  Clear();
  if (!execution_plan.valid || !CanAllocateGraphicsResources()) {
    return;
  }
  for (const auto& allocation : execution_plan.allocations) {
    std::shared_ptr<Image> image;
    std::shared_ptr<Buffer> buffer;
    if (allocation.type == RenderResourceType::Image) {
      image = CreateTransientImage(allocation);
      if (image) {
        images_.emplace_back(image);
      }
    } else if (allocation.type == RenderResourceType::Buffer) {
      buffer = CreateTransientBuffer(allocation);
      if (buffer) {
        buffers_.emplace_back(buffer);
      }
    }
    if (!image && !buffer) {
      continue;
    }
    for (const auto resource_index : allocation.resource_indices) {
      if (resource_index >= resources.size()) {
        continue;
      }
      RenderGraphResourceBinding binding;
      binding.resource_name = resources[resource_index].name;
      binding.buffer = buffer;
      binding.image = image;
      if (image) {
        binding.images.emplace_back(image);
      }
      resource_bindings_.emplace_back(std::move(binding));
    }
  }
}

void RenderGraphTransientResourceStore::Bind(RenderGraphResourceRegistry& resource_registry) {
  for (const auto& binding : resource_bindings_) {
    if (binding.buffer) {
      resource_registry.BindBuffer(binding.resource_name, binding.buffer);
    }
    if (binding.image) {
      resource_registry.BindImage(binding.resource_name, binding.image);
    }
  }
  for (const auto& binding : resource_registry.GetResourceBindings()) {
    if (binding.buffer) {
      buffers_.emplace_back(binding.buffer);
    }
    if (binding.image) {
      images_.emplace_back(binding.image);
    }
    images_.insert(images_.end(), binding.images.begin(), binding.images.end());
    if (binding.descriptor_set) {
      descriptor_sets_.emplace_back(binding.descriptor_set);
    }
    if (binding.render_texture) {
      RetainRenderTextureResources(binding.render_texture);
    }
  }
}

void RenderGraphTransientResourceStore::RetainDescriptorSet(std::shared_ptr<DescriptorSet> descriptor_set) {
  if (descriptor_set) {
    descriptor_sets_.emplace_back(std::move(descriptor_set));
  }
}

void RenderGraphTransientResourceStore::RetainBuffer(std::shared_ptr<Buffer> buffer) {
  if (buffer) {
    buffers_.emplace_back(std::move(buffer));
  }
}

void RenderGraphTransientResourceStore::RetainAsset(std::shared_ptr<IAsset> asset) {
  if (asset) {
    assets_.emplace_back(std::move(asset));
  }
}

void RenderGraphTransientResourceStore::RetainRenderTextureResources(std::shared_ptr<RenderTexture> render_texture) {
  if (!render_texture) {
    return;
  }
  const auto has_color = render_texture->HasColorAttachment();
  const auto has_depth = render_texture->HasDepthAttachment();
  if (has_color) {
    images_.emplace_back(render_texture->GetColorImage());
    samplers_.emplace_back(render_texture->GetColorSampler());
    descriptor_sets_.emplace_back(render_texture->GetColorPresentDescriptorSet());
    descriptor_sets_.emplace_back(render_texture->GetStorageDescriptorSet());
  }
  if (has_depth) {
    images_.emplace_back(render_texture->GetDepthImage());
    samplers_.emplace_back(render_texture->GetDepthSampler());
    descriptor_sets_.emplace_back(render_texture->GetDepthPresentDescriptorSet());
  }
  for (uint32_t mip_level = 0; mip_level < render_texture->GetMipLevels(); ++mip_level) {
    if (has_color) {
      image_views_.emplace_back(render_texture->GetColorImageView(mip_level));
    }
    if (has_depth) {
      image_views_.emplace_back(render_texture->GetDepthImageView(mip_level));
    }
  }
  render_textures_.emplace_back(std::move(render_texture));
}

void RenderGraphTransientResourceStore::RetainImage(std::shared_ptr<Image> image) {
  if (image) {
    images_.emplace_back(std::move(image));
  }
}

void RenderGraphTransientResourceStore::RetainImageView(std::shared_ptr<ImageView> image_view) {
  if (image_view) {
    image_views_.emplace_back(std::move(image_view));
  }
}

const std::vector<RenderGraphResourceBinding>& RenderGraphTransientResourceStore::GetResourceBindings() const {
  return resource_bindings_;
}

RenderGraphExecutionContext::RenderGraphExecutionContext(const std::vector<RenderResourceDescriptor>& resources,
                                                         const std::vector<RenderPassDescriptor>& passes,
                                                         const RenderGraphExecutionPlan& execution_plan,
                                                         const RenderGraphResourceRegistry* resource_registry,
                                                         const size_t current_pass_index)
    : resources_(resources),
      passes_(passes),
      execution_plan_(execution_plan),
      resource_registry_(resource_registry),
      current_pass_index_(current_pass_index) {
}

size_t RenderGraphExecutionContext::GetCurrentPassIndex() const {
  return current_pass_index_;
}

const RenderPassDescriptor* RenderGraphExecutionContext::GetCurrentPassDescriptor() const {
  if (current_pass_index_ >= passes_.size()) {
    return nullptr;
  }
  return &passes_[current_pass_index_];
}

const RenderGraphExecutionPlan& RenderGraphExecutionContext::GetExecutionPlan() const {
  return execution_plan_;
}

const RenderResourceDescriptor* RenderGraphExecutionContext::GetResourceDescriptor(const size_t resource_index) const {
  if (resource_index >= resources_.size()) {
    return nullptr;
  }
  return &resources_[resource_index];
}

const RenderResourceDescriptor* RenderGraphExecutionContext::GetResourceDescriptor(
    const std::string& resource_name) const {
  const auto resource =
      std::find_if(resources_.begin(), resources_.end(), [&](const RenderResourceDescriptor& descriptor) {
        return descriptor.name == resource_name;
      });
  if (resource == resources_.end()) {
    return nullptr;
  }
  return &*resource;
}

std::vector<const RenderResourceTransitionPlan*> RenderGraphExecutionContext::GetCurrentPassTransitions() const {
  std::vector<const RenderResourceTransitionPlan*> transitions;
  for (const auto& transition : execution_plan_.transitions) {
    if (transition.pass_index == current_pass_index_) {
      transitions.emplace_back(&transition);
    }
  }
  return transitions;
}

std::vector<const RenderResourceBarrierPlan*> RenderGraphExecutionContext::GetCurrentPassBarriers() const {
  std::vector<const RenderResourceBarrierPlan*> barriers;
  for (const auto& barrier : execution_plan_.barriers) {
    if (barrier.pass_index == current_pass_index_) {
      barriers.emplace_back(&barrier);
    }
  }
  return barriers;
}

std::vector<const RenderResourceBarrierPlan*> RenderGraphExecutionContext::GetCurrentPassReleaseBarriers() const {
  std::vector<const RenderResourceBarrierPlan*> barriers;
  for (const auto& barrier : execution_plan_.barriers) {
    if (barrier.queue_change && barrier.previous_pass_index == current_pass_index_) {
      barriers.emplace_back(&barrier);
    }
  }
  return barriers;
}

const RenderResourceUsagePlan* RenderGraphExecutionContext::GetResourceUsagePlan(
    const std::string& resource_name) const {
  const auto resource =
      std::find_if(resources_.begin(), resources_.end(), [&](const RenderResourceDescriptor& descriptor) {
        return descriptor.name == resource_name;
      });
  if (resource == resources_.end()) {
    return nullptr;
  }
  const auto resource_index = static_cast<size_t>(resource - resources_.begin());
  if (resource_index >= execution_plan_.resources.size()) {
    return nullptr;
  }
  return &execution_plan_.resources[resource_index];
}

const RenderResourceAllocationPlan* RenderGraphExecutionContext::GetResourceAllocationPlan(
    const std::string& resource_name) const {
  const auto usage_plan = GetResourceUsagePlan(resource_name);
  if (!usage_plan || usage_plan->allocation_slot_index == RenderGraphConstants::invalid_allocation_slot_index ||
      usage_plan->allocation_slot_index >= execution_plan_.allocations.size()) {
    return nullptr;
  }
  return &execution_plan_.allocations[usage_plan->allocation_slot_index];
}

const RenderGraphResourceBinding* RenderGraphExecutionContext::GetResourceBinding(
    const std::string& resource_name) const {
  if (!resource_registry_) {
    return nullptr;
  }
  return resource_registry_->GetResourceBinding(resource_name);
}

void RenderGraph::Clear() {
  resources_.clear();
  passes_.clear();
  execute_functions_.clear();
}

void RenderGraph::AddResource(const RenderResourceDescriptor& descriptor) {
  if (descriptor.name.empty() || HasResource(descriptor.name)) {
    return;
  }
  resources_.emplace_back(descriptor);
}

void RenderGraph::AddPass(const RenderPassDescriptor& descriptor, ExecuteFunction execute) {
  passes_.emplace_back(descriptor);
  execute_functions_.emplace_back(std::move(execute));
}

RenderGraphExecutionPlan RenderGraph::Compile() const {
  return Compile({});
}

RenderGraphExecutionPlan RenderGraph::Compile(const RenderGraphCompileContext& context) const {
  RenderGraphExecutionPlan plan;
  if (!Validate()) {
    return plan;
  }

  plan.valid = true;
  plan.passes.reserve(passes_.size());
  plan.resources.resize(resources_.size());
  std::vector<RenderResourceState> last_states(resources_.size(), RenderResourceState::Undefined);
  std::vector<RenderResourceUsage> last_usages(resources_.size(), RenderResourceUsage::Read);
  std::vector<RenderPassQueue> last_queues(resources_.size(), RenderPassQueue::Graphics);
  std::vector<bool> has_previous_use(resources_.size(), false);
  std::vector<size_t> last_pass_indices(resources_.size(), RenderGraphConstants::invalid_pass_index);
  std::vector<size_t> last_writer_pass_indices(resources_.size(), RenderGraphConstants::invalid_pass_index);
  std::vector<std::vector<size_t>> active_reader_pass_indices(resources_.size());
  for (size_t resource_index = 0; resource_index < resources_.size(); resource_index++) {
    auto& resource_plan = plan.resources[resource_index];
    const auto& resource = resources_[resource_index];
    resource_plan.resource_index = resource_index;
    resource_plan.imported = !resource.managed_by_graph || resource.lifetime == RenderResourceLifetime::Imported ||
                             resource.type == RenderResourceType::External;
    resource_plan.resolved_dimensions = ResolveDimensions(resource.dimensions, context);
  }

  for (size_t pass_index = 0; pass_index < passes_.size(); pass_index++) {
    const auto& pass = passes_[pass_index];
    RenderPassExecutionPlan pass_plan;
    pass_plan.pass_index = pass_index;
    pass_plan.queue = pass.queue;
    plan.uses_graphics_queue = plan.uses_graphics_queue || pass.queue == RenderPassQueue::Graphics;
    plan.uses_compute_queue = plan.uses_compute_queue || pass.queue == RenderPassQueue::Compute;
    plan.uses_ray_tracing_queue = plan.uses_ray_tracing_queue || pass.queue == RenderPassQueue::RayTracing;

    for (const auto& dependency : pass.dependencies) {
      const auto dependency_found =
          std::find_if(passes_.begin(), passes_.begin() + static_cast<std::ptrdiff_t>(pass_index),
                       [&](const RenderPassDescriptor& candidate) {
                         return candidate.name == dependency;
                       });
      AddExplicitScheduleDependency(pass_plan, static_cast<size_t>(dependency_found - passes_.begin()));
    }

    for (const auto& access : pass.resources) {
      const auto resource_found =
          std::find_if(resources_.begin(), resources_.end(), [&](const RenderResourceDescriptor& descriptor) {
            return descriptor.name == access.resource_name;
          });
      const auto resource_index = static_cast<size_t>(resource_found - resources_.begin());
      AddResourceScheduleDependency(pass_plan, last_writer_pass_indices[resource_index]);
      if (IsWriteAccess(access.usage)) {
        for (const auto reader_pass_index : active_reader_pass_indices[resource_index]) {
          AddResourceScheduleDependency(pass_plan, reader_pass_index);
        }
      }
      auto& resource_plan = plan.resources[resource_index];
      resource_plan.used = true;
      if (resource_plan.first_pass_index == RenderGraphConstants::invalid_pass_index) {
        resource_plan.first_pass_index = pass_index;
      }
      resource_plan.last_pass_index = pass_index;
      if (access.usage == RenderResourceUsage::Read || access.usage == RenderResourceUsage::ReadWrite) {
        resource_plan.reader_pass_indices.emplace_back(pass_index);
      }
      if (access.usage == RenderResourceUsage::Write || access.usage == RenderResourceUsage::ReadWrite) {
        resource_plan.writer_pass_indices.emplace_back(pass_index);
      }
      AddRequiredState(resource_plan.required_states, access.state);
      const auto has_previous = has_previous_use[resource_index];
      const auto previous_state = has_previous ? last_states[resource_index] : RenderResourceState::Undefined;
      const auto previous_usage = has_previous ? last_usages[resource_index] : RenderResourceUsage::Read;
      const auto previous_queue = has_previous ? last_queues[resource_index] : pass.queue;
      const auto previous_pass_index =
          has_previous ? last_pass_indices[resource_index] : RenderGraphConstants::invalid_pass_index;
      const auto queue_change = has_previous && previous_queue != pass.queue;
      const auto state_change = previous_state != access.state;
      const auto memory_dependency = has_previous && (state_change || queue_change || IsWriteAccess(previous_usage) ||
                                                      IsWriteAccess(access.usage));
      if (!has_previous || state_change || queue_change || memory_dependency) {
        plan.transitions.push_back({resource_index, pass_index, previous_pass_index, previous_queue, pass.queue,
                                    previous_usage, access.usage, previous_state, access.state, !has_previous,
                                    queue_change, memory_dependency});
        RenderResourceBarrierPlan barrier_plan;
        if (TryCreateBarrierPlan(barrier_plan, resources_[resource_index], plan.transitions.back(), state_change,
                                 plan.transitions.size() - 1)) {
          plan.barriers.emplace_back(std::move(barrier_plan));
        }
      }
      has_previous_use[resource_index] = true;
      last_states[resource_index] = access.state;
      last_usages[resource_index] = access.usage;
      last_queues[resource_index] = pass.queue;
      last_pass_indices[resource_index] = pass_index;
      if (IsWriteAccess(access.usage)) {
        active_reader_pass_indices[resource_index].clear();
        last_writer_pass_indices[resource_index] = pass_index;
      } else {
        AddUniquePassIndex(active_reader_pass_indices[resource_index], pass_index);
      }
    }

    plan.passes.emplace_back(std::move(pass_plan));
  }
  plan.schedule_steps = BuildScheduleSteps(plan.passes);

  for (auto& resource_plan : plan.resources) {
    const auto& resource = resources_[resource_plan.resource_index];
    resource_plan.can_alias =
        resource_plan.used && resource.managed_by_graph &&
        (resource.lifetime == RenderResourceLifetime::Frame || resource.lifetime == RenderResourceLifetime::Camera) &&
        (resource.type == RenderResourceType::Buffer || resource.type == RenderResourceType::Image);
  }
  std::vector<size_t> alias_candidate_indices;
  for (const auto& resource_plan : plan.resources) {
    if (resource_plan.can_alias) {
      alias_candidate_indices.emplace_back(resource_plan.resource_index);
    }
  }
  std::sort(alias_candidate_indices.begin(), alias_candidate_indices.end(), [&](const size_t lhs, const size_t rhs) {
    const auto& lhs_plan = plan.resources[lhs];
    const auto& rhs_plan = plan.resources[rhs];
    if (lhs_plan.first_pass_index == rhs_plan.first_pass_index) {
      return lhs < rhs;
    }
    return lhs_plan.first_pass_index < rhs_plan.first_pass_index;
  });
  for (const auto resource_index : alias_candidate_indices) {
    auto& resource_plan = plan.resources[resource_index];
    const auto& resource = resources_[resource_index];
    bool placed = false;
    for (auto& allocation : plan.allocations) {
      if (!allocation.resource_indices.empty() &&
          AreAllocationResourcesCompatible(resources_[allocation.resource_indices.front()], resource) &&
          CanShareAllocationSlot(allocation, plan.resources, resource_plan)) {
        allocation.resource_indices.emplace_back(resource_index);
        AddRequiredStates(allocation.required_states, resource_plan.required_states);
        resource_plan.allocation_slot_index = allocation.allocation_slot_index;
        placed = true;
        break;
      }
    }
    if (!placed) {
      RenderResourceAllocationPlan allocation;
      allocation.allocation_slot_index = plan.allocations.size();
      allocation.type = resource.type;
      allocation.dimensions = resource.dimensions;
      allocation.resolved_dimensions = resource_plan.resolved_dimensions;
      allocation.format_name = resource.format_name;
      allocation.sample_count = resource.sample_count;
      allocation.byte_size = resource.byte_size;
      allocation.required_states = resource_plan.required_states;
      allocation.resource_indices.emplace_back(resource_index);
      resource_plan.allocation_slot_index = allocation.allocation_slot_index;
      plan.allocations.emplace_back(std::move(allocation));
    }
  }
  return plan;
}

void RenderGraph::Execute() const {
  const RenderGraphResourceRegistry resource_registry;
  Execute(Compile(), resource_registry);
}

void RenderGraph::Execute(const RenderGraphResourceRegistry& resource_registry) const {
  Execute(Compile(), resource_registry);
}

void RenderGraph::Execute(const RenderGraphExecutionPlan& execution_plan,
                          const RenderGraphResourceRegistry& resource_registry) const {
  const auto execute_pass = [&](const size_t pass_index) {
    if (pass_index >= execute_functions_.size()) {
      return;
    }
    RenderGraphExecutionContext context(resources_, passes_, execution_plan, &resource_registry, pass_index);
    execute_functions_[pass_index](context);
  };
  if (!execution_plan.valid || execution_plan.schedule_steps.empty()) {
    for (size_t pass_index = 0; pass_index < execute_functions_.size(); pass_index++) {
      execute_pass(pass_index);
    }
    return;
  }
  for (const auto& schedule_step : execution_plan.schedule_steps) {
    for (const auto pass_index : GetScheduleStepPassIndices(schedule_step)) {
      execute_pass(pass_index);
    }
  }
}

bool RenderGraph::Validate() const {
  if (passes_.size() != execute_functions_.size()) {
    return false;
  }
  for (const auto& resource : resources_) {
    if (resource.sample_count == 0 || resource.history_length == 0) {
      return false;
    }
    if (resource.managed_by_graph && resource.type == RenderResourceType::Image &&
        resource.dimensions.size_mode == RenderResourceSizeMode::None) {
      return false;
    }
  }
  for (size_t pass_index = 0; pass_index < passes_.size(); pass_index++) {
    const auto& pass = passes_[pass_index];
    if (pass.name.empty()) {
      return false;
    }
    const auto duplicate_pass = std::find_if(passes_.begin() + static_cast<std::ptrdiff_t>(pass_index + 1),
                                             passes_.end(), [&](const RenderPassDescriptor& candidate) {
                                               return candidate.name == pass.name;
                                             });
    if (duplicate_pass != passes_.end()) {
      return false;
    }
  }
  for (size_t pass_index = 0; pass_index < passes_.size(); pass_index++) {
    const auto& pass = passes_[pass_index];
    if (!execute_functions_[pass_index]) {
      return false;
    }
    for (const auto& dependency : pass.dependencies) {
      if (dependency.empty() || dependency == pass.name) {
        return false;
      }
      const auto dependency_found =
          std::find_if(passes_.begin(), passes_.begin() + static_cast<std::ptrdiff_t>(pass_index),
                       [&](const RenderPassDescriptor& candidate) {
                         return candidate.name == dependency;
                       });
      if (dependency_found == passes_.begin() + static_cast<std::ptrdiff_t>(pass_index)) {
        return false;
      }
    }
    for (const auto& resource : pass.resources) {
      if (resource.resource_name.empty()) {
        return false;
      }
      const auto resource_found =
          std::find_if(resources_.begin(), resources_.end(), [&](const RenderResourceDescriptor& descriptor) {
            return descriptor.name == resource.resource_name;
          });
      if (resource_found == resources_.end()) {
        return false;
      }
    }
  }
  return true;
}

bool RenderGraph::HasResource(const std::string& name) const {
  return std::find_if(resources_.begin(), resources_.end(), [&](const RenderResourceDescriptor& descriptor) {
           return descriptor.name == name;
         }) != resources_.end();
}

bool RenderGraph::UsesQueue(const RenderPassQueue queue) const {
  return std::find_if(passes_.begin(), passes_.end(), [&](const RenderPassDescriptor& descriptor) {
           return descriptor.queue == queue;
         }) != passes_.end();
}

std::vector<size_t> RenderGraph::GetPassIndices(const RenderPassQueue queue) const {
  std::vector<size_t> pass_indices;
  for (size_t pass_index = 0; pass_index < passes_.size(); pass_index++) {
    if (passes_[pass_index].queue == queue) {
      pass_indices.emplace_back(pass_index);
    }
  }
  return pass_indices;
}

const std::vector<RenderResourceDescriptor>& RenderGraph::GetResources() const {
  return resources_;
}

const std::vector<RenderPassDescriptor>& RenderGraph::GetPasses() const {
  return passes_;
}

RenderGraphPlanCache::RenderGraphPlanCache(const size_t capacity) : capacity_(std::max<size_t>(capacity, 1)) {
  entries_.reserve(capacity_);
  stats_.capacity = capacity_;
}

const RenderGraphExecutionPlan& RenderGraphPlanCache::GetOrCompile(const RenderGraph& graph,
                                                                   const RenderGraphCompileContext& context) {
  const auto& resources = graph.GetResources();
  const auto& passes = graph.GetPasses();
  const auto same_topology = [&](const Entry& entry) {
    return SameCompileContext(entry.context, context) && entry.resources.size() == resources.size() &&
           entry.passes.size() == passes.size() &&
           std::equal(entry.resources.begin(), entry.resources.end(), resources.begin(), SameResource) &&
           std::equal(entry.passes.begin(), entry.passes.end(), passes.begin(), SamePass);
  };
  if (const auto found = std::find_if(entries_.begin(), entries_.end(), same_topology); found != entries_.end()) {
    found->last_use = ++use_counter_;
    ++stats_.hit_count;
    return found->plan;
  }

  ++stats_.miss_count;
  const auto compilation_start = std::chrono::steady_clock::now();
  auto plan = graph.Compile(context);
  stats_.compilation_milliseconds +=
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - compilation_start).count();
  ++stats_.compilation_count;
  if (entries_.size() == capacity_) {
    const auto oldest = std::min_element(entries_.begin(), entries_.end(), [](const Entry& left, const Entry& right) {
      return left.last_use < right.last_use;
    });
    entries_.erase(oldest);
    ++stats_.eviction_count;
  }
  entries_.push_back({context, resources, passes, std::move(plan), ++use_counter_});
  stats_.entry_count = entries_.size();
  return entries_.back().plan;
}

void RenderGraphPlanCache::Clear() {
  entries_.clear();
  use_counter_ = 0;
  stats_ = {};
  stats_.capacity = capacity_;
}

RenderGraphPlanCacheStats RenderGraphPlanCache::GetStats() const {
  auto stats = stats_;
  stats.entry_count = entries_.size();
  return stats;
}

void evo_engine::AddDefaultFrameResources(RenderGraph& graph) {
  graph.AddResource(
      {RenderResourceNames::frame_render_instances, RenderResourceType::Buffer, RenderResourceLifetime::Frame});
  graph.AddResource({RenderResourceNames::frame_per_frame_descriptor_set, RenderResourceType::DescriptorSet,
                     RenderResourceLifetime::Frame});
}

void evo_engine::AddDefaultRasterCameraResources(RenderGraph& graph) {
  AddDefaultFrameResources(graph);
  graph.AddResource(
      {RenderResourceNames::lighting_directional_shadow_map, RenderResourceType::Image, RenderResourceLifetime::Frame});
  graph.AddResource({RenderResourceNames::camera_depth,
                     RenderResourceType::Image,
                     RenderResourceLifetime::Camera,
                     {RenderResourceSizeMode::CameraRelative},
                     "Depth"});
  graph.AddResource({RenderResourceNames::camera_g_buffer,
                     RenderResourceType::Image,
                     RenderResourceLifetime::Camera,
                     {RenderResourceSizeMode::CameraRelative},
                     "GBuffer"});
  graph.AddResource({RenderResourceNames::camera_color,
                     RenderResourceType::Image,
                     RenderResourceLifetime::Camera,
                     {RenderResourceSizeMode::CameraRelative},
                     "Color"});
}

void evo_engine::AddDefaultRayTracingCameraResources(RenderGraph& graph) {
  AddDefaultFrameResources(graph);
  graph.AddResource({RenderResourceNames::frame_ray_tracing_descriptor_set, RenderResourceType::DescriptorSet,
                     RenderResourceLifetime::Frame});
  graph.AddResource(
      {RenderResourceNames::scene_mesh_tlas, RenderResourceType::AccelerationStructure, RenderResourceLifetime::Frame});
  graph.AddResource({RenderResourceNames::camera_color,
                     RenderResourceType::Image,
                     RenderResourceLifetime::Camera,
                     {RenderResourceSizeMode::CameraRelative},
                     "Color"});
  graph.AddResource({RenderResourceNames::camera_ray_hit_distance,
                     RenderResourceType::Image,
                     RenderResourceLifetime::Camera,
                     {RenderResourceSizeMode::CameraRelative},
                     "R32F",
                     1,
                     1,
                     true});
}

namespace {
void AddRayCameraOptionalImage(RenderGraph& graph, const char* name, const char* format_name) {
  graph.AddResource({name,
                     RenderResourceType::Image,
                     RenderResourceLifetime::Camera,
                     {RenderResourceSizeMode::CameraRelative},
                     format_name,
                     1,
                     1,
                     true});
}
}  // namespace

void evo_engine::AddRayCameraOptionalOutputResources(RenderGraph& graph,
                                                     const CameraSettings::RayOutputSettings& outputs) {
  if (outputs.albedo) {
    AddRayCameraOptionalImage(graph, RenderResourceNames::camera_ray_albedo, "RGBA8");
  }
  if (outputs.normal) {
    AddRayCameraOptionalImage(graph, RenderResourceNames::camera_ray_normal, "RGBA16F");
  }
  if (outputs.ray_count) {
    AddRayCameraOptionalImage(graph, RenderResourceNames::camera_ray_count, "R32U");
  }
  if (outputs.path_length) {
    AddRayCameraOptionalImage(graph, RenderResourceNames::camera_ray_path_length, "R32U");
  }
  if (outputs.time) {
    AddRayCameraOptionalImage(graph, RenderResourceNames::camera_ray_time, "R32U");
  }
  if (outputs.debug) {
    AddRayCameraOptionalImage(graph, RenderResourceNames::camera_ray_debug, "RGBA32F");
  }
}

void evo_engine::AddAdvancedFrameResources(RenderGraph& graph) {
  graph.AddResource({RenderResourceNames::frame_visibility_buffer,
                     RenderResourceType::Buffer,
                     RenderResourceLifetime::Frame,
                     {},
                     "Visibility",
                     1,
                     1,
                     true});
}

void evo_engine::AddAdvancedCameraResources(RenderGraph& graph) {
  graph.AddResource({RenderResourceNames::camera_ambient_occlusion,
                     RenderResourceType::Image,
                     RenderResourceLifetime::Camera,
                     {RenderResourceSizeMode::CameraRelative},
                     "R16F",
                     1,
                     1,
                     true});
  graph.AddResource({RenderResourceNames::camera_ambient_occlusion_scratch,
                     RenderResourceType::Image,
                     RenderResourceLifetime::Camera,
                     {RenderResourceSizeMode::CameraRelative},
                     "R16F",
                     1,
                     1,
                     true});
  graph.AddResource({RenderResourceNames::camera_motion_vectors,
                     RenderResourceType::Image,
                     RenderResourceLifetime::Camera,
                     {RenderResourceSizeMode::CameraRelative},
                     "RGBA16F",
                     1,
                     1,
                     true});
  graph.AddResource({RenderResourceNames::camera_object_id,
                     RenderResourceType::Image,
                     RenderResourceLifetime::Camera,
                     {RenderResourceSizeMode::CameraRelative},
                     "R32U",
                     1,
                     1,
                     true});
  graph.AddResource({RenderResourceNames::camera_material_id,
                     RenderResourceType::Image,
                     RenderResourceLifetime::Camera,
                     {RenderResourceSizeMode::CameraRelative},
                     "R32U",
                     1,
                     1,
                     true});
  graph.AddResource({RenderResourceNames::camera_depth_pyramid,
                     RenderResourceType::Image,
                     RenderResourceLifetime::Camera,
                     {RenderResourceSizeMode::CameraRelative, 0, 0, 1, 1, 0},
                     "DepthPyramid",
                     1,
                     1,
                     true});
  graph.AddResource({RenderResourceNames::camera_color_history,
                     RenderResourceType::Image,
                     RenderResourceLifetime::History,
                     {RenderResourceSizeMode::CameraRelative},
                     "Color",
                     1,
                     2,
                     true});
  graph.AddResource({RenderResourceNames::camera_radiance_history,
                     RenderResourceType::Image,
                     RenderResourceLifetime::History,
                     {RenderResourceSizeMode::CameraRelative},
                     "Radiance",
                     1,
                     2,
                     true});
}

void evo_engine::AddVolumetricCloudCameraResources(RenderGraph& graph, const uint32_t resolution_divisor) {
  const uint32_t safe_resolution_divisor = resolution_divisor <= 1u ? 1u : (resolution_divisor <= 2u ? 2u : 4u);
  graph.AddResource({RenderResourceNames::camera_volumetric_cloud_accumulation,
                     RenderResourceType::Image,
                     RenderResourceLifetime::Camera,
                     {RenderResourceSizeMode::CameraRelative, safe_resolution_divisor, safe_resolution_divisor},
                     "RGBA16F",
                     1,
                     1,
                     true});
  graph.AddResource({RenderResourceNames::camera_volumetric_cloud_transmittance,
                     RenderResourceType::Image,
                     RenderResourceLifetime::Camera,
                     {RenderResourceSizeMode::CameraRelative, safe_resolution_divisor, safe_resolution_divisor},
                     "R16F",
                     1,
                     1,
                     true});
}

void evo_engine::AddGaussianSplatCameraResources(RenderGraph& graph) {
  graph.AddResource({RenderResourceNames::camera_gaussian_splat_prepass,
                     RenderResourceType::Buffer,
                     RenderResourceLifetime::Camera,
                     {},
                     "GaussianSplatPrepass",
                     1,
                     1,
                     true,
                     sizeof(uint32_t)});
}
