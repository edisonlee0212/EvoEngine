#include "Platform.hpp"
#include "Application.hpp"
#include "Console.hpp"
#include "EditorLayer.hpp"
#include "GeometryStorage.hpp"
#include "Mesh.hpp"
#include "RenderLayer.hpp"
#include "Resources.hpp"
#include "TextureStorage.hpp"
#include "Times.hpp"
#include "Utilities.hpp"
#include "WindowLayer.hpp"
#include "vk_mem_alloc.h"

using namespace evo_engine;
void Platform::Initialize() {
  auto& graphics = GetInstance();
#pragma region volk
  if (volkInitialize() != VK_SUCCESS) {
    throw std::runtime_error("Volk failed to initialize!");
  }
#pragma endregion
#pragma region Vulkan
  graphics.CreateInstance();
  graphics.CreateSurface();
  graphics.CreateDebugMessenger();
  graphics.SelectPhysicalDevice();
#if USE_NSIGHT_AFTERMATH
  graphics.gpu_crash_tracker.Initialize();
#endif
  graphics.CreateLogicalDevice();
  graphics.SetupVmaAllocator();

  const auto& selected_physical_device = graphics.selected_physical_device;
  if (selected_physical_device->queue_family_indices.present_family.has_value()) {
    graphics.CreateSwapChain();
    graphics.vk_surface_format_ = selected_physical_device->swap_chain_support_details.formats[0];
    for (const auto& available_format : selected_physical_device->swap_chain_support_details.formats) {
      if (available_format.format == VK_FORMAT_B8G8R8A8_SRGB &&
          available_format.colorSpace == VK_COLOR_SPACE_SRGB_NONLINEAR_KHR) {
        graphics.vk_surface_format_ = available_format;
        break;
      }
    }
  }

  if (graphics.selected_physical_device->queue_family_indices.graphics_and_compute_family.has_value()) {
#pragma region Command pool
    VkCommandPoolCreateInfo pool_info{};
    pool_info.sType = VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO;
    pool_info.flags = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;
    pool_info.queueFamilyIndex =
        graphics.selected_physical_device->queue_family_indices.graphics_and_compute_family.value();
    graphics.command_pool_ = std::make_unique<CommandPool>(pool_info);
#pragma endregion
    graphics.used_command_buffer_size_ = 0;
    graphics.command_buffer_pool_.resize(graphics.max_frame_in_flight_);
    graphics.CreateSwapChainSyncObjects();

    constexpr VkDescriptorPoolSize render_layer_descriptor_pool_sizes[] = {
        {VK_DESCRIPTOR_TYPE_SAMPLER, Constants::initial_descriptor_pool_max_size},
        {VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, Constants::initial_descriptor_pool_max_size},
        {VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE, Constants::initial_descriptor_pool_max_size},
        {VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, Constants::initial_descriptor_pool_max_size},
        {VK_DESCRIPTOR_TYPE_UNIFORM_TEXEL_BUFFER, Constants::initial_descriptor_pool_max_size},
        {VK_DESCRIPTOR_TYPE_STORAGE_TEXEL_BUFFER, Constants::initial_descriptor_pool_max_size},
        {VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, Constants::initial_descriptor_pool_max_size},
        {VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, Constants::initial_descriptor_pool_max_size},
        {VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER_DYNAMIC, Constants::initial_descriptor_pool_max_size},
        {VK_DESCRIPTOR_TYPE_STORAGE_BUFFER_DYNAMIC, Constants::initial_descriptor_pool_max_size},
        {VK_DESCRIPTOR_TYPE_INPUT_ATTACHMENT, Constants::initial_descriptor_pool_max_size}};

    VkDescriptorPoolCreateInfo render_layer_descriptor_pool_info{};
    render_layer_descriptor_pool_info.flags = VK_DESCRIPTOR_POOL_CREATE_FREE_DESCRIPTOR_SET_BIT;
    render_layer_descriptor_pool_info.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO;
    render_layer_descriptor_pool_info.poolSizeCount = std::size(render_layer_descriptor_pool_sizes);
    render_layer_descriptor_pool_info.pPoolSizes = render_layer_descriptor_pool_sizes;
    render_layer_descriptor_pool_info.maxSets = Constants::initial_descriptor_pool_max_sets;
    graphics.descriptor_pool_ = std::make_unique<DescriptorPool>(render_layer_descriptor_pool_info);
  }

  graphics.immediate_submit_command_buffer = std::make_shared<CommandBuffer>();

  std::vector<Vertex> vertices = {{{1.0f, 1.0f, 0.0f}}, {{-1.0f, 1.0f, 0.0f}}, {{0.0f, -1.0f, 0.0f}}};
  std::vector<glm::uvec3> indices = {glm::uvec3(0, 1, 2)};
  const auto blas = std::make_shared<BottomLevelAccelerationStructure>(vertices, indices);

#pragma endregion
  const auto& window_layer = Application::GetLayer<WindowLayer>();
  if (const auto& editor_layer = Application::GetLayer<EditorLayer>(); window_layer && editor_layer) {
    // Setup Dear ImGui context
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImNodes::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;
    // io.ConfigFlags |= ImGuiConfigFlags_ViewportsEnable;
    //  io.ConfigFlags |= ImGuiConfigFlags_DpiEnableScaleViewports;
    io.ConfigFlags |= ImGuiConfigFlags_DpiEnableScaleFonts;
    // io.ConfigFlags |= ImGuiConfigFlags_IsSRGB;
    ImGui::StyleColorsDark();

    // When viewports are enabled we tweak WindowRounding/WindowBg so platform windows can look identical to regular
    // ones.
    ImGuiStyle& style = ImGui::GetStyle();
    if (io.ConfigFlags & ImGuiConfigFlags_ViewportsEnable) {
      style.WindowRounding = 0.0f;
      style.Colors[ImGuiCol_WindowBg].w = 1.0f;
    }

    ImGui_ImplGlfw_InitForVulkan(window_layer->GetGlfwWindow(), true);
    ImGui_ImplVulkan_InitInfo init_info = {};
    init_info.Instance = graphics.vk_instance_;
    init_info.PhysicalDevice = selected_physical_device->vk_physical_device;
    init_info.Device = graphics.vk_device_;
    init_info.QueueFamily = graphics.selected_physical_device->queue_family_indices.graphics_and_compute_family.value();
    init_info.Queue = graphics.main_queue_->vk_queue_;
    init_info.PipelineCache = VK_NULL_HANDLE;
    init_info.DescriptorPool = graphics.descriptor_pool_->GetVkDescriptorPool();
    init_info.MinImageCount = graphics.swapchain_->GetAllImageViews().size();
    init_info.ImageCount = graphics.swapchain_->GetAllImageViews().size();
    init_info.MSAASamples = VK_SAMPLE_COUNT_1_BIT;
    init_info.UseDynamicRendering = true;
    init_info.PipelineRenderingCreateInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_RENDERING_CREATE_INFO_KHR;
    init_info.PipelineRenderingCreateInfo.colorAttachmentCount = 1;
    const auto format = graphics.swapchain_->GetImageFormat();
    init_info.PipelineRenderingCreateInfo.pColorAttachmentFormats = &format;
    init_info.PipelineRenderingCreateInfo.pNext = nullptr;
    // init_info.ColorAttachmentFormat = graphics.swapchain_->GetImageFormat();

    ImGui_ImplVulkan_LoadFunctions([](const char* function_name, void*) {
      return vkGetInstanceProcAddr(GetVkInstance(), function_name);
    });
    ImGui_ImplVulkan_Init(&init_info);
  }

  GeometryStorage::Initialize();
  TextureStorage::Initialize();
  graphics.draw_call.resize(graphics.max_frame_in_flight_);
  graphics.triangles.resize(graphics.max_frame_in_flight_);
  graphics.strands_segments.resize(graphics.max_frame_in_flight_);

  const uint32_t mesh_work_group_invocations =
      selected_physical_device->mesh_shader_properties_ext.maxPreferredMeshWorkGroupInvocations;
  Constants::task_work_group_invocations =
      selected_physical_device->mesh_shader_properties_ext.maxPreferredTaskWorkGroupInvocations;

  const uint32_t mesh_subgroup_size = selected_physical_device->vulkan11_properties.subgroupSize;
  const uint32_t mesh_subgroup_count =
      (std::min(std::max(Constants::meshlet_max_vertices_size, Constants::meshlet_max_triangles_size),
                mesh_work_group_invocations) +
       mesh_subgroup_size - 1) /
      mesh_subgroup_size;
  const uint32_t task_subgroup_size = selected_physical_device->vulkan11_properties.subgroupSize;
  const uint32_t task_subgroup_count =
      (Constants::task_work_group_invocations + task_subgroup_size - 1) / task_subgroup_size;

  Constants::task_subgroup_size = glm::max(task_subgroup_size, 1u);
  Constants::mesh_subgroup_size = glm::max(mesh_subgroup_size, 1u);
  Constants::task_subgroup_count = glm::max(task_subgroup_count, 1u);
  Constants::mesh_subgroup_count = glm::max(mesh_subgroup_count, 1u);
  Constants::shader_global_defines =
      "\n#define MAX_DIRECTIONAL_LIGHT_SIZE " + std::to_string(Settings::max_directional_light_size) +
      "\n#define MAX_KERNEL_AMOUNT " + std::to_string(Constants::max_kernel_amount) +
      "\n#define MESHLET_MAX_VERTICES_SIZE " + std::to_string(Constants::meshlet_max_vertices_size) +
      "\n#define MESHLET_MAX_TRIANGLES_SIZE " + std::to_string(Constants::meshlet_max_triangles_size) +
      "\n#define MESHLET_MAX_INDICES_SIZE " + std::to_string(Constants::meshlet_max_triangles_size * 3)

      + "\n#define EXT_TASK_SUBGROUP_SIZE " + std::to_string(task_subgroup_size) + "\n#define EXT_MESH_SUBGROUP_SIZE " +
      std::to_string(mesh_subgroup_size) + "\n#define EXT_TASK_SUBGROUP_COUNT " + std::to_string(task_subgroup_count) +
      "\n#define EXT_MESH_SUBGROUP_COUNT " + std::to_string(mesh_subgroup_count)

      + "\n#define EXT_INVOCATIONS_PER_TASK " + std::to_string(Constants::task_work_group_invocations) + "\n";
}

VkBool32 DebugCallback(const VkDebugUtilsMessageSeverityFlagBitsEXT message_severity,
                       const VkDebugUtilsMessageTypeFlagsEXT message_type,
                       const VkDebugUtilsMessengerCallbackDataEXT* p_callback_data, void* p_user_data) {
  if (message_severity < VK_DEBUG_UTILS_MESSAGE_SEVERITY_WARNING_BIT_EXT)
    return VK_FALSE;
  std::string msg = "Vulkan";
  switch (message_type) {
    case VK_DEBUG_UTILS_MESSAGE_TYPE_GENERAL_BIT_EXT: {
      msg += " [General]";
    } break;
    case VK_DEBUG_UTILS_MESSAGE_TYPE_VALIDATION_BIT_EXT: {
      msg += " [Validation]";
    } break;
    case VK_DEBUG_UTILS_MESSAGE_TYPE_PERFORMANCE_BIT_EXT: {
      msg += " [Performance]";
    } break;
    default:
      break;
  }
  switch (message_severity) {
    case VK_DEBUG_UTILS_MESSAGE_SEVERITY_VERBOSE_BIT_EXT: {
      msg += "-[Diagnostic]: ";
    } break;
    case VK_DEBUG_UTILS_MESSAGE_SEVERITY_INFO_BIT_EXT: {
      msg += "-[Info]: ";
    } break;
    case VK_DEBUG_UTILS_MESSAGE_SEVERITY_WARNING_BIT_EXT: {
      msg += "-[Warning]: ";
    } break;
    case VK_DEBUG_UTILS_MESSAGE_SEVERITY_ERROR_BIT_EXT: {
      msg += "-[Error]: ";
    } break;
    case VK_DEBUG_UTILS_MESSAGE_SEVERITY_FLAG_BITS_MAX_ENUM_EXT:
      break;
  }
  msg += std::string(p_callback_data->pMessage);

  EVOENGINE_LOG(msg);
  return VK_FALSE;
}

void SelectStageFlagsAccessMask(const VkImageLayout image_layout, VkAccessFlags& mask,
                                VkPipelineStageFlags& stage_flags) {
  switch (image_layout) {
    case VK_IMAGE_LAYOUT_UNDEFINED: {
      mask = 0;
      stage_flags = VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT;
    } break;
    case VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL: {
      mask = VK_ACCESS_TRANSFER_WRITE_BIT;
      stage_flags = VK_PIPELINE_STAGE_TRANSFER_BIT;
    } break;
    case VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL: {
      mask = VK_ACCESS_SHADER_READ_BIT;
      stage_flags = VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT;
    } break;
    case VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL: {
      mask = VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_READ_BIT | VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT;
      stage_flags = VK_PIPELINE_STAGE_EARLY_FRAGMENT_TESTS_BIT;
    } break;
    case VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL: {
      mask = VK_ACCESS_COLOR_ATTACHMENT_READ_BIT | VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;
      stage_flags = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
    } break;
    case VK_IMAGE_LAYOUT_ATTACHMENT_OPTIMAL: {
      mask = VK_ACCESS_COLOR_ATTACHMENT_READ_BIT | VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;
      stage_flags = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
    } break;
    case VK_IMAGE_LAYOUT_PRESENT_SRC_KHR: {
      mask = 0;
      stage_flags = VK_PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT;
    } break;
    default: {
      mask = 0;
      stage_flags = VK_PIPELINE_STAGE_ALL_GRAPHICS_BIT;
    } break;
  }
}

void Platform::RecordCommandsMainQueue(const std::function<void(VkCommandBuffer vk_command_buffer)>& action) {
  auto& graphics = GetInstance();
  const unsigned vk_command_buffer_index = graphics.used_command_buffer_size_;
  const auto current_frame_index = graphics.current_frame_index_;
  if (vk_command_buffer_index >= graphics.command_buffer_pool_[current_frame_index].size()) {
    graphics.command_buffer_pool_[current_frame_index].emplace_back(std::make_shared<CommandBuffer>());
  }
  const auto& vk_command_buffer = graphics.command_buffer_pool_[current_frame_index][vk_command_buffer_index];
  vk_command_buffer->Record(action);
  graphics.used_command_buffer_size_++;
}

void Platform::WaitForDeviceIdle() {
  const auto& graphics = GetInstance();
  vkDeviceWaitIdle(graphics.vk_device_);
}

void Platform::RegisterGraphicsPipeline(const std::string& name,
                                        const std::shared_ptr<GraphicsPipeline>& graphics_pipeline) {
  auto& graphics = GetInstance();
  if (graphics.graphics_pipelines_.find(name) != graphics.graphics_pipelines_.end()) {
    EVOENGINE_ERROR("GraphicsPipeline with same name exists!");
    return;
  }
  graphics.graphics_pipelines_[name] = graphics_pipeline;
}

void Platform::RegisterComputePipeline(const std::string& name,
                                       const std::shared_ptr<ComputePipeline>& compute_pipeline) {
  auto& graphics = GetInstance();
  if (graphics.compute_pipelines_.find(name) != graphics.compute_pipelines_.end()) {
    EVOENGINE_ERROR("ComputePipeline with same name exists!");
    return;
  }
  graphics.compute_pipelines_[name] = compute_pipeline;
}

void Platform::RegisterRayTracingPipeline(const std::string& name,
                                          const std::shared_ptr<RayTracingPipeline>& ray_tracing_pipeline) {
  auto& graphics = GetInstance();
  if (graphics.ray_tracing_pipelines_.find(name) != graphics.ray_tracing_pipelines_.end()) {
    EVOENGINE_ERROR("RayTracingPipeline with same name exists!");
    return;
  }
  graphics.ray_tracing_pipelines_[name] = ray_tracing_pipeline;
}

const std::shared_ptr<GraphicsPipeline>& Platform::GetGraphicsPipeline(const std::string& name) {
  const auto& graphics = GetInstance();
  return graphics.graphics_pipelines_.at(name);
}

const std::shared_ptr<ComputePipeline>& Platform::GetComputePipeline(const std::string& name) {
  const auto& graphics = GetInstance();
  return graphics.compute_pipelines_.at(name);
}

const std::shared_ptr<RayTracingPipeline>& Platform::GetRayTracingPipeline(const std::string& name) {
  const auto& graphics = GetInstance();
  return graphics.ray_tracing_pipelines_.at(name);
}

const std::shared_ptr<DescriptorSetLayout>& Platform::GetDescriptorSetLayout(const std::string& name) {
  const auto& graphics = GetInstance();
  return graphics.descriptor_set_layouts_.at(name);
}

void Platform::RegisterDescriptorSetLayout(const std::string& name,
                                           const std::shared_ptr<DescriptorSetLayout>& descriptor_set_layout) {
  auto& graphics = GetInstance();
  if (graphics.descriptor_set_layouts_.find(name) != graphics.descriptor_set_layouts_.end()) {
    EVOENGINE_ERROR("GraphicsPipeline with same name exists!");
    return;
  }
  graphics.descriptor_set_layouts_[name] = descriptor_set_layout;
}

void Platform::TransitImageLayout(VkCommandBuffer vk_command_buffer, const VkImage target_image,
                                  const VkFormat image_format, const uint32_t layer_count,
                                  const VkImageLayout old_layout, const VkImageLayout new_layout,
                                  const uint32_t mip_levels) {
  VkImageMemoryBarrier barrier{};
  barrier.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER;
  barrier.oldLayout = old_layout;
  barrier.newLayout = new_layout;
  barrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
  barrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
  barrier.image = target_image;
  if (image_format == Constants::texture_2d || image_format == Constants::render_texture_color ||
      image_format == Constants::g_buffer_color) {
    barrier.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
  } else if (image_format == Constants::render_texture_depth || image_format == Constants::g_buffer_depth ||
             image_format == Constants::shadow_map) {
    barrier.subresourceRange.aspectMask = VK_IMAGE_ASPECT_DEPTH_BIT;
  } else if (const auto window_layer = Application::GetLayer<WindowLayer>();
             window_layer && image_format == GetSwapchain()->GetImageFormat()) {
    barrier.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
  } else {
    barrier.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
  }

  barrier.subresourceRange.baseMipLevel = 0;
  barrier.subresourceRange.levelCount = mip_levels;
  barrier.subresourceRange.baseArrayLayer = 0;
  barrier.subresourceRange.layerCount = layer_count;

  VkPipelineStageFlags source_stage;
  VkPipelineStageFlags destination_stage;

  SelectStageFlagsAccessMask(old_layout, barrier.srcAccessMask, source_stage);
  SelectStageFlagsAccessMask(new_layout, barrier.dstAccessMask, destination_stage);

  vkCmdPipelineBarrier(vk_command_buffer, source_stage, destination_stage, 0, 0, nullptr, 0, nullptr, 1, &barrier);
}

size_t Platform::GetMaxBoneAmount() {
  const auto& graphics = GetInstance();
  return graphics.max_bone_amount_;
}

size_t Platform::GetMaxShadowCascadeAmount() {
  const auto& graphics = GetInstance();
  return graphics.max_shadow_cascade_amount_;
}

void Platform::ImmediateSubmit(const std::function<void(VkCommandBuffer vk_command_buffer)>& action) {
  const auto& graphics = GetInstance();
  graphics.immediate_submit_command_buffer->Record(action);

  VkSubmitInfo submit_info{};
  submit_info.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
  submit_info.commandBufferCount = 1;
  submit_info.pCommandBuffers = &graphics.immediate_submit_command_buffer->GetVkCommandBuffer();

  VkFenceCreateInfo fence_info{};
  fence_info.sType = VK_STRUCTURE_TYPE_FENCE_CREATE_INFO;
  fence_info.flags = 0;

  VkFence fence;
  CheckVk(vkCreateFence(graphics.vk_device_, &fence_info, nullptr, &fence));

  if (vkQueueSubmit(graphics.immediate_submit_queue_->vk_queue_, 1, &submit_info, fence) != VK_SUCCESS) {
    throw std::runtime_error("Failed to submit command buffer to graphics queue!");
  }
  CheckVk(vkWaitForFences(graphics.vk_device_, 1, &fence, VK_TRUE, UINT64_MAX));
  vkDestroyFence(graphics.vk_device_, fence, nullptr);

  graphics.immediate_submit_command_buffer->Reset();
}

int Platform::GetMaxFramesInFlight() {
  const auto& graphics = GetInstance();
  return graphics.max_frame_in_flight_;
}

void Platform::NotifyRecreateSwapChain() {
  auto& graphics = GetInstance();
  graphics.recreate_swap_chain_ = true;
}

VkInstance Platform::GetVkInstance() {
  const auto& graphics = GetInstance();
  return graphics.vk_instance_;
}

const std::shared_ptr<Platform::PhysicalDevice>& Platform::GetSelectedPhysicalDevice() {
  const auto& graphics = GetInstance();
  return graphics.selected_physical_device;
}

VkDevice Platform::GetVkDevice() {
  const auto& graphics = GetInstance();
  return graphics.vk_device_;
}

uint32_t Platform::GetCurrentFrameIndex() {
  const auto& graphics = GetInstance();
  return graphics.current_frame_index_;
}

uint32_t Platform::GetNextImageIndex() {
  const auto& graphics = GetInstance();
  return graphics.next_image_index_;
}

VkCommandPool Platform::GetVkCommandPool() {
  const auto& graphics = GetInstance();
  return graphics.command_pool_->GetVkCommandPool();
}

const std::unique_ptr<CommandQueue>& Platform::GetMainQueue() {
  const auto& graphics = GetInstance();
  return graphics.main_queue_;
}

const std::unique_ptr<CommandQueue>& Platform::GetImmediateSubmitQueue() {
  const auto& graphics = GetInstance();
  return graphics.immediate_submit_queue_;
}

const std::unique_ptr<CommandQueue>& Platform::GetPresentQueue() {
  const auto& graphics = GetInstance();
  return graphics.present_queue_;
}

VmaAllocator Platform::GetVmaAllocator() {
  const auto& graphics = GetInstance();
  return graphics.vma_allocator_;
}

const std::shared_ptr<Swapchain>& Platform::GetSwapchain() {
  const auto& graphics = GetInstance();
  return graphics.swapchain_;
}

const std::unique_ptr<DescriptorPool>& Platform::GetDescriptorPool() {
  const auto& graphics = GetInstance();
  return graphics.descriptor_pool_;
}

unsigned Platform::GetSwapchainVersion() {
  const auto& graphics = GetInstance();
  return graphics.swapchain_version_;
}

VkSurfaceFormatKHR Platform::GetVkSurfaceFormat() {
  const auto& graphics = GetInstance();
  return graphics.vk_surface_format_;
}

VkResult CreateDebugUtilsMessengerExt(VkInstance instance, const VkDebugUtilsMessengerCreateInfoEXT* p_create_info,
                                      const VkAllocationCallbacks* p_allocator,
                                      VkDebugUtilsMessengerEXT* p_debug_messenger) {
  if (const auto func = reinterpret_cast<PFN_vkCreateDebugUtilsMessengerEXT>(
          vkGetInstanceProcAddr(instance, "vkCreateDebugUtilsMessengerEXT"));
      func != nullptr) {
    return func(instance, p_create_info, p_allocator, p_debug_messenger);
  } else {
    return VK_ERROR_EXTENSION_NOT_PRESENT;
  }
}

void DestroyDebugUtilsMessengerExt(const VkInstance instance, const VkDebugUtilsMessengerEXT debug_messenger,
                                   const VkAllocationCallbacks* p_allocator) {
  if (const auto func = reinterpret_cast<PFN_vkDestroyDebugUtilsMessengerEXT>(
          vkGetInstanceProcAddr(instance, "vkDestroyDebugUtilsMessengerEXT"));
      func != nullptr) {
    func(instance, debug_messenger, p_allocator);
  }
}

void PopulateDebugMessengerCreateInfo(VkDebugUtilsMessengerCreateInfoEXT& debug_utils_messenger_create_info) {
  debug_utils_messenger_create_info = {};
  debug_utils_messenger_create_info.sType = VK_STRUCTURE_TYPE_DEBUG_UTILS_MESSENGER_CREATE_INFO_EXT;
  debug_utils_messenger_create_info.messageSeverity = VK_DEBUG_UTILS_MESSAGE_SEVERITY_VERBOSE_BIT_EXT |
                                                      VK_DEBUG_UTILS_MESSAGE_SEVERITY_WARNING_BIT_EXT |
                                                      VK_DEBUG_UTILS_MESSAGE_SEVERITY_ERROR_BIT_EXT;
  debug_utils_messenger_create_info.messageType = VK_DEBUG_UTILS_MESSAGE_TYPE_GENERAL_BIT_EXT |
                                                  VK_DEBUG_UTILS_MESSAGE_TYPE_VALIDATION_BIT_EXT |
                                                  VK_DEBUG_UTILS_MESSAGE_TYPE_PERFORMANCE_BIT_EXT;
  debug_utils_messenger_create_info.pfnUserCallback = DebugCallback;
  debug_utils_messenger_create_info.pUserData = nullptr;
}

bool Platform::PhysicalDevice::CheckExtensionSupport(const std::string& required_extension_name) const {
  return supported_vk_extensions.find(required_extension_name) != supported_vk_extensions.end();
}

uint32_t Platform::PhysicalDevice::FindMemoryType(uint32_t type_filter, VkMemoryPropertyFlags properties) const {
  for (uint32_t i = 0; i < vk_physical_device_memory_properties.memoryTypeCount; i++) {
    if ((type_filter & (1 << i)) &&
        (vk_physical_device_memory_properties.memoryTypes[i].propertyFlags & properties) == properties) {
      return i;
    }
  }
  throw std::runtime_error("failed to find suitable memory type!");
}

bool Platform::PhysicalDevice::Suitable(const std::vector<std::string>& required_extension_names) const {
  for (const auto& i : required_extension_names) {
    if (!CheckExtensionSupport(i))
      return false;
  }
  if (const auto window_layer = Application::GetLayer<WindowLayer>()) {
    if (!queue_family_indices.present_family.has_value())
      return false;
    if (swap_chain_support_details.formats.empty() || swap_chain_support_details.present_modes.empty())
      return false;
  }
  return true;
}

void Platform::CreateInstance() {
  auto application_info = Application::GetApplicationInfo();
  const auto window_layer = Application::GetLayer<WindowLayer>();
  const auto editor_layer = Application::GetLayer<EditorLayer>();
  if (window_layer) {
#pragma region Windows
    glfwInit();
    glfwWindowHint(GLFW_CLIENT_API, GLFW_NO_API);

    int size;
    const auto monitors = glfwGetMonitors(&size);
    for (auto i = 0; i < size; i++) {
      window_layer->monitors_.push_back(monitors[i]);
    }
    window_layer->primary_monitor_ = glfwGetPrimaryMonitor();
    glfwSetMonitorCallback(window_layer->SetMonitorCallback);

    const auto& application_info = Application::GetApplicationInfo();
    window_layer->window_size_ = application_info.default_window_size;
    if (editor_layer)
      window_layer->window_size_ = {250, 50};
    window_layer->window_ = glfwCreateWindow(window_layer->window_size_.x, window_layer->window_size_.y,
                                             application_info.application_name.c_str(), nullptr, nullptr);

    if (application_info.full_screen)
      glfwMaximizeWindow(window_layer->window_);

    glfwSetFramebufferSizeCallback(window_layer->window_, window_layer->FramebufferSizeCallback);
    glfwSetWindowFocusCallback(window_layer->window_, window_layer->WindowFocusCallback);
    glfwSetKeyCallback(window_layer->window_, Input::KeyCallBack);
    glfwSetMouseButtonCallback(window_layer->window_, Input::MouseButtonCallBack);
  }

  required_layers_ = {"VK_LAYER_KHRONOS_validation"};
  std::vector<const char*> c_required_layers;
  c_required_layers.reserve(required_layers_.size());
  for (const auto& i : required_layers_)
    c_required_layers.emplace_back(i.c_str());

  VkApplicationInfo vk_application_info{};
  vk_application_info.sType = VK_STRUCTURE_TYPE_APPLICATION_INFO;
  vk_application_info.pApplicationName = Application::GetApplicationInfo().application_name.c_str();
  vk_application_info.applicationVersion = VK_MAKE_VERSION(1, 0, 0);
  vk_application_info.pEngineName = "evo_engine";
  vk_application_info.engineVersion = VK_MAKE_VERSION(1, 0, 0);
  vk_application_info.apiVersion = volkGetInstanceVersion();

#pragma region Instance
  VkInstanceCreateInfo instance_create_info{};
  instance_create_info.sType = VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO;
  instance_create_info.pApplicationInfo = &vk_application_info;
  instance_create_info.enabledLayerCount = 0;
  instance_create_info.pNext = nullptr;
#pragma region Extensions
  uint32_t extension_count = 0;
  vkEnumerateInstanceExtensionProperties(nullptr, &extension_count, nullptr);
  std::vector<VkExtensionProperties> supported_extension_list(extension_count);
  vkEnumerateInstanceExtensionProperties(nullptr, &extension_count, supported_extension_list.data());
  for (const auto& i : supported_extension_list) {
    vk_supported_instance_extensions_.insert({i.extensionName, i});
  }

  std::vector<const char*> required_extensions;
  if (window_layer) {
    uint32_t glfw_extension_count = 0;
    const char** glfw_extensions = glfwGetRequiredInstanceExtensions(&glfw_extension_count);

    for (uint32_t i = 0; i < glfw_extension_count; i++) {
      required_extensions.emplace_back(glfw_extensions[i]);
      required_instance_extension_names_.emplace_back(glfw_extensions[i]);
    }
  }
#if GRAPHICS_VALIDATION
  required_extensions.emplace_back(VK_EXT_DEBUG_UTILS_EXTENSION_NAME);
  required_instance_extension_names_.emplace_back(VK_EXT_DEBUG_UTILS_EXTENSION_NAME);
#endif
  instance_create_info.enabledExtensionCount = static_cast<uint32_t>(required_extensions.size());
  instance_create_info.ppEnabledExtensionNames = required_extensions.data();

#pragma endregion
#pragma region Layer
  uint32_t layer_count;
  vkEnumerateInstanceLayerProperties(&layer_count, nullptr);
  vk_supported_layers_.resize(layer_count);
  vkEnumerateInstanceLayerProperties(&layer_count, vk_supported_layers_.data());
#if GRAPHICS_VALIDATION
  if (!CheckLayerSupport("VK_LAYER_KHRONOS_validation")) {
    throw std::runtime_error("Validation layers requested, but not available!");
  }

  instance_create_info.enabledLayerCount = static_cast<uint32_t>(required_layers_.size());
  instance_create_info.ppEnabledLayerNames = c_required_layers.data();

  VkDebugUtilsMessengerCreateInfoEXT debug_create_info{};
  PopulateDebugMessengerCreateInfo(debug_create_info);
  instance_create_info.pNext = &debug_create_info;
#endif

#pragma endregion
  if (vkCreateInstance(&instance_create_info, nullptr, &vk_instance_) != VK_SUCCESS) {
    throw std::runtime_error("Failed to create instance!");
  }

  // Let volk connect with vulkan
  volkLoadInstance(vk_instance_);
#pragma endregion
}

void Platform::CreateSurface() {
  const auto window_layer = Application::GetLayer<WindowLayer>();
#pragma region Surface
  if (window_layer) {
    if (glfwCreateWindowSurface(vk_instance_, window_layer->window_, nullptr, &vk_surface_) != VK_SUCCESS) {
      throw std::runtime_error("failed to create window surface!");
    }
  }
#pragma endregion
}

void Platform::CreateDebugMessenger() {
#pragma region Debug Messenger
#if GRAPHICS_VALIDATION
  VkDebugUtilsMessengerCreateInfoEXT debug_utils_messenger_create_info{};
  PopulateDebugMessengerCreateInfo(debug_utils_messenger_create_info);
  if (CreateDebugUtilsMessengerExt(vk_instance_, &debug_utils_messenger_create_info, nullptr, &vk_debug_messenger_) !=
      VK_SUCCESS) {
    throw std::runtime_error("Failed to set up debug messenger!");
  }
#endif

#pragma endregion
}
int RateDeviceSuitability(const VkPhysicalDevice physical_device) {
  int score = 0;

  VkPhysicalDeviceProperties device_properties;
  VkPhysicalDeviceFeatures device_features;
  vkGetPhysicalDeviceProperties(physical_device, &device_properties);
  vkGetPhysicalDeviceFeatures(physical_device, &device_features);
  // Discrete GPUs have a significant performance advantage
  if (device_properties.deviceType == VK_PHYSICAL_DEVICE_TYPE_DISCRETE_GPU) {
    score += 1000;
  }

  // Maximum possible size of textures affects graphics quality
  score += device_properties.limits.maxImageDimension2D;

  if (!device_features.samplerAnisotropy)
    return 0;

  return score;
}
void Platform::SelectPhysicalDevice() {
  if (const auto window_layer = Application::GetLayer<WindowLayer>()) {
    required_device_extension_names_.emplace_back(VK_KHR_SWAPCHAIN_EXTENSION_NAME);
  }
#if ENABLE_EXTERNAL_MEMORY
#  ifdef _WIN64
  required_device_extension_names_.emplace_back(VK_KHR_EXTERNAL_MEMORY_WIN32_EXTENSION_NAME);
  required_device_extension_names_.emplace_back(VK_KHR_EXTERNAL_SEMAPHORE_WIN32_EXTENSION_NAME);
#  else
  required_device_extension_names_.emplace_back(VK_KHR_EXTERNAL_MEMORY_FD_EXTENSION_NAME);
  required_device_extension_names_.emplace_back(VK_KHR_EXTERNAL_SEMAPHORE_FD_EXTENSION_NAME);
#  endif
#endif
  required_device_extension_names_.emplace_back(VK_KHR_SHADER_DRAW_PARAMETERS_EXTENSION_NAME);
#if USE_NSIGHT_AFTERMATH
  required_device_extension_names_.emplace_back(VK_NV_DEVICE_DIAGNOSTIC_CHECKPOINTS_EXTENSION_NAME);
  required_device_extension_names_.emplace_back(VK_NV_DEVICE_DIAGNOSTICS_CONFIG_EXTENSION_NAME);
#endif
  required_device_extension_names_.emplace_back(VK_EXT_VERTEX_INPUT_DYNAMIC_STATE_EXTENSION_NAME);
  required_device_extension_names_.emplace_back(VK_KHR_FRAGMENT_SHADING_RATE_EXTENSION_NAME);
  required_device_extension_names_.emplace_back(VK_EXT_EXTENDED_DYNAMIC_STATE_EXTENSION_NAME);
  required_device_extension_names_.emplace_back(VK_EXT_EXTENDED_DYNAMIC_STATE_2_EXTENSION_NAME);
  required_device_extension_names_.emplace_back(VK_EXT_EXTENDED_DYNAMIC_STATE_3_EXTENSION_NAME);
#pragma region Physical Device
  uint32_t device_count = 0;
  vkEnumeratePhysicalDevices(vk_instance_, &device_count, nullptr);
  if (device_count == 0) {
    throw std::runtime_error("Failed to find GPUs with Vulkan support!");
  }
#ifndef NDEBUG
  EVOENGINE_LOG("Found " + std::to_string(device_count) + " device(s).");
#endif
  std::vector<VkPhysicalDevice> vk_physical_devices(device_count);
  vkEnumeratePhysicalDevices(vk_instance_, &device_count, vk_physical_devices.data());

  for (const auto& vk_physical_device : vk_physical_devices) {
    physical_devices_.emplace_back(std::make_shared<PhysicalDevice>());
    const auto& physical_device = physical_devices_.back();
    physical_device->vk_physical_device = vk_physical_device;
    physical_device->QueryInformation();

#ifndef NDEBUG
    EVOENGINE_LOG("Found device: " + std::string(physical_device->properties.deviceName) + ".");
#endif

#ifndef NDEBUG
    EVOENGINE_LOG("Device listed as candidate with score " + std::to_string(physical_device->score) + ".");
#endif
  }

  std::map<uint32_t, std::shared_ptr<PhysicalDevice>> candidates;
  for (const auto& i : physical_devices_) {
    if (i->Suitable(required_device_extension_names_)) {
      candidates.insert({i->score, i});
    }
  }
  // Check if the best candidate is suitable at all
  if (!candidates.empty() && candidates.rbegin()->first > 0) {
    selected_physical_device = candidates.rbegin()->second;
#ifndef NDEBUG
    EVOENGINE_LOG("Chose \"" + std::string(selected_physical_device->properties.deviceName) + "\" as physical device.");
#endif
  } else {
    throw std::runtime_error("Failed to find a suitable GPU!");
  }
#pragma endregion

  if (selected_physical_device->CheckExtensionSupport(VK_EXT_MESH_SHADER_EXTENSION_NAME)) {
    required_device_extension_names_.emplace_back(VK_EXT_MESH_SHADER_EXTENSION_NAME);
    Constants::support_mesh_shader = true;
    EVOENGINE_LOG("Target device supports mesh shader!");
  } else {
    Constants::support_mesh_shader = false;
    EVOENGINE_LOG("Target device doesn't support mesh shader!");
  }

  if (selected_physical_device->CheckExtensionSupport(VK_KHR_ACCELERATION_STRUCTURE_EXTENSION_NAME) &&
      selected_physical_device->CheckExtensionSupport(VK_KHR_RAY_TRACING_PIPELINE_EXTENSION_NAME) &&

      selected_physical_device->CheckExtensionSupport(VK_KHR_BUFFER_DEVICE_ADDRESS_EXTENSION_NAME) &&
      selected_physical_device->CheckExtensionSupport(VK_KHR_DEFERRED_HOST_OPERATIONS_EXTENSION_NAME) &&
      selected_physical_device->CheckExtensionSupport(VK_EXT_DESCRIPTOR_INDEXING_EXTENSION_NAME) &&

      selected_physical_device->CheckExtensionSupport(VK_KHR_SPIRV_1_4_EXTENSION_NAME) &&

      selected_physical_device->CheckExtensionSupport(VK_KHR_SHADER_FLOAT_CONTROLS_EXTENSION_NAME)) {
    required_device_extension_names_.emplace_back(VK_KHR_ACCELERATION_STRUCTURE_EXTENSION_NAME);
    required_device_extension_names_.emplace_back(VK_KHR_RAY_TRACING_PIPELINE_EXTENSION_NAME);

    required_device_extension_names_.emplace_back(VK_KHR_BUFFER_DEVICE_ADDRESS_EXTENSION_NAME);
    required_device_extension_names_.emplace_back(VK_KHR_DEFERRED_HOST_OPERATIONS_EXTENSION_NAME);
    required_device_extension_names_.emplace_back(VK_EXT_DESCRIPTOR_INDEXING_EXTENSION_NAME);

    required_device_extension_names_.emplace_back(VK_KHR_SPIRV_1_4_EXTENSION_NAME);

    required_device_extension_names_.emplace_back(VK_KHR_SHADER_FLOAT_CONTROLS_EXTENSION_NAME);
    Constants::support_ray_tracing = true;
    EVOENGINE_LOG("Target device supports ray tracing!");
  } else {
    Constants::support_ray_tracing = false;
    EVOENGINE_LOG("Target device doesn't support ray tracing!");
  }
#if ENABLE_NV_RAY_TRACING_VALIDATION
  if (selected_physical_device->CheckExtensionSupport(VK_NV_RAY_TRACING_VALIDATION_EXTENSION_NAME)) {
    required_device_extension_names_.emplace_back(VK_NV_RAY_TRACING_VALIDATION_EXTENSION_NAME);
    Constants::support_ray_tracing_validation = true;
  } else {
    Constants::support_ray_tracing_validation = false;
  }
#else
  Constants::support_ray_tracing_validation = false;
#endif
}

bool Platform::PhysicalDevice::QueueFamilyIndices::IsComplete() const {
  return graphics_and_compute_family.has_value() && present_family.has_value();
}

void Platform::PhysicalDevice::QueryInformation() {
  uint32_t extension_count = 0;
  supported_vk_extensions.clear();
  vkEnumerateDeviceExtensionProperties(vk_physical_device, nullptr, &extension_count, nullptr);
  std::vector<VkExtensionProperties> supported_extension_list(extension_count);
  vkEnumerateDeviceExtensionProperties(vk_physical_device, nullptr, &extension_count, supported_extension_list.data());
  for (const auto& i : supported_extension_list) {
    supported_vk_extensions.insert({i.extensionName, i});
  }

  vkGetPhysicalDeviceProperties(vk_physical_device, &properties);
  vkGetPhysicalDeviceFeatures(vk_physical_device, &features);
  VkPhysicalDeviceFeatures2 device_features{};
  device_features.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_FEATURES_2;
  device_features.pNext = &acceleration_structure_features;
#if ENABLE_NV_RAY_TRACING_VALIDATION
  acceleration_structure_features.pNext = &ray_tracing_validation_features_nv;
#else
  acceleration_structure_features.pNext = nullptr;
#endif

  vkGetPhysicalDeviceFeatures2(vk_physical_device, &device_features);

  properties2.pNext = &vulkan11_properties;
  vulkan11_properties.pNext = &vulkan12_properties;
  vulkan12_properties.pNext = &mesh_shader_properties_ext;
  mesh_shader_properties_ext.pNext = &subgroup_size_control_properties;
  subgroup_size_control_properties.pNext = &ray_tracing_properties_ext;
  ray_tracing_properties_ext.pNext = nullptr;

  vkGetPhysicalDeviceProperties2(vk_physical_device, &properties2);
  vkGetPhysicalDeviceMemoryProperties(vk_physical_device, &vk_physical_device_memory_properties);

  const auto window_layer = Application::GetLayer<WindowLayer>();

  uint32_t queue_family_count = 0;
  vkGetPhysicalDeviceQueueFamilyProperties(vk_physical_device, &queue_family_count, nullptr);

  std::vector<VkQueueFamilyProperties> queue_families(queue_family_count);
  vkGetPhysicalDeviceQueueFamilyProperties(vk_physical_device, &queue_family_count, queue_families.data());
  const auto& graphics = GetInstance();
  int i = 0;
  for (const auto& queue_family : queue_families) {
    if ((queue_family.queueFlags & VK_QUEUE_GRAPHICS_BIT) && (queue_family.queueFlags & VK_QUEUE_COMPUTE_BIT)) {
      queue_family_indices.graphics_and_compute_family = i;
    }
    VkBool32 present_support = false;
    if (window_layer) {
      vkGetPhysicalDeviceSurfaceSupportKHR(vk_physical_device, i, graphics.vk_surface_, &present_support);
      if (present_support) {
        queue_family_indices.present_family = i;
      }
    }
    if (queue_family_indices.IsComplete()) {
      break;
    }
    i++;
  }

  QuerySwapChainSupport();

  score = 0;
  // Discrete GPUs have a significant performance advantage
  if (properties.deviceType == VK_PHYSICAL_DEVICE_TYPE_DISCRETE_GPU) {
    score += 1000;
  }
  // Maximum possible size of textures affects graphics quality
  score += properties.limits.maxImageDimension2D;
  if (!features.samplerAnisotropy)
    score = 0;
}

void Platform::PhysicalDevice::QuerySwapChainSupport() {
  const auto& graphics = GetInstance();
  vkGetPhysicalDeviceSurfaceCapabilitiesKHR(vk_physical_device, graphics.vk_surface_,
                                            &swap_chain_support_details.capabilities);

  uint32_t format_count;
  vkGetPhysicalDeviceSurfaceFormatsKHR(vk_physical_device, graphics.vk_surface_, &format_count, nullptr);

  if (format_count != 0) {
    swap_chain_support_details.formats.resize(format_count);
    vkGetPhysicalDeviceSurfaceFormatsKHR(vk_physical_device, graphics.vk_surface_, &format_count,
                                         swap_chain_support_details.formats.data());
  }

  uint32_t present_mode_count;
  vkGetPhysicalDeviceSurfacePresentModesKHR(vk_physical_device, graphics.vk_surface_, &present_mode_count, nullptr);

  if (present_mode_count != 0) {
    swap_chain_support_details.present_modes.resize(present_mode_count);
    vkGetPhysicalDeviceSurfacePresentModesKHR(vk_physical_device, graphics.vk_surface_, &present_mode_count,
                                              swap_chain_support_details.present_modes.data());
  }
}

void Platform::CreateLogicalDevice() {
  std::vector<const char*> c_required_device_extensions;
  c_required_device_extensions.reserve(required_device_extension_names_.size());
  for (const auto& i : required_device_extension_names_)
    c_required_device_extensions.emplace_back(i.c_str());
  std::vector<const char*> c_required_layers;
  c_required_layers.reserve(required_layers_.size());
  for (const auto& i : required_layers_)
    c_required_layers.emplace_back(i.c_str());

#pragma region Logical Device
#if USE_NSIGHT_AFTERMATH
  VkDeviceDiagnosticsConfigCreateInfoNV vk_device_diagnostics_config_create_info_nv{};
  vk_device_diagnostics_config_create_info_nv.pNext = nullptr;
  vk_device_diagnostics_config_create_info_nv.sType = VK_STRUCTURE_TYPE_DEVICE_DIAGNOSTICS_CONFIG_CREATE_INFO_NV;
  vk_device_diagnostics_config_create_info_nv.flags = VK_DEVICE_DIAGNOSTICS_CONFIG_ENABLE_RESOURCE_TRACKING_BIT_NV |
                                                      VK_DEVICE_DIAGNOSTICS_CONFIG_ENABLE_AUTOMATIC_CHECKPOINTS_BIT_NV |
                                                      VK_DEVICE_DIAGNOSTICS_CONFIG_ENABLE_SHADER_ERROR_REPORTING_BIT_NV;
#endif

  VkPhysicalDeviceRayTracingPipelineFeaturesKHR vk_physical_device_ray_tracing_pipeline_features_khr{};
  vk_physical_device_ray_tracing_pipeline_features_khr.sType =
      VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_RAY_TRACING_PIPELINE_FEATURES_KHR;
#if USE_NSIGHT_AFTERMATH
  vk_device_diagnostics_config_create_info_nv.pNext = nullptr;
#else
  vk_physical_device_ray_tracing_pipeline_features_khr.pNext = nullptr;
#endif

  vk_physical_device_ray_tracing_pipeline_features_khr.rayTracingPipeline = VK_TRUE;
  vk_physical_device_ray_tracing_pipeline_features_khr.rayTracingPipelineShaderGroupHandleCaptureReplay = VK_TRUE;
  vk_physical_device_ray_tracing_pipeline_features_khr.rayTracingPipelineShaderGroupHandleCaptureReplayMixed = VK_TRUE;
  vk_physical_device_ray_tracing_pipeline_features_khr.rayTracingPipelineTraceRaysIndirect = VK_TRUE;
  vk_physical_device_ray_tracing_pipeline_features_khr.rayTraversalPrimitiveCulling = VK_TRUE;

  VkPhysicalDeviceAccelerationStructureFeaturesKHR vk_physical_device_acceleration_structure_features_khr{};
  vk_physical_device_acceleration_structure_features_khr.sType =
      VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_ACCELERATION_STRUCTURE_FEATURES_KHR;
  vk_physical_device_acceleration_structure_features_khr.pNext = &vk_physical_device_ray_tracing_pipeline_features_khr;
  vk_physical_device_acceleration_structure_features_khr.accelerationStructure = VK_TRUE;
  vk_physical_device_acceleration_structure_features_khr.accelerationStructureCaptureReplay = VK_FALSE;
  vk_physical_device_acceleration_structure_features_khr.accelerationStructureIndirectBuild = VK_FALSE;
  vk_physical_device_acceleration_structure_features_khr.accelerationStructureHostCommands = VK_FALSE;
  vk_physical_device_acceleration_structure_features_khr.descriptorBindingAccelerationStructureUpdateAfterBind =
      VK_FALSE;

  VkPhysicalDeviceVulkan12Features vk_physical_device_vulkan12_features{};
  vk_physical_device_vulkan12_features.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_VULKAN_1_2_FEATURES;
  vk_physical_device_vulkan12_features.shaderInt8 = VK_TRUE;
  vk_physical_device_vulkan12_features.storageBuffer8BitAccess = VK_TRUE;
  vk_physical_device_vulkan12_features.uniformAndStorageBuffer8BitAccess = VK_TRUE;
  vk_physical_device_vulkan12_features.storagePushConstant8 = VK_TRUE;
  vk_physical_device_vulkan12_features.descriptorBindingPartiallyBound = VK_TRUE;
  vk_physical_device_vulkan12_features.runtimeDescriptorArray = VK_TRUE;
  vk_physical_device_vulkan12_features.bufferDeviceAddress = VK_TRUE;

  if (Constants::support_ray_tracing) {
    vk_physical_device_vulkan12_features.pNext = &vk_physical_device_acceleration_structure_features_khr;
  } else {
    vk_physical_device_vulkan12_features.pNext = nullptr;
  }

  VkPhysicalDeviceShaderDrawParametersFeatures shader_draw_parameters_features{};
  shader_draw_parameters_features.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_SHADER_DRAW_PARAMETERS_FEATURES;
  shader_draw_parameters_features.shaderDrawParameters = VK_TRUE;
#if ENABLE_NV_RAY_TRACING_VALIDATION
  VkPhysicalDeviceRayTracingValidationFeaturesNV vk_physical_device_ray_tracing_validation_features_nv = {
      VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_RAY_TRACING_VALIDATION_FEATURES_NV};
  vk_physical_device_ray_tracing_validation_features_nv.rayTracingValidation = VK_TRUE;
  vk_physical_device_ray_tracing_validation_features_nv.pNext = &vk_physical_device_vulkan12_features;
  if (Constants::support_ray_tracing_validation) {
    shader_draw_parameters_features.pNext = &vk_physical_device_ray_tracing_validation_features_nv;
  } else {
    shader_draw_parameters_features.pNext = &vk_physical_device_vulkan12_features;
  }
#else
  shader_draw_parameters_features.pNext = &vk_physical_device_vulkan12_features;
#endif

  VkPhysicalDeviceDynamicRenderingFeaturesKHR dynamic_rendering_features{};
  dynamic_rendering_features.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_DYNAMIC_RENDERING_FEATURES_KHR;
  dynamic_rendering_features.dynamicRendering = VK_TRUE;
  dynamic_rendering_features.pNext = &shader_draw_parameters_features;

  VkPhysicalDeviceFragmentShadingRateFeaturesKHR physical_device_fragment_shading_rate_features{};
  physical_device_fragment_shading_rate_features.sType =
      VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_FRAGMENT_SHADING_RATE_FEATURES_KHR;
  physical_device_fragment_shading_rate_features.pNext = &dynamic_rendering_features;
  physical_device_fragment_shading_rate_features.attachmentFragmentShadingRate = VK_FALSE;
  physical_device_fragment_shading_rate_features.pipelineFragmentShadingRate = VK_FALSE;
  physical_device_fragment_shading_rate_features.primitiveFragmentShadingRate = VK_FALSE;

  VkPhysicalDeviceMultiviewFeatures physical_device_multiview_features{};
  physical_device_multiview_features.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_MULTIVIEW_FEATURES;
  physical_device_multiview_features.pNext = &physical_device_fragment_shading_rate_features;
  physical_device_multiview_features.multiview = VK_FALSE;
  physical_device_multiview_features.multiviewGeometryShader = VK_FALSE;
  physical_device_multiview_features.multiviewTessellationShader = VK_FALSE;

  VkPhysicalDeviceMeshShaderFeaturesEXT mesh_shader_features_ext{};
  mesh_shader_features_ext.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_MESH_SHADER_FEATURES_EXT;
  mesh_shader_features_ext.pNext = &physical_device_multiview_features;
  mesh_shader_features_ext.meshShader = VK_TRUE;
  mesh_shader_features_ext.taskShader = VK_TRUE;
  mesh_shader_features_ext.multiviewMeshShader = VK_FALSE;
  mesh_shader_features_ext.primitiveFragmentShadingRateMeshShader = VK_FALSE;
  mesh_shader_features_ext.meshShaderQueries = VK_FALSE;

  VkPhysicalDeviceSynchronization2Features physical_device_synchronization2_features{};
  physical_device_synchronization2_features.synchronization2 = VK_TRUE;
  if (Constants::support_mesh_shader) {
    physical_device_synchronization2_features.pNext = &mesh_shader_features_ext;
  } else {
    physical_device_synchronization2_features.pNext = &dynamic_rendering_features;
  }

  physical_device_synchronization2_features.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_SYNCHRONIZATION_2_FEATURES;

  VkPhysicalDeviceVertexInputDynamicStateFeaturesEXT extended_vertex_input_dynamic_state_features{};
  extended_vertex_input_dynamic_state_features.vertexInputDynamicState = VK_TRUE;
  extended_vertex_input_dynamic_state_features.pNext = &physical_device_synchronization2_features;
  extended_vertex_input_dynamic_state_features.sType =
      VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_VERTEX_INPUT_DYNAMIC_STATE_FEATURES_EXT;

  VkPhysicalDeviceExtendedDynamicState3FeaturesEXT extended_dynamic_state3_features{};
  extended_dynamic_state3_features.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_EXTENDED_DYNAMIC_STATE_3_FEATURES_EXT;
  extended_dynamic_state3_features.extendedDynamicState3PolygonMode = VK_TRUE;
  extended_dynamic_state3_features.extendedDynamicState3DepthClampEnable = VK_TRUE;
  extended_dynamic_state3_features.extendedDynamicState3ColorBlendEnable = VK_TRUE;
  extended_dynamic_state3_features.extendedDynamicState3LogicOpEnable = VK_TRUE;
  extended_dynamic_state3_features.extendedDynamicState3ColorBlendEquation = VK_TRUE;
  extended_dynamic_state3_features.extendedDynamicState3ColorWriteMask = VK_TRUE;

  extended_dynamic_state3_features.pNext = &extended_vertex_input_dynamic_state_features;

  VkPhysicalDeviceExtendedDynamicState2FeaturesEXT extended_dynamic_state2_features{};
  extended_dynamic_state2_features.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_EXTENDED_DYNAMIC_STATE_2_FEATURES_EXT;
  extended_dynamic_state2_features.extendedDynamicState2 = VK_TRUE;
  extended_dynamic_state2_features.extendedDynamicState2PatchControlPoints = VK_TRUE;
  extended_dynamic_state2_features.extendedDynamicState2LogicOp = VK_TRUE;
  extended_dynamic_state2_features.pNext = &extended_dynamic_state3_features;

  VkPhysicalDeviceExtendedDynamicStateFeaturesEXT extended_dynamic_state_features{};
  extended_dynamic_state_features.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_EXTENDED_DYNAMIC_STATE_FEATURES_EXT;
  extended_dynamic_state_features.extendedDynamicState = VK_TRUE;
  extended_dynamic_state_features.pNext = &extended_dynamic_state2_features;

  VkPhysicalDeviceFeatures2 device_features2{};
  device_features2.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_FEATURES_2;
  device_features2.pNext = &extended_dynamic_state_features;
  vkGetPhysicalDeviceFeatures2(selected_physical_device->vk_physical_device, &device_features2);

  VkDeviceCreateInfo device_create_info{};
  device_create_info.sType = VK_STRUCTURE_TYPE_DEVICE_CREATE_INFO;
#pragma region Queues requirement
  std::vector<VkDeviceQueueCreateInfo> queue_create_infos;
  std::map<uint32_t, std::pair<uint32_t, std::vector<float>>> unique_queue_families;
  if (selected_physical_device->queue_family_indices.graphics_and_compute_family.has_value()) {
    if (unique_queue_families.find(
            selected_physical_device->queue_family_indices.graphics_and_compute_family.value()) ==
        unique_queue_families.end()) {
      unique_queue_families[selected_physical_device->queue_family_indices.graphics_and_compute_family.value()] =
          std::make_pair(0, std::vector<float>());
    }
    unique_queue_families[selected_physical_device->queue_family_indices.graphics_and_compute_family.value()].first +=
        2;
    unique_queue_families[selected_physical_device->queue_family_indices.graphics_and_compute_family.value()]
        .second.emplace_back(1.f);
    unique_queue_families[selected_physical_device->queue_family_indices.graphics_and_compute_family.value()]
        .second.emplace_back(0.f);
  }
  if (selected_physical_device->queue_family_indices.present_family.has_value()) {
    if (unique_queue_families.find(selected_physical_device->queue_family_indices.present_family.value()) ==
        unique_queue_families.end()) {
      unique_queue_families[selected_physical_device->queue_family_indices.present_family.value()] =
          std::make_pair(0, std::vector<float>());
    }
    unique_queue_families[selected_physical_device->queue_family_indices.present_family.value()].first += 1;
    unique_queue_families[selected_physical_device->queue_family_indices.present_family.value()].second.emplace_back(
        0.f);
  }

  for (auto& queue_family : unique_queue_families) {
    VkDeviceQueueCreateInfo queue_create_info{};
    queue_create_info.sType = VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO;
    queue_create_info.queueFamilyIndex = queue_family.first;
    queue_create_info.queueCount = queue_family.second.first;
    queue_create_info.pQueuePriorities = queue_family.second.second.data();
    queue_create_infos.push_back(queue_create_info);
  }
  device_create_info.queueCreateInfoCount = static_cast<uint32_t>(queue_create_infos.size());
  device_create_info.pQueueCreateInfos = queue_create_infos.data();
#pragma endregion

  device_create_info.pEnabledFeatures = nullptr;
  device_create_info.pNext = &device_features2;

  device_create_info.enabledExtensionCount = static_cast<uint32_t>(required_device_extension_names_.size());
  device_create_info.ppEnabledExtensionNames = c_required_device_extensions.data();

  device_create_info.enabledLayerCount = static_cast<uint32_t>(required_layers_.size());
  device_create_info.ppEnabledLayerNames = c_required_layers.data();

  if (vkCreateDevice(selected_physical_device->vk_physical_device, &device_create_info, nullptr, &vk_device_) !=
      VK_SUCCESS) {
    throw std::runtime_error("Failed to create logical device!");
  }
  if (selected_physical_device->queue_family_indices.graphics_and_compute_family.has_value()) {
    main_queue_ = std::make_unique<CommandQueue>();
    immediate_submit_queue_ = std::make_unique<CommandQueue>();
    vkGetDeviceQueue(vk_device_, selected_physical_device->queue_family_indices.graphics_and_compute_family.value(), 0,
                     &immediate_submit_queue_->vk_queue_);
    vkGetDeviceQueue(vk_device_, selected_physical_device->queue_family_indices.graphics_and_compute_family.value(), 1,
                     &main_queue_->vk_queue_);
  }
  if (selected_physical_device->queue_family_indices.present_family.has_value()) {
    present_queue_ = std::make_unique<CommandQueue>();
    if (selected_physical_device->queue_family_indices.graphics_and_compute_family.value() !=
        selected_physical_device->queue_family_indices.present_family.value()) {
      vkGetDeviceQueue(vk_device_, selected_physical_device->queue_family_indices.present_family.value(), 0,
                       &present_queue_->vk_queue_);
    } else {
      vkGetDeviceQueue(vk_device_, selected_physical_device->queue_family_indices.present_family.value(), 2,
                       &present_queue_->vk_queue_);
    }
  }
#pragma endregion
}

auto Platform::DivUp(const uint32_t a, uint32_t b) -> uint32_t {
  return (a + b - 1) / b;
}

void Platform::EverythingBarrier(const VkCommandBuffer vk_command_buffer) {
  VkMemoryBarrier2 memory_barrier{};
  memory_barrier.sType = VK_STRUCTURE_TYPE_MEMORY_BARRIER_2;
  memory_barrier.srcStageMask = memory_barrier.dstStageMask = VK_PIPELINE_STAGE_ALL_COMMANDS_BIT;
  memory_barrier.srcAccessMask = memory_barrier.dstAccessMask = VK_ACCESS_MEMORY_READ_BIT | VK_ACCESS_MEMORY_WRITE_BIT;
  VkDependencyInfo dependency_info{};
  dependency_info.sType = VK_STRUCTURE_TYPE_DEPENDENCY_INFO;
  dependency_info.memoryBarrierCount = 1;
  dependency_info.pMemoryBarriers = &memory_barrier;

  vkCmdPipelineBarrier2(vk_command_buffer, &dependency_info);
}

void Platform::SetupVmaAllocator() {
#pragma region VMA
  VmaVulkanFunctions vulkan_functions{};
  vulkan_functions.vkGetInstanceProcAddr = vkGetInstanceProcAddr;
  vulkan_functions.vkGetDeviceProcAddr = vkGetDeviceProcAddr;
  const auto& graphics = GetInstance();

  VmaAllocatorCreateInfo vma_allocator_create_info{};
  vma_allocator_create_info.flags = VMA_ALLOCATOR_CREATE_BUFFER_DEVICE_ADDRESS_BIT;
  vma_allocator_create_info.physicalDevice = graphics.selected_physical_device->vk_physical_device;
  vma_allocator_create_info.device = vk_device_;
  vma_allocator_create_info.instance = vk_instance_;
  vma_allocator_create_info.vulkanApiVersion = volkGetInstanceVersion();
  vma_allocator_create_info.pVulkanFunctions = &vulkan_functions;
#if ENABLE_EXTERNAL_MEMORY
  std::vector<VkExternalMemoryHandleTypeFlagsKHR> handle_types;
  handle_types.resize(graphics.selected_physical_device->vk_physical_device_memory_properties.memoryTypeCount);
  for (int i = 0; i < graphics.selected_physical_device->vk_physical_device_memory_properties.memoryTypeCount; i++) {
    if (graphics.selected_physical_device->vk_physical_device_memory_properties.memoryTypes[i].propertyFlags |
        VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT) {
#  ifdef _WIN64
      handle_types[i] = VK_EXTERNAL_MEMORY_HANDLE_TYPE_OPAQUE_WIN32_BIT;
#  else
      handleTypes[i] = VK_EXTERNAL_MEMORY_HANDLE_TYPE_OPAQUE_FD_BIT;
#  endif
    }
  }
  vma_allocator_create_info.pTypeExternalMemoryHandleTypes = handle_types.data();
#endif
  vmaCreateAllocator(&vma_allocator_create_info, &vma_allocator_);
#pragma endregion
}

std::string Platform::StringifyResultVk(const VkResult& result) {
  switch (result) {
    case VK_SUCCESS:
      return "Success";
    case VK_NOT_READY:
      return "A fence or query has not yet completed";
    case VK_TIMEOUT:
      return "A wait operation has not completed in the specified time";
    case VK_EVENT_SET:
      return "An event is signaled";
    case VK_EVENT_RESET:
      return "An event is not signaled";
    case VK_INCOMPLETE:
      return "A return array was too small for the result";
    case VK_ERROR_OUT_OF_HOST_MEMORY:
      return "A host memory allocation has failed";
    case VK_ERROR_OUT_OF_DEVICE_MEMORY:
      return "A device memory allocation has failed";
    case VK_ERROR_INITIALIZATION_FAILED:
      return "Initialization of an object could not be completed for implementation-specific reasons";
    case VK_ERROR_DEVICE_LOST:
      return "The logical or physical device has been lost";
    case VK_ERROR_MEMORY_MAP_FAILED:
      return "Mapping of a memory object has failed";
    case VK_ERROR_LAYER_NOT_PRESENT:
      return "A requested layer is not present or could not be loaded";
    case VK_ERROR_EXTENSION_NOT_PRESENT:
      return "A requested extension is not supported";
    case VK_ERROR_FEATURE_NOT_PRESENT:
      return "A requested feature is not supported";
    case VK_ERROR_INCOMPATIBLE_DRIVER:
      return "The requested version of Vulkan is not supported by the driver or is otherwise incompatible";
    case VK_ERROR_TOO_MANY_OBJECTS:
      return "Too many objects of the type have already been created";
    case VK_ERROR_FORMAT_NOT_SUPPORTED:
      return "A requested format is not supported on this device";
    case VK_ERROR_SURFACE_LOST_KHR:
      return "A surface is no longer available";
      // case VK_ERROR_OUT_OF_POOL_MEMORY:
      // return "A allocation failed due to having no more space in the descriptor pool";
    case VK_SUBOPTIMAL_KHR:
      return "A swapchain no longer matches the surface properties exactly, but can still be used";
    case VK_ERROR_OUT_OF_DATE_KHR:
      return "A surface has changed in such a way that it is no longer compatible with the swapchain";
    case VK_ERROR_INCOMPATIBLE_DISPLAY_KHR:
      return "The display used by a swapchain does not use the same presentable image layout";
    case VK_ERROR_NATIVE_WINDOW_IN_USE_KHR:
      return "The requested window is already connected to a VkSurfaceKHR, or to some other non-Vulkan API";
    case VK_ERROR_VALIDATION_FAILED_EXT:
      return "A validation layer found an error";
    default:
      return "Unknown Vulkan error";
  }
}

void Platform::CheckVk(const VkResult& result) {
  if (result >= 0) {
    return;
  }
#if USE_NSIGHT_AFTERMATH
  if (result == VK_ERROR_DEVICE_LOST) {
    // Device lost notification is asynchronous to the NVIDIA display
    // driver's GPU crash handling. Give the Nsight Aftermath GPU crash dump
    // thread some time to do its work before terminating the process.
    constexpr auto tdr_termination_timeout = std::chrono::seconds(3);
    const auto t_start = std::chrono::steady_clock::now();
    auto t_elapsed = std::chrono::milliseconds::zero();

    GFSDK_Aftermath_CrashDump_Status status = GFSDK_Aftermath_CrashDump_Status_Unknown;
    AFTERMATH_CHECK_ERROR(GFSDK_Aftermath_GetCrashDumpStatus(&status));

    while (status != GFSDK_Aftermath_CrashDump_Status_CollectingDataFailed &&
           status != GFSDK_Aftermath_CrashDump_Status_Finished && t_elapsed < tdr_termination_timeout) {
      // Sleep 50ms and poll the status again until timeout or Aftermath finished processing the crash dump.
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
      AFTERMATH_CHECK_ERROR(GFSDK_Aftermath_GetCrashDumpStatus(&status));

      auto t_end = std::chrono::steady_clock::now();
      t_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(t_end - t_start);
    }

    if (status != GFSDK_Aftermath_CrashDump_Status_Finished) {
      std::stringstream err_msg;
      err_msg << "Unexpected crash dump status: " << status;
      throw std::runtime_error(err_msg.str().c_str());
    }

    // Terminate on failure
    EVOENGINE_ERROR("Vulkan error : Device Lost, GPU dump file saved to application folder.");
  }
#endif

  const std::string failure = StringifyResultVk(result);
  throw std::runtime_error("Vulkan error: " + failure);
}

void Platform::CreateSwapChain() {
  const auto application_info = Application::GetApplicationInfo();
  const auto window_layer = Application::GetLayer<WindowLayer>();

  auto& graphics = GetInstance();
  graphics.selected_physical_device->QuerySwapChainSupport();

  const auto& swap_chain_support_details = graphics.selected_physical_device->swap_chain_support_details;
  VkSurfaceFormatKHR surface_format = swap_chain_support_details.formats[0];
  for (const auto& available_format : swap_chain_support_details.formats) {
    if (available_format.format == VK_FORMAT_B8G8R8A8_UNORM &&
        available_format.colorSpace == VK_COLOR_SPACE_SRGB_NONLINEAR_KHR) {
      surface_format = available_format;
      break;
    }
  }
  VkPresentModeKHR present_mode = VK_PRESENT_MODE_FIFO_KHR;
  for (const auto& available_present_mode : swap_chain_support_details.present_modes) {
    if (available_present_mode == VK_PRESENT_MODE_MAILBOX_KHR) {
      present_mode = available_present_mode;
      break;
    }
  }

  VkExtent2D extent = {};
  if (swapchain_)
    extent = swapchain_->GetImageExtent();
  if (swap_chain_support_details.capabilities.currentExtent.width != 0 &&
      swap_chain_support_details.capabilities.currentExtent.height != 0 &&
      swap_chain_support_details.capabilities.currentExtent.width != std::numeric_limits<uint32_t>::max()) {
    extent = swap_chain_support_details.capabilities.currentExtent;
  } else {
    int width, height;
    if (window_layer)
      glfwGetFramebufferSize(window_layer->window_, &width, &height);
    else {
      width = application_info.default_window_size.x;
      height = application_info.default_window_size.y;
    }
    if (width > 0 && height > 0) {
      VkExtent2D actual_extent = {static_cast<uint32_t>(width), static_cast<uint32_t>(height)};

      actual_extent.width =
          std::clamp(actual_extent.width, swap_chain_support_details.capabilities.minImageExtent.width,
                     swap_chain_support_details.capabilities.maxImageExtent.width);
      actual_extent.height =
          std::clamp(actual_extent.height, swap_chain_support_details.capabilities.minImageExtent.height,
                     swap_chain_support_details.capabilities.maxImageExtent.height);
      extent = actual_extent;
    }
  }

  uint32_t image_count = swap_chain_support_details.capabilities.minImageCount + 1;
  if (swap_chain_support_details.capabilities.maxImageCount > 0 &&
      image_count > swap_chain_support_details.capabilities.maxImageCount) {
    image_count = swap_chain_support_details.capabilities.maxImageCount;
  }

  VkSwapchainCreateInfoKHR swapchain_create_info{};
  swapchain_create_info.sType = VK_STRUCTURE_TYPE_SWAPCHAIN_CREATE_INFO_KHR;
  swapchain_create_info.surface = vk_surface_;

  swapchain_create_info.minImageCount = image_count;
  swapchain_create_info.imageFormat = surface_format.format;
  swapchain_create_info.imageColorSpace = surface_format.colorSpace;
  swapchain_create_info.imageExtent = extent;
  swapchain_create_info.imageArrayLayers = 1;
  /*
   * It is also possible that you'll render images to a separate image first to perform operations like post-processing.
   * In that case you may use a value like VK_IMAGE_USAGE_TRANSFER_DST_BIT instead and use a memory operation to
   * transfer the rendered image to a swap chain image.
   */
  swapchain_create_info.imageUsage = VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT;

  uint32_t queue_family_indices[] = {
      graphics.selected_physical_device->queue_family_indices.graphics_and_compute_family.value(),
      graphics.selected_physical_device->queue_family_indices.present_family.value()};

  if (graphics.selected_physical_device->queue_family_indices.graphics_and_compute_family !=
      graphics.selected_physical_device->queue_family_indices.present_family) {
    swapchain_create_info.imageSharingMode = VK_SHARING_MODE_CONCURRENT;
    swapchain_create_info.queueFamilyIndexCount = 2;
    swapchain_create_info.pQueueFamilyIndices = queue_family_indices;
  } else {
    swapchain_create_info.imageSharingMode = VK_SHARING_MODE_EXCLUSIVE;
  }

  swapchain_create_info.preTransform = swap_chain_support_details.capabilities.currentTransform;
  swapchain_create_info.compositeAlpha = VK_COMPOSITE_ALPHA_OPAQUE_BIT_KHR;
  swapchain_create_info.presentMode = present_mode;
  swapchain_create_info.clipped = VK_TRUE;

  if (swapchain_) {
    swapchain_create_info.oldSwapchain = swapchain_->GetVkSwapchain();
  } else {
    swapchain_create_info.oldSwapchain = VK_NULL_HANDLE;
  }

  if (extent.width == 0) {
    EVOENGINE_ERROR("WRONG")
  }
  swapchain_ = std::make_shared<Swapchain>(swapchain_create_info);

  VkSemaphoreCreateInfo semaphore_create_info{};
  semaphore_create_info.sType = VK_STRUCTURE_TYPE_SEMAPHORE_CREATE_INFO;
  image_available_semaphores_.clear();
  for (int i = 0; i < max_frame_in_flight_; i++) {
    image_available_semaphores_.emplace_back(std::make_unique<Semaphore>(semaphore_create_info));
  }

  swapchain_version_++;
}

void Platform::CreateSwapChainSyncObjects() {
  VkSemaphoreCreateInfo semaphore_create_info{};
  semaphore_create_info.sType = VK_STRUCTURE_TYPE_SEMAPHORE_CREATE_INFO;
  VkFenceCreateInfo fence_create_info{};
  fence_create_info.sType = VK_STRUCTURE_TYPE_FENCE_CREATE_INFO;
  fence_create_info.flags = VK_FENCE_CREATE_SIGNALED_BIT;
  render_finished_semaphores_.clear();
  in_flight_fences_.clear();
  for (int i = 0; i < max_frame_in_flight_; i++) {
    render_finished_semaphores_.emplace_back(std::make_unique<Semaphore>(semaphore_create_info));
    in_flight_fences_.emplace_back(std::make_unique<Fence>(fence_create_info));
  }
}

void Platform::RecreateSwapChain() {
  vkDeviceWaitIdle(vk_device_);
  CreateSwapChain();
}

void Platform::OnDestroy() {
  const auto& window_layer = Application::GetLayer<WindowLayer>();
  if (const auto& editor_layer = Application::GetLayer<EditorLayer>(); window_layer && editor_layer) {
    ImGui_ImplVulkan_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImNodes::DestroyContext();
    ImGui::DestroyContext();
  }

  vkDeviceWaitIdle(vk_device_);

  descriptor_pool_.reset();

#pragma region Vulkan
  image_available_semaphores_.clear();
  command_pool_.reset();
  swapchain_.reset();

  vkDestroyDevice(vk_device_, nullptr);
#pragma region Debug Messenger
#ifndef NDEBUG
  DestroyDebugUtilsMessengerExt(vk_instance_, vk_debug_messenger_, nullptr);
#endif
#pragma endregion
#pragma region Surface
  vkDestroySurfaceKHR(vk_instance_, vk_surface_, nullptr);
#pragma endregion
  vmaDestroyAllocator(vma_allocator_);
  vkDestroyInstance(vk_instance_, nullptr);
#pragma endregion
}

void Platform::SwapChainSwapImage() {
  if (const auto& window_layer = Application::GetLayer<WindowLayer>();
      window_layer->window_size_.x == 0 || window_layer->window_size_.y == 0)
    return;
  const auto just_now = Times::Now();
  vkDeviceWaitIdle(vk_device_);
  GeometryStorage::DeviceSync();
  TextureStorage::DeviceSync();
  const VkFence in_flight_fences[] = {in_flight_fences_[current_frame_index_]->GetVkFence()};
  vkWaitForFences(vk_device_, 1, in_flight_fences, VK_TRUE, UINT64_MAX);
  cpu_wait_time = Times::Now() - just_now;
  auto result = vkAcquireNextImageKHR(vk_device_, swapchain_->GetVkSwapchain(), UINT64_MAX,
                                      image_available_semaphores_[current_frame_index_]->GetVkSemaphore(),
                                      VK_NULL_HANDLE, &next_image_index_);
  while (result == VK_ERROR_OUT_OF_DATE_KHR || result == VK_SUBOPTIMAL_KHR || recreate_swap_chain_) {
    RecreateSwapChain();
    result = vkAcquireNextImageKHR(vk_device_, swapchain_->GetVkSwapchain(), UINT64_MAX,
                                   image_available_semaphores_[current_frame_index_]->GetVkSemaphore(), VK_NULL_HANDLE,
                                   &next_image_index_);
    recreate_swap_chain_ = false;
  }
  if (result != VK_SUCCESS) {
    throw std::runtime_error("failed to acquire swap chain image!");
  }
  vkResetFences(vk_device_, 1, in_flight_fences);
}

void Platform::SubmitPresent() {
  if (const auto& window_layer = Application::GetLayer<WindowLayer>();
      window_layer->window_size_.x == 0 || window_layer->window_size_.y == 0)
    return;

  std::vector<std::pair<std::shared_ptr<Semaphore>, VkPipelineStageFlags>> wait_semaphores;
  std::vector<std::shared_ptr<Semaphore>> signal_semaphores;

  wait_semaphores.emplace_back(image_available_semaphores_[current_frame_index_],
                               VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT);
  signal_semaphores.emplace_back(render_finished_semaphores_[current_frame_index_]);

  main_queue_->Submit(command_buffer_pool_[current_frame_index_], 0, used_command_buffer_size_, wait_semaphores,
                      signal_semaphores, in_flight_fences_[current_frame_index_]);

  std::vector<std::pair<std::shared_ptr<Swapchain>, uint32_t>> targets;
  targets.emplace_back(swapchain_, next_image_index_);
  present_queue_->Present(signal_semaphores, targets);

  current_frame_index_ = (current_frame_index_ + 1) % max_frame_in_flight_;
}

void Platform::WaitForCommandsComplete() const {
  vkDeviceWaitIdle(vk_device_);
  GeometryStorage::DeviceSync();
  TextureStorage::DeviceSync();
  const VkFence in_flight_fences[] = {in_flight_fences_[current_frame_index_]->GetVkFence()};
  vkWaitForFences(vk_device_, 1, in_flight_fences, VK_TRUE, UINT64_MAX);
  vkResetFences(vk_device_, 1, in_flight_fences);
}

void Platform::Submit() {
  main_queue_->Submit(command_buffer_pool_[current_frame_index_], 0, used_command_buffer_size_, {}, {},
                      in_flight_fences_[current_frame_index_]);
  current_frame_index_ = (current_frame_index_ + 1) % max_frame_in_flight_;
}

void Platform::ResetCommandBuffers() {
  for (const auto& command_buffer : command_buffer_pool_[current_frame_index_]) {
    if (command_buffer->status_ == CommandBufferStatus::Recorded)
      command_buffer->Reset();
  }
  used_command_buffer_size_ = 0;
}

#pragma endregion


void Platform::PostResourceLoadingInitialization() {
  const auto& graphics = GetInstance();
  PrepareDescriptorSetLayouts();
  graphics.CreateGraphicsPipelines();
}

void Platform::Destroy() {
  auto& graphics = GetInstance();
  graphics.OnDestroy();
}

void Platform::PreUpdate() {
  auto& graphics = GetInstance();
  const auto window_layer = Application::GetLayer<WindowLayer>();
  const auto render_layer = Application::GetLayer<RenderLayer>();
  if (window_layer) {
    if (glfwWindowShouldClose(window_layer->window_)) {
      Application::End();
    }
    if (render_layer || Application::GetLayer<EditorLayer>()) {
      graphics.SwapChainSwapImage();
    }
  } else {
    graphics.WaitForCommandsComplete();
  }

  graphics.ResetCommandBuffers();

  if (render_layer && !Application::GetLayer<EditorLayer>()) {
    if (const auto scene = Application::GetActiveScene()) {
      if (const auto main_camera = scene->main_camera.Get<Camera>(); main_camera && main_camera->IsEnabled()) {
        main_camera->SetRequireRendering(true);
        if (window_layer)
          main_camera->Resize(
              {graphics.swapchain_->GetImageExtent().width, graphics.swapchain_->GetImageExtent().height});
      }
    }
  }
}

void Platform::LateUpdate() {
  auto& graphics = GetInstance();
  if (const auto window_layer = Application::GetLayer<WindowLayer>()) {
    if (Application::GetLayer<RenderLayer>() && !Application::GetLayer<EditorLayer>()) {
      if (const auto scene = Application::GetActiveScene()) {
        if (const auto main_camera = scene->main_camera.Get<Camera>();
            main_camera->IsEnabled() && main_camera->rendered_) {
          const auto& render_texture_present = graphics.graphics_pipelines_["RENDER_TEXTURE_PRESENT"];
          RecordCommandsMainQueue([&](VkCommandBuffer vk_command_buffer) {
            EverythingBarrier(vk_command_buffer);
            TransitImageLayout(vk_command_buffer, graphics.swapchain_->GetVkImage(),
                               graphics.swapchain_->GetImageFormat(), 1, VK_IMAGE_LAYOUT_UNDEFINED,
                               VK_IMAGE_LAYOUT_ATTACHMENT_OPTIMAL_KHR);

            constexpr VkClearValue clear_color = {{{0.0f, 0.0f, 0.0f, 1.0f}}};
            VkRect2D render_area;
            render_area.offset = {0, 0};
            render_area.extent = graphics.swapchain_->GetImageExtent();

            VkRenderingAttachmentInfo color_attachment_info{};
            color_attachment_info.sType = VK_STRUCTURE_TYPE_RENDERING_ATTACHMENT_INFO;
            color_attachment_info.imageView = graphics.swapchain_->GetVkImageView();
            color_attachment_info.imageLayout = VK_IMAGE_LAYOUT_ATTACHMENT_OPTIMAL_KHR;
            color_attachment_info.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
            color_attachment_info.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
            color_attachment_info.clearValue = clear_color;

            VkRenderingInfo render_info{};
            render_info.sType = VK_STRUCTURE_TYPE_RENDERING_INFO;
            render_info.renderArea = render_area;
            render_info.layerCount = 1;
            render_info.colorAttachmentCount = 1;
            render_info.pColorAttachments = &color_attachment_info;
            VkViewport viewport;
            viewport.x = 0.0f;
            viewport.y = 0.0f;
            viewport.width = render_area.extent.width;
            viewport.height = render_area.extent.height;
            viewport.minDepth = 0.0f;
            viewport.maxDepth = 1.0f;

            VkRect2D scissor;
            scissor.offset = {0, 0};
            scissor.extent.width = render_area.extent.width;
            scissor.extent.height = render_area.extent.height;

            render_texture_present->states.view_port = viewport;
            render_texture_present->states.scissor = scissor;
            render_texture_present->states.color_blend_attachment_states.clear();
            render_texture_present->states.color_blend_attachment_states.resize(1);
            for (auto& i : render_texture_present->states.color_blend_attachment_states) {
              i.colorWriteMask = VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT | VK_COLOR_COMPONENT_B_BIT |
                                 VK_COLOR_COMPONENT_A_BIT;
              i.blendEnable = VK_FALSE;
            }
            render_texture_present->states.depth_test = VK_FALSE;
            render_texture_present->states.depth_write = VK_FALSE;
            vkCmdBeginRendering(vk_command_buffer, &render_info);
            // From main camera to swap chain.
            render_texture_present->Bind(vk_command_buffer);
            render_texture_present->BindDescriptorSet(
                vk_command_buffer, 0, main_camera->GetRenderTexture()->present_descriptor_set_->GetVkDescriptorSet());

            const auto mesh = Resources::GetResource<Mesh>("PRIMITIVE_TEX_PASS_THROUGH");
            GeometryStorage::BindVertices(vk_command_buffer);
            mesh->DrawIndexed(vk_command_buffer, render_texture_present->states, 1);
            vkCmdEndRendering(vk_command_buffer);
            TransitImageLayout(vk_command_buffer, graphics.swapchain_->GetVkImage(),
                               graphics.swapchain_->GetImageFormat(), 1, VK_IMAGE_LAYOUT_ATTACHMENT_OPTIMAL_KHR,
                               VK_IMAGE_LAYOUT_PRESENT_SRC_KHR);
          });
        }
      }
    }
    graphics.SubmitPresent();
  } else {
    graphics.Submit();
  }
}

bool Platform::CheckExtensionSupport(const std::string& extension_name) {
  const auto& graphics = GetInstance();

  for (const auto& layer_properties : graphics.vk_supported_layers_) {
    if (strcmp(extension_name.c_str(), layer_properties.layerName) == 0) {
      return true;
    }
  }
  return false;
}

bool Platform::CheckLayerSupport(const std::string& layer_name) {
  const auto& graphics = GetInstance();
  for (const auto& layer_properties : graphics.vk_supported_layers_) {
    if (strcmp(layer_name.c_str(), layer_properties.layerName) == 0) {
      return true;
    }
  }
  return false;
}
