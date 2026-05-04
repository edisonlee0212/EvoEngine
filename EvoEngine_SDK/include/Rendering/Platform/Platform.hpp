
#pragma once
#include "ApplicationInitializationSettings.hpp"
#include "ComputePipeline.hpp"
#include "GraphicsPipeline.hpp"
#include "GraphicsResources.hpp"
#include "RayTracingPipeline.hpp"

#define ENABLE_EXTERNAL_MEMORY true

#define ENABLE_NV_RAY_TRACING_VALIDATION false

#ifndef USE_RENDERDOC
#  undef ENABLE_NVIDIA_NSIGHT_AFTERMATH
#endif  // !USE_RENDERDOC

#ifdef ENABLE_NVIDIA_NSIGHT_AFTERMATH
#  include "NsightAftermathGpuCrashTracker.h"
#  include "NsightAftermathHelpers.h"
#  include "NsightAftermathShaderDatabase.h"
#endif

namespace evo_engine {

/**
 * @brief Class representing platform-specific Vulkan setup and utilities.
 *
 * The Platform class manages all Vulkan-related initialization,
 * logical device creation, resource management, and rendering synchronization.
 */
class Platform final {
 public:
  static Platform& GetInstance();

 private:
  friend class Application;
  friend class Resources;
  friend class Lighting;
  friend class PointLightShadowMap;
  friend class SpotLightShadowMap;

#pragma region Vulkan
  /// Vulkan instance object.
  VkInstance vk_instance_ = VK_NULL_HANDLE;

  /// Indicates whether the platform is initialized.
  bool initialized = false;

  /// List of required Vulkan layers.
  std::vector<std::string> required_layers_ = {};

  /// List of supported Vulkan layers.
  std::vector<VkLayerProperties> vk_supported_layers_;

  /// List of required Vulkan instance extension names.
  std::vector<std::string> required_instance_extension_names_ = {};

  /// Supported Vulkan instance extensions.
  std::unordered_map<std::string, VkExtensionProperties> vk_supported_instance_extensions_;

  /// List of required Vulkan device extension names.
  std::vector<std::string> required_device_extension_names_ = {};

  /**
   * @brief Struct encapsulating Vulkan physical device data.
   */
  struct PhysicalDevice {
    /// Vulkan physical device handler.
    VkPhysicalDevice vk_physical_device{};

    /// Supported Vulkan extensions for the device.
    std::unordered_map<std::string, VkExtensionProperties> supported_vk_extensions{};

    /// Vulkan physical device properties.
    VkPhysicalDeviceProperties properties{};

    /// Vulkan 1.1 specific device properties.
    VkPhysicalDeviceProperties2 properties2 = {VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_PROPERTIES_2};

    /// Vulkan 1.1 specific properties.
    VkPhysicalDeviceVulkan11Properties vulkan11_properties{VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_VULKAN_1_1_PROPERTIES};

    /// Vulkan 1.2 specific properties.
    VkPhysicalDeviceVulkan12Properties vulkan12_properties{VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_VULKAN_1_2_PROPERTIES};

    /// Mesh shader specific properties for Vulkan.
    VkPhysicalDeviceMeshShaderPropertiesEXT mesh_shader_properties_ext = {
        VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_MESH_SHADER_PROPERTIES_EXT};

    /// Subgroup size control properties for Vulkan.
    VkPhysicalDeviceSubgroupSizeControlProperties subgroup_size_control_properties = {
        VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_SUBGROUP_SIZE_CONTROL_PROPERTIES};

    /// Ray tracing pipeline properties for Vulkan.
    VkPhysicalDeviceRayTracingPipelinePropertiesKHR ray_tracing_properties_ext = {
        VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_RAY_TRACING_PIPELINE_PROPERTIES_KHR};

    /// Acceleration structure properties for Vulkan.
    VkPhysicalDeviceAccelerationStructurePropertiesKHR acceleration_structure_properties_khr = {
        VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_ACCELERATION_STRUCTURE_PROPERTIES_KHR};

#if ENABLE_NV_RAY_TRACING_VALIDATION
    /// Ray tracing validation features specific to NVIDIA Vulkan implementation.
    VkPhysicalDeviceRayTracingValidationFeaturesNV ray_tracing_validation_features_nv = {
        VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_RAY_TRACING_VALIDATION_FEATURES_NV};
#endif

    /// Memory properties for the physical device.
    VkPhysicalDeviceMemoryProperties vk_physical_device_memory_properties = {};

    /// Physical device features.
    VkPhysicalDeviceFeatures features{};

    /// Acceleration structure features.
    VkPhysicalDeviceAccelerationStructureFeaturesKHR acceleration_structure_features{};

    /**
     * @brief Struct representing queue family indices needed by the application.
     */
    struct QueueFamilyIndices {
      std::optional<uint32_t> graphics_and_compute_family{};
      std::optional<uint32_t> present_family{};

      /**
       * @brief Checks if the required queue families are complete.
       * @return True if both graphics and present families are specified.
       */
      [[nodiscard]] bool IsComplete() const;
    };

    /**
     * @brief Struct containing swapchain support details.
     */
    struct SwapChainSupportDetails {
      VkSurfaceCapabilitiesKHR capabilities{};        ///< Surface capabilities.
      std::vector<VkSurfaceFormatKHR> formats{};      ///< List of supported formats.
      std::vector<VkPresentModeKHR> present_modes{};  ///< List of supported presentation modes.
    };

    QueueFamilyIndices queue_family_indices = {};          ///< Queue family indices.
    SwapChainSupportDetails swap_chain_support_details{};  ///< Swapchain support details.

    uint32_t score = 0;  ///< Device scoring used for selection.

    /**
     * @brief Queries details of the physical device's capabilities.
     */
    void QueryInformation();

    /**
     * @brief Queries the details of supported swapchains.
     */
    void QuerySwapChainSupport();

    /**
     * @brief Checks if a required extension is supported by the physical device.
     *
     * @param required_extension_name Name of the extension to check.
     * @return True if the extension is supported.
     */
    [[nodiscard]] bool CheckExtensionSupport(const std::string& required_extension_name) const;

    /**
     * @brief Finds a suitable memory type based on a filter and properties.
     *
     * @param type_filter Filter representing the memory types.
     * @param properties Desired memory properties.
     * @return Index of the memory type.
     */
    [[nodiscard]] uint32_t FindMemoryType(uint32_t type_filter, VkMemoryPropertyFlags properties) const;

    /**
     * @brief Determines if the device is suitable for usage based on required extensions.
     *
     * @param required_extension_names List of required extension names.
     * @return True if the device is suitable.
     */
    [[nodiscard]] bool Suitable(const std::vector<std::string>& required_extension_names) const;
  };

  /// Vulkan debug messenger for validation layers.
  VkDebugUtilsMessengerEXT vk_debug_messenger_ = {};

  /// List of all physical devices found on the system.
  std::vector<std::shared_ptr<PhysicalDevice>> physical_devices_{};

  /// The selected physical device used for rendering.
  std::shared_ptr<PhysicalDevice> selected_physical_device{};

#ifdef ENABLE_NVIDIA_NSIGHT_AFTERMATH
  GpuCrashTracker::MarkerMap markerMap;          ///< Marker map for GPU crash tracking.
  GpuCrashTracker gpu_crash_tracker{markerMap};  ///< GPU crash tracker.
#endif

  /// Vulkan surface object.
  VkSurfaceKHR vk_surface_ = VK_NULL_HANDLE;

  /// Vulkan logical device handle.
  VkDevice vk_device_ = VK_NULL_HANDLE;

  /// Vulkan Memory Allocator (VMA) handle.
  VmaAllocator vma_allocator_ = VK_NULL_HANDLE;

  /// Immediate submit queue for rendering commands.
  std::unique_ptr<CommandQueue> immediate_submit_queue_{};

  /// Queue used for primary rendering operations.
  std::unique_ptr<CommandQueue> main_queue_{};

  /// Queue used for presenting frames to the screen.
  std::unique_ptr<CommandQueue> present_queue_{};

  /// Vulkan swapchain object.
  std::shared_ptr<Swapchain> swapchain_ = {};
  friend class WindowLayer;  ///< Window-related operations.

  /// Graphics pipeline used for presenting rendered images to the screen.
  std::shared_ptr<GraphicsPipeline> render_texture_present_pipeline{};

  /// Format of the Vulkan swapchain surface.
  VkSurfaceFormatKHR vk_surface_format_ = {};

#pragma endregion

#pragma region Internals
  std::unique_ptr<CommandPool> command_pool_ = {};        ///< Command pool for Vulkan commands.
  std::unique_ptr<DescriptorPool> descriptor_pool_ = {};  ///< Descriptor pool for Vulkan descriptors.

  int max_frame_in_flight_ = 2;  ///< Max number of frames in flight.

  std::vector<std::shared_ptr<Semaphore>> image_available_semaphores_ = {};  ///< Semaphores for image availability.
  std::vector<std::shared_ptr<Semaphore>> render_finished_semaphores_ = {};  ///< Semaphores for render finish.
  std::vector<std::shared_ptr<Fence>> in_flight_fences_ = {};                ///< Fences for in-flight frames.

  uint32_t current_frame_index_ = 0;  ///< Index of current frame being rendered.

  uint32_t next_image_index_ = 0;  ///< Index of the next image in the swapchain.

#pragma endregion
#pragma region Shader related
  /// Path to the skybox shader.
  std::string shader_skybox_;

  /// Maximum number of bones allowed.
  size_t max_bone_amount_ = 65536;

  /// Maximum number of shadow cascade levels.
  size_t max_shadow_cascade_amount_ = 4;

  friend class RenderLayer;  ///< Access for rendering operations.

#pragma endregion

  void CreateInstance();
  void CreateSurface();
  void CreateDebugMessenger();
  void SelectPhysicalDevice();
  void CreateLogicalDevice();
  void SetupVmaAllocator();
  void CreateSwapChain();
  void CreateSwapChainSyncObjects();
  void RecreateSwapChain();

  /**
   * @brief Resets command buffers for reuse.
   */
  void ResetCommandBuffers();

  /**
   * @brief Initializes the platform and its Vulkan components.
   */
  static void Initialize(const ApplicationInitializationSettings& application_initialization_settings);

  /**
   * @brief Destroys the platform and cleans up Vulkan resources.
   */
  static void OnDestroy();

  /**
   * @brief Performs pre-update operations before the main update loop.
   */
  static void PreUpdate();

  /**
   * @brief Performs late update operations after the main update loop.
   */
  static void LateUpdate();

  /// Frame count since initialization.
  uint32_t frame_count = 0;

  /// Indicates if the swapchain needs to be recreated.
  bool recreate_swap_chain_ = false;

  /// Version of the current swapchain.
  unsigned swapchain_version_ = 0;

  /// Size of used command buffers.
  int used_command_buffer_size_ = 0;

  /// Pool of command buffers categorized by usage.
  std::vector<std::vector<std::shared_ptr<CommandBuffer>>> command_buffer_pool_ = {};

  /// Command buffer used for immediate execution of commands.
  std::shared_ptr<CommandBuffer> immediate_submit_command_buffer{};

  /// Map of named buffer synchronization actions.
  std::unordered_map<std::string, std::function<void()>> buffer_sync_actions{};

  /// Temporary list of buffer synchronization actions.
  std::vector<std::function<void()>> temporary_buffer_sync_actions{};

  /**
   * @brief Global defines for shaders, set during initialization.
   */
  std::string shader_global_defines = {};

 public:
  static bool RayTracingEnabled();
  static bool MeshShaderEnabled();
  /**
   * @brief Checks if the platform is initialized.
   *
   * @return True if the platform is initialized, false otherwise.
   */
  static bool Initialized();

  /**
   * @brief Gets the current frame count since initialization.
   *
   * @return The frame count.
   */
  static uint32_t GetFrameCount();

  /**
   * @brief Adds a temporary buffer synchronization action.
   *
   * @param action The action to add.
   */
  static void AddTemporaryBufferSyncAction(std::function<void()>&& action);

  /**
   * @brief Adds a buffer synchronization action by name.
   *
   * @param action_name The name of the action.
   * @param action The action to add.
   */
  static void AddBufferSyncAction(const std::string& action_name, std::function<void()>&& action);

  /**
   * @brief Removes a buffer synchronization action by name.
   *
   * @param action_name The name of the action to remove.
   */
  static void RemoveBufferSyncAction(const std::string& action_name);

  /**
   * @brief Records commands for the main queue.
   *
   * @param action The action to record.
   */
  static void RecordCommandsMainQueue(const std::function<void(VkCommandBuffer vk_command_buffer)>& action);

  /**
   * @brief Records render commands with specific rendering info.
   *
   * @param rendering_info The Vulkan rendering information.
   * @param vk_command_buffer The Vulkan command buffer.
   * @param action The action to execute during rendering.
   */
  static void RecordRenderCommands(const VkRenderingInfo& rendering_info, const VkCommandBuffer vk_command_buffer,
                                   const std::function<void()>& action);

  /// Time spent waiting on the CPU in seconds.
  double cpu_wait_time = 0.0f;

  /**
   * @brief Waits for the device to finish all operations.
   */
  static void WaitForDeviceIdle();

  /// List of primitive counts for debugging purposes.
  std::vector<size_t> prim_count{};

  /// List of draw calls for debugging purposes.
  std::vector<size_t> draw_call{};

  /**
   * @brief Constants used for internal configuration and limits.
   */
  class Constants {
   public:
    /// Indicates support for mesh shaders.
    inline static bool support_mesh_shader = true;

    /// Indicates support for ray tracing.
    inline static bool support_ray_tracing = true;

    /// Indicates support for ray tracing validation.
    inline static bool support_ray_tracing_validation = false;

    /// Initial descriptor pool maximum size.
    constexpr static uint32_t initial_descriptor_pool_max_size = 16384;

    /// Initial descriptor pool maximum sets.
    constexpr static uint32_t initial_descriptor_pool_max_sets = 16384;

    /// Initial camera count.
    constexpr static uint32_t initial_camera_size = 1;

    /// Initial material count for rendering.
    constexpr static uint32_t initial_material_size = 1;

    /// Initial instance count for rendering.
    constexpr static uint32_t initial_instance_size = 1;

    /// Initial render task count.
    constexpr static uint32_t initial_render_task_size = 1;

    /// Maximum number of compute or graphics kernels.
    constexpr static uint32_t max_kernel_amount = 64;

    /// Format for 2D textures.
    constexpr static VkFormat texture_2d = VK_FORMAT_R32G32B32A32_SFLOAT;

    /// Depth format for render textures.
    constexpr static VkFormat render_texture_depth = VK_FORMAT_D32_SFLOAT;

    /// Color format for render textures.
    constexpr static VkFormat render_texture_color = VK_FORMAT_R32G32B32A32_SFLOAT;

    /// Color format for G-buffer.
    constexpr static VkFormat g_buffer_color = VK_FORMAT_R32G32B32A32_SFLOAT;

    /// Material format for G-buffer.
    constexpr static VkFormat g_buffer_material = VK_FORMAT_R32G32B32A32_SFLOAT;

    /// Format for shadow maps.
    constexpr static VkFormat shadow_map = VK_FORMAT_D32_SFLOAT;

    /// Format for swapchain images.
    constexpr static VkFormat swap_chain_image_format = VK_FORMAT_B8G8R8A8_UNORM;

    /// Maximum number of vertices in a meshlet.
    constexpr static uint32_t meshlet_max_vertices_size = 64;

    /// Maximum number of triangles in a meshlet.
    constexpr static uint32_t meshlet_max_triangles_size = 40;

    /// Subgroup size used for compute tasks.
    inline static uint32_t subgroup_size = 1;

    /// Number of subgroups for task shaders.
    inline static uint32_t task_subgroup_count = 1;

    /// Number of workgroup invocations for task shaders.
    inline static uint32_t task_work_group_invocations = 1;

    /// Number of subgroups for mesh shaders.
    inline static uint32_t mesh_subgroup_count = 1;

    /// Number of subgroups for compute shaders.
    inline static uint32_t compute_subgroup_count = 1;

    /// Number of workgroup invocations for compute shaders.
    inline static uint32_t compute_work_group_invocations = 1;

    /// Maximum number of workgroup invocations supported for compute shaders.
    inline static uint32_t max_compute_work_group_invocations = 1;

    /// Maximum size of shared memory available in shaders.
    inline static uint32_t max_shared_memory_size = 1;
  };

  /**
   * @brief Divides two integers a and b, rounding up to the nearest integer.
   *
   * @param a The numerator.
   * @param b The denominator.
   * @return The result of the division, rounded up.
   */
  static uint32_t DivUp(uint32_t a, uint32_t b);

  /**
   * @brief Inserts a full pipeline and memory barrier for current Vulkan operations.
   *
   * @param vk_command_buffer Vulkan command buffer to record the barrier.
   */
  static void EverythingBarrier(VkCommandBuffer vk_command_buffer);

  /**
   * @brief Transits the layout of an image within a Vulkan command buffer.
   *
   * @param vk_command_buffer Vulkan command buffer to record the operation.
   * @param target_image The image to transition.
   * @param image_format Image format of the target image.
   * @param layer_count Number of layers in the image.
   * @param old_layout The current layout of the image.
   * @param new_layout The desired layout for the image.
   * @param mip_levels Number of mip levels in the image, default is 1.
   */
  static void TransitImageLayout(VkCommandBuffer vk_command_buffer, VkImage target_image, VkFormat image_format,
                                 uint32_t layer_count, VkImageLayout old_layout, VkImageLayout new_layout,
                                 uint32_t mip_levels = 1);

  /**
   * @brief Gets the global shader defines set during initialization.
   *
   * @return A reference to the global shader defines.
   */
  static const std::string& GetShaderGlobalDefines();

  /**
   * @brief Converts Vulkan result errors into string representations.
   *
   * @param result Vulkan result status to convert.
   * @return A string representation of the Vulkan result.
   */
  static std::string StringifyResultVk(const VkResult& result);

  /**
   * @brief Checks the Vulkan result and throws an error if the result is not VK_SUCCESS.
   *
   * @param result Vulkan result status to check.
   */
  static VkResult CheckVk(const VkResult& result);

  /**
   * @brief Gets the maximum number of bones allowed in shaders.
   *
   * @return The maximum bone amount.
   */
  static size_t GetMaxBoneAmount();

  /**
   * @brief Gets the maximum number of shadow cascades supported.
   *
   * @return The maximum cascade amount.
   */
  static size_t GetMaxShadowCascadeAmount();

  /**
   * @brief Executes a task immediately on the GPU using a command buffer.
   *
   * @param action The task to execute.
   */
  static void ImmediateSubmit(const std::function<void(VkCommandBuffer vk_command_buffer)>& action);

  /**
   * @brief Gets the maximum number of frames that can be in flight at any time.
   *
   * @return Maximum frames in flight.
   */
  static int GetMaxFramesInFlight();

  /**
   * @brief Notifies the system that the swapchain should be recreated.
   */
  static void NotifyRecreateSwapChain();

  /**
   * @brief Gets the Vulkan instance object.
   *
   * @return The Vulkan instance.
   */
  static VkInstance GetVkInstance();

  /**
   * @brief Gets the currently selected Vulkan physical device.
   *
   * @return A shared pointer to the selected physical device.
   */
  static const std::shared_ptr<PhysicalDevice>& GetSelectedPhysicalDevice();

  /**
   * @brief Gets the Vulkan logical device handle.
   *
   * @return The Vulkan logical device.
   */
  static VkDevice GetVkDevice();

  /**
   * @brief Gets the index of the current frame being rendered.
   *
   * @return The current frame index.
   */
  static uint32_t GetCurrentFrameIndex();

  /**
   * @brief Gets the index of the next image in the swapchain.
   *
   * @return The next image index.
   */
  static uint32_t GetNextImageIndex();

  /**
   * @brief Gets the Vulkan command pool.
   *
   * @return The Vulkan command pool handle.
   */
  static VkCommandPool GetVkCommandPool();

  /**
   * @brief Gets the main command queue.
   *
   * @return A unique pointer to the main command queue.
   */
  static const std::unique_ptr<CommandQueue>& GetMainQueue();

  /**
   * @brief Gets the immediate submit command queue.
   *
   * @return A unique pointer to the immediate submit queue.
   */
  static const std::unique_ptr<CommandQueue>& GetImmediateSubmitQueue();

  /**
   * @brief Gets the present queue for rendering frames to the surface.
   *
   * @return A unique pointer to the present queue.
   */
  static const std::unique_ptr<CommandQueue>& GetPresentQueue();

  /**
   * @brief Gets the Vulkan Memory Allocator handle.
   *
   * @return The VMA allocator handle.
   */
  static VmaAllocator GetVmaAllocator();

  /**
   * @brief Gets the current Vulkan swapchain.
   *
   * @return A shared pointer to the swapchain.
   */
  static const std::shared_ptr<Swapchain>& GetSwapchain();

  /**
   * @brief Gets the Vulkan descriptor pool.
   *
   * @return A unique pointer to the descriptor pool.
   */
  static const std::unique_ptr<DescriptorPool>& GetDescriptorPool();

  /**
   * @brief Gets the current swapchain version.
   *
   * @return The version of the swapchain.
   */
  static unsigned GetSwapchainVersion();

  /**
   * @brief Gets the Vulkan surface format of the swapchain.
   *
   * @return Vulkan surface format.
   */
  static VkSurfaceFormatKHR GetVkSurfaceFormat();

  /**
   * @brief Checks if a specific Vulkan extension is supported.
   *
   * @param extension_name The name of the extension.
   * @return True if the extension is supported, false otherwise.
   */
  [[nodiscard]] static bool CheckExtensionSupport(const std::string& extension_name);

  /**
   * @brief Checks if a specific Vulkan layer is supported.
   *
   * @param layer_name The name of the layer.
   * @return True if the layer is supported, false otherwise.
   */
  [[nodiscard]] static bool CheckLayerSupport(const std::string& layer_name);
};
}  // namespace evo_engine
