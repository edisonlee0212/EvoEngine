
#pragma once
#include "ApplicationInitializationSettings.hpp"
#include "ComputePipeline.hpp"
#include "GraphicsPipeline.hpp"
#include "GraphicsResources.hpp"
#include "RayTracingPipeline.hpp"
#include "VulkanPipelineCache.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <optional>
#include <set>

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

struct FrameSubmissionState {
  enum class Status { Pending, Submitted, Discarded };
  Status status = Status::Pending;
};
class GpuService;
class PlatformLifecycleTestAccess;

enum class RenderPassDrawBucket : uint8_t {
  FrameExternal,
  PointLightShadow,
  SpotLightShadow,
  DirectionalLightShadow,
  DeferredGeometry,
  DeferredLighting,
  TransparentGeometry,
  ForwardExternal,
  CameraExternal,
  DdgiProbeVisualization,
  DdgiProbeRayVisualization,
  EditorGizmos,
  Count
};

enum class RenderDrawCallKind : uint8_t { Direct, Indirect };

struct RenderPassDrawStats {
  size_t direct_draw_calls = 0;
  size_t indirect_draw_calls = 0;
  size_t indirect_draw_commands = 0;
  size_t prim_count = 0;
  [[nodiscard]] size_t TotalDrawCalls() const;
};

struct RenderCameraDrawStats {
  uint64_t camera_handle = 0;
  uint32_t entity_index = 0;
  bool scene_camera = false;
  std::array<RenderPassDrawStats, static_cast<size_t>(RenderPassDrawBucket::Count)> pass_stats{};

  [[nodiscard]] RenderPassDrawStats Total() const;
};

struct RenderCameraDrawScope {
  uint32_t frame_index = 0;
  uint64_t camera_handle = 0;
  uint32_t entity_index = 0;
  bool scene_camera = false;
};

struct GpuTimestampStats {
  std::string name{};
  double last_milliseconds = 0.0;
  double minimum_milliseconds = 0.0;
  double maximum_milliseconds = 0.0;
  double total_milliseconds = 0.0;
  uint64_t sample_count = 0;
  std::vector<double> samples_milliseconds{};

  void AddSample(double milliseconds);
  [[nodiscard]] double AverageMilliseconds() const;
  [[nodiscard]] double MedianMilliseconds() const;
  [[nodiscard]] double PercentileMilliseconds(double percentile) const;
};

struct GpuMemoryHeapStats {
  uint32_t heap_index = 0;
  bool device_local = false;
  uint64_t heap_size_bytes = 0;
  uint64_t block_count = 0;
  uint64_t allocation_count = 0;
  uint64_t block_bytes = 0;
  uint64_t allocation_bytes = 0;
  uint64_t driver_usage_bytes = 0;
  uint64_t driver_budget_bytes = 0;
};

struct GpuMemorySnapshot {
  uint64_t block_count = 0;
  uint64_t allocation_count = 0;
  uint64_t block_bytes = 0;
  uint64_t allocation_bytes = 0;
  std::vector<GpuMemoryHeapStats> heaps{};
};

struct GpuDeviceFingerprint {
  std::string device_name{};
  uint32_t vendor_id = 0;
  uint32_t device_id = 0;
  uint32_t device_type = 0;
  uint32_t driver_version = 0;
  uint32_t api_version = 0;
  uint32_t driver_id = 0;
  std::string driver_name{};
  std::string driver_info{};
  std::array<uint8_t, VK_UUID_SIZE> pipeline_cache_uuid{};
  std::array<uint8_t, VK_UUID_SIZE> device_uuid{};
  std::array<uint8_t, VK_UUID_SIZE> driver_uuid{};
  std::array<uint8_t, 4> conformance_version{};
};

struct GpuTimestampScopeToken {
  std::string name{};
  uint32_t frame_index = 0;
  uint32_t begin_query = 0;
  uint32_t end_query = 0;
  bool valid = false;
};

/**
 * @brief Class representing platform-specific Vulkan setup and utilities.
 *
 * The Platform class manages all Vulkan-related initialization,
 * logical device creation, resource management, and rendering synchronization.
 */
class Platform final {
 public:
  static constexpr int kMaxFramesInFlight = 2;

  struct QueueFamilySupport {
    VkQueueFlags queue_flags = 0;
    bool present_support = false;
    uint32_t queue_count = 0;
  };

  struct QueueFamilySelection {
    std::optional<uint32_t> graphics_and_compute_family{};
    std::optional<uint32_t> compute_family{};
    std::optional<uint32_t> present_family{};

    [[nodiscard]] bool HasDedicatedComputeFamily() const;
    [[nodiscard]] bool IsComplete(bool require_present) const;
  };

  static Platform& GetInstance();
  ~Platform();

 private:
  friend class Application;
  friend class Resources;
  friend class Lighting;
  friend class PointLightShadowMap;
  friend class SpotLightShadowMap;
  friend class PlatformLifecycleTestAccess;

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

#ifdef VK_EXT_ray_tracing_invocation_reorder
    VkPhysicalDeviceRayTracingInvocationReorderPropertiesEXT ray_tracing_invocation_reorder_properties_ext = {
        VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_RAY_TRACING_INVOCATION_REORDER_PROPERTIES_EXT};

    VkPhysicalDeviceRayTracingInvocationReorderFeaturesEXT ray_tracing_invocation_reorder_features_ext = {
        VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_RAY_TRACING_INVOCATION_REORDER_FEATURES_EXT};
#endif

#ifdef VK_NV_ray_tracing_invocation_reorder
    VkPhysicalDeviceRayTracingInvocationReorderPropertiesNV ray_tracing_invocation_reorder_properties_nv = {
        VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_RAY_TRACING_INVOCATION_REORDER_PROPERTIES_NV};

    VkPhysicalDeviceRayTracingInvocationReorderFeaturesNV ray_tracing_invocation_reorder_features_nv = {
        VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_RAY_TRACING_INVOCATION_REORDER_FEATURES_NV};
#endif

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
    VkPhysicalDeviceAccelerationStructureFeaturesKHR acceleration_structure_features{
        VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_ACCELERATION_STRUCTURE_FEATURES_KHR};

    /// Ray tracing pipeline features.
    VkPhysicalDeviceRayTracingPipelineFeaturesKHR ray_tracing_pipeline_features{
        VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_RAY_TRACING_PIPELINE_FEATURES_KHR};

    /// Ray query features.
    VkPhysicalDeviceRayQueryFeaturesKHR ray_query_features{VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_RAY_QUERY_FEATURES_KHR};

    /**
     * @brief Struct representing queue family indices needed by the application.
     */
    struct QueueFamilyIndices {
      std::optional<uint32_t> graphics_and_compute_family{};
      std::optional<uint32_t> compute_family{};
      std::optional<uint32_t> present_family{};

      /**
       * @brief Checks if the required queue families are complete.
       * @return True if both graphics and present families are specified.
       */
      [[nodiscard]] bool IsComplete() const;
      [[nodiscard]] bool HasDedicatedComputeFamily() const;
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

  /// Persistent, device-scoped pipeline cache shared by engine pipeline creation.
  std::unique_ptr<VulkanPipelineCache> pipeline_cache_{};

  /// Vulkan Memory Allocator (VMA) handle.
  VmaAllocator vma_allocator_ = VK_NULL_HANDLE;

  /// Immediate submit queue for rendering commands.
  std::unique_ptr<CommandQueue> immediate_submit_queue_{};

  /// GPU service runtime for serialized resource work and immediate submissions.
  std::unique_ptr<GpuService> gpu_service_{};

  /// Queue used for primary rendering operations.
  std::unique_ptr<CommandQueue> main_queue_{};

  /// Optional queue used for dedicated compute work when the device exposes an independent compute family.
  std::unique_ptr<CommandQueue> compute_queue_{};

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
  std::unique_ptr<CommandPool> command_pool_ = {};          ///< Command pool for Vulkan commands.
  std::unique_ptr<CommandPool> compute_command_pool_ = {};  ///< Command pool for dedicated compute commands.
  std::unique_ptr<DescriptorPool> descriptor_pool_ = {};    ///< Descriptor pool for Vulkan descriptors.

  int max_frame_in_flight_ = kMaxFramesInFlight;  ///< Max number of frames in flight.

  std::vector<std::shared_ptr<Semaphore>> image_available_semaphores_ = {};   ///< Semaphores for image availability.
  std::vector<std::shared_ptr<Semaphore>> render_finished_semaphores_ = {};   ///< Semaphores for render finish.
  std::vector<std::shared_ptr<Semaphore>> compute_finished_semaphores_ = {};  ///< Semaphores for compute finish.
  std::vector<std::shared_ptr<Fence>> in_flight_fences_ = {};                 ///< Fences for in-flight frames.
  std::vector<std::vector<std::weak_ptr<FrameSubmissionState>>> frame_submission_states_ = {};
  std::vector<bool> frame_slot_submitted_ = {};

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
  void WaitForFrameSlotSubmission(uint32_t frame_index, const std::string& wait_name);
  void InitializeGpuTimestampResources();
  void DestroyGpuTimestampResources();
  void PrepareGpuTimestampFrame(uint32_t frame_index);
  void ResolveGpuTimestampFrame(uint32_t frame_index);
  void AccumulateGpuTimestamp(const std::string& name, double milliseconds);
  void AccumulateCpuTiming(const std::string& name, double milliseconds);

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

  /// Size of used dedicated compute command buffers.
  int used_compute_command_buffer_size_ = 0;

  /// Pool of command buffers allocated from the dedicated compute command pool.
  std::vector<std::vector<std::shared_ptr<CommandBuffer>>> compute_command_buffer_pool_ = {};

  /// Map of named buffer synchronization actions.
  std::unordered_map<std::string, std::function<void()>> buffer_sync_actions{};

  /// Temporary list of buffer synchronization actions.
  std::vector<std::function<void()>> temporary_buffer_sync_actions{};

  /**
   * @brief Global defines for shaders, set during initialization.
   */
  std::string shader_global_defines = {};
  mutable std::mutex shader_include_paths_mutex_;
  std::set<std::filesystem::path> shader_include_paths_{};

 public:
  /**
   * @brief Runtime Vulkan capabilities selected for this application.
   */
  struct Capabilities {
    bool support_mesh_shader = true;
    bool support_acceleration_structure = true;
    bool support_ray_tracing = true;
    bool support_ray_query = true;
    bool support_shader_execution_reordering = false;
    bool support_shader_float16 = false;
    bool support_ray_tracing_validation = false;
    bool support_async_compute = false;
    bool support_pipeline_creation_feedback = false;
    uint32_t subgroup_size = 1;
    uint32_t task_subgroup_count = 1;
    uint32_t task_work_group_invocations = 1;
    uint32_t mesh_subgroup_count = 1;
    uint32_t compute_subgroup_count = 1;
    uint32_t compute_work_group_invocations = 1;
    uint32_t max_compute_work_group_invocations = 1;
    uint32_t max_shared_memory_size = 1;
  };

 private:
  Capabilities capabilities_{};

 public:
  [[nodiscard]] const Capabilities& GetCapabilities() const;
  void RegisterShaderIncludePath(const std::filesystem::path& path);
  [[nodiscard]] std::set<std::filesystem::path> GetRegisteredShaderIncludePaths() const;
  [[nodiscard]] static QueueFamilySelection SelectQueueFamilies(const std::vector<QueueFamilySupport>& queue_families);

  static bool RayTracingEnabled();
  static bool RayQueryEnabled();
  static bool RayAccelerationStructureEnabled();
  static bool ShaderExecutionReorderingEnabled();
  static bool MeshShaderEnabled();
  static constexpr size_t kRenderPassDrawBucketCount = static_cast<size_t>(RenderPassDrawBucket::Count);
  [[nodiscard]] static const char* GetRenderPassDrawBucketName(RenderPassDrawBucket bucket);
  static void ResetRenderPassDrawStats(uint32_t frame_index);
  static void BeginRenderCameraDrawScope(uint32_t frame_index, uint64_t camera_handle, uint32_t entity_index,
                                         bool scene_camera);
  static void EndRenderCameraDrawScope();
  static void CountRenderPassDraw(RenderPassDrawBucket bucket, RenderDrawCallKind kind, uint32_t frame_index,
                                  size_t prim_count, size_t indirect_draw_commands = 0);
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
   * @brief Tracks whether commands recorded for the current frame are submitted or discarded.
   * @return Shared state resolved by the frame lifecycle.
   */
  [[nodiscard]] static std::shared_ptr<FrameSubmissionState> TrackCurrentFrameSubmission();

  static VkResult CreateComputePipeline(const VkComputePipelineCreateInfo& create_info, VkPipeline& pipeline,
                                        PipelineCreationFeedback& feedback);
  static VkResult CreateGraphicsPipeline(const VkGraphicsPipelineCreateInfo& create_info, VkPipeline& pipeline,
                                         PipelineCreationFeedback& feedback);
  static VkResult CreateRayTracingPipeline(const VkRayTracingPipelineCreateInfoKHR& create_info, VkPipeline& pipeline,
                                           PipelineCreationFeedback& feedback);

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
   * @brief Records commands for the dedicated compute queue, or the main queue when no dedicated queue is available.
   *
   * @param action The action to record.
   */
  static void RecordCommandsComputeQueue(const std::function<void(VkCommandBuffer vk_command_buffer)>& action);

  /**
   * @brief Records render commands with specific rendering info.
   *
   * @param rendering_info The Vulkan rendering information.
   * @param vk_command_buffer The Vulkan command buffer.
   * @param action The action to execute during rendering.
   */
  static void RecordRenderCommands(const VkRenderingInfo& rendering_info, const VkCommandBuffer vk_command_buffer,
                                   const std::function<void()>& action);

  /**
   * @brief Begins dynamic rendering through the SDK binary.
   */
  static void BeginRendering(VkCommandBuffer vk_command_buffer, const VkRenderingInfo& rendering_info);

  /**
   * @brief Ends dynamic rendering through the SDK binary.
   */
  static void EndRendering(VkCommandBuffer vk_command_buffer);

  /**
   * @brief Records an indexed draw through the SDK binary.
   */
  static void DrawIndexed(VkCommandBuffer vk_command_buffer, uint32_t index_count, uint32_t instance_count,
                          uint32_t first_index = 0, int32_t vertex_offset = 0, uint32_t first_instance = 0);

  /**
   * @brief Records an indexed indirect draw through the SDK binary.
   */
  static void DrawIndexedIndirect(VkCommandBuffer vk_command_buffer, const Buffer& buffer, VkDeviceSize offset,
                                  uint32_t draw_count, uint32_t stride);

  /**
   * @brief Records a mesh task indirect draw through the SDK binary.
   */
  static void DrawMeshTasksIndirect(VkCommandBuffer vk_command_buffer, const Buffer& buffer, VkDeviceSize offset,
                                    uint32_t draw_count, uint32_t stride);

  /**
   * @brief Clears a color image through the SDK binary.
   */
  static void ClearColorImage(VkCommandBuffer vk_command_buffer, const Image& image, const VkClearColorValue& value,
                              uint32_t range_count, const VkImageSubresourceRange* ranges);

  /**
   * @brief Clears a depth/stencil image through the SDK binary.
   */
  static void ClearDepthStencilImage(VkCommandBuffer vk_command_buffer, const Image& image,
                                     const VkClearDepthStencilValue& value, uint32_t range_count,
                                     const VkImageSubresourceRange* ranges);

  /// Time spent waiting on the CPU in seconds.
  double cpu_wait_time = 0.0f;

  /**
   * @brief Waits for the device to finish all operations.
   */
  static void WaitForDeviceIdle();

  /**
   * @brief Drains pending resource upload work and waits for GPU/device idle.
   */
  static void DrainGpuResourceWork();
  static void WaitForFrameSubmission(uint32_t frame_index, const std::string& wait_name);
  static void WaitForFrameSubmissions(const std::string& wait_name);

  /// List of primitive counts for debugging purposes.
  std::vector<size_t> prim_count{};

  /// List of draw calls for debugging purposes.
  std::vector<size_t> draw_call{};

  /// Per-pass draw call and primitive counts for debugging purposes.
  std::vector<std::array<RenderPassDrawStats, kRenderPassDrawBucketCount>> render_pass_draw_stats{};

  /// Per-camera per-pass draw call and primitive counts for debugging purposes.
  std::vector<std::vector<RenderCameraDrawStats>> render_camera_draw_stats{};

  /**
   * @brief Constants used for internal configuration and limits.
   */
  class Constants {
   public:
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

    /// Attribute format for expanded G-buffer attachments.
    constexpr static VkFormat g_buffer_attribute = VK_FORMAT_R16G16B16A16_SFLOAT;

    /// Utility format for expanded G-buffer attachments.
    constexpr static VkFormat g_buffer_utility = VK_FORMAT_R32G32B32A32_SFLOAT;

    /// Format for shadow maps.
    constexpr static VkFormat shadow_map = VK_FORMAT_D32_SFLOAT;

    /// Format for swapchain images.
    constexpr static VkFormat swap_chain_image_format = VK_FORMAT_B8G8R8A8_UNORM;

    /// Maximum number of vertices in a meshlet.
    constexpr static uint32_t meshlet_max_vertices_size = 64;

    /// Maximum number of triangles in a meshlet.
    constexpr static uint32_t meshlet_max_triangles_size = 40;
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
   * @brief Inserts a full pipeline memory barrier scoped to one buffer.
   *
   * @param vk_command_buffer Vulkan command buffer to record the barrier.
   * @param buffer Buffer to synchronize.
   */
  static void BufferMemoryBarrier(VkCommandBuffer vk_command_buffer, const Buffer& buffer,
                                  uint32_t src_queue_family_index = VK_QUEUE_FAMILY_IGNORED,
                                  uint32_t dst_queue_family_index = VK_QUEUE_FAMILY_IGNORED,
                                  bool release_barrier = false);

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
                                 uint32_t mip_levels = 1, uint32_t src_queue_family_index = VK_QUEUE_FAMILY_IGNORED,
                                 uint32_t dst_queue_family_index = VK_QUEUE_FAMILY_IGNORED,
                                 bool release_barrier = false);

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

  static void ImmediateSubmitWithGpuTimestamp(const std::string& name,
                                              const std::function<void(VkCommandBuffer vk_command_buffer)>& action);

  static void SetGpuTimestampCaptureEnabled(bool enabled);
  [[nodiscard]] static bool GpuTimestampCaptureEnabled();
  [[nodiscard]] static bool GpuTimestampCaptureAvailable();
  static void ResetGpuTimestampStats();
  [[nodiscard]] static std::vector<GpuTimestampStats> GetGpuTimestampStats();
  [[nodiscard]] static std::vector<GpuTimestampStats> GetCpuTimingStats();
  static void RecordCpuTimingSample(const std::string& name, double milliseconds);
  [[nodiscard]] static GpuMemorySnapshot GetGpuMemorySnapshot();
  [[nodiscard]] static GpuDeviceFingerprint GetGpuDeviceFingerprint();
  [[nodiscard]] static bool GraphicsValidationEnabled();
  [[nodiscard]] static GpuTimestampScopeToken BeginGpuTimestampScope(VkCommandBuffer vk_command_buffer,
                                                                     const std::string& name);
  static void EndGpuTimestampScope(VkCommandBuffer vk_command_buffer, const GpuTimestampScopeToken& token);

  /**
   * @brief Retrieves the platform-owned GPU service.
   */
  static GpuService& GetGpuService();
  static std::mutex& GetQueueHostMutex();

  /**
   * @brief Returns the platform-owned GPU service when it has been created.
   */
  [[nodiscard]] static GpuService* TryGetGpuService();

  /**
   * @brief Gets the maximum number of frames that can be in flight at any time.
   *
   * @return Maximum frames in flight.
   */
  static int GetMaxFramesInFlight();
  [[nodiscard]] static uint32_t GetPendingFrameSubmissionCount();

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
   * @brief Gets extended format features for the selected physical device.
   * @param format Vulkan format to query.
   * @return Extended format properties.
   */
  [[nodiscard]] static VkFormatProperties3 GetPhysicalDeviceFormatProperties(VkFormat format);

  /** Checks exact sampled/renderable/exportable cubemap image support for a format and layout. */
  [[nodiscard]] static bool SupportsCubemapFormat(VkFormat format, uint32_t resolution, uint32_t mip_levels);

  /**
   * @brief Gets the Vulkan logical device handle.
   *
   * @return The Vulkan logical device.
   */
  static VkDevice GetVkDevice();

  /**
   * @brief Gets the selected graphics/compute queue family index.
   */
  static uint32_t GetGraphicsAndComputeQueueFamilyIndex();

  /**
   * @brief Gets the selected compute queue family index.
   */
  static uint32_t GetComputeQueueFamilyIndex();

  /**
   * @brief Checks whether the selected device exposes an independent compute queue family.
   */
  static bool HasDedicatedComputeQueue();

  /**
   * @brief Gets unique queue families used by graphics and compute work.
   */
  static std::vector<uint32_t> GetGraphicsComputeQueueFamilyIndices();

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
   * @brief Gets the Vulkan command pool for compute commands, falling back to the graphics pool when queues are shared.
   */
  static VkCommandPool GetComputeVkCommandPool();

  /**
   * @brief Gets the main command queue.
   *
   * @return A unique pointer to the main command queue.
   */
  static const std::unique_ptr<CommandQueue>& GetMainQueue();

  /**
   * @brief Gets the optional dedicated compute queue. Returns null when compute work shares the graphics queue.
   */
  static CommandQueue* TryGetDedicatedComputeQueue();

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

 private:
  static void CountRenderPassDrawInternal(RenderPassDrawBucket bucket, RenderDrawCallKind kind, uint32_t frame_index,
                                          size_t prim_count, size_t indirect_draw_commands);

  struct PendingGpuTimestampScope {
    std::string name{};
    uint32_t begin_query = 0;
    uint32_t end_query = 0;
  };

  struct GpuTimestampFrame {
    VkQueryPool query_pool = VK_NULL_HANDLE;
    uint32_t next_query = 0;
    bool reset_recorded = false;
    bool capacity_warning_reported = false;
    std::vector<PendingGpuTimestampScope> scopes{};
  };

  static constexpr uint32_t kGpuTimestampQueriesPerFrame = 256;
  bool gpu_timestamp_capture_enabled_ = false;
  bool gpu_timestamp_capture_available_ = false;
  uint32_t gpu_timestamp_valid_bits_ = 0;
  double gpu_timestamp_period_nanoseconds_ = 0.0;
  std::vector<GpuTimestampFrame> gpu_timestamp_frames_{};
  VkQueryPool immediate_gpu_timestamp_query_pool_ = VK_NULL_HANDLE;
  std::recursive_mutex immediate_gpu_timestamp_mutex_{};
  mutable std::mutex gpu_timestamp_stats_mutex_{};
  std::unordered_map<std::string, GpuTimestampStats> gpu_timestamp_stats_{};
  mutable std::mutex cpu_timing_stats_mutex_{};
  std::unordered_map<std::string, GpuTimestampStats> cpu_timing_stats_{};
  std::optional<RenderCameraDrawScope> active_render_camera_draw_scope_{};
};
}  // namespace evo_engine
