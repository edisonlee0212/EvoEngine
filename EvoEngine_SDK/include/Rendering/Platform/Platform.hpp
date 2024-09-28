#pragma once
#include "ComputePipeline.hpp"
#include "GraphicsPipeline.hpp"
#include "GraphicsResources.hpp"
#include "ISingleton.hpp"
#include "RayTracingPipeline.hpp"
#define ENABLE_EXTERNAL_MEMORY true
#define ENABLE_NV_RAY_TRACING_VALIDATION false
#define GRAPHICS_VALIDATION true
#define USE_NSIGHT_AFTERMATH true
// Enables the Nsight Aftermath code instrumentation for GPU crash dump creation.

#if USE_NSIGHT_AFTERMATH
#  include "NsightAftermathGpuCrashTracker.h"
#  include "NsightAftermathHelpers.h"
#  include "NsightAftermathShaderDatabase.h"
#endif

namespace evo_engine {

class Platform final {
  EVOENGINE_SINGLETON_INSTANCE(Platform)
  friend class Application;
  friend class Resources;
  friend class Lighting;
  friend class PointLightShadowMap;
  friend class SpotLightShadowMap;
#pragma region Vulkan
  VkInstance vk_instance_ = VK_NULL_HANDLE;

  std::vector<std::string> required_layers_ = {};
  std::vector<VkLayerProperties> vk_supported_layers_;

  std::vector<std::string> required_instance_extension_names_ = {};
  std::unordered_map<std::string, VkExtensionProperties> vk_supported_instance_extensions_;

  std::vector<std::string> required_device_extension_names_ = {};

  struct PhysicalDevice {
    VkPhysicalDevice vk_physical_device{};
    std::unordered_map<std::string, VkExtensionProperties> supported_vk_extensions{};
    VkPhysicalDeviceProperties properties{};

    VkPhysicalDeviceProperties2 properties2 = {VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_PROPERTIES_2};
    VkPhysicalDeviceVulkan11Properties vulkan11_properties{VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_VULKAN_1_1_PROPERTIES};
    VkPhysicalDeviceVulkan12Properties vulkan12_properties{VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_VULKAN_1_2_PROPERTIES};
    VkPhysicalDeviceMeshShaderPropertiesEXT mesh_shader_properties_ext = {
        VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_MESH_SHADER_PROPERTIES_EXT};
    VkPhysicalDeviceSubgroupSizeControlProperties subgroup_size_control_properties = {
        VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_SUBGROUP_SIZE_CONTROL_PROPERTIES};
    VkPhysicalDeviceRayTracingPipelinePropertiesKHR ray_tracing_properties_ext = {
        VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_RAY_TRACING_PIPELINE_PROPERTIES_KHR};
#if ENABLE_NV_RAY_TRACING_VALIDATION
    VkPhysicalDeviceRayTracingValidationFeaturesNV ray_tracing_validation_features_nv = {
        VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_RAY_TRACING_VALIDATION_FEATURES_NV};
#endif


    VkPhysicalDeviceMemoryProperties vk_physical_device_memory_properties = {};

    VkPhysicalDeviceFeatures features{};
    VkPhysicalDeviceAccelerationStructureFeaturesKHR acceleration_structure_features{};

    struct QueueFamilyIndices {
      std::optional<uint32_t> graphics_and_compute_family;
      std::optional<uint32_t> present_family;
      [[nodiscard]] bool IsComplete() const;
    };

    struct SwapChainSupportDetails {
      VkSurfaceCapabilitiesKHR capabilities;
      std::vector<VkSurfaceFormatKHR> formats;
      std::vector<VkPresentModeKHR> present_modes;
    };

    QueueFamilyIndices queue_family_indices = {};
    SwapChainSupportDetails swap_chain_support_details{};

    uint32_t score = 0;
    void QueryInformation();
    void QuerySwapChainSupport();

    [[nodiscard]] bool CheckExtensionSupport(const std::string& required_extension_name) const;
    [[nodiscard]] uint32_t FindMemoryType(uint32_t type_filter, VkMemoryPropertyFlags properties) const;
    [[nodiscard]] bool Suitable(const std::vector<std::string>& required_extension_names) const;
  };
  VkDebugUtilsMessengerEXT vk_debug_messenger_ = {};

  std::vector<std::shared_ptr<PhysicalDevice>> physical_devices_{};
  std::shared_ptr<PhysicalDevice> selected_physical_device{};

#if USE_NSIGHT_AFTERMATH
  GpuCrashTracker::MarkerMap markerMap;
  GpuCrashTracker gpu_crash_tracker{markerMap};
#endif

  VkSurfaceKHR vk_surface_ = VK_NULL_HANDLE;

  VkDevice vk_device_ = VK_NULL_HANDLE;

  VmaAllocator vma_allocator_ = VK_NULL_HANDLE;

  std::unique_ptr<CommandQueue> immediate_submit_queue_{};

  std::unique_ptr<CommandQueue> main_queue_{};
  std::unique_ptr<CommandQueue> present_queue_{};

  std::shared_ptr<Swapchain> swapchain_ = {};

  VkSurfaceFormatKHR vk_surface_format_ = {};

#pragma endregion
#pragma region Internals
  std::unique_ptr<CommandPool> command_pool_ = {};
  std::unique_ptr<DescriptorPool> descriptor_pool_ = {};

  int max_frame_in_flight_ = 2;

  std::vector<std::shared_ptr<Semaphore>> image_available_semaphores_ = {};
  std::vector<std::shared_ptr<Semaphore>> render_finished_semaphores_ = {};
  std::vector<std::shared_ptr<Fence>> in_flight_fences_ = {};

  uint32_t current_frame_index_ = 0;

  uint32_t next_image_index_ = 0;

#pragma endregion
#pragma region Shader related
  std::string shader_skybox_;
  size_t max_bone_amount_ = 65536;
  size_t max_shadow_cascade_amount_ = 4;
  friend class RenderLayer;

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

  void OnDestroy();
  void SwapChainSwapImage();
  void SubmitPresent();
  void WaitForCommandsComplete() const;
  void Submit();

  void ResetCommandBuffers();
  static void Initialize();
  static void PostResourceLoadingInitialization();
  static void Destroy();
  static void PreUpdate();
  static void LateUpdate();

  bool recreate_swap_chain_ = false;
  unsigned swapchain_version_ = 0;

  std::unordered_map<std::string, std::shared_ptr<GraphicsPipeline>> graphics_pipelines_;
  std::unordered_map<std::string, std::shared_ptr<RayTracingPipeline>> ray_tracing_pipelines_;
  std::unordered_map<std::string, std::shared_ptr<ComputePipeline>> compute_pipelines_;

  std::shared_ptr<GraphicsPipeline> bound_graphics_pipeline;
  std::shared_ptr<ComputePipeline> bound_compute_pipeline;

  std::unordered_map<std::string, std::shared_ptr<DescriptorSetLayout>> descriptor_set_layouts_;
  void CreateGraphicsPipelines() const;
  static void PrepareDescriptorSetLayouts();
  int used_command_buffer_size_ = 0;
  std::vector<std::vector<std::shared_ptr<CommandBuffer>>> command_buffer_pool_ = {};
  std::shared_ptr<CommandBuffer> immediate_submit_command_buffer;

 public:
  static void RecordCommandsMainQueue(const std::function<void(VkCommandBuffer vk_command_buffer)>& action);

  double cpu_wait_time = 0.0f;
  static void WaitForDeviceIdle();

  static void RegisterGraphicsPipeline(const std::string& name,
                                       const std::shared_ptr<GraphicsPipeline>& graphics_pipeline);
  static void RegisterComputePipeline(const std::string& name,
                                      const std::shared_ptr<ComputePipeline>& compute_pipeline);
  static void RegisterRayTracingPipeline(const std::string& name,
                                      const std::shared_ptr<RayTracingPipeline>& ray_tracing_pipeline);
  [[nodiscard]] static const std::shared_ptr<GraphicsPipeline>& GetGraphicsPipeline(const std::string& name);
  [[nodiscard]] static const std::shared_ptr<ComputePipeline>& GetComputePipeline(const std::string& name);
  [[nodiscard]] static const std::shared_ptr<RayTracingPipeline>& GetRayTracingPipeline(const std::string& name);

  static void RegisterDescriptorSetLayout(const std::string& name,
                                          const std::shared_ptr<DescriptorSetLayout>& descriptor_set_layout);
  [[nodiscard]] static const std::shared_ptr<DescriptorSetLayout>& GetDescriptorSetLayout(const std::string& name);

  std::vector<size_t> triangles;
  std::vector<size_t> strands_segments;
  std::vector<size_t> draw_call;

  class Settings {
   public:
    inline static bool use_mesh_shader = false;
    inline static uint32_t directional_light_shadow_map_resolution = 2048;
    inline static uint32_t point_light_shadow_map_resolution = 1024;
    inline static uint32_t spot_light_shadow_map_resolution = 1024;
    inline static uint32_t max_texture_2d_resource_size = 2048;
    inline static uint32_t max_cubemap_resource_size = 256;

    inline static uint32_t max_directional_light_size = 16;
    inline static uint32_t max_point_light_size = 16;
    inline static uint32_t max_spot_light_size = 16;
  };

  class Constants {
   public:
    inline static bool support_mesh_shader = true;
    inline static bool support_ray_tracing = true;
    inline static bool support_ray_tracing_validation = true;
    constexpr static uint32_t initial_descriptor_pool_max_size = 16384;
    constexpr static uint32_t initial_descriptor_pool_max_sets = 16384;
    constexpr static uint32_t initial_camera_size = 1;
    constexpr static uint32_t initial_material_size = 1;
    constexpr static uint32_t initial_instance_size = 1;
    constexpr static uint32_t initial_render_task_size = 1;
    constexpr static uint32_t max_kernel_amount = 64;

    constexpr static VkFormat texture_2d = VK_FORMAT_R32G32B32A32_SFLOAT;
    constexpr static VkFormat render_texture_depth = VK_FORMAT_D32_SFLOAT;
    constexpr static VkFormat render_texture_color = VK_FORMAT_R32G32B32A32_SFLOAT;
    constexpr static VkFormat g_buffer_depth = VK_FORMAT_D32_SFLOAT;
    constexpr static VkFormat g_buffer_color = VK_FORMAT_R16G16B16A16_SFLOAT;
    constexpr static VkFormat g_buffer_material = VK_FORMAT_R16G16B16A16_SFLOAT;
    constexpr static VkFormat shadow_map = VK_FORMAT_D32_SFLOAT;
    constexpr static uint32_t meshlet_max_vertices_size = 64;
    constexpr static uint32_t meshlet_max_triangles_size = 40;
  };
  static uint32_t DivUp(uint32_t a, uint32_t b);
  static void EverythingBarrier(VkCommandBuffer vk_command_buffer);

  static void TransitImageLayout(VkCommandBuffer vk_command_buffer, VkImage target_image, VkFormat image_format,
                                 uint32_t layer_count, VkImageLayout old_layout, VkImageLayout new_layout,
                                 uint32_t mip_levels = 1);

  static std::string StringifyResultVk(const VkResult& result);
  static void CheckVk(const VkResult& result);

  static size_t GetMaxBoneAmount();
  static size_t GetMaxShadowCascadeAmount();
  static void ImmediateSubmit(const std::function<void(VkCommandBuffer vk_command_buffer)>& action);
  static int GetMaxFramesInFlight();
  static void NotifyRecreateSwapChain();
  static VkInstance GetVkInstance();
  static const std::shared_ptr<PhysicalDevice>& GetSelectedPhysicalDevice();
  static VkDevice GetVkDevice();
  static uint32_t GetCurrentFrameIndex();
  static uint32_t GetNextImageIndex();
  static VkCommandPool GetVkCommandPool();
  static const std::unique_ptr<CommandQueue>& GetMainQueue();
  static const std::unique_ptr<CommandQueue>& GetImmediateSubmitQueue();
  static const std::unique_ptr<CommandQueue>& GetPresentQueue();
  static VmaAllocator GetVmaAllocator();
  static const std::shared_ptr<Swapchain>& GetSwapchain();
  static const std::unique_ptr<DescriptorPool>& GetDescriptorPool();
  static unsigned GetSwapchainVersion();
  static VkSurfaceFormatKHR GetVkSurfaceFormat();
  [[nodiscard]] static bool CheckExtensionSupport(const std::string& extension_name);
  [[nodiscard]] static bool CheckLayerSupport(const std::string& layer_name);
};
}  // namespace evo_engine
