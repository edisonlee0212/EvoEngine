#include "ImGuiLayer.hpp"

#include "Application.hpp"
#include "EditorTextureRegistry.hpp"
#include "Platform.hpp"
#include "WindowLayer.hpp"

using namespace evo_engine;

void ImGuiLayer::OnDestroy() {
  if (const auto window = ApplicationContext::Get().GetLayer<WindowLayer>())
    window->SetPresentationCallbacks({});
  if (!ImGui::GetCurrentContext())
    return;

  ImGui_ImplVulkan_Shutdown();
  ImGui_ImplGlfw_Shutdown();
  ImNodes::DestroyContext();
  ImGui::DestroyContext();
}

void ImGuiLayer::PreUpdate() {
  EditorTextureRegistry::CollectGarbage();
  {
    const std::lock_guard queue_lock(Platform::GetQueueHostMutex());
    ImGui_ImplVulkan_NewFrame();
  }
  ImGui_ImplGlfw_NewFrame();
  ImGui::NewFrame();
  ImGuizmo::BeginFrame();
}

void ImGuiLayer::OnWindowGraphicsInitialized() {
  const auto window_layer = ApplicationContext::Get().GetLayer<WindowLayer>();
  // Setup Dear ImGui context

  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  ImNodes::CreateContext();
  ImGuiIO& io = ImGui::GetIO();
  if (const char* path = std::getenv("EVOENGINE_IMGUI_INI_PATH"); path && path[0] != '\0') {
    static std::string imgui_ini_path;
    imgui_ini_path = path;
    io.IniFilename = imgui_ini_path.c_str();
  }
  if (ApplicationContext::Get().GetApplicationInfo().enable_docking) {
    io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;
  }
  if (ApplicationContext::Get().GetApplicationInfo().enable_viewport) {
    io.ConfigFlags |= ImGuiConfigFlags_ViewportsEnable;
    io.ConfigFlags |= ImGuiConfigFlags_DpiEnableScaleViewports;
  }
  io.ConfigFlags |= ImGuiConfigFlags_DpiEnableScaleFonts;
  //  io.ConfigFlags |= ImGuiConfigFlags_IsSRGB;
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
  init_info.Instance = Platform::GetVkInstance();
  init_info.PhysicalDevice = Platform::GetSelectedPhysicalDevice()->vk_physical_device;
  init_info.Device = Platform::GetVkDevice();
  init_info.QueueFamily = Platform::GetGraphicsAndComputeQueueFamilyIndex();
  init_info.Queue = Platform::GetMainQueue()->GetVkQueue();
  init_info.PipelineCache = VK_NULL_HANDLE;
  init_info.DescriptorPoolSize = Platform::Constants::initial_descriptor_pool_max_size;
  init_info.MinImageCount = Platform::GetSwapchain()->GetAllImageViews().size();
  init_info.ImageCount = Platform::GetSwapchain()->GetAllImageViews().size();
  init_info.PipelineInfoMain.MSAASamples = VK_SAMPLE_COUNT_1_BIT;
  init_info.UseDynamicRendering = true;
  init_info.PipelineInfoMain.PipelineRenderingCreateInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_RENDERING_CREATE_INFO_KHR;
  init_info.PipelineInfoMain.PipelineRenderingCreateInfo.colorAttachmentCount = 1;
  init_info.PipelineInfoMain.PipelineRenderingCreateInfo.pColorAttachmentFormats =
      &Platform::Constants::swap_chain_image_format;
  init_info.PipelineInfoMain.PipelineRenderingCreateInfo.pNext = nullptr;

  ImGui_ImplVulkan_LoadFunctions(VK_API_VERSION_1_3, [](const char* function_name, void*) {
    return Platform::GetVulkanInstanceFunction(function_name);
  });
  ImGui_ImplVulkan_Init(&init_info);
  window_layer->SetPresentationCallbacks(
      [](VkCommandBuffer commands) {
        ImGui::Render();
        ImGui_ImplVulkan_RenderDrawData(ImGui::GetDrawData(), commands);
      },
      [] {
        if (ImGui::GetIO().ConfigFlags & ImGuiConfigFlags_ViewportsEnable) {
          const std::lock_guard queue_lock(Platform::GetQueueHostMutex());
          ImGui::UpdatePlatformWindows();
          ImGui::RenderPlatformWindowsDefault();
        }
      });
}
