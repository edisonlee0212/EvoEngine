#include "ImGuiLayer.hpp"
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_vulkan.h>
#include "Camera.hpp"
#include "RenderTexture.hpp"
#include "Scene.hpp"

#include "Application.hpp"
#include "GuiTextureRegistry.hpp"
#include "Platform.hpp"
#include "RuntimeGuiLayer.hpp"
#include "RuntimeGuiProof.hpp"
#include "WindowLayer.hpp"

using namespace evo_engine;

void ImGuiLayer::OnDestroy() {
  if (const auto window = ApplicationContext::Get().GetLayer<WindowLayer>())
    window->SetPresentationCallbacks({});
  if (!ImGui::GetCurrentContext())
    return;

  Platform::WaitForFrameSubmissions("Runtime GUI shutdown");
  GuiTextureRegistry::CollectGarbage();
  runtime_gui_proof_.reset();
  ImGui_ImplVulkan_Shutdown();
  ImGui_ImplGlfw_Shutdown();
  ImGui::DestroyContext();
}

void ImGuiLayer::PreUpdate() {
  if (!ImGui::GetCurrentContext())
    return;
  GuiTextureRegistry::CollectGarbage();
  {
    const std::lock_guard queue_lock(Platform::GetQueueHostMutex());
    ImGui_ImplVulkan_NewFrame();
  }
  ImGui_ImplGlfw_NewFrame();
  if (runtime_gui_proof_)
    runtime_gui_proof_->PrepareFrame();
  if (const auto runtime = GetApplication().GetLayer<RuntimeGuiLayer>())
    runtime->PrepareFrame();
  ImGui::NewFrame();
  if (GetApplication().GetApplicationInfo().application_mode == ApplicationMode::Player)
    DrawRuntimeView();
}

void ImGuiLayer::OnWindowGraphicsInitialized() {
  const auto window_layer = ApplicationContext::Get().GetLayer<WindowLayer>();
  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  if (const char* proof = std::getenv("EVOENGINE_RUNTIME_GUI_PROOF"); proof && std::string_view(proof) == "1")
    runtime_gui_proof_ = std::make_shared<RuntimeGuiProof>();
  ImGuiIO& io = ImGui::GetIO();
  if (GetApplication().GetApplicationInfo().application_mode != ApplicationMode::Editor)
    io.IniFilename = nullptr;
  if (const char* path = std::getenv("EVOENGINE_IMGUI_INI_PATH"); path && path[0] != '\0') {
    static std::string imgui_ini_path;
    imgui_ini_path = path;
    io.IniFilename = imgui_ini_path.c_str();
  }
  if (GetApplication().GetApplicationInfo().application_mode == ApplicationMode::Editor &&
      GetApplication().GetApplicationInfo().enable_docking) {
    io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;
  }
  if (GetApplication().GetApplicationInfo().application_mode == ApplicationMode::Editor &&
      GetApplication().GetApplicationInfo().enable_viewport) {
    io.ConfigFlags |= ImGuiConfigFlags_ViewportsEnable;
    io.ConfigFlags |= ImGuiConfigFlags_DpiEnableScaleViewports;
  }
  io.ConfigFlags |= ImGuiConfigFlags_DpiEnableScaleFonts;
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
  // Overlay and editor each advance the backend's per-viewport upload ring.
  init_info.ImageCount *= runtime_gui_proof_ ? 3 : 2;
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
        ImGui_ImplVulkan_RenderDrawData(ImGui::GetDrawData(), commands);
      },
      [] {
        if (ImGui::GetIO().ConfigFlags & ImGuiConfigFlags_ViewportsEnable) {
          const std::lock_guard queue_lock(Platform::GetQueueHostMutex());
          ImGui::RenderPlatformWindowsDefault();
        }
      },
      [this] {
        ImGui::Render();
        if (ImGui::GetIO().ConfigFlags & ImGuiConfigFlags_ViewportsEnable)
          ImGui::UpdatePlatformWindows();
        if (runtime_gui_proof_)
          runtime_gui_proof_->RenderOverlay();
        if (const auto runtime = GetApplication().GetLayer<RuntimeGuiLayer>())
          runtime->RenderOverlay();
      });
}

void ImGuiLayer::DrawRuntimeView() {
  const auto scene = GetApplication().GetActiveScene();
  const auto camera = scene ? scene->main_camera.Get<Camera>() : nullptr;
  const auto* viewport = ImGui::GetMainViewport();
  ImGui::SetNextWindowPos(viewport->Pos);
  ImGui::SetNextWindowSize(viewport->Size);
  ImGui::SetNextWindowViewport(viewport->ID);
  ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0, 0));
  ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0);
  ImGui::Begin("##RuntimeCamera", nullptr,
               ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoInputs | ImGuiWindowFlags_NoSavedSettings |
                   ImGuiWindowFlags_NoDocking | ImGuiWindowFlags_NoBringToFrontOnFocus);
  ImGui::PopStyleVar(2);
  if (camera && camera->IsEnabled()) {
    ImGui::Image(GuiTextureRegistry::GetColorTextureId(*camera->GetRenderTexture()), viewport->Size, {0, 1}, {1, 0});
    if (runtime_gui_proof_)
      runtime_gui_proof_->Draw(viewport->Pos, viewport->Size);
    if (const auto runtime = GetApplication().GetLayer<RuntimeGuiLayer>())
      runtime->DrawView(camera, viewport->Pos, viewport->Size);
  }
  ImGui::End();
  const auto runtime = GetApplication().GetLayer<RuntimeGuiLayer>();
  const bool mouse =
      (runtime && runtime->CapturesMouse()) || (runtime_gui_proof_ && runtime_gui_proof_->CapturesMouse());
  const bool keyboard =
      (runtime && runtime->CapturesKeyboard()) || (runtime_gui_proof_ && runtime_gui_proof_->CapturesKeyboard());
  const auto window = GetApplication().GetLayer<WindowLayer>()->GetGlfwWindow();
  const bool focused = glfwGetWindowAttrib(window, GLFW_FOCUSED) != 0;
  std::vector<Input::InputEvent> scene_events;
  for (const auto& event : gameplay_events_) {
    const bool release = event.key_action == Input::KeyActionType::Release;
    if (!release && (!focused || (event.key <= GLFW_MOUSE_BUTTON_LAST ? mouse : keyboard)))
      continue;
    const bool consumed = ILayer::OnInputEvent(event);
    if (!consumed || release)
      scene_events.push_back(event);
  }
  Input::ApplyGameplayEvents(scene_events, focused, mouse, keyboard);
  gameplay_events_.clear();
}

bool ImGuiLayer::OnInputEvent(const Input::InputEvent& event) {
  if (GetApplication().GetApplicationInfo().application_mode == ApplicationMode::Player) {
    gameplay_events_.push_back(event);
    return true;
  }
  return ILayer::OnInputEvent(event);
}
