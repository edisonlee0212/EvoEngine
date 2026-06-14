#include "WindowLayer.hpp"
#include "Application.hpp"
#include "EditorLayer.hpp"
#include "ImGuiLayer.hpp"
#include "Platform.hpp"
#include "ProjectManager.hpp"
#include "RenderLayer.hpp"
#include "Resources.hpp"
#ifdef EVOENGINE_WINDOWS
#  ifndef GLFW_EXPOSE_NATIVE_WIN32
#    define GLFW_EXPOSE_NATIVE_WIN32
#  endif
#  include <windowsx.h>
#  include "GLFW/glfw3native.h"
#endif
using namespace evo_engine;

#ifdef EVOENGINE_WINDOWS
namespace {
constexpr wchar_t kWindowLayerProperty[] = L"EvoEngineWindowLayer";
constexpr DWORD kDwmWindowCornerPreference = 33;
constexpr DWORD kDwmWindowCornerDefault = 0;
constexpr DWORD kDwmWindowCornerRound = 2;

bool Contains(const glm::vec4& region, const POINT& point) {
  return point.x >= region.x && point.x < region.x + region.z && point.y >= region.y && point.y < region.y + region.w;
}

void SetWindowCornerPreference(HWND hwnd, const DWORD preference) {
  using DwmSetWindowAttributeFunc = HRESULT(WINAPI*)(HWND, DWORD, LPCVOID, DWORD);
  const HMODULE dwmapi = LoadLibraryW(L"dwmapi.dll");
  if (!dwmapi) {
    return;
  }

  const auto set_window_attribute =
      reinterpret_cast<DwmSetWindowAttributeFunc>(GetProcAddress(dwmapi, "DwmSetWindowAttribute"));
  if (set_window_attribute) {
    set_window_attribute(hwnd, kDwmWindowCornerPreference, &preference, sizeof(preference));
  }
  FreeLibrary(dwmapi);
}

LRESULT CALLBACK CustomTitleBarWindowProc(HWND hwnd, UINT message, WPARAM w_param, LPARAM l_param) {
  const auto window_layer = static_cast<WindowLayer*>(GetPropW(hwnd, kWindowLayerProperty));
  if (window_layer) {
    return window_layer->HandleNativeWindowMessage(hwnd, message, w_param, l_param);
  }
  return DefWindowProcW(hwnd, message, w_param, l_param);
}
}  // namespace
#endif

void WindowLayer::FramebufferSizeCallback(GLFWwindow* window, int width, int height) {
  if (const auto window_layer = ApplicationContext::Get().GetLayer<WindowLayer>(); window_layer->window_ == window) {
    window_layer->window_size_ = {width, height};
  }
}

void WindowLayer::SetMonitorCallback(GLFWmonitor* monitor, int event) {
  const auto window_layer = ApplicationContext::Get().GetLayer<WindowLayer>();
  if (event == GLFW_CONNECTED) {
    // The monitor was connected
    for (const auto& i : window_layer->monitors_)
      if (i == monitor)
        return;
    window_layer->monitors_.push_back(monitor);
  } else if (event == GLFW_DISCONNECTED) {
    // The monitor was disconnected
    for (auto i = 0; i < window_layer->monitors_.size(); i++) {
      if (monitor == window_layer->monitors_[i]) {
        window_layer->monitors_.erase(window_layer->monitors_.begin() + i);
      }
    }
  }
  window_layer->primary_monitor_ = glfwGetPrimaryMonitor();
}

void WindowLayer::WindowFocusCallback(GLFWwindow* window, const int focused) {
  const auto window_layer = ApplicationContext::Get().GetLayer<WindowLayer>();

  if (focused) {
    ProjectManager::DispatchScanAssetsTask();
  }
}

void WindowLayer::OnCreate() {
  if (const auto render_layer = ApplicationContext::Get().GetLayer<RenderLayer>(); !render_layer) {
    throw std::runtime_error("RenderLayer not present!");
  }
}

void WindowLayer::OnDestroy() {
#pragma region Windows
  UninstallCustomTitleBar();
  glfwDestroyWindow(window_);
  glfwTerminate();
#pragma endregion
}

void WindowLayer::InstallCustomTitleBar() {
#ifdef EVOENGINE_WINDOWS
  if (!custom_title_bar_ || !window_) {
    return;
  }
  const auto hwnd = glfwGetWin32Window(window_);
  native_window_handle_ = hwnd;
  auto style = GetWindowLongPtrW(hwnd, GWL_STYLE);
  style &= ~(WS_CAPTION | WS_POPUP);
  style |= WS_SYSMENU | WS_MINIMIZEBOX | WS_MAXIMIZEBOX | WS_THICKFRAME;
  SetWindowLongPtrW(hwnd, GWL_STYLE, style);
  SetPropW(hwnd, kWindowLayerProperty, this);
  default_window_proc_ = SetWindowLongPtrW(hwnd, GWLP_WNDPROC, reinterpret_cast<LONG_PTR>(CustomTitleBarWindowProc));
  SetWindowPos(hwnd, nullptr, 0, 0, 0, 0, SWP_NOMOVE | SWP_NOSIZE | SWP_NOZORDER | SWP_NOACTIVATE | SWP_FRAMECHANGED);
  SetWindowCornerPreference(hwnd, kDwmWindowCornerRound);
#endif
}

void WindowLayer::UninstallCustomTitleBar() {
#ifdef EVOENGINE_WINDOWS
  const auto hwnd = static_cast<HWND>(native_window_handle_);
  if (!hwnd) {
    return;
  }
  if (default_window_proc_ &&
      GetWindowLongPtrW(hwnd, GWLP_WNDPROC) == reinterpret_cast<LONG_PTR>(CustomTitleBarWindowProc)) {
    SetWindowLongPtrW(hwnd, GWLP_WNDPROC, default_window_proc_);
  }
  SetWindowCornerPreference(hwnd, kDwmWindowCornerDefault);
  RemovePropW(hwnd, kWindowLayerProperty);
#endif
  native_window_handle_ = nullptr;
  default_window_proc_ = 0;
  has_title_bar_drag_region_ = false;
}

std::optional<intptr_t> WindowLayer::HitTestCustomTitleBar(void* native_window_handle, const intptr_t l_param) const {
#ifdef EVOENGINE_WINDOWS
  if (!custom_title_bar_) {
    return {};
  }
  const auto hwnd = static_cast<HWND>(native_window_handle);
  POINT point{GET_X_LPARAM(l_param), GET_Y_LPARAM(l_param)};
  const bool maximized = IsZoomed(hwnd);
  RECT window_rect{};
  GetWindowRect(hwnd, &window_rect);

  if (!maximized) {
    const int resize_border = std::max(6, GetSystemMetrics(SM_CXFRAME) + GetSystemMetrics(SM_CXPADDEDBORDER));
    const int x = point.x - window_rect.left;
    const int y = point.y - window_rect.top;
    const int width = window_rect.right - window_rect.left;
    const int height = window_rect.bottom - window_rect.top;
    const bool left = x < resize_border;
    const bool right = x >= width - resize_border;
    const bool top = y < resize_border;
    const bool bottom = y >= height - resize_border;

    if (top && left)
      return HTTOPLEFT;
    if (top && right)
      return HTTOPRIGHT;
    if (bottom && left)
      return HTBOTTOMLEFT;
    if (bottom && right)
      return HTBOTTOMRIGHT;
    if (top)
      return HTTOP;
    if (bottom)
      return HTBOTTOM;
    if (left)
      return HTLEFT;
    if (right)
      return HTRIGHT;
  }

  ScreenToClient(hwnd, &point);
  if (has_title_bar_drag_region_ && Contains(title_bar_drag_region_, point)) {
    return HTCAPTION;
  }
#endif
  return {};
}

intptr_t WindowLayer::HandleNativeWindowMessage(void* native_window_handle, const unsigned int message,
                                                const uintptr_t w_param, const intptr_t l_param) const {
#ifdef EVOENGINE_WINDOWS
  const auto hwnd = static_cast<HWND>(native_window_handle);
  switch (message) {
    case WM_NCCALCSIZE:
      if (w_param == TRUE) {
        auto* params = reinterpret_cast<NCCALCSIZE_PARAMS*>(l_param);
        if (IsZoomed(hwnd)) {
          MONITORINFO monitor_info{sizeof(monitor_info)};
          GetMonitorInfoW(MonitorFromWindow(hwnd, MONITOR_DEFAULTTONEAREST), &monitor_info);
          params->rgrc[0] = monitor_info.rcWork;
        }
        return 0;
      }
      break;
    case WM_NCHITTEST:
      if (const auto hit = HitTestCustomTitleBar(hwnd, l_param)) {
        return *hit;
      }
      break;
    default:
      break;
  }
  if (default_window_proc_) {
    return CallWindowProcW(reinterpret_cast<WNDPROC>(default_window_proc_), hwnd, message, w_param, l_param);
  }
  return DefWindowProcW(hwnd, message, w_param, l_param);
#else
  return 0;
#endif
}

void WindowLayer::Render() {
  if (const auto imgui_layer = ApplicationContext::Get().GetLayer<ImGuiLayer>()) {
    Platform::RecordCommandsMainQueue([&](const VkCommandBuffer vk_command_buffer) {
      Platform::EverythingBarrier(vk_command_buffer);
      Platform::TransitImageLayout(vk_command_buffer, Platform::GetSwapchain()->GetVkImage(),
                                   Platform::GetSwapchain()->GetImageFormat(), 1, VK_IMAGE_LAYOUT_UNDEFINED,
                                   VK_IMAGE_LAYOUT_ATTACHMENT_OPTIMAL_KHR);

      constexpr VkClearValue clear_color = {{{0.0f, 0.0f, 0.0f, 1.0f}}};
      VkRect2D render_area;
      render_area.offset = {0, 0};
      render_area.extent = Platform::GetSwapchain()->GetImageExtent();

      VkRenderingAttachmentInfo color_attachment_info{};
      color_attachment_info.sType = VK_STRUCTURE_TYPE_RENDERING_ATTACHMENT_INFO;
      color_attachment_info.imageView = Platform::GetSwapchain()->GetVkImageView();
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

      Platform::BeginRendering(vk_command_buffer, render_info);
      ImGui::Render();
      ImGui_ImplVulkan_RenderDrawData(ImGui::GetDrawData(), vk_command_buffer);
      Platform::EndRendering(vk_command_buffer);
      Platform::TransitImageLayout(vk_command_buffer, Platform::GetSwapchain()->GetVkImage(),
                                   Platform::GetSwapchain()->GetImageFormat(), 1,
                                   VK_IMAGE_LAYOUT_ATTACHMENT_OPTIMAL_KHR, VK_IMAGE_LAYOUT_PRESENT_SRC_KHR);
    });

    if (const ImGuiIO& io = ImGui::GetIO(); io.ConfigFlags & ImGuiConfigFlags_ViewportsEnable) {
      ImGui::UpdatePlatformWindows();
      ImGui::RenderPlatformWindowsDefault();
    }
  } else {
    const auto& graphics = Platform::GetInstance();
    if (const auto scene = ApplicationContext::Get().GetActiveScene()) {
      if (const auto main_camera = scene->main_camera.Get<Camera>();
          main_camera->IsEnabled() && main_camera->Rendered()) {
        const auto swapchain = Platform::GetSwapchain();
        const auto& render_texture_present = graphics.render_texture_present_pipeline;
        Platform::RecordCommandsMainQueue([&](const VkCommandBuffer vk_command_buffer) {
          Platform::EverythingBarrier(vk_command_buffer);
          Platform::TransitImageLayout(vk_command_buffer, swapchain->GetVkImage(), swapchain->GetImageFormat(), 1,
                                       VK_IMAGE_LAYOUT_UNDEFINED, VK_IMAGE_LAYOUT_ATTACHMENT_OPTIMAL_KHR);

          constexpr VkClearValue clear_color = {{{0.0f, 0.0f, 0.0f, 1.0f}}};
          VkRect2D render_area;
          render_area.offset = {0, 0};
          render_area.extent = swapchain->GetImageExtent();

          VkRenderingAttachmentInfo color_attachment_info{};
          color_attachment_info.sType = VK_STRUCTURE_TYPE_RENDERING_ATTACHMENT_INFO;
          color_attachment_info.imageView = swapchain->GetVkImageView();
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
          viewport.width = static_cast<float>(render_area.extent.width);
          viewport.height = static_cast<float>(render_area.extent.height);
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
          Platform::BeginRendering(vk_command_buffer, render_info);
          // From main camera to swap chain.
          render_texture_present->Bind(vk_command_buffer);
          render_texture_present->BindDescriptorSet(
              vk_command_buffer, 0,
              main_camera->GetRenderTexture()->color_present_descriptor_set_->GetVkDescriptorSet());

          const auto mesh = Resources::GetInstance().GetTexturePassThroughQuad();
          GeometryStorage::BindVertices(vk_command_buffer);
          mesh->DrawIndexed(vk_command_buffer, render_texture_present->states, 1);
          Platform::EndRendering(vk_command_buffer);
          Platform::TransitImageLayout(vk_command_buffer, swapchain->GetVkImage(), swapchain->GetImageFormat(), 1,
                                       VK_IMAGE_LAYOUT_ATTACHMENT_OPTIMAL_KHR, VK_IMAGE_LAYOUT_PRESENT_SRC_KHR);
        });
      }
    }
  }
}

GLFWwindow* WindowLayer::GetGlfwWindow() const {
  return window_;
}

bool WindowLayer::UsesCustomTitleBar() const {
  return custom_title_bar_;
}

bool WindowLayer::IsWindowMaximized() const {
  return window_ && glfwGetWindowAttrib(window_, GLFW_MAXIMIZED) == GLFW_TRUE;
}

void WindowLayer::MinimizeWindow() const {
  glfwIconifyWindow(window_);
}

void WindowLayer::ToggleMaximized() const {
  if (IsWindowMaximized()) {
    glfwRestoreWindow(window_);
  } else {
    glfwMaximizeWindow(window_);
  }
}

void WindowLayer::SetCustomTitleBarDragRegion(const glm::vec4& region) {
  title_bar_drag_region_ = region;
  has_title_bar_drag_region_ = region.z > 0.0f && region.w > 0.0f;
}

void WindowLayer::ClearCustomTitleBarDragRegion() {
  has_title_bar_drag_region_ = false;
}

void WindowLayer::ResizeWindow(int x, int y) const {
  glfwSetWindowSize(window_, x, y);
}
