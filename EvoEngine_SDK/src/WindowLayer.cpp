#include "WindowLayer.hpp"
#include "Application.hpp"
#include "EditorLayer.hpp"
#include "ImGuiLayer.hpp"
#include "Platform.hpp"
#include "ProjectManager.hpp"
#include "RenderLayer.hpp"
#include "Resources.hpp"
#include "Texture2D.hpp"
#ifdef EVOENGINE_WINDOWS
#  ifndef GLFW_EXPOSE_NATIVE_WIN32
#    define GLFW_EXPOSE_NATIVE_WIN32
#  endif
#  include <dwmapi.h>
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

void ExtendFrameIntoClientArea(HWND hwnd) {
  using DwmExtendFrameIntoClientAreaFunc = HRESULT(WINAPI*)(HWND, const MARGINS*);
  const HMODULE dwmapi = LoadLibraryW(L"dwmapi.dll");
  if (!dwmapi) {
    return;
  }

  const auto extend_frame =
      reinterpret_cast<DwmExtendFrameIntoClientAreaFunc>(GetProcAddress(dwmapi, "DwmExtendFrameIntoClientArea"));
  if (extend_frame) {
    const MARGINS margins{0, 0, 0, 0};
    extend_frame(hwnd, &margins);
  }
  FreeLibrary(dwmapi);
}

void ApplyCustomTitleBarStyle(HWND hwnd, const bool resizable) {
  auto style = GetWindowLongPtrW(hwnd, GWL_STYLE);
  style &= ~WS_POPUP;
  style |= WS_CAPTION | WS_SYSMENU | WS_MINIMIZEBOX;
  if (resizable) {
    style |= WS_MAXIMIZEBOX | WS_THICKFRAME;
  } else {
    style &= ~(WS_MAXIMIZEBOX | WS_THICKFRAME);
  }
  SetWindowLongPtrW(hwnd, GWL_STYLE, style);
  ExtendFrameIntoClientArea(hwnd);
  SetWindowPos(hwnd, nullptr, 0, 0, 0, 0, SWP_NOMOVE | SWP_NOSIZE | SWP_NOZORDER | SWP_NOACTIVATE | SWP_FRAMECHANGED);
  SetWindowCornerPreference(hwnd, kDwmWindowCornerRound);
}

void ApplyMaximizedWorkArea(HWND hwnd, LPARAM l_param) {
  auto* min_max_info = reinterpret_cast<MINMAXINFO*>(l_param);
  MONITORINFO monitor_info{sizeof(monitor_info)};
  if (!GetMonitorInfoW(MonitorFromWindow(hwnd, MONITOR_DEFAULTTONEAREST), &monitor_info)) {
    return;
  }

  const RECT& work_area = monitor_info.rcWork;
  const RECT& monitor_area = monitor_info.rcMonitor;
  min_max_info->ptMaxPosition.x = work_area.left - monitor_area.left;
  min_max_info->ptMaxPosition.y = work_area.top - monitor_area.top;
  min_max_info->ptMaxSize.x = work_area.right - work_area.left;
  min_max_info->ptMaxSize.y = work_area.bottom - work_area.top;
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

namespace {
bool IsBgrSwapchainFormat(const VkFormat format) {
  return format == VK_FORMAT_B8G8R8A8_UNORM || format == VK_FORMAT_B8G8R8A8_SRGB;
}

void CopySwapchainImageToBuffer(const VkCommandBuffer vk_command_buffer, const std::shared_ptr<Swapchain>& swapchain,
                                const Buffer& readback_buffer) {
  VkBufferImageCopy image_copy_info{};
  image_copy_info.bufferOffset = 0;
  image_copy_info.bufferRowLength = 0;
  image_copy_info.bufferImageHeight = 0;
  image_copy_info.imageSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
  image_copy_info.imageSubresource.mipLevel = 0;
  image_copy_info.imageSubresource.baseArrayLayer = 0;
  image_copy_info.imageSubresource.layerCount = 1;
  image_copy_info.imageOffset = {0, 0, 0};
  image_copy_info.imageExtent = {swapchain->GetImageExtent().width, swapchain->GetImageExtent().height, 1};
  vkCmdCopyImageToBuffer(vk_command_buffer, swapchain->GetVkImage(), VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,
                         readback_buffer.GetVkBuffer(), 1, &image_copy_info);
}
}  // namespace

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

  if (focused && window_layer) {
    window_layer->RefreshCustomTitleBar();
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
  SetPropW(hwnd, kWindowLayerProperty, this);
  const auto current_window_proc = GetWindowLongPtrW(hwnd, GWLP_WNDPROC);
  if (current_window_proc != reinterpret_cast<LONG_PTR>(CustomTitleBarWindowProc)) {
    if (!default_window_proc_ || current_window_proc != default_window_proc_) {
      default_window_proc_ = current_window_proc;
    }
    SetWindowLongPtrW(hwnd, GWLP_WNDPROC, reinterpret_cast<LONG_PTR>(CustomTitleBarWindowProc));
  }
  RefreshCustomTitleBar();
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
  title_bar_drag_regions_.clear();
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

  if (window_resizable_ && !maximized) {
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
  for (const auto& region : title_bar_drag_regions_) {
    if (Contains(region, point)) {
      return HTCAPTION;
    }
  }
#endif
  return {};
}

intptr_t WindowLayer::HandleNativeWindowMessage(void* native_window_handle, const unsigned int message,
                                                const uintptr_t w_param, const intptr_t l_param) const {
#ifdef EVOENGINE_WINDOWS
  const auto hwnd = static_cast<HWND>(native_window_handle);
  switch (message) {
    case WM_ACTIVATE:
    case WM_ENABLE:
      ApplyCustomTitleBarStyle(hwnd, window_resizable_);
      break;
    case WM_GETMINMAXINFO:
      ApplyMaximizedWorkArea(hwnd, l_param);
      return 0;
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
    case WM_NCACTIVATE:
    case WM_NCPAINT:
      return TRUE;
    case WM_NCHITTEST:
      if (const auto hit = HitTestCustomTitleBar(hwnd, l_param)) {
        return *hit;
      }
      break;
    case WM_SYSCOMMAND:
      if (!window_resizable_) {
        const auto command = w_param & 0xfff0;
        if (command == SC_MAXIMIZE || command == SC_SIZE) {
          return 0;
        }
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
  const auto window_layer = ApplicationContext::Get().GetLayer<WindowLayer>();
  const auto prepare_screenshot_capture = [&](const std::shared_ptr<Swapchain>& swapchain) -> std::shared_ptr<Buffer> {
    if (!window_layer || !window_layer->screenshot_capture_ || window_layer->screenshot_capture_->copy_recorded ||
        !swapchain) {
      return {};
    }
    auto& capture = *window_layer->screenshot_capture_;
    const auto extent = swapchain->GetImageExtent();
    capture.resolution = {extent.width, extent.height};
    capture.bgr_format = IsBgrSwapchainFormat(swapchain->GetImageFormat());
    capture.readback_buffer = std::make_shared<Buffer>(static_cast<size_t>(extent.width) * extent.height * 4, true);
    capture.copy_recorded = true;
    return capture.readback_buffer;
  };
  const auto transition_to_present = [](const VkCommandBuffer vk_command_buffer,
                                        const std::shared_ptr<Swapchain>& swapchain, const VkImageLayout old_layout) {
    Platform::TransitImageLayout(vk_command_buffer, swapchain->GetVkImage(), swapchain->GetImageFormat(), 1, old_layout,
                                 VK_IMAGE_LAYOUT_PRESENT_SRC_KHR);
  };
  const auto copy_screenshot_then_present = [&](const VkCommandBuffer vk_command_buffer,
                                                const std::shared_ptr<Swapchain>& swapchain,
                                                const std::shared_ptr<Buffer>& screenshot_buffer) {
    if (!screenshot_buffer) {
      transition_to_present(vk_command_buffer, swapchain, VK_IMAGE_LAYOUT_ATTACHMENT_OPTIMAL_KHR);
      return;
    }
    Platform::TransitImageLayout(vk_command_buffer, swapchain->GetVkImage(), swapchain->GetImageFormat(), 1,
                                 VK_IMAGE_LAYOUT_ATTACHMENT_OPTIMAL_KHR, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL);
    CopySwapchainImageToBuffer(vk_command_buffer, swapchain, *screenshot_buffer);
    transition_to_present(vk_command_buffer, swapchain, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL);
  };
  if (const auto imgui_layer = ApplicationContext::Get().GetLayer<ImGuiLayer>()) {
    const auto swapchain = Platform::GetSwapchain();
    const auto screenshot_buffer = prepare_screenshot_capture(swapchain);
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

      Platform::BeginRendering(vk_command_buffer, render_info);
      ImGui::Render();
      ImGui_ImplVulkan_RenderDrawData(ImGui::GetDrawData(), vk_command_buffer);
      Platform::EndRendering(vk_command_buffer);
      copy_screenshot_then_present(vk_command_buffer, swapchain, screenshot_buffer);
    });

    if (const ImGuiIO& io = ImGui::GetIO(); io.ConfigFlags & ImGuiConfigFlags_ViewportsEnable) {
      ImGui::UpdatePlatformWindows();
      ImGui::RenderPlatformWindowsDefault();
    }
  } else {
    const auto& graphics = Platform::GetInstance();
    bool recorded_present = false;
    if (const auto scene = ApplicationContext::Get().GetActiveScene()) {
      if (const auto main_camera = scene->main_camera.Get<Camera>();
          main_camera->IsEnabled() && main_camera->Rendered()) {
        const auto swapchain = Platform::GetSwapchain();
        const auto screenshot_buffer = prepare_screenshot_capture(swapchain);
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
          copy_screenshot_then_present(vk_command_buffer, swapchain, screenshot_buffer);
        });
        recorded_present = true;
      }
    }
    if (!recorded_present) {
      const auto swapchain = Platform::GetSwapchain();
      const auto screenshot_buffer = prepare_screenshot_capture(swapchain);
      Platform::RecordCommandsMainQueue([&](const VkCommandBuffer vk_command_buffer) {
        Platform::TransitImageLayout(vk_command_buffer, swapchain->GetVkImage(), swapchain->GetImageFormat(), 1,
                                     VK_IMAGE_LAYOUT_UNDEFINED, VK_IMAGE_LAYOUT_ATTACHMENT_OPTIMAL_KHR);

        constexpr VkClearValue clear_color = {{{0.0f, 0.0f, 0.0f, 1.0f}}};
        VkRenderingAttachmentInfo color_attachment_info{};
        color_attachment_info.sType = VK_STRUCTURE_TYPE_RENDERING_ATTACHMENT_INFO;
        color_attachment_info.imageView = swapchain->GetVkImageView();
        color_attachment_info.imageLayout = VK_IMAGE_LAYOUT_ATTACHMENT_OPTIMAL_KHR;
        color_attachment_info.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
        color_attachment_info.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
        color_attachment_info.clearValue = clear_color;

        VkRenderingInfo render_info{};
        render_info.sType = VK_STRUCTURE_TYPE_RENDERING_INFO;
        render_info.renderArea = {{0, 0}, swapchain->GetImageExtent()};
        render_info.layerCount = 1;
        render_info.colorAttachmentCount = 1;
        render_info.pColorAttachments = &color_attachment_info;

        Platform::BeginRendering(vk_command_buffer, render_info);
        Platform::EndRendering(vk_command_buffer);
        copy_screenshot_then_present(vk_command_buffer, swapchain, screenshot_buffer);
      });
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

void WindowLayer::ShowWindow() const {
  if (window_) {
    glfwShowWindow(window_);
  }
}

void WindowLayer::RefreshCustomTitleBar() {
#ifdef EVOENGINE_WINDOWS
  if (!custom_title_bar_ || !window_) {
    return;
  }
  const auto hwnd = native_window_handle_ ? static_cast<HWND>(native_window_handle_) : glfwGetWin32Window(window_);
  if (!hwnd) {
    return;
  }
  native_window_handle_ = hwnd;
  SetPropW(hwnd, kWindowLayerProperty, this);
  ApplyCustomTitleBarStyle(hwnd, window_resizable_);
#endif
}

void WindowLayer::RequestScreenshot(const std::filesystem::path& path) {
  screenshot_capture_ = ScreenshotCapture{};
  screenshot_capture_->path = path;
}

bool WindowLayer::StoreCompletedScreenshot(std::string& error) {
  error.clear();
  if (!screenshot_capture_) {
    error = "window screenshot was not requested";
    return false;
  }
  const auto capture = *screenshot_capture_;
  screenshot_capture_.reset();
  if (!capture.copy_recorded || !capture.readback_buffer || capture.resolution.x == 0 || capture.resolution.y == 0) {
    error = "window screenshot copy was not recorded";
    return false;
  }

  std::vector<uint8_t> bytes;
  capture.readback_buffer->DownloadVector(bytes, static_cast<size_t>(capture.resolution.x) * capture.resolution.y * 4);
  std::vector<float> pixels(bytes.size());
  for (uint32_t y = 0; y < capture.resolution.y; ++y) {
    const auto source_y = capture.resolution.y - 1u - y;
    for (uint32_t x = 0; x < capture.resolution.x; ++x) {
      const auto source_index = (static_cast<size_t>(source_y) * capture.resolution.x + x) * 4;
      const auto target_index = (static_cast<size_t>(y) * capture.resolution.x + x) * 4;
      pixels[target_index] = bytes[source_index + (capture.bgr_format ? 2 : 0)] / 255.0f;
      pixels[target_index + 1] = bytes[source_index + 1] / 255.0f;
      pixels[target_index + 2] = bytes[source_index + (capture.bgr_format ? 0 : 2)] / 255.0f;
      pixels[target_index + 3] = bytes[source_index + 3] / 255.0f;
    }
  }

  if (const auto parent = capture.path.parent_path(); !parent.empty()) {
    std::filesystem::create_directories(parent);
  }
  Texture2D::StoreToPng(capture.path, pixels, capture.resolution.x, capture.resolution.y, 4, 4);
  if (!std::filesystem::exists(capture.path) || std::filesystem::file_size(capture.path) == 0) {
    error = "failed to write window screenshot";
    return false;
  }
  return true;
}

void WindowLayer::MinimizeWindow() const {
#ifdef EVOENGINE_WINDOWS
  if (custom_title_bar_ && native_window_handle_) {
    SendMessageW(static_cast<HWND>(native_window_handle_), WM_SYSCOMMAND, SC_MINIMIZE, 0);
    return;
  }
#endif
  glfwIconifyWindow(window_);
}

void WindowLayer::ToggleMaximized() const {
#ifdef EVOENGINE_WINDOWS
  if (custom_title_bar_ && native_window_handle_) {
    const auto hwnd = static_cast<HWND>(native_window_handle_);
    SendMessageW(hwnd, WM_SYSCOMMAND, IsZoomed(hwnd) ? SC_RESTORE : SC_MAXIMIZE, 0);
    return;
  }
#endif
  if (IsWindowMaximized()) {
    glfwRestoreWindow(window_);
  } else {
    glfwMaximizeWindow(window_);
  }
}

void WindowLayer::SetCustomTitleBarDragRegion(const glm::vec4& region) {
  title_bar_drag_regions_.clear();
  if (region.z > 0.0f && region.w > 0.0f) {
    title_bar_drag_regions_.push_back(region);
  }
}

void WindowLayer::SetCustomTitleBarDragRegions(const std::vector<glm::vec4>& regions) {
  title_bar_drag_regions_.clear();
  for (const auto& region : regions) {
    if (region.z > 0.0f && region.w > 0.0f) {
      title_bar_drag_regions_.push_back(region);
    }
  }
}

void WindowLayer::ClearCustomTitleBarDragRegion() {
  title_bar_drag_regions_.clear();
}

void WindowLayer::CenterWindow() const {
  if (!window_ || IsWindowMaximized()) {
    return;
  }

  GLFWmonitor* monitor = primary_monitor_ ? primary_monitor_ : glfwGetPrimaryMonitor();
  if (!monitor) {
    return;
  }

  int work_x = 0;
  int work_y = 0;
  int work_width = 0;
  int work_height = 0;
  glfwGetMonitorWorkarea(monitor, &work_x, &work_y, &work_width, &work_height);
  if (work_width <= 0 || work_height <= 0) {
    glfwGetMonitorPos(monitor, &work_x, &work_y);
    const GLFWvidmode* mode = glfwGetVideoMode(monitor);
    if (!mode) {
      return;
    }
    work_width = mode->width;
    work_height = mode->height;
  }

  int width = 0;
  int height = 0;
#ifdef EVOENGINE_WINDOWS
  if (custom_title_bar_ && native_window_handle_) {
    RECT rect{};
    GetWindowRect(static_cast<HWND>(native_window_handle_), &rect);
    width = rect.right - rect.left;
    height = rect.bottom - rect.top;
  } else
#endif
  {
    glfwGetWindowSize(window_, &width, &height);
  }
  if (width <= 0 || height <= 0) {
    return;
  }

  const int x = work_x + std::max(0, (work_width - width) / 2);
  const int y = work_y + std::max(0, (work_height - height) / 2);
#ifdef EVOENGINE_WINDOWS
  if (custom_title_bar_ && native_window_handle_) {
    SetWindowPos(static_cast<HWND>(native_window_handle_), nullptr, x, y, 0, 0,
                 SWP_NOSIZE | SWP_NOZORDER | SWP_NOACTIVATE);
    return;
  }
#endif
  glfwSetWindowPos(window_, x, y);
}

void WindowLayer::ResizeWindow(int x, int y) const {
#ifdef EVOENGINE_WINDOWS
  if (custom_title_bar_ && native_window_handle_) {
    SetWindowPos(static_cast<HWND>(native_window_handle_), nullptr, 0, 0, x, y,
                 SWP_NOMOVE | SWP_NOZORDER | SWP_NOACTIVATE);
    return;
  }
#endif
  glfwSetWindowSize(window_, x, y);
}
