#include "WindowLayer.hpp"
#include "Application.hpp"
#include "EditorLayer.hpp"
#include "ImGuiLayer.hpp"
#include "Platform.hpp"
#include "ProjectManager.hpp"
#include "RenderLayer.hpp"
#include "Resources.hpp"
using namespace evo_engine;

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
  glfwDestroyWindow(window_);
  glfwTerminate();
#pragma endregion
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

void WindowLayer::ResizeWindow(int x, int y) const {
  glfwSetWindowSize(window_, x, y);
}
