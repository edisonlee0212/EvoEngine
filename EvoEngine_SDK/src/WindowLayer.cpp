#include "WindowLayer.hpp"
#include "Application.hpp"
#include "Graphics.hpp"
#include "ProjectManager.hpp"

using namespace evo_engine;

void WindowLayer::FramebufferSizeCallback(GLFWwindow* window, int width, int height) {
  if (const auto window_layer = Application::GetLayer<WindowLayer>(); window_layer->window_ == window) {
    window_layer->window_size_ = {width, height};
  }
  if (const auto& graphics_layer = Application::GetLayer<Graphics>()) {
    graphics_layer->NotifyRecreateSwapChain();
  }
}

void WindowLayer::SetMonitorCallback(GLFWmonitor* monitor, int event) {
  const auto window_layer = Application::GetLayer<WindowLayer>();
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
  const auto window_layer = Application::GetLayer<WindowLayer>();

  if (focused) {
    ProjectManager::ScanProject();
  }
}

void WindowLayer::OnCreate() {
}

void WindowLayer::OnDestroy() {
#pragma region Windows
  glfwDestroyWindow(window_);
  glfwTerminate();
#pragma endregion
}

GLFWwindow* WindowLayer::GetGlfwWindow() const {
  return window_;
}

void WindowLayer::ResizeWindow(int x, int y) const {
  glfwSetWindowSize(window_, x, y);
}