#pragma once
#include "ILayer.hpp"
namespace evo_engine {
class WindowLayer final : public ILayer {
  friend class Graphics;
  friend class RenderLayer;
#pragma region Presenters
  std::vector<GLFWmonitor*> monitors_;
  GLFWmonitor* primary_monitor_ = nullptr;
  GLFWwindow* window_ = nullptr;
  glm::ivec2 window_size_ = {1, 1};
#pragma endregion

  static void FramebufferSizeCallback(GLFWwindow*, int, int);
  static void SetMonitorCallback(GLFWmonitor* monitor, int event);
  static void WindowFocusCallback(GLFWwindow* window, int focused);

  void OnCreate() override;
  void OnDestroy() override;

 public:
  [[nodiscard]] GLFWwindow* GetGlfwWindow() const;
  void ResizeWindow(int x, int y) const;
};
}  // namespace evo_engine