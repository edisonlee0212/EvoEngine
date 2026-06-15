
#pragma once
#include <cstdint>
#include <optional>
#include "ILayer.hpp"

namespace evo_engine {

/**
 * @class WindowLayer
 * @brief A window management layer class.
 *
 * The WindowLayer class is responsible for managing GLFW windows, primary monitors,
 * and providing facilities to handle window events, resizing, and rendering.
 */
class WindowLayer final : public ILayer {
  /**
   * @brief Allows `Platform` class access to private and protected members of WindowLayer.
   */
  friend class Platform;

  /**
   * @brief Allows `RenderLayer` class access to private and protected members of WindowLayer.
   */
  friend class RenderLayer;

  /**
   * @brief Allows `Application` class access to private and protected members of WindowLayer.
   */
  friend class Application;

#pragma region Presenters
  /**
   * @brief A collection of monitors currently enumerated by GLFW.
   */
  std::vector<GLFWmonitor*> monitors_;

  /**
   * @brief Pointer to the primary monitor.
   */
  GLFWmonitor* primary_monitor_ = nullptr;

  /**
   * @brief Pointer to the GLFW window.
   */
  GLFWwindow* window_ = nullptr;

  /**
   * @brief Size of the window in pixels.
   */
  glm::ivec2 window_size_ = {1, 1};

  bool custom_title_bar_ = false;
  std::vector<glm::vec4> title_bar_drag_regions_;
  void* native_window_handle_ = nullptr;
  intptr_t default_window_proc_ = 0;
#pragma endregion

  /**
   * @brief GLFW framebuffer size callback function.
   *
   * This function is called when the framebuffer size changes.
   *
   * @param window Pointer to the GLFW window.
   * @param width New width of the framebuffer.
   * @param height New height of the framebuffer.
   */
  static void FramebufferSizeCallback(GLFWwindow*, int, int);

  /**
   * @brief GLFW monitor callback function.
   *
   * This function is called when a monitor is connected or disconnected.
   *
   * @param monitor Pointer to the GLFW monitor.
   * @param event Event type (e.g., connected or disconnected).
   */
  static void SetMonitorCallback(GLFWmonitor* monitor, int event);

  /**
   * @brief GLFW window focus callback function.
   *
   * This function is called when the window gains or loses focus.
   *
   * @param window Pointer to the GLFW window.
   * @param focused A value indicating whether the window is focused (1) or not (0).
   */
  static void WindowFocusCallback(GLFWwindow* window, int focused);

  /**
   * @brief Handles operations required during the creation of the window layer.
   */
  void OnCreate() override;

  /**
   * @brief Handles operations required during the destruction of the window layer.
   */
  void OnDestroy() override;

  void InstallCustomTitleBar();
  void UninstallCustomTitleBar();
  void ShowWindow() const;
  [[nodiscard]] std::optional<intptr_t> HitTestCustomTitleBar(void* native_window_handle, intptr_t l_param) const;

  /**
   * @brief Executes rendering of the window content.
   */
  static void Render();

 public:
  /**
   * @brief Retrieves the GLFW window pointer.
   *
   * @return A pointer to the GLFWwindow object.
   */
  [[nodiscard]] GLFWwindow* GetGlfwWindow() const;

  [[nodiscard]] bool UsesCustomTitleBar() const;

  [[nodiscard]] bool IsWindowMaximized() const;

  intptr_t HandleNativeWindowMessage(void* native_window_handle, unsigned int message, uintptr_t w_param,
                                     intptr_t l_param) const;

  void RefreshCustomTitleBar();

  void MinimizeWindow() const;

  void ToggleMaximized() const;

  void SetCustomTitleBarDragRegion(const glm::vec4& region);

  void SetCustomTitleBarDragRegions(const std::vector<glm::vec4>& regions);

  void ClearCustomTitleBarDragRegion();

  void CenterWindow() const;

  /**
   * @brief Resizes the GLFW window to the specified dimensions.
   *
   * @param x New width of the window.
   * @param y New height of the window.
   */
  void ResizeWindow(int x, int y) const;
};

}  // namespace evo_engine
