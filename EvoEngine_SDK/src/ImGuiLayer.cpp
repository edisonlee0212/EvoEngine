#include "ImGuiLayer.hpp"

#include "Platform.hpp"

using namespace evo_engine;

void ImGuiLayer::OnDestroy() {
  if (!ImGui::GetCurrentContext())
    return;

  ImGui_ImplVulkan_Shutdown();
  ImGui_ImplGlfw_Shutdown();
  ImNodes::DestroyContext();
  ImGui::DestroyContext();
}

void ImGuiLayer::PreUpdate() {
  {
    const std::lock_guard queue_lock(Platform::GetQueueHostMutex());
    ImGui_ImplVulkan_NewFrame();
  }
  ImGui_ImplGlfw_NewFrame();
  ImGui::NewFrame();
  ImGuizmo::BeginFrame();
}
