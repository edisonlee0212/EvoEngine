#include "Cubemap.hpp"
#include "EditorLayer.hpp"
#include "EnvironmentalMap.hpp"
#include "GlobalReflectionProbe.hpp"
#include "Resources.hpp"

using namespace evo_engine;

void EditorLayer::DrawResources(const std::shared_ptr<EditorLayer>& editor_layer) {
  auto& resources = Resources::GetInstance();
  if (editor_layer->show_resources_) {
    if (ImGui::Begin("Resources")) {
      if (ImGui::CollapsingHeader("Textures")) {
        ImGui::Button("Missing");
        editor_layer->DraggableAsset<Texture2D>(resources.GetMissingTexture());
      }
      if (ImGui::CollapsingHeader("Cubemap")) {
        ImGui::Button("Default Skybox");
        editor_layer->DraggableAsset<Cubemap>(resources.GetDefaultSkybox());
      }
      if (ImGui::CollapsingHeader("Environmental Map")) {
        ImGui::Button("Default Env map");
        editor_layer->DraggableAsset<EnvironmentalMap>(resources.GetDefaultEnvironmentalMap());
        ImGui::Button("Default Global Reflection Probe");
        editor_layer->DraggableAsset<GlobalReflectionProbe>(resources.GetDefaultGlobalReflectionProbe());
      }
      if (ImGui::CollapsingHeader("Primitives")) {
        ImGui::Button("Quad");
        editor_layer->DraggableAsset<Mesh>(resources.GetPrimitives().quad);
        ImGui::Button("Sphere");
        editor_layer->DraggableAsset<Mesh>(resources.GetPrimitives().sphere);
        ImGui::Button("Cube");
        editor_layer->DraggableAsset<Mesh>(resources.GetPrimitives().cube);
        ImGui::Button("Cone");
        editor_layer->DraggableAsset<Mesh>(resources.GetPrimitives().cone);
        ImGui::Button("Cylinder");
        editor_layer->DraggableAsset<Mesh>(resources.GetPrimitives().cylinder);
        ImGui::Button("Torus");
        editor_layer->DraggableAsset<Mesh>(resources.GetPrimitives().torus);
        ImGui::Button("Monkey");
        editor_layer->DraggableAsset<Mesh>(resources.GetPrimitives().monkey);
        ImGui::Button("Capsule");
        editor_layer->DraggableAsset<Mesh>(resources.GetPrimitives().capsule);
      }
    }
    ImGui::End();
  }
}
