#include "HeightField.hpp"
using namespace eco_sys_lab_plugin;

float HeightField::GetValue(const glm::vec2& position) const {
  return noises_graph.GetValue(position + position_offset);
}

void HeightField::RandomOffset(const float min, const float max) {
  position_offset = glm::vec2(glm::linearRand(min, max), glm::linearRand(min, max));
}

bool HeightField::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  bool changed = false;
  changed = ImGui::DragInt("Precision level", &precision_level) || changed;
  static bool show_noise_graph = false;
  ImGui::Checkbox("Show noise graph", &show_noise_graph);
  if (show_noise_graph) {
    changed = noises_graph.ShowGraph("Height field noise graph", editor_layer) | changed;
  }
  if (ImGui::DragFloat2("Position offset", &position_offset.x, 0.1f)) {
    changed = true;
  }
  if (ImGui::TreeNode("Visualization")) {
    static int resolution = 64;
    static float position_scale = 1.f;
    bool resolution_changed = false;
    if (ImGui::DragInt("Resolution", &resolution, 1, 16, 1024)) {
      resolution = glm::clamp(resolution, 16, 1024);
      resolution_changed = true;
    }
    if (ImGui::DragFloat("Position scale", &position_scale, 0.1f)) {
      resolution_changed = true;
    }
    static Handle current_handle;
    static bool show_test_texture = true;
    static std::shared_ptr<Texture2D> test_texture_2d;
    if (!test_texture_2d) {
      test_texture_2d = AssetManager::CreateTemporaryAsset<Texture2D>();
    }
    if (show_test_texture) {
      if (changed || resolution_changed || GetHandle() != current_handle) {
        current_handle = GetHandle();
        std::vector<glm::vec4> color(resolution * resolution);
        Jobs::RunParallelFor(resolution * resolution, [&](size_t i) {
          float x = i / resolution;
          float y = i % resolution;
          x /= resolution;
          y /= resolution;
          color[i] = glm::vec4(glm::vec3(GetValue(glm::vec2(x, y) * position_scale + position_offset)), 1.0f);
        });

        test_texture_2d->SetRgbaChannelData(color, glm::uvec2(resolution));
      }

      const auto texture_storage = test_texture_2d->PeekTexture2DStorage();
      if (texture_storage.im_texture_id) {
        static float debug_scale = 1.f;
        ImGui::DragFloat("Scale", &debug_scale, 0.01f, 0.1f, 10.0f);
        debug_scale = glm::clamp(debug_scale, 0.1f, 10.0f);
        ImGui::Image(texture_storage.im_texture_id,
                     ImVec2(texture_storage.image->GetExtent().width * debug_scale,
                            texture_storage.image->GetExtent().height * debug_scale),
                     ImVec2(0, 1), ImVec2(1, 0));
      }
    }
    ImGui::TreePop();
  }
  return changed;
}

void HeightField::Serialize(YAML::Emitter& out) const {
  out << YAML::Key << "precision_level" << YAML::Value << precision_level;
  out << YAML::Key << "position_offset" << YAML::Value << position_offset;
  noises_graph.Save("noises_graph", out);
}

void HeightField::Deserialize(const YAML::Node& in) {
  if (in["precision_level"])
    precision_level = in["precision_level"].as<int>();
  if (in["position_offset"])
    position_offset = in["position_offset"].as<glm::vec2>();
  noises_graph.Load("noises_graph", in);
}

std::shared_ptr<Texture2D> HeightField::GenerateThumbnailTexture() {
  static std::shared_ptr<Texture2D> thumbnail;
  if (!thumbnail) {
    thumbnail = AssetManager::CreateTemporaryAsset<Texture2D>();
    thumbnail->Import(
        std::filesystem::absolute(std::filesystem::path("./EcoSysLabResources") / "Icons/HeightField.png"));
  }
  return thumbnail;
}

void HeightField::GenerateMesh(const glm::vec2& start, const glm::uvec2& resolution, float unit_size,
                               std::vector<Vertex>& vertices, std::vector<glm::uvec3>& triangles, float x_depth,
                               float z_depth) const {
  for (unsigned i = 0; i < resolution.x * precision_level; i++) {
    for (unsigned j = 0; j < resolution.y * precision_level; j++) {
      Vertex archetype;
      archetype.position.x = start.x + unit_size * i / precision_level;
      archetype.position.z = start.y + unit_size * j / precision_level;
      archetype.position.y = GetValue({archetype.position.x, archetype.position.z});
      archetype.tex_coord = glm::vec2(static_cast<float>(i) / (resolution.x * precision_level),
                                      static_cast<float>(j) / (resolution.y * precision_level));
      vertices.push_back(archetype);
    }
  }

  for (int i = 0; i < resolution.x * precision_level - 1; i++) {
    for (int j = 0; j < resolution.y * precision_level - 1; j++) {
      if (static_cast<float>(i) / (resolution.x * precision_level - 2) > (1.0 - z_depth) &&
          static_cast<float>(j) / (resolution.y * precision_level - 2) < x_depth)
        continue;
      const int n = resolution.x * precision_level;
      triangles.emplace_back(i + j * n, i + 1 + j * n, i + (j + 1) * n);
      triangles.emplace_back(i + 1 + (j + 1) * n, i + (j + 1) * n, i + 1 + j * n);
    }
  }
}