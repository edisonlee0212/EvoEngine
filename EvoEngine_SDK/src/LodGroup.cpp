#include "LODGroup.hpp"
#include "EditorLayer.hpp"
#include "MeshRenderer.hpp"
#include "SkinnedMeshRenderer.hpp"
#include "StrandsRenderer.hpp"
using namespace evo_engine;

bool Lod::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  bool changed = false;
  std::string tag = "##LOD" + std::to_string(index);
  ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(1.f, 0.3f, 0, 1));
  ImGui::Button("Drop to Add...");
  ImGui::PopStyleColor(1);
  EntityRef temp{};
  if (EditorLayer::Droppable(temp)) {
    const auto scene = Application::GetActiveScene();
    const auto entity = temp.Get();
    if (scene->HasPrivateComponent<MeshRenderer>(entity)) {
      const auto pc = scene->GetOrSetPrivateComponent<MeshRenderer>(entity).lock();
      bool duplicate = false;
      for (const auto& i : renderers) {
        if (i.GetHandle() == pc->GetHandle()) {
          duplicate = true;
          break;
        }
      }
      if (!duplicate) {
        renderers.emplace_back();
        renderers.back().Set(pc);
      }
    }
    if (scene->HasPrivateComponent<SkinnedMeshRenderer>(entity)) {
      const auto pc = scene->GetOrSetPrivateComponent<SkinnedMeshRenderer>(entity).lock();
      bool duplicate = false;
      for (const auto& i : renderers) {
        if (i.GetHandle() == pc->GetHandle()) {
          duplicate = true;
          break;
        }
      }
      if (!duplicate) {
        renderers.emplace_back();
        renderers.back().Set(pc);
      }
    }
    if (scene->HasPrivateComponent<Particles>(entity)) {
      const auto pc = scene->GetOrSetPrivateComponent<Particles>(entity).lock();
      bool duplicate = false;
      for (const auto& i : renderers) {
        if (i.GetHandle() == pc->GetHandle()) {
          duplicate = true;
          break;
        }
      }
      if (!duplicate) {
        renderers.emplace_back();
        renderers.back().Set(pc);
      }
    }
    if (scene->HasPrivateComponent<StrandsRenderer>(entity)) {
      const auto pc = scene->GetOrSetPrivateComponent<StrandsRenderer>(entity).lock();
      bool duplicate = false;
      for (const auto& i : renderers) {
        if (i.GetHandle() == pc->GetHandle()) {
          duplicate = true;
          break;
        }
      }
      if (!duplicate) {
        renderers.emplace_back();
        renderers.back().Set(pc);
      }
    }
  }
  for (auto it = renderers.begin(); it != renderers.end(); ++it) {
    if (const auto ptr = it->Get<IPrivateComponent>()) {
      const auto scene = Application::GetActiveScene();
      if (!scene->IsEntityValid(ptr->GetOwner())) {
        it->Clear();
        ImGui::Button("none");
        return true;
      }
      ImGui::Button((scene->GetEntityName(ptr->GetOwner()) + "(" + ptr->GetTypeName() + ")").c_str());
      EditorLayer::Draggable(*it);
      if (EditorLayer::Remove(*it)) {
        if (!it->Get<IPrivateComponent>()) {
          renderers.erase(it);
          break;
        }
      }
    }
  }
  return changed;
}

void LodGroup::Serialize(YAML::Emitter& out) const {
  out << YAML::Key << "override_lod_factor" << YAML::Value << override_lod_factor;
  out << YAML::Key << "lod_factor" << YAML::Value << lod_factor;

  if (!lods.empty()) {
    out << YAML::Key << "lods" << YAML::BeginSeq;
    for (const auto& lod : lods) {
      out << YAML::BeginMap;
      {
        out << YAML::Key << "index" << YAML::Value << lod.index;
        out << YAML::Key << "lod_offset" << YAML::Value << lod.lod_offset;
        out << YAML::Key << "transition_width" << YAML::Value << lod.transition_width;
        if (!lod.renderers.empty()) {
          out << YAML::Key << "renderers" << YAML::BeginSeq;
          for (const auto& renderer : lod.renderers) {
            out << YAML::BeginMap;
            { renderer.Serialize(out); }
            out << YAML::EndMap;
          }
          out << YAML::EndSeq;
        }
      }
      out << YAML::EndMap;
    }
    out << YAML::EndSeq;
  }
}

void LodGroup::Deserialize(const YAML::Node& in) {
  const auto scene = GetScene();
  if (in["override_lod_factor"])
    override_lod_factor = in["override_lod_factor"].as<bool>();
  if (in["lod_factor"])
    lod_factor = in["lod_factor"].as<float>();
  if (in["lods"]) {
    const auto& in_lods = in["lods"];
    lods.clear();
    for (const auto& in_lod : in_lods) {
      lods.emplace_back();
      auto& lod = lods.back();
      lod.renderers.clear();
      if (in_lod["index"])
        lod.index = in_lod["index"].as<int>();
      if (in_lod["lod_offset"])
        lod.lod_offset = in_lod["lod_offset"].as<float>();
      if (in_lod["transition_width"])
        lod.transition_width = in_lod["transition_width"].as<float>();
      if (in_lod["renderers"]) {
        for (const auto& in_renderer : in_lod["renderers"]) {
          lod.renderers.emplace_back();
          auto& renderer = lod.renderers.back();
          renderer.Deserialize(in_renderer, scene);
        }
      }
    }
  }
}

bool LodGroup::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  bool changed = false;
  if (ImGui::Checkbox("Override LOD Factor", &override_lod_factor))
    changed = true;
  if (override_lod_factor) {
    if (ImGui::SliderFloat("Current LOD Factor", &lod_factor, 0.f, 1.f))
      changed = true;
  } else {
    ImGui::Text((std::string("LOD Factor: ") + std::to_string(lod_factor)).c_str());
  }
  if (ImGui::TreeNodeEx("LODs", ImGuiTreeNodeFlags_DefaultOpen)) {
    int lod_index = 0;
    for (auto it = lods.begin(); it != lods.end(); ++it) {
      if (ImGui::TreeNode((std::string("LOD ") + std::to_string(lod_index)).c_str())) {
        std::string tag = "##LODs" + std::to_string(lod_index);
        float min = 0.f;
        float max = 1.f;
        if (it != lods.begin()) {
          min = (it - 1)->lod_offset;
        }
        if (it + 1 != lods.end()) {
          max = (it + 1)->lod_offset;
        }
        if (ImGui::SliderFloat("LOD Offset", &it->lod_offset, min, max))
          changed = true;
        if (it->OnInspect(editor_layer))
          changed = true;
        ImGui::TreePop();
      }
      lod_index++;
    }
    if (!lods.empty() && lods.back().lod_offset != 1.f) {
      ImGui::Text((std::string("Culled: [") + std::to_string(lods.back().lod_offset) + " -> 1.0]").c_str());
    }
    if (ImGui::Button("Push LOD")) {
      lods.emplace_back();
      lods.back().lod_offset = 1.f;
      lods.back().index = lods.size() - 1;
      if (lods.size() > 1) {
        float prev_val = 0.f;
        if (lods.size() > 2) {
          prev_val = lods.at(lods.size() - 3).lod_offset;
        }
        lods.at(lods.size() - 2).lod_offset = (1.f + prev_val) * .5f;
      }
      changed = true;
    }
    if (!lods.empty()) {
      ImGui::SameLine();
      if (ImGui::Button("Pop LOD")) {
        const float last_offset = lods.back().lod_offset;
        lods.pop_back();
        if (!lods.empty())
          lods.back().lod_offset = last_offset;
        changed = true;
      }
    }
    ImGui::TreePop();
  }

  return changed;
}

void LodGroup::Relink(const std::unordered_map<Handle, Handle>& map, const std::shared_ptr<Scene>& scene) {
  for (auto& lod : lods) {
    for (auto& i : lod.renderers) {
      i.Relink(map, scene);
    }
  }
}
