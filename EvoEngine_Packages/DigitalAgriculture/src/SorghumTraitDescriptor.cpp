#include "SorghumTraitDescriptor.hpp"

#include "Application.hpp"
#include "EditorLayer.hpp"
#include "Scene.hpp"

#include "SorghumGenerator.hpp"
#include "ProjectManager.hpp"
#include "SorghumLayer.hpp"

using namespace digital_agriculture_package;


bool SorghumLeafTrait::DrawGui(const std::shared_ptr<EditorLayer>& editor_layer) {
  bool changed = false;

    if (ImGui::TreeNode("traits")) {
      ImGui::Text("leaf rank: %d", leaf_index);
      ImGui::Text("leaf width: %.2f", leaf_width);
      ImGui::Text("leaf area: %.2f", leaf_area);
      ImGui::Text("internode length: %.2f", internode_length);


      changed = true;
      ImGui::TreePop();
    
  }

  return changed;

}

void SorghumLeafTrait::Serialize(YAML::Emitter& out) const {

  out << YAML::Key << "leaf_rank" << YAML::Value << leaf_index;
  out << YAML::Key << "leaf_width" << YAML::Value << leaf_width;
  out << YAML::Key << "leaf_area" << YAML::Value << leaf_area;
  out << YAML::Key << "internode_length" << YAML::Value << internode_length;
}

void SorghumLeafTrait::Deserialize(const YAML::Node& in) {
  if (in["leaf_rank"])
    leaf_index = in["leaf_rank"].as<int>();
  if (in["leaf_width"])
    leaf_width = in["leaf_width"].as<float>();
  if (in["leaf_area"])
    leaf_area = in["leaf_area"].as<float>();
  if (in["internode_length"])
    internode_length = in["internode_length"].as<float>();
}

bool SorghumTraitDescriptor::DrawGui(const std::shared_ptr<EditorLayer>& editor_layer) {
  bool changed = false;
  if (ImGui::TreeNodeEx((std::string("Stem")).c_str())) {
    ImGui::Text("stem length: (%.2f)", stem_length);
    changed = true;
    ImGui::TreePop();
  }

  if (ImGui::TreeNodeEx("Leaves")) {

    
    for (auto& leaf : leaf_traits) {
      if (ImGui::TreeNode(
              ("Leaf No." + std::to_string(leaf.leaf_index + 1))
                  .c_str())) {
        if (leaf.DrawGui(editor_layer))
          changed = true;
        ImGui::TreePop();
      }
    }
    ImGui::TreePop();
  }

  return changed;
}

