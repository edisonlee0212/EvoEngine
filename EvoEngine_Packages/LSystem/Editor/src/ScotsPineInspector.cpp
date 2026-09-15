#include "EditorLayer.hpp"
#include "LSystemInspectionAdapters.hpp"
#include "ScotsPine.hpp"
#include "ScotsPineDescriptor.hpp"

using namespace evo_engine;
using namespace l_system_package;
bool l_system_package::InspectScotsPine(InspectorContext& context, ScotsPine& pine) {
  const auto& editor_layer = context.editor_layer;
  bool changed = false;

  if (editor_layer->DragAndDropButton<ScotsPineDescriptor>(pine.descriptor_ref, "Descriptor"))
    changed = true;

  if (editor_layer->DragAndDropButton<ScotsPineDescriptor>(pine.post_repot_descriptor_ref, "Post-Repot Descriptor")) {
    changed = true;
  }

  if (ImGui::Checkbox("Enable Repot Profile Switch", &pine.enable_repot_profile_switch)) {
    changed = true;
  }
  if (pine.enable_repot_profile_switch) {
    if (ImGui::DragFloat("Repot Switch GDD", &pine.repot_switch_gdd, 10.0f, 0.0f, 200000.0f, "%.1f")) {
      pine.repot_switch_gdd = std::max(0.0f, pine.repot_switch_gdd);
      changed = true;
    }
  }

  int seed_int = static_cast<int>(pine.seed);
  if (ImGui::DragInt("Seed", &seed_int, 1, 0, 999999)) {
    pine.seed = static_cast<unsigned int>(seed_int);
    changed = true;
  }

  if (ImGui::DragFloat("Target GDD", &pine.target_gdd, 1.0f, 0.0f, 200000.0f, "%.1f"))
    changed = true;

  if (ImGui::Button("Generate")) {
    pine.GenerateGeometryEntities();
    changed = true;
  }
  ImGui::SameLine();
  if (ImGui::Button("Clear")) {
    pine.ClearGeometryEntities();
    changed = true;
  }

  if (pine.growth_model.IsInitialized()) {
    ImGui::Separator();
    ImGui::Text("GDD: %.1f", pine.growth_model.accumulated_gdd);
    ImGui::Text("Topology: %s", pine.growth_model.IsTopologyComplete() ? "Complete" : "Pending");
    const auto& sorted = pine.growth_model.graph.PeekSortedNodeList();
    ImGui::Text("Nodes: %d", static_cast<int>(sorted.size()));
    ImGui::Text("Internodes: %u   Needles: %u", pine.last_internode_count, pine.last_needle_count);
  }

  return changed;
}
