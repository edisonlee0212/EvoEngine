#include "EcoSysLabSettingsEditor.hpp"
#include "EditorLayer.hpp"
#include "EditorWidgets.hpp"
#include "TreeGrowthSettings.hpp"

using namespace eco_sys_lab_package;

bool eco_sys_lab_package::InspectSettings(TreeGrowthSettings& target,
                                          const std::shared_ptr<EditorLayer>& editor_layer) {
  bool changed = false;
  if (ImGui::Checkbox("Enable space colonization", &target.use_space_colonization))
    changed = true;
  if (target.use_space_colonization) {
    if (ImGui::Checkbox("Space colonization auto resize", &target.space_colonization_auto_resize))
      changed = true;
  }
  return changed;
}
