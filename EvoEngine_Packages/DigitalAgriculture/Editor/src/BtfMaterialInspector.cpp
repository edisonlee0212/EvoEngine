#include "DigitalAgricultureInspectorStates.hpp"
#include "DigitalAgricultureSerializationAdapters.hpp"
#include "EditorFileDialogs.hpp"
using namespace evo_engine;
#include "Console.hpp"
#include "Utilities.hpp"
using namespace evo_engine;
bool digital_agriculture_package::InspectBtfMaterial(InspectorContext& context, BtfMaterial& target) {
  const auto& editor_layer = context.editor_layer;

  bool changed = false;
  EditorFileDialogs::OpenFolder(
      "Import Database",
      [&](const std::filesystem::path& path) {
        try {
          const bool succeed = target.ImportFromFolder(path);
          if (succeed)
            changed = true;
          EVOENGINE_LOG((std::string("BTF Material import ") + (succeed ? "succeed" : "failed")))
        } catch (const std::exception& e) {
          EVOENGINE_ERROR(std::string(e.what()))
        }
      },
      false);

  if (target.btf_base.has_data) {
    if (ImGui::DragFloat("TexCoord Multiplier", &target.btf_base.tex_coord_multiplier, 0.1f)) {
      changed = true;
    }

    if (ImGui::Checkbox("HDR", &target.btf_base.hdr)) {
      changed = true;
    }
    if (target.btf_base.hdr) {
      if (ImGui::DragFloat("HDR Value", &target.btf_base.hdr_value, 0.01f)) {
        changed = true;
      }
    }
    if (ImGui::DragFloat("Gamma Value", &target.btf_base.gamma, 0.01f)) {
      changed = true;
    }
  }
  return changed;
}
