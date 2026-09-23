#include "BasicFoliageDescriptor.hpp"
#include "CurveEditors.hpp"
#include "DynamicsSettingsEditor.hpp"
#include "EcoSysLabGraphEditors.hpp"
#include "EcoSysLabSettingsEditor.hpp"
#include "EditorLayer.hpp"
#include "EditorWidgets.hpp"
#include "SDKInspectionAdapters.hpp"
using namespace evo_engine;
using namespace eco_sys_lab_package;
bool eco_sys_lab_package::InspectSettings(DynamicStrands::PhysicsParameters& target,
                                          const std::shared_ptr<EditorLayer>& editor_layer) {
  bool changed = false;
  if (ImGui::TreeNode("Preset Settings")) {
    if (ImGui::Button("Log Crack")) {
      target.bundle_strength_factor = 1.2f;
      target.crack_bd_shrinkage_offset = 0.1f;
      target.crack_R_scale = 1.0f;
      target.crack_T_scale = 1.0f;
      target.boundary_strength_decay_factor = 3.0f;
      target.internal_pattern = 1;
      changed = true;
    }
    if (ImGui::Button("Oak Trunk Crack Process")) {
      target.bundle_strength_factor = 1.0f;
      target.crack_bd_shrinkage_offset = 0.0f;
      target.crack_R_scale = 0.0f;
      target.crack_T_scale = 1.0f;
      target.boundary_strength_decay_factor = 6.0f;
      target.internal_pattern = 1;
      changed = true;
    }
    if (ImGui::Button("Oak Trunk Full Process")) {
      target.bundle_strength_factor = 1.0f;
      target.crack_bd_shrinkage_offset = 0.0f;
      target.crack_R_scale = 0.0f;
      target.crack_T_scale = 1.0f;
      target.boundary_strength_decay_factor = 6.0f;
      target.internal_pattern = 1;
      target.bd_offset = 0.06f;
      target.HL_threshold = 0.1f;
      target.matrixAb = glm::mat3(0.5f, 0.0f, 0.0f, 0.0f, 0.5f, 0.0f, 0.0f, 0.0f, 2.0f);
      // time_step = 0.005f;
      target.bb = 0.5f;
      target.be = 0.5f;  // ZY: Test it!
      changed = true;
    }
    if (ImGui::Button("Elm")) {
      target.matrixAb = glm::mat3(1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 20.0f);
      target.bb = 0.5f;
      target.HC_threshold = 0.5f;
      target.HL_threshold = 0.1f;
      target.moisture_breaking_rod = 1;
      target.bd_offset = 0.02f;
      target.leaf_break_from_moisture = 1;
      target.leaf_break_threshold = 0.4f;
      changed = true;
    }
    if (ImGui::Button("Spruce")) {
      target.matrixAb = glm::mat3(1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 20.0f);
      target.matrixAm = glm::mat3(0.5f, 0.0f, 0.0f, 0.0f, 0.5f, 0.0f, 0.0f, 0.0f, 0.5f);
      target.bb = 0.5f;
      target.HC_threshold = 0.5f;
      target.HL_threshold = 0.1f;
      target.moisture_breaking_rod = 1;
      target.bd_offset = 0.015f;
      target.leaf_break_from_moisture = 1;
      target.leaf_break_threshold = 0.4f;
      // rod_strength_factor = 0.7f;
      changed = true;
    }
    if (ImGui::Button("Oak")) {
      target.matrixAb = glm::mat3(1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 5.0f);
      target.matrixAm = glm::mat3(0.5f, 0.0f, 0.0f, 0.0f, 0.5f, 0.0f, 0.0f, 0.0f, 0.5f);
      target.bb = 0.5f;
      target.HC_threshold = 0.5f;
      target.HL_threshold = 0.1f;
      target.moisture_breaking_rod = 1;
      target.bd_offset = 0.02f;
      target.leaf_break_from_moisture = 1;
      target.leaf_break_threshold = 0.4f;
      changed = true;
    }
    ImGui::TreePop();
  }
  if (ImGui::DragFloat("Time step", &target.time_step, 0.001f, 0.001f, 1.0f))
    changed = true;
  if (ImGui::DragInt("Sub step", &target.sub_step, 1, 1, 100)) {
    changed = true;
  }
  if (ImGui::IsItemFocused()) {
    if (ImGui::IsKeyPressed(ImGuiKey_UpArrow)) {
      target.sub_step = ImMin(target.sub_step + 1, 100);
      changed = true;
    }
    if (ImGui::IsKeyPressed(ImGuiKey_DownArrow)) {
      target.sub_step = ImMax(target.sub_step - 1, 1);
      changed = true;
    }
  }
  if (ImGui::Checkbox("Fungus model", &target.enable_fungus)) {
    changed = true;
  }
  if (ImGui::Checkbox("Enable Collision", &target.enable_segment_collision)) {
    changed = true;
  }
  if (ImGui::DragFloat("Rod segment strength factor", &target.rod_strength_factor, 0.01f, 0.0f, 1.0f))
    changed = true;
  if (ImGui::DragFloat("Bundle strength factor", &target.bundle_strength_factor, 0.01f, 0.0f, 2.0f))
    changed = true;

  bool pull_cubical_bool = (target.pull_cubical != 0);
  if (ImGui::Checkbox("Test on Pull Operators for cubical rotting?", &pull_cubical_bool)) {
    target.pull_cubical = pull_cubical_bool ? 1u : 0u;
    changed = true;
  }

  if (ImGui::TreeNode("Fungus propagation")) {
    if (ImGui::InputFloat("Time Step", &target.dt, 0.0f, 0.0f, "%.5f")) {
      changed = true;
    }
    if (ImGui::InputFloat("Growth rate (white rot)", &target.aw, 0.0f, 0.0f, "%.2f")) {
      changed = true;
    }
    if (ImGui::InputFloat("Growth rate (brown rot)", &target.ab, 0.0f, 0.0f, "%.2f")) {
      changed = true;
    }
    if (ImGui::InputFloat("Chemical defense (white rot)", &target.bw, 0.0f, 0.0f, "%.2f")) {
      changed = true;
    }
    if (ImGui::InputFloat("Chemical defense (brown rot)", &target.bb, 0.0f, 0.0f, "%.2f")) {
      changed = true;
    }
    if (ImGui::InputFloat("Carbon tissue damage (white rot)", &target.ycw, 0.0f, 0.0f, "%.2f")) {
      changed = true;
    }
    if (ImGui::InputFloat("Carbon tissue damage (brown rot)", &target.ycb, 0.0f, 0.0f, "%.2f")) {
      changed = true;
    }
    if (ImGui::InputFloat("Lignin tissue damage (white rot)", &target.ylw, 0.0f, 0.0f, "%.2f")) {
      changed = true;
    }
    if (ImGui::InputFloat("Carbon regeneration", &target.pc, 0.0f, 0.0f, "%.2f")) {
      changed = true;
    }
    if (ImGui::InputFloat("Lignin regeneration", &target.pl, 0.0f, 0.0f, "%.2f")) {
      changed = true;
    }
    if (ImGui::InputFloat("Defense rate", &target.k, 0.0f, 0.0f, "%.2f")) {
      changed = true;
    }
    if (ImGui::InputFloat("Defence decay rate", &target.delta, 0.0f, 0.0f, "%.2f")) {
      changed = true;
    }
    if (ImGui::InputFloat("Lignin weight for white rot", &target.ll, 0.0f, 0.0f, "%.2f")) {
      changed = true;
    }
    if (ImGui::InputFloat("Carbon weight for white rot", &target.lc, 0.0f, 0.0f, "%.2f")) {
      changed = true;
    }
    if (ImGui::InputFloat("Boundary reaction for rot growth", &target.bo, 0.0f, 0.0f, "%.2f")) {
      changed = true;
    }
    if (ImGui::InputFloat("Boundary reaction for propogation", &target.be, 0.0f, 0.0f, "%.2f")) {
      changed = true;
    }
    if (ImGui::InputFloat("Adjustment from carbon to white rot growth", &target.kc, 0.0f, 0.0f, "%.2f")) {
      changed = true;
    }
    if (ImGui::InputFloat("Lignin threshold", &target.HL_threshold, 0.0f, 0.0f, "%.2f")) {
      changed = true;
    }
    if (ImGui::InputFloat("Carbon threshold", &target.HC_threshold, 0.0f, 0.0f, "%.2f")) {
      changed = true;
    }
    if (ImGui::TreeNode("Moisture Settings")) {
      if (ImGui::InputFloat("Base growth rate for white rot", &target.brw, 0.0f, 0.0f, "%.2f")) {
        changed = true;
      }
      if (ImGui::InputFloat("Base growth rate for brown rot", &target.brb, 0.0f, 0.0f, "%.2f")) {
        changed = true;
      }
      if (ImGui::InputFloat("Moisture spread rate", &target.msr, 0.0f, 0.0f, "%.2f")) {
        changed = true;
      }
      ImGui::TreePop();
    }

    bool global_bool = (target.global_parameter != 0);
    if (ImGui::Checkbox("Global parameter", &global_bool)) {
      target.global_parameter = global_bool ? 1u : 0u;
      changed = true;
    }

    // matrixAw: input by columns (glm is column-major)
    ImGui::Text("Diffusion obstruction matrix for white rot");
    float* pAw = glm::value_ptr(target.matrixAw);
    if (ImGui::InputFloat3("matrixAw col0", pAw + 0, "%.2f")) {
      changed = true;
    }
    if (ImGui::InputFloat3("matrixAw col1", pAw + 3, "%.2f")) {
      changed = true;
    }
    if (ImGui::InputFloat3("matrixAw col2", pAw + 6, "%.2f")) {
      changed = true;
    }

    // matrixAb: same pattern as matrixAw
    ImGui::Text("Diffusion obstruction matrix for brown rot");
    float* pAb = glm::value_ptr(target.matrixAb);
    if (ImGui::InputFloat3("matrixAb col0", pAb + 0, "%.2f")) {
      changed = true;
    }
    if (ImGui::InputFloat3("matrixAb col1", pAb + 3, "%.2f")) {
      changed = true;
    }
    if (ImGui::InputFloat3("matrixAb col2", pAb + 6, "%.2f")) {
      changed = true;
    }

    // matrixAc: same pattern as matrixAw
    ImGui::Text("Diffusion obstruction matrix for carbon");
    float* pAc = glm::value_ptr(target.matrixAc);
    if (ImGui::InputFloat3("matrixAc col0", pAc + 0, "%.2f")) {
      changed = true;
    }
    if (ImGui::InputFloat3("matrixAc col1", pAc + 3, "%.2f")) {
      changed = true;
    }
    if (ImGui::InputFloat3("matrixAc col2", pAc + 6, "%.2f")) {
      changed = true;
    }

    ImGui::Text("Diffusion obstruction matrix for carbon");
    float* pAm = glm::value_ptr(target.matrixAm);
    if (ImGui::InputFloat3("matrixAm col0", pAm + 0, "%.2f")) {
      changed = true;
    }
    if (ImGui::InputFloat3("matrixAm col1", pAm + 3, "%.2f")) {
      changed = true;
    }
    if (ImGui::InputFloat3("matrixAm col2", pAm + 6, "%.2f")) {
      changed = true;
    }

    changed |= ImGui::RadioButton("Tree space", &target.treespace, 1);
    changed |= ImGui::RadioButton("Global space", &target.treespace, 0);
    ImGui::TreePop();
  }

  if (ImGui::TreeNode("Crack Parameters:")) {
    if (ImGui::DragFloat("Crack Boundary Shrinkage Offset", &target.crack_bd_shrinkage_offset, 0.01f, 0.0f, 1.0f)) {
      changed = true;
    }
    if (ImGui::DragFloat("Crack R Scale", &target.crack_R_scale, 0.01f, 0.0f, 2.0f)) {
      changed = true;
    }
    if (ImGui::DragFloat("Crack T Scale", &target.crack_T_scale, 0.01f, 0.0f, 2.0f)) {
      changed = true;
    }
    bool internal_pattern_bool = (target.internal_pattern != 0);
    if (ImGui::Checkbox("Simulate internal cracking?", &internal_pattern_bool)) {
      target.internal_pattern = internal_pattern_bool ? 1u : 0u;
      changed = true;
    }

    ImGui::TreePop();
  }

  if (ImGui::TreeNode("Leaf Parameters:")) {
    if (ImGui::DragFloat("Leaf Break Threshold", &target.leaf_break_threshold, 0.01f, 0.0f, 1.0f)) {
      changed = true;
    }
    bool break_from_moisture = (target.leaf_break_from_moisture != 0);
    if (ImGui::Checkbox("Leaf Break from Moisture?", &break_from_moisture)) {
      target.leaf_break_from_moisture = break_from_moisture ? 1u : 0u;
      changed = true;
    }
    ImGui::TreePop();
  }

  if (ImGui::Checkbox("Structural damage", &target.enable_structural_damage)) {
    changed = true;
  }

  if (target.enable_structural_damage) {
    if (ImGui::Checkbox("Segment breaking", &target.enable_segment_breaking)) {
      changed = true;
    }
    if (ImGui::Checkbox("Segment disconnection", &target.enable_segment_disconnection)) {
      changed = true;
    }
    if (target.enable_segment_breaking) {
      if (ImGui::TreeNode("Segment breaking")) {
        if (ImGui::Checkbox("Segment positional breaking", &target.enable_positional_breaking)) {
          changed = true;
        }
        if (ImGui::Checkbox("Segment rotational breaking", &target.enable_rotational_breaking)) {
          changed = true;
        }
        ImGui::TreePop();
      }
    }
    if (target.enable_segment_disconnection) {
      if (ImGui::TreeNode("Segment disconnection")) {
        if (ImGui::Checkbox("Segment tensile disconnection", &target.enable_segment_tensile_disconnection)) {
          changed = true;
        }
        if (ImGui::Checkbox("Segment compression disconnection", &target.enable_segment_compression_disconnection)) {
          changed = true;
        }
        if (target.enable_segment_compression_disconnection) {
          ImGui::DragFloat("Compression strength factor", &target.compression_strength_factor, 0.1f, 0.1f, 1000.f);
        }
        ImGui::TreePop();
      }
    }
    if (ImGui::Checkbox("Foliage detachment", &target.enable_foliage_detachment)) {
      changed = true;
    }
  }

  if (ImGui::Checkbox("Dynamic Grouping", &target.dynamic_grouping)) {
    changed = true;
  }
  if (!target.dynamic_grouping) {
    if (ImGui::DragInt("Grouping iteration", &target.grouping_iteration, 1, 1, 500)) {
      target.grouping_iteration = glm::clamp(target.grouping_iteration, 1, 500);
      changed = true;
    }
  }
  if (ImGui::Checkbox("Segment collision", &target.enable_segment_collision)) {
    changed = true;
  }
  if (ImGui::DragInt("Position constraint iteration", &target.position_constraint_iteration, 1, 1, 50))
    changed = true;
  if (ImGui::DragInt("Velocity constraint iteration", &target.velocity_constraint_iteration, 1, 1, 50))
    changed = true;
  if (ImGui::DragFloat("Segment Velocity damping", &target.segment_velocity_damping, 0.01f, 0.f, 5.f, "%.2f"))
    changed = true;
  if (ImGui::DragFloat("Segment Angular velocity damping", &target.segment_angular_velocity_damping, 0.01f, 0.f, 5.f,
                       "%.2f"))
    changed = true;
  if (ImGui::DragFloat("Leaf Velocity damping", &target.leaf_velocity_damping, 0.0001f, 0.f, 1.f, "%.4f"))
    changed = true;
  if (ImGui::DragFloat("Leaf Angular velocity damping", &target.leaf_angular_velocity_damping, 0.00001f, 0.f, 1.f,
                       "%.5f"))
    changed = true;

  if (ImGui::DragFloat3("Gravity", &target.gravity.x, 1.f))
    changed = true;

  if (ImGui::DragFloat("Pivot ring radius", &target.pivot_ring_radius, 0.02f, 0.f, 10.f, "%.2f"))
    changed = true;

  return changed;
}
