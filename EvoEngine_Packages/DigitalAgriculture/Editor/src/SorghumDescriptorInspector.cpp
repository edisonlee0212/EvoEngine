#include "DigitalAgricultureInspectionAdapters.hpp"
#include "DigitalAgricultureInspectorStates.hpp"
#include "DigitalAgricultureSerializationAdapters.hpp"
#include "EditorFileDialogs.hpp"
#include "IVolume.hpp"
#include "Sorghum.hpp"
#include "SorghumDescriptorReconstruction.hpp"
#include "SorghumLayer.hpp"
#include "assimp/code/AssetLib/3MF/3MFXmlTags.h"
using namespace digital_agriculture_package;
bool digital_agriculture_package::DrawSorghumMeshGeneratorSettingsGui(SorghumMeshGeneratorSettings& settings) {
  if (ImGui::TreeNode("Sorghum mesh generator settings")) {
    ImGui::Checkbox("Panicle", &settings.enable_panicle);
    ImGui::Checkbox("Stem", &settings.enable_stem);
    ImGui::Checkbox("Leaves", &settings.enable_leaves);
    if (settings.enable_leaves) {
      ImGui::Checkbox("Leaves sheath", &settings.enable_leaf_sheath);
    }
    ImGui::Checkbox("Bottom Face", &settings.bottom_face);
    ImGui::Checkbox("Leaf separated", &settings.leaf_separated);
    ImGui::DragFloat("Leaf thickness", &settings.leaf_thickness, 0.0001f);
    ImGui::TreePop();
  }
  return false;
}
bool digital_agriculture_package::DrawSorghumPanicleDescriptorGui(SorghumPanicleDescriptor& descriptor) {
  (void)descriptor;
  return false;
}
bool digital_agriculture_package::DrawSorghumStemDescriptorGui(SorghumStemDescriptor& descriptor) {
  bool changed = false;
  for (int i = 0; i < descriptor.spline.segments.size(); i++) {
    auto segment = descriptor.spline.segments[i];
    std::string label = "segment No." + std::to_string(i);
    if (ImGui::TreeNode(label.c_str())) {
      ImGui::Text("position: (%.2f, %.2f, %.2f)", segment.position.x, segment.position.y, segment.position.z);
      ImGui::Text("up: (%.2f, %.2f, %.2f)", segment.up.x, segment.up.y, segment.up.z);
      ImGui::Text("front: (%.2f, %.2f, %.2f)", segment.front.x, segment.front.y, segment.front.z);
      ImGui::Text("radius: %.2f", segment.radius);
      ImGui::Text("theta: (%.2f)", segment.theta);
      ImGui::Text("left height offset: (%.2f)", segment.left_height_offset);
      ImGui::Text("right height offset: (%.2f)", segment.right_height_offset);

      changed = true;
      ImGui::TreePop();
    }
  }

  return changed;
}
bool digital_agriculture_package::DrawSorghumLeafDescriptorGui(SorghumLeafDescriptor& descriptor) {
  bool changed = false;
  for (int i = 0; i < descriptor.spline.segments.size(); i++) {
    const auto segment = descriptor.spline.segments[i];
    std::string label = "segment No." + std::to_string(i);
    if (ImGui::TreeNode(label.c_str())) {
      ImGui::Text("position: (%.2f, %.2f, %.2f)", segment.position.x, segment.position.y, segment.position.z);
      ImGui::Text("up: (%.2f, %.2f, %.2f)", segment.up.x, segment.up.y, segment.up.z);
      ImGui::Text("front: (%.2f, %.2f, %.2f)", segment.front.x, segment.front.y, segment.front.z);
      ImGui::Text("radius: %.2f", segment.radius);
      ImGui::Text("theta: (%.2f)", segment.theta);
      ImGui::Text("left height offset: (%.2f)", segment.left_height_offset);
      ImGui::Text("right height offset: (%.2f)", segment.right_height_offset);

      changed = true;
      ImGui::TreePop();
    }
  }

  return changed;
}
bool digital_agriculture_package::InspectSorghumDescriptor(InspectorContext& context, SorghumDescriptor& descriptor) {
  (void)context;
  if (ImGui::Button("Instantiate")) {
    descriptor.CreateEntity("New Sorghum");
  }
  // after load from spline, replace data in sorghumdescriptor
  EditorFileDialogs::OpenFile(
      "Load splines", "YAML", {".yml"},
      [&descriptor](const std::filesystem::path& path) {
        // @edisonlee0212: here I reconstruct the sorghum descriptor from yaml and create the mesh.
        if (auto temp_result = descriptor.ImportPrediction(path)) {
          SorghumDescriptorReconstruction reconstruction;
          auto yaml_content = *temp_result;
          std::cout << "imported from yaml"
                    << "\n"
                    << "leaf count: " << yaml_content.size() << "\n"
                    << "total points: " << yaml_content[0]["centerPoints"].size() * yaml_content.size() * 3 << "\n";

          auto splines = SorghumDescriptorReconstruction::ReconstructBezierSplineFromYaml(yaml_content);
          SorghumDescriptor temp;
          auto bezier_sample_results = reconstruction.ReconstructSorghumFromBezierSplines(temp, splines);
          reconstruction.ReconstructSorghumStem(temp);

          // todo: may need have a copy constructor
          descriptor.leaves = temp.leaves;
          descriptor.stem = temp.stem;
          descriptor.panicle = temp.panicle;
          descriptor.CreateEntity("New Sorghum");
        }
      },
      false);
  bool changed = false;
  if (ImGui::TreeNodeEx((std::string("Stem")).c_str())) {
    if (DrawSorghumStemDescriptorGui(descriptor.stem))
      changed = true;
    ImGui::TreePop();
  }

  if (ImGui::TreeNodeEx("Leaves")) {
    int leaf_size = descriptor.leaves.size();
    if (ImGui::InputInt("Number of leaves", &leaf_size)) {
      changed = true;
      leaf_size = glm::clamp(leaf_size, 0, 999);
      const auto previous_size = descriptor.leaves.size();
      descriptor.leaves.resize(leaf_size);
      for (int i = 0; i < leaf_size; i++) {
        if (i >= previous_size) {
          if (i - 1 >= 0) {
            descriptor.leaves[i] = descriptor.leaves[i - 1];
            /*
            leaves[i].m_rollAngle =
                    glm::mod(leaves[i - 1].m_rollAngle + 180.0f, 360.0f);
            leaves[i].m_startingPoint =
                    leaves[i - 1].m_startingPoint + 0.1f;*/
          } else {
            descriptor.leaves[i] = {};
            /*
            leaves[i].m_rollAngle = 0;
            leaves[i].m_startingPoint = 0.1f;*/
          }
        }
        descriptor.leaves[i].index = i;
      }
    }
    for (auto& leaf : descriptor.leaves) {
      if (ImGui::TreeNode(
              ("Leaf No." + std::to_string(leaf.index + 1) + (leaf.spline.segments.empty() ? " (Dead)" : ""))
                  .c_str())) {
        if (DrawSorghumLeafDescriptorGui(leaf))
          changed = true;
        ImGui::TreePop();
      }
    }
    ImGui::TreePop();
  }

  if (ImGui::TreeNodeEx((std::string("Panicle")).c_str())) {
    if (DrawSorghumPanicleDescriptorGui(descriptor.panicle))
      changed = true;
    ImGui::TreePop();
  }

  return changed;
}
