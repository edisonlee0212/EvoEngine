#include "ProceduralNoise.hpp"

#include "SkeletonSerializer.hpp"

using namespace eco_sys_lab;
bool ValueTypeSelection(ProceduralNoiseValueType& target, const std::string& tag) {
  if (unsigned value_type = static_cast<unsigned>(target);
      ImGui::Combo("Value Type" + tag, {"Constant", "Linear", "Sine", "Tangent", "Simplex", "Perlin"}, value_type)) {
    target = static_cast<ProceduralNoiseValueType>(value_type);
    return true;
  }
  return false;
}

bool OperatorTypeSelection(ProceduralNoiseOperatorType& target, const std::string& tag) {
  unsigned operatorType = static_cast<unsigned>(target);
  if (ImGui::Combo(
          "Operator Type" + tag,
          {"None", "Reset", "Add", "Subtract", "Multiply", "Divide", "Pow", "Min", "Max", "Flip Up", "Flip Down"},
          operatorType)) {
    target = static_cast<ProceduralNoiseOperatorType>(operatorType);
    return true;
  }
  return false;
}
bool ProceduralNoise2D::OnInspect(const SkeletonNodeHandle node_handle) {
  bool changed = false;

  auto& node = m_pipeline.RefNode(node_handle);
  const std::string prefix = "[" + std::to_string(node_handle) + "] ";
  const std::string tag = "##ProceduralNoise3DNode" + std::to_string(node_handle);
  if (ImGui::TreeNode((prefix + node.data.m_name + "'s settings" + tag).c_str())) {
    if (node_handle != 0 && ImGui::Button(("Remove" + tag).c_str())) {
      m_pipeline.RecycleNode(
          node_handle,
          [&](SkeletonFlowHandle) {
          },
          [&](SkeletonNodeHandle) {
          });
      ImGui::TreePop();
      return true;
    }
    if (ImGui::BeginPopupContextItem(tag.c_str())) {
      if (ImGui::BeginMenu(("Rename" + tag).c_str())) {
        static char new_name[256];
        ImGui::InputText(("New name" + tag).c_str(), new_name, 256);
        if (ImGui::Button(("Confirm" + tag).c_str())) {
          node.data.m_name = new_name;
          memset(new_name, 0, 256);
        }
        ImGui::EndMenu();
      }
      ImGui::EndPopup();
    }
    OperatorTypeSelection(node.data.m_operatorType, tag);
    if (node.data.m_operatorType != ProceduralNoiseOperatorType::None)
      ValueTypeSelection(node.data.m_valueType, tag);
    if (node.data.m_valueType != ProceduralNoiseValueType::Constant) {
      if (ImGui::DragFloat2(("Offset" + tag).c_str(), &node.data.m_offset.x, 0.01f))
        changed = true;
      if (ImGui::DragFloat2(("Frequency" + tag).c_str(), &node.data.m_frequency.x, 0.01f))
        changed = true;
    } else {
      if (ImGui::DragFloat(("Value" + tag).c_str(), &node.data.m_constantValue, 0.01f))
        changed = true;
    }

    if (ImGui::TreeNode((prefix + node.data.m_name + "'s children" + tag).c_str())) {
      auto childHandles = node.PeekChildHandles();
      for (int i = 0; i < childHandles.size(); i++) {
        const std::string childTag = "##ProceduralNoise3DNode" + std::to_string(childHandles.at(i));
        if (ImGui::Button(("New..." + childTag + "F").c_str())) {
          changed = true;
          const auto newNodeHandle = m_pipeline.Extend(node_handle, true);
          childHandles.insert(childHandles.begin() + i, newNodeHandle);
          m_pipeline.RefNode(node_handle).UnsafeRefChildHandles() = childHandles;
          break;
        }
        if (OnInspect(childHandles.at(i))) {
          changed = true;
        }
      }
      if (ImGui::Button(("New..." + tag).c_str())) {
        changed = true;
        const auto newNodeHandle = m_pipeline.Extend(node_handle, true);
      }
      ImGui::TreePop();
    }
    ImGui::TreePop();
  }
  return changed;
}

bool ProceduralNoise3D::OnInspect(const SkeletonNodeHandle nodeHandle) {
  bool changed = false;
  auto& node = m_pipeline.RefNode(nodeHandle);
  const std::string prefix = "[" + std::to_string(nodeHandle) + "] ";
  const std::string tag = "##ProceduralNoise3DNode" + std::to_string(nodeHandle);
  if (ImGui::TreeNode((prefix + node.data.m_name + "'s settings" + tag).c_str())) {
    if (nodeHandle != 0 && ImGui::Button(("Remove" + tag).c_str())) {
      m_pipeline.RecycleNode(
          nodeHandle,
          [&](SkeletonFlowHandle) {
          },
          [&](SkeletonNodeHandle) {
          });
      ImGui::TreePop();
      return true;
    }
    if (ImGui::BeginPopupContextItem(tag.c_str())) {
      if (ImGui::BeginMenu(("Rename" + tag).c_str())) {
        static char newName[256];
        ImGui::InputText(("New name" + tag).c_str(), newName, 256);
        if (ImGui::Button(("Confirm" + tag).c_str())) {
          node.data.m_name = newName;
          memset(newName, 0, 256);
        }
        ImGui::EndMenu();
      }
      ImGui::EndPopup();
    }
    OperatorTypeSelection(node.data.m_operatorType, tag);
    if (node.data.m_operatorType != ProceduralNoiseOperatorType::None)
      ValueTypeSelection(node.data.m_valueType, tag);

    if (node.data.m_valueType != ProceduralNoiseValueType::Constant) {
      if (ImGui::DragFloat3(("Offset" + tag).c_str(), &node.data.m_offset.x, 0.01f))
        changed = true;
      if (ImGui::DragFloat3(("Frequency" + tag).c_str(), &node.data.m_frequency.x, 0.01f))
        changed = true;
    } else {
      if (ImGui::DragFloat(("Value" + tag).c_str(), &node.data.m_constantValue, 0.01f))
        changed = true;
    }

    if (ImGui::TreeNode((prefix + node.data.m_name + "'s children" + tag).c_str())) {
      auto childHandles = node.PeekChildHandles();
      for (int i = 0; i < childHandles.size(); i++) {
        const std::string childTag = "##ProceduralNoise3DNode" + std::to_string(childHandles.at(i));
        if (ImGui::Button(("New..." + childTag + "F").c_str())) {
          changed = true;
          const auto newNodeHandle = m_pipeline.Extend(nodeHandle, true);
          childHandles.insert(childHandles.begin() + i, newNodeHandle);
          m_pipeline.RefNode(nodeHandle).UnsafeRefChildHandles() = childHandles;
          break;
        }
        if (OnInspect(childHandles.at(i))) {
          changed = true;
        }
      }
      if (ImGui::Button(("New..." + tag).c_str())) {
        changed = true;
        const auto newNodeHandle = m_pipeline.Extend(nodeHandle, true);
      }
      ImGui::TreePop();
    }
    ImGui::TreePop();
  }
  return changed;
}

float ProceduralNoise2D::Process(const SkeletonNodeHandle nodeHandle, const glm::vec2& samplePoint, float value) {
  const auto& node = m_pipeline.RefNode(nodeHandle);
  const auto& childHandles = node.PeekChildHandles();
  if (!childHandles.empty()) {
    for (const auto& childHandle : childHandles) {
      value = Process(childHandle, samplePoint, value);
      node.data.Calculate(samplePoint, value);
    }
  } else {
    node.data.Calculate(samplePoint, value);
  }
  return value;
}

float ProceduralNoise3D::Process(const SkeletonNodeHandle nodeHandle, const glm::vec3& samplePoint, float value) {
  const auto& node = m_pipeline.RefNode(nodeHandle);
  const auto& childHandles = node.PeekChildHandles();
  if (!childHandles.empty()) {
    for (const auto& childHandle : childHandles) {
      value = Process(childHandle, samplePoint, value);
      node.data.Calculate(samplePoint, value);
    }
  } else {
    node.data.Calculate(samplePoint, value);
  }
  return value;
}

void ProceduralNoise2D::Serialize(YAML::Emitter& out) const {
  out << YAML::Key << "m_pipeline" << YAML::Value << YAML::BeginMap;
  {
    SkeletonSerializer<ProceduralNoiseSkeletonData, ProceduralNoiseFlowData, ProceduralNoiseStage<glm::vec2>>::
        Serialize(
            out, m_pipeline,
            [&](YAML::Emitter& nodeOut, const ProceduralNoiseStage<glm::vec2>& nodeData) {
              nodeData.Save("data", nodeOut);
            },
            [&](YAML::Emitter& flowOut, const ProceduralNoiseFlowData& flowData) {
            },
            [&](YAML::Emitter& skeletonOut, const ProceduralNoiseSkeletonData& skeletonData) {
            });
  }
  out << YAML::EndMap;
}

void ProceduralNoise2D::Deserialize(const YAML::Node& in) {
  if (in["m_pipeline"]) {
    const auto& inPipeline = in["m_pipeline"];
    SkeletonSerializer<ProceduralNoiseSkeletonData, ProceduralNoiseFlowData, ProceduralNoiseStage<glm::vec2>>::
        Deserialize(
            inPipeline, m_pipeline,
            [&](const YAML::Node& nodeIn, ProceduralNoiseStage<glm::vec2>& nodeData) {
              nodeData.Load("data", nodeIn);
            },
            [&](const YAML::Node& flowIn, ProceduralNoiseFlowData& flowData) {
            },
            [&](const YAML::Node& skeletonIn, ProceduralNoiseSkeletonData& skeletonData) {
            });
  }
}

bool ProceduralNoise2D::OnInspect(const std::shared_ptr<EditorLayer>& editorLayer) {
  bool changed = false;
  static NodeGraph<int, int, int, int> graph{};
  static bool showNodeGraph = false;
  if (showNodeGraph) {
    graph.OnInspect(
        "Node Graph", editorLayer,
        [&](const ImVec2 clickPos) {
          if (ImGui::MenuItem("add")) {
            const auto newNodeHandle = graph.AllocateNode(3, true);

            ImNodes::SetNodeScreenSpacePos(newNodeHandle, clickPos);
          }
        },
        [&](NodeGraphNodeHandle nodeHandle) {
          ImGui::Text("Node");
        },
        [&](NodeGraphInputPinHandle inputPinHandle) {
          ImGui::Text("Input");
        },
        [&](NodeGraphOutputPinHandle outputPinHandle) {
          ImGui::Text("Output");
        },
        [&](NodeGraphOutputPinHandle startHandle, NodeGraphInputPinHandle endHandle) {
          graph.AllocateLink(startHandle, endHandle);
        },
        [&](NodeGraphLinkHandle linkHandle) {
          graph.RecycleLink(linkHandle);
        },
        [&](NodeGraphNodeHandle nodeHandle, NodeGraphLinkHandle linkHandle, NodeGraphInputPinHandle inputPinHandle,
            NodeGraphOutputPinHandle outputPinHandle) {

        },
        [&](const std::vector<NodeGraphNodeHandle>& selectedNodeHandles,
            const std::vector<NodeGraphLinkHandle>& selectedLinkHandles) {

        });
  }

  if (m_pipeline.RefRawNodes().empty() || m_pipeline.PeekNode(0).IsRecycled()) {
    return changed;
  }
  static glm::vec2 testPoint{};
  static float initialValue = 0.f;
  ImGui::DragFloat2("Test Point", &testPoint.x, 0.1f);
  ImGui::DragFloat("Input Value", &initialValue, 0.1f);
  float value = initialValue;
  Process(testPoint, value);
  ImGui::Text("Value: %.3f", value);

  ImGui::Separator();
  if (OnInspect(0))
    changed = true;

  return changed;
}

float ProceduralNoise2D::Process(const glm::vec2& samplePoint, float value) {
  if (m_pipeline.RefRawNodes().empty() || m_pipeline.PeekNode(0).IsRecycled()) {
    EVOENGINE_WARNING("Pipeline is empty!");
    return value;
  }
  return Process(0, samplePoint, value);
}

float ProceduralNoise3D::Process(const glm::vec3& samplePoint, float value) {
  if (m_pipeline.RefRawNodes().empty() || m_pipeline.PeekNode(0).IsRecycled()) {
    EVOENGINE_WARNING("Pipeline is empty!");
    return value;
  }
  return Process(0, samplePoint, value);
}

void ProceduralNoise3D::Serialize(YAML::Emitter& out) const {
  out << YAML::Key << "m_pipeline" << YAML::Value << YAML::BeginMap;
  {
    SkeletonSerializer<ProceduralNoiseSkeletonData, ProceduralNoiseFlowData, ProceduralNoiseStage<glm::vec3>>::
        Serialize(
            out, m_pipeline,
            [&](YAML::Emitter& nodeOut, const ProceduralNoiseStage<glm::vec3>& nodeData) {
              nodeData.Save("data", nodeOut);
            },
            [&](YAML::Emitter& flowOut, const ProceduralNoiseFlowData& flowData) {
            },
            [&](YAML::Emitter& skeletonOut, const ProceduralNoiseSkeletonData& skeletonData) {
            });
  }
  out << YAML::EndMap;
}

void ProceduralNoise3D::Deserialize(const YAML::Node& in) {
  if (in["m_pipeline"]) {
    const auto& in_pipeline = in["m_pipeline"];
    SkeletonSerializer<ProceduralNoiseSkeletonData, ProceduralNoiseFlowData, ProceduralNoiseStage<glm::vec3>>::
        Deserialize(
            in_pipeline, m_pipeline,
            [&](const YAML::Node& node_in, ProceduralNoiseStage<glm::vec3>& node_data) {
              node_data.Load("data", node_in);
            },
            [&](const YAML::Node& flow_in, ProceduralNoiseFlowData& flow_data) {
            },
            [&](const YAML::Node& skeleton_in, ProceduralNoiseSkeletonData& skeleton_data) {
            });
  }
}

bool ProceduralNoise3D::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  bool changed = false;
  if (m_pipeline.RefRawNodes().empty() || m_pipeline.PeekNode(0).IsRecycled()) {
    return changed;
  }
  static glm::vec3 test_point{};
  ImGui::DragFloat3("Test Point", &test_point.x, 0.1f);
  float value = 0.f;
  Process(test_point, value);
  ImGui::Text("Value: %.3f", value);
  ImGui::Separator();
  if (OnInspect(0))
    changed = true;
  return changed;
}
