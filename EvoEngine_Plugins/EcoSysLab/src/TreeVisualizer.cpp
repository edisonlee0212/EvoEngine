//
// Created by lllll on 11/20/2022.
//

#include "TreeVisualizer.hpp"
#include "Application.hpp"
#include "EcoSysLabLayer.hpp"
#include "ProfileConstraints.hpp"
#include "Utilities.hpp"
using namespace eco_sys_lab;

bool TreeVisualizer::ScreenCurveSelection(const std::function<void(SkeletonNodeHandle)>& handler,
                                          std::vector<glm::vec2>& mousePositions, ShootSkeleton& skeleton,
                                          const GlobalTransform& globalTransform) {
  auto editorLayer = Application::GetLayer<EditorLayer>();
  auto ecoSysLabLayer = Application::GetLayer<EcoSysLabLayer>();
  const auto cameraRotation = editorLayer->GetSceneCameraRotation();
  const auto cameraPosition = editorLayer->GetSceneCameraPosition();
  const glm::vec3 cameraFront = cameraRotation * glm::vec3(0, 0, -1);
  const glm::vec3 cameraUp = cameraRotation * glm::vec3(0, 1, 0);
  glm::mat4 projectionView = ecoSysLabLayer->m_visualizationCamera->GetProjection() *
                             glm::lookAt(cameraPosition, cameraPosition + cameraFront, cameraUp);

  const auto& sortedInternodeList = skeleton.PeekSortedNodeList();
  bool changed = false;
  for (const auto& internodeHandle : sortedInternodeList) {
    if (internodeHandle == 0)
      continue;
    auto& internode = skeleton.RefNode(internodeHandle);
    if (internode.IsRecycled())
      continue;
    glm::vec3 position = internode.info.global_position;
    auto rotation = internode.info.global_rotation;
    const auto direction = glm::normalize(rotation * glm::vec3(0, 0, -1));
    auto position2 = position + internode.info.length * direction;

    position = (globalTransform.value * glm::translate(position))[3];
    position2 = (globalTransform.value * glm::translate(position2))[3];
    const glm::vec4 internodeScreenStart4 = projectionView * glm::vec4(position, 1.0f);
    const glm::vec4 internodeScreenEnd4 = projectionView * glm::vec4(position2, 1.0f);
    glm::vec3 internodeScreenStart = internodeScreenStart4 / internodeScreenStart4.w;
    glm::vec3 internodeScreenEnd = internodeScreenEnd4 / internodeScreenEnd4.w;
    internodeScreenStart.x *= -1.0f;
    internodeScreenEnd.x *= -1.0f;
    if (internodeScreenStart.x < -1.0f || internodeScreenStart.x > 1.0f || internodeScreenStart.y < -1.0f ||
        internodeScreenStart.y > 1.0f || internodeScreenStart.z < 0.0f)
      continue;
    if (internodeScreenEnd.x < -1.0f || internodeScreenEnd.x > 1.0f || internodeScreenEnd.y < -1.0f ||
        internodeScreenEnd.y > 1.0f || internodeScreenEnd.z < 0.0f)
      continue;
    bool intersect = false;
    for (int i = 0; i < mousePositions.size() - 1; i++) {
      auto& lineStart = mousePositions[i];
      auto& lineEnd = mousePositions[i + 1];
      float a1 = internodeScreenEnd.y - internodeScreenStart.y;
      float b1 = internodeScreenStart.x - internodeScreenEnd.x;
      float c1 = a1 * (internodeScreenStart.x) + b1 * (internodeScreenStart.y);

      // Line CD represented as a2x + b2y = c2
      float a2 = lineEnd.y - lineStart.y;
      float b2 = lineStart.x - lineEnd.x;
      float c2 = a2 * (lineStart.x) + b2 * (lineStart.y);

      float determinant = a1 * b2 - a2 * b1;
      if (determinant == 0.0f)
        continue;
      float x = (b2 * c1 - b1 * c2) / determinant;
      float y = (a1 * c2 - a2 * c1) / determinant;
      if (x <= glm::max(internodeScreenStart.x, internodeScreenEnd.x) &&
          x >= glm::min(internodeScreenStart.x, internodeScreenEnd.x) &&
          y <= glm::max(internodeScreenStart.y, internodeScreenEnd.y) &&
          y >= glm::min(internodeScreenStart.y, internodeScreenEnd.y) && x <= glm::max(lineStart.x, lineEnd.x) &&
          x >= glm::min(lineStart.x, lineEnd.x) && y <= glm::max(lineStart.y, lineEnd.y) &&
          y >= glm::min(lineStart.y, lineEnd.y)) {
        intersect = true;
        break;
      }
    }
    if (intersect) {
      handler(internodeHandle);
      changed = true;
    }
  }
  if (changed) {
    m_selectedInternodeHandle = -1;
    m_selectedInternodeHierarchyList.clear();
  }
  return changed;
}

bool TreeVisualizer::RayCastSelection(const std::shared_ptr<Camera>& cameraComponent, const glm::vec2& mousePosition,
                                      const ShootSkeleton& skeleton, const GlobalTransform& globalTransform) {
  const auto editorLayer = Application::GetLayer<EditorLayer>();
  bool changed = false;
#pragma region Ray selection
  SkeletonNodeHandle currentFocusingNodeHandle = -1;
  std::mutex writeMutex;
  float minDistance = FLT_MAX;
  GlobalTransform cameraLtw;
  cameraLtw.value =
      glm::translate(editorLayer->GetSceneCameraPosition()) * glm::mat4_cast(editorLayer->GetSceneCameraRotation());
  const Ray cameraRay = cameraComponent->ScreenPointToRay(cameraLtw, mousePosition);
  const auto& sortedNodeList = skeleton.PeekSortedNodeList();
  Jobs::RunParallelFor(sortedNodeList.size(), [&](unsigned i) {
    const auto nodeHandle = sortedNodeList[i];
    SkeletonNodeHandle walker = nodeHandle;
    bool subTree = false;
    while (walker != -1) {
      if (walker == m_selectedInternodeHandle) {
        subTree = true;
        break;
      }
      walker = skeleton.PeekNode(walker).GetParentHandle();
    }
    const auto& node = skeleton.PeekNode(nodeHandle);
    auto rotation = globalTransform.GetRotation() * node.info.global_rotation;
    glm::vec3 position = (globalTransform.value * glm::translate(node.info.global_position))[3];
    const auto direction = glm::normalize(rotation * glm::vec3(0, 0, -1));
    const glm::vec3 position2 = position + node.info.length * direction;
    const auto center = (position + position2) / 2.0f;
    auto radius = node.info.thickness;
    if (m_lineThickness != 0.0f) {
      radius = m_lineThickness * (subTree ? 0.625f : 0.5f);
    }
    const auto height = glm::distance(position2, position);
    radius *= height / node.info.length;
    if (!cameraRay.Intersect(center, height / 2.0f) && !cameraRay.Intersect(center, radius)) {
      return;
    }
    const auto& dir = -cameraRay.direction;
#pragma region Line Line intersection
    /*
     * http://geomalgorithms.com/a07-_distance.html
     */
    glm::vec3 v = position - position2;
    glm::vec3 w = (cameraRay.start + dir) - position2;
    const auto a = glm::dot(dir, dir);  // always >= 0
    const auto b = glm::dot(dir, v);
    const auto c = glm::dot(v, v);  // always >= 0
    const auto d = glm::dot(dir, w);
    const auto e = glm::dot(v, w);
    const auto dotP = a * c - b * b;  // always >= 0
    float sc, tc;
    // compute the line parameters of the two closest points
    if (dotP < 0.00001f) {  // the lines are almost parallel
      sc = 0.0f;
      tc = (b > c ? d / b : e / c);  // use the largest denominator
    } else {
      sc = (b * e - c * d) / dotP;
      tc = (a * e - b * d) / dotP;
    }
    // get the difference of the two closest points
    glm::vec3 dP = w + sc * dir - tc * v;  // =  L1(sc) - L2(tc)
    if (glm::length(dP) > radius)
      return;
#pragma endregion

    const auto distance = glm::distance(glm::vec3(cameraLtw.value[3]), glm::vec3(center));
    std::lock_guard<std::mutex> lock(writeMutex);
    if (distance < minDistance) {
      minDistance = distance;
      m_selectedInternodeLengthFactor = glm::clamp(1.0f - tc, 0.0f, 1.0f);
      currentFocusingNodeHandle = sortedNodeList[i];
    }
  });
  if (currentFocusingNodeHandle != -1) {
    SetSelectedNode(skeleton, currentFocusingNodeHandle);
    changed = true;
#pragma endregion
  }
  return changed;
}

void TreeVisualizer::PeekNodeInspectionGui(const ShootSkeleton& skeleton, SkeletonNodeHandle nodeHandle,
                                           const unsigned& hierarchyLevel) {
  const int index = m_selectedInternodeHierarchyList.size() - hierarchyLevel - 1;
  if (!m_selectedInternodeHierarchyList.empty() && index >= 0 && index < m_selectedInternodeHierarchyList.size() &&
      m_selectedInternodeHierarchyList[index] == nodeHandle) {
    ImGui::SetNextItemOpen(true);
  }
  const bool opened = ImGui::TreeNodeEx(
      ("Handle: " + std::to_string(nodeHandle)).c_str(),
      ImGuiTreeNodeFlags_NoTreePushOnOpen | ImGuiTreeNodeFlags_OpenOnArrow | ImGuiTreeNodeFlags_NoAutoOpenOnLog |
          (m_selectedInternodeHandle == nodeHandle ? ImGuiTreeNodeFlags_Framed : ImGuiTreeNodeFlags_FramePadding));
  if (ImGui::IsItemHovered() && ImGui::IsMouseDoubleClicked(0)) {
    SetSelectedNode(skeleton, nodeHandle);
  }
  if (opened) {
    ImGui::TreePush(std::to_string(nodeHandle).c_str());
    const auto& internode = skeleton.PeekNode(nodeHandle);
    const auto& internodeChildren = internode.PeekChildHandles();
    for (const auto& child : internodeChildren) {
      PeekNodeInspectionGui(skeleton, child, hierarchyLevel + 1);
    }
    ImGui::TreePop();
  }
}

void TreeVisualizer::SetSelectedNode(const ShootSkeleton& skeleton, const SkeletonNodeHandle nodeHandle) {
  if (nodeHandle != m_selectedInternodeHandle) {
    m_selectedInternodeHierarchyList.clear();
    if (nodeHandle < 0) {
      m_selectedInternodeHandle = -1;
    } else {
      m_selectedInternodeHandle = nodeHandle;
      auto walker = nodeHandle;
      while (walker != -1) {
        m_selectedInternodeHierarchyList.push_back(walker);
        const auto& internode = skeleton.PeekNode(walker);
        walker = internode.GetParentHandle();
      }
    }
  }
}

void TreeVisualizer::SyncMatrices(const ShootSkeleton& skeleton,
                                  const std::shared_ptr<ParticleInfoList>& particleInfoList,
                                  SkeletonNodeHandle selectedNodeHandle) {
  if (m_randomColors.empty()) {
    for (int i = 0; i < 1000; i++) {
      m_randomColors.emplace_back(glm::abs(glm::ballRand(1.0f)), 1.0f);
    }
  }
  const auto& sortedNodeList = skeleton.PeekSortedNodeList();
  std::vector<ParticleInfo> matrices;

  matrices.resize(sortedNodeList.size());
  Jobs::RunParallelFor(sortedNodeList.size(), [&](unsigned i) {
    const auto nodeHandle = sortedNodeList[i];
    const auto& node = skeleton.PeekNode(nodeHandle);
    bool subTree = false;
    SkeletonNodeHandle walker = nodeHandle;
    while (walker != -1) {
      if (walker == m_selectedInternodeHandle) {
        subTree = true;
        break;
      }
      walker = skeleton.PeekNode(walker).GetParentHandle();
    }
    auto rotation = node.info.global_rotation;
    rotation *= glm::quat(glm::vec3(glm::radians(90.0f), 0.0f, 0.0f));
    const glm::mat4 rotationTransform = glm::mat4_cast(rotation);
    if (m_lineThickness != 0.0f) {
      matrices[i].instance_matrix.value =
          glm::translate(node.info.global_position +
                         (node.info.length / 2.0f) * node.info.GetGlobalDirection()) *
          rotationTransform *
          glm::scale(glm::vec3(m_lineThickness * (subTree ? 1.25f : 1.0f), node.info.length,
                               m_lineThickness * (subTree ? 1.25f : 1.0f)));
    } else {
      matrices[i].instance_matrix.value =
          glm::translate(node.info.global_position +
                         (node.info.length / 2.0f) * node.info.GetGlobalDirection()) *
          rotationTransform *
          glm::scale(glm::vec3(node.info.thickness, node.info.length, node.info.thickness));
    }
  });
  Jobs::RunParallelFor(sortedNodeList.size(), [&](unsigned i) {
    const auto nodeHandle = sortedNodeList[i];
    const auto& node = skeleton.PeekNode(nodeHandle);
    switch (static_cast<ShootVisualizerMode>(m_settings.m_shootVisualizationMode)) {
      case ShootVisualizerMode::Default:
        matrices[i].instance_color = m_randomColors[nodeHandle % m_randomColors.size()];
        break;
      case ShootVisualizerMode::Order:
        matrices[i].instance_color = m_randomColors[node.data.order];
        break;
      case ShootVisualizerMode::Locked:
        matrices[i].instance_color = node.info.locked ? glm::vec4(1, 0, 0, 1) : glm::vec4(0, 1, 0, 1);
        break;
      case ShootVisualizerMode::Level:
        matrices[i].instance_color = m_randomColors[node.data.level];
        break;
      case ShootVisualizerMode::MaxDescendantLightIntensity:
        matrices[i].instance_color =
            glm::mix(glm::vec4(0, 0, 0, 1), glm::vec4(1, 1, 1, 1),
                     glm::clamp(glm::pow(node.data.max_descendant_light_intensity, m_settings.m_shootColorMultiplier),
                                0.0f, 1.f));
        break;
      case ShootVisualizerMode::LightIntensity:
        matrices[i].instance_color =
            glm::mix(glm::vec4(0, 0, 0, 1), glm::vec4(1, 1, 1, 1),
                     glm::clamp(glm::pow(node.data.light_intensity, m_settings.m_shootColorMultiplier), 0.0f, 1.f));
        break;
      case ShootVisualizerMode::LightDirection:
        matrices[i].instance_color = glm::vec4(glm::vec3(glm::clamp(node.data.light_direction, 0.0f, 1.f)), 1.0f);
        break;
      case ShootVisualizerMode::IsMaxChild:
        matrices[i].instance_color = glm::vec4(glm::vec3(node.data.max_child ? 1.0f : 0.0f), 1.0f);
        break;
      case ShootVisualizerMode::DesiredGrowthRate:
        matrices[i].instance_color = glm::mix(
            glm::vec4(0, 1, 0, 1), glm::vec4(1, 0, 0, 1),
            glm::clamp(glm::pow(node.data.desired_growth_rate, m_settings.m_shootColorMultiplier), 0.0f, 1.f));
        break;
      case ShootVisualizerMode::GrowthPotential:
        matrices[i].instance_color =
            glm::mix(glm::vec4(0, 1, 0, 1), glm::vec4(1, 0, 0, 1),
                     glm::clamp(glm::pow(node.data.growth_potential, m_settings.m_shootColorMultiplier), 0.0f, 1.f));
        break;
      case ShootVisualizerMode::SaggingStress:
        matrices[i].instance_color =
            glm::mix(glm::vec4(0, 1, 0, 1), glm::vec4(1, 0, 0, 1),
                     glm::clamp(glm::pow(node.data.sagging_stress, m_settings.m_shootColorMultiplier), 0.0f, 1.f));
        break;
      case ShootVisualizerMode::GrowthRate:
        matrices[i].instance_color =
            glm::mix(glm::vec4(0, 1, 0, 1), glm::vec4(1, 0, 0, 1),
                     glm::clamp(glm::pow(node.data.growth_rate, m_settings.m_shootColorMultiplier), 0.0f, 1.f));
        break;
      default:
        matrices[i].instance_color = m_randomColors[node.data.order];
        break;
    }
    matrices[i].instance_color.a = 1.0f;
    if (selectedNodeHandle != -1)
      matrices[i].instance_color.a = 1.0f;
  });
  particleInfoList->SetParticleInfos(matrices);
}

bool TreeVisualizer::DrawInternodeInspectionGui(TreeModel& treeModel, SkeletonNodeHandle internodeHandle, bool& deleted,
                                                const unsigned& hierarchyLevel) {
  auto& treeSkeleton = treeModel.RefShootSkeleton();
  const int index = m_selectedInternodeHierarchyList.size() - hierarchyLevel - 1;
  if (!m_selectedInternodeHierarchyList.empty() && index >= 0 && index < m_selectedInternodeHierarchyList.size() &&
      m_selectedInternodeHierarchyList[index] == internodeHandle) {
    ImGui::SetNextItemOpen(true);
  }
  const bool opened = ImGui::TreeNodeEx(
      ("Handle: " + std::to_string(internodeHandle)).c_str(),
      ImGuiTreeNodeFlags_NoTreePushOnOpen | ImGuiTreeNodeFlags_OpenOnArrow | ImGuiTreeNodeFlags_NoAutoOpenOnLog |
          (m_selectedInternodeHandle == internodeHandle ? ImGuiTreeNodeFlags_Framed : ImGuiTreeNodeFlags_FramePadding));
  if (ImGui::IsItemHovered() && ImGui::IsMouseDoubleClicked(0)) {
    SetSelectedNode(treeSkeleton, internodeHandle);
  }

  if (ImGui::BeginPopupContextItem(std::to_string(internodeHandle).c_str())) {
    ImGui::Text(("Handle: " + std::to_string(internodeHandle)).c_str());
    if (ImGui::Button("Delete")) {
      deleted = true;
    }
    ImGui::EndPopup();
  }
  bool modified = deleted;
  if (opened && !deleted) {
    ImGui::TreePush(std::to_string(internodeHandle).c_str());
    const auto& internodeChildren = treeSkeleton.RefNode(internodeHandle).PeekChildHandles();
    for (const auto& child : internodeChildren) {
      bool childDeleted = false;
      DrawInternodeInspectionGui(treeModel, child, childDeleted, hierarchyLevel + 1);
      if (childDeleted) {
        treeModel.Step();
        treeModel.PruneInternode(child);

        treeSkeleton.SortLists();
        m_checkpointIteration = treeModel.CurrentIteration();
        modified = true;
        break;
      }
    }
    ImGui::TreePop();
  }
  return modified;
}
void TreeVisualizer::ClearSelections() {
  m_selectedInternodeHandle = -1;
}
bool TreeVisualizer::OnInspect(TreeModel& treeModel) {
  bool updated = false;
  if (ImGui::Combo("Visualizer mode",
                   {"Default", "Order", "Level", "Max descendant light intensity", "Light intensity", "Light direction",
                    "Desired growth rate", "Growth potential", "Growth rate", "Is max child", "Allocated vigor",
                    "Sagging stress", "Locked"},
                   m_settings.m_shootVisualizationMode)) {
    m_needUpdate = true;
  }
  if (ImGui::TreeNodeEx("Checkpoints")) {
    if (ImGui::SliderInt("Current checkpoint", &m_checkpointIteration, 0, treeModel.CurrentIteration())) {
      m_checkpointIteration = glm::clamp(m_checkpointIteration, 0, treeModel.CurrentIteration());
      m_selectedInternodeHandle = -1;
      m_selectedInternodeHierarchyList.clear();
      m_needUpdate = true;
    }
    if (m_checkpointIteration != treeModel.CurrentIteration() && ImGui::Button("Reverse")) {
      treeModel.Reverse(m_checkpointIteration);
      m_needUpdate = true;
    }
    if (ImGui::Button("Clear checkpoints")) {
      m_checkpointIteration = 0;
      treeModel.ClearHistory();
    }
    ImGui::TreePop();
  }
  if (ImGui::Button("Add Checkpoint")) {
    treeModel.Step();
    m_checkpointIteration = treeModel.CurrentIteration();
  }
  if (ImGui::TreeNodeEx("Visualizer Settings")) {
    ImGui::DragInt("History Limit", &treeModel.history_limit, 1, -1, 1024);

    if (ImGui::TreeNode("Shoot Color settings")) {
      if (ImGui::DragFloat("Multiplier", &m_settings.m_shootColorMultiplier, 0.001f)) {
        m_needUpdate = true;
      }
      switch (static_cast<ShootVisualizerMode>(m_settings.m_shootVisualizationMode)) {
        default:
          break;
      }
      ImGui::TreePop();
    }

    ImGui::Checkbox("Visualization", &m_visualization);
    ImGui::Checkbox("Profile", &m_profileGui);
    ImGui::Checkbox("Tree Hierarchy", &m_treeHierarchyGui);

    if (m_visualization) {
      const auto& treeSkeleton = treeModel.PeekShootSkeleton(m_checkpointIteration);
      const auto editorLayer = Application::GetLayer<EditorLayer>();
      const auto& sortedBranchList = treeSkeleton.PeekSortedFlowList();
      const auto& sortedInternodeList = treeSkeleton.PeekSortedNodeList();
      ImGui::Text("Internode count: %d", sortedInternodeList.size());
      ImGui::Text("Shoot stem count: %d", sortedBranchList.size());
    }

    ImGui::TreePop();
  }

  if (ImGui::TreeNodeEx("Inspection")) {
    if (m_selectedInternodeHandle >= 0) {
      if (m_checkpointIteration == treeModel.CurrentIteration()) {
        InspectInternode(treeModel.RefShootSkeleton(), m_selectedInternodeHandle);
      } else {
        PeekInternode(treeModel.PeekShootSkeleton(m_checkpointIteration), m_selectedInternodeHandle);
      }
    }

    if (m_treeHierarchyGui) {
      if (ImGui::TreeNodeEx("Tree Hierarchy")) {
        bool deleted = false;
        auto tempSelection = m_selectedInternodeHandle;
        if (m_checkpointIteration == treeModel.CurrentIteration()) {
          if (DrawInternodeInspectionGui(treeModel, 0, deleted, 0)) {
            m_needUpdate = true;
            updated = true;
          }
        } else
          PeekNodeInspectionGui(treeModel.PeekShootSkeleton(m_checkpointIteration), 0, 0);
        m_selectedInternodeHierarchyList.clear();
        ImGui::TreePop();
      }
    }
    ImGui::TreePop();
  }
  return updated;
}

void TreeVisualizer::Visualize(const TreeModel& treeModel, const GlobalTransform& globalTransform) {
  const auto& treeSkeleton = treeModel.PeekShootSkeleton(m_checkpointIteration);
  if (m_visualization) {
    const auto editorLayer = Application::GetLayer<EditorLayer>();
    const auto ecoSysLabLayer = Application::GetLayer<EcoSysLabLayer>();
    if (m_needUpdate) {
      SyncMatrices(treeSkeleton, m_internodeMatrices, m_selectedInternodeHandle);
      m_needUpdate = false;
    }
    GizmoSettings gizmoSettings;
    gizmoSettings.draw_settings.blending = true;
    gizmoSettings.depth_test = true;
    gizmoSettings.depth_write = true;
    if (!m_internodeMatrices->PeekParticleInfoList().empty()) {
      editorLayer->DrawGizmoMeshInstancedColored(Resources::GetResource<Mesh>("PRIMITIVE_CYLINDER"),
                                                 ecoSysLabLayer->m_visualizationCamera, m_internodeMatrices,
                                                 globalTransform.value, 1.0f, gizmoSettings);
      if (m_selectedInternodeHandle != -1) {
        const auto& node = treeSkeleton.PeekNode(m_selectedInternodeHandle);
        auto rotation = node.info.global_rotation;
        rotation *= glm::quat(glm::vec3(glm::radians(90.0f), 0.0f, 0.0f));
        const glm::mat4 rotationTransform = glm::mat4_cast(rotation);
        const glm::vec3 selectedCenter = node.info.global_position + node.info.length *
                                                                            m_selectedInternodeLengthFactor *
                                                                            node.info.GetGlobalDirection();
        const auto matrix = globalTransform.value * glm::translate(selectedCenter) * rotationTransform *
                            glm::scale(glm::vec3(2.0f * node.info.thickness + 0.01f, node.info.length / 5.0f,
                                                 2.0f * node.info.thickness + 0.01f));
        const auto color = glm::vec4(1.0f);
        editorLayer->DrawGizmoMesh(Resources::GetResource<Mesh>("PRIMITIVE_CYLINDER"),
                                   ecoSysLabLayer->m_visualizationCamera, color, matrix, 1, gizmoSettings);
      }
    }
  }
}

void TreeVisualizer::Visualize(StrandModel& strandModel) {
  if (m_visualization) {
    auto& skeleton = strandModel.strand_model_skeleton;
    static bool showGrid = false;
    if (m_profileGui) {
      const std::string tag = "Profile";
      if (ImGui::Begin(tag.c_str())) {
        if (m_selectedInternodeHandle != -1 && m_selectedInternodeHandle < skeleton.RefRawNodes().size()) {
          auto& node = skeleton.RefNode(m_selectedInternodeHandle);
          glm::vec2 mousePosition{};
          static bool lastFrameClicked = false;
          bool mouseDown = false;
          static bool addAttractor = false;
          ImGui::Checkbox("Attractor", &addAttractor);
          if (ImGui::Button("Clear boundaries")) {
            node.data.profile_constraints.boundaries.clear();
            node.data.boundaries_updated = true;
          }
          ImGui::SameLine();
          if (ImGui::Button("Clear attractors")) {
            node.data.profile_constraints.attractors.clear();
            node.data.boundaries_updated = true;
          }
          ImGui::SameLine();
          ImGui::Checkbox("Show Grid", &showGrid);
          if (node.GetParentHandle() != -1) {
            if (ImGui::Button("Copy from root")) {
              std::vector<SkeletonNodeHandle> parentNodeToRootChain;
              parentNodeToRootChain.emplace_back(m_selectedInternodeHandle);
              SkeletonNodeHandle walker = node.GetParentHandle();
              while (walker != -1) {
                parentNodeToRootChain.emplace_back(walker);
                walker = skeleton.PeekNode(walker).GetParentHandle();
              }
              for (auto it = parentNodeToRootChain.rbegin() + 1; it != parentNodeToRootChain.rend(); ++it) {
                const auto& fromNode = skeleton.PeekNode(*(it - 1));
                auto& toNode = skeleton.RefNode(*it);
                toNode.data.profile_constraints = fromNode.data.profile_constraints;
                toNode.data.boundaries_updated = true;
              }
            }
            const auto& parentNode = skeleton.RefNode(node.GetParentHandle());
            if (!parentNode.data.profile_constraints.boundaries.empty() ||
                !parentNode.data.profile_constraints.attractors.empty()) {
              ImGui::SameLine();
              if (ImGui::Button("Copy parent settings")) {
                node.data.profile_constraints = parentNode.data.profile_constraints;
                node.data.boundaries_updated = true;
              }
            }
          }
          node.data.profile.OnInspect(
              [&](const glm::vec2 position) {
                mouseDown = true;
                mousePosition = position;
              },
              [&](const ImVec2 origin, const float zoomFactor, ImDrawList* drawList) {
                node.data.profile.RenderEdges(origin, zoomFactor, drawList, IM_COL32(0.0f, 0.0f, 128.0f, 128.0f),
                                                  1.0f);
                node.data.profile.RenderBoundary(origin, zoomFactor, drawList,
                                                     IM_COL32(255.f, 255.f, 255.0f, 255.0f), 4.0f);

                if (node.GetParentHandle() != -1) {
                  const auto& parentNode = skeleton.RefNode(node.GetParentHandle());
                  if (!parentNode.data.profile_constraints.boundaries.empty()) {
                    for (const auto& parentBoundary : parentNode.data.profile_constraints.boundaries) {
                      parentBoundary.RenderBoundary(origin, zoomFactor, drawList, IM_COL32(128.0f, 0.0f, 0, 128.0f),
                                                    4.0f);
                    }
                    for (const auto& parentAttractor : parentNode.data.profile_constraints.attractors) {
                      parentAttractor.RenderAttractor(origin, zoomFactor, drawList, IM_COL32(0.0f, 128.0f, 0, 128.0f),
                                                      4.0f);
                    }
                  }
                }
                for (const auto& boundary : node.data.profile_constraints.boundaries) {
                  boundary.RenderBoundary(origin, zoomFactor, drawList, IM_COL32(255.0f, 0.0f, 0, 255.0f), 2.0f);
                }

                for (const auto& attractor : node.data.profile_constraints.attractors) {
                  attractor.RenderAttractor(origin, zoomFactor, drawList, IM_COL32(0.0f, 255.0f, 0, 255.0f), 2.0f);
                }
              },
              showGrid);
          auto& profileBoundaries = node.data.profile_constraints;
          static glm::vec2 attractorStartMousePosition;
          if (lastFrameClicked) {
            if (mouseDown) {
              if (!addAttractor) {
                // Continue recording.
                if (glm::distance(mousePosition, profileBoundaries.boundaries.back().points.back()) > 1.0f)
                  profileBoundaries.boundaries.back().points.emplace_back(mousePosition);
              } else {
                auto& attractorPoints = profileBoundaries.attractors.back().attractor_points;
                if (attractorPoints.empty()) {
                  if (glm::distance(attractorStartMousePosition, mousePosition) > 1.0f) {
                    attractorPoints.emplace_back(attractorStartMousePosition, mousePosition);
                  }
                } else if (glm::distance(mousePosition, attractorPoints.back().second) > 1.0f) {
                  attractorPoints.emplace_back(attractorPoints.back().second, mousePosition);
                }
              }
            } else if (!profileBoundaries.boundaries.empty()) {
              if (!addAttractor) {
                // Stop and check boundary.
                if (!profileBoundaries.Valid(profileBoundaries.boundaries.size() - 1)) {
                  profileBoundaries.boundaries.pop_back();
                } else {
                  profileBoundaries.boundaries.back().CalculateCenter();
                  node.data.boundaries_updated = true;
                }
              } else {
                // Stop and check attractors.
                node.data.boundaries_updated = true;
              }
            }
          } else if (mouseDown) {
            // Start recording.
            if (!addAttractor) {
              node.data.profile_constraints.boundaries.emplace_back();
              node.data.profile_constraints.boundaries.back().points.push_back(mousePosition);
            } else {
              node.data.profile_constraints.attractors.emplace_back();
              attractorStartMousePosition = mousePosition;
            }
          }
          lastFrameClicked = mouseDown;
        } else {
          ImGui::Text("Select an internode to show its profile!");
        }
      }
      ImGui::End();
    }
  }
}

bool TreeVisualizer::InspectInternode(ShootSkeleton& shootSkeleton, SkeletonNodeHandle internodeHandle) {
  bool changed = false;

  auto& internode = shootSkeleton.RefNode(internodeHandle);
  if (internode.info.locked && ImGui::Button("Unlock")) {
    const auto subTree = shootSkeleton.GetSubTree(internodeHandle);
    for (const auto& handle : subTree) {
      shootSkeleton.RefNode(handle).info.locked = false;
    }
    m_needUpdate = true;
  }
  if (!internode.info.locked && ImGui::Button("Lock")) {
    const auto chainToRoot = shootSkeleton.GetChainToRoot(internodeHandle);
    for (const auto& handle : chainToRoot) {
      shootSkeleton.RefNode(handle).info.locked = true;
    }
    m_needUpdate = true;
  }
  if (ImGui::TreeNode("Internode info")) {
    ImGui::Checkbox("Is max child", &internode.data.max_child);
    ImGui::Text("Thickness: %.3f", internode.info.thickness);
    ImGui::Text("Length: %.3f", internode.info.length);
    ImGui::InputFloat3("Position", &internode.info.global_position.x, "%.3f", ImGuiInputTextFlags_ReadOnly);
    auto globalRotationAngle = glm::eulerAngles(internode.info.global_rotation);
    ImGui::InputFloat3("Global rotation", &globalRotationAngle.x, "%.3f", ImGuiInputTextFlags_ReadOnly);
    auto localRotationAngle = glm::eulerAngles(internode.data.desired_local_rotation);
    ImGui::InputFloat3("Local rotation", &localRotationAngle.x, "%.3f", ImGuiInputTextFlags_ReadOnly);
    auto& internodeData = internode.data;
    ImGui::InputFloat("Start Age", &internodeData.start_age, 1, 100, "%.3f", ImGuiInputTextFlags_ReadOnly);
    ImGui::InputFloat("Distance to end", &internode.info.end_distance, 1, 100, "%.3f", ImGuiInputTextFlags_ReadOnly);
    ImGui::InputFloat("Descendent biomass", &internodeData.descendant_total_biomass, 1, 100, "%.3f",
                      ImGuiInputTextFlags_ReadOnly);
    ImGui::InputFloat("Biomass", &internodeData.biomass, 1, 100, "%.3f", ImGuiInputTextFlags_ReadOnly);

    ImGui::InputFloat("Root distance", &internode.info.root_distance, 1, 100, "%.3f", ImGuiInputTextFlags_ReadOnly);

    ImGui::InputFloat("Light Intensity", &internodeData.light_intensity, 1, 100, "%.3f", ImGuiInputTextFlags_ReadOnly);
    ImGui::InputFloat3("Light direction", &internodeData.light_direction.x, "%.3f", ImGuiInputTextFlags_ReadOnly);

    ImGui::InputFloat("Growth rate control", &internodeData.growth_potential, 1, 100, "%.3f",
                      ImGuiInputTextFlags_ReadOnly);
    ImGui::InputFloat("Desired growth rate", &internodeData.desired_growth_rate, 1, 100, "%.3f",
                      ImGuiInputTextFlags_ReadOnly);
    ImGui::InputFloat("Growth rate", &internodeData.growth_rate, 1, 100, "%.3f", ImGuiInputTextFlags_ReadOnly);
    ImGui::InputFloat("Sagging Stress", &internodeData.sagging_stress, 1, 100, "%.3f", ImGuiInputTextFlags_ReadOnly);
    if (ImGui::DragFloat("Sagging", &internodeData.sagging)) {
      changed = true;
    }
    if (ImGui::DragFloat("Extra mass", &internodeData.extra_mass)) {
      changed = true;
    }

    if (ImGui::TreeNodeEx("Buds")) {
      int index = 1;
      for (auto& bud : internodeData.buds) {
        if (ImGui::TreeNode(("Bud " + std::to_string(index)).c_str())) {
          switch (bud.type) {
            case BudType::Apical:
              ImGui::Text("Apical");
              break;
            case BudType::Lateral:
              ImGui::Text("Lateral");
              break;
            case BudType::Leaf:
              ImGui::Text("Leaf");
              break;
            case BudType::Fruit:
              ImGui::Text("Fruit");
              break;
          }
          switch (bud.status) {
            case BudStatus::Dormant:
              ImGui::Text("Dormant");
              break;

            case BudStatus::Died:
              ImGui::Text("Died");
              break;
          }

          auto budRotationAngle = glm::eulerAngles(bud.local_rotation);
          ImGui::InputFloat3("Rotation", &budRotationAngle.x, "%.3f", ImGuiInputTextFlags_ReadOnly);
          /*
          ImGui::InputFloat("Base resource requirement", (float *) &bud.m_maintenanceVigorRequirementWeight, 1, 100,
                                                  "%.3f", ImGuiInputTextFlags_ReadOnly);
                                                  */
          ImGui::TreePop();
        }
        index++;
      }
      ImGui::TreePop();
    }
    ImGui::TreePop();
  }
  if (ImGui::TreeNodeEx("Flow info")) {
    const auto& flow = shootSkeleton.PeekFlow(internode.GetFlowHandle());
    ImGui::Text("Child flow size: %d", flow.PeekChildHandles().size());
    ImGui::Text("Internode size: %d", flow.PeekNodeHandles().size());
    if (ImGui::TreeNode("Internodes")) {
      int i = 0;
      for (const auto& chainedInternodeHandle : flow.PeekNodeHandles()) {
        ImGui::Text("No.%d: Handle: %d", i, chainedInternodeHandle);
        i++;
      }
      ImGui::TreePop();
    }
    ImGui::TreePop();
  }
  return changed;
}

void TreeVisualizer::PeekInternode(const ShootSkeleton& shootSkeleton, SkeletonNodeHandle internodeHandle) const {
  const auto& internode = shootSkeleton.PeekNode(internodeHandle);
  if (ImGui::TreeNode("Internode info")) {
    ImGui::Checkbox("Is max child", (bool*)&internode.data.max_child);
    ImGui::Text("Thickness: %.3f", internode.info.thickness);
    ImGui::Text("Length: %.3f", internode.info.length);
    ImGui::InputFloat3("Position", (float*)&internode.info.global_position.x, "%.3f", ImGuiInputTextFlags_ReadOnly);
    auto globalRotationAngle = glm::eulerAngles(internode.info.global_rotation);
    ImGui::InputFloat3("Global rotation", (float*)&globalRotationAngle.x, "%.3f", ImGuiInputTextFlags_ReadOnly);
    auto localRotationAngle = glm::eulerAngles(internode.data.desired_local_rotation);
    ImGui::InputFloat3("Local rotation", (float*)&localRotationAngle.x, "%.3f", ImGuiInputTextFlags_ReadOnly);
    auto& internodeData = internode.data;
    ImGui::InputInt("Start Age", (int*)&internodeData.start_age, 1, 100, ImGuiInputTextFlags_ReadOnly);
    ImGui::InputFloat("Sagging", (float*)&internodeData.sagging, 1, 100, "%.3f", ImGuiInputTextFlags_ReadOnly);
    ImGui::InputFloat("Distance to end", (float*)&internode.info.end_distance, 1, 100, "%.3f",
                      ImGuiInputTextFlags_ReadOnly);
    ImGui::InputFloat("Descendent biomass", (float*)&internodeData.descendant_total_biomass, 1, 100, "%.3f",
                      ImGuiInputTextFlags_ReadOnly);
    ImGui::InputFloat("Biomass", (float*)&internodeData.biomass, 1, 100, "%.3f", ImGuiInputTextFlags_ReadOnly);
    ImGui::InputFloat("Root distance", (float*)&internode.info.root_distance, 1, 100, "%.3f",
                      ImGuiInputTextFlags_ReadOnly);
    ImGui::InputFloat3("Light dir", (float*)&internodeData.light_direction.x, "%.3f", ImGuiInputTextFlags_ReadOnly);
    ImGui::InputFloat("Growth Potential", (float*)&internodeData.light_intensity, 1, 100, "%.3f",
                      ImGuiInputTextFlags_ReadOnly);

    if (ImGui::TreeNodeEx("Buds")) {
      int index = 1;
      for (auto& bud : internodeData.buds) {
        if (ImGui::TreeNode(("Bud " + std::to_string(index)).c_str())) {
          switch (bud.type) {
            case BudType::Apical:
              ImGui::Text("Apical");
              break;
            case BudType::Lateral:
              ImGui::Text("Lateral");
              break;
            case BudType::Leaf:
              ImGui::Text("Leaf");
              break;
            case BudType::Fruit:
              ImGui::Text("Fruit");
              break;
          }
          switch (bud.status) {
            case BudStatus::Dormant:
              ImGui::Text("Dormant");
              break;

            case BudStatus::Died:
              ImGui::Text("Died");
              break;
          }

          auto budRotationAngle = glm::eulerAngles(bud.local_rotation);
          ImGui::InputFloat3("Rotation", &budRotationAngle.x, "%.3f", ImGuiInputTextFlags_ReadOnly);
          /*
          ImGui::InputFloat("Base resource requirement", (float *) &bud.m_maintenanceVigorRequirementWeight, 1, 100,
                                                  "%.3f", ImGuiInputTextFlags_ReadOnly);
                                                  */
          ImGui::TreePop();
        }
        index++;
      }
      ImGui::TreePop();
    }
    ImGui::TreePop();
  }
  if (ImGui::TreeNodeEx("Stem info", ImGuiTreeNodeFlags_DefaultOpen)) {
    const auto& flow = shootSkeleton.PeekFlow(internode.GetFlowHandle());
    ImGui::Text("Child stem size: %d", flow.PeekChildHandles().size());
    ImGui::Text("Internode size: %d", flow.PeekNodeHandles().size());
    if (ImGui::TreeNode("Internodes")) {
      int i = 0;
      for (const auto& chainedInternodeHandle : flow.PeekNodeHandles()) {
        ImGui::Text("No.%d: Handle: %d", i, chainedInternodeHandle);
        i++;
      }
      ImGui::TreePop();
    }
    ImGui::TreePop();
  }
}

void TreeVisualizer::Reset(TreeModel& treeModel) {
  m_selectedInternodeHandle = -1;
  m_selectedInternodeHierarchyList.clear();
  m_checkpointIteration = treeModel.CurrentIteration();
  m_internodeMatrices->SetParticleInfos({});
  m_needUpdate = true;
}

void TreeVisualizer::Clear() {
  m_selectedInternodeHandle = -1;
  m_selectedInternodeHierarchyList.clear();
  m_checkpointIteration = 0;
  m_internodeMatrices->SetParticleInfos({});
}

bool TreeVisualizer::Initialized() const {
  return m_initialized;
}

void TreeVisualizer::Initialize() {
  m_settings = {};
  m_internodeMatrices = ProjectManager::CreateTemporaryAsset<ParticleInfoList>();
}
