#include "SpatialPlantDistributionSimulator.hpp"
#include "Tree.hpp"
#include "TreeDescriptor.hpp"

using namespace eco_sys_lab;

void SpatialPlantDistributionSimulator::OnInspectSpatialPlantDistributionFunction(
    const SpatialPlantDistribution& spatialPlantDistribution, const std::function<void(glm::vec2 position)>& func,
    const std::function<void(ImVec2 origin, float zoomFactor, ImDrawList*)>& drawFunc) {
  static auto scrolling = glm::vec2(0.0f);
  static float zoomFactor = 2.f;
  static bool enableUniformSize = false;
  static float uniformSize = 4.f;
  static float plantSizeFactor = 4.f;
  ImGui::Checkbox("Uniform size", &enableUniformSize);
  ImGui::SameLine();
  if (enableUniformSize) {
    ImGui::DragFloat("Uniform size", &uniformSize);
  } else {
    ImGui::DragFloat("Plant size factor", &plantSizeFactor, 0.01f, 0.1f, 1.0f);
  }
  ImGui::SameLine();
  ImGui::Text(
      ("Plant count: " +
       std::to_string(spatialPlantDistribution.m_plants.size() - spatialPlantDistribution.m_recycledPlants.size()) +
       " | Simulation time: " + std::to_string(spatialPlantDistribution.m_simulationTime))
          .c_str());
  if (ImGui::Button("Recenter")) {
    scrolling = glm::vec2(0.0f);
  }
  ImGui::SameLine();
  ImGui::DragFloat("Zoom", &zoomFactor, zoomFactor / 100.0f, 0.001f, 10.0f);
  zoomFactor = glm::clamp(zoomFactor, 0.01f, 1000.0f);
  const ImGuiIO& io = ImGui::GetIO();
  ImDrawList* drawList = ImGui::GetWindowDrawList();

  const ImVec2 canvasP0 = ImGui::GetCursorScreenPos();  // ImDrawList API uses screen coordinates!
  ImVec2 canvasSz = ImGui::GetContentRegionAvail();     // Resize canvas to what's available
  if (canvasSz.x < 300.0f)
    canvasSz.x = 300.0f;
  if (canvasSz.y < 300.0f)
    canvasSz.y = 300.0f;
  const ImVec2 canvasP1 = ImVec2(canvasP0.x + canvasSz.x, canvasP0.y + canvasSz.y);
  const ImVec2 origin(canvasP0.x + canvasSz.x / 2.0f + scrolling.x,
                      canvasP0.y + canvasSz.y / 2.0f + scrolling.y);  // Lock scrolled origin
  const ImVec2 mousePosInCanvas((io.MousePos.x - origin.x) / zoomFactor, (io.MousePos.y - origin.y) / zoomFactor);

  // Draw border and background color
  drawList->AddRectFilled(canvasP0, canvasP1, IM_COL32(255, 255, 255, 255));
  drawList->AddRect(canvasP0, canvasP1, IM_COL32(255, 255, 255, 255));

  // This will catch our interactions
  ImGui::InvisibleButton("canvas", canvasSz, ImGuiButtonFlags_MouseButtonLeft | ImGuiButtonFlags_MouseButtonRight);
  const bool isMouseHovered = ImGui::IsItemHovered();  // Hovered
  const bool isMouseActive = ImGui::IsItemActive();    // Held

  // Pan (we use a zero mouse threshold when there's no context menu)
  // You may decide to make that threshold dynamic based on whether the mouse is hovering something etc.
  const float mouseThresholdForPan = -1.0f;
  if (isMouseActive && ImGui::IsMouseDragging(ImGuiMouseButton_Right, mouseThresholdForPan)) {
    scrolling.x += io.MouseDelta.x;
    scrolling.y += io.MouseDelta.y;
  }
  // Context menu (under default mouse threshold)
  const ImVec2 dragDelta = ImGui::GetMouseDragDelta(ImGuiMouseButton_Right);
  if (dragDelta.x == 0.0f && dragDelta.y == 0.0f)
    ImGui::OpenPopupOnItemClick("context", ImGuiPopupFlags_MouseButtonRight);
  if (ImGui::BeginPopup("context")) {
    ImGui::EndPopup();
  }

  // Draw profile + all lines in the canvas
  drawList->PushClipRect(canvasP0, canvasP1, true);
  if (isMouseHovered && ImGui::IsMouseDown(ImGuiMouseButton_Left)) {
    func(glm::vec2(mousePosInCanvas.x, mousePosInCanvas.y));
  }
  const size_t mod = spatialPlantDistribution.m_plants.size() / 15000;
  int index = 0;
  for (const auto& plant : spatialPlantDistribution.m_plants) {
    if (plant.m_recycled)
      continue;
    index++;
    if (mod > 1 && index % mod != 0)
      continue;
    const auto& pointPosition = plant.m_position;
    const auto& pointColor = spatialPlantDistribution.m_spatialPlantParameters[plant.m_parameterHandle].m_color;
    const auto canvasPosition =
        ImVec2(origin.x + pointPosition.x * zoomFactor, origin.y + pointPosition.y * zoomFactor);
    drawList->AddCircleFilled(canvasPosition,
                              zoomFactor * (enableUniformSize ? uniformSize : plant.m_radius * plantSizeFactor),
                              IM_COL32(255.0f * pointColor.x, 255.0f * pointColor.y, 255.0f * pointColor.z, 255.0f));
  }
  drawList->AddCircle(origin, glm::clamp(zoomFactor, 1.0f, 100.0f), IM_COL32(255, 0, 0, 255));
  drawFunc(origin, zoomFactor, drawList);
  drawList->PopClipRect();
}

bool SpatialPlantDistributionSimulator::OnInspect(const std::shared_ptr<EditorLayer>& editorLayer) {
  bool changed = false;
  if (ImGui::TreeNode("Global Parameters")) {
    ImGui::DragFloat("Weighting Factor", &m_distribution.m_spatialPlantGlobalParameters.m_p, 0.01f, 0.0f, 1.0f);
    ImGui::DragFloat("Delta", &m_distribution.m_spatialPlantGlobalParameters.m_p, 1.f, 1.0f, 3.0f);
    ImGui::DragFloat("Simulation Rate", &m_distribution.m_spatialPlantGlobalParameters.m_simulationRate, 0.01f, 0.0f,
                     10.0f);
    ImGui::DragFloat("Spawn Protection Factor", &m_distribution.m_spatialPlantGlobalParameters.m_spawnProtectionFactor,
                     0.01f, 0.0f, 1.0f);
    ImGui::DragFloat("Max Radius", &m_distribution.m_spatialPlantGlobalParameters.m_maxRadius, 1.f, 10.0f, 10000.0f);
    ImGui::Checkbox("Force remove all overlap", &m_distribution.m_spatialPlantGlobalParameters.m_forceRemoveOverlap);
    ImGui::DragFloat("Dynamic balance factor", &m_distribution.m_spatialPlantGlobalParameters.m_dynamicBalanceFactor,
                     0.1f, 0.0f, 3.0f);
    ImGui::TreePop();
  }

  if (ImGui::TreeNode("Parameters")) {
    if (ImGui::Button("Push")) {
      m_distribution.m_spatialPlantParameters.emplace_back();
      m_distribution.m_spatialPlantParameters.back().m_color = glm::vec4(glm::abs(glm::sphericalRand(1.f)), 1.f);
      m_treeDescriptors.resize(m_distribution.m_spatialPlantParameters.size());
    }
    for (int parameterIndex = 0; parameterIndex < m_distribution.m_spatialPlantParameters.size(); parameterIndex++) {
      if (ImGui::TreeNode(("No. " + std::to_string(parameterIndex)).c_str())) {
        auto& parameter = m_distribution.m_spatialPlantParameters.at(parameterIndex);
        ImGui::DragFloat("Final radius", &parameter.m_finalRadius, 1.0f, 1.0f, 100.0f);
        ImGui::DragFloat("Growth rate", &parameter.m_k, 0.01f, 0.01f, 1.0f);
        ImGui::DragFloat("Seeding Range Min", &parameter.m_seedingRangeMin, 1.f, 2.0f, parameter.m_seedingRangeMax);
        ImGui::DragFloat("Seeding Range Max", &parameter.m_seedingRangeMax, 1.f, parameter.m_seedingRangeMin, 10.0f);
        editorLayer->DragAndDropButton<TreeDescriptor>(m_treeDescriptors.at(parameterIndex), "Tree Descriptor", true);
        ImGui::ColorEdit4("Color", &parameter.m_color.x);
        if (ImGui::Button((std::string("Clear No. ") + std::to_string(parameterIndex)).c_str())) {
          for (const auto& plant : m_distribution.m_plants) {
            if (!plant.m_recycled && plant.m_parameterHandle == parameterIndex)
              m_distribution.RecyclePlant(plant.m_handle);
          }
        }
        ImGui::TreePop();
      }
    }
    ImGui::TreePop();
  }

  if (ImGui::TreeNode("Generate Plants...")) {
    static int plantSize = 10;
    static float initialRadius = 1.f;
    static int parameterHandle = 0;
    ImGui::DragInt("Plant size", &plantSize, 10, 10, 1000);
    ImGui::DragFloat("Initial radius", &initialRadius, 1.f, 1.f, 10.f);
    ImGui::DragInt("Parameter Index", &parameterHandle, 1, 0,
                   static_cast<int>(m_distribution.m_spatialPlantParameters.size()) - 1);
    parameterHandle =
        glm::clamp(parameterHandle, 0, static_cast<int>(m_distribution.m_spatialPlantParameters.size()) - 1);
    if (ImGui::Button("Generate")) {
      for (int i = 0; i < plantSize; i++) {
        m_distribution.AddPlant(parameterHandle, initialRadius,
                                glm::linearRand(glm::vec2(-m_distribution.m_spatialPlantGlobalParameters.m_maxRadius),
                                                glm::vec2(m_distribution.m_spatialPlantGlobalParameters.m_maxRadius)));
      }
    }
    ImGui::TreePop();
  }

  ImGui::Checkbox("Simulate", &m_simulate);
  static bool setParent = true;
  static float range = 10.0f;
  static auto offset = glm::vec2(.0f);
  static float positionZoom = 0.5f;
  ImGui::DragFloat("Range", &range, 1.f, 1.f, 100.f);
  ImGui::DragFloat2("Offset", &offset.x, 1.f, 1.f, 200.f);
  ImGui::DragFloat("Position Zoom", &positionZoom, 0.01f, 0.01f, 10.f);
  if (ImGui::Button("Create Trees")) {
    Entity parent;
    const auto scene = GetScene();
    if (setParent) {
      parent = scene->CreateEntity("Forest");
    }
    int i = 0;
    for (const auto& plant : m_distribution.m_plants) {
      if (glm::abs(plant.m_position.x - offset.x) < range && glm::abs(plant.m_position.y - offset.y) < range &&
          m_treeDescriptors.size() > plant.m_parameterHandle &&
          m_treeDescriptors.at(plant.m_parameterHandle).Get<TreeDescriptor>()) {
        auto treeEntity = scene->CreateEntity("Tree No." + std::to_string(i));
        GlobalTransform gt{};
        gt.SetPosition(glm::vec3(plant.m_position.x * positionZoom, -0.05, plant.m_position.y * positionZoom));
        scene->SetDataComponent(treeEntity, gt);
        const auto tree = scene->GetOrSetPrivateComponent<Tree>(treeEntity).lock();
        tree->tree_model.tree_growth_settings = m_treeGrowthSettings;
        tree->tree_descriptor = m_treeDescriptors.at(plant.m_parameterHandle);
        if (setParent)
          scene->SetParent(treeEntity, parent);
        i++;
      }
    }
  }

  const std::string tag = "Spatial Plant Scene";
  ImGui::SetNextWindowSize(ImVec2(300, 300), ImGuiCond_Appearing);
  if (ImGui::Begin(tag.c_str())) {
    OnInspectSpatialPlantDistributionFunction(
        m_distribution,
        [&](glm::vec2 position) {

        },
        [&](ImVec2 origin, float zoomFactor, ImDrawList* drawList) {
          drawList->AddQuad(
              ImVec2(origin.x - (range - offset.x) * zoomFactor, origin.y + (range + offset.y) * zoomFactor),
              ImVec2(origin.x + (range + offset.x) * zoomFactor, origin.y + (range + offset.y) * zoomFactor),
              ImVec2(origin.x + (range + offset.x) * zoomFactor, origin.y - (range - offset.y) * zoomFactor),
              ImVec2(origin.x - (range - offset.x) * zoomFactor, origin.y - (range - offset.y) * zoomFactor),
              IM_COL32(255.0f, 255.0f, 255.0f, 255.0f));
        });
  }
  ImGui::End();

  return changed;
}

void SpatialPlantDistributionSimulator::FixedUpdate() {
  if (m_simulate) {
    m_distribution.Simulate();
  }
}

void SpatialPlantDistributionSimulator::OnCreate() {
  m_distribution = {};
  m_distribution.m_spatialPlantParameters.emplace_back();
  m_treeDescriptors.resize(m_distribution.m_spatialPlantParameters.size());
}
