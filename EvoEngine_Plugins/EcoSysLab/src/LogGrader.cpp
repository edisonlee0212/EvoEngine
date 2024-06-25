#include "LogGrader.hpp"

using namespace eco_sys_lab;

bool ProceduralLogParameters::OnInspect() {
  bool changed = false;
  if (ImGui::TreeNodeEx("Log Parameters", ImGuiTreeNodeFlags_DefaultOpen)) {
    if (ImGui::Checkbox("Butt only", &m_bottom))
      changed = true;
    if (ImGui::DragFloat("Height (Feet)", &m_lengthWithoutTrimInFeet))
      changed = true;
    if (ImGui::DragFloat("Large End Diameter (Inch)", &m_largeEndDiameterInInches))
      changed = true;
    if (ImGui::DragFloat("Small End Diameter (Inch)", &m_smallEndDiameterInInches))
      changed = true;
    if (ImGui::Checkbox("Less than 1/4 sound defects", &m_soundDefect))
      changed = true;
    if (ImGui::Combo("Mode", {"Sweep", "Crook"}, m_mode))
      changed = true;
    if (m_mode == 0) {
      if (ImGui::DragFloat("Sweep (Inch)", &m_spanInInches, 0.1f, .0f, 100.f))
        changed = true;
      if (ImGui::DragFloat("Sweep Angle", &m_angle, 1.f, .0f, 360.f))
        changed = true;
    } else {
      if (ImGui::DragFloat("Crook (Inch)", &m_spanInInches, 0.1f, .0f, 100.f))
        changed = true;
      if (ImGui::DragFloat("Crook Angle", &m_angle, 1.f, .0f, 360.f))
        changed = true;
      if (ImGui::SliderFloat("Crook Ratio", &m_crookRatio, 0.f, 1.f))
        changed = true;
      ImGui::Text(("CL: " + std::to_string(m_lengthWithoutTrimInFeet * (1.f - m_crookRatio)) + " feet.").c_str());
    }
    if (ImGui::Button("Reset")) {
      m_bottom = true;
      m_soundDefect = false;
      m_lengthWithoutTrimInFeet = 16.0f;
      m_lengthStepInInches = 1.f;
      m_largeEndDiameterInInches = 25.f;
      m_smallEndDiameterInInches = 20.f;

      m_mode = 0;
      m_spanInInches = 0.0f;
      m_angle = 180.0f;
      m_crookRatio = 0.7f;
      changed = true;
    }
    ImGui::TreePop();
  }
  return changed;
}

void LogGrader::RefreshMesh(const LogGrading& logGrading) const {
  GenerateCylinderMesh(m_tempCylinderMesh, m_logWoodMeshGenerationSettings);

  GenerateSurface(m_surface4, m_logWoodMeshGenerationSettings, 270 + logGrading.m_angleOffset,
                  360 + logGrading.m_angleOffset);
  GenerateSurface(m_surface3, m_logWoodMeshGenerationSettings, 0 + logGrading.m_angleOffset,
                  90 + logGrading.m_angleOffset);
  GenerateSurface(m_surface2, m_logWoodMeshGenerationSettings, 180 + logGrading.m_angleOffset,
                  270 + logGrading.m_angleOffset);
  GenerateSurface(m_surface1, m_logWoodMeshGenerationSettings, 90 + logGrading.m_angleOffset,
                  180 + logGrading.m_angleOffset);
  // GenerateFlatMesh(m_tempFlatMesh1, m_logWoodMeshGenerationSettings, 90 + logGrading.m_angleOffset, 180 +
  // logGrading.m_angleOffset); GenerateFlatMesh(m_tempFlatMesh2, m_logWoodMeshGenerationSettings, 0 +
  // logGrading.m_angleOffset, 90 + logGrading.m_angleOffset); GenerateFlatMesh(m_tempFlatMesh3,
  // m_logWoodMeshGenerationSettings, 270 + logGrading.m_angleOffset, 360 + logGrading.m_angleOffset);
  // GenerateFlatMesh(m_tempFlatMesh4, m_logWoodMeshGenerationSettings, 180 + logGrading.m_angleOffset, 270 +
  // logGrading.m_angleOffset);
}

bool LogGrader::OnInspect(const std::shared_ptr<EditorLayer>& editorLayer) {
  bool changed = false;
  m_proceduralLogParameters.OnInspect();

  if (ImGui::Button("Initialize Log")) {
    auto branchShape = m_branchShape.Get<BarkDescriptor>();
    if (!branchShape) {
      branchShape = ProjectManager::CreateTemporaryAsset<BarkDescriptor>();
      m_branchShape = branchShape;
      branchShape->bark_depth = branchShape->base_depth = 0.1f;
    }
    InitializeLogRandomly(m_proceduralLogParameters, branchShape);
    m_bestGradingIndex = 0;
    m_logWood.CalculateGradingData(m_availableBestGrading);
    m_logWood.ColorBasedOnGrading(m_availableBestGrading[m_bestGradingIndex]);
    RefreshMesh(m_availableBestGrading[m_bestGradingIndex]);
  }
  if (ImGui::TreeNode("Log Mesh Generation")) {
    // editorLayer->DragAndDropButton<BarkDescriptor>(m_branchShape, "Branch Shape", true);
    ImGui::DragFloat("Y Subdivision", &m_logWoodMeshGenerationSettings.m_ySubdivision, 0.01f, 0.01f, 0.5f);
    static int rotateDegrees = 10;
    ImGui::DragInt("Degrees", &rotateDegrees, 1, 1, 360);
    if (ImGui::Button(("Rotate " + std::to_string(rotateDegrees) + " degrees").c_str())) {
      m_logWood.Rotate(rotateDegrees);
      m_bestGradingIndex = 0;
      m_logWood.CalculateGradingData(m_availableBestGrading);
      m_logWood.ColorBasedOnGrading(m_availableBestGrading[m_bestGradingIndex]);
      RefreshMesh(m_availableBestGrading[m_bestGradingIndex]);
    }

    if (ImGui::Button("Initialize Mesh Renderer"))
      InitializeMeshRenderer(m_logWoodMeshGenerationSettings);
    ImGui::TreePop();
  }

  if (ImGui::Button("Clear Defects")) {
    m_bestGradingIndex = 0;
    m_logWood.ClearDefects();
    m_logWood.CalculateGradingData(m_availableBestGrading);
    m_logWood.ColorBasedOnGrading(m_availableBestGrading[m_bestGradingIndex]);
    RefreshMesh(m_availableBestGrading[m_bestGradingIndex]);
  }

  static bool debugVisualization = true;

  static int rotationAngle = 0;

  if (!m_availableBestGrading.empty()) {
    std::string grading = "Current grading: ";
    if (m_availableBestGrading.front().m_grade <= 3) {
      grading.append("F1");
    } else if (m_availableBestGrading.front().m_grade <= 7) {
      grading.append("F2");
    } else if (m_availableBestGrading.front().m_grade <= 8) {
      grading.append("F3");
    } else {
      grading.append("N/A");
    }
    ImGui::Text(grading.c_str());

    const auto& currentBestGrading = m_availableBestGrading[m_bestGradingIndex];
    ImGui::Text(("Scaling diameter (Inch): " +
                 std::to_string(LogWood::MetersToInches(currentBestGrading.m_scalingDiameterInMeters)))
                    .c_str());
    ImGui::Text(("Length without trim (Feet): " +
                 std::to_string(LogWood::MetersToFeet(currentBestGrading.m_lengthWithoutTrimInMeters)))
                    .c_str());
    ImGui::Separator();
    ImGui::Text(("Crook Deduction: " + std::to_string(currentBestGrading.m_crookDeduction)).c_str());
    ImGui::Text(("Sweep Deduction: " + std::to_string(currentBestGrading.m_sweepDeduction)).c_str());
    ImGui::Separator();
    ImGui::Text(("Doyle Rule: " + std::to_string(currentBestGrading.m_doyleRuleScale)).c_str());
    ImGui::Text(("Scribner Rule: " + std::to_string(currentBestGrading.m_scribnerRuleScale)).c_str());
    ImGui::Text(("International Rule: " + std::to_string(currentBestGrading.m_internationalRuleScale)).c_str());

    if (ImGui::TreeNode("Grading details")) {
      if (ImGui::SliderInt("Grading index", &m_bestGradingIndex, 0, m_availableBestGrading.size())) {
        m_bestGradingIndex = glm::clamp(m_bestGradingIndex, 0, static_cast<int>(m_availableBestGrading.size()));
        m_logWood.ColorBasedOnGrading(m_availableBestGrading[m_bestGradingIndex]);
        RefreshMesh(m_availableBestGrading[m_bestGradingIndex]);
      }
      ImGui::Text(
          ("Grade determine face index: " + std::to_string(currentBestGrading.m_gradeDetermineFaceIndex)).c_str());
      ImGui::Text(("Angle offset: " + std::to_string(currentBestGrading.m_angleOffset)).c_str());
      for (int gradingFaceIndex = 0; gradingFaceIndex < 4; gradingFaceIndex++) {
        const auto& face = currentBestGrading.m_faces[gradingFaceIndex];
        if (ImGui::TreeNodeEx(("Face " + std::to_string(gradingFaceIndex)).c_str(), ImGuiTreeNodeFlags_DefaultOpen)) {
          ImGui::Text(("Start angle: " + std::to_string(face.m_startAngle)).c_str());
          ImGui::Text(("End angle: " + std::to_string(face.m_endAngle)).c_str());
          ImGui::Text(("Face Grade Index: " + std::to_string(face.m_faceGrade)).c_str());
          std::string faceGrading = "Face grading: ";
          if (face.m_faceGrade <= 3) {
            grading.append("F1");
          } else if (face.m_faceGrade <= 7) {
            grading.append("F2");
          } else if (face.m_faceGrade <= 8) {
            grading.append("F3");
          } else {
            grading.append("N/A");
          }
          ImGui::Text(faceGrading.c_str());
          ImGui::Text(("Clear Cuttings Count: " + std::to_string(face.m_clearCuttings.size())).c_str());
          ImGui::Text(("Clear Cuttings Min Length: " + std::to_string(face.m_clearCuttingMinLengthInMeters)).c_str());
          ImGui::Text(("Clear Cuttings Min Proportion: " + std::to_string(face.m_clearCuttingMinProportion)).c_str());
          ImGui::TreePop();
        }
      }
      ImGui::TreePop();
    }
  }

  ImGui::Checkbox("Visualization", &debugVisualization);
  if (debugVisualization) {
    static bool enableDefectSelection = true;
    static bool eraseMode = false;
    ImGui::Checkbox("Defect Marker", &enableDefectSelection);
    static float defectHeightRange = 0.1f;
    static int defectAngleRange = 10.0f;
    if (ImGui::TreeNode("Marker Settings")) {
      ImGui::Checkbox("Erase mode", &eraseMode);
      ImGui::DragFloat("Defect Marker Y", &defectHeightRange, 0.01f, 0.03f, 1.0f);
      ImGui::DragInt("Defect Marker X", &defectAngleRange, 1, 3, 30);
    }
    ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1, 0, 0, 1));
    if (!editorLayer->SceneCameraWindowFocused() && editorLayer->GetKey(GLFW_KEY_F) == KeyActionType::Press) {
      EVOENGINE_WARNING("Select Scene Window FIRST!");
    }
    ImGui::Text("Press F for marking");
    ImGui::PopStyleColor();

    Transform transform{};
    transform.SetEulerRotation(glm::radians(glm::vec3(0, rotationAngle, 0)));
    if (enableDefectSelection) {
      static std::vector<glm::vec2> mousePositions{};
      if (editorLayer->SceneCameraWindowFocused() && editorLayer->GetLockEntitySelection() &&
          editorLayer->GetSelectedEntity() == GetOwner()) {
        if (editorLayer->GetKey(GLFW_MOUSE_BUTTON_RIGHT) == KeyActionType::Press) {
          mousePositions.clear();
        } else if (editorLayer->GetKey(GLFW_KEY_F) == KeyActionType::Hold) {
          mousePositions.emplace_back(editorLayer->GetMouseSceneCameraPosition());
        } else if (editorLayer->GetKey(GLFW_KEY_F) == KeyActionType::Release && !mousePositions.empty()) {
          const auto scene = GetScene();
          GlobalTransform cameraLtw;
          cameraLtw.value = glm::translate(editorLayer->GetSceneCameraPosition()) *
                              glm::mat4_cast(editorLayer->GetSceneCameraRotation());
          for (const auto& position : mousePositions) {
            const Ray cameraRay = editorLayer->GetSceneCamera()->ScreenPointToRay(cameraLtw, position);
            float height, angle;
            if (m_logWood.RayCastSelection(transform.value, 0.02f, cameraRay, height, angle)) {
              if (!eraseMode)
                m_logWood.MarkDefectRegion(height, angle, defectHeightRange, defectAngleRange);
              else
                m_logWood.EraseDefectRegion(height, angle, defectHeightRange, defectAngleRange);
            }
          }
          mousePositions.clear();
          m_bestGradingIndex = 0;
          m_logWood.CalculateGradingData(m_availableBestGrading);
          m_logWood.ColorBasedOnGrading(m_availableBestGrading[m_bestGradingIndex]);
          RefreshMesh(m_availableBestGrading[m_bestGradingIndex]);
        }
      } else {
        mousePositions.clear();
      }
    }
    ImGui::SliderInt("Rotation angle", &rotationAngle, 0, 360);
    GizmoSettings gizmoSettings{};
    gizmoSettings.color_mode = GizmoSettings::ColorMode::VertexColor;
    const float avgDistance = m_logWood.GetMaxAverageDistance();
    const float circleLength = 2.0f * glm::pi<float>() * avgDistance;

    if (m_tempCylinderMesh)
      editorLayer->DrawGizmoMesh(m_tempCylinderMesh, glm::vec4(1.0f), transform.value, 1.f, gizmoSettings);

    float xLeftOffset = -avgDistance * 3.f - circleLength / 4.0f;
    transform.SetPosition({xLeftOffset, 0, 0});
    transform.SetEulerRotation(glm::radians(glm::vec3(0, 180, 0)));
    // if (m_tempFlatMesh1) editorLayer->DrawGizmoMesh(m_tempFlatMesh1, glm::vec4(1.0f), transform.value, 1.f,
    // gizmoSettings);
    if (m_surface1)
      editorLayer->DrawGizmoMeshInstancedColored(Resources::GetResource<Mesh>("PRIMITIVE_QUAD"), m_surface1,
                                                 transform.value, 1, gizmoSettings);
    xLeftOffset -= circleLength / 4.0f + 0.2f;
    transform.SetPosition({xLeftOffset, 0, 0});
    // if (m_tempFlatMesh2) editorLayer->DrawGizmoMesh(m_tempFlatMesh2, glm::vec4(1.0f), transform.value, 1.f,
    // gizmoSettings);
    if (m_surface2)
      editorLayer->DrawGizmoMeshInstancedColored(Resources::GetResource<Mesh>("PRIMITIVE_QUAD"), m_surface2,
                                                 transform.value, 1.f, gizmoSettings);

    float xRightOffset = avgDistance * 3.f;
    transform.SetPosition({xRightOffset, 0, 0});
    // if (m_tempFlatMesh3) editorLayer->DrawGizmoMesh(m_tempFlatMesh3, glm::vec4(1.0f), transform.value, 1.f,
    // gizmoSettings);
    if (m_surface3)
      editorLayer->DrawGizmoMeshInstancedColored(Resources::GetResource<Mesh>("PRIMITIVE_QUAD"), m_surface3,
                                                 transform.value, 1.f, gizmoSettings);

    xRightOffset += circleLength / 4.0f + 0.2f;
    transform.SetPosition({xRightOffset, 0, 0});
    // if (m_tempFlatMesh4) editorLayer->DrawGizmoMesh(m_tempFlatMesh4, glm::vec4(1.0f), transform.value, 1.f,
    // gizmoSettings);
    if (m_surface4)
      editorLayer->DrawGizmoMeshInstancedColored(Resources::GetResource<Mesh>("PRIMITIVE_QUAD"), m_surface4,
                                                 transform.value, 1.f, gizmoSettings);
  }

  return changed;
}

void LogGrader::InitializeLogRandomly(const ProceduralLogParameters& proceduralLogParameters,
                                      const std::shared_ptr<BarkDescriptor>& branchShape) {
  m_logWood.m_intersections.clear();
  m_logWood.m_length = LogWood::FeetToMeter(proceduralLogParameters.m_lengthWithoutTrimInFeet);
  const auto lengthStepInMeters = LogWood::InchesToMeters(proceduralLogParameters.m_lengthStepInInches);
  m_logWood.m_intersections.resize(glm::max(1.0f, m_logWood.m_length / lengthStepInMeters));

  m_logWood.m_sweepInInches = 0.0f;
  m_logWood.m_crookCInInches = 0.0f;
  m_logWood.m_crookCLInFeet = 0.0f;

  m_logWood.m_soundDefect = proceduralLogParameters.m_soundDefect;

  if (proceduralLogParameters.m_mode == 0) {
    m_logWood.m_sweepInInches = proceduralLogParameters.m_spanInInches;
  } else {
    m_logWood.m_crookCInInches = proceduralLogParameters.m_spanInInches;
    m_logWood.m_crookCLInFeet =
        proceduralLogParameters.m_lengthWithoutTrimInFeet * (1.f - proceduralLogParameters.m_crookRatio);
  }
  float theta = 0.0f;
  float r = 0.0f;
  if (proceduralLogParameters.m_mode == 0) {
    theta =
        2.f * glm::atan(LogWood::InchesToMeters(proceduralLogParameters.m_spanInInches) / (m_logWood.m_length / 2.0f));
    r = m_logWood.m_length / 2.0f / glm::sin(theta);
  }
  for (int intersectionIndex = 0; intersectionIndex < m_logWood.m_intersections.size(); intersectionIndex++) {
    const float a = static_cast<float>(intersectionIndex) / (m_logWood.m_intersections.size() - 1);
    const float radius =
        glm::mix(LogWood::InchesToMeters(proceduralLogParameters.m_largeEndDiameterInInches) * 0.5f,
                 LogWood::InchesToMeters(proceduralLogParameters.m_smallEndDiameterInInches) * 0.5f, a);
    auto& intersection = m_logWood.m_intersections[intersectionIndex];
    if (proceduralLogParameters.m_spanInInches != 0.f) {
      if (proceduralLogParameters.m_mode == 0) {
        const glm::vec2 sweepDirection = glm::vec2(glm::cos(glm::radians(proceduralLogParameters.m_angle)),
                                                   glm::sin(glm::radians(proceduralLogParameters.m_angle)));
        const float height = glm::abs(0.5f - a) * m_logWood.m_length;
        const float actualSpan = glm::sqrt(r * r - height * height) - glm::cos(theta) * r;
        intersection.m_center = sweepDirection * actualSpan;
      } else if (a > proceduralLogParameters.m_crookRatio) {
        const glm::vec2 crookDirection = glm::vec2(glm::cos(glm::radians(proceduralLogParameters.m_angle)),
                                                   glm::sin(glm::radians(proceduralLogParameters.m_angle)));
        const float actualA = (a - proceduralLogParameters.m_crookRatio) / (1.f - proceduralLogParameters.m_crookRatio);
        intersection.m_center =
            crookDirection * actualA * LogWood::InchesToMeters(proceduralLogParameters.m_spanInInches);
      }
    }
    intersection.m_boundary.resize(360);
    for (int boundaryPointIndex = 0; boundaryPointIndex < 360; boundaryPointIndex++) {
      auto& boundaryPoint = intersection.m_boundary.at(boundaryPointIndex);
      boundaryPoint.m_centerDistance =
          radius;  // *branchShape->GetValue(static_cast<float>(boundaryPointIndex) / 360.0f, intersectionIndex *
                   // proceduralLogParameters.m_lengthStepInInches);
      boundaryPoint.m_defectStatus = 0.0f;
    }
  }
}

void LogGrader::GenerateCylinderMesh(const std::shared_ptr<Mesh>& mesh,
                                     const LogWoodMeshGenerationSettings& meshGeneratorSettings) const {
  if (!mesh)
    return;
  if (m_logWood.m_intersections.size() < 2)
    return;
  const float logLength = m_logWood.m_length;
  const int yStepSize = logLength / meshGeneratorSettings.m_ySubdivision;
  const float yStep = logLength / yStepSize;

  std::vector<Vertex> vertices{};
  std::vector<unsigned int> indices{};

  vertices.resize((yStepSize + 1) * 360);
  indices.resize(yStepSize * 360 * 6);

  Jobs::RunParallelFor(yStepSize + 1, [&](const unsigned yIndex) {
    Vertex archetype{};
    const float y = yStep * static_cast<float>(yIndex);
    for (int xIndex = 0; xIndex < 360; xIndex++) {
      const float x = static_cast<float>(xIndex);
      const glm::vec2 boundaryPoint = m_logWood.GetSurfacePoint(y, x);
      archetype.position = glm::vec3(boundaryPoint.x, y, boundaryPoint.y);
      archetype.color = m_logWood.GetColor(y, x);
      archetype.tex_coord = {x, y};
      vertices[yIndex * 360 + xIndex] = archetype;
    }
  });
  Jobs::RunParallelFor(yStepSize, [&](const unsigned yIndex) {
    const auto vertexStartIndex = yIndex * 360;
    for (int xIndex = 0; xIndex < 360; xIndex++) {
      auto a = vertexStartIndex + xIndex;
      auto b = vertexStartIndex + (xIndex == 360 - 1 ? 0 : xIndex + 1);
      auto c = vertexStartIndex + 360 + xIndex;
      indices.at((yIndex * 360 + xIndex) * 6) = c;
      indices.at((yIndex * 360 + xIndex) * 6 + 1) = b;
      indices.at((yIndex * 360 + xIndex) * 6 + 2) = a;
      a = vertexStartIndex + 360 + (xIndex == 360 - 1 ? 0 : xIndex + 1);
      b = vertexStartIndex + 360 + xIndex;
      c = vertexStartIndex + (xIndex == 360 - 1 ? 0 : xIndex + 1);
      indices.at((yIndex * 360 + xIndex) * 6 + 3) = c;
      indices.at((yIndex * 360 + xIndex) * 6 + 4) = b;
      indices.at((yIndex * 360 + xIndex) * 6 + 5) = a;
    }
  });

  VertexAttributes attributes{};
  attributes.tex_coord = true;
  mesh->SetVertices(attributes, vertices, indices);
}

void LogGrader::GenerateFlatMesh(const std::shared_ptr<Mesh>& mesh,
                                 const LogWoodMeshGenerationSettings& meshGeneratorSettings, const int startX,
                                 const int endX) const {
  if (!mesh)
    return;
  if (m_logWood.m_intersections.size() < 2)
    return;

  const float avgDistance = m_logWood.GetAverageDistance();
  const float circleLength = 2.0f * glm::pi<float>() * avgDistance;
  const float flatXStep = circleLength / 360;
  const float logLength = m_logWood.m_length;
  const int yStepSize = logLength / meshGeneratorSettings.m_ySubdivision;
  const float yStep = logLength / yStepSize;

  std::vector<Vertex> vertices{};
  std::vector<unsigned int> indices{};

  const int span = endX - startX;

  vertices.resize((yStepSize + 1) * (span + 1));
  indices.resize(yStepSize * span * 6);

  Jobs::RunParallelFor(yStepSize + 1, [&](const unsigned yIndex) {
    Vertex archetype{};
    const float y = yStep * static_cast<float>(yIndex);
    const float intersectionAvgDistance = m_logWood.GetAverageDistance(y);
    for (int xIndex = 0; xIndex <= span; xIndex++) {
      const float x = static_cast<float>(xIndex);
      const float centerDistance = m_logWood.GetCenterDistance(y, x);
      archetype.position =
          glm::vec3(flatXStep * static_cast<float>(xIndex - span), y, centerDistance - intersectionAvgDistance);
      archetype.color = m_logWood.GetColor(y, x + startX);
      archetype.tex_coord = {x, y};
      vertices[yIndex * (span + 1) + xIndex] = archetype;
    }
  });
  Jobs::RunParallelFor(yStepSize, [&](const unsigned yIndex) {
    const auto vertexStartIndex = yIndex * (span + 1);
    for (int xIndex = 0; xIndex < span; xIndex++) {
      auto a = vertexStartIndex + xIndex;
      auto b = vertexStartIndex + xIndex + 1;
      auto c = vertexStartIndex + (span + 1) + xIndex;
      indices.at((yIndex * span + xIndex) * 6) = c;
      indices.at((yIndex * span + xIndex) * 6 + 1) = b;
      indices.at((yIndex * span + xIndex) * 6 + 2) = a;
      a = vertexStartIndex + (span + 1) + xIndex + 1;
      b = vertexStartIndex + (span + 1) + xIndex;
      c = vertexStartIndex + xIndex + 1;
      indices.at((yIndex * span + xIndex) * 6 + 3) = c;
      indices.at((yIndex * span + xIndex) * 6 + 4) = b;
      indices.at((yIndex * span + xIndex) * 6 + 5) = a;
    }
  });

  VertexAttributes attributes{};
  attributes.tex_coord = true;
  mesh->SetVertices(attributes, vertices, indices);
}

void LogGrader::GenerateSurface(const std::shared_ptr<ParticleInfoList>& surface,
                                const LogWoodMeshGenerationSettings& meshGeneratorSettings, const int startX,
                                const int endX) const {
  if (!surface)
    return;
  if (m_logWood.m_intersections.size() < 2)
    return;

  const float avgDistance = m_logWood.GetAverageDistance();
  const float circleLength = 2.0f * glm::pi<float>() * avgDistance;
  const float flatXStep = circleLength / 360;
  const float logLength = m_logWood.m_length;
  const int yStepSize = logLength / meshGeneratorSettings.m_ySubdivision;
  const float yStep = logLength / yStepSize;

  const int span = endX - startX;
  std::vector<ParticleInfo> particleInfos;

  particleInfos.resize(yStepSize * span);

  Jobs::RunParallelFor(yStepSize + 1, [&](const unsigned yIndex) {
    const float y = yStep * static_cast<float>(yIndex);
    const float intersectionAvgDistance = m_logWood.GetAverageDistance(y);
    for (int xIndex = 0; xIndex <= span; xIndex++) {
      const float x = static_cast<float>(xIndex);
      const float centerDistance = m_logWood.GetCenterDistance(y, x);
      const auto position =
          glm::vec3(flatXStep * static_cast<float>(xIndex - span), y, centerDistance - intersectionAvgDistance);
      const auto size = glm::vec3(flatXStep, 0.f, yStep);
      const auto rotation = glm::quat(glm::radians(glm::vec3(-90, 0, 0)));
      auto& particleInfo = particleInfos.at(yIndex * span + xIndex);
      particleInfo.instance_matrix.value = glm::translate(position) * glm::mat4_cast(rotation) * glm::scale(size);
      particleInfo.instance_color = m_logWood.GetColor(y, x + startX);
    }
  });
  surface->SetParticleInfos(particleInfos);
}

void LogGrader::OnCreate() {
  m_tempCylinderMesh = ProjectManager::CreateTemporaryAsset<Mesh>();
  m_tempFlatMesh1 = ProjectManager::CreateTemporaryAsset<Mesh>();
  m_tempFlatMesh2 = ProjectManager::CreateTemporaryAsset<Mesh>();
  m_tempFlatMesh3 = ProjectManager::CreateTemporaryAsset<Mesh>();
  m_tempFlatMesh4 = ProjectManager::CreateTemporaryAsset<Mesh>();
  m_surface1 = ProjectManager::CreateTemporaryAsset<ParticleInfoList>();
  m_surface2 = ProjectManager::CreateTemporaryAsset<ParticleInfoList>();
  m_surface3 = ProjectManager::CreateTemporaryAsset<ParticleInfoList>();
  m_surface4 = ProjectManager::CreateTemporaryAsset<ParticleInfoList>();

  auto branchShape = m_branchShape.Get<BarkDescriptor>();
  if (!branchShape) {
    branchShape = ProjectManager::CreateTemporaryAsset<BarkDescriptor>();
    m_branchShape = branchShape;
    branchShape->bark_depth = branchShape->base_depth = 0.1f;
  }
  InitializeLogRandomly(m_proceduralLogParameters, branchShape);
  m_bestGradingIndex = 0;
  m_logWood.CalculateGradingData(m_availableBestGrading);
  m_logWood.ColorBasedOnGrading(m_availableBestGrading[m_bestGradingIndex]);
  RefreshMesh(m_availableBestGrading[m_bestGradingIndex]);
}

void LogGrader::InitializeMeshRenderer(const LogWoodMeshGenerationSettings& meshGeneratorSettings) const {
  ClearMeshRenderer();
  const auto scene = GetScene();
  const auto self = GetOwner();
  const auto cylinderEntity = scene->CreateEntity("Log Wood Cylinder Mesh");
  if (scene->IsEntityValid(cylinderEntity)) {
    scene->SetParent(cylinderEntity, self);
    const auto mesh = ProjectManager::CreateTemporaryAsset<Mesh>();
    GenerateCylinderMesh(mesh, meshGeneratorSettings);
    const auto material = ProjectManager::CreateTemporaryAsset<Material>();
    const auto meshRenderer = scene->GetOrSetPrivateComponent<MeshRenderer>(cylinderEntity).lock();
    material->material_properties.roughness = 1.0f;
    material->material_properties.metallic = 0.0f;
    material->vertex_color_only = true;
    meshRenderer->mesh = mesh;
    meshRenderer->material = material;
  }

  const float avgDistance = m_logWood.GetMaxAverageDistance();
  const float circleLength = 2.0f * glm::pi<float>() * avgDistance;
  float xOffset = avgDistance * 1.5f;
  const auto flatEntity1 = scene->CreateEntity("Log Wood Flat Mesh 1");
  if (scene->IsEntityValid(flatEntity1)) {
    scene->SetParent(flatEntity1, self);
    Transform transform{};
    transform.SetPosition({xOffset, 0, 0});
    transform.SetEulerRotation(glm::radians(glm::vec3(0, 180, 0)));
    scene->SetDataComponent(flatEntity1, transform);
    const auto mesh = ProjectManager::CreateTemporaryAsset<Mesh>();
    GenerateFlatMesh(mesh, meshGeneratorSettings, 90, 180);
    const auto material = ProjectManager::CreateTemporaryAsset<Material>();
    const auto meshRenderer = scene->GetOrSetPrivateComponent<MeshRenderer>(flatEntity1).lock();
    material->material_properties.roughness = 1.0f;
    material->material_properties.metallic = 0.0f;
    material->vertex_color_only = true;
    meshRenderer->mesh = mesh;
    meshRenderer->material = material;
  }
  xOffset += circleLength / 4.0f + 0.2f;
  const auto flatEntity2 = scene->CreateEntity("Log Wood Flat Mesh 2");
  if (scene->IsEntityValid(flatEntity2)) {
    scene->SetParent(flatEntity2, self);
    Transform transform{};
    transform.SetPosition({xOffset, 0, 0});
    transform.SetEulerRotation(glm::radians(glm::vec3(0, 180, 0)));
    scene->SetDataComponent(flatEntity2, transform);
    const auto mesh = ProjectManager::CreateTemporaryAsset<Mesh>();
    GenerateFlatMesh(mesh, meshGeneratorSettings, 0, 90);
    const auto material = ProjectManager::CreateTemporaryAsset<Material>();
    const auto meshRenderer = scene->GetOrSetPrivateComponent<MeshRenderer>(flatEntity2).lock();
    material->material_properties.roughness = 1.0f;
    material->material_properties.metallic = 0.0f;
    material->vertex_color_only = true;
    meshRenderer->mesh = mesh;
    meshRenderer->material = material;
  }
  xOffset += circleLength / 4.0f + 0.2f;
  const auto flatEntity3 = scene->CreateEntity("Log Wood Flat Mesh 3");
  if (scene->IsEntityValid(flatEntity3)) {
    scene->SetParent(flatEntity3, self);
    Transform transform{};
    transform.SetPosition({xOffset, 0, 0});
    transform.SetEulerRotation(glm::radians(glm::vec3(0, 180, 0)));
    scene->SetDataComponent(flatEntity3, transform);
    const auto mesh = ProjectManager::CreateTemporaryAsset<Mesh>();
    GenerateFlatMesh(mesh, meshGeneratorSettings, 270, 360);
    const auto material = ProjectManager::CreateTemporaryAsset<Material>();
    const auto meshRenderer = scene->GetOrSetPrivateComponent<MeshRenderer>(flatEntity3).lock();
    material->material_properties.roughness = 1.0f;
    material->material_properties.metallic = 0.0f;
    material->vertex_color_only = true;
    meshRenderer->mesh = mesh;
    meshRenderer->material = material;
  }
  xOffset += circleLength / 4.0f + 0.2f;
  const auto flatEntity4 = scene->CreateEntity("Log Wood Flat Mesh 4");
  if (scene->IsEntityValid(flatEntity4)) {
    scene->SetParent(flatEntity4, self);
    Transform transform{};
    transform.SetPosition({xOffset, 0, 0});
    transform.SetEulerRotation(glm::radians(glm::vec3(0, 180, 0)));
    scene->SetDataComponent(flatEntity4, transform);
    const auto mesh = ProjectManager::CreateTemporaryAsset<Mesh>();
    GenerateFlatMesh(mesh, meshGeneratorSettings, 180, 270);
    const auto material = ProjectManager::CreateTemporaryAsset<Material>();
    const auto meshRenderer = scene->GetOrSetPrivateComponent<MeshRenderer>(flatEntity4).lock();
    material->material_properties.roughness = 1.0f;
    material->material_properties.metallic = 0.0f;
    material->vertex_color_only = true;
    meshRenderer->mesh = mesh;
    meshRenderer->material = material;
  }
}

void LogGrader::ClearMeshRenderer() const {
  const auto scene = GetScene();
  const auto self = GetOwner();
  const auto children = scene->GetChildren(self);
  for (const auto& child : children) {
    auto name = scene->GetEntityName(child);
    if (name == "Log Wood Cylinder Mesh") {
      scene->DeleteEntity(child);
    } else if (name == "Log Wood Flat Mesh 1") {
      scene->DeleteEntity(child);
    } else if (name == "Log Wood Flat Mesh 2") {
      scene->DeleteEntity(child);
    } else if (name == "Log Wood Flat Mesh 3") {
      scene->DeleteEntity(child);
    } else if (name == "Log Wood Flat Mesh 4") {
      scene->DeleteEntity(child);
    }
  }
}
