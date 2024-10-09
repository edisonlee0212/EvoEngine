#include "LogGrader.hpp"

using namespace log_grading_plugin;

bool ProceduralLogParameters::OnInspect() {
  bool changed = false;
  if (ImGui::TreeNodeEx("Log Parameters", ImGuiTreeNodeFlags_DefaultOpen)) {
    if (ImGui::Checkbox("Butt only", &m_bottom))
      changed = true;
    if (ImGui::DragFloat("Height (Feet)", &m_length_without_trim_in_feet))
      changed = true;
    if (ImGui::DragFloat("Large End Diameter (Inch)", &m_large_end_diameter_in_inches))
      changed = true;
    if (ImGui::DragFloat("Small End Diameter (Inch)", &m_small_end_diameter_in_inches))
      changed = true;
    if (ImGui::Checkbox("Less than 1/4 sound defects", &m_sound_defect))
      changed = true;
    if (ImGui::Combo("Mode", {"Sweep", "Crook"}, m_mode))
      changed = true;
    if (m_mode == 0) {
      if (ImGui::DragFloat("Sweep (Inch)", &m_span_in_inches, 0.1f, .0f, 100.f))
        changed = true;
      if (ImGui::DragFloat("Sweep Angle", &m_angle, 1.f, .0f, 360.f))
        changed = true;
    } else {
      if (ImGui::DragFloat("Crook (Inch)", &m_span_in_inches, 0.1f, .0f, 100.f))
        changed = true;
      if (ImGui::DragFloat("Crook Angle", &m_angle, 1.f, .0f, 360.f))
        changed = true;
      if (ImGui::SliderFloat("Crook Ratio", &m_crook_ratio, 0.f, 1.f))
        changed = true;
      ImGui::Text(("CL: " + std::to_string(m_length_without_trim_in_feet * (1.f - m_crook_ratio)) + " feet.").c_str());
    }
    if (ImGui::Button("Reset")) {
      m_bottom = true;
      m_sound_defect = false;
      m_length_without_trim_in_feet = 16.0f;
      m_length_step_in_inches = 1.f;
      m_large_end_diameter_in_inches = 25.f;
      m_small_end_diameter_in_inches = 20.f;

      m_mode = 0;
      m_span_in_inches = 0.0f;
      m_angle = 180.0f;
      m_crook_ratio = 0.7f;
      changed = true;
    }
    ImGui::TreePop();
  }
  return changed;
}

void LogGrader::RefreshMesh(const LogGrading& log_grading) const {
  GenerateCylinderMesh(m_tempCylinderMesh, m_log_wood_mesh_generation_settings);

  GenerateSurface(m_surface4, m_log_wood_mesh_generation_settings, 270 + log_grading.m_angle_offset,
                  360 + log_grading.m_angle_offset);
  GenerateSurface(m_surface3, m_log_wood_mesh_generation_settings, 0 + log_grading.m_angle_offset,
                  90 + log_grading.m_angle_offset);
  GenerateSurface(m_surface2, m_log_wood_mesh_generation_settings, 180 + log_grading.m_angle_offset,
                  270 + log_grading.m_angle_offset);
  GenerateSurface(m_surface1, m_log_wood_mesh_generation_settings, 90 + log_grading.m_angle_offset,
                  180 + log_grading.m_angle_offset);
  // GenerateFlatMesh(m_tempFlatMesh1, m_logWoodMeshGenerationSettings, 90 + logGrading.m_angleOffset, 180 +
  // logGrading.m_angleOffset); GenerateFlatMesh(m_tempFlatMesh2, m_logWoodMeshGenerationSettings, 0 +
  // logGrading.m_angleOffset, 90 + logGrading.m_angleOffset); GenerateFlatMesh(m_tempFlatMesh3,
  // m_logWoodMeshGenerationSettings, 270 + logGrading.m_angleOffset, 360 + logGrading.m_angleOffset);
  // GenerateFlatMesh(m_tempFlatMesh4, m_logWoodMeshGenerationSettings, 180 + logGrading.m_angleOffset, 270 +
  // logGrading.m_angleOffset);
}

bool LogGrader::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  bool changed = false;
  m_procedural_log_parameters.OnInspect();

  if (ImGui::Button("Initialize Log")) {
    auto branch_shape = m_branch_shape.Get<BarkDescriptor>();
    if (!branch_shape) {
      branch_shape = ProjectManager::CreateTemporaryAsset<BarkDescriptor>();
      m_branch_shape = branch_shape;
      branch_shape->bark_depth = branch_shape->base_depth = 0.1f;
    }
    InitializeLogRandomly(m_procedural_log_parameters, branch_shape);
    m_best_grading_index = 0;
    m_log_wood.CalculateGradingData(m_available_best_grading);
    m_log_wood.ColorBasedOnGrading(m_available_best_grading[m_best_grading_index]);
    RefreshMesh(m_available_best_grading[m_best_grading_index]);
  }
  if (ImGui::TreeNode("Log Mesh Generation")) {
    // editorLayer->DragAndDropButton<BarkDescriptor>(m_branchShape, "Branch Shape", true);
    ImGui::DragFloat("Y Subdivision", &m_log_wood_mesh_generation_settings.m_y_subdivision, 0.01f, 0.01f, 0.5f);
    static int rotate_degrees = 10;
    ImGui::DragInt("Degrees", &rotate_degrees, 1, 1, 360);
    if (ImGui::Button(("Rotate " + std::to_string(rotate_degrees) + " degrees").c_str())) {
      m_log_wood.Rotate(rotate_degrees);
      m_best_grading_index = 0;
      m_log_wood.CalculateGradingData(m_available_best_grading);
      m_log_wood.ColorBasedOnGrading(m_available_best_grading[m_best_grading_index]);
      RefreshMesh(m_available_best_grading[m_best_grading_index]);
    }

    if (ImGui::Button("Initialize Mesh Renderer"))
      InitializeMeshRenderer(m_log_wood_mesh_generation_settings);
    ImGui::TreePop();
  }

  if (ImGui::Button("Clear Defects")) {
    m_best_grading_index = 0;
    m_log_wood.ClearDefects();
    m_log_wood.CalculateGradingData(m_available_best_grading);
    m_log_wood.ColorBasedOnGrading(m_available_best_grading[m_best_grading_index]);
    RefreshMesh(m_available_best_grading[m_best_grading_index]);
  }

  static bool debug_visualization = true;

  static int rotation_angle = 0;

  if (!m_available_best_grading.empty()) {
    std::string grading = "Current grading: ";
    if (m_available_best_grading.front().m_grade <= 3) {
      grading.append("F1");
    } else if (m_available_best_grading.front().m_grade <= 7) {
      grading.append("F2");
    } else if (m_available_best_grading.front().m_grade <= 8) {
      grading.append("F3");
    } else {
      grading.append("N/A");
    }
    ImGui::Text(grading.c_str());

    const auto& current_best_grading = m_available_best_grading[m_best_grading_index];
    ImGui::Text(("Scaling diameter (Inch): " +
                 std::to_string(LogWood::MetersToInches(current_best_grading.m_scaling_diameter_in_meters)))
                    .c_str());
    ImGui::Text(("Length without trim (Feet): " +
                 std::to_string(LogWood::MetersToFeet(current_best_grading.m_length_without_trim_in_meters)))
                    .c_str());
    ImGui::Separator();
    ImGui::Text(("Crook Deduction: " + std::to_string(current_best_grading.m_crook_deduction)).c_str());
    ImGui::Text(("Sweep Deduction: " + std::to_string(current_best_grading.m_sweep_deduction)).c_str());
    ImGui::Separator();
    ImGui::Text(("Doyle Rule: " + std::to_string(current_best_grading.m_doyle_rule_scale)).c_str());
    ImGui::Text(("Scribner Rule: " + std::to_string(current_best_grading.m_scribner_rule_scale)).c_str());
    ImGui::Text(("International Rule: " + std::to_string(current_best_grading.m_international_rule_scale)).c_str());

    if (ImGui::TreeNode("Grading details")) {
      if (ImGui::SliderInt("Grading index", &m_best_grading_index, 0, m_available_best_grading.size())) {
        m_best_grading_index = glm::clamp(m_best_grading_index, 0, static_cast<int>(m_available_best_grading.size()));
        m_log_wood.ColorBasedOnGrading(m_available_best_grading[m_best_grading_index]);
        RefreshMesh(m_available_best_grading[m_best_grading_index]);
      }
      ImGui::Text(
          ("Grade determine face index: " + std::to_string(current_best_grading.m_grade_determine_face_index)).c_str());
      ImGui::Text(("Angle offset: " + std::to_string(current_best_grading.m_angle_offset)).c_str());
      for (int grading_face_index = 0; grading_face_index < 4; grading_face_index++) {
        const auto& face = current_best_grading.m_faces[grading_face_index];
        if (ImGui::TreeNodeEx(("Face " + std::to_string(grading_face_index)).c_str(), ImGuiTreeNodeFlags_DefaultOpen)) {
          ImGui::Text(("Start angle: " + std::to_string(face.m_start_angle)).c_str());
          ImGui::Text(("End angle: " + std::to_string(face.m_end_angle)).c_str());
          ImGui::Text(("Face Grade Index: " + std::to_string(face.m_face_grade)).c_str());
          std::string face_grading = "Face grading: ";
          if (face.m_face_grade <= 3) {
            grading.append("F1");
          } else if (face.m_face_grade <= 7) {
            grading.append("F2");
          } else if (face.m_face_grade <= 8) {
            grading.append("F3");
          } else {
            grading.append("N/A");
          }
          ImGui::Text(face_grading.c_str());
          ImGui::Text(("Clear Cuttings Count: " + std::to_string(face.m_clear_cuttings.size())).c_str());
          ImGui::Text(("Clear Cuttings Min Length: " + std::to_string(face.m_clear_cutting_min_length_in_meters)).c_str());
          ImGui::Text(("Clear Cuttings Min Proportion: " + std::to_string(face.m_clear_cutting_min_proportion)).c_str());
          ImGui::TreePop();
        }
      }
      ImGui::TreePop();
    }
  }

  ImGui::Checkbox("Visualization", &debug_visualization);
  if (debug_visualization) {
    static bool enable_defect_selection = true;
    static bool erase_mode = false;
    ImGui::Checkbox("Defect Marker", &enable_defect_selection);
    static float defect_height_range = 0.1f;
    static int defect_angle_range = 10.0f;
    if (ImGui::TreeNode("Marker Settings")) {
      ImGui::Checkbox("Erase mode", &erase_mode);
      ImGui::DragFloat("Defect Marker Y", &defect_height_range, 0.01f, 0.03f, 1.0f);
      ImGui::DragInt("Defect Marker X", &defect_angle_range, 1, 3, 30);
    }
    ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1, 0, 0, 1));
    if (!editor_layer->SceneCameraWindowFocused() && editor_layer->GetKey(GLFW_KEY_F) == Input::KeyActionType::Press) {
      EVOENGINE_WARNING("Select Scene Window FIRST!");
    }
    ImGui::Text("Press F for marking");
    ImGui::PopStyleColor();

    Transform transform{};
    transform.SetEulerRotation(glm::radians(glm::vec3(0, rotation_angle, 0)));
    if (enable_defect_selection) {
      static std::vector<glm::vec2> mouse_positions{};
      if (editor_layer->SceneCameraWindowFocused() && editor_layer->GetLockEntitySelection() &&
          editor_layer->GetSelectedEntity() == GetOwner()) {
        if (editor_layer->GetKey(GLFW_MOUSE_BUTTON_RIGHT) == Input::KeyActionType::Press) {
          mouse_positions.clear();
        } else if (editor_layer->GetKey(GLFW_KEY_F) == Input::KeyActionType::Hold) {
          mouse_positions.emplace_back(editor_layer->GetMouseSceneCameraPosition());
        } else if (editor_layer->GetKey(GLFW_KEY_F) == Input::KeyActionType::Release && !mouse_positions.empty()) {
          const auto scene = GetScene();
          GlobalTransform camera_ltw;
          camera_ltw.value = glm::translate(editor_layer->GetSceneCameraPosition()) *
                            glm::mat4_cast(editor_layer->GetSceneCameraRotation());
          for (const auto& position : mouse_positions) {
            const Ray camera_ray = editor_layer->GetSceneCamera()->ScreenPointToRay(camera_ltw, position);
            float height, angle;
            if (m_log_wood.RayCastSelection(transform.value, 0.02f, camera_ray, height, angle)) {
              if (!erase_mode)
                m_log_wood.MarkDefectRegion(height, angle, defect_height_range, defect_angle_range);
              else
                m_log_wood.EraseDefectRegion(height, angle, defect_height_range, defect_angle_range);
            }
          }
          mouse_positions.clear();
          m_best_grading_index = 0;
          m_log_wood.CalculateGradingData(m_available_best_grading);
          m_log_wood.ColorBasedOnGrading(m_available_best_grading[m_best_grading_index]);
          RefreshMesh(m_available_best_grading[m_best_grading_index]);
        }
      } else {
        mouse_positions.clear();
      }
    }
    ImGui::SliderInt("Rotation angle", &rotation_angle, 0, 360);
    GizmoSettings gizmo_settings{};
    gizmo_settings.depth_test = true;
    gizmo_settings.depth_write = true;
    gizmo_settings.color_mode = GizmoSettings::ColorMode::VertexColor;
    const float avg_distance = m_log_wood.GetMaxAverageDistance();
    const float circle_length = 2.0f * glm::pi<float>() * avg_distance;

    if (m_tempCylinderMesh)
      editor_layer->DrawGizmoMesh(m_tempCylinderMesh, glm::vec4(1.0f), transform.value, 1.f, gizmo_settings);

    float x_left_offset = -avg_distance * 3.f - circle_length / 4.0f;
    transform.SetPosition({x_left_offset, 0, 0});
    transform.SetEulerRotation(glm::radians(glm::vec3(0, 180, 0)));
    // if (m_tempFlatMesh1) editorLayer->DrawGizmoMesh(m_tempFlatMesh1, glm::vec4(1.0f), transform.value, 1.f,
    // gizmoSettings);
    if (m_surface1)
      editor_layer->DrawGizmoMeshInstancedColored(Resources::GetResource<Mesh>("PRIMITIVE_QUAD"), m_surface1,
                                                 transform.value, 1, gizmo_settings);
    x_left_offset -= circle_length / 4.0f + 0.2f;
    transform.SetPosition({x_left_offset, 0, 0});
    // if (m_tempFlatMesh2) editorLayer->DrawGizmoMesh(m_tempFlatMesh2, glm::vec4(1.0f), transform.value, 1.f,
    // gizmoSettings);
    if (m_surface2)
      editor_layer->DrawGizmoMeshInstancedColored(Resources::GetResource<Mesh>("PRIMITIVE_QUAD"), m_surface2,
                                                 transform.value, 1.f, gizmo_settings);

    float x_right_offset = avg_distance * 3.f;
    transform.SetPosition({x_right_offset, 0, 0});
    // if (m_tempFlatMesh3) editorLayer->DrawGizmoMesh(m_tempFlatMesh3, glm::vec4(1.0f), transform.value, 1.f,
    // gizmoSettings);
    if (m_surface3)
      editor_layer->DrawGizmoMeshInstancedColored(Resources::GetResource<Mesh>("PRIMITIVE_QUAD"), m_surface3,
                                                 transform.value, 1.f, gizmo_settings);

    x_right_offset += circle_length / 4.0f + 0.2f;
    transform.SetPosition({x_right_offset, 0, 0});
    // if (m_tempFlatMesh4) editorLayer->DrawGizmoMesh(m_tempFlatMesh4, glm::vec4(1.0f), transform.value, 1.f,
    // gizmoSettings);
    if (m_surface4)
      editor_layer->DrawGizmoMeshInstancedColored(Resources::GetResource<Mesh>("PRIMITIVE_QUAD"), m_surface4,
                                                 transform.value, 1.f, gizmo_settings);
  }

  return changed;
}

void LogGrader::InitializeLogRandomly(const ProceduralLogParameters& procedural_log_parameters,
                                      const std::shared_ptr<BarkDescriptor>& branch_shape) {
  m_log_wood.m_intersections.clear();
  m_log_wood.m_length = LogWood::FeetToMeter(procedural_log_parameters.m_length_without_trim_in_feet);
  const auto length_step_in_meters = LogWood::InchesToMeters(procedural_log_parameters.m_length_step_in_inches);
  m_log_wood.m_intersections.resize(glm::max(1.0f, m_log_wood.m_length / length_step_in_meters));

  m_log_wood.m_sweep_in_inches = 0.0f;
  m_log_wood.m_crook_cl_in_inches = 0.0f;
  m_log_wood.m_crook_cl_in_feet = 0.0f;

  m_log_wood.m_sound_defect = procedural_log_parameters.m_sound_defect;

  if (procedural_log_parameters.m_mode == 0) {
    m_log_wood.m_sweep_in_inches = procedural_log_parameters.m_span_in_inches;
  } else {
    m_log_wood.m_crook_cl_in_inches = procedural_log_parameters.m_span_in_inches;
    m_log_wood.m_crook_cl_in_feet =
        procedural_log_parameters.m_length_without_trim_in_feet * (1.f - procedural_log_parameters.m_crook_ratio);
  }
  float theta = 0.0f;
  float r = 0.0f;
  if (procedural_log_parameters.m_mode == 0) {
    theta =
        2.f * glm::atan(LogWood::InchesToMeters(procedural_log_parameters.m_span_in_inches) / (m_log_wood.m_length / 2.0f));
    r = m_log_wood.m_length / 2.0f / glm::sin(theta);
  }
  for (int intersection_index = 0; intersection_index < m_log_wood.m_intersections.size(); intersection_index++) {
    const float a = static_cast<float>(intersection_index) / (m_log_wood.m_intersections.size() - 1);
    const float radius =
        glm::mix(LogWood::InchesToMeters(procedural_log_parameters.m_large_end_diameter_in_inches) * 0.5f,
                 LogWood::InchesToMeters(procedural_log_parameters.m_small_end_diameter_in_inches) * 0.5f, a);
    auto& intersection = m_log_wood.m_intersections[intersection_index];
    if (procedural_log_parameters.m_span_in_inches != 0.f) {
      if (procedural_log_parameters.m_mode == 0) {
        const glm::vec2 sweep_direction = glm::vec2(glm::cos(glm::radians(procedural_log_parameters.m_angle)),
                                                   glm::sin(glm::radians(procedural_log_parameters.m_angle)));
        const float height = glm::abs(0.5f - a) * m_log_wood.m_length;
        const float actual_span = glm::sqrt(r * r - height * height) - glm::cos(theta) * r;
        const auto center = sweep_direction * actual_span;
        intersection.m_center = {center.x, center.y};
      } else if (a > procedural_log_parameters.m_crook_ratio) {
        const glm::vec2 crook_direction = glm::vec2(glm::cos(glm::radians(procedural_log_parameters.m_angle)),
                                                   glm::sin(glm::radians(procedural_log_parameters.m_angle)));
        const float actual_a = (a - procedural_log_parameters.m_crook_ratio) / (1.f - procedural_log_parameters.m_crook_ratio);
        const auto center =
            crook_direction * actual_a * LogWood::InchesToMeters(procedural_log_parameters.m_span_in_inches);
        intersection.m_center = {center.x, center.y};
            ;
      }
    }
    intersection.m_boundary.resize(360);
    for (int boundary_point_index = 0; boundary_point_index < 360; boundary_point_index++) {
      auto& boundary_point = intersection.m_boundary.at(boundary_point_index);
      boundary_point.m_center_distance =
          radius;  // *branchShape->GetValue(static_cast<float>(boundaryPointIndex) / 360.0f, intersectionIndex *
                   // proceduralLogParameters.m_lengthStepInInches);
      boundary_point.m_defect_status = 0.0f;
    }
  }
}

void LogGrader::GenerateCylinderMesh(const std::shared_ptr<Mesh>& mesh,
                                     const LogWoodMeshGenerationSettings& mesh_generator_settings) const {
  if (!mesh)
    return;
  if (m_log_wood.m_intersections.size() < 2)
    return;
  const float log_length = m_log_wood.m_length;
  const int y_step_size = log_length / mesh_generator_settings.m_y_subdivision;
  const float y_step = log_length / y_step_size;

  std::vector<Vertex> vertices{};
  std::vector<unsigned int> indices{};

  vertices.resize((y_step_size + 1) * 360);
  indices.resize(y_step_size * 360 * 6);

  Jobs::RunParallelFor(y_step_size + 1, [&](const unsigned y_index) {
    Vertex archetype{};
    const float y = y_step * static_cast<float>(y_index);
    for (int x_index = 0; x_index < 360; x_index++) {
      const float x = static_cast<float>(x_index);
      const auto boundary_point = m_log_wood.GetSurfacePoint(y, x);
      archetype.position = glm::vec3(boundary_point.v0, y, boundary_point.v1);
      const auto color = m_log_wood.GetColor(y, x);
      archetype.color = {color.v0, color.v1, color.v2, color.v3}; 
      archetype.tex_coord = {x, y};
      vertices[y_index * 360 + x_index] = archetype;
    }
  });
  Jobs::RunParallelFor(y_step_size, [&](const unsigned y_index) {
    const auto vertex_start_index = y_index * 360;
    for (int x_index = 0; x_index < 360; x_index++) {
      auto a = vertex_start_index + x_index;
      auto b = vertex_start_index + (x_index == 360 - 1 ? 0 : x_index + 1);
      auto c = vertex_start_index + 360 + x_index;
      indices.at((y_index * 360 + x_index) * 6) = a;
      indices.at((y_index * 360 + x_index) * 6 + 1) = b;
      indices.at((y_index * 360 + x_index) * 6 + 2) = c;
      a = vertex_start_index + 360 + (x_index == 360 - 1 ? 0 : x_index + 1);
      b = vertex_start_index + 360 + x_index;
      c = vertex_start_index + (x_index == 360 - 1 ? 0 : x_index + 1);
      indices.at((y_index * 360 + x_index) * 6 + 3) = a;
      indices.at((y_index * 360 + x_index) * 6 + 4) = b;
      indices.at((y_index * 360 + x_index) * 6 + 5) = c;
    }
  });

  VertexAttributes attributes{};
  attributes.tex_coord = true;
  mesh->SetVertices(attributes, vertices, indices);
}

void LogGrader::GenerateFlatMesh(const std::shared_ptr<Mesh>& mesh,
                                 const LogWoodMeshGenerationSettings& mesh_generator_settings, const int start_x,
                                 const int end_x) const {
  if (!mesh)
    return;
  if (m_log_wood.m_intersections.size() < 2)
    return;

  const float avg_distance = m_log_wood.GetAverageDistance();
  const float circle_length = 2.0f * glm::pi<float>() * avg_distance;
  const float flat_x_step = circle_length / 360;
  const float log_length = m_log_wood.m_length;
  const int y_step_size = log_length / mesh_generator_settings.m_y_subdivision;
  const float y_step = log_length / y_step_size;

  std::vector<Vertex> vertices{};
  std::vector<unsigned int> indices{};

  const int span = end_x - start_x;

  vertices.resize((y_step_size + 1) * (span + 1));
  indices.resize(y_step_size * span * 6);

  Jobs::RunParallelFor(y_step_size + 1, [&](const unsigned y_index) {
    Vertex archetype{};
    const float y = y_step * static_cast<float>(y_index);
    const float intersection_avg_distance = m_log_wood.GetAverageDistance(y);
    for (int x_index = 0; x_index <= span; x_index++) {
      const float x = static_cast<float>(x_index);
      const float center_distance = m_log_wood.GetCenterDistance(y, x);
      archetype.position =
          glm::vec3(flat_x_step * static_cast<float>(x_index - span), y, center_distance - intersection_avg_distance);
      const auto color = m_log_wood.GetColor(y, x + start_x);
      archetype.color = {color.v0, color.v1, color.v2, color.v3};
      archetype.tex_coord = {x, y};
      vertices[y_index * (span + 1) + x_index] = archetype;
    }
  });
  Jobs::RunParallelFor(y_step_size, [&](const unsigned y_index) {
    const auto vertex_start_index = y_index * (span + 1);
    for (int x_index = 0; x_index < span; x_index++) {
      auto a = vertex_start_index + x_index;
      auto b = vertex_start_index + x_index + 1;
      auto c = vertex_start_index + (span + 1) + x_index;
      indices.at((y_index * span + x_index) * 6) = a;
      indices.at((y_index * span + x_index) * 6 + 1) = b;
      indices.at((y_index * span + x_index) * 6 + 2) = c;
      a = vertex_start_index + (span + 1) + x_index + 1;
      b = vertex_start_index + (span + 1) + x_index;
      c = vertex_start_index + x_index + 1;
      indices.at((y_index * span + x_index) * 6 + 3) = a;
      indices.at((y_index * span + x_index) * 6 + 4) = b;
      indices.at((y_index * span + x_index) * 6 + 5) = c;
    }
  });

  VertexAttributes attributes{};
  attributes.tex_coord = true;
  mesh->SetVertices(attributes, vertices, indices);
}

void LogGrader::GenerateSurface(const std::shared_ptr<ParticleInfoList>& surface,
                                const LogWoodMeshGenerationSettings& mesh_generator_settings, const int start_x,
                                const int end_x) const {
  if (!surface)
    return;
  if (m_log_wood.m_intersections.size() < 2)
    return;

  const float avg_distance = m_log_wood.GetAverageDistance();
  const float circle_length = 2.0f * glm::pi<float>() * avg_distance;
  const float flat_x_step = circle_length / 360;
  const float log_length = m_log_wood.m_length;
  const int y_step_size = log_length / mesh_generator_settings.m_y_subdivision;
  const float y_step = log_length / y_step_size;

  const int span = end_x - start_x;
  std::vector<ParticleInfo> particle_infos;

  particle_infos.resize(y_step_size * (span + 1));

  Jobs::RunParallelFor(y_step_size + 1, [&](const unsigned y_index) {
    const float y = y_step * static_cast<float>(y_index);
    const float intersection_avg_distance = m_log_wood.GetAverageDistance(y);
    for (int x_index = 0; x_index <= span; x_index++) {
      const float x = static_cast<float>(x_index);
      const float center_distance = m_log_wood.GetCenterDistance(y, x);
      const auto position =
          glm::vec3(flat_x_step * static_cast<float>(x_index - span), y, center_distance - intersection_avg_distance);
      const auto size = glm::vec3(flat_x_step, 0.f, y_step);
      const auto rotation = glm::quat(glm::radians(glm::vec3(-90, 0, 0)));
      auto& particle_info = particle_infos.at(y_index * span + x_index);
      particle_info.instance_matrix.value = glm::translate(position) * glm::mat4_cast(rotation) * glm::scale(size);
      const auto color = m_log_wood.GetColor(y, x + start_x);
      particle_info.instance_color = {color.v0, color.v1, color.v2, color.v3};
    }
  });
  surface->SetParticleInfos(particle_infos);
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

  auto branch_shape = m_branch_shape.Get<BarkDescriptor>();
  if (!branch_shape) {
    branch_shape = ProjectManager::CreateTemporaryAsset<BarkDescriptor>();
    m_branch_shape = branch_shape;
    branch_shape->bark_depth = branch_shape->base_depth = 0.1f;
  }
  InitializeLogRandomly(m_procedural_log_parameters, branch_shape);
  m_best_grading_index = 0;
  m_log_wood.CalculateGradingData(m_available_best_grading);
  m_log_wood.ColorBasedOnGrading(m_available_best_grading[m_best_grading_index]);
  RefreshMesh(m_available_best_grading[m_best_grading_index]);
}

void LogGrader::InitializeMeshRenderer(const LogWoodMeshGenerationSettings& mesh_generator_settings) const {
  ClearMeshRenderer();
  const auto scene = GetScene();
  const auto self = GetOwner();
  if (const auto cylinder_entity = scene->CreateEntity("Log Wood Cylinder Mesh"); scene->IsEntityValid(cylinder_entity)) {
    scene->SetParent(cylinder_entity, self);
    const auto mesh = ProjectManager::CreateTemporaryAsset<Mesh>();
    GenerateCylinderMesh(mesh, mesh_generator_settings);
    const auto material = ProjectManager::CreateTemporaryAsset<Material>();
    const auto mesh_renderer = scene->GetOrSetPrivateComponent<MeshRenderer>(cylinder_entity).lock();
    material->material_properties.roughness = 1.0f;
    material->material_properties.metallic = 0.0f;
    material->vertex_color_only = true;
    mesh_renderer->mesh = mesh;
    mesh_renderer->material = material;
  }

  const float avg_distance = m_log_wood.GetMaxAverageDistance();
  const float circle_length = 2.0f * glm::pi<float>() * avg_distance;
  float x_offset = avg_distance * 1.5f;
  if (const auto flat_entity1 = scene->CreateEntity("Log Wood Flat Mesh 1"); scene->IsEntityValid(flat_entity1)) {
    scene->SetParent(flat_entity1, self);
    Transform transform{};
    transform.SetPosition({x_offset, 0, 0});
    transform.SetEulerRotation(glm::radians(glm::vec3(0, 180, 0)));
    scene->SetDataComponent(flat_entity1, transform);
    const auto mesh = ProjectManager::CreateTemporaryAsset<Mesh>();
    GenerateFlatMesh(mesh, mesh_generator_settings, 90, 180);
    const auto material = ProjectManager::CreateTemporaryAsset<Material>();
    const auto mesh_renderer = scene->GetOrSetPrivateComponent<MeshRenderer>(flat_entity1).lock();
    material->material_properties.roughness = 1.0f;
    material->material_properties.metallic = 0.0f;
    material->vertex_color_only = true;
    mesh_renderer->mesh = mesh;
    mesh_renderer->material = material;
  }
  x_offset += circle_length / 4.0f + 0.2f;
  if (const auto flat_entity2 = scene->CreateEntity("Log Wood Flat Mesh 2"); scene->IsEntityValid(flat_entity2)) {
    scene->SetParent(flat_entity2, self);
    Transform transform{};
    transform.SetPosition({x_offset, 0, 0});
    transform.SetEulerRotation(glm::radians(glm::vec3(0, 180, 0)));
    scene->SetDataComponent(flat_entity2, transform);
    const auto mesh = ProjectManager::CreateTemporaryAsset<Mesh>();
    GenerateFlatMesh(mesh, mesh_generator_settings, 0, 90);
    const auto material = ProjectManager::CreateTemporaryAsset<Material>();
    const auto mesh_renderer = scene->GetOrSetPrivateComponent<MeshRenderer>(flat_entity2).lock();
    material->material_properties.roughness = 1.0f;
    material->material_properties.metallic = 0.0f;
    material->vertex_color_only = true;
    mesh_renderer->mesh = mesh;
    mesh_renderer->material = material;
  }
  x_offset += circle_length / 4.0f + 0.2f;
  if (const auto flat_entity3 = scene->CreateEntity("Log Wood Flat Mesh 3"); scene->IsEntityValid(flat_entity3)) {
    scene->SetParent(flat_entity3, self);
    Transform transform{};
    transform.SetPosition({x_offset, 0, 0});
    transform.SetEulerRotation(glm::radians(glm::vec3(0, 180, 0)));
    scene->SetDataComponent(flat_entity3, transform);
    const auto mesh = ProjectManager::CreateTemporaryAsset<Mesh>();
    GenerateFlatMesh(mesh, mesh_generator_settings, 270, 360);
    const auto material = ProjectManager::CreateTemporaryAsset<Material>();
    const auto mesh_renderer = scene->GetOrSetPrivateComponent<MeshRenderer>(flat_entity3).lock();
    material->material_properties.roughness = 1.0f;
    material->material_properties.metallic = 0.0f;
    material->vertex_color_only = true;
    mesh_renderer->mesh = mesh;
    mesh_renderer->material = material;
  }
  x_offset += circle_length / 4.0f + 0.2f;
  if (const auto flat_entity4 = scene->CreateEntity("Log Wood Flat Mesh 4"); scene->IsEntityValid(flat_entity4)) {
    scene->SetParent(flat_entity4, self);
    Transform transform{};
    transform.SetPosition({x_offset, 0, 0});
    transform.SetEulerRotation(glm::radians(glm::vec3(0, 180, 0)));
    scene->SetDataComponent(flat_entity4, transform);
    const auto mesh = ProjectManager::CreateTemporaryAsset<Mesh>();
    GenerateFlatMesh(mesh, mesh_generator_settings, 180, 270);
    const auto material = ProjectManager::CreateTemporaryAsset<Material>();
    const auto mesh_renderer = scene->GetOrSetPrivateComponent<MeshRenderer>(flat_entity4).lock();
    material->material_properties.roughness = 1.0f;
    material->material_properties.metallic = 0.0f;
    material->vertex_color_only = true;
    mesh_renderer->mesh = mesh;
    mesh_renderer->material = material;
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
