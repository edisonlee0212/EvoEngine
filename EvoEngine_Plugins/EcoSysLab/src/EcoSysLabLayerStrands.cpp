//
// Created by lllll on 11/1/2022.
//

#include "EcoSysLabLayer.hpp"
#ifdef CUDA_MODULE_PLUGIN
#  include <RayTracerLayer.hpp>
#endif

#include "ClassRegistry.hpp"
#include "DsColliders.hpp"
#include "DsOperators.hpp"
#include "DynamicTreeStrands.hpp"
#include "Soil.hpp"
#include "SpatialPlantDistributionSimulator.hpp"
#include "Tree.hpp"
#include "TreeStructor.hpp"

using namespace eco_sys_lab_plugin;

PrivateComponentRegistration<DsBoxCollider> ds_box_collider_registry("DsBoxCollider");
PrivateComponentRegistration<DsSphereCollider> ds_sphere_collider_registry("DsSphereCollider");
PrivateComponentRegistration<DsCylinderCollider> ds_cylinder_collider_registry("DsCylinderCollider");
PrivateComponentRegistration<DynamicTreeStrands> dynamic_tree_strands_registry("DynamicTreeStrands");

void EcoSysLabLayer::GenerateStrandModelProfiles() const {
  const auto scene = GetScene();
  if (const std::vector<Entity>* tree_entities = scene->UnsafeGetPrivateComponentOwnersList<Tree>();
      tree_entities && !tree_entities->empty()) {
    const auto copied_entities = *tree_entities;
    for (auto tree_entity : copied_entities) {
      if (const auto tree = scene->GetOrSetPrivateComponent<Tree>(tree_entity).lock(); tree->generate_mesh)
        tree->BuildStrandModel();
    }
  }
}

void EcoSysLabLayer::GenerateStrandModelMeshes(
    const StrandModelMeshGeneratorSettings& target_strand_model_mesh_generator_settings) const {
  const auto scene = GetScene();
  if (const std::vector<Entity>* tree_entities = scene->UnsafeGetPrivateComponentOwnersList<Tree>();
      tree_entities && !tree_entities->empty()) {
    const auto copied_entities = *tree_entities;
    for (auto tree_entity : copied_entities) {
      if (const auto tree = scene->GetOrSetPrivateComponent<Tree>(tree_entity).lock(); tree->generate_mesh)
        tree->InitializeStrandModelMeshRenderer(target_strand_model_mesh_generator_settings);
    }
  }
}

void EcoSysLabLayer::GenerateStrandRenderers() const {
  const auto scene = GetScene();
  if (const std::vector<Entity>* tree_entities = scene->UnsafeGetPrivateComponentOwnersList<Tree>();
      tree_entities && !tree_entities->empty()) {
    const auto copied_entities = *tree_entities;
    for (auto tree_entity : copied_entities) {
      if (const auto tree = scene->GetOrSetPrivateComponent<Tree>(tree_entity).lock(); tree->generate_mesh)
        tree->InitializeStrandRenderer();
    }
  }
}

void EcoSysLabLayer::ClearStrandRenderers() const {
  const auto scene = GetScene();
  if (const std::vector<Entity>* tree_entities = scene->UnsafeGetPrivateComponentOwnersList<Tree>();
      tree_entities && !tree_entities->empty()) {
    const auto copied_entities = *tree_entities;
    for (auto tree_entity : copied_entities) {
      const auto tree = scene->GetOrSetPrivateComponent<Tree>(tree_entity).lock();
      tree->ClearStrandRenderer();
    }
  }
}

void EcoSysLabLayer::ClearStrandModelMeshes() const {
  const auto scene = GetScene();
  if (const std::vector<Entity>* tree_entities = scene->UnsafeGetPrivateComponentOwnersList<Tree>();
      tree_entities && !tree_entities->empty()) {
    const auto copied_entities = *tree_entities;
    for (auto tree_entity : copied_entities) {
      const auto tree = scene->GetOrSetPrivateComponent<Tree>(tree_entity).lock();
      tree->ClearStrandModelMeshRenderer();
    }
  }
}

void EcoSysLabLayer::GenerateDynamicStrandsForAllTrees() const {
  const auto scene = GetScene();
  if (const std::vector<Entity>* tree_entities = scene->UnsafeGetPrivateComponentOwnersList<Tree>();
      tree_entities && !tree_entities->empty()) {
    for (auto tree_entity : *tree_entities) {
      const auto tree = scene->GetOrSetPrivateComponent<Tree>(tree_entity).lock();
      tree->BuildStrandModel();
      const auto ds = scene->GetOrSetPrivateComponent<DynamicTreeStrands>(tree_entity).lock();
      ds->strand_model_skeleton = tree->strand_model.strand_model_skeleton;
      ds->UpdateDynamicStrands();
    }
  }
}

void EcoSysLabLayer::DynamicStrandsVisualization(const std::shared_ptr<EditorLayer>& editor_layer) const {
  const auto scene = GetScene();
  const std::vector<Entity>* dts_entities = scene->UnsafeGetPrivateComponentOwnersList<DynamicTreeStrands>();
  const auto for_each_dts_entity =
      [&](const std::function<void(const std::shared_ptr<DynamicTreeStrands>& dts)>& action) {
        if (dts_entities && !dts_entities->empty()) {
          for (const auto& i : *dts_entities) {
            const auto dts = scene->GetOrSetPrivateComponent<DynamicTreeStrands>(i).lock();
            action(dts);
          }
        }
      };
  const auto* box_collider_entities = scene->UnsafeGetPrivateComponentOwnersList<DsBoxCollider>();
  const auto* sphere_collider_entities = scene->UnsafeGetPrivateComponentOwnersList<DsSphereCollider>();
  const auto* cylinder_collider_entities = scene->UnsafeGetPrivateComponentOwnersList<DsCylinderCollider>();
  const auto for_each_collider_entity =
      [&](const std::function<void(const std::shared_ptr<IDsCollider>& dts)>& action) {
        if (box_collider_entities && !box_collider_entities->empty()) {
          for (const auto& i : *box_collider_entities) {
            const auto box_collider = scene->GetOrSetPrivateComponent<DsBoxCollider>(i).lock();
            action(std::dynamic_pointer_cast<IDsCollider>(box_collider));
          }
        }
        if (sphere_collider_entities && !sphere_collider_entities->empty()) {
          for (const auto& i : *sphere_collider_entities) {
            const auto sphere_collider = scene->GetOrSetPrivateComponent<DsSphereCollider>(i).lock();
            action(std::dynamic_pointer_cast<IDsCollider>(sphere_collider));
          }
        }
        if (cylinder_collider_entities && !cylinder_collider_entities->empty()) {
          for (const auto& i : *cylinder_collider_entities) {
            const auto cylinder_collider = scene->GetOrSetPrivateComponent<DsCylinderCollider>(i).lock();
            action(std::dynamic_pointer_cast<IDsCollider>(cylinder_collider));
          }
        }
      };
  ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2{0, 0});
  if (ImGui::Begin("Plant Visual")) {
    if (ImGui::BeginChild("InternodeCameraRenderer", ImVec2(0, 0), false)) {
      const ImVec2 canvas_p0 = ImGui::GetWindowPos() + ImVec2(1, 0);  // ImDrawList API uses screen coordinates!
      const ImVec2 canvas_size = ImGui::GetWindowSize();              // Resize canvas to what's available
      const ImVec2 canvas_p1 = ImVec2(canvas_p0.x + canvas_size.x - 2, canvas_p0.y + canvas_size.y - 1);
      ImDrawList* draw_list = ImGui::GetWindowDrawList();
      // Draw border and background color
      // draw_list->AddRect(canvas_p0, canvas_p1, IM_COL32(255, 255, 255, 255));
      draw_list->PushClipRect(canvas_p0, canvas_p1, true);
      if (!tree_visualization_settings_.enable ||
          tree_operator_mode == static_cast<unsigned>(TreeOperatorMode::Disabled) ||
          tree_operator_mode == static_cast<unsigned>(TreeOperatorMode::Select)) {
        if (visualization_camera_window_focused_ &&
            editor_layer->GetKey(GLFW_MOUSE_BUTTON_RIGHT) != Input::KeyActionType::Hold &&
            editor_layer->GetKey(GLFW_MOUSE_BUTTON_RIGHT) != Input::KeyActionType::Press) {
          static bool is_box_selection_previously = false;
          static bool is_operating_previously = false;
          static glm::vec2 strands_operator_mouse_start;
          static glm::vec2 strands_operator_mouse_current;
          static std::vector<glm::vec2> strand_operator_mouse_points;

          bool mouse_drag = true;
          glm::vec2 mouse_valid_position =
              glm::clamp(visualization_camera_mouse_position, {0, 0},
                         {visualization_camera_resolution_x - 1, visualization_camera_resolution_y - 1});
          if (editor_layer->GetKey(GLFW_MOUSE_BUTTON_LEFT) != Input::KeyActionType::Hold) {
            mouse_drag = false;
          }
          const auto camera_rotation = editor_layer->GetSceneCameraRotation();
          const auto camera_position = editor_layer->GetSceneCameraPosition();
          const glm::vec3 camera_front = camera_rotation * glm::vec3(0, 0, -1);
          const glm::vec3 camera_up = camera_rotation * glm::vec3(0, 1, 0);
          const glm::vec3 camera_right = camera_rotation * glm::vec3(1, 0, 0);

          if (mouse_drag && !is_operating_previously && !is_box_selection_previously) {
            strands_operator_mouse_start = mouse_valid_position;
            strand_operator_mouse_points.clear();
          }
          const auto camera_projection_view = visualization_camera_->GetProjection() *
                                              glm::lookAt(camera_position, camera_position + camera_front, camera_up);
          if (editor_layer->GetKey(GLFW_KEY_ESCAPE) == Input::KeyActionType::Press) {
            for_each_dts_entity([&](const std::shared_ptr<DynamicTreeStrands>& dts) {
              dts->box_selection_operator->enabled = true;
              dts->box_selection_operator->Update(
                  strands_operator_mouse_start / glm::vec2(canvas_size.x, canvas_size.y),
                  strands_operator_mouse_current / glm::vec2(canvas_size.x, canvas_size.y), {}, 4);
            });
            is_operating_previously = false;
            is_box_selection_previously = false;
          } else if (mouse_drag) {
            strands_operator_mouse_current = mouse_valid_position;
            if (strand_operator_mouse_points.empty() ||
                glm::distance(strand_operator_mouse_points.back(), strands_operator_mouse_current) > 2) {
              strand_operator_mouse_points.emplace_back(strands_operator_mouse_current);
            }
            if (editor_layer->GetKey(GLFW_KEY_Q) == Input::KeyActionType::Hold || is_box_selection_previously) {
              is_box_selection_previously = true;
              draw_list->AddQuad(canvas_p0 + ImVec2(strands_operator_mouse_start.x, strands_operator_mouse_start.y),
                                 canvas_p0 + ImVec2(strands_operator_mouse_start.x, strands_operator_mouse_current.y),
                                 canvas_p0 + ImVec2(strands_operator_mouse_current.x, strands_operator_mouse_current.y),
                                 canvas_p0 + ImVec2(strands_operator_mouse_current.x, strands_operator_mouse_start.y),
                                 IM_COL32(255, 255, 255, 255));
              for_each_dts_entity([&](const std::shared_ptr<DynamicTreeStrands>& dts) {
                dts->box_selection_operator->enabled = true;
                dts->box_selection_operator->Update(
                    strands_operator_mouse_start / glm::vec2(canvas_size.x, canvas_size.y),
                    strands_operator_mouse_current / glm::vec2(canvas_size.x, canvas_size.y), camera_projection_view,
                    editor_layer->GetKey(GLFW_KEY_R) != Input::KeyActionType::Hold ? 0 : 1);
              });
            } else if (editor_layer->GetKey(GLFW_KEY_E) == Input::KeyActionType::Hold || is_operating_previously) {
              is_operating_previously = true;

              switch (static_cast<DynamicStrandsSettings::OperatorMode>(dynamic_strands_settings_.operator_mode)) {
                case DynamicStrandsSettings::OperatorMode::Drag: {
                  draw_list->AddLine(
                      canvas_p0 + ImVec2(strands_operator_mouse_start.x, strands_operator_mouse_start.y),
                      canvas_p0 + ImVec2(strands_operator_mouse_current.x, strands_operator_mouse_current.y),
                      IM_COL32(255, 255, 255, 255));
                  const auto screen_vector = strands_operator_mouse_current - strands_operator_mouse_start;
                  const float line_distance = glm::length(screen_vector);
                  draw_list->AddCircle(
                      canvas_p0 + ImVec2(strands_operator_mouse_current.x, strands_operator_mouse_current.y),
                      line_distance * 0.05f, IM_COL32(255, 255, 255, 255));
                  draw_list->AddCircle(
                      canvas_p0 + ImVec2(strands_operator_mouse_start.x, strands_operator_mouse_start.y), 5.0f,
                      IM_COL32(255, 255, 255, 255));

                  const glm::vec3 acceleration = dynamic_strands_settings_.drag_multiplier *
                                                 (camera_right * screen_vector.x - camera_up * screen_vector.y);
                  for_each_dts_entity([&](const std::shared_ptr<DynamicTreeStrands>& dts) {
                    dts->drag_operator->enabled = true;
                    dts->drag_operator->Update(acceleration);
                  });
                  break;
                }
                case DynamicStrandsSettings::OperatorMode::Saw: {
                  draw_list->AddCircleFilled(
                      canvas_p0 + ImVec2(strands_operator_mouse_start.x, strands_operator_mouse_start.y), 2.0f,
                      IM_COL32(255, 0, 0, 255));
                  draw_list->AddCircleFilled(
                      canvas_p0 + ImVec2(strands_operator_mouse_current.x, strands_operator_mouse_current.y), 2.0f,
                      IM_COL32(255, 0, 0, 255));
                  for (uint32_t line_index = 0; line_index < strand_operator_mouse_points.size() - 1; line_index++) {
                    draw_list->AddLine(canvas_p0 + ImVec2(strand_operator_mouse_points[line_index].x,
                                                          strand_operator_mouse_points[line_index].y),
                                       canvas_p0 + ImVec2(strand_operator_mouse_points[line_index + 1].x,
                                                          strand_operator_mouse_points[line_index + 1].y),
                                       IM_COL32(255, 0, 0, 128));
                  }
                  break;
                }
                case DynamicStrandsSettings::OperatorMode::LineCut: {
                  draw_list->AddCircleFilled(
                      canvas_p0 + ImVec2(strands_operator_mouse_start.x, strands_operator_mouse_start.y), 4.0f,
                      IM_COL32(255, 0, 0, 255));
                  draw_list->AddLine(
                      canvas_p0 + ImVec2(strands_operator_mouse_start.x, strands_operator_mouse_start.y),
                      canvas_p0 + ImVec2(strands_operator_mouse_current.x, strands_operator_mouse_current.y),
                      IM_COL32(255, 0, 0, 128));
                  draw_list->AddCircleFilled(
                      canvas_p0 + ImVec2(strands_operator_mouse_current.x, strands_operator_mouse_current.y), 4.0f,
                      IM_COL32(255, 0, 0, 255));
                  break;
                }
                default:
                  break;
              }
            }
          } else {
            if (is_box_selection_previously) {
              for_each_dts_entity([&](const std::shared_ptr<DynamicTreeStrands>& dts) {
                dts->box_selection_operator->enabled = true;
                dts->box_selection_operator->Update(
                    strands_operator_mouse_start / glm::vec2(canvas_size.x, canvas_size.y),
                    strands_operator_mouse_current / glm::vec2(canvas_size.x, canvas_size.y), camera_projection_view,
                    editor_layer->GetKey(GLFW_KEY_R) != Input::KeyActionType::Hold ? 2 : 3);
              });
            } else if (is_operating_previously) {
              switch (static_cast<DynamicStrandsSettings::OperatorMode>(dynamic_strands_settings_.operator_mode)) {
                case DynamicStrandsSettings::OperatorMode::Saw: {
                  for_each_dts_entity([&](const std::shared_ptr<DynamicTreeStrands>& dts) {
                    dts->saw_operator->enabled = true;
                    auto points = strand_operator_mouse_points;
                    for (auto& point : points) {
                      point /= glm::vec2(canvas_size.x, canvas_size.y);
                    }
                    dts->saw_operator->Update(points, camera_projection_view,
                                              dynamic_strands_settings_.cut_bend_twist_bundle_only ? 1 : 0);
                  });
                  break;
                }
                case DynamicStrandsSettings::OperatorMode::LineCut: {
                  for_each_dts_entity([&](const std::shared_ptr<DynamicTreeStrands>& dts) {
                    dts->line_cut_operator->enabled = true;
                    dts->line_cut_operator->Update(
                        strands_operator_mouse_start / glm::vec2(canvas_size.x, canvas_size.y),
                        strands_operator_mouse_current / glm::vec2(canvas_size.x, canvas_size.y),
                        camera_projection_view, dynamic_strands_settings_.cut_bend_twist_bundle_only ? 1 : 0);
                  });
                  break;
                }
                default:
                  break;
              }
            }

            strand_operator_mouse_points.clear();
          }
          is_operating_previously = mouse_drag && is_operating_previously;
          is_box_selection_previously = mouse_drag && is_box_selection_previously;
        }
        if (dynamic_strands_settings_.transform_mode != 0) {
          const auto imguizmo_transform = [&](glm::mat4& global_transform) {
            ImGuizmo::SetOrthographic(false);
            ImGuizmo::SetDrawlist();
            ImGuizmo::SetRect(ImGui::GetWindowPos().x, ImGui::GetWindowPos().y, visualization_camera_resolution_x,
                              visualization_camera_resolution_y);
            glm::mat4 camera_view = glm::inverse(glm::translate(editor_layer->GetSceneCameraPosition()) *
                                                 glm::mat4_cast(editor_layer->GetSceneCameraRotation()));
            glm::mat4 camera_projection = visualization_camera_->GetProjection();
            auto op = ImGuizmo::OPERATION::TRANSLATE;
            switch (dynamic_strands_settings_.transform_mode) {
              case 2: {
                op = ImGuizmo::OPERATION::ROTATE;
                break;
              }
              default:
                break;
            }
            ImGuizmo::Manipulate(glm::value_ptr(camera_view), glm::value_ptr(camera_projection), op, ImGuizmo::LOCAL,
                                 glm::value_ptr(global_transform));
            return ImGuizmo::IsUsing();
          };
          for_each_dts_entity([&](const std::shared_ptr<DynamicTreeStrands>& dts) {
            const auto entity = dts->GetOwner();
            if (editor_layer->GetSelectedEntity() != entity)
              return;
            auto gt = scene->GetDataComponent<GlobalTransform>(entity);
            if (imguizmo_transform(gt.value)) {
              scene->SetDataComponent(entity, gt);
            }
          });
          for_each_collider_entity([&](const std::shared_ptr<IDsCollider>& collider) {
            const auto entity = collider->GetOwner();
            if (editor_layer->GetSelectedEntity() != entity)
              return;
            auto gt = scene->GetDataComponent<GlobalTransform>(entity);
            if (imguizmo_transform(gt.value)) {
              scene->SetDataComponent(entity, gt);
            }
          });
        }
      }
      draw_list->PopClipRect();
    }
    ImGui::EndChild();
  }
  ImGui::End();
  ImGui::PopStyleVar();
}

void EcoSysLabLayer::DynamicStrandsSettings::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  ImGui::Combo("Transform Mode", {"None", "Translate", "Rotate"}, transform_mode);
  ImGui::Combo("Operator Mode", {"Drag", "Saw", "Line Cut"}, operator_mode);

  switch (static_cast<OperatorMode>(operator_mode)) {
    case OperatorMode::Drag: {
      ImGui::DragFloat("Drag acceleration multiplier", &drag_multiplier, 0.001f, 0.0f, 1.0f);
      break;
    }
    case OperatorMode::Saw:
    case OperatorMode::LineCut: {
      ImGui::Checkbox("Cut Bend/Twist/Bundle only", &cut_bend_twist_bundle_only);
      break;
    }
  }

  ImGui::Checkbox("Physics", &enable_physics);
  if (ImGui::TreeNode("Physics parameters")) {
    physics_parameters.OnInspect(editor_layer);
    ImGui::TreePop();
  }

  ImGui::Checkbox("Visualization", &enable);
  if (ImGui::TreeNode("Visualization settings")) {
    visualization_parameters.OnInspect(editor_layer);
    ImGui::TreePop();
  }
  ImGui::Checkbox("Rendering", &enable_rendering);
}
