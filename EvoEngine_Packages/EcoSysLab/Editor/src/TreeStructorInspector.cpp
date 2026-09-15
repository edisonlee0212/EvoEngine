#include <unordered_set>
#include "BasicFoliageDescriptor.hpp"
#include "EcoSysLabAuthoringInspectors.hpp"
#include "EcoSysLabEditorLayer.hpp"
#include "EcoSysLabLayer.hpp"
#include "EcoSysLabSerializationAdapters.hpp"
#include "EcoSysLabSettingsEditor.hpp"
#include "EditorFileDialogs.hpp"
#include "EditorLayer.hpp"
#include "Platform.hpp"
#include "SDKInspectionAdapters.hpp"
#include "TreeStructor.hpp"
#include "rapidcsv.h"

namespace {
void SetMaterialBaseColor(const std::shared_ptr<evo_engine::Material>& material, const glm::vec3& color) {
  material->material_data.shade_material.pbr_base_color_factor = glm::vec4(color, 1.0f);
  material->MarkDirty();
}
}  // namespace

using namespace evo_engine;
using namespace eco_sys_lab_package;
void TreeStructorInspector::FormInfoEntities(const TreeStructor& target) const {
  const auto scene = target.GetScene();
  const auto owner = target.GetOwner();
  const auto children = scene->GetChildren(owner);
  for (const auto& i : children) {
    if (scene->GetEntityName(i) == "Info") {
      scene->DeleteEntity(i);
    }
  }

  const auto info_entity = scene->CreateEntity("Info");
  scene->SetParent(info_entity, owner);
  if (enable_allocated_points) {
    const auto allocated_point_info_entity = scene->CreateEntity("Allocated Points");
    scene->SetParent(allocated_point_info_entity, info_entity);
    const auto particles = scene->GetOrSetPrivateComponent<Particles>(allocated_point_info_entity).lock();
    particles->mesh = Resources::GetInstance().GetPrimitives().sphere;
    particles->particle_info_list = allocated_point_info_list;
    const auto material = AssetManager::CreateTemporaryAsset<Material>();
    particles->material = material;
    SetMaterialBaseColor(material, allocated_point_color);
  }
  if (enable_scattered_points) {
    const auto scatter_point_info_entity = scene->CreateEntity("Scattered Points");
    scene->SetParent(scatter_point_info_entity, info_entity);
    const auto particles = scene->GetOrSetPrivateComponent<Particles>(scatter_point_info_entity).lock();
    particles->mesh = Resources::GetInstance().GetPrimitives().sphere;
    particles->particle_info_list = scattered_point_info_list;
    const auto material = AssetManager::CreateTemporaryAsset<Material>();
    particles->material = material;
    SetMaterialBaseColor(material, scatter_point_color);
  }
  if (enable_scattered_point_connections) {
    const auto scattered_point_connection_info_entity = scene->CreateEntity("Scattered Point Connections");
    scene->SetParent(scattered_point_connection_info_entity, info_entity);
    scene->SetEnable(scattered_point_connection_info_entity, false);
    const auto particles = scene->GetOrSetPrivateComponent<Particles>(scattered_point_connection_info_entity).lock();
    particles->mesh = Resources::GetInstance().GetPrimitives().cylinder;
    particles->particle_info_list = scattered_point_connection_info_list;
    const auto material = AssetManager::CreateTemporaryAsset<Material>();
    particles->material = material;
    SetMaterialBaseColor(material, scattered_point_connection_color);
  }
  if (enable_candidate_branch_connections) {
    const auto candidate_branch_connection_info_entity = scene->CreateEntity("Candidate Branch Connections");
    scene->SetEnable(candidate_branch_connection_info_entity, false);
    scene->SetParent(candidate_branch_connection_info_entity, info_entity);
    const auto particles = scene->GetOrSetPrivateComponent<Particles>(candidate_branch_connection_info_entity).lock();
    particles->mesh = Resources::GetInstance().GetPrimitives().cylinder;
    particles->particle_info_list = candidate_branch_connection_info_list;
    const auto material = AssetManager::CreateTemporaryAsset<Material>();
    particles->material = material;
    SetMaterialBaseColor(material, candidate_branch_connection_color);
  }
  if (enable_reversed_candidate_branch_connections) {
    const auto reversed_candidate_branch_connection_info_entity =
        scene->CreateEntity("Reversed Candidate Branch Connections");
    scene->SetEnable(reversed_candidate_branch_connection_info_entity, false);
    scene->SetParent(reversed_candidate_branch_connection_info_entity, info_entity);
    const auto particles =
        scene->GetOrSetPrivateComponent<Particles>(reversed_candidate_branch_connection_info_entity).lock();
    particles->mesh = Resources::GetInstance().GetPrimitives().cylinder;
    particles->particle_info_list = reversed_candidate_branch_connection_info_list;
    const auto material = AssetManager::CreateTemporaryAsset<Material>();
    particles->material = material;
    SetMaterialBaseColor(material, reversed_candidate_branch_connection_color);
  }
  if (enable_filtered_branch_connections) {
    const auto filtered_branch_connection_info_entity = scene->CreateEntity("Filtered Branch Connections");
    scene->SetEnable(filtered_branch_connection_info_entity, false);
    scene->SetParent(filtered_branch_connection_info_entity, info_entity);
    const auto particles = scene->GetOrSetPrivateComponent<Particles>(filtered_branch_connection_info_entity).lock();
    particles->mesh = Resources::GetInstance().GetPrimitives().cylinder;
    particles->particle_info_list = filtered_branch_connection_info_list;
    const auto material = AssetManager::CreateTemporaryAsset<Material>();
    particles->material = material;
    SetMaterialBaseColor(material, filtered_branch_connection_color);
  }
  if (enable_selected_branch_connections) {
    const auto branch_connection_info_entity = scene->CreateEntity("Selected Branch Connections");
    scene->SetParent(branch_connection_info_entity, info_entity);
    const auto particles = scene->GetOrSetPrivateComponent<Particles>(branch_connection_info_entity).lock();
    particles->mesh = Resources::GetInstance().GetPrimitives().cylinder;
    particles->particle_info_list = selected_branch_connection_info_list;
    const auto material = AssetManager::CreateTemporaryAsset<Material>();
    particles->material = material;
    SetMaterialBaseColor(material, selected_branch_connection_color);
  }
  if (enable_scatter_point_to_branch_connections) {
    const auto scatter_point_to_branch_connection = scene->CreateEntity("Scatter Point To Branch Connections");
    scene->SetParent(scatter_point_to_branch_connection, info_entity);
    scene->SetEnable(scatter_point_to_branch_connection, false);
    const auto particles = scene->GetOrSetPrivateComponent<Particles>(scatter_point_to_branch_connection).lock();
    particles->mesh = Resources::GetInstance().GetPrimitives().cylinder;
    particles->particle_info_list = scatter_point_to_branch_connection_info_list;
    const auto material = AssetManager::CreateTemporaryAsset<Material>();
    particles->material = material;
    SetMaterialBaseColor(material, scatter_point_to_branch_connection_color);
  }
  if (enable_selected_branches) {
    const auto predicted_branch_connection_info_entity = scene->CreateEntity("Selected Branches");
    scene->SetParent(predicted_branch_connection_info_entity, info_entity);
    const auto particles = scene->GetOrSetPrivateComponent<Particles>(predicted_branch_connection_info_entity).lock();
    particles->mesh = Resources::GetInstance().GetPrimitives().cylinder;
    particles->particle_info_list = selected_branch_info_list;
    const auto material = AssetManager::CreateTemporaryAsset<Material>();
    particles->material = material;
    SetMaterialBaseColor(material, selected_branch_color);
  }
}
bool TreeStructorInspector::Inspect(InspectorContext& context, TreeStructor& target) {
  const auto& editor_layer = context.editor_layer;

  bool changed = false;

  if (!allocated_point_info_list)
    allocated_point_info_list = AssetManager::CreateTemporaryAsset<ParticleInfoList>();
  if (!scattered_point_info_list)
    scattered_point_info_list = AssetManager::CreateTemporaryAsset<ParticleInfoList>();
  if (!scattered_point_connection_info_list)
    scattered_point_connection_info_list = AssetManager::CreateTemporaryAsset<ParticleInfoList>();

  if (!candidate_branch_connection_info_list)
    candidate_branch_connection_info_list = AssetManager::CreateTemporaryAsset<ParticleInfoList>();
  if (!reversed_candidate_branch_connection_info_list)
    reversed_candidate_branch_connection_info_list = AssetManager::CreateTemporaryAsset<ParticleInfoList>();
  if (!filtered_branch_connection_info_list)
    filtered_branch_connection_info_list = AssetManager::CreateTemporaryAsset<ParticleInfoList>();
  if (!selected_branch_connection_info_list)
    selected_branch_connection_info_list = AssetManager::CreateTemporaryAsset<ParticleInfoList>();

  if (!scatter_point_to_branch_connection_info_list)
    scatter_point_to_branch_connection_info_list = AssetManager::CreateTemporaryAsset<ParticleInfoList>();
  if (!selected_branch_info_list)
    selected_branch_info_list = AssetManager::CreateTemporaryAsset<ParticleInfoList>();

  bool refresh_data = false;

  editor_layer->DragAndDropButton<TreeDescriptor>(target.tree_descriptor_ref, "TreeDescriptor", true);

  ImGui::DragFloat("Import scale", &import_scale, 0.01f, 0.01f, 10.0f);
  EditorFileDialogs::OpenFile(
      "Load YAML", "YAML", {".yml"},
      [&](const std::filesystem::path& path) {
        target.ImportGraph(path, import_scale);
        refresh_data = true;
      },
      false);

  if (!target.tree_parts.empty()) {
    if (ImGui::TreeNodeEx("Graph Settings")) {
      InspectSettings(target.connectivity_graph_settings);
      if (ImGui::Button("Rebuild Voxel Grid")) {
        target.BuildVoxelGrid();
      }
      ImGui::TreePop();
    }
    if (ImGui::TreeNodeEx("Reconstruction Settings")) {
      InspectSettings(target.reconstruction_settings);
      ImGui::TreePop();
    }
    if (ImGui::Button("Build Skeletons")) {
      target.EstablishConnectivityGraph();
      target.BuildSkeletons();
      refresh_data = true;
    }
    if (ImGui::Button("Form forest")) {
      if (target.branch_connections.empty()) {
        target.skeletons.clear();
        target.EstablishConnectivityGraph();
        refresh_data = true;
      }
      if (target.skeletons.empty()) {
        target.BuildSkeletons();
        refresh_data = true;
      }
      target.GenerateForest();
    }
    ImGui::SameLine();
    if (ImGui::Button("Clear forest")) {
      target.ClearForest();
    }
    ImGui::Separator();
    if (target.GetScene()->IsEntityValid(target.forest_ref.Get())) {
      const auto eco_sys_lab_layer = ApplicationContext::Get().GetLayer<EcoSysLabLayer>();
      EditorFileDialogs::SaveFile(
          "Export OBJ", "OBJ", {".obj"},
          [&](const std::filesystem::path& path) {
            target.ExportForestObj(eco_sys_lab_layer->mesh_generator_settings, path);
          },
          false);
      EditorFileDialogs::SaveFile(
          "Export flow graph", "YAML", {".yml"},
          [&](const std::filesystem::path& path) {
            target.ExportFlowGraphs(path);
          },
          false);
      EditorFileDialogs::SaveFile(
          "Export node graph", "YAML", {".yml"},
          [&](const std::filesystem::path& path) {
            target.ExportNodeGraphs(path);
          },
          false);
    }
  }

  ImGui::Checkbox("Debug Rendering", &enable_debug_rendering);
  if (enable_debug_rendering) {
    if (ImGui::TreeNode("Debug rendering settings")) {
      if (ImGui::Combo("Color mode", {"TreePart", "Branch", "Node"}, color_mode))
        refresh_data = true;
      ImGui::Checkbox("Use skeleton width", &use_real_branch_width);
      if (!use_real_branch_width)
        if (ImGui::DragFloat("Branch width", &predicted_branch_width, 0.0001f, 0.0001f, 1.0f, "%.4f"))
          refresh_data = true;
      if (ImGui::DragFloat("Connection width", &connection_width, 0.0001f, 0.0001f, 1.0f, "%.4f"))
        refresh_data = true;
      if (ImGui::DragFloat("Point size", &point_size, 0.0001f, 0.0001f, 1.0f, "%.4f"))
        refresh_data = true;
      if (ImGui::Checkbox("Allocated points", &debug_allocated_points))
        refresh_data = true;
      if (ImGui::Checkbox("Scattered points", &debug_scattered_points))
        refresh_data = true;
      if (debug_scattered_points) {
        if (ImGui::ColorEdit4("Scatter Point color", &scatter_point_color.x))
          refresh_data = true;
        if (ImGui::Checkbox("Render Point-Point links", &debug_scattered_point_connections))
          refresh_data = true;
        if (ImGui::Checkbox("Render Point-Branch links", &debug_scatter_point_to_branch_connections))
          refresh_data = true;
        if (debug_scattered_point_connections &&
            ImGui::ColorEdit4("Point-Point links color", &scattered_point_connection_color.x))
          refresh_data = true;
        if (debug_scatter_point_to_branch_connections &&
            ImGui::ColorEdit4("Point-Branch links color", &scatter_point_to_branch_connection_color.x))
          refresh_data = true;
      }

      if (ImGui::Checkbox("Candidate connections", &debug_candidate_connections))
        refresh_data = true;
      if (debug_candidate_connections &&
          ImGui::ColorEdit4("Candidate connection color", &candidate_branch_connection_color.x))
        refresh_data = true;
      if (ImGui::Checkbox("Reversed candidate connections", &debug_reversed_candidate_connections))
        refresh_data = true;
      if (debug_reversed_candidate_connections &&
          ImGui::ColorEdit4("Reversed candidate connection color", &reversed_candidate_branch_connection_color.x))
        refresh_data = true;
      if (ImGui::Checkbox("Filtered connections", &debug_filtered_connections))
        refresh_data = true;
      if (debug_filtered_connections &&
          ImGui::ColorEdit4("Filtered Connection Color", &filtered_branch_connection_color.x))
        refresh_data = true;
      if (ImGui::Checkbox("Selected Branch connections", &debug_selected_branch_connections))
        refresh_data = true;
      if (debug_selected_branch_connections &&
          ImGui::ColorEdit4("Branch Connection Color", &selected_branch_connection_color.x))
        refresh_data = true;
      if (ImGui::Checkbox("Selected branches", &debug_selected_branches))
        refresh_data = true;

      evo_engine::DrawSettingsGui(gizmo_settings.draw_settings);

      ImGui::TreePop();
    }

    if (ImGui::Button("Refresh Data")) {
      refresh_data = true;
    }

    if (target.GetHandle() != previous_handle)
      refresh_data = true;

    if (refresh_data) {
      previous_handle = target.GetHandle();
      const auto eco_sys_lab_layer = ApplicationContext::Get().GetLayer<EcoSysLabEditorLayer>();

      allocated_point_matrices.resize(target.allocated_points.size());

      predicted_branch_starts.resize(target.predicted_branches.size());
      predicted_branch_ends.resize(target.predicted_branches.size());
      predicted_branch_colors.resize(target.predicted_branches.size());
      predicted_branch_widths.resize(target.predicted_branches.size());
      switch (color_mode) {
        case 0: {
          // TreePart
          for (int i = 0; i < target.allocated_points.size(); i++) {
            allocated_point_matrices[i].instance_matrix.value =
                glm::translate(target.allocated_points[i].position) * glm::scale(glm::vec3(0.003f));
            allocated_point_matrices[i].instance_color = glm::vec4(target.allocated_points[i].color, 1.0f);
          }

          for (int i = 0; i < target.predicted_branches.size(); i++) {
            predicted_branch_starts[i] = target.predicted_branches[i].bezier_curve.p0;
            predicted_branch_ends[i] = target.predicted_branches[i].bezier_curve.p3;
            predicted_branch_colors[i] = glm::vec4(target.predicted_branches[i].color, 1.0f);
            if (use_real_branch_width)
              predicted_branch_widths[i] = target.predicted_branches[i].final_thickness;
            else
              predicted_branch_widths[i] = predicted_branch_width;
          }
          selected_branch_info_list->ApplyConnections(predicted_branch_starts, predicted_branch_ends,
                                                      predicted_branch_colors, predicted_branch_widths);
        } break;
        case 1: {
          // Branch
          for (int i = 0; i < target.allocated_points.size(); i++) {
            allocated_point_matrices[i].instance_matrix.value =
                glm::translate(target.allocated_points[i].position) * glm::scale(glm::vec3(0.003f));
            if (target.allocated_points[i].branch_handle >= 0) {
              allocated_point_matrices[i].instance_color =
                  glm::vec4(eco_sys_lab_layer->RandomColors()[target.allocated_points[i].branch_handle], 1.0f);
            } else {
              allocated_point_matrices[i].instance_color =
                  glm::vec4(eco_sys_lab_layer->RandomColors()[target.allocated_points[i].tree_part_handle], 1.0f);
            }
          }

          for (int i = 0; i < target.predicted_branches.size(); i++) {
            predicted_branch_starts[i] = target.predicted_branches[i].bezier_curve.p0;
            predicted_branch_ends[i] = target.predicted_branches[i].bezier_curve.p3;
            predicted_branch_colors[i] =
                glm::vec4(eco_sys_lab_layer->RandomColors()[target.predicted_branches[i].handle], 1.0f);
            if (use_real_branch_width)
              predicted_branch_widths[i] = target.predicted_branches[i].final_thickness;
            else
              predicted_branch_widths[i] = predicted_branch_width;
          }
          selected_branch_info_list->ApplyConnections(predicted_branch_starts, predicted_branch_ends,
                                                      predicted_branch_colors, predicted_branch_widths);
        } break;
        case 2: {
          // Node
          for (int i = 0; i < target.allocated_points.size(); i++) {
            allocated_point_matrices[i].instance_matrix.value =
                glm::translate(target.allocated_points[i].position) * glm::scale(glm::vec3(0.003f));
            if (target.allocated_points[i].node_handle >= 0) {
              allocated_point_matrices[i].instance_color =
                  glm::vec4(eco_sys_lab_layer->RandomColors()[target.allocated_points[i].node_handle], 1.0f);
            } else {
              allocated_point_matrices[i].instance_color =
                  glm::vec4(eco_sys_lab_layer->RandomColors()[target.allocated_points[i].tree_part_handle], 1.0f);
            }
          }

          for (int i = 0; i < target.predicted_branches.size(); i++) {
            predicted_branch_starts[i] = target.predicted_branches[i].bezier_curve.p0;
            predicted_branch_ends[i] = target.predicted_branches[i].bezier_curve.p3;
            predicted_branch_colors[i] = glm::vec4(1.0f);
            if (use_real_branch_width)
              predicted_branch_widths[i] = target.predicted_branches[i].final_thickness;
            else
              predicted_branch_widths[i] = predicted_branch_width;
          }
          selected_branch_info_list->ApplyConnections(predicted_branch_starts, predicted_branch_ends,
                                                      predicted_branch_colors, predicted_branch_widths);
        } break;
      }

      scatter_point_matrices.resize(target.scattered_points.size());
      for (int i = 0; i < target.scattered_points.size(); i++) {
        scatter_point_matrices[i].instance_matrix.value =
            glm::translate(target.scattered_points[i].position) * glm::scale(glm::vec3(0.004f));
        scatter_point_matrices[i].instance_color = scatter_point_color;
      }

      scattered_point_connections_starts.resize(target.scattered_points_connections.size());
      scattered_point_connections_ends.resize(target.scattered_points_connections.size());
      scattered_point_connection_colors.resize(target.scattered_points_connections.size());
      for (int i = 0; i < target.scattered_points_connections.size(); i++) {
        scattered_point_connections_starts[i] = target.scattered_points_connections[i].first;
        scattered_point_connections_ends[i] = target.scattered_points_connections[i].second;
        scattered_point_connection_colors[i] = scatter_point_to_branch_connection_color;
      }
      scattered_point_connection_info_list->ApplyConnections(scattered_point_connections_starts,
                                                             scattered_point_connections_ends,
                                                             scattered_point_connection_colors, connection_width);

      candidate_branch_connection_starts.resize(target.candidate_branch_connections.size());
      candidate_branch_connection_ends.resize(target.candidate_branch_connections.size());
      candidate_branch_connection_colors.resize(target.candidate_branch_connections.size());
      for (int i = 0; i < target.candidate_branch_connections.size(); i++) {
        candidate_branch_connection_starts[i] = target.candidate_branch_connections[i].first;
        candidate_branch_connection_ends[i] = target.candidate_branch_connections[i].second;
        candidate_branch_connection_colors[i] = candidate_branch_connection_color;
      }

      candidate_branch_connection_info_list->ApplyConnections(candidate_branch_connection_starts,
                                                              candidate_branch_connection_ends,
                                                              candidate_branch_connection_colors, connection_width);

      reversed_candidate_branch_connection_starts.resize(target.reversed_candidate_branch_connections.size());
      reversed_candidate_branch_connection_ends.resize(target.reversed_candidate_branch_connections.size());
      reversed_candidate_branch_connection_colors.resize(target.reversed_candidate_branch_connections.size());
      for (int i = 0; i < target.reversed_candidate_branch_connections.size(); i++) {
        reversed_candidate_branch_connection_starts[i] = target.reversed_candidate_branch_connections[i].first;
        reversed_candidate_branch_connection_ends[i] = target.reversed_candidate_branch_connections[i].second;
        reversed_candidate_branch_connection_colors[i] = reversed_candidate_branch_connection_color;
      }

      reversed_candidate_branch_connection_info_list->ApplyConnections(
          reversed_candidate_branch_connection_starts, reversed_candidate_branch_connection_ends,
          reversed_candidate_branch_connection_colors, connection_width);

      filtered_branch_connection_starts.resize(target.filtered_branch_connections.size());
      filtered_branch_connection_ends.resize(target.filtered_branch_connections.size());
      filtered_branch_connection_colors.resize(target.filtered_branch_connections.size());
      for (int i = 0; i < target.filtered_branch_connections.size(); i++) {
        filtered_branch_connection_starts[i] = target.filtered_branch_connections[i].first;
        filtered_branch_connection_ends[i] = target.filtered_branch_connections[i].second;
        filtered_branch_connection_colors[i] = filtered_branch_connection_color;
      }
      filtered_branch_connection_info_list->ApplyConnections(
          filtered_branch_connection_starts, filtered_branch_connection_ends, filtered_branch_connection_colors,
          connection_width * 1.1f);

      selected_branch_connection_starts.resize(target.branch_connections.size());
      selected_branch_connection_ends.resize(target.branch_connections.size());
      selected_branch_connection_colors.resize(target.branch_connections.size());
      for (int i = 0; i < target.branch_connections.size(); i++) {
        selected_branch_connection_starts[i] = target.branch_connections[i].first;
        selected_branch_connection_ends[i] = target.branch_connections[i].second;
        selected_branch_connection_colors[i] = selected_branch_connection_color;
      }
      selected_branch_connection_info_list->ApplyConnections(
          selected_branch_connection_starts, selected_branch_connection_ends, selected_branch_connection_colors,
          connection_width * 1.2f);

      scatter_point_to_branch_connection_starts.resize(target.scattered_point_to_branch_start_connections.size() +
                                                       target.scattered_point_to_branch_end_connections.size());
      scatter_point_to_branch_connection_ends.resize(target.scattered_point_to_branch_start_connections.size() +
                                                     target.scattered_point_to_branch_end_connections.size());
      scatter_point_to_branch_connection_colors.resize(target.scattered_point_to_branch_start_connections.size() +
                                                       target.scattered_point_to_branch_end_connections.size());
      for (int i = 0; i < target.scattered_point_to_branch_start_connections.size(); i++) {
        scatter_point_to_branch_connection_starts[i] = target.scattered_point_to_branch_start_connections[i].first;
        scatter_point_to_branch_connection_ends[i] = target.scattered_point_to_branch_start_connections[i].second;
        scatter_point_to_branch_connection_colors[i] = scatter_point_to_branch_connection_color;
      }
      for (int i = target.scattered_point_to_branch_start_connections.size();
           i < target.scattered_point_to_branch_start_connections.size() +
                   target.scattered_point_to_branch_end_connections.size();
           i++) {
        scatter_point_to_branch_connection_starts[i] =
            target
                .scattered_point_to_branch_end_connections[i -
                                                           target.scattered_point_to_branch_start_connections.size()]
                .first;
        scatter_point_to_branch_connection_ends[i] =
            target
                .scattered_point_to_branch_end_connections[i -
                                                           target.scattered_point_to_branch_start_connections.size()]
                .second;
      }
      scatter_point_to_branch_connection_info_list->ApplyConnections(
          scatter_point_to_branch_connection_starts, scatter_point_to_branch_connection_ends,
          scatter_point_to_branch_connection_colors, connection_width);

      allocated_point_info_list->SetParticleInfos(allocated_point_matrices);
      scattered_point_info_list->SetParticleInfos(scatter_point_matrices);
    }
    if (debug_scattered_points) {
      editor_layer->DrawGizmoMeshInstancedColored(Resources::GetInstance().GetPrimitives().cube,
                                                  scattered_point_info_list, glm::mat4(1.0f), point_size,
                                                  gizmo_settings);
    }
    if (debug_allocated_points) {
      editor_layer->DrawGizmoMeshInstancedColored(Resources::GetInstance().GetPrimitives().cube,
                                                  allocated_point_info_list, glm::mat4(1.0f), point_size,
                                                  gizmo_settings);
    }
    if (debug_selected_branches)
      editor_layer->DrawGizmoMeshInstancedColored(Resources::GetInstance().GetPrimitives().cone,
                                                  selected_branch_info_list, glm::mat4(1.0f), 1.0f, gizmo_settings);
    if (debug_scattered_point_connections)
      editor_layer->DrawGizmoMeshInstancedColored(Resources::GetInstance().GetPrimitives().cylinder,
                                                  scattered_point_connection_info_list, glm::mat4(1.0f), 1.0f,
                                                  gizmo_settings);

    if (debug_candidate_connections)
      editor_layer->DrawGizmoMeshInstancedColored(Resources::GetInstance().GetPrimitives().cone,
                                                  candidate_branch_connection_info_list, glm::mat4(1.0f), 1.0f,
                                                  gizmo_settings);

    if (debug_reversed_candidate_connections)
      editor_layer->DrawGizmoMeshInstancedColored(Resources::GetInstance().GetPrimitives().cylinder,
                                                  reversed_candidate_branch_connection_info_list, glm::mat4(1.0f), 1.0f,
                                                  gizmo_settings);

    if (debug_filtered_connections)
      editor_layer->DrawGizmoMeshInstancedColored(Resources::GetInstance().GetPrimitives().cylinder,
                                                  filtered_branch_connection_info_list, glm::mat4(1.0f), 1.0f,
                                                  gizmo_settings);
    if (debug_selected_branch_connections)
      editor_layer->DrawGizmoMeshInstancedColored(Resources::GetInstance().GetPrimitives().cylinder,
                                                  selected_branch_connection_info_list, glm::mat4(1.0f), 1.0f,
                                                  gizmo_settings);

    if (debug_scatter_point_to_branch_connections)
      editor_layer->DrawGizmoMeshInstancedColored(Resources::GetInstance().GetPrimitives().cylinder,
                                                  scatter_point_to_branch_connection_info_list, glm::mat4(1.0f), 1.0f,
                                                  gizmo_settings);
  }

  if (ImGui::TreeNode("Info settings")) {
    ImGui::Checkbox("Allocated points", &enable_allocated_points);
    ImGui::Checkbox("Scattered points", &enable_scattered_points);
    ImGui::Checkbox("Scatter-Branch connections", &enable_scatter_point_to_branch_connections);
    ImGui::Checkbox("Candidate connections", &enable_candidate_branch_connections);
    ImGui::Checkbox("Filtered branch connections", &enable_filtered_branch_connections);
    ImGui::Checkbox("Selected branch connections", &enable_selected_branch_connections);
    ImGui::Checkbox("Selected branches", &enable_selected_branches);
    ImGui::TreePop();
  }
  if (ImGui::Button("Build Info")) {
    FormInfoEntities(target);
  }

  return changed;
}
