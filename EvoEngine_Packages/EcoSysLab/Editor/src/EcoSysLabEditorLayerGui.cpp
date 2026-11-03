#include "EcoSysLabEditorLayer.hpp"

#include "BranchRenderer.hpp"
#include "DynamicStrandsMeshingInspector.hpp"
#include "DynamicTreeStrands.hpp"
#include "DynamicsSettingsEditor.hpp"
#include "EcoSysLabSettingsEditor.hpp"
#include "EcoSysLabSystem.hpp"
#include "EditorFileDialogs.hpp"
#include "EditorTextureRegistry.hpp"
#include "FoliageRenderer.hpp"
#include "Soil.hpp"
#include "SoilDescriptor.hpp"

using namespace eco_sys_lab_package;

void EcoSysLabEditorLayer::DrawGui(const std::shared_ptr<EditorLayer>& editor_layer) {
  const auto runtime = ApplicationContext::Get().GetLayer<EcoSysLabLayer>();
  if (!runtime)
    return;

  auto scene = GetScene();
  if (!scene || !visualization_camera_) {
    return;
  }
  bool simulate = false;
  const auto growth = scene->GetOrCreateSystem<EcoSysLabSystem>(0.0f);
  const auto status = runtime->GetApplication().GetApplicationStatus();
  const bool runtime_scene = status == Application::ExecutionStatus::Playing ||
                             status == Application::ExecutionStatus::Pause ||
                             status == Application::ExecutionStatus::Step;

  visualization_camera_->Resize({visualization_camera_resolution_x, visualization_camera_resolution_y});

  const auto window_title = runtime->GetLayerName();
  bool open = runtime->enable_inspection;
  if (ImGui::Begin(window_title.c_str(), &open)) {
    const auto* tree_entities = scene->UnsafeGetPrivateComponentOwnersList<Tree>();
    const bool has_trees = tree_entities && !tree_entities->empty();
    ImGui::Text("Trees: %zu", has_trees ? tree_entities->size() : 0);
    ImGui::SameLine();
    ImGui::Text("Simulated time: %.2f years", growth->growth.simulated_time / 365.f);
    if (ImGui::BeginTabBar("EcoSysLab sections")) {
      if (ImGui::BeginTabItem("Growth")) {
        if (has_trees) {
          ImGui::DragFloat("Years to grow", &extra_time, 0.1f, 0.0f, 999.0f);
          if (auto_time_grow) {
            if (ImGui::Button("Stop growth")) {
              auto_time_grow = false;
              growth->growth.Stop();
              target_time = growth->growth.simulated_time;
            }
          } else if (ImGui::Button("Grow")) {
            if (runtime_scene) {
              growth->growth.Queue(extra_time * 365.f);
              auto_time_grow = growth->growth.remaining_time > 0.0f;
              target_time = growth->growth.simulated_time + growth->growth.remaining_time;
            }
          }
          ImGui::SameLine();
          if (ImGui::Button("Grow 1 iteration") && runtime_scene)
            simulate = true;
          if (!runtime_scene)
            ImGui::TextDisabled("Enter Play to grow trees.");
          if (ImGui::CollapsingHeader("Simulation settings"))
            InspectSettings(growth->growth.settings, editor_layer);
          if (ImGui::CollapsingHeader("Statistics"))
            InspectSettings(growth->growth.stats, editor_layer);
          ImGui::Separator();
          if (ImGui::Button("Reset all trees")) {
            runtime->ResetAllTrees(tree_entities);
            runtime->ClearMeshes();
            runtime->ClearGroundFruitAndLeaf();
            target_time = 0.0f;
          }
          if (!growth->growth.settings.auto_clear_fruit_and_leaves && ImGui::Button("Clear ground leaves and fruits"))
            runtime->ClearGroundFruitAndLeaf();
        } else {
          ImGui::TextDisabled("No trees in the scene.");
        }
        ImGui::EndTabItem();
      }
      if (ImGui::BeginTabItem("Wind & air")) {
        ImGui::Checkbox("Show wind particles", &runtime->visualize_wind_particles);
        ImGui::Checkbox("Show air volume", &runtime->visualize_air_grid);
        if (runtime->visualize_air_grid) {
          ImGui::TextDisabled("Blue = cool, orange = warm; opacity = wind speed");
          ImGui::Checkbox("Show registered coverage", &runtime->visualize_air_coverage);
          ImGui::TextDisabled("Green = registered still air");
        }
        if (ImGui::Button("Reset air grid"))
          runtime->ResetAirGrid();
        ImGui::EndTabItem();
      }
      if (ImGui::BeginTabItem("Physics")) {
        auto& dynamics = growth->dynamics;
        ImGui::Checkbox("Subtree physics LOD", &dynamics.use_subtree_clusters);
        if (dynamics.use_subtree_clusters) {
          auto& settings = dynamics.cluster_lod;
          ImGui::TextWrapped(
              "Selected subtrees move as rigid units. Changes preserve the current branch shape. Rules apply to every "
              "tree; larger eligible subtrees take priority.");
          constexpr const char* labels[] = {"Size", "Branch order", "Size and branch order"};
          for (uint32_t policy = 0; policy < 3; ++policy) {
            ImGui::PushID(static_cast<int>(policy));
            ImGui::CheckboxFlags(labels[policy], &settings.enabled_policies, 1u << policy);
            if (settings.enabled_policies & (1u << policy)) {
              auto& rule = settings.rules[policy];
              if (policy != 1) {
                ImGui::DragFloat("Maximum extent / tree height", &rule.x, 0.01f, 0.001f, 2.0f, "%.3f",
                                 ImGuiSliderFlags_AlwaysClamp);
                ImGui::DragFloat("Maximum attachment / trunk radius", &rule.y, 0.01f, 0.001f, 1.0f, "%.3f",
                                 ImGuiSliderFlags_AlwaysClamp);
              }
              if (policy != 0) {
                int order = static_cast<int>(rule.z);
                if (ImGui::DragInt("Minimum branch order", &order, 1, 0, 64, "%d", ImGuiSliderFlags_AlwaysClamp))
                  rule.z = static_cast<float>(order);
              }
              int count = static_cast<int>(rule.w);
              if (ImGui::DragInt("Minimum internodes", &count, 1, 2, 4096, "%d", ImGuiSliderFlags_AlwaysClamp))
                rule.w = static_cast<float>(count);
            }
            ImGui::PopID();
          }
          ImGui::TextDisabled("No enabled rules: individual rigid internodes. Roots remain fixed.");
        } else {
          ImGui::DragFloat("Approximation tolerance (radius fraction)", &dynamics.physics_approximation_tolerance,
                           0.01f, 0.0f, 2.0f, "%.3f", ImGuiSliderFlags_AlwaysClamp);
          ImGui::TextWrapped(
              "Applies to all trees. Zero uses the original skeleton. Physical properties belong to each tree.");
        }
        ImGui::EndTabItem();
      }
      if (ImGui::BeginTabItem("Environment")) {
        auto& environment = growth->environment;
        if (editor_layer->DragAndDropButton<ClimateDescriptor>(environment.climate_descriptor, "Climate", true) |
            editor_layer->DragAndDropButton<SoilDescriptor>(environment.soil_descriptor, "Soil", true))
          environment.initialized = false;
        ImGui::DragFloat("Ambient temperature (C)", &environment.air_temperature_c, 0.5f, -50.0f, 100.0f);
        ImGui::SliderFloat("Ambient relative humidity", &environment.air_relative_humidity, 0.0f, 1.0f);
        ImGui::DragFloat("Ambient pressure (Pa)", &environment.air_pressure_pa, 100.0f, 10000.0f, 120000.0f);
        ImGui::DragFloat("Air coupling (1/s)", &environment.air_coupling_rate, 0.1f, 0.0f, 100.0f);
        ImGui::DragFloat("Air friction (1/s)", &environment.air_friction, 0.1f, 0.0f, 100.0f);
        float cell_size = environment.air_cell_size;
        if (ImGui::InputFloat("Grid cell size (m)", &cell_size, 0.0f, 0.0f, "%.3f",
                              ImGuiInputTextFlags_EnterReturnsTrue))
          environment.air_cell_size = glm::clamp(cell_size, 0.05f, 0.2f);
        ImGui::TextDisabled("0.05-0.2 m; press Enter to rebuild");
        if (ImGui::CollapsingHeader("Soil model and geometry")) {
          environment.InitializeModels();
          if (ImGui::Button("Reinitialize soil")) {
            environment.initialized = false;
            environment.InitializeModels();
          }
          ImGui::SameLine();
          if (ImGui::Button("Reset soil"))
            environment.soil.Reset();
          if (ImGui::Button("Generate surface mesh")) {
            eco_sys_lab_package::SoilVisualization{environment.soil, environment.soil_descriptor,
                                                   environment.GetOrCreateVisualizationOwner(scene)}
                .GenerateMesh();
          }
          ImGui::DragFloat("Cutout X depth", &soil_cutout_x_, 0.01f, 0.0f, 1.0f);
          ImGui::DragFloat("Cutout Z depth", &soil_cutout_z_, 0.01f, 0.0f, 1.0f);
          ImGui::DragFloat("Water factor", &soil_water_factor_, 0.0001f, 0.0f, 1.0f);
          ImGui::DragFloat("Nutrient factor", &soil_nutrient_factor_, 0.0001f, 0.0f, 1.0f);
          ImGui::Checkbox("Ground surface", &soil_ground_surface_);
          auto generate = [&](const char* name, auto action) {
            const auto owner = environment.GetOrCreateVisualizationOwner(scene);
            for (const auto child : scene->GetChildren(owner))
              if (scene->GetEntityName(child) == name) {
                scene->DeleteEntity(child);
                break;
              }
            eco_sys_lab_package::SoilVisualization visualization{environment.soil, environment.soil_descriptor, owner};
            auto result = action(visualization);
            if (scene->IsEntityValid(result))
              scene->SetParent(result, owner);
          };
          if (ImGui::Button("Generate cutout"))
            generate("CutOut", [&](eco_sys_lab_package::SoilVisualization& visualization) {
              return visualization.GenerateCutOut(soil_cutout_x_, soil_cutout_z_, soil_water_factor_,
                                                  soil_nutrient_factor_, soil_ground_surface_);
            });
          ImGui::SameLine();
          if (ImGui::Button("Generate cube"))
            generate("Cube", [&](eco_sys_lab_package::SoilVisualization& visualization) {
              return visualization.GenerateFullBox(soil_water_factor_, soil_nutrient_factor_, soil_ground_surface_);
            });
        }
        ImGui::EndTabItem();
      }
      if (ImGui::BeginTabItem("Visualization")) {
        if (ImGui::Checkbox("Show tree overlay", &tree_visualization_settings_.enable))
          need_full_flow_update = true;
        if (tree_visualization_settings_.enable) {
          if (has_trees && scene->IsEntityValid(selected_tree)) {
            const auto& tree = scene->GetOrSetPrivateComponent<Tree>(selected_tree).lock();
            auto& shoot_visualizer = GetTreeEditorState(*tree).shoot_visualizer;
            const bool can_edit = CanEditTree(*tree);
            if (!can_edit) {
              tree_operator_mode = static_cast<unsigned>(TreeOperatorMode::Select);
              ImGui::TextDisabled("Pause physics to rotate or prune this tree.");
            }
            if (shoot_visualizer.checkpoint_iteration == tree->shoot_model.CurrentIteration()) {
              if (ImGui::CollapsingHeader("Tree operator")) {
                ImGui::BeginDisabled(!can_edit);
                if (ImGui::Combo("Mode", {"None", "Select", "Rotate", "Prune", "Invigorate", "Reduce"},
                                 tree_operator_mode)) {
                  shoot_visualizer.selected_node_handle = -1;
                  shoot_visualizer.selected_node_hierarchy_list.clear();
                }
                ImGui::EndDisabled();
                switch (static_cast<TreeOperatorMode>(tree_operator_mode)) {
                  case TreeOperatorMode::Select:
                    if (can_edit)
                      ImGui::Text("Press T to cut off entire node, press R to cut at point of selection.");
                    break;
                  case TreeOperatorMode::Rotate:
                    ImGui::Text("Press T to cut off entire node.");
                    break;
                  default:
                    break;
                }
                if (tree_operator_mode == static_cast<unsigned>(TreeOperatorMode::Reduce))
                  ImGui::DragFloat("Reduce speed", &tree_reduce_rate, 0.001f, 0.001f, 1.0f);
              }
            } else {
              ImGui::TextDisabled("Go to current skeleton to enable operator.");
            }
            if (ImGui::CollapsingHeader("Tree visualizer") && shoot_visualizer.DrawGui(tree->shoot_model, can_edit))
              tree->NotifyShootModelChanged();
            auto& root_visualizer = GetTreeEditorState(*tree).root_visualizer;
            if (ImGui::CollapsingHeader("Root visualizer") && root_visualizer.DrawGui(tree->root_model, can_edit))
              tree->NotifyRootModelChanged();
          } else {
            ImGui::TextDisabled("Select a tree entity to edit its overlay.");
          }
          if (ImGui::CollapsingHeader("Tree overlay settings")) {
            if (ImGui::Button("Update"))
              need_full_flow_update = true;
            need_full_flow_update |= tree_visualization_settings_.DrawGui(editor_layer);
          }
        }
        if (ImGui::CollapsingHeader("Soil visualization"))
          soil_visualization_settings_.DrawGui(editor_layer);
        ImGui::EndTabItem();
      }
      if (ImGui::BeginTabItem("Geometry")) {
        if (has_trees) {
          if (ImGui::CollapsingHeader("Skeletal graphs")) {
            InspectSettings(runtime->skeletal_graph_settings, editor_layer);
            if (ImGui::Button("Generate skeletal graphs"))
              runtime->GenerateSkeletalGraphs(runtime->skeletal_graph_settings);
            ImGui::SameLine();
            if (ImGui::Button("Clear skeletal graphs"))
              runtime->ClearSkeletalGraphs();
          }
          if (ImGui::CollapsingHeader("Meshes")) {
            InspectSettings(runtime->mesh_generator_settings, editor_layer);
            if (ImGui::Button("Generate meshes"))
              runtime->GenerateMeshes(runtime->mesh_generator_settings);
            ImGui::SameLine();
            if (ImGui::Button("Clear meshes"))
              runtime->ClearMeshes();
          }
          if (ImGui::CollapsingHeader("Strand model meshes")) {
            InspectSettings(runtime->strand_mesh_generator_settings, editor_layer);
            if (ImGui::Button("Build strand renderer"))
              runtime->GenerateStrandRenderers();
            ImGui::SameLine();
            if (ImGui::Button("Clear strand renderer"))
              runtime->ClearStrandRenderers();
            if (ImGui::Button("Generate strand model meshes"))
              runtime->GenerateStrandModelMeshes(runtime->strand_mesh_generator_settings);
            ImGui::SameLine();
            if (ImGui::Button("Clear strand model meshes"))
              runtime->ClearStrandModelMeshes();
          }
          if (ImGui::CollapsingHeader("Automatic geometry generation")) {
            ImGui::Checkbox("Auto generate mesh", &auto_generate_mesh_after_editing_);
            ImGui::Checkbox("Auto generate skeletal graph per frame", &auto_generate_skeletal_graph_every_frame_);
            ImGui::Checkbox("Auto generate strands", &auto_generate_strands_after_editing_);
            ImGui::Checkbox("Auto generate strands mesh", &auto_generate_strand_mesh_after_editing_);
          }
          EditorFileDialogs::SaveFile(
              "Export all trees as OBJ", "OBJ", {".obj"},
              [&](const std::filesystem::path& path) {
                runtime->ExportAllTrees(path);
              },
              false);
        } else {
          ImGui::TextDisabled("No trees in the scene.");
        }
        ImGui::EndTabItem();
      }
      if (ImGui::BeginTabItem("Strands (advanced)")) {
        if (ImGui::Button("Initialize all"))
          runtime->GenerateDynamicStrandsForAllTrees();
        ImGui::SameLine();
        if (ImGui::Button("Refresh meshes"))
          runtime->RefreshMeshForAllDynamicStrands();
        if (const auto* dts_entities = scene->UnsafeGetPrivateComponentOwnersList<DynamicTreeStrands>();
            dts_entities && !dts_entities->empty()) {
          DrawDynamicStrandsSettingsGui(editor_layer);
        } else {
          ImGui::TextDisabled("No dynamic strands in the scene.");
        }
        if (ImGui::CollapsingHeader("Fungus growth"))
          ImGui::DragFloat("Fungus growth rate",
                           &runtime->GetDynamicStrandsSettings().physics_parameters.fungus_growth_rate, 0.01f, 0.0f,
                           1.0f);
        ImGui::EndTabItem();
      }
      ImGui::EndTabBar();
    }
  }
  ImGui::End();
  runtime->enable_inspection = open;

  if (simulate) {
    runtime->Simulate();
  }
  if (const std::vector<Entity>* tree_entities = scene->UnsafeGetPrivateComponentOwnersList<Tree>();
      tree_entities && !tree_entities->empty()) {
    if (growth->growth.remaining_time <= 0.0f && auto_time_grow) {
      auto_time_grow = false;
      for (const auto& tree_entity : *tree_entities) {
        auto tree = scene->GetOrSetPrivateComponent<Tree>(tree_entity).lock();
        if (auto_generate_mesh_after_editing_ && !scene->HasPrivateComponent<BranchRenderer>(tree_entity) &&
            !scene->HasPrivateComponent<FoliageRenderer>(tree_entity)) {
          tree->GenerateGeometryEntities(runtime->mesh_generator_settings, -1);
        }
        if (auto_generate_strands_after_editing_ || auto_generate_strand_mesh_after_editing_) {
          tree->BuildStrandModel();
          if (auto_generate_strands_after_editing_) {
            auto strands = tree->GenerateStrands();
            tree->InitializeStrandRenderer(strands);
          }
          if (auto_generate_strand_mesh_after_editing_) {
            tree->InitializeStrandModelMeshRenderer(runtime->strand_mesh_generator_settings);
          }
        }
      }
    }
  }
#pragma region Internode debugging camera
  ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2{0, 0});
  visualization_camera_window_focused_ = false;
  visualization_camera_mouse_position = glm::vec2(FLT_MAX, -FLT_MAX);
  const bool plant_visual_visible = ImGui::Begin("Plant Visual");
  if (plant_visual_visible) {
    if (ImGui::BeginChild("InternodeCameraRenderer", ImVec2(0, 0), false)) {
      ImVec2 view_port_size;
      view_port_size = ImGui::GetWindowSize();
      const ImVec2 overlay_pos = ImGui::GetWindowPos();

      visualization_camera_resolution_x = view_port_size.x;
      visualization_camera_resolution_y = view_port_size.y;
      ImGui::Image(EditorTextureRegistry::GetColorTextureId(*visualization_camera_->GetRenderTexture()),
                   ImVec2(view_port_size.x, view_port_size.y), ImVec2(0, 1), ImVec2(1, 0));

      VisualizationCameraDragAndDrop();
      const auto window_pos = ImVec2((corner & 1) ? (overlay_pos.x + view_port_size.x) : (overlay_pos.x),
                                     (corner & 2) ? (overlay_pos.y + view_port_size.y) : (overlay_pos.y));
      if (show_visualization_camera_info) {
        const auto window_pos_pivot = ImVec2((corner & 1) ? 1.0f : 0.0f, (corner & 2) ? 1.0f : 0.0f);
        ImGui::SetNextWindowPos(window_pos, ImGuiCond_Always, window_pos_pivot);
        ImGui::SetNextWindowBgAlpha(0.35f);
        constexpr ImGuiWindowFlags window_flags = ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoDocking |
                                                  ImGuiWindowFlags_NoSavedSettings |
                                                  ImGuiWindowFlags_NoFocusOnAppearing;
        if (constexpr ImGuiChildFlags child_flags = ImGuiChildFlags_None;
            ImGui::BeginChild("Render Info", ImVec2(300, 150), child_flags, window_flags)) {
          ImGui::Text("Info & Settings");
          ImGui::Text("%.1f FPS", ImGui::GetIO().Framerate);
          ImGui::Checkbox("Background", &enable_visualization_background);
          uint32_t mode = static_cast<uint32_t>(visualization_camera_->camera_render_mode);
          if (ImGui::Combo("Render Mode", Camera::GetCameraRenderModeNames(), mode)) {
            visualization_camera_->camera_render_mode = static_cast<Camera::CameraRenderMode>(mode);
            visualization_camera_->ResetFrameCount();
          }
        }
        ImGui::EndChild();
      }
      visualization_camera_mouse_position = glm::vec2(FLT_MAX, -FLT_MAX);
      if (ImGui::IsWindowFocused()) {
        visualization_camera_window_focused_ = true;
        auto mp = ImGui::GetMousePos();
        auto wp = ImGui::GetWindowPos();
        visualization_camera_mouse_position = glm::vec2(mp.x - wp.x, mp.y - wp.y);
      } else {
        visualization_camera_window_focused_ = false;
      }
      if (const auto scene_camera = editor_layer->GetSceneCamera()) {
        editor_layer->ApplyEditorCameraFreeFlyControl(
            scene_camera->GetHandle(), visualization_camera_free_fly_state_, visualization_camera_mouse_position,
            {view_port_size.x, view_port_size.y}, visualization_camera_window_focused_);
        editor_layer->RefEditorCameraRotation(visualization_camera_->GetHandle()) =
            editor_layer->GetSceneCameraRotation();
        editor_layer->RefEditorCameraPosition(visualization_camera_->GetHandle()) =
            editor_layer->GetSceneCameraPosition();
      }
    }
    ImGui::EndChild();
    auto* window = ImGui::FindWindowByName("Plant Visual");
    if (!(window->Hidden && !window->Collapsed)) {
      if (enable_visualization_background) {
        visualization_camera_->SetRequireRendering(true);
      } else {
        visualization_camera_->SetRendered();
      }
    }
  }
  ImGui::End();
  ImGui::PopStyleVar();
  if (const auto selected_entity = editor_layer->GetSelectedEntity();
      plant_visual_visible && tree_visualization_settings_.enable && selected_entity != selected_tree) {
    if (scene->IsEntityValid(selected_entity) && scene->HasPrivateComponent<Tree>(selected_entity)) {
      selected_tree = selected_entity;
      last_selected_tree_index_ = selected_tree.GetIndex();
      need_flow_update_for_selection_ = true;
      tree_operator_mode = static_cast<unsigned>(TreeOperatorMode::Select);
    } else if (selected_tree.GetIndex() != 0) {
      selected_tree = Entity();
      need_flow_update_for_selection_ = true;
    }
    if (scene->IsEntityValid(selected_tree)) {
      const auto& tree = scene->GetOrSetPrivateComponent<Tree>(selected_tree).lock();
      auto& shoot_visualizer = GetTreeEditorState(*tree).shoot_visualizer;
      shoot_visualizer.selected_node_handle = -1;
      shoot_visualizer.selected_node_hierarchy_list.clear();
      tree_operator_mode = static_cast<unsigned>(TreeOperatorMode::Select);
    }
  }
  if (plant_visual_visible && tree_visualization_settings_.enable)
    TreeVisualization(editor_layer);
  if (dynamic_strands_settings_.enable_visualization)
    DynamicStrandsVisualization(editor_layer);
  if (soil_visualization_settings_.enable) {
    SoilVisualization();
  }
#pragma endregion
}

void EcoSysLabEditorLayer::DrawDynamicStrandsSettingsGui(const std::shared_ptr<EditorLayer>& editor_layer) {
  const auto runtime = ApplicationContext::Get().GetLayer<EcoSysLabLayer>();
  if (!runtime)
    return;

  if (ImGui::TreeNode("Operators")) {
    ImGui::Combo("Transform Mode", {"None", "Translate", "Rotate"}, dynamic_strands_settings_.transform_mode);
    ImGui::Combo("Operator Mode", {"Drag", "Saw", "Line Cut", "Point Cut", "Fungus Injection"},
                 dynamic_strands_settings_.operator_mode);
    switch (static_cast<DynamicStrandsSettings::OperatorMode>(dynamic_strands_settings_.operator_mode)) {
      case DynamicStrandsSettings::OperatorMode::Drag: {
        ImGui::DragFloat("Drag acceleration multiplier", &dynamic_strands_settings_.drag_multiplier, 0.001f, 0.0f,
                         1.0f);
        break;
      }
      case DynamicStrandsSettings::OperatorMode::Saw:
      case DynamicStrandsSettings::OperatorMode::LineCut: {
        ImGui::Checkbox("Cut Bend/Twist/Bundle only", &dynamic_strands_settings_.cut_bend_twist_bundle_only);
        break;
      }
      case DynamicStrandsSettings::OperatorMode::PointCut: {
        ImGui::DragFloat("Cutter thickness", &dynamic_strands_settings_.point_cut_thickness, 1.f, 1.0f, 100.0f);
        break;
      }
      case DynamicStrandsSettings::OperatorMode::FungusInjection: {
        ImGui::DragFloat("Injection thickness", &dynamic_strands_settings_.point_cut_thickness, 1.f, 1.0f, 100.0f);
        ImGui::DragFloat("Fungus injection amount", &dynamic_strands_settings_.fungus_injection_amount, 0.1f, 0.0f,
                         100.0f);
        // Check boxes for each rot type
        ImGui::Text("Rot types:");
        ImGui::Checkbox("White rot", &dynamic_strands_settings_.fungus_white_rot);
        ImGui::Checkbox("Brown rot", &dynamic_strands_settings_.fungus_brown_rot);
        break;
      }
    }
    ImGui::TreePop();
  }

  ImGui::Checkbox("Physics", &runtime->GetDynamicStrandsSettings().enable_physics);
  if (!runtime->GetDynamicStrandsSettings().enable_physics && ImGui::Button("Physics step")) {
    runtime->GetDynamicStrandsSettings().remaining_step++;
  }
  if (ImGui::TreeNode("Physics settings")) {
    if (InspectSettings(runtime->GetDynamicStrandsSettings().physics_parameters, editor_layer)) {
      runtime->GetDynamicStrandsSettings().fungus_parameters =
          static_cast<const DynamicStrands::FungusParameters&>(runtime->GetDynamicStrandsSettings().physics_parameters);
    }
    ImGui::TreePop();
  }

  ImGui::Checkbox("Fungus updates", &runtime->GetDynamicStrandsSettings().enable_fungus);
  if (!runtime->GetDynamicStrandsSettings().enable_fungus && ImGui::Button("Fungus step")) {
    runtime->GetDynamicStrandsSettings().remaining_fungus_step++;
  }
  ImGui::DragInt("Fungus steps per frame", &runtime->GetDynamicStrandsSettings().fungus_sub_step, 1, 1, 100);
  ImGui::Checkbox("Geometry updates", &runtime->GetDynamicStrandsSettings().enable_geometry_updates);
  if (!runtime->GetDynamicStrandsSettings().enable_geometry_updates && ImGui::Button("Geometry step")) {
    runtime->GetDynamicStrandsSettings().remaining_geometry_step++;
  }
  ImGui::Checkbox("Rendering", &runtime->GetDynamicStrandsSettings().enable_rendering);
  if (ImGui::TreeNode("Rendering settings")) {
    if (ImGui::TreeNode("Alpha Shape Meshing Settings")) {
      DynamicStrandsMeshingInspector::DrawDsAlphaShapeMeshingSettings(editor_layer);
      ImGui::TreePop();
    }
    if (ImGui::TreeNode("Kinetic Voronoi Meshing Settings")) {
      DynamicStrandsMeshingInspector::DrawDsKineticVoronoiMeshingSettings(editor_layer);
      ImGui::TreePop();
    }

    ImGui::Checkbox("Render foliage", &runtime->GetDynamicStrandsSettings().foliage_render_parameters.enabled);
    if (runtime->GetDynamicStrandsSettings().foliage_render_parameters.enabled) {
      if (ImGui::TreeNodeEx("Foliage render settings")) {
        if (ImGui::Button("Rebuild foliage pipelines")) {
          DynamicStrands::BuildFoliageRenderingPipelines();
        }
        InspectSettings(runtime->GetDynamicStrandsSettings().foliage_render_parameters, editor_layer);
        ImGui::TreePop();
      }
    }

    ImGui::Checkbox("Render segment pairs",
                    &runtime->GetDynamicStrandsSettings().segment_pairs_render_parameters.enabled);
    if (runtime->GetDynamicStrandsSettings().segment_pairs_render_parameters.enabled) {
      if (ImGui::TreeNodeEx("Segment pairs render settings")) {
        if (ImGui::Button("Rebuild segment pairs pipelines")) {
          DynamicStrands::BuildSegmentPairsRenderingPipeline();
        }
        InspectSettings(runtime->GetDynamicStrandsSettings().segment_pairs_render_parameters, editor_layer);
        ImGui::TreePop();
      }
    }

    ImGui::TreePop();
  }

  ImGui::Checkbox("Visualization", &dynamic_strands_settings_.enable_visualization);
  if (ImGui::TreeNode("Visualization settings")) {
    InspectSettings(dynamic_strands_settings_.visualization_parameters, editor_layer);
    ImGui::TreePop();
  }
}
