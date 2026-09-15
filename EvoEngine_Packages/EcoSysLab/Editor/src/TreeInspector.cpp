#include <Material.hpp>
#include <Mesh.hpp>
#include <TransformGraph.hpp>
#include "Application.hpp"
#include "BasicShootDescriptor.hpp"
#include "BillboardCloudSettingsEditor.hpp"
#include "Climate.hpp"
#include "EcoSysLabEditorLayer.hpp"
#include "EcoSysLabLayer.hpp"
#include "EcoSysLabSerializationAdapters.hpp"
#include "EcoSysLabSettingsEditor.hpp"
#include "EditorFileDialogs.hpp"
#include "EditorLayer.hpp"
#include "Octree.hpp"
#include "SkeletonSerializer.hpp"
#include "Soil.hpp"
#include "StrandGroupSerializer.hpp"
#include "StrandModelProfileSerializer.hpp"
#include "Tree.hpp"
#include "TreeEditorState.hpp"

using namespace evo_engine;
using namespace eco_sys_lab_package;
bool TreeEditorState::Inspect(InspectorContext& context, Tree& target) {
  const auto& editor_layer = context.editor_layer;
  bool changed = false;
  if (ImGui::TreeNode("Preset settings")) {
    if (ImGui::Button("Oak Trunk Crack Process")) {
      target.strand_model_parameters.end_node_strands = 3200;
      target.strand_model_parameters.strand_radius_distribution.mean.max_value = 0.003f;
      target.strand_model_parameters.strand_radius_distribution.mean.curve = Curve2D(1.0f, 0.8f, {0, 0}, {1, 1});
      changed = true;
    }
    if (ImGui::Button("Oak Trunk Full Process")) {
      target.strand_model_parameters.end_node_strands = 3200;
      target.strand_model_parameters.strand_radius_distribution.mean.max_value = 0.004f;
      target.strand_model_parameters.strand_radius_distribution.mean.curve = Curve2D(1.0f, 0.6f, {0, 0}, {1, 1});
      auto& values = target.strand_model_parameters.strand_radius_distribution.mean.curve.UnsafeGetValues();
      // First logical point (index 0)
      // values[0] = glm::vec2(-0.05f, 0.0f);  // left tangent offset
      values[2] = glm::vec2(0.0f, -0.4f);  // right tangent offset

      // Second logical point (index 1)
      values[3] = glm::vec2(-0.1f, 0.0f);  // left tangent offset
      // values[5] = glm::vec2(0.04f, 0.0f);     // right tangent offset
      changed = true;
    }
    if (ImGui::Button("Elm")) {
      target.strand_model_parameters.strand_radius_distribution.mean.curve = Curve2D(0.65f, 0.5f, {0, 0}, {1, 1});
      changed = true;
    }
    if (ImGui::Button("Spruce")) {
      target.strand_model_parameters.strand_radius_distribution.mean.curve = Curve2D(0.5f, 0.7f, {0, 0}, {1, 1});
      auto& values = target.strand_model_parameters.strand_radius_distribution.mean.curve.UnsafeGetValues();
      // First logical point (index 0)
      // values[0] = glm::vec2(-0.05f, 0.0f);  // left tangent offset
      values[2] = glm::vec2(0.1f, -0.03f);  // right tangent offset

      // Second logical point (index 1)
      values[3] = glm::vec2(-0.06f, -0.12f);  // left tangent offset
      // values[5] = glm::vec2(0.04f, 0.0f);     // right tangent offset
      changed = true;
    }
    if (ImGui::Button("Oak")) {
      target.strand_model_parameters.strand_radius_distribution.mean.max_value = 0.003f;
      target.strand_model_parameters.strand_radius_distribution.mean.curve = Curve2D(0.9f, 0.5f, {0, 0}, {1, 1});
      changed = true;
    }
    ImGui::TreePop();
  }
#ifdef BILLBOARD_CLOUDS_PACKAGE

  billboard_clouds_package::InspectBillboardSettings(foliage_billboard_cloud_generate_settings,
                                                     "Foliage billboard cloud settings");

  if (ImGui::Button("Generate billboard")) {
    target.GenerateBillboardClouds(foliage_billboard_cloud_generate_settings);
  }
#endif
  const auto eco_sys_lab_layer = ApplicationContext::Get().GetLayer<EcoSysLabEditorLayer>();
  const auto scene = target.GetScene();
  editor_layer->DragAndDropButton<TreeDescriptor>(target.tree_descriptor_ref, "TreeDescriptor", true);

  if (!space_colonization_grid_particle_info_list) {
    space_colonization_grid_particle_info_list = AssetManager::CreateTemporaryAsset<ParticleInfoList>();
  }

  if (const auto td = target.tree_descriptor_ref.Get<TreeDescriptor>()) {
    const auto sd = td->shoot_descriptor.Get<BasicShootDescriptor>();
    if (sd) {
      ImGui::DragInt("TreeModel Seed", &target.shoot_model.seed, 1, 0);
      ImGui::DragInt("StrandModel Seed", &target.shoot_strand_model.seed, 1, 0);
      if (ImGui::TreeNode("Tree settings")) {
        if (ImGui::DragFloat("Start time", &target.start_time, 0.01f, 0.0f, 100.f))
          changed = true;
        ImGui::Checkbox("Enable History", &target.enable_history);
        if (target.enable_history) {
          ImGui::DragInt("History per iteration", &target.history_iteration, 1, 1, 1000);
        }
        if (ImGui::TreeNode("Sagging")) {
          bool bending_changed = false;
          bending_changed =
              ImGui::DragFloat("Bending strength", &sd->gravity_bending_strength, 0.01f, 0.0f, 1.0f, "%.3f") ||
              bending_changed;
          bending_changed = ImGui::DragFloat("Bending thickness factor", &sd->gravity_bending_thickness_factor, 0.1f,
                                             0.0f, 10.f, "%.3f") ||
                            bending_changed;
          bending_changed =
              ImGui::DragFloat("Bending angle factor", &sd->gravity_bending_max, 0.01f, 0.0f, 1.0f, "%.3f") ||
              bending_changed;
          if (bending_changed) {
            target.shoot_growth_controller_.sagging = [=](std::mt19937& random_engine,
                                                          const ShootGrowthData& shoot_growth_data,
                                                          const SkeletonNode<InternodeGrowthData>& internode) {
              float strength =
                  internode.data.sagging_force * sd->gravity_bending_strength /
                  glm::pow(internode.info.thickness / sd->end_node_thickness, sd->gravity_bending_thickness_factor);
              strength = sd->gravity_bending_max * (1.f - glm::exp(-glm::abs(strength)));
              return strength;
            };
            target.shoot_model.CalculateTransform(target.shoot_growth_controller_, true);
            shoot_visualizer.need_update = true;
          }
          ImGui::TreePop();
        }
        if (InspectSettings(target.shoot_model.tree_growth_settings, editor_layer))
          changed = true;

        if (target.shoot_model.tree_growth_settings.use_space_colonization &&
            !target.shoot_model.tree_growth_settings.space_colonization_auto_resize) {
          ImGui::DragFloat("Import radius", &radius, 0.01f, 0.01f, 10.0f);
          ImGui::DragInt("Markers per voxel", &markers_per_voxel);
          EditorFileDialogs::OpenFile(
              "Load Voxel Data", "Binvox", {".binvox"},
              [&](const std::filesystem::path& path) {
                auto& occupancy_grid = target.shoot_model.tree_occupancy_grid;
                if (VoxelGrid<TreeOccupancyGridBasicData> input_grid{}; target.ParseBinvox(path, input_grid, 1.f)) {
                  occupancy_grid.Initialize(
                      input_grid, glm::vec3(-radius, 0, -radius), glm::vec3(radius, 2.0f * radius, radius),
                      sd->internode_length,
                      target.shoot_model.tree_growth_settings.space_colonization_removal_distance_factor,
                      target.shoot_model.tree_growth_settings.space_colonization_theta,
                      target.shoot_model.tree_growth_settings.space_colonization_detection_distance_factor,
                      markers_per_voxel);
                }
              },
              false);

          if (editor_layer->DragAndDropButton<MeshRenderer>(private_component_ref, "Add Obstacle")) {
            if (const auto mmr = private_component_ref.Get<MeshRenderer>()) {
              const auto cube_volume = AssetManager::CreateTemporaryAsset<CubeVolume>();
              cube_volume->ApplyMeshBounds(mmr->mesh.Get<Mesh>());
              const auto global_transform = scene->GetDataComponent<GlobalTransform>(mmr->GetOwner());
              target.shoot_model.tree_occupancy_grid.InsertObstacle(global_transform, cube_volume);
              private_component_ref.Clear();
            }
          }
        }

        ImGui::TreePop();
      }

      if (ImGui::TreeNode("Cylindrical Mesh generation settings")) {
        ImGui::DragInt("Iterations", &mesh_generate_iterations, 1, 0, target.shoot_model.CurrentIteration());
        mesh_generate_iterations = glm::clamp(mesh_generate_iterations, 0, target.shoot_model.CurrentIteration());
        InspectSettings(target.tree_mesh_generator_settings, editor_layer);

        ImGui::TreePop();
      }
      if (ImGui::Button("Generate Cylindrical Mesh")) {
        target.GenerateGeometryEntities(target.tree_mesh_generator_settings, mesh_generate_iterations);
      }
      ImGui::SameLine();
      if (ImGui::Button("Clear Cylindrical Mesh")) {
        target.ClearGeometryEntities();
      }

      if (ImGui::Button("Generate Animated Cylindrical Mesh")) {
        target.GenerateAnimatedGeometryEntities(target.tree_mesh_generator_settings, mesh_generate_iterations);
      }
      ImGui::SameLine();
      if (ImGui::Button("Clear Animated Cylindrical Mesh")) {
        target.ClearAnimatedGeometryEntities();
      }
    }

    if (target.shoot_model.tree_growth_settings.use_space_colonization) {
      bool need_grid_update = false;
      if (shoot_visualizer.need_update) {
        need_grid_update = true;
      }
      if (ImGui::Button("Update grids"))
        need_grid_update = true;
      ImGui::Checkbox("Show Space Colonization Grid", &show_space_colonization_grid);
      if (show_space_colonization_grid) {
        if (need_grid_update) {
          auto& occupancy_grid = target.shoot_model.tree_occupancy_grid;
          auto& voxel_grid = occupancy_grid.RefGrid();
          const auto num_voxels = voxel_grid.GetVoxelCount();
          std::vector<ParticleInfo> scalar_matrices{};

          if (scalar_matrices.size() != num_voxels) {
            scalar_matrices.resize(num_voxels);
          }

          if (scalar_matrices.size() != num_voxels) {
            scalar_matrices.reserve(occupancy_grid.GetMarkersPerVoxel() * num_voxels);
          }
          int i = 0;
          for (const auto& voxel : voxel_grid.RefData()) {
            for (const auto& marker : voxel.markers) {
              scalar_matrices.resize(i + 1);
              scalar_matrices[i].instance_matrix.value = glm::translate(marker.position) *
                                                         glm::mat4_cast(glm::quat(glm::vec3(0.0f))) *
                                                         glm::scale(glm::vec3(voxel_grid.GetVoxelSize() * 0.2f));
              if (marker.node_handle == -1)
                scalar_matrices[i].instance_color = glm::vec4(1.0f, 1.0f, 1.0f, 0.75f);
              else {
                scalar_matrices[i].instance_color =
                    glm::vec4(eco_sys_lab_layer->RandomColors()[marker.node_handle], 1.0f);
              }
              i++;
            }
          }
          space_colonization_grid_particle_info_list->SetParticleInfos(scalar_matrices);
        }
        GizmoSettings gizmo_settings{};
        gizmo_settings.draw_settings.blending = true;
        editor_layer->DrawGizmoMeshInstancedColored(Resources::GetInstance().GetPrimitives().cube,
                                                    space_colonization_grid_particle_info_list, glm::mat4(1.0f), 1.0f,
                                                    gizmo_settings);
      }
    }

    if (target.enable_history) {
      if (ImGui::Button("Temporal Progression")) {
        target.temporal_progression = true;
        target.temporal_progression_iteration = 0;
      }
    }
  }

  /*
  ImGui::Checkbox("Split root test", &splitRootTest);
  ImGui::Checkbox("Biomass history", &record_biomass_history);

  if (splitRootTest) ImGui::Text(("Left/Right side biomass: [" + std::to_string(m_leftSideBiomass) + ", " +
  std::to_string(right_side_biomass) + "]").c_str());
  */

  if (ImGui::TreeNode("Strand Model")) {
    if (InspectSettings(target.strand_model_parameters, editor_layer))
      changed = true;

    ImGui::Text(("Strand count: " +
                 std::to_string(target.shoot_strand_model.strand_model_skeleton.data.strand_group.PeekStrands().size()))
                    .c_str());
    ImGui::Text(("Total particle count: " +
                 std::to_string(target.shoot_strand_model.strand_model_skeleton.data.num_of_particles))
                    .c_str());

    if (ImGui::Button("Rebuild Strand Model")) {
      target.BuildStrandModel();
    }

    ImGui::SameLine();
    if (ImGui::Button("Clear Strand Model")) {
      target.shoot_strand_model = {};
    }

    if (ImGui::TreeNodeEx("Strand Model Mesh Generator Settings", ImGuiTreeNodeFlags_DefaultOpen)) {
      InspectSettings(target.strand_model_mesh_generator_settings, editor_layer);
      ImGui::TreePop();
    }

    ImGui::TreePop();
  }

  if (ImGui::Button("Build StrandRenderer")) {
    target.InitializeStrandRenderer();
  }
  ImGui::SameLine();
  if (ImGui::Button("Clear StrandRenderer")) {
    target.ClearStrandRenderer();
  }
  if (ImGui::Button("Build Strand Particles")) {
    target.InitializeStrandParticles();
  }
  ImGui::SameLine();
  if (ImGui::Button("Clear Strand Particles")) {
    target.ClearStrandParticles();
  }
  if (ImGui::Button("Build Strand Mesh")) {
    target.InitializeStrandModelMeshRenderer(target.strand_model_mesh_generator_settings);
  }
  ImGui::SameLine();
  if (ImGui::Button("Clear Strand Mesh")) {
    target.ClearStrandModelMeshRenderer();
  }

  shoot_visualizer.Visualize(target.shoot_strand_model);
  if (ImGui::TreeNode("Skeletal graph settings")) {
    if (InspectSettings(target.skeletal_graph_settings, editor_layer))
      changed = true;

    ImGui::TreePop();
  }
  if (ImGui::Button("Build skeletal graph")) {
    target.GenerateSkeletalGraph(target.skeletal_graph_settings, -1, Resources::GetInstance().GetPrimitives().sphere,
                                 Resources::GetInstance().GetPrimitives().cube);
  }
  ImGui::SameLine();
  if (ImGui::Button("Clear skeletal graph")) {
    target.ClearSkeletalGraph();
  }

  EditorFileDialogs::SaveFile(
      "Export Cylindrical Mesh", "OBJ", {".obj"},
      [&](const std::filesystem::path& path) {
        target.ExportObj(path, target.tree_mesh_generator_settings);
      },
      false);
  ImGui::SameLine();
  EditorFileDialogs::SaveFile(
      "Export Strand Mesh", "OBJ", {".obj"},
      [&](const std::filesystem::path& path) {
        target.ExportStrandModelObj(path, target.strand_model_mesh_generator_settings);
      },
      false);

  return changed;
}
