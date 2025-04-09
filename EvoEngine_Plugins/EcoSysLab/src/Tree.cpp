//
// Created by lllll on 10/24/2022.
//
#include "Tree.hpp"
#include <Material.hpp>
#include <Mesh.hpp>
#include <TransformGraph.hpp>
#include "BasicShootDescriptor.hpp"
#include "SkeletonSerializer.hpp"
#include "StrandGroupSerializer.hpp"

#include "Application.hpp"
#include "Climate.hpp"
#include "EcoSysLabLayer.hpp"
#include "EditorLayer.hpp"
#include "Octree.hpp"
#include "Soil.hpp"
#include "StrandModelProfileSerializer.hpp"

using namespace eco_sys_lab_plugin;
TreeStatistics Tree::GetTreeStatistics() const {
  TreeStatistics ret_val{};
  const auto& skeleton = tree_model.PeekShootSkeleton();
  ret_val.Calculate(skeleton);
  return ret_val;
}

void Tree::Reset() {
  ClearSkeletalGraph();
  ClearGeometryEntities();
  ClearStrandModelMeshRenderer();
  ClearStrandRenderer();
  ClearAnimatedGeometryEntities();
  tree_model.Clear();
  strand_model = {};
  tree_model.shoot_skeleton_.data.index = GetOwner().GetIndex();
  tree_visualizer.Reset(tree_model);
}

bool Tree::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
#ifdef BILLBOARD_CLOUDS_PLUGIN
  static BillboardCloud::GenerateSettings foliage_billboard_cloud_generate_settings{};

  foliage_billboard_cloud_generate_settings.OnInspect("Foliage billboard cloud settings");

  if (ImGui::Button("Generate billboard")) {
    GenerateBillboardClouds(foliage_billboard_cloud_generate_settings);
  }
#endif
  bool changed = false;
  const auto eco_sys_lab_layer = Application::GetLayer<EcoSysLabLayer>();
  const auto scene = GetScene();
  editor_layer->DragAndDropButton<TreeDescriptor>(tree_descriptor_ref, "TreeDescriptor", true);
  static bool show_space_colonization_grid = true;

  static std::shared_ptr<ParticleInfoList> space_colonization_grid_particle_info_list;
  if (!space_colonization_grid_particle_info_list) {
    space_colonization_grid_particle_info_list = AssetManager::CreateTemporaryAsset<ParticleInfoList>();
  }

  if (const auto td = tree_descriptor_ref.Get<TreeDescriptor>()) {
    const auto sd = td->shoot_descriptor.Get<BasicShootDescriptor>();
    if (sd) {
      ImGui::DragInt("TreeModel Seed", &tree_model.seed, 1, 0);
      ImGui::DragInt("StrandModel Seed", &strand_model.seed, 1, 0);
      if (ImGui::TreeNode("Tree settings")) {
        if (ImGui::DragFloat("Start time", &start_time, 0.01f, 0.0f, 100.f))
          changed = true;
        ImGui::Checkbox("Enable History", &enable_history);
        if (enable_history) {
          ImGui::DragInt("History per iteration", &history_iteration, 1, 1, 1000);
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
            shoot_growth_controller_.sagging = [=](std::mt19937& random_engine,
                                                   const ShootGrowthData& shoot_growth_data,
                                                   const SkeletonNode<InternodeGrowthData>& internode) {
              float strength =
                  internode.data.sagging_force * sd->gravity_bending_strength /
                  glm::pow(internode.info.thickness / sd->end_node_thickness, sd->gravity_bending_thickness_factor);
              strength = sd->gravity_bending_max * (1.f - glm::exp(-glm::abs(strength)));
              return strength;
            };
            tree_model.CalculateTransform(shoot_growth_controller_, true);
            tree_visualizer.m_needUpdate = true;
          }
        }
        if (tree_model.tree_growth_settings.OnInspect(editor_layer))
          changed = true;

        if (tree_model.tree_growth_settings.use_space_colonization &&
            !tree_model.tree_growth_settings.space_colonization_auto_resize) {
          static float radius = 1.5f;
          static int markers_per_voxel = 5;
          ImGui::DragFloat("Import radius", &radius, 0.01f, 0.01f, 10.0f);
          ImGui::DragInt("Markers per voxel", &markers_per_voxel);
          FileUtils::OpenFile(
              "Load Voxel Data", "Binvox", {".binvox"},
              [&](const std::filesystem::path& path) {
                auto& occupancy_grid = tree_model.tree_occupancy_grid;
                if (VoxelGrid<TreeOccupancyGridBasicData> input_grid{}; ParseBinvox(path, input_grid, 1.f)) {
                  occupancy_grid.Initialize(
                      input_grid, glm::vec3(-radius, 0, -radius), glm::vec3(radius, 2.0f * radius, radius),
                      sd->internode_length, tree_model.tree_growth_settings.space_colonization_removal_distance_factor,
                      tree_model.tree_growth_settings.space_colonization_theta,
                      tree_model.tree_growth_settings.space_colonization_detection_distance_factor, markers_per_voxel);
                }
              },
              false);

          static PrivateComponentRef private_component_ref{};

          if (editor_layer->DragAndDropButton<MeshRenderer>(private_component_ref, "Add Obstacle")) {
            if (const auto mmr = private_component_ref.Get<MeshRenderer>()) {
              const auto cube_volume = AssetManager::CreateTemporaryAsset<CubeVolume>();
              cube_volume->ApplyMeshBounds(mmr->mesh.Get<Mesh>());
              const auto global_transform = scene->GetDataComponent<GlobalTransform>(mmr->GetOwner());
              tree_model.tree_occupancy_grid.InsertObstacle(global_transform, cube_volume);
              private_component_ref.Clear();
            }
          }
        }

        ImGui::TreePop();
      }
      static int mesh_generate_iterations = 0;
      if (ImGui::TreeNode("Cylindrical Mesh generation settings")) {
        ImGui::DragInt("Iterations", &mesh_generate_iterations, 1, 0, tree_model.CurrentIteration());
        mesh_generate_iterations = glm::clamp(mesh_generate_iterations, 0, tree_model.CurrentIteration());
        tree_mesh_generator_settings.OnInspect(editor_layer);

        ImGui::TreePop();
      }
      if (ImGui::Button("Generate Cylindrical Mesh")) {
        GenerateGeometryEntities(tree_mesh_generator_settings, mesh_generate_iterations);
      }
      ImGui::SameLine();
      if (ImGui::Button("Clear Cylindrical Mesh")) {
        ClearGeometryEntities();
      }

      if (ImGui::Button("Generate Animated Cylindrical Mesh")) {
        GenerateAnimatedGeometryEntities(tree_mesh_generator_settings, mesh_generate_iterations);
      }
      ImGui::SameLine();
      if (ImGui::Button("Clear Animated Cylindrical Mesh")) {
        ClearAnimatedGeometryEntities();
      }
    }

    if (tree_model.tree_growth_settings.use_space_colonization) {
      bool need_grid_update = false;
      if (tree_visualizer.m_needUpdate) {
        need_grid_update = true;
      }
      if (ImGui::Button("Update grids"))
        need_grid_update = true;
      ImGui::Checkbox("Show Space Colonization Grid", &show_space_colonization_grid);
      if (show_space_colonization_grid) {
        if (need_grid_update) {
          auto& occupancy_grid = tree_model.tree_occupancy_grid;
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
        editor_layer->DrawGizmoMeshInstancedColored(Resources::Primitives::cube,
                                                    space_colonization_grid_particle_info_list, glm::mat4(1.0f), 1.0f,
                                                    gizmo_settings);
      }
    }

    if (enable_history) {
      if (ImGui::Button("Temporal Progression")) {
        temporal_progression = true;
        temporal_progression_iteration = 0;
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
    if (strand_model_parameters.OnInspect(editor_layer))
      changed = true;

    ImGui::Text(
        ("Strand count: " + std::to_string(strand_model.strand_model_skeleton.data.strand_group.PeekStrands().size()))
            .c_str());
    ImGui::Text(
        ("Total particle count: " + std::to_string(strand_model.strand_model_skeleton.data.num_of_particles)).c_str());

    if (ImGui::Button("Rebuild Strand Model")) {
      BuildStrandModel();
    }

    ImGui::SameLine();
    if (ImGui::Button("Clear Strand Model")) {
      strand_model = {};
    }

    if (ImGui::TreeNodeEx("Strand Model Mesh Generator Settings", ImGuiTreeNodeFlags_DefaultOpen)) {
      strand_model_mesh_generator_settings.OnInspect(editor_layer);
      ImGui::TreePop();
    }

    ImGui::TreePop();
  }

  if (ImGui::Button("Build StrandRenderer")) {
    InitializeStrandRenderer();
  }
  ImGui::SameLine();
  if (ImGui::Button("Clear StrandRenderer")) {
    ClearStrandRenderer();
  }
  if (ImGui::Button("Build Strand Particles")) {
    InitializeStrandParticles();
  }
  ImGui::SameLine();
  if (ImGui::Button("Clear Strand Particles")) {
    ClearStrandParticles();
  }
  if (ImGui::Button("Build Strand Mesh")) {
    InitializeStrandModelMeshRenderer(strand_model_mesh_generator_settings);
  }
  ImGui::SameLine();
  if (ImGui::Button("Clear Strand Mesh")) {
    ClearStrandModelMeshRenderer();
  }

  tree_visualizer.Visualize(strand_model);
  if (ImGui::TreeNode("Skeletal graph settings")) {
    if (skeletal_graph_settings.OnInspect(editor_layer))
      changed = true;
  }
  if (ImGui::Button("Build skeletal graph")) {
    GenerateSkeletalGraph(skeletal_graph_settings, -1, Resources::Primitives::sphere, Resources::Primitives::cube);
  }
  ImGui::SameLine();
  if (ImGui::Button("Clear skeletal graph")) {
    ClearSkeletalGraph();
  }

  FileUtils::SaveFile(
      "Export Cylindrical Mesh", "OBJ", {".obj"},
      [&](const std::filesystem::path& path) {
        ExportObj(path, tree_mesh_generator_settings);
      },
      false);
  ImGui::SameLine();
  FileUtils::SaveFile(
      "Export Strand Mesh", "OBJ", {".obj"},
      [&](const std::filesystem::path& path) {
        ExportStrandModelObj(path, strand_model_mesh_generator_settings);
      },
      false);

  return changed;
}
void Tree::Update() {
  if (temporal_progression) {
    if (temporal_progression_iteration <= tree_model.CurrentIteration()) {
      GenerateGeometryEntities(tree_mesh_generator_settings, temporal_progression_iteration);
      temporal_progression_iteration++;
    } else {
      temporal_progression_iteration = 0;
      temporal_progression = false;
    }
  }
  const auto editor_layer = Application::GetLayer<EditorLayer>();
  const auto eco_sys_lab_layer = Application::GetLayer<EcoSysLabLayer>();
}

void Tree::OnCreate() {
  tree_visualizer.Initialize();
  tree_visualizer.m_needUpdate = true;
  strand_model_parameters.branch_twist_distribution.mean = {-60.0f, 60.0f};
  strand_model_parameters.branch_twist_distribution.deviation = {0.0f, 1.0f, {0, 0}};

  strand_model_parameters.junction_twist_distribution.mean = {-60.0f, 60.0f};
  strand_model_parameters.junction_twist_distribution.deviation = {0.0f, 1.0f, {0, 0}};

  strand_model_parameters.strand_radius_distribution.mean = {0.0f, 0.002f};
  strand_model_parameters.strand_radius_distribution.deviation = {0.0f, 1.0f, {0, 0}};

  strand_model_parameters.cladoptosis_distribution.mean = {0.0f, 0.02f};
  strand_model_parameters.cladoptosis_distribution.deviation = {0.0f, 1.0f, {0, 0}};
}

void Tree::OnDestroy() {
  tree_model = {};
  strand_model = {};

  tree_descriptor_ref.Clear();
  soil.Clear();
  climate.Clear();
  enable_history = false;

  tree_visualizer.Clear();

  left_side_biomass = right_side_biomass = 0.0f;
  root_biomass_history.clear();
  shoot_biomass_history.clear();

  generate_mesh = true;
  start_time = 0.f;
}

void Tree::CalculateProfiles() {
  const float time = Times::Now();
  strand_model.strand_model_skeleton.Clone(tree_model.RefShootSkeleton());
  strand_model.ResetAllProfiles(strand_model_parameters);
  strand_model.InitializeProfiles(strand_model_parameters);
  const auto worker_handle = strand_model.CalculateProfiles(strand_model_parameters);
  Jobs::Wait(worker_handle);
  const float profile_calculation_time = Times::Now() - time;
  std::string output;
  output += "\nProfile count: [" + std::to_string(strand_model.strand_model_skeleton.PeekSortedNodeList().size());
  output +=
      "], Strand count: [" + std::to_string(strand_model.strand_model_skeleton.data.strand_group.PeekStrands().size());
  output += "], Particle count: [" + std::to_string(strand_model.strand_model_skeleton.data.num_of_particles);
  output += "]\nCalculate Profile Used time: " + std::to_string(profile_calculation_time) + "\n";
  EVOENGINE_LOG(output);
}

void Tree::BuildStrandModel() {
  std::string output;

  CalculateProfiles();
  const float time = Times::Now();
  for (const auto& node_handle : tree_model.PeekShootSkeleton().PeekSortedNodeList()) {
    strand_model.strand_model_skeleton.RefNode(node_handle).info =
        tree_model.PeekShootSkeleton().PeekNode(node_handle).info;
  }
  strand_model.CalculateStrandProfileAdjustedTransforms(strand_model_parameters);
  strand_model.ApplyProfiles(strand_model_parameters);
  const float strand_modeling_time = Times::Now() - time;
  output += "\nBuild Strand Model Used time: " + std::to_string(strand_modeling_time) + "\n";
  EVOENGINE_LOG(output);
}

bool Tree::TryGrow(const SimulationSettings& simulation_settings, bool pruning) {
  const auto scene = GetScene();
  const auto eco_sys_lab_layer = Application::GetLayer<EcoSysLabLayer>();
  if (const auto climate_candidate = EcoSysLabLayer::FindClimate(); !climate_candidate.expired())
    climate = climate_candidate.lock();
  if (const auto soil_candidate = EcoSysLabLayer::FindSoil(); !soil_candidate.expired())
    soil = soil_candidate.lock();
  const auto s = this->soil.Get<Soil>();
  const auto c = this->climate.Get<Climate>();
  if (!s) {
    EVOENGINE_ERROR("No soil model!")
    return false;
  }
  if (!c) {
    EVOENGINE_ERROR("No climate model!")
    return false;
  }

  try {
    PrepareController(simulation_settings);
  } catch (const std::exception& e) {
    EVOENGINE_ERROR(e.what())
    return false;
  }
  const auto owner = GetOwner();
  const bool grown =
      tree_model.Grow(simulation_settings.delta_time, scene->GetDataComponent<GlobalTransform>(owner).value,
                      c->climate_model, shoot_growth_controller_, shoot_pruning_controller_, pruning);
  if (grown) {
    if (pruning)
      tree_visualizer.ClearSelections();
    tree_visualizer.m_needUpdate = true;
  }
  if (enable_history && tree_model.iteration_ % history_iteration == 0)
    tree_model.Step();
  if (record_biomass_history) {
    const auto& base_shoot_node = tree_model.RefShootSkeleton().RefNode(0);
    shoot_biomass_history.emplace_back(base_shoot_node.data.biomass + base_shoot_node.data.descendant_total_biomass);
  }
  return grown;
}

bool Tree::TryGrowSubTree(const SimulationSettings& simulation_settings, const SkeletonNodeHandle base_internode_handle,
                          const bool pruning) {
  const auto scene = GetScene();
  const auto eco_sys_lab_layer = Application::GetLayer<EcoSysLabLayer>();

  const auto climate_candidate = EcoSysLabLayer::FindClimate();
  if (!climate_candidate.expired())
    climate = climate_candidate.lock();
  if (const auto soil_candidate = EcoSysLabLayer::FindSoil(); !soil_candidate.expired())
    soil = soil_candidate.lock();

  const auto s = soil.Get<Soil>();
  const auto c = climate.Get<Climate>();

  if (!s) {
    EVOENGINE_ERROR("No soil model!");
    return false;
  }
  if (!c) {
    EVOENGINE_ERROR("No climate model!");
    return false;
  }
  try {
    PrepareController(simulation_settings);
  } catch (const std::exception& e) {
    EVOENGINE_ERROR(e.what())
    return false;
  }
  const auto owner = GetOwner();

  const bool grown = tree_model.Grow(simulation_settings.delta_time, base_internode_handle,
                                     scene->GetDataComponent<GlobalTransform>(owner).value, c->climate_model,
                                     shoot_growth_controller_, shoot_pruning_controller_, pruning);
  if (grown) {
    if (pruning)
      tree_visualizer.ClearSelections();
    tree_visualizer.m_needUpdate = true;
  }
  if (enable_history && tree_model.iteration_ % history_iteration == 0)
    tree_model.Step();
  if (record_biomass_history) {
    const auto& base_shoot_node = tree_model.RefShootSkeleton().RefNode(0);
    shoot_biomass_history.emplace_back(base_shoot_node.data.biomass + base_shoot_node.data.descendant_total_biomass);
  }
  return grown;
}

void Tree::Serialize(YAML::Emitter& out) const {
  tree_descriptor_ref.Save("tree_descriptor_ref", out);

  strand_model_parameters.Save("strand_model_parameters", out);
  tree_mesh_generator_settings.Save("tree_mesh_generator_settings", out);
  strand_model.Save("strand_model", out);
  tree_model.Save("tree_model", out);
}

void Tree::Deserialize(const YAML::Node& in) {
  tree_descriptor_ref.Load("tree_descriptor_ref", in);

  strand_model_parameters.Load("strand_model_parameters", in);
  tree_mesh_generator_settings.Load("tree_mesh_generator_settings", in);

  strand_model.Load("strand_model", in);
  tree_model.Load("tree_model", in);
}

void Tree::RegisterVoxel() {
  const auto scene = GetScene();
  const auto owner = GetOwner();
  const auto global_transform = scene->GetDataComponent<GlobalTransform>(owner).value;
  tree_model.shoot_skeleton_.data.index = owner.GetIndex();
  const auto c = climate.Get<Climate>();
  tree_model.RegisterVoxel(global_transform, c->climate_model);
}

void Tree::ExportRadialBoundingVolume(const std::shared_ptr<RadialBoundingVolume>& rbv) const {
  const auto& sorted_internode_list = tree_model.shoot_skeleton_.PeekSortedNodeList();
  const auto& skeleton = tree_model.shoot_skeleton_;
  std::vector<glm::vec3> points;
  for (const auto& node_handle : sorted_internode_list) {
    const auto& node = skeleton.PeekNode(node_handle);
    points.emplace_back(node.info.global_position);
    points.emplace_back(node.info.GetGlobalEndPosition());
  }
  rbv->CalculateVolume(points);
}

void Tree::CollectAssetRef(std::vector<AssetRef>& list) {
  if (tree_descriptor_ref.Get<TreeDescriptor>()) {
    list.emplace_back(tree_descriptor_ref);
  }
}

void Tree::PrepareController(const SimulationSettings& simulation_settings) {
  const auto td = tree_descriptor_ref.Get<TreeDescriptor>();
  if (!td) {
    throw std::runtime_error("Growing tree without tree descriptor!");
  }
  const auto shoot_descriptor = td->shoot_descriptor.Get<IShootDescriptor>();
  if (!shoot_descriptor) {
    throw std::runtime_error("Shoot Descriptor Missing!");
  }
  const auto pruning_descriptor = td->pruning_descriptor.Get<IPruningDescriptor>();
  if (!pruning_descriptor) {
    throw std::runtime_error("Pruning Descriptor Missing!");
  }
  const auto foliage_descriptor = td->foliage_descriptor.Get<IFoliageDescriptor>();
  if (!foliage_descriptor) {
    throw std::runtime_error("Foliage Descriptor Missing!");
  }
  if (const auto fruit_descriptor = td->fruit_descriptor.Get<IFruitDescriptor>()) {
    fruit_descriptor->PrepareGrowthController(shoot_growth_controller_);
  } else {
    shoot_growth_controller_.fruit = [&](std::mt19937& random_engine, const ShootGrowthData& shoot_growth_data,
                                         const SkeletonNode<InternodeGrowthData>& internode) {
      return 0.0f;
    };
    shoot_growth_controller_.fruit_fall_probability = [&](std::mt19937& random_engine,
                                                          const ShootGrowthData& shoot_growth_data,
                                                          const SkeletonNode<InternodeGrowthData>& internode) {
      return 0.0f;
    };
  }
  shoot_descriptor->PrepareGrowthController(shoot_growth_controller_);
  foliage_descriptor->PrepareGrowthController(shoot_growth_controller_);
  pruning_descriptor->PreparePruningController(simulation_settings, shoot_pruning_controller_);
}