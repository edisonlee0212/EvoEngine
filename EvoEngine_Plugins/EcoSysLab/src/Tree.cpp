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
#include "assimp/contrib/zip/src/miniz.h"

using namespace eco_sys_lab_plugin;

TreeStatistics Tree::GetTreeStatistics() const {
  TreeStatistics ret_val{};
  const auto& skeleton = shoot_model.PeekShootSkeleton();
  ret_val.Calculate(skeleton);
  return ret_val;
}

void Tree::Reset() {
  ClearSkeletalGraph();
  ClearGeometryEntities();
  ClearStrandModelMeshRenderer();
  ClearStrandRenderer();
  ClearAnimatedGeometryEntities();
  shoot_model.Clear();
  root_model.Clear();
  shoot_strand_model = {};
  shoot_model.shoot_skeleton_.data.entity_index = root_model.root_skeleton_.data.entity_index = GetOwner().GetIndex();
  shoot_visualizer.Reset(shoot_model);
  root_visualizer.Reset(root_model);
}

bool Tree::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  bool changed = false;
  if (ImGui::TreeNode("Preset settings")) {
    if (ImGui::Button("Oak Trunk Crack Process")) {
      strand_model_parameters.end_node_strands = 3200;
      strand_model_parameters.strand_radius_distribution.mean.max_value = 0.003f;
      strand_model_parameters.strand_radius_distribution.mean.curve = Curve2D(1.0f, 0.8f, {0, 0}, {1, 1});
      changed = true;
    }
    if (ImGui::Button("Oak Trunk Full Process")) {
      strand_model_parameters.end_node_strands = 3200;
      strand_model_parameters.strand_radius_distribution.mean.max_value = 0.004f;
      strand_model_parameters.strand_radius_distribution.mean.curve = Curve2D(1.0f, 0.6f, {0, 0}, {1, 1});
      auto& values = strand_model_parameters.strand_radius_distribution.mean.curve.UnsafeGetValues();
      // First logical point (index 0)
      // values[0] = glm::vec2(-0.05f, 0.0f);  // left tangent offset
      values[2] = glm::vec2(0.0f, -0.4f);  // right tangent offset

      // Second logical point (index 1)
      values[3] = glm::vec2(-0.1f, 0.0f);  // left tangent offset
      // values[5] = glm::vec2(0.04f, 0.0f);     // right tangent offset
      changed = true;
    }
    if (ImGui::Button("Elm")) {
      strand_model_parameters.strand_radius_distribution.mean.curve = Curve2D(0.65f, 0.5f, {0, 0}, {1, 1});
      changed = true;
    }
    if (ImGui::Button("Spruce")) {
      strand_model_parameters.strand_radius_distribution.mean.curve = Curve2D(0.5f, 0.7f, {0, 0}, {1, 1});
      auto& values = strand_model_parameters.strand_radius_distribution.mean.curve.UnsafeGetValues();
      // First logical point (index 0)
      // values[0] = glm::vec2(-0.05f, 0.0f);  // left tangent offset
      values[2] = glm::vec2(0.1f, -0.03f);  // right tangent offset

      // Second logical point (index 1)
      values[3] = glm::vec2(-0.06f, -0.12f);  // left tangent offset
      // values[5] = glm::vec2(0.04f, 0.0f);     // right tangent offset
      changed = true;
    }
    if (ImGui::Button("Oak")) {
      strand_model_parameters.strand_radius_distribution.mean.max_value = 0.003f;
      strand_model_parameters.strand_radius_distribution.mean.curve = Curve2D(0.9f, 0.5f, {0, 0}, {1, 1});
      changed = true;
    }
    ImGui::TreePop();
  }
#ifdef BILLBOARD_CLOUDS_PLUGIN
  static BillboardCloud::GenerateSettings foliage_billboard_cloud_generate_settings{};

  foliage_billboard_cloud_generate_settings.OnInspect("Foliage billboard cloud settings");

  if (ImGui::Button("Generate billboard")) {
    GenerateBillboardClouds(foliage_billboard_cloud_generate_settings);
  }
#endif
  const auto eco_sys_lab_layer = ApplicationContext::Get().GetLayer<EcoSysLabLayer>();
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
      ImGui::DragInt("TreeModel Seed", &shoot_model.seed, 1, 0);
      ImGui::DragInt("StrandModel Seed", &shoot_strand_model.seed, 1, 0);
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
            shoot_model.CalculateTransform(shoot_growth_controller_, true);
            shoot_visualizer.need_update = true;
          }
          ImGui::TreePop();
        }
        if (shoot_model.tree_growth_settings.OnInspect(editor_layer))
          changed = true;

        if (shoot_model.tree_growth_settings.use_space_colonization &&
            !shoot_model.tree_growth_settings.space_colonization_auto_resize) {
          static float radius = 1.5f;
          static int markers_per_voxel = 5;
          ImGui::DragFloat("Import radius", &radius, 0.01f, 0.01f, 10.0f);
          ImGui::DragInt("Markers per voxel", &markers_per_voxel);
          FileUtils::OpenFile(
              "Load Voxel Data", "Binvox", {".binvox"},
              [&](const std::filesystem::path& path) {
                auto& occupancy_grid = shoot_model.tree_occupancy_grid;
                if (VoxelGrid<TreeOccupancyGridBasicData> input_grid{}; ParseBinvox(path, input_grid, 1.f)) {
                  occupancy_grid.Initialize(
                      input_grid, glm::vec3(-radius, 0, -radius), glm::vec3(radius, 2.0f * radius, radius),
                      sd->internode_length, shoot_model.tree_growth_settings.space_colonization_removal_distance_factor,
                      shoot_model.tree_growth_settings.space_colonization_theta,
                      shoot_model.tree_growth_settings.space_colonization_detection_distance_factor, markers_per_voxel);
                }
              },
              false);

          static PrivateComponentRef private_component_ref{};

          if (editor_layer->DragAndDropButton<MeshRenderer>(private_component_ref, "Add Obstacle")) {
            if (const auto mmr = private_component_ref.Get<MeshRenderer>()) {
              const auto cube_volume = AssetManager::CreateTemporaryAsset<CubeVolume>();
              cube_volume->ApplyMeshBounds(mmr->mesh.Get<Mesh>());
              const auto global_transform = scene->GetDataComponent<GlobalTransform>(mmr->GetOwner());
              shoot_model.tree_occupancy_grid.InsertObstacle(global_transform, cube_volume);
              private_component_ref.Clear();
            }
          }
        }

        ImGui::TreePop();
      }
      static int mesh_generate_iterations = 0;
      if (ImGui::TreeNode("Cylindrical Mesh generation settings")) {
        ImGui::DragInt("Iterations", &mesh_generate_iterations, 1, 0, shoot_model.CurrentIteration());
        mesh_generate_iterations = glm::clamp(mesh_generate_iterations, 0, shoot_model.CurrentIteration());
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

    if (shoot_model.tree_growth_settings.use_space_colonization) {
      bool need_grid_update = false;
      if (shoot_visualizer.need_update) {
        need_grid_update = true;
      }
      if (ImGui::Button("Update grids"))
        need_grid_update = true;
      ImGui::Checkbox("Show Space Colonization Grid", &show_space_colonization_grid);
      if (show_space_colonization_grid) {
        if (need_grid_update) {
          auto& occupancy_grid = shoot_model.tree_occupancy_grid;
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

    ImGui::Text(("Strand count: " +
                 std::to_string(shoot_strand_model.strand_model_skeleton.data.strand_group.PeekStrands().size()))
                    .c_str());
    ImGui::Text(
        ("Total particle count: " + std::to_string(shoot_strand_model.strand_model_skeleton.data.num_of_particles))
            .c_str());

    if (ImGui::Button("Rebuild Strand Model")) {
      BuildStrandModel();
    }

    ImGui::SameLine();
    if (ImGui::Button("Clear Strand Model")) {
      shoot_strand_model = {};
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

  shoot_visualizer.Visualize(shoot_strand_model);
  if (ImGui::TreeNode("Skeletal graph settings")) {
    if (skeletal_graph_settings.OnInspect(editor_layer))
      changed = true;

    ImGui::TreePop();
  }
  if (ImGui::Button("Build skeletal graph")) {
    GenerateSkeletalGraph(skeletal_graph_settings, -1, Resources::GetInstance().GetPrimitives().sphere,
                          Resources::GetInstance().GetPrimitives().cube);
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
    if (temporal_progression_iteration <= shoot_model.CurrentIteration()) {
      GenerateGeometryEntities(tree_mesh_generator_settings, temporal_progression_iteration);
      temporal_progression_iteration++;
    } else {
      temporal_progression_iteration = 0;
      temporal_progression = false;
    }
  }
  const auto editor_layer = ApplicationContext::Get().GetLayer<EditorLayer>();
  const auto eco_sys_lab_layer = ApplicationContext::Get().GetLayer<EcoSysLabLayer>();
}

void Tree::OnCreate() {
  shoot_visualizer.Initialize();
  shoot_visualizer.need_update = true;
  root_visualizer.Initialize();
  root_visualizer.need_update = true;

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
  shoot_model = {};
  root_model = {};
  shoot_strand_model = {};

  tree_descriptor_ref.Clear();
  soil.Clear();
  climate.Clear();
  enable_history = false;

  shoot_visualizer.Clear();
  root_visualizer.Clear();

  left_side_biomass = right_side_biomass = 0.0f;
  root_biomass_history.clear();
  shoot_biomass_history.clear();

  generate_mesh = true;
  start_time = 0.f;
}

void Tree::CalculateProfiles() {
  const float time = ApplicationContext::Get().GetTimes().Now();
  shoot_strand_model.strand_model_skeleton.Clone(shoot_model.RefShootSkeleton());
  shoot_strand_model.ResetAllProfiles(strand_model_parameters);
  shoot_strand_model.InitializeProfiles(strand_model_parameters);
  const auto worker_handle = shoot_strand_model.CalculateProfiles(strand_model_parameters);
  Jobs::Wait(worker_handle);
  const float profile_calculation_time = ApplicationContext::Get().GetTimes().Now() - time;
  std::string output;
  output += "\nProfile count: [" + std::to_string(shoot_strand_model.strand_model_skeleton.PeekSortedNodeList().size());
  output += "], Strand count: [" +
            std::to_string(shoot_strand_model.strand_model_skeleton.data.strand_group.PeekStrands().size());
  output += "], Particle count: [" + std::to_string(shoot_strand_model.strand_model_skeleton.data.num_of_particles);
  output += "]\nCalculate Profile Used time: " + std::to_string(profile_calculation_time) + "\n";
  EVOENGINE_LOG(output);
}

void Tree::BuildStrandModel() {
  std::string output;

  CalculateProfiles();
  const float time = ApplicationContext::Get().GetTimes().Now();
  for (const auto& node_handle : shoot_model.PeekShootSkeleton().PeekSortedNodeList()) {
    shoot_strand_model.strand_model_skeleton.RefNode(node_handle).info =
        shoot_model.PeekShootSkeleton().PeekNode(node_handle).info;
  }
  shoot_strand_model.CalculateStrandProfileAdjustedTransforms(strand_model_parameters);
  shoot_strand_model.ApplyProfiles(strand_model_parameters);
  const float strand_modeling_time = ApplicationContext::Get().GetTimes().Now() - time;
  output += "\nBuild Strand Model Used time: " + std::to_string(strand_modeling_time) + "\n";
  EVOENGINE_LOG(output);
}

bool Tree::TryGrow(const SimulationSettings& simulation_settings, const SkeletonNodeHandle base_internode_handle,
                   const bool pruning) {
  const auto scene = GetScene();
  const auto eco_sys_lab_layer = ApplicationContext::Get().GetLayer<EcoSysLabLayer>();

  const auto climate_candidate = EcoSysLabLayer::FindClimate();
  if (!climate_candidate.expired())
    climate = climate_candidate.lock();
  if (const auto soil_candidate = EcoSysLabLayer::FindSoil(); !soil_candidate.expired())
    soil = soil_candidate.lock();

  const auto s = soil.Get<Soil>();
  const auto c = climate.Get<Climate>();

  if (!s) {
    EVOENGINE_ERROR("No soil model!")
    return false;
  }
  if (!c) {
    EVOENGINE_ERROR("No climate model!")
    return false;
  }
  bool shoot_grown = false;
  bool root_grown = false;
  try {
    PrepareController(simulation_settings);
    if (shoot_growth_controller_.Initialized() && !shoot_model.initialized_) {
      shoot_model.Initialize(shoot_growth_controller_, foliage_controller_, shoot_reproduction_controller_);
      shoot_grown = true;
    }
    if (root_growth_controller_.Initialized() && !root_model.initialized_) {
      root_model.Initialize(root_growth_controller_);
      root_grown = true;
    }
  } catch (const std::exception& e) {
    EVOENGINE_ERROR(e.what())
    return false;
  }
  const auto owner = GetOwner();
  const auto global_transform = scene->GetDataComponent<GlobalTransform>(owner).value;
  Vigor shoot_vigor;
  Vigor root_vigor;
  if (shoot_growth_controller_.Initialized()) {
    shoot_vigor = shoot_model.SampleShootFlux(global_transform, c->climate_model, shoot_growth_controller_);
  } else {
    shoot_vigor.value = FLT_MAX;
  }
  if (root_growth_controller_.Initialized()) {
    root_vigor = root_model.SampleRootFlux(global_transform, s->soil_model, root_growth_controller_);
  } else {
    root_vigor.value = FLT_MAX;
  }
  Vigor total_vigor;
  total_vigor.value = glm::min(shoot_vigor.value, root_vigor.value);

  if (shoot_growth_controller_.Initialized()) {
    shoot_model.DistributeVigor(shoot_growth_controller_, total_vigor);
    if (base_internode_handle != -1) {
      shoot_grown = shoot_model.Grow(simulation_settings.delta_time, base_internode_handle, global_transform,
                                     c->climate_model, s->soil_model, shoot_growth_controller_, foliage_controller_,
                                     shoot_reproduction_controller_, shoot_pruning_controller_, pruning) ||
                    shoot_grown;
    } else {
      shoot_grown = shoot_model.Grow(simulation_settings.delta_time, global_transform, c->climate_model, s->soil_model,
                                     shoot_growth_controller_, foliage_controller_, shoot_reproduction_controller_,
                                     shoot_pruning_controller_, pruning) ||
                    shoot_grown;
    }
    if (shoot_grown) {
      if (pruning)
        shoot_visualizer.ClearSelections();
      shoot_visualizer.need_update = true;
      if (!shoot_model.PeekShootSkeleton().PeekSortedNodeList().empty())
        root_model.shoot_skeleton_base_thickness = shoot_model.PeekShootSkeleton().PeekNode(0).info.thickness;
    }
  }

  if (root_growth_controller_.Initialized()) {
    root_model.DistributeVigor(root_growth_controller_, total_vigor);
    if (base_internode_handle == -1) {
      root_grown = root_model.Grow(simulation_settings.delta_time, global_transform, c->climate_model, s->soil_model,
                                   root_growth_controller_, fine_root_controller_, root_reproduction_controller_,
                                   root_pruning_controller_, pruning) ||
                   root_grown;
    }
    if (root_grown) {
      if (pruning)
        root_visualizer.ClearSelections();
      root_visualizer.need_update = true;
    }
  }
  if (enable_history && shoot_model.iteration_ % history_iteration == 0) {
    shoot_model.Step();
    root_model.Step();
  }
  if (record_biomass_history) {
    const auto& base_shoot_node = shoot_model.RefShootSkeleton().RefNode(0);
    shoot_biomass_history.emplace_back(base_shoot_node.data.biomass_factor +
                                       base_shoot_node.data.descendant_total_biomass_factor);
  }
  return shoot_grown || root_grown;
}

void Tree::Serialize(YAML::Emitter& out) const {
  tree_descriptor_ref.Save("tree_descriptor_ref", out);

  strand_model_parameters.Save("strand_model_parameters", out);
  tree_mesh_generator_settings.Save("tree_mesh_generator_settings", out);
  shoot_strand_model.Save("shoot_strand_model", out);
  shoot_model.Save("shoot_model", out);
}

void Tree::Deserialize(const YAML::Node& in) {
  tree_descriptor_ref.Load("tree_descriptor_ref", in);

  strand_model_parameters.Load("strand_model_parameters", in);
  tree_mesh_generator_settings.Load("tree_mesh_generator_settings", in);

  shoot_strand_model.Load("shoot_strand_model", in);
  shoot_model.Load("shoot_model", in);
}

void Tree::RegisterVoxel() {
  const auto scene = GetScene();
  const auto owner = GetOwner();
  const auto global_transform = scene->GetDataComponent<GlobalTransform>(owner).value;
  shoot_model.shoot_skeleton_.data.entity_index = owner.GetIndex();
  const auto c = climate.Get<Climate>();
  shoot_model.RegisterVoxel(global_transform, c->climate_model);
}

void Tree::ExportRadialBoundingVolume(const std::shared_ptr<RadialBoundingVolume>& rbv) const {
  const auto& sorted_internode_list = shoot_model.shoot_skeleton_.PeekSortedNodeList();
  const auto& skeleton = shoot_model.shoot_skeleton_;
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
    shoot_growth_controller_ = {};
    shoot_growth_controller_.initialized_ = false;
  } else {
    shoot_growth_controller_.initialized_ = true;
    shoot_descriptor->PrepareController(shoot_growth_controller_);
  }
  const auto root_descriptor = td->root_descriptor.Get<IRootDescriptor>();
  if (!root_descriptor) {
    root_growth_controller_ = {};
    root_growth_controller_.initialized_ = false;
  } else {
    root_growth_controller_.initialized_ = true;
    root_descriptor->PrepareController(root_growth_controller_);
  }
  const auto pruning_descriptor = td->pruning_descriptor.Get<IPruningDescriptor>();
  if (!pruning_descriptor) {
    shoot_pruning_controller_ = {};
    shoot_pruning_controller_.initialized_ = false;
  } else {
    shoot_pruning_controller_.initialized_ = true;
    pruning_descriptor->PrepareController(simulation_settings, shoot_pruning_controller_);
  }
  const auto foliage_descriptor = td->foliage_descriptor.Get<IFoliageDescriptor>();
  if (!foliage_descriptor) {
    foliage_controller_ = {};
    foliage_controller_.initialized_ = false;
  } else {
    foliage_descriptor->PrepareController(foliage_controller_);
    foliage_controller_.initialized_ = true;
  }
  const auto reproduction_module_descriptor = td->reproduction_module_descriptor.Get<IReproductionModuleDescriptor>();
  if (!reproduction_module_descriptor) {
    shoot_reproduction_controller_ = {};
    shoot_reproduction_controller_.initialized_ = false;
  } else {
    shoot_reproduction_controller_.initialized_ = true;
    reproduction_module_descriptor->PrepareController(shoot_reproduction_controller_);
  }
}