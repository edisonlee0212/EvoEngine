//
// Created by lllll on 10/24/2022.
//
#include "Tree.hpp"
#include <Material.hpp>
#include <Mesh.hpp>
#include <TransformGraph.hpp>
#include "ShootDescriptor.hpp"
#include "SkeletonSerializer.hpp"
#include "StrandGroupSerializer.hpp"

#include "Application.hpp"
#include "BarkDescriptor.hpp"
#include "BillboardCloud.hpp"
#include "Climate.hpp"
#include "EcoSysLabLayer.hpp"
#include "EditorLayer.hpp"
#include "FoliageDescriptor.hpp"
#include "HeightField.hpp"
#include "Octree.hpp"
#include "Soil.hpp"
#include "StrandModelProfileSerializer.hpp"
#include "Strands.hpp"
#include "StrandsRenderer.hpp"
#include "TreeSkinnedMeshGenerator.hpp"
#include "RigidBody.hpp"
using namespace eco_sys_lab;
void Tree::SerializeTreeGrowthSettings(const TreeGrowthSettings& tree_growth_settings, YAML::Emitter& out) {
  out << YAML::Key << "node_developmental_vigor_filling_rate" << YAML::Value
      << tree_growth_settings.node_developmental_vigor_filling_rate;

  out << YAML::Key << "use_space_colonization" << YAML::Value << tree_growth_settings.use_space_colonization;
  out << YAML::Key << "space_colonization_auto_resize" << YAML::Value
      << tree_growth_settings.space_colonization_auto_resize;
  out << YAML::Key << "space_colonization_removal_distance_factor" << YAML::Value
      << tree_growth_settings.space_colonization_removal_distance_factor;
  out << YAML::Key << "space_colonization_detection_distance_factor" << YAML::Value
      << tree_growth_settings.space_colonization_detection_distance_factor;
  out << YAML::Key << "space_colonization_theta" << YAML::Value << tree_growth_settings.space_colonization_theta;
}
void Tree::DeserializeTreeGrowthSettings(TreeGrowthSettings& tree_growth_settings, const YAML::Node& in) {
  if (in["node_developmental_vigor_filling_rate"])
    tree_growth_settings.node_developmental_vigor_filling_rate =
        in["node_developmental_vigor_filling_rate"].as<float>();
  if (in["use_space_colonization"])
    tree_growth_settings.use_space_colonization = in["use_space_colonization"].as<bool>();
  if (in["space_colonization_auto_resize"])
    tree_growth_settings.space_colonization_auto_resize = in["space_colonization_auto_resize"].as<bool>();
  if (in["space_colonization_removal_distance_factor"])
    tree_growth_settings.space_colonization_removal_distance_factor =
        in["space_colonization_removal_distance_factor"].as<float>();
  if (in["space_colonization_detection_distance_factor"])
    tree_growth_settings.space_colonization_detection_distance_factor =
        in["space_colonization_detection_distance_factor"].as<float>();
  if (in["space_colonization_theta"])
    tree_growth_settings.space_colonization_theta = in["space_colonization_theta"].as<float>();
}

bool Tree::ParseBinvox(const std::filesystem::path& file_path, VoxelGrid<TreeOccupancyGridBasicData>& voxel_grid,
                       float voxel_size) {
  std::ifstream input(file_path, std::ios::in | std::ios::binary);
  if (!input.is_open()) {
    std::cout << "Error: could not open file " << file_path << std::endl;
    return false;
  }

  // Read header
  std::string line;
  input >> line;  // #binvox
  if (line.compare("#binvox") != 0) {
    std::cout << "Error: first line reads [" << line << "] instead of [#binvox]" << std::endl;
    return false;
  }
  int version;
  input >> version;
#ifndef NDEBUG
  std::cout << "reading binvox version " << version << std::endl;
#endif
  int depth, height, width;
  depth = -1;
  bool done = false;
  while (input.good() && !done) {
    input >> line;
    if (line.compare("data") == 0)
      done = true;
    else if (line.compare("dim") == 0) {
      input >> depth >> height >> width;
    } else {
#ifndef NDEBUG
      std::cout << "  unrecognized keyword [" << line << "], skipping" << std::endl;
#endif
      char c;
      do {  // skip until end of line
        c = input.get();
      } while (input.good() && (c != '\n'));
    }
  }

  if (!done) {
    std::cout << "  error reading header" << std::endl;
    return false;
  }
  if (depth == -1) {
    std::cout << "  missing dimensions in header" << std::endl;
    return false;
  }

  // Initialize the voxel grid based on the dimensions read
  glm::vec3 min_bound(0, 0, 0);  // Assuming starting from origin
  glm::ivec3 resolution(width, height, depth);
  voxel_grid.Initialize(voxel_size, resolution, min_bound,
                       {});  // Assuming voxelSize is globally defined or passed as an argument

  // Read voxel data
  unsigned char value;
  unsigned char count;
  int index = 0;
  int end_index = 0;
  int nr_voxels = 0;

  input.unsetf(std::ios::skipws);  // need to read every byte now (!)
  input >> value;                  // read the linefeed char
  glm::vec3 low_sum = glm::ivec3(0.0f);
  size_t low_sum_count = 0;
  while (end_index < width * height * depth && input.good()) {
    input >> value >> count;

    if (input.good()) {
      end_index = index + count;
      if (end_index > (width * height * depth))
        return false;

      for (int i = index; i < end_index; i++) {
        // Convert 1D index to 3D coordinates
        const int x = (i / width) % height;
        const int y = i % width;
        const int z = i / (width * height);

        if (value) {
          voxel_grid.Ref(glm::ivec3(x, y, z)).occupied = true;
          nr_voxels++;

          if (y < (height * 0.2f)) {
            low_sum += voxel_grid.GetPosition(glm::ivec3(x, y, z));
            low_sum_count++;
          }
        }
      }

      index = end_index;
    }
  }
  low_sum /= low_sum_count;
  voxel_grid.ShiftMinBound(-glm::vec3(low_sum.x, 0, low_sum.z));

  input.close();
#ifndef NDEBUG
  std::cout << "  read " << nr_voxels << " voxels" << std::endl;
#endif
  return true;
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

void Tree::ClearSkeletalGraph() const {
  const auto scene = GetScene();
  const auto self = GetOwner();
  const auto children = scene->GetChildren(self);
  for (const auto& child : children) {
    auto name = scene->GetEntityName(child);
    if (name == "Skeletal Graph Lines") {
      scene->DeleteEntity(child);
    } else if (name == "Skeletal Graph Points") {
      scene->DeleteEntity(child);
    }
  }
}

void Tree::GenerateSkeletalGraph(const SkeletalGraphSettings& skeletal_graph_settings, SkeletonNodeHandle base_node_handle,
                                 const std::shared_ptr<Mesh>& point_mesh_sample,
                                 const std::shared_ptr<Mesh>& line_mesh_sample) const {
  const auto scene = GetScene();
  const auto self = GetOwner();
  ClearSkeletalGraph();

  const auto line_entity = scene->CreateEntity("Skeletal Graph Lines");
  scene->SetParent(line_entity, self);

  const auto point_entity = scene->CreateEntity("Skeletal Graph Points");
  scene->SetParent(point_entity, self);

  bool strand_ready = false;
  if (strand_model.strand_model_skeleton.PeekSortedNodeList().size() > 1) {
    strand_ready = true;
  }

  const auto line_list = ProjectManager::CreateTemporaryAsset<ParticleInfoList>();
  const auto line_material = ProjectManager::CreateTemporaryAsset<Material>();
  const std::shared_ptr<Particles> line_particles = scene->GetOrSetPrivateComponent<Particles>(line_entity).lock();
  line_particles->mesh = line_mesh_sample;
  line_particles->material = line_material;
  line_particles->particle_info_list = line_list;
  line_material->vertex_color_only = true;
  const auto point_list = ProjectManager::CreateTemporaryAsset<ParticleInfoList>();
  const auto point_material = ProjectManager::CreateTemporaryAsset<Material>();
  const std::shared_ptr<Particles> point_particles = scene->GetOrSetPrivateComponent<Particles>(point_entity).lock();
  point_particles->mesh = point_mesh_sample;
  point_particles->material = point_material;
  point_particles->particle_info_list = point_list;
  point_material->vertex_color_only = true;

  std::vector<ParticleInfo> line_particle_infos;
  std::vector<ParticleInfo> point_particle_infos;
  const int node_size = strand_ready ? strand_model.strand_model_skeleton.PeekSortedNodeList().size()
                                   : tree_model.PeekShootSkeleton().PeekSortedNodeList().size();
  if (strand_ready) {
    line_particle_infos.resize(node_size);
    point_particle_infos.resize(node_size);
  } else {
    line_particle_infos.resize(node_size);
    point_particle_infos.resize(node_size);
  }
  Jobs::RunParallelFor(node_size, [&](unsigned internode_index) {
    if (strand_ready) {
      const auto& sorted_internode_list = strand_model.strand_model_skeleton.PeekSortedNodeList();
      const auto internode_handle = sorted_internode_list[internode_index];
      SkeletonNodeHandle walker = internode_handle;
      bool sub_tree = false;
      const auto& skeleton = strand_model.strand_model_skeleton;
      const auto& node = skeleton.PeekNode(internode_handle);

      while (walker != -1) {
        if (walker == base_node_handle) {
          sub_tree = true;
          break;
        }
        walker = skeleton.PeekNode(walker).GetParentHandle();
      }
      const glm::vec3 position = node.info.global_position;
      auto rotation = node.info.global_rotation;
      {
        rotation *= glm::quat(glm::vec3(glm::radians(90.0f), 0.0f, 0.0f));
        const glm::mat4 rotation_transform = glm::mat4_cast(rotation);
        line_particle_infos[internode_index].instance_matrix.value =
            glm::translate(position + (node.info.length / 2.0f) * node.info.GetGlobalDirection()) *
            rotation_transform *
            glm::scale(glm::vec3(skeletal_graph_settings.fixed_line_thickness * (sub_tree ? 1.25f : 1.0f),
                                 node.info.length,
                                 skeletal_graph_settings.fixed_line_thickness * (sub_tree ? 1.25f : 1.0f)));

        if (sub_tree) {
          line_particle_infos[internode_index].instance_color = skeletal_graph_settings.line_focus_color;
        } else {
          line_particle_infos[internode_index].instance_color = skeletal_graph_settings.line_color;
        }
      }
      {
        rotation *= glm::quat(glm::vec3(glm::radians(90.0f), 0.0f, 0.0f));
        const glm::mat4 rotation_transform = glm::mat4_cast(rotation);
        float thickness_factor = node.info.thickness;
        if (skeletal_graph_settings.fixed_point_size)
          thickness_factor = skeletal_graph_settings.fixed_point_size_factor;
        auto scale = glm::vec3(skeletal_graph_settings.branch_point_size * thickness_factor);
        point_particle_infos[internode_index].instance_color = skeletal_graph_settings.branch_point_color;
        if (internode_index == 0 || node.PeekChildHandles().size() > 1) {
          scale = glm::vec3(skeletal_graph_settings.junction_point_size * thickness_factor);
          point_particle_infos[internode_index].instance_color = skeletal_graph_settings.junction_point_color;
        }
        point_particle_infos[internode_index].instance_matrix.value =
            glm::translate(position) * rotation_transform * glm::scale(scale * (sub_tree ? 1.25f : 1.0f));
        if (sub_tree) {
          point_particle_infos[internode_index].instance_color = skeletal_graph_settings.branch_focus_color;
        }
      }
    } else {
      const auto& sorted_internode_list = tree_model.PeekShootSkeleton().PeekSortedNodeList();
      const auto internode_handle = sorted_internode_list[internode_index];
      SkeletonNodeHandle walker = internode_handle;
      bool sub_tree = false;
      const auto& skeleton = tree_model.PeekShootSkeleton();
      const auto& node = skeleton.PeekNode(internode_handle);

      while (walker != -1) {
        if (walker == base_node_handle) {
          sub_tree = true;
          break;
        }
        walker = skeleton.PeekNode(walker).GetParentHandle();
      }
      const glm::vec3 position = node.info.global_position;
      auto rotation = node.info.global_rotation;
      {
        rotation *= glm::quat(glm::vec3(glm::radians(90.0f), 0.0f, 0.0f));
        const glm::mat4 rotation_transform = glm::mat4_cast(rotation);
        line_particle_infos[internode_index].instance_matrix.value =
            glm::translate(position + (node.info.length / 2.0f) * node.info.GetGlobalDirection()) *
            rotation_transform *
            glm::scale(glm::vec3(skeletal_graph_settings.fixed_line_thickness * (sub_tree ? 1.25f : 1.0f),
                                 node.info.length,
                                 skeletal_graph_settings.fixed_line_thickness * (sub_tree ? 1.25f : 1.0f)));

        if (sub_tree) {
          line_particle_infos[internode_index].instance_color = skeletal_graph_settings.line_focus_color;
        } else {
          line_particle_infos[internode_index].instance_color = skeletal_graph_settings.line_color;
        }
      }
      {
        rotation *= glm::quat(glm::vec3(glm::radians(90.0f), 0.0f, 0.0f));
        const glm::mat4 rotation_transform = glm::mat4_cast(rotation);
        float thickness_factor = node.info.thickness;
        if (skeletal_graph_settings.fixed_point_size)
          thickness_factor = skeletal_graph_settings.fixed_point_size_factor;
        auto scale = glm::vec3(skeletal_graph_settings.branch_point_size * thickness_factor);
        point_particle_infos[internode_index].instance_color = skeletal_graph_settings.branch_point_color;
        if (internode_index == 0 || node.PeekChildHandles().size() > 1) {
          scale = glm::vec3(skeletal_graph_settings.junction_point_size * thickness_factor);
          point_particle_infos[internode_index].instance_color = skeletal_graph_settings.junction_point_color;
        }
        point_particle_infos[internode_index].instance_matrix.value =
            glm::translate(position) * rotation_transform * glm::scale(scale * (sub_tree ? 1.25f : 1.0f));
        if (sub_tree) {
          point_particle_infos[internode_index].instance_color = skeletal_graph_settings.branch_focus_color;
        }
      }
    }
  });
  line_list->SetParticleInfos(line_particle_infos);
  point_list->SetParticleInfos(point_particle_infos);
}

bool Tree::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  static BillboardCloud::GenerateSettings foliage_billboard_cloud_generate_settings{};

  foliage_billboard_cloud_generate_settings.OnInspect("Foliage billboard cloud settings");

  if (ImGui::Button("Generate billboard")) {
    GenerateBillboardClouds(foliage_billboard_cloud_generate_settings);
  }

  bool changed = false;
  const auto eco_sys_lab_layer = Application::GetLayer<EcoSysLabLayer>();
  const auto scene = GetScene();
  editor_layer->DragAndDropButton<TreeDescriptor>(tree_descriptor, "TreeDescriptor", true);
  static bool show_space_colonization_grid = true;

  static std::shared_ptr<ParticleInfoList> space_colonization_grid_particle_info_list;
  if (!space_colonization_grid_particle_info_list) {
    space_colonization_grid_particle_info_list = ProjectManager::CreateTemporaryAsset<ParticleInfoList>();
  }

  if (const auto td = tree_descriptor.Get<TreeDescriptor>()) {
    const auto sd = td->shoot_descriptor.Get<ShootDescriptor>();
    if (sd) {
      /*
      ImGui::DragInt("Seed", &tree_model.m_seed, 1, 0);
      if (ImGui::Button("Reset")) {
              Reset();
              modelChanged = true;
      }*/
      if (ImGui::TreeNode("Tree settings")) {
        if (ImGui::DragFloat("Low Branch Pruning", &low_branch_pruning, 0.01f, 0.0f, 1.f))
          changed = true;
        if (ImGui::DragFloat("Crown shyness distance", &crown_shyness_distance, 0.01f, 0.0f, 1.f))
          changed = true;
        if (ImGui::DragFloat("Start time", &start_time, 0.01f, 0.0f, 100.f))
          changed = true;
        ImGui::Checkbox("Enable History", &enable_history);
        if (enable_history) {
          ImGui::DragInt("History per iteration", &history_iteration, 1, 1, 1000);
        }
        if (ImGui::TreeNode("Sagging")) {
          bool bending_changed = false;
          bending_changed = ImGui::DragFloat("Bending strength", &sd->gravity_bending_strength, 0.01f, 0.0f,
                                            1.0f, "%.3f") ||
                           bending_changed;
          bending_changed =
              ImGui::DragFloat("Bending thickness factor", &sd->gravity_bending_thickness_factor, 0.1f,
                               0.0f, 10.f, "%.3f") ||
              bending_changed;
          bending_changed = ImGui::DragFloat("Bending angle factor", &sd->gravity_bending_max, 0.01f, 0.0f,
                                            1.0f, "%.3f") ||
                           bending_changed;
          if (bending_changed) {
            shoot_growth_controller_.m_sagging = [=](const SkeletonNode<InternodeGrowthData>& internode) {
              float strength = internode.data.sagging_force * sd->gravity_bending_strength /
                               glm::pow(internode.info.thickness / sd->end_node_thickness,
                                        sd->gravity_bending_thickness_factor);
              strength = sd->gravity_bending_max * (1.f - glm::exp(-glm::abs(strength)));
              return strength;
            };
            tree_model.CalculateTransform(shoot_growth_controller_, true);
            tree_visualizer.m_needUpdate = true;
          }
        }
        OnInspectTreeGrowthSettings(tree_model.tree_growth_settings);

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
                      sd->internode_length,
                      tree_model.tree_growth_settings.space_colonization_removal_distance_factor,
                      tree_model.tree_growth_settings.space_colonization_theta,
                      tree_model.tree_growth_settings.space_colonization_detection_distance_factor, markers_per_voxel);
                }
              },
              false);

          static PrivateComponentRef private_component_ref{};

          if (editor_layer->DragAndDropButton<MeshRenderer>(private_component_ref, "Add Obstacle")) {
            if (const auto mmr = private_component_ref.Get<MeshRenderer>()) {
              const auto cube_volume = ProjectManager::CreateTemporaryAsset<CubeVolume>();
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
      static bool enable_physics = true;
      ImGui::Checkbox("Enable Physics", &enable_physics);
      if (enable_physics) {
        branch_physics_parameters.OnInspect();
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
        editor_layer->DrawGizmoMeshInstancedColored(Resources::GetResource<Mesh>("PRIMITIVE_CUBE"),
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
    if (ImGui::TreeNodeEx("Profile settings", ImGuiTreeNodeFlags_DefaultOpen)) {
      if (ImGui::TreeNode("Physics settings")) {
        ImGui::DragFloat("Physics damping", &strand_model_parameters.profile_physics_settings.damping, 0.01f, 0.0f, 1.0f);
        ImGui::DragFloat("Physics max speed", &strand_model_parameters.profile_physics_settings.max_speed, 0.01f, 0.0f,
                         100.0f);
        ImGui::DragFloat("Physics particle softness", &strand_model_parameters.profile_physics_settings.particle_softness,
                         0.01f, 0.0f, 1.0f);
        ImGui::TreePop();
      }
      ImGui::DragFloat("Center attraction strength", &strand_model_parameters.center_attraction_strength, 100.f, 0.0f,
                       10000.0f);
      ImGui::DragInt("Max iteration cell factor", &strand_model_parameters.max_simulation_iteration_cell_factor, 1, 0, 500);
      ImGui::DragInt("Branch Packing Timeout", &strand_model_parameters.branch_profile_packing_max_iteration, 1, 0, 10000);
      ImGui::DragInt("Junction Packing Timeout", &strand_model_parameters.junction_profile_packing_max_iteration, 1, 20,
                     10000);
      ImGui::DragInt("Modified Packing Timeout", &strand_model_parameters.modified_profile_packing_max_iteration, 1, 20,
                     10000);
      ImGui::DragInt("Timeout with boundaries)", &strand_model_parameters.modified_profile_packing_max_iteration, 1, 20,
                     10000);
      ImGui::TreePop();
    }
    ImGui::DragFloat("Overlap threshold", &strand_model_parameters.overlap_threshold, 0.01f, 0.0f, 1.0f);
    ImGui::DragInt("Initial branch strand count", &strand_model_parameters.strands_along_branch, 1, 1, 50);
    ImGui::DragInt("Initial end node strand count", &strand_model_parameters.end_node_strands, 1, 1, 50);

    ImGui::Checkbox("Pre-merge", &strand_model_parameters.pre_merge);

    static PlottedDistributionSettings plotted_distribution_settings = {
        0.001f, {0.001f, true, true, ""}, {0.001f, true, true, ""}, ""};
    strand_model_parameters.branch_twist_distribution.OnInspect("Branch Twist", plotted_distribution_settings);
    strand_model_parameters.junction_twist_distribution.OnInspect("Junction Twist", plotted_distribution_settings);
    strand_model_parameters.strand_radius_distribution.OnInspect("Strand Thickness", plotted_distribution_settings);

    ImGui::DragFloat("Cladoptosis Range", &strand_model_parameters.cladoptosis_range, 0.01f, 0.0f, 50.f);
    strand_model_parameters.cladoptosis_distribution.OnInspect("Cladoptosis", plotted_distribution_settings);
    ImGui::Text(("Strand count: " +
                 std::to_string(strand_model.strand_model_skeleton.data.strand_group.PeekStrands().size()))
                    .c_str());
    ImGui::Text(("Total particle count: " + std::to_string(strand_model.strand_model_skeleton.data.num_of_particles))
                    .c_str());

    if (ImGui::TreeNodeEx("Graph Adjustment settings", ImGuiTreeNodeFlags_DefaultOpen)) {
      ImGui::DragFloat("Side factor", &strand_model_parameters.side_push_factor, 0.01f, 0.0f, 2.0f);
      ImGui::DragFloat("Apical Side factor", &strand_model_parameters.apical_side_push_factor, 0.01f, 0.0f, 2.0f);
      ImGui::DragFloat("Rotation factor", &strand_model_parameters.rotation_push_factor, 0.01f, 0.0f, 2.0f);
      ImGui::DragFloat("Apical Rotation factor", &strand_model_parameters.apical_branch_rotation_push_factor, 0.01f, 0.0f,
                       2.0f);
      ImGui::TreePop();
    }
    ImGui::DragInt("Max node count", &strand_model_parameters.node_max_count, 1, -1, 999);
    ImGui::DragInt("Boundary point distance", &strand_model_parameters.boundary_point_distance, 1, 3, 30);
    ImGui::ColorEdit4("Boundary color", &strand_model_parameters.boundary_point_color.x);
    ImGui::ColorEdit4("Content color", &strand_model_parameters.content_point_color.x);

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
  if (ImGui::Button("Build Strand Mesh")) {
    InitializeStrandModelMeshRenderer(strand_model_mesh_generator_settings);
  }
  ImGui::SameLine();
  if (ImGui::Button("Clear Strand Mesh")) {
    ClearStrandModelMeshRenderer();
  }

  tree_visualizer.Visualize(strand_model);
  if (ImGui::TreeNode("Skeletal graph settings")) {
    skeletal_graph_settings.OnInspect();
  }
  if (ImGui::Button("Build skeletal graph")) {
    GenerateSkeletalGraph(skeletal_graph_settings, -1, Resources::GetResource<Mesh>("PRIMITIVE_SPHERE"),
                          Resources::GetResource<Mesh>("PRIMITIVE_CUBE"));
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

  tree_descriptor.Clear();
  soil.Clear();
  climate.Clear();
  enable_history = false;

  tree_visualizer.Clear();

  left_side_biomass = right_side_biomass = 0.0f;
  root_biomass_history.clear();
  shoot_biomass_history.clear();

  generate_mesh = true;
  low_branch_pruning = 0.f;
  crown_shyness_distance = 0.f;
  start_time = 0.f;
}

bool Tree::OnInspectTreeGrowthSettings(TreeGrowthSettings& tree_growth_settings) {
  bool changed = false;
  if (ImGui::Checkbox("Enable space colonization", &tree_growth_settings.use_space_colonization))
    changed = true;
  if (tree_growth_settings.use_space_colonization) {
    if (ImGui::Checkbox("Space colonization auto resize", &tree_growth_settings.space_colonization_auto_resize))
      changed = true;
  }

  return changed;
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
  output += "], Strand count: [" +
            std::to_string(strand_model.strand_model_skeleton.data.strand_group.PeekStrands().size());
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

std::shared_ptr<Strands> Tree::GenerateStrands() const {
  const auto strands_asset = ProjectManager::CreateTemporaryAsset<Strands>();
  const auto& parameters = strand_model_parameters;
  std::vector<glm::uint> strands_list;
  std::vector<StrandPoint> points;
  strand_model.strand_model_skeleton.data.strand_group.BuildStrands(strands_list, points, parameters.node_max_count);
  if (!points.empty())
    strands_list.emplace_back(points.size());
  StrandPointAttributes strand_point_attributes{};
  strand_point_attributes.color = true;
  strands_asset->SetStrands(strand_point_attributes, strands_list, points);
  return strands_asset;
}

void Tree::GenerateTrunkMeshes(const std::shared_ptr<Mesh>& trunk_mesh,
                               const TreeMeshGeneratorSettings& mesh_generator_settings) {
  const auto& sorted_internode_list = tree_model.RefShootSkeleton().PeekSortedNodeList();
  std::unordered_set<SkeletonNodeHandle> trunk_handles{};
  for (const auto& node_handle : sorted_internode_list) {
    const auto& node = tree_model.RefShootSkeleton().PeekNode(node_handle);
    trunk_handles.insert(node_handle);
    if (node.PeekChildHandles().size() > 1)
      break;
  }
  {
    std::vector<Vertex> vertices;
    std::vector<unsigned int> indices;
    const auto td = tree_descriptor.Get<TreeDescriptor>();
    std::shared_ptr<BarkDescriptor> bd{};
    if (td) {
      bd = td->bark_descriptor.Get<BarkDescriptor>();
    }
    CylindricalMeshGenerator<ShootGrowthData, ShootStemGrowthData, InternodeGrowthData>::GeneratePartially(
        trunk_handles, tree_model.PeekShootSkeleton(), vertices, indices, mesh_generator_settings,
        [&](glm::vec3& vertex_position, const glm::vec3& direction, const float x_factor, const float y_factor) {
          if (bd) {
            const float push_value = bd->GetValue(x_factor, y_factor);
            vertex_position += push_value * direction;
          }
        },
        [&](glm::vec2&, float, float) {
        });
    VertexAttributes attributes{};
    attributes.tex_coord = true;
    trunk_mesh->SetVertices(attributes, vertices, indices);
  }
}

std::shared_ptr<Mesh> Tree::GenerateBranchMesh(const TreeMeshGeneratorSettings& mesh_generator_settings) {
  std::vector<Vertex> vertices;
  std::vector<unsigned int> indices;
  if (mesh_generator_settings.branch_mesh_type == 0) {
    auto td = tree_descriptor.Get<TreeDescriptor>();
    if (!td) {
      EVOENGINE_WARNING("TreeDescriptor missing!");
      td = ProjectManager::CreateTemporaryAsset<TreeDescriptor>();
      td->foliage_descriptor = ProjectManager::CreateTemporaryAsset<FoliageDescriptor>();
    }
    std::shared_ptr<BarkDescriptor> bd{};
    bd = td->bark_descriptor.Get<BarkDescriptor>();
    if (strand_model.strand_model_skeleton.RefRawNodes().size() == tree_model.shoot_skeleton_.RefRawNodes().size()) {
      CylindricalMeshGenerator<StrandModelSkeletonData, StrandModelFlowData, StrandModelNodeData>::Generate(
          strand_model.strand_model_skeleton, vertices, indices, mesh_generator_settings,
          [&](glm::vec3& vertex_position, const glm::vec3& direction, const float x_factor, const float y_factor) {
            if (bd) {
              const float push_value = bd->GetValue(x_factor, y_factor);
              vertex_position += push_value * direction;
            }
          },
          [&](glm::vec2&, float, float) {
          });
    } else {
      CylindricalMeshGenerator<ShootGrowthData, ShootStemGrowthData, InternodeGrowthData>::Generate(
          tree_model.PeekShootSkeleton(), vertices, indices, mesh_generator_settings,
          [&](glm::vec3& vertex_position, const glm::vec3& direction, const float x_factor, const float y_factor) {
            if (bd) {
              const float push_value = bd->GetValue(x_factor, y_factor);
              vertex_position += push_value * direction;
            }
          },
          [&](glm::vec2&, float, float) {
          });
    }
  } else {
    auto td = tree_descriptor.Get<TreeDescriptor>();
    if (!td) {
      EVOENGINE_WARNING("TreeDescriptor missing!");
      td = ProjectManager::CreateTemporaryAsset<TreeDescriptor>();
      td->foliage_descriptor = ProjectManager::CreateTemporaryAsset<FoliageDescriptor>();
    }
    VoxelMeshGenerator<ShootGrowthData, ShootStemGrowthData, InternodeGrowthData>::Generate(
        tree_model.PeekShootSkeleton(), vertices, indices, mesh_generator_settings);
  }
  auto mesh = ProjectManager::CreateTemporaryAsset<Mesh>();
  VertexAttributes attributes{};
  attributes.tex_coord = true;
  mesh->SetVertices(attributes, vertices, indices);
  return mesh;
}

std::shared_ptr<Mesh> Tree::GenerateFoliageMesh(const TreeMeshGeneratorSettings& mesh_generator_settings) {
  std::vector<Vertex> vertices;
  std::vector<unsigned int> indices;

  auto quad_mesh = Resources::GetResource<Mesh>("PRIMITIVE_QUAD");
  auto& quad_triangles = quad_mesh->UnsafeGetTriangles();
  size_t quad_vertices_size;
  quad_vertices_size = quad_mesh->GetVerticesAmount();
  size_t offset = 0;
  auto td = tree_descriptor.Get<TreeDescriptor>();
  if (!td) {
    EVOENGINE_WARNING("TreeDescriptor missing!");
    td = ProjectManager::CreateTemporaryAsset<TreeDescriptor>();
    td->foliage_descriptor = ProjectManager::CreateTemporaryAsset<FoliageDescriptor>();
  }
  auto fd = td->foliage_descriptor.Get<FoliageDescriptor>();
  if (!fd)
    fd = ProjectManager::CreateTemporaryAsset<FoliageDescriptor>();
  const auto tree_dim = tree_model.PeekShootSkeleton().max - tree_model.PeekShootSkeleton().min;

  const auto& node_list = tree_model.PeekShootSkeleton().PeekSortedNodeList();
  for (const auto& internode_handle : node_list) {
    const auto& internode_info = tree_model.PeekShootSkeleton().PeekNode(internode_handle).info;
    std::vector<glm::mat4> leaf_matrices;
    fd->GenerateFoliageMatrices(leaf_matrices, internode_info, glm::length(tree_dim));
    Vertex archetype;
    for (const auto& matrix : leaf_matrices) {
      for (auto i = 0; i < quad_mesh->GetVerticesAmount(); i++) {
        archetype.position = matrix * glm::vec4(quad_mesh->UnsafeGetVertices()[i].position, 1.0f);
        archetype.normal =
            glm::normalize(glm::vec3(matrix * glm::vec4(quad_mesh->UnsafeGetVertices()[i].normal, 0.0f)));
        archetype.tangent =
            glm::normalize(glm::vec3(matrix * glm::vec4(quad_mesh->UnsafeGetVertices()[i].tangent, 0.0f)));
        archetype.tex_coord = quad_mesh->UnsafeGetVertices()[i].tex_coord;
        archetype.color = internode_info.color;
        vertices.push_back(archetype);
      }
      for (auto triangle : quad_triangles) {
        triangle.x += offset;
        triangle.y += offset;
        triangle.z += offset;
        indices.push_back(triangle.x);
        indices.push_back(triangle.y);
        indices.push_back(triangle.z);
      }

      offset += quad_vertices_size;

      for (auto i = 0; i < quad_mesh->GetVerticesAmount(); i++) {
        archetype.position = matrix * glm::vec4(quad_mesh->UnsafeGetVertices()[i].position, 1.0f);
        archetype.normal =
            glm::normalize(glm::vec3(matrix * glm::vec4(quad_mesh->UnsafeGetVertices()[i].normal, 0.0f)));
        archetype.tangent =
            glm::normalize(glm::vec3(matrix * glm::vec4(quad_mesh->UnsafeGetVertices()[i].tangent, 0.0f)));
        archetype.tex_coord = quad_mesh->UnsafeGetVertices()[i].tex_coord;
        vertices.push_back(archetype);
      }
      for (auto triangle : quad_triangles) {
        triangle.x += offset;
        triangle.y += offset;
        triangle.z += offset;
        indices.push_back(triangle.z);
        indices.push_back(triangle.y);
        indices.push_back(triangle.x);
      }
      offset += quad_vertices_size;
    }
  }

  auto mesh = ProjectManager::CreateTemporaryAsset<Mesh>();
  VertexAttributes attributes{};
  attributes.tex_coord = true;
  mesh->SetVertices(attributes, vertices, indices);
  return mesh;
}

std::shared_ptr<ParticleInfoList> Tree::GenerateFoliageParticleInfoList(
    const TreeMeshGeneratorSettings& mesh_generator_settings) {
  auto td = tree_descriptor.Get<TreeDescriptor>();
  if (!td) {
    EVOENGINE_WARNING("TreeDescriptor missing!");
    td = ProjectManager::CreateTemporaryAsset<TreeDescriptor>();
    td->foliage_descriptor = ProjectManager::CreateTemporaryAsset<FoliageDescriptor>();
  }
  const auto ret_val = ProjectManager::CreateTemporaryAsset<ParticleInfoList>();
  auto fd = td->foliage_descriptor.Get<FoliageDescriptor>();
  if (!fd)
    fd = ProjectManager::CreateTemporaryAsset<FoliageDescriptor>();
  std::vector<ParticleInfo> particle_infos;
  const auto& node_list = tree_model.PeekShootSkeleton().PeekSortedNodeList();
  const bool sm =
      strand_model.strand_model_skeleton.RefRawNodes().size() == tree_model.shoot_skeleton_.RefRawNodes().size();
  const auto tree_dim = sm ? strand_model.strand_model_skeleton.max - strand_model.strand_model_skeleton.min
                                   : tree_model.PeekShootSkeleton().max - tree_model.PeekShootSkeleton().min;

  for (const auto& internode_handle : node_list) {
    const auto& internode_info = sm ? strand_model.strand_model_skeleton.PeekNode(internode_handle).info
                                            : tree_model.PeekShootSkeleton().PeekNode(internode_handle).info;

    std::vector<glm::mat4> leaf_matrices{};
    fd->GenerateFoliageMatrices(leaf_matrices, internode_info, glm::length(tree_dim));
    const auto start_index = particle_infos.size();
    particle_infos.resize(start_index + leaf_matrices.size());
    for (int i = 0; i < leaf_matrices.size(); i++) {
      auto& particle_info = particle_infos.at(start_index + i);
      particle_info.instance_matrix.value = leaf_matrices.at(i);
      particle_info.instance_color = internode_info.color;
    }
  }
  ret_val->SetParticleInfos(particle_infos);
  return ret_val;
}

std::shared_ptr<Mesh> Tree::GenerateStrandModelFoliageMesh(
    const StrandModelMeshGeneratorSettings& strand_model_mesh_generator_settings) {
  std::vector<Vertex> vertices;
  std::vector<unsigned int> indices;

  auto quad_mesh = Resources::GetResource<Mesh>("PRIMITIVE_QUAD");
  auto& quad_triangles = quad_mesh->UnsafeGetTriangles();
  auto quad_vertices_size = quad_mesh->GetVerticesAmount();
  size_t offset = 0;
  auto td = tree_descriptor.Get<TreeDescriptor>();
  if (!td)
    return nullptr;
  auto fd = td->foliage_descriptor.Get<FoliageDescriptor>();
  if (!fd)
    fd = ProjectManager::CreateTemporaryAsset<FoliageDescriptor>();
  const auto& node_list = strand_model.strand_model_skeleton.PeekSortedNodeList();
  const auto tree_dim = strand_model.strand_model_skeleton.max - strand_model.strand_model_skeleton.min;
  for (const auto& internode_handle : node_list) {
    const auto& strand_model_node = strand_model.strand_model_skeleton.PeekNode(internode_handle);
    std::vector<glm::mat4> leaf_matrices;
    fd->GenerateFoliageMatrices(leaf_matrices, strand_model_node.info, glm::length(tree_dim));
    Vertex archetype;
    for (const auto& matrix : leaf_matrices) {
      for (auto i = 0; i < quad_mesh->GetVerticesAmount(); i++) {
        archetype.position = matrix * glm::vec4(quad_mesh->UnsafeGetVertices()[i].position, 1.0f);
        archetype.normal =
            glm::normalize(glm::vec3(matrix * glm::vec4(quad_mesh->UnsafeGetVertices()[i].normal, 0.0f)));
        archetype.tangent =
            glm::normalize(glm::vec3(matrix * glm::vec4(quad_mesh->UnsafeGetVertices()[i].tangent, 0.0f)));
        archetype.tex_coord = quad_mesh->UnsafeGetVertices()[i].tex_coord;
        archetype.color = strand_model_node.info.color;
        vertices.push_back(archetype);
      }
      for (auto triangle : quad_triangles) {
        triangle.x += offset;
        triangle.y += offset;
        triangle.z += offset;
        indices.push_back(triangle.x);
        indices.push_back(triangle.y);
        indices.push_back(triangle.z);
      }

      offset += quad_vertices_size;

      for (auto i = 0; i < quad_mesh->GetVerticesAmount(); i++) {
        archetype.position = matrix * glm::vec4(quad_mesh->UnsafeGetVertices()[i].position, 1.0f);
        archetype.normal =
            glm::normalize(glm::vec3(matrix * glm::vec4(quad_mesh->UnsafeGetVertices()[i].normal, 0.0f)));
        archetype.tangent =
            glm::normalize(glm::vec3(matrix * glm::vec4(quad_mesh->UnsafeGetVertices()[i].tangent, 0.0f)));
        archetype.tex_coord = quad_mesh->UnsafeGetVertices()[i].tex_coord;
        vertices.push_back(archetype);
      }
      for (auto triangle : quad_triangles) {
        triangle.x += offset;
        triangle.y += offset;
        triangle.z += offset;
        indices.push_back(triangle.z);
        indices.push_back(triangle.y);
        indices.push_back(triangle.x);
      }
      offset += quad_vertices_size;
    }
  }

  auto mesh = ProjectManager::CreateTemporaryAsset<Mesh>();
  VertexAttributes attributes{};
  attributes.tex_coord = true;
  mesh->SetVertices(attributes, vertices, indices);
  return mesh;
}

std::shared_ptr<Mesh> Tree::GenerateStrandModelBranchMesh(
    const StrandModelMeshGeneratorSettings& strand_model_mesh_generator_settings) const {
  std::vector<Vertex> vertices;
  std::vector<unsigned int> indices;
  StrandModelMeshGenerator::Generate(strand_model, vertices, indices, strand_model_mesh_generator_settings);

  auto mesh = ProjectManager::CreateTemporaryAsset<Mesh>();
  VertexAttributes attributes{};
  attributes.tex_coord = true;
  mesh->SetVertices(attributes, vertices, indices);
  return mesh;
}

void Tree::ExportObj(const std::filesystem::path& path, const TreeMeshGeneratorSettings& mesh_generator_settings) {
  if (path.extension() == ".obj") {
    try {
      std::ofstream of;
      of.open(path.string(), std::ofstream::out | std::ofstream::trunc);
      if (of.is_open()) {
        std::string start = "#Forest OBJ exporter, by Bosheng Li";
        start += "\n";
        of.write(start.c_str(), start.size());
        of.flush();
        unsigned start_index = 1;
        if (mesh_generator_settings.enable_branch) {
          if (const auto branch_mesh = GenerateBranchMesh(mesh_generator_settings)) {
            auto& vertices = branch_mesh->UnsafeGetVertices();
            auto& triangles = branch_mesh->UnsafeGetTriangles();
            if (!vertices.empty() && !triangles.empty()) {
              std::string header =
                  "#Vertices: " + std::to_string(vertices.size()) + ", tris: " + std::to_string(triangles.size());
              header += "\n";
              of.write(header.c_str(), header.size());
              of.flush();
              std::stringstream data;
              data << "o branch " + std::to_string(0) + "\n";
#pragma region Data collection
              for (auto i = 0; i < vertices.size(); i++) {
                auto& vertex_position = vertices.at(i).position;
                auto& color = vertices.at(i).color;
                data << "v " + std::to_string(vertex_position.x) + " " + std::to_string(vertex_position.y) + " " +
                            std::to_string(vertex_position.z) + " " + std::to_string(color.x) + " " +
                            std::to_string(color.y) + " " + std::to_string(color.z) + "\n";
              }
              for (const auto& vertex : vertices) {
                data << "vt " + std::to_string(vertex.tex_coord.x) + " " + std::to_string(vertex.tex_coord.y) + "\n";
              }
              // data += "s off\n";
              data << "# List of indices for faces vertices, with (x, y, z).\n";
              for (auto i = 0; i < triangles.size(); i++) {
                const auto triangle = triangles[i];
                const auto f1 = triangle.x + start_index;
                const auto f2 = triangle.y + start_index;
                const auto f3 = triangle.z + start_index;
                data << "f " + std::to_string(f1) + "/" + std::to_string(f1) + "/" + std::to_string(f1) + " " +
                            std::to_string(f2) + "/" + std::to_string(f2) + "/" + std::to_string(f2) + " " +
                            std::to_string(f3) + "/" + std::to_string(f3) + "/" + std::to_string(f3) + "\n";
              }
#pragma endregion
              const auto result = data.str();
              of.write(result.c_str(), result.size());
              of.flush();
              start_index += vertices.size();
            }
          }
        }
        if (mesh_generator_settings.enable_foliage) {
          if (const auto foliage_mesh = GenerateFoliageMesh(mesh_generator_settings)) {
            auto& vertices = foliage_mesh->UnsafeGetVertices();
            auto& triangles = foliage_mesh->UnsafeGetTriangles();
            if (!vertices.empty() && !triangles.empty()) {
              std::string header =
                  "#Vertices: " + std::to_string(vertices.size()) + ", tris: " + std::to_string(triangles.size());
              header += "\n";
              of.write(header.c_str(), header.size());
              of.flush();
              std::stringstream data;
              data << "o foliage " + std::to_string(0) + "\n";
#pragma region Data collection
              for (auto i = 0; i < vertices.size(); i++) {
                auto& vertex_position = vertices.at(i).position;
                auto& color = vertices.at(i).color;
                data << "v " + std::to_string(vertex_position.x) + " " + std::to_string(vertex_position.y) + " " +
                            std::to_string(vertex_position.z) + " " + std::to_string(color.x) + " " +
                            std::to_string(color.y) + " " + std::to_string(color.z) + "\n";
              }
              for (const auto& vertex : vertices) {
                data << "vt " + std::to_string(vertex.tex_coord.x) + " " + std::to_string(vertex.tex_coord.y) + "\n";
              }
              // data += "s off\n";
              data << "# List of indices for faces vertices, with (x, y, z).\n";
              for (auto i = 0; i < triangles.size(); i++) {
                const auto triangle = triangles[i];
                const auto f1 = triangle.x + start_index;
                const auto f2 = triangle.y + start_index;
                const auto f3 = triangle.z + start_index;
                data << "f " + std::to_string(f1) + "/" + std::to_string(f1) + "/" + std::to_string(f1) + " " +
                            std::to_string(f2) + "/" + std::to_string(f2) + "/" + std::to_string(f2) + " " +
                            std::to_string(f3) + "/" + std::to_string(f3) + "/" + std::to_string(f3) + "\n";
              }
#pragma endregion
              const auto result = data.str();
              of.write(result.c_str(), result.size());
              of.flush();
              start_index += vertices.size();
            }
          }
        }
        of.close();
      }
    } catch (std::exception e) {
      EVOENGINE_ERROR("Export failed: " + std::string(e.what()));
    }
  }
}

void Tree::ExportStrandModelObj(const std::filesystem::path& path,
                                const StrandModelMeshGeneratorSettings& mesh_generator_settings) {
  if (path.extension() == ".obj") {
    if (strand_model.strand_model_skeleton.RefRawNodes().size() !=
        tree_model.PeekShootSkeleton().PeekRawNodes().size()) {
      BuildStrandModel();
    }
    try {
      std::ofstream of;
      of.open(path.string(), std::ofstream::out | std::ofstream::trunc);
      if (of.is_open()) {
        std::string start = "#Forest OBJ exporter, by Bosheng Li";
        start += "\n";
        of.write(start.c_str(), start.size());
        of.flush();
        unsigned vertex_start_index = 1;
        unsigned tex_coords_start_index = 1;
        if (mesh_generator_settings.enable_branch) {
          std::vector<Vertex> vertices;
          std::vector<glm::vec2> tex_coords;
          std::vector<std::pair<unsigned int, unsigned int>> indices;
          StrandModelMeshGenerator::Generate(strand_model, vertices, tex_coords, indices, mesh_generator_settings);
          if (!vertices.empty() && !indices.empty()) {
            std::string header =
                "#Vertices: " + std::to_string(vertices.size()) + ", tris: " + std::to_string(indices.size());
            header += "\n";
            of.write(header.c_str(), header.size());
            of.flush();
            std::stringstream data;
            data << "o tree " + std::to_string(0) + "\n";
#pragma region Data collection
            for (auto& vertex : vertices) {
              auto& vertex_position = vertex.position;
              auto& color = vertex.color;
              data << "v " + std::to_string(vertex_position.x) + " " + std::to_string(vertex_position.y) + " " +
                          std::to_string(vertex_position.z) + " " + std::to_string(color.x) + " " +
                          std::to_string(color.y) + " " + std::to_string(color.z) + "\n";
            }
            for (const auto& tex_coord : tex_coords) {
              data << "vt " + std::to_string(tex_coord.x) + " " + std::to_string(tex_coord.y) + "\n";
            }
            data << "# List of indices for faces vertices, with (x, y, z).\n";
            for (auto i = 0; i < indices.size() / 3; i++) {
              const auto f1 = indices.at(i * 3).first + vertex_start_index;
              const auto f2 = indices.at(i * 3 + 1).first + vertex_start_index;
              const auto f3 = indices.at(i * 3 + 2).first + vertex_start_index;
              const auto t1 = indices.at(i * 3).second + tex_coords_start_index;
              const auto t2 = indices.at(i * 3 + 1).second + tex_coords_start_index;
              const auto t3 = indices.at(i * 3 + 2).second + tex_coords_start_index;
              data << "f " + std::to_string(f1) + "/" + std::to_string(t1) + "/" + std::to_string(f1) + " " +
                          std::to_string(f2) + "/" + std::to_string(t2) + "/" + std::to_string(f2) + " " +
                          std::to_string(f3) + "/" + std::to_string(t3) + "/" + std::to_string(f3) + "\n";
            }
#pragma endregion
            const auto result = data.str();
            of.write(result.c_str(), result.size());
            of.flush();
            vertex_start_index += vertices.size();
            tex_coords_start_index += tex_coords.size();
          }
        }
        if (mesh_generator_settings.enable_foliage) {
          if (const auto foliage_mesh = GenerateStrandModelFoliageMesh(mesh_generator_settings)) {
            const auto& vertices = foliage_mesh->UnsafeGetVertices();
            const auto& triangles = foliage_mesh->UnsafeGetTriangles();
            if (!vertices.empty() && !triangles.empty()) {
              std::string header =
                  "#Vertices: " + std::to_string(vertices.size()) + ", tris: " + std::to_string(triangles.size());
              header += "\n";
              of.write(header.c_str(), header.size());
              of.flush();
              std::stringstream data;
              data << "o tree " + std::to_string(0) + "\n";
#pragma region Data collection
              for (auto& vertex : vertices) {
                auto& vertex_position = vertex.position;
                auto& color = vertex.color;
                data << "v " + std::to_string(vertex_position.x) + " " + std::to_string(vertex_position.y) + " " +
                            std::to_string(vertex_position.z) + " " + std::to_string(color.x) + " " +
                            std::to_string(color.y) + " " + std::to_string(color.z) + "\n";
              }
              for (const auto& vertex : vertices) {
                data << "vt " + std::to_string(vertex.tex_coord.x) + " " + std::to_string(vertex.tex_coord.y) + "\n";
              }
              // data += "s off\n";
              data << "# List of indices for faces vertices, with (x, y, z).\n";
              for (auto triangle : triangles) {
                const auto f1 = triangle.x + vertex_start_index;
                const auto f2 = triangle.y + vertex_start_index;
                const auto f3 = triangle.z + vertex_start_index;
                const auto t1 = triangle.x + tex_coords_start_index;
                const auto t2 = triangle.y + tex_coords_start_index;
                const auto t3 = triangle.z + tex_coords_start_index;
                data << "f " + std::to_string(f1) + "/" + std::to_string(t1) + "/" + std::to_string(f1) + " " +
                            std::to_string(f2) + "/" + std::to_string(t2) + "/" + std::to_string(f2) + " " +
                            std::to_string(f3) + "/" + std::to_string(t3) + "/" + std::to_string(f3) + "\n";
              }
#pragma endregion
              const auto result = data.str();
              of.write(result.c_str(), result.size());
              of.flush();
              vertex_start_index += vertices.size();
              tex_coords_start_index += vertices.size();
            }
          }
        }
        of.close();
      }
    } catch (std::exception e) {
      EVOENGINE_ERROR("Export failed: " + std::string(e.what()));
    }
  }
}

void Tree::ExportTrunkObj(const std::filesystem::path& path, const TreeMeshGeneratorSettings& mesh_generator_settings) {
  if (path.extension() == ".obj") {
    std::ofstream of;
    of.open(path.string(), std::ofstream::out | std::ofstream::trunc);
    if (of.is_open()) {
      std::string start = "#Forest OBJ exporter, by Bosheng Li";
      start += "\n";
      of.write(start.c_str(), start.size());
      of.flush();
      unsigned start_index = 1;
      if (mesh_generator_settings.enable_branch) {
        std::shared_ptr<Mesh> trunk_mesh = ProjectManager::CreateTemporaryAsset<Mesh>();
        GenerateTrunkMeshes(trunk_mesh, mesh_generator_settings);
        if (trunk_mesh) {
          auto& vertices = trunk_mesh->UnsafeGetVertices();
          auto& triangles = trunk_mesh->UnsafeGetTriangles();
          if (!vertices.empty() && !triangles.empty()) {
            std::string header =
                "#Vertices: " + std::to_string(vertices.size()) + ", tris: " + std::to_string(triangles.size());
            header += "\n";
            of.write(header.c_str(), header.size());
            of.flush();
            std::stringstream data;
            data << std::string("o trunk") + "\n";
#pragma region Data collection
            for (auto i = 0; i < vertices.size(); i++) {
              auto& vertex_position = vertices.at(i).position;
              auto& color = vertices.at(i).color;
              data << "v " + std::to_string(vertex_position.x) + " " + std::to_string(vertex_position.y) + " " +
                          std::to_string(vertex_position.z) + " " + std::to_string(color.x) + " " +
                          std::to_string(color.y) + " " + std::to_string(color.z) + "\n";
            }
            for (const auto& vertex : vertices) {
              data << "vt " + std::to_string(vertex.tex_coord.x) + " " + std::to_string(vertex.tex_coord.y) + "\n";
            }
            // data += "s off\n";
            data << "# List of indices for faces vertices, with (x, y, z).\n";
            for (auto i = 0; i < triangles.size(); i++) {
              const auto triangle = triangles[i];
              const auto f1 = triangle.x + start_index;
              const auto f2 = triangle.y + start_index;
              const auto f3 = triangle.z + start_index;
              data << "f " + std::to_string(f1) + "/" + std::to_string(f1) + "/" + std::to_string(f1) + " " +
                          std::to_string(f2) + "/" + std::to_string(f2) + "/" + std::to_string(f2) + " " +
                          std::to_string(f3) + "/" + std::to_string(f3) + "/" + std::to_string(f3) + "\n";
            }
#pragma endregion
            const auto result = data.str();
            of.write(result.c_str(), result.size());
            of.flush();
            start_index += vertices.size();
          }
        }
      }
      of.close();
    }
  }
}

bool Tree::TryGrow(float delta_time, bool pruning) {
  const auto scene = GetScene();
  auto td = tree_descriptor.Get<TreeDescriptor>();
  if (!td) {
    EVOENGINE_WARNING("Growing tree without tree descriptor!");
    td = ProjectManager::CreateTemporaryAsset<TreeDescriptor>();
    tree_descriptor = td;
    const auto sd = ProjectManager::CreateTemporaryAsset<ShootDescriptor>();
    td->shoot_descriptor = sd;
    const auto fd = ProjectManager::CreateTemporaryAsset<FoliageDescriptor>();
    td->foliage_descriptor = fd;
  }
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

  const auto owner = GetOwner();

  auto sd = td->shoot_descriptor.Get<ShootDescriptor>();
  if (!sd) {
    sd = ProjectManager::CreateTemporaryAsset<ShootDescriptor>();
    td->shoot_descriptor = sd;
    EVOENGINE_WARNING("Shoot Descriptor Missing!");
  }

  PrepareController(sd, s, c);
  const bool grown = tree_model.Grow(delta_time, scene->GetDataComponent<GlobalTransform>(owner).value,
                                      c->climate_model, shoot_growth_controller_, pruning);
  if (grown) {
    if (pruning)
      tree_visualizer.ClearSelections();
    tree_visualizer.m_needUpdate = true;
  }
  if (enable_history && tree_model.iteration % history_iteration == 0)
    tree_model.Step();
  if (record_biomass_history) {
    const auto& base_shoot_node = tree_model.RefShootSkeleton().RefNode(0);
    shoot_biomass_history.emplace_back(base_shoot_node.data.biomass + base_shoot_node.data.descendant_total_biomass);
  }
  return grown;
}

bool Tree::TryGrowSubTree(const float delta_time, const SkeletonNodeHandle base_internode_handle, const bool pruning) {
  const auto scene = GetScene();
  const auto td = tree_descriptor.Get<TreeDescriptor>();
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

  if (!td) {
    EVOENGINE_ERROR("No tree descriptor!");
    return false;
  }

  const auto owner = GetOwner();

  auto shoot_descriptor = td->shoot_descriptor.Get<ShootDescriptor>();
  if (!shoot_descriptor) {
    shoot_descriptor = ProjectManager::CreateTemporaryAsset<ShootDescriptor>();
    td->shoot_descriptor = shoot_descriptor;
    EVOENGINE_WARNING("Shoot Descriptor Missing!");
  }

  PrepareController(shoot_descriptor, s, c);
  const bool grown =
      tree_model.Grow(delta_time, base_internode_handle, scene->GetDataComponent<GlobalTransform>(owner).value,
                       c->climate_model, shoot_growth_controller_, pruning);
  if (grown) {
    if (pruning)
      tree_visualizer.ClearSelections();
    tree_visualizer.m_needUpdate = true;
  }
  if (enable_history && tree_model.iteration % history_iteration == 0)
    tree_model.Step();
  if (record_biomass_history) {
    const auto& base_shoot_node = tree_model.RefShootSkeleton().RefNode(0);
    shoot_biomass_history.emplace_back(base_shoot_node.data.biomass + base_shoot_node.data.descendant_total_biomass);
  }
  return grown;
}

void Tree::Serialize(YAML::Emitter& out) const {
  tree_descriptor.Save("tree_descriptor", out);

  out << YAML::Key << "strand_model" << YAML::Value << YAML::BeginMap;
  {
    out << YAML::Key << "strand_model_skeleton" << YAML::Value << YAML::BeginMap;
    {
      SkeletonSerializer<StrandModelSkeletonData, StrandModelFlowData, StrandModelNodeData>::Serialize(
          out, strand_model.strand_model_skeleton,
          [&](YAML::Emitter& node_out, const StrandModelNodeData& node_data) {
            node_out << YAML::Key << "profile" << YAML::Value << YAML::BeginMap;
            {
              StrandModelProfileSerializer<CellParticlePhysicsData>::Serialize(
                  node_out, node_data.profile,
                  [&](YAML::Emitter&, const CellParticlePhysicsData&) {
                  });
            }
            node_out << YAML::EndMap;
          },
          [&](YAML::Emitter&, const StrandModelFlowData&) {
          },
          [&](YAML::Emitter& skeleton_out, const StrandModelSkeletonData& skeleton_data) {
            skeleton_out << YAML::Key << "strand_group" << YAML::Value << YAML::BeginMap;
            {
              StrandGroupSerializer<StrandModelStrandGroupData, StrandModelStrandData, StrandModelStrandSegmentData>::
                  Serialize(
                      skeleton_out, skeleton_data.strand_group,
                      [&](YAML::Emitter&, const StrandModelStrandSegmentData&) {
                      },
                      [&](YAML::Emitter&, const StrandModelStrandData&) {
                      },
                      [&](YAML::Emitter& group_out, const StrandModelStrandGroupData&) {
                        const auto strand_segment_size = skeleton_data.strand_group.PeekStrandSegments().size();
                        auto node_handle = std::vector<SkeletonNodeHandle>(strand_segment_size);
                        auto profile_particle_handles = std::vector<ParticleHandle>(strand_segment_size);
                        for (int strand_segment_index = 0; strand_segment_index < strand_segment_size; strand_segment_index++) {
                          const auto& strand_segment = skeleton_data.strand_group.PeekStrandSegment(strand_segment_index);
                          node_handle.at(strand_segment_index) = strand_segment.data.node_handle;
                          profile_particle_handles.at(strand_segment_index) = strand_segment.data.profile_particle_handle;
                        }
                        if (strand_segment_size != 0) {
                          group_out << YAML::Key << "ss.data.node_handle" << YAML::Value
                                   << YAML::Binary(reinterpret_cast<const unsigned char*>(node_handle.data()),
                                                   node_handle.size() * sizeof(SkeletonNodeHandle));
                          group_out << YAML::Key << "ss.data.profile_particle_handle" << YAML::Value
                                   << YAML::Binary(
                                          reinterpret_cast<const unsigned char*>(profile_particle_handles.data()),
                                          profile_particle_handles.size() * sizeof(ParticleHandle));
                        }
                      });
            }
            skeleton_out << YAML::EndMap;

            const auto node_size = strand_model.strand_model_skeleton.PeekRawNodes().size();
            auto offset = std::vector<glm::vec2>(node_size);
            auto twist_angle = std::vector<float>(node_size);
            auto split = std::vector<int>(node_size);
            auto strand_radius = std::vector<float>(node_size);
            auto strand_count = std::vector<int>(node_size);

            for (int node_index = 0; node_index < node_size; node_index++) {
              const auto& node = strand_model.strand_model_skeleton.PeekRawNodes().at(node_index);
              offset.at(node_index) = node.data.offset;
              twist_angle.at(node_index) = node.data.twist_angle;
              split.at(node_index) = node.data.split == 1;
              strand_radius.at(node_index) = node.data.strand_radius;
              strand_count.at(node_index) = node.data.strand_count;
            }
            if (node_size != 0) {
              skeleton_out << YAML::Key << "node.data.offset" << YAML::Value
                          << YAML::Binary(reinterpret_cast<const unsigned char*>(offset.data()),
                                          offset.size() * sizeof(glm::vec2));
              skeleton_out << YAML::Key << "node.data.twist_angle" << YAML::Value
                          << YAML::Binary(reinterpret_cast<const unsigned char*>(twist_angle.data()),
                                          twist_angle.size() * sizeof(float));
              skeleton_out << YAML::Key << "node.data.split" << YAML::Value
                          << YAML::Binary(reinterpret_cast<const unsigned char*>(split.data()),
                                          split.size() * sizeof(int));
              skeleton_out << YAML::Key << "node.data.strand_radius" << YAML::Value
                          << YAML::Binary(reinterpret_cast<const unsigned char*>(strand_radius.data()),
                                          strand_radius.size() * sizeof(float));
              skeleton_out << YAML::Key << "node.data.strand_count" << YAML::Value
                          << YAML::Binary(reinterpret_cast<const unsigned char*>(strand_count.data()),
                                          strand_count.size() * sizeof(float));
            }
          });
    }
    out << YAML::EndMap;
  }
  out << YAML::EndMap;

  out << YAML::Key << "tree_model" << YAML::Value << YAML::BeginMap;
  {
    out << YAML::Key << "shoot_skeleton" << YAML::Value << YAML::BeginMap;
    {
      SkeletonSerializer<ShootGrowthData, ShootStemGrowthData, InternodeGrowthData>::Serialize(
          out, tree_model.PeekShootSkeleton(),
          [&](YAML::Emitter& node_out, const InternodeGrowthData& node_data) {
            node_out << YAML::Key << "buds" << YAML::Value << YAML::BeginSeq;
            for (const auto& bud : node_data.buds) {
              node_out << YAML::BeginMap;
              {
                node_out << YAML::Key << "T" << YAML::Value << static_cast<unsigned>(bud.type);
                node_out << YAML::Key << "S" << YAML::Value << static_cast<unsigned>(bud.status);
                node_out << YAML::Key << "LR" << YAML::Value << bud.local_rotation;
                node_out << YAML::Key << "RM" << YAML::Value << YAML::BeginMap;
                {
                  node_out << YAML::Key << "M" << YAML::Value << bud.reproductive_module.maturity;
                  node_out << YAML::Key << "H" << YAML::Value << bud.reproductive_module.health;
                  node_out << YAML::Key << "T" << YAML::Value << bud.reproductive_module.transform;
                }
                node_out << YAML::EndMap;
              }
              node_out << YAML::EndMap;
            }
            node_out << YAML::EndSeq;
          },
          [&](YAML::Emitter& flow_out, const ShootStemGrowthData& flow_data) {
            flow_out << YAML::Key << "order" << YAML::Value << flow_data.order;
          },
          [&](YAML::Emitter& skeleton_out, const ShootGrowthData& skeleton_data) {
            skeleton_out << YAML::Key << "desired_min" << YAML::Value << skeleton_data.desired_min;
            skeleton_out << YAML::Key << "desired_max" << YAML::Value << skeleton_data.desired_max;

            const auto node_size = tree_model.PeekShootSkeleton().PeekRawNodes().size();
            auto internode_length = std::vector<float>(node_size);
            auto index_of_parent_bud = std::vector<int>(node_size);
            auto start_age = std::vector<float>(node_size);
            auto finish_age = std::vector<float>(node_size);
            auto desired_local_rotation = std::vector<glm::quat>(node_size);
            auto desired_global_rotation = std::vector<glm::quat>(node_size);
            auto desired_global_position = std::vector<glm::vec3>(node_size);
            auto sagging = std::vector<float>(node_size);
            auto order = std::vector<int>(node_size);
            auto extra_mass = std::vector<float>(node_size);
            auto density = std::vector<float>(node_size);
            auto strength = std::vector<float>(node_size);

            for (int node_index = 0; node_index < node_size; node_index++) {
              const auto& node = tree_model.PeekShootSkeleton().PeekRawNodes().at(node_index);
              internode_length.at(node_index) = node.data.internode_length;
              index_of_parent_bud.at(node_index) = node.data.index_of_parent_bud;
              start_age.at(node_index) = node.data.start_age;
              finish_age.at(node_index) = node.data.finish_age;
              desired_local_rotation.at(node_index) = node.data.desired_local_rotation;
              desired_global_rotation.at(node_index) = node.data.desired_global_rotation;
              desired_global_position.at(node_index) = node.data.desired_global_position;
              sagging.at(node_index) = node.data.sagging;
              order.at(node_index) = node.data.order;
              extra_mass.at(node_index) = node.data.extra_mass;
              density.at(node_index) = node.data.density;
              strength.at(node_index) = node.data.strength;
            }
            if (node_size != 0) {
              skeleton_out << YAML::Key << "node.data.internode_length" << YAML::Value
                          << YAML::Binary(reinterpret_cast<const unsigned char*>(internode_length.data()),
                                          internode_length.size() * sizeof(float));
              skeleton_out << YAML::Key << "node.data.index_of_parent_bud" << YAML::Value
                          << YAML::Binary(reinterpret_cast<const unsigned char*>(index_of_parent_bud.data()),
                                          index_of_parent_bud.size() * sizeof(int));
              skeleton_out << YAML::Key << "node.data.start_age" << YAML::Value
                          << YAML::Binary(reinterpret_cast<const unsigned char*>(start_age.data()),
                                          start_age.size() * sizeof(float));
              skeleton_out << YAML::Key << "node.data.finish_age" << YAML::Value
                          << YAML::Binary(reinterpret_cast<const unsigned char*>(finish_age.data()),
                                          finish_age.size() * sizeof(float));
              skeleton_out << YAML::Key << "node.data.desired_local_rotation" << YAML::Value
                          << YAML::Binary(reinterpret_cast<const unsigned char*>(desired_local_rotation.data()),
                                          desired_local_rotation.size() * sizeof(glm::quat));
              skeleton_out << YAML::Key << "node.data.desired_global_rotation" << YAML::Value
                          << YAML::Binary(reinterpret_cast<const unsigned char*>(desired_global_rotation.data()),
                                          desired_global_rotation.size() * sizeof(glm::quat));
              skeleton_out << YAML::Key << "node.data.desired_global_position" << YAML::Value
                          << YAML::Binary(reinterpret_cast<const unsigned char*>(desired_global_position.data()),
                                          desired_global_position.size() * sizeof(glm::vec3));
              skeleton_out << YAML::Key << "node.data.sagging" << YAML::Value
                          << YAML::Binary(reinterpret_cast<const unsigned char*>(sagging.data()),
                                          sagging.size() * sizeof(float));
              skeleton_out << YAML::Key << "node.data.order" << YAML::Value
                          << YAML::Binary(reinterpret_cast<const unsigned char*>(order.data()),
                                          order.size() * sizeof(int));
              skeleton_out << YAML::Key << "node.data.extra_mass" << YAML::Value
                          << YAML::Binary(reinterpret_cast<const unsigned char*>(extra_mass.data()),
                                          extra_mass.size() * sizeof(float));
              skeleton_out << YAML::Key << "node.data.density" << YAML::Value
                          << YAML::Binary(reinterpret_cast<const unsigned char*>(density.data()),
                                          density.size() * sizeof(float));
              skeleton_out << YAML::Key << "node.data.strength" << YAML::Value
                          << YAML::Binary(reinterpret_cast<const unsigned char*>(strength.data()),
                                          strength.size() * sizeof(float));
            }
          });
    }
    out << YAML::EndMap;
  }
  out << YAML::EndMap;
}

void Tree::Deserialize(const YAML::Node& in) {
  tree_descriptor.Load("tree_descriptor", in);
  if (in["strand_model"]) {
    if (const auto& in_strand_model = in["strand_model"]) {
      const auto& in_strand_model_skeleton = in_strand_model["strand_model_skeleton"];
      SkeletonSerializer<StrandModelSkeletonData, StrandModelFlowData, StrandModelNodeData>::Deserialize(
          in_strand_model_skeleton, strand_model.strand_model_skeleton,
          [&](const YAML::Node& node_in, StrandModelNodeData& node_data) {
            node_data = {};
            if (node_in["profile"]) {
              const auto& in_strand_group = node_in["profile"];
              StrandModelProfileSerializer<CellParticlePhysicsData>::Deserialize(
                  in_strand_group, node_data.profile,
                  [&](const YAML::Node&, CellParticlePhysicsData&) {
                  });
            }
          },
          [&](const YAML::Node&, StrandModelFlowData&) {
          },
          [&](const YAML::Node& skeleton_in, StrandModelSkeletonData& skeleton_data) {
            if (skeleton_in["strand_group"]) {
              const auto& in_strand_group = skeleton_in["strand_group"];
              StrandGroupSerializer<StrandModelStrandGroupData, StrandModelStrandData, StrandModelStrandSegmentData>::
                  Deserialize(
                      in_strand_group, strand_model.strand_model_skeleton.data.strand_group,
                      [&](const YAML::Node&, StrandModelStrandSegmentData&) {
                      },
                      [&](const YAML::Node&, StrandModelStrandData&) {
                      },
                      [&](const YAML::Node& group_in, StrandModelStrandGroupData&) {
                        if (group_in["ss.data.node_handle"]) {
                          auto list = std::vector<SkeletonNodeHandle>();
                          const auto data = group_in["ss.data.node_handle"].as<YAML::Binary>();
                          list.resize(data.size() / sizeof(SkeletonNodeHandle));
                          std::memcpy(list.data(), data.data(), data.size());
                          for (size_t i = 0; i < list.size(); i++) {
                            auto& strand_segment = skeleton_data.strand_group.RefStrandSegment(i);
                            strand_segment.data.node_handle = list[i];
                          }
                        }

                        if (group_in["ss.data.profile_particle_handle"]) {
                          auto list = std::vector<ParticleHandle>();
                          const auto data = group_in["ss.data.profile_particle_handle"].as<YAML::Binary>();
                          list.resize(data.size() / sizeof(ParticleHandle));
                          std::memcpy(list.data(), data.data(), data.size());
                          for (size_t i = 0; i < list.size(); i++) {
                            auto& strand_segment = skeleton_data.strand_group.RefStrandSegment(i);
                            strand_segment.data.profile_particle_handle = list[i];
                          }
                        }
                      });
            }

            if (skeleton_in["node.data.offset"]) {
              auto list = std::vector<glm::vec2>();
              const auto data = skeleton_in["node.data.offset"].as<YAML::Binary>();
              list.resize(data.size() / sizeof(glm::vec2));
              std::memcpy(list.data(), data.data(), data.size());
              for (size_t i = 0; i < list.size(); i++) {
                auto& node = strand_model.strand_model_skeleton.RefNode(i);
                node.data.offset = list[i];
              }
            }

            if (skeleton_in["node.data.twist_angle"]) {
              auto list = std::vector<float>();
              const auto data = skeleton_in["node.data.twist_angle"].as<YAML::Binary>();
              list.resize(data.size() / sizeof(float));
              std::memcpy(list.data(), data.data(), data.size());
              for (size_t i = 0; i < list.size(); i++) {
                auto& node = strand_model.strand_model_skeleton.RefNode(i);
                node.data.twist_angle = list[i];
              }
            }

            if (skeleton_in["node.data.packing_iteration"]) {
              auto list = std::vector<int>();
              const auto data = skeleton_in["node.data.packing_iteration"].as<YAML::Binary>();
              list.resize(data.size() / sizeof(int));
              std::memcpy(list.data(), data.data(), data.size());
              for (size_t i = 0; i < list.size(); i++) {
                auto& node = strand_model.strand_model_skeleton.RefNode(i);
                node.data.packing_iteration = list[i];
              }
            }

            if (skeleton_in["node.data.split"]) {
              auto list = std::vector<int>();
              const auto data = skeleton_in["node.data.split"].as<YAML::Binary>();
              list.resize(data.size() / sizeof(int));
              std::memcpy(list.data(), data.data(), data.size());
              for (size_t i = 0; i < list.size(); i++) {
                auto& node = strand_model.strand_model_skeleton.RefNode(i);
                node.data.split = list[i] == 1;
              }
            }

            if (skeleton_in["node.data.strand_radius"]) {
              auto list = std::vector<float>();
              const auto data = skeleton_in["node.data.strand_radius"].as<YAML::Binary>();
              list.resize(data.size() / sizeof(float));
              std::memcpy(list.data(), data.data(), data.size());
              for (size_t i = 0; i < list.size(); i++) {
                auto& node = strand_model.strand_model_skeleton.RefNode(i);
                node.data.strand_radius = list[i];
              }
            }

            if (skeleton_in["node.data.strand_count"]) {
              auto list = std::vector<int>();
              const auto data = skeleton_in["node.data.strand_count"].as<YAML::Binary>();
              list.resize(data.size() / sizeof(int));
              std::memcpy(list.data(), data.data(), data.size());
              for (size_t i = 0; i < list.size(); i++) {
                auto& node = strand_model.strand_model_skeleton.RefNode(i);
                node.data.strand_count = list[i];
              }
            }
          });
    }
  }
  if (in["tree_model"]) {
    if (const auto& in_tree_model = in["tree_model"]) {
      const auto& in_shoot_skeleton = in_tree_model["shoot_skeleton"];
      SkeletonSerializer<ShootGrowthData, ShootStemGrowthData, InternodeGrowthData>::Deserialize(
          in_shoot_skeleton, tree_model.RefShootSkeleton(),
          [&](const YAML::Node& node_in, InternodeGrowthData& node_data) {
            node_data.buds.clear();
            if (node_in["buds"]) {
              const auto& in_buds = node_in["buds"];
              for (const auto& in_bud : in_buds) {
                node_data.buds.emplace_back();
                auto& bud = node_data.buds.back();
                if (in_bud["T"])
                  bud.type = static_cast<BudType>(in_bud["T"].as<unsigned>());
                if (in_bud["S"])
                  bud.status = static_cast<BudStatus>(in_bud["S"].as<unsigned>());
                if (in_bud["LR"])
                  bud.local_rotation = in_bud["LR"].as<glm::quat>();
                if (in_bud["RM"]) {
                  const auto& in_reproductive_module = in_bud["RM"];
                  if (in_reproductive_module["M"])
                    bud.reproductive_module.maturity = in_reproductive_module["M"].as<float>();
                  if (in_reproductive_module["H"])
                    bud.reproductive_module.health = in_reproductive_module["H"].as<float>();
                  if (in_reproductive_module["T"])
                    bud.reproductive_module.transform = in_reproductive_module["T"].as<glm::mat4>();
                }
              }
            }
          },
          [&](const YAML::Node& flow_in, ShootStemGrowthData& flow_data) {
            if (flow_in["data"])
              flow_data.order = flow_in["data"].as<int>();
          },
          [&](const YAML::Node& skeleton_in, ShootGrowthData& skeleton_data) {
            if (skeleton_in["desired_min"])
              skeleton_data.desired_min = skeleton_in["desired_min"].as<glm::vec3>();
            if (skeleton_in["desired_max"])
              skeleton_data.desired_max = skeleton_in["desired_max"].as<glm::vec3>();

            if (skeleton_in["node.data.internode_length"]) {
              auto list = std::vector<float>();
              const auto data = skeleton_in["node.data.internode_length"].as<YAML::Binary>();
              list.resize(data.size() / sizeof(float));
              std::memcpy(list.data(), data.data(), data.size());
              for (size_t i = 0; i < list.size(); i++) {
                auto& node = tree_model.RefShootSkeleton().RefNode(i);
                node.data.internode_length = list[i];
              }
            }

            if (skeleton_in["node.data.index_of_parent_bud"]) {
              auto list = std::vector<int>();
              const auto data = skeleton_in["node.data.index_of_parent_bud"].as<YAML::Binary>();
              list.resize(data.size() / sizeof(int));
              std::memcpy(list.data(), data.data(), data.size());
              for (size_t i = 0; i < list.size(); i++) {
                auto& node = tree_model.RefShootSkeleton().RefNode(i);
                node.data.index_of_parent_bud = list[i];
              }
            }

            if (skeleton_in["node.data.start_age"]) {
              auto list = std::vector<float>();
              const auto data = skeleton_in["node.data.start_age"].as<YAML::Binary>();
              list.resize(data.size() / sizeof(float));
              std::memcpy(list.data(), data.data(), data.size());
              for (size_t i = 0; i < list.size(); i++) {
                auto& node = tree_model.RefShootSkeleton().RefNode(i);
                node.data.start_age = list[i];
              }
            }

            if (skeleton_in["node.data.finish_age"]) {
              auto list = std::vector<float>();
              const auto data = skeleton_in["node.data.finish_age"].as<YAML::Binary>();
              list.resize(data.size() / sizeof(float));
              std::memcpy(list.data(), data.data(), data.size());
              for (size_t i = 0; i < list.size(); i++) {
                auto& node = tree_model.RefShootSkeleton().RefNode(i);
                node.data.finish_age = list[i];
              }
            }

            if (skeleton_in["node.data.desired_local_rotation"]) {
              auto list = std::vector<glm::quat>();
              const auto data = skeleton_in["node.data.desired_local_rotation"].as<YAML::Binary>();
              list.resize(data.size() / sizeof(glm::quat));
              std::memcpy(list.data(), data.data(), data.size());
              for (size_t i = 0; i < list.size(); i++) {
                auto& node = tree_model.RefShootSkeleton().RefNode(i);
                node.data.desired_local_rotation = list[i];
              }
            }

            if (skeleton_in["node.data.desired_global_rotation"]) {
              auto list = std::vector<glm::quat>();
              const auto data = skeleton_in["node.data.desired_global_rotation"].as<YAML::Binary>();
              list.resize(data.size() / sizeof(glm::quat));
              std::memcpy(list.data(), data.data(), data.size());
              for (size_t i = 0; i < list.size(); i++) {
                auto& node = tree_model.RefShootSkeleton().RefNode(i);
                node.data.desired_global_rotation = list[i];
              }
            }

            if (skeleton_in["node.data.desired_global_position"]) {
              auto list = std::vector<glm::vec3>();
              const auto data = skeleton_in["node.data.desired_global_position"].as<YAML::Binary>();
              list.resize(data.size() / sizeof(glm::vec3));
              std::memcpy(list.data(), data.data(), data.size());
              for (size_t i = 0; i < list.size(); i++) {
                auto& node = tree_model.RefShootSkeleton().RefNode(i);
                node.data.desired_global_position = list[i];
              }
            }

            if (skeleton_in["node.data.sagging"]) {
              auto list = std::vector<float>();
              const auto data = skeleton_in["node.data.sagging"].as<YAML::Binary>();
              list.resize(data.size() / sizeof(float));
              std::memcpy(list.data(), data.data(), data.size());
              for (size_t i = 0; i < list.size(); i++) {
                auto& node = tree_model.RefShootSkeleton().RefNode(i);
                node.data.sagging = list[i];
              }
            }

            if (skeleton_in["node.data.order"]) {
              auto list = std::vector<int>();
              const auto data = skeleton_in["node.data.order"].as<YAML::Binary>();
              list.resize(data.size() / sizeof(int));
              std::memcpy(list.data(), data.data(), data.size());
              for (size_t i = 0; i < list.size(); i++) {
                auto& node = tree_model.RefShootSkeleton().RefNode(i);
                node.data.order = list[i];
              }
            }

            if (skeleton_in["node.data.extra_mass"]) {
              auto list = std::vector<float>();
              const auto data = skeleton_in["node.data.extra_mass"].as<YAML::Binary>();
              list.resize(data.size() / sizeof(float));
              std::memcpy(list.data(), data.data(), data.size());
              for (size_t i = 0; i < list.size(); i++) {
                auto& node = tree_model.RefShootSkeleton().RefNode(i);
                node.data.extra_mass = list[i];
              }
            }

            if (skeleton_in["node.data.density"]) {
              auto list = std::vector<float>();
              const auto data = skeleton_in["node.data.density"].as<YAML::Binary>();
              list.resize(data.size() / sizeof(float));
              std::memcpy(list.data(), data.data(), data.size());
              for (size_t i = 0; i < list.size(); i++) {
                auto& node = tree_model.RefShootSkeleton().RefNode(i);
                node.data.density = list[i];
              }
            }

            if (skeleton_in["node.data.strength"]) {
              auto list = std::vector<float>();
              const auto data = skeleton_in["node.data.strength"].as<YAML::Binary>();
              list.resize(data.size() / sizeof(float));
              std::memcpy(list.data(), data.data(), data.size());
              for (size_t i = 0; i < list.size(); i++) {
                auto& node = tree_model.RefShootSkeleton().RefNode(i);
                node.data.strength = list[i];
              }
            }
          });
      tree_model.initialized_ = true;
    }
  }
}
inline void TransformVertex(Vertex& v, const glm::mat4& transform) {
  v.normal = glm::normalize(transform * glm::vec4(v.normal, 0.f));
  v.tangent = glm::normalize(transform * glm::vec4(v.tangent, 0.f));
  v.position = transform * glm::vec4(v.position, 1.f);
}
void Tree::GenerateBillboardClouds(const BillboardCloud::GenerateSettings& foliage_generate_settings) {
  auto mesh_generator_settings = tree_mesh_generator_settings;
  mesh_generator_settings.foliage_instancing = false;
  GenerateGeometryEntities(mesh_generator_settings);

  const auto scene = GetScene();
  const auto owner = GetOwner();
  TransformGraph::CalculateTransformGraphForDescendants(scene, owner);
  const auto children = scene->GetChildren(owner);
  const auto owner_global_transform = scene->GetDataComponent<GlobalTransform>(owner);
  for (const auto& child : children) {
    auto name = scene->GetEntityName(child);
    if (name == "Projected Tree") {
      scene->DeleteEntity(child);
    }
  }
  const auto projected_tree = scene->CreateEntity("Projected Tree");
  scene->SetParent(projected_tree, owner);
  std::vector<BillboardCloud> billboard_clouds;

  for (const auto& child : children) {
    if (!scene->IsEntityValid(child))
      continue;
    auto name = scene->GetEntityName(child);
    const auto model_space_transform =
        glm::inverse(owner_global_transform.value) * scene->GetDataComponent<GlobalTransform>(child).value;
    if (name == "Foliage Mesh") {
      if (scene->HasPrivateComponent<MeshRenderer>(child)) {
        const auto mesh_renderer = scene->GetOrSetPrivateComponent<MeshRenderer>(child).lock();
        const auto mesh = mesh_renderer->mesh.Get<Mesh>();
        const auto material = mesh_renderer->material.Get<Material>();
        if (mesh && material) {
          billboard_clouds.emplace_back();
          auto& billboard_cloud = billboard_clouds.back();
          billboard_cloud.elements.emplace_back();
          auto& element = billboard_cloud.elements.back();
          element.vertices = mesh->UnsafeGetVertices();
          element.material = material;
          element.triangles = mesh->UnsafeGetTriangles();
          Jobs::RunParallelFor(element.vertices.size(), [&](const unsigned vertex_index) {
            TransformVertex(element.vertices.at(vertex_index), model_space_transform);
          });
          billboard_cloud.Generate(foliage_generate_settings);
        }
      }
      scene->SetEnable(child, false);
    } else if (name == "Fruit Mesh") {
    }
  }
  int cloud_index = 0;
  for (const auto& billboard_cloud : billboard_clouds) {
    const auto billboard_cloud_entity = billboard_cloud.BuildEntity(scene);
    scene->SetEntityName(billboard_cloud_entity, "Projected Cluster [" + std::to_string(cloud_index) + "]");
    scene->SetParent(billboard_cloud_entity, projected_tree);
    cloud_index++;
  }
}

void Tree::GenerateAnimatedGeometryEntities(const TreeMeshGeneratorSettings& mesh_generator_settings, const int iteration,
                                            const bool enable_physics) {
  const auto scene = GetScene();
  const auto self = GetOwner();
  const auto children = scene->GetChildren(self);
  auto td = tree_descriptor.Get<TreeDescriptor>();
  const auto tree_global_transform = scene->GetDataComponent<GlobalTransform>(self);
  ClearAnimatedGeometryEntities();
  Entity rag_doll;
  rag_doll = scene->CreateEntity("Rag Doll");
  scene->SetParent(rag_doll, self);
  auto actual_iteration = iteration;
  if (actual_iteration < 0 || actual_iteration > tree_model.CurrentIteration()) {
    actual_iteration = tree_model.CurrentIteration();
  }
  const auto& skeleton = tree_model.PeekShootSkeleton(actual_iteration);
  const auto& sorted_flow_list = skeleton.PeekSortedFlowList();
  std::vector<glm::mat4> offset_matrices;
  std::unordered_map<SkeletonFlowHandle, int> flow_bone_id_map;

  CylindricalSkinnedMeshGenerator<ShootGrowthData, ShootStemGrowthData, InternodeGrowthData>::GenerateBones(
      skeleton, sorted_flow_list, offset_matrices, flow_bone_id_map);

  std::vector<std::string> names;
  std::vector<Entity> bound_entities;
  std::vector<unsigned> bone_indices_lists;
  bone_indices_lists.resize(offset_matrices.size());
  bound_entities.resize(offset_matrices.size());
  names.resize(offset_matrices.size());
  std::unordered_map<SkeletonFlowHandle, Entity> corresponding_flow_handles;
  std::unordered_map<unsigned, SkeletonFlowHandle> corresponding_entities;
  for (const auto& [flowHandle, matrixIndex] : flow_bone_id_map) {
    names[matrixIndex] = std::to_string(flowHandle);
    bound_entities[matrixIndex] = scene->CreateEntity(names[matrixIndex]);
    const auto& flow = skeleton.PeekFlow(flowHandle);

    corresponding_flow_handles[flowHandle] = bound_entities[matrixIndex];
    corresponding_entities[bound_entities[matrixIndex].GetIndex()] = flowHandle;
    GlobalTransform global_transform;

    global_transform.value = tree_global_transform.value * (glm::translate(flow.info.global_start_position) *
                                                             glm::mat4_cast(flow.info.global_start_rotation));
    scene->SetDataComponent(bound_entities[matrixIndex], global_transform);

    bone_indices_lists[matrixIndex] = matrixIndex;
  }
  for (const auto& flow_handle : sorted_flow_list) {
    const auto& flow = skeleton.PeekFlow(flow_handle);
    if (const auto& parent_flow_handle = flow.GetParentHandle(); parent_flow_handle != -1)
      scene->SetParent(corresponding_flow_handles[flow_handle], corresponding_flow_handles[parent_flow_handle]);
    else {
      scene->SetParent(corresponding_flow_handles[flow_handle], rag_doll);
    }
  }
  if (mesh_generator_settings.enable_branch) {
    Entity branch_entity;
    branch_entity = scene->CreateEntity("Animated Branch Mesh");
    scene->SetParent(branch_entity, self);
    auto animator = scene->GetOrSetPrivateComponent<Animator>(branch_entity).lock();
    auto skinned_mesh = ProjectManager::CreateTemporaryAsset<SkinnedMesh>();
    auto material = ProjectManager::CreateTemporaryAsset<Material>();
    auto skinned_mesh_renderer = scene->GetOrSetPrivateComponent<SkinnedMeshRenderer>(branch_entity).lock();
    if (td) {
      if (const auto shoot_descriptor = td->shoot_descriptor.Get<ShootDescriptor>()) {
        if (const auto shoot_material = shoot_descriptor->bark_material.Get<Material>()) {
          material->SetAlbedoTexture(shoot_material->GetAlbedoTexture());
          material->SetNormalTexture(shoot_material->GetNormalTexture());
          material->SetRoughnessTexture(shoot_material->GetRoughnessTexture());
          material->SetMetallicTexture(shoot_material->GetMetallicTexture());
          material->material_properties = shoot_material->material_properties;
        }
      }
    }
    if (bool copied_material = false; !copied_material) {
      material->material_properties.albedo_color = glm::vec3(109, 79, 75) / 255.0f;
      material->material_properties.roughness = 1.0f;
      material->material_properties.metallic = 0.0f;
    }

    std::vector<SkinnedVertex> skinned_vertices;
    std::vector<unsigned int> indices;

    if (!td) {
      EVOENGINE_WARNING("TreeDescriptor missing!");
      td = ProjectManager::CreateTemporaryAsset<TreeDescriptor>();
      td->foliage_descriptor = ProjectManager::CreateTemporaryAsset<FoliageDescriptor>();
    }
    std::shared_ptr<BarkDescriptor> bark_descriptor{};
    bark_descriptor = td->bark_descriptor.Get<BarkDescriptor>();
    if (strand_model.strand_model_skeleton.RefRawNodes().size() == tree_model.shoot_skeleton_.RefRawNodes().size()) {
      CylindricalSkinnedMeshGenerator<StrandModelSkeletonData, StrandModelFlowData, StrandModelNodeData>::Generate(
          strand_model.strand_model_skeleton, skinned_vertices, indices, offset_matrices, mesh_generator_settings,
          [&](glm::vec3& vertex_position, const glm::vec3& direction, const float x_factor, const float y_factor) {
            if (bark_descriptor) {
              const float push_value = bark_descriptor->GetValue(x_factor, y_factor);
              vertex_position += push_value * direction;
            }
          },
          [&](glm::vec2&, float, float) {
          });
    } else {
      CylindricalSkinnedMeshGenerator<ShootGrowthData, ShootStemGrowthData, InternodeGrowthData>::Generate(
          skeleton, skinned_vertices, indices, offset_matrices, mesh_generator_settings,
          [&](glm::vec3& vertex_position, const glm::vec3& direction, const float x_factor, const float y_factor) {
            if (bark_descriptor) {
              const float push_value = bark_descriptor->GetValue(x_factor, y_factor);
              vertex_position += push_value * direction;
            }
          },
          [&](glm::vec2&, float, float) {
          });
    }

    skinned_mesh->bone_animator_indices = bone_indices_lists;
    SkinnedVertexAttributes attributes{};
    attributes.tex_coord = true;
    skinned_mesh->SetVertices(attributes, skinned_vertices, indices);
    skinned_mesh_renderer->animator = animator;
    skinned_mesh_renderer->skinned_mesh = skinned_mesh;
    skinned_mesh_renderer->material = material;

    animator->Setup(names, offset_matrices);
    skinned_mesh_renderer->SetRagDoll(true);
    skinned_mesh_renderer->SetRagDollBoundEntities(bound_entities, false);
  }

  if (mesh_generator_settings.enable_foliage) {
    const auto foliage_entity = scene->CreateEntity("Animated Foliage Mesh");
    scene->SetParent(foliage_entity, self);
    auto animator = scene->GetOrSetPrivateComponent<Animator>(foliage_entity).lock();

    auto skinned_mesh = ProjectManager::CreateTemporaryAsset<SkinnedMesh>();
    auto skinned_mesh_renderer = scene->GetOrSetPrivateComponent<SkinnedMeshRenderer>(foliage_entity).lock();
    const auto material = ProjectManager::CreateTemporaryAsset<Material>();
    bool copied_material = false;
    if (td) {
      if (const auto foliage_descriptor = td->foliage_descriptor.Get<FoliageDescriptor>()) {
        if (const auto leaf_material = foliage_descriptor->leaf_material.Get<Material>()) {
          material->SetAlbedoTexture(leaf_material->GetAlbedoTexture());
          material->SetNormalTexture(leaf_material->GetNormalTexture());
          material->SetRoughnessTexture(leaf_material->GetRoughnessTexture());
          material->SetMetallicTexture(leaf_material->GetMetallicTexture());
          material->material_properties = leaf_material->material_properties;
          copied_material = true;
        }
      }
    }
    if (!copied_material) {
      material->material_properties.albedo_color = glm::vec3(152 / 255.0f, 203 / 255.0f, 0 / 255.0f);
      material->material_properties.roughness = 1.0f;
      material->material_properties.metallic = 0.0f;
    }
    std::vector<SkinnedVertex> skinned_vertices;
    std::vector<unsigned> indices;
    {
      auto quad_mesh = Resources::GetResource<Mesh>("PRIMITIVE_QUAD");
      auto& quad_triangles = quad_mesh->UnsafeGetTriangles();
      auto quad_vertices_size = quad_mesh->GetVerticesAmount();
      size_t offset = 0;
      if (!td) {
        EVOENGINE_WARNING("TreeDescriptor missing!");
        td = ProjectManager::CreateTemporaryAsset<TreeDescriptor>();
        td->foliage_descriptor = ProjectManager::CreateTemporaryAsset<FoliageDescriptor>();
      }
      auto fd = td->foliage_descriptor.Get<FoliageDescriptor>();
      if (!fd)
        fd = ProjectManager::CreateTemporaryAsset<FoliageDescriptor>();
      const auto tree_dim = skeleton.max - skeleton.min;

      const auto& node_list = skeleton.PeekSortedNodeList();
      for (const auto& internode_handle : node_list) {
        const auto& node = skeleton.PeekNode(internode_handle);
        const auto flow_handle = node.GetFlowHandle();
        const auto& flow = skeleton.PeekFlow(flow_handle);
        const auto& internode_info = node.info;
        std::vector<glm::mat4> leaf_matrices;
        fd->GenerateFoliageMatrices(leaf_matrices, internode_info, glm::length(tree_dim));
        SkinnedVertex archetype;
        archetype.bond_id = glm::ivec4(flow_bone_id_map[flow_handle], flow_bone_id_map[flow_handle], -1, -1);
        archetype.bond_id2 = glm::ivec4(-1);
        archetype.weight = glm::vec4(.5f, .5f, .0f, .0f);
        archetype.weight2 = glm::vec4(0.f);

        for (const auto& matrix : leaf_matrices) {
          for (auto i = 0; i < quad_mesh->GetVerticesAmount(); i++) {
            archetype.position = matrix * glm::vec4(quad_mesh->UnsafeGetVertices()[i].position, 1.0f);
            archetype.normal =
                glm::normalize(glm::vec3(matrix * glm::vec4(quad_mesh->UnsafeGetVertices()[i].normal, 0.0f)));
            archetype.tangent =
                glm::normalize(glm::vec3(matrix * glm::vec4(quad_mesh->UnsafeGetVertices()[i].tangent, 0.0f)));
            archetype.tex_coord = quad_mesh->UnsafeGetVertices()[i].tex_coord;
            archetype.color = internode_info.color;
            skinned_vertices.push_back(archetype);
          }
          for (auto triangle : quad_triangles) {
            triangle.x += offset;
            triangle.y += offset;
            triangle.z += offset;
            indices.push_back(triangle.x);
            indices.push_back(triangle.y);
            indices.push_back(triangle.z);
          }

          offset += quad_vertices_size;

          for (auto i = 0; i < quad_mesh->GetVerticesAmount(); i++) {
            archetype.position = matrix * glm::vec4(quad_mesh->UnsafeGetVertices()[i].position, 1.0f);
            archetype.normal =
                glm::normalize(glm::vec3(matrix * glm::vec4(quad_mesh->UnsafeGetVertices()[i].normal, 0.0f)));
            archetype.tangent =
                glm::normalize(glm::vec3(matrix * glm::vec4(quad_mesh->UnsafeGetVertices()[i].tangent, 0.0f)));
            archetype.tex_coord = quad_mesh->UnsafeGetVertices()[i].tex_coord;
            skinned_vertices.push_back(archetype);
          }
          for (auto triangle : quad_triangles) {
            triangle.x += offset;
            triangle.y += offset;
            triangle.z += offset;
            indices.push_back(triangle.z);
            indices.push_back(triangle.y);
            indices.push_back(triangle.x);
          }
          offset += quad_vertices_size;
        }
      }
    }

    skinned_mesh->bone_animator_indices = bone_indices_lists;
    SkinnedVertexAttributes attributes{};
    attributes.tex_coord = true;
    skinned_mesh->SetVertices(attributes, skinned_vertices, indices);
    skinned_mesh_renderer->animator = animator;
    skinned_mesh_renderer->skinned_mesh = skinned_mesh;
    skinned_mesh_renderer->material = material;

    animator->Setup(names, offset_matrices);
    skinned_mesh_renderer->SetRagDoll(true);
    skinned_mesh_renderer->SetRagDollBoundEntities(bound_entities, false);
  }

  if (mesh_generator_settings.enable_fruit) {
    const auto fruit_entity = scene->CreateEntity("Animated Fruit Mesh");
    scene->SetParent(fruit_entity, self);
  }

  if (enable_physics) {
    const auto descendants = scene->GetDescendants(rag_doll);
    auto root_rigid_body = scene->GetOrSetPrivateComponent<RigidBody>(rag_doll).lock();
    if (!root_rigid_body->IsKinematic())
      root_rigid_body->SetKinematic(true);
    root_rigid_body->SetEnableGravity(false);
    root_rigid_body->SetLinearDamping(branch_physics_parameters.linear_damping);
    root_rigid_body->SetAngularDamping(branch_physics_parameters.angular_damping);
    root_rigid_body->SetSolverIterations(branch_physics_parameters.position_solver_iteration,
                                       branch_physics_parameters.velocity_solver_iteration);
    root_rigid_body->SetAngularVelocity(glm::vec3(0.0f));
    root_rigid_body->SetLinearVelocity(glm::vec3(0.0f));

    for (const auto& child : descendants) {
      const auto parent = scene->GetParent(child);
      branch_physics_parameters.Link(scene, skeleton, corresponding_entities, parent, child);
    }
  }
}

void Tree::ClearAnimatedGeometryEntities() const {
  const auto scene = GetScene();
  const auto self = GetOwner();
  const auto children = scene->GetChildren(self);
  for (const auto& child : children) {
    auto name = scene->GetEntityName(child);
    if (name == "Rag Doll") {
      scene->DeleteEntity(child);
    } else if (name == "Animated Branch Mesh") {
      scene->DeleteEntity(child);
    } else if (name == "Animated Root Mesh") {
      scene->DeleteEntity(child);
    } else if (name == "Animated Foliage Mesh") {
      scene->DeleteEntity(child);
    } else if (name == "Animated Fruit Mesh") {
      scene->DeleteEntity(child);
    }
  }
}

void Tree::ClearGeometryEntities() const {
  const auto scene = GetScene();
  const auto self = GetOwner();
  const auto children = scene->GetChildren(self);
  for (const auto& child : children) {
    auto name = scene->GetEntityName(child);
    if (name == "Branch Mesh") {
      scene->DeleteEntity(child);
    } else if (name == "Root Mesh") {
      scene->DeleteEntity(child);
    } else if (name == "Foliage Mesh") {
      scene->DeleteEntity(child);
    } else if (name == "Fruit Mesh") {
      scene->DeleteEntity(child);
    }
  }
}

void Tree::ClearStrandRenderer() const {
  const auto scene = GetScene();
  const auto self = GetOwner();
  const auto children = scene->GetChildren(self);
  for (const auto& child : children) {
    auto name = scene->GetEntityName(child);
    if (name == "Branch Strands") {
      scene->DeleteEntity(child);
    }
  }
}

void Tree::GenerateGeometryEntities(const TreeMeshGeneratorSettings& mesh_generator_settings, int iteration) {
  const auto scene = GetScene();
  const auto self = GetOwner();
  const auto children = scene->GetChildren(self);
  auto td = tree_descriptor.Get<TreeDescriptor>();
  ClearGeometryEntities();
  if (auto actual_iteration = iteration; actual_iteration < 0 || actual_iteration > tree_model.CurrentIteration()) {
    actual_iteration = tree_model.CurrentIteration();
  }
  if (mesh_generator_settings.enable_branch) {
    Entity branch_entity = scene->CreateEntity("Branch Mesh");
    scene->SetParent(branch_entity, self);

    auto mesh = GenerateBranchMesh(mesh_generator_settings);
    auto material = ProjectManager::CreateTemporaryAsset<Material>();
    auto mesh_renderer = scene->GetOrSetPrivateComponent<MeshRenderer>(branch_entity).lock();
    if (td) {
      if (const auto shoot_descriptor = td->shoot_descriptor.Get<ShootDescriptor>()) {
        if (const auto shoot_material = shoot_descriptor->bark_material.Get<Material>()) {
          material->SetAlbedoTexture(shoot_material->GetAlbedoTexture());
          material->SetNormalTexture(shoot_material->GetNormalTexture());
          material->SetRoughnessTexture(shoot_material->GetRoughnessTexture());
          material->SetMetallicTexture(shoot_material->GetMetallicTexture());
          material->material_properties = shoot_material->material_properties;
        }
      }
    }
    if (constexpr bool copied_material = false; !copied_material) {
      material->material_properties.albedo_color = glm::vec3(109, 79, 75) / 255.0f;
      material->material_properties.roughness = 1.0f;
      material->material_properties.metallic = 0.0f;
    }
    mesh_renderer->mesh = mesh;
    mesh_renderer->material = material;
  }

  if (mesh_generator_settings.enable_foliage) {
    const auto foliage_entity = scene->CreateEntity("Foliage Mesh");
    scene->SetParent(foliage_entity, self);
    if (mesh_generator_settings.foliage_instancing) {
      const auto mesh = Resources::GetResource<Mesh>("PRIMITIVE_QUAD");
      const auto particle_info_list = GenerateFoliageParticleInfoList(mesh_generator_settings);
      const auto material = ProjectManager::CreateTemporaryAsset<Material>();
      bool copied_material = false;
      if (td) {
        if (const auto foliage_descriptor = td->foliage_descriptor.Get<FoliageDescriptor>()) {
          if (const auto leaf_material = foliage_descriptor->leaf_material.Get<Material>()) {
            material->SetAlbedoTexture(leaf_material->GetAlbedoTexture());
            material->SetNormalTexture(leaf_material->GetNormalTexture());
            material->SetRoughnessTexture(leaf_material->GetRoughnessTexture());
            material->SetMetallicTexture(leaf_material->GetMetallicTexture());
            material->material_properties = leaf_material->material_properties;
            copied_material = true;
          }
        }
      }
      if (!copied_material) {
        material->material_properties.albedo_color = glm::vec3(152 / 255.0f, 203 / 255.0f, 0 / 255.0f);
        material->material_properties.roughness = 1.0f;
        material->material_properties.metallic = 0.0f;
      }
      const auto particles = scene->GetOrSetPrivateComponent<Particles>(foliage_entity).lock();
      particles->mesh = mesh;
      particles->material = material;
      particles->particle_info_list = particle_info_list;
    } else {
      auto mesh = GenerateFoliageMesh(mesh_generator_settings);
      auto mesh_renderer = scene->GetOrSetPrivateComponent<MeshRenderer>(foliage_entity).lock();
      const auto material = ProjectManager::CreateTemporaryAsset<Material>();
      bool copied_material = false;
      if (td) {
        if (const auto foliage_descriptor = td->foliage_descriptor.Get<FoliageDescriptor>()) {
          if (const auto leaf_material = foliage_descriptor->leaf_material.Get<Material>()) {
            material->SetAlbedoTexture(leaf_material->GetAlbedoTexture());
            material->SetNormalTexture(leaf_material->GetNormalTexture());
            material->SetRoughnessTexture(leaf_material->GetRoughnessTexture());
            material->SetMetallicTexture(leaf_material->GetMetallicTexture());
            material->material_properties = leaf_material->material_properties;
            copied_material = true;
          }
        }
      }
      if (!copied_material) {
        material->material_properties.albedo_color = glm::vec3(152 / 255.0f, 203 / 255.0f, 0 / 255.0f);
        material->material_properties.roughness = 1.0f;
        material->material_properties.metallic = 0.0f;
      }
      mesh_renderer->mesh = mesh;
      mesh_renderer->material = material;
    }
  }
  if (mesh_generator_settings.enable_fruit) {
    const auto fruit_entity = scene->CreateEntity("Fruit Mesh");
    scene->SetParent(fruit_entity, self);
  }
}

void Tree::RegisterVoxel() {
  const auto scene = GetScene();
  const auto owner = GetOwner();
  const auto global_transform = scene->GetDataComponent<GlobalTransform>(owner).value;
  tree_model.shoot_skeleton_.data.index = owner.GetIndex();
  const auto c = climate.Get<Climate>();
  tree_model.RegisterVoxel(global_transform, c->climate_model, shoot_growth_controller_);
}

void Tree::FromLSystemString(const std::shared_ptr<LSystemString>& l_system_string) {
}

void Tree::FromTreeGraph(const std::shared_ptr<TreeGraph>& tree_graph) {
}

void Tree::FromTreeGraphV2(const std::shared_ptr<TreeGraphV2>& tree_graph_v2) {
}

void Tree::GenerateTreeParts(const TreeMeshGeneratorSettings& mesh_generator_settings,
                             std::vector<TreePartData>& tree_parts) {
  auto td = tree_descriptor.Get<TreeDescriptor>();
  if (!td) {
    EVOENGINE_WARNING("TreeDescriptor missing!");
    td = ProjectManager::CreateTemporaryAsset<TreeDescriptor>();
    td->foliage_descriptor = ProjectManager::CreateTemporaryAsset<FoliageDescriptor>();
  }
  auto fd = td->foliage_descriptor.Get<FoliageDescriptor>();
  if (!fd)
    fd = ProjectManager::CreateTemporaryAsset<FoliageDescriptor>();

  const auto& skeleton = tree_model.RefShootSkeleton();
  const auto& sorted_internode_list = skeleton.PeekSortedNodeList();

  std::unordered_map<SkeletonNodeHandle, TreePartInfo> tree_part_infos{};
  int next_line_index = 0;
  for (int internode_handle : sorted_internode_list) {
    const auto& internode = skeleton.PeekNode(internode_handle);
    const auto& internode_info = internode.info;

    auto parent_internode_handle = internode.GetParentHandle();
    const auto flow_handle = internode.GetFlowHandle();
    const auto& flow = skeleton.PeekFlow(flow_handle);
    const auto& chain_handles = flow.PeekNodeHandles();
    const bool has_multiple_children = flow.PeekChildHandles().size() > 1;
    bool only_child = true;
    const auto parent_flow_handle = flow.GetParentHandle();
    float distance_to_chain_start = 0;
    float distance_to_chain_end = 0;
    const auto chain_size = chain_handles.size();
    for (int i = 0; i < chain_size; i++) {
      if (chain_handles[i] == internode_handle)
        break;
      distance_to_chain_start += skeleton.PeekNode(chain_handles[i]).info.length;
    }
    distance_to_chain_end = flow.info.flow_length - distance_to_chain_start - internode.info.length;
    float compare_radius = internode.info.thickness;
    if (parent_flow_handle != -1) {
      const auto& parent_flow = skeleton.PeekFlow(parent_flow_handle);
      only_child = parent_flow.PeekChildHandles().size() <= 1;
      compare_radius = parent_flow.info.end_thickness;
    }
    int tree_part_type = 0;
    if (has_multiple_children && distance_to_chain_end <= mesh_generator_settings.tree_part_base_distance * compare_radius) {
      tree_part_type = 1;
    } else if (!only_child && distance_to_chain_start <= mesh_generator_settings.tree_part_end_distance * compare_radius) {
      tree_part_type = 2;
    }
    int current_tree_part_index = -1;
    int current_line_index = -1;
    if (tree_part_type == 0) {
      // IShape
      // If root or parent is Y Shape or length exceeds limit, create a new IShape from this node.
      bool restart_i_shape = parent_internode_handle == -1 || tree_part_infos[parent_internode_handle].tree_part_type != 0;
      if (!restart_i_shape) {
        if (const auto& parent_junction_info = tree_part_infos[parent_internode_handle]; parent_junction_info.distance_to_start / internode_info.thickness > mesh_generator_settings.tree_part_break_ratio)
          restart_i_shape = true;
      }
      if (restart_i_shape) {
        TreePartInfo tree_part_info;
        tree_part_info.tree_part_type = 0;
        tree_part_info.tree_part_index = tree_parts.size();
        tree_part_info.line_index = next_line_index;
        tree_part_info.distance_to_start = 0.0f;
        tree_part_infos[internode_handle] = tree_part_info;
        tree_parts.emplace_back();
        auto& tree_part = tree_parts.back();
        tree_part.is_junction = false;
        tree_part.num_of_leaves = 0;
        current_tree_part_index = tree_part.tree_part_index = tree_part_info.tree_part_index;

        current_line_index = next_line_index;
        next_line_index++;
      } else {
        auto& current_tree_part_info = tree_part_infos[internode_handle];
        current_tree_part_info = tree_part_infos[parent_internode_handle];
        current_tree_part_info.distance_to_start += internode_info.length;
        current_tree_part_info.tree_part_type = 0;
        current_tree_part_index = current_tree_part_info.tree_part_index;

        current_line_index = current_tree_part_info.line_index;
      }
    } else if (tree_part_type == 1) {
      // Base of Y Shape
      if (parent_internode_handle == -1 || tree_part_infos[parent_internode_handle].tree_part_type != 1 ||
          tree_part_infos[parent_internode_handle].base_flow_handle != flow_handle) {
        TreePartInfo tree_part_info;
        tree_part_info.tree_part_type = 1;
        tree_part_info.tree_part_index = tree_parts.size();
        tree_part_info.line_index = next_line_index;
        tree_part_info.distance_to_start = 0.0f;
        tree_part_info.base_flow_handle = flow_handle;
        tree_part_infos[internode_handle] = tree_part_info;
        tree_parts.emplace_back();
        auto& tree_part = tree_parts.back();
        tree_part.is_junction = true;
        tree_part.num_of_leaves = 0;
        current_tree_part_index = tree_part.tree_part_index = tree_part_info.tree_part_index;

        current_line_index = next_line_index;
        next_line_index++;
      } else {
        auto& current_tree_part_info = tree_part_infos[internode_handle];
        current_tree_part_info = tree_part_infos[parent_internode_handle];
        current_tree_part_info.tree_part_type = 1;
        current_tree_part_index = current_tree_part_info.tree_part_index;

        current_line_index = current_tree_part_info.line_index;
      }
    } else if (tree_part_type == 2) {
      // Branch of Y Shape
      if (parent_internode_handle == -1 || tree_part_infos[parent_internode_handle].tree_part_type == 0 ||
          tree_part_infos[parent_internode_handle].base_flow_handle != parent_flow_handle) {
        EVOENGINE_ERROR("Error!");
      }

      auto& current_tree_part_info = tree_part_infos[internode_handle];
      current_tree_part_info = tree_part_infos[parent_internode_handle];
      if (current_tree_part_info.tree_part_type != 2) {
        current_tree_part_info.line_index = next_line_index;
        next_line_index++;
      }
      current_tree_part_info.tree_part_type = 2;
      current_tree_part_index = current_tree_part_info.tree_part_index;

      current_line_index = current_tree_part_info.line_index;
    }
    auto& tree_part = tree_parts[current_tree_part_index];
    tree_part.node_handles.emplace_back(internode_handle);
    tree_part.is_end.emplace_back(true);
    tree_part.line_index.emplace_back(current_line_index);
    for (int i = 0; i < tree_part.node_handles.size(); i++) {
      if (tree_part.node_handles[i] == parent_internode_handle) {
        tree_part.is_end[i] = false;
        break;
      }
    }
  }
  for (int internode_handle : sorted_internode_list) {
    const auto& internode = skeleton.PeekNode(internode_handle);
    const auto& internode_info = internode.info;
    std::vector<glm::mat4> leaf_matrices;
    const auto tree_dim = skeleton.max - skeleton.min;
    fd->GenerateFoliageMatrices(leaf_matrices, internode_info, glm::length(tree_dim));

    auto& current_tree_part_info = tree_part_infos[internode_handle];
    auto& tree_part = tree_parts[current_tree_part_info.tree_part_index];
    tree_part.num_of_leaves += leaf_matrices.size();
  }
  for (auto& tree_part : tree_parts) {
    const auto& start_internode = skeleton.PeekNode(tree_part.node_handles.front());
    if (tree_part.is_junction) {
      const auto& base_node = skeleton.PeekNode(tree_part.node_handles.front());
      const auto& flow = skeleton.PeekFlow(base_node.GetFlowHandle());
      const auto& chain_handles = flow.PeekNodeHandles();
      const auto center_internode_handle = chain_handles.back();
      const auto& center_internode = skeleton.PeekNode(center_internode_handle);
      tree_part.base_line.start_position = start_internode.info.global_position;
      tree_part.base_line.start_radius = start_internode.info.thickness;
      tree_part.base_line.end_position = center_internode.info.GetGlobalEndPosition();
      tree_part.base_line.end_radius = center_internode.info.thickness;

      tree_part.base_line.start_direction = start_internode.info.GetGlobalDirection();
      tree_part.base_line.end_direction = center_internode.info.GetGlobalDirection();

      tree_part.base_line.line_index = tree_part.line_index.front();
      for (int i = 1; i < tree_part.node_handles.size(); i++) {
        if (tree_part.is_end[i]) {
          const auto& end_internode = skeleton.PeekNode(tree_part.node_handles[i]);
          tree_part.children_lines.emplace_back();
          auto& new_line = tree_part.children_lines.back();
          new_line.start_position = center_internode.info.GetGlobalEndPosition();
          new_line.start_radius = center_internode.info.thickness;
          new_line.end_position = end_internode.info.GetGlobalEndPosition();
          new_line.end_radius = end_internode.info.thickness;

          new_line.start_direction = center_internode.info.GetGlobalDirection();
          new_line.end_direction = end_internode.info.GetGlobalDirection();

          new_line.line_index = tree_part.line_index[i];
        }
      }
    } else {
      const auto& end_internode = skeleton.PeekNode(tree_part.node_handles.back());
      tree_part.base_line.start_position = start_internode.info.global_position;
      tree_part.base_line.start_radius = start_internode.info.thickness;
      tree_part.base_line.end_position = end_internode.info.GetGlobalEndPosition();
      tree_part.base_line.end_radius = end_internode.info.thickness;

      tree_part.base_line.start_direction = start_internode.info.GetGlobalDirection();
      tree_part.base_line.end_direction = end_internode.info.GetGlobalDirection();

      tree_part.base_line.line_index = tree_part.line_index.front();
    }
  }
}

void Tree::ExportTreeParts(const TreeMeshGeneratorSettings& mesh_generator_settings, treeio::json& out) {
}

void Tree::ExportTreeParts(const TreeMeshGeneratorSettings& mesh_generator_settings, YAML::Emitter& out) {
  out << YAML::Key << "Tree" << YAML::Value << YAML::BeginMap;
  {
    std::vector<TreePartData> tree_parts{};
    GenerateTreeParts(mesh_generator_settings, tree_parts);
    std::unordered_set<int> line_index_check{};
    out << YAML::Key << "TreeParts" << YAML::Value << YAML::BeginSeq;
    for (const auto& tree_part : tree_parts) {
      out << YAML::BeginMap;
      out << YAML::Key << "J" << YAML::Value << (tree_part.is_junction ? 1 : 0);
      out << YAML::Key << "I" << YAML::Value << tree_part.tree_part_index + 1;
      out << YAML::Key << "LI" << YAML::Value << tree_part.base_line.line_index + 1;

      out << YAML::Key << "F" << YAML::Value << tree_part.num_of_leaves;
      /*
      if (lineIndexCheck.find(treePart.base_line.line_index) != lineIndexCheck.end())
      {
              EVOENGINE_ERROR("Duplicate!");
      }
      lineIndexCheck.emplace(treePart.base_line.line_index);*/
      out << YAML::Key << "BSP" << YAML::Value << tree_part.base_line.start_position;
      out << YAML::Key << "BEP" << YAML::Value << tree_part.base_line.end_position;
      out << YAML::Key << "BSR" << YAML::Value << tree_part.base_line.start_radius;
      out << YAML::Key << "BER" << YAML::Value << tree_part.base_line.end_radius;
      out << YAML::Key << "BSD" << YAML::Value << tree_part.base_line.start_direction;
      out << YAML::Key << "BED" << YAML::Value << tree_part.base_line.end_direction;

      out << YAML::Key << "C" << YAML::Value << YAML::BeginSeq;
      if (tree_part.children_lines.size() > 3) {
        EVOENGINE_ERROR("Too many child!");
      }
      for (const auto& child_line : tree_part.children_lines) {
        out << YAML::BeginMap;
        out << YAML::Key << "LI" << YAML::Value << child_line.line_index + 1;
        /*
        if (lineIndexCheck.find(childLine.line_index) != lineIndexCheck.end())
        {
                EVOENGINE_ERROR("Duplicate!");
        }
        lineIndexCheck.emplace(childLine.line_index);*/
        out << YAML::Key << "SP" << YAML::Value << child_line.start_position;
        out << YAML::Key << "EP" << YAML::Value << child_line.end_position;
        out << YAML::Key << "SR" << YAML::Value << child_line.start_radius;
        out << YAML::Key << "ER" << YAML::Value << child_line.end_radius;
        out << YAML::Key << "SD" << YAML::Value << child_line.start_direction;
        out << YAML::Key << "ED" << YAML::Value << child_line.end_direction;
        out << YAML::EndMap;
      }
      out << YAML::EndSeq;
      out << YAML::EndMap;
    }
    out << YAML::EndSeq;
  }
  out << YAML::EndMap;
}

void Tree::ExportTreeParts(const TreeMeshGeneratorSettings& mesh_generator_settings, const std::filesystem::path& path) {
  try {
    auto directory = path;
    directory.remove_filename();
    std::filesystem::create_directories(directory);
    YAML::Emitter out;
    ExportTreeParts(mesh_generator_settings, out);
    std::ofstream output_file(path.string());
    output_file << out.c_str();
    output_file.flush();
  } catch (const std::exception& e) {
    EVOENGINE_ERROR("Failed to save!");
  }
}

bool Tree::ExportIoTree(const std::filesystem::path& path) const {
  treeio::ArrayTree tree{};
  using namespace treeio;
  const auto& shoot_skeleton = tree_model.PeekShootSkeleton();
  const auto& sorted_internode_list = shoot_skeleton.PeekSortedNodeList();
  if (sorted_internode_list.empty())
    return false;
  const auto& root_node = shoot_skeleton.PeekNode(0);
  TreeNodeData root_node_data;
  // rootNodeData.direction = rootNode.info.regulated_global_rotation * glm::vec3(0, 0, -1);
  root_node_data.thickness = root_node.info.thickness;
  root_node_data.pos = root_node.info.global_position;

  auto root_id = tree.addRoot(root_node_data);
  std::unordered_map<SkeletonNodeHandle, size_t> node_map;
  node_map[0] = root_id;
  for (const auto& node_handle : sorted_internode_list) {
    if (node_handle == 0)
      continue;
    const auto& node = shoot_skeleton.PeekNode(node_handle);
    TreeNodeData node_data;
    // nodeData.direction = node.info.regulated_global_rotation * glm::vec3(0, 0, -1);
    node_data.thickness = node.info.thickness;
    node_data.pos = node.info.global_position;

    auto current_id = tree.addNodeChild(node_map[node.GetParentHandle()], node_data);
    node_map[node_handle] = current_id;
  }
  return tree.saveTree(path.string());
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
  if (tree_descriptor.Get<TreeDescriptor>()) {
    list.emplace_back(tree_descriptor);
  }
}

void BranchPhysicsParameters::Serialize(YAML::Emitter& out) {
  out << YAML::Key << "density" << YAML::Value << density;
  out << YAML::Key << "linear_damping" << YAML::Value << linear_damping;
  out << YAML::Key << "angular_damping" << YAML::Value << angular_damping;
  out << YAML::Key << "position_solver_iteration" << YAML::Value << position_solver_iteration;
  out << YAML::Key << "velocity_solver_iteration" << YAML::Value << velocity_solver_iteration;
  out << YAML::Key << "joint_drive_stiffness" << YAML::Value << joint_drive_stiffness;
  out << YAML::Key << "joint_drive_stiffness_thickness_factor" << YAML::Value << joint_drive_stiffness_thickness_factor;
  out << YAML::Key << "joint_drive_damping" << YAML::Value << joint_drive_damping;
  out << YAML::Key << "joint_drive_damping_thickness_factor" << YAML::Value << joint_drive_damping_thickness_factor;
  out << YAML::Key << "enable_acceleration_for_drive" << YAML::Value << enable_acceleration_for_drive;
  out << YAML::Key << "minimum_thickness" << YAML::Value << minimum_thickness;
}
void BranchPhysicsParameters::Deserialize(const YAML::Node& in) {
  if (in["density"])
    density = in["density"].as<float>();
  if (in["linear_damping"])
    linear_damping = in["linear_damping"].as<float>();
  if (in["angular_damping"])
    angular_damping = in["angular_damping"].as<float>();
  if (in["position_solver_iteration"])
    position_solver_iteration = in["position_solver_iteration"].as<int>();
  if (in["velocity_solver_iteration"])
    velocity_solver_iteration = in["velocity_solver_iteration"].as<int>();
  if (in["joint_drive_stiffness"])
    joint_drive_stiffness = in["joint_drive_stiffness"].as<float>();
  if (in["joint_drive_stiffness_thickness_factor"])
    joint_drive_stiffness_thickness_factor = in["joint_drive_stiffness_thickness_factor"].as<float>();
  if (in["joint_drive_damping"])
    joint_drive_damping = in["joint_drive_damping"].as<float>();
  if (in["joint_drive_damping_thickness_factor"])
    joint_drive_damping_thickness_factor = in["joint_drive_damping_thickness_factor"].as<float>();
  if (in["enable_acceleration_for_drive"])
    enable_acceleration_for_drive = in["enable_acceleration_for_drive"].as<bool>();
  if (in["minimum_thickness"])
    minimum_thickness = in["minimum_thickness"].as<float>();
}

void BranchPhysicsParameters::OnInspect() {
  if (ImGui::TreeNodeEx("Physics Parameters")) {
    ImGui::DragFloat("Internode Density", &density, 0.1f, 0.01f, 1000.0f);
    ImGui::DragFloat("RigidBody Linear Damping", &linear_damping, 0.1f, 0.01f, 1000.0f);
    ImGui::DragFloat("RigidBody Angular Damping", &angular_damping, 0.1f, 0.01f, 1000.0f);
    ImGui::DragFloat("Drive Stiffness", &joint_drive_stiffness, 0.1f, 0.01f, 1000000.0f);
    ImGui::DragFloat("Drive Stiffness Thickness Factor", &joint_drive_stiffness_thickness_factor, 0.1f, 0.01f,
                     1000000.0f);
    ImGui::DragFloat("Drive Damping", &joint_drive_damping, 0.1f, 0.01f, 1000000.0f);
    ImGui::DragFloat("Drive Damping Thickness Factor", &joint_drive_damping_thickness_factor, 0.1f, 0.01f, 1000000.0f);
    ImGui::Checkbox("Use acceleration", &enable_acceleration_for_drive);

    int pi = position_solver_iteration;
    int vi = velocity_solver_iteration;
    if (ImGui::DragInt("Velocity solver iteration", &vi, 1, 1, 100)) {
      velocity_solver_iteration = vi;
    }
    if (ImGui::DragInt("Position solver iteration", &pi, 1, 1, 100)) {
      position_solver_iteration = pi;
    }

    ImGui::DragFloat("Minimum thickness", &minimum_thickness, 0.001f, 0.001f, 100.0f);
    ImGui::TreePop();
  }
}

void SkeletalGraphSettings::OnInspect() {
  ImGui::DragFloat("Line thickness", &line_thickness, 0.001f, 0.0f, 1.0f);
  ImGui::DragFloat("Fixed line thickness", &fixed_line_thickness, 0.001f, 0.0f, 1.0f);
  ImGui::DragFloat("Branch point size", &branch_point_size, 0.01f, 0.0f, 1.0f);
  ImGui::DragFloat("Junction point size", &junction_point_size, 0.01f, 0.0f, 1.0f);

  ImGui::Checkbox("Fixed point size", &fixed_point_size);
  if (fixed_point_size) {
    ImGui::DragFloat("Fixed point size multiplier", &fixed_point_size_factor, 0.001f, 0.0f, 1.0f);
  }

  ImGui::ColorEdit4("Line color", &line_color.x);
  ImGui::ColorEdit4("Branch point color", &branch_point_color.x);
  ImGui::ColorEdit4("Junction point color", &junction_point_color.x);
}

void Tree::PrepareController(const std::shared_ptr<ShootDescriptor>& shoot_descriptor, const std::shared_ptr<Soil>& soil,
                             const std::shared_ptr<Climate>& climate) {
  shoot_descriptor->PrepareController(shoot_growth_controller_);

  shoot_growth_controller_.m_endToRootPruningFactor = [&](const glm::mat4&, ClimateModel& ,
                                                         const ShootSkeleton& ,
                                                         const SkeletonNode<InternodeGrowthData>& internode) {
    if (shoot_descriptor->trunk_protection && internode.data.order == 0) {
      return 0.f;
    }
    float pruning_probability = 0.0f;
    if (shoot_descriptor->light_pruning_factor != 0.f) {
      if (internode.IsEndNode()) {
        if (internode.data.light_intensity < shoot_descriptor->light_pruning_factor) {
          pruning_probability += 999.f;
        }
      }
    }
    if (internode.data.sagging_stress > 1.) {
      pruning_probability += shoot_descriptor->branch_breaking_multiplier *
                            glm::pow(internode.data.sagging_stress, shoot_descriptor->branch_breaking_multiplier);
    }
    return pruning_probability;
  };
  shoot_growth_controller_.m_rootToEndPruningFactor = [&](const glm::mat4& global_transform, ClimateModel& climate_model,
                                                         const ShootSkeleton& shoot_skeleton,
                                                         const SkeletonNode<InternodeGrowthData>& internode) {
    if (shoot_descriptor->trunk_protection && internode.data.order == 0) {
      return 0.f;
    }

    if (shoot_descriptor->max_flow_length != 0 && shoot_descriptor->max_flow_length < internode.info.chain_index) {
      return 999.f;
    }
    if (const auto max_distance = shoot_skeleton.PeekNode(0).info.end_distance; max_distance > 5.0f * shoot_growth_controller_.m_internodeLength && internode.data.order > 0 &&
                                                                               internode.info.root_distance / max_distance < low_branch_pruning) {
      if (const auto parent_handle = internode.GetParentHandle(); parent_handle != -1) {
        const auto& parent = shoot_skeleton.PeekNode(parent_handle);
        if (parent.PeekChildHandles().size() > 1) {
          return 999.f;
        }
      }
    }
    if (crown_shyness_distance > 0.f && internode.IsEndNode()) {
      const glm::vec3 end_position = global_transform * glm::vec4(internode.info.GetGlobalEndPosition(), 1.0f);
      bool prune_by_crown_shyness = false;
      climate_model.environment_grid.voxel_grid.ForEach(
          end_position, crown_shyness_distance * 2.0f, [&](const EnvironmentVoxel& data) {
            if (prune_by_crown_shyness)
              return;
            for (const auto& i : data.internode_voxel_registrations) {
              if (i.tree_skeleton_index == shoot_skeleton.data.index)
                continue;
              if (glm::distance(end_position, i.position) < crown_shyness_distance)
                prune_by_crown_shyness = true;
            }
          });
      if (prune_by_crown_shyness)
        return 999.f;
    }
    constexpr float pruningProbability = 0.0f;
    return pruningProbability;
  };
}

void Tree::InitializeStrandRenderer() {
  const auto scene = GetScene();
  const auto owner = GetOwner();

  ClearStrandRenderer();
  if (strand_model.strand_model_skeleton.RefRawNodes().size() !=
      tree_model.PeekShootSkeleton().PeekRawNodes().size()) {
    BuildStrandModel();
  }
  const auto strands_entity = scene->CreateEntity("Branch Strands");
  scene->SetParent(strands_entity, owner);

  const auto renderer = scene->GetOrSetPrivateComponent<StrandsRenderer>(strands_entity).lock();
  renderer->strands = GenerateStrands();

  const auto material = ProjectManager::CreateTemporaryAsset<Material>();

  renderer->material = material;
  material->vertex_color_only = true;
  material->material_properties.albedo_color = glm::vec3(0.6f, 0.3f, 0.0f);
}

void Tree::InitializeStrandRenderer(const std::shared_ptr<Strands>& strands) const {
  const auto scene = GetScene();
  const auto owner = GetOwner();

  ClearStrandRenderer();

  const auto strands_entity = scene->CreateEntity("Branch Strands");
  scene->SetParent(strands_entity, owner);

  const auto renderer = scene->GetOrSetPrivateComponent<StrandsRenderer>(strands_entity).lock();

  renderer->strands = strands;

  const auto material = ProjectManager::CreateTemporaryAsset<Material>();

  renderer->material = material;
  material->vertex_color_only = true;
  material->material_properties.albedo_color = glm::vec3(0.6f, 0.3f, 0.0f);
}

void Tree::InitializeStrandModelMeshRenderer(const StrandModelMeshGeneratorSettings& strand_model_mesh_generator_settings) {
  ClearStrandModelMeshRenderer();
  if (strand_model.strand_model_skeleton.RefRawNodes().size() !=
      tree_model.PeekShootSkeleton().PeekRawNodes().size()) {
    BuildStrandModel();
  }
  const float time = Times::Now();
  const auto scene = GetScene();
  const auto self = GetOwner();
  const auto td = tree_descriptor.Get<TreeDescriptor>();
  if (strand_model_mesh_generator_settings.enable_branch) {
    const auto foliage_entity = scene->CreateEntity("Strand Model Branch Mesh");
    scene->SetParent(foliage_entity, self);

    const auto mesh = GenerateStrandModelBranchMesh(strand_model_mesh_generator_settings);
    const auto material = ProjectManager::CreateTemporaryAsset<Material>();
    const auto mesh_renderer = scene->GetOrSetPrivateComponent<MeshRenderer>(foliage_entity).lock();

    if (td) {
      if (const auto sd = td->shoot_descriptor.Get<ShootDescriptor>()) {
        if (const auto shoot_material = sd->bark_material.Get<Material>()) {
          material->SetAlbedoTexture(shoot_material->GetAlbedoTexture());
          material->SetNormalTexture(shoot_material->GetNormalTexture());
          material->SetRoughnessTexture(shoot_material->GetRoughnessTexture());
          material->SetMetallicTexture(shoot_material->GetMetallicTexture());
          material->material_properties = shoot_material->material_properties;
        }
      }
    }
    if (bool copied_material = false; !copied_material) {
      material->material_properties.albedo_color = glm::vec3(109, 79, 75) / 255.0f;
      material->material_properties.roughness = 1.0f;
      material->material_properties.metallic = 0.0f;
    }

    mesh_renderer->mesh = mesh;
    mesh_renderer->material = material;
  }
  if (strand_model_mesh_generator_settings.enable_foliage) {
    const Entity foliage_entity = scene->CreateEntity("Strand Model Foliage Mesh");
    scene->SetParent(foliage_entity, self);

    const auto mesh = GenerateStrandModelFoliageMesh(strand_model_mesh_generator_settings);
    const auto material = ProjectManager::CreateTemporaryAsset<Material>();
    bool copied_material = false;
    if (td) {
      if (const auto fd = td->foliage_descriptor.Get<FoliageDescriptor>()) {
        if (const auto leaf_material = fd->leaf_material.Get<Material>()) {
          material->SetAlbedoTexture(leaf_material->GetAlbedoTexture());
          material->SetNormalTexture(leaf_material->GetNormalTexture());
          material->SetRoughnessTexture(leaf_material->GetRoughnessTexture());
          material->SetMetallicTexture(leaf_material->GetMetallicTexture());
          material->material_properties = leaf_material->material_properties;
          copied_material = true;
        }
      }
    }
    if (!copied_material) {
      material->material_properties.albedo_color = glm::vec3(152 / 255.0f, 203 / 255.0f, 0 / 255.0f);
      material->material_properties.roughness = 1.0f;
      material->material_properties.metallic = 0.0f;
    }
    const auto mesh_renderer = scene->GetOrSetPrivateComponent<MeshRenderer>(foliage_entity).lock();
    mesh_renderer->mesh = mesh;
    mesh_renderer->material = material;
  }
  std::string output;
  const float mesh_generation_time = Times::Now() - time;
  output += "\nMesh generation Used time: " + std::to_string(mesh_generation_time) + "\n";
  EVOENGINE_LOG(output);
}

void Tree::ClearStrandModelMeshRenderer() const {
  const auto scene = GetScene();
  const auto self = GetOwner();
  const auto children = scene->GetChildren(self);
  for (const auto& child : children) {
    auto name = scene->GetEntityName(child);
    if (name == "Strand Model Branch Mesh") {
      scene->DeleteEntity(child);
    } else if (name == "Strand Model Foliage Mesh") {
      scene->DeleteEntity(child);
    }
  }
}
