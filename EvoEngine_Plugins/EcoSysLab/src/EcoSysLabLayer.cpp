//
// Created by lllll on 11/1/2022.
//

#include "EcoSysLabLayer.hpp"
#include "Times.hpp"
#ifdef BILLBOARD_CLOUDS_PLUGIN
#  include "BillboardCloudsConverter.hpp"
#endif
#include "ClassRegistry.hpp"
#include "Climate.hpp"
#include "CubeVolume.hpp"
#include "DynamicStrandsDemo.hpp"
#include "DynamicStrandsVisualizationParameters.hpp"
#include "DynamicTreeSkeleton.hpp"
#include "DynamicTreeStrands.hpp"
#include "ForestDescriptor.hpp"
#include "Prefab.hpp"
#include "Shader.hpp"
#include "Soil.hpp"
#include "SpatialPlantDistributionSimulator.hpp"
#include "Tree.hpp"
#include "TreeStructor.hpp"
#include "Serialization.hpp"
#include <yaml-cpp/yaml.h>
using namespace eco_sys_lab_plugin;

PrivateComponentRegistration<TreeStructor> tree_structor_registry("TreeStructor");
PrivateComponentRegistration<Climate> climate_registry("Climate");

PrivateComponentRegistration<SpatialPlantDistributionSimulator> spds_registry("SpatialPlantDistributionSimulator");
PrivateComponentRegistration<DynamicTreeSkeleton> dynamic_tree_skeleton_registry("DynamicTreeSkeleton");

AssetRegistration<ClimateDescriptor> climate_d_registry("ClimateDescriptor", {".climate"});
AssetRegistration<RadialBoundingVolume> rbv_registry("RadialBoundingVolume", {".rbv"});
AssetRegistration<CubeVolume> cube_volume_registry("CubeVolume", {".cubevolume"});

AssetRegistration<ForestPatch> forest_patch_registry("ForestPatch", {".forestpatch"});

PrivateComponentRegistration<DynamicStrandsDemo> dynamic_strands_demo_registry("DynamicStrandsDemo");

#ifdef BILLBOARD_CLOUDS_PLUGIN
PrivateComponentRegistration<BillboardCloudsConverter> billboard_clouds_converter_register("BillboardCloudsConverter");
#endif

void EcoSysLabLayer::OnCreate() {
  Shader::RegisterShaderIncludePath(std::filesystem::path("./EcoSysLabResources/Shaders/Includes"));
  if (random_colors_.empty()) {
    for (int i = 0; i < 20000; i++) {
      random_colors_.emplace_back(glm::linearRand(glm::vec3(0.0f), glm::vec3(1.0f)));
    }
  }

  shoot_stem_strands_ = AssetManager::CreateTemporaryAsset<Strands>();
  soil_matrices_ = AssetManager::CreateTemporaryAsset<ParticleInfoList>();
  bounding_box_matrices_ = AssetManager::CreateTemporaryAsset<ParticleInfoList>();
  foliage_matrices_ = AssetManager::CreateTemporaryAsset<ParticleInfoList>();
  flower_matrices_ = AssetManager::CreateTemporaryAsset<ParticleInfoList>();
  fruit_matrices_ = AssetManager::CreateTemporaryAsset<ParticleInfoList>();

  ground_fruit_matrices_ = AssetManager::CreateTemporaryAsset<ParticleInfoList>();
  ground_flower_matrices_ = AssetManager::CreateTemporaryAsset<ParticleInfoList>();
  ground_leaf_matrices_ = AssetManager::CreateTemporaryAsset<ParticleInfoList>();
  vector_matrices_ = AssetManager::CreateTemporaryAsset<ParticleInfoList>();
  scalar_matrices_ = AssetManager::CreateTemporaryAsset<ParticleInfoList>();
  shadow_grid_particle_info_list_ = AssetManager::CreateTemporaryAsset<ParticleInfoList>();
  lighting_grid_particle_info_list_ = AssetManager::CreateTemporaryAsset<ParticleInfoList>();
#pragma region Internode camera
  visualization_camera_ = Serialization::ProduceSerializable<Camera>();

  visualization_camera_->OnCreate();
  visualization_camera_->camera_settings.use_clear_color = true;
  visualization_camera_->camera_settings.clear_color = glm::vec4(0.5f, 0.5f, 0.5f, 1.f);
#pragma endregion

  if (const auto editor_layer = Application::GetLayer<EditorLayer>()) {
    editor_layer->RegisterEditorCamera(visualization_camera_);
  }

  Soil::InitializeTerrainPipeline();
}

void EcoSysLabLayer::Serialize(YAML::Emitter& out) const {
  auto emit_mat3 = [&](const char* key, const glm::mat3& value) {
    out << YAML::Key << key << YAML::Value << YAML::BeginSeq;
    for (int c = 0; c < 3; c++) {
      for (int r = 0; r < 3; r++) {
        out << value[c][r];
      }
    }
    out << YAML::EndSeq;
  };

  out << YAML::Key << "show_visualization_camera_info" << YAML::Value << show_visualization_camera_info;
  out << YAML::Key << "enable_visualization_background" << YAML::Value << enable_visualization_background;
  out << YAML::Key << "need_full_flow_update" << YAML::Value << need_full_flow_update;
  out << YAML::Key << "tree_operator_mode" << YAML::Value << tree_operator_mode;
  out << YAML::Key << "tree_reduce_rate" << YAML::Value << tree_reduce_rate;

  out << YAML::Key << "auto_generate_mesh_after_editing" << YAML::Value << auto_generate_mesh_after_editing_;
  out << YAML::Key << "auto_generate_skeletal_graph_every_frame" << YAML::Value << auto_generate_skeletal_graph_every_frame_;
  out << YAML::Key << "auto_generate_strands_after_editing" << YAML::Value << auto_generate_strands_after_editing_;
  out << YAML::Key << "auto_generate_strand_mesh_after_editing" << YAML::Value << auto_generate_strand_mesh_after_editing_;
  out << YAML::Key << "auto_update_strand_renderer" << YAML::Value << auto_update_strand_renderer_;
  out << YAML::Key << "auto_update_strand_renderer_interval" << YAML::Value << auto_update_strand_renderer_interval_;
  out << YAML::Key << "auto_update_strand_model_mesh" << YAML::Value << auto_update_strand_model_mesh_;
  out << YAML::Key << "auto_update_strand_model_mesh_interval" << YAML::Value << auto_update_strand_model_mesh_interval_;

  out << YAML::Key << "simulated_time" << YAML::Value << simulated_time_;
  out << YAML::Key << "auto_time_grow" << YAML::Value << auto_time_grow_;
  out << YAML::Key << "auto_time_target" << YAML::Value << auto_time_target_;
  out << YAML::Key << "extra_time_years" << YAML::Value << extra_time_years_;

  uint64_t selected_tree_handle = 0;
  if (const auto scene = Application::GetActiveScene(); scene && scene->IsEntityValid(selected_tree)) {
    selected_tree_handle = scene->GetEntityHandle(selected_tree).GetValue();
  }
  out << YAML::Key << "selected_tree" << YAML::Value << selected_tree_handle;

  uint64_t selected_entity_handle = 0;
  if (const auto editor_layer = Application::GetLayer<EditorLayer>()) {
    if (const auto scene = Application::GetActiveScene(); scene && scene->IsEntityValid(editor_layer->GetSelectedEntity())) {
      selected_entity_handle = scene->GetEntityHandle(editor_layer->GetSelectedEntity()).GetValue();
    }
  }
  out << YAML::Key << "selected_entity" << YAML::Value << selected_entity_handle;

  simulation_settings.Save("simulation_settings", out);
  mesh_generator_settings.Save("mesh_generator_settings", out);
  skeletal_graph_settings.Save("skeletal_graph_settings", out);

  out << YAML::Key << "tree_visualization_settings" << YAML::Value << YAML::BeginMap;
  out << YAML::Key << "enable" << YAML::Value << tree_visualization_settings_.enable;
  out << YAML::Key << "display_shoot_stem" << YAML::Value << tree_visualization_settings_.display_shoot_stem;
  out << YAML::Key << "display_foliage" << YAML::Value << tree_visualization_settings_.display_foliage;
  out << YAML::Key << "display_flowers" << YAML::Value << tree_visualization_settings_.display_flowers;
  out << YAML::Key << "display_fruits" << YAML::Value << tree_visualization_settings_.display_fruits;
  out << YAML::Key << "display_bounding_box" << YAML::Value << tree_visualization_settings_.display_bounding_box;
  out << YAML::Key << "display_ground_flowers" << YAML::Value << tree_visualization_settings_.display_ground_flowers;
  out << YAML::Key << "display_ground_fruits" << YAML::Value << tree_visualization_settings_.display_ground_fruits;
  out << YAML::Key << "display_ground_leaves" << YAML::Value << tree_visualization_settings_.display_ground_leaves;
  out << YAML::Key << "show_shadow_grid" << YAML::Value << tree_visualization_settings_.show_shadow_grid;
  out << YAML::Key << "show_lighting_grid" << YAML::Value << tree_visualization_settings_.show_lighting_grid;
  out << YAML::EndMap;

  out << YAML::Key << "strand_mesh_generator_settings" << YAML::Value << YAML::BeginMap;
  out << YAML::Key << "generator_type" << YAML::Value << strand_mesh_generator_settings.generator_type;
  out << YAML::Key << "steps_per_segment" << YAML::Value << strand_mesh_generator_settings.steps_per_segment;
  out << YAML::Key << "max_param" << YAML::Value << strand_mesh_generator_settings.max_param;
  out << YAML::Key << "branch_connections" << YAML::Value << strand_mesh_generator_settings.branch_connections;
  out << YAML::Key << "u_multiplier" << YAML::Value << strand_mesh_generator_settings.u_multiplier;
  out << YAML::Key << "v_multiplier" << YAML::Value << strand_mesh_generator_settings.v_multiplier;
  out << YAML::Key << "cluster_distance" << YAML::Value << strand_mesh_generator_settings.cluster_distance;
  out << YAML::Key << "remove_duplicate" << YAML::Value << strand_mesh_generator_settings.remove_duplicate;
  out << YAML::Key << "auto_level" << YAML::Value << strand_mesh_generator_settings.auto_level;
  out << YAML::Key << "voxel_subdivision_level" << YAML::Value << strand_mesh_generator_settings.voxel_subdivision_level;
  out << YAML::Key << "marching_cube_radius" << YAML::Value << strand_mesh_generator_settings.marching_cube_radius;
  out << YAML::Key << "x_subdivision" << YAML::Value << strand_mesh_generator_settings.x_subdivision;
  out << YAML::Key << "y_subdivision" << YAML::Value << strand_mesh_generator_settings.y_subdivision;
  out << YAML::Key << "marching_cube_color" << YAML::Value << strand_mesh_generator_settings.marching_cube_color;
  out << YAML::Key << "cylindrical_color" << YAML::Value << strand_mesh_generator_settings.cylindrical_color;
  out << YAML::Key << "root_distance_multiplier" << YAML::Value << strand_mesh_generator_settings.root_distance_multiplier;
  out << YAML::Key << "circle_multiplier" << YAML::Value << strand_mesh_generator_settings.circle_multiplier;
  out << YAML::Key << "recalculate_uv" << YAML::Value << strand_mesh_generator_settings.recalculate_uv;
  out << YAML::Key << "fast_uv" << YAML::Value << strand_mesh_generator_settings.fast_uv;
  out << YAML::Key << "smooth_iteration" << YAML::Value << strand_mesh_generator_settings.smooth_iteration;
  out << YAML::Key << "min_cell_count_for_major_branches" << YAML::Value << strand_mesh_generator_settings.min_cell_count_for_major_branches;
  out << YAML::Key << "max_cell_count_for_minor_branches" << YAML::Value << strand_mesh_generator_settings.max_cell_count_for_minor_branches;
  out << YAML::Key << "enable_branch" << YAML::Value << strand_mesh_generator_settings.enable_branch;
  out << YAML::Key << "enable_foliage" << YAML::Value << strand_mesh_generator_settings.enable_foliage;
  out << YAML::EndMap;

  out << YAML::Key << "dynamic_strands_settings" << YAML::Value << YAML::BeginMap;
  out << YAML::Key << "drag_multiplier" << YAML::Value << dynamic_strands_settings_.drag_multiplier;
  out << YAML::Key << "point_cut_thickness" << YAML::Value << dynamic_strands_settings_.point_cut_thickness;
  out << YAML::Key << "fungus_injection_amount" << YAML::Value << dynamic_strands_settings_.fungus_injection_amount;
  out << YAML::Key << "fungus_white_rot" << YAML::Value << dynamic_strands_settings_.fungus_white_rot;
  out << YAML::Key << "fungus_brown_rot" << YAML::Value << dynamic_strands_settings_.fungus_brown_rot;
  out << YAML::Key << "transform_mode" << YAML::Value << dynamic_strands_settings_.transform_mode;
  out << YAML::Key << "operator_mode" << YAML::Value << dynamic_strands_settings_.operator_mode;
  out << YAML::Key << "cut_bend_twist_bundle_only" << YAML::Value << dynamic_strands_settings_.cut_bend_twist_bundle_only;
  out << YAML::Key << "enable_visualization" << YAML::Value << dynamic_strands_settings_.enable_visualization;
  out << YAML::Key << "enable_physics" << YAML::Value << dynamic_strands_settings_.enable_physics;
  out << YAML::Key << "enable_rendering" << YAML::Value << dynamic_strands_settings_.enable_rendering;

  out << YAML::Key << "foliage_render_parameters" << YAML::Value << YAML::BeginMap;
  out << YAML::Key << "enabled" << YAML::Value << dynamic_strands_settings_.foliage_render_parameters.enabled;
  out << YAML::Key << "wireframe" << YAML::Value << dynamic_strands_settings_.foliage_render_parameters.wireframe;
  out << YAML::EndMap;

  out << YAML::Key << "segment_pairs_render_parameters" << YAML::Value << YAML::BeginMap;
  out << YAML::Key << "enabled" << YAML::Value << dynamic_strands_settings_.segment_pairs_render_parameters.enabled;
  out << YAML::Key << "thickness_multiplier" << YAML::Value << dynamic_strands_settings_.segment_pairs_render_parameters.thickness_multiplier;
  out << YAML::Key << "segment_pair_render_mode" << YAML::Value << dynamic_strands_settings_.segment_pairs_render_parameters.segment_pair_render_mode;
  out << YAML::Key << "position_scale" << YAML::Value << dynamic_strands_settings_.segment_pairs_render_parameters.position_scale;
  out << YAML::Key << "segment_pair_color_min" << YAML::Value << dynamic_strands_settings_.segment_pairs_render_parameters.segment_pair_color_min;
  out << YAML::Key << "segment_pair_color_max" << YAML::Value << dynamic_strands_settings_.segment_pairs_render_parameters.segment_pair_color_max;
  out << YAML::Key << "segment_pair_color_main" << YAML::Value << dynamic_strands_settings_.segment_pairs_render_parameters.segment_pair_color_main;
  out << YAML::Key << "segment_pair_radius_multiplier" << YAML::Value << dynamic_strands_settings_.segment_pairs_render_parameters.segment_pair_radius_multiplier;
  out << YAML::EndMap;

  out << YAML::Key << "visualization_parameters" << YAML::Value << YAML::BeginMap;
  out << YAML::Key << "render_segments" << YAML::Value << dynamic_strands_settings_.visualization_parameters.render_segments;
  out << YAML::Key << "render_segment_pairs" << YAML::Value << dynamic_strands_settings_.visualization_parameters.render_segment_pairs;
  out << YAML::Key << "render_foliage" << YAML::Value << dynamic_strands_settings_.visualization_parameters.render_foliage;
  out << YAML::Key << "segment_render_mode" << YAML::Value << dynamic_strands_settings_.visualization_parameters.segment_render_mode;
  out << YAML::Key << "segment_pair_render_mode" << YAML::Value << dynamic_strands_settings_.visualization_parameters.segment_pair_render_mode;
  out << YAML::Key << "foliage_render_mode" << YAML::Value << dynamic_strands_settings_.visualization_parameters.foliage_render_mode;
  out << YAML::Key << "segment_color_min" << YAML::Value << dynamic_strands_settings_.visualization_parameters.segment_color_min;
  out << YAML::Key << "segment_color_max" << YAML::Value << dynamic_strands_settings_.visualization_parameters.segment_color_max;
  out << YAML::Key << "segment_color_main" << YAML::Value << dynamic_strands_settings_.visualization_parameters.segment_color_main;
  out << YAML::Key << "segment_radius_multiplier" << YAML::Value << dynamic_strands_settings_.visualization_parameters.segment_radius_multiplier;
  out << YAML::Key << "segment_boundary_distance_modular" << YAML::Value << dynamic_strands_settings_.visualization_parameters.segment_boundary_distance_modular;
  out << YAML::Key << "segment_length_multiplier" << YAML::Value << dynamic_strands_settings_.visualization_parameters.segment_length_multiplier;
  out << YAML::Key << "general_factor" << YAML::Value << dynamic_strands_settings_.visualization_parameters.general_factor;
  out << YAML::Key << "segment_pair_color_min" << YAML::Value << dynamic_strands_settings_.visualization_parameters.segment_pair_color_min;
  out << YAML::Key << "segment_pair_color_max" << YAML::Value << dynamic_strands_settings_.visualization_parameters.segment_pair_color_max;
  out << YAML::Key << "segment_pair_color_main" << YAML::Value << dynamic_strands_settings_.visualization_parameters.segment_pair_color_main;
  out << YAML::Key << "segment_pair_radius_multiplier" << YAML::Value << dynamic_strands_settings_.visualization_parameters.segment_pair_radius_multiplier;
  out << YAML::Key << "foliage_color_min" << YAML::Value << dynamic_strands_settings_.visualization_parameters.foliage_color_min;
  out << YAML::Key << "foliage_color_max" << YAML::Value << dynamic_strands_settings_.visualization_parameters.foliage_color_max;
  out << YAML::Key << "foliage_color_main" << YAML::Value << dynamic_strands_settings_.visualization_parameters.foliage_color_main;
  out << YAML::EndMap;

  out << YAML::Key << "physics_parameters" << YAML::Value << YAML::BeginMap;
  const auto& physics = dynamic_strands_settings_.physics_parameters;
  out << YAML::Key << "time_step" << YAML::Value << physics.time_step;
  out << YAML::Key << "sub_step" << YAML::Value << physics.sub_step;
  out << YAML::Key << "position_constraint_iteration" << YAML::Value << physics.position_constraint_iteration;
  out << YAML::Key << "velocity_constraint_iteration" << YAML::Value << physics.velocity_constraint_iteration;
  out << YAML::Key << "enable_structural_damage" << YAML::Value << physics.enable_structural_damage;
  out << YAML::Key << "enable_segment_breaking" << YAML::Value << physics.enable_segment_breaking;
  out << YAML::Key << "enable_segment_disconnection" << YAML::Value << physics.enable_segment_disconnection;
  out << YAML::Key << "enable_foliage_detachment" << YAML::Value << physics.enable_foliage_detachment;
  out << YAML::Key << "enable_segment_tensile_disconnection" << YAML::Value << physics.enable_segment_tensile_disconnection;
  out << YAML::Key << "enable_segment_compression_disconnection" << YAML::Value << physics.enable_segment_compression_disconnection;
  out << YAML::Key << "compression_strength_factor" << YAML::Value << physics.compression_strength_factor;
  out << YAML::Key << "enable_positional_breaking" << YAML::Value << physics.enable_positional_breaking;
  out << YAML::Key << "enable_rotational_breaking" << YAML::Value << physics.enable_rotational_breaking;
  out << YAML::Key << "segment_velocity_damping" << YAML::Value << physics.segment_velocity_damping;
  out << YAML::Key << "segment_angular_velocity_damping" << YAML::Value << physics.segment_angular_velocity_damping;
  out << YAML::Key << "leaf_velocity_damping" << YAML::Value << physics.leaf_velocity_damping;
  out << YAML::Key << "leaf_angular_velocity_damping" << YAML::Value << physics.leaf_angular_velocity_damping;
  out << YAML::Key << "enable_segment_collision" << YAML::Value << physics.enable_segment_collision;
  out << YAML::Key << "dynamic_grouping" << YAML::Value << physics.dynamic_grouping;
  out << YAML::Key << "grouping_iteration" << YAML::Value << physics.grouping_iteration;
  out << YAML::Key << "gravity" << YAML::Value << physics.gravity;
  out << YAML::Key << "fungus_growth_rate" << YAML::Value << physics.fungus_growth_rate;
  out << YAML::Key << "enable_fungus" << YAML::Value << physics.enable_fungus;
  out << YAML::Key << "a_geom" << YAML::Value << physics.a_geom;
  out << YAML::Key << "b_vel" << YAML::Value << physics.b_vel;
  out << YAML::Key << "c_bias" << YAML::Value << physics.c_bias;
  out << YAML::Key << "s_min" << YAML::Value << physics.s_min;
  out << YAML::Key << "s_max_ratio" << YAML::Value << physics.s_max_ratio;
  out << YAML::Key << "eta" << YAML::Value << physics.eta;
  out << YAML::Key << "bmax_far" << YAML::Value << physics.bmax_far;
  out << YAML::Key << "dt" << YAML::Value << physics.dt;
  out << YAML::Key << "aw" << YAML::Value << physics.aw;
  out << YAML::Key << "ab" << YAML::Value << physics.ab;
  out << YAML::Key << "bw" << YAML::Value << physics.bw;
  out << YAML::Key << "bb" << YAML::Value << physics.bb;
  out << YAML::Key << "ycw" << YAML::Value << physics.ycw;
  out << YAML::Key << "ycb" << YAML::Value << physics.ycb;
  out << YAML::Key << "ylw" << YAML::Value << physics.ylw;
  out << YAML::Key << "pc" << YAML::Value << physics.pc;
  out << YAML::Key << "pl" << YAML::Value << physics.pl;
  out << YAML::Key << "k" << YAML::Value << physics.k;
  out << YAML::Key << "delta" << YAML::Value << physics.delta;
  out << YAML::Key << "ll" << YAML::Value << physics.ll;
  out << YAML::Key << "lc" << YAML::Value << physics.lc;
  out << YAML::Key << "bo" << YAML::Value << physics.bo;
  out << YAML::Key << "kc" << YAML::Value << physics.kc;
  out << YAML::Key << "be" << YAML::Value << physics.be;
  out << YAML::Key << "brw" << YAML::Value << physics.brw;
  out << YAML::Key << "brb" << YAML::Value << physics.brb;
  out << YAML::Key << "msr" << YAML::Value << physics.msr;
  out << YAML::Key << "cpb" << YAML::Value << physics.cpb;
  out << YAML::Key << "cpw" << YAML::Value << physics.cpw;
  out << YAML::Key << "lignin_threshold" << YAML::Value << physics.lignin_threshold;
  out << YAML::Key << "rod_strength_factor" << YAML::Value << physics.rod_strength_factor;
  out << YAML::Key << "bundle_strength_factor" << YAML::Value << physics.bundle_strength_factor;
  out << YAML::Key << "boundary_strength_decay_factor" << YAML::Value << physics.boundary_strength_decay_factor;
  out << YAML::Key << "HL_threshold" << YAML::Value << physics.HL_threshold;
  out << YAML::Key << "HC_threshold" << YAML::Value << physics.HC_threshold;
  out << YAML::Key << "bd_offset" << YAML::Value << physics.bd_offset;
  emit_mat3("matrixAw", physics.matrixAw);
  emit_mat3("matrixAb", physics.matrixAb);
  emit_mat3("matrixAc", physics.matrixAc);
  emit_mat3("matrixAm", physics.matrixAm);
  out << YAML::Key << "pivot_ring_radius" << YAML::Value << physics.pivot_ring_radius;
  out << YAML::Key << "crack_bd_shrinkage_offset" << YAML::Value << physics.crack_bd_shrinkage_offset;
  out << YAML::Key << "crack_R_scale" << YAML::Value << physics.crack_R_scale;
  out << YAML::Key << "crack_T_scale" << YAML::Value << physics.crack_T_scale;
  out << YAML::Key << "global_parameter" << YAML::Value << physics.global_parameter;
  out << YAML::Key << "treespace" << YAML::Value << physics.treespace;
  out << YAML::Key << "moisture_breaking_rod" << YAML::Value << physics.moisture_breaking_rod;
  out << YAML::Key << "internal_pattern" << YAML::Value << physics.internal_pattern;
  out << YAML::Key << "pull_cubical" << YAML::Value << physics.pull_cubical;
  out << YAML::Key << "leaf_break_from_moisture" << YAML::Value << physics.leaf_break_from_moisture;
  out << YAML::Key << "leaf_break_threshold" << YAML::Value << physics.leaf_break_threshold;
  out << YAML::EndMap;

  out << YAML::Key << "dynamic_skeleton_settings" << YAML::Value << YAML::BeginMap;
  out << YAML::Key << "enable_physics" << YAML::Value << dynamic_skeleton_settings_.enable_physics;
  out << YAML::Key << "enable_visualization" << YAML::Value << dynamic_skeleton_settings_.enable_visualization;
  out << YAML::Key << "time_step" << YAML::Value << dynamic_skeleton_settings_.physics_parameters.time_step;
  out << YAML::Key << "sub_step" << YAML::Value << dynamic_skeleton_settings_.physics_parameters.sub_step;
  out << YAML::Key << "constraint_iteration" << YAML::Value << dynamic_skeleton_settings_.physics_parameters.constraint_iteration;
  out << YAML::Key << "enable_disconnection" << YAML::Value << dynamic_skeleton_settings_.physics_parameters.enable_disconnection;
  out << YAML::Key << "enable_breaking" << YAML::Value << dynamic_skeleton_settings_.physics_parameters.enable_breaking;
  out << YAML::Key << "velocity_damping" << YAML::Value << dynamic_skeleton_settings_.physics_parameters.velocity_damping;
  out << YAML::Key << "angular_velocity_damping" << YAML::Value << dynamic_skeleton_settings_.physics_parameters.angular_velocity_damping;
  out << YAML::EndMap;

  out << YAML::Key << "soil_visualization_settings" << YAML::Value << YAML::BeginMap;
  out << YAML::Key << "enable" << YAML::Value << soil_visualization_settings_.enable;
  out << YAML::Key << "vector_enable" << YAML::Value << soil_visualization_settings_.vector_enable;
  out << YAML::Key << "scalar_enable" << YAML::Value << soil_visualization_settings_.scalar_enable;
  out << YAML::Key << "vector_multiplier" << YAML::Value << soil_visualization_settings_.vector_multiplier;
  out << YAML::Key << "vector_base_color" << YAML::Value << soil_visualization_settings_.vector_base_color;
  out << YAML::Key << "vector_soil_property" << YAML::Value << soil_visualization_settings_.vector_soil_property;
  out << YAML::Key << "vector_line_width_factor" << YAML::Value << soil_visualization_settings_.vector_line_width_factor;
  out << YAML::Key << "vector_line_max_width" << YAML::Value << soil_visualization_settings_.vector_line_max_width;
  out << YAML::Key << "scalar_multiplier" << YAML::Value << soil_visualization_settings_.scalar_multiplier;
  out << YAML::Key << "scalar_box_size" << YAML::Value << soil_visualization_settings_.scalar_box_size;
  out << YAML::Key << "scalar_min_alpha" << YAML::Value << soil_visualization_settings_.scalar_min_alpha;
  out << YAML::Key << "scalar_base_color" << YAML::Value << soil_visualization_settings_.scalar_base_color;
  out << YAML::Key << "scalar_soil_property" << YAML::Value << soil_visualization_settings_.scalar_soil_property;
  out << YAML::Key << "soil_cutout_x_depth" << YAML::Value << soil_visualization_settings_.soil_cutout_x_depth;
  out << YAML::Key << "soil_cutout_z_depth" << YAML::Value << soil_visualization_settings_.soil_cutout_z_depth;
  out << YAML::Key << "soil_layer_colors" << YAML::Value << soil_visualization_settings_.soil_layer_colors;
  out << YAML::EndMap;
}

void EcoSysLabLayer::Deserialize(const YAML::Node& in) {
  auto parse_mat3 = [&](const YAML::Node& node, glm::mat3& value) {
    if (!node || !node.IsSequence() || node.size() != 9)
      return;
    for (int c = 0; c < 3; c++) {
      for (int r = 0; r < 3; r++) {
        value[c][r] = node[c * 3 + r].as<float>();
      }
    }
  };

  if (in["show_visualization_camera_info"])
    show_visualization_camera_info = in["show_visualization_camera_info"].as<bool>();
  if (in["enable_visualization_background"])
    enable_visualization_background = in["enable_visualization_background"].as<bool>();
  if (in["need_full_flow_update"])
    need_full_flow_update = in["need_full_flow_update"].as<bool>();
  if (in["tree_operator_mode"])
    tree_operator_mode = in["tree_operator_mode"].as<unsigned>();
  if (in["tree_reduce_rate"])
    tree_reduce_rate = in["tree_reduce_rate"].as<float>();

  if (in["auto_generate_mesh_after_editing"])
    auto_generate_mesh_after_editing_ = in["auto_generate_mesh_after_editing"].as<bool>();
  if (in["auto_generate_skeletal_graph_every_frame"])
    auto_generate_skeletal_graph_every_frame_ = in["auto_generate_skeletal_graph_every_frame"].as<bool>();
  if (in["auto_generate_strands_after_editing"])
    auto_generate_strands_after_editing_ = in["auto_generate_strands_after_editing"].as<bool>();
  if (in["auto_generate_strand_mesh_after_editing"])
    auto_generate_strand_mesh_after_editing_ = in["auto_generate_strand_mesh_after_editing"].as<bool>();
  if (in["auto_update_strand_renderer"])
    auto_update_strand_renderer_ = in["auto_update_strand_renderer"].as<bool>();
  if (in["auto_update_strand_renderer_interval"])
    auto_update_strand_renderer_interval_ = in["auto_update_strand_renderer_interval"].as<float>();
  if (in["auto_update_strand_model_mesh"])
    auto_update_strand_model_mesh_ = in["auto_update_strand_model_mesh"].as<bool>();
  if (in["auto_update_strand_model_mesh_interval"])
    auto_update_strand_model_mesh_interval_ = in["auto_update_strand_model_mesh_interval"].as<float>();

  if (in["simulated_time"])
    simulated_time_ = in["simulated_time"].as<float>();
  if (in["auto_time_grow"])
    auto_time_grow_ = in["auto_time_grow"].as<bool>();
  if (in["auto_time_target"])
    auto_time_target_ = in["auto_time_target"].as<float>();
  if (in["extra_time_years"])
    extra_time_years_ = in["extra_time_years"].as<float>();

  simulation_settings.Load("simulation_settings", in);
  mesh_generator_settings.Load("mesh_generator_settings", in);
  skeletal_graph_settings.Load("skeletal_graph_settings", in);

  if (in["tree_visualization_settings"]) {
    const auto tv = in["tree_visualization_settings"];
    if (tv["enable"]) tree_visualization_settings_.enable = tv["enable"].as<bool>();
    if (tv["display_shoot_stem"]) tree_visualization_settings_.display_shoot_stem = tv["display_shoot_stem"].as<bool>();
    if (tv["display_foliage"]) tree_visualization_settings_.display_foliage = tv["display_foliage"].as<bool>();
    if (tv["display_flowers"]) tree_visualization_settings_.display_flowers = tv["display_flowers"].as<bool>();
    if (tv["display_fruits"]) tree_visualization_settings_.display_fruits = tv["display_fruits"].as<bool>();
    if (tv["display_bounding_box"]) tree_visualization_settings_.display_bounding_box = tv["display_bounding_box"].as<bool>();
    if (tv["display_ground_flowers"]) tree_visualization_settings_.display_ground_flowers = tv["display_ground_flowers"].as<bool>();
    if (tv["display_ground_fruits"]) tree_visualization_settings_.display_ground_fruits = tv["display_ground_fruits"].as<bool>();
    if (tv["display_ground_leaves"]) tree_visualization_settings_.display_ground_leaves = tv["display_ground_leaves"].as<bool>();
    if (tv["show_shadow_grid"]) tree_visualization_settings_.show_shadow_grid = tv["show_shadow_grid"].as<bool>();
    if (tv["show_lighting_grid"]) tree_visualization_settings_.show_lighting_grid = tv["show_lighting_grid"].as<bool>();
  }

  if (in["strand_mesh_generator_settings"]) {
    const auto s = in["strand_mesh_generator_settings"];
    if (s["generator_type"]) strand_mesh_generator_settings.generator_type = s["generator_type"].as<unsigned>();
    if (s["steps_per_segment"]) strand_mesh_generator_settings.steps_per_segment = s["steps_per_segment"].as<int>();
    if (s["max_param"]) strand_mesh_generator_settings.max_param = s["max_param"].as<float>();
    if (s["branch_connections"]) strand_mesh_generator_settings.branch_connections = s["branch_connections"].as<bool>();
    if (s["u_multiplier"]) strand_mesh_generator_settings.u_multiplier = s["u_multiplier"].as<int>();
    if (s["v_multiplier"]) strand_mesh_generator_settings.v_multiplier = s["v_multiplier"].as<float>();
    if (s["cluster_distance"]) strand_mesh_generator_settings.cluster_distance = s["cluster_distance"].as<float>();
    if (s["remove_duplicate"]) strand_mesh_generator_settings.remove_duplicate = s["remove_duplicate"].as<bool>();
    if (s["auto_level"]) strand_mesh_generator_settings.auto_level = s["auto_level"].as<bool>();
    if (s["voxel_subdivision_level"]) strand_mesh_generator_settings.voxel_subdivision_level = s["voxel_subdivision_level"].as<int>();
    if (s["marching_cube_radius"]) strand_mesh_generator_settings.marching_cube_radius = s["marching_cube_radius"].as<float>();
    if (s["x_subdivision"]) strand_mesh_generator_settings.x_subdivision = s["x_subdivision"].as<float>();
    if (s["y_subdivision"]) strand_mesh_generator_settings.y_subdivision = s["y_subdivision"].as<float>();
    if (s["marching_cube_color"]) strand_mesh_generator_settings.marching_cube_color = s["marching_cube_color"].as<glm::vec4>();
    if (s["cylindrical_color"]) strand_mesh_generator_settings.cylindrical_color = s["cylindrical_color"].as<glm::vec4>();
    if (s["root_distance_multiplier"]) strand_mesh_generator_settings.root_distance_multiplier = s["root_distance_multiplier"].as<int>();
    if (s["circle_multiplier"]) strand_mesh_generator_settings.circle_multiplier = s["circle_multiplier"].as<float>();
    if (s["recalculate_uv"]) strand_mesh_generator_settings.recalculate_uv = s["recalculate_uv"].as<bool>();
    if (s["fast_uv"]) strand_mesh_generator_settings.fast_uv = s["fast_uv"].as<bool>();
    if (s["smooth_iteration"]) strand_mesh_generator_settings.smooth_iteration = s["smooth_iteration"].as<int>();
    if (s["min_cell_count_for_major_branches"]) strand_mesh_generator_settings.min_cell_count_for_major_branches = s["min_cell_count_for_major_branches"].as<int>();
    if (s["max_cell_count_for_minor_branches"]) strand_mesh_generator_settings.max_cell_count_for_minor_branches = s["max_cell_count_for_minor_branches"].as<int>();
    if (s["enable_branch"]) strand_mesh_generator_settings.enable_branch = s["enable_branch"].as<bool>();
    if (s["enable_foliage"]) strand_mesh_generator_settings.enable_foliage = s["enable_foliage"].as<bool>();
  }

  if (in["dynamic_strands_settings"]) {
    const auto ds = in["dynamic_strands_settings"];
    if (ds["drag_multiplier"]) dynamic_strands_settings_.drag_multiplier = ds["drag_multiplier"].as<float>();
    if (ds["point_cut_thickness"]) dynamic_strands_settings_.point_cut_thickness = ds["point_cut_thickness"].as<float>();
    if (ds["fungus_injection_amount"]) dynamic_strands_settings_.fungus_injection_amount = ds["fungus_injection_amount"].as<float>();
    if (ds["fungus_white_rot"]) dynamic_strands_settings_.fungus_white_rot = ds["fungus_white_rot"].as<bool>();
    if (ds["fungus_brown_rot"]) dynamic_strands_settings_.fungus_brown_rot = ds["fungus_brown_rot"].as<bool>();
    if (ds["transform_mode"]) dynamic_strands_settings_.transform_mode = ds["transform_mode"].as<unsigned>();
    if (ds["operator_mode"]) dynamic_strands_settings_.operator_mode = ds["operator_mode"].as<unsigned>();
    if (ds["cut_bend_twist_bundle_only"]) dynamic_strands_settings_.cut_bend_twist_bundle_only = ds["cut_bend_twist_bundle_only"].as<bool>();
    if (ds["enable_visualization"]) dynamic_strands_settings_.enable_visualization = ds["enable_visualization"].as<bool>();
    if (ds["enable_physics"]) dynamic_strands_settings_.enable_physics = ds["enable_physics"].as<bool>();
    if (ds["enable_rendering"]) dynamic_strands_settings_.enable_rendering = ds["enable_rendering"].as<bool>();

    if (ds["foliage_render_parameters"]) {
      const auto f = ds["foliage_render_parameters"];
      if (f["enabled"]) dynamic_strands_settings_.foliage_render_parameters.enabled = f["enabled"].as<bool>();
      if (f["wireframe"]) dynamic_strands_settings_.foliage_render_parameters.wireframe = f["wireframe"].as<bool>();
    }
    if (ds["segment_pairs_render_parameters"]) {
      const auto s = ds["segment_pairs_render_parameters"];
      if (s["enabled"]) dynamic_strands_settings_.segment_pairs_render_parameters.enabled = s["enabled"].as<bool>();
      if (s["thickness_multiplier"]) dynamic_strands_settings_.segment_pairs_render_parameters.thickness_multiplier = s["thickness_multiplier"].as<float>();
      if (s["segment_pair_render_mode"]) dynamic_strands_settings_.segment_pairs_render_parameters.segment_pair_render_mode = s["segment_pair_render_mode"].as<uint32_t>();
      if (s["position_scale"]) dynamic_strands_settings_.segment_pairs_render_parameters.position_scale = s["position_scale"].as<glm::vec3>();
      if (s["segment_pair_color_min"]) dynamic_strands_settings_.segment_pairs_render_parameters.segment_pair_color_min = s["segment_pair_color_min"].as<glm::vec4>();
      if (s["segment_pair_color_max"]) dynamic_strands_settings_.segment_pairs_render_parameters.segment_pair_color_max = s["segment_pair_color_max"].as<glm::vec4>();
      if (s["segment_pair_color_main"]) dynamic_strands_settings_.segment_pairs_render_parameters.segment_pair_color_main = s["segment_pair_color_main"].as<glm::vec4>();
      if (s["segment_pair_radius_multiplier"]) dynamic_strands_settings_.segment_pairs_render_parameters.segment_pair_radius_multiplier = s["segment_pair_radius_multiplier"].as<float>();
    }
    if (ds["visualization_parameters"]) {
      const auto v = ds["visualization_parameters"];
      if (v["render_segments"]) dynamic_strands_settings_.visualization_parameters.render_segments = v["render_segments"].as<bool>();
      if (v["render_segment_pairs"]) dynamic_strands_settings_.visualization_parameters.render_segment_pairs = v["render_segment_pairs"].as<bool>();
      if (v["render_foliage"]) dynamic_strands_settings_.visualization_parameters.render_foliage = v["render_foliage"].as<bool>();
      if (v["segment_render_mode"]) dynamic_strands_settings_.visualization_parameters.segment_render_mode = v["segment_render_mode"].as<uint32_t>();
      if (v["segment_pair_render_mode"]) dynamic_strands_settings_.visualization_parameters.segment_pair_render_mode = v["segment_pair_render_mode"].as<uint32_t>();
      if (v["foliage_render_mode"]) dynamic_strands_settings_.visualization_parameters.foliage_render_mode = v["foliage_render_mode"].as<uint32_t>();
      if (v["segment_color_min"]) dynamic_strands_settings_.visualization_parameters.segment_color_min = v["segment_color_min"].as<glm::vec4>();
      if (v["segment_color_max"]) dynamic_strands_settings_.visualization_parameters.segment_color_max = v["segment_color_max"].as<glm::vec4>();
      if (v["segment_color_main"]) dynamic_strands_settings_.visualization_parameters.segment_color_main = v["segment_color_main"].as<glm::vec4>();
      if (v["segment_radius_multiplier"]) dynamic_strands_settings_.visualization_parameters.segment_radius_multiplier = v["segment_radius_multiplier"].as<float>();
      if (v["segment_boundary_distance_modular"]) dynamic_strands_settings_.visualization_parameters.segment_boundary_distance_modular = v["segment_boundary_distance_modular"].as<float>();
      if (v["segment_length_multiplier"]) dynamic_strands_settings_.visualization_parameters.segment_length_multiplier = v["segment_length_multiplier"].as<float>();
      if (v["general_factor"]) dynamic_strands_settings_.visualization_parameters.general_factor = v["general_factor"].as<float>();
      if (v["segment_pair_color_min"]) dynamic_strands_settings_.visualization_parameters.segment_pair_color_min = v["segment_pair_color_min"].as<glm::vec4>();
      if (v["segment_pair_color_max"]) dynamic_strands_settings_.visualization_parameters.segment_pair_color_max = v["segment_pair_color_max"].as<glm::vec4>();
      if (v["segment_pair_color_main"]) dynamic_strands_settings_.visualization_parameters.segment_pair_color_main = v["segment_pair_color_main"].as<glm::vec4>();
      if (v["segment_pair_radius_multiplier"]) dynamic_strands_settings_.visualization_parameters.segment_pair_radius_multiplier = v["segment_pair_radius_multiplier"].as<float>();
      if (v["foliage_color_min"]) dynamic_strands_settings_.visualization_parameters.foliage_color_min = v["foliage_color_min"].as<glm::vec4>();
      if (v["foliage_color_max"]) dynamic_strands_settings_.visualization_parameters.foliage_color_max = v["foliage_color_max"].as<glm::vec4>();
      if (v["foliage_color_main"]) dynamic_strands_settings_.visualization_parameters.foliage_color_main = v["foliage_color_main"].as<glm::vec4>();
    }
    if (ds["physics_parameters"]) {
      auto& p = dynamic_strands_settings_.physics_parameters;
      const auto n = ds["physics_parameters"];
      if (n["time_step"]) p.time_step = n["time_step"].as<float>();
      if (n["sub_step"]) p.sub_step = n["sub_step"].as<int>();
      if (n["position_constraint_iteration"]) p.position_constraint_iteration = n["position_constraint_iteration"].as<int>();
      if (n["velocity_constraint_iteration"]) p.velocity_constraint_iteration = n["velocity_constraint_iteration"].as<int>();
      if (n["enable_structural_damage"]) p.enable_structural_damage = n["enable_structural_damage"].as<bool>();
      if (n["enable_segment_breaking"]) p.enable_segment_breaking = n["enable_segment_breaking"].as<bool>();
      if (n["enable_segment_disconnection"]) p.enable_segment_disconnection = n["enable_segment_disconnection"].as<bool>();
      if (n["enable_foliage_detachment"]) p.enable_foliage_detachment = n["enable_foliage_detachment"].as<bool>();
      if (n["enable_segment_tensile_disconnection"]) p.enable_segment_tensile_disconnection = n["enable_segment_tensile_disconnection"].as<bool>();
      if (n["enable_segment_compression_disconnection"]) p.enable_segment_compression_disconnection = n["enable_segment_compression_disconnection"].as<bool>();
      if (n["compression_strength_factor"]) p.compression_strength_factor = n["compression_strength_factor"].as<float>();
      if (n["enable_positional_breaking"]) p.enable_positional_breaking = n["enable_positional_breaking"].as<bool>();
      if (n["enable_rotational_breaking"]) p.enable_rotational_breaking = n["enable_rotational_breaking"].as<bool>();
      if (n["segment_velocity_damping"]) p.segment_velocity_damping = n["segment_velocity_damping"].as<float>();
      if (n["segment_angular_velocity_damping"]) p.segment_angular_velocity_damping = n["segment_angular_velocity_damping"].as<float>();
      if (n["leaf_velocity_damping"]) p.leaf_velocity_damping = n["leaf_velocity_damping"].as<float>();
      if (n["leaf_angular_velocity_damping"]) p.leaf_angular_velocity_damping = n["leaf_angular_velocity_damping"].as<float>();
      if (n["enable_segment_collision"]) p.enable_segment_collision = n["enable_segment_collision"].as<bool>();
      if (n["dynamic_grouping"]) p.dynamic_grouping = n["dynamic_grouping"].as<bool>();
      if (n["grouping_iteration"]) p.grouping_iteration = n["grouping_iteration"].as<int>();
      if (n["gravity"]) p.gravity = n["gravity"].as<glm::vec3>();
      if (n["fungus_growth_rate"]) p.fungus_growth_rate = n["fungus_growth_rate"].as<float>();
      if (n["enable_fungus"]) p.enable_fungus = n["enable_fungus"].as<bool>();
      if (n["a_geom"]) p.a_geom = n["a_geom"].as<float>();
      if (n["b_vel"]) p.b_vel = n["b_vel"].as<float>();
      if (n["c_bias"]) p.c_bias = n["c_bias"].as<float>();
      if (n["s_min"]) p.s_min = n["s_min"].as<float>();
      if (n["s_max_ratio"]) p.s_max_ratio = n["s_max_ratio"].as<float>();
      if (n["eta"]) p.eta = n["eta"].as<float>();
      if (n["bmax_far"]) p.bmax_far = n["bmax_far"].as<float>();
      if (n["dt"]) p.dt = n["dt"].as<float>();
      if (n["aw"]) p.aw = n["aw"].as<float>();
      if (n["ab"]) p.ab = n["ab"].as<float>();
      if (n["bw"]) p.bw = n["bw"].as<float>();
      if (n["bb"]) p.bb = n["bb"].as<float>();
      if (n["ycw"]) p.ycw = n["ycw"].as<float>();
      if (n["ycb"]) p.ycb = n["ycb"].as<float>();
      if (n["ylw"]) p.ylw = n["ylw"].as<float>();
      if (n["pc"]) p.pc = n["pc"].as<float>();
      if (n["pl"]) p.pl = n["pl"].as<float>();
      if (n["k"]) p.k = n["k"].as<float>();
      if (n["delta"]) p.delta = n["delta"].as<float>();
      if (n["ll"]) p.ll = n["ll"].as<float>();
      if (n["lc"]) p.lc = n["lc"].as<float>();
      if (n["bo"]) p.bo = n["bo"].as<float>();
      if (n["kc"]) p.kc = n["kc"].as<float>();
      if (n["be"]) p.be = n["be"].as<float>();
      if (n["brw"]) p.brw = n["brw"].as<float>();
      if (n["brb"]) p.brb = n["brb"].as<float>();
      if (n["msr"]) p.msr = n["msr"].as<float>();
      if (n["cpb"]) p.cpb = n["cpb"].as<float>();
      if (n["cpw"]) p.cpw = n["cpw"].as<float>();
      if (n["lignin_threshold"]) p.lignin_threshold = n["lignin_threshold"].as<float>();
      if (n["rod_strength_factor"]) p.rod_strength_factor = n["rod_strength_factor"].as<float>();
      if (n["bundle_strength_factor"]) p.bundle_strength_factor = n["bundle_strength_factor"].as<float>();
      if (n["boundary_strength_decay_factor"]) p.boundary_strength_decay_factor = n["boundary_strength_decay_factor"].as<float>();
      if (n["HL_threshold"]) p.HL_threshold = n["HL_threshold"].as<float>();
      if (n["HC_threshold"]) p.HC_threshold = n["HC_threshold"].as<float>();
      if (n["bd_offset"]) p.bd_offset = n["bd_offset"].as<float>();
      parse_mat3(n["matrixAw"], p.matrixAw);
      parse_mat3(n["matrixAb"], p.matrixAb);
      parse_mat3(n["matrixAc"], p.matrixAc);
      parse_mat3(n["matrixAm"], p.matrixAm);
      if (n["pivot_ring_radius"]) p.pivot_ring_radius = n["pivot_ring_radius"].as<float>();
      if (n["crack_bd_shrinkage_offset"]) p.crack_bd_shrinkage_offset = n["crack_bd_shrinkage_offset"].as<float>();
      if (n["crack_R_scale"]) p.crack_R_scale = n["crack_R_scale"].as<float>();
      if (n["crack_T_scale"]) p.crack_T_scale = n["crack_T_scale"].as<float>();
      if (n["global_parameter"]) p.global_parameter = n["global_parameter"].as<int>();
      if (n["treespace"]) p.treespace = n["treespace"].as<int>();
      if (n["moisture_breaking_rod"]) p.moisture_breaking_rod = n["moisture_breaking_rod"].as<int>();
      if (n["internal_pattern"]) p.internal_pattern = n["internal_pattern"].as<int>();
      if (n["pull_cubical"]) p.pull_cubical = n["pull_cubical"].as<int>();
      if (n["leaf_break_from_moisture"]) p.leaf_break_from_moisture = n["leaf_break_from_moisture"].as<int>();
      if (n["leaf_break_threshold"]) p.leaf_break_threshold = n["leaf_break_threshold"].as<float>();
    }
  }

  if (in["dynamic_skeleton_settings"]) {
    const auto d = in["dynamic_skeleton_settings"];
    if (d["enable_physics"]) dynamic_skeleton_settings_.enable_physics = d["enable_physics"].as<bool>();
    if (d["enable_visualization"]) dynamic_skeleton_settings_.enable_visualization = d["enable_visualization"].as<bool>();
    if (d["time_step"]) dynamic_skeleton_settings_.physics_parameters.time_step = d["time_step"].as<float>();
    if (d["sub_step"]) dynamic_skeleton_settings_.physics_parameters.sub_step = d["sub_step"].as<int>();
    if (d["constraint_iteration"]) dynamic_skeleton_settings_.physics_parameters.constraint_iteration = d["constraint_iteration"].as<int>();
    if (d["enable_disconnection"]) dynamic_skeleton_settings_.physics_parameters.enable_disconnection = d["enable_disconnection"].as<bool>();
    if (d["enable_breaking"]) dynamic_skeleton_settings_.physics_parameters.enable_breaking = d["enable_breaking"].as<bool>();
    if (d["velocity_damping"]) dynamic_skeleton_settings_.physics_parameters.velocity_damping = d["velocity_damping"].as<float>();
    if (d["angular_velocity_damping"]) dynamic_skeleton_settings_.physics_parameters.angular_velocity_damping = d["angular_velocity_damping"].as<float>();
  }

  if (in["soil_visualization_settings"]) {
    const auto s = in["soil_visualization_settings"];
    if (s["enable"]) soil_visualization_settings_.enable = s["enable"].as<bool>();
    if (s["vector_enable"]) soil_visualization_settings_.vector_enable = s["vector_enable"].as<bool>();
    if (s["scalar_enable"]) soil_visualization_settings_.scalar_enable = s["scalar_enable"].as<bool>();
    if (s["vector_multiplier"]) soil_visualization_settings_.vector_multiplier = s["vector_multiplier"].as<float>();
    if (s["vector_base_color"]) soil_visualization_settings_.vector_base_color = s["vector_base_color"].as<glm::vec4>();
    if (s["vector_soil_property"]) soil_visualization_settings_.vector_soil_property = s["vector_soil_property"].as<unsigned>();
    if (s["vector_line_width_factor"]) soil_visualization_settings_.vector_line_width_factor = s["vector_line_width_factor"].as<float>();
    if (s["vector_line_max_width"]) soil_visualization_settings_.vector_line_max_width = s["vector_line_max_width"].as<float>();
    if (s["scalar_multiplier"]) soil_visualization_settings_.scalar_multiplier = s["scalar_multiplier"].as<float>();
    if (s["scalar_box_size"]) soil_visualization_settings_.scalar_box_size = s["scalar_box_size"].as<float>();
    if (s["scalar_min_alpha"]) soil_visualization_settings_.scalar_min_alpha = s["scalar_min_alpha"].as<float>();
    if (s["scalar_base_color"]) soil_visualization_settings_.scalar_base_color = s["scalar_base_color"].as<glm::vec3>();
    if (s["scalar_soil_property"]) soil_visualization_settings_.scalar_soil_property = s["scalar_soil_property"].as<unsigned>();
    if (s["soil_cutout_x_depth"]) soil_visualization_settings_.soil_cutout_x_depth = s["soil_cutout_x_depth"].as<float>();
    if (s["soil_cutout_z_depth"]) soil_visualization_settings_.soil_cutout_z_depth = s["soil_cutout_z_depth"].as<float>();
    if (s["soil_layer_colors"]) soil_visualization_settings_.soil_layer_colors = s["soil_layer_colors"].as<std::vector<glm::vec4>>();
  }

  if (in["selected_tree"]) {
    const uint64_t handle = in["selected_tree"].as<uint64_t>();
    if (handle != 0) {
      Application::RegisterPostAttachSceneFunction([this, handle](const std::shared_ptr<Scene>& scene) {
        if (const auto entity = scene->GetEntity(Handle(handle)); scene->IsEntityValid(entity)) {
          selected_tree = entity;
          last_selected_tree_index_ = selected_tree.GetIndex();
          need_flow_update_for_selection_ = true;
        }
      });
    }
  }

  if (in["selected_entity"]) {
    const uint64_t handle = in["selected_entity"].as<uint64_t>();
    if (handle != 0) {
      Application::RegisterPostAttachSceneFunction([handle](const std::shared_ptr<Scene>& scene) {
        if (const auto editor_layer = Application::GetLayer<EditorLayer>()) {
          if (const auto entity = scene->GetEntity(Handle(handle)); scene->IsEntityValid(entity)) {
            editor_layer->SetSelectedEntity(entity);
          }
        }
      });
    }
  }

  soil_visualization_settings_.update_scalar_matrices = true;
  soil_visualization_settings_.update_vector_matrices = true;
}

std::weak_ptr<Climate> EcoSysLabLayer::FindClimate() {
  const auto scene = Application::GetActiveScene();
  const std::vector<Entity>* climate_entities = scene->UnsafeGetPrivateComponentOwnersList<Climate>();
  if (climate_entities && !climate_entities->empty()) {
    return scene->GetOrSetPrivateComponent<Climate>(climate_entities->at(0));
  }
  return {};
}

const std::vector<glm::vec3>& EcoSysLabLayer::RandomColors() {
  return random_colors_;
}

void EcoSysLabLayer::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  auto scene = GetScene();
  bool simulate = false;
  visualization_camera_->Resize({visualization_camera_resolution_x, visualization_camera_resolution_y});
  if (const auto scene_camera = editor_layer->GetSceneCamera()) {
    visualization_camera_->camera_settings.near_distance = scene_camera->camera_settings.near_distance;
    visualization_camera_->camera_settings.far_distance = scene_camera->camera_settings.far_distance;
    visualization_camera_->camera_settings.fov = scene_camera->camera_settings.fov;
  }

  if (EditorLayer::GetKey(GLFW_KEY_LEFT_CONTROL) == Input::KeyActionType::Hold ||
      EditorLayer::GetKey(GLFW_KEY_RIGHT_CONTROL) == Input::KeyActionType::Hold) {
    if (EditorLayer::GetKey(GLFW_KEY_W) == Input::KeyActionType::Press) {
      const std::vector<Entity>* tree_entities = scene->UnsafeGetPrivateComponentOwnersList<Tree>();
      ResetAllTrees(tree_entities);
      ClearMeshes();
      ClearGroundFruitAndLeaf();
      auto_time_target_ = 0.0f;
    }
    if (EditorLayer::GetKey(GLFW_KEY_F) == Input::KeyActionType::Press) {
      const std::vector<Entity>* tree_entities = scene->UnsafeGetPrivateComponentOwnersList<Tree>();
      if (tree_entities && !tree_entities->empty()) {
        if (auto_time_grow_) {
          auto_time_grow_ = false;
          auto_time_target_ = simulated_time_;
        } else {
          auto_time_grow_ = true;
          auto_time_target_ += extra_time_years_ * 365.f;
        }
      }
    }
  }

  ImGui::Checkbox("Show Trees", &tree_visualization_settings_.enable);
  if (tree_visualization_settings_.enable) {
    const std::vector<Entity>* tree_entities = scene->UnsafeGetPrivateComponentOwnersList<Tree>();
    if (ImGui::TreeNodeEx("Tree settings")) {
      if (tree_entities && !tree_entities->empty()) {
        if (scene->IsEntityValid(selected_tree)) {
          const auto& tree = scene->GetOrSetPrivateComponent<Tree>(selected_tree).lock();
          auto& shoot_visualizer = tree->shoot_visualizer;
          if (shoot_visualizer.checkpoint_iteration == tree->shoot_model.CurrentIteration()) {
            if (ImGui::TreeNodeEx("Tree Operator", ImGuiTreeNodeFlags_DefaultOpen)) {
              if (ImGui::Combo("Mode", {"None", "Select", "Rotate", "Prune", "Invigorate", "Reduce"},
                               tree_operator_mode)) {
                shoot_visualizer.selected_node_handle = -1;
                shoot_visualizer.selected_node_hierarchy_list.clear();
              }
              switch (static_cast<TreeOperatorMode>(tree_operator_mode)) {
                case TreeOperatorMode::Select:
                  ImGui::Text("Press T to cut off entire node, press R to cut at point of selection.");
                  break;
                case TreeOperatorMode::Rotate:
                  ImGui::Text("Press T to cut off entire node.");
                  break;
                default:
                  break;
              }
              if (tree_operator_mode == static_cast<unsigned>(TreeOperatorMode::Reduce)) {
                ImGui::DragFloat("Reduce speed", &tree_reduce_rate, 0.001f, 0.001f, 1.0f);
              }

              ImGui::TreePop();
            }
          } else {
            ImGui::Text("Go to current skeleton to enable operator!");
          }
          ImGui::Separator();
          if (ImGui::TreeNodeEx("Tree Visualizer")) {
            // Shoot visualizer section
            if (ImGui::TreeNodeEx("Shoot visualizer", ImGuiTreeNodeFlags_DefaultOpen)) {
              ImGui::PushID("ShootVisualizer");
              tree->shoot_visualizer.OnInspect(tree->shoot_model);
              ImGui::PopID();
              ImGui::TreePop();
            }

            // Root visualizer section
            if (ImGui::TreeNodeEx("Root visualizer", ImGuiTreeNodeFlags_DefaultOpen)) {
              ImGui::PushID("RootVisualizer");
              tree->root_visualizer.OnInspect(tree->root_model);
              ImGui::PopID();
              ImGui::TreePop();
            }
            ImGui::TreePop();
          }
        } else {
          ImGui::Text("Select a tree entity to enable editing & visualization!");
        }
        if (!simulation_settings.auto_clear_fruit_and_leaves && ImGui::Button("Clear ground leaves and fruits")) {
          ClearGroundFruitAndLeaf();
        }
        if (ImGui::TreeNode("Tree Geometries")) {
          if (ImGui::TreeNode("Skeletal graph")) {
            skeletal_graph_settings.OnInspect(editor_layer);
            ImGui::TreePop();
          }
          if (ImGui::Button("Generate Skeletal graphs")) {
            GenerateSkeletalGraphs(skeletal_graph_settings);
          }
          ImGui::SameLine();
          if (ImGui::Button("Clear Skeletal graphs")) {
            ClearSkeletalGraphs();
          }
          ImGui::Separator();
          if (ImGui::TreeNodeEx("Mesh generation")) {
            mesh_generator_settings.OnInspect(editor_layer);
            ImGui::TreePop();
          }
          if (ImGui::Button("Generate Meshes")) {
            GenerateMeshes(mesh_generator_settings);
          }
          ImGui::SameLine();
          if (ImGui::Button("Clear Meshes")) {
            ClearMeshes();
          }
          ImGui::Separator();
          if (ImGui::TreeNodeEx("Strand Model Mesh generation")) {
            strand_mesh_generator_settings.OnInspect(editor_layer);
            ImGui::TreePop();
          }
          if (ImGui::Button("Build Strand Renderer")) {
            GenerateStrandRenderers();
          }
          ImGui::SameLine();
          if (ImGui::Button("Clear Strand Renderer")) {
            ClearStrandRenderers();
          }
          ImGui::Checkbox("Auto update Strand Renderer", &auto_update_strand_renderer_);
          ImGui::SameLine();
          ImGui::BeginDisabled(!auto_update_strand_renderer_);
          ImGui::SetNextItemWidth(220.0f);
          ImGui::DragFloat("Strand Renderer interval (s)", &auto_update_strand_renderer_interval_, 0.1f, 0.1f, 3600.0f);
          ImGui::EndDisabled();
          if (ImGui::Button("Generate Strand Model Meshes")) {
            GenerateStrandModelMeshes(strand_mesh_generator_settings);
          }
          ImGui::SameLine();
          if (ImGui::Button("Clear Strand Model Meshes")) {
            ClearStrandModelMeshes();
          }
          ImGui::Checkbox("Auto update Strand Model Meshes", &auto_update_strand_model_mesh_);
          ImGui::SameLine();
          ImGui::BeginDisabled(!auto_update_strand_model_mesh_);
          ImGui::SetNextItemWidth(220.0f);
          ImGui::DragFloat("Strand Model Mesh interval (s)", &auto_update_strand_model_mesh_interval_, 0.1f, 0.1f, 3600.0f);
          ImGui::EndDisabled();
          ImGui::Separator();

          if (ImGui::TreeNode("Auto geometry generation")) {
            ImGui::Checkbox("Auto generate mesh", &auto_generate_mesh_after_editing_);
            ImGui::Checkbox("Auto generate Skeletal Graph Per Frame", &auto_generate_skeletal_graph_every_frame_);
            ImGui::Checkbox("Auto generate strands", &auto_generate_strands_after_editing_);
            ImGui::Checkbox("Auto generate strands mesh", &auto_generate_strand_mesh_after_editing_);

            ImGui::TreePop();
          }
          FileUtils::SaveFile(
              "Export all trees as OBJ", "OBJ", {".obj"},
              [&](const std::filesystem::path& path) {
                ExportAllTrees(path);
              },
              false);
          ImGui::TreePop();
        }
        simulation_stats.OnInspect(editor_layer);

        if (ImGui::TreeNodeEx("Tree Visualization settings")) {
          if (ImGui::Button("Update")) {
            need_full_flow_update = true;
          }
          tree_visualization_settings_.OnInspect(editor_layer);
          ImGui::TreePop();
        }
      } else {
        ImGui::Text("No trees in the scene!");
        ResetAllTrees(nullptr);
        auto_time_target_ = 0.0f;
      }
      ImGui::TreePop();
    }
    if (ImGui::TreeNodeEx("Tree Simulation", ImGuiTreeNodeFlags_DefaultOpen)) {
      if (tree_entities && !tree_entities->empty()) {
        if (ImGui::TreeNode("Simulation Settings")) {
          simulation_settings.OnInspect(editor_layer);
          ImGui::TreePop();
        }
        if (ImGui::Button("Reset all trees")) {
          ResetAllTrees(tree_entities);
          ClearMeshes();
          ClearGroundFruitAndLeaf();
          auto_time_target_ = 0.0f;
        }
        ImGui::Text(("Simulated time: " + std::to_string(simulated_time_ / 365.f) + " years").c_str());
        ImGui::DragFloat("Target years", &extra_time_years_, 0.1f, simulated_time_ / 365.f, 999);
        if (auto_time_grow_) {
          if (ImGui::Button("Force stop")) {
            auto_time_grow_ = false;
            auto_time_target_ = simulated_time_;
          }
        } else {
          if (ImGui::Button(("Grow " + std::to_string(extra_time_years_) + " years").c_str())) {
            auto_time_grow_ = true;
            auto_time_target_ += extra_time_years_ * 365.f;
          }
        }
        if (ImGui::Button("Grow 1 iteration")) {
          simulate = true;
        }
      } else {
        ImGui::Text("No trees in the scene!");
        ResetAllTrees(nullptr);
        auto_time_target_ = 0.0f;
      }
      ImGui::TreePop();
    }

    if (ImGui::TreeNodeEx("Fungus Simulation", ImGuiTreeNodeFlags_DefaultOpen)) {
      if (ImGui::DragFloat("Fungus growth rate", &dynamic_strands_settings_.physics_parameters.fungus_growth_rate,
                           0.01f, 0.0f, 1.0f)) {
        // need_full_flow_update = true;
        // TODO: do we need to set a variable here?
      }
      ImGui::TreePop();
    }

    if (ImGui::TreeNodeEx("Soil visualization settings")) {
      soil_visualization_settings_.OnInspect(editor_layer);
      ImGui::TreePop();
    }
  }
  if (ImGui::TreeNodeEx("Dynamic Strands settings", ImGuiTreeNodeFlags_DefaultOpen)) {
    if (ImGui::Button("Initialize all")) {
      GenerateDynamicStrandsForAllTrees();
    }
    if (ImGui::Button("Refresh meshes")) {
      RefreshMeshForAllDynamicStrands();
    }
    if (const std::vector<Entity>* dts_entities = scene->UnsafeGetPrivateComponentOwnersList<DynamicTreeStrands>();
        dts_entities && !dts_entities->empty()) {
      OnInspectDynamicStrandsSettings(editor_layer);
    } else {
      ImGui::Text("No dynamic strands in the scene!");
    }
    ImGui::TreePop();
  }

  if (ImGui::TreeNodeEx("Dynamic Skeleton settings")) {
    if (ImGui::Button("Initialize dynamic skeleton for all trees")) {
      GenerateDynamicSkeletonForAllTrees();
    }
    dynamic_skeleton_settings_.OnInspect(editor_layer);
    ImGui::TreePop();
  }

  if (simulate || auto_time_grow_) {
    Simulate();
  }
  if (const std::vector<Entity>* tree_entities = scene->UnsafeGetPrivateComponentOwnersList<Tree>();
      tree_entities && !tree_entities->empty()) {
    if (auto_time_target_ <= simulated_time_ && auto_time_grow_) {
      auto_time_grow_ = false;
      for (const auto& tree_entity : *tree_entities) {
        auto tree = scene->GetOrSetPrivateComponent<Tree>(tree_entity).lock();
        if (auto_generate_mesh_after_editing_) {
          tree->GenerateGeometryEntities(mesh_generator_settings, -1);
        }
        if (auto_generate_strands_after_editing_ || auto_generate_strand_mesh_after_editing_) {
          tree->BuildStrandModel();
          if (auto_generate_strands_after_editing_) {
            auto strands = tree->GenerateStrands();
            tree->InitializeStrandRenderer(strands);
          }
          if (auto_generate_strand_mesh_after_editing_) {
            tree->InitializeStrandModelMeshRenderer(strand_mesh_generator_settings);
          }
        }
      }
    }
  }
#pragma region Internode debugging camera
  ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2{0, 0});
  if (ImGui::Begin("Plant Visual")) {
    if (ImGui::BeginChild("InternodeCameraRenderer", ImVec2(0, 0), false)) {
      ImVec2 view_port_size;
      view_port_size = ImGui::GetWindowSize();
      const ImVec2 overlay_pos = ImGui::GetWindowPos();
      static int corner = 1;
      visualization_camera_resolution_x = view_port_size.x;
      visualization_camera_resolution_y = view_port_size.y;
      ImGui::Image(visualization_camera_->GetRenderTexture()->GetColorImTextureId(),
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
          if (ImGui::Combo("Render Mode", {"Rasterization", "Ray Tracing"}, mode)) {
            visualization_camera_->camera_render_mode = static_cast<Camera::CameraRenderMode>(mode);
            visualization_camera_->ResetFrameCount();
          }
        }
        ImGui::EndChild();
      }
      visualization_camera_mouse_position = glm::vec2(FLT_MAX, -FLT_MAX);
      auto scene_camera_rotation = editor_layer->GetSceneCameraRotation();
      auto scene_camera_position = editor_layer->GetSceneCameraPosition();
      if (ImGui::IsWindowFocused()) {
        visualization_camera_window_focused_ = true;
        bool valid = true;
        auto mp = ImGui::GetMousePos();
        auto wp = ImGui::GetWindowPos();
        visualization_camera_mouse_position = glm::vec2(mp.x - wp.x, mp.y - wp.y);
        if (valid) {
          static bool is_dragging_previously = false;
          bool mouse_out_of_bounds = visualization_camera_mouse_position.x < 0 || visualization_camera_mouse_position.y < 0 ||
                                     visualization_camera_mouse_position.x > view_port_size.x ||
                                     visualization_camera_mouse_position.y > view_port_size.y;
          bool mouse_drag = true;
          if (EditorLayer::GetKey(GLFW_MOUSE_BUTTON_RIGHT) != Input::KeyActionType::Hold) {
            mouse_drag = false;
          } else if (!is_dragging_previously && mouse_out_of_bounds) {
            mouse_drag = false;
          }
          static float prev_x = 0;
          static float prev_y = 0;
          if (mouse_drag && !is_dragging_previously) {
            prev_x = visualization_camera_mouse_position.x;
            prev_y = visualization_camera_mouse_position.y;
          }
          const float x_offset = visualization_camera_mouse_position.x - prev_x;
          const float y_offset = visualization_camera_mouse_position.y - prev_y;
          prev_x = visualization_camera_mouse_position.x;
          prev_y = visualization_camera_mouse_position.y;
          is_dragging_previously = mouse_drag;
#pragma region Scene Camera Controller
          if (mouse_drag && !editor_layer->lock_camera) {
            glm::vec3 front = scene_camera_rotation * glm::vec3(0, 0, -1);
            glm::vec3 right = scene_camera_rotation * glm::vec3(1, 0, 0);

            float current_velocity = editor_layer->velocity;
            if (EditorLayer::GetKey(GLFW_KEY_LEFT_SHIFT) == Input::KeyActionType::Hold) {
              current_velocity *= 5.0f;
            } else if (EditorLayer::GetKey(GLFW_KEY_LEFT_CONTROL) == Input::KeyActionType::Hold) {
              current_velocity *= 0.2f;
            }

            if (EditorLayer::GetKey(GLFW_KEY_W) == Input::KeyActionType::Hold) {
              scene_camera_position += front * static_cast<float>(Times::DeltaTime()) * current_velocity;
            }
            if (EditorLayer::GetKey(GLFW_KEY_S) == Input::KeyActionType::Hold) {
              scene_camera_position -= front * static_cast<float>(Times::DeltaTime()) * current_velocity;
            }
            if (EditorLayer::GetKey(GLFW_KEY_A) == Input::KeyActionType::Hold) {
              scene_camera_position -= right * static_cast<float>(Times::DeltaTime()) * current_velocity;
            }
            if (EditorLayer::GetKey(GLFW_KEY_D) == Input::KeyActionType::Hold) {
              scene_camera_position += right * static_cast<float>(Times::DeltaTime()) * current_velocity;
            }
            if (EditorLayer::GetKey(GLFW_KEY_E) == Input::KeyActionType::Hold) {
              scene_camera_position.y += current_velocity * static_cast<float>(Times::DeltaTime());
            }
            if (EditorLayer::GetKey(GLFW_KEY_Q) == Input::KeyActionType::Hold) {
              scene_camera_position.y -= current_velocity * static_cast<float>(Times::DeltaTime());
            }
            if (x_offset != 0.0f || y_offset != 0.0f) {
              front = glm::rotate(front, glm::radians(-x_offset * editor_layer->sensitivity), glm::vec3(0, 1, 0));
              const glm::vec3 right = glm::normalize(glm::cross(front, glm::vec3(0.0f, 1.0f, 0.0f)));
              if ((front.y < 0.99f && y_offset < 0.0f) || (front.y > -0.99f && y_offset > 0.0f)) {
                front = glm::rotate(front, glm::radians(-y_offset * editor_layer->sensitivity), right);
              }
              const glm::vec3 up = glm::normalize(glm::cross(right, front));
              scene_camera_rotation = glm::quatLookAt(front, up);
            }
            editor_layer->SetSceneCameraRotation(scene_camera_rotation);
            editor_layer->SetSceneCameraPosition(scene_camera_position);
          }
#pragma endregion
        }
      } else {
        visualization_camera_window_focused_ = false;
      }
      editor_layer->RefEditorCameraRotation(visualization_camera_->GetHandle()) = scene_camera_rotation;
      editor_layer->RefEditorCameraPosition(visualization_camera_->GetHandle()) = scene_camera_position;
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
  if (const auto selected_entity = editor_layer->GetSelectedEntity(); selected_entity != selected_tree) {
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
      auto& shoot_visualizer = tree->shoot_visualizer;
      shoot_visualizer.selected_node_handle = -1;
      shoot_visualizer.selected_node_hierarchy_list.clear();
      tree_operator_mode = static_cast<unsigned>(TreeOperatorMode::Select);
    }
  }
  if (tree_visualization_settings_.enable)
    TreeVisualization(editor_layer);
  if (dynamic_strands_settings_.enable_visualization)
    DynamicStrandsVisualization(editor_layer);
  if (soil_visualization_settings_.enable) {
    SoilVisualization();
  }
#pragma endregion
}

void EcoSysLabLayer::OnInspectDynamicStrandsSettings(const std::shared_ptr<EditorLayer>& editor_layer) {
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

  ImGui::Checkbox("Physics", &dynamic_strands_settings_.enable_physics);
  if (!dynamic_strands_settings_.enable_physics && ImGui::Button("Physics step")) {
    dynamic_strands_settings_.remaining_step++;
  }
  if (ImGui::TreeNode("Physics settings")) {
    dynamic_strands_settings_.physics_parameters.OnInspect(editor_layer);
    ImGui::TreePop();
  }

  ImGui::Checkbox("Rendering", &dynamic_strands_settings_.enable_rendering);
  if (ImGui::TreeNode("Rendering settings")) {
    if (ImGui::TreeNode("Alpha Shape Meshing Settings")) {
      DsAlphaShapeMeshing::OnInspectRenderSettings(editor_layer);
      ImGui::TreePop();
    }
    if (ImGui::TreeNode("Kinetic Voronoi Meshing Settings")) {
      DsKineticVoronoiMeshing::OnInspectRenderSettings(editor_layer);
      ImGui::TreePop();
    }

    ImGui::Checkbox("Render foliage", &dynamic_strands_settings_.foliage_render_parameters.enabled);
    if (dynamic_strands_settings_.foliage_render_parameters.enabled) {
      if (ImGui::TreeNodeEx("Foliage render settings")) {
        if (ImGui::Button("Rebuild foliage pipelines")) {
          DynamicStrands::BuildFoliageRenderingPipelines();
        }
        dynamic_strands_settings_.foliage_render_parameters.OnInspect(editor_layer);
        ImGui::TreePop();
      }
    }

    ImGui::Checkbox("Render segment pairs", &dynamic_strands_settings_.segment_pairs_render_parameters.enabled);
    if (dynamic_strands_settings_.segment_pairs_render_parameters.enabled) {
      if (ImGui::TreeNodeEx("Segment pairs render settings")) {
        if (ImGui::Button("Rebuild segment pairs pipelines")) {
          DynamicStrands::BuildSegmentPairsRenderingPipeline();
        }
        dynamic_strands_settings_.segment_pairs_render_parameters.OnInspect(editor_layer);
        ImGui::TreePop();
      }
    }

    ImGui::TreePop();
  }

  ImGui::Checkbox("Visualization", &dynamic_strands_settings_.enable_visualization);
  if (ImGui::TreeNode("Visualization settings")) {
    dynamic_strands_settings_.visualization_parameters.OnInspect(editor_layer);
    ImGui::TreePop();
  }
}

void EcoSysLabLayer::UpdateFlows(const std::vector<Entity>* tree_entities,
                                 const std::shared_ptr<Strands>& branch_strands) {
  {
    const auto scene = Application::GetActiveScene();

    bounding_box_matrices_->SetParticleInfos({});

    std::vector<int> branch_start_indices;
    int branch_last_start_index = 0;
    branch_start_indices.emplace_back(branch_last_start_index);

    std::vector<int> fruit_start_indices;
    int fruit_last_start_index = 0;
    fruit_start_indices.emplace_back(fruit_last_start_index);

    std::vector<int> leaf_start_indices;
    int leaf_last_start_index = 0;
    leaf_start_indices.emplace_back(leaf_last_start_index);

    std::vector<int> flower_start_indices;
    int flower_last_start_index = 0;
    flower_start_indices.emplace_back(flower_last_start_index);
    if (tree_entities->empty()) {
      shoot_stem_segments_.clear();
      shoot_stem_points_.clear();

      foliage_matrices_->SetParticleInfos({});
      fruit_matrices_->SetParticleInfos({});
      flower_matrices_->SetParticleInfos({});
    }
    std::vector<ParticleInfo> bounding_box_matrices;
    for (int list_index = 0; list_index < tree_entities->size(); list_index++) {
      auto tree_entity = tree_entities->at(list_index);
      const auto tree = scene->GetOrSetPrivateComponent<Tree>(tree_entity).lock();
      auto& tree_model = tree->shoot_model;

      // Check if we need to manually count instead of trusting GetLeafCount
      int current_tree_leaf_count = 0;
      int current_tree_fruit_count = 0;
      int current_tree_flower_count = 0;

      const auto& branch_skeleton = tree_model.RefShootSkeleton();
      const auto& sorted_internode_list = branch_skeleton.PeekSortedNodeList();
      for (const auto& internode_handle : sorted_internode_list) {
        const auto& internode_data = branch_skeleton.PeekNode(internode_handle).data;
        for (const auto& leaf : internode_data.leaves) {
          if (leaf.status != OrganStatus::Inactive) current_tree_leaf_count++;
        }
        for (const auto& fruit : internode_data.fruits) {
          if (fruit.status != OrganStatus::Inactive) current_tree_fruit_count++;
        }
        for (const auto& flower : internode_data.flowers) {
          if (flower.status != OrganStatus::Inactive) current_tree_flower_count++;
        }
      }

      // Use these calculated counts instead of tree_model.Get...Count()
      leaf_last_start_index += current_tree_leaf_count;
      fruit_last_start_index += current_tree_fruit_count;
      flower_last_start_index += current_tree_flower_count;

      auto entity_global_transform = scene->GetDataComponent<GlobalTransform>(tree_entity);
      auto& [instanceMatrix, instanceColor] = bounding_box_matrices.emplace_back();
      instanceMatrix.value =
          entity_global_transform.value * (glm::translate((branch_skeleton.max + branch_skeleton.min) / 2.0f) *
                                           glm::scale(branch_skeleton.max - branch_skeleton.min));
      instanceColor = glm::vec4(random_colors_[list_index], 0.05f);

      fruit_last_start_index += tree_model.GetFruitCount();
      fruit_start_indices.emplace_back(fruit_last_start_index);
      fruit_start_indices.emplace_back(fruit_last_start_index);

      leaf_last_start_index += tree_model.GetLeafCount();
      leaf_start_indices.emplace_back(leaf_last_start_index);
      leaf_start_indices.emplace_back(leaf_last_start_index);

      flower_last_start_index += tree_model.GetFlowerCount();
      flower_start_indices.emplace_back(flower_last_start_index);
      flower_start_indices.emplace_back(flower_last_start_index);

      if (tree_entity != selected_tree) {
        branch_last_start_index += sorted_internode_list.size();
        branch_start_indices.emplace_back(branch_last_start_index);
      } else {
        branch_start_indices.emplace_back(branch_last_start_index);
      }
    }

    bounding_box_matrices_->SetParticleInfos(bounding_box_matrices);

    shoot_stem_segments_.resize(3 * branch_last_start_index);
    shoot_stem_points_.resize(6 * branch_last_start_index);

    {
      std::vector<ParticleInfo> foliage_matrices;
      std::vector<ParticleInfo> flower_matrices;
      std::vector<ParticleInfo> fruit_matrices;
      foliage_matrices.resize(leaf_last_start_index);
      flower_matrices.resize(flower_last_start_index);
      fruit_matrices.resize(fruit_last_start_index);
      Jobs::RunParallelFor(tree_entities->size(), [&](unsigned tree_index) {
        auto tree_entity = tree_entities->at(tree_index);
        auto tree = scene->GetOrSetPrivateComponent<Tree>(tree_entity).lock();
        auto& tree_model = tree->shoot_model;
        const auto& branch_skeleton = tree_model.RefShootSkeleton();
        const auto& branch_flow_list = branch_skeleton.PeekSortedFlowList();
        auto entity_global_transform = scene->GetDataComponent<GlobalTransform>(tree_entity);
        auto branch_start_index = branch_start_indices[tree_index];

        auto leaf_start_index = leaf_start_indices[tree_index];
        auto fruit_start_index = fruit_start_indices[tree_index];
        auto flower_start_index = flower_start_indices[tree_index];
        int leaf_index = 0;
        int fruit_index = 0;
        int flower_index = 0;

        const auto& sorted_internode_list = branch_skeleton.PeekSortedNodeList();
        for (const auto& internode_handle : sorted_internode_list) {
          const auto& internode_data = branch_skeleton.PeekNode(internode_handle).data;
          for (const auto& leaf : internode_data.leaves) {
            if (leaf.status != OrganStatus::Inactive) {
              glm::mat4 leaf_transform =
                  glm::translate(leaf.position) * glm::mat4_cast(leaf.rotation) * glm::scale(leaf.scale * .5f);
              foliage_matrices[leaf_start_index + leaf_index].instance_matrix.value =
                  entity_global_transform.value * leaf_transform;
              foliage_matrices[leaf_start_index + leaf_index].instance_color =
                  glm::vec4(glm::mix(glm::vec3(152 / 255.0f, 203 / 255.0f, 0 / 255.0f),
                                     glm::vec3(159 / 255.0f, 100 / 255.0f, 66 / 255.0f), 1.0f - leaf.health),
                            0.5f);
              leaf_index++;
            }
          }

          for (const auto& flower : internode_data.flowers) {
            if (flower.status != OrganStatus::Inactive) {
              glm::mat4 flower_transform =
                  glm::translate(flower.position) * glm::mat4_cast(flower.rotation) * glm::scale(flower.scale);
              flower_matrices[flower_start_index + flower_index].instance_matrix.value =
                  entity_global_transform.value * flower_transform;
              flower_matrices[flower_start_index + flower_index].instance_color =
                  glm::vec4(glm::mix(glm::vec3(255 / 255.0f, 255 / 255.0f, 255 / 255.0f),
                                     glm::vec3(255 / 255.0f, 192 / 255.0f, 203 / 255.0f), flower.maturity),
                            0.75f);
              flower_index++;
            }
          }

          for (const auto& fruit : internode_data.fruits) {
            if (fruit.status != OrganStatus::Inactive) {
              glm::mat4 fruit_transform =
                  glm::translate(fruit.position) * glm::mat4_cast(fruit.rotation) * glm::scale(fruit.scale * .25f);
              fruit_matrices[fruit_start_index + fruit_index].instance_matrix.value =
                  entity_global_transform.value * fruit_transform;
              fruit_matrices[fruit_start_index + fruit_index].instance_color =
                  glm::vec4(glm::mix(glm::vec3(152 / 255.0f, 255 / 255.0f, 152 / 255.0f),
                                     glm::vec3(255 / 255.0f, 165 / 255.0f, 0 / 255.0f), fruit.maturity),
                            0.75f);
              fruit_index++;
            }
          }
        }
        if (tree_entity == selected_tree)
          return;
        for (int i = 0; i < branch_flow_list.size(); i++) {
          auto& flow = branch_skeleton.PeekFlow(branch_flow_list[i]);
          auto cp1 = flow.info.global_start_position;
          auto cp4 = flow.info.global_end_position;
          float distance = glm::distance(cp1, cp4);
          glm::vec3 cp0, cp2;
          if (flow.GetParentHandle() > 0) {
            cp0 = cp1 + branch_skeleton.PeekFlow(flow.GetParentHandle()).info.global_end_rotation * glm::vec3(0, 0, 1) *
                            distance / 3.0f;
            cp2 = cp1 + branch_skeleton.PeekFlow(flow.GetParentHandle()).info.global_end_rotation *
                            glm::vec3(0, 0, -1) * distance / 3.0f;
          } else {
            cp0 = cp1 + flow.info.global_start_rotation * glm::vec3(0, 0, 1) * distance / 3.0f;
            cp2 = cp1 + flow.info.global_start_rotation * glm::vec3(0, 0, -1) * distance / 3.0f;
          }
          auto cp3 = cp4 + flow.info.global_end_rotation * glm::vec3(0, 0, 1) * distance / 3.0f;
          auto cp5 = cp4 + flow.info.global_end_rotation * glm::vec3(0, 0, -1) * distance / 3.0f;

          auto& p0 = shoot_stem_points_[branch_start_index * 6 + i * 6];
          auto& p1 = shoot_stem_points_[branch_start_index * 6 + i * 6 + 1];
          auto& p2 = shoot_stem_points_[branch_start_index * 6 + i * 6 + 2];
          auto& p3 = shoot_stem_points_[branch_start_index * 6 + i * 6 + 3];
          auto& p4 = shoot_stem_points_[branch_start_index * 6 + i * 6 + 4];
          auto& p5 = shoot_stem_points_[branch_start_index * 6 + i * 6 + 5];
          p0.position = (entity_global_transform.value * glm::translate(cp0))[3];
          p1.position = (entity_global_transform.value * glm::translate(cp1))[3];
          p2.position = (entity_global_transform.value * glm::translate(cp2))[3];
          p3.position = (entity_global_transform.value * glm::translate(cp3))[3];
          p4.position = (entity_global_transform.value * glm::translate(cp4))[3];
          p5.position = (entity_global_transform.value * glm::translate(cp5))[3];
          if (flow.GetParentHandle() > 0) {
            p1.thickness = branch_skeleton.PeekFlow(flow.GetParentHandle()).info.end_thickness * 0.5f;
          } else {
            p1.thickness = flow.info.start_thickness * 0.5f;
          }
          p4.thickness = flow.info.end_thickness * 0.5f;

          p2.thickness = p3.thickness = (p1.thickness + p4.thickness) * 0.5f;
          p0.thickness = 2.0f * p1.thickness - p2.thickness;
          p5.thickness = 2.0f * p4.thickness - p3.thickness;

          p0.color = glm::vec4(random_colors_[flow.info.order], 1.0f);
          p1.color = glm::vec4(random_colors_[flow.info.order], 1.0f);
          p2.color = glm::vec4(random_colors_[flow.info.order], 1.0f);
          p3.color = glm::vec4(random_colors_[flow.info.order], 1.0f);
          p4.color = glm::vec4(random_colors_[flow.info.order], 1.0f);
          p5.color = glm::vec4(random_colors_[flow.info.order], 1.0f);

          shoot_stem_segments_[branch_start_index * 3 + i * 3] = branch_start_index * 6 + i * 6;
          shoot_stem_segments_[branch_start_index * 3 + i * 3 + 1] = branch_start_index * 6 + i * 6 + 1;
          shoot_stem_segments_[branch_start_index * 3 + i * 3 + 2] = branch_start_index * 6 + i * 6 + 2;
        }
      });
      StrandPointAttributes strand_point_attributes{};
      strand_point_attributes.normal = false;
      branch_strands->SetSegments(strand_point_attributes, shoot_stem_segments_, shoot_stem_points_);
      foliage_matrices_->SetParticleInfos(foliage_matrices);
      fruit_matrices_->SetParticleInfos(fruit_matrices);
      flower_matrices_->SetParticleInfos(flower_matrices);
    }
  }
}

void EcoSysLabLayer::ClearGroundFruitAndLeaf() {
  fruits_.clear();
  leaves_.clear();
  flowers_.clear();
  UpdateGroundFruitAndLeaves();
}

void EcoSysLabLayer::UpdateGroundFruitAndLeaves() const {
  std::vector<ParticleInfo> fruit_matrices;
  fruit_matrices.resize(fruits_.size());
  for (int i = 0; i < fruits_.size(); i++) {
    fruit_matrices[i].instance_matrix.value = fruits_[i].global_transform.value;
    fruit_matrices[i].instance_matrix.SetScale(fruit_matrices[i].instance_matrix.GetScale() * 0.25f);
    fruit_matrices[i].instance_color =
        glm::vec4(glm::mix(glm::vec3(152 / 255.0f, 255 / 255.0f, 152 / 255.0f),
                           glm::vec3(255 / 255.0f, 165 / 255.0f, 0 / 255.0f), fruits_[i].fruit_maturity),
                  0.75f);
  }
  std::vector<ParticleInfo> flower_matrices;
  flower_matrices.resize(flowers_.size());
  for (int i = 0; i < flowers_.size(); i++) {
    flower_matrices[i].instance_matrix.value = flowers_[i].global_transform.value;
    flower_matrices[i].instance_matrix.SetScale(flower_matrices[i].instance_matrix.GetScale() * 0.5f);
    flower_matrices[i].instance_color =
        glm::vec4(glm::mix(glm::vec3(255 / 255.0f, 255 / 255.0f, 255 / 255.0f),
                           glm::vec3(255 / 255.0f, 192 / 255.0f, 203 / 255.0f), flowers_[i].flower_maturity),
                  0.75f);
  }
  std::vector<ParticleInfo> leaf_matrices;
  leaf_matrices.resize(leaves_.size());
  for (int i = 0; i < leaves_.size(); i++) {
    leaf_matrices[i].instance_matrix.value = leaves_[i].global_transform.value;
    leaf_matrices[i].instance_matrix.SetScale(leaf_matrices[i].instance_matrix.GetScale() * 0.5f);
    leaf_matrices[i].instance_color =
        glm::vec4(glm::mix(glm::vec3(152 / 255.0f, 203 / 255.0f, 0 / 255.0f),
                           glm::vec3(159 / 255.0f, 100 / 255.0f, 66 / 255.0f), 1.0f - leaves_[i].leaf_health),
                  0.5f);
  }
  ground_fruit_matrices_->SetParticleInfos(fruit_matrices);
  ground_leaf_matrices_->SetParticleInfos(leaf_matrices);
  ground_flower_matrices_->SetParticleInfos(flower_matrices);
}

void EcoSysLabLayer::VisualizationCameraDragAndDrop() const {
  if (AssetRef asset_ref; EditorLayer::UnsafeDroppableAsset(asset_ref, {"Scene", "Prefab", "Mesh", "TreeDescriptor"})) {
    const auto scene = GetScene();
    if (const auto asset = asset_ref.Get<IAsset>(); asset->GetTypeName() == "TreeDescriptor") {
      std::dynamic_pointer_cast<TreeDescriptor>(asset)->Instantiate();
    }
  }
}

float EcoSysLabLayer::GetSimulatedTime() const {
  return simulated_time_;
}

glm::vec2 EcoSysLabLayer::GetMouseSceneCameraPosition() const {
  return visualization_camera_mouse_position;
}

void EcoSysLabLayer::Update() {
  const auto scene = GetScene();
  if (!scene)
    return;

  if (auto_update_strand_renderer_) {
    const double now = Times::Now();
    const double interval = glm::max(0.1f, auto_update_strand_renderer_interval_);
    if (next_strand_renderer_update_time_ <= 0.0) {
      next_strand_renderer_update_time_ = now + interval;
    } else if (now >= next_strand_renderer_update_time_) {
      GenerateStrandRenderers();
      next_strand_renderer_update_time_ = now + interval;
    }
  } else {
    next_strand_renderer_update_time_ = 0.0;
  }

  if (auto_update_strand_model_mesh_) {
    const double now = Times::Now();
    const double interval = glm::max(0.1f, auto_update_strand_model_mesh_interval_);
    if (next_strand_model_mesh_update_time_ <= 0.0) {
      next_strand_model_mesh_update_time_ = now + interval;
    } else if (now >= next_strand_model_mesh_update_time_) {
      GenerateStrandModelMeshes(strand_mesh_generator_settings);
      next_strand_model_mesh_update_time_ = now + interval;
    }
  } else {
    next_strand_model_mesh_update_time_ = 0.0;
  }

  // Register terrain tessellation render instances for all active Soil entities
  if (const auto scene = GetScene()) {
    if (const auto* soil_entities = scene->UnsafeGetPrivateComponentOwnersList<Soil>()) {
      for (const auto& entity : *soil_entities) {
        if (scene->IsEntityEnabled(entity)) {
          if (auto soil = scene->GetOrSetPrivateComponent<Soil>(entity).lock()) {
            if (soil->IsEnabled()) {
              soil->RegisterTerrainRenderInstance();
            }
          }
        }
      }
    }
  }

  RegisterStrandRenderingProcedure();
  DynamicSkeletonPhysics();
  DynamicStrandSimulation();
}

void EcoSysLabLayer::LateUpdate() {
  const auto scene = GetScene();
  if (!scene)
    return;
  DynamicSkeletonVisualization();
  DynamicStrandVisualization();
}