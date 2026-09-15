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
#include "EcoSysLabSerializationAdapters.hpp"
#include "Octree.hpp"
#include "Soil.hpp"
#include "StrandModelProfileSerializer.hpp"

using namespace eco_sys_lab_package;

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
  ++shoot_model_revision_.content;
  ++shoot_model_revision_.topology;
  ++root_model_revision_.content;
  ++root_model_revision_.topology;
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
}

void Tree::OnCreate() {
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
      ++shoot_model_revision_.content;
      if (pruning)
        ++shoot_model_revision_.topology;
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
      ++root_model_revision_.content;
      if (pruning)
        ++root_model_revision_.topology;
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

void eco_sys_lab_package::SerializeTree(YAML::Emitter& out, const Tree& target) {
  target.tree_descriptor_ref.Save("tree_descriptor_ref", out);

  target.strand_model_parameters.Save("strand_model_parameters", out);
  target.tree_mesh_generator_settings.Save("tree_mesh_generator_settings", out);
  target.shoot_strand_model.Save("shoot_strand_model", out);
  target.shoot_model.Save("shoot_model", out);
}

void eco_sys_lab_package::DeserializeTree(const YAML::Node& in, Tree& target) {
  ++target.shoot_model_revision_.content;
  ++target.shoot_model_revision_.topology;
  ++target.root_model_revision_.content;
  ++target.root_model_revision_.topology;
  target.tree_descriptor_ref.Load("tree_descriptor_ref", in);

  target.strand_model_parameters.Load("strand_model_parameters", in);
  target.tree_mesh_generator_settings.Load("tree_mesh_generator_settings", in);

  target.shoot_strand_model.Load("shoot_strand_model", in);
  target.shoot_model.Load("shoot_model", in);
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
