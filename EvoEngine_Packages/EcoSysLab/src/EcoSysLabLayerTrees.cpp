//
// Created by lllll on 11/1/2022.
//

#include "EcoSysLabLayer.hpp"
#include "Profiler.hpp"

#include <algorithm>
#include <cmath>
#include <type_traits>
#include "AdvancedShootDescriptor.hpp"
#include "BasicBarkDescriptor.hpp"
#include "BasicFineRootDescriptor.hpp"
#include "BasicFoliageDescriptor.hpp"
#include "BasicPruningDescriptor.hpp"
#include "BasicReproductionModuleDescriptor.hpp"
#include "BasicRootDescriptor.hpp"
#include "ClassRegistry.hpp"
#include "Climate.hpp"
#include "DynamicTreeStrandGraph.hpp"
#include "DynamicTreeStrands.hpp"
#include "EcoSysLabSystem.hpp"
#include "ForestDescriptor.hpp"
#include "Soil.hpp"
#include "SpatialPlantDistributionSimulator.hpp"
#include "Times.hpp"
#include "Tree.hpp"
#include "TreeStructor.hpp"
using namespace eco_sys_lab_package;

namespace {
template <typename T, typename F>
void VisitFungus(T& p, F&& f) {
#define FIELD(name) f(#name, p.name)
  FIELD(dt);
  FIELD(aw);
  FIELD(ab);
  FIELD(bw);
  FIELD(bb);
  FIELD(ycw);
  FIELD(ycb);
  FIELD(ylw);
  FIELD(pc);
  FIELD(pl);
  FIELD(k);
  FIELD(delta);
  FIELD(ll);
  FIELD(lc);
  FIELD(bo);
  FIELD(kc);
  FIELD(be);
  FIELD(brw);
  FIELD(brb);
  FIELD(msr);
  FIELD(cpb);
  FIELD(cpw);
  FIELD(lignin_threshold);
  FIELD(HL_threshold);
  FIELD(HC_threshold);
  FIELD(bd_offset);
  FIELD(matrixAw);
  FIELD(matrixAb);
  FIELD(matrixAc);
  FIELD(matrixAm);
  FIELD(global_parameter);
  FIELD(treespace);
#undef FIELD
}

template <typename T, typename F>
void VisitPhysics(T& p, F&& f) {
  VisitFungus(p, f);
#define FIELD(name) f(#name, p.name)
  FIELD(time_step);
  FIELD(sub_step);
  FIELD(position_constraint_iteration);
  FIELD(velocity_constraint_iteration);
  FIELD(enable_structural_damage);
  FIELD(enable_segment_breaking);
  FIELD(enable_segment_disconnection);
  FIELD(enable_foliage_detachment);
  FIELD(enable_segment_tensile_disconnection);
  FIELD(enable_segment_compression_disconnection);
  FIELD(compression_strength_factor);
  FIELD(enable_positional_breaking);
  FIELD(enable_rotational_breaking);
  FIELD(segment_velocity_damping);
  FIELD(segment_angular_velocity_damping);
  FIELD(leaf_velocity_damping);
  FIELD(leaf_angular_velocity_damping);
  FIELD(enable_segment_collision);
  FIELD(dynamic_grouping);
  FIELD(grouping_iteration);
  FIELD(gravity);
  FIELD(fungus_growth_rate);
  FIELD(enable_fungus);
  FIELD(a_geom);
  FIELD(b_vel);
  FIELD(c_bias);
  FIELD(s_min);
  FIELD(s_max_ratio);
  FIELD(eta);
  FIELD(bmax_far);
  FIELD(rod_strength_factor);
  FIELD(bundle_strength_factor);
  FIELD(boundary_strength_decay_factor);
  FIELD(pivot_ring_radius);
  FIELD(crack_bd_shrinkage_offset);
  FIELD(crack_R_scale);
  FIELD(crack_T_scale);
  FIELD(moisture_breaking_rod);
  FIELD(internal_pattern);
  FIELD(pull_cubical);
  FIELD(leaf_break_from_moisture);
  FIELD(leaf_break_threshold);
#undef FIELD
}

template <typename T, typename F>
void VisitStrands(T& p, F&& f) {
#define FIELD(name) f(#name, p.name)
  FIELD(enable_physics);
  FIELD(enable_fungus);
  FIELD(enable_geometry_updates);
  FIELD(enable_rendering);
  FIELD(fungus_sub_step);
#undef FIELD
  f("foliage_enabled", p.foliage_render_parameters.enabled);
  f("foliage_wireframe", p.foliage_render_parameters.wireframe);
  f("segment_pairs_enabled", p.segment_pairs_render_parameters.enabled);
  f("segment_pairs_thickness_multiplier", p.segment_pairs_render_parameters.thickness_multiplier);
  f("segment_pair_render_mode", p.segment_pairs_render_parameters.segment_pair_render_mode);
  f("segment_pairs_position_scale", p.segment_pairs_render_parameters.position_scale);
  f("segment_pair_color_min", p.segment_pairs_render_parameters.segment_pair_color_min);
  f("segment_pair_color_max", p.segment_pairs_render_parameters.segment_pair_color_max);
  f("segment_pair_color_main", p.segment_pairs_render_parameters.segment_pair_color_main);
  f("segment_pair_radius_multiplier", p.segment_pairs_render_parameters.segment_pair_radius_multiplier);
}

template <typename T, typename F>
void SaveFields(YAML::Emitter& out, const T& value, F&& visit) {
  out << YAML::BeginMap;
  visit(value, [&](const char* name, const auto& field) {
    out << YAML::Key << name << YAML::Value;
    if constexpr (std::is_same_v<std::decay_t<decltype(field)>, glm::mat3>)
      out << YAML::Flow << YAML::BeginSeq << field[0] << field[1] << field[2] << YAML::EndSeq;
    else
      out << field;
  });
  out << YAML::EndMap;
}

template <typename T, typename F>
void LoadFields(const YAML::Node& in, T& value, F&& visit) {
  if (!in)
    return;
  visit(value, [&](const char* name, auto& field) {
    if (const auto node = in[name]) {
      using Field = std::decay_t<decltype(field)>;
      if constexpr (std::is_same_v<Field, glm::mat3>) {
        if (node.IsSequence() && node.size() == 3)
          for (int column = 0; column < 3; ++column)
            field[column] = node[column].as<glm::vec3>();
      } else {
        field = node.as<Field>();
      }
    }
  });
}
}  // namespace

void EcoSysLabSystem::Growth::Queue(const float days) {
  if (std::isfinite(days) && days > 0.0f)
    remaining_time += days;
}

float EcoSysLabSystem::Growth::TakeStep() {
  if (remaining_time <= 0.0f || !(settings.delta_time > 0.0f))
    return 0.0f;
  const float step = std::min(settings.delta_time, remaining_time);
  remaining_time = std::max(0.0f, remaining_time - step);
  return step;
}

void EcoSysLabSystem::Growth::Stop() {
  remaining_time = 0.0f;
  invigoration.reset();
}

void EcoSysLabSystem::Growth::RequestInvigoration(const Entity tree, const int node) {
  invigoration = std::make_pair(tree, node);
}

void EcoSysLabSystem::FixedUpdate() {
  dynamics.Step(GetScene(), static_cast<float>(ApplicationContext::Get().GetTimes().FixedDeltaTime()),
                growth.remaining_time > 0.0f || growth.grew_this_frame);
}

void EcoSysLabSystem::OnDestroy() {
  dynamics.Reset();
}

void EcoSysLabSystem::Interactions::Clear() {
  flowers.clear();
  fruits.clear();
  leaves.clear();
  ++revision;
}

void EcoSysLabSystem::Update() {
  const auto layer = ApplicationContext::Get().GetLayer<EcoSysLabLayer>();
  if (!layer || layer->GetScene() != GetScene())
    return;
  growth.grew_this_frame = false;
  if (growth.remaining_time <= 0.0f) {
    if (growth.invigoration) {
      const auto [entity, node] = *growth.invigoration;
      growth.invigoration.reset();
      const auto scene = GetScene();
      if (scene->IsEntityValid(entity) && scene->IsEntityEnabled(entity) && scene->HasPrivateComponent<Tree>(entity)) {
        environment.EnsureConfiguration(scene);
        environment.InitializeModels();
        environment.PrepareForGrowth(scene, growth.settings);
        const auto tree = scene->GetOrSetPrivateComponent<Tree>(entity).lock();
        if (tree && tree->IsEnabled())
          growth.grew_this_frame = tree->TryGrow(growth.settings, node, false, environment.climate, environment.soil);
      }
    }
    dynamics.StepStrands(GetScene());
    return;
  }
  environment.EnsureConfiguration(GetScene());
  environment.InitializeModels();
  const float step = growth.TakeStep();
  if (!(step > 0.0f)) {
    dynamics.StepStrands(GetScene());
    return;
  }
  auto settings = growth.settings;
  settings.delta_time = step;
  growth.grew_this_frame = true;
  growth.Step(*this, settings, growth.stats);
  dynamics.StepStrands(GetScene());
}

void EcoSysLabSystem::PostCloneAction(const std::shared_ptr<ISystem>&) {
  growth.Stop();
  growth.simulated_time = 0.0f;
  growth.stats = {};
  growth.grew_this_frame = false;
  environment.initialized = false;
  environment.climate.environment_grid = {};
  environment.soil = {};
  dynamics.tree_physics.reset();
  dynamics.cluster_forest.reset();
  dynamics.cluster_members.clear();
  dynamics.physics_scene.reset();
  dynamics.entries.clear();
  dynamics.wind.reset();
  dynamics.strands.remaining_step = 0;
  dynamics.strands.remaining_fungus_step = 0;
  dynamics.strands.remaining_geometry_step = 0;
  interactions = {};
}

void eco_sys_lab_package::SerializeEcoSysLabSystem(YAML::Emitter& out, const EcoSysLabSystem& target) {
  out << YAML::Key << "branch_rendering" << YAML::Value << YAML::BeginMap;
  out << YAML::Key << "screen_space_tessellation" << YAML::Value << target.rendering.screen_space_tessellation;
  out << YAML::Key << "camera_impostors" << YAML::Value << target.rendering.camera_impostors;
  out << YAML::Key << "canopy_shadows" << YAML::Value << target.rendering.canopy_shadows;
  out << YAML::Key << "canopy_shadow_texels" << YAML::Value << target.rendering.canopy_shadow_texels;
  out << YAML::Key << "impostor_diameter_pixels" << YAML::Value << target.rendering.impostor_diameter_pixels;
  out << YAML::Key << "pixel_error" << YAML::Value << target.rendering.pixel_error;
  out << YAML::Key << "segment_length_pixels" << YAML::Value << target.rendering.segment_length_pixels;
  out << YAML::EndMap;
  const auto& env = target.environment;
  env.climate_descriptor.Save("climate_descriptor", out);
  env.soil_descriptor.Save("soil_descriptor", out);
  out << YAML::Key << "soil_visualization_owner" << YAML::Value << env.soil_visualization_owner;
  out << YAML::Key << "air_temperature_c" << YAML::Value << env.air_temperature_c;
  out << YAML::Key << "air_relative_humidity" << YAML::Value << env.air_relative_humidity;
  out << YAML::Key << "air_pressure_pa" << YAML::Value << env.air_pressure_pa;
  out << YAML::Key << "air_coupling_rate" << YAML::Value << env.air_coupling_rate;
  out << YAML::Key << "air_friction" << YAML::Value << env.air_friction;
  out << YAML::Key << "air_cell_size" << YAML::Value << env.air_cell_size;
  target.growth.settings.Save("simulation_settings", out);
  out << YAML::Key << "physics_approximation_tolerance" << YAML::Value
      << target.dynamics.physics_approximation_tolerance;
  out << YAML::Key << "use_subtree_clusters" << YAML::Value << target.dynamics.use_subtree_clusters;
  out << YAML::Key << "cluster_lod" << YAML::Value;
  SerializeTreeClusterLodSettings(out, target.dynamics.cluster_lod);
  const auto& strands = target.dynamics.strands;
  out << YAML::Key << "strands_settings" << YAML::Value;
  SaveFields(out, strands, [](const auto& value, auto&& f) {
    VisitStrands(value, f);
  });
  out << YAML::Key << "strands_physics" << YAML::Value;
  SaveFields(out, strands.physics_parameters, [](const auto& value, auto&& f) {
    VisitPhysics(value, f);
  });
  out << YAML::Key << "strands_fungus" << YAML::Value;
  SaveFields(out, strands.fungus_parameters, [](const auto& value, auto&& f) {
    VisitFungus(value, f);
  });
}

void eco_sys_lab_package::DeserializeEcoSysLabSystem(const YAML::Node& in, EcoSysLabSystem& target) {
  target.rendering = {};
  if (const auto settings = in["branch_rendering"]) {
    if (settings["screen_space_tessellation"])
      target.rendering.screen_space_tessellation = settings["screen_space_tessellation"].as<bool>();
    if (settings["camera_impostors"])
      target.rendering.camera_impostors = settings["camera_impostors"].as<bool>();
    if (settings["canopy_shadows"])
      target.rendering.canopy_shadows = settings["canopy_shadows"].as<bool>();
    if (settings["canopy_shadow_texels"])
      target.rendering.canopy_shadow_texels = settings["canopy_shadow_texels"].as<float>();
    if (settings["impostor_diameter_pixels"])
      target.rendering.impostor_diameter_pixels = settings["impostor_diameter_pixels"].as<float>();
    if (settings["pixel_error"])
      target.rendering.pixel_error = settings["pixel_error"].as<float>();
    if (settings["segment_length_pixels"])
      target.rendering.segment_length_pixels = settings["segment_length_pixels"].as<float>();
  }
  auto& env = target.environment;
  env.climate_descriptor.Load("climate_descriptor", in);
  env.soil_descriptor.Load("soil_descriptor", in);
  if (in["soil_visualization_owner"])
    env.soil_visualization_owner = Handle(in["soil_visualization_owner"].as<uint64_t>());
  if (in["air_temperature_c"])
    env.air_temperature_c = in["air_temperature_c"].as<float>();
  if (in["air_relative_humidity"])
    env.air_relative_humidity = in["air_relative_humidity"].as<float>();
  if (in["air_pressure_pa"])
    env.air_pressure_pa = in["air_pressure_pa"].as<float>();
  if (in["air_coupling_rate"])
    env.air_coupling_rate = in["air_coupling_rate"].as<float>();
  if (in["air_friction"])
    env.air_friction = in["air_friction"].as<float>();
  if (in["air_cell_size"])
    env.air_cell_size = glm::clamp(in["air_cell_size"].as<float>(), 0.05f, 0.2f);
  target.growth.settings.Load("simulation_settings", in);
  if (in["physics_approximation_tolerance"])
    target.dynamics.physics_approximation_tolerance = glm::max(0.0f, in["physics_approximation_tolerance"].as<float>());
  if (in["use_subtree_clusters"])
    target.dynamics.use_subtree_clusters = in["use_subtree_clusters"].as<bool>();
  DeserializeTreeClusterLodSettings(in["cluster_lod"], target.dynamics.cluster_lod);
  auto& strands = target.dynamics.strands;
  LoadFields(in["strands_settings"], strands, [](auto& value, auto&& f) {
    VisitStrands(value, f);
  });
  LoadFields(in["strands_physics"], strands.physics_parameters, [](auto& value, auto&& f) {
    VisitPhysics(value, f);
  });
  LoadFields(in["strands_fungus"], strands.fungus_parameters, [](auto& value, auto&& f) {
    VisitFungus(value, f);
  });
  target.growth.Stop();
  target.growth.simulated_time = 0.0f;
  target.growth.grew_this_frame = false;
  env.initialized = false;
}

void EcoSysLabLayer::ResetAllTrees(const std::vector<Entity>* tree_entities) {
  const auto scene = ApplicationContext::Get().GetActiveScene();
  const auto system = scene->GetSystem<EcoSysLabSystem>();
  if (system) {
    system->dynamics.Reset();
    system->growth.Stop();
    system->growth.simulated_time = 0.0f;
    system->growth.stats = {};
    system->environment.climate.environment_grid = {};
    system->environment.initialized = false;
    ++system->interactions.reset_revision;
  }
  if (tree_entities) {
    for (const auto& i : *tree_entities) {
      const auto tree = scene->GetOrSetPrivateComponent<Tree>(i).lock();
      tree->Reset();
    }
  }
  if (system) {
    ++system->interactions.revision;
  }
}

bool EcoSysLabSystem::Growth::Step(EcoSysLabSystem& system, const SimulationSettings& target_simulation_settings,
                                   SimulationStats& target_simulation_stats) {
  const ProfilerScope growth_profile_scope("EcoSysLab / Growth step", "EcoSysLab");
  const auto scene = system.GetScene();
  auto& simulated_time = system.growth.simulated_time;
  auto& climate = system.environment.climate;
  auto& soil = system.environment.soil;
  auto& flowers_ = system.interactions.flowers;
  auto& fruits_ = system.interactions.fruits;
  auto& leaves_ = system.interactions.leaves;
  auto& simulation_revision_ = system.interactions.revision;
  using Flower = EcoSysLabSystem::Interactions::Flower;
  using Fruit = EcoSysLabSystem::Interactions::Fruit;
  using Leaf = EcoSysLabSystem::Interactions::Leaf;
  const std::vector<Entity>* tree_entities = scene->UnsafeGetPrivateComponentOwnersList<Tree>();
  simulated_time += target_simulation_settings.delta_time;
  bool tree_grown = false;
  if (tree_entities && !tree_entities->empty()) {
    float time = ApplicationContext::Get().GetTimes().Now();

    climate.time = simulated_time;

    if (target_simulation_settings.soil_simulation) {
      soil.Irrigation();
      soil.Step();
    }
    system.environment.PrepareForGrowth(scene, target_simulation_settings);
    std::vector<uint8_t> grown_stat(tree_entities->size());
    {
      const ProfilerScope trees_scope("EcoSysLab / Grow trees parallel", "EcoSysLab");
      Jobs::RunParallelFor(tree_entities->size(), [&](size_t i) {
        const auto tree_entity = tree_entities->at(i);
        if (!scene->IsEntityEnabled(tree_entity))
          return;
        const auto tree = scene->GetOrSetPrivateComponent<Tree>(tree_entity).lock();
        if (!tree->IsEnabled())
          return;
        if (tree->start_time > simulated_time)
          return;
        if (target_simulation_settings.max_node_count > 0 &&
            tree->shoot_model.RefShootSkeleton().PeekSortedNodeList().size() >=
                target_simulation_settings.max_node_count)
          return;
        if (target_simulation_settings.max_flow_count > 0 &&
            tree->shoot_model.RefShootSkeleton().PeekSortedFlowList().size() >=
                target_simulation_settings.max_flow_count)
          return;
        grown_stat[i] = tree->TryGrow(target_simulation_settings, -1, true, climate, soil);
      });
    }

    const auto soil_descriptor = system.environment.soil_descriptor.Get<SoilDescriptor>();
    const auto height_field = soil_descriptor ? soil_descriptor->height_field.Get<HeightField>() : nullptr;
    for (const auto& tree_entity : *tree_entities) {
      if (!scene->IsEntityEnabled(tree_entity))
        continue;
      auto tree = scene->GetOrSetPrivateComponent<Tree>(tree_entity).lock();
      auto tree_global_transform = scene->GetDataComponent<GlobalTransform>(tree_entity);
      if (!tree->IsEnabled())
        continue;
      // Collect fruit and leaves here.
      if (!target_simulation_settings.auto_clear_fruit_and_leaves) {
        for (const auto& flower : tree->shoot_model.RefShootSkeleton().data.dropped_flowers) {
          Flower new_flower;
          Transform flower_transform;
          flower_transform.value =
              glm::translate(flower.position) * glm::mat4_cast(flower.rotation) * glm::scale(flower.scale);
          new_flower.global_transform.value = tree_global_transform.value * flower_transform.value;

          auto position = new_flower.global_transform.GetPosition();
          const auto ground_height = height_field ? height_field->GetValue({position.x, position.z}) : 0.0f;
          const auto height = position.y - ground_height;
          position.x += glm::gaussRand(0.0f, height * 0.1f);
          position.z += glm::gaussRand(0.0f, height * 0.1f);
          position.y = ground_height + 0.1f;
          new_flower.global_transform.SetPosition(position);

          new_flower.flower_maturity = flower.maturity;
          new_flower.flower_health = flower.health;
          flowers_.emplace_back(new_flower);
        }

        for (const auto& fruit : tree->shoot_model.RefShootSkeleton().data.dropped_fruits) {
          Fruit new_fruit;
          Transform fruit_transform;
          fruit_transform.value =
              glm::translate(fruit.position) * glm::mat4_cast(fruit.rotation) * glm::scale(fruit.scale);
          new_fruit.global_transform.value = tree_global_transform.value * fruit_transform.value;

          auto position = new_fruit.global_transform.GetPosition();
          const auto ground_height = height_field ? height_field->GetValue({position.x, position.z}) : 0.0f;
          const auto height = position.y - ground_height;
          position.x += glm::gaussRand(0.0f, height * 0.1f);
          position.z += glm::gaussRand(0.0f, height * 0.1f);
          position.y = ground_height + 0.1f;
          new_fruit.global_transform.SetPosition(position);

          new_fruit.fruit_maturity = fruit.maturity;
          new_fruit.fruit_health = fruit.health;
          fruits_.emplace_back(new_fruit);
        }

        for (const auto& leaf : tree->shoot_model.RefShootSkeleton().data.dropped_leaves) {
          Leaf new_leaf;
          Transform leaf_transform;
          leaf_transform.value = glm::translate(leaf.position) * glm::mat4_cast(leaf.rotation) * glm::scale(leaf.scale);
          new_leaf.global_transform.value = tree_global_transform.value * leaf_transform.value;

          auto position = new_leaf.global_transform.GetPosition();
          const auto ground_height = height_field ? height_field->GetValue({position.x, position.z}) : 0.0f;
          const auto height = position.y - ground_height;
          position.x += glm::gaussRand(0.0f, height * 0.1f);
          position.z += glm::gaussRand(0.0f, height * 0.1f);
          position.y = ground_height + 0.1f;
          new_leaf.global_transform.SetPosition(position);

          new_leaf.leaf_maturity = leaf.maturity;
          new_leaf.leaf_health = leaf.health;
          leaves_.emplace_back(new_leaf);
        }
      }
      tree->shoot_model.RefShootSkeleton().data.dropped_fruits.clear();
      tree->shoot_model.RefShootSkeleton().data.dropped_flowers.clear();
      tree->shoot_model.RefShootSkeleton().data.dropped_leaves.clear();
    }
    target_simulation_stats.last_used_time = ApplicationContext::Get().GetTimes().Now() - time;
    target_simulation_stats.total_time += target_simulation_stats.last_used_time;

    for (auto&& i : grown_stat) {
      if (i) {
        tree_grown = true;
      }
    }
    if (tree_grown) {
      int total_internode_size = 0;
      int total_flow_size = 0;
      int total_root_node_size = 0;
      int total_root_flow_size = 0;
      int total_leaf_size = 0;
      int total_fruit_size = 0;
      for (auto tree_entity : *tree_entities) {
        auto tree = scene->GetOrSetPrivateComponent<Tree>(tree_entity).lock();
        auto& tree_model = tree->shoot_model;
        total_internode_size += tree_model.RefShootSkeleton().PeekSortedNodeList().size();
        total_flow_size += tree_model.RefShootSkeleton().PeekSortedFlowList().size();
        total_leaf_size += tree_model.GetLeafCount();
        total_fruit_size += tree_model.GetFruitCount();
      }
      target_simulation_stats.internode_size = total_internode_size;
      target_simulation_stats.shoot_stem_size = total_flow_size;
      target_simulation_stats.root_node_size = total_root_node_size;
      target_simulation_stats.root_stem_size = total_root_flow_size;
      target_simulation_stats.leaf_size = total_leaf_size;
      target_simulation_stats.fruit_size = total_fruit_size;
    }
  }
  if (target_simulation_settings.auto_clear_fruit_and_leaves) {
    system.interactions.Clear();
  }
  ++simulation_revision_;
  return tree_grown;
}

bool EcoSysLabLayer::Simulate() {
  const auto system = GetScene()->GetOrCreateSystem<EcoSysLabSystem>(0.0f);
  return Simulate(system->growth.settings, system->growth.stats);
}

bool EcoSysLabLayer::Simulate(const SimulationSettings& target_simulation_settings, SimulationStats&) {
  const auto scene = GetScene();
  if (!scene)
    return false;
  auto& growth = scene->GetOrCreateSystem<EcoSysLabSystem>(0.0f)->growth;
  growth.settings = target_simulation_settings;
  growth.Queue(target_simulation_settings.delta_time);
  return false;
}

bool EcoSysLabLayer::SimulateForDataset(const SimulationSettings& settings, SimulationStats& stats) {
  const auto scene = GetScene();
  const auto system = scene->GetOrCreateSystem<EcoSysLabSystem>(0.0f);
  system->environment.EnsureConfiguration(scene);
  system->environment.InitializeModels();
  return system->growth.Step(*system, settings, stats);
}

void EcoSysLabLayer::GenerateMeshes(const TreeMeshGeneratorSettings& target_mesh_generator_settings) const {
  const auto scene = GetScene();
  if (const std::vector<Entity>* tree_entities = scene->UnsafeGetPrivateComponentOwnersList<Tree>();
      tree_entities && !tree_entities->empty()) {
    const auto copied_entities = *tree_entities;
    for (auto tree_entity : copied_entities) {
      if (const auto tree = scene->GetOrSetPrivateComponent<Tree>(tree_entity).lock(); tree->generate_mesh)
        tree->GenerateGeometryEntities(target_mesh_generator_settings);
    }
  }
}

void EcoSysLabLayer::GenerateSkeletalGraphs(const SkeletalGraphSettings& target_skeletal_graph_settings) const {
  const auto scene = GetScene();
  if (const std::vector<Entity>* tree_entities = scene->UnsafeGetPrivateComponentOwnersList<Tree>();
      tree_entities && !tree_entities->empty()) {
    const auto copied_entities = *tree_entities;
    for (auto tree_entity : copied_entities) {
      if (const auto tree = scene->GetOrSetPrivateComponent<Tree>(tree_entity).lock(); tree->generate_mesh)
        tree->GenerateSkeletalGraph(skeletal_graph_settings, -1, Resources::GetInstance().GetPrimitives().sphere,
                                    Resources::GetInstance().GetPrimitives().cube);
    }
  }
}

void EcoSysLabLayer::ClearMeshes() const {
  const auto scene = GetScene();
  if (const std::vector<Entity>* tree_entities = scene->UnsafeGetPrivateComponentOwnersList<Tree>();
      tree_entities && !tree_entities->empty()) {
    const auto copied_entities = *tree_entities;
    for (auto tree_entity : copied_entities) {
      const auto tree = scene->GetOrSetPrivateComponent<Tree>(tree_entity).lock();
      tree->ClearGeometryEntities();
    }
  }
}

void EcoSysLabLayer::ClearSkeletalGraphs() const {
  const auto scene = GetScene();
  if (const std::vector<Entity>* tree_entities = scene->UnsafeGetPrivateComponentOwnersList<Tree>();
      tree_entities && !tree_entities->empty()) {
    const auto copied_entities = *tree_entities;
    for (auto tree_entity : copied_entities) {
      const auto tree = scene->GetOrSetPrivateComponent<Tree>(tree_entity).lock();
      tree->ClearSkeletalGraph();
    }
  }
}

void EcoSysLabLayer::ExportAllTrees(const std::filesystem::path& path) const {
  const auto scene = GetScene();
  if (const std::vector<Entity>* tree_entities = scene->UnsafeGetPrivateComponentOwnersList<Tree>();
      tree_entities && !tree_entities->empty()) {
    if (path.extension() == ".obj") {
      std::ofstream of;
      of.open(path.string(), std::ofstream::out | std::ofstream::trunc);
      if (of.is_open()) {
        std::string start = "#Forest OBJ exporter, by Bosheng Li";
        start += "\n";
        of.write(start.c_str(), start.size());
        of.flush();
        unsigned start_index = 1;
        if (mesh_generator_settings.enable_shoot_branch) {
          unsigned tree_index = 0;
          for (const auto& entity : *tree_entities) {
            const auto tree = scene->GetOrSetPrivateComponent<Tree>(entity).lock();
            const auto mesh = tree->GenerateShootMesh(mesh_generator_settings);
            auto& vertices = mesh->UnsafeGetVertices();
            auto& triangles = mesh->UnsafeGetTriangles();
            const auto gt = scene->GetDataComponent<GlobalTransform>(entity);
            if (!vertices.empty() && !triangles.empty()) {
              std::string header =
                  "#Vertices: " + std::to_string(vertices.size()) + ", tris: " + std::to_string(triangles.size());
              header += "\n";
              of.write(header.c_str(), header.size());
              of.flush();
              std::stringstream data;
              data << "o branch " + std::to_string(tree_index) + "\n";
#pragma region Data collection
              for (auto& vertex : vertices) {
                auto vertex_position = glm::vec4(vertex.position, 1.0f);
                vertex_position = gt.value * vertex_position;
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
            tree_index++;
          }
        }
        if (mesh_generator_settings.enable_foliage) {
          unsigned tree_index = 0;
          for (const auto& entity : *tree_entities) {
            const auto tree = scene->GetOrSetPrivateComponent<Tree>(entity).lock();
            const auto mesh = tree->GenerateFoliageMesh(mesh_generator_settings);
            auto& vertices = mesh->UnsafeGetVertices();
            auto& triangles = mesh->UnsafeGetTriangles();
            const auto gt = scene->GetDataComponent<GlobalTransform>(entity);
            if (!vertices.empty() && !triangles.empty()) {
              std::string header =
                  "#Vertices: " + std::to_string(vertices.size()) + ", tris: " + std::to_string(triangles.size());
              header += "\n";
              of.write(header.c_str(), header.size());
              of.flush();
              std::stringstream data;
              data << "o foliage " + std::to_string(tree_index) + "\n";
#pragma region Data collection
              for (auto& vertex : vertices) {
                auto vertex_position = glm::vec4(vertex.position, 1.0f);
                vertex_position = gt.value * vertex_position;
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
            tree_index++;
          }
        }
        of.close();
      }
    }
  }
}
