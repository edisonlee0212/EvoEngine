//
// Created by lllll on 11/1/2022.
//

#include "EcoSysLabLayer.hpp"

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
#include "ForestDescriptor.hpp"
#include "Soil.hpp"
#include "SpatialPlantDistributionSimulator.hpp"
#include "Times.hpp"
#include "Tree.hpp"
#include "TreeStructor.hpp"
using namespace eco_sys_lab_package;

void EcoSysLabLayer::ResetAllTrees(const std::vector<Entity>* tree_entities) {
  const auto scene = ApplicationContext::Get().GetActiveScene();
  simulated_time_ = 0;
  ++reset_revision_;
  if (tree_entities) {
    for (const auto& i : *tree_entities) {
      const auto tree = scene->GetOrSetPrivateComponent<Tree>(i).lock();
      tree->Reset();
    }
  }

  ++simulation_revision_;
  simulation_stats = {};

  const auto climate_candidate = FindClimate();
  if (!climate_candidate.expired()) {
    const auto climate = climate_candidate.lock();
    climate->climate_model.environment_grid = {};
  }
}

bool EcoSysLabLayer::Simulate(const SimulationSettings& target_simulation_settings,
                              SimulationStats& target_simulation_stats) {
  const auto scene = GetScene();
  const std::vector<Entity>* tree_entities = scene->UnsafeGetPrivateComponentOwnersList<Tree>();
  simulated_time_ += target_simulation_settings.delta_time;
  bool tree_grown = false;
  if (tree_entities && !tree_entities->empty()) {
    float time = ApplicationContext::Get().GetTimes().Now();

    std::shared_ptr<Climate> climate;
    std::shared_ptr<Soil> soil;
    if (const auto climate_candidate = FindClimate(); !climate_candidate.expired())
      climate = climate_candidate.lock();
    if (const auto soil_candidate = FindSoil(); !soil_candidate.expired())
      soil = soil_candidate.lock();
    if (!soil) {
      EVOENGINE_ERROR("Simulation Failed! No soil in scene!");
      return tree_grown;
    }
    if (!climate) {
      EVOENGINE_ERROR("Simulation Failed! No climate in scene!");
      return tree_grown;
    }
    climate->climate_model.time = simulated_time_;

    if (target_simulation_settings.soil_simulation) {
      soil->soil_model.Irrigation();
      soil->soil_model.Step();
    }
    for (const auto& tree_entity : *tree_entities) {
      auto tree = scene->GetOrSetPrivateComponent<Tree>(tree_entity).lock();
      tree->climate = climate;
      tree->soil = soil;
    }
    climate->PrepareForGrowth();
    std::vector<bool> grown_stat{};
    grown_stat.resize(Jobs::GetWorkerSize());
    Jobs::RunParallelFor(tree_entities->size(), [&](size_t i, size_t thread_index) {
      const auto tree_entity = tree_entities->at(i);
      if (!scene->IsEntityEnabled(tree_entity))
        return;
      const auto tree = scene->GetOrSetPrivateComponent<Tree>(tree_entity).lock();
      if (!tree->IsEnabled())
        return;
      if (tree->start_time > simulated_time_)
        return;
      if (target_simulation_settings.max_node_count > 0 &&
          tree->shoot_model.RefShootSkeleton().PeekSortedNodeList().size() >= target_simulation_settings.max_node_count)
        return;
      if (target_simulation_settings.max_flow_count > 0 &&
          tree->shoot_model.RefShootSkeleton().PeekSortedFlowList().size() >= target_simulation_settings.max_flow_count)
        return;
      grown_stat[thread_index] = tree->TryGrow(target_simulation_settings, -1, true);
    });

    auto height_field = soil->soil_descriptor_ref.Get<SoilDescriptor>()->height_field.Get<HeightField>();
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
          const auto ground_height = height_field->GetValue({position.x, position.z});
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
          const auto ground_height = height_field->GetValue({position.x, position.z});
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
  if (simulation_settings.auto_clear_fruit_and_leaves) {
    ClearGroundFruitAndLeaf();
  }
  ++simulation_revision_;
  return tree_grown;
}

bool EcoSysLabLayer::Simulate() {
  return Simulate(simulation_settings, simulation_stats);
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
