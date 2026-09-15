//
// Created by lllll on 11/1/2022.
//

#include "EcoSysLabLayer.hpp"

#include "BasicBarkDescriptor.hpp"
#include "ClassRegistry.hpp"
#include "DsColliders.hpp"
#include "DsOperators.hpp"
#include "DynamicTreeSkeleton.hpp"
#include "DynamicTreeStrands.hpp"
#include "Soil.hpp"
#include "SpatialPlantDistributionSimulator.hpp"
#include "Tree.hpp"
#include "TreeStructor.hpp"

using namespace eco_sys_lab_package;

void EcoSysLabLayer::GenerateStrandModelProfiles() const {
  const auto scene = GetScene();
  if (const std::vector<Entity>* tree_entities = scene->UnsafeGetPrivateComponentOwnersList<Tree>();
      tree_entities && !tree_entities->empty()) {
    const auto copied_entities = *tree_entities;
    for (auto tree_entity : copied_entities) {
      if (const auto tree = scene->GetOrSetPrivateComponent<Tree>(tree_entity).lock(); tree->generate_mesh)
        tree->BuildStrandModel();
    }
  }
}

void EcoSysLabLayer::GenerateStrandModelMeshes(
    const StrandModelMeshGeneratorSettings& target_strand_model_mesh_generator_settings) const {
  const auto scene = GetScene();
  if (const std::vector<Entity>* tree_entities = scene->UnsafeGetPrivateComponentOwnersList<Tree>();
      tree_entities && !tree_entities->empty()) {
    const auto copied_entities = *tree_entities;
    for (auto tree_entity : copied_entities) {
      if (const auto tree = scene->GetOrSetPrivateComponent<Tree>(tree_entity).lock(); tree->generate_mesh)
        tree->InitializeStrandModelMeshRenderer(target_strand_model_mesh_generator_settings);
    }
  }
}

void EcoSysLabLayer::GenerateStrandRenderers() const {
  const auto scene = GetScene();
  if (const std::vector<Entity>* tree_entities = scene->UnsafeGetPrivateComponentOwnersList<Tree>();
      tree_entities && !tree_entities->empty()) {
    const auto copied_entities = *tree_entities;
    for (auto tree_entity : copied_entities) {
      if (const auto tree = scene->GetOrSetPrivateComponent<Tree>(tree_entity).lock(); tree->generate_mesh)
        tree->InitializeStrandRenderer();
    }
  }
}

void EcoSysLabLayer::ClearStrandRenderers() const {
  const auto scene = GetScene();
  if (const std::vector<Entity>* tree_entities = scene->UnsafeGetPrivateComponentOwnersList<Tree>();
      tree_entities && !tree_entities->empty()) {
    const auto copied_entities = *tree_entities;
    for (auto tree_entity : copied_entities) {
      const auto tree = scene->GetOrSetPrivateComponent<Tree>(tree_entity).lock();
      tree->ClearStrandRenderer();
    }
  }
}

void EcoSysLabLayer::ClearStrandModelMeshes() const {
  const auto scene = GetScene();
  if (const std::vector<Entity>* tree_entities = scene->UnsafeGetPrivateComponentOwnersList<Tree>();
      tree_entities && !tree_entities->empty()) {
    const auto copied_entities = *tree_entities;
    for (auto tree_entity : copied_entities) {
      const auto tree = scene->GetOrSetPrivateComponent<Tree>(tree_entity).lock();
      tree->ClearStrandModelMeshRenderer();
    }
  }
}

void EcoSysLabLayer::GenerateDynamicStrandsForAllTrees() const {
  const auto scene = GetScene();
  if (const std::vector<Entity>* tree_entities = scene->UnsafeGetPrivateComponentOwnersList<Tree>();
      tree_entities && !tree_entities->empty()) {
    for (auto tree_entity : *tree_entities) {
      const auto tree = scene->GetOrSetPrivateComponent<Tree>(tree_entity).lock();
      tree->BuildStrandModel();
      const auto ds = scene->GetOrSetPrivateComponent<DynamicTreeStrands>(tree_entity).lock();
      if (const auto td = tree->tree_descriptor_ref.Get<TreeDescriptor>()) {
        ds->initialize_parameters.foliage_descriptor = td->foliage_descriptor;
        if (const auto fd = td->foliage_descriptor.Get<BasicFoliageDescriptor>()) {
          if (const auto mat = fd->leaf_material_ref.Get<Material>())
            ds->materials.leaf_material_ref = mat;
        }
        if (const auto bd = td->bark_descriptor.Get<BasicBarkDescriptor>()) {
          if (const auto mat = bd->bark_material_ref.Get<Material>())
            ds->materials.bark_material_ref = mat;
        }
      }

      ds->strand_model = tree->shoot_strand_model;
      DtsStrandGroup randomly_subdivided_strand_group{}, uniformly_subdivided_strand_group{};
      ds->initialized_from_tree = true;
      ds->UpdateDynamicStrands(randomly_subdivided_strand_group, uniformly_subdivided_strand_group);
      ds->dynamic_strands->Upload();
      ds->dynamic_strands->InitializeMesh(ds->initialize_parameters);
    }
  }
  if (const std::vector<Entity>* dts_entities = scene->UnsafeGetPrivateComponentOwnersList<DynamicTreeStrands>();
      dts_entities && !dts_entities->empty()) {
    for (auto dts_entity : *dts_entities) {
      if (scene->HasPrivateComponent<Tree>(dts_entity))
        continue;
      const auto ds = scene->GetOrSetPrivateComponent<DynamicTreeStrands>(dts_entity).lock();
      DtsStrandGroup randomly_subdivided_strand_group{}, uniformly_subdivided_strand_group{};
      ds->UpdateDynamicStrands(randomly_subdivided_strand_group, uniformly_subdivided_strand_group);
      ds->dynamic_strands->Upload();
      ds->dynamic_strands->InitializeMesh(ds->initialize_parameters);
    }
  }
}

void EcoSysLabLayer::RefreshMeshForAllDynamicStrands() const {
  const auto scene = GetScene();
  if (const std::vector<Entity>* ds_entities = scene->UnsafeGetPrivateComponentOwnersList<DynamicTreeStrands>();
      ds_entities && !ds_entities->empty()) {
    for (auto ds_entity : *ds_entities) {
      const auto ds = scene->GetOrSetPrivateComponent<DynamicTreeStrands>(ds_entity).lock();
      ds->dynamic_strands->InitializeMesh(ds->initialize_parameters);
    }
  }
}

void EcoSysLabLayer::GenerateDynamicSkeletonForAllTrees() const {
  const auto scene = GetScene();
  if (const std::vector<Entity>* tree_entities = scene->UnsafeGetPrivateComponentOwnersList<Tree>();
      tree_entities && !tree_entities->empty()) {
    for (auto tree_entity : *tree_entities) {
      const auto tree = scene->GetOrSetPrivateComponent<Tree>(tree_entity).lock();
      const auto ds = scene->GetOrSetPrivateComponent<DynamicTreeSkeleton>(tree_entity).lock();
      ds->initialize_parameters.root_transform = scene->GetDataComponent<GlobalTransform>(tree_entity);
      ds->dynamic_skeleton.Initialize(ds->initialize_parameters, tree->shoot_strand_model.strand_model_skeleton);
    }
  }
}
