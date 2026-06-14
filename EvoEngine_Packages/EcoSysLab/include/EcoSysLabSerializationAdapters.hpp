#pragma once

#include "AdvancedShootDescriptor.hpp"
#include "BasicBarkDescriptor.hpp"
#include "BasicFineRootDescriptor.hpp"
#include "BasicFoliageDescriptor.hpp"
#include "BasicPruningDescriptor.hpp"
#include "BasicReproductionModuleDescriptor.hpp"
#include "BasicRootDescriptor.hpp"
#include "BasicShootDescriptor.hpp"
#include "Climate.hpp"
#include "CubeVolume.hpp"
#include "DsColliders.hpp"
#include "DynamicTreeStrandGraph.hpp"
#include "DynamicTreeStrands.hpp"
#include "ForestDescriptor.hpp"
#include "HeightField.hpp"
#include "ObjectRotator.hpp"
#include "RadialBoundingVolume.hpp"
#include "Soil.hpp"
#include "SoilDescriptor.hpp"
#include "Tree.hpp"
#include "TreeDescriptor.hpp"
#include "TreeStructor.hpp"

namespace eco_sys_lab_package {
class DynamicStrandsDemo;
class DynamicTreeSkeleton;
class FungusTest;
class ParticlePhysics2DDemo;
class Physics2DDemo;
class SpatialPlantDistributionSimulator;

void SerializeSpatialPlantDistributionSimulator(YAML::Emitter& out, const SpatialPlantDistributionSimulator& target);
void DeserializeSpatialPlantDistributionSimulator(const YAML::Node& in, SpatialPlantDistributionSimulator& target);
void SerializeDynamicTreeSkeleton(YAML::Emitter& out, const DynamicTreeSkeleton& target);
void DeserializeDynamicTreeSkeleton(const YAML::Node& in, DynamicTreeSkeleton& target);
void SerializeDynamicStrandsDemo(YAML::Emitter& out, const DynamicStrandsDemo& target);
void DeserializeDynamicStrandsDemo(const YAML::Node& in, DynamicStrandsDemo& target);
void SerializePhysics2DDemo(YAML::Emitter& out, const Physics2DDemo& target);
void DeserializePhysics2DDemo(const YAML::Node& in, Physics2DDemo& target);
void SerializeParticlePhysics2DDemo(YAML::Emitter& out, const ParticlePhysics2DDemo& target);
void DeserializeParticlePhysics2DDemo(const YAML::Node& in, ParticlePhysics2DDemo& target);
void SerializeFungusTest(YAML::Emitter& out, const FungusTest& target);
void DeserializeFungusTest(const YAML::Node& in, FungusTest& target);
void SerializeObjectRotator(YAML::Emitter& out, const ObjectRotator& target);
void DeserializeObjectRotator(const YAML::Node& in, ObjectRotator& target);
void SerializeBasicFineRootDescriptor(YAML::Emitter& out, const BasicFineRootDescriptor& target);
void DeserializeBasicFineRootDescriptor(const YAML::Node& in, BasicFineRootDescriptor& target);
void SerializeAdvancedShootDescriptor(YAML::Emitter& out, const AdvancedShootDescriptor& target);
void DeserializeAdvancedShootDescriptor(const YAML::Node& in, AdvancedShootDescriptor& target);
void SerializeBasicReproductionModuleDescriptor(YAML::Emitter& out, const BasicReproductionModuleDescriptor& target);
void DeserializeBasicReproductionModuleDescriptor(const YAML::Node& in, BasicReproductionModuleDescriptor& target);
void SerializeBasicFoliageDescriptor(YAML::Emitter& out, const BasicFoliageDescriptor& target);
void DeserializeBasicFoliageDescriptor(const YAML::Node& in, BasicFoliageDescriptor& target);
void SerializeHeightField(YAML::Emitter& out, const HeightField& target);
void DeserializeHeightField(const YAML::Node& in, HeightField& target);
void SerializeClimateDescriptor(YAML::Emitter& out, const ClimateDescriptor& target);
void DeserializeClimateDescriptor(const YAML::Node& in, ClimateDescriptor& target);
void SerializeClimate(YAML::Emitter& out, const Climate& target);
void DeserializeClimate(const YAML::Node& in, Climate& target);
void SerializeForestPatch(YAML::Emitter& out, const ForestPatch& target);
void DeserializeForestPatch(const YAML::Node& in, ForestPatch& target);
void SerializeTreeInfo(YAML::Emitter& out, const TreeInfo& target);
void DeserializeTreeInfo(const YAML::Node& in, TreeInfo& target);
void SerializeForestDescriptor(YAML::Emitter& out, const ForestDescriptor& target);
void DeserializeForestDescriptor(const YAML::Node& in, ForestDescriptor& target);
void SerializeBasicBarkDescriptor(YAML::Emitter& out, const BasicBarkDescriptor& target);
void DeserializeBasicBarkDescriptor(const YAML::Node& in, BasicBarkDescriptor& target);
void SerializeTreeDescriptor(YAML::Emitter& out, const TreeDescriptor& target);
void DeserializeTreeDescriptor(const YAML::Node& in, TreeDescriptor& target);
void SerializeBasicPruningDescriptor(YAML::Emitter& out, const BasicPruningDescriptor& target);
void DeserializeBasicPruningDescriptor(const YAML::Node& in, BasicPruningDescriptor& target);
void SerializeBasicShootDescriptor(YAML::Emitter& out, const BasicShootDescriptor& target);
void DeserializeBasicShootDescriptor(const YAML::Node& in, BasicShootDescriptor& target);
void SerializeBasicRootDescriptor(YAML::Emitter& out, const BasicRootDescriptor& target);
void DeserializeBasicRootDescriptor(const YAML::Node& in, BasicRootDescriptor& target);
void SerializeSoilLayerDescriptor(YAML::Emitter& out, const SoilLayerDescriptor& target);
void DeserializeSoilLayerDescriptor(const YAML::Node& in, SoilLayerDescriptor& target);
void SerializeSoilDescriptor(YAML::Emitter& out, const SoilDescriptor& target);
void DeserializeSoilDescriptor(const YAML::Node& in, SoilDescriptor& target);
void SerializeTreeStructor(YAML::Emitter& out, const TreeStructor& target);
void DeserializeTreeStructor(const YAML::Node& in, TreeStructor& target);
void SerializeTree(YAML::Emitter& out, const Tree& target);
void DeserializeTree(const YAML::Node& in, Tree& target);
void SerializeSoil(YAML::Emitter& out, const Soil& target);
void DeserializeSoil(const YAML::Node& in, Soil& target);
void SerializeDsBoxCollider(YAML::Emitter& out, const DsBoxCollider& target);
void DeserializeDsBoxCollider(const YAML::Node& in, DsBoxCollider& target);
void SerializeDsCylinderCollider(YAML::Emitter& out, const DsCylinderCollider& target);
void DeserializeDsCylinderCollider(const YAML::Node& in, DsCylinderCollider& target);
void SerializeDsSphereCollider(YAML::Emitter& out, const DsSphereCollider& target);
void DeserializeDsSphereCollider(const YAML::Node& in, DsSphereCollider& target);
void SerializeDynamicTreeStrands(YAML::Emitter& out, const DynamicTreeStrands& target);
void DeserializeDynamicTreeStrands(const YAML::Node& in, DynamicTreeStrands& target);
void SerializeRadialBoundingVolume(YAML::Emitter& out, const RadialBoundingVolume& target);
void DeserializeRadialBoundingVolume(const YAML::Node& in, RadialBoundingVolume& target);
void SerializeCubeVolume(YAML::Emitter& out, const CubeVolume& target);
void DeserializeCubeVolume(const YAML::Node& in, CubeVolume& target);
void SerializeModulusGraph(YAML::Emitter& out, const ModulusGraph& target);
void DeserializeModulusGraph(const YAML::Node& in, ModulusGraph& target);
void SerializeStrengthGraph(YAML::Emitter& out, const StrengthGraph& target);
void DeserializeStrengthGraph(const YAML::Node& in, StrengthGraph& target);
void SerializeBiologicalPropertiesGraph(YAML::Emitter& out, const BiologicalPropertiesGraph& target);
void DeserializeBiologicalPropertiesGraph(const YAML::Node& in, BiologicalPropertiesGraph& target);
}  // namespace eco_sys_lab_package
