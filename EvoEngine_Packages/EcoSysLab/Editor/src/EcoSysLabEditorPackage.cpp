#include "DynamicStrandsComponentInspectors.hpp"
#include "DynamicStrandsMeshingInspector.hpp"
#include "DynamicStrandsPhysicsInspectors.hpp"
#include "DynamicsSettingsEditor.hpp"
#include "EcoSysLabAssetPreviews.hpp"
#include "EcoSysLabAuthoringInspectors.hpp"
#include "EcoSysLabDescriptorInspectors.hpp"
#include "EcoSysLabEditorLayer.hpp"
#include "EcoSysLabGraphEditors.hpp"
#include "EcoSysLabObjectInspectors.hpp"
#include "EditorPackage.hpp"
#include "PhysicsDemoInspectors.hpp"
#include "SpatialPlantDistributionInspector.hpp"
#include "TreeAssetInspectors.hpp"

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
#include "DsOperators.hpp"
#include "DynamicStrandsDemo.hpp"
#include "DynamicStrandsProfiler.hpp"
#include "DynamicTreeSkeleton.hpp"
#include "DynamicTreeStrandGraph.hpp"
#include "DynamicTreeStrands.hpp"
#include "EcoSysLabLayer.hpp"
#include "EcoSysLabSerializationAdapters.hpp"
#include "ForestDescriptor.hpp"
#include "FungusTest.hpp"
#include "HeightField.hpp"
#include "InspectorRegistry.hpp"
#include "ObjectRotator.hpp"
#include "ParticlePhysics2DDemo.hpp"
#include "Physics2DDemo.hpp"
#include "RadialBoundingVolume.hpp"
#include "Serialization.hpp"
#include "Soil.hpp"
#include "SoilDescriptor.hpp"
#include "SpatialPlantDistributionSimulator.hpp"
#include "Tree.hpp"
#include "TreeDescriptor.hpp"
#include "TreeStructor.hpp"

#include <iterator>
#include <string>
#include <type_traits>

using namespace eco_sys_lab_package;
using namespace evo_engine;

namespace {
bool RegisterEcoSysLabInspectors(EditorPackageRegistrar& registrar) {
  bool registered = registrar.RegisterLayer<EcoSysLabEditorLayer>("EcoSysLab Editor");
  registered &= registrar.RegisterInspector<ConstantNode>(InspectConstantNode, "ConstantNode");
  registered &= registrar.RegisterInspector<TreeStructor>(
      [state = std::make_shared<TreeStructorInspector>()](InspectorContext& context, TreeStructor& target) {
        return state->Inspect(context, target);
      },
      "TreeStructor");
  registered &= registrar.RegisterInspector<Climate>(
      [state = std::make_shared<ClimateInspector>()](InspectorContext& context, Climate& target) {
        return state->Inspect(context, target);
      },
      "Climate");
  registered &= registrar.RegisterInspector<SpatialPlantDistributionSimulator>(
      [state = std::make_shared<SpatialPlantDistributionInspector>()](InspectorContext& context,
                                                                      SpatialPlantDistributionSimulator& target) {
        return state->Inspect(context, target);
      },
      "SpatialPlantDistributionSimulator");
  registered &= registrar.RegisterInspector<DynamicTreeSkeleton>(
      [state = std::make_shared<DynamicTreeSkeletonInspector>()](InspectorContext& context,
                                                                 DynamicTreeSkeleton& target) {
        return state->Inspect(context, target);
      },
      "DynamicTreeSkeleton");
  registered &= registrar.RegisterInspector<DynamicStrandsDemo>(
      [state = std::make_shared<DynamicStrandsDemoInspector>()](InspectorContext& context, DynamicStrandsDemo& target) {
        return state->Inspect(context, target);
      },
      "DynamicStrandsDemo");
  registered &= registrar.RegisterInspector<Tree>(
      [](InspectorContext& context, Tree& target) {
        return GetTreeEditorState(target).Inspect(context, target);
      },
      "Tree");
  registered &= registrar.RegisterInspector<DynamicStrandsInitializeParameters>(
      [state = std::make_shared<DynamicStrandsInitializationInspector>()](InspectorContext& context,
                                                                          DynamicStrandsInitializeParameters& target) {
        return state->Inspect(context, target);
      },
      "DynamicStrandsInitializeParameters");

  registered &= registrar.RegisterInspector<DsAlphaShapeMeshing>(
      [](InspectorContext& context, DsAlphaShapeMeshing& target) {
        return DynamicStrandsMeshingInspector::Inspect(context, target);
      },
      "DsAlphaShapeMeshing");
  registered &= registrar.RegisterInspector<DsKineticVoronoiMeshing>(
      [](InspectorContext& context, DsKineticVoronoiMeshing& target) {
        return DynamicStrandsMeshingInspector::Inspect(context, target);
      },
      "DsKineticVoronoiMeshing");
  registered &= registrar.RegisterInspector<DsAlphaShapeVisualizationParameters>(
      [](InspectorContext& context, DsAlphaShapeVisualizationParameters& target) {
        return DynamicStrandsMeshingInspector::Inspect(context, target);
      },
      "DsAlphaShapeVisualizationParameters");

  registered &= registrar.RegisterInspector<DsLeafDrop>(InspectDsLeafDrop, "DsLeafDrop");
  registered &= registrar.RegisterInspector<DsAttraction>(InspectDsAttraction, "DsAttraction");
  registered &= registrar.RegisterInspector<DsSnow>(InspectDsSnow, "DsSnow");
  registered &= registrar.RegisterInspector<DsWind>(InspectDsWind, "DsWind");
  registered &= registrar.RegisterInspector<DsStiffRod>(InspectDsStiffRod, "DsStiffRod");
  registered &= registrar.RegisterInspector<DsBundle>(InspectDsBundle, "DsBundle");
  registered &= registrar.RegisterInspector<DsFungus>(InspectDsFungus, "DsFungus");
  registered &= registrar.RegisterInspector<DsPrediction>(InspectDsPrediction, "DsPrediction");
  registered &= registrar.RegisterInspector<DsDynamicHashedGrid>(InspectDsDynamicHashedGrid, "DsDynamicHashedGrid");

  registered &= registrar.RegisterInspector<LSystemString>(InspectLSystemString, "LSystemString");
  registered &= registrar.RegisterInspector<TreeGraph>(InspectTreeGraph, "TreeGraph");
  registered &= registrar.RegisterInspector<TreeGraphV2>(InspectTreeGraphV2, "TreeGraphV2");

  registered &= registrar.RegisterInspector<Soil>(
      [state = std::make_shared<SoilInspector>()](InspectorContext& context, Soil& target) {
        return state->Inspect(context, target);
      },
      "Soil");
  registered &= registrar.RegisterInspector<DsBoxCollider>(
      [state = std::make_shared<DsBoxColliderInspector>()](InspectorContext& context, DsBoxCollider& target) {
        return state->Inspect(context, target);
      },
      "DsBoxCollider");
  registered &= registrar.RegisterInspector<DsSphereCollider>(
      [state = std::make_shared<DsSphereColliderInspector>()](InspectorContext& context, DsSphereCollider& target) {
        return state->Inspect(context, target);
      },
      "DsSphereCollider");
  registered &= registrar.RegisterInspector<DsCylinderCollider>(
      [state = std::make_shared<DsCylinderColliderInspector>()](InspectorContext& context, DsCylinderCollider& target) {
        return state->Inspect(context, target);
      },
      "DsCylinderCollider");
  registered &= registrar.RegisterInspector<DynamicTreeStrands>(
      [state = std::make_shared<DynamicTreeStrandsInspector>()](InspectorContext& context, DynamicTreeStrands& target) {
        return state->Inspect(context, target);
      },
      "DynamicTreeStrands");
  registered &= registrar.RegisterInspector<ObjectRotator>(
      [state = std::make_shared<ObjectRotatorInspector>()](InspectorContext& context, ObjectRotator& target) {
        return state->Inspect(context, target);
      },
      "ObjectRotator");
  registered &= registrar.RegisterInspector<Physics2DDemo>(
      [state = std::make_shared<Physics2DDemoInspector>()](InspectorContext& context, Physics2DDemo& target) {
        return state->Inspect(context, target);
      },
      "Physics2DDemo");
  registered &= registrar.RegisterInspector<ParticlePhysics2DDemo>(
      [state = std::make_shared<ParticlePhysics2DDemoInspector>()](InspectorContext& context,
                                                                   ParticlePhysics2DDemo& target) {
        return state->Inspect(context, target);
      },
      "ParticlePhysics2DDemo");
  registered &= registrar.RegisterInspector<FungusTest>(
      [state = std::make_shared<FungusTestInspector>()](InspectorContext& context, FungusTest& target) {
        return state->Inspect(context, target);
      },
      "FungusTest");
  registered &= registrar.RegisterInspector<ClimateDescriptor>(
      [state = std::make_shared<ClimateDescriptorInspector>()](InspectorContext& context, ClimateDescriptor& target) {
        return state->Inspect(context, target);
      },
      "ClimateDescriptor");
  registered &= registrar.RegisterInspector<RadialBoundingVolume>(
      [state = std::make_shared<RadialBoundingVolumeInspector>()](InspectorContext& context,
                                                                  RadialBoundingVolume& target) {
        return state->Inspect(context, target);
      },
      "RadialBoundingVolume");
  registered &= registrar.RegisterInspector<CubeVolume>(
      [state = std::make_shared<CubeVolumeInspector>()](InspectorContext& context, CubeVolume& target) {
        return state->Inspect(context, target);
      },
      "CubeVolume");
  registered &= registrar.RegisterInspector<ForestPatch>(
      [state = std::make_shared<ForestPatchInspector>()](InspectorContext& context, ForestPatch& target) {
        return state->Inspect(context, target);
      },
      "ForestPatch");
  registered &= registrar.RegisterInspector<BasicBarkDescriptor>(InspectBasicBarkDescriptor, "BasicBarkDescriptor");
  registered &= registrar.RegisterInspector<ForestDescriptor>(
      [state = std::make_shared<ForestDescriptorInspector>()](InspectorContext& context, ForestDescriptor& target) {
        return state->Inspect(context, target);
      },
      "ForestDescriptor");
  registered &= registrar.RegisterInspector<TreeDescriptor>(InspectTreeDescriptor, "TreeDescriptor");
  registered &=
      registrar.RegisterInspector<BasicPruningDescriptor>(InspectBasicPruningDescriptor, "BasicPruningDescriptor");
  registered &= registrar.RegisterInspector<BasicShootDescriptor>(
      [state = std::make_shared<GrowthDescriptorInspectorState>()](InspectorContext& context,
                                                                   BasicShootDescriptor& target) {
        return InspectBasicShootDescriptor(context, target, *state);
      },
      "BasicShootDescriptor");
  registered &= registrar.RegisterInspector<BasicRootDescriptor>(
      [state = std::make_shared<GrowthDescriptorInspectorState>()](InspectorContext& context,
                                                                   BasicRootDescriptor& target) {
        return InspectBasicRootDescriptor(context, target, *state);
      },
      "BasicRootDescriptor");
  registered &=
      registrar.RegisterInspector<BasicFineRootDescriptor>(InspectBasicFineRootDescriptor, "BasicFineRootDescriptor");
  registered &= registrar.RegisterInspector<BasicReproductionModuleDescriptor>(InspectBasicReproductionModuleDescriptor,
                                                                               "BasicReproductionModuleDescriptor");
  registered &=
      registrar.RegisterInspector<BasicFoliageDescriptor>(InspectBasicFoliageDescriptor, "BasicFoliageDescriptor");
  registered &=
      registrar.RegisterInspector<AdvancedShootDescriptor>(InspectAdvancedShootDescriptor, "AdvancedShootDescriptor");
  registered &= registrar.RegisterInspector<ModulusGraph>(
      [state = std::make_shared<ModulusGraphInspectorState>()](InspectorContext& context, ModulusGraph& target) {
        return InspectModulusGraph(context, target, *state);
      },
      "ModulusGraph");
  registered &= registrar.RegisterInspector<StrengthGraph>(
      [state = std::make_shared<StrengthGraphInspectorState>()](InspectorContext& context, StrengthGraph& target) {
        return InspectStrengthGraph(context, target, *state);
      },
      "StrengthGraph");
  registered &= registrar.RegisterInspector<BiologicalPropertiesGraph>(
      [state = std::make_shared<BiologicalPropertiesGraphInspectorState>()](InspectorContext& context,
                                                                            BiologicalPropertiesGraph& target) {
        return InspectBiologicalPropertiesGraph(context, target, *state);
      },
      "TrunkGraph");
  registered &= registrar.RegisterInspector<HeightField>(
      [state = std::make_shared<HeightFieldInspector>()](InspectorContext& context, HeightField& target) {
        return state->Inspect(context, target);
      },
      "HeightField");
  registered &= registrar.RegisterInspector<SoilLayerDescriptor>(
      [state = std::make_shared<SoilLayerDescriptorInspector>()](InspectorContext& context,
                                                                 SoilLayerDescriptor& target) {
        return state->Inspect(context, target);
      },
      "SoilLayerDescriptor");
  registered &= registrar.RegisterInspector<SoilDescriptor>(
      [state = std::make_shared<SoilDescriptorInspector>()](InspectorContext& context, SoilDescriptor& target) {
        return state->Inspect(context, target);
      },
      "SoilDescriptor");
  registered &= registrar.RegisterInspector<EcoSysLabLayer>(
      [](InspectorContext& context, EcoSysLabLayer&) {
        if (const auto layer = ApplicationContext::Get().GetLayer<EcoSysLabEditorLayer>())
          layer->DrawGui(context.editor_layer);
        return false;
      },
      "EcoSysLab Layer");
  return registered;
}
}  // namespace

EVOENGINE_PACKAGE_EXPORT const EditorPackageDescriptor* EvoEngineEditorPackageGetDescriptor() {
  static const EditorPackageDescriptor descriptor{
      EVOENGINE_EDITOR_PACKAGE_API_VERSION, "EcoSysLab",
      EVOENGINE_PACKAGE_SOURCE_ID,          EVOENGINE_PACKAGE_BUILD_IDENTITY,
      EVOENGINE_PACKAGE_RUNTIME_DESCRIPTOR, EVOENGINE_EDITOR_SOURCE_ID,
      EVOENGINE_EDITOR_PACKAGE_SOURCE_ID};
  return &descriptor;
}
EVOENGINE_PACKAGE_EXPORT bool EvoEngineEditorPackageLoad(EditorPackageRegistrar* registrar) {
  return registrar && RegisterEcoSysLabInspectors(*registrar) && RegisterEcoSysLabAssetPreviews(*registrar);
}
EVOENGINE_PACKAGE_EXPORT void EvoEngineEditorPackageUnload() {
}
