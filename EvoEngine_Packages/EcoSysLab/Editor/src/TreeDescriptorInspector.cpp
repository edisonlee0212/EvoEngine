#include "Application.hpp"
#include "AssetManager.hpp"
#include "BasicBarkDescriptor.hpp"
#include "BasicFoliageDescriptor.hpp"
#include "BasicReproductionModuleDescriptor.hpp"
#include "BasicRootDescriptor.hpp"
#include "BasicShootDescriptor.hpp"
#include "Climate.hpp"
#include "DynamicTreeSkeleton.hpp"
#include "EcoSysLabDescriptorInspectors.hpp"
#include "EcoSysLabLayer.hpp"
#include "EcoSysLabSerializationAdapters.hpp"
#include "EditorLayer.hpp"
#include "HeightField.hpp"
#include "Material.hpp"
#include "Octree.hpp"
#include "Soil.hpp"
#include "Strands.hpp"
#include "StrandsRenderer.hpp"
#include "Tree.hpp"
#include "TreeDescriptor.hpp"
#include "TreeMeshGenerator.hpp"

using namespace eco_sys_lab_package;

bool eco_sys_lab_package::InspectTreeDescriptor(InspectorContext& context, TreeDescriptor& target) {
  const auto& editor_layer = context.editor_layer;
  bool changed = false;
  const auto eco_sys_lab_layer = ApplicationContext::Get().GetLayer<EcoSysLabLayer>();
  std::shared_ptr<Climate> climate;
  std::shared_ptr<Soil> soil;
  if (const auto climate_candidate = EcoSysLabLayer::FindClimate(); !climate_candidate.expired())
    climate = climate_candidate.lock();
  if (const auto soil_candidate = EcoSysLabLayer::FindSoil(); !soil_candidate.expired())
    soil = soil_candidate.lock();
  if (soil && climate) {
    if (ImGui::Button("Instantiate")) {
      editor_layer->SetSelectedEntity(target.Instantiate());
    }
  } else {
    ImGui::Text("Create soil and climate entity to instantiate!");
  }
  if (editor_layer->DragAndDropButton<IShootDescriptor>(target.shoot_descriptor, "Shoot Descriptor"))
    changed = true;
  if (editor_layer->DragAndDropButton<IRootDescriptor>(target.root_descriptor, "Root Descriptor"))
    changed = true;
  if (editor_layer->DragAndDropButton<IPruningDescriptor>(target.pruning_descriptor, "Pruning Descriptor"))
    changed = true;
  if (editor_layer->DragAndDropButton<IFoliageDescriptor>(target.foliage_descriptor, "Foliage Descriptor"))
    changed = true;
  if (editor_layer->DragAndDropButton<IReproductionModuleDescriptor>(target.reproduction_module_descriptor,
                                                                     "Reproduction Descriptor"))
    changed = true;

  if (editor_layer->DragAndDropButton<IBarkDescriptor>(target.bark_descriptor, "Bark Descriptor"))
    changed = true;
  return changed;
}
