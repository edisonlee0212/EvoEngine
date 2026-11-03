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
  if (ImGui::Button("Instantiate"))
    editor_layer->SetSelectedEntity(target.Instantiate());
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
  if (ImGui::CollapsingHeader("Default physics properties")) {
    auto& p = target.physics_properties;
    changed |= ImGui::DragFloat("Stretch stiffness", &p.stretch_stiffness, 10, 0, 1000000);
    changed |= ImGui::DragFloat("Bend stiffness", &p.bend_stiffness, 1, 0, 100000);
    changed |= ImGui::DragFloat("Wood density", &p.density, 1, 0.001f, 2000);
    changed |= ImGui::DragFloat("Damping", &p.damping, 0.001f, 0, 10);
    changed |= ImGui::DragFloat("Wind drag", &p.wind_drag, 0.1f, 0, 100);
    ImGui::TextDisabled("Copied into new trees; existing trees retain their own properties.");
  }
  return changed;
}
