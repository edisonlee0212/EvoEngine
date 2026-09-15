#include "AssetPreviewRegistry.hpp"
#include "BtfMaterial.hpp"
#include "BtfMeshRenderer.hpp"
#include "CBTFGroup.hpp"
#include "CBTFImporter.hpp"
#include "DigitalAgricultureInspectionAdapters.hpp"
#include "DigitalAgricultureInspectorStates.hpp"
#include "DigitalAgricultureSerializationAdapters.hpp"
#include "EditorPackage.hpp"
#include "InspectorRegistry.hpp"
#include "PARSensorGroup.hpp"
#include "PackageManager.hpp"
#include "Serialization.hpp"
#include "SkyIlluminance.hpp"
#include "Sorghum.hpp"
#include "SorghumCoordinates.hpp"
#include "SorghumDescriptor.hpp"
#include "SorghumField.hpp"
#include "SorghumGenerator.hpp"
#include "SorghumGrowthStages.hpp"
#include "SorghumLayer.hpp"
#include "SorghumState.hpp"
using namespace digital_agriculture_package;
using namespace evo_engine;
namespace {
bool RegisterInspectors(EditorPackageRegistrar& registrar) {
  bool registered = true;
  registered &= registrar.RegisterInspector<SorghumDescriptor>(InspectSorghumDescriptor, "SorghumDescriptor");
  registered &= registrar.RegisterInspector<Sorghum>(
      [state = std::make_shared<SorghumInspector>()](InspectorContext& context, Sorghum& target) {
        return state->Inspect(context, target);
      },
      "Sorghum");
  registered &= registrar.RegisterInspector<SorghumGrowthStages>(
      [state = std::make_shared<SorghumGrowthStagesInspector>()](InspectorContext& context,
                                                                 SorghumGrowthStages& target) {
        return state->Inspect(context, target);
      },
      "SorghumGrowthStages");
  registered &= registrar.RegisterInspector<SorghumState>(
      [state = std::make_shared<SorghumStateInspector>()](InspectorContext& context, SorghumState& target) {
        return state->Inspect(context, target);
      },
      "SorghumState");
  registered &= registrar.RegisterInspector<SorghumGenerator>(
      [state = std::make_shared<SorghumGeneratorInspector>()](InspectorContext& context, SorghumGenerator& target) {
        return state->Inspect(context, target);
      },
      "SorghumGenerator");
  registered &= registrar.RegisterInspector<SorghumField>(
      [state = std::make_shared<SorghumFieldInspector>()](InspectorContext& context, SorghumField& target) {
        return state->Inspect(context, target);
      },
      "SorghumField");
  registered &= registrar.RegisterInspector<PARSensorGroup>(
      [state = std::make_shared<PARSensorGroupInspector>()](InspectorContext& context, PARSensorGroup& target) {
        return state->Inspect(context, target);
      },
      "PARSensorGroup");
  registered &= registrar.RegisterInspector<CBTFGroup>(
      [state = std::make_shared<CBTFGroupInspector>()](InspectorContext& context, CBTFGroup& target) {
        return state->Inspect(context, target);
      },
      "CBTFGroup");
  registered &= registrar.RegisterInspector<CBTFImporter>(InspectCBTFImporter, "CBTFImporter");
  registered &= registrar.RegisterInspector<BtfMeshRenderer>(
      [](InspectorContext& context, BtfMeshRenderer& renderer) {
        return InspectBtfMeshRenderer(context, renderer);
      },
      "BtfMeshRenderer");
  registered &= registrar.RegisterInspector<BtfMaterial>(
      [](InspectorContext& context, BtfMaterial& material) {
        return InspectBtfMaterial(context, material);
      },
      "BtfMaterial");
  registered &= registrar.RegisterInspector<SkyIlluminance>(
      [state = std::make_shared<SkyIlluminanceInspector>()](InspectorContext& context, SkyIlluminance& target) {
        return state->Inspect(context, target);
      },
      "SkyIlluminance");
  registered &= registrar.RegisterInspector<SorghumCoordinates>(InspectSorghumCoordinates, "SorghumCoordinates");
  registered &= registrar.RegisterInspector<SorghumLayer>(InspectSorghumLayer, "Sorghum Layer");
  return registered;
}

template <typename T>
bool RegisterIcon(EditorPackageRegistrar& registrar, const char* name, const char* icon) {
  return registrar.RegisterAssetPreviewHandler<T>(
      [thumbnail = std::shared_ptr<Texture2D>{},
       path = std::filesystem::path("DigitalAgricultureResources/Icons") / icon](
          const std::shared_ptr<T>&, const OffscreenPreviewSettings&) mutable {
        if (!thumbnail) {
          thumbnail = AssetManager::CreateTemporaryAsset<Texture2D>();
          thumbnail->Import(std::filesystem::absolute(path));
        }
        return thumbnail;
      },
      name);
}

}  // namespace

EVOENGINE_PACKAGE_EXPORT const EditorPackageDescriptor* EvoEngineEditorPackageGetDescriptor() {
  static const EditorPackageDescriptor descriptor{
      EVOENGINE_EDITOR_PACKAGE_API_VERSION, "DigitalAgriculture",
      EVOENGINE_PACKAGE_SOURCE_ID,          EVOENGINE_PACKAGE_BUILD_IDENTITY,
      EVOENGINE_PACKAGE_RUNTIME_DESCRIPTOR, EVOENGINE_EDITOR_SOURCE_ID,
      EVOENGINE_EDITOR_PACKAGE_SOURCE_ID};
  return &descriptor;
}
EVOENGINE_PACKAGE_EXPORT bool EvoEngineEditorPackageLoad(EditorPackageRegistrar* registrar) {
  if (!registrar || !RegisterInspectors(*registrar))
    return false;
  return RegisterIcon<SorghumDescriptor>(*registrar, "SorghumDescriptor", "SorghumDescriptor.png") &&
         RegisterIcon<SorghumField>(*registrar, "SorghumField", "SorghumField.png") &&
         RegisterIcon<SorghumGenerator>(*registrar, "SorghumGenerator", "SorghumGenerator.png") &&
         RegisterIcon<SorghumGrowthStages>(*registrar, "SorghumGrowthStages", "SorghumGrowthStages.png") &&
         RegisterIcon<SorghumState>(*registrar, "SorghumState", "SorghumDescriptor.png");
}
EVOENGINE_PACKAGE_EXPORT void EvoEngineEditorPackageUnload() {
}
