#include "PackageManager.hpp"

#ifdef CUDA_MODULE_SERVICE
#  include "CBTFGroup.hpp"
#  include "PARSensorGroup.hpp"
#endif
#include "DigitalAgricultureInspectionAdapters.hpp"
#include "DigitalAgricultureSerializationAdapters.hpp"
#include "InspectorRegistry.hpp"
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
PackageDescriptor descriptor{EVOENGINE_PACKAGE_API_VERSION, "DigitalAgriculture", "0.1.0",
                             "Digital agriculture runtime package."};

template <typename T>
void RegisterAssetPreviewHandler(const std::string& owner_name, const std::string& type_name) {
  Serialization::RegisterAssetPreviewHandler<T>(
      [](const std::shared_ptr<T>& asset, const OffscreenPreviewSettings&) {
        return asset ? asset->GenerateThumbnailTexture() : nullptr;
      },
      owner_name, type_name);
}

void RegisterDigitalAgricultureAssetPreviewHandlers(const std::string& owner_name) {
  RegisterAssetPreviewHandler<SorghumDescriptor>(owner_name, "SorghumDescriptor");
  RegisterAssetPreviewHandler<SorghumGrowthStages>(owner_name, "SorghumGrowthStages");
  RegisterAssetPreviewHandler<SorghumState>(owner_name, "SorghumState");
  RegisterAssetPreviewHandler<SorghumGenerator>(owner_name, "SorghumGenerator");
  RegisterAssetPreviewHandler<SorghumField>(owner_name, "SorghumField");
}

void RegisterDigitalAgricultureSerializationHandlers(const std::string& owner_name) {
  Serialization::RegisterSerializationHandler<SorghumDescriptor>(
      SerializeSorghumDescriptor, DeserializeSorghumDescriptor, owner_name, "SorghumDescriptor");
  Serialization::RegisterSerializationHandler<Sorghum>(SerializeSorghum, DeserializeSorghum, owner_name, "Sorghum");
  Serialization::RegisterSerializationHandler<SorghumGrowthStages>(
      SerializeSorghumGrowthStages, DeserializeSorghumGrowthStages, owner_name, "SorghumGrowthStages");
  Serialization::RegisterSerializationHandler<SorghumState>(SerializeSorghumState, DeserializeSorghumState, owner_name,
                                                            "SorghumState");
  Serialization::RegisterSerializationHandler<SorghumGenerator>(SerializeSorghumGenerator, DeserializeSorghumGenerator,
                                                                owner_name, "SorghumGenerator");
  Serialization::RegisterSerializationHandler<SorghumField>(SerializeSorghumField, DeserializeSorghumField, owner_name,
                                                            "SorghumField");
#ifdef CUDA_MODULE_SERVICE
  Serialization::RegisterSerializationHandler<PARSensorGroup>(SerializePARSensorGroup, DeserializePARSensorGroup,
                                                              owner_name, "PARSensorGroup");
  Serialization::RegisterSerializationHandler<CBTFGroup>(SerializeCBTFGroup, DeserializeCBTFGroup, owner_name,
                                                         "CBTFGroup");
#endif
  Serialization::RegisterSerializationHandler<SkyIlluminance>(SerializeSkyIlluminance, DeserializeSkyIlluminance,
                                                              owner_name, "SkyIlluminance");
  Serialization::RegisterSerializationHandler<SorghumCoordinates>(
      SerializeSorghumCoordinates, DeserializeSorghumCoordinates, owner_name, "SorghumCoordinates");
}

void RegisterDigitalAgricultureInspectors(const std::string& owner_name) {
  InspectorRegistry::GetInstance().RegisterInspector<SorghumDescriptor>(InspectSorghumDescriptor, owner_name,
                                                                        "SorghumDescriptor");
  InspectorRegistry::GetInstance().RegisterInspector<Sorghum>(InspectSorghum, owner_name, "Sorghum");
  InspectorRegistry::GetInstance().RegisterInspector<SorghumGrowthStages>(InspectSorghumGrowthStages, owner_name,
                                                                          "SorghumGrowthStages");
  InspectorRegistry::GetInstance().RegisterInspector<SorghumState>(InspectSorghumState, owner_name, "SorghumState");
  InspectorRegistry::GetInstance().RegisterInspector<SorghumGenerator>(InspectSorghumGenerator, owner_name,
                                                                       "SorghumGenerator");
  InspectorRegistry::GetInstance().RegisterInspector<SorghumField>(InspectSorghumField, owner_name, "SorghumField");
#ifdef CUDA_MODULE_SERVICE
  InspectorRegistry::GetInstance().RegisterInspector<PARSensorGroup>(InspectPARSensorGroup, owner_name,
                                                                     "PARSensorGroup");
  InspectorRegistry::GetInstance().RegisterInspector<CBTFGroup>(InspectCBTFGroup, owner_name, "CBTFGroup");
#endif
  InspectorRegistry::GetInstance().RegisterInspector<SkyIlluminance>(InspectSkyIlluminance, owner_name,
                                                                     "SkyIlluminance");
  InspectorRegistry::GetInstance().RegisterInspector<SorghumCoordinates>(InspectSorghumCoordinates, owner_name,
                                                                         "SorghumCoordinates");
  InspectorRegistry::GetInstance().RegisterInspector<SorghumLayer>(InspectSorghumLayer, owner_name, "Sorghum Layer");
}
}  // namespace

EVOENGINE_PACKAGE_EXPORT const PackageDescriptor* EvoEnginePackageGetDescriptor() {
  return &descriptor;
}

EVOENGINE_PACKAGE_EXPORT bool EvoEnginePackageRegisterTypes(PackageRegistrar* registrar) {
  if (!registrar) {
    return false;
  }

  bool registered = registrar->RegisterAsset<SorghumDescriptor>("SorghumDescriptor", {".sorghum"}) &&
                    registrar->RegisterPrivateComponent<Sorghum>("Sorghum") &&
                    registrar->RegisterAsset<SorghumGrowthStages>("SorghumGrowthStages", {".sgs"}) &&
                    registrar->RegisterAsset<SorghumState>("SorghumState", {".ss"}) &&
                    registrar->RegisterAsset<SorghumGenerator>("SorghumGenerator", {".sg"}) &&
                    registrar->RegisterAsset<SorghumField>("SorghumField", {".sorghumfield"});
#ifdef CUDA_MODULE_SERVICE
  registered = registered && registrar->RegisterAsset<PARSensorGroup>("PARSensorGroup", {".parsensorgroup"}) &&
               registrar->RegisterAsset<CBTFGroup>("CBTFGroup", {".cbtfgroup"});
#endif
  registered = registered && registrar->RegisterAsset<SkyIlluminance>("SkyIlluminance", {".skyilluminance"}) &&
               registrar->RegisterAsset<SorghumCoordinates>("SorghumCoordinates", {".sorghumcoords"}) &&
               registrar->RegisterLayer<SorghumLayer>("Sorghum Layer");
  if (registered) {
    RegisterDigitalAgricultureSerializationHandlers(descriptor.name);
    RegisterDigitalAgricultureAssetPreviewHandlers(descriptor.name);
    RegisterDigitalAgricultureInspectors(descriptor.name);
  }
  return registered;
}

EVOENGINE_PACKAGE_EXPORT bool EvoEnginePackageLoad(PackageRegistrar*) {
  return true;
}

EVOENGINE_PACKAGE_EXPORT void EvoEnginePackageUnload(PackageRegistrar*) {
}
