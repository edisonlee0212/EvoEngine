#include "PackageManager.hpp"
#include "PlantReconstructionLayer.hpp"
#include "PlantReconstructionSubject.hpp"
#include "Serialization.hpp"

using namespace evo_engine;
using namespace realtime_plant_reconstructor;

namespace {
PackageDescriptor descriptor{EVOENGINE_PACKAGE_API_VERSION, "RealtimePlantReconstructor", "0.1.0",
                             "Realtime plant reconstruction runtime package."};
}  // namespace

EVOENGINE_PACKAGE_EXPORT const PackageDescriptor* EvoEnginePackageGetDescriptor() {
  return &descriptor;
}

EVOENGINE_PACKAGE_EXPORT bool EvoEnginePackageRegisterTypes(PackageRegistrar* registrar) {
  if (!registrar) {
    return false;
  }
  const auto registered = registrar->RegisterPrivateComponent<PlantReconstructionSubject>("PlantReconstructionSubject") &&
                          registrar->RegisterLayer<PlantReconstructionLayer>("Plant Reconstruction Layer");
  if (registered) {
    Serialization::RegisterSerializationHandler<PlantReconstructionSubject>(
        SerializePlantReconstructionSubject, DeserializePlantReconstructionSubject, descriptor.name,
        "PlantReconstructionSubject");
    InspectorRegistry::GetInstance().RegisterInspector<PlantReconstructionSubject>(
        InspectPlantReconstructionSubject, descriptor.name, "PlantReconstructionSubject");
    InspectorRegistry::GetInstance().RegisterInspector<PlantReconstructionLayer>(
        [](InspectorContext& context, PlantReconstructionLayer& target) {
          return target.DrawGui(context.editor_layer);
        },
        descriptor.name, "Plant Reconstruction Layer");
  }
  return registered;
}

EVOENGINE_PACKAGE_EXPORT bool EvoEnginePackageLoad(PackageRegistrar*) {
  return true;
}

EVOENGINE_PACKAGE_EXPORT void EvoEnginePackageUnload(PackageRegistrar*) {
}
