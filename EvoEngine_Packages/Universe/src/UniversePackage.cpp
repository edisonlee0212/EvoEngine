#include "PackageManager.hpp"

#include "InspectorRegistry.hpp"
#include "Serialization.hpp"
#include "UniverseInspectionAdapters.hpp"
#include "UniverseLayer.hpp"
#include "UniverseSerializationAdapters.hpp"

using namespace evo_engine;
using namespace universe_package;

namespace {
PackageDescriptor descriptor{EVOENGINE_PACKAGE_API_VERSION, "Universe", "0.1.0",
                             "Universe simulation runtime package."};

void RegisterUniverseInspectors(const std::string& owner_name) {
  InspectorRegistry::GetInstance().RegisterInspector<PlanetTerrain>(InspectPlanetTerrain, owner_name, "PlanetTerrain");
  InspectorRegistry::GetInstance().RegisterInspector<UniverseLayer>(InspectUniverseLayer, owner_name, "Universe Layer");
}

void RegisterUniverseSerializationHandlers(const std::string& owner_name) {
  Serialization::RegisterSerializationHandler<PlanetTerrain>(SerializePlanetTerrain, DeserializePlanetTerrain,
                                                             owner_name, "PlanetTerrain");
}
}  // namespace

EVOENGINE_PACKAGE_EXPORT const PackageDescriptor* EvoEnginePackageGetDescriptor() {
  return &descriptor;
}

EVOENGINE_PACKAGE_EXPORT bool EvoEnginePackageRegisterTypes(PackageRegistrar* registrar) {
  if (!registrar) {
    return false;
  }

  const bool registered = registrar->RegisterDataComponent<StarPosition>("StarPosition") &&
                          registrar->RegisterDataComponent<SelectionStatus>("SelectionStatus") &&
                          registrar->RegisterDataComponent<StarInfo>("StarInfo") &&
                          registrar->RegisterDataComponent<SurfaceColor>("SurfaceColor") &&
                          registrar->RegisterDataComponent<DisplayColor>("DisplayColor") &&
                          registrar->RegisterDataComponent<OriginalColor>("OriginalColor") &&
                          registrar->RegisterDataComponent<StarOrbitOffset>("StarOrbitOffset") &&
                          registrar->RegisterDataComponent<StarOrbitProportion>("StarOrbitProportion") &&
                          registrar->RegisterDataComponent<StarOrbit>("StarOrbit") &&
                          registrar->RegisterDataComponent<StarClusterIndex>("StarClusterIndex") &&
                          registrar->RegisterPrivateComponent<PlanetTerrain>("PlanetTerrain") &&
                          registrar->RegisterLayer<UniverseLayer>("Universe Layer");
  if (registered) {
    RegisterUniverseSerializationHandlers(descriptor.name);
    RegisterUniverseInspectors(descriptor.name);
  }
  return registered;
}

EVOENGINE_PACKAGE_EXPORT bool EvoEnginePackageLoad(PackageRegistrar*) {
  return true;
}

EVOENGINE_PACKAGE_EXPORT void EvoEnginePackageUnload(PackageRegistrar*) {
}
