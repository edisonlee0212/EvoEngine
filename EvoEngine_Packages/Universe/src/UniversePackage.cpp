#include "PackageManager.hpp"

#include "InspectorRegistry.hpp"
#include "Serialization.hpp"
#include "StarCluster.hpp"
#include "UniverseInspectionAdapters.hpp"
#include "UniverseLayer.hpp"
#include "UniverseProfiler.hpp"
#include "UniverseSerializationAdapters.hpp"

using namespace evo_engine;
using namespace universe_package;

namespace {
PackageDescriptor descriptor{EVOENGINE_PACKAGE_API_VERSION, "Universe", "0.1.0",
                             "Universe simulation runtime package."};

void RegisterUniverseInspectors(const std::string& owner_name) {
  InspectorRegistry::GetInstance().RegisterInspector<PlanetTerrain>(InspectPlanetTerrain, owner_name, "PlanetTerrain");
  InspectorRegistry::GetInstance().RegisterInspector<StarCluster>(InspectStarCluster, owner_name, "Star Cluster");
  InspectorRegistry::GetInstance().RegisterInspector<UniverseLayer>(InspectUniverseLayer, owner_name, "Universe Layer");
}

void RegisterUniverseSerializationHandlers(const std::string& owner_name) {
  Serialization::RegisterSerializationHandler<PlanetTerrain>(SerializePlanetTerrain, DeserializePlanetTerrain,
                                                             owner_name, "PlanetTerrain");
  Serialization::RegisterSerializationHandler<StarCluster>(SerializeStarCluster, DeserializeStarCluster, owner_name,
                                                           "Star Cluster");
}
}  // namespace

EVOENGINE_PACKAGE_EXPORT const PackageDescriptor* EvoEnginePackageGetDescriptor() {
  return &descriptor;
}

EVOENGINE_PACKAGE_EXPORT bool EvoEnginePackageRegisterTypes(PackageRegistrar* registrar) {
  if (!registrar) {
    return false;
  }

  const bool registered = registrar->RegisterPrivateComponent<StarCluster>("Star Cluster") &&
                          registrar->RegisterPrivateComponent<PlanetTerrain>("PlanetTerrain") &&
                          registrar->RegisterLayer<UniverseLayer>("Universe Layer");
  if (registered) {
    RegisterUniverseSerializationHandlers(descriptor.name);
    RegisterUniverseInspectors(descriptor.name);
  }
  return registered;
}

EVOENGINE_PACKAGE_EXPORT bool EvoEnginePackageLoad(PackageRegistrar* registrar) {
  return registrar && universe_profiler::RegisterItems(*registrar);
}

EVOENGINE_PACKAGE_EXPORT void EvoEnginePackageUnload(PackageRegistrar*) {
}
