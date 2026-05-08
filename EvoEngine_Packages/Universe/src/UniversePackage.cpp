#include "PackageManager.hpp"

#include "UniverseLayer.hpp"

using namespace evo_engine;
using namespace universe_plugin;

namespace {
PackageDescriptor descriptor{EVOENGINE_PACKAGE_API_VERSION, "Universe", "0.1.0",
                             "Universe simulation runtime package."};
}  // namespace

EVOENGINE_PACKAGE_EXPORT const PackageDescriptor* EvoEnginePackageGetDescriptor() {
  return &descriptor;
}

EVOENGINE_PACKAGE_EXPORT bool EvoEnginePackageRegisterTypes(PackageRegistrar* registrar) {
  if (!registrar) {
    return false;
  }

  return registrar->RegisterDataComponent<StarPosition>("StarPosition") &&
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
}

EVOENGINE_PACKAGE_EXPORT bool EvoEnginePackageLoad(PackageRegistrar*) {
  return true;
}

EVOENGINE_PACKAGE_EXPORT void EvoEnginePackageUnload(PackageRegistrar*) {
}
