#include "PackageManager.hpp"

#include "MeshColoring.hpp"

using namespace evo_engine;
using namespace mesh_repair_package;

namespace {
PackageDescriptor descriptor{EVOENGINE_PACKAGE_API_VERSION,
                             "MeshRepair",
                             "0.1.0",
                             "Mesh coloring and repair runtime package.",
                             EVOENGINE_PACKAGE_BUILD_IDENTITY,
                             EVOENGINE_PACKAGE_SOURCE_ID};

}  // namespace

EVOENGINE_PACKAGE_EXPORT const PackageDescriptor* EvoEnginePackageGetDescriptor() {
  return &descriptor;
}

EVOENGINE_PACKAGE_EXPORT bool EvoEnginePackageRegisterTypes(PackageRegistrar* registrar) {
  const bool registered = registrar && registrar->RegisterPrivateComponent<MeshColoring>("MeshColoring");
  return registered;
}

EVOENGINE_PACKAGE_EXPORT bool EvoEnginePackageLoad(PackageRegistrar*) {
  return true;
}

EVOENGINE_PACKAGE_EXPORT void EvoEnginePackageUnload(PackageRegistrar*) {
}
