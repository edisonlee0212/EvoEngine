#include "PackageManager.hpp"

#include "LSystemDescriptor.hpp"
#include "LSystemLayer.hpp"
#include "ScotsPine.hpp"
#include "ScotsPineDescriptor.hpp"

using namespace evo_engine;
using namespace l_system_package;

namespace {
PackageDescriptor descriptor{EVOENGINE_PACKAGE_API_VERSION, "LSystem", "0.1.0",
                             "L-system grammar runtime package (plant-agnostic core)."};
}  // namespace

EVOENGINE_PACKAGE_EXPORT const PackageDescriptor* EvoEnginePackageGetDescriptor() {
  return &descriptor;
}

EVOENGINE_PACKAGE_EXPORT bool EvoEnginePackageRegisterTypes(PackageRegistrar* registrar) {
  if (!registrar) {
    return false;
  }

  return registrar->RegisterAsset<LSystemDescriptor>("LSystemDescriptor", {".lsys"}) &&
         registrar->RegisterAsset<ScotsPineDescriptor>("ScotsPineDescriptor", {".spine"}) &&
         registrar->RegisterPrivateComponent<ScotsPine>("ScotsPine") &&
         registrar->RegisterLayer<LSystemLayer>("LSystem Layer");
}

EVOENGINE_PACKAGE_EXPORT bool EvoEnginePackageLoad(PackageRegistrar*) {
  return true;
}

EVOENGINE_PACKAGE_EXPORT void EvoEnginePackageUnload(PackageRegistrar*) {
}
