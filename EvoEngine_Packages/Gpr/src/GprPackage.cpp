#include "PackageManager.hpp"

#include "Gpr.hpp"

using namespace evo_engine;
using namespace gpr_package;

namespace {
PackageDescriptor descriptor{EVOENGINE_PACKAGE_API_VERSION, "Gpr", "0.1.0", "GoPro Raw asset runtime package."};
}  // namespace

EVOENGINE_PACKAGE_EXPORT const PackageDescriptor* EvoEnginePackageGetDescriptor() {
  return &descriptor;
}

EVOENGINE_PACKAGE_EXPORT bool EvoEnginePackageRegisterTypes(PackageRegistrar* registrar) {
  return registrar && registrar->RegisterAsset<Gpr>("Gpr", {".evegpr", ".gpr", ".GPR"});
}

EVOENGINE_PACKAGE_EXPORT bool EvoEnginePackageLoad(PackageRegistrar*) {
  return true;
}

EVOENGINE_PACKAGE_EXPORT void EvoEnginePackageUnload(PackageRegistrar*) {
}
