#include "PackageManager.hpp"

#include "BillboardCloudsConverter.hpp"

using namespace evo_engine;
using namespace billboard_clouds_package;

namespace {
PackageDescriptor descriptor{EVOENGINE_PACKAGE_API_VERSION,
                             "BillboardClouds",
                             "0.1.0",
                             "Billboard cloud conversion runtime package.",
                             EVOENGINE_PACKAGE_BUILD_IDENTITY,
                             EVOENGINE_PACKAGE_SOURCE_ID};

}  // namespace

EVOENGINE_PACKAGE_EXPORT const PackageDescriptor* EvoEnginePackageGetDescriptor() {
  return &descriptor;
}

EVOENGINE_PACKAGE_EXPORT bool EvoEnginePackageRegisterTypes(PackageRegistrar* registrar) {
  const bool registered =
      registrar && registrar->RegisterPrivateComponent<BillboardCloudsConverter>("BillboardCloudsConverter");
  return registered;
}

EVOENGINE_PACKAGE_EXPORT bool EvoEnginePackageLoad(PackageRegistrar*) {
  return true;
}

EVOENGINE_PACKAGE_EXPORT void EvoEnginePackageUnload(PackageRegistrar*) {
}
