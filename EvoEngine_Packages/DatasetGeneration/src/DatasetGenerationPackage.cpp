#include "PackageManager.hpp"

#include "SorghumPointCloudScanner.hpp"
#include "TreePointCloudScanner.hpp"

using namespace dataset_generation_package;
using namespace evo_engine;

namespace {
PackageDescriptor descriptor{EVOENGINE_PACKAGE_API_VERSION, "DatasetGeneration", "0.1.0",
                             "Dataset generation runtime package."};
}

EVOENGINE_PACKAGE_EXPORT const PackageDescriptor* EvoEnginePackageGetDescriptor() {
  return &descriptor;
}

EVOENGINE_PACKAGE_EXPORT bool EvoEnginePackageRegisterTypes(PackageRegistrar* registrar) {
  if (!registrar) {
    return false;
  }

  return registrar->RegisterPrivateComponent<TreePointCloudScanner>("TreePointCloudScanner") &&
         registrar->RegisterPrivateComponent<SorghumPointCloudScanner>("SorghumPointCloudScanner");
}

EVOENGINE_PACKAGE_EXPORT bool EvoEnginePackageLoad(PackageRegistrar*) {
  return true;
}

EVOENGINE_PACKAGE_EXPORT void EvoEnginePackageUnload(PackageRegistrar*) {
}
