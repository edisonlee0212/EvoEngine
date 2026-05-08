#include "PackageManager.hpp"

#include "JoeScanScanner.hpp"

using namespace evo_engine;
using namespace log_scanning_plugin;

namespace {
PackageDescriptor descriptor{EVOENGINE_PACKAGE_API_VERSION, "LogScanning", "0.1.0",
                             "Forestry log scanning runtime package."};
}  // namespace

EVOENGINE_PACKAGE_EXPORT const PackageDescriptor* EvoEnginePackageGetDescriptor() {
  return &descriptor;
}

EVOENGINE_PACKAGE_EXPORT bool EvoEnginePackageRegisterTypes(PackageRegistrar* registrar) {
  return registrar && registrar->RegisterAsset<LogScan>("LogScan", {".jscan"}) &&
         registrar->RegisterPrivateComponent<JoeScanScanner>("JoeScanScanner");
}

EVOENGINE_PACKAGE_EXPORT bool EvoEnginePackageLoad(PackageRegistrar*) {
  return true;
}

EVOENGINE_PACKAGE_EXPORT void EvoEnginePackageUnload(PackageRegistrar*) {
}
