#include "PackageManager.hpp"

#include "LogGrader.hpp"

using namespace evo_engine;
using namespace log_grading_package;

namespace {
PackageDescriptor descriptor{EVOENGINE_PACKAGE_API_VERSION, "LogGrading", "0.1.0",
                             "Forestry log grading runtime package."};
}  // namespace

EVOENGINE_PACKAGE_EXPORT const PackageDescriptor* EvoEnginePackageGetDescriptor() {
  return &descriptor;
}

EVOENGINE_PACKAGE_EXPORT bool EvoEnginePackageRegisterTypes(PackageRegistrar* registrar) {
  return registrar && registrar->RegisterPrivateComponent<LogGrader>("LogGrader");
}

EVOENGINE_PACKAGE_EXPORT bool EvoEnginePackageLoad(PackageRegistrar*) {
  return true;
}

EVOENGINE_PACKAGE_EXPORT void EvoEnginePackageUnload(PackageRegistrar*) {
}
