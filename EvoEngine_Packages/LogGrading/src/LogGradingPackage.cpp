#include "PackageManager.hpp"

#include "InspectorRegistry.hpp"
#include "LogGrader.hpp"
#include "LogGradingInspectionAdapters.hpp"

using namespace evo_engine;
using namespace log_grading_package;

namespace {
PackageDescriptor descriptor{EVOENGINE_PACKAGE_API_VERSION, "LogGrading", "0.1.0",
                             "Forestry log grading runtime package."};

void RegisterLogGradingInspectors(const std::string& owner_name) {
  InspectorRegistry::GetInstance().RegisterInspector<LogGrader>(InspectLogGrader, owner_name, "LogGrader");
}
}  // namespace

EVOENGINE_PACKAGE_EXPORT const PackageDescriptor* EvoEnginePackageGetDescriptor() {
  return &descriptor;
}

EVOENGINE_PACKAGE_EXPORT bool EvoEnginePackageRegisterTypes(PackageRegistrar* registrar) {
  const bool registered = registrar && registrar->RegisterPrivateComponent<LogGrader>("LogGrader");
  if (registered) {
    RegisterLogGradingInspectors(descriptor.name);
  }
  return registered;
}

EVOENGINE_PACKAGE_EXPORT bool EvoEnginePackageLoad(PackageRegistrar*) {
  return true;
}

EVOENGINE_PACKAGE_EXPORT void EvoEnginePackageUnload(PackageRegistrar*) {
}
