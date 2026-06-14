#include "PackageManager.hpp"

#include "InspectorRegistry.hpp"
#include "JoeScanScanner.hpp"
#include "LogScanningInspectionAdapters.hpp"
#include "LogScanningSerializationAdapters.hpp"
#include "Serialization.hpp"

using namespace evo_engine;
using namespace log_scanning_package;

namespace {
PackageDescriptor descriptor{EVOENGINE_PACKAGE_API_VERSION, "LogScanning", "0.1.0",
                             "Forestry log scanning runtime package."};

void RegisterLogScanningInspectors(const std::string& owner_name) {
  InspectorRegistry::GetInstance().RegisterInspector<LogScan>(InspectLogScan, owner_name, "LogScan");
  InspectorRegistry::GetInstance().RegisterInspector<JoeScanScanner>(InspectJoeScanScanner, owner_name,
                                                                     "JoeScanScanner");
}

void RegisterLogScanningSerializationHandlers(const std::string& owner_name) {
  Serialization::RegisterSerializationHandler<LogScan>(SerializeLogScan, DeserializeLogScan, owner_name, "LogScan");
  Serialization::RegisterSerializationHandler<JoeScanScanner>(SerializeJoeScanScanner, DeserializeJoeScanScanner,
                                                              owner_name, "JoeScanScanner");
}
}  // namespace

EVOENGINE_PACKAGE_EXPORT const PackageDescriptor* EvoEnginePackageGetDescriptor() {
  return &descriptor;
}

EVOENGINE_PACKAGE_EXPORT bool EvoEnginePackageRegisterTypes(PackageRegistrar* registrar) {
  const bool registered = registrar && registrar->RegisterAsset<LogScan>("LogScan", {".jscan"}) &&
                          registrar->RegisterPrivateComponent<JoeScanScanner>("JoeScanScanner");
  if (registered) {
    RegisterLogScanningSerializationHandlers(descriptor.name);
    RegisterLogScanningInspectors(descriptor.name);
  }
  return registered;
}

EVOENGINE_PACKAGE_EXPORT bool EvoEnginePackageLoad(PackageRegistrar*) {
  return true;
}

EVOENGINE_PACKAGE_EXPORT void EvoEnginePackageUnload(PackageRegistrar*) {
}
