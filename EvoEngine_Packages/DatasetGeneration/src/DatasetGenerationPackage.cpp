#include "PackageManager.hpp"

#include "DatasetGenerationInspectionAdapters.hpp"
#include "DatasetGenerationSerializationAdapters.hpp"
#include "InspectorRegistry.hpp"
#include "Serialization.hpp"
#include "SorghumPointCloudScanner.hpp"
#include "TreePointCloudScanner.hpp"

using namespace dataset_generation_package;
using namespace evo_engine;

namespace {
PackageDescriptor descriptor{EVOENGINE_PACKAGE_API_VERSION, "DatasetGeneration", "0.1.0",
                             "Dataset generation runtime package."};

void RegisterDatasetGenerationInspectors(const std::string& owner_name) {
  InspectorRegistry::GetInstance().RegisterInspector<TreePointCloudScanner>(InspectTreePointCloudScanner, owner_name,
                                                                            "TreePointCloudScanner");
  InspectorRegistry::GetInstance().RegisterInspector<SorghumPointCloudScanner>(InspectSorghumPointCloudScanner,
                                                                               owner_name, "SorghumPointCloudScanner");
}

void RegisterDatasetGenerationSerializationHandlers(const std::string& owner_name) {
  Serialization::RegisterSerializationHandler<TreePointCloudScanner>(
      SerializeTreePointCloudScanner, DeserializeTreePointCloudScanner, owner_name, "TreePointCloudScanner");
  Serialization::RegisterSerializationHandler<SorghumPointCloudScanner>(
      SerializeSorghumPointCloudScanner, DeserializeSorghumPointCloudScanner, owner_name, "SorghumPointCloudScanner");
}
}  // namespace

EVOENGINE_PACKAGE_EXPORT const PackageDescriptor* EvoEnginePackageGetDescriptor() {
  return &descriptor;
}

EVOENGINE_PACKAGE_EXPORT bool EvoEnginePackageRegisterTypes(PackageRegistrar* registrar) {
  if (!registrar) {
    return false;
  }

  const bool registered = registrar->RegisterPrivateComponent<TreePointCloudScanner>("TreePointCloudScanner") &&
                          registrar->RegisterPrivateComponent<SorghumPointCloudScanner>("SorghumPointCloudScanner");
  if (registered) {
    RegisterDatasetGenerationSerializationHandlers(descriptor.name);
    RegisterDatasetGenerationInspectors(descriptor.name);
  }
  return registered;
}

EVOENGINE_PACKAGE_EXPORT bool EvoEnginePackageLoad(PackageRegistrar*) {
  return true;
}

EVOENGINE_PACKAGE_EXPORT void EvoEnginePackageUnload(PackageRegistrar*) {
}
