#include "PackageManager.hpp"

#include "BillboardCloudsConverter.hpp"
#include "BillboardCloudsInspectionAdapters.hpp"
#include "InspectorRegistry.hpp"

using namespace evo_engine;
using namespace billboard_clouds_package;

namespace {
PackageDescriptor descriptor{EVOENGINE_PACKAGE_API_VERSION, "BillboardClouds", "0.1.0",
                             "Billboard cloud conversion runtime package."};

void RegisterBillboardCloudsInspectors(const std::string& owner_name) {
  InspectorRegistry::GetInstance().RegisterInspector<BillboardCloudsConverter>(InspectBillboardCloudsConverter,
                                                                               owner_name, "BillboardCloudsConverter");
}
}  // namespace

EVOENGINE_PACKAGE_EXPORT const PackageDescriptor* EvoEnginePackageGetDescriptor() {
  return &descriptor;
}

EVOENGINE_PACKAGE_EXPORT bool EvoEnginePackageRegisterTypes(PackageRegistrar* registrar) {
  const bool registered =
      registrar && registrar->RegisterPrivateComponent<BillboardCloudsConverter>("BillboardCloudsConverter");
  if (registered) {
    RegisterBillboardCloudsInspectors(descriptor.name);
  }
  return registered;
}

EVOENGINE_PACKAGE_EXPORT bool EvoEnginePackageLoad(PackageRegistrar*) {
  return true;
}

EVOENGINE_PACKAGE_EXPORT void EvoEnginePackageUnload(PackageRegistrar*) {
}
