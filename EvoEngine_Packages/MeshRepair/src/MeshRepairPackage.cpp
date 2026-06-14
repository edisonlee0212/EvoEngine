#include "PackageManager.hpp"

#include "InspectorRegistry.hpp"
#include "MeshColoring.hpp"
#include "MeshRepairInspectionAdapters.hpp"

using namespace evo_engine;
using namespace mesh_repair_package;

namespace {
PackageDescriptor descriptor{EVOENGINE_PACKAGE_API_VERSION, "MeshRepair", "0.1.0",
                             "Mesh coloring and repair runtime package."};

void RegisterMeshRepairInspectors(const std::string& owner_name) {
  InspectorRegistry::GetInstance().RegisterInspector<MeshColoring>(InspectMeshColoring, owner_name, "MeshColoring");
}
}  // namespace

EVOENGINE_PACKAGE_EXPORT const PackageDescriptor* EvoEnginePackageGetDescriptor() {
  return &descriptor;
}

EVOENGINE_PACKAGE_EXPORT bool EvoEnginePackageRegisterTypes(PackageRegistrar* registrar) {
  const bool registered = registrar && registrar->RegisterPrivateComponent<MeshColoring>("MeshColoring");
  if (registered) {
    RegisterMeshRepairInspectors(descriptor.name);
  }
  return registered;
}

EVOENGINE_PACKAGE_EXPORT bool EvoEnginePackageLoad(PackageRegistrar*) {
  return true;
}

EVOENGINE_PACKAGE_EXPORT void EvoEnginePackageUnload(PackageRegistrar*) {
}
