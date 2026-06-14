#include "PackageManager.hpp"

#include "InspectorRegistry.hpp"
#include "Serialization.hpp"
#include "TextureBaking.hpp"
#include "TextureBakingInspectionAdapters.hpp"
#include "TextureBakingSerializationAdapters.hpp"

using namespace evo_engine;
using namespace texture_baking_package;

namespace {
PackageDescriptor descriptor{EVOENGINE_PACKAGE_API_VERSION, "TextureBaking", "0.1.0",
                             "Mesh texture baking runtime package."};

void RegisterTextureBakingInspectors(const std::string& owner_name) {
  InspectorRegistry::GetInstance().RegisterInspector<TextureBaking>(InspectTextureBaking, owner_name, "TextureBaking");
}

void RegisterTextureBakingSerializationHandlers(const std::string& owner_name) {
  Serialization::RegisterSerializationHandler<TextureBaking>(SerializeTextureBaking, DeserializeTextureBaking,
                                                             owner_name, "TextureBaking");
}
}  // namespace

EVOENGINE_PACKAGE_EXPORT const PackageDescriptor* EvoEnginePackageGetDescriptor() {
  return &descriptor;
}

EVOENGINE_PACKAGE_EXPORT bool EvoEnginePackageRegisterTypes(PackageRegistrar* registrar) {
  const bool registered = registrar && registrar->RegisterPrivateComponent<TextureBaking>("TextureBaking");
  if (registered) {
    RegisterTextureBakingSerializationHandlers(descriptor.name);
    RegisterTextureBakingInspectors(descriptor.name);
  }
  return registered;
}

EVOENGINE_PACKAGE_EXPORT bool EvoEnginePackageLoad(PackageRegistrar*) {
  return true;
}

EVOENGINE_PACKAGE_EXPORT void EvoEnginePackageUnload(PackageRegistrar*) {
}
