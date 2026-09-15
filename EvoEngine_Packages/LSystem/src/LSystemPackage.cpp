#include "PackageManager.hpp"

#include "LSystemDescriptor.hpp"
#include "LSystemLayer.hpp"
#include "LSystemSerializationAdapters.hpp"
#include "ScotsPine.hpp"
#include "ScotsPineDescriptor.hpp"
#include "Serialization.hpp"

using namespace evo_engine;
using namespace l_system_package;

namespace {
PackageDescriptor descriptor{EVOENGINE_PACKAGE_API_VERSION,
                             "LSystem",
                             "0.1.0",
                             "L-system grammar runtime package (plant-agnostic core).",
                             EVOENGINE_PACKAGE_BUILD_IDENTITY,
                             EVOENGINE_PACKAGE_SOURCE_ID};

void RegisterLSystemSerializationHandlers(const std::string& owner_name) {
  Serialization::RegisterSerializationHandler<LSystemDescriptor>(
      SerializeLSystemDescriptor, DeserializeLSystemDescriptor, owner_name, "LSystemDescriptor");
  Serialization::RegisterSerializationHandler<ScotsPineDescriptor>(
      SerializeScotsPineDescriptor, DeserializeScotsPineDescriptor, owner_name, "ScotsPineDescriptor");
  Serialization::RegisterSerializationHandler<ScotsPine>(SerializeScotsPine, DeserializeScotsPine, owner_name,
                                                         "ScotsPine");
  Serialization::RegisterSerializationHandler<LSystemLayer>(SerializeLSystemLayer, DeserializeLSystemLayer, owner_name,
                                                            "LSystem Layer");
}
}  // namespace

EVOENGINE_PACKAGE_EXPORT const PackageDescriptor* EvoEnginePackageGetDescriptor() {
  return &descriptor;
}

EVOENGINE_PACKAGE_EXPORT bool EvoEnginePackageRegisterTypes(PackageRegistrar* registrar) {
  if (!registrar) {
    return false;
  }

  const bool registered = registrar->RegisterAsset<LSystemDescriptor>("LSystemDescriptor", {".lsys"}) &&
                          registrar->RegisterAsset<ScotsPineDescriptor>("ScotsPineDescriptor", {".spine"}) &&
                          registrar->RegisterPrivateComponent<ScotsPine>("ScotsPine") &&
                          registrar->RegisterLayer<LSystemLayer>("LSystem Layer");
  if (registered) {
    RegisterLSystemSerializationHandlers(descriptor.name);
  }
  return registered;
}

EVOENGINE_PACKAGE_EXPORT bool EvoEnginePackageLoad(PackageRegistrar*) {
  return true;
}

EVOENGINE_PACKAGE_EXPORT void EvoEnginePackageUnload(PackageRegistrar*) {
}
