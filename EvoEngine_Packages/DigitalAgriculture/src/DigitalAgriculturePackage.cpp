#include "PackageManager.hpp"

#ifdef CUDA_MODULE_SERVICE
#  include "CBTFGroup.hpp"
#  include "PARSensorGroup.hpp"
#endif
#include "SkyIlluminance.hpp"
#include "Sorghum.hpp"
#include "SorghumCoordinates.hpp"
#include "SorghumDescriptor.hpp"
#include "SorghumField.hpp"
#include "SorghumGenerator.hpp"
#include "SorghumGrowthStages.hpp"
#include "SorghumLayer.hpp"
#include "SorghumState.hpp"

using namespace digital_agriculture_package;
using namespace evo_engine;

namespace {
PackageDescriptor descriptor{EVOENGINE_PACKAGE_API_VERSION, "DigitalAgriculture", "0.1.0",
                             "Digital agriculture runtime package."};
}

EVOENGINE_PACKAGE_EXPORT const PackageDescriptor* EvoEnginePackageGetDescriptor() {
  return &descriptor;
}

EVOENGINE_PACKAGE_EXPORT bool EvoEnginePackageRegisterTypes(PackageRegistrar* registrar) {
  if (!registrar) {
    return false;
  }

  bool registered = registrar->RegisterAsset<SorghumDescriptor>("SorghumDescriptor", {".sorghum"}) &&
                    registrar->RegisterPrivateComponent<Sorghum>("Sorghum") &&
                    registrar->RegisterAsset<SorghumGrowthStages>("SorghumGrowthStages", {".sgs"}) &&
                    registrar->RegisterAsset<SorghumState>("SorghumState", {".ss"}) &&
                    registrar->RegisterAsset<SorghumGenerator>("SorghumGenerator", {".sg"}) &&
                    registrar->RegisterAsset<SorghumField>("SorghumField", {".sorghumfield"});
#ifdef CUDA_MODULE_SERVICE
  registered = registered && registrar->RegisterAsset<PARSensorGroup>("PARSensorGroup", {".parsensorgroup"}) &&
               registrar->RegisterAsset<CBTFGroup>("CBTFGroup", {".cbtfgroup"});
#endif
  return registered && registrar->RegisterAsset<SkyIlluminance>("SkyIlluminance", {".skyilluminance"}) &&
         registrar->RegisterAsset<SorghumCoordinates>("SorghumCoordinates", {".sorghumcoords"}) &&
         registrar->RegisterLayer<SorghumLayer>("Sorghum Layer");
}

EVOENGINE_PACKAGE_EXPORT bool EvoEnginePackageLoad(PackageRegistrar*) {
  return true;
}

EVOENGINE_PACKAGE_EXPORT void EvoEnginePackageUnload(PackageRegistrar*) {
}
