#include "EcoSysLabAssetPreviews.hpp"
#include "AdvancedShootDescriptor.hpp"
#include "AssetManager.hpp"
#include "BasicBarkDescriptor.hpp"
#include "BasicFineRootDescriptor.hpp"
#include "BasicPruningDescriptor.hpp"
#include "BasicReproductionModuleDescriptor.hpp"
#include "BasicRootDescriptor.hpp"
#include "BasicShootDescriptor.hpp"
#include "Climate.hpp"
#include "EditorPackage.hpp"
#include "ForestDescriptor.hpp"
#include "HeightField.hpp"
#include "SoilDescriptor.hpp"
#include "Texture2D.hpp"
#include "TreeDescriptor.hpp"

using namespace evo_engine;
using namespace eco_sys_lab_package;
namespace {
template <typename T>
bool RegisterIcon(EditorPackageRegistrar& registrar, const char* name, const char* icon) {
  return registrar.RegisterAssetPreviewHandler<T>(
      [thumbnail = std::shared_ptr<Texture2D>{}, path = std::filesystem::path("EcoSysLabResources") / icon](
          const std::shared_ptr<T>&, const OffscreenPreviewSettings&) mutable {
        if (!thumbnail) {
          thumbnail = AssetManager::CreateTemporaryAsset<Texture2D>();
          thumbnail->Import(std::filesystem::absolute(path));
        }
        return thumbnail;
      },
      name);
}
}  // namespace
bool eco_sys_lab_package::RegisterEcoSysLabAssetPreviews(EditorPackageRegistrar& registrar) {
  bool registered = true;
  registered &= RegisterIcon<ClimateDescriptor>(registrar, "ClimateDescriptor", "Icons/ClimateDescriptor.png");
  registered &= RegisterIcon<ForestPatch>(registrar, "ForestPatch", "Icons/ForestPatch.png");
  registered &= RegisterIcon<ForestDescriptor>(registrar, "ForestDescriptor", "Icons/ForestDescriptor.png");
  registered &= RegisterIcon<HeightField>(registrar, "HeightField", "Icons/HeightField.png");
  registered &= RegisterIcon<SoilDescriptor>(registrar, "SoilDescriptor", "Icons/SoilDescriptor.png");
  registered &= RegisterIcon<BasicShootDescriptor>(registrar, "BasicShootDescriptor", "Icons/ShootDescriptor.png");
  registered &=
      RegisterIcon<AdvancedShootDescriptor>(registrar, "AdvancedShootDescriptor", "Icons/ShootDescriptor.png");
  registered &= RegisterIcon<BasicRootDescriptor>(registrar, "BasicRootDescriptor", "Icons/RootDescriptor.png");
  registered &= RegisterIcon<BasicFineRootDescriptor>(registrar, "BasicFineRootDescriptor", "Icons/RootDescriptor.png");
  registered &=
      RegisterIcon<BasicPruningDescriptor>(registrar, "BasicPruningDescriptor", "Icons/PruningDescriptor.png");
  registered &=
      RegisterIcon<BasicFoliageDescriptor>(registrar, "BasicFoliageDescriptor", "Icons/FoliageDescriptor.png");
  registered &= RegisterIcon<BasicReproductionModuleDescriptor>(registrar, "BasicReproductionModuleDescriptor",
                                                                "Icons/FruitDescriptor.png");
  registered &= RegisterIcon<BasicBarkDescriptor>(registrar, "BasicBarkDescriptor", "Icons/BarkDescriptor.png");
  registered &= RegisterIcon<TreeDescriptor>(registrar, "TreeDescriptor", "Icons/TreeDescriptor.png");
  return registered;
}
