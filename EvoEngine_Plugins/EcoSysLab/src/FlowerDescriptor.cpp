#include "FlowerDescriptor.hpp"

using namespace eco_sys_lab_plugin;

std::shared_ptr<Texture2D> FlowerDescriptor::GenerateThumbnailTexture() {
  static std::shared_ptr<Texture2D> thumbnail;
  if (!thumbnail) {
    thumbnail = ProjectManager::CreateTemporaryAsset<Texture2D>();
    thumbnail->Import(
        std::filesystem::absolute(std::filesystem::path("./EcoSysLabResources") / "Icons/FlowerDescriptor.png"));
  }
  return thumbnail;
}
