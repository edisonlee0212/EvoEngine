#pragma once

using namespace evo_engine;

namespace eco_sys_lab_plugin {
class FlowerDescriptor : public IAsset {
 public:
  [[nodiscard]] std::shared_ptr<Texture2D> GenerateThumbnailTexture() override;
};
}  // namespace eco_sys_lab_plugin