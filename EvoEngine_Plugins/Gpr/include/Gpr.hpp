#pragma once

#include "gpr.h"
using namespace evo_engine;
namespace gpr_plugin {
class Gpr : public IAsset {
  gpr_allocator allocator_;
  gpr_parameters params_;
  gpr_buffer input_buffer_;

  gpr_rgb_buffer rgb_buffer_;
  AssetRef preview_image_;

 protected:
  bool SaveInternal(const std::filesystem::path& path) const override;
  bool LoadInternal(const std::filesystem::path& path) override;

 public:
  ~Gpr() override;
  Gpr();
  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) override;
};
}  // namespace gpr_plugin
