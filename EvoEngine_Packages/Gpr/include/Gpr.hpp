#pragma once

#include "gpr.h"

namespace gpr_package {
using namespace evo_engine;

class Gpr : public IAsset {
  gpr_allocator allocator_;
  gpr_parameters params_;
  gpr_buffer input_buffer_;

  gpr_rgb_buffer rgb_buffer_;
  AssetRef preview_image_;

 protected:
  bool SaveInternal(const std::filesystem::path& path) const;
  bool LoadInternal(const std::filesystem::path& path);

 public:
  ~Gpr() override;
  Gpr();
  bool SaveGpr(const std::filesystem::path& path) const;
  bool LoadGpr(const std::filesystem::path& path);
  AssetRef& RefPreviewImage();
};

void RegisterGprHandlers(const std::string& owner_name);
}  // namespace gpr_package
