#include "Gpr.hpp"

#include "InspectorRegistry.hpp"
#include "Serialization.hpp"

#include "dng_sdk/dng_exceptions.h"
#include "source/app/gpr_tools/gpr_print_utils.h"
#include "vc5_common/config.h"
#include "vc5_common/error.h"
#include "vc5_decoder/vc5_decoder.h"

using namespace evo_engine;
using namespace gpr_package;

namespace {
bool InspectGpr(InspectorContext& context, Gpr& asset) {
  ImGui::PushID(&asset);
  auto& preview_image = asset.RefPreviewImage();
  const auto preview_texture = preview_image.Get<Texture2D>();
  if (!preview_texture) {
    ImGui::TextDisabled("No GPR preview texture is available.");
    ImGui::PopID();
    return false;
  }
  if (context.editor_layer) {
    context.editor_layer->DragAndDropButton<Texture2D>(preview_image, "Preview Texture", false);
  }
  if (const auto texture_storage = preview_texture->PeekTexture2DStorage(); texture_storage.im_texture_id) {
    static float debug_scale = 0.25f;
    ImGui::DragFloat("Scale", &debug_scale, 0.01f, 0.1f, 10.0f);
    debug_scale = glm::clamp(debug_scale, 0.1f, 10.0f);
    ImGui::Image(texture_storage.im_texture_id,
                 ImVec2(texture_storage.image->GetExtent().width * debug_scale,
                        texture_storage.image->GetExtent().height * debug_scale),
                 ImVec2(0, 1), ImVec2(1, 0));
  }
  ImGui::PopID();
  return false;
}
}  // namespace

bool Gpr::SaveInternal(const std::filesystem::path& path) const {
  return SaveGpr(path);
}

bool Gpr::SaveGpr(const std::filesystem::path& path) const {
  const auto path_string = path.string();
  write_to_file(&input_buffer_, path_string.c_str());
  return true;
}
uint32_t spaces = 0;

bool Gpr::LoadInternal(const std::filesystem::path& path) {
  return LoadGpr(path);
}

bool Gpr::LoadGpr(const std::filesystem::path& path) {
  if (path.extension().string() == ".GPR" || path.extension().string() == ".gpr") {
    if (input_buffer_.buffer) {
      allocator_.Free(input_buffer_.buffer);
    }
    if (rgb_buffer_.buffer) {
      allocator_.Free(rgb_buffer_.buffer);
    }
    const auto path_string = path.string();

    if (read_from_file(&input_buffer_, path_string.c_str(), allocator_.Alloc, allocator_.Free) != 0) {
      EVOENGINE_ERROR("Failed to read GPR Image!");
      return false;
    }
    if (!gpr_parse_metadata(&allocator_, &input_buffer_, &params_)) {
      EVOENGINE_ERROR("Failed to parse GPR Image metadata!");
      return false;
    }

    if (!gpr_convert_gpr_to_rgb(&allocator_, GPR_RGB_RESOLUTION_DEFAULT, 8, &input_buffer_, &rgb_buffer_)) {
      EVOENGINE_ERROR("Failed to convert GPR Image to RGB!");
      return false;
    }
    auto data = static_cast<const uint8_t*>(rgb_buffer_.buffer);
    std::vector<glm::vec3> rgb(rgb_buffer_.width * rgb_buffer_.height);
    const auto width = rgb_buffer_.width;
    const auto height = rgb_buffer_.height;
    Jobs::RunParallelFor(width * height, [&](const auto i) {
      const auto x = i % width;
      const auto y = i / width;
      const auto src_y = height - 1 - y;
      const auto src_index = (src_y * width + x) * 3;
      rgb[i] = glm::vec3(data[src_index] / 255.f, data[src_index + 1] / 255.f, data[src_index + 2] / 255.f);
    });

    preview_image_.Get<Texture2D>()->SetRgbChannelData(rgb, {width, height});
    return true;
  }
  return true;
}

Gpr::~Gpr() {
  preview_image_.Clear();
  gpr_parameters_destroy(&params_, allocator_.Free);
  if (input_buffer_.buffer) {
    allocator_.Free(input_buffer_.buffer);
  }
}

Gpr::Gpr() {
  preview_image_ = AssetManager::CreateTemporaryAsset<Texture2D>();
  input_buffer_ = {nullptr, 0};
  rgb_buffer_ = {nullptr, 0, 0, 0};
  allocator_.Alloc = malloc;
  allocator_.Free = free;
  gpr_parameters_set_defaults(&params_);
}

AssetRef& Gpr::RefPreviewImage() {
  return preview_image_;
}

void gpr_package::RegisterGprHandlers(const std::string& owner_name) {
  Serialization::RegisterAssetIoHandler<Gpr>(
      [](const Gpr& asset, const std::filesystem::path& path) {
        return asset.SaveGpr(path);
      },
      [](Gpr& asset, const std::filesystem::path& path) {
        return asset.LoadGpr(path);
      },
      {}, {}, {}, owner_name, "Gpr");
  InspectorRegistry::GetInstance().RegisterInspector<Gpr>(InspectGpr, owner_name, "Gpr");
}
