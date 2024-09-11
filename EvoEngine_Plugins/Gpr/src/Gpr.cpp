#include "Gpr.hpp"

#include "dng_sdk/dng_exceptions.h"
#include "source/app/gpr_tools/gpr_print_utils.h"
#include "vc5_common/config.h"
#include "vc5_common/error.h"
#include "vc5_decoder/vc5_decoder.h"

using namespace evo_engine;
using namespace gpr_plugin;
bool Gpr::SaveInternal(const std::filesystem::path& path) const {
  const auto path_string = path.string();
  write_to_file(&input_buffer_, path_string.c_str());
  return true;
}
uint32_t spaces = 0;



bool Gpr::LoadInternal(const std::filesystem::path& path) {
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
    auto data = static_cast<const char*>(rgb_buffer_.buffer);
    std::vector<glm::vec3> rgb(rgb_buffer_.width * rgb_buffer_.height);
    Jobs::RunParallelFor(rgb_buffer_.width * rgb_buffer_.height, [&](const auto i) {
      rgb[i] = glm::vec3(data[i * 3] / 255.f, data[i * 3 + 1] / 255.f, data[i * 3 + 2] / 255.f);
    });

    preview_image_.Get<Texture2D>()->SetRgbChannelData(rgb, {rgb_buffer_.width, rgb_buffer_.height});
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
  preview_image_ = ProjectManager::CreateTemporaryAsset<Texture2D>();
  input_buffer_ = {nullptr, 0};
  rgb_buffer_ = {nullptr, 0, 0, 0};
  allocator_.Alloc = malloc;
  allocator_.Free = free;
  gpr_parameters_set_defaults(&params_);
}

bool Gpr::OnInspect(const std::shared_ptr<evo_engine::EditorLayer>& editor_layer) {
  EditorLayer::Draggable<Texture2D>(preview_image_);
  if (const auto texture_storage = preview_image_.Get<Texture2D>()->PeekTexture2DStorage();
      texture_storage.im_texture_id) {
    static float debug_scale = 0.25f;
    ImGui::DragFloat("Scale", &debug_scale, 0.01f, 0.1f, 10.0f);
    debug_scale = glm::clamp(debug_scale, 0.1f, 10.0f);
    ImGui::Image(texture_storage.im_texture_id,
                 ImVec2(texture_storage.image->GetExtent().width * debug_scale,
                        texture_storage.image->GetExtent().height * debug_scale),
                 ImVec2(0, 1), ImVec2(1, 0));
  }
  return false;
}
