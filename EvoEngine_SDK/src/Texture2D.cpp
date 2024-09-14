#include "Texture2D.hpp"

#include "Application.hpp"
#include "ClassRegistry.hpp"
#include "Console.hpp"
#include "EditorLayer.hpp"
#include "Jobs.hpp"
#include "Platform.hpp"
#include "TextureStorage.hpp"

using namespace evo_engine;

void Texture2D::SetData(const std::vector<glm::vec4>& data, const glm::uvec2& resolution, const bool local_copy) {
  auto& texture_storage = TextureStorage::RefTexture2DStorage(texture_storage_handle_);
  texture_storage.SetData(data, resolution);
  if (local_copy) {
    local_data_ = data;
  }
}
void Texture2D::UnsafeUploadDataImmediately() const {
  auto& texture_storage = TextureStorage::RefTexture2DStorage(texture_storage_handle_);
  texture_storage.UploadDataImmediately();
}
bool Texture2D::SaveInternal(const std::filesystem::path& path) const {
  if (path.extension() == ".png") {
    StoreToPng(path);
  } else if (path.extension() == ".jpg") {
    StoreToJpg(path);
  } else if (path.extension() == ".tga") {
    StoreToTga(path);
  } else if (path.extension() == ".hdr") {
    StoreToHdr(path);
  } else if (path.extension() == ".evetexture2d") {
    auto directory = path;
    directory.remove_filename();
    std::filesystem::create_directories(directory);
    YAML::Emitter out;
    out << YAML::BeginMap;
    Serialize(out);
    std::ofstream out_stream(path.string());
    out_stream << out.c_str();
    out_stream.flush();
    return true;
  } else {
    EVOENGINE_ERROR("Not implemented!")
    return false;
  }
  return true;
}

bool Texture2D::LoadInternal(const std::filesystem::path& path) {
  if (path.extension() == ".evetexture2d") {
    std::ifstream stream(path.string());
    std::stringstream string_stream;
    string_stream << stream.rdbuf();
    YAML::Node in = YAML::Load(string_stream.str());
    Deserialize(in);
    return true;
  }
  hdr = false;
  if (path.extension() == ".hdr")
    hdr = true;
  stbi_set_flip_vertically_on_load(true);
  int width, height, nr_components;

  float actual_gamma = hdr ? 2.2f : 1.f;

  stbi_hdr_to_ldr_gamma(actual_gamma);
  stbi_ldr_to_hdr_gamma(actual_gamma);

  void* data = stbi_loadf(path.string().c_str(), &width, &height, &nr_components, STBI_rgb_alpha);

  if (nr_components == 1) {
    red_channel = true;
    green_channel = false;
    blue_channel = false;
    alpha_channel = false;
  } else if (nr_components == 2) {
    red_channel = true;
    green_channel = true;
    blue_channel = false;
    alpha_channel = false;
  } else if (nr_components == 3) {
    red_channel = true;
    green_channel = true;
    blue_channel = true;
    alpha_channel = false;
  } else if (nr_components == 4) {
    red_channel = true;
    green_channel = true;
    blue_channel = true;
    alpha_channel = true;
  }

  if (data) {
    std::vector<glm::vec4> image_data;
    image_data.resize(width * height);
    memcpy(image_data.data(), data, sizeof(glm::vec4) * width * height);
    auto& texture_storage = TextureStorage::RefTexture2DStorage(texture_storage_handle_);
    texture_storage.SetDataImmediately(image_data, {width, height});
  } else {
    EVOENGINE_ERROR("Texture failed to load at path: " + path.filename().string());
    return false;
  }
  stbi_image_free(data);
  return true;
}

void Texture2D::ApplyOpacityMap(const std::shared_ptr<Texture2D>& target) {
  std::vector<glm::vec4> color_data;
  if (!target)
    return;
  GetRgbaChannelData(color_data);
  std::vector<glm::vec4> alpha_data;
  const auto resolution = GetResolution();
  target->GetRgbaChannelData(alpha_data, resolution.x, resolution.y);
  Jobs::RunParallelFor(color_data.size(), [&](unsigned i) {
    color_data[i].a = alpha_data[i].r;
  });
  SetRgbaChannelData(color_data, target->GetResolution());
  alpha_channel = true;
  SetUnsaved();
}

void Texture2D::SetResolution(const glm::uvec2& resolution, bool preserve_data) {
  auto& texture_storage = TextureStorage::RefTexture2DStorage(texture_storage_handle_);
  if (preserve_data && !local_data_.empty()) {
    const auto copy = local_data_;
    Resize(copy, GetResolution(), local_data_, resolution);
    texture_storage.SetData(local_data_, resolution);
  } else {
    texture_storage.Initialize(resolution);
  }
}

void Texture2D::Serialize(YAML::Emitter& out) const {
  std::vector<glm::vec4> pixels;
  if (local_data_.empty())
    GetRgbaChannelData(pixels);
  else
    pixels = local_data_;

  out << YAML::Key << "hdr" << YAML::Value << hdr;

  out << YAML::Key << "red_channel" << YAML::Value << red_channel;
  out << YAML::Key << "green_channel" << YAML::Value << green_channel;
  out << YAML::Key << "blue_channel" << YAML::Value << blue_channel;
  out << YAML::Key << "alpha_channel" << YAML::Value << alpha_channel;
  auto resolution = GetResolution();
  out << YAML::Key << "resolution" << YAML::Value << resolution;
  if (resolution.x != 0 && resolution.y != 0) {
    if (hdr) {
      Serialization::SerializeVector("pixels", pixels, out);
    } else {
      std::vector<unsigned char> transferred_pixels;
      size_t target_channel_size = 0;
      if (red_channel)
        target_channel_size++;
      if (green_channel)
        target_channel_size++;
      if (blue_channel)
        target_channel_size++;
      if (alpha_channel)
        target_channel_size++;

      transferred_pixels.resize(resolution.x * resolution.y * target_channel_size);
      Jobs::RunParallelFor(resolution.x * resolution.y, [&](unsigned i) {
        for (int channel = 0; channel < target_channel_size; channel++) {
          transferred_pixels[i * target_channel_size + channel] =
              static_cast<unsigned char>(glm::clamp(pixels[i][channel] * 255.9f, 0.f, 255.f));
        }
      });
      Serialization::SerializeVector("pixels", transferred_pixels, out);
    }
  }
}

void Texture2D::Deserialize(const YAML::Node& in) {
  std::vector<glm::vec4> pixels;
  glm::ivec2 resolution = glm::ivec2(0);

  if (in["red_channel"])
    red_channel = in["red_channel"].as<bool>();
  if (in["green_channel"])
    green_channel = in["green_channel"].as<bool>();
  if (in["blue_channel"])
    blue_channel = in["blue_channel"].as<bool>();
  if (in["alpha_channel"])
    alpha_channel = in["alpha_channel"].as<bool>();

  if (in["hdr"])
    hdr = in["hdr"].as<bool>();

  if (in["resolution"])
    resolution = in["resolution"].as<glm::ivec2>();

  if (resolution.x != 0 && resolution.y != 0) {
    if (hdr) {
      Serialization::DeserializeVector("pixels", pixels, in);
      SetRgbaChannelData(pixels, resolution);
    } else {
      size_t target_channel_size = 0;
      if (red_channel)
        target_channel_size++;
      if (green_channel)
        target_channel_size++;
      if (blue_channel)
        target_channel_size++;
      if (alpha_channel)
        target_channel_size++;

      std::vector<unsigned char> transferred_pixels;

      Serialization::DeserializeVector("pixels", transferred_pixels, in);
      transferred_pixels.resize(resolution.x * resolution.y * target_channel_size);
      pixels.resize(resolution.x * resolution.y);

      Jobs::RunParallelFor(pixels.size(), [&](unsigned i) {
        for (int channel = 0; channel < target_channel_size; channel++) {
          pixels[i][channel] = glm::clamp(transferred_pixels[i * target_channel_size + channel] / 256.f, 0.f, 1.f);
        }
        if (target_channel_size < 4) {
          pixels[i][3] = 1.f;
        }
        if (target_channel_size < 3) {
          pixels[i][2] = 0.f;
        }
        if (target_channel_size < 2) {
          pixels[i][1] = 0.f;
        }
      });

      SetRgbaChannelData(pixels, resolution);
    }
  }
}

Texture2D::Texture2D() {
  texture_storage_handle_ = TextureStorage::RegisterTexture2D();
}

const Texture2DStorage& Texture2D::PeekTexture2DStorage() const {
  return TextureStorage::PeekTexture2DStorage(texture_storage_handle_);
}

Texture2DStorage& Texture2D::RefTexture2DStorage() const {
  return TextureStorage::RefTexture2DStorage(texture_storage_handle_);
}

uint32_t Texture2D::GetTextureStorageIndex() const {
  return texture_storage_handle_->value;
}

Texture2D::~Texture2D() {
  TextureStorage::UnRegisterTexture2D(texture_storage_handle_);
}

bool Texture2D::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  bool changed = false;
  ImGui::Text((std::string("Red Channel: ") + (red_channel ? "True" : "False")).c_str());
  ImGui::Text((std::string("Green Channel: ") + (green_channel ? "True" : "False")).c_str());
  ImGui::Text((std::string("Blue Channel: ") + (blue_channel ? "True" : "False")).c_str());
  ImGui::Text((std::string("Alpha Channel: ") + (alpha_channel ? "True" : "False")).c_str());

  const auto texture_storage = PeekTexture2DStorage();
  static AssetRef temp;
  if (EditorLayer::DragAndDropButton<Texture2D>(temp, "Apply Opacity...")) {
    if (const auto tex = temp.Get<Texture2D>()) {
      ApplyOpacityMap(tex);
      temp.Clear();
    }
  }
  if (texture_storage.im_texture_id) {
    static float debug_scale = 0.25f;
    ImGui::DragFloat("Scale", &debug_scale, 0.01f, 0.1f, 10.0f);
    debug_scale = glm::clamp(debug_scale, 0.1f, 10.0f);
    ImGui::Image(texture_storage.im_texture_id,
                 ImVec2(texture_storage.image->GetExtent().width * debug_scale,
                        texture_storage.image->GetExtent().height * debug_scale),
                 ImVec2(0, 1), ImVec2(1, 0));
  }

  return changed;
}

glm::uvec2 Texture2D::GetResolution() const {
  const auto texture_storage = PeekTexture2DStorage();
  return {texture_storage.image->GetExtent().width, texture_storage.image->GetExtent().height};
}

void Texture2D::StoreToPng(const std::filesystem::path& path, const int resize_x, const int resize_y,
                           const unsigned compression_level) const {
  const auto& texture_storage = PeekTexture2DStorage();

  const auto resolution = GetResolution();

  size_t target_channel_size = 0;
  if (red_channel)
    target_channel_size++;
  if (green_channel)
    target_channel_size++;
  if (blue_channel)
    target_channel_size++;
  if (alpha_channel)
    target_channel_size++;
  std::vector<float> dst;
  constexpr size_t device_channels = 4;
  const size_t data_length = sizeof(float) * device_channels * resolution.x * resolution.y;
  dst.resize(resolution.x * resolution.y * device_channels);
  if (local_data_.empty()) {
    // Retrieve image data here.
    Buffer image_buffer(data_length);
    image_buffer.CopyFromImage(*texture_storage.image);
    image_buffer.DownloadVector(dst, resolution.x * resolution.y * device_channels);
  } else {
    memcpy(dst.data(), local_data_.data(), data_length);
  }

  StoreToPng(path, dst, resolution.x, resolution.y, 4, target_channel_size, compression_level, resize_x, resize_y);
}
void Texture2D::StoreToPng(const std::filesystem::path& path, const std::vector<float>& src_data, const int src_x,
                           const int src_y, const int src_channel_size, const int target_channel_size,
                           const unsigned compression_level, const int resize_x, const int resize_y) {
  stbi_write_png_compression_level = static_cast<int>(compression_level);
  stbi_flip_vertically_on_write(true);
  std::vector<uint8_t> pixels;
  if (resize_x > 0 && resize_y > 0 && (resize_x != src_x || resize_y != src_y)) {
    std::vector<float> res;
    res.resize(resize_x * resize_y * src_channel_size);
    stbir_resize_float_linear(src_data.data(), src_x, src_y, 0, res.data(), resize_x, resize_y, 0,
                              static_cast<stbir_pixel_layout>(src_channel_size));

    pixels.resize(resize_x * resize_y * target_channel_size);
    Jobs::RunParallelFor(resize_x * resize_y, [&](unsigned i) {
      for (int target_channel_index = 0; target_channel_index < target_channel_size; target_channel_index++) {
        pixels[i * target_channel_size + target_channel_index] =
            glm::clamp(static_cast<int>(255.9f * res[i * src_channel_size + target_channel_index]), 0, 255);
      }
    });
    stbi_write_png(path.string().c_str(), resize_x, resize_y, target_channel_size, pixels.data(),
                   resize_x * target_channel_size);
  } else {
    pixels.resize(src_x * src_y * target_channel_size);
    Jobs::RunParallelFor(src_x * src_y, [&](unsigned i) {
      for (int target_channel_index = 0; target_channel_index < target_channel_size; target_channel_index++) {
        pixels[i * target_channel_size + target_channel_index] =
            glm::clamp(static_cast<int>(255.9f * src_data[i * src_channel_size + target_channel_index]), 0, 255);
      }
    });
    stbi_write_png(path.string().c_str(), src_x, src_y, target_channel_size, pixels.data(),
                   src_x * target_channel_size);
  }
}

void Texture2D::StoreToTga(const std::filesystem::path& path, const int resize_x, const int resize_y) const {
  const auto& texture_storage = PeekTexture2DStorage();

  const auto resolution = GetResolution();

  size_t target_channel_size = 0;
  if (red_channel)
    target_channel_size++;
  if (green_channel)
    target_channel_size++;
  if (blue_channel)
    target_channel_size++;
  if (alpha_channel)
    target_channel_size++;

  std::vector<float> dst;
  constexpr size_t device_channels = 4;
  const size_t data_length = sizeof(float) * device_channels * resolution.x * resolution.y;
  dst.resize(resolution.x * resolution.y * device_channels);
  if (local_data_.empty()) {
    // Retrieve image data here.
    Buffer image_buffer(data_length);
    image_buffer.CopyFromImage(*texture_storage.image);
    image_buffer.DownloadVector(dst, resolution.x * resolution.y * device_channels);
  } else {
    memcpy(dst.data(), local_data_.data(), data_length);
  }

  StoreToTga(path, dst, resolution.x, resolution.y, 4, target_channel_size, resize_x, resize_y);
}

void Texture2D::StoreToJpg(const std::filesystem::path& path, const int resize_x, const int resize_y,
                           const unsigned quality) const {
  const auto& texture_storage = PeekTexture2DStorage();

  const auto resolution = GetResolution();

  size_t target_channel_size = 0;
  if (red_channel)
    target_channel_size++;
  if (green_channel)
    target_channel_size++;
  if (blue_channel)
    target_channel_size++;
  if (alpha_channel)
    target_channel_size++;

  std::vector<float> dst;
  constexpr size_t device_channels = 4;
  const size_t data_length = sizeof(float) * device_channels * resolution.x * resolution.y;
  dst.resize(resolution.x * resolution.y * device_channels);
  if (local_data_.empty()) {
    // Retrieve image data here.
    Buffer image_buffer(data_length);
    image_buffer.CopyFromImage(*texture_storage.image);
    image_buffer.DownloadVector(dst, resolution.x * resolution.y * device_channels);
  } else {
    memcpy(dst.data(), local_data_.data(), data_length);
  }

  StoreToJpg(path, dst, resolution.x, resolution.y, 4, target_channel_size, quality, resize_x, resize_y);
}

void Texture2D::StoreToJpg(const std::filesystem::path& path, const std::vector<float>& src_data, const int src_x,
                           const int src_y, const int src_channel_size, int target_channel_size, const unsigned quality,
                           const int resize_x, const int resize_y) {
  stbi_flip_vertically_on_write(true);

  target_channel_size = glm::max(target_channel_size, 3);
  std::vector<uint8_t> pixels;
  if (resize_x > 0 && resize_y > 0 && (resize_x != src_x || resize_y != src_y)) {
    std::vector<float> res;
    res.resize(resize_x * resize_y * src_channel_size);
    stbir_resize_float_linear(src_data.data(), src_x, src_y, 0, res.data(), resize_x, resize_y, 0,
                              static_cast<stbir_pixel_layout>(src_channel_size));

    pixels.resize(resize_x * resize_y * target_channel_size);

    Jobs::RunParallelFor(resize_x * resize_y, [&](unsigned i) {
      for (int target_channel_index = 0; target_channel_index < target_channel_size; target_channel_index++) {
        pixels[i * target_channel_size + target_channel_index] =
            glm::clamp(static_cast<int>(255.9f * res[i * src_channel_size + target_channel_index]), 0, 255);
      }
    });

    stbi_write_jpg(path.string().c_str(), resize_x, resize_y, target_channel_size, pixels.data(), quality);
  } else {
    pixels.resize(src_x * src_y * target_channel_size);
    Jobs::RunParallelFor(src_x * src_y, [&](unsigned i) {
      for (int target_channel_index = 0; target_channel_index < target_channel_size; target_channel_index++) {
        pixels[i * target_channel_size + target_channel_index] =
            glm::clamp(static_cast<int>(255.9f * src_data[i * src_channel_size + target_channel_index]), 0, 255);
      }
    });
    stbi_write_jpg(path.string().c_str(), src_x, src_y, target_channel_size, pixels.data(), quality);
  }
}

void Texture2D::StoreToTga(const std::filesystem::path& path, const std::vector<float>& src_data, const int src_x,
                           const int src_y, const int src_channel_size, const int target_channel_size,
                           const int resize_x, const int resize_y) {
  stbi_flip_vertically_on_write(true);

  std::vector<uint8_t> pixels;
  if (resize_x > 0 && resize_y > 0 && (resize_x != src_x || resize_y != src_y)) {
    std::vector<float> res;
    res.resize(resize_x * resize_y * src_channel_size);
    stbir_resize_float_linear(src_data.data(), src_x, src_y, 0, res.data(), resize_x, resize_y, 0,
                              static_cast<stbir_pixel_layout>(src_channel_size));

    pixels.resize(resize_x * resize_y * target_channel_size);
    Jobs::RunParallelFor(resize_x * resize_y, [&](unsigned i) {
      for (int target_channel_index = 0; target_channel_index < target_channel_size; target_channel_index++) {
        pixels[i * target_channel_size + target_channel_index] =
            glm::clamp(static_cast<int>(255.9f * res[i * src_channel_size + target_channel_index]), 0, 255);
      }
    });

    stbi_write_tga(path.string().c_str(), resize_x, resize_y, target_channel_size, pixels.data());
  } else {
    pixels.resize(src_x * src_y * target_channel_size);
    Jobs::RunParallelFor(src_x * src_y, [&](unsigned i) {
      for (int target_channel_index = 0; target_channel_index < target_channel_size; target_channel_index++) {
        pixels[i * target_channel_size + target_channel_index] =
            glm::clamp(static_cast<int>(255.9f * src_data[i * src_channel_size + target_channel_index]), 0, 255);
      }
    });
    stbi_write_tga(path.string().c_str(), src_x, src_y, target_channel_size, pixels.data());
  }
}

void Texture2D::StoreToHdr(const std::filesystem::path& path, const std::vector<float>& src_data, const int src_x,
                           const int src_y, const int src_channel_size, const int target_channel_size,
                           const int resize_x, const int resize_y) {
  std::vector<float> pixels;
  stbi_flip_vertically_on_write(true);
  if (resize_x > 0 && resize_y > 0 && (resize_x != src_x || resize_y != src_y)) {
    std::vector<float> res;
    res.resize(resize_x * resize_y * src_channel_size);
    stbir_resize_float_linear(src_data.data(), src_x, src_y, 0, res.data(), resize_x, resize_y, 0,
                              static_cast<stbir_pixel_layout>(src_channel_size));

    pixels.resize(resize_x * resize_y * target_channel_size);
    Jobs::RunParallelFor(resize_x * resize_y, [&](unsigned i) {
      for (int target_channel_index = 0; target_channel_index < target_channel_size; target_channel_index++) {
        pixels[i * target_channel_size + target_channel_index] = res[i * src_channel_size + target_channel_index];
      }
    });

    stbi_write_hdr(path.string().c_str(), resize_x, resize_y, target_channel_size, pixels.data());
  } else {
    pixels.resize(src_x * src_y * target_channel_size);
    Jobs::RunParallelFor(src_x * src_y, [&](unsigned i) {
      for (int target_channel_index = 0; target_channel_index < target_channel_size; target_channel_index++) {
        pixels[i * target_channel_size + target_channel_index] = src_data[i * src_channel_size + target_channel_index];
      }
    });
    stbi_write_hdr(path.string().c_str(), src_x, src_y, target_channel_size, pixels.data());
  }
}

void Texture2D::StoreToHdr(const std::filesystem::path& path, const int resize_x, const int resize_y) const {
  const auto& texture_storage = PeekTexture2DStorage();

  const auto resolution = GetResolution();

  size_t target_channel_size = 0;
  if (red_channel)
    target_channel_size++;
  if (green_channel)
    target_channel_size++;
  if (blue_channel)
    target_channel_size++;
  if (alpha_channel)
    target_channel_size++;

  std::vector<float> dst;
  constexpr size_t device_channels = 4;
  const size_t data_length = sizeof(float) * device_channels * resolution.x * resolution.y;
  dst.resize(resolution.x * resolution.y * device_channels);
  if (local_data_.empty()) {
    // Retrieve image data here.
    Buffer image_buffer(data_length);
    image_buffer.CopyFromImage(*texture_storage.image);
    image_buffer.DownloadVector(dst, resolution.x * resolution.y * device_channels);
  } else {
    memcpy(dst.data(), local_data_.data(), data_length);
  }

  StoreToHdr(path, dst, resolution.x, resolution.y, 4, target_channel_size, resize_x, resize_y);
}

ImTextureID Texture2D::GetImTextureId() const {
  const auto& texture_storage = PeekTexture2DStorage();
  return texture_storage.im_texture_id;
}

VkImageLayout Texture2D::GetLayout() const {
  const auto& texture_storage = PeekTexture2DStorage();
  return texture_storage.image->GetLayout();
}

VkImage Texture2D::GetVkImage() const {
  if (const auto& texture_storage = PeekTexture2DStorage(); texture_storage.image) {
    return texture_storage.image->GetVkImage();
  }
  return VK_NULL_HANDLE;
}

VkImageView Texture2D::GetVkImageView() const {
  if (const auto& texture_storage = PeekTexture2DStorage(); texture_storage.image_view) {
    return texture_storage.image_view->GetVkImageView();
  }
  return VK_NULL_HANDLE;
}

VkSampler Texture2D::GetVkSampler() const {
  if (const auto& texture_storage = PeekTexture2DStorage(); texture_storage.sampler) {
    return texture_storage.sampler->GetVkSampler();
  }
  return VK_NULL_HANDLE;
}

std::shared_ptr<Image> Texture2D::GetImage() const {
  const auto& texture_storage = PeekTexture2DStorage();
  return texture_storage.image;
}

void Texture2D::GetRgbaChannelData(std::vector<glm::vec4>& dst, const int resize_x, const int resize_y) const {
  const auto& texture_storage = PeekTexture2DStorage();
  const auto resolution = GetResolution();
  if ((resize_x == -1 && resize_y == -1) || (resolution.x == resize_x && resolution.y == resize_y)) {
    Buffer image_buffer(sizeof(glm::vec4) * resolution.x * resolution.y);
    image_buffer.CopyFromImage(*texture_storage.image);
    image_buffer.DownloadVector(dst, resolution.x * resolution.y);
    return;
  }
  std::vector<glm::vec4> src;
  src.resize(resolution.x * resolution.y);
  Buffer image_buffer(sizeof(glm::vec4) * resolution.x * resolution.y);
  image_buffer.CopyFromImage(*texture_storage.image);
  image_buffer.DownloadVector(src, resolution.x * resolution.y);

  dst.resize(resize_x * resize_y);
  stbir_resize_float_linear(reinterpret_cast<float*>(src.data()), resolution.x, resolution.y, 0,
                            reinterpret_cast<float*>(dst.data()), resize_x, resize_y, 0,
                            static_cast<stbir_pixel_layout>(4));
}

void Texture2D::GetRgbChannelData(std::vector<glm::vec3>& dst, int resize_x, int resize_y) const {
  const auto& texture_storage = PeekTexture2DStorage();
  const auto resolution = GetResolution();
  std::vector<glm::vec4> pixels;
  pixels.resize(resolution.x * resolution.y);
  Buffer image_buffer(sizeof(glm::vec4) * resolution.x * resolution.y);
  image_buffer.CopyFromImage(*texture_storage.image);
  image_buffer.DownloadVector(pixels, resolution.x * resolution.y);
  dst.resize(pixels.size());
  Jobs::RunParallelFor(pixels.size(), [&](unsigned i) {
    dst[i] = pixels[i];
  });
}

void Texture2D::GetRgChannelData(std::vector<glm::vec2>& dst, int resize_x, int resize_y) const {
  const auto& texture_storage = PeekTexture2DStorage();
  const auto resolution = GetResolution();
  std::vector<glm::vec4> pixels;
  pixels.resize(resolution.x * resolution.y);
  Buffer image_buffer(sizeof(glm::vec4) * resolution.x * resolution.y);
  image_buffer.CopyFromImage(*texture_storage.image);
  image_buffer.DownloadVector(pixels, resolution.x * resolution.y);
  dst.resize(pixels.size());
  Jobs::RunParallelFor(pixels.size(), [&](unsigned i) {
    dst[i] = glm::vec2(pixels[i].r, pixels[i].g);
  });
}

void Texture2D::GetRedChannelData(std::vector<float>& dst, int resize_x, int resize_y) const {
  const auto& texture_storage = PeekTexture2DStorage();
  const auto resolution = GetResolution();
  std::vector<glm::vec4> pixels;
  pixels.resize(resolution.x * resolution.y);
  Buffer image_buffer(sizeof(glm::vec4) * resolution.x * resolution.y);
  image_buffer.CopyFromImage(*texture_storage.image);
  image_buffer.DownloadVector(pixels, resolution.x * resolution.y);
  dst.resize(pixels.size());
  Jobs::RunParallelFor(pixels.size(), [&](unsigned i) {
    dst[i] = pixels[i].r;
  });
}

void Texture2D::SetRgbaChannelData(const std::vector<glm::vec4>& src, const glm::uvec2& resolution,
                                   const bool local_copy) {
  SetData(src, resolution, local_copy);
  red_channel = true;
  green_channel = true;
  blue_channel = true;
  alpha_channel = true;
  SetUnsaved();
}

void Texture2D::SetRgbChannelData(const std::vector<glm::vec3>& src, const glm::uvec2& resolution,
                                  const bool local_copy) {
  std::vector<glm::vec4> image_data;
  image_data.resize(resolution.x * resolution.y);
  Jobs::RunParallelFor(image_data.size(), [&](unsigned i) {
    image_data[i] = glm::vec4(src[i], 1.0f);
  });
  SetData(image_data, resolution, local_copy);
  red_channel = true;
  green_channel = true;
  blue_channel = true;
  alpha_channel = false;

  SetUnsaved();
}

void Texture2D::SetRgChannelData(const std::vector<glm::vec2>& src, const glm::uvec2& resolution,
                                 const bool local_copy) {
  std::vector<glm::vec4> image_data;
  image_data.resize(resolution.x * resolution.y);
  Jobs::RunParallelFor(image_data.size(), [&](unsigned i) {
    image_data[i] = glm::vec4(src[i], 0.0f, 1.0f);
  });
  SetData(image_data, resolution, local_copy);
  red_channel = true;
  green_channel = true;
  blue_channel = false;
  alpha_channel = false;

  SetUnsaved();
}

void Texture2D::SetRedChannelData(const std::vector<float>& src, const glm::uvec2& resolution, const bool local_copy) {
  std::vector<glm::vec4> image_data;
  image_data.resize(resolution.x * resolution.y);
  Jobs::RunParallelFor(image_data.size(), [&](unsigned i) {
    image_data[i] = glm::vec4(src[i], 0.0f, 0.0f, 1.0f);
  });
  SetData(image_data, resolution, local_copy);
  red_channel = true;
  green_channel = false;
  blue_channel = false;
  alpha_channel = false;

  SetUnsaved();
}

void Texture2D::Resize(const std::vector<glm::vec4>& src, const glm::uvec2& src_resolution, std::vector<glm::vec4>& dst,
                       const glm::uvec2& dst_resolution) {
  dst.resize(dst_resolution.x * dst_resolution.y);
  stbir_resize_float_linear(static_cast<const float*>(static_cast<const void*>(src.data())), src_resolution.x,
                            src_resolution.y, 0, static_cast<float*>(static_cast<void*>(dst.data())), dst_resolution.x,
                            dst_resolution.y, 0, static_cast<stbir_pixel_layout>(4));
}

void Texture2D::Resize(const std::vector<glm::vec3>& src, const glm::uvec2& src_resolution, std::vector<glm::vec3>& dst,
                       const glm::uvec2& dst_resolution) {
  dst.resize(dst_resolution.x * dst_resolution.y);
  stbir_resize_float_linear(static_cast<const float*>(static_cast<const void*>(src.data())), src_resolution.x,
                            src_resolution.y, 0, static_cast<float*>(static_cast<void*>(dst.data())), dst_resolution.x,
                            dst_resolution.y, 0, static_cast<stbir_pixel_layout>(3));
}

void Texture2D::Resize(const std::vector<glm::vec2>& src, const glm::uvec2& src_resolution, std::vector<glm::vec2>& dst,
                       const glm::uvec2& dst_resolution) {
  dst.resize(dst_resolution.x * dst_resolution.y);
  stbir_resize_float_linear(static_cast<const float*>(static_cast<const void*>(src.data())), src_resolution.x,
                            src_resolution.y, 0, static_cast<float*>(static_cast<void*>(dst.data())), dst_resolution.x,
                            dst_resolution.y, 0, static_cast<stbir_pixel_layout>(2));
}

void Texture2D::Resize(const std::vector<float>& src, const glm::uvec2& src_resolution, std::vector<float>& dst,
                       const glm::uvec2& dst_resolution) {
  dst.resize(dst_resolution.x * dst_resolution.y);
  stbir_resize_float_linear(static_cast<const float*>(static_cast<const void*>(src.data())), src_resolution.x,
                            src_resolution.y, 0, static_cast<float*>(static_cast<void*>(dst.data())), dst_resolution.x,
                            dst_resolution.y, 0, static_cast<stbir_pixel_layout>(1));
}
