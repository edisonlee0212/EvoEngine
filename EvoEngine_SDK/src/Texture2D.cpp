#include "Texture2D.hpp"

#include <cmath>
#include <cstddef>
#include <cstdint>

#include <stb_image_write.h>
#include "Application.hpp"
#include "AssetManager.hpp"
#include "ClassRegistry.hpp"
#include "Console.hpp"
#include "Jobs.hpp"
#include "Platform.hpp"
#include "Serialization.hpp"
#include "TextureStorage.hpp"

using namespace evo_engine;

VkSamplerCreateInfo Texture2DSamplerSettings::CreateInfo() const {
  VkSamplerCreateInfo sampler_info{};
  sampler_info.sType = VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO;
  sampler_info.magFilter = mag_filter;
  sampler_info.minFilter = min_filter;
  sampler_info.mipmapMode = mipmap_mode;
  sampler_info.addressModeU = address_mode_u;
  sampler_info.addressModeV = address_mode_v;
  sampler_info.addressModeW = VK_SAMPLER_ADDRESS_MODE_REPEAT;
  sampler_info.anisotropyEnable = VK_FALSE;
  sampler_info.maxAnisotropy = 1.0f;
  sampler_info.borderColor = VK_BORDER_COLOR_INT_OPAQUE_BLACK;
  sampler_info.unnormalizedCoordinates = VK_FALSE;
  sampler_info.compareEnable = VK_FALSE;
  sampler_info.compareOp = VK_COMPARE_OP_ALWAYS;
  sampler_info.minLod = min_lod;
  sampler_info.maxLod = max_lod;
  return sampler_info;
}

namespace {
std::vector<glm::vec4> DecodeSrgbPixels(const std::vector<glm::vec4>& encoded) {
  auto linear = encoded;
  Jobs::RunParallelFor(linear.size(), [&](const size_t index) {
    for (int channel = 0; channel < 3; ++channel) {
      const float value = linear[index][channel];
      linear[index][channel] = value <= 0.04045f ? value / 12.92f : std::pow((value + 0.055f) / 1.055f, 2.4f);
    }
  });
  return linear;
}

struct Texture2DStagedLoadPayload final : StagedAssetLoadPayload {
  bool red_channel = false;
  bool green_channel = false;
  bool blue_channel = false;
  bool alpha_channel = false;
  bool hdr = false;
  bool srgb = false;
  Texture2DSamplerSettings sampler_settings;
  glm::uvec2 resolution = glm::uvec2(0);
  std::vector<glm::vec4> pixels;
  VkFormat compressed_format = VK_FORMAT_UNDEFINED;
  uint32_t compressed_mip_levels = 1;
  std::vector<std::byte> compressed_pixels;
};

void CopyTextureImageToBuffer(const std::shared_ptr<Image>& image, Buffer& buffer) {
  Platform::WaitForFrameSubmissions("Texture Readback Fence Wait");
  buffer.CopyFromImage(*image);
}

constexpr uint32_t MakeFourCc(const char a, const char b, const char c, const char d) {
  return static_cast<uint32_t>(static_cast<unsigned char>(a)) |
         (static_cast<uint32_t>(static_cast<unsigned char>(b)) << 8) |
         (static_cast<uint32_t>(static_cast<unsigned char>(c)) << 16) |
         (static_cast<uint32_t>(static_cast<unsigned char>(d)) << 24);
}

uint32_t ReadLe32(const std::vector<std::byte>& bytes, const size_t offset) {
  return static_cast<uint32_t>(std::to_integer<unsigned char>(bytes[offset])) |
         (static_cast<uint32_t>(std::to_integer<unsigned char>(bytes[offset + 1])) << 8) |
         (static_cast<uint32_t>(std::to_integer<unsigned char>(bytes[offset + 2])) << 16) |
         (static_cast<uint32_t>(std::to_integer<unsigned char>(bytes[offset + 3])) << 24);
}

bool IsDdsPath(const std::filesystem::path& path) {
  const auto extension = path.extension().string();
  return extension == ".dds" || extension == ".DDS";
}

bool IsFloatTextureStorage(const Texture2DStorage& texture_storage) {
  return texture_storage.GetFormat() == Platform::Constants::texture_2d;
}

bool LoadDdsTexturePayload(const std::filesystem::path& path, Texture2DStagedLoadPayload& payload) {
  std::error_code error_code;
  const auto file_size = std::filesystem::file_size(path, error_code);
  if (error_code || file_size < 148) {
    EVOENGINE_ERROR("DDS texture is too small or unavailable: " + path.filename().string())
    return false;
  }

  std::ifstream stream(path, std::ios::binary);
  if (!stream) {
    EVOENGINE_ERROR("DDS texture failed to open: " + path.filename().string())
    return false;
  }

  std::vector<std::byte> bytes(static_cast<size_t>(file_size));
  stream.read(reinterpret_cast<char*>(bytes.data()), static_cast<std::streamsize>(bytes.size()));
  if (stream.gcount() != static_cast<std::streamsize>(bytes.size())) {
    EVOENGINE_ERROR("DDS texture failed to read completely: " + path.filename().string())
    return false;
  }

  constexpr uint32_t dds_magic = MakeFourCc('D', 'D', 'S', ' ');
  constexpr uint32_t dx10_four_cc = MakeFourCc('D', 'X', '1', '0');
  if (ReadLe32(bytes, 0) != dds_magic || ReadLe32(bytes, 4) != 124 || ReadLe32(bytes, 76) != 32 ||
      ReadLe32(bytes, 84) != dx10_four_cc) {
    EVOENGINE_ERROR("DDS texture must use a DX10 header: " + path.filename().string())
    return false;
  }

  constexpr uint32_t dxgi_format_bc7_unorm = 98;
  constexpr uint32_t dxgi_format_bc7_unorm_srgb = 99;
  const auto dxgi_format = ReadLe32(bytes, 128);
  if (dxgi_format == dxgi_format_bc7_unorm) {
    payload.compressed_format = VK_FORMAT_BC7_UNORM_BLOCK;
  } else if (dxgi_format == dxgi_format_bc7_unorm_srgb) {
    payload.compressed_format = VK_FORMAT_BC7_SRGB_BLOCK;
  } else {
    EVOENGINE_ERROR("DDS texture uses unsupported DXGI format " + std::to_string(dxgi_format) + ": " +
                    path.filename().string())
    return false;
  }

  const auto resource_dimension = ReadLe32(bytes, 132);
  const auto array_size = ReadLe32(bytes, 140);
  constexpr uint32_t d3d10_resource_dimension_texture2d = 3;
  if (resource_dimension != d3d10_resource_dimension_texture2d || array_size != 1) {
    EVOENGINE_ERROR("DDS texture must be a single 2D texture: " + path.filename().string())
    return false;
  }

  const uint32_t width = ReadLe32(bytes, 16);
  const uint32_t height = ReadLe32(bytes, 12);
  const uint32_t authored_mip_levels = ReadLe32(bytes, 28);
  const uint32_t mip_levels = authored_mip_levels > 0 ? authored_mip_levels : 1u;
  if (width == 0 || height == 0) {
    EVOENGINE_ERROR("DDS texture has invalid dimensions: " + path.filename().string())
    return false;
  }

  constexpr size_t data_offset = 148;
  constexpr size_t bc7_block_size = 16;
  size_t mip_chain_size = 0;
  uint32_t mip_width = width;
  uint32_t mip_height = height;
  for (uint32_t mip_level = 0; mip_level < mip_levels; ++mip_level) {
    const size_t block_width = (static_cast<size_t>(mip_width) + 3) / 4;
    const size_t block_height = (static_cast<size_t>(mip_height) + 3) / 4;
    mip_chain_size += block_width * block_height * bc7_block_size;
    mip_width = mip_width > 1 ? mip_width / 2 : 1u;
    mip_height = mip_height > 1 ? mip_height / 2 : 1u;
  }
  if (bytes.size() < data_offset + mip_chain_size) {
    EVOENGINE_ERROR("DDS texture does not contain the expected BC7 mip chain: " + path.filename().string())
    return false;
  }

  payload.hdr = false;
  payload.red_channel = true;
  payload.green_channel = true;
  payload.blue_channel = true;
  payload.alpha_channel = true;
  payload.resolution = glm::uvec2(width, height);
  payload.pixels.clear();
  payload.compressed_mip_levels = mip_levels;
  payload.compressed_pixels.assign(bytes.begin() + data_offset, bytes.begin() + data_offset + mip_chain_size);
  return true;
}

VkFormat ApplySrgbOverride(const VkFormat format, const std::optional<bool>& srgb_override) {
  if (!srgb_override || (format != VK_FORMAT_BC7_UNORM_BLOCK && format != VK_FORMAT_BC7_SRGB_BLOCK)) {
    return format;
  }
  return *srgb_override ? VK_FORMAT_BC7_SRGB_BLOCK : VK_FORMAT_BC7_UNORM_BLOCK;
}

void DecodeSerializedTexture2D(const YAML::Node& in, Texture2DStagedLoadPayload& payload) {
  if (in["red_channel"])
    payload.red_channel = in["red_channel"].as<bool>();
  if (in["green_channel"])
    payload.green_channel = in["green_channel"].as<bool>();
  if (in["blue_channel"])
    payload.blue_channel = in["blue_channel"].as<bool>();
  if (in["alpha_channel"])
    payload.alpha_channel = in["alpha_channel"].as<bool>();
  if (in["hdr"])
    payload.hdr = in["hdr"].as<bool>();
  if (in["srgb"])
    payload.srgb = in["srgb"].as<bool>();
  if (const auto sampler = in["sampler"]) {
    if (sampler["mag_filter"])
      payload.sampler_settings.mag_filter = static_cast<VkFilter>(sampler["mag_filter"].as<int32_t>());
    if (sampler["min_filter"])
      payload.sampler_settings.min_filter = static_cast<VkFilter>(sampler["min_filter"].as<int32_t>());
    if (sampler["mipmap_mode"])
      payload.sampler_settings.mipmap_mode = static_cast<VkSamplerMipmapMode>(sampler["mipmap_mode"].as<int32_t>());
    if (sampler["address_mode_u"])
      payload.sampler_settings.address_mode_u =
          static_cast<VkSamplerAddressMode>(sampler["address_mode_u"].as<int32_t>());
    if (sampler["address_mode_v"])
      payload.sampler_settings.address_mode_v =
          static_cast<VkSamplerAddressMode>(sampler["address_mode_v"].as<int32_t>());
    if (sampler["min_lod"])
      payload.sampler_settings.min_lod = sampler["min_lod"].as<float>();
    if (sampler["max_lod"])
      payload.sampler_settings.max_lod = sampler["max_lod"].as<float>();
  }

  glm::ivec2 resolution = glm::ivec2(0);
  if (in["resolution"])
    resolution = in["resolution"].as<glm::ivec2>();
  payload.resolution = glm::uvec2(glm::max(resolution.x, 0), glm::max(resolution.y, 0));

  if (payload.resolution.x == 0 || payload.resolution.y == 0) {
    return;
  }

  if (payload.hdr) {
    Serialization::DeserializeVector("pixels", payload.pixels, in);
    return;
  }

  size_t target_channel_size = 0;
  if (payload.red_channel)
    target_channel_size++;
  if (payload.green_channel)
    target_channel_size++;
  if (payload.blue_channel)
    target_channel_size++;
  if (payload.alpha_channel)
    target_channel_size++;

  std::vector<unsigned char> transferred_pixels;
  Serialization::DeserializeVector("pixels", transferred_pixels, in);
  transferred_pixels.resize(payload.resolution.x * payload.resolution.y * target_channel_size);
  payload.pixels.resize(payload.resolution.x * payload.resolution.y);

  Jobs::RunParallelFor(payload.pixels.size(), [&](size_t i) {
    for (int channel = 0; channel < target_channel_size; channel++) {
      payload.pixels[i][channel] = glm::clamp(transferred_pixels[i * target_channel_size + channel] / 255.0f, 0.f, 1.f);
    }
    if (target_channel_size < 4) {
      payload.pixels[i][3] = 1.f;
    }
    if (target_channel_size < 3) {
      payload.pixels[i][2] = 0.f;
    }
    if (target_channel_size < 2) {
      payload.pixels[i][1] = 0.f;
    }
  });
}
}  // namespace

void Texture2D::SetData(const std::vector<glm::vec4>& data, const glm::uvec2& resolution, const bool local_copy) {
  auto& texture_storage = TextureStorage::RefTexture2DStorage(texture_storage_handle_);
  const auto format = srgb && !hdr ? VK_FORMAT_R8G8B8A8_SRGB : VK_FORMAT_UNDEFINED;
  srgb_fallback_linear_ = false;
  if (Platform::Initialized()) {
    auto upload = texture_storage.SetDataAsync(data, resolution, format);
    if (!upload.Valid() && format != VK_FORMAT_UNDEFINED) {
      EVOENGINE_WARNING("sRGB texture storage is unavailable; falling back to linear float storage.")
      upload = texture_storage.SetDataAsync(DecodeSrgbPixels(data), resolution, VK_FORMAT_UNDEFINED, true);
      srgb_fallback_linear_ = upload.Valid();
    }
    TrackPendingGpuWork(upload);
  } else {
    texture_storage.SetData(data, resolution, format);
  }
  if (local_copy) {
    local_data_ = data;
  }
}

void Texture2D::DownloadData() {
  WaitForPendingGpuWork();
  const auto& texture_storage = PeekTexture2DStorage();
  if (!texture_storage.image || !IsFloatTextureStorage(texture_storage)) {
    local_data_.clear();
    return;
  }
  const auto resolution = GetResolution();
  local_data_.resize(resolution.x * resolution.y);
  Buffer image_buffer(sizeof(glm::vec4) * resolution.x * resolution.y);
  CopyTextureImageToBuffer(texture_storage.image, image_buffer);
  image_buffer.DownloadVector(local_data_, resolution.x * resolution.y);
}

void Texture2D::UnsafeUploadDataImmediately() const {
  auto& texture_storage = TextureStorage::RefTexture2DStorage(texture_storage_handle_);
  texture_storage.UploadPendingDataImmediately();
  WaitForPendingGpuWork();
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
    Serialization::SerializeObject(out, static_cast<const IAsset&>(*this));
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
    Serialization::DeserializeObject(in, static_cast<IAsset&>(*this));
    return true;
  }
  if (IsDdsPath(path)) {
    Texture2DStagedLoadPayload payload;
    if (!LoadDdsTexturePayload(path, payload)) {
      return false;
    }
    payload.compressed_format = ApplySrgbOverride(payload.compressed_format, srgb_import_override_);
    hdr = payload.hdr;
    srgb = payload.compressed_format == VK_FORMAT_BC7_SRGB_BLOCK;
    srgb_fallback_linear_ = false;
    red_channel = payload.red_channel;
    green_channel = payload.green_channel;
    blue_channel = payload.blue_channel;
    alpha_channel = payload.alpha_channel;
    local_data_.clear();
    auto& texture_storage = TextureStorage::RefTexture2DStorage(texture_storage_handle_);
    if (Platform::Initialized()) {
      const auto upload = texture_storage.SetCompressedDataAsync(
          payload.compressed_pixels, payload.resolution, payload.compressed_format, payload.compressed_mip_levels);
      if (!upload.Valid()) {
        return false;
      }
      Platform::GetGpuService().Wait(upload);
    } else {
      texture_storage.SetCompressedData(payload.compressed_pixels, payload.resolution, payload.compressed_format,
                                        payload.compressed_mip_levels);
    }
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
    local_data_.resize(width * height);
    memcpy(local_data_.data(), data, sizeof(glm::vec4) * width * height);
    auto& texture_storage = TextureStorage::RefTexture2DStorage(texture_storage_handle_);
    const auto srgb_format = srgb && !hdr ? VK_FORMAT_R8G8B8A8_SRGB : VK_FORMAT_UNDEFINED;
    srgb_fallback_linear_ = false;
    if (Platform::Initialized()) {
      auto upload = texture_storage.SetDataAsync(local_data_, {width, height}, srgb_format);
      if (!upload.Valid() && srgb_format != VK_FORMAT_UNDEFINED) {
        EVOENGINE_WARNING("sRGB texture storage is unavailable; falling back to linear float storage.")
        upload =
            texture_storage.SetDataAsync(DecodeSrgbPixels(local_data_), {width, height}, VK_FORMAT_UNDEFINED, true);
        srgb_fallback_linear_ = upload.Valid();
      }
      if (!upload.Valid()) {
        stbi_image_free(data);
        return false;
      }
      Platform::GetGpuService().Wait(upload);
    } else {
      texture_storage.SetData(local_data_, {width, height}, srgb_format);
    }
  } else {
    EVOENGINE_ERROR("Texture failed to load at path: " + path.filename().string());
    return false;
  }
  stbi_image_free(data);
  return true;
}

bool Texture2D::SupportsStagedLoading() const {
  return true;
}

std::shared_ptr<StagedAssetLoadPayload> Texture2D::LoadStagedPayloadInternal(const std::filesystem::path& path) const {
  auto payload = std::make_shared<Texture2DStagedLoadPayload>();
  payload->srgb = srgb;
  payload->sampler_settings = sampler_settings_;
  if (path.extension() == ".evetexture2d") {
    std::ifstream stream(path.string());
    std::stringstream string_stream;
    string_stream << stream.rdbuf();
    const YAML::Node in = YAML::Load(string_stream.str());
    DecodeSerializedTexture2D(in, *payload);
    return payload;
  }
  if (IsDdsPath(path)) {
    if (!LoadDdsTexturePayload(path, *payload)) {
      return {};
    }
    payload->compressed_format = ApplySrgbOverride(payload->compressed_format, srgb_import_override_);
    payload->srgb = payload->compressed_format == VK_FORMAT_BC7_SRGB_BLOCK;
    return payload;
  }

  payload->hdr = path.extension() == ".hdr";
  stbi_set_flip_vertically_on_load(true);
  int width = 0;
  int height = 0;
  int nr_components = 0;

  const float actual_gamma = payload->hdr ? 2.2f : 1.f;
  stbi_hdr_to_ldr_gamma(actual_gamma);
  stbi_ldr_to_hdr_gamma(actual_gamma);

  void* data = stbi_loadf(path.string().c_str(), &width, &height, &nr_components, STBI_rgb_alpha);
  if (!data) {
    EVOENGINE_ERROR("Texture failed to load at path: " + path.filename().string());
    return {};
  }

  if (nr_components == 1) {
    payload->red_channel = true;
  } else if (nr_components == 2) {
    payload->red_channel = true;
    payload->green_channel = true;
  } else if (nr_components == 3) {
    payload->red_channel = true;
    payload->green_channel = true;
    payload->blue_channel = true;
  } else if (nr_components == 4) {
    payload->red_channel = true;
    payload->green_channel = true;
    payload->blue_channel = true;
    payload->alpha_channel = true;
  }

  payload->resolution = glm::uvec2(width, height);
  payload->pixels.resize(width * height);
  memcpy(payload->pixels.data(), data, sizeof(glm::vec4) * width * height);
  stbi_image_free(data);
  return payload;
}

bool Texture2D::ApplyStagedPayloadInternal(const std::filesystem::path&,
                                           const std::shared_ptr<StagedAssetLoadPayload>& payload) {
  const auto texture_payload = std::dynamic_pointer_cast<Texture2DStagedLoadPayload>(payload);
  if (!texture_payload) {
    return false;
  }

  hdr = texture_payload->hdr;
  srgb = texture_payload->srgb || texture_payload->compressed_format == VK_FORMAT_BC7_SRGB_BLOCK;
  sampler_settings_ = texture_payload->sampler_settings;
  RefTexture2DStorage().SetSampler(sampler_settings_.CreateInfo());
  red_channel = texture_payload->red_channel;
  green_channel = texture_payload->green_channel;
  blue_channel = texture_payload->blue_channel;
  alpha_channel = texture_payload->alpha_channel;
  if (texture_payload->compressed_format != VK_FORMAT_UNDEFINED) {
    srgb_fallback_linear_ = false;
    local_data_.clear();
    if (!texture_payload->compressed_pixels.empty() && texture_payload->resolution.x != 0 &&
        texture_payload->resolution.y != 0) {
      auto& texture_storage = TextureStorage::RefTexture2DStorage(texture_storage_handle_);
      if (Platform::Initialized()) {
        const auto upload = texture_storage.SetCompressedDataAsync(
            texture_payload->compressed_pixels, texture_payload->resolution, texture_payload->compressed_format,
            texture_payload->compressed_mip_levels);
        if (!upload.Valid()) {
          return false;
        }
        TrackPendingGpuWork(upload);
      } else {
        texture_storage.SetCompressedData(texture_payload->compressed_pixels, texture_payload->resolution,
                                          texture_payload->compressed_format, texture_payload->compressed_mip_levels);
      }
    }
    return true;
  }

  local_data_ = texture_payload->pixels;

  if (!local_data_.empty() && texture_payload->resolution.x != 0 && texture_payload->resolution.y != 0) {
    SetData(local_data_, texture_payload->resolution, true);
  }
  return true;
}

bool Texture2D::RegisterAssetIoHandlers(const std::string& owner_name, const std::string& type_name) {
  return Serialization::RegisterAssetIoHandler<Texture2D>(
      [](const Texture2D& asset, const std::filesystem::path& path) {
        return asset.SaveInternal(path);
      },
      [](Texture2D& asset, const std::filesystem::path& path) {
        return asset.LoadInternal(path);
      },
      [](const Texture2D& asset, const std::filesystem::path&) {
        return asset.SupportsStagedLoading();
      },
      [](const Texture2D& asset, const std::filesystem::path& path) {
        return asset.LoadStagedPayloadInternal(path);
      },
      [](Texture2D& asset, const std::filesystem::path& path, const std::shared_ptr<StagedAssetLoadPayload>& payload) {
        return asset.ApplyStagedPayloadInternal(path, payload);
      },
      owner_name, type_name);
}

void Texture2D::ApplyOpacityMap(const std::shared_ptr<Texture2D>& target) {
  std::vector<glm::vec4> color_data;
  if (!target)
    return;
  GetRgbaChannelData(color_data);
  if (color_data.empty())
    return;
  std::vector<glm::vec4> alpha_data;
  const auto resolution = GetResolution();
  target->GetRgbaChannelData(alpha_data, resolution.x, resolution.y);
  if (alpha_data.size() < color_data.size())
    return;
  Jobs::RunParallelFor(color_data.size(), [&](size_t i) {
    color_data[i].a = alpha_data[i].r;
  });
  SetRgbaChannelData(color_data, target->GetResolution());
  alpha_channel = true;
  SetUnsaved();
}

void Texture2D::SetResolution(const glm::uvec2& resolution, bool preserve_data) {
  if (preserve_data && !local_data_.empty()) {
    const auto copy = local_data_;
    Resize(copy, GetResolution(), local_data_, resolution);
    SetData(local_data_, resolution, true);
  } else {
    local_data_.clear();
    if (resolution.x != 0 && resolution.y != 0) {
      SetData(std::vector<glm::vec4>(static_cast<size_t>(resolution.x) * resolution.y), resolution, false);
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

glm::uvec2 Texture2D::GetResolution() const {
  const auto texture_storage = PeekTexture2DStorage();
  if (!texture_storage.image) {
    if (texture_storage.new_compressed_resolution_.x != 0 && texture_storage.new_compressed_resolution_.y != 0) {
      return texture_storage.new_compressed_resolution_;
    }
    return texture_storage.new_resolution_;
  }
  return {texture_storage.image->GetExtent().width, texture_storage.image->GetExtent().height};
}

void Texture2D::StoreToPng(const std::filesystem::path& path, const int resize_x, const int resize_y,
                           const unsigned compression_level) const {
  WaitForPendingGpuWork();
  const auto& texture_storage = PeekTexture2DStorage();
  if (!texture_storage.image || !IsFloatTextureStorage(texture_storage)) {
    return;
  }

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
    CopyTextureImageToBuffer(texture_storage.image, image_buffer);
    image_buffer.DownloadVector(dst, resolution.x * resolution.y * device_channels);
  } else {
    memcpy(dst.data(), local_data_.data(), data_length);
  }

  StoreToPng(path, dst, resolution.x, resolution.y, 4, target_channel_size, compression_level, resize_x, resize_y);
}
void Texture2D::StoreToPng(const std::filesystem::path& path, const std::vector<float>& src_data, const int src_x,
                           const int src_y, const int src_channel_size, const int target_channel_size,
                           const unsigned compression_level, const int resize_x, const int resize_y) {
  stbi_flip_vertically_on_write(true);
  std::vector<uint8_t> pixels;
  if (resize_x > 0 && resize_y > 0 && (resize_x != src_x || resize_y != src_y)) {
    std::vector<float> res;
    res.resize(resize_x * resize_y * src_channel_size);
    stbir_resize_float_linear(src_data.data(), src_x, src_y, 0, res.data(), resize_x, resize_y, 0,
                              static_cast<stbir_pixel_layout>(src_channel_size));

    pixels.resize(resize_x * resize_y * target_channel_size);
    Jobs::RunParallelFor(resize_x * resize_y, [&](size_t i) {
      for (int target_channel_index = 0; target_channel_index < target_channel_size; target_channel_index++) {
        pixels[i * target_channel_size + target_channel_index] =
            glm::clamp(static_cast<int>(255.9f * res[i * src_channel_size + target_channel_index]), 0, 255);
      }
    });
    stbi_write_png(path.string().c_str(), resize_x, resize_y, target_channel_size, pixels.data(),
                   resize_x * target_channel_size);
  } else {
    pixels.resize(src_x * src_y * target_channel_size);
    Jobs::RunParallelFor(src_x * src_y, [&](size_t i) {
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
  WaitForPendingGpuWork();
  const auto& texture_storage = PeekTexture2DStorage();
  if (!texture_storage.image || !IsFloatTextureStorage(texture_storage)) {
    return;
  }

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
    CopyTextureImageToBuffer(texture_storage.image, image_buffer);
    image_buffer.DownloadVector(dst, resolution.x * resolution.y * device_channels);
  } else {
    memcpy(dst.data(), local_data_.data(), data_length);
  }

  StoreToTga(path, dst, resolution.x, resolution.y, 4, target_channel_size, resize_x, resize_y);
}

void Texture2D::StoreToJpg(const std::filesystem::path& path, const int resize_x, const int resize_y,
                           const unsigned quality) const {
  WaitForPendingGpuWork();
  const auto& texture_storage = PeekTexture2DStorage();
  if (!texture_storage.image || !IsFloatTextureStorage(texture_storage)) {
    return;
  }

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
    CopyTextureImageToBuffer(texture_storage.image, image_buffer);
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

    Jobs::RunParallelFor(resize_x * resize_y, [&](size_t i) {
      for (int target_channel_index = 0; target_channel_index < target_channel_size; target_channel_index++) {
        pixels[i * target_channel_size + target_channel_index] =
            glm::clamp(static_cast<int>(255.9f * res[i * src_channel_size + target_channel_index]), 0, 255);
      }
    });

    stbi_write_jpg(path.string().c_str(), resize_x, resize_y, target_channel_size, pixels.data(), quality);
  } else {
    pixels.resize(src_x * src_y * target_channel_size);
    Jobs::RunParallelFor(src_x * src_y, [&](size_t i) {
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
    Jobs::RunParallelFor(resize_x * resize_y, [&](size_t i) {
      for (int target_channel_index = 0; target_channel_index < target_channel_size; target_channel_index++) {
        pixels[i * target_channel_size + target_channel_index] =
            glm::clamp(static_cast<int>(255.9f * res[i * src_channel_size + target_channel_index]), 0, 255);
      }
    });

    stbi_write_tga(path.string().c_str(), resize_x, resize_y, target_channel_size, pixels.data());
  } else {
    pixels.resize(src_x * src_y * target_channel_size);
    Jobs::RunParallelFor(src_x * src_y, [&](size_t i) {
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
    Jobs::RunParallelFor(resize_x * resize_y, [&](size_t i) {
      for (int target_channel_index = 0; target_channel_index < target_channel_size; target_channel_index++) {
        pixels[i * target_channel_size + target_channel_index] = res[i * src_channel_size + target_channel_index];
      }
    });

    stbi_write_hdr(path.string().c_str(), resize_x, resize_y, target_channel_size, pixels.data());
  } else {
    pixels.resize(src_x * src_y * target_channel_size);
    Jobs::RunParallelFor(src_x * src_y, [&](size_t i) {
      for (int target_channel_index = 0; target_channel_index < target_channel_size; target_channel_index++) {
        pixels[i * target_channel_size + target_channel_index] = src_data[i * src_channel_size + target_channel_index];
      }
    });
    stbi_write_hdr(path.string().c_str(), src_x, src_y, target_channel_size, pixels.data());
  }
}

void Texture2D::StoreToHdr(const std::filesystem::path& path, const int resize_x, const int resize_y) const {
  WaitForPendingGpuWork();
  const auto& texture_storage = PeekTexture2DStorage();
  if (!texture_storage.image || !IsFloatTextureStorage(texture_storage)) {
    return;
  }

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
    CopyTextureImageToBuffer(texture_storage.image, image_buffer);
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

void Texture2D::SetSamplerSettings(const Texture2DSamplerSettings& settings) {
  sampler_settings_ = settings;
  RefTexture2DStorage().SetSampler(settings.CreateInfo());
  SetUnsaved();
}

const Texture2DSamplerSettings& Texture2D::GetSamplerSettings() const {
  return sampler_settings_;
}

void Texture2D::SetSrgbImportOverride(const bool value) {
  srgb = value;
  srgb_import_override_ = value;
}

bool Texture2D::ShareGpuImage(const Texture2D& source, const bool requested_srgb,
                              const Texture2DSamplerSettings& sampler_settings) {
  const auto source_format = source.PeekTexture2DStorage().GetFormat();
  VkFormat view_format = source_format;
  if (source_format == VK_FORMAT_BC7_UNORM_BLOCK || source_format == VK_FORMAT_BC7_SRGB_BLOCK) {
    view_format = requested_srgb ? VK_FORMAT_BC7_SRGB_BLOCK : VK_FORMAT_BC7_UNORM_BLOCK;
  } else if (source_format == VK_FORMAT_R8G8B8A8_UNORM || source_format == VK_FORMAT_R8G8B8A8_SRGB) {
    view_format = requested_srgb ? VK_FORMAT_R8G8B8A8_SRGB : VK_FORMAT_R8G8B8A8_UNORM;
  } else if (requested_srgb != source.SamplesLinearSrgb()) {
    return false;
  }
  if (!RefTexture2DStorage().ShareImage(source.PeekTexture2DStorage(), view_format, sampler_settings.CreateInfo())) {
    return false;
  }
  sampler_settings_ = sampler_settings;
  srgb = requested_srgb;
  srgb_import_override_ = requested_srgb;
  srgb_fallback_linear_ = requested_srgb && source.SamplesLinearSrgb() && !RefTexture2DStorage().SamplesLinearSrgb();
  hdr = source.hdr;
  red_channel = source.red_channel;
  green_channel = source.green_channel;
  blue_channel = source.blue_channel;
  alpha_channel = source.alpha_channel;
  local_data_.clear();
  return true;
}

bool Texture2D::SamplesLinearSrgb() const {
  return srgb && (srgb_fallback_linear_ || PeekTexture2DStorage().SamplesLinearSrgb());
}

std::shared_ptr<Image> Texture2D::GetImage() const {
  const auto& texture_storage = PeekTexture2DStorage();
  return texture_storage.image;
}

const std::vector<glm::vec4>& Texture2D::GetLocalData() {
  if (local_data_.empty() && IsFloatTextureStorage(PeekTexture2DStorage()))
    DownloadData();
  return local_data_;
}

const std::vector<glm::vec4>& Texture2D::PeekLocalData() const {
  return local_data_;
}

std::shared_ptr<Texture2D> Texture2D::GenerateThumbnailTexture() {
  std::shared_ptr<Texture2D> ret_val = AssetManager::CreateTemporaryAsset<Texture2D>();
  const glm::vec2 resolution = GetResolution();
  const float max_dim = glm::max(resolution.x, resolution.y);
  const glm::vec2 new_resolution = resolution * glm::min(1.f, 512.f / max_dim);
  const auto& local_data = GetLocalData();
  if (local_data.empty()) {
    return {};
  }
  auto copy_data = local_data;
  Resize(local_data, resolution, copy_data, new_resolution);
  ret_val->SetRgbaChannelData(copy_data, new_resolution, false);
  return ret_val;
}

void Texture2D::GetRgbaChannelData(std::vector<glm::vec4>& dst, const int resize_x, const int resize_y) const {
  WaitForPendingGpuWork();
  const auto& texture_storage = PeekTexture2DStorage();
  if (!texture_storage.image || !IsFloatTextureStorage(texture_storage)) {
    dst.clear();
    return;
  }
  const auto resolution = GetResolution();
  if ((resize_x == -1 && resize_y == -1) || (resolution.x == resize_x && resolution.y == resize_y)) {
    Buffer image_buffer(sizeof(glm::vec4) * resolution.x * resolution.y);
    CopyTextureImageToBuffer(texture_storage.image, image_buffer);
    image_buffer.DownloadVector(dst, resolution.x * resolution.y);
    return;
  }
  std::vector<glm::vec4> src;
  src.resize(resolution.x * resolution.y);
  Buffer image_buffer(sizeof(glm::vec4) * resolution.x * resolution.y);
  CopyTextureImageToBuffer(texture_storage.image, image_buffer);
  image_buffer.DownloadVector(src, resolution.x * resolution.y);

  dst.resize(resize_x * resize_y);
  stbir_resize_float_linear(reinterpret_cast<float*>(src.data()), resolution.x, resolution.y, 0,
                            reinterpret_cast<float*>(dst.data()), resize_x, resize_y, 0,
                            static_cast<stbir_pixel_layout>(4));
}

void Texture2D::GetRgbChannelData(std::vector<glm::vec3>& dst, int resize_x, int resize_y) const {
  WaitForPendingGpuWork();
  const auto& texture_storage = PeekTexture2DStorage();
  if (!texture_storage.image || !IsFloatTextureStorage(texture_storage)) {
    dst.clear();
    return;
  }
  const auto resolution = GetResolution();
  std::vector<glm::vec4> pixels;
  pixels.resize(resolution.x * resolution.y);
  Buffer image_buffer(sizeof(glm::vec4) * resolution.x * resolution.y);
  CopyTextureImageToBuffer(texture_storage.image, image_buffer);
  image_buffer.DownloadVector(pixels, resolution.x * resolution.y);
  dst.resize(pixels.size());
  Jobs::RunParallelFor(pixels.size(), [&](size_t i) {
    dst[i] = pixels[i];
  });
}

void Texture2D::GetRgChannelData(std::vector<glm::vec2>& dst, int resize_x, int resize_y) const {
  WaitForPendingGpuWork();
  const auto& texture_storage = PeekTexture2DStorage();
  if (!texture_storage.image || !IsFloatTextureStorage(texture_storage)) {
    dst.clear();
    return;
  }
  const auto resolution = GetResolution();
  std::vector<glm::vec4> pixels;
  pixels.resize(resolution.x * resolution.y);
  Buffer image_buffer(sizeof(glm::vec4) * resolution.x * resolution.y);
  CopyTextureImageToBuffer(texture_storage.image, image_buffer);
  image_buffer.DownloadVector(pixels, resolution.x * resolution.y);
  dst.resize(pixels.size());
  Jobs::RunParallelFor(pixels.size(), [&](size_t i) {
    dst[i] = glm::vec2(pixels[i].r, pixels[i].g);
  });
}

void Texture2D::GetRedChannelData(std::vector<float>& dst, int resize_x, int resize_y) const {
  WaitForPendingGpuWork();
  const auto& texture_storage = PeekTexture2DStorage();
  if (!texture_storage.image || !IsFloatTextureStorage(texture_storage)) {
    dst.clear();
    return;
  }
  const auto resolution = GetResolution();
  std::vector<glm::vec4> pixels;
  pixels.resize(resolution.x * resolution.y);
  Buffer image_buffer(sizeof(glm::vec4) * resolution.x * resolution.y);
  CopyTextureImageToBuffer(texture_storage.image, image_buffer);
  image_buffer.DownloadVector(pixels, resolution.x * resolution.y);
  dst.resize(pixels.size());
  Jobs::RunParallelFor(pixels.size(), [&](size_t i) {
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
  Jobs::RunParallelFor(image_data.size(), [&](size_t i) {
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
  Jobs::RunParallelFor(image_data.size(), [&](size_t i) {
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
  Jobs::RunParallelFor(image_data.size(), [&](size_t i) {
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
