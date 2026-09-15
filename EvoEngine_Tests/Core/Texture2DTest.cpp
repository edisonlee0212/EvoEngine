#include "EvoEngine_SDK_PCH.hpp"

#include <gtest/gtest.h>

#include "../../EvoEngine_SDK/src/Bc7TextureCodec.hpp"
#include "Application.hpp"
#include "ApplicationContext.hpp"
#include "ApplicationInitializationSettings.hpp"
#include "AssetManager.hpp"
#include "Serialization.hpp"
#include "Texture2D.hpp"

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

using namespace evo_engine;

namespace {
ApplicationInitializationSettings EmptyProjectSettings() {
  ApplicationInitializationSettings settings;
  settings.allow_empty_project = true;
  settings.load_default_resources = false;
  settings.load_project_assets = false;
  settings.load_project_start_scene = false;
  settings.enable_runtime_packages = false;
  return settings;
}

class TempDirectory {
 public:
  TempDirectory() {
    const auto now = std::chrono::steady_clock::now().time_since_epoch().count();
    root_ = std::filesystem::temp_directory_path() / ("EvoEngineTexture2DTest_" + std::to_string(now));
    std::filesystem::create_directories(root_);
  }

  ~TempDirectory() {
    std::error_code error;
    std::filesystem::remove_all(root_, error);
  }

  [[nodiscard]] std::filesystem::path Path() const {
    return root_;
  }

 private:
  std::filesystem::path root_;
};

class Texture2DTestAccess : public Texture2D {
 public:
  using Texture2D::LoadInternal;
};

constexpr uint32_t MakeFourCc(const char a, const char b, const char c, const char d) {
  return static_cast<uint32_t>(static_cast<unsigned char>(a)) |
         (static_cast<uint32_t>(static_cast<unsigned char>(b)) << 8) |
         (static_cast<uint32_t>(static_cast<unsigned char>(c)) << 16) |
         (static_cast<uint32_t>(static_cast<unsigned char>(d)) << 24);
}

void WriteLe32(std::vector<std::byte>& bytes, const size_t offset, const uint32_t value) {
  bytes[offset] = static_cast<std::byte>(value & 0xff);
  bytes[offset + 1] = static_cast<std::byte>((value >> 8) & 0xff);
  bytes[offset + 2] = static_cast<std::byte>((value >> 16) & 0xff);
  bytes[offset + 3] = static_cast<std::byte>((value >> 24) & 0xff);
}

size_t Bc7MipChainSize(uint32_t width, uint32_t height, const uint32_t mip_levels) {
  constexpr size_t bc7_block_size = 16;
  size_t size = 0;
  for (uint32_t mip_level = 0; mip_level < mip_levels; ++mip_level) {
    const size_t block_width = (static_cast<size_t>(width) + 3) / 4;
    const size_t block_height = (static_cast<size_t>(height) + 3) / 4;
    size += block_width * block_height * bc7_block_size;
    width = width > 1 ? width / 2 : 1u;
    height = height > 1 ? height / 2 : 1u;
  }
  return size;
}

void WriteBc7Dds(const std::filesystem::path& path, const uint32_t width, const uint32_t height,
                 const uint32_t dxgi_format, const uint32_t mip_levels = 1) {
  constexpr size_t header_size = 148;
  std::vector<std::byte> bytes(header_size + Bc7MipChainSize(width, height, mip_levels));

  WriteLe32(bytes, 0, MakeFourCc('D', 'D', 'S', ' '));
  WriteLe32(bytes, 4, 124);
  WriteLe32(bytes, 12, height);
  WriteLe32(bytes, 16, width);
  WriteLe32(bytes, 28, mip_levels);
  WriteLe32(bytes, 76, 32);
  WriteLe32(bytes, 84, MakeFourCc('D', 'X', '1', '0'));
  WriteLe32(bytes, 128, dxgi_format);
  WriteLe32(bytes, 132, 3);
  WriteLe32(bytes, 140, 1);
  for (size_t i = header_size; i < bytes.size(); ++i) {
    bytes[i] = static_cast<std::byte>(i & 0xff);
  }

  std::ofstream stream(path, std::ios::binary);
  stream.write(reinterpret_cast<const char*>(bytes.data()), static_cast<std::streamsize>(bytes.size()));
}

std::vector<std::byte> ReadBytes(const std::filesystem::path& path) {
  std::ifstream stream(path, std::ios::binary | std::ios::ate);
  const auto size = stream.tellg();
  if (size < 0)
    throw std::runtime_error("Failed to read DDS fixture.");
  std::vector<std::byte> bytes(static_cast<size_t>(size));
  stream.seekg(0);
  stream.read(reinterpret_cast<char*>(bytes.data()), static_cast<std::streamsize>(size));
  return bytes;
}
}  // namespace

TEST(Texture2D, LoadsDx10Bc7DdsWithoutGpuPlatform) {
  Application application;
  TempDirectory directory;
  const auto path = directory.Path() / "fixture.dds";
  WriteBc7Dds(path, 8, 4, 99);

  Texture2DTestAccess texture;
  ASSERT_TRUE(texture.LoadInternal(path));

  EXPECT_EQ(texture.GetResolution(), glm::uvec2(8, 4));
  EXPECT_EQ(texture.RefTexture2DStorage().GetFormat(), VK_FORMAT_BC7_SRGB_BLOCK);
  EXPECT_TRUE(texture.PeekLocalData().empty());

  std::vector<glm::vec4> pixels;
  texture.GetRgbaChannelData(pixels);
  EXPECT_TRUE(pixels.empty());
}

TEST(Texture2D, LoadsUppercaseDx10Bc7UnormDdsWithoutGpuPlatform) {
  Application application;
  TempDirectory directory;
  const auto path = directory.Path() / "fixture.DDS";
  WriteBc7Dds(path, 4, 8, 98);

  Texture2DTestAccess texture;
  ASSERT_TRUE(texture.LoadInternal(path));

  EXPECT_EQ(texture.GetResolution(), glm::uvec2(4, 8));
  EXPECT_EQ(texture.RefTexture2DStorage().GetFormat(), VK_FORMAT_BC7_UNORM_BLOCK);
  EXPECT_TRUE(texture.PeekLocalData().empty());
}

TEST(Texture2D, LoadsDx10Bc7DdsMipChainWithoutGpuPlatform) {
  Application application;
  TempDirectory directory;
  const auto path = directory.Path() / "fixture.dds";
  WriteBc7Dds(path, 8, 4, 98, 4);

  Texture2DTestAccess texture;
  ASSERT_TRUE(texture.LoadInternal(path));

  EXPECT_EQ(texture.GetResolution(), glm::uvec2(8, 4));
  EXPECT_EQ(texture.RefTexture2DStorage().GetFormat(), VK_FORMAT_BC7_UNORM_BLOCK);
  EXPECT_EQ(texture.RefTexture2DStorage().GetMipLevels(), 4);
}

TEST(Texture2D, SemanticOverrideSelectsBc7ViewColorSpace) {
  Application application;
  TempDirectory directory;
  const auto srgb_path = directory.Path() / "srgb.dds";
  const auto linear_path = directory.Path() / "linear.dds";
  WriteBc7Dds(srgb_path, 4, 4, 99);
  WriteBc7Dds(linear_path, 4, 4, 98);

  Texture2DTestAccess linear_texture;
  linear_texture.SetSrgbImportOverride(false);
  ASSERT_TRUE(linear_texture.LoadInternal(srgb_path));
  EXPECT_FALSE(linear_texture.srgb);
  EXPECT_EQ(linear_texture.RefTexture2DStorage().GetFormat(), VK_FORMAT_BC7_UNORM_BLOCK);
  EXPECT_FALSE(linear_texture.SamplesLinearSrgb());

  Texture2DTestAccess srgb_texture;
  srgb_texture.SetSrgbImportOverride(true);
  ASSERT_TRUE(srgb_texture.LoadInternal(linear_path));
  EXPECT_TRUE(srgb_texture.srgb);
  EXPECT_EQ(srgb_texture.RefTexture2DStorage().GetFormat(), VK_FORMAT_BC7_SRGB_BLOCK);
  EXPECT_TRUE(srgb_texture.SamplesLinearSrgb());
}

TEST(Texture2D, RejectsUnsupportedDdsDxgiFormat) {
  Application application;
  TempDirectory directory;
  const auto path = directory.Path() / "fixture.dds";
  WriteBc7Dds(path, 4, 4, 71);

  Texture2DTestAccess texture;
  EXPECT_FALSE(texture.LoadInternal(path));
}

TEST(Texture2D, SerializedBc7PayloadRoundTripsBytesFormatMipsAndColors) {
  Application application;
  ApplicationContextScope scope(application);
  application.Initialize(EmptyProjectSettings());
  TempDirectory directory;
  const auto path = directory.Path() / "fixture.dds";
  std::vector<bc7_texture_codec::MipLevel> levels;
  auto resolution = glm::uvec2(8, 4);
  while (true) {
    bc7_texture_codec::MipLevel level;
    level.resolution = resolution;
    level.pixels.resize(static_cast<size_t>(resolution.x) * resolution.y, glm::vec4(0.2f, 0.4f, 0.7f, 1.0f));
    levels.emplace_back(std::move(level));
    if (resolution == glm::uvec2(1))
      break;
    resolution = glm::max(resolution / 2u, glm::uvec2(1));
  }
  std::string error;
  ASSERT_TRUE(bc7_texture_codec::WriteDds(path, levels, true, true, &error)) << error;

  const auto source = AssetManager::CreateTemporaryAsset<Texture2D>();
  ASSERT_TRUE(Serialization::LoadAsset(*source, path));
  YAML::Emitter emitter;
  emitter << YAML::BeginMap;
  Serialization::SerializeObject(emitter, static_cast<IAsset&>(*source));
  emitter << YAML::EndMap;
  const auto node = YAML::Load(emitter.c_str());
  ASSERT_TRUE(node["compressed_pixels"]);
  EXPECT_EQ(node["compressed_format"].as<int32_t>(), VK_FORMAT_BC7_SRGB_BLOCK);
  EXPECT_EQ(node["compressed_mip_levels"].as<uint32_t>(), levels.size());

  const auto restored = AssetManager::CreateTemporaryAsset<Texture2D>();
  Serialization::DeserializeObject(node, static_cast<IAsset&>(*restored));
  EXPECT_EQ(restored->PeekCompressedData(), source->PeekCompressedData());
  EXPECT_EQ(restored->GetCompressedFormat(), VK_FORMAT_BC7_SRGB_BLOCK);
  EXPECT_EQ(restored->GetCompressedMipLevels(), levels.size());
  EXPECT_EQ(restored->GetResolution(), levels.front().resolution);

  auto restored_dds = ReadBytes(path);
  ASSERT_EQ(restored_dds.size(), 148 + restored->PeekCompressedData().size());
  std::memcpy(restored_dds.data() + 148, restored->PeekCompressedData().data(), restored->PeekCompressedData().size());
  bc7_texture_codec::MipChain decoded;
  ASSERT_TRUE(bc7_texture_codec::DecodeDds(restored_dds, decoded, &error)) << error;
  ASSERT_EQ(decoded.levels.size(), levels.size());
  for (size_t mip = 0; mip < levels.size(); ++mip) {
    ASSERT_EQ(decoded.levels[mip].pixels.size(), levels[mip].pixels.size());
    for (size_t pixel = 0; pixel < levels[mip].pixels.size(); ++pixel)
      EXPECT_LT(glm::length(decoded.levels[mip].pixels[pixel] - levels[mip].pixels[pixel]), 0.12f);
  }
}

TEST(Texture2D, RejectsNonemptySerializedTextureWithoutPixelPayload) {
  Application application;
  ApplicationContextScope scope(application);
  application.Initialize(EmptyProjectSettings());
  const auto texture = AssetManager::CreateTemporaryAsset<Texture2D>();
  const auto node = YAML::Load(R"(resolution: [4096, 4096]
red_channel: true
green_channel: true
blue_channel: true
alpha_channel: true
hdr: false
)");

  EXPECT_THROW(Serialization::DeserializeObject(node, static_cast<IAsset&>(*texture)), std::runtime_error);
  EXPECT_EQ(texture->GetResolution(), glm::uvec2(0));
  EXPECT_TRUE(texture->PeekLocalData().empty());
}
