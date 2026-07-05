#include "EvoEngine_SDK_PCH.hpp"

#include <gtest/gtest.h>

#include "Application.hpp"
#include "Texture2D.hpp"

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

using namespace evo_engine;

namespace {
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

TEST(Texture2D, RejectsUnsupportedDdsDxgiFormat) {
  Application application;
  TempDirectory directory;
  const auto path = directory.Path() / "fixture.dds";
  WriteBc7Dds(path, 4, 4, 71);

  Texture2DTestAccess texture;
  EXPECT_FALSE(texture.LoadInternal(path));
}
