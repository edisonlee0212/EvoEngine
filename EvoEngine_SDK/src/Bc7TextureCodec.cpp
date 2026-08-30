#include "Bc7TextureCodec.hpp"

#include <array>
#include <atomic>
#include <cstring>
#include <fstream>
#include <mutex>

#include <bc7decomp.h>
#include <bc7enc.h>
#include "Jobs.hpp"

namespace evo_engine::bc7_texture_codec {
namespace {
constexpr size_t kDdsHeaderSize = 148;
constexpr size_t kBc7BlockSize = 16;
constexpr uint32_t kDxgiBc7Unorm = 98;
constexpr uint32_t kDxgiBc7Srgb = 99;

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

void WriteLe32(std::vector<std::byte>& bytes, const size_t offset, const uint32_t value) {
  for (size_t byte = 0; byte < 4; ++byte) {
    bytes[offset + byte] = static_cast<std::byte>((value >> (byte * 8)) & 0xffu);
  }
}

uint64_t HashBytes(const std::vector<std::byte>& bytes) {
  uint64_t hash = 14695981039346656037ull;
  for (const auto value : bytes) {
    hash ^= std::to_integer<unsigned char>(value);
    hash *= 1099511628211ull;
  }
  return hash;
}

void SetError(std::string* error, const std::string& message) {
  if (error) {
    *error = message;
  }
}

bool ReadFile(const std::filesystem::path& path, std::vector<std::byte>& bytes, std::string* error) {
  std::error_code file_error;
  const auto size = std::filesystem::file_size(path, file_error);
  if (file_error || size < kDdsHeaderSize) {
    SetError(error, "DDS file is unavailable or too small.");
    return false;
  }
  std::ifstream stream(path, std::ios::binary);
  bytes.resize(static_cast<size_t>(size));
  if (!stream.read(reinterpret_cast<char*>(bytes.data()), static_cast<std::streamsize>(bytes.size()))) {
    SetError(error, "DDS file could not be read completely.");
    return false;
  }
  return true;
}

bool ParseHeader(const std::vector<std::byte>& bytes, glm::uvec2& resolution, uint32_t& mip_levels, bool& srgb,
                 std::string* error) {
  constexpr uint32_t kDdsMagic = MakeFourCc('D', 'D', 'S', ' ');
  constexpr uint32_t kDx10FourCc = MakeFourCc('D', 'X', '1', '0');
  if (bytes.size() < kDdsHeaderSize || ReadLe32(bytes, 0) != kDdsMagic || ReadLe32(bytes, 4) != 124 ||
      ReadLe32(bytes, 76) != 32 || ReadLe32(bytes, 84) != kDx10FourCc) {
    SetError(error, "DDS must use a DX10 header.");
    return false;
  }
  const auto format = ReadLe32(bytes, 128);
  if (format != kDxgiBc7Unorm && format != kDxgiBc7Srgb) {
    SetError(error, "DDS is not BC7 UNORM or BC7 sRGB.");
    return false;
  }
  if (ReadLe32(bytes, 132) != 3 || ReadLe32(bytes, 140) != 1) {
    SetError(error, "DDS must contain one two-dimensional image.");
    return false;
  }
  resolution = {ReadLe32(bytes, 16), ReadLe32(bytes, 12)};
  mip_levels = std::max(ReadLe32(bytes, 28), 1u);
  srgb = format == kDxgiBc7Srgb;
  if (resolution.x == 0 || resolution.y == 0) {
    SetError(error, "DDS has invalid dimensions.");
    return false;
  }
  return true;
}

size_t LevelByteSize(const glm::uvec2 resolution) {
  return static_cast<size_t>((resolution.x + 3) / 4) * ((resolution.y + 3) / 4) * kBc7BlockSize;
}

void InitializeEncoder() {
  static std::once_flag initialized;
  std::call_once(initialized, bc7enc_compress_block_init);
}
}  // namespace

bool DecodeDds(const std::filesystem::path& path, MipChain& result, std::string* error) {
  std::vector<std::byte> bytes;
  return ReadFile(path, bytes, error) && DecodeDds(bytes, result, error);
}

bool DecodeDds(const std::vector<std::byte>& bytes, MipChain& result, std::string* error) {
  glm::uvec2 resolution;
  uint32_t mip_levels = 0;
  bool srgb = false;
  if (!ParseHeader(bytes, resolution, mip_levels, srgb, error)) {
    return false;
  }

  size_t offset = kDdsHeaderSize;
  result.levels.clear();
  result.levels.reserve(mip_levels);
  auto level_resolution = resolution;
  for (uint32_t mip = 0; mip < mip_levels; ++mip) {
    const size_t level_size = LevelByteSize(level_resolution);
    if (offset + level_size > bytes.size()) {
      result.levels.clear();
      SetError(error, "DDS does not contain its declared BC7 mip chain.");
      return false;
    }
    MipLevel level;
    level.resolution = level_resolution;
    level.pixels.resize(static_cast<size_t>(level_resolution.x) * level_resolution.y);
    const uint32_t block_width = (level_resolution.x + 3) / 4;
    const uint32_t block_height = (level_resolution.y + 3) / 4;
    std::atomic_bool valid = true;
    Jobs::RunParallelFor(static_cast<size_t>(block_width) * block_height, [&](const size_t block_index) {
      const uint32_t block_x = static_cast<uint32_t>(block_index) % block_width;
      const uint32_t block_y = static_cast<uint32_t>(block_index) / block_width;
      std::array<bc7decomp::color_rgba, 16> decoded;
      if (!bc7decomp::unpack_bc7(bytes.data() + offset + block_index * kBc7BlockSize, decoded.data())) {
        valid = false;
        return;
      }
      for (uint32_t y = 0; y < 4; ++y) {
        for (uint32_t x = 0; x < 4; ++x) {
          const uint32_t target_x = block_x * 4 + x;
          const uint32_t target_y = block_y * 4 + y;
          if (target_x >= level_resolution.x || target_y >= level_resolution.y) {
            continue;
          }
          const auto& pixel = decoded[y * 4 + x];
          level.pixels[static_cast<size_t>(target_y) * level_resolution.x + target_x] =
              glm::vec4(pixel.r, pixel.g, pixel.b, pixel.a) / 255.0f;
        }
      }
    });
    if (!valid) {
      result.levels.clear();
      SetError(error, "DDS contains an invalid BC7 block.");
      return false;
    }
    result.levels.emplace_back(std::move(level));
    offset += level_size;
    level_resolution = glm::max(level_resolution / 2u, glm::uvec2(1));
  }
  result.content_hash = HashBytes(bytes);
  return true;
}

bool WriteDds(const std::filesystem::path& path, const std::vector<MipLevel>& levels, const bool srgb,
              const bool perceptual, std::string* error) {
  if (levels.empty() || levels.front().resolution.x == 0 || levels.front().resolution.y == 0) {
    SetError(error, "Cannot encode an empty BC7 mip chain.");
    return false;
  }
  InitializeEncoder();
  bc7enc_compress_block_params params;
  bc7enc_compress_block_params_init(&params);
  if (!perceptual) {
    bc7enc_compress_block_params_init_linear_weights(&params);
  }

  size_t total_size = kDdsHeaderSize;
  for (const auto& level : levels) {
    if (level.pixels.size() != static_cast<size_t>(level.resolution.x) * level.resolution.y) {
      SetError(error, "BC7 mip dimensions do not match their pixels.");
      return false;
    }
    total_size += LevelByteSize(level.resolution);
  }
  std::vector<std::byte> bytes(total_size);
  WriteLe32(bytes, 0, MakeFourCc('D', 'D', 'S', ' '));
  WriteLe32(bytes, 4, 124);
  WriteLe32(bytes, 8, 0x00021007u);
  WriteLe32(bytes, 12, levels.front().resolution.y);
  WriteLe32(bytes, 16, levels.front().resolution.x);
  WriteLe32(bytes, 20, static_cast<uint32_t>(LevelByteSize(levels.front().resolution)));
  WriteLe32(bytes, 28, static_cast<uint32_t>(levels.size()));
  WriteLe32(bytes, 76, 32);
  WriteLe32(bytes, 80, 0x4u);
  WriteLe32(bytes, 84, MakeFourCc('D', 'X', '1', '0'));
  WriteLe32(bytes, 108, levels.size() > 1 ? 0x00401008u : 0x1000u);
  WriteLe32(bytes, 128, srgb ? kDxgiBc7Srgb : kDxgiBc7Unorm);
  WriteLe32(bytes, 132, 3);
  WriteLe32(bytes, 140, 1);

  size_t offset = kDdsHeaderSize;
  for (const auto& level : levels) {
    const uint32_t block_width = (level.resolution.x + 3) / 4;
    const uint32_t block_height = (level.resolution.y + 3) / 4;
    Jobs::RunParallelFor(static_cast<size_t>(block_width) * block_height, [&](const size_t block_index) {
      const uint32_t block_x = static_cast<uint32_t>(block_index) % block_width;
      const uint32_t block_y = static_cast<uint32_t>(block_index) / block_width;
      std::array<uint8_t, 64> block{};
      for (uint32_t y = 0; y < 4; ++y) {
        for (uint32_t x = 0; x < 4; ++x) {
          const uint32_t source_x = std::min(block_x * 4 + x, level.resolution.x - 1);
          const uint32_t source_y = std::min(block_y * 4 + y, level.resolution.y - 1);
          const auto pixel = glm::clamp(level.pixels[static_cast<size_t>(source_y) * level.resolution.x + source_x],
                                        glm::vec4(0.0f), glm::vec4(1.0f));
          for (uint32_t channel = 0; channel < 4; ++channel) {
            block[(y * 4 + x) * 4 + channel] =
                static_cast<uint8_t>(glm::clamp(glm::round(pixel[channel] * 255.0f), 0.0f, 255.0f));
          }
        }
      }
      bc7enc_compress_block(bytes.data() + offset + block_index * kBc7BlockSize, block.data(), &params);
    });
    offset += LevelByteSize(level.resolution);
  }

  std::ofstream stream(path, std::ios::binary | std::ios::trunc);
  if (!stream.write(reinterpret_cast<const char*>(bytes.data()), static_cast<std::streamsize>(bytes.size()))) {
    SetError(error, "BC7 DDS could not be written.");
    return false;
  }
  return true;
}

bool ValidateDds(const std::filesystem::path& path, const glm::uvec2 expected_resolution, const bool expected_srgb,
                 std::string* error) {
  std::vector<std::byte> bytes;
  glm::uvec2 resolution;
  uint32_t mip_levels = 0;
  bool srgb = false;
  if (!ReadFile(path, bytes, error) || !ParseHeader(bytes, resolution, mip_levels, srgb, error)) {
    return false;
  }
  size_t expected_size = kDdsHeaderSize;
  auto level_resolution = resolution;
  uint32_t expected_mip_levels = 1;
  for (uint32_t dimension = glm::max(resolution.x, resolution.y); dimension > 1; dimension /= 2) {
    ++expected_mip_levels;
  }
  for (uint32_t mip = 0; mip < mip_levels; ++mip) {
    expected_size += LevelByteSize(level_resolution);
    level_resolution = glm::max(level_resolution / 2u, glm::uvec2(1));
  }
  if (resolution != expected_resolution || srgb != expected_srgb || mip_levels != expected_mip_levels ||
      bytes.size() != expected_size) {
    SetError(error, "BC7 DDS cache entry has incompatible metadata.");
    return false;
  }
  return true;
}

}  // namespace evo_engine::bc7_texture_codec
