#include "ReflectionProbePack.hpp"

#include "AssetManager.hpp"
#include "Console.hpp"
#include "Serialization.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstring>
#include <fstream>
#include <limits>
#include <unordered_set>

#ifdef _WIN32
#  include <Windows.h>
#endif

using namespace evo_engine;

namespace {
constexpr std::array<uint8_t, 8> kMagic = {'E', 'V', 'R', 'P', 'P', 'K', '1', 0};
constexpr size_t kHeaderSize = 28u;
constexpr size_t kEntrySize = 152u;
constexpr uint32_t kValidPayloadFlag = 1u << 0u;
constexpr uint32_t kBoxProjectionFlag = 1u << 1u;
constexpr uint32_t kEnabledFlag = 1u << 2u;
constexpr uint32_t kDebugDrawFlag = 1u << 3u;
constexpr size_t kMaximumNameBytes = 1024u * 1024u;
std::atomic<uint64_t> temporary_file_counter = 0u;

struct StagedProbe {
  ReflectionProbePack::Probe probe;
  GlobalReflectionProbe::SourceKind source_kind = GlobalReflectionProbe::SourceKind::Empty;
  uint64_t payload_hash = 0u;
  std::vector<uint16_t> payload;
};

class ReflectionProbePackStagedLoadPayload final : public StagedAssetLoadPayload {
 public:
  std::vector<StagedProbe> probes;
};

void AppendU32(std::vector<uint8_t>& bytes, const uint32_t value) {
  for (uint32_t shift = 0u; shift < 32u; shift += 8u)
    bytes.emplace_back(static_cast<uint8_t>(value >> shift));
}

void AppendU64(std::vector<uint8_t>& bytes, const uint64_t value) {
  for (uint32_t shift = 0u; shift < 64u; shift += 8u)
    bytes.emplace_back(static_cast<uint8_t>(value >> shift));
}

void AppendF32(std::vector<uint8_t>& bytes, const float value) {
  uint32_t bits = 0u;
  std::memcpy(&bits, &value, sizeof(bits));
  AppendU32(bytes, bits);
}

uint32_t ReadU32(const std::vector<uint8_t>& bytes, size_t& cursor) {
  if (cursor > bytes.size() || bytes.size() - cursor < sizeof(uint32_t))
    throw std::invalid_argument("Reflection probe pack is truncated.");
  uint32_t value = 0u;
  for (uint32_t shift = 0u; shift < 32u; shift += 8u)
    value |= static_cast<uint32_t>(bytes[cursor++]) << shift;
  return value;
}

uint64_t ReadU64(const std::vector<uint8_t>& bytes, size_t& cursor) {
  if (cursor > bytes.size() || bytes.size() - cursor < sizeof(uint64_t))
    throw std::invalid_argument("Reflection probe pack is truncated.");
  uint64_t value = 0u;
  for (uint32_t shift = 0u; shift < 64u; shift += 8u)
    value |= static_cast<uint64_t>(bytes[cursor++]) << shift;
  return value;
}

float ReadF32(const std::vector<uint8_t>& bytes, size_t& cursor) {
  const uint32_t bits = ReadU32(bytes, cursor);
  float value = 0.0f;
  std::memcpy(&value, &bits, sizeof(value));
  return value;
}

bool IsFinite(const ReflectionProbePack::Probe& probe) {
  for (glm::length_t column = 0; column < 4; ++column) {
    for (glm::length_t row = 0; row < 4; ++row) {
      if (!std::isfinite(probe.transform[column][row]))
        return false;
    }
  }
  return std::isfinite(probe.box_projection_extents.x) && std::isfinite(probe.box_projection_extents.y) &&
         std::isfinite(probe.box_projection_extents.z) && std::isfinite(probe.sphere_radius) &&
         std::isfinite(probe.blend_distance) && std::isfinite(probe.reflection_intensity);
}

bool RangeValid(const uint64_t offset, const uint64_t size, const size_t total_size) {
  return offset <= total_size && size <= total_size - static_cast<size_t>(offset);
}

void CheckedAdd(uint64_t& value, const uint64_t increment) {
  if (increment > std::numeric_limits<uint64_t>::max() - value)
    throw std::overflow_error("Reflection probe pack size overflowed.");
  value += increment;
}

bool ReplaceFile(const std::filesystem::path& temporary_path, const std::filesystem::path& path) {
#ifdef _WIN32
  return MoveFileExW(temporary_path.c_str(), path.c_str(), MOVEFILE_REPLACE_EXISTING | MOVEFILE_WRITE_THROUGH) != FALSE;
#else
  std::error_code error;
  std::filesystem::rename(temporary_path, path, error);
  return !error;
#endif
}

std::shared_ptr<ReflectionProbePackStagedLoadPayload> Decode(const std::filesystem::path& path) {
  try {
    std::ifstream stream(path, std::ios::binary | std::ios::ate);
    if (!stream)
      return {};
    const auto end = stream.tellg();
    if (end < static_cast<std::streamoff>(kHeaderSize))
      throw std::invalid_argument("Reflection probe pack is empty or truncated.");
    std::vector<uint8_t> bytes(static_cast<size_t>(end));
    stream.seekg(0);
    stream.read(reinterpret_cast<char*>(bytes.data()), static_cast<std::streamsize>(bytes.size()));
    if (!stream)
      throw std::invalid_argument("Reflection probe pack could not be read completely.");
    if (!std::equal(kMagic.begin(), kMagic.end(), bytes.begin()))
      throw std::invalid_argument("Reflection probe pack magic is invalid.");

    size_t cursor = kMagic.size();
    const auto version = ReadU32(bytes, cursor);
    const auto format = ReadU32(bytes, cursor);
    const auto resolution = ReadU32(bytes, cursor);
    const auto mip_levels = ReadU32(bytes, cursor);
    const auto entry_count = ReadU32(bytes, cursor);
    if (version != ReflectionProbePack::kSchemaVersion ||
        format != static_cast<uint32_t>(GlobalReflectionProbe::kCanonicalFormat) ||
        resolution != GlobalReflectionProbe::kResolution || mip_levels != GlobalReflectionProbe::kMipLevels)
      throw std::invalid_argument("Reflection probe pack header is not canonical.");
    if (entry_count > (bytes.size() - kHeaderSize) / kEntrySize)
      throw std::invalid_argument("Reflection probe pack entry count is invalid.");

    auto staged = std::make_shared<ReflectionProbePackStagedLoadPayload>();
    staged->probes.reserve(entry_count);
    std::unordered_set<uint64_t> stable_ids;
    const uint64_t data_start = kHeaderSize + static_cast<uint64_t>(entry_count) * kEntrySize;
    std::vector<std::pair<uint64_t, uint64_t>> data_ranges;
    data_ranges.reserve(static_cast<size_t>(entry_count) * 2u);
    for (uint32_t index = 0u; index < entry_count; ++index) {
      auto& staged_probe = staged->probes.emplace_back();
      auto& probe = staged_probe.probe;
      probe.stable_id = ReadU64(bytes, cursor);
      const uint64_t name_offset = ReadU64(bytes, cursor);
      const uint64_t name_size = ReadU64(bytes, cursor);
      const uint64_t payload_offset = ReadU64(bytes, cursor);
      const uint64_t payload_size = ReadU64(bytes, cursor);
      staged_probe.payload_hash = ReadU64(bytes, cursor);
      const auto source_kind = ReadU32(bytes, cursor);
      const auto flags = ReadU32(bytes, cursor);
      for (glm::length_t column = 0; column < 4; ++column)
        for (glm::length_t row = 0; row < 4; ++row)
          probe.transform[column][row] = ReadF32(bytes, cursor);
      probe.box_projection_extents = {ReadF32(bytes, cursor), ReadF32(bytes, cursor), ReadF32(bytes, cursor)};
      probe.sphere_radius = ReadF32(bytes, cursor);
      probe.blend_distance = ReadF32(bytes, cursor);
      probe.reflection_intensity = ReadF32(bytes, cursor);
      probe.artist_priority = static_cast<int32_t>(ReadU32(bytes, cursor));
      probe.shape = static_cast<int32_t>(ReadU32(bytes, cursor));
      probe.box_projection = (flags & kBoxProjectionFlag) != 0u;
      probe.enabled = (flags & kEnabledFlag) != 0u;
      probe.debug_draw_bounds = (flags & kDebugDrawFlag) != 0u;

      if (probe.stable_id == 0u || !stable_ids.emplace(probe.stable_id).second || !IsFinite(probe) ||
          name_offset < data_start || name_size > kMaximumNameBytes ||
          !RangeValid(name_offset, name_size, bytes.size()))
        throw std::invalid_argument("Reflection probe pack entry metadata is invalid.");
      probe.name.assign(reinterpret_cast<const char*>(bytes.data() + name_offset), static_cast<size_t>(name_size));
      if (name_size != 0u)
        data_ranges.emplace_back(name_offset, name_size);

      const bool valid = (flags & kValidPayloadFlag) != 0u;
      if (!valid) {
        if (source_kind != static_cast<uint32_t>(GlobalReflectionProbe::SourceKind::Empty) || payload_size != 0u ||
            staged_probe.payload_hash != 0u)
          throw std::invalid_argument("Reflection probe pack invalid payload marker is inconsistent.");
        continue;
      }
      if (source_kind == static_cast<uint32_t>(GlobalReflectionProbe::SourceKind::Empty) ||
          source_kind > static_cast<uint32_t>(GlobalReflectionProbe::SourceKind::Baked) ||
          payload_size != GlobalReflectionProbe::kCanonicalPayloadByteSize || payload_offset < data_start ||
          !RangeValid(payload_offset, payload_size, bytes.size()))
        throw std::invalid_argument("Reflection probe pack payload metadata is invalid.");
      staged_probe.source_kind = static_cast<GlobalReflectionProbe::SourceKind>(source_kind);
      data_ranges.emplace_back(payload_offset, payload_size);
      staged_probe.payload.resize(GlobalReflectionProbe::kCanonicalTexelCount * 4u);
      size_t byte_cursor = static_cast<size_t>(payload_offset);
      for (auto& value : staged_probe.payload) {
        value = static_cast<uint16_t>(bytes[byte_cursor]) |
                static_cast<uint16_t>(static_cast<uint16_t>(bytes[byte_cursor + 1u]) << 8u);
        byte_cursor += 2u;
      }
      std::string error;
      if (!GlobalReflectionProbe::ValidateCanonicalPayload(staged_probe.payload, error) ||
          GlobalReflectionProbe::CalculatePayloadHash(staged_probe.payload) != staged_probe.payload_hash)
        throw std::invalid_argument("Reflection probe pack payload validation failed.");
    }
    if (cursor != data_start)
      throw std::invalid_argument("Reflection probe pack entry table size is invalid.");
    std::sort(data_ranges.begin(), data_ranges.end());
    uint64_t expected_offset = data_start;
    for (const auto& [offset, size] : data_ranges) {
      if (offset != expected_offset)
        throw std::invalid_argument("Reflection probe pack data ranges overlap or contain gaps.");
      expected_offset += size;
    }
    if (expected_offset != bytes.size())
      throw std::invalid_argument("Reflection probe pack contains trailing data.");
    return staged;
  } catch (const std::exception& exception) {
    EVOENGINE_ERROR("Failed to load reflection probe pack: " + std::string(exception.what()))
    return {};
  }
}
}  // namespace

bool ReflectionProbePack::Probe::HasValidPayload() const {
  return payload && payload->GetSourceKind() != GlobalReflectionProbe::SourceKind::Empty;
}

std::shared_ptr<GlobalReflectionProbe> ReflectionProbePack::Probe::GetOrCreatePayload() {
  if (!payload)
    payload = AssetManager::CreateTemporaryAsset<GlobalReflectionProbe>();
  return payload;
}

bool ReflectionProbePack::RepairStableIds() {
  std::unordered_set<uint64_t> used;
  bool changed = false;
  uint64_t candidate = 1u;
  for (auto& probe : probes) {
    if (probe.stable_id != 0u && used.emplace(probe.stable_id).second)
      continue;
    while (used.find(candidate) != used.end())
      ++candidate;
    probe.stable_id = candidate++;
    used.emplace(probe.stable_id);
    changed = true;
  }
  return changed;
}

ReflectionProbePack::Probe* ReflectionProbePack::FindProbe(const uint64_t stable_id) {
  const auto found = std::find_if(probes.begin(), probes.end(), [stable_id](const auto& probe) {
    return probe.stable_id == stable_id;
  });
  return found == probes.end() ? nullptr : &*found;
}

const ReflectionProbePack::Probe* ReflectionProbePack::FindProbe(const uint64_t stable_id) const {
  const auto found = std::find_if(probes.begin(), probes.end(), [stable_id](const auto& probe) {
    return probe.stable_id == stable_id;
  });
  return found == probes.end() ? nullptr : &*found;
}

bool ReflectionProbePack::SaveInternal(const std::filesystem::path& path) const {
  try {
    if (probes.size() > std::numeric_limits<uint32_t>::max())
      throw std::invalid_argument("Reflection probe pack has too many entries.");
    std::unordered_set<uint64_t> stable_ids;
    std::vector<std::vector<uint16_t>> payloads(probes.size());
    uint64_t data_offset = kHeaderSize + static_cast<uint64_t>(probes.size()) * kEntrySize;
    std::vector<uint64_t> name_offsets(probes.size());
    for (size_t index = 0; index < probes.size(); ++index) {
      const auto& probe = probes[index];
      if (probe.stable_id == 0u || !stable_ids.emplace(probe.stable_id).second || !IsFinite(probe) ||
          probe.name.size() > kMaximumNameBytes)
        throw std::invalid_argument("Reflection probe pack entry metadata is invalid.");
      name_offsets[index] = data_offset;
      CheckedAdd(data_offset, probe.name.size());
    }
    std::vector<uint64_t> payload_offsets(probes.size());
    for (size_t index = 0; index < probes.size(); ++index) {
      if (!probes[index].HasValidPayload())
        continue;
      if (!probes[index].payload->ReadCanonicalPayload(payloads[index]))
        throw std::runtime_error("Reflection probe pack payload readback failed.");
      std::string error;
      if (!GlobalReflectionProbe::ValidateCanonicalPayload(payloads[index], error))
        throw std::invalid_argument(error);
      payload_offsets[index] = data_offset;
      CheckedAdd(data_offset, GlobalReflectionProbe::kCanonicalPayloadByteSize);
    }
    if (data_offset > std::numeric_limits<size_t>::max())
      throw std::overflow_error("Reflection probe pack is too large.");

    std::vector<uint8_t> bytes;
    bytes.reserve(static_cast<size_t>(data_offset));
    bytes.insert(bytes.end(), kMagic.begin(), kMagic.end());
    AppendU32(bytes, kSchemaVersion);
    AppendU32(bytes, static_cast<uint32_t>(GlobalReflectionProbe::kCanonicalFormat));
    AppendU32(bytes, GlobalReflectionProbe::kResolution);
    AppendU32(bytes, GlobalReflectionProbe::kMipLevels);
    AppendU32(bytes, static_cast<uint32_t>(probes.size()));
    for (size_t index = 0; index < probes.size(); ++index) {
      const auto& probe = probes[index];
      const bool valid = !payloads[index].empty();
      AppendU64(bytes, probe.stable_id);
      AppendU64(bytes, name_offsets[index]);
      AppendU64(bytes, probe.name.size());
      AppendU64(bytes, payload_offsets[index]);
      AppendU64(bytes, valid ? GlobalReflectionProbe::kCanonicalPayloadByteSize : 0u);
      AppendU64(bytes, valid ? GlobalReflectionProbe::CalculatePayloadHash(payloads[index]) : 0u);
      AppendU32(bytes, valid ? static_cast<uint32_t>(probe.payload->GetSourceKind())
                             : static_cast<uint32_t>(GlobalReflectionProbe::SourceKind::Empty));
      AppendU32(bytes, (valid ? kValidPayloadFlag : 0u) | (probe.box_projection ? kBoxProjectionFlag : 0u) |
                           (probe.enabled ? kEnabledFlag : 0u) | (probe.debug_draw_bounds ? kDebugDrawFlag : 0u));
      for (glm::length_t column = 0; column < 4; ++column)
        for (glm::length_t row = 0; row < 4; ++row)
          AppendF32(bytes, probe.transform[column][row]);
      AppendF32(bytes, probe.box_projection_extents.x);
      AppendF32(bytes, probe.box_projection_extents.y);
      AppendF32(bytes, probe.box_projection_extents.z);
      AppendF32(bytes, probe.sphere_radius);
      AppendF32(bytes, probe.blend_distance);
      AppendF32(bytes, probe.reflection_intensity);
      AppendU32(bytes, static_cast<uint32_t>(probe.artist_priority));
      AppendU32(bytes, static_cast<uint32_t>(probe.shape));
    }
    for (const auto& probe : probes)
      bytes.insert(bytes.end(), probe.name.begin(), probe.name.end());
    for (const auto& payload : payloads) {
      for (const auto value : payload) {
        bytes.emplace_back(static_cast<uint8_t>(value));
        bytes.emplace_back(static_cast<uint8_t>(value >> 8u));
      }
    }
    if (bytes.size() != static_cast<size_t>(data_offset))
      throw std::runtime_error("Reflection probe pack size calculation is inconsistent.");

    auto temporary_path = path;
    temporary_path += ".tmp." + std::to_string(temporary_file_counter.fetch_add(1u)) + "." +
                      std::to_string(std::chrono::steady_clock::now().time_since_epoch().count());
    std::ofstream stream(temporary_path, std::ios::binary | std::ios::trunc);
    stream.write(reinterpret_cast<const char*>(bytes.data()), static_cast<std::streamsize>(bytes.size()));
    stream.flush();
    if (!stream) {
      stream.close();
      std::filesystem::remove(temporary_path);
      return false;
    }
    stream.close();
    if (ReplaceFile(temporary_path, path))
      return true;
    std::filesystem::remove(temporary_path);
  } catch (const std::exception& exception) {
    EVOENGINE_ERROR("Failed to save reflection probe pack: " + std::string(exception.what()))
  }
  return false;
}

bool ReflectionProbePack::LoadInternal(const std::filesystem::path& path) {
  return ApplyStagedPayloadInternal(path, LoadStagedPayloadInternal(path));
}

std::shared_ptr<StagedAssetLoadPayload> ReflectionProbePack::LoadStagedPayloadInternal(
    const std::filesystem::path& path) const {
  return Decode(path);
}

bool ReflectionProbePack::ApplyStagedPayloadInternal(const std::filesystem::path&,
                                                     const std::shared_ptr<StagedAssetLoadPayload>& payload) {
  const auto staged = std::dynamic_pointer_cast<ReflectionProbePackStagedLoadPayload>(payload);
  if (!staged)
    return false;
  std::vector<Probe> loaded;
  loaded.reserve(staged->probes.size());
  for (const auto& staged_probe : staged->probes) {
    auto probe = staged_probe.probe;
    if (!staged_probe.payload.empty()) {
      probe.payload = AssetManager::CreateTemporaryAsset<GlobalReflectionProbe>();
      if (!probe.payload || !probe.payload->SetCanonicalPayload(staged_probe.payload))
        return false;
      if (staged_probe.source_kind == GlobalReflectionProbe::SourceKind::Baked)
        probe.payload->MarkBaked();
    }
    loaded.emplace_back(std::move(probe));
  }
  probes = std::move(loaded);
  return true;
}

bool ReflectionProbePack::RegisterAssetIoHandlers(const std::string& owner_name, const std::string& type_name) {
  return Serialization::RegisterAssetIoHandler<ReflectionProbePack>(
      [](const ReflectionProbePack& asset, const std::filesystem::path& path) {
        return asset.SaveInternal(path);
      },
      [](ReflectionProbePack& asset, const std::filesystem::path& path) {
        return asset.LoadInternal(path);
      },
      [](const ReflectionProbePack&, const std::filesystem::path&) {
        return true;
      },
      [](const ReflectionProbePack& asset, const std::filesystem::path& path) {
        return asset.LoadStagedPayloadInternal(path);
      },
      [](ReflectionProbePack& asset, const std::filesystem::path& path,
         const std::shared_ptr<StagedAssetLoadPayload>& payload) {
        return asset.ApplyStagedPayloadInternal(path, payload);
      },
      owner_name, type_name);
}

void evo_engine::SerializeReflectionProbePack(YAML::Emitter& out, const ReflectionProbePack&) {
  out << YAML::Key << "binary_schema_version" << YAML::Value << ReflectionProbePack::kSchemaVersion;
}

void evo_engine::DeserializeReflectionProbePack(const YAML::Node&, ReflectionProbePack& pack) {
  pack.probes.clear();
}
