#include "GaussianSplat.hpp"

#include "ApplicationContext.hpp"
#include "GraphicsResources.hpp"
#include "Platform.hpp"
#include "Serialization.hpp"
#include "Tinyply.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <cstring>
#include <fstream>
#include <iostream>
#include <limits>
#include <sstream>
#include <system_error>
#include <unordered_map>
#include <unordered_set>

using namespace evo_engine;
using namespace tinyply;

namespace {
class GaussianSplatStagedLoadPayload final : public StagedAssetLoadPayload {
 public:
  bool yaml = false;
  YAML::Node yaml_node;
  std::vector<glm::vec3> positions;
  std::vector<glm::vec3> scales;
  std::vector<glm::vec4> rotations;
  std::vector<float> opacities;
  std::vector<glm::vec3> colors;
  std::vector<float> spherical_harmonics_rest;
  uint32_t spherical_harmonics_rest_float_count = 0;
};

[[nodiscard]] float GetOrDefault(const std::vector<float>& values, const size_t index, const float fallback) {
  return index < values.size() ? values[index] : fallback;
}

[[nodiscard]] glm::vec3 GetOrDefault(const std::vector<glm::vec3>& values, const size_t index,
                                     const glm::vec3& fallback) {
  return index < values.size() ? values[index] : fallback;
}

[[nodiscard]] glm::vec4 GetOrDefault(const std::vector<glm::vec4>& values, const size_t index,
                                     const glm::vec4& fallback) {
  return index < values.size() ? values[index] : fallback;
}

[[nodiscard]] bool SameMatrix(const glm::mat4& lhs, const glm::mat4& rhs) {
  return std::memcmp(&lhs[0][0], &rhs[0][0], sizeof(glm::mat4)) == 0;
}

[[nodiscard]] Handle CombineSortCacheHandles(const Handle& camera_handle, const Handle& sort_owner_handle) {
  auto value = camera_handle.GetValue() + 0x9e3779b97f4a7c15ull;
  value ^= sort_owner_handle.GetValue() + 0x9e3779b97f4a7c15ull + (value << 6) + (value >> 2);
  return Handle(value);
}

[[nodiscard]] std::string LowercaseExtension(const std::filesystem::path& path) {
  auto extension = path.extension().string();
  std::transform(extension.begin(), extension.end(), extension.begin(), [](const unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });
  return extension;
}

[[nodiscard]] bool CanUploadGpuData() {
  return ApplicationContext::TryGet() && Platform::Initialized();
}

[[nodiscard]] std::shared_ptr<evo_engine::Buffer> CreateStorageBuffer(const size_t byte_size) {
  VkBufferCreateInfo buffer_create_info{};
  buffer_create_info.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
  buffer_create_info.size = glm::max(static_cast<size_t>(1), byte_size);
  buffer_create_info.usage = VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_STORAGE_BUFFER_BIT;
  buffer_create_info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;

  VmaAllocationCreateInfo allocation_create_info{};
  allocation_create_info.usage = VMA_MEMORY_USAGE_AUTO_PREFER_DEVICE;
  return std::make_shared<evo_engine::Buffer>(buffer_create_info, allocation_create_info);
}

template <typename T>
void UploadVector(std::shared_ptr<evo_engine::Buffer>& buffer, const std::vector<T>& data) {
  if (!CanUploadGpuData() || data.empty()) {
    return;
  }

  const auto byte_size = data.size() * sizeof(T);
  if (!buffer || buffer->GetSize() < byte_size) {
    buffer = CreateStorageBuffer(byte_size);
  }
  buffer->UploadVector(data);
}

bool HasVertexProperties(const PlyFile& file, const std::vector<std::string>& property_names) {
  std::unordered_set<std::string> vertex_properties;
  for (const auto& element : file.get_elements()) {
    if (element.name != "vertex") {
      continue;
    }
    for (const auto& property : element.properties) {
      vertex_properties.emplace(property.name);
    }
    break;
  }
  for (const auto& property_name : property_names) {
    if (vertex_properties.find(property_name) == vertex_properties.end()) {
      return false;
    }
  }
  return true;
}

std::vector<std::string> CollectRestPropertyNames(const PlyFile& file) {
  std::unordered_set<std::string> vertex_properties;
  for (const auto& element : file.get_elements()) {
    if (element.name != "vertex") {
      continue;
    }
    for (const auto& property : element.properties) {
      vertex_properties.emplace(property.name);
    }
    break;
  }

  std::vector<std::string> rest_properties;
  for (uint32_t index = 0;; ++index) {
    const auto property_name = "f_rest_" + std::to_string(index);
    if (vertex_properties.find(property_name) == vertex_properties.end()) {
      break;
    }
    rest_properties.emplace_back(property_name);
  }
  return rest_properties;
}

enum class GaussianSplatField {
  Ignore,
  PositionX,
  PositionY,
  PositionZ,
  ScaleX,
  ScaleY,
  ScaleZ,
  ColorX,
  ColorY,
  ColorZ,
  Opacity,
  RotationX,
  RotationY,
  RotationZ,
  RotationW,
  SphericalHarmonicsRest
};

struct GaussianSplatPropertyReader {
  GaussianSplatField field = GaussianSplatField::Ignore;
  Type type = Type::INVALID;
  Type list_type = Type::INVALID;
  bool list = false;
  uint32_t rest_index = 0;
};

[[nodiscard]] const PlyElement* FindElement(const std::vector<PlyElement>& elements, const std::string& name) {
  for (const auto& element : elements) {
    if (element.name == name) {
      return &element;
    }
  }
  return nullptr;
}

[[nodiscard]] size_t TypeStride(const Type type) {
  switch (type) {
    case Type::INT8:
    case Type::UINT8:
      return 1;
    case Type::INT16:
    case Type::UINT16:
      return 2;
    case Type::INT32:
    case Type::UINT32:
    case Type::FLOAT32:
      return 4;
    case Type::FLOAT64:
      return 8;
    default:
      return 0;
  }
}

[[nodiscard]] bool IsFloatProperty(const Type type) {
  return type == Type::FLOAT32 || type == Type::FLOAT64;
}

template <typename T>
[[nodiscard]] bool ReadBinaryValue(std::istream& stream, T& value) {
  stream.read(reinterpret_cast<char*>(&value), sizeof(T));
  return static_cast<bool>(stream);
}

template <typename T>
bool WriteBinaryValue(std::ostream& stream, const T& value) {
  stream.write(reinterpret_cast<const char*>(&value), sizeof(T));
  return static_cast<bool>(stream);
}

[[nodiscard]] uint8_t FloatToByte(const float value) {
  return static_cast<uint8_t>(std::clamp(std::lround(std::clamp(value, 0.0f, 1.0f) * 255.0f), 0l, 255l));
}

[[nodiscard]] uint8_t QuaternionToByte(const float value) {
  return static_cast<uint8_t>(std::clamp(std::lround(std::clamp(value, -1.0f, 1.0f) * 128.0f + 128.0f), 0l, 255l));
}

[[nodiscard]] glm::vec4 NormalizeRotation(const glm::vec4& rotation) {
  const auto length = glm::length(rotation);
  if (length <= std::numeric_limits<float>::epsilon()) {
    return glm::vec4(1.0f, 0.0f, 0.0f, 0.0f);
  }
  return rotation / length;
}

void WarnDropsSphericalHarmonicsRest(const char* format, const uint32_t rest_count,
                                     const std::vector<float>& spherical_harmonics_rest) {
  if (rest_count == 0 || spherical_harmonics_rest.empty()) {
    return;
  }
  std::cerr << "Saving Gaussian splats as " << format << " drops spherical_harmonics_rest." << std::endl;
}

[[nodiscard]] bool ReadBinaryScalar(std::istream& stream, const Type type, float& value) {
  switch (type) {
    case Type::INT8: {
      int8_t v;
      if (!ReadBinaryValue(stream, v)) {
        return false;
      }
      value = static_cast<float>(v);
      return true;
    }
    case Type::UINT8: {
      uint8_t v;
      if (!ReadBinaryValue(stream, v)) {
        return false;
      }
      value = static_cast<float>(v);
      return true;
    }
    case Type::INT16: {
      int16_t v;
      if (!ReadBinaryValue(stream, v)) {
        return false;
      }
      value = static_cast<float>(v);
      return true;
    }
    case Type::UINT16: {
      uint16_t v;
      if (!ReadBinaryValue(stream, v)) {
        return false;
      }
      value = static_cast<float>(v);
      return true;
    }
    case Type::INT32: {
      int32_t v;
      if (!ReadBinaryValue(stream, v)) {
        return false;
      }
      value = static_cast<float>(v);
      return true;
    }
    case Type::UINT32: {
      uint32_t v;
      if (!ReadBinaryValue(stream, v)) {
        return false;
      }
      value = static_cast<float>(v);
      return true;
    }
    case Type::FLOAT32:
      return ReadBinaryValue(stream, value);
    case Type::FLOAT64: {
      double v;
      if (!ReadBinaryValue(stream, v)) {
        return false;
      }
      value = static_cast<float>(v);
      return true;
    }
    default:
      return false;
  }
}

[[nodiscard]] bool ReadAsciiScalar(std::istream& stream, const Type type, float& value) {
  if (type == Type::INVALID) {
    return false;
  }
  double scalar = 0.0;
  stream >> scalar;
  if (!stream) {
    return false;
  }
  value = static_cast<float>(scalar);
  return true;
}

[[nodiscard]] bool ReadScalar(std::istream& stream, const bool binary, const Type type, float& value) {
  return binary ? ReadBinaryScalar(stream, type, value) : ReadAsciiScalar(stream, type, value);
}

[[nodiscard]] bool ReadListCount(std::istream& stream, const bool binary, const Type type, uint64_t& count) {
  float scalar = 0.0f;
  if (!ReadScalar(stream, binary, type, scalar) || scalar < 0.0f) {
    return false;
  }
  count = static_cast<uint64_t>(scalar);
  return true;
}

[[nodiscard]] bool SkipProperty(std::istream& stream, const bool binary, const Type type, const bool is_list,
                                const Type list_type) {
  if (!is_list) {
    if (binary) {
      const auto stride = TypeStride(type);
      if (stride == 0) {
        return false;
      }
      stream.ignore(static_cast<std::streamsize>(stride));
      return static_cast<bool>(stream);
    }
    std::string ignored;
    stream >> ignored;
    return static_cast<bool>(stream);
  }

  uint64_t count = 0;
  if (!ReadListCount(stream, binary, list_type, count)) {
    return false;
  }
  if (binary) {
    const auto stride = TypeStride(type);
    if (stride == 0) {
      return false;
    }
    stream.ignore(static_cast<std::streamsize>(count * stride));
    return static_cast<bool>(stream);
  }
  for (uint64_t i = 0; i < count; ++i) {
    std::string ignored;
    stream >> ignored;
    if (!stream) {
      return false;
    }
  }
  return true;
}

[[nodiscard]] bool SkipProperty(std::istream& stream, const bool binary, const PlyProperty& property) {
  return SkipProperty(stream, binary, property.propertyType, property.isList, property.listType);
}

void ApplyGaussianSplatProperty(GaussianSplatStagedLoadPayload& payload, const size_t splat_index,
                                const GaussianSplatPropertyReader& reader, const float value) {
  switch (reader.field) {
    case GaussianSplatField::PositionX:
      payload.positions[splat_index].x = value;
      break;
    case GaussianSplatField::PositionY:
      payload.positions[splat_index].y = value;
      break;
    case GaussianSplatField::PositionZ:
      payload.positions[splat_index].z = value;
      break;
    case GaussianSplatField::ScaleX:
      payload.scales[splat_index].x = value;
      break;
    case GaussianSplatField::ScaleY:
      payload.scales[splat_index].y = value;
      break;
    case GaussianSplatField::ScaleZ:
      payload.scales[splat_index].z = value;
      break;
    case GaussianSplatField::ColorX:
      payload.colors[splat_index].x = value;
      break;
    case GaussianSplatField::ColorY:
      payload.colors[splat_index].y = value;
      break;
    case GaussianSplatField::ColorZ:
      payload.colors[splat_index].z = value;
      break;
    case GaussianSplatField::Opacity:
      payload.opacities[splat_index] = value;
      break;
    case GaussianSplatField::RotationX:
      payload.rotations[splat_index].x = value;
      break;
    case GaussianSplatField::RotationY:
      payload.rotations[splat_index].y = value;
      break;
    case GaussianSplatField::RotationZ:
      payload.rotations[splat_index].z = value;
      break;
    case GaussianSplatField::RotationW:
      payload.rotations[splat_index].w = value;
      break;
    case GaussianSplatField::SphericalHarmonicsRest:
      payload.spherical_harmonics_rest[splat_index * payload.spherical_harmonics_rest_float_count + reader.rest_index] =
          value;
      break;
    default:
      break;
  }
}

[[nodiscard]] bool BuildVertexReaders(const PlyElement& vertex_element, const std::vector<std::string>& rest_properties,
                                      std::vector<GaussianSplatPropertyReader>& readers) {
  const std::unordered_map<std::string, GaussianSplatField> fields = {
      {"x", GaussianSplatField::PositionX},     {"y", GaussianSplatField::PositionY},
      {"z", GaussianSplatField::PositionZ},     {"scale_0", GaussianSplatField::ScaleX},
      {"scale_1", GaussianSplatField::ScaleY},  {"scale_2", GaussianSplatField::ScaleZ},
      {"f_dc_0", GaussianSplatField::ColorX},   {"f_dc_1", GaussianSplatField::ColorY},
      {"f_dc_2", GaussianSplatField::ColorZ},   {"opacity", GaussianSplatField::Opacity},
      {"rot_0", GaussianSplatField::RotationX}, {"rot_1", GaussianSplatField::RotationY},
      {"rot_2", GaussianSplatField::RotationZ}, {"rot_3", GaussianSplatField::RotationW}};
  std::unordered_map<std::string, uint32_t> rest_indices;
  for (uint32_t i = 0; i < rest_properties.size(); ++i) {
    rest_indices[rest_properties[i]] = i;
  }

  readers.clear();
  readers.reserve(vertex_element.properties.size());
  for (const auto& property : vertex_element.properties) {
    GaussianSplatPropertyReader reader;
    reader.type = property.propertyType;
    reader.list_type = property.listType;
    reader.list = property.isList;
    if (const auto search = fields.find(property.name); search != fields.end()) {
      if (property.isList || !IsFloatProperty(property.propertyType)) {
        return false;
      }
      reader.field = search->second;
    } else if (const auto rest_search = rest_indices.find(property.name); rest_search != rest_indices.end()) {
      if (property.isList || !IsFloatProperty(property.propertyType)) {
        return false;
      }
      reader.field = GaussianSplatField::SphericalHarmonicsRest;
      reader.rest_index = rest_search->second;
    }
    readers.emplace_back(reader);
  }
  return true;
}

[[nodiscard]] bool ReadGaussianSplatRows(std::istream& stream, const bool binary,
                                         const std::vector<PlyElement>& elements, const PlyElement& vertex_element,
                                         const std::vector<GaussianSplatPropertyReader>& vertex_readers,
                                         GaussianSplatStagedLoadPayload& payload) {
  payload.positions.assign(vertex_element.size, glm::vec3(0.0f));
  payload.scales.assign(vertex_element.size, glm::vec3(0.0f));
  payload.colors.assign(vertex_element.size, glm::vec3(0.0f));
  payload.opacities.assign(vertex_element.size, 0.0f);
  payload.rotations.assign(vertex_element.size, glm::vec4(0.0f));
  payload.spherical_harmonics_rest.assign(vertex_element.size * payload.spherical_harmonics_rest_float_count, 0.0f);

  for (const auto& element : elements) {
    const bool read_vertex = element.name == "vertex";
    for (size_t row = 0; row < element.size; ++row) {
      for (size_t property_index = 0; property_index < element.properties.size(); ++property_index) {
        if (!read_vertex) {
          if (!SkipProperty(stream, binary, element.properties[property_index])) {
            return false;
          }
          continue;
        }

        const auto& reader = vertex_readers[property_index];
        if (reader.field == GaussianSplatField::Ignore) {
          if (!SkipProperty(stream, binary, reader.type, reader.list, reader.list_type)) {
            return false;
          }
          continue;
        }
        float value = 0.0f;
        if (!ReadScalar(stream, binary, reader.type, value)) {
          return false;
        }
        ApplyGaussianSplatProperty(payload, row, reader, value);
      }
    }
  }
  return static_cast<bool>(stream);
}

bool LoadGaussianSplatPly(const std::filesystem::path& path, GaussianSplatStagedLoadPayload& payload) {
  try {
    std::ifstream file_stream(path.string(), std::ios::binary);
    if (!file_stream || file_stream.fail()) {
      return false;
    }

    PlyFile file;
    file.parse_header(file_stream);
    const std::vector<std::string> required_properties = {"x",       "y",      "z",      "scale_0", "scale_1",
                                                          "scale_2", "f_dc_0", "f_dc_1", "f_dc_2",  "opacity",
                                                          "rot_0",   "rot_1",  "rot_2",  "rot_3"};
    if (!HasVertexProperties(file, required_properties)) {
      return false;
    }

    const auto rest_properties = CollectRestPropertyNames(file);
    const auto elements = file.get_elements();
    const auto vertex_element = FindElement(elements, "vertex");
    if (!vertex_element || vertex_element->size == 0) {
      return false;
    }
    payload.spherical_harmonics_rest_float_count = static_cast<uint32_t>(rest_properties.size());

    std::vector<GaussianSplatPropertyReader> vertex_readers;
    if (!BuildVertexReaders(*vertex_element, rest_properties, vertex_readers) ||
        !ReadGaussianSplatRows(file_stream, file.is_binary_file(), elements, *vertex_element, vertex_readers,
                               payload)) {
      return false;
    }

    const auto splat_count = payload.positions.size();
    return !payload.positions.empty() && payload.scales.size() == splat_count && payload.colors.size() == splat_count &&
           payload.opacities.size() == splat_count && payload.rotations.size() == splat_count &&
           payload.spherical_harmonics_rest.size() ==
               splat_count * static_cast<size_t>(payload.spherical_harmonics_rest_float_count);
  } catch (const std::exception& e) {
    std::cerr << "Failed to load Gaussian splat PLY: " << e.what() << std::endl;
    return false;
  }
}

bool SaveGaussianSplatPly(const GaussianSplat& gaussian_splat, const std::filesystem::path& path) {
  std::ofstream stream(path, std::ios::binary);
  if (!stream || stream.fail()) {
    return false;
  }

  const auto splat_count = gaussian_splat.positions.size();
  stream << "ply\n";
  stream << "format binary_little_endian 1.0\n";
  stream << "element vertex " << splat_count << "\n";
  const std::vector<std::string> properties = {"x",      "y",      "z",       "scale_0", "scale_1", "scale_2", "f_dc_0",
                                               "f_dc_1", "f_dc_2", "opacity", "rot_0",   "rot_1",   "rot_2",   "rot_3"};
  for (const auto& property : properties) {
    stream << "property float " << property << "\n";
  }
  for (uint32_t i = 0; i < gaussian_splat.spherical_harmonics_rest_float_count; ++i) {
    stream << "property float f_rest_" << i << "\n";
  }
  stream << "end_header\n";

  for (size_t i = 0; i < splat_count; ++i) {
    const auto position = GetOrDefault(gaussian_splat.positions, i, glm::vec3(0.0f));
    const auto scale = GetOrDefault(gaussian_splat.scales, i, glm::vec3(0.0f));
    const auto color = GetOrDefault(gaussian_splat.colors, i, glm::vec3(0.0f));
    const auto opacity = GetOrDefault(gaussian_splat.opacities, i, 0.0f);
    const auto rotation = GetOrDefault(gaussian_splat.rotations, i, glm::vec4(1.0f, 0.0f, 0.0f, 0.0f));
    const std::array<float, 14> values = {position.x, position.y, position.z, scale.x,   scale.y,
                                          scale.z,    color.x,    color.y,    color.z,   opacity,
                                          rotation.x, rotation.y, rotation.z, rotation.w};
    for (const auto value : values) {
      if (!WriteBinaryValue(stream, value)) {
        return false;
      }
    }
    const auto rest_offset = i * gaussian_splat.spherical_harmonics_rest_float_count;
    for (uint32_t rest_index = 0; rest_index < gaussian_splat.spherical_harmonics_rest_float_count; ++rest_index) {
      const auto value = GetOrDefault(gaussian_splat.spherical_harmonics_rest, rest_offset + rest_index, 0.0f);
      if (!WriteBinaryValue(stream, value)) {
        return false;
      }
    }
  }
  return static_cast<bool>(stream);
}

bool LoadGaussianSplatSplat(const std::filesystem::path& path, GaussianSplatStagedLoadPayload& payload) {
  constexpr size_t kSplatRowSize = 32;
  std::error_code error;
  const auto file_size = std::filesystem::file_size(path, error);
  if (error || file_size == 0 || file_size % kSplatRowSize != 0) {
    return false;
  }

  std::ifstream stream(path, std::ios::binary);
  if (!stream || stream.fail()) {
    return false;
  }

  const auto splat_count = static_cast<size_t>(file_size / kSplatRowSize);
  payload.positions.resize(splat_count);
  payload.scales.resize(splat_count);
  payload.colors.resize(splat_count);
  payload.opacities.resize(splat_count);
  payload.rotations.resize(splat_count);
  payload.spherical_harmonics_rest.clear();
  payload.spherical_harmonics_rest_float_count = 0;

  for (size_t i = 0; i < splat_count; ++i) {
    uint8_t rgba[4] = {};
    uint8_t rotation[4] = {};
    if (!ReadBinaryValue(stream, payload.positions[i].x) || !ReadBinaryValue(stream, payload.positions[i].y) ||
        !ReadBinaryValue(stream, payload.positions[i].z) || !ReadBinaryValue(stream, payload.scales[i].x) ||
        !ReadBinaryValue(stream, payload.scales[i].y) || !ReadBinaryValue(stream, payload.scales[i].z)) {
      return false;
    }
    stream.read(reinterpret_cast<char*>(rgba), sizeof(rgba));
    stream.read(reinterpret_cast<char*>(rotation), sizeof(rotation));
    if (!stream) {
      return false;
    }
    payload.colors[i] = glm::vec3(rgba[0], rgba[1], rgba[2]) / 255.0f;
    payload.opacities[i] = static_cast<float>(rgba[3]) / 255.0f;
    payload.rotations[i] = NormalizeRotation(glm::vec4(
        (static_cast<float>(rotation[0]) - 128.0f) / 128.0f, (static_cast<float>(rotation[1]) - 128.0f) / 128.0f,
        (static_cast<float>(rotation[2]) - 128.0f) / 128.0f, (static_cast<float>(rotation[3]) - 128.0f) / 128.0f));
  }
  return true;
}

bool SaveGaussianSplatSplat(const GaussianSplat& gaussian_splat, const std::filesystem::path& path) {
  WarnDropsSphericalHarmonicsRest(".splat", gaussian_splat.spherical_harmonics_rest_float_count,
                                  gaussian_splat.spherical_harmonics_rest);
  std::ofstream stream(path, std::ios::binary);
  if (!stream || stream.fail()) {
    return false;
  }

  for (size_t i = 0; i < gaussian_splat.positions.size(); ++i) {
    const auto position = GetOrDefault(gaussian_splat.positions, i, glm::vec3(0.0f));
    const auto scale = GetOrDefault(gaussian_splat.scales, i, glm::vec3(0.0f));
    const auto color = GetOrDefault(gaussian_splat.colors, i, glm::vec3(0.0f));
    const auto opacity = GetOrDefault(gaussian_splat.opacities, i, 0.0f);
    const auto rotation =
        NormalizeRotation(GetOrDefault(gaussian_splat.rotations, i, glm::vec4(1.0f, 0.0f, 0.0f, 0.0f)));
    const uint8_t rgba[4] = {FloatToByte(color.x), FloatToByte(color.y), FloatToByte(color.z), FloatToByte(opacity)};
    const uint8_t packed_rotation[4] = {QuaternionToByte(rotation.x), QuaternionToByte(rotation.y),
                                        QuaternionToByte(rotation.z), QuaternionToByte(rotation.w)};
    if (!WriteBinaryValue(stream, position.x) || !WriteBinaryValue(stream, position.y) ||
        !WriteBinaryValue(stream, position.z) || !WriteBinaryValue(stream, scale.x) ||
        !WriteBinaryValue(stream, scale.y) || !WriteBinaryValue(stream, scale.z)) {
      return false;
    }
    stream.write(reinterpret_cast<const char*>(rgba), sizeof(rgba));
    stream.write(reinterpret_cast<const char*>(packed_rotation), sizeof(packed_rotation));
    if (!stream) {
      return false;
    }
  }
  return static_cast<bool>(stream);
}

bool LoadGaussianSplatKSplat(const std::filesystem::path& path, GaussianSplatStagedLoadPayload& payload) {
  constexpr uint32_t kHeaderSizeBytes = 4096;
  constexpr uint32_t kSectionHeaderSizeBytes = 1024;
  constexpr uint32_t kBytesPerLevel0SphericalHarmonics0Splat = 44;
  std::ifstream stream(path, std::ios::binary);
  if (!stream || stream.fail()) {
    return false;
  }

  std::vector<uint8_t> header(kHeaderSizeBytes, 0);
  stream.read(reinterpret_cast<char*>(header.data()), header.size());
  if (!stream || header[0] != 0 || header[1] != 1) {
    return false;
  }
  const auto* header_u16 = reinterpret_cast<const uint16_t*>(header.data());
  const auto* header_u32 = reinterpret_cast<const uint32_t*>(header.data());
  const auto max_section_count = header_u32[1];
  const auto section_count = header_u32[2];
  const auto splat_count = header_u32[4];
  const auto compression_level = header_u16[10];
  if (compression_level != 0 || max_section_count == 0 || section_count == 0 || section_count > max_section_count) {
    return false;
  }

  std::vector<uint8_t> section_headers(static_cast<size_t>(max_section_count) * kSectionHeaderSizeBytes, 0);
  stream.read(reinterpret_cast<char*>(section_headers.data()), section_headers.size());
  if (!stream) {
    return false;
  }

  payload.positions.clear();
  payload.scales.clear();
  payload.rotations.clear();
  payload.colors.clear();
  payload.opacities.clear();
  payload.spherical_harmonics_rest.clear();
  payload.spherical_harmonics_rest_float_count = 0;
  payload.positions.reserve(splat_count);
  payload.scales.reserve(splat_count);
  payload.rotations.reserve(splat_count);
  payload.colors.reserve(splat_count);
  payload.opacities.reserve(splat_count);

  for (uint32_t section_index = 0; section_index < max_section_count; ++section_index) {
    const auto* section_base = section_headers.data() + static_cast<size_t>(section_index) * kSectionHeaderSizeBytes;
    const auto* section_u16 = reinterpret_cast<const uint16_t*>(section_base);
    const auto* section_u32 = reinterpret_cast<const uint32_t*>(section_base);
    const auto section_splat_count = section_u32[0];
    const auto section_max_splat_count = section_u32[1];
    const auto storage_size_bytes = section_u32[7];
    const auto sh_degree = section_u16[20];
    if (section_index >= section_count) {
      if (section_max_splat_count != 0) {
        return false;
      }
      continue;
    }
    if (sh_degree != 0 || section_splat_count > section_max_splat_count ||
        storage_size_bytes != section_max_splat_count * kBytesPerLevel0SphericalHarmonics0Splat) {
      return false;
    }
    for (uint32_t i = 0; i < section_max_splat_count; ++i) {
      glm::vec3 position(0.0f);
      glm::vec3 scale(0.0f);
      glm::vec4 rotation(1.0f, 0.0f, 0.0f, 0.0f);
      uint8_t rgba[4] = {};
      if (!ReadBinaryValue(stream, position.x) || !ReadBinaryValue(stream, position.y) ||
          !ReadBinaryValue(stream, position.z) || !ReadBinaryValue(stream, scale.x) ||
          !ReadBinaryValue(stream, scale.y) || !ReadBinaryValue(stream, scale.z) ||
          !ReadBinaryValue(stream, rotation.x) || !ReadBinaryValue(stream, rotation.y) ||
          !ReadBinaryValue(stream, rotation.z) || !ReadBinaryValue(stream, rotation.w)) {
        return false;
      }
      stream.read(reinterpret_cast<char*>(rgba), sizeof(rgba));
      if (!stream) {
        return false;
      }
      if (i >= section_splat_count) {
        continue;
      }
      payload.positions.emplace_back(position);
      payload.scales.emplace_back(scale);
      payload.rotations.emplace_back(NormalizeRotation(rotation));
      payload.colors.emplace_back(glm::vec3(rgba[0], rgba[1], rgba[2]) / 255.0f);
      payload.opacities.emplace_back(static_cast<float>(rgba[3]) / 255.0f);
    }
  }
  return payload.positions.size() == splat_count;
}

bool SaveGaussianSplatKSplat(const GaussianSplat& gaussian_splat, const std::filesystem::path& path) {
  constexpr uint32_t kHeaderSizeBytes = 4096;
  constexpr uint32_t kSectionHeaderSizeBytes = 1024;
  constexpr uint32_t kBytesPerLevel0SphericalHarmonics0Splat = 44;
  WarnDropsSphericalHarmonicsRest(".ksplat", gaussian_splat.spherical_harmonics_rest_float_count,
                                  gaussian_splat.spherical_harmonics_rest);

  std::ofstream stream(path, std::ios::binary);
  if (!stream || stream.fail()) {
    return false;
  }

  const auto splat_count = static_cast<uint32_t>(gaussian_splat.positions.size());
  std::vector<uint8_t> header(kHeaderSizeBytes, 0);
  header[0] = 0;
  header[1] = 1;
  auto* header_u16 = reinterpret_cast<uint16_t*>(header.data());
  auto* header_u32 = reinterpret_cast<uint32_t*>(header.data());
  auto* header_f32 = reinterpret_cast<float*>(header.data());
  header_u32[1] = 1;
  header_u32[2] = 1;
  header_u32[3] = splat_count;
  header_u32[4] = splat_count;
  header_u16[10] = 0;
  const auto scene_center = (gaussian_splat.GetMinBound() + gaussian_splat.GetMaxBound()) * 0.5f;
  header_f32[6] = scene_center.x;
  header_f32[7] = scene_center.y;
  header_f32[8] = scene_center.z;
  header_f32[9] = -1.5f;
  header_f32[10] = 1.5f;
  stream.write(reinterpret_cast<const char*>(header.data()), header.size());

  std::vector<uint8_t> section_header(kSectionHeaderSizeBytes, 0);
  auto* section_u16 = reinterpret_cast<uint16_t*>(section_header.data());
  auto* section_u32 = reinterpret_cast<uint32_t*>(section_header.data());
  section_u32[0] = splat_count;
  section_u32[1] = splat_count;
  section_u32[7] = splat_count * kBytesPerLevel0SphericalHarmonics0Splat;
  section_u16[20] = 0;
  stream.write(reinterpret_cast<const char*>(section_header.data()), section_header.size());

  for (size_t i = 0; i < gaussian_splat.positions.size(); ++i) {
    const auto position = GetOrDefault(gaussian_splat.positions, i, glm::vec3(0.0f));
    const auto scale = GetOrDefault(gaussian_splat.scales, i, glm::vec3(0.0f));
    const auto rotation =
        NormalizeRotation(GetOrDefault(gaussian_splat.rotations, i, glm::vec4(1.0f, 0.0f, 0.0f, 0.0f)));
    const auto color = GetOrDefault(gaussian_splat.colors, i, glm::vec3(0.0f));
    const auto opacity = GetOrDefault(gaussian_splat.opacities, i, 0.0f);
    const uint8_t rgba[4] = {FloatToByte(color.x), FloatToByte(color.y), FloatToByte(color.z), FloatToByte(opacity)};
    if (!WriteBinaryValue(stream, position.x) || !WriteBinaryValue(stream, position.y) ||
        !WriteBinaryValue(stream, position.z) || !WriteBinaryValue(stream, scale.x) ||
        !WriteBinaryValue(stream, scale.y) || !WriteBinaryValue(stream, scale.z) ||
        !WriteBinaryValue(stream, rotation.x) || !WriteBinaryValue(stream, rotation.y) ||
        !WriteBinaryValue(stream, rotation.z) || !WriteBinaryValue(stream, rotation.w)) {
      return false;
    }
    stream.write(reinterpret_cast<const char*>(rgba), sizeof(rgba));
  }
  return static_cast<bool>(stream);
}
}  // namespace

bool GaussianSplat::LoadInternal(const std::filesystem::path& path) {
  const auto extension = LowercaseExtension(path);
  if (extension == ".ply") {
    return LoadPly(path);
  }
  if (extension == ".splat") {
    return LoadSplat(path);
  }
  if (extension == ".ksplat") {
    return LoadKSplat(path);
  }
  return Serialization::LoadAssetFromYaml(*this, path);
}

bool GaussianSplat::SupportsStagedLoading(const std::filesystem::path& path) const {
  const auto extension = LowercaseExtension(path);
  return extension == ".evegaussiansplat" || extension == ".ply" || extension == ".splat" || extension == ".ksplat";
}

std::shared_ptr<StagedAssetLoadPayload> GaussianSplat::LoadStagedPayloadInternal(
    const std::filesystem::path& path) const {
  try {
    auto payload = std::make_shared<GaussianSplatStagedLoadPayload>();
    const auto extension = LowercaseExtension(path);
    if (extension == ".ply") {
      if (!LoadGaussianSplatPly(path, *payload)) {
        return {};
      }
      return payload;
    }
    if (extension == ".splat") {
      if (!LoadGaussianSplatSplat(path, *payload)) {
        return {};
      }
      return payload;
    }
    if (extension == ".ksplat") {
      if (!LoadGaussianSplatKSplat(path, *payload)) {
        return {};
      }
      return payload;
    }

    const std::ifstream stream(path.string());
    std::stringstream string_stream;
    string_stream << stream.rdbuf();
    payload->yaml = true;
    payload->yaml_node = YAML::Load(string_stream.str());
    return payload;
  } catch (const std::exception& e) {
    std::cerr << "Failed to load staged Gaussian splat payload: " << e.what() << std::endl;
    return {};
  }
}

bool GaussianSplat::ApplyStagedPayloadInternal(const std::filesystem::path&,
                                               const std::shared_ptr<StagedAssetLoadPayload>& payload) {
  const auto gaussian_payload = std::dynamic_pointer_cast<GaussianSplatStagedLoadPayload>(payload);
  if (!gaussian_payload) {
    return false;
  }
  try {
    if (gaussian_payload->yaml) {
      Serialization::DeserializeObject(gaussian_payload->yaml_node, static_cast<IAsset&>(*this));
      return true;
    }
    positions = std::move(gaussian_payload->positions);
    scales = std::move(gaussian_payload->scales);
    rotations = std::move(gaussian_payload->rotations);
    opacities = std::move(gaussian_payload->opacities);
    colors = std::move(gaussian_payload->colors);
    spherical_harmonics_rest = std::move(gaussian_payload->spherical_harmonics_rest);
    spherical_harmonics_rest_float_count = gaussian_payload->spherical_harmonics_rest_float_count;
    RecalculateBoundingBox();
    ++version_;
    saved_ = false;
    return true;
  } catch (const std::exception& e) {
    std::cerr << "Failed to apply staged Gaussian splat payload: " << e.what() << std::endl;
    return false;
  }
}

bool GaussianSplat::SaveInternal(const std::filesystem::path& path) const {
  const auto extension = LowercaseExtension(path);
  if (extension == ".ply") {
    return SavePly(path);
  }
  if (extension == ".splat") {
    return SaveSplat(path);
  }
  if (extension == ".ksplat") {
    return SaveKSplat(path);
  }
  return Serialization::SaveAssetAsYaml(*this, path);
}

bool GaussianSplat::RegisterAssetIoHandlers(const std::string& owner_name, const std::string& type_name) {
  return Serialization::RegisterAssetIoHandler<GaussianSplat>(
      [](const GaussianSplat& asset, const std::filesystem::path& path) {
        return asset.SaveInternal(path);
      },
      [](GaussianSplat& asset, const std::filesystem::path& path) {
        return asset.LoadInternal(path);
      },
      [](const GaussianSplat& asset, const std::filesystem::path& path) {
        return asset.SupportsStagedLoading(path);
      },
      [](const GaussianSplat& asset, const std::filesystem::path& path) {
        return asset.LoadStagedPayloadInternal(path);
      },
      [](GaussianSplat& asset, const std::filesystem::path& path,
         const std::shared_ptr<StagedAssetLoadPayload>& payload) {
        return asset.ApplyStagedPayloadInternal(path, payload);
      },
      owner_name, type_name);
}

void GaussianSplat::OnCreate() {
}

size_t GaussianSplat::GetSplatCount() const {
  return positions.size();
}

bool GaussianSplat::Empty() const {
  return positions.empty();
}

glm::vec3 GaussianSplat::GetMinBound() const {
  return min_bound_;
}

glm::vec3 GaussianSplat::GetMaxBound() const {
  return max_bound_;
}

void GaussianSplat::SetBounds(const glm::vec3& min_bound, const glm::vec3& max_bound) {
  min_bound_ = min_bound;
  max_bound_ = max_bound;
}

void GaussianSplat::InvalidateGpuCaches() {
  gpu_data_dirty_ = true;
  gpu_data_buffer_dirty_ = true;
  spherical_harmonics_rest_buffer_dirty_ = true;
  sort_caches_.clear();
}

void GaussianSplat::BuildGpuData() const {
  if (!gpu_data_dirty_ && gpu_data_.size() == positions.size()) {
    return;
  }

  gpu_data_.resize(positions.size());
  for (size_t i = 0; i < positions.size(); ++i) {
    auto& target = gpu_data_[i];
    target.position_opacity = glm::vec4(positions[i], GetOrDefault(opacities, i, 1.0f));
    target.scale_reserved = glm::vec4(GetOrDefault(scales, i, glm::vec3(0.0f)), 0.0f);
    target.rotation = GetOrDefault(rotations, i, glm::vec4(1.0f, 0.0f, 0.0f, 0.0f));
    target.color_rest_offset = glm::vec4(GetOrDefault(colors, i, glm::vec3(1.0f)), -1.0f);

    const auto rest_offset = i * spherical_harmonics_rest_float_count;
    if (spherical_harmonics_rest_float_count > 0 &&
        rest_offset + spherical_harmonics_rest_float_count <= spherical_harmonics_rest.size()) {
      target.color_rest_offset.w = static_cast<float>(rest_offset);
    }
  }

  gpu_data_dirty_ = false;
  gpu_data_buffer_dirty_ = true;
  ++gpu_data_revision_;
}

const std::vector<GaussianSplatGpuData>& GaussianSplat::EnsureGpuData() const {
  BuildGpuData();
  if (gpu_data_buffer_dirty_) {
    UploadVector(gpu_data_buffer_, gpu_data_);
    if (CanUploadGpuData()) {
      gpu_data_buffer_dirty_ = false;
    }
  }
  return gpu_data_;
}

const std::shared_ptr<evo_engine::Buffer>& GaussianSplat::GetGpuDataBuffer() const {
  (void)EnsureGpuData();
  return gpu_data_buffer_;
}

const std::shared_ptr<evo_engine::Buffer>& GaussianSplat::GetSphericalHarmonicsRestBuffer() const {
  if (spherical_harmonics_rest.empty()) {
    spherical_harmonics_rest_buffer_.reset();
    spherical_harmonics_rest_buffer_dirty_ = false;
    return spherical_harmonics_rest_buffer_;
  }
  if (spherical_harmonics_rest_buffer_dirty_) {
    UploadVector(spherical_harmonics_rest_buffer_, spherical_harmonics_rest);
    if (CanUploadGpuData()) {
      spherical_harmonics_rest_buffer_dirty_ = false;
    }
  }
  return spherical_harmonics_rest_buffer_;
}

uint32_t GaussianSplat::GetSphericalHarmonicsRestFloatCount() const {
  return spherical_harmonics_rest_float_count;
}

uint32_t GaussianSplat::GetSphericalHarmonicsDegree() const {
  const auto splat_count = positions.size();
  if (splat_count == 0u || spherical_harmonics_rest_float_count == 0u ||
      spherical_harmonics_rest.size() < splat_count * static_cast<size_t>(spherical_harmonics_rest_float_count)) {
    return 0u;
  }

  const auto coefficients_per_channel = spherical_harmonics_rest_float_count / 3u;
  if (coefficients_per_channel >= 15u) {
    return 3u;
  }
  if (coefficients_per_channel >= 8u) {
    return 2u;
  }
  if (coefficients_per_channel >= 3u) {
    return 1u;
  }
  return 0u;
}

uint32_t GaussianSplat::GetGpuDataRevision() const {
  (void)EnsureGpuData();
  return gpu_data_revision_;
}

const GaussianSplatSortCache& GaussianSplat::EnsureSortedIndices(const Handle& camera_handle,
                                                                 const Handle& sort_owner_handle,
                                                                 const glm::mat4& model, const glm::mat4& view) const {
  (void)EnsureGpuData();
  auto& cache = sort_caches_[CombineSortCacheHandles(camera_handle, sort_owner_handle)];
  if (cache.valid && cache.indices.size() == positions.size() && SameMatrix(cache.model, model) &&
      SameMatrix(cache.view, view)) {
    return cache;
  }

  struct SortEntry {
    uint32_t index = 0;
    float depth = 0.0f;
  };
  std::vector<SortEntry> entries;
  entries.resize(positions.size());
  const auto model_view = view * model;
  for (uint32_t i = 0; i < positions.size(); ++i) {
    const auto view_position = model_view * glm::vec4(positions[i], 1.0f);
    entries[i].index = i;
    entries[i].depth = -view_position.z;
  }

  std::stable_sort(entries.begin(), entries.end(), [](const SortEntry& lhs, const SortEntry& rhs) {
    return lhs.depth > rhs.depth;
  });

  cache.indices.resize(entries.size());
  cache.depths.resize(entries.size());
  for (size_t i = 0; i < entries.size(); ++i) {
    cache.indices[i] = entries[i].index;
    cache.depths[i] = entries[i].depth;
  }

  UploadVector(cache.index_buffer, cache.indices);
  UploadVector(cache.depth_buffer, cache.depths);
  cache.model = model;
  cache.view = view;
  cache.valid = true;
  ++cache.generation;
  return cache;
}

const GaussianSplatSortCache* GaussianSplat::FindSortCache(const Handle& camera_handle,
                                                           const Handle& sort_owner_handle) const {
  const auto search = sort_caches_.find(CombineSortCacheHandles(camera_handle, sort_owner_handle));
  return search == sort_caches_.end() ? nullptr : &search->second;
}

void GaussianSplat::RecalculateBoundingBox() {
  if (positions.empty()) {
    min_bound_ = glm::vec3(0.0f);
    max_bound_ = glm::vec3(0.0f);
    InvalidateGpuCaches();
    return;
  }
  min_bound_ = positions.front();
  max_bound_ = positions.front();
  for (const auto& position : positions) {
    min_bound_ = glm::min(min_bound_, position);
    max_bound_ = glm::max(max_bound_, position);
  }
  InvalidateGpuCaches();
}

bool GaussianSplat::LoadPly(const std::filesystem::path& path) {
  GaussianSplatStagedLoadPayload payload;
  if (!LoadGaussianSplatPly(path, payload)) {
    return false;
  }
  positions = std::move(payload.positions);
  scales = std::move(payload.scales);
  rotations = std::move(payload.rotations);
  opacities = std::move(payload.opacities);
  colors = std::move(payload.colors);
  spherical_harmonics_rest = std::move(payload.spherical_harmonics_rest);
  spherical_harmonics_rest_float_count = payload.spherical_harmonics_rest_float_count;
  RecalculateBoundingBox();
  ++version_;
  saved_ = false;
  return true;
}

bool GaussianSplat::SavePly(const std::filesystem::path& path) const {
  return SaveGaussianSplatPly(*this, path);
}

bool GaussianSplat::LoadSplat(const std::filesystem::path& path) {
  GaussianSplatStagedLoadPayload payload;
  if (!LoadGaussianSplatSplat(path, payload)) {
    return false;
  }
  positions = std::move(payload.positions);
  scales = std::move(payload.scales);
  rotations = std::move(payload.rotations);
  opacities = std::move(payload.opacities);
  colors = std::move(payload.colors);
  spherical_harmonics_rest = std::move(payload.spherical_harmonics_rest);
  spherical_harmonics_rest_float_count = payload.spherical_harmonics_rest_float_count;
  RecalculateBoundingBox();
  ++version_;
  saved_ = false;
  return true;
}

bool GaussianSplat::SaveSplat(const std::filesystem::path& path) const {
  return SaveGaussianSplatSplat(*this, path);
}

bool GaussianSplat::LoadKSplat(const std::filesystem::path& path) {
  GaussianSplatStagedLoadPayload payload;
  if (!LoadGaussianSplatKSplat(path, payload)) {
    return false;
  }
  positions = std::move(payload.positions);
  scales = std::move(payload.scales);
  rotations = std::move(payload.rotations);
  opacities = std::move(payload.opacities);
  colors = std::move(payload.colors);
  spherical_harmonics_rest = std::move(payload.spherical_harmonics_rest);
  spherical_harmonics_rest_float_count = payload.spherical_harmonics_rest_float_count;
  RecalculateBoundingBox();
  ++version_;
  saved_ = false;
  return true;
}

bool GaussianSplat::SaveKSplat(const std::filesystem::path& path) const {
  return SaveGaussianSplatKSplat(*this, path);
}
