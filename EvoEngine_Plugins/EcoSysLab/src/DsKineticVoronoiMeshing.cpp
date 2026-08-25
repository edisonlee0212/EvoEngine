#include "DsKineticVoronoiMeshing.hpp"
#include "DsIntersectionBoundaryMesh.hpp"
#include "DsIntersectionBoundaryMeshGroup.hpp"
#include "DynamicTreeStrands.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>
#include <string>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_inverse.hpp>  // for inverse()
#include <glm/gtx/norm.hpp>            // for length2()
#include <optional>
#include <queue>
#include <unordered_set>
#include <utility>
#include <vector>
#include "BufferExporter.hpp"
#include "ComputePipeline.hpp"
#include "DynamicStrands.hpp"
#include "MeshRenderer.hpp"
#include "Platform/Platform.hpp"
#include "ProgressBar.hpp"
#include "ProjectManager.hpp"
#include "Shader.hpp"
#include "Transform.hpp"
#include "kinDS/kinDS/KineticDelaunay.hpp"
#include "kinDS/kinDS/MeshIntersection.hpp"
#include "kinDS/kinDS/ObjExporter.hpp"
#include "kinDS/kinDS/Polynomial.hpp"
#include "kinDS/kinDS/SegmentBuilder.hpp"
#include "kinDS/kinDS/Statistics.hpp"

using namespace eco_sys_lab_plugin;

namespace {

using GpuMeshletVertex = DsKineticVoronoiMeshing::GpuSegmentMeshletVertex;
using GpuMeshletTriangle = DsKineticVoronoiMeshing::GpuSegmentMeshletTriangle;

constexpr char kMeshingBufferMagic[4] = {'K', 'V', 'M', 'G'};
constexpr uint32_t kMeshingBufferVersion = 1;
constexpr uint64_t kFnvOffset = 14695981039346656037ull;
constexpr uint64_t kFnvPrime = 1099511628211ull;
constexpr uint64_t kMaxMeshingBufferCount = 500000000ull;
constexpr uint64_t kMaxMeshingBufferString = 16ull * 1024ull * 1024ull;

struct Fnv64 {
  uint64_t value = kFnvOffset;
  void MixBytes(const void* data, size_t size) {
    const auto* bytes = static_cast<const uint8_t*>(data);
    for (size_t i = 0; i < size; ++i) {
      value ^= bytes[i];
      value *= kFnvPrime;
    }
  }
  void MixCString(const char* text) { MixBytes(text, std::strlen(text)); }
  template <typename T>
  void MixPod(const T& value_pod) {
    MixBytes(&value_pod, sizeof(T));
  }
  template <typename T>
  void MixVec(const std::vector<T>& values) {
    MixPod(static_cast<uint64_t>(values.size()));
    if (!values.empty()) {
      MixBytes(values.data(), values.size() * sizeof(T));
    }
  }
  template <typename T>
  void MixNested(const std::vector<std::vector<T>>& values) {
    MixPod(static_cast<uint64_t>(values.size()));
    for (const auto& inner : values) {
      MixVec(inner);
    }
  }
};

std::string HashToHex(uint64_t hash) {
  std::ostringstream stream;
  stream << std::hex << std::setw(16) << std::setfill('0') << hash;
  return stream.str();
}

std::filesystem::path MeshingBufferDirectory() {
  const auto project_path = ProjectManager::GetProjectPath();
  if (project_path.empty()) {
    return std::filesystem::path("MeshBuffers");
  }
  return project_path.parent_path() / "MeshBuffers";
}

template <typename Nested>
size_t CountNestedElements(const Nested& nested) {
  size_t count = 0;
  for (const auto& inner : nested) {
    count += inner.size();
  }
  return count;
}

size_t CountTripleNestedElements(const std::vector<std::vector<std::vector<size_t>>>& nested) {
  size_t count = 0;
  for (const auto& by_height : nested) {
    count += CountNestedElements(by_height);
  }
  return count;
}

std::string FormatRootTransformSummary(const GlobalTransform& root_transform) {
  const glm::mat4& m = root_transform.value;
  const glm::vec3 t = m[3];
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(6) << "t=(" << t.x << "," << t.y << "," << t.z << ")"
      << " m00=" << m[0][0] << " m11=" << m[1][1] << " m22=" << m[2][2];
  return oss.str();
}

std::string ComputeMeshingInputHash(const std::vector<std::vector<glm::dvec2>>& support_points,
                                    const std::vector<std::vector<double>>& subdivisions_by_strand,
                                    const std::vector<std::vector<int>>& physics_strand_to_segment_indices,
                                    const std::vector<std::vector<glm::dmat4>>& transforms_by_height_and_branch,
                                    const GlobalTransform& root_transform,
                                    const std::vector<std::vector<size_t>>& branch_indices,
                                    const std::vector<std::vector<std::vector<size_t>>>& strands_by_branch_id) {
  const auto mix_settings = [](Fnv64& hash) {
    hash.MixCString("DsKineticVoronoiMeshing.v1");
    hash.MixPod(kMeshingBufferVersion);
    hash.MixPod(static_cast<uint8_t>(1));  // mesh_cap_at_start
    hash.MixPod(static_cast<uint8_t>(1));  // transform_mesh_at_construction
    hash.MixPod(static_cast<uint8_t>(DsKineticVoronoiMeshing::meshing_settings.store_mesh_metadata ? 1 : 0));
    hash.MixPod(DsKineticVoronoiMeshing::meshing_settings.spline_tension);
  };

  Fnv64 settings_hash;
  mix_settings(settings_hash);

  Fnv64 root_hash;
  root_hash.MixPod(root_transform.value);

  Fnv64 support_hash;
  support_hash.MixNested(support_points);

  Fnv64 subdiv_hash;
  subdiv_hash.MixNested(subdivisions_by_strand);

  Fnv64 physics_hash;
  physics_hash.MixNested(physics_strand_to_segment_indices);

  Fnv64 transforms_hash;
  transforms_hash.MixNested(transforms_by_height_and_branch);

  Fnv64 branch_hash;
  branch_hash.MixNested(branch_indices);

  Fnv64 strands_by_branch_hash;
  strands_by_branch_hash.MixPod(static_cast<uint64_t>(strands_by_branch_id.size()));
  for (const auto& by_height : strands_by_branch_id) {
    strands_by_branch_hash.MixNested(by_height);
  }

  Fnv64 hash;
  mix_settings(hash);
  hash.MixPod(root_transform.value);
  hash.MixNested(support_points);
  hash.MixNested(subdivisions_by_strand);
  hash.MixNested(physics_strand_to_segment_indices);
  hash.MixNested(transforms_by_height_and_branch);
  hash.MixNested(branch_indices);
  hash.MixPod(static_cast<uint64_t>(strands_by_branch_id.size()));
  for (const auto& by_height : strands_by_branch_id) {
    hash.MixNested(by_height);
  }

  const std::string input_hash = HashToHex(hash.value);
  EVOENGINE_LOG("Meshing buffer hash " << input_hash
                                       << " | settings(v=" << kMeshingBufferVersion << ", store_meta="
                                       << (DsKineticVoronoiMeshing::meshing_settings.store_mesh_metadata ? 1 : 0)
                                       << ", spline_tension=" << DsKineticVoronoiMeshing::meshing_settings.spline_tension
                                       << ", cap_start=1, xform_at_construction=1)=" << HashToHex(settings_hash.value)
                                       << " root=" << HashToHex(root_hash.value) << " ["
                                       << FormatRootTransformSummary(root_transform) << "]"
                                       << " support(strands=" << support_points.size()
                                       << ", pts=" << CountNestedElements(support_points)
                                       << ")=" << HashToHex(support_hash.value)
                                       << " subdiv(strands=" << subdivisions_by_strand.size()
                                       << ", vals=" << CountNestedElements(subdivisions_by_strand)
                                       << ")=" << HashToHex(subdiv_hash.value)
                                       << " physics(strands=" << physics_strand_to_segment_indices.size()
                                       << ", segs=" << CountNestedElements(physics_strand_to_segment_indices)
                                       << ")=" << HashToHex(physics_hash.value)
                                       << " transforms(heights=" << transforms_by_height_and_branch.size()
                                       << ", mats=" << CountNestedElements(transforms_by_height_and_branch)
                                       << ")=" << HashToHex(transforms_hash.value)
                                       << " branches(heights=" << branch_indices.size()
                                       << ", ids=" << CountNestedElements(branch_indices)
                                       << ")=" << HashToHex(branch_hash.value)
                                       << " strands_by_branch(outer=" << strands_by_branch_id.size()
                                       << ", ids=" << CountTripleNestedElements(strands_by_branch_id)
                                       << ")=" << HashToHex(strands_by_branch_hash.value));
  return input_hash;
}

class BinaryWriter {
 public:
  explicit BinaryWriter(const std::filesystem::path& path)
      : out_(path, std::ios::binary | std::ios::trunc) {}
  bool Good() const { return static_cast<bool>(out_); }
  template <typename T>
  void WritePod(const T& value) {
    out_.write(reinterpret_cast<const char*>(&value), static_cast<std::streamsize>(sizeof(T)));
  }
  void WriteBytes(const void* data, size_t size) {
    if (size == 0) {
      return;
    }
    out_.write(reinterpret_cast<const char*>(data), static_cast<std::streamsize>(size));
  }
  template <typename T>
  void WriteVec(const std::vector<T>& values) {
    WritePod(static_cast<uint64_t>(values.size()));
    if (!values.empty()) {
      WriteBytes(values.data(), values.size() * sizeof(T));
    }
  }
  void WriteSizeTVec(const std::vector<size_t>& values) {
    WritePod(static_cast<uint64_t>(values.size()));
    if constexpr (sizeof(size_t) == 8) {
      if (!values.empty()) {
        WriteBytes(values.data(), values.size() * sizeof(size_t));
      }
    } else {
      for (const size_t value : values) {
        WritePod(static_cast<uint64_t>(value));
      }
    }
  }
  void WriteString(const std::string& value) {
    WritePod(static_cast<uint64_t>(value.size()));
    WriteBytes(value.data(), value.size());
  }
  void WriteStrings(const std::vector<std::string>& values) {
    WritePod(static_cast<uint64_t>(values.size()));
    for (const auto& value : values) {
      WriteString(value);
    }
  }
  void WriteNestedInt(const std::vector<std::vector<int>>& values) {
    WritePod(static_cast<uint64_t>(values.size()));
    for (const auto& inner : values) {
      WriteVec(inner);
    }
  }
  void WriteNestedSizeT(const std::vector<std::vector<size_t>>& values) {
    WritePod(static_cast<uint64_t>(values.size()));
    for (const auto& inner : values) {
      WriteSizeTVec(inner);
    }
  }

 private:
  std::ofstream out_;
};

class BinaryReader {
 public:
  explicit BinaryReader(const std::filesystem::path& path) : in_(path, std::ios::binary) {}
  bool Good() const { return !failed_ && static_cast<bool>(in_); }
  void Fail() { failed_ = true; }
  template <typename T>
  T ReadPod() {
    T value{};
    in_.read(reinterpret_cast<char*>(&value), static_cast<std::streamsize>(sizeof(T)));
    if (!in_) {
      failed_ = true;
    }
    return value;
  }
  template <typename T>
  std::vector<T> ReadVec() {
    const uint64_t count = ReadPod<uint64_t>();
    if (failed_ || count > kMaxMeshingBufferCount) {
      failed_ = true;
      return {};
    }
    std::vector<T> values(static_cast<size_t>(count));
    if (count > 0) {
      in_.read(reinterpret_cast<char*>(values.data()), static_cast<std::streamsize>(count * sizeof(T)));
      if (!in_) {
        failed_ = true;
        return {};
      }
    }
    return values;
  }
  std::vector<size_t> ReadSizeTVec() {
    const uint64_t count = ReadPod<uint64_t>();
    if (failed_ || count > kMaxMeshingBufferCount) {
      failed_ = true;
      return {};
    }
    std::vector<size_t> values(static_cast<size_t>(count));
    if constexpr (sizeof(size_t) == 8) {
      if (count > 0) {
        in_.read(reinterpret_cast<char*>(values.data()), static_cast<std::streamsize>(count * sizeof(size_t)));
        if (!in_) {
          failed_ = true;
          return {};
        }
      }
    } else {
      for (uint64_t i = 0; i < count; ++i) {
        values[static_cast<size_t>(i)] = static_cast<size_t>(ReadPod<uint64_t>());
      }
    }
    return values;
  }
  std::string ReadString() {
    const uint64_t count = ReadPod<uint64_t>();
    if (failed_ || count > kMaxMeshingBufferString) {
      failed_ = true;
      return {};
    }
    std::string value(static_cast<size_t>(count), '\0');
    if (count > 0) {
      in_.read(value.data(), static_cast<std::streamsize>(count));
      if (!in_) {
        failed_ = true;
        return {};
      }
    }
    return value;
  }
  std::vector<std::string> ReadStrings() {
    const uint64_t count = ReadPod<uint64_t>();
    if (failed_ || count > kMaxMeshingBufferCount) {
      failed_ = true;
      return {};
    }
    std::vector<std::string> values;
    values.reserve(static_cast<size_t>(count));
    for (uint64_t i = 0; i < count; ++i) {
      values.push_back(ReadString());
      if (failed_) {
        return {};
      }
    }
    return values;
  }
  std::vector<std::vector<int>> ReadNestedInt() {
    const uint64_t count = ReadPod<uint64_t>();
    if (failed_ || count > kMaxMeshingBufferCount) {
      failed_ = true;
      return {};
    }
    std::vector<std::vector<int>> values(static_cast<size_t>(count));
    for (auto& inner : values) {
      inner = ReadVec<int>();
      if (failed_) {
        return {};
      }
    }
    return values;
  }
  std::vector<std::vector<size_t>> ReadNestedSizeT() {
    const uint64_t count = ReadPod<uint64_t>();
    if (failed_ || count > kMaxMeshingBufferCount) {
      failed_ = true;
      return {};
    }
    std::vector<std::vector<size_t>> values(static_cast<size_t>(count));
    for (auto& inner : values) {
      inner = ReadSizeTVec();
      if (failed_) {
        return {};
      }
    }
    return values;
  }

 private:
  std::ifstream in_;
  bool failed_ = false;
};

void WriteVoronoiMesh(BinaryWriter& writer, const kinDS::VoronoiMesh& mesh) {
  writer.WritePod(static_cast<int32_t>(mesh.getNormalMode()));
  writer.WritePod(static_cast<uint8_t>(mesh.storeMetadata() ? 1 : 0));
  writer.WritePod(mesh.getCreationKineticTime());
  writer.WriteStrings(mesh.getMaterialNames());
  writer.WriteVec(mesh.getVertices());
  writer.WriteSizeTVec(mesh.getTriangles());
  writer.WriteVec(mesh.getNormals());
  writer.WriteVec(mesh.getUVs());
  writer.WriteSizeTVec(mesh.getUVIndices());
  writer.WriteVec(mesh.getMaterialIDs());
  writer.WriteSizeTVec(mesh.getGroupOffsets());
  writer.WriteStrings(mesh.getGroupNames());
  writer.WriteVec(mesh.getVertexColors());
  writer.WriteStrings(mesh.getVertexMetadata());
  writer.WriteStrings(mesh.getFaceMetadata());
  writer.WriteVec(mesh.getProfilePlaneXY());
  writer.WriteVec(mesh.getVertexKineticTimes());
  writer.WriteVec(mesh.getVertexSemanticUvs());
  writer.WritePod(static_cast<uint64_t>(mesh.getVertexCount()));
  for (size_t i = 0; i < mesh.getVertexCount(); ++i) {
    writer.WritePod(static_cast<uint8_t>(mesh.isVertexFlexible(i) ? 1 : 0));
  }
}

kinDS::VoronoiMesh ReadVoronoiMesh(BinaryReader& reader) {
  const auto normal_mode = static_cast<kinDS::NormalMode>(reader.ReadPod<int32_t>());
  const bool store_metadata = reader.ReadPod<uint8_t>() != 0;
  const double creation_time = reader.ReadPod<double>();
  auto material_names = reader.ReadStrings();
  kinDS::VoronoiMesh mesh(std::move(material_names), normal_mode);
  mesh.setStoreMetadata(store_metadata);
  mesh.setCreationKineticTime(creation_time);
  mesh.getVertices() = reader.ReadVec<glm::dvec3>();
  mesh.getTriangles() = reader.ReadSizeTVec();
  mesh.getNormals() = reader.ReadVec<glm::dvec3>();
  mesh.getUVs() = reader.ReadVec<glm::dvec3>();
  mesh.getUVIndices() = reader.ReadSizeTVec();
  mesh.getMaterialIDs() = reader.ReadVec<int>();
  mesh.setGroupOffsets(reader.ReadSizeTVec());
  mesh.setGroupNames(reader.ReadStrings());
  mesh.getVertexColors() = reader.ReadVec<glm::dvec3>();
  mesh.getVertexMetadata() = reader.ReadStrings();
  mesh.getFaceMetadata() = reader.ReadStrings();
  mesh.getProfilePlaneXY() = reader.ReadVec<glm::dvec2>();
  mesh.getVertexKineticTimes() = reader.ReadVec<double>();
  mesh.getVertexSemanticUvs() = reader.ReadVec<glm::dvec3>();
  const uint64_t flexible_count = reader.ReadPod<uint64_t>();
  if (!reader.Good() || flexible_count > kMaxMeshingBufferCount) {
    reader.Fail();
    return mesh;
  }
  for (uint64_t i = 0; i < flexible_count; ++i) {
    const uint8_t flag = reader.ReadPod<uint8_t>();
    if (flag && i < mesh.getVertexCount()) {
      mesh.setVertexFlexible(static_cast<size_t>(i), true);
    }
  }
  return mesh;
}

bool SaveMeshingBuffer(const std::filesystem::path& bin_path, const std::filesystem::path& yml_path,
                       const std::string& hash, const GlobalTransform& root_transform,
                       const std::vector<GpuMeshletVertex>& gpu_vertices,
                       const std::vector<GpuMeshletTriangle>& gpu_triangles,
                       const std::vector<kinDS::VoronoiMesh>& meshlets,
                       const std::vector<std::vector<int>>& neighbors,
                       const std::vector<size_t>& meshing_to_physics,
                       const std::vector<std::vector<size_t>>& strand_to_segment) {
  std::error_code error;
  std::filesystem::create_directories(bin_path.parent_path(), error);
  if (error) {
    EVOENGINE_WARNING("Failed to create meshing buffer directory: " << error.message());
    return false;
  }

  BinaryWriter writer(bin_path);
  writer.WriteBytes(kMeshingBufferMagic, 4);
  writer.WritePod(kMeshingBufferVersion);
  writer.WritePod(static_cast<uint32_t>(sizeof(GpuMeshletVertex)));
  writer.WritePod(static_cast<uint32_t>(sizeof(GpuMeshletTriangle)));
  writer.WritePod(root_transform.value);
  writer.WriteVec(gpu_vertices);
  writer.WriteVec(gpu_triangles);
  writer.WritePod(static_cast<uint64_t>(meshlets.size()));
  for (const auto& meshlet : meshlets) {
    WriteVoronoiMesh(writer, meshlet);
  }
  writer.WriteNestedInt(neighbors);
  writer.WriteSizeTVec(meshing_to_physics);
  writer.WriteNestedSizeT(strand_to_segment);
  if (!writer.Good()) {
    EVOENGINE_WARNING("Failed to write meshing buffer " << bin_path.string());
    return false;
  }

  std::time_t time = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
  std::tm utc{};
  gmtime_s(&utc, &time);
  std::ostringstream timestamp;
  timestamp << std::put_time(&utc, "%Y-%m-%dT%H:%M:%SZ");

  YAML::Emitter yaml;
  yaml << YAML::BeginMap;
  yaml << YAML::Key << "format" << YAML::Value << "KVMG";
  yaml << YAML::Key << "version" << YAML::Value << kMeshingBufferVersion;
  yaml << YAML::Key << "hash" << YAML::Value << hash;
  yaml << YAML::Key << "created_utc" << YAML::Value << timestamp.str();
  yaml << YAML::Key << "gpu_vertex_count" << YAML::Value << gpu_vertices.size();
  yaml << YAML::Key << "gpu_triangle_count" << YAML::Value << gpu_triangles.size();
  yaml << YAML::Key << "meshlet_count" << YAML::Value << meshlets.size();
  yaml << YAML::Key << "vertex_stride" << YAML::Value << sizeof(GpuMeshletVertex);
  yaml << YAML::Key << "triangle_stride" << YAML::Value << sizeof(GpuMeshletTriangle);
  yaml << YAML::Key << "spline_tension" << YAML::Value << DsKineticVoronoiMeshing::meshing_settings.spline_tension;
  yaml << YAML::Key << "store_mesh_metadata"
       << YAML::Value << DsKineticVoronoiMeshing::meshing_settings.store_mesh_metadata;
  yaml << YAML::Key << "mesh_cap_at_start" << YAML::Value << true;
  yaml << YAML::Key << "transform_mesh_at_construction" << YAML::Value << true;
  yaml << YAML::EndMap;

  std::ofstream yaml_out(yml_path);
  yaml_out << yaml.c_str();
  if (!yaml_out) {
    EVOENGINE_WARNING("Wrote meshing buffer binary but failed to write metadata " << yml_path.string());
  }
  return true;
}

bool LoadMeshingBuffer(const std::filesystem::path& bin_path, const GlobalTransform& root_transform,
                       std::vector<GpuMeshletVertex>& gpu_vertices, std::vector<GpuMeshletTriangle>& gpu_triangles,
                       std::vector<kinDS::VoronoiMesh>& meshlets, std::vector<std::vector<int>>& neighbors,
                       std::vector<size_t>& meshing_to_physics,
                       std::vector<std::vector<size_t>>& strand_to_segment) {
  BinaryReader reader(bin_path);
  char magic[4]{};
  magic[0] = reader.ReadPod<char>();
  magic[1] = reader.ReadPod<char>();
  magic[2] = reader.ReadPod<char>();
  magic[3] = reader.ReadPod<char>();
  if (!reader.Good() || std::memcmp(magic, kMeshingBufferMagic, 4) != 0) {
    return false;
  }
  if (reader.ReadPod<uint32_t>() != kMeshingBufferVersion) {
    return false;
  }
  if (reader.ReadPod<uint32_t>() != sizeof(GpuMeshletVertex) ||
      reader.ReadPod<uint32_t>() != sizeof(GpuMeshletTriangle)) {
    return false;
  }
  const glm::mat4 stored_root = reader.ReadPod<glm::mat4>();
  if (stored_root != root_transform.value) {
    return false;
  }
  gpu_vertices = reader.ReadVec<GpuMeshletVertex>();
  gpu_triangles = reader.ReadVec<GpuMeshletTriangle>();
  const uint64_t meshlet_count = reader.ReadPod<uint64_t>();
  if (!reader.Good() || meshlet_count > kMaxMeshingBufferCount) {
    return false;
  }
  meshlets.clear();
  meshlets.reserve(static_cast<size_t>(meshlet_count));
  for (uint64_t i = 0; i < meshlet_count; ++i) {
    meshlets.push_back(ReadVoronoiMesh(reader));
    if (!reader.Good()) {
      return false;
    }
  }
  neighbors = reader.ReadNestedInt();
  meshing_to_physics = reader.ReadSizeTVec();
  strand_to_segment = reader.ReadNestedSizeT();
  return reader.Good();
}

}  // namespace

// helper functions

/**
 * @brief Compute a 3D affine transformation that maps three coplanar source
 *        points to three coplanar target points, assuming an affine
 *        transformation that preserves the normalized plane normal direction.
 *
 * Given three non-collinear source points (p0, p1, p2) and their corresponding
 * non-collinear target points (q0, q1, q2), this function constructs the unique
 * affine transform T that satisfies:
 *
 *     T * vec4(p0, 1) = vec4(q0, 1)
 *     T * vec4(p1, 1) = vec4(q1, 1)
 *     T * vec4(p2, 1) = vec4(q2, 1)
 *
 * as well as:
 *
 *     T * n  = n'
 *
 * where n and n' are the normalized plane normals of the source and target
 * triangles, respectively. The normal direction is enforced to avoid the
 * underdetermined case that arises when all points lie in a plane.
 *
 * @note The returned transform maps points **from the source frame to the
 *       target frame**, i.e.:
 *
 *           T * vec4(p, 1) = vec4(q, 1)
 *
 *       for any point p lying in the same plane as (p0,p1,p2).
 *
 * @param p0 First source point in 3D.
 * @param p1 Second source point in 3D.
 * @param p2 Third source point in 3D.
 * @param q0 Corresponding target point to p0.
 * @param q1 Corresponding target point to p1.
 * @param q2 Corresponding target point to p2.
 *
 * @return glm::dmat4 The affine transformation matrix T such that T * p = q.
 *
 * @throws Undefined behavior if the three source or target points are collinear
 *         (i.e., they do not span a plane).
 */
glm::dmat4 ComputeAffineFromCoplanarPoints(const glm::vec3& p0, const glm::vec3& p1, const glm::vec3& p2,
                                           const glm::vec3& q0, const glm::vec3& q1, const glm::vec3& q2) {
  // --- Source basis ---
  glm::dvec3 u = p1 - p0;
  glm::dvec3 v = p2 - p0;
  glm::dvec3 n = glm::normalize(glm::cross(u, v));

  // --- Target basis ---
  glm::dvec3 up = q1 - q0;
  glm::dvec3 vp = q2 - q0;
  glm::dvec3 np = glm::normalize(glm::cross(up, vp));

  // (Optional) ensure consistent orientation.
  // If dot(n, np) < 0, flip np.
  if (glm::dot(n, np) < 0.0f)
    np = -np;

  // Build basis matrices B and B'
  glm::dmat3 B;
  B[0] = u;  // column 0
  B[1] = v;  // column 1
  B[2] = n;  // column 2

  glm::dmat3 Bp;
  Bp[0] = up;
  Bp[1] = vp;
  Bp[2] = np;

  // Linear part: A = B' * inverse(B)
  glm::mat3 A = Bp * glm::inverse(B);

  // Translation: t = q0 - A * p0
  glm::vec3 t = q0 - A * p0;

  // Assemble full 4x4 affine transform
  glm::dmat4 T(1.0f);
  T[0][0] = A[0][0];
  T[1][0] = A[1][0];
  T[2][0] = A[2][0];
  T[0][1] = A[0][1];
  T[1][1] = A[1][1];
  T[2][1] = A[2][1];
  T[0][2] = A[0][2];
  T[1][2] = A[1][2];
  T[2][2] = A[2][2];

  T[3] = glm::vec4(t, 1.0f);

  return T;
}

std::optional<std::array<size_t, 3>> FindNonCollinearTriple(std::function<glm::vec3(size_t)> get_point, size_t size,
                                                            float eps = 1e-6f) {
  if (size < 3)
    return std::optional<std::array<size_t, 3>>();

  // Step 1: choose p0
  size_t i0 = 0;
  size_t i1 = -1;
  size_t i2 = -1;
  // Step 2: choose p1 - must be distinct from p0

  for (int j = 1; j < size; ++j) {
    if (glm::length(get_point(j) - get_point(i0)) > eps) {
      i1 = j;
      break;
    }
  }
  if (i1 == -1)
    return std::optional<std::array<size_t, 3>>();  // all points identical

  // Step 3: find p2 that makes area > 0
  for (int k = i1 + 1; k < size; ++k) {
    glm::vec3 u = get_point(i1) - get_point(i0);
    glm::vec3 v = get_point(k) - get_point(i0);
    float area2 = glm::length(glm::cross(u, v));
    if (area2 > eps) {
      i2 = k;
      return std::optional<std::array<size_t, 3>>({i0, i1, i2});  // non-collinear triple found
    }
  }

  return std::optional<std::array<size_t, 3>>();  // all points collinear
}

std::optional<std::array<size_t, 2>> FindNonIdenticalPair(std::function<glm::vec3(size_t)> get_point, size_t size,
                                                          float eps = 1e-6f) {
  if (size < 2)
    return std::optional<std::array<size_t, 2>>();

  size_t i0 = 0;
  size_t i1 = -1;
  for (int j = 1; j < size; ++j) {
    if (glm::length(get_point(j) - get_point(i0)) > eps) {
      i1 = j;
      return std::optional<std::array<size_t, 2>>({i0, i1});
    }
  }
  return std::optional<std::array<size_t, 2>>();  // all points identical
}

glm::vec3 ProfileToModelCoordinates(const std::vector<std::vector<glm::dmat4>>& profile_to_model_transforms,
                                    glm::dvec3 point, float t, const std::vector<size_t>& branch_indices,
                                    float w = 1.0f) {
  size_t lower_section_index = static_cast<size_t>(std::max(0.0f, glm::floor(t)));

  size_t upper_section_index = std::min(profile_to_model_transforms.size() - 1, static_cast<size_t>(glm::ceil(t)));

  // check range
  auto coord_str = std::to_string(t);
  if (lower_section_index >= profile_to_model_transforms.size()) {
    std::cout << ("ProfileToModelCoordinates: lower bound of point z-coordinate out of range: " + coord_str).c_str()
              << std::endl;
  }
  if (upper_section_index >= profile_to_model_transforms.size()) {
    std::cout << ("ProfileToModelCoordinates: upper bound of point z-coordinate out of range: " + coord_str).c_str()
              << std::endl;
  }

  // only set second coordinate to 0 for points, not for normal vectors
  // TODO: I actually wanted to get rid of this coordinate swap at some point
  glm::vec4 local_pos(point[0], (1.0f - w) * point[2], point[1], w);
  size_t lower_branch_index = branch_indices[lower_section_index];
  glm::vec4 global_pos = profile_to_model_transforms[lower_section_index][lower_branch_index] * local_pos;

  if (upper_section_index != lower_section_index) {
    size_t upper_branch_index = branch_indices[upper_section_index];
    glm::vec4 upper_global_pos = profile_to_model_transforms[upper_section_index][upper_branch_index] * local_pos;
    float frac = static_cast<float>(t - static_cast<double>(lower_section_index));
    global_pos = glm::mix(global_pos, upper_global_pos, frac);
  }

  if (w == 0.0f) {
    global_pos = glm::normalize(global_pos);
  }

  return glm::vec3(global_pos);
}

glm::vec3 ToVec3(const glm::dvec3& a) {
  return glm::vec3(static_cast<float>(a[0]), static_cast<float>(a[1]), static_cast<float>(a[2]));
}

struct StrandCrossSectionGuidePoint {
  glm::dvec2 profile_position;
  SkeletonNodeHandle node_handle;
  StrandSegmentHandle segment_handle = -1;
  double root_distance = 0.0;
};

namespace {

/// Profile-plane to model-space transform for an internode cross-section at a given origin.
glm::dmat4 BuildInternodeProfileTransformAtOrigin(const StrandModelSkeleton& skeleton, SkeletonNodeHandle node_handle,
                                                    const glm::dvec3& origin) {
  if (node_handle < 0 || node_handle >= static_cast<SkeletonNodeHandle>(skeleton.PeekRawNodes().size())) {
    return glm::dmat4(1.0);
  }

  const auto& node = skeleton.PeekNode(node_handle);
  const glm::vec3 left_f = node.info.regulated_global_rotation * glm::vec3(1.0f, 0.0f, 0.0f);
  const glm::vec3 up_f = node.info.regulated_global_rotation * glm::vec3(0.0f, 1.0f, 0.0f);
  const glm::vec3 front_f = node.info.regulated_global_rotation * glm::vec3(0.0f, 0.0f, -1.0f);
  const glm::dvec3 left(left_f);
  const glm::dvec3 up(up_f);
  const glm::dvec3 front(front_f);
  const double radius = node.data.strand_radius;

  glm::dmat4 transform(1.0);
  transform[0] = glm::dvec4(left * radius, 0.0);
  transform[1] = glm::dvec4(front, 0.0);
  transform[2] = glm::dvec4(up * radius, 0.0);
  transform[3] = glm::dvec4(origin, 1.0);
  return transform;
}

/// Distal internode cross-section (segment ends), matching @ref StrandModel::ApplyProfile strand segment ends.
glm::dmat4 BuildInternodeProfileTransformAtEnd(const StrandModelSkeleton& skeleton, SkeletonNodeHandle node_handle) {
  if (node_handle < 0 || node_handle >= static_cast<SkeletonNodeHandle>(skeleton.PeekRawNodes().size())) {
    return glm::dmat4(1.0);
  }

  const auto& node = skeleton.PeekNode(node_handle);
  return BuildInternodeProfileTransformAtOrigin(skeleton, node_handle, glm::dvec3(node.info.GetGlobalEndPosition()));
}

/// Proximal internode cross-section (strand roots), matching @ref StrandModel::ApplyProfile for the first segment.
glm::dmat4 BuildInternodeProfileTransformAtStart(const StrandModelSkeleton& skeleton, SkeletonNodeHandle node_handle) {
  if (node_handle < 0 || node_handle >= static_cast<SkeletonNodeHandle>(skeleton.PeekRawNodes().size())) {
    return glm::dmat4(1.0);
  }

  const auto& node = skeleton.PeekNode(node_handle);
  return BuildInternodeProfileTransformAtOrigin(skeleton, node_handle, glm::dvec3(node.info.global_position));
}

glm::dmat4 MixAffineTransforms(const glm::dmat4& lower, const glm::dmat4& upper, double fraction) {
  const double t = glm::clamp(fraction, 0.0, 1.0);
  glm::dmat4 result(1.0);
  for (int column = 0; column < 4; ++column) {
    result[column] = glm::mix(lower[column], upper[column], t);
  }
  return result;
}

glm::dmat4 BuildInterpolatedInternodeTransformAtHeight(
    const StrandModelSkeleton& skeleton, const std::vector<StrandCrossSectionGuidePoint>& guide_points, size_t height) {
  if (guide_points.empty()) {
    return glm::dmat4(1.0);
  }

  const size_t clamped_height = std::min(height, guide_points.size() - 1);
  const SkeletonNodeHandle current_internode = guide_points[clamped_height].node_handle;

  size_t internode_run_start = clamped_height;
  while (internode_run_start > 0 && guide_points[internode_run_start - 1].node_handle == current_internode) {
    --internode_run_start;
  }

  size_t internode_run_end = clamped_height;
  while (internode_run_end + 1 < guide_points.size() &&
         guide_points[internode_run_end + 1].node_handle == current_internode) {
    ++internode_run_end;
  }

  const glm::dmat4 current_internode_end_transform = BuildInternodeProfileTransformAtEnd(skeleton, current_internode);

  const double end_distance = guide_points[internode_run_end].root_distance;

  glm::dmat4 lower_transform;
  glm::dmat4 upper_transform = current_internode_end_transform;
  double start_distance = 0.0;

  if (internode_run_start == 0) {
    // Strand roots are placed at the internode base (global_position), not at the parent's distal end.
    // See StrandModel::ApplyProfile when prev_segment_handle == -1.
    lower_transform = BuildInternodeProfileTransformAtStart(skeleton, current_internode);
    start_distance = guide_points[0].root_distance;
  } else {
    const SkeletonNodeHandle previous_internode = guide_points[internode_run_start - 1].node_handle;
    lower_transform = BuildInternodeProfileTransformAtEnd(skeleton, previous_internode);
    start_distance = guide_points[internode_run_start - 1].root_distance;
  }

  if (end_distance <= start_distance + glm::epsilon<double>()) {
    return current_internode_end_transform;
  }

  const double fraction = (guide_points[clamped_height].root_distance - start_distance) / (end_distance - start_distance);
  return MixAffineTransforms(lower_transform, upper_transform, fraction);
}

struct ProfilePlane {
  glm::dvec3 origin{0.0};
  glm::dvec3 normal{0.0, 1.0, 0.0};
};

struct CubicPlaneHit {
  double t = 0.0;
  glm::dvec3 point{0.0};
  int segment_index = 0;
};

constexpr double kPlaneSplineRootMargin = 1e-6;
constexpr double kPlaneSplineImagEps = 1e-8;
constexpr double kPlaneSplineResidualEps = 1e-4;
constexpr double kPlaneSplineDegenerateEps = 1e-12;

ProfilePlane ExtractProfilePlane(const glm::dmat4& transform) {
  ProfilePlane plane;
  plane.origin = glm::dvec3(transform[3]);
  const glm::dvec3 front = glm::dvec3(transform[1]);
  const double front_length = glm::length(front);
  plane.normal = front_length > kPlaneSplineDegenerateEps ? front / front_length : glm::dvec3(0.0, 1.0, 0.0);
  return plane;
}

glm::dvec2 WorldToProfile(const glm::dmat4& transform, const glm::dvec3& hit) {
  const glm::dvec4 local = glm::inverse(transform) * glm::dvec4(hit, 1.0);
  // ProfileToModelCoordinates uses local (x, 0, y) with the y/z swap convention.
  return glm::dvec2(local.x, local.z);
}

void StrandCubicPowerCoeffs(const glm::dvec3& v0, const glm::dvec3& v1, const glm::dvec3& v2, const glm::dvec3& v3,
                            glm::dvec3& c0, glm::dvec3& c1, glm::dvec3& c2, glm::dvec3& c3, double tension = 0.0) {
  // Meshing-only blend between:
  //   tension 0 → Strands::CubicInterpolation (does not pass through knots)
  //   tension 1 → Catmull-Rom Hermite (passes through v1 and v2)
  const double t = glm::clamp(tension, 0.0, 1.0);

  // Strands::CubicInterpolation power expansion.
  const glm::dvec3 p0 = (v2 + v0) / 6.0 + v1 * (4.0 / 6.0);
  const glm::dvec3 p1 = v2 - v0;
  const glm::dvec3 p2 = v2 - v1;
  const glm::dvec3 p3 = v3 - v1;
  const glm::dvec3 strands_c0 = p0;
  const glm::dvec3 strands_c1 = 0.5 * p1;
  const glm::dvec3 strands_c2 = -0.5 * p1 + p2;
  const glm::dvec3 strands_c3 = (1.0 / 6.0) * p1 - (2.0 / 3.0) * p2 + (1.0 / 6.0) * p3;

  // Catmull-Rom as Hermite through v1 → v2 with tangents 0.5*(v2-v0), 0.5*(v3-v1).
  const glm::dvec3 m0 = 0.5 * (v2 - v0);
  const glm::dvec3 m1 = 0.5 * (v3 - v1);
  const glm::dvec3 catmull_c0 = v1;
  const glm::dvec3 catmull_c1 = m0;
  const glm::dvec3 catmull_c2 = -3.0 * v1 - 2.0 * m0 + 3.0 * v2 - m1;
  const glm::dvec3 catmull_c3 = 2.0 * v1 + m0 - 2.0 * v2 + m1;

  c0 = glm::mix(strands_c0, catmull_c0, t);
  c1 = glm::mix(strands_c1, catmull_c1, t);
  c2 = glm::mix(strands_c2, catmull_c2, t);
  c3 = glm::mix(strands_c3, catmull_c3, t);
}

glm::dvec3 EvalStrandCubic(const glm::dvec3& c0, const glm::dvec3& c1, const glm::dvec3& c2, const glm::dvec3& c3,
                           double t) {
  return c0 + t * (c1 + t * (c2 + t * c3));
}

glm::dvec3 EvalStrandCubicDerivative(const glm::dvec3& c1, const glm::dvec3& c2, const glm::dvec3& c3, double t) {
  return c1 + t * (2.0 * c2 + t * 3.0 * c3);
}

double PlaneResidual(const glm::dvec3& point, const ProfilePlane& plane) {
  return glm::dot(plane.normal, point - plane.origin);
}

std::vector<double> CollectRealRootsIn01(const kinDS::Polynomial& poly) {
  std::vector<double> roots_in_01;
  if (poly.degree() <= 0) {
    return roots_in_01;
  }

  const Eigen::VectorXcd complex_roots = poly.roots();
  for (int i = 0; i < complex_roots.size(); ++i) {
    if (std::abs(complex_roots[i].imag()) > kPlaneSplineImagEps) {
      continue;
    }
    const double root = complex_roots[i].real();
    if (root >= -kPlaneSplineRootMargin && root <= 1.0 + kPlaneSplineRootMargin) {
      roots_in_01.push_back(glm::clamp(root, 0.0, 1.0));
    }
  }
  return roots_in_01;
}

double BisectPlaneRoot(const glm::dvec3& c0, const glm::dvec3& c1, const glm::dvec3& c2, const glm::dvec3& c3,
                       const ProfilePlane& plane, double t_min, double t_max, int iterations = 40) {
  double f_min = PlaneResidual(EvalStrandCubic(c0, c1, c2, c3, t_min), plane);
  double f_max = PlaneResidual(EvalStrandCubic(c0, c1, c2, c3, t_max), plane);
  if (f_min * f_max > 0.0) {
    return 0.5 * (t_min + t_max);
  }

  for (int i = 0; i < iterations; ++i) {
    const double t_mid = 0.5 * (t_min + t_max);
    const double f_mid = PlaneResidual(EvalStrandCubic(c0, c1, c2, c3, t_mid), plane);
    if (f_min * f_mid <= 0.0) {
      t_max = t_mid;
      f_max = f_mid;
    } else {
      t_min = t_mid;
      f_min = f_mid;
    }
  }
  return 0.5 * (t_min + t_max);
}

double NewtonPolishPlaneRoot(const glm::dvec3& c0, const glm::dvec3& c1, const glm::dvec3& c2, const glm::dvec3& c3,
                             const ProfilePlane& plane, double t, int iterations = 4) {
  for (int i = 0; i < iterations; ++i) {
    const glm::dvec3 point = EvalStrandCubic(c0, c1, c2, c3, t);
    const double f = PlaneResidual(point, plane);
    const double fp = glm::dot(plane.normal, EvalStrandCubicDerivative(c1, c2, c3, t));
    if (std::abs(fp) < kPlaneSplineDegenerateEps) {
      break;
    }
    t = glm::clamp(t - f / fp, 0.0, 1.0);
  }
  return t;
}

std::optional<CubicPlaneHit> IntersectCubicSegmentWithPlane(const glm::dvec3& v0, const glm::dvec3& v1,
                                                           const glm::dvec3& v2, const glm::dvec3& v3,
                                                           const ProfilePlane& plane, int segment_index,
                                                           double preferred_t = -1.0, double tension = 0.0) {
  glm::dvec3 c0, c1, c2, c3;
  StrandCubicPowerCoeffs(v0, v1, v2, v3, c0, c1, c2, c3, tension);

  const double f0 = PlaneResidual(EvalStrandCubic(c0, c1, c2, c3, 0.0), plane);
  const double f1 = PlaneResidual(EvalStrandCubic(c0, c1, c2, c3, 1.0), plane);

  Eigen::VectorXd coeffs(4);
  coeffs << (glm::dot(plane.normal, c0) - glm::dot(plane.normal, plane.origin)), glm::dot(plane.normal, c1),
      glm::dot(plane.normal, c2), glm::dot(plane.normal, c3);
  kinDS::Polynomial residual_poly(coeffs);

  std::vector<double> candidate_ts = CollectRealRootsIn01(residual_poly);

  // Degenerate / missed-root fallback: endpoints straddle the plane.
  if (candidate_ts.empty() && f0 * f1 <= 0.0) {
    candidate_ts.push_back(BisectPlaneRoot(c0, c1, c2, c3, plane, 0.0, 1.0));
  }

  // Near-coplanar segment: keep endpoint with smaller residual.
  if (candidate_ts.empty()) {
    if (std::abs(f0) <= kPlaneSplineResidualEps) {
      candidate_ts.push_back(0.0);
    } else if (std::abs(f1) <= kPlaneSplineResidualEps) {
      candidate_ts.push_back(1.0);
    } else {
      return std::nullopt;
    }
  }

  for (double& t : candidate_ts) {
    t = NewtonPolishPlaneRoot(c0, c1, c2, c3, plane, t);
  }

  auto score = [&](double t) {
    const double residual = std::abs(PlaneResidual(EvalStrandCubic(c0, c1, c2, c3, t), plane));
    const double preference = preferred_t >= 0.0 ? std::abs(t - preferred_t) : 0.0;
    return residual + 1e-3 * preference;
  };

  double best_t = candidate_ts.front();
  double best_score = score(best_t);
  for (size_t i = 1; i < candidate_ts.size(); ++i) {
    const double candidate_score = score(candidate_ts[i]);
    if (candidate_score < best_score) {
      best_score = candidate_score;
      best_t = candidate_ts[i];
    }
  }

  CubicPlaneHit hit;
  hit.t = best_t;
  hit.point = EvalStrandCubic(c0, c1, c2, c3, best_t);
  hit.segment_index = segment_index;
  if (std::abs(PlaneResidual(hit.point, plane)) > 10.0 * kPlaneSplineResidualEps && f0 * f1 > 0.0) {
    return std::nullopt;
  }
  return hit;
}

std::optional<CubicPlaneHit> IntersectStrandWithPlane(const StrandModelStrandGroup& strand_group, StrandHandle strand_handle,
                                                     const ProfilePlane& plane, int hint_segment_index,
                                                     double preferred_t = -1.0, double tension = 0.0) {
  const auto& strand = strand_group.PeekStrand(strand_handle);
  const auto& segment_handles = strand.PeekStrandSegmentHandles();
  if (segment_handles.empty()) {
    return std::nullopt;
  }

  const int segment_count = static_cast<int>(segment_handles.size());
  const int clamped_hint = glm::clamp(hint_segment_index, 0, segment_count - 1);

  auto try_segment = [&](int segment_index) -> std::optional<CubicPlaneHit> {
    glm::vec3 p0, p1, p2, p3;
    strand_group.GetPositionControlPoints(segment_handles[segment_index], p0, p1, p2, p3);
    return IntersectCubicSegmentWithPlane(glm::dvec3(p0), glm::dvec3(p1), glm::dvec3(p2), glm::dvec3(p3), plane,
                                          segment_index, preferred_t, tension);
  };

  // Search outward from the height hint so successive samples advance monotonically.
  if (auto hit = try_segment(clamped_hint)) {
    return hit;
  }

  for (int radius = 1; radius < segment_count; ++radius) {
    const int forward = clamped_hint + radius;
    if (forward < segment_count) {
      if (auto hit = try_segment(forward)) {
        return hit;
      }
    }
    const int backward = clamped_hint - radius;
    if (backward >= 0) {
      if (auto hit = try_segment(backward)) {
        return hit;
      }
    }
  }
  return std::nullopt;
}

glm::dvec2 SampleStrandProfileAtPlane(const StrandModelStrandGroup& strand_group, StrandHandle strand_handle,
                                      const glm::dmat4& transform, int& hint_segment_index,
                                      const glm::dvec2& fallback_profile, double preferred_t = -1.0,
                                      double tension = 0.0) {
  const ProfilePlane plane = ExtractProfilePlane(transform);
  const auto hit =
      IntersectStrandWithPlane(strand_group, strand_handle, plane, hint_segment_index, preferred_t, tension);
  if (!hit.has_value()) {
    return fallback_profile;
  }

  hint_segment_index = hit->segment_index;
  const glm::dvec2 profile = WorldToProfile(transform, hit->point);

#ifndef NDEBUG
  const glm::dvec4 reconstructed = transform * glm::dvec4(profile.x, 0.0, profile.y, 1.0);
  const double round_trip = std::abs(PlaneResidual(glm::dvec3(reconstructed), plane));
  if (round_trip > kPlaneSplineResidualEps) {
    EVOENGINE_WARNING("Plane-spline profile round-trip residual " << round_trip << " exceeds tolerance on strand "
                                                                  << strand_handle << " segment "
                                                                  << hit->segment_index);
  }
#endif

  return profile;
}

// Legacy affine-fit helpers (retained for comparison; no longer used in InitData).
[[maybe_unused]] glm::dmat4 FitGlobalProfileToModelTransformAtHeight(
    int h, const std::vector<std::vector<size_t>>& sorted_segments,
    const DtsStrandGroup& uniformly_subdivided_strand_group) {
  const auto& segments = sorted_segments[h == 0 ? 0 : (h - 1)];

  std::function<glm::vec3(size_t)> get_point = [&](size_t idx) {
    size_t segment_handle = segments[idx];
    const auto& segment_data = uniformly_subdivided_strand_group.PeekStrandSegmentData(segment_handle);
    return glm::vec3(segment_data.profile_position.x, 0.0f, segment_data.profile_position.y);
  };

  std::optional<std::array<size_t, 3>> triple_opt = FindNonCollinearTriple(get_point, segments.size());

  glm::vec3 p0_profile, p1_profile, p2_profile;
  glm::vec3 p0_global, p1_global, p2_global;

  if (!triple_opt.has_value()) {
    std::optional<std::array<size_t, 2>> pair_opt = FindNonIdenticalPair(get_point, segments.size());

    StrandSegmentHandle first_segment_handle = segments[0];
    const auto& first_segment_data = uniformly_subdivided_strand_group.PeekStrandSegmentData(first_segment_handle);
    const auto& first_segment = uniformly_subdivided_strand_group.PeekStrandSegment(first_segment_handle);
    const auto& strand = uniformly_subdivided_strand_group.PeekStrand(first_segment.GetStrandHandle());
    StrandSegmentHandle next_segment_handle = first_segment.GetNextHandle();

    float normal_sign = 1.0f;
    glm::vec3 normal_global;

    if (h != 0) {
      if (next_segment_handle == -1) {
        next_segment_handle = first_segment.GetPrevHandle();
        normal_sign = -1.0f;
      }

      const auto& next_segment = uniformly_subdivided_strand_group.PeekStrandSegment(next_segment_handle);
      normal_global = normal_sign * glm::normalize(next_segment.end_position - first_segment.end_position);
    } else {
      normal_global = normal_sign * glm::normalize(strand.start_position - first_segment.end_position);
    }

    glm::vec3 u_global;
    glm::vec3 v_global;
    glm::vec3 u_profile;
    glm::vec3 v_profile;

    if (!pair_opt.has_value()) {
      u_global = glm::normalize(glm::cross(normal_global, glm::vec3(1.0f, 0.0f, 0.0f)));
      if (glm::length(u_global) < glm::epsilon<float>()) {
        u_global = glm::normalize(glm::cross(normal_global, glm::vec3(0.0f, 0.0f, 1.0f)));
      }
      v_global = glm::normalize(glm::cross(normal_global, u_global));

      if (h != 0) {
        p0_global = first_segment.end_position;
      } else {
        p0_global = strand.start_position;
      }
      p1_global = p0_global + u_global;
      p0_profile = glm::vec3(first_segment_data.profile_position.x, 0.0f, first_segment_data.profile_position.y);
      p1_profile = p0_profile + glm::vec3(1.0f, 0.0f, 0.0f);
    } else {
      const auto& pair = pair_opt.value();
      size_t p0_idx = pair[0];
      size_t p1_idx = pair[1];

      const glm::vec2& p0_profile_2d =
          uniformly_subdivided_strand_group.PeekStrandSegmentData(segments[p0_idx]).profile_position;
      p0_profile = glm::vec3(p0_profile_2d.x, 0.0f, p0_profile_2d.y);

      const glm::vec2& p1_profile_2d =
          uniformly_subdivided_strand_group.PeekStrandSegmentData(segments[p1_idx]).profile_position;
      p1_profile = glm::vec3(p1_profile_2d.x, 0.0f, p1_profile_2d.y);

      if (h != 0) {
        p0_global = uniformly_subdivided_strand_group.PeekStrandSegment(segments[p0_idx]).end_position;
        p1_global = uniformly_subdivided_strand_group.PeekStrandSegment(segments[p1_idx]).end_position;
      } else {
        p0_global =
            uniformly_subdivided_strand_group
                .PeekStrand(uniformly_subdivided_strand_group.PeekStrandSegment(segments[p0_idx]).GetStrandHandle())
                .start_position;
        p1_global =
            uniformly_subdivided_strand_group
                .PeekStrand(uniformly_subdivided_strand_group.PeekStrandSegment(segments[p1_idx]).GetStrandHandle())
                .start_position;
      }

      u_profile = glm::normalize(p1_profile - p0_profile);
      u_global = glm::normalize(p1_global - p0_global);
      v_profile = glm::normalize(glm::cross(normal_global, u_profile));
      v_global = glm::normalize(glm::cross(normal_global, u_global));
    }

    p2_global = p0_global + v_global;
    p2_profile = p0_profile + glm::vec3(0.0f, 0.0f, 1.0f);
  } else {
    const auto& triple = triple_opt.value();

    const glm::vec2 p0_profile_2d =
        uniformly_subdivided_strand_group.PeekStrandSegmentData(segments[triple[0]]).profile_position;
    p0_profile = glm::vec3(p0_profile_2d.x, 0.0f, p0_profile_2d.y);

    const glm::vec2& p1_profile_2d =
        uniformly_subdivided_strand_group.PeekStrandSegmentData(segments[triple[1]]).profile_position;
    p1_profile = glm::vec3(p1_profile_2d.x, 0.0f, p1_profile_2d.y);

    const glm::vec2& p2_profile_2d =
        uniformly_subdivided_strand_group.PeekStrandSegmentData(segments[triple[2]]).profile_position;
    p2_profile = glm::vec3(p2_profile_2d.x, 0.0f, p2_profile_2d.y);

    if (h != 0) {
      p0_global = uniformly_subdivided_strand_group.PeekStrandSegment(segments[triple[0]]).end_position;
      p1_global = uniformly_subdivided_strand_group.PeekStrandSegment(segments[triple[1]]).end_position;
      p2_global = uniformly_subdivided_strand_group.PeekStrandSegment(segments[triple[2]]).end_position;
    } else {
      p0_global =
          uniformly_subdivided_strand_group
              .PeekStrand(uniformly_subdivided_strand_group.PeekStrandSegment(segments[triple[0]]).GetStrandHandle())
              .start_position;
      p1_global =
          uniformly_subdivided_strand_group
              .PeekStrand(uniformly_subdivided_strand_group.PeekStrandSegment(segments[triple[1]]).GetStrandHandle())
              .start_position;
      p2_global =
          uniformly_subdivided_strand_group
              .PeekStrand(uniformly_subdivided_strand_group.PeekStrandSegment(segments[triple[2]]).GetStrandHandle())
              .start_position;
    }
  }

  return ComputeAffineFromCoplanarPoints(p0_profile, p1_profile, p2_profile, p0_global, p1_global, p2_global);
}

[[maybe_unused]] glm::dmat4 FitBranchProfileToModelTransformAtHeight(
    int h, size_t branch_index, const std::vector<std::vector<std::vector<size_t>>>& strands_by_branch_id,
    const std::vector<std::vector<StrandCrossSectionGuidePoint>>& strand_guide_points,
    const DtsStrandGroup& uniformly_subdivided_strand_group) {
  const auto& strand_ids = strands_by_branch_id[h][branch_index];
  if (strand_ids.empty()) {
    return glm::dmat4(1.0);
  }

  std::function<glm::vec3(size_t)> get_point = [&](size_t idx) {
    size_t strand_id = strand_ids[idx];
    const auto& segment_data =
        uniformly_subdivided_strand_group.PeekStrandSegmentData(strand_guide_points[strand_id][h].segment_handle);
    return glm::vec3(segment_data.profile_position.x, 0.0f, segment_data.profile_position.y);
  };

  std::optional<std::array<size_t, 3>> triple_opt = FindNonCollinearTriple(get_point, strand_ids.size());

  glm::vec3 p0_profile, p1_profile, p2_profile;
  glm::vec3 p0_global, p1_global, p2_global;

  if (!triple_opt.has_value()) {
    std::optional<std::array<size_t, 2>> pair_opt = FindNonIdenticalPair(get_point, strand_ids.size());

    StrandSegmentHandle first_segment_handle = strand_guide_points[strand_ids[0]][h].segment_handle;
    const auto& first_segment_data = uniformly_subdivided_strand_group.PeekStrandSegmentData(first_segment_handle);
    const auto& first_segment = uniformly_subdivided_strand_group.PeekStrandSegment(first_segment_handle);
    const auto& strand = uniformly_subdivided_strand_group.PeekStrand(first_segment.GetStrandHandle());
    StrandSegmentHandle next_segment_handle = first_segment.GetNextHandle();

    float normal_sign = 1.0f;
    glm::vec3 normal_global;

    if (h != 0) {
      if (next_segment_handle == -1) {
        next_segment_handle = first_segment.GetPrevHandle();
        normal_sign = -1.0f;
      }

      const auto& next_segment = uniformly_subdivided_strand_group.PeekStrandSegment(next_segment_handle);
      normal_global = normal_sign * glm::normalize(next_segment.end_position - first_segment.end_position);
    } else {
      normal_global = normal_sign * glm::normalize(strand.start_position - first_segment.end_position);
    }

    glm::vec3 u_global;
    glm::vec3 v_global;

    if (!pair_opt.has_value()) {
      u_global = glm::normalize(glm::cross(normal_global, glm::vec3(1.0f, 0.0f, 0.0f)));
      if (glm::length(u_global) < glm::epsilon<float>()) {
        u_global = glm::normalize(glm::cross(normal_global, glm::vec3(0.0f, 0.0f, 1.0f)));
      }
      v_global = glm::normalize(glm::cross(normal_global, u_global));

      if (h != 0) {
        p0_global = first_segment.end_position;
      } else {
        p0_global = strand.start_position;
      }
      p1_global = p0_global + u_global;
      p0_profile = glm::vec3(first_segment_data.profile_position.x, 0.0f, first_segment_data.profile_position.y);
      p1_profile = p0_profile + glm::vec3(1.0f, 0.0f, 0.0f);
    } else {
      const auto& pair = pair_opt.value();
      const glm::vec2& p0_profile_2d = uniformly_subdivided_strand_group
                                           .PeekStrandSegmentData(strand_guide_points[strand_ids[pair[0]]][h].segment_handle)
                                           .profile_position;
      p0_profile = glm::vec3(p0_profile_2d.x, 0.0f, p0_profile_2d.y);

      const glm::vec2& p1_profile_2d = uniformly_subdivided_strand_group
                                           .PeekStrandSegmentData(strand_guide_points[strand_ids[pair[1]]][h].segment_handle)
                                           .profile_position;
      p1_profile = glm::vec3(p1_profile_2d.x, 0.0f, p1_profile_2d.y);

      if (h != 0) {
        p0_global = uniformly_subdivided_strand_group
                        .PeekStrandSegment(strand_guide_points[strand_ids[pair[0]]][h].segment_handle)
                        .end_position;
        p1_global = uniformly_subdivided_strand_group
                        .PeekStrandSegment(strand_guide_points[strand_ids[pair[1]]][h].segment_handle)
                        .end_position;
      } else {
        p0_global = uniformly_subdivided_strand_group
                        .PeekStrand(uniformly_subdivided_strand_group
                                        .PeekStrandSegment(strand_guide_points[strand_ids[pair[0]]][h].segment_handle)
                                        .GetStrandHandle())
                        .start_position;
        p1_global = uniformly_subdivided_strand_group
                        .PeekStrand(uniformly_subdivided_strand_group
                                        .PeekStrandSegment(strand_guide_points[strand_ids[pair[1]]][h].segment_handle)
                                        .GetStrandHandle())
                        .start_position;
      }

      glm::vec3 u_profile = glm::normalize(p1_profile - p0_profile);
      (void)u_profile;
      glm::vec3 u_global_dir = glm::normalize(p1_global - p0_global);
      v_global = glm::normalize(glm::cross(normal_global, u_global_dir));
    }

    p2_global = p0_global + v_global;
    p2_profile = p0_profile + glm::vec3(0.0f, 0.0f, 1.0f);
  } else {
    const auto& triple = triple_opt.value();

    const glm::vec2 p0_profile_2d = uniformly_subdivided_strand_group
                                        .PeekStrandSegmentData(strand_guide_points[strand_ids[triple[0]]][h].segment_handle)
                                        .profile_position;
    p0_profile = glm::vec3(p0_profile_2d.x, 0.0f, p0_profile_2d.y);

    const glm::vec2& p1_profile_2d = uniformly_subdivided_strand_group
                                       .PeekStrandSegmentData(strand_guide_points[strand_ids[triple[1]]][h].segment_handle)
                                       .profile_position;
    p1_profile = glm::vec3(p1_profile_2d.x, 0.0f, p1_profile_2d.y);

    const glm::vec2& p2_profile_2d = uniformly_subdivided_strand_group
                                       .PeekStrandSegmentData(strand_guide_points[strand_ids[triple[2]]][h].segment_handle)
                                       .profile_position;
    p2_profile = glm::vec3(p2_profile_2d.x, 0.0f, p2_profile_2d.y);

    if (h != 0) {
      p0_global = uniformly_subdivided_strand_group
                      .PeekStrandSegment(strand_guide_points[strand_ids[triple[0]]][h].segment_handle)
                      .end_position;
      p1_global = uniformly_subdivided_strand_group
                      .PeekStrandSegment(strand_guide_points[strand_ids[triple[1]]][h].segment_handle)
                      .end_position;
      p2_global = uniformly_subdivided_strand_group
                      .PeekStrandSegment(strand_guide_points[strand_ids[triple[2]]][h].segment_handle)
                      .end_position;
    } else {
      p0_global = uniformly_subdivided_strand_group
                      .PeekStrand(uniformly_subdivided_strand_group
                                      .PeekStrandSegment(strand_guide_points[strand_ids[triple[0]]][h].segment_handle)
                                      .GetStrandHandle())
                      .start_position;
      p1_global = uniformly_subdivided_strand_group
                      .PeekStrand(uniformly_subdivided_strand_group
                                      .PeekStrandSegment(strand_guide_points[strand_ids[triple[1]]][h].segment_handle)
                                      .GetStrandHandle())
                      .start_position;
      p2_global = uniformly_subdivided_strand_group
                      .PeekStrand(uniformly_subdivided_strand_group
                                      .PeekStrandSegment(strand_guide_points[strand_ids[triple[2]]][h].segment_handle)
                                      .GetStrandHandle())
                      .start_position;
    }
  }

  return ComputeAffineFromCoplanarPoints(p0_profile, p1_profile, p2_profile, p0_global, p1_global, p2_global);
}

}  // namespace

// kinDS::VoronoiMesh DsKineticVoronoiMeshing::TransformBoundaryMesh(
//     const kinDS::VoronoiMesh& boundary_mesh,
//     const std::vector<std::vector<glm::dmat4>>& transforms_by_height_and_branch,
//     const std::vector<std::vector<glm::dmat4>>& normal_transforms_by_height_and_branch,
//     const GlobalTransform& root_transform, const std::vector<std::vector<size_t>>& branch_indices,
//     const std::vector<size_t>& boundary_vertex_to_strand_id) {
//   // Boundary remap export is intentionally disabled for now.
//   return {};
// }


void DsKineticVoronoiMeshing::WriteIntersectionStatisticsCsv(
    const std::filesystem::path& base_csv_path,
    const std::vector<std::pair<std::string, IntersectionRunStats>>& rows) {
  if (rows.empty()) {
    return;
  }
  const std::filesystem::path csv_path = kinDS::Statistics::timestampedCsvPath(base_csv_path);
  std::ofstream out(csv_path);
  if (!out) {
    EVOENGINE_ERROR("Failed to write intersection statistics CSV " << csv_path.string());
    return;
  }
  out << "name,inside_meshlets,intersecting_meshlets,outside_meshlets,input_poly_count,runtime_s\n";
  out << std::setprecision(std::numeric_limits<double>::max_digits10);
  size_t total_polys = 0;
  double total_runtime = 0.0;
  for (const auto& [name, stats] : rows) {
    out << name << ',' << stats.inside_meshlets << ',' << stats.intersecting_meshlets << ',' << stats.outside_meshlets
        << ',' << stats.input_poly_count << ',' << stats.runtime_seconds << '\n';
    total_polys += stats.input_poly_count;
    total_runtime += stats.runtime_seconds;
  }
  if (rows.size() > 1) {
    out << "total,,,," << total_polys << ',' << total_runtime << '\n';
  }
  EVOENGINE_LOG("Wrote intersection statistics CSV to " << csv_path.string());
}

bool DsKineticVoronoiMeshing::HasMeshedSegmentMeshlets() const {
  return tree_mesher_ && !segment_meshlets_.empty();
}

bool DsKineticVoronoiMeshing::LoadIntersectionSetup(const std::shared_ptr<Scene>& scene, const Entity& owner,
                                                    const std::filesystem::path& yaml_path) {
  if (!scene || !scene->IsEntityValid(owner)) {
    EVOENGINE_ERROR("Load intersection setup: invalid owner entity.");
    return false;
  }
  if (!std::filesystem::exists(yaml_path)) {
    EVOENGINE_ERROR("Load intersection setup: file not found: " << yaml_path.string());
    return false;
  }

  const auto find_group_entity = [](const std::shared_ptr<Scene>& s, const Entity& o) -> Entity {
    for (const auto& child : s->GetChildren(o)) {
      if (s->HasPrivateComponent<DsIntersectionBoundaryMeshGroup>(child)) {
        return child;
      }
    }
    return Entity{};
  };
  const auto ensure_group_entity = [&](const std::shared_ptr<Scene>& s, const Entity& o) -> Entity {
    Entity group = find_group_entity(s, o);
    if (s->IsEntityValid(group)) {
      return group;
    }
    group = s->CreateEntity("Intersection Meshes");
    s->SetParent(group, o);
    GlobalTransform group_gt{};
    group_gt.value = glm::mat4(1.0f);
    s->SetDataComponent(group, group_gt);
    s->GetOrSetPrivateComponent<DsIntersectionBoundaryMeshGroup>(group);
    return group;
  };
  const auto resolve_obj_path = [&](const std::filesystem::path& obj_path) -> std::filesystem::path {
    if (obj_path.is_absolute() && std::filesystem::exists(obj_path)) {
      return obj_path;
    }
    const auto from_assets = ProjectManager::GetAssetsFolderPath() / obj_path;
    if (std::filesystem::exists(from_assets)) {
      return from_assets;
    }
    const auto from_yaml_dir = yaml_path.parent_path() / obj_path;
    if (std::filesystem::exists(from_yaml_dir)) {
      return from_yaml_dir;
    }
    return obj_path;
  };

  try {
    const YAML::Node root = YAML::Load(FileUtils::LoadFileAsString(yaml_path));
    if (!root["intersection_meshes"]) {
      EVOENGINE_ERROR("Load intersection setup: no 'intersection_meshes' key in file.");
      return false;
    }
    const Entity group = ensure_group_entity(scene, owner);
    if (root["group_transform"]) {
      GlobalTransform group_gt{};
      group_gt.value = root["group_transform"].as<glm::mat4>();
      scene->SetDataComponent(group, group_gt);
    }
    size_t loaded_count = 0;
    for (const auto& entry : root["intersection_meshes"]) {
      if (!entry["obj_path"] || !entry["transform"]) {
        continue;
      }
      const std::filesystem::path obj_path = resolve_obj_path(entry["obj_path"].as<std::string>());
      const glm::mat4 transform_value = entry["transform"].as<glm::mat4>();
      try {
        kinDS::VoronoiMesh loaded_mesh = kinDS::ObjExporter::readMesh(obj_path);
        const auto child = scene->CreateEntity("Intersection Mesh (" + obj_path.stem().string() + ")");
        scene->SetParent(child, group);
        GlobalTransform child_gt{};
        child_gt.value = transform_value;
        scene->SetDataComponent(child, child_gt);
        const auto ibm = scene->GetOrSetPrivateComponent<DsIntersectionBoundaryMesh>(child).lock();
        if (ibm) {
          ibm->LoadMesh(std::move(loaded_mesh), obj_path);
          ++loaded_count;
        }
      } catch (const std::exception& ex) {
        EVOENGINE_ERROR("Failed to load OBJ '" << obj_path.string() << "': " << ex.what());
      }
    }
    EVOENGINE_LOG("Loaded intersection setup from " << yaml_path.string() << " (" << loaded_count << " mesh(es)).");
    return loaded_count > 0;
  } catch (const std::exception& ex) {
    EVOENGINE_ERROR("Failed to parse intersection setup YAML: " << ex.what());
    return false;
  }
}

bool DsKineticVoronoiMeshing::IntersectMeshletsWithBoundary(const kinDS::VoronoiMesh& raw_mesh,
                                                             const GlobalTransform& boundary_world_transform,
                                                             const GlobalTransform& tree_world_transform,
                                                             IntersectionRunStats* stats) {
  if (!HasMeshedSegmentMeshlets()) {
    EVOENGINE_ERROR("Intersect: no meshed segment meshlets available. Run meshing first.");
    return false;
  }
  if (raw_mesh.getTriangleCount() == 0) {
    EVOENGINE_ERROR("Intersect: intersection boundary mesh is empty.");
    return false;
  }
  if (!strand_tree) {
    EVOENGINE_ERROR("Intersect: strand tree is missing.");
    return false;
  }

  // Meshlets live in tree-local space (the kinDS algorithm produces geometry relative to the tree
  // origin, before any world transform is applied). The boundary entity's GlobalTransform is in world
  // space. Convert the boundary into tree-local space using the tree entity's *current* world transform.
  kinDS::VoronoiMesh boundary_mesh = raw_mesh;
  const glm::dmat4 clip_transform =
      glm::inverse(glm::dmat4(tree_world_transform.value)) * glm::dmat4(boundary_world_transform.value);
  boundary_mesh.applyTransform(clip_transform);

  // Restore pristine meshlets so Intersect can be re-run after moving the boundary.
  tree_mesher_->getSegmentMeshlets() = segment_meshlets_;
  tree_mesher_->getMeshingNeighborIndices() = meshing_neighbor_indices_;

  const bool previous_fix_missing_meshes = tree_mesher_->getSettings().fix_missing_meshes;
  const bool previous_keep_original_on_failure = tree_mesher_->getSettings().keep_original_on_intersection_failure;
  const bool previous_prefer_meshlet_uv_on_seam = tree_mesher_->getSettings().intersection_prefer_meshlet_uv_on_seam;
  const bool previous_boundary_faces_interior_uv = tree_mesher_->getSettings().intersection_boundary_faces_interior_uv;
  tree_mesher_->getSettings().fix_missing_meshes = meshing_settings.intersection_boundary_fix_missing_meshes;
  tree_mesher_->getSettings().keep_original_on_intersection_failure =
      meshing_settings.intersection_keep_original_on_failure;
  tree_mesher_->getSettings().intersection_prefer_meshlet_uv_on_seam =
      meshing_settings.intersection_prefer_meshlet_uv_on_seam;
  tree_mesher_->getSettings().intersection_boundary_faces_interior_uv =
      meshing_settings.intersection_boundary_faces_interior_uv;
  tree_mesher_->getSettings().export_separate_contributor_objects =
      meshing_settings.export_separate_contributor_objects;
  EVOENGINE_LOG("Intersecting meshlets with boundary (" << boundary_mesh.getTriangleCount() << " triangles)...");
  const auto intersection_started = std::chrono::steady_clock::now();
  const kinDS::TreeMesher::BoundaryTruncateResult truncate_result = tree_mesher_->truncateToBoundary(boundary_mesh);
  const double intersection_seconds =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - intersection_started).count();
  if (stats) {
    stats->inside_meshlets = truncate_result.inside_count;
    stats->intersecting_meshlets = truncate_result.intersecting_count;
    stats->outside_meshlets = truncate_result.outside_count;
    stats->input_poly_count = boundary_mesh.getTriangleCount();
    stats->runtime_seconds = intersection_seconds;
  }
  tree_mesher_->getSettings().fix_missing_meshes = previous_fix_missing_meshes;
  tree_mesher_->getSettings().keep_original_on_intersection_failure = previous_keep_original_on_failure;
  tree_mesher_->getSettings().intersection_prefer_meshlet_uv_on_seam = previous_prefer_meshlet_uv_on_seam;
  tree_mesher_->getSettings().intersection_boundary_faces_interior_uv = previous_boundary_faces_interior_uv;

  segment_meshlet_vertices.clear();
  segment_meshlet_triangles.clear();
  PopulateGpuMeshletBuffers(tree_mesher_->getSegmentMeshlets(), strand_tree->getPhysicsStrandToSegmentIndices(),
                            tree_mesher_->getMeshingStrandToSegmentIndices(), tree_mesher_->getMeshingNeighborIndices(),
                            tree_mesher_->getMeshingToPhysicsSegmentIndices(), meshlets_root_transform_);
  Upload();
  DownloadPhysicsSegmentsAndPairs();
  RestoreDeactivatedPhysicsSegments();
  DeactivateOutsidePhysicsSegments(truncate_result.outside_meshlet_indices);
  UploadPhysicsSegmentsAndPairs();
  UpdateBindings();
  EVOENGINE_LOG("Intersection complete. GPU meshlet buffers updated (" << segment_meshlet_vertices.size()
                                                                       << " vertices, "
                                                                       << segment_meshlet_triangles.size()
                                                                       << " triangles). Deactivated "
                                                                       << deactivated_physics_segment_indices_.size()
                                                                       << " OUTSIDE physics segment(s).");
  return true;
}

bool DsKineticVoronoiMeshing::ResetMeshletsToGpu() {
  if (!HasMeshedSegmentMeshlets()) {
    EVOENGINE_ERROR("Reset meshlets: no meshed segment meshlets available. Run meshing first.");
    return false;
  }
  if (!strand_tree) {
    EVOENGINE_ERROR("Reset meshlets: strand tree is missing.");
    return false;
  }

  tree_mesher_->getSegmentMeshlets() = segment_meshlets_;
  tree_mesher_->getMeshingNeighborIndices() = meshing_neighbor_indices_;

  segment_meshlet_vertices.clear();
  segment_meshlet_triangles.clear();
  PopulateGpuMeshletBuffers(segment_meshlets_, strand_tree->getPhysicsStrandToSegmentIndices(),
                            tree_mesher_->getMeshingStrandToSegmentIndices(), meshing_neighbor_indices_,
                            tree_mesher_->getMeshingToPhysicsSegmentIndices(), meshlets_root_transform_);
  Upload();
  DownloadPhysicsSegmentsAndPairs();
  RestoreDeactivatedPhysicsSegments();
  UploadPhysicsSegmentsAndPairs();
  UpdateBindings();
  EVOENGINE_LOG("Reset meshlets to GPU (no intersection). " << segment_meshlet_vertices.size() << " vertices, "
                                                            << segment_meshlet_triangles.size() << " triangles.");
  return true;
}

void DsKineticVoronoiMeshing::DownloadPhysicsSegmentsAndPairs() {
  if (!dynamic_strands) {
    return;
  }
  if (!dynamic_strands->segments.empty()) {
    dynamic_strands->device_segments_buffer->DownloadVector(dynamic_strands->segments,
                                                            dynamic_strands->segments.size());
  }
  if (!dynamic_strands->segment_pairs.empty()) {
    dynamic_strands->device_segment_pairs_buffer->DownloadVector(dynamic_strands->segment_pairs,
                                                                dynamic_strands->segment_pairs.size());
  }
}

void DsKineticVoronoiMeshing::UploadPhysicsSegmentsAndPairs() {
  if (!dynamic_strands) {
    return;
  }
  dynamic_strands->device_segments_buffer->UploadVector(dynamic_strands->segments);
  dynamic_strands->device_segments_buffer->SetDebugName("Segments Buffer");
  dynamic_strands->device_segment_pairs_buffer->UploadVector(dynamic_strands->segment_pairs);
  dynamic_strands->device_segment_pairs_buffer->SetDebugName("Segment Pairs Buffer");
}

void DsKineticVoronoiMeshing::RestoreDeactivatedPhysicsSegments() {
  if (!dynamic_strands) {
    deactivated_physics_segment_indices_.clear();
    deactivated_pair_integrities_.clear();
    return;
  }
  auto& segments = dynamic_strands->segments;
  for (const int segment_index : deactivated_physics_segment_indices_) {
    if (segment_index < 0 || static_cast<size_t>(segment_index) >= segments.size()) {
      continue;
    }
    segments[segment_index].particle0.disabled = 0;
    segments[segment_index].particle1.disabled = 0;
  }
  auto& segment_pairs = dynamic_strands->segment_pairs;
  for (const auto& saved : deactivated_pair_integrities_) {
    if (saved.pair_handle < 0 || static_cast<size_t>(saved.pair_handle) >= segment_pairs.size()) {
      continue;
    }
    auto& pair = segment_pairs[saved.pair_handle];
    pair.connectivity_integrity = saved.connectivity_integrity;
    pair.bend_twist_bundle_integrity = saved.bend_twist_bundle_integrity;
  }
  deactivated_physics_segment_indices_.clear();
  deactivated_pair_integrities_.clear();
}

void DsKineticVoronoiMeshing::DeactivateOutsidePhysicsSegments(
    const std::vector<size_t>& outside_meshing_indices) {
  deactivated_physics_segment_indices_.clear();
  deactivated_pair_integrities_.clear();
  if (!dynamic_strands || !tree_mesher_ || outside_meshing_indices.empty()) {
    return;
  }

  const auto& meshing_to_physics = tree_mesher_->getMeshingToPhysicsSegmentIndices();
  auto& segments = dynamic_strands->segments;
  auto& segment_pairs = dynamic_strands->segment_pairs;
  const auto& segment_data_list = dynamic_strands->segment_data_list;

  std::unordered_set<int> seen_pairs;
  seen_pairs.reserve(outside_meshing_indices.size() * 4);

  for (const size_t meshing_index : outside_meshing_indices) {
    if (meshing_index >= meshing_to_physics.size()) {
      continue;
    }
    const size_t physics_id = meshing_to_physics[meshing_index];
    if (physics_id == static_cast<size_t>(-1) || physics_id >= segments.size()) {
      continue;
    }
    const int physics_segment_id = static_cast<int>(physics_id);
    segments[physics_id].particle0.disabled = 1;
    segments[physics_id].particle1.disabled = 1;
    deactivated_physics_segment_indices_.push_back(physics_segment_id);

    if (physics_id >= segment_data_list.size()) {
      continue;
    }
    for (const int pair_handle : segment_data_list[physics_id].pair_handles) {
      if (pair_handle < 0 || static_cast<size_t>(pair_handle) >= segment_pairs.size()) {
        continue;
      }
      if (!seen_pairs.insert(pair_handle).second) {
        continue;
      }
      auto& pair = segment_pairs[pair_handle];
      DeactivatedPairIntegrity saved;
      saved.pair_handle = pair_handle;
      saved.connectivity_integrity = pair.connectivity_integrity;
      saved.bend_twist_bundle_integrity = pair.bend_twist_bundle_integrity;
      deactivated_pair_integrities_.push_back(saved);
      pair.connectivity_integrity = 0.f;
      pair.bend_twist_bundle_integrity = 0.f;
    }
  }
}

void DsKineticVoronoiMeshing::RecomputeSegmentPairs(const kinDS::TreeMesher& tree_mesher) {
  const auto& meshes = tree_mesher.getSegmentMeshlets();
  const auto& meshing_neighbor_indices = tree_mesher.getMeshingNeighborIndices();
  const auto& meshing_to_physics_segment_indices = tree_mesher.getMeshingToPhysicsSegmentIndices();
  const auto& meshing_strand_to_segment_indices = tree_mesher.getMeshingStrandToSegmentIndices();
  const auto& physics_strand_to_segment_indices = strand_tree->getPhysicsStrandToSegmentIndices();

  auto& segment_data_list = dynamic_strands->segment_data_list;
  auto& segment_pairs = dynamic_strands->segment_pairs;
  auto& strands = dynamic_strands->strands;

  segment_pairs.clear();
  for (auto& segment_data : segment_data_list) {
    for (int& pair_handle : segment_data.pair_handles) {
      pair_handle = -1;
    }
  }

  // Collect physics-segment neighbors implied by meshlet face adjacency.
  const auto collect_physics_neighbors = [&](const size_t meshing_segment_id) {
    std::unordered_set<int> neighbors;
    if (meshing_segment_id >= meshes.size() || meshing_segment_id >= meshing_neighbor_indices.size()) {
      return neighbors;
    }
    const auto& triangles = meshes[meshing_segment_id].getTriangles();
    const auto& face_neighbors = meshing_neighbor_indices[meshing_segment_id];
    for (size_t triangle_vertex_index = 0; triangle_vertex_index < triangles.size(); triangle_vertex_index += 3) {
      const size_t face_index = triangle_vertex_index / 3;
      if (face_index >= face_neighbors.size()) {
        continue;
      }
      const int meshing_neighbor_segment_index = face_neighbors[face_index];
      if (meshing_neighbor_segment_index < 0 ||
          static_cast<size_t>(meshing_neighbor_segment_index) >= meshing_to_physics_segment_indices.size()) {
        continue;
      }
      const int physics_neighbor =
          static_cast<int>(meshing_to_physics_segment_indices[static_cast<size_t>(meshing_neighbor_segment_index)]);
      if (physics_neighbor >= 0) {
        neighbors.insert(physics_neighbor);
      }
    }
    return neighbors;
  };

  // Pass 1: same-strand vertical pairs.
  // Convention (matches DynamicStrandsInitialize / StiffRod):
  //   pair_handles[0] = below (prev / proximal)
  //   pair_handles[1] = above (next / distal)
  //   GpuSegmentPair.segment0 = below, segment1 = above
  for (size_t strand_id = 0; strand_id < physics_strand_to_segment_indices.size(); ++strand_id) {
    if (strand_id >= strands.size() || strand_id >= meshing_strand_to_segment_indices.size()) {
      continue;
    }
    auto& strand = strands[strand_id];
    strand.begin_segment_pair_handle = -1;
    strand.end_segment_pair_handle = -1;

    const auto& physics_segments = physics_strand_to_segment_indices[strand_id];
    const auto& meshing_segments = meshing_strand_to_segment_indices[strand_id];
    const size_t segment_count = std::min(physics_segments.size(), meshing_segments.size());
    if (segment_count < 2) {
      continue;
    }

    for (size_t segment_no = 0; segment_no + 1 < segment_count; ++segment_no) {
      const int below_physics_id = physics_segments[segment_no];
      const int above_physics_id = physics_segments[segment_no + 1];
      const size_t below_meshing_id = meshing_segments[segment_no];
      const size_t above_meshing_id = meshing_segments[segment_no + 1];
      if (below_physics_id < 0 || above_physics_id < 0 ||
          static_cast<size_t>(below_physics_id) >= segment_data_list.size() ||
          static_cast<size_t>(above_physics_id) >= segment_data_list.size()) {
        continue;
      }

      const auto below_neighbors = collect_physics_neighbors(below_meshing_id);
      const auto above_neighbors = collect_physics_neighbors(above_meshing_id);
      const bool adjacent =
          below_neighbors.count(above_physics_id) > 0 || above_neighbors.count(below_physics_id) > 0;
      if (!adjacent) {
        continue;
      }

      const int pair_handle = static_cast<int>(segment_pairs.size());
      DynamicStrands::GpuSegmentPair segment_pair{};
      segment_pair.segment0_handle = below_physics_id;
      segment_pair.segment1_handle = above_physics_id;
      segment_pairs.emplace_back(segment_pair);

      segment_data_list[below_physics_id].pair_handles[1] = pair_handle;
      segment_data_list[above_physics_id].pair_handles[0] = pair_handle;

      if (strand.begin_segment_pair_handle == -1) {
        strand.begin_segment_pair_handle = pair_handle;
      }
      strand.end_segment_pair_handle = pair_handle;
    }
  }

  dynamic_strands->connection_segment_pair_size = static_cast<uint32_t>(segment_pairs.size());

  // Mirror initialize: refresh strand pair-propagation bookkeeping from the new vertical range.
  for (auto& strand : strands) {
    strand.front_propagate_begin_segment_pair_handle = -1;
    strand.back_propagate_begin_segment_pair_handle = -1;
    strand.alternative_front_propagate_begin_segment_pair_handle = -1;
    strand.alternative_back_propagate_begin_segment_pair_handle = -1;
    if (strand.begin_segment_handle == -1 || strand.begin_segment_pair_handle == -1) {
      continue;
    }
    strand.front_propagate_begin_segment_pair_handle = strand.begin_segment_pair_handle;
    if (strand.begin_segment_pair_handle == strand.end_segment_pair_handle) {
      strand.alternative_front_propagate_begin_segment_pair_handle = strand.begin_segment_pair_handle;
      continue;
    }
    strand.alternative_front_propagate_begin_segment_pair_handle = strand.begin_segment_pair_handle + 1;

    const int connection_size = strand.end_segment_pair_handle - strand.begin_segment_pair_handle + 1;
    strand.back_propagate_begin_segment_pair_handle =
        connection_size % 2 == 0 ? strand.end_segment_pair_handle : strand.end_segment_pair_handle - 1;
    strand.alternative_back_propagate_begin_segment_pair_handle =
        connection_size % 2 == 0 ? strand.end_segment_pair_handle - 1 : strand.end_segment_pair_handle;
  }

  // Pass 2: remaining mesh neighbors as lateral pairs starting at slot 2.
  std::vector<uint32_t> pair_slot_offsets(segment_data_list.size(), 2);
  for (size_t strand_id = 0; strand_id < physics_strand_to_segment_indices.size(); ++strand_id) {
    if (strand_id >= meshing_strand_to_segment_indices.size()) {
      continue;
    }
    const auto& physics_segments = physics_strand_to_segment_indices[strand_id];
    const auto& meshing_segments = meshing_strand_to_segment_indices[strand_id];
    const size_t segment_count = std::min(physics_segments.size(), meshing_segments.size());
    for (size_t segment_no = 0; segment_no < segment_count; ++segment_no) {
      const int physics_segment_id = physics_segments[segment_no];
      if (physics_segment_id < 0 || static_cast<size_t>(physics_segment_id) >= segment_data_list.size()) {
        continue;
      }
      const int below_physics_id = segment_no > 0 ? physics_segments[segment_no - 1] : -1;
      const int above_physics_id =
          segment_no + 1 < segment_count ? physics_segments[segment_no + 1] : -1;

      for (const int physics_neighbor_id : collect_physics_neighbors(meshing_segments[segment_no])) {
        if (physics_neighbor_id == physics_segment_id || physics_neighbor_id == below_physics_id ||
            physics_neighbor_id == above_physics_id) {
          continue;
        }
        if (physics_neighbor_id < 0 || static_cast<size_t>(physics_neighbor_id) >= segment_data_list.size()) {
          continue;
        }
        // Create each undirected pair once.
        if (physics_neighbor_id <= physics_segment_id) {
          continue;
        }

        auto& first_slot = pair_slot_offsets[static_cast<size_t>(physics_segment_id)];
        auto& second_slot = pair_slot_offsets[static_cast<size_t>(physics_neighbor_id)];
        if (first_slot >= BUNDLE_MAX_CONNECTION || second_slot >= BUNDLE_MAX_CONNECTION) {
          continue;
        }

        const int pair_handle = static_cast<int>(segment_pairs.size());
        DynamicStrands::GpuSegmentPair segment_pair{};
        segment_pair.segment0_handle = physics_segment_id;
        segment_pair.segment1_handle = physics_neighbor_id;
        segment_pairs.emplace_back(segment_pair);

        segment_data_list[physics_segment_id].pair_handles[first_slot] = pair_handle;
        segment_data_list[physics_neighbor_id].pair_handles[second_slot] = pair_handle;
        ++first_slot;
        ++second_slot;
      }
    }
  }

  EVOENGINE_LOG("Recomputed segment pairs from mesh: " << dynamic_strands->connection_segment_pair_size
                                                       << " vertical, "
                                                       << (segment_pairs.size() - dynamic_strands->connection_segment_pair_size)
                                                       << " lateral (total " << segment_pairs.size() << ").");
}

void DsKineticVoronoiMeshing::RunMeshingAlgorithm(
    const std::vector<std::vector<glm::dvec2>>& support_points,
    std::vector<std::vector<double>>& subdivisions_by_strand,
    std::vector<std::vector<int>>& physics_strand_to_segment_indices,
    const std::vector<std::vector<glm::dmat4>>& transforms_by_height_and_branch, const GlobalTransform& root_transform,
    const std::vector<std::vector<size_t>>& branch_indices,
    std::vector<std::vector<std::vector<size_t>>>& strands_by_branch_id) {
  std::vector<float> bottom_boundary_distances_by_strand_id(physics_strand_to_segment_indices.size());
  std::vector<float> top_boundary_distances_by_strand_id(physics_strand_to_segment_indices.size());

  for (size_t strand_id = 0; strand_id < physics_strand_to_segment_indices.size(); strand_id++) {
    int bottom_segment_id = physics_strand_to_segment_indices[strand_id].front();
    int top_segment_id = physics_strand_to_segment_indices[strand_id].back();

    bottom_boundary_distances_by_strand_id[strand_id] = dynamic_strands->segments[bottom_segment_id].boundary_distance;
    top_boundary_distances_by_strand_id[strand_id] = dynamic_strands->segments[top_segment_id].boundary_distance;
  }

  EVOENGINE_LOG("Starting Kinetic Delaunay Voronoi Meshing...");

  for (size_t strand_id = 0; strand_id < subdivisions_by_strand.size(); ++strand_id) {
    size_t non_positive_count = 0;
    double example_t = 0.0;
    for (const double t : subdivisions_by_strand[strand_id]) {
      if (t <= 0.0) {
        if (non_positive_count == 0) {
          example_t = t;
        }
        ++non_positive_count;
      }
    }
    if (non_positive_count > 0) {
      EVOENGINE_WARNING("Subdivision parameter list for strand "
                        << strand_id << " contains " << non_positive_count
                        << " value(s) with t<=0 (e.g. t=" << example_t
                        << ") before meshing; these schedule a subdiv at bootstrap and yield zero-length meshlets.");
    }
  }

  strand_tree =
      std::make_shared<kinDS::StrandTree>(support_points, subdivisions_by_strand, physics_strand_to_segment_indices,
                                          transforms_by_height_and_branch, branch_indices, strands_by_branch_id);

  if (meshing_settings.dry_run_strand_tree_only) {
    EVOENGINE_LOG("Dry run: strand tree prepared, skipping meshing algorithm.");
    return;
  }

  tree_mesher_ = std::make_shared<kinDS::TreeMesher>(*strand_tree, [&](size_t count, std::function<void(size_t)> func) {
    Jobs::RunParallelFor(count, [&](size_t i) {
      func(i);
    });
  });
  tree_mesher_->getSettings().transform_mesh_at_construction = true;
  tree_mesher_->getSettings().mesh_cap_at_start = true;
  tree_mesher_->getSettings().collect_meshing_statistics = meshing_settings.collect_meshing_statistics;
  tree_mesher_->getSettings().store_mesh_metadata = meshing_settings.store_mesh_metadata;
  tree_mesher_->getSettings().export_separate_contributor_objects =
      meshing_settings.export_separate_contributor_objects;

  const std::string input_hash =
      ComputeMeshingInputHash(support_points, subdivisions_by_strand, physics_strand_to_segment_indices,
                              transforms_by_height_and_branch, root_transform, branch_indices, strands_by_branch_id);
  const std::filesystem::path buffer_dir = MeshingBufferDirectory();
  const std::filesystem::path bin_path = buffer_dir / (input_hash + ".bin");
  const std::filesystem::path yml_path = buffer_dir / (input_hash + ".yml");

  const auto warn_segment_count_mismatch =
      [&](const std::vector<std::vector<size_t>>& meshing_strand_to_segment_indices) {
        for (size_t strand_id = 0; strand_id < physics_strand_to_segment_indices.size(); ++strand_id) {
          if (strand_id >= meshing_strand_to_segment_indices.size()) {
            continue;
          }
          if (meshing_strand_to_segment_indices[strand_id].size() !=
              physics_strand_to_segment_indices[strand_id].size()) {
            EVOENGINE_WARNING("Meshing algorithm resulted in "
                              << meshing_strand_to_segment_indices[strand_id].size() << " segments for strand "
                              << strand_id << ", but the physics simulation has "
                              << physics_strand_to_segment_indices[strand_id].size() << ". There are "
                              << subdivisions_by_strand[strand_id].size() << " subdivision parameters in range ["
                              << subdivisions_by_strand[strand_id].front() << ", "
                              << subdivisions_by_strand[strand_id].back() << "].");
          }
        }
      };

  const auto debug_export_meshes = [&]() {
    if (!meshing_settings.debug_export_meshes) {
      return;
    }
    EVOENGINE_LOG("Exporting Kinetic Delaunay Voronoi Meshes for Debugging...");
    tree_mesher_->exportMeshlets(kinDS::MeshletExportMode::PerSegment, "meshlets");
    tree_mesher_->exportMeshlets(kinDS::MeshletExportMode::Combined, "combined_mesh.obj");
    EVOENGINE_LOG("Kinetic Delaunay Voronoi Meshes exported.");
  };

  bool loaded_from_cache = false;
  if (!meshing_settings.override_meshing_buffer && std::filesystem::exists(bin_path)) {
    std::vector<GpuMeshletVertex> gpu_vertices;
    std::vector<GpuMeshletTriangle> gpu_triangles;
    std::vector<kinDS::VoronoiMesh> meshlets;
    std::vector<std::vector<int>> neighbors;
    std::vector<size_t> meshing_to_physics;
    std::vector<std::vector<size_t>> strand_to_segment;
    if (LoadMeshingBuffer(bin_path, root_transform, gpu_vertices, gpu_triangles, meshlets, neighbors, meshing_to_physics,
                          strand_to_segment)) {
      tree_mesher_->getSegmentMeshlets() = std::move(meshlets);
      tree_mesher_->getMeshingNeighborIndices() = std::move(neighbors);
      tree_mesher_->setMeshingToPhysicsSegmentIndices(std::move(meshing_to_physics));
      tree_mesher_->setMeshingStrandToSegmentIndices(std::move(strand_to_segment));
      segment_meshlets_ = tree_mesher_->getSegmentMeshlets();
      meshing_neighbor_indices_ = tree_mesher_->getMeshingNeighborIndices();
      meshlets_root_transform_ = root_transform;
      segment_meshlet_vertices = std::move(gpu_vertices);
      segment_meshlet_triangles = std::move(gpu_triangles);
      warn_segment_count_mismatch(tree_mesher_->getMeshingStrandToSegmentIndices());
      EVOENGINE_LOG("Meshing buffer cache hit " << input_hash << " (" << segment_meshlet_vertices.size()
                                                << " vertices, " << segment_meshlet_triangles.size()
                                                << " triangles).");
      loaded_from_cache = true;
    } else {
      EVOENGINE_WARNING("Meshing buffer " << bin_path.string()
                                          << " exists but could not be loaded; remeshing (cache miss).");
    }
  } else if (meshing_settings.override_meshing_buffer) {
    EVOENGINE_LOG("Meshing buffer override enabled; remeshing for hash " << input_hash << ".");
  } else {
    EVOENGINE_LOG("Meshing buffer cache miss " << input_hash << " (no file at " << bin_path.string() << ").");
  }

  if (!loaded_from_cache) {
    auto& meshes = tree_mesher_->runMeshingAlgorithm(meshing_settings.debug_svg);

    // Keep pristine copies for later Intersect button runs (no clipping during meshing).
    segment_meshlets_ = meshes;
    meshing_neighbor_indices_ = tree_mesher_->getMeshingNeighborIndices();
    meshlets_root_transform_ = root_transform;
    warn_segment_count_mismatch(tree_mesher_->getMeshingStrandToSegmentIndices());
  }

  const auto& meshes = tree_mesher_->getSegmentMeshlets();
  const auto& meshing_neighbor_indices = tree_mesher_->getMeshingNeighborIndices();
  const auto& meshing_to_physics_segment_indices = tree_mesher_->getMeshingToPhysicsSegmentIndices();
  const auto& meshing_strand_to_segment_indices = tree_mesher_->getMeshingStrandToSegmentIndices();

  if (meshing_settings.recompute_segment_pairs) {
    RecomputeSegmentPairs(*tree_mesher_);
  }

  // Cache stores GPU buffers for the pairs that existed at save time. Rebuild them when we remeshed
  // or when pairs were recomputed after a cache hit.
  if (!loaded_from_cache || meshing_settings.recompute_segment_pairs) {
    segment_meshlet_vertices.clear();
    segment_meshlet_triangles.clear();
    PopulateGpuMeshletBuffers(meshes, physics_strand_to_segment_indices, meshing_strand_to_segment_indices,
                              meshing_neighbor_indices, meshing_to_physics_segment_indices, root_transform);
  }

  if (!loaded_from_cache) {
    auto& boundary_mesh = tree_mesher_->getBoundaryMesh();
    auto& boundary_vertex_to_strand_id = tree_mesher_->getBoundaryVertexToStrandId();

    boundary_distances_by_vertex.resize(boundary_mesh.getVertexCount(), 0.0f);
    for (size_t i = 0; i < boundary_mesh.getVertexCount(); i++) {
      size_t strand_id = boundary_vertex_to_strand_id[i];

      // as a heuristic, just use the bottom boundary distance if height is <= 0, otherwise use the top boundary
      // distance
      float height = boundary_mesh.getVertices()[i][2];
      if (height <= 0.0f) {
        boundary_distances_by_vertex[i] = bottom_boundary_distances_by_strand_id[strand_id];
      } else {
        boundary_distances_by_vertex[i] = top_boundary_distances_by_strand_id[strand_id];
      }
    }

    if (SaveMeshingBuffer(bin_path, yml_path, input_hash, root_transform, segment_meshlet_vertices,
                          segment_meshlet_triangles, segment_meshlets_, meshing_neighbor_indices_,
                          meshing_to_physics_segment_indices, meshing_strand_to_segment_indices)) {
      EVOENGINE_LOG("Saved Kinetic Voronoi mesh buffer " << input_hash << " to " << bin_path.string());
    }

    EVOENGINE_LOG("Kinetic Delaunay Voronoi Meshing completed.");
  }

  debug_export_meshes();
}

void DsKineticVoronoiMeshing::PopulateGpuMeshletBuffers(
    const std::vector<kinDS::VoronoiMesh>& meshes,
    const std::vector<std::vector<int>>& physics_strand_to_segment_indices,
    const std::vector<std::vector<size_t>>& meshing_strand_to_segment_indices,
    const std::vector<std::vector<int>>& meshing_neighbor_indices,
    const std::vector<size_t>& meshing_to_physics_segment_indices, const GlobalTransform& root_transform) {
  std::vector<DynamicStrands::GpuSegmentData>& segment_data_list = dynamic_strands->segment_data_list;
  std::vector<DynamicStrands::GpuSegmentPair>& segment_pairs = dynamic_strands->segment_pairs;

  for (size_t strand_id = 0; strand_id < physics_strand_to_segment_indices.size(); ++strand_id) {
    for (size_t segment_no = 0; segment_no < meshing_strand_to_segment_indices[strand_id].size(); ++segment_no) {
      const size_t meshing_segment_id = meshing_strand_to_segment_indices[strand_id][segment_no];
      if (meshing_segment_id >= meshes.size()) {
        EVOENGINE_ERROR("meshing_segment_id out of bounds: " << meshing_segment_id
                                                             << "; upper bound is: " << meshes.size())
        continue;
      }

      const auto& mesh = meshes[meshing_segment_id];
      const int physics_segment_id = physics_strand_to_segment_indices[strand_id][segment_no];

      const size_t vertex_offset = segment_meshlet_vertices.size();
      for (const auto& v : mesh.getVertices()) {
        GpuSegmentMeshletVertex vertex;
        vertex.x0 = root_transform.TransformPoint(glm::vec3(v[0], v[1], v[2]));
        vertex.x = vertex.x0;
        vertex.segment_index = physics_segment_id;
        segment_meshlet_vertices.push_back(vertex);
      }

      const auto& triangles = mesh.getTriangles();
      for (size_t triangle_vertex_index = 0; triangle_vertex_index < triangles.size(); triangle_vertex_index += 3) {
        GpuSegmentMeshletTriangle triangle;
        triangle.vertex_index0 = static_cast<unsigned int>(triangles[triangle_vertex_index] + vertex_offset);
        triangle.vertex_index1 = static_cast<unsigned int>(triangles[triangle_vertex_index + 1] + vertex_offset);
        triangle.vertex_index2 = static_cast<unsigned int>(triangles[triangle_vertex_index + 2] + vertex_offset);

        const int meshing_neighbor_segment_index = meshing_neighbor_indices[meshing_segment_id][triangle_vertex_index / 3];
        if (meshing_neighbor_segment_index >= static_cast<long>(meshing_to_physics_segment_indices.size())) {
          EVOENGINE_ERROR("meshing_neighbor_segment_index out of bounds: " << meshing_neighbor_segment_index
                                                                           << "; upper bound is: "
                                                                           << meshing_to_physics_segment_indices.size())
        } else if (meshing_neighbor_segment_index >= 0) {
          triangle.neighbor_segment_index = meshing_to_physics_segment_indices[meshing_neighbor_segment_index];
        } else {
          triangle.neighbor_segment_index = meshing_neighbor_segment_index;
        }

        for (size_t j = 0; j < 3; j++) {
          const auto normal = mesh.getNormal(triangle_vertex_index + j);
          triangle.normal0[j] = triangle.normal[j] =
              glm::vec4(root_transform.TransformVector(glm::vec3(normal[0], normal[1], normal[2])), 0.0f);

          if (mesh.hasValidUVIndex(triangle_vertex_index + j)) {
            triangle.uv[j] = glm::vec4(ToVec3(mesh.getUV(triangle_vertex_index + j)), 0.0);
          } else {
            triangle.uv[j] = glm::vec4(0.0f, 0.0f, 0.0f, 0.0f);
          }
        }

        triangle.segment_pair_index = -1;
        if (triangle.neighbor_segment_index >= 0) {
          for (int pair_handle : segment_data_list[physics_segment_id].pair_handles) {
            if (pair_handle == -1) {
              continue;
            }
            if (segment_pairs[pair_handle].segment0_handle != physics_segment_id &&
                segment_pairs[pair_handle].segment1_handle != physics_segment_id) {
              EVOENGINE_ERROR("Segment pair incorrectly referenced!");
            }
            if (segment_pairs[pair_handle].segment0_handle == triangle.neighbor_segment_index ||
                segment_pairs[pair_handle].segment1_handle == triangle.neighbor_segment_index) {
              triangle.segment_pair_index = static_cast<int>(pair_handle);
              break;
            }
          }
        }

        segment_meshlet_triangles.push_back(triangle);
      }
    }
  }
}

// DsKineticVoronoiMeshing implementation
DsKineticVoronoiMeshing::RenderSettings DsKineticVoronoiMeshing::render_settings = {};
DsKineticVoronoiMeshing::MeshingSettings DsKineticVoronoiMeshing::meshing_settings = {};

DsKineticVoronoiMeshing::DsKineticVoronoiMeshing() {
}

DsKineticVoronoiMeshing::~DsKineticVoronoiMeshing() {
  tree_mesher_.reset();
}

void eco_sys_lab_plugin::DsKineticVoronoiMeshing::InitBuffer(
    VkBufferCreateInfo& buffer_create_info, VmaAllocationCreateInfo& buffer_vma_allocation_create_info) {
  device_segment_meshlet_triangles_buffer =
      std::make_shared<Buffer>(buffer_create_info, buffer_vma_allocation_create_info);
  device_segment_meshlet_vertices_buffer =
      std::make_shared<Buffer>(buffer_create_info, buffer_vma_allocation_create_info);
}

void eco_sys_lab_plugin::DsKineticVoronoiMeshing::InitData(
    const DynamicStrandsInitializeParameters& initialize_parameters, const StrandModelSkeleton& strand_model_skeleton,
    const StrandModelStrandGroup& strand_model_strand_group, DtsStrandGroup& randomly_subdivided_strand_group,
    DtsStrandGroup& uniformly_subdivided_strand_group) {
  const auto& randomly_subdivided_strands = randomly_subdivided_strand_group.PeekStrands();
  const auto& randomly_subdivided_strand_segments = randomly_subdivided_strand_group.PeekStrandSegments();
  if (randomly_subdivided_strands.empty()) {
    EVOENGINE_LOG("Strand Group is empty!");
    return;
  }
  strand_model_strand_group.UniformlySubdivide<DtsStrandGroupData, DtsStrandData, DtsStrandSegmentData>(
      uniformly_subdivided_strand_group, initialize_parameters.uniform_subdivision,
      [&](const StrandHandle src_handle, DtsStrandData& strand_data) {
      },
      [&](const float start_root_distance, const float end_root_distance, const StrandSegmentHandle src_handle,
          const uint32_t original_segment_index, const float segment_t, DtsStrandSegmentData& segment_data,
          const uint32_t sub_segment_index) {
        const auto& src_segment_data = strand_model_strand_group.PeekStrandSegmentData(src_handle);
        segment_data.node_handle = src_segment_data.node_handle;
        segment_data.original_segment_handle = src_handle;
        segment_data.original_segment_index = original_segment_index;
        segment_data.segment_index = sub_segment_index;
        segment_data.original_segment_t = segment_t;
        segment_data.start_root_distance = start_root_distance;
        segment_data.end_root_distance = end_root_distance;
        const auto& strand_segment = strand_model_strand_group.PeekStrandSegment(src_handle);
        const auto& strand = strand_model_strand_group.PeekStrand(strand_segment.GetStrandHandle());
        const auto& strand_segment_handles = strand.PeekStrandSegmentHandles();

        glm::vec2 p0, p1, p3;
        const glm::vec2 p2 = src_segment_data.profile_position;
        float d0, d1, d3;
        const float d2 = src_segment_data.initial_distance_to_boundary;
        if (src_handle == strand_segment_handles.front()) {
          d1 = d2;
          d0 = d1 * 2.0f - d2;

          p1 = p2;
          p0 = p1 * 2.0f - p2;
        } else if (strand_segment.GetPrevHandle() == strand_segment_handles.front()) {
          const auto& prev_segment_data =
              strand_model_strand_group.PeekStrandSegmentData(strand_segment.GetPrevHandle());
          d0 = d2;
          d1 = prev_segment_data.initial_distance_to_boundary;

          p0 = p2;
          p1 = prev_segment_data.profile_position;
        } else {
          const auto& prev_segment = strand_model_strand_group.PeekStrandSegment(strand_segment.GetPrevHandle());
          const auto& prev_segment_data =
              strand_model_strand_group.PeekStrandSegmentData(strand_segment.GetPrevHandle());
          const auto& prev_prev_segment_data =
              strand_model_strand_group.PeekStrandSegmentData(prev_segment.GetPrevHandle());
          d0 = prev_prev_segment_data.initial_distance_to_boundary;
          d1 = prev_segment_data.initial_distance_to_boundary;

          p0 = prev_prev_segment_data.profile_position;
          p1 = prev_segment_data.profile_position;
        }
        if (src_handle == strand_segment_handles.back()) {
          d3 = d2 * 2.0f - d1;

          p3 = p2 * 2.0f - p1;
        } else {
          const auto& next_segment_data =
              strand_model_strand_group.PeekStrandSegmentData(strand_segment.GetNextHandle());
          d3 = next_segment_data.initial_distance_to_boundary;

          p3 = next_segment_data.profile_position;
        }
        segment_data.initial_distance_to_boundary = Strands::CubicInterpolation(d0, d1, d2, d3, segment_t);
        segment_data.profile_position = Strands::CubicInterpolation(p0, p1, p2, p3, segment_t);

        const auto calculate_polar_coordinates = [](const glm::vec2& profile_position) {
          const auto r = glm::length(profile_position);
          if (r <= glm::epsilon<float>()) {
            return glm::vec2(0.0f);
          }
          if (profile_position.y >= 0)
            return glm::vec2(r, glm::acos(profile_position.x / r));
          return glm::vec2(r, -glm::acos(profile_position.x / r));
        };

        segment_data.profile_polar_coordinate = calculate_polar_coordinates(segment_data.profile_position);
      },
      (initialize_parameters.min_segment_length + initialize_parameters.max_segment_length) * .5f * .01f);

  std::vector<std::vector<StrandCrossSectionGuidePoint>> strand_guide_points(randomly_subdivided_strands.size());
  std::vector<std::vector<int>> randomly_subdivided_segment_handles(randomly_subdivided_strands.size());
  std::vector<std::vector<double>> random_subdivisions_by_strand(randomly_subdivided_strands.size());

  int maxSegmentCount = std::numeric_limits<int>::min();
  std::mutex m;

  std::function<void(int&, int)> updateMax = [&](int& cur_max, int candidate) {
    std::lock_guard<std::mutex> lock(m);
    cur_max = std::max(cur_max, candidate);
  };

  Jobs::RunParallelFor(randomly_subdivided_strands.size(), [&](const size_t strand_index) {
    auto& random_subdivided_strand = randomly_subdivided_strands[strand_index];
    auto& uniformly_subdivided_strand = uniformly_subdivided_strand_group.PeekStrand(strand_index);

    size_t first_segment_handle = uniformly_subdivided_strand.PeekStrandSegmentHandles()[0];
    const auto& first_uniform_segment_data = uniformly_subdivided_strand_group.PeekStrandSegmentData(first_segment_handle);

    StrandCrossSectionGuidePoint first_guide_point;
    first_guide_point.profile_position =
        glm::dvec2(first_uniform_segment_data.profile_position.x, first_uniform_segment_data.profile_position.y);
    first_guide_point.node_handle = first_uniform_segment_data.node_handle;
    first_guide_point.segment_handle = first_segment_handle;
    first_guide_point.root_distance = first_uniform_segment_data.start_root_distance;
    strand_guide_points[strand_index].push_back(first_guide_point);

    updateMax(maxSegmentCount, uniformly_subdivided_strand.PeekStrandSegmentHandles().size());
    for (int uniform_segment_index = 0;
         uniform_segment_index < uniformly_subdivided_strand.PeekStrandSegmentHandles().size();
         uniform_segment_index++) {
      size_t segment_handle = uniformly_subdivided_strand.PeekStrandSegmentHandles()[uniform_segment_index];
      const auto& uniform_segment_data = uniformly_subdivided_strand_group.PeekStrandSegmentData(segment_handle);

      StrandCrossSectionGuidePoint guide_point;
      guide_point.profile_position =
          glm::dvec2(uniform_segment_data.profile_position.x, uniform_segment_data.profile_position.y);
      guide_point.node_handle = uniform_segment_data.node_handle;
      guide_point.segment_handle = segment_handle;
      guide_point.root_distance = uniform_segment_data.end_root_distance;
      strand_guide_points[strand_index].push_back(guide_point);
    }

    for (int random_segment_index = 0;
         random_segment_index < random_subdivided_strand.PeekStrandSegmentHandles().size(); random_segment_index++) {
      size_t segment_handle = random_subdivided_strand.PeekStrandSegmentHandles()[random_segment_index];
      const auto& segment = randomly_subdivided_strand_segments[segment_handle];
      const auto& random_segment_data = randomly_subdivided_strand_group.PeekStrandSegmentData(
          random_subdivided_strand.PeekStrandSegmentHandles()[random_segment_index]);

      randomly_subdivided_segment_handles[strand_index].push_back(static_cast<int>(segment_handle));

      if (!isnan(segment.end_t)) {
        random_subdivisions_by_strand[strand_index].push_back(
            initialize_parameters.uniform_subdivision *
            (segment.end_t + random_segment_data.original_segment_index));
      }
    }
  });

  // Create a branch index lookup using [strand_id][h]
  std::vector<std::vector<size_t>> branch_indices(strand_guide_points.size());
  // Maintain the branches as [h][branch_id][strand_no]
  std::vector<std::vector<std::vector<size_t>>> strands_by_branch_id(maxSegmentCount + 1);

  std::map<SkeletonNodeHandle, size_t> node_to_branch_map;
  for (size_t strand_id = 0; strand_id < strand_guide_points.size(); strand_id++) {
    auto& guide_points = strand_guide_points[strand_id];
    auto node_handle = guide_points.front().node_handle;
    auto it = node_to_branch_map.find(node_handle);
    if (it != node_to_branch_map.end()) {
      size_t branch_index = it->second;
      branch_indices[strand_id].push_back(branch_index);
      strands_by_branch_id[0][branch_index].push_back(strand_id);
    } else {
      size_t branch_index = strands_by_branch_id[0].size();
      node_to_branch_map[node_handle] = branch_index;
      strands_by_branch_id[0].push_back({strand_id});
      branch_indices[strand_id].push_back(branch_index);
    }
  }

  for (size_t h = 1; h < maxSegmentCount + 1; h++) {
    // Now iterate over each branch and check if we need to split it
    strands_by_branch_id[h].resize(strands_by_branch_id[h - 1].size());

    // EVOENGINE_LOG("------------------ Height: " << h)

    for (size_t branch_id = 0; branch_id < strands_by_branch_id[h - 1].size(); branch_id++) {
      auto& branch_strands = strands_by_branch_id[h - 1][branch_id];
      // branch might end early:
      if (branch_strands.empty()) {
        // EVOENGINE_LOG("Branch with id " << branch_id << " ended at height " << h);
        continue;
      }

      SkeletonNodeHandle branch_node = strand_guide_points[branch_strands.front()][h].node_handle;
      std::map<SkeletonNodeHandle, size_t> node_to_branch_map;

      node_to_branch_map[branch_node] = branch_id;

      for (size_t& strand_id : branch_strands) {
        const auto& guide_points = strand_guide_points[strand_id];

        if (h >= guide_points.size()) {
          // EVOENGINE_LOG("Strand " << strand_id << " ended early at height " << h);
          continue;  // strand ends here
        }

        auto node_handle = guide_points[h].node_handle;

        auto it = node_to_branch_map.find(node_handle);
        if (it != node_to_branch_map.end()) {
          size_t branch_index = it->second;
          // EVOENGINE_LOG("strand " << strand_id << " belongs to already found branch " << branch_index);
          branch_indices[strand_id].push_back(branch_index);
          strands_by_branch_id[h][branch_index].push_back(strand_id);
        } else {
          size_t branch_index = strands_by_branch_id[h].size();
          // EVOENGINE_LOG("strand " << strand_id << " belongs to newly discovered branch " << branch_index);
          node_to_branch_map[node_handle] = branch_index;
          strands_by_branch_id[h].push_back({strand_id});
          branch_indices[strand_id].push_back(branch_index);
        }
      }
    }
  }

  // For debugging, output the node index for each height:
  /*for (size_t h = 0; h < maxSegmentCount + 1; h++) {
    std::cout << "Node handles at height " << h << ": ";

    // collect in a set to not list duplicates
    std::set<SkeletonNodeHandle> handles;
    for (size_t strand_id = 0; strand_id < strand_guide_points.size(); strand_id++) {
      auto& guide_points = strand_guide_points[strand_id];
      if (h < guide_points.size()) {
        handles.insert(guide_points[h].node_handle);
      }
    }

    for (auto& handle : handles) {
      std::cout << handle << ", ";
    }
    std::cout << std::endl;
  }

  for (size_t h = 0; h < maxSegmentCount + 1; h++) {
    std::cout << "Branch indices at height " << h << ": ";
    std::set<size_t> index_set;
    for (size_t strand_id = 0; strand_id < strand_guide_points.size(); strand_id++) {
      if (h < branch_indices[strand_id].size()) {
        index_set.insert(branch_indices[strand_id][h]);
      }
    }

    for (auto& index : index_set) {
      std::cout << index << ", ";
    }
    std::cout << std::endl;
  }*/

  std::vector<std::vector<glm::dmat4>> transforms_by_height_and_branch(maxSegmentCount + 1);
  Jobs::RunParallelFor(maxSegmentCount + 1, [&](const size_t h) {
    transforms_by_height_and_branch[h].resize(strands_by_branch_id[h].size());
    for (size_t branch_index = 0; branch_index < transforms_by_height_and_branch[h].size(); branch_index++) {
      const auto& strand_ids = strands_by_branch_id[h][branch_index];
      if (strand_ids.empty()) {
        continue;
      }

      transforms_by_height_and_branch[h][branch_index] = BuildInterpolatedInternodeTransformAtHeight(
          strand_model_skeleton, strand_guide_points[strand_ids.front()], h);
    }
  });

  // Replace parametric 2D profile samples with 3D cubic-strand ∩ profile-plane samples.
  Jobs::RunParallelFor(strand_guide_points.size(), [&](const size_t strand_index) {
    auto& guide_points = strand_guide_points[strand_index];
    if (guide_points.empty() || strand_index >= branch_indices.size()) {
      return;
    }

    int hint_segment_index = 0;
    double preferred_t = 0.0;
    for (size_t h = 0; h < guide_points.size(); ++h) {
      if (h >= branch_indices[strand_index].size()) {
        break;
      }

      const size_t branch_index = branch_indices[strand_index][h];
      if (h >= transforms_by_height_and_branch.size() ||
          branch_index >= transforms_by_height_and_branch[h].size()) {
        break;
      }

      if (guide_points[h].segment_handle >= 0) {
        hint_segment_index =
            static_cast<int>(uniformly_subdivided_strand_group.PeekStrandSegmentData(guide_points[h].segment_handle)
                                 .original_segment_index);
        if (h == 0) {
          preferred_t = 0.0;
        } else {
          preferred_t = uniformly_subdivided_strand_group.PeekStrandSegmentData(guide_points[h].segment_handle)
                            .original_segment_t;
        }
      }

      const glm::dmat4& transform = transforms_by_height_and_branch[h][branch_index];
      const glm::dvec2 fallback = guide_points[h].profile_position;
      guide_points[h].profile_position = SampleStrandProfileAtPlane(
          strand_model_strand_group, static_cast<StrandHandle>(strand_index), transform, hint_segment_index, fallback,
          preferred_t, meshing_settings.spline_tension);
    }
  });

#ifndef NDEBUG
  {
    size_t residual_failures = 0;
    double max_residual = 0.0;
    for (size_t strand_index = 0; strand_index < strand_guide_points.size(); ++strand_index) {
      const auto& guide_points = strand_guide_points[strand_index];
      for (size_t h = 0; h < guide_points.size(); ++h) {
        if (h >= branch_indices[strand_index].size()) {
          break;
        }
        const size_t branch_index = branch_indices[strand_index][h];
        if (h >= transforms_by_height_and_branch.size() ||
            branch_index >= transforms_by_height_and_branch[h].size()) {
          break;
        }
        const glm::dmat4& transform = transforms_by_height_and_branch[h][branch_index];
        const ProfilePlane plane = ExtractProfilePlane(transform);
        const glm::dvec2& profile = guide_points[h].profile_position;
        const glm::dvec3 reconstructed = glm::dvec3(transform * glm::dvec4(profile.x, 0.0, profile.y, 1.0));
        const double residual = std::abs(PlaneResidual(reconstructed, plane));
        max_residual = std::max(max_residual, residual);
        if (residual > kPlaneSplineResidualEps) {
          ++residual_failures;
        }
      }
    }
    if (residual_failures > 0) {
      EVOENGINE_WARNING("Plane-spline profile sampling: " << residual_failures
                                                          << " samples exceed residual tolerance; max residual = "
                                                          << max_residual);
    }
  }
#endif

  std::vector<std::vector<glm::dvec2>> strand_splines;
  strand_splines.reserve(strand_guide_points.size());
  for (const auto& guide_points : strand_guide_points) {
    std::vector<glm::dvec2> support_points;
    support_points.reserve(guide_points.size());
    for (const auto& guide_point : guide_points) {
      support_points.emplace_back(guide_point.profile_position);
    }
    strand_splines.push_back(std::move(support_points));
  }

  kinDS::logger.setLogLevel(kinDS::LogLevel::Debug, false);
  RunMeshingAlgorithm(strand_splines, random_subdivisions_by_strand, randomly_subdivided_segment_handles,
                      transforms_by_height_and_branch, initialize_parameters.root_transform, branch_indices,
                      strands_by_branch_id);
}

void eco_sys_lab_plugin::DsKineticVoronoiMeshing::InitializationGraphicsPipeline(
    const DynamicStrandsInitializeParameters& initialize_parameters) {
  // Don't need this for now
}

struct VertexPredictionPushConstant {
  uint32_t vertex_count = 0;
  int padding0;
  int padding1;
  int padding2;
};

struct TrianglePredictionPushConstant {
  uint32_t triangle_count = 0;
  int padding0;
  int padding1;
  int padding2;
};

void eco_sys_lab_plugin::DsKineticVoronoiMeshing::BuildRenderComputePipelines() {
  static std::shared_ptr<Shader> shader{};
  shader = std::make_shared<Shader>();
  shader->TryCompile(ShaderType::Compute, Platform::GetShaderGlobalDefines(),
                     std::filesystem::path("./EcoSysLabResources") /
                         "Shaders/Compute/DynamicStrands/Prediction/KineticVoronoiMeshing/Vertex.comp");

  branches_vertex_update_pipeline = std::make_shared<ComputePipeline>();
  branches_vertex_update_pipeline->compute_shader = shader;
  branches_vertex_update_pipeline->descriptor_set_layouts.emplace_back(DynamicStrands::strands_layout);

  auto& push_constant_range = branches_vertex_update_pipeline->push_constant_ranges.emplace_back();
  push_constant_range.size = sizeof(VertexPredictionPushConstant);
  push_constant_range.offset = 0;
  push_constant_range.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;

  branches_vertex_update_pipeline->Initialize();

  // Triangles
  branches_triangle_update_pipeline = std::make_shared<ComputePipeline>();
  branches_triangle_update_pipeline->compute_shader =
      Shader::CreateTemporary(ShaderType::Compute, Platform::GetShaderGlobalDefines(),
                              std::filesystem::path("./EcoSysLabResources") /
                                  "Shaders/Compute/DynamicStrands/Prediction/KineticVoronoiMeshing/Triangle.comp");
  branches_triangle_update_pipeline->descriptor_set_layouts.emplace_back(DynamicStrands::strands_layout);

  auto& triangle_prediction_push_constant_range =
      branches_triangle_update_pipeline->push_constant_ranges.emplace_back();
  triangle_prediction_push_constant_range.size = sizeof(TrianglePredictionPushConstant);
  triangle_prediction_push_constant_range.offset = 0;
  triangle_prediction_push_constant_range.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;

  branches_triangle_update_pipeline->Initialize();
}

void eco_sys_lab_plugin::DsKineticVoronoiMeshing::RenderCompute() const {
  if (dynamic_strands->segments.empty())
    return;
  const uint32_t work_group_invocations = Platform::Constants::compute_work_group_invocations;
  const auto current_frame_index = Platform::GetCurrentFrameIndex();

  Platform::RecordCommandsMainQueue([&](const VkCommandBuffer vk_command_buffer) {
    // Vertices
    VertexPredictionPushConstant vertex_push_constant;
    vertex_push_constant.vertex_count = segment_meshlet_vertices.size();
    branches_vertex_update_pipeline->Bind(vk_command_buffer);
    branches_vertex_update_pipeline->BindDescriptorSet(
        vk_command_buffer, 0, dynamic_strands->strands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());
    branches_vertex_update_pipeline->PushConstant(vk_command_buffer, 0, vertex_push_constant);
    vkCmdDispatch(vk_command_buffer, Platform::DivUp(vertex_push_constant.vertex_count, work_group_invocations), 1, 1);
    Platform::EverythingBarrier(vk_command_buffer);

    // Triangles
    TrianglePredictionPushConstant triangle_push_constant;
    triangle_push_constant.triangle_count = segment_meshlet_triangles.size();
    branches_triangle_update_pipeline->Bind(vk_command_buffer);
    branches_triangle_update_pipeline->BindDescriptorSet(
        vk_command_buffer, 0, dynamic_strands->strands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());
    branches_triangle_update_pipeline->PushConstant(vk_command_buffer, 0, triangle_push_constant);
    vkCmdDispatch(vk_command_buffer, Platform::DivUp(triangle_push_constant.triangle_count, work_group_invocations), 1,
                  1);
    Platform::EverythingBarrier(vk_command_buffer);
  });
}

void eco_sys_lab_plugin::DsKineticVoronoiMeshing::BuildRenderingPipelines() {
  BuildSegmentMeshletsRenderingPipelines();
}

void eco_sys_lab_plugin::DsKineticVoronoiMeshing::Download() {
  if (!segment_meshlet_vertices.empty()) {
    device_segment_meshlet_vertices_buffer->DownloadVector(segment_meshlet_vertices, segment_meshlet_vertices.size());
  }
  if (!segment_meshlet_triangles.empty()) {
    device_segment_meshlet_triangles_buffer->DownloadVector(segment_meshlet_triangles,
                                                            segment_meshlet_triangles.size());
  }
}

void eco_sys_lab_plugin::DsKineticVoronoiMeshing::Upload() {
  device_segment_meshlet_vertices_buffer->UploadVector(segment_meshlet_vertices);
  device_segment_meshlet_vertices_buffer->SetDebugName("Segment Meshlet Vertices Buffer");
  device_segment_meshlet_triangles_buffer->UploadVector(segment_meshlet_triangles);
  device_segment_meshlet_triangles_buffer->SetDebugName("Segment Meshlet Triangles Buffer");
}

void eco_sys_lab_plugin::DsKineticVoronoiMeshing::Clear() {
  segment_meshlet_vertices.clear();
  segment_meshlet_triangles.clear();
  segment_meshlets_.clear();
  meshing_neighbor_indices_.clear();
  tree_mesher_.reset();
  meshlets_root_transform_ = {};
  deactivated_physics_segment_indices_.clear();
  deactivated_pair_integrities_.clear();
}

void eco_sys_lab_plugin::DsKineticVoronoiMeshing::UpdateBindings() const {
  const auto current_frame_index = Platform::GetCurrentFrameIndex();
  dynamic_strands->strands_descriptor_sets[current_frame_index]->UpdateBufferDescriptorBinding(
      8, device_segment_meshlet_vertices_buffer, 0);
  dynamic_strands->strands_descriptor_sets[current_frame_index]->UpdateBufferDescriptorBinding(
      9, device_segment_meshlet_triangles_buffer, 0);
}

bool eco_sys_lab_plugin::DsKineticVoronoiMeshing::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  ImGui::Checkbox("Dry run (strand tree only)", &meshing_settings.dry_run_strand_tree_only);
  if (ImGui::IsItemHovered()) {
    ImGui::SetTooltip("Prepare the strand tree during initialization but skip the meshing algorithm.");
  }
  ImGui::Checkbox("Override buffer", &meshing_settings.override_meshing_buffer);
  if (ImGui::IsItemHovered()) {
    ImGui::SetTooltip(
        "When enabled, skip loading a cached meshing buffer and overwrite it with newly computed mesh data.");
  }
  ImGui::Checkbox("Recompute segment pairs from mesh", &meshing_settings.recompute_segment_pairs);
  if (ImGui::IsItemHovered()) {
    ImGui::SetTooltip(
        "Rebuild the physics segment-pair graph from meshlet adjacency. pair_handles[0]/[1] are reserved for "
        "same-strand below/above neighbors (-1 if missing); other neighbors start at index 2.");
  }
  ImGui::Checkbox("Debug SVG", &meshing_settings.debug_svg);
  if (ImGui::IsItemHovered()) {
    ImGui::SetTooltip("Export kinDS segment-builder debug SVGs during meshing.");
  }
  ImGui::Checkbox("Collect meshing statistics", &meshing_settings.collect_meshing_statistics);
  if (ImGui::IsItemHovered()) {
    ImGui::SetTooltip(
        "Enable kinDS runtime/event statistics collection and CSV export after meshing "
        "(filename includes a timestamp so previous runs are kept). Also writes a per-mesh intersection "
        "CSV for Intersect and for Intersect and export all.");
  }
  ImGui::Checkbox("Debug export meshes", &meshing_settings.debug_export_meshes);
  if (ImGui::IsItemHovered()) {
    ImGui::SetTooltip("After meshing, export per-segment meshlets and a combined OBJ for debugging.");
  }
  ImGui::Checkbox("Separate interior/boundary OBJ objects", &meshing_settings.export_separate_contributor_objects);
  if (ImGui::IsItemHovered()) {
    ImGui::SetTooltip(
        "When enabled, debug and failed-meshlet OBJs contain one object (o) per interior/boundary contributor "
        "(iN / bN). Disable to write a single object per file.");
  }
  ImGui::Checkbox("Store mesh metadata", &meshing_settings.store_mesh_metadata);
  if (ImGui::IsItemHovered()) {
    ImGui::SetTooltip("Store JSON vertex/face metadata on meshlets (TreeMesher store_mesh_metadata).");
  }
  if (ImGui::DragFloat("Spline tension", &meshing_settings.spline_tension, 0.01f, 0.0f, 1.0f)) {
    meshing_settings.spline_tension = glm::clamp(meshing_settings.spline_tension, 0.0f, 1.0f);
  }
  if (ImGui::IsItemHovered()) {
    ImGui::SetTooltip(
        "Meshing-only blend for plane-spline sampling. 0 = Strands cubic (away from knots), 1 = Catmull-Rom (through "
        "knots).");
  }

  // --- Helpers used by Add / Save / Load / Export all ---
  // Returns the DynamicTreeStrands owner entity for this meshing instance.
  const auto find_owner_entity = [&]() -> Entity {
    const auto scene = Application::GetActiveScene();
    if (!scene || !dynamic_strands) {
      return Entity{};
    }
    const auto dts_owners = scene->UnsafeGetPrivateComponentOwnersList<DynamicTreeStrands>();
    if (!dts_owners) {
      return Entity{};
    }
    for (const auto& e : *dts_owners) {
      const auto dts = scene->GetOrSetPrivateComponent<DynamicTreeStrands>(e).lock();
      if (dts && dts->dynamic_strands && dts->dynamic_strands->meshing.get() == this) {
        return e;
      }
    }
    return Entity{};
  };

  // Finds the group entity (has DsIntersectionBoundaryMeshGroup) under owner, or returns invalid Entity.
  const auto find_group_entity = [](const std::shared_ptr<Scene>& scene, const Entity& owner) -> Entity {
    if (!scene || !scene->IsEntityValid(owner)) {
      return Entity{};
    }
    for (const auto& child : scene->GetChildren(owner)) {
      if (scene->HasPrivateComponent<DsIntersectionBoundaryMeshGroup>(child)) {
        return child;
      }
    }
    return Entity{};
  };

  // Finds or creates the group entity under owner.
  const auto ensure_group_entity = [&](const std::shared_ptr<Scene>& scene, const Entity& owner) -> Entity {
    Entity group = find_group_entity(scene, owner);
    if (scene->IsEntityValid(group)) {
      return group;
    }
    group = scene->CreateEntity("Intersection Meshes");
    scene->SetParent(group, owner);
    GlobalTransform group_gt{};
    group_gt.value = glm::mat4(1.0f);
    scene->SetDataComponent(group, group_gt);
    scene->GetOrSetPrivateComponent<DsIntersectionBoundaryMeshGroup>(group);
    return group;
  };

  // Open a file dialog; only create the child entity and load the mesh if the user picks a file.
  FileUtils::OpenFile(
      "Add Intersection Boundary Mesh", "OBJ", {".obj"},
      [&](const std::filesystem::path& path) {
        if (!dynamic_strands) {
          return;
        }
        const auto scene = Application::GetActiveScene();
        if (!scene) {
          return;
        }
        const Entity owner = find_owner_entity();
        if (!scene->IsEntityValid(owner)) {
          return;
        }
        try {
          kinDS::VoronoiMesh loaded_mesh = kinDS::ObjExporter::readMesh(path);
          const Entity group = ensure_group_entity(scene, owner);
          const auto child = scene->CreateEntity("Intersection Mesh (" + path.stem().string() + ")");
          scene->SetParent(child, group);
          GlobalTransform child_gt{};
          child_gt.value = glm::mat4(1.0f);
          scene->SetDataComponent(child, child_gt);
          const auto ibm = scene->GetOrSetPrivateComponent<DsIntersectionBoundaryMesh>(child).lock();
          if (ibm) {
            ibm->LoadMesh(std::move(loaded_mesh), path);
          }
          EVOENGINE_LOG("Added intersection boundary mesh from " << path.string() << ".");
        } catch (const std::exception& ex) {
          EVOENGINE_ERROR("Failed to load intersection boundary OBJ: " << ex.what());
        }
      },
      false);
  if (ImGui::IsItemHovered()) {
    ImGui::SetTooltip(
        "Open a file dialog to pick an OBJ, then create a child entity under the Intersection Meshes group. "
        "Move/rotate the child and click Intersect in its inspector.");
  }

  ImGui::SameLine();
  FileUtils::SaveFile(
      "Save intersection setup", "YAML", {".yml"},
      [&](const std::filesystem::path& save_path) {
        const auto scene = Application::GetActiveScene();
        const Entity owner = find_owner_entity();
        if (!scene || !scene->IsEntityValid(owner)) {
          EVOENGINE_ERROR("Save intersection setup: could not find owner entity.");
          return;
        }
        const Entity group = find_group_entity(scene, owner);
        YAML::Emitter out;
        out << YAML::BeginMap;
        if (scene->IsEntityValid(group)) {
          const auto group_gt = scene->GetDataComponent<GlobalTransform>(group);
          out << YAML::Key << "group_transform" << YAML::Value << group_gt.value;
          out << YAML::Key << "intersection_meshes" << YAML::Value << YAML::BeginSeq;
          for (const auto& child : scene->GetChildren(group)) {
            if (!scene->HasPrivateComponent<DsIntersectionBoundaryMesh>(child)) {
              continue;
            }
            const auto ibm = scene->GetOrSetPrivateComponent<DsIntersectionBoundaryMesh>(child).lock();
            if (!ibm || ibm->GetMesh().getTriangleCount() == 0) {
              continue;
            }
            const auto gt = scene->GetDataComponent<GlobalTransform>(child);
            out << YAML::BeginMap;
            out << YAML::Key << "obj_path" << YAML::Value << ibm->GetPath().string();
            out << YAML::Key << "transform" << YAML::Value << gt.value;
            out << YAML::EndMap;
          }
          out << YAML::EndSeq;
        } else {
          out << YAML::Key << "intersection_meshes" << YAML::Value << YAML::BeginSeq << YAML::EndSeq;
        }
        out << YAML::EndMap;
        std::ofstream ofs(save_path.string(), std::ofstream::out | std::ofstream::trunc);
        ofs << out.c_str();
        EVOENGINE_LOG("Saved intersection setup to " << save_path.string() << ".");
      },
      false);
  if (ImGui::IsItemHovered()) {
    ImGui::SetTooltip("Save the group transform and all boundary meshes to a YAML file.");
  }

  ImGui::SameLine();
  FileUtils::OpenFile(
      "Load intersection setup", "YAML", {".yml"},
      [&](const std::filesystem::path& load_path) {
        const auto scene = Application::GetActiveScene();
        const Entity owner = find_owner_entity();
        if (!scene || !scene->IsEntityValid(owner)) {
          EVOENGINE_ERROR("Load intersection setup: could not find owner entity.");
          return;
        }
        LoadIntersectionSetup(scene, owner, load_path);
      },
      false);
  if (ImGui::IsItemHovered()) {
    ImGui::SetTooltip("Load an intersection setup YAML, recreating all boundary mesh entities under the group.");
  }

  const bool can_reset_meshlets = HasMeshedSegmentMeshlets();
  if (!can_reset_meshlets) {
    ImGui::BeginDisabled();
  }
  if (ImGui::Button("Reset meshlets")) {
    ResetMeshletsToGpu();
  }
  if (!can_reset_meshlets) {
    ImGui::EndDisabled();
  }
  if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled)) {
    ImGui::SetTooltip(
        "Reload pristine meshlets (from the last meshing run) into GPU buffers without intersection. "
        "Requires a completed meshing run.");
  }

  const bool can_intersect_all = HasMeshedSegmentMeshlets();
  if (!can_intersect_all) {
    ImGui::BeginDisabled();
  }
  FileUtils::SaveFile(
      "Intersect and export all", "OBJ", {".obj"},
      [&](const std::filesystem::path& out_path) {
        const auto scene = Application::GetActiveScene();
        const Entity owner = find_owner_entity();
        if (!scene || !scene->IsEntityValid(owner)) {
          EVOENGINE_ERROR("Intersect and export all: could not find owner entity.");
          return;
        }
        const auto tree_gt = scene->GetDataComponent<GlobalTransform>(owner);
        const Entity group = find_group_entity(scene, owner);
        if (!scene->IsEntityValid(group)) {
          EVOENGINE_ERROR("Intersect and export all: no intersection mesh group found.");
          return;
        }
        std::vector<MeshletObjExport::MeshGroup> export_groups;
        std::vector<std::pair<std::string, IntersectionRunStats>> intersection_stats_rows;
        const bool collect_intersection_stats = meshing_settings.collect_meshing_statistics;
        for (const auto& child : scene->GetChildren(group)) {
          if (!scene->HasPrivateComponent<DsIntersectionBoundaryMesh>(child)) {
            continue;
          }
          const auto ibm = scene->GetOrSetPrivateComponent<DsIntersectionBoundaryMesh>(child).lock();
          if (!ibm || ibm->GetMesh().getTriangleCount() == 0) {
            continue;
          }
          const auto boundary_gt = scene->GetDataComponent<GlobalTransform>(child);
          IntersectionRunStats intersection_stats;
          if (!IntersectMeshletsWithBoundary(ibm->GetMesh(), boundary_gt, tree_gt,
                                             collect_intersection_stats ? &intersection_stats : nullptr)) {
            EVOENGINE_ERROR("Intersect and export all: intersection failed for entity "
                            << child.GetIndex() << ".");
            continue;
          }
          MeshletObjExport::MeshGroup mesh_group;
          mesh_group.name = ibm->GetPath().stem().string();
          if (mesh_group.name.empty()) {
            mesh_group.name = "entity_" + std::to_string(child.GetIndex());
          }
          mesh_group.vertices = segment_meshlet_vertices;
          mesh_group.triangles = segment_meshlet_triangles;
          if (collect_intersection_stats) {
            intersection_stats_rows.emplace_back(mesh_group.name, intersection_stats);
          }
          export_groups.push_back(std::move(mesh_group));
        }
        ResetMeshletsToGpu();
        if (export_groups.empty()) {
          EVOENGINE_ERROR("Intersect and export all: no intersection meshes exported.");
          return;
        }
        MeshletObjExport::ExportObjCombined(
            out_path, export_groups, dynamic_strands->segments,
            render_settings.segment_meshlet_render_parameters.uv_height_factor,
            render_settings.segment_meshlet_render_parameters.uv_circum_factor,
            render_settings.segment_meshlet_render_parameters.fracture_distance);
        EVOENGINE_LOG("Intersect and export all: exported " << export_groups.size() << " object(s) to "
                                                            << out_path.string() << ".");
        if (collect_intersection_stats && !intersection_stats_rows.empty()) {
          const std::filesystem::path stats_base =
              out_path.parent_path() / (out_path.stem().string() + "_intersection_stats.csv");
          WriteIntersectionStatisticsCsv(stats_base, intersection_stats_rows);
        }
      },
      false);
  if (!can_intersect_all) {
    ImGui::EndDisabled();
  }
  if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled)) {
    ImGui::SetTooltip(
        "For each loaded intersection mesh, compute the intersection and export all results as a single OBJ "
        "(one object per boundary mesh) plus shared bark/interior materials and GPU metadata JSON. "
        "When Collect meshing statistics is enabled, also writes a timestamped CSV with per-mesh "
        "inside/intersect/outside counts, input poly count, and clip runtime. "
        "Restores pristine meshlets afterward. Requires a completed meshing run.");
  }

  ImGui::Checkbox("Fix missing meshlets after intersection", &meshing_settings.intersection_boundary_fix_missing_meshes);
  if (ImGui::IsItemHovered()) {
    ImGui::SetTooltip("Attempt to repair empty meshlets after boundary intersection using neighbor triangles.");
  }
  ImGui::Checkbox("Keep original meshlet on intersection failure",
                  &meshing_settings.intersection_keep_original_on_failure);
  if (ImGui::IsItemHovered()) {
    ImGui::SetTooltip(
        "If intersection fails (e.g. non-manifold), keep the uncut meshlet. Disable to replace it with an empty mesh.");
  }
  ImGui::Checkbox("Prefer meshlet UVs on intersection seam",
                  &meshing_settings.intersection_prefer_meshlet_uv_on_seam);
  if (ImGui::IsItemHovered()) {
    ImGui::SetTooltip(
        "Vertices lying on the original meshlet surface receive segment-meshlet UVs at the clip seam, even on "
        "boundary-origin faces.");
  }
  ImGui::Checkbox("Interior UVs on clip-boundary faces",
                  &meshing_settings.intersection_boundary_faces_interior_uv);
  if (ImGui::IsItemHovered()) {
    ImGui::SetTooltip(
        "Faces originating from the clip boundary use interior-style (a,b,h) UVs. Bark polar distance is treated as "
        "r=1 when converting.");
  }

  FileUtils::SaveFile(
      "Download and export PLY", "PLY", {".ply"},
      [&](const std::filesystem::path& path) {
        dynamic_strands->Download();
        EVOENGINE_LOG("Downloaded data from GPU");
        PlyExporter::ExportAscii(path, segment_meshlet_vertices, segment_meshlet_triangles,
                                 render_settings.segment_meshlet_render_parameters.uv_height_factor,
                                 render_settings.segment_meshlet_render_parameters.uv_circum_factor);
      },
      false);
  ImGui::SameLine();
  FileUtils::SaveFile(
      "Export PLY", "PLY", {".ply"},
      [&](const std::filesystem::path& path) {
        PlyExporter::ExportAscii(path, segment_meshlet_vertices, segment_meshlet_triangles,
                                 render_settings.segment_meshlet_render_parameters.uv_height_factor,
                                 render_settings.segment_meshlet_render_parameters.uv_circum_factor);
      },
      false);

  FileUtils::SaveFile(
      "Download and export OBJ", "OBJ", {".obj"},
      [&](const std::filesystem::path& path) {
        dynamic_strands->Download();
        EVOENGINE_LOG("Downloaded data from GPU");
        MeshletObjExport::ExportObj(path, segment_meshlet_vertices, segment_meshlet_triangles,
                                    dynamic_strands->segments,
                                    render_settings.segment_meshlet_render_parameters.uv_height_factor,
                                    render_settings.segment_meshlet_render_parameters.uv_circum_factor,
                                    render_settings.segment_meshlet_render_parameters.fracture_distance);
      },
      false);
  ImGui::SameLine();
  FileUtils::SaveFile(
      "Export OBJ", "OBJ", {".obj"},
      [&](const std::filesystem::path& path) {
        MeshletObjExport::ExportObj(path, segment_meshlet_vertices, segment_meshlet_triangles,
                                    dynamic_strands->segments,
                                    render_settings.segment_meshlet_render_parameters.uv_height_factor,
                                    render_settings.segment_meshlet_render_parameters.uv_circum_factor,
                                    render_settings.segment_meshlet_render_parameters.fracture_distance);
      },
      false);
  // FileUtils::SaveFile(
  //     "Export Boundary OBJ", "OBJ", {".obj"},
  //     [&](const std::filesystem::path& path) {
  //       kinDS::ObjExporter::writeMesh(
  //           transformed_boundary_mesh, path, render_settings.segment_meshlet_render_parameters.uv_height_factor,
  //           render_settings.segment_meshlet_render_parameters.uv_circum_factor, boundary_distances_by_vertex);
  //     },
  //     false);

  if (strand_tree) {
    FileUtils::SaveFile(
        "Export Strand Tree", "TXT", {".txt"},
        [&](const std::filesystem::path& path) {
          strand_tree->saveToFile(path);
        },
        false);
  }
  return false;
}

void DsKineticVoronoiMeshing::Stats(const std::shared_ptr<EditorLayer>& editor_layer) {
  ImGui::Text((std::string("Segment Meshlets Vertices: ") + std::to_string(segment_meshlet_vertices.size())).c_str());
  ImGui::Text((std::string("Segment Meshlets Triangles: ") + std::to_string(segment_meshlet_triangles.size())).c_str());
}

void DsKineticVoronoiMeshing::OnInspectRenderSettings(const std::shared_ptr<EditorLayer>& editor_layer) {
  ImGui::Checkbox("Render Segment Meshlets", &render_settings.segment_meshlet_render_parameters.enabled);
  if (render_settings.segment_meshlet_render_parameters.enabled) {
    if (ImGui::Button("Rebuild segment meshlet pipelines")) {
      BuildSegmentMeshletsRenderingPipelines();
    }

    ImGui::Combo("Color mode", {"Standard", "Normals", "UVs", "Pair"},
                 render_settings.segment_meshlet_render_parameters.color_mode);

    // uv factors
    ImGui::DragFloat("UV height factor", &render_settings.segment_meshlet_render_parameters.uv_height_factor, 0.001f,
                     0.001f, 1.0f);
    ImGui::DragFloat("UV circum factor", &render_settings.segment_meshlet_render_parameters.uv_circum_factor, 1.0f,
                     1.0f, 50.0f, "%.0f");

    ImGui::DragFloat("Fracture distance", &render_settings.segment_meshlet_render_parameters.fracture_distance, 0.0001f,
                     0.0f, 2.0f, "%.4f");
  }
}

void eco_sys_lab_plugin::DsKineticVoronoiMeshing::RegisterRenderInstances(Handle& rendering_instance_handle,
                                                                          std::shared_ptr<Scene> scene, Entity& owner) {
  RegisterSegmentMeshletsRenderInstance(rendering_instance_handle, scene, owner);
}

void DsKineticVoronoiMeshing::RegisterSegmentMeshletsRenderInstance(Handle& rendering_instance_handle,
                                                                    std::shared_ptr<Scene> scene, Entity& owner) {
  const auto render_layer = Application::GetLayer<RenderLayer>();
  if (!render_layer) {
    EVOENGINE_LOG("Failed to render! RenderLayer not present!")
    return;
  }
  const auto inner_wood_material = dynamic_strands->materials.inner_wood_material_ref.Get<Material>();
  const auto snow_material = dynamic_strands->materials.snow_material_ref.Get<Material>();
  if (const auto bark_material = dynamic_strands->materials.bark_material_ref.Get<Material>();
      bark_material && inner_wood_material && snow_material) {
    if (!dynamic_strands->segments.empty()) {
      if (segment_meshlet_point_light_render_pipeline && segment_meshlet_point_light_render_pipeline->Initialized()) {
        render_layer->RenderToPointLightShadowMap([=](VkCommandBuffer vk_command_buffer, const auto& view) {
          return RenderSegmentMeshletsToPointLightShadowMap(render_settings.segment_meshlet_render_parameters,
                                                            vk_command_buffer, view);
        });
      }
      if (segment_meshlet_spot_light_render_pipeline && segment_meshlet_spot_light_render_pipeline->Initialized()) {
        render_layer->RenderToSpotLightShadowMap([=](VkCommandBuffer vk_command_buffer, const auto& view) {
          return RenderSegmentMeshletsToSpotLightShadowMap(render_settings.segment_meshlet_render_parameters,
                                                           vk_command_buffer, view);
        });
      }
      if (segment_meshlet_directional_light_render_pipeline &&
          segment_meshlet_directional_light_render_pipeline->Initialized()) {
        render_layer->RenderToDirectionalLightShadowMap([=](VkCommandBuffer vk_command_buffer, const auto& view) {
          return RenderSegmentMeshletsToDirectionalLightShadowMap(render_settings.segment_meshlet_render_parameters,
                                                                  vk_command_buffer, view);
        });
      }
      if (segment_meshlet_render_pipeline && segment_meshlet_render_pipeline->Initialized()) {
        const auto current_render_storage = Application::GetLayer<RenderLayer>()->GetCurrentRenderInstanceStorage();
        const auto renderer_handle = rendering_instance_handle;
        int bark_material_index = -1;
        current_render_storage->RegisterRenderInstance(scene, owner, renderer_handle, bark_material,
                                                       &bark_material_index);
        const auto inner_material_index = current_render_storage->RegisterMaterial(inner_wood_material);
        const auto snow_material_index = current_render_storage->RegisterMaterial(snow_material);
        render_layer->DeferredRenderingAllCameras(
            [=](const VkCommandBuffer vk_command_buffer,
                const std::vector<VkRenderingAttachmentInfo>& geometry_pass_color_attachment_infos,
                const RenderLayer::DeferredRenderingView& view) {
              return RenderSegmentMeshletsToCameraDeferred(
                  renderer_handle, bark_material_index, inner_material_index, snow_material_index,
                  render_settings.segment_meshlet_render_parameters, vk_command_buffer,
                  geometry_pass_color_attachment_infos, view, VK_POLYGON_MODE_FILL);
            });
      }
    }
  }
}

void eco_sys_lab_plugin::DsKineticVoronoiMeshing::Visualize(
    const std::shared_ptr<Camera>& target_camera, const DynamicStrandsInitializeParameters& initialize_parameters,
    const DynamicStrandsVisualizationParameters& visualization_parameters) {
  // TODO
}

void DsKineticVoronoiMeshing::BuildSegmentMeshletsRenderingPipelines() {
  segment_meshlet_point_light_render_pipeline = std::make_shared<GraphicsPipeline>();
  segment_meshlet_point_light_render_pipeline->task_shader = Shader::CreateTemporary(
      ShaderType::Task, Platform::GetShaderGlobalDefines(),
      std::filesystem::path("./EcoSysLabResources") /
          "Shaders/Graphics/Task/DynamicStrands/Rendering/KineticVoronoiMeshing/SegmentMeshlet.task");
  segment_meshlet_point_light_render_pipeline->mesh_shader =
      Shader::CreateTemporary(ShaderType::Mesh, Platform::GetShaderGlobalDefines(),
                              std::filesystem::path("./EcoSysLabResources") /
                                  "Shaders/Graphics/Mesh/DynamicStrands/Rendering/KineticVoronoiMeshing/SegmentMeshlet/"
                                  "PointLightShadowMap.mesh");
  segment_meshlet_point_light_render_pipeline->fragment_shader =
      Shader::CreateTemporary(ShaderType::Fragment, Platform::GetShaderGlobalDefines(),
                              std::filesystem::path("./EcoSysLabResources") / "Shaders/Graphics/Fragment/Empty.frag");
  segment_meshlet_point_light_render_pipeline->geometry_type = GeometryType::Mesh;
  segment_meshlet_point_light_render_pipeline->descriptor_set_layouts.emplace_back(RenderLayer::per_frame_layout);
  segment_meshlet_point_light_render_pipeline->descriptor_set_layouts.emplace_back(DynamicStrands::strands_layout);
  segment_meshlet_point_light_render_pipeline->depth_attachment_format = Platform::Constants::shadow_map;
  segment_meshlet_point_light_render_pipeline->stencil_attachment_format = VK_FORMAT_UNDEFINED;
  auto& point_light_push_constant_range =
      segment_meshlet_point_light_render_pipeline->push_constant_ranges.emplace_back();
  point_light_push_constant_range.size = sizeof(SegmentMeshletPushConstant);
  point_light_push_constant_range.offset = 0;
  point_light_push_constant_range.stageFlags = VK_SHADER_STAGE_ALL;
  segment_meshlet_point_light_render_pipeline->Initialize();
  // Descriptor set layout
  segment_meshlet_spot_light_render_pipeline = std::make_shared<GraphicsPipeline>();
  segment_meshlet_spot_light_render_pipeline->task_shader = Shader::CreateTemporary(
      ShaderType::Task, Platform::GetShaderGlobalDefines(),
      std::filesystem::path("./EcoSysLabResources") /
          "Shaders/Graphics/Task/DynamicStrands/Rendering/KineticVoronoiMeshing/SegmentMeshlet.task");

  segment_meshlet_spot_light_render_pipeline->mesh_shader =
      Shader::CreateTemporary(ShaderType::Mesh, Platform::GetShaderGlobalDefines(),
                              std::filesystem::path("./EcoSysLabResources") /
                                  "Shaders/Graphics/Mesh/DynamicStrands/Rendering/KineticVoronoiMeshing/SegmentMeshlet/"
                                  "SpotLightShadowMap.mesh");
  segment_meshlet_spot_light_render_pipeline->fragment_shader =
      Shader::CreateTemporary(ShaderType::Fragment, Platform::GetShaderGlobalDefines(),
                              std::filesystem::path("./EcoSysLabResources") / "Shaders/Graphics/Fragment/Empty.frag");
  segment_meshlet_spot_light_render_pipeline->geometry_type = GeometryType::Mesh;
  segment_meshlet_spot_light_render_pipeline->descriptor_set_layouts.emplace_back(RenderLayer::per_frame_layout);
  segment_meshlet_spot_light_render_pipeline->descriptor_set_layouts.emplace_back(DynamicStrands::strands_layout);
  segment_meshlet_spot_light_render_pipeline->depth_attachment_format = Platform::Constants::shadow_map;
  segment_meshlet_spot_light_render_pipeline->stencil_attachment_format = VK_FORMAT_UNDEFINED;
  auto& spot_light_push_constant_range =
      segment_meshlet_spot_light_render_pipeline->push_constant_ranges.emplace_back();
  spot_light_push_constant_range.size = sizeof(SegmentMeshletPushConstant);
  spot_light_push_constant_range.offset = 0;
  spot_light_push_constant_range.stageFlags = VK_SHADER_STAGE_ALL;
  segment_meshlet_spot_light_render_pipeline->Initialize();
  // Descriptor set layout
  segment_meshlet_directional_light_render_pipeline = std::make_shared<GraphicsPipeline>();
  segment_meshlet_directional_light_render_pipeline->task_shader = Shader::CreateTemporary(
      ShaderType::Task, Platform::GetShaderGlobalDefines(),
      std::filesystem::path("./EcoSysLabResources") /
          "Shaders/Graphics/Task/DynamicStrands/Rendering/KineticVoronoiMeshing/SegmentMeshlet.task");

  // TODO: fix path
  segment_meshlet_directional_light_render_pipeline->mesh_shader =
      Shader::CreateTemporary(ShaderType::Mesh, Platform::GetShaderGlobalDefines(),
                              std::filesystem::path("./EcoSysLabResources") /
                                  "Shaders/Graphics/Mesh/DynamicStrands/Rendering/KineticVoronoiMeshing/SegmentMeshlet/"
                                  "DirectionalLightShadowMap.mesh");
  segment_meshlet_directional_light_render_pipeline->fragment_shader =
      Shader::CreateTemporary(ShaderType::Fragment, Platform::GetShaderGlobalDefines(),
                              std::filesystem::path("./EcoSysLabResources") / "Shaders/Graphics/Fragment/Empty.frag");
  segment_meshlet_directional_light_render_pipeline->geometry_type = GeometryType::Mesh;
  segment_meshlet_directional_light_render_pipeline->descriptor_set_layouts.emplace_back(RenderLayer::per_frame_layout);
  segment_meshlet_directional_light_render_pipeline->descriptor_set_layouts.emplace_back(
      DynamicStrands::strands_layout);
  segment_meshlet_directional_light_render_pipeline->depth_attachment_format = Platform::Constants::shadow_map;
  segment_meshlet_directional_light_render_pipeline->stencil_attachment_format = VK_FORMAT_UNDEFINED;
  auto& directional_light_push_constant_range =
      segment_meshlet_directional_light_render_pipeline->push_constant_ranges.emplace_back();
  directional_light_push_constant_range.size = sizeof(SegmentMeshletPushConstant);
  directional_light_push_constant_range.offset = 0;
  directional_light_push_constant_range.stageFlags = VK_SHADER_STAGE_ALL;
  segment_meshlet_directional_light_render_pipeline->Initialize();
  // Descriptor set layout
  segment_meshlet_render_pipeline = std::make_shared<GraphicsPipeline>();
  segment_meshlet_render_pipeline->task_shader = Shader::CreateTemporary(
      ShaderType::Task, Platform::GetShaderGlobalDefines(),
      std::filesystem::path("./EcoSysLabResources") /
          "Shaders/Graphics/Task/DynamicStrands/Rendering/KineticVoronoiMeshing/SegmentMeshlet.task");
  segment_meshlet_render_pipeline->mesh_shader = Shader::CreateTemporary(
      ShaderType::Mesh, Platform::GetShaderGlobalDefines(),
      std::filesystem::path("./EcoSysLabResources") /
          "Shaders/Graphics/Mesh/DynamicStrands/Rendering/KineticVoronoiMeshing/SegmentMeshlet/Rendering.mesh");
  segment_meshlet_render_pipeline->fragment_shader = Shader::CreateTemporary(
      ShaderType::Fragment, Platform::GetShaderGlobalDefines(),
      std::filesystem::path("./EcoSysLabResources") /
          "Shaders/Graphics/Fragment/DynamicStrands/Rendering/KineticVoronoiMeshing/Branches.frag");
  segment_meshlet_render_pipeline->geometry_type = GeometryType::Mesh;
  segment_meshlet_render_pipeline->descriptor_set_layouts.emplace_back(RenderLayer::per_frame_layout);
  segment_meshlet_render_pipeline->descriptor_set_layouts.emplace_back(DynamicStrands::strands_layout);
  segment_meshlet_render_pipeline->descriptor_set_layouts.emplace_back(RenderLayer::lighting_layout);
  segment_meshlet_render_pipeline->depth_attachment_format = Platform::Constants::render_texture_depth;
  segment_meshlet_render_pipeline->stencil_attachment_format = VK_FORMAT_UNDEFINED;
  segment_meshlet_render_pipeline->color_attachment_formats = {2, Platform::Constants::g_buffer_color};
  auto& push_constant_range = segment_meshlet_render_pipeline->push_constant_ranges.emplace_back();
  push_constant_range.size = sizeof(SegmentMeshletPushConstant);
  push_constant_range.offset = 0;
  push_constant_range.stageFlags = VK_SHADER_STAGE_ALL;
  segment_meshlet_render_pipeline->Initialize();
}

uint32_t DsKineticVoronoiMeshing::RenderSegmentMeshletsToPointLightShadowMap(
    const SegmentMeshletsRenderParameters& render_parameters, const VkCommandBuffer vk_command_buffer,
    const RenderLayer::PointLightShadowMapView& view) const {
  if (!render_parameters.enabled) {
    return 0;
  }
  const uint32_t task_work_group_invocations =
      Platform::GetSelectedPhysicalDevice()->mesh_shader_properties_ext.maxPreferredTaskWorkGroupInvocations;
  SegmentMeshletPushConstant push_constant;
  push_constant.index1.sub_light_index = view.face_index;
  push_constant.index2.light_index = view.light_index;
  push_constant.vertex_count = segment_meshlet_vertices.size();
  push_constant.triangle_count = segment_meshlet_triangles.size();
  push_constant.color_mode = render_settings.segment_meshlet_render_parameters.color_mode;
  const auto current_frame_index = Platform::GetCurrentFrameIndex();
  segment_meshlet_point_light_render_pipeline->Bind(vk_command_buffer);
  segment_meshlet_point_light_render_pipeline->BindDescriptorSet(
      vk_command_buffer, 0, RenderLayer::GetPerFrameDescriptorSet()->GetVkDescriptorSet());
  segment_meshlet_point_light_render_pipeline->BindDescriptorSet(
      vk_command_buffer, 1, dynamic_strands->strands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());
  segment_meshlet_point_light_render_pipeline->states.ResetAllStates(0);
  segment_meshlet_point_light_render_pipeline->states.SetViewportScissor(view.viewport);
  segment_meshlet_point_light_render_pipeline->states.ApplyAllStates(vk_command_buffer);

  segment_meshlet_point_light_render_pipeline->PushConstant(vk_command_buffer, 0, push_constant);
  const uint32_t count = Platform::DivUp(segment_meshlet_triangles.size(), task_work_group_invocations);
  vkCmdDrawMeshTasksEXT(vk_command_buffer, count, 1, 1);
  return segment_meshlet_triangles.size();
}

uint32_t DsKineticVoronoiMeshing::RenderSegmentMeshletsToSpotLightShadowMap(
    const SegmentMeshletsRenderParameters& render_parameters, VkCommandBuffer vk_command_buffer,
    const RenderLayer::SpotLightShadowMapView& view) const {
  if (!render_parameters.enabled) {
    return 0;
  }
  const uint32_t task_work_group_invocations =
      Platform::GetSelectedPhysicalDevice()->mesh_shader_properties_ext.maxPreferredTaskWorkGroupInvocations;
  SegmentMeshletPushConstant push_constant;
  push_constant.index1.sub_light_index = 0;
  push_constant.index2.light_index = view.light_index;
  push_constant.vertex_count = segment_meshlet_vertices.size();
  push_constant.triangle_count = segment_meshlet_triangles.size();
  push_constant.color_mode = render_settings.segment_meshlet_render_parameters.color_mode;
  const auto current_frame_index = Platform::GetCurrentFrameIndex();
  segment_meshlet_spot_light_render_pipeline->Bind(vk_command_buffer);
  segment_meshlet_spot_light_render_pipeline->BindDescriptorSet(
      vk_command_buffer, 0, RenderLayer::GetPerFrameDescriptorSet()->GetVkDescriptorSet());
  segment_meshlet_spot_light_render_pipeline->BindDescriptorSet(
      vk_command_buffer, 1, dynamic_strands->strands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());
  segment_meshlet_spot_light_render_pipeline->states.ResetAllStates(0);
  segment_meshlet_spot_light_render_pipeline->states.SetViewportScissor(view.viewport);
  segment_meshlet_spot_light_render_pipeline->states.ApplyAllStates(vk_command_buffer);

  segment_meshlet_spot_light_render_pipeline->PushConstant(vk_command_buffer, 0, push_constant);
  const uint32_t count = Platform::DivUp(segment_meshlet_triangles.size(), task_work_group_invocations);
  vkCmdDrawMeshTasksEXT(vk_command_buffer, count, 1, 1);
  return segment_meshlet_triangles.size();
}

uint32_t DsKineticVoronoiMeshing::RenderSegmentMeshletsToDirectionalLightShadowMap(
    const SegmentMeshletsRenderParameters& render_parameters, VkCommandBuffer vk_command_buffer,
    const RenderLayer::DirectionalLightShadowMapView& view) const {
  if (!render_parameters.enabled) {
    return 0;
  }
  const uint32_t task_work_group_invocations =
      Platform::GetSelectedPhysicalDevice()->mesh_shader_properties_ext.maxPreferredTaskWorkGroupInvocations;
  SegmentMeshletPushConstant push_constant;
  push_constant.index1.sub_light_index = view.split_index;
  push_constant.index2.light_index = view.light_index;
  push_constant.vertex_count = segment_meshlet_vertices.size();
  push_constant.triangle_count = segment_meshlet_triangles.size();
  push_constant.color_mode = render_settings.segment_meshlet_render_parameters.color_mode;
  const auto current_frame_index = Platform::GetCurrentFrameIndex();
  segment_meshlet_directional_light_render_pipeline->Bind(vk_command_buffer);
  segment_meshlet_directional_light_render_pipeline->BindDescriptorSet(
      vk_command_buffer, 0, RenderLayer::GetPerFrameDescriptorSet()->GetVkDescriptorSet());
  segment_meshlet_directional_light_render_pipeline->BindDescriptorSet(
      vk_command_buffer, 1, dynamic_strands->strands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());
  segment_meshlet_directional_light_render_pipeline->states.ResetAllStates(0);
  segment_meshlet_directional_light_render_pipeline->states.SetViewportScissor(view.viewport);
  segment_meshlet_directional_light_render_pipeline->states.ApplyAllStates(vk_command_buffer);

  segment_meshlet_directional_light_render_pipeline->PushConstant(vk_command_buffer, 0, push_constant);
  const uint32_t count = Platform::DivUp(segment_meshlet_triangles.size(), task_work_group_invocations);
  vkCmdDrawMeshTasksEXT(vk_command_buffer, count, 1, 1);
  return segment_meshlet_triangles.size();
}

uint32_t DsKineticVoronoiMeshing::RenderSegmentMeshletsToCameraDeferred(
    const Handle& renderer_handle, int bark_material_index, int inner_wood_material_index, int snow_material_index,
    const SegmentMeshletsRenderParameters& render_parameters, VkCommandBuffer vk_command_buffer,
    const std::vector<VkRenderingAttachmentInfo>& geometry_pass_color_attachment_infos,
    const RenderLayer::DeferredRenderingView& view, VkPolygonMode polygon_mode) const {
  if (!render_parameters.enabled) {
    return 0;
  }
  if (!Platform::Constants::support_mesh_shader) {
    EVOENGINE_LOG("Failed to render! Mesh shader unsupported!")
    return 0;
  }

  // TODO: If we add any compute shaders, also check them here
  if (!segment_meshlet_render_pipeline || !segment_meshlet_render_pipeline->Initialized()) {
    return 0;
  }
  const auto current_frame_index = Platform::GetCurrentFrameIndex();
  const uint32_t task_work_group_invocations =
      Platform::GetSelectedPhysicalDevice()->mesh_shader_properties_ext.maxPreferredTaskWorkGroupInvocations;

  SegmentMeshletPushConstant render_push_constant;
  render_push_constant.index1.instance_index =
      Application::GetLayer<RenderLayer>()->GetCurrentRenderInstanceStorage()->GetRenderInstanceIndex(renderer_handle);
  render_push_constant.index2.camera_index = view.camera_index;
  render_push_constant.vertex_count = segment_meshlet_vertices.size();
  render_push_constant.triangle_count = segment_meshlet_triangles.size();
  render_push_constant.color_mode = render_settings.segment_meshlet_render_parameters.color_mode;
  render_push_constant.inner_wood_material_index = inner_wood_material_index;
  render_push_constant.bark_material_index = bark_material_index;
  render_push_constant.uv_height_factor = render_settings.segment_meshlet_render_parameters.uv_height_factor;
  render_push_constant.uv_circum_factor = render_settings.segment_meshlet_render_parameters.uv_circum_factor;
  render_push_constant.fracture_distance = render_settings.segment_meshlet_render_parameters.fracture_distance;

  segment_meshlet_render_pipeline->states.ResetAllStates(geometry_pass_color_attachment_infos.size());
  segment_meshlet_render_pipeline->states.SetViewportScissor(view.viewport);
  segment_meshlet_render_pipeline->states.polygon_mode = polygon_mode;
  segment_meshlet_render_pipeline->states.line_width = 2.0f;
  segment_meshlet_render_pipeline->states.ApplyAllStates(vk_command_buffer);

#ifdef USE_RENDERDOC
  if (rdoc_api) {
    rdoc_api->StartFrameCapture(NULL, NULL);
    EVOENGINE_LOG("RDOC API detected!");
  }
#endif  //  USERENDERDOC

  segment_meshlet_render_pipeline->Bind(vk_command_buffer);
  segment_meshlet_render_pipeline->BindDescriptorSet(vk_command_buffer, 0,
                                                     RenderLayer::GetPerFrameDescriptorSet()->GetVkDescriptorSet());
  segment_meshlet_render_pipeline->BindDescriptorSet(
      vk_command_buffer, 1, dynamic_strands->strands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());
  segment_meshlet_render_pipeline->BindDescriptorSet(vk_command_buffer, 2,
                                                     RenderLayer::GetLightingDescriptorSet()->GetVkDescriptorSet());

  segment_meshlet_render_pipeline->PushConstant(vk_command_buffer, 0, render_push_constant);

  const uint32_t count = Platform::DivUp(segment_meshlet_triangles.size(), task_work_group_invocations);
  vkCmdDrawMeshTasksEXT(vk_command_buffer, count, 1, 1);
#ifdef USE_RENDERDOC
  if (rdoc_api)
    rdoc_api->EndFrameCapture(NULL, NULL);
#endif
  return dynamic_strands->segments.size();
}

/* uint32_t DsKineticVoronoiMeshing::RenderSegmentMeshletVisualizationToCameraDeferred(
    const Handle& renderer_handle, const DynamicStrandsInitializeParameters& initialize_parameters,
    const SmallSegmentsVisualizationRenderParameters& render_parameters, VkCommandBuffer vk_command_buffer,
    const std::vector<VkRenderingAttachmentInfo>& geometry_pass_color_attachment_infos,
    const RenderLayer::DeferredRenderingView& view) const {
}*/