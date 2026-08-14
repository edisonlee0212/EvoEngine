//
// Created by lllll on 9/26/2022.
//

#include "Strands.hpp"
#include "ClassRegistry.hpp"
#include "Console.hpp"
#include "GeometryStorage.hpp"
#include "Jobs.hpp"
#include "Platform.hpp"
#include "RenderLayer.hpp"
#include "Serialization.hpp"

#include <cmath>
using namespace evo_engine;

void StrandPointAttributes::Serialize(YAML::Emitter& out) const {
  out << YAML::Key << "normal" << YAML::Value << normal;
  out << YAML::Key << "tex_coord" << YAML::Value << tex_coord;
  out << YAML::Key << "color" << YAML::Value << color;
}

void StrandPointAttributes::Deserialize(const YAML::Node& in) {
  if (in["normal"])
    normal = in["normal"].as<bool>();
  if (in["tex_coord"])
    tex_coord = in["tex_coord"].as<bool>();
  if (in["color"])
    color = in["color"].as<bool>();
}

std::vector<StrandPoint>& Strands::UnsafeGetStrandPoints() {
  return strand_points_;
}

const std::vector<StrandPoint>& Strands::PeekStrandPoints() const {
  return strand_points_;
}

std::vector<glm::uint>& Strands::UnsafeGetSegments() {
  return segment_raw_indices_;
}

const std::vector<glm::uint>& Strands::PeekSegments() const {
  return segment_raw_indices_;
}

StrandRayTracingGeometry Strands::BuildRayTracingGeometry(const std::vector<StrandPoint>& strand_points,
                                                          const std::vector<glm::uvec4>& segments) {
  constexpr uint32_t interval_count = 8;
  StrandRayTracingGeometry geometry;
  geometry.points.reserve(segments.size() * interval_count + segments.size());
  geometry.indices.reserve(segments.size() * interval_count);

  const auto finite = [](const StrandPoint& point) {
    return std::isfinite(point.position.x) && std::isfinite(point.position.y) && std::isfinite(point.position.z) &&
           std::isfinite(point.thickness) && std::isfinite(point.normal.x) && std::isfinite(point.normal.y) &&
           std::isfinite(point.normal.z) && std::isfinite(point.tex_coord) && std::isfinite(point.color.x) &&
           std::isfinite(point.color.y) && std::isfinite(point.color.z) && std::isfinite(point.color.w);
  };
  const auto sample = [&](const glm::uvec4& segment, const float u) {
    StrandPoint result;
    const auto& p0 = strand_points[segment.x];
    const auto& p1 = strand_points[segment.y];
    const auto& p2 = strand_points[segment.z];
    const auto& p3 = strand_points[segment.w];
    result.position = CubicInterpolation(p0.position, p1.position, p2.position, p3.position, u);
    result.thickness = glm::abs(CubicInterpolation(p0.thickness, p1.thickness, p2.thickness, p3.thickness, u));
    result.normal = CubicInterpolation(p0.normal, p1.normal, p2.normal, p3.normal, u);
    result.tex_coord = CubicInterpolation(p0.tex_coord, p1.tex_coord, p2.tex_coord, p3.tex_coord, u);
    result.color = CubicInterpolation(p0.color, p1.color, p2.color, p3.color, u);
    return result;
  };

  bool active_run = false;
  glm::uvec4 previous_segment{};
  bool has_previous_segment = false;
  for (const auto& segment : segments) {
    const bool valid_controls = segment.x < strand_points.size() && segment.y < strand_points.size() &&
                                segment.z < strand_points.size() && segment.w < strand_points.size();
    const bool consecutive = has_previous_segment && segment == previous_segment + glm::uvec4(1u);
    if (!consecutive) {
      active_run = false;
    }
    previous_segment = segment;
    has_previous_segment = true;
    if (!valid_controls) {
      active_run = false;
      continue;
    }

    StrandPoint first = sample(segment, 0.0f);
    for (uint32_t interval = 0; interval < interval_count; ++interval) {
      StrandPoint second = sample(segment, static_cast<float>(interval + 1) / interval_count);
      const glm::vec3 delta = second.position - first.position;
      const bool valid_primitive = finite(first) && finite(second) && glm::dot(delta, delta) > 0.0f;
      if (!valid_primitive) {
        active_run = false;
        first = second;
        continue;
      }
      if (!active_run) {
        geometry.points.emplace_back(first);
      }
      geometry.indices.emplace_back(static_cast<uint32_t>(geometry.points.size() - 1));
      geometry.points.emplace_back(second);
      active_run = true;
      first = second;
    }
  }
  return geometry;
}

void Strands::PrepareStrands(const StrandPointAttributes& strand_point_attributes) {
  segments_.resize(segment_raw_indices_.size());
  Jobs::RunParallelFor(segment_raw_indices_.size(), [&](size_t i) {
    segments_[i].x = segment_raw_indices_[i];
    segments_[i].y = segment_raw_indices_[i] + 1;
    segments_[i].z = segment_raw_indices_[i] + 2;
    segments_[i].w = segment_raw_indices_[i] + 3;
  });

#pragma region Bound
  glm::vec3 min_bound = glm::vec3(FLT_MAX);
  glm::vec3 max_bound = glm::vec3(-FLT_MAX);
  for (const auto& vertex : strand_points_) {
    const glm::vec3 radius = glm::vec3(glm::abs(vertex.thickness));
    min_bound = glm::min(min_bound, vertex.position - radius);
    max_bound = glm::max(max_bound, vertex.position + radius);
  }
  bound_.max = max_bound;
  bound_.min = min_bound;
#pragma endregion
  strand_point_attributes_ = strand_point_attributes;
  if (!strand_point_attributes_.normal)
    RecalculateNormal();
  strand_point_attributes_.normal = true;
  if (version_ != 0) {
    blas_.reset();
    GeometryStorage::FreeStrands(GetHandle());
    GeometryStorage::FreeRayTracingStrands(GetHandle());
  }
  if (Platform::Initialized() && Platform::MeshShaderEnabled()) {
    GeometryStorage::AllocateStrands(GetHandle(), strand_points_, segments_, strand_meshlet_range_, segment_range_);
  }
  if (Platform::Initialized() && Platform::RayTracingLinearSweptSpheresEnabled()) {
    auto geometry = BuildRayTracingGeometry(strand_points_, segments_);
    if (!geometry.points.empty() && !geometry.indices.empty()) {
      GeometryStorage::AllocateRayTracingStrands(GetHandle(), geometry.points, geometry.indices,
                                                 ray_tracing_point_range_, ray_tracing_index_range_);
      blas_ = BottomLevelAccelerationStructure::CreateLinearSweptSpheres(geometry.points, geometry.indices);
    }
  }
  version_++;
  saved_ = false;
}

// .hair format spec here: http://www.cemyuksel.com/research/hairmodels/
struct HairHeader {
  // Bytes 0 - 3  Must be "HAIR" in ascii code(48 41 49 52)
  char magic[4];

  // Bytes 4 - 7  Number of hair strands as unsigned int
  uint32_t num_strands;

  // Bytes 8 - 11  Total number of points of all strands as unsigned int
  uint32_t num_points;

  // Bytes 12 - 15  Bit array of data in the file
  // Bit - 5 to Bit - 31 are reserved for future extension(must be 0).
  uint32_t flags;

  // Bytes 16 - 19  Default number of segments of hair strands as unsigned int
  // If the file does not have a segments array, this default value is used.
  uint32_t default_num_segments;

  // Bytes 20 - 23  Default thickness hair strands as float
  // If the file does not have a thickness array, this default value is used.
  float default_thickness;

  // Bytes 24 - 27  Default transparency hair strands as float
  // If the file does not have a transparency array, this default value is used.
  float default_alpha;

  // Bytes 28 - 39  Default color hair strands as float array of size 3
  // If the file does not have a color array, this default value is used.
  glm::vec3 default_color;

  // Bytes 40 - 127  File information as char array of size 88 in ascii
  char file_info[88];

  [[nodiscard]] bool HasSegments() const {
    return (flags & (0x1 << 0)) > 0;
  }

  [[nodiscard]] bool HasPoints() const {
    return (flags & (0x1 << 1)) > 0;
  }

  [[nodiscard]] bool HasThickness() const {
    return (flags & (0x1 << 2)) > 0;
  }

  [[nodiscard]] bool HasAlpha() const {
    return (flags & (0x1 << 3)) > 0;
  }

  [[nodiscard]] bool HasColor() const {
    return (flags & (0x1 << 4)) > 0;
  }
};

namespace {
class StrandsStagedLoadPayload final : public StagedAssetLoadPayload {
 public:
  StrandPointAttributes strand_point_attributes = {};
  std::vector<glm::uint> segment_raw_indices;
  std::vector<StrandPoint> strand_points;
};

std::vector<glm::uint> BuildSegmentRawIndicesFromStrands(const std::vector<glm::uint>& strands) {
  std::vector<glm::uint> segment_raw_indices;
  if (strands.size() < 2) {
    return segment_raw_indices;
  }
  for (auto strand = strands.begin(); strand != strands.end() - 1; ++strand) {
    const int start = static_cast<int>(*strand);
    const int end = static_cast<int>(*(strand + 1)) - 3;
    for (int i = start; i < end; i++) {
      segment_raw_indices.emplace_back(static_cast<glm::uint>(i));
    }
  }
  return segment_raw_indices;
}
}  // namespace

bool Strands::LoadInternal(const std::filesystem::path& path) {
  if (path.extension() == ".evestrands") {
    return Serialization::LoadAssetFromYaml(*this, path);
  }
  if (path.extension() == ".hair") {
    try {
      std::string file_name = path.string();
      std::ifstream input(file_name.c_str(), std::ios::binary);
      HairHeader header;
      input.read(reinterpret_cast<char*>(&header), sizeof(HairHeader));
      assert(input);
      assert(strncmp(header.magic, "HAIR", 4) == 0);
      header.file_info[87] = 0;

      // Segments array(unsigned short)
      // The segments array contains the number of linear segments per strand;
      // thus there are segments + 1 control-points/vertices per strand.
      auto strand_segments = std::vector<unsigned short>(header.num_strands);
      if (header.HasSegments()) {
        input.read(reinterpret_cast<char*>(strand_segments.data()), header.num_strands * sizeof(unsigned short));
        assert(input);
      } else {
        std::fill(strand_segments.begin(), strand_segments.end(), header.default_num_segments);
      }

      // Compute strands vector<unsigned int>. Each element is the index to the
      // first point of the first segment of the strand. The last entry is the
      // index "one beyond the last vertex".
      auto strands = std::vector<glm::uint>(strand_segments.size() + 1);
      auto strand = strands.begin();
      *strand++ = 0;
      for (auto segments : strand_segments) {
        *strand = *(strand - 1) + 1 + segments;
        ++strand;
      }

      // Points array(float)
      assert(header.HasPoints());
      auto points = std::vector<glm::vec3>(header.num_points);
      input.read(reinterpret_cast<char*>(points.data()), header.num_points * sizeof(glm::vec3));
      assert(input);

      // Thickness array(float)
      auto thickness = std::vector<float>(header.num_points);
      if (header.HasThickness()) {
        input.read(reinterpret_cast<char*>(thickness.data()), header.num_points * sizeof(float));
        assert(input);
      } else {
        std::fill(thickness.begin(), thickness.end(), header.default_thickness);
      }

      // Color array(float)
      auto color = std::vector<glm::vec3>(header.num_points);
      if (header.HasColor()) {
        input.read(reinterpret_cast<char*>(color.data()), header.num_points * sizeof(glm::vec3));
        assert(input);
      } else {
        std::fill(color.begin(), color.end(), header.default_color);
      }

      // Alpha array(float)
      auto alpha = std::vector<float>(header.num_points);
      if (header.HasAlpha()) {
        input.read(reinterpret_cast<char*>(alpha.data()), header.num_points * sizeof(float));
        assert(input);
      } else {
        std::fill(alpha.begin(), alpha.end(), header.default_alpha);
      }
      std::vector<StrandPoint> strand_points;
      strand_points.resize(header.num_points);
      for (int i = 0; i < header.num_points; i++) {
        strand_points[i].position = points[i];
        strand_points[i].thickness = thickness[i];
        strand_points[i].color = glm::vec4(color[i], alpha[i]);
        strand_points[i].tex_coord = 0.0f;
      }
      StrandPointAttributes strand_point_attributes{};
      strand_point_attributes.tex_coord = true;
      strand_point_attributes.color = true;
      SetStrands(strand_point_attributes, strands, strand_points);
      return true;
    } catch (std::exception& e) {
      return false;
    }
  }
  return false;
}

bool Strands::SupportsStagedLoading(const std::filesystem::path& path) const {
  return path.extension() == ".evestrands" || path.extension() == ".hair";
}

std::shared_ptr<StagedAssetLoadPayload> Strands::LoadStagedPayloadInternal(const std::filesystem::path& path) const {
  try {
    auto payload = std::make_shared<StrandsStagedLoadPayload>();
    if (path.extension() == ".evestrands") {
      const std::ifstream stream(path.string());
      std::stringstream string_stream;
      string_stream << stream.rdbuf();
      const YAML::Node in = YAML::Load(string_stream.str());
      if (in["segment_raw_indices_"] && in["strand_points_"]) {
        const auto& segment_data = in["segment_raw_indices_"].as<YAML::Binary>();
        payload->segment_raw_indices.resize(segment_data.size() / sizeof(glm::uint));
        std::memcpy(payload->segment_raw_indices.data(), segment_data.data(), segment_data.size());

        const auto& point_data = in["strand_points_"].as<YAML::Binary>();
        payload->strand_points.resize(point_data.size() / sizeof(StrandPoint));
        std::memcpy(payload->strand_points.data(), point_data.data(), point_data.size());

        payload->strand_point_attributes.tex_coord = true;
        payload->strand_point_attributes.color = true;
        payload->strand_point_attributes.normal = true;
      }
      return payload;
    }

    if (path.extension() == ".hair") {
      std::ifstream input(path.string().c_str(), std::ios::binary);
      HairHeader header;
      input.read(reinterpret_cast<char*>(&header), sizeof(HairHeader));
      if (!input || strncmp(header.magic, "HAIR", 4) != 0) {
        return {};
      }
      header.file_info[87] = 0;

      auto strand_segments = std::vector<unsigned short>(header.num_strands);
      if (header.HasSegments()) {
        input.read(reinterpret_cast<char*>(strand_segments.data()), header.num_strands * sizeof(unsigned short));
        if (!input) {
          return {};
        }
      } else {
        std::fill(strand_segments.begin(), strand_segments.end(), header.default_num_segments);
      }

      auto strands = std::vector<glm::uint>(strand_segments.size() + 1);
      auto strand = strands.begin();
      *strand++ = 0;
      for (auto segments : strand_segments) {
        *strand = *(strand - 1) + 1 + segments;
        ++strand;
      }

      if (!header.HasPoints()) {
        return {};
      }
      auto points = std::vector<glm::vec3>(header.num_points);
      input.read(reinterpret_cast<char*>(points.data()), header.num_points * sizeof(glm::vec3));
      if (!input) {
        return {};
      }

      auto thickness = std::vector<float>(header.num_points);
      if (header.HasThickness()) {
        input.read(reinterpret_cast<char*>(thickness.data()), header.num_points * sizeof(float));
        if (!input) {
          return {};
        }
      } else {
        std::fill(thickness.begin(), thickness.end(), header.default_thickness);
      }

      auto color = std::vector<glm::vec3>(header.num_points);
      if (header.HasColor()) {
        input.read(reinterpret_cast<char*>(color.data()), header.num_points * sizeof(glm::vec3));
        if (!input) {
          return {};
        }
      } else {
        std::fill(color.begin(), color.end(), header.default_color);
      }

      auto alpha = std::vector<float>(header.num_points);
      if (header.HasAlpha()) {
        input.read(reinterpret_cast<char*>(alpha.data()), header.num_points * sizeof(float));
        if (!input) {
          return {};
        }
      } else {
        std::fill(alpha.begin(), alpha.end(), header.default_alpha);
      }

      payload->strand_points.resize(header.num_points);
      for (int i = 0; i < header.num_points; i++) {
        payload->strand_points[i].position = points[i];
        payload->strand_points[i].thickness = thickness[i];
        payload->strand_points[i].color = glm::vec4(color[i], alpha[i]);
        payload->strand_points[i].tex_coord = 0.0f;
      }
      payload->segment_raw_indices = BuildSegmentRawIndicesFromStrands(strands);
      payload->strand_point_attributes.tex_coord = true;
      payload->strand_point_attributes.color = true;
      return payload;
    }
  } catch (const std::exception& e) {
    EVOENGINE_ERROR("Failed to load staged strands payload: " + std::string(e.what()))
  }
  return {};
}

bool Strands::ApplyStagedPayloadInternal(const std::filesystem::path&,
                                         const std::shared_ptr<StagedAssetLoadPayload>& payload) {
  const auto strands_payload = std::dynamic_pointer_cast<StrandsStagedLoadPayload>(payload);
  if (!strands_payload || strands_payload->segment_raw_indices.empty() || strands_payload->strand_points.empty()) {
    return false;
  }
  segment_raw_indices_ = std::move(strands_payload->segment_raw_indices);
  strand_points_ = std::move(strands_payload->strand_points);
  PrepareStrands(strands_payload->strand_point_attributes);
  return true;
}

bool Strands::RegisterAssetIoHandlers(const std::string& owner_name, const std::string& type_name) {
  return Serialization::RegisterAssetIoHandler<Strands>(
      {},
      [](Strands& asset, const std::filesystem::path& path) {
        return asset.LoadInternal(path);
      },
      [](const Strands& asset, const std::filesystem::path& path) {
        return asset.SupportsStagedLoading(path);
      },
      [](const Strands& asset, const std::filesystem::path& path) {
        return asset.LoadStagedPayloadInternal(path);
      },
      [](Strands& asset, const std::filesystem::path& path, const std::shared_ptr<StagedAssetLoadPayload>& payload) {
        return asset.ApplyStagedPayloadInternal(path, payload);
      },
      owner_name, type_name);
}

void Strands::OnCreate() {
  version_ = 0;
  bound_ = Bound();
  segment_range_ = std::make_shared<RangeDescriptor>();
  strand_meshlet_range_ = std::make_shared<RangeDescriptor>();
  ray_tracing_point_range_ = std::make_shared<RangeDescriptor>();
  ray_tracing_index_range_ = std::make_shared<RangeDescriptor>();
  blas_.reset();
}
Bound Strands::GetBound() const {
  return bound_;
}

size_t Strands::GetSegmentAmount() const {
  return segments_.size();
}

Strands::~Strands() {
  blas_.reset();
  GeometryStorage::FreeStrands(GetHandle());
  GeometryStorage::FreeRayTracingStrands(GetHandle());
  segment_range_.reset();
  strand_meshlet_range_.reset();
  ray_tracing_point_range_.reset();
  ray_tracing_index_range_.reset();
}

size_t Strands::GetStrandPointAmount() const {
  return strand_points_.size();
}

const std::shared_ptr<BottomLevelAccelerationStructure>& Strands::GetBlas() const {
  return blas_;
}

void Strands::SetSegments(const StrandPointAttributes& strand_point_attributes, const std::vector<glm::uint>& segments,
                          const std::vector<StrandPoint>& points) {
  if (points.empty() || segments.empty()) {
    return;
  }

  segment_raw_indices_ = segments;
  strand_points_ = points;

  PrepareStrands(strand_point_attributes);
}

void Strands::SetStrands(const StrandPointAttributes& strand_point_attributes, const std::vector<glm::uint>& strands,
                         const std::vector<StrandPoint>& points) {
  if (points.empty() || strands.empty()) {
    return;
  }

  segment_raw_indices_.clear();
  // loop to one before end, as last strand value is the "past last valid vertex"
  // index
  for (auto strand = strands.begin(); strand != strands.end() - 1; ++strand) {
    const int start = *(strand);        // first vertex in first segment
    const int end = *(strand + 1) - 3;  // CurveDegree();  // second vertex of last segment
    for (int i = start; i < end; i++) {
      segment_raw_indices_.emplace_back(i);
    }
  }
  strand_points_ = points;
  PrepareStrands(strand_point_attributes);
}

void Strands::RecalculateNormal() {
  glm::vec3 tangent, temp;
  for (const auto& indices : segments_) {
    CubicInterpolation(strand_points_[indices[0]].position, strand_points_[indices[1]].position,
                       strand_points_[indices[2]].position, strand_points_[indices[3]].position, temp, tangent, 0.0f);
    strand_points_[indices[0]].normal = glm::vec3(tangent.y, tangent.z, tangent.x);

    CubicInterpolation(strand_points_[indices[0]].position, strand_points_[indices[1]].position,
                       strand_points_[indices[2]].position, strand_points_[indices[3]].position, temp, tangent, 0.25f);
    strand_points_[indices[1]].normal = glm::cross(glm::cross(tangent, strand_points_[indices[0]].normal), tangent);

    CubicInterpolation(strand_points_[indices[0]].position, strand_points_[indices[1]].position,
                       strand_points_[indices[2]].position, strand_points_[indices[3]].position, temp, tangent, 0.75f);
    strand_points_[indices[2]].normal = glm::cross(glm::cross(tangent, strand_points_[indices[1]].normal), tangent);

    CubicInterpolation(strand_points_[indices[0]].position, strand_points_[indices[1]].position,
                       strand_points_[indices[2]].position, strand_points_[indices[3]].position, temp, tangent, 1.0f);
    strand_points_[indices[3]].normal = glm::cross(glm::cross(tangent, strand_points_[indices[2]].normal), tangent);
  }
}
