//
// Created by lllll on 9/26/2022.
//

#include "Strands.hpp"
#include "ClassRegistry.hpp"
#include "Console.hpp"
#include "GeometryStorage.hpp"
#include "Jobs.hpp"
#include "RenderLayer.hpp"
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

std::vector<glm::uint>& Strands::UnsafeGetSegments() {
  return segment_raw_indices_;
}

void Strands::PrepareStrands(const StrandPointAttributes& strand_point_attributes) {
  segments_.resize(segment_raw_indices_.size());
  Jobs::RunParallelFor(segment_raw_indices_.size(), [&](unsigned i) {
    segments_[i].x = segment_raw_indices_[i];
    segments_[i].y = segment_raw_indices_[i] + 1;
    segments_[i].z = segment_raw_indices_[i] + 2;
    segments_[i].w = segment_raw_indices_[i] + 3;
  });

#pragma region Bound
  glm::vec3 min_bound = strand_points_.at(0).position;
  glm::vec3 max_bound = strand_points_.at(0).position;
  for (auto& vertex : strand_points_) {
    min_bound = glm::vec3((glm::min)(min_bound.x, vertex.position.x), (glm::min)(min_bound.y, vertex.position.y),
                         (glm::min)(min_bound.z, vertex.position.z));
    max_bound = glm::vec3((glm::max)(max_bound.x, vertex.position.x), (glm::max)(max_bound.y, vertex.position.y),
                         (glm::max)(max_bound.z, vertex.position.z));
  }
  bound_.max = max_bound;
  bound_.min = min_bound;
#pragma endregion
  strand_point_attributes_ = strand_point_attributes;
  if (!strand_point_attributes_.normal)
    RecalculateNormal();
  strand_point_attributes_.normal = true;
  if (version_ != 0)
    GeometryStorage::FreeStrands(GetHandle());
  GeometryStorage::AllocateStrands(GetHandle(), strand_points_, segments_, strand_meshlet_range_, segment_range_);
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

bool Strands::LoadInternal(const std::filesystem::path& path) {
  if (path.extension() == ".evestrands") {
    return IAsset::LoadInternal(path);
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

bool Strands::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  bool changed = false;
  ImGui::Text(("Point size: " + std::to_string(strand_points_.size())).c_str());
  return changed;
}

void Strands::Serialize(YAML::Emitter& out) const {
  if (!segment_raw_indices_.empty() && !strand_points_.empty()) {
    out << YAML::Key << "segment_raw_indices_" << YAML::Value
        << YAML::Binary((const unsigned char*)segment_raw_indices_.data(),
                        segment_raw_indices_.size() * sizeof(glm::uint));

    out << YAML::Key << "strand_points_" << YAML::Value
        << YAML::Binary((const unsigned char*)strand_points_.data(), strand_points_.size() * sizeof(StrandPoint));
  }
}

void Strands::Deserialize(const YAML::Node& in) {
  if (in["segment_raw_indices_"] && in["strand_points_"]) {
    const auto& segment_data = in["segment_raw_indices_"].as<YAML::Binary>();
    segment_raw_indices_.resize(segment_data.size() / sizeof(glm::uint));
    std::memcpy(segment_raw_indices_.data(), segment_data.data(), segment_data.size());

    const auto& point_data = in["strand_points_"].as<YAML::Binary>();
    strand_points_.resize(point_data.size() / sizeof(StrandPoint));
    std::memcpy(strand_points_.data(), point_data.data(), point_data.size());

    StrandPointAttributes strand_point_attributes{};
    strand_point_attributes.tex_coord = true;
    strand_point_attributes.color = true;
    strand_point_attributes.normal = true;
    PrepareStrands(strand_point_attributes);
  }
}

void Strands::OnCreate() {
  version_ = 0;
  bound_ = Bound();
  segment_range_ = std::make_shared<RangeDescriptor>();
  strand_meshlet_range_ = std::make_shared<RangeDescriptor>();
}
Bound Strands::GetBound() const {
  return bound_;
}

size_t Strands::GetSegmentAmount() const {
  return segments_.size();
}

Strands::~Strands() {
  GeometryStorage::FreeStrands(GetHandle());
  segment_range_.reset();
  strand_meshlet_range_.reset();
}

size_t Strands::GetStrandPointAmount() const {
  return strand_points_.size();
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

void Strands::DrawIndexed(const VkCommandBuffer vk_command_buffer, GraphicsPipelineStates& global_pipeline_state,
                          const int instances_count) const {
  if (instances_count == 0)
    return;
  global_pipeline_state.ApplyAllStates(vk_command_buffer);
  vkCmdDrawIndexed(vk_command_buffer, segment_range_->prev_frame_index_count * 4, instances_count,
                   segment_range_->prev_frame_offset * 4, 0, 0);
}
