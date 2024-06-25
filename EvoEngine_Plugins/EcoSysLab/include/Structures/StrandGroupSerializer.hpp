#pragma once

#include "StrandGroup.hpp"
using namespace evo_engine;
namespace eco_sys_lab {
template <typename StrandGroupData, typename StrandData, typename StrandSegmentData>
class StrandGroupSerializer {
 public:
  static void Serialize(
      YAML::Emitter& out, const StrandGroup<StrandGroupData, StrandData, StrandSegmentData>& strand_group,
      const std::function<void(YAML::Emitter& strand_segment_out, const StrandSegmentData& strand_segment_data)>&
          strand_segment_func,
      const std::function<void(YAML::Emitter& strand_out, const StrandData& strand_data)>& strand_func,
      const std::function<void(YAML::Emitter& group_out, const StrandGroupData& group_data)>& group_func);

  static void Deserialize(
      const YAML::Node& in, StrandGroup<StrandGroupData, StrandData, StrandSegmentData>& strand_group,
      const std::function<void(const YAML::Node& strand_segment_in, StrandSegmentData& segment_data)>&
          strand_segment_func,
      const std::function<void(const YAML::Node& strand_in, StrandData& strand_data)>& strand_func,
      const std::function<void(const YAML::Node& group_in, StrandGroupData& group_data)>& group_func);
};

template <typename StrandGroupData, typename StrandData, typename StrandSegmentData>
void StrandGroupSerializer<StrandGroupData, StrandData, StrandSegmentData>::Serialize(
    YAML::Emitter& out, const StrandGroup<StrandGroupData, StrandData, StrandSegmentData>& strand_group,
    const std::function<void(YAML::Emitter& strand_segment_out, const StrandSegmentData& strand_segment_data)>&
        strand_segment_func,
    const std::function<void(YAML::Emitter& strand_out, const StrandData& strand_data)>& strand_func,
    const std::function<void(YAML::Emitter& group_out, const StrandGroupData& group_data)>& group_func) {
  const auto strand_size = strand_group.strands_.size();
  auto strand_recycled_list = std::vector<int>(strand_size);
  auto strand_info_color_list = std::vector<glm::vec4>(strand_size);
  auto strand_info_base_info_global_position_list = std::vector<glm::vec3>(strand_size);
  auto strand_info_base_info_thickness_list = std::vector<float>(strand_size);
  auto strand_info_base_info_color_list = std::vector<glm::vec4>(strand_size);
  auto strand_info_base_info_is_boundary_list = std::vector<int>(strand_size);

  for (int strand_index = 0; strand_index < strand_size; strand_index++) {
    const auto& strand = strand_group.strands_[strand_index];
    strand_recycled_list[strand_index] = strand.recycled_ ? 1 : 0;
    strand_info_color_list[strand_index] = strand.info.color;
    strand_info_base_info_global_position_list[strand_index] = strand.info.base_info.global_position;
    strand_info_base_info_thickness_list[strand_index] = strand.info.base_info.thickness;
    strand_info_base_info_color_list[strand_index] = strand.info.base_info.color;
    strand_info_base_info_is_boundary_list[strand_index] = strand.info.base_info.is_boundary;
  }
  if (strand_size != 0) {
    out << YAML::Key << "strands_.recycled_" << YAML::Value
        << YAML::Binary(reinterpret_cast<const unsigned char*>(strand_recycled_list.data()),
                        strand_recycled_list.size() * sizeof(int));
    out << YAML::Key << "strands_.info.color" << YAML::Value
        << YAML::Binary(reinterpret_cast<const unsigned char*>(strand_info_color_list.data()),
                        strand_info_color_list.size() * sizeof(glm::vec4));
    out << YAML::Key << "strands_.info.base_info.global_position" << YAML::Value
        << YAML::Binary(reinterpret_cast<const unsigned char*>(strand_info_base_info_global_position_list.data()),
                        strand_info_base_info_global_position_list.size() * sizeof(glm::vec3));
    out << YAML::Key << "strands_.info.base_info.thickness" << YAML::Value
        << YAML::Binary(reinterpret_cast<const unsigned char*>(strand_info_base_info_thickness_list.data()),
                        strand_info_base_info_thickness_list.size() * sizeof(float));
    out << YAML::Key << "strands_.info.base_info.color" << YAML::Value
        << YAML::Binary(reinterpret_cast<const unsigned char*>(strand_info_base_info_color_list.data()),
                        strand_info_base_info_color_list.size() * sizeof(glm::vec4));
    out << YAML::Key << "strands_.info.base_info.is_boundary" << YAML::Value
        << YAML::Binary(reinterpret_cast<const unsigned char*>(strand_info_base_info_is_boundary_list.data()),
                        strand_info_base_info_is_boundary_list.size() * sizeof(int));
  }
  out << YAML::Key << "strands_" << YAML::Value << YAML::BeginSeq;
  for (const auto& strand : strand_group.strands_) {
    out << YAML::BeginMap;
    {
      if (!strand.strand_segment_handles_.empty()) {
        out << YAML::Key << "strand_segment_handles_" << YAML::Value
            << YAML::Binary(reinterpret_cast<const unsigned char*>(strand.strand_segment_handles_.data()),
                            strand.strand_segment_handles_.size() * sizeof(StrandSegmentHandle));
      }
      out << YAML::Key << "data" << YAML::Value << YAML::BeginMap;
      { strand_func(out, strand.data); }
      out << YAML::EndMap;
    }
    out << YAML::EndMap;
  }
  out << YAML::EndSeq;

  const auto strand_segment_size = strand_group.strand_segments_.size();
  auto strand_segment_end_segment_list = std::vector<int>(strand_segment_size);
  auto strand_segment_recycled_list = std::vector<int>(strand_segment_size);

  auto strand_segment_prev_list = std::vector<StrandSegmentHandle>(strand_segment_size);
  auto strand_segment_next_list = std::vector<StrandSegmentHandle>(strand_segment_size);
  auto strand_segment_strand_handle_list = std::vector<StrandHandle>(strand_segment_size);
  auto strand_segment_index_list = std::vector<int>(strand_segment_size);

  auto strand_segment_global_position_list = std::vector<glm::vec3>(strand_segment_size);
  auto strand_segment_thickness_list = std::vector<float>(strand_segment_size);
  auto strand_segment_color_list = std::vector<glm::vec4>(strand_segment_size);
  auto strand_segment_boundary_list = std::vector<int>(strand_segment_size);

  for (int strand_segment_index = 0; strand_segment_index < strand_segment_size; strand_segment_index++) {
    const auto& strand_segment = strand_group.strand_segments_[strand_segment_index];
    strand_segment_recycled_list[strand_segment_index] = strand_segment.recycled_ ? 1 : 0;
    strand_segment_end_segment_list[strand_segment_index] = strand_segment.end_segment_ ? 1 : 0;

    strand_segment_prev_list[strand_segment_index] = strand_segment.prev_handle_;
    strand_segment_next_list[strand_segment_index] = strand_segment.next_handle_;
    strand_segment_strand_handle_list[strand_segment_index] = strand_segment.strand_handle_;
    strand_segment_index_list[strand_segment_index] = strand_segment.index_;

    strand_segment_global_position_list[strand_segment_index] = strand_segment.info.global_position;
    strand_segment_thickness_list[strand_segment_index] = strand_segment.info.thickness;
    strand_segment_color_list[strand_segment_index] = strand_segment.info.color;
    strand_segment_boundary_list[strand_segment_index] = strand_segment.info.is_boundary;
  }
  if (strand_segment_size != 0) {
    out << YAML::Key << "strand_segments_.end_segment_" << YAML::Value
        << YAML::Binary(reinterpret_cast<const unsigned char*>(strand_segment_end_segment_list.data()),
                        strand_segment_end_segment_list.size() * sizeof(int));
    out << YAML::Key << "strand_segments_.recycled_" << YAML::Value
        << YAML::Binary(reinterpret_cast<const unsigned char*>(strand_segment_recycled_list.data()),
                        strand_segment_recycled_list.size() * sizeof(int));

    out << YAML::Key << "strand_segments_.prev_handle_" << YAML::Value
        << YAML::Binary(reinterpret_cast<const unsigned char*>(strand_segment_prev_list.data()),
                        strand_segment_prev_list.size() * sizeof(StrandSegmentHandle));
    out << YAML::Key << "strand_segments_.next_handle_" << YAML::Value
        << YAML::Binary(reinterpret_cast<const unsigned char*>(strand_segment_next_list.data()),
                        strand_segment_next_list.size() * sizeof(StrandSegmentHandle));
    out << YAML::Key << "strand_segments_.strand_handle_" << YAML::Value
        << YAML::Binary(reinterpret_cast<const unsigned char*>(strand_segment_strand_handle_list.data()),
                        strand_segment_strand_handle_list.size() * sizeof(StrandHandle));
    out << YAML::Key << "strand_segments_.index_" << YAML::Value
        << YAML::Binary(reinterpret_cast<const unsigned char*>(strand_segment_index_list.data()),
                        strand_segment_index_list.size() * sizeof(int));

    out << YAML::Key << "strand_segments_.info.global_position" << YAML::Value
        << YAML::Binary(reinterpret_cast<const unsigned char*>(strand_segment_global_position_list.data()),
                        strand_segment_global_position_list.size() * sizeof(glm::vec3));
    out << YAML::Key << "strand_segments_.info.thickness" << YAML::Value
        << YAML::Binary(reinterpret_cast<const unsigned char*>(strand_segment_thickness_list.data()),
                        strand_segment_thickness_list.size() * sizeof(float));
    out << YAML::Key << "strand_segments_.info.color" << YAML::Value
        << YAML::Binary(reinterpret_cast<const unsigned char*>(strand_segment_color_list.data()),
                        strand_segment_color_list.size() * sizeof(glm::vec4));
    out << YAML::Key << "strand_segments_.info.is_boundary" << YAML::Value
        << YAML::Binary(reinterpret_cast<const unsigned char*>(strand_segment_boundary_list.data()),
                        strand_segment_boundary_list.size() * sizeof(int));
  }
  out << YAML::Key << "strand_segments_.data" << YAML::Value << YAML::BeginSeq;
  for (const auto& strand_segment : strand_group.strand_segments_) {
    out << YAML::BeginMap;
    { strand_segment_func(out, strand_segment.data); }
    out << YAML::EndMap;
  }
  out << YAML::EndSeq;

  out << YAML::Key << "data" << YAML::Value << YAML::BeginMap;
  group_func(out, strand_group.data);
  out << YAML::EndMap;
}

template <typename StrandGroupData, typename StrandData, typename StrandSegmentData>
void StrandGroupSerializer<StrandGroupData, StrandData, StrandSegmentData>::Deserialize(
    const YAML::Node& in, StrandGroup<StrandGroupData, StrandData, StrandSegmentData>& strand_group,
    const std::function<void(const YAML::Node& segment_in, StrandSegmentData& segment_data)>& strand_segment_func,
    const std::function<void(const YAML::Node& strand_in, StrandData& strand_data)>& strand_func,
    const std::function<void(const YAML::Node& group_in, StrandGroupData& group_data)>& group_func) {
  if (in["strands_.recycled_"]) {
    auto list = std::vector<int>();
    const auto data = in["strands_.recycled_"].as<YAML::Binary>();
    list.resize(data.size() / sizeof(int));
    std::memcpy(list.data(), data.data(), data.size());

    strand_group.strands_.resize(list.size());
    for (size_t i = 0; i < list.size(); i++) {
      strand_group.strands_[i].recycled_ = list[i] == 1;
    }
  }

  if (in["strands_.info.color"]) {
    auto list = std::vector<glm::vec4>();
    const auto data = in["strands_.info.color"].as<YAML::Binary>();
    list.resize(data.size() / sizeof(glm::vec4));
    std::memcpy(list.data(), data.data(), data.size());
    for (size_t i = 0; i < list.size(); i++) {
      strand_group.strands_[i].info.color = list[i];
    }
  }

  if (in["strands_.info.base_info.global_position"]) {
    auto list = std::vector<glm::vec3>();
    const auto data = in["strands_.info.base_info.global_position"].as<YAML::Binary>();
    list.resize(data.size() / sizeof(glm::vec3));
    std::memcpy(list.data(), data.data(), data.size());
    for (size_t i = 0; i < list.size(); i++) {
      strand_group.strands_[i].info.base_info.global_position = list[i];
    }
  }

  if (in["strands_.info.base_info.thickness"]) {
    auto list = std::vector<float>();
    const auto data = in["strands_.info.base_info.thickness"].as<YAML::Binary>();
    list.resize(data.size() / sizeof(float));
    std::memcpy(list.data(), data.data(), data.size());
    for (size_t i = 0; i < list.size(); i++) {
      strand_group.strands_[i].info.base_info.thickness = list[i];
    }
  }

  if (in["strands_.info.base_info.color"]) {
    auto list = std::vector<glm::vec4>();
    const auto data = in["strands_.info.base_info.color"].as<YAML::Binary>();
    list.resize(data.size() / sizeof(glm::vec4));
    std::memcpy(list.data(), data.data(), data.size());
    for (size_t i = 0; i < list.size(); i++) {
      strand_group.strands_[i].info.base_info.color = list[i];
    }
  }

  if (in["strands_.info.base_info.is_boundary"]) {
    auto list = std::vector<int>();
    const auto data = in["strands_.info.base_info.is_boundary"].as<YAML::Binary>();
    list.resize(data.size() / sizeof(int));
    std::memcpy(list.data(), data.data(), data.size());
    for (size_t i = 0; i < list.size(); i++) {
      strand_group.strands_[i].info.base_info.is_boundary = list[i] == 1;
    }
  }

  if (in["strands_"]) {
    const auto& in_strands = in["strands_"];
    StrandHandle strand_handle = 0;
    for (const auto& in_strand : in_strands) {
      auto& strand = strand_group.strands_.at(strand_handle);
      strand.handle_ = strand_handle;
      if (in_strand["strand_segment_handles_"]) {
        const auto segment_handles = in_strand["strand_segment_handles_"].as<YAML::Binary>();
        strand.strand_segment_handles_.resize(segment_handles.size() / sizeof(StrandSegmentHandle));
        std::memcpy(strand.strand_segment_handles_.data(), segment_handles.data(), segment_handles.size());
      }
      if (in_strand["data"]) {
        const auto& in_strand_data = in_strand["D"];
        strand_func(in_strand_data, strand.data);
      }
      strand_handle++;
    }
  }

  if (in["strand_segments_.end_segment_"]) {
    auto list = std::vector<int>();
    const auto data = in["strand_segments_.end_segment_"].as<YAML::Binary>();
    list.resize(data.size() / sizeof(int));
    std::memcpy(list.data(), data.data(), data.size());

    strand_group.strand_segments_.resize(list.size());
    for (size_t i = 0; i < list.size(); i++) {
      strand_group.strand_segments_[i].end_segment_ = list[i] == 1;
    }
  }

  if (in["strand_segments_.recycled_"]) {
    auto list = std::vector<int>();
    const auto data = in["strand_segments_.recycled_"].as<YAML::Binary>();
    list.resize(data.size() / sizeof(int));
    std::memcpy(list.data(), data.data(), data.size());
    for (size_t i = 0; i < list.size(); i++) {
      strand_group.strand_segments_[i].recycled_ = list[i] == 1;
    }
  }

  if (in["strand_segments_.prev_handle_"]) {
    auto list = std::vector<StrandSegmentHandle>();
    const auto data = in["strand_segments_.prev_handle_"].as<YAML::Binary>();
    list.resize(data.size() / sizeof(StrandSegmentHandle));
    std::memcpy(list.data(), data.data(), data.size());
    for (size_t i = 0; i < list.size(); i++) {
      strand_group.strand_segments_[i].prev_handle_ = list[i];
    }
  }

  if (in["strand_segments_.next_handle_"]) {
    auto list = std::vector<StrandSegmentHandle>();
    const auto data = in["strand_segments_.next_handle_"].as<YAML::Binary>();
    list.resize(data.size() / sizeof(StrandSegmentHandle));
    std::memcpy(list.data(), data.data(), data.size());
    for (size_t i = 0; i < list.size(); i++) {
      strand_group.strand_segments_[i].next_handle_ = list[i];
    }
  }

  if (in["strand_segments_.strand_handle_"]) {
    auto list = std::vector<StrandHandle>();
    const auto data = in["strand_segments_.strand_handle_"].as<YAML::Binary>();
    list.resize(data.size() / sizeof(StrandHandle));
    std::memcpy(list.data(), data.data(), data.size());
    for (size_t i = 0; i < list.size(); i++) {
      strand_group.strand_segments_[i].strand_handle_ = list[i];
    }
  }

  if (in["strand_segments_.index_"]) {
    auto list = std::vector<int>();
    const auto data = in["strand_segments_.index_"].as<YAML::Binary>();
    list.resize(data.size() / sizeof(int));
    std::memcpy(list.data(), data.data(), data.size());
    for (size_t i = 0; i < list.size(); i++) {
      strand_group.strand_segments_[i].index_ = list[i];
    }
  }

  if (in["strand_segments_.info.global_position"]) {
    auto list = std::vector<glm::vec3>();
    const auto data = in["strand_segments_.info.global_position"].as<YAML::Binary>();
    list.resize(data.size() / sizeof(glm::vec3));
    std::memcpy(list.data(), data.data(), data.size());
    for (size_t i = 0; i < list.size(); i++) {
      strand_group.strand_segments_[i].info.global_position = list[i];
    }
  }

  if (in["strand_segments_.info.thickness"]) {
    auto list = std::vector<float>();
    const auto data = in["strand_segments_.info.thickness"].as<YAML::Binary>();
    list.resize(data.size() / sizeof(float));
    std::memcpy(list.data(), data.data(), data.size());
    for (size_t i = 0; i < list.size(); i++) {
      strand_group.strand_segments_[i].info.thickness = list[i];
    }
  }

  if (in["strand_segments_.info.color"]) {
    auto list = std::vector<glm::vec4>();
    const auto data = in["strand_segments_.info.color"].as<YAML::Binary>();
    list.resize(data.size() / sizeof(glm::vec4));
    std::memcpy(list.data(), data.data(), data.size());
    for (size_t i = 0; i < list.size(); i++) {
      strand_group.strand_segments_[i].info.color = list[i];
    }
  }

  if (in["strand_segments_.info.is_boundary"]) {
    auto list = std::vector<int>();
    const auto data = in["strand_segments_.info.is_boundary"].as<YAML::Binary>();
    list.resize(data.size() / sizeof(int));
    std::memcpy(list.data(), data.data(), data.size());
    for (size_t i = 0; i < list.size(); i++) {
      strand_group.strand_segments_[i].info.is_boundary = list[i] == 1;
    }
  }

  if (in["strand_segments_"]) {
    const auto& in_strand_segments = in["strand_segments_"];
    StrandSegmentHandle strand_segment_handle = 0;
    for (const auto& in_strand_segment : in_strand_segments) {
      auto& strand_segment = strand_group.strand_segments_.at(strand_segment_handle);
      strand_segment.handle_ = strand_segment_handle;
      strand_segment_func(in_strand_segment, strand_segment.data);
      strand_segment_handle++;
    }
  }
  strand_group.strand_pool_ = {};
  strand_group.strand_segment_pool_ = {};

  for (const auto& strand : strand_group.strands_) {
    if (strand.recycled_) {
      strand_group.strand_pool_.emplace(strand.handle_);
    }
  }

  for (const auto& strand_segment : strand_group.strand_segments_) {
    if (strand_segment.recycled_) {
      strand_group.strand_segment_pool_.emplace(strand_segment.handle_);
    }
  }

  if (in["data"])
    group_func(in["data"], strand_group.data);
}
}  // namespace eco_sys_lab
