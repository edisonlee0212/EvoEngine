#pragma once

#include "StrandGroup.hpp"
using namespace evo_engine;
namespace eco_sys_lab_plugin {
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

  out << YAML::Key << "strands_" << YAML::Value << YAML::BeginSeq;
  for (uint32_t strand_index = 0; strand_index < strand_group.strands_.size(); strand_index++) {
    const auto& strand = strand_group.strands_[strand_index];
    out << YAML::BeginMap;
    {
      if (!strand.strand_segment_handles_.empty()) {
        out << YAML::Key << "strand_segment_handles_" << YAML::Value
            << YAML::Binary(reinterpret_cast<const unsigned char*>(strand.strand_segment_handles_.data()),
                            strand.strand_segment_handles_.size() * sizeof(StrandSegmentHandle));
      }
      out << YAML::Key << "start_position" << YAML::Value << strand.start_position;
      out << YAML::Key << "start_thickness" << YAML::Value << strand.start_thickness;
      out << YAML::Key << "start_color" << YAML::Value << strand.start_color;

      out << YAML::Key << "data" << YAML::Value << YAML::BeginMap;
      { strand_func(out, strand_group.strands_data_list[strand_index]); }
      out << YAML::EndMap;
    }
    out << YAML::EndMap;
  }
  out << YAML::EndSeq;

  const auto strand_segment_size = strand_group.strand_segments_.size();
  
  auto strand_segment_prev_list = std::vector<StrandSegmentHandle>(strand_segment_size);
  auto strand_segment_next_list = std::vector<StrandSegmentHandle>(strand_segment_size);
  auto strand_segment_strand_handle_list = std::vector<StrandHandle>(strand_segment_size);
  auto strand_segment_index_list = std::vector<int>(strand_segment_size);

  auto strand_segment_position_list = std::vector<glm::vec3>(strand_segment_size);
  auto strand_segment_rotation_list = std::vector<glm::quat>(strand_segment_size);
  auto strand_segment_thickness_list = std::vector<float>(strand_segment_size);
  auto strand_segment_color_list = std::vector<glm::vec4>(strand_segment_size);

  for (int strand_segment_index = 0; strand_segment_index < strand_segment_size; strand_segment_index++) {
    const auto& strand_segment = strand_group.strand_segments_[strand_segment_index];
    
    strand_segment_prev_list[strand_segment_index] = strand_segment.prev_handle_;
    strand_segment_next_list[strand_segment_index] = strand_segment.next_handle_;
    strand_segment_strand_handle_list[strand_segment_index] = strand_segment.strand_handle_;
    strand_segment_index_list[strand_segment_index] = strand_segment.index_;

    strand_segment_position_list[strand_segment_index] = strand_segment.end_position;
    strand_segment_rotation_list[strand_segment_index] = strand_segment.rotation;

    strand_segment_thickness_list[strand_segment_index] = strand_segment.end_thickness;
    strand_segment_color_list[strand_segment_index] = strand_segment.end_color;
  }
  if (strand_segment_size != 0) {
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

    out << YAML::Key << "strand_segments_.position" << YAML::Value
        << YAML::Binary(reinterpret_cast<const unsigned char*>(strand_segment_position_list.data()),
                        strand_segment_position_list.size() * sizeof(glm::vec3));

    out << YAML::Key << "strand_segments_.rotation" << YAML::Value
        << YAML::Binary(reinterpret_cast<const unsigned char*>(strand_segment_rotation_list.data()),
                        strand_segment_rotation_list.size() * sizeof(glm::quat));

    out << YAML::Key << "strand_segments_.thickness" << YAML::Value
        << YAML::Binary(reinterpret_cast<const unsigned char*>(strand_segment_thickness_list.data()),
                        strand_segment_thickness_list.size() * sizeof(float));
    out << YAML::Key << "strand_segments_.color" << YAML::Value
        << YAML::Binary(reinterpret_cast<const unsigned char*>(strand_segment_color_list.data()),
                        strand_segment_color_list.size() * sizeof(glm::vec4));
  }
  out << YAML::Key << "strand_segments_.data" << YAML::Value << YAML::BeginSeq;
  for (uint32_t strand_segment_index = 0; strand_segment_index < strand_group.strand_segments_data_list.size(); strand_segment_index++) {
   
  }

  for (const auto& strand_segment_data : strand_group.strand_segments_data_list) {
    out << YAML::BeginMap;
    { strand_segment_func(out, strand_segment_data); }
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
  strand_group = {};
  if (in["strands_"]) {
    const auto& in_strands = in["strands_"];
    StrandHandle strand_handle = 0;
    for (const auto& in_strand : in_strands) {
      strand_group.strands_.emplace_back();
      auto& strand = strand_group.strands_.back();
      if (in_strand["strand_segment_handles_"]) {
        const auto segment_handles = in_strand["strand_segment_handles_"].as<YAML::Binary>();
        strand.strand_segment_handles_.resize(segment_handles.size() / sizeof(StrandSegmentHandle));
        std::memcpy(strand.strand_segment_handles_.data(), segment_handles.data(), segment_handles.size());
      }
      if (in_strand["start_position"]) {
        strand.start_position = in_strand["start_position"].as<glm::vec3>();
      }
      if (in_strand["start_color"]) {
        strand.start_color = in_strand["start_color"].as<glm::vec4>();
      }
      if (in_strand["start_thickness"]) {
        strand.start_thickness = in_strand["start_thickness"].as<float>();
      }
      if (in_strand["data"]) {
        const auto& in_strand_data = in_strand["D"];
        strand_group.strands_data_list.emplace_back();
        strand_func(in_strand_data, strand_group.strands_data_list.back());
      }
      strand_handle++;
    }
  }


  if (in["strand_segments_.prev_handle_"]) {
    auto list = std::vector<StrandSegmentHandle>();
    const auto data = in["strand_segments_.prev_handle_"].as<YAML::Binary>();
    list.resize(data.size() / sizeof(StrandSegmentHandle));
    strand_group.strand_segments_.resize(list.size());
    strand_group.strand_segments_data_list.resize(list.size());
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

  if (in["strand_segments_.position"]) {
    auto list = std::vector<glm::vec3>();
    const auto data = in["strand_segments_.position"].as<YAML::Binary>();
    list.resize(data.size() / sizeof(glm::vec3));
    std::memcpy(list.data(), data.data(), data.size());
    for (size_t i = 0; i < list.size(); i++) {
      strand_group.strand_segments_[i].end_position = list[i];
    }
  }
  
  if (in["strand_segments_.rotation"]) {
    auto list = std::vector<glm::quat>();
    const auto data = in["strand_segments_.rotation"].as<YAML::Binary>();
    list.resize(data.size() / sizeof(glm::quat));
    std::memcpy(list.data(), data.data(), data.size());
    for (size_t i = 0; i < list.size(); i++) {
      strand_group.strand_segments_[i].rotation = list[i];
    }
  }

  if (in["strand_segments_.thickness"]) {
    auto list = std::vector<float>();
    const auto data = in["strand_segments_.thickness"].as<YAML::Binary>();
    list.resize(data.size() / sizeof(float));
    std::memcpy(list.data(), data.data(), data.size());
    for (size_t i = 0; i < list.size(); i++) {
      strand_group.strand_segments_[i].end_thickness = list[i];
    }
  }

  if (in["strand_segments_.color"]) {
    auto list = std::vector<glm::vec4>();
    const auto data = in["strand_segments_.color"].as<YAML::Binary>();
    list.resize(data.size() / sizeof(glm::vec4));
    std::memcpy(list.data(), data.data(), data.size());
    for (size_t i = 0; i < list.size(); i++) {
      strand_group.strand_segments_[i].end_color = list[i];
    }
  }

  if (in["strand_segments_"]) {
    const auto& in_strand_segments = in["strand_segments_"];
    StrandSegmentHandle strand_segment_handle = 0;
    for (const auto& in_strand_segment : in_strand_segments) {
      auto& strand_segment = strand_group.strand_segments_.at(strand_segment_handle);
      strand_segment_func(in_strand_segment, strand_group.strand_segments_data_list[strand_segment_handle]);
      strand_segment_handle++;
    }
  }
  

  if (in["data"])
    group_func(in["data"], strand_group.data);
}
}  // namespace eco_sys_lab_plugin
