#pragma once

#include "Vertex.hpp"

using namespace evo_engine;
namespace eco_sys_lab_plugin {

typedef int StrandHandle;
typedef int StrandSegmentHandle;

/**
 * \brief The data structure that holds a strand segment.
 */
class StrandSegment {
  template <typename Pgd, typename Pd, typename Psd>
  friend class StrandGroup;
  template <typename Sgd, typename Sd, typename Ssd>
  friend class StrandGroupSerializer;

  StrandSegmentHandle prev_handle_ = -1;
  StrandSegmentHandle next_handle_ = -1;

  StrandHandle strand_handle_ = -1;

  uint32_t index_ = -1;

 public:
  /**
   * \brief The position of the [[[END]]] of current strand segment.
   */
  glm::vec3 end_position = glm::vec3(0.0f);
  /**
   * \brief The thickness of the [[[END]]] current strand segment.
   */
  float end_thickness = 0.0f;

  /**
   * \brief The color of the [[[END]]] current strand segment.
   */
  glm::vec4 end_color = glm::vec4(1.0f);

  /**
   * \brief The rotation of current strand segment.
   */
  glm::quat rotation{};

  //=====================================================
  // Cosserat Rod
  //=====================================================
  
  glm::vec3 initial_end_position = glm::vec3(0.0f);

  float inv_end_mass;

  glm::vec3 end_velocity = glm::vec3(0.0f);

  float stiffness;

  glm::quat initial_rotation{};

  glm::vec3 angular_velocity;


  /**
   * Whether this segment is the end segment.
   * @return True if this is end segment, false else wise.
   */
  [[nodiscard]] bool IsEnd() const;

  /**
   * Get the handle of belonged strand.
   * @return strandHandle of current segment.
   */
  [[nodiscard]] StrandHandle GetStrandHandle() const;
  /**
   * Get the handle of prev segment.
   * @return strandSegmentHandle of current segment.
   */
  [[nodiscard]] StrandSegmentHandle GetPrevHandle() const;

  /**
   * Get the handle of prev segment.
   * @return strandSegmentHandle of current segment.
   */
  [[nodiscard]] StrandSegmentHandle GetNextHandle() const;

  [[nodiscard]] uint32_t GetIndex() const;
  StrandSegment() = default;
  explicit StrandSegment(StrandHandle strand_handle, StrandSegmentHandle prev_handle);
};

/**
 * \brief The data structure that holds a strand.
 */

class Strand {
  template <typename Pgd, typename Pd, typename Psd>
  friend class StrandGroup;
  template <typename Sgd, typename Sd, typename Ssd>
  friend class StrandGroupSerializer;
  std::vector<StrandSegmentHandle> strand_segment_handles_{};

 public:
  /**
   * \brief The position of the [[[START]]] of first strand segment.
   */
  glm::vec3 start_position = glm::vec3(0.0f);
  /**
   * \brief The thickness of the [[[START]]] current strand segment.
   */
  float start_thickness = 0.0f;

  /**
   * \brief The color of the [[[START]]] current strand segment.
   */
  glm::vec4 start_color = glm::vec4(1.0f);

  //=====================================================
  // Cosserat Rod
  //=====================================================

  glm::vec3 initial_start_position = glm::vec3(0.0f);

  glm::vec3 start_velocity = glm::vec3(0.0f);

  float inv_start_mass;

  /**
   * Access the segments that belongs to this flow.
   * @return The list of handles.
   */
  [[nodiscard]] const std::vector<StrandSegmentHandle>& PeekStrandSegmentHandles() const;
};

/**
 * \brief The data structure that holds a collection of strands.
 * \tparam StrandGroupData The customizable data for entire strand group.
 * \tparam StrandData The customizable data for each strand.
 * \tparam StrandSegmentData The customizable data for each strand segment.
 */
template <typename StrandGroupData, typename StrandData, typename StrandSegmentData>
class StrandGroup {
  template <typename Sgd, typename Sd, typename Ssd>
  friend class StrandGroupSerializer;
  std::vector<Strand> strands_;
  std::vector<StrandSegment> strand_segments_;

  std::vector<StrandData> strands_data_list;
  std::vector<StrandSegmentData> strand_segments_data_list;

  int version_ = -1;
  void BuildStrand(const Strand& strand, std::vector<glm::uint>& strands, std::vector<StrandPoint>& points,
                   int node_max_count) const;

  [[nodiscard]] StrandSegmentHandle AllocateStrandSegment(StrandHandle strand_handle, StrandSegmentHandle prev_handle,
                                                          int index);

  void RemoveSingleStrandSegment(StrandSegmentHandle target_segment_handle);

 public:
  void RegulateRotations();

  void BuildStrands(std::vector<glm::uint>& strands, std::vector<StrandPoint>& points, int node_max_count) const;

  void BuildParticles(std::vector<ParticleInfo>& particle_infos) const;

  StrandGroupData data;

  [[nodiscard]] StrandHandle AllocateStrand();

  /**
   * Extend strand during growth process. The flow structure will also be updated.
   * @param target_handle The handle of the segment to branch/prolong
   * @return The handle of new segment.
   */
  [[nodiscard]] StrandSegmentHandle Extend(StrandHandle target_handle);

  /**
   * Insert strand segment during growth process. The flow structure will also be updated.
   * @param target_handle The handle of the strand to be inserted.
   * @param target_segment_handle The handle of the strand segment to be inserted. If there's no subsequent segment this
   * will be a simple extend.
   * @return The handle of new segment.
   */
  [[nodiscard]] StrandSegmentHandle Insert(StrandHandle target_handle, StrandSegmentHandle target_segment_handle);

  /**
   * Recycle (Remove) a segment, the descendants of this segment will also be recycled. The relevant flow will also be
   * removed/restructured.
   * @param target_segment_handle The handle of the segment to be removed. Must be valid (non-zero and the segment
   * should not be recycled prior to this operation).
   */
  void CutLatter(StrandSegmentHandle target_segment_handle);

  /**
   * Recycle (Remove) a strand. The relevant segment will also be removed/restructured.
   * @param target_strand_handle The handle of the strand to be removed. Must be valid (non-zero and the flow should not
   * be recycled prior to this operation).
   */
  void RemoveStrand(StrandHandle target_strand_handle);

  /**
   * \brief Get a unmodifiable reference to all strands.
   * \return A constant reference to all strands.
   */
  [[nodiscard]] const std::vector<Strand>& PeekStrands() const;

  /**
   * \brief Get a unmodifiable reference to data of all strands.
   * \return A constant reference to data of all strands.
   */
  [[nodiscard]] const std::vector<StrandData>& PeekStrandDataList() const;
  /**
   * \brief Get a unmodifiable reference to all strand segments.
   * \return A constant reference to all strand segments.
   */
  [[nodiscard]] const std::vector<StrandSegment>& PeekStrandSegments() const;
  /**
   * \brief Get a unmodifiable reference to data of all strand segments.
   * \return A constant reference to data to all strand segments.
   */
  [[nodiscard]] const std::vector<StrandSegmentData>& PeekStrandSegmentDataList() const;
  /**
   * \brief Get a reference to all strands.
   * \return A reference to all strands.
   */
  [[nodiscard]] std::vector<Strand>& RefStrands();

  /**
   * \brief Get a reference to data of all strands.
   * \return A reference to data of all strands.
   */
  [[nodiscard]] std::vector<StrandData>& RefStrandDataList();
  /**
   * \brief Get a reference to all strand segments.
   * \return A reference to all strand segments.
   */
  [[nodiscard]] std::vector<StrandSegment>& RefStrandSegments();
  /**
   * \brief Get a reference to data of all strand segments.
   * \return A reference to data of all strand segments.
   */
  [[nodiscard]] std::vector<StrandSegmentData>& RefStrandSegmentDataList();
  /**
   * \brief Get a reference to a specific strand.
   * \param handle The handle of the strand.
   * \return A reference to the target strand.
   */
  [[nodiscard]] Strand& RefStrand(StrandHandle handle);
  /**
   * \brief Get a reference to a specific strand segment.
   * \param handle The handle of the strand segment.
   * \return A reference to the target strand segment.
   */
  [[nodiscard]] StrandSegment& RefStrandSegment(StrandSegmentHandle handle);
  /**
   * \brief Get a unmodifiable reference to a specific strand.
   * \param handle The handle of the strand.
   * \return A unmodifiable reference to the target strand.
   */
  [[nodiscard]] const Strand& PeekStrand(StrandHandle handle) const;
  /**
   * \brief Get a unmodifiable reference to a specific strand segment.
   * \param handle The handle of the strand segment.
   * \return A unmodifiable reference to the target strand segment.
   */
  [[nodiscard]] const StrandSegment& PeekStrandSegment(StrandSegmentHandle handle) const;

  /**
   * \brief Get a reference to a specific strand data.
   * \param handle The handle of the strand.
   * \return A reference to the target strand.
   */
  [[nodiscard]] StrandData& RefStrandData(StrandHandle handle);
  /**
   * \brief Get a reference to a specific strand segment data.
   * \param handle The handle of the strand segment.
   * \return A reference to the target strand segment.
   */
  [[nodiscard]] StrandSegmentData& RefStrandSegmentData(StrandSegmentHandle handle);
  /**
   * \brief Get a unmodifiable reference to a specific strand.
   * \param handle The handle of the strand.
   * \return A unmodifiable reference to the target strand.
   */
  [[nodiscard]] const StrandData& PeekStrandData(StrandHandle handle) const;
  /**
   * \brief Get a unmodifiable reference to a specific strand segment data.
   * \param handle The handle of the strand segment.
   * \return A unmodifiable reference to the target strand segment.
   */
  [[nodiscard]] const StrandSegmentData& PeekStrandSegmentData(StrandSegmentHandle handle) const;

  /**
   * Get the structural version of the tree. The version will change when the tree structure changes.
   * @return The version
   */
  [[nodiscard]] int GetVersion() const;
  /**
   * \brief Get the start position of a specific strand segment.
   * \param handle The handle of the strand segment.
   * \return The start position of a specific strand segment.
   */
  [[nodiscard]] glm::vec3 GetStrandSegmentStart(StrandSegmentHandle handle) const;
  /**
   * \brief Get the end position of a specific strand segment.
   * \param handle The handle of the strand segment.
   * \return The end position of a specific strand segment.
   */
  [[nodiscard]] glm::vec3 GetStrandSegmentCenter(StrandSegmentHandle handle) const;
  /**
   * \brief Get the length of a specific strand segment.
   * \param handle The handle of the strand segment.
   * \return The length of a specific strand segment.
   */
  [[nodiscard]] float GetStrandSegmentLength(StrandSegmentHandle handle) const;
};

template <typename StrandGroupData, typename StrandData, typename StrandSegmentData>
void StrandGroup<StrandGroupData, StrandData, StrandSegmentData>::BuildStrand(const Strand& strand,
                                                                              std::vector<glm::uint>& strands,
                                                                              std::vector<StrandPoint>& points,
                                                                              int node_max_count) const {
  const auto& strand_segment_handles = strand.PeekStrandSegmentHandles();
  if (strand_segment_handles.empty())
    return;

  const auto start_index = points.size();
  strands.emplace_back(start_index);
  StrandPoint base_point;
  base_point.color = strand.start_color;
  base_point.thickness = strand.start_thickness;
  base_point.position = strand.start_position;

  points.emplace_back(base_point);
  points.emplace_back(base_point);

  StrandPoint point;
  for (int i = 0; i < strand_segment_handles.size() && (node_max_count == -1 || i < node_max_count); i++) {
    const auto& strand_segment = PeekStrandSegment(strand_segment_handles[i]);
    point.color = strand_segment.end_color;
    point.thickness = strand_segment.end_thickness;
    point.position = strand_segment.end_position;
    points.emplace_back(point);
  }
  auto& back_point = points.at(points.size() - 2);
  auto& last_point = points.at(points.size() - 1);

  point.color = 2.0f * last_point.color - back_point.color;
  point.thickness = 2.0f * last_point.thickness - back_point.thickness;
  point.position = 2.0f * last_point.position - back_point.position;
  points.emplace_back(point);

  auto& first_point = points.at(start_index);
  auto& second_point = points.at(start_index + 1);
  auto& third_point = points.at(start_index + 2);
  first_point.color = 2.0f * second_point.color - third_point.color;
  first_point.thickness = 2.0f * second_point.thickness - third_point.thickness;
  first_point.position = 2.0f * second_point.position - third_point.position;
}

template <typename StrandGroupData, typename StrandData, typename StrandSegmentData>
StrandSegmentHandle StrandGroup<StrandGroupData, StrandData, StrandSegmentData>::AllocateStrandSegment(
    StrandHandle strand_handle, StrandSegmentHandle prev_handle, const int index) {
  const StrandSegmentHandle new_segment_handle = strand_segments_.size();
  strand_segments_.emplace_back(strand_handle, prev_handle);
  strand_segments_data_list.emplace_back();
  auto& segment = strand_segments_[new_segment_handle];
  if (prev_handle != -1) {
    strand_segments_[prev_handle].next_handle_ = new_segment_handle;
    segment.prev_handle_ = prev_handle;
  }
  segment.next_handle_ = -1;
  segment.strand_handle_ = strand_handle;
  segment.index_ = index;

  version_++;
  return new_segment_handle;
}

template <typename StrandGroupData, typename StrandData, typename StrandSegmentData>
StrandHandle StrandGroup<StrandGroupData, StrandData, StrandSegmentData>::AllocateStrand() {
  const StrandHandle new_strand_handle = strands_.size();
  strands_.emplace_back();
  strands_data_list.emplace_back();
  version_++;
  return new_strand_handle;
}
template <typename StrandGroupData, typename StrandData, typename StrandSegmentData>
void StrandGroup<StrandGroupData, StrandData, StrandSegmentData>::RegulateRotations() {
  for (const auto& strand : strands_) {
    for (uint32_t i = 0; i < strand.strand_segment_handles_.size() - 1; i++) {
      const auto& prev_segment = strand_segments_[strand.strand_segment_handles_[i]];
      auto& segment = strand_segments_[strand.strand_segment_handles_[i + 1]];

      auto front = segment.rotation * glm::vec3(0, 0, -1);
      auto parent_regulated_up = prev_segment.rotation * glm::vec3(0, 1, 0);
      auto regulated_up = glm::normalize(glm::cross(glm::cross(front, parent_regulated_up), front));
      segment.rotation = glm::quatLookAt(front, regulated_up);
    }
  }
}

template <typename StrandGroupData, typename StrandData, typename StrandSegmentData>
void StrandGroup<StrandGroupData, StrandData, StrandSegmentData>::BuildStrands(std::vector<glm::uint>& strands,
                                                                               std::vector<StrandPoint>& points,
                                                                               const int node_max_count) const {
  for (const auto& strand : strands_) {
    BuildStrand(strand, strands, points, node_max_count);
  }
}

template <typename StrandGroupData, typename StrandData, typename StrandSegmentData>
void StrandGroup<StrandGroupData, StrandData, StrandSegmentData>::BuildParticles(
    std::vector<ParticleInfo>& particle_infos) const {
  particle_infos.clear();
  particle_infos.reserve(strand_segments_.size());
  for (const auto& strand : strands_) {
    for (uint32_t i = 0; i < strand.strand_segment_handles_.size(); i++) {
      const auto& strand_segment_handle = strand.strand_segment_handles_[i];
      const auto& segment = strand_segments_[strand_segment_handle];
      ParticleInfo particle_info;
      particle_info.instance_matrix.value = glm::translate(GetStrandSegmentCenter(strand_segment_handle)) *
                                            glm::mat4_cast(segment.rotation) *
                                            glm::scale(glm::vec3(segment.end_thickness, segment.end_thickness,
                                                                 GetStrandSegmentLength(strand_segment_handle)));
      particle_info.instance_color = segment.end_color;
      particle_infos.emplace_back(particle_info);
    }
  }
}

template <typename StrandGroupData, typename StrandData, typename StrandSegmentData>
StrandSegmentHandle StrandGroup<StrandGroupData, StrandData, StrandSegmentData>::Extend(
    const StrandHandle target_handle) {
  auto& strand = strands_[target_handle];
  auto prev_handle = -1;
  if (!strand.strand_segment_handles_.empty())
    prev_handle = strand.strand_segment_handles_.back();
  const auto new_segment_handle =
      AllocateStrandSegment(target_handle, prev_handle, strand.strand_segment_handles_.size());
  strand.strand_segment_handles_.emplace_back(new_segment_handle);
  version_++;
  return new_segment_handle;
}

template <typename StrandGroupData, typename StrandData, typename StrandSegmentData>
StrandSegmentHandle StrandGroup<StrandGroupData, StrandData, StrandSegmentData>::Insert(
    const StrandHandle target_handle, const StrandSegmentHandle target_segment_handle) {
  auto& strand = strands_[target_handle];
  const auto& prev_segment = strand_segments_[target_segment_handle];
  const auto prev_segment_index = prev_segment.index_;
  const auto next_segment_handle = strand.strand_segment_handles_[prev_segment_index + 1];
  if (strand.strand_segment_handles_.size() - 1 == prev_segment_index)
    return Extend(target_handle);
  const auto new_segment_handle = AllocateStrandSegment(target_handle, target_segment_handle, prev_segment_index + 1);
  auto& new_segment = strand_segments_[new_segment_handle];
  new_segment.next_handle_ = next_segment_handle;
  auto& next_segment = strand_segments_[next_segment_handle];
  next_segment.prev_handle_ = new_segment_handle;
  strand.strand_segment_handles_.insert(strand.strand_segment_handles_.begin() + prev_segment_index + 1,
                                        new_segment_handle);
  for (int i = prev_segment_index + 2; i < strand.strand_segment_handles_.size(); ++i) {
    strand_segments_[strand.strand_segment_handles_[i]].index_ = i;
  }
  version_++;
  return new_segment_handle;
}

template <typename StrandGroupData, typename StrandData, typename StrandSegmentData>
void StrandGroup<StrandGroupData, StrandData, StrandSegmentData>::RemoveSingleStrandSegment(
    StrandSegmentHandle target_segment_handle) {
  if (target_segment_handle < strand_segments_.size() - 1) {
    const auto& last_segment = strand_segments_.back();
    if (last_segment.prev_handle_ != -1) {
      strand_segments_[last_segment.prev_handle_].next_handle_ = target_segment_handle;
    }
    if (last_segment.next_handle_ != -1) {
      strand_segments_[last_segment.next_handle_].prev_handle_ = target_segment_handle;
    }

    strand_segments_[target_segment_handle] = last_segment;
    strand_segments_data_list[target_segment_handle] = strand_segments_data_list.back();
  }

  strand_segments_.pop_back();
  strand_segments_data_list.pop_back();
}

template <typename StrandGroupData, typename StrandData, typename StrandSegmentData>
void StrandGroup<StrandGroupData, StrandData, StrandSegmentData>::CutLatter(StrandSegmentHandle target_segment_handle) {
  // Recycle subsequent segments from strand.
  const auto& segment = strand_segments_[target_segment_handle];
  auto& strand = strands_[segment.strand_handle_];
  if (segment.next_handle_ == -1) {
    RemoveSingleStrandSegment(target_segment_handle);
    strand.strand_segment_handles_.pop_back();
    return;
  }
  for (auto i = strand.strand_segment_handles_.begin(); i != strand.strand_segment_handles_.end(); ++i) {
    if (*i == target_segment_handle) {
      for (auto j = i; j != strand.strand_segment_handles_.end(); ++j) {
        RemoveSingleStrandSegment(*j);
      }
      strand.strand_segment_handles_.erase(i, strand.strand_segment_handles_.end());
      break;
    }
  }
  version_++;
}

template <typename StrandGroupData, typename StrandData, typename StrandSegmentData>
void StrandGroup<StrandGroupData, StrandData, StrandSegmentData>::RemoveStrand(StrandHandle target_strand_handle) {
  // Recycle all segments;
  auto& strand = strands_[target_strand_handle];
  for (auto i = strand.strand_segment_handles_.rbegin(); i != strand.strand_segment_handles_.rend(); ++i) {
    RemoveSingleStrandSegment(*i);
  }
  if (target_strand_handle < strands_.size() - 1) {
    const auto& last_strand = strands_.back();
    strand.strand_segment_handles_ = last_strand.strand_segment_handles_;
    strand = last_strand;
    strands_data_list[target_strand_handle] = strands_data_list.back();
  }
  strands_.pop_back();
  strands_data_list.pop_back();
  version_++;
}

template <typename StrandGroupData, typename StrandData, typename StrandSegmentData>
const std::vector<Strand>& StrandGroup<StrandGroupData, StrandData, StrandSegmentData>::PeekStrands() const {
  return strands_;
}

template <typename StrandGroupData, typename StrandData, typename StrandSegmentData>
const std::vector<StrandData>& StrandGroup<StrandGroupData, StrandData, StrandSegmentData>::PeekStrandDataList() const {
  return strands_data_list;
}

template <typename StrandGroupData, typename StrandData, typename StrandSegmentData>
const std::vector<StrandSegment>& StrandGroup<StrandGroupData, StrandData, StrandSegmentData>::PeekStrandSegments()
    const {
  return strand_segments_;
}

template <typename StrandGroupData, typename StrandData, typename StrandSegmentData>
const std::vector<StrandSegmentData>&
StrandGroup<StrandGroupData, StrandData, StrandSegmentData>::PeekStrandSegmentDataList() const {
  return strand_segments_data_list;
}

template <typename StrandGroupData, typename StrandData, typename StrandSegmentData>
std::vector<Strand>& StrandGroup<StrandGroupData, StrandData, StrandSegmentData>::RefStrands() {
  return strands_;
}

template <typename StrandGroupData, typename StrandData, typename StrandSegmentData>
std::vector<StrandData>& StrandGroup<StrandGroupData, StrandData, StrandSegmentData>::RefStrandDataList() {
  return strands_data_list;
}

template <typename StrandGroupData, typename StrandData, typename StrandSegmentData>
std::vector<StrandSegment>& StrandGroup<StrandGroupData, StrandData, StrandSegmentData>::RefStrandSegments() {
  return strand_segments_;
}

template <typename StrandGroupData, typename StrandData, typename StrandSegmentData>
std::vector<StrandSegmentData>&
StrandGroup<StrandGroupData, StrandData, StrandSegmentData>::RefStrandSegmentDataList() {
  return strand_segments_data_list;
}

template <typename StrandGroupData, typename StrandData, typename StrandSegmentData>
Strand& StrandGroup<StrandGroupData, StrandData, StrandSegmentData>::RefStrand(StrandHandle handle) {
  return strands_[handle];
}

template <typename StrandGroupData, typename StrandData, typename StrandSegmentData>
StrandSegment& StrandGroup<StrandGroupData, StrandData, StrandSegmentData>::RefStrandSegment(
    const StrandSegmentHandle handle) {
  return strand_segments_[handle];
}

template <typename StrandGroupData, typename StrandData, typename StrandSegmentData>
const Strand& StrandGroup<StrandGroupData, StrandData, StrandSegmentData>::PeekStrand(const StrandHandle handle) const {
  return strands_[handle];
}

template <typename StrandGroupData, typename StrandData, typename StrandSegmentData>
const StrandSegment& StrandGroup<StrandGroupData, StrandData, StrandSegmentData>::PeekStrandSegment(
    const StrandSegmentHandle handle) const {
  return strand_segments_[handle];
}

template <typename StrandGroupData, typename StrandData, typename StrandSegmentData>
StrandData& StrandGroup<StrandGroupData, StrandData, StrandSegmentData>::RefStrandData(const StrandHandle handle) {
  return strands_data_list[handle];
}

template <typename StrandGroupData, typename StrandData, typename StrandSegmentData>
StrandSegmentData& StrandGroup<StrandGroupData, StrandData, StrandSegmentData>::RefStrandSegmentData(
    const StrandSegmentHandle handle) {
  return strand_segments_data_list[handle];
}

template <typename StrandGroupData, typename StrandData, typename StrandSegmentData>
const StrandData& StrandGroup<StrandGroupData, StrandData, StrandSegmentData>::PeekStrandData(
    const StrandHandle handle) const {
  return strands_data_list[handle];
}

template <typename StrandGroupData, typename StrandData, typename StrandSegmentData>
const StrandSegmentData& StrandGroup<StrandGroupData, StrandData, StrandSegmentData>::PeekStrandSegmentData(
    const StrandSegmentHandle handle) const {
  return strand_segments_data_list[handle];
}

template <typename StrandGroupData, typename StrandData, typename StrandSegmentData>
int StrandGroup<StrandGroupData, StrandData, StrandSegmentData>::GetVersion() const {
  return version_;
}

template <typename StrandGroupData, typename StrandData, typename StrandSegmentData>
glm::vec3 StrandGroup<StrandGroupData, StrandData, StrandSegmentData>::GetStrandSegmentStart(
    const StrandSegmentHandle handle) const {
  const auto& segment = strand_segments_[handle];
  if (segment.prev_handle_ != -1) {
    return strand_segments_[segment.prev_handle_].end_position;
  }
  return strands_[segment.strand_handle_].start_position;
}

template <typename StrandGroupData, typename StrandData, typename StrandSegmentData>
glm::vec3 StrandGroup<StrandGroupData, StrandData, StrandSegmentData>::GetStrandSegmentCenter(
    const StrandSegmentHandle handle) const {
  const auto start_position = GetStrandSegmentStart(handle);
  return (start_position + strand_segments_[handle].end_position) * 0.5f;
}

template <typename StrandGroupData, typename StrandData, typename StrandSegmentData>
float StrandGroup<StrandGroupData, StrandData, StrandSegmentData>::GetStrandSegmentLength(
    const StrandSegmentHandle handle) const {
  const auto start_position = GetStrandSegmentStart(handle);
  return glm::distance(start_position, strand_segments_[handle].end_position);
}
}  // namespace eco_sys_lab_plugin
