#pragma once

#include "Vertex.hpp"

using namespace evo_engine;
namespace eco_sys_lab {

typedef int StrandHandle;
typedef int StrandSegmentHandle;

struct StrandSegmentInfo {
  /**
   * \brief The position of the end of current strand segment.
   */
  glm::vec3 global_position = glm::vec3(0.0f);
  /**
   * \brief The thickness of the end of current strand segment.
   */
  float thickness = 0.0f;
  glm::vec4 color = glm::vec4(1.0f);
  bool is_boundary = false;
};

struct StrandInfo {
  glm::vec4 color = glm::vec4(1.0f);

  /**
   * \brief The info of the start of the first strand segment in this strand.
   */
  StrandSegmentInfo base_info{};
};

/**
 * \brief The data structure that holds a strand segment.
 * \tparam StrandSegmentData The customizable data for each strand segment.
 */
template <typename StrandSegmentData>
class StrandSegment {
  template <typename Pgd, typename Pd, typename Psd>
  friend class StrandGroup;
  template <typename Sgd, typename Sd, typename Ssd>
  friend class StrandGroupSerializer;
  bool end_segment_ = true;
  bool recycled_ = false;
  StrandSegmentHandle prev_handle_ = -1;
  StrandSegmentHandle handle_ = -1;
  StrandSegmentHandle next_handle_ = -1;

  StrandHandle strand_handle_ = -1;

  int index_ = -1;

 public:
  StrandSegmentData data{};
  StrandSegmentInfo info{};

  /**
   * Whether this segment is the end segment.
   * @return True if this is end segment, false else wise.
   */
  [[nodiscard]] bool IsEnd() const;

  /**
   * Whether this segment is recycled (removed).
   * @return True if this segment is recycled (removed), false else wise.
   */
  [[nodiscard]] bool IsRecycled() const;

  /**
   * Get the handle of self.
   * @return strandSegmentHandle of current segment.
   */
  [[nodiscard]] StrandSegmentHandle GetHandle() const;

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

  [[nodiscard]] int GetIndex() const;
  StrandSegment() = default;
  explicit StrandSegment(StrandHandle strand_handle, StrandSegmentHandle handle, StrandSegmentHandle prev_handle);
};

/**
 * \brief The data structure that holds a strand.
 * \tparam StrandData The customizable data for each strand.
 */
template <typename StrandData>
class Strand {
  template <typename Pgd, typename Pd, typename Psd>
  friend class StrandGroup;
  template <typename Sgd, typename Sd, typename Ssd>
  friend class StrandGroupSerializer;
  bool recycled_ = false;
  StrandHandle handle_ = -1;

  std::vector<StrandSegmentHandle> strand_segment_handles_;

 public:
  StrandData data;
  StrandInfo info;

  /**
   * Whether this segment is recycled (removed).
   * @return True if this segment is recycled (removed), false else wise.
   */
  [[nodiscard]] bool IsRecycled() const;

  /**
   * Get the handle of self.
   * @return strandSegmentHandle of current segment.
   */
  [[nodiscard]] StrandHandle GetHandle() const;

  /**
   * Access the segments that belongs to this flow.
   * @return The list of handles.
   */
  [[nodiscard]] const std::vector<StrandSegmentHandle>& PeekStrandSegmentHandles() const;
  Strand() = default;
  explicit Strand(StrandHandle handle);
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
  std::vector<Strand<StrandData>> strands_;
  std::vector<StrandSegment<StrandSegmentData>> strand_segments_;

  std::queue<StrandHandle> strand_pool_;
  std::queue<StrandSegmentHandle> strand_segment_pool_;

  int version_ = -1;
  void BuildStrand(const Strand<StrandData>& strand, std::vector<glm::uint>& strands, std::vector<StrandPoint>& points,
                   int node_max_count) const;

  [[nodiscard]] StrandSegmentHandle AllocateStrandSegment(StrandHandle strand_handle, StrandSegmentHandle prev_handle,
                                                          int index);

 public:
  void BuildStrands(std::vector<glm::uint>& strands, std::vector<StrandPoint>& points, int node_max_count) const;

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
   * Recycle (Remove) a segment, the descendents of this segment will also be recycled. The relevant flow will also be
   * removed/restructured.
   * @param handle The handle of the segment to be removed. Must be valid (non-zero and the segment should not be
   * recycled prior to this operation).
   */
  void RecycleStrandSegment(StrandSegmentHandle handle);

  /**
   * Recycle (Remove) a strand. The relevant segment will also be removed/restructured.
   * @param handle The handle of the strand to be removed. Must be valid (non-zero and the flow should not be recycled
   * prior to this operation).
   */
  void RecycleStrand(StrandHandle handle);

  /**
   * \brief Get a unmodifiable reference to all strands.
   * \return A constant reference to all strands.
   */
  [[nodiscard]] const std::vector<Strand<StrandData>>& PeekStrands() const;
  /**
   * \brief Get a unmodifiable reference to all strand segments.
   * \return A constant reference to all strand segments.
   */
  [[nodiscard]] const std::vector<StrandSegment<StrandSegmentData>>& PeekStrandSegments() const;
  /**
   * \brief Get a reference to all strands.
   * \return A reference to all strands.
   */
  [[nodiscard]] std::vector<Strand<StrandData>>& RefStrands();
  /**
   * \brief Get a reference to all strand segments.
   * \return A reference to all strand segments.
   */
  [[nodiscard]] std::vector<StrandSegment<StrandSegmentData>>& RefStrandSegments();
  /**
   * \brief Get a reference to a specific strand.
   * \param handle The handle of the strand.
   * \return A reference to the target strand.
   */
  [[nodiscard]] Strand<StrandData>& RefStrand(StrandHandle handle);
  /**
   * \brief Get a reference to a specific strand segment.
   * \param handle The handle of the strand segment.
   * \return A reference to the target strand segment.
   */
  [[nodiscard]] StrandSegment<StrandSegmentData>& RefStrandSegment(StrandSegmentHandle handle);
  /**
   * \brief Get a unmodifiable reference to a specific strand.
   * \param handle The handle of the strand.
   * \return A unmodifiable reference to the target strand.
   */
  [[nodiscard]] const Strand<StrandData>& PeekStrand(StrandHandle handle) const;
  /**
   * \brief Get a unmodifiable reference to a specific strand.
   * \param handle The handle of the strand.
   * \return A unmodifiable reference to the target strand.
   */
  [[nodiscard]] const StrandSegment<StrandSegmentData>& PeekStrandSegment(StrandSegmentHandle handle) const;

  /**
   * Get the structural version of the tree. The version will change when the tree structure changes.
   * @return The version
   */
  [[nodiscard]] int GetVersion() const;

  [[nodiscard]] glm::vec3 GetStrandSegmentStart(StrandSegmentHandle handle) const;
};

template <typename StrandGroupData, typename StrandData, typename StrandSegmentData>
void StrandGroup<StrandGroupData, StrandData, StrandSegmentData>::BuildStrand(const Strand<StrandData>& strand,
                                                                              std::vector<glm::uint>& strands,
                                                                              std::vector<StrandPoint>& points,
                                                                              int node_max_count) const {
  const auto& strand_segment_handles = strand.PeekStrandSegmentHandles();
  if (strand_segment_handles.empty())
    return;

  auto& base_info = strand.info.base_info;
  const auto start_index = points.size();
  strands.emplace_back(start_index);
  StrandPoint base_point;
  base_point.color = base_info.color;
  base_point.thickness = base_info.thickness;
  base_point.position = base_info.global_position;

  points.emplace_back(base_point);
  points.emplace_back(base_point);

  StrandPoint point;
  for (int i = 0; i < strand_segment_handles.size() && (node_max_count == -1 || i < node_max_count); i++) {
    const auto& strand_segment = PeekStrandSegment(strand_segment_handles[i]);
    // auto distance = glm::min(prevDistance, nextDistance);
    point.color = strand_segment.info.color;
    point.thickness = strand_segment.info.thickness;
    point.position = strand_segment.info.global_position;
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
  StrandSegmentHandle new_segment_handle;
  if (strand_segment_pool_.empty()) {
    strand_segments_.emplace_back(strand_handle, strand_segments_.size(), prev_handle);
    new_segment_handle = strand_segments_.back().handle_;
  } else {
    new_segment_handle = strand_segment_pool_.front();
    strand_segment_pool_.pop();
  }
  auto& segment = strand_segments_[new_segment_handle];
  if (prev_handle != -1) {
    strand_segments_[prev_handle].next_handle_ = new_segment_handle;
    strand_segments_[prev_handle].end_segment_ = false;
    segment.prev_handle_ = prev_handle;
  }
  segment.next_handle_ = -1;
  segment.strand_handle_ = strand_handle;
  segment.index_ = index;
  segment.recycled_ = false;
  return new_segment_handle;
}

template <typename StrandGroupData, typename StrandData, typename StrandSegmentData>
void StrandGroup<StrandGroupData, StrandData, StrandSegmentData>::BuildStrands(std::vector<glm::uint>& strands,
                                                                               std::vector<StrandPoint>& points,
                                                                               int node_max_count) const {
  for (const auto& strand : strands_) {
    if (strand.IsRecycled())
      continue;
    BuildStrand(strand, strands, points, node_max_count);
  }
}

template <typename StrandGroupData, typename StrandData, typename StrandSegmentData>
StrandHandle StrandGroup<StrandGroupData, StrandData, StrandSegmentData>::AllocateStrand() {
  if (strand_pool_.empty()) {
    strands_.emplace_back(strands_.size());
    version_++;
    return strands_.back().handle_;
  }
  auto handle = strand_pool_.front();
  strand_pool_.pop();
  auto& strand = strands_[handle];
  strand.recycled_ = false;
  version_++;
  return handle;
}

template <typename StrandGroupData, typename StrandData, typename StrandSegmentData>
StrandSegmentHandle StrandGroup<StrandGroupData, StrandData, StrandSegmentData>::Extend(StrandHandle target_handle) {
  auto& strand = strands_[target_handle];
  assert(!strand.recycled_);
  auto prev_handle = -1;
  if (!strand.strand_segment_handles_.empty())
    prev_handle = strand.strand_segment_handles_.back();
  const auto new_segment_handle =
      AllocateStrandSegment(target_handle, prev_handle, strand.strand_segment_handles_.size());
  strand.strand_segment_handles_.emplace_back(new_segment_handle);
  auto& segment = strand_segments_[new_segment_handle];
  segment.end_segment_ = true;
  version_++;
  return new_segment_handle;
}

template <typename StrandGroupData, typename StrandData, typename StrandSegmentData>
StrandSegmentHandle StrandGroup<StrandGroupData, StrandData, StrandSegmentData>::Insert(
    StrandHandle target_handle, StrandSegmentHandle target_segment_handle) {
  auto& strand = strands_[target_handle];
  assert(!strand.recycled_);
  auto& prev_segment = strand_segments_[target_segment_handle];
  const auto prev_segment_index = prev_segment.index_;
  const auto next_segment_handle = strand.strand_segment_handles_[prev_segment_index + 1];
  if (strand.strand_segment_handles_.size() - 1 == prev_segment_index)
    return Extend(target_handle);
  const auto new_segment_handle = AllocateStrandSegment(target_handle, target_segment_handle, prev_segment_index + 1);
  auto& new_segment = strand_segments_[new_segment_handle];
  new_segment.end_segment_ = false;
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
void StrandGroup<StrandGroupData, StrandData, StrandSegmentData>::RecycleStrandSegment(StrandSegmentHandle handle) {
  // Recycle subsequent segments from strand.
  auto& segment = strand_segments_[handle];
  assert(!segment.recycled_);
  if (segment.next_handle_ != -1) {
    RecycleStrandSegment(segment.next_handle_);
  }
  if (segment.prev_handle_ != -1) {
    strand_segments_[segment.prev_handle_].next_handle_ = -1;
    strand_segments_[segment.prev_handle_].end_segment_ = true;
  }

  auto& strand = strands_[segment.strand_handle_];
  assert(strand.m_strandSegmentHandles.back() == handle);
  strand.strand_segment_handles_.pop_back();

  segment.recycled_ = true;
  segment.end_segment_ = true;
  segment.prev_handle_ = segment.next_handle_ = -1;
  segment.data = {};
  segment.info = {};

  segment.index_ = -1;
  segment.strand_handle_ = -1;
  strand_segment_pool_.emplace(handle);
  version_++;
}

template <typename StrandGroupData, typename StrandData, typename StrandSegmentData>
void StrandGroup<StrandGroupData, StrandData, StrandSegmentData>::RecycleStrand(StrandHandle handle) {
  // Recycle all segments;
  auto& strand = strands_[handle];
  assert(!strand.recycled_);
  for (const auto& segment_handle : strand.strand_segment_handles_) {
    auto& segment = strand_segments_[segment_handle];
    segment.recycled_ = true;
    segment.end_segment_ = true;
    segment.prev_handle_ = segment.next_handle_ = -1;
    segment.data = {};
    segment.info = {};

    segment.index_ = -1;
    segment.strand_handle_ = -1;
    strand_segment_pool_.emplace(segment_handle);
  }
  strand.strand_segment_handles_.clear();

  // Recycle strand.
  strand.recycled_ = true;
  strand.data = {};
  strand.info = {};
  strand_pool_.emplace(handle);
  version_++;
}

template <typename StrandGroupData, typename StrandData, typename StrandSegmentData>
const std::vector<Strand<StrandData>>& StrandGroup<StrandGroupData, StrandData, StrandSegmentData>::PeekStrands()
    const {
  return strands_;
}

template <typename StrandGroupData, typename StrandData, typename StrandSegmentData>
const std::vector<StrandSegment<StrandSegmentData>>&
StrandGroup<StrandGroupData, StrandData, StrandSegmentData>::PeekStrandSegments() const {
  return strand_segments_;
}

template <typename StrandGroupData, typename StrandData, typename StrandSegmentData>
std::vector<Strand<StrandData>>& StrandGroup<StrandGroupData, StrandData, StrandSegmentData>::RefStrands() {
  return strands_;
}

template <typename StrandGroupData, typename StrandData, typename StrandSegmentData>
std::vector<StrandSegment<StrandSegmentData>>&
StrandGroup<StrandGroupData, StrandData, StrandSegmentData>::RefStrandSegments() {
  return strand_segments_;
}

template <typename StrandGroupData, typename StrandData, typename StrandSegmentData>
Strand<StrandData>& StrandGroup<StrandGroupData, StrandData, StrandSegmentData>::RefStrand(StrandHandle handle) {
  return strands_[handle];
}

template <typename StrandGroupData, typename StrandData, typename StrandSegmentData>
StrandSegment<StrandSegmentData>& StrandGroup<StrandGroupData, StrandData, StrandSegmentData>::RefStrandSegment(
    StrandSegmentHandle handle) {
  return strand_segments_[handle];
}

template <typename StrandGroupData, typename StrandData, typename StrandSegmentData>
const Strand<StrandData>& StrandGroup<StrandGroupData, StrandData, StrandSegmentData>::PeekStrand(
    StrandHandle handle) const {
  return strands_[handle];
}

template <typename StrandGroupData, typename StrandData, typename StrandSegmentData>
const StrandSegment<StrandSegmentData>& StrandGroup<StrandGroupData, StrandData, StrandSegmentData>::PeekStrandSegment(
    StrandSegmentHandle handle) const {
  return strand_segments_[handle];
}

template <typename StrandGroupData, typename StrandData, typename StrandSegmentData>
int StrandGroup<StrandGroupData, StrandData, StrandSegmentData>::GetVersion() const {
  return version_;
}

template <typename StrandGroupData, typename StrandData, typename StrandSegmentData>
glm::vec3 StrandGroup<StrandGroupData, StrandData, StrandSegmentData>::GetStrandSegmentStart(
    StrandSegmentHandle handle) const {
  const auto& segment = strand_segments_[handle];
  glm::vec3 segment_start;
  if (segment.GetPrevHandle() != -1) {
    segment_start = strand_segments_[segment.GetPrevHandle()].info.global_position;
  } else {
    segment_start = strands_[segment.GetStrandHandle()].info.base_info.global_position;
  }
  return segment_start;
}

template <typename StrandSegmentData>
bool StrandSegment<StrandSegmentData>::IsEnd() const {
  return end_segment_;
}

template <typename StrandSegmentData>
bool StrandSegment<StrandSegmentData>::IsRecycled() const {
  return recycled_;
}

template <typename StrandSegmentData>
StrandSegmentHandle StrandSegment<StrandSegmentData>::GetHandle() const {
  return handle_;
}

template <typename StrandSegmentData>
StrandHandle StrandSegment<StrandSegmentData>::GetStrandHandle() const {
  return strand_handle_;
}

template <typename StrandSegmentData>
StrandSegmentHandle StrandSegment<StrandSegmentData>::GetPrevHandle() const {
  return prev_handle_;
}

template <typename StrandSegmentData>
StrandSegmentHandle StrandSegment<StrandSegmentData>::GetNextHandle() const {
  return next_handle_;
}

template <typename StrandSegmentData>
int StrandSegment<StrandSegmentData>::GetIndex() const {
  return index_;
}

template <typename StrandSegmentData>
StrandSegment<StrandSegmentData>::StrandSegment(const StrandHandle strand_handle, const StrandSegmentHandle handle,
                                                const StrandSegmentHandle prev_handle) {
  strand_handle_ = strand_handle;
  handle_ = handle;
  prev_handle_ = prev_handle;
  next_handle_ = -1;
  recycled_ = false;
  end_segment_ = true;

  index_ = -1;
  data = {};
  info = {};
}

template <typename StrandData>
bool Strand<StrandData>::IsRecycled() const {
  return recycled_;
}

template <typename StrandData>
StrandHandle Strand<StrandData>::GetHandle() const {
  return handle_;
}

template <typename StrandData>
const std::vector<StrandSegmentHandle>& Strand<StrandData>::PeekStrandSegmentHandles() const {
  return strand_segment_handles_;
}

template <typename StrandData>
Strand<StrandData>::Strand(const StrandHandle handle) {
  handle_ = handle;
  recycled_ = false;

  strand_segment_handles_.clear();

  data = {};
  info = {};
}
}  // namespace eco_sys_lab
