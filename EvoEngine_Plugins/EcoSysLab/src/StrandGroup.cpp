#include "StrandGroup.hpp"

using namespace eco_sys_lab_plugin;

bool StrandSegment::IsEnd() const {
  return next_handle_ == -1;
}

StrandHandle StrandSegment::GetStrandHandle() const {
  return strand_handle_;
}

StrandSegmentHandle StrandSegment::GetPrevHandle() const {
  return prev_handle_;
}

StrandSegmentHandle StrandSegment::GetNextHandle() const {
  return next_handle_;
}

int StrandSegment::GetIndex() const {
  return index_;
}

StrandSegment::StrandSegment(const StrandHandle strand_handle, const StrandSegmentHandle prev_handle) {
  strand_handle_ = strand_handle;
  prev_handle_ = prev_handle;
  next_handle_ = -1;

  index_ = -1;
}

const std::vector<StrandSegmentHandle>& Strand::PeekStrandSegmentHandles() const {
  return strand_segment_handles_;
}