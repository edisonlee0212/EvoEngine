struct SliceMember {
  uint base_slice;
  int group;
  uint segment;
  uint dynamic_slice;
};

struct SliceRange {
  uint begin;
  uint count;
  uint base_slice;
  int group;
};

struct DispatchCommand {
  uint x;
  uint y;
  uint z;
};

[[vk::binding(3, 1)]] RWStructuredBuffer<SliceMember, Std430DataLayout> slice_members;
[[vk::binding(4, 1)]] RWStructuredBuffer<SliceRange, Std430DataLayout> slice_ranges;
[[vk::binding(5, 1)]] RWStructuredBuffer<uint, Std430DataLayout> segment_slices;
[[vk::binding(7, 1)]] RWStructuredBuffer<uint, Std430DataLayout> slice_count;
[[vk::binding(8, 1)]] RWStructuredBuffer<DispatchCommand, Std430DataLayout> dispatch_commands;

[[vk::push_constant]] cbuffer PushConstants {
  uint segment_count;
  uint padded_count;
  uint minimum_members;
  uint sort_stage;
  uint sort_pass;
  float shape_matching_strength;
  uint padding0;
  uint padding1;
};

[shader("compute")]
[numthreads(1, 1, 1)]
void main() {
  uint range_count = 0;
  uint begin = 0;
  while (begin < segment_count) {
    SliceMember first = slice_members[begin];
    uint end = begin + 1;
    while (end < segment_count) {
      SliceMember next = slice_members[end];
      if (next.base_slice != first.base_slice || next.group != first.group)
        break;
      ++end;
    }
    SliceRange range;
    range.begin = begin;
    range.count = end - begin;
    range.base_slice = first.base_slice;
    range.group = first.group;
    slice_ranges[range_count] = range;
    for (uint member_index = begin; member_index < end; ++member_index) {
      SliceMember member = slice_members[member_index];
      member.dynamic_slice = range_count;
      slice_members[member_index] = member;
      segment_slices[member.segment] = range_count;
    }
    ++range_count;
    begin = end;
  }
  slice_count[0] = range_count;
  DispatchCommand command;
  command.x = (range_count + COMPUTE_WORK_GROUP_INVOCATIONS - 1) / COMPUTE_WORK_GROUP_INVOCATIONS;
  command.y = 1;
  command.z = 1;
  dispatch_commands[0] = command;
}
