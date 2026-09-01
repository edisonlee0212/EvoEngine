import EcoSysLab.DynamicStrands;
import EcoSysLab.DynamicStrandsSet0;

struct SliceMember {
  uint base_slice;
  int group;
  uint segment;
  uint dynamic_slice;
};

[[vk::binding(2, 1)]] StructuredBuffer<uint, Std430DataLayout> base_slices;
[[vk::binding(3, 1)]] RWStructuredBuffer<SliceMember, Std430DataLayout> slice_members;

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

static const uint WORKGROUP_SIZE = COMPUTE_SUBGROUP_COUNT * SUBGROUP_SIZE;

[shader("compute")]
[numthreads(WORKGROUP_SIZE, 1, 1)]
void main(uint index : SV_DispatchThreadID) {
  SliceMember member;
  member.dynamic_slice = 0;
  if (index < segment_count) {
    member.base_slice = base_slices[index];
    member.group = segments[index].group_index;
    member.segment = index;
  } else {
    member.base_slice = 0xffffffffu;
    member.group = 0x7fffffff;
    member.segment = 0xffffffffu;
  }
  if (index < padded_count)
    slice_members[index] = member;
}
