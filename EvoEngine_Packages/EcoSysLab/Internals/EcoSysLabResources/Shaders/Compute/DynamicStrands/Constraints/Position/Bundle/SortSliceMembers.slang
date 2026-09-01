struct SliceMember {
  uint base_slice;
  int group;
  uint segment;
  uint dynamic_slice;
};

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

bool greater_than(SliceMember a, SliceMember b) {
  return a.base_slice > b.base_slice ||
         (a.base_slice == b.base_slice && (a.group > b.group || (a.group == b.group && a.segment > b.segment)));
}

[shader("compute")]
[numthreads(WORKGROUP_SIZE, 1, 1)]
void main(uint index : SV_DispatchThreadID) {
  uint partner = index ^ sort_pass;
  if (index >= padded_count || partner <= index)
    return;
  bool ascending = (index & sort_stage) == 0;
  SliceMember a = slice_members[index];
  SliceMember b = slice_members[partner];
  if (greater_than(a, b) == ascending) {
    slice_members[index] = b;
    slice_members[partner] = a;
  }
}
