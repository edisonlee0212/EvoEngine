import EcoSysLab.DynamicStrands;
import EcoSysLab.DynamicStrandsSet0;
import EcoSysLab.Math;

static const uint WORKGROUP_SIZE = COMPUTE_SUBGROUP_COUNT * SUBGROUP_SIZE;

static const uint ITERATIONS_PER_TASK = ((COMPUTE_WORK_GROUP_INVOCATIONS + WORKGROUP_SIZE - 1) / WORKGROUP_SIZE);
[[vk::push_constant]] cbuffer PUSH_CONSTANTS {
  float4 box_bound;
  float4x4 projection_view;
  uint selection_mode;
  uint leaf_size;
};

[shader("compute")]
[numthreads(WORKGROUP_SIZE, 1, 1)]
void main(uint3 group_id : SV_GroupID, uint lane_id : SV_GroupIndex, uint3 dispatch_thread_id : SV_DispatchThreadID) {
  const uint baseID = group_id.x * COMPUTE_WORK_GROUP_INVOCATIONS;
  const uint laneID = lane_id;
  [[unroll]]
  for (uint i = 0; i < ITERATIONS_PER_TASK; i++) {
    uint local_index = laneID + i * WORKGROUP_SIZE;
    uint leaf_index = baseID + local_index;

    if (leaf_index >= leaf_size)
      break;

    Leaf leaf = foliage[leaf_index];

    // Reset acceration
    float4 screen_space_coord = mul(projection_view, float4(leaf.x, 1.0));
    float3 screen_space_position = screen_space_coord.xyz / screen_space_coord.w;
    float screen_x = (screen_space_position.x + 1.0) * 0.5;
    float screen_y = (-screen_space_position.y + 1.0) * 0.5;
    bool in_bound =
        screen_x > box_bound.x && screen_y > box_bound.y && screen_x < box_bound.z && screen_y < box_bound.w;
    leaf.highlighted = 0;
    if (in_bound) {
      if (selection_mode == 0) {  // Drawing select box
        leaf.highlighted = 1;
      }
      if (selection_mode == 1 && leaf.selected == 1) {  // Drawing deselect box
        leaf.highlighted = 2;
      }
    }
    if (selection_mode == 2 && in_bound) {  // Select
      leaf.selected = 1;
    }
    if (selection_mode == 3 && in_bound) {  // Deselect
      leaf.selected = 0;
    }

    if (selection_mode == 4) {  // Clear
      leaf.selected = 0;
    }

    foliage[leaf_index].highlighted = leaf.highlighted;
    foliage[leaf_index].selected = leaf.selected;
  }
}