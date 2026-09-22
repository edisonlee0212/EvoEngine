import EcoSysLab.DynamicStrands;
import EcoSysLab.DynamicStrandsSet0;

static const uint WORKGROUP_SIZE = COMPUTE_SUBGROUP_COUNT * SUBGROUP_SIZE;
static const uint ITERATIONS_PER_TASK = ((COMPUTE_WORK_GROUP_INVOCATIONS + WORKGROUP_SIZE - 1) / WORKGROUP_SIZE);
// Push constant for simulation parameters
struct SimulationParams {
  float dt;
  float aw;
  float ab;
  float bw;
  float bb;
  float ycw;
  float ycb;
  float ylw;
  float pc;
  float pl;
  float k;
  float delta;
  float ll;
  float lc;
  uint segment_count;
  uint segment_pair_count;
  float3x3 matrixAw;
  float3x3 matrixAb;
  float3x3 matrixAc;
};
[[vk::push_constant]] ConstantBuffer<SimulationParams> params;
void computeDiffusionWeights(float3 source_pos, float3 target_pos, float source_distance, float target_distance,
                             out float dc_weight, out float dw_weight, out float db_weight) {
  float3 edge_dir = source_pos - target_pos;
  float d = length(edge_dir);

  if (d < 0.000001) {
    dc_weight = 0.0;
    dw_weight = 0.0;
    db_weight = 0.0;
    return;
  }

  edge_dir /= d;

  float3x3 matrixAw = params.matrixAw;
  float3x3 matrixAb = params.matrixAb;
  float3x3 matrixAc = params.matrixAc;

  // Calculate quadratic forms using matrix multiplication
  float kw = max(dot(mul(edge_dir, matrixAw), edge_dir), 0.0);
  float kb = max(dot(mul(edge_dir, matrixAb), edge_dir), 0.0);
  float kc = max(dot(mul(edge_dir, matrixAc), edge_dir), 0.0);

  float coef = min(source_distance, target_distance);
  coef = coef * coef * coef * 64.0;

  dw_weight = kw / d * coef;
  db_weight = kb / d * coef;
  dc_weight = kc / d;
}

[shader("compute")]
[numthreads(WORKGROUP_SIZE, 1, 1)]
void main(uint3 group_id : SV_GroupID, uint lane_id : SV_GroupIndex, uint3 dispatch_thread_id : SV_DispatchThreadID) {
  const uint baseID = group_id.x * COMPUTE_WORK_GROUP_INVOCATIONS;
  const uint laneID = lane_id;
  // PHASE 1: Calculate diffusion contributions from segment pairs
  for (uint pair_index = int(dispatch_thread_id.x); pair_index < params.segment_pair_count;
       pair_index += WorkgroupCount().x * WORKGROUP_SIZE) {
    SegmentPair segment_pair = segment_pairs[pair_index];

    // Get segment handles from the pair
    uint segment0_handle = segment_pair.segment0_handle;
    uint segment1_handle = segment_pair.segment1_handle;

    // Skip invalid handles
    if (segment0_handle < 0 || segment1_handle < 0)
      continue;
    if (segment0_handle >= int(params.segment_count) || segment1_handle >= int(params.segment_count))
      continue;

    // Get segment positions and boundary distances
    float3 segment0_pos = (segment_particle0s[segment0_handle].x + segment_particle1s[segment0_handle].x) / 2;
    float3 segment1_pos = (segment_particle0s[segment1_handle].x + segment_particle1s[segment1_handle].x) / 2;
    float segment0_distance = segments[segment0_handle].boundary_distance;
    float segment1_distance = segments[segment1_handle].boundary_distance;

    // Calculate diffusion weights
    float dc_weight, dw_weight, db_weight;
    computeDiffusionWeights(segment0_pos, segment1_pos, segment0_distance, segment1_distance, dc_weight, dw_weight,
                            db_weight);

    // Accumulate diffusion contribution from neighbors
    InterlockedAdd(segment_biology[segment0_handle].diffusion_c, dc_weight * segment_biology[segment1_handle].C_pre);
    InterlockedAdd(segment_biology[segment0_handle].diffusion_w, dw_weight * segment_biology[segment1_handle].RW_pre);
    InterlockedAdd(segment_biology[segment0_handle].diffusion_b, db_weight * segment_biology[segment1_handle].RB_pre);

    // Add reverse direction contributions (undirected graph)
    InterlockedAdd(segment_biology[segment1_handle].diffusion_c, dc_weight * segment_biology[segment0_handle].C_pre);
    InterlockedAdd(segment_biology[segment1_handle].diffusion_w, dw_weight * segment_biology[segment0_handle].RW_pre);
    InterlockedAdd(segment_biology[segment1_handle].diffusion_b, db_weight * segment_biology[segment0_handle].RB_pre);

    // Add diagonal terms (self-interactions with negative values for conservation)
    InterlockedAdd(segment_biology[segment0_handle].diffusion_c, -dc_weight * segment_biology[segment0_handle].C_pre);
    InterlockedAdd(segment_biology[segment0_handle].diffusion_w, -dw_weight * segment_biology[segment0_handle].RW_pre);
    InterlockedAdd(segment_biology[segment0_handle].diffusion_b, -db_weight * segment_biology[segment0_handle].RB_pre);

    InterlockedAdd(segment_biology[segment1_handle].diffusion_c, -dc_weight * segment_biology[segment1_handle].C_pre);
    InterlockedAdd(segment_biology[segment1_handle].diffusion_w, -dw_weight * segment_biology[segment1_handle].RW_pre);
    InterlockedAdd(segment_biology[segment1_handle].diffusion_b, -db_weight * segment_biology[segment1_handle].RB_pre);
  }

  // Ensure all diffusion calculations are complete
  GroupMemoryBarrierWithGroupSync();

  // PHASE 2: Update fungus state for each segment
  [[unroll]]
  for (uint i = 0; i < ITERATIONS_PER_TASK; i++) {
    uint segment_index = baseID + laneID + i * WORKGROUP_SIZE;
    if (segment_index >= params.segment_count)
      return;

    // Calculate K based on boundary distance
    segment_biology[segment_index].K = 1.0 / (8.0 * segments[segment_index].boundary_distance + 0.25);

    // Get diffusion values from the segment
    float diffusion_c = segment_biology[segment_index].diffusion_c;
    float diffusion_w = segment_biology[segment_index].diffusion_w;
    float diffusion_b = segment_biology[segment_index].diffusion_b;

    // Update chemical protection (C)
    segment_biology[segment_index].C = segment_biology[segment_index].C_pre +
                                params.dt * (0.001 * diffusion_c +
                                             segment_biology[segment_index].K * (1.0 - segment_biology[segment_index].C_pre) *
                                                 (segment_biology[segment_index].RW_pre + segment_biology[segment_index].RB_pre) *
                                                 segment_biology[segment_index].HC_pre -
                                             params.delta * segment_biology[segment_index].C_pre);

    // Clamp to valid range
    segment_biology[segment_index].C = clamp(segment_biology[segment_index].C, 0.0, 1.0);

    // Update health of carbon (HC)
    segment_biology[segment_index].HC =
        segment_biology[segment_index].HC_pre +
        params.dt * ((-params.ycw * segment_biology[segment_index].RW_pre - params.ycb * segment_biology[segment_index].RB_pre) *
                         segment_biology[segment_index].HC_pre +
                     params.pc * segment_biology[segment_index].C_pre * (1.0 - segment_biology[segment_index].HC_pre));

    segment_biology[segment_index].HC = clamp(segment_biology[segment_index].HC, 0.0, 1.0);

    // Update health of lignin (HL)
    segment_biology[segment_index].HL =
        segment_biology[segment_index].HL_pre +
        params.dt * ((-params.ylw * segment_biology[segment_index].RW_pre) * segment_biology[segment_index].HL_pre +
                     params.pl * segment_biology[segment_index].C_pre * (1.0 - segment_biology[segment_index].HL_pre));

    segment_biology[segment_index].HL = clamp(segment_biology[segment_index].HL, 0.0, 1.0);

    // Update white rot (RW)
    segment_biology[segment_index].RW =
        segment_biology[segment_index].RW_pre +
        params.dt * (params.aw * segment_biology[segment_index].RW_pre * (1.0 - segment_biology[segment_index].RW_pre) *
                         (params.ll * segment_biology[segment_index].HL_pre + params.lc * segment_biology[segment_index].HC_pre) -
                     params.bw * segment_biology[segment_index].C_pre * segment_biology[segment_index].RW_pre + 0.1 * diffusion_w);

    segment_biology[segment_index].RW = clamp(segment_biology[segment_index].RW, 0.0, 1.0);

    // Update brown rot (RB)
    segment_biology[segment_index].RB =
        segment_biology[segment_index].RB_pre +
        params.dt * (params.ab * segment_biology[segment_index].RB_pre * (1.0 - segment_biology[segment_index].RB_pre) *
                         segment_biology[segment_index].HC_pre -
                     params.bb * segment_biology[segment_index].C_pre * segment_biology[segment_index].RB_pre + 0.1 * diffusion_b);

    segment_biology[segment_index].RB = clamp(segment_biology[segment_index].RB, 0.0, 1.0);

    // Update previous values for next iteration
    segment_biology[segment_index].C_pre = segment_biology[segment_index].C;
    segment_biology[segment_index].HC_pre = segment_biology[segment_index].HC;
    segment_biology[segment_index].HL_pre = segment_biology[segment_index].HL;
    segment_biology[segment_index].RW_pre = segment_biology[segment_index].RW;
    segment_biology[segment_index].RB_pre = segment_biology[segment_index].RB;
    segment_biology[segment_index].diffusion_c = 0.0;
    segment_biology[segment_index].diffusion_w = 0.0;
    segment_biology[segment_index].diffusion_b = 0.0;
  }
}
