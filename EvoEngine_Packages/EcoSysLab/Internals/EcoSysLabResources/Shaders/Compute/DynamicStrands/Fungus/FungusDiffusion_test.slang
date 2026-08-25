import EcoSysLab.DynamicStrands;
import EcoSysLab.DynamicStrandsSet0;
import EcoSysLab.DynamicStrandsPhysics;
import EcoSysLab.Math;

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
  int num_nodes;
  int edge_count;
  float3x3 matrixAw;
  float3x3 matrixAb;
  float3x3 matrixAc;
};
[[vk::push_constant]] ConstantBuffer<SimulationParams> params;

// Buffers for state variables
[[vk::binding(0, 0)]] RWStructuredBuffer<float, Std430DataLayout> C;
[[vk::binding(1, 0)]] RWStructuredBuffer<float, Std430DataLayout> HC;

[[vk::binding(2, 0)]] RWStructuredBuffer<float, Std430DataLayout> HL;
[[vk::binding(3, 0)]] RWStructuredBuffer<float, Std430DataLayout> RW;
[[vk::binding(4, 0)]] RWStructuredBuffer<float, Std430DataLayout> RB;
[[vk::binding(5, 0)]] RWStructuredBuffer<float, Std430DataLayout> K;

// Temp buffers to store previous state
[[vk::binding(6, 0)]] RWStructuredBuffer<float, Std430DataLayout> C_pre;
[[vk::binding(7, 0)]] RWStructuredBuffer<float, Std430DataLayout> HC_pre;
[[vk::binding(8, 0)]] RWStructuredBuffer<float, Std430DataLayout> HL_pre;
[[vk::binding(9, 0)]] RWStructuredBuffer<float, Std430DataLayout> RW_pre;
[[vk::binding(10, 0)]] RWStructuredBuffer<float, Std430DataLayout> RB_pre;

// Edge data structure
struct Edge {
  int source;
  int target;
};
[[vk::binding(11, 0)]] RWStructuredBuffer<Edge, Std430DataLayout> edges;

// Buffers for calculated diffusion values (new)
[[vk::binding(12, 0)]] RWStructuredBuffer<float, Std430DataLayout> diffusion_c_values;
[[vk::binding(13, 0)]] RWStructuredBuffer<float, Std430DataLayout> diffusion_w_values;
[[vk::binding(14, 0)]] RWStructuredBuffer<float, Std430DataLayout> diffusion_b_values;

[[vk::binding(15, 0)]] RWStructuredBuffer<float3, Std430DataLayout> node_positions;
[[vk::binding(16, 0)]] RWStructuredBuffer<float, Std430DataLayout> node_distance_to_boundary;
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
  // float ex = edge_dir.x;
  // float ey = edge_dir.y;
  // float ez = edge_dir.z;

  // float kw = max(1.0 * ex * ex + 0.5 * ey * ey + 1.0 * ez * ez, 0.0);
  // float kb = kw;
  // float kc = 0.5 * ex * ex + ey * ey + 0.5 * ez * ez;

  float3x3 matrixAw = params.matrixAw;
  float3x3 matrixAb = params.matrixAb;
  float3x3 matrixAc = params.matrixAc;

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
  // PHASE 1: Precompute diffusion contributions
  // Process edges in parallel - each thread handles multiple edges
  for (uint edge_index = int(dispatch_thread_id.x); edge_index < params.edge_count;
       edge_index += WorkgroupCount().x * WORKGROUP_SIZE) {
    Edge edge = edges[edge_index];
    uint i = edge.source;
    uint j = edge.target;

    float3 source_pos = node_positions[i];
    float3 target_pos = node_positions[j];

    float source_distance = node_distance_to_boundary[i];
    float target_distance = node_distance_to_boundary[j];

    float dc_weight, dw_weight, db_weight;
    computeDiffusionWeights(source_pos, target_pos, source_distance, target_distance, dc_weight, dw_weight, db_weight);

    // Accumulate diffusion contribution from neighbors
    InterlockedAdd(diffusion_c_values[i], dc_weight * C_pre[j]);
    InterlockedAdd(diffusion_w_values[i], dw_weight * RW_pre[j]);
    InterlockedAdd(diffusion_b_values[i], db_weight * RB_pre[j]);

    // Add reverse direction contributions (undirected graph)
    InterlockedAdd(diffusion_c_values[j], dc_weight * C_pre[i]);
    InterlockedAdd(diffusion_w_values[j], dw_weight * RW_pre[i]);
    InterlockedAdd(diffusion_b_values[j], db_weight * RB_pre[i]);

    // Add diagonal terms (self-interactions with negative values for conservation)
    InterlockedAdd(diffusion_c_values[i], -dc_weight * C_pre[i]);
    InterlockedAdd(diffusion_w_values[i], -dw_weight * RW_pre[i]);
    InterlockedAdd(diffusion_b_values[i], -db_weight * RB_pre[i]);

    InterlockedAdd(diffusion_c_values[j], -dc_weight * C_pre[j]);
    InterlockedAdd(diffusion_w_values[j], -dw_weight * RW_pre[j]);
    InterlockedAdd(diffusion_b_values[j], -db_weight * RB_pre[j]);
  }

  // Ensure all diffusion calculations are complete
  GroupMemoryBarrierWithGroupSync();

  // PHASE 2: Process each node with precalculated diffusion values
  [[unroll]]
  for (uint i = 0; i < ITERATIONS_PER_TASK; i++) {
    uint local_index = laneID + i * WORKGROUP_SIZE;
    uint node_index = baseID + local_index;
    if (node_index >= params.num_nodes)
      return;

    float diffusion_c = diffusion_c_values[node_index];
    float diffusion_w = diffusion_w_values[node_index];
    float diffusion_b = diffusion_b_values[node_index];
    K[node_index] = 1.0 / (8.0 * node_distance_to_boundary[node_index] + 0.25);

    // Update chemical protection
    C[node_index] = C_pre[node_index] + params.dt * (0.001 * diffusion_c +
                                                               K[node_index] * (1.0 - C_pre[node_index]) *
                                                                   (RW_pre[node_index] + RB_pre[node_index]) *
                                                                   HC_pre[node_index] -
                                                               params.delta * C_pre[node_index]);

    // Clamp to valid range
    C[node_index] = clamp(C[node_index], 0.0, 1.0);

    // Update health of carbon
    HC[node_index] = HC_pre[node_index] +
                          params.dt * ((-params.ycw * RW_pre[node_index] - params.ycb * RB_pre[node_index]) *
                                           HC_pre[node_index] +
                                       params.pc * C_pre[node_index] * (1.0 - HC_pre[node_index]));

    HC[node_index] = clamp(HC[node_index], 0.0, 1.0);

    // Update health of linin
    HL[node_index] =
        HL_pre[node_index] + params.dt * ((-params.ylw * RW_pre[node_index]) * HL_pre[node_index] +
                                               params.pl * C_pre[node_index] * (1.0 - HL_pre[node_index]));

    HL[node_index] = clamp(HL[node_index], 0.0, 1.0);

    // Update white rot
    RW[node_index] =
        RW_pre[node_index] +
        params.dt * (params.aw * RW_pre[node_index] * (1.0 - RW_pre[node_index]) *
                         (params.ll * HL_pre[node_index] + params.lc * HC_pre[node_index]) -
                     params.bw * C_pre[node_index] * RW_pre[node_index] + 0.1 * diffusion_w);

    RW[node_index] = clamp(RW[node_index], 0.0, 1.0);

    // Update brown rot
    RB[node_index] =
        RB_pre[node_index] +
        params.dt * (params.ab * RB_pre[node_index] * (1.0 - RB_pre[node_index]) * HC_pre[node_index] -
                     params.bb * C_pre[node_index] * RB_pre[node_index] + 0.1 * diffusion_b);

    RB[node_index] = clamp(RB[node_index], 0.0, 1.0);
    C_pre[node_index] = C[node_index];
    HC_pre[node_index] = HC[node_index];
    HL_pre[node_index] = HL[node_index];
    RW_pre[node_index] = RW[node_index];
    RB_pre[node_index] = RB[node_index];
    diffusion_c_values[node_index] = 0.0;
    diffusion_w_values[node_index] = 0.0;
    diffusion_b_values[node_index] = 0.0;
  }
}
