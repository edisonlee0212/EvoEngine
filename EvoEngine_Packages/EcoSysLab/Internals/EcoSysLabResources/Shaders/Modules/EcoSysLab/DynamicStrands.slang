public struct Strand {
  public int begin_segment_handle;
  public int end_segment_handle;

  public int begin_connection_handle;
  public int end_connection_handle;

  public int front_propagate_begin_segment_pair_handle;
  public int back_propagate_begin_segment_pair_handle;
  public int front_propagate_begin_segment_handle;
  public int back_propagate_begin_segment_handle;

  public int alternative_front_propagate_begin_segment_pair_handle;
  public int alternative_back_propagate_begin_segment_pair_handle;
  public int alternative_front_propagate_begin_segment_handle;
  public int alternative_back_propagate_begin_segment_handle;
};

public struct Node {
  public int prev_handle;
  public int padding0;
  public int padding1;
  public int padding2;
};

public struct Particle {
  public float3 x0;
  public int padding0;

  public float3 x;
  public int selected;

  public float3 last_x;
  public int highlighted;

  public float3 v;
  public int hop_distance_to_root;

  public float3 acceleration;
  public int node_handle;

  public float dmin_external;
  public float user_bound;
  public float root_distance;
  public int padding2;
};

public struct Segment {
  public int prev_handle;
  public int next_handle;
  public int strand_handle;
  public float inv_mass;
  public float4 color;

  public float4 q0;
  public float4 q;
  public float4 last_q;

  public float3 angular_v;
  public float radius;

  public float3 torque;
  public float rest_length;

  public float max_young_modulus;
  public float shear_stretch_alpha;
  public float strength;
  public float boundary_distance;

  public float2 profile_position;
  public float2 profile_polar_coordinate;

  public float3 inertia_tensor;
  public float max_shear_stretch_strain;
  public float3 inv_inertia_tensor;
  public float shear_stretch_strain_limit;

  public float4x4 inertia_w;
  public float4x4 inv_inertia_w;

  public float shear_stretch_strain;
  public int node_handle;
  public float original_mass;
  public int group_index;

  public float extra_mass;
  public float snow_amount;
  public float screen_depth;
  public int reach_ground;

  public float C;   // Chemical defense
  public float HC;  // Carbon health
  public float HL;  // Lignin health
  public float RW;  // White rot density

  public float RB;  // Brown rot density
  public float C_pre;
  public float HC_pre;
  public float HL_pre;

  public float RW_pre;
  public float RB_pre;
  public float K;  // Defense induction rate
  public float diffusion_c;

  public float diffusion_w;
  public float diffusion_b;
  public int pairs_count;
  public float moisture;

  public float moisture_pre;
  public float diffusion_m;
  public int internal_pattern;
  public int cube_pattern;

  public float3 Obstruction_w;
  public int unlink_constraint;

  public float3 Obstruction_b;
  public float ground_damping;

  public float3 Obstruction_c;
  public int quasi_stable;

  public float3 Obstruction_m;
  public float quasi_damping;

};

public struct SegmentPair {
  public int segment0_handle;
  public int segment1_handle;
  public float bend_twist_bundle_integrity;
  public float connectivity_integrity;

  public float bending_alpha;
  public float twisting_alpha;
  public float max_bending_modulus;
  public float max_torsion_modulus;

  public float4 segment0_offset;
  public float4 segment1_offset;

  public float4 rest_darboux_vector;

  public float3 bending_twist_bundle_strain;
  public float connectivity_strain;
  public float3 max_bending_twist_bundle_strain;
  public float max_connectivity_strain;
  public float3 bending_twist_bundle_strain_limit;
  public float connectivity_strain_limit;

  public int tensile_lock;
  public int compression_lock;
  public int positional_lock;
  public int rotational_lock;
};

public static const uint BUNDLE_MAX_CONNECTION = 64u;
public struct SegmentData {
  public float4 particle0_position_correction;
  public float4 particle1_position_correction;
  public float4 q_correction;
};

public struct SegmentConnectionHandles {
  public int handles[BUNDLE_MAX_CONNECTION];
};

public struct HashedGridElement {
  public uint cell_id;
  public uint segment_handle;
  public uint padding0;
  public uint padding1;
};

public struct HashedGridCellStart {
  public uint start_index;
  public uint end_index;
  public uint padding1;
  public uint padding2;
};

public struct Leaf {
  public float3 x0;
  public int segment_handle;
  public float3 x;
  public float attachment_integrity;
  public float3 last_x;
  public float rotation_integrity;
  public float4 q0;
  public float4 q;
  public float4 last_q;

  public float3 scale;
  public float inv_mass;

  public float3 position_offset;
  public float original_mass;

  public float3 v;
  public float position_strain;

  public float3 acceleration;
  public float rotation_strain;

  public float3 angular_v;
  public float position_alpha;

  public float3 torque;
  public float rotation_alpha;

  public float3 inertia_tensor;
  public float rotation_strain_limit;

  public float3 inv_inertia_tensor;
  public float position_strain_limit;

  public float4x4 inertia_w;
  public float4x4 inv_inertia_w;

  public float extra_mass;
  public float property1;
  public float health;
  public float property3;

  public int selected;
  public int highlighted;
  public int padding0;
  public int padding1;
};

public static const uint HASH_GRID_CELL_SIZE = 2u << 15u;

public uint hash_cell_coordinate(in int3 cell_coordinate) {
  uint p1 = 73856093;  // some large primes
  uint p2 = 19349663;
  uint p3 = 83492791;
  int n = int(p1 * cell_coordinate.x ^ p2 * cell_coordinate.y ^ p3 * cell_coordinate.z);
  n %= HASH_GRID_CELL_SIZE;
  return uint(n);
}
