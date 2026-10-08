public struct UniformParticle {
  public float4 position_t;
  public float4 normal_deg;
  public float4 tangent;

  public float2 profile_position;
  public float2 profile_polar_coordinate;

  public float4 override_color;

  public int segment_handle;
  public int node_index;
  public int segment_index;
  public float boundary_distance;
  public int next_particle_handle;
  public int prev_particle_handle;
  public int next_node_index;
  public int strand_index;

  public int is_single_strand_particle;  // amount of connected tetrahedrons
  public float local_extrusion_distance;
  public int is_on_surface;  // mark if particle is on the surface of the mesh to initialize the normal
  public int is_bark;

  public float4 initial_position;
  public float4 normal_q;
};

public struct DelaunayTetrahedron {
  public int indices[4];
  public int neighbor_tet_ids[4];
  public int render_neighbor[4];
  public int is_bark[4];
  public float4 color;  // for debugging
  public uint task_looked_at;
  public uint mesh_looked_at;
  public int inside;
  public int triangles_accepted;
  public float sidelengths[6];
  public int padding0;
  public int padding1;
  public int segment_pair_index[6];
  public int inside_at_init;
  public int padding2;
};

// kinDS SegmentBuilder::interiorMeshUv (profile centroid at origin).
public float2 AlphaInteriorDiskUv(in UniformParticle uniform_particle, float texture_diameter) {
  float relative_distance = 1.0 - clamp(uniform_particle.boundary_distance, 0.0, 1.0);
  float2 profile = uniform_particle.profile_position;
  float angle = atan2(-profile.y, -profile.x);
  float radial_scale = texture_diameter * relative_distance * 0.5;
  return float2(0.5 + radial_scale * cos(angle), 0.5 + radial_scale * sin(angle));
}

public float2 AlphaBarkPolarUv(in UniformParticle uniform_particle, float u_multiplier, float v_multiplier,
                               int use_polar_coordinates_for_uv) {
  if (use_polar_coordinates_for_uv == 1) {
    float tex_x = fmod(uniform_particle.profile_polar_coordinate.y * u_multiplier / 3.1415926f, 2.f);
    if (tex_x > 1.f) {
      tex_x = 2.f - tex_x;
    }
    float tex_y = float(uniform_particle.segment_index) * v_multiplier;
    return float2(tex_x, tex_y);
  }
  float tex_x = (uniform_particle.initial_position.y + uniform_particle.initial_position.z) * u_multiplier;
  float tex_y = uniform_particle.initial_position.x * v_multiplier;
  return float2(tex_x, tex_y);
}
