struct TreePhysicsVertex {
  float4 position_inverse_mass;
  float4 velocity_radius;
  float4 rest_local;
  uint4 incident_rods;
};

struct TreePhysicsRod {
  uint4 links;
  float4 length_scale_mass_radius;
  float4 rotation;
  float4 rest_rotation;
  uint4 incident_angles;
};

struct TreePhysicsAngle {
  uint4 rods;
  float4 rest_relative;
  float4 bend_scale;
};

struct TreePhysicsTreeParameters {
  float4 mechanics;
  float4 gravity;
  uint4 control;
  float4x4 previous_model;
  float4x4 current_model;
  float4x4 model_delta;
  float4 anchor_previous;
  float4 anchor_current;
  float4 rotation_delta;
  float4 anchor_offset;
};

struct WindParticle {
  float4 position_age;
  float4 velocity_lifetime;
  float4 collision;
};

[[vk::binding(0, 0)]] StructuredBuffer<TreePhysicsVertex, Std430DataLayout> vertices_in;
[[vk::binding(1, 0)]] RWStructuredBuffer<TreePhysicsVertex, Std430DataLayout> vertices_out;
[[vk::binding(2, 0)]] StructuredBuffer<TreePhysicsRod, Std430DataLayout> rods_in;
[[vk::binding(3, 0)]] RWStructuredBuffer<TreePhysicsRod, Std430DataLayout> rods_out;
[[vk::binding(4, 0)]] StructuredBuffer<TreePhysicsAngle, Std430DataLayout> angles;
[[vk::binding(5, 0)]] StructuredBuffer<uint> vertex_rods;
[[vk::binding(6, 0)]] StructuredBuffer<uint> rod_angles;
[[vk::binding(7, 0)]] RWStructuredBuffer<float4> predicted;
[[vk::binding(8, 0)]] RWStructuredBuffer<float4> previous;
[[vk::binding(9, 0)]] StructuredBuffer<TreePhysicsTreeParameters, Std430DataLayout> tree_parameters;
[[vk::binding(0, 1)]] StructuredBuffer<WindParticle> wind_particles;
[[vk::binding(1, 1)]] StructuredBuffer<uint> wind_heads;
[[vk::binding(2, 1)]] StructuredBuffer<uint> wind_next;

[[vk::push_constant]] cbuffer TreePhysicsParameters {
  uint vertex_count;
  uint rod_count;
  uint substep;
  uint iteration;
  uint particle_count;
  uint bucket_count;
  float cell_size;
  float max_particle_radius;
};

uint HashWindCell(int3 cell) {
  uint3 value = uint3(cell) * uint3(73856093u, 19349663u, 83492791u);
  return (value.x ^ value.y ^ value.z) % bucket_count;
}

float4 SampleWind(float3 start, float3 end, float rod_radius) {
  if (particle_count == 0) return 0;
  float reach = rod_radius + max_particle_radius;
  int3 lower = int3(floor((min(start, end) - reach) / cell_size));
  int3 upper = int3(floor((max(start, end) + reach) / cell_size));
  float3 segment = end - start;
  float segment_length_squared = dot(segment, segment);
  float3 velocity = 0;
  float weight_sum = 0;
  float density_sum = 0;
  for (int z = lower.z; z <= upper.z; ++z)
    for (int y = lower.y; y <= upper.y; ++y)
      for (int x = lower.x; x <= upper.x; ++x) {
        int3 cell = int3(x, y, z);
        uint particle_index = wind_heads[HashWindCell(cell)];
        for (uint visited = 0; particle_index != 0xffffffffu && visited < 48; ++visited) {
          WindParticle particle = wind_particles[particle_index];
          if (all(int3(floor(particle.position_age.xyz / cell_size)) == cell) &&
              particle.velocity_lifetime.w > particle.position_age.w) {
            float3 offset = particle.position_age.xyz - start;
            float t = segment_length_squared > 1e-12 ? saturate(dot(offset, segment) / segment_length_squared) : 0.0;
            float distance = length(offset - t * segment);
            float radius = rod_radius + particle.collision.x;
            float weight = radius > 0.0 ? saturate(1.0 - distance / radius) : 0.0;
            velocity += weight * particle.velocity_lifetime.xyz;
            weight_sum += weight;
            density_sum += weight * particle.collision.y;
          }
          particle_index = wind_next[particle_index];
        }
      }
  return weight_sum > 0.0 ? float4(velocity / weight_sum,
                                   saturate(weight_sum * 0.25) * density_sum / (weight_sum * 1.225)) : 0;
}

float4 QuatMultiply(float4 a, float4 b) {
  return float4(a.w * b.xyz + b.w * a.xyz + cross(a.xyz, b.xyz),
                a.w * b.w - dot(a.xyz, b.xyz));
}

float4 QuatInverse(float4 q) { return float4(-q.xyz, q.w); }

float4 QuatSlerp(float4 a, float4 b, float t) {
  float cosine = dot(a, b);
  if (cosine < 0.0) {
    b = -b;
    cosine = -cosine;
  }
  if (cosine > 0.9995)
    return normalize(lerp(a, b, t));
  float angle = acos(clamp(cosine, -1.0, 1.0));
  return (sin((1.0 - t) * angle) * a + sin(t * angle) * b) / sin(angle);
}

float3 QuatRotate(float4 q, float3 v) {
  float3 t = 2.0 * cross(q.xyz, v);
  return v + q.w * t + cross(q.xyz, t);
}

float4 QuatFromTo(float3 from, float3 to) {
  float cosine = dot(from, to);
  if (cosine < -0.9999) {
    float3 axis = normalize(cross(abs(from.y) < 0.9 ? float3(0, 1, 0) : float3(1, 0, 0), from));
    return float4(axis, 0);
  }
  return normalize(float4(cross(from, to), 1.0 + cosine));
}

void Predict(uint3 thread_id : SV_DispatchThreadID) {
  uint i = thread_id.x;
  if (i >= vertex_count) return;
  TreePhysicsVertex vertex = vertices_in[i];
  TreePhysicsTreeParameters tree = tree_parameters[vertex.incident_rods.z];
  if (tree.control.z == 0) {
    if (tree.control.w != 0 && substep == 0) {
      vertex.position_inverse_mass.xyz = mul(float4(vertex.position_inverse_mass.xyz, 1.0), tree.model_delta).xyz;
      vertex.velocity_radius.xyz = QuatRotate(tree.rotation_delta, vertex.velocity_radius.xyz);
    }
    vertices_out[i] = vertex;
    return;
  }
  if (substep >= tree.control.x) {
    vertices_out[i] = vertex;
    return;
  }
  float time_step = tree.mechanics.x;
  float3 position = vertex.position_inverse_mass.xyz;
  previous[i] = float4(position, 0);
  if (vertex.position_inverse_mass.w <= 0.0) {
    float3 rest = vertex.rest_local.xyz + tree.anchor_offset.xyz;
    float fraction = float(substep + 1) / float(tree.control.x);
    position = lerp(mul(float4(rest, 1.0), tree.previous_model).xyz,
                    mul(float4(rest, 1.0), tree.current_model).xyz, fraction);
    vertex.velocity_radius.xyz = float3(0, 0, 0);
  } else {
    float3 acceleration = tree.gravity.xyz;
    if (particle_count != 0 && tree.gravity.w > 0.0) {
      float3 drag = 0;
      for (uint j = 0; j < vertex.incident_rods.y; ++j) {
        TreePhysicsRod rod = rods_in[vertex_rods[vertex.incident_rods.x + j]];
        uint other = rod.links.x == i ? rod.links.y : rod.links.x;
        float3 other_position = vertices_in[other].position_inverse_mass.xyz;
        float4 wind = SampleWind(position, other_position, rod.length_scale_mass_radius.w);
        if (wind.w <= 0.0) continue;
        float3 direction = other_position - position;
        float length_squared = dot(direction, direction);
        if (length_squared <= 1e-12) continue;
        direction *= rsqrt(length_squared);
        float3 relative = wind.xyz - vertex.velocity_radius.xyz;
        float3 crosswind = relative - dot(relative, direction) * direction;
        drag += wind.w * rod.length_scale_mass_radius.w * rod.length_scale_mass_radius.x *
                length(crosswind) * crosswind;
      }
      float drag_length = length(drag);
      if (drag_length > 0.0)
        acceleration += min(50.0, drag_length * 0.6 * tree.gravity.w *
                                      vertex.position_inverse_mass.w) * drag / drag_length;
    }
    position += time_step * vertex.velocity_radius.xyz + time_step * time_step * acceleration;
  }
  vertex.position_inverse_mass.xyz = position;
  predicted[i] = float4(position, 0);
  vertices_out[i] = vertex;
}

void SolvePosition(uint3 thread_id : SV_DispatchThreadID) {
  uint i = thread_id.x;
  if (i >= vertex_count) return;
  TreePhysicsVertex vertex = vertices_in[i];
  TreePhysicsTreeParameters tree = tree_parameters[vertex.incident_rods.z];
  if (tree.control.z == 0 || substep >= tree.control.x || iteration >= tree.control.y) {
    vertices_out[i] = vertex;
    return;
  }
  float time_step = tree.mechanics.x;
  float stretch_stiffness = tree.mechanics.y;
  float inverse_mass = vertex.position_inverse_mass.w;
  if (inverse_mass <= 0.0) {
    vertices_out[i] = vertex;
    return;
  }
  float inertia = 1.0 / (inverse_mass * time_step * time_step);
  float3 numerator = inertia * predicted[i].xyz;
  float denominator = inertia;
  for (uint j = 0; j < vertex.incident_rods.y; ++j) {
    TreePhysicsRod rod = rods_in[vertex_rods[vertex.incident_rods.x + j]];
    bool start = rod.links.x == i;
    uint other = start ? rod.links.y : rod.links.x;
    float length = max(rod.length_scale_mass_radius.x, 1e-6);
    float stiffness = stretch_stiffness * rod.length_scale_mass_radius.y / length;
    float3 direction = QuatRotate(rod.rotation, float3(1, 0, 0));
    numerator += stiffness * (vertices_in[other].position_inverse_mass.xyz +
                              (start ? -1.0 : 1.0) * length * direction);
    denominator += stiffness;
  }
  vertex.position_inverse_mass.xyz = numerator / denominator;
  vertices_out[i] = vertex;
}

void SolveRotation(uint3 thread_id : SV_DispatchThreadID) {
  uint i = thread_id.x;
  if (i >= rod_count) return;
  TreePhysicsRod rod = rods_in[i];
  TreePhysicsTreeParameters tree = tree_parameters[rod.incident_angles.z];
  if (tree.control.z == 0) {
    if (tree.control.w != 0 && substep == 0 && iteration == 0)
      rod.rotation = normalize(QuatMultiply(tree.rotation_delta, rod.rotation));
    rods_out[i] = rod;
    return;
  }
  if (substep >= tree.control.x || iteration >= tree.control.y) {
    rods_out[i] = rod;
    return;
  }
  if (rod.links.w != 0) {
    float fraction = float(substep + 1) / float(tree.control.x);
    rod.rotation = normalize(QuatMultiply(QuatSlerp(tree.anchor_previous, tree.anchor_current, fraction),
                                          rod.rest_rotation));
    rods_out[i] = rod;
    return;
  }
  float3 displacement = vertices_in[rod.links.y].position_inverse_mass.xyz -
                        vertices_in[rod.links.x].position_inverse_mass.xyz;
  if (dot(displacement, displacement) < 1e-12) {
    rods_out[i] = rod;
    return;
  }
  float4 stretch_target = normalize(QuatMultiply(
      QuatFromTo(QuatRotate(rod.rotation, float3(1, 0, 0)), normalize(displacement)), rod.rotation));
  float stretch_weight = tree.mechanics.y * rod.length_scale_mass_radius.y;
  float4 sum = stretch_weight * stretch_target;
  float total = stretch_weight;
  for (uint j = 0; j < rod.incident_angles.y; ++j) {
    TreePhysicsAngle angle = angles[rod_angles[rod.incident_angles.x + j]];
    bool first = angle.rods.x == i;
    TreePhysicsRod other = rods_in[first ? angle.rods.y : angle.rods.x];
    float4 target = normalize(QuatMultiply(other.rotation,
                                           first ? QuatInverse(angle.rest_relative) : angle.rest_relative));
    float weight = tree.mechanics.z * angle.bend_scale.x /
                   max(0.5 * (rod.length_scale_mass_radius.x + other.length_scale_mass_radius.x), 1e-6);
    sum += (dot(target, stretch_target) < 0.0 ? -1.0 : 1.0) * weight * target;
    total += weight;
  }
  if (total > 0.0 && dot(sum, sum) > 1e-12)
    rod.rotation = normalize(sum);
  rods_out[i] = rod;
}

void UpdateVelocity(uint3 thread_id : SV_DispatchThreadID) {
  uint i = thread_id.x;
  if (i >= vertex_count) return;
  TreePhysicsVertex vertex = vertices_in[i];
  TreePhysicsTreeParameters tree = tree_parameters[vertex.incident_rods.z];
  if (tree.control.z == 0 || substep >= tree.control.x) {
    vertices_out[i] = vertex;
    return;
  }
  float time_step = tree.mechanics.x;
  if (vertex.position_inverse_mass.w > 0.0)
    vertex.velocity_radius.xyz = (vertex.position_inverse_mass.xyz - previous[i].xyz) / time_step;
  else
    vertex.velocity_radius.xyz = 0.0;
  vertex.velocity_radius.xyz *= max(0.0, 1.0 - tree.mechanics.w * time_step);
  vertices_out[i] = vertex;
}

[numthreads(64, 1, 1)]
void main(uint3 thread_id : SV_DispatchThreadID) {
#if TREE_PHYSICS_PASS == 0
  Predict(thread_id);
#elif TREE_PHYSICS_PASS == 1
  SolvePosition(thread_id);
#elif TREE_PHYSICS_PASS == 2
  SolveRotation(thread_id);
#elif TREE_PHYSICS_PASS == 3
  UpdateVelocity(thread_id);
#endif
}
