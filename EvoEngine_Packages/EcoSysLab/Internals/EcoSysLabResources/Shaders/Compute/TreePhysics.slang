import EcoSysLab.TreePhysicsData;
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

import EcoSysLab.AirGrid;

[[vk::binding(0, 0)]] StructuredBuffer<TreePhysicsBuffers, Std430DataLayout> physics_trees;
[[vk::binding(1, 0)]] StructuredBuffer<uint2> vertex_groups;
[[vk::binding(2, 0)]] StructuredBuffer<uint2> rod_groups;
[[vk::binding(3, 0)]] StructuredBuffer<TreePhysicsTreeParameters, Std430DataLayout> tree_parameters;
[[vk::binding(0, 1)]] StructuredBuffer<uint> wind_heads;
[[vk::binding(1, 1)]] StructuredBuffer<AirGridAddresses> wind_addresses;

[[vk::push_constant]] cbuffer TreePhysicsParameters {
  uint vertex_index;
  uint rod_index;
  uint substep;
  uint iteration;
  uint cell_count;
  uint bucket_count;
  float cell_size;
  float padding;
};

float4 SampleWind(float3 start, float3 end) {
  if (cell_count == 0) return 0;
  float3 position = (start + end) * (0.5 / cell_size) - 0.5;
  int3 base = int3(floor(position));
  float3 fraction = frac(position);
  float4 result = 0;
  float total = 0;
  for (int z = 0; z < 2; ++z)
    for (int y = 0; y < 2; ++y)
      for (int x = 0; x < 2; ++x) {
        uint index = AirGridFind(wind_heads, wind_addresses, base + int3(x, y, z), bucket_count);
        if (index == 0xffffffffu) continue;
        AirGridValue value = AirGridReadValue(wind_addresses, index);
        float weight = (x != 0 ? fraction.x : 1.0 - fraction.x) *
                       (y != 0 ? fraction.y : 1.0 - fraction.y) *
                       (z != 0 ? fraction.z : 1.0 - fraction.z);
        result += weight * float4(value.velocity_temperature.xyz,
                                  value.humidity_strength_density.y * value.humidity_strength_density.z / 1.225);
        total += weight;
      }
  return total > 0.0 ? result / total : 0;
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

void Predict(uint i, TreePhysicsBuffers buffers, TreePhysicsTreeParameters tree) {
  if (i >= buffers.counts.x) return;
  TreePhysicsVertex vertex = PhysicsLoad<TreePhysicsVertex>(buffers.vertices[vertex_index], i);
  if (tree.control.z == 0) {
    if (tree.control.w != 0 && substep == 0) {
      vertex.position_inverse_mass.xyz = mul(float4(vertex.position_inverse_mass.xyz, 1.0), tree.model_delta).xyz;
      vertex.velocity_radius.xyz = QuatRotate(tree.rotation_delta, vertex.velocity_radius.xyz);
    }
    PhysicsStore<TreePhysicsVertex>(buffers.vertices[1 - vertex_index], i, vertex);
    return;
  }
  if (substep >= tree.control.x) {
    PhysicsStore<TreePhysicsVertex>(buffers.vertices[1 - vertex_index], i, vertex);
    return;
  }
  float time_step = tree.mechanics.x;
  float3 position = vertex.position_inverse_mass.xyz;
  PhysicsStore<float4>(buffers.previous, i, float4(position, 0));
  if (vertex.position_inverse_mass.w <= 0.0) {
    float3 rest = vertex.rest_local.xyz + tree.anchor_offset.xyz;
    float fraction = float(substep + 1) / float(tree.control.x);
    position = lerp(mul(float4(rest, 1.0), tree.previous_model).xyz,
                    mul(float4(rest, 1.0), tree.current_model).xyz, fraction);
    vertex.velocity_radius.xyz = float3(0, 0, 0);
  } else {
    float3 acceleration = tree.gravity.xyz;
    if (cell_count != 0 && tree.gravity.w > 0.0) {
      float3 drag = 0;
      for (uint j = 0; j < vertex.incident_rods.y; ++j) {
        TreePhysicsRod rod = PhysicsLoad<TreePhysicsRod>(buffers.rods[rod_index], PhysicsLoad<uint>(buffers.vertex_rods, vertex.incident_rods.x + j));
        uint other = rod.links.x == i ? rod.links.y : rod.links.x;
        TreePhysicsVertex other_vertex = PhysicsLoad<TreePhysicsVertex>(buffers.vertices[vertex_index], other);
        float3 other_position = other_vertex.position_inverse_mass.xyz;
        float4 wind = SampleWind(position, other_position);
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
  PhysicsStore<float4>(buffers.predicted, i, float4(position, 0));
  PhysicsStore<TreePhysicsVertex>(buffers.vertices[1 - vertex_index], i, vertex);
}

void SolvePosition(uint i, TreePhysicsBuffers buffers, TreePhysicsTreeParameters tree) {
  if (i >= buffers.counts.x) return;
  TreePhysicsVertex vertex = PhysicsLoad<TreePhysicsVertex>(buffers.vertices[vertex_index], i);
  if (tree.control.z == 0 || substep >= tree.control.x || iteration >= tree.control.y) {
    PhysicsStore<TreePhysicsVertex>(buffers.vertices[1 - vertex_index], i, vertex);
    return;
  }
  float time_step = tree.mechanics.x;
  float stretch_stiffness = tree.mechanics.y;
  float inverse_mass = vertex.position_inverse_mass.w;
  if (inverse_mass <= 0.0) {
    PhysicsStore<TreePhysicsVertex>(buffers.vertices[1 - vertex_index], i, vertex);
    return;
  }
  float inertia = 1.0 / (inverse_mass * time_step * time_step);
  float3 numerator = inertia * PhysicsLoad<float4>(buffers.predicted, i).xyz;
  float denominator = inertia;
  for (uint j = 0; j < vertex.incident_rods.y; ++j) {
    TreePhysicsRod rod = PhysicsLoad<TreePhysicsRod>(buffers.rods[rod_index], PhysicsLoad<uint>(buffers.vertex_rods, vertex.incident_rods.x + j));
    bool start = rod.links.x == i;
    uint other = start ? rod.links.y : rod.links.x;
    float length = max(rod.length_scale_mass_radius.x, 1e-6);
    float stiffness = stretch_stiffness * rod.length_scale_mass_radius.y / length;
    float3 direction = QuatRotate(rod.rotation, float3(1, 0, 0));
    numerator += stiffness * (PhysicsLoad<TreePhysicsVertex>(buffers.vertices[vertex_index], other).position_inverse_mass.xyz +
                              (start ? -1.0 : 1.0) * length * direction);
    denominator += stiffness;
  }
  vertex.position_inverse_mass.xyz = numerator / denominator;
  PhysicsStore<TreePhysicsVertex>(buffers.vertices[1 - vertex_index], i, vertex);
}

void SolveRotation(uint i, TreePhysicsBuffers buffers, TreePhysicsTreeParameters tree) {
  if (i >= buffers.counts.y) return;
  TreePhysicsRod rod = PhysicsLoad<TreePhysicsRod>(buffers.rods[rod_index], i);
  if (tree.control.z == 0) {
    if (tree.control.w != 0 && substep == 0 && iteration == 0)
      rod.rotation = normalize(QuatMultiply(tree.rotation_delta, rod.rotation));
    PhysicsStore<TreePhysicsRod>(buffers.rods[1 - rod_index], i, rod);
    return;
  }
  if (substep >= tree.control.x || iteration >= tree.control.y) {
    PhysicsStore<TreePhysicsRod>(buffers.rods[1 - rod_index], i, rod);
    return;
  }
  if (rod.links.w != 0) {
    float fraction = float(substep + 1) / float(tree.control.x);
    rod.rotation = normalize(QuatMultiply(QuatSlerp(tree.anchor_previous, tree.anchor_current, fraction),
                                          rod.rest_rotation));
    PhysicsStore<TreePhysicsRod>(buffers.rods[1 - rod_index], i, rod);
    return;
  }
  float3 displacement = PhysicsLoad<TreePhysicsVertex>(buffers.vertices[vertex_index], rod.links.y).position_inverse_mass.xyz -
                        PhysicsLoad<TreePhysicsVertex>(buffers.vertices[vertex_index], rod.links.x).position_inverse_mass.xyz;
  if (dot(displacement, displacement) < 1e-12) {
    PhysicsStore<TreePhysicsRod>(buffers.rods[1 - rod_index], i, rod);
    return;
  }
  float4 stretch_target = normalize(QuatMultiply(
      QuatFromTo(QuatRotate(rod.rotation, float3(1, 0, 0)), normalize(displacement)), rod.rotation));
  float stretch_weight = tree.mechanics.y * rod.length_scale_mass_radius.y;
  float4 sum = stretch_weight * stretch_target;
  float total = stretch_weight;
  for (uint j = 0; j < rod.incident_angles.y; ++j) {
    TreePhysicsAngle angle = PhysicsLoad<TreePhysicsAngle>(buffers.angles, PhysicsLoad<uint>(buffers.rod_angles, rod.incident_angles.x + j));
    bool first = angle.rods.x == i;
    TreePhysicsRod other = PhysicsLoad<TreePhysicsRod>(buffers.rods[rod_index], first ? angle.rods.y : angle.rods.x);
    float4 target = normalize(QuatMultiply(other.rotation,
                                           first ? QuatInverse(angle.rest_relative) : angle.rest_relative));
    float weight = tree.mechanics.z * angle.bend_scale.x /
                   max(0.5 * (rod.length_scale_mass_radius.x + other.length_scale_mass_radius.x), 1e-6);
    sum += (dot(target, stretch_target) < 0.0 ? -1.0 : 1.0) * weight * target;
    total += weight;
  }
  if (total > 0.0 && dot(sum, sum) > 1e-12)
    rod.rotation = normalize(sum);
  PhysicsStore<TreePhysicsRod>(buffers.rods[1 - rod_index], i, rod);
}

void UpdateVelocity(uint i, TreePhysicsBuffers buffers, TreePhysicsTreeParameters tree) {
  if (i >= buffers.counts.x) return;
  TreePhysicsVertex vertex = PhysicsLoad<TreePhysicsVertex>(buffers.vertices[vertex_index], i);
  if (tree.control.z == 0 || substep >= tree.control.x) {
    PhysicsStore<TreePhysicsVertex>(buffers.vertices[1 - vertex_index], i, vertex);
    return;
  }
  float time_step = tree.mechanics.x;
  if (vertex.position_inverse_mass.w > 0.0)
    vertex.velocity_radius.xyz = (vertex.position_inverse_mass.xyz - PhysicsLoad<float4>(buffers.previous, i).xyz) / time_step;
  else
    vertex.velocity_radius.xyz = 0.0;
  vertex.velocity_radius.xyz *= max(0.0, 1.0 - tree.mechanics.w * time_step);
  PhysicsStore<TreePhysicsVertex>(buffers.vertices[1 - vertex_index], i, vertex);
}

[numthreads(64, 1, 1)]
void main(uint3 group : SV_GroupID, uint3 lane : SV_GroupThreadID) {
#if TREE_PHYSICS_PASS == 2
  uint2 work = rod_groups[group.x];
#else
  uint2 work = vertex_groups[group.x];
#endif
  TreePhysicsBuffers buffers = physics_trees[work.x];
  TreePhysicsTreeParameters tree = tree_parameters[work.x];
  uint i = work.y + lane.x;
#if TREE_PHYSICS_PASS == 0
  Predict(i, buffers, tree);
#elif TREE_PHYSICS_PASS == 1
  SolvePosition(i, buffers, tree);
#elif TREE_PHYSICS_PASS == 2
  SolveRotation(i, buffers, tree);
#elif TREE_PHYSICS_PASS == 3
  UpdateVelocity(i, buffers, tree);
#endif
}
