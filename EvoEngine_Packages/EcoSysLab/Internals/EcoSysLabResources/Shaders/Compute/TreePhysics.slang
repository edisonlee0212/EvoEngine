import EcoSysLab.TreePhysicsData;
import EcoSysLab.TreePhysicsLodData;
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
  float4 material;
};

import EcoSysLab.AirGrid;

[[vk::binding(0, 0)]] StructuredBuffer<TreePhysicsBuffers, Std430DataLayout> physics_trees;
[[vk::binding(1, 0)]] StructuredBuffer<uint2> vertex_groups;
[[vk::binding(2, 0)]] StructuredBuffer<uint2> rod_groups;
[[vk::binding(3, 0)]] StructuredBuffer<TreePhysicsTreeParameters, Std430DataLayout> tree_parameters;
[[vk::binding(4, 0)]] StructuredBuffer<uint2> coefficient_groups;
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
  return AirGridSampleWind(wind_heads, wind_addresses, bucket_count, cell_size, (start+end)*0.5);
}

float3 CoarseWindDrag(uint vertex_id,TreePhysicsBuffers buffers,TreePhysicsVertex vertex) {
  TreeLodHeader header=PhysicsLoad<TreeLodHeader>(buffers.reserved,0);
  TreeLodLevel level=PhysicsLoad<TreeLodLevel>(header.levels,PhysicsLoad<uint>(header.state,0));
  float3 drag=0;
  for (uint j=0;j<vertex.incident_rods.y;++j) {
    uint coarse_id=PhysicsLoad<uint>(buffers.vertex_rods,vertex.incident_rods.x+j);
    TreePhysicsRod coarse=PhysicsRod(buffers,rod_index,coarse_id);
    for (uint k=0;k<coarse.incident_angles.w;++k) {
      uint original=PhysicsLoad<uint>(level.members,coarse.incident_angles.z+k);
      TreePhysicsRod fine=PhysicsRod(header.full,0,original);
      float3 a,b,va,vb;
      LodMappedMotion(buffers,level,fine.links.x,vertex_index,rod_index,a,va);
      LodMappedMotion(buffers,level,fine.links.y,vertex_index,rod_index,b,vb);
      float3 direction=b-a;float squared=dot(direction,direction);
      if (squared<=1e-12) continue;
      direction*=rsqrt(squared);
      float4 wind=SampleWind(a,b);
      if (wind.w<=0) continue;
      float3 ra=wind.xyz-va,rb=wind.xyz-vb;
      float3 ca=ra-dot(ra,direction)*direction,cb=rb-dot(rb,direction)*direction;
      float wa=LodVertexWeight(level,fine.links.x,vertex_id,coarse_id);
      float wb=LodVertexWeight(level,fine.links.y,vertex_id,coarse_id);
      drag+=wind.w*fine.length_scale_mass_radius.w*fine.length_scale_mass_radius.x*
            (wa*length(ca)*ca+wb*length(cb)*cb);
    }
  }
  return drag;
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

// Static attribute columns are initialized in both copies and never written by the solver.
void WriteVertexMotion(TreePhysicsBuffers buffers, uint i, TreePhysicsVertex vertex) {
#if defined(EE_TREE_PHYSICS_FULL_WRITES)
  PhysicsStore<TreePhysicsVertex>(buffers.vertices[1 - vertex_index], i, vertex);
#else
  bool arrays = buffers.counts.w != 0;
  PhysicsStore<float4>(buffers.vertices[1-vertex_index], PhysicsAttributeIndex(i,0,buffers.counts.x,4,arrays), vertex.position_inverse_mass);
  PhysicsStore<float4>(buffers.vertices[1-vertex_index], PhysicsAttributeIndex(i,1,buffers.counts.x,4,arrays), vertex.velocity_radius);
#endif
}
void WriteRodRotation(TreePhysicsBuffers buffers, uint i, TreePhysicsRod rod) {
#if defined(EE_TREE_PHYSICS_FULL_WRITES)
  PhysicsStore<TreePhysicsRod>(buffers.rods[1 - rod_index], i, rod);
#else
  PhysicsStore<float4>(buffers.rods[1-rod_index], PhysicsAttributeIndex(i,2,buffers.counts.y,5,buffers.counts.w != 0), rod.rotation);
#endif
}

void PrepareCoefficients(uint i, TreePhysicsBuffers buffers, TreePhysicsTreeParameters tree) {
  if ((tree.control.w & 2u) == 0) return;
  if (i < buffers.counts.x) {
    TreePhysicsVertex v = PhysicsVertex(buffers,0,i);
    float inertia = v.position_inverse_mass.w > 0 ? tree.material.x/(v.position_inverse_mass.w*tree.mechanics.x*tree.mechanics.x) : 0;
    float denominator = inertia;
    for (uint j=0; j<v.incident_rods.y; ++j) {
      TreePhysicsRod r = PhysicsRod(buffers,0,PhysicsLoad<uint>(buffers.vertex_rods,v.incident_rods.x+j));
      denominator += tree.mechanics.y*r.length_scale_mass_radius.y/max(r.length_scale_mass_radius.x,1e-6);
    }
    PhysicsStore<float2>(buffers.vertex_coefficients,i,float2(inertia,denominator));
  }
  if (i < buffers.counts.y) {
    TreePhysicsRod r = PhysicsRod(buffers,0,i);
    float stretch = tree.mechanics.y*r.length_scale_mass_radius.y;
    PhysicsStore<float2>(buffers.rod_coefficients,i,float2(stretch/max(r.length_scale_mass_radius.x,1e-6),stretch));
  }
  if (i < buffers.counts.z) {
    TreePhysicsAngle a = PhysicsLoad<TreePhysicsAngle>(buffers.angles,i);
    float length_a = PhysicsRod(buffers,0,a.rods.x).length_scale_mass_radius.x;
    float length_b = PhysicsRod(buffers,0,a.rods.y).length_scale_mass_radius.x;
    PhysicsStore<float>(buffers.angle_coefficients,i,tree.mechanics.z*a.bend_scale.x/max(0.5*(length_a+length_b),1e-6));
  }
}

void Predict(uint i, TreePhysicsBuffers buffers, TreePhysicsTreeParameters tree) {
  if (i >= buffers.counts.x) return;
  TreePhysicsVertex vertex = PhysicsVertex(buffers, vertex_index, i);
  if (tree.control.z == 0) {
    if ((tree.control.w & 1u) != 0 && substep == 0) {
      vertex.position_inverse_mass.xyz = mul(float4(vertex.position_inverse_mass.xyz, 1.0), tree.model_delta).xyz;
      vertex.velocity_radius.xyz = QuatRotate(tree.rotation_delta, vertex.velocity_radius.xyz);
    }
    WriteVertexMotion(buffers, i, vertex);
    return;
  }
  if (substep >= tree.control.x) {
    WriteVertexMotion(buffers, i, vertex);
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
      if ((buffers.counts.w & 2u) != 0) drag=CoarseWindDrag(i,buffers,vertex);
      else for (uint j = 0; j < vertex.incident_rods.y; ++j) {
        uint rod_id = PhysicsLoad<uint>(buffers.vertex_rods, vertex.incident_rods.x + j);
        TreePhysicsRod rod = PhysicsRod(buffers, rod_index, rod_id);
        uint other = rod.links.x == i ? rod.links.y : rod.links.x;
        TreePhysicsVertex other_vertex = PhysicsVertex(buffers, vertex_index, other);
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
                                      vertex.position_inverse_mass.w / tree.material.x) * drag / drag_length;
    }
    position += time_step * vertex.velocity_radius.xyz + time_step * time_step * acceleration;
  }
  vertex.position_inverse_mass.xyz = position;
  PhysicsStore<float4>(buffers.predicted, i, float4(position, 0));
  WriteVertexMotion(buffers, i, vertex);
}

void SolvePosition(uint i, TreePhysicsBuffers buffers, TreePhysicsTreeParameters tree) {
  if (i >= buffers.counts.x) return;
  TreePhysicsVertex vertex = PhysicsVertex(buffers, vertex_index, i);
  if (tree.control.z == 0 || substep >= tree.control.x || iteration >= tree.control.y) {
    WriteVertexMotion(buffers, i, vertex);
    return;
  }
  float time_step = tree.mechanics.x;
  float stretch_stiffness = tree.mechanics.y;
  float inverse_mass = vertex.position_inverse_mass.w;
  if (inverse_mass <= 0.0) {
    WriteVertexMotion(buffers, i, vertex);
    return;
  }
#ifdef EE_TREE_PHYSICS_REFERENCE
  float inertia = tree.material.x / (inverse_mass * time_step * time_step);
  float denominator = inertia;
#else
  float2 coefficients = PhysicsLoad<float2>(buffers.vertex_coefficients,i);
  float inertia = coefficients.x;
  float denominator = coefficients.y;
#endif
  float3 numerator = inertia * PhysicsLoad<float4>(buffers.predicted, i).xyz;
  for (uint j = 0; j < vertex.incident_rods.y; ++j) {
    uint rod_id = PhysicsLoad<uint>(buffers.vertex_rods, vertex.incident_rods.x + j);
    TreePhysicsRod rod = PhysicsRod(buffers, rod_index, rod_id);
    bool start = rod.links.x == i;
    uint other = start ? rod.links.y : rod.links.x;
    float length = max(rod.length_scale_mass_radius.x, 1e-6);
#ifdef EE_TREE_PHYSICS_REFERENCE
    float stiffness = stretch_stiffness * rod.length_scale_mass_radius.y / length;
#else
    float stiffness = PhysicsLoad<float2>(buffers.rod_coefficients,rod_id).x;
#endif
    float3 direction = QuatRotate(rod.rotation, float3(1, 0, 0));
    numerator += stiffness * (PhysicsVertex(buffers, vertex_index, other).position_inverse_mass.xyz +
                              (start ? -1.0 : 1.0) * length * direction);
#ifdef EE_TREE_PHYSICS_REFERENCE
    denominator += stiffness;
#endif
  }
  vertex.position_inverse_mass.xyz = numerator / denominator;
  WriteVertexMotion(buffers, i, vertex);
}

void SolveRotation(uint i, TreePhysicsBuffers buffers, TreePhysicsTreeParameters tree) {
  if (i >= buffers.counts.y) return;
  TreePhysicsRod rod = PhysicsRod(buffers, rod_index, i);
  if (tree.control.z == 0) {
    if ((tree.control.w & 1u) != 0 && substep == 0 && iteration == 0)
      rod.rotation = normalize(QuatMultiply(tree.rotation_delta, rod.rotation));
    WriteRodRotation(buffers, i, rod);
    return;
  }
  if (substep >= tree.control.x || iteration >= tree.control.y) {
    WriteRodRotation(buffers, i, rod);
    return;
  }
  if (rod.links.w != 0) {
    float fraction = float(substep + 1) / float(tree.control.x);
    rod.rotation = normalize(QuatMultiply(QuatSlerp(tree.anchor_previous, tree.anchor_current, fraction),
                                          rod.rest_rotation));
    WriteRodRotation(buffers, i, rod);
    return;
  }
  float3 displacement = PhysicsVertex(buffers, vertex_index, rod.links.y).position_inverse_mass.xyz -
                        PhysicsVertex(buffers, vertex_index, rod.links.x).position_inverse_mass.xyz;
  if (dot(displacement, displacement) < 1e-12) {
    WriteRodRotation(buffers, i, rod);
    return;
  }
  float4 stretch_target = normalize(QuatMultiply(
      QuatFromTo(QuatRotate(rod.rotation, float3(1, 0, 0)), normalize(displacement)), rod.rotation));
#ifdef EE_TREE_PHYSICS_REFERENCE
  float stretch_weight = tree.mechanics.y * rod.length_scale_mass_radius.y;
#else
  float stretch_weight = PhysicsLoad<float2>(buffers.rod_coefficients,i).y;
#endif
  float4 sum = stretch_weight * stretch_target;
  float total = stretch_weight;
  for (uint j = 0; j < rod.incident_angles.y; ++j) {
    uint angle_id = PhysicsLoad<uint>(buffers.rod_angles, rod.incident_angles.x + j);
    TreePhysicsAngle angle = PhysicsLoad<TreePhysicsAngle>(buffers.angles, angle_id);
    bool first = angle.rods.x == i;
    TreePhysicsRod other = PhysicsRod(buffers, rod_index, first ? angle.rods.y : angle.rods.x);
    float4 target = normalize(QuatMultiply(other.rotation,
                                           first ? QuatInverse(angle.rest_relative) : angle.rest_relative));
#ifdef EE_TREE_PHYSICS_REFERENCE
    float weight = tree.mechanics.z * angle.bend_scale.x /
                   max(0.5 * (rod.length_scale_mass_radius.x + other.length_scale_mass_radius.x), 1e-6);
#else
    float weight = PhysicsLoad<float>(buffers.angle_coefficients,angle_id);
#endif
    sum += (dot(target, stretch_target) < 0.0 ? -1.0 : 1.0) * weight * target;
    total += weight;
  }
  if (total > 0.0 && dot(sum, sum) > 1e-12)
    rod.rotation = normalize(sum);
  WriteRodRotation(buffers, i, rod);
}

void UpdateVelocity(uint i, TreePhysicsBuffers buffers, TreePhysicsTreeParameters tree) {
  if (i >= buffers.counts.x) return;
  TreePhysicsVertex vertex = PhysicsVertex(buffers, vertex_index, i);
  if (tree.control.z == 0 || substep >= tree.control.x) {
    WriteVertexMotion(buffers, i, vertex);
    return;
  }
  float time_step = tree.mechanics.x;
  if (vertex.position_inverse_mass.w > 0.0)
    vertex.velocity_radius.xyz = (vertex.position_inverse_mass.xyz - PhysicsLoad<float4>(buffers.previous, i).xyz) / time_step;
  else
    vertex.velocity_radius.xyz = 0.0;
  vertex.velocity_radius.xyz *= max(0.0, 1.0 - tree.mechanics.w * time_step);
  WriteVertexMotion(buffers, i, vertex);
}

[numthreads(64, 1, 1)]
void main(uint3 group : SV_GroupID, uint3 lane : SV_GroupThreadID) {
#if TREE_PHYSICS_PASS == 4
  uint2 work = coefficient_groups[group.x];
#elif TREE_PHYSICS_PASS == 2
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
#elif TREE_PHYSICS_PASS == 4
  PrepareCoefficients(i, buffers, tree);
#endif
}
