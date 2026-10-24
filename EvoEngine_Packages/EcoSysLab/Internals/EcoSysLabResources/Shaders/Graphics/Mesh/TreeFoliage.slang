import EvoEngine.PerFrame;
import EvoEngine.RasterVertex;
import EcoSysLab.GpuTreeMesh;

struct FoliageTaskPayload {
  uint node_index;
  uint leaf_count;
};

struct FoliageRasterVertex {
  float4 position : SV_Position;
  [[vk::location(0)]] float3 normal : TEXCOORD0;
  [[vk::location(1)]] float3 tangent : TEXCOORD1;
  [[vk::location(2)]] float2 uv : TEXCOORD2;
  [[vk::location(3)]] nointerpolation uint instance_index : TEXCOORD3;
  [[vk::location(4)]] nointerpolation uint material_index : TEXCOORD4;
};

uint LeafHash(uint value) {
  value ^= value >> 16u;
  value *= 0x7feb352du;
  value ^= value >> 15u;
  value *= 0x846ca68bu;
  return value ^ (value >> 16u);
}

float LeafRandom(uint seed) { return float(LeafHash(seed)) * (1.0 / 4294967296.0); }

float LeafGaussian(uint seed) {
  return sqrt(-2.0 * log(max(1e-7, LeafRandom(seed)))) *
         cos(6.28318530718 * LeafRandom(seed + 1u));
}

float3 LeafRotate(float4 q, float3 v) {
  float3 t = 2.0 * cross(q.xyz, v);
  return v + q.w * t + cross(q.xyz, t);
}

float4 LeafEuler(float3 angle) {
  float3 c = cos(angle * 0.5), s = sin(angle * 0.5);
  return float4(s.x*c.y*c.z-c.x*s.y*s.z, c.x*s.y*c.z+s.x*c.y*s.z,
                c.x*c.y*s.z-s.x*s.y*c.z, c.x*c.y*c.z+s.x*s.y*s.z);
}

float4 LeafMultiply(float4 a, float4 b) {
  return float4(a.w*b.xyz+b.w*a.xyz+cross(a.xyz,b.xyz), a.w*b.w-dot(a.xyz,b.xyz));
}

[outputtopology("triangle")]
[numthreads(64, 1, 1)]
[shader("mesh")]
void main(in payload FoliageTaskPayload task, uint3 group_id : SV_GroupID, uint lane : SV_GroupIndex,
          OutputVertices<FoliageRasterVertex, 64> vertices, OutputIndices<uint3, 32> triangles) {
  uint group_leaves = min(16u, task.leaf_count - group_id.x * 16u);
  SetMeshOutputCounts(group_leaves * 4u, group_leaves * 2u);
  if (lane >= group_leaves * 4u) return;
  GpuTreeNode node = tree_nodes[task.node_index];
  GpuTreeFoliageParams params = tree_foliage_params[node.links.y];
  uint leaf_index = group_id.x * 16u + lane / 4u;
  uint seed = LeafHash(node.foliage_meta.x ^ node.foliage_meta.z ^ (leaf_index * 0x85ebca6bu));
  float t = LeafRandom(seed + 1u);
  float3 center, axis, radial0;
  EvaluateTreeFrame(node, t, center, axis, radial0);
  float3 euler = radians(float3(params.filters.w * LeafGaussian(seed + 3u),
                                params.distributions.z + params.distributions.w * LeafGaussian(seed + 5u),
                                LeafRandom(seed + 2u) * 360.0));
  float4 rotation = LeafMultiply(node.foliage_rotation, LeafEuler(euler));
  float3 front = LeafRotate(rotation, float3(0.0, 0.0, -1.0));
  float3 up = LeafRotate(rotation, float3(0.0, 1.0, 0.0));
  float3 original_axis = normalize(node.direction_length.xyz);
  float3 bend = cross(original_axis, axis);
  float4 alignment = dot(original_axis, axis) > -0.999 ? normalize(float4(bend, 1.0 + dot(original_axis, axis)))
                                                        : float4(radial0, 0.0);
  front = LeafRotate(alignment, front);
  up = LeafRotate(alignment, up);
  float3 tropic = front + float3(0.0, -params.tropism.y, 0.0) +
                  float3(front.x, 0.0, front.z) * params.tropism.x;
  if (dot(tropic, tropic) > 1e-8) front = normalize(tropic);
  float3 right = cross(front, up);
  right = dot(right, right) > 1e-8 ? normalize(right) : radial0;
  up = normalize(cross(right, front));
  float stem = params.distributions.x + params.distributions.y * LeafGaussian(seed + 7u);
  float2 size = params.size_count.xy;
  center += front * (size.y + stem * 0.1);
  uint corner_index = lane & 3u;
  float2 corner = float2(corner_index & 1u, (corner_index >> 1u) & 1u);
  float3 local = center + right * ((corner.x - 0.5) * size.x) - front * ((corner.y - 0.5) * size.y);
  uint instance_index = tree_instance_indices[node.links.y].foliage;
  float4x4 model = EE_INSTANCES[instance_index].model;
  float4x4 projection = EE_CAMERAS[camera_index].projection_view;
  if (pass_index == 1u) projection = EE_POINT_LIGHTS[light_index].light_space_matrix[light_face];
  if (pass_index == 2u) projection = EE_SPOT_LIGHTS[light_index].light_space_matrix;
  if (pass_index == 3u) projection = EE_DIRECTIONAL_LIGHTS[light_index].light_space_matrix[light_face];
  vertices[lane].position = mul(mul(float4(local, 1.0), model), projection);
  vertices[lane].normal = EE_RASTER_WORLD_NORMAL(model, up);
  vertices[lane].tangent = normalize(mul(right, float3x3(model)));
  vertices[lane].uv = corner;
  vertices[lane].instance_index = instance_index;
  vertices[lane].material_index = node.links.w;
  if (corner_index == 0u) {
    uint vertex_base = lane;
    uint triangle_base = leaf_index % 16u * 2u;
    triangles[triangle_base] = uint3(vertex_base, vertex_base + 1u, vertex_base + 2u);
    triangles[triangle_base + 1u] = uint3(vertex_base + 1u, vertex_base + 3u, vertex_base + 2u);
  }
}
