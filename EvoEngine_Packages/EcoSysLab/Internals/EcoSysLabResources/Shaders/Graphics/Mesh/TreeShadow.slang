import EvoEngine.PerFrame;
import EcoSysLab.TreeCanopy;


[outputtopology("triangle")]
[numthreads(1, 1, 1)]
[shader("mesh")]
void main(uint3 group_id : SV_GroupID, in payload TreeShadowPayload task, OutputVertices<TreeShadowOutput, 4> vertices,
          OutputIndices<uint3, 2> triangles) {
  uint index=task.node_indices[group_id.x];
  bool canopy=(index&0x80000000u)!=0;
  uint group=index&0x7fffffffu;
  uint node_index=index;
  if(canopy) node_index=tree_shadow_ranges[group].z;
  GpuTreeNode node = tree_nodes[node_index];
  uint instance = tree_instance_indices[node.links.y].branch;
  float4x4 model = EE_INSTANCES[instance].model;
  float4x4 projection = EE_DIRECTIONAL_LIGHTS[light_index].light_space_matrix[light_face];
  float3 d = normalize(EE_DIRECTIONAL_LIGHTS[light_index].direction);
  float3 u = normalize(cross(abs(d.y) < 0.9 ? float3(0,1,0) : float3(1,0,0), d));
  float3 v = cross(d, u);
  float3 x = mul(float4(1,0,0,0), model).xyz;
  float3 y = mul(float4(0,1,0,0), model).xyz;
  float3 z = mul(float4(0,0,1,0), model).xyz;
  float3 translation = mul(float4(0,0,0,1), model).xyz;
  float determinant = dot(x, cross(y, z));
  if (abs(determinant) < 1e-12) { SetMeshOutputCounts(0, 0); return; }
  float3 local_d = normalize(TreeShadowLocalVector(d, x, y, z));
  float4 clip_direction = mul(mul(float4(local_d, 0), model), projection);
  float4 points[4];
  uint count;
  TreeCanopyBound bound=(TreeCanopyBound)0;
  if(canopy) {
    bound=tree_canopy_bounds[group];
    points[0]=float4(bound.center.xyz,max(bound.radii.x,max(bound.radii.y,bound.radii.z)));
    points[1]=points[2]=points[3]=points[0];
    count=1;
  } else count=TreeShadowPoints(node,points);
  float2 low = float2(1e30), high = float2(-1e30);
  float near = 1e30;
  float3 scale = float3(length(float3(dot(x,u),dot(y,u),dot(z,u))),
                         length(float3(dot(x,v),dot(y,v),dot(z,v))),
                         length(float3(dot(x,d),dot(y,d),dot(z,d))));
  for (uint i = 0; i < count; ++i) {
    float3 center = mul(float4(points[i].xyz, 1), model).xyz;
    float2 xy = float2(dot(center,u), dot(center,v));
    low = min(low, xy - points[i].w * scale.xy);
    high = max(high, xy + points[i].w * scale.xy);
    near = min(near, dot(center,d) - points[i].w * scale.z);
  }
  // Put raster coverage at the near plane; the fragment shader supplies the actual hit depth.
  // This also retains branches whose proxy bounds cross a cascade's near plane.
  SetMeshOutputCounts(4, 2);
  for (uint i = 0; i < 4; ++i) {
    float3 world = u * ((i & 1u) != 0 ? high.x : low.x) +
                   v * ((i & 2u) != 0 ? high.y : low.y) + d * (near - 1e-4);
    float3 relative = world - translation;
    float4 clip_position = mul(float4(world, 1), projection);
    float clip_near = max(0.0, -clip_position.z / clip_direction.z);
    clip_position.z = 0;
    vertices[i].position = clip_position;
    vertices[i].origin = TreeShadowLocalVector(relative, x, y, z) + local_d * clip_near;
    vertices[i].direction = local_d;
    vertices[i].p0 = points[0]; vertices[i].p1 = points[1];
    vertices[i].p2 = points[2]; vertices[i].p3 = points[3];
    if(canopy) {
      vertices[i].p0=bound.center;
      vertices[i].p1=bound.radii;
    }
    vertices[i].instance_count = uint2(instance, canopy ? 0 : count);
  }
  triangles[0] = uint3(0, 1, 2);
  triangles[1] = uint3(2, 1, 3);
}
