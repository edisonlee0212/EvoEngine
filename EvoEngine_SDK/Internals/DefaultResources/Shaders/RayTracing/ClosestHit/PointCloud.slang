// @evoengine-dialect native
import EvoEngine.GltfRayTracingMaterial;
import EvoEngine.PointCloudRayTracingPayload;
import EvoEngine.RayTracingBasic;
import EvoEngine.RayTracingMaterial;

[shader("closesthit")]
void main(inout PointCloudRayTracingPayload hit_value, in BuiltInTriangleIntersectionAttributes attributes) {
  const Instance instance = EE_INSTANCES[int(InstanceID())];
  const uint material_index = uint(instance.material_index);
  const int triangle_offset = instance.triangle_offset + int(PrimitiveIndex());
  const Vertex v0 = EE_VERTICES[EE_INDICES[triangle_offset * 3]];
  const Vertex v1 = EE_VERTICES[EE_INDICES[triangle_offset * 3 + 1]];
  const Vertex v2 = EE_VERTICES[EE_INDICES[triangle_offset * 3 + 2]];
  const float2 hit_barycentrics = attributes.barycentrics;
  const float3 barycentrics =
      float3(1.0f - hit_barycentrics.x - hit_barycentrics.y, hit_barycentrics.x, hit_barycentrics.y);
  const float3 position =
      v0.position * barycentrics.x + v1.position * barycentrics.y + v2.position * barycentrics.z;
  const float2 tex_coord =
      v0.tex_coord * barycentrics.x + v1.tex_coord * barycentrics.y + v2.tex_coord * barycentrics.z;
  float3 normal = v0.normal * barycentrics.x + v1.normal * barycentrics.y + v2.normal * barycentrics.z;
  const float3 tangent = v0.tangent * barycentrics.x + v1.tangent * barycentrics.y + v2.tangent * barycentrics.z;
  const GltfRasterMaterial surface = EE_EVALUATE_GLTF_RASTER_SURFACE(material_index, tex_coord, tex_coord);
  normal = EE_EVALUATE_GLTF_RASTER_NORMAL(material_index, tex_coord, tex_coord, normal, tangent);

  const float3 world_position = mul(float4(position, 1.0f), instance.model).xyz;
  const float3 world_normal = EE_RT_WORLD_NORMAL(instance.model, normal, float3(0.0f, 1.0f, 0.0f));
  const float3 world_tangent = EE_RT_WORLD_TANGENT(instance.model, tangent, world_normal);
  const float4 albedo = surface.base_color;
  hit_value.hit_count += 1;
  hit_value.hit_info.color = float4(surface.emissive * albedo.xyz, 1.0f);
  hit_value.hit_info.position = world_position;
  hit_value.hit_info.normal = world_normal;
  hit_value.hit_info.tangent = world_tangent;
  hit_value.hit_info.tex_coord = tex_coord;
  hit_value.handle = instance.renderer_handle;

  if (barycentrics.x > barycentrics.z && barycentrics.x > barycentrics.y) {
    hit_value.hit_info.vertex_info1 = v1.vertex_info1;
    hit_value.hit_info.vertex_info2 = v1.vertex_info2;
    hit_value.hit_info.vertex_info3 = v1.vertex_info3;
    hit_value.hit_info.vertex_info4 = v1.vertex_info4;
  } else if (barycentrics.y > barycentrics.z) {
    hit_value.hit_info.vertex_info1 = v2.vertex_info1;
    hit_value.hit_info.vertex_info2 = v2.vertex_info2;
    hit_value.hit_info.vertex_info3 = v2.vertex_info3;
    hit_value.hit_info.vertex_info4 = v2.vertex_info4;
  } else {
    hit_value.hit_info.vertex_info1 = v0.vertex_info1;
    hit_value.hit_info.vertex_info2 = v0.vertex_info2;
    hit_value.hit_info.vertex_info3 = v0.vertex_info3;
    hit_value.hit_info.vertex_info4 = v0.vertex_info4;
  }
}
