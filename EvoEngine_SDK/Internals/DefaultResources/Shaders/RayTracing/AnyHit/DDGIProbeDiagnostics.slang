// @evoengine-dialect native
import EvoEngine.DDGIProbeRayConstants;
import EvoEngine.PointCloudRayTracingPayload;
import EvoEngine.RayTracingBasic;
import EvoEngine.RayTracingMaterial;

[shader("anyhit")]
void main(inout PointCloudRayTracingPayload hit_value, in BuiltInTriangleIntersectionAttributes attributes) {
  const Instance instance = EE_INSTANCES[int(InstanceID())];
  const uint material_index = uint(instance.material_index);
  if (EE_GLTF_MATERIALS[material_index].alpha_mode != EE_GLTF_ALPHA_MODE_MASK) {
    return;
  }

  const int triangle_offset = instance.triangle_offset + int(PrimitiveIndex());
  const Vertex v0 = EE_VERTICES[EE_INDICES[triangle_offset * 3]];
  const Vertex v1 = EE_VERTICES[EE_INDICES[triangle_offset * 3 + 1]];
  const Vertex v2 = EE_VERTICES[EE_INDICES[triangle_offset * 3 + 2]];
  const float2 hit_barycentrics = attributes.barycentrics;
  const float3 barycentrics =
      float3(1.0f - hit_barycentrics.x - hit_barycentrics.y, hit_barycentrics.x, hit_barycentrics.y);
  const float3 object_normal = EE_RT_SAFE_NORMALIZE(
      v0.normal * barycentrics.x + v1.normal * barycentrics.y + v2.normal * barycentrics.z,
      float3(0.0f, 1.0f, 0.0f));
  const float3 object_geometric_normal =
      EE_RT_GEOMETRIC_NORMAL(v1.position - v0.position, v2.position - v0.position, object_normal);
  const float3 ray_direction = normalize(WorldRayDirection());
  const float3 world_geometric_normal =
      EE_RT_WORLD_NORMAL(instance.model, object_geometric_normal, -ray_direction);
  const bool shadow_ray = hit_value.hit_count != 0u;
  const float4 tex_gradients =
      shadow_ray
          ? float4(0.0f)
          : EE_RT_TEXTURE_GRADIENTS(0.0f, RayTCurrent(), world_geometric_normal, ray_direction, instance.model, v0, v1,
                                    v2, EE_RT_SPHERICAL_RAY_SPREAD(
                                            EE_DDGI_PROBE_RAY_CONSTANTS.probe_counts_and_ray_count.w));
  const EeRayTracingSurfaceAttributes surface_attributes =
      EE_RT_INTERPOLATE_SURFACE_ATTRIBUTES(v0, v1, v2, barycentrics, tex_gradients);
  const bool alpha_mask_passes =
      shadow_ray
          ? EE_GLTF_RASTER_ALPHA_MASK_PASSES_LOD0(material_index, surface_attributes.tex_coords,
                                                   surface_attributes.vertex_color.a)
          : EE_GLTF_RASTER_ALPHA_MASK_PASSES(material_index, surface_attributes.tex_coords,
                                              surface_attributes.vertex_color.a);
  if (!alpha_mask_passes) {
    IgnoreHit();
  }
}
