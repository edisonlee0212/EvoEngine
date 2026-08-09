// @evoengine-dialect native
#ifndef EE_GLTF_COMPILED_FEATURE_MASK
#  define EE_GLTF_COMPILED_FEATURE_MASK 32767u
#endif
import EvoEngine.CameraRayConstants;
import EvoEngine.CameraRayGeometry;
import EvoEngine.CameraRayTracingPayload;
import EvoEngine.GltfRayTracingMaterial;
import EvoEngine.Random;
import EvoEngine.RayTracingBasic;
import EvoEngine.RayTracingMaterial;

float3 EE_CAMERA_ANY_HIT_SAFE_NORMALIZE(const float3 value, const float3 fallback) {
  const float length_squared = dot(value, value);
  return length_squared > 0.00000001f ? value * rsqrt(length_squared) : fallback;
}

float3 EE_CAMERA_ANY_HIT_SHADOW_TRANSMISSION<let feature_mask : uint>(
    const uint material_index, const float2 tex_coord_0, const float2 tex_coord_1, const float2 tex_coord_2,
    const float2 tex_coord_3, const float3 vertex_color, const float3 world_geometric_normal,
    inout CameraRayTracingPayload hit_value) {
  bool is_inside = hit_value.shadow_is_inside != 0u;
  const float3 ray_direction = WorldRayDirection();
  const float cos_theta =
      abs(dot(normalize(ray_direction), EE_CAMERA_ANY_HIT_SAFE_NORMALIZE(world_geometric_normal, -ray_direction)));
  const float segment_length = abs(RayTCurrent() - hit_value.shadow_previous_hit_t);
  const float3 transmission = EE_GLTF_RASTER_SHADOW_TRANSMISSION_LOD0_SPECIALIZED<feature_mask>(
      material_index, tex_coord_0, tex_coord_1, tex_coord_2, tex_coord_3, vertex_color, cos_theta, segment_length,
      is_inside, EE_CAMERA_MIN_SHADOW_TRANSMISSION);
  hit_value.shadow_is_inside = is_inside ? 1u : 0u;
  return max(transmission, float3(0.0f));
}

[shader("anyhit")] void main(inout CameraRayTracingPayload hit_value,
                             in BuiltInTriangleIntersectionAttributes attributes) {
  const Instance instance = EE_INSTANCES[int(InstanceID())];
  const uint material_index = uint(instance.material_index);
  const int triangle_offset = instance.triangle_offset + int(PrimitiveIndex());
  const Vertex v0 = EE_VERTICES[EE_INDICES[triangle_offset * 3]];
  const Vertex v1 = EE_VERTICES[EE_INDICES[triangle_offset * 3 + 1]];
  const Vertex v2 = EE_VERTICES[EE_INDICES[triangle_offset * 3 + 2]];
  const float2 hit_barycentrics = attributes.barycentrics;
  const float3 barycentrics =
      float3(1.0f - hit_barycentrics.x - hit_barycentrics.y, hit_barycentrics.x, hit_barycentrics.y);
  const float2 tex_coord_0 =
      v0.tex_coord * barycentrics.x + v1.tex_coord * barycentrics.y + v2.tex_coord * barycentrics.z;
  const float2 tex_coord_1 =
      v0.tex_coord_1 * barycentrics.x + v1.tex_coord_1 * barycentrics.y + v2.tex_coord_1 * barycentrics.z;
  const float2 tex_coord_2 =
      v0.tex_coord_2 * barycentrics.x + v1.tex_coord_2 * barycentrics.y + v2.tex_coord_2 * barycentrics.z;
  const float2 tex_coord_3 =
      v0.tex_coord_3 * barycentrics.x + v1.tex_coord_3 * barycentrics.y + v2.tex_coord_3 * barycentrics.z;
  const float4 vertex_color = v0.color * barycentrics.x + v1.color * barycentrics.y + v2.color * barycentrics.z;

  const float opacity = EE_GLTF_RASTER_OPACITY_LOD0_SPECIALIZED<EE_GLTF_COMPILED_FEATURE_MASK>(
      material_index, tex_coord_0, tex_coord_1, tex_coord_2, tex_coord_3, vertex_color.a);
  if (EE_PCG_RANDOM(hit_value.seed) > opacity) {
    IgnoreHit();
  }
  if (hit_value.type != EE_CAMERA_RAY_PAYLOAD_SHADOW) {
    return;
  }

  const float3 object_shading_normal = EE_CAMERA_SCALE_INDEPENDENT_NORMALIZE(
      v0.normal * barycentrics.x + v1.normal * barycentrics.y + v2.normal * barycentrics.z, float3(0.0f, 1.0f, 0.0f));
  const float3 object_geometric_normal =
      EE_CAMERA_GEOMETRIC_NORMAL(v1.position - v0.position, v2.position - v0.position, object_shading_normal);
  const float3 world_shading_normal = EE_RT_WORLD_NORMAL(instance.model, object_shading_normal, -WorldRayDirection());
  const float3 world_geometric_normal =
      EE_RT_WORLD_NORMAL(instance.model, object_geometric_normal, world_shading_normal);
  const float3 transmission = EE_CAMERA_ANY_HIT_SHADOW_TRANSMISSION<EE_GLTF_COMPILED_FEATURE_MASK>(
      material_index, tex_coord_0, tex_coord_1, tex_coord_2, tex_coord_3, vertex_color.rgb, world_geometric_normal,
      hit_value);
  hit_value.shadow_previous_hit_t = RayTCurrent();
  hit_value.shadow_transmission *= transmission;

  if (max(max(hit_value.shadow_transmission.x, hit_value.shadow_transmission.y), hit_value.shadow_transmission.z) <=
      EE_CAMERA_MIN_SHADOW_TRANSMISSION) {
    hit_value.shadow_transmission = float3(0.0f);
    AcceptHitAndEndSearch();
  }
  IgnoreHit();
}
