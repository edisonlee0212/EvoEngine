__exported import EvoEngine.GltfRayTracingMaterial;
__exported import EvoEngine.Vertex;
import EvoEngine.Instances;
public
struct EeRayTracingSurfaceAttributes {
 public
  float3 position;
 public
  float3 normal;
 public
  float3 tangent;
 public
  float tangent_handedness;
 public
  float4 vertex_color;
 public
  GltfTexCoords tex_coords;
};
public
float3 EE_RT_SAFE_NORMALIZE(const float3 value, const float3 fallback) {
  if (any(isnan(value)) || any(isinf(value))) {
    return fallback;
  }
  const float scale = max(max(abs(value.x), abs(value.y)), abs(value.z));
  if (scale == 0.0f) {
    return fallback;
  }
  const float3 scaled_value = value / scale;
  return scaled_value * rsqrt(dot(scaled_value, scaled_value));
}
public
float3 EE_RT_GEOMETRIC_NORMAL(const float3 edge_1, const float3 edge_2, const float3 fallback) {
  const float3 normalized_edge_1 = EE_RT_SAFE_NORMALIZE(edge_1, float3(0.0f));
  const float3 normalized_edge_2 = EE_RT_SAFE_NORMALIZE(edge_2, float3(0.0f));
  return EE_RT_SAFE_NORMALIZE(cross(normalized_edge_1, normalized_edge_2), fallback);
}
public
EeRayTracingSurfaceAttributes EE_RT_INTERPOLATE_SURFACE_ATTRIBUTES(const Vertex v0, const Vertex v1, const Vertex v2,
                                                                   const float3 barycentrics,
                                                                   const float4 tex_gradients) {
  EeRayTracingSurfaceAttributes attributes;
  attributes.position = v0.position * barycentrics.x + v1.position * barycentrics.y + v2.position * barycentrics.z;
  attributes.normal = v0.normal * barycentrics.x + v1.normal * barycentrics.y + v2.normal * barycentrics.z;
  attributes.tangent = v0.tangent * barycentrics.x + v1.tangent * barycentrics.y + v2.tangent * barycentrics.z;
  attributes.tangent_handedness = v0.vertex_info3 < 0.0f ? -1.0f : 1.0f;
  attributes.vertex_color = v0.color * barycentrics.x + v1.color * barycentrics.y + v2.color * barycentrics.z;
  attributes.tex_coords = EE_GLTF_MAKE_TEX_COORDS(
      v0.tex_coord * barycentrics.x + v1.tex_coord * barycentrics.y + v2.tex_coord * barycentrics.z,
      v0.tex_coord_1 * barycentrics.x + v1.tex_coord_1 * barycentrics.y + v2.tex_coord_1 * barycentrics.z,
      v0.tex_coord_2 * barycentrics.x + v1.tex_coord_2 * barycentrics.y + v2.tex_coord_2 * barycentrics.z,
      v0.tex_coord_3 * barycentrics.x + v1.tex_coord_3 * barycentrics.y + v2.tex_coord_3 * barycentrics.z,
      tex_gradients);
  return attributes;
}
public
float3x3 EE_RT_WORLD_NORMAL_MATRIX(const float4x4 model) {
  const float3x3 linear = float3x3(model);
  const float3x3 inverse = spirv_asm{OpExtInst $$float3x3 result glsl450 MatrixInverse $linear};
  return transpose(inverse);
}
public
float3 EE_RT_WORLD_NORMAL(const float3x3 normal_matrix, const float3 object_normal, const float3 fallback) {
  return EE_RT_SAFE_NORMALIZE(mul(object_normal, normal_matrix), fallback);
}
public
float3 EE_RT_WORLD_NORMAL(const float4x4 model, const float3 object_normal, const float3 fallback) {
  return EE_RT_WORLD_NORMAL(EE_RT_WORLD_NORMAL_MATRIX(model), object_normal, fallback);
}
public
float3 EE_RT_WORLD_TANGENT(const float4x4 model, const float3 object_tangent, const float3 world_normal) {
  const float3 tangent = mul(float4(object_tangent, 0.0f), model).xyz;
  return EE_RT_SAFE_NORMALIZE(tangent - world_normal * dot(tangent, world_normal),
                              EE_GLTF_FALLBACK_TANGENT(world_normal));
}
public
float EE_RT_WORLD_TANGENT_HANDEDNESS(const float tangent_handedness, const float4x4 model) {
  return tangent_handedness * EE_TRANSFORM_HANDEDNESS(model);
}
public
float4 EE_RT_TEXEL_DENSITY(const float4x4 model, const Vertex v0, const Vertex v1, const Vertex v2) {
  const float3 world_edge_1 = mul(float4(v1.position - v0.position, 0.0f), model).xyz;
  const float3 world_edge_2 = mul(float4(v2.position - v0.position, 0.0f), model).xyz;
  const float world_area = length(cross(world_edge_1, world_edge_2));
  const float2 uv0_edge_1 = v1.tex_coord - v0.tex_coord;
  const float2 uv0_edge_2 = v2.tex_coord - v0.tex_coord;
  const float2 uv1_edge_1 = v1.tex_coord_1 - v0.tex_coord_1;
  const float2 uv1_edge_2 = v2.tex_coord_1 - v0.tex_coord_1;
  const float2 uv2_edge_1 = v1.tex_coord_2 - v0.tex_coord_2;
  const float2 uv2_edge_2 = v2.tex_coord_2 - v0.tex_coord_2;
  const float2 uv3_edge_1 = v1.tex_coord_3 - v0.tex_coord_3;
  const float2 uv3_edge_2 = v2.tex_coord_3 - v0.tex_coord_3;
  const float4 uv_area = abs(float4(uv0_edge_1.x * uv0_edge_2.y - uv0_edge_2.x * uv0_edge_1.y,
                                    uv1_edge_1.x * uv1_edge_2.y - uv1_edge_2.x * uv1_edge_1.y,
                                    uv2_edge_1.x * uv2_edge_2.y - uv2_edge_2.x * uv2_edge_1.y,
                                    uv3_edge_1.x * uv3_edge_2.y - uv3_edge_2.x * uv3_edge_1.y));
  return sqrt(uv_area / max(world_area, 1e-20f));
}
public
float EE_RT_WORLD_FOOTPRINT(const float ray_cone_width, const float hit_t, const float3 geometric_normal,
                            const float3 ray_direction, const float ray_spread_angle) {
  return (ray_cone_width + hit_t * ray_spread_angle) / max(abs(dot(geometric_normal, -ray_direction)), 1e-3f);
}
public
float4 EE_RT_TEXTURE_GRADIENTS(const float ray_cone_width, const float hit_t, const float3 geometric_normal,
                               const float3 ray_direction, const float4x4 model, const Vertex v0, const Vertex v1,
                               const Vertex v2, const float ray_spread_angle) {
  return EE_RT_WORLD_FOOTPRINT(ray_cone_width, hit_t, geometric_normal, ray_direction, ray_spread_angle) *
         EE_RT_TEXEL_DENSITY(model, v0, v1, v2);
}
public
float EE_RT_SPHERICAL_RAY_SPREAD(const uint ray_count) {
  return sqrt(4.0f * 3.14159265359f / float(max(ray_count, 1u)));
}
public
float3 EE_RT_SAFE_OFFSET_RAY(const float3 world_position, const float3 offset_direction) {
  const int3 scale_int = int3(256.0f * offset_direction);
  const float3 offset_position =
      float3(asfloat(asint(world_position.x) + (world_position.x < 0.0f ? -scale_int.x : scale_int.x)),
             asfloat(asint(world_position.y) + (world_position.y < 0.0f ? -scale_int.y : scale_int.y)),
             asfloat(asint(world_position.z) + (world_position.z < 0.0f ? -scale_int.z : scale_int.z)));
  const float origin = 1.0f / 32.0f;
  const float float_scale = 1.0f / 65536.0f;
  return float3(
      abs(world_position.x) < origin ? world_position.x + float_scale * offset_direction.x : offset_position.x,
      abs(world_position.y) < origin ? world_position.y + float_scale * offset_direction.y : offset_position.y,
      abs(world_position.z) < origin ? world_position.z + float_scale * offset_direction.z : offset_position.z);
}
public
float3 EE_RT_OFFSET_RAY_ORIGIN(const float3 world_position, const float3 geometric_normal, const float3 direction) {
  const float3 offset_direction = dot(direction, geometric_normal) >= 0.0f ? geometric_normal : -geometric_normal;
  return EE_RT_SAFE_OFFSET_RAY(world_position, offset_direction);
}
public
float3 EE_RT_COATED_EMISSION_VALUE(const float3 emissive, const float clearcoat, float3 world_clearcoat_normal,
                                   const float3 world_geometric_normal, const float3 outgoing_direction) {
  const float coat = clamp(clearcoat, 0.0f, 1.0f);
  if (coat <= 0.0f) {
    return emissive;
  }
  if (dot(world_clearcoat_normal, world_geometric_normal) < 0.0f) {
    world_clearcoat_normal = world_geometric_normal;
  }
  const float3 incident_direction = -EE_RT_SAFE_NORMALIZE(outgoing_direction, world_geometric_normal);
  if (dot(reflect(incident_direction, world_clearcoat_normal), world_geometric_normal) < 0.0f) {
    world_clearcoat_normal = world_geometric_normal;
  }
  const float cosine = clamp(
      abs(dot(EE_RT_SAFE_NORMALIZE(outgoing_direction, world_clearcoat_normal), world_clearcoat_normal)), 0.0f, 1.0f);
  const float fresnel = 0.04f + 0.96f * pow(max(1.0f - cosine, 0.0f), 5.0f);
  return emissive * max(1.0f - coat * fresnel, 0.0f);
}
public
float3 EE_RT_COATED_EMISSION(const uint material_index, const GltfRasterMaterial surface,
                             const GltfTexCoords tex_coords, const float3 world_base_normal, const float3 world_tangent,
                             const float world_tangent_handedness, const float facing_sign,
                             const float3 world_geometric_normal, const float3 outgoing_direction) {
  if (clamp(surface.clearcoat, 0.0f, 1.0f) <= 0.0f) {
    return surface.emissive;
  }
  const float3 world_clearcoat_normal =
      facing_sign * EE_EVALUATE_GLTF_RASTER_CLEARCOAT_NORMAL(material_index, tex_coords, world_base_normal,
                                                             world_tangent, world_tangent_handedness);
  return EE_RT_COATED_EMISSION_VALUE(surface.emissive, surface.clearcoat, world_clearcoat_normal,
                                     world_geometric_normal, outgoing_direction);
}
public
float3 EE_RT_COATED_EMISSION_LOD0_SPECIALIZED<let feature_mask : uint>(
    const uint material_index, const GltfTexCoords tex_coords, const float3 world_base_normal,
    const float3 world_tangent, const float world_tangent_handedness, const float facing_sign,
    const float3 world_geometric_normal, const float3 outgoing_direction) {
  const GltfShadeMaterial material = EE_GLTF_MATERIALS[material_index];
  const float3 emissive = max(material.emissive_factor * EE_GLTF_SAMPLE_TEXTURE_SLOT_LOD0_SPECIALIZED<feature_mask>(
                                                             material.emissive_texture, EE_GLTF_RASTER_TEXTURE_EMISSIVE,
                                                             tex_coords, float4(1.0f))
                                                             .rgb,
                              float3(0.0f));
  float clearcoat = 0.0f;
  if ((feature_mask & EE_GLTF_FEATURE_CLEARCOAT) != 0u) {
    clearcoat = material.clearcoat_factor *
                EE_GLTF_SAMPLE_TEXTURE_SLOT_LOD0_SPECIALIZED<feature_mask>(
                    material.clearcoat_texture, EE_GLTF_RASTER_TEXTURE_CLEARCOAT, tex_coords, float4(1.0f))
                    .r;
  }
  if (clamp(clearcoat, 0.0f, 1.0f) <= 0.0f) {
    return emissive;
  }
  const float3 world_clearcoat_normal =
      facing_sign * EE_EVALUATE_GLTF_RASTER_CLEARCOAT_NORMAL_SPECIALIZED<feature_mask>(
                        material_index, tex_coords, world_base_normal, world_tangent, world_tangent_handedness);
  return EE_RT_COATED_EMISSION_VALUE(emissive, clearcoat, world_clearcoat_normal, world_geometric_normal,
                                     outgoing_direction);
}
public
float3 EE_RT_COATED_EMISSION_LOD0(const uint material_index, const GltfTexCoords tex_coords,
                                  const float3 world_base_normal, const float3 world_tangent,
                                  const float world_tangent_handedness, const float facing_sign,
                                  const float3 world_geometric_normal, const float3 outgoing_direction) {
  return EE_RT_COATED_EMISSION_LOD0_SPECIALIZED<EE_GLTF_ALL_FEATURES>(
      material_index, tex_coords, world_base_normal, world_tangent, world_tangent_handedness, facing_sign,
      world_geometric_normal, outgoing_direction);
}
public
float3 EE_RT_COATED_EMISSION_LOD0_SPECIALIZED<let feature_mask : uint>(
    const uint material_index, const GltfTexCoords tex_coords, const float3 world_base_normal,
    const float3 world_tangent, const float3 world_bitangent, const float3 world_geometric_normal,
    const float3 outgoing_direction) {
  const float tangent_handedness = dot(cross(world_base_normal, world_tangent), world_bitangent) < 0.0f ? -1.0f : 1.0f;
  return EE_RT_COATED_EMISSION_LOD0_SPECIALIZED<feature_mask>(material_index, tex_coords, world_base_normal,
                                                              world_tangent, tangent_handedness, 1.0f,
                                                              world_geometric_normal, outgoing_direction);
}
public
float3 EE_RT_COATED_EMISSION_LOD0(const uint material_index, const GltfTexCoords tex_coords,
                                  const float3 world_base_normal, const float3 world_tangent,
                                  const float3 world_bitangent, const float3 world_geometric_normal,
                                  const float3 outgoing_direction) {
  const float tangent_handedness = dot(cross(world_base_normal, world_tangent), world_bitangent) < 0.0f ? -1.0f : 1.0f;
  return EE_RT_COATED_EMISSION_LOD0(material_index, tex_coords, world_base_normal, world_tangent, tangent_handedness,
                                    1.0f, world_geometric_normal, outgoing_direction);
}
