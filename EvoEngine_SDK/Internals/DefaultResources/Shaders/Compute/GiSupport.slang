import EvoEngine.GiSupport;
import EvoEngine.DDGIGatherMulti;
import EvoEngine.GltfRasterMaterial;
import EvoEngine.GBuffer;
[[vk::binding(0, 1)]] ConstantBuffer<GiSupportParams> params;
[[vk::binding(1, 1)]] RWStructuredBuffer<GiSupportProbe> probes;
[[vk::binding(2, 1)]] RWStructuredBuffer<uint> requests;
[[vk::binding(3, 1)]] RWStructuredBuffer<float4> rays;
[[vk::binding(4, 1)]] Texture2D<float4> raw_frame_image;
[[vk::binding(5, 1)]] Texture2D<float> depth_image;
[[vk::binding(6, 1)]] Texture2D<float4> raw_uv_image;
[[vk::binding(7, 1)]] Texture2D<float4> raw_ddx_image;
[[vk::binding(9, 1)]][vk::image_format("r8ui")] RWTexture2D<uint> mask;
[[vk::binding(10, 1)]] Texture3D<uint2> voxels;
[[vk::binding(11, 1)]][vk::image_format("r8ui")] Texture3D<uint> regions;
[[vk::binding(12, 1)]] Texture3D<float3> light_field;
[[vk::binding(13, 1)]] Texture3D<float4> occlusion0;
[[vk::binding(14, 1)]] Texture3D<float4> occlusion1;
[[vk::binding(16, 1)]] SamplerState linear_sampler;
[[vk::binding(19, 1)]] Texture2DArray<float3> diffuse_atlas;
[[vk::binding(20, 1)]] Texture2DArray<float3> specular_atlas;
[[vk::binding(21, 1)]] Texture2D<float4> raw_ddy_image;
[[vk::binding(23, 1)]] RWStructuredBuffer<uint> pending;
[[vk::binding(24, 1)]] Texture2D<uint4> in_metadata;
[[vk::binding(25, 1)]] Texture2D<float4> hddagi_surface;

float3 worldPosition(uint2 pixel) {
  float2 uv = (float2(pixel) + 0.5) / float2(params.viewport);
  return EE_DEPTH_TO_WORLD_POS(params.camera_index, uv, depth_image.Load(int3(pixel, 0)));
}
struct SupportResources : IGiSupportRequests {
  GiSupportProbe loadProbe(uint slot) {
    return probes[slot];
  }
  uint2 loadVoxels(int3 cell) {
    return voxels.Load(int4(cell, 0));
  }
  uint claim(uint slot, uint key) {
    uint previous;
#if GI_SUPPORT_MODE == 0 || GI_SUPPORT_MODE == 4 || GI_SUPPORT_MODE == 5
    InterlockedCompareExchange(requests[slot], 0xffffffffu, key, previous);
#else
    InterlockedCompareExchange(pending[slot], 0xffffffffu, key, previous);
#endif
    return previous;
  }
};
float4 surfaceData(uint2 pixel) {
#ifdef GI_SUPPORT_HDDAGI
  return hddagi_surface.Load(int3(pixel, 0));
#else
  float4 uv = raw_uv_image.Load(int3(pixel, 0)), frame = raw_frame_image.Load(int3(pixel, 0));
  float4 dx = raw_ddx_image.Load(int3(pixel, 0)), dy = raw_ddy_image.Load(int3(pixel, 0));
  EeGBufferMetadata meta = EE_GBUFFER_UNPACK_METADATA(in_metadata.Load(int3(pixel, 0)));
  GltfTexCoords coordinates = EE_GLTF_MAKE_TEX_COORDS_WITH_GRADIENTS(uv.xy, uv.zw, dx.xy, dx.zw, dy.xy, dy.zw);
  float3 normal =
      EE_EVALUATE_GLTF_RASTER_NORMAL(meta.material_index, coordinates, EE_GBUFFER_OCT_DECODE(frame.xy),
                                     EE_GBUFFER_OCT_DECODE(frame.zw), meta.negative_tangent_handedness ? -1 : 1);
  return float4(normal, 1);
#endif
}
#if GI_SUPPORT_MODE == 0
#  ifdef GI_SUPPORT_HDDAGI
float supportVisibility(float3 world, float4 surface, uint cascade) {
  float3 coordinate = GiSupportCell(params, world, cascade);
  float3 diffuse, specular;
  float support, unoccluded;
  float3 normal = normalize(surface.xyz), scaled = normalize(normal * float3(1, params.field.y_mult, 1));
  HddagiGatherProbes(params.field, diffuse_atlas, specular_atlas, occlusion0, occlusion1, linear_sampler, cascade,
                     coordinate, scaled, normal, surface.w >= 2 ? surface.w - 2 : surface.w, surface.w >= 2, diffuse,
                     specular, support);
  HddagiGatherData without_occlusion = params.field;
  without_occlusion.use_occlusion = 0;
  HddagiGatherProbes(without_occlusion, diffuse_atlas, specular_atlas, occlusion0, occlusion1, linear_sampler, cascade,
                     coordinate, scaled, normal, surface.w >= 2 ? surface.w - 2 : surface.w, surface.w >= 2, diffuse,
                     specular, unoccluded);
  return support / max(unoccluded, 1e-8);
}
#  endif
[numthreads(16, 16, 1)] void main(uint3 id : SV_DispatchThreadID) {
  uint2 pixel = id.xy;
  if (any(pixel >= params.viewport))
    return;
  mask[pixel] = 0;
  if (depth_image.Load(int3(pixel, 0)) >= 1)
    return;
  EeGBufferMetadata meta = EE_GBUFFER_UNPACK_METADATA(in_metadata.Load(int3(pixel, 0)));
  if (EE_GLTF_MATERIALS[meta.material_index].unlit != 0)
    return;
  float4 s = surfaceData(pixel);
  if (dot(s.xyz, s.xyz) < 0.5)
    return;
  float3 world = worldPosition(pixel), normal = normalize(s.xyz);
  uint cascade = GiSupportCascade(params, world);
  if (cascade == params.field.cascade_count)
    return;
  float weight;
#  ifdef GI_SUPPORT_HDDAGI
  float3 position = GiSupportFieldPosition(params, world);
  float3 inner_distance =
      float3(params.field.probe_size - 1) * 0.5 - abs(position * (params.field.cascades.data[cascade].to_cell / 8));
  float blend = 1 - smoothstep(0.5, 2.5, min(inner_distance.x, min(inner_distance.y, inner_distance.z)));
  static const float dither[4] = {0, 0.5, 0.75, 0.25};
  if (params.field.blend_ambient == 0 && cascade + 1 < params.field.cascade_count) {
    if (blend > dither[(pixel.x & 1) + (pixel.y & 1) * 2])
      ++cascade;
    blend = 0;
  }
  float visibility = supportVisibility(world, s, cascade);
  if (params.field.blend_ambient != 0 && blend > 0 && cascade + 1 < params.field.cascade_count)
    visibility = lerp(visibility, supportVisibility(world, s, cascade + 1), blend);
  weight = GiSupportRefinementWeight(visibility);
#  else
  EeDdgiGatherResult coarse =
      EE_DDGI_GATHER_IRRADIANCE(normal, normalize(EE_CAMERA_POSITION(params.camera_index) - world), world);
  if (coarse.coverage <= 0)
    return;
  weight = coarse.supported ? GiSupportRefinementWeight(coarse.visibility) : 1;
#  endif
  if (weight <= 0)
    return;
  int4 key;
  float3 start;
  float3 geometric_normal = EE_GBUFFER_OCT_DECODE(raw_frame_image.Load(int3(pixel, 0)).xy);
  if (!GiSupportKey(SupportResources(), params, world, geometric_normal, 1.1, key, start))
    return;
  mask[pixel] = uint(round(weight * 255));
  GiSupportRequest(SupportResources(), params, key, true);
}
#elif GI_SUPPORT_MODE == 5
[numthreads(16, 16, 1)] void main(uint3 id : SV_DispatchThreadID) {
  uint2 pixel = id.xy;
  if (any(pixel >= params.viewport))
    return;
  if (mask[pixel] == 0)
    return;
  int4 key;
  float3 start;
  float3 normal = EE_GBUFFER_OCT_DECODE(raw_frame_image.Load(int3(pixel, 0)).xy);
  if (GiSupportKey(SupportResources(), params, worldPosition(pixel), normal, 1.1, key, start))
    GiSupportRequest(SupportResources(), params, key);
}
#elif GI_SUPPORT_MODE == 1
[numthreads(64, 1, 1)] void main(uint3 id : SV_DispatchThreadID) {
  uint slot = id.x;
  if (slot >= GI_SUPPORT_CAPACITY || requests[slot] == 0xffffffffu)
    return;
  int4 key = GiSupportUnpackKey(params, requests[slot]);
  float3 position = GiSupportWorldPosition(params, key);
  GiSupportProbe probe = probes[slot];
  if (probe.state.x != params.generation || any(probe.key != key)) {
    probe = (GiSupportProbe)0;
    probe.key = key;
    probe.position = float4(position, 1);
    probe.state.x = params.generation;
  }
  probe.state.y = params.frame;
  probes[slot] = probe;
}
#elif GI_SUPPORT_MODE == 2
[numthreads(64, 1, 1)] void main(uint3 id : SV_DispatchThreadID) {
  uint slot = id.x / GI_SUPPORT_RAYS, ray = id.x % GI_SUPPORT_RAYS;
  if (slot >= GI_SUPPORT_CAPACITY || requests[slot] == 0xffffffffu)
    return;
  GiSupportProbe probe = probes[slot];
  float3 direction = GiSupportDirection(ray, params.frame, probe.key.xyz);
  float3 scaled_direction = normalize(direction * float3(1, params.field.y_mult, 1));
  int3 cell, side;
  uint cascade, iterations;
  float3 radiance = float3(0);
  if (HddagiTrace(voxels, regions, params.field.cascades, params.field.grid, params.field.cascade_count,
                  GiSupportFieldPosition(params, probe.position.xyz), scaled_direction, uint(probe.key.w), 10, cell,
                  side, cascade, iterations)) {
    cell += side;
    if (all(cell >= 0) && all(cell < params.field.grid)) {
      int4 hit_key = int4(cell + params.field.cascades.data[cascade].region_world_offset * 8, int(cascade));
      if (!GiSupportOccupied(SupportResources(), params, cell, cascade))
        GiSupportRequest(SupportResources(), params, hit_key);
      int3 physical = HddagiWrap(cell + params.field.cascades.data[cascade].region_world_offset * 8, params.field.grid);
      physical.y += params.field.grid.y * cascade;
      radiance = light_field.Load(int4(physical, 0));
    }
  } else if (params.sky_parameters.z > 0) {
    radiance = params.sky_color_energy.rgb;
    if (params.sky_parameters.z == 2) {
      float sine, cosine;
      sincos(params.sky_parameters.x, sine, cosine);
      float3 rotated =
          float3(cosine * direction.x - sine * direction.z, direction.y, sine * direction.x + cosine * direction.z);
      radiance = pow(max(EE_CUBEMAPS[NonUniformResourceIndex(int(params.sky_parameters.w))].SampleLevel(rotated, 0).rgb,
                         float3(0)),
                     float3(params.sky_parameters.y));
    }
    radiance *= params.sky_color_energy.w;
  }
  rays[slot * GI_SUPPORT_RAYS + ray] = float4(all(isfinite(radiance)) ? max(radiance, float3(0)) : float3(0), 1);
}
#elif GI_SUPPORT_MODE == 4
[numthreads(64, 1, 1)] void main(uint3 id : SV_DispatchThreadID) {
  uint slot = id.x;
  if (slot >= GI_SUPPORT_CAPACITY || pending[slot] == 0xffffffffu)
    return;
  GiSupportRequest(SupportResources(), params, GiSupportUnpackKey(params, pending[slot]));
}
#elif GI_SUPPORT_MODE == 3
[numthreads(64, 1, 1)] void main(uint3 id : SV_DispatchThreadID) {
  uint slot = id.x;
  if (slot >= GI_SUPPORT_CAPACITY || requests[slot] == 0xffffffffu)
    return;
  GiSupportProbe probe = probes[slot];
  float alpha = 1.0 / min(probe.state.z + 1, 64u);
  for (uint i = 0; i < 9; ++i) {
    float3 coefficient = float3(0);
    for (uint ray = 0; ray < GI_SUPPORT_RAYS; ++ray) {
      float3 direction = GiSupportDirection(ray, params.frame, probe.key.xyz);
      coefficient +=
          rays[slot * GI_SUPPORT_RAYS + ray].rgb * GiSupportBasis(i, direction) * (12.5663706144 / GI_SUPPORT_RAYS);
    }
    probe.sh[i] = float4(lerp(probe.sh[i].rgb, coefficient, alpha), 0);
  }
  probe.state.z = min(probe.state.z + 1, 64u);
  probes[slot] = probe;
}
#endif
