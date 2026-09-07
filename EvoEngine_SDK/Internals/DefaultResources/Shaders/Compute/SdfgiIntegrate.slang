// Probe transport and storage adapted from Godot sdfgi_integrate.glsl,
// 34d06658a85845111a50db9e485ec4a0701d4298. See docs/licenses/Godot-MIT.txt.
// Host sky uses scene-linear cubemap radiance at explicit coarse LOD; no camera exposure.
import EvoEngine.SdfgiTypes;
import EvoEngine.SdfgiRelocation;


[[vk::binding(1, 0)]] Texture3D<float> sdf_cascades[8];
[[vk::binding(2, 0)]] Texture3D<float3> light_cascades[8];
[[vk::binding(3, 0)]] Texture3D<float4> aniso0_cascades[8];
[[vk::binding(4, 0)]] Texture3D<float2> aniso1_cascades[8];
[[vk::binding(6, 0)]] SamplerState linear_sampler;
[[vk::binding(7, 0)]] ConstantBuffer<SdfgiCascadeBlock> cascades;
[[vk::binding(8, 0)]] [vk::image_format("r32ui")] RWTexture2DArray<uint> lightprobe_texture_data;
[[vk::binding(9, 0)]] [vk::image_format("rgba16i")] RWTexture2DArray<int4> lightprobe_history_texture;
[[vk::binding(10, 0)]] [vk::image_format("rgba32i")] RWTexture2D<int4> lightprobe_average_texture;
[[vk::binding(11, 0)]] [vk::image_format("rgba16i")] RWTexture2DArray<int4> lightprobe_history_scroll_texture;
[[vk::binding(12, 0)]] [vk::image_format("rgba32i")] RWTexture2D<int4> lightprobe_average_scroll_texture;
[[vk::binding(13, 0)]] [vk::image_format("rgba32i")] RWTexture2D<int4> lightprobe_average_parent_texture;
[[vk::binding(14, 0)]] [vk::image_format("rgba16f")] RWTexture2DArray<float4> lightprobe_ambient_texture;

[[vk::binding(15, 0)]] RWStructuredBuffer<SdfgiFieldStatus> field_status;
[[vk::binding(16, 0)]] RWStructuredBuffer<float4> probe_placement;
[[vk::binding(17, 0)]] RWStructuredBuffer<float4> probe_placement_scroll;
[[vk::binding(0, 1)]] TextureCube<float4> sky_irradiance;
[[vk::binding(1, 1)]] SamplerState linear_sampler_mipmaps;
[[vk::push_constant]] ConstantBuffer<SdfgiIntegratePushConstant> params;

#define SH_SIZE 16
#define OCT_SIZE 6
#define HISTORY_BITS 10
#define SKY_FLAGS_MODE_COLOR 1
#define SKY_FLAGS_MODE_SKY 2
#define SKY_FLAGS_ORIENTATION_SIGN 4

#if defined(EE_SDFGI_RESOURCE_CHECK)
[numthreads(8, 8, 1)]
void main(uint3 id : SV_DispatchThreadID) {
  if (any(id != 0)) return;
  uint errors = 0;
  for (uint c = 0; c < 8; ++c) {
    errors |= sdf_cascades[c].SampleLevel(linear_sampler, 0.5, 0) != 0 ? 4u : 0u;
    errors |= any(abs(light_cascades[c].SampleLevel(linear_sampler, 0.5, 0) - 1.0) > 0.001) ? 8u : 0u;
    errors |= any(aniso0_cascades[c].SampleLevel(linear_sampler, 0.5, 0) != 0) ? 16u : 0u;
    errors |= any(aniso1_cascades[c].SampleLevel(linear_sampler, 0.5, 0) != 0) ? 32u : 0u;
  }
  uint width, height, layers;
  lightprobe_history_texture.GetDimensions(width, height, layers);
  int2 last = int2(width, height) - 1;
  for (uint h = 0; h < params.history_size; ++h) {
    errors |= any(lightprobe_history_texture[int3(last, h)] != 0) ? 64u : 0u;
    errors |= any(lightprobe_history_scroll_texture[int3(last, h)] != 0) ? 128u : 0u;
  }
  errors |= any(lightprobe_average_texture[last] != 0) ? 256u : 0u;
  errors |= any(lightprobe_average_scroll_texture[last] != 0) ? 512u : 0u;
  errors |= any(lightprobe_average_parent_texture[last] != 0) ? 1024u : 0u;
  for (uint c = 0; c < params.max_cascades; ++c)
    errors |= any(lightprobe_ambient_texture[int3(width - 1, height / SH_SIZE - 1, c)] != 0) ? 2048u : 0u;
  errors |= any(sky_irradiance.SampleLevel(linear_sampler_mipmaps, float3(0, 1, 0), 0) != 0) ? 4096u : 0u;
  field_status[0].failure_flags |= errors;
  field_status[0].generation = asuint(cascades.data[7].pad2.w);
}


#elif defined(MODE_RELOCATE)
[numthreads(8, 8, 1)]
void main(uint3 id : SV_DispatchThreadID) {
  int2 pos = int2(id.xy);
  if (any(pos >= params.image_size) || field_status[0].failure_flags != 0) return;
  float spacing = params.grid_size.x / float(params.probe_axis_size - 1);
  int3 size = int3(params.grid_size / spacing) + 1;
  int3 cell = SdfgiProbeCell(pos, params.probe_axis_size, params.image_size.x);
  if (any(cell >= size)) return;
  uint index = SdfgiPlacementIndex(cell, size, params.cascade);
  float4 previous = probe_placement[index];
  if (any(params.scroll != 0) && previous.w != 0) {
    float3 previous_position = (float3(cell) + previous.xyz) * spacing;
    bool affected = any(previous_position < 0) || any(previous_position >= params.grid_size);
    float margin = 0.45 + 1.0 / spacing;
    for (uint axis = 0; axis < 3; ++axis) {
      if (params.scroll[axis] > 0 && cell[axis] <= params.scroll[axis] + margin) affected = true;
      if (params.scroll[axis] < 0 && cell[axis] >= size[axis] - 1 + params.scroll[axis] - margin) affected = true;
    }
    if (!affected) return;
  }
  float4 placement = SdfgiFindPlacement(sdf_cascades[params.cascade], linear_sampler, float3(cell) * spacing,
                                       params.grid_size, spacing);
  if (any(previous != placement)) {
    for (int y = 0; y < OCT_SIZE + 2; ++y)
      for (int x = 0; x < OCT_SIZE + 2; ++x) {
        int2 texel = pos * (OCT_SIZE + 2) + int2(x, y);
        lightprobe_texture_data[int3(texel, params.cascade)] = 0;
        lightprobe_texture_data[int3(texel, params.cascade + params.max_cascades)] = 0;
      }
    lightprobe_ambient_texture[int3(pos, params.cascade)] = float4(0);
    for (int i = 0; i < SH_SIZE; ++i) {
      lightprobe_average_texture[int2(pos.x, pos.y * SH_SIZE + i)] = int4(0);
      for (uint h = 0; h < params.history_size; ++h)
        lightprobe_history_texture[int3(pos.x, pos.y * SH_SIZE + i, h)] = int4(0);
    }
    probe_placement[index] = placement;
  }
}

#elif defined(MODE_PROCESS) || defined(MODE_STORE)
static const float PI = 3.14159265f;
static const float GOLDEN_ANGLE = PI * (3.0 - sqrt(5.0));

float3 vogel_hemisphere(uint p_index, uint p_count, float p_offset) {
  float r = sqrt(float(p_index) + 0.5f) / sqrt(float(p_count));
  float theta = float(p_index) * GOLDEN_ANGLE + p_offset;
  float y = cos(r * PI * 0.5);
  float l = sin(r * PI * 0.5);
  return float3(l * cos(theta), l * sin(theta), y * (float(p_index & 1) * 2.0 - 1.0));
}

uint3 hash3(uint3 x) {
  x = ((x >> 16) ^ x) * 0x45d9f3b;
  x = ((x >> 16) ^ x) * 0x45d9f3b;
  x = (x >> 16) ^ x;
  return x;
}

float hashf3(float3 co) {
  return frac(sin(dot(co, float3(12.9898, 78.233, 137.13451))) * 43758.5453);
}

float3 octahedron_encode(float2 f) {
  // https://twitter.com/Stubbesaurus/status/937994790553227264
  f = f * 2.0 - 1.0;
  float3 n = float3(f.x, f.y, 1.0f - abs(f.x) - abs(f.y));
  float t = clamp(-n.z, 0.0, 1.0);
  n.x += n.x >= 0 ? -t : t;
  n.y += n.y >= 0 ? -t : t;
  return normalize(n);
}

uint rgbe_encode(float3 color) {
  const float pow2to9 = 512.0f;
  const float B = 15.0f;
  const float N = 9.0f;
  const float LN2 = 0.6931471805599453094172321215;

  float cRed = clamp(color.r, 0.0, 65408.0);
  float cGreen = clamp(color.g, 0.0, 65408.0);
  float cBlue = clamp(color.b, 0.0, 65408.0);

  float cMax = max(cRed, max(cGreen, cBlue));

  float expp = max(-B - 1.0f, floor(log(cMax) / LN2)) + 1.0f + B;

  float sMax = floor((cMax / pow(2.0f, expp - B - N)) + 0.5f);

  float exps = expp + 1.0f;

  if (0.0 <= sMax && sMax < pow2to9) {
    exps = expp;
  }

  float sRed = floor((cRed / pow(2.0f, exps - B - N)) + 0.5f);
  float sGreen = floor((cGreen / pow(2.0f, exps - B - N)) + 0.5f);
  float sBlue = floor((cBlue / pow(2.0f, exps - B - N)) + 0.5f);
  return (uint(sRed) & 0x1FF) | ((uint(sGreen) & 0x1FF) << 9) | ((uint(sBlue) & 0x1FF) << 18) | ((uint(exps) & 0x1F) << 27);
}

struct SH {
#if (SH_SIZE == 16)
  float c[48];
#else
  float c[28];
#endif
};

groupshared SH sh_accum[64]; //8x8

[numthreads(8, 8, 1)]
void main(uint3 id : SV_DispatchThreadID, uint3 local_id : SV_GroupThreadID) {
  if (field_status[0].failure_flags != 0) return;
  int2 pos = int2(id.xy);
  if (any((pos >= params.image_size))) { //too large, do nothing
    return;
  }

  uint probe_index = local_id.x + local_id.y * 8;

#ifdef MODE_PROCESS

  float probe_cell_size = float(params.grid_size.x / float(params.probe_axis_size - 1)) / cascades.data[params.cascade].to_cell;

  int3 probe_cell = SdfgiProbeCell(pos, params.probe_axis_size, params.image_size.x);
  if (probe_cell.y > int(params.grid_size.y / (params.grid_size.x / float(params.probe_axis_size - 1)))) return;

  float3 probe_pos = cascades.data[params.cascade].offset + float3(probe_cell) * probe_cell_size;
  if ((params.sky_flags & 8) != 0) {
    int3 size = int3(params.grid_size / (params.grid_size.x / float(params.probe_axis_size - 1))) + 1;
    float4 placement = probe_placement[SdfgiPlacementIndex(probe_cell, size, params.cascade)];
    if (placement.w <= 0) return;
    probe_pos += placement.xyz * probe_cell_size;
  }
  float3 pos_to_uvw = 1.0 / params.grid_size;

  for (uint i = 0; i < SH_SIZE * 3; i++) {
    sh_accum[probe_index].c[i] = 0.0;
  }

  // quickly ensure each probe has a different "offset" for the vogel function, based on integer world position
  uint3 h3 = hash3(uint3(params.world_offset + probe_cell) ^ uint3(params.pad)); // Zero preserves reference sampling.
  float offset = hashf3(float3(h3 & uint3(0xFFFFF)));

  //for a more homogeneous hemisphere, alternate based on history frames
  uint ray_offset = params.history_index;
  uint ray_mult = params.history_size;
  uint ray_total = ray_mult * params.ray_count;

  for (uint i = 0; i < params.ray_count; i++) {
    float3 ray_dir = vogel_hemisphere(ray_offset + i * ray_mult, ray_total, offset);
    ray_dir.y *= params.y_mult;
    ray_dir = normalize(ray_dir);

    //needs to be visible
    float3 ray_pos = probe_pos;
    float3 inv_dir = 1.0 / ray_dir;

    bool hit = false;
    uint hit_cascade;

    float bias = params.ray_bias;
    if ((params.sky_flags & 8) != 0)
      bias = min(bias, min(1.0, 0.25 * params.grid_size.x / float(params.probe_axis_size - 1)) * 0.5);
    float3 abs_ray_dir = abs(ray_dir);
    ray_pos += ray_dir * 1.0 / max(abs_ray_dir.x, max(abs_ray_dir.y, abs_ray_dir.z)) * bias / cascades.data[params.cascade].to_cell;
    float3 uvw;

    for (uint j = params.cascade; j < params.max_cascades; j++) {
      //convert to local bounds
      float3 pos = ray_pos - cascades.data[j].offset;
      pos *= cascades.data[j].to_cell;

      if (any((pos < float3(0.0))) || any((pos >= params.grid_size))) {
        continue; //already past bounds for this cascade, goto next
      }

      //find maximum advance distance (until reaching bounds)
      float3 t0 = -pos * inv_dir;
      float3 t1 = (params.grid_size - pos) * inv_dir;
      float3 tmax = max(t0, t1);
      float max_advance = min(tmax.x, min(tmax.y, tmax.z));

      float advance = 0.0;

      while (advance < max_advance) {
        //read how much to advance from SDF
        uvw = (pos + ray_dir * advance) * pos_to_uvw;

        float distance = sdf_cascades[j].SampleLevel(linear_sampler, uvw, 0.0) * 255.0 - 1.0;
        if (distance < 0.05) {
          //consider hit
          hit = true;
          break;
        }

        advance += distance;
      }

      if (hit) {
        hit_cascade = j;
        break;
      }

      //change ray origin to collision with bounds
      pos += ray_dir * max_advance;
      pos /= cascades.data[j].to_cell;
      pos += cascades.data[j].offset;
      ray_pos = pos;
    }

    float4 light;
    if (hit) {
      //avoid reading different texture from different threads
      for (uint j = params.cascade; j < params.max_cascades; j++) {
        if (j == hit_cascade) {
          const float3 EPSILON = 0.128 / params.grid_size;
          float3 hit_normal = normalize(float3(
              sdf_cascades[hit_cascade].SampleLevel(linear_sampler, uvw + float3(EPSILON.x, 0.0, 0.0), 0.0).r - sdf_cascades[hit_cascade].SampleLevel(linear_sampler, uvw - float3(EPSILON.x, 0.0, 0.0), 0.0),
              sdf_cascades[hit_cascade].SampleLevel(linear_sampler, uvw + float3(0.0, EPSILON.y, 0.0), 0.0).r - sdf_cascades[hit_cascade].SampleLevel(linear_sampler, uvw - float3(0.0, EPSILON.y, 0.0), 0.0),
              sdf_cascades[hit_cascade].SampleLevel(linear_sampler, uvw + float3(0.0, 0.0, EPSILON.z), 0.0).r - sdf_cascades[hit_cascade].SampleLevel(linear_sampler, uvw - float3(0.0, 0.0, EPSILON.z), 0.0)));

          float3 hit_light = light_cascades[hit_cascade].SampleLevel(linear_sampler, uvw, 0.0);
          float4 aniso0 = aniso0_cascades[hit_cascade].SampleLevel(linear_sampler, uvw, 0.0);
          float3 hit_aniso0 = aniso0.rgb;
          float3 hit_aniso1 = float3(aniso0.a, aniso1_cascades[hit_cascade].SampleLevel(linear_sampler, uvw, 0.0).rg);

          //one liner magic
          light.rgb = hit_light * (dot(max(float3(0.0), (hit_normal * hit_aniso0)), float3(1.0)) + dot(max(float3(0.0), (-hit_normal * hit_aniso1)), float3(1.0)));
          light.a = 1.0;
        }
      }

    } else if (bool(params.sky_flags & SKY_FLAGS_MODE_SKY)) {
      // Reconstruct sky orientation as quaternion and rotate ray_dir before sampling.
      float sky_sign = bool(params.sky_flags & SKY_FLAGS_ORIENTATION_SIGN) ? 1.0 : -1.0;
      float4 sky_quat = float4(params.sky_color_or_orientation, sky_sign * sqrt(1.0 - dot(params.sky_color_or_orientation, params.sky_color_or_orientation)));
      float3 sky_dir = cross(sky_quat.xyz, ray_dir);
      sky_dir = ray_dir + ((sky_dir * sky_quat.w) + cross(sky_quat.xyz, sky_dir)) * 2.0;
      light.rgb = pow(sky_irradiance.SampleLevel(linear_sampler_mipmaps, sky_dir,
          params.sky_lod_inverse_gamma.x).rgb, params.sky_lod_inverse_gamma.y);
      light.rgb *= params.sky_energy;
      light.a = 0.0;

    } else if (bool(params.sky_flags & SKY_FLAGS_MODE_COLOR)) {
      light.rgb = params.sky_color_or_orientation;
      light.rgb *= params.sky_energy;
      light.a = 0.0;
    } else {
      light = float4(0, 0, 0, 0);
    }

    float3 ray_dir2 = ray_dir * ray_dir;

#define SH_ACCUM(m_idx, m_value) \
  { \
    float3 l = light.rgb * (m_value); \
    sh_accum[probe_index].c[m_idx * 3 + 0] += l.r; \
    sh_accum[probe_index].c[m_idx * 3 + 1] += l.g; \
    sh_accum[probe_index].c[m_idx * 3 + 2] += l.b; \
  }
    SH_ACCUM(0, 0.282095); //l0
    SH_ACCUM(1, 0.488603 * ray_dir.y); //l1n1
    SH_ACCUM(2, 0.488603 * ray_dir.z); //l1n0
    SH_ACCUM(3, 0.488603 * ray_dir.x); //l1p1
    SH_ACCUM(4, 1.092548 * ray_dir.x * ray_dir.y); //l2n2
    SH_ACCUM(5, 1.092548 * ray_dir.y * ray_dir.z); //l2n1
    SH_ACCUM(6, 0.315392 * (3.0 * ray_dir2.z - 1.0)); //l20
    SH_ACCUM(7, 1.092548 * ray_dir.x * ray_dir.z); //l2p1
    SH_ACCUM(8, 0.546274 * (ray_dir2.x - ray_dir2.y)); //l2p2
#if (SH_SIZE == 16)
    SH_ACCUM(9, 0.590043 * ray_dir.y * (3.0f * ray_dir2.x - ray_dir2.y));
    SH_ACCUM(10, 2.890611 * ray_dir.y * ray_dir.x * ray_dir.z);
    SH_ACCUM(11, 0.646360 * ray_dir.y * (-1.0f + 5.0f * ray_dir2.z));
    SH_ACCUM(12, 0.373176 * (5.0f * ray_dir2.z * ray_dir.z - 3.0f * ray_dir.z));
    SH_ACCUM(13, 0.457045 * ray_dir.x * (-1.0f + 5.0f * ray_dir2.z));
    SH_ACCUM(14, 1.445305 * (ray_dir2.x - ray_dir2.y) * ray_dir.z);
    SH_ACCUM(15, 0.590043 * ray_dir.x * (ray_dir2.x - 3.0f * ray_dir2.y));

#endif
  }

  for (uint i = 0; i < SH_SIZE; i++) {
    // store in history texture
    int3 prev_pos = int3(pos.x, pos.y * SH_SIZE + i, int(params.history_index));
    int2 average_pos = prev_pos.xy;

    float4 value = float4(sh_accum[probe_index].c[i * 3 + 0], sh_accum[probe_index].c[i * 3 + 1], sh_accum[probe_index].c[i * 3 + 2], 1.0) * 4.0 / float(params.ray_count);

    int4 ivalue = clamp(int4(value * float(1 << HISTORY_BITS)), -32768, 32767); //clamp to 16 bits, so higher values don't break average

    int4 prev_value = lightprobe_history_texture[prev_pos];
    int4 average = lightprobe_average_texture[average_pos];

    average -= prev_value;
    average += ivalue;

    lightprobe_history_texture[prev_pos] = ivalue;
    lightprobe_average_texture[average_pos] = average;

    if (params.store_ambient_texture != 0 && i == 0) {
      int3 ambient_pos = int3(pos, int(params.cascade));
      float4 ambient_light = (float4(average) / float(params.history_size)) / float(1 << HISTORY_BITS);
      ambient_light *= 0.88622; // SHL0
      lightprobe_ambient_texture[ambient_pos] = ambient_light;
    }
  }
#endif // MODE PROCESS

#ifdef MODE_STORE

  // converting to octahedral in this step is required because
  // octahedral is much faster to read from the screen than spherical harmonics,
  // despite the very slight quality loss

  int3 store_probe = SdfgiProbeCell(pos / OCT_SIZE, params.probe_axis_size, params.image_size.x / OCT_SIZE);
  if (store_probe.y > int(params.grid_size.y / (params.grid_size.x / float(params.probe_axis_size - 1)))) return;
  int2 sh_pos = (pos / OCT_SIZE) * int2(1, SH_SIZE);
  int2 oct_pos = (pos / OCT_SIZE) * (OCT_SIZE + 2) + int2(1);
  int2 local_pos = pos % OCT_SIZE;

  //compute the octahedral normal for this texel
  float3 normal = octahedron_encode(float2(local_pos) / float(OCT_SIZE));

  // read the spherical harmonic

  float3 normal2 = normal * normal;
  float c[SH_SIZE] = {

      0.282095, //l0
      0.488603 * normal.y, //l1n1
      0.488603 * normal.z, //l1n0
      0.488603 * normal.x, //l1p1
      1.092548 * normal.x * normal.y, //l2n2
      1.092548 * normal.y * normal.z, //l2n1
      0.315392 * (3.0 * normal2.z - 1.0), //l20
      1.092548 * normal.x * normal.z, //l2p1
      0.546274 * (normal2.x - normal2.y) //l2p2
#if (SH_SIZE == 16)
      ,
      0.590043 * normal.y * (3.0f * normal2.x - normal2.y),
      2.890611 * normal.y * normal.x * normal.z,
      0.646360 * normal.y * (-1.0f + 5.0f * normal2.z),
      0.373176 * (5.0f * normal2.z * normal.z - 3.0f * normal.z),
      0.457045 * normal.x * (-1.0f + 5.0f * normal2.z),
      1.445305 * (normal2.x - normal2.y) * normal.z,
      0.590043 * normal.x * (normal2.x - 3.0f * normal2.y)

#endif
  };

  static const float l_mult[SH_SIZE] = {
      1.0,
      2.0 / 3.0,
      2.0 / 3.0,
      2.0 / 3.0,
      1.0 / 4.0,
      1.0 / 4.0,
      1.0 / 4.0,
      1.0 / 4.0,
      1.0 / 4.0
#if (SH_SIZE == 16)
      , // l4 does not contribute to irradiance
      0.0,
      0.0,
      0.0,
      0.0,
      0.0,
      0.0,
      0.0
#endif
  };

  float3 irradiance = float3(0.0);
  float3 radiance = float3(0.0);

  for (uint i = 0; i < SH_SIZE; i++) {
    // store in history texture
    int2 average_pos = sh_pos + int2(0, i);
    int4 average = lightprobe_average_texture[average_pos];

    float4 sh = (float4(average) / float(params.history_size)) / float(1 << HISTORY_BITS);

    float3 m = sh.rgb * c[i] * 4.0;

    irradiance += m * l_mult[i];
    radiance += m;
  }

  //encode RGBE9995 for the final texture

  uint irradiance_rgbe = rgbe_encode(irradiance);
  uint radiance_rgbe = rgbe_encode(radiance);

  //store in octahedral map

  int3 texture_pos = int3(oct_pos, int(params.cascade));
  int3 copy_to[4] = {int3(-2, -2, -2), int3(-2, -2, -2), int3(-2, -2, -2), int3(-2, -2, -2)};
  copy_to[0] = texture_pos + int3(local_pos, 0);

  if (all(local_pos == int2(0, 0))) {
    copy_to[1] = texture_pos + int3(OCT_SIZE - 1, -1, 0);
    copy_to[2] = texture_pos + int3(-1, OCT_SIZE - 1, 0);
    copy_to[3] = texture_pos + int3(OCT_SIZE, OCT_SIZE, 0);
  } else if (all(local_pos == int2(OCT_SIZE - 1, 0))) {
    copy_to[1] = texture_pos + int3(0, -1, 0);
    copy_to[2] = texture_pos + int3(OCT_SIZE, OCT_SIZE - 1, 0);
    copy_to[3] = texture_pos + int3(-1, OCT_SIZE, 0);
  } else if (all(local_pos == int2(0, OCT_SIZE - 1))) {
    copy_to[1] = texture_pos + int3(-1, 0, 0);
    copy_to[2] = texture_pos + int3(OCT_SIZE - 1, OCT_SIZE, 0);
    copy_to[3] = texture_pos + int3(OCT_SIZE, -1, 0);
  } else if (all(local_pos == int2(OCT_SIZE - 1, OCT_SIZE - 1))) {
    copy_to[1] = texture_pos + int3(0, OCT_SIZE, 0);
    copy_to[2] = texture_pos + int3(OCT_SIZE, 0, 0);
    copy_to[3] = texture_pos + int3(-1, -1, 0);
  } else if (local_pos.y == 0) {
    copy_to[1] = texture_pos + int3(OCT_SIZE - local_pos.x - 1, local_pos.y - 1, 0);
  } else if (local_pos.x == 0) {
    copy_to[1] = texture_pos + int3(local_pos.x - 1, OCT_SIZE - local_pos.y - 1, 0);
  } else if (local_pos.y == OCT_SIZE - 1) {
    copy_to[1] = texture_pos + int3(OCT_SIZE - local_pos.x - 1, local_pos.y + 1, 0);
  } else if (local_pos.x == OCT_SIZE - 1) {
    copy_to[1] = texture_pos + int3(local_pos.x + 1, OCT_SIZE - local_pos.y - 1, 0);
  }

  for (int i = 0; i < 4; i++) {
    if (all(copy_to[i] == int3(-2, -2, -2))) {
      continue;
    }
    lightprobe_texture_data[copy_to[i]] = (uint4(irradiance_rgbe)).x;
    lightprobe_texture_data[copy_to[i] + int3(0, 0, int(params.max_cascades))] = (uint4(radiance_rgbe)).x;
  }

#endif

}
#elif defined(MODE_SCROLL) || defined(MODE_SCROLL_STORE)
[numthreads(8, 8, 1)]
void main(uint3 id : SV_DispatchThreadID) {
  int2 pos = int2(id.xy);
  if (any(pos >= params.image_size) || field_status[0].failure_flags != 0) return;
#ifdef MODE_SCROLL
  int3 probe_cell = SdfgiProbeCell(pos, params.probe_axis_size, params.image_size.x);
  int3 probe_size = int3(params.grid_size / (params.grid_size.x / float(params.probe_axis_size - 1))) + 1;
  if (any(probe_cell >= probe_size)) return;
  int3 read_probe = probe_cell - params.scroll;
  bool retained = all(read_probe >= 0) && all(read_probe < probe_size);
  if ((params.sky_flags & 8) != 0) {
    probe_placement_scroll[SdfgiPlacementIndex(probe_cell, probe_size, 0)] = retained ?
        probe_placement[SdfgiPlacementIndex(read_probe, probe_size, params.cascade)] : float4(0);
    if (!retained) {
      for (int i = 0; i < SH_SIZE; ++i) {
        lightprobe_average_scroll_texture[int2(pos.x, pos.y * SH_SIZE + i)] = int4(0);
        for (uint h = 0; h < params.history_size; ++h)
          lightprobe_history_scroll_texture[int3(pos.x, pos.y * SH_SIZE + i, h)] = int4(0);
      }
      return;
    }
  }
  if (retained || params.cascade == params.max_cascades - 1) {
    // Godot keeps the destination's old value at the highest cascade's entering edge.
    int2 tex_pos = retained ? SdfgiProbeTexel(read_probe, params.probe_axis_size, params.image_size.x) : pos;
    for (uint h = 0; h < params.history_size; ++h)
      for (int i = 0; i < SH_SIZE; ++i)
        lightprobe_history_scroll_texture[int3(pos.x, pos.y * SH_SIZE + i, h)] =
            lightprobe_history_texture[int3(tex_pos.x, tex_pos.y * SH_SIZE + i, h)];
    for (int i = 0; i < SH_SIZE; ++i)
      lightprobe_average_scroll_texture[int2(pos.x, pos.y * SH_SIZE + i)] =
          lightprobe_average_texture[int2(tex_pos.x, tex_pos.y * SH_SIZE + i)];
  } else {
    float cell_to_probe = float(params.grid_size.x) / float(params.probe_axis_size - 1);
    float3 probe_pos = cascades.data[params.cascade].offset + float3(probe_cell) *
        (cell_to_probe / cascades.data[params.cascade].to_cell);
    probe_pos = (probe_pos - cascades.data[params.cascade + 1].offset) /
        (cell_to_probe / cascades.data[params.cascade + 1].to_cell);
    int3 probe_posi = int3(probe_pos);
    float4 average_light[SH_SIZE];
    for (int i = 0; i < SH_SIZE; ++i) average_light[i] = 0;
    float total_weight = 0;
    for (int i = 0; i < 8; ++i) {
      int3 offset = probe_posi + ((int3(i) >> int3(0, 1, 2)) & 1);
      float3 trilinear = 1.0 - abs(probe_pos - float3(offset));
      float weight = trilinear.x * trilinear.y * trilinear.z;
      if (any(offset < 0) || any(offset >= probe_size)) continue;
      int2 tex_pos = SdfgiProbeTexel(offset, params.probe_axis_size, params.image_size.x);
      for (int j = 0; j < SH_SIZE; ++j)
        average_light[j] += (float4(lightprobe_average_parent_texture[int2(tex_pos.x, tex_pos.y * SH_SIZE + j)]) /
            float(params.history_size) / float(1 << HISTORY_BITS)) * weight;
      total_weight += weight;
    }
    if (total_weight > 0) total_weight = 1.0 / total_weight;
    for (int i = 0; i < SH_SIZE; ++i) {
      int4 value = clamp(int4(average_light[i] * total_weight * float(1 << HISTORY_BITS)), int4(-32768), int4(32767));
      for (uint h = 0; h < params.history_size; ++h)
        lightprobe_history_scroll_texture[int3(pos.x, pos.y * SH_SIZE + i, h)] = value;
      lightprobe_average_scroll_texture[int2(pos.x, pos.y * SH_SIZE + i)] = value * int(params.history_size);
    }
  }
#else
  if ((params.sky_flags & 8) != 0) {
    int3 size = int3(params.grid_size / (params.grid_size.x / float(params.probe_axis_size - 1))) + 1;
    int3 cell = SdfgiProbeCell(pos, params.probe_axis_size, params.image_size.x);
    if (any(cell >= size)) return;
    probe_placement[SdfgiPlacementIndex(cell, size, params.cascade)] = probe_placement_scroll[SdfgiPlacementIndex(cell, size, 0)];
  }
  for (uint h = 0; h < params.history_size; ++h)
    for (int i = 0; i < SH_SIZE; ++i) {
      int3 p = int3(pos.x, pos.y * SH_SIZE + i, h);
      lightprobe_history_texture[p] = lightprobe_history_scroll_texture[p];
    }
  for (int i = 0; i < SH_SIZE; ++i) {
    int2 p = int2(pos.x, pos.y * SH_SIZE + i);
    lightprobe_average_texture[p] = lightprobe_average_scroll_texture[p];
  }
#endif
}
#elif defined(EE_SDFGI_ABI_ONLY)
[numthreads(8, 8, 1)]
void main(uint3 id : SV_DispatchThreadID) {}
#else
#error Unknown SDFGI integration mode
#endif
