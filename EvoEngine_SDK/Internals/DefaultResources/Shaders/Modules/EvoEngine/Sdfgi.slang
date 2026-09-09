// Deferred probe gather and sharp SDF tracing from Godot shaders/environment/gi.glsl,
// 34d06658a85845111a50db9e485ec4a0701d4298. See docs/licenses/Godot-MIT.txt.
// Shared-anchor coordinates and publication checks are EvoEngine host adapters.
import EvoEngine.SdfgiTypes;

[[vk::binding(0, 5)]] ConstantBuffer<SdfgiGatherData> sdfgi;
[[vk::binding(1, 5)]] Texture2DArray<float3> sdfgi_lightprobe_texture;
[[vk::binding(2, 5)]] Texture3D<float4> sdfgi_occlusion_cascades;
[[vk::binding(3, 5)]] SamplerState sdfgi_sampler;
[[vk::binding(4, 5)]] StructuredBuffer<SdfgiFieldStatus> sdfgi_status;
[[vk::binding(5, 5)]] Texture3D<float> sdf_cascades[8];
[[vk::binding(6, 5)]] Texture3D<float3> light_cascades[8];
static const uint SDFGI_OCT_SIZE = 6;

float2 EeSdfgiOctWrap(float2 v) {
  float2 signVal;
  signVal.x = v.x >= 0.0 ? 1.0 : -1.0;
  signVal.y = v.y >= 0.0 ? 1.0 : -1.0;
  return (1.0 - abs(v.yx)) * signVal;
}

float2 EeSdfgiOctEncode(float3 n) {
  // https://twitter.com/Stubbesaurus/status/937994790553227264
  n /= (abs(n.x) + abs(n.y) + abs(n.z));
  n.xy = n.z >= 0.0 ? n.xy : EeSdfgiOctWrap(n.xy);
  n.xy = n.xy * 0.5 + 0.5;
  return n.xy;
}

void EeSdfgiProcess(uint cascade, float3 cascade_pos, float3 cam_pos, float3 cam_normal, float3 cam_specular_normal, float roughness, out float3 diffuse_light, out float3 specular_light, bool include_specular) {
  cascade_pos += cam_normal * sdfgi.normal_bias;

  float3 base_pos = floor(cascade_pos);
  int3 probe_base_pos = int3(base_pos);

  float4 diffuse_accum = float4(0.0);
  float3 specular_accum;

  uint atlas_width, atlas_height, atlas_layers;
  sdfgi_lightprobe_texture.GetDimensions(atlas_width, atlas_height, atlas_layers);

  specular_accum = float3(0.0);

  for (uint j = 0; j < 8; j++) {
    int3 offset = (int3(j) >> int3(0, 1, 2)) & int3(1, 1, 1);
    int3 probe_posi = probe_base_pos;
    probe_posi += offset;
    if (any(probe_posi < 0) || any(probe_posi > int3(sdfgi.cascade_probe_size))) continue;
    int2 tile = SdfgiProbeTexel(probe_posi, sdfgi.probe_axis_size, atlas_width / 8);
    float3 tex_pos = float3(tile * 8 + 1, cascade);

    // Compute weight

    float3 probe_pos = float3(probe_posi);

    float3 probe_to_pos = cascade_pos - probe_pos;
    float3 probe_dir = normalize(-probe_to_pos);

    float3 trilinear = float3(1.0) - abs(cascade_pos - float3(probe_posi));
    float weight = trilinear.x * trilinear.y * trilinear.z * max(0.005, dot(cam_normal, probe_dir));

    // Compute lightprobe occlusion

    if (sdfgi.use_occlusion != 0) {
      int3 occ_indexv = abs((sdfgi.cascades[cascade].probe_world_offset + probe_posi) & int3(1, 1, 1)) * int3(1, 2, 4);
      float4 occ_mask = select(int4(occ_indexv.x | occ_indexv.y) == int4(0, 1, 2, 3), float4(1.0), float4(0.0));

      float3 occ_pos = clamp(cascade_pos, probe_pos - sdfgi.occlusion_clamp, probe_pos + sdfgi.occlusion_clamp) / sdfgi.cascade_probe_size;
      occ_pos.z += float(cascade);
      if (occ_indexv.z != 0) { //z bit is on, means index is >=4, so make it switch to the other half of textures
        occ_pos.x += 1.0;
      }

      occ_pos *= sdfgi.occlusion_renormalize;
      float occlusion = dot(sdfgi_occlusion_cascades.SampleLevel(sdfgi_sampler, occ_pos, 0.0), occ_mask);

      weight *= max(occlusion, 0.01);
    }

    // Compute lightprobe texture position

    float3 diffuse;
    float3 pos_uvw = (tex_pos + float3(EeSdfgiOctEncode(cam_normal) * float(SDFGI_OCT_SIZE), 0)) * sdfgi.lightprobe_tex_pixel_size;
    diffuse = sdfgi_lightprobe_texture.SampleLevel(sdfgi_sampler, pos_uvw, 0.0).rgb;

    diffuse_accum += float4(diffuse * weight * sdfgi.cascades[cascade].exposure_normalization, weight);

    if (include_specular) {
      float3 specular = float3(0.0);
      float3 pos_uvw = (tex_pos + float3(EeSdfgiOctEncode(cam_specular_normal) * float(SDFGI_OCT_SIZE), 0)) * sdfgi.lightprobe_tex_pixel_size;
      if (roughness < 0.99) {
        specular = sdfgi_lightprobe_texture.SampleLevel(sdfgi_sampler, pos_uvw + float3(0, 0, float(sdfgi.max_cascades)), 0.0).rgb;
      }
      if (roughness > 0.2) {
        specular = lerp(specular, sdfgi_lightprobe_texture.SampleLevel(sdfgi_sampler, pos_uvw, 0.0).rgb, (roughness - 0.2) * 1.25);
      }

      specular_accum += specular * weight * sdfgi.cascades[cascade].exposure_normalization;
    }
  }

  if (diffuse_accum.a > 0.0) {
    diffuse_accum.rgb /= diffuse_accum.a;
  }

  diffuse_light = diffuse_accum.rgb;

  if (diffuse_accum.a > 0.0) {
    specular_accum /= diffuse_accum.a;
  }

  specular_light = specular_accum;
}

void EeSdfgiReferenceGather(float3 vertex, float3 normal, float3 reflection, float roughness, out float4 ambient_light, out float4 reflection_light, bool include_specular) {
  vertex.y *= sdfgi.y_mult;
  normal.y *= sdfgi.y_mult;
  reflection.y *= sdfgi.y_mult;

  //renormalize
  normal = normalize(normal);
  reflection = normalize(reflection);

  float3 cam_pos = vertex - sdfgi.anchor_origin;
  float3 cam_normal = normal;

  uint cascade = 0xFFFFFFFF;
  float3 cascade_pos;

  for (uint i = 0; i < sdfgi.max_cascades; i++) {
    cascade_pos = (cam_pos - sdfgi.cascades[i].position) * sdfgi.cascades[i].to_probe;

    if (any(cascade_pos < float3(0.0)) || any(cascade_pos >= sdfgi.cascade_probe_size)) {
      continue; //skip cascade
    }

    cascade = i;
    break;
  }

  if (cascade < SDFGI_MAX_CASCADES) {
    ambient_light = float4(0, 0, 0, 1);
    reflection_light = float4(0, 0, 0, 1);

    float blend;
    float3 diffuse, specular;
    EeSdfgiProcess(cascade, cascade_pos, cam_pos, cam_normal, reflection, roughness, diffuse, specular, include_specular);
    ambient_light.a = reflection_light.a = 1.0;

    {
      //process blend
      float3 blend_from = sdfgi.cascade_probe_size * 0.5 - 2.5;
      float3 inner_pos = abs(cam_pos * sdfgi.cascades[cascade].to_probe);
      float3 axis_blend = smoothstep(blend_from, blend_from + 2.0, inner_pos);
      blend = max(axis_blend.x, max(axis_blend.y, axis_blend.z));
    }

    if (blend > 0.0) {
      //blend
      if (cascade == sdfgi.max_cascades - 1) {
        ambient_light.a *= 1.0 - blend;
        reflection_light.a *= 1.0 - blend;

      } else {
        float3 diffuse2, specular2;
        cascade_pos = (cam_pos - sdfgi.cascades[cascade + 1].position) * sdfgi.cascades[cascade + 1].to_probe;
        EeSdfgiProcess(cascade + 1, cascade_pos, cam_pos, cam_normal, reflection, roughness, diffuse2, specular2, include_specular);
        diffuse = lerp(diffuse, diffuse2, blend);
        specular = lerp(specular, specular2, blend);
      }
    }

    ambient_light.rgb = diffuse;

    // Reflection captures reuse diffuse GI, never GI reflections or sharp tracing.
    if (!include_specular) {
      ambient_light.rgb *= sdfgi.energy;
      reflection_light = float4(0.0);
      return;
    }

    if (roughness < 0.2) {
      float3 pos_to_uvw = 1.0 / sdfgi.grid_size;
      float4 light_accum = float4(0.0);

      float blend_size = (sdfgi.grid_size.x / float(sdfgi.probe_axis_size - 1)) * 0.5;

      float radius_sizes[SDFGI_MAX_CASCADES];
      cascade = 0xFFFF;

      float3 radius_scale = (sdfgi.grid_size - blend_size * 2.0) / (sdfgi.grid_size.y - blend_size * 2.0);
      float base_distance = length(cam_pos / radius_scale);
      for (uint i = 0; i < sdfgi.max_cascades; i++) {
        radius_sizes[i] = (1.0 / sdfgi.cascades[i].to_cell) * (sdfgi.grid_size.y * 0.5 - blend_size);
        if (cascade == 0xFFFF && base_distance < radius_sizes[i]) {
          cascade = i;
        }
      }

      cascade = min(cascade, sdfgi.max_cascades - 1);

      float max_distance = radius_sizes[sdfgi.max_cascades - 1];
      float3 ray_pos = cam_pos;
      float3 ray_dir = reflection;

      {
        float prev_radius = cascade > 0 ? radius_sizes[cascade - 1] : 0.0;
        float base_blend = (base_distance - prev_radius) / (radius_sizes[cascade] - prev_radius);
        float bias = (1.0 + base_blend) * 1.1;
        float3 abs_ray_dir = abs(ray_dir);
        ray_pos += (ray_dir * 1.0 / max(abs_ray_dir.x, max(abs_ray_dir.y, abs_ray_dir.z)) + cam_normal * 1.4) * bias / sdfgi.cascades[cascade].to_cell;
      }
      float softness = 0.2 + min(1.0, roughness * 5.0) * 4.0; //approximation to roughness so it does not seem like a hard fade
      uint i = 0;
      bool found = false;
      while (true) {
        if (length(ray_pos / radius_scale) >= max_distance || light_accum.a > 0.99) {
          break;
        }
        if (!found && i >= cascade && length(ray_pos / radius_scale) < radius_sizes[i]) {
          uint next_i = min(i + 1, sdfgi.max_cascades - 1);
          cascade = max(i, cascade); //never go down

          float3 pos = ray_pos - sdfgi.cascades[i].position;
          pos *= sdfgi.cascades[i].to_cell * pos_to_uvw;

          // Sharp reflections can select different cascades within a subgroup.
          float fdistance = sdf_cascades[NonUniformResourceIndex(i)].SampleLevel(sdfgi_sampler, pos, 0.0).r * 255.0 - 1.1;

          float4 hit_light = float4(0.0);
          if (fdistance < softness) {
            hit_light.rgb = light_cascades[NonUniformResourceIndex(i)].SampleLevel(sdfgi_sampler, pos, 0.0).rgb;
            hit_light.rgb *= 0.5; //approximation given value read is actually meant for anisotropy
            hit_light.a = clamp(1.0 - (fdistance / softness), 0.0, 1.0);
            hit_light.rgb *= hit_light.a;
          }

          fdistance /= sdfgi.cascades[i].to_cell;

          if (i < (sdfgi.max_cascades - 1)) {
            pos = ray_pos - sdfgi.cascades[next_i].position;
            pos *= sdfgi.cascades[next_i].to_cell * pos_to_uvw;

            float fdistance2 = sdf_cascades[NonUniformResourceIndex(next_i)].SampleLevel(sdfgi_sampler, pos, 0.0).r * 255.0 - 1.1;

            float4 hit_light2 = float4(0.0);
            if (fdistance2 < softness) {
              hit_light2.rgb = light_cascades[NonUniformResourceIndex(next_i)].SampleLevel(sdfgi_sampler, pos, 0.0).rgb;
              hit_light2.rgb *= 0.5; //approximation given value read is actually meant for anisotropy
              hit_light2.a = clamp(1.0 - (fdistance2 / softness), 0.0, 1.0);
              hit_light2.rgb *= hit_light2.a;
            }

            float prev_radius = i == 0 ? 0.0 : radius_sizes[max(0, i - 1)];
            float blend = clamp((length(ray_pos / radius_scale) - prev_radius) / (radius_sizes[i] - prev_radius), 0.0, 1.0);

            fdistance2 /= sdfgi.cascades[next_i].to_cell;

            hit_light = lerp(hit_light, hit_light2, blend);
            fdistance = lerp(fdistance, fdistance2, blend);
          }

          light_accum += hit_light;
          ray_pos += ray_dir * fdistance;
          found = true;
        }
        i++;
        if (i == sdfgi.max_cascades) {
          i = 0;
          found = false;
        }
      }

      float3 light = light_accum.rgb / max(light_accum.a, 0.00001);
      float alpha = min(1.0, light_accum.a);

      float b = min(1.0, roughness * 5.0);

      float sa = 1.0 - b;

      reflection_light.a = alpha * sa + b;
      if (reflection_light.a == 0) {
        specular = float3(0.0);
      } else {
        specular = (light * alpha * sa + specular * b) / reflection_light.a;
      }
    }

    reflection_light.rgb = specular;

    ambient_light.rgb *= sdfgi.energy;
    reflection_light.rgb *= sdfgi.energy;
  } else {
    ambient_light = float4(0);
    reflection_light = float4(0);
  }
}

public struct EeSdfgiLighting {
  public float3 diffuse;
  public float3 specular;
  public float weight;
  public float specular_weight;
};

public EeSdfgiLighting EE_SDFGI_GATHER(float3 world_position, float3 normal, float3 reflection, float roughness, bool include_specular = true) {
  EeSdfgiLighting result = { float3(0), float3(0), 0, 0 };
  if (sdfgi_status[0].ready == 0 || sdfgi_status[0].failure_flags != 0 ||
      sdfgi_status[0].generation != sdfgi.generation || sdfgi.max_cascades == 0) return result;
  float4 ambient_light, reflection_light;
  EeSdfgiReferenceGather(world_position, normal, reflection, roughness, ambient_light, reflection_light, include_specular);
  result.diffuse = ambient_light.rgb;
  result.specular = reflection_light.rgb;
  result.weight = ambient_light.a;
  result.specular_weight = reflection_light.a;
  return result;
}
