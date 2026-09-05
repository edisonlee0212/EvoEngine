// Probe gather from Godot scene_forward_gi_inc.glsl::sdfgi_process and its forward caller,
// 34d06658a85845111a50db9e485ec4a0701d4298. See docs/licenses/Godot-MIT.txt.
// Shared-anchor coordinates and output coverage/energy adapt the host composition (gi.glsl).
import EvoEngine.SdfgiTypes;

[[vk::binding(0, 5)]] ConstantBuffer<SdfgiGatherData> sdfgi;
[[vk::binding(1, 5)]] Texture2DArray<float3> sdfgi_lightprobe_texture;
[[vk::binding(2, 5)]] Texture3D<float4> sdfgi_occlusion_cascades;
[[vk::binding(3, 5)]] SamplerState sdfgi_sampler;
[[vk::binding(4, 5)]] StructuredBuffer<SdfgiFieldStatus> sdfgi_status;
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

void EeSdfgiProcess(uint cascade, float3 cascade_pos, float3 cam_pos, float3 cam_normal, float3 cam_specular_normal, bool use_specular, float roughness, out float3 diffuse_light, out float3 specular_light, out float blend) {
  cascade_pos += cam_normal * sdfgi.normal_bias;

  float3 base_pos = floor(cascade_pos);
  //cascade_pos += lerp(float3(0.0),float3(0.01),lessThan(abs(cascade_pos-base_pos),float3(0.01))) * cam_normal;
  int3 probe_base_pos = int3(base_pos);

  float4 diffuse_accum = float4(0.0);
  float3 specular_accum = float3(0);

  int3 tex_pos = int3(probe_base_pos.xy, int(cascade));
  tex_pos.x += probe_base_pos.z * sdfgi.probe_axis_size;
  tex_pos.xy = tex_pos.xy * (SDFGI_OCT_SIZE + 2) + int2(1);

  float3 diffuse_posf = (float3(tex_pos) + float3(EeSdfgiOctEncode(cam_normal) * float(SDFGI_OCT_SIZE), 0.0)) * sdfgi.lightprobe_tex_pixel_size;

  float3 specular_posf;

  if (use_specular) {
    specular_accum = float3(0.0);
    specular_posf = (float3(tex_pos) + float3(EeSdfgiOctEncode(cam_specular_normal) * float(SDFGI_OCT_SIZE), 0.0)) * sdfgi.lightprobe_tex_pixel_size;
  }

  float4 light_accum = float4(0.0);
  float weight_accum = 0.0;

  for (uint j = 0; j < 8; j++) {
    int3 offset = (int3(j) >> int3(0, 1, 2)) & int3(1, 1, 1);
    int3 probe_posi = probe_base_pos;
    probe_posi += offset;

    // Compute weight

    float3 probe_pos = float3(probe_posi);
    float3 probe_to_pos = cascade_pos - probe_pos;
    float3 probe_dir = normalize(-probe_to_pos);

    float3 trilinear = float3(1.0) - abs(probe_to_pos);
    float weight = trilinear.x * trilinear.y * trilinear.z * max(0.005, dot(cam_normal, probe_dir));

    // Compute lightprobe occlusion

    if (sdfgi.use_occlusion) {
      int3 occ_indexv = abs((sdfgi.cascades[cascade].probe_world_offset + probe_posi) & int3(1, 1, 1)) * int3(1, 2, 4);
      float4 occ_mask = select(int4(occ_indexv.x | occ_indexv.y) == int4(0, 1, 2, 3), float4(1.0), float4(0.0));

      float3 occ_pos = clamp(cascade_pos, probe_pos - sdfgi.occlusion_clamp, probe_pos + sdfgi.occlusion_clamp) * sdfgi.probe_to_uvw;
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
    float3 pos_uvw = diffuse_posf;
    pos_uvw.xy += float2(offset.xy) * sdfgi.lightprobe_uv_offset.xy;
    pos_uvw.x += float(offset.z) * sdfgi.lightprobe_uv_offset.z;
    diffuse = sdfgi_lightprobe_texture.SampleLevel(sdfgi_sampler, pos_uvw, 0.0).rgb;

    diffuse_accum += float4(diffuse * weight * sdfgi.cascades[cascade].exposure_normalization, weight);

    if (use_specular) {
      float3 specular = float3(0.0);
      float3 pos_uvw = specular_posf;
      pos_uvw.xy += float2(offset.xy) * sdfgi.lightprobe_uv_offset.xy;
      pos_uvw.x += float(offset.z) * sdfgi.lightprobe_uv_offset.z;
      if (roughness < 0.99) {
        specular = sdfgi_lightprobe_texture.SampleLevel(sdfgi_sampler, pos_uvw + float3(0, 0, float(sdfgi.max_cascades)), 0.0).rgb;
      }
      if (roughness > 0.5) {
        specular = lerp(specular, sdfgi_lightprobe_texture.SampleLevel(sdfgi_sampler, pos_uvw, 0.0).rgb, (roughness - 0.5) * 2.0);
      }

      specular_accum += specular * weight * sdfgi.cascades[cascade].exposure_normalization;
    }
  }

  if (diffuse_accum.a > 0.0) {
    diffuse_accum.rgb /= diffuse_accum.a;
  }

  diffuse_light = diffuse_accum.rgb;

  if (use_specular) {
    if (diffuse_accum.a > 0.0) {
      specular_accum /= diffuse_accum.a;
    }

    specular_light = specular_accum;
  }

  {
    //process blend
    float blend_from = (float(sdfgi.probe_axis_size - 1) / 2.0) - 2.5;
    float blend_to = blend_from + 2.0;

    float3 inner_pos = cam_pos * sdfgi.cascades[cascade].to_probe;

    float len = length(inner_pos);

    inner_pos = abs(normalize(inner_pos));
    len *= max(inner_pos.x, max(inner_pos.y, inner_pos.z));

    if (len >= blend_from) {
      blend = smoothstep(blend_from, blend_to, len);
    } else {
      blend = 0.0;
    }
  }
}


public struct EeSdfgiLighting {
  public float3 diffuse;
  public float3 specular;
  public float weight;
};

public EeSdfgiLighting EE_SDFGI_GATHER(float3 world_position, float3 normal, float3 reflection, float roughness) {
  EeSdfgiLighting result = { float3(0), float3(0), 0 };
  if (sdfgi_status[0].ready == 0 || sdfgi_status[0].failure_flags != 0 ||
      sdfgi_status[0].generation != sdfgi.generation || sdfgi.max_cascades == 0) return result;
  float3 cam_pos = world_position * float3(1, sdfgi.y_mult, 1) - sdfgi.anchor_origin;
  normal.y *= sdfgi.y_mult;
  reflection.y *= sdfgi.y_mult;
  normal = normalize(normal);
  reflection = normalize(reflection);
  uint cascade = 0xffffffff;
  float3 cascade_pos;
  for (uint i = 0; i < sdfgi.max_cascades; ++i) {
    cascade_pos = (cam_pos - sdfgi.cascades[i].position) * sdfgi.cascades[i].to_probe;
    if (any(cascade_pos < float3(0)) || any(cascade_pos >= sdfgi.cascade_probe_size)) continue;
    cascade = i;
    break;
  }
  if (cascade == 0xffffffff) return result;
  float blend;
  EeSdfgiProcess(cascade, cascade_pos, cam_pos, normal, reflection, true, roughness, result.diffuse, result.specular, blend);
  result.weight = 1;
  if (blend > 0) {
    if (cascade == sdfgi.max_cascades - 1) {
      result.weight = 1 - blend;
    } else {
      float3 diffuse2, specular2;
      float blend2;
      cascade_pos = (cam_pos - sdfgi.cascades[cascade + 1].position) * sdfgi.cascades[cascade + 1].to_probe;
      EeSdfgiProcess(cascade + 1, cascade_pos, cam_pos, normal, reflection, true, roughness, diffuse2, specular2, blend2);
      result.diffuse = lerp(result.diffuse, diffuse2, blend);
      result.specular = lerp(result.specular, specular2, blend);
    }
  }
  result.diffuse *= sdfgi.energy;
  result.specular *= sdfgi.energy;
  return result;
}
