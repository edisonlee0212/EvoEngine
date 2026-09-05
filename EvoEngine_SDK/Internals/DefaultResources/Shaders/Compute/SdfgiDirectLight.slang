// Direct-light injection adapted from Godot sdfgi_direct_light.glsl,
// 34d06658a85845111a50db9e485ec4a0701d4298. See docs/licenses/Godot-MIT.txt.
// Host changes: native photometry, no area/projector family, whole-field capacity guard.
import EvoEngine.SdfgiTypes;


[[vk::binding(1, 0)]] Texture3D<float> sdf_cascades[8];
[[vk::binding(2, 0)]] SamplerState linear_sampler;
[[vk::binding(3, 0)]] SamplerState linear_sampler_with_mipmaps;
[[vk::binding(4, 0)]] StructuredBuffer<SdfgiDispatchData> dispatch_data;
[[vk::binding(5, 0)]] RWStructuredBuffer<SdfgiSolidCell> process_voxels;
[[vk::binding(6, 0)]] [vk::image_format("r32ui")] RWTexture3D<uint> dst_light;
[[vk::binding(7, 0)]] [vk::image_format("rgba8")] RWTexture3D<float4> dst_aniso0;
[[vk::binding(8, 0)]] [vk::image_format("rg8")] RWTexture3D<float2> dst_aniso1;

[[vk::binding(9, 0)]] ConstantBuffer<SdfgiCascadeBlock> cascades;
[[vk::binding(10, 0)]] StructuredBuffer<SdfgiLight> lights;
[[vk::binding(11, 0)]] Texture2DArray<float3> lightprobe_texture;
[[vk::binding(12, 0)]] Texture3D<float4> occlusion_texture;
[[vk::binding(13, 0)]] RWStructuredBuffer<SdfgiFieldStatus> field_status;
[[vk::push_constant]] ConstantBuffer<SdfgiDirectLightPushConstant> params;

#define OCT_SIZE 6
#define LIGHT_TYPE_DIRECTIONAL 0
#define LIGHT_TYPE_OMNI 1
#define LIGHT_TYPE_SPOT 2

#if defined(EE_SDFGI_RESOURCE_CHECK)
[numthreads(64, 1, 1)]
void main(uint3 id : SV_DispatchThreadID) {
  if (any(id != 0)) return;
  uint errors = 0;
  errors |= lights[1].host_photometry.w != 13.25 ? 8192u : 0u;
  for (uint c = 0; c < params.max_cascades; ++c) {
    float3 light = lightprobe_texture.SampleLevel(linear_sampler, float3(0.5, 0.5, c), 0);
    errors |= any(abs(light - 1.0) > 0.001) ? 1u : 0u;
  }
  float4 occlusion = occlusion_texture.SampleLevel(linear_sampler, 0.5, 0);
  errors |= any(abs(occlusion - float4(1, 0, 10.0 / 15.0, 5.0 / 15.0)) > 0.001) ? 2u : 0u;
  field_status[0].failure_flags |= errors;
  field_status[0].generation = asuint(lights[1].host_photometry.w);
}

#elif defined(MODE_PROCESS_STATIC) || defined(MODE_PROCESS_DYNAMIC)
float2 octahedron_wrap(float2 v) {
  float2 signVal;
  signVal.x = v.x >= 0.0 ? 1.0 : -1.0;
  signVal.y = v.y >= 0.0 ? 1.0 : -1.0;
  return (1.0 - abs(v.yx)) * signVal;
}

float2 octahedron_encode(float3 n) {
  // https://twitter.com/Stubbesaurus/status/937994790553227264
  n /= (abs(n.x) + abs(n.y) + abs(n.z));
  n.xy = n.z >= 0.0 ? n.xy : octahedron_wrap(n.xy);
  n.xy = n.xy * 0.5 + 0.5;
  return n.xy;
}

[numthreads(64, 1, 1)]
void main(uint3 id : SV_DispatchThreadID) {
  if (field_status[0].failure_flags != 0) return;
  uint voxel_index = id.x;

  //used for skipping voxels every N frames
  if (params.process_increment > 1) {
    voxel_index *= params.process_increment;
    voxel_index += params.process_offset;
  }

  if (voxel_index >= min(dispatch_data[0].total_count, field_status[0].solid_cell_capacity)) {
    return;
  }

  uint voxel_position = process_voxels[voxel_index].position;

  //keep for storing to texture
  int3 positioni = int3((uint3(voxel_position, voxel_position, voxel_position) >> uint3(0, 7, 14)) & uint3(0x7F));

  float3 position = float3(positioni) + float3(0.5);
  position /= cascades.data[params.cascade].to_cell;
  position += cascades.data[params.cascade].offset;

  uint voxel_albedo = process_voxels[voxel_index].albedo;

  float3 albedo = float3(uint3(voxel_albedo >> 10, voxel_albedo >> 5, voxel_albedo) & uint3(0x1F)) / float(0x1F);
  float3 light_accum[6] = {float3(0.0), float3(0.0), float3(0.0), float3(0.0), float3(0.0), float3(0.0)};
  uint valid_aniso = (voxel_albedo >> 15) & 0x3F;

  static const float3 aniso_dir[6] = {
      float3(1, 0, 0),
      float3(0, 1, 0),
      float3(0, 0, 1),
      float3(-1, 0, 0),
      float3(0, -1, 0),
      float3(0, 0, -1)};

  // Add indirect light first, in order to save computation resources
#ifdef MODE_PROCESS_DYNAMIC
  if (params.bounce_feedback > 0.001) {
    float3 feedback = (params.bounce_feedback < 1.0) ? (albedo * params.bounce_feedback) : lerp(albedo, float3(1.0), params.bounce_feedback - 1.0);
    float3 pos = (float3(positioni) + float3(0.5)) * float(params.probe_axis_size - 1) / params.grid_size;
    int3 probe_base_pos = int3(pos);

    float weight_accum[6] = {0, 0, 0, 0, 0, 0};

    int3 tex_pos = int3(probe_base_pos.xy, int(params.cascade));
    tex_pos.x += probe_base_pos.z * int(params.probe_axis_size);

    tex_pos.xy = tex_pos.xy * (OCT_SIZE + 2) + int2(1);

    float3 base_tex_posf = float3(tex_pos);
    float2 tex_pixel_size = 1.0 / float2(int2((OCT_SIZE + 2) * params.probe_axis_size * params.probe_axis_size, (OCT_SIZE + 2) * params.probe_axis_size));
    float3 probe_uv_offset = float3(int3(OCT_SIZE + 2, OCT_SIZE + 2, (OCT_SIZE + 2) * params.probe_axis_size)) * tex_pixel_size.xyx;

    for (uint j = 0; j < 8; j++) {
      int3 offset = (int3(j) >> int3(0, 1, 2)) & int3(1, 1, 1);
      int3 probe_posi = probe_base_pos;
      probe_posi += offset;

      // Compute weight

      float3 probe_pos = float3(probe_posi);
      float3 probe_to_pos = pos - probe_pos;
      float3 probe_dir = normalize(-probe_to_pos);

      // Compute lightprobe texture position

      float3 trilinear = float3(1.0) - abs(probe_to_pos);

      for (uint k = 0; k < 6; k++) {
        if (bool(valid_aniso & (1 << k))) {
          float3 n = aniso_dir[k];
          float weight = trilinear.x * trilinear.y * trilinear.z * max(0, dot(n, probe_dir));

          if (weight > 0.0 && params.use_occlusion) {
            int3 occ_indexv = abs((cascades.data[params.cascade].probe_world_offset + probe_posi) & int3(1, 1, 1)) * int3(1, 2, 4);
            float4 occ_mask = select(int4(occ_indexv.x | occ_indexv.y) == int4(0, 1, 2, 3), float4(1.0), float4(0.0));

            float3 occ_pos = (float3(positioni) + aniso_dir[k] + float3(0.5)) / params.grid_size;
            occ_pos.z += float(params.cascade);
            if (occ_indexv.z != 0) { //z bit is on, means index is >=4, so make it switch to the other half of textures
              occ_pos.x += 1.0;
            }
            occ_pos *= float3(0.5, 1.0, 1.0 / float(params.max_cascades)); //renormalize
            float occlusion = dot(occlusion_texture.SampleLevel(linear_sampler, occ_pos, 0.0), occ_mask);

            weight *= occlusion;
          }

          if (weight > 0.0) {
            float3 tex_posf = base_tex_posf + float3(octahedron_encode(n) * float(OCT_SIZE), 0.0);
            tex_posf.xy *= tex_pixel_size;

            float3 pos_uvw = tex_posf;
            pos_uvw.xy += float2(offset.xy) * probe_uv_offset.xy;
            pos_uvw.x += float(offset.z) * probe_uv_offset.z;
            float3 indirect_light = lightprobe_texture.SampleLevel(linear_sampler, pos_uvw, 0.0);

            light_accum[k] += indirect_light * weight;
            weight_accum[k] += weight;
          }
        }
      }
    }

    for (uint k = 0; k < 6; k++) {
      if (weight_accum[k] > 0.0) {
        light_accum[k] /= weight_accum[k];
        light_accum[k] *= feedback;
      }
    }
  }

#endif

  {
    uint rgbe = process_voxels[voxel_index].light;

    //read rgbe8985
    float r = float((rgbe & 0xff) << 1);
    float g = float((rgbe >> 8) & 0x1ff);
    float b = float(((rgbe >> 17) & 0xff) << 1);
    float e = float((rgbe >> 25) & 0x1F);
    float m = pow(2.0, e - 15.0 - 9.0);

    float3 l = float3(r, g, b) * m;

    uint aniso = process_voxels[voxel_index].light_aniso;
    for (uint i = 0; i < 6; i++) {
      float strength = ((aniso >> (i * 5)) & 0x1F) / float(0x1F);
      light_accum[i] += l * strength;
    }
  }

  // Raytrace light

  float3 pos_to_uvw = 1.0 / params.grid_size;
  float3 uvw_ofs = pos_to_uvw * 0.5;

  for (uint i = 0; i < params.light_count; i++) {
    float attenuation = 1.0;
    float3 direction;
    float light_distance = 1e20;
    float3 texture_color = float3(1.0);

    switch (lights[i].type) {
      case LIGHT_TYPE_DIRECTIONAL: {
        direction = -lights[i].direction;
      } break;
      case LIGHT_TYPE_OMNI:
      case LIGHT_TYPE_SPOT: {
        float3 rel_vec = lights[i].position - position;
        direction = normalize(rel_vec);
        light_distance = length(rel_vec);
        rel_vec.y /= params.y_mult;
        float world_distance = length(rel_vec);
        if (world_distance > lights[i].radius) continue;
        // Accepted host photometry: native distance coefficients and linear inner/outer cone ramp.
        attenuation = 1.0 / dot(lights[i].host_photometry.xyz, float3(1, world_distance, world_distance * world_distance));
        if (lights[i].type == LIGHT_TYPE_SPOT) {
          float cosine = dot(-normalize(rel_vec), lights[i].direction);
          attenuation *= clamp((cosine - lights[i].cos_spot_angle) /
              (lights[i].host_photometry.w - lights[i].cos_spot_angle), 0.0, 1.0);
        }
      } break;
    }

    if (attenuation < 0.001) {
      continue;
    }

    bool hit = false;

    float3 ray_pos = position;
    float3 ray_dir = direction;
    float3 inv_dir = 1.0 / ray_dir;

    //this is how to properly bias outgoing rays
    float cell_size = 1.0 / cascades.data[params.cascade].to_cell;
    ray_pos += sign(direction) * cell_size * 0.48; // go almost to the box edge but remain inside
    ray_pos += ray_dir * 0.4 * cell_size; //apply a small bias from there

    for (uint j = params.cascade; j < params.max_cascades; j++) {
      //convert to local bounds
      float3 pos = ray_pos - cascades.data[j].offset;
      pos *= cascades.data[j].to_cell;
      float local_distance = light_distance * cascades.data[j].to_cell;

      if (any(pos < float3(0.0)) || any(pos >= params.grid_size)) {
        continue; //already past bounds for this cascade, goto next
      }

      //find maximum advance distance (until reaching bounds)
      float3 t0 = -pos * inv_dir;
      float3 t1 = (params.grid_size - pos) * inv_dir;
      float3 tmax = max(t0, t1);
      float max_advance = min(tmax.x, min(tmax.y, tmax.z));

      max_advance = min(local_distance, max_advance);

      float advance = 0.0;
      float occlusion = 1.0;

      while (advance < max_advance) {
        //read how much to advance from SDF
        float3 uvw = (pos + ray_dir * advance) * pos_to_uvw;

        float distance = sdf_cascades[j].SampleLevel(linear_sampler, uvw, 0.0) * 255.0 - 1.0;
        if (distance < 0.001) {
          //consider hit
          hit = true;
          break;
        }

        occlusion = min(occlusion, distance);

        advance += distance;
      }

      if (hit) {
        attenuation *= occlusion;
        break;
      }

      if (advance >= local_distance) {
        break; //past light distance, abandon search
      }
      //change ray origin to collision with bounds
      pos += ray_dir * max_advance;
      pos /= cascades.data[j].to_cell;
      pos += cascades.data[j].offset;
      light_distance -= max_advance / cascades.data[j].to_cell;
      ray_pos = pos;
    }

    if (!hit) {
      float3 light = albedo * lights[i].color.rgb * texture_color * lights[i].energy * attenuation;

      for (int j = 0; j < 6; j++) {
        if (bool(valid_aniso & (1 << j))) {
          light_accum[j] += max(0.0, dot(aniso_dir[j], direction)) * light;
        }
      }
    }
  }

  // Store the light in the light texture

  float lumas[6];
  float3 light_total = float3(0);

  for (int i = 0; i < 6; i++) {
    light_total += light_accum[i];
    lumas[i] = max(light_accum[i].r, max(light_accum[i].g, light_accum[i].b));
  }

  float luma_total = max(light_total.r, max(light_total.g, light_total.b));

  uint light_total_rgbe;

  {
    //compress to RGBE9995 to save space

    const float pow2to9 = 512.0f;
    const float B = 15.0f;
    const float N = 9.0f;
    const float LN2 = 0.6931471805599453094172321215;

    float cRed = clamp(light_total.r, 0.0, 65408.0);
    float cGreen = clamp(light_total.g, 0.0, 65408.0);
    float cBlue = clamp(light_total.b, 0.0, 65408.0);

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
#ifdef MODE_PROCESS_STATIC
    //since its self-save, use RGBE8985
    light_total_rgbe = ((uint(sRed) & 0x1FF) >> 1) | ((uint(sGreen) & 0x1FF) << 8) | (((uint(sBlue) & 0x1FF) >> 1) << 17) | ((uint(exps) & 0x1F) << 25);

#else
    light_total_rgbe = (uint(sRed) & 0x1FF) | ((uint(sGreen) & 0x1FF) << 9) | ((uint(sBlue) & 0x1FF) << 18) | ((uint(exps) & 0x1F) << 27);
#endif
  }

#ifdef MODE_PROCESS_DYNAMIC

  float4 aniso0;
  aniso0.r = lumas[0] / luma_total;
  aniso0.g = lumas[1] / luma_total;
  aniso0.b = lumas[2] / luma_total;
  aniso0.a = lumas[3] / luma_total;

  float2 aniso1;
  aniso1.r = lumas[4] / luma_total;
  aniso1.g = lumas[5] / luma_total;

  //save to 3D textures
  dst_aniso0[positioni] = aniso0;
  dst_aniso1[positioni] = aniso1;
  dst_light[positioni] = light_total_rgbe;

  //also fill neighbors, so light interpolation during the indirect pass works

  //recover the neighbor list from the leftover bits
  uint neighbors = (voxel_albedo >> 21) | ((voxel_position >> 21) << 11) | ((process_voxels[voxel_index].light >> 30) << 22) | ((process_voxels[voxel_index].light_aniso >> 30) << 24);

  const uint max_neighbours = 26;
  static const int3 neighbour_positions[max_neighbours] = {
      int3(-1, -1, -1),
      int3(-1, -1, 0),
      int3(-1, -1, 1),
      int3(-1, 0, -1),
      int3(-1, 0, 0),
      int3(-1, 0, 1),
      int3(-1, 1, -1),
      int3(-1, 1, 0),
      int3(-1, 1, 1),
      int3(0, -1, -1),
      int3(0, -1, 0),
      int3(0, -1, 1),
      int3(0, 0, -1),
      int3(0, 0, 1),
      int3(0, 1, -1),
      int3(0, 1, 0),
      int3(0, 1, 1),
      int3(1, -1, -1),
      int3(1, -1, 0),
      int3(1, -1, 1),
      int3(1, 0, -1),
      int3(1, 0, 0),
      int3(1, 0, 1),
      int3(1, 1, -1),
      int3(1, 1, 0),
      int3(1, 1, 1)};

  for (uint i = 0; i < max_neighbours; i++) {
    if (bool(neighbors & (1 << i))) {
      int3 neighbour_pos = positioni + neighbour_positions[i];
      dst_light[neighbour_pos] = light_total_rgbe;
      dst_aniso0[neighbour_pos] = aniso0;
      dst_aniso1[neighbour_pos] = aniso1;
    }
  }

#endif

#ifdef MODE_PROCESS_STATIC

  //save back the anisotropic

  uint light = process_voxels[voxel_index].light & (3 << 30);
  light |= light_total_rgbe;
  process_voxels[voxel_index].light = light; //replace

  uint light_aniso = process_voxels[voxel_index].light_aniso & (3 << 30);
  for (int i = 0; i < 6; i++) {
    light_aniso |= min(31, uint((lumas[i] / luma_total) * 31.0)) << (i * 5);
  }

  process_voxels[voxel_index].light_aniso = light_aniso;

#endif
}


#else
#error Select a direct-light variant
#endif
