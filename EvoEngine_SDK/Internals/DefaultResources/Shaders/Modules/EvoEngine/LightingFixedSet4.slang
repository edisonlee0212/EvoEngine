// @evoengine-dialect native
__exported import EvoEngine.Lighting;

[[vk::binding(0, 4)]] Sampler2D EE_RASTER_BRDF_LUT;
[[vk::binding(1, 4)]] SamplerCube EE_RASTER_SKYBOX;
[[vk::binding(2, 4)]] SamplerCube EE_RASTER_IRRADIANCE_MAP;
[[vk::binding(3, 4)]] SamplerCube EE_RASTER_PREFILTERED_MAP;
[[vk::binding(5, 4)]] SamplerCube EE_RASTER_REFLECTION_PROBES[EE_REFLECTION_PROBE_MAX_COUNT * 2];

struct EeLightingResources : ILightingResources {
  float3 sampleSky(float3 direction) {
    return EE_RASTER_SKYBOX.Sample(direction).rgb;
  }

  float3 sampleReflectionProbe(int index, float3 direction, float lod) {
    switch (index) {
      case 0: return EE_RASTER_REFLECTION_PROBES[0].SampleLevel(direction, lod).rgb;
      case 1: return EE_RASTER_REFLECTION_PROBES[1].SampleLevel(direction, lod).rgb;
      case 2: return EE_RASTER_REFLECTION_PROBES[2].SampleLevel(direction, lod).rgb;
      case 3: return EE_RASTER_REFLECTION_PROBES[3].SampleLevel(direction, lod).rgb;
      case 4: return EE_RASTER_REFLECTION_PROBES[4].SampleLevel(direction, lod).rgb;
      case 5: return EE_RASTER_REFLECTION_PROBES[5].SampleLevel(direction, lod).rgb;
      case 6: return EE_RASTER_REFLECTION_PROBES[6].SampleLevel(direction, lod).rgb;
      case 7: return EE_RASTER_REFLECTION_PROBES[7].SampleLevel(direction, lod).rgb;
      case 8: return EE_RASTER_REFLECTION_PROBES[8].SampleLevel(direction, lod).rgb;
      case 9: return EE_RASTER_REFLECTION_PROBES[9].SampleLevel(direction, lod).rgb;
      case 10: return EE_RASTER_REFLECTION_PROBES[10].SampleLevel(direction, lod).rgb;
      case 11: return EE_RASTER_REFLECTION_PROBES[11].SampleLevel(direction, lod).rgb;
      case 12: return EE_RASTER_REFLECTION_PROBES[12].SampleLevel(direction, lod).rgb;
      case 13: return EE_RASTER_REFLECTION_PROBES[13].SampleLevel(direction, lod).rgb;
      case 14: return EE_RASTER_REFLECTION_PROBES[14].SampleLevel(direction, lod).rgb;
      case 15: return EE_RASTER_REFLECTION_PROBES[15].SampleLevel(direction, lod).rgb;
      case 16: return EE_RASTER_REFLECTION_PROBES[16].SampleLevel(direction, lod).rgb;
      case 17: return EE_RASTER_REFLECTION_PROBES[17].SampleLevel(direction, lod).rgb;
      case 18: return EE_RASTER_REFLECTION_PROBES[18].SampleLevel(direction, lod).rgb;
      case 19: return EE_RASTER_REFLECTION_PROBES[19].SampleLevel(direction, lod).rgb;
      case 20: return EE_RASTER_REFLECTION_PROBES[20].SampleLevel(direction, lod).rgb;
      case 21: return EE_RASTER_REFLECTION_PROBES[21].SampleLevel(direction, lod).rgb;
      case 22: return EE_RASTER_REFLECTION_PROBES[22].SampleLevel(direction, lod).rgb;
      case 23: return EE_RASTER_REFLECTION_PROBES[23].SampleLevel(direction, lod).rgb;
      case 24: return EE_RASTER_REFLECTION_PROBES[24].SampleLevel(direction, lod).rgb;
      case 25: return EE_RASTER_REFLECTION_PROBES[25].SampleLevel(direction, lod).rgb;
      case 26: return EE_RASTER_REFLECTION_PROBES[26].SampleLevel(direction, lod).rgb;
      case 27: return EE_RASTER_REFLECTION_PROBES[27].SampleLevel(direction, lod).rgb;
      case 28: return EE_RASTER_REFLECTION_PROBES[28].SampleLevel(direction, lod).rgb;
      case 29: return EE_RASTER_REFLECTION_PROBES[29].SampleLevel(direction, lod).rgb;
      case 30: return EE_RASTER_REFLECTION_PROBES[30].SampleLevel(direction, lod).rgb;
      case 31: return EE_RASTER_REFLECTION_PROBES[31].SampleLevel(direction, lod).rgb;
      case 32: return EE_RASTER_REFLECTION_PROBES[32].SampleLevel(direction, lod).rgb;
      case 33: return EE_RASTER_REFLECTION_PROBES[33].SampleLevel(direction, lod).rgb;
      case 34: return EE_RASTER_REFLECTION_PROBES[34].SampleLevel(direction, lod).rgb;
      case 35: return EE_RASTER_REFLECTION_PROBES[35].SampleLevel(direction, lod).rgb;
      case 36: return EE_RASTER_REFLECTION_PROBES[36].SampleLevel(direction, lod).rgb;
      case 37: return EE_RASTER_REFLECTION_PROBES[37].SampleLevel(direction, lod).rgb;
      case 38: return EE_RASTER_REFLECTION_PROBES[38].SampleLevel(direction, lod).rgb;
      case 39: return EE_RASTER_REFLECTION_PROBES[39].SampleLevel(direction, lod).rgb;
      case 40: return EE_RASTER_REFLECTION_PROBES[40].SampleLevel(direction, lod).rgb;
      case 41: return EE_RASTER_REFLECTION_PROBES[41].SampleLevel(direction, lod).rgb;
      case 42: return EE_RASTER_REFLECTION_PROBES[42].SampleLevel(direction, lod).rgb;
      case 43: return EE_RASTER_REFLECTION_PROBES[43].SampleLevel(direction, lod).rgb;
      case 44: return EE_RASTER_REFLECTION_PROBES[44].SampleLevel(direction, lod).rgb;
      case 45: return EE_RASTER_REFLECTION_PROBES[45].SampleLevel(direction, lod).rgb;
      case 46: return EE_RASTER_REFLECTION_PROBES[46].SampleLevel(direction, lod).rgb;
      case 47: return EE_RASTER_REFLECTION_PROBES[47].SampleLevel(direction, lod).rgb;
      case 48: return EE_RASTER_REFLECTION_PROBES[48].SampleLevel(direction, lod).rgb;
      case 49: return EE_RASTER_REFLECTION_PROBES[49].SampleLevel(direction, lod).rgb;
      case 50: return EE_RASTER_REFLECTION_PROBES[50].SampleLevel(direction, lod).rgb;
      case 51: return EE_RASTER_REFLECTION_PROBES[51].SampleLevel(direction, lod).rgb;
      case 52: return EE_RASTER_REFLECTION_PROBES[52].SampleLevel(direction, lod).rgb;
      case 53: return EE_RASTER_REFLECTION_PROBES[53].SampleLevel(direction, lod).rgb;
      case 54: return EE_RASTER_REFLECTION_PROBES[54].SampleLevel(direction, lod).rgb;
      case 55: return EE_RASTER_REFLECTION_PROBES[55].SampleLevel(direction, lod).rgb;
      case 56: return EE_RASTER_REFLECTION_PROBES[56].SampleLevel(direction, lod).rgb;
      case 57: return EE_RASTER_REFLECTION_PROBES[57].SampleLevel(direction, lod).rgb;
      case 58: return EE_RASTER_REFLECTION_PROBES[58].SampleLevel(direction, lod).rgb;
      case 59: return EE_RASTER_REFLECTION_PROBES[59].SampleLevel(direction, lod).rgb;
      case 60: return EE_RASTER_REFLECTION_PROBES[60].SampleLevel(direction, lod).rgb;
      case 61: return EE_RASTER_REFLECTION_PROBES[61].SampleLevel(direction, lod).rgb;
      case 62: return EE_RASTER_REFLECTION_PROBES[62].SampleLevel(direction, lod).rgb;
      case 63: return EE_RASTER_REFLECTION_PROBES[63].SampleLevel(direction, lod).rgb;
    }
    return float3(0.0f);
  }

  float3 sampleIrradiance(float3 direction) {
    return EE_RASTER_IRRADIANCE_MAP.Sample(direction).rgb;
  }

  float3 samplePrefiltered(float3 direction, float lod) {
    return EE_RASTER_PREFILTERED_MAP.SampleLevel(direction, lod).rgb;
  }

  uint prefilteredLevelCount() {
    uint width;
    uint height;
    uint levels;
    EE_RASTER_PREFILTERED_MAP.GetDimensions(0, width, height, levels);
    return levels;
  }

  float2 sampleBrdf(float2 uv) {
    return EE_RASTER_BRDF_LUT.Sample(uv).rg;
  }
};

public float3 EE_SKY_COLOR(float3 direction) {
  return EE_SKY_COLOR(EeLightingResources(), direction);
}

public EeEnvironmentalLighting EE_FUNC_CALCULATE_ENVIRONMENTAL_COMPONENTS(
    float3 albedo, float3 normal, float3 view_dir, float3 frag_pos, float metallic, float roughness, float3 f0,
    float f90, float material_occlusion, float screen_space_visibility, float ddgi_visibility) {
  return EE_FUNC_CALCULATE_ENVIRONMENTAL_COMPONENTS(
      EeLightingResources(), albedo, normal, view_dir, frag_pos, metallic, roughness, f0, f90, material_occlusion,
      screen_space_visibility, float3(0.0f), 0.0f, ddgi_visibility);
}

public float3 EE_FUNC_CALCULATE_ENVIRONMENTAL_LIGHT(float3 albedo, float3 normal, float3 view_dir, float3 frag_pos,
                                                    float metallic, float roughness, float3 f0, float f90) {
  return EE_FUNC_CALCULATE_ENVIRONMENTAL_LIGHT(EeLightingResources(), albedo, normal, view_dir, frag_pos, metallic,
                                               roughness, f0, f90);
}

public float3 EE_FUNC_CALCULATE_DDGI_ENVIRONMENTAL_LIGHT(
    float3 albedo, float3 normal, float3 view_dir, float3 frag_pos, float metallic, float roughness, float3 f0,
    float f90, float material_occlusion, float screen_space_visibility) {
  return EE_FUNC_CALCULATE_DDGI_ENVIRONMENTAL_LIGHT(
      EeLightingResources(), albedo, normal, view_dir, frag_pos, metallic, roughness, f0, f90, material_occlusion,
      screen_space_visibility);
}
