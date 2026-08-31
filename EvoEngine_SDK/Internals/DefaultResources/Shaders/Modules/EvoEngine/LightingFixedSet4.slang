__exported import EvoEngine.Lighting;

struct EeLightingResources : ILightingResources {
  float3 sampleSky(float3 direction) {
    const int texture_index = EE_CAMERAS[EE_BASIC_CONSTANTS.camera_index].skybox_tex_index;
    return EE_CUBEMAPS[NonUniformResourceIndex(texture_index)].SampleLevel(direction, 0.0f).rgb;
  }

  float3 sampleReflectionProbe(int texture_index, float3 direction, float lod) {
    return EE_CUBEMAPS[NonUniformResourceIndex(texture_index)].SampleLevel(direction, lod).rgb;
  }

  float3 sampleIrradiance(float3 direction) {
    const int texture_index = EE_CAMERAS[EE_BASIC_CONSTANTS.camera_index].irradiance_map_index;
    return EE_CUBEMAPS[NonUniformResourceIndex(texture_index)].SampleLevel(direction, 0.0f).rgb;
  }

  float3 samplePrefiltered(float3 direction, float lod) {
    const int texture_index = EE_CAMERAS[EE_BASIC_CONSTANTS.camera_index].prefiltered_map_index;
    return EE_CUBEMAPS[NonUniformResourceIndex(texture_index)].SampleLevel(direction, lod).rgb;
  }

  uint prefilteredLevelCount() {
    const int texture_index = EE_CAMERAS[EE_BASIC_CONSTANTS.camera_index].prefiltered_map_index;
    uint width;
    uint height;
    uint levels;
    EE_CUBEMAPS[NonUniformResourceIndex(texture_index)].GetDimensions(0, width, height, levels);
    return levels;
  }

  float2 sampleBrdf(float2 uv) {
    return EE_TEXTURE_2DS[NonUniformResourceIndex(EE_RENDER_INFO.brdf_lut_map_index)].SampleLevel(uv, 0.0f).rg;
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
