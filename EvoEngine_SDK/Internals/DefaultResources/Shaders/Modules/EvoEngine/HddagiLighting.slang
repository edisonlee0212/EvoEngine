// Native material/AO/reflection composition for incoming HDDAGI radiance.
__exported import EvoEngine.LightingFixedSet3;
import EvoEngine.HddagiCamera;

public
float3 EE_FUNC_CALCULATE_HDDAGI_ENVIRONMENTAL_LIGHT(uint2 pixel, float3 albedo, float3 normal, float3 viewDir,
                                                    float3 fragPos, float metallic, float roughness, float3 F0,
                                                    float F90, float materialOcclusion, float screenSpaceVisibility) {
  EeLightingResources resources;
  float2 uv = (float2(pixel) + 0.5) / float2(EE_HDDAGI_CAMERA.viewport);
  float2 coverage = EE_HDDAGI_STATUS[2] != 0 && EE_HDDAGI_STATUS[1] == 0
                        ? EE_HDDAGI_CAMERA_BLEND.SampleLevel(EE_HDDAGI_SAMPLER, uv, 0)
                        : float2(0);
  float3 field_diffuse = EE_HDDAGI_CAMERA_DIFFUSE.SampleLevel(EE_HDDAGI_SAMPLER, uv, 0);
  float3 field_specular = EE_HDDAGI_CAMERA_SPECULAR.SampleLevel(EE_HDDAGI_SAMPLER, uv, 0);
  float normalDotView = max(dot(normal, viewDir), 0);
  float3 F = EE_FUNC_FRESNEL_SCHLICK_ROUGHNESS(normalDotView, F0, F90, roughness);
  float3 dielectricF = EE_FUNC_FRESNEL_SCHLICK_ROUGHNESS(normalDotView, EE_RECOVER_DIELECTRIC_F0(albedo, metallic, F0),
                                                         EE_RECOVER_DIELECTRIC_F90(metallic, F90), roughness);
  float3 R = reflect(-viewDir, normal);
  float3 environment_diffuse =
      (EE_ENVIRONMENT.background_color.w == 1 ? EE_ENVIRONMENT.background_color.xyz
                                              : pow(resources.sampleIrradiance(EE_ENVIRONMENT_LOCAL_DIRECTION(normal)),
                                                    float3(1.0 / EE_ENVIRONMENT.gamma))) *
      EE_ENVIRONMENT.diffuse_fallback_intensity;
  bool gtao_available = (EE_CAMERAS[EE_BASIC_CONSTANTS.camera_index].raster_lighting_flags & 1u) != 0;
  float screen_visibility = gtao_available ? screenSpaceVisibility : 1;
  float3 diffuse = (1 - dielectricF) * (1 - saturate(metallic)) * albedo *
                   lerp(environment_diffuse, field_diffuse, coverage.x) *
                   saturate(materialOcclusion * screen_visibility) * EE_RENDER_INFO.indirect_lighting_intensity;
  if (EE_HDDAGI_CAMERA.reflection_capture != 0)
    return diffuse;
  float lod_scale = float(max(int(resources.prefilteredLevelCount()) - 1, 0));
  float3 environment_specular =
      (EE_ENVIRONMENT.background_color.w == 1
           ? EE_ENVIRONMENT.background_color.xyz
           : pow(resources.samplePrefiltered(EE_ENVIRONMENT_LOCAL_DIRECTION(R), roughness * lod_scale),
                 float3(1.0 / EE_ENVIRONMENT.gamma))) *
      EE_ENVIRONMENT.specular_fallback_intensity;
  float3 prefiltered = EE_SPATIAL_REFLECTION_PREFILTERED(resources, fragPos, R, roughness,
                                                         lerp(environment_specular, field_specular, coverage.y));
  float2 brdf = resources.sampleBrdf(float2(normalDotView, roughness));
  float3 unoccluded_specular = prefiltered * (F * brdf.x + float3(F90 * brdf.y));
  float visibility = EE_ROUGH_SPECULAR_VISIBILITY(materialOcclusion, screen_visibility, 1, roughness, normalDotView);
  float3 specular = unoccluded_specular * visibility;
  int debug = EE_INDIRECT_LIGHTING_DEBUG_VIEW();
  if (debug == 1)
    return diffuse;
  if (debug == 2)
    return unoccluded_specular;
  if (debug == 3)
    return float3(visibility);
  if (debug == 4)
    return specular;
  if (debug == 5)
    return float3(coverage.x);
  return diffuse + specular;
}
