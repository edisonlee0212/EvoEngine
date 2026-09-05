// Host material/AO/reflection composition for incoming scene-linear SDFGI lighting.
__exported import EvoEngine.LightingFixedSet3;
import EvoEngine.Sdfgi;

public float3 EE_FUNC_CALCULATE_SDFGI_ENVIRONMENTAL_LIGHT(
    float3 albedo, float3 normal, float3 viewDir, float3 fragPos, float metallic, float roughness,
    float3 F0, float F90, float materialOcclusion, float screenSpaceVisibility) {
  EeLightingResources resources;
  const float normalDotView = max(dot(normal, viewDir), 0.0f);
  const float3 F = EE_FUNC_FRESNEL_SCHLICK_ROUGHNESS(normalDotView, F0, F90, roughness);
  const float3 dielectricF = EE_FUNC_FRESNEL_SCHLICK_ROUGHNESS(
      normalDotView, EE_RECOVER_DIELECTRIC_F0(albedo, metallic, F0), EE_RECOVER_DIELECTRIC_F90(metallic, F90), roughness);
  const float3 R = reflect(-viewDir, normal);
  const float3 diffuse_weight = (float3(1) - dielectricF) * (1 - clamp(metallic, 0.0f, 1.0f));
  const EeSdfgiLighting field = EE_SDFGI_GATHER(fragPos, normal, R, roughness);
  const uint sdfgi_debug_view = EE_BASIC_CONSTANTS.meshlet_culling_flags >> 8u;
  if (sdfgi_debug_view == 9u) return float3(1.0f - field.weight, field.weight, 0.0f);
  const float3 environment_diffuse = (EE_ENVIRONMENT.background_color.w == 1.0f
      ? EE_ENVIRONMENT.background_color.xyz
      : pow(resources.sampleIrradiance(EE_ENVIRONMENT_LOCAL_DIRECTION(normal)), float3(1.0f / EE_ENVIRONMENT.gamma))) *
      EE_ENVIRONMENT.diffuse_fallback_intensity;
  const float3 irradiance = lerp(environment_diffuse, field.diffuse, field.weight);
  const bool gtao_available = (EE_CAMERAS[EE_BASIC_CONSTANTS.camera_index].raster_lighting_flags & 1u) != 0u;
  const float screen_visibility = gtao_available ? screenSpaceVisibility : 1.0f;
  const float3 diffuse = diffuse_weight * albedo * irradiance * clamp(materialOcclusion * screen_visibility, 0.0f, 1.0f) *
      EE_RENDER_INFO.indirect_lighting_intensity;
  if (sdfgi_debug_view == 7u) return diffuse_weight * albedo * field.diffuse * field.weight *
      clamp(materialOcclusion * screen_visibility, 0.0f, 1.0f) * EE_RENDER_INFO.indirect_lighting_intensity;
  const float lod_scale = float(max(int(resources.prefilteredLevelCount()) - 1, 0));
  const float3 environment_specular = (EE_ENVIRONMENT.background_color.w == 1.0f
      ? EE_ENVIRONMENT.background_color.xyz
      : pow(resources.samplePrefiltered(EE_ENVIRONMENT_LOCAL_DIRECTION(R), roughness * lod_scale), float3(1.0f / EE_ENVIRONMENT.gamma))) *
      EE_ENVIRONMENT.specular_fallback_intensity;
  const float3 specular_base = lerp(environment_specular, field.specular, field.specular_weight);
  const float3 prefiltered = EE_SPATIAL_REFLECTION_PREFILTERED(resources, fragPos, R, roughness, specular_base);
  const float2 brdf = resources.sampleBrdf(float2(normalDotView, roughness));
  const float3 unoccluded_specular = prefiltered * (F * brdf.x + float3(F90 * brdf.y));
  const float specular_visibility = EE_ROUGH_SPECULAR_VISIBILITY(materialOcclusion, screen_visibility, 1.0f, roughness, normalDotView);
  const float3 specular = unoccluded_specular * specular_visibility;
  if (sdfgi_debug_view == 8u) return field.specular * field.specular_weight *
      (F * brdf.x + float3(F90 * brdf.y)) * specular_visibility;
  const int debug_view = EE_INDIRECT_LIGHTING_DEBUG_VIEW();
  if (debug_view == 1) return diffuse;
  if (debug_view == 2) return unoccluded_specular;
  if (debug_view == 3) return float3(specular_visibility);
  if (debug_view == 4) return specular;
  if (debug_view == 5) return float3(field.weight);
  return diffuse + specular;
}
