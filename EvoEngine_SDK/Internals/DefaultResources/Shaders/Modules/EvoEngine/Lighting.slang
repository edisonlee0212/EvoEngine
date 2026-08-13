// @evoengine-dialect native
__exported import EvoEngine.PerFrame;
__exported import EvoEngine.DDGIGatherMulti;
__exported import EvoEngine.BasicConstants;
import EvoEngine.VogelDisk;

public
interface ILightingResources {
  float3 sampleSky(float3 direction);
  float3 sampleReflectionProbe(int index, float3 direction, float lod);
  float3 sampleIrradiance(float3 direction);
  float3 samplePrefiltered(float3 direction, float lod);
  uint prefilteredLevelCount();
  float2 sampleBrdf(float2 uv);
}

[[vk::binding(14, 2)]]
Sampler2DArrayShadow EE_DIRECTIONAL_LIGHT_SM;
[[vk::binding(15, 2)]]
Sampler2DArray EE_POINT_LIGHT_SM;
[[vk::binding(16, 2)]]
Sampler2D EE_SPOT_LIGHT_SM;

float2 EeDirectionalShadowMapSize() {
  uint width;
  uint height;
  uint elements;
  uint levels;
  EE_DIRECTIONAL_LIGHT_SM.GetDimensions(0, width, height, elements, levels);
  return float2(width, height);
}

float2 EePointShadowMapSize() {
  uint width;
  uint height;
  uint elements;
  uint levels;
  EE_POINT_LIGHT_SM.GetDimensions(0, width, height, elements, levels);
  return float2(width, height);
}

float2 EeSpotShadowMapSize() {
  uint width;
  uint height;
  EE_SPOT_LIGHT_SM.GetDimensions(width, height);
  return float2(width, height);
}

uint EeMaxDirectionalLightsPerCamera() {
  uint light_count;
  uint light_stride;
  uint camera_count;
  uint camera_stride;
  EE_DIRECTIONAL_LIGHTS.GetDimensions(light_count, light_stride);
  EE_CAMERAS.GetDimensions(camera_count, camera_stride);
  return light_count / max(camera_count, 1u);
}

public
float3 EE_SKY_COLOR<T : ILightingResources>(T resources, float3 direction) {
  Camera camera = EE_CAMERAS[EE_BASIC_CONSTANTS.camera_index];
  if (camera.background_source == 1) {
    return camera.clear_color.xyz * camera.clear_color.w;
  }
  return pow(resources.sampleSky(normalize(EE_ENVIRONMENT_LOCAL_DIRECTION(direction))),
             float3(1.0f / EE_ENVIRONMENT.gamma)) *
         camera.clear_color.w;
}

static const float PI = 3.14159265359f;
// ----------------------------------------------------------------------------
public
float EE_FUNC_DISTRIBUTION_GGX(float3 N, float3 H, float roughness) {
  float a = roughness * roughness;
  float a2 = a * a;
  float NdotH = max(dot(N, H), 0.0f);
  float NdotH2 = NdotH * NdotH;

  float nom = a2;
  float denom = (NdotH2 * (a2 - 1.0f) + 1.0f);
  denom = PI * denom * denom;

  return nom / max(denom, 0.001f);  // prevent divide by zero for roughness=0.0f and NdotH=1.0
}
// ----------------------------------------------------------------------------
public
float EE_FUNC_GEOMETRY_SCHLICK_GGX(float NdotV, float roughness) {
  float r = (roughness + 1.0f);
  float k = (r * r) / 8.0f;

  float nom = NdotV;
  float denom = NdotV * (1.0f - k) + k;

  return nom / denom;
}
// ----------------------------------------------------------------------------
public
float EE_FUNC_GEOMETRY_SMITH(float3 N, float3 V, float3 L, float roughness) {
  float NdotV = max(dot(N, V), 0.0f);
  float NdotL = max(dot(N, L), 0.0f);
  float ggx2 = EE_FUNC_GEOMETRY_SCHLICK_GGX(NdotV, roughness);
  float ggx1 = EE_FUNC_GEOMETRY_SCHLICK_GGX(NdotL, roughness);

  return ggx1 * ggx2;
}
// ----------------------------------------------------------------------------
public
float3 EE_FUNC_FRESNEL_SCHLICK(float cosTheta, float3 F0, float F90) {
  return F0 + (float3(F90) - F0) * pow(max(1.0f - clamp(cosTheta, 0.0f, 1.0f), 0.0f), 5.0f);
}

// ----------------------------------------------------------------------------
public
float3 EE_FUNC_FRESNEL_SCHLICK_ROUGHNESS(float cosTheta, float3 F0, float F90, float roughness) {
  return F0 +
         (max(float3(F90 * (1.0f - roughness)), F0) - F0) * pow(max(1.0f - clamp(cosTheta, 0.0f, 1.0f), 0.0f), 5.0f);
}

public
float3 EE_FUNC_CALCULATE_LIGHTS(in bool calculateShadow, float3 albedo, float specular, float dist, float3 normal,
                                float3 viewDir, float3 fragPos, float metallic, float roughness, float3 F0, float F90);
public
float3 EE_FUNC_DIRECTIONAL_LIGHT(float3 albedo, float specular, int i, float3 normal, float3 viewDir, float metallic,
                                 float roughness, float3 F0, float F90);
public
float3 EE_FUNC_POINT_LIGHT(float3 albedo, float specular, int i, float3 normal, float3 fragPos, float3 viewDir,
                           float metallic, float roughness, float3 F0, float F90);
public
float3 EE_FUNC_SPOT_LIGHT(float3 albedo, float specular, int i, float3 normal, float3 fragPos, float3 viewDir,
                          float metallic, float roughness, float3 F0, float F90);
public
float EE_FUNC_DIRECTIONAL_LIGHT_SHADOW(int i, int splitIndex, float3 fragPos, float3 normal, float cameraFragDistance);
public
float EE_FUNC_POINT_LIGHT_SHADOW(int i, float3 fragPos, float cameraFragDistance);
public
float EE_FUNC_SPOT_LIGHT_SHADOW(int i, float3 fragPos, float cameraFragDistance);
public
int EE_FUNC_DIRECTIONAL_SHADOW_CAMERA_INDEX();
public
int EE_FUNC_DIRECTIONAL_SHADOW_CASCADE_INDEX(float dist);
public
float EE_FUNC_DIRECTIONAL_LIGHT_CASCADE_SHADOW(int lightIndex, float dist, float3 fragPos, float3 normal,
                                               float cameraFragDistance);
public
float4 EE_FUNC_DIRECTIONAL_SHADOW_DEBUG(float dist, float3 fragPos);

public
float3 EE_FUNC_CALCULATE_DDGI_DIFFUSE(float3 albedo, float3 normal, float3 viewDir, float3 fragPos) {
  return EE_DDGI_WEIGHTED_DIFFUSE(EE_DDGI_GATHER_IRRADIANCE(normal, viewDir, fragPos), albedo);
}

public
struct EeEnvironmentalLighting {
 public
  float3 diffuse;
 public
  float3 specular;
 public
  float3 unoccluded_specular;
 public
  float3 diffuse_weight;
 public
  float specular_visibility;
};

public
float EE_REFLECTION_PROBE_SCALAR_VISIBILITY(float materialOcclusion, float screenSpaceVisibility,
                                            float ddgiVisibility) {
  return min(clamp(materialOcclusion, 0.0f, 1.0f),
             min(clamp(screenSpaceVisibility, 0.0f, 1.0f), clamp(ddgiVisibility, 0.0f, 1.0f)));
}

public
float EE_ROUGH_SPECULAR_VISIBILITY(float materialOcclusion, float screenSpaceVisibility, float ddgiVisibility,
                                   float roughness, float normalDotView) {
  const float scalarVisibility =
      EE_REFLECTION_PROBE_SCALAR_VISIBILITY(materialOcclusion, screenSpaceVisibility, ddgiVisibility);
  const float clampedRoughness = clamp(roughness, 0.0f, 1.0f);
  const float lobeWidth = clampedRoughness * clampedRoughness;
  const float scalarOcclusion = 1.0f - scalarVisibility;
  const float grazingOcclusion = 0.04f * tanh(scalarOcclusion / 0.04f);
  const float viewConfidence = smoothstep(0.8f, 1.0f, clamp(normalDotView, 0.0f, 1.0f));
  const float trustedOcclusion = lerp(grazingOcclusion, scalarOcclusion, viewConfidence);
  return 1.0f - lobeWidth * trustedOcclusion;
}

public
int EE_INDIRECT_LIGHTING_DEBUG_VIEW() {
  return EE_BASIC_CONSTANTS.instance_index == 2 ? 0 : int(round(EE_RENDER_INFO.shadow_fade_parameters.y));
}

bool EE_REFLECTION_PROBE_INFLUENCE(ReflectionProbeInfo probe, float3 frag_pos, out float3 local_position,
                                   out float weight) {
  local_position = mul(float4(frag_pos, 1.0f), probe.world_to_probe).xyz;
  float edge_distance;
  if (int(probe.lighting_parameters.z) == 0) {
    const float3 edge = probe.shape_parameters.xyz - abs(local_position);
    edge_distance = min(edge.x, min(edge.y, edge.z));
  } else {
    edge_distance = probe.shape_parameters.w - length(local_position);
  }
  if (edge_distance < 0.0f) {
    weight = 0.0f;
    return false;
  }
  const float blend_distance = probe.projection_parameters.w;
  weight = blend_distance <= 0.0f ? 1.0f : clamp(edge_distance / blend_distance, 0.0f, 1.0f);
  return true;
}

float EE_REFLECTION_SAFE_DIRECTION_COMPONENT(float value) {
  if (abs(value) >= 1.0e-6f)
    return value;
  return value >= 0.0f ? 1.0e-6f : -1.0e-6f;
}

float3 EE_REFLECTION_PROBE_DIRECTION(ReflectionProbeInfo probe, float3 local_position, float3 reflection_direction) {
  if (int(probe.lighting_parameters.z) != 0 || probe.lighting_parameters.w < 0.5f)
    return normalize(reflection_direction);
  const float3 local_direction = mul(reflection_direction, float3x3(probe.world_to_probe));
  const float3 projection_extents = probe.projection_parameters.xyz;
  if (any(abs(local_position) > projection_extents))
    return normalize(reflection_direction);
  const float3 safe_direction = float3(EE_REFLECTION_SAFE_DIRECTION_COMPONENT(local_direction.x),
                                       EE_REFLECTION_SAFE_DIRECTION_COMPONENT(local_direction.y),
                                       EE_REFLECTION_SAFE_DIRECTION_COMPONENT(local_direction.z));
  const float3 t0 = (-projection_extents - local_position) / safe_direction;
  const float3 t1 = (projection_extents - local_position) / safe_direction;
  const float3 far_plane = max(t0, t1);
  const float distance = min(far_plane.x, min(far_plane.y, far_plane.z));
  if (distance <= 0.0f || isnan(distance) || isinf(distance))
    return normalize(reflection_direction);
  const float3 local_hit = local_position + local_direction * distance;
  const float3x3 world_to_probe = float3x3(probe.world_to_probe);
  const float3 inverse_column0 = cross(world_to_probe[1], world_to_probe[2]);
  const float3 inverse_column1 = cross(world_to_probe[2], world_to_probe[0]);
  const float3 inverse_column2 = cross(world_to_probe[0], world_to_probe[1]);
  const float determinant = dot(world_to_probe[0], inverse_column0);
  if (abs(determinant) <= 1.0e-8f)
    return normalize(reflection_direction);
  return normalize(
      float3(dot(local_hit, inverse_column0), dot(local_hit, inverse_column1), dot(local_hit, inverse_column2)) /
      determinant);
}

float3 EE_SAMPLE_REFLECTION_PROBE<T : ILightingResources>(T resources, int index, ReflectionProbeInfo probe,
                                                          float3 direction, float lod, float3 global_prefiltered) {
  const float blend_weight = clamp(asfloat(probe.transition_parameters.z), 0.0f, 1.0f);
  const float3 source = probe.identity_and_flags.y != 0u
                            ? resources.sampleReflectionProbe(index * 2, direction, lod)
                            : global_prefiltered;
  if (probe.transition_parameters.y == 0u || blend_weight <= 0.0f)
    return source;
  const float3 target = resources.sampleReflectionProbe(index * 2 + 1, direction, lod);
  return blend_weight >= 1.0f ? target : lerp(source, target, blend_weight);
}

float3 EE_SPATIAL_REFLECTION_PREFILTERED<T : ILightingResources>(T resources, float3 frag_pos,
                                                                 float3 reflection_direction, float roughness,
                                                                 float3 global_prefiltered) {
  int primary = -1;
  int secondary = -1;
  float3 primary_local = float3(0.0f);
  float3 secondary_local = float3(0.0f);
  float primary_weight = 0.0f;
  float secondary_raw_weight = 0.0f;
  const int probe_count = min(int(EE_RENDER_INFO.reflection_probe_header.x), int(EE_REFLECTION_PROBE_MAX_COUNT));
  for (int index = 0; index < probe_count; ++index) {
    float3 local_position;
    float weight;
    if (EE_REFLECTION_PROBE_INFLUENCE(EE_RENDER_INFO.reflection_probes[index], frag_pos, local_position, weight)) {
      primary = index;
      primary_local = local_position;
      primary_weight = weight;
      break;
    }
  }
  if (primary < 0)
    return global_prefiltered;
  if (primary_weight < 1.0f) {
    for (int index = primary + 1; index < probe_count; ++index) {
      if (EE_RENDER_INFO.reflection_probes[index].lighting_parameters.y >=
          EE_RENDER_INFO.reflection_probes[primary].lighting_parameters.y)
        continue;
      float3 local_position;
      float weight;
      if (EE_REFLECTION_PROBE_INFLUENCE(EE_RENDER_INFO.reflection_probes[index], frag_pos, local_position, weight)) {
        secondary = index;
        secondary_local = local_position;
        secondary_raw_weight = weight;
        break;
      }
    }
  }
  const float global_weight = (1.0f - primary_weight) * (1.0f - secondary_raw_weight);
  float3 result = global_prefiltered * global_weight;
  ReflectionProbeInfo primary_probe = EE_RENDER_INFO.reflection_probes[primary];
  if (primary_probe.identity_and_flags.y != 0u || primary_probe.transition_parameters.y != 0u) {
    const float3 direction = EE_REFLECTION_PROBE_DIRECTION(primary_probe, primary_local, reflection_direction);
    result += EE_SAMPLE_REFLECTION_PROBE(resources, primary, primary_probe, direction, roughness * 8.0f,
                                         global_prefiltered) *
              primary_probe.lighting_parameters.x * primary_weight;
  } else {
    result += global_prefiltered * primary_weight;
  }
  if (secondary >= 0) {
    const float secondary_weight = (1.0f - primary_weight) * secondary_raw_weight;
    ReflectionProbeInfo secondary_probe = EE_RENDER_INFO.reflection_probes[secondary];
    if (secondary_probe.identity_and_flags.y != 0u || secondary_probe.transition_parameters.y != 0u) {
      const float3 direction = EE_REFLECTION_PROBE_DIRECTION(secondary_probe, secondary_local, reflection_direction);
      result += EE_SAMPLE_REFLECTION_PROBE(resources, secondary, secondary_probe, direction, roughness * 8.0f,
                                           global_prefiltered) *
                secondary_probe.lighting_parameters.x * secondary_weight;
    } else {
      result += global_prefiltered * secondary_weight;
    }
  }
  return result;
}

public
EeEnvironmentalLighting EE_FUNC_CALCULATE_ENVIRONMENTAL_COMPONENTS<T : ILightingResources>(
    T resources, float3 albedo, float3 normal, float3 viewDir, float3 fragPos, float metallic, float roughness,
    float3 F0, float F90, float materialOcclusion, float screenSpaceVisibility, float3 ddgiIrradiance,
    float ddgiWeight, float ddgiVisibility) {
  EeEnvironmentalLighting result;
  const float normalDotView = max(dot(normal, viewDir), 0.0f);
  const float3 F = EE_FUNC_FRESNEL_SCHLICK_ROUGHNESS(normalDotView, F0, F90, roughness);
  const float3 R = reflect(-viewDir, normal);
  result.diffuse_weight = (float3(1.0f) - F) * (1.0f - metallic);
  const bool reflectionProbeCapture = EE_BASIC_CONSTANTS.instance_index == 2;

  const float3 irradiance = EE_ENVIRONMENT.background_color.w == 1.0f
                                ? EE_ENVIRONMENT.background_color.xyz
                                : pow(resources.sampleIrradiance(EE_ENVIRONMENT_LOCAL_DIRECTION(normal)),
                                      float3(1.0f / EE_ENVIRONMENT.gamma));
  result.diffuse = result.diffuse_weight * irradiance * albedo *
                   (reflectionProbeCapture ? 0.0f : EE_ENVIRONMENT.diffuse_fallback_intensity);

  const uint reflection_levels = resources.prefilteredLevelCount();
  const float reflectionLodScale = float(max(int(reflection_levels) - 1, 0));
  const float3 globalPrefiltered =
      (EE_ENVIRONMENT.background_color.w == 1.0f
           ? EE_ENVIRONMENT.background_color.xyz
           : pow(resources.samplePrefiltered(EE_ENVIRONMENT_LOCAL_DIRECTION(R), roughness * reflectionLodScale),
                 float3(1.0f / EE_ENVIRONMENT.gamma))) *
      (reflectionProbeCapture ? 0.0f : EE_ENVIRONMENT.specular_fallback_intensity);
  float3 reflectionFallback = globalPrefiltered;
  const float ddgiSpecularWeight = reflectionProbeCapture ? 0.0f : clamp(ddgiWeight, 0.0f, 1.0f);
  const float3 ddgiSpecularFallback = max(ddgiIrradiance, float3(0.0f)) / EE_DDGI_PI;
  if (ddgiSpecularWeight > 0.0f && !any(isnan(ddgiSpecularFallback)) && !any(isinf(ddgiSpecularFallback))) {
    reflectionFallback = lerp(globalPrefiltered, ddgiSpecularFallback, ddgiSpecularWeight);
  }
  const float3 prefilteredColor = reflectionProbeCapture
                                      ? float3(0.0f)
                                      : EE_SPATIAL_REFLECTION_PREFILTERED(resources, fragPos, R, roughness,
                                                                         reflectionFallback);
  const float2 brdf = resources.sampleBrdf(float2(normalDotView, roughness));
  result.unoccluded_specular = prefilteredColor * (F * brdf.x + float3(F90 * brdf.y));
  const bool gtaoVisibilityAvailable = (EE_CAMERAS[EE_BASIC_CONSTANTS.camera_index].raster_lighting_flags & 1u) != 0u;
  result.specular_visibility =
      EE_ROUGH_SPECULAR_VISIBILITY(materialOcclusion, gtaoVisibilityAvailable ? screenSpaceVisibility : 1.0f,
                                   ddgiVisibility, roughness, normalDotView);
  result.specular = result.unoccluded_specular * result.specular_visibility;
  return result;
}

public
float3 EE_FUNC_CALCULATE_ENVIRONMENTAL_LIGHT<T : ILightingResources>(T resources, float3 albedo, float3 normal,
                                                                     float3 viewDir, float3 fragPos, float metallic,
                                                                     float roughness, float3 F0, float F90) {
  const EeEnvironmentalLighting environment = EE_FUNC_CALCULATE_ENVIRONMENTAL_COMPONENTS(
      resources, albedo, normal, viewDir, fragPos, metallic, roughness, F0, F90, 1.0f, 1.0f, float3(0.0f), 0.0f,
      1.0f);
  const float3 diffuseIndirect = environment.diffuse * EE_RENDER_INFO.indirect_lighting_intensity;
  return diffuseIndirect + environment.specular;
}

public
float3 EE_FUNC_CALCULATE_DDGI_ENVIRONMENTAL_LIGHT<T : ILightingResources>(T resources, float3 albedo, float3 normal,
                                                                          float3 viewDir, float3 fragPos,
                                                                          float metallic, float roughness, float3 F0,
                                                                          float F90, float materialOcclusion,
                                                                          float screenSpaceVisibility) {
  const EeDdgiGatherResult gather = EE_DDGI_GATHER_IRRADIANCE(normal, viewDir, fragPos);
  const float gather_weight = EE_DDGI_GATHER_WEIGHT(gather);
  const float ddgiSpecularVisibility = lerp(1.0f, EE_DDGI_GATHER_VISIBILITY(gather), gather_weight);
  const EeEnvironmentalLighting environment =
      EE_FUNC_CALCULATE_ENVIRONMENTAL_COMPONENTS(resources, albedo, normal, viewDir, fragPos, metallic, roughness, F0,
                                                 F90, materialOcclusion, screenSpaceVisibility, gather.irradiance,
                                                 gather_weight, ddgiSpecularVisibility);
  const float3 diffuse_albedo = environment.diffuse_weight * albedo;
  float3 diffuse = environment.diffuse;
  if (gather_weight > 0.0f && any(diffuse_albedo > float3(0.0f))) {
    const float3 ddgi_diffuse = EE_DDGI_DIFFUSE_RADIANCE(gather, diffuse_albedo);
    if (!any(isnan(ddgi_diffuse)) && !any(isinf(ddgi_diffuse))) {
      diffuse = lerp(diffuse, ddgi_diffuse, gather_weight);
    }
  }
  const float indirectVisibility = clamp(materialOcclusion * screenSpaceVisibility, 0.0f, 1.0f);
  const float3 diffuseIndirect = diffuse * indirectVisibility * EE_RENDER_INFO.indirect_lighting_intensity;
  const float3 unoccludedSpecular = environment.unoccluded_specular;
  const float3 specular = environment.specular;
  const int debugView = EE_INDIRECT_LIGHTING_DEBUG_VIEW();
  if (debugView == 1)
    return diffuseIndirect;
  if (debugView == 2)
    return unoccludedSpecular;
  if (debugView == 3)
    return float3(environment.specular_visibility);
  if (debugView == 4)
    return specular;
  return diffuseIndirect + specular;
}

public
float3 EE_FUNC_CALCULATE_LIGHTS(bool calculateShadow, float3 albedo, float specular, float dist, float3 normal,
                                float3 viewDir, float3 fragPos, float metallic, float roughness, float3 F0, float F90) {
  float3 result = float3(0.0, 0.0, 0.0f);
  float3 fragToCamera = fragPos - EE_CAMERA_POSITION(EE_BASIC_CONSTANTS.camera_index);
  float cameraFragDistance = length(fragToCamera);
  int directionalShadowCameraIndex = EE_FUNC_DIRECTIONAL_SHADOW_CAMERA_INDEX();
  float3 fragToDirectionalShadowCamera = fragPos - EE_CAMERA_POSITION(directionalShadowCameraIndex);
  float directionalShadowCameraFragDistance = length(fragToDirectionalShadowCamera);
  float directionalShadowDepth = abs(mul(float4(fragPos, 1.0f), EE_CAMERAS[directionalShadowCameraIndex].view).z);
  // phase 1: directional lighting
  for (int i = 0; i < EE_RENDER_INFO.directional_light_size; i++) {
    float shadow = 1.0f;
    int lightIndex = directionalShadowCameraIndex * int(EeMaxDirectionalLightsPerCamera()) + i;
    if (calculateShadow && EE_DIRECTIONAL_LIGHTS[lightIndex].diffuse.w == 1.0f) {
      shadow = EE_FUNC_DIRECTIONAL_LIGHT_CASCADE_SHADOW(lightIndex, directionalShadowDepth, fragPos, normal,
                                                        directionalShadowCameraFragDistance);
    }
    result +=
        EE_FUNC_DIRECTIONAL_LIGHT(albedo, specular, lightIndex, normal, viewDir, metallic, roughness, F0, F90) * shadow;
  }
  // phase 2: point lights
  for (int i = 0; i < EE_RENDER_INFO.point_light_size; i++) {
    float shadow = 1.0f;
    if (calculateShadow && EE_POINT_LIGHTS[i].diffuse.w == 1.0f) {
      shadow = EE_FUNC_POINT_LIGHT_SHADOW(i, fragPos, cameraFragDistance);
    }
    result += EE_FUNC_POINT_LIGHT(albedo, specular, i, normal, fragPos, viewDir, metallic, roughness, F0, F90) * shadow;
  }
  // phase 3: spot light
  for (int i = 0; i < EE_RENDER_INFO.spot_light_size; i++) {
    float shadow = 1.0f;
    if (calculateShadow && EE_SPOT_LIGHTS[i].diffuse.w == 1.0f) {
      shadow = EE_FUNC_SPOT_LIGHT_SHADOW(i, fragPos, cameraFragDistance);
    }
    result += EE_FUNC_SPOT_LIGHT(albedo, specular, i, normal, fragPos, viewDir, metallic, roughness, F0, F90) * shadow;
  }
  return result;
}

// calculates the color when using a directional light.
public
float3 EE_FUNC_DIRECTIONAL_LIGHT(float3 albedo, float specular, int i, float3 normal, float3 viewDir, float metallic,
                                 float roughness, float3 F0, float F90) {
  DirectionalLight light = EE_DIRECTIONAL_LIGHTS[i];
  float3 lightDir = normalize(-light.direction);
  float3 H = normalize(viewDir + lightDir);
  float3 radiance = light.diffuse.xyz;
  float normalDF = EE_FUNC_DISTRIBUTION_GGX(normal, H, roughness);
  float G = EE_FUNC_GEOMETRY_SMITH(normal, viewDir, lightDir, roughness);
  float3 F = EE_FUNC_FRESNEL_SCHLICK(clamp(dot(H, viewDir), 0.0, 1.0f), F0, F90);
  float3 nominator = normalDF * G * F;
  float denominator = 4 * max(dot(normal, viewDir), 0.0f) * max(dot(normal, lightDir), 0.0f);
  float3 spec = nominator / max(denominator, 0.001f) * specular;
  float3 kS = F;
  float3 kD = float3(1.0f) - kS;
  kD *= 1.0f - metallic;
  float NdotL = max(dot(normal, lightDir), 0.0f);
  return (kD * albedo / PI + spec) * radiance * NdotL;
}

// calculates the color when using a point light.
public
float3 EE_FUNC_POINT_LIGHT(float3 albedo, float specular, int i, float3 normal, float3 fragPos, float3 viewDir,
                           float metallic, float roughness, float3 F0, float F90) {
  PointLight light = EE_POINT_LIGHTS[i];
  float3 lightDir = normalize(light.position - fragPos);
  float3 H = normalize(viewDir + lightDir);
  float distance = length(light.position - fragPos);
  float attenuation = 1.0f / (light.constant_linear_quadratic_far.x + light.constant_linear_quadratic_far.y * distance +
                              light.constant_linear_quadratic_far.z * (distance * distance));
  float3 radiance = light.diffuse.xyz * attenuation;
  float normalDF = EE_FUNC_DISTRIBUTION_GGX(normal, H, roughness);
  float G = EE_FUNC_GEOMETRY_SMITH(normal, viewDir, lightDir, roughness);
  float3 F = EE_FUNC_FRESNEL_SCHLICK(clamp(dot(H, viewDir), 0.0f, 1.0f), F0, F90);
  float3 nominator = normalDF * G * F;
  float denominator = 4 * max(dot(normal, viewDir), 0.0f) * max(dot(normal, lightDir), 0.0f);
  float3 spec = nominator / max(denominator, 0.001) * specular;
  float3 kS = F;
  float3 kD = float3(1.0f) - kS;
  kD *= 1.0f - metallic;
  float NdotL = max(dot(normal, lightDir), 0.0f);
  return (kD * albedo / PI + spec) * radiance * NdotL;
}

// calculates the color when using a spot light.
public
float3 EE_FUNC_SPOT_LIGHT(float3 albedo, float specular, int i, float3 normal, float3 fragPos, float3 viewDir,
                          float metallic, float roughness, float3 F0, float F90) {
  SpotLight light = EE_SPOT_LIGHTS[i];
  float3 lightDir = normalize(light.position - fragPos);
  float3 H = normalize(viewDir + lightDir);
  float distance = length(light.position - fragPos);
  float attenuation = 1.0f / (light.constant_linear_quadratic_far.x + light.constant_linear_quadratic_far.y * distance +
                              light.constant_linear_quadratic_far.z * (distance * distance));
  // spotlight intensity
  float theta = dot(lightDir, normalize(-light.direction));
  float epsilon = light.cutoff_outer_inner_size_bias.x - light.cutoff_outer_inner_size_bias.y;
  float intensity = clamp((theta - light.cutoff_outer_inner_size_bias.y) / epsilon, 0.0f, 1.0f);

  float3 radiance = light.diffuse.xyz * attenuation * intensity;
  float normalDF = EE_FUNC_DISTRIBUTION_GGX(normal, H, roughness);
  float G = EE_FUNC_GEOMETRY_SMITH(normal, viewDir, lightDir, roughness);
  float3 F = EE_FUNC_FRESNEL_SCHLICK(clamp(dot(H, viewDir), 0.0f, 1.0f), F0, F90);
  float3 nominator = normalDF * G * F;
  float denominator = 4 * max(dot(normal, viewDir), 0.0f) * max(dot(normal, lightDir), 0.0f);
  float3 spec = nominator / max(denominator, 0.001f) * specular;
  float3 kS = F;
  float3 kD = float3(1.0f) - kS;
  kD *= 1.0f - metallic;
  float NdotL = max(dot(normal, lightDir), 0.0f);
  return (kD * albedo / PI + spec) * radiance * NdotL;
}

public
int EE_FUNC_DIRECTIONAL_SHADOW_CAMERA_INDEX() {
  return EE_BASIC_CONSTANTS.light_split_index < 0 ? -EE_BASIC_CONSTANTS.light_split_index - 1
                                                  : EE_BASIC_CONSTANTS.camera_index;
}

public
int EE_FUNC_DIRECTIONAL_SHADOW_CASCADE_INDEX(float dist) {
  float4 splitDistances = EE_CAMERAS[EE_FUNC_DIRECTIONAL_SHADOW_CAMERA_INDEX()].shadow_split_distances;
  if (dist < splitDistances.x)
    return 0;
  if (dist < splitDistances.y)
    return 1;
  if (dist < splitDistances.z)
    return 2;
  if (dist < splitDistances.w)
    return 3;
  return -1;
}

public
float EE_FUNC_DIRECTIONAL_SHADOW_SPLIT_DISTANCE(int splitIndex) {
  return EE_CAMERAS[EE_FUNC_DIRECTIONAL_SHADOW_CAMERA_INDEX()].shadow_split_distances[clamp(splitIndex, 0, 3)];
}

public
float EE_FUNC_DIRECTIONAL_SHADOW_TRANSITION_HALF_WIDTH(int boundaryIndex) {
  float requestedWidth = max(EE_RENDER_INFO.shadow_cascade_transition_width, 0.0f);
  if (requestedWidth <= 0.0f)
    return 0.0f;

  float splitDistance = EE_FUNC_DIRECTIONAL_SHADOW_SPLIT_DISTANCE(boundaryIndex);
  float previousSplit = boundaryIndex == 0 ? 0.0f : EE_FUNC_DIRECTIONAL_SHADOW_SPLIT_DISTANCE(boundaryIndex - 1);
  float nextSplit = EE_FUNC_DIRECTIONAL_SHADOW_SPLIT_DISTANCE(boundaryIndex + 1);
  float availableWidth = max(min(splitDistance - previousSplit, nextSplit - splitDistance), 0.0f);
  return min(requestedWidth, availableWidth) * 0.5f;
}

public
float EE_FUNC_DIRECTIONAL_SHADOW_DISTANCE_FADE(float dist) {
  float maxShadowDistance = EE_CAMERAS[EE_FUNC_DIRECTIONAL_SHADOW_CAMERA_INDEX()].shadow_split_distances.w;
  if (dist >= maxShadowDistance)
    return 0.0f;

  float fadeWidth = max(EE_RENDER_INFO.shadow_fade_parameters.x, 0.0f);
  if (fadeWidth <= 0.0f)
    return 1.0f;
  return clamp((maxShadowDistance - dist) / fadeWidth, 0.0f, 1.0f);
}

public
float EE_FUNC_DIRECTIONAL_LIGHT_CASCADE_SHADOW(int lightIndex, float dist, float3 fragPos, float3 normal,
                                               float cameraFragDistance) {
  int splitIndex = EE_FUNC_DIRECTIONAL_SHADOW_CASCADE_INDEX(dist);
  if (splitIndex < 0)
    return 1.0f;

  float shadow = EE_FUNC_DIRECTIONAL_LIGHT_SHADOW(lightIndex, splitIndex, fragPos, normal, cameraFragDistance);
  for (int boundaryIndex = 0; boundaryIndex < 3; boundaryIndex++) {
    float halfWidth = EE_FUNC_DIRECTIONAL_SHADOW_TRANSITION_HALF_WIDTH(boundaryIndex);
    if (halfWidth <= 0.0f)
      continue;

    float splitDistance = EE_FUNC_DIRECTIONAL_SHADOW_SPLIT_DISTANCE(boundaryIndex);
    if (dist < splitDistance - halfWidth || dist >= splitDistance + halfWidth)
      continue;

    float previousShadow =
        EE_FUNC_DIRECTIONAL_LIGHT_SHADOW(lightIndex, boundaryIndex, fragPos, normal, cameraFragDistance);
    float nextShadow =
        EE_FUNC_DIRECTIONAL_LIGHT_SHADOW(lightIndex, boundaryIndex + 1, fragPos, normal, cameraFragDistance);
    float blend = smoothstep(splitDistance - halfWidth, splitDistance + halfWidth, dist);
    shadow = lerp(previousShadow, nextShadow, blend);
    break;
  }

  float distanceFade = EE_FUNC_DIRECTIONAL_SHADOW_DISTANCE_FADE(dist);
  return lerp(1.0f, shadow, distanceFade);
}

public
float3 EE_FUNC_DIRECTIONAL_SHADOW_CASCADE_COLOR(int cascadeIndex) {
  if (cascadeIndex == 0)
    return float3(0.08f, 0.55f, 1.0f);
  if (cascadeIndex == 1)
    return float3(0.0f, 0.95f, 0.35f);
  if (cascadeIndex == 2)
    return float3(1.0f, 0.85f, 0.05f);
  if (cascadeIndex == 3)
    return float3(1.0f, 0.18f, 0.16f);
  return float3(0.02f);
}

public
float4 EE_FUNC_DIRECTIONAL_SHADOW_DEBUG(float dist, float3 fragPos) {
  if (EE_BASIC_CONSTANTS.instance_index == 2)
    return float4(0.0f);
  int mode = EE_RENDER_INFO.shadow_debug_parameters.x;
  if (mode <= 0) {
    return float4(0.0f);
  }

  int cascadeIndex = EE_FUNC_DIRECTIONAL_SHADOW_CASCADE_INDEX(dist);
  if (mode == 1) {
    float3 cascadeColor = EE_FUNC_DIRECTIONAL_SHADOW_CASCADE_COLOR(cascadeIndex);
    for (int boundaryIndex = 0; boundaryIndex < 3; boundaryIndex++) {
      float halfWidth = EE_FUNC_DIRECTIONAL_SHADOW_TRANSITION_HALF_WIDTH(boundaryIndex);
      if (halfWidth <= 0.0f)
        continue;

      float splitDistance = EE_FUNC_DIRECTIONAL_SHADOW_SPLIT_DISTANCE(boundaryIndex);
      if (dist < splitDistance - halfWidth || dist >= splitDistance + halfWidth)
        continue;

      float blend = smoothstep(splitDistance - halfWidth, splitDistance + halfWidth, dist);
      cascadeColor = lerp(EE_FUNC_DIRECTIONAL_SHADOW_CASCADE_COLOR(boundaryIndex),
                          EE_FUNC_DIRECTIONAL_SHADOW_CASCADE_COLOR(boundaryIndex + 1), blend);
      break;
    }
    cascadeColor = lerp(float3(0.02f), cascadeColor, EE_FUNC_DIRECTIONAL_SHADOW_DISTANCE_FADE(dist));
    return float4(cascadeColor, 1.0f);
  }

  if (EE_RENDER_INFO.directional_light_size <= 0) {
    return float4(1.0f, 0.0f, 1.0f, 1.0f);
  }

  int localLightIndex =
      clamp(EE_RENDER_INFO.shadow_debug_parameters.z, 0, max(EE_RENDER_INFO.directional_light_size - 1, 0));
  int lightIndex = EE_FUNC_DIRECTIONAL_SHADOW_CAMERA_INDEX() * int(EeMaxDirectionalLightsPerCamera()) + localLightIndex;
  DirectionalLight light = EE_DIRECTIONAL_LIGHTS[lightIndex];
  if (light.diffuse.w < 0.5f || light.viewport_x_size <= 0 || light.viewport_y_size <= 0) {
    return float4(0.25f, 0.0f, 0.25f, 1.0f);
  }

  int splitIndex = clamp(EE_RENDER_INFO.shadow_debug_parameters.y, 0, 3);
  if (cascadeIndex != splitIndex) {
    return float4(0.02f, 0.02f, 0.02f, 1.0f);
  }

  float4 fragPosLightSpace = mul(float4(fragPos, 1.0f), light.light_space_matrix[splitIndex]);
  if (abs(fragPosLightSpace.w) < 0.000001f) {
    return float4(1.0f, 0.0f, 0.0f, 1.0f);
  }

  float3 projCoords = fragPosLightSpace.xyz / fragPosLightSpace.w;
  float2 lightUv = projCoords.xy * 0.5f + 0.5f;
  bool insideLightRect =
      all(lightUv >= float2(0.0f)) && all(lightUv <= float2(1.0f)) && projCoords.z >= 0.0f && projCoords.z <= 1.0f;
  float insideScale = insideLightRect ? 1.0f : 0.25f;

  if (mode == 2) {
    return float4(lerp(float3(0.75f, 0.0f, 0.0f), float3(lightUv, 1.0f), insideScale), 1.0f);
  }
  if (mode == 3) {
    float lightDepth = clamp(projCoords.z, 0.0f, 1.0f);
    float3 depthColor = float3(lightDepth);
    if (!insideLightRect) {
      depthColor = lerp(float3(0.75f, 0.0f, 0.0f), depthColor, 0.25f);
    }
    return float4(depthColor, 1.0f);
  }

  float2 atlasSize = EeDirectionalShadowMapSize();
  float2 viewportOffset = float2(float(light.viewport_x_offset), float(light.viewport_y_offset));
  float2 viewportSize = float2(float(light.viewport_x_size), float(light.viewport_y_size));
  float2 atlasUv = (lightUv * viewportSize + viewportOffset) / max(atlasSize, float2(1.0f));
  if (mode == 4) {
    return float4(lerp(float3(0.75f, 0.0f, 0.0f), float3(atlasUv, 1.0f), insideScale), 1.0f);
  }

  float2 worldUnitsPerTexelAxes =
      2.0f * float2(light.light_frustum_width[splitIndex], light.light_frustum_height[splitIndex]) /
      max(viewportSize, float2(1.0f));
  float worldUnitsPerTexel = max(worldUnitsPerTexelAxes.x, worldUnitsPerTexelAxes.y);
  float density = clamp(log2(max(worldUnitsPerTexel, 0.001f)) * 0.125f + 0.5f, 0.0f, 1.0f);
  float3 densityColor = lerp(float3(0.0f, 0.35f, 1.0f), float3(1.0f, 0.1f, 0.0f), density);
  float2 texelUv = fract(lightUv * viewportSize);
  float grid = texelUv.x < 0.04f || texelUv.y < 0.04f ? 1.0f : 0.0f;
  return float4(lerp(densityColor, float3(1.0f), grid * 0.6f) * insideScale, 1.0f);
}

public
float2 EE_FUNC_DIRECTIONAL_SHADOW_ATLAS_UV(DirectionalLight light, float2 lightUv) {
  float2 atlasSize = max(EeDirectionalShadowMapSize(), float2(1.0f));
  float2 viewportSize = max(float2(float(light.viewport_x_size), float(light.viewport_y_size)), float2(1.0f));
  float2 halfTexel = float2(0.5f) / viewportSize;
  float2 clampedLightUv = clamp(lightUv, halfTexel, float2(1.0f) - halfTexel);
  return (float2(float(light.viewport_x_offset), float(light.viewport_y_offset)) + clampedLightUv * viewportSize) /
         atlasSize;
}

public
float EE_FUNC_DIRECTIONAL_SHADOW_SAMPLE(DirectionalLight light, int splitIndex, float2 lightUv, float receiverDepth) {
  float2 viewportSize = max(float2(float(light.viewport_x_size), float(light.viewport_y_size)), float2(1.0f));
  float2 halfTexel = float2(0.5f) / viewportSize;
  if (any(lightUv < halfTexel) || any(lightUv > float2(1.0f) - halfTexel)) {
    return 1.0f;
  }
  return EE_DIRECTIONAL_LIGHT_SM.SampleCmp(
      float3(EE_FUNC_DIRECTIONAL_SHADOW_ATLAS_UV(light, lightUv), float(splitIndex)), receiverDepth);
}

public
float EE_FUNC_DIRECTIONAL_SHADOW_COMPARE(float receiverDepth, float closestDepth) {
  if (closestDepth <= 0.0f)
    return 1.0f;
  return receiverDepth < closestDepth ? 1.0f : 0.0f;
}

public
float EE_FUNC_DIRECTIONAL_SHADOW_HARD(DirectionalLight light, int splitIndex, float3 projCoords) {
  return EE_FUNC_DIRECTIONAL_SHADOW_SAMPLE(light, splitIndex, projCoords.xy, projCoords.z);
}

public
float EE_FUNC_DIRECTIONAL_SHADOW_PCF(DirectionalLight light, int splitIndex, float3 projCoords, float2 radiusUv,
                                     int sampleAmount, float3 randomSeed) {
  if (all(radiusUv <= float2(0.0f)) || sampleAmount <= 1) {
    return EE_FUNC_DIRECTIONAL_SHADOW_HARD(light, splitIndex, projCoords);
  }

  float shadow = 0.0f;
  for (int sampleIndex = 0; sampleIndex < sampleAmount; sampleIndex++) {
    float2 texCoord = projCoords.xy + EE_VOGEL_DISK_SAMPLE(sampleIndex, sampleAmount, randomSeed) * radiusUv;
    shadow += EE_FUNC_DIRECTIONAL_SHADOW_SAMPLE(light, splitIndex, texCoord, projCoords.z);
  }
  return shadow / float(sampleAmount);
}

public
float EE_FUNC_DIRECTIONAL_LIGHT_SHADOW(int i, int splitIndex, float3 fragPos, float3 normal, float cameraFragDistance) {
  DirectionalLight light = EE_DIRECTIONAL_LIGHTS[i];
  float3 lightDir = light.direction;
  float nDotL = max(dot(normal, -lightDir), 0.0f);
  float2 viewportSize = max(float2(float(light.viewport_x_size), float(light.viewport_y_size)), float2(1.0f));
  float2 halfExtent =
      max(float2(light.light_frustum_width[splitIndex], light.light_frustum_height[splitIndex]), float2(0.001f));
  float2 worldUnitsPerTexel = 2.0f * halfExtent / viewportSize;
  float shadowTexelSize = max(worldUnitsPerTexel.x, worldUnitsPerTexel.y);
  float constantBias = light.reserved_parameters.z;
  float slopeBias = light.reserved_parameters.y * (1.0f - nDotL);
  float lightDepthSpan = max(2.0f * light.light_frustum_distance[splitIndex], 0.001f);
  float bias = (constantBias + slopeBias) * shadowTexelSize / lightDepthSpan;
  float normalOffset = light.reserved_parameters.w * shadowTexelSize;

  fragPos = fragPos + normal * normalOffset;
  float4 fragPosLightSpace = mul(float4(fragPos, 1.0f), light.light_space_matrix[splitIndex]);
  float3 projCoords = fragPosLightSpace.xyz / fragPosLightSpace.w;
  if (projCoords.z < 0.0f || projCoords.z > 1.0f) {
    return 1.0f;
  }
  projCoords.x = projCoords.x * 0.5f + 0.5f;
  projCoords.y = projCoords.y * 0.5f + 0.5f;
  if (projCoords.x < 0.0f || projCoords.x > 1.0f || projCoords.y < 0.0f || projCoords.y > 1.0f) {
    return 1.0f;
  }
  projCoords = float3(projCoords.xy, projCoords.z - bias);

  float2 radiusUv = float2(max(light.reserved_parameters.x, 0.0f)) / (2.0f * halfExtent);
  int sampleAmount = clamp(EE_RENDER_INFO.shadow_debug_parameters.w, 1, 64);
  return EE_FUNC_DIRECTIONAL_SHADOW_PCF(light, splitIndex, projCoords, radiusUv, sampleAmount, fragPos * 3141);
}

public
float2 EE_FUNC_SHADOW_ATLAS_UV(float2 lightUv, int viewportXOffset, int viewportYOffset, int viewportXSize,
                               int viewportYSize, float2 atlasSize) {
  float2 viewportSize = max(float2(float(viewportXSize), float(viewportYSize)), float2(1.0f));
  float2 halfTexel = float2(0.5f) / viewportSize;
  float2 clampedLightUv = clamp(lightUv, halfTexel, float2(1.0f) - halfTexel);
  return (float2(float(viewportXOffset), float(viewportYOffset)) + clampedLightUv * viewportSize) /
         max(atlasSize, float2(1.0f));
}

public
float EE_FUNC_SPOT_SHADOW_DEPTH(SpotLight light, float2 lightUv) {
  return EE_SPOT_LIGHT_SM
      .Sample(EE_FUNC_SHADOW_ATLAS_UV(lightUv, light.viewport_x_offset, light.viewport_y_offset, light.viewport_x_size,
                                      light.viewport_y_size, EeSpotShadowMapSize()))
      .r;
}

public
float EE_FUNC_POINT_SHADOW_DEPTH(PointLight light, int slice, float2 lightUv) {
  return EE_POINT_LIGHT_SM
      .Sample(float3(EE_FUNC_SHADOW_ATLAS_UV(lightUv, light.viewport_x_offset, light.viewport_y_offset,
                                             light.viewport_x_size, light.viewport_y_size, EePointShadowMapSize()),
                     float(slice)))
      .r;
}

public
float EE_FUNC_SPOT_LIGHT_SHADOW(int i, float3 fragPos, float cameraFragDistance) {
  SpotLight light = EE_SPOT_LIGHTS[i];
  float4 fragPosLightSpace = mul(float4(fragPos, 1.0f), light.light_space_matrix);
  if (abs(fragPosLightSpace.w) < 0.000001f) {
    return 1.0f;
  }
  float3 projCoords = fragPosLightSpace.xyz / fragPosLightSpace.w;
  projCoords.z -= light.cutoff_outer_inner_size_bias.w;
  projCoords.x = projCoords.x * 0.5f + 0.5f;
  projCoords.y = projCoords.y * 0.5f + 0.5f;

  if (projCoords.z < 0.0f || projCoords.z > 1.0f || projCoords.x < 0.0f || projCoords.x > 1.0f || projCoords.y < 0.0f ||
      projCoords.y > 1.0f) {
    return 1.0f;
  }

  float radiusUv = max(light.cutoff_outer_inner_size_bias.z * 100.0f, 0.0f) / max(float(light.viewport_x_size), 1.0f);
  int sampleAmount = clamp(EE_RENDER_INFO.shadow_sample_size, 1, 64);
  if (radiusUv <= 0.0f || sampleAmount <= 1) {
    return EE_FUNC_DIRECTIONAL_SHADOW_COMPARE(projCoords.z, EE_FUNC_SPOT_SHADOW_DEPTH(light, projCoords.xy));
  }

  float shadow = 0.0f;
  for (int sampleIndex = 0; sampleIndex < sampleAmount; sampleIndex++) {
    float2 texCoord = projCoords.xy + EE_VOGEL_DISK_SAMPLE(sampleIndex, sampleAmount, fragPos * 3141) * radiusUv;
    float closestDepth = EE_FUNC_SPOT_SHADOW_DEPTH(light, texCoord);
    shadow += EE_FUNC_DIRECTIONAL_SHADOW_COMPARE(projCoords.z, closestDepth);
  }
  return shadow / float(sampleAmount);
}

public
float EE_FUNC_POINT_LIGHT_SHADOW(int i, float3 fragPos, float cameraFragDistance) {
  PointLight light = EE_POINT_LIGHTS[i];
  // get vector between fragment position and light position
  float3 fragToLight = fragPos - light.position;
  float shadow = 0.0f;
  int slice = 0;
  if (abs(fragToLight.x) >= abs(fragToLight.y) && abs(fragToLight.x) >= abs(fragToLight.z)) {
    if (fragToLight.x > 0) {
      slice = 0;
    } else {
      slice = 1;
    }
  } else if (abs(fragToLight.y) >= abs(fragToLight.z)) {
    if (fragToLight.y > 0) {
      slice = 2;
    } else {
      slice = 3;
    }
  } else {
    if (fragToLight.z > 0) {
      slice = 4;
    } else {
      slice = 5;
    }
  }
  float4 fragPosLightSpace = mul(float4(fragPos, 1.0f), light.light_space_matrix[slice]);
  if (abs(fragPosLightSpace.w) < 0.000001f) {
    return 1.0f;
  }
  float3 projCoords = fragPosLightSpace.xyz / fragPosLightSpace.w;
  projCoords.z -= light.reserved_parameters.x;
  projCoords.x = projCoords.x * 0.5f + 0.5f;
  projCoords.y = projCoords.y * 0.5f + 0.5f;

  if (projCoords.z < 0.0f || projCoords.z > 1.0f || projCoords.x < 0.0f || projCoords.x > 1.0f || projCoords.y < 0.0f ||
      projCoords.y > 1.0f) {
    return 1.0f;
  }

  float radiusUv = max(light.reserved_parameters.y * 100.0f, 0.0f) / max(float(light.viewport_x_size), 1.0f);
  int sampleAmount = clamp(EE_RENDER_INFO.shadow_sample_size, 1, 64);
  if (radiusUv <= 0.0f || sampleAmount <= 1) {
    return EE_FUNC_DIRECTIONAL_SHADOW_COMPARE(projCoords.z, EE_FUNC_POINT_SHADOW_DEPTH(light, slice, projCoords.xy));
  }

  for (int sampleIndex = 0; sampleIndex < sampleAmount; sampleIndex++) {
    float2 texCoord = projCoords.xy + EE_VOGEL_DISK_SAMPLE(sampleIndex, sampleAmount, fragPos * 3141) * radiusUv;
    float closestDepth = EE_FUNC_POINT_SHADOW_DEPTH(light, slice, texCoord);
    shadow += EE_FUNC_DIRECTIONAL_SHADOW_COMPARE(projCoords.z, closestDepth);
  }
  return shadow / float(sampleAmount);
}
