import EvoEngine.VolumetricClouds;

[CUDADeviceExport]
float4 marchCameraCloud(Sampler3D baseShape, Sampler3D detail, Sampler2D weather, Sampler2D curl,
                        float3 origin, float3 direction, float distance, float3 sunDirection, float3 sun,
                        float3 ambient, float jitter, float4 layer, float4 noise, float4 atmosphere,
                        float4 wind, float4 lighting, float2 stepScale, int3 steps,
                        float4 sceneColor, bool surfaceHit, float shadowStrength, int shadowSteps, int debugMode,
                        bool prepass, bool useCache, bool transmittanceOnly, float4 cachedCloud, out float cloudDepth) {
  VolumetricCloudSettingsGpu settings;
  settings.baseShape = baseShape;
  settings.detailErosion = detail;
  settings.weatherCoverage = weather;
  settings.curlNoise = curl;
  settings.enabled = 1;
  settings.bottom_altitude = layer.x;
  settings.top_altitude = layer.y;
  settings.coverage = layer.z;
  settings.density = layer.w;
  settings.base_noise_scale = noise.x;
  settings.detail_noise_scale = noise.y;
  settings.extinction_scale = noise.z;
  settings.max_march_distance = noise.w;
  settings.use_spherical_atmosphere = int(atmosphere.x);
  settings.atmosphere_radius = atmosphere.y;
  settings.cloud_type = atmosphere.z;
  settings.curl_strength = atmosphere.w;
  settings.wind_direction = wind.xy;
  settings.wind_speed = wind.z;
  settings.lighting_intensity = lighting.x;
  settings.ambient_lighting_strength = lighting.y;
  settings.phase_anisotropy = lighting.z;
  settings.coarse_step_fraction = stepScale.x;
  settings.fine_step_scale = stepScale.y;
  settings.primary_step_count = steps.x;
  settings.light_step_count = steps.y;
  settings.empty_step_fallback_count = steps.z;
  settings.debug_mode = debugMode;
  settings.enable_cloud_shadows = int(shadowStrength > 0);
  settings.cloud_shadow_strength = shadowStrength;
  settings.cloud_shadow_step_count = shadowSteps;
  cloudDepth = 0;
  // The cached image covers the whole sky ray. Clip against geometry per camera sample.
  if (useCache && surfaceHit && distance < noise.w) {
    let interval = EE_VOLUMETRIC_CLOUD_IntersectLayer(origin, direction, settings, noise.w);
    if (interval.valid == 0 || distance <= interval.t_min)
      cachedCloud = float4(float3(debugMode == 2 ? 1 : 0), 1);
    else if (distance < interval.t_max)
      useCache = false;
  }
  if (!useCache) {
    let result = EE_VOLUMETRIC_CLOUD_March(settings, origin, direction, min(distance, noise.w), distance,
                                           sunDirection, sun, ambient, wind.w, jitter, transmittanceOnly);
    cloudDepth = result.march_distance;
    float3 light = EE_VOLUMETRIC_CLOUD_DebugAccumulation(result, debugMode);
    if (debugMode == 2)
      light = float3(result.transmittance);
    if (debugMode == 3)
      light = float3(saturate(cloudDepth / noise.w));
    cachedCloud = float4(light, result.transmittance);
  }
  if (prepass)
    return cachedCloud;
  if (debugMode != 0)
    return float4(cachedCloud.rgb, 1);

  if (surfaceHit && shadowStrength > 0) {
    let shadow = EE_VOLUMETRIC_CLOUD_SurfaceShadowTransmittance(
        settings, origin + distance * direction, origin, sunDirection, wind.w);
    sceneColor.rgb *= lerp(1.0f, shadow, shadowStrength);
  }
  return float4(cachedCloud.rgb, 1 - cachedCloud.a) + cachedCloud.a * sceneColor;
}
