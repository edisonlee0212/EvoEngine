// @evoengine-dialect native
import EvoEngine.DDGIProbeRayConstants;
import EvoEngine.DDGIProbeRayPayload;
import EvoEngine.RayTracingBasic;

float3 EE_DDGI_SAMPLE_ENVIRONMENT_CUBEMAP(const uint environment_index, const float3 direction) {
  float3 environment_color =
      EE_CUBEMAPS[NonUniformResourceIndex(int(environment_index))]
          .SampleLevel(normalize(EE_ENVIRONMENT_LOCAL_DIRECTION(direction)), 0.0f)
          .rgb;
  if (EE_ENVIRONMENT.gamma != 1.0f) {
    environment_color =
        pow(max(environment_color, float3(0.0f)), float3(1.0f / max(EE_ENVIRONMENT.gamma, 0.001f)));
  }
  return max(environment_color, float3(0.0f));
}

float3 EE_DDGI_MISS_RADIANCE(const float3 direction) {
  if (EE_ENVIRONMENT.background_color.w == 1.0f) {
    return max(EE_ENVIRONMENT.background_color.rgb * EE_ENVIRONMENT.diffuse_sky_intensity, float3(0.0f));
  }
  if (EE_ENVIRONMENT.diffuse_sky_intensity <= 0.0f) {
    return float3(0.0f);
  }
  const uint environment_index =
      EE_DDGI_PROBE_RAY_CONSTANTS.selected_probe_volume_flags_environment.w;
  return EE_DDGI_SAMPLE_ENVIRONMENT_CUBEMAP(environment_index, direction) *
         EE_ENVIRONMENT.diffuse_sky_intensity;
}

[shader("miss")]
void main(inout DdgiProbeRayPayload hit_value) {
  hit_value.hit_count = 0u;
  hit_value.handle = uint64_t(0);
  hit_value.hit_info.position = float3(0.0f);
  hit_value.hit_info.normal = float3(0.0f);
  hit_value.hit_info.tangent = float3(0.0f);
  hit_value.hit_info.color = float4(EE_DDGI_MISS_RADIANCE(WorldRayDirection()), 0.0f);
  hit_value.hit_info.tex_coord = float2(0.0f);
  hit_value.hit_info.vertex_info1 = 0.0f;
  hit_value.hit_info.vertex_info2 = 0.0f;
  hit_value.hit_info.vertex_info3 = 0.0f;
  hit_value.hit_info.vertex_info4 = float2(0.0f);
}
