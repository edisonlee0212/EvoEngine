// @evoengine-dialect native
import EvoEngine.PerFrame;
import EvoEngine.DDGI;

struct DdgiProbeVisualizationInput {
  [[vk::location(0)]]
  float3 normal : NORMAL;
  [[vk::location(1)]]
  float4 irradiance : COLOR0;
  [[vk::location(2)]]
  float4 visibility : COLOR1;
  [[vk::location(3)]]
  float4 state : COLOR2;
  [[vk::location(4)]]
  nointerpolation uint physical_probe_index : TEXCOORD0;
  [[vk::location(5)]]
  nointerpolation float selected : TEXCOORD1;
  [[vk::location(6)]]
  nointerpolation float camera_fade : TEXCOORD2;
};

struct DdgiProbeVisualizationConstants {
  uint4 camera_selected_mode;
  float4 radius_intensity_alpha_selected_scale;
};

[[vk::binding(17, 1)]]
Sampler2D EE_DDGI_IRRADIANCE_ATLAS;
[[vk::push_constant]]
ConstantBuffer<DdgiProbeVisualizationConstants> constants;

float3 EeDdgiDecodeDebugIrradiance(float3 encoded_irradiance) {
  float irradiance_gamma =
      max(EE_RENDER_INFO.ddgi_volumes[constants.camera_selected_mode.w].probe_counts.w, 1.0f);
  float3 sqrt_irradiance = pow(max(encoded_irradiance, float3(0.0f)), float3(irradiance_gamma * 0.5f));
  return sqrt_irradiance * sqrt_irradiance * (2.0f * EE_DDGI_PI);
}

float3 EeDdgiTonemapDebugColor(float3 value) {
  float3 positive = max(value, float3(0.0f));
  float3 mapped = positive / (positive + float3(1.0f));
  return pow(clamp(mapped, float3(0.0f), float3(1.0f)), float3(1.0f / 2.2f));
}

float3 EeDdgiIrradianceDebugColor(float3 encoded_irradiance, float intensity) {
  return EeDdgiTonemapDebugColor(EeDdgiDecodeDebugIrradiance(encoded_irradiance) * intensity);
}

float3 EeDdgiVisibilityDebugColor(float4 visibility) {
  float average_distance = clamp(visibility.x / max(constants.radius_intensity_alpha_selected_scale.y, 0.001f), 0.0f,
                                 1.0f);
  float backface_ratio = clamp(visibility.y, 0.0f, 1.0f);
  float relocation = clamp(visibility.z, 0.0f, 1.0f);
  return lerp(float3(0.95f, 0.1f, 0.05f), float3(0.05f, 0.85f, 1.0f), average_distance) +
         float3(relocation * 0.15f, backface_ratio * 0.2f, 0.0f);
}

[shader("fragment")]
float4 main(DdgiProbeVisualizationInput input) : SV_Target {
  bool selected_only = (constants.camera_selected_mode.z & 0x80000000u) != 0u;
  if (selected_only && input.selected < 0.5f) discard;
  uint mode = constants.camera_selected_mode.z & 3u;
  float probe_inactive = clamp(input.state.w, 0.0f, 1.0f);
  float3 color;
  if (mode == 0u) {
    uint irradiance_tile_size =
        max(EE_RENDER_INFO.ddgi_volumes[constants.camera_selected_mode.w].atlas_parameters.x, 1u);
    uint irradiance_columns =
        max(EE_RENDER_INFO.ddgi_volumes[constants.camera_selected_mode.w].atlas_parameters.y, 1u);
    uint width;
    uint height;
    EE_DDGI_IRRADIANCE_ATLAS.GetDimensions(width, height);
    float2 irradiance_uv = EE_DDGI_ATLAS_UV(input.physical_probe_index, irradiance_columns,
                                            irradiance_tile_size, normalize(input.normal),
                                            max(float2(width, height), float2(1.0f)));
    color = EeDdgiIrradianceDebugColor(EE_DDGI_IRRADIANCE_ATLAS.Sample(irradiance_uv).rgb,
                                       constants.radius_intensity_alpha_selected_scale.y);
  } else if (mode == 1u) {
    color = EeDdgiIrradianceDebugColor(input.irradiance.rgb, constants.radius_intensity_alpha_selected_scale.y);
  } else if (mode == 2u) {
    color = EeDdgiVisibilityDebugColor(input.visibility);
  } else {
    color = lerp(float3(0.8f, 0.05f, 0.08f), float3(0.08f, 0.85f, 0.2f),
                 clamp(input.irradiance.a, 0.0f, 1.0f));
  }
  if (probe_inactive > 0.5f) color = lerp(float3(0.08f, 0.08f, 0.1f), color, 0.2f);
  if (input.selected > 0.5f) color = lerp(color, float3(1.0f, 0.18f, 0.08f), 0.35f);
  float alpha = clamp(constants.radius_intensity_alpha_selected_scale.z, 0.0f, 1.0f) *
                clamp(input.camera_fade, 0.0f, 1.0f);
  return float4(color, alpha);
}
