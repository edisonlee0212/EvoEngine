import EvoEngine.PerFrame;
import EvoEngine.DDGI;

struct DdgiProbeVisualizationVertexInput {
  [[vk::location(0)]]
  float3 position : POSITION;
  [[vk::location(1)]]
  float3 normal : NORMAL;
};

struct DdgiProbeVisualizationOutput {
  float4 position : SV_Position;
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

[[vk::binding(0, 1)]]
StructuredBuffer<float4> EE_DDGI_PROBE_METADATA;
[[vk::binding(1, 1)]]
StructuredBuffer<float4> EE_DDGI_PROBE_STATE;
[[vk::push_constant]]
ConstantBuffer<DdgiProbeVisualizationConstants> constants;

uint3 EeDdgiGridFromProbeIndex(uint probe_index, uint3 probe_counts) {
  uint x_count = max(probe_counts.x, 1u);
  uint y_count = max(probe_counts.y, 1u);
  uint z_plane = x_count * y_count;
  return uint3(probe_index % x_count, (probe_index / x_count) % y_count, probe_index / z_plane);
}

float EeDdgiProbeCameraFadeAlpha(float3 probe_position, uint camera_index, float probe_radius) {
  float fade_start = max(probe_radius, 0.001f);
  float fade_end = max(probe_radius * 8.0f, fade_start + 0.001f);
  return smoothstep(fade_start, fade_end, distance(probe_position, EE_CAMERA_POSITION(int(camera_index))));
}

[shader("vertex")]
DdgiProbeVisualizationOutput main(DdgiProbeVisualizationVertexInput input, uint instance_index : SV_InstanceID) {
  DdgiProbeVisualizationOutput output;
  uint logical_probe_index = instance_index;
  DdgiVolumeInfo volume = EE_RENDER_INFO.ddgi_volumes[constants.camera_selected_mode.w];
  uint3 probe_counts = max(uint3(volume.probe_counts.xyz), uint3(1u));
  uint3 probe_grid = EeDdgiGridFromProbeIndex(logical_probe_index, probe_counts);
  int3 probe_scroll_offset = volume.probe_scroll_and_priority.xyz;
  uint physical_probe_index = EE_DDGI_SCROLL_PROBE_INDEX(probe_grid, probe_scroll_offset, probe_counts);
  uint metadata_offset = physical_probe_index * 3u;
  float4 state = EE_DDGI_PROBE_STATE[physical_probe_index];
  float selected = logical_probe_index == constants.camera_selected_mode.y ? 1.0f : 0.0f;
  float selected_scale = lerp(1.0f, max(constants.radius_intensity_alpha_selected_scale.w, 1.0f), selected);
  float radius = max(constants.radius_intensity_alpha_selected_scale.x, 0.0001f) * selected_scale;
  float3 world_position =
      EE_DDGI_PROBE_WORLD_POSITION(float3(probe_grid), volume.first_probe.xyz, volume.probe_step_x.xyz,
                                   volume.probe_step_y.xyz, volume.probe_step_z.xyz) +
      state.xyz;

  output.normal = normalize(input.normal);
  output.irradiance = EE_DDGI_PROBE_METADATA[metadata_offset];
  output.visibility = EE_DDGI_PROBE_METADATA[metadata_offset + 1u];
  output.state = state;
  output.physical_probe_index = physical_probe_index;
  output.selected = selected;
  uint camera_index = constants.camera_selected_mode.x;
  output.camera_fade = EeDdgiProbeCameraFadeAlpha(world_position, camera_index, radius);
  output.position =
      mul(float4(world_position + input.position * radius, 1.0f), EE_CAMERAS[camera_index].projection_view);
  return output;
}
