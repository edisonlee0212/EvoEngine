#extension GL_ARB_shading_language_include : enable

#include "PerFrame.glsl"
#include "DDGI.glsl"

layout(location = 0) in vec3 inPosition;
layout(location = 1) in vec3 inNormal;

layout(set = 1, binding = 0) readonly buffer EE_DDGI_PROBE_METADATA_BLOCK {
  vec4 EE_DDGI_PROBE_METADATA[];
};

layout(set = 1, binding = 1) readonly buffer EE_DDGI_PROBE_STATE_BLOCK {
  vec4 EE_DDGI_PROBE_STATE[];
};

layout(push_constant) uniform EE_DDGI_PROBE_VISUALIZATION_CONSTANTS {
  uvec4 camera_selected_mode;
  vec4 radius_intensity_alpha_selected_scale;
};

layout(location = 0) out VS_OUT {
  vec3 Normal;
  vec4 Irradiance;
  vec4 Visibility;
  vec4 State;
  flat uint PhysicalProbeIndex;
  flat float Selected;
  flat float CameraFade;
} vs_out;

uvec3 EE_DDGI_GRID_FROM_PROBE_INDEX(const uint probe_index, const uvec3 probe_counts) {
  const uint x_count = max(probe_counts.x, 1u);
  const uint y_count = max(probe_counts.y, 1u);
  const uint z_plane = x_count * y_count;
  return uvec3(probe_index % x_count, (probe_index / x_count) % y_count, probe_index / z_plane);
}

float EE_DDGI_PROBE_CAMERA_FADE_ALPHA(const vec3 probe_position, const uint camera_index, const float probe_radius) {
  const float fade_start = max(probe_radius, 0.001f);
  const float fade_end = max(probe_radius * 8.0f, fade_start + 0.001f);
  const float camera_distance = distance(probe_position, EE_CAMERA_POSITION(int(camera_index)));
  return smoothstep(fade_start, fade_end, camera_distance);
}

void main() {
  const uint logical_probe_index = gl_InstanceIndex;
  const DdgiVolumeInfo volume = EE_RENDER_INFO.ddgi_volumes[0];
  const uvec3 probe_counts = max(uvec3(volume.probe_counts.xyz), uvec3(1u));
  const uvec3 probe_grid = EE_DDGI_GRID_FROM_PROBE_INDEX(logical_probe_index, probe_counts);
  const ivec3 probe_scroll_offset = volume.probe_scroll_and_priority.xyz;
  const uint physical_probe_index = EE_DDGI_SCROLL_PROBE_INDEX(probe_grid, probe_scroll_offset, probe_counts);
  const uint metadata_offset = physical_probe_index * 3u;
  const vec4 state = EE_DDGI_PROBE_STATE[physical_probe_index];
  const float selected = logical_probe_index == camera_selected_mode.y ? 1.0f : 0.0f;
  const float selected_scale = mix(1.0f, max(radius_intensity_alpha_selected_scale.w, 1.0f), selected);
  const float radius = max(radius_intensity_alpha_selected_scale.x, 0.0001f) * selected_scale;
  const vec3 world_position =
      EE_DDGI_PROBE_WORLD_POSITION(vec3(probe_grid), volume.first_probe.xyz, volume.probe_step_x.xyz,
                                   volume.probe_step_y.xyz, volume.probe_step_z.xyz) +
      state.xyz;

  vs_out.Normal = normalize(inNormal);
  vs_out.Irradiance = EE_DDGI_PROBE_METADATA[metadata_offset];
  vs_out.Visibility = EE_DDGI_PROBE_METADATA[metadata_offset + 1u];
  vs_out.State = state;
  vs_out.PhysicalProbeIndex = physical_probe_index;
  vs_out.Selected = selected;

  const uint camera_index = camera_selected_mode.x;
  vs_out.CameraFade = EE_DDGI_PROBE_CAMERA_FADE_ALPHA(world_position, camera_index, radius);
  gl_Position = EE_CAMERAS[camera_index].projection_view * vec4(world_position + inPosition * radius, 1.0f);
}
