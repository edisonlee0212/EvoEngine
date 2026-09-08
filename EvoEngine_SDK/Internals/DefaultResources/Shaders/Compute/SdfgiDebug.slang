// Godot shaders/environment/sdfgi_debug.glsl, 34d06658a85845111a50db9e485ec4a0701d4298.
// See docs/licenses/Godot-MIT.txt. Host extensions: distance slice and camera-outside-coverage clipping.
import EvoEngine.SdfgiDebug;

float3 linear_to_srgb(float3 color) {
  return select(color < 0.0031308, 12.92 * color, 1.055 * pow(max(color, 0), float3(1.0 / 2.4)) - 0.055);
}

[numthreads(8, 8, 1)]
void main(uint3 id : SV_DispatchThreadID) {
  uint width, height;
  output_image.GetDimensions(width, height);
  if (id.x >= width || id.y >= height) return;
  float2 uv = (float2(id.xy) + 0.5) / float2(width, height);
  if (data.selection.x == 6) {
    // XY plane at the selected Z slice, preserving square voxels on a wide output.
    float aspect = float(width) / height * float(debug_grid().y) / debug_grid().x;
    float2 scale = aspect >= 1 ? float2(aspect, 1) : float2(1, 1 / aspect);
    float2 xy = (uv - 0.5) * scale + 0.5;
    float3 color = float3(0);
    if (all(xy >= 0) && all(xy < 1)) {
      float d = sdf_textures[data.selection.y].SampleLevel(linear_sampler,
          float3(xy, (data.selection.w + 0.5) / debug_grid().z), 0) * 255;
      color = d < 0.5 ? float3(1, 0.3, 0.05) : float3(saturate(d / 32));
    }
    output_image[id.xy] = float4(color, 1);
    return;
  }
  float3 ray_pos = data.origin_y_mult.xyz;
  float3 ray_dir = normalize(debug_world(uv, 0.5) - ray_pos);
  ray_pos.y *= data.origin_y_mult.w;
  ray_dir.y *= data.origin_y_mult.w;
  ray_dir = normalize(ray_dir);
  float3 inv_dir = 1.0 / select(abs(ray_dir) < 1e-8, float3(1e-8), ray_dir);
  float3 light = float3(0);
  for (uint c = 0; c < data.field.x; ++c) {
    float3 pos = (ray_pos - data.cascades[c].offset) * data.cascades[c].to_cell;
    float3 t0 = -pos * inv_dir;
    float3 t1 = (debug_grid() - pos) * inv_dir;
    float3 tmin = min(t0, t1), tmax = max(t0, t1);
    float advance = max(0.0, max(tmin.x, max(tmin.y, tmin.z)));
    float maximum = min(tmax.x, min(tmax.y, tmax.z));
    if (maximum <= advance) continue;
    bool hit = false;
    float3 uvw = float3(0);
    while (advance < maximum) {
      uvw = (pos + ray_dir * advance) / float3(debug_grid());
      float distance = sdf_textures[c].SampleLevel(linear_sampler, uvw, 0) * 255 - 1.7;
      if (distance < 0.001) { hit = true; break; }
      advance += distance;
    }
    if (!hit) {
      ray_pos = (pos + ray_dir * min(advance, maximum)) / data.cascades[c].to_cell + data.cascades[c].offset;
      continue;
    }
    const float3 EPSILON = 0.128 / float3(debug_grid());
    float3 normal = normalize(float3(
        sdf_textures[c].SampleLevel(linear_sampler, uvw + float3(EPSILON.x, 0, 0), 0) - sdf_textures[c].SampleLevel(linear_sampler, uvw - float3(EPSILON.x, 0, 0), 0),
        sdf_textures[c].SampleLevel(linear_sampler, uvw + float3(0, EPSILON.y, 0), 0) - sdf_textures[c].SampleLevel(linear_sampler, uvw - float3(0, EPSILON.y, 0), 0),
        sdf_textures[c].SampleLevel(linear_sampler, uvw + float3(0, 0, EPSILON.z), 0) - sdf_textures[c].SampleLevel(linear_sampler, uvw - float3(0, 0, EPSILON.z), 0)));
    float4 aniso0 = aniso0_textures[c].SampleLevel(linear_sampler, uvw, 0);
    float3 aniso1 = float3(aniso0.a, aniso1_textures[c].SampleLevel(linear_sampler, uvw, 0));
    light = light_textures[c].SampleLevel(linear_sampler, uvw, 0) *
        (dot(max(float3(0), normal * aniso0.rgb), float3(1)) + dot(max(float3(0), -normal * aniso1), float3(1)));
    break;
  }
  output_image[id.xy] = float4(saturate(linear_to_srgb(light)), 1);
}
