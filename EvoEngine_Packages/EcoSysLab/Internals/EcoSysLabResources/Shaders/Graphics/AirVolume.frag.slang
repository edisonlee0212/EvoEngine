import EcoSysLab.AirGrid;
import EvoEngine.PerFrame;

struct Parameters {
  uint camera_index;
  float cell_size;
  float ambient_temperature;
  uint bucket_count;
  float4 lower;
  float4 upper;
};
struct Input { float4 position : SV_Position; };

[[vk::binding(0, 1)]] StructuredBuffer<uint> heads;
[[vk::binding(1, 1)]] StructuredBuffer<AirGridAddresses> addresses;
[[vk::binding(0, 2)]] Texture2D<float> scene_depth;
[[vk::push_constant]] ConstantBuffer<Parameters> parameters;

[shader("fragment")]
float4 main(Input input) : SV_Target {
  int2 pixel = int2(input.position.xy);
  float2 uv = (float2(pixel) + 0.5) /
              float2(EE_CAMERA_RESOLUTION_X(parameters.camera_index), EE_CAMERA_RESOLUTION_Y(parameters.camera_index));
  float3 origin = EE_CAMERA_POSITION(parameters.camera_index);
  float3 far_position = EE_DEPTH_TO_WORLD_POS(parameters.camera_index, uv, 1.0);
  float3 direction = normalize(far_position - origin);
  float3 safe_direction = select(abs(direction) < 1e-6, select(direction < 0, -1e-6, 1e-6), direction);
  float3 a = (parameters.lower.xyz - origin) / safe_direction;
  float3 b = (parameters.upper.xyz - origin) / safe_direction;
  float3 near_axis = min(a, b);
  float3 far_axis = max(a, b);
  float near_distance = max(0.0, max(near_axis.x, max(near_axis.y, near_axis.z)));
  float far_distance = min(far_axis.x, min(far_axis.y, far_axis.z));
  if (near_distance >= far_distance) discard;
  float depth = scene_depth.Load(int3(pixel, 0));
  if (depth < 1.0) {
    float3 surface = EE_DEPTH_TO_WORLD_POS(parameters.camera_index, uv, depth);
    far_distance = min(far_distance, length(surface - origin));
  }
  if (near_distance >= far_distance) discard;

  float3 accumulated = 0;
  float opacity = 0;
  float distance_along = near_distance;
  uint step_limit = min(1024u, uint(ceil(64.0 / parameters.cell_size)));
  for (uint step = 0; step < step_limit && distance_along < far_distance && opacity < 0.97; ++step) {
    float3 point = origin + direction * (distance_along + 1e-4);
    int3 cell = int3(floor(point / parameters.cell_size));
    float3 exit_position = (float3(cell) + select(direction >= 0, 1.0, 0.0)) * parameters.cell_size;
    float3 exit_distance = (exit_position - origin) / safe_direction;
    float next_distance = min(far_distance, min(exit_distance.x, min(exit_distance.y, exit_distance.z)));
    next_distance = max(next_distance, distance_along + 1e-4);
    uint index = AirGridFind(heads, addresses, cell, parameters.bucket_count);
    if (index != 0xffffffffu) {
      AirGridValue air = AirGridReadValue(addresses, index);
      float speed = length(air.velocity_temperature.xyz);
      float alpha = 1.0 - exp(-max(speed * 0.09, parameters.lower.w) * (next_distance - distance_along));
      float kelvin = air.velocity_temperature.w > 0.0 ? air.velocity_temperature.w : parameters.ambient_temperature;
      float temperature = clamp((kelvin - parameters.ambient_temperature) / 20.0, -1.0, 1.0);
      float3 neutral = float3(0.82, 0.84, 0.82);
      float3 color = temperature < 0.0 ? lerp(neutral, float3(0.12, 0.42, 1.0), -temperature)
                                       : lerp(neutral, float3(1.0, 0.38, 0.08), temperature);
      if (parameters.lower.w > 0.0 && speed < 0.05) color = float3(0.25, 0.85, 0.55);
      accumulated += (1.0 - opacity) * alpha * color;
      opacity += (1.0 - opacity) * alpha;
    }
    distance_along = next_distance + 1e-4;
  }
  if (opacity < 1e-4) discard;
  return float4(accumulated / opacity, opacity);
}
