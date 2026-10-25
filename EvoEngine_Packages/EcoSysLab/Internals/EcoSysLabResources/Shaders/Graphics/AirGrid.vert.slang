import EcoSysLab.AirGrid;
import EvoEngine.PerFrame;

struct Parameters {
  uint camera_index;
  float cell_size;
  float velocity_scale;
  float padding;
};
struct Output {
  float4 position : SV_Position;
  [[vk::location(0)]] float4 color : COLOR0;
};

[[vk::binding(0, 1)]] StructuredBuffer<AirGridAddresses> addresses;
[[vk::push_constant]] ConstantBuffer<Parameters> parameters;

static const uint2 edges[12] = {
  uint2(0, 1), uint2(1, 3), uint2(3, 2), uint2(2, 0),
  uint2(4, 5), uint2(5, 7), uint2(7, 6), uint2(6, 4),
  uint2(0, 4), uint2(1, 5), uint2(2, 6), uint2(3, 7)
};

[shader("vertex")]
Output main(uint vertex_id : SV_VertexID, uint instance_id : SV_InstanceID) {
  int3 coordinate = AirGridReadCell(addresses, instance_id).coordinate_next.xyz;
  float3 center = (float3(coordinate) + 0.5) * parameters.cell_size;
  float3 velocity = AirGridReadValue(addresses, instance_id).velocity_temperature.xyz;
  float speed = length(velocity);
  float3 direction = speed > 1e-5 ? velocity / speed : float3(0, 1, 0);
  float3 position;
  Output output;
  if (vertex_id < 24) {
    uint corner = (vertex_id & 1) != 0 ? edges[vertex_id / 2].y : edges[vertex_id / 2].x;
    position = (float3(coordinate) + float3(corner & 1, (corner >> 1) & 1, (corner >> 2) & 1)) *
               parameters.cell_size;
    output.color = float4(lerp(float3(0.15, 0.45, 0.8), float3(1, 0.55, 0.15),
                                saturate(speed / 10.0)), 0.25);
  } else {
    float length = min(speed * parameters.velocity_scale, parameters.cell_size * 0.65);
    float3 tip = center + direction * length;
    float3 side = normalize(cross(direction, abs(direction.y) < 0.9 ? float3(0, 1, 0) : float3(1, 0, 0)));
    float3 wing = direction * min(length * 0.3, parameters.cell_size * 0.15);
    float3 spread = side * min(length * 0.2, parameters.cell_size * 0.1);
    switch (vertex_id - 24) {
      case 0: position = center; break;
      case 1: case 2: case 4: position = tip; break;
      case 3: position = tip - wing + spread; break;
      default: position = tip - wing - spread; break;
    }
    output.color = float4(0.95, 0.9, 0.3, speed > 1e-5 ? 0.9 : 0.0);
  }
  output.position = mul(float4(position, 1), EE_CAMERAS[parameters.camera_index].projection_view);
  return output;
}
