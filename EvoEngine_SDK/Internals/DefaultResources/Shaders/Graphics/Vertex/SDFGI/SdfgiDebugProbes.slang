// Godot sdfgi_debug_probes.glsl, 34d06658a85845111a50db9e485ec4a0701d4298.
// See docs/licenses/Godot-MIT.txt. MODE_BOXES is a host-only bounds/classification overlay.
import EvoEngine.SdfgiDebug;

struct Output {
  float4 position : SV_Position;
  float3 normal : NORMAL0;
  nointerpolation uint probe : TEXCOORD0;
  float4 color : COLOR0;
};
float3 sphere_vertex(uint vertex) {
  float x_angle = (float(vertex & 1u) + (vertex >> 4)) * (6.28318530718 / 7) * 0.5;
  float y_angle = (float((vertex & 14u) >> 1) + (vertex >> 4) * 7) * -(6.28318530718 / 7);
  return float3(sin(x_angle) * sin(y_angle), cos(x_angle), sin(x_angle) * cos(y_angle));
}

Output main(uint vertex : SV_VertexID, uint instance : SV_InstanceID) {
  Output output = (Output)0;
  float3 position;
#ifdef MODE_BOXES
  static const uint edges[24] = {0,1, 0,2, 0,4, 1,3, 1,5, 2,3, 2,6, 3,7, 4,5, 4,6, 5,7, 6,7};
  float3 corner = float3((uint3(edges[vertex]) >> uint3(0, 1, 2)) & 1);
  position = lerp(boxes[instance].minimum.xyz, boxes[instance].maximum.xyz, corner);
  output.color = boxes[instance].color;
#else
  output.normal = sphere_vertex(vertex);
  output.probe = instance;
#ifdef MODE_VISIBILITY
  output.probe = data.selection.z;
  int3 probe_cell = debug_probe_cell(output.probe);
  int3 offset = int3(instance % 16, instance / (16 * 16), (instance / 16) % 16) - 8;
  uint c = data.selection.y;
  position = debug_probe_world(output.probe) + output.normal * 0.01 +
      (float3(offset) + 0.5) / (data.cascades[c].to_cell * float3(1, data.origin_y_mult.w, 1));
  int3 texel = probe_cell * 8 + offset;
  int3 global_cell = probe_cell + data.cascades[c].probe_world_offset;
  uint layer = (global_cell.x & 1) | ((global_cell.y & 1) << 1) | ((global_cell.z & 1) << 2);
  float visibility = 0;
  if (all(texel >= 0) && all(texel < debug_grid())) {
    texel.z += c * debug_grid().z;
    if (layer >= 4) { texel.x += debug_grid().x; layer -= 4; }
    visibility = occlusion_texture.Load(int4(texel, 0))[layer];
  }
  output.color = float4(1, visibility, visibility, 1);
#else
  position = debug_probe_world(instance) + output.normal * 0.2;
#endif
#endif
  output.position = mul(float4(position, 1), data.view_projection);
  return output;
}
