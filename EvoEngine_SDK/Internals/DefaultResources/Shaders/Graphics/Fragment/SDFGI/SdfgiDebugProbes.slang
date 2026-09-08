// Godot sdfgi_debug_probes.glsl, 34d06658a85845111a50db9e485ec4a0701d4298.
// See docs/licenses/Godot-MIT.txt. Depth is sampled from the host camera without modifying it.
import EvoEngine.SdfgiDebug;
float4 main(float4 position : SV_Position, float3 normal : NORMAL0,
    nointerpolation uint probe : TEXCOORD0, float4 color : COLOR0) : SV_Target0 {
  if (data.field.z != 0 && position.z > scene_depth.Load(int3(int2(position.xy), 0)) + 0.00001) discard;
#if defined(MODE_VISIBILITY) || defined(MODE_BOXES)
  return color;
#else
  return float4(debug_probe_color(probe, normal), 1);
#endif
}
