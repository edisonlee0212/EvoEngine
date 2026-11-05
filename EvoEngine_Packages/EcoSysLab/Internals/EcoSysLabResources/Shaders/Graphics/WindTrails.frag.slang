import EcoSysLab.Noise;
struct Input {
  float4 position : SV_Position;
  [[vk::location(0)]] float4 color : COLOR0;
  [[vk::location(1)]] float2 shape : TEXCOORD0;
  [[vk::location(2)]] float4 smoke : TEXCOORD1;
};
[shader("fragment")]
float4 main(Input input) : SV_Target {
  // World-space noise stays continuous across recorded segments and history-buffer wraps.
  float3 p = input.smoke.xyz * 2.0 + float3(input.shape.x * 2.7, input.smoke.w, 0);
  float noise = 0.5 + 0.35 * EE_SIMPLEX_NOISE(p) + 0.15 * EE_SIMPLEX_NOISE(p * 2.03 + 17.0);
  float side = input.shape.x + 0.2 * (noise - 0.5);
  float feather = (1.0 - smoothstep(0.0, 1.0, abs(side))) *
                  (1.0 - smoothstep(0.8, 1.0, abs(input.shape.x)));
  float ends = smoothstep(0.0, 0.12, input.shape.y) * (1.0 - smoothstep(0.85, 1.0, input.shape.y));
  float density = smoothstep(0.15, 0.85, noise);
  return float4(input.color.rgb * lerp(0.85, 1.0, density), input.color.a * feather * ends * density);
}
