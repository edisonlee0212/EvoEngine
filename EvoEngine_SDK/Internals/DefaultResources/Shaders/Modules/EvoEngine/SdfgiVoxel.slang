// Packing/facing adapted from Godot scene_forward_clustered.glsl::MODE_RENDER_SDF,
// 34d06658a85845111a50db9e485ec4a0701d4298. See docs/licenses/Godot-MIT.txt.
__exported import EvoEngine.SdfgiTypes;

struct SdfgiVoxelVertex {
  float4 position : SV_Position;
  [[vk::location(0)]] float3 world_position : TEXCOORD0;
  [[vk::location(1)]] float3 normal : TEXCOORD1;
  [[vk::location(2)]] float4 color : COLOR0;
  [[vk::location(3)]] float2 uv0 : TEXCOORD2;
  [[vk::location(4)]] float2 uv1 : TEXCOORD3;
};

static const float3 SDFGI_FACING_DIRECTIONS[6] = {
    float3(1, 0, 0), float3(0, 1, 0), float3(0, 0, 1),
    float3(-1, 0, 0), float3(0, -1, 0), float3(0, 0, -1)};

uint SdfgiPackAlbedo(float3 albedo) {
  uint3 color = min(uint3(max(albedo, float3(0)) * 31.0), uint3(31));
  return 1u | (color.r << 11) | (color.g << 6) | (color.b << 1);
}

uint SdfgiFacingBits(float3 normal) {
  float closest = -1e20;
  uint result = 0;
  for (uint i = 0; i < 6; ++i) {
    float d = dot(normal, SDFGI_FACING_DIRECTIONS[i]);
    if (d > closest) {
      closest = d;
      result = 1u << i;
    }
  }
  return result;
}

uint2 SdfgiPackEmission(float3 emission, float3 normal) {
  float lumas[6];
  float3 total = float3(0);
  for (uint i = 0; i < 6; ++i) {
    float3 light = emission * max(0.0, dot(normal, SDFGI_FACING_DIRECTIONS[i]));
    total += light;
    lumas[i] = max(light.r, max(light.g, light.b));
  }
  float luma_total = max(total.r, max(total.g, total.b));
  uint aniso = 0;
  for (uint i = 0; i < 6; ++i)
    aniso |= min(31u, uint((lumas[i] / luma_total) * 31.0)) << (i * 5);

  float3 c = clamp(total, float3(0), float3(65408));
  float c_max = max(c.r, max(c.g, c.b));
  const float LN2 = 0.6931471805599453094172321215;
  float expp = max(-16.0, floor(log(c_max) / LN2)) + 16.0;
  float s_max = floor(c_max / pow(2.0, expp - 24.0) + 0.5);
  float exps = (s_max >= 0.0 && s_max < 512.0) ? expp : expp + 1.0;
  uint3 s = uint3(floor(c / pow(2.0, exps - 24.0) + 0.5));
  uint rgbe = ((s.r & 0x1ffu) >> 1) | ((s.g & 0x1ffu) << 8) |
              (((s.b & 0x1ffu) >> 1) << 17) | ((uint(exps) & 0x1fu) << 25);
  return uint2(rgbe, aniso);
}
