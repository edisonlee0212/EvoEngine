struct EntitySelectionHighlightInput {
  [[vk::location(0)]] float2 tex_coord : TEXCOORD0;
};

struct EntitySelectionHighlightPushConstant {
  float4 outline_color;
  float4 settings;
};

[[vk::binding(24, 1)]] Sampler2D<float4> inUtility;
[[vk::push_constant]] EntitySelectionHighlightPushConstant push_constant;

bool IsSelected(int2 pixel, int2 dimensions) {
  int2 texel = clamp(pixel, int2(0), dimensions - int2(1));
  return (int(round(inUtility.Load(int3(texel, 0)).y)) & 1) != 0;
}

[shader("fragment")]
float4 main(EntitySelectionHighlightInput input) : SV_Target {
  uint width;
  uint height;
  inUtility.GetDimensions(width, height);
  int2 dimensions = int2(width, height);
  int2 pixel = clamp(int2(input.tex_coord * float2(width, height)), int2(0), dimensions - int2(1));
  if (IsSelected(pixel, dimensions)) return float4(0.0f);

  int radius = max(0, int(round(push_constant.settings.z)));
  for (int y = -radius; y <= radius; ++y) {
    for (int x = -radius; x <= radius; ++x) {
      int2 neighbor = pixel + int2(x, y);
      if (all(neighbor >= int2(0)) && all(neighbor < dimensions) && IsSelected(neighbor, dimensions)) {
        return float4(push_constant.outline_color.rgb, push_constant.outline_color.a * 0.9f);
      }
    }
  }
  float focus_alpha = clamp(push_constant.settings.x * push_constant.settings.y, 0.0f, 1.0f);
  return float4(0.5f, 0.5f, 0.5f, focus_alpha);
}
