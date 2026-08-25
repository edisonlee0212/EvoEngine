struct BlurInput {
  [[vk::location(0)]]
  float2 tex_coord : TEXCOORD0;
};

struct BlurConstants {
  int horizontal;
  float weights[5];
};

[[vk::push_constant]]
ConstantBuffer<BlurConstants> constants;

[[vk::binding(0, 0)]]
Sampler2D image;

[shader("fragment")]
float4 main(BlurInput input) : SV_Target {
  uint width;
  uint height;
  image.GetDimensions(width, height);
  float2 texel_offset = 1.0f / float2(width, height);
  float4 result = image.Sample(input.tex_coord) * constants.weights[0];
  for (int i = 1; i < 5; ++i) {
    float2 offset = constants.horizontal != 0 ? float2(texel_offset.x * float(i), 0.0f)
                                               : float2(0.0f, texel_offset.y * float(i));
    result += image.Sample(input.tex_coord + offset) * constants.weights[i];
    result += image.Sample(input.tex_coord - offset) * constants.weights[i];
  }
  return result;
}
