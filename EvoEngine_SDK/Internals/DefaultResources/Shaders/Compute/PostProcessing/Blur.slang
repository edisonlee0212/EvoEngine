// @evoengine-dialect native
struct BlurConstants {
  int horizontal;
  float weight0;
  float weight1;
  float weight2;
  float weight3;
  float weight4;
  float padding0;
  float padding1;
};

[[vk::push_constant]]
ConstantBuffer<BlurConstants> constants;

[[vk::binding(0, 0)]]
Sampler2D image;

[[vk::binding(1, 0)]]
[vk::image_format("rgba32f")]
RWTexture2D<float4> outImage;

float BlurWeight(int index) {
  if (index == 0) return constants.weight0;
  if (index == 1) return constants.weight1;
  if (index == 2) return constants.weight2;
  if (index == 3) return constants.weight3;
  return constants.weight4;
}

[shader("compute")]
[numthreads(16, 16, 1)]
void main(uint3 dispatch_thread_id : SV_DispatchThreadID) {
  uint2 pixel = dispatch_thread_id.xy;
  uint width = 0;
  uint height = 0;
  outImage.GetDimensions(width, height);
  uint2 resolution = uint2(width, height);
  if (any(pixel >= resolution)) return;

  float2 tex_coord = (float2(pixel) + float2(0.5f)) / float2(resolution);
  uint source_width = 0;
  uint source_height = 0;
  image.GetDimensions(source_width, source_height);
  float2 tex_offset = 1.0f / float2(source_width, source_height);
  float4 result = image.SampleLevel(tex_coord, 0.0f) * BlurWeight(0);

  for (int i = 1; i < 5; ++i) {
    float weight = BlurWeight(i);
    float2 offset = constants.horizontal != 0 ? float2(tex_offset.x * i, 0.0f) : float2(0.0f, tex_offset.y * i);
    result += image.SampleLevel(tex_coord + offset, 0.0f) * weight;
    result += image.SampleLevel(tex_coord - offset, 0.0f) * weight;
  }
  outImage[int2(pixel)] = result;
}
