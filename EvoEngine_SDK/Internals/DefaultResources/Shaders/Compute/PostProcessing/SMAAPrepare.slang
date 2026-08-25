[[vk::binding(0, 0)]]
Sampler2D inputColor;

[[vk::binding(1, 0)]]
[vk::image_format("rgba32f")]
RWTexture2D<float4> blendColor;

[[vk::binding(2, 0)]]
[vk::image_format("rgba32f")]
RWTexture2D<float4> edgeColor;

struct EeSmaaConstants {
  float4 metrics;
  float4 subsample_indices;
  int tone_mapped;
  int debug_mode;
  int padding0;
  int padding1;
};

[[vk::push_constant]]
ConstantBuffer<EeSmaaConstants> constants;

float3 LinearToSrgb(float3 color) {
  float3 low = color * 12.92f;
  float3 high = 1.055f * pow(max(color, float3(0.0f)), float3(1.0f / 2.4f)) - 0.055f;
  return select(color > float3(0.0031308f), high, low);
}

float3 SrgbToLinear(float3 color) {
  float3 low = color / 12.92f;
  float3 high = pow((max(color, float3(0.0f)) + 0.055f) / 1.055f, float3(2.4f));
  return select(color > float3(0.04045f), high, low);
}

[shader("compute")]
[numthreads(8, 8, 1)]
void main(uint3 dispatch_thread_id : SV_DispatchThreadID) {
  int2 pixel = int2(dispatch_thread_id.xy);
  uint width = 0;
  uint height = 0;
  inputColor.GetDimensions(width, height);
  if (any(pixel >= int2(width, height))) return;

  float4 input_value = inputColor.Load(int3(pixel, 0));
  float4 blend_value = input_value;
  float4 edge_value = input_value;
  if (constants.tone_mapped != 0) {
    blend_value.rgb = SrgbToLinear(clamp(input_value.rgb, float3(0.0f), float3(1.0f)));
  } else {
    blend_value.rgb = max(input_value.rgb, float3(0.0f));
    float luminance = dot(blend_value.rgb, float3(0.2126f, 0.7152f, 0.0722f));
    edge_value.rgb = LinearToSrgb(clamp(blend_value.rgb / (1.0f + luminance), float3(0.0f), float3(1.0f)));
  }
  blendColor[pixel] = blend_value;
  edgeColor[pixel] = edge_value;
}
