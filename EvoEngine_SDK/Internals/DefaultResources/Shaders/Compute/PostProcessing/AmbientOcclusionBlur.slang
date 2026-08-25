[[vk::binding(17, 1)]]
Sampler2D inDepth;

[[vk::binding(0, 0)]]
Sampler2D image;

[[vk::binding(1, 0)]]
[vk::image_format("r16f")]
RWTexture2D<float> outImage;

struct BlurConstants {
  int horizontal;
  float camera_near;
  float camera_far;
  float denoise_radius;
  float weight0;
  float weight1;
  float weight2;
  float weight3;
  float weight4;
  float padding0;
  float padding1;
  float padding2;
};

[[vk::push_constant]]
ConstantBuffer<BlurConstants> constants;

float BlurWeight(int index) {
  if (index == 0) return constants.weight0;
  if (index == 1) return constants.weight1;
  if (index == 2) return constants.weight2;
  if (index == 3) return constants.weight3;
  return constants.weight4;
}

float LinearizeDepth(float ndc_depth) {
  return constants.camera_near * constants.camera_far /
         (constants.camera_far - ndc_depth * (constants.camera_far - constants.camera_near));
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
  float current = image.SampleLevel(tex_coord, 0.0f).r;
  float result = current * BlurWeight(0);
  float depth = LinearizeDepth(inDepth.SampleLevel(tex_coord, 0.0f).x);

  for (int i = 1; i < 5; ++i) {
    float weight = BlurWeight(i);
    float2 offset = constants.horizontal != 0 ? float2(tex_offset.x * i, 0.0f) : float2(0.0f, tex_offset.y * i);
    float2 coord1 = tex_coord + offset;
    float depth1 = LinearizeDepth(inDepth.SampleLevel(coord1, 0.0f).x);
    result += abs(depth1 - depth) < constants.denoise_radius ? image.SampleLevel(coord1, 0.0f).r * weight
                                                              : current * weight;

    float2 coord2 = tex_coord - offset;
    float depth2 = LinearizeDepth(inDepth.SampleLevel(coord2, 0.0f).x);
    result += abs(depth2 - depth) < constants.denoise_radius ? image.SampleLevel(coord2, 0.0f).r * weight
                                                              : current * weight;
  }
  outImage[int2(pixel)] = result;
}
