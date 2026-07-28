#extension GL_ARB_shading_language_include : enable

layout(set = 1, binding = 17) uniform sampler2D inDepth;

layout(set = 0, binding = 0) uniform sampler2D image;
layout(set = 0, binding = 1, r16f) uniform writeonly image2D outImage;

layout(push_constant) uniform BLUR_CONSTANTS {
  int horizontal;
  float camera_near;
  float camera_far;
  float avoid_distance;
  float weight0;
  float weight1;
  float weight2;
  float weight3;
  float weight4;
};

float BlurWeight(int index) {
  if (index == 0) {
    return weight0;
  }
  if (index == 1) {
    return weight1;
  }
  if (index == 2) {
    return weight2;
  }
  if (index == 3) {
    return weight3;
  }
  return weight4;
}

float LinearizeDepth(float ndcDepth) {
  return camera_near * camera_far / (camera_far - ndcDepth * (camera_far - camera_near));
}

[shader("compute")][numthreads(16, 16, 1)] void main(uint3 dispatchThreadId : SV_DispatchThreadID) {
  uvec2 pixel = dispatchThreadId.xy;
  uvec2 resolution = uvec2(imageSize(outImage));
  if (pixel.x >= resolution.x || pixel.y >= resolution.y) {
    return;
  }

  vec2 texCoord = (vec2(pixel) + vec2(0.5f)) / vec2(resolution);
  vec2 texOffset = 1.0 / textureSize(image, 0);
  vec4 current = textureLod(image, texCoord, 0.0f).rgba;
  vec4 result = current * BlurWeight(0);
  float depth = LinearizeDepth(textureLod(inDepth, texCoord, 0.0f).x);

  if (horizontal != 0) {
    for (int i = 1; i < 5; ++i) {
      float weight = BlurWeight(i);
      vec2 coord1 = texCoord + vec2(texOffset.x * i, 0.0);
      float depth1 = LinearizeDepth(textureLod(inDepth, coord1, 0.0f).x);
      result += abs(depth1 - depth) < avoid_distance ? textureLod(image, coord1, 0.0f).rgba * weight : current * weight;

      vec2 coord2 = texCoord - vec2(texOffset.x * i, 0.0);
      float depth2 = LinearizeDepth(textureLod(inDepth, coord2, 0.0f).x);
      result += abs(depth2 - depth) < avoid_distance ? textureLod(image, coord2, 0.0f).rgba * weight : current * weight;
    }
  } else {
    for (int i = 1; i < 5; ++i) {
      float weight = BlurWeight(i);
      vec2 coord1 = texCoord + vec2(0.0, texOffset.y * i);
      float depth1 = LinearizeDepth(textureLod(inDepth, coord1, 0.0f).x);
      result += abs(depth1 - depth) < avoid_distance ? textureLod(image, coord1, 0.0f).rgba * weight : current * weight;

      vec2 coord2 = texCoord - vec2(0.0, texOffset.y * i);
      float depth2 = LinearizeDepth(textureLod(inDepth, coord2, 0.0f).x);
      result += abs(depth2 - depth) < avoid_distance ? textureLod(image, coord2, 0.0f).rgba * weight : current * weight;
    }
  }

  imageStore(outImage, ivec2(pixel), result);
}
