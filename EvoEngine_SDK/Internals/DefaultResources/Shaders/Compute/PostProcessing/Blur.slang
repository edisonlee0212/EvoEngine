layout(local_size_x = 16, local_size_y = 16, local_size_z = 1) in;

layout(push_constant) uniform BLUR_CONSTANTS {
  int horizontal;
  float weight0;
  float weight1;
  float weight2;
  float weight3;
  float weight4;
};

layout(set = 0, binding = 0) uniform sampler2D image;
layout(set = 0, binding = 1, rgba32f) uniform writeonly image2D outImage;

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

void main() {
  uvec2 pixel = gl_GlobalInvocationID.xy;
  uvec2 resolution = uvec2(imageSize(outImage));
  if (pixel.x >= resolution.x || pixel.y >= resolution.y) {
    return;
  }

  vec2 texCoord = (vec2(pixel) + vec2(0.5f)) / vec2(resolution);
  vec2 texOffset = 1.0 / textureSize(image, 0);
  vec4 result = textureLod(image, texCoord, 0.0f).rgba * BlurWeight(0);

  if (horizontal != 0) {
    for (int i = 1; i < 5; ++i) {
      float weight = BlurWeight(i);
      result += textureLod(image, texCoord + vec2(texOffset.x * i, 0.0), 0.0f).rgba * weight;
      result += textureLod(image, texCoord - vec2(texOffset.x * i, 0.0), 0.0f).rgba * weight;
    }
  } else {
    for (int i = 1; i < 5; ++i) {
      float weight = BlurWeight(i);
      result += textureLod(image, texCoord + vec2(0.0, texOffset.y * i), 0.0f).rgba * weight;
      result += textureLod(image, texCoord - vec2(0.0, texOffset.y * i), 0.0f).rgba * weight;
    }
  }

  imageStore(outImage, ivec2(pixel), result);
}
