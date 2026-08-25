#extension GL_ARB_shading_language_include : enable

layout(location = 0) out vec4 FragColor;

layout(location = 0) in VS_OUT {
  vec2 TexCoord;
}
fs_in;

layout(set = 1, binding = 17) uniform sampler2D inDepth;
layout(set = 1, binding = 18) uniform sampler2D inNormal;
layout(set = 1, binding = 19) uniform sampler2D inMaterial;

layout(set = 0, binding = 0) uniform sampler2D image;

layout(push_constant) uniform BLUR_CONSTANTS {
  int horizontal;
  float camera_near;
  float camera_far;
  float avoid_distance;
  float weight[5];
};

float linearize_depth(float ndcDepth) {
  return camera_near * camera_far / (camera_far - ndcDepth * (camera_far - camera_near));
}

void main() {
  vec2 tex_offset = 1.0 / textureSize(image, 0);  // gets size of single texel
  vec4 current = texture(image, fs_in.TexCoord).rgba;

  vec4 result = current * weight[0];  // current fragment's contribution

  float depth = linearize_depth(texture(inDepth, fs_in.TexCoord).x);

  float max_depth_diff = avoid_distance;

  if (horizontal != 0) {
    for (int i = 1; i < 5; ++i) {
      vec2 coord1 = fs_in.TexCoord + vec2(tex_offset.x * i, 0.0);
      float depth1 = linearize_depth(texture(inDepth, coord1).x);

      if (abs(depth1 - depth) < max_depth_diff)
        result += texture(image, coord1).rgba * weight[i];
      else
        result += current * weight[i];

      vec2 coord2 = fs_in.TexCoord - vec2(tex_offset.x * i, 0.0);
      float depth2 = linearize_depth(texture(inDepth, coord2).x);

      if (abs(depth2 - depth) < max_depth_diff)
        result += texture(image, coord2).rgba * weight[i];
      else
        result += current * weight[i];
    }
  } else {
    for (int i = 1; i < 5; ++i) {
      vec2 coord1 = fs_in.TexCoord + vec2(0.0, tex_offset.y * i);
      float depth1 = linearize_depth(texture(inDepth, coord1).x);
      if (abs(depth1 - depth) < max_depth_diff)
        result += texture(image, coord1).rgba * weight[i];
      else
        result += current * weight[i];

      vec2 coord2 = fs_in.TexCoord - vec2(0.0, tex_offset.y * i);
      float depth2 = linearize_depth(texture(inDepth, coord2).x);
      if (abs(depth2 - depth) < max_depth_diff)
        result += texture(image, coord2).rgba * weight[i];
      else
        result += current * weight[i];
    }
  }
  FragColor = result;
}