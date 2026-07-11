#extension GL_ARB_shading_language_include : enable

#include "BasicConstants.glsl"
#include "Basic.glsl"
#include "GltfRasterMaterial.glsl"

layout(location = 0) in vec2 inTexCoord;
layout(location = 1) in vec4 inCurrentClip;
layout(location = 2) in vec4 inPreviousClip;
layout(location = 3) in flat uint inInstanceIndex;
layout(location = 4) in flat uint inPreviousPoseValid;
layout(location = 5) in vec2 inTexCoord1;
layout(location = 6) in vec4 inColor;

layout(location = 0) out vec4 outMotionVectors;

float NormalizedLinearDepth(vec4 clip) {
  mat4 projection = EE_CAMERAS[EE_CAMERA_INDEX].projection;
  float farDistance = abs(projection[3][2] / (projection[2][2] + 1.0f));
  return clamp(clip.w / max(farDistance, 1e-5f), 0.0f, 1.0f);
}

void main() {
  GltfRasterMaterial surface = EE_EVALUATE_GLTF_RASTER_SURFACE(
      uint(EE_INSTANCES[inInstanceIndex].material_index), inTexCoord, inTexCoord1, inColor);
  if (EE_GLTF_RASTER_SHOULD_DISCARD(surface)) {
    discard;
  }
  if (inPreviousPoseValid == 0u || inCurrentClip.w <= 0.0001f || inPreviousClip.w <= 0.0001f) {
    outMotionVectors = vec4(65000.0f, 65000.0f, 0.0f, 0.0f);
    return;
  }
  vec3 currentNdc = inCurrentClip.xyz / inCurrentClip.w;
  vec3 previousNdc = inPreviousClip.xyz / inPreviousClip.w;
  vec2 resolution = vec2(EE_CAMERA_RESOLUTION_X(EE_CAMERA_INDEX), EE_CAMERA_RESOLUTION_Y(EE_CAMERA_INDEX));
  vec2 currentPixel = (currentNdc.xy * 0.5f + vec2(0.5f)) * resolution;
  vec2 previousPixel = (previousNdc.xy * 0.5f + vec2(0.5f)) * resolution;
  outMotionVectors = vec4(previousPixel - currentPixel,
                          NormalizedLinearDepth(inPreviousClip) - NormalizedLinearDepth(inCurrentClip), 0.0f);
}
