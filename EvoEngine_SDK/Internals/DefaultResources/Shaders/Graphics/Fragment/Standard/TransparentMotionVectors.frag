#extension GL_ARB_shading_language_include : enable

#include "BasicConstants.glsl"
#include "Basic.glsl"
#include "GltfRasterMaterial.glsl"

layout(location = 0) in vec2 inTexCoord;
layout(location = 1) in vec4 inCurrentClip;
layout(location = 2) in vec4 inPreviousClip;
layout(location = 3) in flat uint inInstanceIndex;
layout(location = 4) in flat uint inPreviousTransformValid;
layout(location = 5) in vec2 inTexCoord1;
layout(location = 6) in vec4 inColor;

layout(location = 0) out vec4 outMotionVectors;

void main() {
  GltfRasterMaterial surface = EE_EVALUATE_GLTF_RASTER_SURFACE(
      uint(EE_INSTANCES[inInstanceIndex].material_index), inTexCoord, inTexCoord1, inColor);
  if (EE_GLTF_RASTER_SHOULD_DISCARD(surface)) {
    discard;
  }
  if (inPreviousTransformValid == 0u || inCurrentClip.w <= 0.0001f || inPreviousClip.w <= 0.0001f) {
    outMotionVectors = vec4(65000.0f, 65000.0f, 0.0f, 0.0f);
    return;
  }

  vec2 resolution = vec2(EE_CAMERA_RESOLUTION_X(EE_CAMERA_INDEX), EE_CAMERA_RESOLUTION_Y(EE_CAMERA_INDEX));
  vec2 currentPixel = (inCurrentClip.xy / inCurrentClip.w * 0.5f + vec2(0.5f)) * resolution;
  vec2 previousPixel = (inPreviousClip.xy / inPreviousClip.w * 0.5f + vec2(0.5f)) * resolution;
  outMotionVectors = vec4(previousPixel - currentPixel, 0.0f, 0.0f);
}
