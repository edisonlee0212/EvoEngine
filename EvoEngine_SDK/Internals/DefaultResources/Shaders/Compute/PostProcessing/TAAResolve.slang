#extension GL_ARB_shading_language_include : enable

#include "PerFrame.slangh"

// Algorithm and constants are adapted from GameTechDev/TAA (MIT).
#ifndef EE_TAA_USE_TGSM
#define EE_TAA_USE_TGSM 0
#endif
#ifndef EE_TAA_USE_FP16
#define EE_TAA_USE_FP16 0
#endif
#if EE_TAA_USE_FP16
#extension GL_EXT_shader_explicit_arithmetic_types_float16 : require
#endif

layout(set = 1, binding = 17) uniform sampler2D inDepth;
layout(set = 2, binding = 0) uniform sampler2D currentColor;
layout(set = 2, binding = 1) uniform sampler2D historyColor;
layout(set = 2, binding = 2) uniform sampler2D motionVectors;
layout(set = 2, binding = 3) uniform sampler2D previousDepthHistory;
layout(set = 2, binding = 4, rgba32f) uniform writeonly image2D outColor;
layout(set = 2, binding = 5, rgba32f) uniform writeonly image2D outHistory;
layout(set = 2, binding = 6, rgba32f) uniform writeonly image2D outDepthHistory;

layout(push_constant) uniform EE_TAA_CONSTANTS {
  int EE_CAMERA_INDEX;
  int historyValid;
  int frameIndex;
  int varianceClippingMode;
  int varianceSampleCount;
  int useYCoCg;
  int useNeighborhoodSampling;
  int useBicubicFilter;
  int useLongestVelocity;
  int longestVelocitySampleCount;
  int useDepthThreshold;
  int historyColorMode;
  int debugMode;
  int padding0;
  int padding1;
  int padding2;
  float minVarianceGamma;
  float maxVarianceGamma;
  float velocityRejectionThreshold;
  float depthThreshold;
  float sharpen;
  float padding3;
  float padding4;
  float padding5;
};

const int EE_TAA_VARIANCE_DISABLED = 0;
const int EE_TAA_VARIANCE_CLAMP = 1;
const int EE_TAA_VARIANCE_INTERSECTION = 2;
const int EE_TAA_HISTORY_TONE_MAPPED = 0;
const int EE_TAA_DEBUG_MOTION = 1;
const int EE_TAA_DEBUG_DEPTH_CONFIDENCE = 2;
const int EE_TAA_DEBUG_HISTORY_CONFIDENCE = 3;
const int EE_TAA_DEBUG_NO_HISTORY = 4;
const float EE_TAA_INTERSECTION_MAX_T = 100.0f;
const float EE_TAA_FP16_MIN_REINHARD_HEADROOM = 1.0f / 256.0f;
const float EE_TAA_INVALID_VELOCITY = 32768.0f;
const float EE_TAA_HISTORY_LUMINANCE_SCALE = 2.0f;
const float EE_TAA_HISTORY_LUMINANCE_BIAS = 0.25f;

#if EE_TAA_USE_TGSM
const int EE_TAA_TILE_WIDTH = 10;
const int EE_TAA_TILE_SIZE = EE_TAA_TILE_WIDTH * EE_TAA_TILE_WIDTH;
shared vec4 eeTaaCurrentTile[EE_TAA_TILE_SIZE];
#endif

float LuminanceRec709(vec3 rgb) {
  return dot(rgb, vec3(0.2126f, 0.7152f, 0.0722f));
}

bool IsFiniteColor(vec3 rgb) {
  return !any(isnan(rgb)) && !any(isinf(rgb));
}

bool IsValidWorkingColor(vec3 rgb) {
  return IsFiniteColor(rgb) && all(greaterThanEqual(rgb, vec3(0.0f))) && LuminanceRec709(rgb) < 1.0f;
}

bool IsStableFp16WorkingColor(vec3 rgb) {
  return IsValidWorkingColor(rgb) && 1.0f - LuminanceRec709(rgb) >= EE_TAA_FP16_MIN_REINHARD_HEADROOM;
}

vec3 Reinhard(vec3 rgb) {
  if (!IsFiniteColor(rgb)) {
    return vec3(0.0f);
  }
  rgb = max(rgb, vec3(0.0f));
  vec3 result = rgb / (1.0f + LuminanceRec709(rgb));
#if EE_TAA_USE_FP16
  vec3 fp16Result = vec3(f16vec3(result));
  if (IsStableFp16WorkingColor(fp16Result)) {
    result = fp16Result;
  }
#endif
  return result;
}

vec3 InverseReinhard(vec3 rgb, vec3 fallback) {
  if (!IsValidWorkingColor(rgb)) {
    return fallback;
  }
  vec3 result = rgb / (1.0f - LuminanceRec709(rgb));
  return IsFiniteColor(result) ? max(result, vec3(0.0f)) : fallback;
}

vec3 RgbToYCoCg(vec3 rgb) {
  if (useYCoCg == 0) {
    return rgb;
  }
  return vec3(dot(rgb, vec3(0.25f, 0.5f, 0.25f)), dot(rgb, vec3(0.5f, 0.0f, -0.5f)),
              dot(rgb, vec3(-0.25f, 0.5f, -0.25f)));
}

vec3 YCoCgToRgb(vec3 ycocg) {
  if (useYCoCg == 0) {
    return ycocg;
  }
  return vec3(dot(ycocg, vec3(1.0f, 1.0f, -1.0f)), dot(ycocg, vec3(1.0f, 0.0f, 1.0f)),
              dot(ycocg, vec3(1.0f, -1.0f, -1.0f)));
}

vec3 FetchCurrentHdr(ivec2 texel, ivec2 maxTexel) {
  vec3 result = texelFetch(currentColor, clamp(texel, ivec2(0), maxTexel), 0).rgb;
  return IsFiniteColor(result) ? max(result, vec3(0.0f)) : vec3(0.0f);
}

void StoreCurrentColorToShared(ivec2 groupBase, ivec2 maxTexel, uint localInvocationIndex) {
#if EE_TAA_USE_TGSM
  for (uint index = localInvocationIndex; index < uint(EE_TAA_TILE_SIZE); index += 64u) {
    ivec2 tileTexel = ivec2(int(index) % EE_TAA_TILE_WIDTH, int(index) / EE_TAA_TILE_WIDTH);
    ivec2 sourceTexel = clamp(groupBase + tileTexel - ivec2(1), ivec2(0), maxTexel);
    eeTaaCurrentTile[index] = vec4(Reinhard(FetchCurrentHdr(sourceTexel, maxTexel)), 1.0f);
  }
  memoryBarrierShared();
  barrier();
#endif
}

vec3 FetchCurrentWorking(ivec2 texel, ivec2 groupBase, ivec2 maxTexel) {
#if EE_TAA_USE_TGSM
  ivec2 tileTexel = clamp(texel - groupBase + ivec2(1), ivec2(0), ivec2(EE_TAA_TILE_WIDTH - 1));
  return eeTaaCurrentTile[tileTexel.y * EE_TAA_TILE_WIDTH + tileTexel.x].rgb;
#else
  return Reinhard(FetchCurrentHdr(texel, maxTexel));
#endif
}

vec3 ClipToAabb(vec3 history, vec3 current, vec3 center, vec3 extents) {
  vec3 direction = current - history;
  vec3 boundary = center - sign(direction) * extents;
  vec3 possibleT = vec3(EE_TAA_INTERSECTION_MAX_T + 1.0f);
  if (abs(direction.x) > 1e-6f) {
    float value = (boundary.x - history.x) / direction.x;
    possibleT.x = value >= 0.0f ? value : EE_TAA_INTERSECTION_MAX_T + 1.0f;
  }
  if (abs(direction.y) > 1e-6f) {
    float value = (boundary.y - history.y) / direction.y;
    possibleT.y = value >= 0.0f ? value : EE_TAA_INTERSECTION_MAX_T + 1.0f;
  }
  if (abs(direction.z) > 1e-6f) {
    float value = (boundary.z - history.z) / direction.z;
    possibleT.z = value >= 0.0f ? value : EE_TAA_INTERSECTION_MAX_T + 1.0f;
  }
  float t = min(EE_TAA_INTERSECTION_MAX_T, min(possibleT.x, min(possibleT.y, possibleT.z)));
  return t < EE_TAA_INTERSECTION_MAX_T ? history + direction * t : history;
}

vec3 ClipHistoryColor(vec3 current, vec3 history, ivec2 texel, ivec2 groupBase, ivec2 maxTexel,
                      float varianceGamma) {
  if (varianceClippingMode == EE_TAA_VARIANCE_DISABLED) {
    return history;
  }

  const ivec2 offsets9[8] = ivec2[](ivec2(-1, -1), ivec2(-1, 0), ivec2(-1, 1), ivec2(0, 1),
                                    ivec2(1, 1),   ivec2(1, 0),  ivec2(1, -1), ivec2(0, -1));
  const ivec2 offsetsAxial[4] = ivec2[](ivec2(-1, 0), ivec2(0, 1), ivec2(1, 0), ivec2(0, -1));

  vec3 currentSpace = RgbToYCoCg(current);
  vec3 moment1 = currentSpace;
  vec3 moment2 = currentSpace * currentSpace;
  int sampleCount = varianceSampleCount >= 9 ? 9 : 5;
  int neighborCount = sampleCount - 1;
  for (int index = 0; index < neighborCount; ++index) {
    ivec2 offset = sampleCount == 9 ? offsets9[index] : offsetsAxial[index];
    vec3 sampleColor = RgbToYCoCg(FetchCurrentWorking(texel + offset, groupBase, maxTexel));
    moment1 += sampleColor;
    moment2 += sampleColor * sampleColor;
  }

  float reciprocalSampleCount = 1.0f / float(sampleCount);
  vec3 mean = moment1 * reciprocalSampleCount;
  vec3 standardDeviation = sqrt(max(moment2 * reciprocalSampleCount - mean * mean, vec3(0.0f))) * varianceGamma;
  vec3 historySpace = RgbToYCoCg(history);
  if (varianceClippingMode == EE_TAA_VARIANCE_CLAMP) {
    return clamp(history, YCoCgToRgb(mean - standardDeviation), YCoCgToRgb(mean + standardDeviation));
  }
  return YCoCgToRgb(ClipToAabb(historySpace, currentSpace, mean, standardDeviation));
}

vec4 BicubicHistory(vec2 historySt, vec2 resolution) {
  vec2 reciprocalResolution = 1.0f / resolution;
  vec2 fractional = fract(historySt);
  vec2 uv = (floor(historySt) + vec2(0.5f)) * reciprocalResolution;
  vec2 t = fractional;
  vec2 t2 = t * t;
  vec2 t3 = t2 * t;
  const float s = 0.5f;
  vec2 w0 = -s * t3 + 2.0f * s * t2 - s * t;
  vec2 w1 = (2.0f - s) * t3 + (s - 3.0f) * t2 + vec2(1.0f);
  vec2 w2 = (s - 2.0f) * t3 + (3.0f - 2.0f * s) * t2 + s * t;
  vec2 w3 = s * t3 - s * t2;
  vec2 s0 = w1 + w2;
  vec2 f0 = w2 / max(s0, vec2(1e-6f));
  vec2 m0 = uv + f0 * reciprocalResolution;
  vec2 tc0 = uv - reciprocalResolution;
  vec2 tc3 = uv + 2.0f * reciprocalResolution;

  vec4 a = textureLod(historyColor, vec2(m0.x, tc0.y), 0.0f);
  vec4 b = textureLod(historyColor, vec2(tc0.x, m0.y), 0.0f);
  vec4 c = textureLod(historyColor, m0, 0.0f);
  vec4 d = textureLod(historyColor, vec2(tc3.x, m0.y), 0.0f);
  vec4 e = textureLod(historyColor, vec2(m0.x, tc3.y), 0.0f);
  return (0.5f * (a + b) * w0.x + a * s0.x + 0.5f * (a + b) * w3.x) * w0.y +
         (b * w0.x + c * s0.x + d * w3.x) * s0.y +
         (0.5f * (b + e) * w0.x + e * s0.x + 0.5f * (d + e) * w3.x) * w3.y;
}

vec4 GetHistory(vec2 historyUv, vec2 historySt, bool isOnEdge, vec2 resolution) {
  bool useBilinearOnEdges = useLongestVelocity == 0;
  bool useBicubic = useLongestVelocity != 0 || (!isOnEdge && useBilinearOnEdges);
  vec4 result = useBicubicFilter != 0 && useBicubic ? BicubicHistory(historySt, resolution)
                                                    : textureLod(historyColor, historyUv, 0.0f);
  if (historyColorMode != EE_TAA_HISTORY_TONE_MAPPED) {
    result.rgb = Reinhard(result.rgb);
  }
  result.a = clamp(result.a, 0.0f, 1.0f);
  return result;
}

vec3 GetVelocity(ivec2 texel, ivec2 maxTexel) {
  vec3 result = texelFetch(motionVectors, clamp(texel, ivec2(0), maxTexel), 0).xyz;
  bool invalidResult = !IsFiniteColor(result) || any(greaterThanEqual(abs(result.xy), vec2(EE_TAA_INVALID_VELOCITY)));
  if (useLongestVelocity == 0 || invalidResult) {
    return result;
  }
  const ivec2 offsets9[8] = ivec2[](ivec2(-1, -1), ivec2(-1, 0), ivec2(-1, 1), ivec2(0, 1),
                                    ivec2(1, 1),   ivec2(1, 0),  ivec2(1, -1), ivec2(0, -1));
  const ivec2 offsets5[4] = ivec2[](ivec2(-1, -1), ivec2(-1, 1), ivec2(1, 1), ivec2(1, -1));
  int neighborCount = longestVelocitySampleCount >= 9 ? 8 : 4;
  float longestLengthSquared = dot(result.xy, result.xy);
  for (int index = 0; index < neighborCount; ++index) {
    ivec2 offset = longestVelocitySampleCount >= 9 ? offsets9[index] : offsets5[index];
    vec3 candidate = texelFetch(motionVectors, clamp(texel + offset, ivec2(0), maxTexel), 0).xyz;
    if (!IsFiniteColor(candidate) ||
        any(greaterThanEqual(abs(candidate.xy), vec2(EE_TAA_INVALID_VELOCITY)))) {
      continue;
    }
    float candidateLengthSquared = dot(candidate.xy, candidate.xy);
    if (candidateLengthSquared > longestLengthSquared) {
      result = candidate;
      longestLengthSquared = candidateLengthSquared;
    }
  }
  return result;
}

float NormalizedLinearDepth(float ndcDepth) {
  if (ndcDepth >= 1.0f) {
    return 1.0f;
  }
  vec4 viewPosition = EE_CAMERAS[EE_CAMERA_INDEX].inverse_projection * vec4(0.0f, 0.0f, ndcDepth, 1.0f);
  mat4 projection = EE_CAMERAS[EE_CAMERA_INDEX].projection;
  float farDistance = abs(projection[3][2] / (projection[2][2] + 1.0f));
  return clamp(abs(viewPosition.z / viewPosition.w) / max(farDistance, 1e-5f), 0.0f, 1.0f);
}

float GetCurrentDepth(ivec2 texel, ivec2 maxTexel, out bool isOnEdge) {
  float depths[4] = float[](NormalizedLinearDepth(texelFetch(inDepth, clamp(texel, ivec2(0), maxTexel), 0).x),
                            NormalizedLinearDepth(texelFetch(inDepth, clamp(texel + ivec2(1, 0), ivec2(0), maxTexel), 0).x),
                            NormalizedLinearDepth(texelFetch(inDepth, clamp(texel + ivec2(0, 1), ivec2(0), maxTexel), 0).x),
                            NormalizedLinearDepth(texelFetch(inDepth, clamp(texel + ivec2(1, 1), ivec2(0), maxTexel), 0).x));
  float minDepth = min(min(depths[0], depths[1]), min(depths[2], depths[3]));
  float maxDepth = max(max(depths[0], depths[1]), max(depths[2], depths[3]));
  isOnEdge = maxDepth - minDepth > depthThreshold;
  return minDepth;
}

float GetPreviousDepth(vec2 uv) {
  vec4 depths = textureGather(previousDepthHistory, uv, 2);
  return max(max(depths.x, depths.y), max(depths.z, depths.w)) + 0.001f;
}

float BoundsConfidence(vec2 uv) {
  return all(greaterThanEqual(uv, vec2(0.0f))) && all(lessThan(uv, vec2(1.0f))) ? 1.0f : 0.0f;
}

float GetDepthConfidence(vec2 previousDepthUv, vec3 velocity, float currentDepth, bool isOnEdge) {
  if (useDepthThreshold == 0) {
    return 1.0f;
  }
  bool needsEdgeDetection = useBicubicFilter == 0 && useLongestVelocity == 0;
  if (needsEdgeDetection && isOnEdge) {
    return 1.0f;
  }
  return step(currentDepth + velocity.z, GetPreviousDepth(previousDepthUv));
}

vec3 GetCurrentNeighborhood(vec3 current, ivec2 texel, ivec2 groupBase, ivec2 maxTexel) {
  if (useNeighborhoodSampling == 0) {
    return current;
  }
  const ivec2 offsets[4] = ivec2[](ivec2(-1, -1), ivec2(-1, 1), ivec2(1, 1), ivec2(1, -1));
  vec3 result = current;
  for (int index = 0; index < 4; ++index) {
    result += FetchCurrentWorking(texel + offsets[index], groupBase, maxTexel);
  }
  return result * 0.2f;
}

[shader("compute")][numthreads(8, 8, 1)] void main(uint3 dispatchThreadId : SV_DispatchThreadID,
                                                    uint3 groupId : SV_GroupID,
                                                    uint localInvocationIndex : SV_GroupIndex) {
  ivec2 resolution = imageSize(outColor);
  ivec2 maxTexel = resolution - ivec2(1);
  ivec2 texel = ivec2(dispatchThreadId.xy);
  ivec2 groupBase = ivec2(groupId.xy) * ivec2(8);
  StoreCurrentColorToShared(groupBase, maxTexel, localInvocationIndex);
  if (any(greaterThanEqual(texel, resolution))) {
    return;
  }

  vec2 resolutionFloat = vec2(resolution);
  vec3 currentHdr = FetchCurrentHdr(texel, maxTexel);
  vec3 currentWorking = FetchCurrentWorking(texel, groupBase, maxTexel);
  vec3 velocity = GetVelocity(texel, maxTexel);
  float scaledVelocityThreshold = velocityRejectionThreshold * max(resolutionFloat.y / 1080.0f, 1e-4f);
  float velocityConfidence = clamp(1.0f - length(velocity.xy) / max(scaledVelocityThreshold, 1.0f), 0.0f, 1.0f);

  vec2 previousSt = vec2(texel) + velocity.xy;
  vec2 previousUv = (previousSt + vec2(0.5f)) / resolutionFloat;
  vec2 jitterPixels = (EE_CAMERAS[EE_CAMERA_INDEX].jitter.zw - EE_CAMERAS[EE_CAMERA_INDEX].jitter.xy) * 0.5f *
                      resolutionFloat;
  vec2 previousDepthUv = (previousSt + jitterPixels + vec2(0.5f)) / resolutionFloat;

  bool isOnEdge = false;
  float currentDepth = GetCurrentDepth(texel, maxTexel, isOnEdge);
  float uvConfidence = BoundsConfidence(previousUv);
  float depthUvConfidence = BoundsConfidence(previousDepthUv);
  float depthConfidence = depthUvConfidence > 0.0f
                              ? GetDepthConfidence(previousDepthUv, velocity, currentDepth, isOnEdge)
                              : 0.0f;
  float geometricConfidence = velocityConfidence * depthConfidence * uvConfidence * depthUvConfidence;
  bool hasValidHistory = historyValid != 0 && geometricConfidence > 0.0f;

  vec3 finalWorking;
  float newWeight;
  float historyConfidence = 0.0f;
  if (hasValidHistory) {
    vec4 rawHistory = GetHistory(previousUv, previousSt, isOnEdge, resolutionFloat);
    float varianceGamma = mix(minVarianceGamma, maxVarianceGamma, velocityConfidence * velocityConfidence);
    vec3 clippedHistory =
        ClipHistoryColor(currentWorking, rawHistory.rgb, texel, groupBase, maxTexel, varianceGamma);
    if (!IsValidWorkingColor(clippedHistory)) {
      clippedHistory = currentWorking;
    }
    historyConfidence = rawHistory.a * velocityConfidence * depthConfidence;
    finalWorking = mix(currentWorking, clippedHistory, historyConfidence);
    newWeight = clamp(1.0f / (2.0f - historyConfidence), 0.0f, 1.0f);
  } else {
    finalWorking = GetCurrentNeighborhood(currentWorking, texel, groupBase, maxTexel);
    newWeight = 0.5f;
  }
  if (!IsValidWorkingColor(finalWorking)) {
    finalWorking = currentWorking;
  }

  vec3 averageHdr = vec3(0.0f);
  float maxNeighborhoodLuminance = 0.0f;
  for (int y = -1; y <= 1; ++y) {
    for (int x = -1; x <= 1; ++x) {
      vec3 sampleHdr = FetchCurrentHdr(texel + ivec2(x, y), maxTexel);
      averageHdr += sampleHdr;
      maxNeighborhoodLuminance = max(maxNeighborhoodLuminance, LuminanceRec709(sampleHdr));
    }
  }
  averageHdr /= 9.0f;
  vec3 finalHdr = InverseReinhard(finalWorking, currentHdr);
  if (hasValidHistory) {
    float finalLuminance = LuminanceRec709(finalHdr);
    float maxTemporalLuminance = max(maxNeighborhoodLuminance * EE_TAA_HISTORY_LUMINANCE_SCALE,
                                     maxNeighborhoodLuminance + EE_TAA_HISTORY_LUMINANCE_BIAS);
    if (finalLuminance > maxTemporalLuminance) {
      finalHdr *= maxTemporalLuminance / max(finalLuminance, 1e-5f);
      finalWorking = Reinhard(finalHdr);
    }
  }
  vec3 outputHdr = max(finalHdr + (currentHdr - averageHdr) * sharpen, vec3(0.0f));
  vec3 historyRgb = historyColorMode == EE_TAA_HISTORY_TONE_MAPPED ? finalWorking : finalHdr;

  vec3 debugColor = outputHdr;
  if (debugMode == EE_TAA_DEBUG_MOTION) {
    debugColor = vec3(velocity.xy * 0.02f + vec2(0.5f), clamp(velocity.z * 32.0f + 0.5f, 0.0f, 1.0f));
  } else if (debugMode == EE_TAA_DEBUG_DEPTH_CONFIDENCE) {
    debugColor = vec3(depthConfidence);
  } else if (debugMode == EE_TAA_DEBUG_HISTORY_CONFIDENCE) {
    debugColor = vec3(historyConfidence);
  } else if (debugMode == EE_TAA_DEBUG_NO_HISTORY) {
    debugColor = hasValidHistory ? vec3(0.0f) : vec3(0.2f, 0.8f, 0.2f);
  }

  float ndcDepth = texelFetch(inDepth, texel, 0).x;
  float linearDepth = EE_LINEARIZE_DEPTH(EE_CAMERA_INDEX, min(ndcDepth, 1.0f));
  float normalizedDepth = NormalizedLinearDepth(ndcDepth);
  imageStore(outColor, texel, vec4(debugColor, 1.0f));
  imageStore(outHistory, texel, vec4(historyRgb, newWeight));
  imageStore(outDepthHistory, texel, vec4(ndcDepth, linearDepth, normalizedDepth, geometricConfidence));
}
