// @evoengine-dialect native
import EvoEngine.PerFrame;

// Algorithm and constants are adapted from GameTechDev/TAA (MIT).
#ifndef EE_TAA_USE_TGSM
#define EE_TAA_USE_TGSM 0
#endif
#ifndef EE_TAA_USE_FP16
#define EE_TAA_USE_FP16 0
#endif
[[vk::binding(17, 1)]] Sampler2D inDepth;
[[vk::binding(0, 2)]] Sampler2D currentColor;
[[vk::binding(1, 2)]] Sampler2D historyColor;
[[vk::binding(2, 2)]] Sampler2D motionVectors;
[[vk::binding(3, 2)]] Sampler2D previousDepthHistory;
[[vk::binding(4, 2)]] [vk::image_format("rgba32f")] RWTexture2D<float4> outColor;
[[vk::binding(5, 2)]] [vk::image_format("rgba32f")] RWTexture2D<float4> outHistory;
[[vk::binding(6, 2)]] [vk::image_format("rgba32f")] RWTexture2D<float4> outDepthHistory;

struct EeTaaConstants {
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

[[vk::push_constant]]
ConstantBuffer<EeTaaConstants> constants;

static const int EE_TAA_VARIANCE_DISABLED = 0;
static const int EE_TAA_VARIANCE_CLAMP = 1;
static const int EE_TAA_VARIANCE_INTERSECTION = 2;
static const int EE_TAA_HISTORY_TONE_MAPPED = 0;
static const int EE_TAA_DEBUG_MOTION = 1;
static const int EE_TAA_DEBUG_DEPTH_CONFIDENCE = 2;
static const int EE_TAA_DEBUG_HISTORY_CONFIDENCE = 3;
static const int EE_TAA_DEBUG_NO_HISTORY = 4;
static const float EE_TAA_INTERSECTION_MAX_T = 100.0f;
static const float EE_TAA_FP16_MIN_REINHARD_HEADROOM = 1.0f / 256.0f;
static const float EE_TAA_INVALID_VELOCITY = 32768.0f;
static const float EE_TAA_HISTORY_LUMINANCE_SCALE = 2.0f;
static const float EE_TAA_HISTORY_LUMINANCE_BIAS = 0.25f;

#if EE_TAA_USE_TGSM
static const int EE_TAA_TILE_WIDTH = 10;
static const int EE_TAA_TILE_SIZE = EE_TAA_TILE_WIDTH * EE_TAA_TILE_WIDTH;
groupshared float4 eeTaaCurrentTile[EE_TAA_TILE_SIZE];
#endif

float LuminanceRec709(float3 rgb) {
  return dot(rgb, float3(0.2126f, 0.7152f, 0.0722f));
}

bool IsFiniteColor(float3 rgb) {
  return !any(isnan(rgb)) && !any(isinf(rgb));
}

bool IsValidWorkingColor(float3 rgb) {
  return IsFiniteColor(rgb) && all(rgb >= float3(0.0f)) && LuminanceRec709(rgb) < 1.0f;
}

bool IsStableFp16WorkingColor(float3 rgb) {
  return IsValidWorkingColor(rgb) && 1.0f - LuminanceRec709(rgb) >= EE_TAA_FP16_MIN_REINHARD_HEADROOM;
}

float3 Reinhard(float3 rgb) {
  if (!IsFiniteColor(rgb)) {
    return float3(0.0f);
  }
  rgb = max(rgb, float3(0.0f));
  float3 result = rgb / (1.0f + LuminanceRec709(rgb));
#if EE_TAA_USE_FP16
  float3 fp16Result = float3(half3(result));
  if (IsStableFp16WorkingColor(fp16Result)) {
    result = fp16Result;
  }
#endif
  return result;
}

float3 InverseReinhard(float3 rgb, float3 fallback) {
  if (!IsValidWorkingColor(rgb)) {
    return fallback;
  }
  float3 result = rgb / (1.0f - LuminanceRec709(rgb));
  return IsFiniteColor(result) ? max(result, float3(0.0f)) : fallback;
}

float3 RgbToYCoCg(float3 rgb) {
  if (constants.useYCoCg == 0) {
    return rgb;
  }
  return float3(dot(rgb, float3(0.25f, 0.5f, 0.25f)), dot(rgb, float3(0.5f, 0.0f, -0.5f)),
              dot(rgb, float3(-0.25f, 0.5f, -0.25f)));
}

float3 YCoCgToRgb(float3 ycocg) {
  if (constants.useYCoCg == 0) {
    return ycocg;
  }
  return float3(dot(ycocg, float3(1.0f, 1.0f, -1.0f)), dot(ycocg, float3(1.0f, 0.0f, 1.0f)),
              dot(ycocg, float3(1.0f, -1.0f, -1.0f)));
}

float3 FetchCurrentHdr(int2 texel, int2 maxTexel) {
  float3 result = currentColor.Load(int3(clamp(texel, int2(0), maxTexel), 0)).rgb;
  return IsFiniteColor(result) ? max(result, float3(0.0f)) : float3(0.0f);
}

void StoreCurrentColorToShared(int2 groupBase, int2 maxTexel, uint localInvocationIndex) {
#if EE_TAA_USE_TGSM
  for (uint index = localInvocationIndex; index < uint(EE_TAA_TILE_SIZE); index += 64u) {
    int2 tileTexel = int2(int(index) % EE_TAA_TILE_WIDTH, int(index) / EE_TAA_TILE_WIDTH);
    int2 sourceTexel = clamp(groupBase + tileTexel - int2(1), int2(0), maxTexel);
    eeTaaCurrentTile[index] = float4(Reinhard(FetchCurrentHdr(sourceTexel, maxTexel)), 1.0f);
  }
  GroupMemoryBarrier();
  GroupMemoryBarrierWithGroupSync();
#endif
}

float3 FetchCurrentWorking(int2 texel, int2 groupBase, int2 maxTexel) {
#if EE_TAA_USE_TGSM
  int2 tileTexel = clamp(texel - groupBase + int2(1), int2(0), int2(EE_TAA_TILE_WIDTH - 1));
  return eeTaaCurrentTile[tileTexel.y * EE_TAA_TILE_WIDTH + tileTexel.x].rgb;
#else
  return Reinhard(FetchCurrentHdr(texel, maxTexel));
#endif
}

float3 ClipToAabb(float3 history, float3 current, float3 center, float3 extents) {
  float3 direction = current - history;
  float3 boundary = center - sign(direction) * extents;
  float3 possibleT = float3(EE_TAA_INTERSECTION_MAX_T + 1.0f);
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

float3 ClipHistoryColor(float3 current, float3 history, int2 texel, int2 groupBase, int2 maxTexel,
                      float varianceGamma) {
  if (constants.varianceClippingMode == EE_TAA_VARIANCE_DISABLED) {
    return history;
  }

  const int2 offsets9[8] = int2[](int2(-1, -1), int2(-1, 0), int2(-1, 1), int2(0, 1),
                                    int2(1, 1),   int2(1, 0),  int2(1, -1), int2(0, -1));
  const int2 offsetsAxial[4] = int2[](int2(-1, 0), int2(0, 1), int2(1, 0), int2(0, -1));

  float3 currentSpace = RgbToYCoCg(current);
  float3 moment1 = currentSpace;
  float3 moment2 = currentSpace * currentSpace;
  int sampleCount = constants.varianceSampleCount >= 9 ? 9 : 5;
  int neighborCount = sampleCount - 1;
  for (int index = 0; index < neighborCount; ++index) {
    int2 offset = sampleCount == 9 ? offsets9[index] : offsetsAxial[index];
    float3 sampleColor = RgbToYCoCg(FetchCurrentWorking(texel + offset, groupBase, maxTexel));
    moment1 += sampleColor;
    moment2 += sampleColor * sampleColor;
  }

  float reciprocalSampleCount = 1.0f / float(sampleCount);
  float3 mean = moment1 * reciprocalSampleCount;
  float3 standardDeviation = sqrt(max(moment2 * reciprocalSampleCount - mean * mean, float3(0.0f))) * varianceGamma;
  float3 historySpace = RgbToYCoCg(history);
  if (constants.varianceClippingMode == EE_TAA_VARIANCE_CLAMP) {
    return clamp(history, YCoCgToRgb(mean - standardDeviation), YCoCgToRgb(mean + standardDeviation));
  }
  return YCoCgToRgb(ClipToAabb(historySpace, currentSpace, mean, standardDeviation));
}

float4 BicubicHistory(float2 historySt, float2 resolution) {
  float2 reciprocalResolution = 1.0f / resolution;
  float2 fractional = frac(historySt);
  float2 uv = (floor(historySt) + float2(0.5f)) * reciprocalResolution;
  float2 t = fractional;
  float2 t2 = t * t;
  float2 t3 = t2 * t;
  const float s = 0.5f;
  float2 w0 = -s * t3 + 2.0f * s * t2 - s * t;
  float2 w1 = (2.0f - s) * t3 + (s - 3.0f) * t2 + float2(1.0f);
  float2 w2 = (s - 2.0f) * t3 + (3.0f - 2.0f * s) * t2 + s * t;
  float2 w3 = s * t3 - s * t2;
  float2 s0 = w1 + w2;
  float2 f0 = w2 / max(s0, float2(1e-6f));
  float2 m0 = uv + f0 * reciprocalResolution;
  float2 tc0 = uv - reciprocalResolution;
  float2 tc3 = uv + 2.0f * reciprocalResolution;

  float4 a = historyColor.SampleLevel(float2(m0.x, tc0.y), 0.0f);
  float4 b = historyColor.SampleLevel(float2(tc0.x, m0.y), 0.0f);
  float4 c = historyColor.SampleLevel(m0, 0.0f);
  float4 d = historyColor.SampleLevel(float2(tc3.x, m0.y), 0.0f);
  float4 e = historyColor.SampleLevel(float2(m0.x, tc3.y), 0.0f);
  return (0.5f * (a + b) * w0.x + a * s0.x + 0.5f * (a + b) * w3.x) * w0.y +
         (b * w0.x + c * s0.x + d * w3.x) * s0.y +
         (0.5f * (b + e) * w0.x + e * s0.x + 0.5f * (d + e) * w3.x) * w3.y;
}

float4 GetHistory(float2 historyUv, float2 historySt, bool isOnEdge, float2 resolution) {
  bool useBilinearOnEdges = constants.useLongestVelocity == 0;
  bool useBicubic = constants.useLongestVelocity != 0 || (!isOnEdge && useBilinearOnEdges);
  float4 result = constants.useBicubicFilter != 0 && useBicubic ? BicubicHistory(historySt, resolution)
                                                    : historyColor.SampleLevel(historyUv, 0.0f);
  if (constants.historyColorMode != EE_TAA_HISTORY_TONE_MAPPED) {
    result.rgb = Reinhard(result.rgb);
  }
  result.a = clamp(result.a, 0.0f, 1.0f);
  return result;
}

float3 GetVelocity(int2 texel, int2 maxTexel) {
  float3 result = motionVectors.Load(int3(clamp(texel, int2(0), maxTexel), 0)).xyz;
  bool invalidResult = !IsFiniteColor(result) || any(abs(result.xy) >= float2(EE_TAA_INVALID_VELOCITY));
  if (constants.useLongestVelocity == 0 || invalidResult) {
    return result;
  }
  const int2 offsets9[8] = int2[](int2(-1, -1), int2(-1, 0), int2(-1, 1), int2(0, 1),
                                    int2(1, 1),   int2(1, 0),  int2(1, -1), int2(0, -1));
  const int2 offsets5[4] = int2[](int2(-1, -1), int2(-1, 1), int2(1, 1), int2(1, -1));
  int neighborCount = constants.longestVelocitySampleCount >= 9 ? 8 : 4;
  float longestLengthSquared = dot(result.xy, result.xy);
  for (int index = 0; index < neighborCount; ++index) {
    int2 offset = constants.longestVelocitySampleCount >= 9 ? offsets9[index] : offsets5[index];
    float3 candidate = motionVectors.Load(int3(clamp(texel + offset, int2(0), maxTexel), 0)).xyz;
    if (!IsFiniteColor(candidate) ||
        any(abs(candidate.xy) >= float2(EE_TAA_INVALID_VELOCITY))) {
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
  float4 viewPosition = mul(float4(0.0f, 0.0f, ndcDepth, 1.0f),
                            EE_CAMERAS[constants.EE_CAMERA_INDEX].inverse_projection);
  float4x4 projection = EE_CAMERAS[constants.EE_CAMERA_INDEX].projection;
  float farDistance = abs(projection[3][2] / (projection[2][2] + 1.0f));
  return clamp(abs(viewPosition.z / viewPosition.w) / max(farDistance, 1e-5f), 0.0f, 1.0f);
}

float GetCurrentDepth(int2 texel, int2 maxTexel, out bool isOnEdge) {
  float depths[4] = {
      NormalizedLinearDepth(inDepth.Load(int3(clamp(texel, int2(0), maxTexel), 0)).x),
      NormalizedLinearDepth(inDepth.Load(int3(clamp(texel + int2(1, 0), int2(0), maxTexel), 0)).x),
      NormalizedLinearDepth(inDepth.Load(int3(clamp(texel + int2(0, 1), int2(0), maxTexel), 0)).x),
      NormalizedLinearDepth(inDepth.Load(int3(clamp(texel + int2(1, 1), int2(0), maxTexel), 0)).x)};
  float minDepth = min(min(depths[0], depths[1]), min(depths[2], depths[3]));
  float maxDepth = max(max(depths[0], depths[1]), max(depths[2], depths[3]));
  isOnEdge = maxDepth - minDepth > constants.depthThreshold;
  return minDepth;
}

float GetPreviousDepth(float2 uv) {
  float4 depths = previousDepthHistory.GatherBlue(uv);
  return max(max(depths.x, depths.y), max(depths.z, depths.w)) + 0.001f;
}

float BoundsConfidence(float2 uv) {
  return all(uv >= float2(0.0f)) && all(uv < float2(1.0f)) ? 1.0f : 0.0f;
}

float GetDepthConfidence(float2 previousDepthUv, float3 velocity, float currentDepth, bool isOnEdge) {
  if (constants.useDepthThreshold == 0) {
    return 1.0f;
  }
  bool needsEdgeDetection = constants.useBicubicFilter == 0 && constants.useLongestVelocity == 0;
  if (needsEdgeDetection && isOnEdge) {
    return 1.0f;
  }
  return step(currentDepth + velocity.z, GetPreviousDepth(previousDepthUv));
}

float3 GetCurrentNeighborhood(float3 current, int2 texel, int2 groupBase, int2 maxTexel) {
  if (constants.useNeighborhoodSampling == 0) {
    return current;
  }
  const int2 offsets[4] = int2[](int2(-1, -1), int2(-1, 1), int2(1, 1), int2(1, -1));
  float3 result = current;
  for (int index = 0; index < 4; ++index) {
    result += FetchCurrentWorking(texel + offsets[index], groupBase, maxTexel);
  }
  return result * 0.2f;
}

[shader("compute")][numthreads(8, 8, 1)] void main(uint3 dispatchThreadId : SV_DispatchThreadID,
                                                    uint3 groupId : SV_GroupID,
                                                    uint localInvocationIndex : SV_GroupIndex) {
  uint width = 0;
  uint height = 0;
  outColor.GetDimensions(width, height);
  int2 resolution = int2(width, height);
  int2 maxTexel = resolution - int2(1);
  int2 texel = int2(dispatchThreadId.xy);
  int2 groupBase = int2(groupId.xy) * int2(8);
  StoreCurrentColorToShared(groupBase, maxTexel, localInvocationIndex);
  if (any(texel >= resolution)) {
    return;
  }

  float2 resolutionFloat = float2(resolution);
  float3 currentHdr = FetchCurrentHdr(texel, maxTexel);
  float3 currentWorking = FetchCurrentWorking(texel, groupBase, maxTexel);
  float3 velocity = GetVelocity(texel, maxTexel);
  float scaledVelocityThreshold = constants.velocityRejectionThreshold * max(resolutionFloat.y / 1080.0f, 1e-4f);
  float velocityConfidence = clamp(1.0f - length(velocity.xy) / max(scaledVelocityThreshold, 1.0f), 0.0f, 1.0f);

  float2 previousSt = float2(texel) + velocity.xy;
  float2 previousUv = (previousSt + float2(0.5f)) / resolutionFloat;
  float2 jitterPixels = (EE_CAMERAS[constants.EE_CAMERA_INDEX].jitter.zw - EE_CAMERAS[constants.EE_CAMERA_INDEX].jitter.xy) * 0.5f *
                      resolutionFloat;
  float2 previousDepthUv = (previousSt + jitterPixels + float2(0.5f)) / resolutionFloat;

  bool isOnEdge = false;
  float currentDepth = GetCurrentDepth(texel, maxTexel, isOnEdge);
  float uvConfidence = BoundsConfidence(previousUv);
  float depthUvConfidence = BoundsConfidence(previousDepthUv);
  float depthConfidence = depthUvConfidence > 0.0f
                              ? GetDepthConfidence(previousDepthUv, velocity, currentDepth, isOnEdge)
                              : 0.0f;
  float geometricConfidence = velocityConfidence * depthConfidence * uvConfidence * depthUvConfidence;
  bool hasValidHistory = constants.historyValid != 0 && geometricConfidence > 0.0f;

  float3 finalWorking;
  float newWeight;
  float historyConfidence = 0.0f;
  if (hasValidHistory) {
    float4 rawHistory = GetHistory(previousUv, previousSt, isOnEdge, resolutionFloat);
    float varianceGamma = lerp(constants.minVarianceGamma, constants.maxVarianceGamma, velocityConfidence * velocityConfidence);
    float3 clippedHistory =
        ClipHistoryColor(currentWorking, rawHistory.rgb, texel, groupBase, maxTexel, varianceGamma);
    if (!IsValidWorkingColor(clippedHistory)) {
      clippedHistory = currentWorking;
    }
    historyConfidence = rawHistory.a * velocityConfidence * depthConfidence;
    finalWorking = lerp(currentWorking, clippedHistory, historyConfidence);
    newWeight = clamp(1.0f / (2.0f - historyConfidence), 0.0f, 1.0f);
  } else {
    finalWorking = GetCurrentNeighborhood(currentWorking, texel, groupBase, maxTexel);
    newWeight = 0.5f;
  }
  if (!IsValidWorkingColor(finalWorking)) {
    finalWorking = currentWorking;
  }

  float3 averageHdr = float3(0.0f);
  float maxNeighborhoodLuminance = 0.0f;
  for (int y = -1; y <= 1; ++y) {
    for (int x = -1; x <= 1; ++x) {
      float3 sampleHdr = FetchCurrentHdr(texel + int2(x, y), maxTexel);
      averageHdr += sampleHdr;
      maxNeighborhoodLuminance = max(maxNeighborhoodLuminance, LuminanceRec709(sampleHdr));
    }
  }
  averageHdr /= 9.0f;
  float3 finalHdr = InverseReinhard(finalWorking, currentHdr);
  if (hasValidHistory) {
    float finalLuminance = LuminanceRec709(finalHdr);
    float maxTemporalLuminance = max(maxNeighborhoodLuminance * EE_TAA_HISTORY_LUMINANCE_SCALE,
                                     maxNeighborhoodLuminance + EE_TAA_HISTORY_LUMINANCE_BIAS);
    if (finalLuminance > maxTemporalLuminance) {
      finalHdr *= maxTemporalLuminance / max(finalLuminance, 1e-5f);
      finalWorking = Reinhard(finalHdr);
    }
  }
  float3 outputHdr = max(finalHdr + (currentHdr - averageHdr) * constants.sharpen, float3(0.0f));
  float3 historyRgb = constants.historyColorMode == EE_TAA_HISTORY_TONE_MAPPED ? finalWorking : finalHdr;

  float3 debugColor = outputHdr;
  if (constants.debugMode == EE_TAA_DEBUG_MOTION) {
    debugColor = float3(velocity.xy * 0.02f + float2(0.5f), clamp(velocity.z * 32.0f + 0.5f, 0.0f, 1.0f));
  } else if (constants.debugMode == EE_TAA_DEBUG_DEPTH_CONFIDENCE) {
    debugColor = float3(depthConfidence);
  } else if (constants.debugMode == EE_TAA_DEBUG_HISTORY_CONFIDENCE) {
    debugColor = float3(historyConfidence);
  } else if (constants.debugMode == EE_TAA_DEBUG_NO_HISTORY) {
    debugColor = hasValidHistory ? float3(0.0f) : float3(0.2f, 0.8f, 0.2f);
  }

  float ndcDepth = inDepth.Load(int3(texel, 0)).x;
  float linearDepth = EE_LINEARIZE_DEPTH(constants.EE_CAMERA_INDEX, min(ndcDepth, 1.0f));
  float normalizedDepth = NormalizedLinearDepth(ndcDepth);
  outColor[texel] = float4(debugColor, 1.0f);
  outHistory[texel] = float4(historyRgb, newWeight);
  outDepthHistory[texel] = float4(ndcDepth, linearDepth, normalizedDepth, geometricConfidence);
}
