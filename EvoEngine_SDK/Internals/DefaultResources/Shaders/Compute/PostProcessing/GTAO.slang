import EvoEngine.PerFrame;
import EvoEngine.VogelDisk;

[[vk::binding(17, 1)]]
Sampler2D inDepth;

[[vk::binding(21, 1)]]
Sampler2D inNormalRoughness;

[[vk::binding(0, 2)]]
[vk::image_format("r16f")]
RWTexture2D<float> outAmbientOcclusion;

struct EeAoConstants {
  int camera_index;
  float radius;
  float bias;
  float intensity;
  int sliceCount;
  int stepsPerSlice;
  float thickness;
  float padding;
};

[[vk::push_constant]]
ConstantBuffer<EeAoConstants> constants;

float3 GetNormal(float2 texCoord) {
  return inNormalRoughness.SampleLevel(texCoord, 0.0f).rgb;
}

float3 GetViewNormal(float2 texCoord) {
  return normalize(mul(float4(GetNormal(texCoord), 0.0f), EE_CAMERAS[constants.camera_index].view).xyz);
}

bool GetViewPosition(float2 texCoord, out float3 position) {
  if (texCoord.x < 0.0f || texCoord.x > 1.0f || texCoord.y < 0.0f || texCoord.y > 1.0f) {
    return false;
  }
  float ndcDepth = inDepth.SampleLevel(texCoord, 0.0f).x;
  if (ndcDepth == 1.0f) {
    return false;
  }
  position = EE_DEPTH_TO_VIEW_POS(constants.camera_index, texCoord, ndcDepth);
  return true;
}

float ComputeGtao(float2 texCoord, float3 viewPos, float3 normal, float2 resolution) {
  int slices = clamp(constants.sliceCount, 1, 16);
  int steps = clamp(constants.stepsPerSlice, 1, 16);
  float3 centerPos = viewPos;
  centerPos.z *= 0.9992f;
  float viewDepth = max(-centerPos.z, 0.001f);
  float effectiveRadius = max(constants.radius * 1.457f, 0.001f);
  float noise = EE_INTERLEAVED_GRADIENT_NOISE(float3(texCoord, viewDepth));
  float centerDepth = inDepth.SampleLevel(texCoord, 0.0f).x;
  float2 pixelSize = 1.0f / resolution;
  float3 pixelAxisX = EE_DEPTH_TO_VIEW_POS(constants.camera_index, clamp(texCoord + float2(pixelSize.x, 0.0f), float2(0.0f), float2(1.0f)),
                                         centerDepth) -
                    viewPos;
  float3 pixelAxisY = EE_DEPTH_TO_VIEW_POS(constants.camera_index, clamp(texCoord + float2(0.0f, pixelSize.y), float2(0.0f), float2(1.0f)),
                                         centerDepth) -
                    viewPos;
  float falloffRange = max(effectiveRadius * 0.615f, 0.001f);
  float falloffFrom = effectiveRadius - falloffRange;
  float falloffMul = -1.0f / falloffRange;
  float falloffAdd = falloffFrom / falloffRange + 1.0f;
  float3 viewVec = normalize(-centerPos);
  float visibility = 0.0f;

  for (int slice = 0; slice < slices; ++slice) {
    float angle = (float(slice) + noise) * 3.14159265f / float(slices);
    float2 direction = float2(cos(angle), sin(angle));
    float3 directionStep = pixelAxisX * direction.x + pixelAxisY * direction.y;
    float directionStepLength = length(directionStep);
    if (directionStepLength <= 0.0001f) {
      continue;
    }
    float pixelRadius = max(1.0f, effectiveRadius / directionStepLength);
    float minStep = 1.3f / pixelRadius;
    float3 directionVec = directionStep / directionStepLength;
    float3 orthoDirectionVec = directionVec - dot(directionVec, viewVec) * viewVec;
    float orthoDirectionLength = length(orthoDirectionVec);
    if (orthoDirectionLength <= 0.0001f) {
      continue;
    }
    orthoDirectionVec /= orthoDirectionLength;
    float3 axisVec = cross(orthoDirectionVec, viewVec);
    float axisLength = length(axisVec);
    if (axisLength <= 0.0001f) {
      continue;
    }
    axisVec /= axisLength;
    float3 projectedNormalVec = normal - axisVec * dot(normal, axisVec);
    float projectedNormalLength = length(projectedNormalVec);
    if (projectedNormalLength <= 0.0001f) {
      continue;
    }
    float signNorm = sign(dot(orthoDirectionVec, projectedNormalVec));
    float cosNorm = clamp(dot(projectedNormalVec, viewVec) / projectedNormalLength, 0.0f, 1.0f);
    float n = signNorm * acos(cosNorm);
    float lowHorizonCos0 = cos(n + 1.57079633f);
    float lowHorizonCos1 = cos(n - 1.57079633f);
    float horizonCos0 = lowHorizonCos0;
    float horizonCos1 = lowHorizonCos1;

    for (int stepIndex = 0; stepIndex < steps; ++stepIndex) {
      float stepNoise = frac(noise + float(slice + stepIndex * steps) * 0.61803398875f);
      float stepRatio = pow((float(stepIndex) + stepNoise) / float(steps), 2.0f) + minStep;
      float2 sampleOffset = round(direction * pixelRadius * stepRatio) / resolution;

      float3 sampleViewPos0;
      if (GetViewPosition(texCoord + sampleOffset, sampleViewPos0)) {
        float3 delta = sampleViewPos0 - centerPos;
        float distanceToSample = length(delta);
        if (distanceToSample > max(constants.bias, 0.0001f)) {
          float weight = clamp(distanceToSample * falloffMul + falloffAdd, 0.0f, 1.0f);
          float sampleHorizonCos = dot(delta / distanceToSample, viewVec);
          sampleHorizonCos = lerp(lowHorizonCos0, sampleHorizonCos, weight);
          horizonCos0 = max(horizonCos0, sampleHorizonCos);
        }
      }

      float3 sampleViewPos1;
      if (GetViewPosition(texCoord - sampleOffset, sampleViewPos1)) {
        float3 delta = sampleViewPos1 - centerPos;
        float distanceToSample = length(delta);
        if (distanceToSample > max(constants.bias, 0.0001f)) {
          float weight = clamp(distanceToSample * falloffMul + falloffAdd, 0.0f, 1.0f);
          float sampleHorizonCos = dot(delta / distanceToSample, viewVec);
          sampleHorizonCos = lerp(lowHorizonCos1, sampleHorizonCos, weight);
          horizonCos1 = max(horizonCos1, sampleHorizonCos);
        }
      }
    }

    projectedNormalLength = lerp(projectedNormalLength, 1.0f, 0.05f);
    float h0 = -acos(clamp(horizonCos1, -1.0f, 1.0f));
    float h1 = acos(clamp(horizonCos0, -1.0f, 1.0f));
    float iarc0 = (cosNorm + 2.0f * h0 * sin(n) - cos(2.0f * h0 - n)) * 0.25f;
    float iarc1 = (cosNorm + 2.0f * h1 * sin(n) - cos(2.0f * h1 - n)) * 0.25f;
    visibility += projectedNormalLength * (iarc0 + iarc1);
  }

  visibility = clamp(visibility / float(slices), 0.0f, 1.0f);
  return max(0.03f, pow(visibility, max(constants.intensity, 0.001f)));
}

[shader("compute")]
[numthreads(16, 16, 1)]
void main(uint3 dispatchThreadId : SV_DispatchThreadID) {
  uint2 pixel = dispatchThreadId.xy;
  uint width = 0;
  uint height = 0;
  outAmbientOcclusion.GetDimensions(width, height);
  uint2 resolution = uint2(width, height);
  if (pixel.x >= resolution.x || pixel.y >= resolution.y) {
    return;
  }

  float2 texCoord = (float2(pixel) + float2(0.5f)) / float2(resolution);
  float3 viewPos;
  if (!GetViewPosition(texCoord, viewPos)) {
    outAmbientOcclusion[int2(pixel)] = 1.0f;
    return;
  }

  float3 normal = GetViewNormal(texCoord);
  float ambientOcclusion = ComputeGtao(texCoord, viewPos, normal, float2(resolution));
  outAmbientOcclusion[int2(pixel)] = ambientOcclusion;
}
