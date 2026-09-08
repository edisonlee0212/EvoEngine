import EvoEngine.PerFrame;

struct StarClusterResult
{
  double4 worldPositionRadius;
  float4 colorEmission;
  float4 alphaPadding;
};

struct StarRenderPushConstant
{
  int cameraIndex;
  float subpixelBrightnessLimit;
  float2 viewportSize;
  float fadeStrength;
  float2 distanceCompression;
};

struct StarVertexOutput
{
  float4 position : SV_Position;
  [[vk::location(0)]] float2 discPosition : TEXCOORD0;
  [[vk::location(1)]] float3 radiance : COLOR0;
};

[[vk::binding(0, 1)]] StructuredBuffer<StarClusterResult> starResults;
[[vk::push_constant]] ConstantBuffer<StarRenderPushConstant> constants;

static const float2 corners[6] =
{
  float2(-1.0, -1.0), float2(1.0, -1.0), float2(1.0, 1.0),
  float2(-1.0, -1.0), float2(1.0, 1.0), float2(-1.0, 1.0)
};

[shader("vertex")]
StarVertexOutput main(uint vertexIndex : SV_VertexID, uint instanceIndex : SV_InstanceID)
{
  const StarClusterResult star = starResults[instanceIndex];
  const int cameraIndex = constants.cameraIndex;
  const double3 cameraPosition = double3(EE_CAMERA_POSITION(cameraIndex));
  const float3 cameraRelativePosition = float3(star.worldPositionRadius.xyz - cameraPosition);
  float4 viewPosition = mul(float4(cameraRelativePosition, 0.0), EE_CAMERAS[cameraIndex].view);
  float displayRadius = float(star.worldPositionRadius.w);
  const float start = constants.distanceCompression.x;
  const float limit = constants.distanceCompression.y;
  const float distance = length(viewPosition.xyz);
  if (viewPosition.z < 0.0 && distance > start && limit > start)
  {
    const float maximumDistance = limit - max(abs(limit), limit - start) * 1e-7;
    const float compressedDistance = min(
      start + (limit - start) * (1.0 - exp(-(distance - start) / (limit - start))), maximumDistance);
    const float scale = compressedDistance / distance;
    viewPosition.xyz *= scale;
    displayRadius *= scale;
  }
  viewPosition.w = 1.0;

  StarVertexOutput output;
  output.position = mul(viewPosition, EE_CAMERAS[cameraIndex].projection);
  output.discPosition = corners[vertexIndex];
  output.radiance = star.colorEmission.xyz * star.colorEmission.w;
  const float2 viewportSize = max(constants.viewportSize, float2(1.0));
  const float2 diameterPixels = abs(displayRadius *
    float2(EE_CAMERAS[cameraIndex].projection[0][0], EE_CAMERAS[cameraIndex].projection[1][1])) *
    viewportSize / max(abs(output.position.w), 1e-6);
  const float diameter = min(diameterPixels.x, diameterPixels.y);
  if (diameter < 1.414214 && output.position.w > 0.0)
  {
    // Below sqrt(2) pixels, a disc can fall between all samples even with a one-pixel diameter.
    const float2 pixel = (output.position.xy / output.position.w * 0.5 + 0.5) * viewportSize;
    output.position.xy = ((floor(pixel) + 0.5) * 2.0 / viewportSize - 1.0) * output.position.w;
  }
  if (diameter < 1.0)
  {
    const float3 baseColor = max(star.colorEmission.xyz, float3(0.0));
    const float brightness = max(baseColor.x, max(baseColor.y, baseColor.z));
    // Compensate for the area added by minimum-pixel enlargement, independently on each screen axis.
    const float2 retainedSize = diameterPixels / max(diameterPixels, float2(1.0));
    const float retainedArea = retainedSize.x * retainedSize.y;
    const float enlargementFade = constants.fadeStrength <= 0.0 ? 1.0 :
      constants.fadeStrength == 1.0 ? retainedArea : pow(retainedArea, constants.fadeStrength);
    output.radiance = baseColor * min(1.0, constants.subpixelBrightnessLimit / max(brightness, 1e-6)) * enlargementFade;
  }
  const float2 drawDiameter = diameter > 0.0 ? max(diameterPixels, float2(1.0)) : float2(0.0);
  output.position.xy += corners[vertexIndex] * drawDiameter / viewportSize * output.position.w;
  return output;
}
