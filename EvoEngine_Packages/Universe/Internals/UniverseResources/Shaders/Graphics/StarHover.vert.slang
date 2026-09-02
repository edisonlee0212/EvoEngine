import EvoEngine.PerFrame;

struct StarClusterResult
{
  double4 worldPositionRadius;
  float4 colorEmission;
  float4 alphaPadding;
};

struct StarHoverPushConstant
{
  int cameraIndex;
  float brightnessLimit;
  float2 viewportSize;
  float2 displaySize;
  float minimumRadius;
  uint starIndex;
};

struct HoverVertexOutput
{
  float4 position : SV_Position;
  [[vk::location(0)]] float2 pixelOffset : TEXCOORD0;
  [[vk::location(1)]] nointerpolation float2 starRadius : TEXCOORD1;
  [[vk::location(2)]] nointerpolation float3 color : COLOR0;
};

[[vk::binding(0, 1)]] StructuredBuffer<StarClusterResult> starResults;
[[vk::push_constant]] ConstantBuffer<StarHoverPushConstant> constants;

static const float2 corners[6] =
{
  float2(-1, -1), float2(1, -1), float2(1, 1),
  float2(-1, -1), float2(1, 1), float2(-1, 1)
};

[shader("vertex")]
HoverVertexOutput main(uint vertexIndex : SV_VertexID)
{
  const StarClusterResult star = starResults[constants.starIndex];
  const int camera = constants.cameraIndex;
  const float3 relativePosition = float3(star.worldPositionRadius.xyz - double3(EE_CAMERA_POSITION(camera)));
  float4 viewPosition = mul(float4(relativePosition, 0), EE_CAMERAS[camera].view);
  viewPosition.w = 1;

  HoverVertexOutput output;
  output.position = mul(viewPosition, EE_CAMERAS[camera].projection);
  const float2 displaySize = max(constants.displaySize, float2(1));
  const float2 viewportSize = max(constants.viewportSize, float2(1));
  const float2 displayToFramebuffer = viewportSize / displaySize;
  const float2 framebufferRadius = abs(float(star.worldPositionRadius.w) *
    float2(EE_CAMERAS[camera].projection[0][0], EE_CAMERAS[camera].projection[1][1])) *
    viewportSize * 0.5 / max(abs(output.position.w), 1e-6);
  output.starRadius = max(framebufferRadius / displayToFramebuffer, float2(constants.minimumRadius));
  // One framebuffer pixel beyond the outer edge leaves room for derivative smoothing.
  output.pixelOffset = corners[vertexIndex] * (output.starRadius + 4 + 1 / displayToFramebuffer);
  output.position.xy += output.pixelOffset * displayToFramebuffer * 2 / viewportSize * output.position.w;
  output.color = float3(0, 1, 1) * clamp(constants.brightnessLimit, 0.0, 1.0);
  return output;
}
