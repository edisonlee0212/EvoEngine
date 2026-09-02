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
  viewPosition.w = 1.0;

  const float2 corner = corners[vertexIndex];
  viewPosition.xy += corner * float(star.worldPositionRadius.w);

  StarVertexOutput output;
  output.position = mul(viewPosition, EE_CAMERAS[cameraIndex].projection);
  output.discPosition = corner;
  output.radiance = star.colorEmission.xyz * star.colorEmission.w * star.alphaPadding.x;
  return output;
}
