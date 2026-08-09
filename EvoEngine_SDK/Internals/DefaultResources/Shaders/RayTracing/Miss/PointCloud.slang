// @evoengine-dialect native
import EvoEngine.PointCloudRayTracingPayload;

[shader("miss")]
void main(inout PointCloudRayTracingPayload hit_value) {
  hit_value.hit_info.color = float4(0.0f, 0.0f, 0.0f, 1.0f);
  hit_value.handle = 0;
  hit_value.hit_info.position = float3(0.0f);
  hit_value.hit_info.normal = float3(0.0f);
  hit_value.hit_info.tangent = float3(0.0f);
  hit_value.hit_info.tex_coord = float2(0.0f);
  hit_value.hit_info.vertex_info1 = 0.0f;
  hit_value.hit_info.vertex_info2 = 0.0f;
  hit_value.hit_info.vertex_info3 = 0.0f;
  hit_value.hit_info.vertex_info4 = float2(0.0f);
}
