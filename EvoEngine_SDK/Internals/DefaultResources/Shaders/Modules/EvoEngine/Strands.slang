// @evoengine-dialect native
import EvoEngine.PerFrame;

public int EE_STRANDS_SEGMENT_SUBDIVISION(int camera_index, float3 world_position_a, float3 world_position_b) {
  float4 coord_a = mul(float4(world_position_a, 1.0f), EE_CAMERAS[camera_index].projection_view);
  float4 coord_b = mul(float4(world_position_b, 1.0f), EE_CAMERAS[camera_index].projection_view);
  float2 screen_size = float2(EE_CAMERA_RESOLUTION_X(camera_index), EE_CAMERA_RESOLUTION_Y(camera_index));
  coord_a /= coord_a.w;
  coord_b /= coord_b.w;
  if (coord_a.z < -1.0f && coord_b.z < -1.0f) return 0;
  float pixel_distance = distance(coord_a.xy * screen_size * 0.5f, coord_b.xy * screen_size * 0.5f);
  return max(1, min(EE_RENDER_INFO.strand_subdivision_max_x,
                    int(pixel_distance / EE_RENDER_INFO.strand_subdivision_x)));
}

public void EE_SPLINE_INTERPOLATION(float3 v0, float3 v1, float3 v2, float3 v3, out float3 result,
                                    out float3 tangent, float u) {
  float3 p0 = (v2 + v0) / 6.0f + v1 * (4.0f / 6.0f);
  float3 p1 = v2 - v0;
  float3 p2 = v2 - v1;
  float3 p3 = v3 - v1;
  float uu = u * u;
  float u3 = (1.0f / 6.0f) * uu * u;
  float3 q = float3(u3 + 0.5f * (u - uu), uu - 4.0f * u3, u3);
  result = p0 + q.x * p1 + q.y * p2 + q.z * p3;
  u = u == 0.0f ? 0.000001f : (u == 1.0f ? 0.999999f : u);
  float v = 1.0f - u;
  tangent = 0.5f * v * v * p1 + 2.0f * v * u * p2 + 0.5f * u * u * p3;
}

public void EE_SPLINE_INTERPOLATION(float v0, float v1, float v2, float v3, out float result,
                                    out float tangent, float u) {
  float p0 = (v2 + v0) / 6.0f + v1 * (4.0f / 6.0f);
  float p1 = v2 - v0;
  float p2 = v2 - v1;
  float p3 = v3 - v1;
  float uu = u * u;
  float u3 = (1.0f / 6.0f) * uu * u;
  float3 q = float3(u3 + 0.5f * (u - uu), uu - 4.0f * u3, u3);
  result = p0 + q.x * p1 + q.y * p2 + q.z * p3;
  u = u == 0.0f ? 0.000001f : (u == 1.0f ? 0.999999f : u);
  float v = 1.0f - u;
  tangent = 0.5f * v * v * p1 + 2.0f * v * u * p2 + 0.5f * u * u * p3;
}

public void EE_SPLINE_INTERPOLATION(float4 v0, float4 v1, float4 v2, float4 v3, out float4 result,
                                    out float4 tangent, float u) {
  float4 p0 = (v2 + v0) / 6.0f + v1 * (4.0f / 6.0f);
  float4 p1 = v2 - v0;
  float4 p2 = v2 - v1;
  float4 p3 = v3 - v1;
  float uu = u * u;
  float u3 = (1.0f / 6.0f) * uu * u;
  float3 q = float3(u3 + 0.5f * (u - uu), uu - 4.0f * u3, u3);
  result = p0 + q.x * p1 + q.y * p2 + q.z * p3;
  u = u == 0.0f ? 0.000001f : (u == 1.0f ? 0.999999f : u);
  float v = 1.0f - u;
  tangent = 0.5f * v * v * p1 + 2.0f * v * u * p2 + 0.5f * u * u * p3;
}

public int EE_STRANDS_RING_SUBDIVISION(int camera_index, float4x4 model, float3 world_position,
                                       float3 model_position, float thickness) {
  float3 model_position_b = thickness * float3(0.0f, 1.0f, 0.0f) + model_position;
  float3 endpoint_world_position = mul(float4(model_position_b, 1.0f), model).xyz;
  float3 redirected_world_position =
      world_position + EE_CAMERA_UP(camera_index) * distance(world_position, endpoint_world_position);
  float subdivision = EE_PIXEL_DISTANCE(camera_index, world_position, redirected_world_position) * 50.0f /
                      max(EE_RENDER_INFO.strand_subdivision_y, 1.0f);
  return max(3, min(int(subdivision), min(EE_RENDER_INFO.strand_subdivision_max_y, 15)));
}
