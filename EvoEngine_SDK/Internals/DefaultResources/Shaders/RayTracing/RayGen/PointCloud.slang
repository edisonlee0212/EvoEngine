// @evoengine-dialect native
import EvoEngine.PointCloudRayTracingPayload;
import EvoEngine.RayTracingBasic;

struct PointCloudSample {
  float3 direction;
  float padding0;
  float3 start;
  uint hit_count;
  uint64_t handle;
  uint64_t padding1;
  HitInfo hit_info;
};

[[vk::binding(0, 2)]] RWStructuredBuffer<PointCloudSample> EE_POINT_CLOUD_SAMPLES;

[shader("raygeneration")]
void main() {
  const uint sample_index = DispatchRaysIndex().x;
  PointCloudRayTracingPayload hit_value = {};
  PointCloudSample point_cloud_sample = EE_POINT_CLOUD_SAMPLES[sample_index];
  RayDesc ray = {point_cloud_sample.start, 0.0f, point_cloud_sample.direction, 1e20f};
  TraceRay(EE_TLAS, RAY_FLAG_FORCE_OPAQUE, 0xff, 0, 0, 0, ray, hit_value);
  point_cloud_sample.hit_count = hit_value.hit_count;
  point_cloud_sample.handle = hit_value.handle;
  point_cloud_sample.hit_info = hit_value.hit_info;
  EE_POINT_CLOUD_SAMPLES[sample_index] = point_cloud_sample;
}
