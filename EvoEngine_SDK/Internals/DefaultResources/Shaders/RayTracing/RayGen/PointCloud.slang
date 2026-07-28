#extension GL_ARB_shading_language_include : enable
#include "RayTracingBasic.slangh"

#include "PointCloudRayTracingPayload.slangh"

struct PointCloudSample {
	// Input
	vec3 direction; /**< Ray or sampling direction vector. */
	float padding0;
	vec3 start;     /**< Starting point of the ray or sample. */

	// Output
	uint hit_count;    /**< Flag indicating whether a hit occurred. */
	uint64_t handle; /**< Handle or identifier for the point-cloud object. */
	uint64_t padding1;

	HitInfo hit_info; /**< Detailed information about the hit, if one occurred. */
};

layout(set = 2, binding = 0) buffer EE_POINT_CLOUD_SAMPLE_BLOCK {
  PointCloudSample EE_POINT_CLOUD_SAMPLES[];
};


PointCloudRayTracingPayload hit_value;

[shader("raygeneration")]
void main()
{
	uint sample_index = DispatchRaysIndex().x;
	hit_value.hit_count = 0;
	PointCloudSample point_cloud_sample = EE_POINT_CLOUD_SAMPLES[sample_index];

	RayDesc ray;
	ray.Origin = point_cloud_sample.start;
	ray.Direction = point_cloud_sample.direction;
	ray.TMin = 0.0f;
	ray.TMax = 1e20f;
	TraceRay(EE_TLAS, gl_RayFlagsOpaqueEXT, 0xff, 0, 0, 0, ray, hit_value);
	point_cloud_sample.hit_count = hit_value.hit_count;
	point_cloud_sample.handle = hit_value.handle;
	point_cloud_sample.hit_info = hit_value.hit_info;
	EE_POINT_CLOUD_SAMPLES[sample_index] = point_cloud_sample;

}
