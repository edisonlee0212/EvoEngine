import EvoEngine.Trace;

[[vk::binding(3, 0)]]
StructuredBuffer<float4> rays;

[[vk::binding(4, 0)]]
RWStructuredBuffer<float4> ray_casting_results;

struct RayCastingConstants {
  float4 ray_cast_config;
};

[[vk::push_constant]]
ConstantBuffer<RayCastingConstants> constants;

void Execute(uint current_ray_index) {
  float3 sample_origin = rays[current_ray_index * 2].xyz;
  float3 sample_direction = rays[current_ray_index * 2 + 1].xyz;
  RayDescriptor ray_descriptor;
  ray_descriptor.origin = sample_origin;
  ray_descriptor.direction = sample_direction;
  ray_descriptor.t_min = rays[current_ray_index * 2].w;
  ray_descriptor.t_max = rays[current_ray_index * 2 + 1].w;
  bool hit = false;
  HitInfo hit_info = Trace(ray_descriptor, constants.ray_cast_config.x != 0.0f,
                           constants.ray_cast_config.y != 0.0f, hit);
  if (hit) {
    ray_casting_results[current_ray_index * 4] = float4(hit_info.hit, 1.0f);
    ray_casting_results[current_ray_index * 4 + 1] =
        float4(hit_info.barycentric, hit_info.back_face ? 1.0f : 0.0f);
    ray_casting_results[current_ray_index * 4 + 2] = float4(hit_info.normal, hit_info.hit_dist);
    ray_casting_results[current_ray_index * 4 + 3] =
        float4(asfloat(hit_info.instance_index), asfloat(hit_info.node_index), asfloat(hit_info.mesh_index),
               asfloat(hit_info.local_triangle_index));
  } else {
    ray_casting_results[current_ray_index * 4] = float4(0.0f);
    ray_casting_results[current_ray_index * 4 + 1] = float4(0.0f);
    ray_casting_results[current_ray_index * 4 + 2] = float4(0.0f);
    ray_casting_results[current_ray_index * 4 + 3] = float4(0.0f);
  }
}

[shader("compute")]
[numthreads(COMPUTE_WORK_GROUP_INVOCATIONS, 1, 1)]
void main(uint3 dispatch_thread_id : SV_DispatchThreadID) {
  uint rays_count = asuint(constants.ray_cast_config.z);
  uint current_ray_index = dispatch_thread_id.x;
  if (current_ray_index >= rays_count) return;
  Execute(current_ray_index);
}
