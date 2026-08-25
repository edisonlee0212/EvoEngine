import EvoEngine.PerFrame;

[[vk::binding(17, 1)]]
Sampler2D inDepth;

[[vk::binding(24, 1)]]
Sampler2D inUtility;

[[vk::binding(0, 2)]]
[vk::image_format("rgba16f")]
RWTexture2D<float4> outMotionVectors;

struct PreviousInstance {
  float4x4 previous_model;
  uint4 flags;
};

[[vk::binding(1, 2)]]
StructuredBuffer<PreviousInstance> EE_PREVIOUS_INSTANCES;

struct EeMotionVectorConstants {
  int camera_index;
  int instance_count;
};

[[vk::push_constant]]
ConstantBuffer<EeMotionVectorConstants> constants;

float CameraFarDistance() {
  float4x4 projection = EE_CAMERAS[constants.camera_index].projection;
  return abs(projection[3][2] / (projection[2][2] + 1.0f));
}

bool ProjectToPixel(float4x4 projection_view, float3 world_position, float2 resolution, out float2 pixel,
                    out float linear_depth) {
  float4 clip = mul(float4(world_position, 1.0f), projection_view);
  if (clip.w <= 0.0001f) {
    return false;
  }
  float3 ndc = clip.xyz / clip.w;
  pixel = (ndc.xy * 0.5f + float2(0.5f)) * resolution;
  linear_depth = clamp(clip.w / max(CameraFarDistance(), 1e-5f), 0.0f, 1.0f);
  return ndc.z >= -0.001f && ndc.z <= 1.001f;
}

float3 TransformBetweenModels(float3 world_position, float4x4 current_model, float4x4 previous_model) {
  float3 row0 = current_model[0].xyz;
  float3 row1 = current_model[1].xyz;
  float3 row2 = current_model[2].xyz;
  float determinant = dot(row0, cross(row1, row2));
  float3x3 inverse_linear = transpose(float3x3(cross(row1, row2), cross(row2, row0), cross(row0, row1))) /
                            determinant;
  float3 local_position = mul(world_position - current_model[3].xyz, inverse_linear);
  float3x3 previous_linear = float3x3(previous_model[0].xyz, previous_model[1].xyz, previous_model[2].xyz);
  return mul(local_position, previous_linear) + previous_model[3].xyz;
}

[shader("compute")]
[numthreads(16, 16, 1)]
void main(uint3 dispatch_thread_id : SV_DispatchThreadID) {
  int2 texel = int2(dispatch_thread_id.xy);
  uint width = 0;
  uint height = 0;
  outMotionVectors.GetDimensions(width, height);
  int2 resolution = int2(width, height);
  if (any(texel >= resolution)) {
    return;
  }

  float2 tex_coord = (float2(texel) + float2(0.5f)) / float2(resolution);
  float ndc_depth = inDepth.Load(int3(texel, 0)).x;
  bool has_geometry = ndc_depth < 1.0f;
  float reconstruction_depth = min(ndc_depth, 0.999999f);
  float3 current_world = EE_DEPTH_TO_WORLD_POS(constants.camera_index, tex_coord, reconstruction_depth);
  float3 previous_world = current_world;
  if (has_geometry) {
    int instance_index = int(round(inUtility.Load(int3(texel, 0)).x));
    if (instance_index >= 0 && instance_index < constants.instance_count &&
        EE_PREVIOUS_INSTANCES[instance_index].flags.x != 0u) {
      float4x4 current_model = EE_INSTANCES[instance_index].model;
      float4x4 previous_model = EE_PREVIOUS_INSTANCES[instance_index].previous_model;
      previous_world = TransformBetweenModels(current_world, current_model, previous_model);
    }
  }

  float2 current_pixel;
  float2 previous_pixel;
  float current_linear_depth;
  float previous_linear_depth;
  bool current_valid = ProjectToPixel(EE_CAMERAS[constants.camera_index].unjittered_projection_view, current_world,
                                      float2(resolution), current_pixel, current_linear_depth);
  bool previous_valid = ProjectToPixel(EE_CAMERAS[constants.camera_index].previous_unjittered_projection_view,
                                       previous_world, float2(resolution), previous_pixel, previous_linear_depth);
  float2 velocity_pixels = current_valid && previous_valid ? previous_pixel - current_pixel : float2(0.0f);
  float depth_delta = current_valid && previous_valid ? previous_linear_depth - current_linear_depth : 0.0f;
  outMotionVectors[texel] = float4(velocity_pixels, depth_delta, 0.0f);
}
