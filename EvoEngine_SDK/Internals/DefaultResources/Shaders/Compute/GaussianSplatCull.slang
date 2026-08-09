// @evoengine-dialect native
import EvoEngine.Cameras;
import EvoEngine.Instances;

static const uint EE_GAUSSIAN_SPLAT_MESH_SPLATS_PER_TASK = 32u;

struct GaussianSplat {
  float4 position_opacity;
  float4 scale_reserved;
  float4 rotation;
  float4 color_rest_offset;
};

struct DrawIndirectCommand {
  uint vertexCount;
  uint instanceCount;
  uint firstVertex;
  uint firstInstance;
};

struct MeshTasksIndirectCommand {
  uint groupCountX;
  uint groupCountY;
  uint groupCountZ;
};

[[vk::binding(0, 1)]]
StructuredBuffer<GaussianSplat> EE_GAUSSIAN_SPLATS;

[[vk::binding(1, 1)]]
RWStructuredBuffer<uint> EE_GAUSSIAN_SPLAT_VISIBLE_INDICES;

[[vk::binding(2, 1)]]
RWStructuredBuffer<uint> EE_GAUSSIAN_SPLAT_DEPTH_KEYS;

[[vk::binding(3, 1)]]
RWStructuredBuffer<DrawIndirectCommand> EE_GAUSSIAN_SPLAT_DRAW_COMMANDS;

[[vk::binding(4, 1)]]
RWStructuredBuffer<MeshTasksIndirectCommand> EE_GAUSSIAN_SPLAT_MESH_TASK_DRAW_COMMANDS;

struct EeGaussianSplatCullConstants {
  uint4 camera_instance_count_flags;
  float4 opacity_extent_min_max;
};

[[vk::push_constant]]
ConstantBuffer<EeGaussianSplatCullConstants> constants;

float EE_GAUSSIAN_SPLAT_SIGMOID(const float value) {
  return 1.0f / (1.0f + exp(-value));
}

uint EE_GAUSSIAN_SPLAT_DESCENDING_DEPTH_KEY(const float depth) {
  const uint positive_depth_key = asuint(max(depth, 0.0f));
  return 0xffffffffu - positive_depth_key;
}

bool EE_GAUSSIAN_SPLAT_CENTER_VISIBLE(const float4 clip_position) {
  if (clip_position.w <= 0.0f) {
    return false;
  }
  const float3 clip_abs = abs(clip_position.xyz);
  return clip_abs.x <= clip_position.w && clip_abs.y <= clip_position.w && clip_position.z >= 0.0f &&
         clip_position.z <= clip_position.w;
}

[shader("compute")]
[numthreads(256, 1, 1)]
void main(uint3 dispatch_thread_id : SV_DispatchThreadID) {
  const uint splat_index = dispatch_thread_id.x;
  const uint splat_count = constants.camera_instance_count_flags.z;
  if (splat_index >= splat_count) {
    return;
  }

  const int camera_index = int(constants.camera_instance_count_flags.x);
  const int instance_index = int(constants.camera_instance_count_flags.y);
  const GaussianSplat splat = EE_GAUSSIAN_SPLATS[splat_index];
  const float raw_opacity = splat.position_opacity.w;
  const float opacity = raw_opacity >= 0.0f && raw_opacity <= 1.0f ? raw_opacity : EE_GAUSSIAN_SPLAT_SIGMOID(raw_opacity);
  if (clamp(opacity * constants.opacity_extent_min_max.x, 0.0f, 1.0f) <= 1.0f / 255.0f) {
    return;
  }

  const float4x4 model = EE_INSTANCES[instance_index].model;
  const float4 world_center = mul(float4(splat.position_opacity.xyz, 1.0f), model);
  const float4 clip_center = mul(world_center, EE_CAMERAS[camera_index].projection_view);
  if (!EE_GAUSSIAN_SPLAT_CENTER_VISIBLE(clip_center)) {
    return;
  }

  uint visible_index;
  InterlockedAdd(EE_GAUSSIAN_SPLAT_DRAW_COMMANDS[0].instanceCount, 1u, visible_index);
  if (visible_index % EE_GAUSSIAN_SPLAT_MESH_SPLATS_PER_TASK == 0u) {
    InterlockedAdd(EE_GAUSSIAN_SPLAT_MESH_TASK_DRAW_COMMANDS[0].groupCountX, 1u);
  }
  EE_GAUSSIAN_SPLAT_VISIBLE_INDICES[visible_index] = splat_index;
  const float4 view_center = mul(world_center, EE_CAMERAS[camera_index].view);
  EE_GAUSSIAN_SPLAT_DEPTH_KEYS[visible_index] = EE_GAUSSIAN_SPLAT_DESCENDING_DEPTH_KEY(-view_center.z);
}
