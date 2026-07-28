#extension GL_ARB_shading_language_include : enable

#define EE_CAMERAS_BLOCK_SET 0
#define EE_CAMERAS_BLOCK_BINDING 2
#include "Cameras.slangh"
#define EE_INSTANCES_BLOCK_SET 0
#define EE_INSTANCES_BLOCK_BINDING 4
#include "Instances.slangh"

const uint EE_GAUSSIAN_SPLAT_MESH_SPLATS_PER_TASK = 32u;

struct GaussianSplat {
  vec4 position_opacity;
  vec4 scale_reserved;
  vec4 rotation;
  vec4 color_rest_offset;
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

layout(set = 1, binding = 0) readonly buffer EE_GAUSSIAN_SPLAT_BLOCK {
  GaussianSplat EE_GAUSSIAN_SPLATS[];
};

layout(set = 1, binding = 1) writeonly buffer EE_GAUSSIAN_SPLAT_VISIBLE_INDEX_BLOCK {
  uint EE_GAUSSIAN_SPLAT_VISIBLE_INDICES[];
};

layout(set = 1, binding = 2) writeonly buffer EE_GAUSSIAN_SPLAT_DEPTH_KEY_BLOCK {
  uint EE_GAUSSIAN_SPLAT_DEPTH_KEYS[];
};

layout(set = 1, binding = 3) buffer EE_GAUSSIAN_SPLAT_INDIRECT_DRAW_BLOCK {
  DrawIndirectCommand EE_GAUSSIAN_SPLAT_DRAW_COMMAND;
};

layout(set = 1, binding = 4) buffer EE_GAUSSIAN_SPLAT_MESH_TASK_INDIRECT_DRAW_BLOCK {
  MeshTasksIndirectCommand EE_GAUSSIAN_SPLAT_MESH_TASK_DRAW_COMMAND;
};

layout(push_constant) uniform EE_GAUSSIAN_SPLAT_CULL_CONSTANTS {
  uvec4 camera_instance_count_flags;
  vec4 opacity_extent_min_max;
};

float EE_GAUSSIAN_SPLAT_SIGMOID(const float value) {
  return 1.0f / (1.0f + exp(-value));
}

uint EE_GAUSSIAN_SPLAT_DESCENDING_DEPTH_KEY(const float depth) {
  const uint positive_depth_key = floatBitsToUint(max(depth, 0.0f));
  return 0xffffffffu - positive_depth_key;
}

bool EE_GAUSSIAN_SPLAT_CENTER_VISIBLE(const vec4 clip_position) {
  if (clip_position.w <= 0.0f) {
    return false;
  }
  const vec3 clip_abs = abs(clip_position.xyz);
  return clip_abs.x <= clip_position.w && clip_abs.y <= clip_position.w && clip_position.z >= 0.0f &&
         clip_position.z <= clip_position.w;
}

[shader("compute")]
[numthreads(256, 1, 1)]
void main(uint3 dispatch_thread_id : SV_DispatchThreadID) {
  const uint splat_index = dispatch_thread_id.x;
  const uint splat_count = camera_instance_count_flags.z;
  if (splat_index >= splat_count) {
    return;
  }

  const int camera_index = int(camera_instance_count_flags.x);
  const int instance_index = int(camera_instance_count_flags.y);
  const GaussianSplat splat = EE_GAUSSIAN_SPLATS[splat_index];
  const float raw_opacity = splat.position_opacity.w;
  const float opacity = raw_opacity >= 0.0f && raw_opacity <= 1.0f ? raw_opacity : EE_GAUSSIAN_SPLAT_SIGMOID(raw_opacity);
  if (clamp(opacity * opacity_extent_min_max.x, 0.0f, 1.0f) <= 1.0f / 255.0f) {
    return;
  }

  const mat4 model = EE_INSTANCES[instance_index].model;
  const vec4 world_center = model * vec4(splat.position_opacity.xyz, 1.0f);
  const vec4 clip_center = EE_CAMERAS[camera_index].projection_view * world_center;
  if (!EE_GAUSSIAN_SPLAT_CENTER_VISIBLE(clip_center)) {
    return;
  }

  const uint visible_index = atomicAdd(EE_GAUSSIAN_SPLAT_DRAW_COMMAND.instanceCount, 1u);
  if (visible_index % EE_GAUSSIAN_SPLAT_MESH_SPLATS_PER_TASK == 0u) {
    atomicAdd(EE_GAUSSIAN_SPLAT_MESH_TASK_DRAW_COMMAND.groupCountX, 1u);
  }
  EE_GAUSSIAN_SPLAT_VISIBLE_INDICES[visible_index] = splat_index;
  const vec4 view_center = EE_CAMERAS[camera_index].view * world_center;
  EE_GAUSSIAN_SPLAT_DEPTH_KEYS[visible_index] = EE_GAUSSIAN_SPLAT_DESCENDING_DEPTH_KEY(-view_center.z);
}
