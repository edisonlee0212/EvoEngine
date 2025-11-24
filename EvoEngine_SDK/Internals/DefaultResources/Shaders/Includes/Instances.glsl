#extension GL_ARB_gpu_shader_int64 : enable

struct Instance {
  mat4 model;
  int material_index;
  int triangle_offset;
  int meshlet_offset;
  int meshlet_size;

  int info_index;
  uint entity_index;
  uint64_t renderer_handle;
};

layout(set = EE_INSTANCES_BLOCK_SET, binding = EE_INSTANCES_BLOCK_BINDING) readonly buffer EE_INSTANCE_BLOCK {
  Instance EE_INSTANCES[];
};
