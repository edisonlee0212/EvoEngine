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

  int material_index_1;  // secondary material (-1 = unused)
  int material_index_2;  // tertiary material  (-1 = unused)

  int padding_0;  // std430 padding: mat4 forces 16-byte struct alignment → stride must be 112
  int padding_1;
};

layout(set = EE_INSTANCES_BLOCK_SET, binding = EE_INSTANCES_BLOCK_BINDING) readonly buffer EE_INSTANCE_BLOCK {
  Instance EE_INSTANCES[];
};
