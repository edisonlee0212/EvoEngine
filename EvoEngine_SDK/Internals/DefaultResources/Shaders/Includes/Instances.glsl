
struct Instance {
  mat4 model;
  int material_index;
  int triangle_offset;
  int meshlet_offset;
  int meshlet_size;

  int info_index;
  int padding0;
  int padding1;
  int padding2;
};

layout(set = EE_INSTANCES_BLOCK_SET, binding = EE_INSTANCES_BLOCK_BINDING) readonly buffer EE_INSTANCE_BLOCK {
  Instance EE_INSTANCES[];
};
