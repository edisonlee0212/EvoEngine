
struct Instance {
  mat4 model;
  uint material_index;
  uint triangle_offset;
  uint meshlet_offset;
  uint meshlet_size;

  uint info_index;
  uint padding0;
  uint padding1;
  uint padding2;
};

layout(set = EE_INSTANCES_BLOCK_SET, binding = EE_INSTANCES_BLOCK_BINDING) readonly buffer EE_INSTANCE_BLOCK {
  Instance EE_INSTANCES[];
};
