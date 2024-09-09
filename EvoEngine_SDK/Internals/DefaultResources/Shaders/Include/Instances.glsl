
struct Instance {
  mat4 model;
  uint material_index;
  uint info_index;
  uint meshlet_offset;
  uint meshlet_size;
};

layout(set = EE_PER_FRAME_SET, binding = EE_INSTANCE_BLOCK_BINDING) readonly buffer EE_INSTANCE_BLOCK {
  Instance EE_INSTANCES[];
};
