#extension GL_KHR_shader_subgroup_basic : enable
#extension GL_KHR_shader_subgroup_arithmetic : enable
#extension GL_KHR_shader_subgroup_ballot : enable

const uint EE_GAUSSIAN_SPLAT_RADIX = 256u;
const uint EE_GAUSSIAN_SPLAT_RADIX_WORKGROUP_SIZE = 512u;
const uint EE_GAUSSIAN_SPLAT_RADIX_PARTITION_DIVISION = 8u;
const uint EE_GAUSSIAN_SPLAT_RADIX_PARTITION_SIZE = EE_GAUSSIAN_SPLAT_RADIX_PARTITION_DIVISION * 512u;

struct DrawIndirectCommand {
  uint vertexCount;
  uint instanceCount;
  uint firstVertex;
  uint firstInstance;
};

layout(set = 0, binding = 0) readonly buffer EE_GAUSSIAN_SPLAT_INDIRECT_DRAW_BLOCK {
  DrawIndirectCommand EE_GAUSSIAN_SPLAT_DRAW_COMMAND;
};

layout(set = 0, binding = 1) readonly buffer EE_GAUSSIAN_SPLAT_RADIX_GLOBAL_HISTOGRAM_BLOCK {
  uint EE_GAUSSIAN_SPLAT_RADIX_GLOBAL_HISTOGRAM[];
};

layout(set = 0, binding = 2) readonly buffer EE_GAUSSIAN_SPLAT_RADIX_PARTITION_HISTOGRAM_BLOCK {
  uint EE_GAUSSIAN_SPLAT_RADIX_PARTITION_HISTOGRAM[];
};

layout(set = 0, binding = 3) readonly buffer EE_GAUSSIAN_SPLAT_RADIX_KEY_INPUT_BLOCK {
  uint EE_GAUSSIAN_SPLAT_RADIX_KEYS_IN[];
};

layout(set = 0, binding = 4) buffer EE_GAUSSIAN_SPLAT_RADIX_KEY_OUTPUT_BLOCK {
  uint EE_GAUSSIAN_SPLAT_RADIX_KEYS_OUT[];
};

layout(set = 0, binding = 5) readonly buffer EE_GAUSSIAN_SPLAT_RADIX_VALUE_INPUT_BLOCK {
  uint EE_GAUSSIAN_SPLAT_RADIX_VALUES_IN[];
};

layout(set = 0, binding = 6) buffer EE_GAUSSIAN_SPLAT_RADIX_VALUE_OUTPUT_BLOCK {
  uint EE_GAUSSIAN_SPLAT_RADIX_VALUES_OUT[];
};

layout(push_constant) uniform EE_GAUSSIAN_SPLAT_RADIX_CONSTANTS {
  uvec4 pass_count_partition_reserved;
};

shared uint EE_GAUSSIAN_SPLAT_LOCAL_HISTOGRAM[EE_GAUSSIAN_SPLAT_RADIX_PARTITION_SIZE];
shared uint EE_GAUSSIAN_SPLAT_LOCAL_HISTOGRAM_SUM[EE_GAUSSIAN_SPLAT_RADIX];

uvec4 EE_GAUSSIAN_SPLAT_EXCLUSIVE_WAVE_MASK(const uint id) {
  const uint shift = (1u << bitfieldExtract(id, 0, 5)) - 1u;
  const int x = int(id) >> 5;
  return uvec4((shift & uint((-1 - x) >> 31)) | uint((0 - x) >> 31),
               (shift & uint((0 - x) >> 31)) | uint((1 - x) >> 31),
               (shift & uint((1 - x) >> 31)) | uint((2 - x) >> 31),
               (shift & uint((2 - x) >> 31)) | uint((3 - x) >> 31));
}

uint EE_GAUSSIAN_SPLAT_BIT_COUNT(const uvec4 value) {
  return bitCount(value.x) + bitCount(value.y) + bitCount(value.z) + bitCount(value.w);
}

[shader("compute")]
[numthreads(512, 1, 1)]
void main(uint3 group_id : SV_GroupID, uint group_index : SV_GroupIndex) {
  const uint element_count = EE_GAUSSIAN_SPLAT_DRAW_COMMAND.instanceCount;
  const uint subgroup_size = gl_SubgroupSize;
  const uint lane_index = gl_SubgroupInvocationID;
  const uint wave_index = group_index / subgroup_size;
  const uint wave_count = EE_GAUSSIAN_SPLAT_RADIX_WORKGROUP_SIZE / subgroup_size;
  const uint index = wave_index * subgroup_size + lane_index;
  const uint partition_index = group_id.x;
  const uint partition_start = partition_index * EE_GAUSSIAN_SPLAT_RADIX_PARTITION_SIZE;

  if (partition_start >= element_count) {
    return;
  }

  const uvec4 wave_mask = EE_GAUSSIAN_SPLAT_EXCLUSIVE_WAVE_MASK(lane_index);
  if (index < EE_GAUSSIAN_SPLAT_RADIX) {
    for (uint i = 0u; i < wave_count; ++i) {
      EE_GAUSSIAN_SPLAT_LOCAL_HISTOGRAM[wave_count * index + i] = 0u;
    }
  }
  barrier();

  uint local_keys[EE_GAUSSIAN_SPLAT_RADIX_PARTITION_DIVISION];
  uint local_values[EE_GAUSSIAN_SPLAT_RADIX_PARTITION_DIVISION];
  uint local_radix[EE_GAUSSIAN_SPLAT_RADIX_PARTITION_DIVISION];
  uint local_offsets[EE_GAUSSIAN_SPLAT_RADIX_PARTITION_DIVISION];
  uint wave_histogram[EE_GAUSSIAN_SPLAT_RADIX_PARTITION_DIVISION];

  for (uint i = 0u; i < EE_GAUSSIAN_SPLAT_RADIX_PARTITION_DIVISION; ++i) {
    const uint key_index =
        partition_start + EE_GAUSSIAN_SPLAT_RADIX_PARTITION_DIVISION * subgroup_size * wave_index +
        i * subgroup_size + lane_index;
    const bool in_range = key_index < element_count;
    const uint key = in_range ? EE_GAUSSIAN_SPLAT_RADIX_KEYS_IN[key_index] : 0xffffffffu;
    local_keys[i] = key;
    local_values[i] = in_range ? EE_GAUSSIAN_SPLAT_RADIX_VALUES_IN[key_index] : 0u;

    const uint radix = bitfieldExtract(key, int(pass_count_partition_reserved.x * 8u), 8);
    local_radix[i] = radix;

    uvec4 mask = subgroupBallot(true);
    for (uint bit_index = 0u; bit_index < 8u; ++bit_index) {
      const uint digit = (radix >> bit_index) & 1u;
      const uvec4 ballot = subgroupBallot(digit == 1u);
      mask &= uvec4(digit - 1u) ^ ballot;
    }

    const uint wave_offset = EE_GAUSSIAN_SPLAT_BIT_COUNT(wave_mask & mask);
    const uint radix_count = EE_GAUSSIAN_SPLAT_BIT_COUNT(mask);
    if (wave_offset == 0u) {
      atomicAdd(EE_GAUSSIAN_SPLAT_LOCAL_HISTOGRAM[wave_count * radix + wave_index], radix_count);
      wave_histogram[i] = radix_count;
    } else {
      wave_histogram[i] = 0u;
    }
    local_offsets[i] = wave_offset;
  }
  barrier();

  for (uint i = index; i < EE_GAUSSIAN_SPLAT_RADIX * wave_count; i += EE_GAUSSIAN_SPLAT_RADIX_WORKGROUP_SIZE) {
    const uint value = EE_GAUSSIAN_SPLAT_LOCAL_HISTOGRAM[i];
    const uint sum = subgroupAdd(value);
    const uint exclusive = subgroupExclusiveAdd(value);
    EE_GAUSSIAN_SPLAT_LOCAL_HISTOGRAM[i] = exclusive;
    if (subgroupElect()) {
      EE_GAUSSIAN_SPLAT_LOCAL_HISTOGRAM_SUM[i / subgroup_size] = sum;
    }
  }
  barrier();

  const uint intermediate_offset0 = EE_GAUSSIAN_SPLAT_RADIX * wave_count / subgroup_size;
  if (index < intermediate_offset0) {
    const uint value = EE_GAUSSIAN_SPLAT_LOCAL_HISTOGRAM_SUM[index];
    const uint sum = subgroupAdd(value);
    const uint exclusive = subgroupExclusiveAdd(value);
    EE_GAUSSIAN_SPLAT_LOCAL_HISTOGRAM_SUM[index] = exclusive;
    if (subgroupElect()) {
      EE_GAUSSIAN_SPLAT_LOCAL_HISTOGRAM_SUM[intermediate_offset0 + index / subgroup_size] = sum;
    }
  }
  barrier();

  const uint intermediate_size1 = max(EE_GAUSSIAN_SPLAT_RADIX * wave_count / subgroup_size / subgroup_size, 1u);
  if (index < intermediate_size1) {
    const uint value = EE_GAUSSIAN_SPLAT_LOCAL_HISTOGRAM_SUM[intermediate_offset0 + index];
    EE_GAUSSIAN_SPLAT_LOCAL_HISTOGRAM_SUM[intermediate_offset0 + index] = subgroupExclusiveAdd(value);
  }
  barrier();

  if (index < intermediate_offset0) {
    EE_GAUSSIAN_SPLAT_LOCAL_HISTOGRAM_SUM[index] +=
        EE_GAUSSIAN_SPLAT_LOCAL_HISTOGRAM_SUM[intermediate_offset0 + index / subgroup_size];
  }
  barrier();

  for (uint i = index; i < EE_GAUSSIAN_SPLAT_RADIX * wave_count; i += EE_GAUSSIAN_SPLAT_RADIX_WORKGROUP_SIZE) {
    EE_GAUSSIAN_SPLAT_LOCAL_HISTOGRAM[i] += EE_GAUSSIAN_SPLAT_LOCAL_HISTOGRAM_SUM[i / subgroup_size];
  }
  barrier();

  for (uint i = 0u; i < EE_GAUSSIAN_SPLAT_RADIX_PARTITION_DIVISION; ++i) {
    const uint radix = local_radix[i];
    local_offsets[i] += EE_GAUSSIAN_SPLAT_LOCAL_HISTOGRAM[wave_count * radix + wave_index];

    barrier();
    if (wave_histogram[i] > 0u) {
      atomicAdd(EE_GAUSSIAN_SPLAT_LOCAL_HISTOGRAM[wave_count * radix + wave_index], wave_histogram[i]);
    }
    barrier();
  }

  if (index < EE_GAUSSIAN_SPLAT_RADIX) {
    const uint previous = index == 0u ? 0u : EE_GAUSSIAN_SPLAT_LOCAL_HISTOGRAM[wave_count * index - 1u];
    EE_GAUSSIAN_SPLAT_LOCAL_HISTOGRAM_SUM[index] =
        EE_GAUSSIAN_SPLAT_RADIX_PARTITION_HISTOGRAM[EE_GAUSSIAN_SPLAT_RADIX * partition_index + index] - previous;
  }
  barrier();

  for (uint i = 0u; i < EE_GAUSSIAN_SPLAT_RADIX_PARTITION_DIVISION; ++i) {
    EE_GAUSSIAN_SPLAT_LOCAL_HISTOGRAM[local_offsets[i]] = local_keys[i];
  }
  barrier();

  for (uint i = index; i < EE_GAUSSIAN_SPLAT_RADIX_PARTITION_SIZE; i += EE_GAUSSIAN_SPLAT_RADIX_WORKGROUP_SIZE) {
    const uint key = EE_GAUSSIAN_SPLAT_LOCAL_HISTOGRAM[i];
    const uint radix = bitfieldExtract(key, int(pass_count_partition_reserved.x * 8u), 8);
    const uint destination_offset = EE_GAUSSIAN_SPLAT_LOCAL_HISTOGRAM_SUM[radix] + i;
    if (destination_offset < element_count) {
      EE_GAUSSIAN_SPLAT_RADIX_KEYS_OUT[destination_offset] = key;
    }
    local_keys[i / EE_GAUSSIAN_SPLAT_RADIX_WORKGROUP_SIZE] = destination_offset;
  }
  barrier();

  for (uint i = 0u; i < EE_GAUSSIAN_SPLAT_RADIX_PARTITION_DIVISION; ++i) {
    EE_GAUSSIAN_SPLAT_LOCAL_HISTOGRAM[local_offsets[i]] = local_values[i];
  }
  barrier();

  for (uint i = index; i < EE_GAUSSIAN_SPLAT_RADIX_PARTITION_SIZE; i += EE_GAUSSIAN_SPLAT_RADIX_WORKGROUP_SIZE) {
    const uint destination_offset = local_keys[i / EE_GAUSSIAN_SPLAT_RADIX_WORKGROUP_SIZE];
    if (destination_offset < element_count) {
      EE_GAUSSIAN_SPLAT_RADIX_VALUES_OUT[destination_offset] = EE_GAUSSIAN_SPLAT_LOCAL_HISTOGRAM[i];
    }
  }
}
