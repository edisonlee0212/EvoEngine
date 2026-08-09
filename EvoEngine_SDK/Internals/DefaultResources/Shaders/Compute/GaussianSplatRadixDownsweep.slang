// @evoengine-dialect native
static const uint EE_GAUSSIAN_SPLAT_RADIX = 256u;
static const uint EE_GAUSSIAN_SPLAT_RADIX_WORKGROUP_SIZE = 512u;
static const uint EE_GAUSSIAN_SPLAT_RADIX_PARTITION_DIVISION = 8u;
static const uint EE_GAUSSIAN_SPLAT_RADIX_PARTITION_SIZE = EE_GAUSSIAN_SPLAT_RADIX_PARTITION_DIVISION * 512u;

struct DrawIndirectCommand {
  uint vertexCount;
  uint instanceCount;
  uint firstVertex;
  uint firstInstance;
};

[[vk::binding(0, 0)]]
StructuredBuffer<DrawIndirectCommand> EE_GAUSSIAN_SPLAT_DRAW_COMMANDS;

[[vk::binding(1, 0)]]
StructuredBuffer<uint> EE_GAUSSIAN_SPLAT_RADIX_GLOBAL_HISTOGRAM;

[[vk::binding(2, 0)]]
StructuredBuffer<uint> EE_GAUSSIAN_SPLAT_RADIX_PARTITION_HISTOGRAM;

[[vk::binding(3, 0)]]
StructuredBuffer<uint> EE_GAUSSIAN_SPLAT_RADIX_KEYS_IN;

[[vk::binding(4, 0)]]
RWStructuredBuffer<uint> EE_GAUSSIAN_SPLAT_RADIX_KEYS_OUT;

[[vk::binding(5, 0)]]
StructuredBuffer<uint> EE_GAUSSIAN_SPLAT_RADIX_VALUES_IN;

[[vk::binding(6, 0)]]
RWStructuredBuffer<uint> EE_GAUSSIAN_SPLAT_RADIX_VALUES_OUT;

struct EeGaussianSplatRadixConstants {
  uint4 pass_count_partition_reserved;
};

[[vk::push_constant]]
ConstantBuffer<EeGaussianSplatRadixConstants> constants;

groupshared uint EE_GAUSSIAN_SPLAT_LOCAL_HISTOGRAM[EE_GAUSSIAN_SPLAT_RADIX_PARTITION_SIZE];
groupshared uint EE_GAUSSIAN_SPLAT_LOCAL_HISTOGRAM_SUM[EE_GAUSSIAN_SPLAT_RADIX];

uint4 EE_GAUSSIAN_SPLAT_EXCLUSIVE_WAVE_MASK(const uint id) {
  const uint shift = (1u << (id & 31u)) - 1u;
  const int x = int(id) >> 5;
  return uint4((shift & uint((-1 - x) >> 31)) | uint((0 - x) >> 31),
               (shift & uint((0 - x) >> 31)) | uint((1 - x) >> 31),
               (shift & uint((1 - x) >> 31)) | uint((2 - x) >> 31),
               (shift & uint((2 - x) >> 31)) | uint((3 - x) >> 31));
}

uint EE_GAUSSIAN_SPLAT_BIT_COUNT(const uint4 value) {
  return countbits(value.x) + countbits(value.y) + countbits(value.z) + countbits(value.w);
}

[shader("compute")]
[numthreads(512, 1, 1)]
void main(uint3 group_id : SV_GroupID, uint group_index : SV_GroupIndex) {
  const uint element_count = EE_GAUSSIAN_SPLAT_DRAW_COMMANDS[0].instanceCount;
  const uint subgroup_size = WaveGetLaneCount();
  const uint lane_index = WaveGetLaneIndex();
  const uint wave_index = group_index / subgroup_size;
  const uint wave_count = EE_GAUSSIAN_SPLAT_RADIX_WORKGROUP_SIZE / subgroup_size;
  const uint index = wave_index * subgroup_size + lane_index;
  const uint partition_index = group_id.x;
  const uint partition_start = partition_index * EE_GAUSSIAN_SPLAT_RADIX_PARTITION_SIZE;

  if (partition_start >= element_count) {
    return;
  }

  const uint4 wave_mask = EE_GAUSSIAN_SPLAT_EXCLUSIVE_WAVE_MASK(lane_index);
  if (index < EE_GAUSSIAN_SPLAT_RADIX) {
    for (uint i = 0u; i < wave_count; ++i) {
      EE_GAUSSIAN_SPLAT_LOCAL_HISTOGRAM[wave_count * index + i] = 0u;
    }
  }
  GroupMemoryBarrierWithGroupSync();

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

    const uint radix = (key >> (constants.pass_count_partition_reserved.x * 8u)) & 0xffu;
    local_radix[i] = radix;

    uint4 mask = WaveActiveBallot(true);
    for (uint bit_index = 0u; bit_index < 8u; ++bit_index) {
      const uint digit = (radix >> bit_index) & 1u;
      const uint4 ballot = WaveActiveBallot(digit == 1u);
      mask &= uint4(digit - 1u) ^ ballot;
    }

    const uint wave_offset = EE_GAUSSIAN_SPLAT_BIT_COUNT(wave_mask & mask);
    const uint radix_count = EE_GAUSSIAN_SPLAT_BIT_COUNT(mask);
    if (wave_offset == 0u) {
      InterlockedAdd(EE_GAUSSIAN_SPLAT_LOCAL_HISTOGRAM[wave_count * radix + wave_index], radix_count);
      wave_histogram[i] = radix_count;
    } else {
      wave_histogram[i] = 0u;
    }
    local_offsets[i] = wave_offset;
  }
  GroupMemoryBarrierWithGroupSync();

  for (uint i = index; i < EE_GAUSSIAN_SPLAT_RADIX * wave_count; i += EE_GAUSSIAN_SPLAT_RADIX_WORKGROUP_SIZE) {
    const uint value = EE_GAUSSIAN_SPLAT_LOCAL_HISTOGRAM[i];
    const uint sum = WaveActiveSum(value);
    const uint exclusive = WavePrefixSum(value);
    EE_GAUSSIAN_SPLAT_LOCAL_HISTOGRAM[i] = exclusive;
    if (WaveIsFirstLane()) {
      EE_GAUSSIAN_SPLAT_LOCAL_HISTOGRAM_SUM[i / subgroup_size] = sum;
    }
  }
  GroupMemoryBarrierWithGroupSync();

  const uint intermediate_offset0 = EE_GAUSSIAN_SPLAT_RADIX * wave_count / subgroup_size;
  if (index < intermediate_offset0) {
    const uint value = EE_GAUSSIAN_SPLAT_LOCAL_HISTOGRAM_SUM[index];
    const uint sum = WaveActiveSum(value);
    const uint exclusive = WavePrefixSum(value);
    EE_GAUSSIAN_SPLAT_LOCAL_HISTOGRAM_SUM[index] = exclusive;
    if (WaveIsFirstLane()) {
      EE_GAUSSIAN_SPLAT_LOCAL_HISTOGRAM_SUM[intermediate_offset0 + index / subgroup_size] = sum;
    }
  }
  GroupMemoryBarrierWithGroupSync();

  const uint intermediate_size1 = max(EE_GAUSSIAN_SPLAT_RADIX * wave_count / subgroup_size / subgroup_size, 1u);
  if (index < intermediate_size1) {
    const uint value = EE_GAUSSIAN_SPLAT_LOCAL_HISTOGRAM_SUM[intermediate_offset0 + index];
    EE_GAUSSIAN_SPLAT_LOCAL_HISTOGRAM_SUM[intermediate_offset0 + index] = WavePrefixSum(value);
  }
  GroupMemoryBarrierWithGroupSync();

  if (index < intermediate_offset0) {
    EE_GAUSSIAN_SPLAT_LOCAL_HISTOGRAM_SUM[index] +=
        EE_GAUSSIAN_SPLAT_LOCAL_HISTOGRAM_SUM[intermediate_offset0 + index / subgroup_size];
  }
  GroupMemoryBarrierWithGroupSync();

  for (uint i = index; i < EE_GAUSSIAN_SPLAT_RADIX * wave_count; i += EE_GAUSSIAN_SPLAT_RADIX_WORKGROUP_SIZE) {
    EE_GAUSSIAN_SPLAT_LOCAL_HISTOGRAM[i] += EE_GAUSSIAN_SPLAT_LOCAL_HISTOGRAM_SUM[i / subgroup_size];
  }
  GroupMemoryBarrierWithGroupSync();

  for (uint i = 0u; i < EE_GAUSSIAN_SPLAT_RADIX_PARTITION_DIVISION; ++i) {
    const uint radix = local_radix[i];
    local_offsets[i] += EE_GAUSSIAN_SPLAT_LOCAL_HISTOGRAM[wave_count * radix + wave_index];

    GroupMemoryBarrierWithGroupSync();
    if (wave_histogram[i] > 0u) {
      InterlockedAdd(EE_GAUSSIAN_SPLAT_LOCAL_HISTOGRAM[wave_count * radix + wave_index], wave_histogram[i]);
    }
    GroupMemoryBarrierWithGroupSync();
  }

  if (index < EE_GAUSSIAN_SPLAT_RADIX) {
    const uint previous = index == 0u ? 0u : EE_GAUSSIAN_SPLAT_LOCAL_HISTOGRAM[wave_count * index - 1u];
    EE_GAUSSIAN_SPLAT_LOCAL_HISTOGRAM_SUM[index] =
        EE_GAUSSIAN_SPLAT_RADIX_PARTITION_HISTOGRAM[EE_GAUSSIAN_SPLAT_RADIX * partition_index + index] - previous;
  }
  GroupMemoryBarrierWithGroupSync();

  for (uint i = 0u; i < EE_GAUSSIAN_SPLAT_RADIX_PARTITION_DIVISION; ++i) {
    EE_GAUSSIAN_SPLAT_LOCAL_HISTOGRAM[local_offsets[i]] = local_keys[i];
  }
  GroupMemoryBarrierWithGroupSync();

  for (uint i = index; i < EE_GAUSSIAN_SPLAT_RADIX_PARTITION_SIZE; i += EE_GAUSSIAN_SPLAT_RADIX_WORKGROUP_SIZE) {
    const uint key = EE_GAUSSIAN_SPLAT_LOCAL_HISTOGRAM[i];
    const uint radix = (key >> (constants.pass_count_partition_reserved.x * 8u)) & 0xffu;
    const uint destination_offset = EE_GAUSSIAN_SPLAT_LOCAL_HISTOGRAM_SUM[radix] + i;
    if (destination_offset < element_count) {
      EE_GAUSSIAN_SPLAT_RADIX_KEYS_OUT[destination_offset] = key;
    }
    local_keys[i / EE_GAUSSIAN_SPLAT_RADIX_WORKGROUP_SIZE] = destination_offset;
  }
  GroupMemoryBarrierWithGroupSync();

  for (uint i = 0u; i < EE_GAUSSIAN_SPLAT_RADIX_PARTITION_DIVISION; ++i) {
    EE_GAUSSIAN_SPLAT_LOCAL_HISTOGRAM[local_offsets[i]] = local_values[i];
  }
  GroupMemoryBarrierWithGroupSync();

  for (uint i = index; i < EE_GAUSSIAN_SPLAT_RADIX_PARTITION_SIZE; i += EE_GAUSSIAN_SPLAT_RADIX_WORKGROUP_SIZE) {
    const uint destination_offset = local_keys[i / EE_GAUSSIAN_SPLAT_RADIX_WORKGROUP_SIZE];
    if (destination_offset < element_count) {
      EE_GAUSSIAN_SPLAT_RADIX_VALUES_OUT[destination_offset] = EE_GAUSSIAN_SPLAT_LOCAL_HISTOGRAM[i];
    }
  }
}
