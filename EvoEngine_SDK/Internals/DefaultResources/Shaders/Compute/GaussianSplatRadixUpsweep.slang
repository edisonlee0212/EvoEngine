// @evoengine-dialect native
static const uint EE_GAUSSIAN_SPLAT_RADIX = 256u;
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
RWStructuredBuffer<uint> EE_GAUSSIAN_SPLAT_RADIX_GLOBAL_HISTOGRAM;

[[vk::binding(2, 0)]]
RWStructuredBuffer<uint> EE_GAUSSIAN_SPLAT_RADIX_PARTITION_HISTOGRAM;

[[vk::binding(3, 0)]]
StructuredBuffer<uint> EE_GAUSSIAN_SPLAT_RADIX_KEYS_IN;

struct EeGaussianSplatRadixConstants {
  uint4 pass_count_partition_reserved;
};

[[vk::push_constant]]
ConstantBuffer<EeGaussianSplatRadixConstants> constants;

groupshared uint EE_GAUSSIAN_SPLAT_LOCAL_HISTOGRAM[EE_GAUSSIAN_SPLAT_RADIX];

[shader("compute")]
[numthreads(512, 1, 1)]
void main(uint3 group_id : SV_GroupID, uint3 group_thread_id : SV_GroupThreadID) {
  const uint index = group_thread_id.x;
  const uint partition_index = group_id.x;
  const uint partition_start = partition_index * EE_GAUSSIAN_SPLAT_RADIX_PARTITION_SIZE;
  const uint element_count = EE_GAUSSIAN_SPLAT_DRAW_COMMANDS[0].instanceCount;
  const uint histogram_base = partition_index * EE_GAUSSIAN_SPLAT_RADIX;

  if (index < EE_GAUSSIAN_SPLAT_RADIX) {
    EE_GAUSSIAN_SPLAT_LOCAL_HISTOGRAM[index] = 0u;
    if (partition_start >= element_count) {
      EE_GAUSSIAN_SPLAT_RADIX_PARTITION_HISTOGRAM[histogram_base + index] = 0u;
    }
  }
  if (partition_start >= element_count) {
    return;
  }
  GroupMemoryBarrierWithGroupSync();

  for (uint i = 0u; i < EE_GAUSSIAN_SPLAT_RADIX_PARTITION_DIVISION; ++i) {
    const uint key_index = partition_start + 512u * i + index;
    const uint key = key_index < element_count ? EE_GAUSSIAN_SPLAT_RADIX_KEYS_IN[key_index] : 0xffffffffu;
    const uint radix = (key >> (8u * constants.pass_count_partition_reserved.x)) & 0xffu;
    InterlockedAdd(EE_GAUSSIAN_SPLAT_LOCAL_HISTOGRAM[radix], 1u);
  }
  GroupMemoryBarrierWithGroupSync();

  if (index < EE_GAUSSIAN_SPLAT_RADIX) {
    const uint count = EE_GAUSSIAN_SPLAT_LOCAL_HISTOGRAM[index];
    EE_GAUSSIAN_SPLAT_RADIX_PARTITION_HISTOGRAM[histogram_base + index] = count;
    InterlockedAdd(EE_GAUSSIAN_SPLAT_RADIX_GLOBAL_HISTOGRAM[index], count);
  }
}
