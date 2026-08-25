static const uint EE_GAUSSIAN_SPLAT_RADIX = 256u;

[[vk::binding(1, 0)]]
StructuredBuffer<uint> EE_GAUSSIAN_SPLAT_RADIX_GLOBAL_HISTOGRAM;

[[vk::binding(2, 0)]]
RWStructuredBuffer<uint> EE_GAUSSIAN_SPLAT_RADIX_PARTITION_HISTOGRAM;

struct EeGaussianSplatRadixConstants {
  uint4 pass_count_partition_reserved;
};

[[vk::push_constant]]
ConstantBuffer<EeGaussianSplatRadixConstants> constants;

[shader("compute")]
[numthreads(1, 1, 1)]
void main(uint3 group_id : SV_GroupID) {
  const uint radix = group_id.x;
  if (radix >= EE_GAUSSIAN_SPLAT_RADIX) {
    return;
  }

  uint running_offset = 0u;
  for (uint previous_radix = 0u; previous_radix < radix; ++previous_radix) {
    running_offset += EE_GAUSSIAN_SPLAT_RADIX_GLOBAL_HISTOGRAM[previous_radix];
  }

  const uint partition_count = constants.pass_count_partition_reserved.z;
  for (uint partition_index = 0u; partition_index < partition_count; ++partition_index) {
    const uint histogram_index = partition_index * EE_GAUSSIAN_SPLAT_RADIX + radix;
    const uint partition_count_for_radix = EE_GAUSSIAN_SPLAT_RADIX_PARTITION_HISTOGRAM[histogram_index];
    EE_GAUSSIAN_SPLAT_RADIX_PARTITION_HISTOGRAM[histogram_index] = running_offset;
    running_offset += partition_count_for_radix;
  }
}
