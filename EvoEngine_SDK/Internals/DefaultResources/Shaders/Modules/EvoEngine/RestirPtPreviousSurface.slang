import EvoEngine.RestirPt;
import EvoEngine.Instances;

public struct RestirPtPreviousInstance {
  float4x4 previous_model;
  uint4 flags;
};

[[vk::binding(21, 2)]] public StructuredBuffer<RestirPtPreviousInstance> previous_instances;

public bool EE_RESTIR_PT_INSTANCE_STABLE(const uint instance_index) {
  if (instance_index == EE_RESTIR_PT_INVALID_INSTANCE) return false;
  uint previous_count;
  uint previous_stride;
  uint current_count;
  uint current_stride;
  previous_instances.GetDimensions(previous_count, previous_stride);
  EE_INSTANCES.GetDimensions(current_count, current_stride);
  if (instance_index >= min(previous_count, current_count)) return false;
  const RestirPtPreviousInstance previous = previous_instances[instance_index];
  return previous.flags.x != 0u && previous.flags.z == instance_index + 1u;
}

public bool EE_RESTIR_PT_PREVIOUS_SURFACE(const RestirPtPrimarySurface current, out RestirPtPrimarySurface previous) {
  previous = current;
  if (current.instance_index == EE_RESTIR_PT_INVALID_INSTANCE) return false;
  uint previous_count;
  uint previous_stride;
  uint current_count;
  uint current_stride;
  previous_instances.GetDimensions(previous_count, previous_stride);
  EE_INSTANCES.GetDimensions(current_count, current_stride);
  if (current.instance_index >= min(previous_count, current_count)) return false;
  const float4x4 current_model = EE_INSTANCES[current.instance_index].model;
  const RestirPtPreviousInstance instance = previous_instances[current.instance_index];
  if (instance.flags.z != current.instance_index + 1u) return false;
  const float4x4 previous_model = instance.previous_model;
  if (!any(current_model[0] != previous_model[0]) && !any(current_model[1] != previous_model[1]) &&
      !any(current_model[2] != previous_model[2]) && !any(current_model[3] != previous_model[3])) return true;
  if (instance.flags.x == 0u) return false;

  const float3 current_row0 = current_model[0].xyz;
  const float3 current_row1 = current_model[1].xyz;
  const float3 current_row2 = current_model[2].xyz;
  const float current_determinant = dot(current_row0, cross(current_row1, current_row2));
  const float3 previous_row0 = previous_model[0].xyz;
  const float3 previous_row1 = previous_model[1].xyz;
  const float3 previous_row2 = previous_model[2].xyz;
  const float previous_determinant = dot(previous_row0, cross(previous_row1, previous_row2));
  if (abs(current_determinant) < 1e-8f || abs(previous_determinant) < 1e-8f) return false;
  const float3x3 current_linear = float3x3(current_row0, current_row1, current_row2);
  const float3x3 previous_linear = float3x3(previous_row0, previous_row1, previous_row2);
  const float3x3 current_inverse = transpose(float3x3(cross(current_row1, current_row2),
                                                      cross(current_row2, current_row0),
                                                      cross(current_row0, current_row1))) / current_determinant;
  const float3x3 previous_inverse = transpose(float3x3(cross(previous_row1, previous_row2),
                                                       cross(previous_row2, previous_row0),
                                                       cross(previous_row0, previous_row1))) / previous_determinant;
  const float3 local_position = mul(current.position - current_model[3].xyz, current_inverse);
  previous.position = mul(local_position, previous_linear) + previous_model[3].xyz;
  const float3 local_normal = mul(current.normal, transpose(current_linear));
  previous.normal = normalize(mul(local_normal, transpose(previous_inverse)));
  return all(isfinite(previous.position)) && all(isfinite(previous.normal));
}

public bool EE_RESTIR_PT_INSTANCE_UNMOVED(const uint instance_index) {
  if (!EE_RESTIR_PT_INSTANCE_STABLE(instance_index)) return false;
  const float4x4 current_model = EE_INSTANCES[instance_index].model;
  const float4x4 previous_model = previous_instances[instance_index].previous_model;
  return !any(current_model[0] != previous_model[0]) && !any(current_model[1] != previous_model[1]) &&
         !any(current_model[2] != previous_model[2]) && !any(current_model[3] != previous_model[3]);
}
