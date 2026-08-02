// @evoengine-dialect native
public struct Environment {
  public float4 background_color;
  public float gamma;
  public float diffuse_sky_intensity;
  public float global_reflection_intensity;
  public float environment_type;
  public float environment_pdf_texture_index;
  public float environment_cubemap_index;
  public float environment_rotation;
  public float diffuse_fallback_intensity;
  public float specular_fallback_intensity;
};

[[vk::binding(1, 0)]]
public ConstantBuffer<Environment, Std140DataLayout> EE_ENVIRONMENT;

public float3 EE_ENVIRONMENT_ROTATE_Y(const float3 direction, const float angle) {
  const float c = cos(angle);
  const float s = sin(angle);
  return float3(c * direction.x + s * direction.z, direction.y, -s * direction.x + c * direction.z);
}

public float3 EE_ENVIRONMENT_LOCAL_DIRECTION(const float3 world_direction) {
  return EE_ENVIRONMENT_ROTATE_Y(world_direction, -EE_ENVIRONMENT.environment_rotation);
}

public float3 EE_ENVIRONMENT_WORLD_DIRECTION(const float3 local_direction) {
  return EE_ENVIRONMENT_ROTATE_Y(local_direction, EE_ENVIRONMENT.environment_rotation);
}
