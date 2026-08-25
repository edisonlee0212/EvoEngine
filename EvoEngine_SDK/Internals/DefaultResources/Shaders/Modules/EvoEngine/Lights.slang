// Lights
public struct DirectionalLight {
  public float3 direction;
  public float4 diffuse;
  public float3 specular;
  public float4x4 light_space_matrix[4];
  public float4 light_frustum_width;
  public float4 light_frustum_height;
  public float4 light_frustum_distance;
  public float4 reserved_parameters;
  public int viewport_x_offset;
  public int viewport_y_offset;
  public int viewport_x_size;
  public int viewport_y_size;
};

public struct PointLight {
  public float3 position;
  public float4 constant_linear_quadratic_far;
  public float4 diffuse;
  public float3 specular;
  public float4x4 light_space_matrix[6];
  public float4 reserved_parameters;
  public int viewport_x_offset;
  public int viewport_y_offset;
  public int viewport_x_size;
  public int viewport_y_size;
};

public struct SpotLight {
  public float3 position;
  public float padding0;

  public float3 direction;
  public float padding1;

  public float4x4 light_space_matrix;
  public float4 cutoff_outer_inner_size_bias;
  public float4 constant_linear_quadratic_far;
  public float4 diffuse;
  public float3 specular;
  public float padding3;
  public int viewport_x_offset;
  public int viewport_y_offset;
  public int viewport_x_size;
  public int viewport_y_size;
};

[[vk::binding(6, 0)]]
public StructuredBuffer<DirectionalLight, Std430DataLayout> EE_DIRECTIONAL_LIGHTS;

[[vk::binding(7, 0)]]
public StructuredBuffer<PointLight, Std430DataLayout> EE_POINT_LIGHTS;

[[vk::binding(8, 0)]]
public StructuredBuffer<SpotLight, Std430DataLayout> EE_SPOT_LIGHTS;
