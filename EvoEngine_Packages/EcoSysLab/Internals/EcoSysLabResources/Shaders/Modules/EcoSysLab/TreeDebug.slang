public struct TreeDebugState {
  public float4x4 model;
  public float4x4 projection_view;
  public uint4 selected;
  public float4 selection;
  public float4x4 inverse_projection_view;
};
[[vk::binding(3, 3)]] public StructuredBuffer<TreeDebugState, Std430DataLayout> tree_debug_state;
[[vk::binding(4, 3)]] public StructuredBuffer<float4> tree_debug_colors;
