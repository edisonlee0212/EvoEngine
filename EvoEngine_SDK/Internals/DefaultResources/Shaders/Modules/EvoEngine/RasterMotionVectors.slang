public struct PreviousInstance {
  public float4x4 previous_model;
  public uint4 flags;
};

public struct RasterMotionVectorOutput {
  public float4 position : SV_Position;
  [[vk::location(0)]] public float2 tex_coord0 : TEXCOORD0;
  [[vk::location(1)]] public float4 current_clip : TEXCOORD1;
  [[vk::location(2)]] public float4 previous_clip : TEXCOORD2;
  [[vk::location(3)]] public nointerpolation uint instance_index : TEXCOORD3;
  [[vk::location(4)]] public nointerpolation uint previous_pose_valid : TEXCOORD4;
  [[vk::location(5)]] public float2 tex_coord1 : TEXCOORD5;
  [[vk::location(6)]] public float4 color : COLOR0;
  [[vk::location(7)]] public float2 tex_coord2 : TEXCOORD6;
  [[vk::location(8)]] public float2 tex_coord3 : TEXCOORD7;
};
