struct Input {
  float4 position : SV_Position;
  [[vk::location(0)]] float4 color : COLOR0;
};

[shader("fragment")]
float4 main(Input input) : SV_Target { return input.color; }
