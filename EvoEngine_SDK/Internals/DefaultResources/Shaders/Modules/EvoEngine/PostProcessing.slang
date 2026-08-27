public struct EeResolutionPushConstant {
  public uint2 resolution;
};

public struct EeBloomDownsamplingPushConstant {
  public uint2 source_resolution;
  public uint2 target_resolution;
  public int apply_karis;
  public int3 padding;
};

public struct EeBloomUpsamplingPushConstant {
  public uint2 source_resolution;
  public uint2 target_resolution;
  public float filter_radius;
  public float3 padding;
};

public bool EePixelInBounds(uint2 pixel, uint2 resolution) {
  return pixel.x < resolution.x && pixel.y < resolution.y;
}

public int2 EeTexel(uint2 pixel) {
  return int2(pixel);
}
