public struct EeResolutionPushConstant {
  public uint2 resolution;
};

public struct EeBloomDownsamplingPushConstant {
  public float2 source_resolution;
  public int mip_level;
  public int padding;
};

public struct EeBloomUpsamplingPushConstant {
  public uint2 target_resolution;
  public float filter_radius;
  public float padding;
};

public bool EePixelInBounds(uint2 pixel, uint2 resolution) {
  return pixel.x < resolution.x && pixel.y < resolution.y;
}

public int2 EeTexel(uint2 pixel) {
  return int2(pixel);
}
