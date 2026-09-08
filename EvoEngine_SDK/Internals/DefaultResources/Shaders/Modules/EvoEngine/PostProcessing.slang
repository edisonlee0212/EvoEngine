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

public float3 EeLimitBloomSource(float3 color, float compressionStart, float sourceCeiling) {
  color = clamp(color, float3(0.0f), float3(1e38f));
  float ceiling = max(sourceCeiling, 0.0f);
  float start = clamp(compressionStart, 0.0f, ceiling);
  float brightness = max(color.x, max(color.y, color.z));
  if (brightness <= start)
    return color;
  float range = ceiling - start;
  float excess = brightness - start;
  // Normalize before multiplying to avoid overflowing HDR inputs or underflowing a tiny RGB scale.
  float limited = start + range * (excess / (range + excess));
  return (color / brightness) * limited;
}

public float3 EePrefilterBloom(float3 color, float threshold, float knee, float compressionStart, float sourceCeiling) {
  // HDR target overflow must not introduce Inf/Inf into bloom extraction.
  color = clamp(color, float3(0.0f), float3(1e38f));
  float brightness = max(color.x, max(color.y, color.z));
  knee = max(knee, 0.0001f);
  float soft = clamp(brightness - (threshold - knee), 0.0f, 2.0f * knee);
  soft = soft * soft * (0.25f / knee);
  color *= max(soft, brightness - threshold) / max(brightness, 0.0001f);
  return EeLimitBloomSource(color, compressionStart, sourceCeiling);
}
