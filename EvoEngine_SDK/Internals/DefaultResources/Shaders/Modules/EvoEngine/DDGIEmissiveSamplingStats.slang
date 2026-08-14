// @evoengine-dialect native
__exported import EvoEngine.DDGIProbeRayConstants;

public
static const uint EE_DDGI_EMISSIVE_STAT_NEE_ATTEMPT = 0u;
public
static const uint EE_DDGI_EMISSIVE_STAT_ZERO_PDF_REJECT = 1u;
public
static const uint EE_DDGI_EMISSIVE_STAT_EMITTER_BACKFACE_REJECT = 2u;
public
static const uint EE_DDGI_EMISSIVE_STAT_ALPHA_MASK_REJECT = 3u;
public
static const uint EE_DDGI_EMISSIVE_STAT_INVALID_SAMPLE_REJECT = 4u;
public
static const uint EE_DDGI_EMISSIVE_STAT_RECEIVER_BACKFACE_REJECT = 5u;
public
static const uint EE_DDGI_EMISSIVE_STAT_SHADOWED_SAMPLE = 6u;
public
static const uint EE_DDGI_EMISSIVE_STAT_ZERO_RADIANCE_SAMPLE = 7u;
public
static const uint EE_DDGI_EMISSIVE_STAT_NONZERO_CONTRIBUTION = 8u;

[[vk::binding(19, 2)]] public
RWStructuredBuffer<uint, Std430DataLayout> EE_DDGI_EMISSIVE_SAMPLING_STATS;

public
bool EE_DDGI_EMISSIVE_SAMPLING_STATS_ENABLED() {
  return (EE_DDGI_PROBE_RAY_CONSTANTS.selected_probe_volume_flags_environment.z & (1u << 2u)) != 0u;
}

public
void EE_DDGI_COUNT_EMISSIVE_SAMPLE(const uint counter_index) {
  if (EE_DDGI_EMISSIVE_SAMPLING_STATS_ENABLED()) {
    uint previous_value;
    InterlockedAdd(EE_DDGI_EMISSIVE_SAMPLING_STATS[counter_index], 1u, previous_value);
  }
}
