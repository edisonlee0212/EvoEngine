public struct EeToneMappingConstants {
  public int camera_index;
  public int method;
  public int is_active;
  public int auto_exposure;
  public int enable_center_metering;
  public int average_mode;
  public int dither;
  public float exposure;
  public float brightness;
  public float contrast;
  public float saturation;
  public float vignette;
  public float auto_exposure_speed;
  public float ev_min_value;
  public float ev_max_value;
  public float center_metering_size;
};

[[vk::push_constant]]
public ConstantBuffer<EeToneMappingConstants> EE_TONE_MAPPING_CONSTANTS;

public static const uint EE_EXPOSURE_HISTOGRAM_SIZE = 256u;
