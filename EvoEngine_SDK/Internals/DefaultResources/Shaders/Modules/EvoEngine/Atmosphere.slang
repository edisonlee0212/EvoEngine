// @evoengine-dialect native
public struct Atmosphere {
  public float earth_radius;       // In the paper this is usually Rg or Re (radius ground, eart)
  public float atmosphere_radius;  // In the paper this is usually R or Ra (radius atmosphere)
  public float hr;                 // Thickness of the atmosphere if density was uniform (Hr) for Rayleigh scattering
  public float hm;                 // Same as above but for Mie scattering (Hm)

  public float g;  // Mean cosine for Mie scattering
  public int num_samples;
  public int num_samples_light;
  public float intensity;
};

public bool SolveQuadratic(in float a, in float b, in float c, inout float x1, inout float x2) {
  if (b == 0) {
    // Handle special case where the the two vector ray.dir and V are perpendicular
    // with V = ray.rayOrigin - sphere.centre
    if (a == 0)
      return false;
    x1 = 0;
    x2 = sqrt(-c / a);
    return true;
  }

  float discr = b * b - 4 * a * c;

  if (discr < 0)
    return false;

  float q = (b < 0.f) ? -0.5f * (b - sqrt(discr)) : -0.5f * (b + sqrt(discr));
  x1 = q / a;
  x2 = c / q;

  return true;
}

public bool RaySphereIntersect(in float3 rayOrigin, in float3 dir, in float radius, inout float t0, inout float t1) {
  // They ray dir is normalized so A = 1
  float A = dir.x * dir.x + dir.y * dir.y + dir.z * dir.z;
  float B = 2.0f * (dir.x * rayOrigin.x + dir.y * rayOrigin.y + dir.z * rayOrigin.z);
  float C = rayOrigin.x * rayOrigin.x + rayOrigin.y * rayOrigin.y + rayOrigin.z * rayOrigin.z - radius * radius;

  if (!SolveQuadratic(A, B, C, t0, t1))
    return false;

  if (t0 > t1) {
    float temp = t0;
    t0 = t1;
    t1 = temp;
  }

  return true;
}

public float3 NishitaSkyIncidentLight(in Atmosphere atmosphere, in float3 camera_position, in float3 rayDir, in float3 sun_direction) {
  float earthRadius = atmosphere.earth_radius * 1000.0f;  // In the paper this is usually Rg or Re (radius ground, eart)
  float atmosphereRadius =
      atmosphere.atmosphere_radius * 1000.0f;  // In the paper this is usually R or Ra (radius atmosphere)
  float Hr = atmosphere.hr;                    // Thickness of the atmosphere if density was uniform (Hr)
  float Hm = atmosphere.hm;                    // Same as above but for Mie scattering (Hm)

  float3 betaR = float3(3.8e-6f, 13.5e-6f, 33.1e-6f);
  float3 betaM = float3(21e-6f);
  float tmin = 0;
  float tmax = 1e16f;
  float3 rayOrigin = camera_position + float3(0.0f, earthRadius, 0.0f);
  float t0, t1;

  float3 actual_ray_dir = rayDir;
  actual_ray_dir.y = abs(actual_ray_dir.y);

  if (!RaySphereIntersect(rayOrigin, actual_ray_dir, atmosphereRadius, t0, t1) || t1 < 0.0f)
    return float3(0.0f, 0.0f, 0.0f);
  if (t0 > tmin && t0 > 0.0f)
    tmin = t0;
  if (t1 < tmax)
    tmax = t1;
  uint numSamples = atmosphere.num_samples;
  uint numSamplesLight = atmosphere.num_samples_light;
  float segmentLength = (tmax - tmin) / numSamples;
  float tCurrent = tmin;
  float3 sumR = float3(0.0f, 0.0f, 0.0f);
  float3 sumM = float3(0.0f, 0.0f, 0.0f);  // mie and rayleigh contribution
  float opticalDepthR = 0, opticalDepthM = 0;
  float mu = dot(actual_ray_dir, sun_direction);  // mu in the paper which is the cosine of the angle between the sun
                                                  // direction and the ray direction
  float phaseR = 3.f / (16.f * 3.1415926f) * (1.0f + mu * mu);
  float g = atmosphere.g;
  float phaseM = 3.f / (8.f * 3.1415926f) * ((1.f - g * g) * (1.f + mu * mu)) /
                 ((2.f + g * g) * pow(1.f + g * g - 2.f * g * mu, 1.5f));
  for (uint i = 0; i < numSamples; ++i) {
    float3 samplePosition = rayOrigin + (tCurrent + segmentLength * 0.5f) * actual_ray_dir;
    float height = length(samplePosition) - earthRadius;
    // compute optical depth for light
    float hr = exp(-height / Hr) * segmentLength;
    float hm = exp(-height / Hm) * segmentLength;
    opticalDepthR += hr;
    opticalDepthM += hm;
    // light optical depth
    float t0Light, t1Light;
    RaySphereIntersect(samplePosition, sun_direction, atmosphereRadius, t0Light, t1Light);
    float segmentLengthLight = t1Light / numSamplesLight, tCurrentLight = 0;
    float opticalDepthLightR = 0, opticalDepthLightM = 0;
    uint j;
    for (j = 0; j < numSamplesLight; ++j) {
      float3 samplePositionLight = samplePosition + (tCurrentLight + segmentLengthLight * 0.5f) * sun_direction;
      float heightLight = length(samplePositionLight) - earthRadius;
      if (heightLight < 0)
        break;
      opticalDepthLightR += exp(-heightLight / Hr) * segmentLengthLight;
      opticalDepthLightM += exp(-heightLight / Hm) * segmentLengthLight;
      tCurrentLight += segmentLengthLight;
    }
    if (j == numSamplesLight) {
      float3 tau = betaR * (opticalDepthR + opticalDepthLightR) + betaM * 1.1f * (opticalDepthM + opticalDepthLightM);
      float3 attenuation = float3(exp(-tau.x), exp(-tau.y), exp(-tau.z));
      sumR += attenuation * hr;
      sumM += attenuation * hm;
    }
    tCurrent += segmentLength;
  }
  // We use a magic number here for the intensity of the sun (20). We will make it more
  // scientific in a future revision of this lesson/code
  float3 result = (float3(sumR.x * betaR.x, sumR.y * betaR.y, sumR.z * betaR.z) * phaseR +
                 float3(sumM.x * betaM.x, sumM.y * betaM.y, sumM.z * betaM.z) * phaseM) *
                20.0f;

  return result * atmosphere.intensity;
}
