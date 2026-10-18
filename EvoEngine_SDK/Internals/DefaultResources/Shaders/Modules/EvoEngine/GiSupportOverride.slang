// Per-invocation input; receiver matching keeps backface gathers independent.
public
static bool EE_GI_SUPPORT_VALID = false;
public
static float EE_GI_SUPPORT_WEIGHT = 0;
public
static float3 EE_GI_SUPPORT_POSITION = float3(0);
public
static float3 EE_GI_SUPPORT_NORMAL = float3(0);
public
static float3 EE_GI_SUPPORT_IRRADIANCE = float3(0);
public
bool EE_GI_SUPPORT_MATCH(float3 position, float3 normal) {
  return EE_GI_SUPPORT_VALID && dot(position - EE_GI_SUPPORT_POSITION, position - EE_GI_SUPPORT_POSITION) < 1e-8 &&
         dot(normal, EE_GI_SUPPORT_NORMAL) > 0.999;
}
public
void EE_GI_SUPPORT_SET(float3 position, float3 normal, float3 irradiance, bool valid, float weight) {
  EE_GI_SUPPORT_WEIGHT = weight;
  EE_GI_SUPPORT_VALID = valid;
  EE_GI_SUPPORT_POSITION = position;
  EE_GI_SUPPORT_NORMAL = normal;
  EE_GI_SUPPORT_IRRADIANCE = irradiance;
}

public
float EE_GI_SUPPORT_REFINEMENT_WEIGHT(float visibility) {
  return 1 - smoothstep(0.05, 0.25, visibility);
}
