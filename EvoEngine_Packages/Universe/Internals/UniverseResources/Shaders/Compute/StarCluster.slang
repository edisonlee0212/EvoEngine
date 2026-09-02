struct StarBaseSample
{
  double orbitalProportion;
  double gaussianX;
  double gaussianY;
  double gaussianZ;
};

struct StarClusterParameters
{
  uint64_t populationRevision;
  uint starCount;
  uint padding0;
  double4 ellipse0;
  double4 ellipse1;
  double4 spreadSpeed;
  double4 speedTilt;
  double4 tiltRadius;
  double4 centerOffset;
  double4 centerPosition;
  double4 world0;
  double4 world1;
  double4 world2;
  double4 world3;
  float4 diskColorIntensity;
  float4 coreColorIntensity;
  float4 centerColorIntensity;
  double4 timePadding;
};

struct StarClusterResult
{
  double4 worldPositionRadius;
  float4 colorEmission;
  float4 alphaPadding;
};

[[vk::binding(0, 0)]] StructuredBuffer<StarClusterParameters> parameters;
[[vk::binding(1, 0)]] StructuredBuffer<StarBaseSample> baseSamples;
[[vk::binding(2, 0)]] RWStructuredBuffer<StarClusterResult> results;

struct DispatchParameters
{
  uint parameterIndex;
  uint starOffset;
};
[[vk::push_constant]] ConstantBuffer<DispatchParameters> dispatchParameters;

void sinCos64(double angle, out double sine, out double cosine)
{
  const double pi = 3.1415926535897932384626433832795L;
  const double halfPi = 1.5707963267948966192313216916398L;
  const double twoPi = 6.283185307179586476925286766559L;
  angle -= double(int(angle / twoPi)) * twoPi;
  if (angle > pi)
    angle -= twoPi;
  else if (angle < -pi)
    angle += twoPi;

  double cosineSign = 1.0;
  if (angle > halfPi)
  {
    angle = pi - angle;
    cosineSign = -1.0;
  }
  else if (angle < -halfPi)
  {
    angle = -pi - angle;
    cosineSign = -1.0;
  }

  const double x2 = angle * angle;
  sine = angle * (1.0 + x2 * (-0.16666666666666666667L + x2 * (0.00833333333333333333L +
         x2 * (-0.00019841269841269841L + x2 * (0.00000275573192239859L +
         x2 * (-0.00000002505210838544L + x2 * 0.00000000016059043837L))))));
  cosine = cosineSign * (1.0 + x2 * (-0.5 + x2 * (0.04166666666666666667L +
           x2 * (-0.00138888888888888889L + x2 * (0.00002480158730158730L +
           x2 * (-0.00000027557319223986L + x2 * (0.00000000208767569879L +
           x2 * -0.00000000001147074560L)))))));
}

double sqrt64(double value)
{
  double estimate = double(sqrt(float(value)));
  estimate = 0.5 * (estimate + value / estimate);
  return 0.5 * (estimate + value / estimate);
}

double3 rotateX(double3 point, double degrees)
{
  double s;
  double c;
  sinCos64(degrees * 0.01745329251994329576923690768489L, s, c);
  return double3(point.x, c * point.y - s * point.z, s * point.y + c * point.z);
}

double3 rotateY(double3 point, double degrees)
{
  double s;
  double c;
  sinCos64(degrees * 0.01745329251994329576923690768489L, s, c);
  return double3(c * point.x + s * point.z, point.y, -s * point.x + c * point.z);
}

double3 rotateZ(double3 point, double degrees)
{
  double s;
  double c;
  sinCos64(degrees * 0.01745329251994329576923690768489L, s, c);
  return double3(c * point.x - s * point.y, s * point.x + c * point.y, point.z);
}

[shader("compute")]
[numthreads(256, 1, 1)]
void main(uint3 dispatchThreadId : SV_DispatchThreadID)
{
  const StarClusterParameters p = parameters[dispatchParameters.parameterIndex];
  const uint index = dispatchThreadId.x;
  if (index >= p.starCount)
    return;

  const StarBaseSample sample = baseSamples[dispatchParameters.starOffset + index];
  const double proportion = sample.orbitalProportion;
  double a;
  double b;
  double tiltX;
  double tiltZ;
  double speed;
  float4 colorEmission;
  if (proportion > p.ellipse1.z)
  {
    const double t = (proportion - p.ellipse1.z) / (1.0 - p.ellipse1.z);
    a = lerp(p.ellipse0.z, p.ellipse0.x, t);
    b = lerp(p.ellipse0.w, p.ellipse0.y, t);
    tiltX = lerp(p.speedTilt.w, p.speedTilt.y, t);
    tiltZ = lerp(p.tiltRadius.x, p.speedTilt.z, t);
    speed = lerp(p.spreadSpeed.w, p.spreadSpeed.z, t);
    colorEmission = lerp(p.coreColorIntensity, p.diskColorIntensity, float(t));
  }
  else
  {
    const double t = proportion / p.ellipse1.z;
    a = lerp(p.ellipse1.x, p.ellipse0.z, t);
    b = lerp(p.ellipse1.y, p.ellipse0.w, t);
    tiltX = lerp(p.tiltRadius.y, p.speedTilt.w, t);
    tiltZ = lerp(p.tiltRadius.z, p.tiltRadius.x, t);
    speed = lerp(p.speedTilt.x, p.spreadSpeed.w, t);
    colorEmission = lerp(p.centerColorIntensity, p.coreColorIntensity, float(t));
  }

  const double angle = (proportion * 360.0 + p.timePadding.x) / sqrt64(a + b) * speed;
  double orbitalSin;
  double orbitalCos;
  sinCos64(angle, orbitalSin, orbitalCos);
  double3 point = double3(orbitalSin * a, 0.0, orbitalCos * b);
  point = rotateX(point, tiltX);
  point = rotateY(point, -p.ellipse1.w * proportion);
  point = rotateZ(point, tiltZ);
  point += p.centerOffset.xyz * (1.0 - proportion) + p.centerPosition.xyz;
  point += double3(sample.gaussianX * (p.ellipse0.x + p.ellipse0.y) * p.spreadSpeed.y,
                   sample.gaussianY * (p.ellipse0.x + p.ellipse0.y) * p.spreadSpeed.x,
                   sample.gaussianZ * (p.ellipse0.x + p.ellipse0.y) * p.spreadSpeed.y);
  point /= 20.0;
  const double4 local = double4(point, 1.0);
  const double4 world = p.world0 * local.x + p.world1 * local.y + p.world2 * local.z + p.world3 * local.w;

  StarClusterResult result;
  result.worldPositionRadius = double4(world.xyz, p.tiltRadius.w);
  result.colorEmission = colorEmission;
  result.alphaPadding = float4(float(p.timePadding.y), 0.0, 0.0, 0.0);
  results[dispatchParameters.starOffset + index] = result;
}
