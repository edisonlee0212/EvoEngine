import EcoSysLab.TreePhysicsLodData;

public float3 RotationVector(float4 q) {
  if (q.w<0) q=-q;
  float sine=length(q.xyz);
  return sine<1e-7 ? 2*q.xyz : (2*atan2(sine,q.w)/sine)*q.xyz;
}
public float4 Rotation(float3 angular) {
  float angle=length(angular);
  return angle<1e-7 ? normalize(float4(0.5*angular,1)) : float4(sin(0.5*angle)*angular/angle,cos(0.5*angle));
}
public float3 Solve(float3x3 matrix,float3 rhs) {
  // SPD inertia plus implicit spring/damping terms; normalize for small twig inertias.
  float scale=max(matrix[0][0],max(matrix[1][1],matrix[2][2]));
  matrix/=scale;rhs/=scale;
  float3 a=matrix[0],b=matrix[1],c=matrix[2];
  return (rhs.x*cross(b,c)+rhs.y*cross(c,a)+rhs.z*cross(a,b))/dot(a,cross(b,c));
}
public float3x3 Inverse(float3x3 a) {
  return transpose(float3x3(Solve(a,float3(1,0,0)),Solve(a,float3(0,1,0)),Solve(a,float3(0,0,1))));
}
public float3x3 CrossMatrix(float3 v) { return float3x3(0,-v.z,v.y,v.z,0,-v.x,-v.y,v.x,0); }
public float3x3 WorldMatrix(float4 q,float3x3 local) {
  float3x3 r=transpose(float3x3(LodRotate(q,float3(1,0,0)),LodRotate(q,float3(0,1,0)),LodRotate(q,float3(0,0,1))));
  return mul(r,mul(local,transpose(r)));
}
public float3x3 Stiffness(float4 q,float3 k) {
  return WorldMatrix(q,float3x3(k.x,0,0,0,k.y,0,0,0,k.z));
}
