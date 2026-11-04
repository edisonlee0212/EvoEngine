import EcoSysLab.TreeRigidBodyMath;
import EcoSysLab.TreePhysicsData;
import EcoSysLab.TreePhysicsLodData;

struct Properties {
  float4 center_mass, inertia0, inertia1, inertia2;
  uint4 counts;
};
struct Motion {
  float4 rotation, angular_velocity, anchor, anchor_velocity, external_force, external_torque;
};
struct Rod { float4 start_radius, end_length, relative_rotation; uint4 indices; };
struct Vertex { float4 offset; uint4 indices; };
struct Joint { float4 parent_offset, rest_rotation; };
[[vk::binding(0,0)]] StructuredBuffer<Properties,Std430DataLayout> properties;
[[vk::binding(1,0)]] RWStructuredBuffer<Motion,Std430DataLayout> motion;
[[vk::binding(2,0)]] StructuredBuffer<Rod,Std430DataLayout> rods;
[[vk::binding(3,0)]] StructuredBuffer<Vertex,Std430DataLayout> vertices;
[[vk::binding(4,0)]] StructuredBuffer<TreePhysicsBuffers,Std430DataLayout> trees;
[[vk::binding(5,0)]] StructuredBuffer<Joint,Std430DataLayout> joints;
[[vk::push_constant]] cbuffer Parameters {
  float4 anchor, target_rotation, gravity, wind_density, stiffness_damping, step, base_stiffness;
  uint4 control;
};

void Loads(Properties p,Motion m,out float3 force,out float3 torque) {
  force=p.center_mass.w*gravity.xyz;
  torque=cross(LodRotate(m.rotation,p.center_mass.xyz),force);
    for (uint i=0;i<p.counts.x;++i) {
      Rod rod=rods[p.counts.z+i];
      float3 a=LodRotate(m.rotation,rod.start_radius.xyz),b=LodRotate(m.rotation,rod.end_length.xyz);
      float3 direction=normalize(b-a);
      float3 va=wind_density.xyz-m.anchor_velocity.xyz-cross(m.angular_velocity.xyz,a);
      float3 vb=wind_density.xyz-m.anchor_velocity.xyz-cross(m.angular_velocity.xyz,b);
      va-=dot(va,direction)*direction;vb-=dot(vb,direction)*direction;
      // Same endpoint drag convention as the full tree solver, with the actual member radius/length.
      float drag=0.6*step.y*(wind_density.w/1.225)*rod.start_radius.w*rod.end_length.w;
      float3 fa=drag*length(va)*va,fb=drag*length(vb)*vb;
      force+=fa+fb;torque+=cross(a,fa)+cross(b,fb);
    }
}
void Simulate() {
  Properties p=properties[0];Motion m=motion[0];
  float dt=step.x/float(control.x);
  float3 pivot_velocity=(anchor.xyz-m.anchor.xyz)/step.x;
  float3 pivot_acceleration=(pivot_velocity-m.anchor_velocity.xyz)/step.x;
  float3x3 inertia=float3x3(p.inertia0.xyz,p.inertia1.xyz,p.inertia2.xyz);
  float3 stiffness=stiffness_damping.xyz;
  float3x3 spring=float3x3(stiffness.x,0,0,0,stiffness.y,0,0,0,stiffness.z);
  for (uint substep=0;substep<control.x;++substep) {
    m.anchor_velocity=float4(pivot_velocity,0);
    float3 force,torque;Loads(p,m,force,torque);
    m.external_force=float4(force,0);m.external_torque=float4(torque,0);
    torque-=cross(LodRotate(m.rotation,p.center_mass.xyz),p.center_mass.w*pivot_acceleration);
    float3 omega=LodRotate(LodInverse(m.rotation),m.angular_velocity.xyz);
    float3 local_torque=LodRotate(LodInverse(m.rotation),torque);
    float3 error=RotationVector(LodMultiply(LodInverse(m.rotation),target_rotation));
    float3 momentum=mul(inertia,omega);
    // Implicit spring and angular damping; Euler's gyroscopic term retains the full inertia tensor.
    omega=Solve((1+dt*stiffness_damping.w)*inertia+dt*dt*spring,
                momentum+dt*(local_torque-cross(omega,momentum)+stiffness*error));
    m.angular_velocity=float4(LodRotate(m.rotation,omega),0);
    m.rotation=normalize(LodMultiply(m.rotation,Rotation(dt*omega)));
  }
  m.anchor=anchor;m.anchor_velocity=float4(pivot_velocity,0);
  motion[0]=m;
}
void SimulateCoupled() {
  Properties p0=properties[0],p1=properties[1];Motion a=motion[0],b=motion[1];Joint joint=joints[0];
  float dt=step.x/float(control.x);
  float3 pivot_velocity=(anchor.xyz-a.anchor.xyz)/step.x;
  float3 pivot_acceleration=(pivot_velocity-a.anchor_velocity.xyz)/step.x;
  float3x3 local0=float3x3(p0.inertia0.xyz,p0.inertia1.xyz,p0.inertia2.xyz);
  float3x3 local1=float3x3(p1.inertia0.xyz,p1.inertia1.xyz,p1.inertia2.xyz);
  for (uint substep=0;substep<control.x;++substep) {
    float3 w0=a.angular_velocity.xyz,w1=b.angular_velocity.xyz;
    float3 j=LodRotate(a.rotation,joint.parent_offset.xyz);
    float3 r0=LodRotate(a.rotation,p0.center_mass.xyz),r1=LodRotate(b.rotation,p1.center_mass.xyz);
    a.anchor_velocity=float4(pivot_velocity,0);
    b.anchor_velocity=float4(pivot_velocity+cross(w0,j),0);
    float3 f0,t0,f1,t1;Loads(p0,a,f0,t0);Loads(p1,b,f1,t1);
    a.external_force=float4(f0,0);a.external_torque=float4(t0,0);
    b.external_force=float4(f1,0);b.external_torque=float4(t1,0);
    float3x3 i0=WorldMatrix(a.rotation,local0),i1=WorldMatrix(b.rotation,local1);
    float3x3 cj=CrossMatrix(j),cr=CrossMatrix(r1);
    // Absolute parent/child rotations share translational inertia through the joint.
    float3x3 m00=i0-p1.center_mass.w*mul(cj,cj);
    float3x3 m01=-p1.center_mass.w*mul(cj,cr);
    float3x3 m11=i1;
    float3 centrifugal0=cross(w0,cross(w0,j)),centrifugal1=cross(w1,cross(w1,r1));
    float3 bias0=cross(w0,mul(i0,w0))+p1.center_mass.w*cross(j,centrifugal0+centrifugal1)+
                 cross(p0.center_mass.w*r0+p1.center_mass.w*j,pivot_acceleration);
    float3 bias1=cross(w1,mul(i1,w1))+p1.center_mass.w*cross(r1,centrifugal0+pivot_acceleration);
    float4 target=LodMultiply(a.rotation,joint.rest_rotation);
    float3x3 k=Stiffness(target,stiffness_damping.xyz),k0=Stiffness(target_rotation,base_stiffness.xyz);
    float3 spring=mul(k,RotationVector(LodMultiply(target,LodInverse(b.rotation))));
    float3 base_spring=mul(k0,RotationVector(LodMultiply(target_rotation,LodInverse(a.rotation))));
    float3 rhs0=mul(m00,w0)+mul(m01,w1)+dt*(t0+cross(j,f1)-bias0+base_spring-spring);
    float3 rhs1=mul(transpose(m01),w0)+mul(m11,w1)+dt*(t1-bias1+spring);
    float damping=1+dt*stiffness_damping.w;
    float3x3 aa=damping*m00+dt*dt*(k+k0),bb=damping*m01-dt*dt*k,dd=damping*m11+dt*dt*k;
    // Schur complement of the symmetric 6x6 articulated mass/spring system.
    float3x3 di=Inverse(dd),bd=mul(bb,di);
    w0=Solve(aa-mul(bd,transpose(bb)),rhs0-mul(bd,rhs1));
    w1=mul(di,rhs1-mul(transpose(bb),w0));
    a.angular_velocity=float4(w0,0);b.angular_velocity=float4(w1,0);
    a.rotation=normalize(LodMultiply(Rotation(dt*w0),a.rotation));
    b.rotation=normalize(LodMultiply(Rotation(dt*w1),b.rotation));
  }
  a.anchor=anchor;a.anchor_velocity=float4(pivot_velocity,0);
  float3 j=LodRotate(a.rotation,joint.parent_offset.xyz);
  b.anchor=float4(anchor.xyz+j,0);
  b.anchor_velocity=float4(pivot_velocity+cross(a.angular_velocity.xyz,j),0);
  motion[0]=a;motion[1]=b;
}
void Reconstruct(uint i) {
  TreePhysicsBuffers full=trees[0];
  if (i<control.w) {
    Vertex map=vertices[i];Motion m=motion[map.indices.w];
    TreePhysicsVertex vertex=PhysicsVertex(full,0,map.indices.x);
    float3 arm=LodRotate(m.rotation,map.offset.xyz);
    vertex.position_inverse_mass.xyz=m.anchor.xyz+arm;
    vertex.velocity_radius.xyz=m.anchor_velocity.xyz+cross(m.angular_velocity.xyz,arm);
    LodStoreVertex(full,0,map.indices.x,vertex,false);LodStoreVertex(full,1,map.indices.x,vertex,false);
  }
  if (i<control.z) {
    Rod map=rods[i];Motion m=motion[map.indices.w];
    TreePhysicsRod rod=PhysicsRod(full,0,map.indices.x);
    rod.rotation=normalize(LodMultiply(m.rotation,map.relative_rotation));
    LodStoreRod(full,0,map.indices.x,rod,false);LodStoreRod(full,1,map.indices.x,rod,false);
  }
}
[numthreads(64,1,1)]
void main(uint i:SV_DispatchThreadID) {
#if SUBTREE_PASS == 0
  if (i==0) {
    if (control.y==2) SimulateCoupled();
    else Simulate();
  }
#else
  Reconstruct(i);
#endif
}
