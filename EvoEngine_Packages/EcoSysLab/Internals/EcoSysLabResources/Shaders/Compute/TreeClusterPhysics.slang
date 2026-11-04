import EcoSysLab.TreePhysicsData;
import EcoSysLab.TreePhysicsLodData;
import EcoSysLab.TreeRigidBodyMath;
import EcoSysLab.AirGrid;
[[vk::binding(0,1)]] StructuredBuffer<uint> wind_heads;
[[vk::binding(1,1)]] StructuredBuffer<AirGridAddresses> wind_addresses;

struct Body {
  float4 center_mass,inertia0,inertia1,inertia2;
  float4 rest_anchor,rest_rotation,parent_offset,relative_rotation;
  uint4 links;
};
struct Motion { float4 rotation,angular_velocity,anchor,anchor_velocity,external_force,external_torque; };
struct Rod { float4 start_radius,end_length,relative_rotation;uint4 indices; };
struct Vertex { float4 offset;uint4 indices; };
struct Work {
  float4 a0,a1,a2,b0,b1,b2,d0,d1,d2;
  float4 hv,hw,bias,next_anchor,next_rotation;
};
struct Addresses { uint64_t values[18]; };
struct LodInput { uint4 control; float4 rules[3];float4 material; };
struct LodState { uint4 status;LodInput settings; };
struct Candidate { uint4 links;float4 bounds; };
struct Parameters {
  float4 model_rotation;
  float3 model_translation;uint wind_bucket_count;
  float3 gravity;uint wind_cell_count;
  float4 wind_density,stiffness_damping;
  float2 mechanics; uint level_begin,substep_index;
  float3 base_stiffness;float wind_cell_size;
  uint4 counts;
};
#if !CLUSTER_FOREST
[[vk::binding(0,0)]] StructuredBuffer<Body,Std430DataLayout> dataBodies;
[[vk::binding(1,0)]] RWStructuredBuffer<Motion,Std430DataLayout> dataMotions;
[[vk::binding(2,0)]] StructuredBuffer<Rod,Std430DataLayout> dataRods;
[[vk::binding(3,0)]] StructuredBuffer<Vertex,Std430DataLayout> dataVertices;
[[vk::binding(4,0)]] StructuredBuffer<TreePhysicsBuffers,Std430DataLayout> dataTrees;
[[vk::binding(5,0)]] RWStructuredBuffer<Work,Std430DataLayout> dataWork;
[[vk::binding(6,0)]] StructuredBuffer<uint4,Std430DataLayout> dataSchedule;
[[vk::binding(7,0)]] RWStructuredBuffer<Work,Std430DataLayout> dataContributions;
#endif
struct Solver {
  Addresses buffers;
  Parameters parameters;
  LodInput lod;
  Body LoadBodies(uint i) {
#if CLUSTER_FOREST
    return PhysicsLoad<Body>(buffers.values[0],i);
#else
    return dataBodies[i];
#endif
  }
  Motion LoadMotions(uint i) {
#if CLUSTER_FOREST
    return PhysicsLoad<Motion>(buffers.values[1],i);
#else
    return dataMotions[i];
#endif
  }
  void StoreMotions(uint i,Motion value) {
#if CLUSTER_FOREST
    PhysicsStore<Motion>(buffers.values[1],i,value);
#else
    dataMotions[i]=value;
#endif
  }
  Rod LoadRods(uint i) {
#if CLUSTER_FOREST
    return PhysicsLoad<Rod>(buffers.values[2],i);
#else
    return dataRods[i];
#endif
  }
  Vertex LoadVertices(uint i) {
#if CLUSTER_FOREST
    return PhysicsLoad<Vertex>(buffers.values[3],i);
#else
    return dataVertices[i];
#endif
  }
  TreePhysicsBuffers LoadTrees(uint i) {
#if CLUSTER_FOREST
    return PhysicsLoad<TreePhysicsBuffers>(buffers.values[4],i);
#else
    return dataTrees[i];
#endif
  }
  Work LoadWork(uint i) {
#if CLUSTER_FOREST
    return PhysicsLoad<Work>(buffers.values[5],i);
#else
    return dataWork[i];
#endif
  }
  void StoreWork(uint i,Work value) {
#if CLUSTER_FOREST
    PhysicsStore<Work>(buffers.values[5],i,value);
#else
    dataWork[i]=value;
#endif
  }
  uint4 LoadSchedule(uint i) {
#if CLUSTER_FOREST
    return PhysicsLoad<uint4>(buffers.values[6],i);
#else
    return dataSchedule[i];
#endif
  }
  Work LoadContributions(uint i) {
#if CLUSTER_FOREST
    return PhysicsLoad<Work>(buffers.values[7],i);
#else
    return dataContributions[i];
#endif
  }
  void StoreContributions(uint i,Work value) {
#if CLUSTER_FOREST
    PhysicsStore<Work>(buffers.values[7],i,value);
#else
    dataContributions[i]=value;
#endif
  }
  float3x3 A(Work w) { return float3x3(w.a0.xyz,w.a1.xyz,w.a2.xyz); }
  float3x3 B(Work w) { return float3x3(w.b0.xyz,w.b1.xyz,w.b2.xyz); }
  float3x3 D(Work w) { return float3x3(w.d0.xyz,w.d1.xyz,w.d2.xyz); }
  void StoreMatrix(out float4 a,out float4 b,out float4 c,float3x3 value) {
    a=float4(value[0],0);b=float4(value[1],0);c=float4(value[2],0);
  }
  void RodLoads(Rod rod,Motion m,out float3 force,out float3 torque) {
    float3 a=LodRotate(m.rotation,rod.start_radius.xyz),b=LodRotate(m.rotation,rod.end_length.xyz);
    float3 direction=normalize(b-a);
    float4 air=float4(parameters.wind_density.xyz,parameters.wind_density.w/1.225);
    if(parameters.wind_cell_count!=0)
      air=AirGridSampleWind(wind_heads,wind_addresses,parameters.wind_bucket_count,parameters.wind_cell_size,m.anchor.xyz+(a+b)*0.5);
    float3 va=air.xyz-m.anchor_velocity.xyz-cross(m.angular_velocity.xyz,a);
    float3 vb=air.xyz-m.anchor_velocity.xyz-cross(m.angular_velocity.xyz,b);
    va-=dot(va,direction)*direction;vb-=dot(vb,direction)*direction;
    float drag=0.6*parameters.mechanics.y*air.w*rod.start_radius.w*rod.end_length.w;
    float3 fa=drag*length(va)*va,fb=drag*length(vb)*vb;
#if CLUSTER_FOREST
    if(buffers.values[8]!=0) {
      float cap=25*lod.material.x*PhysicsRod(LoadTrees(0),0,rod.indices.x).length_scale_mass_radius.z;
      fa*=min(1.0,cap/max(length(fa),1e-12));fb*=min(1.0,cap/max(length(fb),1e-12));
    }
#endif
    force=fa+fb;torque=cross(a,fa)+cross(b,fb);
  }
  void Loads(Body p,Motion m,out float3 force,out float3 torque) {
    force=p.center_mass.w*lod.material.x*parameters.gravity.xyz;
    torque=cross(LodRotate(m.rotation,p.center_mass.xyz),force);
    for (uint i=0;i<p.links.y;++i) {
      uint rod=p.links.x+i;
      float3 f,t;
#if CLUSTER_FOREST
      f=PhysicsLoad<float4>(buffers.values[17],2*rod).xyz;
      t=PhysicsLoad<float4>(buffers.values[17],2*rod+1).xyz;
#else
      RodLoads(LoadRods(rod),m,f,t);
#endif
      force+=f;torque+=t;
    }
  }
  float3x3 JointSpring(Body body,Motion parent) {
    float3 k=LoadBodies(body.links.z).links.w!=0 ? parameters.base_stiffness.xyz : parameters.stiffness_damping.xyz;
#if CLUSTER_FOREST
    if(buffers.values[8]!=0) k*=body.rest_anchor.w;
#endif
    return Stiffness(LodMultiply(parent.rotation,body.relative_rotation),k);
  }
  void Prepare(uint i,uint substep,float dt) {
    Body p=LoadBodies(i);Motion m=LoadMotions(i);Work w=(Work)0;
    if (p.links.w!=0) {
      float3 target=parameters.model_translation.xyz+LodRotate(parameters.model_rotation,p.rest_anchor.xyz);
      float4 q=LodMultiply(parameters.model_rotation,p.rest_rotation);
      float fraction=1.0/float(parameters.counts.w-substep);
      w.next_anchor=float4(lerp(m.anchor.xyz,target,fraction),0);
      w.next_rotation=normalize(LodMultiply(Rotation(fraction*RotationVector(LodMultiply(q,LodInverse(m.rotation)))),m.rotation));
      StoreWork(i,w);return;
    }
    Motion parent=LoadMotions(p.links.z);
    float3 j=LodRotate(parent.rotation,p.parent_offset.xyz);
    float3 omega=m.angular_velocity.xyz,v=m.anchor_velocity.xyz;
    w.bias=float4(LoadWork(p.links.z).bias.xyz+cross(parent.angular_velocity.xyz,cross(parent.angular_velocity.xyz,j)),0);
    float mass=p.center_mass.w*lod.material.x;
    float3 r=LodRotate(m.rotation,p.center_mass.xyz);
    float3x3 aa=float3x3(mass,0,0,0,mass,0,0,0,mass),bb=-mass*CrossMatrix(r);
    float3x3 dd=lod.material.x*WorldMatrix(m.rotation,float3x3(p.inertia0.xyz,p.inertia1.xyz,p.inertia2.xyz));
    float3 force,torque;Loads(p,m,force,torque);
    m.external_force=float4(force,0);m.external_torque=float4(torque,0);StoreMotions(i,m);
    w.hv=float4(mul(aa,v)+mul(bb,omega)+dt*(force-mass*(w.bias.xyz+cross(omega,cross(omega,r)))),0);
    w.hw=float4(mul(transpose(bb),v)+mul(dd,omega)+dt*(torque-cross(omega,mul(dd,omega))-mass*cross(r,w.bias.xyz)),0);
    float damping=1+dt*parameters.stiffness_damping.w;
    StoreMatrix(w.a0,w.a1,w.a2,damping*aa);
    StoreMatrix(w.b0,w.b1,w.b2,damping*bb);
    StoreMatrix(w.d0,w.d1,w.d2,damping*dd);
    StoreWork(i,w);
  }
  void Eliminate(uint i,float dt) {
    Body p=LoadBodies(i);if (p.links.w!=0) return;
    Work w=LoadWork(i);
#if CLUSTER_PASS == 3 || CLUSTER_PASS == 10
    // Each body owns its reduction; siblings never write their shared parent.
    for (uint child=LoadSchedule(i).y;child!=0xffffffffu;child=LoadSchedule(child).z) {
      Work c=LoadContributions(child);
      StoreMatrix(w.a0,w.a1,w.a2,A(w)+A(c));
      StoreMatrix(w.b0,w.b1,w.b2,B(w)+B(c));
      StoreMatrix(w.d0,w.d1,w.d2,D(w)+D(c));
      w.hv+=c.hv;w.hw+=c.hw;
    }
#endif
    Motion parent=LoadMotions(p.links.z);
    float3x3 k=JointSpring(p,parent),s=dt*dt*k;
    float4 target=LodMultiply(parent.rotation,p.relative_rotation);
    float3 spring=dt*mul(k,RotationVector(LodMultiply(target,LodInverse(LoadMotions(i).rotation))));
    w.hw.xyz+=spring;
    float3x3 di=Inverse(D(w)+s),bb=B(w),bd=mul(bb,di);
    StoreMatrix(w.d0,w.d1,w.d2,di);StoreWork(i,w);
    if (LoadBodies(p.links.z).links.w!=0) return;
    float3x3 ga=A(w)-mul(bd,transpose(bb)),gb=mul(bd,s),gd=s-mul(s,mul(di,s));
    float3 gv=w.hv.xyz-mul(bd,w.hw.xyz),gw=mul(s,mul(di,w.hw.xyz))-spring;
    float3x3 offset=-CrossMatrix(LodRotate(parent.rotation,p.parent_offset.xyz));
#if CLUSTER_PASS == 3 || CLUSTER_PASS == 10
    Work result=(Work)0;
#else
    Work result=LoadWork(p.links.z);
#endif
    StoreMatrix(result.a0,result.a1,result.a2,A(result)+ga);
    StoreMatrix(result.b0,result.b1,result.b2,B(result)+mul(ga,offset)+gb);
    StoreMatrix(result.d0,result.d1,result.d2,D(result)+mul(transpose(offset),mul(ga,offset))+
        mul(transpose(offset),gb)+mul(transpose(gb),offset)+gd);
    result.hv.xyz+=gv;result.hw.xyz+=mul(transpose(offset),gv)+gw;
#if CLUSTER_PASS == 3 || CLUSTER_PASS == 10
    StoreContributions(i,result);
#else
    StoreWork(p.links.z,result);
#endif
  }
  void SolveBody(uint i,float dt) {
    Body p=LoadBodies(i);Motion m=LoadMotions(i);Work w=LoadWork(i);
    if (p.links.w!=0) {
      m.anchor_velocity=float4((w.next_anchor.xyz-m.anchor.xyz)/dt,0);
      m.angular_velocity=float4(RotationVector(LodMultiply(w.next_rotation,LodInverse(m.rotation)))/dt,0);
    } else {
      Motion parent=LoadMotions(p.links.z);
      float3 v=parent.anchor_velocity.xyz+cross(parent.angular_velocity.xyz,LodRotate(parent.rotation,p.parent_offset.xyz));
      float3 rhs=w.hw.xyz+dt*dt*mul(JointSpring(p,parent),parent.angular_velocity.xyz)-mul(transpose(B(w)),v);
      m.angular_velocity=float4(mul(D(w),rhs),0);
      m.anchor_velocity=float4(v,0);
    }
    StoreMotions(i,m);
  }
  void Integrate(uint i,float dt) {
    Body p=LoadBodies(i);Motion m=LoadMotions(i);
    if (p.links.w!=0) {
      m.anchor=LoadWork(i).next_anchor;m.rotation=LoadWork(i).next_rotation;
    } else {
      m.rotation=normalize(LodMultiply(Rotation(dt*m.angular_velocity.xyz),m.rotation));
      Motion parent=LoadMotions(p.links.z);
      float3 arm=LodRotate(parent.rotation,p.parent_offset.xyz);
      m.anchor=float4(parent.anchor.xyz+arm,0);
      m.anchor_velocity=float4(parent.anchor_velocity.xyz+cross(parent.angular_velocity.xyz,arm),0);
    }
    StoreMotions(i,m);
  }
  void Reconstruct(uint i) {
    TreePhysicsBuffers full=LoadTrees(0);
    if (i<parameters.counts.z) {
      Vertex map=LoadVertices(i);Motion m=LoadMotions(map.indices.w);
      TreePhysicsVertex v=PhysicsVertex(full,0,map.indices.x);
      float3 arm=LodRotate(m.rotation,map.offset.xyz);
      v.position_inverse_mass.xyz=m.anchor.xyz+arm;
      v.velocity_radius.xyz=m.anchor_velocity.xyz+cross(m.angular_velocity.xyz,arm);
      LodStoreVertex(full,0,map.indices.x,v,false);LodStoreVertex(full,1,map.indices.x,v,false);
    }
    if (i<parameters.counts.y) {
      Rod map=LoadRods(i);Motion m=LoadMotions(map.indices.w);
      TreePhysicsRod r=PhysicsRod(full,0,map.indices.x);
      r.rotation=normalize(LodMultiply(m.rotation,map.relative_rotation));
      LodStoreRod(full,0,map.indices.x,r,false);LodStoreRod(full,1,map.indices.x,r,false);
    }
  }

#if CLUSTER_FOREST
  uint DepthCount() { return buffers.values[8]!=0 ? PhysicsLoad<LodState>(buffers.values[8],0).settings.control.z : lod.control.z; }
  bool UseWorkgroup() { return lod.control.w==2 || (lod.control.w==0 && parameters.counts.x>4*DepthCount()); }
  bool Eligible(Candidate c) {
    if(c.links.x==0xffffffffu || c.bounds.z==0) return false;
    for(uint policy=0;policy<3;++policy) {
      if((lod.control.x&(1u<<policy))==0 || c.links.z-c.links.y<max(2u,uint(lod.rules[policy].w))) continue;
      bool order=c.links.w!=0xffffffffu && c.links.w>=uint(lod.rules[policy].z);
      bool size=c.bounds.x<=lod.rules[policy].x && c.bounds.y<=lod.rules[policy].y;
      if((policy==0 && size) || (policy==1 && order) || (policy==2 && order && size)) return true;
    }
    return false;
  }
  uint SelectLod() {
    if(buffers.values[8]==0) return 0;
    LodState state=PhysicsLoad<LodState>(buffers.values[8],0);
    bool changed=state.status.y==0 || any(state.settings.control.xy!=lod.control.xy);
    for(uint r=0;r<3;++r) changed=changed || any(state.settings.rules[r]!=lod.rules[r]);
    state.status.z=changed ? 1 : 0;
    if(!changed) {PhysicsStore<LodState>(buffers.values[8],0,state);return 0;}
    if(state.settings.control.y!=lod.control.y) state.status.y=0;
    for(uint i=0;i<parameters.counts.y;++i) PhysicsStore<uint4>(buffers.values[16],i,uint4(0));
    uint count=0,depth_count=0;
    for(uint position=0;position<parameters.counts.y;) {
      uint root=PhysicsLoad<uint>(buffers.values[10],position);
      Candidate c=PhysicsLoad<Candidate>(buffers.values[9],root);
      uint end=Eligible(c) ? c.links.z : position+1;
      uint parent=c.links.x==0xffffffffu ? 0xffffffffu : PhysicsLoad<uint>(buffers.values[13],c.links.x);
      PhysicsStore<uint4>(buffers.values[15],count,uint4(root,parent,position,end));
      uint4 row=uint4(0,0xffffffffu,0xffffffffu,0);
      if(parent!=0xffffffffu) {
        uint4 p=LoadSchedule(parent);row.w=p.w+1;row.z=p.y;p.y=count;
        PhysicsStore<uint4>(buffers.values[6],parent,p);
      }
      PhysicsStore<uint4>(buffers.values[6],count,row);
      uint4 level=PhysicsLoad<uint4>(buffers.values[16],row.w);level.y++;
      PhysicsStore<uint4>(buffers.values[16],row.w,level);depth_count=max(depth_count,row.w+1);
      for(;position<end;++position) PhysicsStore<uint>(buffers.values[13],PhysicsLoad<uint>(buffers.values[10],position),count);
      ++count;
    }
    uint prefix=0;
    for(uint depth=0;depth<depth_count;++depth) {
      uint4 level=PhysicsLoad<uint4>(buffers.values[16],depth);level.x=level.z=prefix;prefix+=level.y;
      PhysicsStore<uint4>(buffers.values[16],depth,level);
    }
    for(uint body=0;body<count;++body) {
      uint depth=LoadSchedule(body).w;uint4 level=PhysicsLoad<uint4>(buffers.values[16],depth);
      uint4 row=LoadSchedule(level.z);row.x=body;PhysicsStore<uint4>(buffers.values[6],level.z,row);
      level.z++;PhysicsStore<uint4>(buffers.values[16],depth,level);
    }
    state.status.x=count;state.status.w++;state.settings=lod;state.settings.control.z=depth_count;
    PhysicsStore<LodState>(buffers.values[8],0,state);
    return (count+63)/64;
  }
  float3x3 Outer(float3 a,float3 b) {return float3x3(a.x*b,a.y*b,a.z*b);}
  float3x3 Identity(float value) {return float3x3(value,0,0,0,value,0,0,0,value);}
  float3x3 ParallelAxis(float3 p) {return Identity(dot(p,p))-Outer(p,p);}
  void TransferLod(uint i) {
    LodState state=PhysicsLoad<LodState>(buffers.values[8],0);
    if(i>=state.status.x) return;
    uint4 range=PhysicsLoad<uint4>(buffers.values[15],i);
    TreePhysicsBuffers full=LoadTrees(0);
    TreePhysicsRod root=PhysicsRod(full,parameters.substep_index,range.x);
    float3 anchor=PhysicsVertex(full,parameters.level_begin,root.links.x).position_inverse_mass.xyz;
    float4 q=root.rotation,inv=LodInverse(q);
    float mass=0;float3 weighted_center=0,momentum=0,angular=0;
    float3x3 inertia=Identity(0);
    for(uint at=range.z;at<range.w;++at) {
      uint id=PhysicsLoad<uint>(buffers.values[10],at);
      TreePhysicsRod rod=PhysicsRod(full,parameters.substep_index,id);
      TreePhysicsVertex av=PhysicsVertex(full,parameters.level_begin,rod.links.x),bv=PhysicsVertex(full,parameters.level_begin,rod.links.y);
      float3 a=av.position_inverse_mass.xyz-anchor,b=bv.position_inverse_mass.xyz-anchor;
      float3 delta=b-a;float rod_length=max(length(delta),1e-6);float3 axis=delta/rod_length,center=(a+b)*0.5;
      float m=rod.length_scale_mass_radius.z,radius=rod.length_scale_mass_radius.w;
      float axial=0.5*m*radius*radius,transverse=m*(3*radius*radius+rod_length*rod_length)/12;
      float3x3 spin_inertia=Identity(transverse)+(axial-transverse)*Outer(axis,axis);
      float3 omega=state.status.y!=0 ? LoadMotions(PhysicsLoad<uint>(buffers.values[12],id)).angular_velocity.xyz :
                   cross(delta,bv.velocity_radius.xyz-av.velocity_radius.xyz)/max(dot(delta,delta),1e-12);
      float3 velocity=(av.velocity_radius.xyz+bv.velocity_radius.xyz)*0.5;
      mass+=m;weighted_center+=m*center;momentum+=m*velocity;
      angular+=mul(spin_inertia,omega)+cross(center,m*velocity);
      inertia+=spin_inertia+m*ParallelAxis(center);
      Rod map;
      map.start_radius=float4(LodRotate(inv,a),radius);map.end_length=float4(LodRotate(inv,b),rod_length);
      map.relative_rotation=LodMultiply(inv,rod.rotation);map.indices=uint4(id,rod.links.x,rod.links.y,i);
      PhysicsStore<Rod>(buffers.values[2],at,map);
      Vertex vertex;vertex.offset=float4(LodRotate(inv,b),0);vertex.indices=uint4(rod.links.y,0,0,i);
      PhysicsStore<Vertex>(buffers.values[3],rod.links.y,vertex);
    }
    float3 center=weighted_center/mass;
    float3 omega=Solve(inertia-mass*ParallelAxis(center),angular-cross(center,momentum));
    Motion motion=(Motion)0;motion.rotation=q;motion.anchor=float4(anchor,0);
    motion.angular_velocity=float4(omega,0);motion.anchor_velocity=float4(momentum/mass-cross(omega,center),0);
    PhysicsStore<Motion>(buffers.values[14],i,motion);
    Body body=PhysicsLoad<Body>(buffers.values[11],range.x);
    body.center_mass=float4(LodRotate(inv,center),mass);
    float3x3 local=WorldMatrix(inv,inertia);
    StoreMatrix(body.inertia0,body.inertia1,body.inertia2,local);
    body.links=uint4(range.z,range.w-range.z,range.y,range.y==0xffffffffu ? 1u : 0u);
    if(range.y!=0xffffffffu) {
      uint parent_root=PhysicsLoad<uint4>(buffers.values[15],range.y).x;
      TreePhysicsRod parent=PhysicsRod(full,parameters.substep_index,parent_root);
      float3 parent_anchor=PhysicsVertex(full,parameters.level_begin,parent.links.x).position_inverse_mass.xyz;
      body.parent_offset=float4(LodRotate(LodInverse(parent.rotation),anchor-parent_anchor),0);
      body.relative_rotation=LodMultiply(LodInverse(PhysicsLoad<Body>(buffers.values[11],parent_root).rest_rotation),body.rest_rotation);
    } else {
      Vertex vertex;vertex.offset=0;vertex.indices=uint4(root.links.x,0,0,i);
      PhysicsStore<Vertex>(buffers.values[3],root.links.x,vertex);
    }
    PhysicsStore<Body>(buffers.values[0],i,body);
  }
  void FinishLod(uint i) {
    LodState state=PhysicsLoad<LodState>(buffers.values[8],0);
    if(i>=state.status.x) return;
    StoreMotions(i,PhysicsLoad<Motion>(buffers.values[14],i));
    uint4 range=PhysicsLoad<uint4>(buffers.values[15],i);
    for(uint at=range.z;at<range.w;++at) PhysicsStore<uint>(buffers.values[12],PhysicsLoad<uint>(buffers.values[10],at),i);
    if(i==0) {state.status.y=1;PhysicsStore<LodState>(buffers.values[8],0,state);}
  }
#endif

  void Run(uint i,uint first_substep,uint last_substep) {
#if CLUSTER_PASS == 0
    if (i!=0) return;
    float dt=parameters.mechanics.x/float(parameters.counts.w);
    for (uint substep=first_substep;substep<last_substep;++substep) {
      for (uint body=0;body<parameters.counts.x;++body) Prepare(body,substep,dt);
      for (uint body=parameters.counts.x;body>0;--body) Eliminate(body-1,dt);
      for (uint body=0;body<parameters.counts.x;++body) SolveBody(body,dt);
      for (uint body=0;body<parameters.counts.x;++body) Integrate(body,dt);
    }
#elif CLUSTER_PASS == 1
    Reconstruct(i);
#else
    if (i>=parameters.counts.x) return;
    uint body=LoadSchedule(parameters.level_begin+i).x;
    float dt=parameters.mechanics.x/float(parameters.counts.w);
#if CLUSTER_PASS == 2
    Prepare(body,parameters.substep_index,dt);
#elif CLUSTER_PASS == 3
    Eliminate(body,dt);
#elif CLUSTER_PASS == 4
    SolveBody(body,dt);
#else
    Integrate(body,dt);
#endif
#endif
  }

};
#if CLUSTER_FOREST
[[vk::binding(0,0)]] StructuredBuffer<Solver,Std430DataLayout> forest;
[[vk::binding(1,0)]] RWStructuredBuffer<uint2> transition_jobs;
[[vk::binding(2,0)]] RWStructuredBuffer<uint> transition_dispatch;
[[vk::push_constant]] cbuffer DispatchParameters { uint4 dispatch_counts; };
#else
[[vk::push_constant]] cbuffer DispatchParameters { Parameters parameters; };
#endif
[numthreads(64,1,1)]
void main(uint3 id:SV_DispatchThreadID,uint3 group:SV_GroupID) {
#if CLUSTER_FOREST
#if CLUSTER_PASS == 6
  if(id.x==0) {transition_dispatch[0]=0;transition_dispatch[1]=1;transition_dispatch[2]=1;transition_dispatch[3]=0;}
#elif CLUSTER_PASS == 7
  if(id.x>=dispatch_counts.x) return;
  Solver solver=forest[id.x];
  uint count=solver.SelectLod();
  if(count==0) return;
  uint begin;InterlockedAdd(transition_dispatch[3],count,begin);
  InterlockedMax(transition_dispatch[0],min(begin+count,65535u));
  InterlockedMax(transition_dispatch[1],(begin+count+65534)/65535);
  for(uint i=0;i<count;++i) transition_jobs[begin+i]=uint2(id.x,i*64);
#elif CLUSTER_PASS == 8 || CLUSTER_PASS == 9
  uint job=group.x+65535*group.y;
  if(job>=transition_dispatch[3]) return;
  uint2 task=transition_jobs[job];Solver solver=forest[task.x];
#if CLUSTER_PASS == 8
  solver.TransferLod(task.y+id.x%64);
#else
  solver.FinishLod(task.y+id.x%64);
#endif
#elif CLUSTER_PASS == 11
  if(id.y>=dispatch_counts.x) return;
  Solver solver=forest[id.y];
  if(dispatch_counts.y>=solver.parameters.counts.w || id.x>=solver.parameters.counts.y) return;
  Rod rod=solver.LoadRods(id.x);
  float3 force,torque;solver.RodLoads(rod,solver.LoadMotions(rod.indices.w),force,torque);
  PhysicsStore<float4>(solver.buffers.values[17],2*id.x,float4(force,0));
  PhysicsStore<float4>(solver.buffers.values[17],2*id.x+1,float4(torque,0));
#elif CLUSTER_PASS == 10
  uint tree=group.x;
  if(tree>=dispatch_counts.x) return;
  Solver solver=forest[tree];
  if(dispatch_counts.y>=solver.parameters.counts.w) return;
  if(solver.buffers.values[8]!=0) solver.parameters.counts.x=PhysicsLoad<LodState>(solver.buffers.values[8],0).status.x;
  if(!solver.UseWorkgroup()) return;
  uint depth_count=solver.DepthCount(),lane=id.x%64;
  float dt=solver.parameters.mechanics.x/float(solver.parameters.counts.w);
  for(uint substep=dispatch_counts.y;substep<dispatch_counts.y+1;++substep) {
    for(uint phase=0;phase<4;++phase) {
      for(uint at=0;at<depth_count;++at) {
        uint depth=phase==1 ? depth_count-1-at : at;
        uint4 level=PhysicsLoad<uint4>(solver.buffers.values[16],depth);
        for(uint j=lane;j<level.y;j+=64) {
          uint body=solver.LoadSchedule(level.x+j).x;
          if(phase==0) solver.Prepare(body,substep,dt);
          else if(phase==1) solver.Eliminate(body,dt);
          else if(phase==2) solver.SolveBody(body,dt);
          else solver.Integrate(body,dt);
        }
        AllMemoryBarrierWithGroupSync();
      }
    }
  }
#else
#if CLUSTER_PASS == 0
  uint tree=id.x;
#else
  uint tree=id.y;
#endif
  if(tree>=dispatch_counts.x) return;
  Solver solver=forest[tree];
  if(dispatch_counts.y>=solver.parameters.counts.w) return;
  if(solver.buffers.values[8]!=0) solver.parameters.counts.x=PhysicsLoad<LodState>(solver.buffers.values[8],0).status.x;
#if CLUSTER_PASS == 0
  if(!solver.UseWorkgroup()) solver.Run(0,dispatch_counts.y,dispatch_counts.y+1);
#else
  solver.Run(id.x,0,solver.parameters.counts.w);
#endif
#endif
#else
  Solver solver;
  solver.buffers=(Addresses)0;solver.parameters=parameters;solver.lod=(LodInput)0;solver.lod.material.x=1;
  solver.Run(id.x,0,solver.parameters.counts.w);
#endif
}
