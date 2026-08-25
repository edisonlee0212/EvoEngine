__exported import EvoEngine.AnimationBones;

public float4x4 EE_SKINNING_MATRIX(int4 bone_ids, float4 weights, int4 bone_ids2, float4 weights2) {
  float4x4 result = EE_ANIM_BONES[bone_ids.x] * weights.x;
  if (bone_ids.y != -1) result += EE_ANIM_BONES[bone_ids.y] * weights.y;
  if (bone_ids.z != -1) result += EE_ANIM_BONES[bone_ids.z] * weights.z;
  if (bone_ids.w != -1) result += EE_ANIM_BONES[bone_ids.w] * weights.w;
  if (bone_ids2.x != -1) result += EE_ANIM_BONES[bone_ids2.x] * weights2.x;
  if (bone_ids2.y != -1) result += EE_ANIM_BONES[bone_ids2.y] * weights2.y;
  if (bone_ids2.z != -1) result += EE_ANIM_BONES[bone_ids2.z] * weights2.z;
  if (bone_ids2.w != -1) result += EE_ANIM_BONES[bone_ids2.w] * weights2.w;
  return result;
}
