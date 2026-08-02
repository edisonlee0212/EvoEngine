// @evoengine-dialect native
import EvoEngine.GizmosConstants;

[shader("fragment")]
float4 main() : SV_Target {
  return EE_GIZMOS_CONSTANTS.color;
}
