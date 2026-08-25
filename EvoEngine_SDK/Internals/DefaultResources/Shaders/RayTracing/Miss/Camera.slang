import EvoEngine.CameraRayConstants;
import EvoEngine.CameraRayTracingPayload;

[shader("miss")]
void main(inout CameraRayTracingPayload hit_value) {
  hit_value.type = EE_CAMERA_RAY_PAYLOAD_MISS;
}
