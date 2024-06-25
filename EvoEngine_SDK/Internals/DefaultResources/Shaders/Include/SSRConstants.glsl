
layout(push_constant) uniform EE_SSR_CONSTANTS{
    uint EE_CAMERA_INDEX;
    int numBinarySearchSteps;
    float step;
    float minRayStep;
    int maxSteps;
    float reflectionSpecularFalloffExponent;
    int horizontal;
    float weight[5];
};
