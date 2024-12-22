
layout(push_constant) uniform EE_SSR_CONSTANTS{
    int EE_CAMERA_INDEX;
    float maxDistance;
    float resolution;
    int initial_steps;
    float thickness;
    int horizontal;
    float weight[5];
};
