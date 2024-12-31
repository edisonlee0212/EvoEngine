
layout(push_constant) uniform EE_SSR_CONSTANTS{
    int EE_CAMERA_INDEX;
    float max_distance;
    float distance_confidence;
    int iteration_count;
    int initial_steps;
    float thickness;
};
