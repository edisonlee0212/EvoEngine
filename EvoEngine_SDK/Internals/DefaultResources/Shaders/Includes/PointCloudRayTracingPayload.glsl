#extension GL_ARB_gpu_shader_int64 : enable

struct HitInfo {
	vec3 position;  /**< Position of the hit point in world coordinates. */
	float vertex_info1;
	vec3 normal;    /**< Normal vector at the hit surface. */
	float vertex_info2;
	vec3 tangent;   /**< Tangent vector at the hit surface. */
	float vertex_info3;
	vec4 color;     /**< Color information at the hit point. */
	vec2 tex_coord; /**< Texture coordinates at the hit point. */
	vec2 vertex_info4;     /**< Extra data for user-specific needs. */
};

struct PointCloudRayTracingPayload{
	uint hit_count;    /**< Flag indicating whether a hit occurred. */
	uint seed;
	uint64_t handle; /**< Handle or identifier for the point-cloud object. */

	HitInfo hit_info; /**< Detailed information about the hit, if one occurred. */
};