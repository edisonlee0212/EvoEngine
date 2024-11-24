
struct SegmentPair {
	int segment0_handle;
	int segment1_handle;
	int valid;
	float max_strain;
	vec4 stiffness;

	vec4 segment0_particle0_offset;
	vec4 segment0_particle1_offset;

	vec4 segment1_particle0_offset;
	vec4 segment1_particle1_offset;

	vec4 rest_darboux_vector;
};

struct ProfileData {
	int start_pair_handle;
	int end_pair_handle;
};

layout(std430, set = 1, binding = 0) buffer SEGMENT_PAIR_BLOCK {
	SegmentPair segment_pairs[];
};

layout(std430, set = 1, binding = 1) buffer PROFILE_DATA_0_BLOCK {
	ProfileData profile_data_list_front[];
};

layout(std430, set = 1, binding = 2) buffer PROFILE_DATA_1_BLOCK {
	ProfileData profile_data_list_back[];
};


void BundleProfile(in ProfileData profile_data, in float inv_time_step) {
	for (int pair_handle = profile_data.start_pair_handle; pair_handle <= profile_data.end_pair_handle; pair_handle++) {
		SegmentPair segment_pair = segment_pairs[pair_handle];
		if (segment_pair.valid == 0)
		  continue;
		Segment segment0 = segments[segment_pair.segment0_handle];
		Segment segment1 = segments[segment_pair.segment1_handle];

		int segment0_particle0_handle = floatBitsToInt(segment0.inertia_tensor_particle_0_handle.w);
		int segment0_particle1_handle = floatBitsToInt(segment0.inv_inertia_tensor_particle_1_handle.w);

		int segment1_particle0_handle = floatBitsToInt(segment1.inertia_tensor_particle_0_handle.w);
		int segment1_particle1_handle = floatBitsToInt(segment1.inv_inertia_tensor_particle_1_handle.w);

		Particle segment0_particle0 = particles[segment0_particle0_handle];
		Particle segment0_particle1 = particles[segment0_particle1_handle];
		Particle segment1_particle0 = particles[segment1_particle0_handle];
		Particle segment1_particle1 = particles[segment1_particle1_handle];

		vec3 segment0_center_position = (segment0_particle0.x_node_handle.xyz + segment0_particle1.x_node_handle.xyz) * 0.5;
		vec3 segment1_center_position = (segment1_particle0.x_node_handle.xyz + segment1_particle1.x_node_handle.xyz) * 0.5;

		vec3 target_segment0_particle0_position =
			segment1_center_position + rotate_vec3(segment1.q, segment_pair.segment0_particle0_offset.xyz);
		vec3 target_segment0_particle1_position =
			segment1_center_position + rotate_vec3(segment1.q, segment_pair.segment0_particle1_offset.xyz);
		vec3 target_segment1_particle0_position =
			segment0_center_position + rotate_vec3(segment0.q, segment_pair.segment1_particle0_offset.xyz);
		vec3 target_segment1_particle1_position =
			segment0_center_position + rotate_vec3(segment0.q, segment_pair.segment1_particle1_offset.xyz);

		float t2 = inv_time_step * inv_time_step;
		float stiffness_factor = t2 / segment_pair.stiffness.w;
		float lambda = (segment0.inv_mass + segment1.inv_mass + stiffness_factor);
		float factor0 = segment0.inv_mass / lambda;
		float factor1 = segment1.inv_mass / lambda;

		vec3 segment0_particle0_position_correction =
			(target_segment0_particle0_position - segment0_particle0.x_node_handle.xyz) * factor0;
		vec3 segment0_particle1_position_correction =
			(target_segment0_particle1_position - segment0_particle1.x_node_handle.xyz) * factor0;
		vec3 segment1_particle0_position_correction =
			(target_segment1_particle0_position - segment1_particle0.x_node_handle.xyz) * factor1;
		vec3 segment1_particle1_position_correction =
			(target_segment1_particle1_position - segment1_particle1.x_node_handle.xyz) * factor1;

		vec3 new_segment0_particle0_position =
			segment0_particle0.x_node_handle.xyz + segment0_particle0_position_correction;
		vec3 new_segment0_particle1_position =
			segment0_particle1.x_node_handle.xyz + segment0_particle1_position_correction;
		vec3 new_segment1_particle0_position =
			segment1_particle0.x_node_handle.xyz + segment1_particle0_position_correction;
		vec3 new_segment1_particle1_position =
			segment1_particle1.x_node_handle.xyz + segment1_particle1_position_correction;

		particles[segment0_particle0_handle].x_node_handle.xyz = new_segment0_particle0_position;
		particles[segment0_particle1_handle].x_node_handle.xyz = new_segment0_particle1_position;
		particles[segment1_particle0_handle].x_node_handle.xyz = new_segment1_particle0_position;
		particles[segment1_particle1_handle].x_node_handle.xyz = new_segment1_particle1_position;

		vec4 q0_correction, q1_correction;
		project_bend_twist_constraint(inv_time_step, segment0.q, segment0.inv_mass, segment1.q,
										segment1.inv_mass, segment_pair.stiffness.xyz,
										segment_pair.rest_darboux_vector, q0_correction, q1_correction);

		segments[segment_pair.segment0_handle].q = normalize(q0_correction + segment0.q);
		segments[segment_pair.segment1_handle].q = normalize(q1_correction + segment1.q);

		if (segment0.prev_handle != -1) {
			int copy_particle_handle =
			floatBitsToInt(segments[segment0.prev_handle].inv_inertia_tensor_particle_1_handle.w); 
			if(connections[segment0_particle0.connection_handle].bend_twist_strain_valid.w != 0.0) {
				particles[copy_particle_handle].x_node_handle.xyz = new_segment0_particle0_position;
			}
		}

		if (segment0.next_handle != -1) {
			int copy_particle_handle = floatBitsToInt(segments[segment0.next_handle].inertia_tensor_particle_0_handle.w);
			if (connections[segment0_particle1.connection_handle].bend_twist_strain_valid.w != 0.0) {
				particles[copy_particle_handle].x_node_handle.xyz = new_segment0_particle1_position;
			}
		}

		if (segment1.prev_handle != -1) {
			int copy_particle_handle =
			floatBitsToInt(segments[segment1.prev_handle].inv_inertia_tensor_particle_1_handle.w);
			if(connections[segment1_particle0.connection_handle].bend_twist_strain_valid.w != 0.0) {
				particles[copy_particle_handle].x_node_handle.xyz = new_segment1_particle0_position;
			}
		}

		if (segment1.next_handle != -1) {
			int copy_particle_handle = floatBitsToInt(segments[segment1.next_handle].inertia_tensor_particle_0_handle.w);
			if (connections[segment1_particle1.connection_handle].bend_twist_strain_valid.w != 0.0) {
				particles[copy_particle_handle].x_node_handle.xyz = new_segment1_particle1_position;
			}
		}
	}
}