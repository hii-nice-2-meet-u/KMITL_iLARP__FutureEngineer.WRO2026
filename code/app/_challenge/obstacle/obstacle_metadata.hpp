#pragma once

#include "obstacle_controller.hpp"
#include "perception.hpp"
#include "run_metadata.hpp"

namespace obstacle_challenge {

inline logging::JsonObject sensor_mount_json(
	const perception::SensorMount &mount) {
	logging::JsonObject object;
	object.add_number("right_m", mount.right_m)
		.add_number("forward_m", mount.forward_m)
		.add_number("yaw_rad", mount.yaw_rad);
	return object;
}

inline logging::JsonObject perception_config_json(
	const perception::PerceptionConfig &config) {
	logging::JsonObject object;
	object
		.add_unsigned("max_sensor_time_difference_us",
			config.max_sensor_time_difference_us)
		.add_number(
			"max_bearing_difference_rad", config.max_bearing_difference_rad)
		.add_number("minimum_confirmed_confidence",
			config.minimum_confirmed_confidence)
		.add_number(
			"minimum_lidar_distance_m", config.minimum_lidar_distance_m)
		.add_number(
			"maximum_lidar_distance_m", config.maximum_lidar_distance_m)
		.add_object("lidar_mount", sensor_mount_json(config.lidar_mount))
		.add_object("camera_mount", sensor_mount_json(config.camera_mount));
	return object;
}

inline logging::JsonObject obstacle_config_json(const ObstacleConfig &config) {
	logging::JsonObject object;
	object.add_number("wheelbase_m", config.wheelbase_m)
		.add_number("curvature_gain", config.curvature_gain)
		.add_number("maximum_combined_steering_rad",
			config.maximum_combined_steering_rad)
		.add_number("activation_distance_m", config.activation_distance_m)
		.add_number("release_forward_m", config.release_forward_m)
		.add_number("pass_clearance_m", config.pass_clearance_m)
		.add_number("minimum_lookahead_m", config.minimum_lookahead_m)
		.add_number("avoidance_speed_mps", config.avoidance_speed_mps)
		.add_number("maximum_avoidance_steering_rad",
			config.maximum_avoidance_steering_rad)
		.add_number("emergency_distance_m", config.emergency_distance_m)
		.add_number("emergency_speed_mps", config.emergency_speed_mps)
		.add_number("emergency_steering_rad",
			config.emergency_steering_rad)
		.add_number("observation_merge_distance_m",
			config.observation_merge_distance_m)
		.add_integer("confirmation_frames", config.confirmation_frames)
		.add_integer("maximum_confirmation_missed_frames",
			config.maximum_confirmation_missed_frames);
	return object;
}

} // namespace obstacle_challenge
