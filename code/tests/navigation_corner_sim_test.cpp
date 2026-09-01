#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <optional>
#include <string>

#include "navigation_controller.hpp"

namespace {

constexpr float PI = 3.14159265358979323846f;
constexpr float DEG_TO_RAD = PI / 180.0f;
constexpr float DT_S = 0.05f;

[[noreturn]] void fail(const std::string &message) {
	std::cerr << "FAIL: " << message << '\n';
	std::exit(1);
}

void expect(bool condition, const std::string &message) {
	if (!condition) {
		fail(message);
	}
}

float normalize_angle(float angle_rad) {
	return std::atan2(std::sin(angle_rad), std::cos(angle_rad));
}

lidar::LineSegment make_segment(
	const cv::Point2f &start, const cv::Point2f &end) {
	lidar::LineSegment segment;
	segment.start = start;
	segment.end = end;

	const cv::Point2f tangent = end - start;
	const float length = cv::norm(tangent);

	expect(length > 1e-5f, "synthetic segment must have non-zero length");

	segment.angle_rad = std::atan2(tangent.y, tangent.x);
	segment.normal_x = -tangent.y / length;
	segment.normal_y = tangent.x / length;
	segment.line_c = -(segment.normal_x * start.x + segment.normal_y * start.y);

	return segment;
}

lidar::ProcessedLidarData make_direction_frame(DrivingDirection direction,
	float front_distance_m, std::uint64_t timestamp_us) {
	lidar::ProcessedLidarData data;
	data.timestamp_us = timestamp_us;

	const float side_x =
		direction == DrivingDirection::CLOCKWISE ? -0.30f : 0.30f;
	const auto side = make_segment({side_x, 0.05f}, {side_x, front_distance_m});

	lidar::LineSegment front;
	if (direction == DrivingDirection::CLOCKWISE) {
		front =
			make_segment({side_x, front_distance_m}, {0.45f, front_distance_m});
		data.walls.left = side;
	} else {
		front = make_segment(
			{-0.45f, front_distance_m}, {side_x, front_distance_m});
		data.walls.right = side;
	}

	data.walls.front = front;
	data.line_segments = {side, front};

	return data;
}

lidar::ProcessedLidarData make_clockwise_corner_landmark_frame(
	float corner_forward_from_lidar_m, std::uint64_t timestamp_us) {
	lidar::ProcessedLidarData data;
	data.timestamp_us = timestamp_us;

	const auto outer = make_segment({-0.30f, 0.05f}, {-0.30f, 1.20f});
	const auto inner =
		make_segment({0.30f, 0.05f}, {0.30f, corner_forward_from_lidar_m});
	const auto front = make_segment({-0.30f, 1.20f}, {0.30f, 1.20f});
	data.walls.left = outer;
	data.walls.right = inner;
	data.walls.front = front;
	data.line_segments = {outer, inner, front};
	return data;
}

void test_world_stable_inner_corner_trigger() {
	navigation::NavigationConfig config;
	config.use_wall_corner_trigger = true;
	config.lidar_forward_offset_m = 0.08f;
	config.corner_radius_m = 0.30f;
	config.turn_preview_time_s = 0.20f;
	config.wall_corner_stability_tolerance_m = 0.04f;
	config.wall_corner_confirm_frames = 3;
	config.turn_trigger_confirm_frames = 2;
	config.front_wall_fallback_distance_m = 0.30f;
	config.steering_filter_time_constant_s = 0.0f;
	config.max_steering_rate_rad_s = 100.0f;
	config.max_acceleration_mps2 = 100.0f;
	config.max_deceleration_mps2 = 100.0f;

	navigation::InitialDirectionConfig direction_config;
	direction_config.frame_min_score = 5.0f;
	direction_config.frame_score_margin = 1.0f;
	direction_config.required_confirm_frames = 1;

	navigation::NavigationController controller(config, direction_config);
	std::uint64_t timestamp_us = 1'000'000;
	constexpr float speed_mps = 0.50f;

	auto result = controller.update(
		make_direction_frame(DrivingDirection::CLOCKWISE, 1.20f, timestamp_us),
		0.0f, speed_mps, std::nullopt, navigation::MapPose{0.0f, 0.0f, 0.0f});
	expect(controller.state().mode == navigation::NavigationMode::NORMAL,
		"direction lock failed before wall-corner tracking test");

	for (int frame_index = 0; frame_index < 6; ++frame_index) {
		timestamp_us += 50'000;
		const float rear_y_m = 0.03f * static_cast<float>(frame_index);
		result = controller.update(
			make_clockwise_corner_landmark_frame(0.90f, timestamp_us), 0.0f,
			speed_mps, std::nullopt, navigation::MapPose{0.0f, rear_y_m, 0.0f});
	}
	expect(!result.debug.wall_corner_confirmed,
		"sensor-fixed line endpoint was mistaken for a world landmark");

	constexpr float wall_corner_world_y_m = 1.00f;
	for (const float rear_y_m : {0.20f, 0.23f, 0.26f, 0.29f}) {
		timestamp_us += 50'000;
		const float corner_from_lidar_m =
			wall_corner_world_y_m - rear_y_m - config.lidar_forward_offset_m;
		result = controller.update(make_clockwise_corner_landmark_frame(
									   corner_from_lidar_m, timestamp_us),
			0.0f, speed_mps, std::nullopt,
			navigation::MapPose{0.0f, rear_y_m, 0.0f});
	}
	expect(result.debug.wall_corner_confirmed,
		"stationary world-frame inner corner was not confirmed");
	expect(controller.state().mode == navigation::NavigationMode::NORMAL,
		"corner triggered before the geometric turn-entry distance");

	for (const float rear_y_m : {0.59f, 0.61f, 0.63f}) {
		timestamp_us += 50'000;
		const float corner_from_lidar_m =
			wall_corner_world_y_m - rear_y_m - config.lidar_forward_offset_m;
		result = controller.update(make_clockwise_corner_landmark_frame(
									   corner_from_lidar_m, timestamp_us),
			0.0f, speed_mps, std::nullopt,
			navigation::MapPose{0.0f, rear_y_m, 0.0f});
	}

	expect(controller.state().mode == navigation::NavigationMode::TURNING,
		"confirmed inner corner did not trigger at radius + speed preview");
	expect(result.debug.wall_corner_trigger_active,
		"debug output did not identify the wall-corner trigger source");
}

void test_corridor_center_cross_track_error() {
	for (const DrivingDirection direction : {DrivingDirection::CLOCKWISE,
			 DrivingDirection::COUNTER_CLOCKWISE}) {
		navigation::NavigationConfig config;
		config.follow_corridor_center = true;
		config.use_wall_corner_trigger = false;

		navigation::InitialDirectionConfig direction_config;
		direction_config.frame_min_score = 5.0f;
		direction_config.frame_score_margin = 1.0f;
		direction_config.required_confirm_frames = 1;

		navigation::NavigationController controller(config, direction_config);
		std::uint64_t timestamp_us = 1'000'000;
		controller.update(
			make_direction_frame(direction, 1.20f, timestamp_us), 0.0f, 0.25f);

		lidar::ProcessedLidarData data;
		data.timestamp_us = timestamp_us + 50'000;
		data.walls.left = make_segment({-0.20f, 0.05f}, {-0.20f, 1.20f});
		data.walls.right = make_segment({0.40f, 0.05f}, {0.40f, 1.20f});
		const auto result = controller.update(data, 0.0f, 0.25f);

		expect(result.debug.corridor_center_active,
			"corridor center did not activate with both side walls");
		expect(std::abs(result.debug.distance_error_m - 0.10f) < 1e-4f,
			"corridor center correction had the wrong magnitude or sign");

	}
}

void run_corner_simulation(
	DrivingDirection direction, float initial_speed_mps) {
	navigation::NavigationConfig config;
	config.target_outer_distance_m = 0.30f;
	config.stanley.k = 0.85f;
	config.stanley.softening_speed_mps = 0.30f;
	config.stanley.max_steering_rad = 30.0f * DEG_TO_RAD;
	config.approach_distance_m = 0.90f;
	config.turn_trigger_distance_m = 0.50f;
	config.turn_preview_time_s = 0.10f;
	config.turn_trigger_confirm_frames = 2;
	config.wheelbase_m = 0.18f;
	config.corner_radius_m = 0.40f;
	config.turn_entry_blend_rad = 10.0f * DEG_TO_RAD;
	config.turn_exit_blend_rad = 22.0f * DEG_TO_RAD;
	config.exit_acceleration_blend_rad = 15.0f * DEG_TO_RAD;
	config.turn_heading_pid.kp = 0.8f;
	config.heading_tolerance_rad = 3.0f * DEG_TO_RAD;
	config.heading_confirm_frames = 3;
	config.normal_speed_mps = 0.85f;
	config.approach_speed_mps = 0.72f;
	config.turning_speed_mps = 0.65f;
	config.max_lateral_acceleration_mps2 = 1.40f;
	config.steering_filter_time_constant_s = 0.035f;
	config.max_steering_rate_rad_s = 7.0f;
	config.max_acceleration_mps2 = 1.8f;
	config.max_deceleration_mps2 = 3.0f;
	config.total_turns = 1;

	navigation::InitialDirectionConfig direction_config;
	direction_config.max_connection_gap_m = 0.35f;
	direction_config.frame_min_score = 5.0f;
	direction_config.frame_score_margin = 1.0f;
	direction_config.required_confirm_frames = 1;

	navigation::NavigationController controller(config, direction_config);

	float heading_rad = 0.0f;
	float vehicle_speed_mps = initial_speed_mps;
	float previous_steering_rad = 0.0f;
	std::uint64_t timestamp_us = 1'000'000;

	auto frame = make_direction_frame(direction, 0.54f, timestamp_us);
	auto result = controller.update(frame, heading_rad, vehicle_speed_mps);

	expect(controller.state().direction == std::optional(direction),
		"initial direction estimator did not lock to the synthetic corner");
	expect(controller.state().mode == navigation::NavigationMode::NORMAL,
		"controller did not enter NORMAL after direction lock");

	// Verify that start_turn advances the stable straight reference instead of
	// accumulating an instantaneous heading error into every future corner.
	if (std::abs(initial_speed_mps - config.turning_speed_mps) < 0.01f) {
		heading_rad = direction == DrivingDirection::CLOCKWISE
			? 4.0f * DEG_TO_RAD
			: -4.0f * DEG_TO_RAD;
	}

	bool entered_turn = false;
	bool finished = false;
	float peak_steering_rad = 0.0f;
	float peak_steering_progress = 0.0f;
	float peak_raw_steering_rad = 0.0f;
	float peak_steering_rate_rad_s = 0.0f;
	float minimum_mid_corner_speed_mps = 100.0f;
	int mid_corner_samples = 0;
	int completion_step = -1;

	for (int step = 0; step < 300; ++step) {
		timestamp_us += 50'000;
		frame = make_direction_frame(direction, 0.54f, timestamp_us);

		if (entered_turn) {
			frame.walls.front.reset();
		}

		result = controller.update(frame, heading_rad, vehicle_speed_mps);

		if (controller.state().mode == navigation::NavigationMode::TURNING) {
			entered_turn = true;

			const float expected_sign =
				direction == DrivingDirection::CLOCKWISE ? 1.0f : -1.0f;
			expect(result.command.steering_rad * expected_sign > -0.01f,
				"corner steering changed to the wrong direction");

			if (result.debug.turn_progress > 0.25f &&
				result.debug.turn_progress < 0.75f) {
				minimum_mid_corner_speed_mps =
					std::min(minimum_mid_corner_speed_mps,
						result.command.target_speed_mps);
				++mid_corner_samples;
			}
		}

		if (controller.state().mode != navigation::NavigationMode::FINISHED) {
			const float steering_step_rad =
				std::abs(result.command.steering_rad - previous_steering_rad);
			peak_steering_rate_rad_s =
				std::max(peak_steering_rate_rad_s, steering_step_rad / DT_S);
			expect(steering_step_rad <=
					config.max_steering_rate_rad_s * DT_S + 1e-4f,
				"steering slew-rate limit was exceeded");
		}

		if (std::abs(result.command.steering_rad) > peak_steering_rad) {
			peak_steering_rad = std::abs(result.command.steering_rad);
			peak_steering_progress = result.debug.turn_progress;
			peak_raw_steering_rad = std::abs(result.debug.raw_steering_rad);
		}
		previous_steering_rad = result.command.steering_rad;

		vehicle_speed_mps +=
			std::clamp(result.command.target_speed_mps - vehicle_speed_mps,
				-config.max_deceleration_mps2 * DT_S,
				config.max_acceleration_mps2 * DT_S);

		// Kinematic bicycle model using this project's steering convention:
		// positive steering turns right, while positive OTOS heading is CCW.
		heading_rad = normalize_angle(heading_rad -
			vehicle_speed_mps / config.wheelbase_m *
				std::tan(result.command.steering_rad) * DT_S);

		if (controller.state().mode == navigation::NavigationMode::FINISHED) {
			finished = true;
			completion_step = step + 1;
			break;
		}
	}

	expect(entered_turn, "speed-preview corner trigger never fired");
	expect(finished, "smooth 90-degree corner did not converge");
	expect(mid_corner_samples > 3,
		"simulation did not collect enough mid-corner samples");
	expect(minimum_mid_corner_speed_mps >= 0.60f,
		"conditioned speed dropped below the fast-corner target");
	expect(peak_steering_rad <= config.stanley.max_steering_rad + 1e-5f,
		"steering exceeded the physical limit");
	if (std::abs(initial_speed_mps - config.turning_speed_mps) < 0.01f) {
		expect(peak_raw_steering_rad < 29.5f * DEG_TO_RAD,
			"nominal corner saturated raw steering; trajectory is too "
			"aggressive");
	}

	const float expected_heading_rad =
		direction == DrivingDirection::CLOCKWISE ? -0.5f * PI : 0.5f * PI;
	expect(std::abs(normalize_angle(expected_heading_rad - heading_rad)) <
			5.0f * DEG_TO_RAD,
		"final heading error exceeded five degrees");

	std::cout << (direction == DrivingDirection::CLOCKWISE ? "CW" : "CCW")
			  << " from " << initial_speed_mps << " m/s: turn converged in "
			  << completion_step * DT_S << " s, peak steering "
			  << peak_steering_rad / DEG_TO_RAD << " deg at "
			  << peak_steering_progress * 100.0f << "% (raw "
			  << peak_raw_steering_rad / DEG_TO_RAD << " deg), peak slew "
			  << peak_steering_rate_rad_s / DEG_TO_RAD
			  << " deg/s, mid-corner speed >= " << minimum_mid_corner_speed_mps
			  << " m/s\n";
}

} // namespace

int main() {
	test_world_stable_inner_corner_trigger();
	test_corridor_center_cross_track_error();

	for (const float initial_speed_mps : {0.45f, 0.65f, 0.80f}) {
		run_corner_simulation(DrivingDirection::CLOCKWISE, initial_speed_mps);
		run_corner_simulation(
			DrivingDirection::COUNTER_CLOCKWISE, initial_speed_mps);
	}

	std::cout << "PASS: smooth high-speed CW and CCW corner simulations\n";
	return 0;
}
