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

[[noreturn]] void fail(const std::string &message) {
	std::cerr << "FAIL: " << message << '\n';
	std::exit(1);
}

void expect(bool condition, const std::string &message) {
	if (!condition) {
		fail(message);
	}
}

void expect_near(
	float actual, float expected, float tolerance, const std::string &message) {
	if (std::abs(actual - expected) > tolerance) {
		fail(message + ": actual=" + std::to_string(actual) +
			" expected=" + std::to_string(expected));
	}
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

lidar::ProcessedLidarData make_clockwise_wall_frame(
	std::uint64_t timestamp_us) {
	lidar::ProcessedLidarData data;
	data.timestamp_us = timestamp_us;

	const auto outer_wall = make_segment({-0.30f, 0.05f}, {-0.30f, 0.90f});
	const auto front_wall = make_segment({-0.30f, 0.90f}, {0.45f, 0.90f});
	data.walls.left = outer_wall;
	data.walls.front = front_wall;
	data.line_segments = {outer_wall, front_wall};
	return data;
}

lidar::ProcessedLidarData make_wall_lost_frame(std::uint64_t timestamp_us) {
	lidar::ProcessedLidarData data;
	data.timestamp_us = timestamp_us;
	return data;
}

void test_temporary_wall_loss_holds_otos_heading() {
	navigation::NavigationConfig config;
	config.max_heading_hold_s = 0.30f;
	config.lost_wall_speed_mps = 0.25f;
	config.steering_filter_time_constant_s = 0.0f;
	config.max_steering_rate_rad_s = 100.0f;
	config.max_acceleration_mps2 = 100.0f;
	config.max_deceleration_mps2 = 100.0f;

	navigation::InitialDirectionConfig direction_config;
	direction_config.max_connection_gap_m = 0.35f;
	direction_config.frame_min_score = 5.0f;
	direction_config.frame_score_margin = 1.0f;
	direction_config.required_confirm_frames = 1;

	navigation::NavigationController controller(config, direction_config);
	std::uint64_t timestamp_us = 1'000'000;

	auto result =
		controller.update(make_clockwise_wall_frame(timestamp_us), 0.0f, 0.40f);
	expect(controller.state().direction ==
			std::optional(DrivingDirection::CLOCKWISE),
		"direction estimator did not lock to CW");
	expect(controller.state().mode == navigation::NavigationMode::NORMAL,
		"controller did not enter NORMAL mode");

	// A NORMAL frame with a valid outer wall captures the OTOS heading.
	timestamp_us += 50'000;
	result =
		controller.update(make_clockwise_wall_frame(timestamp_us), 0.0f, 0.40f);
	expect(
		result.debug.outer_wall_valid, "synthetic outer wall was not accepted");

	// The wall disappears for 0.2 s while OTOS reports +6 degrees of drift.
	// OTOS positive is CCW, so the controller must command positive/right
	// steering to return to the captured heading.
	timestamp_us += 200'000;
	result = controller.update(
		make_wall_lost_frame(timestamp_us), 6.0f * DEG_TO_RAD, 0.40f);
	expect(result.debug.heading_hold_active,
		"heading hold was not active during a short wall loss");
	expect(!result.debug.outer_wall_valid,
		"wall-lost frame was incorrectly marked valid");
	expect(result.command.steering_rad > 0.01f,
		"heading hold steering did not oppose positive OTOS drift");
	expect_near(result.command.target_speed_mps, config.lost_wall_speed_mps,
		1e-5f, "short wall loss did not use lost-wall speed");
	expect_near(result.debug.lost_wall_time_s, 0.20f, 1e-5f,
		"short wall-loss timer was incorrect");

	// Total wall loss is now 0.35 s, beyond the configured 0.30 s hold.
	timestamp_us += 150'000;
	result = controller.update(
		make_wall_lost_frame(timestamp_us), 6.0f * DEG_TO_RAD, 0.40f);
	expect(!result.debug.heading_hold_active,
		"heading hold remained active after its timeout");
	expect_near(result.command.steering_rad, 0.0f, 1e-6f,
		"timed-out wall loss did not fall back to zero steering");
	expect_near(result.command.target_speed_mps, config.lost_wall_speed_mps,
		1e-5f, "timed-out wall loss did not retain lost-wall speed");
	expect_near(result.debug.lost_wall_time_s, 0.35f, 1e-5f,
		"wall-loss timeout timer was incorrect");
}

} // namespace

int main() {
	test_temporary_wall_loss_holds_otos_heading();
	std::cout << "PASS: OTOS heading hold and lost-wall timeout\n";
	return 0;
}
