#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <string>

#include "perception.hpp"

namespace {

constexpr float PI = 3.14159265358979323846f;

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
	if (!std::isfinite(actual) || std::abs(actual - expected) > tolerance) {
		fail(message + " actual=" + std::to_string(actual) +
			" expected=" + std::to_string(expected));
	}
}

lidar::ObstacleObject lidar_object(
	float right_m, float forward_m, float width_m = 0.05f) {
	lidar::ObstacleObject object;
	object.center = {right_m, forward_m};
	object.width_m = width_m;
	object.angle_rad = 0.0f;
	return object;
}

camera::CameraObject camera_object(camera::Color color, float bearing_rad) {
	camera::CameraObject object;
	object.color = color;
	object.bounding_box = {100, 100, 30, 60};
	object.bottom_center = {115.0f, 160.0f};
	object.bearing_rad = bearing_rad;
	return object;
}

} // namespace

int main() {
	perception::PerceptionConfig config;
	config.max_sensor_time_difference_us = 100'000;
	config.max_bearing_difference_rad = 8.0f * PI / 180.0f;
	perception::Perception fusion(config);

	// 1) A time-synchronised, angularly aligned pair must fuse. Red maps to
	// pass-right and the LiDAR supplies range/world position.
	lidar::ProcessedLidarData lidar_data;
	lidar_data.timestamp_us = 1'000'000;
	lidar_data.obstacles.push_back(lidar_object(0.20f, 1.00f));
	camera::ProcessedCameraData camera_data;
	camera_data.timestamp_us = 1'020'000;
	camera_data.objects.push_back(
		camera_object(camera::Color::Red, std::atan2(0.20f, 1.00f)));

	auto result = fusion.process(lidar_data, camera_data, {10.0f, 20.0f, 0.0f});
	expect(result.diagnostics.camera_time_synchronized,
		"20 ms sensor delta was rejected");
	expect(
		result.obstacles.size() == 1 && result.diagnostics.matched_count == 1,
		"aligned camera/LiDAR objects were not fused");
	const auto &red = result.obstacles.front();
	expect(red.frame_confirmed,
		"high-quality fused obstacle was not frame-confirmed");
	expect(red.traffic_color == navigation::TrafficColor::RED,
		"red camera object did not become a red traffic obstacle");
	expect(red.required_pass_side == navigation::PassSide::RIGHT,
		"red obstacle did not request pass-right");
	expect(
		red.world_position.has_value(), "valid pose produced no world point");
	expect_near(red.world_position->x_m, 11.0f, 1e-4f,
		"forward distance was transformed to the wrong world X");
	expect_near(red.world_position->y_m, 19.8f, 1e-4f,
		"right offset was transformed to the wrong world Y");

	// 2) A stale camera frame must never colour a newer LiDAR object.
	camera_data.timestamp_us = 800'000;
	result = fusion.process(lidar_data, camera_data, navigation::MapPose{});
	expect(!result.diagnostics.camera_time_synchronized,
		"stale camera frame was marked synchronised");
	expect(result.obstacles.front().source ==
			perception::ObservationSource::LIDAR_ONLY,
		"stale camera frame was fused");
	expect(result.diagnostics.unmatched_camera_count == 1,
		"stale camera detection was not reported as unmatched");
	expect(result.diagnostics.unmatched_lidar_count == 1,
		"stale LiDAR object was not reported as unmatched");

	// 3) A synchronised object outside the angular gate remains LiDAR-only.
	camera_data.timestamp_us = 1'000'000;
	camera_data.objects.front().bearing_rad = -0.50f;
	result = fusion.process(lidar_data, camera_data, navigation::MapPose{});
	expect(result.diagnostics.matched_count == 0,
		"large bearing residual passed the association gate");
	expect(result.diagnostics.unmatched_camera_count == 1,
		"bearing-rejected camera object was not exposed to diagnostics");

	// 4) Association is one-to-one: one camera blob cannot colour two LiDAR
	// clusters even when both fall inside the angular gate.
	lidar_data.obstacles.push_back(lidar_object(0.22f, 1.00f));
	camera_data.objects.front().bearing_rad = std::atan2(0.21f, 1.00f);
	result = fusion.process(lidar_data, camera_data, navigation::MapPose{});
	expect(result.diagnostics.matched_count == 1,
		"one camera blob was associated more than once");
	expect(result.obstacles.size() == 2,
		"valid unmatched LiDAR object was discarded");

	// 5) Invalid geometry is rejected before matching, without poisoning good
	// objects in the same frame.
	lidar_data.obstacles.push_back(
		lidar_object(std::numeric_limits<float>::quiet_NaN(), 1.0f));
	camera_data.objects.push_back(camera_object(camera::Color::Green, 0.0f));
	camera_data.objects.back().bounding_box.width = 0;
	result = fusion.process(lidar_data, camera_data, navigation::MapPose{});
	expect(result.diagnostics.rejected_lidar_count == 1,
		"non-finite LiDAR object was not rejected");
	expect(result.diagnostics.rejected_camera_count == 1,
		"invalid camera bounding box was not rejected");

	// 6) Camera mounting yaw is part of the predicted bearing. A camera aimed
	// 5 degrees right sees a straight-ahead object at -5 degrees in its image.
	config.camera_mount.yaw_rad = 5.0f * PI / 180.0f;
	perception::Perception yaw_compensated(config);
	lidar_data.obstacles = {lidar_object(0.0f, 1.0f)};
	camera_data.objects = {
		camera_object(camera::Color::Green, -5.0f * PI / 180.0f)};
	result =
		yaw_compensated.process(lidar_data, camera_data, {0.0f, 0.0f, 0.0f});
	expect(result.diagnostics.matched_count == 1,
		"camera mounting yaw was not compensated");
	expect(result.obstacles.front().required_pass_side ==
			navigation::PassSide::LEFT,
		"green obstacle did not request pass-left");

	// 7) Invalid OTOS pose must not block robot-frame perception; it only
	// suppresses the world coordinate.
	const float nan = std::numeric_limits<float>::quiet_NaN();
	result =
		yaw_compensated.process(lidar_data, camera_data, {nan, 0.0f, 0.0f});
	expect(!result.diagnostics.pose_valid,
		"non-finite OTOS pose was marked valid");
	expect(!result.obstacles.front().world_position.has_value(),
		"invalid OTOS pose produced a world coordinate");
	expect(result.obstacles.front().frame_confirmed,
		"invalid OTOS pose incorrectly disabled robot-frame fusion");

	std::cout << "PASS: timestamp, geometry, one-to-one association, colour, "
				 "mount calibration, confidence and world transform\n";
	return 0;
}
