#include "perception.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace perception {

namespace {

struct MatchCandidate {
	std::size_t fused_index{0};
	std::size_t camera_index{0};
	float predicted_camera_bearing_rad{0.0f};
	float bearing_error_rad{0.0f};
};

} // namespace

Perception::Perception(PerceptionConfig config) : config_(config) {
	config_.max_bearing_difference_rad =
		std::max(0.001f, std::abs(config_.max_bearing_difference_rad));
	config_.minimum_confirmed_confidence =
		std::clamp(config_.minimum_confirmed_confidence, 0.0f, 1.0f);
	config_.minimum_lidar_distance_m =
		std::max(0.0f, config_.minimum_lidar_distance_m);
	config_.maximum_lidar_distance_m = std::max(
		config_.minimum_lidar_distance_m, config_.maximum_lidar_distance_m);
}

PerceptionData Perception::process(const lidar::ProcessedLidarData &lidar_data,
	const camera::ProcessedCameraData &camera_data,
	const std::optional<navigation::MapPose> &vehicle_pose) const {
	PerceptionData output;
	output.timestamp_us = lidar_data.timestamp_us;
	output.track_walls = lidar_data.walls;
	output.parking_wall = lidar_data.parking_wall;

	auto &diagnostics = output.diagnostics;
	diagnostics.lidar_timestamp_us = lidar_data.timestamp_us;
	diagnostics.camera_timestamp_us = camera_data.timestamp_us;
	diagnostics.pose_valid =
		vehicle_pose.has_value() && valid_pose(*vehicle_pose);
	diagnostics.lidar_input_count = lidar_data.obstacles.size();
	diagnostics.camera_input_count = camera_data.objects.size();
	diagnostics.sensor_time_difference_us =
		timestamp_difference(lidar_data.timestamp_us, camera_data.timestamp_us);
	diagnostics.camera_time_synchronized = lidar_data.timestamp_us != 0 &&
		camera_data.timestamp_us != 0 &&
		diagnostics.sensor_time_difference_us <=
			config_.max_sensor_time_difference_us;

	output.obstacles.reserve(lidar_data.obstacles.size());
	for (std::size_t index = 0; index < lidar_data.obstacles.size(); ++index) {
		const auto &object = lidar_data.obstacles[index];
		if (!valid_lidar_object(object)) {
			++diagnostics.rejected_lidar_count;
			continue;
		}

		FusedObstacle fused;
		fused.lidar_object_index = index;
		fused.robot_position = lidar_to_robot(object.center);
		fused.world_position =
			robot_to_world(fused.robot_position, vehicle_pose);
		fused.distance_m = std::hypot(
			fused.robot_position.right_m, fused.robot_position.forward_m);
		fused.width_m = object.width_m;
		fused.lidar_bearing_rad = std::atan2(
			fused.robot_position.right_m, fused.robot_position.forward_m);
		output.obstacles.push_back(fused);
		++diagnostics.valid_lidar_count;
	}

	std::vector<std::size_t> valid_camera_indices;
	valid_camera_indices.reserve(camera_data.objects.size());
	for (std::size_t index = 0; index < camera_data.objects.size(); ++index) {
		if (valid_camera_object(camera_data.objects[index])) {
			valid_camera_indices.push_back(index);
			++diagnostics.valid_camera_count;
		} else {
			++diagnostics.rejected_camera_count;
		}
	}

	if (!diagnostics.camera_time_synchronized) {
		output.unmatched_camera_object_indices =
			std::move(valid_camera_indices);
		diagnostics.unmatched_lidar_count = diagnostics.valid_lidar_count;
		diagnostics.unmatched_camera_count =
			output.unmatched_camera_object_indices.size();
		return output;
	}

	std::vector<MatchCandidate> candidates;
	candidates.reserve(output.obstacles.size() * valid_camera_indices.size());
	for (std::size_t fused_index = 0; fused_index < output.obstacles.size();
		 ++fused_index) {
		const float predicted = predicted_camera_bearing(
			output.obstacles[fused_index].robot_position);
		if (!std::isfinite(predicted)) {
			continue;
		}

		for (const std::size_t camera_index : valid_camera_indices) {
			const float measured =
				camera_data.objects[camera_index].bearing_rad;
			const float error = std::abs(normalize_angle(measured - predicted));
			if (error <= config_.max_bearing_difference_rad) {
				candidates.push_back(
					{fused_index, camera_index, predicted, error});
			}
		}
	}

	// Greedy one-to-one association by the smallest angular residual. A camera
	// blob and a LiDAR cluster can each be consumed at most once.
	std::sort(candidates.begin(), candidates.end(),
		[](const MatchCandidate &a, const MatchCandidate &b) {
			return a.bearing_error_rad < b.bearing_error_rad;
		});

	std::vector<bool> fused_used(output.obstacles.size(), false);
	std::vector<bool> camera_used(camera_data.objects.size(), false);
	for (const auto &candidate : candidates) {
		if (fused_used[candidate.fused_index] ||
			camera_used[candidate.camera_index]) {
			continue;
		}

		auto &fused = output.obstacles[candidate.fused_index];
		const auto &camera_object = camera_data.objects[candidate.camera_index];
		fused.source = ObservationSource::LIDAR_CAMERA_FUSED;
		fused.camera_object_index = candidate.camera_index;
		fused.camera_color = camera_object.color;
		fused.traffic_color = to_traffic_color(camera_object.color);
		fused.required_pass_side = pass_side_for(camera_object.color);
		fused.camera_bearing_rad = camera_object.bearing_rad;
		fused.predicted_camera_bearing_rad =
			candidate.predicted_camera_bearing_rad;
		fused.bearing_error_rad = candidate.bearing_error_rad;

		const float bearing_score = std::clamp(1.0f -
				candidate.bearing_error_rad /
					config_.max_bearing_difference_rad,
			0.0f, 1.0f);
		const float time_score = config_.max_sensor_time_difference_us == 0
			? 1.0f
			: std::clamp(1.0f -
					  static_cast<float>(
						  diagnostics.sensor_time_difference_us) /
						  static_cast<float>(
							  config_.max_sensor_time_difference_us),
				  0.0f, 1.0f);
		fused.fusion_confidence = 0.75f * bearing_score + 0.25f * time_score;
		fused.frame_confirmed =
			fused.fusion_confidence >= config_.minimum_confirmed_confidence;

		fused_used[candidate.fused_index] = true;
		camera_used[candidate.camera_index] = true;
		++diagnostics.matched_count;
		if (fused.frame_confirmed) {
			++diagnostics.frame_confirmed_count;
		}
	}

	for (const std::size_t camera_index : valid_camera_indices) {
		if (!camera_used[camera_index]) {
			output.unmatched_camera_object_indices.push_back(camera_index);
		}
	}
	diagnostics.unmatched_camera_count =
		output.unmatched_camera_object_indices.size();
	diagnostics.unmatched_lidar_count =
		diagnostics.valid_lidar_count - diagnostics.matched_count;
	return output;
}

float Perception::normalize_angle(float angle_rad) {
	return std::atan2(std::sin(angle_rad), std::cos(angle_rad));
}

std::uint64_t Perception::timestamp_difference(
	std::uint64_t a_us, std::uint64_t b_us) {
	return a_us >= b_us ? a_us - b_us : b_us - a_us;
}

bool Perception::valid_pose(const navigation::MapPose &pose) {
	return std::isfinite(pose.x_m) && std::isfinite(pose.y_m) &&
		std::isfinite(pose.heading_rad);
}

navigation::TrafficColor Perception::to_traffic_color(camera::Color color) {
	return color == camera::Color::Red ? navigation::TrafficColor::RED
									   : navigation::TrafficColor::GREEN;
}

navigation::PassSide Perception::pass_side_for(camera::Color color) {
	return color == camera::Color::Red ? navigation::PassSide::RIGHT
									   : navigation::PassSide::LEFT;
}

bool Perception::valid_lidar_object(const lidar::ObstacleObject &object) const {
	if (!std::isfinite(object.center.x) || !std::isfinite(object.center.y) ||
		!std::isfinite(object.width_m) || !std::isfinite(object.angle_rad)) {
		return false;
	}
	const RobotPoint position = lidar_to_robot(object.center);
	const float distance = std::hypot(position.right_m, position.forward_m);
	return std::isfinite(distance) &&
		distance >= config_.minimum_lidar_distance_m &&
		distance <= config_.maximum_lidar_distance_m;
}

bool Perception::valid_camera_object(const camera::CameraObject &object) {
	return std::isfinite(object.bearing_rad) && object.bounding_box.width > 0 &&
		object.bounding_box.height > 0 &&
		std::isfinite(object.bottom_center.x) &&
		std::isfinite(object.bottom_center.y);
}

RobotPoint Perception::lidar_to_robot(const cv::Point2f &point) const {
	const float cosine = std::cos(config_.lidar_mount.yaw_rad);
	const float sine = std::sin(config_.lidar_mount.yaw_rad);
	return {
		config_.lidar_mount.right_m + point.x * cosine + point.y * sine,
		config_.lidar_mount.forward_m - point.x * sine + point.y * cosine,
	};
}

std::optional<WorldPoint> Perception::robot_to_world(const RobotPoint &point,
	const std::optional<navigation::MapPose> &pose) const {
	if (!pose.has_value() || !valid_pose(*pose)) {
		return std::nullopt;
	}

	// OTOS/world convention: heading 0 points along +world X and positive
	// heading turns toward +world Y. Robot-right is therefore -world Y at
	// heading 0.
	const float cosine = std::cos(pose->heading_rad);
	const float sine = std::sin(pose->heading_rad);
	return WorldPoint{
		pose->x_m + point.forward_m * cosine + point.right_m * sine,
		pose->y_m + point.forward_m * sine - point.right_m * cosine,
	};
}

float Perception::predicted_camera_bearing(const RobotPoint &point) const {
	const float relative_right = point.right_m - config_.camera_mount.right_m;
	const float relative_forward =
		point.forward_m - config_.camera_mount.forward_m;
	if (std::hypot(relative_right, relative_forward) <
		std::numeric_limits<float>::epsilon()) {
		return std::numeric_limits<float>::quiet_NaN();
	}
	return normalize_angle(std::atan2(relative_right, relative_forward) -
		config_.camera_mount.yaw_rad);
}

} // namespace perception
