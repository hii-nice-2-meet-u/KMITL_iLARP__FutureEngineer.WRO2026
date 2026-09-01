#pragma once

#include <algorithm>
#include <cmath>
#include <optional>

#include "navigation_controller.hpp"
#include "perception.hpp"

namespace obstacle_challenge {

struct ObstacleConfig {
	float activation_distance_m{1.50f};
	float release_forward_m{-0.10f};
	float pass_clearance_m{0.32f};
	float minimum_lookahead_m{0.35f};
	float avoidance_speed_mps{0.25f};
	float maximum_avoidance_steering_rad{
		32.0f * 3.14159265358979323846f / 180.0f};
	float emergency_distance_m{0.35f};
	float emergency_speed_mps{0.16f};
	float emergency_steering_rad{
		32.0f * 3.14159265358979323846f / 180.0f};
	float observation_merge_distance_m{0.18f};
	int confirmation_frames{2};
	int maximum_confirmation_missed_frames{5};
};

struct ObstacleStatus {
	bool active{false};
	bool activated{false};
	bool released{false};
	std::optional<navigation::TrafficColor> color;
	std::optional<navigation::PassSide> pass_side;
	float forward_m{0.0f};
	float right_m{0.0f};
	float target_right_m{0.0f};
	float steering_rad{0.0f};
	float confidence{0.0f};
};

class ObstacleController {
  public:
	explicit ObstacleController(ObstacleConfig config = {}) : config_(config) {}

	ObstacleStatus apply(const perception::PerceptionData &perception_data,
		const navigation::MapPose &pose, navigation::NavigationMode mode,
		const std::vector<navigation::TrafficLandmark> &landmarks,
		bool use_map, navigation::NavigationCommand &command) {
		ObstacleStatus status;
		if (mode == navigation::NavigationMode::FINISHED) {
			candidate_.reset();
			candidate_frames_ = 0;
			candidate_missed_frames_ = 0;
			if (active_.has_value()) {
				active_.reset();
				status.released = true;
			}
			return status;
		}

		const auto observation = nearest_observation(perception_data);
		update_candidate(observation);

		const bool emergency_observation = observation.has_value() &&
			observation->forward_m <= config_.emergency_distance_m;
		if (!active_.has_value() && candidate_.has_value() &&
			(emergency_observation ||
				candidate_frames_ >= std::max(1, config_.confirmation_frames))) {
			active_ = candidate_;
			status.activated = true;
		}
		if (!active_.has_value() && use_map) {
			const auto mapped = nearest_landmark(landmarks, pose);
			if (mapped.has_value()) {
				active_ = mapped;
				status.activated = true;
			}
		}

		if (active_.has_value() && observation.has_value() &&
			observation->color == active_->color &&
			distance(*observation, *active_) <=
				config_.observation_merge_distance_m) {
			active_->x_m += (observation->x_m - active_->x_m) * 0.25f;
			active_->y_m += (observation->y_m - active_->y_m) * 0.25f;
			active_->confidence = std::max(
				active_->confidence, observation->confidence);
		}

		if (!active_.has_value()) {
			return status;
		}

		const RobotPosition relative = to_robot(*active_, pose);
		if (relative.forward_m <= config_.release_forward_m) {
			active_.reset();
			candidate_.reset();
			candidate_frames_ = 0;
			candidate_missed_frames_ = 0;
			status.released = true;
			return status;
		}

		status.active = true;
		status.color = active_->color;
		status.pass_side = active_->pass_side;
		status.forward_m = relative.forward_m;
		status.right_m = relative.right_m;
		status.confidence = active_->confidence;

		const float side = active_->pass_side == navigation::PassSide::RIGHT
			? 1.0f
			: -1.0f;
		status.target_right_m =
			relative.right_m + side * config_.pass_clearance_m;
		const float lookahead_m =
			std::max(config_.minimum_lookahead_m, relative.forward_m);
		const float avoidance_steering_rad = std::clamp(
			std::atan2(status.target_right_m, lookahead_m),
			-config_.maximum_avoidance_steering_rad,
			config_.maximum_avoidance_steering_rad);
		if (relative.forward_m <= config_.emergency_distance_m) {
			command.steering_rad = side * std::max(0.0f,
				config_.emergency_steering_rad);
			command.target_speed_mps = std::min(command.target_speed_mps,
				config_.emergency_speed_mps);
		} else {
			command.steering_rad = avoidance_steering_rad;
			command.target_speed_mps = std::min(command.target_speed_mps,
				config_.avoidance_speed_mps);
		}
		status.steering_rad = command.steering_rad;
		return status;
	}

  private:
	struct Observation {
		float x_m{0.0f};
		float y_m{0.0f};
		navigation::TrafficColor color{navigation::TrafficColor::RED};
		navigation::PassSide pass_side{navigation::PassSide::RIGHT};
		float confidence{0.0f};
		float forward_m{0.0f};
	};

	struct RobotPosition {
		float right_m{0.0f};
		float forward_m{0.0f};
	};

	std::optional<Observation> nearest_observation(
		const perception::PerceptionData &data) const {
		std::optional<Observation> nearest;
		for (const auto &object : data.obstacles) {
			if (!object.frame_confirmed || !object.world_position.has_value() ||
				!object.traffic_color.has_value() ||
				!object.required_pass_side.has_value() ||
				object.robot_position.forward_m <= 0.05f ||
				object.robot_position.forward_m > config_.activation_distance_m) {
				continue;
			}
			Observation observation{object.world_position->x_m,
				object.world_position->y_m, *object.traffic_color,
				*object.required_pass_side, object.fusion_confidence,
				object.robot_position.forward_m};
			if (!nearest.has_value() ||
				observation.forward_m < nearest->forward_m) {
				nearest = observation;
			}
		}
		return nearest;
	}

	std::optional<Observation> nearest_landmark(
		const std::vector<navigation::TrafficLandmark> &landmarks,
		const navigation::MapPose &pose) const {
		std::optional<Observation> nearest;
		for (const auto &landmark : landmarks) {
			if (landmark.confidence < 0.60f || landmark.observations < 3) {
				continue;
			}
			Observation observation{landmark.x_m, landmark.y_m, landmark.color,
				landmark.pass_side, landmark.confidence, 0.0f};
			const RobotPosition relative = to_robot(observation, pose);
			if (relative.forward_m <= 0.05f ||
				relative.forward_m > config_.activation_distance_m ||
				std::abs(relative.right_m) > 0.75f) {
				continue;
			}
			observation.forward_m = relative.forward_m;
			if (!nearest.has_value() ||
				observation.forward_m < nearest->forward_m) {
				nearest = observation;
			}
		}
		return nearest;
	}

	void update_candidate(const std::optional<Observation> &observation) {
		if (!observation.has_value()) {
			if (candidate_.has_value() &&
				++candidate_missed_frames_ <=
					std::max(0, config_.maximum_confirmation_missed_frames)) {
				return;
			}
			candidate_.reset();
			candidate_frames_ = 0;
			candidate_missed_frames_ = 0;
			return;
		}
		if (candidate_.has_value() && candidate_->color == observation->color &&
			distance(*candidate_, *observation) <=
				config_.observation_merge_distance_m) {
			candidate_->x_m += (observation->x_m - candidate_->x_m) * 0.25f;
			candidate_->y_m += (observation->y_m - candidate_->y_m) * 0.25f;
			candidate_->confidence = std::max(
				candidate_->confidence, observation->confidence);
			++candidate_frames_;
			candidate_missed_frames_ = 0;
			return;
		}
		candidate_ = observation;
		candidate_frames_ = 1;
		candidate_missed_frames_ = 0;
	}

	static float distance(const Observation &a, const Observation &b) {
		return std::hypot(a.x_m - b.x_m, a.y_m - b.y_m);
	}

	static RobotPosition to_robot(
		const Observation &observation, const navigation::MapPose &pose) {
		const float dx = observation.x_m - pose.x_m;
		const float dy = observation.y_m - pose.y_m;
		const float cosine = std::cos(pose.heading_rad);
		const float sine = std::sin(pose.heading_rad);
		return {dx * sine - dy * cosine, dx * cosine + dy * sine};
	}

	ObstacleConfig config_;
	std::optional<Observation> candidate_;
	std::optional<Observation> active_;
	int candidate_frames_{0};
	int candidate_missed_frames_{0};
};

inline const char *color_name(navigation::TrafficColor color) {
	return color == navigation::TrafficColor::RED ? "RED" : "GREEN";
}

inline const char *side_name(navigation::PassSide side) {
	return side == navigation::PassSide::RIGHT ? "RIGHT" : "LEFT";
}

}
