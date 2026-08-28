#include "track_map.hpp"

#include <algorithm>
#include <cmath>

namespace navigation {

TrackMap::TrackMap(TrackMapConfig config) : config_(config) {}

void TrackMap::reset() {
	direction_.reset();
	corners_ = {};
	traffic_landmarks_.clear();
}

void TrackMap::set_direction(DrivingDirection direction) {
	direction_ = direction;
}

void TrackMap::record_corner_entry(
	std::size_t corner_index, const CornerObservation &observation) {
	CornerLandmark &corner = corners_.at(corner_index % TRACK_CORNER_COUNT);
	const float weight = std::clamp(config_.update_weight, 0.0f, 1.0f);

	if (!corner.entry_valid) {
		corner.entry_pose = observation.pose;
		corner.preferred_turn_trigger_m = observation.turn_trigger_m;
		corner.preferred_corner_radius_m = observation.corner_radius_m;
		corner.safe_speed_mps = observation.safe_speed_mps;
		corner.entry_valid = true;
	} else {
		corner.entry_pose.x_m +=
			(observation.pose.x_m - corner.entry_pose.x_m) * weight;
		corner.entry_pose.y_m +=
			(observation.pose.y_m - corner.entry_pose.y_m) * weight;
		corner.entry_pose.heading_rad =
			blend_angle(corner.entry_pose.heading_rad,
				observation.pose.heading_rad, weight);
		corner.preferred_turn_trigger_m +=
			(observation.turn_trigger_m - corner.preferred_turn_trigger_m) *
			weight;
		corner.preferred_corner_radius_m +=
			(observation.corner_radius_m - corner.preferred_corner_radius_m) *
			weight;
		corner.safe_speed_mps +=
			(observation.safe_speed_mps - corner.safe_speed_mps) * weight;
	}

	++corner.entry_observations;
	corner.confidence = std::clamp(0.45f +
			0.15f * static_cast<float>(corner.entry_observations) +
			0.20f * static_cast<float>(corner.exit_observations),
		0.0f, 1.0f);
}

void TrackMap::record_corner_exit(
	std::size_t corner_index, const MapPose &pose) {
	CornerLandmark &corner = corners_.at(corner_index % TRACK_CORNER_COUNT);
	const float weight = std::clamp(config_.update_weight, 0.0f, 1.0f);

	if (!corner.exit_valid) {
		corner.exit_pose = pose;
		corner.exit_valid = true;
	} else {
		corner.exit_pose.x_m += (pose.x_m - corner.exit_pose.x_m) * weight;
		corner.exit_pose.y_m += (pose.y_m - corner.exit_pose.y_m) * weight;
		corner.exit_pose.heading_rad =
			blend_angle(corner.exit_pose.heading_rad, pose.heading_rad, weight);
	}

	++corner.exit_observations;
	corner.confidence = std::clamp(0.45f +
			0.15f * static_cast<float>(corner.entry_observations) +
			0.20f * static_cast<float>(corner.exit_observations),
		0.0f, 1.0f);
}

std::optional<ReplayHint> TrackMap::replay_hint(
	const MapPose &pose, std::size_t next_corner_index) const {
	if (!ready_for_replay()) {
		return std::nullopt;
	}

	const std::size_t index = next_corner_index % TRACK_CORNER_COUNT;
	const CornerLandmark &corner = corners_[index];
	if (!corner.entry_valid ||
		corner.confidence < config_.minimum_replay_confidence) {
		return std::nullopt;
	}

	ReplayHint hint;
	hint.corner_index = index;
	hint.distance_to_entry_m = distance(
		pose.x_m, pose.y_m, corner.entry_pose.x_m, corner.entry_pose.y_m);
	hint.heading_error_rad =
		normalize_angle(corner.entry_pose.heading_rad - pose.heading_rad);
	hint.preferred_turn_trigger_m = corner.preferred_turn_trigger_m;
	hint.preferred_corner_radius_m = corner.preferred_corner_radius_m;
	hint.safe_speed_mps = corner.safe_speed_mps;
	hint.confidence = corner.confidence;
	hint.approach_recommended =
		hint.distance_to_entry_m <= config_.replay_preview_distance_m &&
		std::abs(hint.heading_error_rad) <=
			config_.replay_max_heading_error_rad;

	return hint;
}

std::size_t TrackMap::observe_traffic_light(float x_m, float y_m,
	TrafficColor color, PassSide pass_side, float confidence) {
	const float observation_confidence = std::clamp(confidence, 0.0f, 1.0f);
	const float weight = std::clamp(config_.update_weight, 0.0f, 1.0f);

	for (std::size_t index = 0; index < traffic_landmarks_.size(); ++index) {
		TrafficLandmark &landmark = traffic_landmarks_[index];
		if (landmark.color != color ||
			distance(x_m, y_m, landmark.x_m, landmark.y_m) >
				config_.traffic_merge_distance_m) {
			continue;
		}

		landmark.x_m += (x_m - landmark.x_m) * weight;
		landmark.y_m += (y_m - landmark.y_m) * weight;
		landmark.pass_side = pass_side;
		landmark.confidence = std::clamp(
			landmark.confidence + 0.25f * observation_confidence, 0.0f, 1.0f);
		++landmark.observations;
		return index;
	}

	traffic_landmarks_.push_back(
		{x_m, y_m, color, pass_side, observation_confidence, 1});
	return traffic_landmarks_.size() - 1;
}

bool TrackMap::ready_for_replay() const {
	if (!direction_.has_value()) {
		return false;
	}

	return std::all_of(
		corners_.begin(), corners_.end(), [](const CornerLandmark &corner) {
			return corner.entry_valid && corner.exit_valid;
		});
}

std::size_t TrackMap::learned_corner_count() const {
	return static_cast<std::size_t>(std::count_if(
		corners_.begin(), corners_.end(), [](const CornerLandmark &corner) {
			return corner.entry_valid && corner.exit_valid;
		}));
}

float TrackMap::normalize_angle(float angle_rad) {
	return std::atan2(std::sin(angle_rad), std::cos(angle_rad));
}

float TrackMap::blend_angle(
	float current_rad, float observed_rad, float weight) {
	return normalize_angle(
		current_rad + normalize_angle(observed_rad - current_rad) * weight);
}

float TrackMap::distance(float ax_m, float ay_m, float bx_m, float by_m) {
	return std::hypot(ax_m - bx_m, ay_m - by_m);
}

} // namespace navigation
