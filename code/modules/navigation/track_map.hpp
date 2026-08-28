#pragma once

#include <array>
#include <cstddef>
#include <optional>
#include <vector>

#include "direction.hpp"

namespace navigation {

constexpr std::size_t TRACK_CORNER_COUNT = 4;

struct MapPose {
	float x_m{0.0f};
	float y_m{0.0f};
	float heading_rad{0.0f};
};

struct CornerObservation {
	MapPose pose;
	float turn_trigger_m{0.0f};
	float corner_radius_m{0.0f};
	float safe_speed_mps{0.0f};
};

struct CornerLandmark {
	MapPose entry_pose;
	MapPose exit_pose;

	float preferred_turn_trigger_m{0.0f};
	float preferred_corner_radius_m{0.0f};
	float safe_speed_mps{0.0f};
	float confidence{0.0f};

	int entry_observations{0};
	int exit_observations{0};

	bool entry_valid{false};
	bool exit_valid{false};
};

enum class TrafficColor { RED, GREEN };
enum class PassSide { LEFT, RIGHT };

struct TrafficLandmark {
	float x_m{0.0f};
	float y_m{0.0f};
	TrafficColor color{TrafficColor::RED};
	PassSide pass_side{PassSide::RIGHT};
	float confidence{0.0f};
	int observations{0};
};

struct ReplayHint {
	std::size_t corner_index{0};
	float distance_to_entry_m{0.0f};
	float heading_error_rad{0.0f};
	float preferred_turn_trigger_m{0.0f};
	float preferred_corner_radius_m{0.0f};
	float safe_speed_mps{0.0f};
	float confidence{0.0f};
	bool approach_recommended{false};
};

struct TrackMapConfig {
	float replay_preview_distance_m{1.10f};
	float replay_max_heading_error_rad{0.45f};
	float minimum_replay_confidence{0.60f};
	// TODO(WRO tuning): A three-lap run provides few updates per corner, so the
	// fixed 0.25 EMA can over-weight lap 1. After real logging is available,
	// compare a 0.5-0.6 weight with a running average of
	// 1 / (observation_count + 1). Keep 0.25 until data supports changing it.
	float update_weight{0.25f};
	float traffic_merge_distance_m{0.25f};
};

class TrackMap {
  public:
	explicit TrackMap(TrackMapConfig config = {});

	void reset();
	void set_direction(DrivingDirection direction);

	void record_corner_entry(
		std::size_t corner_index, const CornerObservation &observation);
	void record_corner_exit(std::size_t corner_index, const MapPose &pose);

	std::optional<ReplayHint> replay_hint(
		const MapPose &pose, std::size_t next_corner_index) const;

	// Reserved for Obstacle Challenge; not wired into Open Challenge main loops
	// yet.
	std::size_t observe_traffic_light(float x_m, float y_m, TrafficColor color,
		PassSide pass_side, float confidence);

	bool ready_for_replay() const;
	std::size_t learned_corner_count() const;

	const std::array<CornerLandmark, TRACK_CORNER_COUNT> &corners() const {
		return corners_;
	}

	const std::vector<TrafficLandmark> &traffic_landmarks() const {
		return traffic_landmarks_;
	}

	const std::optional<DrivingDirection> &direction() const {
		return direction_;
	}

  private:
	static float normalize_angle(float angle_rad);
	static float blend_angle(
		float current_rad, float observed_rad, float weight);
	static float distance(float ax_m, float ay_m, float bx_m, float by_m);

	TrackMapConfig config_;
	std::optional<DrivingDirection> direction_;
	std::array<CornerLandmark, TRACK_CORNER_COUNT> corners_{};
	std::vector<TrafficLandmark> traffic_landmarks_;
};

} // namespace navigation
