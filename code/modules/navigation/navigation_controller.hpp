#pragma once

#include "initial_direction_estimator.hpp"
#include "lidar_processor.hpp"
#include "navigation_state.hpp"
#include "stanley_controller.hpp"

namespace navigation {

struct NavigationConfig {

	float target_outer_distance_m{0.30f};

	control::StanleyConfig stanley{};

	// Center robot while direction is unknown.
	float search_center_kp{0.8f};

	// ---------------------------------------------------------
	// Corner
	// ---------------------------------------------------------

	float approach_distance_m{0.80f};

	float turn_trigger_distance_m{0.50f};

	float turn_rearm_distance_m{0.80f};

	// ---------------------------------------------------------

	float turn_heading_kp{1.5f};

	float heading_tolerance_rad{5.0f * 3.14159265358979323846f / 180.0f};

	int heading_confirm_frames{3};

	// Default assumes OTOS:
	// +heading = CCW
	//
	// CW  = -90 deg
	// CCW = +90 deg
	float clockwise_turn_delta_rad{-3.14159265358979323846f / 2.0f};

	float counter_clockwise_turn_delta_rad{3.14159265358979323846f / 2.0f};

	// Converts OTOS angular error to steering convention:
	//
	// steering < 0 = LEFT
	// steering > 0 = RIGHT
	//
	// Default:
	// OTOS + = CCW
	// steering + = RIGHT
	float heading_to_steering_sign{-1.0f};

	// ---------------------------------------------------------

	float search_speed_mps{0.20f};

	float normal_speed_mps{0.50f};

	float approach_speed_mps{0.35f};

	float turning_speed_mps{0.25f};

	float lost_wall_speed_mps{0.20f};

	int total_turns{12};
};

class NavigationController {
  public:
	explicit NavigationController(NavigationConfig config = {},
		InitialDirectionConfig direction_config = {});

	NavigationResult update(const lidar::ProcessedLidarData &lidar_data,
		float heading_rad, float speed_mps);

	void reset(float heading_rad = 0.0f);

	const NavigationState &state() const { return state_; }

  private:
	struct TrackWalls {
		const lidar::LineSegment *inner{nullptr};
		const lidar::LineSegment *outer{nullptr};
	};

	NavigationCommand update_search_direction(
		const lidar::ProcessedLidarData &lidar_data, float heading_rad,
		NavigationDebug &debug);

	NavigationCommand update_normal(const lidar::ProcessedLidarData &lidar_data,
		float heading_rad, float speed_mps, NavigationDebug &debug);

	NavigationCommand update_turning(float heading_rad, NavigationDebug &debug);

	TrackWalls resolve_track_walls(const lidar::ResolvedWalls &walls) const;

	float calculate_cross_track_error(
		const lidar::LineSegment &outer_wall) const;

	float calculate_wall_heading_error(
		const lidar::LineSegment &outer_wall) const;

	bool should_start_turn(const lidar::ResolvedWalls &walls) const;

	void start_turn(float heading_rad);

	float calculate_turn_heading_error(float heading_rad) const;

	bool is_turn_complete(float heading_error_rad);

	float calculate_search_steering(const lidar::ResolvedWalls &walls) const;

	float clamp_steering(float steering_rad) const;

	static float normalize_angle(float angle_rad);

  private:
	NavigationConfig config_;
	NavigationState state_;

	InitialDirectionEstimator direction_estimator_;

	control::StanleyController stanley_;
};

} // namespace navigation