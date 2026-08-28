#pragma once

#include <cstdint>

#include "init_direction.hpp"
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

	// Compensate perception + steering latency at higher speed. The effective
	// trigger is base distance + speed * preview time.
	float turn_preview_time_s{0.08f};

	int turn_trigger_confirm_frames{2};

	// ---------------------------------------------------------
	// Smooth geometric corner trajectory
	// ---------------------------------------------------------

	// Measure wheelbase from rear-axle center to front-axle center.
	float wheelbase_m{0.18f};

	// WRO outer corners have a 0.10 m wall radius. A vehicle path 0.30 m
	// inside that wall therefore starts with an approximately 0.40 m radius.
	float corner_radius_m{0.40f};

	float turn_entry_blend_rad{10.0f * 3.14159265358979323846f / 180.0f};

	float turn_exit_blend_rad{22.0f * 3.14159265358979323846f / 180.0f};

	float exit_acceleration_blend_rad{15.0f * 3.14159265358979323846f / 180.0f};

	// Tracks a moving heading reference around the geometric corner. Unlike
	// targeting the final 90-degree heading immediately, this avoids saturating
	// the steering at corner entry.

	float turn_heading_kp{0.8f};

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

	float search_speed_mps{0.25f};

	float normal_speed_mps{0.85f};

	float approach_speed_mps{0.72f};

	float turning_speed_mps{0.65f};

	float lost_wall_speed_mps{0.25f};

	// The corner speed is capped by sqrt(max lateral acceleration * radius).
	// Raise this only after tyre-grip testing on the real competition surface.
	float max_lateral_acceleration_mps2{1.40f};

	// Output shaping removes LiDAR/line-fit jitter and prevents step commands.
	float steering_filter_time_constant_s{0.035f};
	float max_steering_rate_rad_s{7.0f};
	float max_acceleration_mps2{1.8f};
	float max_deceleration_mps2{3.0f};

	float nominal_update_period_s{0.05f};
	float min_update_period_s{0.005f};
	float max_update_period_s{0.12f};

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
		float heading_rad, float speed_mps, float dt_s, NavigationDebug &debug);

	NavigationCommand update_turning(
		float heading_rad, float speed_mps, float dt_s, NavigationDebug &debug);

	TrackWalls resolve_track_walls(const lidar::ResolvedWalls &walls) const;

	float calculate_cross_track_error(
		const lidar::LineSegment &outer_wall) const;

	float calculate_wall_heading_error(
		const lidar::LineSegment &outer_wall) const;

	bool should_start_turn(const lidar::ResolvedWalls &walls, float speed_mps,
		NavigationDebug &debug);

	void start_turn(float heading_rad);

	float calculate_turn_heading_error(float heading_rad) const;

	bool is_turn_complete(float heading_error_rad);

	float calculate_corner_speed_mps() const;

	float calculate_effective_turn_trigger_m(float speed_mps) const;

	float calculate_approach_speed_mps(
		float front_distance_m, float effective_trigger_m) const;

	float calculate_dt_s(std::uint64_t timestamp_us);

	NavigationCommand condition_command(
		const NavigationCommand &command, float dt_s, bool stop_immediately);

	static float smoothstep(float value);

	float calculate_search_steering(const lidar::ResolvedWalls &walls) const;

	float clamp_steering(float steering_rad) const;

	static float normalize_angle(float angle_rad);

  private:
	NavigationConfig config_;
	NavigationState state_;

	InitialDirectionEstimator direction_estimator_;

	control::StanleyController stanley_;

	std::uint64_t previous_timestamp_us_{0};

	float turn_start_heading_rad_{0.0f};
	float turn_reference_heading_rad_{0.0f};
	float turn_reference_progress_rad_{0.0f};
	float turn_total_angle_rad_{0.0f};
	float turn_heading_sign_{0.0f};
	float turn_entry_steering_rad_{0.0f};

	int turn_trigger_frames_{0};

	float conditioned_steering_rad_{0.0f};
	float conditioned_speed_mps_{0.0f};
	bool command_conditioner_initialized_{false};
};

} // namespace navigation
