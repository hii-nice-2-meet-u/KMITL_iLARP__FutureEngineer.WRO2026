#pragma once

#include <cstddef>
#include <optional>

#include "direction.hpp"

namespace navigation {

enum class NavigationMode { SEARCH_DIRECTION, NORMAL, TURNING, FINISHED };

// Which mechanism actually decided to start the current turn. Logged as a
// small integer so a run can be attributed to a trigger source offline
// instead of being inferred from a combination of booleans.
enum class TurnTriggerSource {
	NONE = 0,
	INNER_CORNER = 1,	// confirmed wall-corner landmark (preferred)
	FRONT_FALLBACK = 2, // front wall, close-range safety fallback
	LEGACY_FRONT = 3	// front wall, no pose available for the landmark
};

struct NavigationCommand {
	// SI-unit setpoints for the downstream actuator controller.
	// speed: non-negative forward speed [m/s]
	// steering: negative = left, positive = right [rad]
	float target_speed_mps{0.0f};
	float steering_rad{0.0f};

	// Path curvature [1/m]. Negative curves left, positive right, matching the
	// steering sign. During the delta->kappa migration the controllers still
	// compute steering_rad and set this from it at the boundary; the actuator
	// conditioning converts it back. It is the same information as steering_rad
	// expressed in the representation that superposes linearly.
	float curvature_1pm{0.0f};
};

struct NavigationDebug {
	// Wall following
	float outer_distance_m{0.0f};
	float inner_distance_m{0.0f};
	float distance_error_m{0.0f};

	float wall_angle_rad{0.0f};
	float angle_error_rad{0.0f};

	// Turning
	float heading_error_rad{0.0f};
	float heading_tracking_error_rad{0.0f};
	float turn_reference_heading_rad{0.0f};
	float turn_progress{0.0f};
	float turn_feedforward_rad{0.0f};

	// Command shaping
	float raw_steering_rad{0.0f};
	float raw_target_speed_mps{0.0f};
	float corner_speed_mps{0.0f};
	float effective_turn_trigger_m{0.0f};
	float update_dt_s{0.0f};
	float map_distance_to_corner_m{0.0f};
	float map_confidence{0.0f};
	float lost_wall_time_s{0.0f};
	float wall_corner_forward_m{0.0f};
	float wall_corner_lateral_m{0.0f};
	float wall_corner_stability_error_m{0.0f};
	float replay_speed_factor{1.0f};
	float active_normal_speed_mps{0.0f};
	float active_approach_speed_mps{0.0f};
	int wall_corner_confirm_frames{0};

	// Corner-planner shadow instrumentation (redesign C-2). Computed every tick
	// the confirmed corner landmark and pose are available, whether or not the
	// planner is actuating (use_corner_planner), so corner_plan.csv can validate
	// the racing line / apex offset from real logs before the planner is enabled.
	// corner_plan_active is true only on ticks it actually drove the actuator.
	bool corner_plan_valid{false};
	bool corner_plan_active{false};
	float corner_plan_curvature_1pm{0.0f};
	float corner_plan_apex_forward_m{0.0f};
	float corner_plan_apex_lateral_m{0.0f};
	float corner_plan_path_offset_m{0.0f};
	float corner_plan_corner_world_x_m{0.0f};
	float corner_plan_corner_world_y_m{0.0f};

	// Controller internals. These are what make the gains tunable from a log:
	// the summed command alone cannot separate a cross-track correction from a
	// heading correction of the opposite sign.
	float stanley_cross_track_term_rad{0.0f};
	float stanley_heading_term_rad{0.0f};
	float stanley_heading_integral{0.0f};
	float turn_heading_pid_output_rad{0.0f};
	float turn_heading_pid_integral{0.0f};

	// Turn-trigger diagnostics.
	TurnTriggerSource turn_trigger_source{TurnTriggerSource::NONE};
	int turn_trigger_frames{0};
	bool turn_armed{false};
	bool replay_gate_suppressed{false};

	// Loop timing before clamping to [min_update_period_s, max_update_period_s].
	// update_dt_s is the clamped value the controllers actually integrate with;
	// a stalled iteration is indistinguishable from a healthy one there.
	float raw_update_dt_s{0.0f};

	// Measurement flags. These describe what the LiDAR resolved this tick and
	// are populated in EVERY mode, so a zero distance means "measured zero",
	// never "this mode does not compute it". Gate every distance column on the
	// matching flag before plotting it.
	bool outer_wall_valid{false};
	bool inner_wall_valid{false};
	bool corridor_center_active{false};
	bool front_wall_valid{false};

	// Control-state flags. Unlike the measurement flags above, these say which
	// computation actually ran, so a reader can tell a real controller input
	// from a value that was never calculated on this tick.
	//
	// wall_following_active : distance_error_m / angle_error_rad hold live
	//                         Stanley inputs (NORMAL, outer wall resolved).
	// turn_trigger_evaluated: effective_turn_trigger_m and the wall_corner_*
	//                         group were evaluated (should_start_turn ran).
	bool wall_following_active{false};
	bool turn_trigger_evaluated{false};
	bool heading_hold_active{false};
	bool map_preview_valid{false};
	bool map_approach_active{false};
	bool wall_corner_candidate_valid{false};
	bool wall_corner_confirmed{false};
	bool wall_corner_trigger_active{false};
	bool front_wall_fallback_active{false};
};

struct NavigationState {
	NavigationMode mode{NavigationMode::SEARCH_DIRECTION};

	std::optional<DrivingDirection> direction;

	// Heading target of current straight / turn
	float target_heading_rad{0.0f};

	// Turn state
	bool turn_armed{true};
	int heading_confirm_frames{0};

	// Track progress
	int turn_count{0};
	int lap{0};
	std::size_t corner_index{0};
};

struct NavigationResult {
	NavigationCommand command;
	NavigationDebug debug;
};

} // namespace navigation
