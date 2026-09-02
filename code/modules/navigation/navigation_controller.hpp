#pragma once

#include <cstdint>

#include "init_direction.hpp"
#include "kinematics.hpp"
#include "lidar_processor.hpp"
#include "navigation_state.hpp"
#include "pid.hpp"
#include "stanley_controller.hpp"
#include "track_map.hpp"

namespace navigation {

struct NavigationConfig {

	float target_outer_distance_m{0.30f};
	bool follow_corridor_center{true};

	control::StanleyConfig stanley{};

	// Mode-independent steering saturation. clamp_steering() enforces this in
	// SEARCH, TURNING, and the final command conditioning -- none of which
	// involve the Stanley controller -- so it must not be scoped to
	// StanleyConfig (audit F-11). Defaults to the same 38 deg the Stanley limit
	// used, so behaviour is unchanged until it is tuned independently.
	float max_steering_rad{38.0f * 3.14159265358979323846f / 180.0f};

	// Center robot while direction is unknown.
	float search_center_kp{1.5f};
	bool search_preserve_initial_offset{false};
	float search_front_slowdown_distance_m{0.70f};
	float search_front_minimum_distance_m{0.20f};
	float search_minimum_speed_mps{0.06f};

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

	// Prefer a physical inner-wall endpoint over a field-width-dependent front
	// wall threshold. The front wall remains a close-range safety fallback.
	bool use_wall_corner_trigger{true};
	float front_wall_fallback_distance_m{0.40f};

	// Corner-strategy redesign (C-2). When true, TURNING steers along a curvature
	// re-planned every tick to the *measured* corner point (corner_planner.hpp)
	// instead of the fixed-radius open-loop arc, the confirmed landmark is kept
	// alive through the corner (C-1), and the learned-map veto is replaced by
	// live-wins (C-3). MUST stay false until curvature_gain is measured (M-4) and
	// a C-0 baseline exists: with it off the corner behaviour is byte-identical
	// to the legacy path. See docs/CORNER_STRATEGY_REDESIGN.md.
	bool use_corner_planner{false};

	// Lateral distance from the measured corner point to the planned path apex.
	// Encodes the racing line; validate in the sim before enabling the planner.
	float corner_planner_path_offset_m{0.30f};

	// Run the planner in shadow every tick (compute + log, do NOT actuate) so
	// corner_plan.csv captures what it would do, next to what the legacy path
	// actually did. Independent of use_corner_planner and behaviour-neutral: it
	// only fills debug fields and keeps the confirmed landmark alive through the
	// corner for the log. Set false to drop the extra per-tick computation.
	bool log_corner_plan{true};

	// LiDAR origin relative to rear-axle center in the robot frame.
	// +right points right and +forward points toward the front of the robot.
	float lidar_lateral_offset_m{0.0f};
	float lidar_forward_offset_m{0.0f};

	// Signed longitudinal displacement from the detected wall corner to the
	// virtual intersection of the incoming and outgoing vehicle paths.
	float wall_corner_to_path_offset_m{0.0f};

	// Local LiDAR gates for rejecting endpoints behind the robot or too far away
	// to be a useful corner observation.
	float wall_corner_min_forward_m{0.08f};
	float wall_corner_max_forward_m{1.50f};

	// Reject short line-fit fragments before treating an endpoint as a corner.
	float wall_corner_min_inner_length_m{0.20f};

	// A new endpoint must remain within the stability gate in the OTOS world
	// frame for the configured number of frames. Once confirmed, the wider
	// association gate tolerates normal LiDAR endpoint noise.
	float wall_corner_stability_tolerance_m{0.05f};
	float wall_corner_association_distance_m{0.12f};
	float wall_corner_filter_weight{0.25f};

	// A collinear segment extending beyond the selected endpoint means the inner
	// wall continues forward, so that endpoint is not the physical wall corner.
	float wall_corner_collinear_angle_rad{
		8.0f * 3.14159265358979323846f / 180.0f};
	float wall_corner_collinear_offset_m{0.05f};
	float wall_corner_continuation_gap_m{0.20f};

	int wall_corner_confirm_frames{4};
	int wall_corner_max_missed_frames{3};

	// ---------------------------------------------------------
	// Smooth geometric corner trajectory
	// ---------------------------------------------------------

	// Measure wheelbase from rear-axle center to front-axle center.
	float wheelbase_m{0.16375f};

	// Ratio of achieved curvature to the Ackermann prediction, fed to the shared
	// BicycleModel. 1.0 is the textbook model; this chassis achieves MORE (the
	// fixed rear axle scrubs), so at 1.0 the feed-forward under-steers the model
	// and the real turn radius comes out too tight -- the corners cut inside and
	// the position drifts (seen in LOG_5_NOOB and reproduced in sim_track). Raise
	// it toward the measured value so the planned radius matches the real one.
	// Refine with measure --speed-sweep (M-4).
	float curvature_gain{1.0f};

	// WRO outer corners have a 0.10 m wall radius. A vehicle path 0.30 m
	// inside that wall therefore starts with an approximately 0.40 m radius.
	float corner_radius_m{0.40f};
	float corner_clothoid_ramp_m{0.10f};

	float turn_entry_blend_rad{10.0f * 3.14159265358979323846f / 180.0f};

	float turn_exit_blend_rad{22.0f * 3.14159265358979323846f / 180.0f};

	float exit_acceleration_blend_rad{15.0f * 3.14159265358979323846f / 180.0f};

	// Tracks a moving heading reference around the geometric corner. Unlike
	// targeting the final 90-degree heading immediately, this avoids saturating
	// the steering at corner entry.

	// PID correction for the moving heading reference during a corner.
	// The output limits must leave room under the steering clamp once the
	// corner feed-forward atan2(wheelbase_m, corner_radius_m) is added, or the
	// composite clamp binds first and this correction has no effect. At the
	// configured geometry the feed-forward is 20 deg and the clamp is 38 deg,
	// so +/-15 deg keeps the sum (35 deg) inside the clamp.
	control::PIDConfig turn_heading_pid{
		0.80f, 0.08f, 0.020f, -0.261799f, 0.261799f, -0.50f, 0.50f, 0.10f};

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

	// Main2 can enable these after its lap-one map becomes available. Straight
	// speed gets the full factor, approach speed gets only the configured share,
	// and turning speed remains independently limited.
	bool enable_replay_speed_factors{false};
	float lap2_speed_factor{1.20f};
	float lap3_speed_factor{1.30f};
	float replay_approach_factor_weight{0.50f};
	float maximum_replay_speed_mps{0.70f};
	float replay_turn_gate_distance_m{0.40f};
	float replay_front_safety_override_distance_m{0.25f};

	// Briefly hold the last OTOS heading when the outer wall disappears.
	// Longer losses fall back to lost_wall_speed_mps with zero steering.
	float max_heading_hold_s{0.30f};

	// The corner speed is capped by sqrt(max lateral acceleration * radius).
	// Raise this only after tyre-grip testing on the real competition surface.
	// Conservative p90 from the available speed-sweep measurement.
	float max_lateral_acceleration_mps2{1.097f};

	// Output shaping removes LiDAR/line-fit jitter and prevents step commands.
	float steering_filter_time_constant_s{0.035f};
	float max_steering_rate_rad_s{7.0f};
	// Pi-side actuator pulse guard. The limiter is applied once in κ-domain;
	// these values mirror ActuatorConfig and are recorded per run.
	std::uint16_t servo_min_pulse_us{900};
	std::uint16_t servo_center_pulse_us{1475};
	std::uint16_t servo_max_pulse_us{2100};
	std::uint16_t maximum_servo_step_us{500};
	float maximum_steering_command_deg{45.0f};
	float max_acceleration_mps2{5.0f};
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
		float heading_rad, float speed_mps,
		const std::optional<ReplayHint> &replay_hint = std::nullopt,
		const std::optional<MapPose> &map_pose = std::nullopt);

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
		float heading_rad, float speed_mps, float dt_s,
		const std::optional<ReplayHint> &replay_hint,
		const std::optional<MapPose> &map_pose, NavigationDebug &debug);

	NavigationCommand update_turning(float heading_rad, float speed_mps,
		float dt_s, const std::optional<MapPose> &map_pose,
		NavigationDebug &debug);

	TrackWalls resolve_track_walls(const lidar::ResolvedWalls &walls) const;

	// Reporting only. Fills the wall measurement group before the mode switch
	// so a non-NORMAL tick logs what the LiDAR actually saw instead of a
	// default-constructed zero. update_normal() overwrites these with the
	// values it feeds to Stanley, so NORMAL rows are unchanged. Nothing in the
	// control path reads what this writes.
	void populate_wall_observation(
		const lidar::ProcessedLidarData &lidar_data, NavigationDebug &debug) const;

	float calculate_cross_track_error(
		const lidar::LineSegment &outer_wall) const;
	float calculate_center_cross_track_error(
		const lidar::LineSegment &inner_wall,
		const lidar::LineSegment &outer_wall) const;

	float calculate_wall_heading_error(
		const lidar::LineSegment &outer_wall) const;

	bool should_start_turn(const lidar::ProcessedLidarData &lidar_data,
		float speed_mps, const std::optional<MapPose> &map_pose,
		const std::optional<ReplayHint> &replay_hint, NavigationDebug &debug);

	void update_wall_corner_landmark(
		const lidar::ProcessedLidarData &lidar_data,
		const std::optional<MapPose> &map_pose, NavigationDebug &debug);

	// Shadow corner-planner instrumentation: fills the corner_plan_* debug group
	// from the confirmed landmark + pose, without touching the command.
	void populate_corner_plan_debug(
		const std::optional<MapPose> &map_pose, NavigationDebug &debug) const;

	std::optional<cv::Point2f> find_inner_wall_corner_candidate(
		const lidar::ProcessedLidarData &lidar_data) const;

	bool has_forward_wall_continuation(const lidar::LineSegment &inner_wall,
		const cv::Point2f &forward_endpoint,
		const std::vector<lidar::LineSegment> &segments) const;

	void reset_wall_corner_tracker();

	float calculate_geometric_turn_trigger_m(float speed_mps) const;

	float calculate_front_wall_fallback_trigger_m(float speed_mps) const;

	void start_turn(float heading_rad);

	float calculate_turn_heading_error(float heading_rad) const;

	bool is_turn_complete(float heading_error_rad);

	float calculate_corner_speed_mps() const;

	float calculate_replay_speed_factor() const;

	float calculate_active_normal_speed_mps() const;

	float calculate_active_approach_speed_mps() const;

	float calculate_effective_turn_trigger_m(float speed_mps) const;

	float calculate_approach_speed_mps(
		float front_distance_m, float effective_trigger_m) const;

	float calculate_dt_s(std::uint64_t timestamp_us);

	NavigationCommand condition_command(const NavigationCommand &command,
		float dt_s, bool stop_immediately);

	static float smoothstep(float value);

	float calculate_search_steering(const lidar::ResolvedWalls &walls);

	float clamp_steering(float steering_rad) const;

	static float normalize_angle(float angle_rad);

  private:
	NavigationConfig config_;
	NavigationState state_;

	// Single kinematic model for the whole controller. curvature_gain stays 1.0
	// until the M-4 floor measurement; wheelbase mirrors the config so there is
	// one source of truth.
	kinematics::BicycleModel model_;

	InitialDirectionEstimator direction_estimator_;

	control::StanleyController stanley_;
	control::PID turn_heading_pid_;

	std::uint64_t previous_timestamp_us_{0};
	float last_elapsed_update_s_{0.0f};
	float last_raw_update_dt_s_{0.0f};

	float turn_start_heading_rad_{0.0f};
	float turn_reference_heading_rad_{0.0f};
	float turn_reference_progress_rad_{0.0f};
	float turn_total_angle_rad_{0.0f};
	float turn_reference_distance_m_{0.0f};
	float turn_heading_sign_{0.0f};
	float turn_entry_steering_rad_{0.0f};

	int turn_trigger_frames_{0};

	std::optional<cv::Point2f> wall_corner_anchor_world_;
	std::optional<cv::Point2f> wall_corner_filtered_world_;
	int wall_corner_stable_frames_{0};
	int wall_corner_missed_frames_{0};
	bool wall_corner_confirmed_{false};
	bool replay_speed_active_{false};
	float search_initial_center_error_m_{0.0f};
	bool search_initial_center_error_valid_{false};

	float last_valid_wall_heading_rad_{0.0f};
	float lost_wall_timer_s_{0.0f};
	bool has_last_valid_wall_heading_{false};
	bool outer_wall_was_valid_{false};

	float conditioned_steering_rad_{0.0f};
	float conditioned_speed_mps_{0.0f};
	bool command_conditioner_initialized_{false};
};

} // namespace navigation
