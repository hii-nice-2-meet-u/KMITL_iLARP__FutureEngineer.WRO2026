#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>

#include "navigation_state.hpp"
#include "track_map.hpp"

namespace logging {

// ---------------------------------------------------------------------------
// Missing-value convention (see docs OBSERVABILITY_AND_DATA_INTEGRITY.md §4)
//
// A numeric column of value 0 means "measured, and the measurement was zero".
// It NEVER means "not measured". Any group that can be absent carries a
// paired boolean flag, and that flag is written in every navigation mode --
// not only in the mode that happens to compute the value. Consumers must gate
// on the flag before plotting or averaging the number.
//
//   outer_distance_m / inner_distance_m / wall_angle_rad
//                                     -> outer_wall_valid / inner_wall_valid
//   distance_error_m / angle_error_rad -> wall_following_active
//   effective_turn_trigger_m, wall_corner_*
//                                     -> turn_trigger_evaluated
//
// Do not express absence as NaN: to_csv_row uses fixed/precision(6) and a
// "nan" token parses inconsistently across tools.
// ---------------------------------------------------------------------------

struct OutputSnapshot {
	std::int16_t wheel_rpm{0};
	std::uint16_t servo_pulse_us{1475};
	// Pre-step-limit servo command; equals servo_pulse_us unless
	// maximum_servo_step_us clipped this tick.
	std::uint16_t commanded_servo_pulse_us{1475};
};

// Raw OTOS output, before the control path reduces it.
//
// The controller consumes only hypot(vx, vy), which is unsigned: it cannot tell
// reversing from driving forward, and it discards the velocity direction that
// carries sideslip. getPosVelAcc() also returns acceleration and yaw rate every
// tick, both of which were previously read and thrown away. Yaw rate in
// particular is what makes the servo-pulse -> wheel-angle curve identifiable
// offline via delta = atan(wheelbase * yaw_rate / speed).
//
// pose_timestamp_us shares the steady_clock epoch with TimedLidarData, so
// (pose_timestamp_us - timestamp_us) is the real scan-to-pose skew. Both were
// previously logged under the scan timestamp alone, which reported two sensors
// read at different moments as if they were simultaneous.
struct OdometrySample {
	std::uint64_t pose_timestamp_us{0};
	float velocity_x_mps{0.0f};
	float velocity_y_mps{0.0f};
	float yaw_rate_rps{0.0f};
	float accel_x_mps2{0.0f};
	float accel_y_mps2{0.0f};
};

// Battery voltage, sampled at a low cadence rather than every tick:
// read_voltage_v() is a blocking SPI round-trip (~2 ms) and must not run at
// loop rate. The app holds the last successful reading between samples;
// sample_age_us reports how old it is at the tick it is logged with, so a
// stale hold is visible. valid is false until the first successful read, so a
// zero voltage is never mistaken for a real 0 V (see the missing-value
// convention above).
struct BatterySample {
	float voltage_v{0.0f};
	std::uint64_t sample_age_us{0};
	bool valid{false};
};

// Wall-clock time spent inside each per-tick processing stage, measured on the
// steady_clock at the app call sites. update_dt_s is the clamped loop period
// and raw_update_dt_s the pre-clamp period; neither says where the time went.
// lidar runs every tick, so lidar_process_us is always measured when this is
// supplied. camera runs only when a frame arrived (and never in the open app),
// so camera_process_valid distinguishes "ran fast" from "did not run".
struct StageTiming {
	std::uint32_t lidar_process_us{0};
	std::uint32_t camera_process_us{0};
	bool camera_process_valid{false};
};

struct TelemetryRow {
	// Monotonic per-run counter. AsyncCsvWriter drops the oldest row when its
	// queue is full and leaves no marker, so a gap in this column is the only
	// way to tell a dropped row from a slow loop iteration.
	std::uint64_t row_index{0};
	std::uint64_t timestamp_us{0};
	int lap{0};
	std::size_t corner_index{0};
	std::string mode{"UNKNOWN"};

	// OTOS pose and measured planar speed in the world frame.
	float pos_x_m{0.0f};
	float pos_y_m{0.0f};
	float heading_rad{0.0f};
	float measured_speed_mps{0.0f};

	// Raw OTOS channels; see OdometrySample. measured_speed_mps above is the
	// unsigned magnitude the controller actually used.
	std::uint64_t pose_timestamp_us{0};
	float otos_velocity_x_mps{0.0f};
	float otos_velocity_y_mps{0.0f};
	float otos_yaw_rate_rps{0.0f};
	float otos_accel_x_mps2{0.0f};
	float otos_accel_y_mps2{0.0f};

	// Battery, held from a ~1 Hz sample; see BatterySample. Gate on
	// battery_valid before reading battery_voltage_v.
	float battery_voltage_v{0.0f};
	std::uint64_t battery_sample_age_us{0};
	bool battery_valid{false};

	// Per-stage processing time; see StageTiming.
	std::uint32_t lidar_process_us{0};
	std::uint32_t camera_process_us{0};
	bool camera_process_valid{false};

	// Wall-following measurements and controller errors.
	float outer_distance_m{0.0f};
	float inner_distance_m{0.0f};
	float distance_error_m{0.0f};
	float wall_angle_rad{0.0f};
	float angle_error_rad{0.0f};
	bool outer_wall_valid{false};
	bool inner_wall_valid{false};
	bool corridor_center_active{false};
	bool wall_following_active{false};
	bool front_wall_valid{false};

	// Heading fallback and corner tracking state.
	bool heading_hold_active{false};
	float heading_tracking_error_rad{0.0f};
	float lost_wall_time_s{0.0f};
	float heading_error_rad{0.0f};
	float turn_progress{0.0f};
	float turn_feedforward_rad{0.0f};

	// Unfiltered and shaped navigation-controller outputs.
	float raw_steering_rad{0.0f};
	float raw_target_speed_mps{0.0f};
	float corner_speed_mps{0.0f};
	float replay_speed_factor{1.0f};
	float active_normal_speed_mps{0.0f};
	float active_approach_speed_mps{0.0f};
	float target_acceleration_mps2{0.0f};
	float effective_turn_trigger_m{0.0f};
	float update_dt_s{0.0f};
	float wall_corner_forward_m{0.0f};
	float wall_corner_lateral_m{0.0f};
	float wall_corner_stability_error_m{0.0f};
	int wall_corner_confirm_frames{0};
	bool wall_corner_candidate_valid{false};
	bool wall_corner_confirmed{false};
	bool wall_corner_trigger_active{false};
	bool front_wall_fallback_active{false};
	bool turn_trigger_evaluated{false};

	// Controller internals, so PID/Stanley gains can be tuned from a log
	// instead of only from the summed steering command.
	float stanley_cross_track_term_rad{0.0f};
	float stanley_heading_term_rad{0.0f};
	float stanley_heading_integral{0.0f};
	float turn_heading_pid_output_rad{0.0f};
	float turn_heading_pid_integral{0.0f};

	// Turn-trigger attribution and gate state.
	int turn_trigger_source{0};
	int turn_trigger_frames{0};
	bool turn_armed{false};
	bool replay_gate_suppressed{false};

	// Pre-clamp loop period. update_dt_s is clamped, so a stalled iteration is
	// invisible there.
	float raw_update_dt_s{0.0f};

	// Learned-map preview state used by replay navigation.
	bool map_preview_valid{false};
	bool map_approach_active{false};
	float map_distance_to_corner_m{0.0f};
	float map_confidence{0.0f};

	// Obstacle avoidance. Populated every tick by the obstacle-challenge app,
	// including ticks where avoidance is inactive, so an activation can be
	// read against the surrounding navigation state.
	bool obstacle_active{false};
	int obstacle_color{0};	   // 0 none, 1 RED, 2 GREEN
	int obstacle_pass_side{0}; // 0 none, 1 RIGHT, 2 LEFT
	float obstacle_forward_m{0.0f};
	float obstacle_right_m{0.0f};
	float obstacle_target_right_m{0.0f};
	float obstacle_steering_rad{0.0f};
	float obstacle_confidence{0.0f};
	float obstacle_world_x_m{0.0f};
	float obstacle_world_y_m{0.0f};

	// Camera/LiDAR fusion counters from PerceptionDiagnostics.
	int lidar_valid_count{0};
	int camera_valid_count{0};
	int matched_count{0};
	int frame_confirmed_count{0};
	bool camera_time_synchronized{false};

	// Per-scan LiDAR point-rejection tally from ProcessedLidarData. Makes a
	// mis-set quality/range threshold visible without persisting the raw scan.
	int lidar_points_total{0};
	int lidar_points_rejected_quality{0};
	int lidar_points_rejected_range{0};

	// Heading correction (heading_rad - target_heading_rad) applied to wall
	// resolution before classification. wall_angle_rad and walls.csv are in
	// this corrected frame, not the raw sensor frame; subtract this to recover
	// raw geometry offline.
	float wall_correction_rad{0.0f};

	// Final navigation command and detected LiDAR-object count.
	float target_speed_mps{0.0f};
	float steering_rad{0.0f};
	std::size_t obstacle_count{0};

	// Values produced by the actuator conversion. They are empty for monitor-
	// only applications and do not represent an MCU acknowledgement.
	std::optional<std::int16_t> wheel_rpm;
	std::optional<std::uint16_t> servo_pulse_us;
	std::optional<std::uint16_t> commanded_servo_pulse_us;
};

struct CornerRow {
	std::size_t corner_index{0};
	bool entry_valid{false};
	bool exit_valid{false};
	float entry_x_m{0.0f};
	float entry_y_m{0.0f};
	float entry_heading_rad{0.0f};
	float exit_x_m{0.0f};
	float exit_y_m{0.0f};
	float exit_heading_rad{0.0f};
	float preferred_turn_trigger_m{0.0f};
	float preferred_corner_radius_m{0.0f};
	float safe_speed_mps{0.0f};
	float confidence{0.0f};
	int entry_observations{0};
	int exit_observations{0};
};

struct WallRow {
	std::uint64_t timestamp_us{0};
	const char *wall_role{"LEFT"};
	// Wall geometry is recorded in every mode, so the reader can filter rather
	// than being handed a file that silently contains only straights.
	const char *mode{"UNKNOWN"};
	float pos_x_m{0.0f};
	float pos_y_m{0.0f};
	float heading_rad{0.0f};
	float segment_start_x_m{0.0f};
	float segment_start_y_m{0.0f};
	float segment_end_x_m{0.0f};
	float segment_end_y_m{0.0f};
	float wall_angle_rad{0.0f};
};

struct SegmentRow {
	std::uint64_t timestamp_us{0};
	std::uint64_t row_index{0};
	const char *mode{"UNKNOWN"};
	std::size_t segment_index{0};
	float start_x_m{0.0f};
	float start_y_m{0.0f};
	float end_x_m{0.0f};
	float end_y_m{0.0f};
	float angle_rad{0.0f};
	float length_m{0.0f};
	const char *role{"NONE"};
};

const char *telemetry_csv_header();
const char *corners_csv_header();
const char *walls_csv_header();
const char *segments_csv_header();

std::string to_csv_row(const TelemetryRow &row);
std::string to_csv_row(const CornerRow &row);
std::string to_csv_row(const WallRow &row);
std::string to_csv_row(const SegmentRow &row);

const char *navigation_mode_name(navigation::NavigationMode mode);

TelemetryRow make_telemetry_row(std::uint64_t timestamp_us,
	const navigation::MapPose &pose, float measured_speed_mps,
	const navigation::NavigationState &state,
	const navigation::NavigationResult &result, std::size_t obstacle_count,
	const std::optional<OutputSnapshot> &output = std::nullopt,
	const std::optional<OdometrySample> &odometry = std::nullopt,
	const std::optional<BatterySample> &battery = std::nullopt,
	const std::optional<StageTiming> &timing = std::nullopt);

} // namespace logging
