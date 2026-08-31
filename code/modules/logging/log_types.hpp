#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>

#include "navigation_state.hpp"
#include "track_map.hpp"

namespace logging {

struct OutputSnapshot {
	std::int16_t wheel_rpm{0};
	std::uint16_t servo_pulse_us{1550};
};

struct TelemetryRow {
	std::uint64_t timestamp_us{0};
	int lap{0};
	std::size_t corner_index{0};
	std::string mode{"UNKNOWN"};

	// OTOS pose and measured planar speed in the world frame.
	float pos_x_m{0.0f};
	float pos_y_m{0.0f};
	float heading_rad{0.0f};
	float measured_speed_mps{0.0f};

	// Wall-following measurements and controller errors.
	float outer_distance_m{0.0f};
	float distance_error_m{0.0f};
	float wall_angle_rad{0.0f};
	float angle_error_rad{0.0f};
	bool outer_wall_valid{false};
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
	float target_acceleration_mps2{0.0f};
	float effective_turn_trigger_m{0.0f};
	float update_dt_s{0.0f};

	// Learned-map preview state used by replay navigation.
	bool map_preview_valid{false};
	bool map_approach_active{false};
	float map_distance_to_corner_m{0.0f};
	float map_confidence{0.0f};

	// Final navigation command and detected LiDAR-object count.
	float target_speed_mps{0.0f};
	float steering_rad{0.0f};
	std::size_t obstacle_count{0};

	// Values produced by the actuator conversion. They are empty for monitor-
	// only applications and do not represent an MCU acknowledgement.
	std::optional<std::int16_t> wheel_rpm;
	std::optional<std::uint16_t> servo_pulse_us;
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
	float pos_x_m{0.0f};
	float pos_y_m{0.0f};
	float heading_rad{0.0f};
	float segment_start_x_m{0.0f};
	float segment_start_y_m{0.0f};
	float segment_end_x_m{0.0f};
	float segment_end_y_m{0.0f};
	float wall_angle_rad{0.0f};
};

const char *telemetry_csv_header();
const char *corners_csv_header();
const char *walls_csv_header();

std::string to_csv_row(const TelemetryRow &row);
std::string to_csv_row(const CornerRow &row);
std::string to_csv_row(const WallRow &row);

const char *navigation_mode_name(navigation::NavigationMode mode);

TelemetryRow make_telemetry_row(std::uint64_t timestamp_us,
	const navigation::MapPose &pose, float measured_speed_mps,
	const navigation::NavigationState &state,
	const navigation::NavigationResult &result, std::size_t obstacle_count,
	const std::optional<OutputSnapshot> &output = std::nullopt);

} // namespace logging
