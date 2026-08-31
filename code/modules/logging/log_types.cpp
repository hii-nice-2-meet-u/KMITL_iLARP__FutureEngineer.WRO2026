#include "log_types.hpp"

#include <sstream>

namespace logging {

namespace {

template <typename Value>
void write_optional(
	std::ostringstream &stream, const std::optional<Value> &value) {
	if (value.has_value()) {
		stream << *value;
	}
}

} // namespace

const char *telemetry_csv_header() {
	return "timestamp_us,lap,corner_index,mode,"
		   "pos_x_m,pos_y_m,heading_rad,measured_speed_mps,"
		   "outer_distance_m,distance_error_m,wall_angle_rad,angle_error_rad,"
		   "outer_wall_valid,front_wall_valid,heading_hold_active,"
		   "heading_tracking_error_rad,lost_wall_time_s,heading_error_rad,"
		   "turn_progress,turn_feedforward_rad,raw_steering_rad,"
		   "raw_target_speed_mps,corner_speed_mps,target_acceleration_mps2,"
		   "effective_turn_trigger_m,update_dt_s,map_preview_valid,"
		   "map_approach_active,map_distance_to_corner_m,map_confidence,"
		   "target_speed_mps,steering_rad,obstacle_count,"
		   "wheel_rpm,servo_pulse_us";
}

const char *corners_csv_header() {
	return "corner_index,entry_valid,exit_valid,entry_x_m,entry_y_m,"
		   "entry_heading_rad,exit_x_m,exit_y_m,exit_heading_rad,"
		   "preferred_turn_trigger_m,preferred_corner_radius_m,safe_speed_mps,"
		   "confidence,entry_observations,exit_observations";
}

const char *walls_csv_header() {
	return "timestamp_us,wall_role,pos_x_m,pos_y_m,heading_rad,"
		   "segment_start_x_m,segment_start_y_m,segment_end_x_m,"
		   "segment_end_y_m,wall_angle_rad";
}

std::string to_csv_row(const TelemetryRow &row) {
	std::ostringstream stream;
	stream.setf(std::ios::fixed);
	stream.precision(6);
	stream << row.timestamp_us << ',' << row.lap << ',' << row.corner_index
		   << ',' << row.mode << ',' << row.pos_x_m << ',' << row.pos_y_m << ','
		   << row.heading_rad << ',' << row.measured_speed_mps << ','
		   << row.outer_distance_m << ',' << row.distance_error_m << ','
		   << row.wall_angle_rad << ',' << row.angle_error_rad << ','
		   << (row.outer_wall_valid ? 1 : 0) << ','
		   << (row.front_wall_valid ? 1 : 0) << ','
		   << (row.heading_hold_active ? 1 : 0) << ','
		   << row.heading_tracking_error_rad << ',' << row.lost_wall_time_s
		   << ',' << row.heading_error_rad << ',' << row.turn_progress << ','
		   << row.turn_feedforward_rad << ',' << row.raw_steering_rad << ','
		   << row.raw_target_speed_mps << ',' << row.corner_speed_mps << ','
		   << row.target_acceleration_mps2 << ','
		   << row.effective_turn_trigger_m << ',' << row.update_dt_s << ','
		   << (row.map_preview_valid ? 1 : 0) << ','
		   << (row.map_approach_active ? 1 : 0) << ','
		   << row.map_distance_to_corner_m << ',' << row.map_confidence << ','
		   << row.target_speed_mps << ',' << row.steering_rad << ','
		   << row.obstacle_count << ',';
	write_optional(stream, row.wheel_rpm);
	stream << ',';
	write_optional(stream, row.servo_pulse_us);
	return stream.str();
}

std::string to_csv_row(const CornerRow &row) {
	std::ostringstream stream;
	stream.setf(std::ios::fixed);
	stream.precision(6);
	stream << row.corner_index << ',' << (row.entry_valid ? 1 : 0) << ','
		   << (row.exit_valid ? 1 : 0) << ',' << row.entry_x_m << ','
		   << row.entry_y_m << ',' << row.entry_heading_rad << ','
		   << row.exit_x_m << ',' << row.exit_y_m << ',' << row.exit_heading_rad
		   << ',' << row.preferred_turn_trigger_m << ','
		   << row.preferred_corner_radius_m << ',' << row.safe_speed_mps << ','
		   << row.confidence << ',' << row.entry_observations << ','
		   << row.exit_observations;
	return stream.str();
}

std::string to_csv_row(const WallRow &row) {
	std::ostringstream stream;
	stream.setf(std::ios::fixed);
	stream.precision(6);
	stream << row.timestamp_us << ',' << row.wall_role << ',' << row.pos_x_m
		   << ',' << row.pos_y_m << ',' << row.heading_rad << ','
		   << row.segment_start_x_m << ',' << row.segment_start_y_m << ','
		   << row.segment_end_x_m << ',' << row.segment_end_y_m << ','
		   << row.wall_angle_rad;
	return stream.str();
}

const char *navigation_mode_name(navigation::NavigationMode mode) {
	switch (mode) {
	case navigation::NavigationMode::SEARCH_DIRECTION:
		return "SEARCH_DIRECTION";
	case navigation::NavigationMode::NORMAL:
		return "NORMAL";
	case navigation::NavigationMode::TURNING:
		return "TURNING";
	case navigation::NavigationMode::FINISHED:
		return "FINISHED";
	}
	return "UNKNOWN";
}

TelemetryRow make_telemetry_row(std::uint64_t timestamp_us,
	const navigation::MapPose &pose, float measured_speed_mps,
	const navigation::NavigationState &state,
	const navigation::NavigationResult &result, std::size_t obstacle_count,
	const std::optional<OutputSnapshot> &output) {
	TelemetryRow row;
	row.timestamp_us = timestamp_us;
	row.lap = state.lap;
	row.corner_index = state.corner_index;
	row.mode = navigation_mode_name(state.mode);
	row.pos_x_m = pose.x_m;
	row.pos_y_m = pose.y_m;
	row.heading_rad = pose.heading_rad;
	row.measured_speed_mps = measured_speed_mps;
	row.outer_distance_m = result.debug.outer_distance_m;
	row.distance_error_m = result.debug.distance_error_m;
	row.wall_angle_rad = result.debug.wall_angle_rad;
	row.angle_error_rad = result.debug.angle_error_rad;
	row.outer_wall_valid = result.debug.outer_wall_valid;
	row.front_wall_valid = result.debug.front_wall_valid;
	row.heading_hold_active = result.debug.heading_hold_active;
	row.heading_tracking_error_rad = result.debug.heading_tracking_error_rad;
	row.lost_wall_time_s = result.debug.lost_wall_time_s;
	row.heading_error_rad = result.debug.heading_error_rad;
	row.turn_progress = result.debug.turn_progress;
	row.turn_feedforward_rad = result.debug.turn_feedforward_rad;
	row.raw_steering_rad = result.debug.raw_steering_rad;
	row.raw_target_speed_mps = result.debug.raw_target_speed_mps;
	row.corner_speed_mps = result.debug.corner_speed_mps;
	row.target_acceleration_mps2 = result.debug.target_acceleration_mps2;
	row.effective_turn_trigger_m = result.debug.effective_turn_trigger_m;
	row.update_dt_s = result.debug.update_dt_s;
	row.map_preview_valid = result.debug.map_preview_valid;
	row.map_approach_active = result.debug.map_approach_active;
	row.map_distance_to_corner_m = result.debug.map_distance_to_corner_m;
	row.map_confidence = result.debug.map_confidence;
	row.target_speed_mps = result.command.target_speed_mps;
	row.steering_rad = result.command.steering_rad;
	row.obstacle_count = obstacle_count;

	if (output.has_value()) {
		row.wheel_rpm = output->wheel_rpm;
		row.servo_pulse_us = output->servo_pulse_us;
	}

	return row;
}

} // namespace logging
