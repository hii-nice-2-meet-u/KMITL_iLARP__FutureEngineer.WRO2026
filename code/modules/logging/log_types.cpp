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
	return "row_index,timestamp_us,lap,corner_index,mode,"
		   "pos_x_m,pos_y_m,heading_rad,measured_speed_mps,"
		   "pose_timestamp_us,otos_velocity_x_mps,otos_velocity_y_mps,"
		   "otos_yaw_rate_rps,otos_accel_x_mps2,otos_accel_y_mps2,"
		   "outer_distance_m,inner_distance_m,distance_error_m,wall_angle_rad,"
		   "angle_error_rad,outer_wall_valid,inner_wall_valid,"
		   "corridor_center_active,wall_following_active,front_wall_valid,"
		   "heading_hold_active,"
		   "heading_tracking_error_rad,lost_wall_time_s,heading_error_rad,"
		   "turn_progress,turn_feedforward_rad,raw_steering_rad,"
		   "raw_target_speed_mps,corner_speed_mps,replay_speed_factor,"
		   "active_normal_speed_mps,active_approach_speed_mps,"
		   "target_acceleration_mps2,"
		   "effective_turn_trigger_m,update_dt_s,wall_corner_forward_m,"
		   "wall_corner_lateral_m,wall_corner_stability_error_m,"
		   "wall_corner_confirm_frames,wall_corner_candidate_valid,"
		   "wall_corner_confirmed,wall_corner_trigger_active,"
		   "front_wall_fallback_active,turn_trigger_evaluated,"
		   "stanley_cross_track_term_rad,stanley_heading_term_rad,"
		   "stanley_heading_integral,turn_heading_pid_output_rad,"
		   "turn_heading_pid_integral,turn_trigger_source,"
		   "turn_trigger_frames,turn_armed,replay_gate_suppressed,"
		   "raw_update_dt_s,map_preview_valid,"
		   "map_approach_active,map_distance_to_corner_m,map_confidence,"
		   "obstacle_active,obstacle_color,obstacle_pass_side,"
		   "obstacle_forward_m,obstacle_right_m,obstacle_target_right_m,"
		   "obstacle_steering_rad,obstacle_confidence,"
		   "obstacle_world_x_m,obstacle_world_y_m,"
		   "lidar_valid_count,camera_valid_count,matched_count,"
		   "frame_confirmed_count,camera_time_synchronized,"
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
	return "timestamp_us,wall_role,mode,pos_x_m,pos_y_m,heading_rad,"
		   "segment_start_x_m,segment_start_y_m,segment_end_x_m,"
		   "segment_end_y_m,wall_angle_rad";
}

std::string to_csv_row(const TelemetryRow &row) {
	std::ostringstream stream;
	stream.setf(std::ios::fixed);
	stream.precision(6);
	stream << row.row_index << ',' << row.timestamp_us << ',' << row.lap
		   << ',' << row.corner_index
		   << ',' << row.mode << ',' << row.pos_x_m << ',' << row.pos_y_m << ','
		   << row.heading_rad << ',' << row.measured_speed_mps << ','
		   << row.pose_timestamp_us << ',' << row.otos_velocity_x_mps << ','
		   << row.otos_velocity_y_mps << ',' << row.otos_yaw_rate_rps << ','
		   << row.otos_accel_x_mps2 << ',' << row.otos_accel_y_mps2 << ','
		   << row.outer_distance_m << ',' << row.inner_distance_m << ','
		   << row.distance_error_m << ','
		   << row.wall_angle_rad << ',' << row.angle_error_rad << ','
		   << (row.outer_wall_valid ? 1 : 0) << ','
		   << (row.inner_wall_valid ? 1 : 0) << ','
		   << (row.corridor_center_active ? 1 : 0) << ','
		   << (row.wall_following_active ? 1 : 0) << ','
		   << (row.front_wall_valid ? 1 : 0) << ','
		   << (row.heading_hold_active ? 1 : 0) << ','
		   << row.heading_tracking_error_rad << ',' << row.lost_wall_time_s
		   << ',' << row.heading_error_rad << ',' << row.turn_progress << ','
		   << row.turn_feedforward_rad << ',' << row.raw_steering_rad << ','
		   << row.raw_target_speed_mps << ',' << row.corner_speed_mps << ','
		   << row.replay_speed_factor << ',' << row.active_normal_speed_mps << ','
		   << row.active_approach_speed_mps << ','
		   << row.target_acceleration_mps2 << ','
		   << row.effective_turn_trigger_m << ',' << row.update_dt_s << ','
		   << row.wall_corner_forward_m << ',' << row.wall_corner_lateral_m
		   << ',' << row.wall_corner_stability_error_m << ','
		   << row.wall_corner_confirm_frames << ','
		   << (row.wall_corner_candidate_valid ? 1 : 0) << ','
		   << (row.wall_corner_confirmed ? 1 : 0) << ','
		   << (row.wall_corner_trigger_active ? 1 : 0) << ','
		   << (row.front_wall_fallback_active ? 1 : 0) << ','
		   << (row.turn_trigger_evaluated ? 1 : 0) << ','
		   << row.stanley_cross_track_term_rad << ','
		   << row.stanley_heading_term_rad << ','
		   << row.stanley_heading_integral << ','
		   << row.turn_heading_pid_output_rad << ','
		   << row.turn_heading_pid_integral << ','
		   << row.turn_trigger_source << ',' << row.turn_trigger_frames << ','
		   << (row.turn_armed ? 1 : 0) << ','
		   << (row.replay_gate_suppressed ? 1 : 0) << ','
		   << row.raw_update_dt_s << ','
		   << (row.map_preview_valid ? 1 : 0) << ','
		   << (row.map_approach_active ? 1 : 0) << ','
		   << row.map_distance_to_corner_m << ',' << row.map_confidence << ','
		   << (row.obstacle_active ? 1 : 0) << ',' << row.obstacle_color
		   << ',' << row.obstacle_pass_side << ','
		   << row.obstacle_forward_m << ',' << row.obstacle_right_m << ','
		   << row.obstacle_target_right_m << ','
		   << row.obstacle_steering_rad << ',' << row.obstacle_confidence
		   << ',' << row.obstacle_world_x_m << ',' << row.obstacle_world_y_m
		   << ',' << row.lidar_valid_count << ',' << row.camera_valid_count
		   << ',' << row.matched_count << ',' << row.frame_confirmed_count
		   << ',' << (row.camera_time_synchronized ? 1 : 0) << ','
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
	stream << row.timestamp_us << ',' << row.wall_role << ',' << row.mode << ','
		   << row.pos_x_m
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
	const std::optional<OutputSnapshot> &output,
	const std::optional<OdometrySample> &odometry) {
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
	row.inner_distance_m = result.debug.inner_distance_m;
	row.distance_error_m = result.debug.distance_error_m;
	row.wall_angle_rad = result.debug.wall_angle_rad;
	row.angle_error_rad = result.debug.angle_error_rad;
	row.outer_wall_valid = result.debug.outer_wall_valid;
	row.inner_wall_valid = result.debug.inner_wall_valid;
	row.corridor_center_active = result.debug.corridor_center_active;
	row.wall_following_active = result.debug.wall_following_active;
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
	row.replay_speed_factor = result.debug.replay_speed_factor;
	row.active_normal_speed_mps = result.debug.active_normal_speed_mps;
	row.active_approach_speed_mps = result.debug.active_approach_speed_mps;
	row.target_acceleration_mps2 = result.debug.target_acceleration_mps2;
	row.effective_turn_trigger_m = result.debug.effective_turn_trigger_m;
	row.update_dt_s = result.debug.update_dt_s;
	row.wall_corner_forward_m = result.debug.wall_corner_forward_m;
	row.wall_corner_lateral_m = result.debug.wall_corner_lateral_m;
	row.wall_corner_stability_error_m =
		result.debug.wall_corner_stability_error_m;
	row.wall_corner_confirm_frames = result.debug.wall_corner_confirm_frames;
	row.wall_corner_candidate_valid = result.debug.wall_corner_candidate_valid;
	row.wall_corner_confirmed = result.debug.wall_corner_confirmed;
	row.wall_corner_trigger_active = result.debug.wall_corner_trigger_active;
	row.front_wall_fallback_active = result.debug.front_wall_fallback_active;
	row.turn_trigger_evaluated = result.debug.turn_trigger_evaluated;
	row.stanley_cross_track_term_rad =
		result.debug.stanley_cross_track_term_rad;
	row.stanley_heading_term_rad = result.debug.stanley_heading_term_rad;
	row.stanley_heading_integral = result.debug.stanley_heading_integral;
	row.turn_heading_pid_output_rad =
		result.debug.turn_heading_pid_output_rad;
	row.turn_heading_pid_integral = result.debug.turn_heading_pid_integral;
	row.turn_trigger_source =
		static_cast<int>(result.debug.turn_trigger_source);
	row.turn_trigger_frames = result.debug.turn_trigger_frames;
	row.turn_armed = result.debug.turn_armed;
	row.replay_gate_suppressed = result.debug.replay_gate_suppressed;
	row.raw_update_dt_s = result.debug.raw_update_dt_s;
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

	if (odometry.has_value()) {
		row.pose_timestamp_us = odometry->pose_timestamp_us;
		row.otos_velocity_x_mps = odometry->velocity_x_mps;
		row.otos_velocity_y_mps = odometry->velocity_y_mps;
		row.otos_yaw_rate_rps = odometry->yaw_rate_rps;
		row.otos_accel_x_mps2 = odometry->accel_x_mps2;
		row.otos_accel_y_mps2 = odometry->accel_y_mps2;
	}

	return row;
}

} // namespace logging
