#include "run_metadata.hpp"

#include <charconv>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>

#include "log_types.hpp"

#ifndef ILARP_GIT_COMMIT_HASH
#define ILARP_GIT_COMMIT_HASH "UNKNOWN"
#endif

#ifndef ILARP_GIT_DIRTY
#define ILARP_GIT_DIRTY 1
#endif

#ifndef ILARP_BUILD_TIMESTAMP
#define ILARP_BUILD_TIMESTAMP "UNKNOWN"
#endif

namespace logging {

namespace {

std::string escape_json(const std::string &value) {
	std::ostringstream stream;
	for (const unsigned char character : value) {
		switch (character) {
		case '"':
			stream << "\\\"";
			break;
		case '\\':
			stream << "\\\\";
			break;
		case '\b':
			stream << "\\b";
			break;
		case '\f':
			stream << "\\f";
			break;
		case '\n':
			stream << "\\n";
			break;
		case '\r':
			stream << "\\r";
			break;
		case '\t':
			stream << "\\t";
			break;
		default:
			if (character < 0x20) {
				stream << "\\u" << std::hex << std::setw(4)
					   << std::setfill('0') << static_cast<int>(character)
					   << std::dec;
			} else {
				stream << static_cast<char>(character);
			}
		}
	}
	return stream.str();
}

std::string quoted(const std::string &value) {
	return "\"" + escape_json(value) + "\"";
}

template <typename Number>
std::string number_json(Number value) {
	if (!std::isfinite(value)) {
		return "null";
	}
	char buffer[64];
	const auto result = std::to_chars(
		buffer, buffer + sizeof(buffer), value, std::chars_format::general);
	if (result.ec != std::errc{}) {
		return "null";
	}
	return std::string(buffer, result.ptr);
}

std::size_t csv_field_count(const char *header) {
	if (header == nullptr || *header == '\0') {
		return 0;
	}
	std::size_t count = 1;
	for (const char *character = header; *character != '\0'; ++character) {
		if (*character == ',') {
			++count;
		}
	}
	return count;
}

} // namespace

JsonObject &JsonObject::add_string(
	const std::string &key, const std::string &value) {
	entries_.emplace_back(key, quoted(value));
	return *this;
}

JsonObject &JsonObject::add_bool(const std::string &key, bool value) {
	entries_.emplace_back(key, value ? "true" : "false");
	return *this;
}

JsonObject &JsonObject::add_number(const std::string &key, float value) {
	entries_.emplace_back(key, number_json(value));
	return *this;
}

JsonObject &JsonObject::add_number(const std::string &key, double value) {
	entries_.emplace_back(key, number_json(value));
	return *this;
}

JsonObject &JsonObject::add_integer(
	const std::string &key, std::int64_t value) {
	entries_.emplace_back(key, std::to_string(value));
	return *this;
}

JsonObject &JsonObject::add_unsigned(
	const std::string &key, std::uint64_t value) {
	entries_.emplace_back(key, std::to_string(value));
	return *this;
}

JsonObject &JsonObject::add_null(const std::string &key) {
	entries_.emplace_back(key, "null");
	return *this;
}

JsonObject &JsonObject::add_object(
	const std::string &key, const JsonObject &value) {
	entries_.emplace_back(key, value.to_json());
	return *this;
}

std::string JsonObject::to_json() const {
	std::ostringstream stream;
	stream << "{\n";
	for (std::size_t index = 0; index < entries_.size(); ++index) {
		stream << "  " << quoted(entries_[index].first) << ": "
			   << entries_[index].second;
		if (index + 1 < entries_.size()) {
			stream << ',';
		}
		stream << '\n';
	}
	stream << '}';
	return stream.str();
}

JsonObject pid_config_json(const control::PIDConfig &config) {
	JsonObject object;
	object.add_number("kp", config.kp)
		.add_number("ki", config.ki)
		.add_number("kd", config.kd)
		.add_number("min_output", config.min_output)
		.add_number("max_output", config.max_output)
		.add_number("min_integral", config.min_integral)
		.add_number("max_integral", config.max_integral)
		.add_number("max_dt_s", config.max_dt_s);
	return object;
}

JsonObject stanley_config_json(const control::StanleyConfig &config) {
	JsonObject object;
	object.add_number("k", config.k)
		.add_number("softening_speed_mps", config.softening_speed_mps)
		.add_number("max_steering_rad", config.max_steering_rad)
		.add_object("heading_pid", pid_config_json(config.heading_pid));
	return object;
}

JsonObject navigation_config_json(const navigation::NavigationConfig &config) {
	JsonObject object;
	object.add_number("target_outer_distance_m", config.target_outer_distance_m)
		.add_bool("follow_corridor_center", config.follow_corridor_center)
		.add_object("stanley", stanley_config_json(config.stanley))
		.add_number("search_center_kp", config.search_center_kp)
		.add_bool(
			"search_preserve_initial_offset", config.search_preserve_initial_offset)
		.add_number("search_front_slowdown_distance_m",
			config.search_front_slowdown_distance_m)
		.add_number("search_front_minimum_distance_m",
			config.search_front_minimum_distance_m)
		.add_number("search_minimum_speed_mps", config.search_minimum_speed_mps)
		.add_number("approach_distance_m", config.approach_distance_m)
		.add_number("turn_trigger_distance_m", config.turn_trigger_distance_m)
		.add_number("turn_rearm_distance_m", config.turn_rearm_distance_m)
		.add_number("turn_preview_time_s", config.turn_preview_time_s)
		.add_integer(
			"turn_trigger_confirm_frames", config.turn_trigger_confirm_frames)
		.add_bool("use_wall_corner_trigger", config.use_wall_corner_trigger)
		.add_number("front_wall_fallback_distance_m",
			config.front_wall_fallback_distance_m)
		.add_number("lidar_lateral_offset_m", config.lidar_lateral_offset_m)
		.add_number("lidar_forward_offset_m", config.lidar_forward_offset_m)
		.add_number("wall_corner_to_path_offset_m",
			config.wall_corner_to_path_offset_m)
		.add_number(
			"wall_corner_min_forward_m", config.wall_corner_min_forward_m)
		.add_number(
			"wall_corner_max_forward_m", config.wall_corner_max_forward_m)
		.add_number("wall_corner_min_inner_length_m",
			config.wall_corner_min_inner_length_m)
		.add_number("wall_corner_stability_tolerance_m",
			config.wall_corner_stability_tolerance_m)
		.add_number("wall_corner_association_distance_m",
			config.wall_corner_association_distance_m)
		.add_number(
			"wall_corner_filter_weight", config.wall_corner_filter_weight)
		.add_number("wall_corner_collinear_angle_rad",
			config.wall_corner_collinear_angle_rad)
		.add_number("wall_corner_collinear_offset_m",
			config.wall_corner_collinear_offset_m)
		.add_number("wall_corner_continuation_gap_m",
			config.wall_corner_continuation_gap_m)
		.add_integer(
			"wall_corner_confirm_frames", config.wall_corner_confirm_frames)
		.add_integer("wall_corner_max_missed_frames",
			config.wall_corner_max_missed_frames)
		.add_number("wheelbase_m", config.wheelbase_m)
		.add_number("corner_radius_m", config.corner_radius_m)
		.add_number("turn_entry_blend_rad", config.turn_entry_blend_rad)
		.add_number("turn_exit_blend_rad", config.turn_exit_blend_rad)
		.add_number(
			"exit_acceleration_blend_rad", config.exit_acceleration_blend_rad)
		.add_object(
			"turn_heading_pid", pid_config_json(config.turn_heading_pid))
		.add_number("heading_tolerance_rad", config.heading_tolerance_rad)
		.add_integer("heading_confirm_frames", config.heading_confirm_frames)
		.add_number(
			"clockwise_turn_delta_rad", config.clockwise_turn_delta_rad)
		.add_number("counter_clockwise_turn_delta_rad",
			config.counter_clockwise_turn_delta_rad)
		.add_number(
			"heading_to_steering_sign", config.heading_to_steering_sign)
		.add_number("search_speed_mps", config.search_speed_mps)
		.add_number("normal_speed_mps", config.normal_speed_mps)
		.add_number("approach_speed_mps", config.approach_speed_mps)
		.add_number("turning_speed_mps", config.turning_speed_mps)
		.add_number("lost_wall_speed_mps", config.lost_wall_speed_mps)
		.add_bool(
			"enable_replay_speed_factors", config.enable_replay_speed_factors)
		.add_number("lap2_speed_factor", config.lap2_speed_factor)
		.add_number("lap3_speed_factor", config.lap3_speed_factor)
		.add_number("replay_approach_factor_weight",
			config.replay_approach_factor_weight)
		.add_number(
			"maximum_replay_speed_mps", config.maximum_replay_speed_mps)
		.add_number("replay_turn_gate_distance_m",
			config.replay_turn_gate_distance_m)
		.add_number("replay_front_safety_override_distance_m",
			config.replay_front_safety_override_distance_m)
		.add_number("max_heading_hold_s", config.max_heading_hold_s)
		.add_number("max_lateral_acceleration_mps2",
			config.max_lateral_acceleration_mps2)
		.add_number("steering_filter_time_constant_s",
			config.steering_filter_time_constant_s)
		.add_number(
			"max_steering_rate_rad_s", config.max_steering_rate_rad_s)
		.add_number("max_acceleration_mps2", config.max_acceleration_mps2)
		.add_number("max_deceleration_mps2", config.max_deceleration_mps2)
		.add_object("speed_pid", pid_config_json(config.speed_pid))
		.add_number(
			"nominal_update_period_s", config.nominal_update_period_s)
		.add_number("min_update_period_s", config.min_update_period_s)
		.add_number("max_update_period_s", config.max_update_period_s)
		.add_integer("total_turns", config.total_turns);
	return object;
}

JsonObject make_run_metadata(const std::string &executable_path,
	const navigation::NavigationConfig &navigation_config,
	const std::optional<float> &otos_linear_scalar,
	const std::optional<float> &otos_angular_scalar) {
	JsonObject build;
	build.add_string("git_commit", ILARP_GIT_COMMIT_HASH)
		.add_bool("git_dirty", ILARP_GIT_DIRTY != 0)
		.add_string("build_timestamp_utc", ILARP_BUILD_TIMESTAMP);

	JsonObject schema;
	schema.add_unsigned(
		"telemetry_field_count", csv_field_count(telemetry_csv_header()));

	JsonObject otos;
	otos.add_string("linear_unit", "meters")
		.add_string("angular_unit", "radians")
		.add_bool("linear_scalar_valid", otos_linear_scalar.has_value())
		.add_bool("angular_scalar_valid", otos_angular_scalar.has_value());
	if (otos_linear_scalar.has_value()) {
		otos.add_number("linear_scalar", *otos_linear_scalar);
	} else {
		otos.add_null("linear_scalar");
	}
	if (otos_angular_scalar.has_value()) {
		otos.add_number("angular_scalar", *otos_angular_scalar);
	} else {
		otos.add_null("angular_scalar");
	}

	JsonObject root;
	root.add_object("build", build)
		.add_string("executable_name",
			std::filesystem::path(executable_path).filename().string())
		.add_object("schema", schema)
		.add_object(
			"navigation_config", navigation_config_json(navigation_config))
		.add_object("otos", otos);
	return root;
}

bool write_run_metadata(
	const std::string &run_directory, const JsonObject &metadata) {
	std::error_code error;
	std::filesystem::create_directories(run_directory, error);
	if (error) {
		std::cerr << "[LOGGER] cannot create " << run_directory << ": "
				  << error.message() << '\n';
		return false;
	}

	const std::filesystem::path final_path =
		std::filesystem::path(run_directory) / "run_meta.json";
	const std::filesystem::path temporary_path =
		std::filesystem::path(run_directory) / "run_meta.json.tmp";

	std::ofstream file(temporary_path, std::ios::out | std::ios::trunc);
	if (!file.is_open()) {
		std::cerr << "[LOGGER] cannot open " << temporary_path << '\n';
		return false;
	}
	file << metadata.to_json() << '\n';
	file.flush();
	if (!file.good()) {
		std::cerr << "[LOGGER] write failed for " << temporary_path << '\n';
		file.close();
		std::filesystem::remove(temporary_path, error);
		return false;
	}
	file.close();

	std::filesystem::rename(temporary_path, final_path, error);
	if (error) {
		std::cerr << "[LOGGER] cannot replace " << final_path << ": "
				  << error.message() << '\n';
		std::filesystem::remove(temporary_path, error);
		return false;
	}
	return true;
}

} // namespace logging
