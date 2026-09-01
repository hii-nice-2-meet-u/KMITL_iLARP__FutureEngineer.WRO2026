#include "wall_logger.hpp"

#include <filesystem>
#include <fstream>
#include <iostream>

namespace logging {

WallLogger::WallLogger(const std::string &run_directory)
	: writer_(std::make_unique<AsyncCsvWriter>(
		  run_directory + "/walls.csv", walls_csv_header())) {}

void WallLogger::record(const lidar::ResolvedWalls &walls,
	navigation::NavigationMode mode, const navigation::MapPose &pose,
	std::uint64_t timestamp_us) {
	if (!writer_) {
		return;
	}

	// Recorded in every mode. Gating this on NORMAL left walls.csv with no
	// geometry at all through corners and direction search, which is exactly
	// where a mis-resolved wall matters most. The mode column lets the reader
	// filter instead.
	const char *mode_name = navigation_mode_name(mode);
	queue_wall(walls.left, "LEFT", mode_name, pose, timestamp_us);
	queue_wall(walls.right, "RIGHT", mode_name, pose, timestamp_us);
	queue_wall(walls.front, "FRONT", mode_name, pose, timestamp_us);
}

void WallLogger::queue_wall(const std::optional<lidar::LineSegment> &segment,
	const char *role, const char *mode_name, const navigation::MapPose &pose,
	std::uint64_t timestamp_us) {
	if (!segment.has_value()) {
		return;
	}

	WallRow row;
	row.timestamp_us = timestamp_us;
	row.wall_role = role;
	row.mode = mode_name;
	row.pos_x_m = pose.x_m;
	row.pos_y_m = pose.y_m;
	row.heading_rad = pose.heading_rad;
	row.segment_start_x_m = segment->start.x;
	row.segment_start_y_m = segment->start.y;
	row.segment_end_x_m = segment->end.x;
	row.segment_end_y_m = segment->end.y;
	row.wall_angle_rad = segment->angle_rad;
	writer_->push(to_csv_row(row));
}

bool WallLogger::flush() { return writer_ && writer_->flush(); }

bool WallLogger::has_write_error() const {
	return writer_ && writer_->has_write_error();
}

std::size_t WallLogger::dropped_row_count() const {
	return writer_ ? writer_->dropped_row_count() : 0;
}

bool dump_corners(
	const std::string &run_directory, const navigation::TrackMap &map) {
	std::error_code error;
	std::filesystem::create_directories(run_directory, error);
	if (error) {
		std::cerr << "[LOGGER] cannot create " << run_directory << ": "
				  << error.message() << '\n';
		return false;
	}

	const std::string path = run_directory + "/corners.csv";
	std::ofstream file(path, std::ios::out | std::ios::trunc);
	if (!file.is_open()) {
		std::cerr << "[LOGGER] cannot open " << path << '\n';
		return false;
	}

	file << corners_csv_header() << '\n';
	const auto &corners = map.corners();

	for (std::size_t index = 0; index < corners.size(); ++index) {
		const auto &corner = corners[index];
		CornerRow row;
		row.corner_index = index;
		row.entry_valid = corner.entry_valid;
		row.exit_valid = corner.exit_valid;
		row.entry_x_m = corner.entry_pose.x_m;
		row.entry_y_m = corner.entry_pose.y_m;
		row.entry_heading_rad = corner.entry_pose.heading_rad;
		row.exit_x_m = corner.exit_pose.x_m;
		row.exit_y_m = corner.exit_pose.y_m;
		row.exit_heading_rad = corner.exit_pose.heading_rad;
		row.preferred_turn_trigger_m = corner.preferred_turn_trigger_m;
		row.preferred_corner_radius_m = corner.preferred_corner_radius_m;
		row.safe_speed_mps = corner.safe_speed_mps;
		row.confidence = corner.confidence;
		row.entry_observations = corner.entry_observations;
		row.exit_observations = corner.exit_observations;
		file << to_csv_row(row) << '\n';
	}

	file.flush();
	if (!file.good()) {
		std::cerr << "[LOGGER] write failed for " << path << '\n';
		return false;
	}

	return true;
}

} // namespace logging
