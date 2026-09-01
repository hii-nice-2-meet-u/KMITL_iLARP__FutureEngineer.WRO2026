#include "segment_logger.hpp"

#include <cmath>

#include "log_types.hpp"

namespace logging {

SegmentLogger::SegmentLogger(const std::string &run_directory)
	: writer_(std::make_unique<AsyncCsvWriter>(
		  run_directory + "/segments.csv", segments_csv_header())) {}

void SegmentLogger::record(const lidar::ProcessedLidarData &lidar_data,
	navigation::NavigationMode mode) {
	const std::uint64_t row_index = next_row_index_++;
	if (!writer_) {
		return;
	}

	const char *mode_name = navigation_mode_name(mode);
	for (std::size_t index = 0; index < lidar_data.line_segments.size();
		 ++index) {
		const auto &segment = lidar_data.line_segments[index];
		SegmentRow row;
		row.timestamp_us = lidar_data.timestamp_us;
		row.row_index = row_index;
		row.mode = mode_name;
		row.segment_index = index;
		row.start_x_m = segment.start.x;
		row.start_y_m = segment.start.y;
		row.end_x_m = segment.end.x;
		row.end_y_m = segment.end.y;
		row.angle_rad = segment.angle_rad;
		row.length_m = segment.length();
		row.role = role_for(segment, lidar_data.walls);
		writer_->push(to_csv_row(row));
	}
}

bool SegmentLogger::flush() { return writer_ && writer_->flush(); }

bool SegmentLogger::has_write_error() const {
	return writer_ && writer_->has_write_error();
}

std::size_t SegmentLogger::dropped_row_count() const {
	return writer_ ? writer_->dropped_row_count() : 0;
}

const char *SegmentLogger::role_for(const lidar::LineSegment &segment,
	const lidar::ResolvedWalls &walls) {
	if (same_segment(segment, walls.left)) {
		return "LEFT";
	}
	if (same_segment(segment, walls.right)) {
		return "RIGHT";
	}
	if (same_segment(segment, walls.front)) {
		return "FRONT";
	}
	return "NONE";
}

bool SegmentLogger::same_segment(const lidar::LineSegment &first,
	const std::optional<lidar::LineSegment> &second) {
	if (!second.has_value()) {
		return false;
	}

	constexpr float EPSILON_M = 0.001f;
	auto same_point = [](const cv::Point2f &a, const cv::Point2f &b) {
		return std::hypot(a.x - b.x, a.y - b.y) < EPSILON_M;
	};
	return (same_point(first.start, second->start) &&
			same_point(first.end, second->end)) ||
		(same_point(first.start, second->end) &&
			same_point(first.end, second->start));
}

} // namespace logging
