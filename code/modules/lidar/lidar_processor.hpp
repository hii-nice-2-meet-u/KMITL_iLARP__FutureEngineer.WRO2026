#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <iostream>
#include <optional>
#include <vector>

#include "direction.hpp"
#include "lidar_struct.hpp"
#include <opencv2/core.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/imgproc.hpp>

namespace lidar {

struct CartesianPoint {
	float x_m;
	float y_m;
	float distance_m; // in meter
};

struct LineSegment {
	cv::Point2f start;
	cv::Point2f end;
	float angle_rad{0.0f};

	float normal_x{0.0f};
	float normal_y{0.0f};
	float line_c{0.0f};

	float rms_error_m{0.0f};

	float length() const {
		const float dx = end.x - start.x;
		const float dy = end.y - start.y;
		return std::hypot(dx, dy);
	}

	float perpendicular_distance() const { return std::abs(line_c); }
};

struct ResolvedWalls {
	std::optional<LineSegment> left;
	std::optional<LineSegment> right;
	std::optional<LineSegment> front;
};

struct TrackWalls {
	std::optional<LineSegment> inner;
	std::optional<LineSegment> outer;
	std::optional<LineSegment> front;
};

struct ObstacleObject {
	cv::Point2f center;

	float width_m{0.0f};
	float angle_rad{0.0f};

	float distance_m() const { return std::hypot(center.x, center.y); }

	float bearing_rad() const { return std::atan2(center.x, center.y); }

	cv::Point2f start() const {
		const cv::Point2f dir{std::cos(angle_rad), std::sin(angle_rad)};

		return center - dir * (width_m * 0.5f);
	}

	cv::Point2f end() const {
		const cv::Point2f dir{std::cos(angle_rad), std::sin(angle_rad)};

		return center + dir * (width_m * 0.5f);
	}
};

struct ProcessedLidarData {
	std::uint64_t timestamp_us{0};

	std::vector<LineSegment> line_segments;

	ResolvedWalls walls;

	std::optional<LineSegment> parking_wall;

	std::vector<ObstacleObject> obstacles;
};

class LidarProcessor {
  public:
	ProcessedLidarData process(const TimedLidarData &data,
		float heading_error_rad = 0.0f, std::size_t min_segment_point = 5,
		float max_line_error_m = 0.035f, float max_point_gap_m = 0.10f,
		float max_angle_diff = 3.0f, float max_collinear_error_m = 0.03f,
		float max_segment_gap_m = 0.05f) const;

	void draw_segment(
		cv::Mat &img, const LineSegment &segment, float scale_px_per_m) const;

  private:
	bool is_valid_point(const LidarPoint &point) const;

	CartesianPoint polar2cartesian(const LidarPoint &lidar_point) const;

	// clang-format off
	std::vector<std::vector<CartesianPoint>> split_line_segments(
		const std::vector<CartesianPoint> &points,
		float max_line_error_m,
		float max_point_gap_m,
		std::size_t min_points) const;

	void split_line_segments_recursive(
		const std::vector<CartesianPoint> &points,
		std::size_t start,
		std::size_t end,
		float max_line_error_m,
		float max_point_gap_m,
		std::size_t min_points,
		std::vector<std::vector<CartesianPoint>> &segments) const;

	
	std::optional<LineSegment> fit_line_segment(
    	const std::vector<CartesianPoint> &points) const;

	void merge_aligned_segments(
		std::vector<LineSegment> &segments,
		float max_angle_diff_rad,
		float max_collinear_error_m,
		float max_gap_m) const;

	// clang-format on

	ResolvedWalls resolve_track_walls(const std::vector<LineSegment> &segments,
		float heading_error_rad) const;

	bool is_same_segment(
		const LineSegment &a, const std::optional<LineSegment> &b) const;

	bool is_wall_fragment(const LineSegment &segment,
		const std::optional<LineSegment> &wall) const;

	std::vector<ObstacleObject> detect_obstacles(
		const std::vector<CartesianPoint> &points,
		const ResolvedWalls &walls) const;

	std::optional<LineSegment> find_parking_wall(
		const std::vector<LineSegment> &segments,
		const ResolvedWalls &walls) const;
};

} // namespace lidar