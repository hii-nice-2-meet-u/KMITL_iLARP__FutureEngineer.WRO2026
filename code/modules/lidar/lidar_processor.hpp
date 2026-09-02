#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <iostream>
#include <optional>
#include <vector>

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

// Vehicle motion across one scan window, used to undo LiDAR motion distortion.
// Passed into process() by value so the processor stays a pure function of its
// inputs.
//
// Deskew is applied per point only when `valid` is true. Production apps set it
// after M-1 confirmed the rearward raw-zero mount and +Y robot-forward frame.
struct ScanMotion {
	float forward_speed_mps{0.0f}; // signed; forward positive, robot +Y
	float yaw_rate_rps{0.0f};	   // OTOS convention (+ = CCW)
	float scan_period_s{0.05f};	   // measured revolution time, not assumed
	bool valid{false};
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

	// Per-scan point-rejection tally. total counts every raw return; the two
	// rejected_* buckets show why points were dropped by is_valid_point, so a
	// badly-set quality or range threshold is visible without persisting the
	// raw scan. accepted = total - rejected_quality - rejected_range.
	struct ScanRejectStats {
		std::size_t total{0};
		std::size_t rejected_quality{0};
		std::size_t rejected_range{0};
	};
	ScanRejectStats reject_stats;
};

class LidarProcessor {
  public:
	ProcessedLidarData process(const TimedLidarData &data,
		std::size_t min_segment_point = 5,
		float max_line_error_m = 0.035f, float max_point_gap_m = 0.10f,
		float max_angle_diff = 3.0f, float max_collinear_error_m = 0.03f,
		float max_segment_gap_m = 0.05f,
		const ScanMotion &motion = ScanMotion{}) const;

	// Express a point measured mid-scan (at fraction `scan_phase` of the
	// revolution) in the robot frame as it was at scan end, undoing the body's
	// motion since that sample. Exact SE(2) for a constant twist; see the .cpp
	// for the derivation and the error bound. Public so it can be unit-tested
	// in isolation.
	CartesianPoint deskew(const CartesianPoint &point, float scan_phase,
		const ScanMotion &motion) const;

	// Fit a line with iteratively reweighted Huber loss. Public so the robust
	// estimator can be tested without requiring a live LiDAR device.
	std::optional<LineSegment> fit_line_segment(
		const std::vector<CartesianPoint> &points) const;

	void draw_segment(
		cv::Mat &img, const LineSegment &segment, float scale_px_per_m) const;

  private:
	// Single source of truth for the point-validity gate. is_valid_point()
	// delegates to it so the accept/reject decision and the rejection-reason
	// tally can never diverge.
	enum class PointRejectReason { ACCEPT, QUALITY, RANGE };
	PointRejectReason classify_point(const LidarPoint &point) const;

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

	
	void merge_aligned_segments(
		std::vector<LineSegment> &segments,
		float max_angle_diff_rad,
		float max_collinear_error_m,
		float max_gap_m) const;

	// clang-format on

	ResolvedWalls resolve_track_walls(const std::vector<LineSegment> &segments) const;

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
