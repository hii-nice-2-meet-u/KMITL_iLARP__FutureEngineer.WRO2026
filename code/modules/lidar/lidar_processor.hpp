#pragma once

#include <algorithm>
#include <cmath>
#include <iostream>
#include <optional>
#include <vector>

#include "lidar_struct.hpp"
#include <opencv2/core/types.hpp>

namespace lidar {

struct CartesianPoint {
	float x_m;
	float y_m;
	float distance_m; // in meter
};

struct WallEstimate {
	cv::Point2f start;
	cv::Point2f end;
	float angle_rad = 0.0f;

	float normal_x = 0.0f;
	float normal_y = 0.0f;
	float line_c = 0.0f;

	float rms_error_m = 0.0f;
};

struct ObstacleObject {
	float angle_deg{0.0f};
	float distance_m{0.0f};

	float x_m{0.0f};
	float y_m{0.0f};

	float width_m{0.0f};
};

struct ProcessedLidarData {
	std::uint64_t timestamp_us{0};
	std::vector<WallEstimate> merged_walls;
	std::vector<ObstacleObject> obstacles;
};
class LidarProcessor {
  public:
	ProcessedLidarData process(const TimedLidarData &data) const;

  private:
	bool is_valid_point(const LidarPoint &point) const;

	CartesianPoint polar2cartesian(const LidarPoint &lidar_point) const;

	// clang-format off
	std::vector<std::vector<CartesianPoint>> split_wall_points(
		const std::vector<CartesianPoint> &points,
		float max_line_error_m,
		float max_point_gap_m,
		std::size_t min_points) const;

	void split_wall_points_recursive(
		const std::vector<CartesianPoint> &points,
		std::size_t start, std::size_t end,
		float max_line_error_m,
		float max_point_gap_m,
		std::size_t min_points,
		std::vector<std::vector<CartesianPoint>> &segments) const;

	
	std::optional<WallEstimate> fit_wall(
		const std::vector<CartesianPoint> &points) const;

	void merge_aligned_wall(
		std::vector<WallEstimate> &walls,
		float max_angle_diff_rad, 
		float max_collinear_error_m,
		float max_gap_m) const;
	// clang-format on

	// std::vector<ObstacleObject> detect_obstacles(
	// 	const TimedLidarData &data) const;

	// float calculate_bearing(float x_m, float y_m) const;
};

} // namespace lidar