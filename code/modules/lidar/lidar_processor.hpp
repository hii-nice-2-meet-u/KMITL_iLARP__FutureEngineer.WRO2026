#pragma once

#include "lidar_struct.hpp"
#include <algorithm>
#include <cmath>
#include <vector>

namespace lidar {
struct CartesianPoint {
	float x_m;
	float y_m;
	float angle_deg;
	float distance_m; // in meter
};

struct WallEstimate {
	bool valid{false};

	float distance_m{0.0f};
	float angle_deg{0.0f};
};

struct ProcessedLidarData {
	std::uint64_t timestamp_us{0};

	WallEstimate left_wall;
	WallEstimate right_wall;
	WallEstimate front_wall;

	float front_distance_m{0.0f};
	float left_distance_m{0.0f};
	float right_distance_m{0.0f};
};
class LidarProcessor {
  public:
	ProcessedLidarData process(const TimedLidarData &point) const;

  private:
	bool is_valid_point(const LidarPoint &point) const;

	CartesianPoint polar2cartesian(const LidarPoint &lidar_point) const;

	std::vector<CartesianPoint> get_sector(
		const TimedLidarData &data, float start_angle, float end_angle) const;

	float median_distance(const std::vector<CartesianPoint> &points) const;

	std::vector<CartesianPoint> extract_wall_candidates(
		const std::vector<CartesianPoint> &points) const;

	WallEstimate fit_wall(const std::vector<CartesianPoint> &points) const;
};

} // namespace lidar