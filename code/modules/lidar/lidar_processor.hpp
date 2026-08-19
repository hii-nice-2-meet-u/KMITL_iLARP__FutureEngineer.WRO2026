#pragma once

#include <algorithm>
#include <cmath>
#include <iostream>
#include <vector>

#include "lidar_struct.hpp"

namespace lidar {
struct CartesianPoint {
	float x_m;
	float y_m;
	float distance_m; // in meter
};

struct WallEstimate {
	bool valid = false;

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

	WallEstimate left_wall;
	WallEstimate right_wall;
	WallEstimate front_wall;

	float front_distance_m{0.0f};
	float left_distance_m{0.0f};
	float right_distance_m{0.0f};

	std::vector<ObstacleObject> obstacles;
};
class LidarProcessor {
  public:
	ProcessedLidarData process(const TimedLidarData &data) const;

  private:
	bool is_valid_point(const LidarPoint &point) const;

	CartesianPoint polar2cartesian(const LidarPoint &lidar_point) const;

	WallEstimate fit_wall(const std::vector<CartesianPoint> &points) const;

	// std::vector<ObstacleObject> detect_obstacles(
	// 	const TimedLidarData &data) const;

	// float calculate_bearing(float x_m, float y_m) const;
};

} // namespace lidar