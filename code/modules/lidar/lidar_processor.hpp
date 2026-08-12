#pragma once

#include "lidar_struct.hpp"
#include <vector>

namespace lidar {
struct CartesianPoint {
	float x_m;
	float y_m;
	float angle_deg;
	float distance_m; // in meter
};

class LidarProcessor {
  public:
  private:
	bool is_valid_point(const LidarPoint &point) const;
	CartesianPoint polar2cartesian(const LidarPoint &lidar_point) const;
	std::vector<CartesianPoint> get_sector(
		TimedLidarData &data, float start_angle, float end_angle) const;

	}

} // namespace lidar