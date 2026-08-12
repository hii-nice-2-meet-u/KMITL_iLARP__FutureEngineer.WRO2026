#include "lidar_processor.hpp"
#include "lidar_struct.hpp"
#include <cmath>

namespace lidar {
LidarProcessor::is_valid_point(const LidarPoint &point) const {
	if (point.quality < 50)
		return false;
	if (point.distance_m < 0.01)
		return false;
	if (point.distance_m <= 0.0f)
		return false;

	// if (point.angle_deg) return false;  // ซักอย่าง ลืใ
	return true;
}

CartesianPoint polar2cartesian(const LidarPoint &point) const {
	CartesianPoint result;
	result.angle_deg = point.angle_deg;
	result.distance_m = point.distance_m;
	float rad = point.angle_deg * static_cast<float>(M_PI) / 180.f;
	result.x_m = point.distance_m * std::cos(point.angle_deg);
	result.y_m = point.distance_m * std::sin(point.angle_deg);

	return result;
}
std::vector<CartesianPoint> get_sector(
	TimedLidarData &data, float start_angle, float end_angle) const {
	std::vector<CartesianPoint> result;

	result.reserve(data.points.size());

	for (const auto &point : data.points) {
		if (!is_valid_point(point)) {
			continue;
		}

		bool inside_sector = false;

		// Normal sector
		if (start_angle_deg <= end_angle_deg) {
			inside_sector = point.angle_deg >= start_angle_deg &&
				point.angle_deg <= end_angle_deg;
		} else { // Wrap-around sector 
			inside_sector = point.angle_deg >= start_angle_deg ||
				point.angle_deg <= end_angle_deg;
		}

		if (!inside_sector) {
			continue;
		}

		result.push_back(polar_to_cartesian(point));
	}

	return result;
}
} // namespace lidar