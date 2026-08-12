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

	// if (point.angle_deg) return false;  // ซักอย่าง ลืม
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

float LidarProcessor::median_distance(
	const std::vector<CartesianPoint> &points) const {
	if (points.empty()) {
		return 0.0f;
	}

	std::vector<float> distances;
	distances.reserve(points.size());

	for (const auto &point : points) {
		distances.push_back(point.distance_m);
	}

	std::sort(distances.begin(), distances.end());

	const std::size_t middle = distances.size() / 2;

	if (distances.size() % 2 == 0) {
		return (distances[middle - 1] + distances[middle]) / 2.0f;
	}

	return distances[middle];
}
std::vector<CartesianPoint> LidarProcessor::extract_wall_candidates(
	const std::vector<CartesianPoint> &points) const {
	std::vector<CartesianPoint> candidates;

	if (points.empty()) {
		return candidates;
	}

	/*
	 * First implementation:
	 *
	 *
	 *  after test will add:
	 * - distance gating
	 * - neighbor continuity
	 * - cluster rejection
	 * - outlier rejection
	 */

	candidates.reserve(points.size());

	for (const auto &point : points) {
		candidates.push_back(point);
	}

	return candidates;
}

WallEstimate LidarProcessor::fit_wall(
	const std::vector<CartesianPoint> &points) const {
	WallEstimate result;

	if (points.size() < 2) {
		return result;
	}

	// y = mx + b

	float sum_x = 0.0f;
	float sum_y = 0.0f;
	float sum_xx = 0.0f;
	float sum_xy = 0.0f;

	for (const auto &point : points) {
		sum_x += point.x_m;
		sum_y += point.y_m;
		sum_xx += point.x_m * point.x_m;
		sum_xy += point.x_m * point.y_m;
	}

	const float n = static_cast<float>(points.size());

	const float denominator = n * sum_xx - sum_x * sum_x;

	if (std::abs(denominator) < 1e-6f) {
		return result;
	}

	const float slope = (n * sum_xy - sum_x * sum_y) / denominator;

	const float intercept = (sum_y - slope * sum_x) / n;

	//   mx - y + b = 0

	result.distance_m = std::abs(intercept) / std::sqrt(slope * slope + 1.0f);

	constexpr float PI = 3.14159265358979323846f;

	result.angle_deg = std::atan(slope) * 180.0f / PI;

	result.valid = true;
	return result;
} // namespace lidar