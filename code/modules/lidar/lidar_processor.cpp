#include "lidar_processor.hpp"

namespace lidar {
ProcessedLidarData LidarProcessor::process(const TimedLidarData &data) const {
	ProcessedLidarData result;
	result.timestamp_us = data.timestamp_us;

	const auto front_points = get_sector(data, 340.0f, 20.0f);
	const auto right_points = get_sector(data, 70.0f, 110.0f);
	const auto left_points = get_sector(data, 250.0f, 290.0f);

	// DEBUG: Check right-sector Polar -> Cartesian conversion
	// std::cout << "\n--- RIGHT CARTESIAN ---\n";

	// for (std::size_t i = 0; i < right_points.size() ; ++i) {

	// 	const auto &p = right_points[i];

	// 	std::cout << "angle=" << p.angle_deg << " dist=" << p.distance_m
	// 			  << " x=" << p.x_m << " y=" << p.y_m << '\n';
	// }

	result.front_distance_m = median_distance(front_points);
	result.left_distance_m = median_distance(left_points);
	result.right_distance_m = median_distance(right_points);

	const auto left_candidates = extract_wall_candidates(left_points);

	const auto right_candidates = extract_wall_candidates(right_points);

	const auto front_candidates = extract_wall_candidates(front_points);

	result.left_wall = fit_wall(left_candidates);
	result.right_wall = fit_wall(right_candidates);
	result.front_wall = fit_wall(front_candidates);

	return result;
}

bool LidarProcessor::is_valid_point(const LidarPoint &point) const {
	if (point.quality < 50)
		return false;
	if (point.distance_m < 0.01f)
		return false;
	// if (point.distance_m <= 0.0f)
	// 	return false;

	// if (point.angle_deg) return false;  // ซักอย่าง ลืม
	return true;
}

CartesianPoint LidarProcessor::polar2cartesian(const LidarPoint &point) const {
	CartesianPoint result;
	result.distance_m = point.distance_m;
	const float rad = point.angle_deg * static_cast<float>(M_PI) / 180.f;
	result.x_m = point.distance_m * std::cos(rad);
	result.y_m = point.distance_m * std::sin(rad);

	return result;
}

WallEstimate LidarProcessor::fit_wall(
	const std::vector<CartesianPoint> &points) const {
	WallEstimate result;

	if (points.size() < 2) {
		return result;
	}

	float mean_x = 0.0f;
	float mean_y = 0.0f;

	for (const auto &p : points) {
		mean_x += p.x_m;
		mean_y += p.y_m;
	}

	const float n = static_cast<float>(points.size());

	mean_x /= n;
	mean_y /= n;

	float sxx = 0.0f;
	float syy = 0.0f;
	float sxy = 0.0f;

	for (const auto &p : points) {
		const float dx = p.x_m - mean_x;
		const float dy = p.y_m - mean_y;

		sxx += dx * dx;
		syy += dy * dy;
		sxy += dx * dy;
	}

	const float theta = 0.5f * std::atan2(2.0f * sxy, sxx - syy);

	const float dir_x = std::cos(theta);
	const float dir_y = std::sin(theta);

	const float normal_x = -dir_y;
	const float normal_y = dir_x;

	const float c = -(normal_x * mean_x + normal_y * mean_y);

	float error_sum = 0.0f;

	for (const auto &p : points) {
		const float distance = normal_x * p.x_m + normal_y * p.y_m + c;

		error_sum += distance * distance;
	}

	result.valid = true;
	result.angle_rad = theta;

	result.normal_x = normal_x;
	result.normal_y = normal_y;

	result.line_c = c;

	result.rms_error_m = std::sqrt(error_sum / n);

	return result;
}

} // namespace lidar