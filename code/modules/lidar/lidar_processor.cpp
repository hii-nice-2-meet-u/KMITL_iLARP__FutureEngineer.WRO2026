#include "lidar_processor.hpp"
#include <cmath>
#include <cstddef>

namespace lidar {
ProcessedLidarData LidarProcessor::process(const TimedLidarData &data) const {
	ProcessedLidarData result;
	result.timestamp_us = data.timestamp_us;
	std::vector<CartesianPoint> points;

	for (const auto &point : data.points) {
		if (!is_valid_point(point)) continue;

		points.push_back(polar2cartesian(point));
		if (points.empty()) return result;
		
		constexpr float MAX_LINE_ERROR_M = 0.02f;
		constexpr float MAX_POINT_GAP_M = 0.08f;
		constexpr std::size_t MIN_WALL_POINTS = 8;

		const auto segments = split_wall_points(
			points, MAX_LINE_ERROR_M, MAX_POINT_GAP_M, MIN_WALL_POINTS);
	}

	return result;
}

bool LidarProcessor::is_valid_point(const LidarPoint &point) const {
	if (point.quality < 50) return false;
	if (point.distance_m < 0.01f) return false;
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

// clang-format off
std::vector<std::vector<CartesianPoint>> LidarProcessor::split_wall_points(
	const std::vector<CartesianPoint> &points,
	float max_line_error_m,
	float max_point_gap_m,
	std::size_t min_points) const {

	std::vector<std::vector<CartesianPoint>> segments;

	if (points.size() < min_points) {
		return segments;
	}

	split_wall_points_recursive(
		points, 
		0, 
		points.size() - 1, 
		max_line_error_m,
		max_point_gap_m, min_points, 
		segments);

	return segments;
}

void LidarProcessor::split_wall_points_recursive(
	const std::vector<CartesianPoint> &points,
	std::size_t start,
	std::size_t end,
	float max_line_error_m,
	float max_point_gap_m,
	std::size_t min_points,
	std::vector<std::vector<CartesianPoint>> &segments) const {

	if (end <= start) {
		return;
	}

	const std::size_t count = end - start + 1;

	if (count < min_points) {
		return;
	}

	const auto &first = points[start];
	const auto &last = points[end];

	const float dx = last.x_m - first.x_m;
	const float dy = last.y_m - first.y_m;

	const float line_length = std::hypot(dx, dy);

	if (line_length < 1e-6f) {
		return;
	}

	// Check Gap between 2 Points
	for (std::size_t i = start; i < end; ++i) {

		const float gap = std::hypot(points[i + 1].x_m - points[i].x_m, points[i + 1].y_m - points[i].y_m);

		if (gap > max_point_gap_m) {

			split_wall_points_recursive(points, start, i, max_line_error_m,
				max_point_gap_m, min_points, segments);

			split_wall_points_recursive(points, i + 1, end, max_line_error_m,
				max_point_gap_m, min_points, segments);

			return;
		}
	}

	// find Point that far from line(start to end)
	float max_error = 0.0f;
	std::size_t split_index = start;

	for (std::size_t i = start + 1; i < end; ++i) {

		const float px = points[i].x_m;
		const float py = points[i].y_m;

		const float numerator = std::abs(

			//Ax+ By +C / hypot(dy, dx)
			dy * px - dx * py + last.x_m * first.y_m - last.y_m * first.x_m);

		const float distance = numerator / line_length;

		if (distance > max_error) {
			max_error = distance;
			split_index = i;
		}
	}

	
	if (max_error > max_line_error_m && split_index > start && split_index < end) {

		split_wall_points_recursive(points, start, split_index,
			max_line_error_m, max_point_gap_m, min_points, segments);

		split_wall_points_recursive(points, split_index, end, max_line_error_m,
			max_point_gap_m, min_points, segments);

		return;
	}

	
	std::vector<CartesianPoint> segment;

	segment.reserve(count);

	for (std::size_t i = start; i <= end; ++i) {
		segment.push_back(points[i]);
	}

	segments.push_back(std::move(segment));
}

// clang-format on
void LidarProcessor::merge_aligned_wall(std::vector<WallEstimate> &walls,
	float max_angle_diff_rad, float max_collinear_error_m,
	float max_gap_m) const {

	if (walls.size() < 2) {
		return;
	}

	std::vector<bool> removed(walls.size(), false);

	for (std::size_t i = 0; i < walls.size(); ++i) {
		if (removed[i]) {
			continue;
		}
		for (std::size_t j = i + 1; i < walls.size(); ++j) {
			if (removed[j]) {
				continue;
			}

			auto &a = walls[i];
			const auto &b = walls[j];

			// angle check
			float angle_diff = std::abs(b.angle_rad - a.angle_rad);
			angle_diff = std::fmod(angle_diff, static_cast<float>(M_PI));

			if (angle_diff > static_cast<float>(M_PI) * 0.5f) {
				angle_diff = static_cast<float>(M_PI) - angle_diff;
			}

			if (angle_diff > max_angle_diff_rad) {
				continue;
			}
			// Collinear check
			const cv::Point2f b_center = (b.start + b.end) * 0.5f;

			const float line_error = std::abs(
				a.normal_x * b_center.x + a.normal_y * b_center.y + a.line_c);

			if (line_error > max_collinear_error_m) {
				continue;
			}

			// gap check
			const float gap = std::min(
				{cv::norm(a.start - b.start), cv::norm(a.start - b.end),
					cv::norm(a.end - b.start), cv::norm(a.end - b.end)});
			if (gap > max_gap_m) {
				continue;
			}

			// merge
			std::vector<cv::Point2f> endpoints = {
				a.start, a.end, b.start, b.end};

			const cv::Point2f dir(std::cos(a.angle_rad), std::sin(a.angle_rad));

			auto projection = [&](const cv::Point2f &p) {
				return p.x * dir.x + p.y * dir.y;
			};

			cv::Point2f new_start = endpoints[0];
			cv::Point2f new_end = endpoints[0];

			float min_proj = projection(endpoints[0]);
			float max_proj = projection(endpoints[0]);

			for (const auto &p : endpoints) {
				float proj = projection(p);

				if (proj < min_proj) {
					min_proj = proj;
					new_start = p;
				}

				if (proj > max_proj) {
					max_proj = proj;
					new_end = p;
				}
			}

			a.start = new_start;
			a.end = new_end;
			removed[j] = true;
		}
	}

	std::vector<WallEstimate> merged;
	merged.reserve(walls.size());

	for (std::size_t i = 0; i < walls.size(); ++i) {
		if (!removed[i]) {
			merged.push_back(walls[i]);
		}
	}

	walls = std::move(merged);
}

std::optional<WallEstimate> LidarProcessor::fit_wall(
	const std::vector<CartesianPoint> &points) const {

	if (points.size() < 2) {
		return std::nullopt;
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

	WallEstimate result;
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

	result.start = cv::Point2f(points.front().x_m, points.front().y_m);
	result.end = cv::Point2f(points.back().x_m, points.back().y_m);

	result.angle_rad = theta;

	result.normal_x = normal_x;
	result.normal_y = normal_y;

	result.line_c = c;

	result.rms_error_m = std::sqrt(error_sum / n);

	return result;
}

} // namespace lidar