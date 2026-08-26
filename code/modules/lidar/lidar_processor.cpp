#include "lidar_processor.hpp"
#include <cmath>
#include <cstddef>
#include <iostream>
#include <opencv2/core/types.hpp>

namespace lidar {

// clang-format off
ProcessedLidarData LidarProcessor::process(
	const TimedLidarData &data,
	std::size_t min_segment_point,
	float max_line_error_m,
	float max_point_gap_m,
	float max_angle_diff,
	float max_collinear_error_m,
	float max_segment_gap_m) const {

	// clang-format on

	ProcessedLidarData result;
	result.timestamp_us = data.timestamp_us;
	std::vector<CartesianPoint> points;

	for (const auto &point : data.points) {
		if (!is_valid_point(point)) continue;

		points.push_back(polar2cartesian(point));
	}
	if (points.empty()) return result;

	const auto point_segments = split_line_segments(
		points, max_line_error_m, max_point_gap_m, min_segment_point);

	std::vector<LineSegment> segments;
	segments.reserve(point_segments.size());

	for (const auto &point_segment : point_segments) {
		const auto line_segment = fit_line_segment(point_segment);

		if (!line_segment.has_value()) continue;

		if (line_segment->rms_error_m > max_line_error_m) continue;

		segments.push_back(*line_segment);
	}

	float MAX_ANGLE_DIFF_RAD =
		max_angle_diff * static_cast<float>(M_PI) / 180.0f;

	merge_aligned_segments(
		segments, MAX_ANGLE_DIFF_RAD, max_collinear_error_m, max_segment_gap_m);

	const ResolvedWalls walls = resolve_track_walls(segments);

	const auto obstacles = detect_obstacles(points, walls);

	const auto parking = find_parking_wall(segments, walls);

	result.walls = walls;

	result.obstacles = obstacles;

	result.line_segments = std::move(segments);

	return result;
}

bool LidarProcessor::is_valid_point(const LidarPoint &point) const {
	if (point.quality < 1) return false;
	if (point.distance_m < 0.01f) return false;

	// const bool front_region =
	// 	point.angle_deg >= 75.0f && point.angle_deg <= 285.0f;

	// if (!front_region && point.distance_m > 0.70f) return false;

	return true;
}

CartesianPoint LidarProcessor::polar2cartesian(const LidarPoint &point) const {
	CartesianPoint result;
	result.distance_m = point.distance_m;
	const float rad = point.angle_deg * static_cast<float>(M_PI) / 180.f;
	result.x_m = point.distance_m * -std::sin(rad);
	result.y_m = point.distance_m * -std::cos(rad);

	return result;
}

// clang-format on
std::vector<std::vector<CartesianPoint>> LidarProcessor::split_line_segments(
	const std::vector<CartesianPoint> &points, float max_line_error_m,
	float max_point_gap_m, std::size_t min_points) const {

	std::vector<std::vector<CartesianPoint>> segments;

	if (points.size() < min_points) {
		return segments;
	}

	split_line_segments_recursive(points, 0, points.size() - 1,
		max_line_error_m, max_point_gap_m, min_points, segments);

	return segments;
}

void LidarProcessor::split_line_segments_recursive(
	const std::vector<CartesianPoint> &points, std::size_t start,
	std::size_t end, float max_line_error_m, float max_point_gap_m,
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

		const float dx = points[i + 1].x_m - points[i].x_m;
		const float dy = points[i + 1].y_m - points[i].y_m;

		const float gap_sq = dx * dx + dy * dy;
		const float max_gap_sq = max_point_gap_m * max_point_gap_m;

		if (gap_sq > max_gap_sq) {

			split_line_segments_recursive(points, start, i, max_line_error_m,
				max_point_gap_m, min_points, segments);

			split_line_segments_recursive(points, i + 1, end, max_line_error_m,
				max_point_gap_m, min_points, segments);

			return;
		}
	}

	// find Point that far from line(start to end)
	const float line_length_sq = dx * dx + dy * dy;

	if (line_length_sq < 1e-12f) {
		return;
	}

	float max_numerator_sq = 0.0f;
	std::size_t split_index = start;

	const float line_c = last.x_m * first.y_m - last.y_m * first.x_m;

	for (std::size_t i = start + 1; i < end; ++i) {

		const float numerator =
			dy * points[i].x_m - dx * points[i].y_m + line_c;

		const float numerator_sq = numerator * numerator;

		if (numerator_sq > max_numerator_sq) {
			max_numerator_sq = numerator_sq;
			split_index = i;
		}
	}

	const float max_error_sq = max_line_error_m * max_line_error_m;

	if (max_numerator_sq > max_error_sq * line_length_sq &&
		split_index > start && split_index < end) {
		split_line_segments_recursive(points, start, split_index,
			max_line_error_m, max_point_gap_m, min_points, segments);

		split_line_segments_recursive(points, split_index, end,
			max_line_error_m, max_point_gap_m, min_points, segments);

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
void LidarProcessor::merge_aligned_segments(std::vector<LineSegment> &segments,
	float max_angle_diff_rad, float max_collinear_error_m,
	float max_gap_m) const {

	if (segments.size() < 2) {
		return;
	}

	std::vector<bool> removed(segments.size(), false);

	for (std::size_t i = 0; i < segments.size(); ++i) {
		if (removed[i]) {
			continue;
		}
		for (std::size_t j = i + 1; j < segments.size(); ++j) {
			if (removed[j]) {
				continue;
			}

			auto &a = segments[i];
			const auto &b = segments[j];

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
			const cv::Point2f dir(std::cos(a.angle_rad), std::sin(a.angle_rad));

			auto projection = [&](const cv::Point2f &p) {
				return p.x * dir.x + p.y * dir.y;
			};

			const cv::Point2f origin = a.start;

			float min_proj = 0.0f;
			float max_proj = (a.end - origin).dot(dir);

			if (min_proj > max_proj) {
				std::swap(min_proj, max_proj);
			}

			auto extend_projection = [&](const cv::Point2f &p) {
				const float t = (p - origin).dot(dir);

				min_proj = std::min(min_proj, t);

				max_proj = std::max(max_proj, t);
			};

			extend_projection(b.start);
			extend_projection(b.end);

			a.start = origin + dir * min_proj;

			a.end = origin + dir * max_proj;
			removed[j] = true;
		}
	}

	std::vector<LineSegment> merged;
	merged.reserve(segments.size());

	for (std::size_t i = 0; i < segments.size(); ++i) {
		if (!removed[i]) {
			merged.push_back(segments[i]);
		}
	}

	segments = std::move(merged);
}

std::optional<LineSegment> LidarProcessor::fit_line_segment(
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

	LineSegment result;
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
	auto project_to_line = [&](const CartesianPoint &p) -> cv::Point2f {
		const float d = normal_x * p.x_m + normal_y * p.y_m + c;
		return cv::Point2f(p.x_m - d * normal_x, p.y_m - d * normal_y);
	};

	result.start = project_to_line(points.front());
	result.end = project_to_line(points.back());

	result.angle_rad = theta;

	result.normal_x = normal_x;
	result.normal_y = normal_y;

	result.line_c = c;

	result.rms_error_m = std::sqrt(error_sum / n);

	return result;
}

ResolvedWalls LidarProcessor::resolve_track_walls(
	const std::vector<LineSegment> &segments) const {

	ResolvedWalls result;

	constexpr float MIN_WALL_LENGTH_M = 0.25f;

	constexpr float MAX_ANGLE_ERROR_RAD =
		15.0f * static_cast<float>(M_PI) / 180.0f;

	for (const auto &segment : segments) {

		if (segment.length() < MIN_WALL_LENGTH_M) continue;

		const cv::Point2f center = (segment.start + segment.end) * 0.5f;
		const float side_angle_error = std::abs(
			std::abs(segment.angle_rad) - static_cast<float>(M_PI) * 0.5f);

		if (side_angle_error < MAX_ANGLE_ERROR_RAD) {

			if (center.x < 0.0f) {

				// LEFT
				if (!result.left.has_value() ||
					segment.perpendicular_distance() <
						std::abs(result.left->line_c)) {

					result.left = segment;
				}

			} else {

				// RIGHT
				if (!result.right.has_value() ||
					segment.perpendicular_distance() <
						std::abs(result.right->line_c)) {

					result.right = segment;
				}
			}

			continue;
		}
		const float front_angle_error = std::abs(segment.angle_rad);

		if (front_angle_error < MAX_ANGLE_ERROR_RAD && center.y > 0.0f) {

			if (!result.front.has_value() ||
				segment.perpendicular_distance() <
					std::abs(result.front->line_c)) {

				result.front = segment;
			}
		}
	}
	return result;
}

bool LidarProcessor::is_same_segment(
	const LineSegment &a, const std::optional<LineSegment> &b) const {

	if (!b.has_value()) {
		return false;
	}

	constexpr float EPSILON_M = 0.001f;

	auto same_point = [](const cv::Point2f &p1, const cv::Point2f &p2) {
		const float dx = p1.x - p2.x;
		const float dy = p1.y - p2.y;

		return dx * dx + dy * dy < EPSILON_M * EPSILON_M;
	};

	// ปกติ
	if (same_point(a.start, b->start) && same_point(a.end, b->end)) {
		return true;
	}

	// start/end กลับด้าน
	if (same_point(a.start, b->end) && same_point(a.end, b->start)) {
		return true;
	}

	return false;
}

bool LidarProcessor::is_wall_fragment(
	const LineSegment &segment, const std::optional<LineSegment> &wall) const {

	if (!wall.has_value()) {
		return false;
	}

	constexpr float MAX_WALL_DISTANCE_M = 0.04f;
	constexpr float MAX_ANGLE_DIFF_RAD =
		10.0f * static_cast<float>(M_PI) / 180.0f;

	const cv::Point2f center = (segment.start + segment.end) * 0.5f;

	const float distance_to_wall = std::abs(
		wall->normal_x * center.x + wall->normal_y * center.y + wall->line_c);

	if (distance_to_wall > MAX_WALL_DISTANCE_M) {
		return false;
	}

	if (segment.length() < 0.10f) {
		return true;
	}

	float angle_diff = std::abs(segment.angle_rad - wall->angle_rad);

	angle_diff = std::fmod(angle_diff, static_cast<float>(M_PI));

	if (angle_diff > static_cast<float>(M_PI) * 0.5f) {
		angle_diff = static_cast<float>(M_PI) - angle_diff;
	}

	return angle_diff < MAX_ANGLE_DIFF_RAD;
}

std::vector<ObstacleObject> LidarProcessor::detect_obstacles(
	const std::vector<CartesianPoint> &points,
	const ResolvedWalls &walls) const {

	std::vector<ObstacleObject> obstacles;

	constexpr float WALL_REJECT_DISTANCE_M = 0.05f;
	constexpr float WALL_EXTENSION_M = 0.08f;

	constexpr float CLUSTER_GAP_M = 0.07f;

	constexpr std::size_t MIN_CLUSTER_POINTS = 3;

	constexpr float MIN_OBSTACLE_WIDTH_M = 0.035f;
	constexpr float MAX_OBSTACLE_WIDTH_M = 0.07f;

	constexpr float MIN_FORWARD_M = 0.03f;

	std::vector<CartesianPoint> candidates;
	candidates.reserve(points.size());

	auto point_near_wall = [&](const CartesianPoint &point,
							   const std::optional<LineSegment> &wall) {
		if (!wall.has_value()) {
			return false;
		}

		const cv::Point2f p{point.x_m, point.y_m};

		const cv::Point2f ab = wall->end - wall->start;

		const float length_sq = ab.dot(ab);

		if (length_sq < 1e-8f) {
			return false;
		}

		const float length = std::sqrt(length_sq);

		// Projection along the finite wall.
		float t = (p - wall->start).dot(ab) / length_sq;

		// Allow a small extension beyond both endpoints.
		const float extension_t = WALL_EXTENSION_M / length;

		if (t < -extension_t || t > 1.0f + extension_t) {

			return false;
		}

		// Clamp only for closest-point calculation.
		t = std::clamp(t, 0.0f, 1.0f);

		const cv::Point2f closest = wall->start + ab * t;

		const float distance = cv::norm(p - closest);

		return distance < WALL_REJECT_DISTANCE_M;
	};

	for (const auto &point : points) {

		if (point.y_m <= MIN_FORWARD_M) {
			continue;
		}

		if (point_near_wall(point, walls.left)) {
			continue;
		}

		if (point_near_wall(point, walls.right)) {
			continue;
		}

		if (point_near_wall(point, walls.front)) {
			continue;
		}

		candidates.push_back(point);
	}

	if (candidates.empty()) {
		return obstacles;
	}

	std::vector<CartesianPoint> cluster;
	cluster.reserve(32);

	auto finish_cluster = [&]() {
		if (cluster.size() < MIN_CLUSTER_POINTS) {

			cluster.clear();
			return;
		}

		cv::Point2f center{0.0f, 0.0f};

		for (const auto &point : cluster) {

			center.x += point.x_m;
			center.y += point.y_m;
		}

		const float count = static_cast<float>(cluster.size());

		center.x /= count;
		center.y /= count;

		float max_distance_sq = 0.0f;

		cv::Point2f extent_start{cluster.front().x_m, cluster.front().y_m};

		cv::Point2f extent_end = extent_start;

		for (std::size_t i = 0; i < cluster.size(); ++i) {

			const cv::Point2f a{cluster[i].x_m, cluster[i].y_m};

			for (std::size_t j = i + 1; j < cluster.size(); ++j) {

				const cv::Point2f b{cluster[j].x_m, cluster[j].y_m};

				const cv::Point2f diff = b - a;

				const float distance_sq = diff.dot(diff);

				if (distance_sq > max_distance_sq) {

					max_distance_sq = distance_sq;

					extent_start = a;
					extent_end = b;
				}
			}
		}

		const float width = std::sqrt(max_distance_sq);

		if (width < MIN_OBSTACLE_WIDTH_M || width > MAX_OBSTACLE_WIDTH_M) {

			cluster.clear();
			return;
		}

		ObstacleObject obstacle;

		obstacle.center = center;

		obstacle.width_m = width;

		obstacle.angle_rad = std::atan2(
			extent_end.y - extent_start.y, extent_end.x - extent_start.x);

		obstacles.push_back(obstacle);

		cluster.clear();
	};

	for (const auto &point : candidates) {

		if (cluster.empty()) {

			cluster.push_back(point);
			continue;
		}

		const auto &previous = cluster.back();

		const float dx = point.x_m - previous.x_m;

		const float dy = point.y_m - previous.y_m;

		const float distance_sq = dx * dx + dy * dy;

		if (distance_sq > CLUSTER_GAP_M * CLUSTER_GAP_M) {

			finish_cluster();
		}

		cluster.push_back(point);
	}

	// Last cluster
	finish_cluster();

	return obstacles;
}

std::optional<LineSegment> LidarProcessor::find_parking_wall(
	const std::vector<LineSegment> &segments,
	const ResolvedWalls &walls) const {

	constexpr float MIN_PARKING_WALL_LENGTH_M = 0.10f;

	constexpr float MAX_PERPENDICULAR_ERROR_RAD =
		15.0f * static_cast<float>(M_PI) / 180.0f;

	std::optional<LineSegment> best_wall;

	for (const auto &segment : segments) {

		if (is_same_segment(segment, walls.left) ||
			is_same_segment(segment, walls.right) ||
			is_same_segment(segment, walls.front)) {
			continue;
		}

		if (is_wall_fragment(segment, walls.left) ||
			is_wall_fragment(segment, walls.right) ||
			is_wall_fragment(segment, walls.front)) {
			continue;
		}

		if (segment.length() < MIN_PARKING_WALL_LENGTH_M) {
			continue;
		}

		const cv::Point2f center = (segment.start + segment.end) * 0.5f;

		if (center.y <= 0.0f) {
			continue;
		}

		bool perpendicular_to_side = false;

		auto check_side = [&](const std::optional<LineSegment> &side) {
			if (!side.has_value()) {
				return;
			}

			float angle_diff = std::abs(segment.angle_rad - side->angle_rad);

			angle_diff = std::fmod(angle_diff, static_cast<float>(M_PI));

			if (angle_diff > static_cast<float>(M_PI) * 0.5f) {

				angle_diff = static_cast<float>(M_PI) - angle_diff;
			}

			const float error =
				std::abs(angle_diff - static_cast<float>(M_PI) * 0.5f);

			if (error < MAX_PERPENDICULAR_ERROR_RAD) {
				perpendicular_to_side = true;
			}
		};

		check_side(walls.left);
		check_side(walls.right);

		if (!perpendicular_to_side) {
			continue;
		}

		if (!best_wall.has_value()) {
			best_wall = segment;
			continue;
		}

		// Prefer longer candidate
		if (segment.length() > best_wall->length()) {
			best_wall = segment;
		}
	}

	return best_wall;
}

void LidarProcessor::draw_segment(
	cv::Mat &img, const LineSegment &segment, float scale_px_per_m) const {
	if (img.empty()) {
		return;
	}

	// LiDAR origin at center of image
	const cv::Point2f origin(static_cast<float>(img.cols) * 0.5f,
		static_cast<float>(img.rows) * 0.5f);

	auto world_to_pixel = [&](const cv::Point2f &point_m) -> cv::Point {
		const float pixel_x = origin.x + point_m.x * scale_px_per_m;

		const float pixel_y = origin.y - point_m.y * scale_px_per_m;

		return cv::Point(static_cast<int>(std::lround(pixel_x)),
			static_cast<int>(std::lround(pixel_y)));
	};

	const cv::Point start_px = world_to_pixel(segment.start);
	const cv::Point end_px = world_to_pixel(segment.end);

	// Draw fitted / merged segment
	cv::line(img, start_px, end_px, cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
}

} // namespace lidar