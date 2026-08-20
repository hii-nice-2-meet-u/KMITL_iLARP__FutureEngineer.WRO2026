#include "lidar_processor.hpp"
#include <cmath>
#include <iostream>
#include <opencv2/core/types.hpp>

namespace lidar {

ProcessedLidarData LidarProcessor::process(const TimedLidarData &data) const {

	ProcessedLidarData result;
	result.timestamp_us = data.timestamp_us;
	std::vector<CartesianPoint> points;

	for (const auto &point : data.points) {
		if (!is_valid_point(point)) continue;

		points.push_back(polar2cartesian(point));
	}
	if (points.empty()) return result;

	constexpr std::size_t MIN_SEGMENT_POINTS = 5;
	constexpr float MAX_LINE_ERROR_M = 0.035f;
	constexpr float MAX_POINT_GAP_M = 0.11f;

	const auto point_segments = split_line_segments(
		points, MAX_LINE_ERROR_M, MAX_POINT_GAP_M, MIN_SEGMENT_POINTS);

	std::vector<LineSegment> segments;
	segments.reserve(point_segments.size());

	for (const auto &point_segment : point_segments) {
		const auto line_segment = fit_line_segment(point_segment);

		if (!line_segment.has_value()) continue;

		if (line_segment->rms_error_m > MAX_LINE_ERROR_M) continue;

		segments.push_back(*line_segment);
	}

	constexpr float MAX_ANGLE_DIFF_RAD =
		5.0f * static_cast<float>(M_PI) / 180.0f;

	constexpr float MAX_COLLINEAR_ERROR_M = 0.02f;
	constexpr float MAX_SEGMENT_GAP_M = 0.05f;

	merge_aligned_segments(
		segments, MAX_ANGLE_DIFF_RAD, MAX_COLLINEAR_ERROR_M, MAX_SEGMENT_GAP_M);

	const ResolvedWalls walls = resolve_track_walls(segments);

	const auto obstacle_segments = detect_obstacle_segments(segments, walls);

	result.walls = walls;

	result.obstacles = obstacle_segments;

	result.line_segments = std::move(segments);

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
	result.x_m = point.distance_m * std::sin(rad);
	result.y_m = point.distance_m * std::cos(rad);

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

TrackWalls LidarProcessor::resolve_inner_outer(
	const ResolvedWalls &walls, DrivingDirection direction) {

	TrackWalls result;

	result.front = walls.front;

	if (direction == DrivingDirection::CLOCKWISE) {
		result.inner = walls.right;
		result.outer = walls.left;
	} else {
		result.inner = walls.left;
		result.outer = walls.right;
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

std::vector<LineSegment> LidarProcessor::detect_obstacle_segments(
	const std::vector<LineSegment> &segments,
	const ResolvedWalls &walls) const {

	std::vector<LineSegment> obstacles;
	obstacles.reserve(segments.size());

	constexpr float MIN_OBSTACLE_SEGMENT_LENGTH_M = 0.028f;
	constexpr float MAX_OBSTACLE_SEGMENT_LENGTH_M = 0.08f;
	constexpr float SAME_OBSTACLE_DISTANCE_SQ = pow(0.08f, 2);

	for (const auto &segment : segments) {

		const cv::Point2f center = (segment.start + segment.end) * 0.5f;

		const float length = segment.length();

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
		if (length < MIN_OBSTACLE_SEGMENT_LENGTH_M) continue;

		if (length > MAX_OBSTACLE_SEGMENT_LENGTH_M) continue;

		if (center.y <= 0.0f) continue;

		bool merged = false;

		for (auto &existing : obstacles) {

			const cv::Point2f existing_center =
				(existing.start + existing.end) * 0.5f;

			const float center_dx = center.x - existing_center.x;

			const float center_dy = center.y - existing_center.y;

			const float center_distance_sq =
				center_dx * center_dx + center_dy * center_dy;

			if (center_distance_sq > SAME_OBSTACLE_DISTANCE_SQ) {

				continue;
			}

			const cv::Point2f merged_center = (existing_center + center) * 0.5f;

			LineSegment merged_segment;

			if (segment.length() > existing.length()) {
				merged_segment = segment;
			} else {
				merged_segment = existing;
			}

			const cv::Point2f old_center =
				(merged_segment.start + merged_segment.end) * 0.5f;

			const cv::Point2f offset = merged_center - old_center;

			merged_segment.start += offset;
			merged_segment.end += offset;

			merged_segment.line_c =
				-(merged_segment.normal_x * merged_center.x +
					merged_segment.normal_y * merged_center.y);

			existing = merged_segment;

			merged = true;
			break;
		}

		if (!merged) {
			obstacles.push_back(segment);
		}
	}

	return obstacles;
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