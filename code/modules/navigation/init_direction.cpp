#include "init_direction.hpp"
namespace navigation {

namespace {

constexpr float PI = static_cast<float>(M_PI);
constexpr float HALF_PI = PI * 0.5f;

constexpr float EPSILON = 1e-6f;

constexpr float MIN_FORWARD_INTERSECTION_M = 0.05f;

constexpr float MIN_OUTWARD_EXTENSION_M = 0.04f;

constexpr float SAME_SEGMENT_EPSILON_M = 0.001f;

} // namespace

InitialDirectionEstimator::InitialDirectionEstimator(
	InitialDirectionConfig config)
	: config_(config) {}

cv::Point2f InitialDirectionEstimator::forward_endpoint(
	const lidar::LineSegment &segment) {
	return segment.end.y > segment.start.y ? segment.end : segment.start;
}

float InitialDirectionEstimator::angle_difference(float a_rad, float b_rad) {

	float diff = std::fabs(a_rad - b_rad);

	diff = std::fmod(diff, PI);

	if (diff > HALF_PI) {
		diff = PI - diff;
	}

	return std::fabs(diff);
}

float InitialDirectionEstimator::point_to_segment_distance(
	const cv::Point2f &point, const lidar::LineSegment &segment) {

	const cv::Point2f ab = segment.end - segment.start;

	const float length_sq = ab.dot(ab);

	if (length_sq < EPSILON) {
		return cv::norm(point - segment.start);
	}

	const cv::Point2f ap = point - segment.start;

	float t = ap.dot(ab) / length_sq;

	t = std::clamp(t, 0.0f, 1.0f);

	const cv::Point2f closest = segment.start + ab * t;

	return cv::norm(point - closest);
}

std::optional<cv::Point2f> InitialDirectionEstimator::line_intersection(
	const lidar::LineSegment &a, const lidar::LineSegment &b) const {

	const float determinant = a.normal_x * b.normal_y - b.normal_x * a.normal_y;

	if (std::fabs(determinant) < EPSILON) {
		return std::nullopt;
	}

	const float x =
		(a.normal_y * b.line_c - a.line_c * b.normal_y) / determinant;

	const float y =
		(a.line_c * b.normal_x - a.normal_x * b.line_c) / determinant;

	if (!std::isfinite(x) || !std::isfinite(y)) {
		return std::nullopt;
	}

	return cv::Point2f{x, y};
}

bool InitialDirectionEstimator::has_forward_continuation(
	const lidar::LineSegment &side_wall,
	const std::vector<lidar::LineSegment> &segments) const {

	const cv::Point2f forward = forward_endpoint(side_wall);

	cv::Point2f forward_dir = side_wall.end - side_wall.start;

	const float dir_length = cv::norm(forward_dir);

	if (dir_length < EPSILON) {
		return false;
	}

	forward_dir *= 1.0f / dir_length;

	if (forward_dir.y < 0.0f) {
		forward_dir *= -1.0f;
	}

	auto same_segment = [](const lidar::LineSegment &a,
							const lidar::LineSegment &b) {
		const bool same_order =
			cv::norm(a.start - b.start) < SAME_SEGMENT_EPSILON_M &&
			cv::norm(a.end - b.end) < SAME_SEGMENT_EPSILON_M;

		const bool reverse_order =
			cv::norm(a.start - b.end) < SAME_SEGMENT_EPSILON_M &&
			cv::norm(a.end - b.start) < SAME_SEGMENT_EPSILON_M;

		return same_order || reverse_order;
	};

	for (const auto &candidate : segments) {

		if (same_segment(side_wall, candidate)) {
			continue;
		}

		if (candidate.length() < config_.min_candidate_length_m) {
			continue;
		}

		const float angle_error =
			angle_difference(side_wall.angle_rad, candidate.angle_rad);

		if (angle_error > config_.max_collinear_angle_error_rad) {
			continue;
		}

		const cv::Point2f candidate_center =
			(candidate.start + candidate.end) * 0.5f;

		const float collinear_error =
			std::fabs(side_wall.normal_x * candidate_center.x +
				side_wall.normal_y * candidate_center.y + side_wall.line_c);

		if (collinear_error > config_.max_collinear_offset_m) {
			continue;
		}

		const float start_projection =
			(candidate.start - forward).dot(forward_dir);

		const float end_projection = (candidate.end - forward).dot(forward_dir);

		const float max_projection = std::max(start_projection, end_projection);

		if (max_projection <= 0.02f) {
			continue;
		}

		const float gap = point_to_segment_distance(forward, candidate);

		if (gap > config_.max_continuation_gap_m) {
			continue;
		}

		return true;
	}

	return false;
}

const lidar::LineSegment *InitialDirectionEstimator::find_perpendicular_wall(
	const lidar::LineSegment &side_wall, TurnDirection turn,
	const std::vector<lidar::LineSegment> &segments) const {

	const cv::Point2f forward = forward_endpoint(side_wall);

	const cv::Point2f side_center = (side_wall.start + side_wall.end) * 0.5f;

	const bool right_side = side_center.x > 0.0f;

	const bool left_side = side_center.x < 0.0f;

	const lidar::LineSegment *best = nullptr;

	float best_gap = std::numeric_limits<float>::max();

	for (const auto &candidate : segments) {

		if (candidate.length() < config_.min_candidate_length_m) {
			continue;
		}

		const float diff =
			angle_difference(side_wall.angle_rad, candidate.angle_rad);

		const float perpendicular_error = std::fabs(diff - HALF_PI);

		if (perpendicular_error > config_.max_perpendicular_error_rad) {
			continue;
		}

		const cv::Point2f candidate_center =
			(candidate.start + candidate.end) * 0.5f;

		if (candidate_center.y < forward.y - config_.max_connection_gap_m) {
			continue;
		}

		const float endpoint_gap =
			point_to_segment_distance(forward, candidate);

		if (endpoint_gap > config_.max_connection_gap_m * 1.5f) {
			continue;
		}

		const auto intersection = line_intersection(side_wall, candidate);

		if (!intersection.has_value()) {
			continue;
		}

		if (turn == TurnDirection::RIGHT) {

			const float furthest_x =
				std::max(candidate.start.x, candidate.end.x);

			if (furthest_x < intersection->x + MIN_OUTWARD_EXTENSION_M) {
				continue;
			}

		} else if (turn == TurnDirection::LEFT) {

			const float furthest_x =
				std::min(candidate.start.x, candidate.end.x);

			if (furthest_x > intersection->x - MIN_OUTWARD_EXTENSION_M) {
				continue;
			}

		} else {
			continue;
		}

		if (endpoint_gap < best_gap) {
			best_gap = endpoint_gap;
			best = &candidate;
		}
	}

	return best;
}

bool InitialDirectionEstimator::connection_is_valid(
	const lidar::LineSegment &side_wall,
	const lidar::LineSegment &perpendicular_wall,
	const cv::Point2f &intersection) const {

	if (intersection.y < MIN_FORWARD_INTERSECTION_M) {
		return false;
	}

	const cv::Point2f forward = forward_endpoint(side_wall);

	const float side_gap = cv::norm(intersection - forward);

	if (side_gap > config_.max_connection_gap_m) {
		return false;
	}

	const float perpendicular_gap =
		point_to_segment_distance(intersection, perpendicular_wall);

	if (perpendicular_gap > config_.max_connection_gap_m) {
		return false;
	}

	if (intersection.y < forward.y - config_.max_connection_gap_m) {
		return false;
	}

	return true;
}

bool InitialDirectionEstimator::front_wall_supports(
	const lidar::LineSegment &side_wall,
	const std::optional<lidar::LineSegment> &front_wall) const {

	if (!front_wall.has_value()) {
		return false;
	}

	const float diff =
		angle_difference(side_wall.angle_rad, front_wall->angle_rad);

	const float perpendicular_error = std::fabs(diff - HALF_PI);

	if (perpendicular_error > config_.max_perpendicular_error_rad) {
		return false;
	}

	const auto intersection = line_intersection(side_wall, *front_wall);

	if (!intersection.has_value()) {
		return false;
	}

	return connection_is_valid(side_wall, *front_wall, *intersection);
}

InitialDirectionEstimator::TurnEvidence
InitialDirectionEstimator::evaluate_side(
	const std::optional<lidar::LineSegment> &side_wall, TurnDirection turn,
	const std::vector<lidar::LineSegment> &segments,
	const std::optional<lidar::LineSegment> &front_wall) const {

	TurnEvidence evidence{};

	evidence.turn = turn;

	if (!side_wall.has_value()) {
		return evidence;
	}

	const auto &side = *side_wall;

	if (side.length() < config_.min_candidate_length_m) {
		return evidence;
	}

	const cv::Point2f side_center = (side.start + side.end) * 0.5f;

	if (turn == TurnDirection::RIGHT && side_center.x >= 0.0f) {
		return evidence;
	}

	if (turn == TurnDirection::LEFT && side_center.x <= 0.0f) {
		return evidence;
	}

	const cv::Point2f forward = forward_endpoint(side);

	if (forward.y <= 0.05f) {
		return evidence;
	}

	evidence.side_wall_valid = true;
	evidence.forward_endpoint = forward;

	evidence.score += 1.0f;

	evidence.has_forward_continuation =
		has_forward_continuation(side, segments);

	if (evidence.has_forward_continuation) {

		return evidence;
	}

	evidence.score += 2.0f;

	const lidar::LineSegment *perpendicular =
		find_perpendicular_wall(side, turn, segments);

	if (perpendicular == nullptr) {
		return evidence;
	}

	evidence.perpendicular_wall_found = true;

	evidence.score += 2.0f;

	const auto intersection = line_intersection(side, *perpendicular);

	if (intersection.has_value() &&
		connection_is_valid(side, *perpendicular, *intersection)) {

		evidence.connection_valid = true;

		evidence.score += 2.0f;
	}

	if (front_wall_supports(side, front_wall)) {

		evidence.front_wall_support = true;

		evidence.score += 1.0f;
	}

	return evidence;
}

std::optional<DrivingDirection> InitialDirectionEstimator::update(
	const lidar::ProcessedLidarData &data) {

	const TurnEvidence cw = evaluate_side(data.walls.left, TurnDirection::RIGHT,
		data.line_segments, data.walls.front);

	const TurnEvidence ccw = evaluate_side(data.walls.right,
		TurnDirection::LEFT, data.line_segments, data.walls.front);

	const float decay = std::clamp(config_.score_decay, 0.0f, 1.0f);

	clockwise_score_ *= decay;
	counter_clockwise_score_ *= decay;

	clockwise_score_ += cw.score;
	counter_clockwise_score_ += ccw.score;

	const bool cw_geometry_valid = cw.side_wall_valid &&
		!cw.has_forward_continuation && cw.perpendicular_wall_found &&
		cw.connection_valid;

	const bool ccw_geometry_valid = ccw.side_wall_valid &&
		!ccw.has_forward_continuation && ccw.perpendicular_wall_found &&
		ccw.connection_valid;

	const bool cw_wins = cw_geometry_valid &&
		cw.score >= config_.frame_min_score &&
		cw.score > ccw.score + config_.frame_score_margin;

	const bool ccw_wins = ccw_geometry_valid &&
		ccw.score >= config_.frame_min_score &&
		ccw.score > cw.score + config_.frame_score_margin;

	if (cw_wins) {

		++clockwise_confirm_frames_;
		counter_clockwise_confirm_frames_ = 0;

	} else if (ccw_wins) {

		++counter_clockwise_confirm_frames_;
		clockwise_confirm_frames_ = 0;

	} else {

		clockwise_confirm_frames_ = 0;
		counter_clockwise_confirm_frames_ = 0;
	}

	const int required_frames = std::max(1, config_.required_confirm_frames);

	if (clockwise_confirm_frames_ >= required_frames &&
		clockwise_score_ >
			counter_clockwise_score_ + config_.frame_score_margin) {

		return DrivingDirection::CLOCKWISE;
	}

	if (counter_clockwise_confirm_frames_ >= required_frames &&
		counter_clockwise_score_ >
			clockwise_score_ + config_.frame_score_margin) {

		return DrivingDirection::COUNTER_CLOCKWISE;
	}

	return std::nullopt;
}

void InitialDirectionEstimator::reset() {

	clockwise_score_ = 0.0f;
	counter_clockwise_score_ = 0.0f;

	clockwise_confirm_frames_ = 0;
	counter_clockwise_confirm_frames_ = 0;
}

} // namespace navigation