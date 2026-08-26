#pragma once

#include <cmath>
#include <optional>
#include <vector>

#include <opencv2/core/types.hpp>

#include "direction.hpp"
#include "lidar_processor.hpp"

namespace navigation {

struct InitialDirectionConfig {
	// Side-wall continuation check
	float max_collinear_angle_error_rad{
		8.0f * static_cast<float>(M_PI) / 180.0f};

	float max_collinear_offset_m{0.04f};

	float max_continuation_gap_m{0.20f};

	// Perpendicular wall / corner geometry
	float max_perpendicular_error_rad{
		15.0f * static_cast<float>(M_PI) / 180.0f};

	float max_connection_gap_m{0.25f};

	float min_candidate_length_m{0.15f};

	// Temporal confirmation
	float frame_min_score{5.0f};

	float frame_score_margin{1.5f};

	float score_decay{0.7f};

	int required_confirm_frames{3};
};

class InitialDirectionEstimator {
  public:
	explicit InitialDirectionEstimator(InitialDirectionConfig config = {});

	std::optional<DrivingDirection> update(
		const lidar::ProcessedLidarData &data);

	void reset();

  private:
	struct TurnEvidence {
		TurnDirection turn;

		float score{0.0f};

		bool side_wall_valid{false};

		bool has_forward_continuation{false};

		bool perpendicular_wall_found{false};

		bool connection_valid{false};

		bool front_wall_support{false};

		cv::Point2f forward_endpoint{};
	};

	TurnEvidence evaluate_side(
		const std::optional<lidar::LineSegment> &side_wall, TurnDirection turn,
		const std::vector<lidar::LineSegment> &segments,
		const std::optional<lidar::LineSegment> &front_wall) const;

	bool has_forward_continuation(const lidar::LineSegment &side_wall,
		const std::vector<lidar::LineSegment> &segments) const;

	const lidar::LineSegment *find_perpendicular_wall(
		const lidar::LineSegment &side_wall, TurnDirection turn,
		const std::vector<lidar::LineSegment> &segments) const;

	std::optional<cv::Point2f> line_intersection(
		const lidar::LineSegment &a, const lidar::LineSegment &b) const;

	bool connection_is_valid(const lidar::LineSegment &side_wall,
		const lidar::LineSegment &perpendicular_wall,
		const cv::Point2f &intersection) const;

	bool front_wall_supports(const lidar::LineSegment &side_wall,
		const std::optional<lidar::LineSegment> &front_wall) const;

	static cv::Point2f forward_endpoint(const lidar::LineSegment &segment);

	static float angle_difference(float a_rad, float b_rad);

	static float point_to_segment_distance(
		const cv::Point2f &point, const lidar::LineSegment &segment);

  private:
	InitialDirectionConfig config_;

	float clockwise_score_{0.0f};

	float counter_clockwise_score_{0.0f};

	int clockwise_confirm_frames_{0};

	int counter_clockwise_confirm_frames_{0};
};

} // namespace navigation