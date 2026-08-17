#pragma once

#include <opencv2/core/hal/interface.h>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/imgproc.hpp>
#include <vector>

#include "camera_struct.hpp"

namespace camera {

// Color Ranges (HSV)
inline const cv::Scalar LOWER_RED_1{0, 70, 50};
inline const cv::Scalar UPPER_RED_1{10, 255, 255};

inline const cv::Scalar LOWER_RED_2{170, 70, 50};
inline const cv::Scalar UPPER_RED_2{179, 255, 255};

inline const cv::Scalar LOWER_GREEN{20, 180, 66};
inline const cv::Scalar UPPER_GREEN{85, 255, 190};

enum class Color { Red, Green };

struct CameraTraffic {
	float angle;
	Color color;
};

struct MaskColor {
	cv::Mat red;
	cv::Mat green;
};

struct CameraObject {
	Color color;

	cv::Rect bounding_box;
	cv::Point2f bottom_center;

	float bearing_deg{0.0f};
};

struct ProcessedCameraData {
	std::uint64_t timestamp_us{0};

	std::vector<CameraObject> objects;
};
class CameraProcessor {
  public:
	CameraProcessor() = default;

	ProcessedCameraData process(const TimedFrameData &timed_frame) const;

  private:
	MaskColor mask_filter(const TimedFrameData &timedFrame) const;

	std::vector<CameraObject> extract_objects(
		const cv::Mat &mask, Color color) const;

	bool is_valid_contour(const std::vector<cv::Point> &contour,
		const cv::Rect &b_b) const;

	cv::Point2f calculate_bottom_center(const cv::Rect &b_b) const;

	float calculate_bearing(float pixel_x) const;

  private:
	float fx{0.0f};
	float cx{0.0f};

	double min_contour_area{500.0}; // NOT TEST YET

	int min_width{10}; // NOT TEST YET
	int min_height{20};

	float min_aspect_ratio{0.4f};
	float max_aspect_ratio{2.5f};

	float min_fill_ratio{0.3f}; // not test yet
};

} // namespace camera