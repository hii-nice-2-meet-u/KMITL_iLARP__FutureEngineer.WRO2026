#pragma once

#include "camera_struct.hpp"
#include "opencv2/highgui.hpp"

namespace camera {

// Color Ranges (HSV)
inline const cv::Scalar LOWER_RED_1{0, 110, 170};

inline const cv::Scalar UPPER_RED_1{1, 230, 255};

inline const cv::Scalar LOWER_RED_2{166, 70, 170};

inline const cv::Scalar UPPER_RED_2{179, 230, 255};

inline const cv::Scalar LOWER_GREEN{45, 110, 100};

inline const cv::Scalar UPPER_GREEN{85, 230, 200};

// const cv::Scalar lowerMagenta1Light(165, 244, 200);
// const cv::Scalar upperMagenta1Light(171, 255, 255);

// const cv::Scalar lowerMagenta2Light(171, 255, 255);
// const cv::Scalar upperMagenta2Light(171, 255, 255);

enum class Color {
	Red,
	Green
}

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
		const cv::Rect &bounding_box) const;

	cv::Point2f calculate_bottom_center(const cv::Rect &bounding_box) const;

	float calculate_bearing(float pixel_x) const;

  private:
	float fx_{0.0f};
	float cx_{0.0f};

	double min_contour_area_{500.0}; // NOT TEST YET

	int min_width_{10}; //NOT TEST YET
	int min_height_{20};

	float min_aspect_ratio{0.4f};
	float max_aspect_ratio{2.5f};

	float min_fill_ratio_{0.3f}; //not test yet
};

} // namespace camera