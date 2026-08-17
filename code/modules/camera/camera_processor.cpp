#include "camera_processor.hpp"
#include <opencv2/core/mat.hpp>
#include <opencv2/imgproc.hpp>
#include <vector>

namespace camera {
MaskColor CameraProcessor::mask_filter(const TimedFrameData &timedFrame) const {
	const cv::Mat &frame = TimedFrameData.frame;

	CV_Assert(!frame.empty());

	cv::Mat hsvImage;
	cv::cvtColor(frame, hsvImage, cv::COLOR_BGR2HSV);

	cv::Mat Redmask1, Redmask2, Redmask;
	cv::inRange(hsvImage, LOWER_RED_1, UPPER_RED_1, Redmask1);
	cv::inRange(hsvImage, LOWER_RED_2, UPPER_RED_2, Redmask2);
	Redmask = Redmask1 | Redmask2;

	cv::Mat Greenmask;
	cv::inRange(hsvImage, LOWER_GREEN, UPPER_GREEN, Greenmask);

	cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));

	// morph
	cv::morphologyEx(redMask, redMask, cv::MORPH_OPEN, kernel);
	cv::morphologyEx(redMask, redMask, cv::MORPH_CLOSE, kernel);

	cv::morphologyEx(greenMask, greenMask, cv::MORPH_OPEN, kernel);
	cv::morphologyEx(greenMask, greenMask, cv::MORPH_CLOSE, kernel);

	return {Redmask, Greenmask};
}

std::vector<CameraObject> CameraProcessor::extract_objects(
	const cv::Mat &mask, Color color) const {}

// clang-format off
bool CameraProcessor::is_valid_contour(const std::vector<cv::Point> &contour, const cv::Rect &bounding_box) const {

	// clang-format on
	const double area = cv::contourArea(contour);

	if (area < min_contour_area_) {
		return false;
	}

	if (rect.width < min_width_ || rect.height < min_height_) {
		return false;
	}

	const float aspect_ratio =
		static_cast<float>(rect.width) / static_cast<float>(rect.height);

	if (aspect_ratio < min_aspect_ratio || aspect_ratio > max_aspect_ratio) {
		return false;
	}

	const float rect_area = static_cast<float>(rect.width * rect.height);
	const float fill_ratio = static_cast<float>(area) / rect_area;

	if (fill_ratio < min_fill_ratio_) {
		return false;
	}

	return true;
}
cv::Point2f CameraProcessor::calculate_bottom_center(
	const cv::Rect &bounding_box) const {}

float CameraProcessor::calculate_bearing(float pixel_x) const {}
} // namespace camera