#include "camera_processor.hpp"

namespace camera {

ProcessedCameraData CameraProcessor::process(
	const TimedFrameData &timed_frame) const {
	const MaskColor masks = mask_filter(timed_frame);

	std::vector<CameraObject> objects;
	auto red_objects = extract_objects(masks.red, Color::Red);

	auto green_objects = extract_objects(masks.green, Color::Green);

	objects.insert(objects.end(), red_objects.begin(), red_objects.end());

	objects.insert(objects.end(), green_objects.begin(), green_objects.end());

	return {timed_frame.timestamp_us, std::move(objects)};
}

MaskColor CameraProcessor::mask_filter(const TimedFrameData &timedFrame) const {
	const cv::Mat &frame = timedFrame.frame;

	CV_Assert(!frame.empty() && frame.type() == CV_8UC3);

	cv::Mat hsvImage;
	cv::cvtColor(frame, hsvImage, cv::COLOR_BGR2HSV);

	cv::Mat redmask1, redmask2, redmask;
	cv::inRange(hsvImage, LOWER_RED_1, UPPER_RED_1, redmask1);
	cv::inRange(hsvImage, LOWER_RED_2, UPPER_RED_2, redmask2);
	redmask = redmask1 | redmask2;

	cv::Mat greenmask;
	cv::inRange(hsvImage, LOWER_GREEN, UPPER_GREEN, greenmask);

	cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(4, 4));

	// morph
	cv::morphologyEx(redmask, redmask, cv::MORPH_OPEN, kernel);
	cv::morphologyEx(redmask, redmask, cv::MORPH_CLOSE, kernel);

	cv::morphologyEx(greenmask, greenmask, cv::MORPH_OPEN, kernel);
	cv::morphologyEx(greenmask, greenmask, cv::MORPH_CLOSE, kernel);

	return {redmask, greenmask};
}

std::vector<CameraObject> CameraProcessor::extract_objects(
	const cv::Mat &mask, Color color) const {
	std::vector<std::vector<cv::Point>> contours;

	cv::findContours(
		mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

	std::vector<CameraObject> objects;

	for (const auto &contour : contours) {

		const cv::Rect rect = cv::boundingRect(contour);

		if (!is_valid_contour(contour, rect)) {
			continue;
		}

		CameraObject object;

		object.color = color;
		object.bounding_box = rect;

		object.bottom_center =
			cv::Point2f(rect.x + rect.width * 0.5, rect.y + rect.height);

		object.bearing_rad = calculate_bearing(object.bottom_center.x);

		objects.push_back(object);
	}

	return objects;
}

bool CameraProcessor::is_valid_contour(
	const std::vector<cv::Point> &contour, const cv::Rect &b_b) const {

	const double area = cv::contourArea(contour);

	if (area < min_contour_area) return false;

	if (b_b.width < min_width || b_b.height < min_height) return false;

	const float aspect_ratio =
		static_cast<float>(b_b.width) / static_cast<float>(b_b.height);

	if (aspect_ratio < min_aspect_ratio || aspect_ratio > max_aspect_ratio) {
		return false;
	}

	const float rect_area = static_cast<float>(b_b.width * b_b.height);
	const float fill_ratio = static_cast<float>(area) / rect_area;

	if (fill_ratio < min_fill_ratio) return false;

	return true;
}


float CameraProcessor::calculate_bearing(float pixel_x) const {
	return std::atan((pixel_x - cx) / fx);
}
} // namespace camera