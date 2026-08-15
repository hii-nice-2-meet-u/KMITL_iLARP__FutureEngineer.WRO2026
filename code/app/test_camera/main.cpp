#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>
#include <opencv2/opencv.hpp>

#include "camera_module.hpp"
#include "camera_processor.hpp"

bool findFirstRectangle(
	const cv::Mat &inputImage, std::vector<cv::Rect> &outputRects) {
	cv::Mat thresh, blurred;

	cv::medianBlur(inputImage, blurred, 5); // Use separate output variable
	cv::threshold(blurred, thresh, 100, 255, cv::THRESH_BINARY);

	std::vector<std::vector<cv::Point>> contours;

	cv::findContours(
		thresh, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

	outputRects.clear();

	for (const auto &contour : contours) {
		std::vector<cv::Point> approx;

		double peri = cv::arcLength(contour, true);

		// Approximate contour with accuracy proportional to perimeter
		cv::approxPolyDP(contour, approx, 0.02 * peri, true);

		if (approx.size() == 4 && cv::isContourConvex(approx) &&
			cv::contourArea(approx) > 500) {

			outputRects.push_back(cv::boundingRect(approx));
		}
	}

	return !outputRects.empty();
}

int main() {
	std::string folder_path = "/home/hii/captured_wro/test_bb/";

	std::string mkdir_cmd = "mkdir -p " + folder_path;

	system(mkdir_cmd.c_str());

	// init cam
	camera::CameraModule camera_module;

	if (!camera_module.start()) {
		std::cerr << "Failed to start camera!" << std::endl;
		return 1;
	}

	cv::Mat hsvImage;
	cv::Mat mask1;
	cv::Mat mask2;
	cv::Mat redMask;
	cv::Mat redMaskBGR;
	cv::Mat combined;
	cv::Mat green_mask;
	cv::Mat cp_frame;

	std::vector<cv::Rect> red_rects;
	std::vector<cv::Rect> green_rects;
	std::vector<cv::Rect> all_rects;

	int couttttttt = 0;

	while (true) {

		TimedFrameData frame_data;

		if (!camera_module.wait_for_frame(frame_data)) {
			std::cerr << "Failed to get camera frame." << std::endl;
			break;
		}

		cv::Mat frame = frame_data.frame;

		if (frame.empty()) {
			continue;
		}

		cv::cvtColor(frame, hsvImage, cv::COLOR_BGR2HSV);

		// cv::inRange(
		//     hsvImage,
		//     cv::Scalar(0, 70, 50),
		//     cv::Scalar(10, 255, 255),
		//     mask1);

		// cv::inRange(
		//     hsvImage,
		//     cv::Scalar(170, 70, 50),
		//     cv::Scalar(179, 255, 255),
		//     mask2);

		// redMask = mask1 | mask2;

		cv::inRange(hsvImage, cv::Scalar(11, 79, 37), cv::Scalar(22, 255, 255),
			redMask);

		// cv::inRange(
		//     hsvImage,
		//     cv::Scalar(52, 100, 53),
		//     cv::Scalar(116, 255, 255),
		//     green_mask);

		findFirstRectangle(redMask, red_rects);

		// findFirstRectangle(
		//     green_mask,
		//     green_rects);

		all_rects.clear();

		all_rects.insert(all_rects.end(), red_rects.begin(), red_rects.end());

		// all_rects.insert(
		//     all_rects.end(),
		//     green_rects.begin(),
		//     green_rects.end());

		for (const auto &rect : all_rects) {
			cv::rectangle(frame, rect, cv::Scalar(0, 255, 0), 3);
		}

		// cv::rectangle(
		//     frame,
		//     red_Rect,
		//     cv::Scalar(0, 0, 255),
		//     2);

		// cv::rectangle(
		//     frame,
		//     green_Rect,
		//     cv::Scalar(0, 0, 255),
		//     2);

		cv::imshow("Frame", frame);

		char key = static_cast<char>(cv::waitKey(1));

		if (key == 's') {
			std::string file_path =
				folder_path + "img_" + std::to_string(couttttttt) + ".jpg";

			if (cv::imwrite(file_path, frame)) {
				std::cout << "Saved: " << file_path << std::endl;
			} else {
				std::cerr << "Error saving image to " << file_path << std::endl;
			}

			++couttttttt;
		}

		if (key == 'q') {
			camera_module.stop();
			break;
		}
	}

	cv::destroyAllWindows();

	return 0;
}