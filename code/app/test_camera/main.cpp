#include <chrono>
#include <iomanip>
#include <iostream>
#include <random>
#include <sstream>

#include "lccv.hpp"
#include "libcamera/controls.h"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"
#include <opencv2/opencv.hpp>
#include <string>

std::string generateHexID() {
	std::random_device rd;
	std::mt19937 gen(rd());
	std::uniform_int_distribution<uint32_t> dis(0, 0xFFFFFFFF);

	std::stringstream ss;
	ss << std::hex << std::setw(8) << std::setfill('0') << dis(gen);
	return ss.str();
}

bool findFirstRectangle(
	const cv::Mat &inputImage, std::vector<cv::Rect> &outputRects) {
	cv::Mat thresh;
	cv::medianBlur(inputImage, inputImage, 5);
	cv::threshold(inputImage, thresh, 100, 255, cv::THRESH_BINARY);

	std::vector<std::vector<cv::Point>> contours;
	cv::findContours(
		thresh, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

	std::vector<cv::Point> approx;
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
	return false; // No rectangle found
}

int main() {
	//
	std::string folder_path = "/home/hii/captured_wro/test_bb/";

	std::string mkdir_cmd = "mkdir -p " + folder_path;
	system(mkdir_cmd.c_str());

	// init cam
	lccv::PiCamera cam;
	cam.options->video_width = 1920;
	cam.options->video_height = 1080;
	cam.options->framerate = 90;
	cam.options->verbose = true;
	// cam.options->setWhiteBalance(WB_INDOOR);
	// cam.options->awb_gain_r = 1.0f;
	// cam.options->awb_gain_b = 1.0f;

	libcamera::ControlList &controls = cam.getControlList();
	controls.set(
		libcamera::controls::AfMode, libcamera::controls::AfModeContinuous);

	if (!cam.startVideo()) {
		std::cerr << "Failed to start video!" << std::endl;
		return -1;
	}

	cv::Mat frame, hsvImage, mask1, mask2, redMask, redMaskBGR, combined,
		green_mask, cp_frame;
	std::vector<cv::Rect> red_rects, green_rects, all_rects;
	// cv::namedWindow("Frame vs Red Mask", cv::WINDOW_NORMAL);
	int couttttttt = 0;
	while (true) {
		if (cam.getVideoFrame(frame, 1000)) {
			cv::flip(frame, frame, 0);
			cv::flip(frame, frame, 1);
			cv::cvtColor(frame, hsvImage, cv::COLOR_BGR2HSV);
			// cp_frame = frame.clone();
			// cv::inRange(hsvImage, cv::Scalar(0, 70, 50),
			// 	cv::Scalar(10, 255, 255), mask1);
			// cv::inRange(hsvImage, cv::Scalar(170, 70, 50),
			// 	cv::Scalar(179, 255, 255), mask2);
			// redMask = mask1 | mask2;

			cv::inRange(hsvImage, cv::Scalar(11, 79, 37),
				cv::Scalar(22, 255, 255), redMask);

			// cv::inRange(hsvImage, cv::Scalar(52, 100, 53),
				// cv::Scalar(116, 255, 255), green_mask);

			findFirstRectangle(redMask, red_rects);
			// findFirstRectangle(green_mask, green_rects);
			all_rects.clear();
			all_rects.insert(
				all_rects.end(), red_rects.begin(), red_rects.end());
			// all_rects.insert(
				// all_rects.end(), green_rects.begin(), green_rects.end());

			for (const auto &rect : all_rects) {
				cv::rectangle(frame, rect, cv::Scalar(0, 255, 0), 3);
			}
			// cv::rectangle(frame, red_Rect, cv::Scalar(0, 0, 255), 2);
			// cv::rectangle(frame, green_Rect, cv::Scalar(0, 0, 255), 2);
			cv::imshow("Frame ", frame);
			// cv::imshow("Red Mask", redMask);
			// cv::imshow("Green Mask", green_mask);
		}
		char key = (char)cv::waitKey(1);
		if (key == 's') {
			// std::string hex_id = generateHexID();
			std::string file_path =
				folder_path + "img_" + std::to_string(couttttttt) + ".jpg";

			if (cv::imwrite(file_path, frame)) {
				std::cout << "Saved: " << file_path << std::endl;
			} else {
				std::cerr << "Error saving image to " << file_path << std::endl;
			}
			couttttttt++;
		}

		if (key == 'q')
			break;
	}

	cam.stopVideo();
	cv::destroyAllWindows();
	return 0;
}