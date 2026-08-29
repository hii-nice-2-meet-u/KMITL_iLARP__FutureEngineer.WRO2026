#include "tunner.hpp"
#include "opencv2/highgui.hpp"

#include <algorithm>
#include <chrono>
#include <iostream>
#include <thread>

namespace camera {

Tunner::Tunner(unsigned int video_width, unsigned int video_height,
	float framerate, float awb_gain_r, float awb_gain_b)
	: video_width_(video_width), video_height_(video_height),
	  framerate_(framerate), awb_gain_r_(awb_gain_r), awb_gain_b_(awb_gain_b),
	  awb_gain_r_x10_(static_cast<int>(awb_gain_r * 10.0f)),
	  awb_gain_b_x10_(static_cast<int>(awb_gain_b * 10.0f)) {

	camera_module_ = std::make_unique<CameraModule>(
		video_width_, video_height_, framerate_, awb_gain_r_, awb_gain_b_);
}

Tunner::~Tunner() {
	stop_camera();
	cv::destroyAllWindows();
}

bool Tunner::ensure_camera_started() {
	if (camera_started_) {
		return true;
	}

	if (!camera_module_) {
		camera_module_ = std::make_unique<CameraModule>(
			video_width_, video_height_, framerate_, awb_gain_r_, awb_gain_b_);
	}

	if (!camera_module_->start()) {
		std::cerr << "[Tunner] Failed to start CameraModule\n";
		return false;
	}

	camera_started_ = true;
	return true;
}

bool Tunner::restart_camera() {
	if (camera_started_ && camera_module_) {
		camera_module_->stop();
		camera_started_ = false;
	}

	camera_module_.reset();

	std::this_thread::sleep_for(std::chrono::milliseconds(150));

	camera_module_ = std::make_unique<CameraModule>(
		video_width_, video_height_, framerate_, awb_gain_r_, awb_gain_b_);

	return ensure_camera_started();
}

void Tunner::stop_camera() {
	if (!camera_started_ || !camera_module_) {
		return;
	}

	camera_module_->stop();
	camera_started_ = false;
}

void Tunner::run() {
	if (!ensure_camera_started()) {
		return;
	}

	const std::string white_window = "white_tunner_window";
	const std::string red_window = "red_tunner_window";
	const std::string green_window = "green_tunner_window";

	cv::namedWindow(white_window, cv::WINDOW_NORMAL);
	cv::namedWindow(red_window, cv::WINDOW_NORMAL);
	cv::namedWindow(green_window, cv::WINDOW_NORMAL);
	cv::namedWindow(TUNE_MASK_WINDOW, cv::WINDOW_NORMAL);

	cv::resizeWindow(white_window, 500, 180);
	cv::resizeWindow(red_window, 500, 280);
	cv::resizeWindow(green_window, 500, 280);

	cv::resizeWindow(TUNE_MASK_WINDOW, video_width_, video_height_);

	cv::createTrackbar("AWB R x10", white_window, &awb_gain_r_x10_, 50);

	cv::createTrackbar("AWB B x10", white_window, &awb_gain_b_x10_, 50);

	create_hsv_trackbars(red_window, red_hsv_);

	create_hsv_trackbars(green_window, green_hsv_);

	int previous_r = awb_gain_r_x10_;
	int previous_b = awb_gain_b_x10_;

	bool gain_changed = false;

	auto last_gain_change = std::chrono::steady_clock::now();

	while (true) {

		const int current_r =
			std::max(1, cv::getTrackbarPos("AWB R x10", white_window));

		const int current_b =
			std::max(1, cv::getTrackbarPos("AWB B x10", white_window));

		if (current_r != previous_r || current_b != previous_b) {

			previous_r = current_r;
			previous_b = current_b;

			gain_changed = true;

			last_gain_change = std::chrono::steady_clock::now();
		}

		if (gain_changed) {
			const auto elapsed =
				std::chrono::duration_cast<std::chrono::milliseconds>(
					std::chrono::steady_clock::now() - last_gain_change);

			if (elapsed.count() >= 300) {
				awb_gain_r_x10_ = previous_r;
				awb_gain_b_x10_ = previous_b;

				awb_gain_r_ = static_cast<float>(awb_gain_r_x10_) / 10.0f;

				awb_gain_b_ = static_cast<float>(awb_gain_b_x10_) / 10.0f;

				if (!restart_camera()) {
					break;
				}

				gain_changed = false;
				continue;
			}
		}

		TimedFrameData frame_data;

		if (!camera_module_->wait_for_frame(frame_data)) {
			break;
		}

		if (frame_data.frame.empty()) {
			continue;
		}

		cv::Mat result = frame_data.frame.clone();

		result = apply_hsv_overlay(result, red_hsv_, cv::Scalar(0, 0, 255));

		result = apply_hsv_overlay(result, green_hsv_, cv::Scalar(0, 255, 0));

		draw_awb_values(result);

		cv::imshow(TUNE_MASK_WINDOW, result);

		const int key = cv::waitKey(1);

		if (key == 27 || key == 'q') {
			break;
		}

		if (key == 'p') {
			print_awb();

			print_hsv("RED", red_hsv_);

			print_hsv("GREEN", green_hsv_);
		}
	}

	print_awb();
	print_hsv("RED", red_hsv_);
	print_hsv("GREEN", green_hsv_);

	cv::destroyWindow(white_window);
	cv::destroyWindow(red_window);
	cv::destroyWindow(green_window);
	cv::destroyWindow(TUNE_MASK_WINDOW);
}

void Tunner::tune_white() {
	if (!ensure_camera_started()) {
		return;
	}

	const std::string window_name = "white_tunner_window";

	cv::namedWindow(window_name, cv::WINDOW_NORMAL);
	cv::resizeWindow(window_name, 640, 220);

	cv::namedWindow(TUNE_MASK_WINDOW, cv::WINDOW_NORMAL);
	cv::resizeWindow(TUNE_MASK_WINDOW, video_width_, video_height_);

	show_control_panel(window_name, "WHITE BALANCE TUNNER");

	cv::createTrackbar("AWB R x10", window_name, &awb_gain_r_x10_, 50);

	cv::createTrackbar("AWB B x10", window_name, &awb_gain_b_x10_, 50);

	int previous_r = awb_gain_r_x10_;
	int previous_b = awb_gain_b_x10_;

	bool gain_changed = false;
	auto last_gain_change = std::chrono::steady_clock::now();

	while (true) {
		if (cv::getWindowProperty(window_name, cv::WND_PROP_VISIBLE) < 1) {
			break;
		}

		if (cv::getWindowProperty(TUNE_MASK_WINDOW, cv::WND_PROP_VISIBLE) < 1) {
			break;
		}

		const int current_r =
			std::max(1, cv::getTrackbarPos("AWB R x10", window_name));

		const int current_b =
			std::max(1, cv::getTrackbarPos("AWB B x10", window_name));

		if (current_r != previous_r || current_b != previous_b) {
			previous_r = current_r;
			previous_b = current_b;

			gain_changed = true;
			last_gain_change = std::chrono::steady_clock::now();
		}

		if (gain_changed) {
			const auto elapsed =
				std::chrono::duration_cast<std::chrono::milliseconds>(
					std::chrono::steady_clock::now() - last_gain_change);

			if (elapsed.count() >= 300) {
				awb_gain_r_x10_ = previous_r;
				awb_gain_b_x10_ = previous_b;

				awb_gain_r_ = static_cast<float>(awb_gain_r_x10_) / 10.0f;

				awb_gain_b_ = static_cast<float>(awb_gain_b_x10_) / 10.0f;

				std::cout << "[Tunner] Apply AWB" << " R=" << awb_gain_r_
						  << " B=" << awb_gain_b_ << '\n';

				if (!restart_camera()) {
					break;
				}

				gain_changed = false;
				continue;
			}
		}

		TimedFrameData frame_data;

		if (!camera_module_->wait_for_frame(frame_data)) {
			if (!camera_started_) {
				break;
			}
			continue;
		}

		if (frame_data.frame.empty()) {
			continue;
		}

		cv::Mat frame = frame_data.frame.clone();

		draw_awb_values(frame);

		cv::imshow(TUNE_MASK_WINDOW, frame);

		const int key = cv::waitKey(1);

		if (key == 27 || key == 'q') {
			break;
		}

		if (key == 'p') {
			print_awb();
		}
	}

	print_awb();

	cv::destroyWindow(window_name);
	cv::destroyWindow(TUNE_MASK_WINDOW);
}

void Tunner::tune_red_hsv() {
	tune_hsv("red_tunner_window", "RED", red_hsv_, cv::Scalar(0, 0, 255));
}

void Tunner::tune_green_hsv() {
	tune_hsv("green_tunner_window", "GREEN", green_hsv_, cv::Scalar(0, 255, 0));
}

void Tunner::tune_hsv(const std::string &tunner_window_name,
	const std::string &color_name, HSVRange &range,
	const cv::Scalar &overlay_color) {

	if (!ensure_camera_started()) {
		return;
	}

	cv::namedWindow(tunner_window_name, cv::WINDOW_NORMAL);
	cv::resizeWindow(tunner_window_name, 640, 320);

	cv::namedWindow(TUNE_MASK_WINDOW, cv::WINDOW_NORMAL);
	cv::resizeWindow(TUNE_MASK_WINDOW, video_width_, video_height_);

	show_control_panel(tunner_window_name, color_name + " HSV TUNNER");

	create_hsv_trackbars(tunner_window_name, range);

	while (true) {
		if (cv::getWindowProperty(tunner_window_name, cv::WND_PROP_VISIBLE) <
			1) {
			break;
		}

		if (cv::getWindowProperty(TUNE_MASK_WINDOW, cv::WND_PROP_VISIBLE) < 1) {
			break;
		}

		TimedFrameData frame_data;

		if (!camera_module_->wait_for_frame(frame_data)) {
			break;
		}

		if (frame_data.frame.empty()) {
			continue;
		}

		cv::Mat frame = frame_data.frame.clone();

		cv::Mat result = apply_hsv_overlay(frame, range, overlay_color);

		draw_hsv_values(result, range);

		cv::imshow(TUNE_MASK_WINDOW, result);

		const int key = cv::waitKey(1);

		if (key == 27 || key == 'q') {
			break;
		}

		if (key == 'p') {
			print_hsv(color_name, range);
		}
	}

	print_hsv(color_name, range);

	cv::destroyWindow(tunner_window_name);
	cv::destroyWindow(TUNE_MASK_WINDOW);
}

void Tunner::create_hsv_trackbars(
	const std::string &window_name, HSVRange &range) {

	cv::createTrackbar("Hue Min", window_name, &range.h_min, 179);
	cv::createTrackbar("Hue Max", window_name, &range.h_max, 179);

	cv::createTrackbar("Sat Min", window_name, &range.s_min, 255);
	cv::createTrackbar("Sat Max", window_name, &range.s_max, 255);

	cv::createTrackbar("Val Min", window_name, &range.v_min, 255);
	cv::createTrackbar("Val Max", window_name, &range.v_max, 255);
}

cv::Mat Tunner::apply_hsv_overlay(const cv::Mat &frame, const HSVRange &range,
	const cv::Scalar &overlay_color) const {

	cv::Mat hsv;
	cv::Mat mask;

	cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);

	if (range.h_min <= range.h_max) {
		cv::inRange(hsv, cv::Scalar(range.h_min, range.s_min, range.v_min),
			cv::Scalar(range.h_max, range.s_max, range.v_max), mask);
	} else {

		cv::Mat mask_high;
		cv::Mat mask_low;

		cv::inRange(hsv, cv::Scalar(range.h_min, range.s_min, range.v_min),
			cv::Scalar(179, range.s_max, range.v_max), mask_high);

		cv::inRange(hsv, cv::Scalar(0, range.s_min, range.v_min),
			cv::Scalar(range.h_max, range.s_max, range.v_max), mask_low);

		cv::bitwise_or(mask_high, mask_low, mask);
	}

	cv::Mat color_layer(frame.size(), frame.type(), overlay_color);

	cv::Mat blended;

	cv::addWeighted(frame, 0.55, color_layer, 0.45, 0.0, blended);

	cv::Mat result = frame.clone();

	blended.copyTo(result, mask);

	return result;
}

void Tunner::draw_hsv_values(cv::Mat &frame, const HSVRange &range) const {

	const std::string line1 = "H: " + std::to_string(range.h_min) + " - " +
		std::to_string(range.h_max);

	const std::string line2 = "S: " + std::to_string(range.s_min) + " - " +
		std::to_string(range.s_max);

	const std::string line3 = "V: " + std::to_string(range.v_min) + " - " +
		std::to_string(range.v_max);

	cv::putText(frame, line1, cv::Point(20, 30), cv::FONT_HERSHEY_SIMPLEX, 0.6,
		cv::Scalar(255, 255, 255), 2);

	cv::putText(frame, line2, cv::Point(20, 60), cv::FONT_HERSHEY_SIMPLEX, 0.6,
		cv::Scalar(255, 255, 255), 2);

	cv::putText(frame, line3, cv::Point(20, 90), cv::FONT_HERSHEY_SIMPLEX, 0.6,
		cv::Scalar(255, 255, 255), 2);
}

void Tunner::draw_awb_values(cv::Mat &frame) const {

	const std::string text = "AWB R: " + std::to_string(awb_gain_r_) +
		"  AWB B: " + std::to_string(awb_gain_b_);

	cv::putText(frame, text, cv::Point(20, 30), cv::FONT_HERSHEY_SIMPLEX, 0.6,
		cv::Scalar(255, 255, 255), 2);
}

void Tunner::show_control_panel(
	const std::string &window_name, const std::string &title) const {

	cv::Mat panel(120, 640, CV_8UC3, cv::Scalar(30, 30, 30));

	cv::putText(panel, title, cv::Point(20, 35), cv::FONT_HERSHEY_SIMPLEX, 0.7,
		cv::Scalar(255, 255, 255), 2);

	cv::putText(panel, "ESC / Q = exit   P = print values", cv::Point(20, 75),
		cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(255, 255, 255), 1);

	cv::imshow(window_name, panel);
}

void Tunner::print_hsv(const std::string &name, const HSVRange &range) const {

	std::cout << "\n[" << name << " HSV]\n"
			  << "H_MIN = " << range.h_min << '\n'
			  << "H_MAX = " << range.h_max << '\n'
			  << "S_MIN = " << range.s_min << '\n'
			  << "S_MAX = " << range.s_max << '\n'
			  << "V_MIN = " << range.v_min << '\n'
			  << "V_MAX = " << range.v_max << "\n\n";
}

void Tunner::print_awb() const {
	std::cout << "\n[WHITE BALANCE]\n"
			  << "AWB_GAIN_R = " << awb_gain_r_ << '\n'
			  << "AWB_GAIN_B = " << awb_gain_b_ << "\n\n";
}

float Tunner::awb_gain_r() const noexcept { return awb_gain_r_; }

float Tunner::awb_gain_b() const noexcept { return awb_gain_b_; }

const HSVRange &Tunner::red_hsv() const noexcept { return red_hsv_; }

const HSVRange &Tunner::green_hsv() const noexcept { return green_hsv_; }

}
