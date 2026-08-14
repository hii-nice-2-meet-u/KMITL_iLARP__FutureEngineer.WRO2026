#include <iostream>
#include <string>

#include <lccv.hpp>
#include <opencv2/opencv.hpp>

class HSVRangeHighlighter {
  public:
	explicit HSVRangeHighlighter(
		const std::string &window_name = "HSV Range Highlighter")
		: window_name_(window_name) {
		cv::namedWindow(window_name_, cv::WINDOW_NORMAL);
		cv::resizeWindow(window_name_, 640, 640);

		createHSVTrackbars();
	}

	~HSVRangeHighlighter() { cv::destroyAllWindows(); }

	void adjustHSVImage(const cv::Mat &image) {
		if (image.empty()) {
			std::cerr << "Image is empty\n";
			return;
		}

		while (true) {
			if (cv::getWindowProperty(window_name_, cv::WND_PROP_VISIBLE) < 1) {
				break;
			}

			cv::Mat result = applyHSVMask(image);
			cv::imshow(window_name_, result);

			int key = cv::waitKey(1);

			if (key == 27) { // ESC
				break;
			}
		}
	}

	void adjustHSVVideo(const std::string &video_path) {
		cv::VideoCapture cap(video_path);

		if (!cap.isOpened()) {
			std::cerr << "Failed to open video: " << video_path << '\n';
			return;
		}

		const int total_frames =
			static_cast<int>(cap.get(cv::CAP_PROP_FRAME_COUNT));

		int current_frame = 0;
		bool paused = false;

		if (total_frames > 0) {
			cv::createTrackbar(
				"Position", window_name_, &current_frame, total_frames - 1);
		}

		cv::Mat frame;

		while (true) {
			if (cv::getWindowProperty(window_name_, cv::WND_PROP_VISIBLE) < 1) {
				break;
			}

			if (!paused) {
				if (!cap.read(frame)) {
					break;
				}

				current_frame =
					static_cast<int>(cap.get(cv::CAP_PROP_POS_FRAMES));

				if (total_frames > 0) {
					cv::setTrackbarPos("Position", window_name_,
						std::min(current_frame, total_frames - 1));
				}
			} else {
				// User moved Position slider
				int requested_frame =
					cv::getTrackbarPos("Position", window_name_);

				if (requested_frame != current_frame) {
					current_frame = requested_frame;

					cap.set(cv::CAP_PROP_POS_FRAMES, current_frame);

					cap.read(frame);
				}
			}

			if (!frame.empty()) {
				cv::imshow(window_name_, applyHSVMask(frame));
			}

			int key = cv::waitKey(30);

			if (key == 27) {
				break;
			}

			if (key == ' ') {
				paused = !paused;
			}

			// Easier than relying on OpenCV arrow key codes.
			if (paused && key == 'a') {
				current_frame = std::max(0, current_frame - 1);

				cap.set(cv::CAP_PROP_POS_FRAMES, current_frame);

				cap.read(frame);

				if (total_frames > 0) {
					cv::setTrackbarPos("Position", window_name_, current_frame);
				}
			}

			if (paused && key == 'd') {
				current_frame = std::min(total_frames - 1, current_frame + 1);

				cap.set(cv::CAP_PROP_POS_FRAMES, current_frame);

				cap.read(frame);

				if (total_frames > 0) {
					cv::setTrackbarPos("Position", window_name_, current_frame);
				}
			}
		}

		cap.release();
	}

	void adjustHSVCamera() {
		lccv::PiCamera camera;

		// Start with a resolution suitable for HSV tuning.
		camera.options->video_width = 640;
		camera.options->video_height = 640;
		camera.options->framerate = 100;
		camera.options->awb_gain_r = 1.4f;
		camera.options->awb_gain_b = 2.6f;

		std::cout << "Starting LCCV camera...\n";

		camera.startVideo();

		cv::Mat frame;
		while (true) {
			if (cv::getWindowProperty(window_name_, cv::WND_PROP_VISIBLE) < 1) {
				break;
			}

			if (!camera.getVideoFrame(frame, 1000)) {
				std::cerr << "Failed to get camera frame\n";
				continue;
			}

			if (frame.empty()) {
				continue;
			}
			cv::flip(frame, frame, 0);

			cv::Mat result = applyHSVMask(frame);

			drawHSVValues(result);

			cv::imshow(window_name_, result);

			const int key = cv::waitKey(1);

			if (key == 27) { // ESC
				break;
			}

			if (key == 's') {
				cv::imwrite("hsv_capture.jpg", frame);
				std::cout << "Saved hsv_capture.jpg\n";
			}
		}

		camera.stopVideo();
	}

  private:
	std::string window_name_;

	int h_min_ = 0;
	int h_max_ = 179;

	int s_min_ = 0;
	int s_max_ = 255;

	int v_min_ = 0;
	int v_max_ = 255;

	void createHSVTrackbars() {
		cv::createTrackbar("Hue Min", window_name_, &h_min_, 179);

		cv::createTrackbar("Hue Max", window_name_, &h_max_, 179);

		cv::createTrackbar("Sat Min", window_name_, &s_min_, 255);

		cv::createTrackbar("Sat Max", window_name_, &s_max_, 255);

		cv::createTrackbar("Val Min", window_name_, &v_min_, 255);

		cv::createTrackbar("Val Max", window_name_, &v_max_, 255);
	}

	cv::Mat applyHSVMask(const cv::Mat &bgr_image) {
		cv::Mat hsv;
		cv::Mat mask;

		cv::cvtColor(bgr_image, hsv, cv::COLOR_BGR2HSV);

		const cv::Scalar lower(h_min_, s_min_, v_min_);

		const cv::Scalar upper(h_max_, s_max_, v_max_);

		cv::inRange(hsv, lower, upper, mask);

		cv::Mat red_overlay(
			bgr_image.size(), bgr_image.type(), cv::Scalar(0, 0, 255));

		cv::Mat highlighted =
			cv::Mat::zeros(bgr_image.size(), bgr_image.type());

		red_overlay.copyTo(highlighted, mask);

		cv::Mat result;

		cv::addWeighted(bgr_image, 0.5, highlighted, 0.5, 0.0, result);

		return result;
	}

	void drawHSVValues(cv::Mat &image) {
		const std::string text = "H: " + std::to_string(h_min_) + "-" +
			std::to_string(h_max_) + "  S: " + std::to_string(s_min_) + "-" +
			std::to_string(s_max_) + "  V: " + std::to_string(v_min_) + "-" +
			std::to_string(v_max_);

		cv::putText(image, text, cv::Point(20, 30), cv::FONT_HERSHEY_SIMPLEX,
			0.6, cv::Scalar(255, 255, 255), 2);
	}
};

int main() {
	HSVRangeHighlighter highlighter;

	highlighter.adjustHSVCamera();

	return 0;
}