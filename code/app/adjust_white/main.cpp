#include "libcamera_app_options.hpp"
#include <iostream>
#include <lccv.hpp>
#include <opencv2/opencv.hpp>

int main() {
	const std::string win_name = "AWB Gain Tuner";
	cv::namedWindow(win_name, cv::WINDOW_NORMAL);
	cv::resizeWindow(win_name, 640, 640);

	int r_slider = 10; // 0.0 - 5.0 (0 - 50)
	int b_slider = 10; // 0.0 - 5.0 (0 - 50

	cv::createTrackbar("Red Gain x10", win_name, &r_slider, 50);
	cv::createTrackbar("Blue Gain x10", win_name, &b_slider, 50);

	lccv::PiCamera camera;
	camera.options->video_width = 640;
	camera.options->video_height = 640;
	camera.options->framerate = 60;

	camera.options->awb_gain_r = r_slider / 10.0f;
	camera.options->awb_gain_b = b_slider / 10.0f;

	camera.startVideo();

	cv::Mat frame;
	int prev_r = r_slider, prev_b = b_slider;

	while (true) {
		if (!camera.getVideoFrame(frame, 1000) || frame.empty())
			continue;

		cv::flip(frame, frame, 0);
		cv::imshow(win_name, frame);

		if (r_slider != prev_r || b_slider != prev_b) {

			float r_gain = r_slider / 10.0f;
			float b_gain = b_slider / 10.0f;

			camera.options->awb_gain_r = r_gain;
			camera.options->awb_gain_b = b_gain;

			camera.getControlList().set(libcamera::controls::ColourGains,
				libcamera::Span<const float, 2>({r_gain, b_gain}));
			prev_r = r_slider;
			prev_b = b_slider;
		}

		int key = cv::waitKey(1);
		if (key == 27)
			break;
	}
	camera.stopVideo();
	cv::destroyAllWindows();
	return 0;
}
