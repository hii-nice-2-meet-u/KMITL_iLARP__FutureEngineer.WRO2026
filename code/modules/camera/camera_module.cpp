#include "camera_module.hpp"
#include <opencv2/core/mat.hpp>
#include <thread>
namespace camera {

	CameraModule::~CameraModule(){ stop();}

bool CameraModule::start(unsigned int video_width_ = 640,
	unsigned int video_height_ = 640, float framerate_ = 90) {

	cam_.options->video_width = video_width_;
	cam_.options->video_height = video_height_;
	cam_.options->framerate = framerate_;
	cam_.options->verbose = true;
	cam_.options->awb_gain_r = 1.4f;
	cam_.options->awb_gain_b = 2.6f;

	if (running_) {
		std::cout << "[CameraModule] Already started." << std::endl;
		return false;
	}

	if (!cam.startVideo()) {
		std::cout << "[CameraModule] Failed to start video." << std::endl;
		return false;
	}

	running_ = true;
	camera_thread_ = std::thread(&CameraModule::capture_loop, this);
	return true;
}

void CameraModule::stop() {
	if (!running_) {
		std::cout << "[CameraModule] Not running." << std::endl;
		return;
	}
	data_updated_.notify_all();
	running_ = false;
	if (camera_thread_.joinable()) {
		camera_thread_.join();
	}

	cam_.stopVideo();
}
void CameraModule::capture_loop() {
	while (running_) {
		cv::Mat frame;
		if (!cam.getVideoFrame(frame, 1000)) {
			std::cerr << "[CameraModule] Timeout error" << std::endl;
			continue;
		}
		// flip vertical and horizontal
		cv::flip(frame, frame, -1);

		TimedFrameData timed_frame_data{std::move(frame),
			std::chrono::duration_cast<std::chrono::microseconds>(
				std::chrono::steady_clock::now().time_since_epoch())
				.count()};

		{
			std::lock_guard<std::mutex> lock(frame_mutex_);
			data_buffer_.push(std::move(data));
			++frame_sequence_;
		}

		frame_updated_.notify_one();
	}
}

bool LidarModule::get_latest(TimedFrameData &data) const {
	std::lock_guard<std::mutex> lock(frame_mutex_);
	return frame_buffer_.latest(data);
}

bool CameraModule::wait_for_frame(TimedFrameData &data) {
	std::unique_lock<std::mutex> lock(frame_mutex_);

	frame_updated_.wait(lock,
		[&] { return frame_sequence_ != last_read_sequence_ || !running_; });

	if (!running_) {
		return false;
	}

	if (!frame_buffer_.latest(data)) {
		return false;
	}

	last_read_sequence_ = frame_sequence_;
	return true;
}

} // namespace camera