#include <chrono>
#include <iomanip>
#include <iostream>
#include <opencv2/core/mat.hpp>
#include <random>
#include <sstream>
#include <string>

#include "lccv.hpp"
#include "libcamera/controls.h"
#include <opencv2/opencv.hpp>

namespace camera {
class CameraModule {
  public:
	~CameraModule();

	bool start(unsigned int video_width_ = 640,
		unsigned int video_height_ = 640, float framerate_{90});
	void stop();

	bool wait_for_frame(TimedFrameData &data);

  private:
	void capture_loop();

  private:
	lccv::PiCamera cam_;

	std::atomic<bool> running_{false};

	std::thread camera_thread_;

	mutable std::mutex frame_mutex_;
	std::condition_variable frame_updated_;

	RingBuffer<TimedFrameData, 30> frame_buffer_;

	std::uint64_t frame_sequence_{0};
	std::uint64_t last_read_sequence_{0};
}
} // namespace camera