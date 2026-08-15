#include <chrono>
#include <iomanip>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <random>
#include <sstream>
#include <string>
#include <thread>

#include "RingBuffer.hpp"
#include "camera_struct.hpp"
#include "lccv.hpp"

namespace camera {
class CameraModule {
  public:
	CameraModule(unsigned int video_width = 640,
		unsigned int video_height = 640, float framerate = 90,
		float awb_gain_r = 1.4f, float awb_gain_b = 2.6f);
	~CameraModule();

	bool start();
	void stop();

	bool get_latest(TimedFrameData &data) const;
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
};
} // namespace camera