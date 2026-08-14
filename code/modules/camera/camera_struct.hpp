#pragma once

#include <vector>

struct TimedFrameData {
	cv::Mat frame;
	std::uint64_t timestamp_us{0};
};