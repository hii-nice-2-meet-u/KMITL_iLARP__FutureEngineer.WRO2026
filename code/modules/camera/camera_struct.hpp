#pragma once

#include <vector>
#include <opencv2/core.hpp>
#include <cstdint>

struct TimedFrameData {
	cv::Mat frame;
	std::uint64_t timestamp_us{0};
};