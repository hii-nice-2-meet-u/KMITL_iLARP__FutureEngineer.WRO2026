#pragma once

#include <vector>

struct TimedFrameData {
	cv::Mat frames;
	std::uint64_t timestamp_us{0};
};