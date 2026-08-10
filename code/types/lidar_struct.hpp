#pragma once

#include <cstdint>
#include <vector>

struct LidarPoint
{
    float angle_deg{0.0f};
    float distance_m{0.0f};
    std::uint8_t quality{0};
};

struct TimedLidarData
{
    std::vector<LidarPoint> points;
    std::uint64_t timestamp_us{0};
};