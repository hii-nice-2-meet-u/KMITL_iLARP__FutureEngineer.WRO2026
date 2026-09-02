#pragma once

#include <cstdint>
#include <vector>

struct LidarPoint
{
    float angle_deg{0.0f};
    float distance_m{0.0f};
    std::uint8_t quality{0};

    // Fraction of the revolution already swept when this sample was captured:
    // 0.0 at the first acquired sample of the scan, approaching 1.0 at the
    // last. Reconstructed from the angle relative to the scan's start angle
    // under a constant-angular-velocity assumption, because ascendScanData()
    // sorts the buffer by angle and destroys the acquisition order. It is the
    // sole input to motion deskew; nothing reads it yet (deskew is gated on
    // the LiDAR-zero hardware check, HARDWARE_CHECKS.md Check 1 / M-1).
    float scan_phase{0.0f};
};

struct TimedLidarData
{
    std::vector<LidarPoint> points;
    std::uint64_t timestamp_us{0};

    // Measured wall-clock duration of this revolution [us], from the previous
    // scan's timestamp. 0 on the first scan. Deskew needs the real period, not
    // the nominal 50 ms, because motor RPM drifts under load.
    std::uint64_t scan_period_us{0};
};