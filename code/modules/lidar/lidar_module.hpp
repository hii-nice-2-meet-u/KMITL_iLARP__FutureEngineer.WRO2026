#pragma once

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>

#include "sl_lidar_driver.h"

namespace lidar {

class LidarModule {
  public:
	LidarModule(std::string serial_port = "/dev/ttyUSB0",
		std::uint32_t baud_rate = 1000000);

	~LidarModule();

	LidarModule(const LidarModule &) = delete;
	LidarModule &operator=(const LidarModule &) = delete;

	bool initialize();
	void shutdown();

	bool start();
	void stop();

	// bool get_latest(TimedLidarData& data) const;
	// bool wait_for_data(TimedLidarData& data);

	// bool is_initialized() const noexcept;
	// bool is_running() const noexcept;
  private:
	void scan_loop();

	bool grabScan(
		sl_lidar_response_measurement_node_hq_t *nodes, size_t &count);

	void checkHealth();

	void processScan(
		const sl_lidar_response_measurement_node_hq_t *nodes, size_t count);

  private:
	sl::ILidarDriver *lidar_driver_ = nullptr;
	sl::IChannel *serial_channel_ = nullptr;

	std::string serial_port_;
	std::uint32_t baud_rate_{1000000};

	bool initialized_{false};
	std::atomic<bool> running_{false};

	std::thread lidar_thread_;

	mutable std::mutex data_mutex_;
	std::condition_variable data_updated_;

	// RingBuffer<TimedLidarData> data_buffer_{10};
};

} // namespace lidar