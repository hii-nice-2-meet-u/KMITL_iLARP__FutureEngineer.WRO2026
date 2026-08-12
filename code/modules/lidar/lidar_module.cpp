#include "lidar_module.hpp"

namespace lidar {
LidarModule::LidarModule(std::string serial_port, std::uint32_t baud_rate)
	: serial_port_(std::move(serial_port)), baud_rate_(baud_rate) {}

LidarModule::~LidarModule() { shutdown(); }

bool LidarModule::initialize() {
	if (initialized_) {
		std::cout << "[LidarModule] Already initialized." << std::endl;
		return true;
	}

	auto channel = sl::createSerialPortChannel(serial_port_, baud_rate_);
	if (!channel) {
		std::cerr << "[LidarModule] Failed to create serial Port Channel."
				  << std::endl;
		return false;
	}

	serial_channel_ = *channel;

	auto driver = sl::createLidarDriver();
	if (!driver) {
		std::cerr << "[LidarModule] Failed to create SLAMTEC LIDAR driver."
				  << std::endl;
		return false;
	}

	lidar_driver_ = *driver;

	sl_result res = lidar_driver_->connect(serial_channel_);

	if (SL_IS_FAIL(res)) {
		std::cerr << "[LidarModule] Failed to connect to the LIDAR."
				  << std::endl;
		delete lidar_driver_;
		lidar_driver_ = nullptr;

		delete serial_channel_;
		serial_channel_ = nullptr;
		return false;
	}

	initialized_ = true;
	return true;
}

bool LidarModule::start() {

	if (!initialized_) {
		std::cerr << "[LidarModule] Failed to start. Not initialized."
				  << std::endl;
		return false;
	}

	if (running_) {
		std::cout << "[LidarModule] Already started." << std::endl;
		return true;
	}

	lidar_driver_->setMotorSpeed(DEFAULT_MOTOR_SPEED);
	sl_result result = lidar_driver_->startScan(0, 1);
	if (SL_IS_FAIL(result)) {
		std::cerr << "[LidarModule] Failed to start scan." << std::endl;
		lidar_driver_->setMotorSpeed(0);
		return false;
	}

	running_ = true;
	lidar_thread_ = std::thread(&LidarModule::scan_loop, this);
	return true;
}

void LidarModule::stop() {
	if (!running_) {
		std::cout << "[LidarModule] Not started." << std::endl;
		return;
	}

	running_ = false;

	data_updated_.notify_all();
	if (lidar_thread_.joinable()) {
		lidar_thread_.join();
	}

	if (lidar_driver_) {
		lidar_driver_->stop();
		lidar_driver_->setMotorSpeed(0);
	}
}

void LidarModule::shutdown() {
	if (running_)
		stop();

	if (lidar_driver_) {
		lidar_driver_->disconnect();
		delete lidar_driver_;
		lidar_driver_ = nullptr;
	}

	if (serial_channel_) {
		delete serial_channel_;
		serial_channel_ = nullptr;
	}

	initialized_ = false;
}

bool LidarModule::grabScan(
	sl_lidar_response_measurement_node_hq_t *nodes, size_t &count) {
	return SL_IS_OK(lidar_driver_->grabScanDataHq(nodes, count));
}

void LidarModule::checkHealth() {
	sl_lidar_response_device_health_t healthinfo;
	if (SL_IS_OK(lidar_driver_->getHealth(healthinfo))) {
		std::cerr << "[LidarModule] Device Health Status: ";
		switch (healthinfo.status) {
		case SL_LIDAR_STATUS_OK:
			std::cerr << "OK" << healthinfo.error_code << std::endl;
			break;
		case SL_LIDAR_STATUS_WARNING:
			std::cerr << "Warning" << healthinfo.error_code << std::endl;
			break;
		case SL_LIDAR_STATUS_ERROR:
			std::cerr << "Error. Error Code: " << healthinfo.error_code
					  << std::endl;
			break;
		}
	} else {
		std::cerr << "[LidarModule] [WARN] Failed to retrieve "
					 "device health."
				  << std::endl;
	}
}

void LidarModule::processScan(
	const sl_lidar_response_measurement_node_hq_t *nodes, size_t count) {

	TimedLidarData data;
	data.points.reserve(count);
	for (size_t i = 0; i < count; ++i) {
		LidarPoint point;

		point.angle_deg = nodes[i].angle_z_q14 * 90.0f / (1 << 14);

		point.distance_m = nodes[i].dist_mm_q2 / 1000.0f / (1 << 2); // meter
		// point.distance_m = nodes[i].dist_mm_q2 / (1 << 2);			 // mm

		point.quality = nodes[i].quality;

		data.points.push_back(point);
	}
	const auto now = std::chrono::steady_clock::now().time_since_epoch();

	data.timestamp_us =
		std::chrono::duration_cast<std::chrono::microseconds>(now).count();

	{
		std::lock_guard<std::mutex> lock(data_mutex_);
		data_buffer_.push(std::move(data));
		++data_sequence_;
	}

	data_updated_.notify_one();
}

void LidarModule::scan_loop() {
	std::uint8_t fail_count = 0;
	while (running_) {
		sl_lidar_response_measurement_node_hq_t nodes[8192];
		size_t count = sizeof(nodes) / sizeof(nodes[0]);

		if (!grabScan(nodes, count)) {
			++fail_count;
			if (fail_count >= 3) {
				checkHealth();
				fail_count = 0;
			}
			continue;
		}

		fail_count = 0;
		if (SL_IS_FAIL(lidar_driver_->ascendScanData(nodes, count))) {
			continue;
		}
		processScan(nodes, count);
	}
}

bool LidarModule::get_latest(TimedLidarData &data) const {
	std::lock_guard<std::mutex> lock(data_mutex_);
	return data_buffer_.latest(data);
}

bool LidarModule::wait_for_data(TimedLidarData &data) {
	std::unique_lock<std::mutex> lock(data_mutex_);

	data_updated_.wait(lock,
		[&] { return data_sequence_ != last_read_sequence_ || !running_; });

	if (!running_) {
		return false;
	}

	if (!data_buffer_.latest(data)) {
		return false;
	}

	last_read_sequence_ = data_sequence_;

	return true;
}
} // namespace lidar