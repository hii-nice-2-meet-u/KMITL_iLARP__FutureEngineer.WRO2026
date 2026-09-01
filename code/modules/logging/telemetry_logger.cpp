#include "telemetry_logger.hpp"

#include <csignal>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <unistd.h>

namespace logging {

namespace {

volatile std::sig_atomic_t stop_signal_received = 0;

void handle_stop_signal(int) { stop_signal_received = 1; }

double elapsed_seconds(
	std::uint64_t timestamp_us, std::uint64_t run_start_timestamp_us) {
	if (timestamp_us <= run_start_timestamp_us) {
		return 0.0;
	}
	return static_cast<double>(timestamp_us - run_start_timestamp_us) * 1e-6;
}

} // namespace

void install_stop_signal_handlers() {
	std::signal(SIGINT, handle_stop_signal);
	std::signal(SIGTERM, handle_stop_signal);
}

void notify_stop_requested() { stop_signal_received = 1; }

bool stop_requested() { return stop_signal_received != 0; }

std::string make_run_directory(const std::string &logs_root) {
	const std::time_t current_time = std::time(nullptr);
	std::tm local_time{};
	localtime_r(&current_time, &local_time);

	std::ostringstream stream;
	stream << logs_root << "/run_" << local_time.tm_year + 1900 << '-'
		   << std::setfill('0') << std::setw(2) << local_time.tm_mon + 1 << '-'
		   << std::setw(2) << local_time.tm_mday << '_' << std::setw(2)
		   << local_time.tm_hour << std::setw(2) << local_time.tm_min
		   << std::setw(2) << local_time.tm_sec << "_pid"
		   << static_cast<long>(getpid());
	return stream.str();
}

TelemetryLogger::TelemetryLogger(const std::string &run_directory)
	: writer_(std::make_unique<AsyncCsvWriter>(
		  run_directory + "/telemetry.csv", telemetry_csv_header())) {}

void TelemetryLogger::record(TelemetryRow row) {
	if (writer_) {
		row.row_index = next_row_index_++;
		writer_->push(to_csv_row(row));
	}
}

bool TelemetryLogger::flush() { return writer_ && writer_->flush(); }

bool TelemetryLogger::has_write_error() const {
	return writer_ && writer_->has_write_error();
}

std::size_t TelemetryLogger::dropped_row_count() const {
	return writer_ ? writer_->dropped_row_count() : 0;
}

EventLogger::EventLogger(
	const std::string &run_directory, std::uint64_t run_start_timestamp_us)
	: writer_(std::make_unique<AsyncCsvWriter>(
		  run_directory + "/events.log", "", 64)),
	  run_start_timestamp_us_(run_start_timestamp_us) {}

void EventLogger::event(std::uint64_t timestamp_us, int lap,
	std::size_t corner_index, const std::string &message) {
	if (!writer_) {
		return;
	}

	std::ostringstream stream;
	stream.setf(std::ios::fixed);
	stream.precision(3);
	stream << "[t=" << elapsed_seconds(timestamp_us, run_start_timestamp_us_)
		   << "s][lap=" << lap << "][corner=" << corner_index << "] "
		   << message;
	writer_->push(stream.str());
}

void EventLogger::fault(
	std::uint64_t timestamp_us, const std::string &message) {
	if (!writer_) {
		return;
	}

	std::ostringstream stream;
	stream.setf(std::ios::fixed);
	stream.precision(3);
	stream << "[t=" << elapsed_seconds(timestamp_us, run_start_timestamp_us_)
		   << "s] FAULT " << message;
	writer_->push(stream.str());
}

bool EventLogger::flush() { return writer_ && writer_->flush(); }

bool EventLogger::has_write_error() const {
	return writer_ && writer_->has_write_error();
}

std::size_t EventLogger::dropped_row_count() const {
	return writer_ ? writer_->dropped_row_count() : 0;
}

} // namespace logging
