#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>

#include "async_csv_writer.hpp"
#include "log_types.hpp"

namespace logging {

void install_stop_signal_handlers();
void notify_stop_requested();
bool stop_requested();

std::string make_run_directory(const std::string &logs_root = "logs");

class TelemetryLogger {
  public:
	explicit TelemetryLogger(const std::string &run_directory);

	bool is_open() const { return writer_ && writer_->is_open(); }

	// Stamps row.row_index with a monotonic per-run counter before writing.
	// The writer drops the oldest queued row when it is saturated and leaves
	// no marker in the file, so a gap in row_index is the only way to tell a
	// dropped row from a merely slow loop iteration.
	void record(TelemetryRow row);
	bool flush();
	bool has_write_error() const;
	std::size_t dropped_row_count() const;

  private:
	std::unique_ptr<AsyncCsvWriter> writer_;
	std::uint64_t next_row_index_{0};
};

class EventLogger {
  public:
	EventLogger(
		const std::string &run_directory, std::uint64_t run_start_timestamp_us);

	bool is_open() const { return writer_ && writer_->is_open(); }
	void event(std::uint64_t timestamp_us, int lap, std::size_t corner_index,
		const std::string &message);
	void fault(std::uint64_t timestamp_us, const std::string &message);
	bool flush();
	bool has_write_error() const;
	std::size_t dropped_row_count() const;

  private:
	std::unique_ptr<AsyncCsvWriter> writer_;
	std::uint64_t run_start_timestamp_us_{0};
};

} // namespace logging
