#pragma once

#include <atomic>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <fstream>
#include <mutex>
#include <string>
#include <thread>

namespace logging {

class AsyncCsvWriter {
  public:
	AsyncCsvWriter(const std::string &path, const std::string &header,
		std::size_t maximum_queued_rows = 400);
	~AsyncCsvWriter();

	AsyncCsvWriter(const AsyncCsvWriter &) = delete;
	AsyncCsvWriter &operator=(const AsyncCsvWriter &) = delete;

	bool is_open() const { return file_.is_open(); }
	void push(std::string row);
	bool flush();

	bool has_write_error() const { return write_failed_.load(); }

	std::size_t dropped_row_count() const {
		std::lock_guard<std::mutex> lock(mutex_);
		return dropped_rows_;
	}

  private:
	void writer_loop();
	void note_write_result();

	std::ofstream file_;
	std::thread worker_;
	mutable std::mutex mutex_;
	std::condition_variable condition_;
	std::deque<std::string> queue_;
	std::size_t maximum_queued_rows_;
	std::atomic_bool stop_{false};
	std::atomic_bool write_failed_{false};
	bool flush_requested_{false};
	bool write_error_reported_{false};
	std::uint64_t pushed_rows_{0};
	std::uint64_t written_rows_{0};
	std::uint64_t flushed_rows_{0};
	std::size_t dropped_rows_{0};

	// Flush once per second to limit process-crash data loss without placing
	// storage I/O on the control thread.
	static constexpr std::int64_t FLUSH_INTERVAL_MS = 1000;
};

} // namespace logging
