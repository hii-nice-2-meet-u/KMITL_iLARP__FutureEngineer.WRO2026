#include "async_csv_writer.hpp"

#include <algorithm>
#include <chrono>
#include <filesystem>
#include <iostream>

namespace logging {

AsyncCsvWriter::AsyncCsvWriter(const std::string &path,
	const std::string &header, std::size_t maximum_queued_rows)
	: maximum_queued_rows_(std::max<std::size_t>(1, maximum_queued_rows)) {
	const std::filesystem::path file_path(path);
	const std::filesystem::path parent = file_path.parent_path();

	if (!parent.empty()) {
		std::error_code error;
		std::filesystem::create_directories(parent, error);
		if (error) {
			std::cerr << "[LOGGER] cannot create " << parent << ": "
					  << error.message() << '\n';
			return;
		}
	}

	file_.open(path, std::ios::out | std::ios::trunc);
	if (!file_.is_open()) {
		std::cerr << "[LOGGER] cannot open " << path << '\n';
		return;
	}

	if (!header.empty()) {
		file_ << header << '\n';
		file_.flush();
		if (file_.fail()) {
			write_failed_.store(true);
			file_.close();
			return;
		}
	}

	worker_ = std::thread(&AsyncCsvWriter::writer_loop, this);
}

AsyncCsvWriter::~AsyncCsvWriter() {
	if (!worker_.joinable()) {
		if (file_.is_open()) {
			file_.close();
		}
		return;
	}

	stop_.store(true);
	condition_.notify_all();
	worker_.join();
}

void AsyncCsvWriter::push(std::string row) {
	if (!worker_.joinable() || write_failed_.load()) {
		return;
	}

	{
		std::lock_guard<std::mutex> lock(mutex_);
		if (queue_.size() >= maximum_queued_rows_) {
			queue_.pop_front();
			++dropped_rows_;
		}
		queue_.push_back(std::move(row));
		++pushed_rows_;
	}

	condition_.notify_one();
}

bool AsyncCsvWriter::flush() {
	if (!worker_.joinable()) {
		return false;
	}

	std::uint64_t target_rows = 0;
	{
		std::lock_guard<std::mutex> lock(mutex_);
		target_rows = pushed_rows_ - dropped_rows_;
		flush_requested_ = true;
	}

	condition_.notify_one();

	std::unique_lock<std::mutex> lock(mutex_);
	condition_.wait(lock, [this, target_rows] {
		return flushed_rows_ >= target_rows || stop_.load();
	});

	return flushed_rows_ >= target_rows && !write_failed_.load();
}

void AsyncCsvWriter::writer_loop() {
	auto last_flush = std::chrono::steady_clock::now();

	for (;;) {
		std::deque<std::string> batch;
		bool flush_requested = false;

		{
			std::unique_lock<std::mutex> lock(mutex_);
			condition_.wait_for(lock, std::chrono::milliseconds(200), [this] {
				return !queue_.empty() || flush_requested_ || stop_.load();
			});
			batch.swap(queue_);
			flush_requested = flush_requested_;
		}

		for (const auto &row : batch) {
			file_ << row << '\n';
		}
		note_write_result();

		{
			std::lock_guard<std::mutex> lock(mutex_);
			written_rows_ += batch.size();
		}

		const auto now = std::chrono::steady_clock::now();
		const auto elapsed_ms =
			std::chrono::duration_cast<std::chrono::milliseconds>(
				now - last_flush)
				.count();

		if (flush_requested || elapsed_ms >= FLUSH_INTERVAL_MS ||
			stop_.load()) {
			file_.flush();
			note_write_result();
			last_flush = now;

			{
				std::lock_guard<std::mutex> lock(mutex_);
				flushed_rows_ = written_rows_;
				if (flush_requested) {
					flush_requested_ = false;
				}
			}

			condition_.notify_all();
		}

		if (stop_.load()) {
			std::deque<std::string> remaining;
			{
				std::lock_guard<std::mutex> lock(mutex_);
				remaining.swap(queue_);
			}

			for (const auto &row : remaining) {
				file_ << row << '\n';
			}
			note_write_result();
			file_.flush();
			note_write_result();

			{
				std::lock_guard<std::mutex> lock(mutex_);
				written_rows_ += remaining.size();
				flushed_rows_ = written_rows_;
			}

			file_.close();
			condition_.notify_all();
			break;
		}
	}
}

void AsyncCsvWriter::note_write_result() {
	if (!file_.fail()) {
		return;
	}

	write_failed_.store(true);
	if (!write_error_reported_) {
		std::cerr << "[LOGGER] write failed; remaining errors suppressed\n";
		write_error_reported_ = true;
	}
}

} // namespace logging
