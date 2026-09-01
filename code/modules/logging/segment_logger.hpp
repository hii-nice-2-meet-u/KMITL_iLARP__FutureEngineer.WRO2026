#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>

#include "async_csv_writer.hpp"
#include "lidar_processor.hpp"
#include "navigation_state.hpp"

namespace logging {

class SegmentLogger {
  public:
	explicit SegmentLogger(const std::string &run_directory);

	bool is_open() const { return writer_ && writer_->is_open(); }
	void record(const lidar::ProcessedLidarData &lidar_data,
		navigation::NavigationMode mode);
	bool flush();
	bool has_write_error() const;
	std::size_t dropped_row_count() const;

  private:
	static const char *role_for(const lidar::LineSegment &segment,
		const lidar::ResolvedWalls &walls);
	static bool same_segment(const lidar::LineSegment &first,
		const std::optional<lidar::LineSegment> &second);

	std::unique_ptr<AsyncCsvWriter> writer_;
	std::uint64_t next_row_index_{0};
};

} // namespace logging
