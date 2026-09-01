#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>

#include "async_csv_writer.hpp"
#include "lidar_processor.hpp"
#include "log_types.hpp"
#include "navigation_state.hpp"
#include "track_map.hpp"

namespace logging {

class WallLogger {
  public:
	explicit WallLogger(const std::string &run_directory);

	bool is_open() const { return writer_ && writer_->is_open(); }
	void record(const lidar::ResolvedWalls &walls,
		navigation::NavigationMode mode, const navigation::MapPose &pose,
		std::uint64_t timestamp_us);
	bool flush();
	bool has_write_error() const;
	std::size_t dropped_row_count() const;

  private:
	void queue_wall(const std::optional<lidar::LineSegment> &segment,
		const char *role, const char *mode_name,
		const navigation::MapPose &pose, std::uint64_t timestamp_us);

	std::unique_ptr<AsyncCsvWriter> writer_;
};

bool dump_corners(
	const std::string &run_directory, const navigation::TrackMap &map);

} // namespace logging
