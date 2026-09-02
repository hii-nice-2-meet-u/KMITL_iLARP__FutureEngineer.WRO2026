#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>

#include "async_csv_writer.hpp"
#include "log_types.hpp"
#include "navigation_state.hpp"
#include "track_map.hpp"

namespace logging {

// Writes corner_plan.csv: the shadow corner planner's per-tick output next to
// the actual command, for the corner-strategy redesign. One more log set
// alongside telemetry/walls/segments/corners in the same run directory.
//
// To keep the file about corners rather than full of straights, a row is written
// only in the corner window -- ticks where the wall-corner landmark is confirmed
// or the vehicle is TURNING. Uses the same async drop-oldest writer as the other
// sets, so a saturated loop drops a row (visible as a row_index gap) rather than
// blocking the control loop.
class CornerPlanLogger {
  public:
	explicit CornerPlanLogger(const std::string &run_directory);

	bool is_open() const { return writer_ && writer_->is_open(); }

	void record(const navigation::NavigationResult &result,
		const navigation::NavigationState &state,
		const navigation::MapPose &pose, std::uint64_t timestamp_us);

	bool flush();
	bool has_write_error() const;
	std::size_t dropped_row_count() const;

  private:
	std::unique_ptr<AsyncCsvWriter> writer_;
	std::uint64_t next_row_index_{0};
};

} // namespace logging
