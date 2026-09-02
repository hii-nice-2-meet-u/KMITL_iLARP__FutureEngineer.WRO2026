#include "corner_plan_logger.hpp"

namespace logging {

CornerPlanLogger::CornerPlanLogger(const std::string &run_directory)
	: writer_(std::make_unique<AsyncCsvWriter>(
		  run_directory + "/corner_plan.csv", corner_plan_csv_header())) {}

void CornerPlanLogger::record(const navigation::NavigationResult &result,
	const navigation::NavigationState &state, const navigation::MapPose &pose,
	std::uint64_t timestamp_us) {
	if (!writer_) {
		return;
	}

	// Only the corner window is interesting; skip long straights so the file
	// stays small and about corners.
	const bool in_corner_window = result.debug.wall_corner_confirmed ||
		state.mode == navigation::NavigationMode::TURNING;
	if (!in_corner_window) {
		return;
	}

	CornerPlanRow row =
		make_corner_plan_row(timestamp_us, result, state, pose);
	row.row_index = next_row_index_++;
	writer_->push(to_csv_row(row));
}

bool CornerPlanLogger::flush() { return writer_ && writer_->flush(); }

bool CornerPlanLogger::has_write_error() const {
	return writer_ && writer_->has_write_error();
}

std::size_t CornerPlanLogger::dropped_row_count() const {
	return writer_ ? writer_->dropped_row_count() : 0;
}

} // namespace logging
