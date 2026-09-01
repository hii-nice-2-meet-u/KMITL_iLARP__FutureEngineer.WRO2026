#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "navigation_controller.hpp"

namespace logging {

// Minimal JSON object builder for small, static run metadata. Values are
// serialized when inserted, so the writer owns no references to live config.
class JsonObject {
  public:
	JsonObject &add_string(const std::string &key, const std::string &value);
	JsonObject &add_bool(const std::string &key, bool value);
	JsonObject &add_number(const std::string &key, float value);
	JsonObject &add_number(const std::string &key, double value);
	JsonObject &add_integer(const std::string &key, std::int64_t value);
	JsonObject &add_unsigned(const std::string &key, std::uint64_t value);
	JsonObject &add_null(const std::string &key);
	JsonObject &add_object(const std::string &key, const JsonObject &value);

	std::string to_json() const;

  private:
	std::vector<std::pair<std::string, std::string>> entries_;
};

JsonObject pid_config_json(const control::PIDConfig &config);
JsonObject stanley_config_json(const control::StanleyConfig &config);
JsonObject navigation_config_json(const navigation::NavigationConfig &config);

JsonObject make_run_metadata(const std::string &executable_path,
	const navigation::NavigationConfig &navigation_config,
	const std::optional<float> &otos_linear_scalar,
	const std::optional<float> &otos_angular_scalar);

// Writes run_meta.json via a temporary file and atomic rename. The same
// function is used again at shutdown when drop counters are available.
bool write_run_metadata(
	const std::string &run_directory, const JsonObject &metadata);

} // namespace logging
