#include "third_party/abb_hardware_module/abb_hwm/abb_elog.h"

#include <sstream>
#include <string>
#include <vector>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "intrinsic/util/status/status_macros.h"
#include "nlohmann/json_fwd.hpp"

namespace {

absl::StatusOr<abb_hardware_module::AbbEventLogEntry::Level> ParseMessageType(
    const std::string& msgtype) {
  if (msgtype == "1") {
    return abb_hardware_module::AbbEventLogEntry::Level::INFO;
  }
  if (msgtype == "2") {
    return abb_hardware_module::AbbEventLogEntry::Level::WARNING;
  }
  if (msgtype == "3") {
    return abb_hardware_module::AbbEventLogEntry::Level::ERROR;
  }
  return absl::InvalidArgumentError("Invalid message type: " + msgtype);
}

absl::StatusOr<abb_hardware_module::AbbEventLogEntry> ParseEventLogEntry(
    const nlohmann::json& entry) {
  abb_hardware_module::AbbEventLogEntry log_entry;
  if (!entry.contains("title") || !entry.contains("desc") ||
      !entry.contains("causes") || !entry.contains("conseqs") ||
      !entry.contains("code") || !entry.contains("tstamp") ||
      !entry.contains("msgtype") || !entry.contains("actions")) {
    return absl::InvalidArgumentError(
        "Event log entry is missing required fields. ");
  }
  log_entry.title = entry["title"];
  log_entry.desc = entry["desc"];
  log_entry.causes = entry["causes"];
  log_entry.conseqs = entry["conseqs"];
  log_entry.actions = entry["actions"];
  log_entry.code = std::stoi(entry["code"].get<std::string>());
  log_entry.tstamp = entry["tstamp"];
  INTR_ASSIGN_OR_RETURN(log_entry.msgtype, ParseMessageType(entry["msgtype"]));
  return log_entry;
}

}  // namespace

namespace abb_hardware_module {

std::string AbbEventLogEntry::ToString() const {
  std::ostringstream oss;
  oss << "Title: " << title << "\n"
      << "Timestamp: " << tstamp << "\n"
      << "Description: " << desc << "\n"
      << "Causes: " << causes << "\n"
      << "Consequences: " << conseqs << "\n"
      << "Actions: " << actions << "\n";
  return oss.str();
}

absl::StatusOr<std::vector<AbbEventLogEntry>> ParseAbbEventLog(
    const json& event_list, AbbEventLogEntry::Level level) {
  std::vector<AbbEventLogEntry> entries;
  for (const auto& json_event : event_list) {
    INTR_ASSIGN_OR_RETURN(auto entry, ParseEventLogEntry(json_event));
    if (entry.msgtype >= level) {
      entries.push_back(entry);
    }
  }
  return entries;
}

}  // namespace abb_hardware_module
