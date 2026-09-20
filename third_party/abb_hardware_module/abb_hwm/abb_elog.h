#ifndef ABB_HARDWARE_MODULE_ABB_HWM_ABB_ELOG_H_
#define ABB_HARDWARE_MODULE_ABB_HWM_ABB_ELOG_H_

#include <sstream>  // For std::ostringstream
#include <string>
#include <vector>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "intrinsic/icon/testing/realtime_annotations.h"
#include "nlohmann/json.hpp"

namespace abb_hardware_module {
using json = nlohmann::json;

struct AbbEventLogEntry {
  enum class Level {
    INFO = 1,
    WARNING = 2,
    ERROR = 3,
  };

  std::string title;
  std::string desc;
  std::string causes;
  std::string conseqs;
  std::string actions;
  int code;
  std::string code_description;
  std::string tstamp;
  Level msgtype;

  // Add a ToString method for string representation
  std::string ToString() const INTRINSIC_NON_REALTIME_ONLY;
};

absl::StatusOr<std::vector<AbbEventLogEntry>> ParseAbbEventLog(
    const json& event_list,
    AbbEventLogEntry::Level level = AbbEventLogEntry::Level::ERROR)
    INTRINSIC_NON_REALTIME_ONLY;

}  // namespace abb_hardware_module

#endif  // ABB_HARDWARE_MODULE_ABB_HWM_ABB_ELOG_H_
