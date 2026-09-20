// Copyright 2026 Intrinsic Innovation LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     https://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef INTRINSIC_ICON_HARDWARE_MODULES_UNIVERSAL_ROBOTS_LOG_HANDLER_H_
#define INTRINSIC_ICON_HARDWARE_MODULES_UNIVERSAL_ROBOTS_LOG_HANDLER_H_

#include "absl/log/check.h"
#include "absl/log/log.h"
#include "include/ur_client_library/log.h"
#include "intrinsic/icon/testing/realtime_annotations.h"
#include "intrinsic/icon/utils/log.h"

namespace intrinsic::icon {

// Custom realtime safe log handler for the ur_driver.
// Usage:
//   urcl::setLogLevel(urcl::LogLevel::DEBUG);
//   auto log_handler = std::make_unique<RealtimeLogHandler>;
//   urcl::registerLogHandler(std::move(log_handler));
class RealtimeLogHandler : public urcl::LogHandler {
 public:
  RealtimeLogHandler() = default;
  void log(const char* file, int line, urcl::LogLevel loglevel,
           const char* log) INTRINSIC_CHECK_REALTIME_SAFE override {
    switch (loglevel) {
      case urcl::LogLevel::INFO:
        // intrinsic::SourceLocation doesn't offer a constructor with file and
        // line.
        INTRINSIC_RT_LOG_THROTTLED(INFO) << file << ":" << line << "] " << log;
        break;
      case urcl::LogLevel::DEBUG:
        INTRINSIC_RT_LOG_THROTTLED(INFO) << file << ":" << line << "] " << log;
        break;
      case urcl::LogLevel::WARN:
        INTRINSIC_RT_LOG_THROTTLED(WARNING)
            << file << ":" << line << "] " << log;
        break;
      case urcl::LogLevel::ERROR:
        INTRINSIC_RT_LOG(ERROR) << file << ":" << line << "] " << log;
        break;
      case urcl::LogLevel::FATAL:
        INTRINSIC_RT_LOG(ERROR) << file << ":" << line << "] " << log;
        // The string logged by QCHECK can be longer than INTRINSIC_RT_LOG.
        QCHECK(false) << file << ":" << line << "] " << log;
        break;
      case urcl::LogLevel::NONE:
        INTRINSIC_RT_LOG_THROTTLED(ERROR) << file << ":" << line << "] " << log;
        break;
    }
  }
};

}  // namespace intrinsic::icon

#endif  // INTRINSIC_ICON_HARDWARE_MODULES_UNIVERSAL_ROBOTS_LOG_HANDLER_H_
