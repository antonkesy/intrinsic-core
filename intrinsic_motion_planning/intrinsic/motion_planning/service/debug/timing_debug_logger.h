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

#ifndef INTRINSIC_MOTION_PLANNING_SERVICE_DEBUG_TIMING_DEBUG_LOGGER_H_
#define INTRINSIC_MOTION_PLANNING_SERVICE_DEBUG_TIMING_DEBUG_LOGGER_H_

#include <string>

#include "absl/time/time.h"
#include "intrinsic/motion_planning/service/debug/time_interval.pb.h"

// We define the proto TimeInterval that represents a time breakdown of a
// specific contiguous interval.
//
// We define a simple c++ api which helps with this construction for the
// use-case of measuring timing within a function.

namespace intrinsic {

// This class's api generates intervals on the Start call, and logs the
// created object on deletion. (Note, we use a "Start" call because this works
// better with our ASSIGN_OR_RETURN/RETURN_IF_ERROR macros. This way the user
// can name what is currently happening and we will see the interval terminate
// if there is an error).
//
// The caller can also attach arbitrary debug strings to the interval for later
// debugging. Note that these calls should be done after the Start of the
// interval.
//
// void MyFunction() {
//   TimingDebugLogger logger("MyFunction");
//
//   logger.Start("CoolThingOne");
//   CoolThingOne();
//
//   logger.Start("CoolThingTwo");
//   auto result = CoolThingTwo();
//   logger.AddData(result.error_str);
//
//   // Last interval automatically closes upon deletion of object.
//   // The whole TimeInterval is logged after.
// }
class TimingDebugLogger {
 public:
  explicit TimingDebugLogger(std::string name,
                             std::string starting_interval_name = "<init>");
  // Ends the running interval and logs the final result.
  ~TimingDebugLogger();

  // Starts a new interval (implicitly ending the previous one). The last
  // interval is automically stopped upon destruction of the object.
  void Start(std::string interval_name);

  // Attach debug data to the current interval. The key/value become entries in
  // the `data` field of the corresponding interval.
  //
  // We can consider allowing users arbitrary access to the json, but keeping
  // this simple for now.
  void Attach(std::string key, std::string value);

  // Same as above, but attaches the data to the top-level interval. This is
  // useful when you want to associate data with all intervals.
  void AttachTop(std::string key, std::string value);

 private:
  // The top level object to logged.
  intrinsic_proto::motion_planning::TimeIntervalLog log_;
  // The incremental object representing the current interval.
  intrinsic_proto::motion_planning::TimeInterval interval_;
  // Time the last interval started.
  absl::Time last_interval_start_;

  // TODO(b/369423744): Add tests for this class.
};

}  // namespace intrinsic

#endif  // INTRINSIC_MOTION_PLANNING_SERVICE_DEBUG_TIMING_DEBUG_LOGGER_H_
