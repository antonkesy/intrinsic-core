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

#ifndef INTRINSIC_ICON_SERVER_TIME_SLICE_H_
#define INTRINSIC_ICON_SERVER_TIME_SLICE_H_

#include "intrinsic/icon/utils/clock.h"
#include "intrinsic/icon/utils/duration.h"

namespace intrinsic::icon {

// Describes the time slice allocated to a TimeSlicer operation.
// Long ago, the predecessor of MainLoop was called TimeSlicer.
class TimeSlice {
 public:
  explicit TimeSlice(Time start, Duration length)
      : start_(start), length_(length) {}
  explicit TimeSlice(Time start, Time end)
      : start_(start), length_(end - start) {}

  // Returns the start time of the operation.
  inline Time Start() const { return start_; }

  // Returns the amount of time allocated to the operation.
  inline Duration GetDuration() const { return length_; }

 private:
  const Time start_;
  const Duration length_;
};

}  // namespace intrinsic::icon

#endif  // INTRINSIC_ICON_SERVER_TIME_SLICE_H_
