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

#ifndef INTRINSIC_GEOMETRY_PROCESSING_PROCESSOR_H_
#define INTRINSIC_GEOMETRY_PROCESSING_PROCESSOR_H_

#include <vector>

#include "absl/status/statusor.h"
#include "intrinsic/geometry/api/geometry.h"
#include "intrinsic/longrunning/cc/operation_context.h"

namespace intrinsic::geo {
// A geometry processor is a single instance of an algorithm that can be run on
// some geometry. These can be simple or complex algorithms that take
// milliseconds or minutes. The given observer can be used to send updates
// during the processing of the input geometry.
class GeometryProcessor {
 public:
  virtual ~GeometryProcessor() = default;

  // Process a single geometry input sending updates to the given observer as
  // desired. The output will be a vector of processed geometry instances or an
  // error describing the reason for the failure.
  virtual absl::StatusOr<std::vector<Geometry>> Process(
      longrunning::OperationContext& context, const Geometry& input) const = 0;
};

}  // namespace intrinsic::geo
#endif  // INTRINSIC_GEOMETRY_PROCESSING_PROCESSOR_H_
