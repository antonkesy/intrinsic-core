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

#ifndef INTRINSIC_GEOMETRY_SERVICE_UTIL_GEOMETRY_DATA_H_
#define INTRINSIC_GEOMETRY_SERVICE_UTIL_GEOMETRY_DATA_H_

#include "absl/status/statusor.h"
#include "intrinsic/geometry/api/geometry.h"
#include "intrinsic/geometry/proto/geometry_service_types.pb.h"

namespace intrinsic::geo {
// Returns a Geometry instance that represents the input proto
absl::StatusOr<Geometry> ToGeometry(
    const intrinsic_proto::geometry::GeometryData& data);

}  // namespace intrinsic::geo

namespace intrinsic {
using ::intrinsic::geo::ToGeometry;
}  // namespace intrinsic

#endif  // INTRINSIC_GEOMETRY_SERVICE_UTIL_GEOMETRY_DATA_H_
