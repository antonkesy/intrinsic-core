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

#ifndef INTRINSIC_ICON_CONTROL_PARTS_REALTIME_PART_PROPERTY_ACCESS_H_
#define INTRINSIC_ICON_CONTROL_PARTS_REALTIME_PART_PROPERTY_ACCESS_H_

#include "absl/base/attributes.h"
#include "absl/container/fixed_array.h"
#include "intrinsic/icon/common/part_properties.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/icon/utils/realtime_status_or.h"

namespace intrinsic::icon {

// Allows a realtime thread to access the properties of a single part by
// PartPropertyId.
//
// This class is short-lived: It springs into life at the beginning of each
// cycle, taking references to the two FixedArrays, and is discarded at the end
// of the cycle. This ensures that all parts receive and submit property changes
// atomically.
//
// In practice, the FixedArray references come from two AsyncBuffers that
// mediate communication between the realtime and non-realtime thread. The
// surrounding code is responsible for getting and committing the corresponding
// pointers from and into those AsyncBuffers.
//
// We expect the indices of the two FixedArrays to correspond directly to
// PartPropertyIds.
class RealtimePartPropertyAccess {
 public:
  explicit RealtimePartPropertyAccess(
      const absl::FixedArray<PartPropertyValue>& non_rt_to_rt_values
          ABSL_ATTRIBUTE_LIFETIME_BOUND,
      absl::FixedArray<PartPropertyValue>& rt_to_non_rt_values
          ABSL_ATTRIBUTE_LIFETIME_BOUND)
      : non_rt_to_rt_values_(non_rt_to_rt_values),
        rt_to_non_rt_values_(rt_to_non_rt_values) {}

  // Returns the current value of the boolean property with the PartPropertyId
  // `id` as last reported by the non-realtime thread. Note that this *does not*
  // get updated when you call SetBoolProperty() below!
  //
  // Returns an error if
  // * `id` is not a valid PartPropertyId
  // * the property exists, but is not a boolean
  RealtimeStatusOr<bool> GetBoolProperty(PartPropertyId id) const;

  // Returns the current value of the double property with the PartPropertyId
  // `id` as last reported by the non-realtime thread. Note that this *does not*
  // get updated when you call SetDoubleProperty() below!
  //
  // Returns an error if
  // * `id` is not a valid PartPropertyId
  // * the property exists, but is not a double
  RealtimeStatusOr<double> GetDoubleProperty(PartPropertyId id) const;

  // Sets the value of the bool property with the PartPropertyId `id`. Note that
  // this change does not become visible to the non-realtime thread until the
  // underlying storage is committed back to its AsyncBuffer.
  //
  // Returns an error if
  // * `id` is not a valid PartPropertyId
  // * the property exists, but is not a bool
  RealtimeStatus SetBoolProperty(PartPropertyId id, bool value);

  // Sets the value of the double property with the PartPropertyId `id`. Note
  // that this change does not become visible to the non-realtime thread until
  // the underlying storage is committed back to its AsyncBuffer.
  //
  // Returns an error if
  // * `id` is not a valid PartPropertyId
  // * the property exists, but is not a double
  RealtimeStatus SetDoubleProperty(PartPropertyId id, double value);

 private:
  const absl::FixedArray<PartPropertyValue>& non_rt_to_rt_values_;
  absl::FixedArray<PartPropertyValue>& rt_to_non_rt_values_;
};

}  // namespace intrinsic::icon

#endif  // INTRINSIC_ICON_CONTROL_PARTS_REALTIME_PART_PROPERTY_ACCESS_H_
