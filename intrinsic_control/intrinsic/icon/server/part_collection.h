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

#ifndef INTRINSIC_ICON_SERVER_PART_COLLECTION_H_
#define INTRINSIC_ICON_SERVER_PART_COLLECTION_H_

#include <memory>
#include <string>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "absl/container/flat_hash_set.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/time/time.h"
#include "absl/types/span.h"
#include "intrinsic/icon/common/part_properties.h"
#include "intrinsic/icon/proto/part_status.pb.h"
#include "intrinsic/icon/proto/safety_status.pb.h"
#include "intrinsic/icon/proto/v1/types.pb.h"
#include "intrinsic/icon/server/parts_manager.h"

namespace intrinsic::icon {

// This bundles access to all Parts on a server, as well as a corresponding
// PartsManager for resource management.
//
// A PartCollection is thread-safe as long as its StatusGetter is.
class PartCollection {
 public:
  // Interface for retrieving dynamic part data.
  // Please ensure thread-safety when implementing a new StatusGetter.
  class StatusGetter {
   public:
    virtual ~StatusGetter() = default;
    // Block at most until `deadline` to retrieve the latest status for all
    // Parts. This is a separate interface, rather than each part having a
    // GetStatus() method, to
    //
    // a) Ensure that all status values are sampled at the same time.
    // b) Conserve bandwidth in communicating with the real-time system.
    //
    // Returns a map from Part name to PartStatus proto.
    virtual absl::StatusOr<
        absl::flat_hash_map<std::string, intrinsic_proto::icon::PartStatus>>
    GetPartStatuses(absl::Time deadline) = 0;

    // Block at most until `deadline` to retrieve the latest SafetyStatus of the
    // robot.
    virtual absl::StatusOr<intrinsic_proto::icon::SafetyStatus> GetSafetyStatus(
        absl::Time deadline) = 0;

    // Retrieves the latest part property values, including wall and control
    // timestamps.
    //
    // Thread-safe
    virtual absl::StatusOr<TimestampedPartProperties> GetPartProperties() = 0;

    // Applies the given part property values atomically.
    //
    // Thread-safe, but there is no guarantee that the realtime thread reads a
    // property value before another call overwrites it.
    // For example, consider the following calls:
    //
    // SetPartProperties({.properties={{"my_part", {"my_prop", 1}}}});
    // SetPartProperties({.properties={{"my_part", {"my_prop", 2}}}});
    //
    // The realtime thread may or may not tick between these calls. If it does
    // not, it will never see "my_prop" == 1.
    //
    // Returns NotFoundError if any of the part or property names in
    // `properties` are unknown. Returns InvalidArgument if a property value has
    // the wrong type (bool instead of double, or vice versa).
    virtual absl::Status SetPartProperties(
        const PartPropertyMap& properties) = 0;
  };

  // Verifies that `status_getter` returns status messages for each part in
  // `part_configs`, then builds a PartCollection.
  // Note that invoking `status_getter` must be thread-safe.
  //
  // Takes ownership of its arguments.
  //
  // Returns InvalidArgumentError if there's an error retrieving status values
  // from `status_getter`, or if the status values returned are inconsistent
  // with `part_configs`.
  static absl::StatusOr<PartCollection> Create(
      absl::Span<const intrinsic_proto::icon::v1::PartConfig> part_configs,
      std::unique_ptr<StatusGetter> status_getter);

  // Returns one PartConfig proto for each part.
  // The PartConfigs are specific to the currently active ModeOfSafeOperation
  // and all sessions end on changes of ModeOfSafeOperation. Call
  // GetPartConfigs() again to get updated PartConfigs. This method is
  // thread-safe.
  std::vector<intrinsic_proto::icon::v1::PartConfig> GetPartConfigs() const;

  // Retrieves the current status of all parts using `status_getter`, then
  // returns a map from part name to part status proto.
  // This method is thread-safe because `status_getter` is thread-safe.
  absl::StatusOr<
      absl::flat_hash_map<std::string, intrinsic_proto::icon::PartStatus>>
  GetPartStatuses(absl::Time deadline) const;

  // Retrieves the latest part property values, including wall and control
  // timestamps.
  //
  // This method is thread-safe because `status_getter` is thread-safe.
  absl::StatusOr<TimestampedPartProperties> GetPartProperties();

  // Applies the given part property values atomically.
  //
  // Returns NotFoundError if any of the part or property names in
  // `properties` are unknown. Returns InvalidArgument if a property value has
  // the wrong type (bool instead of double, or vice versa).
  // This method is thread-safe because `status_getter` is thread-safe.
  absl::Status SetPartProperties(const PartPropertyMap& properties);

  // Retrieves the current status of the safety system using `status_getter`.
  // This method is thread-safe because `status_getter` is thread-safe.
  absl::StatusOr<intrinsic_proto::icon::SafetyStatus> GetSafetyStatus(
      absl::Time deadline) const;

  // Returns the PartManager. Use this to claim and release ownership of parts.
  // PartsManager is thread-safe, and so are these two methods.
  internal::PartsManager& GetPartsManager();
  const internal::PartsManager& GetPartsManager() const;

 private:
  PartCollection(absl::flat_hash_set<std::string> part_names,
                 absl::flat_hash_set<std::string> config_part_names,
                 absl::flat_hash_map<
                     intrinsic_proto::icon::ModeOfSafeOperation,
                     const std::vector<intrinsic_proto::icon::v1::PartConfig>>
                     part_configs_map,
                 std::unique_ptr<StatusGetter> status_getter);

  // Holds part_configs for every ModeOfSafeOperation.
  // The configs are created once and constant.
  const absl::flat_hash_map<
      intrinsic_proto::icon::ModeOfSafeOperation,
      const std::vector<intrinsic_proto::icon::v1::PartConfig>>
      mode_of_safe_operation_part_configs_map_;

  const absl::flat_hash_set<std::string> config_part_names_;
  std::unique_ptr<StatusGetter> status_getter_;
  // unique_ptr because PartsManager itself is not movable
  std::unique_ptr<internal::PartsManager> parts_manager_;
};
}  // namespace intrinsic::icon

#endif  // INTRINSIC_ICON_SERVER_PART_COLLECTION_H_
