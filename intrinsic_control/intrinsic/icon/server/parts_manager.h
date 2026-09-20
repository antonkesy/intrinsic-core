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

#ifndef INTRINSIC_ICON_SERVER_PARTS_MANAGER_H_
#define INTRINSIC_ICON_SERVER_PARTS_MANAGER_H_

#include <string>
#include <vector>

#include "absl/base/thread_annotations.h"
#include "absl/container/flat_hash_set.h"
#include "absl/status/status.h"
#include "absl/synchronization/mutex.h"

namespace intrinsic::icon::internal {

// PartsManager keeps track of which of a set of parts (identified by string
// names) are currently available and unavailable. Claiming and releasing parts
// is implemented in a thread-safe manner.
class PartsManager {
 public:
  explicit PartsManager(const absl::flat_hash_set<std::string>& parts);

  // Gets the part names of all parts, sorted alphabetically.
  //
  // This method is thread-safe.
  const std::vector<std::string>& AllParts() const;

  // Gets the part names of all available parts, sorted alphabetically.
  //
  // This method is thread-safe.
  std::vector<std::string> AvailableParts() const;

  // Gets the part names of all unavailable (i.e. currently claimed) parts,
  // sorted alphabetically.
  //
  // This method is thread-safe.
  std::vector<std::string> UnavailableParts() const;

  // Marks the `part_names` as unavailable. Returns an error if any part name
  // is unknown, or is known but already marked unavailable. If an error is
  // returned, no parts are marked unavailable.
  //
  // This method is thread-safe.
  absl::Status SetPartsAsUnavailable(
      const absl::flat_hash_set<std::string>& part_names);

  // Marks the `part_names` as available. Returns an error if any part name is
  // unknown. If an error is returned, no parts are marked as available. If a
  // part in `part_names` is already marked available, it stays available and no
  // error is returned.
  //
  // This method is thread-safe.
  absl::Status SetPartsAsAvailable(
      const absl::flat_hash_set<std::string>& part_names);

 private:
  // Part names, sorted alphabetically.
  const std::vector<std::string> parts_sorted_;

  // Mutable to allow AvailableParts() and UnavailableParts() to be const.
  mutable absl::Mutex parts_mutex_;
  // All parts in `parts_sorted_` are either available or unavailable.
  absl::flat_hash_set<std::string> available_parts_
      ABSL_GUARDED_BY(parts_mutex_);
  absl::flat_hash_set<std::string> unavailable_parts_
      ABSL_GUARDED_BY(parts_mutex_);
};

}  // namespace intrinsic::icon::internal

#endif  // INTRINSIC_ICON_SERVER_PARTS_MANAGER_H_
