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

#include "intrinsic/icon/server/parts_manager.h"

#include <algorithm>
#include <string>
#include <vector>

#include "absl/container/flat_hash_set.h"
#include "absl/status/status.h"
#include "absl/strings/str_cat.h"
#include "absl/synchronization/mutex.h"

namespace intrinsic::icon::internal {

namespace {

std::vector<std::string> MakeSortedParts(
    const absl::flat_hash_set<std::string>& parts) {
  std::vector<std::string> sorted_parts(parts.begin(), parts.end());
  std::sort(sorted_parts.begin(), sorted_parts.end());
  return sorted_parts;
}

}  // namespace

PartsManager::PartsManager(const absl::flat_hash_set<std::string>& parts)
    : parts_sorted_(MakeSortedParts(parts)), available_parts_(parts) {}

const std::vector<std::string>& PartsManager::AllParts() const {
  return parts_sorted_;
}

std::vector<std::string> PartsManager::AvailableParts() const {
  absl::ReaderMutexLock l(parts_mutex_);
  return MakeSortedParts(available_parts_);
}

std::vector<std::string> PartsManager::UnavailableParts() const {
  absl::ReaderMutexLock l(parts_mutex_);
  return MakeSortedParts(unavailable_parts_);
}

absl::Status PartsManager::SetPartsAsUnavailable(
    const absl::flat_hash_set<std::string>& part_names) {
  absl::MutexLock l(parts_mutex_);
  // Check that all parts are present and available
  for (const auto& part : part_names) {
    if (unavailable_parts_.contains(part)) {
      return absl::FailedPreconditionError(
          absl::StrCat("Part: '", part, "' is already in use."));
    }
    if (!available_parts_.contains(part)) {
      return absl::NotFoundError(
          absl::StrCat("Part: '", part, "' does not exist on this server."));
    }
  }

  // Move parts from available to unavailable
  for (const auto& part : part_names) {
    available_parts_.erase(part);
    unavailable_parts_.insert(part);
  }

  return absl::OkStatus();
}

absl::Status PartsManager::SetPartsAsAvailable(
    const absl::flat_hash_set<std::string>& part_names) {
  absl::MutexLock l(parts_mutex_);
  // Check that all parts are present
  for (const auto& part : part_names) {
    if (!unavailable_parts_.contains(part) &&
        !available_parts_.contains(part)) {
      return absl::NotFoundError(
          absl::StrCat("Part: '", part, "' does not exist on this server."));
    }
  }

  // Move parts from unavailable to available
  for (const auto& part : part_names) {
    unavailable_parts_.erase(part);
    available_parts_.insert(part);
  }

  return absl::OkStatus();
}
}  // namespace intrinsic::icon::internal
