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

#include "intrinsic/world/pubsub/tf_echo_guard.h"

#include <algorithm>
#include <string>
#include <vector>

#include "absl/base/thread_annotations.h"
#include "absl/container/flat_hash_map.h"
#include "absl/log/log.h"
#include "absl/status/status.h"
#include "absl/synchronization/mutex.h"
#include "google/protobuf/timestamp.pb.h"
#include "google/protobuf/util/time_util.h"
#include "intrinsic/math/proto/tf_message.pb.h"
#include "intrinsic/util/proto/repeated_field_util.h"
#include "intrinsic/util/proto_time.h"

namespace intrinsic {

using ::intrinsic_proto::TFMessage;
using ::intrinsic_proto::TransformStamped;

absl::Status TfEchoGuard::FilterTfMessagePendingPublish(
    TFMessage& message_to_filter) {
  absl::MutexLock lock(&successful_tf_updates_mu_);
  auto num_removed =
      RemoveIf(message_to_filter.mutable_transforms(),
               [this](const TransformStamped* t) {
                 successful_tf_updates_mu_.AssertHeld();
                 auto it = successful_tf_updates_.find(t->child_frame_id());
                 if (it != successful_tf_updates_.end()) {
                   return t->header().stamp() <= it->second;
                 }
                 return false;
               });
  VLOG(1) << "TfEchoGuard removed " << num_removed << " transforms.";
  return absl::OkStatus();
}

absl::Status TfEchoGuard::RecordPublishedMessage(const TFMessage& tf_message) {
  absl::MutexLock lock(&world_published_tf_mu_);
  for (const auto& t : tf_message.transforms()) {
    auto stamp = ToAbslTime(t.header().stamp());
    if (!stamp.ok()) {
      LOG(ERROR) << "Failed to convert timestamp: " << stamp.status();
      continue;
    }
    absl::Time time = *stamp;
    auto& history = world_published_tf_[t.child_frame_id()];
    history.insert(time);
    if (history.size() > kMaxHistorySize) {
      history.erase(history.begin());
    }
  }
  return absl::OkStatus();
}

absl::Status TfEchoGuard::RecordSuccessfulUpdates(
    absl::flat_hash_map<std::string, google::protobuf::Timestamp>
        successful_updates) {
  {
    absl::MutexLock lock(&successful_tf_updates_mu_);
    for (const auto& [frame_id, stamp] : successful_updates) {
      auto& current_stamp = successful_tf_updates_[frame_id];
      current_stamp = std::max(stamp, current_stamp);
    }
  }

  {
    absl::MutexLock lock(&world_published_tf_mu_);
    for (const auto& [frame_id, stamp] : successful_updates) {
      auto it = world_published_tf_.find(frame_id);
      if (it != world_published_tf_.end()) {
        auto stamp_absl = ToAbslTime(stamp);
        if (stamp_absl.ok()) {
          auto& history = it->second;
          auto bound = history.lower_bound(*stamp_absl);
          history.erase(history.begin(), bound);
        }
      }
    }
  }
  return absl::OkStatus();
}

absl::Status TfEchoGuard::FilterPendingUpdates(
    std::vector<TransformStamped>& pending_updates) {
  absl::MutexLock lock(&world_published_tf_mu_);
  auto remove_result = std::remove_if(
      pending_updates.begin(), pending_updates.end(),
      [this](const TransformStamped& t) {
        world_published_tf_mu_.AssertHeld();
        auto it = world_published_tf_.find(t.child_frame_id());
        if (it == world_published_tf_.end()) {
          return false;
        }
        auto stamp = ToAbslTime(t.header().stamp());
        if (!stamp.ok()) {
          LOG(ERROR) << "Failed to convert timestamp: " << stamp.status();
          return false;
        }
        absl::Time time = *stamp;
        const auto& history = it->second;
        return history.contains(time);
      });
  pending_updates.erase(remove_result, pending_updates.end());
  return absl::OkStatus();
}

}  // namespace intrinsic
