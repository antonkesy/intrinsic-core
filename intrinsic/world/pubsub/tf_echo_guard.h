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

#ifndef INTRINSIC_WORLD_PUBSUB_TF_ECHO_GUARD_H_
#define INTRINSIC_WORLD_PUBSUB_TF_ECHO_GUARD_H_

#include <string>
#include <vector>

#include "absl/container/btree_set.h"
#include "absl/container/flat_hash_map.h"
#include "absl/status/status.h"
#include "absl/synchronization/mutex.h"
#include "absl/time/time.h"
#include "google/protobuf/timestamp.pb.h"
#include "intrinsic/math/proto/tf_message.pb.h"
#include "intrinsic/world/pubsub/tf_echo_guard_interfaces.h"

namespace intrinsic {

class TfEchoGuard : public TfEchoGuardPublisherInterface,
                    public TfEchoGuardSubscriberInterface {
 public:
  TfEchoGuard() = default;

  virtual ~TfEchoGuard() = default;

  absl::Status FilterTfMessagePendingPublish(
      intrinsic_proto::TFMessage& message_to_filter) override;

  absl::Status RecordPublishedMessage(
      const intrinsic_proto::TFMessage& tf_message) override;

  absl::Status RecordSuccessfulUpdates(
      absl::flat_hash_map<std::string, google::protobuf::Timestamp>
          successful_updates) override;

  absl::Status FilterPendingUpdates(
      std::vector<intrinsic_proto::TransformStamped>& pending_updates) override;

  static constexpr size_t kMaxHistorySize = 100;

 private:
  absl::Mutex successful_tf_updates_mu_;
  // A tf frame id to latest update timestamp mapping. Stores the latest
  // succesful timestamps when a external source updated tf frames through the
  // world updater. Use these to avoid publishing tf frames that are no later
  // than than these timestamps.
  absl::flat_hash_map<std::string, google::protobuf::Timestamp>
      successful_tf_updates_ ABSL_GUARDED_BY(successful_tf_updates_mu_);

  absl::Mutex world_published_tf_mu_;
  // A tf frame id to a history of world published timestamps. Stores the
  // recent timestamps when the world published world authoritative tf
  // frames. Use these to avoid attempting to update the world with tf frames
  // that match these timestamps (echoes).
  absl::flat_hash_map<std::string, absl::btree_set<absl::Time>>
      world_published_tf_ ABSL_GUARDED_BY(world_published_tf_mu_);
};

}  // namespace intrinsic

#endif  // INTRINSIC_WORLD_PUBSUB_TF_ECHO_GUARD_H_
