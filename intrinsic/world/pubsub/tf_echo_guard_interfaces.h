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

#ifndef INTRINSIC_WORLD_PUBSUB_TF_ECHO_GUARD_INTERFACES_H_
#define INTRINSIC_WORLD_PUBSUB_TF_ECHO_GUARD_INTERFACES_H_

#include <string>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "absl/status/status.h"
#include "intrinsic/math/proto/tf_message.pb.h"

namespace intrinsic {

// `TfEchoGuardPublisherInterface` and `TfEchoGuardSubscriberInterface` are
// interfaces that we pass to a pair of publisher and subscriber that
// maintains the interface between the intrinsic world and the tf topic.
// We use these to avoid two echoing scenarios:
// 1. The world updater shouldn't attempt to update the world again with tf
// frames published by the world.
// 2. The world tf publisher shouldn't publish the same frame back to tf
// received from a tf subscription.
// Each interface should be used on a single thread only.
// For more information on tf frames in general, refer to
// https://docs.ros.org/en/kilted/Concepts/Intermediate/About-Tf2.html
class TfEchoGuardPublisherInterface {
 public:
  virtual ~TfEchoGuardPublisherInterface() = default;
  // Filters a message and removes any transforms that the world is not
  // authoritative.
  virtual absl::Status FilterTfMessagePendingPublish(
      intrinsic_proto::TFMessage& message_to_filter) = 0;

  // Records the latest tf frame messages containing world authoritative tf
  // frames.
  virtual absl::Status RecordPublishedMessage(
      const intrinsic_proto::TFMessage& tf_message) = 0;
};

class TfEchoGuardSubscriberInterface {
 public:
  virtual ~TfEchoGuardSubscriberInterface() = default;
  // Records the frame id and timestamp of latest tf frames that successfully
  // updated the world. These frames are not world authoritative.
  virtual absl::Status RecordSuccessfulUpdates(
      absl::flat_hash_map<std::string, google::protobuf::Timestamp>
          successful_updates) = 0;

  // Filters pending updates before applying, removes frames that are world
  // authoritative.
  virtual absl::Status FilterPendingUpdates(
      std::vector<intrinsic_proto::TransformStamped>& pending_updates) = 0;
};

}  // namespace intrinsic

#endif  // INTRINSIC_WORLD_PUBSUB_TF_ECHO_GUARD_INTERFACES_H_
