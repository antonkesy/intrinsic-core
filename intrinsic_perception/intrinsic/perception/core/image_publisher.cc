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

#include "intrinsic/perception/core/image_publisher.h"

#include <memory>
#include <string>
#include <utility>

#include "absl/base/no_destructor.h"
#include "absl/base/thread_annotations.h"
#include "absl/container/flat_hash_map.h"
#include "absl/status/status.h"
#include "absl/strings/string_view.h"
#include "absl/synchronization/mutex.h"
#include "intrinsic/platform/pubsub/publisher.h"
#include "intrinsic/platform/pubsub/pubsub.h"
#include "intrinsic/util/status/status_macros.h"

namespace intrinsic {
namespace perception {

absl::Status PublishImage(
    const intrinsic_proto::perception::v1::ImageBuffer& image,
    absl::string_view topic) {
  static absl::Mutex mutex(absl::kConstInit);
  static auto* publishers ABSL_GUARDED_BY(mutex) =
      new absl::flat_hash_map<std::string, std::unique_ptr<Publisher>>();

  Publisher* publisher = nullptr;
  {
    absl::MutexLock lock(&mutex);
    auto it = publishers->find(topic);
    if (it == publishers->end()) {
      static const absl::NoDestructor<PubSub> pub_sub;
      INTR_ASSIGN_OR_RETURN(
          Publisher new_publisher,
          pub_sub->CreatePublisher(
              topic, TopicConfig{.topic_qos = TopicConfig::Sensor}));
      auto placed = publishers->emplace(
          std::string(topic),
          std::make_unique<Publisher>(std::move(new_publisher)));
      publisher = placed.first->second.get();
    } else {
      publisher = it->second.get();
    }
  }

  return publisher->Publish(image);
}

}  // namespace perception
}  // namespace intrinsic
