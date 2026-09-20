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

#ifndef INTRINSIC_PROTO_TOOLS_REGISTRY_REGISTRY_PUBSUB_H_
#define INTRINSIC_PROTO_TOOLS_REGISTRY_REGISTRY_PUBSUB_H_

#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "absl/base/thread_annotations.h"
#include "absl/container/flat_hash_map.h"
#include "absl/container/flat_hash_set.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/synchronization/mutex.h"
#include "google/protobuf/descriptor.pb.h"
#include "intrinsic_runtime/intrinsic/proto_tools/registry/resolver.h"
#include "intrinsic/platform/pubsub/pubsub.h"
#include "intrinsic/platform/pubsub/queryable.h"
#include "intrinsic/util/proto/parsed_type_url.h"

namespace intrinsic::proto_registry {

class PubSubRegistry {
 public:
  PubSubRegistry() = delete;
  PubSubRegistry(const PubSubRegistry&) = delete;
  PubSubRegistry(PubSubRegistry&&) = delete;
  PubSubRegistry& operator=(const PubSubRegistry&) = delete;
  PubSubRegistry& operator=(PubSubRegistry&&) = delete;
  static absl::StatusOr<std::unique_ptr<PubSubRegistry>> Create();

  class PubSubResolver : public Resolver {
   public:
    explicit PubSubResolver(const PubSubRegistry& pubsub_registry)
        : pubsub_registry_(pubsub_registry) {}
    PubSubResolver() = delete;
    PubSubResolver(const PubSubResolver&) = delete;
    PubSubResolver(PubSubResolver&&) = delete;
    PubSubResolver operator=(const PubSubResolver&) = delete;
    PubSubResolver operator=(PubSubResolver&&) = delete;

    std::string_view GetResolverName() const override {
      return "PubSubResolver";
    }
    std::vector<std::string_view> GetAreas() const override {
      return {"topics"};
    }

    absl::StatusOr<google::protobuf::FileDescriptorSet> Resolve(
        const ParsedUrl& parsed_url) override;

   private:
    const PubSubRegistry& pubsub_registry_;
  };

  std::unique_ptr<Resolver> GetResolver() {
    return std::make_unique<PubSubResolver>(*this);
  }

 private:
  explicit PubSubRegistry(PubSub&& pubsub);
  absl::Status Init();

  struct RegisterTopicOptions {};

  absl::StatusOr<google::protobuf::FileDescriptorSet>
  FindTopicFileDescriptorSet(std::string_view topic_hash) const
      ABSL_LOCKS_EXCLUDED(registrations_mu_);

  struct RegistrationInfo {
    google::protobuf::FileDescriptorSet file_descriptor_set;
    std::string topic_hash;
    std::string type_url;
  };

  absl::StatusOr<RegistrationInfo> RegisterTopicType(
      std::string_view topic, std::string_view message_type,
      const google::protobuf::FileDescriptorSet& file_descriptor_set,
      const RegisterTopicOptions& options = {})
      ABSL_LOCKS_EXCLUDED(registrations_mu_);

  Queryable register_queryable_;
  Queryable get_queryable_;
  PubSub pubsub_;

  mutable absl::Mutex registrations_mu_;
  absl::flat_hash_map<std::string, RegistrationInfo> registrations_
      ABSL_GUARDED_BY(registrations_mu_);
};

}  // namespace intrinsic::proto_registry

#endif  // INTRINSIC_PROTO_TOOLS_REGISTRY_REGISTRY_PUBSUB_H_
