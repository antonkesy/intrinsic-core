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

#include "intrinsic_runtime/intrinsic/proto_tools/registry/registry_pubsub.h"

#include <cstdint>
#include <memory>
#include <string>
#include <string_view>
#include <utility>

#include "absl/base/thread_annotations.h"
#include "absl/container/flat_hash_set.h"
#include "absl/memory/memory.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/escaping.h"
#include "absl/strings/str_format.h"
#include "absl/synchronization/mutex.h"
#include "intrinsic/platform/pubsub/pubsub.h"
#include "intrinsic/platform/pubsub/queryable.h"
#include "intrinsic/proto_tools/proto/proto_registry_pubsub.pb.h"
#include "intrinsic/util/proto/parsed_type_url.h"
#include "intrinsic/util/proto/type_url.h"
#include "intrinsic/util/status/status_macros.h"
#include "intrinsic/util/status/status_specs.h"
#include "openssl/is_boringssl.h"
// NOLINTNEXTLINE(google3-security-unsafe-crypto-dependencies)
#include "openssl/sha.h"

namespace intrinsic::proto_registry {

constexpr std::string_view kRegisterQueryableKey = "_types/register";
constexpr std::string_view kGetQueryableKey = "_types/get";

namespace {
std::string GenerateTopicHashString(std::string_view topic_name) {
  uint8_t sha256_hash[SHA256_DIGEST_LENGTH];
  SHA256(reinterpret_cast<const uint8_t*>(topic_name.data()), topic_name.size(),
         sha256_hash);
  return absl::BytesToHexString(std::string_view(
      reinterpret_cast<const char*>(sha256_hash), SHA256_DIGEST_LENGTH));
}
}  // namespace

PubSubRegistry::PubSubRegistry(PubSub&& pubsub) : pubsub_(std::move(pubsub)) {}

absl::Status PubSubRegistry::Init() {
  INTR_ASSIGN_OR_RETURN(
      register_queryable_,
      pubsub_.CreateQueryable(
          kRegisterQueryableKey,
          [this](
              std::string_view keyexpr, const QueryableContext& context,
              const intrinsic_proto::proto_registry::RegisterTopicTypeRequest&
                  request,
              intrinsic_proto::proto_registry::RegisterTopicTypeResponse&
                  response) -> absl::Status {
            INTR_ASSIGN_OR_RETURN(
                PubSubRegistry::RegistrationInfo info,
                RegisterTopicType(request.topic(), request.message_full_name(),
                                  request.file_descriptor_set()));
            response.mutable_info()->set_type_url(info.type_url);
            response.mutable_info()->set_topic_hash(info.topic_hash);
            return absl::OkStatus();
          }));

  INTR_ASSIGN_OR_RETURN(
      get_queryable_,
      pubsub_.CreateQueryable(
          kGetQueryableKey,
          [](std::string_view keyexpr, const QueryableContext& context,
             const intrinsic_proto::proto_registry::GetTopicTypeRequest&
                 request,
             intrinsic_proto::proto_registry::GetTopicTypeResponse& response)
              -> absl::Status {
            // call function that gets type
            return absl::UnimplementedError("Not yet implemented");
          }));

  return absl::OkStatus();
}

absl::StatusOr<std::unique_ptr<PubSubRegistry>> PubSubRegistry::Create() {
  PubSub pubsub;
  auto pubsub_registry =
      absl::WrapUnique(new PubSubRegistry(std::move(pubsub)));
  INTR_RETURN_IF_ERROR(pubsub_registry->Init());
  return pubsub_registry;
}

absl::StatusOr<PubSubRegistry::RegistrationInfo>
PubSubRegistry::RegisterTopicType(
    std::string_view topic, std::string_view message_full_name,
    const google::protobuf::FileDescriptorSet& file_descriptor_set,
    const RegisterTopicOptions& options) {
  absl::MutexLock lock(registrations_mu_);

  std::string topic_hash = GenerateTopicHashString(topic);

  // If an entry exists, check
  if (auto it = registrations_.find(topic_hash); it != registrations_.end()) {
    // TODO: compatibility check
    return it->second;
  }

  std::string type_url =
      GenerateIntrinsicTypeUrl("topics", topic_hash, message_full_name);

  RegistrationInfo info{.file_descriptor_set = file_descriptor_set,
                        .topic_hash = topic_hash,
                        .type_url = type_url};
  registrations_[topic_hash] = info;

  return info;
}

absl::StatusOr<google::protobuf::FileDescriptorSet>
PubSubRegistry::FindTopicFileDescriptorSet(std::string_view topic_hash) const
    ABSL_LOCKS_EXCLUDED(registrations_mu_) {
  absl::MutexLock lock(registrations_mu_);
  if (auto const it = registrations_.find(topic_hash);
      it != registrations_.end()) {
    return it->second.file_descriptor_set;
  }
  return CreateStatus(12100,
                      absl::StrFormat("Topic hash '%s' is unknown", topic_hash),
                      /*generic_code=*/absl::StatusCode::kNotFound);
}

absl::StatusOr<google::protobuf::FileDescriptorSet>
PubSubRegistry::PubSubResolver::Resolve(const ParsedUrl& parsed_url) {
  const std::string_view topic_hash = parsed_url.path;
  return pubsub_registry_.FindTopicFileDescriptorSet(topic_hash);
}
}  // namespace intrinsic::proto_registry
