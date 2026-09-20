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

#include "intrinsic_runtime/intrinsic/proto_tools/registry/resolver_common.h"

#include <memory>
#include <string>
#include <string_view>
#include <utility>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_format.h"
#include "google/protobuf/descriptor.h"
#include "google/protobuf/descriptor.pb.h"
#include "intrinsic_runtime/intrinsic/proto_tools/util/descriptor_pool_loader.h"
#include "intrinsic/util/proto/parsed_type_url.h"
#include "intrinsic/util/status/status_macros.h"
#include "intrinsic/util/status/status_specs.h"

namespace intrinsic::proto_registry {

namespace {

inline constexpr std::string_view kDefaultCommonTypesRunfilesPath =
    "intrinsic_runtime/intrinsic/proto_tools/registry/"
    "common_types_transitive_set_sci.proto.bin";

class CommonResolverImpl : public CommonResolver {
 public:
  explicit CommonResolverImpl(
      std::unique_ptr<intrinsic::executive::LoadedDescriptorPool> loaded_pool)
      : loaded_pool_(std::move(loaded_pool)) {}

  absl::StatusOr<google::protobuf::FileDescriptorSet> Resolve(
      const ParsedUrl& parsed_url) override {
    if (!parsed_url.path.empty()) {
      return CreateStatus(12001,
                          absl::StrFormat("Invalid path '%s' for common type "
                                          "URL. Expected empty path.",
                                          parsed_url.path),
                          absl::StatusCode::kInvalidArgument);
    }

    if (parsed_url.message_type.empty() ||
        loaded_pool_->descriptor_pool()->FindMessageTypeByName(
            parsed_url.message_type) != nullptr ||
        loaded_pool_->descriptor_pool()->FindEnumTypeByName(
            parsed_url.message_type) != nullptr) {
      // TODO(b/530915170): Return only the relevant part of the descriptor set
      // when a specific message or enum was requested.
      return loaded_pool_->file_descriptor_set();
    }
    return CreateStatus(
        12101,
        absl::StrFormat("Message '%s' is not a known common type",
                        parsed_url.message_type),
        absl::StatusCode::kNotFound);
  }

 private:
  std::unique_ptr<intrinsic::executive::LoadedDescriptorPool> loaded_pool_;
};

}  // namespace

absl::StatusOr<std::unique_ptr<CommonResolver>> CommonResolver::Create() {
  INTR_ASSIGN_OR_RETURN(
      std::unique_ptr<intrinsic::executive::LoadedDescriptorPool> loaded_pool,
      intrinsic::executive::LoadedDescriptorPool::LoadFromRunfiles(
          kDefaultCommonTypesRunfilesPath));

  return std::make_unique<CommonResolverImpl>(std::move(loaded_pool));
}

}  // namespace intrinsic::proto_registry
