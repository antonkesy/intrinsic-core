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

#include "intrinsic/icon/control/rtcl_action_factory_registry.h"

#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "absl/base/no_destructor.h"
#include "absl/container/flat_hash_map.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/string_view.h"
#include "absl/synchronization/mutex.h"
#include "cppregpattern/registry.h"
#include "google/protobuf/any.pb.h"
#include "google/protobuf/descriptor.h"
#include "intrinsic/icon/control/action_factory_context.h"
#include "intrinsic/icon/control/rtcl_action.h"
#include "intrinsic/icon/proto/v1/types.pb.h"
#include "intrinsic/icon/utils/realtime_guard.h"

namespace intrinsic::icon {
namespace {

using ActionFactoryRegistry =
    registry::Registry<std::string, RtclActionFactoryRegistry::GenericSignature,
                       registry::MissingKeyPolicy::default_construct>;

bool IsEmpty(const google::protobuf::Message& message) {
  const google::protobuf::Reflection* reflection = message.GetReflection();
  std::vector<const google::protobuf::FieldDescriptor*> fields;
  reflection->ListFields(message, &fields);
  return fields.empty() && reflection->GetUnknownFields(message).empty();
}

// Wraps `factory` to conform to the "raw" signature API. This includes a check
// that ensures that the Action receives no parameters.
std::function<RtclActionFactoryRegistry::GenericSignature>
WrapNoParametersFactory(
    absl::string_view action_type_name,
    std::function<RtclActionFactoryRegistry::NoParametersSignature> factory) {
  return [no_parameters_factory = std::move(factory),
          action_type_name = std::string(action_type_name)](
             const google::protobuf::Any& params_any,
             ActionFactoryContext& context)
             -> absl::StatusOr<std::unique_ptr<RtclActionInterface>> {
    if (!IsEmpty(params_any)) {
      return absl::InvalidArgumentError(
          absl::StrCat("Factory for default action type '", action_type_name,
                       "'  expected no parameters, but received '",
                       params_any.type_url(), "'."));
    }
    return no_parameters_factory(context);
  };
}
}  // namespace

bool RtclActionFactoryRegistry::RegisterGeneric(
    absl::string_view action_type_name, std::function<GenericSignature> factory,
    const intrinsic_proto::icon::v1::ActionSignature& signature) {
  INTRINSIC_ASSERT_NON_REALTIME();
  absl::MutexLock l(mutex_);
  return ActionFactoryRegistry::Register(std::string(action_type_name),
                                         std::move(factory)) &&
         signature_registry_.try_emplace(action_type_name, signature).second;
}

bool RtclActionFactoryRegistry::RegisterNoParameters(
    absl::string_view action_type_name,
    std::function<NoParametersSignature> factory,
    const intrinsic_proto::icon::v1::ActionSignature& signature) {
  INTRINSIC_ASSERT_NON_REALTIME();
  return RegisterGeneric(
      action_type_name,
      WrapNoParametersFactory(action_type_name, std::move(factory)), signature);
}

absl::StatusOr<std::unique_ptr<RtclActionInterface>>
RtclActionFactoryRegistry::CallFactory(absl::string_view action_type_name,
                                       const google::protobuf::Any& params,
                                       ActionFactoryContext& context) const {
  INTRINSIC_ASSERT_NON_REALTIME();
  absl::MutexLock l(mutex_);
  if (!ActionFactoryRegistry::IsRegistered(
          std::string(std::string(action_type_name)))) {
    return absl::NotFoundError(absl::StrCat(
        "There is no factory for action type '", action_type_name,
        "'. Did you link the corresponding _register target / load the "
        "corresponding plugin?"));
  };
  return ActionFactoryRegistry::Dispatch(std::string(action_type_name), params,
                                         context);
}

absl::StatusOr<intrinsic_proto::icon::v1::ActionSignature>
RtclActionFactoryRegistry::GetSignature(
    absl::string_view action_type_name) const {
  INTRINSIC_ASSERT_NON_REALTIME();
  absl::MutexLock l(mutex_);
  auto maybe_signature = signature_registry_.find(action_type_name);
  if (maybe_signature == signature_registry_.end()) {
    return absl::NotFoundError(absl::StrCat(
        "No ActionSignature available for Action type '", action_type_name,
        "', did you forget to register it? (If using a _register library, "
        "make sure it has alwayslink=True set.)"));
  }
  return maybe_signature->second;
}

void RtclActionFactoryRegistry::ClearForTestingOnly() {
  INTRINSIC_ASSERT_NON_REALTIME();
  absl::MutexLock l(mutex_);
  for (auto& [key, value] : signature_registry_) {
    ActionFactoryRegistry::Unregister(key);
  }
  signature_registry_.clear();
}

absl::flat_hash_map<std::string, intrinsic_proto::icon::v1::ActionSignature>
RtclActionFactoryRegistry::GetAllSignatures() const {
  INTRINSIC_ASSERT_NON_REALTIME();
  absl::MutexLock l(mutex_);
  return signature_registry_;
}

RtclActionFactoryRegistry& GetGlobalRtclActionFactoryRegistry() {
  INTRINSIC_ASSERT_NON_REALTIME();
  static absl::NoDestructor<RtclActionFactoryRegistry> kRegistry;
  return *kRegistry;
}

}  // namespace intrinsic::icon
