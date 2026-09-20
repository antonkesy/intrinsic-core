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

#ifndef INTRINSIC_ICON_CONTROL_PARTS_REALTIME_PART_FROM_PROTO_FACTORY_REGISTRY_H_
#define INTRINSIC_ICON_CONTROL_PARTS_REALTIME_PART_FROM_PROTO_FACTORY_REGISTRY_H_

#include <functional>
#include <memory>
#include <string>
#include <utility>

#include "absl/base/attributes.h"
#include "absl/base/thread_annotations.h"
#include "absl/container/flat_hash_map.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/string_view.h"
#include "absl/synchronization/mutex.h"
#include "google/protobuf/any.pb.h"
#include "intrinsic/icon/control/parts/feature_interface_registry.h"
#include "intrinsic/icon/control/parts/realtime_part_factory_common.h"
#include "intrinsic/icon/control/parts/realtime_part_interface.h"
#include "intrinsic/icon/proto/generic_part_config.pb.h"
#include "intrinsic/icon/proto/v1/types.pb.h"
#include "intrinsic/util/proto/descriptors.h"
#include "intrinsic/util/status/status_macros.h"

namespace intrinsic::icon {

// A global registry for create functions for RealtimeParts.
// 'Register' should be called at static initialization time, see
// intrinsic/icon/control/parts/hal/arm_part/hal_arm_part_register.cc
// for how to register a realtime part.
//
// Thread safe. Do not use in realtime contexts. (This wrapper asserts that
// calls do not come from realtime.)
class RealtimePartFromProtoFactoryRegistry {
 public:
  // TODO(b/213573627): Migrate Part factories, then remove this
  using OldSignature = absl::StatusOr<std::unique_ptr<RealtimePartInterface>>(
      PartFactoryContext context, const google::protobuf::Any& config);
  using FactorySignature = absl::StatusOr<PartPtrAndPartConfig>(
      PartFactoryContext context, const google::protobuf::Any& config);

  template <typename ProtoT>
  using TypedOldSignature =
      absl::StatusOr<std::unique_ptr<RealtimePartInterface>>(
          PartFactoryContext context, const ProtoT& config);
  template <typename ProtoT>
  using TypedSignature = absl::StatusOr<PartPtrAndGenericConfig>(
      PartFactoryContext context, const ProtoT& config);

  // Registers a factory function that expects a specific config type.
  //
  // The factory that actually gets registered will be a wrapper around
  // `factory`. the wrapper factory first converts the config from an Any proto
  // to the specific message type, then invokes the provided `factory`. The
  // wrapper factory returns InvalidArgumentError if it cannot unpack the any
  // proto to a message of type ProtoT.
  template <typename ProtoT>
  bool RegisterTyped(absl::string_view part_type_name,
                     std::function<TypedSignature<ProtoT>> factory) {
    return Register(part_type_name, WrapFactory(part_type_name, factory));
  }

  template <typename ProtoT>
  ABSL_DEPRECATED(
      "Do not use the old Part Factory signature for new Parts (see "
      "b/213573627)")
  bool RegisterTyped(absl::string_view part_type_name,
                     std::function<TypedOldSignature<ProtoT>> factory) {
    return Register(part_type_name, WrapOldFactory(part_type_name, factory));
  }

  std::function<FactorySignature> Get(absl::string_view part_type_name) const;

  void ClearForTestingOnly();

 private:
  // Registers a factory function that expects an Any proto config.
  bool Register(absl::string_view part_type_name,
                std::function<FactorySignature> factory);

  // Converts a TypedSignature to a generic Signature by wrapping the factory
  // function.
  template <typename ProtoT>
  std::function<FactorySignature> WrapOldFactory(
      absl::string_view part_type_name,
      std::function<TypedOldSignature<ProtoT>> factory);
  template <typename ProtoT>
  std::function<FactorySignature> WrapFactory(
      absl::string_view part_type_name,
      std::function<TypedSignature<ProtoT>> factory);

  mutable absl::Mutex mutex_;
  absl::flat_hash_map<std::string, std::function<FactorySignature>> registry_
      ABSL_GUARDED_BY(mutex_);
};

// Access to the global registry.
RealtimePartFromProtoFactoryRegistry&
GetGlobalRealtimePartFromProtoFactoryRegistry();

// WrapOldFactory implementation.
template <typename ProtoT>
std::function<RealtimePartFromProtoFactoryRegistry::FactorySignature>
RealtimePartFromProtoFactoryRegistry::WrapOldFactory(
    absl::string_view part_type_name,
    std::function<TypedOldSignature<ProtoT>> factory) {
  // Return a wrapper around `factory`.
  return [factory = std::move(factory),
          part_type_name = std::string(part_type_name)](
             PartFactoryContext context,
             const google::protobuf::Any& any_config)
             -> absl::StatusOr<PartPtrAndPartConfig> {
    ProtoT config;
    if (!any_config.UnpackTo(&config)) {
      return absl::InvalidArgumentError(absl::StrCat(
          "Failed to unpack config for part '", context.part_name,
          "' of type '", part_type_name, "': expected a config of type '",
          ProtoT::GetDescriptor()->full_name(), "' but got a config of type '",
          any_config.type_url()));
    }
    INTR_ASSIGN_OR_RETURN(std::unique_ptr<RealtimePartInterface> part_ptr,
                          factory(context, config));
    intrinsic_proto::icon::v1::PartConfig part_config;
    part_config.set_name(context.part_name);
    part_config.set_hardware_resource_name(context.hardware_resource_name);
    part_config.set_part_type_name(part_type_name);
    *part_config.mutable_config() = any_config;
    const auto& supported_feature_interfaces =
        part_ptr->GetFeatureInterfaces().SupportedFeatureInterfaceTypes();
    *part_config.mutable_generic_config() =
        ExtractGenericConfig(part_ptr->GetFeatureInterfaces());
    INTR_RETURN_IF_ERROR(ValidateGenericConfig(supported_feature_interfaces,
                                               part_config.generic_config()));
    for (const auto& feature_interface : supported_feature_interfaces) {
      part_config.add_feature_interfaces(feature_interface);
    }
    part_config.set_config_message_type(ProtoT::GetDescriptor()->full_name());
    *part_config.mutable_config_descriptor_set() =
        GenFileDescriptorSet(*ProtoT::GetDescriptor());
    return PartPtrAndPartConfig{.part_ptr = std::move(part_ptr),
                                .config = std::move(part_config)};
  };
}

// WrapFactory implementation.
template <typename ProtoT>
std::function<RealtimePartFromProtoFactoryRegistry::FactorySignature>
RealtimePartFromProtoFactoryRegistry::WrapFactory(
    absl::string_view part_type_name,
    std::function<TypedSignature<ProtoT>> factory) {
  // Return a wrapper around `factory`.
  return [factory = std::move(factory),
          part_type_name = std::string(part_type_name)](
             PartFactoryContext context,
             const google::protobuf::Any& any_config)
             -> absl::StatusOr<PartPtrAndPartConfig> {
    ProtoT config;
    if (!any_config.UnpackTo(&config)) {
      return absl::InvalidArgumentError(absl::StrCat(
          "Failed to unpack config for part '", context.part_name,
          "' of type '", part_type_name, "': expected a config of type '",
          ProtoT::GetDescriptor()->full_name(), "' but got a config of type '",
          any_config.type_url()));
    }

    INTR_ASSIGN_OR_RETURN(PartPtrAndGenericConfig part_ptr_and_config,
                          factory(context, config));
    intrinsic_proto::icon::v1::PartConfig part_config;
    part_config.set_name(context.part_name);
    part_config.set_hardware_resource_name(context.hardware_resource_name);
    part_config.set_part_type_name(part_type_name);
    *part_config.mutable_config() = any_config;
    const auto& supported_feature_interfaces =
        part_ptr_and_config.part_ptr->GetFeatureInterfaces()
            .SupportedFeatureInterfaceTypes();
    *part_config.mutable_generic_config() =
        std::move(part_ptr_and_config.config);
    INTR_RETURN_IF_ERROR(ValidateGenericConfig(supported_feature_interfaces,
                                               part_config.generic_config()));
    for (const auto& feature_interface : supported_feature_interfaces) {
      part_config.add_feature_interfaces(feature_interface);
    }
    part_config.set_config_message_type(ProtoT::GetDescriptor()->full_name());
    *part_config.mutable_config_descriptor_set() =
        GenFileDescriptorSet(*ProtoT::GetDescriptor());
    return PartPtrAndPartConfig{
        .part_ptr = std::move(part_ptr_and_config.part_ptr),
        .config = std::move(part_config)};
  };
}

}  // namespace intrinsic::icon

#endif  // INTRINSIC_ICON_CONTROL_PARTS_REALTIME_PART_FROM_PROTO_FACTORY_REGISTRY_H_
