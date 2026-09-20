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

#ifndef INTRINSIC_ICON_CONTROL_RTCL_ACTION_FACTORY_REGISTRY_H_
#define INTRINSIC_ICON_CONTROL_RTCL_ACTION_FACTORY_REGISTRY_H_

#include <functional>
#include <memory>
#include <string>

#include "absl/base/thread_annotations.h"
#include "absl/container/flat_hash_map.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/string_view.h"
#include "absl/synchronization/mutex.h"
#include "google/protobuf/any.pb.h"
#include "intrinsic/icon/control/action_factory_context.h"
#include "intrinsic/icon/control/rtcl_action.h"
#include "intrinsic/icon/proto/v1/types.pb.h"

namespace intrinsic::icon {

// A global registry for create functions for realtime actions.
// 'Register' should be called at static initialization time, see
// intrinsic/icon/control/actions/empty_action_register.cc for how to
// register a realtime action. Thread safe. Do not use in realtime contexts.
// (This wrapper asserts that calls do not come from realtime.)
class RtclActionFactoryRegistry {
 public:
  // Most Actions should be using this signature for their factory – we add an
  // automated wrapper that checks and unpacks the proto parameters before
  // invoking the actual factory.
  //
  // The lifetime of `context' is only guaranteed for the function call to
  // 'Signature'.
  //
  // `params` contains proto of type ProtoT.
  //
  // `context` allows a factory to
  // * Access an intrinsic_proto::icon::v1::ServerConfig, which contains the
  // name of the
  //   server (useful for logging) and, more importantly, the control frequency.
  //   Many Actions will need this to parameterize algorithms (Actions are
  //   called once per cycle, so they can assume a fixed step size).
  // * Access configuration proto and a RealtimeSlotId for each Slot that
  //   appears in the Action's signature. The factory should use it to:
  //   * Read any configuration it needs for non-realtime initialization (for
  //     example, many Actions need the number of degrees of freedom to allocate
  //     vectors of the correct size)
  //   * Determine the RealtimeSlotId for all Slots it needs to control. These
  //     IDs are *required* to get access to those parts in realtime.
  // * Register parser/converter functions streaming input and output values.
  //   Look at the documentation of StreamingIoRegistry for
  //   details. Similar to RealtimeSlotIds, a factory should save the
  //   StreamingInputIds of any streaming inputs it registers, so that the
  //   Action can use them to actually read the inputs during cyclic operation.
  template <class ProtoT>
  using TypedSignature = absl::StatusOr<std::unique_ptr<RtclActionInterface>>(
      const ProtoT& params, ActionFactoryContext& context);

  // Factory for an Action that takes no parameters at all. Internally, this is
  // wrapped to conform to GenericSignature below.
  using NoParametersSignature =
      absl::StatusOr<std::unique_ptr<RtclActionInterface>>(
          ActionFactoryContext& context);

  // The type of the generic "Create" function that each real-time action
  // needs to provide.
  //
  // The parameters are as for TypedSignature, except that `params` contains an
  // Any proto, into which the actual Action parameters have been packed.
  using GenericSignature = absl::StatusOr<std::unique_ptr<RtclActionInterface>>(
      const google::protobuf::Any& params, ActionFactoryContext& context);

  // Registers a `factory` and `signature` for the given `action_type_name`.
  //
  // Most Action types should use this to register their factory, since it
  // automates the unpacking of Action parameters.
  //
  // Returns true on success, false if `action_type_name` is already associated
  // with a factory and signature.
  template <class ProtoT>
  bool RegisterTyped(
      absl::string_view action_type_name,
      std::function<TypedSignature<ProtoT>> factory,
      const intrinsic_proto::icon::v1::ActionSignature& signature);

  // Registers a `factory` and `signature` for the given `action_type_name`.
  //
  // Use this for Actions that do not expect any parameters (such as those that
  // are used as default Actions).
  //
  // Returns true on success, false if `action_type_name` is already associated
  // with a factory and signature.
  bool RegisterNoParameters(
      absl::string_view action_type_name,
      std::function<NoParametersSignature> factory,
      const intrinsic_proto::icon::v1::ActionSignature& signature);

  // Registers `factory` and `signature` for the given `action_type_name`. Only
  // use this if you explicitly need to use the "raw" Factory Signature and
  // cannot use `RegisterTyped` or `RegisterDefaultActionType` above.
  //
  // Returns true on success, false if `action_type_name` is already associated
  // with a factory and signature.
  bool RegisterGeneric(
      absl::string_view action_type_name,
      std::function<GenericSignature> factory,
      const intrinsic_proto::icon::v1::ActionSignature& signature);

  // Calls the factory for the given `action_type_name` with the given
  // `params` and `context` and returns the resulting RtclActionInterface.
  // Returns a NotFoundError if no factory is registered for the given
  // `action_type_name`.
  absl::StatusOr<std::unique_ptr<RtclActionInterface>> CallFactory(
      absl::string_view action_type_name, const google::protobuf::Any& params,
      ActionFactoryContext& context) const;
  absl::StatusOr<intrinsic_proto::icon::v1::ActionSignature> GetSignature(
      absl::string_view action_type_name) const;

  // Clears the registry completely. This is only for tests, and useful there to
  // clear the (global) registry between test cases in the same suite.
  void ClearForTestingOnly();

  // Returns a map from Action type name to the corresponding ActionSignature
  // proto.
  absl::flat_hash_map<std::string, intrinsic_proto::icon::v1::ActionSignature>
  GetAllSignatures() const;

 private:
  // Wraps `factory` to conform to the "raw" signature API. The wrapper attempts
  // to unpack the parameter Any proto into a `ProtoT`, and returns an error if
  // that fails.
  template <class ProtoT>
  std::function<GenericSignature> WrapTypedFactory(
      absl::string_view action_type_name,
      std::function<RtclActionFactoryRegistry::TypedSignature<ProtoT>> factory);

  mutable absl::Mutex mutex_;
  absl::flat_hash_map<std::string, intrinsic_proto::icon::v1::ActionSignature>
      signature_registry_ ABSL_GUARDED_BY(mutex_);
};

// Access to the global registry.
RtclActionFactoryRegistry& GetGlobalRtclActionFactoryRegistry();

template <class ProtoT>
bool RtclActionFactoryRegistry::RegisterTyped(
    absl::string_view action_type_name,
    std::function<RtclActionFactoryRegistry::TypedSignature<ProtoT>> factory,
    const intrinsic_proto::icon::v1::ActionSignature& signature) {
  return RegisterGeneric(action_type_name,
                         WrapTypedFactory(action_type_name, std::move(factory)),
                         signature);
}

template <class ProtoT>
std::function<RtclActionFactoryRegistry::GenericSignature>
RtclActionFactoryRegistry::WrapTypedFactory(
    absl::string_view action_type_name,
    std::function<RtclActionFactoryRegistry::TypedSignature<ProtoT>> factory) {
  return [typed_factory = std::move(factory),
          action_type_name = std::string(action_type_name)](
             const google::protobuf::Any& params_any,
             ActionFactoryContext& context)
             -> absl::StatusOr<std::unique_ptr<RtclActionInterface>> {
    ProtoT params;
    if (!params_any.UnpackTo(&params)) {
      return absl::InvalidArgumentError(
          absl::StrCat("Factory for Action type '", action_type_name,
                       "'  expected '", ProtoT::GetDescriptor()->full_name(),
                       "', but received '", params_any.type_url(), "'."));
    }
    return typed_factory(params, context);
  };
}

}  // namespace intrinsic::icon

#endif  // INTRINSIC_ICON_CONTROL_RTCL_ACTION_FACTORY_REGISTRY_H_
