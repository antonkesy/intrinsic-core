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

#ifndef INTRINSIC_ICON_CONTROL_REALTIME_SLOT_MAP_H_
#define INTRINSIC_ICON_CONTROL_REALTIME_SLOT_MAP_H_

#include <functional>
#include <tuple>
#include <type_traits>

#include "intrinsic/icon/control/parts/feature_interface_registry.h"
#include "intrinsic/icon/control/slot_types.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/icon/utils/realtime_status_or.h"

namespace intrinsic::icon {

class RealtimeSlotMapInterface {
 public:
  virtual ~RealtimeSlotMapInterface() = default;

  // Both of these return nullptr if there is no slot with `slot_id`.
  virtual FeatureInterfaceRegistry* GetMutableRegistryForSlot(
      RealtimeSlotId slot_id) = 0;
  virtual const FeatureInterfaceRegistry* GetRegistryForSlot(
      RealtimeSlotId slot_id) const = 0;
};

// RtclActions use this to access the slots that are assigned to them. Action
// Factories must save RealtimeSlotIds for the Slots they care about, and pass
// them to RtclAction instances.
//
// Uses RealtimeSlotMapInterface for easier mocking/testing.
class RealtimeSlotMap {
 public:
  explicit RealtimeSlotMap(RealtimeSlotMapInterface& slot_map)
      : slot_map_(slot_map) {}

  // Returns a mutable pointer to the requested FeatureInterface for `slot_id`.
  // Returns nullptr if `slot_id` is invalid, or if the slot it points to does
  // not have `FeatureInterfaceT`.
  template <typename FeatureInterfaceT>
  FeatureInterfaceT* GetMutableInterfaceForSlot(RealtimeSlotId slot_id);

  // Returns a const pointer to the requested FeatureInterface for `slot_id`.
  // Returns nullptr if `slot_id` is invalid, or if the slot it points to does
  // not have `FeatureInterfaceT`.
  template <typename FeatureInterfaceT>
  const FeatureInterfaceT* GetInterfaceForSlot(RealtimeSlotId slot_id) const;

  // Returns a tuple of const pointers to the requested `FeatureInterfaceTs` for
  // `slot_id`.
  //
  // Returns NotFoundError if `slot_id` is invalid, or if the slot it points to
  // is missing one or more of `FeatureInterfaceTs`.
  template <typename... FeatureInterfaceTs>
  RealtimeStatusOr<
      std::tuple<std::add_pointer_t<std::add_const_t<FeatureInterfaceTs>>...>>
  GetInterfacesForSlot(RealtimeSlotId slot_id) const;

  // Returns a tuple of mutable pointers to the requested `FeatureInterfaceTs`
  // for `slot_id`.
  //
  // Returns NotFoundError if `slot_id` is invalid, or if the slot it points to
  // is missing one or more of `FeatureInterfaceTs`.
  template <typename... FeatureInterfaceTs>
  RealtimeStatusOr<std::tuple<std::add_pointer_t<FeatureInterfaceTs>...>>
  GetMutableInterfacesForSlot(RealtimeSlotId slot_id);

 private:
  RealtimeSlotMapInterface& slot_map_;
};

template <typename FeatureInterfaceT>
FeatureInterfaceT* RealtimeSlotMap::GetMutableInterfaceForSlot(
    RealtimeSlotId slot_id) {
  if (FeatureInterfaceRegistry* registry =
          slot_map_.GetMutableRegistryForSlot(slot_id);
      registry == nullptr) {
    return nullptr;
  } else {
    return registry->GetInterface<FeatureInterfaceT>();
  }
}

template <typename FeatureInterfaceT>
const FeatureInterfaceT* RealtimeSlotMap::GetInterfaceForSlot(
    RealtimeSlotId slot_id) const {
  if (const FeatureInterfaceRegistry* registry =
          slot_map_.GetRegistryForSlot(slot_id);
      registry == nullptr) {
    return nullptr;
  } else {
    return registry->GetInterface<FeatureInterfaceT>();
  }
}

template <typename... FeatureInterfaceTs>
RealtimeStatusOr<
    std::tuple<std::add_pointer_t<std::add_const_t<FeatureInterfaceTs>>...>>
RealtimeSlotMap::GetInterfacesForSlot(RealtimeSlotId slot_id) const {
  std::tuple<std::add_pointer_t<std::add_const_t<FeatureInterfaceTs>>...>
      pointers{GetInterfaceForSlot<FeatureInterfaceTs>(slot_id)...};
  bool any_nullptr = std::apply(
      [](auto... xs) {
        return ([](auto* x) { return x == nullptr; }(xs) || ...);
      },
      pointers);
  if (any_nullptr) {
    return NotFoundError(
        "One or more of the requested feature interfaces are unavailable");
  }
  return std::move(pointers);
}

template <typename... FeatureInterfaceTs>
RealtimeStatusOr<std::tuple<std::add_pointer_t<FeatureInterfaceTs>...>>
RealtimeSlotMap::GetMutableInterfacesForSlot(RealtimeSlotId slot_id) {
  std::tuple<std::add_pointer_t<FeatureInterfaceTs>...> pointers{
      GetMutableInterfaceForSlot<FeatureInterfaceTs>(slot_id)...};
  bool any_nullptr = std::apply(
      [](auto... xs) {
        return ([](auto* x) { return x == nullptr; }(xs) || ...);
      },
      pointers);
  if (any_nullptr) {
    return NotFoundError(
        "One or more of the requested feature interfaces are unavailable");
  }
  return std::move(pointers);
}

}  // namespace intrinsic::icon

#endif  // INTRINSIC_ICON_CONTROL_REALTIME_SLOT_MAP_H_
