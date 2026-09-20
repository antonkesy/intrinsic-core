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

#include "intrinsic/kinematics/chain.h"

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "intrinsic/kinematics/elements.h"
#include "intrinsic/kinematics/group.h"
#include "intrinsic/kinematics/model_interface.h"
#include "intrinsic/util/status/status_builder.h"
#include "intrinsic/util/status/status_macros.h"

namespace intrinsic {
namespace kinematics {

Chain::Chain(absl::string_view name) : Group(name) {}

absl::Status Chain::ExtractFromModel(const ModelInterface& model,
                                     const ElementId& base_id,
                                     const ElementId& tip_id) {
  return Group::ExtractFromModel(model, base_id, {tip_id});
}

ElementId Chain::GetTipId() const {
  if (GetTipIds().empty()) {
    return kInvalidElementId;
  }
  return GetTipIds().front();
}

absl::StatusOr<Chain> CreateChainFromModel(const ModelInterface& model) {
  const auto tip_ids = model.GetTipIds();
  if (!model.HasOneTip()) {
    return ::intrinsic::InvalidArgumentErrorBuilder()
           << "Model " << model.GetName() << " is not a chain. There is "
           << tip_ids.size()
           << " tips. Must specify which tip to use to create a chain.";
  }
  Chain new_chain(model.GetName());
  INTR_RETURN_IF_ERROR(
      new_chain.ExtractFromModel(model, model.GetBaseId(), tip_ids.front()));
  return new_chain;
}

absl::StatusOr<Chain> CreateChainFromModel(const ModelInterface& model,
                                           const ElementId& base_id,
                                           const ElementId& tip_id) {
  Chain new_chain(model.GetName());
  INTR_RETURN_IF_ERROR(
      new_chain.ExtractFromModel(model, model.GetBaseId(), tip_id));
  return new_chain;
}

absl::StatusOr<Chain> ExtractNonBranchingChain(const ModelInterface& model) {
  INTR_ASSIGN_OR_RETURN(ElementId tip,
                        model.FindNonBranchingKinematicChainTip());

  Chain new_chain(model.GetName());
  INTR_RETURN_IF_ERROR(
      new_chain.ExtractFromModel(model, model.GetBaseId(), tip));
  return new_chain;
}

}  // namespace kinematics
}  // namespace intrinsic
