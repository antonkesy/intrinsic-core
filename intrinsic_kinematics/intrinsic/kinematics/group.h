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

#ifndef INTRINSIC_KINEMATICS_GROUP_H_
#define INTRINSIC_KINEMATICS_GROUP_H_

#include "absl/container/flat_hash_map.h"
#include "absl/status/status.h"
#include "absl/strings/string_view.h"
#include "absl/types/span.h"
#include "intrinsic/icon/testing/realtime_annotations.h"
#include "intrinsic/icon/utils/realtime_status_or.h"
#include "intrinsic/kinematics/elements.h"
#include "intrinsic/kinematics/model_interface.h"

namespace intrinsic {
namespace kinematics {

// A Group is a connected subset of a model. The Group doesn't own its elements.
// The lifespan of the model that owns the elements used in the Group must
// outlive the Group. The Group can be seen as a "const view" of the underlying
// model.
//
// Normally, shared_ptr would be used to keep tract of the model, but these are
// not considered real-time safe. Therefore, special care need to be taken when
// using groups.
class Group : public ModelInterface {
 public:
  // Creates an empty kinematic group.
  explicit Group(absl::string_view name);

  // Creates a kinematic group from a base id and a set of tip_ids that are
  // connected to the base.
  absl::Status ExtractFromModel(
      const ModelInterface& model, const ElementId& base_id,
      absl::Span<const ElementId> tip_ids) INTRINSIC_NON_REALTIME_ONLY;

  // Creates a kinematic group from a set of connected tip_ids. The base will be
  // common ancestor to all tips.
  absl::Status ExtractFromModel(const ModelInterface& model,
                                absl::Span<const ElementId> tip_ids)
      INTRINSIC_NON_REALTIME_ONLY;

  icon::RealtimeStatusOr<int> GetDofIndexForElementId(
      const ElementId& id) const override;

  icon::RealtimeStatusOr<ElementId> GetElementIdForDofIndex(
      int dof_index) const override;

  icon::RealtimeStatusOr<const Element*> GetElement(
      const ElementId& id) const override;

  icon::RealtimeStatusOr<ElementId> GetElementId(
      const Element* element) const override;

 protected:
  void Clear();

  absl::Status AddElement(const ModelInterface& model,
                          const Element* new_element)
      INTRINSIC_NON_REALTIME_ONLY;

  // The group only refers to externally owned elements.
  absl::flat_hash_map<ElementId, const Element*> element_id_to_element_;
};

}  // namespace kinematics
}  // namespace intrinsic

#endif  // INTRINSIC_KINEMATICS_GROUP_H_
