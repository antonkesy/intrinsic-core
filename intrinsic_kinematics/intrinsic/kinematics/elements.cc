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

#include "intrinsic/kinematics/elements.h"

#include <optional>

#include "absl/algorithm/container.h"
#include "absl/log/check.h"
#include "absl/status/status.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/string_view.h"
#include "absl/types/span.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/icon/utils/realtime_status_or.h"
#include "intrinsic/math/pose3.h"

namespace intrinsic {
namespace kinematics {

Element::Element(absl::string_view name)
    : parent_t_this_(Pose3d::Identity()), name_(name) {}

Element::Element(absl::string_view name, const Element* parent,
                 const Pose3d& parent_t_this, bool is_static_frame)
    : parent_(parent), parent_t_this_(parent_t_this), name_(name) {
  CHECK_NE(parent_, nullptr);
}

absl::string_view Element::GetName() const { return name_; }

Pose3d Element::GetParentTThis() const { return parent_t_this_; }

std::optional<const Element*> Element::GetParentElement() const {
  if (parent_ == nullptr) {
    return std::nullopt;
  }
  return parent_;
}

absl::Status Element::AddChildElement(const Element* child) {
  if (child == nullptr) {
    return absl::InvalidArgumentError("Child is nullptr.");
  }
  if (absl::c_find(children_, child) != children_.end()) {
    return absl::InvalidArgumentError("Child was already part of children.");
  }
  children_.push_back(child);
  return absl::OkStatus();
}

absl::Status Element::RemoveChildElement(const Element* child) {
  if (child == nullptr) {
    return absl::InvalidArgumentError("Cannot remove nullptr");
  }

  for (int i = 0; i < children_.size(); ++i) {
    if (child == children_[i]) {
      children_.erase(children_.begin() + i);
      return absl::OkStatus();
    }
  }
  return absl::NotFoundError(
      absl::StrCat("Element '", child->GetName(), "' is not a child of '",
                   this->GetName(), "', cannot remove."));
}

icon::RealtimeStatus Element::SetParentElement(const Element* parent,
                                               const Pose3d& parent_t_this) {
  if (parent == nullptr) {
    return icon::InvalidArgumentError("No valid parent element provided.");
  }
  parent_ = parent;
  parent_t_this_ = parent_t_this;

  return icon::OkStatus();
}

void Element::RemoveParent() {
  parent_ = nullptr;
  parent_t_this_ = Pose3d::Identity();
}

absl::Span<const Element* const> Element::GetChildElements() const {
  return children_;
}

int Element::GetNumberOfChildren() const { return children_.size(); }

icon::RealtimeStatusOr<const Element*> Element::GetChildByIndex(
    int index) const {
  if (index >= children_.size()) {
    return icon::InvalidArgumentError(
        "No valid child element provided: Child index out of bounds.");
  }
  return children_[index];
}

}  // namespace kinematics

}  // namespace intrinsic
