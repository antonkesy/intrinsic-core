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

#include "intrinsic/world/component/ppr_component.h"

#include <memory>
#include <optional>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "intrinsic/world/proto/ppr_component.pb.h"

namespace intrinsic {

namespace {

class PPRComponentImpl : public PPRComponent {
 public:
  explicit PPRComponentImpl(const intrinsic_proto::world::PPRComponent& proto)
      : proto_(proto) {}

  absl::StatusOr<intrinsic_proto::world::PPRComponent> ToProto() const override;
  std::unique_ptr<PPRComponent> Clone() const override;

  std::optional<absl::string_view> ResourceName() const override;

  void SetResourceName(absl::string_view resource_name) override;

  absl::Status UpdateFromProto(
      const intrinsic_proto::world::PPRComponent& proto) override;

 private:
  intrinsic_proto::world::PPRComponent proto_;
};

absl::StatusOr<intrinsic_proto::world::PPRComponent> PPRComponentImpl::ToProto()
    const {
  return proto_;
}

std::unique_ptr<PPRComponent> PPRComponentImpl::Clone() const {
  return std::make_unique<PPRComponentImpl>(proto_);
}

std::optional<absl::string_view> PPRComponentImpl::ResourceName() const {
  if (proto_.has_resource_name()) {
    return proto_.resource_name();
  }
  return std::nullopt;
}

void PPRComponentImpl::SetResourceName(absl::string_view resource_name) {
  proto_.set_resource_name(resource_name);
}

absl::Status PPRComponentImpl::UpdateFromProto(
    const intrinsic_proto::world::PPRComponent& proto) {
  proto_ = proto;
  return absl::OkStatus();
}

}  // namespace

std::unique_ptr<PPRComponent> PPRComponent::Create() {
  return std::make_unique<PPRComponentImpl>(
      intrinsic_proto::world::PPRComponent{});
}

absl::StatusOr<std::unique_ptr<PPRComponent>> PPRComponent::FromProto(
    const intrinsic_proto::world::PPRComponent& proto) {
  return std::make_unique<PPRComponentImpl>(proto);
}

}  // namespace intrinsic
