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

#include "intrinsic/world/component/simulation_component.h"

#include <memory>
#include <utility>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "intrinsic/world/proto/simulation_component.pb.h"

namespace intrinsic {

namespace {

class SimulationComponentImpl : public SimulationComponent {
 public:
  explicit SimulationComponentImpl(bool disabled, bool is_static)
      : disabled_(disabled), is_static_(is_static) {}

  absl::StatusOr<intrinsic_proto::world::SimulationComponent> ToProto()
      const override;
  std::unique_ptr<SimulationComponent> Clone() const override;
  bool IsDisabled() const override { return disabled_; }
  void SetDisabled(bool disabled) override { disabled_ = disabled; }
  bool IsStatic() const override { return is_static_; }
  void SetIsStatic(bool is_static) override { is_static_ = is_static; }

  absl::Status UpdateFromProto(
      const intrinsic_proto::world::SimulationComponent& proto) override;

 private:
  bool disabled_;
  bool is_static_;
};

absl::StatusOr<intrinsic_proto::world::SimulationComponent>
SimulationComponentImpl::ToProto() const {
  intrinsic_proto::world::SimulationComponent result;
  result.set_disabled(disabled_);
  result.set_is_static(is_static_);
  return result;
}

std::unique_ptr<SimulationComponent> SimulationComponentImpl::Clone() const {
  return std::make_unique<SimulationComponentImpl>(disabled_, is_static_);
}

absl::Status SimulationComponentImpl::UpdateFromProto(
    const intrinsic_proto::world::SimulationComponent& proto) {
  disabled_ = proto.disabled();
  is_static_ = proto.is_static();
  return absl::OkStatus();
}

}  // namespace

std::unique_ptr<SimulationComponent> SimulationComponent::Create() {
  return std::make_unique<SimulationComponentImpl>(false, false);
}

absl::StatusOr<std::unique_ptr<SimulationComponent>>
SimulationComponent::FromProto(
    const intrinsic_proto::world::SimulationComponent& proto) {
  return std::make_unique<SimulationComponentImpl>(proto.disabled(),
                                                   proto.is_static());
}

}  // namespace intrinsic
