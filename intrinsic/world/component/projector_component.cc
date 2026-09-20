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

#include "intrinsic/world/component/projector_component.h"

#include <cstdint>
#include <memory>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "intrinsic/util/status/status_macros.h"
#include "intrinsic/world/proto/projector_component.pb.h"

namespace intrinsic {

namespace {

class ProjectorComponentImpl : public ProjectorComponent {
 public:
  explicit ProjectorComponentImpl() = default;
  explicit ProjectorComponentImpl(double hfov, double near, double far,
                                  uint32_t visibility_flags,
                                  const Texture& texture)
      : hfov_(hfov),
        near_(near),
        far_(far),
        visibility_flags_(visibility_flags),
        texture_(texture) {}

  absl::StatusOr<intrinsic_proto::world::ProjectorComponent> ToProto()
      const override;
  std::unique_ptr<ProjectorComponent> Clone() const override;
  double GetHorizontalFov() const override { return hfov_; }
  void SetHorizontalFov(double hfov) override { hfov_ = hfov; }
  double GetNearClip() const override { return near_; }
  void SetNearClip(double near) override { near_ = near; }
  double GetFarClip() const override { return far_; }
  void SetFarClip(double far) override { far_ = far; }
  uint32_t GetVisibilityFlags() const override { return visibility_flags_; }
  void SetVisibilityFlags(uint32_t visibility_flags) override {
    visibility_flags_ = visibility_flags;
  }
  const Texture& GetTexture() const override { return texture_; }
  void SetTexture(const Texture& texture) override { texture_ = texture; }
  absl::Status UpdateFromProto(
      const intrinsic_proto::world::ProjectorComponent& proto) override;

 private:
  double hfov_ = 0.0;
  double near_ = 0.0;
  double far_ = 0.0;
  uint32_t visibility_flags_ = 0u;
  Texture texture_;
};

absl::StatusOr<intrinsic_proto::world::ProjectorComponent>
ProjectorComponentImpl::ToProto() const {
  intrinsic_proto::world::ProjectorComponent result;
  result.set_horizontal_fov(hfov_);
  result.mutable_clip()->set_near(near_);
  result.mutable_clip()->set_far(far_);
  result.set_visibility_flags(visibility_flags_);
  result.set_texture_data(texture_.data);
  result.set_texture_format(texture_.format);
  return result;
}

std::unique_ptr<ProjectorComponent> ProjectorComponentImpl::Clone() const {
  return std::make_unique<ProjectorComponentImpl>(hfov_, near_, far_,
                                                  visibility_flags_, texture_);
}

absl::Status ProjectorComponentImpl::UpdateFromProto(
    const intrinsic_proto::world::ProjectorComponent& proto) {
  INTR_ASSIGN_OR_RETURN(auto other_ptr, FromProto(proto));
  auto* other = dynamic_cast<ProjectorComponentImpl*>(other_ptr.get());

  hfov_ = other->hfov_;
  near_ = other->near_;
  far_ = other->far_;
  visibility_flags_ = other->visibility_flags_;
  texture_ = other->texture_;

  return absl::OkStatus();
}

}  // namespace

std::unique_ptr<ProjectorComponent> ProjectorComponent::Create() {
  return std::make_unique<ProjectorComponentImpl>();
}

absl::StatusOr<std::unique_ptr<ProjectorComponent>>
ProjectorComponent::FromProto(
    const intrinsic_proto::world::ProjectorComponent& proto) {
  double horizontal_fov = proto.horizontal_fov();
  double near = proto.clip().near();
  double far = proto.clip().far();
  uint32_t visibility_flags = proto.visibility_flags();
  Texture texture = {.data = proto.texture_data(),
                     .format = proto.texture_format()};
  return std::make_unique<ProjectorComponentImpl>(horizontal_fov, near, far,
                                                  visibility_flags, texture);
}

}  // namespace intrinsic
