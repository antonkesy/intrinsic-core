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

#include "intrinsic/world/component/user_data_component.h"

#include <memory>
#include <string>
#include <utility>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "google/protobuf/any.pb.h"
#include "intrinsic/util/status/status_macros.h"
#include "intrinsic/world/hashing/hashing.h"
#include "intrinsic/world/proto/user_data_component.pb.h"

namespace intrinsic {

namespace {

class UserDataComponentImpl : public UserDataComponent {
 public:
  explicit UserDataComponentImpl(
      WorldHashMap<std::string, std::string> user_data,
      WorldHashMap<std::string, google::protobuf::Any> user_data_protos)
      : user_data_(std::move(user_data)),
        user_data_protos_(std::move(user_data_protos)) {}

  absl::StatusOr<intrinsic_proto::world::UserDataComponent> ToProto()
      const override;
  std::unique_ptr<UserDataComponent> Clone() const override;
  WorldHashMap<std::string, std::string>& MutableUserDataMap() override;
  const WorldHashMap<std::string, std::string>& UserDataMap() const override;

  WorldHashMap<std::string, google::protobuf::Any>& MutableUserDataProtos()
      override;

  const WorldHashMap<std::string, google::protobuf::Any>& UserDataProtos()
      const override;

  absl::Status UpdateFromProto(
      const intrinsic_proto::world::UserDataComponent& proto) override;

 private:
  WorldHashMap<std::string, std::string> user_data_;
  WorldHashMap<std::string, google::protobuf::Any> user_data_protos_;
};

absl::StatusOr<intrinsic_proto::world::UserDataComponent>
UserDataComponentImpl::ToProto() const {
  intrinsic_proto::world::UserDataComponent result;
  for (const auto& [key, value] : user_data_) {
    result.mutable_user_data_map()->insert({key, value});
  }
  for (const auto& [key, value] : user_data_protos_) {
    result.mutable_user_data_protos()->insert({key, value});
  }
  return std::move(result);
}

std::unique_ptr<UserDataComponent> UserDataComponentImpl::Clone() const {
  return std::make_unique<UserDataComponentImpl>(user_data_, user_data_protos_);
}

WorldHashMap<std::string, std::string>&
UserDataComponentImpl::MutableUserDataMap() {
  return user_data_;
}

const WorldHashMap<std::string, std::string>&
UserDataComponentImpl::UserDataMap() const {
  return user_data_;
}

WorldHashMap<std::string, google::protobuf::Any>&
UserDataComponentImpl::MutableUserDataProtos() {
  return user_data_protos_;
}

const WorldHashMap<std::string, google::protobuf::Any>&
UserDataComponentImpl::UserDataProtos() const {
  return user_data_protos_;
}

absl::StatusOr<WorldHashMap<std::string, std::string>> ParseUserData(
    const intrinsic_proto::world::UserDataComponent& proto) {
  WorldHashMap<std::string, std::string> user_data;
  user_data.reserve(proto.user_data_map_size());
  for (const auto& kv : proto.user_data_map()) {
    user_data.emplace(kv.first, kv.second);
  }

  return std::move(user_data);
}

absl::StatusOr<WorldHashMap<std::string, google::protobuf::Any>>
ParseUserDataProtos(const intrinsic_proto::world::UserDataComponent& proto) {
  WorldHashMap<std::string, google::protobuf::Any> user_data_protos;
  user_data_protos.reserve(proto.user_data_protos_size());
  for (const auto& kv : proto.user_data_protos()) {
    user_data_protos.emplace(kv.first, kv.second);
  }
  return user_data_protos;
}

absl::Status UserDataComponentImpl::UpdateFromProto(
    const intrinsic_proto::world::UserDataComponent& proto) {
  INTR_ASSIGN_OR_RETURN(user_data_, ParseUserData(proto));
  INTR_ASSIGN_OR_RETURN(user_data_protos_, ParseUserDataProtos(proto));
  return absl::OkStatus();
}

}  // namespace

std::unique_ptr<UserDataComponent> UserDataComponent::Create() {
  return std::make_unique<UserDataComponentImpl>(
      WorldHashMap<std::string, std::string>(),
      WorldHashMap<std::string, google::protobuf::Any>());
}

absl::StatusOr<std::unique_ptr<UserDataComponent>> UserDataComponent::FromProto(
    const intrinsic_proto::world::UserDataComponent& proto) {
  INTR_ASSIGN_OR_RETURN(auto user_data, ParseUserData(proto));
  INTR_ASSIGN_OR_RETURN(auto user_data_protos, ParseUserDataProtos(proto));
  return std::make_unique<UserDataComponentImpl>(std::move(user_data),
                                                 std::move(user_data_protos));
}

}  // namespace intrinsic
