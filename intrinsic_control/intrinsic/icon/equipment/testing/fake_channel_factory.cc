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

#include "intrinsic/icon/equipment/testing/fake_channel_factory.h"

#include <memory>

#include "absl/log/check.h"
#include "absl/status/statusor.h"
#include "absl/time/time.h"
#include "intrinsic/icon/cc_client/testing/channel_fake.h"
#include "intrinsic/icon/cc_client/testing/rtcl_controller_channel_fake.h"
#include "intrinsic/util/grpc/channel_interface.h"
#include "intrinsic/util/grpc/connection_params.h"

namespace intrinsic {
namespace icon {

absl::StatusOr<std::shared_ptr<ChannelInterface>>
FakeChannelFactory::MakeChannel(const ConnectionParams& params,
                                absl::Duration) const {
  RtclControllerChannelFake::Builder builder;
  if (!params.instance_name.empty()) {
    builder.WithServerName(params.instance_name);
  }
  return builder.Build(DefaultPartConfigs());
}

absl::StatusOr<std::shared_ptr<ChannelInterface>>
FakeChannelPassThroughFactory::MakeChannel(const ConnectionParams&,
                                           absl::Duration) const {
  CHECK(channel_ != nullptr) << "Channel is not set!";
  std::shared_ptr<ChannelInterface> result(channel_);
  return result;
}

}  // namespace icon
}  // namespace intrinsic
