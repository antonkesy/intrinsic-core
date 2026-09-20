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

#ifndef INTRINSIC_ICON_EQUIPMENT_TESTING_FAKE_CHANNEL_FACTORY_H_
#define INTRINSIC_ICON_EQUIPMENT_TESTING_FAKE_CHANNEL_FACTORY_H_

#include <memory>
#include <utility>

#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "absl/time/time.h"
#include "intrinsic/icon/equipment/channel_factory.h"
#include "intrinsic/util/grpc/channel_interface.h"
#include "intrinsic/util/grpc/connection_params.h"

namespace intrinsic {
namespace icon {

// FakeChannelFactory creates a ChannelFake, ignoring the gRPC address.
//
// If `params.instance_name` is non-empty, the resulting ChannelFake sets its
// server name to that name.
// Otherwise, the server name defaults to
// `RtclControllerChannelFake::kDefaultServerName`.
class FakeChannelFactory : public ChannelFactory {
 public:
  absl::StatusOr<std::shared_ptr<ChannelInterface>> MakeChannel(
      const ConnectionParams& params, absl::Duration timeout) const override;
};

class FakeChannelPassThroughFactory : public ChannelFactory {
 public:
  explicit FakeChannelPassThroughFactory(
      std::shared_ptr<ChannelInterface> channel)
      : channel_(std::move(channel)) {}

  absl::StatusOr<std::shared_ptr<ChannelInterface>> MakeChannel(
      const ConnectionParams& params, absl::Duration timeout) const override;

 private:
  mutable std::shared_ptr<ChannelInterface> channel_;
};

}  // namespace icon
}  // namespace intrinsic

#endif  // INTRINSIC_ICON_EQUIPMENT_TESTING_FAKE_CHANNEL_FACTORY_H_
