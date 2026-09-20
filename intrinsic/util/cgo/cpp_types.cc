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

#include "intrinsic/util/cgo/cpp_types.h"

#include <cstddef>
#include <cstdint>
#include <string>

#include "absl/log/log.h"
#include "absl/status/status.h"
#include "absl/strings/string_view.h"
#include "google/rpc/status.pb.h"
#include "intrinsic/util/cgo/c_types.h"

namespace intrinsic {

void WriteStatus(go_c_Handle status, const char* data, size_t length) {
  absl::string_view s(data, length);
  auto* status_ptr = reinterpret_cast<absl::Status*>(status);
  google::rpc::Status status_proto;
  if (!status_proto.ParseFromString(s)) {
    LOG(FATAL) << "Failed to parse StatusProto";
  }
  *status_ptr = absl::Status(static_cast<absl::StatusCode>(status_proto.code()),
                             status_proto.message());
}

go_c_StringIn ToStringIn(absl::string_view s) {
  return go_c_StringIn{reinterpret_cast<const uint8_t*>(s.data()), s.size()};
}

go_c_SliceIn ToSliceIn(const std::string& arr) {
  return {const_cast<char*>(arr.data()), arr.size(), GO_C_ELEMENT_TYPE_UINT8};
}

void ParseFromString(go_c_Handle message, const char* data, size_t length) {
  absl::string_view s(data, length);
  auto* msg = reinterpret_cast<google::protobuf::Message*>(message);
  if (!msg->ParseFromString(s)) {
    LOG(FATAL) << "Failed to parse proto";
  }
}

}  // namespace intrinsic
