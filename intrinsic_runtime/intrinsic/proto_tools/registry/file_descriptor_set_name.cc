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

#include "intrinsic_runtime/intrinsic/proto_tools/registry/file_descriptor_set_name.h"

#include <string>
#include <string_view>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/escaping.h"
#include "absl/strings/str_format.h"
#include "intrinsic/proto_tools/proto/proto_registry_internal.pb.h"

namespace intrinsic::proto_registry {

// Obfuscates the given type URL prefix following the scheme proposed by
// go/aip/158 for page tokens: store content in an internal proto and return the
// serialized proto, base-64 encoded.
absl::StatusOr<FileDescriptorSetName> FileDescriptorSetNameForTypeUrlPrefix(
    std::string_view type_url_prefix) {
  intrinsic_proto::proto_registry::internal::FileDescriptorSetName msg;
  msg.set_type_url_prefix(type_url_prefix);

  std::string msg_bytes;
  if (!msg.SerializeToString(&msg_bytes)) {
    return absl::InvalidArgumentError(absl::StrFormat(
        "cannot serialize FileDescriptorSetName proto: %s", msg.DebugString()));
  }

  return FileDescriptorSetName(absl::WebSafeBase64Escape(msg_bytes));
}

absl::StatusOr<std::string> TypeUrlPrefixForFileDescriptorSetName(
    const FileDescriptorSetName& name) {
  std::string msg_bytes;
  if (!absl::WebSafeBase64Unescape(name.value(), &msg_bytes)) {
    return absl::InvalidArgumentError(absl::StrFormat(
        "cannot decode file descriptor set name: %s", name.value()));
  }

  intrinsic_proto::proto_registry::internal::FileDescriptorSetName msg;
  if (!msg.ParseFromString(msg_bytes)) {
    return absl::InvalidArgumentError(
        absl::StrFormat("invalid file descriptor set name: %s", name.value()));
  }

  return msg.type_url_prefix();
}

}  // namespace intrinsic::proto_registry
