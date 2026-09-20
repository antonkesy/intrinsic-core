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

// Converts a text proto to a binary proto using a given file descriptor set.
//
// Note that protoc can do this natively but it does not have customizations
// such as the correct handling of Intrinsic-style type URLs in Any fields.

#include <unistd.h>

#include <cstdlib>
#include <string>

#include "absl/flags/flag.h"
#include "absl/log/log.h"
#include "absl/status/status.h"
#include "google/protobuf/descriptor.pb.h"
#include "intrinsic/icon/release/portable/init_intrinsic.h"
#include "intrinsic/util/proto/dynamic_message_parser.h"
#include "intrinsic/util/status/return.h"
#include "intrinsic/util/status/status_macros.h"
#include "ortools/base/helpers.h"
#include "ortools/base/options.h"

ABSL_FLAG(
    std::string, message_full_name, "",
    "Full name of the message type to parse, e.g. 'intrinsic_proto.Pose3d'");
ABSL_FLAG(std::string, in_text_proto, "",
          "Name of the input file containing the text proto to parse.");
ABSL_FLAG(std::string, out_binary_proto, "",
          "Name of the output file to which the binary proto will be written.");
ABSL_FLAG(std::string, transitive_descriptor_set, "",
          "Name of the file containing a binary-encoded file descriptor set "
          "with all dependencies of the message type to parse.");

namespace intrinsic {
namespace {

absl::Status Run() {
  if (absl::GetFlag(FLAGS_message_full_name).empty()) {
    return absl::InvalidArgumentError("--message_full_name required");
  }
  if (absl::GetFlag(FLAGS_in_text_proto).empty()) {
    return absl::InvalidArgumentError("--in_text_proto required");
  }
  if (absl::GetFlag(FLAGS_out_binary_proto).empty()) {
    return absl::InvalidArgumentError("--out_binary_proto required");
  }
  if (absl::GetFlag(FLAGS_transitive_descriptor_set).empty()) {
    return absl::InvalidArgumentError(
        "--transitive_descriptor_set_file required");
  }

  INTR_ASSIGN_OR_RETURN(
      std::string contents,
      file::GetContents(absl::GetFlag(FLAGS_in_text_proto), file::Defaults()));

  google::protobuf::FileDescriptorSet fds;
  INTR_RETURN_IF_ERROR(file::GetBinaryProto(
      absl::GetFlag(FLAGS_transitive_descriptor_set), &fds, file::Defaults()));

  INTR_ASSIGN_OR_RETURN(
      MessageAndDeps parsed_message,
      DynamicMessageParser::ParseSingleTextProto(
          fds, absl::GetFlag(FLAGS_message_full_name), contents));

  INTR_RETURN_IF_ERROR(
      file::SetBinaryProto(absl::GetFlag(FLAGS_out_binary_proto),
                           *parsed_message.message, file::Defaults()));

  return absl::OkStatus();
}

}  // namespace
}  // namespace intrinsic

int main(int argc, char** argv) {
  InitIntrinsic(argv[0], argc, argv);
  INTR_RETURN_IF_ERROR(intrinsic::Run())
      .LogError()
      .With(intrinsic::Return(EXIT_FAILURE));
  return EXIT_SUCCESS;
}
