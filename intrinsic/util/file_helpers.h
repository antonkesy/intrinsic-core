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

#ifndef INTRINSIC_UTIL_FILE_HELPERS_H_
#define INTRINSIC_UTIL_FILE_HELPERS_H_

#include <string>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "intrinsic/util/status/status_macros.h"
#include "ortools/base/helpers.h"
#include "ortools/base/options.h"

namespace intrinsic {

// A version of the file::SetTextProto that expands any protos.
absl::Status SetExpandedTextProto(absl::string_view filename,
                                  const google::protobuf::Message& proto,
                                  const file::Options& options);

// Creates all directories for the given path. Existing directories are
// ignored. Returns an error if the path exists but is not a directory.
absl::Status RecursivelyCreateDir(absl::string_view path);

// Deletes the given path and all its contents.
//
// Has the same behavior as `std::filesystem::remove_all`.
// Symlinks are not followed (symlink is removed, not its target).
// https://en.cppreference.com/w/cpp/filesystem/remove.html
absl::Status RecursivelyDelete(absl::string_view path);

// Same as `RecursivelyDelete`, but does not delete the directory itself.
// Returns an error if the path does not exist or is not a directory.
absl::Status RecursivelyDeleteDirectoryContents(absl::string_view dir_path);

template <typename ProtoType>
absl::StatusOr<ProtoType> ReadBinaryProto(absl::string_view filename) {
  std::string contents;
  INTR_RETURN_IF_ERROR(
      file::GetContents(filename, &contents, file::Defaults()));

  ProtoType proto;
  if (!proto.ParsePartialFromString(contents)) {
    return absl::InvalidArgumentError(
        absl::StrCat("Couldn't parse binary proto of format ",
                     proto.GetTypeName(), "from ", filename));
  }
  if (!proto.IsInitialized()) {
    return absl::InvalidArgumentError(absl::StrCat(
        "Couldn't parse binary proto of format ", proto.GetTypeName(), "from ",
        filename, ": ", proto.InitializationErrorString()));
  }
  return proto;
}

// Returns the depth of a filesystem path.
// Leading and trailing slashes are ignored.
// Paths are *not* resolved before computing the depth.
//
// For example:
// - `a/../b` has depth of 3.
// - "a/b/c" has depth 3
// - "/a" has depth 1.
int FilePathDepth(absl::string_view path);

absl::Status CopyFile(absl::string_view source_path,
                      absl::string_view target_path);

}  // namespace intrinsic

#endif  // INTRINSIC_UTIL_FILE_HELPERS_H_
