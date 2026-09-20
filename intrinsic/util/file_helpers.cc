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

#include "intrinsic/util/file_helpers.h"

#include <algorithm>
#include <filesystem>  // NOLINT
#include <string>
#include <system_error>  // NOLINT

#include "absl/status/status.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/string_view.h"
#include "google/protobuf/text_format.h"
#include "intrinsic/util/status/status_macros.h"
#include "ortools/base/helpers.h"
#include "ortools/base/options.h"

namespace intrinsic {

absl::Status SetExpandedTextProto(absl::string_view filename,
                                  const google::protobuf::Message& proto,
                                  const file::Options& options) {
  std::string text_proto;
  if (!proto.IsInitialized()) {
    return absl::Status(
        absl::StatusCode::kFailedPrecondition,
        absl::StrCat("Cannot serialize proto, missing required field ",
                     proto.InitializationErrorString()));
  }

  google::protobuf::TextFormat::Printer printer;
  printer.SetExpandAny(true);

  if (!printer.PrintToString(proto, &text_proto)) {
    return absl::Status(
        absl::StatusCode::kFailedPrecondition,
        absl::StrCat(
            "Failed to convert proto to text for saving to ", filename,
            " (this generally stems from massive protobufs that either "
            "exhaust memory or overflow a 32-bit buffer somewhere)."));
  }
  return file::SetContents(filename, text_proto, options);
}

absl::Status RecursivelyCreateDir(absl::string_view path) {
  std::error_code ec;
  bool path_exists = std::filesystem::exists(path, ec);
  if (ec) {
    return absl::InternalError(
        absl::StrCat("Error checking the existence of path: ", path,
                     " with error: ", ec.message()));
  }
  if (path_exists) {
    bool path_is_directory = std::filesystem::is_directory(path, ec);
    if (ec) {
      return absl::InternalError(
          absl::StrCat("Error checking if path: ", path,
                       " is a directory, with error: ", ec.message()));
    }
    if (!path_is_directory) {
      return absl::AlreadyExistsError(
          absl::StrCat("Path exists but is not a directory: ", path));
    }
    return absl::OkStatus();
  }
  if (!std::filesystem::create_directories(path, ec)) {
    return absl::InternalError(absl::StrCat(
        "Failed to create directory: ", path, " with error: ", ec.message()));
  }
  return absl::OkStatus();
}

absl::Status RecursivelyDelete(absl::string_view path) {
  std::error_code ec;

  if (!std::filesystem::remove_all(path, ec)) {
    return absl::NotFoundError(absl::StrCat("Failed to delete path: ", path));
  }

  if (ec) {
    return absl::InternalError(absl::StrCat("Failed to delete path: ", path,
                                            " with error: ", ec.message()));
  }

  return absl::OkStatus();
}

absl::Status RecursivelyDeleteDirectoryContents(
    const absl::string_view dir_path) {
  std::error_code ec;
  if (!std::filesystem::exists(dir_path, ec)) {
    return absl::FailedPreconditionError(
        absl::StrCat("Path does not exist: ", dir_path));
  }

  if (!std::filesystem::is_directory(dir_path, ec)) {
    return absl::FailedPreconditionError(
        absl::StrCat("Path is not a directory: ", dir_path));
  }

  for (const auto& entry : std::filesystem::directory_iterator(dir_path, ec)) {
    if (ec) {
      return absl::InternalError(
          absl::StrCat("Failed to iterate over directory: ", dir_path));
    }
    INTR_RETURN_IF_ERROR(RecursivelyDelete(entry.path().string()));
  }

  return absl::OkStatus();
}

int FilePathDepth(absl::string_view path) {
  std::filesystem::path fspath(path);

  // Uses `relative_path()` to strip any leading slashes. Otherwise `/path`
  // would return depth of 2.
  return std::ranges::count_if(fspath.relative_path(),
                               [](const std::filesystem::path& part) {
                                 // Filters out empty or `.` parts since they
                                 // don't represent actual path.
                                 return !part.empty() && part != ".";
                               });
}

absl::Status CopyFile(absl::string_view source_path,
                      absl::string_view target_path) {
  std::error_code ec;
  if (!std::filesystem::copy_file(source_path, target_path, ec) || ec) {
    return absl::InternalError(
        absl::StrCat("Failed to copy file: ", source_path, " to ", target_path,
                     ": ", ec.message()));
  }
  return absl::OkStatus();
}

}  // namespace intrinsic
