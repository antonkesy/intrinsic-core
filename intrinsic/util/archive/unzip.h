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

#ifndef INTRINSIC_UTIL_ARCHIVE_UNZIP_H_
#define INTRINSIC_UTIL_ARCHIVE_UNZIP_H_

#include "absl/status/status.h"
#include "absl/strings/string_view.h"

namespace intrinsic {

// Default depth to allow traversing into a zip archive.
constexpr int kMaxDirDepthDefault = 100;

// Unzips zipped_data to the destination directory. Creates the directory if
// needed. Returns an error if the unzip is not successful.
struct UnzipDataOptions {
  // If true, strips the common prefix directories that have no files in them
  // until a directory with files is encountered.
  // For example, if the zip contains the following files:
  //   foo/bar/baz/t.txt
  //   foo/baz/t.txt
  // Then the common prefix directory "foo" will be stripped, and the files will
  // be unzipped to:
  //   bar/baz/t.txt
  //   baz/t.txt"
  bool strip_common_prefix_directories = false;

  // Maximum allowed total uncompressed size of all files in the archive.
  // If 0, no limit is enforced.
  uint64_t max_uncompressed_size_bytes = 0;

  // Maximum allowed directory depth for any file in the archive.
  // If 0, no limit is enforced.
  // Not impacted by whether `strip_common_prefix_directories` is set or not.
  int max_directory_depth = kMaxDirDepthDefault;
};

// Returns OK if zipped data is successfully decompressed to the destination
// using the given options.
//
// For security purpose, paths in the zipped data are required to be not
// absolute, or contain `..`.
absl::Status UnzipData(absl::string_view zipped_data,
                       absl::string_view destination,
                       const UnzipDataOptions& options = {});
}  // namespace intrinsic

#endif  // INTRINSIC_UTIL_ARCHIVE_UNZIP_H_
