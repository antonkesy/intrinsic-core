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

#include "intrinsic/util/archive/unzip.h"

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

#include "absl/algorithm/container.h"
#include "absl/cleanup/cleanup.h"
#include "absl/container/flat_hash_set.h"
#include "absl/log/log.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/match.h"
#include "absl/strings/str_split.h"
#include "absl/strings/string_view.h"
#include "absl/strings/strip.h"
#include "absl/strings/substitute.h"
#include "intrinsic/util/file_helpers.h"
#include "intrinsic/util/status/status_macros.h"
#include "ortools/base/filesystem.h"
#include "ortools/base/helpers.h"
#include "ortools/base/options.h"
#include "ortools/base/path.h"
#include "zip.h"
#include "zipconf.h"

namespace intrinsic {

namespace {

// Returns true if path is safe for unzip.
// Path should not be absolute or allow change of directory. Otherwise, the
// unzipped path may leak out of the target unzip directory and overwrite other
// paths unintentionally.
bool IsPathSafeToUnzip(absl::string_view path_str) {
  if (path_str.empty()) {
    return false;
  }

  std::filesystem::path path(path_str);
  return !path.is_absolute() &&
         !std::ranges::any_of(path,
                              [](const auto& part) { return part == ".."; });
}

absl::StatusOr<std::string> GetGreatestCommonPrefixPath(zip_t& zip_archive) {
  std::string skip_prefix;

  absl::flat_hash_set<std::string> seen_files;
  absl::flat_hash_set<std::string> seen_directories;
  const int num_entries = zip_get_num_entries(&zip_archive, 0);
  for (int i = 0; i < num_entries; ++i) {
    zip_stat_t sb;
    zip_stat_init(&sb);
    if (zip_stat_index(&zip_archive, i, 0, &sb) != 0) {
      return absl::InvalidArgumentError(
          absl::Substitute("zip stat failed at index $0", i));
    }
    if (sb.name == nullptr || sb.name[0] == '\0') {
      continue;
    }
    std::string name(sb.name);
    if (name.back() != '/') {
      seen_files.emplace(name);
    }
    seen_directories.emplace(file::Dirname(name));
  }
  for (const auto& dir : seen_directories) {
    bool all_files_in_dir = absl::c_all_of(
        seen_files,
        [&dir](const auto& file) { return absl::StartsWith(file, dir); });
    bool extend_skip_prefix =
        dir != skip_prefix && absl::StartsWith(dir, skip_prefix);
    if (all_files_in_dir && extend_skip_prefix) {
      skip_prefix = dir;
    }
  }
  return skip_prefix;
}

// Returns OK if decompressing the archive succeeded.
//
// Returns DataLossError if decompressed size does not match the expected size.
// This guards against malformed zip archives that can result in out-of-memory
// errors.
absl::Status UnzipToPath(zip_file_t* source_file,
                         const std::string& destination_path,
                         const uint64_t expected_size) {
  std::ofstream out(destination_path, std::ios::binary);
  if (!out) {
    return absl::InternalError(absl::StrCat(
        "Failed to open destination file for writing: ", destination_path));
  }

  // Reads the file in chunks (streaming) rather than reading the entire
  // expected size in one go. This keeps memory usage constant and low,
  // preventing Out-Of-Memory (OOM) errors when decompressing large files. 4MB
  // is a good balance between memory footprint and I/O throughput.
  constexpr size_t kChunkSize = 4 * 1024 * 1024;  // 4MB
  std::vector<char> buffer(kChunkSize);
  uint64_t total_read = 0;

  while (total_read < expected_size) {
    size_t to_read =
        std::min(static_cast<uint64_t>(kChunkSize), expected_size - total_read);
    zip_int64_t read_bytes = zip_fread(source_file, buffer.data(), to_read);
    if (read_bytes < 0) {
      return absl::DataLossError("Error reading from zip file");
    }
    if (read_bytes == 0) {
      break;  // EOF
    }
    out.write(buffer.data(), read_bytes);
    if (!out) {
      return absl::InternalError("Error writing to destination file");
    }
    total_read += read_bytes;
  }

  if (total_read == expected_size) {
    char dummy;
    zip_int64_t extra = zip_fread(source_file, &dummy, 1);
    if (extra > 0) {
      return absl::DataLossError(absl::Substitute(
          "File in archive is larger than expected $0 bytes", expected_size));
    }
  } else {
    return absl::DataLossError(absl::Substitute(
        "Failed to read expected bytes. Wanted $0 bytes, got $1 bytes",
        expected_size, total_read));
  }
  return absl::OkStatus();
}

absl::Status UnzipArchive(zip_t& zip_archive, absl::string_view destination,
                          const UnzipDataOptions& options) {
  const int num_entries = zip_get_num_entries(&zip_archive, 0);
  std::string skip_prefix;
  if (options.strip_common_prefix_directories) {
    INTR_ASSIGN_OR_RETURN(skip_prefix,
                          GetGreatestCommonPrefixPath(zip_archive));
  }

  // Sanity checks before attempting to unzip the archive.
  uint64_t total_uncompressed_size = 0;
  for (int i = 0; i < num_entries; ++i) {
    zip_stat_t sb;
    zip_stat_init(&sb);
    if (zip_stat_index(&zip_archive, i, 0, &sb) != 0) {
      return absl::InvalidArgumentError(
          absl::Substitute("zip stat failed at index $0", i));
    }

    if (sb.name == nullptr || sb.name[0] == '\0') {
      return absl::InvalidArgumentError("Empty file name in archive");
    }

    // Validates that the path is safe before doing anything.
    if (!IsPathSafeToUnzip(sb.name)) {
      return absl::InvalidArgumentError(
          absl::Substitute("Invalid path detected in archive: $0. Paths should "
                           "not be absolute or contain `..`.",
                           sb.name));
    }

    if (options.max_uncompressed_size_bytes > 0) {
      total_uncompressed_size += sb.size;
      if (total_uncompressed_size > options.max_uncompressed_size_bytes) {
        return absl::FailedPreconditionError(absl::Substitute(
            "Decompressed archive exceeds maximum allowed size of $0 bytes",
            options.max_uncompressed_size_bytes));
      }
    }
    if (options.max_directory_depth > 0) {
      absl::string_view dir_path = sb.name;
      // `sb.name` contains full path including `/` for a directory. So strips
      // the filename to correctly compute the directory depth.
      if (dir_path.back() != '/') {
        dir_path = file::SplitPath(dir_path).first;
      }
      const int depth = FilePathDepth(dir_path);
      if (depth > options.max_directory_depth) {
        return absl::FailedPreconditionError(absl::Substitute(
            "Directory depth $0 exceeds maximum allowed depth of $1", depth,
            options.max_directory_depth));
      }
    }
  }

  for (int i = 0; i < num_entries; ++i) {
    zip_stat_t sb;
    zip_stat_init(&sb);
    if (zip_stat_index(&zip_archive, i, 0, &sb) != 0) {
      return absl::InvalidArgumentError(
          absl::Substitute("zip stat failed at index $0", i));
    }
    std::string name(sb.name);
    if (name.back() == '/') {
      if (absl::StartsWith(skip_prefix,
                           absl::ClippedSubstr(name, 0, name.size() - 1))) {
        continue;
      }
      // Create dir
      absl::string_view skipped_name = absl::StripPrefix(name, skip_prefix);
      std::string destination_path = file::JoinPath(destination, skipped_name);
      destination_path = absl::StripPrefix(destination_path, skip_prefix);
      LOG(INFO) << "Creating dir " << destination_path;
      INTR_RETURN_IF_ERROR(
          file::RecursivelyCreateDir(destination_path, file::Defaults()));
    } else {
      auto [rel_path, file_name] = file::SplitPath(name);
      rel_path = absl::StripPrefix(rel_path, skip_prefix);
      INTR_RETURN_IF_ERROR(file::RecursivelyCreateDir(
          file::JoinPath(destination, rel_path), file::Defaults()));
      std::string destination_path =
          file::JoinPath(destination, file::JoinPath(rel_path, file_name));
      zip_file_t* source_file = zip_fopen_index(&zip_archive, i, 0);
      absl::Cleanup close_file = [source_file]() {
        if (source_file) {
          zip_fclose(source_file);
        }
      };

      if (source_file == nullptr) {
        return absl::InternalError(
            absl::Substitute("Failed to open $0 in archive", name));
      }

      INTR_RETURN_IF_ERROR(UnzipToPath(source_file, destination_path, sb.size))
              .LogError()
          << "Failed to stream file " << name;
    }
  }
  if (zip_close(&zip_archive) != 0) {
    return absl::InternalError("Failed to close zip archive");
  }
  return absl::OkStatus();
}

}  // namespace

absl::Status UnzipData(absl::string_view zipped_data,
                       absl::string_view destination,
                       const UnzipDataOptions& options) {
  zip_error_t error;
  zip_source_t* zip_source = zip_source_buffer_create(
      zipped_data.data(), zipped_data.size(), 0, &error);
  zip_t* zip_archive = zip_open_from_source(zip_source, 0, &error);
  return UnzipArchive(*zip_archive, destination, options);
}

}  // namespace intrinsic
