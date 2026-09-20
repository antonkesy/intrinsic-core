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

#ifndef INTRINSIC_PERCEPTION_CORE_CORE_IO_H_
#define INTRINSIC_PERCEPTION_CORE_CORE_IO_H_

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "absl/log/log.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/string_view.h"
#include "absl/time/time.h"
#include "google/protobuf/text_format.h"
#include "intrinsic/perception/cameras/capture_result.h"
#include "intrinsic/perception/cameras/sensor_image.h"
#include "intrinsic/perception/core/camera_params.h"
#include "intrinsic/perception/core/image.h"
#include "intrinsic/perception/core/image_traits.h"
#include "intrinsic/util/proto/error_collector.h"
#include "intrinsic/util/status/ret_check.h"
#include "intrinsic/util/status/status_macros.h"
#include "ortools/base/options.h"

namespace intrinsic {
namespace perception {

struct FilenameSuffixConstants {
  static constexpr char kSuffixGrayFilename[] = "gray.png";
  static constexpr char kSuffixGray8uFilename[] = "gray8u.png";
  static constexpr char kSuffixRgbFilename[] = "rgb.png";
  static constexpr char kSuffixDepthFilename[] = "depth.png";
  static constexpr char kSuffixPointFilename[] = "point.png";
  static constexpr char kSuffixNormalFilename[] = "normal.png";
};

std::string SensorIdFilenameSuffix(int64_t sensor_id);

// Reads image from disk.
//
// Example use:
//   Image<Bgr8u> image = ReadImage<Bgr8u>("filename.png").value();
template <typename ImageTrait>
absl::StatusOr<Image<ImageTrait>> ReadImage(absl::string_view filename);

// Reads image from serialized string, e.g. a string that contains the
// serialized image in JPEG or PNG.
template <typename ImageTrait>
absl::StatusOr<Image<ImageTrait>> ReadImageFromString(
    absl::string_view serialized_image);

// WriteImage writes an image to disk at the specified path.
//
//  Supports all image formats supported by opencv.
//
//  Note: Compression_level is only supported for png and jpg images.
//  Compression_level for png must be between 0 and 9 (where 9 has the highest
//  compression and is slowest, with default = 6), while compression_level for
//  jpg is between 0 and 100 (where 0 is lowest quality and 100 is best quality,
//  with default = 95). Note: For jpg and png images, writing to the Google file
//  system is supported. For all other formats, only local writing is supported.
template <typename ImageTrait>
absl::Status WriteImage(
    absl::string_view filename, const Image<ImageTrait>& image,
    std::optional<uint8_t> compression_level = std::nullopt);

absl::Status WriteSensorImage(absl::string_view filename,
                              const SensorImage& sensor_image);

// Writes all available sensor images of a capture result as png images
// into 'images_dir'. The capture result contains an acquisition time and the
// generated filename will be '%Y%m%d-%H%M%S_s123.png' for `sensor_id=123`. The
// function returns a map of sensor id to string which holds all filenames in
// which the individual sensor images were saved.
absl::StatusOr<absl::flat_hash_map<int64_t, std::string>> WriteCaptureResult(
    const CaptureResult& capture_result, absl::string_view images_dir);

// Writes all available sensor images of a capture result as png images into
// 'images_dir' and uses 'num_frame' to distinguish individual frames in the
// filenames. The filename format will be '000000_s123.png' for `sensor_id=123`
// when num_frame = 0. The function returns a map of sensor id to string which
// holds all filenames in which the individual sensor images were saved.
absl::StatusOr<absl::flat_hash_map<int64_t, std::string>> WriteCaptureResult(
    const CaptureResult& capture_result, absl::string_view images_dir,
    int num_frame);

absl::StatusOr<absl::flat_hash_map<int64_t, std::string>> WriteCaptureResult(
    const CaptureResult& capture_result, absl::string_view images_dir,
    absl::string_view images_prefix);

absl::StatusOr<CaptureResult> ReadCaptureResult(
    absl::string_view images_dir, int num_frame,
    const CameraParams& camera_params, absl::Time capture_at);

// SensorIdFilenameSuffix are used to extract the id of each image from
// 'relative_filenames', which are relative with respect to 'images_dir'.
absl::StatusOr<CaptureResult> ReadCaptureResult(
    absl::string_view images_dir,
    const std::vector<std::string>& relative_filenames,
    const CameraParams& camera_params, absl::Time capture_at);

// Checks whether the file exists in storage.
absl::Status FileExistsInStorage(absl::string_view filename);

// Creates a directory at the path provided. Also creates any parent directories
// that didn't exist at the time of the call. If the provided path is for
// GCS, this function does nothing.
absl::Status RecursivelyCreateDirInStorage(absl::string_view dir_path);

// ReadFileFromStorage reads a file and outputs its content into a string.
absl::StatusOr<std::string> ReadFileFromStorage(absl::string_view filename);

// FindFilesInStorage finds all files in one directory with a glob pattern. The
// dir path can also be given in glob pattern (for GCS paths the function
// doesn't support this feature).
// Note : For GCS paths the extension pattern must start with a single "*" or
// left empty.
// Example call: FindFilesInStorage("/tmp/my/path", "*.png")
absl::StatusOr<std::vector<std::string>> FindFilesInStorage(
    absl::string_view dir_path, absl::string_view extension_pattern,
    bool recursive = false);

// WriteFileFromStorage writes content into a file and returns its full path.
// In case of a GCS path the returned full path will have the generation number
// attached.
absl::StatusOr<std::string> WriteFileToStorage(absl::string_view content,
                                               absl::string_view filename);

// Reads text proto from storage.
template <typename ProtoType>
absl::StatusOr<ProtoType> ReadTextProto(absl::string_view filename) {
  INTR_ASSIGN_OR_RETURN(std::string content, ReadFileFromStorage(filename));

  SimpleErrorCollector collector;
  google::protobuf::TextFormat::Parser parser;
  parser.RecordErrorsTo(&collector);
  ProtoType proto;
  if (!parser.ParseFromString(content, &proto)) {
    LOG(INFO) << collector.str();
    return absl::InvalidArgumentError(absl::StrCat(
        "Couldn't parse text proto of format ", proto.GetTypeName(), "from ",
        filename, ": ", collector.str()));
  }
  return proto;
}

template <typename ProtoType>
absl::StatusOr<ProtoType> ReadBinaryProto(absl::string_view filename) {
  INTR_ASSIGN_OR_RETURN(std::string content, ReadFileFromStorage(filename));

  ProtoType proto;

  if (!proto.ParsePartialFromString(content)) {
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

// Writes proto in text format to storage and returns its full path.
template <typename ProtoType>
absl::StatusOr<std::string> WriteTextProto(const ProtoType& proto,
                                           absl::string_view filename) {
  std::string text_proto;
  INTR_RET_CHECK(proto.IsInitialized())
          .SetCode(absl::StatusCode::kFailedPrecondition)
      << "Missing required field: " << proto.InitializationErrorString();
  INTR_RET_CHECK(
      google::protobuf::TextFormat::PrintToString(proto, &text_proto));
  return WriteFileToStorage(text_proto, filename);
}

// Writes proto in binary format to storage and returns its full path.
template <typename ProtoType>
absl::StatusOr<std::string> WriteBinaryProto(const ProtoType& proto,
                                             absl::string_view filename) {
  return WriteFileToStorage(proto.SerializePartialAsString(), filename);
}

bool IsGcsPath(absl::string_view path);

// Writes point_cloud as ply to disk.
absl::Status WritePointCloudAsPly(const Image<Point32f>& point_cloud,
                                  absl::string_view filename);

}  // namespace perception
}  // namespace intrinsic

#endif  // INTRINSIC_PERCEPTION_CORE_CORE_IO_H_
