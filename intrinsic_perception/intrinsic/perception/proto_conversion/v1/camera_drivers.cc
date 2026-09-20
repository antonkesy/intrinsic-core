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

#include "intrinsic/perception/proto_conversion/v1/camera_drivers.h"

#include <cstdint>
#include <optional>
#include <utility>
#include <variant>
#include <vector>

#include "absl/functional/overload.h"
#include "intrinsic/perception/cameras/camera_identifier.h"
#include "intrinsic/perception/core/image_file_reference.h"
#include "intrinsic/perception/proto/v1/camera_drivers.pb.h"
#include "intrinsic/perception/proto/v1/image_file_reference.pb.h"
#include "intrinsic/perception/proto_conversion/v1/image_file_reference.h"

namespace intrinsic_proto::perception::v1 {

intrinsic::perception::CameraIdentifier::GenICam FromProto(
    const CameraDrivers::GenICam& genicam) {
  return intrinsic::perception::CameraIdentifier::GenICam{
      .device_id = genicam.device_id()};
}

CameraDrivers::GenICam ToProto(
    const intrinsic::perception::CameraIdentifier::GenICam& genicam) {
  CameraDrivers::GenICam proto;
  proto.set_device_id(genicam.device_id);
  return proto;
}

intrinsic::perception::CameraIdentifier::Ros FromProto(
    const CameraDrivers::Ros& ros) {
  return intrinsic::perception::CameraIdentifier::Ros{
      .driver_type = ros.driver_type(), .device_id = ros.device_id()};
}

CameraDrivers::Ros ToProto(
    const intrinsic::perception::CameraIdentifier::Ros& ros) {
  CameraDrivers::Ros proto;
  proto.set_driver_type(ros.driver_type);
  proto.set_device_id(ros.device_id);

  return proto;
}

intrinsic::perception::CameraIdentifier::FileCamera FromProto(
    const CameraDrivers::FileCamera& file_camera) {
  intrinsic::perception::CameraIdentifier::FileCamera::Source source =
      std::monostate();
  if (file_camera.has_directory()) {
    std::vector<intrinsic::perception::ImageFileReference> sensor_refs;
    sensor_refs.reserve(file_camera.directory().sensor_references_size());
    for (const intrinsic_proto::perception::v1::ImageFileReference& ref :
         file_camera.directory().sensor_references()) {
      sensor_refs.push_back(FromProto(ref));
    }
    source = intrinsic::perception::CameraIdentifier::FileCamera::Directory{
        .path = file_camera.directory().path(),
        .sensor_references = std::move(sensor_refs),
        .remove_incomplete_frames =
            file_camera.directory().remove_incomplete_frames()};
  } else if (file_camera.has_files()) {
    std::vector<std::vector<intrinsic::perception::ImageFileReference>>
        sensor_files_lists;
    sensor_files_lists.reserve(file_camera.files().sensor_files_lists_size());
    for (const intrinsic_proto::perception::v1::
             CameraDrivers_FileCamera_SensorFilesList& sensor_files_list_proto :
         file_camera.files().sensor_files_lists()) {
      std::vector<intrinsic::perception::ImageFileReference> file_references;
      file_references.reserve(sensor_files_list_proto.file_references_size());
      for (const intrinsic_proto::perception::v1::ImageFileReference&
               file_reference_proto :
           sensor_files_list_proto.file_references()) {
        file_references.push_back(FromProto(file_reference_proto));
      }
      sensor_files_lists.push_back(std::move(file_references));
    }
    source = intrinsic::perception::CameraIdentifier::FileCamera::Files{
        .sensor_files_lists = std::move(sensor_files_lists),
    };
  }
  return intrinsic::perception::CameraIdentifier::FileCamera{
      .source = std::move(source),
      .start_index = file_camera.has_start_index()
                         ? std::optional<int64_t>(file_camera.start_index())
                         : std::nullopt,
      .loop_files = file_camera.loop_files(),
  };
}

CameraDrivers::FileCamera ToProto(
    const intrinsic::perception::CameraIdentifier::FileCamera& file_camera) {
  CameraDrivers::FileCamera proto;
  std::visit(
      absl::Overload{
          [](const std::monostate&) {},
          [&proto](const intrinsic::perception::CameraIdentifier::FileCamera::
                       Directory& source) {
            intrinsic_proto::perception::v1::CameraDrivers_FileCamera_Directory*
                dir_proto = proto.mutable_directory();
            dir_proto->set_path(source.path);
            dir_proto->set_remove_incomplete_frames(
                source.remove_incomplete_frames);
            for (const intrinsic::perception::ImageFileReference& ref :
                 source.sensor_references) {
              *dir_proto->add_sensor_references() = ToProto(ref);
            }
          },
          [&proto](
              const intrinsic::perception::CameraIdentifier::FileCamera::Files&
                  source) {
            for (const std::vector<intrinsic::perception::ImageFileReference>&
                     sensor_files_list : source.sensor_files_lists) {
              intrinsic_proto::perception::v1::
                  CameraDrivers_FileCamera_SensorFilesList*
                      sensor_files_list_proto =
                          proto.mutable_files()->add_sensor_files_lists();
              for (const intrinsic::perception::ImageFileReference&
                       file_reference : sensor_files_list) {
                sensor_files_list_proto->mutable_file_references()->Add(
                    ToProto(file_reference));
              }
            }
          },
      },
      file_camera.source);
  if (file_camera.start_index.has_value()) {
    proto.set_start_index(*file_camera.start_index);
  }
  proto.set_loop_files(file_camera.loop_files);
  return proto;
}

intrinsic::perception::CameraIdentifier::FakeGenICam FromProto(
    const CameraDrivers::FakeGenICam& fake_genicam) {
  return intrinsic::perception::CameraIdentifier::FakeGenICam();
}

CameraDrivers::FakeGenICam ToProto(
    const intrinsic::perception::CameraIdentifier::FakeGenICam& fake_genicam) {
  return CameraDrivers::FakeGenICam();
}

}  // namespace intrinsic_proto::perception::v1
