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

#ifndef INTRINSIC_PERCEPTION_PROTO_CONVERSION_V1_CAMERA_DRIVERS_H_
#define INTRINSIC_PERCEPTION_PROTO_CONVERSION_V1_CAMERA_DRIVERS_H_

#include "intrinsic/perception/cameras/camera_identifier.h"
#include "intrinsic/perception/proto/v1/camera_drivers.pb.h"

namespace intrinsic_proto::perception::v1 {

intrinsic::perception::CameraIdentifier::GenICam FromProto(
    const CameraDrivers::GenICam& genicam);

CameraDrivers::GenICam ToProto(
    const intrinsic::perception::CameraIdentifier::GenICam& genicam);

intrinsic::perception::CameraIdentifier::Ros FromProto(
    const CameraDrivers::Ros& ros);

CameraDrivers::Ros ToProto(
    const intrinsic::perception::CameraIdentifier::Ros& ros);

intrinsic::perception::CameraIdentifier::FileCamera FromProto(
    const CameraDrivers::FileCamera& file_camera);

CameraDrivers::FileCamera ToProto(
    const intrinsic::perception::CameraIdentifier::FileCamera& file_camera);

intrinsic::perception::CameraIdentifier::FakeGenICam FromProto(
    const CameraDrivers::FakeGenICam& fake_genicam);

CameraDrivers::FakeGenICam ToProto(
    const intrinsic::perception::CameraIdentifier::FakeGenICam& fake_genicam);

}  // namespace intrinsic_proto::perception::v1

// Convenience ToProto aliases to allow argument-dependent lookup. Directly use
// the versioned namespace alternatives above instead when dealing with
// different versions at the same time.
namespace intrinsic::perception {

using ::intrinsic_proto::perception::v1::ToProto;

}  // namespace intrinsic::perception

#endif  // INTRINSIC_PERCEPTION_PROTO_CONVERSION_V1_CAMERA_DRIVERS_H_
