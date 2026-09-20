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

// This file offers protobuf:native conversion routines, provided by
// blue/shared/messages/proto_utils according to our preferred conventions.
// See proto_conversion.h for details.
//
// We intentionally add those definitions to the respective namespace of their
// argument, so that argument-dependent lookup will work naturally.

#ifndef THIRD_PARTY_INTRINSIC_UTILS_PROTO_CONVERSION_BLUE_WRAPPERS_H_
#define THIRD_PARTY_INTRINSIC_UTILS_PROTO_CONVERSION_BLUE_WRAPPERS_H_

#include "intrinsic/eigenmath/pose2.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/math/pose3.h"
#include "intrinsic/messages/geometry_types.pb.h"
#include "intrinsic/utils/proto_conversion/blue_wrappers_no_using.h"
#include "intrinsic/utils/proto_conversion/proto_conversion.h"

namespace intrinsic {

using blue::eigenmath::ToProto;
using blue::messages_proto::FromProto;
using Eigen::ToProto;

}  // namespace intrinsic

#endif  // THIRD_PARTY_INTRINSIC_UTILS_PROTO_CONVERSION_BLUE_WRAPPERS_H_
