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

#ifndef INTRINSIC_PERCEPTION_CORE_COMPUTE_NORMALS_LEAST_SQR_H_
#define INTRINSIC_PERCEPTION_CORE_COMPUTE_NORMALS_LEAST_SQR_H_

#include <cstdint>

#include "intrinsic/perception/core/image.h"
#include "intrinsic/perception/core/image_traits.h"
#include "intrinsic/perception/core/intrinsic_params.h"

namespace intrinsic::perception {

// Estimates normals for a set of ordered 3D points.
//
// Implements algorithm described in "Multimodal Templates for Real-Time
// Detection of Texture-less Objects in Heavily Cluttered Scenes" from
// S. Hinterstoisser, S. Holzer, C. Cagniart,
// S. Ilic, K. Konolige, N. Navab, V. Lepetit
//
// This algorithm computes the normals in a linear least squares way
// using a bilateral filtering schema. It expects a depth image as input.
// The distance threshold is given in [meter]. radius
// is in [pixels]. Threshold is in [meters].
Image<Normal32f> ComputeNormalsLeastSqr(const IntrinsicParams& intrinsic_params,
                                        const Image<Depth32f>& depth,
                                        float threshold, int32_t radius,
                                        int32_t step);

}  // namespace intrinsic::perception

#endif  // INTRINSIC_PERCEPTION_CORE_COMPUTE_NORMALS_LEAST_SQR_H_
