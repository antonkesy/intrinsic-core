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

#ifndef INTRINSIC_ICON_CONTROL_ALGORITHMS_COMPUTE_CRITICAL_DAMPING_H_
#define INTRINSIC_ICON_CONTROL_ALGORITHMS_COMPUTE_CRITICAL_DAMPING_H_

#include "intrinsic/eigenmath/types.h"
#include "intrinsic/icon/utils/realtime_status_or.h"

namespace intrinsic::icon {

// Computes scalar-valued critical damping 'd' for a single degree of freedom
// damped oscillator of form m*dv/dt + d*v + k*x = f.
RealtimeStatusOr<double> ComputeCriticalDamping(double m, double k);

// Computes the critical damping matrix D for a multivariate system of
// form M*dv/dt + D*v + K*x = f. The critical damping matrix is given
// as: D = 2.0 * sqrt(M) * sqrt(sqrt(M).inverse() * K * sqrt(M).inverse()) *
// sqrt(M), where M is the symmetric positive definite cartesian inertia matrix
// and K is the symmetric positive definite cartesian stiffness. Since the
// matrices M and K are assumed to be symmetric, it is guaranteed that a damping
// matrix exists which decouples the motion into N independent degrees of
// freedom. Further reference: D. Inman, "CRITICAL DAMPING",  Encyclopedia of
// Vibration, 2001.
RealtimeStatusOr<eigenmath::MatrixNd> ComputeCriticalDamping(
    const eigenmath::MatrixNd& M, const eigenmath::MatrixNd& K);

// Computes the critical damping matrix D for a multivariate system of
// form M*dv/dt + D*v + K*x = f. The critical damping matrix is given
// as: D = 2.0 * sqrt(M) * sqrt(sqrt(M).inverse() * K * sqrt(M).inverse()) *
// sqrt(M). `Minv` is the inverse of the symmetric positive definite cartesian
// inertia matrix and `K` is the symmetric positive definite cartesian
// stiffness. Since the matrices `Minv` and `K` are assumed to be symmetric, it
// is guaranteed that a damping matrix exists which decouples the motion into N
// independent degrees of freedom. Further reference: D. Inman, "CRITICAL
// DAMPING",  Encyclopedia of Vibration, 2001.
RealtimeStatusOr<eigenmath::MatrixNd>
ComputeCriticalDampingFromInverseInertiaMatrix(const eigenmath::MatrixNd& Minv,
                                               const eigenmath::MatrixNd& K);

}  // namespace intrinsic::icon

#endif  // INTRINSIC_ICON_CONTROL_ALGORITHMS_COMPUTE_CRITICAL_DAMPING_H_
