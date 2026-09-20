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

#ifndef INTRINSIC_MATH_SIGNALS_GAUSS_LEGENDRE_QUADRATURE_H_
#define INTRINSIC_MATH_SIGNALS_GAUSS_LEGENDRE_QUADRATURE_H_

// A library for the numerical integration of functions via Gauss-Legendre
// quadrature (https://en.wikipedia.org/wiki/Gauss%E2%80%93Legendre_quadrature).
// The computation of nodes and weights is based on the method described in:
//
// Bogaert, Ignace. "Iteration-Free Computation of Gauss-Legendre Quadrature
// Nodes and Weights." SIAM Journal on Scientific Computing 36.3 (2014):
// A1008-A1026,
//
// chosen for its efficiency and high accuracy. For a literature review on this
// topic, refer to:
//
// Townsend, Alex. "The race for high order Gauss–Legendre quadrature."
// http://math.mit.edu/~ajt/papers/QuadratureEssay.pdf.

#include "absl/functional/function_ref.h"
#include "absl/status/statusor.h"

namespace intrinsic {

// A struct representing a Gaussian quadrature node-weight pair (see
// https://en.wikipedia.org/wiki/Gaussian_quadrature for more details).
// Note that the quadrature node is expressed as an angle in radians, since it
// is numerically advantageous to do so. A node is related to its angle
// representation `node_as_angle_rad` by node = cos(`node_as_angle_rad`).
struct QuadraturePair {
  // The node expressed as angle. A node is related to its angle
  // representation `node_as_angle_rad` by node = cos(`node_as_angle_rad`).
  double node_as_angle_rad;

  // The weight associated with the node.
  double weight;
};

// Computes the `node_idx`-th QuadraturePair for the Legendre polynomial of
// degree `num_nodes`. The `node_as_angle_rad` of the returned pair represents
// the angle value associated with the `node_idx`-th zero of the Legendre
// polynomial. Its value increases in the range [0,pi] as `node_idx` increases
// between 0 and `num_nodes`-1. Returns an error for non-positive `num_nodes`,
// or for `node_idx` outside of the range [0,n-1].
absl::StatusOr<QuadraturePair> ComputeGaussLegendreQuadraturePair(int num_nodes,
                                                                  int node_idx);

// Computes the `num_nodes`-point Gauss-Legendre quadrature of the given
// `function` over the interval [start,end], approximating the integral of
// `function` over such interval. The `function` must be defined in the interval
// [start,end] and be sufficiently smooth for the numerical integration to be
// accurate. Returns an error for non-positive `num_nodes`, or for `start`
// greater than `end`.
absl::StatusOr<double> GaussLegendreQuadrature(
    absl::FunctionRef<absl::StatusOr<double>(double)> function, double start,
    double end, int num_nodes);

}  // namespace intrinsic

#endif  // INTRINSIC_MATH_SIGNALS_GAUSS_LEGENDRE_QUADRATURE_H_
