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

#ifndef INTRINSIC_MATH_SIGNALS_GAUSS_KRONROD_QUADRATURE_H_
#define INTRINSIC_MATH_SIGNALS_GAUSS_KRONROD_QUADRATURE_H_

// A library for the numerical integration of functions via Gauss-Kronrod nested
// quadrature
// (https://en.wikipedia.org/wiki/Gauss%E2%80%93Kronrod_quadrature_formula). The
// computation is based on tabulated data to optimize efficiency and accuracy.
// Quadrature rules of different orders are supported via the specification of
// the `GaussKronrodKey`.
//
// Gauss-Kronrod quadrature is a nested quadrature rule based on a total number
// of `N` nodes, `N` being an odd number. It uses an `(N-1)/2`-point
// Gauss-Legendre quadrature as embedded, lower-order rule and an
// `(N+1)/2`-point Kronrod extension. The resulting rule can integrate exactly
// polynomial functions with degree up to `3n+1`. For a literature review on
// this topic, refer to:
//
// Notaris, Sotirios E. "Gauss-Kronrod quadrature formulae
// - a survey of fifty years of research." Electron. Trans. Numer. Anal 45
// (2016): 371-404.

#include <stdbool.h>

#include <memory>

#include "absl/functional/function_ref.h"
#include "absl/status/statusor.h"
#include "absl/types/span.h"
#include "intrinsic/math/signals/nested_quadrature.h"

namespace intrinsic {

// The key to select the order of the nested quadrature rule. The number used as
// suffix represents the total number `N` of nodes used in the rule. That is,
// the embedded Gauss-Legendre rule has `(N-1)/2` nodes, and the Kronrod rule
// adds `(N+1)/2` nodes. Note that N must always be an odd number.
enum class GaussKronrodKey {
  kGaussKronrod15,
  kGaussKronrod21,
};

// A nested quadrature rule based on Gauss-Kronrod quadrature.
class GaussKronrodQuadrature : public NestedQuadratureRule {
 public:
  static absl::StatusOr<std::unique_ptr<GaussKronrodQuadrature>> Create(
      GaussKronrodKey key);

  absl::StatusOr<NestedQuadratureResult> Integrate(
      absl::FunctionRef<absl::StatusOr<double>(double)> function, double start,
      double end) const override;

  int NumNodesEmbeddedRule() const override;

  int NumNodesExtendedRule() const override;

 private:
  explicit GaussKronrodQuadrature(absl::Span<const double> positive_nodes,
                                  absl::Span<const double> kronrod_weights,
                                  absl::Span<const double> gauss_weights);

  // The set of all nodes used in Gauss-Kronrod quadrature. Since the nodes are
  // symmetric with respect to 0, only the nodes in the range (0,1] are stored.
  absl::Span<const double> positive_nodes_;

  // The weights for the Kronrod rule. Contains the weights corresponding to
  // each node in `positive_nodes_` (and their symmetric counterparts) plus the
  // weight of the central 0 node.
  absl::Span<const double> kronrod_weights_;

  // The weights for the Gauss rule. By construction, only the elements in
  // `positive_nodes_` that are in odd positions (and their symmetric
  // counterparts) are used in the Gauss rule.
  absl::Span<const double> gauss_weights_;
};

}  // namespace intrinsic

#endif  // INTRINSIC_MATH_SIGNALS_GAUSS_KRONROD_QUADRATURE_H_
