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

#include "intrinsic/math/signals/gauss_kronrod_quadrature.h"

#include <cmath>
#include <cstddef>
#include <memory>

#include "absl/functional/function_ref.h"
#include "absl/log/log.h"
#include "absl/memory/memory.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/string_view.h"
#include "absl/types/span.h"
#include "intrinsic/math/signals/nested_quadrature.h"
#include "intrinsic/util/status/status_macros.h"

namespace intrinsic {
namespace {

// Tabulated data for the Gauss-Kronrod rules of order `N` (embedded,
// lower-order Gaussian quadrature of order `(N-1)/2`, Kronrod extension of
// order `(N+1)/2`). Note that N is always an odd number. For each rule, the
// nodes and weights (quadrature pairs) for the canonical integration interval
// [0,1] are stored. Since the nodes are symmetric with respect to 0, only the
// nodes in the range (0,1] are stored in `kPositiveNodesGK<N>`. Note that the
// central 0 element is not part of the stored nodes. The nodes in even
// positions are exclusive to the Kronrod extension (higher-order rule), whereas
// the ones in odd positions are also used for the embedded (lower-order)
// Gaussian rule. In case `(N-1)/2` is an odd number, the central 0 element will
// be also considered in the Gaussian quadrature. The weights corresponding to
// each node are stored separately for the Gaussian quadrature in
// `kGaussWeightsGK<N>` and the Kronrod extension in `kKronrodWeightsGK<N>`. The
// last element of `kKronrodWeightsGK<N>` corresponds to the weight of the
// central 0 node. This is true also for `kGaussWeightsGK<N>`, in case `(N-1)/2`
// is an odd number.

// Tabulated data for the Gauss-Kronrod rule with 15 nodes (embedded rule with 7
// nodes).
constexpr double kPositiveNodesGK15[7] = {
    0.991455371120812639206854697526329, 0.949107912342758524526189684047851,
    0.864864423359769072789712788640926, 0.741531185599394439863864773280788,
    0.586087235467691130294144838258730, 0.405845151377397166906606412076961,
    0.207784955007898467600689403773245};

constexpr double kGaussWeightsGK15[4] = {
    0.129484966168869693270611432679082, 0.279705391489276667901467771423780,
    0.381830050505118944950369775488975, 0.417959183673469387755102040816327};

constexpr double kKronrodWeightsGK15[8] = {
    0.022935322010529224963732008058970, 0.063092092629978553290700663189204,
    0.104790010322250183839876322541518, 0.140653259715525918745189590510238,
    0.169004726639267902826583426598550, 0.190350578064785409913256402421014,
    0.204432940075298892414161999234649, 0.209482141084727828012999174891714};

// Tabulated data for the Gauss-Kronrod rule with 21 nodes (embedded rule with
// 10 nodes).
constexpr double kPositiveNodesGK21[10] = {
    0.995657163025808080735527280689003, 0.973906528517171720077964012084452,
    0.930157491355708226001207180059508, 0.865063366688984510732096688423493,
    0.780817726586416897063717578345042, 0.679409568299024406234327365114874,
    0.562757134668604683339000099272694, 0.433395394129247190799265943165784,
    0.294392862701460198131126603103866, 0.148874338981631210884826001129720};

constexpr double kGaussWeightsGK21[5] = {
    0.066671344308688137593568809893332, 0.149451349150580593145776339657697,
    0.219086362515982043995534934228163, 0.269266719309996355091226921569469,
    0.295524224714752870173892994651338};

constexpr double kKronrodWeightsGK21[11] = {
    0.011694638867371874278064396062192, 0.032558162307964727478818972459390,
    0.054755896574351996031381300244580, 0.075039674810919952767043140916190,
    0.093125454583697605535065465083366, 0.109387158802297641899210590325805,
    0.123491976262065851077958109831074, 0.134709217311473325928054001771707,
    0.142775938577060080797094273138717, 0.147739104901338491374841515972068,
    0.149445554002916905664936468389821};

// A struct to hold (references to) tabulated data for a Gauss-Kronrod rule.
struct GaussKronrodQuadraturePairs {
  absl::Span<const double> positive_nodes;
  absl::Span<const double> gauss_weights;
  absl::Span<const double> kronrod_weights;
};

// A function to validate the tabulated data for a Gauss-Kronrod rule.
absl::Status ValidateGaussKronrodQuadraturePairs(
    const GaussKronrodQuadraturePairs& quadrature_pairs) {
  if (quadrature_pairs.positive_nodes.empty()) {
    return absl::InvalidArgumentError("The positive nodes must not be empty.");
  }

  const size_t num_nodes = quadrature_pairs.positive_nodes.size();
  if (num_nodes != quadrature_pairs.kronrod_weights.size() - 1) {
    return absl::InvalidArgumentError(absl::StrCat(
        "The number of positive nodes must be one less than "
        "the number of Kronrod weights. Got ",
        num_nodes, " nodes and ", quadrature_pairs.kronrod_weights.size(),
        " Kronrod weights."));
  }

  if (num_nodes % 2 == 0 &&
      quadrature_pairs.gauss_weights.size() != num_nodes / 2) {
    return absl::InvalidArgumentError(
        absl::StrCat("For an even number of positive nodes, the number "
                     "of Gauss weights must be half the number of nodes. Got ",
                     num_nodes, " nodes and ",
                     quadrature_pairs.gauss_weights.size(), " Gauss weights."));
  }

  if (num_nodes % 2 != 0 &&
      quadrature_pairs.gauss_weights.size() != (num_nodes + 1) / 2) {
    return absl::InvalidArgumentError(absl::StrCat(
        "For an odd number of positive nodes `num_nodes`, the number "
        "of Gauss weights must be (`num_nodes` + 1)/2. Got ",
        num_nodes, " nodes and ", quadrature_pairs.gauss_weights.size(),
        " Gauss weights."));
  }

  return absl::OkStatus();
}

// Returns the tabulated data for the Gauss-Kronrod rule corresponding to
// the given `key`. Returns an error for unsupported `key`.
absl::StatusOr<GaussKronrodQuadraturePairs> GetGaussKronrodQuadraturePairs(
    GaussKronrodKey key) {
  GaussKronrodQuadraturePairs quadrature_pairs;
  switch (key) {
    case GaussKronrodKey::kGaussKronrod15:
      quadrature_pairs = {
          .positive_nodes = absl::MakeConstSpan(kPositiveNodesGK15),
          .gauss_weights = absl::MakeConstSpan(kGaussWeightsGK15),
          .kronrod_weights = absl::MakeConstSpan(kKronrodWeightsGK15)};
      break;
    case GaussKronrodKey::kGaussKronrod21:
      quadrature_pairs = {
          .positive_nodes = absl::MakeConstSpan(kPositiveNodesGK21),
          .gauss_weights = absl::MakeConstSpan(kGaussWeightsGK21),
          .kronrod_weights = absl::MakeConstSpan(kKronrodWeightsGK21)};
      break;
    default:
      return absl::InvalidArgumentError(
          absl::StrCat("Unsupported GaussKronrodKey: ", key));
  }

  absl::Status validation_status =
      ValidateGaussKronrodQuadraturePairs(quadrature_pairs);
  if (!validation_status.ok()) {
    return absl::InternalError(
        absl::StrCat("Invalid Gauss-Kronrod tabulated data for key ", key, ": ",
                     validation_status.message()));
  }
  return quadrature_pairs;
}

}  // namespace

GaussKronrodQuadrature::GaussKronrodQuadrature(
    absl::Span<const double> positive_nodes,
    absl::Span<const double> kronrod_weights,
    absl::Span<const double> gauss_weights)
    : positive_nodes_(positive_nodes),
      kronrod_weights_(kronrod_weights),
      gauss_weights_(gauss_weights) {}

absl::StatusOr<std::unique_ptr<GaussKronrodQuadrature>>
GaussKronrodQuadrature::Create(GaussKronrodKey key) {
  INTR_ASSIGN_OR_RETURN(const GaussKronrodQuadraturePairs quadrature_pairs,
                        GetGaussKronrodQuadraturePairs(key));

  // WrapUnique due to private constructor.
  return absl::WrapUnique(new GaussKronrodQuadrature(
      quadrature_pairs.positive_nodes, quadrature_pairs.kronrod_weights,
      quadrature_pairs.gauss_weights));
}

absl::StatusOr<NestedQuadratureResult> GaussKronrodQuadrature::Integrate(
    absl::FunctionRef<absl::StatusOr<double>(double)> function,
    const double start, const double end) const {
  if (start > end) {
    return absl::InvalidArgumentError(absl::StrCat(
        "Invalid integration interval. Got start > end: ", start, " > ", end));
  }

  // Initialize the results of the two quadrature rules with the contribution of
  // the center node (if any).
  double kronrod_result = 0.0;
  double gauss_result = 0.0;

  // Kronrod extension is based on an odd number of nodes. Thus, we start
  // adding the contribution of the central 0 node. Scaling this node to the
  // integration interval [start, end] would simply return the center of the
  // interval. The corresponding Kronrod weight is stored at the end of the
  // Kronrod weights vector.
  const double center = 0.5 * (end + start);
  INTR_ASSIGN_OR_RETURN(const double f_eval_center, function(center));
  const double kronrod_weight_center =
      kronrod_weights_[kronrod_weights_.size() - 1];
  kronrod_result += kronrod_weight_center * f_eval_center;

  // The total number of nodes for the Gauss quadrature corresponds to the
  // number of positive nodes. If this is an odd number, the central 0
  // node is also part of the Gauss quadrature. The corresponding Gauss weight
  // is stored at the end of the Gauss weights vector.
  if (positive_nodes_.size() % 2 != 0) {
    const double gauss_weight_center =
        gauss_weights_[gauss_weights_.size() - 1];
    gauss_result += gauss_weight_center * f_eval_center;
  }

  // Add contribution of all the other nodes to the integrals. For each node in
  // `positive_nodes_`, we need to consider the symmetric node in [-1,0) as
  // well. Note that the Kronrod rule takes all nodes in account, whereas the
  // Gauss rule only takes the odd ones. To compute the integrals over the
  // interval [start, end], we need to scale the nodes from their canonical
  // interval [-1, 1]. The final results needs finally to be scaled by half of
  // the interval length (see
  // https://en.wikipedia.org/wiki/Gaussian_quadrature#Change_of_interval).
  const double half_interval_length = 0.5 * (end - start);
  for (int i = 0; i < positive_nodes_.size(); ++i) {
    const double node = positive_nodes_[i];

    // Scale the node to the integration interval [start, end] and compute the
    // symmetric node in [-1,0). For the symmetric node, we directly compute the
    // scaled value.
    const double node_scaled = center + half_interval_length * node;
    const double symmetric_node_scaled = center - half_interval_length * node;

    INTR_ASSIGN_OR_RETURN(const double f_eval_node, function(node_scaled));
    INTR_ASSIGN_OR_RETURN(const double f_eval_symmetric_node,
                          function(symmetric_node_scaled));
    const double f_eval_sum = f_eval_node + f_eval_symmetric_node;

    const double kronrod_weight = kronrod_weights_[i];
    kronrod_result += kronrod_weight * f_eval_sum;

    if (i % 2 != 0) {
      // Nodes in odd positions are used for the lower order Gaussian
      // quadrature.
      const double gauss_weight = gauss_weights_[i / 2];
      gauss_result += gauss_weight * f_eval_sum;
    }
  }

  // Scale the results to the integration interval [start, end].
  kronrod_result *= half_interval_length;
  gauss_result *= half_interval_length;

  return NestedQuadratureResult{
      .integral = kronrod_result,
      .error_estimate = std::abs(kronrod_result - gauss_result)};
}

int GaussKronrodQuadrature::NumNodesEmbeddedRule() const {
  return positive_nodes_.size();
}

int GaussKronrodQuadrature::NumNodesExtendedRule() const {
  return 2 * positive_nodes_.size() + 1;
}

}  // namespace intrinsic
