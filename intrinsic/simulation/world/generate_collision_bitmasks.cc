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

#include "intrinsic/simulation/world/generate_collision_bitmasks.h"

#include <algorithm>
#include <cstdint>
#include <iterator>
#include <limits>
#include <random>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "absl/container/flat_hash_set.h"
#include "absl/numeric/bits.h"
#include "absl/status/statusor.h"
#include "intrinsic/util/status/status_builder.h"

namespace intrinsic {
namespace simulation {

namespace {

// A graph is represented by a mapping of IDs to their neighbors.
using Graph = absl::flat_hash_map<uint32_t, absl::flat_hash_set<uint32_t>>;

// A coloring is represented by a mapping of IDs to labels.
using Coloring = absl::flat_hash_map<uint32_t, int>;

// Returns the largest color value used in the given coloring.
int MaxColorValue(const Coloring& c) {
  return std::max_element(c.begin(), c.end(),
                          [](const auto& c1, const auto& c2) {
                            return c1.second < c2.second;
                          })
      ->second;
}

// Returns the complement of a given graph.
Graph GraphComplement(const Graph& graph) {
  Graph complement;

  std::vector<uint32_t> object_ids;
  std::transform(graph.begin(), graph.end(), std::back_inserter(object_ids),
                 [](const auto& p) { return p.first; });

  for (uint32_t oid : object_ids) {
    complement[oid] = absl::flat_hash_set<uint32_t>();
    for (uint32_t other_oid : object_ids) {
      if (oid == other_oid) {
        continue;
      }

      if (!graph.at(oid).contains(other_oid)) {
        complement[oid].insert(other_oid);
      }
    }
  }

  return complement;
}

// Runs the greedy graph coloring algorithm on its input. Assumes that the graph
// will require fewer colors than can fit in an int. A preconditioner can be
// supplied that attempts to be used as a starting point for a resulting
// coloring.
absl::flat_hash_map<uint32_t, int> GraphColoring(
    const Graph& graph, absl::flat_hash_map<uint32_t, int> precondition,
    std::minstd_rand0& rng) {
  std::vector<uint32_t> object_ids;
  std::transform(graph.begin(), graph.end(), std::back_inserter(object_ids),
                 [](const auto& p) { return p.first; });

  // If a color is preconditioned, set it as the starting color if valid,
  // otherwise set it to the default (-1).
  absl::flat_hash_map<uint32_t, int> colors;
  for (const auto& pair : graph) {
    uint32_t oid = pair.first;
    if (precondition[oid] >= 0 &&
        !std::any_of(pair.second.begin(), pair.second.end(),
                     [oid, &precondition](const uint32_t neighbor) {
                       return precondition[neighbor] >= 0 &&
                              precondition[oid] == precondition[neighbor];
                     })) {
      colors[oid] = precondition[oid];
    } else {
      colors[oid] = -1;
    }
  }

  // Shuffle the object ids based on the passed in RNG so that we can maintain
  // determinism. We sort first to get rid of the variability of the hash map
  // iteration.
  std::sort(object_ids.begin(), object_ids.end());
  std::shuffle(object_ids.begin(), object_ids.end(), rng);

  // For each object_id, look at the exclusions and select the smallest
  // available color.
  for (uint32_t object_id : object_ids) {
    // Preserve default label if there are no collision exclusions.
    if (graph.at(object_id).empty()) {
      continue;
    }

    // Map the exclusions (neighboring vertices) to their respective color,
    // again sorting and shuffling to maintain determinism.
    std::vector<uint32_t> exclusions(graph.at(object_id).begin(),
                                     graph.at(object_id).end());
    std::sort(exclusions.begin(), exclusions.end());
    std::shuffle(exclusions.begin(), exclusions.end(), rng);

    absl::flat_hash_set<int> labels;
    for (const uint32_t exclusion : exclusions) {
      labels.insert(colors[exclusion]);
    }

    // Take the smallest non-negative label.
    for (int i = 0; true; ++i) {
      if (!labels.contains(i)) {
        colors[object_id] = i;
        break;
      }
    }
  }

  // Any objects that were not colored are not connected in the graph and can be
  // assigned any color, so we go with color zero.
  for (auto& c : colors) {
    if (c.first == -1) {
      c.second = 0;
    }
  }

  return colors;
}

// Colorings depend on the order that we iterate over a graph's vertices. While
// we provided a deterministic RNG for this, we may need to do multiple tries to
// cover an entire graph using our colorings.
constexpr int kNumLabelingTries = 3;

// The number of cliques for which to assign bits after each coloring iteration.
// Greedy coloring biases towards lower bits, and exclusion graphs are usually
// sparse, so a small number here generally is better. Note, must be a positive
// number or the code will run forever.
constexpr int kNumCliquesPerColoring = 2;
static_assert(kNumCliquesPerColoring > 0,
              "Not taking any cliques per coloring will cause bitmask "
              "generation to never finish.");

// The number of neighbors to include in our preconditioning step. If this is
// too large, then we run the risk of including too many false-positives in our
// preconditioner, which get thrown out during the coloring step, and don't
// provide a good precondition. Converseley, too few would be close to providing
// no preconditioning at all.
constexpr int kNumNeighborsToPrecondition = 5;

// Currently we use 32-bit unsigned integers for our collision masks.
constexpr int kNumBitsInCollisionMask = 32;
static_assert(
    std::numeric_limits<Graph::key_type>::digits == kNumBitsInCollisionMask,
    "kNumBitsInCollisionMask must match number of bits in graph key type.");

}  // namespace

// Implementation details:
//
// For each object (denoted by their object ID), we are trying to generate a
// bitmask that is mutually exclusive with the bitmask of all other objects that
// are neighbors in the exclusion graph, but shares at least one bit with
// objects that are not neighbors. For this, we consider the complement: the
// collision graph of objects. In this graph, we need to cover all edges in
// order to preserve the "collidability" between all objects. The manner in
// which we cover the graphs (bitmasks) corresponds to cliques in the collision
// graph. In other words, a bit is set to 1 in an object iff it corresponds to a
// clique in the collision graph of all objects that are pairwise collidable.
// With this in mind, the goal is to provide a clique edge-cover of the
// collision graph using (hopefully) only kNumBitsInCollisionMask cliques or
// less, which is the size of our bitmask.
//
// A clique in the collision graph corresponds to an independent set in the
// exclusion graph. One can find independent sets by running a vertex coloring
// on the exclusion graph. The naive algorithm is therefore this:
//   1. Perform vertex coloring on the exclusion graph
//   2. Take all vertices of the same color, and remove the corresponding clique
//      from the collision graph.
//   3. Repeat until we've removed kNumBitsInCollisionMask cliques.
//   4. If there are no more edges in the collision graph, we're done, otherwise
//      fail.
// This will work, but usually we cannot cover the edges of the collision graph
// with kNumBitsInCollisionMask cliques derived from coloring the graph. As
// such, we apply two additional heuristics in order to reduce the bit count.
//
// Heuristic #1: Instead of using all of the cliques generated from a given
// coloring, only use the first kNumCliquesPerColoring. Since the greedy
// algorithm for vertex coloring fortunately biases for low-value colors
// (namely 0 and 1), these end up being the largest cliques. Higher-valued
// colors tend to be smaller cliques, and therefore aren't worth the bits.
//
// Heuristic #2: Precondition the coloring for the exclusion graph with
// connected vertices in the collision graph sharing the same color. If none of
// the exclusion relationships are violated, this may increase the number of
// colors used to color the graph, but it will guarantee that the first few
// colors should include edges that were not found on the previous coloring,
// increasing the chances that we've colored the entire graph.
absl::StatusOr<absl::flat_hash_map<uint32_t, uint32_t>>
GenerateCollisionBitmasks(Graph exclusion_graph) {
  if (exclusion_graph.empty()) {
    return absl::flat_hash_map<uint32_t, uint32_t>();
  }

  // When we traverse a graph, make sure that we always traverse it in the same
  // order to preserve determinism between multiple runs on the same input.
  std::vector<uint32_t> sorted_objects;
  std::transform(exclusion_graph.begin(), exclusion_graph.end(),
                 std::back_inserter(sorted_objects),
                 [](const auto p) { return p.first; });
  std::sort(sorted_objects.begin(), sorted_objects.end());

  // Make sure all neighboring vertices are present as keys.
  for (const auto& node : exclusion_graph) {
    for (const auto& neighbor : node.second) {
      if (!exclusion_graph.contains(neighbor)) {
        return ::intrinsic::InvalidArgumentErrorBuilder()
               << "Exclusion graph has edge between objects with ID "
               << node.first << " and " << neighbor << " but object with ID "
               << neighbor << " is not found in graph";
      }
    }
  }

  for (int label_try = 0; label_try < kNumLabelingTries; ++label_try) {
    // We need to make sure that we cover all of the edges in the collision
    // graph.
    Graph collision_graph = GraphComplement(exclusion_graph);

    // Seed random number generator with known value to keep world to sdf
    // deterministic.
    std::minstd_rand0 rng(label_try);

    // Each bitmask starts with zero
    absl::flat_hash_map<uint32_t, uint32_t> collision_bitmasks;
    for (const auto& object : exclusion_graph) {
      collision_bitmasks[object.first] = 0;

      // If there are no exclusions for this object, then it receives the
      // default collision bitmask.
      if (object.second.empty()) {
        collision_bitmasks[object.first] = kDefaultCollisionBitMask;
      }
    }

    int next_bit = 0;
    absl::flat_hash_map<uint32_t, int> precondition{};
    for (const auto& v : exclusion_graph) {
      precondition[v.first] = -1;
    }

    while (true) {
      // Generate coloring
      Coloring coloring = GraphColoring(exclusion_graph, precondition, rng);

      // For each color, assign a bit in the collision mask.
      const int max_color =
          std::min(kNumCliquesPerColoring - 1, MaxColorValue(coloring));

      // If this is the first coloring, then make sure that we don't have so
      // many colors that would exceed the default set in gazebo. This
      // guarantees that we will always collide with the default set.
      static_assert(absl::has_single_bit(kDefaultCollisionBitMask + 1),
                    "Default bit mask should be of the form (2^n - 1)");
      if (next_bit == 0 &&
          max_color >= absl::bit_width(kDefaultCollisionBitMask + 1) + 1) {
        return intrinsic::InternalErrorBuilder()
               << "Initial coloring generated " << max_color + 1 << " colors, "
               << "which is more than the bit-width of the default collision "
               << "mask (" << absl::bit_width(kDefaultCollisionBitMask + 1) + 1
               << ")";
      }

      // If we have more colorings than bits available, then we've done as much
      // as we can, and it's time to bail.
      if (next_bit + max_color >= kNumBitsInCollisionMask) {
        break;
      }

      // Assign bits based on this coloring.
      for (const auto& exclusion_node : exclusion_graph) {
        // If the set of edges for this node in the exclusion graph is empty,
        // then it should use the default bitmask.
        if (exclusion_node.second.empty()) continue;

        int node_color = coloring[exclusion_node.first];
        uint32_t& collision_bitmask = collision_bitmasks[exclusion_node.first];
        if (node_color < kNumCliquesPerColoring) {
          collision_bitmask |= 1 << (next_bit + node_color);
        }
      }
      next_bit += kNumCliquesPerColoring;

      // Remove any edges from the collision graph that we've covered.
      for (auto& object : collision_graph) {
        const uint32_t object_id = object.first;
        absl::flat_hash_set<uint32_t>& neighbors = object.second;
        for (const auto& other : collision_graph) {
          const uint32_t other_id = other.first;
          if (collision_bitmasks[object_id] & collision_bitmasks[other_id]) {
            neighbors.erase(other_id);
          }
        }
      }

      // Are we done? We've succeeded if the collision graph has no more edges
      // to cover.
      if (std::all_of(collision_graph.begin(), collision_graph.end(),
                      [](const auto& p) { return p.second.empty(); })) {
        break;
      }

      // If we're not done, set the preconditioning for the next iteration.
      // Since we're looking for cliques in the collision graph, which are
      // denoted as colorings in the exclusion graph, we can seed the coloring
      // with vertices that we know are connected, but whose edges didn't cover
      // in this iteration. This is not necessarily exact, since neighbors to
      // a vertex in the collision graph may be neighbors themselves in the
      // exclusion graph (but will receive the same color).
      for (const uint32_t v : sorted_objects) {
        precondition[v] = -1;
      }
      for (const uint32_t v : sorted_objects) {
        // Don't supply precondition for empty collision sets.
        if (collision_graph[v].empty()) continue;

        // If we've already preconditioned this vertex, then skip it.
        if (precondition[v] >= 0) continue;

        precondition[v] = rng() % kNumCliquesPerColoring;
        auto neighbor_itr = collision_graph[v].begin();
        for (int i = 0; i < kNumNeighborsToPrecondition &&
                        neighbor_itr != collision_graph[v].end();
             ++i) {
          uint32_t neighbor = *(neighbor_itr++);
          if (precondition[neighbor] < 0) {
            precondition[neighbor] = precondition[v];
          }
        }
      }
    }

    // We've succeeded if the collision graph has no more edges to cover.
    if (std::all_of(collision_graph.begin(), collision_graph.end(),
                    [](const auto& p) { return p.second.empty(); })) {
      return collision_bitmasks;
    }
  }

  return intrinsic::InternalErrorBuilder()
         << "Failed to generate collision bitmasks after " << kNumLabelingTries
         << " tries";
}

}  // namespace simulation
}  // namespace intrinsic
