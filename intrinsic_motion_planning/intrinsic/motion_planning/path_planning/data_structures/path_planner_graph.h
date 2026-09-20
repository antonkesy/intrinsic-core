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

#ifndef INTRINSIC_MOTION_PLANNING_PATH_PLANNING_DATA_STRUCTURES_PATH_PLANNER_GRAPH_H_
#define INTRINSIC_MOTION_PLANNING_PATH_PLANNING_DATA_STRUCTURES_PATH_PLANNER_GRAPH_H_

#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <utility>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/types/optional.h"
#include "boost/graph/adjacency_list.hpp"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/motion_planning/path_planning/data_structures/nearest_neighbor.h"
#include "intrinsic/motion_planning/path_planning/data_structures/planner_graph.pb.h"
#include "intrinsic/motion_planning/path_planning/path_planner_definitions.h"
#include "intrinsic/motion_planning/path_planning/robot_chain.h"

namespace intrinsic {
// Minimal graph class that is used to encapsulate the planning graph for the
// Probabilistic Road Map and operation on the graph. The graph can be
// extended later to store collision data for future planning.
// Typical use is to create an (initial) road map or extend it to refine the
// planning effort. To allow such operations the graph contains simple
// edge and vertex operations, as well as graph extensions and search
// operations.
class PathPlannerGraph {
 public:
  // Definition of the information that a vertex encapsulates.
  struct Vertex {
    // For the internal use of the chosen graph representation (i.e.,
    // boost::adjacency_list). Applying a search algorithm on the boost graph
    // requires the definition of a vertex index that numerates the graph
    // vertex.
    mutable int64_t vertex_index;

    // The joint configuration that defines a vertex.
    eigenmath::VectorXd configuration;

    // Vertex validation information.
    proto::PathPlannerGraph::Validity validity;

    bool operator==(const Vertex& other) {
      return configuration.isApprox(other.configuration);
    }
  };

  struct Edge {
    // The 'start' vertex index of the two vertices that are connected through
    // the edge.
    int64_t start_vertex;
    // The second or 'end' vertex index of the two vertices that are connected
    // through the edge.
    int64_t end_vertex;
    // The cost or weight of an edge. This will be defined by a user provided
    // cost function. Used for path planning and optimization.
    double cost;

    // Validation information of the edge.
    proto::PathPlannerGraph::Validity validity;
  };

  // The boost graph definition is chosen to support all existing path planner
  // implementations. The choice of OutEdgeList and VertexList has mostly
  // run time implications. However, the choice of vecS for both OutEdgeList
  // and Vertex list is crutial for the functionality of the algorithms.
  typedef boost::adjacency_list<boost::vecS,         // OutEdgeList
                                boost::vecS,         // VertexList
                                boost::undirectedS,  // for walking in edges
                                Vertex,              // vertex property
                                Edge>                // edge property
      Graph;

  // Shortcuts for often used graph structures.
  using GraphTraits = boost::graph_traits<Graph>;
  using VertexIterator = GraphTraits::vertex_iterator;
  using VertexDescriptor = GraphTraits::vertex_descriptor;
  using EdgeDescriptor = GraphTraits::edge_descriptor;
  using EdgeIterator = GraphTraits::edge_iterator;
  using AdjacencyIterator = GraphTraits::adjacency_iterator;

  // Definition of a cost function that defines the cost/weight of an edge.
  // Set upon construction.
  using EdgeCostFn =
      std::function<double(const VertexDescriptor&, const VertexDescriptor&)>;

  // The PathPlannerGraph requires the specification of PathPlannerGraphConfig,
  // the identifier of the robot for which the graph is generated and a distance
  // measure between two robot configurations. The distance function is used
  // for nearest neighbor measurements and the computation of the edge cost.
  PathPlannerGraph(const proto::PathPlannerGraphConfig& config,
                   const RobotChain& base_tip_ids,
                   const DistanceFn& distance_fn);

  // Creates a PathPlannerGraph from a graph_proto, the specification of a
  // PathPlannerGraphConfig, and a distance function between two robot
  // configurations.
  PathPlannerGraph(const proto::PathPlannerGraphConfig& config,
                   const proto::PathPlannerGraph& graph_proto,
                   const DistanceFn& distance_fn);

  PathPlannerGraph(const PathPlannerGraph& other);
  PathPlannerGraph& operator=(PathPlannerGraph&& other) = default;

  // Adds a vertex to the internal graph representation and adds the
  // configuration to the nearest neighbor data structure.
  absl::StatusOr<VertexDescriptor> AddVertex(
      const eigenmath::VectorXd& configuration,
      const proto::PathPlannerGraph::Validity& validity =
          proto::PathPlannerGraph::UNTESTED);

  // Adds an edge between two graph vertices in the graph. The two vertex
  // descriptors needs to belong to the internal graph representation.
  // The validity of the edge defines if it was tested for valididty or not.
  absl::Status AddEdge(const VertexDescriptor& vertex1,
                       const VertexDescriptor& vertex2,
                       const proto::PathPlannerGraph::Validity& validity);

  // Checks if a vertex exist in the graph or not. The function returns a pair
  // consisting of the VertexDescriptor that corresponds to the found vertex
  // in the graph and a bool that defines if the vertex was found in the graph
  // and if the returned VertexDescriptor is valid.
  std::pair<VertexDescriptor, bool> ExistInGraph(
      const eigenmath::VectorXd& configuration) const;

  // Checks if a vertex associated with the VertexDescriptor exist in the graph
  // or not.
  bool ExistInGraph(const VertexDescriptor& vertex) const;

  // Gets the nearest neighbors of a given vertex in the graph. It expects as
  // input a VertexDescriptor of a vertex in the path planner graph. The
  // VertexDescriptor must be a valid vertex descriptor of the graph.
  absl::StatusOr<std::vector<VertexDescriptor>> GetNearestNeighbors(
      const VertexDescriptor& vertex) const;

  // Gets the nearest neighbor of a given vertex in the graph that is defined
  // by the VertexDescriptor. The function expects as input a valid
  // VertexDescriptor, i.e., it must exist in the graph otherwise it will
  // return an error status.
  absl::StatusOr<VertexDescriptor> GetNearestNeighbor(
      const VertexDescriptor& vertex) const;

  // Gets the nearest neighbors of a configuration. The result is a vector of
  // VertexDescriptors. If the configuration is already part of the path
  // planning graph, the set of nearest neighbors will contain also the vertex
  // with the same configuration. In this case it should be preferred to call
  // GetNearestNeighbor with the VertexDescriptor of the corresponding vertex.
  absl::StatusOr<std::vector<VertexDescriptor>> GetNearestNeighborsForConfig(
      const eigenmath::VectorXd& configuration) const;

  // Gets the nearest neighbor of a configuration, using the HNS (Hierarchical
  // Neighbor Search) approach. This approach calls GetKNearestNeighbors() and
  // then runs a neural network on these K neighbors, choosing the neighbor with
  // the smallest 'distance' outputted from the network.
  absl::StatusOr<VertexDescriptor> GetNearestNeighborHNS(
      const eigenmath::VectorXd& configuration) const;

  // Gets the nearest neighbor of a configuration. The result is a
  // VertexDescriptor of the vertex in the graph that contains the
  // configuration that is closest to the given configuration. If the
  // configuration is already in the graph or will be defenitely be added to the
  // graph consider to first add the graph and then use the functionality of
  // GetNearestNeighbor(const VertexDescriptor&) instead.
  absl::StatusOr<VertexDescriptor> GetNearestNeighborForConfig(
      const eigenmath::VectorXd& configuration) const;

  // Get list of adjacent vertices of vertex, i.e., all vertices that vertex is
  // connected with via an edge. The method only tests the existence of an edge
  // not the status of the edge.
  std::vector<VertexDescriptor> GetAdjacentVertices(
      const VertexDescriptor& vertex) const;

  // Runs an a-star graph search algorithm on the graph to find a path between
  // two vertices that are associated with the input VertexDescriptors. Will
  // return an empty path if not path was found.
  std::vector<VertexDescriptor> AStarSearch(const VertexDescriptor& start,
                                            const VertexDescriptor& stop) const;

  // Returns the number of vertices in the graph.
  size_t Size() const;

  // Gets a vector of vertex descriptors from all vertices in the graph.
  std::vector<VertexDescriptor> GetVertexDescriptors() const;

  // Returns the validity of an edge. Returns an InvalidArgumentError if the
  // vertices start and end do not exist in the graph. Returns a NotFoundError
  // if the edge does not exist in the graph.
  absl::StatusOr<proto::PathPlannerGraph::Validity> GetEdgeValidity(
      const VertexDescriptor& start, const VertexDescriptor& end) const;

  // Gets a vector of all edges contained in the graph;
  std::vector<Edge> GetEdges() const;

  // Gets the edge defined by the two vertex descriptors. Returns an
  // InvalidInputArgument error if the vertices do not exist in the graph and a
  // NotFound error if the edge between two edges does not exist.
  absl::StatusOr<Edge> GetEdge(const VertexDescriptor& start,
                               const VertexDescriptor& end) const;

  // Sets the validity status of the edge defined by the two vertices that
  // define the end-points of the edge. The graph is not direction sensitive,
  // i.e., the order of start and end do not matter.
  absl::Status SetEdgeValidity(
      const VertexDescriptor& start, const VertexDescriptor& end,
      const proto::PathPlannerGraph::Validity& validity);

  // Gets the data (i.e., joint configuration) associated with a vertex
  // descriptor if it exist in the internal graph structure.
  absl::StatusOr<eigenmath::VectorXd> GetConfiguration(
      const VertexDescriptor& vertex_descriptor) const;

  // Gets the full vertex information associated with the vertex descriptor.
  absl::StatusOr<Vertex> GetVertex(
      const VertexDescriptor& vertex_descriptor) const;

  // Converts a vector of vertex descriptors to a vector of configurations
  // that are associated with the vertex descriptor.
  absl::StatusOr<std::vector<eigenmath::VectorXd>> GetConfigurationVector(
      const std::vector<VertexDescriptor>& vertices) const;

  // Resets the graph by clearing the data structure and resetting the robot id
  // associated with this graph.
  absl::Status ResetGraph(std::optional<RobotChain> base_tip_ids);

  // Removes an edge from the graph if it exists. Returns an
  // InvalidArgumentError if the start and vertices defining the edge do not
  // exist.
  absl::Status RemoveEdge(const VertexDescriptor& start,
                          const VertexDescriptor& end);

  // Converts the graph to a proto::PathPlannerGraph.
  absl::StatusOr<proto::PathPlannerGraph> ToProto();

  // Gets the base and tip ids for which the graph was generated.
  RobotChain GetRobotChain() const;

  // Returns the connected component of the planner graph that contains root.
  // A vertex is considered connected with root, if there exist a path with
  // edges that have fulfill the status criteria defined in valid_edge_status.
  absl::StatusOr<PathPlannerGraph> GetConnectedGraphWithEdgeProperty(
      const VertexDescriptor& root,
      const std::vector<proto::PathPlannerGraph::Validity>& valid_edge_status)
      const;

  // Checks if a vertex with the given configuration is in the path planner
  // graph and returns the connected component graph that contains the
  // configuration as vertex.
  // A vertex is considered connected with root, if there exist a path with
  // edges that fullfills the status criteria defined in valid_edge_status.
  absl::StatusOr<PathPlannerGraph> GetConnectedGraphWithEdgeProperty(
      const eigenmath::VectorXd& config,
      const std::vector<proto::PathPlannerGraph::Validity>& valid_edge_status)
      const;

  // Returns the connected component of the path planner graph that contains
  // the vertex root as a tree structure, i.e., there are no circles with path
  // planner graph.
  // A vertex is considered connected with root, if there exist a path with
  // edges that fullfills the status criteria defined in valid_edge_status.
  absl::StatusOr<PathPlannerGraph> GetConnectedTree(
      const VertexDescriptor& root,
      const std::vector<proto::PathPlannerGraph::Validity>& valid_edge_status,
      bool distance_optimized) const;

  // Checks if a vertex with the given configuration is in the path planner
  // graph and returns the connected component of the path planner graph that
  // contains the vertex root. The connected component has a tree structure,
  // i.e., there are no circles with path planner graph.
  // A vertex is considered connected with root, if there exist a path with
  // edges that fullfills the status criteria defined in valid_edge_status.
  // This method is a shortcut for GetFilteredConnectedComponent with
  // tree = true.
  absl::StatusOr<PathPlannerGraph> GetConnectedTree(
      const eigenmath::VectorXd& config,
      const std::vector<proto::PathPlannerGraph::Validity>& valid_edge_status,
      bool distance_optimized) const;

  // Merges the merge_from graph with this path planner graph. The method
  // checks for each of the vertices and edges of the target_graph if they exist
  //  within this graph and if their properties were updated. We assume that the
  // properties will be only updated from untested to valid/collision but not
  // the other way around.
  absl::Status MergeGraph(const PathPlannerGraph& merge_from);

  // Copies the graph into this path planner graph. This operation does not
  // check if the vertices and edges already exist in the graph with the
  // exception of those defined within possible_common_vertices.
  absl::Status CopyGraph(
      const PathPlannerGraph& copy_graph,
      const std::vector<VertexDescriptor>& possible_common_vertices);

  // Rebuilds the internal vertex index of the set of vertices. This operation
  // is required for some boost::graph operations.
  void BuildVertexIndexMap() const;

  // Adds a vector of configurations to a graph. It is assumed that all
  // configurations have an edge with validity status `valid_edge_status` if
  // they are next to each other in the vector.
  absl::Status AddConnectedVectorToGraph(
      const std::vector<eigenmath::VectorXd>& valid_configurations,
      proto::PathPlannerGraph::Validity valid_edge_status);

  // Computes the lower and upper bound of all the values in the graph for each
  // degree of freedom independently.
  absl::StatusOr<std::pair<eigenmath::VectorXd, eigenmath::VectorXd>>
  GetLowerAndUpperValueBounds() const;

 private:
  // Reads vertices and edges from a proto file and adds them to the graph
  // infrastructure. The cost of the edges will be recomputed with the
  // distance function defined for this graph. Validity will be assumed is the
  // same.
  absl::Status FromProto(const proto::PathPlannerGraph& graph_proto);

  // The way we organize the path planner
  // graph into a graph and separate nearest neighbor structure makes it
  // necessary to keep a separate data strcuture for the query_point for
  // the case that the queried data point is not part of the graph.
  struct NearestNeighborData {
    // Nearest neighbor data structure.
    std::unique_ptr<NearestNeighbors<VertexDescriptor>> nearest_neighbor;

    // Nearest neighbor query point. Will be set for nearest neighbor requests
    // of configurations that are not in the graph.
    mutable eigenmath::VectorXd query_point;
  };

  // Returns a partial copy of the path planner graph, consisting of all the
  // vertices (and their connections) that are connected with the vertex
  // defined by the vertex descriptor root. A vertex is considered connected
  // to the root if there exist a path from root to the vertex and the edges
  // have one of the validity properties defined in valid_edge_status. If the
  // parameter tree is set true, the returned planner graph has a tree
  // structure, i.e., there are no circles in the graph. All other edges are
  // discarded.
  // If distance_optimization is set to true, the tree is constructed such
  // that each existing connection from the root to any other connected vertex
  // corresponds to the shortest possible connection in the existing graph.
  absl::StatusOr<PathPlannerGraph> GetFilteredConnectedComponent(
      const VertexDescriptor& root,
      const std::vector<proto::PathPlannerGraph::Validity>& valid_edge_status,
      bool tree, bool distance_optimized) const;

  // Gets an edge from vertex descriptors.
  absl::StatusOr<EdgeDescriptor> GetEdgeDescriptor(
      const VertexDescriptor& start, const VertexDescriptor& end) const;

  // Removes an edge from the graph if it exists. It is assumed that the edge
  // exist in the graph.
  void RemoveEdge(const EdgeDescriptor& edge);

  // Computs the cost of a path defined by the vector of VertexDescriptors.
  double ComputePathCost(const std::vector<VertexDescriptor>& path) const;

  Graph graph_;

  // Vertex existence map.
  absl::flat_hash_map<VertexDescriptor, bool> vertex_existence_map_;

  // Dimensionality of the vertex data in the graph.
  int vertex_data_dimension_;

  // Nearest neighbor data structure.
  NearestNeighborData nearest_neighbor_data_;

  // Save cost function to judge distance of edges.
  EdgeCostFn cost_fn_;

  // Save the distance function currently used.
  DistanceFn distance_fn_;

  // Chain for which this planning graph was created.
  RobotChain base_tip_ids_;

  // Not existing graph vertex as defined by boost::graph_traits.
  VertexDescriptor null_vertex_;

  // Graph configuration
  proto::PathPlannerGraphConfig config_;
};

proto::PathPlannerGraphConfig GetDefaultPathPlannerGraphConfig();
}  // namespace intrinsic

#endif  // INTRINSIC_MOTION_PLANNING_PATH_PLANNING_DATA_STRUCTURES_PATH_PLANNER_GRAPH_H_
