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

#include "intrinsic/motion_planning/path_planning/data_structures/path_planner_graph.h"

#include <algorithm>
#include <cstddef>
#include <optional>
#include <queue>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "absl/flags/flag.h"
#include "absl/log/check.h"
#include "absl/log/log.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_format.h"
#include "boost/graph/astar_search.hpp"
#include "boost/graph/lookup_edge.hpp"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/icon/proto/joint_space.pb.h"
#include "intrinsic/messages/geometry_types.pb.h"
#include "intrinsic/motion_planning/path_planning/data_structures/nearest_neighbor_configs.pb.h"
#include "intrinsic/motion_planning/path_planning/data_structures/nearest_neighbor_utils.h"
#include "intrinsic/motion_planning/path_planning/data_structures/planner_graph.pb.h"
#include "intrinsic/motion_planning/path_planning/path_planner_definitions.h"
#include "intrinsic/motion_planning/path_planning/robot_chain.h"
#include "intrinsic/util/eigen.h"
#include "intrinsic/util/macros.h"
#include "intrinsic/util/proto/get_text_proto.h"
#include "intrinsic/util/proto/parse_text_proto.h"
#include "intrinsic/util/status/status_builder.h"
#include "intrinsic/util/status/status_macros.h"

ABSL_FLAG(std::string, path_planner_graph_config_path, "",
          "Path to a PathPlannerGraphConfig pbtxt.");

namespace intrinsic {
namespace {
constexpr char kDefaultPathPlannerGraphConfig[] = R"(
    nearest_neighbor_spec {
      name: "LinearNearestNeighbor"
      config {
        [type.googleapis.com/intrinsic.proto.NearestNeighborConfig] {
          k: 5
          radius: 0.8
          strategy: KNEAREST
        }
      }
    }
  )";

// Exception for termination of a star. Required by graph search algorithm.
struct AStarFoundGoal {};

// Class that defines operations for examining the graph vertices during the
// boost graph search algorithms.
class AStarGoalVisitor : public boost::default_astar_visitor {
 public:
  explicit AStarGoalVisitor(const PathPlannerGraph::VertexDescriptor& goal)
      : goal_(goal) {}

  // Boost requires the definition of a function with name examine_vertex.
  // Cannot mark it as override.
  void examine_vertex(const PathPlannerGraph::VertexDescriptor& u,
                      const PathPlannerGraph::Graph& /*unused*/) const {
    if (u == goal_) {
      throw AStarFoundGoal();
    }
  }

 private:
  PathPlannerGraph::VertexDescriptor goal_;
};
}  // namespace

PathPlannerGraph::PathPlannerGraph(const proto::PathPlannerGraphConfig& config,
                                   const RobotChain& base_tip_ids,
                                   const DistanceFn& distance_fn)
    : vertex_data_dimension_(0),
      cost_fn_(nullptr),
      distance_fn_(distance_fn),
      base_tip_ids_(base_tip_ids),
      null_vertex_(GraphTraits::null_vertex()),
      config_(config) {
  auto vertex_distance_fn = [this, distance_fn](
                                const VertexDescriptor& vd1,
                                const VertexDescriptor& vd2) -> double {
    // Make sure that both descriptors are valid graph vertex descriptor (i.e.,
    //  >=0).
    CHECK_GE(vd1, 0);
    CHECK_GE(vd2, 0);
    if (vd1 == null_vertex_) {
      return distance_fn(nearest_neighbor_data_.query_point,
                         graph_[vd2].configuration);
    }
    if (vd2 == null_vertex_) {
      return distance_fn(graph_[vd1].configuration,
                         nearest_neighbor_data_.query_point);
    }
    return distance_fn(graph_[vd1].configuration, graph_[vd2].configuration);
  };

  nearest_neighbor_data_.nearest_neighbor =
      CreateNearestNeighbor<VertexDescriptor>(config.nearest_neighbor_spec())
          .value();
  CHECK_OK(nearest_neighbor_data_.nearest_neighbor->SetDistanceFunction(
      vertex_distance_fn));

  cost_fn_ = vertex_distance_fn;

  vertex_existence_map_.clear();

  // We need to load the network here if the 'HNS' strategy is called.
  if (nearest_neighbor_data_.nearest_neighbor->GetStrategy() ==
      proto::NearestNeighborConfig::HNS) {
    // TODO(b/185879851): Call LoadModelBundle from
    // intrinsic/motion_planning/path_planning/data_structures/hns_planner_graph.h
    // Set the model directory
    LOG(FATAL) << "Currently not supported: see b/185879851";
  }
}

PathPlannerGraph::PathPlannerGraph(const proto::PathPlannerGraphConfig& config,
                                   const proto::PathPlannerGraph& graph_proto,
                                   const DistanceFn& distance_fn)
    : PathPlannerGraph(config,
                       RobotChain(graph_proto.robot().base_id(),
                                  graph_proto.robot().tip_id()),
                       distance_fn) {
  CHECK_OK(FromProto(graph_proto));
}

PathPlannerGraph::PathPlannerGraph(const PathPlannerGraph& other)
    : PathPlannerGraph(other.config_, other.base_tip_ids_, other.distance_fn_) {
  graph_ = other.graph_;
  vertex_existence_map_ = other.vertex_existence_map_;
  vertex_data_dimension_ = other.vertex_data_dimension_;

  // Rebuild nearest neighbor data structure
  VertexIterator vertex_iterator;
  VertexIterator vertex_end;
  for (std::tie(vertex_iterator, vertex_end) = boost::vertices(graph_);
       vertex_iterator != vertex_end; ++vertex_iterator) {
    CHECK_OK(
        nearest_neighbor_data_.nearest_neighbor->AddElement(*vertex_iterator));
  }
}

std::pair<PathPlannerGraph::VertexDescriptor, bool>
PathPlannerGraph::ExistInGraph(const eigenmath::VectorXd& configuration) const {
  VertexIterator vertex_iterator;
  VertexIterator vertex_end;
  bool found = false;

  // Quick check if configuration dimensionality is correct
  if (configuration.size() != vertex_data_dimension_) {
    return std::pair<VertexDescriptor, bool>(0, false);
  }

  // Go through graph and check if the configuration is the same.
  for (std::tie(vertex_iterator, vertex_end) = boost::vertices(graph_);
       !found && vertex_iterator != vertex_end; ++vertex_iterator) {
    found = ((graph_[*vertex_iterator].configuration).isApprox(configuration));
  }
  --vertex_iterator;
  return std::pair<VertexDescriptor, bool>(*vertex_iterator, found);
}

bool PathPlannerGraph::ExistInGraph(const VertexDescriptor& vertex) const {
  auto in_map = vertex_existence_map_.find(vertex);
  if (in_map == vertex_existence_map_.end()) {
    return false;
  }
  return in_map->second;
}

absl::StatusOr<PathPlannerGraph::VertexDescriptor> PathPlannerGraph::AddVertex(
    const eigenmath::VectorXd& configuration,
    const proto::PathPlannerGraph::Validity& validity) {
  // Set dimensionality of data if we add the first element.
  if (Size() < 1) {
    vertex_data_dimension_ = configuration.size();
  }

  // Check size of vector.
  if (configuration.size() != vertex_data_dimension_) {
    return absl::InvalidArgumentError(
        absl::StrFormat("Provided vector data should have dimensionality of "
                        "%d. Input vector has dimensionality of %d.",
                        vertex_data_dimension_, configuration.size()));
  }

  // Add to graph.
  Vertex vertex;
  vertex.vertex_index = Size();
  vertex.configuration = configuration;
  vertex.validity = validity;
  VertexDescriptor v_descriptor = boost::add_vertex(vertex, graph_);

  // Add vertex to existence map.
  vertex_existence_map_[v_descriptor] = true;

  // Add to nearest neighbor data structure.
  INTR_RETURN_IF_ERROR(
      nearest_neighbor_data_.nearest_neighbor->AddElement(v_descriptor));

  return v_descriptor;
}

absl::Status PathPlannerGraph::AddEdge(
    const PathPlannerGraph::VertexDescriptor& vertex1,
    const PathPlannerGraph::VertexDescriptor& vertex2,
    const proto::PathPlannerGraph::Validity& validity) {
  // Check if vertices exist in graph.
  if (!vertex_existence_map_[vertex1] || !vertex_existence_map_[vertex2]) {
    return absl::InvalidArgumentError(
        "Provided vertex descriptor is not valid.");
  }
  // Add edge.
  auto edge = boost::add_edge(vertex1, vertex2, graph_);
  if (!edge.second) {
    // Edge was not added to the graph.
    return absl::InternalError("Could not add edge to planner graph.");
  }
  auto edge_descriptor = edge.first;
  graph_[edge_descriptor].validity = validity;
  graph_[edge_descriptor].cost = cost_fn_(vertex1, vertex2);
  graph_[edge_descriptor].start_vertex = vertex1;
  graph_[edge_descriptor].end_vertex = vertex2;

  // It is assumed, that if the edge between two vertices is valid, so are the
  // vertices themselves.
  // Set the vertex status to ensure consistency in the graph.
  if (validity == proto::PathPlannerGraph::VALID) {
    graph_[vertex1].validity = proto::PathPlannerGraph::VALID;
    graph_[vertex2].validity = proto::PathPlannerGraph::VALID;
  }

  return absl::OkStatus();
}

absl::StatusOr<std::vector<PathPlannerGraph::VertexDescriptor>>
PathPlannerGraph::GetNearestNeighbors(const VertexDescriptor& vertex) const {
  if (!ExistInGraph(vertex)) {
    return absl::InvalidArgumentError("Vertex does not exist in graph");
  }

  // Care needs to be taken when getting the nearest neighbors as the vertex
  // for which we request the nearest neighbors will also contain the vertex
  // itself. We therefore have to remove this nearest neighbor and have to
  // request an additional nearest neighbor.
  std::vector<VertexDescriptor> result;
  switch (nearest_neighbor_data_.nearest_neighbor->GetStrategy()) {
    case proto::NearestNeighborConfig::NEAREST: {
      INTR_ASSIGN_OR_RETURN(auto neighbor, GetNearestNeighbor(vertex));
      result.push_back(neighbor);
      return result;
    }
    case proto::NearestNeighborConfig::HNS:
    case proto::NearestNeighborConfig::KNEAREST: {
      INTR_ASSIGN_OR_RETURN(
          result,
          nearest_neighbor_data_.nearest_neighbor->GetKNearestNeighbors(
              vertex, nearest_neighbor_data_.nearest_neighbor->GetK() + 1));
      break;
    }
    default:
      INTR_ASSIGN_OR_RETURN(
          result,
          nearest_neighbor_data_.nearest_neighbor->GetNearestNeighbors(vertex));
  }

  for (int i = 0; i < result.size(); ++i) {
    if (result[i] == vertex) {
      result.erase(result.begin() + i);
      return result;
    }
  }
  return result;
}

absl::StatusOr<PathPlannerGraph::VertexDescriptor>
PathPlannerGraph::GetNearestNeighbor(const VertexDescriptor& vertex) const {
  if (!ExistInGraph(vertex)) {
    return absl::InvalidArgumentError("Vertex does not exist in graph");
  }

  // The vertex is already in the data structure. It therefore will be returned
  // as well. To return the true nearest neighbor, we need to look for the
  // two closest neighbors in the graph.
  constexpr int kNumberNeighbors = 2;
  INTR_ASSIGN_OR_RETURN(
      auto result,
      nearest_neighbor_data_.nearest_neighbor->GetKNearestNeighbors(
          vertex, kNumberNeighbors));
  for (auto neighbor : result) {
    if (neighbor != vertex) {
      return neighbor;
    }
  }

  return absl::NotFoundError("No nearest neighbor found in path planner graph");
}

absl::StatusOr<std::vector<PathPlannerGraph::VertexDescriptor>>
PathPlannerGraph::GetNearestNeighborsForConfig(
    const eigenmath::VectorXd& configuration) const {
  nearest_neighbor_data_.query_point = configuration;
  // A descriptor == null_vertex_ indicates the use of the query point
  // instead of the graph to infer the configuration we are looking for.
  return nearest_neighbor_data_.nearest_neighbor->GetNearestNeighbors(
      null_vertex_);
}

absl::StatusOr<PathPlannerGraph::VertexDescriptor>
PathPlannerGraph::GetNearestNeighborForConfig(
    const eigenmath::VectorXd& configuration) const {
  // If specified, run using HNS.
  if (nearest_neighbor_data_.nearest_neighbor->GetStrategy() ==
      proto::NearestNeighborConfig::HNS) {
    // TODO(b/185879851): Call GetNearestNeighborHNS from
    // intrinsic/motion_planning/path_planning/data_structures/hns_planner_graph.h
    // Set the model directory
    LOG(FATAL) << "Currently not supported: see b/185879851";
  }

  // Otherwise get the nearest neighbor as usual.
  // A descriptor == null_vertex_ indicates the use of the query point
  // instead of the graph to infer the configuration we are looking for.
  nearest_neighbor_data_.query_point = configuration;

  return nearest_neighbor_data_.nearest_neighbor->GetNearestNeighbor(
      null_vertex_);
}

std::vector<PathPlannerGraph::VertexDescriptor> PathPlannerGraph::AStarSearch(
    const VertexDescriptor& start, const VertexDescriptor& stop) const {
  std::vector<VertexDescriptor> result;
  std::vector<Graph::vertex_descriptor> predecessor(Size());
  std::vector<double> dist(Size());

  // Check if the vertices exist in graph.
  if (!ExistInGraph(start) || !ExistInGraph(stop)) {
    LOG(ERROR) << "Start or stop vertex does not exist in path planner graph. "
                  "No path between two vertices exist.";
    return result;
  }

  if (start == stop) {
    LOG(ERROR) << "Start and stop vertex are identical.";
    result.push_back(start);
    return result;
  }

  // Create index: needs to be 0 to N-1; otherwise it will crash.
  int index = 0;
  VertexIterator vertex_iterator;
  VertexIterator v_end;
  for (boost::tie(vertex_iterator, v_end) = boost::vertices(graph_);
       vertex_iterator != v_end; ++vertex_iterator, ++index) {
    graph_[*vertex_iterator].vertex_index = index;
  }

  // The a-star algorithm throws an exception if it finds a path.
  try {
    boost::astar_search(
        graph_, start,
        [this, stop](VertexDescriptor v) { return cost_fn_(v, stop); },
        boost::weight_map(get(&Edge::cost, graph_))
            .vertex_index_map(get(&Vertex::vertex_index, graph_))
            .predecessor_map(&predecessor[0])
            .visitor(AStarGoalVisitor(stop))
            .distance_map(&dist[0]));
  } catch (AStarFoundGoal) {
    for (VertexDescriptor i = stop; i != predecessor[i]; i = predecessor[i]) {
      result.push_back(i);
      if (i == start) {
        break;
      }
    }
    result.push_back(start);
  }
  std::reverse(result.begin(), result.end());
  return result;
}

absl::Status PathPlannerGraph::RemoveEdge(
    const PathPlannerGraph::VertexDescriptor& start,
    const PathPlannerGraph::VertexDescriptor& end) {
  // Get Edge.
  INTR_ASSIGN_OR_RETURN(auto edge, GetEdgeDescriptor(start, end));

  // Remove edge
  RemoveEdge(edge);
  return absl::OkStatus();
}

absl::StatusOr<PathPlannerGraph::EdgeDescriptor>
PathPlannerGraph::GetEdgeDescriptor(const VertexDescriptor& start,
                                    const VertexDescriptor& end) const {
  // Check if vertices exist in graph.
  if (!ExistInGraph(start) || !ExistInGraph(end)) {
    return absl::InvalidArgumentError(
        "Vertex descriptors defining the end-points of the edge do not exist "
        "in PathPlanningGraph");
  }

  // Look up if an edge exist between those two.
  auto edge = boost::lookup_edge(start, end, graph_);
  if (!edge.second) {
    return absl::NotFoundError(
        "Attempting to access not existing edge in PathPlannerGraph.");
  }

  return edge.first;
}

absl::Status PathPlannerGraph::SetEdgeValidity(
    const VertexDescriptor& start, const VertexDescriptor& end,
    const proto::PathPlannerGraph::Validity& validity) {
  INTR_ASSIGN_OR_RETURN(auto edge, GetEdgeDescriptor(start, end));
  graph_[edge].validity = validity;
  return absl::OkStatus();
}

void PathPlannerGraph::RemoveEdge(
    const PathPlannerGraph::EdgeDescriptor& edge) {
  boost::remove_edge(edge, graph_);
}

// Searches the graph for the vertex descriptor and returns the configuration
// associated with this vertex as well as if it was found in the graph.
absl::StatusOr<eigenmath::VectorXd> PathPlannerGraph::GetConfiguration(
    const PathPlannerGraph::VertexDescriptor& vertex_descriptor) const {
  eigenmath::VectorXd config;
  // boost::adjacency_list has undefined behavior if vertex_descriptor
  // does not exist. ExistInGraph performs a check on a flat_hash_map to
  // confirm existence of the vertex_descriptor before finding the vertex in the
  // graph.
  if (!ExistInGraph(vertex_descriptor)) {
    return absl::InvalidArgumentError(
        "Vertex descriptor does not exist in PathPlanningGraph");
  }

  config = graph_[vertex_descriptor].configuration;
  return config;
}

absl::StatusOr<std::vector<eigenmath::VectorXd>>
PathPlannerGraph::GetConfigurationVector(
    const std::vector<VertexDescriptor>& vertices) const {
  std::vector<eigenmath::VectorXd> configurations(vertices.size());
  for (int i = 0; i < vertices.size(); ++i) {
    INTR_ASSIGN_OR_RETURN(configurations[i], GetConfiguration(vertices[i]));
  }
  return configurations;
}

size_t PathPlannerGraph::Size() const { return boost::num_vertices(graph_); }

absl::Status PathPlannerGraph::ResetGraph(
    std::optional<RobotChain> base_tip_ids) {
  INTR_RETURN_IF_ERROR(
      nearest_neighbor_data_.nearest_neighbor->ClearDataStructure());
  vertex_existence_map_.clear();
  graph_ = Graph();
  if (base_tip_ids.has_value()) {
    base_tip_ids_ = base_tip_ids.value();
  }
  return absl::OkStatus();
}

absl::StatusOr<proto::PathPlannerGraph> PathPlannerGraph::ToProto() {
  proto::PathPlannerGraph out;

  auto& robot = *out.mutable_robot();
  robot.set_base_id(base_tip_ids_.base_id);
  robot.set_tip_id(base_tip_ids_.tip_id);

  // First iterate through all the vertices in the graph.
  VertexIterator vertex_iterator;
  VertexIterator vertex_end;
  for (std::tie(vertex_iterator, vertex_end) = boost::vertices(graph_);
       vertex_iterator != vertex_end; ++vertex_iterator) {
    auto* vertex_proto = out.add_vertices();
    vertex_proto->set_id(*vertex_iterator);
    vertex_proto->set_validity(graph_[*vertex_iterator].validity);
    eigenmath::VectorXd vec = graph_[*vertex_iterator].configuration;
    VectorXdToRepeatedDouble(
        vec, vertex_proto->mutable_configuration()->mutable_joints());
  }

  // Now iterate through the edges of the graph
  EdgeIterator edge_iterator;
  EdgeIterator edge_end;
  for (std::tie(edge_iterator, edge_end) = boost::edges(graph_);
       edge_iterator != edge_end; ++edge_iterator) {
    auto* edge_proto = out.add_edges();
    edge_proto->set_start_vertex(graph_[*edge_iterator].start_vertex);
    edge_proto->set_end_vertex(graph_[*edge_iterator].end_vertex);
    edge_proto->set_cost(graph_[*edge_iterator].cost);
    edge_proto->set_validity(graph_[*edge_iterator].validity);
  }

  return out;
}

absl::Status PathPlannerGraph::FromProto(
    const proto::PathPlannerGraph& graph_proto) {
  // Only add the information in the proto if the robot ids match.
  if (base_tip_ids_.base_id != graph_proto.robot().base_id()) {
    return ::intrinsic::InvalidArgumentErrorBuilder()

           << "Base id in proto does not match robot id of planner graph.";
  }
  if (base_tip_ids_.tip_id != graph_proto.robot().tip_id()) {
    return ::intrinsic::InvalidArgumentErrorBuilder()
           << "Tip id in proto does not match robot id of planner graph.";
  }

  // Parse vertices. Map the saved vertex id to the new created vertex
  // descriptor to integrate the edges correctly.
  absl::flat_hash_map<int, VertexDescriptor> vertex_id_to_descriptor;
  eigenmath::VectorXd config;
  VertexDescriptor new_vertex;
  for (auto i = 0; i < graph_proto.vertices_size(); ++i) {
    const auto& vertex_proto = graph_proto.vertices(i);
    config = RepeatedDoubleToVectorXd(vertex_proto.configuration().joints());
    INTR_ASSIGN_OR_RETURN(new_vertex,
                          AddVertex(config, vertex_proto.validity()));
    vertex_id_to_descriptor[vertex_proto.id()] = new_vertex;
  }

  // Parse edges
  absl::flat_hash_map<int, VertexDescriptor>::const_iterator start_vertex;
  absl::flat_hash_map<int, VertexDescriptor>::const_iterator end_vertex;
  for (auto i = 0; i < graph_proto.edges_size(); ++i) {
    const auto& edge_proto = graph_proto.edges(i);
    start_vertex = vertex_id_to_descriptor.find(edge_proto.start_vertex());
    end_vertex = vertex_id_to_descriptor.find(edge_proto.end_vertex());
    if (start_vertex == vertex_id_to_descriptor.end() ||
        end_vertex == vertex_id_to_descriptor.end()) {
      return absl::InvalidArgumentError(
          "PathPlannerGraph proto contains an edge for undefined vertex.");
    }

    INTR_RETURN_IF_ERROR(AddEdge(start_vertex->second, end_vertex->second,
                                 edge_proto.validity()));
  }

  return absl::OkStatus();
}

std::vector<PathPlannerGraph::VertexDescriptor>
PathPlannerGraph::GetVertexDescriptors() const {
  std::vector<VertexDescriptor> out;
  out.reserve(Size());
  VertexIterator vertex_iterator;
  VertexIterator vertex_end;
  for (std::tie(vertex_iterator, vertex_end) = boost::vertices(graph_);
       vertex_iterator != vertex_end; ++vertex_iterator) {
    out.push_back(*vertex_iterator);
  }
  return out;
}

std::vector<PathPlannerGraph::Edge> PathPlannerGraph::GetEdges() const {
  std::vector<Edge> out;
  out.reserve(boost::num_edges(graph_));

  EdgeIterator edge_iterator;
  EdgeIterator edge_end;
  for (std::tie(edge_iterator, edge_end) = boost::edges(graph_);
       edge_iterator != edge_end; ++edge_iterator) {
    out.push_back(graph_[*edge_iterator]);
  }
  return out;
}

absl::StatusOr<proto::PathPlannerGraph::Validity>
PathPlannerGraph::GetEdgeValidity(const VertexDescriptor& start,
                                  const VertexDescriptor& end) const {
  INTR_ASSIGN_OR_RETURN(auto edge, GetEdgeDescriptor(start, end));
  return graph_[edge].validity;
}

absl::StatusOr<PathPlannerGraph::Vertex> PathPlannerGraph::GetVertex(
    const VertexDescriptor& vertex_descriptor) const {
  // boost::adjacency_list has undefined behavior if vertex_descriptor
  // does not exist. ExistInGraph performs a check on a flat_hash_map to
  // confirm existence of the vertex_descriptor before finding the vertex in the
  // graph.
  if (!ExistInGraph(vertex_descriptor)) {
    return absl::InvalidArgumentError(
        "Vertex descriptor does not exist in PathPlanningGraph");
  }

  return graph_[vertex_descriptor];
}

RobotChain PathPlannerGraph::GetRobotChain() const { return base_tip_ids_; }

absl::StatusOr<PathPlannerGraph>
PathPlannerGraph::GetConnectedGraphWithEdgeProperty(
    const VertexDescriptor& root,
    const std::vector<proto::PathPlannerGraph::Validity>& valid_edge_status)
    const {
  constexpr bool kTree = false;
  constexpr bool kOptimizeTree = false;
  return GetFilteredConnectedComponent(root, valid_edge_status, kTree,
                                       kOptimizeTree);
}

absl::StatusOr<PathPlannerGraph>
PathPlannerGraph::GetConnectedGraphWithEdgeProperty(
    const eigenmath::VectorXd& config,
    const std::vector<proto::PathPlannerGraph::Validity>& valid_edge_status)
    const {
  const std::pair<VertexDescriptor, bool> vertex = ExistInGraph(config);
  if (!vertex.second) {
    return absl::NotFoundError(
        "Configuration not found in path planner graph.");
  }
  return GetConnectedGraphWithEdgeProperty(vertex.first, valid_edge_status);
}

absl::StatusOr<PathPlannerGraph> PathPlannerGraph::GetConnectedTree(
    const VertexDescriptor& root,
    const std::vector<proto::PathPlannerGraph::Validity>& valid_edge_status,
    bool distance_optimized) const {
  constexpr bool kTree = true;
  return GetFilteredConnectedComponent(root, valid_edge_status, kTree,
                                       distance_optimized);
}

absl::StatusOr<PathPlannerGraph> PathPlannerGraph::GetConnectedTree(
    const eigenmath::VectorXd& config,
    const std::vector<proto::PathPlannerGraph::Validity>& valid_edge_status,
    bool distance_optimized) const {
  const std::pair<VertexDescriptor, bool> vertex = ExistInGraph(config);
  if (!vertex.second) {
    return absl::NotFoundError(
        "Configuration not found in path planner graph.");
  }
  return GetConnectedTree(vertex.first, valid_edge_status, distance_optimized);
}

absl::StatusOr<PathPlannerGraph>
PathPlannerGraph::GetFilteredConnectedComponent(
    const VertexDescriptor& root,
    const std::vector<proto::PathPlannerGraph::Validity>& valid_edge_status,
    bool tree, bool distance_optimized) const {
  // Check if the root of the connected component is contained
  if (!ExistInGraph(root)) {
    return absl::InvalidArgumentError(
        "Vertex descriptor does not exist in PathPlanningGraph");
  }

  // Create PathPlannerGraph with the same configuration as this graph and the
  // root as first vertex.
  PathPlannerGraph connected_component(config_, base_tip_ids_, distance_fn_);
  INTR_ASSIGN_OR_RETURN(auto new_vertex_id,
                        connected_component.AddVertex(
                            graph_[root].configuration, graph_[root].validity));

  std::queue<VertexDescriptor> vertex_queue;
  absl::flat_hash_map<VertexDescriptor, VertexDescriptor> visited;
  vertex_queue.push(root);
  visited[root] = new_vertex_id;

  // Find all components connected to root with breath search.
  while (!vertex_queue.empty()) {
    auto current_vertex = vertex_queue.front();
    vertex_queue.pop();

    for (auto adjacent_vertex : GetAdjacentVertices(current_vertex)) {
      auto edge = boost::lookup_edge(current_vertex, adjacent_vertex, graph_);
      if (!edge.second) continue;
      proto::PathPlannerGraph::Validity edge_validity =
          graph_[edge.first].validity;
      // Filter edge validity status.
      bool edge_is_valid = false;
      for (int i = 0; !edge_is_valid && i < valid_edge_status.size(); ++i) {
        edge_is_valid = (valid_edge_status[i] == edge_validity);
      }
      if (!edge_is_valid) continue;
      if (visited.find(adjacent_vertex) == visited.end()) {
        INTR_ASSIGN_OR_RETURN(
            new_vertex_id,
            connected_component.AddVertex(graph_[adjacent_vertex].configuration,
                                          graph_[adjacent_vertex].validity));
        visited[adjacent_vertex] = new_vertex_id;
        vertex_queue.push(adjacent_vertex);
      }
      // Add edge if it does not exist yet.
      auto edge_status = connected_component.GetEdge(visited[current_vertex],
                                                     visited[adjacent_vertex]);
      if (edge_status.ok()) continue;
      // Edge does not yet exist in the connected_component. If we do not want
      // to construct a tree or there exist no edge to this vertex yet, add a
      // component. Otherwise check if we can optimize the connection.
      if (tree &&
          !connected_component.GetAdjacentVertices(visited[adjacent_vertex])
               .empty()) {
        // There exist already a connection between root and the
        // adjacent_vertex.
        if (distance_optimized) {
          auto existing_path = connected_component.AStarSearch(
              visited[root], visited[adjacent_vertex]);
          auto new_path = connected_component.AStarSearch(
              visited[root], visited[current_vertex]);
          new_path.push_back(visited[adjacent_vertex]);
          if (ComputePathCost(existing_path) > ComputePathCost(new_path)) {
            INTR_RETURN_IF_ERROR(connected_component.RemoveEdge(
                existing_path.back(), existing_path[existing_path.size() - 2]));
            INTR_RETURN_IF_ERROR(connected_component.AddEdge(
                visited[current_vertex], visited[adjacent_vertex],
                edge_validity));
          }
        }
      } else {
        INTR_RETURN_IF_ERROR(connected_component.AddEdge(
            visited[current_vertex], visited[adjacent_vertex], edge_validity));
      }
    }
  }

  return connected_component;
}

double PathPlannerGraph::ComputePathCost(
    const std::vector<VertexDescriptor>& path) const {
  double cost = 0;

  for (int i = 1; i < path.size(); ++i) {
    cost += cost_fn_(path[i - 1], path[i]);
  }

  return cost;
}

std::vector<PathPlannerGraph::VertexDescriptor>
PathPlannerGraph::GetAdjacentVertices(const VertexDescriptor& vertex) const {
  std::vector<VertexDescriptor> adjacent_vertices;
  AdjacencyIterator adjacent_vertex_iter;
  AdjacencyIterator end;
  boost::tie(adjacent_vertex_iter, end) =
      boost::adjacent_vertices(vertex, graph_);
  for (; adjacent_vertex_iter != end; ++adjacent_vertex_iter) {
    adjacent_vertices.push_back(*adjacent_vertex_iter);
  }
  return adjacent_vertices;
}

absl::StatusOr<PathPlannerGraph::Edge> PathPlannerGraph::GetEdge(
    const VertexDescriptor& start, const VertexDescriptor& end) const {
  // Check if vertices exist in graph.
  if (!ExistInGraph(start) || !ExistInGraph(end)) {
    return absl::InvalidArgumentError(
        "Vertex descriptors defining the end-points of the edge do not exist "
        "in PathPlanningGraph");
  }

  // Look up if an edge exist between the two vertices.
  auto edge = boost::lookup_edge(start, end, graph_);
  if (!edge.second) {
    return absl::NotFoundError(
        "Attempting to access not existing edge in PathPlannerGraph.");
  }

  return graph_[edge.first];
}

void PathPlannerGraph::BuildVertexIndexMap() const {
  int index = 0;
  VertexIterator vertex_iterator;
  VertexIterator v_end;
  for (boost::tie(vertex_iterator, v_end) = boost::vertices(graph_);
       vertex_iterator != v_end; ++vertex_iterator, ++index) {
    graph_[*vertex_iterator].vertex_index = index;
  }
}

absl::Status PathPlannerGraph::MergeGraph(const PathPlannerGraph& merge_from) {
  // Merge vertices from merge_from into graph_
  auto vertices_from = merge_from.GetVertexDescriptors();
  Vertex vertex;
  absl::flat_hash_map<VertexDescriptor, VertexDescriptor> from_to_vertex_map;
  for (auto vertex_descriptor : vertices_from) {
    INTR_ASSIGN_OR_RETURN(vertex, merge_from.GetVertex(vertex_descriptor));
    auto exist_in_graph = ExistInGraph(vertex.configuration);
    if (!exist_in_graph.second) {
      INTR_ASSIGN_OR_RETURN(auto new_index,
                            AddVertex(vertex.configuration, vertex.validity));
      from_to_vertex_map[vertex_descriptor] = new_index;
    } else {
      if (graph_[exist_in_graph.first].validity < vertex.validity) {
        graph_[exist_in_graph.first].validity = vertex.validity;
      }
      from_to_vertex_map[vertex_descriptor] = exist_in_graph.first;
    }
  }

  // Merge edges.
  for (auto edge : merge_from.GetEdges()) {
    // Check if edge already exist in this graph:
    auto edge_exist =
        boost::lookup_edge(from_to_vertex_map[edge.start_vertex],
                           from_to_vertex_map[edge.end_vertex], graph_);
    if (!edge_exist.second) {
      INTR_RETURN_IF_ERROR(AddEdge(from_to_vertex_map[edge.start_vertex],
                                   from_to_vertex_map[edge.end_vertex],
                                   edge.validity));
    } else {
      // If edge exist, just check in its properties need to be updated.
      if (graph_[edge_exist.first].validity < edge.validity) {
        graph_[edge_exist.first].validity = edge.validity;
      }
    }
  }

  return absl::OkStatus();
}

absl::Status PathPlannerGraph::CopyGraph(
    const PathPlannerGraph& copy_graph,
    const std::vector<VertexDescriptor>& possible_common_vertices) {
  // Map vertex descriptor from copy_graph to this graph
  absl::flat_hash_map<VertexDescriptor, VertexDescriptor> visited;
  absl::flat_hash_map<VertexDescriptor, bool> common_vertices;
  for (auto vertex_descriptor : possible_common_vertices) {
    auto vertex = copy_graph.GetVertex(vertex_descriptor);
    if (!vertex.ok()) continue;
    auto vertex_exist = ExistInGraph(vertex.value().configuration);
    if (vertex_exist.second) {
      common_vertices[vertex_descriptor] = true;
      visited.emplace(vertex_descriptor, vertex_exist.first);
    }
  }

  // Copy vertices.
  Vertex vertex;
  auto verices_copy_graph = copy_graph.GetVertexDescriptors();
  for (auto vertex_descriptor : verices_copy_graph) {
    INTR_ASSIGN_OR_RETURN(vertex, copy_graph.GetVertex(vertex_descriptor));
    auto it = visited.find(vertex_descriptor);
    if (it == visited.end()) {
      INTR_ASSIGN_OR_RETURN(auto new_descriptor,
                            AddVertex(vertex.configuration, vertex.validity));
      visited.emplace(vertex_descriptor, new_descriptor);
    } else {
      // Check if the status needs to be updated.
      graph_[it->second].validity =
          std::max(graph_[it->second].validity, vertex.validity);
    }
  }

  // Copy edges.
  auto edges_copy_graph = copy_graph.GetEdges();
  for (auto edge : edges_copy_graph) {
    auto start_vertex = visited.find(edge.start_vertex);
    auto end_vertex = visited.find(edge.end_vertex);
    if (start_vertex == visited.end() || end_vertex == visited.end()) {
      return absl::InternalError("Edge from copy_graph contains invalid edge.");
    }

    if (!common_vertices[edge.start_vertex] ||
        !common_vertices[edge.end_vertex]) {
      INTR_RETURN_IF_ERROR(
          AddEdge(start_vertex->second, end_vertex->second, edge.validity));
    } else {
      // Check if we need to update the existing graph.
      auto current_edge = GetEdge(start_vertex->second, end_vertex->second);
      if (!current_edge.ok()) {
        INTR_RETURN_IF_ERROR(
            AddEdge(start_vertex->second, end_vertex->second, edge.validity));
      } else {
        if (current_edge.value().validity < edge.validity) {
          INTR_RETURN_IF_ERROR(SetEdgeValidity(
              start_vertex->second, end_vertex->second, edge.validity));
        }
      }
    }
  }

  return absl::OkStatus();
}

proto::PathPlannerGraphConfig GetDefaultPathPlannerGraphConfig() {
  static const proto::PathPlannerGraphConfig* config = []() {
    intrinsic::proto::PathPlannerGraphConfig* config =
        new proto::PathPlannerGraphConfig();

    std::string path_planner_graph_config_path =
        absl::GetFlag(FLAGS_path_planner_graph_config_path);

    if (path_planner_graph_config_path.empty()) {
      *config = ParseTextProtoOrDie(kDefaultPathPlannerGraphConfig);
    } else {
      CHECK_OK(
          intrinsic::GetTextProto(path_planner_graph_config_path, *config));
    }

    CHECK(config->has_nearest_neighbor_spec());
    return config;
  }();

  return *config;
}

absl::Status PathPlannerGraph::AddConnectedVectorToGraph(
    const std::vector<eigenmath::VectorXd>& valid_configurations,
    proto::PathPlannerGraph::Validity valid_edge_status) {
  VertexDescriptor last_vertex;
  bool add_edge = false;
  for (const eigenmath::VectorXd& config : valid_configurations) {
    INTR_ASSIGN_OR_RETURN(auto new_vertex,
                          AddVertex(config, valid_edge_status));
    if (add_edge) {
      INTR_RETURN_IF_ERROR(AddEdge(last_vertex, new_vertex, valid_edge_status));
      last_vertex = new_vertex;
    } else {
      add_edge = true;
      last_vertex = new_vertex;
    }
  }
  return absl::OkStatus();
}

absl::StatusOr<std::pair<eigenmath::VectorXd, eigenmath::VectorXd>>
PathPlannerGraph::GetLowerAndUpperValueBounds() const {
  eigenmath::VectorXd lower(vertex_data_dimension_);
  eigenmath::VectorXd upper(vertex_data_dimension_);
  VertexIterator vertex_iterator;
  VertexIterator vertex_end;

  // Check if graph has any vertices to compute the bounds.
  if (Size() == 0 || graph_.m_vertices.empty()) {
    return absl::FailedPreconditionError(
        "Cannot compute lower and upper bounds because graph is empty.");
  }

  lower = graph_.m_vertices.begin()->m_property.configuration;
  upper = graph_.m_vertices.begin()->m_property.configuration;

  for (std::tie(vertex_iterator, vertex_end) = boost::vertices(graph_);
       vertex_iterator != vertex_end; ++vertex_iterator) {
    lower = lower.cwiseMin(graph_[*vertex_iterator].configuration);
    upper = upper.cwiseMax(graph_[*vertex_iterator].configuration);
  }
  return std::make_pair(lower, upper);
}

}  // namespace intrinsic
