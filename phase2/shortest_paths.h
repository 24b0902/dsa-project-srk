#ifndef SHORTEST_PATHS_H
#define SHORTEST_PATHS_H

#include "graph.h"
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <string>
#include <nlohmann/json.hpp> // Required for json return types

using json = nlohmann::json;

// Reconstructs path from parent map. Used by both A* and K-Shortest.
std::vector<int> reconstruct_path(int target, const std::unordered_map<int, int>& parent);

// Euclidean distance heuristic. Used by A* and Penalized A*.
double heuristic_h(const Node& a, const Node& b);

// strict travel time calculation for time-dependent logic
double calculate_strict_travel_time(const Edge& edge, double departure_time_s);

// Standard A* (Minimizing Distance)
std::pair<std::vector<int>, double> minimising_distance(
    Graph& graph, int source, int target,
    const std::unordered_set<int>& forbidden_nodes,
    const std::unordered_set<std::string>& forbidden_road_types
);

// Time-Dependent Dijkstra (Minimizing Time)
std::pair<std::vector<int>, double> minimising_time(
    Graph& graph, int source, int target,
    const std::unordered_set<int>& forbidden_nodes,
    const std::unordered_set<std::string>& forbidden_road_types
);

json handle_shortest_path(Graph& graph, const json& query);
json handle_k_shortest_paths(Graph& graph, const json& query);
json handle_k_shortest_paths_heuristic(Graph& graph, const json& query);
json handle_approx_shortest_paths(Graph& graph, const json& query);
#endif // SHORTEST_PATHS_H