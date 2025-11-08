#include "Graph.h"
#include <nlohmann/json.hpp>
#include <cmath>
#include <algorithm>
#include <map>
#include <queue>
#include <stdexcept>
#include <limits>
#include <vector>
#include <unordered_map>
using json = nlohmann::json;

//Helper functions
bool node_has_poi(const Node& node, const std::string& poi) {
    for (const std::string& p : node.pois) {
        if (p == poi) {
            return true;
        }
    }
    return false;
}

int find_nearest_node(Graph& graph, double lat, double lon) {
    const auto& all_nodes = graph.get_all_nodes();
    if (all_nodes.empty()) {
        return -1;
    }

    double min_dist_sq = std::numeric_limits<double>::max();
    int best_node = -1;

    for (const auto& pair : all_nodes) {
        const Node& node = pair.second;
        double dx = node.lat - lat;
        double dy = node.lon - lon;
        double dist_sq = dx*dx + dy*dy;
        if (dist_sq < min_dist_sq) {
            min_dist_sq = dist_sq;
            best_node = node.id;
        }
    }
    return best_node;
}

std::unordered_map<int, double> dijkstra(Graph& graph, int source) {
    std::unordered_map<int, double> dist;
    for (const auto& pair : graph.get_all_nodes()) {
        dist[pair.first] = std::numeric_limits<double>::infinity();
    }
    
    dist[source] = 0.0;
    
    // Min-heap type priority queue: {distance, node_id}
    std::priority_queue<std::pair<double, int>, 
                        std::vector<std::pair<double, int>>, 
                        std::greater<std::pair<double, int>>> pq;
    pq.push({0.0, source});

    while (!pq.empty()) {
        double d = pq.top().first;
        int u = pq.top().second;
        pq.pop();

        if (d > dist.at(u)) {
            continue;
        }

        for (Edge* edge_ptr : graph.get_neighbors(u)) {
            const Edge& edge = *edge_ptr;
            if (!edge.is_active) {
                continue;
            }
            
            int v = edge.v;
            double weight = edge.length; // KNN shortest path is by distance

            if (dist.at(u) + weight < dist.at(v)) {
                dist[v] = dist.at(u) + weight;
                pq.push({dist[v], v});
            }
        }
    }
    return dist;
}

//main functions
std::vector<int> find_knn_euclidean(Graph& graph, const std::string& poi, double lat, double lon, int k) {
    std::vector<std::pair<double, int>> candidates;
    
    for (const auto& pair : graph.get_all_nodes()) {
        const Node& node = pair.second;
        if (node_has_poi(node, poi)) {
            double dx = node.lat - lat;
            double dy = node.lon - lon;
            double dist = std::sqrt(dx * dx + dy * dy);
            candidates.push_back({dist, node.id});
        }
    }

    // Sort candidates by distance
    std::sort(candidates.begin(), candidates.end());

    // Get the top 'k' results
    std::vector<int> results;
    for (int i = 0; i < k && i < candidates.size(); ++i) {
        results.push_back(candidates[i].second);
    }
    return results;
}

std::vector<int> find_knn_shortest_path(Graph& graph, const std::string& poi, double lat, double lon, int k) {
    int start_node = find_nearest_node(graph, lat, lon);
    
    if (start_node==-1) {
         return {}; // Graph is empty
    }

    //get shortest path distances from that start node to all other nodes
    std::unordered_map<int, double> dists = dijkstra(graph, start_node);
    
    std::vector<std::pair<double, int>> candidates;
    for (const auto& pair : graph.get_all_nodes()) {
        const Node& node = pair.second;
        // Find nodes that have the POI
        if (node_has_poi(node, poi)) {
            double dist = dists[node.id];
            // Only add if it's reachable
            if (dist != std::numeric_limits<double>::infinity()) {
                candidates.push_back({dist, node.id});
            }
        }
    }
    std::sort(candidates.begin(), candidates.end());

    //get the top 'k' results
    std::vector<int> results;
    for (int i = 0; i < k && i < candidates.size(); ++i) {
        results.push_back(candidates[i].second);
    }
    return results;
}


json handle_knn_query(Graph& graph, const json& query) {
    json result;
    result["id"] = query["id"];

    try {
        int k = query["k"].get<int>();
        std::string metric = query["metric"].get<std::string>();
        std::string poi = query["poi"].get<std::string>();
        double lat = query["query_point"]["lat"].get<double>();
        double lon = query["query_point"]["lon"].get<double>();

        std::vector<int> knn_nodes;

        if (metric == "euclidean") {
            knn_nodes = find_knn_euclidean(graph, poi, lat, lon, k);
        } 
        else if (metric == "shortest_path") {
            knn_nodes = find_knn_shortest_path(graph, poi, lat, lon, k);
        } 
        else {
            throw std::runtime_error("Unknown KNN metric: " + metric);
        }
        result["nodes"] = knn_nodes;

    } catch (const std::exception& e) {
        std::cerr << "Error in handle_knn_query: " << e.what() << std::endl;
        result["error"] = e.what();
    }
    
    return result;
}