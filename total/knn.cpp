#include "graph.h"
#include <cmath>
#include <algorithm>
#include <map>
#include <functional>
#include <queue> 
#include <stdexcept>

// Helper: Calculate Euclidean distance
// Treating Lat/Lon as Euclidean coordinates
double euclidean_distance(double lat1, double lon1, double lat2, double lon2) {
    double dx = lat1 - lat2;
    double dy = lon1 - lon2;
    return std::sqrt(dx * dx + dy * dy);
}

// Helper: Filter node by POI type
bool node_has_poi(const Node& node, const std::string& poi) {
    for (const std::string& node_poi : node.pois) {
        if (node_poi == poi) return true;
    }
    return false;
}

// Helper: Run Dijkstra's for a single source (distance/length only)
std::unordered_map<int, double> dijkstra_distance_only(Graph& g, int src_id) {
    std::unordered_map<int, double> dist;
    std::priority_queue<std::pair<double, int>, std::vector<std::pair<double, int>>, std::greater<std::pair<double, int>>> pq;

    dist[src_id] = 0.0;
    pq.push({0.0, src_id});

    while (!pq.empty()) {
        double d = pq.top().first;
        int u = pq.top().second;
        pq.pop();

        if (dist.count(u) && d > dist.at(u)) continue;

        const std::vector<Edge*>& neighbors = g.get_neighbors(u);
        for (Edge* edge_ptr : neighbors) {
            const Edge& edge = *edge_ptr;
            
            // KNN for shortest path metric should respect dynamic updates (is_active)
            if (!edge.is_active) continue; 

            int v = edge.v;
            double weight = edge.length; // Use length for distance metric
            double new_dist = dist.at(u) + weight;

            if (!dist.count(v) || new_dist < dist.at(v)) {
                dist[v] = new_dist;
                pq.push({new_dist, v});
            }
        }
    }
    return dist;
}

// --- 1. KNN Shortest Path Distance Metric ---
std::vector<int> knn_shortest_path(Graph& graph, const std::string& poi, 
    const Node& source_node, int k) 
{
    // Run SSSP (Dijkstra's) from the closest graph node (source_node.id)
    std::unordered_map<int, double> dists = dijkstra_distance_only(graph, source_node.id);
    
    std::vector<std::pair<int, double>> candidates;

    // Filter by POI and collect distances
    for (const auto& node : graph.get_all_nodes()) {
        //if (node.id == source_node.id) continue;
        
        // Check POI constraint AND reachability/valid distance
        if (node_has_poi(node, poi) && dists.count(node.id) && dists.at(node.id) < std::numeric_limits<double>::infinity()) {
            candidates.push_back({node.id, dists.at(node.id)});
        }
    }

    // Sort by distance
    std::sort(candidates.begin(), candidates.end(), 
              [](const auto& a, const auto& b) { return a.second < b.second; });

    // Collect top K
    std::vector<int> result;
    for (int i = 0; i < k && i < candidates.size(); ++i) {
        result.push_back(candidates[i].first);
    }

    return result;
}

// --- 2. KNN Euclidean Distance Metric ---
std::vector<int> knn_euclidean(Graph& graph, const std::string& poi, 
    const Node& query_point_coords, int k) 
{
    std::vector<std::pair<int, double>> candidates;

    // Calculate distance and filter by POI
    for (const auto& node : graph.get_all_nodes()) {
        // Exclude the node itself (though distances would be 0)
        // Note: The query_point_coords is usually NOT a graph node, so this check might be irrelevant.
        // if (node.id == query_point_coords.id) continue; 

        if (node_has_poi(node, poi)) {
            // Use the raw query point coordinates (lat, lon)
            double dist = euclidean_distance(query_point_coords.lat, query_point_coords.lon, node.lat, node.lon);
            candidates.push_back({node.id, dist});
        }
    }

    // Sort by distance
    std::sort(candidates.begin(), candidates.end(), 
              [](const auto& a, const auto& b) { return a.second < b.second; });

    // Collect top K
    std::vector<int> result;
    for (int i = 0; i < k && i < candidates.size(); ++i) {
        result.push_back(candidates[i].first);
    }

    return result;
}

// --- 3. Main KNN Handler ---
json handle_knn_query(Graph& graph, const json& query) {
    json result;
    result["id"] = query["id"];

    std::string poi = query["poi"].get<std::string>();
    int k = query["k"].get<int>();
    std::string metric = query["metric"].get<std::string>();
    
    double q_lat = query["query_point"]["lat"].get<double>();
    double q_lon = query["query_point"]["lon"].get<double>();

    // 1. Find nearest graph node to query point (for shortest_path metric)
    int nearest_node_id = -1;
    double min_euclidean_dist = std::numeric_limits<double>::max();
    
    for(const auto& node : graph.get_all_nodes()){
        double dist = euclidean_distance(q_lat, q_lon, node.lat, node.lon);
        if (dist < min_euclidean_dist) {
            min_euclidean_dist = dist;
            nearest_node_id = node.id;
        }
    }
    
    // Create a temporary Node structure to hold query point data
    Node query_point_data;
    query_point_data.lat = q_lat;
    query_point_data.lon = q_lon;
    query_point_data.id = nearest_node_id; 

    std::vector<int> knn_nodes;

    if (metric == "shortest_path") {
        if (nearest_node_id == -1) {
             knn_nodes = {}; // Graph is empty
        } else {
            // Use the nearest graph node as the source (required by instructions)
            knn_nodes = knn_shortest_path(graph, poi, graph.get_node_by_id(nearest_node_id), k);
        }
    } else if (metric == "euclidean") {
        // Use the raw query point coordinates
        knn_nodes = knn_euclidean(graph, poi, query_point_data, k);
    } else {
        std::cerr << "Error: Unknown KNN metric '" << metric << "'" << std::endl;
    }

    result["nodes"] = knn_nodes;
    return result;
}