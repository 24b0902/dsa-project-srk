#include "Graph.h"
#include <nlohmann/json.hpp>
#include <queue>
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <map>
#include <limits>
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <iostream>
#include "shortest_paths.cpp"
using json = nlohmann::json;

std::vector<Edge*> get_path_edges(Graph& graph, const std::vector<int>& path_nodes) {
    std::vector<Edge*> path_edges;
    if (path_nodes.size()<2) {
        return path_edges;
    }
    for (long unsigned int i=0; i<path_nodes.size()-1; i++) {
        int u_id = path_nodes[i];
        int v_id = path_nodes[i+1];
        
        bool found_edge = false;
        for (Edge* edge : graph.get_neighbors(u_id)) {
            if (edge->v == v_id) {
                path_edges.push_back(edge);
                found_edge = true;
                break;
            }
        }
        if (!found_edge) {
            std::cerr << "Error: Couldnt find edge between " << u_id << " and " << v_id << std::endl;
        }
    }
    return path_edges;
}

double calculate_true_length(const std::vector<Edge*>& path_edges) {
    double true_length = 0.0;
    for (Edge* edge : path_edges) {
        true_length+=edge->length;
    }
    return true_length;
}

std::pair<std::vector<int>, double> minimising_distance_penalized(Graph& graph, int source, int target,const std::map<Edge*, int>& edge_usage_count,double penalty_factor) {
    std::priority_queue<std::pair<double, int>, std::vector<std::pair<double, int>>, std::greater<std::pair<double, int>>> pq;
    std::unordered_map<int, double> g_dist;
    std::unordered_map<int, int> parent;
    const Node& target_node = graph.get_node_by_id(target);

    for(const auto& pair : graph.get_all_nodes()) {
        g_dist[pair.first] = std::numeric_limits<double>::infinity();
    }
    g_dist[source] = 0.0;
    parent[source] = -1;
    
    pq.push({heuristic_h(graph.get_node_by_id(source), target_node), source});

    bool target_found = false;

    while (!pq.empty()) {
        double current_f_score = pq.top().first;
        int u = pq.top().second;
        pq.pop();

        double current_g_dist = g_dist.at(u);

        if (current_f_score > (current_g_dist + heuristic_h(graph.get_node_by_id(u), target_node) + 1e-9)) {
             continue;
        }

        if (u == target) {
            target_found = true;
            break; 
        }

        for (Edge* edge_ptr : graph.get_neighbors(u)) {
            const Edge& edge = *edge_ptr;
            int v = edge.v;
            
            if (!edge.is_active) continue;
            int usage = 0;
            auto it = edge_usage_count.find(edge_ptr);
            if (it != edge_usage_count.end()) {
                usage = it->second;
            }
        
            double edge_cost = edge.length * (1.0 + usage*penalty_factor);

            double new_g_dist = current_g_dist+edge_cost;

            if (new_g_dist < g_dist.at(v)) {
                g_dist[v] = new_g_dist;
                parent[v] = u;
                double h_v = heuristic_h(graph.get_node_by_id(v), target_node);
                double f_v = new_g_dist + h_v;
                pq.push({f_v, v});
            }
        }
    }

    std::pair<std::vector<int>, double> result;
    if (target_found) {
        // Note: g_dist.at(target) is the PENALIZED length.
        // We must recalculate the true length in the handler.
        result.second = g_dist.at(target); 
        result.first = reconstruct_path(target, parent);
    } else {
        result.first.clear();
        result.second = std::numeric_limits<double>::infinity();
    }
    return result;
}


json handle_k_shortest_paths_heuristic(Graph& graph, const json& query) {
    json result;
    result["id"] = query["id"];

    try {
        int source = query["source"].get<int>();
        int target = query["target"].get<int>();
        int k = query["k"].get<int>();

        //should change this !!!!
        const double penalty_factor = 1.0; 

        std::vector<std::pair<double, std::vector<int>>> final_paths;
        // Map to store how many times each edge has been used
        std::map<Edge*, int> edge_usage_count;

        auto path_res_1 = minimising_distance(graph, source, target, {}, {});
        if (path_res_1.second == std::numeric_limits<double>::infinity()) {
            result["paths"] = json::array();
            return result; //no path exists
        }
        
        std::vector<int> path_1_nodes = path_res_1.first;
        double path_1_len = path_res_1.second;
        final_paths.push_back({path_1_len, path_1_nodes});
        std::vector<Edge*> path_1_edges = get_path_edges(graph, path_1_nodes);
        for (Edge* edge : path_1_edges) {
            edge_usage_count[edge]++;
        }

        //next k-1 paths
        for (int i = 1; i < k; ++i) {
            auto next_path_res = minimising_distance_penalized(graph, source, target, edge_usage_count, penalty_factor);

            if (next_path_res.second == std::numeric_limits<double>::infinity()) {
                break; //no more paths exist
            }

            std::vector<int> next_path_nodes = next_path_res.first;
            std::vector<Edge*> next_path_edges = get_path_edges(graph, next_path_nodes);
            
            double true_length = calculate_true_length(next_path_edges);
            final_paths.push_back({true_length, next_path_nodes});

            for (Edge* edge : next_path_edges) {
                edge_usage_count[edge]++;
            }
        }

        std::sort(final_paths.begin(), final_paths.end());

        json paths_json = json::array();
        for (const auto& p : final_paths) {
            paths_json.push_back({
                {"path", p.second},
                {"length", p.first}
            });
        }
        result["paths"] = paths_json;

    } catch (const std::exception& e) {
        std::cerr << "Error has ocurred: " << e.what() << std::endl;
        result["error"] = e.what();
    }
    
    return result;
}