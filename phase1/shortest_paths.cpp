#include "Graph.h" 
#include <nlohmann/json.hpp>

#include <queue>
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <limits>
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <iostream>

using json = nlohmann::json;

std::vector<int> reconstruct_path(int target, const std::unordered_map<int, int>& parent) {
    std::vector<int> path;
    int curr = target;
    while (curr != -1) {
        path.push_back(curr);
        auto it = parent.find(curr);
        if (it == parent.end()) break;
        curr = it->second;
    }
    std::reverse(path.begin(), path.end());
    return path;
}

double heuristic_h(const Node& a, const Node& b) {
    double dx = a.lat - b.lat;
    double dy = a.lon - b.lon;
    return std::sqrt(dx * dx + dy * dy); 
}


//Minimising Distance (A* algorithm)
std::pair<std::vector<int>, double> minimising_distance(
    Graph& graph, int source, int target,
    const std::unordered_set<int>& forbidden_nodes,
    const std::unordered_set<std::string>& forbidden_road_types
) {
   
    std::priority_queue<std::pair<double, int>, std::vector<std::pair<double, int>>, std::greater<std::pair<double, int>>> pq;
    
    std::unordered_map<int, double> g_dist; 
    std::unordered_map<int, int> parent;

    const Node& target_node = graph.get_node_by_id(target); 

    g_dist[source] = 0.0;
    const Node& source_node = graph.get_node_by_id(source);
    double h_source = heuristic_h(source_node, target_node);
    
    pq.push({h_source, source}); 
    parent[source] = -1;
    bool target_found = false;

    while (!pq.empty()) {
        double current_f_score = pq.top().first;
        int u = pq.top().second;
        pq.pop();

        //Check for stale entry
        if (g_dist.count(u) && current_f_score > (g_dist.at(u) + heuristic_h(graph.get_node_by_id(u), target_node))) {
             continue;
        }
        
        //special case to skip processing if we already have a better g_dist
        if (g_dist.count(u) && g_dist.at(u) < (current_f_score - heuristic_h(graph.get_node_by_id(u), target_node)) - 1e-9 ) {
            continue;
        }

        if (u == target) {
            target_found = true;
            break; 
        }

        const std::vector<Edge*>& neighbors = graph.get_neighbors(u);
        for (Edge* edge_ptr : neighbors) {
            const Edge& edge = *edge_ptr;
            int v = edge.v; 
            
            if (!edge.is_active || forbidden_nodes.count(v) || forbidden_road_types.count(edge.road_type)) {
                continue;
            }
            
            double edge_cost = edge.length;
            double new_g_dist = g_dist.at(u) + edge_cost;

            if (!g_dist.count(v) || new_g_dist < g_dist.at(v)) {
                g_dist[v] = new_g_dist;
                parent[v] = u;
                
                const Node& v_node = graph.get_node_by_id(v);
                double h_v = heuristic_h(v_node, target_node);
                double f_v = new_g_dist + h_v;

                pq.push({f_v, v});
            }
        }
    }

    std::pair<std::vector<int>, double> result;
    if (target_found) {
        result.second = g_dist.at(target);
        result.first = reconstruct_path(target, parent);
    } else {
        result.first.clear();
        result.second = std::numeric_limits<double>::infinity();
    }
    return result;
}

double calculate_strict_travel_time(const Edge& edge, double departure_time_s) {
    if (edge.speed_profile.empty() || edge.speed_profile.size() != 96) {
        return edge.average_time; // Fallback to average_time
    }

    double remaining_length = edge.length;
    double current_time = departure_time_s;
    double total_time_spent = 0.0;
    
    const double TIME_SLOT_SECONDS = Edge::TIME_SLOT_SECONDS;

    while (remaining_length > 0.0) {
        int slot = ((int)(current_time) / (int)(TIME_SLOT_SECONDS)) % 96;
        double speed_mps = edge.speed_profile[slot];

        if (speed_mps <= 0.0) return std::numeric_limits<double>::infinity(); 

        double time_into_slot = std::fmod(current_time, TIME_SLOT_SECONDS);
        double time_to_next_slot_boundary = TIME_SLOT_SECONDS - time_into_slot;
        
        double dist_in_slot_limit = speed_mps * time_to_next_slot_boundary;
        
        if (remaining_length <= (dist_in_slot_limit + 1e-9)) { // Add epsilon for float comparison
            double time_spent_in_slot = remaining_length / speed_mps;
            total_time_spent += time_spent_in_slot;
            remaining_length = 0.0; 
        } else {
            total_time_spent += time_to_next_slot_boundary;
            remaining_length -= dist_in_slot_limit;
            current_time += time_to_next_slot_boundary; 
        }
    }

    return total_time_spent;
}


//Minimising Time(Time-Dependent Dijkstra's)
std::pair<std::vector<int>, double> minimising_time(
    Graph& graph, int source, int target,
    const std::unordered_set<int>& forbidden_nodes,
    const std::unordered_set<std::string>& forbidden_road_types
) {
    std::priority_queue<std::pair<double, int>, std::vector<std::pair<double, int>>, std::greater<std::pair<double, int>>> pq;
    std::unordered_map<int, double> arrival_time;
    std::unordered_map<int, int> parent;

    for(const auto& pair : graph.get_all_nodes()) {
        arrival_time[pair.first] = std::numeric_limits<double>::infinity();
    }

    arrival_time[source] = 0.0;
    pq.push({0.0, source});
    parent[source] = -1;
    bool target_found = false;

    while (!pq.empty()) {
        double current_arrival = pq.top().first; 
        int u = pq.top().second;
        pq.pop();

        if (current_arrival > arrival_time.at(u)) continue;

        if (u == target) {
            target_found = true;
            break;
        }

        const std::vector<Edge*>& neighbors = graph.get_neighbors(u);
        for (Edge* edge_ptr : neighbors) {
            const Edge& edge = *edge_ptr;
            int v = edge.v;

            if (!edge.is_active || forbidden_nodes.count(v) || forbidden_road_types.count(edge.road_type)) {
                continue;
            }

            double travel_time = calculate_strict_travel_time(edge, current_arrival);
            
            if (travel_time == std::numeric_limits<double>::infinity()) {
                continue; 
            }

            double new_arrival_at_v = current_arrival + travel_time;

            if (new_arrival_at_v < arrival_time.at(v)) {
                arrival_time[v] = new_arrival_at_v;
                parent[v] = u;
                pq.push({new_arrival_at_v, v});
            }
        }
    }

    std::pair<std::vector<int>, double> result;
    if (target_found) {
        result.second = arrival_time.at(target);
        result.first = reconstruct_path(target, parent);
    } else {
        result.first.clear();
        result.second = std::numeric_limits<double>::infinity();
    }
    return result;
}


json handle_shortest_path(Graph& graph, const json& query) {
    json result;
    result["id"] = query["id"];

    //Parse inputs and constraints
    int source = query["source"];
    int target = query["target"];
    std::string mode = query["mode"].get<std::string>();

    std::unordered_set<int> forbidden_nodes;
    if (query.contains("constraints") && query["constraints"].contains("forbidden_nodes")) {
        forbidden_nodes = query["constraints"]["forbidden_nodes"].get<std::unordered_set<int>>();
    }

    std::unordered_set<std::string> forbidden_road_types;
    if (query.contains("constraints") && query["constraints"].contains("forbidden_road_types")) {
        forbidden_road_types = query["constraints"]["forbidden_road_types"].get<std::unordered_set<std::string>>();
    }

    //edge case
    if (source == target) {
        result["possible"] = true;
        const std::string key = (mode == "time") ? "minimum_time" : "minimum_distance";
        result[key] = 0.0;
        result["path"] = {source};
        return result;
    }
    
    //checking forbidden nodes
    if (forbidden_nodes.count(source) || forbidden_nodes.count(target)) {
        result["possible"] = false;
        return result;
    }

    std::pair<std::vector<int>, double> path_res;
    
    if (mode=="distance"){
        path_res = minimising_distance(graph, source, target, forbidden_nodes, forbidden_road_types);
        
        if (path_res.second != std::numeric_limits<double>::infinity()) {
            result["possible"] = true;
            result["minimum_distance"] = path_res.second; 
            result["path"] = path_res.first; 
        } else {
            result["possible"] = false;
        }

    }else if(mode=="time"){
        path_res = minimising_time(graph, source, target, forbidden_nodes, forbidden_road_types);

        if (path_res.second != std::numeric_limits<double>::infinity()) {
            result["possible"] = true;
            result["minimum_time"] = path_res.second;
            result["path"] = path_res.first;
        } else {
            result["possible"] = false;
        }

    }else{
        std::cerr << "Error: Unknown shortest_path mode '" << mode << "'" << std::endl;
        result["possible"] = false;
    }

    return result;
}