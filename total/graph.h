#ifndef GRAPH_H
#define GRAPH_H

#include <vector>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <limits>
#include <iostream>
#include <algorithm>
#include <cmath>
#include <deque>
#include <nlohmann/json.hpp>
using json = nlohmann::json;

struct Node {
    int id;
    double lat;
    double lon;
    std::vector<std::string> pois;
    int internal_index;   // Internal index used for efficient array/vector access???
};

struct Edge {
    int id;               
    int u; //source node id
    int v; //target node id
    double length;        // Distance (meters)
    double average_time;  // time (seconds)
    std::vector<double> speed_profile; // 96 x 15-minute slots (m/s)
    std::string road_type; 
    bool oneway;          
    
    //Status (required for updates: remove_edge/modify_edge)
    bool is_active = true; 
    const double TIME_SLOT_SECONDS = 900.0; 
};

class Graph {
private:
    std::vector<Node> all_nodes;
    std::unordered_map<int, int> node_id_to_index; // Node ID->internal_index
    std::vector<Edge> all_edges;
    
    //Adjacency List: internal_index->vector of pointers to Edge objects
    std::vector<std::vector<Edge*>> adj; 
    
    //Map Edge ID to a pointer/reference in 'all_edges'
    std::unordered_map<int, Edge*> edge_id_to_pointer; 

public:
    Graph() = default; //initialize empty graph
    bool load_from_json(const json& graph_data);
    
    //functions to get node and neighbors
    const Node& get_node_by_id(int node_id) const;
    const std::vector<Edge*>& get_neighbors(int u_id) const;
    const std::vector<Node>& get_all_nodes() const { return all_nodes; }

    //updates to the graph
    bool remove_edge(int edge_id);
    bool modify_edge(int edge_id, const json& patch);
};


//phase 1: Shortest Path Algorithms
std::pair<std::vector<int>, double> minimising_distance(
    Graph& graph, int source, int target,
    const std::unordered_set<int>& forbidden_nodes,
    const std::unordered_set<std::string>& forbidden_road_types
);
std::pair<std::vector<int>, double> minimising_time(
    Graph& graph, int source, int target,
    const std::unordered_set<int>& forbidden_nodes,
    const std::unordered_set<std::string>& forbidden_road_types
);
json handle_shortest_path(Graph& graph, const json& query);

//phase 1: KNN Queries
json handle_knn_query(Graph& graph, const json& query);

//phase 2
json handle_k_shortest_paths(Graph& graph, const json& query);
json handle_k_shortest_paths_heuristic(Graph& graph, const json& query);
json handle_approx_shortest_paths(Graph& graph, const json& query);
//phase 3
json handle_delivery_scheduling(Graph& graph, const json& query);
#endif