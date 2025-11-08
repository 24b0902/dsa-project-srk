#ifndef GRAPH_H
#define GRAPH_H

#include <nlohmann/json.hpp>
using json = nlohmann::json;
#include <iostream>
#include <vector>
#include <string>
#include <unordered_map>
#include <list>
#include <stdexcept>
#include <utility> // for std::move : used for faster implementation

struct Node {
    int id;
    double lat;
    double lon;
    std::vector<std::string> pois;

    Node(){
        this->id = -1;
        this->lat = 0.0;
        this->lon = 0.0;
    }
    Node(int id, double lat, double lon, std::vector<std::string> pois_list) {
        this->id = id;
        this->lat = lat;
        this->lon = lon;
        this->pois = std::move(pois_list);
    }
};

struct Edge {
    int id;
    int v; // destination node ID
    double length;
    double average_time;
    std::vector<double> speed_profile; 
    std::string road_type;
    bool is_active;

    // 15 minutes*60 seconds = 900
    const double TIME_SLOT_SECONDS = 900.0; 

    Edge(){
        this->id = -1;
        this->v = -1;
        this->length = 0.0;
        this->average_time = 0.0;
        this->is_active = true;
    }
    Edge(int id, int dest_v, double len, double avg_time, std::vector<double> profile, std::string type){
        this->id = id;
        this->v = dest_v;
        this->length = len;
        this->average_time = avg_time;
        this->speed_profile = std::move(profile);
        this->road_type = std::move(type);
        this->is_active = true;
    }
};

class Graph {
private:
    // Stores the Node objects, accessible by node ID.
    std::unordered_map<int, Node> node_map;

    //std::list to store all Edge objects.
    // This ensures that pointers to these objects remain valid even if we add new edges.
    std::list<Edge> edge_list;

    //adjacency list: maps node IDs to a vector of pointers to outgoing Edge objects.
    std::unordered_map<int, std::vector<Edge*>> adj;

    // Maps an edge ID to all Edge objects associated with it (e.g., forward and reverse).
    std::unordered_map<int, std::vector<Edge*>> edge_id_map;
    const std::vector<Edge*> empty_neighbors;

public:

    void add_node(int id, double lat, double lon, std::vector<std::string> pois);
    const Node& get_node_by_id(int id) const;
    void add_edge(int id, int u, int v, double length, double avg_time, std::vector<double> profile, bool oneway, std::string road_type);
    const std::vector<Edge*>& get_neighbors(int u) const;
    bool remove_edge(int edge_id);
    bool modify_edge(int edge_id, const nlohmann::json& patch);
    bool restore_edge(int edge_id);
    const std::unordered_map<int, Node>& get_all_nodes() const;


};


#endif
