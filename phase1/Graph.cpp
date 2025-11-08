#include "Graph.h"

void Graph::add_node(int id, double lat, double lon, std::vector<std::string> pois) {
    node_map.emplace(id, Node(id, lat, lon, std::move(pois)));
    //emplace instead of insert to save some time
}

const Node& Graph::get_node_by_id(int id) const {
    try {
        return node_map.at(id);
    } catch (const std::out_of_range& e) {
        throw std::out_of_range("Node with ID " + std::to_string(id) + "not found.");
    }
}

void Graph::add_edge(int id, int u, int v, double length, double avg_time, std::vector<double> profile, bool oneway, std::string road_type) {
    
    //creating a forward edge and pushing it into the list .
    edge_list.push_front(Edge(id, v, length, avg_time, std::move(profile), road_type));
    Edge* edge_list_pointer = &edge_list.front();
    adj[u].push_back(edge_list_pointer);
    
    //mapping edge id to edge pointer 
    edge_id_map[id].push_back(edge_list_pointer);

    if (!oneway) {
        //creating a reverse edge and repeating the same process.
        edge_list.push_front(Edge(id, u, length, avg_time, edge_list_pointer->speed_profile, road_type));
        Edge* reverse_edge_pointer = &edge_list.front();
        adj[v].push_back(reverse_edge_pointer);
        edge_id_map[id].push_back(reverse_edge_pointer);
    }
}

const std::vector<Edge*>& Graph::get_neighbors(int u) const {
   
    auto it = adj.find(u);
    if (it == adj.end()) {
        return empty_neighbors;
    }
    return it->second;
}


bool Graph::remove_edge(int edge_id) {
    auto it = edge_id_map.find(edge_id);
    if (it == edge_id_map.end()) {
        return false; //edge ID not found
    }

    bool state_changed = false;
    for (Edge* edge : it->second) {
        if (edge->is_active) {
            edge->is_active = false;
            state_changed = true;
        }
    }
    return state_changed;
}

bool Graph::modify_edge(int edge_id, const nlohmann::json& patch) {
    auto it = edge_id_map.find(edge_id);
    if (it == edge_id_map.end()) {
        return false;//edge ID not found
    }

    for (Edge* edge : it->second) {
        edge->is_active = true; //modifying make edge active
        if (patch.contains("length")) {
            edge->length = patch["length"].get<double>();
        }
        if (patch.contains("average_time")) {
            edge->average_time = patch["average_time"].get<double>();
        }
        if (patch.contains("road_type")) {
            edge->road_type = patch["road_type"].get<std::string>();
        }
        if (patch.contains("speed_profile")) {
            edge->speed_profile = patch["speed_profile"].get<std::vector<double>>();
        }
    }
    return true;
}

bool Graph::restore_edge(int edge_id) {
    auto it = edge_id_map.find(edge_id);
    if (it == edge_id_map.end()) {
        return false; //edge ID not found
    }

    bool state_changed = false;
    for (Edge* edge : it->second) {
        if (!edge->is_active) {
            edge->is_active = true;
            state_changed = true;
        }
    }
    return state_changed;
}