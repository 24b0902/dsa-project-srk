#include "graph.h"
#include <stdexcept>
#include <cmath>
#include <iostream>
#include <algorithm>


bool Graph::load_from_json(const json& graph_data) {
    
    //error handling for missing meta or nodes count
    if (!graph_data.contains("meta") || !graph_data["meta"].contains("nodes")) {
        std::cerr << "JSON Error: Missing 'meta' or node count in graph file." << std::endl;
        return false;
    }
    
    int node_count = graph_data["meta"]["nodes"].get<int>();
    all_nodes.reserve(node_count);
    //resize the adjacency list vector to the exact number of nodes for O(1) access
    adj.resize(node_count);
    int internal_idx_counter = 0;

    //Process Nodes and create the mapping (Node ID -> Internal Index)
    if (graph_data.contains("nodes")) {
        for (const auto& node_json : graph_data["nodes"]) {
            Node new_node;
            new_node.id = node_json["id"].get<int>();
            new_node.lat = node_json["lat"].get<double>();
            new_node.lon = node_json["lon"].get<double>();
            
            //beacause only some nodes have pois
            if (node_json.contains("pois")) {
                new_node.pois = node_json["pois"].get<std::vector<std::string>>();
            }
            
            // Assign the internal index and store mapping
            new_node.internal_index = internal_idx_counter++;
            all_nodes.push_back(new_node);
            node_id_to_index[new_node.id] = new_node.internal_index;
        }
    }

    //Process Edges and fill Adjacency List
    if (graph_data.contains("edges")) {
        size_t estimated_edges = graph_data["edges"].size();
        all_edges.reserve(estimated_edges * 2);
        for (const auto& edge_json : graph_data["edges"]) {
            Edge base_edge;
            
            base_edge.id = edge_json["id"].get<int>();
            base_edge.u = edge_json["u"].get<int>();
            base_edge.v = edge_json["v"].get<int>();
            base_edge.length = edge_json["length"].get<double>();
            base_edge.average_time = edge_json["average_time"].get<double>();
            base_edge.road_type = edge_json["road_type"].get<std::string>();
            base_edge.oneway = edge_json["oneway"].get<bool>();

            // speed profile
            if (edge_json.contains("speed_profile")) {
                base_edge.speed_profile = edge_json["speed_profile"].get<std::vector<double>>();
            }

            all_edges.push_back(base_edge);
            Edge* edge_ptr_uv = &all_edges.back();
            edge_id_to_pointer[base_edge.id] = edge_ptr_uv; //Register original ID

            try {
                // Use internal index for O(1) adjacency list access
                int u_idx = node_id_to_index.at(base_edge.u);
                adj[u_idx].push_back(edge_ptr_uv);
            } catch (const std::out_of_range& e) {
                std::cerr << "Warning: Edge " << base_edge.id << " source node " << base_edge.u << " not found." << std::endl;
            }

            //Store v -> u Edge (if not one-way)
            if (!base_edge.oneway) {
                // Create a separate reverse edge object (imp for clean graph traversal)
                Edge reverse_edge = base_edge;
                reverse_edge.u = base_edge.v;
                reverse_edge.v = base_edge.u;
                // Use a computed ID (e.g., negative) for the reverse edge lookup
                // This ensures the two directions of a two-way road are managed by the same updates.
                reverse_edge.id = -(base_edge.id); 
                
                all_edges.push_back(reverse_edge);
                Edge* edge_ptr_vu = &all_edges.back();
                edge_id_to_pointer[reverse_edge.id] = edge_ptr_vu; // Register reverse ID

                try {
                    int v_idx = node_id_to_index.at(base_edge.v);
                    adj[v_idx].push_back(edge_ptr_vu);
                } catch (const std::out_of_range& e) {
                    std::cerr << "Warning: Reverse edge for " << base_edge.id << " source node " << base_edge.v << " not found." << std::endl;
                }
            }
        }
    }
    
    return true;
}

//functions to access nodes and neighbours

const Node& Graph::get_node_by_id(int node_id) const {
    auto it = node_id_to_index.find(node_id);
    if (it == node_id_to_index.end()) {
        // Use an exception if a critical node (like source/target) is missing
        throw std::out_of_range("Node ID not found in graph during access.");
    }
    return all_nodes[it->second];
}

const std::vector<Edge*>& Graph::get_neighbors(int u_id) const {
    static const std::vector<Edge*> empty_vec;
    auto it = node_id_to_index.find(u_id);
    
    // 1. Check if node is found
    if (it == node_id_to_index.end()) {
        std::cerr << "Neighbors: Node ID " << u_id << " not mapped." << std::endl;
        return empty_vec; 
    }
    
    int internal_index = it->second;
    
    //error handling for out-of-bounds access
    if (internal_index < 0 || internal_index >= adj.size()) {
        std::cerr << "CRITICAL ERROR: Index " << internal_index << " out of bounds for adj vector!" << std::endl;
        // NOTE: We return empty_vec to avoid a crash, but this confirms the bug.
        return empty_vec; 
    }
    
    return adj[internal_index]; 
}

//updating functions
bool Graph::remove_edge(int edge_id) {
    auto it = edge_id_to_pointer.find(edge_id);
    if (it == edge_id_to_pointer.end()) return false; // Edge ID not in central map
    
    if (!it->second->is_active) {
        return false; // Already removed, output "done": false (as required)
    }
    
    // Deactivate forward edge
    it->second->is_active = false;
    
    //Deactivate reverse edge also (if it's a two-way road)
    auto rev_it = edge_id_to_pointer.find(-edge_id);
    if (rev_it != edge_id_to_pointer.end()) {
        rev_it->second->is_active = false;
    }
    
    return true;
}

bool Graph::modify_edge(int edge_id, const json& patch) {
    auto it = edge_id_to_pointer.find(edge_id);
    if (it == edge_id_to_pointer.end()) return false; 
    
    Edge* edge = it->second;
    
    //Ensure pointer is valid before access
    if (!edge) return false; 

    //Case 1: Empty patch on active edge
    if (patch.empty() && edge->is_active) {
        return false;
    }
    
    //Case 2: Restore
    if (!edge->is_active) {
        edge->is_active = true;
    } 
    
    // Helper to apply patch (Avoids code duplication)
    auto apply_patch = [&](Edge* e) {
        if (patch.contains("length")) e->length = patch["length"].get<double>();
        if (patch.contains("average_time")) e->average_time = patch["average_time"].get<double>();
        if (patch.contains("road_type")) e->road_type = patch["road_type"].get<std::string>();
        if (patch.contains("speed_profile")) {
            e->speed_profile = patch["speed_profile"].get<std::vector<double>>();
        }
    };

    // Apply to forward edge
    apply_patch(edge);

    // Apply to reverse edge
    // Check if reverse ID is different to avoid double-application (e.g. if id is 0)
    if (edge_id != -edge_id) {
        auto rev_it = edge_id_to_pointer.find(-edge_id);
        if (rev_it != edge_id_to_pointer.end()) {
            Edge* rev_edge = rev_it->second;
            
            //Ensure reverse pointer is valid
            if (rev_edge) {
                if (!rev_edge->is_active) rev_edge->is_active = true; 
                apply_patch(rev_edge);
            }
        }
    }
    
    return true; 
}