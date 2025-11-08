#include <nlohmann/json.hpp>
#include <iostream>
#include <fstream>
#include <chrono>
#include <vector>
#include <string>
#include <unordered_set>
#include <stdexcept>
#include "Graph.h" 

json handle_shortest_path(Graph& graph, const json& query);
json handle_knn_query(Graph& graph, const json& query);
json handle_k_shortest_paths(Graph& graph, const json& query);
json handle_k_shortest_paths_heuristic(Graph& graph, const json& query);
json handle_approx_shortest_path(Graph& graph, const json& query);

json process_query(Graph& graph, const json& query){
    json result;
    try{
        if (!query.contains("id")){
            throw std::runtime_error("Missing 'id' field");
        }
        result["id"] = query["id"];

        if (!query.contains("type")){
            throw std::runtime_error("Missing 'type' field");
        }
        std::string type = query["type"];

        if (type=="remove_edge") {
            int edge_id = query["edge_id"].get<int>();
            bool done = graph.remove_edge(edge_id);
            result["done"] = done;
        }else if (type=="modify_edge") {
            int edge_id = query["edge_id"].get<int>();
            if (query.contains("patch") && !query["patch"].empty()) {
                bool done = graph.modify_edge(edge_id, query["patch"]);
                result["done"] = done;
            } else {
                //empty patch restores the edge
                bool done = graph.restore_edge(edge_id);
                result["done"] = done;
            }
        }else if (type == "shortest_path") {
            result = handle_shortest_path(graph, query);
        }else if (type == "knn") {
            result = handle_knn_query(graph, query);
        } else if (type == "k_shortest_paths") {
            result = handle_k_shortest_paths(graph, query);
        }else if (type == "k_shortest_paths_heuristic") {
            result = handle_k_shortest_paths_heuristic(graph, query);
        }else if (type == "approx_shortest_path") {
            result = handle_approx_shortest_path(graph, query);
        }else{
            result["error"] = "Unknown query type: " + type;
        }
    }catch (const std::exception& e) {
        if (query.contains("id")) result["id"] = query["id"];
        result["error"] = std::string("Exception occured: ") + e.what();
    }

    return result;
}


int main(int argc, char* argv[]) {
   // 3 extra arguments required: graph, queries,output file paths
    if (argc != 4) {
        std::cerr << "give arguments as: " << argv[0] << " <graph.json> <queries.json> <output.json>" << std::endl;
        return 1;
    }

    std::string graph_path = argv[1];
    std::string queries_path = argv[2];
    std::string output_path = argv[3];

    std::ifstream graph_file(graph_path);
    if (!graph_file.is_open()) {
        std::cerr << "Failed to open " << graph_path << std::endl;
        return 1;
    }
    json graph_data;
    graph_file >> graph_data;
    graph_file.close();
    
    Graph graph;
    std::cout << "Loading nodes..." << std::endl;
    for(const auto& x : graph_data["nodes"]){
        graph.add_node(
            x["id"].get<int>(),
            x["lat"].get<double>(),
            x["lon"].get<double>(),
            x["pois"].get<std::vector<std::string>>()
        );
    }
    
    std::cout << "Loading edges..." << std::endl;
    for(const auto& y: graph_data["edges"]){
        std::vector<double> profile;
        if(y.contains("speed_profile")) {
            profile = y["speed_profile"].get<std::vector<double>>();
        }

        graph.add_edge(
            y["id"].get<int>(),
            y["u"].get<int>(),
            y["v"].get<int>(),
            y["length"].get<double>(),
            y["average_time"].get<double>(),
            std::move(profile),
            y["oneway"].get<bool>(),
            y["road_type"].get<std::string>()
        );
    }
    std::cout << "Graph loading complete." << std::endl;
 
    std::ifstream queries_file(queries_path);
    if (!queries_file.is_open()) {
        std::cerr << "Failed to open " << queries_path << std::endl;
        return 1;
    }
    json queries_json;
    queries_file >> queries_json;
    queries_file.close();

    std::ofstream output_file(output_path);
    if (!output_file.is_open()) {
        std::cerr << "Failed to open " << output_path << " for writing" << std::endl;
        return 1;
    }

    json output_json;
    output_json["meta"] = queries_json["meta"];
    output_json["results"] = json::array();
    
    for (const auto& query : queries_json["events"]) {
        auto start_time = std::chrono::high_resolution_clock::now();

        json result = process_query(graph, query);

        auto end_time = std::chrono::high_resolution_clock::now();
        result["processing_time_ms"] = std::chrono::duration<double, std::milli>(end_time - start_time).count();

        output_json["results"].push_back(result);
    }
    output_file << output_json.dump(4) << '\n';
    output_file.close();

    std::cout << "Processing complete. Output written to " << output_path << std::endl;
    return 0;
}