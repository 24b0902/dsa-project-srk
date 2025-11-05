#include <nlohmann/json.hpp>
#include <iostream>
#include <fstream>
#include <chrono>
/*
    Add other includes that you require, only write code wherever indicated
*/
#include "Graph.h"

using json = nlohmann::json;
json process_query(){
    json result;
    try{
    if (query.contains("id"))result["id"] = query["id"];
    if (!query.contains("type"))throw std::runtime_error("Missing 'type' field");
    std::string type = query["type"];
    if (type == "remove_edge") {
            int edge_id = query["edge_id"].get<int>();
            bool done =remove_edge(edge_id);
            result["done"] = done;
        }
        else if (type == "modify_edge") {
            int edge_id = query["edge_id"].get<int>();
            json patch;
        if (query.contains("patch"))patch = query["patch"];
            bool done = modify_edge(edge_id, patch);
            result["done"] = done;
        }
        else if (type == "shortest_path") {
            int source = query["source"].get<int>();
            int target = query["target"].get<int>();
            std::string mode = query["mode"].get<std::string>();
            std::vector<int> forbidden_nodes;
            std::vector<std::string> forbidden_types;

            if (query.contains("constraints")) {
                const json& constraints = query["constraints"];
                if (constraints.contains("forbidden_nodes"))
                    forbidden_nodes = constraints["forbidden_nodes"].get<std::vector<int>>();
                if (constraints.contains("forbidden_road_types"))
                    forbidden_types = constraints["forbidden_road_types"].get<std::vector<std::string>>();
            }
            auto [possible,x, path] =shortest_path(
                source, target, mode, forbidden_nodes, forbidden_types
            );
            result["possible"] = possible;
            if (possible) {
                if (mode == "time") result["minimum_time"] =x;
                else result["minimum_distance"] = x;
                result["path"] = path;
            }
        }
        else if (type == "knn") {
            std::string poi = query["poi"].get<std::string>();
            json query_point=query["query_point"];
            double lat = query_point["lat"].get<double>();
            double lon = query_point["lon"].get<double>();
            int k = query["value"];
            std::string metric = query["metric"].get<std::string>();
            std::vector<int> nodes=knn_query(poi, lat, lon, k, metric);
            result["nodes"] = nodes;
        }
        else if (type == "k_shortest_paths") {
            int source = query["source"].get<int>();
            int target = query["target"].get<int>();
            int k = query.["k"].get<int>();
            std::string mode = query["mode"].get<std::string>();
            auto paths=k_shortest_paths(source, target, k, mode);
            result["paths"] = json::array();
            for (const auto& p : paths) {
                json path_json;
                path_json["path"] = p.path;
                path_json["length"] = p.length;
                result["paths"].push_back(path_json);
            }
        }
        else if (type == "k_shortest_paths_heuristic") {
            int source = query["source"].get<int>();
            int target = query["target"].get<int>();
            int k = query["k"].get<int>();
            double overlap_threshold = query["overlap_threshold"].get<double>();
            auto paths =k_shortest_paths_heuristic(source, target, k, overlap_threshold);
            result["paths"] = json::array();
            for (const auto& p : paths) {
                json path_json;
                path_json["path"] = p.path;
                path_json["length"] = p.length;
                result["paths"].push_back(path_json);
            }
        }
         else if (type == "approx_shortest_path") {
            int time_budget = query["time_budget_ms"].get<int>();
            double acceptable_error = query["acceptable_error_pct"].get<double>();
             json distances = json::array();
            for (const auto& q : query["queries"]) {
                int source = q["source"];
                int target = q["target"];
                
                double approx_dist =approx_shortest_path(source, target, time_budget, acceptable_error);
                json d;
                d["source"] = source;
                d["target"] = target;
                d["approx_shortest_distance"] = approx_dist;
                distances.push_back(d);
            }

            result["distances"] = distances;
        }
        else {
            result["error"] = "Unknown query type: " + type;
        }
    }
    
    catch (const std::exception& e) {
        result["error"] = std::string("Exception: ") + e.what();
    }

    return result;
}


int main(int argc, char* argv[]) {
    if (argc != 3) {
        std::cerr << "Usage: " << argv[0] << " <graph.json> <queries.json>" << std::endl;
        return 1;
    }




    std::ifstream graph_file(argv[1]);
    if (!graph_file.is_open()) {
        std::cerr << "Failed to open " << argv[1] << std::endl;
        return 1;
    }
    json graph_data;
    graph_file >> graph_data;
    
    // Read graph from first file
    /*
        Add your graph reading and processing code here
        Initialize any classes and data structures needed for query processing
    */
    Graph graph;
    graph.nodes_num=graph_data["meta"]["nodes"];
    for(auto& x : graph_data["nodes"]){
        int Id=x["id"].get<int>();
        double Lat =x["lat"].get<double>();
        double Lon=x["lon"].get<double>();
        std::vector<std::string> pos=x["pois"].get<std::vector<std::string>>();
        graph.node_map[Id]=Node(Id,Lat,Lon,pos);
        graph.nodes.push_back(Id);
    }
    for(auto& y: graph_data["edges"]){
        int Id=y["id"].get<int>();
        graph.Edges.push_back(Id);
        int u=y["u"].get<int>();
        int v=y["v"].get<int>();
        double length=y["length"].get<double>();
        double average_time=y["average_time"].get<double>();
        if(y.count("speed_profile"))std::vector<int> profile=y["speed_profile"].get<std::vector<int>>();
        bool oneway=y["oneway"].get<bool>();
        std::string roadtype=y["road_type"].get<std::string>();
        graph.edge_map[{u,v}]=Edge(Id,u,v,length,average_time,profile,oneway,roadtype);
        graph.node_map[x].neighbours.push_back(y);
       if(!oneway) graph.node_map[y].neighbours.push_back(x);

    }

    // Read queries from second file
    std::ifstream queries_file(argv[2]);
    if (!queries_file.is_open()) {
        std::cerr << "Failed to open " << argv[2] << std::endl;
        return 1;
    }
    json queries_json;
    queries_file >> queries_json;

    std::ofstream output_file("output.json");
    if (!output_file.is_open()) {
        std::cerr << "Failed to open output.json for writing" << std::endl;
        return 1;
    }

    json output_json;
output_json["meta"] = queries_json["meta"];
output_json["results"] = json::array();
    
    for (const auto& query : queries_json["events"]) {
        auto start_time = std::chrono::high_resolution_clock::now();

        /*
            Add your query processing code here
            Each query should return a json object which should be printed to sample.json
        */


        // Answer each query replacing the function process_query using 
        // whatever function or class methods that you have implemented
        json result = process_query(query);

        auto end_time = std::chrono::high_resolution_clock::now();
        result["processing_time"] = std::chrono::duration<double, std::milli>(end_time - start_time).count();

        output_json["results"].push_back(result);
    }
    output_file << output_json.dump(4) << '\n';
    output_file.close();
    return 0;
}