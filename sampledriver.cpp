#include <nlohmann/json.hpp>
#include <iostream>
#include <fstream>
#include <chrono>
/*
    Add other includes that you require, only write code wherever indicated
*/
#include "Graph.h"

using json = nlohmann::json;

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
        int Id=x["id"];
        double Lat =x["lat"];
        double Lon=x["lon"];
        std::vector<std::string> pos=x["pois"];
        graph.node_map[Id]=Node(Id,Lat,Lon,pos);
        graph.nodes.push_back(Id);
    }
    for(auto& y: graph_data["edges"]){
        int Id=y["id"];
        graph.Edges.push_back(Id);
        int u=y["u"];
        int v=y["v"];
        double length=y["length"];
        double average_time=y["average_time"];
        if(y.count("speed_profile"))std::vector<int> profile=y["speed_profile"];
        bool oneway=y["oneway"];
        std::string roadtype=y["road_type"];
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

    
    for (const auto& query : queries_json) {
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

        output_file << result.dump(4) << '\n';
    }

    output_file.close();
    return 0;
}