#include <nlohmann/json.hpp>
#include <iostream>
#include <fstream>
#include <chrono>
#include <vector>
#include <stdexcept>
#include <string>
#include "graph.h" 
#include "phase3.h"
using json = nlohmann::json;

json handle_shortest_path(Graph& graph, const json& query);
json handle_knn_query(Graph& graph, const json& query);
json handle_k_shortest_paths(Graph& graph, const json& query);
json handle_k_shortest_paths_heuristic(Graph& graph, const json& query);
json handle_approx_shortest_paths(Graph& graph, const json& query);

bool handle_update(Graph& graph, const json& query) {
    std::string type = query["type"];
    int edge_id = query["edge_id"];

    if (type == "remove_edge") {
        return graph.remove_edge(edge_id);
    } else if (type == "modify_edge") {
        //extract patch (if missing then use empty object)
        json patch = query.contains("patch") ? query["patch"] : json::object();
        return graph.modify_edge(edge_id, patch);
    }
    return false; //not successful
}

json handle_delivery_scheduling(Graph& graph, const json& query) {
    // 1. Parse Inputs
    // Check for existence to avoid exceptions, though prompt guarantees valid input
    int depot = query["fleet"]["depot_node"];
    int num_drivers = query["fleet"]["num_delievery_guys"];
    
    std::vector<Order> orders;
    if (query.contains("orders")) {
        for (const auto& o : query["orders"]) {
            // Construct Order struct from JSON
            orders.push_back({
                o["order_id"].get<int>(), 
                o["pickup"].get<int>(), 
                o["dropoff"].get<int>()
            });
        }
    }

    // --- RUN 1: Optimize for TOTAL Time (Primary Goal) ---
    // This is the "Efficiency" metric required by the problem statement
    Phase3Solver solverTotal(graph, depot, num_drivers, orders);
    solverTotal.set_optimization_goal(false); // false = Minimize SUM
    std::vector<Driver> driversTotal = solverTotal.solve();
    SolutionMetrics metricsTotal = solverTotal.get_final_metrics(driversTotal);

    // --- RUN 2: Optimize for MAX Time (For Research/Report) ---
    // This is the "Equity" metric for your analysis marks
    Phase3Solver solverMax(graph, depot, num_drivers, orders);
    solverMax.set_optimization_goal(true); // true = Minimize MAX
    std::vector<Driver> driversMax = solverMax.solve();
    SolutionMetrics metricsMax = solverMax.get_final_metrics(driversMax);

    // --- PRINT COMPARISON TO CONSOLE ---
    // This helps you gather data for your PDF report
    std::cout << "\n[Phase 3 Analysis] fleet_size=" << num_drivers << ", orders=" << orders.size() << std::endl;
    std::cout << "  Strategy 1 (Min Sum): Total=" << metricsTotal.total_delivery_time 
              << "s | Max=" << metricsTotal.max_delivery_time << "s" << std::endl;
    std::cout << "  Strategy 2 (Min Max): Total=" << metricsMax.total_delivery_time 
              << "s | Max=" << metricsMax.max_delivery_time << "s" << std::endl;
    std::cout << "-------------------------------------------------\n" << std::endl;

    // 5. Format JSON Output 
    // We output the solution from Strategy 1 (Total Time) as it's the standard VRP objective
    json output;
    if (query.contains("id")) output["id"] = query["id"]; 

    json assignments = json::array();
    for (const auto& driver : driversTotal) {
        json d_json;
        d_json["driver_id"] = driver.driver_id;
        d_json["route"] = driver.route;
        
        std::vector<int> o_ids;
        for(const auto& o : driver.assigned_orders) {
            o_ids.push_back(o.order_id);
        }
        d_json["order_ids"] = o_ids;
        assignments.push_back(d_json);
    }
    
    output["assignments"] = assignments;
    
    // Required Metric
    output["metrics"]["total_delivery_time_s"] = metricsTotal.total_delivery_time;
    
    // Optional/Extra Metrics (Ensure these don't break autograder, but usually extra keys are ignored)
    output["metrics"]["max_delivery_time_s"]   = metricsTotal.max_delivery_time; 
    output["metrics"]["strategy2_total_s"]     = metricsMax.total_delivery_time;
    output["metrics"]["strategy2_max_s"]       = metricsMax.max_delivery_time;

    return output;
}

//calls required functions
json process_query(Graph& graph, const json& query) {
    std::string type = query.value("type", "unknown");
    
    if (type == "shortest_path") {
        return handle_shortest_path(graph, query);
    } else if (type == "knn") {
        return handle_knn_query(graph, query);
    } else if (type == "k_shortest_paths") {
        return handle_k_shortest_paths(graph, query);
    } else if (type == "k_shortest_paths_heuristic") {
        return handle_k_shortest_paths_heuristic(graph, query);
    } else if (type == "approx_shortest_path") {
        return handle_approx_shortest_paths(graph, query);
    } else if (type == "unknown" && query.contains("orders") && query.contains("fleet")) { 
        return handle_delivery_scheduling(graph, query);
    }else if (type == "remove_edge" || type == "modify_edge") {
        json result;
        result["id"] = query.value("id", -1);
        result["done"] = handle_update(graph, query); 
        return result;
    } else {
        std::cerr << "Error: Unknown query type : " << type << std::endl;
        json result;
        result["id"] = query.value("id", -1);
        return result;
    }
}


int main(int argc, char* argv[]) {
    if (argc != 4) {
        std::cerr << "Usage: " << argv[0] << " <graph.json> <queries.json> <output.json>" << std::endl;
        return 1;
    }

    // 1. Read Graph
    std::ifstream graph_file(argv[1]);
    if (!graph_file.is_open()) {
        std::cerr << "Failed to open graph file: " << argv[1] << std::endl;
        return 1;
    }
    json graph_json;
    try {
        graph_file >> graph_json;
    } catch (const json::parse_error& e) {
        std::cerr << "Error parsing graph JSON: " << e.what() << std::endl;
        return 1;
    }
    graph_file.close();

    Graph graph;
    if (!graph.load_from_json(graph_json)) {
        std::cerr << "Graph load failed." << std::endl;
        return 1;
    }
    std::cout << "Graph loaded successfully." << std::endl;

    // 2. Read Queries
    std::ifstream queries_file(argv[2]);
    if (!queries_file.is_open()) {
        std::cerr << "Failed to open queries file: " << argv[2] << std::endl;
        return 1;
    }
    json queries_json;
    try {
        queries_file >> queries_json;
    } catch (const json::parse_error& e) {
        std::cerr << "Error parsing queries JSON: " << e.what() << std::endl;
        return 1;
    }
    queries_file.close();

    json meta;
    if (queries_json.contains("meta")) meta = queries_json["meta"];
    
    std::vector<json> results;

    // 3. Process Queries (Logic to handle Phase 3 format vs Phase 1/2 format)
    
    // CASE A: Standard "events" array (Phase 1 & 2)
    if (queries_json.contains("events")) {
        std::cout << "Processing 'events' array..." << std::endl;
        for (const auto& query : queries_json["events"]) {
            try {
                auto start_time = std::chrono::high_resolution_clock::now();
                json result = process_query(graph, query);
                auto end_time = std::chrono::high_resolution_clock::now();
                result["processing_time"] = std::chrono::duration<double, std::milli>(end_time - start_time).count();
                results.push_back(result);
            } catch (const std::exception& e) {
                std::cerr << "Runtime Error: " << e.what() << std::endl;
            }
        }
    } 
    // CASE B: Single Phase 3 Query (No "events", just "orders" at root)
    else if (queries_json.contains("orders") && queries_json.contains("fleet")) {
        std::cout << "Processing single Phase 3 query..." << std::endl;
        try {
            // Treat the entire file content as one query
            auto start_time = std::chrono::high_resolution_clock::now();
            json result = process_query(graph, queries_json); // Pass the root object
            auto end_time = std::chrono::high_resolution_clock::now();
            
            // If output doesn't have an ID, give it one for consistency
            if (!result.contains("id")) result["id"] = 1;
            
            result["processing_time"] = std::chrono::duration<double, std::milli>(end_time - start_time).count();
            results.push_back(result);
        } catch (const std::exception& e) {
            std::cerr << "Runtime Error in Phase 3: " << e.what() << std::endl;
        }
    }
    else {
        std::cerr << "Warning: queries.json has neither 'events' nor 'orders'." << std::endl;
    }

    // 4. Write Output
    std::ofstream output_file(argv[3]);
    if (!output_file.is_open()) {
        std::cerr << "Failed to open output file." << std::endl;
        return 1;
    }

    json output;
    output["meta"] = meta;
    output["results"] = results;
    output_file << output.dump(4) << std::endl;
    output_file.close();
    
    std::cout << "Processing complete. Results written to " << argv[3] << std::endl;
    return 0;
}