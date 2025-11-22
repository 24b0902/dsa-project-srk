#ifndef PHASE3_H
#define PHASE3_H

#include "graph.h"
#include <vector>
#include <map>
#include <algorithm>

// Structs required by SampleDriver.cpp and the Solver
struct Order {
    int order_id;
    int pickup_node;
    int dropoff_node;
    int ready_time = 0; // IDEA 1: Restaurant Preparation Time
};

struct Driver {
    int driver_id;
    std::vector<int> route; // Sequence of Node IDs (Depot -> P -> D -> ...)
    std::vector<Order> assigned_orders; // For tracking/output
    double total_time_traveled = 0.0;
    bool active = true; // IDEA 2: Driver availability status
};

struct SolutionMetrics {
    double total_delivery_time; // Sum of (Arrival Time - 0) for all orders
    double max_delivery_time;   // Time when the last driver finishes
};

class Phase3Solver {
private:
    Graph& graph;
    int depot_node;
    int num_drivers;
    std::vector<Order> orders;
    bool minimize_max_time; // Goal switch

    // Optimization: Subset of relevant nodes (Depot + Pickups + Dropoffs)
    std::vector<int> poi_nodes; 
    std::map<int, int> node_id_to_matrix_idx; // Real ID -> 0..K index
    std::vector<std::vector<double>> time_matrix; // K x K matrix of average times

    // Internal helper to build the matrix efficiently
    void build_time_matrix();
    
    // Helper to run Dijkstra specifically for Average Time (ignoring speed profiles)
    std::vector<double> run_internal_dijkstra(int start_node_id);

    // Helper to evaluate cost of a specific route sequence
    // Returns {Sum of Completion Times, Route Duration}
    // IDEA 1: Now accounts for ready_time wait penalties
    std::pair<double, double> evaluate_route(const std::vector<int>& route, const std::vector<Order>& route_orders);

    // Helper: Core logic to insert one order into the best spot in the fleet
    // Used by solve() and simulate_breakdown()
    bool attempt_insert_order(const Order& order, std::vector<Driver>& fleet);

public:
    Phase3Solver(Graph& g, int depot, int n_drivers, const std::vector<Order>& ords);

    // true = Minimize Max Time (Equity), false = Minimize Total Time (Efficiency)
    void set_optimization_goal(bool minimize_max);

    // Main Solver Logic
    std::vector<Driver> solve();

    // Helper to calculate final metrics for reporting
    SolutionMetrics get_final_metrics(const std::vector<Driver>& fleet);

    // --- RESEARCH & EXTRA IDEAS ---

    // IDEA 1: Assign random ready times to orders to simulate kitchen lag
    void enable_kitchen_lag_simulation();

    // IDEA 2: Simulate a driver breakdown and reassign their orders
    // Returns the reassigned fleet
    std::vector<Driver> simulate_breakdown(std::vector<Driver> current_fleet, int driver_to_fail);

    // IDEA 4: Simulate an order cancellation and remove it from the schedule
    // Returns the updated fleet
    std::vector<Driver> simulate_cancellation(std::vector<Driver> current_fleet, int order_id_to_cancel);
};

#endif