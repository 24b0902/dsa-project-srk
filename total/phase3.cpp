#include "phase3.h"
#include <queue>
#include <algorithm>
#include <iostream>
#include <limits>
#include <map>
#include <random> // Required for Idea 1 simulation

const double INF = 1e15; 

//Constructor
Phase3Solver::Phase3Solver(Graph& g, int depot, int n_drivers, const std::vector<Order>& ords)
    : graph(g), depot_node(depot), num_drivers(n_drivers), orders(ords), minimize_max_time(false) {
    
    if (num_drivers <= 0) num_drivers = 1;

    // 1. Identify Points of Interest (POIs)
    poi_nodes.push_back(depot_node);
    for (const auto& o : orders) {
        poi_nodes.push_back(o.pickup_node);
        poi_nodes.push_back(o.dropoff_node);
    }

    // Remove duplicates
    std::sort(poi_nodes.begin(), poi_nodes.end());
    poi_nodes.erase(std::unique(poi_nodes.begin(), poi_nodes.end()), poi_nodes.end());

    // Map Real ID -> Matrix Index
    for (size_t i = 0; i < poi_nodes.size(); ++i) {
        node_id_to_matrix_idx[poi_nodes[i]] = i;
    }

    // 2. Pre-compute Time Matrix
    build_time_matrix();
}

void Phase3Solver::set_optimization_goal(bool minimize_max) {
    this->minimize_max_time = minimize_max;
}

//Helper: Internal Dijkstra
std::vector<double> Phase3Solver::run_internal_dijkstra(int start_node_id) {
    size_t max_size = graph.get_all_nodes().size(); 
    std::vector<double> dists(max_size, INF);
    
    // Robustly find start index
    int start_internal_idx = -1;
    try {
        start_internal_idx = graph.get_node_by_id(start_node_id).internal_index;
    } catch (...) {
        std::cerr << "[Error] Dijkstra start node " << start_node_id << " not found in graph!" << std::endl;
        return dists;
    }

    dists[start_internal_idx] = 0.0;

    using P = std::pair<double, int>;
    std::priority_queue<P, std::vector<P>, std::greater<P>> pq;
    pq.push({0.0, start_internal_idx});

    while (!pq.empty()) {
        double d = pq.top().first;
        int u_idx = pq.top().second;
        pq.pop();

        if (d > dists[u_idx]) continue;

        int u_real_id = graph.get_all_nodes()[u_idx].id;
        const std::vector<Edge*>& neighbors = graph.get_neighbors(u_real_id);

        for (Edge* edge : neighbors) {
            if (!edge->is_active) continue;

            double weight = edge->average_time;
            int v_real_id = edge->v;
            
            try {
                int v_idx = graph.get_node_by_id(v_real_id).internal_index;
                if (dists[u_idx] + weight < dists[v_idx]) {
                    dists[v_idx] = dists[u_idx] + weight;
                    pq.push({dists[v_idx], v_idx});
                }
            } catch (...) { continue; }
        }
    }
    return dists;
}

void Phase3Solver::build_time_matrix() {
    int n = poi_nodes.size();
    time_matrix.assign(n, std::vector<double>(n, INF));
    
    for (int i = 0; i < n; ++i) {
        int u_id = poi_nodes[i];
        std::vector<double> all_dists = run_internal_dijkstra(u_id);

        for (int j = 0; j < n; ++j) {
            int v_id = poi_nodes[j];
            try {
                int v_idx = graph.get_node_by_id(v_id).internal_index;
                if (v_idx < all_dists.size()) {
                    time_matrix[i][j] = all_dists[v_idx];
                }
            } catch (...) { }
        }
    }
}

// Evaluate a Route
// IMPLEMENTS IDEA 1: Checks Ready Time
std::pair<double, double> Phase3Solver::evaluate_route(const std::vector<int>& route, const std::vector<Order>& route_orders) {
    if (route.empty()) return {0.0, 0.0};

    double current_time = 0.0;
    double sum_arrival_times = 0.0;
    int current_node = depot_node; 

    size_t start_k = (route.size() > 0 && route[0] == depot_node) ? 1 : 0;

    for (size_t k = start_k; k < route.size(); ++k) {
        int next_node = route[k];
        
        int u_idx = node_id_to_matrix_idx[current_node];
        int v_idx = node_id_to_matrix_idx[next_node];
        
        double travel_time = time_matrix[u_idx][v_idx];
        
        if (travel_time >= INF) return {1e14, 1e14}; 
        
        current_time += travel_time;

        // IDEA 1: Kitchen Lag Logic
        // If this node is a PICKUP, we might have to wait
        for(const auto& o : route_orders) {
            if (o.pickup_node == next_node) {
                // If we arrived at 10:00 but food is ready at 10:15, current_time becomes 10:15
                if (current_time < o.ready_time) {
                    current_time = (double)o.ready_time;
                }
                break; 
            }
        }

        // If this node is a DROPOFF, record completion
        for(const auto& o : route_orders) {
            if(o.dropoff_node == next_node) {
                sum_arrival_times += current_time;
            }
        }
        current_node = next_node;
    }
    return {sum_arrival_times, current_time};
}

//Helper: Greedy Insertion Logic
// This logic was extracted from solve() so it can be reused in simulate_breakdown()
bool Phase3Solver::attempt_insert_order(const Order& order, std::vector<Driver>& fleet) {
    double best_cost_increase = INF;
    int best_driver = -1;
    std::vector<int> best_route_seq;

    for (int d = 0; d < fleet.size(); ++d) {
        // IDEA 2 Check: Skip inactive drivers (broken down)
        if (!fleet[d].active) continue;

        const std::vector<int>& current_route = fleet[d].route;
        
        // Try insertion (1 to Size)
        for (size_t i = 1; i <= current_route.size(); ++i) {
            for (size_t j = i + 1; j <= current_route.size() + 1; ++j) {
                
                std::vector<int> candidate_route = current_route;
                candidate_route.insert(candidate_route.begin() + i, order.pickup_node);
                candidate_route.insert(candidate_route.begin() + j, order.dropoff_node);

                std::vector<Order> candidate_orders = fleet[d].assigned_orders;
                candidate_orders.push_back(order);

                std::pair<double, double> metrics = evaluate_route(candidate_route, candidate_orders);
                
                if (metrics.second >= 1e13) continue;

                double cost = minimize_max_time ? metrics.second : metrics.first;

                if (minimize_max_time) {
                    double max_t = metrics.second;
                    for(int k=0; k < fleet.size(); ++k) {
                        if(k != d && fleet[k].active) max_t = std::max(max_t, fleet[k].total_time_traveled);
                    }
                    cost = max_t;
                } else {
                    cost = metrics.first;
                }

                if (cost < best_cost_increase) {
                    best_cost_increase = cost;
                    best_driver = d;
                    best_route_seq = candidate_route;
                }
            }
        }
    }

    if (best_driver != -1) {
        fleet[best_driver].route = best_route_seq;
        fleet[best_driver].assigned_orders.push_back(order);
        auto res = evaluate_route(fleet[best_driver].route, fleet[best_driver].assigned_orders);
        fleet[best_driver].total_time_traveled = res.second;
        return true;
    }
    return false;
}

//Main function(solver)
std::vector<Driver> Phase3Solver::solve() {
    std::vector<Driver> fleet(num_drivers);
    for (int i = 0; i < num_drivers; ++i) {
        fleet[i].driver_id = i;
        fleet[i].route.push_back(depot_node); 
        fleet[i].active = true;
    }

    std::vector<Order> pending_orders = orders;
    int dep_idx = node_id_to_matrix_idx[depot_node];
    
    // Sort: Farthest First
    std::sort(pending_orders.begin(), pending_orders.end(), [&](const Order& a, const Order& b) {
        double distA = time_matrix[dep_idx][node_id_to_matrix_idx[a.pickup_node]];
        double distB = time_matrix[dep_idx][node_id_to_matrix_idx[b.pickup_node]];
        if (distA >= INF && distB < INF) return false;
        if (distA < INF && distB >= INF) return true;
        return distA > distB; 
    });

    int unassigned_count = 0;
    // REFACTORED: Now uses the helper function
    for (const auto& order : pending_orders) {
        if (!attempt_insert_order(order, fleet)) {
            unassigned_count++;
        }
    }
    
    if (unassigned_count > 0 && !minimize_max_time) {
        std::cout << "[Warning] " << unassigned_count << " orders could not be assigned." << std::endl;
    }

    return fleet;
}

SolutionMetrics Phase3Solver::get_final_metrics(const std::vector<Driver>& fleet) {
    SolutionMetrics m;
    m.total_delivery_time = 0;
    m.max_delivery_time = 0;

    for (const auto& d : fleet) {
        if (!d.active) continue; // Don't count broken trucks
        auto res = evaluate_route(d.route, d.assigned_orders);
        m.total_delivery_time += res.first;
        m.max_delivery_time = std::max(m.max_delivery_time, res.second);
    }
    return m;
}

// extra implementations

// IDEA 1: Randomize Ready Times
void Phase3Solver::enable_kitchen_lag_simulation() {
    std::mt19937 rng(42); // Fixed seed for reproducibility
    std::uniform_int_distribution<int> dist(5, 30); // 5 to 30 minutes prep time

    for (auto& order : orders) {
        order.ready_time = dist(rng);
    }
    std::cout << "[Simulation] Kitchen Lag Enabled: Orders now have random prep times." << std::endl;
}

// IDEA 2: Dynamic Breakdown
std::vector<Driver> Phase3Solver::simulate_breakdown(std::vector<Driver> current_fleet, int driver_to_fail) {
    if (driver_to_fail < 0 || driver_to_fail >= current_fleet.size()) {
        std::cerr << "[Error] Invalid driver ID for breakdown." << std::endl;
        return current_fleet;
    }

    std::cout << "[Simulation] Driver " << driver_to_fail << " broke down! Reassigning orders..." << std::endl;

    // 1. Mark driver as inactive
    current_fleet[driver_to_fail].active = false;

    // 2. Collect their orders
    std::vector<Order> orphaned_orders = current_fleet[driver_to_fail].assigned_orders;
    
    // Clear the broken driver's route
    current_fleet[driver_to_fail].assigned_orders.clear();
    current_fleet[driver_to_fail].route.clear();

    // 3. Reassign using the same greedy logic
    int rescued_count = 0;
    for (const auto& order : orphaned_orders) {
        if (attempt_insert_order(order, current_fleet)) {
            rescued_count++;
        } else {
            std::cerr << "  -> Failed to rescue Order " << order.order_id << std::endl;
        }
    }
    std::cout << "  -> Rescued " << rescued_count << "/" << orphaned_orders.size() << " orders." << std::endl;

    return current_fleet;
}

// IDEA 4: Order Cancellation
std::vector<Driver> Phase3Solver::simulate_cancellation(std::vector<Driver> current_fleet, int order_id_to_cancel) {
    std::cout << "[Simulation] Customer cancelled Order " << order_id_to_cancel << "." << std::endl;
    
    bool found = false;
    for (auto& driver : current_fleet) {
        if (!driver.active) continue;

        // Find order in assigned list
        auto it = std::find_if(driver.assigned_orders.begin(), driver.assigned_orders.end(), 
                               [&](const Order& o){ return o.order_id == order_id_to_cancel; });
        
        if (it != driver.assigned_orders.end()) {
            Order o = *it;
            driver.assigned_orders.erase(it);
            
            // Remove nodes from route (Pickup and Dropoff)
            driver.route.erase(std::remove(driver.route.begin(), driver.route.end(), o.pickup_node), driver.route.end());
            driver.route.erase(std::remove(driver.route.begin(), driver.route.end(), o.dropoff_node), driver.route.end());
            
            // Recalculate this driver's time
            auto res = evaluate_route(driver.route, driver.assigned_orders);
            driver.total_time_traveled = res.second;
            
            std::cout << "  -> Removed from Driver " << driver.driver_id << "'s route." << std::endl;
            found = true;
            break;
        }
    }

    if (!found) {
        std::cout << "  -> Order " << order_id_to_cancel << " was not found in current schedule." << std::endl;
    }

    return current_fleet;
}
