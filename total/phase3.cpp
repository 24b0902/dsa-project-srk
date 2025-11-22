#include "phase3.h"
#include <queue>
#include <algorithm>
#include <iostream>
#include <limits>
#include <map>

// Use standard infinity for cleaner logic
const double INF = 1e15; 

// --- Constructor ---
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

// --- Helper: Internal Dijkstra ---
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

        // Get Neighbors via Circular Lookup (InternalIdx -> RealID -> Neighbors)
        int u_real_id = graph.get_all_nodes()[u_idx].id;
        const std::vector<Edge*>& neighbors = graph.get_neighbors(u_real_id);

        for (Edge* edge : neighbors) {
            if (!edge->is_active) continue;

            double weight = edge->average_time;
            int v_real_id = edge->v;
            
            try {
                // Optimization: In a real scenario, we would cache this ID->Idx lookup
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
    
    int unreachable_pairs = 0;

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
            
            if (time_matrix[i][j] >= INF && i != j) unreachable_pairs++;
        }
    }
    
    // DIAGNOSTIC: Check if Depot is disconnected
    int dep_idx = node_id_to_matrix_idx[depot_node];
    int reachable_from_depot = 0;
    for(int j=0; j<n; ++j) {
        if(time_matrix[dep_idx][j] < INF) reachable_from_depot++;
    }
    
    if (reachable_from_depot <= 1 && n > 1) {
        std::cerr << "[WARNING] Depot Node " << depot_node << " cannot reach any POIs! Graph may be disconnected or one-way." << std::endl;
    }
}

// --- Logic: Evaluate a Route ---
std::pair<double, double> Phase3Solver::evaluate_route(const std::vector<int>& route, const std::vector<Order>& route_orders) {
    if (route.empty()) return {0.0, 0.0};

    double current_time = 0.0;
    double sum_arrival_times = 0.0;
    int current_node = depot_node; // Route strictly starts from Depot

    // If route[0] is depot, skip it in iteration to avoid Depot->Depot cost
    size_t start_k = (route.size() > 0 && route[0] == depot_node) ? 1 : 0;

    for (size_t k = start_k; k < route.size(); ++k) {
        int next_node = route[k];
        
        int u_idx = node_id_to_matrix_idx[current_node];
        int v_idx = node_id_to_matrix_idx[next_node];
        
        double travel_time = time_matrix[u_idx][v_idx];
        
        // Safety check for infinite paths
        if (travel_time >= INF) {
            // Return valid but huge cost to discourage this route
            return {1e14, 1e14}; 
        }
        
        current_time += travel_time;

        for(const auto& o : route_orders) {
            if(o.dropoff_node == next_node) {
                sum_arrival_times += current_time;
            }
        }
        current_node = next_node;
    }
    return {sum_arrival_times, current_time};
}

// --- Main Solver ---
std::vector<Driver> Phase3Solver::solve() {
    std::vector<Driver> fleet(num_drivers);
    for (int i = 0; i < num_drivers; ++i) {
        fleet[i].driver_id = i;
        fleet[i].route.push_back(depot_node); 
    }

    std::vector<Order> pending_orders = orders;
    int dep_idx = node_id_to_matrix_idx[depot_node];
    
    // Sort by distance, but put unreachable orders at the end
    std::sort(pending_orders.begin(), pending_orders.end(), [&](const Order& a, const Order& b) {
        double distA = time_matrix[dep_idx][node_id_to_matrix_idx[a.pickup_node]];
        double distB = time_matrix[dep_idx][node_id_to_matrix_idx[b.pickup_node]];
        if (distA >= INF && distB < INF) return false; // A is unreachable, put after B
        if (distA < INF && distB >= INF) return true;
        return distA > distB; // Farthest reachable first
    });

    int unassigned_count = 0;

    for (const auto& order : pending_orders) {
        double best_cost_increase = INF;
        int best_driver = -1;
        std::vector<int> best_route_seq;

        for (int d = 0; d < num_drivers; ++d) {
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
                    
                    // If route is invalid/unreachable, evaluate_route returns huge numbers
                    if (metrics.second >= 1e13) continue;

                    double cost = minimize_max_time ? metrics.second : metrics.first;

                    // Logic for Min Max: we compare against the NEW max of the fleet
                    if (minimize_max_time) {
                        double max_t = metrics.second;
                        for(int k=0; k<num_drivers; ++k) {
                            if(k != d) max_t = std::max(max_t, fleet[k].total_time_traveled);
                        }
                        cost = max_t;
                    } else {
                        // For Min Sum, we just want the smallest increase in this driver's sum
                        // We can approximate by using the raw sum of the new route
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
        } else {
            unassigned_count++;
        }
    }
    
    if (unassigned_count > 0 && !minimize_max_time) {
        std::cout << "[Warning] " << unassigned_count << " orders could not be assigned (likely unreachable)." << std::endl;
    }

    return fleet;
}

SolutionMetrics Phase3Solver::get_final_metrics(const std::vector<Driver>& fleet) {
    SolutionMetrics m;
    m.total_delivery_time = 0;
    m.max_delivery_time = 0;

    for (const auto& d : fleet) {
        auto res = evaluate_route(d.route, d.assigned_orders);
        m.total_delivery_time += res.first;
        m.max_delivery_time = std::max(m.max_delivery_time, res.second);
    }
    return m;
}