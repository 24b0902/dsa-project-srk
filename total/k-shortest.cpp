#include "graph.h"
#include "shortest_paths.h"
#include <queue>
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <set>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>

struct PathData {
    std::vector<int> nodes;
    double length;
    std::unordered_set<int> edge_ids; //for O(1) lookup during overlap checks

    // Operator < (Required for std::set uniqueness)
    bool operator<(const PathData& other) const {
        if (std::abs(length - other.length) > 1e-6){
            return length < other.length;
        }
        return nodes < other.nodes;
    }

    // Operator > (Required for std::greater in priority_queue)
    bool operator>(const PathData& other) const {
        if (std::abs(length - other.length) > 1e-6) return length > other.length;
        return nodes > other.nodes;
    }
};

// Fills length and edge_id cache for a path
void fill_path_metadata(Graph& graph, PathData& p_data) {
    p_data.length = 0.0;
    p_data.edge_ids.clear();
    if (p_data.nodes.size() < 2) return;

    for (size_t i = 0; i < p_data.nodes.size() - 1; i++) {
        int u = p_data.nodes[i];
        int v = p_data.nodes[i+1];
        //find edge u->v
        for (Edge* edge : graph.get_neighbors(u)) {
            if (edge->v == v) { 
                p_data.length += edge->length;
                p_data.edge_ids.insert(edge->id);
                break;
            }
        }
    }
}

double calculate_overlap_fast(const PathData& p1, const PathData& p2) {
    if (p1.edge_ids.empty() || p2.edge_ids.empty()) return 0.0;
    
    size_t intersection_size = 0;
    const auto& smaller = (p1.edge_ids.size() < p2.edge_ids.size()) ? p1.edge_ids : p2.edge_ids;
    const auto& larger = (p1.edge_ids.size() < p2.edge_ids.size()) ? p2.edge_ids : p1.edge_ids;

    for (int id : smaller) {
        if (larger.count(id)) intersection_size++;
    }
    // % overlap relative to p1
    return (double)intersection_size / p1.edge_ids.size() * 100.0;
}

// =========================================================
//  HEURISTIC K-SHORTEST PATHS (Optimized)
// =========================================================

double calculate_set_penalty(
    const std::vector<PathData>& selected_paths,
    const PathData& candidate,
    double shortest_length,
    double overlap_threshold
) {
    double total_penalty = 0.0;
    // Combine candidate with existing to check full set penalty
    std::vector<const PathData*> all_paths;
    for (const auto& p : selected_paths) all_paths.push_back(&p);
    all_paths.push_back(&candidate);

    for (const PathData* p1 : all_paths) {
        double dev_pct = (p1->length - shortest_length) / shortest_length;
        double dist_penalty = dev_pct + 0.1;

        int overlap_count = 0;
        for (const PathData* p2 : all_paths) {
            if (p1 == p2) continue;
            if (calculate_overlap_fast(*p1, *p2) > overlap_threshold) {
                overlap_count++;
            }
        }
        total_penalty += (double)overlap_count * dist_penalty;
    }
    return total_penalty;
}

json handle_k_shortest_paths_heuristic(Graph& graph, const json& query) {
    json result;
    result["id"] = query.value("id", -1);

    try {
        int source = query.at("source");
        int target = query.at("target");
        int k = query.at("k");
        double overlap_threshold = query.value("overlap_threshold", 0.0);

        std::vector<PathData> final_paths;

        // 1. Absolute Shortest Path
        auto res1 = minimising_distance(graph, source, target, {}, {});
        if (res1.first.empty()) {
            result["paths"] = json::array();
            return result;
        }

        PathData p1; p1.nodes = res1.first; fill_path_metadata(graph, p1);
        final_paths.push_back(p1);
        double shortest_len = p1.length;

        std::vector<PathData> paths_to_deviate = {p1};
        std::set<std::vector<int>> unique_paths_set;
        unique_paths_set.insert(p1.nodes);
        std::vector<PathData> candidates;

        // 2. Iterative Deviation & Selection
        while (final_paths.size() < k) {
            // A. Generate Deviations
            for (const auto& base_path : paths_to_deviate) {
                for (size_t i = 0; i < base_path.nodes.size() - 1; i++) {
                    int u = base_path.nodes[i];
                    int v = base_path.nodes[i+1];
                    std::vector<int> root(base_path.nodes.begin(), base_path.nodes.begin() + i + 1);
                    
                    Edge* disabled_edge = nullptr;
                    for (Edge* e : graph.get_neighbors(u)) {
                        if (e->v == v && e->is_active) { disabled_edge = e; break; }
                    }

                    if (disabled_edge) {
                        disabled_edge->is_active = false;
                        auto spur = minimising_distance(graph, u, target, {}, {});
                        disabled_edge->is_active = true;

                        if (!spur.first.empty()) {
                            std::vector<int> new_nodes = root;
                            new_nodes.insert(new_nodes.end(), spur.first.begin() + 1, spur.first.end());
                            if (unique_paths_set.find(new_nodes) == unique_paths_set.end()) {
                                PathData pd; pd.nodes = new_nodes; fill_path_metadata(graph, pd);
                                candidates.push_back(pd);
                                unique_paths_set.insert(new_nodes);
                            }
                        }
                    }
                }
            }
            paths_to_deviate.clear(); 
            if (candidates.empty()) break;

            // B. Selection (Optimized)
            std::sort(candidates.begin(), candidates.end(), [](const PathData& a, const PathData& b){
                return a.length < b.length;
            });

            int best_idx = -1;
            
            // Greedy Check: Find first candidate with 0 penalty overlap
            for (size_t i = 0; i < candidates.size(); i++) {
                bool clean = true;
                for (const auto& selected : final_paths) {
                    if (calculate_overlap_fast(candidates[i], selected) > overlap_threshold) {
                        clean = false; break;
                    }
                }
                if (clean) { best_idx = i; break; }
            }

            // Fallback: Minimize Total Penalty
            if (best_idx == -1) {
                double min_penalty = std::numeric_limits<double>::infinity();
                for (size_t i = 0; i < candidates.size(); i++) {
                    double p = calculate_set_penalty(final_paths, candidates[i], shortest_len, overlap_threshold);
                    if (p < min_penalty) { min_penalty = p; best_idx = i; }
                }
            }

            if (best_idx != -1) {
                final_paths.push_back(candidates[best_idx]);
                paths_to_deviate.push_back(candidates[best_idx]);
                candidates.erase(candidates.begin() + best_idx);
            } else {
                break;
            }
        }

        json paths = json::array();
        std::sort(final_paths.begin(), final_paths.end(), [](const PathData& a, const PathData& b){
            return a.length < b.length;
        });
        for (const auto& pd : final_paths) {
            paths.push_back({{"path", pd.nodes}, {"length", pd.length}});
        }
        result["paths"] = paths;

    } catch (const std::exception& e) {
        result["error"] = e.what();
    }
    return result;
}

// =========================================================
//  EXACT K-SHORTEST PATHS (Yen's)
// =========================================================

json handle_k_shortest_paths(Graph& graph, const json& query) {
    json result;
    result["id"] = query.value("id", -1);
    try {
        int source = query.at("source");
        int target = query.at("target");
        int k = query.at("k");

        auto res1 = minimising_distance(graph, source, target, {}, {});
        if (res1.first.empty()) {
            result["paths"] = json::array();
            return result;
        }

        std::vector<PathData> found_paths;
        PathData p1; p1.nodes = res1.first; fill_path_metadata(graph, p1);
        found_paths.push_back(p1);

        // Min-heap for candidates
        std::priority_queue<PathData, std::vector<PathData>, std::greater<PathData>> candidates_pq;
        std::set<std::vector<int>> unique;
        unique.insert(p1.nodes);

        // Process deviations from the last path added
        std::vector<PathData> to_process = {p1};

        while (found_paths.size() < k) {
            if (to_process.empty()) {
                if (candidates_pq.empty()) break;
                PathData next = candidates_pq.top(); candidates_pq.pop();
                found_paths.push_back(next);
                to_process.push_back(next);
                continue;
            }

            PathData base = to_process.back();
            to_process.pop_back();

            for (size_t i = 0; i < base.nodes.size() - 1; i++) {
                int u = base.nodes[i];
                int v = base.nodes[i+1];
                std::vector<int> root(base.nodes.begin(), base.nodes.begin() + i + 1);

                std::vector<Edge*> disabled;
                // Standard Yen's: Disable edges used by ANY path starting with this root
                for (const auto& fp : found_paths) {
                    if (fp.nodes.size() > i + 1 && std::equal(root.begin(), root.end(), fp.nodes.begin())) {
                        int next_node = fp.nodes[i+1];
                        for(Edge* e : graph.get_neighbors(u)) {
                            if(e->v == next_node && e->is_active) {
                                e->is_active = false;
                                disabled.push_back(e);
                                break; 
                            }
                        }
                    }
                }

                std::unordered_set<int> forbidden(root.begin(), root.end()-1);
                auto spur = minimising_distance(graph, u, target, forbidden, {});
                
                // Restore
                for(Edge* e : disabled) e->is_active = true;

                if (!spur.first.empty()) {
                    std::vector<int> total = root;
                    total.insert(total.end(), spur.first.begin() + 1, spur.first.end());
                    if (unique.find(total) == unique.end()) {
                        PathData cand; cand.nodes = total; fill_path_metadata(graph, cand);
                        candidates_pq.push(cand);
                        unique.insert(total);
                    }
                }
            }
            
            if (found_paths.size() < k && !candidates_pq.empty()) {
                PathData best = candidates_pq.top(); candidates_pq.pop();
                found_paths.push_back(best);
                to_process.push_back(best);
            }
        }

        json paths = json::array();
        for (const auto& p : found_paths) paths.push_back({{"path", p.nodes}, {"length", p.length}});
        result["paths"] = paths;

    } catch (const std::exception& e) {
        result["error"] = e.what();
    }
    return result;
}

std::pair<std::vector<int>, double> weighted_a_star(Graph& graph, int source, int target, double weight) {
    using P = std::pair<double, int>;
    std::priority_queue<P, std::vector<P>, std::greater<P>> pq;
    std::unordered_map<int, double> g_dist;
    std::unordered_map<int, int> parent;

    const Node& target_node = graph.get_node_by_id(target);
    const Node& source_node = graph.get_node_by_id(source);

    g_dist[source] = 0.0;
    parent[source] = -1;
    
    // Initial priority includes the weighted heuristic
    double h_start = heuristic_h(source_node, target_node);
    pq.push({weight * h_start, source});

    bool found = false;

    while (!pq.empty()) {
        double current_f = pq.top().first;
        int u = pq.top().second;
        pq.pop();

        if (u == target) {
            found = true;
            break;
        }

        // Optimization: Skip if we found a shorter path to 'u' already
        if (g_dist.count(u) && current_f > g_dist.at(u) + weight * heuristic_h(graph.get_node_by_id(u), target_node)) {
            continue;
        }

        for (Edge* edge_ptr : graph.get_neighbors(u)) {
            if (!edge_ptr->is_active){
                continue;
            }

            double new_g = g_dist.at(u) + edge_ptr->length;
            int v = edge_ptr->v;

            if (!g_dist.count(v) || new_g < g_dist.at(v)) {
                g_dist[v] = new_g;
                parent[v] = u;

                double h_v = heuristic_h(graph.get_node_by_id(v), target_node);
                double f_v = new_g + (weight * h_v);
                
                pq.push({f_v, v});
            }
        }
    }

    if (found) {
        return {reconstruct_path(target, parent), g_dist.at(target)};
    }
    return {{}, -1.0};
}

json handle_approx_shortest_paths(Graph& graph, const json& query) {
    json result;
    // Use .at() to ensure ID exists; catch block in driver will handle errors
    result["id"] = query.at("id");

    try {
        // TUNING PARAMETER (Weighted A*)
        double weight = 5.0; 
        if (query.contains("epsilon")) weight = query["epsilon"]; // Using epsilon for weight

        // CASE 1: Batch Query
        if (query.contains("queries")) {
            json results_array = json::array();
            auto queries_list = query.at("queries");

            for (const auto& sub_query : queries_list) {
                int s = sub_query.at("source");
                int t = sub_query.at("target");

                auto res = weighted_a_star(graph, s, t, weight);

                json sub_result;
                // Output must match spec for batch: distance, path
                sub_result["source"] = s;
                sub_result["target"] = t;

                if (!res.first.empty()) {
                    sub_result["approx_shortest_distance"] = res.second;
                } else {
                    sub_result["approx_shortest_distance"] = -1.0;
                }
                results_array.push_back(sub_result);
            }
            result["results"] = results_array;
        } 
        // CASE 2: Single Query (Legacy support)
        else {
            int source = query.at("source");
            int target = query.at("target");

            auto res = weighted_a_star(graph, source, target, weight);

            if (!res.first.empty()) {
                result["possible"] = true;
                result["approximate_distance"] = res.second;
            } else {
                result["possible"] = false;
            }
        }

    } catch (const std::exception& e) {
        result["error"] = e.what();
    }
    return result;
}