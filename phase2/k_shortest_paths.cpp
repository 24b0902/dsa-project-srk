#include <bits/stdc++.h>
#include "graph.h"
using namespace std;

vector<int> dijkstra_path(const Graph &graph, int src, int dest, double &dist_out) {
    unordered_map<int, double> dist;
    unordered_map<int, int> parent;

    for (const auto& pair : graph.get_all_nodes()) {
        dist[pair.first] = 1e9;
    }
    dist[src] = 0.0;

    using P = pair<double, int>;
    priority_queue<P, vector<P>, greater<P>> pq;
    pq.push({0.0, src});

    while (!pq.empty()) {
        auto [d, u] = pq.top(); pq.pop();
        if (d > dist[u]) continue;
        if (u == dest) break;

        for (Edge* edge : graph.get_neighbors(u)) {
            if (!edge->is_active) continue;
            int v = edge->v;
            double nd = d + edge->length;
            if (nd < dist[v]) {
                dist[v] = nd;
                parent[v] = u;
                pq.push({nd, v});
            }
        }
    }

    vector<int> path;
    if (dist[dest] == 1e9) {
        dist_out = -1; 
        return path;
    }

    for (int cur = dest; cur != src; cur = parent[cur])
        path.push_back(cur);
    path.push_back(src);
    reverse(path.begin(), path.end());

    dist_out = dist[dest];
    return path;
}

vector<pair<vector<int>, double>> k_shortest_paths(Graph &graph, int src, int dest, int k, const string &mode) {
    
    if (mode != "distance") {
        cerr << " Error: Only 'distance' mode is supported in K-shortest paths.\n";
        return {};
    }

    vector<pair<vector<int>, double>> results;

    double dist;
    vector<int> first = dijkstra_path(graph, src, dest, dist);
    if (first.empty()) return results; 
    results.push_back({first, dist});

    for (int i = 1; i < k; i++) {
        vector<int> prev_path = results.back().first;
        vector<int> best_path;
        double best_length = 1e9;

        for (size_t j = 0; j + 1 < prev_path.size(); j++) {
            int u = prev_path[j];
            int v = prev_path[j + 1];

            Edge* removed = nullptr;
            for (Edge* e : graph.get_neighbors(u)) {
                if (e->v == v && e->is_active) {
                    removed = e;
                    e->is_active = false; 
                    break;
                }
            }

            double new_dist;
            vector<int> new_path = dijkstra_path(graph, src, dest, new_dist);
            if (!new_path.empty() && new_dist < best_length) {
                best_path = new_path;
                best_length = new_dist;
            }

            if (removed) removed->is_active = true;
        }

        if (best_path.empty()) break; 
        results.push_back({best_path, best_length});
    }

    return results;
}
