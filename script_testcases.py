import json, random, math, os, argparse

ROAD_TYPES = ["primary", "secondary", "tertiary", "local", "expressway"]
POIS = ["restaurant", "hospital", "pharmacy", "hotel", "atm", "petrol station"]

# -----------------------------------
# GRAPH GENERATOR
# -----------------------------------
def generate_graph(num_nodes, num_edges):
    nodes = []
    for i in range(num_nodes):
        lat = 19.0 + random.random() * 0.2
        lon = 72.8 + random.random() * 0.2
        pois = random.sample(POIS, random.randint(0, 2))
        nodes.append({
            "id": i,
            "lat": lat,
            "lon": lon,
            "pois": pois
        })

    edges = []
    edge_set = set()
    while len(edges) < num_edges:
        u, v = random.sample(range(num_nodes), 2)
        if (u, v) in edge_set or (v, u) in edge_set:
            continue
        edge_set.add((u, v))

        length = round(random.uniform(50, 500), 2)
        avg_time = round(length / random.uniform(5, 25), 2)
        speed_profile = [round(random.uniform(20, 60), 2) for _ in range(96)]

        edges.append({
            "id": 1000 + len(edges),
            "u": u,
            "v": v,
            "length": length,
            "average_time": avg_time,
            "speed_profile": speed_profile,
            "oneway": random.choice([True, False]),
            "road_type": random.choice(ROAD_TYPES)
        })

    return {
        "meta": {
            "id": "autogen_graph",
            "nodes": num_nodes,
            "description": "Auto-generated graph for full system test"
        },
        "nodes": nodes,
        "edges": edges
    }

# -----------------------------------
# QUERY GENERATOR (Phase 1 + Phase 2)
# -----------------------------------
def generate_all_queries(num_nodes, num_edges, num_queries):
    events = []

    query_types = [
        "remove_edge",
        "modify_edge",
        "shortest_path",
        "knn",
        "k_shortest_paths",
        "k_shortest_paths_heuristic",
        "approx_shortest_path"
    ]

    for qid in range(1, num_queries + 1):
        qtype = random.choice(query_types)

        if qtype == "remove_edge":
            events.append({
                "id": qid,
                "type": qtype,
                "edge_id": 1000 + random.randint(0, num_edges - 1)
            })

        elif qtype == "modify_edge":
            events.append({
                "id": qid,
                "type": qtype,
                "edge_id": 1000 + random.randint(0, num_edges - 1),
                "patch": {
                    "length": round(random.uniform(100, 400), 2),
                    "average_time": round(random.uniform(10, 50), 2),
                    "road_type": random.choice(ROAD_TYPES)
                }
            })

        elif qtype == "shortest_path":
            s, t = random.sample(range(num_nodes), 2)
            events.append({
                "id": qid,
                "type": qtype,
                "source": s,
                "target": t,
                "mode": random.choice(["time", "distance"]),
                "constraints": {
                    "forbidden_nodes": random.sample(range(num_nodes),
                                                     k=min(2, max(1, num_nodes // 10))),
                    "forbidden_road_types": random.sample(
                        ROAD_TYPES, k=random.randint(0, 2)
                    )
                }
            })

        elif qtype == "knn":
            events.append({
                "id": qid,
                "type": qtype,
                "poi": random.choice(POIS),
                "query_point": {
                    "lat": 19.05 + random.random() * 0.1,
                    "lon": 72.85 + random.random() * 0.1
                },
                "k": random.randint(1, 5),
                "metric": random.choice(["shortest_path", "euclidean"])
            })

        elif qtype == "k_shortest_paths":
            s, t = random.sample(range(num_nodes), 2)
            events.append({
                "id": qid,
                "type": qtype,
                "source": s,
                "target": t,
                "k": random.randint(2, 5),
                "mode": random.choice(["distance", "time"])
            })

        elif qtype == "k_shortest_paths_heuristic":
            s, t = random.sample(range(num_nodes), 2)
            events.append({
                "id": qid,
                "type": qtype,
                "source": s,
                "target": t,
                "k": random.randint(2, 5),
                "overlap_threshold": random.randint(40, 80)
            })

        elif qtype == "approx_shortest_path":
            queries = [
                {
                    "source": random.randint(0, num_nodes - 1),
                    "target": random.randint(0, num_nodes - 1)
                }
                for _ in range(random.randint(2, 5))
            ]
            events.append({
                "id": qid,
                "type": qtype,
                "queries": queries,
                "time_budget_ms": random.randint(10, 30),
                "acceptable_error_pct": random.choice([5.0, 10.0, 15.0])
            })

    return {
        "meta": {"id": "qset_combined"},
        "events": events
    }

# -----------------------------------
# MAIN
# -----------------------------------
def main():
    parser = argparse.ArgumentParser(description="Generate full mixed testcases (Phase 1 + 2)")
    parser.add_argument("--outdir", type=str, default="mixed_tests")
    parser.add_argument("--nodes", type=int, default=50)
    parser.add_argument("--edges", type=int, default=100)
    parser.add_argument("--queries", type=int, default=20)   # <-- NEW ARGUMENT 👍
    args = parser.parse_args()

    os.makedirs(args.outdir, exist_ok=True)

    graph = generate_graph(args.nodes, args.edges)
    queries = generate_all_queries(args.nodes, args.edges, args.queries)

    with open(os.path.join(args.outdir, "graph.json"), "w") as f:
        json.dump(graph, f, indent=4)

    with open(os.path.join(args.outdir, "queries.json"), "w") as f:
        json.dump(queries, f, indent=4)

    print(f"✅ Generated testcases in {args.outdir}")
    print(f"➡ Nodes: {args.nodes}, Edges: {args.edges}, Queries: {args.queries}")

if __name__ == "__main__":
    main()
