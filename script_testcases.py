import json, random, math, os, argparse

# -----------------------------------
# CONSTANTS
# -----------------------------------
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
            "description": "Auto-generated graph for routing tests"
        },
        "nodes": nodes,
        "edges": edges
    }

# -----------------------------------
# QUERY GENERATOR
# -----------------------------------
def generate_queries(num_nodes, num_edges, phase):
    events = []
    eid1 = random.randint(0, num_edges - 1)
    eid2 = random.randint(0, num_edges - 1)
    src = random.randint(0, num_nodes - 1)
    tgt = random.randint(0, num_nodes - 1)
    while tgt == src:
        tgt = random.randint(0, num_nodes - 1)

    # remove_edge query
    events.append({
        "id": 1,
        "type": "remove_edge",
        "edge_id": 1000 + eid1
    })

    # modify_edge query
    events.append({
        "id": 2,
        "type": "modify_edge",
        "edge_id": 1000 + eid2,
        "patch": {
            "length": round(random.uniform(100, 400), 2),
            "average_time": round(random.uniform(10, 50), 2),
            "road_type": random.choice(ROAD_TYPES)
        }
    })

    # shortest_path query (always includes all fields)
    events.append({
        "id": 3,
        "type": "shortest_path",
        "source": src,
        "target": tgt,
        "mode": random.choice(["time", "distance"]),
        "constraints": {
            "forbidden_nodes": random.sample(range(num_nodes), k=min(2, num_nodes//10 or 1)),
            "forbidden_road_types": random.sample(ROAD_TYPES, k=random.randint(0, 2))
        }
    })

    # knn query
    events.append({
        "id": 4,
        "type": "knn",
        "poi": random.choice(POIS),
        "query_point": {
            "lat": 19.05 + random.random() * 0.1,
            "lon": 72.85 + random.random() * 0.1
        },
        "k": random.randint(1, 5),
        "metric": random.choice(["shortest_path", "euclidean"])
    })

    if phase == 2:
        # k_shortest_paths (exact)
        events.append({
            "id": 5,
            "type": "k_shortest_paths",
            "source": random.randint(0, num_nodes - 1),
            "target": random.randint(0, num_nodes - 1),
            "k": random.randint(2, 5),
            "mode": "distance"
        })

        # k_shortest_paths_heuristic
        events.append({
            "id": 6,
            "type": "k_shortest_paths_heuristic",
            "source": random.randint(0, num_nodes - 1),
            "target": random.randint(0, num_nodes - 1),
            "k": random.randint(2, 5),
            "overlap_threshold": random.randint(50, 80)
        })

        # approx shortest path
        batch_queries = [
            {
                "source": random.randint(0, num_nodes - 1),
                "target": random.randint(0, num_nodes - 1)
            } for _ in range(3)
        ]
        events.append({
            "id": 7,
            "type": "approx_shortest_path",
            "queries": batch_queries,
            "time_budget_ms": random.randint(5, 20),
            "acceptable_error_pct": random.choice([5.0, 10.0, 15.0])
        })

    return {
        "meta": {"id": "qset_phase_" + str(phase)},
        "events": events
    }

# -----------------------------------
# MAIN
# -----------------------------------
def main():
    parser = argparse.ArgumentParser(description="Generate consistent graph + query testcases for Phase 1/2")
    parser.add_argument("--outdir", type=str, default="autotests")
    parser.add_argument("--nodes", type=int, default=30)
    parser.add_argument("--edges", type=int, default=60)
    parser.add_argument("--phase", type=int, choices=[1, 2], default=1)
    args = parser.parse_args()

    os.makedirs(args.outdir, exist_ok=True)
    graph = generate_graph(args.nodes, args.edges)
    queries = generate_queries(args.nodes, args.edges, args.phase)

    with open(os.path.join(args.outdir, "graph.json"), "w") as f:
        json.dump(graph, f, indent=4)
    with open(os.path.join(args.outdir, "queries.json"), "w") as f:
        json.dump(queries, f, indent=4)

    print(f"✅ Generated graph.json and queries.json for Phase {args.phase} in {args.outdir}")

if __name__ == "__main__":
    main()
