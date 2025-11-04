#include <iostream>
#include <vector>
#include <string>
#include <ordered_map>
#include <utility>

#ifndef GRAPH_H
#define GRAPH_H

class Node{
        int id;
        double lat,lon;
        public:
        std::vector<std::string> pois;
        std::vector<int> neighbours;
        Node(int id,double lat,double lon,std::vector<std::string>  pos);
    };
    class Edge{
        public:
    int id;
    std::pair<int,int> v;
    double length,average_time;
    std::vector<int> speed_profile;
    bool oneway;
    std::string roadway;

    Edge(int id,int u,int v,double length,double avg_time,std::vector<int> profile,bool oneway,std::string roadway);
    
    };
    class Graph{
        public:
        int nodes_num;
        std::vector<int> nodes;
        std::vector<int> Edges;
        std::ordered_map<int,Node> node_map;
        std::ordered_map<std::pair<int,int>,Edge> edge_map;
        std::vector<int> get_neighbour(int u);
        Node get_node(int i);
        Edge get_edge(int u,int v);

    };
    #endif