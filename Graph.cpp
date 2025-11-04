#include "Graph.h"

Node:: Node(int id,double lat,double lon,std::vector<std::string>  pos){
            this->id=id;
            this->lat=lat;
            this->lon=lon;
            this->pois=pos;


        }
Edge:: Edge(int id,int x,int y,double length,double avg_time,std::vector<int> profile,bool oneway,std::string roadway){
        this->id=id;
        this->v=make_pair(x,y);
        this->length=length;
        this->average_time=avg_time;
        this->speed_profile=profile;
        this->oneway=oneway;
        this->roadway=roadway;

    }
std::vector<int> Graph:: get_neighbour(int u){
    return node_map[u].neighbours;
}
Node Graph:: get_node(int u){
            return node_map[u];
        }
Edge Graph::get_edge(int u,int v){
    return edge_map[{u,v}];
}