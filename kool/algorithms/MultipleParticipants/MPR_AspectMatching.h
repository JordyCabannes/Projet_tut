#include <iostream>
#include <cstdlib>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/cycle_canceling.hpp>
#include <boost/graph/edmonds_karp_max_flow.hpp>

#include "MPR_AspectPrivacy.h"

namespace boost {

struct SampleGraph {

typedef adjacency_list_traits < vecS, vecS, directedS > Traits;

typedef adjacency_list < vecS, vecS, directedS, no_property,
property < edge_capacity_t, long,
property < edge_residual_capacity_t, long,
property < edge_reverse_t, Traits::edge_descriptor,
property <edge_weight_t, long>
>
>
> > Graph;
typedef property_map < Graph, edge_capacity_t >::type Capacity;
typedef property_map < Graph, edge_residual_capacity_t >::type ResidualCapacity;
typedef property_map < Graph, edge_weight_t >::type Weight;
typedef property_map < Graph, edge_reverse_t>::type Reversed;
typedef boost::graph_traits<Graph>::vertices_size_type size_type;
typedef Traits::vertex_descriptor vertex_descriptor;

class EdgeAdder {
public:
EdgeAdder(Graph & g, Weight & w, Capacity & c, Reversed & rev, ResidualCapacity & residualCapacity)
: m_g(g), m_w(w), m_cap(c), m_resCap(residualCapacity), m_rev(rev) {}
void addEdge(vertex_descriptor v, vertex_descriptor w, long weight, long capacity) {
Traits::edge_descriptor e,f;
e = add(v, w, weight, capacity);
f = add(w, v, -weight, 0);
m_rev[e] = f;
m_rev[f] = e;
}
private:
Traits::edge_descriptor add(vertex_descriptor v, vertex_descriptor w, long weight, long capacity) {
bool b;
Traits::edge_descriptor e;
boost::tie(e, b) = add_edge(vertex(v, m_g), vertex(w, m_g), m_g);
if (!b) {
std::cerr << "Edge between " << v << " and " << w << " already exists." << std::endl;
std::abort();
}
m_cap[e] = capacity;
m_w[e] = weight;
return e;
}
Graph & m_g;
Weight & m_w;
Capacity & m_cap;
ResidualCapacity & m_resCap;
Reversed & m_rev;
};

static void carpoolGraph(Graph &g, vertex_descriptor & s, vertex_descriptor & t) {
size_type N(7);
typedef property_map < Graph, edge_reverse_t >::type Reversed;
for(size_type i = 0; i < N; ++i){
add_vertex(g);
}
Capacity capacity = get(edge_capacity, g);
Reversed rev = get(edge_reverse, g);
ResidualCapacity residual_capacity = get(edge_residual_capacity, g);
Weight weight = get(edge_weight, g);
s = 0;
t = 6;

EdgeAdder ea(g, weight, capacity, rev, residual_capacity);
ea.addEdge(0, 1, 0 ,1);
ea.addEdge(0, 2, 0 ,1);
ea.addEdge(0, 5, 0 ,1);


ea.addEdge(1, 3, 5 ,1);
ea.addEdge(1, 4, 10 ,1);

ea.addEdge(2, 3, 10 ,1);
ea.addEdge(2, 4, 6 ,1);

ea.addEdge(5, 4, 8 ,1);

ea.addEdge(3, 6, 0 ,1);
ea.addEdge(4, 6, 0 ,1);
}

//static void bipartiteGraphBuilder(std::vector<Privacy::Driver> drivers, std::vector<Privacy::Pedestrian> passengers, Graph &g, vertex_descriptor & s, vertex_descriptor & t){
static void bipartiteGraphBuilder(int m, int n, Graph &g, vertex_descriptor & s, vertex_descriptor & t){

//int m=drivers.size();
//int n=passengers.size();

size_type N(m+n+2);

typedef property_map < Graph, edge_reverse_t >::type Reversed;

for(size_type i = 0; i < N; ++i){
add_vertex(g);
}
Capacity capacity = get(edge_capacity, g);
Reversed rev = get(edge_reverse, g);
ResidualCapacity residual_capacity = get(edge_residual_capacity, g);
Weight weight = get(edge_weight, g);
s = 0;
t = m+n+1;

EdgeAdder ea(g, weight, capacity, rev, residual_capacity);

for(vertex_descriptor v=1; v<=m; ++v){
    ea.addEdge(s, v, 0 ,1);
}

for(vertex_descriptor v=m+1; v<=n; ++v){
    ea.addEdge(v, t, 0 ,1);
}

for(vertex_descriptor i=1; i<=m; ++i){
    for(vertex_descriptor j=m+1; j<=n; ++j){
        ea.addEdge(i, j, 10 ,1);
    }
}


}



};


} //boost