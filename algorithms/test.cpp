#include <iostream>     // cout, endl

#include "../lib/core/graph_wrapper.h"
#include "DataStructures/GraphFactory.h"

using namespace std;
using namespace boost;

/*
#include <stdlib.h>

#include "../lib/core/graph_wrapper.h"
#include "utils/GeoTools.h"

#include "DataStructures/GraphFactory.h"
#include "muparo.h"
#include "run_configurations.h"
#include "utils.h"
#include "node_filter_utils.h"
#include "../RegLC/AlgoTypedefs.h"


#include "../tests/JsonWriter.h"

#include"../RegLC/reglc_graph.h"

#include <boost/tokenizer.hpp>

using namespace std;
using namespace boost;

JsonWriter * out;


*/



int main() 
{

  Transport::GraphFactory gf("/home/matchi/Desktop/ST/hercule.txt-dump", false);
      
    
    const Transport::Graph * g = gf.get();

    cout <<"nombre de points d'échange: "<<g->PickUpZone().size()<<endl;
      
    
      
      /*

      
      JsonWriter writer("/home/matchi/Desktop/ST/viz.json");
      out = &writer;
      out->step_in();
    
  
      
      out->add("bus edges:", (int)g->listEdges(BusEdge).size());
      out->add("whatever edges:", (int)g->listEdges(WhateverEdge).size());
      out->add("bike edges:", (int)g->listEdges(BikeEdge).size());
      out->add("foot edges:", (int)g->listEdges(FootEdge).size());
      out->add("car edges:", (int)g->listEdges(CarEdge).size());
      out->add("subway edges:", (int)g->listEdges(SubwayEdge).size());
      out->add("tram edges:", (int)g->listEdges(TramEdge).size());
      out->add("transfer edges:", (int)g->listEdges(TransferEdge).size());
      out->add("unknown edges:", (int)g->listEdges(UnknownEdgeType).size());
      out->add("number of vertices:", g->num_vertices());
      
      for(int i=0; i<10; i++){
	Node n=g->mapNode(i);
	
	out->step_in();
	out->add("longitude", n.lon);
	out->add("latitude", n.lat);
	out->step_out();
	
      }
      
      double dist=haversine_dist(1.44543,43.5619,1.44525,43.5619);
            out->add("haversine distance in meter", dist);

      out->step_out();*/
    
    
    
}
