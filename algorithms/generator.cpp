#include <iostream>     // cout, endl
#include <fstream>      // fstream
#include <vector>
#include <string>
#include <iterator>     // ostream_operator
#include <boost/tokenizer.hpp>

#include "../lib/core/graph_wrapper.h"
#include "DataStructures/GraphFactory.h"

#include "utils/GeoTools.h"

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

void GraphModeler(Transport::GraphFactory gf, float lon, float lat, double radius){
  
  for(int i=0; i<gf.total_vertices(); i++){
    Node n=gf.getNode(i);
    if(proximity(n.lon,n.lat,lon,lat,radius)) {
      gf.set_pickUp(i);
    }
  }
}
*/

void GraphModeler(Transport::GraphFactory gf, float lon, float lat, double radius){
  
  for(int i=0; i<gf.total_vertices(); i++){
    Node n=gf.getNode(i);
    if(proximity(n.lon,n.lat,lon,lat,radius)) {
      gf.set_pickUp(i);
    }
  }
}

int main() 
{

    Transport::GraphFactory gf("/home/matchi/Desktop/ST/posseidon.txt-dump", false);
      
    string data("/home/matchi/Desktop/ST/cov.csv");

    ifstream in(data.c_str());
    if (!in.is_open()) return 1;

    typedef tokenizer< escaped_list_separator<char> > Tokenizer;

    vector< string > vec;
    string line;
    int i=0;

    while (getline(in,line))
    {
      cout <<i<<endl;
      i++;
        Tokenizer tok(line);
        vec.assign(tok.begin(),tok.end());
	
	string lo=vec.at(0);
	string la=vec.at(1);
	
        float lon=std::stof(lo);
	float lat=std::stof(la);
	
	cout <<"point échange: "<<lat<<endl;
	GraphModeler(gf, lon,lat, 100);
	
    }
    const Transport::Graph * g = gf.get();

    cout <<"nombre de points d'échange: "<<g->PickUpZone().size()<<endl;
      
    gf.save_to_txt("/home/matchi/Desktop/ST/hercule.txt-dump");
      
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
