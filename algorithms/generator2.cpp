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
    
   
      gf.set_pickUp(i);
    
  }
}

int main() 
{

    Transport::GraphFactory gf("/home/matchi/Desktop/ST/aum_good.txt-dump", false);
      
    
	GraphModeler(gf, 1,2, 50);
	
    
    const Transport::Graph * g = gf.get();

    cout <<"nombre de points d'échange: "<<g->PickUpZone().size()<<endl;
      
    gf.save_to_txt("/home/matchi/Desktop/ST/aum_good3.txt-dump");
      
      
    
    
}
