
#include <stdlib.h>
#include <iostream>
using std::cerr;
using std::cout;
using std::endl;
#include <fstream>
using std::ifstream;
#include <time.h>
#include <boost/concept_check.hpp>
#include "MultipleParticipants/MPR_AspectPrivacy.h"

#include "GraphFactory.h"
#include "muparo.h"
#include "run_configurations.h"
#include "utils.h"
#include "node_filter_utils.h"
#include "../RegLC/AlgoTypedefs.h"


#include "../tests/JsonWriter.h"

#include"../RegLC/reglc_graph.h"


#include "MPR_AspectPrivacy.h"

using namespace MuPaRo;
using namespace AlgoMPR;
using namespace Privacy;

JsonWriter * out;



int main() 
{
      
      Transport::GraphFactory gf("/home/matchi/Desktop/ST/aum_good2.txt-dump", false);
      
      const Transport::Graph * g = gf.get();
      
      RLC::DFA dfa_passenger=RLC::pt_foot_dfa();
      RLC::DFA dfa_car=RLC::car_dfa();
      
     
      
      RLC::Graph *ag=new RLC::Graph( g,dfa_passenger );
      
      NodeSet *ns=isochrone (ag, 7, 36 );
      
      cout<<ns->bitset.size()<<endl;
      
      int compt=0;
      
      for(int i=0; i<g->num_vertices(); i++){
	Node k=g->mapNode(i);
	if (ns->bitset[i] && k.pick_up){
	  compt++;
	}
      }
      
      
     cout<<compt<<endl;
      
      /*Privacy::Pedestrian * p1=new Privacy::Pedestrian(150, 8653);
      Privacy::Driver * d1=new Privacy::Driver(784,32681);
      Privacy::Manager *m=new Privacy::Manager();
      
      //p1->findZ(g,100);
      d1->findZ(g,100);
      
      BiList psi_result=m->PSI(*d1,*p1);
      
      //cout<<"taille des pick up"<<psi_result.PickUp.size()<<endl;
      //cout<<"taille des drop off"<<psi_result.DropOff.size()<<endl;
      //cout<<"---------------------------------------------------------------"<<endl;
     // cout<<"pick up foot: "<<p1->data_before.size()<<endl;
      //cout<<"drop off foot: "<<p1->data_after.size()<<endl;
      cout<<"---------------------------------------------------------------"<<endl;
      cout<<"pick up  car: "<<d1->data_before.size()<<endl;
      cout<<"drop off car: "<<d1->data_after.size()<<endl;
      

      
     pathList shared_path=m->findA(g,psi_result);
      
      cout<<"pick up foot: "<<p1->data_before.size()<<endl;
      cout<<"drop off foot: "<<p1->data_after.size()<<endl;
      
      cout<<"pick up  car: "<<d1->data_before.size()<<endl;
      cout<<"drop off car: "<<d1->data_after.size()<<endl;
      
      cout<<"---resultat---------: "<<shared_path.size()<<endl;
      
      
      
       BOOST_FOREACH(path p, shared_path){
	 
	 cout<<"Path id: "<< p.id<<" | "<<"depart: "<<p.start<<" end: "<<p.end<<" cost: "<<p.cost<<endl;
	 
       }
       
       
      prefs p_car=d1->getFavorites(shared_path);
      prefs p_foot=p1->getFavorites(shared_path);
      
      BOOST_FOREACH(pref i, p_car){
	 
	 cout<<i.weight<<endl;
	 
       }
       cout<<"-------------------------------------------"<<endl;
        BOOST_FOREACH(pref i, p_foot){
	 
	 cout<<i.weight<<endl;
	 
       }
       
        cout<<"-------------------------------------------"<<endl;
	prefs z=m->match(* d1, * p1, shared_path);
	int o=shared_path.size();
	cout<<z[o-1].id<<endl;

       path p=m->getPathById(z[o-1].id, shared_path);
       
       
       cout<<"------------------------------------------"<<endl;
       cout <<"best pick up: "<<p.start<<endl;
       cout<<"best drop off: "<<p.end<<endl;
       cout<<"estimated car sharing travel time: "<<p.cost<<endl;
       
      
      JsonWriter writer("/home/matchi/Desktop/data.json");
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
      
      out->step_out();*/
    
    
    
}
