#include"RegLC/reglc_graph.h"
#include"RegLC/AlgoTypedefs.h"
#include "DataStructures/GraphFactory.h" 
//#include "MultipleParticipants/run_configurations.h"

RLC::DFA dfa_car=RLC::car_dfa();

using namespace Algo;

void run_test(){
  
  Transport::GraphFactory gf("/home/matchi/Desktop/ST/hercule.txt-dump", false);
  gf.setAll2();
  const Transport::Graph * g = gf.get();
  
  RLC::Graph *ag=new RLC::Graph( g,dfa_car );


  //NodeSet *ns=new NodeSet(10);
  
   Area * pickup=new Area(g, 35897);
  
  for(int i=0; i<35897;i++){
    pickup->add_node(i);
  }
 
  pickup->init();
  
  typedef TargetArea tg;
  
  tg::ParamType p(
    RLC::DRegLCParams( ag, 0, 1 ),
		RLC::AspectTargetAreaStopParams(pickup));
  
  tg dij(p);
  
    
    BOOST_FOREACH( int state, ag->dfa_start_states() ) {
        dij.add_source_node( RLC::Vertice(89, state), 0, 0 );
    }
    
    while( !dij.finished() ) {
        cout<<"yo"<<endl;
        RLC::Label lab = dij.treat_next();
	int n=lab.node.first;
	
	//bool ck=std::find(nl.begin(), nl.end(), n) != nl.end();
	//bool ck= pickup->isIn( lab.node.first );
	
	/*if (ck){
	  cout<<"node: "<<n<<" cost: "<<lab.cost<<endl;	  
	}*/
    }
  
}
