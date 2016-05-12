#pragma once
#include "header.h"
#include "aux_data.h"

/*
 * Model for Driver
 */ 

using namespace MuPaRo;

namespace Privacy {  
	class Driver{
	  
	  public:
	    
	  int posStart, posEnd;
	  
	  dataList data_before, data_after, dBefore, dAfter;
	  prefs favorites;

	  Driver(){
	    
	  }

	  Driver(int start, int end){
	    
	    posStart=start;
	    posEnd=end; 
	  }

	  ~Driver(){}

	  journey getMyJourney(const Transport::Graph * g){

	    journey jr;

	    Node start=g->mapNode(posStart);
	    Node end=g->mapNode(posEnd);

	    jr.origin.lat=start.lat;
	    jr.origin.lon=start.lon;

	    jr.destination.lat=end.lat;
	    jr.destination.lon=end.lon;

	    return jr;
	  }

	  /*
	    * return node without cost
	    */      
	  BiList getNodeOnly(){
	    
	    BiList bl;
	    
	    BOOST_FOREACH(data d, data_before){
	      bl.PickUp.push_back(d.node);
	    }
	    
	    BOOST_FOREACH(data t, data_after){
	      bl.DropOff.push_back(t.node);
	    }
	    
	    return bl;
	  }
	  
	  /*
	    * return pickup node without cost
	    */  
	  NodeList getPickup(){
	    NodeList nl;
	    BOOST_FOREACH(data d, data_before){
	      nl.push_back(d.node);
	    }
	    return nl;
	  }
	  
	  NodeList getPickup(int limit){
	    NodeList nl;
	    BOOST_FOREACH(data d, data_before){
	      if(d.cost<=limit){
	      nl.push_back(d.node);
	      }
	    }
	    return nl;
	  }
	  
	   NodeList getPickup(int limit_a, int limit_b){
	    NodeList nl;
	    BOOST_FOREACH(data d, data_before){
	      if(d.cost<=limit_b && d.cost>limit_a){
	      nl.push_back(d.node);
	      }
	    }
	    return nl;
	  }
	  /*
	    * return dropoff node without cost
	    */  
	  NodeList getDropoff(){
	    NodeList nl;
	    BOOST_FOREACH(data d, data_after){
	      nl.push_back(d.node);
	    }
	    return nl;
	  }
	  
	  NodeList getDropoff(int limit){
	    NodeList nl;
	    BOOST_FOREACH(data d, data_after){
	      if(d.cost<=limit){
	      nl.push_back(d.node);
	      }
	    }
	    return nl;
	  }
	  
	  NodeList getDropoff(int limit_a, int limit_b){
	    NodeList nl;
	    BOOST_FOREACH(data d, data_after){
	      if(d.cost<=limit_b && d.cost>limit_a){
	      nl.push_back(d.node);
	      }
	    }
	    return nl;
	  }

	  /*
	    *computes isochrones to search for potential pickup/drop off location
	    */            
	  void findZ(const Transport::Graph * g, int limit_a, int limit_b){
	      
	    RLC::Graph *ag=new RLC::Graph( g,dfa_car );
	    my_isochrone m1=cool_iso(ag, posStart, limit_a);
	    
	    for(int i=0; i<g->num_vertices(); i++){
	      Node k=g->mapNode(i);
	      
	      if (m1.ns->bitset[i] && k.pick_up && i!=posStart){   
	      	data db;
	      	db.node=i;
	      	db.cost=m1.costs[i];
	      	data_before.push_back(db);
	      }
	    }

	    my_isochrone m2=cool_iso (ag, posEnd, limit_b);

	    for(int i=0; i<g->num_vertices(); i++){
	      Node k=g->mapNode(i);
	      
	      if (m2.ns->bitset[i] && k.pick_up && i!=posEnd){
	      	data da;
	      	da.node=i;
	      	da.cost=m2.costs[i];
	      	data_after.push_back(da);
	      }
	    }
		  
	  }
	  
	  /*
	    *computes isochrones to search for potential pickup
	    */ 
	  void findPickup(const Transport::Graph * g, int limit_a){
	    
	    dBefore.clear();
	        
	    RLC::Graph *ag=new RLC::Graph( g,dfa_car );
	    my_isochrone m1=cool_iso(ag, posStart, limit_a);
	    
	    for(int i=0; i<g->num_vertices(); i++){
	      Node k=g->mapNode(i);
	      
	      if (m1.ns->bitset[i] && k.pick_up){   
	      	data db;
	      	db.node=i;
	      	db.cost=m1.costs[i];
	      	dBefore.push_back(db);
	      }
	    }
	    
	  }

	  void choosePickup(int limit_a, int limit_b){

	    data_before.clear();
	   
	    BOOST_FOREACH(data d, dBefore){
	      if(d.cost<=limit_b && d.cost>=limit_a){
	      data_before.push_back(d);
	      }
	    }

	  }
	  
	  /*
	    *computes isochrones to search for potential dropoff
	    */ 
	  void findDropoff(const Transport::Graph * g, int limit_b){
	    
	    dAfter.clear();
	        
	    RLC::Graph *ag=new RLC::Graph( g,dfa_car );
	    my_isochrone m2=cool_iso (ag, posEnd, limit_b);

	    for(int i=0; i<g->num_vertices(); i++){
	      Node k=g->mapNode(i);
	      
	      if (m2.ns->bitset[i] && k.pick_up){
	      	data da;
	      	da.node=i;
	      	da.cost=m2.costs[i];
	      	dAfter.push_back(da);
	      }
	    }
	    
	  }

	  void chooseDropoff(int limit_a, int limit_b){

	    data_after.clear();
	   
	    BOOST_FOREACH(data d, dAfter){
	      if(d.cost<=limit_b && d.cost>=limit_a){
	      data_after.push_back(d);
	      }
	    }

	  }

	  /*
	    *computes A* to search for potential pick-up/ drop-off
	    */ 
	  void findRoad(const Transport::Graph * g){
	    
	    dAfter.clear();
	    dBefore.clear();

	    data_after.clear();
	    data_before.clear();
	        
	    RLC::Graph *ag=new RLC::Graph( g,dfa_car );

	      //RLC::Landmark *h=RLC::create_car_landmark( g, posEnd);

	      //typedef RLC::AspectTargetLandmark <RLC::DRegLC> Dij;

	    typedef RLC::AspectAstar <RLC::DRegLC> Dij;
	   
	      Dij::ParamType p1(
	          RLC::DRegLCParams( ag, 0, 1 ),
	          RLC::AspectAstarParams( posEnd )
	      );

	      Dij::ParamType p2(
	          RLC::DRegLCParams( ag, 0, 1 ),
	          RLC::AspectAstarParams( posStart )
	      );
	      
	      Dij dij1( p1 );

	      Dij dij2( p2 );

	      //int clock_start = 60*(60*9 + 5);

	      int clock_start = 0;
    

	      BOOST_FOREACH( int state, ag->dfa_start_states() ) {
	          dij1.add_source_node( RLC::Vertice(posStart, state), clock_start, 0 );
	      }

	      BOOST_FOREACH( int state, ag->dfa_start_states() ) {
	          dij2.add_source_node( RLC::Vertice(posEnd, state), clock_start, 0 );
	      }


	      while( !dij1.finished() ) {
	        RLC::Label lab = dij1.treat_next();
	        data db;
	        db.node=lab.node.first;
	        db.cost=lab.cost;

	        dBefore.push_back(db);

	        //cout << "visited " << lab.node.first <<endl;
	      }

	      while( !dij2.finished() ) {
	        RLC::Label lab = dij2.treat_next();
	        data da;
	        da.node=lab.node.first;
	        da.cost=lab.cost;
	        dAfter.push_back(da);
	        //cout << "visited " << lab.node.first <<endl;

	      }
	    
	    
	  }

	  /*
	    *computes A* to search for potential pick-up/ drop-off
	    */ 
	  void findRoadH(const Transport::Graph * g){
	    
	    data_after.clear();
	    data_before.clear();
	        
	    RLC::Graph *ag=new RLC::Graph( g,dfa_car );

	    

	      RLC::Landmark *h2=RLC::create_car_landmark( g, posEnd);

	      RLC::Landmark *h1=RLC::create_car_landmark( g, posStart);

	      typedef RLC::AspectTargetLandmark <RLC::DRegLC> Dij;

	    //typedef RLC::AspectAstar <RLC::DRegLC> Dij;

	   
	      Dij::ParamType p1(
	          RLC::DRegLCParams( ag, 0, 1 ),
	          RLC::AspectTargetLandmarkParams<>( posEnd, h2 )
	      );

	      Dij::ParamType p2(
	          RLC::DRegLCParams( ag, 0, 1 ),
	          RLC::AspectTargetLandmarkParams<>( posStart, h1 )
	      );
	      
	      Dij dij1( p1 );

	      Dij dij2( p2 );

	      //int clock_start = 60*(60*9 + 5);
	      int clock_start = 0;
    

	      BOOST_FOREACH( int state, ag->dfa_start_states() ) {
	          dij1.add_source_node( RLC::Vertice(posStart, state), clock_start, 0 );
	      }

	      BOOST_FOREACH( int state, ag->dfa_start_states() ) {
	          dij2.add_source_node( RLC::Vertice(posEnd, state), clock_start, 0 );
	      }


	      while( !dij1.finished() ) {
	        RLC::Label lab = dij1.treat_next();
	        data db;
	        db.node=lab.node.first;
	        db.cost=lab.cost;
	        data_before.push_back(db);

	        //cout << "visited " << lab.node.first <<endl;
	      }

	      while( !dij2.finished() ) {
	        RLC::Label lab = dij2.treat_next();
	        data da;
	        da.node=lab.node.first;
	        da.cost=lab.cost;
	        data_after.push_back(da);
	        //cout << "visited " << lab.node.first <<endl;

	      }
	    
	    
	  }

	  /*
	    * return the total traveling duration
	    * */
	  costVector ProcessTotalCost(pathList paths){
	    
	    costVector cv;
	    
	    BOOST_FOREACH(path p, paths){
	      data d1=getData(p.start, data_before), d2=getData(p.end, data_after);
	      totalCost tc;
	      tc.path_id=p.id;
	      tc.cost=d1.cost+p.cost+d2.cost;
	      cv.push_back(tc);
	    }
	    return cv;
	  }

	    /*
	  * return the total traveling duration with double cost
	  * */
	  costVector ProcessTotalCost_with_doubleCost(pathList paths){
	      
	    costVector cv;

	    BOOST_FOREACH(path p, paths){
	      data d1=getData(p.start, data_before), d2=getData(p.end, data_after);
	      totalCost tc;
	      tc.path_id=p.id;
	      tc.cost=2*d1.cost+p.cost+d2.cost;
	      cv.push_back(tc);
	    }
	    
	    return cv;
	  }
		
	  /*
	    * order favorites path according to the total traveling time
	    * */      
	  void getFavorites(pathList paths){
	    
	    costVector cv=ProcessTotalCost(paths);
	    costVector c=cv;
	    cv.sort(compare_cost);
	    int n=cv.size();
	    
	    BOOST_FOREACH(totalCost tc, cv){
	      pref p;
	      p.id=tc.path_id;
	      p.weight=n;
	      n--;
	      favorites.push_back(p);
	    }	
	  }

	   costVector ProcessTotalCostB(pathList paths){
	    
	    costVector cv;
	    
	    BOOST_FOREACH(path p, paths){
	      data d1=getData(p.start, data_before), d2=getData(p.end, data_after);
	      totalCost tc;
	      tc.path_id=p.id;
	      tc.cost=2*d1.cost+p.cost+d2.cost;
	      cv.push_back(tc);
	    }
	    return cv;
	  }

	  void getFavoritesB(pathList paths){
	    
	    costVector cv=ProcessTotalCostB(paths);
	    costVector c=cv;
	    cv.sort(compare_cost);
	    int n=cv.size();
	    
	    BOOST_FOREACH(totalCost tc, cv){
	      pref p;
	      p.id=tc.path_id;
	      p.weight=n;
	      n--;
	      favorites.push_back(p);
	    }	
	  }

	  /*
	    * order favorites path according to the total traveling time with doublecost function
	    * */      
	  void getFavorites_(pathList paths){
	    
	    costVector cv=ProcessTotalCost_with_doubleCost(paths);
	    cv.sort(compare_cost);
	    int n=cv.size()+10;
	    
	    BOOST_FOREACH(totalCost tc, cv){
	      //cout << tc.cost <<endl;
	      pref p;
	      p.id=tc.path_id;
	      p.weight=n;
	      n--;
	      favorites.push_back(p);
	    } 
	  }
	};
}


