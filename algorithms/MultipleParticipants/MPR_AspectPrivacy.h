/** Copyright : Ulrich Matchi AÏVODJI (2014)  umaivodj@laas.fr

This software is a computer program whose purpose is to [describe
functionalities and technical features of your software].

This software is governed by the CeCILL-B license under French law and
abiding by the rules of distribution of free software.  You can  use, 
modify and/ or redistribute the software under the terms of the CeCILL-B
license as circulated by CEA, CNRS and INRIA at the following URL
"http://www.cecill.info". 

As a counterpart to the access to the source code and  rights to copy,
modify and redistribute granted by the license, users are provided only
with a limited warranty  and the software's author,  the holder of the
economic rights,  and the successive licensors  have only  limited
liability. 

In this respect, the user's attention is drawn to the risks associated
with loading,  using,  modifying and/or developing or reproducing the
software by the user in light of its specific status of free software,
that may mean  that it is complicated to manipulate,  and  that  also
therefore means  that it is reserved for developers  and  experienced
professionals having in-depth computer knowledge. Users are therefore
encouraged to load and test the software's suitability as regards their
requirements in conditions enabling the security of their systems and/or 
data to be ensured and,  more generally, to use and operate it in the 
same conditions as regards security. 

The fact that you are presently reading this means that you have had
knowledge of the CeCILL-B license and that you accept its terms. 
*/

#ifndef MPR_ASPECT_PRIVACY_H
#define MPR_ASPECT_PRIVACY_H

#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <time.h>
#include <iterator>     // ostream_operator
#include <boost/tokenizer.hpp>
#include <boost/concept_check.hpp>
#include "../utils/GeoTools.h"
#include <random>
#include <algorithm>

#include "../DataStructures/GraphFactory.h" 
#include"../RegLC/reglc_graph.h"
#include "../RegLC/AlgoTypedefs.h"
#include "run_configurations.h"
#include <vector>
#include <climits>
#include <boost/lexical_cast.hpp>
#include <string>


using namespace MuPaRo;
 
namespace Privacy {

RLC::DFA dfa_car=RLC::car_dfa();
RLC::DFA dfa_passenger=RLC::pt_foot_dfa();
//RLC::DFA dfa_passenger=RLC::foot_dfa(); 

struct data{
  int node, cost;
};
    
struct totalCost{
  int path_id, cost;
};
    
typedef std::list<totalCost>costVector;

typedef std::list<data> dataList;
      
struct pref{
  int id, weight;
  bool operator==(const pref a)
    {
	return id==a.id;
    }
};
  
struct path {
  int id, start, end, cost;
  bool operator==(const path a)
    {
	return id==a.id;
    }	  
};


/*
 * structure for test configuration
 */

struct Test_config{
  
  int synchro2_a; //_a reprensent upper limit
  int synchro2_b; //_b represent lower limit
  
  int car1_a;
  int car1_b;
  int foot1_a;
  int foot1_b;
  
  
  int car2_a;
  int car2_b;
  int foot2_a;
  int foot2_b;
  
  int time_car;
  int time_foot;
  
  int limit_car;
  int limit_foot;
  
  int number_of_scenarios;
 
};


/*
 * structure for five cost
 */

struct Five_cost{
  
  int len_car_pickup;
  int len_foot_pickup;
  int len_shared_path;
  int len_car_dropoff;
  int len_foot_dropoff;
  
  int wait_time;
  
  int total_cost;
 
};

/*
 * structure for five cost
 */

struct Positions{
  
  float car_start_long;
  float car_start_lat;
  
  float car_end_long;
  float car_end_lat;
  
  float foot_start_long;
  float foot_start_lat;
  
  float foot_end_long;
  float foot_end_lat;
  
  float pickup_long;
  float pickup_lat;
  
  
  float dropoff_long;
  float dropoff_lat;
  
 
};


/*
 * structure for test output
 */

struct Test_output{
  
  int scenario_id;
  
  int car_start;
  int car_end;
  int foot_start;
  int foot_end;
  
  int cd_pickup;
  int cd_dropoff;
  
  
  int cc_pickup;
  int cc_dropoff;
  
  int ccl_pickup;
  int ccl_dropoff;
  
  int pickup_foot_size;
  int pickup_car_size;
  
  int dropoff_foot_size;
  int dropoff_car_size;
  
  int psi_pickup_size;
  int psi_dropoff_size;
  
  int shared_path_size;
  
  string iso_foot_pickup_time;
  string iso_foot_dropoff_time;
  string iso_car_pickup_time;
  string iso_car_dropoff_time;
  
  string psi_pickup_time;
  string psi_dropoff_time;
  
  string path_computing_time;
  
  string path_ordering_foot_time;
  string path_ordering_car_time;
  
  string path_election_time;
  
  string cd_time;
  string cc_time;
  string ccl_time;
  
  Five_cost cd_costs;
  Five_cost cc_costs;
  Five_cost ccl_costs;
  
  Positions cd_positions;
  Positions cc_positions;
  Positions ccl_positions;
  
  Test_config input;
  
};

/*
 * structure for cc_test output
 */

struct cc_output{
  
 
  
  int cc_pickup;
  int cc_dropoff;
  
  string cc_time;
  

  Five_cost cc_costs;
  
  Positions cc_positions;
  
};

      
typedef std::vector<path> pathList;

typedef std::vector<pref> prefs;
  
struct BiList{
  NodeList PickUp, DropOff;
};




data getData(int id, dataList myData){
	
  data dt;
  
  BOOST_FOREACH(data d, myData){
    if(d.node==id)
      dt=d;
  }
  return dt;
}

pref getPref(int id, prefs myPrefs){
	
  pref pr;
  
  BOOST_FOREACH(pref p, myPrefs){
    if(p.id==id)
      pr=p;
  }
  return pr;
}
  
bool compare_cost(totalCost t1, totalCost t2){
  return t1.cost<t2.cost;
}
  
bool compare_pref(pref p1, pref p2){
  return p1.weight<p2.weight;
}
  
    
/*
 * Model for Driver
 */    
class Driver{
  

    
  public:
    
  int posStart, posEnd;
  dataList data_before, data_after;
  prefs favorites;

  Driver(){
    
  }

  Driver(int start, int end){
    
    posStart=start;
    posEnd=end; 
  }

  ~Driver(){}

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
    
    data_before.clear();
        
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
    
  }
  
  /*
    *computes isochrones to search for potential dropoff
    */ 
  void findDropoff(const Transport::Graph * g, int limit_b){
    
    data_after.clear();
        
    RLC::Graph *ag=new RLC::Graph( g,dfa_car );
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

};
    
/*
 * Model for Pedestrian
 */
class Pedestrian{
  
    
  
  public:
  
 
  int posStart, posEnd;
  dataList data_before, data_after;
  prefs favorites;
  
  Pedestrian(){
    
  }
  Pedestrian(int start, int end){
    
    posStart=start;
    posEnd=end; 
  }    
  ~Pedestrian(){}

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
	  
    RLC::Graph *ag=new RLC::Graph( g,dfa_passenger );
    my_isochrone m1=cool_iso (ag, posStart, limit_a);

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
    
    data_before.clear();
     
    RLC::Graph *ag=new RLC::Graph( g,dfa_passenger );
    my_isochrone m1=cool_iso (ag, posStart, limit_a);

    for(int i=0; i<g->num_vertices(); i++){
      Node k=g->mapNode(i);
      
      if (m1.ns->bitset[i] && k.pick_up && i!=posStart){
	data db;
	db.node=i;
	db.cost=m1.costs[i];
	data_before.push_back(db);
      }
    }
  }
  
  void findDropoff(const Transport::Graph * g, int limit_b){
    
    data_after.clear();
    
    RLC::Graph *ag=new RLC::Graph( g,dfa_passenger );
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
  * order favorites path according to the total traveling time
  * */
  void getFavorites(pathList paths){
	  
    costVector cv=ProcessTotalCost(paths);
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
         
};
  

  
/*
 *In charge of carpooling negociation  
 */
class Manager {
      
  public:
    pathList shared_path;
    prefs matching;
    Manager(){}
    ~Manager(){}
    
    bool checker(BiList result, unsigned int limit_a,  unsigned int limit_b){
      return ((result.PickUp.size()>=limit_a)&&(result.DropOff.size()>=limit_b));
    }
       
    /*
     *compute the intersection of pickup and dropoff set of the driver and the pedestrian
     */
    BiList PSI(Driver driver, Pedestrian pedestrian ){
      
      BiList psi, dri, ped;
      
      ped=pedestrian.getNodeOnly();
      ped.PickUp.sort();
      ped.DropOff.sort();
      
      dri=driver.getNodeOnly();
      dri.PickUp.sort();
      dri.DropOff.sort();
      
      //pickup intersection
      std::set_intersection(ped.PickUp.begin(), ped.PickUp.end(), dri.PickUp.begin(), dri.PickUp.end(), back_inserter(psi.PickUp));
      
      //drop off intersection
      std::set_intersection(ped.DropOff.begin(), ped.DropOff.end(), dri.DropOff.begin(), dri.DropOff.end(), back_inserter(psi.DropOff));
      
      return psi;
    }

          
    /*
     *compute the intersection of pickup set of the driver and the pedestrian
     */
    NodeList PSI_Pickup(Driver driver, Pedestrian pedestrian ){
      
      NodeList psi, ped, dri;
      
      ped=pedestrian.getPickup();
      ped.sort();
      
      dri=driver.getPickup();
      dri.sort();
      
      //pickup intersection
      std::set_intersection(ped.begin(), ped.end(), dri.begin(), dri.end(), back_inserter(psi));
        
      return psi;
    }




    /*
     *compute the intersection of dropoff set of the driver and the pedestrian
     */
    NodeList PSI_Dropoff(Driver driver, Pedestrian pedestrian ){
      
      NodeList psi, ped, dri;
      
      ped=pedestrian.getDropoff();
      ped.sort();
      
      dri=driver.getDropoff();
      dri.sort();
      
      //dropoff intersection
      std::set_intersection(ped.begin(), ped.end(), dri.begin(), dri.end(), back_inserter(psi));
        
      return psi;
    }

    BiList getPSI(NodeList pickup, NodeList dropoff){
        
      BiList bl;
      
      BOOST_FOREACH(int  p, pickup){
        bl.PickUp.push_back(p);
      }
      
      BOOST_FOREACH(int d, dropoff){
        bl.DropOff.push_back(d);
      }
      
      return bl;
    }

    bool not_empty(NodeList nl){
      return !nl.empty();
    }

        
    /* 
    //depricated path finder  
    void findA(const Transport::Graph * g, BiList psi){
      
    	int i=0;
    	
    	    BOOST_FOREACH(int o, psi.PickUp) {
    	      BOOST_FOREACH(int d, psi.DropOff) {
    	      
    		  AlgoMPR::PtToPt * mup =  point_to_point( g, o, d, dfa_car);
    		  mup->run();
    			if (mup->get_cost(0, d)>0){
    			  
    			  path  pt;
    			  pt.id=i;
    			  pt.start=o;
    			  pt.end=d;
    			  pt.cost=mup->get_cost(0,d);
    			  
    			  i++;
    			  
    			  shared_path.push_back(pt);
    			  
    			  delete mup;
    			 
    			 
    			
    			}
    			else{
    			  
    			  delete mup;
    		
    			}   
    	    }
    	  }
    	
    }*/
          


    /*
     * Compute all possible paths between shared pickup set and shared dropoff set using min(pickup_size, dropoff size) dijskstra
     * */
    void GetAllPath(const Transport::Graph * g, BiList psi){ 
          
      RLC::Graph *fg=new RLC::Graph( g,dfa_car );
      RLC::BackwardGraph *bg=new RLC::BackwardGraph(fg);
      bool direction=(psi.PickUp.size()<=psi.DropOff.size());
      //bool direction=true;

      int i=0;

      if (direction){   
        
        BOOST_FOREACH(int center, psi.PickUp){ 
          uint compt=0;
          typedef RLC::DRegLC Dij;    
          Dij::ParamType p(RLC::DRegLCParams( fg, 0, 1 ));
          Dij dij( p );
          
          BOOST_FOREACH( int state, fg->dfa_start_states() ){
    	dij.add_source_node( RLC::Vertice(center, state), 0, 0 );
          }
          
          //while( (!dij.finished()) && (compt!=psi.DropOff.size()) ){
          while(!dij.finished()){

    	RLC::Label lab = dij.treat_next();
    	int n=lab.node.first;
    	bool ck=std::find(psi.DropOff.begin(), psi.DropOff.end(), n) != psi.DropOff.end();
    	
    	//avoid pickup=dropoff scenarios
    	if (ck && lab.cost>0){ 
    	  path  pt;
    	  pt.id=i;
    	  pt.start=center;
    	  pt.end=n;
    	  pt.cost=lab.cost;
    	  shared_path.push_back(pt);
    	  compt++;
    	  i++;
    	}
    	
    	if(compt==psi.DropOff.size()){
    	  break;
    	}
    	
          }
        }
      }
      else{
            
        BOOST_FOREACH(int center, psi.DropOff){ 
          uint compt=0;
          typedef RLC::DRegLC Dij;
          Dij::ParamType p(RLC::DRegLCParams( bg, 0, 1 ));
          Dij dij( p );
          
          BOOST_FOREACH( int state, bg->dfa_start_states() ){
    	dij.add_source_node( RLC::Vertice(center, state), 0, 0 );
          }
          
          while(!dij.finished()){
    		
    	RLC::Label lab = dij.treat_next();
    	int n=lab.node.first;
    	bool ck=std::find(psi.PickUp.begin(), psi.PickUp.end(), n) != psi.PickUp.end();
    	
    	//avoid pickup=dropoff scenarios
    	if (ck && lab.cost>0){
    	  path  pt;
    	  pt.id=i;
    	  pt.start=n;
    	  pt.end=center;
    	  pt.cost=lab.cost;
    	  shared_path.push_back(pt);
    	  compt++;
    	  i++;
    	}
    	
    	if(compt==psi.PickUp.size()){
    	  break;
    	}
          } 
        }
      }
           
    }
        
    /*
     *odrer paths such that the best path if the one who minimize the totat travel time for both driver and pedestrian
     */    
    void match(Driver d, Pedestrian p){
      
      prefs p_car=d.favorites, p_foot=p.favorites;
      int n=p_car.size();

      for(int i=0; i<n; ++i){
        for(int j=0; j<n; ++j){
          
          if(p_car[i].id==p_foot[j].id){
    	pref p;
    	p.id=p_car[i].id;
    	p.weight=p_car[i].weight+p_foot[j].weight;
    	matching.push_back(p);
          }
        
        }
      }
      std::sort(matching.begin(), matching.end(), compare_pref);
    }

    /*
     *odrer paths such that the best path if the one who minimize the totat travel time for both driver and pedestrian
     */    
    void match_2(Driver d, Pedestrian p){
      
      prefs p_car=d.favorites, p_foot=p.favorites;
      
      BOOST_FOREACH(pref pc, p_car){
        
        auto ck=std::find(p_foot.begin(), p_foot.end(), pc);
        
        if (ck != p_foot.end())
        {
          
          int index = std::distance(p_foot.begin(), ck);
          pref p;
          p.id=pc.id;
          p.weight=pc.weight+ p_foot[index].weight;
          matching.push_back(p);
        }
        
        
        
      }


      std::sort(matching.begin(), matching.end(), compare_pref);
    }

    /*
     * return the size of the shared path
     */
    int shared_path_len(){
      return shared_path.size();
    }

    /*
     * return a path given its id
     */
    path getPathById(int id, pathList paths) const{
    	path myPath;
    	BOOST_FOREACH(path p, paths){
    	  if(p.id==id){
    	    myPath=p;
    	    break;
    	  }
    	}
    	return myPath;
    }

    /*
     * return the best path
     */
    path getThePath() const{
      return getPathById(matching[matching.size()-1].id, shared_path);
    }
    
};

class Toolbox{
  public:
   
    Toolbox(){}
    ~Toolbox(){}
    
    /*
   * random values generator
   */
  int rgenerator(int min,int max){
    
    std::random_device rd;

    std::mt19937 eng(rd());
    
    std::uniform_int_distribution<int> dist(min,max);
    return dist(eng);
  }

  /*
   * returns a randomly chosen pick up location among all the predefined pick up locations
   */
  int synchro(const Transport::Graph * g) {
    int min=0;
    int max=g->PickUpZone().size()-1;
    int j=rgenerator(min,max);
    NodeList::iterator it=std::next(g->PickUpZone().begin(), j);
    int k=*it;
    return k;
  }

  /*
   * returns a randomly chosen pick up location among all the predefined pick up locations in a given Area
   */
  int synchro_area(Area * area) {
    int min=0;
    int max=area->size()-1;
    int j=rgenerator(min,max);
    
    int k=area->get(j);
    return k;
  }

  /*
   * returns a vector of pick up nodes located between min_d and max_d from node
   */
  std::vector<int> satellites(int node,  double max_d,  double min_d, const Transport::Graph * g){
    std::vector<int> v;
    Node start=g->mapNode(node);
    for(int i=0; i<g->num_vertices(); i++){
      Node n=g->mapNode(i);
      if(proximity(n.lon,n.lat,start.lon,start.lat,max_d) && !proximity(n.lon,n.lat,start.lon,start.lat,min_d) && n.pick_up){
        v.push_back(i);
      }
    }
    cout << "taille de v: " << v.size()<<endl;
    return v;
  }

  /*
   * returns a vector of nodes located between min_d and max_d from node
   */
  std::vector<int> satellites_free(int node,  double max_d,  double min_d, const Transport::Graph * g){
    std::vector<int> v;
    Node start=g->mapNode(node);
    for(int i=0; i<g->num_vertices(); i++){
      Node n=g->mapNode(i);
      if(proximity(n.lon,n.lat,start.lon,start.lat,max_d) && !proximity(n.lon,n.lat,start.lon,start.lat,min_d)){
        v.push_back(i);
      }
    }
    return v;
  }

  /*
   * return a random locations among satellite locations
   */
  int get_satellite(std::vector<int> satellites){
    int r;
    int min=0;
    int max=satellites.size()-1;
    
    if (max!=0) {
      int j=rgenerator(min,max);
      r = satellites[j];
    }
    else{
      r = 0;
      cout << "aucun noeuds trouvé" << endl;
    }
    return r;
  }



  /*
   *return the carpooling solution: origins and destinations for both driver and pedestrian and pickup-dropoff 
   */
  Positions cd_carpooling_positions(Driver car, Pedestrian foot, Manager m, const Transport::Graph * g ){
    
      Positions pos;
      
      Node dep_car=g->mapNode(car.posStart);
      Node dep_foot=g->mapNode(foot.posStart);
      
      Node arr_car=g->mapNode(car.posEnd);
      Node arr_foot=g->mapNode(foot.posEnd);
      
      Node pick=g->mapNode(m.getThePath().start);
      Node drop=g->mapNode(m.getThePath().end);
      
      pos.car_start_long=dep_car.lon;
      pos.car_start_lat=dep_car.lat;
      
      pos.car_end_long=arr_car.lon;
      pos.car_end_lat=arr_car.lat;
      
      
      pos.foot_start_long=dep_foot.lon;
      pos.foot_start_lat=dep_foot.lat;
      
      pos.foot_end_long=arr_foot.lon;
      pos.foot_end_lat=arr_foot.lat;
      
      pos.pickup_long=pick.lon;
      pos.pickup_lat=pick.lat;
      
      pos.dropoff_long=drop.lon;
      pos.dropoff_lat=drop.lat;
        
      return pos;
  }

  /*
   * save size of pickup dropoff shared_pickup shared_dropoff shared_path set and the cost of the 5 paths coresponding to the carpooling solution
   */
  Five_cost cd_costs(Driver car, Pedestrian foot, Manager m, const Transport::Graph * g ){
        
      RLC::DFA dfa_car=RLC::car_dfa();
      RLC::DFA dfa_passenger=RLC::pt_foot_dfa();
      
      int len1,len2,len3,len4,len5;
      Five_cost costs;
      
      AlgoMPR::PtToPt * c1 =  point_to_point( g,foot.posStart , m.getThePath().start, dfa_passenger);
      AlgoMPR::PtToPt * c2 =  point_to_point( g,car.posStart , m.getThePath().start, dfa_car);
      
      AlgoMPR::PtToPt * c3 =  point_to_point( g, m.getThePath().start, m.getThePath().end, dfa_car);
      
      AlgoMPR::PtToPt * c4 =  point_to_point( g, m.getThePath().end, foot.posEnd, dfa_passenger);
      AlgoMPR::PtToPt * c5 =  point_to_point( g,m.getThePath().end, car.posEnd, dfa_car);
      
      c1->run();
      len1=c1->get_cost(0, m.getThePath().start);
      delete c1;
      
      c2->run();
      len2=c2->get_cost(0, m.getThePath().start);
      delete c2;
      
      c3->run();
      len3=c3->get_cost(0, m.getThePath().end);
      delete c3;
      
      c4->run();
      len4=c4->get_cost(0, foot.posEnd);
      delete c4;
      
      c5->run();
      len5=c5->get_cost(0, car.posEnd);
      delete c5;
      
      int waiting=abs(len1-len2);
      
      
      costs.len_foot_pickup=len1;
      costs.len_car_pickup=len2;
      costs.len_shared_path=len3;
      costs.len_foot_dropoff=len4;
      costs.len_car_dropoff=len5;
      
      costs.wait_time=waiting;
      
      costs.total_cost=len1 + len2 + 2*(len3) + len4 + len5 +waiting;
      
      return costs;
  } 

  /*
   * return centralised carpooling output
   */
  cc_output cc_carpooling_test(Driver car, Pedestrian foot,const Transport::Graph * g){
    
    cc_output output;
    Positions pos;
    Five_cost costs;
    
    typedef AlgoMPR::CarSharingTest CurrAlgo;
    
    START_TICKING;
    CurrAlgo::ParamType p(
              MuparoParams( g, 5 ),
              AspectTargetParams( 4, foot.posEnd ),
              AspectPropagationRuleParams( SumPlusWaitCost, MaxArrival, 2, 0, 1),
              AspectPropagationRuleParams( SumCost, FirstLayerArrival, 4, 2, 3)
          );
          
    CurrAlgo cs( p );
  	
    init_car_sharing<CurrAlgo>( &cs, g, foot.posStart, car.posStart, foot.posEnd, car.posEnd, dfa_passenger, dfa_car);
    cs.run();
    STOP_TICKING;
    output.cc_time=boost::lexical_cast<std::string>(RUNTIME);
    
    int time=0;
    
    Node dep_car=g->mapNode(car.posStart);
    Node dep_foot=g->mapNode(foot.posStart);
    
    Node arr_car=g->mapNode(car.posEnd);
    Node arr_foot=g->mapNode(foot.posEnd);
    
    int drop_off = cs.get_source(4, foot.posEnd);
    int pick_up = cs.get_source(2, drop_off);
    
    output.cc_pickup=pick_up;
    output.cc_dropoff=drop_off;
    
    Node pick=g->mapNode(pick_up);
    Node drop=g->mapNode(drop_off);
    
    int len1=cs.arrival(0, pick_up)-time;
    int len2=cs.arrival(1, pick_up)-time;
    int len3=cs.arrival(2, drop_off) - cs.arrival(2, pick_up);
    
    int len4=cs.get_cost(3, drop_off);
    int len5=cs.arrival(4, foot.posEnd) - cs.arrival(4, drop_off);
    
    //int wait_time=abs(cs.arrival(0, pick_up) - cs.arrival(1, pick_up)) ;
    
    pos.car_start_long=dep_car.lon;
    pos.car_start_lat=dep_car.lat;
    
    pos.car_end_long=arr_car.lon;
    pos.car_end_lat=arr_car.lat;
    
    
    pos.foot_start_long=dep_foot.lon;
    pos.foot_start_lat=dep_foot.lat;
    
    pos.foot_end_long=arr_foot.lon;
    pos.foot_end_lat=arr_foot.lat;
    
    pos.pickup_long=pick.lon;
    pos.pickup_lat=pick.lat;
    
    pos.dropoff_long=drop.lon;
    pos.dropoff_lat=drop.lat;
    
    output.cc_positions=pos;
    
    costs.len_foot_pickup=len1;
    costs.len_car_pickup=len2;
    costs.len_shared_path=len3;
    costs.len_foot_dropoff=len5;
    costs.len_car_dropoff=len4;
    
    costs.wait_time=abs(cs.arrival(0, pick_up) - cs.arrival(1, pick_up)) ;
    
    costs.total_cost=len1 + len2 + 2*len3 + len4 + len5 + costs.wait_time;
    
    output.cc_costs=costs;
    
    
    
    return output;
    
  }

  /*
   * return with limit centralised carpooling output 
   */
  cc_output cc_carpooling_test_with_limit(Driver car, Pedestrian foot,const Transport::Graph * g, int limit_a, int limit_b){
    
    cc_output output;
    Positions pos;
    Five_cost costs;
    
    typedef AlgoMPR::CarSharingTest CurrAlgo;
    
    START_TICKING;
    CurrAlgo::ParamType p(
              MuparoParams( g, 5 ),
              AspectTargetParams( 4, foot.posEnd ),
              AspectPropagationRuleParams( SumPlusWaitCost, MaxArrival, 2, 0, 1),
              AspectPropagationRuleParams( SumCost, FirstLayerArrival, 4, 2, 3)
          );
          
    CurrAlgo cs( p );
  	
    init_car_sharing_with_limit<CurrAlgo>( &cs, g, foot.posStart, car.posStart, foot.posEnd, car.posEnd, dfa_passenger, dfa_car, limit_a, limit_b);
    cs.run();
    STOP_TICKING;
    output.cc_time=boost::lexical_cast<std::string>(RUNTIME);
    
    
    int time=0;
    
    Node dep_car=g->mapNode(car.posStart);
    Node dep_foot=g->mapNode(foot.posStart);
    
    Node arr_car=g->mapNode(car.posEnd);
    Node arr_foot=g->mapNode(foot.posEnd);
    
    int drop_off = cs.get_source(4, foot.posEnd);
    int pick_up = cs.get_source(2, drop_off);
    
    output.cc_pickup=pick_up;
    output.cc_dropoff=drop_off;
    
    Node pick=g->mapNode(pick_up);
    Node drop=g->mapNode(drop_off);
    
    int len1=cs.arrival(0, pick_up)-time;
    int len2=cs.arrival(1, pick_up)-time;
    int len3=cs.arrival(2, drop_off) - cs.arrival(2, pick_up);
    
    int len4=cs.get_cost(3, drop_off);
    int len5=cs.arrival(4, foot.posEnd) - cs.arrival(4, drop_off);
    
    //int wait_time=abs(cs.arrival(0, pick_up) - cs.arrival(1, pick_up)) ;
    
    pos.car_start_long=dep_car.lon;
    pos.car_start_lat=dep_car.lat;
    
    pos.car_end_long=arr_car.lon;
    pos.car_end_lat=arr_car.lat;
    
    
    pos.foot_start_long=dep_foot.lon;
    pos.foot_start_lat=dep_foot.lat;
    
    pos.foot_end_long=arr_foot.lon;
    pos.foot_end_lat=arr_foot.lat;
    
    pos.pickup_long=pick.lon;
    pos.pickup_lat=pick.lat;
    
    pos.dropoff_long=drop.lon;
    pos.dropoff_lat=drop.lat;
    
    output.cc_positions=pos;
    
    costs.len_foot_pickup=len1;
    costs.len_car_pickup=len2;
    costs.len_shared_path=len3;
    costs.len_foot_dropoff=len5;
    costs.len_car_dropoff=len4;
    
    costs.wait_time=abs(cs.arrival(0, pick_up) - cs.arrival(1, pick_up)) ;
    
    costs.total_cost=len1 + len2 + 2*len3 + len4 + len5 + costs.wait_time;
    
    output.cc_costs=costs;
    
    
    
    return output;
    
  }


  /*
   * read test configuration file
   */
  Test_config test_loader(string filename){  
    
    Test_config tcf;
    
    typedef boost::tokenizer< boost::char_separator<char> > Tokenizer;
    boost::char_separator<char> sep(";");
    
    ifstream in(filename.c_str());

    if (!in.is_open()) exit(0);

    vector< string > vec;
    string line;
    int i=0;

    while (getline(in,line)){
      
      Tokenizer tok(line,sep);
      vec.assign(tok.begin(),tok.end());

      if(i==1){
      tcf.synchro2_a=std::atoi(vec.at(0).c_str());
      tcf.synchro2_b=std::atoi(vec.at(1).c_str());
      
      tcf.car1_a=std::atoi(vec.at(2).c_str());
      tcf.car1_b=std::atoi(vec.at(3).c_str());
      tcf.foot1_a=std::atoi(vec.at(4).c_str());
      tcf.foot1_b=std::atoi(vec.at(5).c_str());
      
      tcf.car2_a=std::atoi(vec.at(6).c_str());
      tcf.car2_b=std::atoi(vec.at(7).c_str());
      tcf.foot2_a=std::atoi(vec.at(8).c_str());
      tcf.foot2_b=std::atoi(vec.at(9).c_str());
      
      tcf.time_car=std::atoi(vec.at(10).c_str());
      tcf.time_foot=std::atoi(vec.at(11).c_str());
      
      tcf.limit_car=std::atoi(vec.at(12).c_str());
      tcf.limit_foot=std::atoi(vec.at(13).c_str());
      
      tcf.number_of_scenarios=std::atoi(vec.at(14).c_str());
      
      break;
      }
      i++;
    }
    
    cout<<"============================================Test configuration===================================="<<endl;
    cout<<"synchro2 sup: "<<tcf.synchro2_a<<" synchro2 inf: "<<tcf.synchro2_b<<endl;
    
    cout<<"car1 sup: "<<tcf.car1_a<<" car1 inf: "<<tcf.car1_b<<endl;
    cout<<"foot1 sup: "<<tcf.foot1_a<<" foot1 inf: "<<tcf.foot1_b<<endl;
    
    cout<<"car2 sup: "<<tcf.car2_a<<" car2 inf: "<<tcf.car2_b<<endl;
    cout<<"foot2 sup: "<<tcf.foot2_a<<" foot2 inf: "<<tcf.foot2_b<<endl;
    
    cout<<"time car: "<<tcf.time_car<<" time foot: "<<tcf.time_foot<<endl;
    cout<<"limit car: "<<tcf.limit_car<<" limit foot: "<<tcf.limit_foot<<endl;
    
    cout<<"number of scenarios: "<<tcf.number_of_scenarios<<endl; 
    cout<<"===================================================================================================="<<endl;
    
    return tcf;
  }

  /*
   * save all outputs
   */

  void create_result_file(string fileName){
    
    ofstream fout;
   
    fout.open (fileName, ofstream::out | ofstream::app);
    
    fout << "scenario_id" <<';';
    
    fout<< "iso_foot_pickup_time"<<';'<<"pickup_foot_size"<<';'<<"iso_car_pickup_time"<<';'<<"pickup_car_size"<<';';
    
    fout << "iso_foot_dropoff_time" <<';'<< "dropoff_foot_size"<<';'<<"iso_car_dropoff_time"<<';'<<"dropoff_car_size"<<';';
    
    fout << "psi_pickup_time" <<';'<< "psi_pickup_size"<<';'<<"psi_dropoff_time"<<';'<<"psi_dropoff_size"<<';';
    
    fout << "path_computing_time" <<';'<< "shared_path_size"<<';'<<"path_ordering_foot_time"<<';'<<"path_ordering_car_time"<<';';
    
    fout << "cd_total_cost"<<';'<< "cc_total_cost"<<';'<<"ecart_cost"<<';'<<"diff_cost"<<';';
    
    fout<<"ecart_distance_pickup (meters)"<<';'<<"ecart_distance_dropoff (meters)"<<';';
    
    fout<< "cd_len1"<<';'<<"cc_len1"<<';'<<"cd_len2"<<';'<<"cc_len2"<<';'<< "cd_len3"<<';'<<"cc_len3"<<';';
    
    fout<< "cd_len4"<<';'<<"cc_len4"<<';'<<"cd_len5"<<';'<<"cc_len5"<<';';
    
    fout<< "foot1"<<';'<<"car1"<<';'<<"foot2"<<';'<<"car2"<<';';
    
    fout<< "cd_waiting_time"<<';'<<"cd_waiting_time"<<';'<<"diff_waiting_time"<<';';
    
    fout << "cd_total_cost"<<';'<< "ccl_total_cost"<<';'<<"ecart_cost_2"<<';'<<"diff_cost_2"<<';';

    fout << "cc_runtime"<<';'<<"ccl_runtime"<<'\n';
    
    fout.close();
    
  }

  void save_all(string fileName,Test_output output, const Transport::Graph * g ){
    
    Node pick=g->mapNode(output.cd_pickup);
    Node drop=g->mapNode(output.cd_dropoff);
    
    Node pick2=g->mapNode(output.cc_pickup);
    Node drop2=g->mapNode(output.cc_dropoff);
    
    double d_pickup=haversine_dist(pick.lon,pick.lat, pick2.lon,pick2.lat);
    double d_dropoff=haversine_dist(drop.lon,drop.lat, drop2.lon,drop2.lat);
    
    ofstream fout;
   
    fout.open (fileName, ofstream::out | ofstream::app);
    
    fout <<output.scenario_id <<';';
    
    fout<< output.iso_foot_pickup_time <<';'<<output.pickup_foot_size<<';'<<output.iso_car_pickup_time<<';' <<output.pickup_car_size<<';';
    
    fout<< output.iso_foot_dropoff_time <<';'<<output.dropoff_foot_size<<';'<<output.iso_car_dropoff_time<<';' <<output.dropoff_car_size<<';';
    
    fout<< output.psi_pickup_time <<';'<<output.psi_pickup_size<<';'<<output.psi_dropoff_time<<';' <<output.psi_dropoff_size<<';';
    
    fout<< output.path_computing_time <<';'<<output.shared_path_size<<';'<<output.path_ordering_foot_time<<';' <<output.path_ordering_car_time<<';';
    
    fout<< output.cd_costs.total_cost<<';'<< output.cc_costs.total_cost <<';'<< abs(output.cd_costs.total_cost-output.cc_costs.total_cost) <<';'<<(output.cd_costs.total_cost-output.cc_costs.total_cost)<<';';
    
    fout<<d_pickup<<';'<<d_dropoff<<';';
    
    fout<< output.cd_costs.len_foot_pickup<<';'<<output.cc_costs.len_foot_pickup<<';'<<output.cd_costs.len_car_pickup<<';'<<output.cc_costs.len_car_pickup<<';'<< output.cd_costs.len_shared_path<<';'<<output.cc_costs.len_shared_path<<';';
    
    fout<< output.cd_costs.len_foot_dropoff<<';'<<output.cc_costs.len_foot_dropoff<<';'<<output.cd_costs.len_car_dropoff<<';'<<output.cc_costs.len_car_dropoff<<';';
    
    fout<< output.foot_start<<';'<<output.car_start<<';'<<output.foot_end<<';'<<output.car_end<<';';
    
    fout<< output.cd_costs.wait_time<<';'<<output.cc_costs.wait_time<<';'<<output.cd_costs.wait_time - output.cc_costs.wait_time<<';';
    
    fout<< output.cd_costs.total_cost<<';'<< output.ccl_costs.total_cost <<';'<< abs(output.cd_costs.total_cost-output.ccl_costs.total_cost) <<';'<<(output.cd_costs.total_cost-output.ccl_costs.total_cost)<<';';

    
    fout << output.cc_time<<';'<<output.ccl_time<<'\n';

    fout.close();
    
  }

  
  /*
   * test centralized carpooling
   */
  void test_cc(){
    cout<<endl;
    cout<<"========================Begin centralized carpooling test====================================="<<endl;
    
    Transport::GraphFactory gf("/home/matchi/Desktop/ST/hercule.txt-dump", false);
    gf.setAll2();
    const Transport::Graph * g = gf.get();
    
    int car1=13089;
    int foot1=13384;
    int car2=38925;
    int foot2=71781;

    
    int time=0;
      
      
    typedef AlgoMPR::CarSharingTest CurrAlgo;

    CurrAlgo::ParamType p(
              MuparoParams( g, 5 ),
              AspectTargetParams( 4, foot2 ),
              AspectPropagationRuleParams( SumPlusWaitCost, MaxArrival, 2, 0, 1),
              AspectPropagationRuleParams( SumCost, FirstLayerArrival, 4, 2, 3)
          );
          
    CurrAlgo cs( p );
  	
    init_car_sharing<CurrAlgo>( &cs, g, foot1, car1, foot2, car2, dfa_passenger, dfa_car);
    cs.run();
    
    int drop_off = cs.get_source(4, foot2);
    int pick_up = cs.get_source(2, drop_off);
    
    int len1=cs.arrival(0, pick_up)-time;
    int len2=cs.arrival(1, pick_up)-time;
    int len3=cs.arrival(2, drop_off) - cs.arrival(2, pick_up);
    int len4=cs.get_cost(3, drop_off);
    int len5=cs.arrival(4, foot2) - cs.arrival(4, drop_off);
    int wait_time=abs(len1-len2);
    
    
    int total_cost=len1+len2+ 2*len3 +len4+len5+ wait_time;
    
    cout << "len foot1: "<<len1<<endl;
    cout << "len car1: "<<len2<<endl;
    cout << "len shared: "<<len3<<endl;
    cout << "len foot2: "<<len5<<endl;
    cout << "len car2: "<<len4<<endl;
    cout << "total cost: "<<total_cost<<endl;
    
    cout<<"_____________postions__________"<<endl;
    
    cout << "foot1: "<<foot1<<endl;
    cout << "car1: "<<car1<<endl;
    
    cout << "pickup: "<<pick_up<<endl;
    cout << "dropoff: "<<drop_off<<endl;
    
    cout << "foot2: "<<foot2<<endl;
    cout << "car2: "<<car2<<endl;
    /*
    cout<<"===============================================Checking========================================================"<<endl;
    
    int len11,len21,len31,len41,len51;
    
    AlgoMPR::PtToPt * c1 =  point_to_point( g,foot1 , pick_up, dfa_passenger);
    AlgoMPR::PtToPt * c2 =  point_to_point( g,car1 , pick_up, dfa_car);
    
    AlgoMPR::PtToPt * c3 =  point_to_point( g, pick_up, drop_off, dfa_car);
    
    AlgoMPR::PtToPt * c4 =  point_to_point( g, drop_off, foot2, dfa_passenger);
    AlgoMPR::PtToPt * c5 =  point_to_point( g,drop_off, car2, dfa_car);
    
    c1->run();
    len11=c1->get_cost(0, pick_up);
    delete c1;
    
    c2->run();
    len21=c2->get_cost(0, pick_up);
    delete c2;
    
    c3->run();
    len31=c3->get_cost(0, drop_off);
    delete c3;
    
    c4->run();
    len41=c4->get_cost(0, foot2);
    delete c4;
    
    c5->run();
    len51=c5->get_cost(0, car2);
    delete c5;
    
     
    int wait_time2=abs(len11-len21);
    int total_cost1=len11+len21+ 2*len31+len41+len51+ wait_time2;
    
    cout << "len foot1: "<<len11<<endl;
    cout << "len car1: "<<len21<<endl;
    cout << "len shared: "<<len31<<endl;
    cout << "len foot2: "<<len41<<endl;
    cout << "len car2: "<<len51<<endl;
    cout << "total cost1: "<<total_cost1<<endl;*/
    
    cout<<"=============================End centralized carpooling test======================================"<<endl;
    cout<<endl;
  }

  /*
   * test distributed carpooling
   */
  void test_cd(){
    
    cout<<endl;
    cout<<"==========================Begin distributed carpooling test==========================================="<<endl;

    
    Transport::GraphFactory gf("/home/matchi/Desktop/ST/hercule.txt-dump", false);
    gf.setAll2();
    const Transport::Graph * g = gf.get();
    
    int car1=13089;
    int foot1=13384;
    int car2=38925;
    int foot2=71781;

    
    Privacy::Driver * d1=new Privacy::Driver(car1, car2);
    Privacy::Pedestrian * p1=new Privacy::Pedestrian(foot1, foot2);
    Privacy::Manager * m=new Privacy::Manager();
    
    int limit_car=600 +1, limit_foot=300 +1;
    
    int time_foot=60, time_car=60;
      
    int time_foot_pickup=time_foot, time_foot_dropoff=time_foot, time_car_pickup=time_car, time_car_dropoff=time_car;
    
    
    
    Test_output output;
      
      
      BiList psi_result;
      NodeList psi_pickup,psi_dropoff;
      
      bool ck_pickup_foot,ck_pickup_car, ck_dropoff_foot,ck_dropoff_car;
      
      do{	
  	  
        	    //------------------for pedestrian-----------
        
        
  	    //isochrone for pedesdrian to get potentiel pickup
  	    if(time_foot_pickup<=limit_foot){
  	      
  	      p1->findPickup(g,time_foot_pickup);
  	      time_foot_pickup+=time_foot;
  	      
  	    }
  	    
  	    //isochrone for pedesdrian to get potentiel dropoff
  	    if(time_foot_dropoff<=limit_foot){
  	      
  	      p1->findDropoff(g,time_foot_dropoff);
  	      time_foot_dropoff+=time_foot;
  	      //cout<<"yoooooo dropoff foot"<<endl;
  	      
  	    }
  	    
  	    //------------------for driver-----------
  	    
  	    //isochrone for driver to get potentiel pickup
  	    if(time_car_pickup<=limit_car){
  	      
  	      d1->findPickup(g,time_car_pickup);     
  	      time_car_pickup+=time_car;
  	      
  	    }
  	    
  	    //isochrone for driver to get potentiel dropoff
  	    if(time_car_dropoff<=limit_car){
  	      
  	      d1->findDropoff(g,time_car_dropoff);      
  	      time_car_dropoff+=time_car;
  	      
  	    }
  	    
  	    
  	    //checking if limits are reached
  	    
  	    ck_pickup_foot=(time_foot_pickup>limit_foot);
  	    ck_dropoff_foot=(time_foot_dropoff>limit_foot);
  	    
  	    ck_pickup_car=(time_car_pickup>limit_car);
  	    ck_dropoff_car=(time_car_dropoff>limit_car);
  	    
  	    psi_pickup=m->PSI_Pickup(*d1, *p1);
  	        
  	    psi_dropoff=m->PSI_Dropoff(*d1, *p1);	    
  	    
  	    psi_result=m->getPSI(psi_pickup, psi_dropoff);
  	    
  	    m->GetAllPath(g, psi_result);
  	    
  	  
  	 
        //stop when the max travel time is reached
        if(ck_pickup_foot && ck_dropoff_foot && ck_pickup_car && ck_dropoff_car){
  	break;
        }
        
      } 
      while(m->shared_path.size()<=0);
      
      if(m->shared_path.size()>0){
      
        
        d1->getFavorites(m->shared_path);
        
        
        p1->getFavorites(m->shared_path);
        
        output.path_ordering_foot_time=RUNTIME;
        
        m->match(*d1,*p1);
        
        
        output.cd_pickup=m->getThePath().start;
        output.cd_dropoff=m->getThePath().end;
        
             
        //costs of the distributed solution 
        output.cd_costs=cd_costs(*d1,*p1,*m,g);
        
       
        int len1=output.cd_costs.len_foot_pickup;
        int len2=output.cd_costs.len_car_pickup;
        int len3=output.cd_costs.len_shared_path;
        int len4=output.cd_costs.len_foot_dropoff;
        int len5=output.cd_costs.len_car_dropoff;
        int wait_time=abs(len1-len2);
        int total_cost=len1+len2+2*len3+len4+len5+ wait_time;
        
        cout << "len foot1: "<<len1<<endl;
        cout << "len car1: "<<len2<<endl;
        cout << "len shared: "<<len3<<endl;
        cout << "len foot2: "<<len4<<endl;
        cout << "len car2: "<<len5<<endl;
        cout << "total cost: "<<total_cost<<endl;
        
        cout<<"_____________postions__________"<<endl;
    
        cout << "foot1: "<<foot1<<endl;
        cout << "car1: "<<car1<<endl;
        
        cout << "pickup: "<<output.cd_pickup<<endl;
        cout << "dropoff: "<<output.cd_dropoff<<endl;
        
        cout << "foot2: "<<foot2<<endl;
        cout << "car2: "<<car2<<endl;
  	
    /*    
    cout<<"=============================================Checking=========================================================="<<endl;
    
    int len11,len21,len31,len41,len51;
    
    AlgoMPR::PtToPt * c1 =  point_to_point( g,foot1 , output.cd_pickup, dfa_passenger);
    AlgoMPR::PtToPt * c2 =  point_to_point( g,car1 , output.cd_pickup, dfa_car);
    
    AlgoMPR::PtToPt * c3 =  point_to_point( g, output.cd_pickup, output.cd_dropoff, dfa_car);
    
    AlgoMPR::PtToPt * c4 =  point_to_point( g, output.cd_dropoff, foot2, dfa_passenger);
    AlgoMPR::PtToPt * c5 =  point_to_point( g,output.cd_dropoff, car2, dfa_car);
    
    c1->run();
    len11=c1->get_cost(0, output.cd_pickup);
    delete c1;
    
    c2->run();
    len21=c2->get_cost(0, output.cd_pickup);
    delete c2;
    
    c3->run();
    len31=c3->get_cost(0, output.cd_dropoff);
    delete c3;
    
    c4->run();
    len41=c4->get_cost(0, foot2);
    delete c4;
    
    c5->run();
    len51=c5->get_cost(0, car2);
    delete c5;
    
    
    
    int wait_time2=abs(len11-len21);
    int total_cost1=len11+len21+2*len31+len41+len51+ wait_time2;
    
    cout << "len foot1: "<<len11<<endl;
    cout << "len car1: "<<len21<<endl;
    cout << "len shared: "<<len31<<endl;
    cout << "len foot2: "<<len41<<endl;
    cout << "len car2: "<<len51<<endl;
    cout << "total cost1: "<<total_cost1<<endl;*/
        
    cout<<"===========================End distributed carpooling test======================================="<<endl;
    cout<<endl;
  	
        
      
    }
    
    else{
      
      cout<<"========================================>Pas de solution"<<endl;
      
    }
        
  }
  /*
   * test with limit centralized carpooling
   */
  void test_cc_with_limit(){
    
    cout<<endl;
    cout<<"===========================Begin centralized carpooling test with 5 min stop condition============="<<endl;
    
    Transport::GraphFactory gf("/home/matchi/Desktop/ST/hercule.txt-dump", false);
    gf.setAll2();
    const Transport::Graph * g = gf.get();
    
    
    int time=0;
    
    
    int car1=13089;
    int foot1=13384;
    int car2=38925;
    int foot2=71781;

    
    int limit_a=300, limit_b=348;  
      

   
    AlgoMPR::CarSharingTest::ParamType p(
        MuparoParams( g, 5 ),
        AspectTargetParams( 4, foot2 ),
        AspectPropagationRuleParams( SumPlusWaitCost, MaxArrival, 2, 0, 1),
        AspectPropagationRuleParams( SumCost, FirstLayerArrival, 4, 2, 3)
    );
    
    

    AlgoMPR::CarSharingTest cs( p );
    
    init_car_sharing_with_limit<AlgoMPR::CarSharingTest>( &cs, g, foot1, car1, foot2, car2, dfa_passenger, dfa_car, limit_a, limit_b );

    
    cs.run();

    
    int drop_off = cs.get_source(4, foot2);
    int pick_up = cs.get_source(2, drop_off);
    
    int len1=cs.arrival(0, pick_up)-time;
    int len2=cs.arrival(1, pick_up)-time;
    int len3=cs.arrival(2, drop_off) - cs.arrival(2, pick_up);
    int len4=cs.get_cost(3, drop_off);
    int len5=cs.arrival(4, foot2) - cs.arrival(4, drop_off);
    
    int wait_time=abs(len1-len2);
    int total_cost=len1+len2+2*len3+len4+len5+ wait_time;
        
    
    cout << "len foot1: "<<len1<<endl;
    cout << "len car1: "<<len2<<endl;
    cout << "len shared: "<<len3<<endl;
    cout << "len foot2: "<<len5<<endl;
    cout << "len car2: "<<len4<<endl;
    cout << "total cost: "<<total_cost<<endl;
    
    cout<<"_____________postions__________"<<endl;
    
    cout << "foot1: "<<foot1<<endl;
    cout << "car1: "<<car1<<endl;
    
    cout << "pickup: "<<pick_up<<endl;
    cout << "dropoff: "<<drop_off<<endl;
    
    cout << "foot2: "<<foot2<<endl;
    cout << "car2: "<<car2<<endl;
    
    cout<<"==================End centralized carpooling test with 5 min stop condition================================"<<endl;
    cout<<endl;
  }


  /*
   * run the test
   */
  void run_test(string filename){
    
    
    
    Transport::GraphFactory gf("graph.txt-dump", false);
    gf.setAll2();
    
    cout<<"==============================================Graph configuration====================================="<<endl;
    const Transport::Graph * g = gf.get();
    
    cout<<"whatever edges: "<<g->listEdges(WhateverEdge).size()<<endl;
    cout<<"bus edges: "<<g->listEdges(BusEdge).size()<<endl;
    cout<<"bike edges: "<<g->listEdges(BikeEdge).size()<<endl;
    cout<<"foot edges: "<<g->listEdges(FootEdge).size()<<endl;
    cout<<"car edges:"<<g->listEdges(CarEdge).size()<<endl;
    cout<<"subway edges:"<<g->listEdges(SubwayEdge).size()<<endl;
    cout<<"tram edges: "<<g->listEdges(TramEdge).size()<<endl;
    cout<<"transfer edges: "<<g->listEdges(TransferEdge).size()<<endl;
    cout<<"unknown edges: "<<g->listEdges(UnknownEdgeType).size()<<endl;
    
    cout<<"number of pick up points: "<<g->PickUpZone().size()<<endl;
     
    
    Test_config tcf=test_loader(filename);
    create_result_file("output.csv");
    
    int k=0;
   
    while (k<tcf.number_of_scenarios){
      
      cout <<"################################################ Scenario "<<k<<" #####################################################"<<endl;
      
      //generate position    
      int synchro_1=synchro(g);
      int synchro_2=get_satellite(satellites(synchro_1,tcf.synchro2_a,tcf.synchro2_b,g));
    
      int car_1=get_satellite(satellites_free(synchro_1, tcf.car1_a,tcf.car1_b,g));
      int foot_1=get_satellite(satellites_free(synchro_1, tcf.foot1_a,tcf.foot1_b,g));
     
      int car_2=get_satellite(satellites_free(synchro_2, tcf.car2_a,tcf.car2_b,g));
      int foot_2=get_satellite(satellites_free(synchro_2, tcf.foot2_a,tcf.foot2_b,g));
      
      
      Privacy::Driver * d1=new Privacy::Driver(car_1, car_2);
      Privacy::Pedestrian * p1=new Privacy::Pedestrian(foot_1, foot_2);
      Privacy::Manager * m=new Privacy::Manager();
      
      cout<<"synchro 1: "<<synchro_1<<endl;
      cout<<"synchro 2: "<<synchro_2<<endl;
    
      cout<<"car 1: "<<car_1<<endl;
      cout<<"foot 1: "<<foot_1<<endl;

      cout<<"car 2: "<<car_2<<endl;
      cout<<"foot 2: "<<foot_2<<endl;
    
    
      int limit_car=tcf.limit_car +1, limit_foot=tcf.limit_foot +1;
      
      int time_foot_pickup=tcf.time_foot, time_foot_dropoff=tcf.time_foot, time_car_pickup=tcf.time_car, time_car_dropoff=tcf.time_car;
      
      Test_output output;
      
      output.input=tcf;
      
      BiList psi_result;
      NodeList psi_pickup,psi_dropoff;
      
      bool ck_pickup_foot,ck_pickup_car, ck_dropoff_foot,ck_dropoff_car;
      
      do{	
  	  
        	    //------------------for pedestrian-----------
        
        
  	    //isochrone for pedesdrian to get potentiel pickup
  	    if(time_foot_pickup<=limit_foot){
  	      
  	      START_TICKING;
  	      p1->findPickup(g,time_foot_pickup);
  	      STOP_TICKING;
  	      
  	      output.iso_foot_pickup_time=RUNTIME;
  	      
  	      time_foot_pickup+=tcf.time_foot;
  	      //cout<<"yoooooo pickup foot"<<endl;
  	      
  	    }
  	    
  	    //isochrone for pedesdrian to get potentiel dropoff
  	    if(time_foot_dropoff<=limit_foot){
  	      
  	      START_TICKING;
  	      p1->findDropoff(g,time_foot_dropoff);
  	      STOP_TICKING;
  	      
  	      output.iso_foot_dropoff_time=RUNTIME;
  	      
  	      time_foot_dropoff+=tcf.time_foot;
  	      //cout<<"yoooooo dropoff foot"<<endl;
  	      
  	    }
  	    
  	    //------------------for driver-----------
  	    
  	    //isochrone for driver to get potentiel pickup
  	    if(time_car_pickup<=limit_car){
  	      
  	      START_TICKING;
  	      d1->findPickup(g,time_car_pickup);
  	      STOP_TICKING;
  	      
  	      output.iso_car_pickup_time=RUNTIME;
  	      
  	      time_car_pickup+=tcf.time_car;
  	      //cout<<"yoooooo pickup car"<<endl;
  	      
  	    }
  	    
  	    //isochrone for driver to get potentiel dropoff
  	    if(time_car_dropoff<=limit_car){
  	      
  	      START_TICKING;
  	      d1->findDropoff(g,time_car_dropoff);
  	      STOP_TICKING;
  	      
  	      output.iso_car_dropoff_time=RUNTIME;
  	      
  	      time_car_dropoff+=tcf.time_car;
  	      //cout<<"yoooooo dropoff car"<<endl;
  	      
  	    }
  	    
  	    
  	    //checking if limits are reached
  	    
  	    ck_pickup_foot=(time_foot_pickup>limit_foot);
  	    ck_dropoff_foot=(time_foot_dropoff>limit_foot);
  	    
  	    ck_pickup_car=(time_car_pickup>limit_car);
  	    ck_dropoff_car=(time_car_dropoff>limit_car);
  	    
  	    START_TICKING;
  	    psi_pickup=m->PSI_Pickup(*d1, *p1);
  	    STOP_TICKING;
  	    
  	    output.psi_pickup_time=RUNTIME;
  	    output.psi_pickup_size=psi_pickup.size();
  	    
  	    START_TICKING;
  	    psi_dropoff=m->PSI_Dropoff(*d1, *p1);
  	    STOP_TICKING;
  	    
  	    output.psi_dropoff_time=RUNTIME;
  	    output.psi_dropoff_size=psi_dropoff.size();
  	    
  	    
  	    
  	    psi_result=m->getPSI(psi_pickup, psi_dropoff);
  	    
  	    START_TICKING;
  	    m->GetAllPath(g, psi_result);
  	    STOP_TICKING;
  	    
  	    output.path_computing_time=RUNTIME;
  	  
  	 
        //stop when the max travel time is reached
        if(ck_pickup_foot && ck_dropoff_foot && ck_pickup_car && ck_dropoff_car){
  	break;
        }
        
      } 
      while(m->shared_path.size()<=0);
      
      if(m->shared_path.size()>0){
      
        START_TICKING;
        d1->getFavorites(m->shared_path);
        STOP_TICKING;
        
        output.path_ordering_car_time=RUNTIME;
        
        START_TICKING;
        p1->getFavorites(m->shared_path);
        STOP_TICKING;
        
        output.path_ordering_foot_time=RUNTIME;
        
        START_TICKING;
        m->match(*d1,*p1);
        STOP_TICKING;
        
        output.path_election_time=RUNTIME;
        
        //id of the scenario
        output.scenario_id=k;
        
        //id of positions
        output.car_start=d1->posStart;
        output.car_end=d1->posEnd;
        output.foot_start=p1->posStart;
        output.foot_end=p1->posEnd;
        
        //id of pickup/dropoff
        output.cd_pickup=m->getThePath().start;
        output.cd_dropoff=m->getThePath().end;
        
        //postions of the distributed solution
        output.cd_positions=cd_carpooling_positions(*d1,*p1,*m,g);
        
        //costs of the distributed solution 
        output.cd_costs=cd_costs(*d1,*p1,*m,g);
        
        //size of potential pickup for pedesdrian
        output.pickup_foot_size=p1->data_before.size();
        //size of potential pickup for driver
        output.pickup_car_size=d1->data_before.size();
       //size of potential dropoff for pedesdrian
        output.dropoff_foot_size=p1->data_after.size();
        //size of potential dropoff for driver
        output.dropoff_car_size=d1->data_after.size();
        
        cc_output cc=cc_carpooling_test(*d1,*p1,g);
        cc_output ccl=cc_carpooling_test_with_limit(*d1,*p1,g,limit_foot+1,limit_car+1);
        
        //costs in centralized
        output.cc_costs=cc.cc_costs;
        //pickup in centralized
        output.cc_pickup=cc.cc_pickup;
        //dropoff in centralized
        output.cc_dropoff=cc.cc_dropoff;
        //position in centralized
        output.cc_positions=cc.cc_positions;
        //runtime in centralized
        output.cc_time=cc.cc_time;
        
        //costs in centralized
        output.ccl_costs=ccl.cc_costs;
        //pickup in centralized
        output.ccl_pickup=ccl.cc_pickup;
        //dropoff in centralized
        output.ccl_dropoff=ccl.cc_dropoff;
        //position in centralized
        output.ccl_positions=ccl.cc_positions;
        //runtime in centralized
        output.ccl_time=ccl.cc_time;
        
        output.shared_path_size=m->shared_path.size();
        
        
        save_all("output.csv", output, g);
        
        cout<<"========================================>Solution"<<endl;
      
    }
    
    else{
      
      cout<<"========================================>Pas de solution"<<endl;
      
    }
        
     
      
      k=k+1;

    }
    
    
  }

  int dlen(dataList d){
      return d.size();
  }
  
};
    
   
}
#endif