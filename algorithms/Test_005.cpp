#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <time.h>
#include <iterator>     // ostream_operator
#include <boost/tokenizer.hpp>
#include <boost/concept_check.hpp>
#include "DataStructures/GraphFactory.h"
#include "MultipleParticipants/MPR_AspectPrivacy.h"
#include"RegLC/reglc_graph.h"
#include "utils/GeoTools.h"
#include "../../COLT99/lib/core/graph_wrapper.h"

#include <time.h>

using namespace Privacy;
using namespace std;
using namespace boost;

int proxima(int node,double d,const Transport::Graph * g ){
  
  int low=1;
  int high=g->num_vertices();	
  int j=(rand() % (high-low+1) + low);
  
  Node start=g->mapNode(node);
  
  for(int i=0; i<g->num_vertices(); i++){
    
    Node n=g->mapNode(i);
   
    
      if(proximity(n.lon,n.lat,start.lon,start.lat,d)){
      j=i;
      }
    
  }
  return j;}
  
int proximaR(int node,double d,const Transport::Graph * g ){
  
  int low=1;
  int high=g->num_vertices();	
  //int j=(rand() % (high-low+1) + low);
  int j=0;
  
  Node start=g->mapNode(node);
  
  for(int i=0; i<g->num_vertices(); i++){
    
    Node n=g->mapNode(i);
   
    
      if(proximityR(n.lon,n.lat,start.lon,start.lat,d)){
      j=i;
      }
    
  }
  return j;}
  
  
int proxima_tag(int node,double d,const Transport::Graph * g ){
  
  int low=1;
  int high=g->num_vertices();	
  int j=(rand() % (high-low+1) + low);
  
  Node start=g->mapNode(node);
  
  for(int i=0; i<g->num_vertices(); i++){
    
    Node n=g->mapNode(i);
   
    
      if(proximity(n.lon,n.lat,start.lon,start.lat,d) && n.pick_up){
      j=i;
      }
    
  }
  return j;}
  
int proxima_tagR(int node,double d,const Transport::Graph * g ){
  
  int low=1;
  int high=g->num_vertices();	
  //int j=(rand() % (high-low+1) + low);
  int j=0;
  
  Node start=g->mapNode(node);
  
  for(int i=0; i<g->num_vertices(); i++){
    
    Node n=g->mapNode(i);
   
    
      if(proximityR(n.lon,n.lat,start.lon,start.lat,d) && n.pick_up){
      j=i;
      }
    
  }
  cout<<"drop point "<<j<<endl;
  return j;}
  
int proxima2(float lon, float lat,double d,const Transport::Graph * g ){
  int j=0;
  for(int i=0; i<g->num_vertices(); i++){
    Node n=g->mapNode(i);
    if(proximity(n.lon,n.lat,lon,lat,d)){
      j=i;}}
  return j;}
void save_position(string fileName, Driver car, Pedestrian foot, const Transport::Graph * g ){
  
    ofstream fout(fileName);
    fout << "longitude" <<','<< "latitude"<<','<<"name"<<'\n';   
    
    Node dep_car=g->mapNode(car.posStart);
    Node dep_foot=g->mapNode(foot.posStart);
    
    Node arr_car=g->mapNode(car.posEnd);
    Node arr_foot=g->mapNode(foot.posEnd);
    
    fout <<  dep_car.lon <<','<< dep_car.lat <<','<<"Start car"<< '\n';	
    fout <<  dep_foot.lon <<','<< dep_foot.lat <<','<<"Start foot"<< '\n';
    
    fout << arr_car.lon <<','<< arr_car.lat <<','<<"End car"<< '\n';	
    fout <<  arr_foot.lon <<','<< arr_foot.lat <<','<<"End foot"<< '\n';
    
    fout.close();}
void save_position2(string fileName, Driver car, Pedestrian foot, Manager m, const Transport::Graph * g ){
  
    ofstream fout(fileName);
    fout << "longitude" <<','<< "latitude"<<','<<"name"<<'\n';   
    
    Node dep_car=g->mapNode(car.posStart);
    Node dep_foot=g->mapNode(foot.posStart);
    
    Node arr_car=g->mapNode(car.posEnd);
    Node arr_foot=g->mapNode(foot.posEnd);
    
    Node pick=g->mapNode(m.getThePath().start);
    Node drop=g->mapNode(m.getThePath().end);
    
    
    fout <<  dep_car.lon <<','<< dep_car.lat <<','<<"Start car"<< '\n';	
    fout <<  dep_foot.lon <<','<< dep_foot.lat <<','<<"Start foot"<< '\n';
    
    fout << arr_car.lon <<','<< arr_car.lat <<','<<"End car"<< '\n';	
    fout <<  arr_foot.lon <<','<< arr_foot.lat <<','<<"End foot"<< '\n';
    
    fout << pick.lon <<','<< pick.lat <<','<<"Pick Up"<< '\n';	
    fout << drop.lon <<','<< drop.lat <<','<<"Drop off"<< '\n';
    
    fout.close();}
void save_runtime(time_t isochrone_time, time_t psi_time, time_t matching_time, int scenario_id,Driver car, Pedestrian foot, Manager m){
    
    string filename="/home/matchi/Desktop/ST/results009/Runtime"+std::to_string(scenario_id) +".csv";
    ofstream fout(filename);
    fout << "isochrone" <<','<< "psi"<<','<<"matching"<<','<<"driver b_set"<<','<<"pedestrian b_set"<<','<<"driver e_set"<<','<<"pedestrian e_set"<<'\n';   
    fout <<  isochrone_time <<','<< psi_time <<','<<matching_time<<','<<car.data_before.size()<<',' <<foot.data_before.size()<<','<<car.data_after.size()<<','<<foot.data_after.size()<<'\n';	
    fout.close();
}

int random_pick(const Transport::Graph * g){
  int pick=0;
  int j=0;
  int low=1;
  int high=g->num_vertices();	
  
  
  while(1){
  
  //j=(rand() % (high-low+1) + low);
    j=0;
  Node n=g->mapNode(j);
  
  if (n.pick_up) break;
  }
  
  pick=j;
  
  cout<<"pick up: "<<pick<<endl;
  return pick;

  
  
}

int synchro(float lon, float lat,double d,const Transport::Graph * g ){
  
  int low=1;
  int high=g->num_vertices();	
  int j=(rand() % (high-low+1) + low);
  
  for(int i=0; i<g->num_vertices(); i++){
    Node n=g->mapNode(i);
      if(proximity(n.lon,n.lat,lon,lat,d) && n.pick_up){j=i;}
  }
  return j;} 
  
  
void csv_writer(string fileName, NodeList nl,const Transport::Graph * g) {
    ofstream fout(fileName);
    fout << "longitude" <<','<< "latitude"<<','<<"name"<<'\n';
    BOOST_FOREACH(int i, nl){
       Node p=g->mapNode(i);
       fout <<  p.lon <<','<< p.lat <<','<<"P"<< '\n';}
    fout.close();}
int main(int argc, char*argv[]){
  
    RLC::DFA dfa_car=RLC::car_dfa();
    RLC::DFA dfa_passenger=RLC::pt_foot_dfa();
    Transport::GraphFactory gf("/home/matchi/Desktop/ST/hercule.txt-dump", false);
    //gf.setAll2();
    const Transport::Graph * g = gf.get();
    
    int _pick,_drop;
    
    

    int x_foot, y_foot, x_car, y_car;
    int alpha, beta, delta=121; 
  
    time_t iso_t, psi_t, match_t;
    int compt=0;
    
	while (compt<101){
	  alpha=15, beta=15;
	  
	  cout<<"itération: "<<compt<<".................................................................."<<endl;
	 //_pick=random_pick(g);
	  _pick=compt;
	  //_pick=28;
	  _drop=proxima_tag(_pick, 5000,g);
	  
	  
	  x_car=proxima(_pick, 2000,g);
	  x_foot=proxima(_pick, 1000,g);
	  
	  y_car=proxima(_drop, 5000,g);
	  y_foot=proxima(_drop,2000,g);
	  
	  cout<<"x_car: "<<x_car<<" y_foot: "<<y_foot<<".................................................................."<<endl;
	  cout<<"y_car: "<<y_car<<" y_foot: "<<y_foot<<".................................................................."<<endl;

	  Privacy::Driver * d1=new Privacy::Driver(x_car, y_car);
	  Privacy::Pedestrian * p1=new Privacy::Pedestrian(x_foot, y_foot);
	  Privacy::Manager * m=new Privacy::Manager();
	  
	  BiList psi_result;
	do{
	
	START_TICKING;
	p1->findZ(g,alpha,beta);
	d1->findZ(g,alpha,beta);
	STOP_TICKING;
	
	iso_t=RUNTIME;

	START_TICKING;
	psi_result=m->PSI(*d1,*p1);
	m->findA(g, psi_result);
	STOP_TICKING;
	
	psi_t=RUNTIME;
	
	cout<<alpha<<endl;
	alpha=alpha+5;
	beta=beta+5;
	if(alpha>delta){
	  cout<<"pas de solution"<<endl;
	  break;}}
	while(m->shared_path.size()<=0);
      
	if(m->shared_path.size()>0){
	
        
        START_TICKING;
	d1->getFavorites(m->shared_path);
	p1->getFavorites(m->shared_path);
	m->match(*d1,*p1);
	STOP_TICKING;
	
	match_t=RUNTIME;
	
	save_runtime(iso_t,psi_t,match_t,compt,*d1, *p1,*m);
	save_position2("/home/matchi/Desktop/ST/results009/Scenario"+std::to_string(compt) +".csv", *d1, *p1,*m,g); 
	}
	 compt++; 
	}
	
	
	
	}
	
