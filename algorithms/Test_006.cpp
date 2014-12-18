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


#include "run_configurations.h"


using namespace Privacy;
using namespace std;
using namespace boost;



int proxima(int node,double d,const Transport::Graph * g ){
  
  int j=0;
  Node start=g->mapNode(node);
  
  for(int i=0; i<g->num_vertices(); i++){
    
    Node n=g->mapNode(i);
   
    
      if(proximity(n.lon,n.lat,start.lon,start.lat,d)){
      j=i;
      }
    
  }
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
    
void save_parameter(string fileName, Driver car, Pedestrian foot, Manager m, BiList psi_result, const Transport::Graph * g ){
  
RLC::DFA dfa_car=RLC::car_dfa();
RLC::DFA dfa_passenger=RLC::pt_foot_dfa();
  
    ofstream fout(fileName);
    fout << "pickup set" <<','<< "dropoff set"<<','<< "shared paths"<<','<<"len1"<<','<<"len2"<<','<<"len3"<<','<<"len4"<<','<<"len5"<<'\n';   
    
    Node dep_car=g->mapNode(car.posStart);
    Node dep_foot=g->mapNode(foot.posStart);
    
    Node arr_car=g->mapNode(car.posEnd);
    Node arr_foot=g->mapNode(foot.posEnd);
    
    Node pick=g->mapNode(m.getThePath().start);
    Node drop=g->mapNode(m.getThePath().end);
    int len1,len2,len3,len4,len5;
    
    AlgoMPR::PtToPt * c1 =  point_to_point( g,foot.posStart , m.getThePath().start, dfa_passenger);
    AlgoMPR::PtToPt * c2 =  point_to_point( g,car.posStart , m.getThePath().start, dfa_car);
    AlgoMPR::PtToPt * c3 =  point_to_point( g, m.getThePath().start, m.getThePath().end, dfa_car);
    AlgoMPR::PtToPt * c4 =  point_to_point( g,foot.posEnd , m.getThePath().end, dfa_passenger);
    AlgoMPR::PtToPt * c5 =  point_to_point( g,car.posEnd , m.getThePath().end, dfa_car);
    
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
    len4=c4->get_cost(0, m.getThePath().end);
    delete c4;
    
    c5->run();
    len5=c5->get_cost(0, m.getThePath().end);
    delete c5;
    
    fout << psi_result.PickUp.size() <<','<< psi_result.DropOff.size()<<','<< m.shared_path_len()<<','<<len1<<','<<len2<<','<<len3<<','<<len4<<','<<len5<<'\n';  
    
    fout.close();}
void save_runtime(time_t isochrone_time, time_t psi_time, time_t matching_time, int scenario_id,Driver car, Pedestrian foot, Manager m){
    
    string filename="/home/matchi/Desktop/ST/results004/Runtime"+std::to_string(scenario_id) +".csv";
    ofstream fout(filename);
    fout << "isochrone" <<','<< "psi"<<','<<"matching"<<','<<"driver b_set"<<','<<"pedestrian b_set"<<','<<"driver e_set"<<','<<"pedestrian e_set"<<'\n';   
    fout <<  isochrone_time <<','<< psi_time <<','<<matching_time<<','<<car.data_before.size()<<',' <<foot.data_before.size()<<','<<car.data_after.size()<<','<<foot.data_after.size()<<'\n';	
    fout.close();
}

int synchro(float lon, float lat,double d,const Transport::Graph * g ){
  int j=0;
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
  
    
    Transport::GraphFactory gf("/home/matchi/Desktop/ST/hercule.txt-dump", false);
    //gf.setAll2();
    const Transport::Graph * g = gf.get();

    int x_foot=0, y_foot=0, x_car=0, y_car=0;
    int alpha, beta, delta=121; 
  
    time_t iso_t, psi_t, match_t;
  
    
	
	int low=1;
	int high=g->num_vertices();
	
	for(int i=0; i<463; i++){
	  alpha=15;
	  beta=15;
	  
	  cout<<"itération: "<<i<<".................................................................."<<endl;
	  
	  x_car=i;
	  //x_foot=(rand() % (high-low+1) + low);
	  x_foot=proxima(x_car, 500,g);
	  
	  y_car=i+4;
	  //y_foot=(rand() % (high-low+1) + low);
	  y_foot=proxima(y_car,500,g);
	  cout<<"x_car: "<<x_car<<" x_foot: "<<x_foot<<".................................................................."<<endl;
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
	
	save_runtime(iso_t,psi_t,match_t,i,*d1, *p1,*m);
	save_position2("/home/matchi/Desktop/ST/results004/Scenario"+std::to_string(i) +".csv", *d1, *p1,*m,g);
	save_parameter("/home/matchi/Desktop/ST/results004/Parameter"+std::to_string(i) +".csv", *d1, *p1, *m, psi_result, g );
	
	  
	}
	  
	}
	
	
	
	}
	
