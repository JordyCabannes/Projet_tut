#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <time.h>
#include <boost/concept_check.hpp>
#include "MultipleParticipants/MPR_AspectPrivacy.h"
#include <iterator>     // ostream_operator
#include <boost/tokenizer.hpp>
#include "DataStructures/GraphFactory.h"
#include"RegLC/reglc_graph.h"
#include "utils/GeoTools.h"

using namespace Privacy;
using namespace std;
using namespace boost;

struct Config{
  int foot, car;
};



int proxima(int node,double d,const Transport::Graph * g ){
  
  int j=0;
  Node start=g->mapNode(node);
  
  for(int i=0; i<g->num_vertices(); i++){
    
    Node n=g->mapNode(i);
   
    if((n.lat!=start.lat) && (n.lon!=start.lon)) {
      if(proximity(n.lon,n.lat,start.lon,start.lat,d)/* &&  !proximity(n.lon,n.lat,start.lon,start.lat,d+1000)*/){
      j=i;
      }
    }
  }
  return j;}
  
  
int proxima2(float lon, float lat,double d,const Transport::Graph * g ){
  int j=0;
  
  
  for(int i=0; i<g->num_vertices(); i++){
    
    Node n=g->mapNode(i);
   
    
      if(proximity(n.lon,n.lat,lon,lat,d)){
      j=i;
      }
    
  }
  return j;}
  
  
int synchro(float lon, float lat,double d,const Transport::Graph * g ){
  int j=0;
  for(int i=0; i<g->num_vertices(); i++){
    Node n=g->mapNode(i);
      if(proximity(n.lon,n.lat,lon,lat,d) && n.pick_up){j=i;}
  }
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
void csv_writer(string fileName, NodeList nl,const Transport::Graph * g) {
    ofstream fout(fileName);
    fout << "longitude" <<','<< "latitude"<<','<<"name"<<'\n';
    BOOST_FOREACH(int i, nl){
       
        Node p=g->mapNode(i);
        fout <<  p.lon <<','<< p.lat <<','<<"P"<< '\n';	
    }
    fout.close();}
int num_line(string filename){
  int k=0;
  string data=fileName;
  string line;
  ifstream in(data.c_str());
  if (!in.is_open()) return 1;
  for (int i = 0; getline(f, line); ++i){
    k++;
  }
  return k-1;
}

Config Yuki(string fileName,const Transport::Graph * g){
  
    Config cg;
    string data=fileName;
    ifstream in(data.c_str());
    if (!in.is_open()) return 1;
    vector< string > vec;
    string line;
    int k=0;
    int low=1;
    int high=num_line(fileName);
    int random=(rand() % (high-low+1) + low);
    while (getline(in,line)){
	
        Tokenizer tok(line);
        vec.assign(tok.begin(),tok.end());
	
	if(k==random){
	  string lo=vec.at(0);
	  string la=vec.at(1);
	  float lon=std::stof(lo);
	  float lat=std::stof(la);
	  
	  int a=proxima2(lon,lat, 100, g);
	  
	  
	  
	  
	
	
      
	
	k=k+1;
      
    }
  
  
  return cg;
}
    
    
int main(int argc, char*argv[]){
  
   RLC::DFA dfa_car=RLC::car_dfa();
   RLC::DFA dfa_passenger=RLC::pt_foot_dfa();
  
   Transport::GraphFactory gf("/home/matchi/Desktop/ST/hercule.txt-dump", false);
   gf.setAll2();
   const Transport::Graph * g = gf.get();
   typedef tokenizer< escaped_list_separator<char> > Tokenizer;
   
   int alpha=60, beta=60; 
  
  //configuration
  if (argc != 2){
   cout << "Bad Input" << endl;
   exit(0);}
   
   else{
    
    string data=argv[1];
    cout<<data<<endl;
    ifstream in(data.c_str());
    if (!in.is_open()) return 1;
    vector< string > vec;
    string line;
    int k=0;
    while (getline(in,line)){
	
	cout<<"hotspot num: "<<k<<endl;
        Tokenizer tok(line);
        vec.assign(tok.begin(),tok.end());
	
	string lo=vec.at(0);
	string la=vec.at(1);
	
        float lon=std::stof(lo);
	float lat=std::stof(la);
	int start_car2=proxima2(lon,lat, 1000, g);
	//cout <<"hotspot:  "<<start_car2<<endl;
	
	Privacy::Driver * d1=new Privacy::Driver(start_car2,proxima(start_car2,5300,g));
	
	Privacy::Pedestrian * p1=new Privacy::Pedestrian(proxima(start_car2,500,g), proxima(start_car2,6200,g));
	
	save_position("/home/matchi/Desktop/ST/results/pp/Scenario"+std::to_string(k)+".csv", *d1, *p1,g);
	Privacy::Manager *m=new Privacy::Manager();
	BiList psi_result;
	
	do{
	p1->findZ(g,alpha,beta);
	d1->findZ(g,alpha,beta);
	psi_result=m->PSI(*d1,*p1);
	m->findA(g, psi_result);
	//cout<<alpha<<endl;
	alpha=alpha+30;
	beta=beta+30;
	if(alpha>121) {
	  cout<<"pas de solution"<<endl;
	  break;}}
	  
	while(m->shared_path.size()<=0);
      
	if(m->shared_path.size()>0){
	  d1->getFavorites(m->shared_path);
	  p1->getFavorites(m->shared_path);
	  m->match(*d1,*p1);
	  save_position2("/home/matchi/Desktop/ST/Tests/results/pp/Scenario"+std::to_string(k)+".csv", *d1, *p1,*m,g);}
	k=k+1;
      
    }}}
