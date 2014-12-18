
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


#include <iterator>     // ostream_operator
#include <boost/tokenizer.hpp>


#include "GraphFactory.h"
#include "muparo.h"
#include "run_configurations.h"
#include "utils.h"
#include "node_filter_utils.h"
#include "../RegLC/AlgoTypedefs.h"


#include "../tests/JsonWriter.h"

#include"../RegLC/reglc_graph.h"


#include "MPR_AspectPrivacy.h"

#include "utils/GeoTools.h"


using namespace MuPaRo;
using namespace AlgoMPR;
using namespace Privacy;

using namespace std;
using namespace boost;

JsonWriter * out;

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
  return j;
}

int proxima2(float lon, float lat,double d,const Transport::Graph * g ){
  
  int j=0;
  
  
  for(int i=0; i<g->num_vertices(); i++){
    
    Node n=g->mapNode(i);
   
    
      if(proximity(n.lon,n.lat,lon,lat,d)){
      j=i;
      }
    
  }
  return j;
}

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
    
    fout.close();
}

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
    
    fout.close();
}

void csv_writer(string fileName, NodeList nl,const Transport::Graph * g) {
    ofstream fout(fileName);
    fout << "longitude" <<','<< "latitude"<<','<<"name"<<'\n';
    BOOST_FOREACH(int i, nl){
       
        Node p=g->mapNode(i);
        fout <<  p.lon <<','<< p.lat <<','<<"P"<< '\n';	
    }
    fout.close();
}



int main(int argc, char*argv[]) 
{
      RLC::DFA dfa_car=RLC::car_dfa();
      RLC::DFA dfa_passenger=RLC::pt_foot_dfa();
  
  
      Transport::GraphFactory gf("/home/matchi/Desktop/ST/hercule.txt-dump", false);
      //gf.setAll2();
      const Transport::Graph * g = gf.get();
  
      typedef tokenizer< escaped_list_separator<char> > Tokenizer;

  
      int x_foot, y_foot, x_car, y_car;
      int alpha, beta; 
  
  //configuration
  //string data("/home/matchi/Desktop/ST/cov.csv");
  if (argc != 2){
   
   cout << "Bad Input" << endl;
   exit(0);
   
  }else{
    
    string data=argv[1];
    
    cout<<data<<endl;
    
    ifstream in(data.c_str());
    if (!in.is_open()) return 1;


    vector< string > vec;
    string line;
    int i=0;

    /*while (getline(in,line))
    {
        Tokenizer tok(line);
        vec.assign(tok.begin(),tok.end());
	if(i==1){
	string origine_foot=vec.at(0);
	string destination_foot=vec.at(1);
	string origine_car=vec.at(2);
	string destination_car=vec.at(3);
	
	string x=vec.at(4);
	string y=vec.at(5);
	
        int a=std::atoi(origine_foot.c_str());
	int b=std::atoi(destination_foot.c_str());
	int c=std::atoi(origine_car.c_str());
	int d=std::atoi(destination_car.c_str());
	
	int xx=std::atoi(x.c_str());
	int yy=std::atoi(y.c_str());
	
	//cout <<"départ piéton:: "<<a<<" arrivée piéton: "<<b<<endl;
	x_foot=a; 
	y_foot=b; 
	x_car=c; 
	y_car=d;
	alpha=xx;
	beta=yy;
	
	break;
	}
	
	i++;
	
    }*/
    
    
    while (getline(in,line))
    {
        Tokenizer tok(line);
        vec.assign(tok.begin(),tok.end());
	
	if(i>1){
	string origine_foot=vec.at(0);
	string destination_foot=vec.at(1);
	string origine_car=vec.at(2);
	string destination_car=vec.at(3);
	
	string x=vec.at(4);
	string y=vec.at(5);
	
        int a=std::atoi(origine_foot.c_str());
	int b=std::atoi(destination_foot.c_str());
	int c=std::atoi(origine_car.c_str());
	int d=std::atoi(destination_car.c_str());
	
	int xx=std::atoi(x.c_str());
	int yy=std::atoi(y.c_str());
	
	x_foot=a; 
	y_foot=b; 
	x_car=c; 
	y_car=d;
	alpha=xx;
	beta=yy;
	Privacy::Driver * d1=new Privacy::Driver(x_car, y_car);
	Privacy::Pedestrian * p1=new Privacy::Pedestrian(x_foot, y_foot);
	Privacy::Manager *m=new Privacy::Manager();
	BiList psi_result;
	
	do{
	p1->findZ(g,alpha,beta);
	d1->findZ(g,alpha,beta);
	psi_result=m->PSI(*d1,*p1);
	m->findA(g, psi_result);
	cout<<alpha<<endl;
	alpha=alpha+50;
	beta=beta+50;
	if(alpha>201){
	  cout<<"pas de solution"<<endl;
	  break;}}
	while(m->shared_path.size()<=0);
      
	if(m->shared_path.size()>=0){
	d1->getFavorites(m->shared_path);
	p1->getFavorites(m->shared_path);
	m->match(*d1,*p1);
	save_position2("/home/matchi/Desktop/ST/results/Scenario"+std::to_string(i) +".csv", *d1, *p1,*m,g);}}
	
	i++;}
      
  }
  

  
   //***************************************************************   
     
      
      
      
      //*****************************************
      /*string cov("/home/matchi/Desktop/ST/cov.csv");

    ifstream in2(cov.c_str());
    if (!in2.is_open()) return 1;

    

    vector< string > vec2;
    string line2;
    int ii=0;

    while (getline(in2,line2))
    {
      cout <<ii<<endl;
        Tokenizer tok2(line2);
        vec2.assign(tok2.begin(),tok2.end());
	
	string lo=vec2.at(0);
	string la=vec2.at(1);
	
        float lon=std::stof(lo);
	float lat=std::stof(la);
	int start_car2=proxima2(lon,lat, 100, g);
	cout <<"hotspot:  "<<start_car2<<endl;
	Privacy::Driver * d1=new Privacy::Driver(start_car2,proxima(start_car2,5300,g));
	Privacy::Pedestrian * p1=new Privacy::Pedestrian(proxima(start_car2,500,g), proxima(start_car2,6200,g));
	Privacy::Manager *m=new Privacy::Manager();
	BiList psi_result;
	
	do{
	
	p1->findZ(g,alpha,beta);
	d1->findZ(g,alpha,beta);
	psi_result=m->PSI(*d1,*p1);
	m->findA(g, psi_result);
	
	cout<<alpha<<endl;
	
	alpha=alpha+50;
	beta=beta+50;
	
	if(alpha>121) {
	  cout<<"pas de solution"<<endl;
	  break;
	  
	}
	
      }
      while(m->shared_path.size()<=0);
      
      if(m->shared_path.size()){
	d1->getFavorites(m->shared_path);
	p1->getFavorites(m->shared_path);
	m->match(*d1,*p1);
	save_position2("/home/matchi/Desktop/ST/Tests/Scenario"+std::to_string(ii)+".csv", *d1, *p1,*m,g);}
	
	//ii++;
	
	//if(ii==10)
	  //break;


    }*/
      
      
      //***********************************
      
      
      
      
      
      //NodeList nl=g->PickUpZone();
      
      //csv_writer("/home/matchi/Desktop/ST/show.csv", nl,g);
      
     
     
      /*
      Privacy::Driver * d1=new Privacy::Driver(x_car, y_car);
      Privacy::Pedestrian * p1=new Privacy::Pedestrian(x_foot, y_foot);
      
      save_position("/home/matchi/Desktop/ST/position.csv", *d1, *p1,g);
      Privacy::Manager *m=new Privacy::Manager();
      BiList psi_result;
      
      
      d1->test(g,alpha,beta);
      cout<<"------------------------------------------------------------------------"<<endl;
      p1->test(g,alpha,beta);
      
      do{
	
	p1->findZ(g,alpha,beta);
	d1->findZ(g,alpha,beta);
	psi_result=m->PSI(*d1,*p1);
	m->findA(g, psi_result);
	
	cout<<alpha<<endl;
	
	alpha=alpha+50;
	beta=beta+50;
	
	if(alpha>121) {
	  cout<<"pas de solution"<<endl;
	  break;
	  
	}
	
      }
      while(m->shared_path.size()<=0);
	
      if(m->shared_path.size()){
      
      
      
      
      cout<<"---------------------------------------------------------------"<<endl;
      cout<<"pick up foot: "<<p1->data_before.size()<<endl;
      cout<<"drop off foot: "<<p1->data_after.size()<<endl;
      cout<<"---------------------------------------------------------------"<<endl;
      cout<<"pick up  car: "<<d1->data_before.size()<<endl;
      cout<<"drop off car: "<<d1->data_after.size()<<endl;
      cout<<"------------------------------------------------------------------"<<endl;
      
      cout<<"---resultat---------: "<<m->shared_path.size()<<endl;
      
      int c=0;
      
       BOOST_FOREACH(path p, m->shared_path){
	 
	 cout<<"Path id: "<< p.id<<" | "<<"depart: "<<p.start<<" end: "<<p.end<<" cost: "<<p.cost<<endl;
	 c++;
	 //if(c==10) break;
	 
       }
      cout<<"------------------------------------------------------------------"<<endl;
      d1->getFavorites(m->shared_path);
      p1->getFavorites(m->shared_path);

      m->match(*d1,*p1);

      path best=m->getThePath();
    
      cout <<"the best path is: " <<best.id <<" starting from: "<<best.start<< " and ending at "<<best.end<<" with the cost: "<<best.cost<<endl;
      
      
	
      }
      */
      
      
      
      
      //-------------------------Car sharing----------------------------------------------------------------------
   /* {
	
    JsonWriter writer("/home/matchi/Desktop/ST/carsharing.txt");
    out = &writer;
    
       
      int passenger_start_node=start_foot, car_start_node=start_car, passenger_arrival_node=end_foot, car_arrival_node=end_car;

      typedef CarSharingTest CurrAlgo;
        
        out->step_in("original");

        START_TICKING;
        CurrAlgo::ParamType p(
            MuparoParams( g, 5 ),
            AspectTargetParams( 4, passenger_arrival_node ),
            AspectPropagationRuleParams( SumPlusWaitCost, MaxArrival, 2, 0, 1),
            AspectPropagationRuleParams( SumCost, FirstLayerArrival, 4, 2, 3)
        );
        
        CurrAlgo cs( p );
	
        
        init_car_sharing<CurrAlgo>( &cs, g, passenger_start_node, car_start_node, passenger_arrival_node, car_arrival_node, dfa_passenger, dfa_car );

        STOP_TICKING;
        out->add("init-time", RUNTIME);
        
        START_TICKING;
        cs.run();
        STOP_TICKING;
        
        out->add("runtime", RUNTIME);
        out->add("visited-nodes", cs.count);
        std::vector<int> per_layer;
        int tot = 0;
        for(int i=0 ; i<cs.num_layers ; ++i) {
            per_layer.push_back( cs.dij[i]->count );
            tot += cs.dij[i]->count;
        }
        cout << cs.count <<" "<< tot <<endl;
        out->add("visited-per-layer", per_layer);
        out->add("solution-cost", cs.solution_cost());
        
        
        out->step_in("node-details");
        {
            out->step_in("pass-arr");
            out->add("node", passenger_arrival_node);
            out->step_in("5");
            out->add("cost", cs.get_cost(4, passenger_arrival_node));
            out->add("arrival", cs.arrival(4, passenger_arrival_node));
            
            out->step_out(); out->step_out();
        }
        
        int drop_off = cs.get_source(4, passenger_arrival_node);
        {
            out->step_in("drop_off");
            out->add("node", drop_off);
            
            out->step_in("5");
            out->add("cost", cs.get_cost(4, drop_off));
            out->add("arrival", cs.arrival(4, drop_off));
            out->step_out(); 
            
            out->step_in("4");
            out->add("cost", cs.get_cost(3, drop_off));
            out->add("arrival", cs.arrival(3, drop_off));
            out->step_out(); 
            
            out->step_in("3");
            out->add("cost", cs.get_cost(2, drop_off));
            out->add("arrival", cs.arrival(2, drop_off));
            out->step_out(); 
            
            out->step_out();
        }
        
        int pick_up = cs.get_source(2, drop_off);
        {
            out->step_in("pick-up");
            out->add("node", drop_off);
            
            out->step_in("3");
            out->add("cost", cs.get_cost(2, pick_up));
            out->add("arrival", cs.arrival(2, pick_up));
            out->step_out(); 
            
            out->step_in("2");
            out->add("cost", cs.get_cost(1, pick_up));
            out->add("arrival", cs.arrival(1, pick_up));
            out->step_out(); 
            
            out->step_in("1");
            out->add("cost", cs.get_cost(0, pick_up));
            out->add("arrival", cs.arrival(0, pick_up));
            out->step_out(); 
            
            out->step_out();
        }
        
        
        //long int t = static_cast<long int> (time(NULL));
        out->step_out(); //node details
        
        {
            
	    
	    out->add("1-length", cs.get_cost(0, pick_up));
            out->add("2-length", cs.get_cost(1, pick_up));
            out->add("3-length", cs.arrival(2, drop_off) - cs.arrival(2, pick_up));
            out->add("4-length", cs.get_cost(3, drop_off));
            out->add("5-length", cs.arrival(4, passenger_arrival_node) - cs.arrival(4, drop_off));
            out->add("wait-time", abs(cs.arrival(0, pick_up) - cs.arrival(1, pick_up)) );
            out->add("num-pick-up", cs.num_pick_up);
            out->add("num-drop-off", cs.num_drop_off);
	    
	    
	    out->add("pick-up node", pick_up);
            out->add("drop-off node", drop_off);
            
//             out->add("nodes-set-in-MOM", cs.dij[4]->count_set);
//             out->add("avg-label-in-MOM", cs.dij[4]->average_label);
        }
        
        out->step_out();
	}
    */
    
    
    
    
}
