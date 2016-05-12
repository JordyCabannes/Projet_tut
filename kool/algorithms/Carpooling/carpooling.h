#pragma once
#include "../DataStructures/GraphFactory.h"
#include "../MultipleParticipants/MPR_AspectPrivacy.h"
#include "../Model/Model.h"

#include <boost/graph/cycle_canceling.hpp>
#include <boost/graph/edmonds_karp_max_flow.hpp>

#include <boost/lexical_cast.hpp>

#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <iterator>  
#include <boost/tokenizer.hpp>
#include <vector>
#include <climits>
#include <string>


using namespace boost;

namespace carpooling {

  typedef adjacency_list_traits < vecS, vecS, directedS > Traits;

  typedef adjacency_list < vecS, vecS, directedS, no_property,
  property < edge_capacity_t, long,
  property < edge_residual_capacity_t, long,
  property < edge_reverse_t, Traits::edge_descriptor,
  property <edge_weight_t, long>
  >
  >
  > > Graph;

  typedef property_map < Graph, edge_capacity_t >::type Capacity;
  typedef property_map < Graph, edge_residual_capacity_t >::type ResidualCapacity;
  typedef property_map < Graph, edge_weight_t >::type Weight;
  typedef property_map < Graph, edge_reverse_t>::type Reversed;
  typedef boost::graph_traits<Graph>::vertices_size_type size_type;
  typedef Traits::vertex_descriptor vertex_descriptor;

  class EdgeAdder {
    public:
    EdgeAdder(Graph & g, Weight & w, Capacity & c, Reversed & rev, ResidualCapacity & residualCapacity)
    : m_g(g), m_w(w), m_cap(c), m_resCap(residualCapacity), m_rev(rev) {}
    void addEdge(vertex_descriptor v, vertex_descriptor w, long weight, long capacity) {
    Traits::edge_descriptor e,f;
    e = add(v, w, weight, capacity);
    f = add(w, v, -weight, 0);
    m_rev[e] = f;
    m_rev[f] = e;
    }
    private:
    Traits::edge_descriptor add(vertex_descriptor v, vertex_descriptor w, long weight, long capacity) {
    bool b;
    Traits::edge_descriptor e;
    boost::tie(e, b) = add_edge(vertex(v, m_g), vertex(w, m_g), m_g);
    if (!b) {
    std::cerr << "Edge between " << v << " and " << w << " already exists." << std::endl;
    std::abort();
    }
    m_cap[e] = capacity;
    m_w[e] = weight;
    return e;
    }
    Graph & m_g;
    Weight & m_w;
    Capacity & m_cap;
    ResidualCapacity & m_resCap;
    Reversed & m_rev;
  };


  struct SampleGraph {

    /*static void carpoolGraph_test(Graph &g, vertex_descriptor & s, vertex_descriptor & t) {
      size_type N(7);
      typedef property_map < Graph, edge_reverse_t >::type Reversed;
      for(size_type i = 0; i < N; ++i){
      add_vertex(g);
      }
      Capacity capacity = get(edge_capacity, g);
      Reversed rev = get(edge_reverse, g);
      ResidualCapacity residual_capacity = get(edge_residual_capacity, g);
      Weight weight = get(edge_weight, g);
      s = 0;
      t = 6;
      EdgeAdder ea(g, weight, capacity, rev, residual_capacity);
      ea.addEdge(0, 1, 0 ,1);
      ea.addEdge(0, 2, 0 ,1);
      ea.addEdge(0, 5, 0 ,1);


      ea.addEdge(1, 3, 5 ,1);
      ea.addEdge(1, 4, 10 ,1);

      ea.addEdge(2, 3, 10 ,1);
      ea.addEdge(2, 4, 6 ,1);

      ea.addEdge(5, 4, 8 ,1);

      ea.addEdge(3, 6, 0 ,1);
      ea.addEdge(4, 6, 0 ,1);
    }*/

    static void carpoolGraph(int m, int n, Graph &g, vertex_descriptor & s, vertex_descriptor & t){

      size_type N(m+n+2);

      typedef property_map < Graph, edge_reverse_t >::type Reversed;

      for(size_type i = 0; i < N; ++i){
      add_vertex(g);
      }
      Capacity capacity = get(edge_capacity, g);
      Reversed rev = get(edge_reverse, g);
      ResidualCapacity residual_capacity = get(edge_residual_capacity, g);
      Weight weight = get(edge_weight, g);
      s = 0;
      t = m+n+1;

      EdgeAdder ea(g, weight, capacity, rev, residual_capacity);

      for(int v=1; v<=m; v++){
          ea.addEdge(s, v, 0 ,1);
      }

      for(int v=m+1; v<=m+n; v++){
          ea.addEdge(v, t, 0 ,1);
      }

      for(int i=1; i<=m; i++){
          for(int j=m+1; j<=m+n; ++j){
              ea.addEdge(i, j, 10 ,1);
          }
      }

    }

    static User findByPk_driver(int k, Users myUsers){

      return myUsers.drivers[k];
    }

    static User findByPk_passenger(int k, Users myUsers){

      return myUsers.passengers[k];
    }

    static void heuristik_Graph(Users users, Graph &g, vertex_descriptor & s, vertex_descriptor & t){

      int m=users.drivers.size();
      int n=users.passengers.size();

      size_type N(m+n+2);

      typedef property_map < Graph, edge_reverse_t >::type Reversed;

      for(size_type i = 0; i < N; ++i){
      add_vertex(g);
      }
      Capacity capacity = get(edge_capacity, g);
      Reversed rev = get(edge_reverse, g);
      ResidualCapacity residual_capacity = get(edge_residual_capacity, g);
      Weight weight = get(edge_weight, g);
      s = 0;
      t = m+n+1;

      EdgeAdder ea(g, weight, capacity, rev, residual_capacity);

      for(int v=1; v<=m; v++){
          ea.addEdge(s, v, 0 ,1);
      }

      for(int v=m+1; v<=m+n; v++){
          ea.addEdge(v, t, 0 ,1);
      }

      for(int i=1; i<=m; i++){
          User _driver=findByPk_driver(i-1,users);
          for(int j=m+1; j<=m+n; ++j){
            User _pedestrian= findByPk_passenger(j-m-1,users); 
            ea.addEdge(i, j, _driver.get_distance_to(_pedestrian) ,1);
          }
      }

    }

    
    static void ideal_Graph(Users users, Graph &g, vertex_descriptor & s, vertex_descriptor & t, const Transport::Graph * gr){

      

      int m=users.drivers.size();
      int n=users.passengers.size();

      size_type N(m+n+2);

      typedef property_map < Graph, edge_reverse_t >::type Reversed;

      for(size_type i = 0; i < N; ++i){
      add_vertex(g);
      }
      Capacity capacity = get(edge_capacity, g);
      Reversed rev = get(edge_reverse, g);
      ResidualCapacity residual_capacity = get(edge_residual_capacity, g);
      Weight weight = get(edge_weight, g);
      s = 0;
      t = m+n+1;

      EdgeAdder ea(g, weight, capacity, rev, residual_capacity);

      for(int v=1; v<=m; v++){
          ea.addEdge(s, v, 0 ,1);
      }

      for(int v=m+1; v<=m+n; v++){
          ea.addEdge(v, t, 0 ,1);
      }

      for(int i=1; i<=m; i++){
          

          for(int j=m+1; j<=m+n; ++j){

            //cout << "Driver number: " << i << " Passenger number: " << j <<endl;
            int cost;
            Privacy::cc_output cc;

            User _driver=findByPk_driver(i-1,users);
            Privacy::Driver *d1=new Privacy::Driver(_driver.origin, _driver.destination);

            //cout << "Driver origin: " << _driver.origin << " Driver destination: " << _driver.destination <<endl;

            User _pedestrian= findByPk_passenger(j-m-1,users);
            Privacy::Pedestrian *p1=new Privacy::Pedestrian(_pedestrian.origin, _pedestrian.destination);

            //cout << "Passenger origin: " << _pedestrian.origin << " Passenger destination: " << _pedestrian.destination <<endl;


            Privacy::Toolbox *tb=new Privacy::Toolbox();
          
            cc=tb->cc_carpooling_test(*d1,*p1,gr);
           

            /*if (cc.cc_costs.total_cost >= 0)
            {
              cost=cc.cc_costs.total_cost;
            }
            if (cc.cc_costs.total_cost < 0){
              cost=std::numeric_limits<int>::max();
            }*/

            cost=cc.cc_costs.total_cost;

            //cout << "the cost is ------------------------------------> " << cost << endl;

            ea.addEdge(i, j, cost ,1);
            delete d1;
            delete p1;
            delete tb;
          }


      }

    }



    static void save_to_carpooling(string filename, const Transport::Graph * g, string path){  
      
      typedef boost::tokenizer< boost::char_separator<char> > Tokenizer;
      boost::char_separator<char> sep(";");
      
      ifstream in(filename.c_str());

      if (!in.is_open()) exit(0);

      vector< string > vec;
      string line;
      int i=0;

      ofstream fout;
     
      fout.open (path, ofstream::out | ofstream::app);

      fout <<"car1"<<';'<<"car1 lon"<<';'<<"car1 lat"<<';';
      fout <<"foot1"<<';'<<"foot1 lon"<<';'<<"foot1 lat"<<';';
      fout <<"car2"<<';'<<"car2 lon"<<';'<<"car2 lat"<<';';
      fout <<"foot2"<<';'<<"foot2 lon"<<';'<<"foot2 lat"<<';'<<'\n';

      while (getline(in,line)){
        
        Tokenizer tok(line,sep);
        vec.assign(tok.begin(),tok.end());

        if(i>0){
        int car1=std::atoi(vec.at(0).c_str());
        int foot1=std::atoi(vec.at(1).c_str());
        
        int car2=std::atoi(vec.at(2).c_str());
        int foot2=std::atoi(vec.at(3).c_str());

        Node car_origin =g->mapNode(car1);
        Node foot_origin =g->mapNode(foot1);

        Node car_destination =g->mapNode(car2);
        Node foot_destination =g->mapNode(foot2);

        fout <<car1<<';'<<car_origin.lon<<';'<<car_origin.lat<<';';
        fout <<foot1<<';'<<foot_origin.lon<<';'<<foot_origin.lat<<';';
        fout <<car2<<';'<<car_destination.lon<<';'<<car_destination.lat<<';';
        fout <<foot2<<';'<<foot_destination.lon<<';'<<foot_destination.lat<<';'<<'\n';

        
        }
        i++;
      }
    }

    static std::pair< std::map< int, User>, std::map<int, User> > load_to_map(string filename){  
      
      typedef boost::tokenizer< boost::char_separator<char> > Tokenizer;
      boost::char_separator<char> sep(";");
      
      ifstream in(filename.c_str());

      if (!in.is_open()) exit(0);

      vector< string > vec;
      string line;
      int i=0;

      std::map<int, User> drivers;
      std::map<int, User> pedestrians;

      

      while (getline(in,line)){
        
        Tokenizer tok(line,sep);
        vec.assign(tok.begin(),tok.end());

        if(i>0){

        //cout<<"iteration: "<<i<<endl;

        Journey driver, pedestrian;
        int car1, car2, foot1, foot2;

        car1=std::atoi(vec.at(0).c_str());
        car2=std::atoi(vec.at(6).c_str());

        foot1=std::atoi(vec.at(3).c_str());
        foot2=std::atoi(vec.at(9).c_str());

        driver.origin.lon=std::atof(vec.at(1).c_str());
        driver.origin.lat=std::atof(vec.at(2).c_str());
        driver.destination.lon=std::atof(vec.at(7).c_str());
        driver.destination.lat=std::atof(vec.at(8).c_str());

        User A(i);
        A.setJourney(driver);
        A.setOrigin(car1);
        A.setDestination(car2);
        drivers.insert(std::pair<int,User>(i, A));

        pedestrian.origin.lon=std::atof(vec.at(4).c_str());
        pedestrian.origin.lat=std::atof(vec.at(5).c_str());
        pedestrian.destination.lon=std::atof(vec.at(10).c_str());
        pedestrian.destination.lat=std::atof(vec.at(11).c_str());

        User B(i);
        B.setOrigin(foot1);
        B.setDestination(foot2);
        B.setJourney(pedestrian);
        pedestrians.insert(std::pair<int,User>(i, B));
        
        
        }
        i++;
      }
  

      return std::pair<std::map<int, User>,std::map<int, User>>(drivers,pedestrians);
    }


    static Users load(string filename){  
      
      typedef boost::tokenizer< boost::char_separator<char> > Tokenizer;
      boost::char_separator<char> sep(";");
      
      ifstream in(filename.c_str());

      if (!in.is_open()) exit(0);

      vector< string > vec;
      string line;
      int i=0;

      Users myUsers;

      

      while (getline(in,line)){
        
        Tokenizer tok(line,sep);
        vec.assign(tok.begin(),tok.end());

        if(i>0){

        //cout<<"iteration: "<<i<<endl;

        Journey driver, pedestrian;
        int car1, car2, foot1, foot2;

        car1=std::atoi(vec.at(0).c_str());
        car2=std::atoi(vec.at(6).c_str());

        foot1=std::atoi(vec.at(3).c_str());
        foot2=std::atoi(vec.at(9).c_str());

        driver.origin.lon=std::atof(vec.at(1).c_str());
        driver.origin.lat=std::atof(vec.at(2).c_str());
        driver.destination.lon=std::atof(vec.at(7).c_str());
        driver.destination.lat=std::atof(vec.at(8).c_str());

        User A(i);
        A.setJourney(driver);
        A.setOrigin(car1);
        A.setDestination(car2);
        myUsers.drivers.push_back(A);

        pedestrian.origin.lon=std::atof(vec.at(4).c_str());
        pedestrian.origin.lat=std::atof(vec.at(5).c_str());
        pedestrian.destination.lon=std::atof(vec.at(10).c_str());
        pedestrian.destination.lat=std::atof(vec.at(11).c_str());

        User B(i);
        B.setOrigin(foot1);
        B.setDestination(foot2);
        B.setJourney(pedestrian);
        myUsers.passengers.push_back(B);
        
        }
        i++;
      }
  

      return myUsers;
    }


    static void goProjection(){
      
    }







};



  

} //boost