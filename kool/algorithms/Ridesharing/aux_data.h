#pragma once
#include "header.h"

namespace Privacy { 

RLC::DFA dfa_car=RLC::car_dfa();
RLC::DFA dfa_passenger=RLC::pt_foot_dfa();

struct location{
	float lon;
	float lat;
};

struct journey{
	location origin;
	location destination;
};

struct data{

  int node, cost;
};

struct harvData{
	
  int node; 
  double distance;
};

bool compare_harvData(harvData h1, harvData h2){

  return h1.distance<h2.distance;
}

typedef std::vector<harvData> harvDataList;
    
struct totalCost{

  int path_id, cost;
};
    
typedef std::list<totalCost>costVector;

typedef std::list<data> dataList;

typedef std::map<int,data> database;
      
struct pref{
  int id, weight;
  bool operator==(const pref a)
    {
	return id==a.id;
    }
};

typedef std::vector<pref> prefs;
typedef std::map<int,pref> prefbase;

  
struct path {
  int id, start, end, cost;
  bool operator==(const path a)
    {
	return id==a.id;
    }	  
};

struct poi
{
  int id;
  Node node;
  bool operator==(const poi other){
    return id==other.id;
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

struct detour
{
  int original_route_cost;
  int ridesharing_route_cost;
  int detour;
};

struct distanceBw
{
  double at_origin;
  double at_destination;

};

/*
 * structure for test output
 */


struct Test_output{
  
  int scenario_id;

  int isochrone_radius;
  int  step;
  
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
  string psi_time;
  
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

  int cc_success;
  int ccl_success;
  int cd_success;

  detour cc_detour;
  detour cd_detour;

  distanceBw proximity;
  
  Test_config input;
  
};

struct Output{
      int scenario_id;
      int isochrone_radius;
      int  step;
      int rider_origin;
      int driver_origin;
      int rider_destination;
      int driver_destination;
      int optimal_pickup;
      int privacy_pickup;
      int optimal_dropoff;
      int privacy_dropoff;
      //int harversine_distance_origin;
      //int harversine_distance_destination;
      //int harversine_distance_pickup;
      //int harversine_distance_dropoff;
      int rider_pickup_size;
      int driver_pickup_size;
      int num_steps_pickup;
      int pickup_size;
      int rider_dropoff_size;
      int driver_dropoff_size;
      int num_steps_dropoff;
      int dropoff_size;
      int shared_path_size;

      double rider_origin_isochrone_time;
      double driver_origin_isochrone_time;
      double rider_destination_isochrone_time;
      double driver_destination_isochrone_time;
      double psi_pickup_time;
      double psi_dropoff_time;
      double shared_path_computing_time;
      double rider_path_ordering_time;
      double driver_path_ordering_time;
      double path_election_time;
      double rider_privacy_runtime;
      double driver_privacy_runtime;
      double optimal_runtime;
      int optimal_total_cost;
      int privacy_total_cost;
      int optimal_waiting_time;
      int privacy_waiting_time;
      int driver_original_trip_cost;
      int rider_original_trip_cost;
      Five_cost cd_costs;
      Five_cost cc_costs;
      Positions cd_positions;
      Positions cc_positions;
      Test_config input;
};

/*
 * structure for cc_test output
 */

struct cc_output{
  int cc_pickup;
  int cc_dropoff;
  double cc_time;
  Five_cost cc_costs;
  Positions cc_positions;
};


struct cd_output{ 
  int cd_pickup;
  int cd_dropoff;  
  string cd_time;
  Five_cost cd_costs;
  Positions cd_positions;
};



struct genData
  {
    string fileName;
    float synchro2_a;
    float synchro2_b;
    float foot1_a;
    float foot1_b;
    float foot2_a;
    float foot2_b;
    float car1_a;
    float car1_b;
    float car2_a;
    float car2_b;
    int number; 
    const Transport::Graph * g;
  };



      
typedef std::vector<path> pathList;

  
struct BiList{
  NodeList PickUp, DropOff;
};

struct TwoIso
{
  int start;
  dataList driverList, riderList;
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


data getData2(int id, database myData){
  return myData[id];
}

pref getPref2(int id, prefbase myPrefs){
  return myPrefs[id];
}
  
bool compare_cost(totalCost t1, totalCost t2){

  return t1.cost<t2.cost;
}
  
bool compare_pref(pref p1, pref p2){

  return p1.weight<p2.weight;
}

}