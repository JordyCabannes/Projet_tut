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

#pragma once
#pragma once
#include "header.h"
#include "aux_data.h"
#include "driver.h"
#include "rider.h"
#include "manager.h"


using namespace MuPaRo;
 
namespace Privacy {

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
   * returns a vector of pick up nodes located between min_d and max_d from node
   */
  std::vector<int> mapCoord(float lon, float lat,  double max_d, const Transport::Graph * g){
    std::vector<int> v;
    for(int i=0; i<g->num_vertices(); i++){
      Node n=g->mapNode(i);
      if( proximity(n.lon,n.lat,lon,lat,max_d) && n.pick_up ){
        v.push_back(i);
      }
    }
    //cout << "taille de v: " << v.size()<<endl;
    return v;
  }


  int mapCoord_(float lon, float lat, const Transport::Graph * g){

    harvDataList hd;

    for(int i=0; i<g->num_vertices(); i++){
      Node n=g->mapNode(i);
      harvData h;
      h.node=i;
      h.distance=proximity_d(n.lon,n.lat,lon,lat);
      hd.push_back(h); 
    }

    std::sort(hd.begin(), hd.end(), compare_harvData);
    cout << "begin: " << hd[0].distance << " , end: "<<hd[hd.size()-1].distance <<endl;
    return hd[0].node;
  }


  /*
   * return a random locations among node
   */
  int getBestNode(std::vector<int> satellites){
    int r;
    int min=0;
    int max=satellites.size()-1;
    
    if (max>0) {
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

  std::map<int, Node> get_zone(float max_lon, float min_lon, float max_lat, float min_lat,int number, const Transport::Graph * g){
    std::map<int, Node> zone;
    int compt=0;

    //int min=0;
    //int max=g->num_vertices()-1;

    //while (compt<number){

    for(int i=0; i<g->num_vertices(); i++){

      Node node=g->mapNode(i);
      bool in = node.lon < max_lon 
        && node.lon > min_lon
        && node.lat < max_lat
        && node.lat > min_lat;

        //map<int,Node>::iterator it = zone.find(i);

        if(in){
          zone[i]=node;
          compt++;
          cout << "number of points: " << compt << endl;
        }

        }
    //}

    return zone;
  }


  std::vector<poi> get_area(float max_lon, float min_lon, float max_lat, float min_lat,int number, const Transport::Graph * g){
    std::vector<poi> zone;
    int compt=0;

    for(int i=0; i<g->num_vertices(); i++){
      Node node=g->mapNode(i);
      bool in = node.lon < max_lon 
        && node.lon > min_lon
        && node.lat < max_lat
        && node.lat > min_lat;
        if(in){
          poi x;
          x.id=i;
          x.node=node;
          zone.push_back(x);
          compt++;
          cout << "number of points: " << compt << endl;
        }
      }
        int inc=0;
        int min=0;
        int max=zone.size()-1;

        std::vector<poi> my_area;

        while(inc<=number){
          int j=rgenerator(min,max);
          poi x=zone[j];
          auto ck=std::find(my_area.begin(), my_area.end(), x);
          if (ck == my_area.end()){
            my_area.push_back(x);
            inc++;
          }
        }
    return my_area;
  }

  TwoIso get_iso(const Transport::Graph * g, int start, int limit){
    
    
    TwoIso my_bilist;
    my_bilist.start=start;

    RLC::Graph *riderGraph=new RLC::Graph( g,dfa_passenger );
    RLC::Graph *driverGraph=new RLC::Graph( g,dfa_car );

    my_isochrone riderIso=cool_iso (riderGraph, start, limit);

    for(int i=0; i<g->num_vertices(); i++){
      Node k=g->mapNode(i);
      if (riderIso.ns->bitset[i] && k.pick_up && i!=start){
        data db;
        db.node=i;
        db.cost=riderIso.costs[i];
        my_bilist.riderList.push_back(db);
      }
    }

    my_isochrone driverIso=cool_iso (driverGraph, start, limit);

    for(int i=0; i<g->num_vertices(); i++){
      Node k=g->mapNode(i);
      if (driverIso.ns->bitset[i] && k.pick_up && i!=start){
        data db;
        db.node=i;
        db.cost=driverIso.costs[i];
        my_bilist.driverList.push_back(db);
      }
    }

    return my_bilist;

  }

  int iso_rider_size(TwoIso iso){
    return iso.riderList.size();
  }

  int iso_driver_size(TwoIso iso){
    return iso.driverList.size();
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
      
      Node pick=g->mapNode(m.getThePath_great().start);
      Node drop=g->mapNode(m.getThePath_great().end);
      
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
      
      AlgoMPR::PtToPt * c1 =  point_to_point( g,foot.posStart , m.getThePath_great().start, dfa_passenger);
      AlgoMPR::PtToPt * c2 =  point_to_point( g,car.posStart , m.getThePath_great().start, dfa_car);
      
      AlgoMPR::PtToPt * c3 =  point_to_point( g, m.getThePath_great().start, m.getThePath_great().end, dfa_car);
      
      AlgoMPR::PtToPt * c4 =  point_to_point( g, m.getThePath_great().end, foot.posEnd, dfa_passenger);
      AlgoMPR::PtToPt * c5 =  point_to_point( g,m.getThePath_great().end, car.posEnd, dfa_car);
      
      c1->run();
      len1=c1->get_cost(0, m.getThePath_great().start);
      delete c1;
      
      c2->run();
      len2=c2->get_cost(0, m.getThePath_great().start);
      delete c2;
      
      c3->run();
      len3=c3->get_cost(0, m.getThePath_great().end);
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

  /*detour cc_detour(Driver car, const Transport::Graph * g, cc_output cc ){
    detour my_detour;

    int waiting_time=0;

    if(cc.cc_costs.len_car_pickup<cc.cc_costs.len_foot_pickup){
      waiting_time=cc.cc_costs.wait_time;
    }

    my_detour.ridesharing_route_cost=cc.cc_costs.len_car_pickup + cc.cc_costs.len_shared_path + cc.cc_costs.len_car_dropoff+waiting_time;

    AlgoMPR::PtToPt * c2 =  point_to_point( g,car.posStart , car.posEnd, dfa_car);
    c2->run();
    my_detour.original_route_cost=c2->get_cost(0, car.posEnd);
    delete c2;

    my_detour.detour=my_detour.ridesharing_route_cost - my_detour.original_route_cost;

    return my_detour;
  }*/

 

  distanceBw get_proximity(Pedestrian foot, Driver car, const Transport::Graph * g){
    distanceBw proximity;

    Node car1=g->mapNode(car.posStart);
    Node car2=g->mapNode(car.posEnd);

    Node foot1=g->mapNode(foot.posStart);
    Node foot2=g->mapNode(foot.posEnd);

    double origin = haversine_dist(car1.lon, car1.lat, foot1.lon, foot1.lat);
    double destination = haversine_dist(car2.lon, car2.lat, foot2.lon, foot2.lat);

    proximity.at_origin=origin;
    proximity.at_destination=destination;
    

    return proximity;
  }  

  int pointTopoint(const Transport::Graph * g, int start, int stop, bool car){
    int res;
    if (car)
    {
      AlgoMPR::PtToPt * alg =  point_to_point( g, start , stop, dfa_car);
      alg->run();
      res = alg->get_cost(0, stop);
    }
    else{
      AlgoMPR::PtToPt * alg =  point_to_point( g, start , stop, dfa_passenger);
      alg->run();
      res = alg->get_cost(0, stop);
    }
    return res;
  }

  detour cc_detour(Driver car, const Transport::Graph * g, cc_output cc ){
    detour my_detour;

    int waiting_time=0;

    if(cc.cc_costs.len_car_pickup<cc.cc_costs.len_foot_pickup){
      waiting_time=cc.cc_costs.wait_time;
    }

    my_detour.ridesharing_route_cost=cc.cc_costs.len_car_pickup + cc.cc_costs.len_shared_path + cc.cc_costs.len_car_dropoff+waiting_time;

    AlgoMPR::PtToPt * c2 =  point_to_point( g, car.posStart , car.posEnd, dfa_car);
    c2->run();
    my_detour.original_route_cost=c2->get_cost(0, car.posEnd);
    delete c2;

    my_detour.detour=my_detour.ridesharing_route_cost - my_detour.original_route_cost;

    return my_detour;
  }



  detour cd_detour(Driver car, const Transport::Graph * g, Five_cost cd ){
    detour my_detour;

    int waiting_time=0;

    if(cd.len_car_pickup<cd.len_foot_pickup){
      waiting_time=cd.wait_time;
    }

    my_detour.ridesharing_route_cost=cd.len_car_pickup + cd.len_shared_path + cd.len_car_dropoff+waiting_time;

    AlgoMPR::PtToPt * c2 =  point_to_point( g,car.posStart , car.posEnd, dfa_car);
    c2->run();
    my_detour.original_route_cost=c2->get_cost(0, car.posEnd);
    delete c2;

    my_detour.detour=my_detour.ridesharing_route_cost - my_detour.original_route_cost;

    return my_detour;
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
    output.cc_time=boost::lexical_cast<double>(RUNTIME);
    
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

    
    
    //delete &cs;
    //delete &car;
    //delete &foot;
    //delete g;
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
    output.cc_time=boost::lexical_cast<double>(RUNTIME);
    
    
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
  cd_output cd_carpooling_test(Driver car, Pedestrian foot, Manager m, const Transport::Graph * g, int iso, int step, float ratio){
    cd_output output;
    Five_cost costs;
    NodeList psi_pickup, psi_dropoff;
    BiList psi_result;
    START_TICKING;
    car.findPickup(g,iso);
    car.findDropoff(g,iso);
    foot.findPickup(g,iso);
    foot.findDropoff(g,iso);
    int Ppistar=0;
    int Ppiend=step;
    int Pdostar=0;
    int Pdoend=step;
    while (Ppiend <= iso){
      foot.choosePickup(Ppistar, Ppiend);
      car.choosePickup(Ppistar/ratio, Ppiend/ratio);
      psi_pickup=m.PSI_Pickup(car, foot);
      //Ppistar=Ppiend;
      Ppiend+=step;
      if (psi_pickup.size()>50) break;
    }
    while (Pdoend <=iso) {
      foot.chooseDropoff(Pdostar, Pdoend);
      car.chooseDropoff(Pdostar/ratio, Pdoend/ratio);
      psi_dropoff=m.PSI_Dropoff(car, foot);
      //Pdostar=Pdoend;
      Pdoend+=step;
      if (psi_dropoff.size()>50)  break;
    }
    psi_result=m.getPSI(psi_pickup, psi_dropoff);
    m.GetAllPath(g, psi_result);
    if(m.shared_path_len()>0) {
      car.getFavorites(m.shared_path);
      foot.getFavorites(m.shared_path);
      m.match(car,foot);
      output.cd_positions=cd_carpooling_positions(car,foot,m,g);
      output.cd_costs=cd_costs(car,foot,m,g);
      output.cd_pickup=m.getThePath_great().start;
      output.cd_dropoff=m.getThePath_great().end;
    }
    else{
      costs.len_foot_pickup=0;
      costs.len_car_pickup=0;
      costs.len_shared_path=0;
      costs.len_foot_dropoff=0;
      costs.len_car_dropoff=0;
      costs.wait_time=0;
      costs.total_cost=0;
      output.cd_costs=costs;
    }
    STOP_TICKING;
    output.cd_time=boost::lexical_cast<std::string>(RUNTIME);
    return output;
  }


  cd_output cd_carpooling_test_astar(Driver car, Pedestrian foot, Manager m, const Transport::Graph * g, int iso, int step, int ratio){

    cd_output output;
    Five_cost costs;

    NodeList psi_pickup, psi_dropoff;
    BiList psi_result;



    START_TICKING;

    car.findRoad(g);


    foot.findPickup(g,iso);
    foot.findDropoff(g,iso);


    int Ppistar=0;
    int Ppiend=step;

    int Pdostar=0;
    int Pdoend=step;


    while (Ppiend <=iso){

      foot.choosePickup(Ppistar, Ppiend);
      car.choosePickup(Ppistar/ratio, Ppiend/ratio);
      psi_pickup=m.PSI_Pickup(car, foot);

      Ppistar=Ppiend;
      Ppiend+=step;

      if (psi_pickup.size()>0) break;

    }



    while (Pdoend <=iso) {

      foot.chooseDropoff(Pdostar, Pdoend);
      car.chooseDropoff(Pdostar/ratio, Pdoend/ratio);
      psi_dropoff=m.PSI_Dropoff(car, foot);

      Pdostar=Pdoend;
      Pdoend+=step;

      if (psi_dropoff.size()>0)  break;
    }

    psi_result=m.getPSI(psi_pickup, psi_dropoff);
    m.GetAllPath(g, psi_result);

    if(m.shared_path_len()>0) {


      car.getFavorites(m.shared_path);
      foot.getFavorites(m.shared_path);
      m.match(car,foot);

      output.cd_positions=cd_carpooling_positions(car,foot,m,g);
      output.cd_costs=cd_costs(car,foot,m,g);
      output.cd_pickup=m.getThePath().start;
      output.cd_dropoff=m.getThePath().end;
    }

    else{

    costs.len_foot_pickup=0;
    costs.len_car_pickup=0;
    costs.len_shared_path=0;
    costs.len_foot_dropoff=0;
    costs.len_car_dropoff=0;
    costs.wait_time=0;
    costs.total_cost=0;
    output.cd_costs=costs;

    }



    STOP_TICKING;
    output.cd_time=boost::lexical_cast<std::string>(RUNTIME);





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
    fout << "iso_foot_pickup_time"<<';'<<"pickup_foot_size"<<';'<<"iso_car_pickup_time"<<';'<<"pickup_car_size"<<';';  
    fout << "iso_foot_dropoff_time" <<';'<< "dropoff_foot_size"<<';'<<"iso_car_dropoff_time"<<';'<<"dropoff_car_size"<<';';
    fout << "psi_time" <<';'<< "psi_pickup_size"<<';'<<"psi_dropoff_size"<<';';
    fout << "path_computing_time" <<';'<< "shared_path_size"<<';'<<"path_ordering_foot_time"<<';'<<"path_ordering_car_time"<<';';
    fout << "cd_total_cost"<<';'<< "cc_total_cost"<<';'<<"ecart_cost"<<';'<<"diff_cost"<<';';
    fout << "ecart_distance_pickup (meters)"<<';'<<"ecart_distance_dropoff (meters)"<<';';
    fout << "cd_len1"<<';'<<"cd_len2"<<';'<<"cd_len3"<<';'<<"cd_len4"<<';'<<"cd_len5"<<';';
    fout << "cc_len1"<<';'<<"cc_len2"<<';'<<"cc_len3"<<';'<<"cc_len4"<<';'<<"cc_len5"<<';';
    fout<< "foot1"<<';'<<"car1"<<';'<<"foot2"<<';'<<"car2"<<';';
    fout<< "cd_waiting_time"<<';'<<"cc_waiting_time"<<';'<<"ecart_waiting_time"<<';';  
    fout << "cc_runtime"<<';';
    fout<< "cc_original_route_cost"<<';'<<"cc_ridesharing_route_cost"<<';'<<"cc_detour"<<';'; 
    fout<< "cd_original_route_cost"<<';'<<"cd_ridesharing_route_cost"<<';'<<"cd_detour"<<';'; 

    fout << "distance origin" << ';' << "distance destination" << '\n'; 

    fout.close();
    
  }

  void save_all(string fileName,Test_output output, const Transport::Graph * g ){
    
    Node pick=g->mapNode(output.cd_pickup);
    Node drop=g->mapNode(output.cd_dropoff);
    Node pick2=g->mapNode(output.cc_pickup);
    Node drop2=g->mapNode(output.cc_dropoff);
    double d_pickup=haversine_dist(pick.lon, pick.lat, pick2.lon, pick2.lat);
    double d_dropoff=haversine_dist(drop.lon, drop.lat, drop2.lon, drop2.lat);
    ofstream fout;
   
    fout.open (fileName, ofstream::out | ofstream::app);
    fout <<output.scenario_id <<';';
    fout<< output.iso_foot_pickup_time <<';'<<output.pickup_foot_size<<';'<<output.iso_car_pickup_time<<';' <<output.pickup_car_size<<';';
    fout<< output.iso_foot_dropoff_time <<';'<<output.dropoff_foot_size<<';'<<output.iso_car_dropoff_time<<';' <<output.dropoff_car_size<<';';   
    fout<< output.psi_time <<';'<<output.psi_pickup_size<<';'<<output.psi_dropoff_size<<';';   
    fout<< output.path_computing_time <<';'<<output.shared_path_size<<';'<<output.path_ordering_foot_time<<';' <<output.path_ordering_car_time<<';';
    fout<< output.cd_costs.total_cost<<';'<< output.cc_costs.total_cost <<';'<< abs(output.cd_costs.total_cost-output.cc_costs.total_cost) <<';'<<(output.cd_costs.total_cost-output.cc_costs.total_cost)<<';';  
    fout<<d_pickup<<';'<<d_dropoff<<';';
    fout << output.cd_costs.len_foot_pickup<<';'<<output.cd_costs.len_car_pickup<<';'<<output.cd_costs.len_shared_path<<';'<<output.cd_costs.len_foot_dropoff<<';'<<output.cd_costs.len_car_dropoff<<';';
    fout << output.cc_costs.len_foot_pickup<<';'<<output.cc_costs.len_car_pickup<<';'<<output.cc_costs.len_shared_path<<';'<<output.cc_costs.len_foot_dropoff<<';'<<output.cc_costs.len_car_dropoff<<';';
    fout<< output.foot_start<<';'<<output.car_start<<';'<<output.foot_end<<';'<<output.car_end<<';';
    fout<< output.cd_costs.wait_time<<';'<<output.cc_costs.wait_time<<';'<<abs(output.cd_costs.wait_time - output.cc_costs.wait_time)<<';';
    fout << output.cc_time<<';';
    fout<< output.cc_detour.original_route_cost<<';'<<output.cc_detour.ridesharing_route_cost<<';'<<output.cc_detour.detour<<';'; 
    fout<< output.cd_detour.original_route_cost<<';'<<output.cd_detour.ridesharing_route_cost<<';'<<output.cd_detour.detour<<';'; 
    fout << output.proximity.at_origin << ';' << output.proximity.at_destination << '\n';  
    fout.close();
  }


  // new experiments helper

  void create_result(string fileName){
    
    ofstream fout;
    fout.open (fileName, ofstream::out | ofstream::app);

    fout << "scenario_id" <<';';
    fout << "isochrone_radius" << ';' << "step" << ';';
    fout << "rider_origin" << ';' << "driver_origin" << ';';
    fout << "rider_destination" << ';' << "driver_destination" << ';';
    fout << "rider_destination" << ';' << "driver_destination" << ';';
    fout << "optimal_pickup" << ';' << "privacy_pickup" << ';';
    fout << "optimal_dropoff" << ';' << "privacy_dropoff" << ';';
    fout << "harversine_distance_origin" << ';' << "harversine_distance_destination" << ';';
    fout << "harversine_distance_pickup" << ';' << "harversine_distance_dropoff" << ';';
    fout << "optimal_len1"<<';'<<"optimal_len2"<<';'<<"optimal_len3"<<';'<<"optimal_len4"<<';'<<"optimal_len5"<<';';
    fout << "privacy_len1"<<';'<<"privacy_len2"<<';'<<"privacy_len3"<<';'<<"privacy_len4"<<';'<<"privacy_len5"<<';';
    fout << "rider_pickup_size" << ';' << "driver_pickup_size" << ';' << "num_steps_pickup" << ';'<< "pickup_size" << ';';
    fout << "rider_dropoff_size" << ';' << "driver_dropoff_size" << ';' << "num_steps_dropoff" << ';'<< "dropoff_size" << ';';
    fout << "shared_path_size" << ';';
    fout << "rider_origin_isochrone_time" << ';' << "driver_origin_isochrone_time" << ';';
    fout << "rider_destination_isochrone_time" << ';' << "driver_destination_isochrone_time" << ';';
    fout << "psi_pickup_time"<<';'<<"psi_dropoff_time"<<';';
    fout << "shared_path_computing_time" << ';'<<"rider_path_ordering_time"<<';'<<"driver_path_ordering_time"<<';';
    fout << "path_election_time"<<';'<<"rider_privacy_runtime"<<';';
    fout << "driver_privacy_runtime"<<';'<<"optimal_runtime"<<';';
    fout << "privacy_runtime" <<';';
    fout << "optimal_total_cost"<<';'<< "privacy_total_cost"<<';'<<"gap_cost"<<';';
    fout << "optimal_waiting_time"<<';'<< "privacy_waiting_time"<<';'<<"gap_waiting_time"<<';';
    fout<< "driver_original_trip_cost"<<';'<<"rider_original_trip_cost"<< '\n'; 


    fout.close();
    
  }

  void create_result_complement(string fileName){
    ofstream fout;
    fout.open (fileName, ofstream::out | ofstream::app);
    fout << "driver_original_trip_cost" << ';' << "rider_original_trip_cost" << '\n';
    fout.close();
  }

  void save_result_complement(string fileName, int driver_original_trip_cost, int rider_original_trip_cost){
    ofstream fout;
    fout.open (fileName, ofstream::out | ofstream::app);
    fout << driver_original_trip_cost << ';' << rider_original_trip_cost << '\n';
    fout.close();
  }

  void save_result(string fileName, Output output, const Transport::Graph * g ){
    
    Node pick=g->mapNode(output.privacy_pickup);
    Node drop=g->mapNode(output.privacy_dropoff);
    Node pick2=g->mapNode(output.optimal_pickup);
    Node drop2=g->mapNode(output.optimal_dropoff);
    Node car1 = g->mapNode(output.driver_origin);
    Node car2 = g->mapNode(output.driver_destination);
    Node foot1=g->mapNode(output.rider_origin);
    Node foot2=g->mapNode(output.rider_destination);
    double harversine_distance_pickup = haversine_dist(pick.lon, pick.lat, pick2.lon, pick2.lat);
    double harversine_distance_dropoff = haversine_dist(drop.lon, drop.lat, drop2.lon, drop2.lat);
    double harversine_distance_origin =  haversine_dist(car1.lon, car1.lat, foot1.lon, foot1.lat);
    double harversine_distance_destination = haversine_dist(car2.lon, car2.lat, foot2.lon, foot2.lat);

    output.rider_privacy_runtime = output.rider_origin_isochrone_time + output.rider_destination_isochrone_time + output.psi_pickup_time + output.psi_dropoff_time + output.rider_path_ordering_time + output.path_election_time;
    output.driver_privacy_runtime = output.driver_origin_isochrone_time + output.driver_destination_isochrone_time + output.psi_pickup_time + output.psi_dropoff_time + output.driver_path_ordering_time + output.shared_path_computing_time + output.path_election_time;
    output.optimal_total_cost = output.cc_costs.total_cost;
    output.privacy_total_cost = output.cd_costs.total_cost;

    ofstream fout;
    fout.open (fileName, ofstream::out | ofstream::app);
    fout <<output.scenario_id <<';';
    fout << output.isochrone_radius << ';' << output.step << ';';
    fout << output.rider_origin << ';' << output.driver_origin << ';';
    fout << output.rider_destination << ';' << output.driver_destination << ';';
    fout << output.rider_destination << ';' << output.driver_destination << ';';
    fout << output.optimal_pickup << ';' << output.privacy_pickup << ';';
    fout << output.optimal_dropoff << ';' << output.privacy_dropoff << ';';
    fout << harversine_distance_origin << ';' << harversine_distance_destination << ';';
    fout << harversine_distance_pickup << ';' << harversine_distance_dropoff << ';';
    fout << output.cc_costs.len_foot_pickup<<';'<<output.cc_costs.len_car_pickup<<';'<<output.cc_costs.len_shared_path<<';'<<output.cc_costs.len_foot_dropoff<<';'<<output.cc_costs.len_car_dropoff<<';';
    fout << output.cd_costs.len_foot_pickup<<';'<<output.cd_costs.len_car_pickup<<';'<<output.cd_costs.len_shared_path<<';'<<output.cd_costs.len_foot_dropoff<<';'<<output.cd_costs.len_car_dropoff<<';';
    fout << output.rider_pickup_size << ';' << output.driver_pickup_size << ';' << output.num_steps_pickup << ';'<< output.pickup_size << ';';
    fout << output.rider_dropoff_size << ';' << output.driver_dropoff_size << ';' << output.num_steps_dropoff << ';'<< output.dropoff_size << ';';
    fout << output.shared_path_size << ';';
    fout << output.rider_origin_isochrone_time << ';' << output.driver_origin_isochrone_time << ';';
    fout << output.rider_destination_isochrone_time << ';' << output.driver_destination_isochrone_time << ';';
    fout << output.psi_pickup_time <<';'<< output.psi_dropoff_time <<';';
    fout << output.shared_path_computing_time << ';'<< output.rider_path_ordering_time <<';'<< output.driver_path_ordering_time <<';';
    fout << output.path_election_time <<';'<< output.rider_privacy_runtime <<';';
    fout << output.driver_privacy_runtime <<';'<< output.optimal_runtime <<';';
    fout << max(output.driver_privacy_runtime,output.rider_privacy_runtime) <<';';
    fout << output.optimal_total_cost <<';'<< output.privacy_total_cost <<';'<< abs(output.optimal_total_cost - output.privacy_total_cost) <<';';
    fout<< output.cd_costs.wait_time<<';'<<output.cc_costs.wait_time<<';'<<abs(output.cd_costs.wait_time - output.cc_costs.wait_time)<<';';
    fout << output.driver_original_trip_cost <<';' << output.rider_original_trip_cost << '\n'; 
    fout.close();
  }
//---------- 

  void save_all_dumb(string fileName,Test_output output, const Transport::Graph * g ){
    
    /*Node pick=g->mapNode(output.cd_pickup);
    Node drop=g->mapNode(output.cd_dropoff);
    Node pick2=g->mapNode(output.cc_pickup);
    Node drop2=g->mapNode(output.cc_dropoff);
    double d_pickup=haversine_dist(pick.lon, pick.lat, pick2.lon, pick2.lat);
    double d_dropoff=haversine_dist(drop.lon, drop.lat, drop2.lon, drop2.lat);*/
    ofstream fout;
   
    fout.open (fileName, ofstream::out | ofstream::app);
    fout <<output.scenario_id <<';';
    fout<< output.iso_foot_pickup_time <<';'<<output.pickup_foot_size<<';'<<output.iso_car_pickup_time<<';' <<output.pickup_car_size<<';';
    fout<< output.iso_foot_dropoff_time <<';'<<output.dropoff_foot_size<<';'<<output.iso_car_dropoff_time<<';' <<output.dropoff_car_size<<';';   
    fout<< output.psi_time <<';'<<output.psi_pickup_size<<';'<<output.psi_dropoff_size<<';';   
    fout<< output.path_computing_time <<';'<<output.shared_path_size<<';'<<output.path_ordering_foot_time<<';' <<output.path_ordering_car_time<<';';
    fout<< output.cd_costs.total_cost<<';'<< output.cc_costs.total_cost <<';'<< abs(output.cd_costs.total_cost-output.cc_costs.total_cost) <<';'<<(output.cd_costs.total_cost-output.cc_costs.total_cost)<<';';  
    fout<< 0 <<';'<< 0 <<';';
    fout << output.cd_costs.len_foot_pickup<<';'<<output.cd_costs.len_car_pickup<<';'<<output.cd_costs.len_shared_path<<';'<<output.cd_costs.len_foot_dropoff<<';'<<output.cd_costs.len_car_dropoff<<';';
    fout << output.cc_costs.len_foot_pickup<<';'<<output.cc_costs.len_car_pickup<<';'<<output.cc_costs.len_shared_path<<';'<<output.cc_costs.len_foot_dropoff<<';'<<output.cc_costs.len_car_dropoff<<';';
    fout<< output.foot_start<<';'<<output.car_start<<';'<<output.foot_end<<';'<<output.car_end<<';';
    fout<< output.cd_costs.wait_time<<';'<<output.cc_costs.wait_time<<';'<<abs(output.cd_costs.wait_time - output.cc_costs.wait_time)<<';';
    fout << output.cc_time<<';';
    fout<< output.cc_detour.original_route_cost<<';'<<output.cc_detour.ridesharing_route_cost<<';'<<output.cc_detour.detour<<';'; 
    fout<< output.cd_detour.original_route_cost<<';'<<output.cd_detour.ridesharing_route_cost<<';'<<output.cd_detour.detour<<';'; 
    fout << output.proximity.at_origin << ';' << output.proximity.at_destination << '\n';  
    fout.close();
  }


  void create_poi_file(string fileName){
    
    ofstream fout;
    fout.open (fileName, ofstream::out | ofstream::app);
    fout << "poi_id" <<'\n';    
    fout.close();
    
  }

  void save_poi_all(string fileName,std::vector<poi> pois){
    ofstream fout;
    fout.open (fileName, ofstream::out | ofstream::app);
    BOOST_FOREACH(poi x, pois){
        fout << x.id <<'\n';
      }
    fout.close();
  }


  void create_iso_file(string fileName){
    
    ofstream fout;
    fout.open (fileName, ofstream::out | ofstream::app);
    fout << "poi_id" <<';';  
    fout << "iso_rider" <<';';   
    fout << "iso_driver" <<'\n';     
    fout.close();
    
  }

  void save_iso_all(string fileName, TwoIso iso){
    ofstream fout;
    fout.open (fileName, ofstream::out | ofstream::app);
    fout << iso.start <<';';  
    fout << iso_rider_size(iso) <<';';   
    fout << iso_driver_size(iso) <<'\n';     
    fout.close();
  }



  void create_result_file2(string fileName){
    
    ofstream fout;
   
    fout.open (fileName, ofstream::out | ofstream::app);
    
    
    fout << "cc_exact_len1"<<';'<<"cc_exact_len2"<<';'<<"cc_exact_len3"<<';'<<"cc_exact_len4"<<';'<<"cc_exact_len5"<<';';

    fout<< "cc_exact_waiting_time"<<';';  

    fout << "cc_exact_total_cost"<<';';

    fout << "cc_exact_runtime"<<';';

    fout<< "foot1"<<';'<<"car1"<<';'<<"foot2"<<';'<<"car2"<<'\n';

   
    
    fout.close();
    
  }

  void save_all2(string fileName,Test_output output){
    
    
    ofstream fout;
   
    fout.open (fileName, ofstream::out | ofstream::app);

    
    fout << output.cc_costs.len_foot_pickup<<';'<<output.cc_costs.len_car_pickup<<';'<<output.cc_costs.len_shared_path<<';'<<output.cc_costs.len_foot_dropoff<<';'<<output.cc_costs.len_car_dropoff<<';';

    fout << output.cc_costs.wait_time<<';';

    fout << output.cc_costs.total_cost <<';'; 

    fout << output.cc_time<<';';
    
    fout<< output.foot_start<<';'<<output.car_start<<';'<<output.foot_end<<';'<<output.car_end<<'\n';

    

    

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

  void exact(string filename){

    Transport::GraphFactory gf("graph.txt-dump", false);
    gf.setAll2();
    const Transport::Graph * g = gf.get();
    cc_output cc;
    Test_output output;
    
    typedef boost::tokenizer< boost::char_separator<char> > Tokenizer;
    boost::char_separator<char> sep(";");
    
    ifstream in(filename.c_str());
    if (!in.is_open()) exit(0);

    vector< string > vec;
    string line;
    int i=0;
    int car1, car2, foot1, foot2;

    create_result_file2("output_exact.csv");

    while (getline(in,line)){
      
      Tokenizer tok(line,sep);
      vec.assign(tok.begin(),tok.end());

      if(i>0){
      foot1=std::atoi(vec.at(0).c_str());
      car1=std::atoi(vec.at(1).c_str());

      cout <<"car1"<<car1<<endl;
      
      foot2=std::atoi(vec.at(2).c_str());
      car2=std::atoi(vec.at(3).c_str());

      Privacy::Driver * d1=new Privacy::Driver(car1, car2);
      Privacy::Pedestrian * p1=new Privacy::Pedestrian(foot1, foot2);
     
      cc=cc_carpooling_test(*d1,*p1,g);
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

      //id of positions
      output.car_start=d1->posStart;
      output.car_end=d1->posEnd;
      output.foot_start=p1->posStart;
      output.foot_end=p1->posEnd;

      save_all2("output_exact.csv", output);

      cout << "------------------------"<<i<<"--------------------"<<endl;

      }
      i++;
    }



  }

  void create_compare(string fileName){
    
    ofstream fout;
   
    fout.open (fileName, ofstream::out | ofstream::app);
    
    
    fout << "A-Star pick-up size"<<';'<<"Isochrone pick-up size"<< ';' << "Path size"<<'\n';

   
    
    fout.close();
    
  }


  void save_compare(string fileName,int astar, int isochrone, int path_size){
    
    
    ofstream fout;
   
    fout.open (fileName, ofstream::out | ofstream::app);

    fout<< astar<<';'<<isochrone<<';'<<path_size<<'\n';

    fout.close();
    
  }


  void create_vizu(string fileName){
    
    ofstream fout;
   
    fout.open (fileName, ofstream::out | ofstream::app);
    
    fout << "longitude"<<';'<<"latitude"<<';'<<"name"<<';'<<"color"<<'\n';
    
    fout.close();
    
  }


  void save_vizu(string fileName, int node, const Transport::Graph * g, int car1,  int car2){
    
    
    ofstream fout;
   
    fout.open (fileName, ofstream::out | ofstream::app);

    Node ax=g->mapNode(node);

    if ((node != car1)&&(node != car2))
    {
      fout << ax.lon<<';'<<ax.lat<<';'<<node<<';'<<"yellow"<<'\n';
    }

    /*if (node == car1)
    {
      fout << ax.lon<<';'<<ax.lat<<';'<<to_string(node)+"#origine"<<';'<<"red"<<'\n';
    }

    if (node == car2)
    {
      fout << ax.lon<<';'<<ax.lat<<';'<<to_string(node)+"#destination"<<';'<<"navy"<<'\n';
    }*/

    

    fout.close();
    
  }

  void save_pos(string fileName, const Transport::Graph * g, int car1,  int car2){
    
    
    ofstream fout;
   
    fout.open (fileName, ofstream::out | ofstream::app);

    Node start=g->mapNode(car1);
    Node end=g->mapNode(car2);

    fout << start.lon<<';'<<start.lat<<';'<<to_string(car1)+"#origine"<<';'<<"red"<<'\n';
    
    fout << end.lon<<';'<<end.lat<<';'<<to_string(car2)+"#destination"<<';'<<"navy"<<'\n';
    
    fout.close();
    
  }

  void generate(int number, char *fileNameg, const Transport::Graph * g){
    
    ofstream fout;
    //string fileName="/vagrant/dev/krog/yassa.csv";
   
    fout.open (fileNameg, ofstream::out | ofstream::app);

    fout << "foot1" <<';'<< "foot2" <<';'<< "car1" <<';'<<"car2"<<'\n';


    
    for (int i = 0; i < number; ++i)
    {
      
      int synchro_1=synchro(g);

      //int synchro_2=get_satellite(satellites(synchro_1, gd.synchro2_b, gd.synchro2_a, gd.g));

      /*int car_1=get_satellite(satellites_free(synchro_1, car1_b, car1_a, g));

      int foot_1=get_satellite(satellites_free(synchro_1, foot1_b, foot1_a, g));

   
      int car_2=get_satellite(satellites_free(synchro_2, car2_b, car2_a, g));

      int foot_2=get_satellite(satellites_free(synchro_2, foot2_b, foot2_a, g));

      fout << foot_1 <<';'<< foot_2 <<';'<< car_1 <<';'<< car_2 <<'\n';*/

      fout << synchro_1<<'\n';
    }
    
    
    
    fout.close();
    
  }


  void hola(int ct){
    for (int i = 0; i < ct; ++i)
    {
      cout <<"hola:"<<i<<endl;
    }
    
  }

  void astar_vs_iso(string filename){

    Transport::GraphFactory gf("graph.txt-dump", false);
    gf.setAll2();
    const Transport::Graph * g = gf.get();

    typedef boost::tokenizer< boost::char_separator<char> > Tokenizer;
    boost::char_separator<char> sep(";");
    RLC::Graph *ag=new RLC::Graph(g,dfa_car);


    
    ifstream in(filename.c_str());
    if (!in.is_open()) exit(0);

    vector< string > vec;
    string line;
    int i=0;
    int car1, car2;

    create_compare("ft.csv");


    while (getline(in,line)){
      
      Tokenizer tok(line,sep);
      vec.assign(tok.begin(),tok.end());

      if(i>0){

      create_vizu("visu2/astar/astar#"+std::to_string(i)+".csv");
      create_vizu("visu2/iso/iso#"+std::to_string(i)+".csv");

      


      int tot=0;
      car1=std::atoi(vec.at(1).c_str());

      car2=std::atoi(vec.at(3).c_str());

      save_pos("visu2/astar/astar#"+std::to_string(i)+".csv", g, car1, car2);
      save_pos("visu2/iso/iso#"+std::to_string(i)+".csv", g, car1, car2);

      Privacy::Driver * d1=new Privacy::Driver(car1, car2);

      d1->findPickup(g,600);

      BOOST_FOREACH(data d, d1->data_before){
        
        int node=d.node;
        save_vizu("visu2/iso/iso#"+std::to_string(i)+".csv", node, g, car1, car2);
        
      }

      typedef RLC::AspectStorePreds <RLC::AspectAstar <RLC::DRegLC>> Dij;

   
      Dij::ParamType p(
          RLC::DRegLCParams( ag, 0, 1 ),
          RLC::AspectAstarParams( car2 )
      );
      //int clock_start = 60*(60*9 + 5);
      int clock_start = 0;
    
      
      Dij dij( p );
      BOOST_FOREACH( int state, ag->dfa_start_states() ) {
          dij.add_source_node( RLC::Vertice(car1, state), clock_start, 0 );
      }
      while( !dij.finished() ) {

        tot++;
        RLC::Label lab = dij.treat_next();
        int ax=lab.node.first;
        save_vizu("visu2/astar/astar#"+std::to_string(i)+".csv", ax, g, car1, car2);
          
      }

      int ki=dij.get_path_to(car2).edges.size();

      int p_size=ki;
       
      

      save_compare("ft.csv", tot, d1->data_before.size(), p_size);

      cout << "------------------------"<<i<<"--------------------"<<endl;

      }
      i++;
    }

    //car1=27133;
    //car2=63291;

  }

  void astar_vs_iso_2(string filename){

    Transport::GraphFactory gf("graph.txt-dump", false);
    gf.setAll2();
    const Transport::Graph * g = gf.get();

    typedef boost::tokenizer< boost::char_separator<char> > Tokenizer;
    boost::char_separator<char> sep(";");
    RLC::Graph *ag=new RLC::Graph(g,dfa_car);


    
    ifstream in(filename.c_str());
    if (!in.is_open()) exit(0);

    vector< string > vec;
    string line;
    int i=0;
    int car1, car2;

    create_compare("astar_vs_iso_2.csv");


    while (getline(in,line)){
      
      Tokenizer tok(line,sep);
      vec.assign(tok.begin(),tok.end());

      if(i>0){

      int tot=0;
      car1=std::atoi(vec.at(1).c_str());
      car2=std::atoi(vec.at(3).c_str());


      create_vizu("visu/astar/astar#"+std::to_string(i)+".csv");
      create_vizu("visu/iso/iso#"+std::to_string(i)+".csv");

      save_pos("visu/astar/astar#"+std::to_string(i)+".csv", g, car1, car2);
      save_pos("visu/iso/iso#"+std::to_string(i)+".csv", g, car1, car2);

      Privacy::Driver * d1=new Privacy::Driver(car1, car2);

      d1->findPickup(g,600);

      BOOST_FOREACH(data d, d1->data_before){
        
        int node=d.node;
        save_vizu("visu/iso/iso#"+std::to_string(i)+".csv", node, g, car1, car2);        
      }

      //typedef RLC::AspectTarget <RLC::DRegLC> Dij;

      RLC::Landmark *h=RLC::create_car_landmark( g, car2);

      typedef RLC::AspectTargetLandmark <RLC::DRegLC> Dij;

   
      Dij::ParamType p(
          RLC::DRegLCParams( ag, 0, 1 ),
          //RLC::AspectTargetParams( car2 )
          RLC::AspectTargetLandmarkParams<>( car2, h )
      );


      //int clock_start = 60*(60*9 + 5);
      int clock_start = 0;
    
      
      Dij dij( p );
      BOOST_FOREACH( int state, ag->dfa_start_states() ) {
          dij.add_source_node( RLC::Vertice(car1, state), clock_start, 0 );
      }
      while( !dij.finished() ) {

        tot++;
        dij.treat_next();
        RLC::Label lab = dij.treat_next();
        int ax=lab.node.first;
        save_vizu("visu/astar/astar#"+std::to_string(i)+".csv", ax, g, car1, car2);
          
      }
       
      //int p_size=dij.get_path().size();

      save_compare("astar_vs_iso_2.csv", tot, d1->data_before.size() , 0);

      cout << "------------------------"<<i<<"--------------------"<<endl;

      }
      i++;
    }

    

  }
};
    
   
}
