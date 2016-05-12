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
#include "opt_driver.h"
#include "opt_rider.h"
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


 

  int dlen(dataList d){

      return d.size();
  }

  int dlen2(database d){

      return d.size();
  }


};
    
   
}
