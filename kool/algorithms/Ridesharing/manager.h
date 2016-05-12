#pragma once
#include "header.h"
#include "aux_data.h"
#include "opt_driver.h"
#include "opt_rider.h"

/*
 *In charge of carpooling negociation  
 */

using namespace MuPaRo;

namespace Privacy {
  class Manager {
        
    public:
      pathList shared_path;
      prefs matching;
      costVector theSolution;

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
        //int clock_start = 60*(60*9 + 5);
        int clock_start = 0;

        int i=0;

        if (direction){   
          
          BOOST_FOREACH(int center, psi.PickUp){ 
            uint compt=0;
            typedef RLC::DRegLC Dij;    
            Dij::ParamType p(RLC::DRegLCParams( fg, 0, 1 ));
            Dij dij( p );
            
    
            
            BOOST_FOREACH( int state, fg->dfa_start_states() ){
      	dij.add_source_node( RLC::Vertice(center, state), clock_start, 0 );
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
      	dij.add_source_node( RLC::Vertice(center, state), clock_start, 0 );
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


      void GetAllPathO(const Transport::Graph * g, BiList psi){ 
            
        RLC::Graph *fg=new RLC::Graph( g,dfa_car );
        RLC::BackwardGraph *bg=new RLC::BackwardGraph(fg);
        bool direction=(psi.PickUp.size()<=psi.DropOff.size());
        //bool direction=true;
        //int clock_start = 60*(60*9 + 5);
        int clock_start = 0;
    

        int i=0;

        if (direction){   
          
          BOOST_FOREACH(int center, psi.PickUp){ 
            uint compt=0;
            typedef RLC::DRegLC Dij;    
            Dij::ParamType p(RLC::DRegLCParams( fg, 0, 1 ));
            Dij dij( p );
            
            BOOST_FOREACH( int state, fg->dfa_start_states() ){
        dij.add_source_node( RLC::Vertice(center, state), clock_start, 0 );
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
        
        if(i>10){
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
        
        if(i > 10){
          break;
        }
            } 
          }
        }
             
      }

      void GetAllPath_dumb(const Transport::Graph * g, BiList psi){ 

        cout << "------------------------------> " << psi.PickUp.size() * psi.DropOff.size() << endl;

        int i = 0;
          BOOST_FOREACH(int center_a, psi.PickUp){ 
            BOOST_FOREACH(int center_b, psi.DropOff){ 
              Node a = g->mapNode(center_a);
              Node b = g->mapNode(center_b);
              int cost = haversine_dist(a.lon, a.lat, b.lon, b.lat)/10;
              //cout << center_a << "---" << center_b << " cost: " << cost << endl;

              path  pt;
              pt.id = i;
              pt.start = center_a;
              pt.end = center_b;
              pt.cost = cost;
              shared_path.push_back(pt);
              //if(i==9) break;
              i++;
            }
        } 
        cout << "------------------------------> end haversine "  << endl;
      
      }
          
      /*
       *odrer paths such that the best path if the one who minimize the totat travel time for both driver and pedestrian
       */ 

    void ProcessTotalCost(pathList paths, Driver driver, Pedestrian rider){
      
      BOOST_FOREACH(path p, paths){

        data origin_driver = getData2(p.start, driver.data_before);
        data destination_driver = getData2(p.end, driver.data_after);

        data origin_rider = getData2(p.start, rider.data_before);
        data destination_rider = getData2(p.end, rider.data_after);


        totalCost tc;
        tc.path_id=p.id;

        /*tc.cost= origin_driver.cost + origin_rider.cost + 
                 abs(origin_driver.cost - origin_rider.cost) + 
                 2*p.cost + 
                 destination_driver.cost + destination_rider.cost;*/

        tc.cost= 2*(origin_driver.cost + origin_rider.cost) + 
                 2*p.cost + 
                 destination_driver.cost + destination_rider.cost;


        theSolution.push_back(tc);
      }
    }  

    void match(){
      theSolution.sort(compare_cost);
    }

    path getThePath_great() const{
        return getPathById(theSolution.front().path_id, shared_path);
    }


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
}
