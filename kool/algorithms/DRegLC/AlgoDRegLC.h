#ifndef ALGODREGLC_H
#define ALGODREGLC_H

#include <iostream>
#include "../RegLC/reglc_graph.h"
#include "../DataStructures/GraphFactory.h"
#include "../../lib/core/graph_wrapper.h"
#include "../Interface/Path.h"
#include <map>
#include <src/rlc_explorer.hpp>
#include <fstream>
#include <src/dreglc.hpp>
using namespace std;

class algodreglc
{

 public:
	
 	dreglc _dreglc;
	
	std::map<int,RLC::DFA> mymap;

    algodreglc(){
       	mymap.insert(make_pair(1, RLC::foot_subway_dfa()));
    	mymap.insert(make_pair(2, RLC::bike_dfa()));
    	mymap.insert(make_pair(3, RLC::foot_dfa()));
    	mymap.insert(make_pair(4, RLC::car_dfa()));
    	mymap.insert(make_pair(5, RLC::pt_foot_dfa()));
    	mymap.insert(make_pair(6, RLC::pt_car_dfa()));
    	mymap.insert(make_pair(7, RLC::bike_pt_dfa()));
		mymap.insert(make_pair(8, RLC::pt_dfa()));
        mymap.insert(make_pair(9, RLC::all_dfa()));
	}

    bool init_dfa(int x){
      std::map<int,RLC::DFA>::const_iterator it;
    	it = mymap.find(x);
    	if (it==mymap.end())
    	{
    		return false;
    	}
    	else
    	{
    		_dreglc.dfa(&(it->second));
    		return true;
    	}
	}

	bool init_graph(Transport::Graph * graphe){
    	if (graphe==nullptr)
    		return false;
    	else
    	{
    		_dreglc.graph(graphe);
    		return true;
    	}
    }
	
    bool init_day(int day){
    	_dreglc.day(day);
        return (_dreglc.day()==day);
    }

    bool init_trafic(int coeff){
        _dreglc.explorer().trafic(coeff);
        return true;
    }

	bool init_mode(EdgeMode mode, bool val){
    	_dreglc.explorer().mode(mode, val);
        return (_dreglc.explorer().mode(mode)==val);
    }
	
	bool init_modes(unsigned int val)
	{
	  _dreglc.modes(val);
	  return (_dreglc.modes() == val);
	}
    
    bool solve (int origin, int dest, int cost){
    	bool test;
        //std::ofstream fs("/home/user/plateforme/muparo/build/erreur.txt");
    	//_dreglc.graph()->dumpt_pt_duration(fs);
        _dreglc.prepare();
        //std::cout << "origin = " << origin;
        //std::cout << "dest = " << dest;
        //std::cout << "cost = " << cost << std::endl;
    	test=_dreglc.solve(origin, dest, cost);
    	_dreglc.clear();

    	return test;
    }

    int costs [10] = {0};
    std::vector<Path_mode> _path_mode;

    Path read_resul(int cost){

        Path p;
        int cout_cour=cost;
        int cout_prec=cost;
        int edge_id;
        //fprintf(stderr,"read \n");

        rlc_explorer::tag_id_type first_tag_id = _dreglc.explorer().hop_id_get_target_tag_id(_dreglc._result.front());
        rlc_explorer::tag_type tag_type_cour = rlc_explorer::tag_type(first_tag_id, cost);
        
        p.end_node = first_tag_id.first;
        /*fprintf(stderr,"tag id \t");
        fprintf(stderr,"automate \t");
        fprintf(stderr,"edge id \t\t");
        fprintf(stderr,"\n");*/

        for(dreglc::result_type::const_reverse_iterator it = _dreglc.result().rbegin(); it != _dreglc.result().rend(); ++it) {
            
            rlc_explorer::hop_id_type hop_id = *it;

            typename rlc_explorer::traversal_type traversal = _dreglc.explorer().traverse_forward(tag_type_cour, hop_id);

            typename rlc_explorer::tag_type next_tag = _dreglc.explorer().hop_get_value(traversal.value());

            cout_prec = cout_cour;
            cout_cour = next_tag.cost();
            tag_type_cour = next_tag;

            edge_id = _dreglc.explorer().graph()->edgeIndex(hop_id.first);
            p.edges.push_back(edge_id);

            fprintf(stderr,"%d \t",tag_type_cour.id().first);
            //fprintf(stderr,"%d \t",tag_type_cour.id().second);
            fprintf(stderr,"%d \t\t",edge_id);
            //_dreglc.explorer().hop_id_stream(std::cerr, hop_id);
            //fprintf(stderr,"\n");

            costs[_dreglc.explorer().graph()->map(hop_id.first).type]+=(cout_cour - cout_prec);
        }

        /*p.edges.clear();
        p.edges.push_back(23087);
        p.edges.push_back(67173);
        p.edges.push_back(227855);
        p.edges.push_back(18023);*/

        p.start_node = _dreglc.explorer().hop_id_get_source_tag_id(_dreglc._result.back()).first;

        fprintf(stderr,"path \n");

        /*for(std::list<int>::iterator it = p.edges.begin(); it != p.edges.end(); ++it) {

            fprintf(stderr,"%d \t\t",*it);
            fprintf(stderr,"\n");

        }*/
            //fprintf(stderr,"fin de read resul \n");

        return p;
    }

    void read_resul_mode(int cost){

        for (int i = 0; i<10;i++){
            costs [i]=0;
        }

        Path_mode pm;
        int cout_cour=cost;
        int cout_prec=cost;
        int edge_id;
        int c=0;

        EdgeMode mode;

        rlc_explorer::hop_id_type hop_id;
        rlc_explorer::tag_id_type tag_id;
        rlc_explorer::tag_type tag_type_cour;
        rlc_explorer::tag_type next_tag;
        typename rlc_explorer::traversal_type traversal;
        //fprintf(stderr,"debut read result mode \n");

        hop_id = _dreglc._result.back();
        tag_id = _dreglc.explorer().hop_id_get_target_tag_id(hop_id);
        tag_type_cour = rlc_explorer::tag_type(tag_id, cout_cour);

        pm.edges.clear();
        mode = _dreglc.explorer().graph()->map(hop_id.first).type;
        pm.mode = mode;
        //std::cerr << mode << std::endl;
        pm.end_node = _dreglc.explorer().hop_id_get_target_tag_id(hop_id).first;
        edge_id = _dreglc.explorer().graph()->edgeIndex(hop_id.first);
        pm.edges.push_back(edge_id);


        for(dreglc::result_type::const_reverse_iterator it = _dreglc.result().rbegin(); it != _dreglc.result().rend(); ++it)
        {
            hop_id = *it;
            if (_dreglc.explorer().graph()->map(hop_id.first).type == mode)
            {
                edge_id = _dreglc.explorer().graph()->edgeIndex(hop_id.first);
                pm.edges.push_back(edge_id);
            }
            else
            {
                _path_mode.push_back(pm);
                pm.edges.clear();
                mode = _dreglc.explorer().graph()->map(hop_id.first).type;
                pm.mode = mode;
                //std::cerr << mode << std::endl;
                pm.end_node = _dreglc.explorer().hop_id_get_target_tag_id(hop_id).first;
            }

            traversal = _dreglc.explorer().traverse_forward(tag_type_cour, hop_id);

            next_tag = _dreglc.explorer().hop_get_value(traversal.value());

            cout_prec = cout_cour;
            cout_cour = next_tag.cost();
            tag_type_cour = next_tag;

            edge_id = _dreglc.explorer().graph()->edgeIndex(hop_id.first);
            pm.start_node = _dreglc.explorer().hop_id_get_source_tag_id(hop_id).first;

            costs[mode]+=(cout_cour - cout_prec);
            c += cout_cour - cout_prec;
            //std::cout << "cout_arc= " << c << " type= " << mode << " edge_id= ";
            //std::cout << edge_id << " node= " << pm.start_node << std::endl;

            pm.edges.push_back(edge_id);
        }

        _path_mode.push_back(pm);

    }
    

    int read_cost(int x){
        return costs[x];
    }

    Path_mode read_path_mode(int x){
        return _path_mode.at(x);
    }

    int read_mode(int x){
        return _path_mode.at(x).mode;
    }

    int read_path_mode_size(){
        return _path_mode.size();
    }

    void write_path_mode(){
        std::cerr << "debut du vecteur path_mode" << std::endl;
        for (uint i = 0; i < _path_mode.size() ; i++)
        {
            std::cerr << _path_mode.at(i).mode << std::endl;
        }
    }

};

#endif
