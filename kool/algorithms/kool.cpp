#include "RegLC/AlgoTypedefs.h"
#include "DataStructures/GraphFactory.h"
#include "MultipleParticipants/run_configurations.h"
#include "RegLC/reglc_graph.h"
#include "RegLC/DRegLC.h"
#include "MultipleParticipants/muparo.h"
#include <iostream>
#include <fstream>
#include <random>
#include <string>
#include <boost/tokenizer.hpp>
#include <boost/concept_check.hpp>
#include <boost/lexical_cast.hpp>
typedef struct
{
        int src_car;
        int dest_car;
        int src_ped;
        int dest_ped;
	int scenario_id;
	int car_od_cost;
	int ped_od_cost;

} structure_test;

typedef struct
{
        int scenario_id;
        int pick_up;
        int drop_off;
        int cost_tot;
        int cost_a1;
        int cost_a2;
        int cost_a3;
	int cost_a4;
	int cost_a5;
	int tps_exec;

} structure_result_algo;

void covoiturage_simple(string filename, structure_test test, const Transport::Graph * gr);

//utiliser pour générer les instances
int rgenerator(int min,int max){

    std::random_device rd;

    std::mt19937 eng(rd());

    std::uniform_int_distribution<int> dist(min,max);
    return dist(eng);
}

//test A*
int pointToPoint(int src, int dest, const Transport::Graph * gr, bool car){
	int cost;
	RLC::DFA dfa_car=RLC::car_dfa();
    	RLC::DFA dfa_passenger=RLC::pt_foot_dfa();
	
	if (car){
	AlgoMPR::PtToPt * c1 =  point_to_point( gr, src, dest, dfa_car);
        c1->run();
        cost = c1->get_cost(0, dest);
	}else{
	AlgoMPR::PtToPt * c1 =  point_to_point( gr, src, dest, dfa_passenger);
        c1->run();
        cost = c1->get_cost(0, dest);
	}
	
	return cost;
}


//sauvegarde
void create_file(string fileName){

    ofstream fout;

    fout.open (fileName, ofstream::out | ofstream::app);

    fout << "scenario_id" <<';';
    
    fout << "src_car"<<';'<<"dest_car"<<";"<<"car_od_cost"<<';';

    fout << "src_ped"<<';'<<"dest_ped"<<";"<<"ped_od_cost"<<'\n';

    fout.close();
  }

void save(string fileName,  structure_test test){

  
    ofstream fout;

    fout.open (fileName, ofstream::out | ofstream::app);

    fout <<test.scenario_id <<';';

    fout << test.src_car <<';'<< test.dest_car << ";" << test.car_od_cost <<';';

    fout << test.src_ped <<';'<< test.dest_ped << ";" << test.ped_od_cost <<'\n';
    
    fout.close();
    
}

void create_file_covoit(string fileName){

    ofstream fout;

    fout.open (fileName, ofstream::out | ofstream::app);

    fout << "scenario_id" <<';';

    fout << "point_pick_up"<<';'<<"point_drop_off"<<";"<<"coût_tot"<<';'<<"coût_algo_1"<<';'<<"coût_algo_2"<<';';

    fout << "coût_algo_3"<<';'<<"coût_algo_4"<<';'<<"coût_algo_5"<<';'<<"temps_execution"<<'\n';

    fout.close();
  }

void save_covoit(string fileName,  structure_result_algo test){


    ofstream fout;

    fout.open (fileName, ofstream::out | ofstream::app);

    fout <<test.scenario_id <<';';

    fout << test.pick_up <<';'<< test.drop_off << ';' << test.cost_tot <<';';

    fout << test.cost_a1 <<';'<< test.cost_a2 << ';' << test.cost_a3 <<';' ;

    fout << test.cost_a4 <<';'<< test.cost_a5 << ';' << test.tps_exec <<'\n';

    fout.close();

}

//lire le fichier csv

void lecture(string filename, const Transport::Graph * gr, int number){

    typedef boost::tokenizer< boost::char_separator<char> > Tokenizer;
    boost::char_separator<char> sep(";");

   string nom_res="/vagrant/projet/kool/analysis/resultat_bis.csv";

    create_file_covoit(nom_res);

    ifstream in(filename.c_str());

    if (!in.is_open()) exit(0);

    vector< string > vec;
    string line;
    int i=0;

    while (getline(in,line) && i<=number+1){

      Tokenizer tok(line,sep);
      vec.assign(tok.begin(),tok.end());
	//std::cout<<"il a pas dit Bonjour"<<endl;
      if(i>=1){
      structure_test test;
      std::cout<<"Aurevoir"<<endl;
      test.scenario_id=std::atoi(vec.at(0).c_str());
      test.src_car=std::atoi(vec.at(1).c_str());
      test.dest_car=std::atoi(vec.at(2).c_str());
      test.car_od_cost=std::atoi(vec.at(3).c_str());
      test.src_ped=std::atoi(vec.at(4).c_str());
      test.dest_ped=std::atoi(vec.at(5).c_str());
      test.ped_od_cost=std::atoi(vec.at(6).c_str());

	// lancer ici la méthode pour carsharing avec pour parametre la structure test, et le graph gr)
       covoiturage_simple(nom_res, test, gr); 
	std::cout << test.src_car << std::endl;
      }
      i++;
    }
}


//generer les instances
void generer_instances(int number, int nb_nodes, const Transport::Graph * gr)
{
	
	int compt = 0;
	int min =0;
        int max = nb_nodes -1;
	bool valide=false;
	create_file("/vagrant/projet/kool/analysis/instances_bis.csv");
	while (compt!= number){
        
        //std::cout<<"Valeur de destination voiture  : "<< dest_car <<std::endl;
	while(!valide)
	{
		int src_car = rgenerator(min, max);
        	int src_ped = rgenerator(min, max);
		int cost_ocar_oped = pointToPoint(src_car, src_ped, gr, true);
		if(cost_ocar_oped < 3600)
		{	
			int dest_car = rgenerator(min, max);
			int dest_ped = rgenerator(min, max);
			int cost_dcar_dped = pointToPoint(dest_car, dest_ped, gr, true);
			if(cost_dcar_dped < 3600)
			{
				int cost_car = pointToPoint(src_car, dest_car, gr, true);
        			int cost_ped = pointToPoint(src_ped, dest_ped, gr, false);		
				if (cost_car > 0 && cost_ped > 0 && cost_car > 600 && cost_ped > 600) 
				{	
					structure_test test;
					test.scenario_id =compt;
					compt++;
					test.src_car = src_car;
					test.src_ped = src_ped;
					test.dest_car = dest_car;
					test.dest_ped = dest_ped;
					test.car_od_cost = cost_car;
					test.ped_od_cost = cost_ped;
					save("/vagrant/projet/kool/analysis/instances_bis.csv", test);
					std::cout <<" ..... "<<"instance num: " << compt<< std::endl;
					valide=true;	
				}
        		}
		}
	}
	valide=false;
	}
}

void covoiturage_simple(string filename, structure_test test, const Transport::Graph * gr){


// appeler l'algo de covoiturage sur "test", sauvegarder dans filename
	structure_result_algo resultat;
    	typedef AlgoMPR::CarSharingTest CurrAlgo;

    	// std::cout<< "4" <<std::endl;
   	 START_TICKING;

    	RLC::DFA dfa_car=RLC::car_dfa();
    	RLC::DFA dfa_passenger=RLC::pt_foot_dfa();
    	//std::cout<< "5" <<std::endl;
    	//structure_test test= generer_instances(gr->num_vertices(), gr, dfa_car);

    	CurrAlgo::ParamType p(
              MuparoParams( gr, 5 ),
              AspectTargetParams( 4, test.dest_ped),
              AspectPropagationRuleParams( SumPlusWaitCost, MaxArrival, 2, 0, 1),
              AspectPropagationRuleParams( SumCost, FirstLayerArrival, 4, 2, 3));
    	//std::cout<< "6" <<std::endl;
    	CurrAlgo cs(p);
    	//std::cout<< "7" <<std::endl;
	std::cout<<"Début covoiturage"<<endl;
    	init_car_sharing<CurrAlgo>( &cs, gr, test.src_ped, test.src_car, test.dest_ped, test.dest_car, dfa_passenger, dfa_car);
	cs.run();
	std::cout<<"fin covoiturage"<<endl;
    	STOP_TICKING;

	int time=0;

    	int drop_off = cs.get_source(4, test.dest_ped);
    	int pick_up = cs.get_source(2, drop_off);

    	int len1=cs.arrival(0, pick_up)-time;
    	int len2=cs.arrival(1, pick_up)-time;
    	int len3=cs.arrival(2, drop_off) - cs.arrival(2, pick_up); //temps entre pickup et le dropoff

    	int len4=cs.get_cost(3, drop_off); //on récupère le coût entre le dropof et la destination de la voiture
    	int len5=cs.arrival(4, test.dest_ped) - cs.arrival(4, drop_off); //te
	int tps_execution=RUNTIME;

	resultat.scenario_id=test.scenario_id;
	resultat.drop_off=drop_off;
	resultat.pick_up=pick_up;
	resultat.cost_tot=len1+len2+2*len3+len4+len5;
	resultat.cost_a1=len1;
	resultat.cost_a2=len2;
	resultat.cost_a3=len3;
	resultat.cost_a4=len4;
	resultat.cost_a5=len5;
	resultat.tps_exec=tps_execution;

	save_covoit(filename, resultat);
}


int main(int argc, char const *argv[])
{
    Transport::GraphFactory gf("/vagrant/projet/graph.txt-dump", false);
    gf.setAll2();
    const Transport::Graph * gr = gf.get();
   generer_instances(4, gr->num_vertices(), gr);
   	std::cout<<"Je runne le main --------------------------"<<endl;
	lecture("/vagrant/projet/kool/analysis/instances_bis.csv", gr, 4);

    return 0;
}
