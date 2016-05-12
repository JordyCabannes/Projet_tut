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


//lire le fichier csv

void lecture(string filename, const Transport::Graph * gr){

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
	
      if(i>1){
      structure_test test;

      test.scenario_id=std::atoi(vec.at(0).c_str());
      test.src_car=std::atoi(vec.at(1).c_str());
      test.dest_car=std::atoi(vec.at(2).c_str());
      test.car_od_cost=std::atoi(vec.at(3).c_str());
      test.src_ped=std::atoi(vec.at(4).c_str());
      test.dest_ped=std::atoi(vec.at(5).c_str());
      test.ped_od_cost=std::atoi(vec.at(6).c_str());

	// lancer ici la méthode pour carsharing avec pour parametre la structure test, et le graph gr)

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
	create_file("/vagrant/projet/kool/analysis/instances.csv");
	while (compt!= number){
        
	structure_test test;
	int src_car = rgenerator(min, max);
        int dest_car = rgenerator(min, max);
	int src_ped = rgenerator(min, max);
	int dest_ped = rgenerator(min, max);

        //std::cout<<"Valeur de origine voiture  : "<< src_car <<std::endl;
        //std::cout<<"Valeur de destination voiture  : "<< dest_car <<std::endl;
        

        int cost_car = pointToPoint(src_car, dest_car, gr, true);
	int cost_ped = pointToPoint(src_ped, dest_ped, gr, false);
	if (cost_car > 0 && cost_ped > 0) {
	test.scenario_id =compt;
	compt++;
	test.src_car = src_car;
	test.src_ped = src_ped;
	test.dest_car = dest_car;
	test.dest_ped = dest_ped;
	test.car_od_cost = cost_car;
	test.ped_od_cost = cost_ped;
	save("/vagrant/projet/kool/analysis/instances.csv", test);
	std::cout <<" ..... "<<"instance num: " << compt<< std::endl;
	}
        
	}
}

void covoiturage_simple(string filename,  structure_test test, const Transport::Graph * gr){


// appeler l'algo de covoiturage sur "test", sauvegarder dans filename
create_file(filename);
//....

}


int main(int argc, char const *argv[])
{
    Transport::GraphFactory gf("/vagrant/projet/graph.txt-dump", false);
    gf.setAll2();
    const Transport::Graph * gr = gf.get();
   // generer_instances(10, gr->num_vertices(), gr);
   lecture("/vagrant/projet/kool/analysis/instances.csv", gr);
    return 0;
}
