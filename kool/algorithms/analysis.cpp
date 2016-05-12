

#include "Ridesharing/privacy.h"

//#include "DRegLC/AlgoDRegLC.h"

int main(int argc, char*argv[]){
  
   
	Transport::GraphFactory gf("/vagrant/ubimob/graph.txt-dump", false);
	gf.setAll2();
	const Transport::Graph * g = gf.get();
	Privacy::Toolbox * tb=new Privacy::Toolbox();
	
	/*Transport::Graph * gg = const_cast<Transport::Graph *>(g);
  

  	

 	algodreglc *algo =new algodreglc();

 	
 	algo->init_dfa(1);
	algo->init_day(0);
	algo->init_graph(gg);
	algo->init_trafic(0);
	algo->init_modes(WhateverEdge);

	int hour = 1;
	int minute = 30;

	int clock_start = 60*(60*hour + minute);

	int solve = algo->solve(foot_1, foot_2, clock_start);

	algo->read_resul_mode(clock_start);

	int ss = algo->read_path_mode_size();




	cout << "solve: " << solve << " size: " << ss << endl;

	cout << "foot cost: " << algo->read_cost(0) << endl;
	cout << "bike cost: " << algo->read_cost(1) << endl;
	cout << "car cost: " << algo->read_cost(2) << endl;
	cout << "subway cost: " << algo->read_cost(3) << endl;
	cout << "bus cost cost: " << algo->read_cost(4) << endl;

	cout << "tramway cost: " << algo->read_cost(5) << endl;
	cout << "transfer cost: " << algo->read_cost(6) << endl;
	cout << "unknow cost: " << algo->read_cost(7) << endl;
	cout << "whatever cost: " << algo->read_cost(8) << endl;*/



	typedef boost::tokenizer< boost::char_separator<char> > Tokenizer;
    boost::char_separator<char> sep(";");
    string filename="/vagrant/ubimob/analysis/output/new_short_instances.csv";
    ifstream in(filename.c_str());
    if (!in.is_open()) exit(0);
    vector< string > vec;
    string line;
    int i=0;
    int car_1, car_2, foot_1, foot_2;
    tb->create_result_complement("/vagrant/ubimob/analysis/experiments/dec2015/short_comp.csv");


    while (getline(in,line)){
      Tokenizer tok(line,sep);
      vec.assign(tok.begin(),tok.end());
      if(i>0){
        car_1=std::atoi(vec.at(0).c_str());
        foot_1=std::atoi(vec.at(1).c_str());
        car_2=std::atoi(vec.at(2).c_str());
        foot_2=std::atoi(vec.at(3).c_str());
		int driver_original_trip_cost = tb->pointTopoint(g, car_1, car_2, true);
    	int rider_original_trip_cost = tb->pointTopoint(g, foot_1, foot_2, false);
    	cout << "driver_original_trip_cost: " << driver_original_trip_cost/60.0 << endl;
    	cout << "rider_original_trip_cost: " << rider_original_trip_cost/60.0 << endl;
    	cout << "<===============================>" << endl;
    	tb->save_result_complement("/vagrant/ubimob/analysis/experiments/dec2015/short_comp.csv", driver_original_trip_cost, rider_original_trip_cost);
		}
		i++;
	}
}